/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos_pppos.h"

#include "common/cs_dbg.h"
#include "common/mbuf.h"
#include "common/mg_str.h"
#include "common/queue.h"

#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppapi.h"
#include "netif/ppp/pppos.h"

#include "mongoose.h"

#include "mgos_gpio.h"
#include "mgos_net_hal.h"
#include "mgos_sys_config.h"
#include "mgos_system.h"
#include "mgos_time.h"
#include "mgos_timers.h"
#include "mgos_uart.h"
#include "mgos_utils.h"

#define AT_CMD_TIMEOUT 3.0

enum mgos_pppos_state {
  PPPOS_IDLE = 0,
  PPPOS_INIT = 1,
  PPPOS_RESET = 2,
  PPPOS_RESET_HOLD = 3,
  PPPOS_RESET_WAIT = 4,
  PPPOS_BEGIN_WAIT = 5,
  PPPOS_BEGIN = 6,
  PPPOS_SETUP = 7,
  PPPOS_CMD = 8,
  PPPOS_CMD_RESP = 9,
  PPPOS_START_PPP = 10,
  PPPOS_RUN = 11,
  PPPOS_USER_CMD = 12,
  PPPOS_USER_CMD_DONE = 13,
};

struct mgos_pppos_cmd;

struct mgos_pppos_data {
  int if_instance;
  const struct mgos_config_pppos *cfg;
  enum mgos_pppos_state state;
  struct mbuf data;
  double delay, deadline;
  bool baud_ok;
  int attempt;
  mgos_timer_id poll_timer_id;

  struct mgos_pppos_cmd *cmds;
  int num_cmds;
  int cmd_idx;
  int try_baud_idx;
  int try_baud_fc;

  char *user_cmd;
  enum mgos_pppos_state interrupted_state;
  int interrupted_data_len;

  struct netif pppif;
  ppp_pcb *pppcb;
  enum mgos_net_event net_status;
  enum mgos_net_event net_status_last_reported;
  struct mg_str ati_resp, cimi_resp;

  SLIST_ENTRY(mgos_pppos_data) next;
};

typedef bool (*mgos_pppos_cmd_cb_t)(struct mgos_pppos_data *pd, bool ok,
                                    struct mg_str data);

struct mgos_pppos_cmd {
  char *cmd;
  mgos_pppos_cmd_cb_t cb;
};

static SLIST_HEAD(s_pds, mgos_pppos_data) s_pds = SLIST_HEAD_INITIALIZER(s_pds);

/* If we fail to communicate with the modem at the specified rate,
 * we will try these (with no flow control), in this order. */
static const int s_baud_rates[] = {
    0 /* first we try the configured rate */, 9600, 115200, 230400, 460800,
    921600,
};

static void mgos_pppos_try_baud_rate(struct mgos_pppos_data *pd) {
  struct mgos_uart_config ucfg;
  if (!mgos_uart_config_get(pd->cfg->uart_no, &ucfg)) return;
  if (pd->try_baud_idx == 0) {
    ucfg.baud_rate = pd->cfg->start_baud_rate;
    ucfg.rx_fc_type = ucfg.tx_fc_type =
        (pd->cfg->start_fc_enable ? MGOS_UART_FC_HW : MGOS_UART_FC_NONE);
  } else {
    ucfg.baud_rate = s_baud_rates[pd->try_baud_idx];
    ucfg.rx_fc_type = ucfg.tx_fc_type = MGOS_UART_FC_NONE;
    /* If the user specified flow control, try with FC on as well
     * (modem may already have it enabled). */
    if (pd->cfg->fc_enable && pd->try_baud_fc) {
      ucfg.rx_fc_type = ucfg.tx_fc_type = MGOS_UART_FC_HW;
    }
  }
  LOG(LL_INFO, ("Trying baud rate %d (fc %s)...", ucfg.baud_rate,
                (ucfg.rx_fc_type != MGOS_UART_FC_NONE ? "on" : "off")));
  mgos_uart_configure(pd->cfg->uart_no, &ucfg);
  mgos_uart_set_rx_enabled(pd->cfg->uart_no, true);
  pd->try_baud_idx++;
  if (pd->try_baud_idx >= (int) ARRAY_SIZE(s_baud_rates)) {
    pd->try_baud_idx = 0;
    pd->try_baud_fc ^= 1;
  }
}

static void mgos_pppos_at_cmd(int uart_no, const char *cmd) {
  LOG(LL_VERBOSE_DEBUG, (">> %s", cmd));
  mgos_uart_write(uart_no, cmd, strlen(cmd));
  if (strcmp(cmd, "+++") != 0) {
    mgos_uart_write(uart_no, "\r\n", 2);
  }
}

static void mgos_pppos_set_state(struct mgos_pppos_data *pd,
                                 enum mgos_pppos_state state) {
  LOG(LL_DEBUG,
      ("%d -> %d %lf %lf", pd->state, state, mgos_uptime(), pd->deadline));
  pd->state = state;
}

static void mgos_pppos_net_status_cb(void *arg) {
  struct mgos_pppos_data *pd = (struct mgos_pppos_data *) arg;
  mgos_net_dev_event_cb(MGOS_NET_IF_TYPE_PPP, 0, pd->net_status);
}

static void mgos_pppos_set_net_status(struct mgos_pppos_data *pd,
                                      enum mgos_net_event status) {
  pd->net_status = status;
  if (pd->net_status != pd->net_status_last_reported) {
    pd->net_status_last_reported = pd->net_status;
    mgos_invoke_cb(mgos_pppos_net_status_cb, pd, false /* from_isr */);
  }
}

static u32_t mgos_pppos_send_cb(ppp_pcb *pcb, u8_t *data, u32_t len,
                                void *ctx) {
  struct mgos_pppos_data *pd = (struct mgos_pppos_data *) ctx;
  size_t wr_av = mgos_uart_write_avail(pd->cfg->uart_no);
  if (wr_av < len) {
    /* Can't send all - don't send any. Caller does not expect partial
     * transmissions. */
    return 0;
  }
  LOG(LL_DEBUG, ("> %d (av %d)", (int) len, (int) wr_av));
  len = MIN(len, wr_av);
  len = mgos_uart_write(pd->cfg->uart_no, data, len);
  if (pd->cfg->hexdump_enable) mg_hexdumpf(stderr, data, len);
  (void) pcb;
  return len;
}

/* Note: run on the lwip's tcpip thread. */
static void mgos_pppos_status_cb(ppp_pcb *pcb, int err_code, void *arg) {
  struct mgos_pppos_data *pd = (struct mgos_pppos_data *) arg;
  switch (err_code) {
    case PPPERR_NONE: {
      ppp_set_default(pd->pppcb);
      mgos_pppos_set_net_status(pd, MGOS_NET_EV_IP_ACQUIRED);
      break;
    }
    case PPPERR_USER: {
      /* User (us) called close, do not retry. */
      ppp_free(pcb);
      break;
    }
    default: {
      LOG(LL_ERROR, ("Error %d (phase %d), reconnect", err_code, pcb->phase));
      pd->pppcb = NULL;
      /* ppp_close delivers PPPERR_USER, so proto will be freed. */
      ppp_close(pcb, 0 /* nocarrier */);
      mgos_pppos_set_state(pd, PPPOS_INIT);
      mgos_uart_schedule_dispatcher(pd->cfg->uart_no, false /* from_isr */);
      break;
    }
  }
}

static void free_cmds(struct mgos_pppos_data *pd) {
  for (int i = 0; i < pd->num_cmds; i++) {
    free(pd->cmds[i].cmd);
  }
  pd->num_cmds = 0;
  pd->cmd_idx = 0;
  free(pd->cmds);
  pd->cmds = NULL;
}

static void add_cmd(struct mgos_pppos_data *pd, mgos_pppos_cmd_cb_t cb,
                    const char *fmt, ...) {
  va_list ap;
  struct mgos_pppos_cmd *cmd = NULL;
  pd->cmds = (struct mgos_pppos_cmd *) realloc(
      pd->cmds, (pd->num_cmds + 1) * sizeof(*cmd));
  cmd = pd->cmds + pd->num_cmds;
  va_start(ap, fmt);
  mg_avprintf(&cmd->cmd, 0, fmt, ap);
  cmd->cb = cb;
  pd->num_cmds++;
  va_end(ap);
}

static bool mgos_pppos_at_cb(struct mgos_pppos_data *pd, bool ok,
                             struct mg_str data) {
  /* Some modems (Sequans) don't like +++ when there's no data session
   * and send "ERROR". Ignore it. */
  (void) pd;
  (void) ok;
  (void) data;
  return true;
}

static bool mgos_pppos_ati_cb(struct mgos_pppos_data *pd, bool ok,
                              struct mg_str data) {
  if (ok) {
    pd->ati_resp = mg_strdup(data);
  }
  return ok;
}

static bool mgos_pppos_gsn_cb(struct mgos_pppos_data *pd, bool ok,
                              struct mg_str data) {
  if (!ok) {
    data = mg_mk_str("unknown");
  }
  LOG(LL_INFO, ("%.*s, IMEI: %.*s", (int) pd->ati_resp.len, pd->ati_resp.p,
                (int) data.len, data.p));
  mgos_event_trigger(MGOS_PPPOS_GOT_IMEI, &pd->ati_resp);
  mg_strfree(&pd->ati_resp);
  return true;
}

static bool mgos_pppos_ifr_cb(struct mgos_pppos_data *pd, bool ok,
                              struct mg_str data) {
  int uart_no = pd->cfg->uart_no;
  struct mgos_uart_config ucfg;
  if (!ok) return false;
  if (!mgos_uart_config_get(uart_no, &ucfg)) return false;
  if (ucfg.baud_rate != pd->cfg->baud_rate) {
    LOG(LL_DEBUG,
        ("Switching UART%d to %d...", pd->cfg->uart_no, pd->cfg->baud_rate));
    ucfg.baud_rate = pd->cfg->baud_rate;
    ucfg.rx_fc_type = ucfg.tx_fc_type =
        (pd->cfg->fc_enable ? MGOS_UART_FC_HW : MGOS_UART_FC_NONE);
    ok = mgos_uart_configure(uart_no, &ucfg);
    if (ok) mgos_uart_set_rx_enabled(uart_no, true);
    // Some modems (SIM5320E) need a bit of time to switch baud rate.
    mgos_msleep(10);
  }
  (void) data;
  return ok;
}

static bool mgos_pppos_ifc_cb(struct mgos_pppos_data *pd, bool ok,
                              struct mg_str data) {
  int uart_no = pd->cfg->uart_no;
  struct mgos_uart_config ucfg;
  if (!ok) return false;
  /* We were able to read the response, so baud rate must be fine. */
  pd->baud_ok = true;
  /* After receiving the response, set our own FC accordingly. */
  if (!mgos_uart_config_get(uart_no, &ucfg)) return false;
  enum mgos_uart_fc_type fc_type =
      (pd->cfg->fc_enable ? MGOS_UART_FC_HW : MGOS_UART_FC_NONE);
  if (ucfg.rx_fc_type != fc_type || ucfg.tx_fc_type != fc_type) {
    ucfg.rx_fc_type = ucfg.tx_fc_type = fc_type;
    ok = mgos_uart_configure(uart_no, &ucfg);
    if (ok) mgos_uart_set_rx_enabled(uart_no, true);
  }
  (void) data;
  return ok;
}

static bool mgos_pppos_cimi_cb(struct mgos_pppos_data *pd, bool ok,
                               struct mg_str data) {
  if (ok) {
    pd->cimi_resp = mg_strdup(data);
  }
  return true;
}

static bool mgos_pppos_cpin_cb(struct mgos_pppos_data *pd, bool ok,
                               struct mg_str data) {
  if (!ok) {
    LOG(LL_ERROR, ("Error response to AT+CPIN. No SIM?"));
    return false;
  }
  if (data.len < 8) {
    LOG(LL_WARN, ("Unknown response to AT+CPIN, proceeding anyway"));
    return true;
  }
  /* +CPIN: status */
  struct mg_str st = mg_mk_str_n(data.p + 7, data.len - 7);
  if (mg_vcmp(&st, "READY") == 0) {
    struct mg_str imsi = pd->cimi_resp;
    if (imsi.len == 0) imsi = mg_mk_str("unknown");
    LOG(LL_INFO, ("SIM is ready, IMSI: %.*s", (int) imsi.len, imsi.p));
    mgos_event_trigger(MGOS_PPPOS_GOT_IMSI, &imsi);
    mg_strfree(&pd->cimi_resp);
  } else {
    LOG(LL_WARN,
        ("SIM is not ready (%.*s), proceeding anyway", (int) st.len, st.p));
    /* Proceed anyway, maybe we didn't get it right */
  }
  (void) pd;
  return true;
}

static bool mgos_pppos_creg_cb(struct mgos_pppos_data *pd, bool ok,
                               struct mg_str data) {
  if (!ok) {
    LOG(LL_WARN, ("%s response to AT+CREG, proceeding anyway", "Error"));
    return true;
  }
  int n, st;
  if (sscanf(data.p, "+CREG: %d,%d", &n, &st) != 2) {
    LOG(LL_WARN, ("%s response to AT+CREG, proceeding anyway", "Unknown"));
    return true;
  }
  ok = false;
  const char *sts = NULL;
  switch (st) {
    case 0:
      sts = "idle";
      break;
    case 1:
      sts = "home";
      ok = true;
      break;
    case 2:
      sts = "searching";
      break;
    case 3:
      sts = "denied";
      break;
    case 4:
      sts = "unknown";
      break;
    case 5:
      ok = true;
      sts = "roaming";
      break;
    default:
      sts = "???";
      break;
  }
  if (ok) {
    LOG(LL_ERROR, ("Connected to mobile network (%s)", sts));
  } else {
    LOG(LL_ERROR, ("Not connected to mobile network, status %d (%s)", st, sts));
    pd->cmd_idx--;
    pd->delay = mgos_uptime() + 1.0;
    ok = true;
  }
  return ok;
}

static bool mgos_pppos_cops_cb(struct mgos_pppos_data *pd, bool ok,
                               struct mg_str data) {
  if (!ok) return true;
  const char *q1 = mg_strchr(data, '"');
  if (q1 == NULL) return true;
  const char *q2 =
      mg_strchr(mg_mk_str_n(q1 + 1, data.len - (q1 - data.p) - 1), '"');
  if (q2 == NULL) return true;
  LOG(LL_INFO, ("Operator: %.*s", (int) (q2 - q1 - 1), q1 + 1));
  (void) pd;
  return true;
}

static bool mgos_pppos_csq_cb(struct mgos_pppos_data *pd, bool ok,
                              struct mg_str data) {
  if (!ok) return true;
  int sq, ber;
  if (sscanf(data.p, "+CSQ: %d,%d", &sq, &ber) != 2) return true;
  if (sq < 0 || sq > 32) return true;
  LOG(LL_INFO, ("RSSI: %d", (-113 + sq * 2)));
  (void) pd;
  return true;
}

struct ppp_set_auth_arg {
  u8_t authtype;
  const char *user;
  const char *passwd;
};

static err_t pppapi_do_ppp_set_auth(struct tcpip_api_call_data *m) {
  struct pppapi_msg *msg = (struct pppapi_msg *) m;
  struct ppp_set_auth_arg *aa =
      (struct ppp_set_auth_arg *) msg->msg.msg.ioctl.arg;
  ppp_set_auth(msg->msg.ppp, aa->authtype, aa->user, aa->passwd);
  return ERR_OK;
}

static void pppos_set_auth(ppp_pcb *pcb, u8_t authtype, const char *user,
                           const char *passwd) {
  struct ppp_set_auth_arg auth_arg = {
      .authtype = authtype, .user = user, .passwd = passwd,
  };
  struct pppapi_msg msg = {
      .msg.ppp = pcb, .msg.msg.ioctl.arg = &auth_arg,
  };
  tcpip_api_call(pppapi_do_ppp_set_auth, &msg.call);
}

static void mgos_pppos_poll_timer_cb(void *arg);

static void mgos_pppos_dispatch_once(struct mgos_pppos_data *pd) {
  int uart_no = pd->cfg->uart_no;
  switch (pd->state) {
    case PPPOS_IDLE: {
      if (pd->data.len > 0) mbuf_clear(&pd->data);
      if (pd->poll_timer_id != MGOS_INVALID_TIMER_ID) {
        mgos_clear_timer(pd->poll_timer_id);
        pd->poll_timer_id = MGOS_INVALID_TIMER_ID;
      }
      mgos_pppos_set_net_status(pd, MGOS_NET_EV_DISCONNECTED);
      break;
    }
    case PPPOS_INIT: {
      if (pd->cfg->apn == NULL) {
        LOG(LL_ERROR, ("APN is not set"));
        mgos_pppos_set_state(pd, PPPOS_IDLE);
        break;
      }
      mgos_pppos_set_net_status(pd, MGOS_NET_EV_DISCONNECTED);
      mbuf_free(&pd->data);
      mbuf_init(&pd->data, 0);
      mg_strfree(&pd->ati_resp);
      mg_strfree(&pd->cimi_resp);

      pd->attempt++;
      pd->delay = 0;
      pd->deadline = 0;
      pd->pppcb = NULL;
      memset(&pd->pppif, 0, sizeof(pd->pppif));
      pd->net_status = MGOS_NET_EV_DISCONNECTED;
      pd->net_status_last_reported = MGOS_NET_EV_DISCONNECTED;
      free_cmds(pd);
      if (!pd->baud_ok) mgos_pppos_try_baud_rate(pd);
      if (pd->poll_timer_id == MGOS_INVALID_TIMER_ID) {
        pd->poll_timer_id = mgos_set_timer(100, MGOS_TIMER_REPEAT,
                                           mgos_pppos_poll_timer_cb, pd);
      }
      if (pd->cfg->rst_gpio >= 0 &&
          (pd->attempt == 1 || pd->cfg->rst_mode == 1)) {
        pd->baud_ok = false;
        pd->try_baud_idx = 0;
        pd->try_baud_fc = 0;
        mgos_pppos_set_state(pd, PPPOS_RESET);
      } else {
        mgos_pppos_set_state(pd, PPPOS_BEGIN_WAIT);
      }
      break;
    }
    case PPPOS_RESET: {
      const double now = mgos_uptime();
      mgos_gpio_setup_output(pd->cfg->rst_gpio, pd->cfg->rst_act);
      pd->deadline = now + (pd->cfg->rst_hold_ms / 1000.0);
      LOG(LL_INFO, ("Resetting modem..."));
      mgos_pppos_set_state(pd, PPPOS_RESET_HOLD);
      break;
    }
    case PPPOS_RESET_HOLD: {
      const double now = mgos_uptime();
      if (now < pd->deadline) break;
      mgos_gpio_write(pd->cfg->rst_gpio, !pd->cfg->rst_act);
      pd->deadline = now + (pd->cfg->rst_wait_ms / 1000.0);
      mgos_pppos_set_state(pd, PPPOS_RESET_WAIT);
      break;
    }
    case PPPOS_RESET_WAIT: {
      const double now = mgos_uptime();
      if (now < pd->deadline) break;
      /* Note: not setting deadline to 0 so BEGIN goes straight to sending
       * commands. */
      mgos_pppos_set_state(pd, PPPOS_BEGIN);
      break;
    }
    case PPPOS_BEGIN_WAIT: {
      const double now = mgos_uptime();
      if (pd->deadline == 0) {
        /* Initial 1s pause before sending +++ */
        pd->deadline = now + 1.2;
      } else if (now > pd->deadline) {
        mgos_pppos_set_state(pd, PPPOS_BEGIN);
        pd->deadline = 0;
      }
      break;
    }
    case PPPOS_BEGIN: {
      const double now = mgos_uptime();
      if (pd->deadline == 0) {
        mgos_pppos_at_cmd(uart_no, "+++");
        /* At least 1s after +++ */
        pd->deadline = now + 1.2;
      } else if (now > pd->deadline) {
        pd->deadline = 0;
        if (pd->user_cmd) {
          mgos_pppos_set_state(pd, PPPOS_USER_CMD);
        } else {
          mgos_pppos_set_state(pd, PPPOS_SETUP);
        }
      }
      break;
    }

    case PPPOS_SETUP: {
      const char *apn = pd->cfg->apn;
      mgos_pppos_set_net_status(pd, MGOS_NET_EV_CONNECTING);
      LOG(LL_INFO, ("Connecting (UART%d, APN '%s')...", pd->cfg->uart_no,
                    (apn ? apn : "")));
      mbuf_remove(&pd->data, pd->data.len);
      add_cmd(pd, mgos_pppos_at_cb, "AT");
      add_cmd(pd, NULL, "ATH");
      add_cmd(pd, NULL, "ATZ");
      add_cmd(pd, NULL, "ATE0");
      add_cmd(pd, mgos_pppos_ati_cb, "ATI");
      if (!pd->baud_ok) {
        add_cmd(pd, NULL, "AT+IPR?");
        add_cmd(pd, NULL, "AT+IFC?");
        add_cmd(pd, mgos_pppos_ifr_cb, "AT+IPR=%d", pd->cfg->baud_rate);
        int ifc = (pd->cfg->fc_enable ? 2 : 0);
        add_cmd(pd, mgos_pppos_ifc_cb, "AT+IFC=%d,%d", ifc, ifc);
      }
      add_cmd(pd, mgos_pppos_gsn_cb, "AT+GSN");
      add_cmd(pd, mgos_pppos_cimi_cb, "AT+CIMI");
      add_cmd(pd, mgos_pppos_cpin_cb, "AT+CPIN?");
      add_cmd(pd, NULL, "AT+CFUN=1"); /* Full functionality */
      add_cmd(pd, mgos_pppos_creg_cb, "AT+CREG?");
      add_cmd(pd, mgos_pppos_at_cb, "AT+COPS=3,0");
      add_cmd(pd, mgos_pppos_cops_cb, "AT+COPS?");
      add_cmd(pd, mgos_pppos_csq_cb, "AT+CSQ");
      add_cmd(pd, NULL, "AT+CREG=0"); /* Disable unsolicited reports */
      add_cmd(pd, NULL, "AT+CGDCONT=1,\"IP\",\"%s\"", pd->cfg->apn);
      add_cmd(pd, NULL, "ATDT*99***1#");
      mgos_pppos_set_state(pd, PPPOS_CMD);
      break;
    }

    case PPPOS_USER_CMD: {
      const char *eol;
      const double now = mgos_uptime();
      struct mg_str s = mg_mk_str_n(pd->data.buf + pd->interrupted_data_len,
                                    pd->data.len - pd->interrupted_data_len);
      if (pd->deadline == 0) {
        mgos_pppos_at_cmd(uart_no, pd->user_cmd);
        pd->deadline = now + AT_CMD_TIMEOUT;
        pd->interrupted_data_len = pd->data.len;
      } else if (now > pd->deadline) {
        LOG(LL_INFO, ("Command timed out: %s", pd->user_cmd));
        free(pd->user_cmd);
        pd->user_cmd = NULL;
        mgos_pppos_set_state(pd, pd->interrupted_state);
      } else if ((eol = mg_strchr(s, '\r')) != NULL) {
        /* Trigger event */
        struct mgos_pppos_cmd_resp data = {
            .iface = pd->if_instance,
            .request = mg_mk_str(pd->user_cmd),
            .response = s,
        };
        mgos_event_trigger(MGOS_PPPOS_CMD_RESP, &data);
        free(pd->user_cmd);
        pd->user_cmd = NULL;
        pd->data.len = pd->interrupted_data_len;
        mgos_pppos_at_cmd(uart_no, "ATO");
        mgos_pppos_set_state(pd, PPPOS_USER_CMD_DONE);
        pd->deadline = now + AT_CMD_TIMEOUT;
      }
      break;
    }
    case PPPOS_USER_CMD_DONE: {
      const double now = mgos_uptime();
      mgos_pppos_set_state(pd, pd->interrupted_state);
      struct mg_str s = mg_mk_str_n(pd->data.buf + pd->interrupted_data_len,
                                    pd->data.len - pd->interrupted_data_len);
      if (mg_strchr(s, '\r') == NULL && now < pd->deadline) break;
      mgos_pppos_set_state(pd, pd->interrupted_state);
      pd->data.len = pd->interrupted_data_len;
      pd->deadline = 0;
      break;
    }
    case PPPOS_CMD: {
      const double now = mgos_uptime();
      struct mgos_pppos_cmd *cur_cmd = &pd->cmds[pd->cmd_idx];
      if (now < pd->delay) break;
      mgos_pppos_at_cmd(uart_no, cur_cmd->cmd);
      pd->deadline = now + AT_CMD_TIMEOUT;
      mbuf_clear(&pd->data);
      mgos_pppos_set_state(pd, PPPOS_CMD_RESP);
      break;
    }
    case PPPOS_CMD_RESP: {
      const double now = mgos_uptime();
      struct mgos_pppos_cmd *cur_cmd = &pd->cmds[pd->cmd_idx];
      if (now > pd->deadline) {
        LOG(LL_INFO, ("Command timed out: %s", cur_cmd->cmd));
        if (cur_cmd->cb != NULL) cur_cmd->cb(pd, false, mg_mk_str(NULL));
        mgos_pppos_set_state(pd, PPPOS_INIT);
        pd->deadline = 0;
        break;
      }
      bool cmd_done = false, cmd_res = false;
      /* Consume data line by line */
      struct mg_str s = mg_mk_str_n(pd->data.buf, pd->data.len);
      struct mg_str sd = mg_mk_str_n(pd->data.buf, 0);
      const char *eol;
      while ((eol = mg_strchr(s, '\r')) != NULL) {
        struct mg_str l = mg_mk_str_n(s.p, eol - s.p);
        s.len -= (l.len + 1);
        s.p += (l.len + 1);
        l = mg_strstrip(l);
        if (l.len == 0) continue;
        if (mg_vcmp(&l, "OK") == 0 ||
            mg_strstr(l, mg_mk_str("CONNECT")) == l.p) {
          cmd_done = true;
          if (cur_cmd->cb != NULL) {
            cmd_res = cur_cmd->cb(pd, true, mg_strstrip(sd));
            break;
          } else {
            cmd_res = true;
          }
        } else if (mg_vcmp(&l, "ERROR") == 0 ||
                   mg_vcmp(&l, "NO CARRIER") == 0) {
          cmd_done = true;
          if (cur_cmd->cb != NULL) {
            cmd_res = cur_cmd->cb(pd, false, mg_strstrip(sd));
          } else {
            LOG(LL_ERROR, ("Command failed: %s", cur_cmd->cmd));
            cmd_res = false;
          }
          break;
        } else {
          sd.len = eol - pd->data.buf;
        }
      }
      if (cmd_done) {
        pd->deadline = 0;
        if (cmd_res) {
          pd->cmd_idx++;
          if (pd->cmd_idx >= pd->num_cmds) {
            free_cmds(pd);
            mgos_pppos_set_net_status(pd, MGOS_NET_EV_CONNECTED);
            mgos_pppos_set_state(pd, PPPOS_START_PPP);
            mbuf_clear(&pd->data);
          } else {
            mgos_pppos_set_state(pd, PPPOS_CMD);
          }
        } else {
          /* Retry everything */
          mgos_pppos_set_state(pd, PPPOS_INIT);
        }
      }
      break;
    }
    case PPPOS_START_PPP: {
      const char *user = pd->cfg->user;
      LOG(LL_INFO, ("Starting PPP, user '%s'", (user ? user : "")));
      mbuf_remove(&pd->data, pd->data.len);
      pd->pppcb = pppapi_pppos_create(&pd->pppif, mgos_pppos_send_cb,
                                      mgos_pppos_status_cb, pd);
      if (pd->pppcb == NULL) {
        LOG(LL_ERROR, ("pppapi_pppos_create failed"));
        mgos_pppos_set_state(pd, PPPOS_INIT);
        break;
      }
      if (user != NULL) {
        pppos_set_auth(pd->pppcb, PPPAUTHTYPE_PAP, user, pd->cfg->pass);
      }
      pd->pppcb->settings.lcp_echo_interval = pd->cfg->echo_interval;
      pd->pppcb->settings.lcp_echo_fails = pd->cfg->echo_fails;
      err_t err = pppapi_connect(pd->pppcb, 0 /* holdoff */);
      if (err != ERR_OK) {
        LOG(LL_ERROR, ("pppapi_connect failed: %d", err));
        pppapi_free(pd->pppcb);
        mgos_pppos_set_state(pd, PPPOS_INIT);
      }
      mgos_pppos_set_state(pd, PPPOS_RUN);
      break;
    }
    case PPPOS_RUN: {
      // We don't need polling anymore.
      if (pd->poll_timer_id != MGOS_INVALID_TIMER_ID) {
        mgos_clear_timer(pd->poll_timer_id);
        pd->poll_timer_id = MGOS_INVALID_TIMER_ID;
      }
      if (pd->data.len > 0) {
        pppos_input_tcpip(pd->pppcb, (u8_t *) pd->data.buf, pd->data.len);
        mbuf_remove(&pd->data, pd->data.len);
      }
      break;
    }
  }
}

static void mgos_pppos_dispatch(struct mgos_pppos_data *pd) {
  do {
    mgos_pppos_dispatch_once(pd);
  } while (pd->state == PPPOS_INIT && pd->deadline == 0);
}

static void mgos_pppos_poll_timer_cb(void *arg) {
  struct mgos_pppos_data *pd = (struct mgos_pppos_data *) arg;
  ;
  mgos_pppos_dispatch(pd);
}

static void mgos_pppos_uart_dispatcher(int uart_no, void *arg) {
  struct mgos_pppos_data *pd = (struct mgos_pppos_data *) arg;
  size_t rx_av = mgos_uart_read_avail(uart_no);
  if (rx_av > 0) {
    size_t nr = mgos_uart_read_mbuf(uart_no, &pd->data, rx_av);
    if (nr > 0) {
      LOG(LL_DEBUG, ("< %d", (int) nr));
      if (pd->cfg->hexdump_enable) {
        mg_hexdumpf(stderr, pd->data.buf, pd->data.len);
      } else if (pd->state != PPPOS_RUN) {
        LOG(LL_VERBOSE_DEBUG,
            ("<< %.*s", (int) nr, pd->data.buf + (pd->data.len - nr)));
      }
    }
  }
  mgos_pppos_dispatch(pd);
}

bool mgos_pppos_dev_get_ip_info(int if_instance,
                                struct mgos_net_ip_info *ip_info) {
  memset(ip_info, 0, sizeof(*ip_info));
  struct mgos_pppos_data *pd;
  SLIST_FOREACH(pd, &s_pds, next) {
    if (pd->if_instance == if_instance) {
      if (pd->pppif.flags & NETIF_FLAG_UP) {
        ip_info->ip.sin_addr.s_addr = ip_addr_get_ip4_u32(&pd->pppif.ip_addr);
        ip_info->netmask.sin_addr.s_addr =
            ip_addr_get_ip4_u32(&pd->pppif.netmask);
        ip_info->gw.sin_addr.s_addr = ip_addr_get_ip4_u32(&pd->pppif.gw);
      }
      return true;
    }
  }
  return false;
}

bool mgos_pppos_send_cmd(int iface, const char *req) {
  struct mgos_pppos_data *pd;
  SLIST_FOREACH(pd, &s_pds, next) {
    if (pd->if_instance == iface && pd->user_cmd == NULL) {
      mg_asprintf(&pd->user_cmd, 0, "%s", req);
      if (pd->state == PPPOS_RUN) {
        pd->interrupted_state = PPPOS_RUN;
        pd->state = PPPOS_BEGIN_WAIT;
      } else {
        pd->interrupted_state = PPPOS_INIT;
        pd->state = PPPOS_INIT;
      }
      mgos_uart_schedule_dispatcher(pd->cfg->uart_no, false);
      return true;
    }
  }
  return false;
}

bool mgos_pppos_connect(int if_instance) {
  struct mgos_pppos_data *pd;
  SLIST_FOREACH(pd, &s_pds, next) {
    if (pd->if_instance != if_instance) continue;
    if (pd->state != PPPOS_IDLE) return false;
    mgos_pppos_set_state(pd, PPPOS_INIT);
    mgos_pppos_dispatch(pd);
    return true;
  }
  return false;
}

bool mgos_pppos_disconnect(int if_instance) {
  struct mgos_pppos_data *pd;
  SLIST_FOREACH(pd, &s_pds, next) {
    if (pd->if_instance != if_instance) continue;
    if (pd->state == PPPOS_IDLE) return true;
    if (pd->pppcb != NULL) {
      ppp_pcb *pppcb = pd->pppcb;
      pd->pppcb = NULL;
      pppapi_close(pppcb, 0 /* no_carrier */);
    }
    mgos_pppos_set_state(pd, PPPOS_IDLE);
    mgos_pppos_dispatch(pd);
    return true;
  }
  return false;
}

struct mg_str mgos_pppos_get_imsi(int if_instance) {
  struct mgos_pppos_data *pd;
  SLIST_FOREACH(pd, &s_pds, next) {
    if (pd->if_instance == if_instance) return mg_strdup(pd->cimi_resp);
  }
  return mg_mk_str_n(NULL, 0);
}

struct mg_str mgos_pppos_get_imei(int if_instance) {
  struct mgos_pppos_data *pd;
  SLIST_FOREACH(pd, &s_pds, next) {
    if (pd->if_instance == if_instance) return mg_strdup(pd->ati_resp);
  }
  return mg_mk_str_n(NULL, 0);
}

bool mgos_pppos_create(const struct mgos_config_pppos *cfg, int if_instance) {
  struct mgos_uart_config ucfg;
  mgos_uart_config_set_defaults(cfg->uart_no, &ucfg);
  ucfg.baud_rate = cfg->start_baud_rate;
  ucfg.rx_buf_size = 1500;
  /* TX buffer must be greater than PPP interface MTU. */
  ucfg.tx_buf_size = 1500;
  ucfg.rx_fc_type = ucfg.tx_fc_type =
      (cfg->start_fc_enable ? MGOS_UART_FC_HW : MGOS_UART_FC_NONE);
#if CS_PLATFORM == CS_P_ESP32
  if (mgos_sys_config_get_pppos_rx_gpio() >= 0) {
    ucfg.dev.rx_gpio = mgos_sys_config_get_pppos_rx_gpio();
  }
  if (mgos_sys_config_get_pppos_tx_gpio() >= 0) {
    ucfg.dev.tx_gpio = mgos_sys_config_get_pppos_tx_gpio();
  }
  if (mgos_sys_config_get_pppos_cts_gpio() >= 0) {
    ucfg.dev.cts_gpio = mgos_sys_config_get_pppos_cts_gpio();
  }
  if (mgos_sys_config_get_pppos_rts_gpio() >= 0) {
    ucfg.dev.rts_gpio = mgos_sys_config_get_pppos_rts_gpio();
  }
  char b1[8], b2[8], b3[8], b4[8];
  LOG(LL_INFO, ("PPPoS UART%d (RX:%s TX:%s CTS:%s RTS:%s), %d, fc %s, APN '%s'",
                cfg->uart_no, mgos_gpio_str(ucfg.dev.rx_gpio, b1),
                mgos_gpio_str(ucfg.dev.tx_gpio, b2),
                mgos_gpio_str(ucfg.dev.cts_gpio, b3),
                mgos_gpio_str(ucfg.dev.rts_gpio, b4), ucfg.baud_rate,
                cfg->fc_enable ? "on" : "off", (cfg->apn ? cfg->apn : "")));
#else
#if CS_PLATFORM == CS_P_STM32
  if (mgos_sys_config_get_pppos_rx_gpio() >= 0) {
    ucfg.dev.pins.rx = mgos_sys_config_get_pppos_rx_gpio();
  }
  if (mgos_sys_config_get_pppos_tx_gpio() >= 0) {
    ucfg.dev.pins.tx = mgos_sys_config_get_pppos_tx_gpio();
  }
  if (mgos_sys_config_get_pppos_cts_gpio() >= 0) {
    ucfg.dev.pins.cts = mgos_sys_config_get_pppos_cts_gpio();
  }
  if (mgos_sys_config_get_pppos_rts_gpio() >= 0) {
    ucfg.dev.pins.rts = mgos_sys_config_get_pppos_rts_gpio();
  }
  char b1[8], b2[8], b3[8], b4[8];
  LOG(LL_INFO, ("PPPoS UART%d (RX:%s TX:%s CTS:%s RTS:%s), %d, fc %s, APN '%s'",
                cfg->uart_no, mgos_gpio_str(ucfg.dev.pins.rx, b1),
                mgos_gpio_str(ucfg.dev.pins.tx, b2),
                mgos_gpio_str(ucfg.dev.pins.cts, b3),
                mgos_gpio_str(ucfg.dev.pins.rts, b4), ucfg.baud_rate,
                cfg->fc_enable ? "on" : "off", (cfg->apn ? cfg->apn : "")));
#else
  if (mgos_sys_config_get_pppos_rx_gpio() >= 0 ||
      mgos_sys_config_get_pppos_tx_gpio() >= 0 ||
      mgos_sys_config_get_pppos_cts_gpio() >= 0 ||
      mgos_sys_config_get_pppos_rts_gpio() >= 0) {
    LOG(LL_ERROR, ("Setting UART pins is not supported on this platform"));
    return false;
  }
  LOG(LL_INFO,
      ("PPPoS UART%d %d, fc %s, APN '%s'", cfg->uart_no, ucfg.baud_rate,
       cfg->fc_enable ? "on" : "off", (cfg->apn ? cfg->apn : "")));
#endif
#endif
  if (!mgos_uart_configure(cfg->uart_no, &ucfg)) {
    LOG(LL_ERROR, ("Failed to configure UART%d", cfg->uart_no));
    return false;
  }
  struct mgos_pppos_data *pd =
      (struct mgos_pppos_data *) calloc(1, sizeof(*pd));
  pd->cfg = cfg;
  pd->if_instance = if_instance;
  pd->state = PPPOS_IDLE;
  SLIST_INSERT_HEAD(&s_pds, pd, next);
  mgos_uart_set_dispatcher(cfg->uart_no, mgos_pppos_uart_dispatcher, pd);
  mgos_uart_set_rx_enabled(cfg->uart_no, true);
  return true;
}

bool mgos_pppos_init(void) {
  mgos_event_register_base(MGOS_PPPOS_BASE, "pppos");
  if (!mgos_sys_config_get_pppos_enable()) return true;
  if (!mgos_pppos_create(&mgos_sys_config.pppos, 0 /* if_instance */)) {
    return false;
  }
  if (mgos_sys_config_get_pppos_connect_on_startup()) {
    return mgos_pppos_connect(0 /* if_instance */);
  }
  return true;
}
