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

#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "netif/ppp/pppapi.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppos.h"

#include "common/cs_dbg.h"
#include "common/mbuf.h"
#include "common/mg_str.h"
#include "common/queue.h"

#include "mongoose.h"

#include "mgos_net_hal.h"
#include "mgos_sys_config.h"
#include "mgos_system.h"
#include "mgos_uart.h"
#include "mgos_utils.h"

#define AT_CMD_TIMEOUT 3.0

enum mgos_pppos_state {
  PPPOS_START = 0,
  PPPOS_ENTER_WAIT,
  PPPOS_ENTER,
  PPPOS_SETUP,
  PPPOS_START_PPP,
  PPPOS_RUN,
  PPPOS_CLOSE,
};

struct mgos_pppos_data {
  int if_instance;
  struct mgos_config_pppos cfg;
  enum mgos_pppos_state state;
  struct mbuf data;
  double deadline;

  char **cmds;
  int num_cmds;
  int cmd_idx;

  struct netif pppif;
  ppp_pcb *pppcb;
  enum mgos_net_event net_status;
  enum mgos_net_event net_status_last_reported;

  SLIST_ENTRY(mgos_pppos_data) next;
};

static SLIST_HEAD(s_pds, mgos_pppos_data) s_pds = SLIST_HEAD_INITIALIZER(s_pds);

static void mgos_pppos_at_cmd(int uart_no, const char *cmd) {
  LOG(LL_DEBUG, (">> %s", cmd));
  mgos_uart_write(uart_no, cmd, strlen(cmd));
  if (strcmp(cmd, "+++") != 0) mgos_uart_write(uart_no, "\r\n", 2);
}

static void mgos_pppos_set_state(struct mgos_pppos_data *pd,
                                 enum mgos_pppos_state state) {
  LOG(LL_DEBUG, ("%d -> %d", pd->state, state));
  pd->state = state;
  pd->deadline = 0;
}

static void mgos_pppos_net_status_cb(void *arg) {
  struct mgos_pppos_data *pd = (struct mgos_pppos_data *) arg;
  pd->net_status_last_reported = pd->net_status;
  mgos_net_dev_event_cb(MGOS_NET_IF_TYPE_PPP, 0, pd->net_status);
}

static void mgos_pppos_set_net_status(struct mgos_pppos_data *pd,
                                      enum mgos_net_event status) {
  LOG(LL_DEBUG, ("%d -> %d", pd->net_status, status));
  pd->net_status = status;
  if (pd->net_status != pd->net_status_last_reported) {
    mgos_invoke_cb(mgos_pppos_net_status_cb, pd, false /* from_isr */);
  }
}

static u32_t mgos_pppos_send_cb(ppp_pcb *pcb, u8_t *data, u32_t len,
                                void *ctx) {
  struct mgos_pppos_data *pd = (struct mgos_pppos_data *) ctx;
  size_t wr_av = mgos_uart_write_avail(pd->cfg.uart_no);
  LOG(LL_DEBUG, (">> %d (av %d)", (int) len, (int) wr_av));
  len = MIN(len, wr_av);
  len = mgos_uart_write(pd->cfg.uart_no, data, len);
  if (pd->cfg.hexdump_enable) mg_hexdumpf(stderr, data, len);
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
      /* User called close, do not retry. */
      pd->pppcb = NULL;
      ppp_free(pcb);
      break;
    }
    default: {
      LOG(LL_ERROR, ("Error %d (phase %d), reconnect", err_code, pcb->phase));
      /* ppp_close delivers PPPERR_USER, so proto will be freed. */
      ppp_close(pcb, 0 /* nocarrier */);
      assert(pd->pppcb == NULL);
      mgos_pppos_set_state(pd, PPPOS_START);
      mgos_uart_schedule_dispatcher(pd->cfg.uart_no, false /* from_isr */);
      break;
    }
  }
}

static void free_cmds(struct mgos_pppos_data *pd) {
  for (int i = 0; i < pd->num_cmds; i++) {
    free(pd->cmds[i]);
  }
  pd->num_cmds = 0;
  free(pd->cmds);
  pd->cmds = NULL;
}

static void prepare_cmds(struct mgos_pppos_data *pd) {
  free_cmds(pd);
  pd->cmds = (char **) calloc(10, sizeof(char *));
  int n = 0;
  mg_asprintf(&pd->cmds[n++], 0, "ATH");
  mg_asprintf(&pd->cmds[n++], 0, "ATZ");
  mg_asprintf(&pd->cmds[n++], 0, "ATE0");
  mg_asprintf(&pd->cmds[n++], 0, "ATI");
  if (pd->cfg.apn != NULL) {
    mg_asprintf(&pd->cmds[n++], 0, "AT+CGDCONT=1,\"IP\",\"%s\"", pd->cfg.apn);
  }
  mg_asprintf(&pd->cmds[n++], 0, "%s", pd->cfg.connect_cmd);
  pd->num_cmds = n;
  pd->cmd_idx = 0;
}

static void mgos_pppos_uart_dispatcher(int uart_no, void *arg) {
  struct mgos_pppos_data *pd = (struct mgos_pppos_data *) arg;
  size_t rx_av = mgos_uart_read_avail(uart_no);
  const double now = mg_time();
  if (rx_av > 0) {
    size_t nr = mgos_uart_read_mbuf(uart_no, &pd->data, rx_av);
    if (nr > 0) {
      LOG(LL_DEBUG, ("<< %d", (int) nr));
      if (pd->cfg.hexdump_enable) {
        mg_hexdumpf(stderr, pd->data.buf, pd->data.len);
      }
    }
  }
  switch (pd->state) {
    case PPPOS_START: {
      const char *apn = pd->cfg.apn;
      mgos_pppos_set_net_status(pd, MGOS_NET_EV_DISCONNECTED);
      LOG(LL_INFO, ("Connecting (UART%d, APN '%s')...", pd->cfg.uart_no,
                    (apn ? apn : "")));
      mbuf_free(&pd->data);
      mbuf_init(&pd->data, 0);
      prepare_cmds(pd);
      pd->deadline = 0;
      pd->pppcb = NULL;
      memset(&pd->pppif, 0, sizeof(pd->pppif));
      pd->net_status = MGOS_NET_EV_DISCONNECTED;
      pd->net_status_last_reported = MGOS_NET_EV_DISCONNECTED;
      prepare_cmds(pd);
      mgos_pppos_set_state(pd, PPPOS_ENTER_WAIT);
      break;
    }
    case PPPOS_ENTER_WAIT: {
      if (pd->deadline == 0) {
        /* Initial 1s pause before sending +++ */
        pd->deadline = now + 1.0;
      } else if (now > pd->deadline) {
        mgos_pppos_set_state(pd, PPPOS_ENTER);
      }
      /* Discard all the incoming data. */
      mbuf_remove(&pd->data, pd->data.len);
      break;
    }
    case PPPOS_ENTER: {
      if (pd->deadline == 0) {
        mgos_pppos_at_cmd(uart_no, "+++");
        /* At least 1s after +++ */
        pd->deadline = now + 1.2;
      } else if (now > pd->deadline) {
        mgos_pppos_set_net_status(pd, MGOS_NET_EV_CONNECTING);
        mgos_pppos_set_state(pd, PPPOS_SETUP);
      }
      /* Keep discarding all the data. */
      mbuf_remove(&pd->data, pd->data.len);
      break;
    }
    case PPPOS_SETUP: {
      if (pd->deadline == 0) {
        mgos_pppos_at_cmd(uart_no, pd->cmds[pd->cmd_idx]);
        pd->deadline = now + AT_CMD_TIMEOUT;
      } else if (now > pd->deadline) {
        LOG(LL_INFO, ("Command timed out: %s", pd->cmds[pd->cmd_idx]));
        mgos_pppos_set_state(pd, PPPOS_START);
      } else {
        /* Consume data line by line */
        struct mg_str s = mg_mk_str_n(pd->data.buf, pd->data.len);
        const char *eol;
        while ((eol = mg_strchr(s, '\r')) != NULL) {
          struct mg_str l = mg_mk_str_n(s.p, eol - s.p);
          s.len -= (l.len + 1);
          s.p += (l.len + 1);
          /* Strip leading whitespace */
          while (l.len > 0 && isspace((int) *l.p)) {
            l.p++;
            l.len--;
          }
          if (l.len == 0) continue;
          LOG(LL_DEBUG, ("<< %.*s", (int) l.len, l.p));
          if (mg_vcmp(&l, "OK") == 0 ||
              mg_strstr(l, mg_mk_str("CONNECT")) == l.p) {
            pd->cmd_idx++;
            if (pd->cmd_idx >= pd->num_cmds) {
              free_cmds(pd);
              mgos_pppos_set_net_status(pd, MGOS_NET_EV_CONNECTED);
              mgos_pppos_set_state(pd, PPPOS_START_PPP);
              break;
            } else {
              pd->deadline = 0;
            }
          } else if (mg_vcmp(&l, "ERROR") == 0) {
            LOG(LL_INFO, ("Command failed: %s", pd->cmds[pd->cmd_idx]));
            mgos_pppos_set_state(pd, PPPOS_START);
            break;
          }
        }
        if (pd->state == PPPOS_SETUP) {
          mbuf_remove(&pd->data, s.p - pd->data.buf);
        }
      }
      break;
    }
    case PPPOS_START_PPP: {
      const char *user = pd->cfg.user;
      LOG(LL_INFO, ("Starting PPP, user '%s'", (user ? user : "")));
      mbuf_remove(&pd->data, pd->data.len);
      pd->pppcb = pppapi_pppos_create(&pd->pppif, mgos_pppos_send_cb,
                                      mgos_pppos_status_cb, pd);
      LOG(LL_DEBUG, ("%p", pd->pppcb));
      if (pd->pppcb == NULL) {
        LOG(LL_ERROR, ("pppapi_pppos_create failed"));
        mgos_pppos_set_state(pd, PPPOS_START);
        break;
      }
      if (user != NULL) {
        pppapi_set_auth(pd->pppcb, PPPAUTHTYPE_PAP, user, pd->cfg.pass);
      }
      pd->pppcb->settings.lcp_echo_interval = pd->cfg.echo_interval;
      pd->pppcb->settings.lcp_echo_fails = pd->cfg.echo_fails;
      err_t err = pppapi_connect(pd->pppcb, 0 /* holdoff */);
      if (err != ERR_OK) {
        LOG(LL_ERROR, ("pppapi_connect failed: %d", err));
        pppapi_free(pd->pppcb);
        mgos_pppos_set_state(pd, PPPOS_START);
      }
      mgos_pppos_set_state(pd, PPPOS_RUN);
      break;
    }
    case PPPOS_RUN: {
      if (pd->data.len > 0) {
        pppos_input_tcpip(pd->pppcb, (u8_t *) pd->data.buf, pd->data.len);
        mbuf_remove(&pd->data, pd->data.len);
      }
      break;
    }
    case PPPOS_CLOSE: {
      break;
    }
  }
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

bool mgos_pppos_create(const struct mgos_config_pppos *cfg, int if_instance) {
  if (cfg->connect_cmd == NULL) {
    LOG(LL_ERROR, ("connect_cmd is required"));
    return NULL;
  }
  struct mgos_uart_config ucfg;
  mgos_uart_config_set_defaults(cfg->uart_no, &ucfg);
  ucfg.baud_rate = cfg->baud_rate;
  ucfg.rx_buf_size = 1500;
  ucfg.tx_buf_size = 1500;
  if (cfg->fc_enable) {
    ucfg.rx_fc_type = MGOS_UART_FC_HW;
    ucfg.tx_fc_type = MGOS_UART_FC_HW;
  }
  LOG(LL_INFO,
      ("PPPoS UART%d, %d, fc %s, APN '%s'", cfg->uart_no, ucfg.baud_rate,
       cfg->fc_enable ? "on" : "off", (cfg->apn ? cfg->apn : "")));
  if (!mgos_uart_configure(cfg->uart_no, &ucfg)) {
    LOG(LL_ERROR, ("Failed to configure UART%d", cfg->uart_no));
    return false;
  }
  struct mgos_pppos_data *pd =
      (struct mgos_pppos_data *) calloc(1, sizeof(*pd));
  memcpy(&pd->cfg, cfg, sizeof(pd->cfg));
  pd->if_instance = if_instance;
  pd->state = PPPOS_START;
  SLIST_INSERT_HEAD(&s_pds, pd, next);
  mgos_uart_set_dispatcher(cfg->uart_no, mgos_pppos_uart_dispatcher, pd);
  mgos_uart_set_rx_enabled(cfg->uart_no, true);
  return true;
}

bool mgos_pppos_init(void) {
  if (!mgos_sys_config_get_pppos_enable()) return true;
  return mgos_pppos_create(&mgos_sys_config.pppos, 0 /* if_instance */);
}
