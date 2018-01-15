/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 */

#ifndef CS_MOS_LIBS_PPPOS_SRC_MGOS_PPPOS_H_
#define CS_MOS_LIBS_PPPOS_SRC_MGOS_PPPOS_H_

#include "mgos_net.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*
 * Retrieve IP configuration of the provided instance number (which should be
 * of type `MGOS_NET_IF_TYPE_PPP`), and fill provided `ip_info` with it. Returns
 * `true` in case of success, false otherwise.
 */
bool mgos_pppos_dev_get_ip_info(int if_instance,
                                struct mgos_net_ip_info *ip_info);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CS_MOS_LIBS_PPPOS_SRC_MGOS_PPPOS_H_ */
