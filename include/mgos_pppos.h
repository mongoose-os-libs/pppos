/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#ifndef CS_MOS_LIBS_PPPOS_INCLUDE_MGOS_PPPOS_H_
#define CS_MOS_LIBS_PPPOS_INCLUDE_MGOS_PPPOS_H_

#include "mgos_net.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

bool mgos_pppos_dev_get_ip_info(int if_instance,
                                struct mgos_net_ip_info *ip_info);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CS_MOS_LIBS_PPPOS_INCLUDE_MGOS_PPPOS_H_ */
