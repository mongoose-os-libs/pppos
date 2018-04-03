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
