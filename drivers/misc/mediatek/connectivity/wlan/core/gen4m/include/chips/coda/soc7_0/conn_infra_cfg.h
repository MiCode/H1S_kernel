/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __CONN_INFRA_CFG_REGS_H__
#define __CONN_INFRA_CFG_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONN_INFRA_CFG_BASE                                    0x18011000

#define CONN_INFRA_CFG_IP_VERSION_ADDR                         (CONN_INFRA_CFG_BASE + 0x000)
#define CONN_INFRA_CFG_EMI_CTL_WF_ADDR                         (CONN_INFRA_CFG_BASE + 0x114)

#ifdef __cplusplus
}
#endif

#endif
