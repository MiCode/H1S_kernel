/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __CONN_WT_SLP_CTL_REG_REGS_H__
#define __CONN_WT_SLP_CTL_REG_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONN_WT_SLP_CTL_REG_BASE                               0x18003000

#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_ADDR               (CONN_WT_SLP_CTL_REG_BASE + 0x124)

#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_WB_SLP_TOP_CK_1_BSY_ADDR CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_ADDR
#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_WB_SLP_TOP_CK_1_BSY_MASK 0x00000002
#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_WB_SLP_TOP_CK_1_BSY_SHFT 1

#ifdef __cplusplus
}
#endif

#endif
