/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __WF_TOP_SLPPROT_ON_REGS_H__
#define __WF_TOP_SLPPROT_ON_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WF_TOP_SLPPROT_ON_BASE                                 0x184C3000

#define WF_TOP_SLPPROT_ON_STATUS_READ_ADDR                     (WF_TOP_SLPPROT_ON_BASE + 0xC)

#define WF_TOP_SLPPROT_ON_STATUS_READ_ro_slpprot_en_source_1_ADDR WF_TOP_SLPPROT_ON_STATUS_READ_ADDR
#define WF_TOP_SLPPROT_ON_STATUS_READ_ro_slpprot_en_source_1_MASK 0x00800000
#define WF_TOP_SLPPROT_ON_STATUS_READ_ro_slpprot_en_source_1_SHFT 23
#define WF_TOP_SLPPROT_ON_STATUS_READ_ro_slpprot_en_source_2_ADDR WF_TOP_SLPPROT_ON_STATUS_READ_ADDR
#define WF_TOP_SLPPROT_ON_STATUS_READ_ro_slpprot_en_source_2_MASK 0x00200000
#define WF_TOP_SLPPROT_ON_STATUS_READ_ro_slpprot_en_source_2_SHFT 21

#ifdef __cplusplus
}
#endif

#endif
