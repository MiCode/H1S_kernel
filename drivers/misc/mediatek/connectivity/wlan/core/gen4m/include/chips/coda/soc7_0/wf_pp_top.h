/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __WF_PP_TOP_REGS_H__
#define __WF_PP_TOP_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WF_PP_TOP_BASE 0x820CC000

#define WF_PP_TOP_DBG_CTRL_ADDR \
	(WF_PP_TOP_BASE + 0x00FC)
#define WF_PP_TOP_DBG_CS_0_ADDR \
	(WF_PP_TOP_BASE + 0x0104)
#define WF_PP_TOP_DBG_CS_1_ADDR \
	(WF_PP_TOP_BASE + 0x0108)
#define WF_PP_TOP_DBG_CS_2_ADDR \
	(WF_PP_TOP_BASE + 0x010C)
#define WF_PP_TOP_DBG_CS_3_ADDR \
	(WF_PP_TOP_BASE + 0x00D0)
#define WF_PP_TOP_DBG_CS_4_ADDR \
	(WF_PP_TOP_BASE + 0x00E8)
#define WF_PP_TOP_DBG_CS_5_ADDR \
	(WF_PP_TOP_BASE + 0x00EC)

#ifdef __cplusplus
}
#endif

#endif
