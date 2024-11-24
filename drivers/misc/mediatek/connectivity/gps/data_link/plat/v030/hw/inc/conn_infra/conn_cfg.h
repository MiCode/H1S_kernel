/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef __CONN_CFG_REGS_H__
#define __CONN_CFG_REGS_H__


#define CONN_CFG_BASE                                          0x18001000

#define CONN_CFG_PLL_STATUS_ADDR                               (CONN_CFG_BASE + 0x320)


#define CONN_CFG_PLL_STATUS_BPLL_RDY_ADDR                      CONN_CFG_PLL_STATUS_ADDR
#define CONN_CFG_PLL_STATUS_BPLL_RDY_MASK                      0x00000002
#define CONN_CFG_PLL_STATUS_BPLL_RDY_SHFT                      1

#endif /* __CONN_CFG_REGS_H__ */
