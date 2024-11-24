/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*! \file   hal_dmashdl_soc7_0.h
*    \brief  DMASHDL HAL API for soc7_0
*
*    This file contains all routines which are exported
     from MediaTek 802.11 Wireless LAN driver stack to GLUE Layer.
*/

#ifndef _HAL_DMASHDL_SOC7_0_H
#define _HAL_DMASHDL_SOC7_0_H

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/
/* align MT7663 setting first */

/* 1: 3rd arbitration makes decision based on group priority in current slot.
 * 0: 3rd arbitration makes decision based on fixed user-defined priority.
 */
#define SOC7_0_DMASHDL_SLOT_ARBITER_EN                 (0)
#define SOC7_0_DMASHDL_PKT_PLE_MAX_PAGE                (0x1)
/* Buzzard CMD packet flow control is controlled by WFDMA, not DMASHDL.
 * So, CMD packet (group 15) related CRs in DMASHDL are ignored.
 */
#define SOC7_0_DMASHDL_PKT_PSE_MAX_PAGE                (0x18)
#define SOC7_0_DMASHDL_GROUP_0_REFILL_EN               (1)
#define SOC7_0_DMASHDL_GROUP_1_REFILL_EN               (1)
#define SOC7_0_DMASHDL_GROUP_2_REFILL_EN               (1)
#define SOC7_0_DMASHDL_GROUP_3_REFILL_EN               (1)
#define SOC7_0_DMASHDL_GROUP_4_REFILL_EN               (0)
#define SOC7_0_DMASHDL_GROUP_5_REFILL_EN               (0)
#define SOC7_0_DMASHDL_GROUP_6_REFILL_EN               (0)
#define SOC7_0_DMASHDL_GROUP_7_REFILL_EN               (0)
#define SOC7_0_DMASHDL_GROUP_8_REFILL_EN               (0)
#define SOC7_0_DMASHDL_GROUP_9_REFILL_EN               (0)
#define SOC7_0_DMASHDL_GROUP_10_REFILL_EN              (0)
#define SOC7_0_DMASHDL_GROUP_11_REFILL_EN              (0)
#define SOC7_0_DMASHDL_GROUP_12_REFILL_EN              (0)
#define SOC7_0_DMASHDL_GROUP_13_REFILL_EN              (0)
#define SOC7_0_DMASHDL_GROUP_14_REFILL_EN              (0)
#define SOC7_0_DMASHDL_GROUP_15_REFILL_EN              (0)
#define SOC7_0_DMASHDL_GROUP_0_MAX_QUOTA               (0x200)
#define SOC7_0_DMASHDL_GROUP_1_MAX_QUOTA               (0x200)
#define SOC7_0_DMASHDL_GROUP_2_MAX_QUOTA               (0x200)
#define SOC7_0_DMASHDL_GROUP_3_MAX_QUOTA               (0x1ff)
#define SOC7_0_DMASHDL_GROUP_4_MAX_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_5_MAX_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_6_MAX_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_7_MAX_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_8_MAX_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_9_MAX_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_10_MAX_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_11_MAX_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_12_MAX_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_13_MAX_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_14_MAX_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_15_MAX_QUOTA              (0x30)
#define SOC7_0_DMASHDL_GROUP_0_MIN_QUOTA               (0x10)
#define SOC7_0_DMASHDL_GROUP_1_MIN_QUOTA               (0x10)
#define SOC7_0_DMASHDL_GROUP_2_MIN_QUOTA               (0x10)
#define SOC7_0_DMASHDL_GROUP_3_MIN_QUOTA               (0x10)
#define SOC7_0_DMASHDL_GROUP_4_MIN_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_5_MIN_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_6_MIN_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_7_MIN_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_8_MIN_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_9_MIN_QUOTA               (0x0)
#define SOC7_0_DMASHDL_GROUP_10_MIN_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_11_MIN_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_12_MIN_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_13_MIN_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_14_MIN_QUOTA              (0x0)
#define SOC7_0_DMASHDL_GROUP_15_MIN_QUOTA              (0x30)
#define SOC7_0_DMASHDL_QUEUE_0_TO_GROUP                (0x0)   /* LMAC AC00 */
#define SOC7_0_DMASHDL_QUEUE_1_TO_GROUP                (0x0)   /* LMAC AC01 */
#define SOC7_0_DMASHDL_QUEUE_2_TO_GROUP                (0x0)   /* LMAC AC02 */
#define SOC7_0_DMASHDL_QUEUE_3_TO_GROUP                (0x2)   /* LMAC AC03 */
#define SOC7_0_DMASHDL_QUEUE_4_TO_GROUP                (0x1)   /* LMAC AC10 */
#define SOC7_0_DMASHDL_QUEUE_5_TO_GROUP                (0x1)   /* LMAC AC11 */
#define SOC7_0_DMASHDL_QUEUE_6_TO_GROUP                (0x1)   /* LMAC AC12 */
#define SOC7_0_DMASHDL_QUEUE_7_TO_GROUP                (0x2)   /* LMAC AC13 */
#define SOC7_0_DMASHDL_QUEUE_8_TO_GROUP                (0x0)   /* LMAC AC20 */
#define SOC7_0_DMASHDL_QUEUE_9_TO_GROUP                (0x0)   /* LMAC AC21 */
#define SOC7_0_DMASHDL_QUEUE_10_TO_GROUP               (0x0)   /* LMAC AC22 */
#define SOC7_0_DMASHDL_QUEUE_11_TO_GROUP               (0x2)   /* LMAC AC23 */
#define SOC7_0_DMASHDL_QUEUE_12_TO_GROUP               (0x1)   /* LMAC AC30 */
#define SOC7_0_DMASHDL_QUEUE_13_TO_GROUP               (0x1)   /* LMAC AC31 */
#define SOC7_0_DMASHDL_QUEUE_14_TO_GROUP               (0x1)   /* LMAC AC32 */
#define SOC7_0_DMASHDL_QUEUE_15_TO_GROUP               (0x2)   /* LMAC AC33 */
#define SOC7_0_DMASHDL_QUEUE_16_TO_GROUP               (0x0)   /* ALTX */
#define SOC7_0_DMASHDL_QUEUE_17_TO_GROUP               (0x0)   /* BMC */
#define SOC7_0_DMASHDL_QUEUE_18_TO_GROUP               (0x0)   /* BCN */
#define SOC7_0_DMASHDL_QUEUE_19_TO_GROUP               (0x1)   /* HW Reserved */
#define SOC7_0_DMASHDL_QUEUE_20_TO_GROUP               (0x1)   /* HW Reserved */
#define SOC7_0_DMASHDL_QUEUE_21_TO_GROUP               (0x1)   /* HW Reserved */
#define SOC7_0_DMASHDL_QUEUE_22_TO_GROUP               (0x1)   /* HW Reserved */
#define SOC7_0_DMASHDL_QUEUE_23_TO_GROUP               (0x1)   /* HW Reserved */
#define SOC7_0_DMASHDL_QUEUE_24_TO_GROUP               (0x0)   /* NAF */
#define SOC7_0_DMASHDL_QUEUE_25_TO_GROUP               (0x0)   /* NBCN */
#define SOC7_0_DMASHDL_QUEUE_26_TO_GROUP               (0x0)   /* FIXFID */
#define SOC7_0_DMASHDL_QUEUE_27_TO_GROUP               (0x1)   /* Reserved */
#define SOC7_0_DMASHDL_QUEUE_28_TO_GROUP               (0x1)   /* Reserved */
#define SOC7_0_DMASHDL_QUEUE_29_TO_GROUP               (0x1)   /* Reserved */
#define SOC7_0_DMASHDL_QUEUE_30_TO_GROUP               (0x1)   /* Reserved */
#define SOC7_0_DMASHDL_QUEUE_31_TO_GROUP               (0x1)   /* Reserved */
#define SOC7_0_DMASHDL_PRIORITY0_GROUP                 (0x0)
#define SOC7_0_DMASHDL_PRIORITY1_GROUP                 (0x1)
#define SOC7_0_DMASHDL_PRIORITY2_GROUP                 (0x2)
#define SOC7_0_DMASHDL_PRIORITY3_GROUP                 (0x3)
#define SOC7_0_DMASHDL_PRIORITY4_GROUP                 (0x4)
#define SOC7_0_DMASHDL_PRIORITY5_GROUP                 (0x5)
#define SOC7_0_DMASHDL_PRIORITY6_GROUP                 (0x6)
#define SOC7_0_DMASHDL_PRIORITY7_GROUP                 (0x7)
#define SOC7_0_DMASHDL_PRIORITY8_GROUP                 (0x8)
#define SOC7_0_DMASHDL_PRIORITY9_GROUP                 (0x9)
#define SOC7_0_DMASHDL_PRIORITY10_GROUP                (0xA)
#define SOC7_0_DMASHDL_PRIORITY11_GROUP                (0xB)
#define SOC7_0_DMASHDL_PRIORITY12_GROUP                (0xC)
#define SOC7_0_DMASHDL_PRIORITY13_GROUP                (0xD)
#define SOC7_0_DMASHDL_PRIORITY14_GROUP                (0xE)
#define SOC7_0_DMASHDL_PRIORITY15_GROUP                (0xF)
/* 4 rings are used (3 data + cmd) */
#define SOC7_0_DMASHDL_HIF_ACK_CNT_TH                  (0x4)
/* Ring 0/1/2/15 are used */
#define SOC7_0_DMASHDL_HIF_GUP_ACT_MAP                 (0x8007)

/*******************************************************************************
*                         D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/
extern struct DMASHDL_CFG rSOC7_0_DmashdlCfg;

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/

void soc7_0DmashdlInit(struct ADAPTER *prAdapter);

#endif /* _HAL_DMASHDL_MT7961_H */
