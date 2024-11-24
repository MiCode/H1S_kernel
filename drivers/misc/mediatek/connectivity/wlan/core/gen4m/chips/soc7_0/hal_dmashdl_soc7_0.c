/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*! \file   hal_dmashdl_soc7_0.c
*    \brief  DMASHDL HAL API for soc7_0
*
*    This file contains all routines which are exported
     from MediaTek 802.11 Wireless LAN driver stack to GLUE Layer.
*/

#ifdef SOC7_0
#if defined(_HIF_PCIE) || defined(_HIF_AXI)

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

#include "precomp.h"
#include "soc7_0.h"
#include "coda/soc7_0/wf_ple_top.h"
#include "coda/soc7_0/wf_pse_top.h"
#include "coda/soc7_0/wf_hif_dmashdl_top.h"
#include "hal_dmashdl_soc7_0.h"

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/


/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

struct DMASHDL_CFG rSOC7_0_DmashdlCfg = {
	.fgSlotArbiterEn = SOC7_0_DMASHDL_SLOT_ARBITER_EN,

	.u2PktPleMaxPage = SOC7_0_DMASHDL_PKT_PLE_MAX_PAGE,

	.u2PktPseMaxPage = SOC7_0_DMASHDL_PKT_PSE_MAX_PAGE,

	.afgRefillEn = {
		SOC7_0_DMASHDL_GROUP_0_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_1_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_2_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_3_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_4_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_5_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_6_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_7_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_8_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_9_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_10_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_11_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_12_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_13_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_14_REFILL_EN,
		SOC7_0_DMASHDL_GROUP_15_REFILL_EN,
	},

	.au2MaxQuota = {
		SOC7_0_DMASHDL_GROUP_0_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_1_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_2_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_3_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_4_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_5_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_6_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_7_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_8_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_9_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_10_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_11_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_12_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_13_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_14_MAX_QUOTA,
		SOC7_0_DMASHDL_GROUP_15_MAX_QUOTA,
	},

	.au2MinQuota = {
		SOC7_0_DMASHDL_GROUP_0_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_1_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_2_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_3_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_4_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_5_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_6_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_7_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_8_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_9_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_10_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_11_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_12_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_13_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_14_MIN_QUOTA,
		SOC7_0_DMASHDL_GROUP_15_MIN_QUOTA,
	},

	.aucQueue2Group = {
		SOC7_0_DMASHDL_QUEUE_0_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_1_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_2_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_3_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_4_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_5_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_6_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_7_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_8_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_9_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_10_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_11_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_12_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_13_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_14_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_15_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_16_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_17_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_18_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_19_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_20_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_21_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_22_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_23_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_24_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_25_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_26_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_27_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_28_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_29_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_30_TO_GROUP,
		SOC7_0_DMASHDL_QUEUE_31_TO_GROUP,
	},

	.aucPriority2Group = {
		SOC7_0_DMASHDL_PRIORITY0_GROUP,
		SOC7_0_DMASHDL_PRIORITY1_GROUP,
		SOC7_0_DMASHDL_PRIORITY2_GROUP,
		SOC7_0_DMASHDL_PRIORITY3_GROUP,
		SOC7_0_DMASHDL_PRIORITY4_GROUP,
		SOC7_0_DMASHDL_PRIORITY5_GROUP,
		SOC7_0_DMASHDL_PRIORITY6_GROUP,
		SOC7_0_DMASHDL_PRIORITY7_GROUP,
		SOC7_0_DMASHDL_PRIORITY8_GROUP,
		SOC7_0_DMASHDL_PRIORITY9_GROUP,
		SOC7_0_DMASHDL_PRIORITY10_GROUP,
		SOC7_0_DMASHDL_PRIORITY11_GROUP,
		SOC7_0_DMASHDL_PRIORITY12_GROUP,
		SOC7_0_DMASHDL_PRIORITY13_GROUP,
		SOC7_0_DMASHDL_PRIORITY14_GROUP,
		SOC7_0_DMASHDL_PRIORITY15_GROUP,
	},

	.u2HifAckCntTh = SOC7_0_DMASHDL_HIF_ACK_CNT_TH,
	.u2HifGupActMap = SOC7_0_DMASHDL_HIF_GUP_ACT_MAP,
	.u4GroupNum = ENUM_DMASHDL_GROUP_NUM,

	.rPlePacketMaxSize = {
		WF_HIF_DMASHDL_TOP_PACKET_MAX_SIZE_ADDR,
		WF_HIF_DMASHDL_TOP_PACKET_MAX_SIZE_PLE_PACKET_MAX_SIZE_MASK,
		WF_HIF_DMASHDL_TOP_PACKET_MAX_SIZE_PLE_PACKET_MAX_SIZE_SHFT
	},

	.rPsePacketMaxSize = {
		WF_HIF_DMASHDL_TOP_PACKET_MAX_SIZE_ADDR,
		WF_HIF_DMASHDL_TOP_PACKET_MAX_SIZE_PSE_PACKET_MAX_SIZE_MASK,
		WF_HIF_DMASHDL_TOP_PACKET_MAX_SIZE_PSE_PACKET_MAX_SIZE_SHFT
	},

	.rGroup0RefillDisable = {
		WF_HIF_DMASHDL_TOP_REFILL_CONTROL_ADDR,
		WF_HIF_DMASHDL_TOP_REFILL_CONTROL_GROUP0_REFILL_DISABLE_MASK,
		0
	},

	.rGroup0ControlMaxQuota = {
		WF_HIF_DMASHDL_TOP_GROUP0_CONTROL_ADDR,
		WF_HIF_DMASHDL_TOP_GROUP0_CONTROL_GROUP0_MAX_QUOTA_MASK,
		WF_HIF_DMASHDL_TOP_GROUP0_CONTROL_GROUP0_MAX_QUOTA_SHFT
	},

	.rGroup0ControlMinQuota = {
		WF_HIF_DMASHDL_TOP_GROUP0_CONTROL_ADDR,
		WF_HIF_DMASHDL_TOP_GROUP0_CONTROL_GROUP0_MIN_QUOTA_MASK,
		WF_HIF_DMASHDL_TOP_GROUP0_CONTROL_GROUP0_MIN_QUOTA_SHFT
	},

	.rQueueMapping0Queue0 = {
		WF_HIF_DMASHDL_TOP_QUEUE_MAPPING0_ADDR,
		WF_HIF_DMASHDL_TOP_QUEUE_MAPPING0_QUEUE0_MAPPING_MASK,
		WF_HIF_DMASHDL_TOP_QUEUE_MAPPING0_QUEUE0_MAPPING_SHFT
	},

	.rPageSettingGroupSeqOrderType = {
		WF_HIF_DMASHDL_TOP_PAGE_SETTING_ADDR,
		WF_HIF_DMASHDL_TOP_PAGE_SETTING_GROUP_SEQUENCE_ORDER_TYPE_MASK,
		WF_HIF_DMASHDL_TOP_PAGE_SETTING_GROUP_SEQUENCE_ORDER_TYPE_SHFT,
	},

	.rSchdulerSetting0Priority0Group = {
		WF_HIF_DMASHDL_TOP_HIF_SCHEDULER_SETTING0_ADDR,
		WF_HIF_DMASHDL_TOP_HIF_SCHEDULER_SETTING0_PRIORITY0_GROUP_MASK,
		WF_HIF_DMASHDL_TOP_HIF_SCHEDULER_SETTING0_PRIORITY0_GROUP_SHFT
	},

	.rStatusRdGp0RsvCnt = {
		WF_HIF_DMASHDL_TOP_STATUS_RD_GP0_ADDR,
		WF_HIF_DMASHDL_TOP_STATUS_RD_GP0_G0_RSV_CNT_MASK,
		WF_HIF_DMASHDL_TOP_STATUS_RD_GP0_G0_RSV_CNT_SHFT
	},

	.rStatusRdGp0SrcCnt = {
		WF_HIF_DMASHDL_TOP_STATUS_RD_GP0_ADDR,
		WF_HIF_DMASHDL_TOP_STATUS_RD_GP0_G0_SRC_CNT_MASK,
		WF_HIF_DMASHDL_TOP_STATUS_RD_GP0_G0_SRC_CNT_SHFT
	},

	.rRdGroupPktCnt0 = {
		WF_HIF_DMASHDL_TOP_RD_GROUP_PKT_CNT0_ADDR,
		0,
		0
	},

	.rOptionalControlCrHifAckCntTh = {
		WF_HIF_DMASHDL_TOP_OPTIONAL_CONTROL_ADDR,
		WF_HIF_DMASHDL_TOP_OPTIONAL_CONTROL_CR_HIF_ACK_CNT_TH_MASK,
		WF_HIF_DMASHDL_TOP_OPTIONAL_CONTROL_CR_HIF_ACK_CNT_TH_SHFT
	},

	.rOptionalControlCrHifGupActMap = {
		WF_HIF_DMASHDL_TOP_OPTIONAL_CONTROL_ADDR,
		WF_HIF_DMASHDL_TOP_OPTIONAL_CONTROL_CR_HIF_GUP_ACT_MAP_MASK,
		WF_HIF_DMASHDL_TOP_OPTIONAL_CONTROL_CR_HIF_GUP_ACT_MAP_SHFT,
	},

	.rErrorFlagCtrl = {
		WF_HIF_DMASHDL_TOP_ERROR_FLAG_CTRL_ADDR,
		0,
		0
	},

	.rStatusRdFfaCnt = {
		WF_HIF_DMASHDL_TOP_STATUS_RD_ADDR,
		WF_HIF_DMASHDL_TOP_STATUS_RD_FFA_CNT_MASK,
		WF_HIF_DMASHDL_TOP_STATUS_RD_FFA_CNT_SHFT
	},

	.rStatusRdFreePageCnt = {
		WF_HIF_DMASHDL_TOP_STATUS_RD_ADDR,
		WF_HIF_DMASHDL_TOP_STATUS_RD_FREE_PAGE_CNT_MASK,
		WF_HIF_DMASHDL_TOP_STATUS_RD_FREE_PAGE_CNT_SHFT
	},

	.rHifPgInfoHifRsvCnt = {
		WF_PLE_TOP_HIF_PG_INFO_ADDR,
		WF_PLE_TOP_HIF_PG_INFO_HIF_RSV_CNT_MASK,
		WF_PLE_TOP_HIF_PG_INFO_HIF_RSV_CNT_SHFT
	},

	.rHifPgInfoHifSrcCnt = {
		WF_PLE_TOP_HIF_PG_INFO_ADDR,
		WF_PLE_TOP_HIF_PG_INFO_HIF_SRC_CNT_MASK,
		WF_PLE_TOP_HIF_PG_INFO_HIF_SRC_CNT_SHFT
	},
};

void soc7_0DmashdlInit(struct ADAPTER *prAdapter)
{
	uint32_t idx;

	asicConnac2xDmashdlSetPlePktMaxPage(prAdapter,
					 rSOC7_0_DmashdlCfg.u2PktPleMaxPage);

	asicConnac2xDmashdlSetPsePktMaxPage(prAdapter,
					 rSOC7_0_DmashdlCfg.u2PktPseMaxPage);

	for (idx = 0; idx < ENUM_DMASHDL_GROUP_NUM; idx++) {
		asicConnac2xDmashdlSetRefill(
			prAdapter, idx,
			rSOC7_0_DmashdlCfg.afgRefillEn[idx]);

		asicConnac2xDmashdlSetMaxQuota(
			prAdapter, idx,
			rSOC7_0_DmashdlCfg.au2MaxQuota[idx]);

		asicConnac2xDmashdlSetMinQuota(
			prAdapter, idx,
			rSOC7_0_DmashdlCfg.au2MinQuota[idx]);
	}

	for (idx = 0; idx < 32; idx++)
		asicConnac2xDmashdlSetQueueMapping(
			prAdapter, idx,
			rSOC7_0_DmashdlCfg.aucQueue2Group[idx]);

	for (idx = 0; idx < 16; idx++)
		asicConnac2xDmashdlSetUserDefinedPriority(
			prAdapter, idx,
			rSOC7_0_DmashdlCfg.aucPriority2Group[idx]);

	asicConnac2xDmashdlSetSlotArbiter(prAdapter,
				       rSOC7_0_DmashdlCfg.fgSlotArbiterEn);

	asicConnac2xDmashdlSetOptionalControl(prAdapter,
		rSOC7_0_DmashdlCfg.u2HifAckCntTh,
		rSOC7_0_DmashdlCfg.u2HifGupActMap);
}

#endif /* defined(_HIF_PCIE) || defined(_HIF_AXI) */
#endif /* SOC7_0 */
