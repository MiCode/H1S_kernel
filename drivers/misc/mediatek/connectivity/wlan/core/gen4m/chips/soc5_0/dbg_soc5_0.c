/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/******************************************************************************
 *[File]             dbg_soc5_0.c
 *[Version]          v1.0
 *[Revision Date]    2020-05-22
 *[Author]
 *[Description]
 *    The program provides WIFI FALCON MAC Debug APIs
 *[Copyright]
 *    Copyright (C) 2015 MediaTek Incorporation. All Rights Reserved.
 ******************************************************************************/

#ifdef SOC5_0

#include "precomp.h"
#include "soc5_0.h"
#include "coda/soc5_0/wf_wfdma_host_dma0.h"
#include "coda/soc5_0/wf_wfdma_mcu_dma0.h"
#include "coda/soc5_0/wf_hif_dmashdl_top.h"
#include "coda/soc5_0/wf_ple_top.h"
#include "coda/soc5_0/wf_pse_top.h"

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */
struct PLE_TOP_CR rSoc5_0_PleTopCr = {
	.rAc0QueueEmpty0 = {
		WF_PLE_TOP_AC0_QUEUE_EMPTY0_ADDR,
		0,
		0
	},
	.rAc1QueueEmpty0 = {
		WF_PLE_TOP_AC1_QUEUE_EMPTY0_ADDR,
		0,
		0
	},
	.rAc2QueueEmpty0 = {
		WF_PLE_TOP_AC2_QUEUE_EMPTY0_ADDR,
		0,
		0
	},
	.rAc3QueueEmpty0 = {
		WF_PLE_TOP_AC3_QUEUE_EMPTY0_ADDR,
		0,
		0
	},
	.rCpuPgInfo = {
		WF_PLE_TOP_CPU_PG_INFO_ADDR,
		0,
		0
	},
	.rCpuPgInfoCpuRsvCnt = {
		0,
		WF_PLE_TOP_CPU_PG_INFO_CPU_RSV_CNT_MASK,
		WF_PLE_TOP_CPU_PG_INFO_CPU_RSV_CNT_SHFT
	},
	.rCpuPgInfoCpuSrcCnt = {
		0,
		WF_PLE_TOP_CPU_PG_INFO_CPU_SRC_CNT_MASK,
		WF_PLE_TOP_CPU_PG_INFO_CPU_SRC_CNT_SHFT
	},
	.rDisStaMap0 = {
		WF_PLE_TOP_DIS_STA_MAP0_ADDR,
		0,
		0
	},
	.rFlQueCtrl0 = {
		WF_PLE_TOP_FL_QUE_CTRL_0_ADDR,
		0,
		0
	},
	.rFlQueCtrl0Execute = {
		0,
		WF_PLE_TOP_FL_QUE_CTRL_0_EXECUTE_MASK,
		0
	},
	.rFlQueCtrl0QBufPid = {
		0,
		0,
		WF_PLE_TOP_FL_QUE_CTRL_0_Q_BUF_PID_SHFT
	},
	.rFlQueCtrl0QBufQid = {
		0,
		0,
		WF_PLE_TOP_FL_QUE_CTRL_0_Q_BUF_QID_SHFT
	},
	.rFlQueCtrl0QBufWlanid = {
		0,
		0,
		WF_PLE_TOP_FL_QUE_CTRL_0_Q_BUF_WLANID_SHFT
	},
	.rFlQueCtrl2 = {
		WF_PLE_TOP_FL_QUE_CTRL_2_ADDR,
		0,
		0
	},
	.rFlQueCtrl2QueueHeadFid = {
		0,
		WF_PLE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_MASK,
		WF_PLE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_SHFT
	},
	.rFlQueCtrl2QueueTailFid = {
		0,
		WF_PLE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_MASK,
		WF_PLE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_SHFT
	},
	.rFlQueCtrl3 = {
		WF_PLE_TOP_FL_QUE_CTRL_3_ADDR,
		0,
		0
	},
	.rFlQueCtrl3QueuePktNum = {
		0,
		WF_PLE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_MASK,
		WF_PLE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_SHFT
	},
	.rFreepgCnt = {
		WF_PLE_TOP_FREEPG_CNT_ADDR,
		0,
		0
	},
	.rFreepgCntFfaCnt = {
		0,
		WF_PLE_TOP_FREEPG_CNT_FFA_CNT_MASK,
		WF_PLE_TOP_FREEPG_CNT_FFA_CNT_SHFT
	},
	.rFreepgCntFreepgCnt = {
		0,
		WF_PLE_TOP_FREEPG_CNT_FREEPG_CNT_MASK,
		WF_PLE_TOP_FREEPG_CNT_FREEPG_CNT_SHFT
	},
	.rFreepgHeadTail = {
		WF_PLE_TOP_FREEPG_HEAD_TAIL_ADDR,
		0,
		0
	},
	.rFreepgHeadTailFreepgHead = {
		0,
		WF_PLE_TOP_FREEPG_HEAD_TAIL_FREEPG_HEAD_MASK,
		WF_PLE_TOP_FREEPG_HEAD_TAIL_FREEPG_HEAD_SHFT
	},
	.rFreepgHeadTailFreepgTail = {
		0,
		WF_PLE_TOP_FREEPG_HEAD_TAIL_FREEPG_TAIL_MASK,
		WF_PLE_TOP_FREEPG_HEAD_TAIL_FREEPG_TAIL_SHFT
	},
	.rFsmPeekCr00 = {
		WF_PLE_TOP_PEEK_CR_00_ADDR,
		0,
		0
	},
	.rFsmPeekCr01 = {
		WF_PLE_TOP_PEEK_CR_01_ADDR,
		0,
		0
	},
	.rFsmPeekCr02 = {
		WF_PLE_TOP_PEEK_CR_02_ADDR,
		0,
		0
	},
	.rFsmPeekCr03 = {
		WF_PLE_TOP_PEEK_CR_03_ADDR,
		0,
		0
	},
	.rFsmPeekCr04 = {
		WF_PLE_TOP_PEEK_CR_04_ADDR,
		0,
		0
	},
	.rFsmPeekCr05 = {
		WF_PLE_TOP_PEEK_CR_05_ADDR,
		0,
		0
	},
	.rFsmPeekCr06 = {
		WF_PLE_TOP_PEEK_CR_06_ADDR,
		0,
		0
	},
	.rFsmPeekCr07 = {
		WF_PLE_TOP_PEEK_CR_07_ADDR,
		0,
		0
	},
	.rFsmPeekCr08 = {
		WF_PLE_TOP_PEEK_CR_08_ADDR,
		0,
		0
	},
	.rFsmPeekCr09 = {
		WF_PLE_TOP_PEEK_CR_09_ADDR,
		0,
		0
	},
	.rFsmPeekCr10 = {
		WF_PLE_TOP_PEEK_CR_10_ADDR,
		0,
		0
	},
	.rFsmPeekCr11 = {
		WF_PLE_TOP_PEEK_CR_11_ADDR,
		0,
		0
	},
	.rHifPgInfo = {
		WF_PLE_TOP_HIF_PG_INFO_ADDR,
		0,
		0
	},
	.rHifPgInfoHifRsvCnt = {
		0,
		WF_PLE_TOP_HIF_PG_INFO_HIF_RSV_CNT_MASK,
		WF_PLE_TOP_HIF_PG_INFO_HIF_RSV_CNT_SHFT
	},
	.rHifPgInfoHifSrcCnt = {
		0,
		WF_PLE_TOP_HIF_PG_INFO_HIF_SRC_CNT_MASK,
		WF_PLE_TOP_HIF_PG_INFO_HIF_SRC_CNT_SHFT
	},
	.rHifTxcmdPgInfo = {
		WF_PLE_TOP_HIF_TXCMD_PG_INFO_ADDR,
		0,
		0
	},
	.rHifTxcmdPgInfoHifTxcmdRsvCnt = {
		0,
		WF_PLE_TOP_HIF_TXCMD_PG_INFO_HIF_TXCMD_RSV_CNT_MASK,
		WF_PLE_TOP_HIF_TXCMD_PG_INFO_HIF_TXCMD_RSV_CNT_SHFT
	},
	.rHifTxcmdPgInfoHifTxcmdSrcCnt = {
		0,
		WF_PLE_TOP_HIF_TXCMD_PG_INFO_HIF_TXCMD_SRC_CNT_MASK,
		WF_PLE_TOP_HIF_TXCMD_PG_INFO_HIF_TXCMD_SRC_CNT_SHFT
	},
	.rHifWmtxdPgInfo = {
		WF_PLE_TOP_HIF_WMTXD_PG_INFO_ADDR,
		0,
		0
	},
	.rHifWmtxdPgInfoHifWmtxdRsvCnt = {
		0,
		WF_PLE_TOP_HIF_WMTXD_PG_INFO_HIF_WMTXD_RSV_CNT_MASK,
		WF_PLE_TOP_HIF_WMTXD_PG_INFO_HIF_WMTXD_RSV_CNT_SHFT
	},
	.rHifWmtxdPgInfoHifWmtxdSrcCnt = {
		0,
		WF_PLE_TOP_HIF_WMTXD_PG_INFO_HIF_WMTXD_SRC_CNT_MASK,
		WF_PLE_TOP_HIF_WMTXD_PG_INFO_HIF_WMTXD_SRC_CNT_SHFT
	},
	.rIntN9ErrSts = {
		WF_PLE_TOP_INT_N9_ERR_STS_ADDR,
		0,
		0
	},
	.rIntN9ErrSts1 = {
		WF_PLE_TOP_INT_N9_ERR_STS_1_ADDR,
		0,
		0
	},
	.rPbufCtrl = {
		WF_PLE_TOP_PBUF_CTRL_ADDR,
		0,
		0
	},
	.rPbufCtrlPageSizeCfg = {
		0,
		WF_PLE_TOP_PBUF_CTRL_PAGE_SIZE_CFG_MASK,
		WF_PLE_TOP_PBUF_CTRL_PAGE_SIZE_CFG_SHFT
	},
	.rPbufCtrlPbufOffset = {
		0,
		WF_PLE_TOP_PBUF_CTRL_PBUF_OFFSET_MASK,
		WF_PLE_TOP_PBUF_CTRL_PBUF_OFFSET_SHFT
	},
	.rPbufCtrlTotalPageNum = {
		0,
		WF_PLE_TOP_PBUF_CTRL_TOTAL_PAGE_NUM_MASK,
		WF_PLE_TOP_PBUF_CTRL_TOTAL_PAGE_NUM_SHFT
	},
	.rPgCpuGroup = {
		WF_PLE_TOP_PG_CPU_GROUP_ADDR,
		0,
		0
	},
	.rPgCpuGroupCpuMaxQuota = {
		0,
		WF_PLE_TOP_PG_CPU_GROUP_CPU_MAX_QUOTA_MASK,
		WF_PLE_TOP_PG_CPU_GROUP_CPU_MAX_QUOTA_SHFT
	},
	.rPgCpuGroupCpuMinQuota = {
		0,
		WF_PLE_TOP_PG_CPU_GROUP_CPU_MIN_QUOTA_MASK,
		WF_PLE_TOP_PG_CPU_GROUP_CPU_MIN_QUOTA_SHFT
	},
	.rPgHifGroup = {
		WF_PLE_TOP_PG_HIF_GROUP_ADDR,
		0,
		0
	},
	.rPgHifGroupHifMaxQuota = {
		0,
		WF_PLE_TOP_PG_HIF_GROUP_HIF_MAX_QUOTA_MASK,
		WF_PLE_TOP_PG_HIF_GROUP_HIF_MAX_QUOTA_SHFT
	},
	.rPgHifGroupHifMinQuota = {
		0,
		WF_PLE_TOP_PG_HIF_GROUP_HIF_MIN_QUOTA_MASK,
		WF_PLE_TOP_PG_HIF_GROUP_HIF_MIN_QUOTA_SHFT
	},
	.rPgHifTxcmdGroup = {
		WF_PLE_TOP_PG_HIF_TXCMD_GROUP_ADDR,
		0,
		0
	},
	.rPgHifTxcmdGroupHifTxcmdMaxQuota = {
		0,
		WF_PLE_TOP_PG_HIF_TXCMD_GROUP_HIF_TXCMD_MAX_QUOTA_MASK,
		WF_PLE_TOP_PG_HIF_TXCMD_GROUP_HIF_TXCMD_MAX_QUOTA_SHFT
	},
	.rPgHifTxcmdGroupHifTxcmdMinQuota = {
		0,
		WF_PLE_TOP_PG_HIF_TXCMD_GROUP_HIF_TXCMD_MIN_QUOTA_MASK,
		WF_PLE_TOP_PG_HIF_TXCMD_GROUP_HIF_TXCMD_MIN_QUOTA_SHFT
	},
	.rPgHifWmtxdGroup = {
		WF_PLE_TOP_PG_HIF_WMTXD_GROUP_ADDR,
		0,
		0
	},
	.rPgHifWmtxdGroupHifWmtxdMaxQuota = {
		0,
		WF_PLE_TOP_PG_HIF_WMTXD_GROUP_HIF_WMTXD_MAX_QUOTA_MASK,
		WF_PLE_TOP_PG_HIF_WMTXD_GROUP_HIF_WMTXD_MAX_QUOTA_SHFT
	},
	.rPgHifWmtxdGroupHifWmtxdMinQuota = {
		0,
		WF_PLE_TOP_PG_HIF_WMTXD_GROUP_HIF_WMTXD_MIN_QUOTA_MASK,
		WF_PLE_TOP_PG_HIF_WMTXD_GROUP_HIF_WMTXD_MIN_QUOTA_SHFT
	},
	.rQueueEmpty = {
		WF_PLE_TOP_QUEUE_EMPTY_ADDR,
		0,
		0
	},
	.rQueueEmptyAllAcEmpty = {
		0,
		WF_PLE_TOP_QUEUE_EMPTY_ALL_AC_EMPTY_MASK,
		0
	},
	.rStationPause0 = {
		WF_PLE_TOP_STATION_PAUSE0_ADDR,
		0,
		0
	},
	.rTxdQueueEmpty = {
		WF_PLE_TOP_TXD_QUEUE_EMPTY_ADDR,
		0,
		0
	},
	.rToN9IntToggle = {
		WF_PLE_TOP_TO_N9_INT_TOGGLE_ADDR,
		WF_PLE_TOP_TO_N9_INT_TOGGLE_MASK,
		WF_PLE_TOP_TO_N9_INT_TOGGLE_SHFT
	},
};

struct PSE_TOP_CR rSoc5_0_PseTopCr = {
	.rFlQueCtrl0 = {
		WF_PSE_TOP_FL_QUE_CTRL_0_ADDR,
		0,
		0
	},
	.rFlQueCtrl0Execute = {
		0,
		WF_PSE_TOP_FL_QUE_CTRL_0_EXECUTE_MASK,
		0
	},
	.rFlQueCtrl0QBufPid = {
		0,
		0,
		WF_PSE_TOP_FL_QUE_CTRL_0_Q_BUF_PID_SHFT
	},
	.rFlQueCtrl0QBufQid = {
		0,
		0,
		WF_PSE_TOP_FL_QUE_CTRL_0_Q_BUF_QID_SHFT
	},
	.rFlQueCtrl2 = {
		WF_PSE_TOP_FL_QUE_CTRL_2_ADDR,
		0,
		0
	},
	.rFlQueCtrl2QueueHeadFid = {
		0,
		WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_MASK,
		WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_HEAD_FID_SHFT
	},
	.rFlQueCtrl2QueueTailFid = {
		0,
		WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_MASK,
		WF_PSE_TOP_FL_QUE_CTRL_2_QUEUE_TAIL_FID_SHFT
	},
	.rFlQueCtrl3 = {
		WF_PSE_TOP_FL_QUE_CTRL_3_ADDR,
		0,
		0
	},
	.rFlQueCtrl3QueuePktNum = {
		0,
		WF_PSE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_MASK,
		WF_PSE_TOP_FL_QUE_CTRL_3_QUEUE_PKT_NUM_SHFT
	},
	.rFreepgCnt = {
		WF_PSE_TOP_FREEPG_CNT_ADDR,
		0,
		0
	},
	.rFreepgCntFfaCnt = {
		0,
		WF_PSE_TOP_FREEPG_CNT_FFA_CNT_MASK,
		WF_PSE_TOP_FREEPG_CNT_FFA_CNT_SHFT
	},
	.rFreepgCntFreepgCnt = {
		0,
		WF_PSE_TOP_FREEPG_CNT_FREEPG_CNT_MASK,
		WF_PSE_TOP_FREEPG_CNT_FREEPG_CNT_SHFT
	},
	.rFreepgHeadTail = {
		WF_PSE_TOP_FREEPG_HEAD_TAIL_ADDR,
		0,
		0
	},
	.rFreepgHeadTailFreepgHead = {
		0,
		WF_PSE_TOP_FREEPG_HEAD_TAIL_FREEPG_HEAD_MASK,
		WF_PSE_TOP_FREEPG_HEAD_TAIL_FREEPG_HEAD_SHFT
	},
	.rFreepgHeadTailFreepgTail = {
		0,
		WF_PSE_TOP_FREEPG_HEAD_TAIL_FREEPG_TAIL_MASK,
		WF_PSE_TOP_FREEPG_HEAD_TAIL_FREEPG_TAIL_SHFT
	},
	.rFsmPeekCr00 = {
		WF_PSE_TOP_PSE_SEEK_CR_00_ADDR,
		0,
		0
	},
	.rFsmPeekCr01 = {
		WF_PSE_TOP_PSE_SEEK_CR_01_ADDR,
		0,
		0
	},
	.rFsmPeekCr02 = {
		WF_PSE_TOP_PSE_SEEK_CR_02_ADDR,
		0,
		0
	},
	.rFsmPeekCr03 = {
		WF_PSE_TOP_PSE_SEEK_CR_03_ADDR,
		0,
		0
	},
	.rFsmPeekCr04 = {
		WF_PSE_TOP_PSE_SEEK_CR_04_ADDR,
		0,
		0
	},
	.rFsmPeekCr05 = {
		WF_PSE_TOP_PSE_SEEK_CR_05_ADDR,
		0,
		0
	},
	.rFsmPeekCr06 = {
		WF_PSE_TOP_PSE_SEEK_CR_06_ADDR,
		0,
		0
	},
	.rFsmPeekCr07 = {
		WF_PSE_TOP_PSE_SEEK_CR_07_ADDR,
		0,
		0
	},
	.rFsmPeekCr08 = {
		WF_PSE_TOP_PSE_SEEK_CR_08_ADDR,
		0,
		0
	},
	.rFsmPeekCr09 = {
		WF_PSE_TOP_PSE_SEEK_CR_09_ADDR,
		0,
		0
	},
	.rHif0PgInfoHif0RsvCnt = {
		0,
		WF_PSE_TOP_HIF0_PG_INFO_HIF0_RSV_CNT_MASK,
		WF_PSE_TOP_HIF0_PG_INFO_HIF0_RSV_CNT_SHFT
	},
	.rHif0PgInfoHif0SrcCnt = {
		0,
		WF_PSE_TOP_HIF0_PG_INFO_HIF0_SRC_CNT_MASK,
		WF_PSE_TOP_HIF0_PG_INFO_HIF0_SRC_CNT_SHFT
	},
	.rIntN9Sts = {
		WF_PSE_TOP_INT_N9_STS_ADDR,
		0,
		0
	},
	.rIntN9Err1Sts = {
		WF_PSE_TOP_INT_N9_ERR1_STS_ADDR,
		0,
		0
	},
	.rIntN9ErrSts = {
		WF_PSE_TOP_INT_N9_ERR_STS_ADDR,
		0,
		0
	},
	.rPbufCtrl = {
		WF_PSE_TOP_PBUF_CTRL_ADDR,
		0,
		0
	},
	.rPbufCtrlPageSizeCfg = {
		0,
		WF_PSE_TOP_PBUF_CTRL_PAGE_SIZE_CFG_MASK,
		WF_PSE_TOP_PBUF_CTRL_PAGE_SIZE_CFG_SHFT
	},
	.rPbufCtrlPbufOffset = {
		0,
		WF_PSE_TOP_PBUF_CTRL_PBUF_OFFSET_MASK,
		WF_PSE_TOP_PBUF_CTRL_PBUF_OFFSET_SHFT
	},
	.rPbufCtrlTotalPageNum = {
		0,
		WF_PSE_TOP_PBUF_CTRL_TOTAL_PAGE_NUM_MASK,
		WF_PSE_TOP_PBUF_CTRL_TOTAL_PAGE_NUM_SHFT
	},
	.rPgHif0GroupHif0MaxQuota = {
		0,
		WF_PSE_TOP_PG_HIF0_GROUP_HIF0_MAX_QUOTA_MASK,
		WF_PSE_TOP_PG_HIF0_GROUP_HIF0_MAX_QUOTA_SHFT
	},
	.rPgHif0GroupHif0MinQuota = {
		0,
		WF_PSE_TOP_PG_HIF0_GROUP_HIF0_MIN_QUOTA_MASK,
		WF_PSE_TOP_PG_HIF0_GROUP_HIF0_MIN_QUOTA_SHFT
	},
	.rQueueEmpty = {
		WF_PSE_TOP_QUEUE_EMPTY_ADDR,
		WF_PSE_TOP_QUEUE_EMPTY_MASK_ADDR,
		0
	},
	.rQueueEmptyCpuQ0Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_CPU_Q0_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_CPU_Q0_EMPTY_SHFT
	},
	.rQueueEmptyCpuQ1Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_CPU_Q1_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_CPU_Q1_EMPTY_SHFT
	},
	.rQueueEmptyCpuQ2Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_CPU_Q2_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_CPU_Q2_EMPTY_SHFT
	},
	.rQueueEmptyCpuQ3Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_CPU_Q3_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_CPU_Q3_EMPTY_SHFT
	},
	.rQueueEmptyHif0Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_0_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_0_EMPTY_SHFT
	},
	.rQueueEmptyHif10Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_10_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_10_EMPTY_SHFT
	},
	.rQueueEmptyHif11Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_11_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_11_EMPTY_SHFT
	},
	.rQueueEmptyHif1Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_1_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_1_EMPTY_SHFT
	},
	.rQueueEmptyHif2Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_2_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_2_EMPTY_SHFT
	},
	.rQueueEmptyHif3Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_3_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_3_EMPTY_SHFT
	},
	.rQueueEmptyHif4Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_4_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_4_EMPTY_SHFT
	},
	.rQueueEmptyHif5Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_5_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_5_EMPTY_SHFT
	},
	.rQueueEmptyHif6Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_6_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_6_EMPTY_SHFT
	},
	.rQueueEmptyHif7Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_7_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_7_EMPTY_SHFT
	},
	.rQueueEmptyHif8Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_8_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_8_EMPTY_SHFT
	},
	.rQueueEmptyHif9Empty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_9_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_HIF_9_EMPTY_SHFT
	},
	.rQueueEmptyLmacTxQueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_LMAC_TX_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_LMAC_TX_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptyMdpRxQueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_RX_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_RX_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptyMdpRxioc1QueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_RXIOC1_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_RXIOC1_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptyMdpRxiocQueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_RXIOC_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_RXIOC_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptyMdpTx1QueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_TX1_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_TX1_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptyMdpTxQueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_TX_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_TX_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptyMdpTxioc1QueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_TXIOC1_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_TXIOC1_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptyMdpTxiocQueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_TXIOC_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_MDP_TXIOC_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptyRlsQEmtpy = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_RLS_Q_EMTPY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_RLS_Q_EMTPY_SHFT
	},
	.rQueueEmptySecRxQueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_SEC_RX_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_SEC_RX_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptySecTx1QueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_SEC_TX1_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_SEC_TX1_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptySecTxQueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_SEC_TX_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_SEC_TX_QUEUE_EMPTY_SHFT
	},
	.rQueueEmptySfdParkQueueEmpty = {
		0,
		WF_PSE_TOP_QUEUE_EMPTY_SFD_PARK_QUEUE_EMPTY_MASK,
		WF_PSE_TOP_QUEUE_EMPTY_SFD_PARK_QUEUE_EMPTY_SHFT
	},
};

struct PP_TOP_CR rSoc5_0_PpTopCr = {
	.rDbgCtrl = {
		WF_PP_TOP_DBG_CTRL_ADDR,
		0,
		0
	},
	.rDbgCs0 = {
		WF_PP_TOP_DBG_CS_0_ADDR,
		0,
		0
	},
	.rDbgCs1 = {
		WF_PP_TOP_DBG_CS_1_ADDR,
		0,
		0
	},
	.rDbgCs2 = {
		WF_PP_TOP_DBG_CS_2_ADDR,
		0,
		0
	},
	.rDbgCs3 = {
		0,
		0,
		0
	},
	.rDbgCs4 = {
		0,
		0,
		0
	},
	.rDbgCs5 = {
		0,
		0,
		0
	},
};

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

void soc5_0_show_wfdma_dbg_probe_info(IN struct ADAPTER *prAdapter,
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{
	uint32_t dbg_cr_idx[] = {0x0, 0x1, 0x31, 0x2D, 0x2, 0x3, 0x54, 0x4,
		0x29, 0x30, 0x5, 0x7, 0xA, 0xB, 0xC};
	uint32_t i = 0, u4DbgIdxAddr = 0, u4DbgProbeAddr = 0, u4DbgIdxValue = 0,
		u4DbgProbeValue = 0;

	if (!prAdapter)
		return;

	if (enum_wfdma_type == WFDMA_TYPE_HOST) {
		u4DbgIdxAddr = WF_WFDMA_HOST_DMA0_WPDMA_DBG_IDX_ADDR;
		u4DbgProbeAddr = WF_WFDMA_HOST_DMA0_WPDMA_DBG_PROBE_ADDR;
	} else {
		u4DbgIdxAddr = WF_WFDMA_MCU_DMA0_WPDMA_DBG_IDX_ADDR;
		u4DbgProbeAddr = WF_WFDMA_MCU_DMA0_WPDMA_DBG_PROBE_ADDR;
	}

	for (i = 0; i < ARRAY_SIZE(dbg_cr_idx); i++) {
		u4DbgIdxValue = 0x100 + dbg_cr_idx[i];
		HAL_MCR_WR(prAdapter, u4DbgIdxAddr, u4DbgIdxValue);
		HAL_MCR_RD(prAdapter, u4DbgProbeAddr, &u4DbgProbeValue);
		DBGLOG(HAL, INFO, "\t Write(0x%2x) DBG_PROBE[0x%X]=0x%08X\n",
			u4DbgIdxValue, u4DbgProbeAddr, u4DbgProbeValue);
	}
}

void soc5_0_show_wfdma_wrapper_info(IN struct ADAPTER *prAdapter,
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{
	uint32_t u4DmaCfgCr = 0;
	uint32_t u4RegValue = 0;

	if (!prAdapter)
		return;

	if (enum_wfdma_type == WFDMA_TYPE_HOST) {
		u4DmaCfgCr = 0x7c027044;
		HAL_MCR_RD(prAdapter, u4DmaCfgCr, &u4RegValue);
		DBGLOG(INIT, INFO, "WFDMA_HIF_BUSY(0x%08x): 0x%08x\n",
				u4DmaCfgCr,
				u4RegValue);

		u4DmaCfgCr = 0x7c027050;
		HAL_MCR_RD(prAdapter, u4DmaCfgCr, &u4RegValue);
		DBGLOG(INIT, INFO, "WFDMA_AXI_SLPPROT_CTRL(0x%08x): 0x%08x\n",
				u4DmaCfgCr,
				u4RegValue);

		u4DmaCfgCr = 0x7c027078;
		HAL_MCR_RD(prAdapter, u4DmaCfgCr, &u4RegValue);
		DBGLOG(INIT, INFO, "WFDMA_AXI_SLPPROT0_CTRL(0x%08x): 0x%08x\n",
				u4DmaCfgCr,
				u4RegValue);
	} else {
		u4DmaCfgCr = 0x57000044;
		HAL_MCR_RD(prAdapter, u4DmaCfgCr, &u4RegValue);
		DBGLOG(INIT, INFO, "WFDMA_HIF_BUSY(0x%08x): 0x%08x\n",
				u4DmaCfgCr,
				u4RegValue);

		u4DmaCfgCr = 0x57000050;
		HAL_MCR_RD(prAdapter, u4DmaCfgCr, &u4RegValue);
		DBGLOG(INIT, INFO, "WFDMA_AXI_SLPPROT_CTRL(0x%08x): 0x%08x\n",
				u4DmaCfgCr,
				u4RegValue);

		u4DmaCfgCr = 0x57000078;
		HAL_MCR_RD(prAdapter, u4DmaCfgCr, &u4RegValue);
		DBGLOG(INIT, INFO, "WFDMA_AXI_SLPPROT0_CTRL(0x%08x): 0x%08x\n",
				u4DmaCfgCr,
				u4RegValue);
	}
}

void soc5_0_dump_mac_info(IN struct ADAPTER *prAdapter)
{
#define BUF_SIZE 1024
#define CR_COUNT 13
#define LOOP_COUNT 30


	uint32_t i = 0, j = 0, pos = 0;
	uint32_t value = 0;
	uint32_t cr_band0[] = {
			/* #define BN0_WF_MIB_TOP_M0SDR6_ADDR */
			0x820ED020,
			/* #define BN0_WF_TMAC_TOP_DBGR0_ADDR */
			0x820E4360,
			/* #define BN0_WF_TMAC_TOP_DBGR2_ADDR */
			0x820E4368,
			/* #define BN0_WF_AGG_TOP_SMCR0_ADDR */
			0x820E22F0,
			/* #define BN0_WF_AGG_TOP_SMCR1_ADDR */
			0x820E22F4,
			/* #define BN0_WF_AGG_TOP_SMCR2_ADDR */
			0x820E22F8,
			/* #define BN0_WF_AGG_TOP_SMCR3_ADDR */
			0x820E22FC,
			/* #define BN0_WF_ARB_TOP_BFCR_ADDR */
			0x820E3150,
			/* #define WF_PLE_TOP_TXD_QUEUE_EMPTY_ADDR */
			0x820C0364,
			/* #define WF_PLE_TOP_HIF_PG_INFO_ADDR */
			0x820C0388,
			/* #define WF_PLE_TOP_HIF_WMTXD_PG_INFO_ADDR */
			0x820C038C,
			/* #define WF_PLE_TOP_CPU_PG_INFO_ADDR */
			0x820C0394,
			/* #define BN0_WF_CFG_TOP_DBG0_ADDR */
			0x820E0104
	};
	uint32_t cr_band1[] = {
			/* #define BN1_WF_MIB_TOP_M0SDR6_ADDR */
			0x820FD020,
			/* #define BN1_WF_TMAC_TOP_DBGR0_ADDR */
			0x820F4360,
			/* #define BN1_WF_TMAC_TOP_DBGR1_ADDR */
			0x820F4368,
			/* #define BN1_WF_AGG_TOP_SMCR0_ADDR */
			0x820F22F0,
			/* #define BN1_WF_AGG_TOP_SMCR1_ADDR */
			0x820F22F4,
			/* #define BN1_WF_AGG_TOP_SMCR2_ADDR */
			0x820F22F8,
			/* #define BN1_WF_AGG_TOP_SMCR3_ADDR */
			0x820F22FC,
			/* #define BN1_WF_ARB_TOP_BFCR_ADDR */
			0x820F3150,
			/* #define WF_PLE_TOP_TXD_QUEUE_EMPTY_ADDR */
			0x820C0364,
			/* #define WF_PLE_TOP_HIF_PG_INFO_ADDR */
			0x820C0388,
			/* #define WF_PLE_TOP_HIF_WMTXD_PG_INFO_ADDR */
			0x820C038C,
			/* #define WF_PLE_TOP_CPU_PG_INFO_ADDR */
			0x820C0394,
			/* #define BN1_WF_CFG_TOP_DBG0_ADDR */
			0x820F0104
	};

	char *buf = (char *) kalMemAlloc(BUF_SIZE, VIR_MEM_TYPE);

	DBGLOG(HAL, INFO, "Dump for band0\n");
	HAL_MCR_WR(prAdapter, 0x7C060390, 0x1F);
	HAL_MCR_WR(prAdapter, 0x7C060394, 0x07070707);
	HAL_MCR_WR(prAdapter, 0x7C060398, 0x0A0A0B09);

	/* #define WF_LMACON_CFG_TOP_LOCGCR0_ADDR */
	HAL_MCR_RD(prAdapter, 0x820D0000, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820D0000 = 0x%08x\n", value);

	/* #define BN0_WF_ARB_TOP_SCR_ADDR */
	HAL_MCR_RD(prAdapter, 0x820E3000, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820E3000 = 0x%08x\n", value);

	/* #define WF_PLE_TOP_INT_N9_ERR_STS_ADDR */
	HAL_MCR_RD(prAdapter, 0x820C0304, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C0304 = 0x%08x\n", value);

	/* #define WF_PLE_TOP_INT_N9_ERR_STS_1_ADDR */
	HAL_MCR_RD(prAdapter, 0x820C0308, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C0308 = 0x%08x\n", value);

	/* #define WF_PSE_TOP_INT_N9_ERR_STS_ADDR */
	HAL_MCR_RD(prAdapter, 0x820C8034, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C8034 = 0x%08x\n", value);

	/* #define WF_PSE_TOP_INT_N9_ERR1_STS_ADDR */
	HAL_MCR_RD(prAdapter, 0x820C8038, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C8038 = 0x%08x\n", value);

	/* Band 0 TXV_C and TXV_P */
	/* #define BN0_WF_TMAC_TOP_TXV0_ADDR */
	/* #define BN0_WF_TMAC_TOP_TXV11_ADDR */
	for (i = 0x820E4378; i < 0x820E43A8; i += 4) {
		HAL_MCR_RD(prAdapter, i, &value);
		DBGLOG(HAL, INFO, "Dump CR: 0x%08x = 0x%08x\n", i, value);
		kalMdelay(1);
	}

	/* #define BN0_WF_AGG_TOP_PCR0_ADDR */
	HAL_MCR_RD(prAdapter, 0x820E2040, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820E2040 = 0x%08x\n", value);

	if (buf) {
		kalMemZero(buf, BUF_SIZE);
		for (i = 0; i < LOOP_COUNT; i++) {
			for (j = 0; j < CR_COUNT; j++) {
				HAL_MCR_RD(prAdapter, cr_band0[j], &value);
				pos += kalSnprintf(buf + pos, 25,
					"0x%08x = 0x%08x%s", cr_band0[j], value,
					j == CR_COUNT - 1 ? ";" : ",");
			}
			DBGLOG(HAL, INFO, "Dump CR: %s\n", buf);
			pos = 0;
			kalMdelay(1);
		}
	}

	DBGLOG(HAL, INFO, "Dump for band1\n");
	HAL_MCR_WR(prAdapter, 0x7C06039C, 0x1F);
	HAL_MCR_WR(prAdapter, 0x7C0603A0, 0x07070707);
	HAL_MCR_WR(prAdapter, 0x7C0603A4, 0x0A0A0B09);

	/* #define WF_LMACON_CFG_TOP_LOCGCR0_ADDR */
	HAL_MCR_RD(prAdapter, 0x820D0000, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820D0000 = 0x%08x\n", value);

	/* #define BN1_WF_ARB_TOP_SCR_ADDR */
	HAL_MCR_RD(prAdapter, 0x820F3000, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820F3000 = 0x%08x\n", value);

	/* #define WF_PLE_TOP_INT_N9_ERR_STS_ADDR */
	HAL_MCR_RD(prAdapter, 0x820C0304, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C0304 = 0x%08x\n", value);

	/* #define WF_PLE_TOP_INT_N9_ERR_STS_1_ADDR */
	HAL_MCR_RD(prAdapter, 0x820C0308, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C0308 = 0x%08x\n", value);

	/* #define WF_PSE_TOP_INT_N9_ERR_STS_ADDR */
	HAL_MCR_RD(prAdapter, 0x820C8034, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C8034 = 0x%08x\n", value);

	/* #define WF_PSE_TOP_INT_N9_ERR1_STS_ADDR */
	HAL_MCR_RD(prAdapter, 0x820C8038, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C8038 = 0x%08x\n", value);

	/* Band 1 TXV_C and TXV_P */
	/* #define BN1_WF_TMAC_TOP_TXV0_ADDR */
	/* #define BN1_WF_TMAC_TOP_TXV11_ADDR */
	for (i = 0x820F4378; i < 0x820F43A8; i += 4) {
		HAL_MCR_RD(prAdapter, i, &value);
		DBGLOG(HAL, INFO, "Dump CR: 0x%08x = 0x%08x\n", i, value);
		kalMdelay(1);
	}

	/* #define BN1_WF_AGG_TOP_PCR0_ADDR */
	HAL_MCR_RD(prAdapter, 0x820F2040, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820F2040 = 0x%08x\n", value);

	if (buf) {
		kalMemZero(buf, BUF_SIZE);
		for (i = 0; i < LOOP_COUNT; i++) {
			for (j = 0; j < CR_COUNT; j++) {
				HAL_MCR_RD(prAdapter, cr_band1[j], &value);
				pos += kalSnprintf(buf + pos, 25,
					"0x%08x = 0x%08x%s", cr_band1[j], value,
					j == CR_COUNT - 1 ? ";" : ",");
			}
			DBGLOG(HAL, INFO, "Dump CR: %s\n", buf);
			pos = 0;
			kalMdelay(1);
		}
	}

	if (buf)
		kalMemFree(buf, VIR_MEM_TYPE, BUF_SIZE);
}

#ifdef CFG_SUPPORT_LINK_QUALITY_MONITOR
int soc5_0_get_rx_rate_info(IN struct ADAPTER *prAdapter,
		IN uint8_t ucBssIdx,
		OUT uint32_t *pu4Rate, OUT uint32_t *pu4Nss,
		OUT uint32_t *pu4RxMode, OUT uint32_t *pu4FrMode,
		OUT uint32_t *pu4Sgi)
{
	struct STA_RECORD *prStaRec;
	uint32_t rxmode = 0, rate = 0, frmode = 0, sgi = 0, nsts = 0;
	uint32_t stbc = 0, nss = 0;
	uint32_t u4RxVector0 = 0;
	uint8_t ucWlanIdx, ucStaIdx;

	if ((!pu4Rate) || (!pu4Nss) || (!pu4RxMode) || (!pu4FrMode) ||
		(!pu4Sgi))
		return -1;

	prStaRec = aisGetStaRecOfAP(prAdapter, ucBssIdx);
	if (prStaRec) {
		ucWlanIdx = prStaRec->ucWlanIndex;
	} else {
		DBGLOG(SW4, ERROR, "prStaRecOfAP is null\n");
		return -1;
	}

	if (wlanGetStaIdxByWlanIdx(prAdapter, ucWlanIdx, &ucStaIdx) ==
		WLAN_STATUS_SUCCESS) {
		u4RxVector0 = prAdapter->arStaRec[ucStaIdx].u4RxVector0;
		if (u4RxVector0 == 0) {
			DBGLOG(SW4, WARN, "u4RxVector0 is 0\n");
			return -1;
		}
	} else {
		DBGLOG(SW4, ERROR, "wlanGetStaIdxByWlanIdx fail\n");
		return -1;
	}

	/* P-RXV1 */
	rate = (u4RxVector0 & SOC5_0_RX_VT_RX_RATE_MASK)
				>> SOC5_0_RX_VT_RX_RATE_OFFSET;
	nsts = ((u4RxVector0 & SOC5_0_RX_VT_NSTS_MASK)
				>> SOC5_0_RX_VT_NSTS_OFFSET);
	/* C-B-0 */
	rxmode = (u4RxVector0 & SOC5_0_RX_VT_TXMODE_MASK)
				>> SOC5_0_RX_VT_TXMODE_OFFSET;
	frmode = (u4RxVector0 & SOC5_0_RX_VT_FR_MODE_MASK)
				>> SOC5_0_RX_VT_FR_MODE_OFFSET;
	sgi = (u4RxVector0 & SOC5_0_RX_VT_GI_MASK)
				>> SOC5_0_RX_VT_GI_OFFSET;
	stbc = (u4RxVector0 & SOC5_0_RX_VT_STBC_MASK)
				>> SOC5_0_RX_VT_STBC_OFFSET;

	nsts += 1;
	if (nsts == 1)
		nss = nsts;
	else
		nss = stbc ? (nsts >> 1) : nsts;

	if (frmode >= 4) {
		DBGLOG(SW4, ERROR, "frmode error: %u\n", frmode);
		return -1;
	}

	*pu4Rate = rate;
	*pu4Nss = nss;
	*pu4RxMode = rxmode;
	*pu4FrMode = frmode;
	*pu4Sgi = sgi;

	DBGLOG(SW4, TRACE,
		   "rxvec0=[0x%x] rxmode=[%u], rate=[%u], bw=[%u], sgi=[%u], nss=[%u]\n",
		   u4RxVector0, rxmode, rate, frmode, sgi, nss
	);

	return 0;
}
#endif


void soc5_0_get_rx_link_stats(IN struct ADAPTER *prAdapter,
	IN struct SW_RFB *prSwRfb, IN uint32_t u4RxVector0)
{
#if CFG_SUPPORT_LLS
	static const uint8_t TX_MODE_2_LLS_MODE[] = {
		LLS_MODE_CCK,
		LLS_MODE_OFDM,
		LLS_MODE_HT,
		LLS_MODE_HT,
		LLS_MODE_VHT,
		LLS_MODE_RESERVED,
		LLS_MODE_RESERVED,
		LLS_MODE_RESERVED,
		LLS_MODE_HE,
		LLS_MODE_HE,
		LLS_MODE_HE,
		LLS_MODE_HE,
		LLS_MODE_RESERVED,
		LLS_MODE_RESERVED,
		LLS_MODE_RESERVED,
		LLS_MODE_RESERVED
	};
	static const uint8_t OFDM_RATE[STATS_LLS_OFDM_NUM] = {
						6, 4, 2, 0, 7, 5, 3, 1};
		/* report &= 7:  3  7  2   6   1   5   0    4 */
		/* Mbps       : 6M 9M 12M 18M 24M 36M 48M 54M */
		/* in 0.5 Mbps: 12 18 24  36  48  72  96  108 */
		/* Save format:  0  1  2   3   4   5   6    7 */
	struct STATS_LLS_WIFI_RATE rate = {0};
	struct STA_RECORD *prStaRec;

	if (prAdapter->rWifiVar.fgLinkStatsDump)
		DBGLOG(RX, INFO, "RXV: pmbl=%u nsts=%u stbc=%u bw=%u mcs=%u",
			RXV_GET_TXMODE(u4RxVector0),
			RXV_GET_RX_NSTS(u4RxVector0),
			RXV_GET_STBC(u4RxVector0),
			RXV_GET_FR_MODE(u4RxVector0),
			RXV_GET_RX_RATE(u4RxVector0));

	if (!(prSwRfb->ucPayloadFormat == RX_PAYLOAD_FORMAT_MSDU ||
		prSwRfb->ucPayloadFormat == RX_PAYLOAD_FORMAT_FIRST_SUB_AMSDU))
		return;

	rate.preamble = TX_MODE_2_LLS_MODE[RXV_GET_TXMODE(u4RxVector0)];

	if (rate.preamble == LLS_MODE_RESERVED)
		return;

	rate.bw = RXV_GET_FR_MODE(u4RxVector0);
	rate.nss = RXV_GET_RX_NSTS(u4RxVector0);
	if (rate.preamble >= LLS_MODE_VHT) {
		if (RXV_GET_STBC(u4RxVector0))
			rate.nss /= 2;
	}

	rate.rateMcsIdx = RXV_GET_RX_RATE(u4RxVector0) & 0xF; /* 0 ~ 15 */

	if (rate.preamble == LLS_MODE_CCK)
		rate.rateMcsIdx &= 0x3; /* 0: 1M; 1: 2M; 2: 5.5M; 3: 11M  */
	else if (rate.preamble == LLS_MODE_OFDM)
		rate.rateMcsIdx = OFDM_RATE[rate.rateMcsIdx & 0x7];

	if (rate.nss >= STATS_LLS_MAX_NSS_NUM)
		goto wrong_rate;

	prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);
	if (!prStaRec) {
		DBGLOG(RX, WARN, "StaRec %u not found", prSwRfb->ucStaRecIdx);
		goto wrong_rate;
	}

	if (rate.preamble == LLS_MODE_OFDM) {
		if (rate.rateMcsIdx >= STATS_LLS_OFDM_NUM)
			goto wrong_rate;
		prStaRec->u4RxMpduOFDM[0][0][rate.rateMcsIdx]++;
	} else if (rate.preamble == LLS_MODE_CCK) {
		if (rate.rateMcsIdx >= STATS_LLS_CCK_NUM)
			goto wrong_rate;
		prStaRec->u4RxMpduCCK[0][0][rate.rateMcsIdx]++;
	} else if (rate.preamble == LLS_MODE_HT) {
		if (rate.bw >= STATS_LLS_MAX_HT_BW_NUM ||
				rate.rateMcsIdx >= STATS_LLS_HT_NUM)
			goto wrong_rate;
		prStaRec->u4RxMpduHT[0][rate.bw][rate.rateMcsIdx]++;
	} else if (rate.preamble == LLS_MODE_VHT) {
		if (rate.bw >= STATS_LLS_MAX_VHT_BW_NUM ||
				rate.rateMcsIdx >= STATS_LLS_VHT_NUM)
			goto wrong_rate;
		prStaRec->u4RxMpduVHT[rate.nss][rate.bw][rate.rateMcsIdx]++;
	} else if (rate.preamble == LLS_MODE_HE) {
		if (rate.bw >= STATS_LLS_MAX_HE_BW_NUM ||
				rate.rateMcsIdx >= STATS_LLS_HE_NUM)
			goto wrong_rate;
		prStaRec->u4RxMpduHE[rate.nss][rate.bw][rate.rateMcsIdx]++;
	}

	if (prAdapter->rWifiVar.fgLinkStatsDump)
		DBGLOG(RX, INFO, "rate preamble=%u, nss=%u, bw=%u, mcsIdx=%u",
			rate.preamble, rate.nss, rate.bw, rate.rateMcsIdx);
	return;

wrong_rate:
	DBGLOG(RX, WARN, "Invalid rate preamble=%u, nss=%u, bw=%u, mcsIdx=%u",
			rate.preamble, rate.nss, rate.bw, rate.rateMcsIdx);
#endif
}

#endif /* SOC5_0 */
