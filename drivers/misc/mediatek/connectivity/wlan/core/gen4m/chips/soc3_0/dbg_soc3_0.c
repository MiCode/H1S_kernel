/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2016 MediaTek Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
/******************************************************************************
 *[File]             dbg_soc3_0.c
 *[Version]          v1.0
 *[Revision Date]    2019-09-09
 *[Author]
 *[Description]
 *    The program provides WIFI FALCON MAC Debug APIs
 *[Copyright]
 *    Copyright (C) 2015 MediaTek Incorporation. All Rights Reserved.
 ******************************************************************************/

#ifdef SOC3_0
/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "coda/soc3_0/wf_ple_top.h"
#include "coda/soc3_0/wf_pse_top.h"
#include "coda/soc3_0/wf_wfdma_host_dma0.h"
#include "coda/soc3_0/wf_wfdma_host_dma1.h"
#include "coda/soc3_0/wf_hif_dmashdl_top.h"
#include "coda/soc3_0/wf_wfdma_mcu_dma0.h"
#include "coda/soc3_0/wf_wfdma_mcu_dma1.h"
#include "precomp.h"
#include "mt_dmac.h"
#include "wf_ple.h"
#include "hal_dmashdl_soc3_0.h"
#include "soc3_0.h"
/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/* TODO : need to Merge related API with non-driver base */
#define WFSYS_SUPPORT_BUS_HANG_READ_FROM_DRIVER_BASE 1

#if WFSYS_SUPPORT_BUS_HANG_READ_FROM_DRIVER_BASE
/* define for read Driver Base CR */
#define CONNAC2X_MCU_WPDMA_0_DRIVER_BASE		0x18402000
#define CONNAC2X_MCU_WPDMA_1_DRIVER_BASE		0x18403000
#define CONNAC2X_HOST_WPDMA_0_DRIVER_BASE		0x18024000
#define CONNAC2X_HOST_WPDMA_1_DRIVER_BASE		0x18025000

#define CONNAC2X_HOST_EXT_CONN_HIF_WRAP_DRIVER_BASE	0x18027000
#define CONNAC2X_MCU_INT_CONN_HIF_WRAP_DRIVER_BASE	0x18405000

#define WF_WFDMA_HOST_DMA1_WPDMA_TX_RING0_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x300) /* 5300 */
#define WF_WFDMA_HOST_DMA1_WPDMA_TX_RING0_CTRL1_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x304) /* 5304 */
#define WF_WFDMA_HOST_DMA1_WPDMA_TX_RING1_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x310) /* 5310 */
#define WF_WFDMA_HOST_DMA1_WPDMA_TX_RING16_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x400) /* 5400 */
#define WF_WFDMA_HOST_DMA1_WPDMA_TX_RING8_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x380) /* 5380 */
#define WF_WFDMA_HOST_DMA1_WPDMA_TX_RING17_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x410) /* 5410 */
#define WF_WFDMA_HOST_DMA1_WPDMA_TX_RING18_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x420) /* 5420 */

#define WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_0_DRIVER_BASE + 0x500) /* 4500 */
#define WF_WFDMA_HOST_DMA0_WPDMA_RX_RING1_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_0_DRIVER_BASE + 0x510) /* 4510 */
#define WF_WFDMA_HOST_DMA0_WPDMA_RX_RING2_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_0_DRIVER_BASE + 0x520) /* 4520 */
#define WF_WFDMA_HOST_DMA0_WPDMA_RX_RING3_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_0_DRIVER_BASE + 0x530) /* 4530 */
#define WF_WFDMA_HOST_DMA1_WPDMA_RX_RING0_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x500) /* 5500 */
#define WF_WFDMA_HOST_DMA0_WPDMA_RX_RING4_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_0_DRIVER_BASE + 0x540) /* 4540 */
#define WF_WFDMA_HOST_DMA0_WPDMA_RX_RING5_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_0_DRIVER_BASE + 0x550) /* 4550 */
#define WF_WFDMA_HOST_DMA0_WPDMA_RX_RING6_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_0_DRIVER_BASE + 0x560) /* 4560 */
#define WF_WFDMA_HOST_DMA0_WPDMA_RX_RING7_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_0_DRIVER_BASE + 0x570) /* 4570 */
#define WF_WFDMA_HOST_DMA1_WPDMA_RX_RING1_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_HOST_WPDMA_1_DRIVER_BASE + 0x510) /* 5510 */

#define WF_WFDMA_MCU_DMA0_WPDMA_TX_RING0_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_MCU_WPDMA_0_DRIVER_BASE + 0x300)
#define WF_WFDMA_MCU_DMA1_WPDMA_TX_RING0_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_MCU_WPDMA_0_DRIVER_BASE + 0x300)
#define WF_WFDMA_MCU_DMA1_WPDMA_TX_RING1_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_MCU_WPDMA_1_DRIVER_BASE + 0x310)
#define WF_WFDMA_MCU_DMA0_WPDMA_RX_RING0_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_MCU_WPDMA_1_DRIVER_BASE + 0x500)
#define WF_WFDMA_MCU_DMA0_WPDMA_RX_RING1_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_MCU_WPDMA_0_DRIVER_BASE + 0x510)
#define WF_WFDMA_MCU_DMA1_WPDMA_RX_RING0_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_MCU_WPDMA_1_DRIVER_BASE + 0x500)
#define WF_WFDMA_MCU_DMA1_WPDMA_RX_RING1_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_MCU_WPDMA_1_DRIVER_BASE + 0x510)
#define WF_WFDMA_MCU_DMA1_WPDMA_RX_RING2_CTRL0_ADDR_DRIVER_BASE \
		(CONNAC2X_MCU_WPDMA_1_DRIVER_BASE + 0x520)

#endif /* WFSYS_SUPPORT_BUS_HANG_READ_FROM_DRIVER_BASE */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */
struct PLE_TOP_CR rSoc3_0_PleTopCr = {
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

struct PSE_TOP_CR rSoc3_0_PseTopCr = {
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

struct PP_TOP_CR rSoc3_0_PpTopCr = {
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

#if 0
static struct EMPTY_QUEUE_INFO ple_txcmd_queue_empty_info[] = {
	{"AC00Q", ENUM_UMAC_LMAC_PORT_2, 0x40},
	{"AC01Q", ENUM_UMAC_LMAC_PORT_2, 0x41},
	{"AC02Q", ENUM_UMAC_LMAC_PORT_2, 0x42},
	{"AC03Q", ENUM_UMAC_LMAC_PORT_2, 0x43},
	{"AC10Q", ENUM_UMAC_LMAC_PORT_2, 0x44},
	{"AC11Q", ENUM_UMAC_LMAC_PORT_2, 0x45},
	{"AC12Q", ENUM_UMAC_LMAC_PORT_2, 0x46},
	{"AC13Q", ENUM_UMAC_LMAC_PORT_2, 0x47},
	{"AC20Q", ENUM_UMAC_LMAC_PORT_2, 0x48},
	{"AC21Q", ENUM_UMAC_LMAC_PORT_2, 0x49},
	{"AC22Q", ENUM_UMAC_LMAC_PORT_2, 0x4a},
	{"AC23Q", ENUM_UMAC_LMAC_PORT_2, 0x4b},
	{"AC30Q", ENUM_UMAC_LMAC_PORT_2, 0x4c},
	{"AC31Q", ENUM_UMAC_LMAC_PORT_2, 0x4d},
	{"AC32Q", ENUM_UMAC_LMAC_PORT_2, 0x4e},
	{"AC33Q", ENUM_UMAC_LMAC_PORT_2, 0x4f},
	{"ALTX Q0", ENUM_UMAC_LMAC_PORT_2, 0x50},
	{"TF Q0", ENUM_UMAC_LMAC_PORT_2, 0x51},
	{"TWT TSF-TF Q0", ENUM_UMAC_LMAC_PORT_2, 0x52},
	{"TWT DL Q0", ENUM_UMAC_LMAC_PORT_2, 0x53},
	{"TWT UL Q0", ENUM_UMAC_LMAC_PORT_2, 0x54},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0},
	{NULL, 0, 0} };
#endif

#if WFSYS_SUPPORT_BUS_HANG_READ_FROM_DRIVER_BASE
struct wfdma_group_info wfmda_host_tx_group_driver_base[] = {
	{"P1T0:AP DATA0",
		WF_WFDMA_HOST_DMA1_WPDMA_TX_RING0_CTRL0_ADDR_DRIVER_BASE},
	{"P1T1:AP DATA1",
		WF_WFDMA_HOST_DMA1_WPDMA_TX_RING1_CTRL0_ADDR_DRIVER_BASE},
	{"P1T16:FWDL",
		WF_WFDMA_HOST_DMA1_WPDMA_TX_RING16_CTRL0_ADDR_DRIVER_BASE},
	{"P1T17:AP CMD",
		WF_WFDMA_HOST_DMA1_WPDMA_TX_RING17_CTRL0_ADDR_DRIVER_BASE},
	{"P1T8:MD DATA",
		WF_WFDMA_HOST_DMA1_WPDMA_TX_RING8_CTRL0_ADDR_DRIVER_BASE},
	{"P1T18:MD CMD",
		WF_WFDMA_HOST_DMA1_WPDMA_TX_RING18_CTRL0_ADDR_DRIVER_BASE},
};

struct wfdma_group_info wfmda_host_rx_group_driver_base[] = {
	{"P0R0:AP DATA0",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL0_ADDR_DRIVER_BASE},
	{"P0R1:AP DATA1",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING1_CTRL0_ADDR_DRIVER_BASE},
	{"P0R2:AP TDONE0",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING2_CTRL0_ADDR_DRIVER_BASE},
	{"P0R3:AP TDONE1",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING3_CTRL0_ADDR_DRIVER_BASE},
	{"P1R0:AP EVENT",
		WF_WFDMA_HOST_DMA1_WPDMA_RX_RING0_CTRL0_ADDR_DRIVER_BASE},
	{"P0R4:MD DATA0",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING4_CTRL0_ADDR_DRIVER_BASE},
	{"P0R5:MD DATA1",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING5_CTRL0_ADDR_DRIVER_BASE},
	{"P0R6:MD TDONE0",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING6_CTRL0_ADDR_DRIVER_BASE},
	{"P0R7:MD TDONE1",
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING7_CTRL0_ADDR_DRIVER_BASE},
	{"P1R1:MD EVENT",
		WF_WFDMA_HOST_DMA1_WPDMA_RX_RING1_CTRL0_ADDR_DRIVER_BASE},
};

struct wfdma_group_info wfmda_wm_tx_group_driver_base[] = {
	{"P0T0:DATA",
		WF_WFDMA_MCU_DMA0_WPDMA_TX_RING0_CTRL0_ADDR_DRIVER_BASE},
	{"P1T0:AP EVENT",
		WF_WFDMA_MCU_DMA1_WPDMA_TX_RING0_CTRL0_ADDR_DRIVER_BASE},
	{"P1T1:MD EVENT",
		WF_WFDMA_MCU_DMA1_WPDMA_TX_RING1_CTRL0_ADDR_DRIVER_BASE},
};

struct wfdma_group_info wfmda_wm_rx_group_driver_base[] = {
	{"P0R0:DATA",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING0_CTRL0_ADDR_DRIVER_BASE},
	{"P0R1:TXDONE",
		WF_WFDMA_MCU_DMA0_WPDMA_RX_RING1_CTRL0_ADDR_DRIVER_BASE},
	{"P1R0:FWDL",
		WF_WFDMA_MCU_DMA1_WPDMA_RX_RING0_CTRL0_ADDR_DRIVER_BASE},
	{"P1R1:AP CMD",
		WF_WFDMA_MCU_DMA1_WPDMA_RX_RING1_CTRL0_ADDR_DRIVER_BASE},
	{"P1R2:MD CMD",
		WF_WFDMA_MCU_DMA1_WPDMA_RX_RING2_CTRL0_ADDR_DRIVER_BASE},
};
#endif /* WFSYS_SUPPORT_BUS_HANG_READ_FROM_DRIVER_BASE */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

void soc3_0_show_wfdma_dbg_probe_info(IN struct ADAPTER *prAdapter,
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{
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

	for (i = 0; i <= 0x12; i++) {
		u4DbgIdxValue = 0x100 + i;
		HAL_MCR_WR(prAdapter, u4DbgIdxAddr, u4DbgIdxValue);
		HAL_MCR_RD(prAdapter, u4DbgProbeAddr, &u4DbgProbeValue);
		DBGLOG(HAL, INFO, "\t Write(0x%2x) DBG_PROBE[0x%X]=0x%08X\n",
			u4DbgIdxValue, u4DbgProbeAddr, u4DbgProbeValue);
	}
}

static void DumpPPDebugCr(struct ADAPTER *prAdapter)
{
	uint32_t ReadRegValue[4];
	uint32_t u4Value[4] = {0};

	/* 0x820CC0F0 : PP DBG_CTRL */
	ReadRegValue[0] = 0x820CC0F0;
	HAL_MCR_RD(prAdapter, ReadRegValue[0], &u4Value[0]);

	/* 0x820CC0F8 : PP DBG_CS0 */
	ReadRegValue[1] = 0x820CC0F8;
	HAL_MCR_RD(prAdapter, ReadRegValue[1], &u4Value[1]);

	/* 0x820CC0FC : PP DBG_CS1 */
	ReadRegValue[2] = 0x820CC0FC;
	HAL_MCR_RD(prAdapter, ReadRegValue[2], &u4Value[2]);

	/* 0x820CC100 : PP DBG_CS2 */
	ReadRegValue[3] = 0x820CC100;
	HAL_MCR_RD(prAdapter, ReadRegValue[3], &u4Value[3]);

	DBGLOG(HAL, INFO,
	"PP[0x%08x]=0x%08x,[0x%08x]=0x%08x,[0x%08x]=0x%08x,[0x%08x]=0x%08x,",
		ReadRegValue[0], u4Value[0],
		ReadRegValue[1], u4Value[1],
		ReadRegValue[2], u4Value[2],
		ReadRegValue[3], u4Value[3]);
}

static void dump_wfdma_dbg_value(
	IN struct ADAPTER *prAdapter,
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type,
	IN uint32_t wfdma_idx)
{
#define BUF_SIZE 1024

	uint32_t pdma_base_cr;
	uint32_t set_debug_flag_value;
	char *buf;
	uint32_t pos = 0;
	uint32_t set_debug_cr, get_debug_cr, get_debug_value = 0;

	if (enum_wfdma_type == WFDMA_TYPE_HOST) {
		if (wfdma_idx == 0)
			pdma_base_cr = CONNAC2X_HOST_WPDMA_0_BASE;
		else
			pdma_base_cr = CONNAC2X_HOST_WPDMA_1_BASE;
	} else {
		if (wfdma_idx == 0)
			pdma_base_cr = CONNAC2X_MCU_WPDMA_0_BASE;
		else
			pdma_base_cr = CONNAC2X_MCU_WPDMA_1_BASE;
	}

	buf = (char *) kalMemAlloc(BUF_SIZE, VIR_MEM_TYPE);
	if (!buf) {
		DBGLOG(HAL, ERROR, "Mem allocation failed.\n");
		return;
	}
	set_debug_cr = pdma_base_cr + 0x124;
	get_debug_cr = pdma_base_cr + 0x128;
	kalMemZero(buf, BUF_SIZE);
	pos += kalSnprintf(buf + pos, 50,
			"set_debug_cr:0x%08x get_debug_cr:0x%08x; ",
			set_debug_cr, get_debug_cr);
	for (set_debug_flag_value = 0x100; set_debug_flag_value <= 0x112;
			set_debug_flag_value++) {
		HAL_MCR_WR(prAdapter, set_debug_cr, set_debug_flag_value);
		HAL_MCR_RD(prAdapter, get_debug_cr, &get_debug_value);
		pos += kalSnprintf(buf + pos, 40, "Set:0x%03x, result=0x%08x%s",
			set_debug_flag_value,
			get_debug_value,
			set_debug_flag_value == 0x112 ? "\n" : "; ");
	}
	DBGLOG(HAL, INFO, "%s", buf);
	kalMemFree(buf, VIR_MEM_TYPE, BUF_SIZE);
}

void show_wfdma_dbg_flag_log(
	IN struct ADAPTER *prAdapter,
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{
	dump_wfdma_dbg_value(prAdapter, enum_wfdma_type, 0);
	dump_wfdma_dbg_value(prAdapter, enum_wfdma_type, 1);
}

void show_wfdma_dbg_log(
	IN struct ADAPTER *prAdapter,
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{
	/* Dump Host WFMDA info */
	DBGLOG(HAL, TRACE, "WFMDA Configuration:\n");
	connac2x_show_wfdma_interrupt_info(prAdapter, enum_wfdma_type, 2);
	connac2x_show_wfdma_glo_info(prAdapter, enum_wfdma_type, 2);
	connac2x_show_wfdma_ring_info(prAdapter, enum_wfdma_type);
}

void soc3_0_show_wfdma_info(IN struct ADAPTER *prAdapter)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	if (prSwWfdmaInfo->rOps.dumpDebugLog)
		prSwWfdmaInfo->rOps.dumpDebugLog(prAdapter->prGlueInfo);

	/* dump WFDMA info by host or WM*/
	show_wfdma_dbg_log(prAdapter, WFDMA_TYPE_HOST);
	show_wfdma_dbg_log(prAdapter, WFDMA_TYPE_WM);

	/* dump debug flag CR by host or WM*/
	show_wfdma_dbg_flag_log(prAdapter, WFDMA_TYPE_HOST);
	show_wfdma_dbg_flag_log(prAdapter, WFDMA_TYPE_WM);

	DumpPPDebugCr(prAdapter);
}

void soc3_0_show_wfdma_info_by_type(IN struct ADAPTER *prAdapter,
	IN bool bShowWFDMA_type)
{
	if (bShowWFDMA_type) {
		show_wfdma_dbg_log(prAdapter, WFDMA_TYPE_WM);
		show_wfdma_dbg_flag_log(prAdapter, WFDMA_TYPE_WM);
	} else {
		show_wfdma_dbg_log(prAdapter, WFDMA_TYPE_HOST);
		show_wfdma_dbg_flag_log(prAdapter, WFDMA_TYPE_HOST);
	}
}

void soc3_0_show_dmashdl_info(IN struct ADAPTER *prAdapter)
{
	uint32_t value = 0;
	uint8_t idx;
	uint32_t rsv_cnt = 0;
	uint32_t src_cnt = 0;
	uint32_t total_src_cnt = 0;
	uint32_t total_rsv_cnt = 0;
	uint32_t ffa_cnt = 0;
	uint32_t free_pg_cnt = 0;
	uint32_t ple_rpg_hif;
	uint32_t ple_upg_hif;
	uint8_t is_mismatch = FALSE;

	DBGLOG(HAL, INFO, "DMASHDL info:\n");

	asicConnac2xDmashdlGetRefill(prAdapter);
	asicConnac2xDmashdlGetPktMaxPage(prAdapter);

	HAL_MCR_RD(prAdapter, WF_HIF_DMASHDL_TOP_ERROR_FLAG_CTRL_ADDR, &value);
	DBGLOG(HAL, INFO, "DMASHDL ERR FLAG CTRL(0x%08x): 0x%08x\n",
		WF_HIF_DMASHDL_TOP_ERROR_FLAG_CTRL_ADDR, value);

	for (idx = 0; idx < ENUM_DMASHDL_GROUP_2; idx++) {
		DBGLOG(HAL, INFO, "Group %d info:\n", idx);
		asicConnac2xDmashdlGetGroupControl(prAdapter, idx);
		rsv_cnt = asicConnac2xDmashdlGetRsvCount(prAdapter, idx);
		src_cnt = asicConnac2xDmashdlGetSrcCount(prAdapter, idx);
		asicConnac2xDmashdlGetPKTCount(prAdapter, idx);
		total_src_cnt += src_cnt;
		total_rsv_cnt += rsv_cnt;
	}
	HAL_MCR_RD(prAdapter, WF_HIF_DMASHDL_TOP_STATUS_RD_ADDR, &value);
	ffa_cnt = (value & WF_HIF_DMASHDL_TOP_STATUS_RD_FFA_CNT_MASK) >>
		WF_HIF_DMASHDL_TOP_STATUS_RD_FFA_CNT_SHFT;
	free_pg_cnt = (value &
		WF_HIF_DMASHDL_TOP_STATUS_RD_FREE_PAGE_CNT_MASK) >>
		WF_HIF_DMASHDL_TOP_STATUS_RD_FREE_PAGE_CNT_SHFT;
	DBGLOG(HAL, INFO, "\tDMASHDL Status_RD(0x%08x): 0x%08x\n",
		WF_HIF_DMASHDL_TOP_STATUS_RD_ADDR, value);
	DBGLOG(HAL, INFO, "\tfree page cnt = 0x%03x, ffa cnt = 0x%03x\n",
		free_pg_cnt, ffa_cnt);

	DBGLOG(HAL, INFO, "\nDMASHDL Counter Check:\n");
	HAL_MCR_RD(prAdapter, WF_PLE_TOP_HIF_PG_INFO_ADDR, &value);
	ple_rpg_hif = (value & WF_PLE_TOP_HIF_PG_INFO_HIF_RSV_CNT_MASK) >>
		  WF_PLE_TOP_HIF_PG_INFO_HIF_RSV_CNT_SHFT;
	ple_upg_hif = (value & WF_PLE_TOP_HIF_PG_INFO_HIF_SRC_CNT_MASK) >>
		  WF_PLE_TOP_HIF_PG_INFO_HIF_SRC_CNT_SHFT;
	DBGLOG(HAL, INFO,
		"\tPLE:The used/reserved pages of PLE HIF group=0x%03x/0x%03x\n",
		 ple_upg_hif, ple_rpg_hif);
	DBGLOG(HAL, INFO,
		"\tDMASHDL:The total used pages of group0~14=0x%03x\n",
		total_src_cnt);

	if (ple_upg_hif != total_src_cnt) {
		DBGLOG(HAL, INFO,
			"\tPLE used pages & total used pages mismatch!\n");
		is_mismatch = TRUE;
	}

	DBGLOG(HAL, INFO,
		"\tThe total reserved pages of group0~14=0x%03x\n",
		total_rsv_cnt);
	DBGLOG(HAL, INFO,
		"\tThe total ffa pages of group0~14=0x%03x\n",
		ffa_cnt);
	DBGLOG(HAL, INFO,
		"\tThe total free pages of group0~14=0x%03x\n",
		free_pg_cnt);

	if (free_pg_cnt != total_rsv_cnt + ffa_cnt) {
		DBGLOG(HAL, INFO,
			"\tmismatch(total_rsv_cnt + ffa_cnt in DMASHDL)\n");
		is_mismatch = TRUE;
	}

	if (free_pg_cnt != ple_rpg_hif) {
		DBGLOG(HAL, INFO, "\tmismatch(reserved pages in PLE)\n");
		is_mismatch = TRUE;
	}


	if (!is_mismatch)
		DBGLOG(HAL, INFO, "DMASHDL: no counter mismatch\n");
}

void soc3_0_dump_mac_info(IN struct ADAPTER *prAdapter)
{
#define BUF_SIZE 1024
#define CR_COUNT 12
#define LOOP_COUNT 30

	uint32_t i = 0, j = 0, pos = 0;
	uint32_t value = 0;
	uint32_t cr_band0[] = {
			0x820ED020,
			0x820E4120,
			0x820E4128,
			0x820E22F0,
			0x820E22F4,
			0x820E22F8,
			0x820E22FC,
			0x820E3190,
			0x820C0220,
			0x820C0114,
			0x820C0154,
			0x820E0024
	};
	uint32_t cr_band1[] = {
			0x820FD020,
			0x820F4120,
			0x820F4128,
			0x820F22F0,
			0x820F22F4,
			0x820F22F8,
			0x820F22FC,
			0x820F3190,
			0x820C0220,
			0x820C0114,
			0x820C0154,
			0x820F0024
	};

	char *buf = (char *) kalMemAlloc(BUF_SIZE, VIR_MEM_TYPE);

	DBGLOG(HAL, INFO, "Dump for band0\n");
	HAL_MCR_WR(prAdapter, 0x7C006100, 0x1F);
	HAL_MCR_WR(prAdapter, 0x7C006104, 0x07070707);
	HAL_MCR_WR(prAdapter, 0x7C006108, 0x0D0D0C0C);
	HAL_MCR_RD(prAdapter, 0x820D0000, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820D0000 = 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, 0x820E3080, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820E3080 = 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, 0x820C0028, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C0028 = 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, 0x820C8028, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C8028 = 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, 0x820C8030, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C8030 = 0x%08x\n", value);
	/* Band 0 TXV_C and TXV_P */
	for (i = 0x820E412C; i < 0x820E4160; i += 4) {
		HAL_MCR_RD(prAdapter, i, &value);
		DBGLOG(HAL, INFO, "Dump CR: 0x%08x = 0x%08x\n", i, value);
		kalMdelay(1);
	}
	HAL_MCR_RD(prAdapter, 0x820E206C, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820E206C = 0x%08x\n", value);

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
	HAL_MCR_WR(prAdapter, 0x7C006400, 0x1F);
	HAL_MCR_WR(prAdapter, 0x7C006404, 0x07070707);
	HAL_MCR_WR(prAdapter, 0x7C006408, 0x0D0D0C0C);
	HAL_MCR_RD(prAdapter, 0x820D0000, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820D0000 = 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, 0x820F3080, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820F3080 = 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, 0x820C0028, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C0028 = 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, 0x820C8028, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C8028 = 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, 0x820C8030, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820C8030 = 0x%08x\n", value);
	/* Band 0 TXV_C and TXV_P */
	for (i = 0x820F412C; i < 0x820F4160; i += 4) {
		HAL_MCR_RD(prAdapter, i, &value);
		DBGLOG(HAL, INFO, "Dump CR: 0x%08x = 0x%08x\n", i, value);
		kalMdelay(1);
	}
	HAL_MCR_RD(prAdapter, 0x820F206C, &value);
	DBGLOG(HAL, INFO, "Dump CR: 0x820F206C = 0x%08x\n", value);

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

#if WFSYS_SUPPORT_BUS_HANG_READ_FROM_DRIVER_BASE
void show_wfdma_interrupt_info_without_adapter(
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{
	uint32_t idx;
	uint32_t u4hostBaseCrAddr = 0;
	uint32_t u4DmaCfgCrAddr = 0;
	uint32_t u4DmaCfgCrAddrByWFDMA[CONNAC2X_MAX_WFDMA_COUNT];
	uint32_t u4RegValue = 0;
	uint32_t u4RegValueByWFDMA[CONNAC2X_MAX_WFDMA_COUNT] = {0};

	/* Dump Interrupt Status info */
	if (enum_wfdma_type == WFDMA_TYPE_HOST) {
		/* Dump Global Status CR only in WFMDA HOST*/
		u4hostBaseCrAddr = CONNAC2X_HOST_EXT_CONN_HIF_WRAP_DRIVER_BASE;

		u4DmaCfgCrAddr = CONNAC2X_WPDMA_EXT_INT_STA(u4hostBaseCrAddr);
		wf_ioremap_read(u4DmaCfgCrAddr, &u4RegValue);
	}

	/* Dump PDMA Status CR */
	for (idx = 0; idx < CONNAC2X_MAX_WFDMA_COUNT; idx++) {

		if (enum_wfdma_type == WFDMA_TYPE_HOST)
			u4hostBaseCrAddr = idx ?
				CONNAC2X_HOST_WPDMA_1_DRIVER_BASE :
				CONNAC2X_HOST_WPDMA_0_DRIVER_BASE;
		else
			u4hostBaseCrAddr = idx ?
				CONNAC2X_MCU_WPDMA_1_DRIVER_BASE :
				CONNAC2X_MCU_WPDMA_0_DRIVER_BASE;

		u4DmaCfgCrAddrByWFDMA[idx] =
			CONNAC2X_WPDMA_INT_STA(u4hostBaseCrAddr);

		wf_ioremap_read(u4DmaCfgCrAddrByWFDMA[idx],
			&u4RegValueByWFDMA[idx]);
	}

	DBGLOG(INIT, INFO,
	"G_INT_S(0x%08x):0x%08x,W_%d(0x%08x):0x%08x, W_%d(0x%08x):0x%08x\n",
		u4DmaCfgCrAddr, u4RegValue,
		0, u4DmaCfgCrAddrByWFDMA[0], u4RegValueByWFDMA[0],
		1, u4DmaCfgCrAddrByWFDMA[1], u4RegValueByWFDMA[1]);

	/* Dump Interrupt Enable Info */
	if (enum_wfdma_type == WFDMA_TYPE_HOST) {

		/* Dump Global Enable CR */
		u4hostBaseCrAddr = CONNAC2X_HOST_EXT_CONN_HIF_WRAP_DRIVER_BASE;

		u4DmaCfgCrAddr = CONNAC2X_WPDMA_EXT_INT_MASK(u4hostBaseCrAddr);

		wf_ioremap_read(u4DmaCfgCrAddr, &u4RegValue);
	}

	/* Dump PDMA Enable CR */
	for (idx = 0; idx < CONNAC2X_MAX_WFDMA_COUNT; idx++) {

		if (enum_wfdma_type == WFDMA_TYPE_HOST)
			u4hostBaseCrAddr = idx ?
				CONNAC2X_HOST_WPDMA_1_DRIVER_BASE :
				CONNAC2X_HOST_WPDMA_0_DRIVER_BASE;
		else
			u4hostBaseCrAddr = idx ?
				CONNAC2X_MCU_WPDMA_1_DRIVER_BASE :
				CONNAC2X_MCU_WPDMA_0_DRIVER_BASE;

		u4DmaCfgCrAddrByWFDMA[idx] =
			CONNAC2X_WPDMA_INT_MASK(u4hostBaseCrAddr);

		wf_ioremap_read(u4DmaCfgCrAddrByWFDMA[idx],
			&u4RegValueByWFDMA[idx]);
	}

	DBGLOG(INIT, INFO,
	"G_INT_E(0x%08x):0x%08x,W_%d(0x%08x):0x%08x, W_%d(0x%08x):0x%08x\n",
		u4DmaCfgCrAddr, u4RegValue,
		0, u4DmaCfgCrAddrByWFDMA[0], u4RegValueByWFDMA[0],
		1, u4DmaCfgCrAddrByWFDMA[1], u4RegValueByWFDMA[1]);
}

void show_wfdma_glo_info_without_adapter(
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{
	uint32_t idx;
	uint32_t u4hostBaseCrAddr = 0;
	uint32_t u4DmaCfgCrAddr = 0;
	union WPDMA_GLO_CFG_STRUCT GloCfgValue;

	for (idx = 0; idx < CONNAC2X_MAX_WFDMA_COUNT; idx++) {

		if (enum_wfdma_type == WFDMA_TYPE_HOST)
			u4hostBaseCrAddr = idx ?
			CONNAC2X_HOST_WPDMA_1_DRIVER_BASE :
			CONNAC2X_HOST_WPDMA_0_DRIVER_BASE;
		else
			u4hostBaseCrAddr = idx ?
			CONNAC2X_MCU_WPDMA_1_DRIVER_BASE :
			CONNAC2X_MCU_WPDMA_0_DRIVER_BASE;

		u4DmaCfgCrAddr = CONNAC2X_WPDMA_GLO_CFG(u4hostBaseCrAddr);

		wf_ioremap_read(u4DmaCfgCrAddr, &GloCfgValue.word);

		DBGLOG(INIT, INFO,
		"WFDMA_%d GLO(0x%08x):0x%08x,EN T/R=(%d/%d), Busy T/R=(%d/%d)\n",
			idx, u4DmaCfgCrAddr, GloCfgValue.word,
			GloCfgValue.field_conn2x.tx_dma_en,
			GloCfgValue.field_conn2x.rx_dma_en,
			GloCfgValue.field_conn2x.tx_dma_busy,
			GloCfgValue.field_conn2x.rx_dma_busy);
	}

}

void show_wfdma_ring_info_without_adapter(
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{

	uint32_t idx = 0;
	uint32_t group_cnt = 0;
	uint32_t u4DmaCfgCrAddr;
	struct wfdma_group_info *group;
	uint32_t u4_hw_desc_base_value = 0;
	uint32_t u4_hw_cnt_value = 0;
	uint32_t u4_hw_cidx_value = 0;
	uint32_t u4_hw_didx_value = 0;
	uint32_t queue_cnt;

	/* Dump All TX Ring Info */
	DBGLOG(HAL, TRACE, "----------- TX Ring Config -----------\n");
	DBGLOG(HAL, TRACE, "%4s %16s %8s %10s %6s %6s %6s %6s\n",
		"Idx", "Attr", "Reg", "Base", "Cnt", "CIDX", "DIDX", "QCnt");

	/* Dump TX Ring */
	if (enum_wfdma_type == WFDMA_TYPE_HOST)
		group_cnt = sizeof(wfmda_host_tx_group_driver_base) /
		sizeof(struct wfdma_group_info);
	else
		group_cnt = sizeof(wfmda_wm_tx_group_driver_base) /
		sizeof(struct wfdma_group_info);

	for (idx = 0; idx < group_cnt; idx++) {
		if (enum_wfdma_type == WFDMA_TYPE_HOST)
			group = &wfmda_host_tx_group_driver_base[idx];
		else
			group = &wfmda_wm_tx_group_driver_base[idx];

		u4DmaCfgCrAddr = group->hw_desc_base;

		wf_ioremap_read(u4DmaCfgCrAddr, &u4_hw_desc_base_value);
		wf_ioremap_read(u4DmaCfgCrAddr+0x04, &u4_hw_cnt_value);
		wf_ioremap_read(u4DmaCfgCrAddr+0x08, &u4_hw_cidx_value);
		wf_ioremap_read(u4DmaCfgCrAddr+0x0c, &u4_hw_didx_value);

		queue_cnt = (u4_hw_cidx_value >= u4_hw_didx_value) ?
			(u4_hw_cidx_value - u4_hw_didx_value) :
			(u4_hw_cidx_value - u4_hw_didx_value + u4_hw_cnt_value);

		DBGLOG(HAL, INFO, "%4d %16s %8x %10x %6x %6x %6x %6x\n",
					idx,
					group->name,
					u4DmaCfgCrAddr, u4_hw_desc_base_value,
					u4_hw_cnt_value, u4_hw_cidx_value,
					u4_hw_didx_value, queue_cnt);

	}

	/* Dump All RX Ring Info */
	DBGLOG(HAL, TRACE, "----------- RX Ring Config -----------\n");
	DBGLOG(HAL, TRACE, "%4s %16s %8s %10s %6s %6s %6s %6s\n",
		"Idx", "Attr", "Reg", "Base", "Cnt", "CIDX", "DIDX", "QCnt");

	/* Dump RX Ring */
	if (enum_wfdma_type == WFDMA_TYPE_HOST)
		group_cnt = sizeof(wfmda_host_rx_group_driver_base) /
		sizeof(struct wfdma_group_info);
	else
		group_cnt = sizeof(wfmda_wm_rx_group_driver_base) /
		sizeof(struct wfdma_group_info);

	for (idx = 0; idx < group_cnt; idx++) {
		if (enum_wfdma_type == WFDMA_TYPE_HOST)
			group = &wfmda_host_rx_group_driver_base[idx];
		else
			group = &wfmda_wm_rx_group_driver_base[idx];

		u4DmaCfgCrAddr = group->hw_desc_base;

		wf_ioremap_read(u4DmaCfgCrAddr, &u4_hw_desc_base_value);
		wf_ioremap_read(u4DmaCfgCrAddr+0x04, &u4_hw_cnt_value);
		wf_ioremap_read(u4DmaCfgCrAddr+0x08, &u4_hw_cidx_value);
		wf_ioremap_read(u4DmaCfgCrAddr+0x0c, &u4_hw_didx_value);

		queue_cnt = (u4_hw_didx_value > u4_hw_cidx_value) ?
			(u4_hw_didx_value - u4_hw_cidx_value - 1) :
			(u4_hw_didx_value - u4_hw_cidx_value
			+ u4_hw_cnt_value - 1);

		DBGLOG(HAL, INFO, "%4d %16s %8x %10x %6x %6x %6x %6x\n",
					idx,
					group->name,
					u4DmaCfgCrAddr, u4_hw_desc_base_value,
					u4_hw_cnt_value, u4_hw_cidx_value,
					u4_hw_didx_value, queue_cnt);
	}

}

static void dump_dbg_value_without_adapter(
	IN uint32_t pdma_base_cr,
	IN uint32_t set_value,
	IN uint32_t isMandatoryDump)
{
	uint32_t set_debug_cr = 0;
	uint32_t get_debug_cr = 0;
	uint32_t get_debug_value = 0;

	set_debug_cr = pdma_base_cr + 0x124;
	get_debug_cr = pdma_base_cr + 0x128;

	wf_ioremap_write(set_debug_cr, set_value);
	wf_ioremap_read(get_debug_cr, &get_debug_value);

	if (isMandatoryDump == 1) {
		DBGLOG(INIT, INFO, "set(0x%08x):0x%08x, get(0x%08x):0x%08x,",
						set_debug_cr, set_value,
						get_debug_cr, get_debug_value);
	} else {
		DBGLOG(INIT, TRACE, "set(0x%08x):0x%08x, get(0x%08x):0x%08x,",
						set_debug_cr, set_value,
						get_debug_cr, get_debug_value);
	}
}

static void dump_wfdma_dbg_value_without_adapter(
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type,
	IN uint32_t wfdma_idx,
	IN uint32_t isMandatoryDump)
{
	uint32_t pdma_base_cr = 0;
	uint32_t set_debug_flag_value = 0;

	if (enum_wfdma_type == WFDMA_TYPE_HOST) {
		if (wfdma_idx == 0)
			pdma_base_cr = CONNAC2X_HOST_WPDMA_0_DRIVER_BASE;
		else
			pdma_base_cr = CONNAC2X_HOST_WPDMA_1_DRIVER_BASE;
	} else{
		if (wfdma_idx == 0)
			pdma_base_cr = CONNAC2X_MCU_WPDMA_0_DRIVER_BASE;
		else
			pdma_base_cr = CONNAC2X_MCU_WPDMA_1_DRIVER_BASE;
	}

	set_debug_flag_value = 0x100;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x101;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x102;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x103;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x104;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x105;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x107;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x10A;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x10D;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x10E;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x10F;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x110;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x111;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

	set_debug_flag_value = 0x112;
	dump_dbg_value_without_adapter(pdma_base_cr,
		set_debug_flag_value, isMandatoryDump);

}

static void show_wfdma_dbg_flag_log_without_adapter(
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type,
	IN uint32_t isMandatoryDump)
{
	dump_wfdma_dbg_value_without_adapter(
		enum_wfdma_type, 0, isMandatoryDump);
	dump_wfdma_dbg_value_without_adapter(
		enum_wfdma_type, 1, isMandatoryDump);
}

static void show_wfdma_dbg_log_without_adapter(
	IN enum _ENUM_WFDMA_TYPE_T enum_wfdma_type)
{
	/* Dump Host WFMDA info */
	DBGLOG(HAL, TRACE, "WFMDA Configuration:\n");
	show_wfdma_interrupt_info_without_adapter(enum_wfdma_type);
	show_wfdma_glo_info_without_adapter(enum_wfdma_type);
	show_wfdma_ring_info_without_adapter(enum_wfdma_type);
}

/* bIsHostDMA = 1 (how Host PDMA) else (WM PDMA) */
void soc3_0_show_wfdma_info_by_type_without_adapter(
	IN bool bShowWFDMA_type)
{
	if (bShowWFDMA_type) {
		show_wfdma_dbg_log_without_adapter(WFDMA_TYPE_WM);
		show_wfdma_dbg_flag_log_without_adapter(WFDMA_TYPE_WM, TRUE);
	} else {
		show_wfdma_dbg_log_without_adapter(WFDMA_TYPE_HOST);
		show_wfdma_dbg_flag_log_without_adapter(WFDMA_TYPE_HOST, TRUE);
	}
}

#endif /* WFSYS_SUPPORT_BUS_HANG_READ_FROM_DRIVER_BASE */

#endif /* SOC3_0 */
