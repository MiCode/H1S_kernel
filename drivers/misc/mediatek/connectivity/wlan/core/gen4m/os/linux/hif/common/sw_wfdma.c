/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * Id: @(#) sw_wfdma.c@@
 */
/*  \file   sw_wfdma.c
 *  \brief  The program provides SW WIFIDMA HIF APIs
 *
 *  This file contains the support sw version wifi dma.
 */

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"

#include "hif_pdma.h"

#include <linux/mm.h>
#ifndef CONFIG_X86
#include <asm/memory.h>
#endif

#include "mt66xx_reg.h"
#include "gl_kal.h"

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

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

void halSwWfdmaInit(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;
	uint32_t u4Value = 0;
#if CFG_MTK_ANDROID_EMI
	void __iomem *pucEmiBaseAddr = NULL;
#endif

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	if (!prSwWfdmaInfo->fgIsEnSwWfdma)
		return;

	prSwWfdmaInfo->u4PortIdx = TX_RING_CMD_IDX_3;

	/* update sw wfdma emi offset */
	if (prSwWfdmaInfo->u4EmiOffsetAddr) {
		kalDevRegRead(prGlueInfo,
			      prSwWfdmaInfo->u4EmiOffsetAddr,
			      &u4Value);
		u4Value = (u4Value & prSwWfdmaInfo->u4EmiOffsetMask) |
			prSwWfdmaInfo->u4EmiOffsetBase;
		if (u4Value)
			prSwWfdmaInfo->u4EmiOffset = u4Value;
		DBGLOG(INIT, INFO, "EMI offset[0x%x]\n", u4Value);
	}

	if (!prSwWfdmaInfo->fgIsEnSwWfdma)
		return;

#if CFG_MTK_ANDROID_EMI
	if (prSwWfdmaInfo->pucIoremapAddr) {
		DBGLOG(INIT, ERROR, "prDmad already remap\n");
		return;
	}

	if (!gConEmiPhyBaseFinal) {
		DBGLOG(INIT, ERROR,
		       "Consys emi memory address gConEmiPhyBaseFinal invalid\n");
		return;
	}

	request_mem_region(
		gConEmiPhyBaseFinal + prSwWfdmaInfo->u4EmiOffset,
		SW_WFDMA_EMI_SIZE,
		"WIFI-SW-WFDMA");
	pucEmiBaseAddr = ioremap(
		gConEmiPhyBaseFinal + prSwWfdmaInfo->u4EmiOffset,
		SW_WFDMA_EMI_SIZE);

	DBGLOG_LIMITED(INIT, INFO,
		       "EmiPhyBase:0x%llx offset:0x%x, ioremap region 0x%lX @ 0x%p\n",
		       (uint64_t)gConEmiPhyBaseFinal,
		       prSwWfdmaInfo->u4EmiOffset,
		       gConEmiSizeFinal, pucEmiBaseAddr);

	if (!pucEmiBaseAddr) {
		DBGLOG(INIT, ERROR, "ioremap_nocache failed\n");
		return;
	}

	prSwWfdmaInfo->pucIoremapAddr = pucEmiBaseAddr;
	prSwWfdmaInfo->prDmad =
		(struct SW_WFDMAD *)(pucEmiBaseAddr);
	halSwWfdmaReset(prSwWfdmaInfo);
#endif /* CFG_MTK_ANDROID_EMI */
}

void halSwWfdmaUninit(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

#if CFG_MTK_ANDROID_EMI
	if (!prSwWfdmaInfo->pucIoremapAddr) {
		DBGLOG(INIT, ERROR, "prDmad not remap\n");
		return;
	}

	DBGLOG_LIMITED(INIT, INFO, "iounmap 0x%p\n",
		       prSwWfdmaInfo->pucIoremapAddr);
	iounmap(prSwWfdmaInfo->pucIoremapAddr);
	release_mem_region(
		gConEmiPhyBase + prSwWfdmaInfo->u4EmiOffset,
		SW_WFDMA_EMI_SIZE);
#endif /* CFG_MTK_ANDROID_EMI */
	prSwWfdmaInfo->pucIoremapAddr = NULL;
	prSwWfdmaInfo->prDmad = NULL;
}

void halSwWfdmaEn(struct GLUE_INFO *prGlueInfo, bool fgEn)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	if (prSwWfdmaInfo->fgIsEnSwWfdma == fgEn)
		return;

	prSwWfdmaInfo->fgIsEnSwWfdma = fgEn;
	if (fgEn)
		halSwWfdmaInit(prGlueInfo);
	else
		halSwWfdmaUninit(prGlueInfo);
}

void halSwWfdmaReset(struct SW_WFDMA_INFO *prSwWfdmaInfo)
{
	if (!prSwWfdmaInfo->prDmad)
		return;

	prSwWfdmaInfo->u4CpuIdx = 0;
	prSwWfdmaInfo->u4DmaIdx = 0;
	prSwWfdmaInfo->prDmad->u4DrvIdx = 0;
	prSwWfdmaInfo->prDmad->u4FwIdx = 0;
	memset_io(prSwWfdmaInfo->prDmad->aucBuf, 0,
		   SW_WFDMA_CMD_NUM * SW_WFDMA_CMD_PKT_SIZE);
}

void halSwWfdmaBackup(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;
	struct SW_WFDMAD *prSwWfDmad;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;
	prSwWfDmad = prSwWfdmaInfo->prDmad;

	if (!prSwWfdmaInfo->fgIsEnSwWfdma || !prSwWfDmad)
		return;

	prSwWfdmaInfo->u4CpuIdxBackup = prSwWfdmaInfo->u4CpuIdx;
	prSwWfdmaInfo->u4DmaIdxBackup = prSwWfdmaInfo->u4DmaIdx;

	kalMemCopyFromIo(&prSwWfdmaInfo->rBackup, prSwWfDmad,
		      sizeof(struct SW_WFDMAD));
}

void halSwWfdmaRestore(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;
	struct SW_WFDMAD *prSwWfDmad, *prBackup;
	uint8_t ucCID;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;
	prSwWfDmad = prSwWfdmaInfo->prDmad;
	prBackup = &prSwWfdmaInfo->rBackup;

	if (!prSwWfdmaInfo->fgIsEnSwWfdma || !prSwWfDmad)
		return;

	prSwWfdmaInfo->u4CpuIdx = prSwWfdmaInfo->u4CpuIdxBackup;
	prSwWfdmaInfo->u4DmaIdx = prSwWfdmaInfo->u4DmaIdxBackup;

	while (prBackup->u4FwIdx != prBackup->u4DrvIdx) {
		kalMemCopyToIo(prSwWfDmad->aucBuf[prSwWfDmad->u4DrvIdx],
			    prBackup->aucBuf[prBackup->u4FwIdx],
			    SW_WFDMA_CMD_PKT_SIZE);
		ucCID = prSwWfdmaInfo->aucCID[prSwWfDmad->u4DrvIdx];

		prSwWfDmad->u4DrvIdx =
			(prSwWfDmad->u4DrvIdx + 1) % SW_WFDMA_CMD_NUM;
		prBackup->u4FwIdx = (prBackup->u4FwIdx + 1) % SW_WFDMA_CMD_NUM;

		DBGLOG(HAL, INFO,
		       "Restore CMD DRV[%u] FW[%u] BDRV[%u] BFW[%u] CID[%u]\n",
		       prSwWfDmad->u4DrvIdx,
		       prSwWfDmad->u4FwIdx,
		       prBackup->u4DrvIdx,
		       prBackup->u4FwIdx,
		       ucCID);

		if (prSwWfdmaInfo->rOps.triggerInt)
			prSwWfdmaInfo->rOps.triggerInt(prGlueInfo);
		else
			DBGLOG(HAL, ERROR, "triggerInt callback is null!\n");
	}
}

void halSwWfdmaGetCidx(struct GLUE_INFO *prGlueInfo, uint32_t *pu4Cidx)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	*pu4Cidx = prSwWfdmaInfo->u4CpuIdx;
}

void halSwWfdmaSetCidx(struct GLUE_INFO *prGlueInfo, uint32_t u4Cidx)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	if (u4Cidx >= prSwWfdmaInfo->u4MaxCnt) {
		DBGLOG(HAL, ERROR, "invalid cidx[%u]\n", u4Cidx);
		return;
	}

	prSwWfdmaInfo->u4CpuIdx = u4Cidx;
	halSwWfdmaWriteCmd(prGlueInfo);
}

void halSwWfdmaGetDidx(struct GLUE_INFO *prGlueInfo, uint32_t *pu4Didx)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	*pu4Didx = prSwWfdmaInfo->u4DmaIdx;
}

bool halSwWfdmaIsFull(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;
	struct SW_WFDMAD *prSwWfDmad;
	uint32_t u4Idx;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;
	prSwWfDmad = prSwWfdmaInfo->prDmad;

	u4Idx = (prSwWfDmad->u4DrvIdx + 1) % SW_WFDMA_CMD_NUM;

	return u4Idx == prSwWfDmad->u4FwIdx;
}

bool halSwWfdmaWriteCmd(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct GL_HIF_INFO *prHifInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;
	struct SW_WFDMAD *prSwWfDmad;
	struct RTMP_TX_RING *prTxRing;
	struct RTMP_DMACB *pTxCell;
	struct TXD_STRUCT *pTxD;
	struct CMD_INFO *prCmdInfo;
	uint32_t u4Size;
	void *prBuf;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prHifInfo = &prGlueInfo->rHifInfo;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;
	prSwWfDmad = prSwWfdmaInfo->prDmad;

	prTxRing = &prHifInfo->TxRing[TX_RING_CMD_IDX_3];

	if (!prSwWfdmaInfo->fgIsEnSwWfdma || !prSwWfDmad)
		return false;

	if (prSwWfdmaInfo->u4DmaIdx == prSwWfdmaInfo->u4CpuIdx)
		return false;

	while (prSwWfdmaInfo->u4DmaIdx != prSwWfdmaInfo->u4CpuIdx) {
		if (halSwWfdmaIsFull(prGlueInfo))
			break;

		prBuf = (void *)prSwWfDmad->aucBuf[prSwWfDmad->u4DrvIdx];
		pTxCell = &prTxRing->Cell[prSwWfdmaInfo->u4DmaIdx];
		pTxD = (struct TXD_STRUCT *)pTxCell->AllocVa;
		prCmdInfo = (struct CMD_INFO *)pTxCell->pPacket;
		prSwWfdmaInfo->aucCID[prSwWfDmad->u4DrvIdx] =
			prCmdInfo ? prCmdInfo->ucCID : 0;

		u4Size = pTxD->SDLen0;
		if (u4Size > SW_WFDMA_CMD_PKT_SIZE || u4Size == 0) {
			DBGLOG(HAL, ERROR,
			       "Incorrect cmd size[%u] CID[0x%x]. Skip it!\n",
			       u4Size,
			       prSwWfdmaInfo->aucCID[prSwWfDmad->u4DrvIdx]);
			prSwWfdmaInfo->u4DmaIdx =
				(prSwWfdmaInfo->u4DmaIdx + 1) %
				prSwWfdmaInfo->u4MaxCnt;
			continue;
		}

		kalMemCopyToIo(prBuf, (void *)pTxCell->DmaBuf.AllocVa, u4Size);
		pTxD->DMADONE = 1;

		prSwWfdmaInfo->u4DmaIdx =
			(prSwWfdmaInfo->u4DmaIdx + 1) % prSwWfdmaInfo->u4MaxCnt;
		prSwWfDmad->u4DrvIdx =
			(prSwWfDmad->u4DrvIdx + 1) % SW_WFDMA_CMD_NUM;

		DBGLOG_LIMITED(
			HAL, INFO,
			"Write CMD DRV[%u] FW[%u] CID[0x%02X]\n",
			prSwWfDmad->u4DrvIdx,
			prSwWfDmad->u4FwIdx,
			prCmdInfo ? prCmdInfo->ucCID : 0);

		DBGLOG_MEM32(HAL, TRACE, prBuf, u4Size);

		if (prSwWfdmaInfo->rOps.triggerInt)
			prSwWfdmaInfo->rOps.triggerInt(prGlueInfo);
		else
			DBGLOG(HAL, ERROR, "triggerInt callback is null!\n");
	}

	return true;
}

bool halSwWfdmaProcessDmaDone(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	if (!prSwWfdmaInfo->fgIsEnSwWfdma)
		return false;

	halWpdmaProcessCmdDmaDone(prGlueInfo, TX_RING_CMD_IDX_3);
	return halSwWfdmaWriteCmd(prGlueInfo);
}

void halSwWfdmaDumpDebugLog(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;
	struct SW_WFDMAD *prSwWfDmad;
	uint32_t u4Val = 0, u4Idx;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;
	prSwWfDmad = prSwWfdmaInfo->prDmad;

	DBGLOG(HAL, INFO,
	       "EN[%u], CIDX[%u] DIDX[%u] MCNT[%u] ADDR[0x%lX] OFFSET[0x%X]\n",
	       prSwWfdmaInfo->fgIsEnSwWfdma,
	       prSwWfdmaInfo->u4CpuIdx,
	       prSwWfdmaInfo->u4DmaIdx,
	       prSwWfdmaInfo->u4MaxCnt,
	       prSwWfdmaInfo->pucIoremapAddr,
	       prSwWfdmaInfo->u4EmiOffset);

	if (!prSwWfDmad)
		return;

	if (prSwWfdmaInfo->rOps.getIntSta)
		prSwWfdmaInfo->rOps.getIntSta(prGlueInfo, &u4Val);
	DBGLOG(HAL, INFO, "DRV[%u] FW[%u] STA[0x%X]\n",
	       prSwWfDmad->u4DrvIdx, prSwWfDmad->u4FwIdx, u4Val);
	for (u4Idx = 0; u4Idx < SW_WFDMA_CMD_NUM; u4Idx++) {
		DBGLOG(HAL, INFO, "IDX[%u] CID[0x%02X]\n",
		       u4Idx, prSwWfdmaInfo->aucCID[u4Idx]);
	}
}
