/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*! \file   "eht_rlm.h"
 *  \brief
 */

#ifndef _EHT_RLM_H
#define _EHT_RLM_H

#if (CFG_SUPPORT_802_11BE == 1)

/******************************************************************************
 *                         C O M P I L E R   F L A G S
 ******************************************************************************
 */

/******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 ******************************************************************************
 */

/******************************************************************************
 *                              C O N S T A N T S
 ******************************************************************************
 */

/******************************************************************************
 *                                 M A C R O S
 ******************************************************************************
 */

/******************************************************************************
 *                             D A T A   T Y P E S
 ******************************************************************************
 */

/******************************************************************************
 *                            P U B L I C   D A T A
 ******************************************************************************
 */

/******************************************************************************
 *                           P R I V A T E   D A T A
 ******************************************************************************
 */

/******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 ******************************************************************************
 */

u_int32_t ehtRlmCalculateCapIELen(
	struct ADAPTER *prAdapter,
	u_int8_t ucBssIndex,
	struct STA_RECORD *prStaRec);
u_int32_t ehtRlmCalculateOpIELen(
	struct ADAPTER *prAdapter,
	u_int8_t ucBssIndex,
	struct STA_RECORD *prStaRec);
void ehtRlmReqGenerateCapIE(
	struct ADAPTER *prAdapter,
	struct MSDU_INFO *prMsduInfo);
void ehtRlmRspGenerateCapIE(
	struct ADAPTER *prAdapter,
	struct MSDU_INFO *prMsduInfo);
void ehtRlmRspGenerateOpIE(
	struct ADAPTER *prAdapter,
	struct MSDU_INFO *prMsduInfo);
void ehtRlmRecCapInfo(
	struct ADAPTER *prAdapter,
	struct STA_RECORD *prStaRec,
	u_int8_t *pucIE);
void ehtRlmRecOperation(
	struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo,
	u_int8_t *pucIE);
void ehtRlmInit(
	struct ADAPTER *prAdapter);
#endif /* CFG_SUPPORT_802_11BE == 1 */
#endif /* !_EHT_RLM_H */
