/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
** Id: @(#) eht_rlm.c@@
*/

/*! \file   "eht_rlm.c"
*    \brief This file contains EHT Phy processing routines.
*
*/

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

#include "precomp.h"

#if (CFG_SUPPORT_802_11BE == 1)
#include "eht_rlm.h"

#define CFG_SUPPORT_WIFI_6G 0

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
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
uint8_t _ehtGetBssBandBw(struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo,
	enum ENUM_BAND eBand)
{
	uint8_t ucMaxBandwidth = MAX_BW_20MHZ;

	if (IS_BSS_AIS(prBssInfo)) {
		if (eBand == BAND_2G4)
			ucMaxBandwidth = prAdapter->rWifiVar.ucSta2gBandwidth;
		else if (eBand == BAND_5G)
			ucMaxBandwidth = prAdapter->rWifiVar.ucSta5gBandwidth;
#if (CFG_SUPPORT_WIFI_6G == 1)
		else if (eBand == BAND_6G)
			ucMaxBandwidth = prAdapter->rWifiVar.ucSta6gBandwidth;
#endif
		if (ucMaxBandwidth > prAdapter->rWifiVar.ucStaBandwidth)
			ucMaxBandwidth = prAdapter->rWifiVar.ucStaBandwidth;
	} else if (IS_BSS_P2P(prBssInfo)) {
		/* AP mode */
		if (p2pFuncIsAPMode(
				prAdapter->rWifiVar.prP2PConnSettings[
					prBssInfo->u4PrivateData])) {
			if (prBssInfo->eBand == BAND_2G4)
				ucMaxBandwidth = prAdapter->rWifiVar
					.ucAp2gBandwidth;
			else if (prBssInfo->eBand == BAND_5G)
				ucMaxBandwidth = prAdapter->rWifiVar
					.ucAp5gBandwidth;
#if (CFG_SUPPORT_WIFI_6G == 1)
			else if (prBssInfo->eBand == BAND_6G)
				ucMaxBandwidth = prAdapter->rWifiVar
					.ucAp6gBandwidth;
#endif
			if (ucMaxBandwidth
				> prAdapter->rWifiVar.ucApBandwidth)
				ucMaxBandwidth = prAdapter->rWifiVar
					.ucApBandwidth;
		}
		/* P2P mode */
		else {
			if (prBssInfo->eBand == BAND_2G4)
				ucMaxBandwidth = prAdapter->rWifiVar
					.ucP2p2gBandwidth;
			else if (prBssInfo->eBand == BAND_5G)
				ucMaxBandwidth = prAdapter->rWifiVar
					.ucP2p5gBandwidth;
#if (CFG_SUPPORT_WIFI_6G == 1)
			else if (prBssInfo->eBand == BAND_6G)
				ucMaxBandwidth = prAdapter->rWifiVar
					.ucP2p6gBandwidth;
#endif
		}
	}

	return ucMaxBandwidth;
}

uint32_t ehtRlmCalculateCapIELen(
	struct ADAPTER *prAdapter,
	uint8_t ucBssIndex,
	struct STA_RECORD *prStaRec)
{
	/* struct BSS_INFO *prBssInfo = (struct BSS_INFO *) NULL; */
	uint32_t u4OverallLen = OFFSET_OF(struct _IE_EHT_CAP_T, aucVarInfo[0]);

	return u4OverallLen;
}

uint32_t ehtRlmCalculateOpIELen(
	struct ADAPTER *prAdapter,
	uint8_t ucBssIndex,
	struct STA_RECORD *prStaRec)
{
	/* struct BSS_INFO *prBssInfo = (struct BSS_INFO *) NULL; */
	uint32_t u4OverallLen = OFFSET_OF(struct _IE_EHT_OP_T, aucVarInfo[0]);

	return u4OverallLen;
}

static void ehtRlmFillCapIE(
	struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo,
	struct MSDU_INFO *prMsduInfo)
{
	struct _IE_EHT_CAP_T *prEhtCap;
	struct eht_phy_capinfo eht_phy_cap;
	uint32_t phy_cap_1 = 0;
	uint32_t phy_cap_2 = 0;
	uint32_t u4OverallLen = OFFSET_OF(struct _IE_EHT_CAP_T, aucVarInfo[0]);
	uint8_t eht_mcs15_mru = EHT_MCS15_MRU_106_or_52_w_26_tone;
	uint8_t eht_bw;

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prMsduInfo);

	prEhtCap = (struct _IE_EHT_CAP_T *)
		(((uint8_t *)prMsduInfo->prPacket)+prMsduInfo->u2FrameLength);

	prEhtCap->ucId = ELEM_ID_RESERVED;
	prEhtCap->ucExtId = EID_EXT_EHT_CAPS;

	/* MAC capabilities */
	EHT_RESET_MAC_CAP(prEhtCap->ucEhtMacCap);

	SET_EHT_MAC_CAP_NSEP_PRI_ACCESS(prEhtCap->ucEhtMacCap);
	SET_EHT_MAC_CAP_OM_CTRL(prEhtCap->ucEhtMacCap);
	SET_EHT_MAC_CAP_TXOP_SHARING(prEhtCap->ucEhtMacCap);

	/* PHY capabilities */
	EHT_RESET_PHY_CAP(prEhtCap->ucEhtPhyCap);

#if (CFG_SUPPORT_WIFI_6G == 1)
	if (prBssInfo->eBand == BAND_6G) {
		if (_ehtGetBssBandBw(prAdapter, prBssInfo, BAND_6G)
			>= MAX_BW_320MHZ)
			phy_cap_1 |= DOT11BE_PHY_CAP_320M_6G;
	}
#endif

	eht_bw = _ehtGetBssBandBw(prAdapter,
		prBssInfo,
		prBssInfo->eBand);

	if (!IS_BSS_APGO(prBssInfo)) {
		phy_cap_2 |= DOT11BE_PHY_CAP_PARTIAL_BW_DL_MU_MIMO;

		if (eht_bw == MAX_BW_20MHZ)
			phy_cap_1 &= ~DOT11BE_PHY_CAP_242_TONE_RU_WT_20M;
	} else {
		phy_cap_2 |= DOT11BE_PHY_CAP_PPE_THRLD_PRESENT;
	}


	phy_cap_1 |= DOT11BE_PHY_CAP_PARTIAL_BW_UL_MU_MIMO;

	if (eht_bw >= MAX_BW_80MHZ)
		eht_mcs15_mru |= EHT_MCS15_MRU_484_w_242_tone_80M;
	if (eht_bw >= MAX_BW_160MHZ)
		eht_mcs15_mru |= EHT_MCS15_MRU_996_to_242_tone_160M;
	if (eht_bw == MAX_BW_320MHZ)
		eht_mcs15_mru |= EHT_MCS15_MRU_3x996_tone_320M;

	if (1) {
		phy_cap_1 |= DOT11BE_PHY_CAP_NDP_4X_EHT_LTF_3DOT2US_GI;
		phy_cap_1 |= DOT11BE_PHY_CAP_SU_BFER;
		phy_cap_1 |= DOT11BE_PHY_CAP_SU_BFEE;
		phy_cap_1 |= DOT11BE_PHY_CAP_MU_BFER;
		phy_cap_1 |= DOT11BE_PHY_CAP_NG16_SU_FEEDBACK;
		phy_cap_1 |= DOT11BE_PHY_CAP_NG32_SU_FEEDBACK;
		phy_cap_1 |= DOT11BE_PHY_CAP_CODEBOOK_4_2_SU_FEEDBACK;
		phy_cap_1 |= DOT11BE_PHY_CAP_CODEBOOK_7_5_SU_FEEDBACK;
		phy_cap_1 |= DOT11BE_PHY_CAP_TRIGED_SU_BF_FEEDBACK;
		phy_cap_1 |= DOT11BE_PHY_CAP_TRIGED_MU_BF_PARTIAL_BW_FEEDBACK;

#if 0
		SET_DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M(phy_cap_1,
			eht_bf_struct.bfee_ss_le_eq_bw80);
		SET_DOT11BE_PHY_CAP_BFEE_160M(phy_cap_1,
			eht_bf_struct.bfee_ss_bw160);
		SET_DOT11BE_PHY_CAP_BFEE_320M(phy_cap_1,
			eht_bf_struct.bfee_ss_bw320);
		SET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M(phy_cap_1,
			eht_bf_struct.snd_dim_le_eq_bw80);
		SET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M(phy_cap_1,
			eht_bf_struct.snd_dim_bw160);
		SET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M(phy_cap_1,
			eht_bf_struct.snd_dim_bw320);
		SET_DOT11BE_PHY_CAP_MAX_NC(phy_cap_1,
			eht_bf_struct.bfee_max_nc);
		SET_DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU(phy_cap_2,
			eht_bf_struct.max_ltf_num_su_non_ofdma);
		SET_DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU(phy_cap_2,
			eht_bf_struct.max_ltf_num_mu);
#endif
	}


	SET_DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD(phy_cap_2,
		COMMON_NOMINAL_PAD_0_US);

	SET_DOT11BE_PHY_CAP_MCS_15(phy_cap_2, eht_mcs15_mru);

	eht_phy_cap.phy_capinfo_1 = (phy_cap_1);
	eht_phy_cap.phy_capinfo_2 = (phy_cap_2);

	memcpy(prEhtCap->ucEhtPhyCap, prEhtCap->ucEhtPhyCap,
		sizeof(eht_phy_cap));

	prEhtCap->ucLength = u4OverallLen - ELEM_HDR_LEN;

	prMsduInfo->u2FrameLength += IE_SIZE(prEhtCap);
}

void ehtRlmReqGenerateCapIE(
	struct ADAPTER *prAdapter,
	struct MSDU_INFO *prMsduInfo)
{
	struct BSS_INFO *prBssInfo;
	struct STA_RECORD *prStaRec;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo)
		return;

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	if ((prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11BE)
	    && (!prStaRec || (prStaRec->ucPhyTypeSet & PHY_TYPE_SET_802_11BE)))
		ehtRlmFillCapIE(prAdapter, prBssInfo, prMsduInfo);
}

void ehtRlmRspGenerateCapIE(
	struct ADAPTER *prAdapter,
	struct MSDU_INFO *prMsduInfo)
{
	struct BSS_INFO *prBssInfo;
	struct STA_RECORD *prStaRec;
	uint8_t ucPhyTypeSet;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo)
		return;

	if (!IS_BSS_ACTIVE(prBssInfo))
		return;

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	/* Decide PHY type set source */
	if (prStaRec) {
		/* Get PHY type set from target STA */
		ucPhyTypeSet = prStaRec->ucPhyTypeSet;
	} else {
		/* Get PHY type set from current BSS */
		ucPhyTypeSet = prBssInfo->ucPhyTypeSet;
	}

	if (RLM_NET_IS_11BE(prBssInfo) &&
	    (ucPhyTypeSet & PHY_TYPE_SET_802_11BE))
		ehtRlmFillCapIE(prAdapter, prBssInfo, prMsduInfo);
}

static void ehtRlmFillOpIE(
	struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo,
	struct MSDU_INFO *prMsduInfo)
{
	struct _IE_EHT_OP_T *prEhtOp;
	uint32_t u4OverallLen = OFFSET_OF(struct _IE_EHT_OP_T, aucVarInfo[0]);

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prMsduInfo);

	prEhtOp = (struct _IE_EHT_OP_T *)
		(((uint8_t *)prMsduInfo->prPacket)+prMsduInfo->u2FrameLength);

	prEhtOp->ucId = ELEM_ID_RESERVED;
	prEhtOp->ucExtId = EID_EXT_EHT_CAPS;

	/* MAC capabilities */
	EHT_RESET_OP(prEhtOp->ucEhtOpParams);

	prEhtOp->ucLength = u4OverallLen - ELEM_HDR_LEN;

	prMsduInfo->u2FrameLength += IE_SIZE(prEhtOp);
}

void ehtRlmRspGenerateOpIE(
	struct ADAPTER *prAdapter,
	struct MSDU_INFO *prMsduInfo)
{
	struct BSS_INFO *prBssInfo;
	struct STA_RECORD *prStaRec;
	uint8_t ucPhyTypeSet;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo)
		return;

	if (!IS_BSS_ACTIVE(prBssInfo))
		return;

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	/* Decide PHY type set source */
	if (prStaRec) {
		/* Get PHY type set from target STA */
		ucPhyTypeSet = prStaRec->ucPhyTypeSet;
	} else {
		/* Get PHY type set from current BSS */
		ucPhyTypeSet = prBssInfo->ucPhyTypeSet;
	}

	if (RLM_NET_IS_11BE(prBssInfo) &&
	    (ucPhyTypeSet & PHY_TYPE_SET_802_11BE))
		ehtRlmFillOpIE(prAdapter, prBssInfo, prMsduInfo);
}

void ehtRlmRecCapInfo(
	struct ADAPTER *prAdapter,
	struct STA_RECORD *prStaRec,
	uint8_t *pucIE)
{
	struct _IE_EHT_CAP_T *prEhtCap = (struct _IE_EHT_CAP_T *) pucIE;

	/* if payload not contain any aucVarInfo,
	 * IE size = sizeof(struct _IE_EHT_CAP_T)
	 */
	if (IE_SIZE(prEhtCap) < (sizeof(struct _IE_EHT_CAP_T))) {
		DBGLOG(SCN, WARN,
			"EHT_CAP IE_LEN err(%d)!\n", IE_LEN(prEhtCap));
		return;
	}

	memcpy(prStaRec->ucEhtMacCapInfo, prEhtCap->ucEhtMacCap,
		EHT_MAC_CAP_BYTE_NUM);
	memcpy(prStaRec->ucEhtPhyCapInfo, prEhtCap->ucEhtPhyCap,
		EHT_PHY_CAP_BYTE_NUM);
}

void ehtRlmRecOperation(
	struct ADAPTER *prAdapter,
	struct BSS_INFO *prBssInfo,
	uint8_t *pucIE)
{
	struct _IE_EHT_OP_T *prEhtOp = (struct _IE_EHT_OP_T *) pucIE;

	/* if payload not contain any aucVarInfo,
	 * IE size = sizeof(struct _IE_EHT_OP_T)
	 */
	if (IE_SIZE(prEhtOp) < (sizeof(struct _IE_EHT_OP_T))) {
		DBGLOG(SCN, WARN,
			"HE_OP IE_LEN err(%d)!\n", IE_LEN(prEhtOp));
		return;
	}

	memcpy(prBssInfo->ucEhtOpParams, prEhtOp->ucEhtOpParams,
		HE_OP_BYTE_NUM);
}
void ehtRlmInit(
	struct ADAPTER *prAdapter)
{
}
#endif /* CFG_SUPPORT_802_11BE == 1 */
