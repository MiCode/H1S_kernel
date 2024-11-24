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

#include "precomp.h"

/*
 * definition for AP selection algrithm
 */
#define BSS_FULL_SCORE                          (100)
#define CHNL_BSS_NUM_THRESOLD                   100
#define BSS_STA_CNT_THRESOLD                    30
#define SCORE_PER_AP                            1
#define ROAMING_NO_SWING_SCORE_STEP             100
/* MCS9 at BW 160 requires rssi at least -48dbm */
#define BEST_RSSI                               -48
/* MCS7 at 20BW, MCS5 at 40BW, MCS4 at 80BW, MCS3 at 160BW */
#define GOOD_RSSI_FOR_HT_VHT                    -64
/* Link speed 1Mbps need at least rssi -94dbm for 2.4G */
#define MINIMUM_RSSI_2G4                        -94
/* Link speed 6Mbps need at least rssi -86dbm for 5G */
#define MINIMUM_RSSI_5G                         -86
#if (CFG_SUPPORT_WIFI_6G == 1)
/* Link speed 6Mbps need at least rssi -86dbm for 6G */
#define MINIMUM_RSSI_6G                         -86
#endif

/* level of rssi range on StatusBar */
#define RSSI_MAX_LEVEL                          -55
#define RSSI_SECOND_LEVEL                       -66

#if (CFG_TC10_FEATURE == 1)
#define RCPI_FOR_DONT_ROAM                      80 /*-70dbm*/
#else
#define RCPI_FOR_DONT_ROAM                      60 /*-80dbm*/
#endif

/* Real Rssi of a Bss may range in current_rssi - 5 dbm
 *to current_rssi + 5 dbm
 */
#define RSSI_DIFF_BETWEEN_BSS                   10 /* dbm */
#define LOW_RSSI_FOR_5G_BAND                    -70 /* dbm */
#define HIGH_RSSI_FOR_5G_BAND                   -60 /* dbm */

/* Support driver triggers roaming */
#define RCPI_DIFF_DRIVER_ROAM			20 /* 10 dbm */

/* In case 2.4G->5G, the trigger rssi is RSSI_BAD_NEED_ROAM_24G_TO_5G
 * In other case(2.4G->2.4G/5G->2.4G/5G->5G), the trigger
 * rssi is RSSI_BAD_NEED_ROAM
 *
 * The reason of using two rssi threshold is that we only
 * want to benifit 2.4G->5G case, and keep original logic in
 * other cases.
 */
#define RSSI_BAD_NEED_ROAM_24G_TO_5G_6G         -40 /* dbm */
#define RSSI_BAD_NEED_ROAM                      -80 /* dbm */

#define CHNL_DWELL_TIME_DEFAULT  100
#define CHNL_DWELL_TIME_ONLINE   50

/* When roam to 5G AP, the AP's rcpi should great than
 * RCPI_THRESHOLD_ROAM_2_5G dbm
 */
#define RCPI_THRESHOLD_ROAM_TO_5G_6G  90 /* rssi -65 */

#define WEIGHT_IDX_CHNL_UTIL                    0
#define WEIGHT_IDX_RSSI                         2
#define WEIGHT_IDX_SCN_MISS_CNT                 2
#define WEIGHT_IDX_PROBE_RSP                    1
#define WEIGHT_IDX_CLIENT_CNT                   0
#define WEIGHT_IDX_AP_NUM                       0
#define WEIGHT_IDX_5G_BAND                      2
#define WEIGHT_IDX_BAND_WIDTH                   1
#define WEIGHT_IDX_STBC                         1
#define WEIGHT_IDX_DEAUTH_LAST                  1
#define WEIGHT_IDX_BLACK_LIST                   2
#define WEIGHT_IDX_SAA                          0
#define WEIGHT_IDX_CHNL_IDLE                    1
#define WEIGHT_IDX_OPCHNL                       0
#define WEIGHT_IDX_TPUT                         1
#define WEIGHT_IDX_PREFERENCE                   2

#define WEIGHT_IDX_CHNL_UTIL_PER                0
#define WEIGHT_IDX_RSSI_PER                     4
#define WEIGHT_IDX_SCN_MISS_CNT_PER             4
#define WEIGHT_IDX_PROBE_RSP_PER                1
#define WEIGHT_IDX_CLIENT_CNT_PER               1
#define WEIGHT_IDX_AP_NUM_PER                   6
#define WEIGHT_IDX_5G_BAND_PER                  4
#define WEIGHT_IDX_BAND_WIDTH_PER               1
#define WEIGHT_IDX_STBC_PER                     1
#define WEIGHT_IDX_DEAUTH_LAST_PER              1
#define WEIGHT_IDX_BLACK_LIST_PER               4
#define WEIGHT_IDX_SAA_PER                      1
#define WEIGHT_IDX_CHNL_IDLE_PER                6
#define WEIGHT_IDX_OPCHNL_PER                   6
#define WEIGHT_IDX_TPUT_PER                     2
#define WEIGHT_IDX_PREFERENCE_PER               2

struct WEIGHT_CONFIG {
	uint8_t ucChnlUtilWeight;
	uint8_t ucSnrWeight;
	uint8_t ucRssiWeight;
	uint8_t ucScanMissCntWeight;
	uint8_t ucProbeRespWeight;
	uint8_t ucClientCntWeight;
	uint8_t ucApNumWeight;
	uint8_t ucBandWeight;
	uint8_t ucBandWidthWeight;
	uint8_t ucStbcWeight;
	uint8_t ucLastDeauthWeight;
	uint8_t ucBlackListWeight;
	uint8_t ucSaaWeight;
	uint8_t ucChnlIdleWeight;
	uint8_t ucOpchnlWeight;
	uint8_t ucTputWeight;
	uint8_t ucPreferenceWeight;
};

struct WEIGHT_CONFIG gasMtkWeightConfig[ROAM_TYPE_NUM] = {
	[ROAM_TYPE_RCPI] = {
		.ucChnlUtilWeight = WEIGHT_IDX_CHNL_UTIL,
		.ucRssiWeight = WEIGHT_IDX_RSSI,
		.ucScanMissCntWeight = WEIGHT_IDX_SCN_MISS_CNT,
		.ucProbeRespWeight = WEIGHT_IDX_PROBE_RSP,
		.ucClientCntWeight = WEIGHT_IDX_CLIENT_CNT,
		.ucApNumWeight = WEIGHT_IDX_AP_NUM,
		.ucBandWeight = WEIGHT_IDX_5G_BAND,
		.ucBandWidthWeight = WEIGHT_IDX_BAND_WIDTH,
		.ucStbcWeight = WEIGHT_IDX_STBC,
		.ucLastDeauthWeight = WEIGHT_IDX_DEAUTH_LAST,
		.ucBlackListWeight = WEIGHT_IDX_BLACK_LIST,
		.ucSaaWeight = WEIGHT_IDX_SAA,
		.ucChnlIdleWeight = WEIGHT_IDX_CHNL_IDLE,
		.ucOpchnlWeight = WEIGHT_IDX_OPCHNL,
		.ucTputWeight = WEIGHT_IDX_TPUT,
		.ucPreferenceWeight = WEIGHT_IDX_PREFERENCE
	},
	[ROAM_TYPE_PER] = {
		.ucChnlUtilWeight = WEIGHT_IDX_CHNL_UTIL_PER,
		.ucRssiWeight = WEIGHT_IDX_RSSI_PER,
		.ucScanMissCntWeight = WEIGHT_IDX_SCN_MISS_CNT_PER,
		.ucProbeRespWeight = WEIGHT_IDX_PROBE_RSP_PER,
		.ucClientCntWeight = WEIGHT_IDX_CLIENT_CNT_PER,
		.ucApNumWeight = WEIGHT_IDX_AP_NUM_PER,
		.ucBandWeight = WEIGHT_IDX_5G_BAND_PER,
		.ucBandWidthWeight = WEIGHT_IDX_BAND_WIDTH_PER,
		.ucStbcWeight = WEIGHT_IDX_STBC_PER,
		.ucLastDeauthWeight = WEIGHT_IDX_DEAUTH_LAST_PER,
		.ucBlackListWeight = WEIGHT_IDX_BLACK_LIST_PER,
		.ucSaaWeight = WEIGHT_IDX_SAA_PER,
		.ucChnlIdleWeight = WEIGHT_IDX_CHNL_IDLE_PER,
		.ucOpchnlWeight = WEIGHT_IDX_OPCHNL_PER,
		.ucTputWeight = WEIGHT_IDX_TPUT_PER,
		.ucPreferenceWeight = WEIGHT_IDX_PREFERENCE_PER
	}
};

static uint8_t *apucBandStr[BAND_NUM] = {
	(uint8_t *) DISP_STRING("NULL"),
	(uint8_t *) DISP_STRING("2.4G"),
	(uint8_t *) DISP_STRING("5G")
#if (CFG_SUPPORT_WIFI_6G == 1)
	,
	(uint8_t *) DISP_STRING("6G")
#endif
};

struct NETWORK_SELECTION_POLICY_BY_BAND networkReplaceHandler[BAND_NUM] = {
	[BAND_2G4] = {BAND_2G4, scanNetworkReplaceHandler2G4},
	[BAND_5G]  = {BAND_5G,  scanNetworkReplaceHandler5G},
#if (CFG_SUPPORT_WIFI_6G == 1)
	[BAND_6G]  = {BAND_6G,  scanNetworkReplaceHandler6G},
#endif
};

#if (CFG_SUPPORT_AVOID_DESENSE == 1)
const struct WFA_DESENSE_CHANNEL_LIST desenseChList[BAND_NUM] = {
	[BAND_5G]  = {120, 157},
#if (CFG_SUPPORT_WIFI_6G == 1)
	[BAND_6G]  = {13,  53},
#endif
};
#endif

uint8_t roamReasonToType[ROAMING_REASON_NUM] = {
	[0 ... ROAMING_REASON_NUM - 1] = ROAM_TYPE_RCPI,
	[ROAMING_REASON_TX_ERR]	       = ROAM_TYPE_PER,
};

#if (CFG_SUPPORT_802_11AX == 1)
#define CALCULATE_SCORE_BY_AX_AP(prAdapter, prBssDesc, eRoamType) \
	((eRoamType == ROAM_TYPE_PER) ? \
	((prAdapter->rWifiVar).ucApSelAxWeight * \
	(prBssDesc->fgIsHEPresent ? \
	(BSS_FULL_SCORE/(prAdapter->rWifiVar).ucApSelAxScoreDiv) : 0)):0)
#endif
#if (CFG_SUPPORT_802_11BE == 1)
/* TODO */
#endif

#define CALCULATE_SCORE_BY_PROBE_RSP(prBssDesc, eRoamType) \
	(gasMtkWeightConfig[eRoamType].ucProbeRespWeight * \
	(prBssDesc->fgSeenProbeResp ? BSS_FULL_SCORE : 0))

#define CALCULATE_SCORE_BY_MISS_CNT(prAdapter, prBssDesc, eRoamType) \
	(gasMtkWeightConfig[eRoamType].ucScanMissCntWeight * \
	(prAdapter->rWifiVar.rScanInfo.u4ScanUpdateIdx - \
	prBssDesc->u4UpdateIdx > 3 ? 0 : \
	(BSS_FULL_SCORE - (prAdapter->rWifiVar.rScanInfo.u4ScanUpdateIdx - \
	prBssDesc->u4UpdateIdx) * 25)))

#if (CFG_SUPPORT_WIFI_6G == 1)
#define CALCULATE_SCORE_BY_BAND(prAdapter, prBssDesc, cRssi, eRoamType) \
	(gasMtkWeightConfig[eRoamType].ucBandWeight * \
	((((prBssDesc->eBand == BAND_5G && prAdapter->fgEnable5GBand) || \
	   (prBssDesc->eBand == BAND_6G && prAdapter->fgIsHwSupport6G)) && \
	cRssi > -70) ? BSS_FULL_SCORE : 0))
#else
#define CALCULATE_SCORE_BY_BAND(prAdapter, prBssDesc, cRssi, eRoamType) \
	(gasMtkWeightConfig[eRoamType].ucBandWeight * \
	((prBssDesc->eBand == BAND_5G && prAdapter->fgEnable5GBand && \
	cRssi > -70) ? BSS_FULL_SCORE : 0))
#endif

#define CALCULATE_SCORE_BY_STBC(prAdapter, prBssDesc, eRoamType) \
	(gasMtkWeightConfig[eRoamType].ucStbcWeight * \
	(prBssDesc->fgMultiAnttenaAndSTBC ? BSS_FULL_SCORE:0))

#define CALCULATE_SCORE_BY_DEAUTH(prBssDesc, eRoamType) \
	(gasMtkWeightConfig[eRoamType].ucLastDeauthWeight * \
	(prBssDesc->prBlack && prBssDesc->prBlack->fgDeauthLastTime ? 0 : \
	BSS_FULL_SCORE))

#if CFG_SUPPORT_RSN_SCORE
#define CALCULATE_SCORE_BY_RSN(prBssDesc) \
	(WEIGHT_IDX_RSN * (prBssDesc->fgIsRSNSuitableBss ? BSS_FULL_SCORE:0))
#endif

#if 0
static uint16_t scanCaculateScoreBySTBC(struct ADAPTER *prAdapter,
	struct BSS_DESC *prBssDesc, enum ROAM_TYPE eRoamType)
{
	uint16_t u2Score = 0;
	uint8_t ucSpatial = 0;

	ucSpatial = prBssDesc->ucSpatial;

	if (prBssDesc->fgMultiAnttenaAndSTBC) {
		ucSpatial = (ucSpatial >= 4)?4:ucSpatial;
		u2Score = (BSS_FULL_SCORE-50)*ucSpatial;
	} else {
		u2Score = 0;
	}
	return u2Score*mtk_weight_config[eRoamType].ucStbcWeight;
}
#endif

/* Channel Utilization: weight index will be */
static uint16_t scanCalculateScoreByChnlInfo(
	struct AIS_SPECIFIC_BSS_INFO *prAisSpecificBssInfo, uint8_t ucChannel,
	enum ROAM_TYPE eRoamType)
{
	struct ESS_CHNL_INFO *prEssChnlInfo = &prAisSpecificBssInfo->
		arCurEssChnlInfo[0];
	uint8_t i = 0;
	uint16_t u2Score = 0;
	uint8_t weight = gasMtkWeightConfig[eRoamType].ucApNumWeight;

	for (; i < prAisSpecificBssInfo->ucCurEssChnlInfoNum; i++) {
		if (ucChannel == prEssChnlInfo[i].ucChannel) {
#if 0	/* currently, we don't take channel utilization into account */
			/* the channel utilization max value is 255.
			 *great utilization means little weight value.
			 * the step of weight value is 2.6
			 */
			u2Score = mtk_weight_config[eRoamType].
				ucChnlUtilWeight * (BSS_FULL_SCORE -
				(prEssChnlInfo[i].ucUtilization * 10 / 26));
#endif
			/* if AP num on this channel is greater than 100,
			 * the weight will be 0.
			 * otherwise, the weight value decrease 1
			 * if AP number increase 1
			 */
			if (prEssChnlInfo[i].ucApNum <= CHNL_BSS_NUM_THRESOLD)
				u2Score += weight *
				(BSS_FULL_SCORE - prEssChnlInfo[i].ucApNum *
					SCORE_PER_AP);
			log_dbg(SCN, TRACE, "channel %d, AP num %d\n",
				ucChannel, prEssChnlInfo[i].ucApNum);
			break;
		}
	}
	return u2Score;
}

static uint16_t scanCalculateScoreByBandwidth(struct ADAPTER *prAdapter,
	struct BSS_DESC *prBssDesc, enum ROAM_TYPE eRoamType)
{
	uint16_t u2Score = 0;
	enum ENUM_CHANNEL_WIDTH eChannelWidth = prBssDesc->eChannelWidth;
#if (CFG_SUPPORT_WIFI_6G == 1)
	uint8_t ucSta6GBW = prAdapter->rWifiVar.ucSta6gBandwidth;
#endif
	uint8_t ucSta5GBW = prAdapter->rWifiVar.ucSta5gBandwidth;
	uint8_t ucSta2GBW = prAdapter->rWifiVar.ucSta2gBandwidth;
	uint8_t ucStaBW = prAdapter->rWifiVar.ucStaBandwidth;

	if (prBssDesc->fgIsVHTPresent && prAdapter->fgEnable5GBand) {
		if (ucSta5GBW > ucStaBW)
			ucSta5GBW = ucStaBW;
		switch (ucSta5GBW) {
		case MAX_BW_20MHZ:
		case MAX_BW_40MHZ:
			eChannelWidth = CW_20_40MHZ;
			break;
		case MAX_BW_80MHZ:
			eChannelWidth = CW_80MHZ;
			break;
		}
		switch (eChannelWidth) {
		case CW_20_40MHZ:
			u2Score = 60;
			break;
		case CW_80MHZ:
			u2Score = 80;
			break;
		case CW_160MHZ:
		case CW_80P80MHZ:
			u2Score = BSS_FULL_SCORE;
			break;
		}
	} else if (prBssDesc->fgIsHTPresent) {
		if (prBssDesc->eBand == BAND_2G4) {
			if (ucSta2GBW > ucStaBW)
				ucSta2GBW = ucStaBW;
			u2Score = (prBssDesc->eSco == 0 ||
					ucSta2GBW == MAX_BW_20MHZ) ? 40:60;
		} else if (prBssDesc->eBand == BAND_5G) {
			if (ucSta5GBW > ucStaBW)
				ucSta5GBW = ucStaBW;
			u2Score = (prBssDesc->eSco == 0 ||
					ucSta5GBW == MAX_BW_20MHZ) ? 40:60;
		}
#if (CFG_SUPPORT_WIFI_6G == 1)
		else if (prBssDesc->eBand == BAND_6G) {
			if (ucSta6GBW > ucStaBW)
				ucSta6GBW = ucStaBW;
			u2Score = (prBssDesc->eSco == 0 ||
				ucSta6GBW == MAX_BW_20MHZ) ? 40:60;
		}
#endif
	} else if (prBssDesc->u2BSSBasicRateSet & RATE_SET_OFDM)
		u2Score = 20;
	else
		u2Score = 10;

	return u2Score * gasMtkWeightConfig[eRoamType].ucBandWidthWeight;
}

static uint16_t scanCalculateScoreByClientCnt(struct BSS_DESC *prBssDesc,
			enum ROAM_TYPE eRoamType)
{
	uint16_t u2Score = 0;
	uint16_t u2StaCnt = 0;
#define BSS_STA_CNT_NORMAL_SCORE 50
#define BSS_STA_CNT_GOOD_THRESOLD 10

	log_dbg(SCN, TRACE, "Exist bss load %d, sta cnt %d\n",
			prBssDesc->fgExsitBssLoadIE, prBssDesc->u2StaCnt);

	if (!prBssDesc->fgExsitBssLoadIE) {
		u2Score = BSS_STA_CNT_NORMAL_SCORE;
		return u2Score *
		gasMtkWeightConfig[eRoamType].ucClientCntWeight;
	}

	u2StaCnt = prBssDesc->u2StaCnt;
	if (u2StaCnt > BSS_STA_CNT_THRESOLD)
		u2Score = 0;
	else if (u2StaCnt < BSS_STA_CNT_GOOD_THRESOLD)
		u2Score = BSS_FULL_SCORE - u2StaCnt;
	else
		u2Score = BSS_STA_CNT_NORMAL_SCORE;

	return u2Score * gasMtkWeightConfig[eRoamType].ucClientCntWeight;
}

#if CFG_SUPPORT_802_11K
struct NEIGHBOR_AP *scanGetNeighborAPEntry(
	struct ADAPTER *prAdapter, uint8_t *pucBssid, uint8_t ucBssIndex)
{
	struct LINK *prNeighborAPLink =
		&aisGetAisSpecBssInfo(prAdapter, ucBssIndex)
		->rNeighborApList.rUsingLink;
	struct NEIGHBOR_AP *prNeighborAP = NULL;

	LINK_FOR_EACH_ENTRY(prNeighborAP, prNeighborAPLink, rLinkEntry,
			    struct NEIGHBOR_AP)
	{
		if (EQUAL_MAC_ADDR(prNeighborAP->aucBssid, pucBssid))
			return prNeighborAP;
	}
	return NULL;
}

#endif

uint8_t scanNetworkReplaceHandler2G4(enum ENUM_BAND eCurrentBand,
	int8_t cCandidateRssi, int8_t cCurrentRssi)
{
	/* Current AP is 2.4G, replace candidate AP if target AP is good */
	if (eCurrentBand == BAND_5G
#if (CFG_SUPPORT_WIFI_6G == 1)
		|| eCurrentBand == BAND_6G
#endif
		) {
		if (cCurrentRssi >= GOOD_RSSI_FOR_HT_VHT)
			return TRUE;

		if (cCurrentRssi < LOW_RSSI_FOR_5G_BAND &&
			(cCandidateRssi > cCurrentRssi + 5))
			return FALSE;
	}
	return FALSE;
}

uint8_t scanNetworkReplaceHandler5G(enum ENUM_BAND eCurrentBand,
	int8_t cCandidateRssi, int8_t cCurrentRssi)
{
	/* Candidate AP is 5G, don't replace it if it's good enough. */
	if (eCurrentBand == BAND_2G4) {
		if (cCandidateRssi >= GOOD_RSSI_FOR_HT_VHT)
			return FALSE;

		if (cCandidateRssi < LOW_RSSI_FOR_5G_BAND &&
			(cCurrentRssi > cCandidateRssi + 5))
			return TRUE;
	}
#if (CFG_SUPPORT_WIFI_6G == 1)
	else if (eCurrentBand == BAND_6G) {
		/* Target AP is 6G, replace candidate AP if target AP is good */
		if (cCurrentRssi >= GOOD_RSSI_FOR_HT_VHT)
			return TRUE;

		if (cCurrentRssi < LOW_RSSI_FOR_5G_BAND &&
			(cCandidateRssi > cCurrentRssi + 5))
			return FALSE;
	}
#endif
	return FALSE;
}

#if (CFG_SUPPORT_WIFI_6G == 1)
uint8_t scanNetworkReplaceHandler6G(enum ENUM_BAND eCurrentBand,
	int8_t cCandidateRssi, int8_t cCurrentRssi)
{
	if (eCurrentBand < BAND_2G4 || eCurrentBand > BAND_6G)
		return FALSE;

	/* Candidate AP is 6G, don't replace it if it's good enough. */
	if (cCandidateRssi >= GOOD_RSSI_FOR_HT_VHT)
		return FALSE;

	if (cCandidateRssi < LOW_RSSI_FOR_5G_BAND &&
		(cCurrentRssi > cCandidateRssi + 7))
		return TRUE;

	return FALSE;
}
#endif

static u_int8_t scanNeedReplaceCandidate(struct ADAPTER *prAdapter,
	struct BSS_DESC *prCandBss, struct BSS_DESC *prCurrBss,
	uint16_t u2CandScore, uint16_t u2CurrScore,
	enum ENUM_ROAMING_REASON eRoamReason, uint8_t ucBssIndex)
{
	int8_t cCandRssi = RCPI_TO_dBm(prCandBss->ucRCPI);
	int8_t cCurrRssi = RCPI_TO_dBm(prCurrBss->ucRCPI);
	uint32_t u4UpdateIdx = prAdapter->rWifiVar.rScanInfo.u4ScanUpdateIdx;
	uint16_t u2CandMiss = u4UpdateIdx - prCandBss->u4UpdateIdx;
	uint16_t u2CurrMiss = u4UpdateIdx - prCurrBss->u4UpdateIdx;
	struct BSS_DESC *prBssDesc = NULL;
	int8_t ucOpChannelNum = 0;
	enum ROAM_TYPE eRoamType = roamReasonToType[eRoamReason];

	prBssDesc = aisGetTargetBssDesc(prAdapter, ucBssIndex);
	if (prBssDesc)
		ucOpChannelNum = prBssDesc->ucChannelNum;

#if CFG_SUPPORT_NCHO
	if (prAdapter->rNchoInfo.fgNCHOEnabled)
		return cCurrRssi >= cCandRssi ? TRUE : FALSE;
#endif

	/* 1. No need check score case
	 * 1.1 Scan missing count of CurrBss is too more,
	 * but Candidate is suitable, don't replace
	 */
	if (u2CurrMiss > 2 && u2CurrMiss > u2CandMiss) {
		log_dbg(SCN, INFO, "Scan Miss count of CurrBss > 2, and Candidate <= 2\n");
		return FALSE;
	}
	/* 1.2 Scan missing count of Candidate is too more,
	 * but CurrBss is suitable, replace
	 */
	if (u2CandMiss > 2 && u2CandMiss > u2CurrMiss) {
		log_dbg(SCN, INFO, "Scan Miss count of Candidate > 2, and CurrBss <= 2\n");
		return TRUE;
	}
	/* 1.3 Hard connecting RSSI check */
	if ((prCurrBss->eBand == BAND_5G && cCurrRssi < MINIMUM_RSSI_5G) ||
#if (CFG_SUPPORT_WIFI_6G == 1)
		(prCurrBss->eBand == BAND_6G && cCurrRssi < MINIMUM_RSSI_6G) ||
#endif
		(prCurrBss->eBand == BAND_2G4 && cCurrRssi < MINIMUM_RSSI_2G4))
		return FALSE;
	else if ((prCandBss->eBand == BAND_5G && cCandRssi < MINIMUM_RSSI_5G) ||
#if (CFG_SUPPORT_WIFI_6G == 1)
		(prCandBss->eBand == BAND_6G && cCandRssi < MINIMUM_RSSI_6G) ||
#endif
		(prCandBss->eBand == BAND_2G4 && cCandRssi < MINIMUM_RSSI_2G4))
		return TRUE;

	/* 1.4 prefer to select 5G Bss if Rssi of a 5G band BSS is good */
	if (eRoamType != ROAM_TYPE_PER &&
		prCandBss->eBand != prCurrBss->eBand) {
		if (prCandBss->eBand >= BAND_2G4 &&
#if (CFG_SUPPORT_WIFI_6G == 1)
			prCandBss->eBand <= BAND_6G &&
#else
			prCandBss->eBand <= BAND_5G &&
#endif
			networkReplaceHandler[prCandBss->eBand].
			pfnNetworkSelection(
			prCurrBss->eBand, cCandRssi, cCurrRssi))
			return TRUE;
	}

	/* 1.5 RSSI of Current Bss is lower than Candidate, don't replace
	 * If the lower Rssi is greater than -59dbm,
	 * then no need check the difference
	 * Otherwise, if difference is greater than 10dbm, select the good RSSI
	 */
	 do {
		if ((eRoamType == ROAM_TYPE_PER) &&
			cCandRssi >= RSSI_SECOND_LEVEL &&
			cCurrRssi >= RSSI_SECOND_LEVEL)
			break;
		if (cCandRssi - cCurrRssi >= RSSI_DIFF_BETWEEN_BSS)
			return FALSE;
		if (cCurrRssi - cCandRssi >= RSSI_DIFF_BETWEEN_BSS)
			return TRUE;
	} while (FALSE);

	if (eRoamType == ROAM_TYPE_PER) {
		if (prCandBss->ucChannelNum == ucOpChannelNum) {
			log_dbg(SCN, INFO, "CandBss in opchnl,add CurBss Score\n");
			u2CurrScore += BSS_FULL_SCORE *
				gasMtkWeightConfig[eRoamType].ucOpchnlWeight;
		}
		if (prCurrBss->ucChannelNum == ucOpChannelNum) {
			log_dbg(SCN, INFO, "CurrBss in opchnl,add CandBss Score\n");
			u2CandScore += BSS_FULL_SCORE *
				gasMtkWeightConfig[eRoamType].ucOpchnlWeight;
		}
	}

	/* 2. Check Score */
	/* 2.1 Cases that no need to replace candidate */
	if (prCandBss->fgIsConnected) {
		if ((u2CandScore + ROAMING_NO_SWING_SCORE_STEP) >= u2CurrScore)
			return FALSE;
	} else if (prCurrBss->fgIsConnected) {
		if (u2CandScore >= (u2CurrScore + ROAMING_NO_SWING_SCORE_STEP))
			return FALSE;
	} else if (u2CandScore >= u2CurrScore)
		return FALSE;
	/* 2.2 other cases, replace candidate */
	return TRUE;
}

static u_int8_t scanSanityCheckBssDesc(struct ADAPTER *prAdapter,
	struct BSS_DESC *prBssDesc, enum ENUM_BAND eBand, uint8_t ucChannel,
	u_int8_t fgIsFixedChannel, enum ENUM_ROAMING_REASON eRoamReason,
	uint8_t ucBssIndex)
{
	struct BSS_INFO *prAisBssInfo;
	struct BSS_DESC *target;

#if CFG_SUPPORT_MBO
	struct PARAM_BSS_DISALLOWED_LIST *disallow;
	uint32_t i = 0;

	disallow = &prAdapter->rWifiVar.rBssDisallowedList;
	for (i = 0; i < disallow->u4NumBssDisallowed; ++i) {
		uint32_t index = i * MAC_ADDR_LEN;

		if (EQUAL_MAC_ADDR(prBssDesc->aucBSSID,
				&disallow->aucList[index])) {
			log_dbg(SCN, WARN, MACSTR" disallowed list\n",
				MAC2STR(prBssDesc->aucBSSID));
			return FALSE;
		}
	}

	if (prBssDesc->fgIsDisallowed) {
		log_dbg(SCN, WARN, MACSTR" disallowed\n",
			MAC2STR(prBssDesc->aucBSSID));
		return FALSE;
	}

	if (prBssDesc->prBlack && prBssDesc->prBlack->fgDisallowed &&
	    !(prBssDesc->prBlack->i4RssiThreshold > 0 &&
	      RCPI_TO_dBm(prBssDesc->ucRCPI) >
			prBssDesc->prBlack->i4RssiThreshold)) {
		log_dbg(SCN, WARN, MACSTR" disallowed delay, rssi %d(%d)\n",
			MAC2STR(prBssDesc->aucBSSID),
			RCPI_TO_dBm(prBssDesc->ucRCPI),
			prBssDesc->prBlack->i4RssiThreshold);
		return FALSE;
	}

	if (prBssDesc->prBlack && prBssDesc->prBlack->fgDisallowed) {
		log_dbg(SCN, WARN, MACSTR" disallowed delay\n",
			MAC2STR(prBssDesc->aucBSSID));
		return FALSE;
	}
#endif

	prAisBssInfo = aisGetAisBssInfo(prAdapter, ucBssIndex);
	target = aisGetTargetBssDesc(prAdapter, ucBssIndex);
	if (prBssDesc->prBlack) {
		if (prBssDesc->prBlack->fgIsInFWKBlacklist) {
			log_dbg(SCN, WARN, MACSTR" in FWK blacklist\n",
				MAC2STR(prBssDesc->aucBSSID));
			return FALSE;
		}

		if (prBssDesc->prBlack->fgDeauthLastTime) {
			log_dbg(SCN, WARN, MACSTR " is sending deauth.\n",
				MAC2STR(prBssDesc->aucBSSID));
			return FALSE;
		}

		if (prBssDesc->prBlack->ucCount >= 10)  {
			log_dbg(SCN, WARN,
				MACSTR
				" Skip AP that add toblacklist count >= 10\n",
				MAC2STR(prBssDesc->aucBSSID));
			return FALSE;
		}
	}

	/* roaming case */
	if (target &&
	   (prAisBssInfo->eConnectionState == MEDIA_STATE_CONNECTED ||
	    aisFsmIsInProcessPostpone(prAdapter, ucBssIndex))) {
		int32_t r1, r2;

		r1 = RCPI_TO_dBm(target->ucRCPI);
		r2 = RCPI_TO_dBm(prBssDesc->ucRCPI);
		if (prBssDesc->ucRCPI < RCPI_FOR_DONT_ROAM
#if CFG_SUPPORT_NCHO
		|| (prAdapter->rNchoInfo.fgNCHOEnabled &&
		    (eRoamReason == ROAMING_REASON_POOR_RCPI ||
		     eRoamReason == ROAMING_REASON_RETRY) &&
		    r1 - r2 <= prAdapter->rNchoInfo.i4RoamDelta)
#endif
		) {
			log_dbg(SCN, INFO, MACSTR " low rssi %d (cur=%d)\n",
				MAC2STR(prBssDesc->aucBSSID), r2, r1);
			return FALSE;
		}
	}

	if (ucBssIndex != AIS_DEFAULT_INDEX) {
		struct BSS_DESC *target =
			aisGetTargetBssDesc(prAdapter, AIS_DEFAULT_INDEX);

		if (target && prBssDesc->eBand == target->eBand) {
			log_dbg(SCN, WARN,
				MACSTR" used the same band with main\n",
				MAC2STR(prBssDesc->aucBSSID));
			if (!prAdapter->rWifiVar.fgAllowSameBandDualSta)
				return FALSE;
		}
	}

	if (!(prBssDesc->ucPhyTypeSet &
		(prAdapter->rWifiVar.ucAvailablePhyTypeSet))) {
		log_dbg(SCN, WARN,
			MACSTR" ignore unsupported ucPhyTypeSet = %x\n",
			MAC2STR(prBssDesc->aucBSSID),
			prBssDesc->ucPhyTypeSet);
		return FALSE;
	}
	if (prBssDesc->fgIsUnknownBssBasicRate) {
		log_dbg(SCN, WARN, MACSTR" unknown bss basic rate\n",
			MAC2STR(prBssDesc->aucBSSID));
		return FALSE;
	}
	if (fgIsFixedChannel &&	(eBand != prBssDesc->eBand || ucChannel !=
		prBssDesc->ucChannelNum)) {
		log_dbg(SCN, WARN,
			MACSTR" fix channel required band %d, channel %d\n",
			MAC2STR(prBssDesc->aucBSSID), eBand, ucChannel);
		return FALSE;
	}

#if CFG_SUPPORT_WIFI_SYSDVT
	if (!IS_SKIP_CH_CHECK(prAdapter))
#endif
	if (!rlmDomainIsLegalChannel(prAdapter, prBssDesc->eBand,
		prBssDesc->ucChannelNum)) {
		log_dbg(SCN, WARN, MACSTR" band %d channel %d is not legal\n",
			MAC2STR(prBssDesc->aucBSSID), prBssDesc->eBand,
			prBssDesc->ucChannelNum);
		return FALSE;
	}
	if (CHECK_FOR_TIMEOUT(kalGetTimeTick(), prBssDesc->rUpdateTime,
		SEC_TO_SYSTIME(wlanWfdEnabled(prAdapter) ?
		SCN_BSS_DESC_STALE_SEC_WFD : SCN_BSS_DESC_STALE_SEC))) {
		log_dbg(SCN, WARN, MACSTR " description is too old.\n",
			MAC2STR(prBssDesc->aucBSSID));
		return FALSE;
	}

#if CFG_SUPPORT_WAPI
	if (aisGetWapiMode(prAdapter, ucBssIndex)) {
		if (!wapiPerformPolicySelection(prAdapter, prBssDesc,
			ucBssIndex)) {
			log_dbg(SCN, WARN, MACSTR " wapi policy select fail.\n",
				MAC2STR(prBssDesc->aucBSSID));
			return FALSE;
		}
	} else
#endif
	if (!rsnPerformPolicySelection(prAdapter, prBssDesc,
		ucBssIndex)) {
		log_dbg(SCN, WARN, MACSTR " rsn policy select fail.\n",
			MAC2STR(prBssDesc->aucBSSID));
		return FALSE;
	}
	if (aisGetAisSpecBssInfo(prAdapter,
		ucBssIndex)->fgCounterMeasure) {
		log_dbg(SCN, WARN, MACSTR " Skip in counter measure period.\n",
			MAC2STR(prBssDesc->aucBSSID));
		return FALSE;
	}


#if CFG_SUPPORT_802_11K
	if (eRoamReason == ROAMING_REASON_BTM ||
	    eRoamReason == ROAMING_REASON_BTM_DISASSOC) {
		struct BSS_TRANSITION_MGT_PARAM *prBtmParam;
		uint8_t ucRequestMode = 0;

		prBtmParam = aisGetBTMParam(prAdapter, ucBssIndex);
		ucRequestMode = prBtmParam->ucRequestMode;
		if (ucRequestMode & WNM_BSS_TM_REQ_ABRIDGED) {
			if (!prBssDesc->prNeighbor) {
				log_dbg(SCN, INFO,
				     MACSTR " not in candidate list, skip it\n",
				     MAC2STR(prBssDesc->aucBSSID));
				return FALSE;
			}
		}
	}
#endif
	return TRUE;
}

#if (CFG_TC10_FEATURE == 1)
static int32_t scanCalculateScoreByCu(IN struct ADAPTER *prAdapter,
	IN struct BSS_DESC *prBssDesc, IN uint8_t ucBssIndex)
{
	struct SCAN_INFO *info;
	struct SCAN_PARAM *param;
	struct BSS_INFO *bss;
	int32_t score, rssi, cu = 0, cuRatio, dwell;
	uint32_t rssiFactor, cuFactor, rssiWeight, cuWeight;
	uint32_t slot = 0, idle;
	uint8_t i;

	if (!prBssDesc ||
	    (prBssDesc->prBlack && prBssDesc->prBlack->fgDeauthLastTime))
		return -1;
#if CFG_SUPPORT_NCHO
	if (prAdapter->rNchoInfo.fgNCHOEnabled)
		return -1;
#endif
	rssi = RCPI_TO_dBm(prBssDesc->ucRCPI);
	rssiWeight = 65;
	cuWeight = 35;
	if (rssi >= -55)
		rssiFactor = 100;
	else if (rssi < -55 && rssi >= -60)
		rssiFactor = 90 + 2 * (60 + rssi);
	else if (rssi < -60 && rssi >= -70)
		rssiFactor = 60 + 3 * (70 + rssi);
	else if (rssi < -70 && rssi >= -80)
		rssiFactor = 20 + 4 * (80 + rssi);
	else if (rssi < -80 && rssi >= -90)
		rssiFactor = 2 * (90 + rssi);
	else
		rssiFactor = 0;
	if (prBssDesc->fgExsitBssLoadIE) {
		cu = prBssDesc->ucChnlUtilization;
	} else {
		bss = aisGetAisBssInfo(prAdapter, ucBssIndex);
		info = &(prAdapter->rWifiVar.rScanInfo);
		param = &(info->rScanParam);
		if (param->u2ChannelDwellTime > 0)
			dwell = param->u2ChannelDwellTime;
		else if (bss->eConnectionState == MEDIA_STATE_CONNECTED)
			dwell = CHNL_DWELL_TIME_ONLINE;
		else
			dwell = CHNL_DWELL_TIME_DEFAULT;
		for (i = 0; i < info->ucSparseChannelArrayValidNum; i++) {
			if (prBssDesc->ucChannelNum == info->aucChannelNum[i]) {
				slot = info->au2ChannelIdleTime[i];
				idle = (slot * 9 * 100) / (dwell * 1000);
				cu = 255 - idle * 255 / 100;
				break;
			}
		}
	}
	cuRatio = cu * 100 / 255;
	if (prBssDesc->eBand == BAND_2G4) {
		if (cuRatio < 10)
			cuFactor = 100;
		else if (cuRatio < 70 && cuRatio >= 10)
			cuFactor = 111 - (13 * cuRatio / 10);
		else
			cuFactor = 20;
	} else {
		if (cuRatio < 30)
			cuFactor = 100;
		else if (cuRatio < 80 && cuRatio >= 30)
			cuFactor = 148 - (16 * cuRatio / 10);
		else
			cuFactor = 20;
	}
	score = rssiFactor * rssiWeight + cuFactor * cuWeight;
	log_dbg(SCN, INFO,
		MACSTR
		" 5G[%d],chl[%d],slt[%d],ld[%d] Basic Score %d,rssi[%d],cu[%d],cuR[%d],rf[%d],rw[%d],cf[%d],cw[%d]\n",
		MAC2STR(prBssDesc->aucBSSID),
		(prBssDesc->eBand == BAND_5G ? 1 : 0),
		prBssDesc->ucChannelNum, slot,
		prBssDesc->fgExsitBssLoadIE, score, rssi, cu, cuRatio,
		rssiFactor, rssiWeight,	cuFactor, cuWeight);
	return score;
}
#endif

static uint16_t scanCalculateScoreByRssi(struct BSS_DESC *prBssDesc,
	enum ROAM_TYPE eRoamType)
{
	uint16_t u2Score = 0;
	int8_t cRssi = RCPI_TO_dBm(prBssDesc->ucRCPI);

	if (cRssi >= BEST_RSSI)
		u2Score = 100;
	else if (cRssi <= -98)
		u2Score = 0;
	else
		u2Score = (cRssi + 98) * 2;
	u2Score *= gasMtkWeightConfig[eRoamType].ucRssiWeight;

	return u2Score;
}

static uint16_t scanCalculateScoreBySaa(struct ADAPTER *prAdapter,
	struct BSS_DESC *prBssDesc, enum ROAM_TYPE eRoamType)
{
	uint16_t u2Score = 0;
	struct STA_RECORD *prStaRec = (struct STA_RECORD *) NULL;

	prStaRec = cnmGetStaRecByAddress(prAdapter, NETWORK_TYPE_AIS,
		prBssDesc->aucSrcAddr);
	if (prStaRec)
		u2Score = gasMtkWeightConfig[eRoamType].ucSaaWeight *
		(prStaRec->ucTxAuthAssocRetryCount ? 0 : BSS_FULL_SCORE);
	else
		u2Score = gasMtkWeightConfig[eRoamType].ucSaaWeight *
		BSS_FULL_SCORE;

	return u2Score;
}

static uint16_t scanCalculateScoreByIdleTime(struct ADAPTER *prAdapter,
	uint8_t ucChannel, enum ROAM_TYPE eRoamType,
	IN struct BSS_DESC *prBssDesc, uint8_t ucBssIndex,
	enum ENUM_BAND eBand)
{
	struct SCAN_INFO *info;
	struct SCAN_PARAM *param;
	struct BSS_INFO *bss;
	int32_t score, rssi, cu = 0, cuRatio, dwell;
	uint32_t rssiFactor, cuFactor, rssiWeight, cuWeight;
	uint32_t slot = 0, idle;
	uint8_t i;

	rssi = RCPI_TO_dBm(prBssDesc->ucRCPI);
	rssiWeight = 65;
	cuWeight = 35;
	if (rssi >= -55)
		rssiFactor = 100;
	else if (rssi < -55 && rssi >= -60)
		rssiFactor = 90 + 2 * (60 + rssi);
	else if (rssi < -60 && rssi >= -70)
		rssiFactor = 60 + 3 * (70 + rssi);
	else if (rssi < -70 && rssi >= -80)
		rssiFactor = 20 + 4 * (80 + rssi);
	else if (rssi < -80 && rssi >= -90)
		rssiFactor = 2 * (90 + rssi);
	else
		rssiFactor = 0;

	if (prBssDesc->fgExsitBssLoadIE) {
		cu = prBssDesc->ucChnlUtilization;
	} else {
		bss = aisGetAisBssInfo(prAdapter, ucBssIndex);
		info = &(prAdapter->rWifiVar.rScanInfo);
		param = &(info->rScanParam);

		if (param->u2ChannelDwellTime > 0)
			dwell = param->u2ChannelDwellTime;
		else if (bss->eConnectionState == MEDIA_STATE_CONNECTED)
			dwell = CHNL_DWELL_TIME_ONLINE;
		else
			dwell = CHNL_DWELL_TIME_DEFAULT;

		for (i = 0; i < info->ucSparseChannelArrayValidNum; i++) {
			if (prBssDesc->ucChannelNum == info->aucChannelNum[i] &&
					eBand == info->aeChannelBand[i]) {
				slot = info->au2ChannelIdleTime[i];
				idle = (slot * 9 * 100) / (dwell * 1000);
				if (eRoamType == ROAM_TYPE_PER) {
					score = idle > BSS_FULL_SCORE ?
						BSS_FULL_SCORE : idle;
					goto done;
				}
				cu = 255 - idle * 255 / 100;
				break;
			}
		}
	}

	cuRatio = cu * 100 / 255;
	if (prBssDesc->eBand == BAND_2G4) {
		if (cuRatio < 10)
			cuFactor = 100;
		else if (cuRatio < 70 && cuRatio >= 10)
			cuFactor = 111 - (13 * cuRatio / 10);
		else
			cuFactor = 20;
	} else {
		if (cuRatio < 30)
			cuFactor = 100;
		else if (cuRatio < 80 && cuRatio >= 30)
			cuFactor = 148 - (16 * cuRatio / 10);
		else
			cuFactor = 20;
	}

	score = (rssiFactor * rssiWeight + cuFactor * cuWeight) >> 6;

	log_dbg(SCN, TRACE,
		MACSTR
		" Band[%s],chl[%d],slt[%d],ld[%d] idle Score %d,rssi[%d],cu[%d],cuR[%d],rf[%d],rw[%d],cf[%d],cw[%d]\n",
		MAC2STR(prBssDesc->aucBSSID),
		apucBandStr[prBssDesc->eBand],
		prBssDesc->ucChannelNum, slot,
		prBssDesc->fgExsitBssLoadIE, score, rssi, cu, cuRatio,
		rssiFactor, rssiWeight, cuFactor, cuWeight);
done:
	return score * gasMtkWeightConfig[eRoamType].ucChnlIdleWeight;
}

u_int8_t scanApOverload(uint16_t status, uint16_t reason)
{
	switch (status) {
	case STATUS_CODE_ASSOC_DENIED_AP_OVERLOAD:
	case STATUS_CODE_ASSOC_DENIED_BANDWIDTH:
	case STATUS_CODE_ASSOC_DENIED_OUTSIDE_STANDARD:
	case STATUS_CODE_AUTH_TIMEOUT:
	case STATUS_CODE_ASSOC_TIMEOUT:
		return TRUE;
	}
	switch (reason) {
	case REASON_CODE_DISASSOC_LACK_OF_BANDWIDTH:
	case REASON_CODE_DISASSOC_AP_OVERLOAD:
		return TRUE;
	}
	return FALSE;
}

uint16_t scanCalculateScoreByBlackList(struct ADAPTER *prAdapter,
	    struct BSS_DESC *prBssDesc, enum ROAM_TYPE eRoamType)
{
	uint16_t u2Score = 0;

	if (!prBssDesc->prBlack)
		u2Score = 100;
	else if (scanApOverload(prBssDesc->prBlack->u2AuthStatus,
		prBssDesc->prBlack->u2DeauthReason) ||
		 prBssDesc->prBlack->ucCount >= 10)
		u2Score = 0;
	else
		u2Score = 100 - prBssDesc->prBlack->ucCount * 10;

	return u2Score * gasMtkWeightConfig[eRoamType].ucBlackListWeight;
}

uint16_t scanCalculateScoreByTput(struct ADAPTER *prAdapter,
	    struct BSS_DESC *prBssDesc, enum ROAM_TYPE eRoamType)
{
	uint16_t u2Score = 0;

#if CFG_SUPPORT_MBO
	if (prBssDesc->fgExistEspIE)
		u2Score = (prBssDesc->u4EspInfo[WIFI_AC_BE] >> 8) & 0xff;
#endif
	return u2Score * gasMtkWeightConfig[eRoamType].ucTputWeight;
}

uint16_t scanCalculateScoreByPreference(struct ADAPTER *prAdapter,
	    struct BSS_DESC *prBssDesc, enum ENUM_ROAMING_REASON eRoamReason)
{
#if CFG_SUPPORT_802_11K
	if (eRoamReason == ROAMING_REASON_BTM ||
	    eRoamReason == ROAMING_REASON_BTM_DISASSOC) {
		if (prBssDesc->prNeighbor) {
			enum ROAM_TYPE eRoamType =
				roamReasonToType[eRoamReason];

			return prBssDesc->prNeighbor->ucPreference *
			       gasMtkWeightConfig[eRoamType].ucPreferenceWeight;
		}
	}
#endif
	return 0;
}

uint16_t scanCalculateTotalScore(struct ADAPTER *prAdapter,
	struct BSS_DESC *prBssDesc, enum ENUM_ROAMING_REASON eRoamReason,
	uint8_t ucBssIndex)
{
	struct AIS_SPECIFIC_BSS_INFO *prAisSpecificBssInfo = NULL;
	uint16_t u2ScoreStaCnt = 0;
	uint16_t u2ScoreBandwidth = 0;
	uint16_t u2ScoreSTBC = 0;
	uint16_t u2ScoreChnlInfo = 0;
	uint16_t u2ScoreSnrRssi = 0;
	uint16_t u2ScoreDeauth = 0;
	uint16_t u2ScoreProbeRsp = 0;
	uint16_t u2ScoreScanMiss = 0;
	uint16_t u2ScoreBand = 0;
	uint16_t u2ScoreSaa = 0;
	uint16_t u2ScoreIdleTime = 0;
	uint16_t u2ScoreTotal = 0;
	uint16_t u2BlackListScore = 0;
	uint16_t u2PreferenceScore = 0;
	uint16_t u2AxApScore = 0;
	uint16_t u2TputScore = 0;
#if (CFG_SUPPORT_AVOID_DESENSE == 1)
	uint8_t fgBssInDenseRange =
		IS_CHANNEL_IN_DESENSE_RANGE(prAdapter,
		prBssDesc->ucChannelNum,
		prBssDesc->eBand);
	char extra[16] = {0};
#else
	char *extra = "";
#endif
	int8_t cRssi = -128;
	enum ROAM_TYPE eRoamType = roamReasonToType[eRoamReason];

	prAisSpecificBssInfo = aisGetAisSpecBssInfo(prAdapter, ucBssIndex);
	cRssi = RCPI_TO_dBm(prBssDesc->ucRCPI);

	u2ScoreBandwidth =
		scanCalculateScoreByBandwidth(prAdapter, prBssDesc, eRoamType);
	u2ScoreStaCnt = scanCalculateScoreByClientCnt(prBssDesc, eRoamType);
	u2ScoreSTBC = CALCULATE_SCORE_BY_STBC(prAdapter, prBssDesc, eRoamType);
	u2ScoreChnlInfo = scanCalculateScoreByChnlInfo(prAisSpecificBssInfo,
				prBssDesc->ucChannelNum, eRoamType);
	u2ScoreSnrRssi = scanCalculateScoreByRssi(prBssDesc, eRoamType);
	u2ScoreDeauth = CALCULATE_SCORE_BY_DEAUTH(prBssDesc, eRoamType);
	u2ScoreProbeRsp = CALCULATE_SCORE_BY_PROBE_RSP(prBssDesc, eRoamType);
	u2ScoreScanMiss = CALCULATE_SCORE_BY_MISS_CNT(prAdapter,
		prBssDesc, eRoamType);
	u2ScoreBand = CALCULATE_SCORE_BY_BAND(prAdapter, prBssDesc,
		cRssi, eRoamType);
	u2ScoreSaa = scanCalculateScoreBySaa(prAdapter, prBssDesc, eRoamType);
	u2ScoreIdleTime = scanCalculateScoreByIdleTime(prAdapter,
		prBssDesc->ucChannelNum, eRoamType, prBssDesc, ucBssIndex,
		prBssDesc->eBand);
	u2BlackListScore =
	       scanCalculateScoreByBlackList(prAdapter, prBssDesc, eRoamType);
	u2PreferenceScore =
	      scanCalculateScoreByPreference(prAdapter, prBssDesc, eRoamReason);

#if (CFG_SUPPORT_802_11AX == 1)
	u2AxApScore = CALCULATE_SCORE_BY_AX_AP(prAdapter, prBssDesc, eRoamType);
#endif
#if (CFG_SUPPORT_802_11BE == 1)
	/* TODO */
#endif
	u2TputScore = scanCalculateScoreByTput(prAdapter, prBssDesc, eRoamType);

	u2ScoreTotal = u2ScoreBandwidth + u2ScoreChnlInfo +
		u2ScoreDeauth + u2ScoreProbeRsp + u2ScoreScanMiss +
		u2ScoreSnrRssi + u2ScoreStaCnt + u2ScoreSTBC +
		u2ScoreBand + u2BlackListScore + u2ScoreSaa +
		u2ScoreIdleTime + u2AxApScore + u2TputScore;

#if (CFG_SUPPORT_AVOID_DESENSE == 1)
	if (fgBssInDenseRange)
		u2ScoreTotal /= 4;
	kalSnprintf(extra, sizeof(extra), ", DESENSE[%d]", fgBssInDenseRange);
#endif

#define TEMP_LOG_TEMPLATE\
		MACSTR" cRSSI[%d] Band[%s] Score,Total %d,DE[%d]"\
		", PR[%d], SM[%d], RSSI[%d],BD[%d],BL[%d],SAA[%d]"\
		", BW[%d], SC[%d],ST[%d],CI[%d],IT[%d],CU[%d,%d],PF[%d]"\
		", AX[%d], TPUT[%d]%s\n"

	log_dbg(SCN, INFO,
		TEMP_LOG_TEMPLATE,
		MAC2STR(prBssDesc->aucBSSID), cRssi,
		apucBandStr[prBssDesc->eBand], u2ScoreTotal,
		u2ScoreDeauth, u2ScoreProbeRsp, u2ScoreScanMiss,
		u2ScoreSnrRssi, u2ScoreBand, u2BlackListScore,
		u2ScoreSaa, u2ScoreBandwidth, u2ScoreStaCnt,
		u2ScoreSTBC, u2ScoreChnlInfo, u2ScoreIdleTime,
		prBssDesc->fgExsitBssLoadIE,
		prBssDesc->ucChnlUtilization,
		u2PreferenceScore,
#if (CFG_SUPPORT_AVOID_DESENSE == 1)
		u2AxApScore, u2TputScore, extra, fgBssInDenseRange);
#else
		u2AxApScore, u2TputScore, extra);
#endif

#undef TEMP_LOG_TEMPLATE

	return u2ScoreTotal;
}
/*
 * Bss Characteristics to be taken into account when calculate Score:
 * Channel Loading Group:
 * 1. Client Count (in BSS Load IE).
 * 2. AP number on the Channel.
 *
 * RF Group:
 * 1. Channel utilization.
 * 2. SNR.
 * 3. RSSI.
 *
 * Misc Group:
 * 1. Deauth Last time.
 * 2. Scan Missing Count.
 * 3. Has probe response in scan result.
 *
 * Capability Group:
 * 1. Prefer 5G band.
 * 2. Bandwidth.
 * 3. STBC and Multi Anttena.
 */
struct BSS_DESC *scanSearchBssDescByScoreForAis(struct ADAPTER *prAdapter,
	enum ENUM_ROAMING_REASON eRoamReason, uint8_t ucBssIndex)
{
	struct AIS_SPECIFIC_BSS_INFO *prAisSpecificBssInfo = NULL;
	struct ROAMING_INFO *prRoamingFsmInfo = NULL;
	struct BSS_INFO *prAisBssInfo = NULL;
	struct LINK *prEssLink = NULL;
	struct CONNECTION_SETTINGS *prConnSettings = NULL;
	struct BSS_DESC *prBssDesc = NULL;
	struct BSS_DESC *prCandBssDesc = NULL;
	struct BSS_DESC *prCandBssDescForLowRssi = NULL;
	uint16_t u2ScoreTotal = 0;
	uint16_t u2CandBssScore = 0;
	uint16_t u2CandBssScoreForLowRssi = 0;
	u_int8_t fgSearchBlackList = FALSE;
	u_int8_t fgIsFixedChnl = FALSE;
	enum ENUM_BAND eBand = BAND_2G4;
	uint8_t ucChannel = 0;
	enum ENUM_PARAM_CONNECTION_POLICY policy;
	struct ROAMING_INFO *roam;
	enum ROAM_TYPE eRoamType;
#if (CFG_TC10_FEATURE == 1)
	int32_t base, delta, goal;
#endif

	if (!prAdapter ||
	    eRoamReason >= ROAMING_REASON_NUM || eRoamReason < 0) {
		log_dbg(SCN, ERROR,
			"prAdapter %p, reason %d!\n", prAdapter, eRoamReason);
		return NULL;
	}
	prAisSpecificBssInfo = aisGetAisSpecBssInfo(prAdapter, ucBssIndex);
	prConnSettings = aisGetConnSettings(prAdapter, ucBssIndex);
	prEssLink = &prAisSpecificBssInfo->rCurEssLink;
	prRoamingFsmInfo = aisGetRoamingInfo(prAdapter, ucBssIndex);
	prAisBssInfo = aisGetAisBssInfo(prAdapter, ucBssIndex);
	roam = aisGetRoamingInfo(prAdapter, ucBssIndex);
	eRoamType = roamReasonToType[eRoamReason];
#if CFG_SUPPORT_CHNL_CONFLICT_REVISE
	fgIsFixedChnl =	cnmAisDetectP2PChannel(prAdapter, &eBand, &ucChannel);
#else
	fgIsFixedChnl =	cnmAisInfraChannelFixed(prAdapter, &eBand, &ucChannel);
#endif
	aisRemoveTimeoutBlacklist(prAdapter);
#if CFG_SUPPORT_802_11K
	/* check before using neighbor report */
	aisCheckNeighborApValidity(prAdapter, prAisBssInfo->ucBssIndex);
#endif
	log_dbg(SCN, INFO, "ConnectionPolicy = %d, reason = %d\n",
		prConnSettings->eConnectionPolicy, eRoamReason);
	policy = prConnSettings->eConnectionPolicy;
#if (CFG_TC10_FEATURE == 1)
	base = (eRoamReason == ROAMING_REASON_BTM_DISASSOC) ? 6000 :
		scanCalculateScoreByCu(prAdapter,
			aisGetTargetBssDesc(prAdapter, ucBssIndex), ucBssIndex);
#endif
try_again:
	LINK_FOR_EACH_ENTRY(prBssDesc, prEssLink, rLinkEntryEss[ucBssIndex],
		struct BSS_DESC) {
		if (!fgSearchBlackList) {
			/* update blacklist info */
			prBssDesc->prBlack =
				aisQueryBlackList(prAdapter, prBssDesc);
#if CFG_SUPPORT_802_11K
			/* update neighbor report entry */
			prBssDesc->prNeighbor = scanGetNeighborAPEntry(
				prAdapter, prBssDesc->aucBSSID, ucBssIndex);
#endif
		}
		/*
		 * Skip if
		 * 1. sanity check fail or
		 * 2. bssid is in driver's blacklist in the first round
		 */
		if (!scanSanityCheckBssDesc(prAdapter, prBssDesc, eBand,
			ucChannel, fgIsFixedChnl, eRoamReason, ucBssIndex) ||
		    (!fgSearchBlackList && prBssDesc->prBlack))
			continue;

		/* pick by bssid first */
		if (policy == CONNECT_BY_BSSID) {
			if (EQUAL_MAC_ADDR(prBssDesc->aucBSSID,
					prConnSettings->aucBSSID)) {
				log_dbg(SCN, INFO, MACSTR" match bssid\n",
					MAC2STR(prBssDesc->aucBSSID));
				prCandBssDesc = prBssDesc;
				break;
			}
			/* skip when bssid unmatched if conn by bssid */
			log_dbg(SCN, INFO, MACSTR" unmatch bssid\n",
				MAC2STR(prBssDesc->aucBSSID));
			continue;
		} else if (policy == CONNECT_BY_BSSID_HINT) {
			uint8_t oce = FALSE;

#if CFG_SUPPORT_MBO
			oce = prAdapter->rWifiVar.u4SwTestMode ==
				ENUM_SW_TEST_MODE_SIGMA_OCE;
#endif
			if (!oce && EQUAL_MAC_ADDR(prBssDesc->aucBSSID,
					prConnSettings->aucBSSIDHint)) {
#if (CFG_SUPPORT_AVOID_DESENSE == 1)
				if (IS_CHANNEL_IN_DESENSE_RANGE(prAdapter,
					prBssDesc->ucChannelNum,
					prBssDesc->eBand)) {
					log_dbg(SCN, INFO,
						"Do network selection even match bssid_hint\n");
				} else
#endif
				{
					log_dbg(SCN, INFO,
						MACSTR" match bssid_hint\n",
						MAC2STR(prBssDesc->aucBSSID));
					prCandBssDesc = prBssDesc;
					break;
				}
			}
		}

#if (CFG_TC10_FEATURE == 1)
		if (base > 0) {
			if (UNEQUAL_MAC_ADDR(prBssDesc->aucBSSID,
						prAisBssInfo->aucBSSID)) {
				u2ScoreTotal = scanCalculateScoreByCu(
					prAdapter, prBssDesc, ucBssIndex);
				switch (eRoamReason) {
				case ROAMING_REASON_POOR_RCPI:
				case ROAMING_REASON_RETRY:
					delta = 20;
					break;
				case ROAMING_REASON_IDLE:
					delta = 1;
					break;
				default:
					delta = 0;
				}
				goal = base + base * delta / 100;
				if (u2ScoreTotal < goal) {
					log_dbg(SCN, WARN,
					   MACSTR " reason %d, score %d < %d\n",
					   MAC2STR(prBssDesc->aucBSSID),
					   eRoamReason, u2ScoreTotal, goal);
					continue;
				}
			} else if (eRoamReason == ROAMING_REASON_BTM_DISASSOC) {
				log_dbg(SCN, WARN,
					MACSTR " reason %d, given zero\n",
					MAC2STR(prBssDesc->aucBSSID),
					eRoamReason);
				continue;
			}
		}
#endif

		u2ScoreTotal = scanCalculateTotalScore(prAdapter, prBssDesc,
			prRoamingFsmInfo->eReason, ucBssIndex);
		if (!prCandBssDesc ||
			scanNeedReplaceCandidate(prAdapter, prCandBssDesc,
			prBssDesc, u2CandBssScore, u2ScoreTotal,
			prRoamingFsmInfo->eReason, ucBssIndex)) {
			prCandBssDesc = prBssDesc;
			u2CandBssScore = u2ScoreTotal;
		}
	}

	if (prCandBssDesc) {
		if (prCandBssDesc->fgIsConnected && !fgSearchBlackList &&
			prEssLink->u4NumElem > 0) {
			fgSearchBlackList = TRUE;
			log_dbg(SCN, INFO, "Can't roam out, try blacklist\n");
			goto try_again;
		}

		if (prConnSettings->eConnectionPolicy == CONNECT_BY_BSSID)
			log_dbg(SCN, INFO, "Selected "
				MACSTR
				" %d base on ssid,when find %s, "
				MACSTR
				" in %d bssid,fix channel %d.\n",
				MAC2STR(prCandBssDesc->aucBSSID),
				RCPI_TO_dBm(prCandBssDesc->ucRCPI),
				HIDE(prConnSettings->aucSSID),
				MAC2STR(prConnSettings->aucBSSID),
				prEssLink->u4NumElem, ucChannel);
		else
			log_dbg(SCN, INFO,
				"Selected "
				MACSTR
				", cRSSI[%d] Band[%s] Score %d when find %s, "
				MACSTR
				" in %d BSSes, fix channel %d.\n",
				MAC2STR(prCandBssDesc->aucBSSID),
				RCPI_TO_dBm(prCandBssDesc->ucRCPI),
				apucBandStr[prCandBssDesc->eBand],
				u2CandBssScore, prConnSettings->aucSSID,
				MAC2STR(prConnSettings->aucBSSID),
				prEssLink->u4NumElem, ucChannel);

		return prCandBssDesc;
	} else if (prCandBssDescForLowRssi) {
		log_dbg(SCN, INFO, "Selected " MACSTR
			", Score %d when find %s, " MACSTR
			" in %d BSSes, fix channel %d.\n",
			MAC2STR(prCandBssDescForLowRssi->aucBSSID),
			u2CandBssScoreForLowRssi, prConnSettings->aucSSID,
			MAC2STR(prConnSettings->aucBSSID), prEssLink->u4NumElem,
			ucChannel);
		return prCandBssDescForLowRssi;
	}

	/* if No Candidate BSS is found, try BSSes which are in blacklist */
	if (!fgSearchBlackList && prEssLink->u4NumElem > 0) {
		fgSearchBlackList = TRUE;
		log_dbg(SCN, INFO, "No Bss is found, Try blacklist\n");
		goto try_again;
	}
	log_dbg(SCN, INFO, "Selected None when find %s, " MACSTR
		" in %d BSSes, fix channel %d.\n",
		prConnSettings->aucSSID, MAC2STR(prConnSettings->aucBSSID),
		prEssLink->u4NumElem, ucChannel);
	return NULL;
}

uint8_t scanUpdateChannelList(uint8_t channel, enum ENUM_BAND eBand,
	uint8_t *bitmap, uint8_t *count, struct ESS_CHNL_INFO *info)
{
	uint8_t byteNum = 0;
	uint8_t bitNum = 0;

	byteNum = channel / 8;
	bitNum = channel % 8;
	if (bitmap[byteNum] & BIT(bitNum))
		return 1;
	bitmap[byteNum] |= BIT(bitNum);
	info[*count].ucChannel = channel;
	info[*count].eBand = eBand;
	*count += 1;
	if (*count >= CFG_MAX_NUM_OF_CHNL_INFO)
		return 0;

	return 1;
}

void scanGetCurrentEssChnlList(struct ADAPTER *prAdapter,
	uint8_t ucBssIndex)
{
	struct BSS_DESC *prBssDesc = NULL;
	struct LINK *prBSSDescList =
		&prAdapter->rWifiVar.rScanInfo.rBSSDescList;
	struct CONNECTION_SETTINGS *prConnSettings =
		aisGetConnSettings(prAdapter, ucBssIndex);
	struct ESS_CHNL_INFO *prEssChnlInfo;
	struct LINK *prCurEssLink;
	struct AIS_SPECIFIC_BSS_INFO *prAisSpecBssInfo;
	uint8_t aucChnlBitMap[30] = {0,};
	uint8_t aucChnlApNum[234] = {0,};
	uint8_t aucChnlUtil[234] = {0,};
	uint8_t ucChnlCount = 0;
	uint32_t i;
	uint8_t j = 0;
#if CFG_SUPPORT_802_11K
	struct LINK *prNeighborAPLink;
#endif
	struct CFG_SCAN_CHNL *prRoamScnChnl = &prAdapter->rAddRoamScnChnl;

	if (!prConnSettings)  {
		log_dbg(SCN, INFO, "No prConnSettings\n");
		return;
	}

	if (prConnSettings->ucSSIDLen == 0) {
		log_dbg(SCN, INFO, "No Ess are expected to connect\n");
		return;
	}

	prAisSpecBssInfo =
		aisGetAisSpecBssInfo(prAdapter, ucBssIndex);
	if (!prAisSpecBssInfo) {
		log_dbg(SCN, INFO, "No prAisSpecBssInfo\n");
		return;
	}
	prEssChnlInfo =
		&prAisSpecBssInfo->arCurEssChnlInfo[0];
	if (!prEssChnlInfo) {
		log_dbg(SCN, INFO, "No prEssChnlInfo\n");
		return;
	}
	prCurEssLink =
		&prAisSpecBssInfo->rCurEssLink;
	if (!prCurEssLink) {
		log_dbg(SCN, INFO, "No prCurEssLink\n");
		return;
	}

	kalMemZero(prEssChnlInfo, CFG_MAX_NUM_OF_CHNL_INFO *
		sizeof(struct ESS_CHNL_INFO));

	while (!LINK_IS_EMPTY(prCurEssLink)) {
		prBssDesc = LINK_PEEK_HEAD(prCurEssLink,
			struct BSS_DESC, rLinkEntryEss[ucBssIndex]);
		LINK_REMOVE_KNOWN_ENTRY(prCurEssLink,
			&prBssDesc->rLinkEntryEss[ucBssIndex]);
	}

	LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry,
		struct BSS_DESC) {
		if (prBssDesc->ucChannelNum > 233)
			continue;
		/* Statistic AP num for each channel */
		if (aucChnlApNum[prBssDesc->ucChannelNum] < 255)
			aucChnlApNum[prBssDesc->ucChannelNum]++;
		if (aucChnlUtil[prBssDesc->ucChannelNum] <
			prBssDesc->ucChnlUtilization)
			aucChnlUtil[prBssDesc->ucChannelNum] =
				prBssDesc->ucChnlUtilization;
		if (!EQUAL_SSID(prConnSettings->aucSSID,
			prConnSettings->ucSSIDLen,
			prBssDesc->aucSSID, prBssDesc->ucSSIDLen))
			continue;
		/* Record same BSS list */
		LINK_INSERT_HEAD(prCurEssLink,
			&prBssDesc->rLinkEntryEss[ucBssIndex]);

#if CFG_SUPPORT_NCHO
		/* scan control is 1: use NCHO channel list only */
		if (prAdapter->rNchoInfo.u4RoamScanControl)
			continue;
#endif
		if (!scanUpdateChannelList(prBssDesc->ucChannelNum,
			prBssDesc->eBand, aucChnlBitMap, &ucChnlCount,
			prEssChnlInfo))
			goto updated;
	}

#if CFG_SUPPORT_NCHO
	if (prAdapter->rNchoInfo.fgNCHOEnabled) {
		struct CFG_NCHO_SCAN_CHNL *ncho;

		if (prAdapter->rNchoInfo.u4RoamScanControl)
			ncho = &prAdapter->rNchoInfo.rRoamScnChnl;
		else
			ncho = &prAdapter->rNchoInfo.rAddRoamScnChnl;

		/* handle user-specefied scan channel info */
		for (i = 0; ucChnlCount < CFG_MAX_NUM_OF_CHNL_INFO &&
			i < ncho->ucChannelListNum; i++) {
			uint8_t chnl;
			enum ENUM_BAND eBand;

			chnl = ncho->arChnlInfoList[i].ucChannelNum;
			eBand = ncho->arChnlInfoList[i].eBand;
			if (!scanUpdateChannelList(chnl, eBand,
			    aucChnlBitMap, &ucChnlCount, prEssChnlInfo))
				goto updated;
		}

		if (prAdapter->rNchoInfo.u4RoamScanControl)
			goto updated;
	}
#endif

#if CFG_SUPPORT_802_11K
	prNeighborAPLink = &prAisSpecBssInfo->rNeighborApList.rUsingLink;
	if (!LINK_IS_EMPTY(prNeighborAPLink)) {
		/* Add channels provided by Neighbor Report to
		 ** channel list for roaming scanning.
		 */
		struct NEIGHBOR_AP *prNeiAP = NULL;
		enum ENUM_BAND eBand;
		uint8_t ucChannel;

		LINK_FOR_EACH_ENTRY(prNeiAP, prNeighborAPLink,
		    rLinkEntry, struct NEIGHBOR_AP) {
			ucChannel = prNeiAP->ucChannel;
			eBand = prNeiAP->eBand;
			if (!rlmDomainIsLegalChannel(
				prAdapter, eBand, ucChannel))
				continue;
			if (!scanUpdateChannelList(ucChannel, eBand,
				aucChnlBitMap, &ucChnlCount, prEssChnlInfo))
				goto updated;
		}
	}
#endif

	/* handle user-specefied scan channel info */
	for (i = 0; ucChnlCount < CFG_MAX_NUM_OF_CHNL_INFO &&
		i < prRoamScnChnl->ucChannelListNum; i++) {
		uint8_t chnl;
		enum ENUM_BAND eBand;

		chnl = prRoamScnChnl->arChnlInfoList[i].ucChannelNum;
		eBand = prRoamScnChnl->arChnlInfoList[i].eBand;
		if (!scanUpdateChannelList(chnl, eBand,
		    aucChnlBitMap, &ucChnlCount, prEssChnlInfo))
			goto updated;
	}

updated:

	prAisSpecBssInfo->ucCurEssChnlInfoNum = ucChnlCount;
	for (j = 0; j < ucChnlCount; j++) {
		uint8_t ucChnl = prEssChnlInfo[j].ucChannel;

		prEssChnlInfo[j].ucApNum = aucChnlApNum[ucChnl];
		prEssChnlInfo[j].ucUtilization = aucChnlUtil[ucChnl];
	}
#if 0
	/* Sort according to AP number */
	for (j = 0; j < ucChnlCount; j++) {
		for (i = j + 1; i < ucChnlCount; i++)
			if (prEssChnlInfo[j].ucApNum >
				prEssChnlInfo[i].ucApNum) {
				struct ESS_CHNL_INFO rTemp = prEssChnlInfo[j];

				prEssChnlInfo[j] = prEssChnlInfo[i];
				prEssChnlInfo[i] = rTemp;
			}
	}
#endif
	log_dbg(SCN, INFO, "Find %s in %d BSSes, result %d\n",
		prConnSettings->aucSSID, prBSSDescList->u4NumElem,
		prCurEssLink->u4NumElem);
}

uint8_t scanCheckNeedDriverRoaming(
	struct ADAPTER *prAdapter, uint8_t ucBssIndex)
{
	struct ROAMING_INFO *roam;
	struct AIS_FSM_INFO *ais;
	struct CONNECTION_SETTINGS *setting;
	int8_t rssi;

	roam = aisGetRoamingInfo(prAdapter, ucBssIndex);
	ais = aisGetAisFsmInfo(prAdapter, ucBssIndex);
	setting = aisGetConnSettings(prAdapter, ucBssIndex);
	rssi = prAdapter->rLinkQuality.rLq[ucBssIndex].cRssi;

	GET_CURRENT_SYSTIME(&roam->rRoamingDiscoveryUpdateTime);

#if CFG_SUPPORT_DRIVER_ROAMING
	/*
	 * try to select AP only when roaming is enabled and rssi is bad
	 */
	if (roamingFsmInDecision(prAdapter, ucBssIndex) &&
	    ais->eCurrentState == AIS_STATE_ONLINE_SCAN &&
	    CHECK_FOR_TIMEOUT(roam->rRoamingDiscoveryUpdateTime,
		      roam->rRoamingLastDecisionTime,
		      SEC_TO_SYSTIME(prAdapter->rWifiVar.u4InactiveTimeout))) {
		struct BSS_DESC *target;
		struct BSS_DESC *bss;

		target = aisGetTargetBssDesc(prAdapter, ucBssIndex);

		bss = scanSearchBssDescByScoreForAis(prAdapter,
			ROAMING_REASON_INACTIVE, ucBssIndex);

		if (bss == NULL)
			return FALSE;

		/* 2.4 -> 5 */
#if (CFG_SUPPORT_WIFI_6G == 1)
		if ((bss->eBand == BAND_5G || bss->eBand == BAND_6G)
#else
		if (bss->eBand == BAND_5G
#endif
			&& target->eBand == BAND_2G4) {
			if (rssi > RSSI_BAD_NEED_ROAM_24G_TO_5G_6G)
				return FALSE;
			if (bss->ucRCPI >= RCPI_THRESHOLD_ROAM_TO_5G_6G ||
			bss->ucRCPI - target->ucRCPI > RCPI_DIFF_DRIVER_ROAM) {
				log_dbg(SCN, INFO,
					"Driver trigger roaming to 5G band.\n");
				return TRUE;
			}
		} else {
			if (rssi > RSSI_BAD_NEED_ROAM)
				return FALSE;
			if (bss->ucRCPI - target->ucRCPI >
				RCPI_DIFF_DRIVER_ROAM) {
				log_dbg(SCN, INFO,
				"Driver trigger roaming for other cases.\n");
				return TRUE;
			}
		}
	}
#endif

	return FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
* \brief    This function is to decide if we can roam out by this beacon time
*
* \param[in] prAdapter  Pointer of ADAPTER_T
*
* \return true	if we can roam out
*         false	others
*/
/*----------------------------------------------------------------------------*/
uint8_t scanBeaconTimeoutFilterPolicyForAis(struct ADAPTER *prAdapter,
	uint8_t ucBssIndex)
{
	int8_t rssi;

	rssi = prAdapter->rLinkQuality.rLq[ucBssIndex].cRssi;
	if (roamingFsmInDecision(prAdapter, ucBssIndex) &&
	    rssi > GOOD_RSSI_FOR_HT_VHT - 5) {
		struct BSS_DESC *target;
		struct BSS_DESC *bss;

		/* Good rssi but beacon timeout happened => PER */
		target = aisGetTargetBssDesc(prAdapter, ucBssIndex);
		bss = scanSearchBssDescByScoreForAis(prAdapter,
			ROAMING_REASON_TX_ERR, ucBssIndex);
		if (bss && UNEQUAL_MAC_ADDR(bss->aucBSSID, target->aucBSSID)) {
			log_dbg(SCN, INFO, "Better AP for beacon timeout");
			return TRUE;
		}
	}

	return FALSE;
}
