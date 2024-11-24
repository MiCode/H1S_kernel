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
/*
 * Id: //Department/DaVinci/BRANCHES/MT6620_WIFI_DRIVER_V2_3/mgmt/scan_fsm.c#2
 */

/*! \file   "scan_fsm.c"
 *    \brief  This file defines the state transition function for SCAN FSM.
 *
 *    The SCAN FSM is part of SCAN MODULE and responsible for performing basic
 *    SCAN behavior as metioned in IEEE 802.11 2007 11.1.3.1 & 11.1.3.2.
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
static uint8_t *apucDebugScanState[SCAN_STATE_NUM] = {
	(uint8_t *) DISP_STRING("IDLE"),
	(uint8_t *) DISP_STRING("SCANNING"),
};

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */
#if (CFG_SUPPORT_WIFI_6G == 1)
#define SCN_GET_EBAND_BY_CH_NUM(_ucChNum) \
	((_ucChNum <= HW_CHNL_NUM_MAX_2G4) ? BAND_2G4 : \
	(_ucChNum > HW_CHNL_NUM_MAX_5G) ? BAND_6G : \
	BAND_5G)
#else
#define SCN_GET_EBAND_BY_CH_NUM(_ucChNum) \
	((_ucChNum <= HW_CHNL_NUM_MAX_2G4) ? BAND_2G4 :	BAND_5G)
#endif

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */
/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnFsmSteps(IN struct ADAPTER *prAdapter,
	IN enum ENUM_SCAN_STATE eNextState)
{
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	struct MSG_HDR *prMsgHdr;

	u_int8_t fgIsTransition = (u_int8_t) FALSE;

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	do {
		/* Coverity */
		if (prScanInfo->eCurrentState >= 0 && eNextState >= 0) {
			log_dbg(SCN, STATE, "[SCAN]TRANSITION: [%s] -> [%s]\n",
				apucDebugScanState[prScanInfo->eCurrentState],
				apucDebugScanState[eNextState]);
		}
		/* NOTE(Kevin): This is the only place to change the
		 * eCurrentState(except initial)
		 */
		prScanInfo->eCurrentState = eNextState;

		fgIsTransition = (u_int8_t) FALSE;

		switch (prScanInfo->eCurrentState) {
		case SCAN_STATE_IDLE:
			prScanParam->fg6gOobRnrParseEn = FALSE;
			/* check for pending scanning requests */
			if (!LINK_IS_EMPTY(&(prScanInfo->rPendingMsgList))) {
				/* load next message from pending list as
				 * scan parameters
				 */
				LINK_REMOVE_HEAD(&(prScanInfo->rPendingMsgList),
					prMsgHdr, struct MSG_HDR *);

#define __MSG_ID__ prMsgHdr->eMsgId
				if (__MSG_ID__ == MID_AIS_SCN_SCAN_REQ
					|| __MSG_ID__ == MID_BOW_SCN_SCAN_REQ
					|| __MSG_ID__ == MID_P2P_SCN_SCAN_REQ
					|| __MSG_ID__ == MID_RLM_SCN_SCAN_REQ) {
					scnFsmHandleScanMsg(prAdapter,
						(struct MSG_SCN_SCAN_REQ *)
						 prMsgHdr);

					eNextState = SCAN_STATE_SCANNING;
					fgIsTransition = TRUE;
				} else if (__MSG_ID__ == MID_AIS_SCN_SCAN_REQ_V2
					|| __MSG_ID__ == MID_BOW_SCN_SCAN_REQ_V2
					|| __MSG_ID__ == MID_P2P_SCN_SCAN_REQ_V2
					|| __MSG_ID__ == MID_RLM_SCN_SCAN_REQ_V2
					) {
					scnFsmHandleScanMsgV2(prAdapter,
						(struct MSG_SCN_SCAN_REQ_V2 *)
						 prMsgHdr);

					eNextState = SCAN_STATE_SCANNING;
					fgIsTransition = TRUE;
				} else {
					/* should not happen */
					ASSERT(0);
				}
#undef __MSG_ID__

				/* switch to next state */
				cnmMemFree(prAdapter, prMsgHdr);
			}
			break;

		case SCAN_STATE_SCANNING:
			/* Support AP Selection */
			prScanInfo->u4ScanUpdateIdx++;
			if (prScanParam->fgIsScanV2 == FALSE)
				scnSendScanReq(prAdapter);
			else
				scnSendScanReqV2(prAdapter);
			break;

		default:
			ASSERT(0);
			break;

		}
	} while (fgIsTransition);
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief        Generate CMD_ID_SCAN_REQ command
 *
 * Because CMD_ID_SCAN_REQ is deprecated,
 * wrap this command to CMD_ID_SCAN_REQ_V2
 *
 * \param[in] prAdapter   adapter
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnSendScanReq(IN struct ADAPTER *prAdapter)
{
	log_dbg(SCN, WARN,
		"CMD_ID_SCAN_REQ is deprecated, use CMD_ID_SCAN_REQ_V2\n");
	scnSendScanReqV2(prAdapter);
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief        Generate CMD_ID_SCAN_REQ_V2 command
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnSendScanReqV2(IN struct ADAPTER *prAdapter)
{
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	/* CMD_SCAN_REQ_V2 rCmdScanReq; */
	struct CMD_SCAN_REQ_V2 *prCmdScanReq;
	uint32_t i, j;
	uint8_t ucChannel;
	uint8_t ucBand;
	struct CHANNEL_INFO *prCnlInfo;

	ASSERT(prAdapter);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &(prScanInfo->rScanParam);

	prCmdScanReq = kalMemAlloc(
		sizeof(struct CMD_SCAN_REQ_V2), VIR_MEM_TYPE);
	if (!prCmdScanReq) {
		log_dbg(SCN, ERROR, "alloc CmdScanReq V2 fail\n");
		return;
	}
	/* send command packet for scan */
	kalMemZero(prCmdScanReq, sizeof(struct CMD_SCAN_REQ_V2));
	/* Modify channelList number from 32 to 54 */
	if (prScanParam->ucScnFuncMask & ENUM_SCN_USE_PADDING_AS_BSSID) {
		kalMemCopy(prCmdScanReq->aucExtBSSID,
			&prScanParam->aucBSSID[0][0],
			CFG_SCAN_OOB_MAX_NUM * MAC_ADDR_LEN);
	} else {
		COPY_MAC_ADDR(prCmdScanReq->aucBSSID,
		&prScanParam->aucBSSID[0][0]);
	}
	if (!EQUAL_MAC_ADDR(prCmdScanReq->aucBSSID, "\xff\xff\xff\xff\xff\xff"))
		DBGLOG(SCN, INFO, "Include BSSID "MACSTR" in probe request\n",
			MAC2STR(prCmdScanReq->aucBSSID));

	prCmdScanReq->ucSeqNum = prScanParam->ucSeqNum;
	prCmdScanReq->ucBssIndex = prScanParam->ucBssIndex;
	prCmdScanReq->ucScanType = (uint8_t) prScanParam->eScanType;
	prCmdScanReq->ucSSIDType = prScanParam->ucSSIDType;
	prCmdScanReq->auVersion[0] = 1;
	prCmdScanReq->ucScnFuncMask |= prScanParam->ucScnFuncMask;
	/* for 6G OOB scan */
	kalMemCopy(prCmdScanReq->ucBssidMatchCh, prScanParam->ucBssidMatchCh,
			CFG_SCAN_OOB_MAX_NUM);
	kalMemCopy(prCmdScanReq->ucBssidMatchSsidInd,
		prScanParam->ucBssidMatchSsidInd, CFG_SCAN_OOB_MAX_NUM);

	if (kalIsValidMacAddr(prScanParam->aucRandomMac)) {
		prCmdScanReq->ucScnFuncMask |= (ENUM_SCN_RANDOM_MAC_EN |
						ENUM_SCN_RANDOM_SN_EN);
		kalMemCopy(prCmdScanReq->aucRandomMac,
			prScanParam->aucRandomMac, MAC_ADDR_LEN);
	}
	if (prAdapter->rWifiVar.eDbdcMode == ENUM_DBDC_MODE_DISABLED
#if (CFG_SUPPORT_POWER_THROTTLING == 1 && CFG_SUPPORT_CNM_POWER_CTRL == 1)
		|| prAdapter->fgPowerForceOneNss
#endif
		)
		prCmdScanReq->ucScnFuncMask |= ENUM_SCN_DBDC_SCAN_DIS;

	/* Set SSID to scan request */
	if (prScanParam->ucSSIDNum <= SCAN_CMD_SSID_NUM) {
		prCmdScanReq->ucSSIDNum = prScanParam->ucSSIDNum;
		prCmdScanReq->ucSSIDExtNum = 0;
	} else if (prScanParam->ucSSIDNum <= CFG_SCAN_SSID_MAX_NUM) {
		prCmdScanReq->ucSSIDNum = SCAN_CMD_SSID_NUM;
		prCmdScanReq->ucSSIDExtNum = prScanParam->ucSSIDNum
			- SCAN_CMD_SSID_NUM;
	} else {
		log_dbg(SCN, WARN, "Too many SSID %u\n",
					prScanParam->ucSSIDNum);
		prCmdScanReq->ucSSIDNum = SCAN_CMD_SSID_NUM;
		prCmdScanReq->ucSSIDExtNum = SCAN_CMD_EXT_SSID_NUM;
	}

	for (i = 0; i < prCmdScanReq->ucSSIDNum; i++) {
		COPY_SSID(prCmdScanReq->arSSID[i].aucSsid,
			prCmdScanReq->arSSID[i].u4SsidLen,
			prScanParam->aucSpecifiedSSID[i],
			prScanParam->ucSpecifiedSSIDLen[i]);
		log_dbg(SCN, TRACE,
			"Ssid=%s, SsidLen=%d\n",
			HIDE(prCmdScanReq->arSSID[i].aucSsid),
			prCmdScanReq->arSSID[i].u4SsidLen);
	}
	for (i = 0; i < prCmdScanReq->ucSSIDExtNum; i++) {
		COPY_SSID(prCmdScanReq->arSSIDExtend[i].aucSsid,
			prCmdScanReq->arSSIDExtend[i].u4SsidLen,
			prScanParam->aucSpecifiedSSID
			[prCmdScanReq->ucSSIDNum+i],
			prScanParam->ucSpecifiedSSIDLen
			[prCmdScanReq->ucSSIDNum+i]);
		log_dbg(SCN, TRACE,
			"Ssid=%s, SsidLen=%d\n",
			prCmdScanReq->arSSIDExtend[i].aucSsid,
			prCmdScanReq->arSSIDExtend[i].u4SsidLen);
	}

	prCmdScanReq->u2ProbeDelayTime
		= (uint8_t) prScanParam->u2ProbeDelayTime;
	prCmdScanReq->ucChannelType
		= (uint8_t) prScanParam->eScanChannel;

	/* Set channel info to scan request */
	if (prScanParam->eScanChannel == SCAN_CHANNEL_SPECIFIED) {
		for (i = 0, j = 0; i < prScanParam->ucChannelListNum; i++) {
			ucBand = (uint8_t) prScanParam->arChnlInfoList[i].eBand;
			ucChannel = prScanParam->arChnlInfoList[i].ucChannelNum;
			/* remove DFS channel */
			if (prAdapter->rWifiVar.rScanInfo.fgSkipDFS &&
				rlmDomainIsDfsChnls(prAdapter, ucChannel))
				continue;

			if (j < SCAN_CMD_CHNL_NUM) {
				prCnlInfo = &(prCmdScanReq->arChannelList[j]);
				prCnlInfo->ucBand = ucBand;
				prCnlInfo->ucChannelNum = ucChannel;
				prCmdScanReq->ucChannelListNum = ++j;
			} else if (j < MAXIMUM_OPERATION_CHANNEL_LIST) {
				prCnlInfo = &(prCmdScanReq->
				   arChannelListExtend[j - SCAN_CMD_CHNL_NUM]);
				prCnlInfo->ucBand = ucBand;
				prCnlInfo->ucChannelNum = ucChannel;
				prCmdScanReq->ucChannelListExtNum =
					(++j - SCAN_CMD_CHNL_NUM);
			} else {
				log_dbg(SCN, WARN, "Too many Channel %u\n",
					prScanParam->ucChannelListNum);
				prCmdScanReq->ucChannelListNum = 0;
				prCmdScanReq->ucChannelListExtNum = 0;
				prCmdScanReq->ucChannelType = SCAN_CHANNEL_FULL;
				break;
			}
		}
	}

	prCmdScanReq->u2ChannelDwellTime = prScanParam->u2ChannelDwellTime;
	prCmdScanReq->u2ChannelMinDwellTime =
		prScanParam->u2ChannelMinDwellTime;
	prCmdScanReq->u2TimeoutValue = prScanParam->u2TimeoutValue;

	/* If ProbeDelayTime bigger than MinDwellTime,
	 * reset ProbeDelayTime to 0
	 */
	if (prCmdScanReq->u2ProbeDelayTime >
	    prCmdScanReq->u2ChannelMinDwellTime)
		prCmdScanReq->u2ProbeDelayTime = 0;

	/* OCE certification handling */
	if (prAdapter->rWifiVar.u4SwTestMode == ENUM_SW_TEST_MODE_SIGMA_OCE) {
		scanHandleOceIE(prScanParam, prCmdScanReq);
		prCmdScanReq->ucScnFuncMask |= ENUM_SCN_OCE_SCAN_EN;
	}

	/* enable split scan when (not in roam) && (WFD || 1s TRX pkt > 30) */
	if (scnEnableSplitScan(prAdapter, prScanParam->ucBssIndex)) {
		prCmdScanReq->ucScnFuncMask |= ENUM_SCN_SPLIT_SCAN_EN;
		/* if WFD enable, not do dbdc scan and reduce dwell time to
		 * enhance latency
		 */
		if (wlanWfdEnabled(prAdapter)) {
			prCmdScanReq->ucScnFuncMask |= ENUM_SCN_DBDC_SCAN_DIS;
			prCmdScanReq->u2ChannelDwellTime =
				SCAN_CHANNEL_DWELL_TIME_MIN_MSEC;
			prCmdScanReq->u2ChannelMinDwellTime =
				SCAN_CHANNEL_DWELL_TIME_MIN_MSEC;
		}
	}

	if (prScanParam->u2IELen <= MAX_IE_LENGTH)
		prCmdScanReq->u2IELen = prScanParam->u2IELen;
	else
		prCmdScanReq->u2IELen = MAX_IE_LENGTH;

	if (prScanParam->u2IELen)
		kalMemCopy(prCmdScanReq->aucIE, prScanParam->aucIE,
			sizeof(uint8_t) * prCmdScanReq->u2IELen);

	log_dbg(SCN, TRACE, "ScanReqV2: ScanType=%d,BSS=%u,SSIDType=%d,Num=%u,Ext=%u,ChannelType=%d,Num=%d,Ext=%u,Seq=%u,Ver=%u,Dw=%u,Min=%u,Func=0x%X,Mac="
		MACSTR ",BSSID:"MACSTR"\n",
		prCmdScanReq->ucScanType,
		prCmdScanReq->ucBssIndex,
		prCmdScanReq->ucSSIDType,
		prCmdScanReq->ucSSIDNum,
		prCmdScanReq->ucSSIDExtNum,
		prCmdScanReq->ucChannelType,
		prCmdScanReq->ucChannelListNum,
		prCmdScanReq->ucChannelListExtNum,
		prCmdScanReq->ucSeqNum, prCmdScanReq->auVersion[0],
		prCmdScanReq->u2ChannelDwellTime,
		prCmdScanReq->u2ChannelMinDwellTime,
		prCmdScanReq->ucScnFuncMask,
		MAC2STR(prCmdScanReq->aucRandomMac),
		MAC2STR(prCmdScanReq->aucBSSID));

	scanLogCacheFlushAll(prAdapter, &(prScanInfo->rScanLogCache),
		LOG_SCAN_REQ_D2F);
	scanReqLog(prCmdScanReq);

	wlanSendSetQueryCmd(prAdapter,
		CMD_ID_SCAN_REQ_V2,
		TRUE,
		FALSE,
		FALSE,
		NULL,
		NULL,
		sizeof(struct CMD_SCAN_REQ_V2),
		(uint8_t *)prCmdScanReq, NULL, 0);
	log_dbg(SCN, TRACE, "Send %zu bytes\n", sizeof(struct CMD_SCAN_REQ_V2));

	GET_CURRENT_SYSTIME(&prScanInfo->rLastScanStartTime);

	kalMemFree(prCmdScanReq, VIR_MEM_TYPE, sizeof(struct CMD_SCAN_REQ_V2));

}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnFsmMsgStart(IN struct ADAPTER *prAdapter, IN struct MSG_HDR *prMsgHdr)
{
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;

	ASSERT(prMsgHdr);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	if (prScanInfo->eCurrentState == SCAN_STATE_IDLE) {
		if (prMsgHdr->eMsgId == MID_AIS_SCN_SCAN_REQ
			|| prMsgHdr->eMsgId == MID_BOW_SCN_SCAN_REQ
			|| prMsgHdr->eMsgId == MID_P2P_SCN_SCAN_REQ
			|| prMsgHdr->eMsgId == MID_RLM_SCN_SCAN_REQ) {
			scnFsmHandleScanMsg(prAdapter,
				(struct MSG_SCN_SCAN_REQ *) prMsgHdr);
		} else if (prMsgHdr->eMsgId == MID_AIS_SCN_SCAN_REQ_V2
			|| prMsgHdr->eMsgId == MID_BOW_SCN_SCAN_REQ_V2
			|| prMsgHdr->eMsgId == MID_P2P_SCN_SCAN_REQ_V2
			|| prMsgHdr->eMsgId == MID_RLM_SCN_SCAN_REQ_V2) {
			scnFsmHandleScanMsgV2(prAdapter,
				(struct MSG_SCN_SCAN_REQ_V2 *) prMsgHdr);
		} else {
			/* should not deliver to this function */
			ASSERT(0);
		}

		cnmMemFree(prAdapter, prMsgHdr);
		scnFsmSteps(prAdapter, SCAN_STATE_SCANNING);
	} else {
		LINK_INSERT_TAIL(&prScanInfo->rPendingMsgList,
			&prMsgHdr->rLinkEntry);
	}
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnFsmMsgAbort(IN struct ADAPTER *prAdapter, IN struct MSG_HDR *prMsgHdr)
{
	struct MSG_SCN_SCAN_CANCEL *prScanCancel;
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	struct CMD_SCAN_CANCEL rCmdScanCancel;

	ASSERT(prMsgHdr);

	prScanCancel = (struct MSG_SCN_SCAN_CANCEL *) prMsgHdr;
	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	if (prScanInfo->eCurrentState != SCAN_STATE_IDLE) {
		if (prScanCancel->ucSeqNum == prScanParam->ucSeqNum &&
			prScanCancel->ucBssIndex == prScanParam->ucBssIndex) {
			enum ENUM_SCAN_STATUS eStatus = SCAN_STATUS_DONE;

			/* send cancel message to firmware domain */
			rCmdScanCancel.ucSeqNum = prScanParam->ucSeqNum;
			rCmdScanCancel.ucIsExtChannel
				= (uint8_t) prScanCancel->fgIsChannelExt;

			scanlog_dbg(LOG_SCAN_ABORT_REQ_D2F, INFO, "Scan Abort#%u to Q: isExtCh=%u",
				rCmdScanCancel.ucSeqNum,
				rCmdScanCancel.ucIsExtChannel);

			wlanSendSetQueryCmd(prAdapter,
				CMD_ID_SCAN_CANCEL,
				TRUE,
				FALSE,
				FALSE,
				NULL,
				NULL,
				sizeof(struct CMD_SCAN_CANCEL),
				(uint8_t *) &rCmdScanCancel,
				NULL,
				0);

			/* Full2Partial: ignore this statistics */
			if (prScanInfo->fgIsScanForFull2Partial) {
				prScanInfo->fgIsScanForFull2Partial = FALSE;
				prScanInfo->u4LastFullScanTime = 0;
				log_dbg(SCN, INFO,
					"Full2Partial: scan canceled(%u)\n",
					prScanParam->ucSeqNum);
			}

			/* generate scan-done event for caller */
			if (prScanCancel->fgIsOidRequest)
				eStatus = SCAN_STATUS_CANCELLED;
			else
				eStatus = SCAN_STATUS_DONE;
			scnFsmGenerateScanDoneMsg(prAdapter,
				prScanParam->eMsgId,
				prScanParam->ucSeqNum,
				prScanParam->ucBssIndex,
				eStatus);

			/* switch to next pending scan */
			scnFsmSteps(prAdapter, SCAN_STATE_IDLE);
		} else {
			scnFsmRemovePendingMsg(prAdapter,
				prScanCancel->ucSeqNum,
				prScanCancel->ucBssIndex);
		}
	}

	cnmMemFree(prAdapter, prMsgHdr);
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief            Scan Message Parsing (Legacy)
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnFsmHandleScanMsg(IN struct ADAPTER *prAdapter,
	IN struct MSG_SCN_SCAN_REQ *prScanReqMsg)
{
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	uint32_t i;

	ASSERT(prAdapter);
	ASSERT(prScanReqMsg);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	kalMemZero(prScanParam, sizeof(*prScanParam));
	prScanParam->eScanType = prScanReqMsg->eScanType;
	prScanParam->ucBssIndex = prScanReqMsg->ucBssIndex;
	prScanParam->ucSSIDType = prScanReqMsg->ucSSIDType;
	if (prScanParam->ucSSIDType
		& (SCAN_REQ_SSID_SPECIFIED | SCAN_REQ_SSID_P2P_WILDCARD)) {
		prScanParam->ucSSIDNum = 1;

		COPY_SSID(prScanParam->aucSpecifiedSSID[0],
			prScanParam->ucSpecifiedSSIDLen[0],
			prScanReqMsg->aucSSID, prScanReqMsg->ucSSIDLength);

		/* reset SSID length to zero for rest array entries */
		for (i = 1; i < SCN_SSID_MAX_NUM; i++)
			prScanParam->ucSpecifiedSSIDLen[i] = 0;
	} else {
		prScanParam->ucSSIDNum = 0;

		for (i = 0; i < SCN_SSID_MAX_NUM; i++)
			prScanParam->ucSpecifiedSSIDLen[i] = 0;
	}

	prScanParam->u2ProbeDelayTime = 0;
	prScanParam->eScanChannel = prScanReqMsg->eScanChannel;
	if (prScanParam->eScanChannel == SCAN_CHANNEL_SPECIFIED) {
		if (prScanReqMsg->ucChannelListNum
			<= MAXIMUM_OPERATION_CHANNEL_LIST) {
			prScanParam->ucChannelListNum
				= prScanReqMsg->ucChannelListNum;
		} else {
			prScanParam->ucChannelListNum
				= MAXIMUM_OPERATION_CHANNEL_LIST;
		}

		kalMemCopy(prScanParam->arChnlInfoList,
			prScanReqMsg->arChnlInfoList,
			sizeof(struct RF_CHANNEL_INFO)
				* prScanParam->ucChannelListNum);
	}

	if (prScanReqMsg->u2IELen <= MAX_IE_LENGTH)
		prScanParam->u2IELen = prScanReqMsg->u2IELen;
	else
		prScanParam->u2IELen = MAX_IE_LENGTH;

	if (prScanParam->u2IELen) {
		kalMemCopy(prScanParam->aucIE,
			prScanReqMsg->aucIE, prScanParam->u2IELen);
	}

	prScanParam->u2ChannelDwellTime = prScanReqMsg->u2ChannelDwellTime;
	prScanParam->u2TimeoutValue = prScanReqMsg->u2TimeoutValue;
	prScanParam->ucSeqNum = prScanReqMsg->ucSeqNum;
	prScanParam->eMsgId = prScanReqMsg->rMsgHdr.eMsgId;
	prScanParam->fgIsScanV2 = FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief            Scan Message Parsing - V2 with multiple SSID support
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnFsmHandleScanMsgV2(IN struct ADAPTER *prAdapter,
	IN struct MSG_SCN_SCAN_REQ_V2 *prScanReqMsg)
{
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	uint32_t i;

	ASSERT(prAdapter);
	ASSERT(prScanReqMsg);
	ASSERT(prScanReqMsg->ucSSIDNum <= SCN_SSID_MAX_NUM);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	kalMemZero(prScanParam, sizeof(*prScanParam));
	prScanParam->eScanType = prScanReqMsg->eScanType;
	prScanParam->ucBssIndex = prScanReqMsg->ucBssIndex;
	prScanParam->ucSSIDType = prScanReqMsg->ucSSIDType;
	prScanParam->ucSSIDNum = prScanReqMsg->ucSSIDNum;
	prScanParam->ucScnFuncMask |= prScanReqMsg->ucScnFuncMask;

	kalMemCopy(prScanParam->aucRandomMac, prScanReqMsg->aucRandomMac,
		MAC_ADDR_LEN);
	/* for 6G OOB scan */
	kalMemCopy(prScanParam->ucBssidMatchCh, prScanReqMsg->ucBssidMatchCh,
			CFG_SCAN_OOB_MAX_NUM);
	kalMemCopy(prScanParam->ucBssidMatchSsidInd,
		prScanReqMsg->ucBssidMatchSsidInd, CFG_SCAN_OOB_MAX_NUM);
	prScanParam->fg6gOobRnrParseEn = prScanReqMsg->fg6gOobRnrParseEn;

	if ((prScanParam->ucSSIDType & SCAN_REQ_SSID_SPECIFIED_ONLY) &&
		((prScanReqMsg->ucScnFuncMask &
		ENUM_SCN_USE_PADDING_AS_BSSID) == 0)) {
		prScanParam->ucSSIDNum = 1;
		kalMemZero(prScanParam->ucSpecifiedSSIDLen,
			   sizeof(prScanParam->ucSpecifiedSSIDLen));
		COPY_SSID(prScanParam->aucSpecifiedSSID[0],
			  prScanParam->ucSpecifiedSSIDLen[0],
			  &prScanReqMsg->prSsid[0].aucSsid[0],
			  prScanReqMsg->prSsid[0].u4SsidLen);
	} else {
		for (i = 0; i < prScanReqMsg->ucSSIDNum; i++) {
			COPY_SSID(prScanParam->aucSpecifiedSSID[i],
				  prScanParam->ucSpecifiedSSIDLen[i],
				  prScanReqMsg->prSsid[i].aucSsid,
				  (uint8_t) prScanReqMsg->prSsid[i].u4SsidLen);
		}
	}

	prScanParam->u2ProbeDelayTime = prScanReqMsg->u2ProbeDelay;
	prScanParam->eScanChannel = prScanReqMsg->eScanChannel;
	if (prScanParam->eScanChannel == SCAN_CHANNEL_SPECIFIED) {
		if (prScanReqMsg->ucChannelListNum
			<= MAXIMUM_OPERATION_CHANNEL_LIST) {
			prScanParam->ucChannelListNum
				= prScanReqMsg->ucChannelListNum;
		} else {
			prScanParam->ucChannelListNum
				= MAXIMUM_OPERATION_CHANNEL_LIST;
		}

		kalMemCopy(prScanParam->arChnlInfoList,
			prScanReqMsg->arChnlInfoList,
			sizeof(struct RF_CHANNEL_INFO)
				* prScanParam->ucChannelListNum);
	}

	if (prScanReqMsg->u2IELen <= MAX_IE_LENGTH)
		prScanParam->u2IELen = prScanReqMsg->u2IELen;
	else
		prScanParam->u2IELen = MAX_IE_LENGTH;

	if (prScanParam->u2IELen) {
		kalMemCopy(prScanParam->aucIE,
			prScanReqMsg->aucIE, prScanParam->u2IELen);
	}

	prScanParam->u2ChannelDwellTime = prScanReqMsg->u2ChannelDwellTime;
	prScanParam->u2ChannelMinDwellTime =
		prScanReqMsg->u2ChannelMinDwellTime;
	prScanParam->u2TimeoutValue = prScanReqMsg->u2TimeoutValue;
	prScanParam->ucSeqNum = prScanReqMsg->ucSeqNum;
	prScanParam->eMsgId = prScanReqMsg->rMsgHdr.eMsgId;
	prScanParam->fgIsScanV2 = TRUE;

	kalMemCopy(&prScanParam->aucBSSID[0][0],
		&prScanReqMsg->aucExtBssid[0][0],
		CFG_SCAN_OOB_MAX_NUM * MAC_ADDR_LEN);
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief            Remove pending scan request
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnFsmRemovePendingMsg(IN struct ADAPTER *prAdapter, IN uint8_t ucSeqNum,
	IN uint8_t ucBssIndex)
{
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	struct MSG_HDR *prPendingMsgHdr = NULL;
	struct MSG_HDR *prPendingMsgHdrNext = NULL;
	struct MSG_HDR *prRemoveMsgHdr = NULL;
	struct LINK_ENTRY *prRemoveLinkEntry = NULL;
	u_int8_t fgIsRemovingScan = FALSE;

	ASSERT(prAdapter);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	/* traverse through rPendingMsgList for removal */
	LINK_FOR_EACH_ENTRY_SAFE(prPendingMsgHdr,
		prPendingMsgHdrNext, &(prScanInfo->rPendingMsgList),
		rLinkEntry, struct MSG_HDR) {

#define __MSG_ID__ prPendingMsgHdr->eMsgId
		if (__MSG_ID__ == MID_AIS_SCN_SCAN_REQ
		    || __MSG_ID__ == MID_BOW_SCN_SCAN_REQ
		    || __MSG_ID__ == MID_P2P_SCN_SCAN_REQ
		    || __MSG_ID__ == MID_RLM_SCN_SCAN_REQ) {
			struct MSG_SCN_SCAN_REQ *prScanReqMsg
				= (struct MSG_SCN_SCAN_REQ *)
					prPendingMsgHdr;

			if (ucSeqNum == prScanReqMsg->ucSeqNum
				&& ucBssIndex == prScanReqMsg->ucBssIndex) {
				prRemoveLinkEntry
					= &(prScanReqMsg->rMsgHdr.rLinkEntry);
				prRemoveMsgHdr = prPendingMsgHdr;
				fgIsRemovingScan = TRUE;
			}
		} else if (__MSG_ID__ == MID_AIS_SCN_SCAN_REQ_V2
			   || __MSG_ID__ == MID_BOW_SCN_SCAN_REQ_V2
			   || __MSG_ID__ == MID_P2P_SCN_SCAN_REQ_V2
			   || __MSG_ID__ == MID_RLM_SCN_SCAN_REQ_V2) {
			struct MSG_SCN_SCAN_REQ_V2 *prScanReqMsgV2
				= (struct MSG_SCN_SCAN_REQ_V2 *)
					prPendingMsgHdr;

			if (ucSeqNum == prScanReqMsgV2->ucSeqNum
				&& ucBssIndex == prScanReqMsgV2->ucBssIndex) {
				prRemoveLinkEntry
					= &(prScanReqMsgV2->rMsgHdr.rLinkEntry);
				prRemoveMsgHdr = prPendingMsgHdr;
				fgIsRemovingScan = TRUE;
			}
		}
#undef __MSG_ID__

		if (prRemoveLinkEntry) {
			if (fgIsRemovingScan == TRUE) {
				/* generate scan-done event for caller */
				scnFsmGenerateScanDoneMsg(prAdapter,
					prPendingMsgHdr->eMsgId, ucSeqNum,
					ucBssIndex, SCAN_STATUS_CANCELLED);
			}

			/* remove from pending list */
			LINK_REMOVE_KNOWN_ENTRY(&(prScanInfo->rPendingMsgList),
				prRemoveLinkEntry);
			cnmMemFree(prAdapter, prRemoveMsgHdr);

			break;
		}
	}
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnEventScanDone(IN struct ADAPTER *prAdapter,
	IN struct EVENT_SCAN_DONE *prScanDone, u_int8_t fgIsNewVersion)
{
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	KAL_SPIN_LOCK_DECLARATION();

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	if (fgIsNewVersion) {
		scanlog_dbg(LOG_SCAN_DONE_F2D, INFO, "scnEventScanDone Version%u!size of ScanDone%zu,ucCompleteChanCount[%u],ucCurrentState%u, u4ScanDurBcnCnt[%u],Seq[%u]\n",
			prScanDone->ucScanDoneVersion,
			sizeof(struct EVENT_SCAN_DONE),
			prScanDone->ucCompleteChanCount,
			prScanDone->ucCurrentState,
			prScanDone->u4ScanDurBcnCnt,
			prScanDone->ucSeqNum);

		KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_BSSLIST_FW);
		scanLogCacheFlushBSS(&(prScanInfo->rScanLogCache.rBSSListFW),
			LOG_SCAN_DONE_F2D);
		KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_BSSLIST_FW);

		if (prScanDone->ucCurrentState != FW_SCAN_STATE_SCAN_DONE) {
			log_dbg(SCN, INFO, "FW Scan timeout!generate ScanDone event at State%d complete chan count%d ucChannelListNum%d\n",
				prScanDone->ucCurrentState,
				prScanDone->ucCompleteChanCount,
				prScanParam->ucChannelListNum);

		} else {
			log_dbg(SCN, TRACE, " scnEventScanDone at FW_SCAN_STATE_SCAN_DONE state\n");
		}
	} else {
		scanlog_dbg(LOG_SCAN_DONE_F2D, INFO, "Old scnEventScanDone Version\n");
	}

	/* buffer empty channel information */
	if (prScanDone->ucSparseChannelValid) {
		scnFsmDumpScanDoneInfo(prAdapter, prScanDone);
	} else {
		prScanInfo->fgIsSparseChannelValid = FALSE;
	}

	/* Full2Partial */
	if (prScanInfo->fgIsScanForFull2Partial &&
		prScanInfo->ucFull2PartialSeq == prScanDone->ucSeqNum) {
		uint32_t *pu4BitMap = &(prScanInfo->au4ChannelBitMap[0]);
#if (CFG_SUPPORT_WIFI_6G == 1)
		log_dbg(SCN, INFO,
			"Full2Partial(%u):%08X %08X %08X %08X %08X %08X %08X %08X %08X %08X %08X %08X %08X %08X %08X %08X\n",
			scanCountBits(prScanInfo->au4ChannelBitMap,
			sizeof(prScanInfo->au4ChannelBitMap)),
			pu4BitMap[7], pu4BitMap[6], pu4BitMap[5], pu4BitMap[4],
			pu4BitMap[3], pu4BitMap[2], pu4BitMap[1], pu4BitMap[0],
			pu4BitMap[15], pu4BitMap[14], pu4BitMap[13],
			pu4BitMap[12], pu4BitMap[11], pu4BitMap[10],
			pu4BitMap[9], pu4BitMap[8]);
#else
		log_dbg(SCN, INFO,
			"Full2Partial(%u):%08X %08X %08X %08X %08X %08X %08X %08X\n",
			scanCountBits(prScanInfo->au4ChannelBitMap,
			sizeof(prScanInfo->au4ChannelBitMap)),
			pu4BitMap[7], pu4BitMap[6], pu4BitMap[5], pu4BitMap[4],
			pu4BitMap[3], pu4BitMap[2], pu4BitMap[1], pu4BitMap[0]);
#endif

		prScanInfo->fgIsScanForFull2Partial = FALSE;
	}

	if (prScanInfo->eCurrentState == SCAN_STATE_SCANNING
		&& prScanDone->ucSeqNum == prScanParam->ucSeqNum) {
#if (CFG_SUPPORT_WIFI_RNR == 1)
		struct NEIGHBOR_AP_INFO *prNeighborAPInfo;
		enum ENUM_MSG_ID msg;

		if (!LINK_IS_EMPTY(&prScanInfo->rNeighborAPInfoList)) {
			LINK_REMOVE_HEAD(&prScanInfo->rNeighborAPInfoList,
				prNeighborAPInfo, struct NEIGHBOR_AP_INFO *);

			/* copy before write */
			msg = prScanParam->eMsgId;

			kalMemCopy(prScanParam,	&prNeighborAPInfo->rScanParam,
				sizeof(prScanInfo->rScanParam));

			/* restore for later scan done event */
			prScanParam->ucSeqNum = prScanDone->ucSeqNum;
			prScanParam->eMsgId = msg;

			cnmMemFree(prAdapter, prNeighborAPInfo);

			/* go for next scan */
			scnFsmSteps(prAdapter, SCAN_STATE_SCANNING);
			return;
		}
#endif

		scanRemoveBssDescsByPolicy(prAdapter,
		       SCN_RM_POLICY_EXCLUDE_CONNECTED | SCN_RM_POLICY_TIMEOUT);

		/* generate scan-done event for caller */
		scnFsmGenerateScanDoneMsg(prAdapter,
			prScanParam->eMsgId, prScanParam->ucSeqNum,
			prScanParam->ucBssIndex, SCAN_STATUS_DONE);

		/* switch to next pending scan */
		scnFsmSteps(prAdapter, SCAN_STATE_IDLE);
	} else {
		log_dbg(SCN, INFO, "Unexpected SCAN-DONE event: SeqNum = %d, Current State = %d\n",
			prScanDone->ucSeqNum,
			prScanInfo->eCurrentState);
	}
#if CFG_SUPPORT_SCAN_NO_AP_RECOVERY
	/* SCAN NO AP RECOVERY is only for AIS and not OOB scan,
	 * FW report scan done, reset ScnTimeoutTimes and reset count to 0
	 */
	prScanInfo->ucScnTimeoutTimes = 0;
	prScanInfo->ucScnTimeoutSubsysResetCnt = 0;

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucScanNoApRecover) &&
		prScanInfo->fgIsSparseChannelValid &&
		prScanDone->ucSparseChannelArrayValidNum > 3 &&
		(prScanParam->eMsgId == MID_AIS_SCN_SCAN_REQ ||
		prScanParam->eMsgId == MID_AIS_SCN_SCAN_REQ_V2) &&
		!(prScanParam->ucScnFuncMask & ENUM_SCN_USE_PADDING_AS_BSSID)) {
		scnDoZeroMdrdyRecoveryCheck(prAdapter, prScanDone,
				prScanInfo, prScanParam->ucBssIndex);
	}
#endif

}	/* end of scnEventScanDone */


/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void
scnFsmDumpScanDoneInfo(IN struct ADAPTER *prAdapter,
	IN struct EVENT_SCAN_DONE *prScanDone)
{
	uint8_t ucScanChNum = 0;
	uint8_t ucChCnt = 0;
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	char strbuf[SCN_SCAN_DONE_PRINT_BUFFER_LENGTH];

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	prScanInfo->fgIsSparseChannelValid = TRUE;
	prScanInfo->rSparseChannel.eBand
		= (enum ENUM_BAND) prScanDone->rSparseChannel.ucBand;
	prScanInfo->rSparseChannel.ucChannelNum
		= prScanDone->rSparseChannel.ucChannelNum;
	ucScanChNum = prScanInfo->ucSparseChannelArrayValidNum
		= prScanDone->ucSparseChannelArrayValidNum;

	if (prAdapter->rWifiVar.u2CountryCode) {
		log_dbg(SCN, INFO,
			"Country Code = %c%c, Detected_Channel_Num = %d\n",
			((prAdapter->rWifiVar.u2CountryCode
				& 0xff00) >> 8),
			(prAdapter->rWifiVar.u2CountryCode
				& 0x00ff), ucScanChNum);
	} else {
		log_dbg(SCN, INFO,
			"Country Code is NULL, Detected_Channel_Num = %d\n",
			ucScanChNum);
	}

#define print_info_ch(_Mod, _Clz, _Fmt, var) \
	do { \
		uint16_t u2Written = 0; \
		uint16_t u2TotalLen = SCN_SCAN_DONE_PRINT_BUFFER_LENGTH; \
		enum ENUM_BAND eBand = BAND_NULL; \
		for (ucChCnt = 0; ucChCnt < ucScanChNum; ucChCnt++) { \
			eBand = \
			SCN_GET_EBAND_BY_CH_NUM( \
			prScanDone->var[ucChCnt]); \
			prScanInfo->aeChannelBand[ucChCnt] = eBand; \
			prScanInfo->var[ucChCnt] \
				= prScanDone->var[ucChCnt]; \
			nicRxdChNumTranslate(eBand, \
			&prScanInfo->var[ucChCnt]); \
			u2Written += kalSnprintf(strbuf + u2Written, \
				u2TotalLen - u2Written, "%6d", \
				prScanInfo->var[ucChCnt]); \
		} \
		log_dbg(_Mod, _Clz, _Fmt, strbuf); \
	} while (0)

#define print_info(_Mod, _Clz, _Fmt, var) \
	do { \
		uint16_t u2Written = 0; \
		uint16_t u2TotalLen = SCN_SCAN_DONE_PRINT_BUFFER_LENGTH; \
		for (ucChCnt = 0; ucChCnt < ucScanChNum; ucChCnt++) { \
			prScanInfo->var[ucChCnt] \
				= prScanDone->var[ucChCnt]; \
			u2Written += kalSnprintf(strbuf + u2Written, \
				u2TotalLen - u2Written, "%6d", \
				prScanInfo->var[ucChCnt]); \
		} \
		log_dbg(_Mod, _Clz, _Fmt, strbuf); \
	} while (0)

	/* If FW scan channel count more than Driver request,
	*  means this scan done event might have something wrong,
	*  not to print it to avoid unexpected issue
	*/
	if ((ucScanChNum > prScanParam->ucChannelListNum) &&
		(prScanParam->ucChannelListNum != 0)) {
		log_dbg(SCN, INFO,
			"Driver request %d ch, but FW scan %d ch!\n",
			prScanParam->ucChannelListNum,
			ucScanChNum);
		return;
	}

	print_info_ch(SCN, INFO, "Channel  : %s\n", aucChannelNum);
	print_info(SCN, INFO, "IdleTime : %s\n", au2ChannelIdleTime);
	print_info(SCN, INFO, "MdrdyCnt : %s\n", aucChannelMDRDYCnt);
	print_info(SCN, INFO, "BAndPCnt : %s\n", aucChannelBAndPCnt);
	if (prScanDone->ucScanDoneVersion >= 4)
		print_info(SCN, LOUD,
			"ScanTime : %s\n", au2ChannelScanTime);
#undef	print_scan_info
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void
scnFsmGenerateScanDoneMsg(IN struct ADAPTER *prAdapter,
	IN enum ENUM_MSG_ID eMsgId, IN uint8_t ucSeqNum, IN uint8_t ucBssIndex,
	IN enum ENUM_SCAN_STATUS eScanStatus)
{
	struct SCAN_INFO *prScanInfo;
	struct SCAN_PARAM *prScanParam;
	struct MSG_SCN_SCAN_DONE *prScanDoneMsg;

	ASSERT(prAdapter);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prScanParam = &prScanInfo->rScanParam;

	prScanDoneMsg = (struct MSG_SCN_SCAN_DONE *) cnmMemAlloc(prAdapter,
		RAM_TYPE_MSG, sizeof(struct MSG_SCN_SCAN_DONE));
	if (!prScanDoneMsg) {
		ASSERT(0);	/* Can't indicate SCAN FSM Complete */
		return;
	}

	switch (eMsgId) {
	case MID_AIS_SCN_SCAN_REQ:
	case MID_AIS_SCN_SCAN_REQ_V2:
		prScanDoneMsg->rMsgHdr.eMsgId = MID_SCN_AIS_SCAN_DONE;
		break;
	case MID_P2P_SCN_SCAN_REQ:
	case MID_P2P_SCN_SCAN_REQ_V2:
		prScanDoneMsg->rMsgHdr.eMsgId = MID_SCN_P2P_SCAN_DONE;
		break;
	case MID_BOW_SCN_SCAN_REQ:
	case MID_BOW_SCN_SCAN_REQ_V2:
		prScanDoneMsg->rMsgHdr.eMsgId = MID_SCN_BOW_SCAN_DONE;
		break;
	case MID_RLM_SCN_SCAN_REQ:
	case MID_RLM_SCN_SCAN_REQ_V2:
		prScanDoneMsg->rMsgHdr.eMsgId = MID_SCN_RLM_SCAN_DONE;
		break;
	default:
		log_dbg(SCN, ERROR, "Unexpected Network Type: %d\n");
		cnmMemFree(prAdapter, prScanDoneMsg);
		return;
	}

	prScanDoneMsg->ucSeqNum = ucSeqNum;
	prScanDoneMsg->ucBssIndex = ucBssIndex;
	prScanDoneMsg->eScanStatus = eScanStatus;

	mboxSendMsg(prAdapter, MBOX_ID_0,
		(struct MSG_HDR *) prScanDoneMsg, MSG_SEND_METHOD_BUF);

}	/* end of scnFsmGenerateScanDoneMsg() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief        Query for most sparse channel
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
u_int8_t scnQuerySparseChannel(IN struct ADAPTER *prAdapter,
	enum ENUM_BAND *prSparseBand, uint8_t *pucSparseChannel)
{
	struct SCAN_INFO *prScanInfo;

	ASSERT(prAdapter);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	if (prScanInfo->fgIsSparseChannelValid == TRUE) {
		if (prSparseBand)
			*prSparseBand = prScanInfo->rSparseChannel.eBand;

		if (pucSparseChannel) {
			*pucSparseChannel
				= prScanInfo->rSparseChannel.ucChannelNum;
		}

		return TRUE;
	} else {
		return FALSE;
	}
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief        Event handler for schedule scan done event
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void scnEventSchedScanDone(IN struct ADAPTER *prAdapter,
	IN struct EVENT_SCHED_SCAN_DONE *prSchedScanDone)
{
	struct SCAN_INFO *prScanInfo;
	struct SCHED_SCAN_PARAM *prSchedScanParam;

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prSchedScanParam = &prScanInfo->rSchedScanParam;

	if (prScanInfo->fgSchedScanning == TRUE) {
		scanlog_dbg(LOG_SCHED_SCAN_DONE_F2D, INFO, "scnEventSchedScanDone seq %u\n",
			prSchedScanDone->ucSeqNum);

		kalSchedScanResults(prAdapter->prGlueInfo);
	} else {
		scanlog_dbg(LOG_SCHED_SCAN_DONE_F2D, INFO, "Unexpected SCHEDSCANDONE event: Seq = %u, Current State = %d\n",
			prSchedScanDone->ucSeqNum, prScanInfo->eCurrentState);
	}
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief        Function to check spilit scan enable or not
 *
 * \param[in]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
bool scnEnableSplitScan(struct ADAPTER *prAdapter, uint8_t ucBssIndex)
{
	uint8_t ucWfdEn = FALSE, ucTrxPktEn = FALSE, ucRoamingEn = FALSE;
	struct PERF_MONITOR *prPerMonitor;
	struct BSS_INFO *prBssInfo = NULL;
	struct AIS_FSM_INFO *prAisFsmInfo;
	unsigned long ulTrxPacketsDiffTotal = 0;

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);
	prPerMonitor = &prAdapter->rPerMonitor;

	if (!prBssInfo)
		return FALSE;
	/* Enable condition 1: WFD case*/
	ucWfdEn = wlanWfdEnabled(prAdapter);

	/* Enable condition 2: (TX + RX) packets in last 1s > 30,
	 * exclude P2P device because prPerMonitor not include P2P device
	 */
	if (ucBssIndex < P2P_DEV_BSS_INDEX && IS_BSS_ACTIVE(prBssInfo)) {
		ulTrxPacketsDiffTotal +=
			(prPerMonitor->ulTxPacketsDiffLastSec[ucBssIndex] +
			prPerMonitor->ulRxPacketsDiffLastSec[ucBssIndex]);

		if (ulTrxPacketsDiffTotal > SCAN_SPLIT_PACKETS_THRESHOLD) {
			log_dbg(SCN, TRACE, "SplitScan: TRXPacket=%ld",
				ulTrxPacketsDiffTotal);
			ucTrxPktEn = TRUE;
		}
	}
	/* Enable Pre-condition: not in roaming, avoid roaming scan too long */
	if (ucBssIndex < KAL_AIS_NUM) {
		prAisFsmInfo = aisGetAisFsmInfo(prAdapter, ucBssIndex);
		if (prAisFsmInfo &&
			prBssInfo->eConnectionState == MEDIA_STATE_CONNECTED &&
			prAisFsmInfo->eCurrentState == AIS_STATE_LOOKING_FOR)
			ucRoamingEn = TRUE;
	}
	log_dbg(SCN, TRACE, "SplitScan: Roam(%d),WFD(%d),TRX(%d)",
				ucRoamingEn, ucWfdEn, ucTrxPktEn);
	/* Enable split scan when (not in roam) & (WFD or TRX packet > 30) */
	if ((!ucRoamingEn) && (ucWfdEn || ucTrxPktEn))
		return TRUE;
	else
		return FALSE;
}

#if CFG_SUPPORT_SCHED_SCAN
/*----------------------------------------------------------------------------*/
/*!
 * \brief        handler for starting schedule scan
 *
 * \param[in]
 *
 * \return       TRUE if send sched scan successfully. FALSE otherwise
 */
/*----------------------------------------------------------------------------*/
u_int8_t
scnFsmSchedScanRequest(IN struct ADAPTER *prAdapter,
	IN struct PARAM_SCHED_SCAN_REQUEST *prRequest)
{
	struct SCAN_INFO *prScanInfo;
	struct SCHED_SCAN_PARAM *prSchedScanParam;
	struct CMD_SCHED_SCAN_REQ *prSchedScanCmd = NULL;
	struct SSID_MATCH_SETS *prMatchSets = NULL;
	struct PARAM_SSID *prSsid = NULL;
	uint32_t i;
	uint16_t u2IeLen;
	enum ENUM_BAND ePreferedChnl = BAND_NULL;
	struct BSS_INFO *prAisBssInfo;

	ASSERT(prAdapter);
	ASSERT(prRequest);
	ASSERT(prRequest->u4SsidNum <= CFG_SCAN_HIDDEN_SSID_MAX_NUM);
	ASSERT(prRequest->u4MatchSsidNum <= CFG_SCAN_SSID_MATCH_MAX_NUM);
	log_dbg(SCN, TRACE, "scnFsmSchedScanRequest\n");

	prAisBssInfo = aisGetAisBssInfo(prAdapter,
		prRequest->ucBssIndex);
	if (prAisBssInfo == NULL) {
		log_dbg(SCN, WARN, "prAisBssInfo is NULL\n");
		return FALSE;
	}

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prSchedScanParam = &prScanInfo->rSchedScanParam;

	if (prScanInfo->fgSchedScanning) {
		log_dbg(SCN, WARN, "prScanInfo->fgSchedScanning = TRUE already scanning\n");

		return FALSE;
	}

	/* 0. allocate memory for schedule scan command */
	if (prRequest->u4IELength <= MAX_IE_LENGTH)
		u2IeLen = (uint16_t)prRequest->u4IELength;
	else
		u2IeLen = MAX_IE_LENGTH;

	prSchedScanCmd = (struct CMD_SCHED_SCAN_REQ *) cnmMemAlloc(prAdapter,
		RAM_TYPE_BUF, sizeof(struct CMD_SCHED_SCAN_REQ) + u2IeLen);
	if (!prSchedScanCmd) {
		log_dbg(SCN, ERROR, "alloc CMD_SCHED_SCAN_REQ (%zu+%u) fail\n",
			sizeof(struct CMD_SCHED_SCAN_REQ), u2IeLen);
		return FALSE;
	}
	kalMemZero(prSchedScanCmd, sizeof(struct CMD_SCHED_SCAN_REQ) + u2IeLen);
	prMatchSets = &(prSchedScanCmd->auMatchSsid[0]);
	prSsid = &(prSchedScanCmd->auSsid[0]);

	/* 1 Set Sched scan param parameters */
	prSchedScanParam->ucSeqNum++;
	prSchedScanParam->ucBssIndex = prAisBssInfo->ucBssIndex;
	prSchedScanParam->fgStopAfterIndication = FALSE;

	prSchedScanCmd->ucBssIndex = prSchedScanParam->ucBssIndex;
	if (!IS_NET_ACTIVE(prAdapter, prAisBssInfo->ucBssIndex)) {
		SET_NET_ACTIVE(prAdapter, prAisBssInfo->ucBssIndex);
		/* sync with firmware */
		nicActivateNetwork(prAdapter,
			prAisBssInfo->ucBssIndex);
	}

	/* 2.1 Prepare command. Set FW struct SSID_MATCH_SETS */
	/* ssid in ssid list will be send in probe request in advance */
	prSchedScanCmd->ucSsidNum = prRequest->u4SsidNum;
	for (i = 0; i < prSchedScanCmd->ucSsidNum; i++) {
		kalMemCopy(&(prSsid[i]), &(prRequest->arSsid[i]),
			sizeof(struct PARAM_SSID));
		log_dbg(SCN, TRACE, "ssid set(%d) %s\n", i,
			HIDE(prSsid[i].aucSsid));
	}

	prSchedScanCmd->ucMatchSsidNum = prRequest->u4MatchSsidNum;
	for (i = 0; i < prSchedScanCmd->ucMatchSsidNum; i++) {
		COPY_SSID(prMatchSets[i].aucSsid, prMatchSets[i].ucSsidLen,
			prRequest->arMatchSsid[i].aucSsid,
			prRequest->arMatchSsid[i].u4SsidLen);
		prMatchSets[i].i4RssiThresold = prRequest->ai4RssiThold[i];
		log_dbg(SCN, TRACE, "Match set(%d) %s, rssi>%d\n",
				i, prMatchSets[i].aucSsid,
				prMatchSets[i].i4RssiThresold);
	}

	/* 2.2 Prepare command. Set channel */

	ePreferedChnl
		= prAdapter->aePreferBand[NETWORK_TYPE_AIS];
	if (ePreferedChnl == BAND_2G4) {
		prSchedScanCmd->ucChannelType =
			SCAN_CHANNEL_2G4;
		prSchedScanCmd->ucChnlNum = 0;
	} else if (ePreferedChnl == BAND_5G) {
		prSchedScanCmd->ucChannelType =
			SCAN_CHANNEL_5G;
		prSchedScanCmd->ucChnlNum = 0;
#if (CFG_SUPPORT_WIFI_6G == 1)
	} else if (ePreferedChnl == BAND_6G) {
		prSchedScanCmd->ucChannelType =
			SCAN_CHANNEL_6G;
		prSchedScanCmd->ucChnlNum = 0;
#endif
	} else if (prRequest->ucChnlNum > 0 &&
		prRequest->ucChnlNum <= MAXIMUM_OPERATION_CHANNEL_LIST) {
		prSchedScanCmd->ucChannelType =
			SCAN_CHANNEL_SPECIFIED;
		prSchedScanCmd->ucChnlNum = prRequest->ucChnlNum;
		for (i = 0; i < prRequest->ucChnlNum; i++) {
			prSchedScanCmd->aucChannel[i].ucChannelNum =
				prRequest->aucChannel[i].ucChannelNum;
			prSchedScanCmd->aucChannel[i].ucBand =
				prRequest->aucChannel[i].ucBand;
		}
	} else {
		prSchedScanCmd->ucChnlNum = 0;
		prSchedScanCmd->ucChannelType =
			SCAN_CHANNEL_FULL;
	}

	prSchedScanCmd->ucSeqNum = prSchedScanParam->ucSeqNum;
	prSchedScanCmd->fgStopAfterIndication =
		prSchedScanParam->fgStopAfterIndication;
	prSchedScanCmd->u2IELen = u2IeLen;
	prSchedScanCmd->ucVersion = SCHED_SCAN_CMD_VERSION;
	if (prSchedScanCmd->u2IELen) {
		kalMemCopy(prSchedScanCmd->aucIE, prRequest->pucIE,
				prSchedScanCmd->u2IELen);
	}

	prSchedScanCmd->ucScnFuncMask |= prRequest->ucScnFuncMask;

	scnSetSchedScanPlan(prAdapter, prSchedScanCmd);

	log_dbg(SCN, INFO, "V(%u)seq(%u)sz(%zu)chT(%u)chN(%u)ssid(%u)match(%u)IE(%u=>%u)MSP(%u)Func(0x%X)\n",
		prSchedScanCmd->ucVersion,
		prSchedScanCmd->ucSeqNum, sizeof(struct CMD_SCHED_SCAN_REQ),
		prSchedScanCmd->ucChannelType, prSchedScanCmd->ucChnlNum,
		prSchedScanCmd->ucSsidNum, prSchedScanCmd->ucMatchSsidNum,
		prRequest->u4IELength, prSchedScanCmd->u2IELen,
		prSchedScanCmd->ucMspEntryNum,
		prSchedScanCmd->ucScnFuncMask);

	/* 3. send command packet to FW */
	do {
		if (!scnFsmSchedScanSetCmd(prAdapter, prSchedScanCmd)) {
			log_dbg(SCN, TRACE, "scnFsmSchedScanSetCmd failed\n");
			break;
		}
		if (!scnFsmSchedScanSetAction(prAdapter,
				SCHED_SCAN_ACT_ENABLE)) {
			log_dbg(SCN, TRACE, "scnFsmSchedScanSetAction failed\n");
			break;
		}
		prScanInfo->fgSchedScanning = TRUE;
	} while (0);

	if (!prScanInfo->fgSchedScanning)
		nicDeactivateNetwork(prAdapter,
			prAisBssInfo->ucBssIndex);

	cnmMemFree(prAdapter, (void *) prSchedScanCmd);

	return prScanInfo->fgSchedScanning;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief         handler for stopping scheduled scan
 *
 * \param[in]
 *
 * \return        TRUE if send stop command successfully. FALSE otherwise
 */
/*----------------------------------------------------------------------------*/
u_int8_t scnFsmSchedScanStopRequest(IN struct ADAPTER *prAdapter)
{
	uint8_t ucBssIndex = 0;

	ASSERT(prAdapter);

	ucBssIndex =
		prAdapter->rWifiVar.rScanInfo.rSchedScanParam.ucBssIndex;

	if (aisGetAisBssInfo(prAdapter,
		ucBssIndex) == NULL) {
		log_dbg(SCN, WARN,
			"prAisBssInfo%d is NULL\n",
			ucBssIndex);
		return FALSE;
	}

	if (!scnFsmSchedScanSetAction(prAdapter, SCHED_SCAN_ACT_DISABLE)) {
		log_dbg(SCN, TRACE, "scnFsmSchedScanSetAction failed\n");
		return FALSE;
	}

	prAdapter->rWifiVar.rScanInfo.fgSchedScanning = FALSE;

	return TRUE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief handler for setting schedule scan action
 * \param prAdapter       adapter
 * \param ucSchedScanAct  schedule scan action. set enable/disable to FW
 *
 * \return TRUE if send query command successfully. FALSE otherwise
 */
/*----------------------------------------------------------------------------*/
u_int8_t
scnFsmSchedScanSetAction(IN struct ADAPTER *prAdapter,
		 IN enum ENUM_SCHED_SCAN_ACT ucSchedScanAct)
{
	struct CMD_SET_SCHED_SCAN_ENABLE rCmdSchedScanAction;
	uint32_t rStatus;

	ASSERT(prAdapter);

	kalMemZero(&rCmdSchedScanAction,
		sizeof(struct CMD_SET_SCHED_SCAN_ENABLE));

	/* 0:enable, 1:disable */
	rCmdSchedScanAction.ucSchedScanAct = ucSchedScanAct;

	if (ucSchedScanAct == SCHED_SCAN_ACT_ENABLE) {
		scanlog_dbg(LOG_SCHED_SCAN_REQ_START_D2F, INFO, "sched scan action = %d\n",
			rCmdSchedScanAction.ucSchedScanAct);
	} else {
		scanlog_dbg(LOG_SCHED_SCAN_REQ_STOP_D2F, INFO, "sched scan action = %d\n",
			rCmdSchedScanAction.ucSchedScanAct);
	}

	rStatus = wlanSendSetQueryCmd(prAdapter,
			    CMD_ID_SET_SCAN_SCHED_ENABLE,
			    TRUE,
			    FALSE,
			    FALSE,
			    nicCmdEventSetCommon,
			    nicOidCmdTimeoutCommon,
			    sizeof(struct CMD_SET_SCHED_SCAN_ENABLE),
			    (uint8_t *)&rCmdSchedScanAction, NULL, 0);

	return (rStatus != WLAN_STATUS_FAILURE) ? TRUE : FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief                 handler for setting schedule scan command
 * \param prAdapter       adapter
 * \param prSchedScanCmd  schedule scan command
 *
 * \return                TRUE if send query command successfully.
 *                        FAIL otherwise
 */
/*----------------------------------------------------------------------------*/
u_int8_t
scnFsmSchedScanSetCmd(IN struct ADAPTER *prAdapter,
		IN struct CMD_SCHED_SCAN_REQ *prSchedScanCmd)
{
	uint16_t u2IeSize = 0;
	uint32_t rStatus;

	ASSERT(prAdapter);

	log_dbg(SCN, TRACE, "--> %s()\n", __func__);

	if (prSchedScanCmd)
		u2IeSize = prSchedScanCmd->u2IELen;
	rStatus = wlanSendSetQueryCmd(prAdapter,
			    CMD_ID_SET_SCAN_SCHED_REQ,
			    TRUE,
			    FALSE,
			    FALSE,
			    nicCmdEventSetCommon,
			    nicOidCmdTimeoutCommon,
			    sizeof(struct CMD_SCHED_SCAN_REQ) + u2IeSize,
			    (uint8_t *) prSchedScanCmd, NULL, 0);

	return (rStatus != WLAN_STATUS_FAILURE) ? TRUE : FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief                 Set schedule scan multiple scan plan (scan interval)
 * \param prAdapter       adapter
 * \param prSchedScanCmd  schedule scan command request
 *
 * \return                void
 */
/*----------------------------------------------------------------------------*/
void
scnSetSchedScanPlan(IN struct ADAPTER *prAdapter,
		IN struct CMD_SCHED_SCAN_REQ *prSchedScanCmd)
{
	/* Set Multiple Scan Plan here */
	log_dbg(SCN, TRACE, "--> %s()\n", __func__);

	ASSERT(prAdapter);

	prSchedScanCmd->ucMspEntryNum = 0;
	kalMemZero(prSchedScanCmd->au2MspList,
			sizeof(prSchedScanCmd->au2MspList));
}

#endif /* CFG_SUPPORT_SCHED_SCAN */

#if CFG_SUPPORT_SCAN_NO_AP_RECOVERY
/*----------------------------------------------------------------------------*/
/*!
 * \brief                 Check scan result Mdrdy is all zero or not
 * \param aucChannelMDRDYCnt       Channel Mdrdy result
 * \param ucChNum	  Scan channel count
 *
 * \return                void
 */
/*----------------------------------------------------------------------------*/
void
scnDoZeroMdrdyRecoveryCheck(IN struct ADAPTER *prAdapter,
		IN struct EVENT_SCAN_DONE *prScanDone,
		IN struct SCAN_INFO *prScanInfo, IN uint8_t ucBssIndex)
{
	struct BSS_INFO *prAisBssInfo;
	struct WIFI_VAR *prWifiVar = &prAdapter->rWifiVar;
	uint8_t i;
	uint8_t fgZeroMdrdy = TRUE, fgZeroBeaconProbeReq = TRUE;

	prAisBssInfo = aisGetAisBssInfo(prAdapter, ucBssIndex);

	for (i = 0; i < prScanDone->ucSparseChannelArrayValidNum; i++) {
		if (prScanDone->aucChannelMDRDYCnt[i] > 0)
			fgZeroMdrdy = FALSE;
		if (prScanDone->aucChannelBAndPCnt[i] > 0)
			fgZeroBeaconProbeReq = FALSE;
	}

	/* Abnormal: total Mdrdy=0 or beacon+ProbReq=0 case */
	if (fgZeroMdrdy || fgZeroBeaconProbeReq) {
		prScanInfo->ucScnZeroMdrdyTimes++;

		log_dbg(SCN, WARN, "ScanRecover: Mdrdy(%d),BandP(%d), Count(%d), Conn(%d), SER(%d), Reset(%d)",
			fgZeroMdrdy, fgZeroBeaconProbeReq,
			prScanInfo->ucScnZeroMdrdyTimes,
			prAisBssInfo->eConnectionState,
			prScanInfo->ucScnZeroMdrdySerCnt,
			prScanInfo->ucScnZeroMdrdySubsysResetCnt);

		/* Do L1 SER if continuous abnormal count >= 3,
		 * if first L1 SER not useful, will do twice
		 */
		if (prScanInfo->ucScnZeroMdrdyTimes >= prWifiVar->
			ucScanNoApRecoverTh)
			if (prScanInfo->ucScnZeroMdrdySerCnt < 2) {
				prScanInfo->ucScnZeroMdrdySerCnt++;
				wlanoidSerExtCmd(prAdapter,
						SER_ACTION_SET_ENABLE_MASK,
						(SER_ENABLE_TRACKING |
						SER_ENABLE_L1_RECOVER |
						SER_ENABLE_L2_RECOVER |
						SER_ENABLE_L3_RX_ABORT |
						SER_ENABLE_L3_TX_ABORT |
						SER_ENABLE_L3_TX_DISABLE |
						SER_ENABLE_L3_BF_RECOVER), 0);
				wlanoidSerExtCmd(prAdapter, SER_ACTION_RECOVER,
					SER_SET_L1_RECOVER, 0);
			}
			/* TODO:
			 * If still abnormal after do twice L1 SER
			 * (ucScnZeroMdrdySerCnt >= 2)
			 * do subsys reset if no connection.
			 */
#if 0
			else if (prScanInfo->ucScnZeroMdrdySubsysResetCnt < 1) {
				if (prAisBssInfo->eConnectionState
					== MEDIA_STATE_DISCONNECTED) {
					prScanInfo->
						ucScnZeroMdrdySubsysResetCnt++;
					glSetRstReason(RST_SCAN_RECOVERY);
					GL_RESET_TRIGGER(prAdapter,
						RST_FLAG_CHIP_RESET);
				}
			}
#endif
	}
	/* Normal: Mdrdy>0 and beacon+ProbReq>0 case */
	else {
		prScanInfo->ucScnZeroMdrdyTimes = 0;
		prScanInfo->ucScnZeroMdrdySerCnt = 0;
		prScanInfo->ucScnZeroMdrdySubsysResetCnt = 0;
	}
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief                 Check scan result Mdrdy is all zero or not
 * \param aucChannelMDRDYCnt       Channel Mdrdy result
 * \param ucChNum	  Scan channel count
 *
 * \return                void
 */
/*----------------------------------------------------------------------------*/
void
scnDoScanTimeoutRecoveryCheck(IN struct ADAPTER *prAdapter,
			IN uint8_t ucBssIndex)
{
	struct BSS_INFO *prAisBssInfo;
	struct SCAN_INFO *prScanInfo;
	struct WIFI_VAR *prWifiVar = &prAdapter->rWifiVar;

	prAisBssInfo = aisGetAisBssInfo(prAdapter, ucBssIndex);
	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	prScanInfo->ucScnTimeoutTimes++;

	log_dbg(SCN, WARN, "ScanRecover:Conn(%d), ScnTimeoutCount(%d), Reset(%d)",
		prAisBssInfo->eConnectionState,
		prScanInfo->ucScnTimeoutTimes,
		prScanInfo->ucScnTimeoutSubsysResetCnt);

	/* If scanDoneTimeout count > 3 and no connection, do subsys reset */
	if (prScanInfo->ucScnTimeoutTimes >= prWifiVar->ucScanNoApRecoverTh) {
		if (prScanInfo->ucScnTimeoutSubsysResetCnt < 1 &&
		   prAisBssInfo->eConnectionState == MEDIA_STATE_DISCONNECTED) {
			prScanInfo->ucScnTimeoutSubsysResetCnt++;
			glSetRstReason(RST_SCAN_RECOVERY);
			GL_RESET_TRIGGER(prAdapter,
				RST_FLAG_CHIP_RESET);
		}
	}
}

#endif

enum ENUM_SCN_DONE_REASON {
	SCN_DONE_OK = 0,
	SCN_DONE_TIMEOUT,
	SCN_DONE_DRIVER_ABORT,
	SCN_DONE_NUM
};

void
scnFsmNotifyEvent(IN struct ADAPTER *prAdapter,
		IN enum ENUM_SCAN_STATUS eStatus,
		IN uint8_t ucBssIndex)
{
	struct SCAN_INFO *prScanInfo;
	char uEvent[300], strbuf[200] = "N/A";
	uint8_t fgIsScanNormal = TRUE, fgIsDbdcScan = TRUE;
	uint8_t i, ucReasonInd, ucWritten = 0, fgAnyConnection = FALSE;
	uint8_t ucTotalLen = 200;
	uint8_t *apucScnReason[3] = {
		(uint8_t *) DISP_STRING("OK"),
		(uint8_t *) DISP_STRING("TIMEOUT"),
		(uint8_t *) DISP_STRING("DRIVER ABORT"),
	};
	uint32_t u4ScanTime = 0;

	/*
	 * Status: NORMAL, ABNORMAL (scan timeout or driver abort scan both
	 * classify to ABNORMAL)
	 * DBDC: ENABLE, DISABLE
	 * Reason: TIMEOUT, DRIVER ABORT (if status NORMAL, print OK)
	 * Time: in ms
	 * Channel: (divide by space) ex: 1 2 3 4 5 6
	 *
	 * eStatus = SCAN_STATUS_DONE --> normal scan done OR scan timeout!
	 * eStatus = SCAN_STATUS_CANCELLED --> driver abort
	 */
	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	if (prScanInfo->fgIsScanTimeout || eStatus == SCAN_STATUS_CANCELLED) {
		fgIsScanNormal = FALSE;
		ucReasonInd = prScanInfo->fgIsScanTimeout ? SCN_DONE_TIMEOUT :
				SCN_DONE_DRIVER_ABORT;
		prScanInfo->fgIsScanTimeout = FALSE;
	}
	/* Currently FW only do DBDC scan when no connection, but this condition
	 * might changed.
	 */
	for (i = 0; i < KAL_AIS_NUM; i++) {
		if (IS_BSS_ALIVE(prAdapter, prAdapter->aprBssInfo[i])) {
			fgAnyConnection	= TRUE;
			break;
		}
	}
	if (prAdapter->rWifiVar.eDbdcMode == ENUM_DBDC_MODE_DISABLED
		|| fgAnyConnection)
		fgIsDbdcScan = FALSE;

	if (fgIsScanNormal) {
		u4ScanTime = USEC_TO_MSEC(SYSTIME_TO_USEC(kalGetTimeTick()
					- prScanInfo->rLastScanStartTime));
		ucReasonInd = SCN_DONE_OK;
		for (i = 0; i < prScanInfo->ucSparseChannelArrayValidNum; i++) {
			ucWritten += kalSnprintf(strbuf + ucWritten,
					ucTotalLen - ucWritten, "%d ",
					prScanInfo->aucChannelNum[i]);
		}
	}
	kalSnprintf(uEvent, sizeof(uEvent),
		"Scan=Status:%s,DBDC:%s,Time:%d,Channel:%s,Reason:%s",
		(fgIsScanNormal ? "NORMAL" : "ABNORMAL"),
		(fgIsDbdcScan ? "ENABLE" : "DISABLE"),
		u4ScanTime,
		strbuf,
		apucScnReason[ucReasonInd]);

	DBGLOG(SCN, LOUD, "request uevent:%s\n", uEvent);
	/* Only send Uevent if BSS is AIS */
	if (IS_BSS_INDEX_AIS(prAdapter, ucBssIndex))
		kalSendUevent(uEvent);
}
