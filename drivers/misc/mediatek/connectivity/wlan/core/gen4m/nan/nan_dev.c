/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include "precomp.h"
#include "nan/nan_sec.h"

uint8_t
nanDevInit(IN struct ADAPTER *prAdapter, uint8_t ucIdx) {
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
		(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;
	struct BSS_INFO *prnanBssInfo = (struct BSS_INFO *)NULL;
	struct _GL_NAN_INFO_T *prNANInfo = (struct _GL_NAN_INFO_T *)NULL;
	enum ENUM_PARAM_NAN_MODE_T eNanMode;
	const struct NON_HT_PHY_ATTRIBUTE *prLegacyPhyAttr;
	const struct NON_HT_ADHOC_MODE_ATTRIBUTE *prLegacyModeAttr;
	uint8_t ucLegacyPhyTp;
	struct WIFI_VAR *prWifiVar;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR,
			"[%s] prAdapter is NULL\n", __func__);
		return MAX_BSS_INDEX;
	}

	prnanBssInfo = cnmGetBssInfoAndInit(prAdapter, NETWORK_TYPE_NAN,
					    FALSE);
	if (prnanBssInfo == NULL) {
		DBGLOG(NAN, INFO, "No enough BSS INDEX\n");
		return MAX_BSS_INDEX;
	}

	prNANSpecInfo =
		prAdapter->rWifiVar.aprNanSpecificBssInfo[ucIdx];

	prNANInfo = prAdapter->prGlueInfo->aprNANDevInfo[NAN_BSS_INDEX_BAND0];
	if (prnanBssInfo != NULL) {
		DBGLOG(NAN, INFO, "NAN DEV BSSIFO INDEX %d %p\n",
		       prnanBssInfo->ucBssIndex, prnanBssInfo);
		COPY_MAC_ADDR(prnanBssInfo->aucOwnMacAddr,
			      prNANInfo->prDevHandler->dev_addr);
		prNANSpecInfo->ucBssIndex = prnanBssInfo->ucBssIndex;
		prNANSpecInfo->u4ModuleUsed = 0;
		prnanBssInfo->eCurrentOPMode = OP_MODE_NAN;

		eNanMode = NAN_MODE_MIXED_11BG;
		prLegacyModeAttr = &rNonHTNanModeAttr[eNanMode];
		ucLegacyPhyTp =
			(uint8_t)prLegacyModeAttr->ePhyTypeIndex;
		prLegacyPhyAttr = &rNonHTPhyAttributes[ucLegacyPhyTp];
		prWifiVar = &prAdapter->rWifiVar;

#if (CFG_SUPPORT_DBDC == 1)
		if (ucIdx == NAN_BSS_INDEX_BAND1)
			prnanBssInfo->eBand = BAND_5G;
		else
#endif
			prnanBssInfo->eBand = BAND_2G4;

		prnanBssInfo->u4PrivateData = 0;
		prnanBssInfo->ucSSIDLen = 0;
		prnanBssInfo->fgIsQBSS = 1;
		prnanBssInfo->eConnectionState = MEDIA_STATE_DISCONNECTED;
		prnanBssInfo->ucBMCWlanIndex = WTBL_RESERVED_ENTRY;
		prnanBssInfo->ucOpRxNss = wlanGetSupportNss(
			prAdapter, prnanBssInfo->ucBssIndex);

		/* let secIsProtectedFrame to decide protection or
		 * not by STA record
		 */
		prnanBssInfo->fgIsProtection = FALSE;

#if (CFG_HW_WMM_BY_BSS == 1)
		if (prnanBssInfo->fgIsWmmInited == FALSE)
			prnanBssInfo->ucWmmQueSet = cnmWmmIndexDecision(
				prAdapter, prnanBssInfo);
#else
		prnanBssInfo->ucWmmQueSet =
			(prAdapter->rWifiVar.ucDbdcMode ==
			DBDC_MODE_DISABLED) ? DBDC_5G_WMM_INDEX
			: DBDC_2G_WMM_INDEX;
#endif


#if (CFG_SUPPORT_DBDC == 1)
		if (ucIdx == NAN_BSS_INDEX_BAND1)
			prnanBssInfo->ucPhyTypeSet =
				prWifiVar->ucAvailablePhyTypeSet &
				PHY_TYPE_SET_802_11ANAC;
		else
#endif
			prnanBssInfo->ucPhyTypeSet =
				prWifiVar->ucAvailablePhyTypeSet &
				PHY_TYPE_SET_802_11BGN;

		prnanBssInfo->ucNonHTBasicPhyType = ucLegacyPhyTp;
		if (prLegacyPhyAttr
			    ->fgIsShortPreambleOptionImplemented &&
		    (prWifiVar->ePreambleType == PREAMBLE_TYPE_SHORT ||
		     prWifiVar->ePreambleType == PREAMBLE_TYPE_AUTO))
			prnanBssInfo->fgUseShortPreamble = TRUE;
		else
			prnanBssInfo->fgUseShortPreamble = FALSE;
		prnanBssInfo->fgUseShortSlotTime =
			prLegacyPhyAttr
				->fgIsShortSlotTimeOptionImplemented;

		prnanBssInfo->u2OperationalRateSet =
			prLegacyPhyAttr->u2SupportedRateSet;
		prnanBssInfo->u2BSSBasicRateSet =
			prLegacyModeAttr->u2BSSBasicRateSet;
		/* Mask CCK 1M For Sco scenario except FDD mode */
		if (prAdapter->u4FddMode == FALSE)
			prnanBssInfo->u2BSSBasicRateSet &=
				~RATE_SET_BIT_1M;
		prnanBssInfo->u2VhtBasicMcsSet = 0;

		prnanBssInfo->fgErpProtectMode = FALSE;
		prnanBssInfo->eHtProtectMode = HT_PROTECT_MODE_NONE;
		prnanBssInfo->eGfOperationMode = GF_MODE_DISALLOWED;
		prnanBssInfo->eRifsOperationMode = RIFS_MODE_DISALLOWED;

#if (CFG_SUPPORT_DBDC == 1)
		if (ucIdx == NAN_BSS_INDEX_BAND1)
			prnanBssInfo->ucPrimaryChannel = 149;
		else
#endif
			prnanBssInfo->ucPrimaryChannel = 6;

		prnanBssInfo->eBssSCO = CHNL_EXT_SCN;
		prnanBssInfo->ucHtOpInfo1 = 0;
		prnanBssInfo->u2HtOpInfo2 = 0;
		prnanBssInfo->u2HtOpInfo3 = 0;

#if (CFG_SUPPORT_DBDC == 1)
		if (ucIdx == NAN_BSS_INDEX_BAND1)
			prnanBssInfo->ucVhtChannelWidth = CW_80MHZ;
		else
#endif
			prnanBssInfo->ucVhtChannelWidth = CW_20_40MHZ;

		prnanBssInfo->ucVhtChannelFrequencyS1 = 0;
		prnanBssInfo->ucVhtChannelFrequencyS2 = 0;
		if (prWifiVar->ucNanBandwidth >= MAX_BW_40MHZ) {
			prnanBssInfo->eBssSCO = CHNL_EXT_SCA;
			prnanBssInfo->ucHtOpInfo1 |=
				HT_OP_INFO1_STA_CHNL_WIDTH;
			prnanBssInfo->fgAssoc40mBwAllowed = TRUE;
		}

		prnanBssInfo->u2HwDefaultFixedRateCode = RATE_OFDM_6M;
		rateGetDataRatesFromRateSet(
			prnanBssInfo->u2OperationalRateSet,
			prnanBssInfo->u2BSSBasicRateSet,
			prnanBssInfo->aucAllSupportedRates,
			&prnanBssInfo->ucAllSupportedRatesLen);
		/* Activate NAN BSS */
		if (!IS_BSS_ACTIVE(
			    prAdapter->aprBssInfo
				    [prnanBssInfo->ucBssIndex])) {
			nicUpdateBss(prAdapter, prnanBssInfo->ucBssIndex);

#if (CFG_SUPPORT_DBDC == 1)
			/* Check if DBDC is required to be enabled first */
			cnmDbdcPreConnectionEnableDecision(
				prAdapter,
				prnanBssInfo->ucBssIndex,
				prnanBssInfo->eBand,
				prnanBssInfo->ucPrimaryChannel,
				prnanBssInfo->ucWmmQueSet);
#endif

			/* DBDC decsion may change OpNss */
			cnmOpModeGetTRxNss(prAdapter,
				prnanBssInfo->ucBssIndex,
				&prnanBssInfo->ucOpRxNss,
				&prnanBssInfo->ucOpTxNss);

			SET_NET_ACTIVE(prAdapter,
				prnanBssInfo->ucBssIndex);
			prnanBssInfo->eConnectionState
				= MEDIA_STATE_CONNECTED;

			nicQmUpdateWmmParms(prAdapter,
					    prnanBssInfo->ucBssIndex);
		}
	}

	if (prnanBssInfo)
		return prnanBssInfo->ucBssIndex;
	else
		return MAX_BSS_INDEX;
} /* p2pDevFsmInit */

void
nanDevFsmUninit(IN struct ADAPTER *prAdapter, uint8_t ucIdx) {
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
		(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;
	struct BSS_INFO *prnanBssInfo = (struct BSS_INFO *)NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR,
			"[%s] prAdapter is NULL\n", __func__);
		return;
	}

	for (ucIdx = 0; ucIdx < NAN_BSS_INDEX_NUM; ucIdx++) {
		prNANSpecInfo =
			prAdapter->rWifiVar.aprNanSpecificBssInfo[ucIdx];

		if (prNANSpecInfo == NULL) {
			DBGLOG(NAN, ERROR,
				"[%s] prNANSpecInfo is NULL\n", __func__);
			return;
		}

		prnanBssInfo = prAdapter->aprBssInfo[prNANSpecInfo->ucBssIndex];
		DBGLOG(NAN, INFO, "UNINIT NAN DEV BSSIFO INDEX %d\n",
		       prnanBssInfo->ucBssIndex);

		/* Clear CmdQue */
		kalClearMgmtFramesByBssIdx(prAdapter->prGlueInfo,
					   prnanBssInfo->ucBssIndex);
		kalClearSecurityFramesByBssIdx(prAdapter->prGlueInfo,
					       prnanBssInfo->ucBssIndex);
		/* Clear PendingCmdQue */
		wlanReleasePendingCMDbyBssIdx(prAdapter,
					      prnanBssInfo->ucBssIndex);
		/* Clear PendingTxMsdu */
		nicFreePendingTxMsduInfo(prAdapter, prnanBssInfo->ucBssIndex,
					 MSDU_REMOVE_BY_BSS_INDEX);

		nicPmIndicateBssAbort(prAdapter, prnanBssInfo->ucBssIndex);

		/* Deactivate BSS. */
		prnanBssInfo->eConnectionState = MEDIA_STATE_DISCONNECTED;
		UNSET_NET_ACTIVE(prAdapter, prnanBssInfo->ucBssIndex);
		nicDeactivateNetwork(prAdapter, prnanBssInfo->ucBssIndex);
		nicUpdateBss(prAdapter, prnanBssInfo->ucBssIndex);

		cnmFreeBssInfo(prAdapter, prnanBssInfo);
	}
} /* p2pDevFsmUninit */
struct _NAN_SPECIFIC_BSS_INFO_T *
nanGetSpecificBssInfo(IN struct ADAPTER *prAdapter,
		      enum NAN_BSS_ROLE_INDEX eIndex) {
	return prAdapter->rWifiVar.aprNanSpecificBssInfo[eIndex];
}

uint8_t
nanGetBssIdxbyBand(IN struct ADAPTER *prAdapter,
		      enum ENUM_BAND eBand) {
	uint8_t ucIdx = 0;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo;
	struct BSS_INFO *prBssInfo;

	/* Use default BSS if can't find correct peerSchRec or no band info */
	if (eBand == BAND_NULL) {
		DBGLOG(NAN, WARN, "no band info\n");
		prNANSpecInfo = nanGetSpecificBssInfo(
				prAdapter, NAN_BSS_INDEX_BAND0);
		return prNANSpecInfo->ucBssIndex;
	}

	for (ucIdx = 0; ucIdx < NAN_BSS_INDEX_NUM; ucIdx++) {
		prNANSpecInfo = nanGetSpecificBssInfo(prAdapter, ucIdx);
		prBssInfo = GET_BSS_INFO_BY_INDEX(
			prAdapter,
			prNANSpecInfo->ucBssIndex);

		if (prBssInfo->eBand == eBand)
			break;
	}

	return prNANSpecInfo->ucBssIndex;
}

void
nanDevCommonSetCb(IN struct ADAPTER *prAdapter, IN struct CMD_INFO *prCmdInfo,
		  IN uint8_t *pucEventBuf) {

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	if (prCmdInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prCmdInfo is NULL\n", __func__);
		return;
	}
}

void
nanDevSetMasterPreference(IN struct ADAPTER *prAdapter,
			  uint8_t ucMasterPreference) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_CMD_MASTER_PREFERENCE_T *prCmdNanMasterPreference = NULL;

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_CMD_MASTER_PREFERENCE_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(CNM, ERROR, "Memory allocation fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus =
		nicAddNewTlvElement(NAN_CMD_MASTER_PREFERENCE,
				    sizeof(struct _NAN_CMD_MASTER_PREFERENCE_T),
				    u4CmdBufferLen, prCmdBuffer);

	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(TX, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);

	if (prTlvElement == NULL) {
		DBGLOG(TX, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return;
	}

	prCmdNanMasterPreference =
		(struct _NAN_CMD_MASTER_PREFERENCE_T *)prTlvElement->aucbody;
	prCmdNanMasterPreference->ucMasterPreference = ucMasterPreference;

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, nanDevCommonSetCb,
				      nicCmdTimeoutCommon, u4CmdBufferLen,
				      (uint8_t *)prCmdBuffer, NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);
}

enum NanStatusType
nanDevEnableRequest(IN struct ADAPTER *prAdapter,
		    struct NanEnableRequest *prEnableReq) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct NanEnableRequest *prCmdNanEnableReq = NULL;

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct NanEnableRequest);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(CNM, ERROR, "Memory allocation fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return NAN_STATUS_NO_RESOURCE_AVAILABLE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(NAN_CMD_ENABLE_REQUEST,
				      sizeof(struct NanEnableRequest),
				      u4CmdBufferLen, prCmdBuffer);

	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(TX, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return NAN_STATUS_NO_RESOURCE_AVAILABLE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);

	if (prTlvElement == NULL) {
		DBGLOG(TX, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return NAN_STATUS_NO_RESOURCE_AVAILABLE;
	}

	prCmdNanEnableReq = (struct NanEnableRequest *)prTlvElement->aucbody;
	kalMemCopy(prCmdNanEnableReq, prEnableReq,
		   sizeof(struct NanEnableRequest));

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, nanDevCommonSetCb,
				      nicCmdTimeoutCommon, u4CmdBufferLen,
				      (uint8_t *)prCmdBuffer, NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	if (rStatus == WLAN_STATUS_SUCCESS)
		return NAN_STATUS_SUCCESS;
	else
		return NAN_STATUS_INTERNAL_FAILURE;
}

enum NanStatusType
nanDevDisableRequest(IN struct ADAPTER *prAdapter) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(CNM, ERROR, "Memory allocation fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return NAN_STATUS_NO_RESOURCE_AVAILABLE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(NAN_CMD_DISABLE_REQUEST, 0,
				      u4CmdBufferLen, prCmdBuffer);

	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(TX, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return NAN_STATUS_NO_RESOURCE_AVAILABLE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);

	if (prTlvElement == NULL) {
		DBGLOG(TX, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return NAN_STATUS_NO_RESOURCE_AVAILABLE;
	}

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, nanDevCommonSetCb,
				      nicCmdTimeoutCommon, u4CmdBufferLen,
				      (uint8_t *)prCmdBuffer, NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	if (rStatus == WLAN_STATUS_SUCCESS)
		return NAN_STATUS_SUCCESS;
	else
		return NAN_STATUS_INTERNAL_FAILURE;
}

void
nanDevMasterIndEvtHandler(IN struct ADAPTER *prAdapter, IN uint8_t *pcuEvtBuf) {
	struct _NAN_ATTR_MASTER_INDICATION_T *prMasterIndEvt;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
		(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;

	prMasterIndEvt = (struct _NAN_ATTR_MASTER_INDICATION_T *)pcuEvtBuf;
	/* dumpMemory8((PUINT_8) pcuEvtBuf, 32); */

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	prNANSpecInfo = nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);

	kalMemCopy(&prNANSpecInfo->rMasterIndAttr, prMasterIndEvt,
		   sizeof(struct _NAN_ATTR_MASTER_INDICATION_T));
}

uint32_t
nanDevGetMasterIndAttr(IN struct ADAPTER *prAdapter,
		       uint8_t *pucMasterIndAttrBuf,
		       uint32_t *pu4MasterIndAttrLength) {
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
		(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_FAILURE;
	}

	prNANSpecInfo = nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);

	if (prNANSpecInfo == NULL)
		return WLAN_STATUS_FAILURE;

	kalMemCopy(pucMasterIndAttrBuf, &prNANSpecInfo->rMasterIndAttr,
		   sizeof(struct _NAN_ATTR_MASTER_INDICATION_T));
	*pu4MasterIndAttrLength = sizeof(struct _NAN_ATTR_MASTER_INDICATION_T);

	return WLAN_STATUS_SUCCESS;
}

void
nanDevClusterIdEvtHandler(IN struct ADAPTER *prAdapter, IN uint8_t *pcuEvtBuf) {
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
		(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	prNANSpecInfo = nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);

	kalMemCopy(&prNANSpecInfo->aucClusterId, pcuEvtBuf, MAC_ADDR_LEN);
}

uint32_t
nanDevGetClusterId(IN struct ADAPTER *prAdapter, OUT uint8_t *pucClusterId) {
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
		(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_FAILURE;
	}

	prNANSpecInfo = nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);

	if (prNANSpecInfo == NULL)
		return WLAN_STATUS_FAILURE;

	kalMemCopy(pucClusterId, &prNANSpecInfo->aucClusterId, MAC_ADDR_LEN);

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanDevSendEnableRequestToCnm(IN struct ADAPTER *prAdapter)
{
	struct MSG_CH_REQ *prMsgChReq = NULL;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
	(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;
	struct BSS_INFO *prnanBssInfo = (struct BSS_INFO *)NULL;
	uint8_t ucIdx;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_FAILURE;
	}

	prNANSpecInfo = prAdapter
		->rWifiVar.aprNanSpecificBssInfo[NAN_BSS_INDEX_BAND0];
	prnanBssInfo =
		GET_BSS_INFO_BY_INDEX(prAdapter, prNANSpecInfo->ucBssIndex);
	if (prNANSpecInfo == NULL) {
		DBGLOG(NAN, ERROR,
			"[%s] prNANSpecInfo is NULL\n", __func__);
		return WLAN_STATUS_FAILURE;
	}

	prMsgChReq =
		(struct MSG_CH_REQ *)cnmMemAlloc(prAdapter,
		RAM_TYPE_MSG,
		sizeof(struct MSG_CH_REQ));
	if (!prMsgChReq) {
		DBGLOG(NAN, ERROR, "Can't indicate CNM\n");
		return WLAN_STATUS_FAILURE;
	}

	prMsgChReq->rMsgHdr.eMsgId = MID_MNY_CNM_CH_REQ;
	prMsgChReq->ucBssIndex = prNANSpecInfo->ucBssIndex;
	prMsgChReq->ucTokenID = ++prAdapter->ucNanReqTokenId;
	prMsgChReq->ucPrimaryChannel = prnanBssInfo->ucPrimaryChannel;
	prMsgChReq->eRfSco = prnanBssInfo->eBssSCO;
	prMsgChReq->eRfBand = prnanBssInfo->eBand;
	prMsgChReq->eRfChannelWidth = prnanBssInfo->ucVhtChannelWidth;
	prMsgChReq->ucRfCenterFreqSeg1 = prnanBssInfo->ucVhtChannelFrequencyS1;
	prMsgChReq->ucRfCenterFreqSeg2 = prnanBssInfo->ucVhtChannelFrequencyS1;
	prMsgChReq->eReqType = CH_REQ_TYPE_NAN_ON;
	prMsgChReq->u4MaxInterval = 20;
	prMsgChReq->eDBDCBand = ENUM_BAND_AUTO;

	DBGLOG(NAN, INFO,
		"NAN req CH for N:%d,Tkn,%d\n",
		prMsgChReq->ucBssIndex,
		prMsgChReq->ucTokenID);

	/** Set BSS to active */
	for (ucIdx = 0; ucIdx < NAN_BSS_INDEX_NUM; ucIdx++) {
		prNANSpecInfo = prAdapter
			->rWifiVar.aprNanSpecificBssInfo[ucIdx];
		prnanBssInfo = GET_BSS_INFO_BY_INDEX(
					prAdapter, prNANSpecInfo->ucBssIndex);

		UNSET_NET_ACTIVE(prAdapter, prnanBssInfo->ucBssIndex);
	}

	mboxSendMsg(prAdapter, MBOX_ID_0,
		(struct MSG_HDR *)prMsgChReq,
		MSG_SEND_METHOD_BUF);

	prAdapter->fgIsNanSendRequestToCnm = TRUE;

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanDevSendAbortRequestToCnm(IN struct ADAPTER *prAdapter)
{
	struct MSG_CH_ABORT *prMsgChAbort = NULL;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
		(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_FAILURE;
	}

	prNANSpecInfo = prAdapter
		->rWifiVar.aprNanSpecificBssInfo[NAN_BSS_INDEX_BAND0];
	if (prNANSpecInfo == NULL) {
		DBGLOG(NAN, ERROR,
			"[%s] prNANSpecInfo is NULL\n", __func__);
		return WLAN_STATUS_FAILURE;
	}

	prMsgChAbort =
	(struct MSG_CH_ABORT *)cnmMemAlloc(prAdapter,
		RAM_TYPE_MSG,
		sizeof(struct MSG_CH_ABORT));
	if (!prMsgChAbort) {
		DBGLOG(NAN, ERROR, "Can't indicate CNM\n");
		return WLAN_STATUS_FAILURE;
	}

	prMsgChAbort->rMsgHdr.eMsgId = MID_MNY_CNM_CH_ABORT;
	prMsgChAbort->ucBssIndex = prNANSpecInfo->ucBssIndex;
	prMsgChAbort->ucTokenID = prAdapter->ucNanReqTokenId;
	prMsgChAbort->eDBDCBand = ENUM_BAND_AUTO;

	DBGLOG(NAN, INFO,
		"NAN abort CH for N:%d,Tkn,%d\n",
		prMsgChAbort->ucBssIndex,
		prMsgChAbort->ucTokenID);

	mboxSendMsg(prAdapter, MBOX_ID_0,
		(struct MSG_HDR *)prMsgChAbort,
		MSG_SEND_METHOD_BUF);

	prAdapter->fgIsNanSendRequestToCnm = FALSE;

	return WLAN_STATUS_SUCCESS;
}

void
nanDevSendEnableRequest(struct ADAPTER *prAdapter,
		     struct MSG_HDR *prMsgHdr)
{
	struct NanEnableRequest rEnableReq;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNANSpecInfo =
		(struct _NAN_SPECIFIC_BSS_INFO_T *)NULL;
	struct BSS_INFO *prnanBssInfo = (struct BSS_INFO *)NULL;
	uint8_t ucIdx;

	/** Update BSS Info and set BSS to connected state */
	for (ucIdx = 0; ucIdx < NAN_BSS_INDEX_NUM; ucIdx++) {

		prNANSpecInfo = prAdapter->rWifiVar
				.aprNanSpecificBssInfo[ucIdx];
		prnanBssInfo =
			prAdapter->aprBssInfo[prNANSpecInfo->ucBssIndex];

		if (!IS_BSS_ACTIVE(prnanBssInfo)) {
			SET_NET_ACTIVE(prAdapter, prnanBssInfo->ucBssIndex);
			nicActivateNetwork(prAdapter, prnanBssInfo->ucBssIndex);
		}

		prnanBssInfo->eConnectionState = MEDIA_STATE_CONNECTED;

		nicUpdateBss(prAdapter, prnanBssInfo->ucBssIndex);
	}

	/** Send NAN enable request to FW */
	kalMemZero(&rEnableReq, sizeof(struct NanEnableRequest));
	rEnableReq.master_pref = prAdapter->rWifiVar.ucMasterPref;
	rEnableReq.config_random_factor_force = 0;
	rEnableReq.random_factor_force_val = 0;
	rEnableReq.config_hop_count_force = 0;
	rEnableReq.hop_count_force_val = 0;
	rEnableReq.config_5g_channel = prAdapter->rWifiVar.ucConfig5gChannel;
	rEnableReq.channel_5g_val = prAdapter->rWifiVar.ucChannel5gVal;

	nanDevEnableRequest(prAdapter, &rEnableReq);

	nanDevSendAbortRequestToCnm(prAdapter);

	cnmMemFree(prAdapter, prMsgHdr);
}
