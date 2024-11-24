/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include "precomp.h"
#include "nan_base.h"

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define UINT8_MAX 0xFF

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

uint8_t g_aucRangingIEBuffer[NAN_IE_BUF_MAX_SIZE];

static uint8_t *apucDebugRangingState[RANGING_STATE_NUM] = {
	(uint8_t *)DISP_STRING("IDLE"),
	(uint8_t *)DISP_STRING("INIT"),
	(uint8_t *)DISP_STRING("SCHEDULE"),
	(uint8_t *)DISP_STRING("REQUEST"),
	(uint8_t *)DISP_STRING("REQUEST_IND"),
	(uint8_t *)DISP_STRING("RESPONSE"),
	(uint8_t *)DISP_STRING("ACTIVE"),
	(uint8_t *)DISP_STRING("REPORT"),
	(uint8_t *)DISP_STRING("TERMINATE"),
};

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

/************************************************
 *   Initialization and Uninitialization
 ************************************************
 */
void
nanRangingEngineInit(struct ADAPTER *prAdapter) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	prRangingInfo = &(prAdapter->rRangingInfo);

	kalMemSet(prRangingInfo, 0, sizeof(struct _NAN_RANGING_INFO_T));

	dl_list_init(&prRangingInfo->ranging_list);

	prRangingInfo->ranging_cfg_def.ranging_resolution = 0; /* unused */
	prRangingInfo->ranging_cfg_def.ranging_interval_msec = 1000;
	prRangingInfo->ranging_cfg_def.config_ranging_indications =
		NAN_RANGING_INDICATE_CONTINUOUS_MASK;
	prRangingInfo->ranging_cfg_def.distance_ingress_cm = 0;
	prRangingInfo->ranging_cfg_def.distance_egress_cm = 0;

	prRangingInfo->response_ctl_def.ranging_auto_response =
		NAN_RANGING_AUTO_RESPONSE_ENABLE;
	prRangingInfo->response_ctl_def.range_report = NAN_ENABLE_RANGE_REPORT;
	prRangingInfo->response_ctl_def.ranging_response_code =
		NAN_RANGE_REQUEST_ACCEPT;
}

void
nanRangingEngineUninit(struct ADAPTER *prAdapter) {
	struct dl_list *ranging_list;
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	ranging_list = &prAdapter->rRangingInfo.ranging_list;
	if (ranging_list == NULL)
		return;

	do {
		prRanging = dl_list_first(ranging_list,
					  struct _NAN_RANGING_INSTANCE_T, list);

		if (prRanging) {
			nanRangingInstanceDel(prAdapter, prRanging);
			cnmMemFree(prAdapter, prRanging);
		} else {
			break;
		}

	} while (1);
}

/************************************************
 *   NAN Ranging Instance
 ************************************************
 */
void
nanRangingInstanceInit(struct ADAPTER *prAdapter,
		       struct _NAN_RANGING_INSTANCE_T *prRanging,
		       uint8_t *puc_peer_mac, uint8_t ucRole) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	if (prRanging == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRanging is NULL\n", __func__);
		return;
	}

	prRangingInfo = &(prAdapter->rRangingInfo);

	kalMemSet(&prRanging->ranging_ctrl, 0,
		  sizeof(struct _NAN_RANGING_CTRL_T));

	kalMemCopy(&prRanging->ranging_ctrl.aucPeerAddr[0], &puc_peer_mac[0],
		   MAC_ADDR_LEN);

	prRanging->ranging_ctrl.ucRole = ucRole;
	prRanging->ranging_ctrl.ucInvoker = NAN_RANGING_APPLICATION;

	kalMemCopy(&prRanging->ranging_ctrl.ranging_cfg,
		   &prRangingInfo->ranging_cfg_def,
		   sizeof(struct NanRangingCfg));
	kalMemCopy(&prRanging->ranging_ctrl.response_ctl,
		   &prRangingInfo->response_ctl_def,
		   sizeof(struct NanRangeResponseCtl));

	prRanging->ranging_ctrl.rNanFtmParam.fgRttTrigger = TRUE;
	prRanging->ranging_ctrl.rNanFtmParam.ucAPStatus = RTT_AP_SUCCESS;
	prRanging->ranging_ctrl.rNanFtmParam.ucFTMNum = 2;
	prRanging->ranging_ctrl.rNanFtmParam.ucMinDeltaIn100US = 10;
	prRanging->ranging_ctrl.rNanFtmParam.ucFTMBandwidth =
		prAdapter->rWifiVar.ucNanFtmBw;
	prRanging->ranging_ctrl.rNanFtmParam.u2TSFTimer = 0;
	prRanging->ranging_ctrl.rNanFtmParam.uc2BurstTimeout = 11;
	prRanging->ranging_ctrl.rNanFtmParam.ucBurstExponent = 0;

	DBGLOG(NAN, INFO, "Default FTMBandwidth (%d)\n",
	       prAdapter->rWifiVar.ucNanFtmBw);

	if (ucRole == NAN_PROTOCOL_INITIATOR) {
		prRanging->ranging_ctrl.rNanFtmParam.fgASAP = 1;
		prRanging->ranging_ctrl.rNanFtmParam.fgASAP_CAP = 0;
	} else {
		prRanging->ranging_ctrl.rNanFtmParam.fgASAP = 1;
		prRanging->ranging_ctrl.rNanFtmParam.fgASAP_CAP = 1;
	}

	cnmTimerInitTimer(prAdapter,
			  &(prRanging->ranging_ctrl.rRangingSessionTimer),
			  (PFN_MGMT_TIMEOUT_FUNC)nanRangingSessionTimeout,
			  (unsigned long)prRanging);

	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_INIT);
}

void
nanRangingInstanceAdd(struct ADAPTER *prAdapter,
		      struct _NAN_RANGING_INSTANCE_T *prRanging) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	if (prRanging == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRanging is NULL\n", __func__);
		return;
	}

	prRangingInfo = &(prAdapter->rRangingInfo);

	if (prRangingInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRangingInfo is NULL\n", __func__);
		return;
	}

	dl_list_add(&prRangingInfo->ranging_list, &(prRanging->list));

	prRangingInfo->u2RangingCnt++;

	prRanging->ranging_ctrl.u2RangingId = nanRangingGenerateId(prAdapter);

	DBGLOG(NAN, INFO, "ID (%d)\n", prRanging->ranging_ctrl.u2RangingId);
}

void
nanRangingInstanceDel(struct ADAPTER *prAdapter,
		      struct _NAN_RANGING_INSTANCE_T *prRanging) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	if (prRanging == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRanging is NULL\n", __func__);
		return;
	}

	prRangingInfo = &(prAdapter->rRangingInfo);
	if (prRangingInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRangingInfo is NULL\n", __func__);
		return;
	}

	DBGLOG(NAN, INFO, "ID (%d)\n", prRanging->ranging_ctrl.u2RangingId);

	dl_list_del(&(prRanging->list));

	prRangingInfo->u2RangingCnt--;
}

uint8_t
nanRangingGenerateId(struct ADAPTER *prAdapter) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;
	struct _NAN_RANGING_INSTANCE_T *prRanging;
	uint8_t ucRangingId;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return 0;
	}

	prRangingInfo = &(prAdapter->rRangingInfo);
	ucRangingId = prRangingInfo->ucSeqNum;

	/* ID conflict check */
	do {
		/* incremental */
		if (ucRangingId < 255)
			ucRangingId++;
		else
			ucRangingId = 1; /* keep 0 as reserved purposes ... */

		prRanging =
			nanRangingInstanceSearchById(prAdapter, ucRangingId);
		if (prRanging == NULL)
			break;

	} while (1);

	/* update to local tracker */
	prRangingInfo->ucSeqNum = ucRangingId;

	return ucRangingId;
}

struct _NAN_RANGING_INSTANCE_T *
nanRangingInstanceSearchByMac(struct ADAPTER *prAdapter,
			      const uint8_t *puc_peer_mac) {
	struct dl_list *ranging_list;
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return NULL;
	}

	if (puc_peer_mac == NULL)
		return NULL;

	ranging_list = &prAdapter->rRangingInfo.ranging_list;
	if (ranging_list == NULL)
		return NULL;

	dl_list_for_each(prRanging, ranging_list,
			 struct _NAN_RANGING_INSTANCE_T, list) {

		if (prRanging) {
			if (kalMemCmp(prRanging->ranging_ctrl.aucPeerAddr,
				      puc_peer_mac, MAC_ADDR_LEN) == 0)
				return prRanging;
		}
	}

	return NULL;
}

struct _NAN_RANGING_INSTANCE_T *
nanRangingInstanceSearchById(struct ADAPTER *prAdapter, uint16_t u2RangingId) {
	struct dl_list *ranging_list;
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return NULL;
	}

	if (u2RangingId == 0)
		return NULL;

	ranging_list = &prAdapter->rRangingInfo.ranging_list;
	if (ranging_list == NULL)
		return NULL;

	dl_list_for_each(prRanging, ranging_list,
			 struct _NAN_RANGING_INSTANCE_T, list) {

		if (prRanging) {
			if (prRanging->ranging_ctrl.u2RangingId == u2RangingId)
				return prRanging;
		}
	}

	return NULL;
}

/************************************************
 *   NAN Ranging Attribute
 ************************************************
 */
uint32_t
nanGetFtmRangeReportAttr(struct ADAPTER *prAdapter, uint8_t **ppucAttr,
			 uint32_t *pu4AttrLen, uint8_t *pucDevAddr) {
	struct _NAN_RANGING_INSTANCE_T *prRanging;
	uint8_t *pucPos;
	uint8_t ucRangeEntryCnt;
	uint8_t ucErrorEntryCnt;
	struct _NAN_ATTR_FTM_RANGE_REPORT_T *prAttr;
	uint32_t u4Idx;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prRanging = nanRangingInstanceSearchByMac(prAdapter, pucDevAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_RESOURCES;

	kalMemZero(g_aucRangingIEBuffer, NAN_IE_BUF_MAX_SIZE);

	prAttr = (struct _NAN_ATTR_FTM_RANGE_REPORT_T *)g_aucRangingIEBuffer;
	prAttr->ucAttrId = NAN_ATTR_ID_FTM_RANGING_REPORT;

	pucPos = prAttr->aucFtmRangeReport;

	ucRangeEntryCnt = prRanging->ranging_ctrl.rNanFtmReport.ucRangeEntryCnt;
	*(pucPos++) = ucRangeEntryCnt;

	for (u4Idx = 0; u4Idx < ucRangeEntryCnt; u4Idx++) {
		kalMemCopy(pucPos, &prRanging->ranging_ctrl.rNanFtmReport
					    .arRangeEntry[u4Idx],
			   sizeof(struct _FTM_REPORT_RANGE_ENTRY_T));
		pucPos += sizeof(struct _FTM_REPORT_RANGE_ENTRY_T);
	}

	ucErrorEntryCnt = prRanging->ranging_ctrl.rNanFtmReport.ucErrorEntryCnt;
	*(pucPos++) = ucErrorEntryCnt;

	for (u4Idx = 0; u4Idx < ucErrorEntryCnt; u4Idx++) {
		kalMemCopy(pucPos, &prRanging->ranging_ctrl.rNanFtmReport
					    .arErrorEntry[u4Idx],
			   sizeof(struct _FTM_REPORT_ERROR_ENTRY_T));
		pucPos += sizeof(struct _FTM_REPORT_ERROR_ENTRY_T);
	}

	prAttr->u2Length = (pucPos - g_aucRangingIEBuffer) - NAN_ATTR_HDR_LEN;

	*ppucAttr = g_aucRangingIEBuffer;
	*pu4AttrLen = (pucPos - g_aucRangingIEBuffer);

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanGetRangingInfoAttr(struct ADAPTER *prAdapter, uint8_t **ppucAttr,
		      uint32_t *pu4AttrLen, uint8_t *pucDevAddr) {
	struct _NAN_RANGING_INSTANCE_T *prRanging;
	uint8_t *pucPos;
	struct _NAN_ATTR_RANGING_INFO_T *prAttr;
	unsigned char fgLastMovement = FALSE;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prRanging = nanRangingInstanceSearchByMac(prAdapter, pucDevAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_RESOURCES;

	kalMemZero(g_aucRangingIEBuffer, NAN_IE_BUF_MAX_SIZE);

	prAttr = (struct _NAN_ATTR_RANGING_INFO_T *)g_aucRangingIEBuffer;
	prAttr->ucAttrId = NAN_ATTR_ID_RANGING_INFORMATION;
	prAttr->ucLocationInfo =
		prRanging->ranging_ctrl.location_info_availability;
	pucPos = (uint8_t *)&prAttr->u2LastMovement;

	fgLastMovement =
		(prAttr->ucLocationInfo & NAN_RANGING_LAST_MOVEMENT_PRESENT)
			? TRUE
			: FALSE;
	if (fgLastMovement) {
		prAttr->u2LastMovement =
			prRanging->ranging_ctrl.location_last_indication;
		pucPos += sizeof(prAttr->u2LastMovement);
	}

	prAttr->u2Length = (pucPos - g_aucRangingIEBuffer) - NAN_ATTR_HDR_LEN;

	*ppucAttr = g_aucRangingIEBuffer;
	*pu4AttrLen = (pucPos - g_aucRangingIEBuffer);

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanGetRangingSetupAttr(struct ADAPTER *prAdapter, uint8_t **ppucAttr,
		       uint32_t *pu4AttrLen, uint8_t *pucDevAddr) {
	struct _NAN_RANGING_INSTANCE_T *prRanging;
	struct _NAN_ATTR_RANGING_SETUP_T *prAttr;
	uint8_t *pucPos;
	unsigned char fgFtmParameter = TRUE;
	unsigned char fgScheduleEntry = TRUE;
	uint32_t u4Status = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prRanging = nanRangingInstanceSearchByMac(prAdapter, pucDevAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_RESOURCES;

	kalMemZero(g_aucRangingIEBuffer, NAN_IE_BUF_MAX_SIZE);

	prAttr = (struct _NAN_ATTR_RANGING_SETUP_T *)g_aucRangingIEBuffer;
	prAttr->ucAttrId = NAN_ATTR_ID_RANGING_SETUP;
	prAttr->ucDialogToken = prRanging->ranging_ctrl.dialog_token;
	prAttr->ucTypeStatus = prRanging->ranging_ctrl.TypeStatus;
	prAttr->ucReasonCode = prRanging->ranging_ctrl.ReasonCode;
	prAttr->ucRangingCtl = prRanging->ranging_ctrl.RangingControl;
	pucPos = (uint8_t *)&prAttr->rFtmParameter;

	if ((prRanging->ranging_ctrl.TypeStatus & NAN_RANGING_TYPE_MASK) ==
	    NAN_RANGING_TYPE_TERMINATION) {
		fgFtmParameter = FALSE;
		fgScheduleEntry = FALSE;
	}

	if (fgFtmParameter) {
		prAttr->rFtmParameter.max_burst_duration =
			prRanging->ranging_ctrl.rNanFtmParam.uc2BurstTimeout;
		prAttr->rFtmParameter.min_delata_ftm =
			prRanging->ranging_ctrl.rNanFtmParam.ucMinDeltaIn100US;
		prAttr->rFtmParameter.max_ftms_per_brrst =
			prRanging->ranging_ctrl.rNanFtmParam.ucFTMNum;
		prAttr->rFtmParameter.ftm_format_and_bandwidth =
			prRanging->ranging_ctrl.rNanFtmParam.ucFTMBandwidth;

		prAttr->ucRangingCtl |= NAN_RANGING_CTL_FTM_PARAMETERS_PRESENT;
		pucPos = prAttr->aucScheduleEntryList;
	} else {
		prAttr->ucRangingCtl &= ~NAN_RANGING_CTL_FTM_PARAMETERS_PRESENT;
	}

	prAttr->ucRangingCtl &= ~NAN_RANGING_CTL_SCHEDULE_ENTRY_PRESENT;

	if (fgScheduleEntry) {
		uint8_t *pucSched;
		uint32_t u4SchedLen;

		u4Status = nanSchedNegoGetRangingScheduleList(
			prAdapter, &pucSched, &u4SchedLen);

		if (u4Status == WLAN_STATUS_SUCCESS) {
			kalMemCopy(pucPos, pucSched, u4SchedLen);
			pucPos += u4SchedLen;
			prAttr->ucRangingCtl |=
				NAN_RANGING_CTL_SCHEDULE_ENTRY_PRESENT;
		}
	}

	/* sync final Ranging Control */
	prRanging->ranging_ctrl.RangingControl = prAttr->ucRangingCtl;

	prAttr->u2Length = (pucPos - g_aucRangingIEBuffer) - NAN_ATTR_HDR_LEN;

	*ppucAttr = g_aucRangingIEBuffer;
	*pu4AttrLen = (pucPos - g_aucRangingIEBuffer);

	return WLAN_STATUS_SUCCESS;
}

void
nanRangingFrameCompose(struct ADAPTER *prAdapter, struct MSDU_INFO *prMsduInfo,
		       struct _NAN_RANGING_INSTANCE_T *prRanging,
		       uint8_t ucNafSubType) {
	struct _NAN_ACTION_FRAME_T *prActionFrame = NULL;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNanSpecificBssInfo;
	struct BSS_INFO *prBssInfo;
	uint16_t u2FrameCtrl;
	uint8_t aucOui[] = VENDOR_OUI_WFA_SPECIFIC;
	uint8_t *pucAttr;
	uint32_t u4AttrLen;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	if (prMsduInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prMsduInfo is NULL\n", __func__);
		return;
	}

	/* Get BSS info */
	prNanSpecificBssInfo =
		nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter,
					  prNanSpecificBssInfo->ucBssIndex);

	prActionFrame = (struct _NAN_ACTION_FRAME_T *)prMsduInfo->prPacket;

	/* MAC Header */

	/* Decide the Frame Control Field and AID Field */
	u2FrameCtrl = MAC_FRAME_ACTION;

	/* Fill the Frame Control field. */
	prActionFrame->u2FrameCtrl = u2FrameCtrl;

	/* Fill the DA field */
	COPY_MAC_ADDR(prActionFrame->aucDestAddr,
		      prRanging->ranging_ctrl.aucPeerAddr);

	/* Fill the SA field. */
	COPY_MAC_ADDR(prActionFrame->aucSrcAddr, prBssInfo->aucOwnMacAddr);

	/* Fill the BSSID field with the desired BSSID. */
	COPY_MAC_ADDR(prActionFrame->aucClusterID,
		      prNanSpecificBssInfo->aucClusterId);

	prActionFrame->u2SeqCtrl = 0;

	/* NAF Header (Fixed Field) */
	prActionFrame->ucCategory = CATEGORY_PUBLIC_ACTION;
	prActionFrame->ucAction = ACTION_PUBLIC_VENDOR_SPECIFIC;
	kalMemCopy(prActionFrame->aucOUI, aucOui, VENDOR_OUI_LEN);
	prActionFrame->ucOUItype = VENDOR_OUI_TYPE_NAN_NAF;
	prActionFrame->ucOUISubtype = ucNafSubType;

	prMsduInfo->u2FrameLength =
		OFFSET_OF(struct _NAN_ACTION_FRAME_T, aucInfoContent);

	/* Playload (Mandatory) */

	/* Ranging FTM Range Report */
	if (ucNafSubType == NAN_ACTION_RANGING_REPORT) {
		nanGetFtmRangeReportAttr(prAdapter, &pucAttr, &u4AttrLen,
					 prRanging->ranging_ctrl.aucPeerAddr);
		kalMemCopy(((uint8_t *)prMsduInfo->prPacket) +
				   prMsduInfo->u2FrameLength,
			   pucAttr, u4AttrLen);
		prMsduInfo->u2FrameLength += u4AttrLen;
	}

	/* Ranging Info */
	if ((ucNafSubType == NAN_ACTION_RANGING_REQUEST) ||
	    (ucNafSubType == NAN_ACTION_RANGING_RESPONSE)) {
		nanGetRangingInfoAttr(prAdapter, &pucAttr, &u4AttrLen,
				      prRanging->ranging_ctrl.aucPeerAddr);
		kalMemCopy(((uint8_t *)prMsduInfo->prPacket) +
				   prMsduInfo->u2FrameLength,
			   pucAttr, u4AttrLen);
		prMsduInfo->u2FrameLength += u4AttrLen;
	}

	/* Ranging Setup */
	if ((ucNafSubType == NAN_ACTION_RANGING_REQUEST) ||
	    (ucNafSubType == NAN_ACTION_RANGING_RESPONSE) ||
	    (ucNafSubType == NAN_ACTION_RANGING_TERMINATION)) {
		nanGetRangingSetupAttr(prAdapter, &pucAttr, &u4AttrLen,
				       prRanging->ranging_ctrl.aucPeerAddr);
		kalMemCopy(((uint8_t *)prMsduInfo->prPacket) +
				   prMsduInfo->u2FrameLength,
			   pucAttr, u4AttrLen);
		prMsduInfo->u2FrameLength += u4AttrLen;
	}

	/* NAN Availability */
	if ((ucNafSubType == NAN_ACTION_RANGING_REQUEST) ||
	    (ucNafSubType == NAN_ACTION_RANGING_RESPONSE)) {

		uint32_t rStatus = WLAN_STATUS_SUCCESS;

		rStatus = nanSchedGetAvailabilityAttr(prAdapter, &pucAttr,
						      &u4AttrLen);

		DBGLOG(NAN, INFO, "nanSchedGetAvailabilityAttr 0x%08x\n",
		       rStatus);

		if (rStatus == WLAN_STATUS_SUCCESS) {
			kalMemCopy(((uint8_t *)prMsduInfo->prPacket) +
					   prMsduInfo->u2FrameLength,
				   pucAttr, u4AttrLen);
			prMsduInfo->u2FrameLength += u4AttrLen;
		}
	}

	/* Device Capability */
	if ((ucNafSubType == NAN_ACTION_RANGING_REQUEST) ||
	    (ucNafSubType == NAN_ACTION_RANGING_RESPONSE)) {
		if (nanSchedGetDevCapabilityAttr(prAdapter, &pucAttr,
						 &u4AttrLen) ==
		    WLAN_STATUS_SUCCESS) {
			if ((pucAttr != NULL) && (u4AttrLen != 0)) {
				kalMemCopy(((uint8_t *)prMsduInfo->prPacket) +
						   prMsduInfo->u2FrameLength,
					   pucAttr, u4AttrLen);
				prMsduInfo->u2FrameLength += u4AttrLen;
			}
		}
	}
}

uint32_t
nanFtmRangeReportAttrHandler(struct ADAPTER *prAdapter,
			     struct _NAN_ATTR_FTM_RANGE_REPORT_T *prAttr,
			     uint8_t *pucDevAddr) {
	struct _NAN_RANGING_INSTANCE_T *prRanging;
	uint8_t *pucPos;
	uint8_t ucRangeEntryCnt;
	uint8_t ucErrorEntryCnt;
	uint32_t u4Idx;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prRanging = nanRangingInstanceSearchByMac(prAdapter, pucDevAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_RESOURCES;

	pucPos = prAttr->aucFtmRangeReport;

	ucRangeEntryCnt = *(pucPos++);
	if (ucRangeEntryCnt >= NAN_FTM_REPORT_OK_MAX_NUM)
		ucRangeEntryCnt = NAN_FTM_REPORT_OK_MAX_NUM;
	prRanging->ranging_ctrl.rNanFtmReport.ucRangeEntryCnt = ucRangeEntryCnt;

	for (u4Idx = 0; u4Idx < ucRangeEntryCnt; u4Idx++) {
		kalMemCopy(&prRanging->ranging_ctrl.rNanFtmReport
				    .arRangeEntry[u4Idx],
			   pucPos, sizeof(struct _FTM_REPORT_RANGE_ENTRY_T));
		pucPos += sizeof(struct _FTM_REPORT_RANGE_ENTRY_T);
	}

	ucErrorEntryCnt = *(pucPos++);
	if (ucErrorEntryCnt >= NAN_FTM_REPORT_NG_MAX_NUM)
		ucErrorEntryCnt = NAN_FTM_REPORT_NG_MAX_NUM;
	prRanging->ranging_ctrl.rNanFtmReport.ucErrorEntryCnt = ucErrorEntryCnt;

	for (u4Idx = 0; u4Idx < ucErrorEntryCnt; u4Idx++) {
		kalMemCopy(&prRanging->ranging_ctrl.rNanFtmReport
				    .arErrorEntry[u4Idx],
			   pucPos, sizeof(struct _FTM_REPORT_ERROR_ENTRY_T));
		pucPos += sizeof(struct _FTM_REPORT_ERROR_ENTRY_T);
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanRangingInfoAttrHandler(struct ADAPTER *prAdapter,
			  struct _NAN_ATTR_RANGING_INFO_T *prAttr,
			  uint8_t *pucDevAddr) {
	struct _NAN_RANGING_INSTANCE_T *prRanging;
	unsigned char fgLastMovement = FALSE;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prRanging = nanRangingInstanceSearchByMac(prAdapter, pucDevAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_RESOURCES;

	prRanging->ranging_ctrl.location_info_availability =
		prAttr->ucLocationInfo;

	fgLastMovement =
		(prAttr->ucLocationInfo & NAN_RANGING_LAST_MOVEMENT_PRESENT)
			? TRUE
			: FALSE;
	if (fgLastMovement)
		prRanging->ranging_ctrl.location_last_indication =
			prAttr->u2LastMovement;

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanRangingSetupAttrHandler(struct ADAPTER *prAdapter,
			   struct _NAN_ATTR_RANGING_SETUP_T *prAttr,
			   uint8_t *pucDevAddr) {
	struct _NAN_RANGING_INSTANCE_T *prRanging;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Length;
	uint8_t *pucPos;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prRanging = nanRangingInstanceSearchByMac(prAdapter, pucDevAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_RESOURCES;

	prRanging->ranging_ctrl.dialog_token = prAttr->ucDialogToken;

	prRanging->ranging_ctrl.TypeStatus = prAttr->ucTypeStatus;

	if (((prRanging->ranging_ctrl.TypeStatus & NAN_RANGING_TYPE_MASK) ==
	     NAN_RANGING_TYPE_RESPONSE) ||
	    ((prRanging->ranging_ctrl.TypeStatus & NAN_RANGING_TYPE_MASK) ==
	     NAN_RANGING_TYPE_TERMINATION)) {
		prRanging->ranging_ctrl.ReasonCode = prAttr->ucReasonCode;
	}

	prRanging->ranging_ctrl.RangingControl = prAttr->ucRangingCtl;
	u4Length = OFFSET_OF(struct _NAN_ATTR_RANGING_SETUP_T, rFtmParameter);
	pucPos = (uint8_t *)&prAttr->rFtmParameter;

	if (prRanging->ranging_ctrl.RangingControl &
	    NAN_RANGING_CTL_FTM_PARAMETERS_PRESENT) {
		prRanging->ranging_ctrl.rNanFtmParam.uc2BurstTimeout =
			prAttr->rFtmParameter.max_burst_duration;
		prRanging->ranging_ctrl.rNanFtmParam.ucMinDeltaIn100US =
			prAttr->rFtmParameter.min_delata_ftm;
		prRanging->ranging_ctrl.rNanFtmParam.ucFTMNum =
			prAttr->rFtmParameter.max_ftms_per_brrst;
		prRanging->ranging_ctrl.rNanFtmParam.ucFTMBandwidth =
			prAttr->rFtmParameter.ftm_format_and_bandwidth;
		u4Length += sizeof(struct _NAN_ATTR_FTM_PARAMETERS_T);
		pucPos += sizeof(struct _NAN_ATTR_FTM_PARAMETERS_T);
	}

	if (prRanging->ranging_ctrl.RangingControl &
	    NAN_RANGING_CTL_SCHEDULE_ENTRY_PRESENT) {
		rStatus = nanSchedPeerUpdateRangingScheduleList(
			prAdapter, pucDevAddr,
			(struct _NAN_SCHEDULE_ENTRY_T *)pucPos,
			NAN_ATTR_SIZE(prAttr) - u4Length);

		DBGLOG(NAN, INFO,
		       "nanSchedPeerUpdateRangingScheduleList 0x%08x\n",
		       rStatus);
	}

	return rStatus;
}

uint32_t
nanParseRangingFrame(IN struct ADAPTER *prAdapter, IN struct SW_RFB *prSwRfb,
		     IN struct _NAN_RANGING_INSTANCE_T *prRanging) {
	uint16_t u2Offset;
	uint8_t *pucNanAttr;
	uint16_t u2ContentLen;
	struct _NAN_ACTION_FRAME_T *prActionFrame = NULL;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prSwRfb == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prSwRfb is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prRanging == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRanging is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "\n");

	prActionFrame = (struct _NAN_ACTION_FRAME_T *)(prSwRfb->pvHeader);
	pucNanAttr = prActionFrame->aucInfoContent;
	u2ContentLen = prSwRfb->u2PacketLen -
		       OFFSET_OF(struct _NAN_ACTION_FRAME_T, aucInfoContent);

	NAN_ATTR_FOR_EACH(pucNanAttr, u2ContentLen, u2Offset) {

		DBGLOG(NAN, INFO, "ID %d, size %d\n", NAN_ATTR_ID(pucNanAttr),
		       NAN_ATTR_SIZE(pucNanAttr));

		switch (NAN_ATTR_ID(pucNanAttr)) {

		case NAN_ATTR_ID_FTM_RANGING_REPORT:
			rStatus = nanFtmRangeReportAttrHandler(
				prAdapter,
				(struct _NAN_ATTR_FTM_RANGE_REPORT_T *)
					pucNanAttr,
				prActionFrame->aucSrcAddr);
			break;

		case NAN_ATTR_ID_RANGING_INFORMATION:
			rStatus = nanRangingInfoAttrHandler(
				prAdapter,
				(struct _NAN_ATTR_RANGING_INFO_T *)pucNanAttr,
				prActionFrame->aucSrcAddr);
			break;

		case NAN_ATTR_ID_RANGING_SETUP:
			rStatus = nanRangingSetupAttrHandler(
				prAdapter,
				(struct _NAN_ATTR_RANGING_SETUP_T *)pucNanAttr,
				prActionFrame->aucSrcAddr);
			break;

		case NAN_ATTR_ID_NAN_AVAILABILITY:
			rStatus = nanSchedPeerUpdateAvailabilityAttr(
				prAdapter, prActionFrame->aucSrcAddr,
				(uint8_t *)pucNanAttr);

			DBGLOG(NAN, INFO,
			       "nanSchedPeerUpdateAvailabilityAttr 0x%08x\n",
			       rStatus);
			break;

		case NAN_ATTR_ID_DEVICE_CAPABILITY:
			rStatus = nanSchedPeerUpdateDevCapabilityAttr(
				prAdapter, prActionFrame->aucSrcAddr,
				(uint8_t *)pucNanAttr);
			break;

		default:
			break;
		}

		if (rStatus != WLAN_STATUS_SUCCESS)
			break;
	}

	return rStatus;
}

uint32_t
nanRangingFrameSend(struct ADAPTER *prAdapter, uint8_t *PeerAddr,
		    uint8_t ucNafSubType) {
	struct _NAN_RANGING_INSTANCE_T *prRanging;
	uint16_t u2FrameLen = 0;
	struct MSDU_INFO *prMsduInfo = NULL;
	PFN_TX_DONE_HANDLER pfTxDoneHandler = NULL;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNanSpecificBssInfo;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (PeerAddr == NULL)
		return WLAN_STATUS_INVALID_DATA;

	prRanging = nanRangingInstanceSearchByMac(prAdapter, PeerAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_INVALID_DATA;

	u2FrameLen = WLAN_MAC_MGMT_HEADER_LEN + NAN_IE_BUF_MAX_SIZE;

	/* Allocate a MSDU_INFO_T */
	prMsduInfo = cnmMgtPktAlloc(prAdapter, u2FrameLen);
	if (prMsduInfo == NULL) {
		DBGLOG(TX, ERROR,
		       "No MSDU_INFO_T for sending Ranging frame.\n");
		return WLAN_STATUS_RESOURCES;
	}

	nanRangingFrameCompose(prAdapter, prMsduInfo, prRanging, ucNafSubType);

	if (ucNafSubType == NAN_ACTION_RANGING_REQUEST)
		pfTxDoneHandler = (PFN_TX_DONE_HANDLER)nanRangingRequestTxDone;
	else if (ucNafSubType == NAN_ACTION_RANGING_RESPONSE)
		pfTxDoneHandler = (PFN_TX_DONE_HANDLER)nanRangingResponseTxDone;
	else if (ucNafSubType == NAN_ACTION_RANGING_TERMINATION)
		pfTxDoneHandler =
			(PFN_TX_DONE_HANDLER)nanRangingTerminationTxDone;
	else if (ucNafSubType == NAN_ACTION_RANGING_REPORT)
		pfTxDoneHandler = (PFN_TX_DONE_HANDLER)nanRangingReportTxDone;

	prNanSpecificBssInfo =
		nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);

	/* 4 <3> Update information of MSDU_INFO_T */
	TX_SET_MMPDU(prAdapter, prMsduInfo, prNanSpecificBssInfo->ucBssIndex,
		     STA_REC_INDEX_NOT_FOUND, WLAN_MAC_MGMT_HEADER_LEN,
		     prMsduInfo->u2FrameLength, pfTxDoneHandler,
		     MSDU_RATE_MODE_AUTO);

	prMsduInfo->ucTxToNafQueFlag = TRUE;

	nicTxSetPktRetryLimit(prMsduInfo, 3);

	nicTxSetPktLifeTime(prMsduInfo, 0);

	/* 4 <6> Enqueue the frame to send this ranging frame. */
	nicTxEnqueueMsdu(prAdapter, prMsduInfo);

	return WLAN_STATUS_SUCCESS;
}

int32_t
nanRangingRequestTx(struct ADAPTER *prAdapter,
		    struct _NAN_RANGING_INSTANCE_T *prRanging) {
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prRanging->ranging_ctrl.dialog_token < UINT8_MAX)
		prRanging->ranging_ctrl.dialog_token++;
	else
		prRanging->ranging_ctrl.dialog_token = 1; /* always non-zero */

	prRanging->ranging_ctrl.TypeStatus = NAN_RANGING_TYPE_REQUEST;

	prRanging->ranging_ctrl.ReasonCode = NAN_REASON_CODE_RESERVED;

	prRanging->ranging_ctrl.RangingControl =
		(NAN_RANGING_CTL_FTM_PARAMETERS_PRESENT |
		 NAN_RANGING_CTL_SCHEDULE_ENTRY_PRESENT);

	rStatus = nanRangingFrameSend(prAdapter,
				      prRanging->ranging_ctrl.aucPeerAddr,
				      NAN_ACTION_RANGING_REQUEST);

	DBGLOG(NAN, INFO, "nanRangingFrameSend %d\n", rStatus);

	return 0;
}

uint32_t
nanRangingRequestTxDone(IN struct ADAPTER *prAdapter,
			IN struct MSDU_INFO *prMsduInfo,
			IN enum ENUM_TX_RESULT_CODE rTxDoneStatus) {
	struct _NAN_ACTION_FRAME_T *prNAF;
	struct _NAN_RANGING_INSTANCE_T *prRanging;

	if (prMsduInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prMsduInfo is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prNAF = (struct _NAN_ACTION_FRAME_T *)prMsduInfo->prPacket;

	prRanging =
		nanRangingInstanceSearchByMac(prAdapter, prNAF->aucDestAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_FAILURE;

	if (rTxDoneStatus == TX_RESULT_SUCCESS) {
		DBGLOG(NAN, INFO, "Success\n");
	} else {
		DBGLOG(NAN, INFO, "Failed\n");
		nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_IDLE);
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanRangingRequestRx(IN struct ADAPTER *prAdapter, IN struct SW_RFB *prSwRfb) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	struct _NAN_ACTION_FRAME_T *prActionFrame = NULL;
	uint32_t u4Status = WLAN_STATUS_SUCCESS;

	DBGLOG(NAN, INFO, "\n");

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}
	prRangingInfo = &(prAdapter->rRangingInfo);

	if (prSwRfb == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prSwRfb is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prActionFrame = (struct _NAN_ACTION_FRAME_T *)prSwRfb->pvHeader;
	if (prActionFrame == NULL)
		return WLAN_STATUS_INVALID_PACKET;

	if (prActionFrame->ucOUItype == VENDOR_OUI_TYPE_NAN_SDF)
		return WLAN_STATUS_SUCCESS;

	prRanging = nanRangingInstanceSearchByMac(prAdapter,
						  prActionFrame->aucSrcAddr);
	if (prRanging == NULL) {
		if (dl_list_len(&prRangingInfo->ranging_list) >=
		    NAN_MAX_SUPPORT_RANGING_NUM) {
			DBGLOG(NAN, ERROR, "support max %d peers\n",
			       NAN_MAX_SUPPORT_RANGING_NUM);
			return WLAN_STATUS_RESOURCES;
		}

		prRanging = cnmMemAlloc(prAdapter, RAM_TYPE_BUF,
					sizeof(struct _NAN_RANGING_INSTANCE_T));

		if (prRanging == NULL) {
			DBGLOG(NAN, ERROR, "fail to create a new instance\n");
			return WLAN_STATUS_RESOURCES;
		}

		nanRangingInstanceInit(prAdapter, prRanging,
				       prActionFrame->aucSrcAddr,
				       NAN_PROTOCOL_RESPONDER);
		nanRangingInstanceAdd(prAdapter, prRanging);
	} else {
		/* Ignore before this peer's ranging is done. */
		if (prRanging->ranging_ctrl.eCurrentState > RANGING_STATE_INIT)
			return WLAN_STATUS_SUCCESS;

		nanRangingInstanceInit(prAdapter, prRanging,
				       prActionFrame->aucSrcAddr,
				       NAN_PROTOCOL_RESPONDER);
	}

	nanParseRangingFrame(prAdapter, prSwRfb, prRanging);

	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_SCHEDULE);

	u4Status = nanSchedNegoStart(
		prAdapter, prRanging->ranging_ctrl.aucPeerAddr,
		ENUM_NAN_NEGO_RANGING,
		prRanging->ranging_ctrl.ucRole == NAN_PROTOCOL_INITIATOR
			? ENUM_NAN_NEGO_ROLE_INITIATOR
			: ENUM_NAN_NEGO_ROLE_RESPONDER,
		nanRangingScheduleNegoGranted, (void *)prRanging);

	return u4Status;
}

int32_t
nanRangingResponseTx(struct ADAPTER *prAdapter,
		     struct _NAN_RANGING_INSTANCE_T *prRanging) {
	struct NanRangeResponseCtl *prResponseCtl;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prRanging == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRanging is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prResponseCtl = &prRanging->ranging_ctrl.response_ctl;

	DBGLOG(NAN, INFO, "auto response %d report required %d\n",
	       (prResponseCtl->ranging_auto_response ==
		NAN_RANGING_AUTO_RESPONSE_ENABLE)
		       ? TRUE
		       : FALSE,
	       (prResponseCtl->range_report == NAN_ENABLE_RANGE_REPORT)
		       ? TRUE
		       : FALSE);

	prRanging->ranging_ctrl.TypeStatus = NAN_RANGING_TYPE_RESPONSE;

	if (prResponseCtl->ranging_auto_response ==
	    NAN_RANGING_AUTO_RESPONSE_DISABLE) {

		if (prResponseCtl->ranging_response_code ==
		    NAN_RANGE_REQUEST_ACCEPT) {
			prRanging->ranging_ctrl.TypeStatus |=
				(NAN_RANGING_STATUS_ACCEPTED
				 << NAN_RANGING_STATUS_OFFSET);
		} else {
			prRanging->ranging_ctrl.TypeStatus |=
				(NAN_RANGING_STATUS_REJECTED
				 << NAN_RANGING_STATUS_OFFSET);
		}
	} else {

		if (prRanging->ranging_ctrl.bSchedPass) {
			prRanging->ranging_ctrl.TypeStatus |=
				(NAN_RANGING_STATUS_ACCEPTED
				 << NAN_RANGING_STATUS_OFFSET);
		} else {
			prRanging->ranging_ctrl.TypeStatus |=
				(NAN_RANGING_STATUS_REJECTED
				 << NAN_RANGING_STATUS_OFFSET);
		}
	}

	prRanging->ranging_ctrl.RangingControl =
		(NAN_RANGING_CTL_FTM_PARAMETERS_PRESENT |
		 NAN_RANGING_CTL_SCHEDULE_ENTRY_PRESENT);

	if (prResponseCtl->range_report == NAN_ENABLE_RANGE_REPORT)
		prRanging->ranging_ctrl.RangingControl |=
			NAN_RANGING_CTL_REPORT_REQUIRED;

	rStatus = nanRangingFrameSend(prAdapter,
				      prRanging->ranging_ctrl.aucPeerAddr,
				      NAN_ACTION_RANGING_RESPONSE);

	DBGLOG(NAN, INFO, "nanRangingFrameSend %d\n", rStatus);

	return 0;
}

uint32_t
nanRangingResponseTxDone(IN struct ADAPTER *prAdapter,
			 IN struct MSDU_INFO *prMsduInfo,
			 IN enum ENUM_TX_RESULT_CODE rTxDoneStatus) {
	struct _NAN_ACTION_FRAME_T *prNAF;
	struct _NAN_RANGING_INSTANCE_T *prRanging;

	if (prMsduInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prMsduInfo is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prNAF = (struct _NAN_ACTION_FRAME_T *)prMsduInfo->prPacket;

	prRanging =
		nanRangingInstanceSearchByMac(prAdapter, prNAF->aucDestAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_FAILURE;

	if (rTxDoneStatus == TX_RESULT_SUCCESS) {
		DBGLOG(NAN, INFO, "Success\n");
		if (prRanging->ranging_ctrl.bSchedPass) {
			nanRangingFtmParamCmd(prAdapter, prRanging);
			nanRangingFsmStep(prAdapter, prRanging,
					  RANGING_STATE_ACTIVE);
		}
	} else {
		DBGLOG(NAN, INFO, "Failed\n");
		if (prRanging->ranging_ctrl.bSchedPass)
			nanRangingFsmStep(prAdapter, prRanging,
					  RANGING_STATE_IDLE);
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanRangingResponseRx(IN struct ADAPTER *prAdapter, IN struct SW_RFB *prSwRfb) {
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	struct _NAN_ACTION_FRAME_T *prActionFrame = NULL;
	unsigned char bSchedPass = TRUE;
	uint32_t u4Status = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "\n");

	if (prSwRfb == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prSwRfb is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prActionFrame = (struct _NAN_ACTION_FRAME_T *)prSwRfb->pvHeader;
	if (prActionFrame == NULL)
		return WLAN_STATUS_INVALID_PACKET;

	if (prActionFrame->ucOUItype == VENDOR_OUI_TYPE_NAN_SDF)
		return WLAN_STATUS_SUCCESS;

	prRanging = nanRangingInstanceSearchByMac(prAdapter,
						  prActionFrame->aucSrcAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_FAILURE;

	if (prRanging->ranging_ctrl.eCurrentState != RANGING_STATE_REQUEST)
		return WLAN_STATUS_SUCCESS;

	nanParseRangingFrame(prAdapter, prSwRfb, prRanging);

	/* nan_report_upper_layer(RANGE_CONFIRM_INDICATON, DATA_SUCCESS); */

	if ((prRanging->ranging_ctrl.TypeStatus & NAN_RANGING_STATUS_MASK) ==
	    (NAN_RANGING_STATUS_ACCEPTED << NAN_RANGING_STATUS_OFFSET)) {

		uint32_t u4RejectCode = NAN_REASON_CODE_RESERVED;

		u4Status =
			nanSchedNegoChkRmtCrbProposal(prAdapter, &u4RejectCode);

		DBGLOG(NAN, INFO, "nanSchedNegoChkRmtCrbProposal 0x%08x\n",
		       u4Status);

		if (u4Status == WLAN_STATUS_SUCCESS) {
			bSchedPass = (u4RejectCode) ? FALSE : TRUE;
			prRanging->ranging_ctrl.ReasonCode = u4RejectCode;
		} else {
			bSchedPass = FALSE;
		}
	} else {
		bSchedPass = FALSE;
	}

	prRanging->ranging_ctrl.bSchedPass = bSchedPass;

	DBGLOG(NAN, INFO, "bSchedPass %d\n", bSchedPass);

	nanSchedNegoStop(prAdapter);

	DBGLOG(NAN, INFO, "nanSchedNegoStop\n");

	if (bSchedPass) {
		nanRangingFtmParamCmd(prAdapter, prRanging);
		nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_ACTIVE);
	} else {
		nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_IDLE);
	}

	return u4Status;
}

int32_t
nanRangingTerminationTx(struct ADAPTER *prAdapter,
			struct _NAN_RANGING_INSTANCE_T *prRanging) {
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prRanging->ranging_ctrl.dialog_token < UINT8_MAX)
		prRanging->ranging_ctrl.dialog_token++;
	else
		prRanging->ranging_ctrl.dialog_token = 1; /* always non-zero */

	prRanging->ranging_ctrl.TypeStatus = NAN_RANGING_TYPE_TERMINATION;

	prRanging->ranging_ctrl.ReasonCode = NAN_REASON_CODE_RESERVED;

	prRanging->ranging_ctrl.RangingControl = 0;

	rStatus = nanRangingFrameSend(prAdapter,
				      prRanging->ranging_ctrl.aucPeerAddr,
				      NAN_ACTION_RANGING_TERMINATION);

	DBGLOG(NAN, INFO, "nanRangingFrameSend %d\n", rStatus);

	return 0;
}

uint32_t
nanRangingTerminationTxDone(IN struct ADAPTER *prAdapter,
			    IN struct MSDU_INFO *prMsduInfo,
			    IN enum ENUM_TX_RESULT_CODE rTxDoneStatus) {
	if (rTxDoneStatus == TX_RESULT_SUCCESS)
		DBGLOG(NAN, INFO, "Success\n");
	else
		DBGLOG(NAN, INFO, "Failed\n");

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanRangingTerminationRx(IN struct ADAPTER *prAdapter,
			IN struct SW_RFB *prSwRfb) {
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	struct _NAN_ACTION_FRAME_T *prActionFrame = NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prSwRfb == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prSwRfb is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prActionFrame = (struct _NAN_ACTION_FRAME_T *)prSwRfb->pvHeader;
	if (prActionFrame == NULL)
		return WLAN_STATUS_INVALID_PACKET;

	if (prActionFrame->ucOUItype == VENDOR_OUI_TYPE_NAN_SDF)
		return WLAN_STATUS_SUCCESS;

	DBGLOG(NAN, LOUD, "\n");

	prRanging = nanRangingInstanceSearchByMac(prAdapter,
						  prActionFrame->aucSrcAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_SUCCESS;

	nanParseRangingFrame(prAdapter, prSwRfb, prRanging);

	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_IDLE);

	return WLAN_STATUS_SUCCESS;
}

int32_t
nanRangingReportTx(struct ADAPTER *prAdapter,
		   struct _NAN_RANGING_INSTANCE_T *prRanging) {
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return -1;
	}

	rStatus = nanRangingFrameSend(prAdapter,
				      prRanging->ranging_ctrl.aucPeerAddr,
				      NAN_ACTION_RANGING_REPORT);

	DBGLOG(NAN, INFO, "nanRangingFrameSend %d\n", rStatus);

	return 0;
}

uint32_t
nanRangingReportTxDone(IN struct ADAPTER *prAdapter,
		       IN struct MSDU_INFO *prMsduInfo,
		       IN enum ENUM_TX_RESULT_CODE rTxDoneStatus) {
	if (rTxDoneStatus == TX_RESULT_SUCCESS)
		DBGLOG(NAN, INFO, "Success\n");
	else
		DBGLOG(NAN, INFO, "Failed\n");

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanRangingReportRx(IN struct ADAPTER *prAdapter, IN struct SW_RFB *prSwRfb) {
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	struct _NAN_ACTION_FRAME_T *prActionFrame = NULL;
	struct _NAN_RANGING_REPORT_CMD rgrpt;
	uint32_t u4IndChk;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "\n");

	if (prSwRfb == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prSwRfb is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prActionFrame = (struct _NAN_ACTION_FRAME_T *)prSwRfb->pvHeader;
	if (prActionFrame == NULL)
		return WLAN_STATUS_INVALID_PACKET;

	if (prActionFrame->ucOUItype == VENDOR_OUI_TYPE_NAN_SDF)
		return WLAN_STATUS_SUCCESS;

	prRanging = nanRangingInstanceSearchByMac(prAdapter,
						  prActionFrame->aucSrcAddr);
	if (prRanging == NULL)
		return -1;

	nanParseRangingFrame(prAdapter, prSwRfb, prRanging);

	/* Update distance by new measurement */
	if (nanRangingUpdateDistance(prAdapter, prRanging) == FALSE)
		return WLAN_STATUS_SUCCESS;

	/* Reset timer to track Ranging seesion */
	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_ACTIVE);

	/* Check geofencing */
	u4IndChk = nanRangingGeofencingCheck(prAdapter, prRanging);
	if (!u4IndChk)
		return WLAN_STATUS_SUCCESS;

	if (prRanging->ranging_ctrl.ucInvoker == NAN_RANGING_APPLICATION) {

		nanRangingResult(prAdapter, prRanging, u4IndChk);

	} else {

		kalMemZero(&rgrpt, sizeof(struct _NAN_RANGING_REPORT_CMD));

		rgrpt.ucStatus = WLAN_STATUS_SUCCESS;
		rgrpt.ranging_id = prRanging->ranging_ctrl.u2RangingId;
		rgrpt.ranging_event_type = u4IndChk;
		rgrpt.range_measurement_cm =
			prRanging->ranging_ctrl.range_measurement_cm;

		COPY_MAC_ADDR(rgrpt.range_req_intf_addr,
			      prRanging->ranging_ctrl.aucPeerAddr);

		nanRangingReportDiscCmd(prAdapter, &rgrpt);
	}

	return 0;
}

/************************************************
 *   NAN Ranging State Machine
 ************************************************
 */
void
nanRangingFsmStep(struct ADAPTER *prAdapter,
		  struct _NAN_RANGING_INSTANCE_T *prRanging,
		  enum _ENUM_RANGING_STATE_T eNextState) {
	unsigned char fgIsTransition = (unsigned char)FALSE;
	struct _NAN_RANGING_CTRL_T *prRangingCtrl;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	prRangingCtrl = &prRanging->ranging_ctrl;

	do {

		if (prRangingCtrl->eCurrentState != eNextState) {
			DBGLOG(NAN, STATE, "TRANSITION: [%s] -> [%s]\n",
			       apucDebugRangingState[prRangingCtrl
							     ->eCurrentState],
			       apucDebugRangingState[eNextState]);
		}

		prRangingCtrl->eCurrentState = eNextState;
		fgIsTransition = (unsigned char)FALSE;

		switch (prRangingCtrl->eCurrentState) {

		case RANGING_STATE_IDLE:
			cnmTimerStopTimer(prAdapter,
					  &(prRanging->ranging_ctrl
						    .rRangingSessionTimer));
			nanSchedDropResources(
				prAdapter, prRanging->ranging_ctrl.aucPeerAddr,
				ENUM_NAN_NEGO_RANGING);
			break;

		case RANGING_STATE_INIT:
			/* Initialized but do nothing */
			break;

		case RANGING_STATE_SCHEDULE:
			/* Busy on Ranging schedule */
			break;

		case RANGING_STATE_REQUEST:
			nanRangingRequestTx(prAdapter, prRanging);
			break;

		case RANGING_STATE_REQUEST_IND:
			/* Wait for Response from Application */
			break;

		case RANGING_STATE_RESPONSE:
			nanRangingResponseTx(prAdapter, prRanging);
			break;

		case RANGING_STATE_ACTIVE:
			cnmTimerStopTimer(prAdapter,
					  &(prRanging->ranging_ctrl
						    .rRangingSessionTimer));
			cnmTimerStartTimer(
				prAdapter,
				&(prRanging->ranging_ctrl.rRangingSessionTimer),
				NAN_RANGING_SESSION_TIMEOUT);
			break;

		case RANGING_STATE_REPORT:
			nanRangingReportTx(prAdapter, prRanging);
			break;

		case RANGING_STATE_TERMINATE:
			nanRangingTerminationTx(prAdapter, prRanging);
			break;

		default:
			break;
		}
	} while (fgIsTransition);
}

void
nanRangingSessionTimeout(struct ADAPTER *prAdapter, unsigned long ulParam) {
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	prRanging = (struct _NAN_RANGING_INSTANCE_T *)ulParam;

	if (prRanging != NULL) {

		nanRangingFsmStep(prAdapter, prRanging,
				  RANGING_STATE_TERMINATE);

		nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_IDLE);
	}
}

/************************************************
 *   Interface for FTM
 ************************************************
 */
void
nanRangingFtmParamCmd(IN struct ADAPTER *prAdapter,
		      struct _NAN_RANGING_INSTANCE_T *prRanging) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_FTM_PARAM_CMD *prFtmParam;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	DBGLOG(NAN, INFO, "\n");

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_FTM_PARAM_CMD);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (prCmdBuffer == NULL) {
		DBGLOG(CNM, ERROR, "Memory allocation fail\n");
		return;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(NAN_CMD_FTM_PARAM,
				      sizeof(struct _NAN_FTM_PARAM_CMD),
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

	prFtmParam = (struct _NAN_FTM_PARAM_CMD *)prTlvElement->aucbody;

	prFtmParam->ucRole = prRanging->ranging_ctrl.ucRole;
	prFtmParam->ucInvoker = prRanging->ranging_ctrl.ucInvoker;
	COPY_MAC_ADDR(prFtmParam->aucPeerAddr,
		      prRanging->ranging_ctrl.aucPeerAddr);
	kalMemCopy(&prFtmParam->rNanFtmParam,
		   &prRanging->ranging_ctrl.rNanFtmParam,
		   sizeof(struct _NAN_FTM_PARAM_T));

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, NULL, u4CmdBufferLen,
				      (uint8_t *)prCmdBuffer, NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);
}

unsigned char
nanRangingUpdateDistance(struct ADAPTER *prAdapter,
			 struct _NAN_RANGING_INSTANCE_T *prRanging) {
	struct _NAN_RANGING_CTRL_T *pCtrl;
	uint32_t u4IngressTh;
	uint32_t u4EgressTh;

	pCtrl = (struct _NAN_RANGING_CTRL_T *)&prRanging->ranging_ctrl;
	u4IngressTh = pCtrl->ranging_cfg.distance_ingress_cm;
	u4EgressTh = pCtrl->ranging_cfg.distance_egress_cm;

	if ((pCtrl->rNanFtmReport.ucRangeEntryCnt == 0) ||
	    (pCtrl->rNanFtmReport.arRangeEntry[0].u4Range == 0)) {
		DBGLOG(NAN, INFO, "No valid distance to update\n");
		return FALSE;
	}

	DBGLOG(NAN, INFO, "Report %u (1/4096 m), Range %u cm\n",
	       pCtrl->rNanFtmReport.arRangeEntry[0].u4Range,
	       FTM_FMT_TO_RANGE_CM(
		       pCtrl->rNanFtmReport.arRangeEntry[0].u4Range));

	/* Support only one now, get it directly */
	pCtrl->range_measurement_cm = FTM_FMT_TO_RANGE_CM(
		pCtrl->rNanFtmReport.arRangeEntry[0].u4Range);

	DBGLOG(NAN, INFO, "Ingress Th %u cm, Egress Th %u cm\n", u4IngressTh,
	       u4EgressTh);

	/* Ingress geofence */
	pCtrl->bPreInside = pCtrl->bCurInside;

	if (pCtrl->range_measurement_cm <= u4IngressTh)
		pCtrl->bCurInside = TRUE;
	else
		pCtrl->bCurInside = FALSE;

	/* Egress geofence */
	pCtrl->bPreOutside = pCtrl->bCurOutside;

	if (pCtrl->range_measurement_cm >= u4EgressTh)
		pCtrl->bCurOutside = TRUE;
	else
		pCtrl->bCurOutside = FALSE;

	return TRUE;
}

uint32_t
nanRangingGeofencingCheck(struct ADAPTER *prAdapter,
			  struct _NAN_RANGING_INSTANCE_T *prRanging) {
	struct _NAN_RANGING_CTRL_T *pCtrl;
	uint32_t u4IndConfig;
	uint32_t u4IndStatus = 0;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return u4IndStatus;
	}

	if (prRanging == NULL)
		return u4IndStatus;

	pCtrl = (struct _NAN_RANGING_CTRL_T *)&prRanging->ranging_ctrl;
	u4IndConfig = pCtrl->ranging_cfg.config_ranging_indications;

	if (u4IndConfig & NAN_RANGING_INDICATE_CONTINUOUS_MASK) {

		u4IndStatus |= NAN_RANGING_INDICATE_CONTINUOUS_MASK;

		DBGLOG(NAN, INFO, "Continuous Indication! Range %u cm\n",
		       prRanging->ranging_ctrl.range_measurement_cm);
	}

	if (u4IndConfig & NAN_RANGING_INDICATE_INGRESS_MET_MASK) {

		if (!pCtrl->bPreInside && pCtrl->bCurInside) {
			u4IndStatus |= NAN_RANGING_INDICATE_INGRESS_MET_MASK;

			DBGLOG(NAN, INFO, "Ingress Indication! Range %u cm\n",
			       prRanging->ranging_ctrl.range_measurement_cm);
		}
	}

	if (u4IndConfig & NAN_RANGING_INDICATE_EGRESS_MET_MASK) {

		if (!pCtrl->bPreOutside && pCtrl->bCurOutside) {
			u4IndStatus |= NAN_RANGING_INDICATE_EGRESS_MET_MASK;

			DBGLOG(NAN, INFO, "Egress Indication! Range %u cm\n",
			       prRanging->ranging_ctrl.range_measurement_cm);
		}
	}

	return u4IndStatus;
}

void
nanRangingFtmDoneEvt(IN struct ADAPTER *prAdapter, IN uint8_t *pcuEvtBuf) {
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	struct _NAN_FTM_DONE_EVENT *prEvent;
	struct _NAN_RANGING_REPORT_CMD rgrpt;
	unsigned char bReportEn = FALSE;
	uint32_t u4IndChk;
	uint8_t ucRangeEntryCnt;
	uint8_t ucErrorEntryCnt;
	uint32_t u4Idx;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	if (pcuEvtBuf == NULL) {
		DBGLOG(NAN, ERROR, "[%s] pcuEvtBuf is NULL\n", __func__);
		return;
	}

	DBGLOG(NAN, INFO, "\n");

	prEvent = (struct _NAN_FTM_DONE_EVENT *)pcuEvtBuf;

	prRanging =
		nanRangingInstanceSearchByMac(prAdapter, prEvent->aucPeerAddr);
	if (prRanging == NULL)
		return;

	ucRangeEntryCnt = prEvent->rNanFtmReport.ucRangeEntryCnt;
	if (ucRangeEntryCnt >= NAN_FTM_REPORT_OK_MAX_NUM)
		ucRangeEntryCnt = NAN_FTM_REPORT_OK_MAX_NUM;
	prRanging->ranging_ctrl.rNanFtmReport.ucRangeEntryCnt = ucRangeEntryCnt;

	for (u4Idx = 0; u4Idx < ucRangeEntryCnt; u4Idx++) {
		kalMemCopy(&prRanging->ranging_ctrl.rNanFtmReport
				    .arRangeEntry[u4Idx],
			   &prEvent->rNanFtmReport.arRangeEntry[u4Idx],
			   sizeof(struct _FTM_REPORT_RANGE_ENTRY_T));
	}

	ucErrorEntryCnt = prEvent->rNanFtmReport.ucErrorEntryCnt;
	if (ucErrorEntryCnt >= NAN_FTM_REPORT_NG_MAX_NUM)
		ucErrorEntryCnt = NAN_FTM_REPORT_NG_MAX_NUM;
	prRanging->ranging_ctrl.rNanFtmReport.ucErrorEntryCnt = ucErrorEntryCnt;

	for (u4Idx = 0; u4Idx < ucErrorEntryCnt; u4Idx++) {
		kalMemCopy(&prRanging->ranging_ctrl.rNanFtmReport
				    .arErrorEntry[u4Idx],
			   &prEvent->rNanFtmReport.arErrorEntry[u4Idx],
			   sizeof(struct _FTM_REPORT_ERROR_ENTRY_T));
	}

	DBGLOG(NAN, INFO, "ucRangeEntryCnt (%u), ucErrorEntryCnt (%u)\n",
	       ucRangeEntryCnt, ucErrorEntryCnt);

	/* Send Ranging report if need */
	if (prRanging->ranging_ctrl.RangingControl &
	    NAN_RANGING_CTL_REPORT_REQUIRED)
		bReportEn = TRUE;

	DBGLOG(NAN, INFO, "report required %d\n", bReportEn);

	if (bReportEn)
		nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_REPORT);

	/* Update distance by new measurement */
	if (nanRangingUpdateDistance(prAdapter, prRanging) == FALSE)
		return;

	/* Reset timer to track Ranging seesion */
	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_ACTIVE);

	/* Check geofencing */
	u4IndChk = nanRangingGeofencingCheck(prAdapter, prRanging);
	if (!u4IndChk)
		return;

	if (prRanging->ranging_ctrl.ucInvoker == NAN_RANGING_APPLICATION) {

		nanRangingResult(prAdapter, prRanging, u4IndChk);

	} else {

		kalMemZero(&rgrpt, sizeof(struct _NAN_RANGING_REPORT_CMD));

		rgrpt.ucStatus = WLAN_STATUS_SUCCESS;
		rgrpt.ranging_id = prRanging->ranging_ctrl.u2RangingId;
		rgrpt.ranging_event_type = u4IndChk;
		rgrpt.range_measurement_cm =
			prRanging->ranging_ctrl.range_measurement_cm;

		COPY_MAC_ADDR(rgrpt.range_req_intf_addr,
			      prRanging->ranging_ctrl.aucPeerAddr);

		nanRangingReportDiscCmd(prAdapter, &rgrpt);
	}
}

void
nanRangingReportDiscCmd(IN struct ADAPTER *prAdapter,
			struct _NAN_RANGING_REPORT_CMD *msg) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_RANGING_REPORT_CMD *prReport;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	DBGLOG(NAN, INFO, "\n");

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_RANGING_REPORT_CMD);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (prCmdBuffer == NULL) {
		DBGLOG(CNM, ERROR, "Memory allocation fail\n");
		return;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(NAN_CMD_RANGING_REPORT_DISC,
				      sizeof(struct _NAN_RANGING_REPORT_CMD),
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

	prReport = (struct _NAN_RANGING_REPORT_CMD *)prTlvElement->aucbody;
	kalMemCopy(prReport, msg, sizeof(struct _NAN_RANGING_REPORT_CMD));

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, NULL, u4CmdBufferLen,
				      (uint8_t *)prCmdBuffer, NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);
}

/************************************************
 *   Interface for NAN Discovery Engine
 ************************************************
 */
uint32_t
nanRangingInvokedByDisc(struct ADAPTER *prAdapter, uint16_t *pu2Id,
			struct NanRangeRequest *msg) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	uint32_t u4Status = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (msg == NULL) {
		DBGLOG(NAN, ERROR, "[%s] msg is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "\n");

	prRangingInfo = &(prAdapter->rRangingInfo);
	if (prRangingInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRangingInfo is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prRanging = nanRangingInstanceSearchByMac(prAdapter, msg->peer_addr);
	if (prRanging == NULL) {
		if (dl_list_len(&prRangingInfo->ranging_list) >=
		    NAN_MAX_SUPPORT_RANGING_NUM) {
			DBGLOG(NAN, ERROR, "support max %d peers\n",
			       NAN_MAX_SUPPORT_RANGING_NUM);
			return WLAN_STATUS_RESOURCES;
		}

		prRanging = cnmMemAlloc(prAdapter, RAM_TYPE_BUF,
					sizeof(struct _NAN_RANGING_INSTANCE_T));

		if (prRanging == NULL) {
			DBGLOG(NAN, ERROR, "fail to create a new instance\n");
			return WLAN_STATUS_RESOURCES;
		}

		nanRangingInstanceInit(prAdapter, prRanging, msg->peer_addr,
				       NAN_PROTOCOL_INITIATOR);
		nanRangingInstanceAdd(prAdapter, prRanging);
	} else {
		/* Ignore before this peer's ranging is done. */
		if (prRanging->ranging_ctrl.eCurrentState > RANGING_STATE_INIT)
			return WLAN_STATUS_SUCCESS;

		nanRangingInstanceInit(prAdapter, prRanging, msg->peer_addr,
				       NAN_PROTOCOL_INITIATOR);
	}

	/* update from msg */
	prRanging->ranging_ctrl.ucInvoker = NAN_RANGING_DISCOVERY;
	kalMemCopy(&prRanging->ranging_ctrl.ranging_cfg, &msg->ranging_cfg,
		   sizeof(struct NanRangingCfg));

	if (pu2Id)
		*pu2Id = prRanging->ranging_ctrl.u2RangingId;

	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_SCHEDULE);

	u4Status = nanSchedNegoStart(
		prAdapter, prRanging->ranging_ctrl.aucPeerAddr,
		ENUM_NAN_NEGO_RANGING,
		prRanging->ranging_ctrl.ucRole == NAN_PROTOCOL_INITIATOR
			? ENUM_NAN_NEGO_ROLE_INITIATOR
			: ENUM_NAN_NEGO_ROLE_RESPONDER,
		nanRangingScheduleNegoGranted, (void *)prRanging);

	return u4Status;
}

void
nanRangingInvokedByDiscEvt(IN struct ADAPTER *prAdapter,
		IN uint8_t *pcuEvtBuf) {
	struct _NAN_RANGING_BY_DISC_EVENT *prEvent;
	struct _NAN_RANGING_REPORT_CMD rgrpt;
	uint16_t rgId = 0;
	uint32_t rStatus;

	DBGLOG(NAN, INFO, "\n");

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	if (pcuEvtBuf == NULL) {
		DBGLOG(NAN, ERROR, "[%s] pcuEvtBuf is NULL\n", __func__);
		return;
	}

	prEvent = (struct _NAN_RANGING_BY_DISC_EVENT *)pcuEvtBuf;

	rStatus = nanRangingInvokedByDisc(prAdapter, &rgId, &prEvent->rReq);

	if (rStatus != WLAN_STATUS_SUCCESS) {

		kalMemZero(&rgrpt, sizeof(struct _NAN_RANGING_REPORT_CMD));

		rgrpt.ucStatus = rStatus;

		COPY_MAC_ADDR(rgrpt.range_req_intf_addr,
			      prEvent->rReq.peer_addr);

		nanRangingReportDiscCmd(prAdapter, &rgrpt);
	}
}

/************************************************
 *   Interface for Application
 ************************************************
 */
uint32_t
nanRangingRequest(struct ADAPTER *prAdapter, uint16_t *pu2Id,
		  struct NanRangeRequest *msg) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	uint32_t u4Status = WLAN_STATUS_SUCCESS;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (msg == NULL) {
		DBGLOG(NAN, ERROR, "[%s] msg is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "\n");

	prRangingInfo = &(prAdapter->rRangingInfo);
	if (prRangingInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRangingInfo is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	prRanging = nanRangingInstanceSearchByMac(prAdapter, msg->peer_addr);
	if (prRanging == NULL) {
		if (dl_list_len(&prRangingInfo->ranging_list) >=
		    NAN_MAX_SUPPORT_RANGING_NUM) {
			DBGLOG(NAN, ERROR, "support max %d peers\n",
			       NAN_MAX_SUPPORT_RANGING_NUM);
			return WLAN_STATUS_RESOURCES;
		}

		prRanging = cnmMemAlloc(prAdapter, RAM_TYPE_BUF,
					sizeof(struct _NAN_RANGING_INSTANCE_T));

		if (prRanging == NULL) {
			DBGLOG(NAN, ERROR, "fail to create a new instance\n");
			return WLAN_STATUS_RESOURCES;
		}

		nanRangingInstanceInit(prAdapter, prRanging, msg->peer_addr,
				       NAN_PROTOCOL_INITIATOR);
		nanRangingInstanceAdd(prAdapter, prRanging);
	} else {
		/* Ignore before this peer's ranging is done. */
		if (prRanging->ranging_ctrl.eCurrentState > RANGING_STATE_INIT)
			return WLAN_STATUS_SUCCESS;

		nanRangingInstanceInit(prAdapter, prRanging, msg->peer_addr,
				       NAN_PROTOCOL_INITIATOR);
	}

	/* update from msg */
	prRanging->ranging_ctrl.ucInvoker = NAN_RANGING_APPLICATION;
	kalMemCopy(&prRanging->ranging_ctrl.ranging_cfg, &msg->ranging_cfg,
		   sizeof(struct NanRangingCfg));

	if (pu2Id)
		*pu2Id = prRanging->ranging_ctrl.u2RangingId;

	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_SCHEDULE);

	u4Status = nanSchedNegoStart(
		prAdapter, prRanging->ranging_ctrl.aucPeerAddr,
		ENUM_NAN_NEGO_RANGING,
		prRanging->ranging_ctrl.ucRole == NAN_PROTOCOL_INITIATOR
			? ENUM_NAN_NEGO_ROLE_INITIATOR
			: ENUM_NAN_NEGO_ROLE_RESPONDER,
		nanRangingScheduleNegoGranted, (void *)prRanging);

	return u4Status;
}

int32_t
nanRangingCancel(struct ADAPTER *prAdapter, struct NanRangeCancelRequest *msg) {
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return -1;
	}

	DBGLOG(NAN, INFO, "\n");

	prRanging = nanRangingInstanceSearchByMac(prAdapter, msg->peer_addr);
	if (prRanging == NULL)
		return -1;

	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_TERMINATE);

	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_IDLE);

	return 0;
}

uint32_t
nanRangingResponse(struct ADAPTER *prAdapter, struct NanRangeResponse *msg) {
	struct _NAN_RANGING_INFO_T *prRangingInfo;
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	struct NanRangeResponseCtl *prResponseCtl;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "\n");

	prRangingInfo = &(prAdapter->rRangingInfo);

	/* search peer's instance */
	prRanging = nanRangingInstanceSearchById(prAdapter, msg->range_id);
	if (prRanging == NULL) {
		if (!msg->range_id) { /* change auto response only */

			kalMemCopy(&prRangingInfo->ranging_cfg_def,
				   &msg->ranging_cfg,
				   sizeof(struct NanRangingCfg));
			kalMemCopy(&prRangingInfo->response_ctl_def,
				   &msg->response_ctl,
				   sizeof(struct NanRangeResponseCtl));

			return WLAN_STATUS_SUCCESS;
		} else {
			return WLAN_STATUS_INVALID_DATA;
		}
	}

	/* update from msg */
	kalMemCopy(&prRanging->ranging_ctrl.ranging_cfg, &msg->ranging_cfg,
		   sizeof(struct NanRangingCfg));
	prResponseCtl = &prRanging->ranging_ctrl.response_ctl;
	kalMemCopy(prResponseCtl, &msg->response_ctl,
		   sizeof(struct NanRangeResponseCtl));

	DBGLOG(NAN, INFO, "auto response %d report required %d\n",
	       (prResponseCtl->ranging_auto_response ==
		NAN_RANGING_AUTO_RESPONSE_ENABLE)
		       ? TRUE
		       : FALSE,
	       (prResponseCtl->range_report == NAN_ENABLE_RANGE_REPORT)
		       ? TRUE
		       : FALSE);

	if (prRanging->ranging_ctrl.eCurrentState ==
	    RANGING_STATE_REQUEST_IND) {

		prRanging->ranging_ctrl.ReasonCode =
			NAN_REASON_CODE_UNSPECIFIED;

		nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_RESPONSE);

		nanSchedNegoStop(prAdapter);

		DBGLOG(NAN, INFO, "nanSchedNegoStop\n");

		if (prResponseCtl->ranging_response_code !=
		    NAN_RANGE_REQUEST_ACCEPT)
			nanRangingFsmStep(prAdapter, prRanging,
					  RANGING_STATE_IDLE);
	}

	return WLAN_STATUS_SUCCESS;
}

void
nanRangingRequestIndication(struct ADAPTER *prAdapter,
			    struct _NAN_RANGING_INSTANCE_T *prRanging) {
	struct NanRangeRequestInd event;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	DBGLOG(NAN, INFO, "\n");

	event.eventID = ENUM_NAN_RG_INDICATION;
	event.publish_id = prRanging->ranging_ctrl.u2RangingId;

	COPY_MAC_ADDR(event.range_req_intf_addr,
		      prRanging->ranging_ctrl.aucPeerAddr);

	event.request_cfg.range_report = NAN_DISABLE_RANGE_REPORT;

	event.request_cfg.rangereq_ftm_cfg.max_burst_duration =
		prRanging->ranging_ctrl.rNanFtmParam.uc2BurstTimeout;
	event.request_cfg.rangereq_ftm_cfg.min_delta_ftm =
		prRanging->ranging_ctrl.rNanFtmParam.ucMinDeltaIn100US;
	event.request_cfg.rangereq_ftm_cfg.max_ftms_per_burst =
		prRanging->ranging_ctrl.rNanFtmParam.ucFTMNum;
	event.request_cfg.rangereq_ftm_cfg.ftm_format_and_bandwidth =
		prRanging->ranging_ctrl.rNanFtmParam.ucFTMBandwidth;

	kalIndicateNetlink2User(prAdapter->prGlueInfo, &event,
				sizeof(struct NanRangeRequestInd));
}

void
nanRangingResult(struct ADAPTER *prAdapter,
		 struct _NAN_RANGING_INSTANCE_T *prRanging, uint32_t u4IndChk) {
	struct NanRangeReportInd event;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	if (prRanging == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prRanging is NULL\n", __func__);
		return;
	}

	DBGLOG(NAN, INFO,
	       "Peer Addr: " MACSTR ", Range: %d cm, Indication: 0x%x\n",
	       MAC2STR(prRanging->ranging_ctrl.aucPeerAddr),
	       prRanging->ranging_ctrl.range_measurement_cm, u4IndChk);

	event.eventID = ENUM_NAN_RG_RESULT;
	event.publish_id = prRanging->ranging_ctrl.u2RangingId;

	COPY_MAC_ADDR(event.range_req_intf_addr,
		      prRanging->ranging_ctrl.aucPeerAddr);

	event.range_measurement_cm =
		prRanging->ranging_ctrl.range_measurement_cm;

	event.ranging_event_type = u4IndChk;

	kalIndicateNetlink2User(prAdapter->prGlueInfo, &event,
				sizeof(struct NanRangeReportInd));
}

/************************************************
 *   Interface for NAN Scheduler
 ************************************************
 */
void
nanRangingScheduleNegoGranted(struct ADAPTER *prAdapter, uint8_t *pu1DevAddr,
			      enum _ENUM_NAN_NEGO_TYPE_T eType,
			      enum _ENUM_NAN_NEGO_ROLE_T eRole, void *pvToken) {
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;
	uint32_t u4Status = WLAN_STATUS_SUCCESS;
	unsigned char bSchedPass = TRUE;
	uint32_t u4RejectCode = NAN_REASON_CODE_RESERVED;
	enum NanRangingAutoResponse ucAutoRsp;

	prRanging = (struct _NAN_RANGING_INSTANCE_T *)pvToken;
	if (prRanging == NULL)
		return;

	if (prRanging->ranging_ctrl.ucRole == NAN_PROTOCOL_INITIATOR) {

		u4Status = nanSchedNegoGenLocalCrbProposal(prAdapter);

		DBGLOG(NAN, INFO, "nanSchedNegoGenLocalCrbProposal 0x%08x\n",
		       u4Status);

		bSchedPass = (u4Status == WLAN_STATUS_SUCCESS) ? TRUE : FALSE;
		prRanging->ranging_ctrl.ReasonCode =
			NAN_REASON_CODE_RESOURCE_LIMITATION;
		prRanging->ranging_ctrl.bSchedPass = bSchedPass;

		DBGLOG(NAN, INFO, "bSchedPass %d\n", bSchedPass);

		if (bSchedPass)
			nanRangingFsmStep(prAdapter, prRanging,
					  RANGING_STATE_REQUEST);
		else
			nanRangingFsmStep(prAdapter, prRanging,
					  RANGING_STATE_IDLE);

	} else { /* NAN_PROTOCOL_RESPONDER */

		u4Status =
			nanSchedNegoChkRmtCrbProposal(prAdapter, &u4RejectCode);

		DBGLOG(NAN, INFO, "nanSchedNegoChkRmtCrbProposal 0x%08x\n",
		       u4Status);

		if (u4Status == WLAN_STATUS_SUCCESS) {
			bSchedPass = (u4RejectCode) ? FALSE : TRUE;
			prRanging->ranging_ctrl.ReasonCode = u4RejectCode;
		} else {
			bSchedPass = FALSE;
		}

		prRanging->ranging_ctrl.bSchedPass = bSchedPass;

		ucAutoRsp = prRanging->ranging_ctrl.response_ctl
				    .ranging_auto_response;

		DBGLOG(NAN, INFO, "bSchedPass %d ucAutoRsp %d\n", bSchedPass,
		       ucAutoRsp);

		if (bSchedPass &&
		    (ucAutoRsp == NAN_RANGING_AUTO_RESPONSE_DISABLE)) {

			nanRangingRequestIndication(prAdapter, prRanging);
			nanRangingFsmStep(prAdapter, prRanging,
					  RANGING_STATE_REQUEST_IND);

		} else {

			nanRangingFsmStep(prAdapter, prRanging,
					  RANGING_STATE_RESPONSE);

			nanSchedNegoStop(prAdapter);

			DBGLOG(NAN, INFO, "nanSchedNegoStop\n");

			if (bSchedPass == FALSE)
				nanRangingFsmStep(prAdapter, prRanging,
						  RANGING_STATE_IDLE);
		}
	}
}

uint32_t
nanRangingScheduleViolation(struct ADAPTER *prAdapter, uint8_t *pu1DevAddr) {
	struct _NAN_RANGING_INSTANCE_T *prRanging;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "\n");

	if (pu1DevAddr == NULL)
		return WLAN_STATUS_INVALID_DATA;

	prRanging = nanRangingInstanceSearchByMac(prAdapter, pu1DevAddr);
	if (prRanging == NULL)
		return WLAN_STATUS_INVALID_DATA;

	nanRangingFsmStep(prAdapter, prRanging, RANGING_STATE_IDLE);

	return WLAN_STATUS_SUCCESS;
}

void
nanRangingListPrint(struct ADAPTER *prAdapter) {
	struct dl_list *ranging_list;
	struct _NAN_RANGING_INSTANCE_T *prRanging = NULL;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return;
	}

	DBGLOG(NAN, INFO, "u2RangingCnt %d\n",
	       prAdapter->rRangingInfo.u2RangingCnt);

	ranging_list = &prAdapter->rRangingInfo.ranging_list;

	dl_list_for_each(prRanging, ranging_list,
			 struct _NAN_RANGING_INSTANCE_T, list) {

		if (prRanging) {
			DBGLOG(NAN, INFO, "[%d] [" MACSTR "] [%s] [%s]\n",
			       prRanging->ranging_ctrl.u2RangingId,
			       MAC2STR(prRanging->ranging_ctrl.aucPeerAddr),
			       prRanging->ranging_ctrl.ucRole ==
					       NAN_PROTOCOL_INITIATOR
				       ? "Initiator"
				       : "Responder",
			       apucDebugRangingState[prRanging->ranging_ctrl
							     .eCurrentState]);
		}
	}
}
