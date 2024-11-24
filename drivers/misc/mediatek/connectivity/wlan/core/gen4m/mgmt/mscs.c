/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*
 * Id: @(#) mscs.c@@
 */

/*! \file   mscs.c
 *    \brief  Functions for processing MSCS related elements and frames.
 *
 *    Functions for processing MSCS related elements and frames.
 */


/*******************************************************************************
 *                        C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                   E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"

#if CFG_MSCS_SUPPORT

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                           P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

struct MSCS_FIVE_TUPLE_T *mscsSearchFiveTuple(IN struct ADAPTER *prAdapter,
	IN uint8_t *prTargetTuple)
{
	struct STA_RECORD *prStaRec = NULL;
	struct LINK *prMonitorList;
	struct MSCS_FIVE_TUPLE_T *prFiveTuple;

	prStaRec = aisGetStaRecOfAP(prAdapter, AIS_DEFAULT_INDEX);
	if (!prStaRec)
		return NULL;

	prMonitorList = &prStaRec->rMscsMonitorList;
	DBGLOG(TX, TRACE, "[F] monitor list(%p) len:%d\n",
			prMonitorList, prMonitorList->u4NumElem);
	DBGLOG_MEM8(TX, LOUD, prTargetTuple, LEN_OF_FIVE_TUPLE);

	LINK_FOR_EACH_ENTRY(prFiveTuple, prMonitorList, rLinkEntry,
			    struct MSCS_FIVE_TUPLE_T) {
		if (!prFiveTuple)
			break;

		if (!kalMemCmp(
				&prFiveTuple->u4SrcIp, prTargetTuple,
				LEN_OF_FIVE_TUPLE)) {
			DBGLOG(TX, LOUD,
			       "Current 5-tuple already in monitoring list\n");
			return prFiveTuple;
		}
	}
	return NULL;
}

void mscsAddFiveTuple(IN struct ADAPTER *prAdapter,
	IN uint8_t *prTargetTuple)
{
	struct STA_RECORD *prStaRec = NULL;
	struct LINK *prMonitorList;
	struct MSCS_FIVE_TUPLE_T *prFiveTuple = NULL;
	struct MSCS_FIVE_TUPLE_T *prEntry;

	prStaRec = aisGetStaRecOfAP(prAdapter, AIS_DEFAULT_INDEX);
	if (!prStaRec)
		return;

	prMonitorList = &prStaRec->rMscsMonitorList;
	DBGLOG(TX, TRACE, "[F] monitor list(%p) len:%d\n",
			prMonitorList, prMonitorList->u4NumElem);

	LINK_FOR_EACH_ENTRY(prFiveTuple, prMonitorList, rLinkEntry,
			    struct MSCS_FIVE_TUPLE_T) {
		if (!prFiveTuple) {
			DBGLOG(TX, TRACE, "[F] return because of no entry\n");
			break;
		}

		if (!kalMemCmp(&prFiveTuple->u4SrcIp, prTargetTuple,
			LEN_OF_FIVE_TUPLE)) {
			DBGLOG(TX, TRACE,
			       "Current 5-tuple already in monitoring list\n");
			return;
		}
	}

	DBGLOG(TX, TRACE, "No match, add into monitor list\n");

	if (prMonitorList->u4NumElem >= MAX_MONITOR_FIVE_TUPLE) {
		LINK_REMOVE_HEAD(prMonitorList, prEntry,
			struct MSCS_FIVE_TUPLE_T *);
		if (prEntry)
			kalMemFree(prEntry, VIR_MEM_TYPE,
				sizeof(struct MSCS_FIVE_TUPLE_T));
	}

	prEntry = kalMemAlloc(sizeof(struct MSCS_FIVE_TUPLE_T), VIR_MEM_TYPE);
	if (!prEntry) {
		DBGLOG(TX, WARN, "No memory for 5 tuple\n");
		return;
	}
	kalMemCopy(&prEntry->u4SrcIp, prTargetTuple, LEN_OF_FIVE_TUPLE);
	LINK_INSERT_TAIL(prMonitorList, &prEntry->rLinkEntry);
}

void mscsDelFiveTuple(IN struct ADAPTER *prAdapter,
		IN struct SW_RFB *prSwRfb, IN uint8_t *prTargetTuple)
{
	struct STA_RECORD *prStaRec;
	struct LINK *prMonitorList;
	struct MSCS_FIVE_TUPLE_T *prEntry;

	prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);
	if (!prStaRec)
		return;

	prMonitorList = &prStaRec->rMscsMonitorList;

	DBGLOG(TX, TRACE, "[F] monitor list(%p) len:%d\n",
			prMonitorList, prMonitorList->u4NumElem);

	DBGLOG(TX, TRACE, "[F] del 5-tuple\n");
	DBGLOG_MEM8(TX, TRACE, prTargetTuple, 12);

	LINK_FOR_EACH_ENTRY(prEntry, prMonitorList, rLinkEntry,
			    struct MSCS_FIVE_TUPLE_T) {
		if (!prEntry)
			break;

		if (!kalMemCmp(&prEntry->u4SrcIp, prTargetTuple, 12)) {
			LINK_REMOVE_KNOWN_ENTRY(prMonitorList, prEntry);
			kalMemFree(prEntry, VIR_MEM_TYPE,
				sizeof(struct MSCS_FIVE_TUPLE_T));
			DBGLOG(TX, TRACE, "[F] del a tuple!!!!\n");
			return;
		}
	}
	DBGLOG(TX, TRACE, "[F] del 5-tuple\n");
	DBGLOG_MEM8(TX, TRACE, prTargetTuple, 12);
}

void mscsFlushFiveTuple(IN struct ADAPTER *prAdapter,
	IN struct STA_RECORD *prStaRec)
{
	struct LINK *prMonitorList = &prStaRec->rMscsMonitorList;
	struct MSCS_FIVE_TUPLE_T *prEntry;

	DBGLOG(TX, INFO, "Flush all five tuples(%d)\n",
		prMonitorList->u4NumElem);

	while (!LINK_IS_EMPTY(prMonitorList)) {
		LINK_REMOVE_HEAD(prMonitorList, prEntry,
			struct MSCS_FIVE_TUPLE_T *);
		kalMemFree(prEntry, VIR_MEM_TYPE,
			sizeof(struct MSCS_FIVE_TUPLE_T));
	}
}

struct MSCS_TCP_INFO_T *mscsSearchTcpEntry(IN struct ADAPTER *prAdapter,
	IN uint8_t *prTargetTcp)
{
	struct STA_RECORD *prStaRec = NULL;
	struct LINK *prMonitorList;
	struct MSCS_TCP_INFO_T *prTcpInfo;

	prStaRec = aisGetStaRecOfAP(prAdapter, AIS_DEFAULT_INDEX);
	if (!prStaRec)
		return NULL;

	prMonitorList = &prStaRec->rMscsTcpMonitorList;
	DBGLOG(TX, LOUD, "[F] monitor list(%p) len:%d\n",
			prMonitorList, prMonitorList->u4NumElem);
	DBGLOG_MEM8(TX, LOUD, prTargetTcp, LEN_OF_SRC_DST_IP_PORT);

	LINK_FOR_EACH_ENTRY(prTcpInfo, prMonitorList, rLinkEntry,
			    struct MSCS_TCP_INFO_T) {
		if (!prTcpInfo)
			break;

		if (!kalMemCmp(&prTcpInfo->u4SrcIp, prTargetTcp,
				LEN_OF_SRC_DST_IP_PORT)) {
			DBGLOG(TX, LOUD,
			       "Current 5-tuple already in monitoring list\n");
			return prTcpInfo;
		}
	}
	return NULL;
}


void mscsAddTcpForMonitor(IN struct ADAPTER *prAdapter,
	IN uint8_t *prTargetTuple)
{
	struct STA_RECORD *prStaRec = NULL;
	struct LINK *prMonitorList;
	struct MSCS_TCP_INFO_T *prTcpInfo = NULL;
	struct MSCS_TCP_INFO_T *prEntry;

	prStaRec = aisGetStaRecOfAP(prAdapter, AIS_DEFAULT_INDEX);
	if (!prStaRec)
		return;

	prMonitorList = &prStaRec->rMscsTcpMonitorList;
	DBGLOG(TX, TRACE, "[F] TCP monitor list(%p) len:%d\n",
			prMonitorList, prMonitorList->u4NumElem);

	LINK_FOR_EACH_ENTRY(prTcpInfo, prMonitorList, rLinkEntry,
			    struct MSCS_TCP_INFO_T) {
		if (!prTcpInfo) {
			DBGLOG(TX, TRACE, "[F] return because of no entry\n");
			break;
		}

		if (!kalMemCmp(&prTcpInfo->u4SrcIp, prTargetTuple,
			LEN_OF_SRC_DST_IP_PORT)) {
			DBGLOG(TX, TRACE,
			       "Current TCP already in monitoring list\n");
			return;
		}
	}
	DBGLOG(TX, TRACE, "No match add into monitor list\n");

	if (prMonitorList->u4NumElem >= MAX_MONITOR_FIVE_TUPLE) {
		LINK_REMOVE_HEAD(prMonitorList, prEntry,
			struct MSCS_TCP_INFO_T *);
		if (prEntry)
			kalMemFree(prEntry, VIR_MEM_TYPE,
				sizeof(struct MSCS_TCP_INFO_T));
	}

	prEntry = kalMemAlloc(sizeof(struct MSCS_TCP_INFO_T), VIR_MEM_TYPE);
	if (!prEntry) {
		DBGLOG(TX, WARN, "No memory for TCP Monitor\n");
		return;
	}
	kalMemCopy(&prEntry->u4SrcIp, prTargetTuple, LEN_OF_TCP_INFO);
	LINK_INSERT_TAIL(prMonitorList, &prEntry->rLinkEntry);
}

void mscsFlushMonitorTcp(IN struct ADAPTER *prAdapter,
	IN struct STA_RECORD *prStaRec)
{
	struct LINK *prMonitorList = &prStaRec->rMscsTcpMonitorList;
	struct MSCS_TCP_INFO_T *prEntry;

	DBGLOG(TX, TRACE, "Flush all monitor TCP(%d)\n",
		prMonitorList->u4NumElem);

	while (!LINK_IS_EMPTY(prMonitorList)) {
		LINK_REMOVE_HEAD(prMonitorList, prEntry,
			struct MSCS_TCP_INFO_T *);
		kalMemFree(prEntry, VIR_MEM_TYPE,
			sizeof(struct MSCS_TCP_INFO_T));
	}
}

void mscsDeactivate(IN struct ADAPTER *prAdapter,
	IN struct STA_RECORD *prStaRec)
{
	if (!prStaRec)
		return;

	mscsFlushFiveTuple(prAdapter, prStaRec);
	mscsFlushMonitorTcp(prAdapter, prStaRec);
}

uint32_t
mscsTxDoneCb(IN struct ADAPTER *prAdapter,
	      IN struct MSDU_INFO *prMsduInfo,
	      IN enum ENUM_TX_RESULT_CODE rTxDoneStatus)
{
	DBGLOG(TX, INFO,
		"MSCS TX DONE, WIDX:PID[%u:%u] Status[%u], SeqNo: %d\n",
		prMsduInfo->ucWlanIndex, prMsduInfo->ucPID, rTxDoneStatus,
		prMsduInfo->ucTxSeqNum);
	return WLAN_STATUS_SUCCESS;
}

uint8_t fpIsPortAuthorized(IN struct ADAPTER *prAdapter)
{
	struct BSS_INFO *prAisBssInfo = NULL;

	prAisBssInfo = aisGetAisBssInfo(prAdapter, AIS_DEFAULT_INDEX);
	if (!prAisBssInfo) {
		DBGLOG(TX, TRACE, "prAisBssInfo equal to NULL\n");
		return FALSE;
	}

	DBGLOG_LIMITED(TX, TRACE,
		"Support fast path->(%s), auth status->(%d)\n",
		prAdapter->rFastPathCap.fgSupportFastPath ? "Y" : "N",
		prAisBssInfo->rFastPathInfo.eAuthStatus);

	/* Check if FW supported */
	if (!prAdapter->rFastPathCap.fgSupportFastPath)
		return FALSE;

	if (prAisBssInfo->rFastPathInfo.eAuthStatus != PORT_AUTHORIZED)
		return FALSE;

	return TRUE;
}

uint8_t mscsIsFpSupport(IN struct ADAPTER *prAdapter)
{
	struct MSCS_CAP_FAST_PATH *prFastPathCap = &prAdapter->rFastPathCap;

	DBGLOG(TX, TRACE,
		"Fast path version(%d) support(%d) vendor key(0x%x) group key(0x%x)\n",
		prFastPathCap->ucVersion, prFastPathCap->fgSupportFastPath,
		prFastPathCap->u4KeyBitmap[0], prFastPathCap->u4KeyBitmap[2]);

	/* Check if Fast Path and key supported */
	if (prFastPathCap->fgSupportFastPath &&
	    (prFastPathCap->u4KeyBitmap[0] > 0 ||
		 prFastPathCap->u4KeyBitmap[2] > 0))
		return TRUE;

	return FALSE;
}

uint8_t mscsIsTcpNeedMonitor(IN struct ADAPTER *prAdapter, IN uint8_t *pucPkt)
{
	uint8_t *pucEthBody = &pucPkt[ETH_HLEN];
	uint8_t *pucTcpBody = &pucEthBody[IP_HEADER_LEN];
	uint8_t ucTcpFlag;
	uint32_t u4Ack;
	struct MSCS_TCP_INFO_T rTcpInfo;
	struct MSCS_TCP_INFO_T *prTcpEntry = NULL;

	/* Follow this rules:
	 * 1) For the same source/desntination IP and port,
	 * request to monitor after three-way handshake.
	 */
	ucTcpFlag = (pucTcpBody[TCP_HDR_FLAG_OFFSET]) & 0x3F;
	u4Ack = GET_TCP_ACK_FROM_TCP_BODY(pucTcpBody);
	DBGLOG(TX, LOUD, "[F] TCP flag:%d ack=%d\n", ucTcpFlag, u4Ack);

	SET_SRC_IP_FROM_ETH_BODY_DST_IP(&rTcpInfo, pucEthBody);
	SET_DST_IP_FROM_ETH_BODY_STC_IP(&rTcpInfo, pucEthBody);
	SET_SRC_PORT_FROM_PRO_BODY_DST_PORT(&rTcpInfo, pucTcpBody);
	SET_DST_PORT_FROM_PRO_BODY_SRC_PORT(&rTcpInfo, pucTcpBody);
	rTcpInfo.u4Seq = GET_TCP_SEQ_FROM_TCP_BODY(pucTcpBody);

	rTcpInfo.ucFlag = ucTcpFlag;
	DBGLOG_MEM8(TX, LOUD, &rTcpInfo.u4SrcIp, LEN_OF_TCP_INFO);

	if (ucTcpFlag == TCP_FLAG_SYN) {
		mscsAddTcpForMonitor(prAdapter, (uint8_t *) &rTcpInfo.u4SrcIp);
		return FALSE;
	} else if (ucTcpFlag == TCP_FLAG_ACK) {
		prTcpEntry = mscsSearchTcpEntry(prAdapter,
			(uint8_t *) &rTcpInfo.u4SrcIp);
		if (prTcpEntry && prTcpEntry->ucFlag == TCP_FLAG_SYN_ACK &&
			(u4Ack == prTcpEntry->u4Seq + 1))
			return TRUE;
	}
	return FALSE;
}

uint8_t mscsIsNeedRequest(IN struct ADAPTER *prAdapter, IN void *prPacket)
{
	struct sk_buff *prSkb = (struct sk_buff *)prPacket;
	struct STA_RECORD *prStaRec = NULL;
	struct MSCS_FIVE_TUPLE_T rTargetFiveTuple;
	struct WIFI_VAR *prWifiVar = &prAdapter->rWifiVar;
	uint16_t u2EtherType;
	uint8_t *pPkt, *pucEthBody, *pucUdpBody;
	uint8_t ucMonitorProto = prWifiVar->ucSupportProtocol;

	/* check feature enable */
	if (prWifiVar->ucEnableFastPath != FEATURE_ENABLED)
		return FALSE;

	/* 0. check port auth status */
	if (!fpIsPortAuthorized(prAdapter))
		return FALSE;

	/* 1. Check if packets come from monitor applications */
	if (prWifiVar->ucFastPathAllPacket != FEATURE_ENABLED) {
		if (prSkb->mark != NIC_TX_SKB_PRIORITY_MARK1 &&
		    !(prSkb->mark & BIT(NIC_TX_SKB_PRIORITY_MARK_BIT)))
			return FALSE;
	}

	/* 2. Check packets protocol */
	pPkt = prSkb->data;
	u2EtherType = (pPkt[ETH_TYPE_LEN_OFFSET] << 8)
		| (pPkt[ETH_TYPE_LEN_OFFSET + 1]);
	pucEthBody = &pPkt[ETH_HLEN];
	pucUdpBody = &pucEthBody[IP_HEADER_LEN];
	DBGLOG(TX, LOUD, "[F] IP protocol:%d\n", pucEthBody[9]);

	if (u2EtherType != ETH_P_IPV4 || (pucEthBody[9] != IP_PRO_UDP
		&& pucEthBody[9] != IP_PRO_TCP))
		return FALSE;

	if ((ucMonitorProto == MONITOR_UDP && pucEthBody[9] != IP_PRO_UDP) ||
		(ucMonitorProto == MONITOR_TCP && pucEthBody[9] != IP_PRO_TCP))
		return FALSE;

	/* 3. Check if connected AP support MSCS */
	prStaRec = aisGetStaRecOfAP(prAdapter, AIS_DEFAULT_INDEX);
	if (!prStaRec || !prStaRec->fgIsMscsSupported)
		return FALSE;

	/* Prepare a swap content for asking AP to monitor */
	SET_SRC_IP_FROM_ETH_BODY_DST_IP(&rTargetFiveTuple, pucEthBody);
	SET_DST_IP_FROM_ETH_BODY_STC_IP(&rTargetFiveTuple, pucEthBody);
	SET_SRC_PORT_FROM_PRO_BODY_DST_PORT(&rTargetFiveTuple, pucUdpBody);
	SET_DST_PORT_FROM_PRO_BODY_SRC_PORT(&rTargetFiveTuple, pucUdpBody);
	rTargetFiveTuple.ucProtocol = pucEthBody[9];

	DBGLOG(TX, LOUD, "[F] original IP content:\n");
	DBGLOG_MEM8(TX, LOUD, &pucEthBody[IPV4_HDR_IP_SRC_ADDR_OFFSET], 12);
	if (mscsSearchFiveTuple(
		prAdapter, (uint8_t *) &rTargetFiveTuple.u4SrcIp))
		return FALSE;

	/* Handle TCP case */
	if (pucEthBody[9] == IP_PRO_TCP &&
		!mscsIsTcpNeedMonitor(prAdapter, pPkt))
		return FALSE;

	return TRUE;
}

uint8_t mscsIsMtkOui(IN uint8_t *pucAddress)
{
	uint8_t aucMTKOui[] = VENDOR_OUI_MTK;

	if (!kalMemCmp(pucAddress, aucMTKOui, sizeof(aucMTKOui)))
		return TRUE;

	return FALSE;
}

uint32_t mscsSend(IN struct ADAPTER *prAdapter,
	IN enum ENUM_MSCS_REQUEST_ACTION eAction, IN uint8_t fgRequestType,
	IN uint8_t ucUPBitmap, IN uint8_t ucUPLimit,
	IN struct IE_TCLAS_MASK *prTclas)
{
	static uint8_t ucDialogToken = 1;
	struct MSDU_INFO *prMsduInfo;
	struct ACTION_MSCS_REQ_FRAME *prTxFrame;
	struct BSS_INFO *prAisBssInfo;
	struct STA_RECORD *prStaRec;
	struct IE_MSCS_DESC *prMscsDesc;
	struct IE_TSPEC_BODY *prTspec;
	struct WIFI_VAR *prWifiVar = NULL;
	uint16_t u2EstimatedFrameLen;
	uint8_t *pucPayload = NULL;
	uint8_t *pucMscsDescPayload = NULL;
	uint8_t fgIsMtkPortAuthorized = FALSE;
	uint16_t u2FrameLen = 0;
	uint8_t fgIsUdp = GET_PROTOCOL_BY_IE(prTclas) == IP_PRO_UDP;

	prWifiVar = &prAdapter->rWifiVar;
	prAisBssInfo = aisGetAisBssInfo(prAdapter, AIS_DEFAULT_INDEX);
	prStaRec = aisGetStaRecOfAP(prAdapter, AIS_DEFAULT_INDEX);

	if (!prAisBssInfo || !prStaRec) {
		DBGLOG(TX, ERROR, "prAisBssInfo or prStaRec equal to NULL\n");
		return WLAN_STATUS_FAILURE;
	}

	fgIsMtkPortAuthorized = fpIsPortAuthorized(prAdapter);

	/* Calculate MSDU buffer length */
	u2EstimatedFrameLen = MAC_TX_RESERVED_FIELD + PUBLIC_ACTION_MAX_LEN;

	/* Alloc MSDU_INFO */
	prMsduInfo = (struct MSDU_INFO *)cnmMgtPktAlloc(prAdapter,
							u2EstimatedFrameLen);
	if (!prMsduInfo) {
		DBGLOG(TX, ERROR, "Alloc packet failed for MSDU");
		return WLAN_STATUS_RESOURCES;
	}

	kalMemZero(prMsduInfo->prPacket, u2EstimatedFrameLen);
	prTxFrame = prMsduInfo->prPacket;

	/* 1. Fill MSCS descriptor */
	prMscsDesc = kalMemAlloc(
			sizeof(struct IE_MSCS_DESC) + prTclas->ucLength + 2,
			VIR_MEM_TYPE);
	if (!prMscsDesc)
		return WLAN_STATUS_RESOURCES;

	prMscsDesc->ucId = ELEM_ID_RESERVED;
	prMscsDesc->ucIdExt = ELEM_ID_EXT_MSCS_DESC;
	prMscsDesc->ucReqType = (fgRequestType == FALSE ?
		MSCS_DESC_REQ_TYPE_REMOVE : MSCS_DESC_REQ_TYPE_ADD);
	prMscsDesc->u2UserPriorityCtrl = ((ucUPLimit << 8) | ucUPBitmap);
	prMscsDesc->u4StreamTimeout = SEC_TO_TU(DEFAULT_STREAM_TIMEOUT);
	pucMscsDescPayload = &prMscsDesc->aucData[0];
	kalMemCopy(pucMscsDescPayload, prTclas, prTclas->ucLength + 2);
	prMscsDesc->ucLength = OFFSET_OF(struct IE_MSCS_DESC, aucData) - 2;
	prMscsDesc->ucLength += (prTclas->ucLength + 2);
	u2FrameLen += prMscsDesc->ucLength + 2;

	/* 2. Check if need to add TSPEC for DABS */
	if (fgIsMtkPortAuthorized) {
		prTspec = kalMemAlloc(
				sizeof(struct IE_TSPEC_BODY), VIR_MEM_TYPE);
		if (!prTspec) {
			DBGLOG(TX, ERROR, "Alloc packet failed for TSPEC");
			kalMemFree(prMscsDesc, VIR_MEM_TYPE,
				sizeof(struct IE_MSCS_DESC)
				+ prTclas->ucLength + 2);

			return WLAN_STATUS_RESOURCES;
		}
		kalMemZero(prTspec, sizeof(struct IE_TSPEC_BODY));

		prTspec->ucId = ELEM_ID_TSPEC;
		prTspec->ucLength = sizeof(struct IE_TSPEC_BODY) - 2;

		prTspec->aucTsInfo[1] = (fgIsUdp ? prWifiVar->ucUdpTspecUp :
			prWifiVar->ucTcpTspecUp) << 3;
		prTspec->u4DelayBound = (fgIsUdp ? prWifiVar->u4UdpDelayBound :
			prWifiVar->u4TcpDelayBound);
		u2FrameLen += sizeof(struct IE_TSPEC_BODY);
		DBGLOG(TX, LOUD, "[F] tspec len:%d\n",
				(uint8_t)sizeof(struct IE_TSPEC_BODY));
	}
	DBGLOG(TX, LOUD, "[F] u2FrameLen: %d\n", u2FrameLen);

	/* 3. Fill frame ctrl */
	prTxFrame->u2FrameCtrl = MAC_FRAME_ACTION;
	COPY_MAC_ADDR(prTxFrame->aucDestAddr, prStaRec->aucMacAddr);
	COPY_MAC_ADDR(prTxFrame->aucSrcAddr, prAisBssInfo->aucOwnMacAddr);
	COPY_MAC_ADDR(prTxFrame->aucBSSID, prAisBssInfo->aucBSSID);

	/* 4. Compose the frame body's frame */
	prTxFrame->ucCategory = CATEGORY_ROBUST_AV_STREAMING_ACTION;
	prTxFrame->ucAction = eAction;
	prTxFrame->ucDialogToken = ucDialogToken++;
	pucPayload = &prTxFrame->aucMSCSDesc[0];
	kalMemCopy(pucPayload, prMscsDesc, prMscsDesc->ucLength + 2);
	if (fgIsMtkPortAuthorized)
		kalMemCopy(pucPayload + (prMscsDesc->ucLength + 2),
			prTspec, prTspec->ucLength + 2);
	u2FrameLen += OFFSET_OF(struct ACTION_MSCS_REQ_FRAME, aucMSCSDesc);

	nicTxSetMngPacket(prAdapter, prMsduInfo, prStaRec->ucBssIndex,
			  prStaRec->ucIndex, WLAN_MAC_MGMT_HEADER_LEN,
			  u2FrameLen, mscsTxDoneCb, MSDU_RATE_MODE_AUTO);

	DBGLOG(TX, LOUD, "[F] Frame content:");
	DBGLOG_MEM8(TX, LOUD, prMsduInfo->prPacket, u2FrameLen);
	DBGLOG(TX, LOUD, "[F] !=======================================!\n");

	/* 5. Enqueue the frame to send this action frame. */
	nicTxEnqueueMsdu(prAdapter, prMsduInfo);

	/* 6. Return resources */
	kalMemFree(prMscsDesc, VIR_MEM_TYPE,
		sizeof(struct IE_MSCS_DESC) + prTclas->ucLength + 2);
	if (fgIsMtkPortAuthorized)
		kalMemFree(prTspec, VIR_MEM_TYPE, sizeof(struct IE_TSPEC_BODY));

	return WLAN_STATUS_SUCCESS;
}

uint32_t mscsGenerateTCLASType4(IN struct ADAPTER *prAdapter,
	IN void *prPacket, OUT uint8_t *pucContent, IN uint32_t pu4Size)
{
	struct IE_TCLAS_MASK *prTclas;
	struct IE_TCLAS_CLASS_TYPE_4 *prContent;
	struct sk_buff *prSkb = (struct sk_buff *)prPacket;
	uint8_t *pucEthBody, *pucUdpBody;

	/* Fill content */
	prTclas = (struct IE_TCLAS_MASK *) pucContent;
	prContent = (struct IE_TCLAS_CLASS_TYPE_4 *)
		&prTclas->aucFrameClassifier[0];

	pucEthBody = &prSkb->data[ETH_HLEN];
	pucUdpBody = &pucEthBody[IP_HEADER_LEN];

	prTclas->ucId = ELEM_ID_RESERVED;
	prTclas->ucLength = OFFSET_OF(struct IE_TCLAS_MASK, aucFrameClassifier)
			- 2 + sizeof(struct IE_TCLAS_CLASS_TYPE_4);
	prTclas->ucIdExt = ELEM_ID_EXT_TCLAS_MASK;
	prContent->ucClassifierType = FRAME_CLASSIFIER_TYPE_4;
	prContent->ucClassifierMask = 94; /* 01011110 */
	prContent->ucVersion = IP_VERSION_4;
	/* Ask AP to monitor, so swap source and destination */
	SET_SRC_IP_FROM_ETH_BODY_DST_IP(prContent, pucEthBody);
	SET_DST_IP_FROM_ETH_BODY_STC_IP(prContent, pucEthBody);
	SET_SRC_PORT_FROM_PRO_BODY_DST_PORT(prContent, pucUdpBody);
	SET_DST_PORT_FROM_PRO_BODY_SRC_PORT(prContent, pucUdpBody);
	prContent->ucDSCP = (pucEthBody[IPV4_HDR_TOS_OFFSET] >> 2) & 0x3F;
	prContent->ucProtocol = pucEthBody[IPV4_HDR_IP_PROTOCOL_OFFSET];
	DBGLOG(TX, LOUD, "TCLAS len:%d %d\n", pu4Size, prTclas->ucLength);
	return WLAN_STATUS_SUCCESS;
}

uint32_t mscsRequest(IN struct ADAPTER *prAdapter,
	IN void *prPacket, IN enum ENUM_MSCS_REQUEST_ACTION eAction,
	IN enum ENUM_MSCS_TCLAS_TYPE eTCLASType)
{
	uint32_t u4Status = WLAN_STATUS_SUCCESS;
	uint32_t u4TCLASLen = 0;
	struct MSCS_FIVE_TUPLE_T rTargetFiveTuple;
	uint8_t *pucTCLAS = NULL;

	/* 1. Retrieve TCLAS if needed */
	switch (eTCLASType) {
	case FRAME_CLASSIFIER_TYPE_4:
		u4TCLASLen = sizeof(struct IE_TCLAS_MASK) +
			sizeof(struct IE_TCLAS_CLASS_TYPE_4);

		pucTCLAS = kalMemAlloc(u4TCLASLen, VIR_MEM_TYPE);
		if (!pucTCLAS) {
			DBGLOG(TX, ERROR, "Alloc TCLAS type 4 failed");
			return WLAN_STATUS_RESOURCES;
		}
		u4Status = mscsGenerateTCLASType4(
			prAdapter, prPacket, pucTCLAS, u4TCLASLen);
		break;
	default:
		DBGLOG(TX, WARN, "NOT support TCLAS type:%d\n", eTCLASType);
		return WLAN_STATUS_SUCCESS;
		break;
	}
	if (u4Status != WLAN_STATUS_SUCCESS)
		return u4Status;

	/* 2. Request to send a MSCS action frame */
	u4Status = mscsSend(prAdapter, eAction, TRUE, MSCS_USER_PRIORITY_BITMAP,
			MSCS_USER_PRIORITY_LIMIT,
			(struct IE_TCLAS_MASK *) pucTCLAS);

	/* 3. Add to maintained list */
	if (u4Status == WLAN_STATUS_SUCCESS) {
		DBGLOG(TX, TRACE, "[F] add 5-tuple\n");
		/* pucTargetFiveTuple = GET_SRC_IP_BY_IE(pucTCLAS);  */
		kalMemCopy(&rTargetFiveTuple.u4SrcIp,
			GET_SRC_IP_BY_IE(pucTCLAS), 12);
		rTargetFiveTuple.ucProtocol = GET_PROTOCOL_BY_IE(pucTCLAS);
		DBGLOG_MEM8(TX, TRACE, &rTargetFiveTuple.u4SrcIp,
			LEN_OF_FIVE_TUPLE);
		mscsAddFiveTuple(prAdapter, (uint8_t *)
			&rTargetFiveTuple.u4SrcIp);
	}

	if (pucTCLAS)
		kalMemFree(pucTCLAS, VIR_MEM_TYPE, u4TCLASLen);

	return u4Status;
}

void mscsProcessRobustAVStreaming(IN struct ADAPTER *prAdapter,
			   IN struct SW_RFB *prSwRfb)
{
	struct ACTION_MSCS_RSP_FRAME *prRxFrame;
	struct STA_RECORD *prStaRec;
	struct WIFI_VAR *prWifiVar = &prAdapter->rWifiVar;
	uint8_t *pucIE;
	uint16_t u2IELength, u2Offset;

	ASSERT(prAdapter);
	ASSERT(prSwRfb);

	if (prWifiVar->ucEnableFastPath != FEATURE_ENABLED)
		return;

	prRxFrame = (struct ACTION_MSCS_RSP_FRAME *)prSwRfb->pvHeader;
	prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);

	if (!prStaRec)
		return;

	DBGLOG(RX, INFO,
		"Received RAVS action:%d\n", prRxFrame->ucAction);

	switch (prRxFrame->ucAction) {
	case ACTION_MSCS_RSP:
		DBGLOG(RX, INFO,
			"Received MSCS response with status:%d\n",
			prRxFrame->u2StatusCode);
		/* Remove 5-tuple matched with any in monitor list */
		if (prRxFrame->u2StatusCode) {
			/* Process intolerant channel report IE */
			pucIE = (uint8_t *) &prRxFrame->aucMSCSDesc;
			u2IELength = prSwRfb->u2PacketLen -
				OFFSET_OF(struct ACTION_MSCS_RSP_FRAME,
				aucMSCSDesc);

			IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
				switch (IE_ID(pucIE)) {
				case ELEM_ID_RESERVED:
					if (IE_LEN(pucIE) >
						MSCS_DESC_MANDATORY_LEN &&
						IE_ID_EXT(pucIE) ==
						ELEM_ID_EXT_MSCS_DESC) {
						mscsDelFiveTuple(prAdapter,
							prSwRfb,
							GET_SRC_IP_BY_IE(
								pucIE));
					}
					break;
				default:
					break;
				}
			}
		}
		break;
	default:
		break;
	}
}

void mscsHandleRxPacket(IN struct ADAPTER *prAdapter, struct SW_RFB *prSwRfb)
{
	uint8_t *pucPkt = NULL;
	uint8_t *pucEthBody;
	uint16_t u2EtherType;
	uint8_t *pucTcpBody;
	uint32_t u4Ack;
	struct MSCS_TCP_INFO_T *prTcpEntry = NULL;

	if (prSwRfb->u2PacketLen <= ETHER_HEADER_LEN)
		return;

	pucPkt = prSwRfb->pvHeader;
	if (!pucPkt)
		return;

	pucEthBody = &pucPkt[ETH_HLEN];
	pucTcpBody = &pucEthBody[IP_HEADER_LEN];
	u2EtherType = (pucPkt[ETH_TYPE_LEN_OFFSET] << 8) |
			(pucPkt[ETH_TYPE_LEN_OFFSET + 1]);
	DBGLOG(NIC, TRACE, "TCP type:%d protocol:%d raw flag:%d\n", u2EtherType,
		pucEthBody[9], (pucTcpBody[TCP_HDR_FLAG_OFFSET] & 0x3F));
	if (u2EtherType != ETH_P_IPV4 || pucEthBody[9] != IP_PRO_TCP
	    || ((pucTcpBody[TCP_HDR_FLAG_OFFSET] & 0x3F) != TCP_FLAG_SYN_ACK))
		return;

	prTcpEntry = mscsSearchTcpEntry(prAdapter,
		(uint8_t *) &pucEthBody[IPV4_HDR_IP_SRC_ADDR_OFFSET]);
	if (!prTcpEntry)
		return;

	u4Ack = GET_TCP_ACK_FROM_TCP_BODY(pucTcpBody);
	DBGLOG(NIC, TRACE, "TCP ACK:%d, monitored SEQ:%d\n", u4Ack,
		prTcpEntry->u4Seq);
	if (prTcpEntry->u4Seq + 1 == u4Ack) {
		prTcpEntry->u4Seq = GET_TCP_SEQ_FROM_TCP_BODY(pucTcpBody);
		prTcpEntry->ucFlag = TCP_FLAG_SYN_ACK;
	}
}

uint32_t
fpTxDoneCb(IN struct ADAPTER *prAdapter,
	      IN struct MSDU_INFO *prMsduInfo,
	      IN enum ENUM_TX_RESULT_CODE rTxDoneStatus)
{
	DBGLOG(TX, INFO,
		"Fast path request TX DONE, WIDX:PID[%u:%u] Status[%u], SeqNo: %d\n",
		prMsduInfo->ucWlanIndex, prMsduInfo->ucPID, rTxDoneStatus,
		prMsduInfo->ucTxSeqNum);
	return WLAN_STATUS_SUCCESS;
}

uint32_t fpRequestPortAuth(IN struct ADAPTER *prAdapter,
	enum ENUM_FAST_PATH_TYPE eAuthType, enum ENUM_FAST_PATH_STATUS eStatus,
	uint8_t ucTransactionId, uint16_t u2Mic)
{
	struct BSS_INFO *prAisBssInfo;
	struct STA_RECORD *prStaRec;
	struct MSDU_INFO *prMsduInfo;
	struct ACTION_VENDOR_SPEC_PROTECTED_FRAME *prTxFrame;
	uint16_t u2FrameLen = 0;
	uint8_t aucMtkOui[] = VENDOR_OUI_MTK;

	prAisBssInfo = aisGetAisBssInfo(prAdapter, AIS_DEFAULT_INDEX);
	prStaRec = aisGetStaRecOfAP(prAdapter, AIS_DEFAULT_INDEX);

	if (!prAisBssInfo || !prStaRec) {
		DBGLOG(OID, ERROR, "prAisBssInfo or prStaRec equal to NULL\n");
		return WLAN_STATUS_FAILURE;
	}
	DBGLOG(OID, INFO, "Sending fast path response\n");

	/* Calculate MSDU buffer length */
	u2FrameLen = MAC_TX_RESERVED_FIELD +
		sizeof(struct ACTION_VENDOR_SPEC_PROTECTED_FRAME);

	/* Alloc MSDU_INFO */
	prMsduInfo = (struct MSDU_INFO *)cnmMgtPktAlloc(prAdapter, u2FrameLen);
	if (!prMsduInfo) {
		DBGLOG(OID, ERROR, "Alloc packet failed for MSDU");
		return WLAN_STATUS_RESOURCES;
	}
	kalMemZero(prMsduInfo->prPacket, u2FrameLen);

	prTxFrame = prMsduInfo->prPacket;

	/* Fill frame ctrl */
	prTxFrame->u2FrameCtrl = MAC_FRAME_ACTION;

	COPY_MAC_ADDR(prTxFrame->aucDestAddr, prStaRec->aucMacAddr);
	COPY_MAC_ADDR(prTxFrame->aucSrcAddr, prAisBssInfo->aucOwnMacAddr);
	COPY_MAC_ADDR(prTxFrame->aucBSSID, prAisBssInfo->aucBSSID);

	/* 3 Compose the frame body's frame */
	prTxFrame->ucCategory = CATEGORY_VENDOR_SPECIFIC_PROTECTED_ACTION;
	kalMemCopy(prTxFrame->aucOui, aucMtkOui, MAC_OUI_LEN);
	prTxFrame->ucFastPathVersion = prAdapter->rFastPathCap.ucVersion;
	prTxFrame->ucFastPathStatus = eStatus;
	prTxFrame->ucTransactionId = ucTransactionId;
	prTxFrame->ucFastPathType = eAuthType;
	prTxFrame->u2RandomNum = prAisBssInfo->rFastPathInfo.u2RandomNumSta;
	prTxFrame->u2Mic = u2Mic;
	prTxFrame->u2KeyNum = (uint16_t)
		(0x00FF & prAisBssInfo->rFastPathInfo.ucKeyNumHitted);
	kalMemCopy(prTxFrame->u4KeyBitmap,
		prAisBssInfo->rFastPathInfo.au4KeyBitmapHitted,
		KEY_BITMAP_LEN_BYTE);

	nicTxSetMngPacket(prAdapter, prMsduInfo, prStaRec->ucBssIndex,
			  prStaRec->ucIndex, WLAN_MAC_MGMT_HEADER_LEN,
			  u2FrameLen, fpTxDoneCb, MSDU_RATE_MODE_AUTO);

	/* 4 Enqueue the frame to send this action frame. */
	nicTxEnqueueMsdu(prAdapter, prMsduInfo);

	return WLAN_STATUS_SUCCESS;
}

void fpReset(struct ADAPTER *prAdapter)
{
	struct BSS_INFO *prAisBssInfo = NULL;

	prAisBssInfo = aisGetAisBssInfo(prAdapter, AIS_DEFAULT_INDEX);
	if (!prAisBssInfo) {
		DBGLOG(INIT, WARN, "Can't get AIS BSSInfo\n");
		return;
	}

	prAisBssInfo->rFastPathInfo.eAuthStatus = PORT_NOT_AUTHORIZED;
	kalMemZero(&prAisBssInfo->rFastPathInfo, sizeof(struct FAST_PATH_INFO));
}

void fpQueryInfo(struct ADAPTER *prAdapter, uint8_t *pucMacAddr,
	uint16_t u2RandomNum, uint32_t *pu4KeyBitmap)
{
	struct CMD_FAST_PATH *prCmdBody = NULL;
	struct BSS_INFO *prAisBssInfo = NULL;
	uint32_t rStatus;

	prCmdBody = (struct CMD_FAST_PATH *) kalMemAlloc(
					  sizeof(struct CMD_FAST_PATH),
					  VIR_MEM_TYPE);
	if (!prCmdBody) {
		DBGLOG(INIT, WARN, "No buf for query fast path info\n");
		return;
	}

	prAisBssInfo = aisGetAisBssInfo(prAdapter, AIS_DEFAULT_INDEX);
	if (!prAisBssInfo) {
		DBGLOG(INIT, WARN, "Can't get AIS BSSInfo\n");
		goto err;
	}

	kalMemZero(prCmdBody, sizeof(struct CMD_FAST_PATH));
	prCmdBody->ucCmdVer = 0x1;
	kalMemCopy(prCmdBody->aucOwnMac, pucMacAddr, MAC_ADDR_LEN);
	prCmdBody->u2RandomNum = u2RandomNum;
	kalMemCopy(prCmdBody->u4Keybitmap, pu4KeyBitmap, KEY_BITMAP_LEN_BYTE);

	rStatus = wlanSendSetQueryCmd(
		prAdapter,			      /* prAdapter */
		CMD_ID_FAST_PATH,	              /* ucCID */
		TRUE,				      /* fgSetQuery */
		FALSE,				      /* fgNeedResp */
		FALSE,				      /* fgIsOid */
		NULL,				      /* pfCmdDoneHandler */
		NULL,				      /* pfCmdTimeoutHandler */
		sizeof(struct CMD_FAST_PATH),         /* u4SetQueryInfoLen */
		(uint8_t *)prCmdBody,		      /* pucInfoBuffer */
		NULL,				      /* pvSetQueryBuffer */
		0				      /* u4SetQueryBufferLen */
		);

	if (rStatus != WLAN_STATUS_PENDING)
		DBGLOG(INIT, WARN, "set cmd fail\n");

err:
	kalMemFree(prCmdBody, VIR_MEM_TYPE, sizeof(struct CMD_FAST_PATH));
}

uint8_t fpExamKeyBitmap(uint32_t *pu4KeyBitmapA, uint32_t *pu4KeyBitmapB,
	uint32_t *pu4KeyBitmap)
{
	uint8_t ucIdx = 0;
	uint8_t fgIsKeyBitmapHitted = FALSE;

	for (ucIdx; ucIdx < KEY_BITMAP_LEN_DW; ucIdx++) {
		pu4KeyBitmap[ucIdx] =
			pu4KeyBitmapA[ucIdx] & pu4KeyBitmapB[ucIdx];
		if (pu4KeyBitmap[ucIdx])
			fgIsKeyBitmapHitted = TRUE;
	}
	return fgIsKeyBitmapHitted;
}

void fpEventHandler(IN struct ADAPTER *prAdapter,
		      IN struct WIFI_EVENT *prEvent)
{
	struct EVENT_FAST_PATH *prFastPathInfo;
	struct BSS_INFO *prAisBssInfo = NULL;

	DBGLOG(INIT, INFO, "Process fast path event\n");
	prFastPathInfo = (struct EVENT_FAST_PATH *) (prEvent->aucBuffer);
	prAisBssInfo = aisGetAisBssInfo(prAdapter, AIS_DEFAULT_INDEX);
	if (!prAisBssInfo) {
		DBGLOG(INIT, WARN, "Can't get AIS BSSInfo\n");
		return;
	}

	DBGLOG(INIT, INFO, "Port auth:%d, Key bitmap match:%d, KeyNum:%d\n",
		prAisBssInfo->rFastPathInfo.eAuthStatus,
		prFastPathInfo->ucKeyBitmapMatchStatus,
		prFastPathInfo->ucKeynum);
	if (prFastPathInfo->ucKeyBitmapMatchStatus != 0) {
		DBGLOG(INIT, WARN, "KeyBitmap match failed\n");
		fpReset(prAdapter);
		return;
	}

	if (prAisBssInfo->rFastPathInfo.eAuthStatus ==
		PORT_AUTHORIZING_VERIFY_AP) {
		prAisBssInfo->rFastPathInfo.ucKeyNumHitted =
			prFastPathInfo->ucKeynum;
		DBGLOG(INIT, INFO,
			"Fast path info from FW: KeyNum->%d Mic->%d\n",
			prFastPathInfo->ucKeynum,
			prFastPathInfo->u2Mic);

		/* Sanity check OK, then reterive sta info */
		prAisBssInfo->rFastPathInfo.u2RandomNumSta =
			(uint16_t) (kalRandomNumber() & 0xFFFF);
		prAisBssInfo->rFastPathInfo.u2MicRspAP = prFastPathInfo->u2Mic;
		fpQueryInfo(prAdapter, prAisBssInfo->aucOwnMacAddr,
			prAisBssInfo->rFastPathInfo.u2RandomNumSta,
			prAisBssInfo->rFastPathInfo.au4KeyBitmapHitted);
		prAisBssInfo->rFastPathInfo.eAuthStatus =
			PORT_AUTHORIZING_RETRIEVE_STA;
		fpRequestPortAuth(prAdapter, TYPE_REQ, STATUS_OK,
			prAisBssInfo->rFastPathInfo.ucTransactionId,
			prFastPathInfo->u2Mic);
	} else if (prAisBssInfo->rFastPathInfo.eAuthStatus ==
		PORT_AUTHORIZING_RETRIEVE_STA) {
		DBGLOG(INIT, INFO,
			"Fast path info from FW: KeyNum->%d Mic->%d\n",
			prFastPathInfo->ucKeynum,
			prFastPathInfo->u2Mic);
		prAisBssInfo->rFastPathInfo.ucKeyNumHitted =
						prFastPathInfo->ucKeynum;
		prAisBssInfo->rFastPathInfo.u2MicReqSta = prFastPathInfo->u2Mic;
	}
}

void fpProcessVendorSpecProtectedFrame(IN struct ADAPTER *prAdapter,
					 IN struct SW_RFB *prSwRfb)
{
	struct ACTION_VENDOR_SPEC_PROTECTED_FRAME *prRxFrame;
	struct BSS_INFO *prAisBssInfo = NULL;

	ASSERT(prAdapter);
	ASSERT(prSwRfb);

	prRxFrame =
		(struct ACTION_VENDOR_SPEC_PROTECTED_FRAME *)prSwRfb->pvHeader;

	if (!mscsIsMtkOui(prRxFrame->aucOui)) {
		DBGLOG(INIT, INFO,
			"Ignore non-MTK vendor specific protected IE");
		return;
	}

	if (prSwRfb->u2PacketLen <
		sizeof(struct ACTION_VENDOR_SPEC_PROTECTED_FRAME)) {
		DBGLOG(INIT, INFO, "Received frame len is unexpected:%d\n",
			prSwRfb->u2PacketLen);
		return;
	}

	prAisBssInfo = aisGetAisBssInfo(prAdapter, AIS_DEFAULT_INDEX);
	if (!prAisBssInfo) {
		DBGLOG(INIT, WARN, "Can't get AIS BSSInfo\n");
		return;
	}

	prAisBssInfo->rFastPathInfo.eAuthStatus = PORT_AUTHORIZE_REQ;
	switch (prRxFrame->ucFastPathType) {
	case TYPE_ANNOUNCE:
	{
		if (prAdapter->rFastPathCap.ucVersion !=
			prRxFrame->ucFastPathVersion) {
			DBGLOG(INIT, INFO,
				"Version mismatch between AP & STA\n");
			fpReset(prAdapter);
			return;
		}
		if (!fpExamKeyBitmap(prAdapter->rFastPathCap.u4KeyBitmap,
			prRxFrame->u4KeyBitmap,
			prAisBssInfo->rFastPathInfo.au4KeyBitmapHitted)) {
			DBGLOG(INIT, INFO, "Key mismatch between AP & STA\n");
			fpReset(prAdapter);
			return;
		}
		prAisBssInfo->rFastPathInfo.ucTransactionId =
			prRxFrame->ucTransactionId;
		DBGLOG(INIT, INFO,
			"Fast path ANNOUNCE\n");
		fpQueryInfo(prAdapter, prRxFrame->aucSrcAddr,
			prRxFrame->u2RandomNum,
			prAisBssInfo->rFastPathInfo.au4KeyBitmapHitted);
		prAisBssInfo->rFastPathInfo.eAuthStatus =
			PORT_AUTHORIZING_VERIFY_AP;
		break;
	}
	case TYPE_RSP:
	{
		enum ENUM_FAST_PATH_STATUS eStatus = STATUS_FAIL;

		DBGLOG(INIT, INFO, "Fast path negotiate result:%s\n",
			(prRxFrame->ucFastPathStatus == STATUS_OK) ?
			"OK" : "Failed");
		prAisBssInfo->rFastPathInfo.ucTransactionId =
			prRxFrame->ucTransactionId;
		if (prRxFrame->ucFastPathStatus == STATUS_OK) {
			/* Verify MIC which is calculated by AP */
			if (prAisBssInfo->rFastPathInfo.u2MicReqSta !=
				prRxFrame->u2Mic) {
				DBGLOG(INIT, INFO, "Fast path Mic mismatch\n");
				fpReset(prAdapter);
				eStatus = STATUS_FAIL;
			} else {
				prAisBssInfo->rFastPathInfo.eAuthStatus =
					PORT_AUTHORIZED;
				eStatus = STATUS_OK;
			}
			fpRequestPortAuth(prAdapter, TYPE_CONFIRM, eStatus,
				prRxFrame->ucTransactionId, 0);
		} else {
			fpReset(prAdapter);
		}
		break;
	}
	default:
		DBGLOG(INIT, INFO, "Unsupported fast path type:%d\n",
			prRxFrame->ucFastPathType);
		break;
	}
}

#endif

