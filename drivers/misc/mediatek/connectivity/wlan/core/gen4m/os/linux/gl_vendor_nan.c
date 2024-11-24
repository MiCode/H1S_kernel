/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

/*
 * gl_vendor_nan.c
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
#include "gl_cfg80211.h"
#include "gl_os.h"
#include "debug.h"
#include "gl_vendor.h"
#include "gl_wext.h"
#include "wlan_lib.h"
#include "wlan_oid.h"
#include <linux/can/netlink.h>
#include <net/cfg80211.h>
#include <net/netlink.h>

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

/* Helper function to Write and Read TLV called in indication as well as
 * request
 */
u16
nanWriteTlv(struct _NanTlv *pInTlv, u8 *pOutTlv)
{
	u16 writeLen = 0;
	u16 i;

	if (!pInTlv) {
		DBGLOG(NAN, ERROR, "NULL pInTlv\n");
		return writeLen;
	}

	if (!pOutTlv) {
		DBGLOG(NAN, ERROR, "NULL pOutTlv\n");
		return writeLen;
	}

	*pOutTlv++ = pInTlv->type & 0xFF;
	*pOutTlv++ = (pInTlv->type & 0xFF00) >> 8;
	writeLen += 2;

	DBGLOG(NAN, LOUD, "Write TLV type %u, writeLen %u\n", pInTlv->type,
	       writeLen);

	*pOutTlv++ = pInTlv->length & 0xFF;
	*pOutTlv++ = (pInTlv->length & 0xFF00) >> 8;
	writeLen += 2;

	DBGLOG(NAN, LOUD, "Write TLV length %u, writeLen %u\n", pInTlv->length,
	       writeLen);

	for (i = 0; i < pInTlv->length; ++i)
		*pOutTlv++ = pInTlv->value[i];

	writeLen += pInTlv->length;
	DBGLOG(NAN, LOUD, "Write TLV value, writeLen %u\n", writeLen);
	return writeLen;
}

u16
nan_read_tlv(u8 *pInTlv, struct _NanTlv *pOutTlv)
{
	u16 readLen = 0;

	if (!pInTlv)
		return readLen;

	if (!pOutTlv)
		return readLen;

	pOutTlv->type = *pInTlv++;
	pOutTlv->type |= *pInTlv++ << 8;
	readLen += 2;

	pOutTlv->length = *pInTlv++;
	pOutTlv->length |= *pInTlv++ << 8;
	readLen += 2;

	if (pOutTlv->length) {
		pOutTlv->value = pInTlv;
		readLen += pOutTlv->length;
	} else {
		pOutTlv->value = NULL;
	}
	return readLen;
}

u8 *
nanAddTlv(u16 type, u16 length, u8 *value, u8 *pOutTlv)
{
	struct _NanTlv nanTlv;
	u16 len;

	nanTlv.type = type;
	nanTlv.length = length;
	nanTlv.value = (u8 *)value;

	len = nanWriteTlv(&nanTlv, pOutTlv);
	return (pOutTlv + len);
}

u16
nanMapPublishReqParams(u16 *pIndata, struct NanPublishRequest *pOutparams)
{
	u16 readLen = 0;
	u32 *pPublishParams = NULL;

	DBGLOG(REQ, INFO, "Into nanMapPublishReqParams\n");

	/* Get value of ttl(time to live) */
	pOutparams->ttl = *pIndata;

	/* Get value of ttl(time to live) */
	pIndata++;
	readLen += 2;

	/* Get value of period */
	pOutparams->period = *pIndata;

	/* Assign default value */
	if (pOutparams->period == 0)
		pOutparams->period = 1;

	pIndata++;
	readLen += 2;

	pPublishParams = (u32 *)pIndata;
	dumpMemory32(pPublishParams, 4);
	pOutparams->recv_indication_cfg =
		(u8)(GET_PUB_REPLY_IND_FLAG(*pPublishParams) |
		     GET_PUB_FOLLOWUP_RX_IND_DISABLE_FLAG(*pPublishParams) |
		     GET_PUB_MATCH_EXPIRED_IND_DISABLE_FLAG(*pPublishParams) |
		     GET_PUB_TERMINATED_IND_DISABLE_FLAG(*pPublishParams));
	pOutparams->publish_type = GET_PUB_PUBLISH_TYPE(*pPublishParams);
	pOutparams->tx_type = GET_PUB_TX_TYPE(*pPublishParams);
	pOutparams->rssi_threshold_flag =
		(u8)GET_PUB_RSSI_THRESHOLD_FLAG(*pPublishParams);
	pOutparams->publish_match_indicator =
		GET_PUB_MATCH_ALG(*pPublishParams);
	pOutparams->publish_count = (u8)GET_PUB_COUNT(*pPublishParams);
	pOutparams->connmap = (u8)GET_PUB_CONNMAP(*pPublishParams);
	readLen += 4;

	DBGLOG(REQ, INFO,
	       "[Publish Req] ttl: %u, period: %u, recv_indication_cfg: %x, publish_type: %u,tx_type: %u, rssi_threshold_flag: %u, publish_match_indicator: %u, publish_count:%u, connmap:%u\n",
	       pOutparams->ttl, pOutparams->period,
	       pOutparams->recv_indication_cfg, pOutparams->publish_type,
	       pOutparams->tx_type, pOutparams->rssi_threshold_flag,
	       pOutparams->publish_match_indicator, pOutparams->publish_count,
	       pOutparams->connmap);

	DBGLOG(REQ, INFO, " readLen : %d\n", readLen);
	return readLen;
}

u16
nanMapSubscribeReqParams(u16 *pIndata, struct NanSubscribeRequest *pOutparams)
{
	u16 readLen = 0;
	u32 *pSubscribeParams = NULL;

	DBGLOG(NAN, ERROR, "IN %s\n", __func__);

	pOutparams->ttl = *pIndata;
	pIndata++;
	readLen += 2;

	pOutparams->period = *pIndata;
	pIndata++;
	readLen += 2;

	pSubscribeParams = (u32 *)pIndata;

	pOutparams->subscribe_type = GET_SUB_SUBSCRIBE_TYPE(*pSubscribeParams);
	pOutparams->serviceResponseFilter = GET_SUB_SRF_ATTR(*pSubscribeParams);
	pOutparams->serviceResponseInclude =
		GET_SUB_SRF_INCLUDE(*pSubscribeParams);
	pOutparams->useServiceResponseFilter =
		GET_SUB_SRF_SEND(*pSubscribeParams);
	pOutparams->ssiRequiredForMatchIndication =
		GET_SUB_SSI_REQUIRED(*pSubscribeParams);
	pOutparams->subscribe_match_indicator =
		GET_SUB_MATCH_ALG(*pSubscribeParams);
	pOutparams->subscribe_count = (u8)GET_SUB_COUNT(*pSubscribeParams);
	pOutparams->rssi_threshold_flag =
		(u8)GET_SUB_RSSI_THRESHOLD_FLAG(*pSubscribeParams);
	pOutparams->recv_indication_cfg =
		(u8)GET_SUB_FOLLOWUP_RX_IND_DISABLE_FLAG(*pSubscribeParams) |
		GET_SUB_MATCH_EXPIRED_IND_DISABLE_FLAG(*pSubscribeParams) |
		GET_SUB_TERMINATED_IND_DISABLE_FLAG(*pSubscribeParams);

	DBGLOG(REQ, ERROR,
	       "[Subscribe Req] subscribe_type: %u, ssiRequiredForMatchIndication: %u, subscribe_match_indicator: %x, rssi_threshold_flag: %u\n",
	       pOutparams->subscribe_type,
	       pOutparams->ssiRequiredForMatchIndication,
	       pOutparams->subscribe_match_indicator,
	       pOutparams->rssi_threshold_flag);
	pOutparams->connmap = (u8)GET_SUB_CONNMAP(*pSubscribeParams);
	readLen += 4;
	DBGLOG(REQ, LOUD, "Subscribe readLen : %d\n", readLen);
	return readLen;
}

u16
nanMapFollowupReqParams(u32 *pIndata,
			struct NanTransmitFollowupRequest *pOutparams)
{
	u16 readLen = 0;
	u32 *pXmitFollowupParams = NULL;

	pOutparams->requestor_instance_id = *pIndata;
	pIndata++;
	readLen += 4;

	pXmitFollowupParams = pIndata;

	pOutparams->priority = GET_FLWUP_PRIORITY(*pXmitFollowupParams);
	pOutparams->dw_or_faw = GET_FLWUP_WINDOW(*pXmitFollowupParams);
	pOutparams->recv_indication_cfg =
		GET_FLWUP_TX_RSP_DISABLE_FLAG(*pXmitFollowupParams);
	readLen += 4;

	DBGLOG(NAN, INFO,
	       "[%s]priority: %u, dw_or_faw: %u, recv_indication_cfg: %u\n",
	       __func__, pOutparams->priority, pOutparams->dw_or_faw,
	       pOutparams->recv_indication_cfg);

	return readLen;
}

void
nanMapSdeaCtrlParams(u32 *pIndata,
		     struct NanSdeaCtrlParams *prNanSdeaCtrlParms)
{
	prNanSdeaCtrlParms->config_nan_data_path =
		GET_SDEA_DATA_PATH_REQUIRED(*pIndata);
	prNanSdeaCtrlParms->ndp_type = GET_SDEA_DATA_PATH_TYPE(*pIndata);
	prNanSdeaCtrlParms->security_cfg = GET_SDEA_SECURITY_REQUIRED(*pIndata);
	prNanSdeaCtrlParms->ranging_state = GET_SDEA_RANGING_REQUIRED(*pIndata);
	prNanSdeaCtrlParms->range_report = GET_SDEA_RANGE_REPORT(*pIndata);
	prNanSdeaCtrlParms->fgFSDRequire = GET_SDEA_FSD_REQUIRED(*pIndata);
	prNanSdeaCtrlParms->fgGAS = GET_SDEA_FSD_WITH_GAS(*pIndata);
	prNanSdeaCtrlParms->fgQoS = GET_SDEA_QOS_REQUIRED(*pIndata);
	prNanSdeaCtrlParms->fgRangeLimit =
		GET_SDEA_RANGE_LIMIT_PRESENT(*pIndata);

	DBGLOG(NAN, INFO,
	       "[%s]config_nan_data_path: %u, ndp_type: %u, security_cfg: %u\n",
	       __func__, prNanSdeaCtrlParms->config_nan_data_path,
	       prNanSdeaCtrlParms->ndp_type, prNanSdeaCtrlParms->security_cfg);
	DBGLOG(NAN, INFO,
	       "[%s]ranging_state: %u, range_report: %u, fgFSDRequire: %u\n",
	       __func__, prNanSdeaCtrlParms->ranging_state,
	       prNanSdeaCtrlParms->range_report,
	       prNanSdeaCtrlParms->fgFSDRequire);
	DBGLOG(NAN, INFO, "[%s]fgGAS: %u, fgQoS: %u, fgRangeLimit: %u\n",
	       __func__, prNanSdeaCtrlParms->fgGAS, prNanSdeaCtrlParms->fgQoS,
	       prNanSdeaCtrlParms->fgRangeLimit);
}

void
nanMapRangingConfigParams(u32 *pIndata, struct NanRangingCfg *prNanRangingCfg)
{
	struct NanFWRangeConfigParams *prNanFWRangeCfgParams;

	prNanFWRangeCfgParams = (struct NanFWRangeConfigParams *)pIndata;

	prNanRangingCfg->ranging_resolution =
		prNanFWRangeCfgParams->range_resolution;
	prNanRangingCfg->ranging_interval_msec =
		prNanFWRangeCfgParams->range_interval;
	prNanRangingCfg->config_ranging_indications =
		prNanFWRangeCfgParams->ranging_indication_event;

	if (prNanRangingCfg->config_ranging_indications &
	    NAN_RANGING_INDICATE_INGRESS_MET_MASK)
		prNanRangingCfg->distance_ingress_cm =
			prNanFWRangeCfgParams->geo_fence_threshold
				.inner_threshold /
			10;
	if (prNanRangingCfg->config_ranging_indications &
	    NAN_RANGING_INDICATE_EGRESS_MET_MASK)
		prNanRangingCfg->distance_egress_cm =
			prNanFWRangeCfgParams->geo_fence_threshold
				.outer_threshold /
			10;

	DBGLOG(NAN, INFO,
	       "[%s]ranging_resolution: %u, ranging_interval_msec: %u, config_ranging_indications: %u\n",
	       __func__, prNanRangingCfg->ranging_resolution,
	       prNanRangingCfg->ranging_interval_msec,
	       prNanRangingCfg->config_ranging_indications);
	DBGLOG(NAN, INFO, "[%s]distance_egress_cm: %u\n", __func__,
	       prNanRangingCfg->distance_egress_cm);
}

void
nanMapNan20RangingReqParams(u32 *pIndata,
			    struct NanRangeResponseCfg *prNanRangeRspCfgParms)
{
	struct NanFWRangeReqMsg *pNanFWRangeReqMsg;

	pNanFWRangeReqMsg = (struct NanFWRangeReqMsg *)pIndata;

	prNanRangeRspCfgParms->requestor_instance_id =
		pNanFWRangeReqMsg->range_id;
	memcpy(&prNanRangeRspCfgParms->peer_addr,
	       &pNanFWRangeReqMsg->range_mac_addr, NAN_MAC_ADDR_LEN);
	if (pNanFWRangeReqMsg->ranging_accept == 1)
		prNanRangeRspCfgParms->ranging_response_code =
			NAN_RANGE_REQUEST_ACCEPT;
	else if (pNanFWRangeReqMsg->ranging_reject == 1)
		prNanRangeRspCfgParms->ranging_response_code =
			NAN_RANGE_REQUEST_REJECT;
	else
		prNanRangeRspCfgParms->ranging_response_code =
			NAN_RANGE_REQUEST_CANCEL;

	DBGLOG(NAN, INFO,
	       "[%s]requestor_instance_id: %u, ranging_response_code:%u\n",
	       __func__, prNanRangeRspCfgParms->requestor_instance_id,
	       prNanRangeRspCfgParms->ranging_response_code);
	DBGFWLOG(NAN, INFO, "[%s] addr=>%02x:%02x:%02x:%02x:%02x:%02x\n",
		 __func__, prNanRangeRspCfgParms->peer_addr[0],
		 prNanRangeRspCfgParms->peer_addr[1],
		 prNanRangeRspCfgParms->peer_addr[2],
		 prNanRangeRspCfgParms->peer_addr[3],
		 prNanRangeRspCfgParms->peer_addr[4],
		 prNanRangeRspCfgParms->peer_addr[5]);
}

u32
wlanoidGetNANCapabilitiesRsp(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
			     IN uint32_t u4SetBufferLen,
			     OUT uint32_t *pu4SetInfoLen)
{
	struct NanCapabilitiesRspMsg nanCapabilitiesRsp;
	struct NanCapabilitiesRspMsg *pNanCapabilitiesRsp =
		(struct NanCapabilitiesRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	nanCapabilitiesRsp.fwHeader.msgVersion = 1;
	nanCapabilitiesRsp.fwHeader.msgId = NAN_MSG_ID_CAPABILITIES_RSP;
	nanCapabilitiesRsp.fwHeader.msgLen =
		sizeof(struct NanCapabilitiesRspMsg);
	nanCapabilitiesRsp.fwHeader.transactionId =
		pNanCapabilitiesRsp->fwHeader.transactionId;
	nanCapabilitiesRsp.status = 0;
	nanCapabilitiesRsp.max_service_name_len = NAN_MAX_SERVICE_NAME_LEN;
	nanCapabilitiesRsp.max_match_filter_len = NAN_MAX_MATCH_FILTER_LEN;
	nanCapabilitiesRsp.max_service_specific_info_len =
		NAN_MAX_SERVICE_SPECIFIC_INFO_LEN;
	nanCapabilitiesRsp.max_scid_len = NAN_MAX_SCID_BUF_LEN;
	nanCapabilitiesRsp.max_publishes = 4;
	nanCapabilitiesRsp.max_subscribes = 4;
	nanCapabilitiesRsp.max_total_match_filter_len =
		NAN_MAX_SERVICE_SPECIFIC_INFO_LEN;
	nanCapabilitiesRsp.cipher_suites_supported =
		NAN_CIPHER_SUITE_SHARED_KEY_128_MASK;

	/*  Fill values of nanCapabilitiesRsp */
	skb = cfg80211_vendor_event_alloc(wiphy, wdev,
					  sizeof(struct NanCapabilitiesRspMsg) +
						  NLMSG_HDRLEN,
					  WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanCapabilitiesRspMsg),
			     &nanCapabilitiesRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

u32
wlanoidNANEnableRsp(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
		    IN uint32_t u4SetBufferLen, OUT uint32_t *pu4SetInfoLen)
{
	struct NanEnableRspMsg nanEnableRsp;
	struct NanEnableRspMsg *pNanEnableRsp =
		(struct NanEnableRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	nanEnableRsp.fwHeader.msgVersion = 1;
	nanEnableRsp.fwHeader.msgId = NAN_MSG_ID_ENABLE_RSP;
	nanEnableRsp.fwHeader.msgLen = sizeof(struct NanEnableRspMsg);
	nanEnableRsp.fwHeader.transactionId =
		pNanEnableRsp->fwHeader.transactionId;
	nanEnableRsp.status = 0;
	nanEnableRsp.value = 0;

	/*  Fill values of nanCapabilitiesRsp */
	skb = cfg80211_vendor_event_alloc(
		wiphy, wdev, sizeof(struct NanEnableRspMsg) + NLMSG_HDRLEN,
		WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanEnableRspMsg),
			     &nanEnableRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

u32
wlanoidNANDisableRsp(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
		     IN uint32_t u4SetBufferLen, OUT uint32_t *pu4SetInfoLen)
{
	struct NanDisableRspMsg nanDisableRsp;
	struct NanDisableRspMsg *pNanDisableRsp =
		(struct NanDisableRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	nanDisableRsp.fwHeader.msgVersion = 1;
	nanDisableRsp.fwHeader.msgId = NAN_MSG_ID_DISABLE_RSP;
	nanDisableRsp.fwHeader.msgLen = sizeof(struct NanDisableRspMsg);
	nanDisableRsp.fwHeader.transactionId =
		pNanDisableRsp->fwHeader.transactionId;
	nanDisableRsp.status = 0;

	/*  Fill values of nanCapabilitiesRsp */
	skb = cfg80211_vendor_event_alloc(
		wiphy, wdev, sizeof(struct NanDisableRspMsg) + NLMSG_HDRLEN,
		WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanDisableRspMsg),
			     &nanDisableRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

u32
wlanoidGetNANConfigurationRsp(IN struct ADAPTER *prAdapter,
			      IN void *pvSetBuffer, IN uint32_t u4SetBufferLen,
			      OUT uint32_t *pu4SetInfoLen)
{
	struct NanConfigRspMsg nanConfigRsp;
	struct NanConfigRspMsg *pNanConfigRsp =
		(struct NanConfigRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	nanConfigRsp.fwHeader.msgVersion = 1;
	nanConfigRsp.fwHeader.msgId = NAN_MSG_ID_CONFIGURATION_RSP;
	nanConfigRsp.fwHeader.msgLen = sizeof(struct NanConfigRspMsg);
	nanConfigRsp.fwHeader.transactionId =
		pNanConfigRsp->fwHeader.transactionId;
	nanConfigRsp.status = 0;
	nanConfigRsp.value = 0;

	/*  Fill values of nanCapabilitiesRsp */
	skb = cfg80211_vendor_event_alloc(
		wiphy, wdev, sizeof(struct NanConfigRspMsg) + NLMSG_HDRLEN,
		WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanConfigRspMsg),
			     &nanConfigRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

u32
wlanoidNanPublishRsp(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
		     IN uint32_t u4SetBufferLen, OUT uint32_t *pu4SetInfoLen)
{
	struct NanPublishServiceRspMsg nanPublishRsp;
	struct NanPublishServiceRspMsg *pNanPublishRsp =
		(struct NanPublishServiceRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	kalMemZero(&nanPublishRsp, sizeof(struct NanPublishServiceRspMsg));
	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	DBGLOG(REQ, ERROR, "%s\n", __func__);

	/* Prepare publish response header*/
	nanPublishRsp.fwHeader.msgVersion = 1;
	nanPublishRsp.fwHeader.msgId = NAN_MSG_ID_PUBLISH_SERVICE_RSP;
	nanPublishRsp.fwHeader.msgLen = sizeof(struct NanPublishServiceRspMsg);
	nanPublishRsp.fwHeader.handle = pNanPublishRsp->fwHeader.handle;
	nanPublishRsp.fwHeader.transactionId =
		pNanPublishRsp->fwHeader.transactionId;
	nanPublishRsp.value = 0;

	if (nanPublishRsp.fwHeader.handle != WLAN_STATUS_FAILURE)
		nanPublishRsp.status = NAN_I_STATUS_SUCCESS;
	else
		nanPublishRsp.status = NAN_I_STATUS_INVALID_HANDLE;

	DBGLOG(REQ, ERROR, "publish ID:%u, msgId:%u, msgLen:%u, tranID:%u\n",
	       nanPublishRsp.fwHeader.handle, nanPublishRsp.fwHeader.msgId,
	       nanPublishRsp.fwHeader.msgLen,
	       nanPublishRsp.fwHeader.transactionId);

	/*  Fill values of nanPublishRsp */
	skb = cfg80211_vendor_event_alloc(
		wiphy, wdev,
		sizeof(struct NanPublishServiceRspMsg) + NLMSG_HDRLEN,
		WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanPublishServiceRspMsg),
			     &nanPublishRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);

	/* Free the memory due to no use anymore */
	kfree(pvSetBuffer);

	return WLAN_STATUS_SUCCESS;
}

u32
wlanoidNANCancelPublishRsp(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
			   IN uint32_t u4SetBufferLen,
			   OUT uint32_t *pu4SetInfoLen)
{
	struct NanPublishServiceCancelRspMsg nanPublishCancelRsp;
	struct NanPublishServiceCancelRspMsg *pNanPublishCancelRsp =
		(struct NanPublishServiceCancelRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	kalMemZero(&nanPublishCancelRsp,
		   sizeof(struct NanPublishServiceCancelRspMsg));
	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	DBGLOG(REQ, ERROR, "%s\n", __func__);

	nanPublishCancelRsp.fwHeader.msgVersion = 1;
	nanPublishCancelRsp.fwHeader.msgId =
		NAN_MSG_ID_PUBLISH_SERVICE_CANCEL_RSP;
	nanPublishCancelRsp.fwHeader.msgLen =
		sizeof(struct NanPublishServiceCancelRspMsg);
	nanPublishCancelRsp.fwHeader.handle =
		pNanPublishCancelRsp->fwHeader.handle;
	nanPublishCancelRsp.fwHeader.transactionId =
		pNanPublishCancelRsp->fwHeader.transactionId;
	nanPublishCancelRsp.value = 0;
	nanPublishCancelRsp.status = pNanPublishCancelRsp->status;

	DBGLOG(REQ, INFO, "[%s] nanPublishCancelRsp.fwHeader.handle = %d\n",
	       __func__, nanPublishCancelRsp.fwHeader.handle);
	DBGLOG(REQ, INFO,
	       "[%s] nanPublishCancelRsp.fwHeader.transactionId = %d\n",
	       __func__, nanPublishCancelRsp.fwHeader.transactionId);

	skb = cfg80211_vendor_event_alloc(
		wiphy, wdev,
		sizeof(struct NanPublishServiceCancelRspMsg) + NLMSG_HDRLEN,
		WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanPublishServiceCancelRspMsg),
			     &nanPublishCancelRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);

	kfree(pvSetBuffer);
	return WLAN_STATUS_SUCCESS;
}

u32
wlanoidNanSubscribeRsp(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
		       IN uint32_t u4SetBufferLen,
		       OUT uint32_t *pu4SetInfoLen)
{
	struct NanSubscribeServiceRspMsg nanSubscribeRsp;
	struct NanSubscribeServiceRspMsg *pNanSubscribeRsp =
		(struct NanSubscribeServiceRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	kalMemZero(&nanSubscribeRsp, sizeof(struct NanSubscribeServiceRspMsg));
	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	DBGLOG(REQ, ERROR, "%s\n", __func__);

	nanSubscribeRsp.fwHeader.msgVersion = 1;
	nanSubscribeRsp.fwHeader.msgId = NAN_MSG_ID_SUBSCRIBE_SERVICE_RSP;
	nanSubscribeRsp.fwHeader.msgLen =
		sizeof(struct NanSubscribeServiceRspMsg);
	nanSubscribeRsp.fwHeader.handle = pNanSubscribeRsp->fwHeader.handle;
	nanSubscribeRsp.fwHeader.transactionId =
		pNanSubscribeRsp->fwHeader.transactionId;
	nanSubscribeRsp.value = 0;
	if (nanSubscribeRsp.fwHeader.handle != WLAN_STATUS_FAILURE)
		nanSubscribeRsp.status = NAN_I_STATUS_SUCCESS;
	else
		nanSubscribeRsp.status = NAN_I_STATUS_INVALID_HANDLE;

	/*  Fill values of nanSubscribeRsp */
	skb = cfg80211_vendor_event_alloc(
		wiphy, wdev,
		sizeof(struct NanSubscribeServiceRspMsg) + NLMSG_HDRLEN,
		WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanSubscribeServiceRspMsg),
			     &nanSubscribeRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);

	DBGLOG(REQ, INFO, "handle:%u,transactionId:%u\n",
	       nanSubscribeRsp.fwHeader.handle,
	       nanSubscribeRsp.fwHeader.transactionId);

	kfree(pvSetBuffer);
	return WLAN_STATUS_SUCCESS;
}

u32
wlanoidNANCancelSubscribeRsp(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
			     IN uint32_t u4SetBufferLen,
			     OUT uint32_t *pu4SetInfoLen)
{
	struct NanSubscribeServiceCancelRspMsg nanSubscribeCancelRsp;
	struct NanSubscribeServiceCancelRspMsg *pNanSubscribeCancelRsp =
		(struct NanSubscribeServiceCancelRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	kalMemZero(&nanSubscribeCancelRsp,
		   sizeof(struct NanSubscribeServiceCancelRspMsg));
	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	DBGLOG(REQ, ERROR, "%s\n", __func__);

	nanSubscribeCancelRsp.fwHeader.msgVersion = 1;
	nanSubscribeCancelRsp.fwHeader.msgId =
		NAN_MSG_ID_SUBSCRIBE_SERVICE_CANCEL_RSP;
	nanSubscribeCancelRsp.fwHeader.msgLen =
		sizeof(struct NanSubscribeServiceCancelRspMsg);
	nanSubscribeCancelRsp.fwHeader.handle =
		pNanSubscribeCancelRsp->fwHeader.handle;
	nanSubscribeCancelRsp.fwHeader.transactionId =
		pNanSubscribeCancelRsp->fwHeader.transactionId;
	nanSubscribeCancelRsp.value = 0;
	nanSubscribeCancelRsp.status = pNanSubscribeCancelRsp->status;

	/*  Fill values of NanSubscribeServiceCancelRspMsg */
	skb = cfg80211_vendor_event_alloc(
		wiphy, wdev,
		sizeof(struct NanSubscribeServiceCancelRspMsg) + NLMSG_HDRLEN,
		WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanSubscribeServiceCancelRspMsg),
			     &nanSubscribeCancelRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	DBGLOG(REQ, ERROR, "handle:%u, transactionId:%u\n",
	       nanSubscribeCancelRsp.fwHeader.handle,
	       nanSubscribeCancelRsp.fwHeader.transactionId);

	kfree(pvSetBuffer);
	return WLAN_STATUS_SUCCESS;
}

u32
wlanoidNANFollowupRsp(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
		      IN uint32_t u4SetBufferLen, OUT uint32_t *pu4SetInfoLen)
{
	struct NanTransmitFollowupRspMsg nanXmitFollowupRsp;
	struct NanTransmitFollowupRspMsg *pNanXmitFollowupRsp =
		(struct NanTransmitFollowupRspMsg *)pvSetBuffer;
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	kalMemZero(&nanXmitFollowupRsp,
		   sizeof(struct NanTransmitFollowupRspMsg));

	DBGLOG(REQ, ERROR, "%s\n", __func__);

	/* Prepare Transmit Follow up response */
	nanXmitFollowupRsp.fwHeader.msgVersion = 1;
	nanXmitFollowupRsp.fwHeader.msgId = NAN_MSG_ID_TRANSMIT_FOLLOWUP_RSP;
	nanXmitFollowupRsp.fwHeader.msgLen =
		sizeof(struct NanTransmitFollowupRspMsg);
	nanXmitFollowupRsp.fwHeader.handle =
		pNanXmitFollowupRsp->fwHeader.handle;
	nanXmitFollowupRsp.fwHeader.transactionId =
		pNanXmitFollowupRsp->fwHeader.transactionId;
	nanXmitFollowupRsp.status = pNanXmitFollowupRsp->status;
	nanXmitFollowupRsp.value = 0;

	/*  Fill values of NanSubscribeServiceCancelRspMsg */
	skb = cfg80211_vendor_event_alloc(
		wiphy, wdev,
		sizeof(struct NanTransmitFollowupRspMsg) + NLMSG_HDRLEN,
		WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}
	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN,
			     sizeof(struct NanTransmitFollowupRspMsg),
			     &nanXmitFollowupRsp) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}
	cfg80211_vendor_event(skb, GFP_KERNEL);

	kfree(pvSetBuffer);
	return WLAN_STATUS_SUCCESS;
}

struct NanDataPathInitiatorNDPE g_ndpReqNDPE;

int mtk_cfg80211_vendor_nan(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int data_len)
{
	struct GLUE_INFO *prGlueInfo = NULL;
	struct sk_buff *skb = NULL;
	struct ADAPTER *prAdapter;

	struct _NanMsgHeader nanMsgHdr;
	struct _NanTlv outputTlv;
	u16 readLen = 0;
	u32 u4BufLen;
	u32 i4Status = -EINVAL;

	int remainingLen = (data_len - (sizeof(struct _NanMsgHeader)));

	if (!wiphy) {
		DBGLOG(NAN, ERROR, "wiphy error!\n");
		return -EINVAL;
	}
	if (!wdev) {
		DBGLOG(NAN, ERROR, "wdev error!\n");
		return -EINVAL;
	}

	if (data == NULL || data_len <= 0) {
		log_dbg(REQ, ERROR, "data error(len=%d)\n", data_len);
		return -EINVAL;
	}
	WIPHY_PRIV(wiphy, prGlueInfo);

	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error!\n");
		return -EINVAL;
	}

	prAdapter = prGlueInfo->prAdapter;

	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return -EINVAL;
	}

	prAdapter->fgIsNANfromHAL = TRUE;
	DBGLOG(NAN, LOUD, "NAN fgIsNANfromHAL set %u\n",
		prAdapter->fgIsNANfromHAL);

	dumpMemory8((uint8_t *)data, data_len);
	DBGLOG(INIT, ERROR, "DATA len from user %d\n", data_len);

	memcpy(&nanMsgHdr, (struct _NanMsgHeader *)data,
		sizeof(struct _NanMsgHeader));
	data += sizeof(struct _NanMsgHeader);

	dumpMemory8((uint8_t *)data, remainingLen);
	DBGLOG(INIT, ERROR, "nanMsgHdr.length %u, nanMsgHdr.msgId %d\n",
		nanMsgHdr.msgLen, nanMsgHdr.msgId);

	switch (nanMsgHdr.msgId) {
	case NAN_MSG_ID_ENABLE_REQ: {
		struct NanEnableRequest nanEnableReq;
		struct NanEnableRspMsg nanEnableRsp;

		kalMemZero(&nanEnableReq, sizeof(struct NanEnableRequest));
		kalMemZero(&nanEnableRsp, sizeof(struct NanEnableRspMsg));
		while ((remainingLen > 0) &&
		       (0 !=
			(readLen = nan_read_tlv((u8 *)data, &outputTlv)))) {
			switch (outputTlv.type) {
			case NAN_TLV_TYPE_CLUSTER_ID_LOW:
				memcpy(&nanEnableReq.cluster_low,
				       outputTlv.value, outputTlv.length);
				break;
			case NAN_TLV_TYPE_CLUSTER_ID_HIGH:
				memcpy(&nanEnableReq.cluster_high,
				       outputTlv.value, outputTlv.length);
				break;
			case NAN_TLV_TYPE_MASTER_PREFERENCE:
				memcpy(&nanEnableReq.master_pref,
				       outputTlv.value, outputTlv.length);
				break;
			default:
				break;
			}
			remainingLen -= readLen;
			data += readLen;
			memset(&outputTlv, 0, sizeof(outputTlv));
		}

		kalMemZero(&nanEnableReq, sizeof(struct NanEnableRequest));
		nanEnableReq.master_pref =
			prGlueInfo->prAdapter->rWifiVar.ucMasterPref;
		nanEnableReq.config_random_factor_force = 0;
		nanEnableReq.random_factor_force_val = 0;
		nanEnableReq.config_hop_count_force = 0;
		nanEnableReq.hop_count_force_val = 0;
		nanEnableReq.config_5g_channel =
			prGlueInfo->prAdapter->rWifiVar.ucConfig5gChannel;
		nanEnableReq.channel_5g_val =
			prGlueInfo->prAdapter->rWifiVar.ucChannel5gVal;

		nanEnableRsp.status = nanDevEnableRequest(prGlueInfo->prAdapter,
							  &nanEnableReq);

		memcpy(&nanEnableRsp.fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanEnableRspMsg));

		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}
		if (unlikely(nla_put_nohdr(skb, sizeof(struct NanEnableRspMsg),
					   &nanEnableRsp) < 0)) {
			kfree_skb(skb);
			return -EFAULT;
		}
		i4Status = kalIoctl(prGlueInfo, wlanoidNANEnableRsp,
				    (void *)&nanEnableRsp,
				    sizeof(struct NanEnableRequest), FALSE,
				    FALSE, FALSE, &u4BufLen);
		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, ERROR, "kalIoctl failed\n");
			return -EFAULT;
		}
		break;
	}
	case NAN_MSG_ID_DISABLE_REQ: {
		struct NanDisableRspMsg nanDisableRsp;

		kalMemZero(&nanDisableRsp, sizeof(struct NanDisableRspMsg));
		nanDisableRsp.status =
			nanDevDisableRequest(prGlueInfo->prAdapter);

		memcpy(&nanDisableRsp.fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanDisableRspMsg));

		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}
		if (unlikely(nla_put_nohdr(skb, sizeof(struct NanDisableRspMsg),
					   &nanDisableRsp) < 0)) {
			kfree_skb(skb);
			return -EFAULT;
		}
		i4Status = kalIoctl(prGlueInfo, wlanoidNANDisableRsp,
				    (void *)&nanDisableRsp,
				    sizeof(struct NanDisableRspMsg), FALSE,
				    FALSE, FALSE, &u4BufLen);
		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, ERROR, "kalIoctl failed\n");
			return -EFAULT;
		}
		break;
	}
	case NAN_MSG_ID_CONFIGURATION_REQ: {
		struct NanConfigRequest nanConfigReq;
		struct NanConfigRspMsg nanConfigRsp;

		kalMemZero(&nanConfigReq, sizeof(struct NanEnableRequest));
		kalMemZero(&nanConfigRsp, sizeof(struct NanEnableRspMsg));

		while ((remainingLen > 0) &&
		       (0 !=
			(readLen = nan_read_tlv((u8 *)data, &outputTlv)))) {
			switch (outputTlv.type) {
			case NAN_TLV_TYPE_MASTER_PREFERENCE:
				memcpy(&nanConfigReq.master_pref,
				       outputTlv.value, outputTlv.length);
				nanDevSetMasterPreference(
					prGlueInfo->prAdapter,
					nanConfigReq.master_pref);
				break;
			default:
				break;
			}
			remainingLen -= readLen;
			data += readLen;
			memset(&outputTlv, 0, sizeof(outputTlv));
		}

		memcpy(&nanConfigRsp.fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanConfigRspMsg));

		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}
		if (unlikely(nla_put_nohdr(skb, sizeof(struct NanConfigRspMsg),
					   &nanConfigRsp) < 0)) {
			kfree_skb(skb);
			return -EFAULT;
		}
		break;
	}
	case NAN_MSG_ID_CAPABILITIES_REQ: {
		struct NanCapabilitiesRspMsg nanCapabilitiesRsp;

		memcpy(&nanCapabilitiesRsp.fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanCapabilitiesRspMsg));

		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}
		if (unlikely(nla_put_nohdr(skb,
					   sizeof(struct NanCapabilitiesRspMsg),
					   &nanCapabilitiesRsp) < 0)) {
			kfree_skb(skb);
			return -EFAULT;
		}
		i4Status = kalIoctl(prGlueInfo, wlanoidGetNANCapabilitiesRsp,
				    (void *)&nanCapabilitiesRsp,
				    sizeof(struct NanCapabilitiesRspMsg), FALSE,
				    FALSE, FALSE, &u4BufLen);
		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, ERROR, "kalIoctl failed\n");
			return -EFAULT;
		}
		DBGLOG(INIT, ERROR, "i4Status = %u\n", i4Status);

		break;
	}
	case NAN_MSG_ID_PUBLISH_SERVICE_REQ: {
		struct NanPublishRequest *pNanPublishReq = NULL;
		struct NanPublishServiceRspMsg *pNanPublishRsp = NULL;
		uint16_t publish_id = 0;
		uint8_t ucCipherType = 0;

		DBGLOG(REQ, INFO, "IN case NAN_MSG_ID_PUBLISH_SERVICE_REQ\n");

		pNanPublishReq =
			kmalloc(sizeof(struct NanPublishRequest), GFP_ATOMIC);
		pNanPublishRsp = kmalloc(sizeof(struct NanPublishServiceRspMsg),
					 GFP_ATOMIC);

		kalMemZero(pNanPublishReq, sizeof(struct NanPublishRequest));
		kalMemZero(pNanPublishRsp,
			   sizeof(struct NanPublishServiceRspMsg));

		/* Mapping publish req related parameters */
		readLen = nanMapPublishReqParams((u16 *)data, pNanPublishReq);
		remainingLen -= readLen;
		data += readLen;

		dumpMemory8((uint8_t *)data, remainingLen);

		while ((remainingLen > 0) &&
		       (0 !=
			(readLen = nan_read_tlv((u8 *)data, &outputTlv)))) {
			switch (outputTlv.type) {
			case NAN_TLV_TYPE_SERVICE_NAME:
				memcpy(pNanPublishReq->service_name,
				       outputTlv.value, outputTlv.length);
				pNanPublishReq->service_name_len =
					outputTlv.length;
				DBGLOG(INIT, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(INIT, INFO, "outputTlv.length:%u\n",
				       outputTlv.length);
				DBGLOG(INIT, INFO,
				       "pNanPublishReq->service_name_len:%u\n",
				       pNanPublishReq->service_name_len);
				break;
			case NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO:
				memcpy(pNanPublishReq->service_specific_info,
				       outputTlv.value, outputTlv.length);
				pNanPublishReq->service_specific_info_len =
					outputTlv.length;
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length:%u\n",
				       outputTlv.length);
				DBGLOG(REQ, INFO,
				       "pNanPublishReq->service_specific_info_len:%u\n",
				       pNanPublishReq
					       ->service_specific_info_len);
				break;
			case NAN_TLV_TYPE_RX_MATCH_FILTER:
				memcpy(pNanPublishReq->rx_match_filter,
				       outputTlv.value, outputTlv.length);
				pNanPublishReq->rx_match_filter_len =
					outputTlv.length;
				DBGLOG(REQ, INFO,
				       "RX_MATCH_FILTER outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length:%u\n",
				       outputTlv.length);
				DBGLOG(REQ, INFO,
				       "pNanPublishReq->rx_match_filter_len:%u\n",
				       pNanPublishReq->rx_match_filter_len);
				dumpMemory8(
					(uint8_t *)
						pNanPublishReq->rx_match_filter,
					pNanPublishReq->rx_match_filter_len);
				break;
			case NAN_TLV_TYPE_TX_MATCH_FILTER:
				memcpy(pNanPublishReq->tx_match_filter,
				       outputTlv.value, outputTlv.length);
				pNanPublishReq->tx_match_filter_len =
					outputTlv.length;
				DBGLOG(REQ, INFO,
				       "TX_MATCH_FILTER outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length:%u\n",
				       outputTlv.length);
				DBGLOG(REQ, INFO,
				       "pNanPublishReq->tx_match_filter_len:%u\n",
				       pNanPublishReq->tx_match_filter_len);
				dumpMemory8(
					(uint8_t *)
						pNanPublishReq->tx_match_filter,
					pNanPublishReq->tx_match_filter_len);
				break;
			case NAN_TLV_TYPE_NAN_SERVICE_ACCEPT_POLICY:
				pNanPublishReq->service_responder_policy =
					*(outputTlv.value);
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length:%u\n",
				       outputTlv.length);
				break;
			case NAN_TLV_TYPE_NAN_CSID:
				pNanPublishReq->cipher_type =
					*(outputTlv.value);
				break;
			case NAN_TLV_TYPE_NAN_PMK:
				memcpy(pNanPublishReq->key_info.body.pmk_info
					       .pmk,
				       outputTlv.value, outputTlv.length);
				pNanPublishReq->key_info.body.pmk_info.pmk_len =
					outputTlv.length;
				break;
			case NAN_TLV_TYPE_NAN_PASSPHRASE:
				memcpy(pNanPublishReq->key_info.body
					       .passphrase_info.passphrase,
				       outputTlv.value, outputTlv.length);
				pNanPublishReq->key_info.body.passphrase_info
					.passphrase_len = outputTlv.length;
				break;
			case NAN_TLV_TYPE_SDEA_CTRL_PARAMS:
				nanMapSdeaCtrlParams(
					(u32 *)outputTlv.value,
					&pNanPublishReq->sdea_params);
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length:%u\n",
				       outputTlv.length);
				break;
			case NAN_TLV_TYPE_NAN_RANGING_CFG:
				nanMapRangingConfigParams(
					(u32 *)outputTlv.value,
					&pNanPublishReq->ranging_cfg);
				break;
			case NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO:
				pNanPublishReq->sdea_service_specific_info[0] =
					*(outputTlv.value);
				pNanPublishReq->sdea_service_specific_info_len =
					outputTlv.length;
				break;
			case NAN_TLV_TYPE_NAN20_RANGING_REQUEST:
				nanMapNan20RangingReqParams(
					(u32 *)outputTlv.value,
					&pNanPublishReq->range_response_cfg);
				break;
			default:
				break;
			}
			remainingLen -= readLen;
			data += readLen;
			memset(&outputTlv, 0, sizeof(outputTlv));
		}

		/* Publish response message */
		memcpy(&pNanPublishRsp->fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanPublishServiceRspMsg));

		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}

		if (unlikely(nla_put_nohdr(
				     skb,
				     sizeof(struct NanPublishServiceRspMsg),
				     pNanPublishRsp) < 0)) {
			kfree_skb(skb);
			DBGLOG(REQ, ERROR, "Fail send reply\n");
			return -EFAULT;
		}

		/* return publish ID */
		publish_id = (uint16_t)nanPublishRequest(prGlueInfo->prAdapter,
							pNanPublishReq);
		pNanPublishRsp->fwHeader.handle = publish_id;
		DBGLOG(REQ, INFO,
		       "pNanPublishRsp->fwHeader.handle %u, publish_id : %u\n",
		       pNanPublishRsp->fwHeader.handle, publish_id);

		if (pNanPublishReq->sdea_params.security_cfg) {
			/* Fixme: supply a cipher suite list */
			ucCipherType = pNanPublishReq->cipher_type;
			nanCmdAddCsid(
				prGlueInfo->prAdapter,
				publish_id,
				1,
				&ucCipherType);
			nanSetPublishPmkid(
				prGlueInfo->prAdapter,
				pNanPublishReq);
			if (pNanPublishReq->scid_len) {
				if (pNanPublishReq->scid_len
						> NAN_SCID_DEFAULT_LEN)
					pNanPublishReq->scid_len
						= NAN_SCID_DEFAULT_LEN;
				nanCmdManageScid(
					prGlueInfo->prAdapter,
					TRUE,
					publish_id,
					pNanPublishReq->scid);
			}
		}

		i4Status = kalIoctl(prGlueInfo, wlanoidNanPublishRsp,
				    (void *)pNanPublishRsp,
				    sizeof(struct NanPublishServiceRspMsg),
				    FALSE, FALSE, FALSE, &u4BufLen);
		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, ERROR, "kalIoctl failed\n");
			return -EFAULT;
		}

		kfree(pNanPublishReq);
		break;
	}
	case NAN_MSG_ID_PUBLISH_SERVICE_CANCEL_REQ: {
		uint32_t rStatus;
		struct NanPublishCancelRequest *pNanPublishCancelReq = NULL;
		struct NanPublishServiceCancelRspMsg *pNanPublishCancelRsp =
			NULL;

		pNanPublishCancelReq = kmalloc(
			sizeof(struct NanPublishCancelRequest), GFP_ATOMIC);
		pNanPublishCancelRsp =
			kmalloc(sizeof(struct NanPublishServiceCancelRspMsg),
				GFP_ATOMIC);

		DBGLOG(REQ, INFO, "Enter CANCEL Publish Request\n");
		pNanPublishCancelReq->publish_id = nanMsgHdr.handle;

		DBGLOG(REQ, INFO, "PID %d\n", pNanPublishCancelReq->publish_id);
		rStatus = nanCancelPublishRequest(prGlueInfo->prAdapter,
						  pNanPublishCancelReq);

		/* Prepare for command reply */
		memcpy(&pNanPublishCancelRsp->fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanPublishServiceCancelRspMsg));
		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}
		if (unlikely(nla_put_nohdr(
				     skb, sizeof(struct
						 NanPublishServiceCancelRspMsg),
				     pNanPublishCancelRsp) < 0)) {
			kfree_skb(skb);
			return -EFAULT;
		}

		if (rStatus != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, INFO, "CANCEL Publish Error %x\n", rStatus);
			pNanPublishCancelRsp->status = NAN_I_STATUS_DE_FAILURE;
		} else {
			DBGLOG(REQ, INFO, "CANCEL Publish Success %x\n",
			       rStatus);
			pNanPublishCancelRsp->status = NAN_I_STATUS_SUCCESS;
		}

		i4Status =
			kalIoctl(prGlueInfo, wlanoidNANCancelPublishRsp,
				 (void *)pNanPublishCancelRsp,
				 sizeof(struct NanPublishServiceCancelRspMsg),
				 FALSE, FALSE, FALSE, &u4BufLen);
		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, ERROR, "kalIoctl failed\n");
			return -EFAULT;
		}

		kfree(pNanPublishCancelReq);
		break;
	}
	case NAN_MSG_ID_SUBSCRIBE_SERVICE_REQ: {
		struct NanSubscribeRequest *pNanSubscribeReq = NULL;
		struct NanSubscribeServiceRspMsg *pNanSubscribeRsp = NULL;
		bool fgRangingCFG = FALSE;
		bool fgRangingREQ = FALSE;
		uint16_t Subscribe_id = 0;
		int i = 0;

		DBGLOG(REQ, INFO, "In NAN_MSG_ID_SUBSCRIBE_SERVICE_REQ\n");
		dumpMemory8((uint8_t *)data, remainingLen);

		pNanSubscribeReq =
			kmalloc(sizeof(struct NanSubscribeRequest), GFP_ATOMIC);
		pNanSubscribeRsp = kmalloc(
			sizeof(struct NanSubscribeServiceRspMsg), GFP_ATOMIC);

		kalMemZero(pNanSubscribeReq,
			   sizeof(struct NanSubscribeRequest));
		kalMemZero(pNanSubscribeRsp,
			   sizeof(struct NanSubscribeServiceRspMsg));
		/* Mapping subscribe req related parameters */
		readLen =
			nanMapSubscribeReqParams((u16 *)data, pNanSubscribeReq);
		remainingLen -= readLen;
		data += readLen;

		dumpMemory8((uint8_t *)data, remainingLen);

		while ((remainingLen > 0) &&
		       (0 !=
			(readLen = nan_read_tlv((u8 *)data, &outputTlv)))) {
			switch (outputTlv.type) {
			case NAN_TLV_TYPE_SERVICE_NAME:
				memcpy(pNanSubscribeReq->service_name,
				       outputTlv.value, outputTlv.length);
				pNanSubscribeReq->service_name_len =
					outputTlv.length;
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length: %u\n",
				       outputTlv.length);
				DBGLOG(REQ, INFO, "outputTlv.value: %s\n",
				       outputTlv.value);
				DBGLOG(REQ, INFO,
				       "pNanSubscribeReq->service_name_len: %u\n",
				       pNanSubscribeReq->service_name_len);
				DBGLOG(REQ, INFO,
				       "pNanSubscribeReq->service_name:%s\n",
				       pNanSubscribeReq->service_name);
				break;
			case NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO:
				memcpy(pNanSubscribeReq->service_specific_info,
				       outputTlv.value, outputTlv.length);
				pNanSubscribeReq->service_specific_info_len =
					outputTlv.length;
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length: %u\n",
				       outputTlv.length);
				DBGLOG(REQ, INFO, "outputTlv.value: %s\n",
				       outputTlv.value);
				DBGLOG(REQ, INFO,
				       "pNanSubscribeReq->service_specific_info_len: %u\n",
				       pNanSubscribeReq
					       ->service_specific_info_len);
				DBGLOG(REQ, INFO,
				       "pNanSubscribeReq->service_specific_info:%s\n",
				       pNanSubscribeReq->service_specific_info);
				break;
			case NAN_TLV_TYPE_RX_MATCH_FILTER:
				memcpy(pNanSubscribeReq->rx_match_filter,
				       outputTlv.value, outputTlv.length);
				pNanSubscribeReq->rx_match_filter_len =
					outputTlv.length;
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length: %u\n",
				       outputTlv.length);
				DBGLOG(REQ, INFO, "outputTlv.value: %s\n",
				       outputTlv.value);
				DBGLOG(REQ, INFO,
				       "pNanSubscribeReq->rx_match_filter_len: %u\n",
				       pNanSubscribeReq->rx_match_filter_len);
				DBGLOG(REQ, INFO,
				       "pNanSubscribeReq->rx_match_filter:%s\n",
				       pNanSubscribeReq->rx_match_filter);
				dumpMemory8((uint8_t *)pNanSubscribeReq
						    ->rx_match_filter,
					    outputTlv.length);
				break;
			case NAN_TLV_TYPE_TX_MATCH_FILTER:
				memcpy(pNanSubscribeReq->tx_match_filter,
				       outputTlv.value, outputTlv.length);
				pNanSubscribeReq->tx_match_filter_len =
					outputTlv.length;
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length: %u\n",
				       outputTlv.length);
				DBGLOG(REQ, INFO, "outputTlv.value: %s\n",
				       outputTlv.value);
				DBGLOG(REQ, INFO,
				       "pNanSubscribeReq->tx_match_filter_len: %u\n",
				       pNanSubscribeReq->tx_match_filter_len);
				DBGLOG(REQ, INFO,
				       "pNanSubscribeReq->tx_match_filter:%s\n",
				       pNanSubscribeReq->tx_match_filter);
				dumpMemory8((uint8_t *)pNanSubscribeReq
						    ->tx_match_filter,
					    outputTlv.length);
				break;
			case NAN_TLV_TYPE_MAC_ADDRESS:
				/* Get column neumbers */
				memcpy(pNanSubscribeReq->intf_addr[i],
				       outputTlv.value, outputTlv.length);
				i++;
				break;
			case NAN_TLV_TYPE_NAN_CSID:
				pNanSubscribeReq->cipher_type =
					*(outputTlv.value);
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length: %u\n",
				       outputTlv.length);
				break;
			case NAN_TLV_TYPE_NAN_PMK:
				memcpy(pNanSubscribeReq->key_info.body.pmk_info
					       .pmk,
				       outputTlv.value, outputTlv.length);
				pNanSubscribeReq->key_info.body.pmk_info
					.pmk_len = outputTlv.length;
				break;
			case NAN_TLV_TYPE_NAN_PASSPHRASE:
				memcpy(pNanSubscribeReq->key_info.body
					       .passphrase_info.passphrase,
				       outputTlv.value, outputTlv.length);
				pNanSubscribeReq->key_info.body.passphrase_info
					.passphrase_len = outputTlv.length;
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length: %u\n",
				       outputTlv.length);
				break;
			case NAN_TLV_TYPE_SDEA_CTRL_PARAMS:
				nanMapSdeaCtrlParams(
					(u32 *)outputTlv.value,
					&pNanSubscribeReq->sdea_params);
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length: %u\n",
				       outputTlv.length);
				break;
			case NAN_TLV_TYPE_NAN_RANGING_CFG:
				fgRangingCFG = TRUE;
				DBGLOG(NAN, INFO, "fgRangingCFG %d\n",
					fgRangingCFG);
				nanMapRangingConfigParams(
					(u32 *)outputTlv.value,
					&pNanSubscribeReq->ranging_cfg);
				break;
			case NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO:
				memcpy(pNanSubscribeReq
					->sdea_service_specific_info,
				       outputTlv.value, outputTlv.length);
				pNanSubscribeReq
					->sdea_service_specific_info_len =
					outputTlv.length;
				DBGLOG(REQ, INFO, "outputTlv.type:%u\n",
				       outputTlv.type);
				DBGLOG(REQ, INFO, "outputTlv.length: %u\n",
				       outputTlv.length);
				break;
			case NAN_TLV_TYPE_NAN20_RANGING_REQUEST:
				fgRangingREQ = TRUE;
				DBGLOG(NAN, INFO, "fgRangingREQ %d\n",
					fgRangingREQ);
				nanMapNan20RangingReqParams(
					(u32 *)outputTlv.value,
					&pNanSubscribeReq->range_response_cfg);
				break;
			default:
				break;
			}
			remainingLen -= readLen;
			data += readLen;
			memset(&outputTlv, 0, sizeof(outputTlv));
		}

		/* Prepare command reply of Subscriabe response */
		memcpy(&pNanSubscribeRsp->fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanSubscribeServiceRspMsg));

		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}
		if (unlikely(nla_put_nohdr(
				     skb,
				     sizeof(struct NanSubscribeServiceRspMsg),
				     pNanSubscribeRsp) < 0)) {
			kfree_skb(skb);
			return -EFAULT;
		}
		/* Ranging */
		if (fgRangingCFG && fgRangingREQ) {

			struct NanRangeRequest *rgreq = NULL;
			uint16_t rgId = 0;
			uint32_t rStatus;

			rgreq = kmalloc(sizeof(struct NanRangeRequest),
				GFP_ATOMIC);
			kalMemZero(rgreq, sizeof(struct NanRangeRequest));

			memcpy(&rgreq->peer_addr,
				&pNanSubscribeReq->range_response_cfg.peer_addr,
				NAN_MAC_ADDR_LEN);
			memcpy(&rgreq->ranging_cfg,
				&pNanSubscribeReq->ranging_cfg,
				sizeof(struct NanRangingCfg));
			rgreq->range_id =
			pNanSubscribeReq->range_response_cfg
				.requestor_instance_id;
			DBGLOG(NAN, INFO, MACSTR
				" id %d reso %d intev %d indicat %d ING CM %d ENG CM %d\n",
				MAC2STR(rgreq->peer_addr),
				rgreq->range_id,
				rgreq->ranging_cfg.ranging_resolution,
				rgreq->ranging_cfg.ranging_interval_msec,
				rgreq->ranging_cfg.config_ranging_indications,
				rgreq->ranging_cfg.distance_ingress_cm,
				rgreq->ranging_cfg.distance_egress_cm);
			rStatus =
			nanRangingRequest(prGlueInfo->prAdapter, &rgId, rgreq);

			pNanSubscribeRsp->fwHeader.handle = rgId;
			i4Status = kalIoctl(prGlueInfo, wlanoidNanSubscribeRsp,
				       (void *)pNanSubscribeRsp,
				       sizeof(struct NanSubscribeServiceRspMsg),
				       FALSE, FALSE, FALSE, &u4BufLen);
			if (i4Status != WLAN_STATUS_SUCCESS) {
				DBGLOG(REQ, ERROR, "kalIoctl failed\n");
				return -EFAULT;
			}
			kfree(rgreq);
			kfree(pNanSubscribeReq);
			break;

		}

		prAdapter->fgIsNANfromHAL = TRUE;

		/* return subscribe ID */
		Subscribe_id = (uint16_t)nanSubscribeRequest(
			prGlueInfo->prAdapter, pNanSubscribeReq);
		pNanSubscribeRsp->fwHeader.handle = Subscribe_id;

		DBGLOG(REQ, ERROR,
		       "Subscribe_id:%u, pNanSubscribeRsp->fwHeader.handle:%u\n",
		       Subscribe_id, pNanSubscribeRsp->fwHeader.handle);
		i4Status = kalIoctl(prGlueInfo, wlanoidNanSubscribeRsp,
				    (void *)pNanSubscribeRsp,
				    sizeof(struct NanSubscribeServiceRspMsg),
				    FALSE, FALSE, FALSE, &u4BufLen);
		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, ERROR, "kalIoctl failed\n");
			return -EFAULT;
		}

		kfree(pNanSubscribeReq);
		break;
	}
	case NAN_MSG_ID_SUBSCRIBE_SERVICE_CANCEL_REQ: {
		uint32_t rStatus;
		struct NanSubscribeCancelRequest *pNanSubscribeCancelReq = NULL;
		struct NanSubscribeServiceCancelRspMsg *pNanSubscribeCancelRsp =
			NULL;

		pNanSubscribeCancelReq = kmalloc(
			sizeof(struct NanSubscribeCancelRequest), GFP_ATOMIC);
		pNanSubscribeCancelRsp =
			kmalloc(sizeof(struct NanSubscribeServiceCancelRspMsg),
				GFP_ATOMIC);
		kalMemZero(pNanSubscribeCancelReq,
			   sizeof(struct NanSubscribeCancelRequest));
		kalMemZero(pNanSubscribeCancelRsp,
			   sizeof(struct NanSubscribeServiceCancelRspMsg));

		DBGLOG(REQ, INFO, "Enter CANCEL Subscribe Request\n");
		pNanSubscribeCancelReq->subscribe_id = nanMsgHdr.handle;

		DBGLOG(REQ, INFO, "PID %d\n",
		       pNanSubscribeCancelReq->subscribe_id);
		rStatus = nanCancelSubscribeRequest(prGlueInfo->prAdapter,
						    pNanSubscribeCancelReq);

		/* Prepare Cancel Subscribe command reply message */
		memcpy(&pNanSubscribeCancelRsp->fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));

		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanSubscribeServiceCancelRspMsg));
		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}
		if (unlikely(nla_put_nohdr(
				     skb,
				     sizeof(struct
					    NanSubscribeServiceCancelRspMsg),
				     pNanSubscribeCancelRsp) < 0)) {
			kfree_skb(skb);
			return -EFAULT;
		}

		if (rStatus != WLAN_STATUS_SUCCESS) {
			DBGLOG(NAN, ERROR, "CANCEL Publish Error %X\n",
			       rStatus);
			pNanSubscribeCancelRsp->status =
				NAN_I_STATUS_DE_FAILURE;
		} else {
			DBGLOG(NAN, INFO, "CANCEL Publish Success %X\n",
			       rStatus);
			pNanSubscribeCancelRsp->status = NAN_I_STATUS_SUCCESS;
		}

		i4Status =
			kalIoctl(prGlueInfo, wlanoidNANCancelSubscribeRsp,
				 (void *)pNanSubscribeCancelRsp,
				 sizeof(struct NanSubscribeServiceCancelRspMsg),
				 FALSE, FALSE, FALSE, &u4BufLen);
		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, ERROR, "kalIoctl failed\n");
			return -EFAULT;
		}

		kfree(pNanSubscribeCancelReq);
		break;
	}
	case NAN_MSG_ID_TRANSMIT_FOLLOWUP_REQ: {
		uint32_t rStatus;
		struct NanTransmitFollowupRequest *pNanXmitFollowupReq = NULL;
		struct NanTransmitFollowupRspMsg *pNanXmitFollowupRsp = NULL;

		pNanXmitFollowupReq = kmalloc(
			sizeof(struct NanTransmitFollowupRequest), GFP_ATOMIC);
		pNanXmitFollowupRsp = kmalloc(
			sizeof(struct NanTransmitFollowupRspMsg), GFP_ATOMIC);
		kalMemZero(pNanXmitFollowupReq,
			   sizeof(struct NanTransmitFollowupRequest));
		kalMemZero(pNanXmitFollowupRsp,
			   sizeof(struct NanTransmitFollowupRspMsg));

		DBGLOG(REQ, INFO, "Enter Transmit follow up Request\n");

		/* Mapping publish req related parameters */
		readLen = nanMapFollowupReqParams((u32 *)data,
						  pNanXmitFollowupReq);
		remainingLen -= readLen;
		data += readLen;
		pNanXmitFollowupReq->publish_subscribe_id = nanMsgHdr.handle;

		while ((remainingLen > 0) &&
		       (0 !=
			(readLen = nan_read_tlv((u8 *)data, &outputTlv)))) {
			switch (outputTlv.type) {
			case NAN_TLV_TYPE_MAC_ADDRESS:
				memcpy(pNanXmitFollowupReq->addr,
				       outputTlv.value, outputTlv.length);
				break;
			case NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO:
				memcpy(pNanXmitFollowupReq
					       ->service_specific_info,
				       outputTlv.value, outputTlv.length);
				pNanXmitFollowupReq->service_specific_info_len =
					outputTlv.length;
				break;
			case NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO:
				memcpy(pNanXmitFollowupReq
					       ->sdea_service_specific_info,
				       outputTlv.value, outputTlv.length);
				pNanXmitFollowupReq
					->sdea_service_specific_info_len =
					outputTlv.length;
				break;
			default:
				break;
			}
			remainingLen -= readLen;
			data += readLen;
			memset(&outputTlv, 0, sizeof(outputTlv));
		}

		/* Follow up Command reply message */
		memcpy(&pNanXmitFollowupRsp->fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanTransmitFollowupRspMsg));

		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}

		if (unlikely(nla_put_nohdr(
				     skb,
				     sizeof(struct NanTransmitFollowupRspMsg),
				     pNanXmitFollowupRsp) < 0)) {
			kfree_skb(skb);
			DBGLOG(REQ, ERROR, "Fail send reply\n");
			return -EFAULT;
		}

		rStatus = nanTransmitRequest(prGlueInfo->prAdapter,
					     pNanXmitFollowupReq);
		if (rStatus != WLAN_STATUS_SUCCESS)
			pNanXmitFollowupRsp->status = NAN_I_STATUS_DE_FAILURE;
		else
			pNanXmitFollowupRsp->status = NAN_I_STATUS_SUCCESS;

		i4Status = kalIoctl(prGlueInfo, wlanoidNANFollowupRsp,
				    (void *)pNanXmitFollowupRsp,
				    sizeof(struct NanTransmitFollowupRspMsg),
				    FALSE, FALSE, FALSE, &u4BufLen);
		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(REQ, ERROR, "kalIoctl failed\n");
			return -EFAULT;
		}

		kfree(pNanXmitFollowupReq);
		break;
	}
	case NAN_MSG_ID_BEACON_SDF_REQ: {
		uint32_t rStatus = WLAN_STATUS_SUCCESS;
		u16 vsa_length = 0;
		u32 *pXmitVSAparms = NULL;
		struct NanTransmitVendorSpecificAttribute *pNanXmitVSAttrReq =
			NULL;
		struct NanBeaconSdfPayloadRspMsg *pNanBcnSdfVSARsp = NULL;

		DBGLOG(NAN, INFO, "Enter Beacon SDF Request.\n");

		pNanXmitVSAttrReq = kmalloc(
			sizeof(struct NanTransmitVendorSpecificAttribute),
			GFP_ATOMIC);
		pNanBcnSdfVSARsp = kmalloc(
			sizeof(struct NanBeaconSdfPayloadRspMsg), GFP_ATOMIC);

		while ((remainingLen > 0) &&
		       (0 !=
			(readLen = nan_read_tlv((u8 *)data, &outputTlv)))) {
			switch (outputTlv.type) {
			case NAN_TLV_TYPE_VENDOR_SPECIFIC_ATTRIBUTE_TRANSMIT:
				pXmitVSAparms = (u32 *)outputTlv.value;
				pNanXmitVSAttrReq->payload_transmit_flag =
					(u8)(*pXmitVSAparms & BIT(0));
				pNanXmitVSAttrReq->tx_in_discovery_beacon =
					(u8)(*pXmitVSAparms & BIT(1));
				pNanXmitVSAttrReq->tx_in_sync_beacon =
					(u8)(*pXmitVSAparms & BIT(2));
				pNanXmitVSAttrReq->tx_in_service_discovery =
					(u8)(*pXmitVSAparms & BIT(3));
				pNanXmitVSAttrReq->vendor_oui =
					*pXmitVSAparms & BITS(8, 31);
				outputTlv.value += 4;

				vsa_length = outputTlv.length - sizeof(u32);
				memcpy(pNanXmitVSAttrReq->vsa, outputTlv.value,
				       vsa_length);
				pNanXmitVSAttrReq->vsa_len = vsa_length;
				break;
			default:
				break;
			}
			remainingLen -= readLen;
			data += readLen;
			memset(&outputTlv, 0, sizeof(outputTlv));
		}

		/* To be implement
		 * Beacon SDF VSA request.................................
		 * rStatus = ;
		 */

		/* Prepare Beacon Sdf Payload Response */
		memcpy(&pNanBcnSdfVSARsp->fwHeader, &nanMsgHdr,
		       sizeof(struct _NanMsgHeader));
		pNanBcnSdfVSARsp->fwHeader.msgId = NAN_MSG_ID_BEACON_SDF_RSP;
		pNanBcnSdfVSARsp->fwHeader.msgLen =
			sizeof(struct NanBeaconSdfPayloadRspMsg);

		if (rStatus != WLAN_STATUS_FAILURE)
			pNanBcnSdfVSARsp->status = NAN_I_STATUS_SUCCESS;
		else
			pNanBcnSdfVSARsp->status = NAN_I_STATUS_INVALID_HANDLE;

		skb = cfg80211_vendor_cmd_alloc_reply_skb(
			wiphy, sizeof(struct NanBeaconSdfPayloadRspMsg));

		if (!skb) {
			DBGLOG(REQ, ERROR, "Allocate skb failed\n");
			return -ENOMEM;
		}
		if (unlikely(nla_put_nohdr(
				     skb,
				     sizeof(struct NanBeaconSdfPayloadRspMsg),
				     pNanBcnSdfVSARsp) < 0)) {
			kfree_skb(skb);
			return -EFAULT;
		}

		kfree(pNanXmitVSAttrReq);
		kfree(pNanBcnSdfVSARsp);

		break;
	}
	case NAN_MSG_ID_TESTMODE_REQ:
	{
		struct NanDebugParams *pNanDebug = NULL;

		pNanDebug = kmalloc(sizeof(struct NanDebugParams), GFP_ATOMIC);
		kalMemZero(pNanDebug, sizeof(struct NanDebugParams));
		DBGLOG(REQ, ERROR, "NAN_MSG_ID_TESTMODE_REQ\n");

		while ((remainingLen > 0) &&
			(0 != (readLen = nan_read_tlv((u8 *)data,
			&outputTlv)))) {
			DBGLOG(REQ, ERROR, "outputTlv.type= %d\n",
				outputTlv.type);
			if (outputTlv.type ==
				NAN_TLV_TYPE_TESTMODE_GENERIC_CMD) {
				memcpy(pNanDebug, outputTlv.value,
					outputTlv.length);
				switch (pNanDebug->cmd) {
				case NAN_TEST_MODE_CMD_DISABLE_NDPE:
					g_ndpReqNDPE.fgEnNDPE = TRUE;
					g_ndpReqNDPE.ucNDPEAttrPresent =
						pNanDebug->
						debug_cmd_data[0];
					DBGLOG(REQ, ERROR,
						"NAN_TEST_MODE_CMD_DISABLE_NDPE: fgEnNDPE = %d\n",
						g_ndpReqNDPE.fgEnNDPE);
					break;
				default:
					break;
				}
			} else {
				DBGLOG(REQ, ERROR,
					"Testmode invalid TLV type\n");
			}
			remainingLen -= readLen;
			data += readLen;
			memset(&outputTlv, 0, sizeof(outputTlv));
		}

		kfree(pNanDebug);
		return 0;
	}
	default:
		return -EOPNOTSUPP;
	}

	return cfg80211_vendor_cmd_reply(skb);
}

/* Indication part */
int
mtk_cfg80211_vendor_event_nan_event_indication(IN struct ADAPTER *prAdapter,
					       uint8_t ucEventType,
					       uint8_t *pcuEvtBuf)
{
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	struct NanEventIndMsg *prNanEventInd;
	uint8_t *pucNanEventInd = pcuEvtBuf;
	uint16_t u2EventType;
	uint8_t *tlvs = NULL;
	size_t message_len = 0;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	/*Final length includes all TLVs*/
	message_len = sizeof(struct _NanMsgHeader) +
		SIZEOF_TLV_HDR + MAC_ADDR_LEN;

	prNanEventInd = kalMemAlloc(message_len, VIR_MEM_TYPE);

	prNanEventInd->fwHeader.msgVersion = 1;
	prNanEventInd->fwHeader.msgId = NAN_MSG_ID_DE_EVENT_IND;
	prNanEventInd->fwHeader.msgLen = message_len;
	prNanEventInd->fwHeader.handle = 0;
	prNanEventInd->fwHeader.transactionId = 0;

	tlvs = prNanEventInd->ptlv;

	if (ucEventType == NAN_EVENT_ID_DISC_MAC_ADDR)
		u2EventType = NAN_TLV_TYPE_EVENT_SELF_STATION_MAC_ADDRESS;
	else if (ucEventType == NAN_EVENT_ID_STARTED_CLUSTER)
		u2EventType = NAN_TLV_TYPE_EVENT_STARTED_CLUSTER;
	else if (ucEventType == NAN_EVENT_ID_JOINED_CLUSTER)
		u2EventType = NAN_TLV_TYPE_EVENT_JOINED_CLUSTER;
	else {
		kalMemFree(prNanEventInd, VIR_MEM_TYPE, message_len);
		return WLAN_STATUS_SUCCESS;
	}

	/* Add TLV datas */
	tlvs = nanAddTlv(u2EventType, MAC_ADDR_LEN, pucNanEventInd, tlvs);

	/* Fill skb and send to kernel by nl80211 */
	skb = cfg80211_vendor_event_alloc(wiphy, wdev,
					  message_len + NLMSG_HDRLEN,
					  WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		kalMemFree(prNanEventInd, VIR_MEM_TYPE, message_len);
		return -ENOMEM;
	}
	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN, message_len,
			     prNanEventInd) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kalMemFree(prNanEventInd, VIR_MEM_TYPE, message_len);
		kfree_skb(skb);
		return -EFAULT;
	}
	cfg80211_vendor_event(skb, GFP_KERNEL);
	kalMemFree(prNanEventInd, VIR_MEM_TYPE, message_len);

	return WLAN_STATUS_SUCCESS;
}

/* Indication part */
int
mtk_cfg80211_vendor_event_nan_replied_indication(IN struct ADAPTER *prAdapter,
						 uint8_t *pcuEvtBuf)
{
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	struct NAN_REPLIED_EVENT *prRepliedEvt = NULL;
	struct NanPublishRepliedIndMsg *prNanPubRepliedInd;
	uint8_t *tlvs = NULL;
	size_t message_len = 0;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	prRepliedEvt = (struct NAN_REPLIED_EVENT *)pcuEvtBuf;

	/* Final length includes all TLVs */
	message_len = sizeof(struct _NanMsgHeader) +
		      sizeof(struct _NanPublishRepliedIndParams) +
		      ((SIZEOF_TLV_HDR) + MAC_ADDR_LEN) +
		      ((SIZEOF_TLV_HDR) + sizeof(prRepliedEvt->ucRssi_value));

	prNanPubRepliedInd = kmalloc(message_len, GFP_KERNEL);
	kalMemZero(prNanPubRepliedInd, message_len);

	DBGLOG(NAN, INFO, "[%s] message_len : %d\n", __func__, message_len);
	prNanPubRepliedInd->fwHeader.msgVersion = 1;
	prNanPubRepliedInd->fwHeader.msgId = NAN_MSG_ID_PUBLISH_REPLIED_IND;
	prNanPubRepliedInd->fwHeader.msgLen = message_len;
	prNanPubRepliedInd->fwHeader.handle = prRepliedEvt->u2Pubid;
	prNanPubRepliedInd->fwHeader.transactionId = 0;

	prNanPubRepliedInd->publishRepliedIndParams.matchHandle =
		prRepliedEvt->u2Subid;

	tlvs = prNanPubRepliedInd->ptlv;
	/* Add TLV datas */
	tlvs = nanAddTlv(NAN_TLV_TYPE_MAC_ADDRESS, MAC_ADDR_LEN,
			 &prRepliedEvt->auAddr[0], tlvs);

	tlvs = nanAddTlv(NAN_TLV_TYPE_RECEIVED_RSSI_VALUE,
			 sizeof(prRepliedEvt->ucRssi_value),
			 &prRepliedEvt->ucRssi_value, tlvs);

	/* Fill skb and send to kernel by nl80211 */
	skb = cfg80211_vendor_event_alloc(wiphy, wdev,
					  message_len + NLMSG_HDRLEN,
					  WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}
	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN, message_len,
			     prNanPubRepliedInd) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}
	cfg80211_vendor_event(skb, GFP_KERNEL);

	kfree(prNanPubRepliedInd);
	return WLAN_STATUS_SUCCESS;
}

int
mtk_cfg80211_vendor_event_nan_match_indication(IN struct ADAPTER *prAdapter,
					       uint8_t *pcuEvtBuf)
{
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	struct NAN_DISCOVERY_EVENT *prDiscEvt;
	struct NanMatchIndMsg *prNanMatchInd;
	struct NanSdeaCtrlParams peer_sdea_params;
	struct NanFWSdeaCtrlParams nanPeerSdeaCtrlarms;
	size_t message_len = 0;
	uint8_t *tlvs = NULL;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	kalMemZero(&nanPeerSdeaCtrlarms, sizeof(struct NanFWSdeaCtrlParams));
	kalMemZero(&peer_sdea_params, sizeof(struct NanSdeaCtrlParams));

	prDiscEvt = (struct NAN_DISCOVERY_EVENT *)pcuEvtBuf;

	message_len = sizeof(struct _NanMsgHeader) +
		      sizeof(struct _NanMatchIndParams) +
		      (SIZEOF_TLV_HDR + MAC_ADDR_LEN) +
		      (SIZEOF_TLV_HDR + prDiscEvt->u2Service_info_len) +
		      (SIZEOF_TLV_HDR + sizeof(struct NanFWSdeaCtrlParams));

	prNanMatchInd = kmalloc(message_len, GFP_KERNEL);
	kalMemZero(prNanMatchInd, message_len);

	prNanMatchInd->fwHeader.msgVersion = 1;
	prNanMatchInd->fwHeader.msgId = NAN_MSG_ID_MATCH_IND;
	prNanMatchInd->fwHeader.msgLen = message_len;
	prNanMatchInd->fwHeader.handle = prDiscEvt->u2SubscribeID;
	prNanMatchInd->fwHeader.transactionId = 0;

	prNanMatchInd->matchIndParams.matchHandle = prDiscEvt->u2PublishID;
	prNanMatchInd->matchIndParams.matchOccuredFlag =
		0; /* means match in SDF */
	prNanMatchInd->matchIndParams.outOfResourceFlag =
		0; /* doesn't outof resource. */

	tlvs = prNanMatchInd->ptlv;
	/* Add TLV datas */
	tlvs = nanAddTlv(NAN_TLV_TYPE_MAC_ADDRESS, MAC_ADDR_LEN,
			 &prDiscEvt->aucNanAddress[0], tlvs);
	DBGLOG(NAN, INFO, "[%s] :NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO %u\n",
	       __func__, NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO);

	tlvs = nanAddTlv(NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO,
			 prDiscEvt->u2Service_info_len,
			 &prDiscEvt->aucSerive_specificy_info[0], tlvs);

	nanPeerSdeaCtrlarms.data_path_required =
		(prDiscEvt->ucDataPathParm != 0) ? 1 : 0;
	nanPeerSdeaCtrlarms.security_required =
		(prDiscEvt->aucSecurityInfo[0] != 0) ? 1 : 0;
	nanPeerSdeaCtrlarms.ranging_required =
		(prDiscEvt->ucRange_measurement != 0) ? 1 : 0;

	DBGLOG(NAN, LOUD,
	       "[%s] data_path_required : %d, security_required:%d, ranging_required:%d\n",
	       __func__, nanPeerSdeaCtrlarms.data_path_required,
	       nanPeerSdeaCtrlarms.security_required,
	       nanPeerSdeaCtrlarms.ranging_required);

	tlvs = nanAddTlv(NAN_TLV_TYPE_SDEA_CTRL_PARAMS,
			 sizeof(struct NanFWSdeaCtrlParams),
			 (u8 *)&nanPeerSdeaCtrlarms, tlvs);

	/* Fill skb and send to kernel by nl80211 */
	skb = cfg80211_vendor_event_alloc(wiphy, wdev,
					  message_len + NLMSG_HDRLEN,
					  WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		kfree(prNanMatchInd);
		return -ENOMEM;
	}
	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN, message_len,
			     prNanMatchInd) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}
	cfg80211_vendor_event(skb, GFP_KERNEL);
	kfree(prNanMatchInd);

	return WLAN_STATUS_SUCCESS;
}

int
mtk_cfg80211_vendor_event_nan_publish_terminate(IN struct ADAPTER *prAdapter,
						uint8_t *pcuEvtBuf)
{
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	struct NAN_PUBLISH_TERMINATE_EVENT *prPubTerEvt;
	struct NanPublishTerminatedIndMsg nanPubTerInd;
	size_t message_len = 0;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	kalMemZero(&nanPubTerInd, sizeof(struct NanPublishTerminatedIndMsg));
	prPubTerEvt = (struct NAN_PUBLISH_TERMINATE_EVENT *)pcuEvtBuf;

	message_len = sizeof(struct NanPublishTerminatedIndMsg);

	nanPubTerInd.fwHeader.msgVersion = 1;
	nanPubTerInd.fwHeader.msgId = NAN_MSG_ID_PUBLISH_TERMINATED_IND;
	nanPubTerInd.fwHeader.msgLen = message_len;
	nanPubTerInd.fwHeader.handle = prPubTerEvt->u2Pubid;
	/* Indication doesn't have transactionId, don't care */
	nanPubTerInd.fwHeader.transactionId = 0;
	/* For all user should be success. */
	nanPubTerInd.reason = prPubTerEvt->ucReasonCode;

	DBGLOG(NAN, ERROR, "[%s] Cancel Pub ID = %d\n",
	       nanPubTerInd.fwHeader.handle);

	/* Fill skb and send to kernel by nl80211 */
	skb = cfg80211_vendor_event_alloc(wiphy, wdev,
					  message_len + NLMSG_HDRLEN,
					  WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}
	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN, message_len,
			     &nanPubTerInd) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}
	cfg80211_vendor_event(skb, GFP_KERNEL);

	return WLAN_STATUS_SUCCESS;
}

int
mtk_cfg80211_vendor_event_nan_subscribe_terminate(IN struct ADAPTER *prAdapter,
						  uint8_t *pcuEvtBuf)
{
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	struct NAN_SUBSCRIBE_TERMINATE_EVENT *prSubTerEvt;
	struct NanSubscribeTerminatedIndMsg nanSubTerInd;
	size_t message_len = 0;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	kalMemZero(&nanSubTerInd, sizeof(struct NanSubscribeTerminatedIndMsg));
	prSubTerEvt = (struct NAN_SUBSCRIBE_TERMINATE_EVENT *)pcuEvtBuf;

	message_len = sizeof(struct NanSubscribeTerminatedIndMsg);

	nanSubTerInd.fwHeader.msgVersion = 1;
	nanSubTerInd.fwHeader.msgId = NAN_MSG_ID_SUBSCRIBE_TERMINATED_IND;
	nanSubTerInd.fwHeader.msgLen = message_len;
	nanSubTerInd.fwHeader.handle = prSubTerEvt->u2Subid;
	/* Indication doesn't have transactionId, don't care */
	nanSubTerInd.fwHeader.transactionId = 0;
	/* For all user should be success. */
	nanSubTerInd.reason = prSubTerEvt->ucReasonCode;

	/* Fill skb and send to kernel by nl80211 */
	skb = cfg80211_vendor_event_alloc(wiphy, wdev,
					  message_len + NLMSG_HDRLEN,
					  WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}
	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN, message_len,
			     &nanSubTerInd) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}
	cfg80211_vendor_event(skb, GFP_KERNEL);

	return WLAN_STATUS_SUCCESS;
}

int
mtk_cfg80211_vendor_event_nan_followup_indication(IN struct ADAPTER *prAdapter,
						  uint8_t *pcuEvtBuf)
{
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	struct NanFollowupIndMsg *prNanFollowupInd;
	struct NAN_FOLLOW_UP_EVENT *prFollowupEvt;
	uint8_t *tlvs = NULL;
	size_t message_len = 0;

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;

	prFollowupEvt = (struct NAN_FOLLOW_UP_EVENT *)pcuEvtBuf;

	message_len =
		sizeof(struct _NanMsgHeader) +
		sizeof(struct _NanFollowupIndParams) +
		(SIZEOF_TLV_HDR + MAC_ADDR_LEN) +
		(SIZEOF_TLV_HDR + prFollowupEvt->service_specific_info_len);

	prNanFollowupInd = kmalloc(message_len, GFP_KERNEL);
	kalMemZero(prNanFollowupInd, message_len);

	prNanFollowupInd->fwHeader.msgVersion = 1;
	prNanFollowupInd->fwHeader.msgId = NAN_MSG_ID_FOLLOWUP_IND;
	prNanFollowupInd->fwHeader.msgLen = message_len;
	prNanFollowupInd->fwHeader.handle = prFollowupEvt->publish_subscribe_id;

	/* Indication doesn't have transition ID */
	prNanFollowupInd->fwHeader.transactionId = 0;

	/* Mapping datas */
	prNanFollowupInd->followupIndParams.matchHandle =
		prFollowupEvt->requestor_instance_id;
	prNanFollowupInd->followupIndParams.window = prFollowupEvt->dw_or_faw;

	DBGLOG(NAN, LOUD, "[%s] matchHandle: %d, window:%d\n", __func__,
	       prNanFollowupInd->followupIndParams.matchHandle,
	       prNanFollowupInd->followupIndParams.window);

	tlvs = prNanFollowupInd->ptlv;
	/* Add TLV datas */
	tlvs = nanAddTlv(NAN_TLV_TYPE_MAC_ADDRESS, MAC_ADDR_LEN,
			 prFollowupEvt->addr, tlvs);

	tlvs = nanAddTlv(NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO,
			 prFollowupEvt->service_specific_info_len,
			 prFollowupEvt->service_specific_info, tlvs);

	/* Ranging report
	 * To be implement. NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO
	 */

	/*  Fill skb and send to kernel by nl80211 */
	skb = cfg80211_vendor_event_alloc(wiphy, wdev,
					  message_len + NLMSG_HDRLEN,
					  WIFI_EVENT_SUBCMD_NAN, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}
	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NAN, message_len,
			     prNanFollowupInd) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		kfree(prNanFollowupInd);
		return -EFAULT;
	}
	cfg80211_vendor_event(skb, GFP_KERNEL);
	kfree(prNanFollowupInd);

	return WLAN_STATUS_SUCCESS;
}
