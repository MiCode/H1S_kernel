/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

/*
 * gl_vendor_ndp.c
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
#include "gl_vendor_ndp.h"
#include "debug.h"
#include "gl_cfg80211.h"
#include "gl_os.h"
#include "gl_vendor.h"
#include "gl_wext.h"
#include "nan_data_engine.h"
#include "nan_sec.h"
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
uint8_t g_InitiatorMacAddr[6];

const struct nla_policy
	mtk_wlan_vendor_ndp_policy[MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX + 1] = {
			[MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID] = {
				.type = NLA_U16 },
			[MTK_WLAN_VENDOR_ATTR_NDP_IFACE_STR] = {
				.type = NLA_NUL_STRING,
				.len = IFNAMSIZ - 1 },
			[MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR] = {
				.type = NLA_BINARY,
				.len = MAC_ADDR_LEN },
			[MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_SECURITY] = {
				.type = NLA_NESTED },
			[MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO] = {
				.type = NLA_BINARY,
				.len = NDP_APP_INFO_LEN },
			[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_NDI_MAC_ADDR] = {
				.type = NLA_BINARY,
				.len = MAC_ADDR_LEN },
			[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY] = {
				.type = NLA_BINARY,
				.len = NDP_NUM_INSTANCE_ID },
			[MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_CONFIG] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_PMK] = {
				.type = NLA_BINARY,
				.len = NDP_PMK_LEN },
			[MTK_WLAN_VENDOR_ATTR_NDP_SCID] = {
				.type = NLA_BINARY,
				.len = NDP_SCID_BUF_LEN },
			[MTK_WLAN_VENDOR_ATTR_NDP_CSID] = {
				.type = NLA_U32 },
			[MTK_WLAN_VENDOR_ATTR_NDP_PASSPHRASE] = {
				.type = NLA_BINARY,
				.len = NAN_PASSPHRASE_MAX_LEN },
			[MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_NAME] = {
				.type = NLA_BINARY,
				.len = NAN_MAX_SERVICE_NAME_LEN },
	};

/*----------------------------------------------------------------------------*/
/*!
* \brief After NDI interface create, send create response to wifi hal
*
* \param[in] prNDP: NDP info
*
* \return WLAN_STATUS
*/
/*----------------------------------------------------------------------------*/
uint32_t
nanNdiCreateRspEvent(struct ADAPTER *prAdapter) {
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	uint16_t u2CreateRspLen;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}


	DBGLOG(NAN, INFO, "Send NDI Create Rsp event\n");

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	u2CreateRspLen = (3 * sizeof(uint32_t)) + sizeof(uint16_t) +
			 (4 * NLA_HDRLEN) + NLMSG_HDRLEN;

	skb = cfg80211_vendor_event_alloc(wiphy, wdev, u2CreateRspLen,
					  WIFI_EVENT_SUBCMD_NDP, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
				 MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_CREATE) <
		     0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	/* set Transaction ID = 1 as workaround */
	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID,
				 1) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	/* prNDP no NanInternalStatusType field,
	 * set NAN_I_STATUS_SUCCESS as workaround
	 */
	if (unlikely(nla_put_u32(
			     skb,
			     MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
			     NAN_I_STATUS_SUCCESS) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE,
				 DP_REASON_SUCCESS) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief After NDI interface delete, send delete response to wifi hal
 *
 * \param[in] prNDP: NDP info
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdiDeleteRspEvent(struct ADAPTER *prAdapter) {
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	uint16_t u2CreateRspLen;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "Send NDI Delete Rsp event\n");

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	u2CreateRspLen = (3 * sizeof(uint32_t)) + sizeof(uint16_t) +
			 (4 * NLA_HDRLEN) + NLMSG_HDRLEN;

	skb = cfg80211_vendor_event_alloc(wiphy, wdev, u2CreateRspLen,
					  WIFI_EVENT_SUBCMD_NDP, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
				 MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_DELETE) <
		     0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	/* set Transaction ID = 1 as workaround */
	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID,
				 1) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	/* prNDP no NanInternalStatusType field,
	 * set NAN_I_STATUS_SUCCESS as workaround
	 */
	if (unlikely(nla_put_u32(
			     skb,
			     MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
			     NAN_I_STATUS_SUCCESS) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE,
				 DP_REASON_SUCCESS) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}
/*----------------------------------------------------------------------------*/
/*!
 * \brief After Tx initiator req NAF TxDone, send initiator response to wifi hal
 *
 * \param[in] prNDP: NDP info
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdpInitiatorRspEvent(struct ADAPTER *prAdapter,
			struct _NAN_NDP_INSTANCE_T *prNDP,
			uint32_t rTxDoneStatus) {
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	uint16_t u2InitiatorRspLen;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prNDP == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prNDP is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "[%s] Send NDP Initiator Rsp event\n", __func__);

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	u2InitiatorRspLen = (4 * sizeof(uint32_t)) + (1 * sizeof(uint16_t)) +
			    (5 * NLA_HDRLEN) + NLMSG_HDRLEN;

	skb = cfg80211_vendor_event_alloc(wiphy, wdev, u2InitiatorRspLen,
					  WIFI_EVENT_SUBCMD_NDP, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
				 MTK_WLAN_VENDOR_ATTR_NDP_INITIATOR_RESPONSE) <
		     0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u16(skb, MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID,
				 prNDP->ucDialogToken) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID,
				 prNDP->ucNDPID) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	/* prNDP no NanInternalStatusType field,
	 * set NAN_I_STATUS_SUCCESS and NAN_I_STATUS_TIMEOUT as workaround
	 */
	if (rTxDoneStatus == WLAN_STATUS_SUCCESS) {
		if (unlikely(nla_put_u32(skb,
			    MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
			    NAN_I_STATUS_SUCCESS) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
	} else {
		if (unlikely(nla_put_u32(skb,
			    MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
			    NAN_I_STATUS_TIMEOUT) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE,
				 prNDP->eDataPathFailReason) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief After Tx responder req NAF TxDone, send responder response to wifi hal
 *
 * \param[in] prNDP: NDP info
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdpResponderRspEvent(struct ADAPTER *prAdapter,
			struct _NAN_NDP_INSTANCE_T *prNDP,
			uint32_t rTxDoneStatus) {
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	uint16_t u2ResponderRspLen;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prNDP == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prNDP is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "Send NDP Response event\n");

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	u2ResponderRspLen = (3 * sizeof(uint32_t)) + sizeof(uint16_t) +
			    (4 * NLA_HDRLEN) + NLMSG_HDRLEN;

	skb = cfg80211_vendor_event_alloc(wiphy, wdev, u2ResponderRspLen,
					  WIFI_EVENT_SUBCMD_NDP, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
				 MTK_WLAN_VENDOR_ATTR_NDP_RESPONDER_RESPONSE) <
		     0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u16(skb, MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID,
				 prNDP->ucDialogToken) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	/* prNDP no NanInternalStatusType field,
	 * set NAN_I_STATUS_SUCCESS and NAN_I_STATUS_TIMEOUT as workaround
	 */
	if (rTxDoneStatus == WLAN_STATUS_SUCCESS) {
		if (unlikely(nla_put_u32(skb,
			    MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
			    NAN_I_STATUS_SUCCESS) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
	} else {
		if (unlikely(nla_put_u32(skb,
			    MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
			    NAN_I_STATUS_TIMEOUT) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE,
				 prNDP->eDataPathFailReason) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief After Tx termination req NAF TxDone, send end response to wifi hal
 *
 * \param[in] prNDP: NDP info
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdpEndRspEvent(struct ADAPTER *prAdapter, struct _NAN_NDP_INSTANCE_T *prNDP,
		  uint32_t rTxDoneStatus) {
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	uint16_t u2EndRspLen;

	if (prAdapter == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prAdapter is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "Send NDI End Rsp event\n");

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	u2EndRspLen = (3 * sizeof(uint32_t)) + sizeof(uint16_t) +
		      (4 * NLA_HDRLEN) + NLMSG_HDRLEN;

	skb = cfg80211_vendor_event_alloc(wiphy, wdev, u2EndRspLen,
					  WIFI_EVENT_SUBCMD_NDP, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
				 MTK_WLAN_VENDOR_ATTR_NDP_END_RESPONSE) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	/* prNDP no NanInternalStatusType field,
	 * set NAN_I_STATUS_SUCCESS and NAN_I_STATUS_TIMEOUT as workaround
	 */
	if (rTxDoneStatus == WLAN_STATUS_SUCCESS) {
		if (unlikely(nla_put_u32(
			    skb,
			    MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
			    NAN_I_STATUS_SUCCESS) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
	} else {
		if (unlikely(nla_put_u32(skb,
			    MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
			    NAN_I_STATUS_TIMEOUT) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE,
				 prNDP->eDataPathFailReason) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}
	if (unlikely(nla_put_u16(skb, MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID,
				 prNDP->ucDialogToken) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Handle NAN interface create request vendor cmd.
 *
 * \param[in] prGlueInfo: Pointer to glue info structure.
 *
 * \param[in] tb: NDP vendor cmd attributes.
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdiCreateHandler(struct GLUE_INFO *prGlueInfo, struct nlattr **tb) {
	/* Need implement */
	struct ADAPTER *prAdapter = NULL;

	if (prGlueInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prGlueInfo is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "NAN interface create request, need implement!\n");

	/* Workaround: send event to wifi hal */
	prAdapter = prGlueInfo->prAdapter;
	nanNdiCreateRspEvent(prAdapter);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Handle NAN interface delete request vendor cmd.
 *
 * \param[in] prGlueInfo: Pointer to glue info structure.
 *
 * \param[in] tb: NDP vendor cmd attributes.
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdiDeleteHandler(struct GLUE_INFO *prGlueInfo, struct nlattr **tb) {
	/* Need implement */
	struct ADAPTER *prAdapter = NULL;

	if (prGlueInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prGlueInfo is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "NAN interface delete request, need implement!\n");

	/* Workaround: send event to wifi hal */
	prAdapter = prGlueInfo->prAdapter;
	nanNdiDeleteRspEvent(prAdapter);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Handle NDP initiator request vendor cmd.
 *
 * \param[in] prGlueInfo: Pointer to glue info structure.
 *
 * \param[in] tb: NDP vendor cmd attributes.
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdpInitiatorReqHandler(struct GLUE_INFO *prGlueInfo, struct nlattr **tb) {
	struct _NAN_CMD_DATA_REQUEST rNanCmdDataRequest;
	struct NanDataReqReceive rDataRcv;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	kalMemZero(&rNanCmdDataRequest, sizeof(rNanCmdDataRequest));

	/* Instance ID */
	if (!tb[MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID]) {
		DBGLOG(NAN, ERROR, "Get NDP Instance ID unavailable!\n");
		return -EINVAL;
	}
	/* Peer mac Addr */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]) {
		kalMemCopy(rNanCmdDataRequest.aucResponderDataAddress,
		nla_data(
		tb[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]),
		MAC_ADDR_LEN);
	}
	rNanCmdDataRequest.ucPublishID =
		nla_get_u32(tb[MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID]);
	if (rNanCmdDataRequest.ucPublishID == 0)
		rNanCmdDataRequest.ucPublishID = g_u2IndPubId;

	/* Security */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_SECURITY]) {
		rNanCmdDataRequest.ucSecurity =
			nla_get_u32(tb[MTK_WLAN_VENDOR_ATTR_NDP_CSID]);
		if (rNanCmdDataRequest.ucSecurity &&
		    tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]) {
			kalMemCopy(rNanCmdDataRequest.aucPMK,
				   nla_data(tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]),
				   nla_len(tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]));
			if (tb[MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_NAME]) {
				nanSetNdpPmkid(
				prGlueInfo->prAdapter,
				&rNanCmdDataRequest,
				nla_data(
				tb[MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_NAME]
				));
			}
#if (ENABLE_SEC_UT_LOG == 1)
			DBGLOG(NAN, INFO, "PMK from APP\n");
			dumpMemory8(nla_data(tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]),
				    nla_len(tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]));
#endif
		}
	}

	/* QoS: Default set to false for testing */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS]) {
		rNanCmdDataRequest.ucRequireQOS = 0;
		/* rNanCmdDataRequest.ucRequireQOS = */
		/* nla_get_u32(tb[MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS]); */
	}

	/* Peer mac Addr */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]) {
		kalMemCopy(rNanCmdDataRequest.aucResponderDataAddress,
			nla_data(
			tb[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]),
			MAC_ADDR_LEN);
	}

	rNanCmdDataRequest.fgNDPE = g_ndpReqNDPE.fgEnNDPE;
	if (rNanCmdDataRequest.fgNDPE) {
		/* Ipv6: vendor cmd did not fill this attribute,
		 * default set to FALSE
		 */
		rNanCmdDataRequest.fgCarryIpv6 = 1;

		/* APP Info */
		if (tb[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]) {
			rNanCmdDataRequest.u2SpecificInfoLength =
				nla_len(tb[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]);
			kalMemCopy(rNanCmdDataRequest.aucSpecificInfo,
				nla_data(tb[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]),
				nla_len(tb[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]));
		}
	}

	/* Send cmd request */
	rStatus = nanCmdDataRequest(prGlueInfo->prAdapter, &rNanCmdDataRequest,
				    &rDataRcv.ndpid,
				    rDataRcv.initiator_data_addr);
	DBGLOG(NAN, INFO, "Initiator request to peer " MACSTR ", status = %d\n",
	       MAC2STR(rNanCmdDataRequest.aucResponderDataAddress), rStatus);

	/* Return status */
	return rStatus;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Handle NDP reponder request vendor cmd.
 *
 * \param[in] prGlueInfo: Pointer to glue info structure.
 *
 * \param[in] tb: NDP vendor cmd attributes.
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdpResponderReqHandler(struct GLUE_INFO *prGlueInfo, struct nlattr **tb) {
	struct _NAN_CMD_DATA_RESPONSE rNanCmdDataResponse;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	kalMemZero(&rNanCmdDataResponse, sizeof(rNanCmdDataResponse));
	/* Decision status */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE]) {
		rNanCmdDataResponse.ucDecisionStatus =
			nla_get_u32(tb[MTK_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE]);
	}

	/* Instance ID */
	if (!tb[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID]) {
		DBGLOG(NAN, ERROR, "Get NDP Instance ID unavailable!\n");
		return -EINVAL;
	}

	rNanCmdDataResponse.ucNDPId =
		nla_get_u32(tb[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID]);
	DBGLOG(NAN, INFO, "[Data Resp] RespID:%d\n",
	       rNanCmdDataResponse.ucNDPId);
	if (rNanCmdDataResponse.ucNDPId == 0)
		rNanCmdDataResponse.ucNDPId = g_u2IndPubId;

	/* QoS: Default set to false for testing */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS]) {
		rNanCmdDataResponse.ucRequireQOS = 0;
		/* rNanCmdDataResponse.ucRequireQOS =
		 * nla_get_u32(tb[MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS]);
		 */
	}

	/* Security type */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_SECURITY]) {
		rNanCmdDataResponse.ucSecurity =
			nla_get_u32(tb[MTK_WLAN_VENDOR_ATTR_NDP_SECURITY_TYPE]);
	}

	/* App Info */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]) {
		rNanCmdDataResponse.u2SpecificInfoLength =
			IPV6MACLEN;
		kalMemCopy(rNanCmdDataResponse.aucSpecificInfo,
			nla_data(tb[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]),
			IPV6MACLEN);
		kalMemCopy(rNanCmdDataResponse.aucIPv6Addr,
			nla_data(tb[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]),
			IPV6MACLEN);
		/* Ipv6: vendor cmd did not fill this attribute,
		 * set to TRUE if carry Ipv6 by Sigma
		 */
		rNanCmdDataResponse.fgCarryIpv6 = 1;

		DBGLOG(NAN, ERROR, "[%s] appInfoLen= %d, Ipv6 ="IPV6STR"\n",
			__func__,
			rNanCmdDataResponse.u2SpecificInfoLength,
			IPV6TOSTR(rNanCmdDataResponse.aucIPv6Addr));
	}

	/* PortNum: vendor cmd did not fill this attribute,
	 * default set to 9000
	 */
	rNanCmdDataResponse.u2PortNum = 9000;

	/* Service protocol type: vendor cmd did not fill this attribute,
	 * default set to 0xFF
	 */
	rNanCmdDataResponse.ucServiceProtocolType = IP_PRO_TCP;

	/* Peer mac addr */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]) {
		kalMemCopy(
			rNanCmdDataResponse.aucInitiatorDataAddress,
			nla_data(
			tb[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]),
			MAC_ADDR_LEN);
	} else {
		kalMemCopy(rNanCmdDataResponse.aucInitiatorDataAddress,
			   g_InitiatorMacAddr, MAC_ADDR_LEN);
	}
	DBGLOG(NAN, INFO, "[%s] aucInitiatorDataAddress = " MACSTR "\n",
	       __func__, MAC2STR(rNanCmdDataResponse.aucInitiatorDataAddress));
	/* PMK */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]) {
		kalMemCopy(rNanCmdDataResponse.aucPMK,
			   nla_data(tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]),
			   nla_len(tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]));
#if (ENABLE_SEC_UT_LOG == 1)
		DBGLOG(NAN, INFO, "PMK from APP\n");
		dumpMemory8(nla_data(tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]),
			    nla_len(tb[MTK_WLAN_VENDOR_ATTR_NDP_PMK]));
#endif
	}
	/* Send data response */
	rStatus =
		nanCmdDataResponse(prGlueInfo->prAdapter, &rNanCmdDataResponse);
	DBGLOG(NAN, INFO,
	       "Responder response to peer " MACSTR ", status = %d\n",
	       MAC2STR(rNanCmdDataResponse.aucInitiatorDataAddress), rStatus);

	/* Return */
	return rStatus;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Handle NDP end vendor cmd.
 *
 * \param[in] prGlueInfo: Pointer to glue info structure.
 *
 * \param[in] tb: NDP vendor cmd attributes.
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/
uint32_t
nanNdpEndReqHandler(struct GLUE_INFO *prGlueInfo, struct nlattr **tb) {
	struct _NAN_CMD_DATA_END rNanCmdDataEnd;
	uint32_t rStatus;
	uint32_t instanceIdNum;
	uint32_t i;

	kalMemZero(&rNanCmdDataEnd, sizeof(rNanCmdDataEnd));

	/* trial run! */
	DBGLOG(NAN, INFO, "NDP end request\n");

	if (!tb[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY]) {
		DBGLOG(NAN, ERROR, "Get NDP Instance ID unavailable!\n");
		return -EINVAL;
	}
	instanceIdNum =
		nla_len(tb[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY]) /
		sizeof(uint32_t);
	if (instanceIdNum <= 0) {
		DBGLOG(NAN, ERROR, "No NDP Instance ID!\n");
		return -EINVAL;
	}

	for (i = 0; i < instanceIdNum; i++) {
		rNanCmdDataEnd.ucNDPId = nla_get_u32(
			tb[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY] +
			i * sizeof(uint32_t));
		rStatus = nanCmdDataEnd(prGlueInfo->prAdapter, &rNanCmdDataEnd);
	}
	return rStatus;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Send data indication event to hal.
 *
 * \param[in] prNDP: NDP info attribute
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/

uint32_t
nanNdpDataIndEvent(IN struct ADAPTER *prAdapter,
		   struct _NAN_NDP_INSTANCE_T *prNDP,
		   struct _NAN_NDL_INSTANCE_T *prNDL) {
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	uint16_t u2IndiEventLen;

	if (prNDP == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prNDP is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	if (prNDL == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prNDL is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "Send NDP Data Indication event\n");

	if (prAdapter->rNanNetRegState == ENUM_NET_REG_STATE_UNREGISTERED) {
		DBGLOG(NAN, ERROR, "Net device for NAN unregistered\n");
		return WLAN_STATUS_FAILURE;
	}

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	u2IndiEventLen = (3 * sizeof(uint32_t)) + (2 * MAC_ADDR_LEN) +
			 prNDP->u2AppInfoLen + NAN_SCID_DEFAULT_LEN +
			 (6 * NLA_HDRLEN) + NLMSG_HDRLEN;

	skb = cfg80211_vendor_event_alloc(wiphy, wdev, u2IndiEventLen,
					  WIFI_EVENT_SUBCMD_NDP, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
				 MTK_WLAN_VENDOR_ATTR_NDP_REQUEST_IND) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u32(skb,
				 MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID,
				 prNDP->ucPublishId)) < 0) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NDP_NDI_MAC_ADDR,
			     MAC_ADDR_LEN, prNDP->aucPeerNDIAddr) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}
	kalMemCopy(g_InitiatorMacAddr, prNDP->aucPeerNDIAddr, MAC_ADDR_LEN);
	DBGLOG(NAN, INFO, "[%s] gInitiatorMacAddr = " MACSTR "\n", __func__,
	       MAC2STR(g_InitiatorMacAddr));

	if (unlikely(nla_put(skb,
			     MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR,
			     MAC_ADDR_LEN, prNDL->aucPeerMacAddr) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID,
				 prNDP->ucNDPID) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (prNDP->u2AppInfoLen) {
		if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO,
				     prNDP->u2AppInfoLen,
				     prNDP->pucAppInfo) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
	}

/* Todo:
 * 1. QoS currently not support.
 * 2. Need to clarify security parameter
 */
#if 0
	if (prNDP->fgQoSRequired)
		continue;

	if (prNDP->fgSecurityRequired) {
		if (unlikely(nla_put_u32(skb,
					MTK_WLAN_VENDOR_ATTR_NDP_NCS_SK_TYPE,
					prNDP->ucCipherType) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
		if (unlikely(nla_put(skb,
				     MTK_WLAN_VENDOR_ATTR_NDP_SCID,
				     NAN_SCID_DEFAULT_LEN,
				     prNDP->au1Scid) < 0)) {
			DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
			kfree_skb(skb);
			return -EFAULT;
		}
	}
#endif
	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Send data confirm event to hal.
 *
 * \param[in] prNDP: NDP info attribute
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/

uint32_t
nanNdpDataConfirmEvent(IN struct ADAPTER *prAdapter,
		       struct _NAN_NDP_INSTANCE_T *prNDP) {
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	uint32_t u2ConfirmEventLen;

	if (prNDP == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prNDP is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "Send NDP Data Confirm event\n");

	if (prAdapter->rNanNetRegState == ENUM_NET_REG_STATE_UNREGISTERED) {
		DBGLOG(NAN, ERROR, "Net device for NAN unregistered\n");
		return WLAN_STATUS_FAILURE;
	}

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	u2ConfirmEventLen = (4 * sizeof(uint32_t)) + MAC_ADDR_LEN +
			    +NLMSG_HDRLEN + (6 * NLA_HDRLEN) +
			    prNDP->u2AppInfoLen;
	/* WIFI_EVENT_SUBCMD_NDP: Event Idx is 13 for kernel,
	 *  but for WifiHal is 81
	 */
	skb = cfg80211_vendor_event_alloc(wiphy, wdev, u2ConfirmEventLen,
					  WIFI_EVENT_SUBCMD_NDP, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
				 MTK_WLAN_VENDOR_ATTR_NDP_CONFIRM_IND) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID,
				 prNDP->ucNDPID) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NDP_NDI_MAC_ADDR,
			     MAC_ADDR_LEN, prNDP->aucPeerNDIAddr) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (prNDP->pucAppInfo &&
	    nla_put(skb, MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO, prNDP->u2AppInfoLen,
		    prNDP->pucAppInfo)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE,
				 prNDP->ucReasonCode) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE,
				 prNDP->eDataPathFailReason) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	DBGLOG(NAN, INFO, "NDP Data Confirm event, ndp instance: %d,",
		prNDP->ucNDPID);
	DBGLOG(NAN, INFO, "peer MAC addr : "MACSTR "rsp reason code: %d,",
		prNDP->aucPeerNDIAddr, prNDP->ucReasonCode);
	DBGLOG(NAN, INFO, "protocol reason code: %d\n ",
		prNDP->eDataPathFailReason);

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Send data termination event to hal.
 *
 * \param[in] prNDP: NDP info attribute
 *
 * \return WLAN_STATUS
 */
/*----------------------------------------------------------------------------*/

uint32_t
nanNdpDataTerminationEvent(IN struct ADAPTER *prAdapter,
			   struct _NAN_NDP_INSTANCE_T *prNDP) {
	struct sk_buff *skb = NULL;
	struct wiphy *wiphy;
	struct wireless_dev *wdev;
	uint32_t u2ConfirmEventLen;
	uint32_t *pu2NDPInstance;

	if (prNDP == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prNDP is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	pu2NDPInstance = kalMemAlloc(1 * sizeof(*pu2NDPInstance), VIR_MEM_TYPE);
	if (pu2NDPInstance == NULL) {
		DBGLOG(NAN, ERROR, "[%s] pu2NDPInstance is NULL\n", __func__);
		return WLAN_STATUS_INVALID_DATA;
	}

	DBGLOG(NAN, INFO, "Send NDP Data Termination event\n");

	wiphy = wlanGetWiphy();
	wdev = (wlanGetNetDev(prAdapter->prGlueInfo, AIS_DEFAULT_INDEX))
		       ->ieee80211_ptr;
	u2ConfirmEventLen = sizeof(uint32_t) + NLMSG_HDRLEN + (2 * NLA_HDRLEN) +
			    1 * sizeof(*pu2NDPInstance);

	skb = cfg80211_vendor_event_alloc(wiphy, wdev, u2ConfirmEventLen,
					  WIFI_EVENT_SUBCMD_NDP, GFP_KERNEL);
	if (!skb) {
		DBGLOG(REQ, ERROR, "Allocate skb failed\n");
		return -ENOMEM;
	}

	if (unlikely(nla_put_u32(skb, MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
				 MTK_WLAN_VENDOR_ATTR_NDP_END_IND) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	if (unlikely(nla_put(skb, MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY,
			     1 * sizeof(*pu2NDPInstance),
			     pu2NDPInstance) < 0)) {
		DBGLOG(REQ, ERROR, "nla_put_nohdr failed\n");
		kfree_skb(skb);
		return -EFAULT;
	}

	DBGLOG(NAN, INFO, "NDP Data Termination event, ndp instance: %d\n",
	       prNDP->ucNDPID);

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return WLAN_STATUS_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief This function is the enter point of NDP vendor cmd.
 *
 * \param[in] wiphy: for AIS STA
 *
 * \param[in] wdev (not used here).
 *
 * \param[in] data: Content of NDP vendor cmd .
 *
 * \param[in] data_len: NDP vendor cmd length.
 *
 * \return int
 */
/*----------------------------------------------------------------------------*/
int
mtk_cfg80211_vendor_ndp(struct wiphy *wiphy, struct wireless_dev *wdev,
			const void *data, int data_len) {
	struct GLUE_INFO *prGlueInfo = NULL;
	struct nlattr *tb[MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX + 1];
	uint32_t u4NdpCmdType;
	uint16_t u2NdpTransactionId;
	char *ifaceName;
	uint32_t rStatus;

	if (wiphy == NULL) {
		DBGLOG(NAN, ERROR, "[%s] wiphy is NULL\n", __func__);
		return -EINVAL;
	}

	if (wdev == NULL) {
		DBGLOG(NAN, ERROR, "[%s] wdev is NULL\n", __func__);
		return -EINVAL;
	}

	if (data == NULL || data_len <= 0) {
		log_dbg(REQ, ERROR, "data error(len=%d)\n", data_len);
		return -EINVAL;
	}

	WIPHY_PRIV(wiphy, prGlueInfo);
	if (prGlueInfo == NULL) {
		DBGLOG(NAN, ERROR, "[%s] prGlueInfo is NULL\n", __func__);
		return -EINVAL;
	}

	DBGLOG(NAN, INFO, "DATA len from user %d\n", data_len);

	/* Parse NDP vendor cmd */
	if (NLA_PARSE(tb, MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX, data, data_len,
		      mtk_wlan_vendor_ndp_policy)) {
		DBGLOG(NAN, ERROR, "Parse NDP cmd fail!\n");
		return -EINVAL;
	}

	/* Get NDP command type*/
	if (!tb[MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD]) {
		DBGLOG(NAN, ERROR, "Get NDP cmd type error!\n");
		return -EINVAL;
	}
	u4NdpCmdType = nla_get_u32(tb[MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD]);

	/* Get transaction ID */
	if (!tb[MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID]) {
		DBGLOG(NAN, ERROR, "Get NDP Transaction ID error!\n");
		return -EINVAL;
	}
	u2NdpTransactionId =
		nla_get_u16(tb[MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID]);

	/* Get interface name */
	if (tb[MTK_WLAN_VENDOR_ATTR_NDP_IFACE_STR]) {
		ifaceName = nla_data(tb[MTK_WLAN_VENDOR_ATTR_NDP_IFACE_STR]);
		DBGLOG(NAN, INFO,
		       "NDP cmd type: %d Transaction ID: %d Interface name: %s!\n",
		       u4NdpCmdType, u2NdpTransactionId, ifaceName);
	} else {
		DBGLOG(NAN, INFO,
		       "NDP cmd type: %d Transaction ID: %d Interface name: Unspecified!\n",
		       u4NdpCmdType, u2NdpTransactionId);
	}

	switch (u4NdpCmdType) {
	/* Command to create a NAN data path interface */
	case MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_CREATE:
		rStatus = nanNdiCreateHandler(prGlueInfo, tb);
		break;
	/* Command to delete a NAN data path interface */
	case MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_DELETE:
		rStatus = nanNdiDeleteHandler(prGlueInfo, tb);
		break;
	/* Command to initiate a NAN data path session */
	case MTK_WLAN_VENDOR_ATTR_NDP_INITIATOR_REQUEST:
		rStatus = nanNdpInitiatorReqHandler(prGlueInfo, tb);
		break;
	/* Command to respond to NAN data path session */
	case MTK_WLAN_VENDOR_ATTR_NDP_RESPONDER_REQUEST:
		rStatus = nanNdpResponderReqHandler(prGlueInfo, tb);
		break;
	/* Command to initiate a NAN data path end */
	case MTK_WLAN_VENDOR_ATTR_NDP_END_REQUEST:
		rStatus = nanNdpEndReqHandler(prGlueInfo, tb);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return rStatus;
}
