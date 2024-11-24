/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

/*
 * Log: gl_vendor_ndp.h
 *
 *
 *
 */

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "gl_wext.h"
#include "wlan_lib.h"
#include <linux/can/netlink.h>
#include <linux/ieee80211.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/wireless.h>
#include <net/cfg80211.h>
#include <net/netlink.h>

#ifndef _GL_VENDOR_NDP_H_
#define _GL_VENDOR_NDP_N

#if CFG_SUPPORT_NAN

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
 *				P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */
/* NAN Social channels */
#define NAN_SOCIAL_CHANNEL_2_4GHZ 6
#define NAN_SOCIAL_CHANNEL_5GHZ_LOWER_BAND 44
#define NAN_SOCIAL_CHANNEL_5GHZ_UPPER_BAND 149

#define NDP_APP_INFO_LEN 255
#define NDP_PMK_LEN 32
#define NDP_SCID_BUF_LEN 256
#define NDP_NUM_INSTANCE_ID 255

#define NAN_MAX_SERVICE_NAME_LEN 255
#define NAN_PASSPHRASE_MIN_LEN 8
#define NAN_PASSPHRASE_MAX_LEN 63

#define PACKED __packed

extern struct NanDataPathInitiatorNDPE g_ndpReqNDPE;

enum mtk_wlan_ndp_sub_cmd {
	MTK_WLAN_VENDOR_ATTR_NDP_INVALID = 0,
	/* Command to create a NAN data path interface */
	MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_CREATE = 1,
	/* Command to delete a NAN data path interface */
	MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_DELETE = 2,
	/* Command to initiate a NAN data path session */
	MTK_WLAN_VENDOR_ATTR_NDP_INITIATOR_REQUEST = 3,
	/* Command to notify if the NAN data path session was sent */
	MTK_WLAN_VENDOR_ATTR_NDP_INITIATOR_RESPONSE = 4,
	/* Command to respond to NAN data path session */
	MTK_WLAN_VENDOR_ATTR_NDP_RESPONDER_REQUEST = 5,
	/* Command to notify on the responder about the response */
	MTK_WLAN_VENDOR_ATTR_NDP_RESPONDER_RESPONSE = 6,
	/* Command to initiate a NAN data path end */
	MTK_WLAN_VENDOR_ATTR_NDP_END_REQUEST = 7,
	/* Command to notify the if end request was sent */
	MTK_WLAN_VENDOR_ATTR_NDP_END_RESPONSE = 8,
	/* Command to notify the peer about the end request */
	MTK_WLAN_VENDOR_ATTR_NDP_REQUEST_IND = 9,
	/* Command to confirm the NAN data path session is complete */
	MTK_WLAN_VENDOR_ATTR_NDP_CONFIRM_IND = 10,
	/* Command to indicate the peer about the end request being received */
	MTK_WLAN_VENDOR_ATTR_NDP_END_IND = 11,
	/* Command to indicate the peer of schedule update */
	MTK_WLAN_VENDOR_ATTR_NDP_SCHEDULE_UPDATE_IND = 12
};

enum mtk_wlan_vendor_attr_ndp_params {
	MTK_WLAN_VENDOR_ATTR_NDP_PARAM_INVALID = 0,
	/* Unsigned 32-bit value */
	MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
	/* Unsigned 16-bit value */
	MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID,
	/* NL attributes for data used NDP SUB cmds
	 * Unsigned 32-bit value indicating a service info
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID,
	/* Unsigned 32-bit value; channel frequency in MHz */
	MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL,
	/* Interface Discovery MAC address. An array of 6 Unsigned int8 */
	MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR,
	/* Interface name on which NDP is being created */
	MTK_WLAN_VENDOR_ATTR_NDP_IFACE_STR,
	/* Unsigned 32-bit value for security */
	MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_SECURITY,
	/* Unsigned 32-bit value for QoS */
	MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS,
	/* Array of u8 */
	MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO,
	/* Unsigned 32-bit value for NDP instance Id */
	MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID,
	/* Array of instance Ids */
	MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY,
	/* Unsigned 32-bit value for initiator/responder NDP response code */
	MTK_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE,
	/* NDI MAC address. An array of 6 Unsigned int8 */
	MTK_WLAN_VENDOR_ATTR_NDP_NDI_MAC_ADDR,
	/* Unsigned 32-bit value errors types returned by driver
	 * wifi_nan.h in AOSP project platform/hardware/libhardware_legacy
	 * NanStatusType includes these values.
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
	/* Unsigned 32-bit value error values returned by driver */
	MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE,
	/* Unsigned 32-bit value for Channel setup configuration
	 * The wifi_nan.h in AOSP project platform/hardware/
	 * libhardware_legacy
	 * NanDataPathChannelCfg includes these values.
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_CONFIG,

	/* Unsigned 32-bit value for Cipher Suite Shared Key Type */
	MTK_WLAN_VENDOR_ATTR_NDP_CSID,
	/* Array of u8: len = NAN_PMK_INFO_LEN 32 bytes */
	MTK_WLAN_VENDOR_ATTR_NDP_PMK,
	/* Security Context Identifier that contains the PMKID
	 * Array of u8: len = NAN_SCID_BUF_LEN 1024 bytes
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_SCID,
	/* Array of u8: len = NAN_SECURITY_MAX_PASSPHRASE_LEN 63 bytes */
	MTK_WLAN_VENDOR_ATTR_NDP_PASSPHRASE,
	/* Array of u8: len = NAN_MAX_SERVICE_NAME_LEN 255 bytes */
	MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_NAME,
	/* Unsigned 32-bit bitmap indicating schedule update
	 * BIT_0: NSS Update
	 * BIT_1: Channel list update
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_SCHEDULE_UPDATE_REASON,
	/* Unsigned 32-bit value for NSS */
	MTK_WLAN_VENDOR_ATTR_NDP_NSS,
	/* Unsigned 32-bit value for NUMBER NDP CHANNEL */
	MTK_WLAN_VENDOR_ATTR_NDP_NUM_CHANNELS,
	/* Unsigned 32-bit value for CHANNEL BANDWIDTH
	 * 0:20 MHz, 1:40 MHz, 2:80 MHz, 3:160 MHz
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_WIDTH,
	/* Array of channel/band width */
	MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO,
	/* IPv6 address used by NDP (in network byte order), 16 bytes array.
	 * This attribute is used and optional for ndp request, ndp response,
	 * ndp indication, and ndp confirm.
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_IPV6_ADDR = 27,
	/* Unsigned 16-bit value indicating transport port used by NDP.
	 * This attribute is used and optional for ndp response, ndp indication,
	 * and ndp confirm.
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_TRANSPORT_PORT = 28,
	/* Unsigned 8-bit value indicating protocol used by NDP and assigned by
	 * the Internet Assigned Numbers Authority (IANA) as per:
	 * https://www.iana.org/assignments/protocol-numbers/
	 * protocol-numbers.xhtml
	 * This attribute is used and optional for ndp response, ndp indication,
	 * and ndp confirm.
	 */
	MTK_WLAN_VENDOR_ATTR_NDP_TRANSPORT_PROTOCOL = 29,
	/* keep last */
	MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_AFTER_LAST,
	MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX =
		MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_AFTER_LAST - 1,
};

enum mtk_wlan_vendor_attr_ndp_cfg_security {
	/* Security info will be added when proposed in the specification */
	MTK_WLAN_VENDOR_ATTR_NDP_SECURITY_TYPE = 1,
};

extern const struct nla_policy
	mtk_wlan_vendor_ndp_policy[MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX + 1];


/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */
uint32_t nanNdiCreateRspEvent(struct ADAPTER *prAdapter);

uint32_t nanNdiDeleteRspEvent(struct ADAPTER *prAdapter);

uint32_t nanNdpInitiatorRspEvent(struct ADAPTER *prAdapter,
				 struct _NAN_NDP_INSTANCE_T *prNDP,
				 uint32_t rTxDoneStatus);

uint32_t nanNdpResponderRspEvent(struct ADAPTER *prAdapter,
				 struct _NAN_NDP_INSTANCE_T *prNDP,
				 uint32_t rTxDoneStatus);

uint32_t nanNdpEndRspEvent(struct ADAPTER *prAdapter,
			   struct _NAN_NDP_INSTANCE_T *prNDP,
			   uint32_t rTxDoneStatus);

uint32_t nanNdiCreateHandler(struct GLUE_INFO *prGlueInfo, struct nlattr **tb);

uint32_t nanNdiDeleteHandler(struct GLUE_INFO *prGlueInfo, struct nlattr **tb);

uint32_t nanNdpInitiatorReqHandler(struct GLUE_INFO *prGlueInfo,
				   struct nlattr **tb);

uint32_t nanNdpResponderReqHandler(struct GLUE_INFO *prGlueInfo,
				   struct nlattr **tb);

uint32_t nanNdpEndReqHandler(struct GLUE_INFO *prGlueInfo, struct nlattr **tb);

uint32_t nanNdpDataIndEvent(IN struct ADAPTER *prAdapter,
			    struct _NAN_NDP_INSTANCE_T *prNDP,
			    struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanNdpDataConfirmEvent(IN struct ADAPTER *prAdapter,
				struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanNdpDataTerminationEvent(IN struct ADAPTER *prAdapter,
				    struct _NAN_NDP_INSTANCE_T *prNDP);

int mtk_cfg80211_vendor_ndp(struct wiphy *wiphy, struct wireless_dev *wdev,
			    const void *data, int data_len);

#endif /* CFG_SUPPORT_NAN */
#endif /* _GL_VENDOR_NDP_H_ */
