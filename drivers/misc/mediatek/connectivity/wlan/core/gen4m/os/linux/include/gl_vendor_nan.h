/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

/*
 * Log: gl_vendor_nan.h
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
#define PACKED __packed
/* 8-byte control message header used by NAN */
struct _NanMsgHeader {
	u16 msgVersion : 4;
	u16 msgId : 12;
	u16 msgLen;
	u16 handle;
	u16 transactionId;
} PACKED;

struct _NanTlv  {
	u16 type;
	u16 length;
	u8 *value;
} PACKED;

/* NAN Enable Rsp */
struct NanEnableRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 value;
} PACKED;

/* NAN Disable Rsp */
struct NanDisableRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 reserved;
} PACKED;

/* NAN Configuration Rsp */
struct NanConfigRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 value;
} PACKED;

/* NAN Publish Response */
struct NanPublishServiceRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 value;
} PACKED;

/* NAN Publish Cancel Rsp */
struct NanPublishServiceCancelRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 value;
} PACKED;

/* NAN Subscribe Service Rsp */
struct NanSubscribeServiceRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 value;
} PACKED;

struct NanSubscribeServiceCancelRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 value;
} PACKED;

/* NAN Transmit Followup Rsp */
struct NanTransmitFollowupRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 value;
} PACKED;

/* NAN Capabilities Rsp */
struct NanCapabilitiesRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u32 status;
	u32 value;
	u32 max_concurrent_nan_clusters;
	u32 max_publishes;
	u32 max_subscribes;
	u32 max_service_name_len;
	u32 max_match_filter_len;
	u32 max_total_match_filter_len;
	u32 max_service_specific_info_len;
	u32 max_vsa_data_len;
	u32 max_mesh_data_len;
	u32 max_ndi_interfaces;
	u32 max_ndp_sessions;
	u32 max_app_info_len;
	u32 max_queued_transmit_followup_msgs;
	u32 ndp_supported_bands;
	u32 cipher_suites_supported;
	u32 max_scid_len;
	u32 is_ndp_security_supported : 1;
	u32 max_sdea_service_specific_info_len : 16;
	u32 reserved1 : 5;
	u32 reserved2 : 5;
	u32 ndpe_attr_supported : 1;
	u32 reserved : 4;
	u32 max_subscribe_address;
} PACKED;

/* NAN Beacon Sdf Payload Rsp */
struct NanBeaconSdfPayloadRspMsg {
	struct _NanMsgHeader fwHeader;
	/* status of the request */
	u16 status;
	u16 reserved;
} PACKED;

/* Indication Part */
/* Params for NAN Publish Replied Ind */
struct _NanPublishRepliedIndParams  {u32 matchHandle; } PACKED;

/* NAN Publish Replied Ind */
struct NanPublishRepliedIndMsg {
	struct _NanMsgHeader fwHeader;
	struct _NanPublishRepliedIndParams publishRepliedIndParams;
	/*
     * Excludes TLVs
     *
	 * Required: MAC Address
	 * Optional: Received RSSI Value
	 *
	 */
	u8 ptlv[];
} PACKED;

/* NAN Publish Terminated Ind */
struct NanPublishTerminatedIndMsg {
	struct _NanMsgHeader fwHeader;
	/* reason for the termination */
	u16 reason;
	u16 reserved;
} PACKED;

/* NAN Subscribe Terminated Ind */
struct NanSubscribeTerminatedIndMsg {
	struct _NanMsgHeader fwHeader;
	/* reason for the termination */
	u16 reason;
	u16 reserved;
} PACKED;

/* NAN Publish Followup Ind */
struct _NanFollowupIndParams {
	u32 matchHandle;
	u32 window : 1;
	u32 reserved : 31;
	/*
	 * Excludes TLVs
	 *
	 * Required: Service Specific Info or
	 * Extended Service Specific Info
	 */
} PACKED;

struct NanFollowupIndMsg {
	struct _NanMsgHeader fwHeader;
	struct _NanFollowupIndParams followupIndParams;
	u8 ptlv[];
} PACKED;

/* Event Ind */
struct NanEventIndParams {
	u32 eventId : 8;
	u32 reserved : 24;
} PACKED;

struct NanEventIndMsg {
	struct _NanMsgHeader fwHeader;
	u8 ptlv[];
} PACKED;

/* NAN Subscribe Match Ind */
struct _NanMatchIndParams  {
	u32 matchHandle;
	u32 matchOccuredFlag : 1;
	u32 outOfResourceFlag : 1;
	u32 reserved : 30;
} PACKED;

struct NanMatchIndMsg {
	struct _NanMsgHeader fwHeader;
	struct _NanMatchIndParams matchIndParams;
	u8 ptlv[];
} PACKED;

/* NAN Ranging Configuration params */
struct _NanFWGeoFenceDescriptor {
	u32 inner_threshold;
	u32 outer_threshold;
} PACKED;

struct NanFWRangeConfigParams {
	u32 range_resolution;
	u32 range_interval;
	u32 ranging_indication_event;
	struct _NanFWGeoFenceDescriptor geo_fence_threshold;
} PACKED;

/* 2 word representation of MAC addr */
struct _fw_mac_addr {
	/* upper 4 bytes of  MAC address */
	u32 mac_addr31to0;
	/* lower 2 bytes of  MAC address */
	u32 mac_addr47to32;
};

struct NanFWRangeReqMsg {
	struct _fw_mac_addr range_mac_addr;
	u32 range_id; /* Match handle in match_ind, publish_id in result ind */
	u32 ranging_accept : 1;
	u32 ranging_reject : 1;
	u32 ranging_cancel : 1;
	u32 reserved : 29;
} PACKED;

struct NanDebugParams {
    /* To indicate the debug command type. */
	u32 cmd;
    /* To hold the data for the above command
     * type.
     */
	u8 debug_cmd_data[NAN_MAX_DEBUG_MESSAGE_DATA_LEN];
} PACKED;

extern const struct nla_policy mtk_wlan_vendor_nan_policy[NL80211_ATTR_MAX + 1];

/* Service Discovery Extended Attribute params Format to HAL */
struct NanFWSdeaCtrlParams {
	u32 fsd_required : 1;
	u32 fsd_with_gas : 1;
	u32 data_path_required : 1;
	u32 data_path_type : 1;
	u32 multicast_type : 1;
	u32 qos_required : 1;
	u32 security_required : 1;
	u32 ranging_required : 1;
	u32 range_limit_present : 1;
	u32 service_update_ind_present : 1;
	u32 reserved1 : 6;
	u32 range_report : 1;
	u32 reserved2 : 15;
} PACKED;

enum NanMsgId {
	NAN_MSG_ID_ERROR_RSP = 0,
	NAN_MSG_ID_CONFIGURATION_REQ = 1,
	NAN_MSG_ID_CONFIGURATION_RSP = 2,
	NAN_MSG_ID_PUBLISH_SERVICE_REQ = 3,
	NAN_MSG_ID_PUBLISH_SERVICE_RSP = 4,
	NAN_MSG_ID_PUBLISH_SERVICE_CANCEL_REQ = 5,
	NAN_MSG_ID_PUBLISH_SERVICE_CANCEL_RSP = 6,
	NAN_MSG_ID_PUBLISH_REPLIED_IND = 7,
	NAN_MSG_ID_PUBLISH_TERMINATED_IND = 8,
	NAN_MSG_ID_SUBSCRIBE_SERVICE_REQ = 9,
	NAN_MSG_ID_SUBSCRIBE_SERVICE_RSP = 10,
	NAN_MSG_ID_SUBSCRIBE_SERVICE_CANCEL_REQ = 11,
	NAN_MSG_ID_SUBSCRIBE_SERVICE_CANCEL_RSP = 12,
	NAN_MSG_ID_MATCH_IND = 13,
	NAN_MSG_ID_MATCH_EXPIRED_IND = 14,
	NAN_MSG_ID_SUBSCRIBE_TERMINATED_IND = 15,
	NAN_MSG_ID_DE_EVENT_IND = 16,
	NAN_MSG_ID_TRANSMIT_FOLLOWUP_REQ = 17,
	NAN_MSG_ID_TRANSMIT_FOLLOWUP_RSP = 18,
	NAN_MSG_ID_FOLLOWUP_IND = 19,
	NAN_MSG_ID_STATS_REQ = 20,
	NAN_MSG_ID_STATS_RSP = 21,
	NAN_MSG_ID_ENABLE_REQ = 22,
	NAN_MSG_ID_ENABLE_RSP = 23,
	NAN_MSG_ID_DISABLE_REQ = 24,
	NAN_MSG_ID_DISABLE_RSP = 25,
	NAN_MSG_ID_DISABLE_IND = 26,
	NAN_MSG_ID_TCA_REQ = 27,
	NAN_MSG_ID_TCA_RSP = 28,
	NAN_MSG_ID_TCA_IND = 29,
	NAN_MSG_ID_BEACON_SDF_REQ = 30,
	NAN_MSG_ID_BEACON_SDF_RSP = 31,
	NAN_MSG_ID_BEACON_SDF_IND = 32,
	NAN_MSG_ID_CAPABILITIES_REQ = 33,
	NAN_MSG_ID_CAPABILITIES_RSP = 34,
	NAN_MSG_ID_SELF_TRANSMIT_FOLLOWUP_IND = 35,
	NAN_MSG_ID_RANGING_REQUEST_RECEVD_IND = 36,
	NAN_MSG_ID_RANGING_RESULT_IND = 37,
	NAN_MSG_ID_TESTMODE_REQ = 1025,
	NAN_MSG_ID_TESTMODE_RSP = 1026
};

/* Various TLV Type ID sent as part of NAN Stats Response
 * or NAN TCA Indication
 */
enum NanTlvType {
	NAN_TLV_TYPE_FIRST = 0,

	/* Service Discovery Frame types */
	NAN_TLV_TYPE_SDF_FIRST = NAN_TLV_TYPE_FIRST,
	NAN_TLV_TYPE_SERVICE_NAME = NAN_TLV_TYPE_SDF_FIRST,
	NAN_TLV_TYPE_SDF_MATCH_FILTER,
	NAN_TLV_TYPE_TX_MATCH_FILTER,
	NAN_TLV_TYPE_RX_MATCH_FILTER,
	NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO,
	NAN_TLV_TYPE_EXT_SERVICE_SPECIFIC_INFO = 5,
	NAN_TLV_TYPE_VENDOR_SPECIFIC_ATTRIBUTE_TRANSMIT = 6,
	NAN_TLV_TYPE_VENDOR_SPECIFIC_ATTRIBUTE_RECEIVE = 7,
	NAN_TLV_TYPE_POST_NAN_CONNECTIVITY_CAPABILITIES_RECEIVE = 8,
	NAN_TLV_TYPE_POST_NAN_DISCOVERY_ATTRIBUTE_RECEIVE = 9,
	NAN_TLV_TYPE_BEACON_SDF_PAYLOAD_RECEIVE = 10,
	NAN_TLV_TYPE_NAN_DATA_PATH_PARAMS = 11,
	NAN_TLV_TYPE_NAN_DATA_SUPPORTED_BAND = 12,
	NAN_TLV_TYPE_2G_COMMITTED_DW = 13,
	NAN_TLV_TYPE_5G_COMMITTED_DW = 14,
	NAN_TLV_TYPE_NAN_DATA_RESPONDER_MODE = 15,
	NAN_TLV_TYPE_NAN_DATA_ENABLED_IN_MATCH = 16,
	NAN_TLV_TYPE_NAN_SERVICE_ACCEPT_POLICY = 17,
	NAN_TLV_TYPE_NAN_CSID = 18,
	NAN_TLV_TYPE_NAN_SCID = 19,
	NAN_TLV_TYPE_NAN_PMK = 20,
	NAN_TLV_TYPE_SDEA_CTRL_PARAMS = 21,
	NAN_TLV_TYPE_NAN_RANGING_CFG = 22,
	NAN_TLV_TYPE_CONFIG_DISCOVERY_INDICATIONS = 23,
	NAN_TLV_TYPE_NAN20_RANGING_REQUEST = 24,
	NAN_TLV_TYPE_NAN20_RANGING_RESULT = 25,
	NAN_TLV_TYPE_NAN20_RANGING_REQUEST_RECEIVED = 26,
	NAN_TLV_TYPE_NAN_PASSPHRASE = 27,
	NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO = 28,
	NAN_TLV_TYPE_DEV_CAP_ATTR_CAPABILITY = 29,
	NAN_TLV_TYPE_SDF_LAST = 4095,

	/* Configuration types */
	NAN_TLV_TYPE_CONFIG_FIRST = 4096,
	NAN_TLV_TYPE_24G_SUPPORT = NAN_TLV_TYPE_CONFIG_FIRST,
	NAN_TLV_TYPE_24G_BEACON,
	NAN_TLV_TYPE_24G_SDF,
	NAN_TLV_TYPE_24G_RSSI_CLOSE,
	NAN_TLV_TYPE_24G_RSSI_MIDDLE = 4100,
	NAN_TLV_TYPE_24G_RSSI_CLOSE_PROXIMITY,
	NAN_TLV_TYPE_5G_SUPPORT,
	NAN_TLV_TYPE_5G_BEACON,
	NAN_TLV_TYPE_5G_SDF,
	NAN_TLV_TYPE_5G_RSSI_CLOSE,
	NAN_TLV_TYPE_5G_RSSI_MIDDLE,
	NAN_TLV_TYPE_5G_RSSI_CLOSE_PROXIMITY,
	NAN_TLV_TYPE_SID_BEACON,
	NAN_TLV_TYPE_HOP_COUNT_LIMIT,
	NAN_TLV_TYPE_MASTER_PREFERENCE = 4110,
	NAN_TLV_TYPE_CLUSTER_ID_LOW,
	NAN_TLV_TYPE_CLUSTER_ID_HIGH,
	NAN_TLV_TYPE_RSSI_AVERAGING_WINDOW_SIZE,
	NAN_TLV_TYPE_CLUSTER_OUI_NETWORK_ID,
	NAN_TLV_TYPE_SOURCE_MAC_ADDRESS,
	NAN_TLV_TYPE_CLUSTER_ATTRIBUTE_IN_SDF,
	NAN_TLV_TYPE_SOCIAL_CHANNEL_SCAN_PARAMS,
	NAN_TLV_TYPE_DEBUGGING_FLAGS,
	NAN_TLV_TYPE_POST_NAN_CONNECTIVITY_CAPABILITIES_TRANSMIT,
	NAN_TLV_TYPE_POST_NAN_DISCOVERY_ATTRIBUTE_TRANSMIT = 4120,
	NAN_TLV_TYPE_FURTHER_AVAILABILITY_MAP,
	NAN_TLV_TYPE_HOP_COUNT_FORCE,
	NAN_TLV_TYPE_RANDOM_FACTOR_FORCE,
	NAN_TLV_TYPE_RANDOM_UPDATE_TIME = 4124,
	NAN_TLV_TYPE_EARLY_WAKEUP,
	NAN_TLV_TYPE_PERIODIC_SCAN_INTERVAL,
	NAN_TLV_TYPE_DW_INTERVAL = 4128,
	NAN_TLV_TYPE_DB_INTERVAL,
	NAN_TLV_TYPE_FURTHER_AVAILABILITY,
	NAN_TLV_TYPE_24G_CHANNEL,
	NAN_TLV_TYPE_5G_CHANNEL,
	NAN_TLV_TYPE_DISC_MAC_ADDR_RANDOM_INTERVAL,
	NAN_TLV_TYPE_RANGING_AUTO_RESPONSE_CFG = 4134,
	NAN_TLV_TYPE_SUBSCRIBE_SID_BEACON = 4135,
	NAN_TLV_TYPE_DW_EARLY_TERMINATION = 4136,
	NAN_TLV_TYPE_TX_RX_CHAINS = 4137,
	NAN_TLV_TYPE_ENABLE_DEVICE_RANGING = 4138,
	NAN_TLV_TYPE_CONFIG_LAST = 8191,

	/* Attributes types */
	NAN_TLV_TYPE_ATTRS_FIRST = 8192,
	NAN_TLV_TYPE_AVAILABILITY_INTERVALS_MAP = NAN_TLV_TYPE_ATTRS_FIRST,
	NAN_TLV_TYPE_WLAN_MESH_ID,
	NAN_TLV_TYPE_MAC_ADDRESS,
	NAN_TLV_TYPE_RECEIVED_RSSI_VALUE,
	NAN_TLV_TYPE_CLUSTER_ATTRIBUTE,
	NAN_TLV_TYPE_WLAN_INFRA_SSID,
	NAN_TLV_TYPE_ATTRS_LAST = 12287,

	/* Events Type */
	NAN_TLV_TYPE_EVENTS_FIRST = 12288,
	NAN_TLV_TYPE_EVENT_SELF_STATION_MAC_ADDRESS = NAN_TLV_TYPE_EVENTS_FIRST,
	NAN_TLV_TYPE_EVENT_STARTED_CLUSTER,
	NAN_TLV_TYPE_EVENT_JOINED_CLUSTER,
	NAN_TLV_TYPE_EVENT_CLUSTER_SCAN_RESULTS,
	NAN_TLV_TYPE_FAW_MEM_AVAIL,
	NAN_TLV_TYPE_EVENTS_LAST = 16383,

	/* TCA types */
	NAN_TLV_TYPE_TCA_FIRST = 16384,
	NAN_TLV_TYPE_CLUSTER_SIZE_REQ = NAN_TLV_TYPE_TCA_FIRST,
	NAN_TLV_TYPE_CLUSTER_SIZE_RSP,
	NAN_TLV_TYPE_TCA_LAST = 32767,

	/* Statistics types */
	NAN_TLV_TYPE_STATS_FIRST = 32768,
	NAN_TLV_TYPE_DE_PUBLISH_STATS = NAN_TLV_TYPE_STATS_FIRST,
	NAN_TLV_TYPE_DE_SUBSCRIBE_STATS,
	NAN_TLV_TYPE_DE_MAC_STATS,
	NAN_TLV_TYPE_DE_TIMING_SYNC_STATS,
	NAN_TLV_TYPE_DE_DW_STATS,
	NAN_TLV_TYPE_DE_STATS,
	NAN_TLV_TYPE_STATS_LAST = 36863,

	/* Testmode types */
	NAN_TLV_TYPE_TESTMODE_FIRST = 36864,
	NAN_TLV_TYPE_TESTMODE_GENERIC_CMD = NAN_TLV_TYPE_TESTMODE_FIRST,
	NAN_TLV_TYPE_TESTMODE_LAST = 37000,

	NAN_TLV_TYPE_LAST = 65535
};

/* Definitions of debug subcommand type for the
 * generic debug command.
 * This debug command carries any one command type
 * followed by corresponding command data content
 * as indicated below.
 *
 * command: NAN_TEST_MODE_CMD_NAN_AVAILABILITY
 * content: NAN Avaiability attribute blob
 *
 * command: NAN_TEST_MODE_CMD_NDP_INCLUDE_IMMUTABLE
 * content: u32 value (0 - Ignore 1 - Include immuatable,
 *                     2 - Don't include immutable)
 *
 * command: NAN_TEST_MODE_CMD_NDP_AVOID_CHANNEL
 * content: u32 channel_frequency; (0 - Ignore)
 *
 * command: NAN_TEST_MODE_CMD_NAN_SUPPORTED_BANDS
 * content: u32 supported_bands; (0 . Ignore, 1 . 2g,
 *                                2 . 5g, 3 . 2g & 5g)
 *
 * command: NAN_TEST_MODE_CMD_AUTO_RESPONDER_MODE
 * content: u32 auto_resp_mode; (0 . Auto, 1 . Accept,
 *                               2 . Reject, 3 . Counter)
 *
 * command: NAN_TEST_MODE_CMD_M4_RESPONSE_TYPE
 * content: u32 m4_response_type; (0.Ignore, 1.Accept,
 *                                 2.Reject, 3.BadMic)
 *
 * command: NAN_TEST_MODE_CMD_NAN_SCHED_TYPE
 * content: u32 invalid_nan_schedule; (0. Valid sched,
 *                                     1.Invalid Sched bad FA,
 *                                     2.Invalid schedbad NDC,
 *                                     3.Invalid sched bad Immutable)
 *
 * command: NAN_TEST_MODE_CMD_NAN_NMF_CLEAR_CONFIG
 * content: u32 nmf_security_config_val;(0:NAN_NMF_CLEAR_DISABLE,
 *                                       1:NAN_NMF_CLEAR_ENABLE)
 *
 * command: NAN_TEST_MODE_CMD_NAN_SCHED_UPDATE_ULW_NOTIFY
 * content: u32 channel_availability;(0/1)
 *
 * command: NAN_TEST_MODE_CMD_NAN_SCHED_UPDATE_NDL_NEGOTIATE
 * content: responder_nmi_mac (Responder NMI Mac Address)
 *
 * command: NAN_TEST_MODE_CMD_NAN_SCHED_UPDATE_NDL_NOTIFY
 * content: NONE
 *
 * command: NAN_TEST_MODE_CMD_NAN_AVAILABILITY_MAP_ORDER
 * content: u32 map_order_val; (0/1)
 *
 */
enum NanDebugModeCmd {
	NAN_TEST_MODE_CMD_NAN_AVAILABILITY = 1,
	NAN_TEST_MODE_CMD_NDP_INCLUDE_IMMUTABLE = 2,
	NAN_TEST_MODE_CMD_NDP_AVOID_CHANNEL = 3,
	NAN_TEST_MODE_CMD_NAN_SUPPORTED_BANDS = 4,
	NAN_TEST_MODE_CMD_AUTO_RESPONDER_MODE = 5,
	NAN_TEST_MODE_CMD_M4_RESPONSE_TYPE = 6,
	NAN_TEST_MODE_CMD_NAN_SCHED_TYPE = 7,
	NAN_TEST_MODE_CMD_NAN_NMF_CLEAR_CONFIG = 8,
	NAN_TEST_MODE_CMD_NAN_SCHED_UPDATE_ULW_NOTIFY = 9,
	NAN_TEST_MODE_CMD_NAN_SCHED_UPDATE_NDL_NEGOTIATE = 10,
	NAN_TEST_MODE_CMD_NAN_SCHED_UPDATE_NDL_NOTIFY = 11,
	NAN_TEST_MODE_CMD_NAN_AVAILABILITY_MAP_ORDER = 12,
	NAN_TEST_MODE_CMD_CONFIG_QOS = 13,
	NAN_TEST_MODE_CMD_DEVICE_TYPE = 14,
	NAN_TEST_MODE_CMD_DISABLE_NDPE = 15,
	NAN_TEST_MODE_CMD_ENABLE_NDP = 16,
};

/* NAN Resp status type */
enum NanInternalStatusType {
	/* NAN Protocol Response Codes */
	NAN_I_STATUS_SUCCESS = 0,
	NAN_I_STATUS_TIMEOUT = 1,
	NAN_I_STATUS_DE_FAILURE = 2,
	NAN_I_STATUS_INVALID_MSG_VERSION = 3,
	NAN_I_STATUS_INVALID_MSG_LEN = 4,
	NAN_I_STATUS_INVALID_MSG_ID = 5,
	NAN_I_STATUS_INVALID_HANDLE = 6,
	NAN_I_STATUS_NO_SPACE_AVAILABLE = 7,
	NAN_I_STATUS_INVALID_PUBLISH_TYPE = 8,
	NAN_I_STATUS_INVALID_TX_TYPE = 9,
	NAN_I_STATUS_INVALID_MATCH_ALGORITHM = 10,
	NAN_I_STATUS_DISABLE_IN_PROGRESS = 11,
	NAN_I_STATUS_INVALID_TLV_LEN = 12,
	NAN_I_STATUS_INVALID_TLV_TYPE = 13,
	NAN_I_STATUS_MISSING_TLV_TYPE = 14,
	NAN_I_STATUS_INVALID_TOTAL_TLVS_LEN = 15,
	NAN_I_STATUS_INVALID_REQUESTER_INSTANCE_ID = 16,
	NAN_I_STATUS_INVALID_TLV_VALUE = 17,
	NAN_I_STATUS_INVALID_TX_PRIORITY = 18,
	NAN_I_STATUS_INVALID_CONNECTION_MAP = 19,
	NAN_I_STATUS_INVALID_THRESHOLD_CROSSING_ALERT_ID = 20,
	NAN_I_STATUS_INVALID_STATS_ID = 21,
	NAN_I_STATUS_NAN_NOT_ALLOWED = 22,
	NAN_I_STATUS_NO_OTA_ACK = 23,
	NAN_I_STATUS_TX_FAIL = 24,
	NAN_I_STATUS_NAN_ALREADY_ENABLED = 25,
	NAN_I_STATUS_FOLLOWUP_QUEUE_FULL = 26,
	/* 27-4095 Reserved */
	/* NAN Configuration Response codes */
	NAN_I_STATUS_INVALID_RSSI_CLOSE_VALUE = 4096,
	NAN_I_STATUS_INVALID_RSSI_MIDDLE_VALUE = 4097,
	NAN_I_STATUS_INVALID_HOP_COUNT_LIMIT = 4098,
	NAN_I_STATUS_INVALID_MASTER_PREFERENCE_VALUE = 4099,
	NAN_I_STATUS_INVALID_LOW_CLUSTER_ID_VALUE = 4100,
	NAN_I_STATUS_INVALID_HIGH_CLUSTER_ID_VALUE = 4101,
	NAN_I_STATUS_INVALID_BACKGROUND_SCAN_PERIOD = 4102,
	NAN_I_STATUS_INVALID_RSSI_PROXIMITY_VALUE = 4103,
	NAN_I_STATUS_INVALID_SCAN_CHANNEL = 4104,
	NAN_I_STATUS_INVALID_POST_NAN_CONNECTIVITY_CAPABILITIES_BITMAP = 4105,
	NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_NUMCHAN_VALUE = 4106,
	NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_DURATION_VALUE = 4107,
	NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_CLASS_VALUE = 4108,
	NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_CHANNEL_VALUE = 4109,
	NAN_I_STATUS_INVALID_FURTHER_AVAIL_MAP_AVAIL_INTERVAL_BITMAP_VALUE =
		4110,
	NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_MAP_ID = 4111,
	NAN_I_STATUS_INVALID_POST_NAN_DISCOVERY_CONN_TYPE_VALUE = 4112,
	NAN_I_STATUS_INVALID_POST_NAN_DISCOVERY_DEVICE_ROLE_VALUE = 4113,
	NAN_I_STATUS_INVALID_POST_NAN_DISCOVERY_DURATION_VALUE = 4114,
	NAN_I_STATUS_INVALID_POST_NAN_DISCOVERY_BITMAP_VALUE = 4115,
	NAN_I_STATUS_MISSING_FUTHER_AVAILABILITY_MAP = 4116,
	NAN_I_STATUS_INVALID_BAND_CONFIG_FLAGS = 4117,
	NAN_I_STATUS_INVALID_RANDOM_FACTOR_UPDATE_TIME_VALUE = 4118,
	NAN_I_STATUS_INVALID_ONGOING_SCAN_PERIOD = 4119,
	NAN_I_STATUS_INVALID_DW_INTERVAL_VALUE = 4120,
	NAN_I_STATUS_INVALID_DB_INTERVAL_VALUE = 4121,
	/* 4122-8191 RESERVED */
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_INVALID = 8192,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_TIMEOUT = 8193,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_USER_REQUEST = 8194,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_FAILURE = 8195,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_COUNT_REACHED = 8196,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_DE_SHUTDOWN = 8197,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_DISABLE_IN_PROGRESS = 8198,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_POST_DISC_ATTR_EXPIRED = 8199,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_POST_DISC_LEN_EXCEEDED = 8200,
	NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_FURTHER_AVAIL_MAP_EMPTY =
		8201,
	/* 9000-9500 NDP Status type */
	NDP_I_UNSUPPORTED_CONCURRENCY = 9000,
	NDP_I_NAN_DATA_IFACE_CREATE_FAILED = 9001,
	NDP_I_NAN_DATA_IFACE_DELETE_FAILED = 9002,
	NDP_I_DATA_INITIATOR_REQUEST_FAILED = 9003,
	NDP_I_DATA_RESPONDER_REQUEST_FAILED = 9004,
	NDP_I_INVALID_SERVICE_INSTANCE_ID = 9005,
	NDP_I_INVALID_NDP_INSTANCE_ID = 9006,
	NDP_I_INVALID_RESPONSE_CODE = 9007,
	NDP_I_INVALID_APP_INFO_LEN = 9008,
	/* OTA failures and timeouts during negotiation */
	NDP_I_MGMT_FRAME_REQUEST_FAILED = 9009,
	NDP_I_MGMT_FRAME_RESPONSE_FAILED = 9010,
	NDP_I_MGMT_FRAME_CONFIRM_FAILED = 9011,
	NDP_I_END_FAILED = 9012,
	NDP_I_MGMT_FRAME_END_REQUEST_FAILED = 9013,
	NDP_I_MGMT_FRAME_SECURITY_INSTALL_FAILED = 9014,

	/* 9500 onwards vendor specific error codes */
	NDP_I_VENDOR_SPECIFIC_ERROR = 9500
};

/* #define SIZEOF_TLV_HDR (sizeof(NanTlv.type) + sizeof(NanTlv.length)) */
#define SIZEOF_TLV_HDR                                                  \
	(sizeof(((struct _NanTlv *)0)->type) + \
	sizeof(((struct _NanTlv *)0)->length))

/* NAN ranging indication condition MASKS */
#define NAN_RANGING_INDICATE_CONTINUOUS_MASK 0x01
#define NAN_RANGING_INDICATE_INGRESS_MET_MASK 0x02
#define NAN_RANGING_INDICATE_EGRESS_MET_MASK 0x04

/* SDEA CTRL Parameters BIT Mapping*/
#define SDEA_CTRL_PARMS_FSD_REQUIRED BIT(0)
#define SDEA_CTRL_PARMS_FSD_WITH_GAS BIT(1)
#define SDEA_CTRL_PARMS_DATA_PATH_REQUIRED BIT(2)
#define SDEA_CTRL_PARMS_DATA_PATH_TYPE BIT(3)
#define SDEA_CTRL_PARMS_MULTICAST_TYPE BIT(4)
#define SDEA_CTRL_PARMS_QOS_REQUIRED BIT(5)
#define SDEA_CTRL_PARMS_SECURITY_REQUIRED BIT(6)
#define SDEA_CTRL_PARMS_RANGING_REQUIRED BIT(7)
#define SDEA_CTRL_PARMS_RANGE_LIMIT_PRESENT BIT(8)
#define SDEA_CTRL_PARMS_SERVICE_UPDATE_IND_PRESENT BIT(9)
#define SDEA_CTRL_PARMS_RESERVED_1 BITS(10, 15)
#define SDEA_CTRL_PARMS_RANGE_REPORT BIT(16)
#define SDEA_CTRL_PARMS_RESERVED_2 BIT(17, 31)

/* Publish Service Req parameters bit map */
#define PUB_RESERVED BITS(27, 31)
#define PUB_FOLLOWUP_RX_IND_DISABLE_FLAG BIT(26)
#define PUB_MATCH_EXPIRED_IND_DISABLE_FLAG BIT(25)
#define PUB_TERMINATED_IND_DISABLE_FLAG BIT(24)
#define PUB_CONNMAP BITS(16, 23)
#define PUB_COUNT BITS(8, 15)
#define PUB_MATCH_ALG BITS(6, 7)
#define PUB_OTA_FLAG BIT(5)
#define PUB_RSSI_THRESHOLD_FLAG BIT(4)
#define PUB_TX_TYPE BIT(3)
#define PUB_PUBLISH_TYPE BITS(1, 2)
#define PUB_REPLY_IND_FLAG BIT(0)

/* Subscribe Service Req parameters bit map */
#define SUB_CONNMAP BITS(24, 31)
#define SUB_RESERVED BITS(21, 23)
#define SUB_FOLLOWUP_RX_IND_DISABLE_FLAG BIT(20)
#define SUB_MATCH_EXPIRED_IND_DISABLE_FLAG BIT(19)
#define SUB_TERMINATED_IND_DISABLE_FLAG BIT(18)
#define SUB_OTA_FLAG BIT(17)
#define SUB_RSSI_THRESHOLD_FLAG BIT(16)
#define SUB_COUNT BITS(8, 15)
#define SUB_XBIT BIT(7)
#define SUB_MATCH_ALG BITS(5, 6)
#define SUB_SSI_REQUIRED BIT(4)
#define SUB_SRF_SEND BIT(3)
#define SUB_SRF_INCLUDE BIT(2)
#define SUB_SRF_ATTR BIT(1)
#define SUB_SUBSCRIBE_TYPE BIT(0)

/* Transmit Follow up Request parameters bit map */
#define FLWUP_RESERVED BITS(6, 31)
#define FLWUP_TX_RSP_DISABLE_FLAG BIT(5)
#define FLWUP_WINDOW BIT(4)
#define FLWUP_PRIORITY BITS(0, 3)

/* Get SDEA Ctrl parameters */
#define GET_SDEA_DATA_PATH_REQUIRED(flags)                                     \
	((flags & SDEA_CTRL_PARMS_DATA_PATH_REQUIRED) >>                       \
	 2) /* config_nan_data_path */
#define GET_SDEA_DATA_PATH_TYPE(flags)                                         \
	((flags & SDEA_CTRL_PARMS_DATA_PATH_TYPE) >> 3) /* ndp_type */
#define GET_SDEA_SECURITY_REQUIRED(flags)                                      \
	((flags & SDEA_CTRL_PARMS_SECURITY_REQUIRED) >> 6) /* security_cfg */
#define GET_SDEA_RANGING_REQUIRED(flags)                                       \
	((flags & SDEA_CTRL_PARMS_RANGING_REQUIRED) >> 7) /* ranging_state */
#define GET_SDEA_RANGE_REPORT(flags)                                           \
	((flags & SDEA_CTRL_PARMS_RANGE_REPORT) >> 16) /* range_report */
#define GET_SDEA_QOS_REQUIRED(flags)                                           \
	((flags & SDEA_CTRL_PARMS_QOS_REQUIRED) >> 5) /* fgQoS */
/* Below define is not used now */
#define GET_SDEA_FSD_REQUIRED(flags)                                           \
	(flags & SDEA_CTRL_PARMS_FSD_REQUIRED) /* fgFSDRequire */
#define GET_SDEA_FSD_WITH_GAS(flags)                                           \
	((flags & SDEA_CTRL_PARMS_FSD_WITH_GAS) >> 1) /* fgGAS */
#define GET_SDEA_RANGE_LIMIT_PRESENT(flags)                                    \
	((flags & SDEA_CTRL_PARMS_RANGE_LIMIT_PRESENT) >> 8) /* fgRangeLimit */

/* Get flags
 * BIT0 - Disable publish termination indication.
 * BIT1 - Disable match expired indication.
 * BIT2 - Disable followUp indication received (OTA).
 * BIT3 - Disable publishReplied indication.
 */
#define GET_PUB_REPLY_IND_FLAG(flags) ((flags & PUB_REPLY_IND_FLAG) << 3)
#define GET_PUB_FOLLOWUP_RX_IND_DISABLE_FLAG(flags)                            \
	(((flags & PUB_FOLLOWUP_RX_IND_DISABLE_FLAG) >> 26) << 2)
#define GET_PUB_MATCH_EXPIRED_IND_DISABLE_FLAG(flags)                          \
	(((flags & PUB_MATCH_EXPIRED_IND_DISABLE_FLAG) >> 25) << 1)
#define GET_PUB_TERMINATED_IND_DISABLE_FLAG(flags)                             \
	((flags & PUB_TERMINATED_IND_DISABLE_FLAG) >> 24)
#define GET_PUB_CONNMAP(flags) ((flags & PUB_CONNMAP) >> 16)
#define GET_PUB_COUNT(flags) ((flags & PUB_COUNT) >> 8)
#define GET_PUB_MATCH_ALG(flags) ((flags & PUB_MATCH_ALG) >> 6)
#define GET_PUB_OTA_FLAG(flags) ((flags & PUB_OTA_FLAG) >> 5)
#define GET_PUB_RSSI_THRESHOLD_FLAG(flags)                                     \
	((flags & PUB_RSSI_THRESHOLD_FLAG) >> 4)
#define GET_PUB_TX_TYPE(flags) ((flags & PUB_TX_TYPE) >> 3)
#define GET_PUB_PUBLISH_TYPE(flags) ((flags & PUB_PUBLISH_TYPE) >> 1)

/* Get Subscribe Request Parameters */
#define GET_SUB_CONNMAP(flags) ((flags & SUB_CONNMAP) >> 24)

#define GET_SUB_FOLLOWUP_RX_IND_DISABLE_FLAG(flags)                            \
	(((flags & SUB_FOLLOWUP_RX_IND_DISABLE_FLAG) >> 20) << 2)
#define GET_SUB_MATCH_EXPIRED_IND_DISABLE_FLAG(flags)                          \
	(((flags & SUB_MATCH_EXPIRED_IND_DISABLE_FLAG) >> 19) << 1)
#define GET_SUB_TERMINATED_IND_DISABLE_FLAG(flags)                             \
	(((flags & SUB_TERMINATED_IND_DISABLE_FLAG) >> 18))
#define GET_SUB_OTA_FLAG(flags) ((flags & SUB_OTA_FLAG) >> 17)
#define GET_SUB_RSSI_THRESHOLD_FLAG(flags)                                     \
	((flags & SUB_RSSI_THRESHOLD_FLAG) >> 16)
#define GET_SUB_COUNT(flags) ((flags & SUB_COUNT) >> 8)
#define GET_SUB_XBIT(flags) ((flags & SUB_XBIT) >> 7)
#define GET_SUB_MATCH_ALG(flags) ((flags & SUB_MATCH_ALG) >> 5)
#define GET_SUB_SSI_REQUIRED(flags) ((flags & SUB_SSI_REQUIRED) >> 4)
#define GET_SUB_SRF_SEND(flags) ((flags & SUB_SRF_SEND) >> 3)
#define GET_SUB_SRF_INCLUDE(flags) ((flags & SUB_SRF_INCLUDE) >> 2)
#define GET_SUB_SRF_ATTR(flags) ((flags & SUB_SRF_ATTR) >> 1)
#define GET_SUB_SUBSCRIBE_TYPE(flags) (flags & SUB_SUBSCRIBE_TYPE)

/* Get Transmit Follow up Request Parameters */
#define GET_FLWUP_PRIORITY(flags) (flags & FLWUP_PRIORITY)
#define GET_FLWUP_WINDOW(flags) ((flags & FLWUP_WINDOW) >> 4)
#define GET_FLWUP_TX_RSP_DISABLE_FLAG(flags)                                   \
	((flags & FLWUP_TX_RSP_DISABLE_FLAG) >> 5)

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */
u16 nan_read_tlv(u8 *pInTlv, struct _NanTlv *pOutTlv);
void nanSetPublishPmkid(struct ADAPTER *prAdapter,
				struct NanPublishRequest *msg);
u16 nanMapPublishReqParams(u16 *pIndata, struct NanPublishRequest *pOutparams);
u16 nanMapSubscribeReqParams(u16 *pIndata,
			     struct NanSubscribeRequest *pOutparams);
u16 nanMapFollowupReqParams(u32 *pIndata,
			    struct NanTransmitFollowupRequest *pOutparams);
void nanMapSdeaCtrlParams(u32 *pIndata,
			  struct NanSdeaCtrlParams *prNanSdeaCtrlParms);
void nanMapRangingConfigParams(u32 *pIndata,
			       struct NanRangingCfg *prNanRangingCfg);
void
nanMapNan20RangingReqParams(u32 *pIndata,
			    struct NanRangeResponseCfg *prNanRangeRspCfgParms);
int mtk_cfg80211_vendor_nan(struct wiphy *wiphy, struct wireless_dev *wdev,
			    const void *data, int data_len);
int mtk_cfg80211_vendor_event_nan_event_indication(IN struct ADAPTER *prAdapter,
						   uint8_t ucEventType,
						   uint8_t *pcuEvtBuf);
int
mtk_cfg80211_vendor_event_nan_replied_indication(IN struct ADAPTER *prAdapter,
						 uint8_t *pcuEvtBuf);
int
mtk_cfg80211_vendor_event_nan_publish_terminate(IN struct ADAPTER *prAdapter,
						uint8_t *pcuEvtBuf);
int
mtk_cfg80211_vendor_event_nan_subscribe_terminate(IN struct ADAPTER *prAdapter,
						  uint8_t *pcuEvtBuf);
int
mtk_cfg80211_vendor_event_nan_followup_indication(IN struct ADAPTER *prAdapter,
						  uint8_t *pcuEvtBuf);
int mtk_cfg80211_vendor_event_nan_match_indication(IN struct ADAPTER *prAdapter,
						   uint8_t *pcuEvtBuf);
#endif
