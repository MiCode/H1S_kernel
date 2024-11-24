/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __NAN_INTF_H__
#define __NAN_INTF_H__

#if CFG_SUPPORT_NAN
/*****************************************************************************
 * Neighbour Aware Network Service Structures and Functions
 *****************************************************************************
 */
#ifndef IFNAMSIZ
#define IFNAMSIZ 16
#endif

#ifdef __KAL_ATTRIB_PACKED__
#undef __KAL_ATTRIB_PACKED__
#define __KAL_ATTRIB_PACKED__ __attribute__((__packed__))
#endif

/* Buffer size to generate NAN attribute */
#define NAN_IE_BUF_MAX_SIZE 1000

#define NAN_MAC_ADDR_LEN 6
#define NAN_MAX_SOCIAL_CHANNELS 3

/* NAN Maximum Lengths */
#define NAN_MAX_SERVICE_NAME_LEN 255
#define NAN_MAX_MATCH_FILTER_LEN 255
#define NAN_MAX_SERVICE_SPECIFIC_INFO_LEN 1024

#define NAN_FW_MAX_SERVICE_NAME_LEN 32
#define NAN_FW_MAX_MATCH_FILTER_LEN 32
#define NAN_FW_MAX_SERVICE_SPECIFIC_INFO_LEN 255
#define NAN_FW_MAX_SCID_BUF_LEN 128
#define NAN_FW_MAX_SDEA_SERVICE_SPECIFIC_INFO_LEN 255

#define NAN_MAX_VSA_DATA_LEN 1024
#define NAN_MAX_MESH_DATA_LEN 32
#define NAN_MAX_INFRA_DATA_LEN 32
#define NAN_MAX_CLUSTER_ATTRIBUTE_LEN 255
#define NAN_MAX_SUBSCRIBE_MAX_ADDRESS 42
#define NAN_MAX_FAM_CHANNELS 32
#define NAN_MAX_POSTDISCOVERY_LEN 5
#define NAN_MAX_FRAME_DATA_LEN 504
#define NAN_MAX_DEBUG_MESSAGE_DATA_LEN 100
#define NAN_DP_MAX_APP_INFO_LEN 512
#define NAN_ERROR_STR_LEN 255
#define NAN_PMK_INFO_LEN 32
#define NAN_MAX_SCID_BUF_LEN 1024
#define NAN_FW_MAX_SCID_BUF_LEN 128
#define NAN_SDEA_SERVICE_SPECIFIC_INFO_LEN 1024
#define NAN_FW_SDEA_SPECIFIC_INFO_LEN 255
#define NAN_SECURITY_MIN_PASSPHRASE_LEN 8
#define NAN_SECURITY_MAX_PASSPHRASE_LEN 63
#define IPV6MACLEN 8

/* NAN Shared Key Security Cipher Suites Mask */
#define NAN_CIPHER_SUITE_SHARED_KEY_NONE 0x00
#define NAN_CIPHER_SUITE_SHARED_KEY_128_MASK 0x01
#define NAN_CIPHER_SUITE_SHARED_KEY_256_MASK 0x02

/* NAN ranging indication condition MASKS */
#define NAN_RANGING_INDICATE_CONTINUOUS_MASK 0x01
#define NAN_RANGING_INDICATE_INGRESS_MET_MASK 0x02
#define NAN_RANGING_INDICATE_EGRESS_MET_MASK 0x04

/* Mask to determine on which frames attribute was received */
#define RX_DISCOVERY_BEACON_MASK 0x01
#define RX_SYNC_BEACON_MASK 0x02
#define RX_SERVICE_DISCOVERY_MASK 0x04

enum NAN_BSS_ROLE_INDEX {
	NAN_BSS_INDEX_BAND0 = 0,
#if (CFG_SUPPORT_DBDC == 1)
	NAN_BSS_INDEX_BAND1,
#endif
	NAN_BSS_INDEX_NUM
};

enum _ENUM_NAN_PROTOCOL_ROLE_T {
	NAN_PROTOCOL_RESPONDER = 0,
	NAN_PROTOCOL_INITIATOR,
	NAN_PROTOCOL_ROLE_NUM
};

/* Definition of various NanResponseType */
enum NanResponseType {
	NAN_RESPONSE_ENABLED = 0,
	NAN_RESPONSE_DISABLED = 1,
	NAN_RESPONSE_PUBLISH = 2,
	NAN_RESPONSE_PUBLISH_CANCEL = 3,
	NAN_RESPONSE_TRANSMIT_FOLLOWUP = 4,
	NAN_RESPONSE_SUBSCRIBE = 5,
	NAN_RESPONSE_SUBSCRIBE_CANCEL = 6,
	NAN_RESPONSE_STATS = 7,
	NAN_RESPONSE_CONFIG = 8,
	NAN_RESPONSE_TCA = 9,
	NAN_RESPONSE_ERROR = 10,
	NAN_RESPONSE_BEACON_SDF_PAYLOAD = 11,
	NAN_GET_CAPABILITIES = 12,
	NAN_DP_INTERFACE_CREATE = 13,
	NAN_DP_INTERFACE_DELETE = 14,
	NAN_DP_INITIATOR_RESPONSE = 15,
	NAN_DP_RESPONDER_RESPONSE = 16,
	NAN_DP_END = 17
};

/* NAN Publish Types */
enum NanPublishType {
	NAN_PUBLISH_TYPE_UNSOLICITED = 0,
	NAN_PUBLISH_TYPE_SOLICITED,
	NAN_PUBLISH_TYPE_UNSOLICITED_SOLICITED
};

/* NAN Transmit Priorities */
enum NanTxPriority { NAN_TX_PRIORITY_NORMAL = 0, NAN_TX_PRIORITY_HIGH };

/* NAN Statistics Request ID Codes */
enum NanStatsType {
	NAN_STATS_ID_DE_PUBLISH = 0,
	NAN_STATS_ID_DE_SUBSCRIBE,
	NAN_STATS_ID_DE_MAC,
	NAN_STATS_ID_DE_TIMING_SYNC,
	NAN_STATS_ID_DE_DW,
	NAN_STATS_ID_DE
};

/* NAN Protocol Event ID Codes */
enum NanDiscEngEventType {
	NAN_EVENT_ID_DISC_MAC_ADDR = 0,
	NAN_EVENT_ID_STARTED_CLUSTER,
	NAN_EVENT_ID_JOINED_CLUSTER
};

/* NAN Data Path type */
enum NdpType { NAN_DATA_PATH_UNICAST_MSG = 0, NAN_DATA_PATH_MULTICAST_MSG };

/* NAN Ranging Configuration */
enum NanRangingState { NAN_RANGING_DISABLE = 0, NAN_RANGING_ENABLE };

/* Various NAN Protocol Response code */
enum NanStatusType {
	/* NAN Protocol Response Codes */
	NAN_STATUS_SUCCESS = 0,
	/*  NAN Discovery Engine/Host driver failures */
	NAN_STATUS_INTERNAL_FAILURE = 1,
	/*  NAN OTA failures */
	NAN_STATUS_PROTOCOL_FAILURE = 2,
	/* if the publish/subscribe id is invalid */
	NAN_STATUS_INVALID_PUBLISH_SUBSCRIBE_ID = 3,
	/* If we run out of resources allocated */
	NAN_STATUS_NO_RESOURCE_AVAILABLE = 4,
	/* if invalid params are passed */
	NAN_STATUS_INVALID_PARAM = 5,
	/*  if the requestor instance id is invalid */
	NAN_STATUS_INVALID_REQUESTOR_INSTANCE_ID = 6,
	/*  if the ndp id is invalid */
	NAN_STATUS_INVALID_NDP_ID = 7,
	/* if NAN is enabled when wifi is turned off */
	NAN_STATUS_NAN_NOT_ALLOWED = 8,
	/* if over the air ack is not received */
	NAN_STATUS_NO_OTA_ACK = 9,
	/* If NAN is already enabled and we are try to re-enable the same */
	NAN_STATUS_ALREADY_ENABLED = 10,
	/* If followup message internal queue is full */
	NAN_STATUS_FOLLOWUP_QUEUE_FULL = 11,
	/* Unsupported concurrency session enabled, NAN disabled notified */
	NAN_STATUS_UNSUPPORTED_CONCURRENCY_NAN_DISABLED = 12
};

/* NAN Transmit Types */
enum NanTxType { NAN_TX_TYPE_BROADCAST = 0, NAN_TX_TYPE_UNICAST };

/* NAN Subscribe Type */
enum NanSubscribeType {
	NAN_SUBSCRIBE_TYPE_PASSIVE = 0,
	NAN_SUBSCRIBE_TYPE_ACTIVE
};

/* NAN Service Response Filter Attribute Bit */
enum NanSRFType {
	NAN_SRF_ATTR_BLOOM_FILTER = 0,
	NAN_SRF_ATTR_PARTIAL_MAC_ADDR
};

/* NAN Service Response Filter Include Bit */
enum NanSRFIncludeType {
	NAN_SRF_INCLUDE_DO_NOT_RESPOND = 0,
	NAN_SRF_INCLUDE_RESPOND
};

/* NAN Match indication type */
enum NanMatchAlg {
	NAN_MATCH_ALG_MATCH_ONCE = 0,
	NAN_MATCH_ALG_MATCH_CONTINUOUS,
	NAN_MATCH_ALG_MATCH_NEVER
};

/* NAN Transmit Window Type */
enum NanTransmitWindowType { NAN_TRANSMIT_IN_DW = 0, NAN_TRANSMIT_IN_FAW };

/* NAN SRF State in Subscribe */
enum NanSRFState { NAN_DO_NOT_USE_SRF = 0, NAN_USE_SRF };

/* NAN Include SSI in MatchInd */
enum NanSsiInMatchInd {
	NAN_SSI_NOT_REQUIRED_IN_MATCH_IND = 0,
	NAN_SSI_REQUIRED_IN_MATCH_IND
};

/* NAN DP security Configuration */
enum NanDataPathSecurityCfgStatus {
	NAN_DP_CONFIG_NO_SECURITY = 0,
	NAN_DP_CONFIG_SECURITY
};

/* Data request Responder's response */
enum NanDataPathResponseCode {
	NAN_DP_REQUEST_ACCEPT = 0,
	NAN_DP_REQUEST_REJECT
};

/* NAN DP channel config options */
enum NanDataPathChannelCfg {
	NAN_DP_CHANNEL_NOT_REQUESTED = 0,
	NAN_DP_REQUEST_CHANNEL_SETUP,
	NAN_DP_FORCE_CHANNEL_SETUP
};

/* Enable/Disable NAN Ranging Auto response */
enum NanRangingAutoResponse {
	NAN_RANGING_AUTO_RESPONSE_ENABLE = 1,
	NAN_RANGING_AUTO_RESPONSE_DISABLE
};

/* Enable/Disable NAN service range report */
enum NanRangeReport { NAN_DISABLE_RANGE_REPORT = 1, NAN_ENABLE_RANGE_REPORT };

/* NAN Range Response */
enum NanRangeResponseCode {
	NAN_RANGE_REQUEST_ACCEPT = 1,
	NAN_RANGE_REQUEST_REJECT,
	NAN_RANGE_REQUEST_CANCEL
};

/* NAN Security Key Input Type */
enum NanSecurityKeyInputType {
	NAN_SECURITY_KEY_INPUT_PMK = 1,
	NAN_SECURITY_KEY_INPUT_PASSPHRASE
};

/* Host can set the Periodic scan parameters for each of the
 * 3(6, 44, 149) Social channels. Only these channels are allowed
 * any other channels are rejected
 */
enum NanChannelIndex {
	NAN_CHANNEL_24G_BAND = 0,
	NAN_CHANNEL_5G_BAND_LOW,
	NAN_CHANNEL_5G_BAND_HIGH
};

/* Nan accept policy: Per service basis policy
 * Based on this policy(ALL/NONE), responder side
 * will send ACCEPT/REJECT
 */
enum NanServiceAcceptPolicy {
	NAN_SERVICE_ACCEPT_POLICY_NONE = 0,
	/* Default value */
	NAN_SERVICE_ACCEPT_POLICY_ALL
};

/* Indicates the availability interval duration associated with the
 * Availability Intervals Bitmap field
 */
enum NanAvailDuration {
	NAN_DURATION_16MS = 0,
	NAN_DURATION_32MS = 1,
	NAN_DURATION_64MS = 2
};

/* QoS configuration */
enum NanDataPathQosCfg { NAN_DP_CONFIG_NO_QOS = 0, NAN_DP_CONFIG_QOS };

/* Host can send Post-Nan Discovery attributes which the Discovery Engine can
 * enclose in Service Discovery frames
 */
/* Possible connection types in Post NAN Discovery attributes */
enum NanConnectionType {
	NAN_CONN_WLAN_INFRA = 0,
	NAN_CONN_P2P_OPER = 1,
	NAN_CONN_WLAN_IBSS = 2,
	NAN_CONN_WLAN_MESH = 3,
	NAN_CONN_FURTHER_SERVICE_AVAILABILITY = 4,
	NAN_CONN_WLAN_RANGING = 5
};

/* Possible device roles in Post NAN Discovery attributes */
enum NanDeviceRole {
	NAN_WLAN_INFRA_AP = 0,
	NAN_WLAN_INFRA_STA = 1,
	NAN_P2P_OPER_GO = 2,
	NAN_P2P_OPER_DEV = 3,
	NAN_P2P_OPER_CLI = 4
};

struct NanSecurityPmk {
	/* pmk length */
	uint32_t pmk_len;

	/* PMK: Info is optional in Discovery phase.
	 * PMK info can be passed during
	 * the NDP session.
	 */
	uint8_t pmk[NAN_PMK_INFO_LEN];
} __KAL_ATTRIB_PACKED__;

struct NanSecurityPassPhrase {
	/* passphrase length */
	uint32_t passphrase_len;

	/* passphrase info is optional in Discovery phase.
	 * passphrase info can be passed during
	 * the NDP session.
	 */
	uint8_t passphrase[NAN_SECURITY_MAX_PASSPHRASE_LEN];
} __KAL_ATTRIB_PACKED__;

struct NanSecurityKeyInfo {
	enum NanSecurityKeyInputType key_type;

	union {
		struct NanSecurityPmk pmk_info;
		struct NanSecurityPassPhrase passphrase_info;
	} body;
} __KAL_ATTRIB_PACKED__;

/* Structure to set the Service Descriptor Extension
 * Attribute (SDEA) passed as part of NanPublishRequest/
 * NanSubscribeRequest/NanMatchInd.
 */
struct NanSdeaCtrlParams {
	/* Optional configuration of Data Path Enable request.
	 * configure flag determines whether configuration needs
	 * to be passed or not.
	 */
	uint8_t config_nan_data_path;

	enum NdpType ndp_type;

	/* NAN secuirty required flag to indicate
	 * if the security is enabled or disabled
	 */
	enum NanDataPathSecurityCfgStatus security_cfg;

	/* NAN ranging required flag to indicate
	 * if ranging is enabled on disabled
	 */
	enum NanRangingState ranging_state;

	/* Enable/Disable Ranging report,
	 * when configured NanRangeReportInd received
	 */
	enum NanRangeReport range_report;

	/* FSD require */
	unsigned char fgFSDRequire;

	/* FSD with GAS */
	unsigned char fgGAS;
	unsigned char fgQoS;
	unsigned char fgRangeLimit;
} __KAL_ATTRIB_PACKED__;

/* Nan Ranging Peer Info in MatchInd */
struct NanRangeInfo {
	/* Distance to the NAN device with the MAC address indicated
	 * with ranged mac address.
	 */
	uint32_t range_measurement_cm;

	/* Ranging event matching the configuration of */
	/* continuous/ingress/egress. */
	uint32_t ranging_event_type;
} __KAL_ATTRIB_PACKED__;

/* Response control parameters */
struct NanRangeResponseCtl {
	/* Enable/disable NAN serivce Ranging auto response mode */
	enum NanRangingAutoResponse ranging_auto_response;

	/* Enable/Disable Ranging report,
	 * when configured NanRangeReportInd received
	 */
	enum NanRangeReport range_report;

	/* Response indicating ACCEPT/REJECT/CANCEL of Range Request */
	enum NanRangeResponseCode ranging_response_code;
} __KAL_ATTRIB_PACKED__;

/* NAN FTM Parameters */
struct NanRangeRequestFtmCfg {
	uint8_t max_burst_duration;
	uint8_t min_delta_ftm;
	uint8_t max_ftms_per_burst;
	uint8_t ftm_format_and_bandwidth;
} __KAL_ATTRIB_PACKED__;

/* Configuration parameters received from the
 * Ranging Request frame
 */
struct NanRangeRequestCfg {
	/* Ranging Report is required by the Responder */
	enum NanRangeReport range_report;

	/* NAN FTM Parameters */
	struct NanRangeRequestFtmCfg rangereq_ftm_cfg;
} __KAL_ATTRIB_PACKED__;

/* Nan/NDP Capabilities info */
struct NanCapabilities {
	uint32_t max_concurrent_nan_clusters;
	uint32_t max_publishes;
	uint32_t max_subscribes;
	uint32_t max_service_name_len;
	uint32_t max_match_filter_len;
	uint32_t max_total_match_filter_len;
	uint32_t max_service_specific_info_len;
	uint32_t max_vsa_data_len;
	uint32_t max_mesh_data_len;
	uint32_t max_ndi_interfaces;
	uint32_t max_ndp_sessions;
	uint32_t max_app_info_len;
	uint32_t max_queued_transmit_followup_msgs;
	uint32_t ndp_supported_bands;
	uint32_t cipher_suites_supported;
	uint32_t max_scid_len;
	uint32_t max_sdea_service_specific_info_len;
	uint32_t max_subscribe_address;

	unsigned char is_ndp_security_supported;
} __KAL_ATTRIB_PACKED__;

/* Host can send Vendor specific attributes which the Discovery Engine can
 * enclose in Beacons and/or Service Discovery frames transmitted.
 * Below structure is used to populate that.
 */
struct NanTransmitVendorSpecificAttribute {
	/* 0 = transmit only in the next discovery window
	 * 1 = transmit in next 16 discovery window
	 */
	uint8_t payload_transmit_flag;
	/* Below flags will determine in which all frames
	 * the vendor specific attributes should be included
	 */
	uint8_t tx_in_discovery_beacon;
	uint8_t tx_in_sync_beacon;
	uint8_t tx_in_service_discovery;

	/* Organizationally Unique Identifier */
	uint32_t vendor_oui;

	/* vendor specific attribute to be transmitted
	 * vsa_len : Length of the vsa data.
	 */
	uint32_t vsa_len;

	uint8_t vsa[NAN_MAX_VSA_DATA_LEN];
} __KAL_ATTRIB_PACKED__;

/* Discovery Engine will forward any Vendor Specific Attributes
 * which it received as part of this structure.
 */
struct NanReceiveVendorSpecificAttribute {
	/* Frames on which this vendor specific attribute
	 * was received. Mask defined above
	 */
	uint8_t vsa_received_on;

	/* Organizationally Unique Identifier */
	uint32_t vendor_oui;

	/* vendor specific attribute */
	uint32_t attr_len;

	uint8_t vsa[NAN_MAX_VSA_DATA_LEN];
} __KAL_ATTRIB_PACKED__;

/* NAN Beacon SDF Payload Received structure
 * Discovery engine sends the details of received Beacon or
 * Service Discovery Frames as part of this structure.
 */
struct NanBeaconSdfPayloadReceive {
	/* Frame data */
	uint32_t frame_len;

	uint8_t frame_data[NAN_MAX_FRAME_DATA_LEN];
} __KAL_ATTRIB_PACKED__;

/* Structure to set the Social Channel Scan parameters
 * passed as part of NanEnableRequest/NanConfigRequest
 */
struct NanSocialChannelScanParams {
	/* Dwell time of each social channel in milliseconds
	 * NanChannelIndex corresponds to the respective channel
	 * If time set to 0 then the FW default time will be used.
	 */
	uint8_t dwell_time[NAN_MAX_SOCIAL_CHANNELS];

	/* Scan period of each social channel in seconds
	 * NanChannelIndex corresponds to the respective channel
	 * If time set to 0 then the FW default time will be used.
	 */
	uint16_t scan_period[NAN_MAX_SOCIAL_CHANNELS];
} __KAL_ATTRIB_PACKED__;

/* Host can send Post Connectivity Capability attributes
 * to be included in Service Discovery frames transmitted
 * as part of this structure.
 */
struct NanTransmitPostConnectivityCapability {
	/* 0 = transmit only in the next discovery window
	 * 1 = transmit in next 16 discovery window
	 */
	uint8_t payload_transmit_flag;
	/* 1 - Wifi Direct supported 0 - Not supported */
	uint8_t is_wfd_supported;
	/* 1 - Wifi Direct Services supported 0 - Not supported */
	uint8_t is_wfds_supported;
	/* 1 - TDLS supported 0 - Not supported */
	uint8_t is_tdls_supported;

	/* 1 - IBSS supported 0 - Not supported */
	uint8_t is_ibss_supported;
	/* 1 - Mesh supported 0 - Not supported */
	uint8_t is_mesh_supported;
	/* 1 - NAN Device currently connect to WLAN Infra AP
	 * 0 - otherwise
	 */
	uint8_t wlan_infra_field;
} __KAL_ATTRIB_PACKED__;

/*
 *  Discovery engine providing the post connectivity capability
 *  received.
 */
struct NanReceivePostConnectivityCapability {
	/* 1 - Wifi Direct supported 0 - Not supported */
	uint8_t is_wfd_supported;
	/* 1 - Wifi Direct Services supported 0 - Not supported */
	uint8_t is_wfds_supported;
	/* 1 - TDLS supported 0 - Not supported */
	uint8_t is_tdls_supported;
	/* 1 - IBSS supported 0 - Not supported */
	uint8_t is_ibss_supported;

	/* 1 - Mesh supported 0 - Not supported */
	uint8_t is_mesh_supported;
	/*
	 *   1 - NAN Device currently connect to WLAN Infra AP
	 *   0 - otherwise
	 */
	uint8_t wlan_infra_field;
} __KAL_ATTRIB_PACKED__;

/* Further availability per channel information */
struct NanFurtherAvailabilityChannel {
	/* Defined above */
	enum NanAvailDuration entry_control;

	/* 1 byte field indicating the frequency band the NAN Device
	 * will be available as defined in IEEE Std. 802.11-2012
	 * Annex E Table E-4 Global Operating Classes
	 */
	uint8_t class_val;
	/* 1 byte field indicating the channel the NAN Device
	 * will be available.
	 */
	uint8_t channel;
	/* Map Id - 4 bit field which identifies the Further
	 * availability map attribute.
	 */
	uint8_t mapid;
	/* divides the time between the beginnings of consecutive Discovery
	 * Windows of a given NAN cluster into consecutive time intervals
	 * of equal durations. The time interval duration is specified by
	 * the Availability Interval Duration subfield of the Entry Control
	 * field.
	 * A Nan device that sets the i-th bit of the Availability
	 * Intervals Bitmap to 1 shall be present during the corresponding
	 * i-th time interval in the operation channel indicated by the
	 * Operating Class and Channel Number fields in the same
	 * Availability Entry.
	 * A Nan device that sets the i-th bit of the Availability Intervals
	 * Bitmap to
	 * 0 may be present during the corresponding i-th time interval
	 * in the operation
	 * channel indicated by the Operating Class and Channel Number fields
	 * in the same
	 * Availability Entry.
	 * The size of the Bitmap is dependent upon the Availability Interval
	 * Duration chosen in the Entry Control Field.
	 * The size can be either 1, 2 or 4 bytes long
	 * - Duration field is equal to 0, only AIB[0] is valid
	 * - Duration field is equal to 1, only AIB [0] and AIB [1] is valid
	 * - Duration field is equal to 2, AIB [0], AIB [1], AIB [2] and AIB
	 *   [3] are valid
	 */
	uint32_t avail_interval_bitmap;
} __KAL_ATTRIB_PACKED__;


/* Further availability map which can be sent and received from
 * Discovery engine
 */
struct NanFurtherAvailabilityMap {
	/* Number of channels indicates the number of channel
	 * entries which is part of fam
	 */
	uint8_t numchans;

	struct NanFurtherAvailabilityChannel famchan[NAN_MAX_FAM_CHANNELS];
} __KAL_ATTRIB_PACKED__;

/* Configuration params of NAN Ranging */
struct NanRangingCfg {
	/* Determine the accuracy required from the ranging */
	uint32_t ranging_resolution;

	/* Interval in milli sec between two ranging measurements.
	 * If the Awake DW intervals in NanEnable/Config are larger
	 * than the ranging intervals priority is given to Awake DW
	 * Intervals. Only on a match the ranging is initiated for the
	 * peer
	 */
	uint32_t ranging_interval_msec;

	/* Flags indicating the type of ranging event to be notified
	 * NAN_RANGING_INDICATE_ MASKS are used to set these.
	 * BIT0 - Continuous Ranging event notification.
	 * BIT1 - Ingress distance is <=.
	 * BIT2 - Egress distance is >=.
	 */
	uint32_t config_ranging_indications;

	/* Ingress distance in centimeters (optional) */
	uint32_t distance_ingress_cm;

	/* Egress distance in centimeters (optional) */
	uint32_t distance_egress_cm;
} __KAL_ATTRIB_PACKED__;

/* NAN Ranging request's response */
struct NanRangeResponseCfg {
	/* Publish Id of an earlier Publisher */
	uint16_t publish_id;

	/* A 32 bit Requestor instance Id which is sent to the Application.
	 * This Id will be used in subsequent RangeResponse on Subscribe side.
	 */
	uint32_t requestor_instance_id;

	/* Peer MAC addr of Range Requestor */
	uint8_t peer_addr[NAN_MAC_ADDR_LEN];

	/* Response indicating ACCEPT/REJECT/CANCEL of Range Request */
	enum NanRangeResponseCode ranging_response_code;
} __KAL_ATTRIB_PACKED__;

/* Structure of Post NAN Discovery attribute */
struct NanTransmitPostDiscovery {
	/* Connection type of the host */
	enum NanConnectionType type;

	/* Device role of the host based on
	 * the connection type
	 */
	enum NanDeviceRole role;

	/* Flag to send the information as a single shot or repeated
	 * for next 16 discovery windows
	 * 0 - Single_shot
	 * 1 - next 16 discovery windows
	 */
	uint8_t transmit_freq;

	/* Duration of the availability bitmask */
	enum NanAvailDuration duration;

	/* Availability interval bitmap based on duration */
	uint32_t avail_interval_bitmap;

	/* Mac address depending on the conn type and device role
	 * --------------------------------------------------
	 * | Conn Type  |  Device Role |  Mac address Usage  |
	 * --------------------------------------------------
	 * | WLAN_INFRA |  AP/STA      |   BSSID of the AP   |
	 * --------------------------------------------------
	 * | P2P_OPER   |  GO          |   GO's address      |
	 * --------------------------------------------------
	 * | P2P_OPER   |  P2P_DEVICE  |   Address of who    |
	 * |            |              |   would become GO   |
	 * --------------------------------------------------
	 * | WLAN_IBSS  |  NA          |   BSSID             |
	 * --------------------------------------------------
	 * | WLAN_MESH  |  NA          |   BSSID             |
	 * --------------------------------------------------
	 */
	uint8_t addr[NAN_MAC_ADDR_LEN];

	/* Mandatory mesh id value if connection type is WLAN_MESH
	 * Mesh id contains 0-32 octet identifier and should be
	 * as per IEEE Std.802.11-2012 spec.
	 */
	uint16_t mesh_id_len;
	uint8_t mesh_id[NAN_MAX_MESH_DATA_LEN];
	/* Optional infrastructure SSID if conn_type is set to
	 * NAN_CONN_WLAN_INFRA
	 */
	uint16_t infrastructure_ssid_len;
	uint8_t infrastructure_ssid_val[NAN_MAX_INFRA_DATA_LEN];
} __KAL_ATTRIB_PACKED__;

/* Discovery engine providing the structure of Post NAN Discovery */
struct NanReceivePostDiscovery {
	/* Connection type of the host */
	enum NanConnectionType type;

	/* Device role of the host based on
	 * the connection type
	 */
	enum NanDeviceRole role;

	/* Duration of the availability bitmask */
	enum NanAvailDuration duration;

	/* Availability interval bitmap based on duration */
	uint32_t avail_interval_bitmap;

	/* Map Id - 4 bit field which identifies the Further
	 * availability map attribute.
	 */
	uint8_t mapid;
	/* Mac address depending on the conn type and device role
	 * --------------------------------------------------
	 * | Conn Type  |  Device Role |  Mac address Usage  |
	 * --------------------------------------------------
	 * | WLAN_INFRA |  AP/STA      |   BSSID of the AP   |
	 * --------------------------------------------------
	 * | P2P_OPER   |  GO          |   GO's address      |
	 * --------------------------------------------------
	 * | P2P_OPER   |  P2P_DEVICE  |   Address of who    |
	 * |            |              |   would become GO   |
	 * --------------------------------------------------
	 * | WLAN_IBSS  |  NA          |   BSSID             |
	 * --------------------------------------------------
	 * | WLAN_MESH  |  NA          |   BSSID             |
	 * --------------------------------------------------
	 */
	uint8_t addr[NAN_MAC_ADDR_LEN];

	/* Mandatory mesh id value if connection type is WLAN_MESH
	 * Mesh id contains 0-32 octet identifier and should be
	 * as per IEEE Std.802.11-2012 spec.
	 */
	uint16_t mesh_id_len;
	uint8_t mesh_id[NAN_MAX_MESH_DATA_LEN];
	/* Optional infrastructure SSID if conn_type is set to
	 * NAN_CONN_WLAN_INFRA
	 */
	uint16_t infrastructure_ssid_len;
	uint8_t infrastructure_ssid_val[NAN_MAX_INFRA_DATA_LEN];

} __KAL_ATTRIB_PACKED__;

/* NAN device level configuration of SDF and Sync beacons in both
 * 2.4/5GHz bands
 */
struct NanConfigDW {
	/* Configure 2.4GHz DW Band */
	uint8_t config_2dot4g_dw_band;

	/* Indicates the interval for Sync beacons and SDF's in 2.4GHz band.
	 * Valid values of DW Interval are: 1, 2, 3, 4 and 5, 0 is reserved.
	 * The SDF includes in OTA when enabled. The publish/subscribe period
	 * values don't override the device level configurations.
	 */
	uint32_t dw_2dot4g_interval_val;

	/* Configure 5GHz DW Band */
	uint8_t config_5g_dw_band;

	/* Indicates the interval for Sync beacons and SDF's in 5GHz band
	 * Valid values of DW Interval are: 1, 2, 3, 4 and 5, 0 no wake up for
	 * any interval. The SDF includes in OTA when enabled.
	 * The publish/subscribe
	 * period values don't override the device level configurations.
	 */
	uint32_t dw_5g_interval_val;
} __KAL_ATTRIB_PACKED__;

/* Enable Request Message Structure
 * The NanEnableReq message instructs the Discovery Engine to enter an
 * operational state
 */
struct NanEnableRequest {
	/* Mandatory parameters below */
	uint8_t master_pref;

	/* A cluster_low value matching cluster_high indicates a request to join
	 * a cluster with that value. If the requested cluster is not found the
	 * device will start its own cluster.
	 */
	uint16_t cluster_low;
	uint16_t cluster_high;

	/* Optional configuration of Enable request.
	 * Each of the optional parameters have configure flag which
	 * determine whether configuration is to be passed or not.
	 */
	uint8_t config_support_5g;
	uint8_t support_5g_val;
	/* BIT 0 is used to specify to include Service IDs in Sync/Discovery
	 * beacons
	 * 0 - Do not include SIDs in any beacons
	 * 1 - Include SIDs in all beacons.
	 * Rest 7 bits are count field which allows control over the number
	 * of SIDs included in the Beacon.
	 * 0 means to include as many SIDs that fit into
	 * the maximum allow Beacon frame size
	 */
	uint8_t config_sid_beacon;
	uint8_t sid_beacon_val;

	/* The rssi values below should be specified without sign.
	 * For eg: -70dBm should be specified as 70.
	 */
	uint8_t config_2dot4g_rssi_close;
	uint8_t rssi_close_2dot4g_val;
	uint8_t config_2dot4g_rssi_middle;
	uint8_t rssi_middle_2dot4g_val;

	uint8_t config_2dot4g_rssi_proximity;
	uint8_t rssi_proximity_2dot4g_val;
	uint8_t config_hop_count_limit;
	uint8_t hop_count_limit_val;

	/* Defines 2.4G channel access support
	 * 0 - No Support
	 * 1 - Supported
	 */
	uint8_t config_2dot4g_support;
	uint8_t support_2dot4g_val;
	/* Defines 2.4G channels will be used for sync/discovery beacons
	 * 0 - 2.4G channels not used for beacons
	 * 1 - 2.4G channels used for beacons
	 */
	uint8_t config_2dot4g_beacons;
	uint8_t beacon_2dot4g_val;

	/* Defines 2.4G channels will be used for Service Discovery frames
	 * 0 - 2.4G channels not used for Service Discovery frames
	 * 1 - 2.4G channels used for Service Discovery frames
	 */
	uint8_t config_2dot4g_sdf;
	uint8_t sdf_2dot4g_val;
	/* Defines 5G channels will be used for sync/discovery beacons
	 * 0 - 5G channels not used for beacons
	 * 1 - 5G channels used for beacons
	 */
	uint8_t config_5g_beacons;
	uint8_t beacon_5g_val;

	/* Defines 5G channels will be used for Service Discovery frames
	 * 0 - 5G channels not used for Service Discovery frames
	 * 1 - 5G channels used for Service Discovery frames
	 */
	uint8_t config_5g_sdf;
	uint8_t sdf_5g_val;
	/* 1 byte value which defines the RSSI in
	 * dBm for a close by Peer in 5 Ghz channels.
	 * The rssi values should be specified without sign.
	 * For eg: -70dBm should be specified as 70.
	 */
	uint8_t config_5g_rssi_close;
	uint8_t rssi_close_5g_val;
	/* 1 byte value which defines the RSSI value in
	 * dBm for a close by Peer in 5 Ghz channels.
	 * The rssi values should be specified without sign.
	 * For eg: -70dBm should be specified as 70.
	 */
	uint8_t config_5g_rssi_middle;
	uint8_t rssi_middle_5g_val;
	/* 1 byte value which defines the RSSI filter
	 * threshold.  Any Service Descriptors received above this
	 * value that are configured for RSSI filtering will be dropped.
	 * The rssi values should be specified without sign.
	 * For eg: -70dBm should be specified as 70.
	 */
	uint8_t config_5g_rssi_close_proximity;
	uint8_t rssi_close_proximity_5g_val;

	/* 1 byte quantity which defines the window size over
	 * which the ??average RSSI??will be calculated over.
	 */
	uint8_t config_rssi_window_size;
	uint8_t rssi_window_size_val;
	/* The 24 bit Organizationally Unique ID + the 8 bit Network Id. */
	uint8_t config_oui;
	uint32_t oui_val;

	/* NAN Interface Address, If not configured the Discovery Engine
	 * will generate a 6 byte Random MAC.
	 */
	uint8_t config_intf_addr;
	uint8_t intf_addr_val[NAN_MAC_ADDR_LEN];
	/* If set to 1, the Discovery Engine will enclose the Cluster
	 * Attribute only sent in Beacons in a Vendor Specific Attribute
	 * and transmit in a Service Descriptor Frame.
	 */
	uint8_t config_cluster_attribute_val;

	/* The periodicity in seconds between full scan??s to find any new
	 * clusters available in the area.  A Full scan should not be done
	 * more than every 10 seconds and should not be done less than every
	 * 30 seconds.
	 */
	uint8_t config_scan_params;
	struct NanSocialChannelScanParams scan_params_val;

	/* 1 byte quantity which forces the Random Factor to a particular
	 * value for all transmitted Sync/Discovery beacons
	 */
	uint8_t config_random_factor_force;
	uint8_t random_factor_force_val;
	/* 1 byte quantity which forces the HC for all transmitted Sync and
	 * Discovery Beacon NO matter the real HC being received over the
	 * air.
	 */
	uint8_t config_hop_count_force;
	uint8_t hop_count_force_val;

	/* channel frequency in MHz to enable Nan on */
	uint8_t config_24g_channel;
	uint32_t channel_24g_val;

	uint8_t config_5g_channel;
	uint32_t channel_5g_val;

	/* Configure 2.4/5GHz DW */
	struct NanConfigDW config_dw;

	/* By default discovery MAC address randomization is enabled
	 * and default interval value is 30 minutes i.e. 1800 seconds.
	 * The value 0 is used to disable MAC addr randomization.
	 */
	uint8_t config_disc_mac_addr_randomization;
	uint32_t disc_mac_addr_rand_interval_sec;

	/* Set/Enable corresponding bits to disable Discovery indications:
	 * BIT0 - Disable Discovery MAC Address Event.
	 * BIT1 - Disable Started Cluster Event.
	 * BIT2 - Disable Joined Cluster Event.
	 */
	uint8_t discovery_indication_cfg;
	/* BIT 0 is used to specify to include Service IDs in Sync/Discovery
	 * beacons
	 * 0 - Do not include SIDs in any beacons
	 * 1 - Include SIDs in all beacons.
	 * Rest 7 bits are count field which allows control over the number
	 * of SIDs included in the Beacon.
	 * 0 means to include as many SIDs that fit into
	 * the maximum allow Beacon frame size
	 */
	uint8_t config_subscribe_sid_beacon;
	uint32_t subscribe_sid_beacon_val;
} __KAL_ATTRIB_PACKED__;

struct NanDataReqReceive {
	uint8_t ndpid;
	uint8_t initiator_data_addr[NAN_MAC_ADDR_LEN];
} __KAL_ATTRIB_PACKED__;

/* Publish Msg Structure
 * Message is used to request the DE to publish the Service Name
 * using the parameters passed into the Discovery Window
 */
struct NanPublishRequest {
	/* id  0 means new publish, any other id is existing publish */
	uint16_t publish_id;
	/* how many seconds to run for. 0 means forever until canceled */
	uint16_t ttl;

	/* period: Awake DW Interval for publish(service)
	 * Indicates the interval between two Discovery Windows in which
	 * the device supporting the service is awake to transmit or
	 * receive the Service Discovery frames.
	 * Valid values of Awake DW Interval are: 1, 2, 4, 8 and 16, value 0
	 * will default to 1.
	 */
	uint16_t period;
	/* 0= unsolicited, solicited = 1, 2= both */
	enum NanPublishType publish_type;
	/* 0 = broadcast, 1= unicast  if solicited publish */
	enum NanTxType tx_type;

	/* number of OTA Publish, 0 means forever until canceled */
	uint8_t publish_count;
	/* length of service name */
	uint16_t service_name_len;
	/* UTF-8 encoded string identifying the service */
	uint8_t service_name[NAN_MAX_SERVICE_NAME_LEN];

	/* Field which specifies how the matching indication to host is
	 * controlled.
	 * 0 - Match and Indicate Once
	 * 1 - Match and Indicate continuous
	 * 2 - Match and Indicate never. This means don't indicate the
	 * match to the host.
	 * 3 - Reserved
	 */
	enum NanMatchAlg publish_match_indicator;

	/* Sequence of values
	 * NAN Device that has invoked a Subscribe method corresponding to
	 * this Publish method
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Ordered sequence of <length, value> pairs which specify
	 * further response conditions beyond the service name used to filter
	 * subscribe messages to respond to.
	 * This is only needed when the PT is set to NAN_SOLICITED or
	 * NAN_SOLICITED_UNSOLICITED.
	 */
	uint16_t rx_match_filter_len;
	uint8_t rx_match_filter[NAN_MAX_MATCH_FILTER_LEN];

	/* Ordered sequence of <length, value> pairs to be included
	 * in the Discovery Frame.
	 * If present it is always sent in a Discovery Frame
	 */
	uint16_t tx_match_filter_len;
	uint8_t tx_match_filter[NAN_MAX_MATCH_FILTER_LEN];

	/* flag which specifies that the Publish should use the configured RSSI
	 * threshold and the received RSSI in order to filter requests
	 * 0 - ignore the configured RSSI threshold when running a Service
	 * Descriptor attribute or Service ID List Attribute through the
	 * DE matching logic.
	 * 1 - use the configured RSSI threshold when running a Service
	 * Descriptor attribute or Service ID List Attribute through the
	 * DE matching logic.
	 */
	uint8_t rssi_threshold_flag;
	/* 8-bit bitmap which allows the Host to associate this publish
	 * with a particular Post-NAN Connectivity attribute
	 * which has been sent down in a NanConfigureRequest/NanEnableRequest
	 * message.  If the DE fails to find a configured Post-NAN
	 * connectivity attributes referenced by the bitmap,
	 * the DE will return an error code to the Host.
	 * If the Publish is configured to use a Post-NAN Connectivity
	 * attribute and the Host does not refresh the Post-NAN Connectivity
	 * attribute the Publish will be canceled and the Host will be sent
	 * a PublishTerminatedIndication message.
	 */
	uint8_t connmap;
	/* Set/Enable corresponding bits to disable any indications
	 * that follow a publish.
	 * BIT0 - Disable publish termination indication.
	 * BIT1 - Disable match expired indication.
	 * BIT2 - Disable followUp indication received (OTA).
	 * BIT3 - Disable publishReplied indication.
	 */
	uint8_t recv_indication_cfg;

	/* Nan accept policy for the specific service(publish) */
	enum NanServiceAcceptPolicy service_responder_policy;

	/* NAN Cipher Suite Type */
	uint32_t cipher_type;

	/* Nan Security Key Info is optional in Discovery phase.
	 * PMK or passphrase info can be passed during
	 * the NDP session.
	 */
	struct NanSecurityKeyInfo key_info;

	/* Security Context Identifiers length */
	uint32_t scid_len;

	/* Security Context Identifier attribute contains PMKID
	 * shall be included in NDP setup and response messages.
	 * Security Context Identifier, Identifies the Security
	 * Context. For NAN Shared Key Cipher Suite, this field
	 * contains the 16 octet PMKID identifying the PMK used
	 * for setting up the Secure Data Path.
	 */
	uint8_t scid[NAN_MAX_SCID_BUF_LEN];

	/* NAN configure service discovery extended attributes */
	struct NanSdeaCtrlParams sdea_params;

	/* NAN Ranging configuration */
	struct NanRangingCfg ranging_cfg;

	/* Enable/disable NAN serivce Ranging auto response mode */
	enum NanRangingAutoResponse ranging_auto_response;

	/* When the ranging_auto_response_cfg is not set, NanRangeRequestInd is
	 * received. Nan Range Response to Peer MAC Addr is notified to
	 * indicate ACCEPT/REJECT/CANCEL to the requestor.
	 */
	struct NanRangeResponseCfg range_response_cfg;

	/* Sequence of values indicating the service specific info in SDEA */
	uint16_t sdea_service_specific_info_len;
	uint8_t sdea_service_specific_info[NAN_SDEA_SERVICE_SPECIFIC_INFO_LEN];
} __KAL_ATTRIB_PACKED__;

/* Publish Cancel Msg Structure
 * The PublishServiceCancelReq Message is used to request the DE to stop
 * publishing the Service Name identified by the Publish Id in the message.
 */
struct NanPublishCancelRequest {
	uint16_t publish_id;
} __KAL_ATTRIB_PACKED__;

/* NAN Subscribe Structure
 * The SubscribeServiceReq message is sent to the Discovery Engine
 * whenever the Upper layers would like to listen for a Service Name
 */
struct NanSubscribeRequest {
	/* id 0 means new subscribe, non zero is existing subscribe */
	uint16_t subscribe_id;
	/* how many seconds to run for. 0 means forever until canceled */
	uint16_t ttl;

	/* period: Awake DW Interval for subscribe(service)
	 * Indicates the interval between two Discovery Windows in which
	 * the device supporting the service is awake to transmit or
	 * receive the Service Discovery frames.
	 * Valid values of Awake DW Interval are: 1, 2, 4, 8 and 16, value 0
	 * will default to 1.
	 */
	uint16_t period;

	/* Flag which specifies how the Subscribe request shall be processed.
	 * 0 - PASSIVE , 1- ACTIVE
	 */
	enum NanSubscribeType subscribe_type; /*  */

	/* Flag which specifies on Active Subscribes how the Service
	 * Response Filter attribute is populated.
	 * 0 - Bloom Filter, 1 - MAC Addr
	 */
	enum NanSRFType
		serviceResponseFilter;

	/* Flag which specifies how the Service Response Filter
	 * Include bit is populated.
	 * 0=Do not respond if in the Address Set, 1= Respond
	 */
	enum NanSRFIncludeType serviceResponseInclude;

	/* Flag which specifies if the Service Response Filter should be
	 * used when creating Subscribes.
	 * 0 = Do not send the Service Response Filter,1 = send
	 */
	enum NanSRFState useServiceResponseFilter;

	/* Flag which specifies if the Service Specific Info is needed in
	 * the Publish message before creating the MatchIndication
	 * 0=Not needed, 1= Required
	 */
	enum NanSsiInMatchInd ssiRequiredForMatchIndication;

	/* Field which specifies how the matching indication to host is
	 * controlled.
	 * 0 - Match and Indicate Once
	 * 1 - Match and Indicate continuous
	 * 2 - Match and Indicate never. This means don't indicate the match
	 * to the host.
	 * 3 - Reserved
	 */
	enum NanMatchAlg subscribe_match_indicator;

	/* The number of Subscribe Matches which should occur
	 * before the Subscribe request is automatically terminated.
	 * If this value is 0 this field is not used by the DE.
	 */
	uint8_t subscribe_count;
	/* length of service name */
	uint16_t service_name_len;
	/* UTF-8 encoded string identifying the service */
	uint8_t service_name[NAN_MAX_SERVICE_NAME_LEN];

	/* Sequence of values which further specify the published service
	 * beyond the service name
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Ordered sequence of <length, value> pairs used to filter out
	 * received publish discovery messages.
	 * This can be sent both for a Passive or an Active Subscribe
	 */
	uint16_t rx_match_filter_len;
	uint8_t rx_match_filter[NAN_MAX_MATCH_FILTER_LEN];

	/* Ordered sequence of <length, value> pairs  included in the
	 * Discovery Frame when an Active Subscribe is used.
	 */
	uint16_t tx_match_filter_len;
	uint8_t tx_match_filter[NAN_MAX_MATCH_FILTER_LEN];

	/* Flag which specifies that the Subscribe should use the configured
	 * RSSI threshold and the received RSSI in order to filter requests
	 * 0 - ignore the configured RSSI threshold when running a Service
	 * Descriptor attribute or Service ID List Attribute through the
	 * DE matching logic.
	 * 1 - use the configured RSSI threshold when running a Service
	 * Descriptor attribute or Service ID List Attribute through the
	 * DE matching logic.
	 */
	uint8_t rssi_threshold_flag;
	/* 8-bit bitmap which allows the Host to associate this Active
	 * Subscribe with a particular Post-NAN Connectivity attribute
	 * which has been sent down in a NanConfigureRequest/NanEnableRequest
	 * message.  If the DE fails to find a configured Post-NAN
	 * connectivity attributes referenced by the bitmap,
	 * the DE will return an error code to the Host.
	 * If the Subscribe is configured to use a Post-NAN Connectivity
	 * attribute and the Host does not refresh the Post-NAN Connectivity
	 * attribute the Subscribe will be canceled and the Host will be sent
	 * a SubscribeTerminatedIndication message.
	 */
	uint8_t connmap;
	/* NAN Interface Address, conforming to the format as described in
	 * 8.2.4.3.2 of IEEE Std. 802.11-2012.
	 */
	uint8_t num_intf_addr_present;
	uint8_t intf_addr[NAN_MAX_SUBSCRIBE_MAX_ADDRESS][NAN_MAC_ADDR_LEN];
	/* SetEnable corresponding bits to disable indications that
	 * follow a subscribe.
	 * BIT0 - Disable subscribe termination indication.
	 * BIT1 - Disable match expired indication.
	 * BIT2 - Disable followUp indication received (OTA).
	 */
	uint8_t recv_indication_cfg;

	/* NAN Cipher Suite Type */
	uint32_t cipher_type;
	/* Nan Security Key Info is optional in Discovery phase.
	 * PMK or passphrase info can be passed during
	 * the NDP session.
	 */
	struct NanSecurityKeyInfo key_info;

	/* Security Context Identifiers length */
	uint32_t scid_len;

	/* Security Context Identifier attribute contains PMKID
	 * shall be included in NDP setup and response messages.
	 * Security Context Identifier, Identifies the Security
	 * Context. For NAN Shared Key Cipher Suite, this field
	 * contains the 16 octet PMKID identifying the PMK used
	 * for setting up the Secure Data Path.
	 */
	uint8_t scid[NAN_MAX_SCID_BUF_LEN];

	/* NAN configure service discovery extended attributes */
	struct NanSdeaCtrlParams sdea_params;

	/* NAN Ranging configuration */
	struct NanRangingCfg ranging_cfg;

	/* Enable/disable NAN serivce Ranging auto response mode */
	enum NanRangingAutoResponse ranging_auto_response;

	/* When the ranging_auto_response_cfg is not set, NanRangeRequestInd is
	 * received. Nan Range Response to Peer MAC Addr is notified to
	 * indicate ACCEPT/REJECT/CANCEL to the requestor.
	 */
	struct NanRangeResponseCfg range_response_cfg;

	/* Sequence of values indicating the service specific info in SDEA */
	uint16_t sdea_service_specific_info_len;
	uint8_t sdea_service_specific_info[NAN_SDEA_SERVICE_SPECIFIC_INFO_LEN];
} __KAL_ATTRIB_PACKED__;

/* NAN Subscribe Cancel Structure
 * The SubscribeCancelReq Message is used to request the DE to stop looking for
 * the Service Name.
 */
struct NanSubscribeCancelRequest {
	uint16_t subscribe_id;
} __KAL_ATTRIB_PACKED__;

/* Transmit follow up Structure
 * The TransmitFollowupReq message is sent to the DE to allow the sending of
 * the Service_Specific_Info to a particular MAC address.
 */
struct NanTransmitFollowupRequest {
	/* Publish or Subscribe Id of an earlier Publish/Subscribe */
	uint16_t publish_subscribe_id;

	/* This Id is the Requestor Instance that is passed as
	 * part of earlier MatchInd/FollowupInd message.
	 */
	uint32_t requestor_instance_id;
	/* Unicast address */
	uint8_t addr[NAN_MAC_ADDR_LEN];
	/* priority of the request 2=high */
	enum NanTxPriority priority;
	/* 0= send in a DW, 1=send in FAW */
	enum NanTransmitWindowType dw_or_faw;

	/* Sequence of values which further specify the published service
	 * beyond the service name.
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Set/Enable corresponding bits to disable responses after followUp.
	 * BIT0 - Disable followUp response from FW.
	 */
	uint8_t recv_indication_cfg;

	/* Sequence of values indicating the service specific info in SDEA */
	uint16_t sdea_service_specific_info_len;
	uint8_t sdea_service_specific_info[NAN_SDEA_SERVICE_SPECIFIC_INFO_LEN];
} __KAL_ATTRIB_PACKED__;

/* Stats Request structure
 * The Discovery Engine can be queried at runtime by the Host processor for
 * statistics concerning various parts of the Discovery Engine.
 */
struct NanStatsRequest {
	/* NAN Statistics Request Type */
	enum NanStatsType stats_type;

	uint8_t clear;
	/* 0= Do not clear the stats and return the current contents
	 * 1= Clear the associated stats
	 */
} __KAL_ATTRIB_PACKED__;

/* Config Structure
 * The NanConfigurationReq message is sent by the Host to the
 * Discovery Engine in order to configure the Discovery Engine during runtime.
 */
struct NanConfigRequest {
	uint8_t config_sid_beacon;
	uint8_t sid_beacon;
	uint8_t config_rssi_proximity;
	uint8_t rssi_proximity;

	uint8_t config_master_pref;
	uint8_t master_pref;
	/* 1 byte value which defines the RSSI filter threshold.
	 * Any Service Descriptors received above this value
	 * that are configured for RSSI filtering will be dropped.
	 * The rssi values should be specified without sign.
	 * For eg: -70dBm should be specified as 70.
	 */
	uint8_t config_5g_rssi_close_proximity;
	uint8_t rssi_close_proximity_5g_val;

	/* Optional configuration of Configure request.
	 * Each of the optional parameters have configure flag which
	 * determine whether configuration is to be passed or not.
	 */
	/* 1 byte quantity which defines the window size over
	 * which the ??average RSSI??will be calculated over.
	 */
	uint8_t config_rssi_window_size;
	uint8_t rssi_window_size_val;
	/* If set to 1, the Discovery Engine will enclose the Cluster
	 * Attribute only sent in Beacons in a Vendor Specific Attribute
	 * and transmit in a Service Descriptor Frame.
	 */
	uint8_t config_cluster_attribute_val;
	/* The periodicity in seconds between full scan??s to find any new
	 * clusters available in the area.  A Full scan should not be done
	 * more than every 10 seconds and should not be done less than every
	 * 30 seconds.
	 */
	uint8_t config_scan_params;
	struct NanSocialChannelScanParams scan_params_val;

	/* 1 byte quantity which forces the Random Factor to a particular
	 * value for all transmitted Sync/Discovery beacons
	 */
	uint8_t config_random_factor_force;
	uint8_t random_factor_force_val;
	/* 1 byte quantity which forces the HC for all transmitted Sync and
	 * Discovery Beacon NO matter the real HC being received over the
	 * air.
	 */
	uint8_t config_hop_count_force;
	uint8_t hop_count_force_val;
	/* NAN Post Connectivity Capability */
	uint8_t config_conn_capability;
	struct NanTransmitPostConnectivityCapability conn_capability_val;

	/* NAN Post Discover Capability */
	uint8_t num_config_discovery_attr;
	struct NanTransmitPostDiscovery
		discovery_attr_val[NAN_MAX_POSTDISCOVERY_LEN];

	/* NAN Further availability Map */
	uint8_t config_fam;
	struct NanFurtherAvailabilityMap fam_val;

	/* Configure 2.4/5GHz DW */
	struct NanConfigDW config_dw;

	/* By default discovery MAC address randomization is enabled
	 * and default interval value is 30 minutes i.e. 1800 seconds.
	 * The value 0 is used to disable MAC addr randomization.
	 */
	uint8_t config_disc_mac_addr_randomization;
	uint32_t disc_mac_addr_rand_interval_sec;
	/* Set/Enable corresponding bits to disable Discovery indications:
	 * BIT0 - Disable Discovery MAC Address Event.
	 * BIT1 - Disable Started Cluster Event.
	 * BIT2 - Disable Joined Cluster Event.
	 */
	uint8_t discovery_indication_cfg;
	/* BIT 0 is used to specify to include Service IDs in Sync/Discovery
	 * beacons
	 * 0 - Do not include SIDs in any beacons
	 * 1 - Include SIDs in all beacons.
	 * Rest 7 bits are count field which allows control over the number
	 * of SIDs included in the Beacon.
	 * 0 means to include as many SIDs that fit into
	 * the maximum allow Beacon frame size
	 */
	uint8_t config_subscribe_sid_beacon;
	uint32_t subscribe_sid_beacon_val;
} __KAL_ATTRIB_PACKED__;

/* Beacon Sdf Payload Structure
 * The Discovery Engine can be configured to publish vendor specific attributes
 * as part of
 * beacon or service discovery frame transmitted as part of this request..
 */
struct NanBeaconSdfPayloadRequest {
	/* NanVendorAttribute will have the Vendor Specific Attribute which the
	 * vendor wants to publish as part of Discovery or Sync or
	 * Service discovery frame
	 */
	struct NanTransmitVendorSpecificAttribute vsa;
} __KAL_ATTRIB_PACKED__;

/* Publish statistics. */
struct NanPublishStats {
	uint32_t validPublishServiceReqMsgs;
	uint32_t validPublishServiceRspMsgs;
	uint32_t validPublishServiceCancelReqMsgs;
	uint32_t validPublishServiceCancelRspMsgs;
	uint32_t validPublishRepliedIndMsgs;
	uint32_t validPublishTerminatedIndMsgs;
	uint32_t validActiveSubscribes;
	uint32_t validMatches;
	uint32_t validFollowups;
	uint32_t invalidPublishServiceReqMsgs;
	uint32_t invalidPublishServiceCancelReqMsgs;
	uint32_t invalidActiveSubscribes;
	uint32_t invalidMatches;
	uint32_t invalidFollowups;
	uint32_t publishCount;
	uint32_t publishNewMatchCount;
	uint32_t pubsubGlobalNewMatchCount;
} __KAL_ATTRIB_PACKED__;

/* Subscribe statistics. */
struct NanSubscribeStats {
	uint32_t validSubscribeServiceReqMsgs;
	uint32_t validSubscribeServiceRspMsgs;
	uint32_t validSubscribeServiceCancelReqMsgs;
	uint32_t validSubscribeServiceCancelRspMsgs;
	uint32_t validSubscribeTerminatedIndMsgs;
	uint32_t validSubscribeMatchIndMsgs;
	uint32_t validSubscribeUnmatchIndMsgs;
	uint32_t validSolicitedPublishes;
	uint32_t validMatches;
	uint32_t validFollowups;
	uint32_t invalidSubscribeServiceReqMsgs;
	uint32_t invalidSubscribeServiceCancelReqMsgs;
	uint32_t invalidSubscribeFollowupReqMsgs;
	uint32_t invalidSolicitedPublishes;
	uint32_t invalidMatches;
	uint32_t invalidFollowups;
	uint32_t subscribeCount;
	uint32_t bloomFilterIndex;
	uint32_t subscribeNewMatchCount;
	uint32_t pubsubGlobalNewMatchCount;
} __KAL_ATTRIB_PACKED__;

/* NAN DW Statistics*/
struct NanDWStats {
	/* RX stats */
	uint32_t validFrames;
	uint32_t validActionFrames;
	uint32_t validBeaconFrames;
	uint32_t ignoredActionFrames;
	uint32_t ignoredBeaconFrames;
	uint32_t invalidFrames;
	uint32_t invalidActionFrames;
	uint32_t invalidBeaconFrames;
	uint32_t invalidMacHeaders;
	uint32_t invalidPafHeaders;
	uint32_t nonNanBeaconFrames;

	uint32_t earlyActionFrames;
	uint32_t inDwActionFrames;
	uint32_t lateActionFrames;

	/* TX stats */
	uint32_t framesQueued;
	uint32_t totalTRSpUpdates;
	uint32_t completeByTRSp;
	uint32_t completeByTp75DW;
	uint32_t completeByTendDW;
	uint32_t lateActionFramesTx;
} __KAL_ATTRIB_PACKED__;

/* NAN MAC Statistics. */
struct NanMacStats {
	/* RX stats */
	uint32_t validFrames;
	uint32_t validActionFrames;
	uint32_t validBeaconFrames;
	uint32_t ignoredActionFrames;
	uint32_t ignoredBeaconFrames;
	uint32_t invalidFrames;
	uint32_t invalidActionFrames;
	uint32_t invalidBeaconFrames;
	uint32_t invalidMacHeaders;
	uint32_t invalidPafHeaders;
	uint32_t nonNanBeaconFrames;

	uint32_t earlyActionFrames;
	uint32_t inDwActionFrames;
	uint32_t lateActionFrames;

	/* TX stats */
	uint32_t framesQueued;
	uint32_t totalTRSpUpdates;
	uint32_t completeByTRSp;
	uint32_t completeByTp75DW;
	uint32_t completeByTendDW;
	uint32_t lateActionFramesTx;

	uint32_t twIncreases;
	uint32_t twDecreases;
	uint32_t twChanges;
	uint32_t twHighwater;
	uint32_t bloomFilterIndex;
} __KAL_ATTRIB_PACKED__;

/* Fixme, add padding for 4-byte alignment */

/* NAN Sync Statistics*/
struct NanSyncStats {
	unsigned long long currTsf;
	unsigned long long myRank;
	unsigned long long currAmRank;
	unsigned long long lastAmRank;
	uint32_t currAmBTT;
	uint32_t lastAmBTT;
	uint8_t currAmHopCount;
	uint8_t currRole;
	uint16_t currClusterId;

	unsigned long long timeSpentInCurrRole;
	unsigned long long totalTimeSpentAsMaster;
	unsigned long long totalTimeSpentAsNonMasterSync;
	unsigned long long totalTimeSpentAsNonMasterNonSync;
	uint32_t transitionsToAnchorMaster;
	uint32_t transitionsToMaster;
	uint32_t transitionsToNonMasterSync;
	uint32_t transitionsToNonMasterNonSync;
	uint32_t amrUpdateCount;
	uint32_t amrUpdateRankChangedCount;
	uint32_t amrUpdateBTTChangedCount;
	uint32_t amrUpdateHcChangedCount;
	uint32_t amrUpdateNewDeviceCount;
	uint32_t amrExpireCount;
	uint32_t mergeCount;
	uint32_t beaconsAboveHcLimit;
	uint32_t beaconsBelowRssiThresh;
	uint32_t beaconsIgnoredNoSpace;
	uint32_t beaconsForOurCluster;
	uint32_t beaconsForOtherCluster;
	uint32_t beaconCancelRequests;
	uint32_t beaconCancelFailures;
	uint32_t beaconUpdateRequests;
	uint32_t beaconUpdateFailures;
	uint32_t syncBeaconTxAttempts;
	uint32_t syncBeaconTxFailures;
	uint32_t discBeaconTxAttempts;
	uint32_t discBeaconTxFailures;
	uint32_t amHopCountExpireCount;
	uint32_t ndpChannelFreq;
	uint32_t ndpChannelFreq2;
} __KAL_ATTRIB_PACKED__;

/* NAN Misc DE Statistics */
struct NanDeStats {
	uint32_t validErrorRspMsgs;
	uint32_t validTransmitFollowupReqMsgs;
	uint32_t validTransmitFollowupRspMsgs;
	uint32_t validFollowupIndMsgs;
	uint32_t validConfigurationReqMsgs;
	uint32_t validConfigurationRspMsgs;
	uint32_t validStatsReqMsgs;
	uint32_t validStatsRspMsgs;
	uint32_t validEnableReqMsgs;
	uint32_t validEnableRspMsgs;
	uint32_t validDisableReqMsgs;
	uint32_t validDisableRspMsgs;
	uint32_t validDisableIndMsgs;
	uint32_t validEventIndMsgs;
	uint32_t validTcaReqMsgs;
	uint32_t validTcaRspMsgs;
	uint32_t validTcaIndMsgs;
	uint32_t invalidTransmitFollowupReqMsgs;
	uint32_t invalidConfigurationReqMsgs;
	uint32_t invalidStatsReqMsgs;
	uint32_t invalidEnableReqMsgs;
	uint32_t invalidDisableReqMsgs;
	uint32_t invalidTcaReqMsgs;
} __KAL_ATTRIB_PACKED__;

/* Publish Response Message structure */
struct NanPublishResponse {
	uint16_t publish_id;
} __KAL_ATTRIB_PACKED__;

/* Subscribe Response Message structure */
struct NanSubscribeResponse {
	uint16_t subscribe_id;
} __KAL_ATTRIB_PACKED__;

/* Stats Response Message structure
 * The Discovery Engine response to a request by the Host for statistics.
 */
struct NanStatsResponse {
	enum NanStatsType stats_type;
	union {
		struct NanPublishStats publish_stats;
		struct NanSubscribeStats subscribe_stats;
		struct NanMacStats mac_stats;
		struct NanSyncStats sync_stats;
		struct NanDeStats de_stats;
		struct NanDWStats dw_stats;
	} data;
} __KAL_ATTRIB_PACKED__;

/* Response returned for Initiators Data request */
struct NanDataPathRequestResponse {
	/* Unique token Id generated on the initiator
	 * side used for a NDP session between two NAN devices
	 */
	uint32_t ndp_instance_id;
} __KAL_ATTRIB_PACKED__;

/* Publish Replied Indication
 * The PublishRepliedInd Message is sent by the DE when an Active Subscribe is
 * received over the air and it matches a Solicited PublishServiceReq which had
 * been created with the replied_event_flag set.
 */
struct NanPublishRepliedInd {
	uint8_t eventID;
	/* A 32 bit Requestor Instance Id which is sent to the Application.
	 * This Id will be sent in any subsequent UnmatchInd/FollowupInd
	 * messages
	 */
	uint16_t pubid;
	uint16_t subid;
	uint8_t addr[NAN_MAC_ADDR_LEN];
	/* If RSSI filtering was configured in NanPublishRequest then this
	 * field will contain the received RSSI value. 0 if not
	 */
	uint8_t rssi_value;
} __KAL_ATTRIB_PACKED__;

/* Publish Terminated
 * The PublishTerminatedInd message is sent by the DE whenever a Publish
 * terminates from a user-specified timeout or a unrecoverable error in the DE.
 */
struct NanPublishTerminatedInd {
	uint8_t eventID;
	/* Id returned during the initial Publish */
	uint16_t publish_id;
	/* For all user configured termination NAN_STATUS_SUCCESS
	 * and no other reasons expected from firmware.
	 */
	enum NanStatusType reason;
	/* Describe the NAN reason type */
	uint8_t nan_reason[NAN_ERROR_STR_LEN];
} __KAL_ATTRIB_PACKED__;

/* Match Indication
 * The MatchInd message is sent once per responding MAC address whenever
 * the Discovery Engine detects a match for a previous SubscribeServiceReq
 * or PublishServiceReq.
 */
struct NanMatchInd {
	uint8_t eventID;
	/* Publish or Subscribe Id of an earlier Publish/Subscribe */
	uint16_t publish_subscribe_id;
	/* A 32 bit Requestor Instance Id which is sent to the Application.
	 * This Id will be sent in any subsequent UnmatchInd/FollowupInd
	 * messages
	 */
	uint16_t requestor_instance_id;
	uint8_t addr[NAN_MAC_ADDR_LEN];

	/* Sequence of octets which were received in a Discovery Frame matching
	 * the Subscribe Request.
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Ordered sequence of <length, value> pairs received in the Discovery
	 * Frame matching the Subscribe Request.
	 */
	uint16_t sdf_match_filter_len;
	uint8_t sdf_match_filter[NAN_MAX_MATCH_FILTER_LEN];

	/* flag to indicate if the Match occurred in a Beacon Frame or in a
	 * Service Discovery Frame.
	 * 0 - Match occurred in a Service Discovery Frame
	 * 1 - Match occurred in a Beacon Frame
	 */
	uint8_t match_occurred_flag;

	/* flag to indicate FW is out of resource and that it can no longer
	 * track this Service Name. The Host still need to send the received
	 * Match_Handle but duplicate MatchInd messages may be received on
	 * this Handle until the resource frees up.
	 * 0 - FW is caching this match
	 * 1 - FW is unable to cache this match
	 */
	uint8_t out_of_resource_flag;

	/* If RSSI filtering was configured in NanSubscribeRequest then this
	 * field will contain the received RSSI value. 0 if not.
	 * All rssi values should be specified without sign.
	 * For eg: -70dBm should be specified as 70.
	 */
	uint8_t rssi_value;

	/* optional attributes. Each optional attribute is associated with a
	 * flag which specifies whether the attribute is valid or not
	 */
	/* NAN Post Connectivity Capability received */
	uint8_t is_conn_capability_valid;
	struct NanReceivePostConnectivityCapability conn_capability;

	/* NAN Post Discover Capability */
	uint8_t num_rx_discovery_attr;
	struct NanReceivePostDiscovery
		discovery_attr[NAN_MAX_POSTDISCOVERY_LEN];

	/* NAN Further availability Map */
	uint8_t num_chans;
	struct NanFurtherAvailabilityChannel famchan[NAN_MAX_FAM_CHANNELS];

	/* NAN Cluster Attribute */
	uint8_t cluster_attribute_len;
	uint8_t cluster_attribute[NAN_MAX_CLUSTER_ATTRIBUTE_LEN];

	/* NAN Cipher Suite */
	uint32_t peer_cipher_type;

	/* Security Context Identifiers length */
	uint32_t scid_len;
	/* Security Context Identifier attribute contains PMKID
	 * shall be included in NDP setup and response messages.
	 * Security Context Identifier, Identifies the Security
	 * Context. For NAN Shared Key Cipher Suite, this field
	 * contains the 16 octet PMKID identifying the PMK used
	 * for setting up the Secure Data Path.
	 */
	uint8_t scid[NAN_MAX_SCID_BUF_LEN];

	/* Peer service discovery extended attributes */
	struct NanSdeaCtrlParams peer_sdea_params;

	/* Ranging indication and NanMatchAlg are not tied.
	 * Ex: NanMatchAlg can indicate Match_ONCE, but ranging
	 * indications can be continuous. All ranging indications
	 * depend on SDEA control parameters of ranging required for
	 * continuous, and ingress/egress values in the ranging config.
	 * Ranging indication data is notified if:
	 * 1) Ranging required is enabled in SDEA.
	 *    range info notified continuous.
	 * 2) if range_limit ingress/egress MASKS are enabled
	 *    notify once for ingress >= ingress_distance
	 *    and egress <= egress_distance, same for ingress_egress_both
	 * 3) if the Awake DW intervals are higher than the ranging intervals,
	 *    priority is given to the device DW intervalsi.
	 */
	/* Range Info includes:
	 * 1) distance to the NAN device with the MAC address indicated
	 *    with ranged mac address.
	 * 2) Ranging event matching the configuration
	 *    of continuous/ingress/egress.
	 */
	struct NanRangeInfo range_info;

	/* Sequence of values indicating the service specific info in SDEA */
	uint16_t sdea_service_specific_info_len;
	uint8_t sdea_service_specific_info[NAN_SDEA_SERVICE_SPECIFIC_INFO_LEN];
} __KAL_ATTRIB_PACKED__;

/* MatchExpired Indication
 * The MatchExpiredInd message is sent whenever the Discovery Engine detects
 * that
 * a previously Matched Service has been gone for too long. If the previous
 * MatchInd message for this Publish/Subscribe Id had the out_of_resource_flag
 * set then this message will not be received
 */
struct NanMatchExpiredInd {
	uint8_t eventID;
	/* Publish or Subscribe Id of an earlier Publish/Subscribe */
	uint16_t publish_subscribe_id;
	/* 32 bit value sent by the DE in a previous
	 * MatchInd/FollowupInd to the application.
	 */
	uint32_t requestor_instance_id;
} __KAL_ATTRIB_PACKED__;

/* Subscribe Terminated
 * The SubscribeTerminatedInd message is sent by the DE whenever a
 * Subscribe terminates from a user-specified timeout or a unrecoverable error
 * in the DE.
 */
struct NanSubscribeTerminatedInd {
	uint8_t eventID;
	/* Id returned during initial Subscribe */
	uint16_t subscribe_id;
	/* For all user configured termination NAN_STATUS_SUCCESS
	 * and no other reasons expected from firmware.
	 */
	enum NanStatusType reason;
	/* Describe the NAN reason type */
	uint8_t nan_reason[NAN_ERROR_STR_LEN];
} __KAL_ATTRIB_PACKED__;

/* Followup Indication Message
 * The FollowupInd message is sent by the DE to the Host whenever it receives a
 * Followup message from another peer.
 */
struct NanFollowupInd {
	uint8_t eventID;
	/* Publish or Subscribe Id of an earlier Publish/Subscribe */
	uint16_t publish_subscribe_id;

	/* A 32 bit Requestor instance Id which is sent to the Application.
	 * This Id will be used in subsequent UnmatchInd/FollowupInd messages.
	 */
	uint16_t requestor_instance_id;
	uint8_t addr[NAN_MAC_ADDR_LEN];

	/* Flag which the DE uses to decide if received in a DW or a FAW
	 * 0=Received  in a DW, 1 = Received in a FAW
	 */
	uint8_t dw_or_faw;
	/* Sequence of values which further specify the published service
	 * beyond the service name
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Sequence of values indicating the service specific info in SDEA */
	uint16_t sdea_service_specific_info_len;
	uint8_t sdea_service_specific_info[NAN_SDEA_SERVICE_SPECIFIC_INFO_LEN];
} __KAL_ATTRIB_PACKED__;

/* Event data notifying the Mac address of the Discovery engine.
 * which is reported as one of the Discovery engine event
 */
struct NanMacAddressEvent {
	uint8_t addr[NAN_MAC_ADDR_LEN];
} __KAL_ATTRIB_PACKED__;

/* Event data notifying the Cluster address of the cluster
 * which is reported as one of the Discovery engine event
 */
struct NanClusterEvent {
	uint8_t addr[NAN_MAC_ADDR_LEN];
} __KAL_ATTRIB_PACKED__;

/* Discovery Engine Event Indication
 * The Discovery Engine can inform the Host when significant events occur
 * The data following the EventId is dependent upon the EventId type.
 * In other words, each new event defined will carry a different
 * structure of information back to the host.
 */
struct NanDiscEngEventInd {
	uint8_t eventID;
	/* NAN Protocol Event Codes */
	enum NanDiscEngEventType event_type;
	union {
		/* MacAddressEvent which will have 6 byte mac address
		 * of the Discovery engine.
		 */
		struct NanMacAddressEvent mac_addr;
		/* Cluster Event Data which will be obtained when the
		 * device starts a new cluster or joins a cluster.
		 * The event data will have 6 byte octet string of the
		 * cluster started or joined.
		 */
		struct NanClusterEvent cluster;
	} data;
} __KAL_ATTRIB_PACKED__;

/* NAN Disabled Indication
 * The NanDisableInd message indicates to the upper layers that the Discovery
 * Engine has flushed all state and has been shutdown.
 * When this message is received
 * the DE is guaranteed to have left the NAN cluster
 * it was part of and will have terminated
 * any in progress Publishes or Subscribes.
 */
struct NanDisabledInd {
	uint8_t eventID;
	/* Following reasons expected:
	 * NAN_STATUS_SUCCESS
	 * NAN_STATUS_UNSUPPORTED_CONCURRENCY_NAN_DISABLED
	 */
	enum NanStatusType reason;
	/* Describe the NAN reason type */
	uint8_t nan_reason[NAN_ERROR_STR_LEN];
} __KAL_ATTRIB_PACKED__;

/* NAN Beacon or SDF Payload Indication
 * The NanBeaconSdfPayloadInd message indicates to the upper layers that
 *  information
 * elements were received either in a Beacon or SDF which needs to be delivered
 * outside of a Publish/Subscribe Handle.
 */
struct NanBeaconSdfPayloadInd {
	uint8_t eventID;
	/* The MAC address of the peer which sent the attributes. */
	uint8_t addr[NAN_MAC_ADDR_LEN];
	/* Optional attributes. Each optional attribute is associated with
	 * a flag which specifies whether the attribute is valid or not
	 */
	/* NAN Receive Vendor Specific Attribute */
	uint8_t is_vsa_received;
	struct NanReceiveVendorSpecificAttribute vsa;

	/* NAN Beacon or SDF Payload Received */
	uint8_t is_beacon_sdf_payload_received;
	struct NanBeaconSdfPayloadReceive data;
} __KAL_ATTRIB_PACKED__;

/* Event Indication notifying the
 * transmit followup in progress
 */
struct NanTransmitFollowupInd {
	uint8_t eventID;
	uint16_t id;
	/* Following reason codes returned:
	 * NAN_STATUS_SUCCESS
	 * NAN_STATUS_NO_OTA_ACK
	 * NAN_STATUS_PROTOCOL_FAILURE
	 */
	enum NanStatusType reason;
	/* Describe the NAN reason type */
	uint8_t nan_reason[NAN_ERROR_STR_LEN];
} __KAL_ATTRIB_PACKED__;

/* Data request Initiator/Responder
 * app/service related info
 */
struct NanDataPathAppInfo {
	uint16_t ndp_app_info_len;
	uint8_t ndp_app_info[NAN_DP_MAX_APP_INFO_LEN];
} __KAL_ATTRIB_PACKED__;

/* Configuration params of Data request Initiator/Responder */
struct NanDataPathCfg {
	/* Status Indicating Security/No Security */
	enum NanDataPathSecurityCfgStatus security_cfg;
	enum NanDataPathQosCfg qos_cfg;
} __KAL_ATTRIB_PACKED__;

/* Nan Data Path Initiator requesting a data session */
struct NanDataPathInitiatorRequest {
	/* Unique Instance Id identifying the Responder's service.
	 * This is same as publish_id notified on the subscribe side
	 * in a publish/subscribe scenario
	 * Value 0 for no publish/subscribe
	 */
	uint32_t requestor_instance_id;

	/* Config flag for channel request */
	enum NanDataPathChannelCfg channel_request_type;
	/* Channel frequency in MHz to start data-path */
	uint32_t channel;
	/* Discovery MAC addr of the publisher/peer */
	uint8_t peer_disc_mac_addr[NAN_MAC_ADDR_LEN];
	/* Interface name on which this NDP session is to be started.
	 * This will be the same interface name provided during interface
	 * create.
	 */
	uint8_t ndp_iface[IFNAMSIZ + 1];
	/* Initiator/Responder Security/QoS configuration */
	struct NanDataPathCfg ndp_cfg;
	/* App/Service information of the Initiator */
	struct NanDataPathAppInfo app_info;
	/* NAN Cipher Suite Type */
	uint32_t cipher_type;
	/* Nan Security Key Info is optional in Discovery phase.
	 * PMK or passphrase info can be passed during
	 * the NDP session.
	 */
	struct NanSecurityKeyInfo key_info;
	/* length of service name */
	uint32_t service_name_len;
	/* UTF-8 encoded string identifying the service name.
	 * The service name field is only used if a Nan discovery
	 * is not associated with the NDP (out-of-band discovery).
	 */
	uint8_t service_name[NAN_MAX_SERVICE_NAME_LEN];
	uint32_t unicast_traffic_identifier;
	uint32_t service_pkt_size;
	uint16_t mean_data_rate;
	uint16_t max_service_interal;

	/* NDP integration */
	uint8_t type; /*0:unicast, 1:multicast */

	/* Security Context Identifiers length */
	uint32_t scid_len;
	/* Security Context Identifier attribute contains PMKID
	 * shall be included in NDP setup and response messages.
	 * Security Context Identifier, Identifies the Security
	 * Context. For NAN Shared Key Cipher Suite, this field
	 * contains the 16 octet PMKID identifying the PMK used
	 * for setting up the Secure Data Path.
	 */
	uint8_t scid[NAN_MAX_SCID_BUF_LEN];
	/* NAN 3 Capability */
	uint8_t fgCarryIpv6;
	uint8_t aucIPv6Addr[IPV6MACLEN];

	uint8_t ucMinTimeSlot;
	uint16_t u2MaxLatency;
} __KAL_ATTRIB_PACKED__;

struct NanDataPathInitiatorNDPE {
	bool fgEnNDPE;
	uint8_t ucNDPEAttrPresent;
} __KAL_ATTRIB_PACKED__;

/* Data struct to initiate a data response on the responder side
 * for an indication received with a data request
 */
struct NanDataPathIndicationResponse {
	/* Unique token Id generated on the initiator/responder
	 * side used for a NDP session between two NAN devices
	 */
	uint32_t ndp_instance_id;

	/* Interface name on which this NDP session is to be started.
	 * This will be the same interface name provided during interface
	 * create.
	 */
	uint8_t ndp_iface[IFNAMSIZ + 1];
	/* Initiator/Responder Security/QoS configuration */
	struct NanDataPathCfg ndp_cfg;
	/* App/Service information of the responder */
	struct NanDataPathAppInfo app_info;
	/* Response Code indicating ACCEPT/REJECT/DEFER */
	enum NanDataPathResponseCode rsp_code;
	/* NAN Cipher Suite Type */
	uint32_t cipher_type;
	/* Nan Security Key Info is optional in Discovery phase.
	 * PMK or passphrase info can be passed during
	 * the NDP session.
	 */
	struct NanSecurityKeyInfo key_info;
	/* length of service name */
	uint32_t service_name_len;
	/* UTF-8 encoded string identifying the service name.
	 * The service name field is only used if a Nan discovery
	 * is not associated with the NDP (out-of-band discovery).
	 */
	uint8_t service_name[NAN_MAX_SERVICE_NAME_LEN];
	uint32_t unicast_traffic_identifier;
	uint32_t service_pkt_size;
	uint16_t mean_data_rate;
	uint16_t max_service_interal;

	/* NDP integration */
	uint8_t type; /*0:unicast, 1:multicast */
	uint8_t initiator_mac_addr[NAN_MAC_ADDR_LEN];

	/* Security Context Identifiers length */
	uint32_t scid_len;
	/* Security Context Identifier attribute contains PMKID
	 * shall be included in NDP setup and response messages.
	 * Security Context Identifier, Identifies the Security
	 * Context. For NAN Shared Key Cipher Suite, this field
	 * contains the 16 octet PMKID identifying the PMK used
	 * for setting up the Secure Data Path.
	 */
	uint8_t scid[NAN_MAX_SCID_BUF_LEN];
	/* NAN 3 Capability */
	uint8_t fgCarryIpv6;
	uint8_t aucIPv6Addr[IPV6MACLEN];
	uint16_t u2PortNum;
	uint8_t ucServiceProtocolType;

	uint8_t ucMinTimeSlot;
	uint16_t u2MaxLatency;
} __KAL_ATTRIB_PACKED__;

/* NDP termination info */
struct NanDataPathEndRequest {
	/* UINT_8 num_ndp_instances; */

	/* Unique token Id generated on the initiator/responder side
	 * used for a NDP session between two NAN devices
	 */
	uint32_t ndp_instance_id;

	/* NDP integration */
	uint8_t type; /*0:unicast, 1:multicast */
	uint8_t initiator_mac_addr[NAN_MAC_ADDR_LEN];

} __KAL_ATTRIB_PACKED__;

/* Event indication received on the
 * responder side when a Nan Data request or
 * NDP session is initiated on the Initiator side
 */
struct NanDataPathRequestInd {
	uint8_t eventID;
	/* Unique Instance Id corresponding to a service/session.
	 * This is similar to the publish_id generated on the
	 * publisher side
	 */
	uint16_t service_instance_id;
	/* Discovery MAC addr of the peer/initiator */
	uint8_t peer_disc_mac_addr[NAN_MAC_ADDR_LEN];
	/* Unique token Id generated on the initiator/responder side
	 * used for a NDP session between two NAN devices
	 */
	uint32_t ndp_instance_id;
	uint8_t fgSupportNDPE;
	uint8_t aucIPv6Addr[IPV6MACLEN];
	/* Initiator/Responder Security/QoS configuration */
	struct NanDataPathCfg ndp_cfg;
	/* App/Service information of the initiator */
	struct NanDataPathAppInfo app_info;
	uint8_t aucSCID[NAN_SCID_DEFAULT_LEN];
	uint8_t uCipher;
} __KAL_ATTRIB_PACKED__;

/* Event indication of data confirm is received on both
 * initiator and responder side confirming a NDP session
 */
struct NanDataPathConfirmInd {
	uint8_t eventID;
	/* Unique token Id generated on the initiator/responder side
	 * used for a NDP session between two NAN devices
	 */
	uint32_t ndp_instance_id;
	/* NDI mac address of the peer
	 * (required to derive target ipv6 address)
	 */
	uint8_t peer_ndi_mac_addr[NAN_MAC_ADDR_LEN];
	uint8_t fgSupportNDPE;
	uint8_t aucIPv6Addr[IPV6MACLEN];
	uint16_t u2Port;
	uint8_t ucProtocol;
	/* App/Service information of Initiator/Responder */
	struct NanDataPathAppInfo app_info;
	/* Response code indicating ACCEPT/REJECT/DEFER */
	enum NanDataPathResponseCode rsp_code;
	/* Reason code indicating the cause for REJECT.
	 * NAN_STATUS_SUCCESS and NAN_STATUS_PROTOCOL_FAILURE are
	 * expected reason codes.
	 */
	enum NanStatusType reason_code;

	uint8_t fgIsSec;
} __KAL_ATTRIB_PACKED__;

/* Event indication received on the
 * initiator/responder side terminating
 * a NDP session
 */
struct NanDataPathEndInd {
	uint8_t eventID;
	uint8_t num_ndp_instances;
	/* Unique token Id generated on the initiator/responder side
	 * used for a NDP session between two NAN devices
	 */
	uint32_t ndp_instance_id;
} __KAL_ATTRIB_PACKED__;

/* Event indicating Range Request received on the
 * Published side.
 */
struct NanRangeRequestInd {
	uint8_t eventID;
	uint16_t publish_id; /* id is existing publish */
	/* Range Requestor's MAC address */
	uint8_t range_req_intf_addr[NAN_MAC_ADDR_LEN];
	/* Configuration parameters received from the Ranging Request frame */
	struct NanRangeRequestCfg request_cfg;
} __KAL_ATTRIB_PACKED__;

/* Event indicating Range report on the
 * Published side.
 */
struct NanRangeReportInd {
	uint8_t eventID;
	/* id is existing publish */
	uint16_t publish_id;
	/* Range Requestor's MAC address */
	uint8_t range_req_intf_addr[NAN_MAC_ADDR_LEN];
	/* Distance to the NAN device with the MAC address indicated
	 * with ranged mac address.
	 */
	uint32_t range_measurement_cm;
	/* Ranging event matching the configuration of
	 * continuous/ingress/egress.
	 */
	uint32_t ranging_event_type;
} __KAL_ATTRIB_PACKED__;

/* NAN Range Request Structure
 * The message is sent to the Ranging Engine
 * whenever the Upper layers would like to invoke a ranging service
 */
struct NanRangeRequest {
	/* A handle uniquely identifying a ranging */
	uint16_t range_id;
	/* Range Responder's MAC address */
	uint8_t peer_addr[NAN_MAC_ADDR_LEN];
	/* NAN Ranging configuration */
	struct NanRangingCfg ranging_cfg;
} __KAL_ATTRIB_PACKED__;

/* NAN Range Cancel Structure
 * The message is used to request the Ranging engine to stop a session.
 */
struct NanRangeCancelRequest {
	/* A handle uniquely identifying a ranging */
	uint8_t peer_addr[NAN_MAC_ADDR_LEN];
} __KAL_ATTRIB_PACKED__;

/* NAN Range Response Structure
 * The message is sent to initiate a range response
 * for an indication received with a range request
 */
struct NanRangeResponse {
	/* A handle uniquely identifying a ranging */
	uint16_t range_id;
	/* NAN Ranging configuration */
	struct NanRangingCfg ranging_cfg;
	/* NAN Ranging response control parameters*/
	struct NanRangeResponseCtl response_ctl;
} __KAL_ATTRIB_PACKED__;

#endif
#endif /* __NAN_INTF_H__ */
