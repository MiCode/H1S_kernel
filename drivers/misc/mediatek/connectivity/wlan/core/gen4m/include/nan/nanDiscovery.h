/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef _NAN_DISCOVERY_H_
#define _NAN_DISCOVERY_H_

#if CFG_SUPPORT_NAN

extern uint8_t g_u2IndPubId;

struct NAN_DISCOVERY_EVENT {
	uint16_t u2SubscribeID;
	uint16_t u2PublishID;
	uint16_t u2Service_info_len;
	uint8_t aucSerive_specificy_info[255];
	uint16_t u2Service_update_indicator;
	uint8_t aucNanAddress[6];
	uint8_t ucRange_measurement;
	uint8_t ucFSDType;
	uint8_t ucDataPathParm;
	uint8_t aucSecurityInfo[32];
};

/* Followup event
 * The Followup message is sent by the DE to the Host whenever it receives a
 * Followup message from another peer.
 */
struct NAN_FOLLOW_UP_EVENT {
	/* Publish or Subscribe Id of an earlier Publish/Subscribe */
	uint16_t publish_subscribe_id;
	/* A 32 bit Requestor instance Id which is sent to the Application.
	 * This Id will be used in subsequent UnmatchInd/FollowupInd messages.
	 */
	uint32_t requestor_instance_id;
	uint8_t addr[NAN_MAC_ADDR_LEN];

	/* Flag which the DE uses to decide if received in a DW or a FAW */
	uint8_t dw_or_faw; /* 0=Received  in a DW, 1 = Received in a FAW */

	/* Sequence of values which further specify the published service
	 * beyond the service name
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Sequence of values indicating the service specific info in SDEA */
	uint16_t sdea_service_specific_info_len;
	uint8_t sdea_service_specific_info[NAN_SDEA_SERVICE_SPECIFIC_INFO_LEN];
};

struct NAN_REPLIED_EVENT {
	uint16_t u2Pubid;
	uint16_t u2Subid;
	uint8_t auAddr[MAC_ADDR_LEN];

	/* If RSSI filtering was configured in NanPublishRequest then this
	 * field will contain the received RSSI value. 0 if not
	 */
	uint8_t ucRssi_value;
};

struct NAN_PUBLISH_TERMINATE_EVENT {
	uint16_t u2Pubid;
	uint8_t ucReasonCode;
};

struct NAN_SUBSCRIBE_TERMINATE_EVENT {
	uint16_t u2Subid;
	uint8_t ucReasonCode;
};

/* Publish Msg Structure
 * Message is used to request the DE to publish the Service Name
 * using the parameters passed into the Discovery Window
 */
struct NanFWPublishRequest {
	/* id  0 means new publish, any other id is existing publish */
	uint16_t publish_id;
	/* how many seconds to run for. 0 means forever until canceled */
	uint16_t ttl;
	/* period: Awake DW Interval for publish(service)
	 * Indicates the interval between two Discovery Windows in which
	 * the device supporting the service is awake to transmit or
	 * receive the Service Discovery frames.
	 * Valid values of Awake DW Interval are: 1, 2, 4, 8 and 16,
	 * value 0 will	   default to 1.
	 */
	uint16_t period;
	uint16_t service_name_len; /* length of service name */
	/* UTF-8 encoded string identifying the service */
	uint8_t service_name[NAN_FW_MAX_SERVICE_NAME_LEN];

	/* number of OTA Publish, 0 means forever until canceled */
	uint8_t publish_count;

	enum NanPublishType
		publish_type; /* 0= unsolicited, solicited = 1, 2= both */
	enum NanTxType
		tx_type; /* 0 = broadcast, 1= unicast  if solicited publish */

	/* Field which specifies how the matching indication to host is
	 * controlled.
	 * 0 - Match and Indicate Once
	 * 1 - Match and Indicate continuous
	 * 2 - Match and Indicate never. This means don't indicate the
	 *	   match to the host.
	 * 3 - Reserved
	 */
	enum NanMatchAlg publish_match_indicator;

	/* Sequence of values
	 * NAN Device that has invoked a Subscribe method corresponding to this
	 * Publish method
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_FW_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Ordered sequence of <length, value> pairs which specify further
	 * response conditions beyond the service name used to filter subscribe
	 * messages to respond to. This is only needed when the PT is set to
	 * NAN_SOLICITED or NAN_SOLICITED_UNSOLICITED.
	 */
	uint8_t rx_match_filter[NAN_FW_MAX_MATCH_FILTER_LEN];
	uint16_t rx_match_filter_len;

	/* Ordered sequence of <length, value> pairs to be included in the
	 * Discovery Frame.If present it is always sent in a Discovery Frame
	 */
	uint16_t tx_match_filter_len;
	uint8_t tx_match_filter[NAN_FW_MAX_MATCH_FILTER_LEN];

	/* flag which specifies that the Publish should use the configured RSSI
	 * threshold and the received RSSI in order to filter requests
	 * 0 - ignore the configured RSSI threshold when running a Service
	 * Descriptor attribute or Service ID List Attribute through the DE
	 * matching logic. use the configured RSSI threshold when running a
	 * Service
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
	/* Set/Enable corresponding bits to disable any indications that follow
	 * a publish.
	 * BIT0 - Disable publish termination indication.
	 * BIT1 - Disable match expired indication.
	 * BIT2 - Disable followUp indication received (OTA).
	 * BIT3 - Disable publishReplied indication.
	 */
	uint8_t recv_indication_cfg;
	/* Nan accept policy for the specific service(publish)
	 * NanServiceAcceptPolicy service_responder_policy;
	 * NAN Cipher Suite Type
	 */
	uint32_t cipher_type;

	/* Security Context Identifiers length */
	uint32_t scid_len;

	/* Security Context Identifier attribute contains PMKID
	 * shall be included in NDP setup and response messages.
	 * Security Context Identifier, Identifies the Security
	 * Context. For NAN Shared Key Cipher Suite, this field
	 * contains the 16 octet PMKID identifying the PMK used
	 * for setting up the Secure Data Path.
	 */
	uint8_t scid[NAN_FW_MAX_SCID_BUF_LEN];

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
	uint8_t sdea_service_specific_info[NAN_FW_SDEA_SPECIFIC_INFO_LEN];

	/* Nan Security Key Info is optional in Discovery phase.
	 * PMK or passphrase info can be passed during
	 * the NDP session.
	 */
	struct NanSecurityKeyInfo key_info;

	uint8_t service_name_hash[NAN_SERVICE_HASH_LENGTH];
} __KAL_ATTRIB_PACKED__;

/* NAN Subscribe Structure
 * The SubscribeServiceReq message is sent to the Discovery Engine
 * whenever the Upper layers would like to listen for a Service Name
 */
struct NanFWSubscribeRequest {
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

	/* Flag which specifies how the Subscribe request shall be processed. */
	enum NanSubscribeType subscribe_type; /* 0 - PASSIVE , 1- ACTIVE */

	/* Flag which specifies on Active Subscribes how the Service Response
	 * Filter attribute is populated.
	 */
	enum NanSRFType
		serviceResponseFilter; /* 0 - Bloom Filter, 1 - MAC Addr */

	/* Flag which specifies how the Service Response Filter
	 * Include bit is populated.
	 * 0=Do not respond if in the Address Set, 1= Respond
	 */
	enum NanSRFIncludeType serviceResponseInclude;

	/* Flag which specifies if the Service Response Filter should be used
	 * when creating Subscribes.
	 * 0=Do not send the Service Response Filter,1= send
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
	 */

	/* If this value is 0 this field is not used by the DE.*/
	uint8_t subscribe_count;

	/* UTF-8 encoded string identifying the service */
	uint8_t service_name[NAN_FW_MAX_SERVICE_NAME_LEN];
	uint16_t service_name_len; /* length of service name */

	/* Sequence of values which further specify the published
	 * service beyond the service name
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_FW_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Ordered sequence of <length, value> pairs used to filter out
	 * received publish discovery messages.
	 * This can be sent both for a Passive or an Active Subscribe
	 */
	uint16_t rx_match_filter_len;
	uint8_t rx_match_filter[NAN_FW_MAX_MATCH_FILTER_LEN];

	/* Ordered sequence of <length, value> pairs  included in the
	 * Discovery Frame when an Active Subscribe is used.
	 */
	uint16_t tx_match_filter_len;
	uint8_t tx_match_filter[NAN_FW_MAX_MATCH_FILTER_LEN];

	/* Flag which specifies that the Subscribe should use the configured
	 * RSSI threshold and the received RSSI in order to filter requests
	 * 0 - ignore the configured RSSI threshold when running a Service
	 * Descriptor attribute or Service ID List Attribute through the DE
	 * matching logic.
	 * 1 - use the configured RSSI threshold when running a Service
	 * Descriptor attribute or Service ID List Attribute through the DE
	 * matching logic.
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

	/* Set/Enable corresponding bits to disable indications that
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
	uint8_t scid[NAN_FW_MAX_SCID_BUF_LEN];

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
	uint8_t sdea_service_specific_info[NAN_FW_SDEA_SPECIFIC_INFO_LEN];

	uint8_t service_name_hash[NAN_SERVICE_HASH_LENGTH];
} __KAL_ATTRIB_PACKED__;

struct NanFWTransmitFollowupRequest {
	/* Publish or Subscribe Id of an earlier Publish/Subscribe */
	uint16_t publish_subscribe_id;

	/* This Id is the Requestor Instance that is passed as
	 * part of earlier MatchInd/FollowupInd message.
	 */
	uint32_t requestor_instance_id;
	uint8_t addr[NAN_MAC_ADDR_LEN]; /* Unicast address */
	/* NanTxPriority priority; */    /* priority of the request 2=high */
	enum NanTransmitWindowType
		dw_or_faw; /* 0= send in a DW, 1=send in FAW */

	/* Sequence of values which further specify the published service
	 * beyond the service name.
	 */
	uint16_t service_specific_info_len;
	uint8_t service_specific_info[NAN_FW_MAX_SERVICE_SPECIFIC_INFO_LEN];

	/* Set/Enable corresponding bits to disable responses after followUp.
	 * BIT0 - Disable followUp response from FW.
	 * u8 recv_indication_cfg;
	 */

	/* Sequence of values indicating the service specific info in SDEA */
	uint16_t sdea_service_specific_info_len;
	uint8_t sdea_service_specific_info[NAN_FW_SDEA_SPECIFIC_INFO_LEN];
} __KAL_ATTRIB_PACKED__;

struct service_specificy_info {
	uint8_t aucOUI[3];
	uint8_t ucServiceType;
	uint8_t aucServiceIno[32];
};

#define NAN_MAX_SCID_NUM 8
#define NAN_MAX_CIPHER_SUITE_NUM 2
#define NAN_INVALID_CIPHER_SUITE_ID 0

#define NAN_NUM_SERVICE_SESSION 30

struct _NAN_SERVICE_SESSION_T {
	struct LINK_ENTRY rLinkEntry;

	uint8_t aucPublishNmiAddr[MAC_ADDR_LEN];

	uint8_t ucSubscribeID;
	uint8_t ucPublishID;

	uint8_t ucNumCSID;
	uint8_t aucSupportedCipherSuite[NAN_MAX_CIPHER_SUITE_NUM];
	uint8_t ucNumSCID;
	uint8_t aaucSupportedSCID[NAN_MAX_SCID_NUM][NAN_SCID_DEFAULT_LEN];
};

struct _NAN_DISC_ENGINE_T {
	unsigned char fgInit;

	struct LINK rSeviceSessionList;
	struct LINK rFreeServiceSessionList;
	struct _NAN_SERVICE_SESSION_T
		arServiceSessionList[NAN_NUM_SERVICE_SESSION];
};

struct _NAN_DISC_CMD_ADD_CSID_T {
	uint8_t ucPubID;
	uint8_t ucNum;
	uint8_t aucSupportedCSID[NAN_MAX_CIPHER_SUITE_NUM];
} __KAL_ATTRIB_PACKED__;

struct _NAN_DISC_CMD_MANAGE_SCID_T {
	unsigned char fgAddDelete;
	uint8_t ucPubID;
	uint8_t aucSCID[NAN_SCID_DEFAULT_LEN];
} __KAL_ATTRIB_PACKED__;

uint32_t nanCancelPublishRequest(struct ADAPTER *prAdapter,
				 struct NanPublishCancelRequest *msg);

uint32_t nanUpdatePublishRequest(struct ADAPTER *prAdapter,
				 struct NanPublishRequest *msg);

uint32_t nanPublishRequest(struct ADAPTER *prAdapter,
			  struct NanPublishRequest *msg);

uint32_t nanCancelSubscribeRequest(struct ADAPTER *prAdapter,
				   struct NanSubscribeCancelRequest *msg);

uint32_t nanSubscribeRequest(struct ADAPTER *prAdapter,
			    struct NanSubscribeRequest *msg);

uint32_t nanTransmitRequest(struct ADAPTER *prAdapter,
			   struct NanTransmitFollowupRequest *msg);

void nanCmdManageScid(IN struct ADAPTER *prAdapter, unsigned char fgAddDelete,
		      uint8_t ucPubID, uint8_t *pucScid);
void nanCmdAddCsid(IN struct ADAPTER *prAdapter, uint8_t ucPubID,
		   uint8_t ucNumCsid, uint8_t *pucCsidList);

uint32_t nanDiscUpdateCipherSuiteInfoAttr(struct ADAPTER *prAdapter,
					  uint8_t *pcuEvtBuf);

uint32_t nanDiscUpdateSecContextInfoAttr(struct ADAPTER *prAdapter,
					 uint8_t *pcuEvtBuf);

uint32_t nanDiscInit(struct ADAPTER *prAdapter);
struct _NAN_SERVICE_SESSION_T *
nanDiscSearchServiceSession(struct ADAPTER *prAdapter,
			    uint8_t *pucPublishNmiAddr, uint8_t ucPubID);
struct _NAN_SERVICE_SESSION_T *
nanDiscAcquireServiceSession(struct ADAPTER *prAdapter,
			     uint8_t *pucPublishNmiAddr, uint8_t ucPubID);
#endif
#endif
