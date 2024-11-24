/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef _NAN_DATA_ENGINE_H
#define _NAN_DATA_ENGINE_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */
#if CFG_SUPPORT_NAN

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "nan_base.h"

extern struct net_device *gPrDev;

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/****************************************************
 *                    Common part
 ***************************************************
 */

/****************************************************
 *                    Local part
 ****************************************************
 */

#define NAN_NDL_IMMUTABLE_SCHEDULE_MAX_LEN (512)
#define NAN_MAX_SUPPORT_NDP_NUM 3
#define NAN_MAX_SUPPORT_NDP_CXT_NUM (NAN_MAX_SUPPORT_NDP_NUM + 1)
#define NAN_MAX_SUPPORT_NDL_NUM 8

#define NAN_PROTOCOL_TIMEOUT 10000 /*2000*/
#define NAN_SECURITY_TIMEOUT 1000
#define NAN_DATA_RETRY_TIMEOUT 3000 /*300*/
#define NAN_DATA_RETRY_LIMIT 5

/* Macros used by NAN Data Engine */
#define NAN_DATAREQ_REQUIRE_QOS_UNICAST BIT(0)
#define NAN_DATAREQ_REQUIRE_QOS_MULTICAST BIT(1)

#define NAN_DATA_TYPE_UNICAST BIT(0)

#define NAN_DATA_RESP_DECISION_ACCEPT 0
#define NAN_DATA_RESP_DECISION_REJECT 1

#define NAN_OUI                                                                \
	{ 0x50, 0x6f, 0x9a }

#define ACTION_PUBLIC_VENDOR_SPECIFIC 9

#define ENABLE_NDP_UT_LOG 1

#define NAN_CATEGORY_HDR_OFFSET 7

/****************************************************
 *                    Local part
 ****************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/* NAN_DATA_ENGINE_REQUEST is used to serve multiple NDL/NDP establishment */
enum _NAN_DATA_ENGINE_REQUEST_TYPE_T {
	NAN_DATA_ENGINE_REQUEST_NDL_SETUP,
	NAN_DATA_ENGINE_REQUEST_NDP_SETUP,
	NAN_DATA_ENGINE_REQUEST_NUM
};

struct _NAN_DATA_ENGINE_REQUEST_T {
	struct LINK_ENTRY rLinkEntry;

	enum _NAN_DATA_ENGINE_REQUEST_TYPE_T eRequestType;
	enum _ENUM_NAN_PROTOCOL_ROLE_T eNDLRole;
	struct _NAN_NDP_INSTANCE_T *prNDP;
};

/* all NDL TX_* states means the corersponding frame will be TX in the state,
 * then advance to NDL RX_* state for waiting incoming frame as soon as
 *  TX Done is confirmed
 */
enum _ENUM_NDL_MGMT_STATE_T {
	NDL_IDLE,		  /* 0 */
	NDL_REQUEST_SCHEDULE_NDP, /* 1 */
	NDL_REQUEST_SCHEDULE_NDL, /* 2 */
	/* 3 - NDL/NDP estalibshment through data request/response */
	NDL_SCHEDULE_SETUP,
	/* 4 - NDL establishment through schedule request/response */
	NDL_INITIATOR_TX_SCHEDULE_REQUEST,
	/* 5 - NDL establishment through schedule request/response */
	NDL_RESPONDER_RX_SCHEDULE_REQUEST,
	/* 6 - NDL establishment through schedule request/response */
	NDL_RESPONDER_TX_SCHEDULE_RESPONSE,
	/* 7 - NDL establishment through schedule request/response */
	NDL_INITIATOR_RX_SCHEDULE_RESPONSE,
	/* 8 - NDL establishment through schedule request/response */
	NDL_INITIATOR_TX_SCHEDULE_CONFIRM,
	/* 9 - NDL establishment through schedule request/response */
	NDL_RESPONDER_RX_SCHEDULE_CONFIRM,
	NDL_SCHEDULE_ESTABLISHED,		    /* 10 */
	NDL_TEARDOWN_BY_NDP_TERMINATION,	    /* 11 */
	NDL_TEARDOWN,				    /* 12 */
	NDL_INITIATOR_WAITFOR_RX_SCHEDULE_RESPONSE, /* 13 */
	NDL_MGMT_STATE_NUM
};

enum _ENUM_NDP_PROTOCOL_STATE_T {
	NDP_IDLE,		     /* 0 */
	/* 1 - INITIATOR: for sending Data Path Request */
	NDP_INITIATOR_TX_DP_REQUEST,
	/* 2 - INITIATOR: for receiving Data Path Response */
	NDP_INITIATOR_RX_DP_RESPONSE,
	/* 3 - INITIATOR: for sending Data Path Confirm (optional) */
	NDP_INITIATOR_TX_DP_CONFIRM,
	/* 4 - INITIATOR: for receiving Data Path Security Install (optional) */
	NDP_INITIATOR_RX_DP_SECURITY_INSTALL,
	/* 5 - RESPONDER: for waiting Data Path method (FW-CMD) */
	NDP_RESPONDER_WAIT_DATA_RSP,
	/* 6 - RESPONDER: for sending Data Path Response */
	NDP_RESPONDER_TX_DP_RESPONSE,
	/* 7 - RESPONDER: for waiting Data Path Confirm (optional) */
	NDP_RESPONDER_RX_DP_CONFIRM,
	/* 8 - RESPONDER: for sending Data Path Security Install (optional) */
	NDP_RESPONDER_TX_DP_SECURITY_INSTALL,
	/* 9 - Data path established */
	NDP_NORMAL_TR,
	/* 10 - for sending Data Path Termination */
	NDP_TX_DP_TERMINATION,
	/* 11 - release NDP, and inform NDL for releasing resource */
	NDP_DISCONNECT,
	NDP_PROTOCOL_STATE_NUM
};

enum _ENUM_NAN_NDP_STATUS_T {
	NAN_NDP_DISCONNECT = 0,
	NAN_NDP_CONNECTED,
	NAN_NDP_STATUS_NUM
};

enum _ENUM_DP_PROTOCOL_REASON_CODE_T {
	DP_REASON_SUCCESS,
	DP_REASON_RX_TIMEOUT,
	DP_REASON_SECURITY_TIMEOUT,
	DP_REASON_SECURITY_FAIL,
	DP_REASON_SCHEDULE_CONFLICT,
	DP_REASON_SCHEDULE_UPDATE_FAIL,
	DP_REASON_USER_SPACE_RESPONSE_TIMEOUT,
	DP_REASON_NUM
};

struct wpa_sm;
struct wpa_state_machine;

struct _NAN_NDP_INSTANCE_T;

struct _NAN_NDP_CONTEXT_T {
	unsigned char fgValid;
	uint8_t ucId;

	uint8_t aucLocalNDIAddr[MAC_ADDR_LEN];
	uint8_t aucPeerNDIAddr[MAC_ADDR_LEN];

	/* STA-REC management */
	struct STA_RECORD *prNanStaRec;

	uint8_t ucNumEnrollee;
	struct _NAN_NDP_INSTANCE_T *aprEnrollNdp[NAN_MAX_SUPPORT_NDP_NUM];
};

struct _NAN_NDP_INSTANCE_T {
	unsigned char fgNDPValid;

	uint8_t aucLocalNDIAddr[MAC_ADDR_LEN];
	uint8_t aucPeerNDIAddr[MAC_ADDR_LEN];

	uint8_t ucNDPID;
	uint8_t ucDialogToken; /* carried in NDP attribute */
	uint8_t ucTxNextTypeStatus;
	uint8_t ucTxRetryCounter;
	uint8_t ucRCPI;

	uint8_t ucNDPSetupStatus;
	uint8_t ucReasonCode;

	uint8_t ucPublishId;
	unsigned char fgNDPActive;
	unsigned char fgSecurityRequired;
	unsigned char fgQoSRequired;
	unsigned char fgConfirmRequired;
	unsigned char fgRejectPending;
	unsigned char fgSupportNDPE;
	unsigned char fgNDPEstablish;
	uint8_t ucCipherType;
	uint8_t aucPMK[NAN_PMK_INFO_LEN];

	/*Need security module data structure*/
	/*NanSecurityKeyInfo    rkey_info;*/

	uint8_t aucNanMulticastAddr[6];
	enum _ENUM_NAN_PROTOCOL_ROLE_T eNDPRole;
	enum _ENUM_NDP_PROTOCOL_STATE_T eCurrentNDPProtocolState;
	enum _ENUM_NDP_PROTOCOL_STATE_T eLastNDPProtocolState;
	enum _ENUM_DP_PROTOCOL_REASON_CODE_T eDataPathFailReason;

	uint8_t ucNdlIndex;

	/* buffered data from retrieved/generated NAN Action Frame */

	/* whole content of NAN_ATTR_SHARED_KEY_DESCRIPTOR_T */
	uint16_t u2KdeLen;

	/* pucKdeInfo is *not* a dynamically allocated buffer, */
	/* but pointing to somewhere in pucAttrList */
	uint8_t *pucKdeInfo;

	/* NAN ATTRs - for both TX & RX, dynamically allocated */
	uint16_t u2AttrListLength;
	/* TODO: timing of freeing - after security install*/
	uint8_t *pucAttrList;

	/* Workaround: save Rx SwRfb Category pointer */
	uint16_t u2RxMsgLen;
	uint8_t *pucRxMsgBuf;
	uint16_t u2RxSkdAttrOffset;

	uint16_t u2AppInfoLen;
	/* TODO: timing of freeing - after event indication */
	uint8_t *pucAppInfo;
	/* NAN R3 feature */
	uint8_t ucServiceProtocolType; /* NAN_SERVICE_PROTOCOL_TYPE_* */
	uint8_t ucProtocolType;
	uint16_t u2PortNum;

	/* IPv6 - NAN R3 feature */
	unsigned char fgCarryIPV6;
	unsigned char fgIsInitiator;
	uint8_t aucInterfaceId[8];
	uint8_t aucRspInterfaceId[8];

	uint8_t *pucServiceInfo;

	/* Non-WFA Service Information - NAN R3 feature */
	uint8_t aucOtherAppInfoOui[VENDOR_OUI_LEN]; /* others than NAN_OUI */
	uint16_t u2OtherAppInfoLen;
	uint8_t *pucOtherAppInfo;

	struct wpa_sm *prResponderSecSmInfo;
	struct wpa_state_machine *prInitiatorSecSmInfo;

	uint8_t au1Scid[NAN_SCID_DEFAULT_LEN];

	uint8_t u1RmtCipher;
	uint8_t u1RmtCipherPId;

	uint8_t au1RmtScid[NAN_SCID_DEFAULT_LEN];
	uint8_t u1RmtScidPId;

	struct TIMER rNDPUserSpaceResponseTimer;

	struct _NAN_NDP_CONTEXT_T *prContext;
};

struct _NAN_NDL_INSTANCE_T;

struct _NAN_DATA_ENGINE_SCHEDULE_TOKEN_T {
	struct _NAN_NDL_INSTANCE_T *prNDL;
	struct ADAPTER *prAdapter;
};

struct _NAN_NDL_INSTANCE_T {
	unsigned char fgNDLValid;
	uint8_t ucIndex;

	uint8_t aucPeerMacAddr[MAC_ADDR_LEN]; /* NMI */
	uint8_t ucNDPNum;
	struct _NAN_NDP_CONTEXT_T arNdpCxt[NAN_MAX_SUPPORT_NDP_CXT_NUM];
	struct _NAN_NDP_INSTANCE_T arNDP[NAN_MAX_SUPPORT_NDP_NUM];

	enum _ENUM_NDL_MGMT_STATE_T eCurrentNDLMgmtState;
	enum _ENUM_NDL_MGMT_STATE_T eLastNDLMgmtState;
	enum _ENUM_NAN_PROTOCOL_ROLE_T eNDLRole;

	/* for tracking NDP under negotiation */
	struct _NAN_NDP_INSTANCE_T *prOperatingNDP;

	uint8_t ucTxRetryCounter;

	struct TIMER rNDPProtocolExpireTimer;
	struct TIMER rNDPProtocolRetryTimer;
	struct TIMER rNDPSecurityExpireTimer;
	uint8_t ucNDLSetupCurrentStatus;
	uint8_t ucReasonCode;
	uint8_t ucSeqNum; /* used to assign NDP-ID */

	unsigned char fgScheduleEstablished;
	unsigned char fgScheduleUnderNegotiation;
	unsigned char fgRejectPending;
	unsigned char fgPagingRequired;
	unsigned char fgCarryImmutableSchedule;
	unsigned char fgIsCounter;

	/* ATTR_NDL parameters */
	uint8_t ucDialogToken;
	uint8_t ucPeerID;
	uint16_t u2MaximumIdlePeriod;

	/* ATTR_NDL_QOS parameters */
	uint8_t ucMinimumTimeSlot;

	uint16_t u2MaximumLatency;

	/* token for communicate with NAN scheduler */
	struct _NAN_DATA_ENGINE_SCHEDULE_TOKEN_T rToken;

	uint8_t ucPhyTypeSet;

	/* remote HT-CAP / VHT-CAP IEs */
	struct IE_HT_CAP rIeHtCap;
	struct IE_VHT_CAP rIeVhtCap;

	struct LINK rPendingReqList;
};

struct _NAN_DATA_PATH_INFO_T {

	struct _NAN_NDL_INSTANCE_T arNDL[NAN_MAX_SUPPORT_NDL_NUM];
	uint8_t ucNDLNum;

	uint8_t aucLocalNMIAddr[MAC_ADDR_LEN]; /* NMI */
	uint8_t ucSeqNum; /* used to assign event SeqNum Generation */

	unsigned char fgIsECSet;
	uint8_t aucECAttr[48];
	/* large enought to contain
	 * OFFSET_OF(struct _NAN_ATTR_ELEMENT_CONTAINER_T, aucElements) +
	 * ELEM_HDR_LEN + ELEM_MAX_LEN_HT_CAP +
	 * ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_CAP
	 */

	struct IE_HT_CAP *prLocalHtCap;
	struct IE_VHT_CAP *prLocalVhtCap;

	/* NET-DEV reference count */
	atomic_t NetDevRefCount[NAN_BSS_INDEX_NUM];

	unsigned char fgAutoHandleDPRequest;
	uint8_t ucDPResponseDecisionStatus;
	uint8_t aucRemoteAddr[6];

	/* NAN R3 feature */
	bool             fgCarryIPV6;
	uint8_t              aucIPv6Addr[IPV6MACLEN];
	uint16_t             u2AppInfoLen;
	/* TODO: timing of freeing - after event indication */
	uint8_t             *pucAppInfo;
	/* NAN_SERVICE_PROTOCOL_TYPE_* */
	uint8_t              ucServiceProtocolType;
	uint8_t              ucProtocolType;
	uint16_t             u2PortNum;
};

/* CMD and EVT for OID */
struct _NAN_CMD_DATA_REQUEST {
	uint8_t ucType; /* bit#0: unicast */
	uint8_t ucPublishID;
	uint8_t ucRequireQOS; /* bit#0: unicast, bit#1: multicast */
	uint8_t ucSecurity;   /* refer to NAN_CIPHER_SUITE_ID_XXX */

	uint8_t aucScid[NAN_SCID_DEFAULT_LEN];

	uint8_t aucResponderDataAddress[6];
	uint8_t aucMulticastAddress[6];

	uint8_t aucPMK[32];

	uint16_t u2SpecificInfoLength;
	uint8_t aucSpecificInfo[NAN_DP_MAX_APP_INFO_LEN]; /* may be longer */
	uint8_t fgCarryIpv6;
	uint8_t aucIPv6Addr[IPV6MACLEN];
	uint8_t ucMinTimeSlot;
	uint16_t u2MaxLatency;
	bool     fgNDPE;
};

struct _NAN_CMD_DATA_RESPONSE {
	uint8_t ucType;		 /* bit#0: unicast */
	uint8_t ucDecisionStatus; /* NAN_DATA_RESP_DECISION_* */
	uint8_t ucReasonCode;     /* refer to NAN_REASON_CODE_* */
	uint8_t ucNDPId;

	uint8_t aucInitiatorDataAddress[6];
	uint8_t aucMulticastAddress[6];

	uint8_t ucRequireQOS; /* bit#0: unicast, bit#1: multicast */

	uint8_t ucSecurity; /* refer to NAN_CIPHER_SUITE_ID_* */
	uint8_t aucReserved[2];

	uint8_t aucPMK[32];

	uint16_t u2SpecificInfoLength;
	uint8_t aucSpecificInfo[NAN_DP_MAX_APP_INFO_LEN]; /* may be longer */
	uint8_t fgCarryIpv6;
	uint8_t aucIPv6Addr[IPV6MACLEN];
	uint16_t u2PortNum;
	uint8_t ucServiceProtocolType;
	uint8_t ucMinTimeSlot;
	uint16_t u2MaxLatency;
};

struct _NAN_CMD_DATA_END {
	uint8_t ucType;       /* bit#0: unicast */
	uint8_t ucReasonCode; /* refer to NAN_REASON_CODE_* */
	uint8_t ucNDPId;
	uint8_t ucNMSGId;

	uint8_t aucInitiatorDataAddress[6];
	uint8_t aucReserved[2];
};

struct _NAN_PARAMETER_NDL_SCH {
	uint8_t ucType; /* bit#0: unicast */
	uint8_t ucNDPId;
	uint8_t ucNMSGId; /* bit#0: unicast, bit#1: multicast */
	uint8_t ucRequireQOS;
	uint8_t ucMinTimeSlot;
	uint16_t u2MaxLatency;

	uint8_t aucPeerDataAddress[6];
	uint8_t aucReserved2[2];
};

/* attribute filling functions */
struct _APPEND_ATTR_ENTRY_T {
	/* refer to Table.33 in NAN Tech.Spec v2.0 */
	uint8_t u8AttrID;

	/* NULL stands for fixed length */
	uint16_t(*pfnCalculateVariableAttrLen)
	(struct ADAPTER *, struct _NAN_NDL_INSTANCE_T *,
	 struct _NAN_NDP_INSTANCE_T *);

	/* attribute appending */
	void(*pfnAppendAttr)
	(struct ADAPTER *, struct MSDU_INFO *, struct _NAN_NDL_INSTANCE_T *,
	 struct _NAN_NDP_INSTANCE_T *);
};

/* attribute filling functions */
struct _APPEND_SUB_ATTR_ENTRY_T {
	/* refer to Table.33 in NAN Tech.Spec v2.0 */
	uint8_t u8AttrID;

	/* NULL stands for fixed length */
	uint16_t(*pfnCalculateVariableAttrLen)
	(struct ADAPTER *, struct _NAN_NDL_INSTANCE_T *,
	 struct _NAN_NDP_INSTANCE_T *);

	/* attribute appending */
	void(*pfnAppendAttr)
	(struct ADAPTER *, uint8_t *, struct _NAN_NDL_INSTANCE_T *,
	 struct _NAN_NDP_INSTANCE_T *);
};

struct _NAN_SCHED_EVENT_NDL_DISCONN_T {
	uint8_t ucReason;
	uint8_t ucStaIdx;
};

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
#define NAN_ATTR_NDP(fp) ((struct _NAN_ATTR_NDP_T *)fp)
#define NAN_ATTR_NDL(fp) ((struct _NAN_ATTR_NDL_T *)fp)

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

void nanDataEngineInit(IN struct ADAPTER *prAdapter, IN uint8_t *pu1NMIAddress);

void nanDataEngineUninit(IN struct ADAPTER *prAdapter);

void nanSetNdpPmkid(
	IN struct ADAPTER *prAdapter,
	IN struct _NAN_CMD_DATA_REQUEST *prNanCmdDataRequest,
	IN uint8_t	*puServiceName
);

/* Command Handlers */
uint32_t nanCmdDataRequest(IN struct ADAPTER *prAdapter,
			   IN struct _NAN_CMD_DATA_REQUEST *prNanCmdDataRequest,
			   OUT uint8_t *pu1NdpId,
			   OUT uint8_t *au1InitiatorDataAddr);

uint32_t
nanCmdDataResponse(IN struct ADAPTER *prAdapter,
		   struct _NAN_CMD_DATA_RESPONSE *prNanCmdDataResponse);

uint32_t nanCmdDataEnd(IN struct ADAPTER *prAdapter,
		       struct _NAN_CMD_DATA_END *prNanCmdDataEnd);

uint32_t nanUpdateNdlSchedule(IN struct ADAPTER *prAdapter,
			      struct _NAN_PARAMETER_NDL_SCH *prNanparamUDSCH);

uint32_t nanCmdDataUpdtae(IN struct ADAPTER *prAdapter,
			  struct _NAN_PARAMETER_NDL_SCH *prNanUpdateSchParam);

/* Incoming NAN Action Frame Handlers */
uint32_t nanNdpProcessDataRequest(IN struct ADAPTER *prAdapter,
				  IN struct SW_RFB *prSwRfb);

uint32_t nanNdpProcessDataResponse(IN struct ADAPTER *prAdapter,
				   IN struct SW_RFB *prSwRfb);

uint32_t nanNdpProcessDataConfirm(IN struct ADAPTER *prAdapter,
				  IN struct SW_RFB *prSwRfb);

uint32_t nanNdpProcessDataKeyInstall(IN struct ADAPTER *prAdapter,
				     IN struct SW_RFB *prSwRfb);

uint32_t nanNdpProcessDataTermination(IN struct ADAPTER *prAdapter,
				      IN struct SW_RFB *prSwRfb);

uint32_t nanNdlProcessScheduleRequest(IN struct ADAPTER *prAdapter,
				      IN struct SW_RFB *prSwRfb);

uint32_t nanNdlProcessScheduleResponse(IN struct ADAPTER *prAdapter,
				       IN struct SW_RFB *prSwRfb);

uint32_t nanNdlProcessScheduleConfirm(IN struct ADAPTER *prAdapter,
				      IN struct SW_RFB *prSwRfb);

uint32_t nanNdlProcessScheduleUpdateNotification(IN struct ADAPTER *prAdapter,
						 IN struct SW_RFB *prSwRfb);

/* NDL schedule update handler through NDL attribute carried
 * by NAN beacon & other NAFs
 */
uint32_t nanNdlProcessNdlAttribute(IN struct ADAPTER *prAdapter,
				   IN struct _NAN_ATTR_NDL_T *prNdlAttr);

/* Host Event Indication APIs - comply with NAN specification */
void nanNdpSendDataIndicationEvent(IN struct ADAPTER *prAdapter,
				   struct _NAN_NDP_INSTANCE_T *prNDP);

void nanNdpSendDataConfirmEvent(IN struct ADAPTER *prAdapter,
				struct _NAN_NDP_INSTANCE_T *prNDP);

void nanNdpSendDataTerminationEvent(IN struct ADAPTER *prAdapter,
				    struct _NAN_NDP_INSTANCE_T *prNDP);

/* Outgoing Frame Generation */
uint32_t nanNdpSendDataPathRequest(IN struct ADAPTER *prAdapter,
				   struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanNdpSendDataPathResponse(
	IN struct ADAPTER *prAdapter, struct _NAN_NDP_INSTANCE_T *prNDP,

	/* below are optional params, only valid when prNDP == NULL */
	IN uint8_t *pucDestMacAddr, IN struct _NAN_ATTR_NDP_T *prPeerAttrNDP,
	IN uint8_t ucNDPReasonCode, IN struct _NAN_ATTR_NDPE_T *prPeerAttrNDPE,
	IN uint8_t ucNDPEReasonCode, IN struct _NAN_ATTR_NDL_T *prPeerAttrNDL,
	IN uint8_t ucNDLReasonCode);

uint32_t nanNdpSendDataPathConfirm(IN struct ADAPTER *prAdapter,
				   struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanNdpSendDataPathKeyInstall(IN struct ADAPTER *prAdapter,
				      struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanNdpSendDataPathTermination(IN struct ADAPTER *prAdapter,
				       struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanNdlSendScheduleRequest(IN struct ADAPTER *prAdapter,
				   struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanNdlSendScheduleResponse(
	IN struct ADAPTER *prAdapter, IN struct _NAN_NDL_INSTANCE_T *prNDL,

	/* below are optional params, only valid when prNDL == NULL */
	IN uint8_t *pucDestMacAddr, IN struct _NAN_ATTR_NDL_T *prPeerAttrNDL,
	IN uint8_t ucReasonCode);

uint32_t nanNdlSendScheduleConfirm(IN struct ADAPTER *prAdapter,
				   struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanNdlSendScheduleUpdateNotify(IN struct ADAPTER *prAdapter,
					struct _NAN_NDL_INSTANCE_T *prNDL);

/* NAF TX Done Callbacks */
uint32_t nanDPReqTxDone(IN struct ADAPTER *prAdapter,
			IN struct MSDU_INFO *prMsduInfo,
			IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanDPRespTxDone(IN struct ADAPTER *prAdapter,
			 IN struct MSDU_INFO *prMsduInfo,
			 IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanDPConfirmTxDone(IN struct ADAPTER *prAdapter,
			    IN struct MSDU_INFO *prMsduInfo,
			    IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanDPSecurityInstallTxDone(IN struct ADAPTER *prAdapter,
				    IN struct MSDU_INFO *prMsduInfo,
				    IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanDPTerminationTxDone(IN struct ADAPTER *prAdapter,
				IN struct MSDU_INFO *prMsduInfo,
				IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t
nanDataEngineScheduleReqTxDone(IN struct ADAPTER *prAdapter,
			       IN struct MSDU_INFO *prMsduInfo,
			       IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t
nanDataEngineScheduleRespTxDone(IN struct ADAPTER *prAdapter,
				IN struct MSDU_INFO *prMsduInfo,
				IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t
nanDataEngineScheduleConfirmTxDone(IN struct ADAPTER *prAdapter,
				   IN struct MSDU_INFO *prMsduInfo,
				   IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanDataEngineScheduleUpdateNotificationTxDone(
	IN struct ADAPTER *prAdapter, IN struct MSDU_INFO *prMsduInfo,
	IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

/* TX wrapper function for NAN Data Engine */
uint32_t nanDataEngineSendNAF(IN struct ADAPTER *prAdapter,
			      IN struct MSDU_INFO *prMsduInfo,
			      IN uint16_t u2FrameLength,
			      IN PFN_TX_DONE_HANDLER pfTxDoneHandler,
			      IN struct STA_RECORD *prStaRec);

/* parsers for NAN_ATTR list */
uint32_t nanNdpParseAttributes(IN struct ADAPTER *prAdapter,
			       enum _NAN_ACTION_T eNanAction,
			       uint8_t *pucNanAttrList,
			       uint16_t u2NanAttrListLength,
			       struct _NAN_NDL_INSTANCE_T *prNDL,
			       struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanNdlParseAttributes(IN struct ADAPTER *prAdapter,
			       enum _NAN_ACTION_T eNanAction,
			       uint8_t *pucNanAttrList,
			       uint16_t u2NanAttrListLength,
			       struct _NAN_NDL_INSTANCE_T *prNDL);

/* separate NAN_ATTR handlers */
uint32_t nanNdpAttrUpdateNdp(IN struct ADAPTER *prAdapter,
			     enum _NAN_ACTION_T eNanAction,
			     struct _NAN_ATTR_NDP_T *prAttrNDP,
			     struct _NAN_NDL_INSTANCE_T *prNDL,
			     struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanNdpeAttrUpdateNdp(IN struct ADAPTER *prAdapter,
			      enum _NAN_ACTION_T eNanAction,
			      struct _NAN_ATTR_NDPE_T *prAttrNDPE,
			      struct _NAN_NDL_INSTANCE_T *prNDL,
			      struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanNdlAttrUpdateNdl(IN struct ADAPTER *prAdapter,
			     enum _NAN_ACTION_T eNanAction,
			     struct _NAN_ATTR_NDL_T *prAttrNDL,
			     struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanNdlQosAttrUpdateNdl(IN struct ADAPTER *prAdapter,
				enum _NAN_ACTION_T eNanAction,
				struct _NAN_ATTR_NDL_QOS_T *prNdlQosAttr,
				struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanDeviceCapabilityAttrHandler(
	IN struct ADAPTER *prAdapter, enum _NAN_ACTION_T eNanAction,
	struct _NAN_ATTR_DEVICE_CAPABILITY_T *prDeviceCapabilityAttr,
	struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanAvailabilityAttrHandler(
	IN struct ADAPTER *prAdapter, enum _NAN_ACTION_T eNanAction,
	struct _NAN_ATTR_NAN_AVAILABILITY_T *prAvailabilityAttr,
	struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanNDCAttrHandler(IN struct ADAPTER *prAdapter,
			   enum _NAN_ACTION_T eNanAction,
			   struct _NAN_ATTR_NDC_T *prNDCAttr,
			   struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanElemContainerAttrHandler(
	IN struct ADAPTER *prAdapter, enum _NAN_ACTION_T eNanAction,
	struct _NAN_ATTR_ELEMENT_CONTAINER_T *prElemContainerAttr,
	struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t
nanUnalignedAttrHandler(IN struct ADAPTER *prAdapter,
			enum _NAN_ACTION_T eNanAction,
			struct _NAN_ATTR_UNALIGNED_SCHEDULE_T *prUnalignedAttr,
			struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanCipherSuiteAttrHandler(
	IN struct ADAPTER *prAdapter, enum _NAN_ACTION_T eNanAction,
	struct _NAN_ATTR_CIPHER_SUITE_INFO_T *prCipherSuiteAttr,
	struct _NAN_NDL_INSTANCE_T *prNDL, struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanSecContextAttrHandler(
	IN struct ADAPTER *prAdapter, enum _NAN_ACTION_T eNanAction,
	struct _NAN_ATTR_SECURITY_CONTEXT_INFO_T *prSecContextAttr,
	struct _NAN_NDL_INSTANCE_T *prNDL, struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanSharedKeyAttrHandler(
	IN struct ADAPTER *prAdapter, enum _NAN_ACTION_T eNanAction,
	struct _NAN_ATTR_SHARED_KEY_DESCRIPTOR_T *prAttrSharedKeyDescriptor,
	struct _NAN_NDL_INSTANCE_T *prNDL, struct _NAN_NDP_INSTANCE_T *prNDP);

/* NDP update utility functions*/
void nanDataUpdateNdpPeerNDI(IN struct ADAPTER *prAdapter,
			     struct _NAN_NDP_INSTANCE_T *prNDP,
			     IN uint8_t *pucPeerNDIAddr);

void nanDataUpdateNdpLocalNDI(IN struct ADAPTER *prAdapter,
			      struct _NAN_NDP_INSTANCE_T *prNDP);

/* utility function */
struct _NAN_ATTR_HDR_T *nanRetrieveAttrById(uint8_t *pucAttrLists,
					    uint16_t u2Length,
					    uint8_t ucTargetAttrId);

/* TX frame composing */
uint32_t nanDataEngineComposeNAFHeader(IN struct ADAPTER *prAdapter,
				       struct MSDU_INFO *prMsduInfo,
				       enum _NAN_ACTION_T eAction,
				       uint8_t *pucLocalMacAddr,
				       uint8_t *pucPeerMacAddr);

/* functions for attribute generation */
uint16_t
nanDataEngineNDPAttrLength(IN struct ADAPTER *prAdapter,
			   struct _NAN_NDL_INSTANCE_T *prNDL,
			   struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNDPAttrAppend(IN struct ADAPTER *prAdapter,
				struct MSDU_INFO *prMsduInfo,
				struct _NAN_NDL_INSTANCE_T *prNDL,
				struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNDPAttrAppendImpl(
	IN struct ADAPTER *prAdapter, struct MSDU_INFO *prMsduInfo,
	IN struct _NAN_NDL_INSTANCE_T *prNDL,
	IN struct _NAN_NDP_INSTANCE_T *prNDP,
	/* below are optional params, only valid when prNDP == NULL */
	IN struct _NAN_ATTR_NDP_T *prPeerAttrNDP, IN uint8_t ucTypeStatus,
	IN uint8_t ucReasonCode);

uint16_t
nanDataEngineNDLAttrLength(IN struct ADAPTER *prAdapter,
			   struct _NAN_NDL_INSTANCE_T *prNDL,
			   struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNDLAttrAppend(IN struct ADAPTER *prAdapter,
				struct MSDU_INFO *prMsduInfo,
				struct _NAN_NDL_INSTANCE_T *prNDL,
				struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNDLAttrAppendImpl(
	IN struct ADAPTER *prAdapter, struct MSDU_INFO *prMsduInfo,
	IN struct _NAN_NDL_INSTANCE_T *prNDL,
	/* below are optional params, only valid when prNDL == NULL */
	IN struct _NAN_ATTR_NDL_T *prPeerAttrNDL, IN uint8_t ucTypeStatus,
	IN uint8_t ucReasonCode);

uint16_t
nanDataEngineElemContainerAttrLength(IN struct ADAPTER *prAdapter,
				     struct _NAN_NDL_INSTANCE_T *prNDL,
				     struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineElemContainerAttrAppend(IN struct ADAPTER *prAdapter,
					  struct MSDU_INFO *prMsduInfo,
					  struct _NAN_NDL_INSTANCE_T *prNDL,
					  struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineDevCapAttrLength(IN struct ADAPTER *prAdapter,
			      struct _NAN_NDL_INSTANCE_T *prNDL,
			      struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineDevCapAttrAppend(IN struct ADAPTER *prAdapter,
				   struct MSDU_INFO *prMsduInfo,
				   struct _NAN_NDL_INSTANCE_T *prNDL,
				   struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineNanAvailAttrLength(IN struct ADAPTER *prAdapter,
				struct _NAN_NDL_INSTANCE_T *prNDL,
				struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNanAvailAttrAppend(IN struct ADAPTER *prAdapter,
				     struct MSDU_INFO *prMsduInfo,
				     struct _NAN_NDL_INSTANCE_T *prNDL,
				     struct _NAN_NDP_INSTANCE_T *prNDP);

unsigned char
nanDataEngineNDPECheck(struct ADAPTER *prAdapter, unsigned char fgPeerNDPE);

uint16_t
nanDataEngineNdcAttrLength(IN struct ADAPTER *prAdapter,
			   struct _NAN_NDL_INSTANCE_T *prNDL,
			   struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNdcAttrAppend(IN struct ADAPTER *prAdapter,
				struct MSDU_INFO *prMsduInfo,
				struct _NAN_NDL_INSTANCE_T *prNDL,
				struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineNdlQosAttrLength(IN struct ADAPTER *prAdapter,
			      struct _NAN_NDL_INSTANCE_T *prNDL,
			      struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNdlQosAttrAppend(IN struct ADAPTER *prAdapter,
				   struct MSDU_INFO *prMsduInfo,
				   struct _NAN_NDL_INSTANCE_T *prNDL,
				   struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineUnalignedAttrLength(IN struct ADAPTER *prAdapter,
				 struct _NAN_NDL_INSTANCE_T *prNDL,
				 struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineUnalignedAttrAppend(IN struct ADAPTER *prAdapter,
				      struct MSDU_INFO *prMsduInfo,
				      struct _NAN_NDL_INSTANCE_T *prNDL,
				      struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineCipherSuiteAttrLength(IN struct ADAPTER *prAdapter,
				   struct _NAN_NDL_INSTANCE_T *prNDL,
				   struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineCipherSuiteAttrAppend(IN struct ADAPTER *prAdapter,
					struct MSDU_INFO *prMsduInfo,
					struct _NAN_NDL_INSTANCE_T *prNDL,
					struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineSecContextAttrLength(IN struct ADAPTER *prAdapter,
				  struct _NAN_NDL_INSTANCE_T *prNDL,
				  struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineSecContextAttrAppend(IN struct ADAPTER *prAdapter,
				       struct MSDU_INFO *prMsduInfo,
				       struct _NAN_NDL_INSTANCE_T *prNDL,
				       struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineSharedKeyAttrLength(IN struct ADAPTER *prAdapter,
				 struct _NAN_NDL_INSTANCE_T *prNDL,
				 struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineSharedKeyAttrAppend(IN struct ADAPTER *prAdapter,
				      struct MSDU_INFO *prMsduInfo,
				      struct _NAN_NDL_INSTANCE_T *prNDL,
				      struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineNDPEAttrLength(IN struct ADAPTER *prAdapter,
			    struct _NAN_NDL_INSTANCE_T *prNDL,
			    struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNDPEAttrAppend(IN struct ADAPTER *prAdapter,
				 struct MSDU_INFO *prMsduInfo,
				 struct _NAN_NDL_INSTANCE_T *prNDL,
				 struct _NAN_NDP_INSTANCE_T *prNDP);

void nanDataEngineNDPEAttrAppendImpl(
	IN struct ADAPTER *prAdapter, struct MSDU_INFO *prMsduInfo,
	IN struct _NAN_NDL_INSTANCE_T *prNDL,
	IN struct _NAN_NDP_INSTANCE_T *prNDP,
	/* below are optional params, only valid when prNDP == NULL */
	IN struct _NAN_ATTR_NDPE_T *prPeerAttrNDPE, IN uint8_t ucTypeStatus,
	IN uint8_t ucReasonCode);

/* NDP 's ucTxNextTypeStatus Generation */
uint32_t nanNdpUpdateTypeStatus(IN struct ADAPTER *prAdapter,
				IN struct _NAN_NDP_INSTANCE_T *prNDP);

/* NDP DialogToken Generation */
uint32_t nanNdpGenerateDialogToken(IN struct ADAPTER *prAdapter,
				   IN struct _NAN_NDP_INSTANCE_T *prNDP);

/* NDL DialogToken Generation */
uint32_t nanNdlGenerateDialogToken(IN struct ADAPTER *prAdapter,
				   IN struct _NAN_NDL_INSTANCE_T *prNDL);
uint32_t nanDataEngineUpdateSSI(IN struct ADAPTER *prAdapter,
			IN struct _NAN_NDP_INSTANCE_T *prNDP,
			IN uint8_t ucServiceProtocolType,
			IN uint16_t u2ContextLen, IN uint8_t *pucContext);

/* utility functino for APP-Info buffer/update */
uint32_t nanDataEngineUpdateAppInfo(IN struct ADAPTER *prAdapter,
			IN struct _NAN_NDP_INSTANCE_T *prNDP,
			IN uint8_t ucServiceProtocolType,
			IN uint16_t u2AppInfoLen,
			IN uint8_t *pucAppInfo);

uint32_t nanDataEngineUpdateOtherAppInfo(
	IN struct ADAPTER *prAdapter, IN struct _NAN_NDP_INSTANCE_T *prNDP,
	IN struct _NAN_ATTR_NDPE_SVC_INFO_TLV_T *prAppInfoTLV);

uint32_t nanNdlDeactivateTimers(IN struct ADAPTER *prAdapter,
				IN struct _NAN_NDL_INSTANCE_T *prNDL);

/* Exported API for Element Container Generation */
uint32_t nanDataEngineGetECAttr(struct ADAPTER *prAdapter, uint8_t **ppucECAttr,
				uint16_t *puECAttrLength);

uint32_t nanDataEngineGetECAttrImpl(struct ADAPTER *prAdapter,
				    uint8_t **ppucECAttr,
				    uint16_t *puECAttrLength,
				    struct BSS_INFO *prBssInfo,
				    struct _NAN_NDL_INSTANCE_T *prNDL);

/* Pending Request Management Functions */
uint32_t
nanDataEngineInsertRequest(IN struct ADAPTER *prAdapter,
			   IN struct _NAN_NDL_INSTANCE_T *prNDL,
			   IN enum _NAN_DATA_ENGINE_REQUEST_TYPE_T eRequestType,
			   IN enum _ENUM_NAN_PROTOCOL_ROLE_T eNDLRole,
			   IN struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanDataEngineFlushRequest(IN struct ADAPTER *prAdapter,
				   IN struct _NAN_NDL_INSTANCE_T *prNDL);

struct _NAN_DATA_ENGINE_REQUEST_T *
nanDataEngineGetNextRequest(IN struct ADAPTER *prAdapter,
			    IN struct _NAN_NDL_INSTANCE_T *prNDL);

uint32_t nanDataEngineRemovePendingRequests(
	IN struct ADAPTER *prAdapter, IN struct _NAN_NDL_INSTANCE_T *prNDL,
	IN enum _NAN_DATA_ENGINE_REQUEST_TYPE_T eRequestType,
	IN struct _NAN_NDP_INSTANCE_T *prNDP);

uint16_t
nanDataEngineNDPESpecAttrLength(IN struct ADAPTER *prAdapter,
				struct _NAN_NDL_INSTANCE_T *prNDL,
				struct _NAN_NDP_INSTANCE_T *prNDP);
void nanDataEngineNDPESpecAttrAppend(IN struct ADAPTER *prAdapter,
				     uint8_t *pucOffset,
				     struct _NAN_NDL_INSTANCE_T *prNDL,
				     struct _NAN_NDP_INSTANCE_T *prNDP);
uint16_t
nanDataEngineNDPEProtocolAttrLength(IN struct ADAPTER *prAdapter,
				    struct _NAN_NDL_INSTANCE_T *prNDL,
				    struct _NAN_NDP_INSTANCE_T *prNDP);
void nanDataEngineNDPEProtocolAttrAppend(IN struct ADAPTER *prAdapter,
					 uint8_t *pucOffset,
					 struct _NAN_NDL_INSTANCE_T *prNDL,
					 struct _NAN_NDP_INSTANCE_T *prNDP);
uint16_t
nanDataEngineNDPEPORTAttrLength(IN struct ADAPTER *prAdapter,
				struct _NAN_NDL_INSTANCE_T *prNDL,
				struct _NAN_NDP_INSTANCE_T *prNDP);
void nanDataEngineNDPEPORTAttrAppend(IN struct ADAPTER *prAdapter,
				     uint8_t *pucOffset,
				     struct _NAN_NDL_INSTANCE_T *prNDL,
				     struct _NAN_NDP_INSTANCE_T *prNDP);
void nanDataEngineDisconnectByStaIdx(IN struct ADAPTER *prAdapter,
				     uint8_t ucStaIdx);
void nanDataEngingDisconnectEvt(IN struct ADAPTER *prAdapter,
				IN uint8_t *pcuEvtBuf);

uint32_t nanDataEngineEnrollNDPContext(IN struct ADAPTER *prAdapter,
				       IN struct _NAN_NDL_INSTANCE_T *prNDL,
				       IN struct _NAN_NDP_INSTANCE_T *prNDP);

uint32_t nanDataEngineUnrollNDPContext(IN struct ADAPTER *prAdapter,
				       IN struct _NAN_NDL_INSTANCE_T *prNDL,
				       IN struct _NAN_NDP_INSTANCE_T *prNDP);

struct STA_RECORD *
nanDataEngineSearchNDPContext(IN struct ADAPTER *prAdapter,
			      IN struct _NAN_NDL_INSTANCE_T *prNDL,
			      IN uint8_t *pucLocalAddr,
			      IN uint8_t *pucPeerAddr);

struct STA_RECORD *nanGetStaRecByNDI(struct ADAPTER *prAdapter,
				     uint8_t *pucPeerMacAddr);

struct _NAN_NDL_INSTANCE_T *
nanDataUtilSearchNdlByMac(struct ADAPTER *prAdapter, uint8_t *pucAddr);

#endif
#endif
