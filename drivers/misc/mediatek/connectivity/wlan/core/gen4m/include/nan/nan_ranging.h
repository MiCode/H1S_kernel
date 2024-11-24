/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef _NAN_RANGING_H_
#define _NAN_RANGING_H_

#if CFG_SUPPORT_NAN

#include "src/utils/list.h"

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

#define NAN_MAX_SUPPORT_RANGING_NUM 8

#define NAN_FTM_REPORT_OK_MAX_NUM 1
#define NAN_FTM_REPORT_NG_MAX_NUM 1

#define FTM_FORMAT_BW_HT_MIXED_BW20 9
#define FTM_FORMAT_BW_VHT_BW20 10
#define FTM_FORMAT_BW_HT_MIXED_BW40 11
#define FTM_FORMAT_BW_VHT_BW40 12
#define FTM_FORMAT_BW_VHT_BW80 13

#define NAN_RANGING_SESSION_TIMEOUT 120000

enum _ENUM_RTT_HANDSHAKE_T {
	RTT_AP_RSV = 0,
	RTT_AP_SUCCESS,
	RTT_AP_FAIL1,
	RTT_AP_FAIL2
};

enum _NAN_RANGING_INVOKER {
	NAN_RANGING_DISCOVERY = 1,
	NAN_RANGING_APPLICATION
};

enum _ENUM_RANGING_STATE_T {
	RANGING_STATE_IDLE = 0,
	RANGING_STATE_INIT,
	RANGING_STATE_SCHEDULE,
	RANGING_STATE_REQUEST,
	RANGING_STATE_REQUEST_IND,
	RANGING_STATE_RESPONSE,
	RANGING_STATE_ACTIVE,
	RANGING_STATE_REPORT,
	RANGING_STATE_TERMINATE,
	RANGING_STATE_NUM
};

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

struct _NAN_FTM_PARAM_T {
	unsigned char fgRttTrigger;
	unsigned char ucAPStatus;
	uint8_t ucFTMNum;	  /* FTM Numer */
	uint8_t ucMinDeltaIn100US; /* Each FTM packet duration */
	uint8_t ucFTMBandwidth;
	/* Depend on what you want by peer packet rate(decode) */
	unsigned char fgASAP;
	uint16_t u2TSFTimer;
	uint8_t uc2BurstTimeout;
	uint8_t ucBurstExponent;
	unsigned char fgASAP_CAP;
};

struct _FTM_REPORT_RANGE_ENTRY_T {
	uint32_t u4StartTime;
	uint8_t aucBssId[MAC_ADDR_LEN];
	uint32_t u4Range : 24;
	uint8_t ucMaxRangeErrorExp;
	uint8_t ucReserved;
};

struct _FTM_REPORT_ERROR_ENTRY_T {
	uint32_t u4StartTime;
	uint8_t aucBssId[MAC_ADDR_LEN];
	uint8_t ucErrorCode;
};

struct _NAN_FTM_REPORT_T {
	uint8_t ucRangeEntryCnt;
	uint8_t ucErrorEntryCnt;
	struct _FTM_REPORT_RANGE_ENTRY_T
		arRangeEntry[NAN_FTM_REPORT_OK_MAX_NUM];
	struct _FTM_REPORT_ERROR_ENTRY_T
		arErrorEntry[NAN_FTM_REPORT_NG_MAX_NUM];
};

struct _NAN_RANGING_CTRL_T {
	/* Ranging management */
	uint16_t u2RangingId;
	uint8_t aucPeerAddr[MAC_ADDR_LEN];
	uint8_t ucRole;
	enum _NAN_RANGING_INVOKER ucInvoker;
	enum _ENUM_RANGING_STATE_T eCurrentState;
	uint32_t range_measurement_cm;
	unsigned char bSchedPass;

	/* Timer to check for range update */
	struct TIMER rRangingSessionTimer;

	/* Ranging Information attribute */
	uint8_t location_info_availability;
	uint16_t location_last_indication;

	/* Ranging Setup attribute */
	uint8_t dialog_token;
	uint8_t TypeStatus;
	uint8_t ReasonCode;
	uint8_t RangingControl;

	/* Geofencing */
	unsigned char bCurInside;
	unsigned char bPreInside;
	unsigned char bCurOutside;
	unsigned char bPreOutside;

	/* from Application */
	struct NanRangingCfg ranging_cfg;
	struct NanRangeResponseCtl response_ctl;

	/* for FTM session */
	struct _NAN_FTM_PARAM_T rNanFtmParam;
	struct _NAN_FTM_REPORT_T rNanFtmReport;
};

struct _NAN_RANGING_INSTANCE_T {
	struct dl_list list;
	struct _NAN_RANGING_CTRL_T ranging_ctrl;
};

struct _NAN_RANGING_INFO_T {
	struct dl_list ranging_list;
	uint16_t u2RangingCnt;
	uint8_t ucSeqNum; /* used to assign ranging ID */

	/* Default Value */
	struct NanRangingCfg ranging_cfg_def;
	struct NanRangeResponseCtl response_ctl_def;
};

struct _NAN_FTM_PARAM_CMD {
	uint8_t ucRole;
	uint8_t ucInvoker;
	uint8_t aucPeerAddr[MAC_ADDR_LEN];
	struct _NAN_FTM_PARAM_T rNanFtmParam;
} __KAL_ATTRIB_PACKED__;

struct _NAN_FTM_DONE_EVENT {
	uint8_t aucPeerAddr[MAC_ADDR_LEN];
	struct _NAN_FTM_REPORT_T rNanFtmReport;
} __KAL_ATTRIB_PACKED__;

struct _NAN_RANGING_BY_DISC_EVENT {
	struct NanRangeRequest rReq;
} __KAL_ATTRIB_PACKED__;

struct _NAN_RANGING_REPORT_CMD {
	uint32_t ucStatus;
	uint16_t ranging_id;
	uint8_t range_req_intf_addr[MAC_ADDR_LEN];
	uint32_t range_measurement_cm;
	uint32_t ranging_event_type;
} __KAL_ATTRIB_PACKED__;

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

#define FTM_FMT_TO_RANGE_CM(_ftm_fmt) (((_ftm_fmt & BITS(0, 23)) * 100) >> 12)

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/************************************************
 *   Initialization and Uninitialization
 ************************************************
 */

void nanRangingEngineInit(struct ADAPTER *prAdapter);

void nanRangingEngineUninit(struct ADAPTER *prAdapter);

/************************************************
 *   NAN Ranging Instance
 ************************************************
 */
void nanRangingInstanceInit(struct ADAPTER *prAdapter,
			    struct _NAN_RANGING_INSTANCE_T *prRanging,
			    uint8_t *puc_peer_mac, uint8_t ucRole);

void nanRangingInstanceAdd(struct ADAPTER *prAdapter,
			   struct _NAN_RANGING_INSTANCE_T *prRanging);

void nanRangingInstanceDel(struct ADAPTER *prAdapter,
			   struct _NAN_RANGING_INSTANCE_T *prRanging);

uint8_t
nanRangingGenerateId(struct ADAPTER *prAdapter);

struct _NAN_RANGING_INSTANCE_T *
nanRangingInstanceSearchByMac(struct ADAPTER *prAdapter,
			      const uint8_t *puc_peer_mac);

struct _NAN_RANGING_INSTANCE_T *
nanRangingInstanceSearchById(struct ADAPTER *prAdapter, uint16_t u2RangingId);

/************************************************
 *   NAN Ranging Attribute
 ************************************************
 */
uint32_t nanGetFtmRangeReportAttr(struct ADAPTER *prAdapter, uint8_t **ppucAttr,
				  uint32_t *pu4AttrLen, uint8_t *pucDevAddr);

uint32_t nanGetRangingInfoAttr(struct ADAPTER *prAdapter, uint8_t **ppucAttr,
			       uint32_t *pu4AttrLen, uint8_t *pucDevAddr);

uint32_t nanGetRangingSetupAttr(struct ADAPTER *prAdapter, uint8_t **ppucAttr,
				uint32_t *pu4AttrLen, uint8_t *pucDevAddr);

uint32_t
nanFtmRangeReportAttrHandler(struct ADAPTER *prAdapter,
			     struct _NAN_ATTR_FTM_RANGE_REPORT_T *prAttr,
			     uint8_t *pucDevAddr);

uint32_t nanRangingInfoAttrHandler(struct ADAPTER *prAdapter,
				   struct _NAN_ATTR_RANGING_INFO_T *prAttr,
				   uint8_t *pucDevAddr);

uint32_t nanRangingSetupAttrHandler(struct ADAPTER *prAdapter,
				    struct _NAN_ATTR_RANGING_SETUP_T *prAttr,
				    uint8_t *pucDevAddr);

/************************************************
 *   NAN Ranging Frame
 ************************************************
 */
void nanRangingFrameCompose(struct ADAPTER *prAdapter,
			    struct MSDU_INFO *prMsduInfo,
			    struct _NAN_RANGING_INSTANCE_T *prRanging,
			    uint8_t ucNafSubType);

uint32_t nanParseRangingFrame(IN struct ADAPTER *prAdapter,
			      IN struct SW_RFB *prSwRfb,
			      IN struct _NAN_RANGING_INSTANCE_T *prRanging);

uint32_t nanRangingFrameSend(struct ADAPTER *prAdapter, uint8_t *PeerAddr,
			     uint8_t ucNafSubType);

int32_t
nanRangingRequestTx(struct ADAPTER *prAdapter,
		    struct _NAN_RANGING_INSTANCE_T *prRanging);

uint32_t nanRangingRequestTxDone(IN struct ADAPTER *prAdapter,
				 IN struct MSDU_INFO *prMsduInfo,
				 IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanRangingRequestRx(IN struct ADAPTER *prAdapter,
			     IN struct SW_RFB *prSwRfb);

int32_t
nanRangingResponseTx(struct ADAPTER *prAdapter,
		     struct _NAN_RANGING_INSTANCE_T *prRanging);

uint32_t nanRangingResponseTxDone(IN struct ADAPTER *prAdapter,
				  IN struct MSDU_INFO *prMsduInfo,
				  IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanRangingResponseRx(IN struct ADAPTER *prAdapter,
			      IN struct SW_RFB *prSwRfb);

int32_t
nanRangingTerminationTx(struct ADAPTER *prAdapter,
			struct _NAN_RANGING_INSTANCE_T *prRanging);

uint32_t nanRangingTerminationTxDone(IN struct ADAPTER *prAdapter,
				     IN struct MSDU_INFO *prMsduInfo,
				     IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanRangingTerminationRx(IN struct ADAPTER *prAdapter,
				 IN struct SW_RFB *prSwRfb);

int32_t
nanRangingReportTx(IN struct ADAPTER *prAdapter,
		   struct _NAN_RANGING_INSTANCE_T *prRanging);

uint32_t nanRangingReportTxDone(IN struct ADAPTER *prAdapter,
				IN struct MSDU_INFO *prMsduInfo,
				IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

uint32_t nanRangingReportRx(IN struct ADAPTER *prAdapter,
			    IN struct SW_RFB *prSwRfb);

/************************************************
 *   NAN Ranging State Machine
 ************************************************
 */
void nanRangingFsmStep(struct ADAPTER *prAdapter,
		       struct _NAN_RANGING_INSTANCE_T *prRanging,
		       enum _ENUM_RANGING_STATE_T eNextState);

void nanRangingSessionTimeout(struct ADAPTER *prAdapter, unsigned long ulParam);

/************************************************
 *   Interface for FTM
 ************************************************
 */
void nanRangingFtmParamCmd(IN struct ADAPTER *prAdapter,
			   struct _NAN_RANGING_INSTANCE_T *prRanging);

unsigned char
nanRangingUpdateDistance(struct ADAPTER *prAdapter,
			 struct _NAN_RANGING_INSTANCE_T *prRanging);

uint32_t
nanRangingGeofencingCheck(struct ADAPTER *prAdapter,
			  struct _NAN_RANGING_INSTANCE_T *prRanging);

void nanRangingFtmDoneEvt(IN struct ADAPTER *prAdapter, IN uint8_t *pcuEvtBuf);

/************************************************
 *   Interface for NAN Discovery Engine
 ************************************************
 */
void nanRangingReportDiscCmd(IN struct ADAPTER *prAdapter,
			     struct _NAN_RANGING_REPORT_CMD *msg);

uint32_t nanRangingInvokedByDisc(struct ADAPTER *prAdapter, uint16_t *pu2Id,
				 struct NanRangeRequest *msg);

void nanRangingInvokedByDiscEvt(IN struct ADAPTER *prAdapter,
				IN uint8_t *pcuEvtBuf);

/************************************************
 *   Interface for Application
 ************************************************
 */
uint32_t nanRangingRequest(struct ADAPTER *prAdapter, uint16_t *pu2Id,
			   struct NanRangeRequest *msg);

int32_t
nanRangingCancel(struct ADAPTER *prAdapter, struct NanRangeCancelRequest *msg);

uint32_t nanRangingResponse(struct ADAPTER *prAdapter,
			    struct NanRangeResponse *msg);

void nanRangingRequestIndication(struct ADAPTER *prAdapter,
				 struct _NAN_RANGING_INSTANCE_T *prRanging);

void nanRangingResult(struct ADAPTER *prAdapter,
		      struct _NAN_RANGING_INSTANCE_T *prRanging,
		      uint32_t u4IndChk);

/************************************************
 *   Interface for NAN Scheduler
 ************************************************
 */
void nanRangingScheduleNegoGranted(struct ADAPTER *prAdapter,
				   uint8_t *pu1DevAddr,
				   enum _ENUM_NAN_NEGO_TYPE_T eType,
				   enum _ENUM_NAN_NEGO_ROLE_T eRole,
				   void *pvToken);

uint32_t nanRangingScheduleViolation(struct ADAPTER *prAdapter,
				     uint8_t *pu1DevAddr);

void nanRangingListPrint(struct ADAPTER *prAdapter);

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

#endif
#endif /* _NAN_RANGING_H_ */
