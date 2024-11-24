/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*! \file   "mscs.h"
 *    \brief  The declaration of MSCS functions
 *
 *    Interfaces for MSCS related handling functions
 */


#ifndef _MSCS_H
#define _MSCS_H

#if CFG_MSCS_SUPPORT

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

#define MSCS_USER_PRIORITY_BITMAP                  0xF0
#define MSCS_USER_PRIORITY_LIMIT                   7
#define DEFAULT_STREAM_TIMEOUT                     60
#define MAX_MONITOR_FIVE_TUPLE                     10

#define KEY_BITMAP_LEN_BIT                         128
#define KEY_BITMAP_LEN_BYTE                        16
#define KEY_BITMAP_LEN_DW                          4

#define LEN_OF_FIVE_TUPLE                          13
#define LEN_OF_TCP_INFO                            17
#define LEN_OF_SRC_DST_IP_PORT                     12

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */
enum ENUM_MSCS_REQUEST_ACTION {
	MSCS_REQUEST = 4,
	MSCS_RESPONSE
};

enum ENUM_MSCS_TCLAS_TYPE {
	FRAME_CLASSIFIER_TYPE_0 = 0,
	FRAME_CLASSIFIER_TYPE_1,
	FRAME_CLASSIFIER_TYPE_2,
	FRAME_CLASSIFIER_TYPE_3,
	FRAME_CLASSIFIER_TYPE_4,
	FRAME_CLASSIFIER_TYPE_5,
	FRAME_CLASSIFIER_TYPE_6,
	FRAME_CLASSIFIER_TYPE_7,
	FRAME_CLASSIFIER_TYPE_8,
	FRAME_CLASSIFIER_TYPE_9,
	FRAME_CLASSIFIER_TYPE_10,
	FRAME_CLASSIFIER_TYPE_NUM
};

enum ENUM_MSCS_PORT_AUTHORITY {
	PORT_NOT_AUTHORIZED,
	PORT_AUTHORIZE_REQ,
	PORT_AUTHORIZING_VERIFY_AP,
	PORT_AUTHORIZING_RETRIEVE_STA,
	PORT_AUTHORIZED,
};

enum ENUM_FAST_PATH_TYPE {
	TYPE_ANNOUNCE = 0,
	TYPE_REQ,
	TYPE_RSP,
	TYPE_CONFIRM,
};

enum ENUM_FAST_PATH_STATUS {
	STATUS_OK = 0,
	STATUS_FAIL,
	STATUS_NO_MATCH_KEY,
};

enum ENUM_INTRESET_PROTOCOL {
	MONITOR_UDP = 0,
	MONITOR_TCP,
	MONITOR_BOTH_UDP_TCP,
};

struct MSCS_FIVE_TUPLE_T {
	struct LINK_ENTRY rLinkEntry;
	uint32_t u4SrcIp;
	uint32_t u4DestIp;
	uint16_t u2SrcPort;
	uint16_t u2DestPort;
	uint8_t  ucProtocol;
};

struct MSCS_TCP_INFO_T {
	struct LINK_ENTRY rLinkEntry;
	uint32_t u4SrcIp;
	uint32_t u4DestIp;
	uint16_t u2SrcPort;
	uint16_t u2DestPort;
	uint32_t u4Seq;
	uint8_t  ucFlag;
};

struct MSCS_CAP_FAST_PATH {
	uint8_t  ucVersion;
	uint8_t  fgSupportFastPath;
	uint32_t u4KeyBitmap[4];
	uint8_t  ucReserved[2];
};

struct FAST_PATH_INFO {
	enum ENUM_MSCS_PORT_AUTHORITY eAuthStatus;
	uint8_t  ucKeyNumHitted;
	uint8_t  fgKeyBitmapMatchStatus;
	uint8_t  ucTransactionId;
	uint16_t u2RandomNumSta;
	uint16_t u2MicReqSta;
	uint16_t u2MicRspAP;
	uint32_t au4KeyBitmapHitted[4];
};
/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

#define GET_SRC_IP_BY_IE(_pucIE) \
	((uint8_t *) &(((struct IE_TCLAS_CLASS_TYPE_4 *) \
	(&((struct IE_TCLAS_MASK *) _pucIE)->aucFrameClassifier[0]))->u4SrcIp))

#define GET_PROTOCOL_BY_IE(_pucIE) \
	(((struct IE_TCLAS_CLASS_TYPE_4 *) \
	(&((struct IE_TCLAS_MASK *) _pucIE)->aucFrameClassifier[0])) \
	->ucProtocol)

#define SET_SRC_IP_FROM_ETH_BODY_DST_IP(_dst, _pucEthBody) \
	((_dst)->u4SrcIp = \
	*(uint32_t *) &_pucEthBody[IPV4_HDR_IP_DST_ADDR_OFFSET])

#define SET_DST_IP_FROM_ETH_BODY_STC_IP(_dst, _pucEthBody) \
	((_dst)->u4DestIp = \
	*(uint32_t *) &_pucEthBody[IPV4_HDR_IP_SRC_ADDR_OFFSET])

#define SET_SRC_PORT_FROM_PRO_BODY_DST_PORT(_dst, _pucProtocolBody) \
	((_dst)->u2SrcPort = \
	*(uint16_t *) &_pucProtocolBody[TCP_HDR_DST_PORT_OFFSET])

#define SET_DST_PORT_FROM_PRO_BODY_SRC_PORT(_dst, _pucProtocolBody) \
	((_dst)->u2DestPort = \
	*(uint16_t *) &_pucProtocolBody[TCP_HDR_SRC_PORT_OFFSET])

#define GET_TCP_SEQ_FROM_TCP_BODY(_pucTcpBody) \
	((_pucTcpBody[TCP_HDR_SEQ] << 24) | \
	(_pucTcpBody[TCP_HDR_SEQ + 1] << 16) | \
	(_pucTcpBody[TCP_HDR_SEQ + 2] << 8) | \
	(_pucTcpBody[TCP_HDR_SEQ + 3]))

#define GET_TCP_ACK_FROM_TCP_BODY(_pucTcpBody) \
	((_pucTcpBody[TCP_HDR_ACK] << 24) | \
	(_pucTcpBody[TCP_HDR_ACK + 1] << 16) | \
	(_pucTcpBody[TCP_HDR_ACK + 2] << 8) | \
	(_pucTcpBody[TCP_HDR_ACK + 3]))


/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

uint8_t mscsIsFpSupport(IN struct ADAPTER *prAdapter);
uint8_t mscsIsNeedRequest(IN struct ADAPTER *prAdapter, IN void *prPacket);
uint32_t mscsRequest(IN struct ADAPTER *prAdapter, IN void *prPacket,
	IN enum ENUM_MSCS_REQUEST_ACTION eAction,
	IN enum ENUM_MSCS_TCLAS_TYPE eTCLASType);
void mscsProcessRobustAVStreaming(IN struct ADAPTER *prAdapter,
	IN struct SW_RFB *prSwRfb);
void mscsDeactivate(IN struct ADAPTER *prAdapter,
	IN struct STA_RECORD *prStaRec);
void fpEventHandler(IN struct ADAPTER *prAdapter,
	IN struct WIFI_EVENT *prEvent);
void fpProcessVendorSpecProtectedFrame(IN struct ADAPTER *prAdapter,
	IN struct SW_RFB *prSwRfb);
void mscsHandleRxPacket(IN struct ADAPTER *prAdapter, struct SW_RFB *prSwRfb);

#endif
#endif /* _MSCS_H */
