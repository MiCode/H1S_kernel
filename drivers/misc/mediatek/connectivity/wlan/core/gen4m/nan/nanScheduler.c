/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include "precomp.h"

#define CFG_NAN_SIGMA_TEST 1
#define CFG_NAN_AVAIL_CTRL_RESET_TIMEOUT 100

#define NAN_INVALID_SCH_RECORD 0xFFFFFFFF
#define NAN_INVALID_AVAIL_DB_INDEX 0xFF
#define NAN_INVALID_AVAIL_ENTRY_INDEX 0xFF
#define NAN_INVALID_TIMELINE_EVENT_TAG 0xFF
#define NAN_INVALID_SLOT_INDEX 0xFFFF
#define NAN_INVALID_CHNL_ID 0xFF

#define NAN_MAX_NDC_RECORD (NAN_MAX_CONN_CFG + 1) /* one for default NDC */
#define NAN_DEFAULT_NDC_IDX 0

#define NAN_2G_DW_INDEX 0
#define NAN_5G_DW_INDEX 8

#define NAN_5G_HIGH_DISC_CH_OP_CLASS 124
#define NAN_5G_HIGH_DISC_CHANNEL 149
#define NAN_5G_LOW_DISC_CH_OP_CLASS 115
#define NAN_5G_LOW_DISC_CHANNEL 44
#define NAN_2P4G_DISC_CH_OP_CLASS 81
#define NAN_2P4G_DISC_CHANNEL 6

#define NAN_5G_HIGH_BW40_DISC_CH_OP_CLASS 126
#define NAN_5G_LOW_BW40_DISC_CH_OP_CLASS 116
#define NAN_2P4G_BW40_DISC_CH_OP_CLASS 83

#define NAN_5G_HIGH_BW80_DISC_CH_OP_CLASS 130
#define NAN_5G_LOW_BW80_DISC_CH_OP_CLASS 128

#define NAN_IS_AVAIL_MAP_SET(pu4AvailMap, u2SlotIdx)                           \
	((pu4AvailMap[u2SlotIdx / NAN_SLOTS_PER_DW_INTERVAL] &                 \
	  BIT(u2SlotIdx % NAN_SLOTS_PER_DW_INTERVAL)) != 0)

#define NAN_TIMELINE_SET(pu4AvailMap, u2SlotIdx)                               \
	(pu4AvailMap[(u2SlotIdx) / NAN_SLOTS_PER_DW_INTERVAL] |=               \
	 BIT((u2SlotIdx) % NAN_SLOTS_PER_DW_INTERVAL))
#define NAN_TIMELINE_UNSET(pu4AvailMap, u2SlotIdx)                             \
	(pu4AvailMap[(u2SlotIdx) / NAN_SLOTS_PER_DW_INTERVAL] &=               \
	 (~BIT((u2SlotIdx) % NAN_SLOTS_PER_DW_INTERVAL)))

#define NAN_MAX_POTENTIAL_CHNL_LIST 10

enum _ENUM_NAN_WINDOW_T {
	ENUM_NAN_DW,
	ENUM_NAN_FAW,
	ENUM_NAN_IDLE_WINDOW,
};

enum _ENUM_CHNL_CHECK_T {
	ENUM_CHNL_CHECK_PASS,
	ENUM_CHNL_CHECK_CONFLICT,
	ENUM_CHNL_CHECK_NOT_FOUND,
};

enum _ENUM_NEGO_CTRL_RST_REASON_T {
	ENUM_NEGO_CTRL_RST_BY_WAIT_RSP_STATE = 1,
	ENUM_NEGO_CTRL_RST_PREPARE_WAIT_RSP_STATE,
	ENUM_NEGO_CTRL_RST_PREPARE_CONFIRM_STATE,
};

enum _ENUM_TIME_BITMAP_CTRL_PERIOD_T {
	ENUM_TIME_BITMAP_CTRL_PERIOD_128 = 1, /* 8 NAN slots */
	ENUM_TIME_BITMAP_CTRL_PERIOD_256,     /* 16 NAN slots */
	ENUM_TIME_BITMAP_CTRL_PERIOD_512,     /* 32 NAN slots */
	ENUM_TIME_BITMAP_CTRL_PERIOD_1024,    /* 64 NAN slots */
	ENUM_TIME_BITMAP_CTRL_PERIOD_2048,    /* 128 NAN slots */
	ENUM_TIME_BITMAP_CTRL_PERIOD_4096,    /* 256 NAN slots */
	ENUM_TIME_BITMAP_CTRL_PERIOD_8192,    /* 512 NAN slots */
};

enum _ENUM_TIME_BITMAP_CTRL_DURATION {
	ENUM_TIME_BITMAP_CTRL_DURATION_16 = 0, /* 1 NAN slot */
	ENUM_TIME_BITMAP_CTRL_DURATION_32,     /* 2 NAN slots */
	ENUM_TIME_BITMAP_CTRL_DURATION_64,     /* 4 NAN slots */
	ENUM_TIME_BITMAP_CTRL_DURATION_128,    /* 8 NAN slots */
};

struct _NAN_DEVICE_CAPABILITY_T {
	unsigned char fgValid;
	uint8_t ucMapId;
	uint8_t ucSupportedBand;
	uint8_t ucOperationMode;
	uint8_t ucDw24g;
	uint8_t ucDw5g;
	uint8_t ucOvrDw24gMapId;
	uint8_t ucOvrDw5gMapId;
	uint8_t ucNumTxAnt;
	uint8_t ucNumRxAnt;
	uint8_t ucCapabilitySet;

	uint16_t u2MaxChnlSwitchTime;
};

struct _NAN_NDC_CTRL_T {
	unsigned char fgValid;
	uint8_t aucNdcId[NAN_NDC_ATTRIBUTE_ID_LENGTH];
	uint8_t aucRsvd[1];
	struct _NAN_SCHEDULE_TIMELINE_T arTimeline[NAN_NUM_AVAIL_DB];
};

union _NAN_AVAIL_ENTRY_CTRL {
	struct {
		uint16_t u2Type : 3;
		uint16_t u2Preference : 2;
		uint16_t u2Util : 3;
		uint16_t u2RxNss : 4;
		uint16_t u2TimeMapAvail : 1;
		uint16_t u2Rsvd : 3;
	} rField;

	uint16_t u2RawData;
};

struct _NAN_AVAILABILITY_TIMELINE_T {
	unsigned char fgActive;

	union _NAN_AVAIL_ENTRY_CTRL rEntryCtrl;

	uint8_t ucNumBandChnlCtrl;
	union _NAN_BAND_CHNL_CTRL arBandChnlCtrl[NAN_NUM_BAND_CHNL_ENTRY];

	uint32_t au4AvailMap[NAN_TOTAL_DW];
};

struct _NAN_AVAILABILITY_DB_T {
	uint8_t ucMapId;

	struct _NAN_AVAILABILITY_TIMELINE_T
		arAvailEntryList[NAN_NUM_AVAIL_TIMELINE];
};

struct _NAN_PEER_SCH_DESC_T {
	struct LINK_ENTRY rLinkEntry;

	uint8_t aucNmiAddr[MAC_ADDR_LEN];

	OS_SYSTIME rUpdateTime;

	unsigned char fgUsed;   /* indicate the SCH DESC is used by a SCH REC */
	uint32_t u4SchIdx; /* valid only when fgUsed is TRUE */

	uint32_t u4AvailAttrToken;
	struct _NAN_AVAILABILITY_DB_T arAvailAttr[NAN_NUM_AVAIL_DB];

	uint32_t u4DevCapAttrToken;
	struct _NAN_DEVICE_CAPABILITY_T arDevCapability[NAN_NUM_AVAIL_DB + 1];

	uint32_t u4QosMinSlots;
	uint32_t u4QosMaxLatency;

	/* for peer proposal cache during shedule negotiation */
	struct _NAN_NDC_CTRL_T rSelectedNdcCtrl;

	unsigned char fgImmuNdlTimelineValid;
	struct _NAN_SCHEDULE_TIMELINE_T arImmuNdlTimeline[NAN_NUM_AVAIL_DB];

	unsigned char fgRangingTimelineValid;
	struct _NAN_SCHEDULE_TIMELINE_T arRangingTimeline[NAN_NUM_AVAIL_DB];
};

/** NAN Peer Schedule Record */
struct _NAN_PEER_SCHEDULE_RECORD_T {
	uint8_t aucNmiAddr[MAC_ADDR_LEN];
	unsigned char fgActive;
	unsigned char fgUseRanging;
	unsigned char fgUseDataPath;

	uint32_t u4DefNdlNumSlots;
	uint32_t u4DefRangingNumSlots;

	uint8_t aucStaRecIdx[NAN_MAX_SUPPORT_NDP_NUM];

	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	/* [Common timeline information]
	 * 1. not support simultaneous availability schedule on local side
	 * 2. the Map ID refers to the local availability schedule
	 *    (the Map ID between rmt & local doesn't need to be same)
	 */
	struct _NAN_NDC_CTRL_T *prCommNdcCtrl;
	struct _NAN_SCHEDULE_TIMELINE_T rCommImmuNdlTimeline;
	struct _NAN_SCHEDULE_TIMELINE_T rCommRangingTimeline;
	struct _NAN_SCHEDULE_TIMELINE_T rCommFawTimeline;

	uint32_t u4FinalQosMinSlots;
	uint32_t u4FinalQosMaxLatency;

	int32_t i4InNegoContext;
	enum ENUM_BAND eBand;

};

enum _ENUM_NAN_CRB_NEGO_STATE_T {
	ENUM_NAN_CRB_NEGO_STATE_IDLE,
	ENUM_NAN_CRB_NEGO_STATE_INITIATOR,
	ENUM_NAN_CRB_NEGO_STATE_RESPONDER,
	ENUM_NAN_CRB_NEGO_STATE_WAIT_RESP,
	ENUM_NAN_CRB_NEGO_STATE_CONFIRM,
};

struct _NAN_CRB_NEGO_TRANSACTION_T {
	uint8_t aucNmiAddr[MAC_ADDR_LEN];
	enum _ENUM_NAN_NEGO_TYPE_T eType;
	enum _ENUM_NAN_NEGO_ROLE_T eRole;

	void(*pfCallback)
	(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
	 enum _ENUM_NAN_NEGO_TYPE_T eType, enum _ENUM_NAN_NEGO_ROLE_T eRole,
	 void *pvToken);
	void *pvToken;
};

struct _NAN_POTENTIAL_CHNL_MAP_T {
	uint8_t ucPrimaryChnl;
	uint8_t aucOperatingClass[NAN_CHNL_BW_NUM];
	uint16_t au2ChnlMapIdx[NAN_CHNL_BW_NUM];
	/* the index in the operating class group */
	uint8_t aucPriChnlMapIdx[NAN_CHNL_BW_NUM];
};

struct _NAN_POTENTIAL_CHNL_T {
	uint8_t ucOpClass;
	uint8_t ucPriChnlBitmap;
	uint16_t u2ChnlBitmap;
};

enum _ENUM_NAN_SYNC_SCH_UPDATE_STATE_T {
	ENUM_NAN_SYNC_SCH_UPDATE_STATE_IDLE = 0,
	ENUM_NAN_SYNC_SCH_UPDATE_STATE_PREPARE,
	ENUM_NAN_SYNC_SCH_UPDATE_STATE_CHECK,
	ENUM_NAN_SYNC_SCH_UPDATE_STATE_RUN,
	ENUM_NAN_SYNC_SCH_UPDATE_STATE_DONE, /* 4 */

	ENUM_NAN_SYNC_SCH_UPDATE_STATE_NUM
};

/* NAN CRB Negotiation Control Block */
struct _NAN_CRB_NEGO_CTRL_T {
	uint32_t u4SchIdx;

	uint32_t u4DefNdlNumSlots;
	uint32_t u4DefRangingNumSlots;

	enum _ENUM_NAN_CRB_NEGO_STATE_T eState;
	enum _ENUM_NAN_NEGO_TYPE_T eType;
	enum _ENUM_NAN_NEGO_ROLE_T eRole;

	/* for local proposal or final proposal cache
	 * during schedule negotiation
	 */
	struct _NAN_NDC_CTRL_T rSelectedNdcCtrl;
	struct _NAN_SCHEDULE_TIMELINE_T rImmuNdlTimeline;
	struct _NAN_SCHEDULE_TIMELINE_T rRangingTimeline;

	uint32_t u4QosMinSlots;
	uint32_t u4QosMaxLatency;
	uint32_t u4NegoQosMinSlots;
	uint32_t u4NegoQosMaxLatency;

	/* for QoS negotiation */
	uint32_t au4AvailSlots[NAN_TOTAL_DW];
	uint32_t au4UnavailSlots[NAN_TOTAL_DW];
	uint32_t au4FreeSlots[NAN_TOTAL_DW];
	uint32_t au4CondSlots[NAN_TOTAL_DW];
	uint32_t au4FawSlots[NAN_TOTAL_DW];

	uint8_t ucNegoTransNum;
	uint8_t ucNegoTransHeadPos;
	struct _NAN_CRB_NEGO_TRANSACTION_T
		rNegoTrans[NAN_CRB_NEGO_MAX_TRANSACTION];

	struct TIMER rCrbNegoDispatchTimer;

	enum _ENUM_NAN_SYNC_SCH_UPDATE_STATE_T eSyncSchUpdateLastState;
	enum _ENUM_NAN_SYNC_SCH_UPDATE_STATE_T eSyncSchUpdateCurrentState;
	uint32_t u4SyncPeerSchRecIdx;

	int32_t i4InNegoContext;
};

struct _NAN_CHANNEL_TIMELINE_T {
	unsigned char fgValid;
	uint8_t aucRsvd[3];

	union _NAN_BAND_CHNL_CTRL rChnlInfo;

	int32_t i4Num; /* track the number of slot in availability map */
	uint32_t au4AvailMap[NAN_TOTAL_DW];
};

struct _NAN_TIMELINE_MGMT_T {
	uint8_t ucMapId;

	/* for committed availability type */
	struct _NAN_CHANNEL_TIMELINE_T
		arChnlList[NAN_TIMELINE_MGMT_CHNL_LIST_NUM];

	/* for conditional availability type */
	unsigned char fgChkCondAvailability;
	struct _NAN_CHANNEL_TIMELINE_T
		arCondChnlList[NAN_TIMELINE_MGMT_CHNL_LIST_NUM];

	/* for custom committed FAW */
	struct _NAN_CHANNEL_TIMELINE_T
		arCustChnlList[NAN_TIMELINE_MGMT_CHNL_LIST_NUM];
};

/* NAN Scheduler Control Block */
struct _NAN_SCHEDULER_T {
	unsigned char fgInit;

	unsigned char fgEn2g;
	unsigned char fgEn5gH;
	unsigned char fgEn5gL;

	uint8_t ucNanAvailAttrSeqId; /* shared by all availability attr */
	uint16_t u2NanAvailAttrControlField;     /* tracking changed flags */
	uint16_t u2NanCurrAvailAttrControlField; /* tracking changed flags */

	struct _NAN_NDC_CTRL_T arNdcCtrl[NAN_MAX_NDC_RECORD];

	struct LINK rPeerSchDescList;
	struct LINK rFreePeerSchDescList;
	struct _NAN_PEER_SCH_DESC_T arPeerSchDescArray[NAN_NUM_PEER_SCH_DESC];

	unsigned char fgAttrDevCapValid;
	struct _NAN_ATTR_DEVICE_CAPABILITY_T rAttrDevCap;

	uint32_t u4NumOfPotentialChnlList;
	struct _NAN_CHNL_ENTRY_T
		arPotentialChnlList[NAN_MAX_POTENTIAL_CHNL_LIST];

	struct TIMER rAvailAttrCtrlResetTimer;

	uint8_t ucCommitDwInterval;
};

struct _NAN_SCHED_CMD_UPDATE_CRB_T {
	uint32_t u4SchIdx;
	uint8_t fgUseDataPath;
	uint8_t fgUseRanging;
	uint8_t aucRsvd[2];
	struct _NAN_SCHEDULE_TIMELINE_T rCommRangingTimeline;
	struct _NAN_SCHEDULE_TIMELINE_T rCommFawTimeline;
	struct _NAN_NDC_CTRL_T rCommNdcCtrl;
};

struct _NAN_SCHED_CMD_MANAGE_PEER_SCH_REC_T {
	uint32_t u4SchIdx;
	uint8_t fgActivate;
	uint8_t aucNmiAddr[MAC_ADDR_LEN];
	uint8_t aucRsvd[1];
};

struct _NAN_SCHED_CMD_UPDATE_PEER_CAPABILITY_T {
	uint32_t u4SchIdx;
	uint8_t ucSupportedBands;
	uint8_t aucRsvd[3];
};

struct _NAN_SCHED_CMD_MAP_STA_REC_T {
	uint8_t aucNmiAddr[MAC_ADDR_LEN];
	uint8_t ucStaRecIdx;
	uint8_t ucNdpCxtId;

	enum NAN_BSS_ROLE_INDEX eRoleIdx;
};

struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_T {
	uint8_t ucMapId;
	uint8_t fgChkCondAvailability;
	uint8_t aucRsvd[2];

	struct _NAN_CHANNEL_TIMELINE_T
		arChnlList[NAN_TIMELINE_MGMT_CHNL_LIST_NUM];
};

struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_CTRL_T {
	uint16_t u2AvailAttrControlField;
	uint8_t ucAvailSeqID;
	uint8_t aucRsvd[1];
};

struct _NAN_SCHED_CMD_UPDATE_PHY_PARAM_T {
	struct _NAN_PHY_SETTING_T r2P4GPhySettings;
	struct _NAN_PHY_SETTING_T r5GPhySettings;
};

struct _NAN_SCHED_CMD_UPDATE_PONTENTIAL_CHNL_LIST_T {
	uint32_t u4Num;
	struct _NAN_POTENTIAL_CHNL_T arChnlList[NAN_MAX_POTENTIAL_CHNL_LIST];
};

struct _NAN_SCHED_EVENT_SCHEDULE_CONFIG_T {
	uint8_t fgEn2g;
	uint8_t fgEn5gH;
	uint8_t fgEn5gL;
	uint8_t aucRsvd[1];
};

struct _NAN_SCHED_EVENT_DW_INTERVAL_T {
	uint8_t ucDwInterval;
};

uint8_t g_aucNanIEBuffer[NAN_IE_BUF_MAX_SIZE];

uint32_t g_u4MaxChnlSwitchTimeUs = 8000;

struct _NAN_CRB_NEGO_CTRL_T g_rNanSchNegoCtrl;
struct _NAN_PEER_SCHEDULE_RECORD_T g_arNanPeerSchedRecord[NAN_MAX_CONN_CFG];
struct _NAN_TIMELINE_MGMT_T g_rNanTimelineMgmt;
struct _NAN_SCHEDULER_T g_rNanScheduler;

union _NAN_BAND_CHNL_CTRL g_rNullChnl = {.u4RawData = 0 };

union _NAN_BAND_CHNL_CTRL g_r2gDwChnl = {
	.rChannel.u4Type = NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL,
	.rChannel.u4OperatingClass = NAN_2P4G_DISC_CH_OP_CLASS,
	.rChannel.u4PrimaryChnl = NAN_2P4G_DISC_CHANNEL,
	.rChannel.u4AuxCenterChnl = 0
};

union _NAN_BAND_CHNL_CTRL g_r5gDwChnl = {
	.rChannel.u4Type = NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL,
	.rChannel.u4OperatingClass = NAN_5G_HIGH_DISC_CH_OP_CLASS,
	.rChannel.u4PrimaryChnl = NAN_5G_HIGH_DISC_CHANNEL,
	.rChannel.u4AuxCenterChnl = 0
};

union _NAN_BAND_CHNL_CTRL g_rPreferredChnl = {
	.rChannel.u4Type = NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL,
	.rChannel.u4OperatingClass = NAN_2P4G_DISC_CH_OP_CLASS,
	.rChannel.u4PrimaryChnl = NAN_2P4G_DISC_CHANNEL,
	.rChannel.u4AuxCenterChnl = 0
};

struct _NAN_POTENTIAL_CHNL_MAP_T g_arPotentialChnlMap[] = {
	{ 149,
	  { 124, 126, 128, 0 },
	  { BIT(0), BIT(0), BIT(5), 0 },
	  { 0, 0, BIT(0), 0 } },
	{ 153,
	  { 124, 127, 128, 0 },
	  { BIT(1), BIT(0), BIT(5), 0 },
	  { 0, 0, BIT(1), 0 } },
	{ 157,
	  { 124, 126, 128, 0 },
	  { BIT(2), BIT(1), BIT(5), 0 },
	  { 0, 0, BIT(2), 0 } },
	{ 161,
	  { 124, 127, 128, 0 },
	  { BIT(3), BIT(1), BIT(5), 0 },
	  { 0, 0, BIT(3), 0 } },
	{ 36,
	  { 115, 116, 128, 129 },
	  { BIT(0), BIT(0), BIT(0), BIT(0) },
	  { 0, 0, BIT(0), BIT(0) } },
	{ 40,
	  { 115, 117, 128, 129 },
	  { BIT(1), BIT(0), BIT(0), BIT(0) },
	  { 0, 0, BIT(1), BIT(1) } },
	{ 44,
	  { 115, 116, 128, 129 },
	  { BIT(2), BIT(1), BIT(0), BIT(0) },
	  { 0, 0, BIT(2), BIT(2) } },
	{ 48,
	  { 115, 117, 128, 129 },
	  { BIT(3), BIT(1), BIT(0), BIT(0) },
	  { 0, 0, BIT(3), BIT(3) } },
	{ 1, { 81, 0, 0, 0 }, { BIT(0), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 2, { 81, 0, 0, 0 }, { BIT(1), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 3, { 81, 0, 0, 0 }, { BIT(2), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 4, { 81, 0, 0, 0 }, { BIT(3), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 5, { 81, 0, 0, 0 }, { BIT(4), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 6, { 81, 0, 0, 0 }, { BIT(5), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 7, { 81, 0, 0, 0 }, { BIT(6), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 8, { 81, 0, 0, 0 }, { BIT(7), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 9, { 81, 0, 0, 0 }, { BIT(8), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 10, { 81, 0, 0, 0 }, { BIT(9), 0, 0, 0 }, { 0, 0, 0, 0 } },
	{ 11, { 81, 0, 0, 0 }, { BIT(10), 0, 0, 0 }, { 0, 0, 0, 0 } },

	{ 0, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } }
};

/* Porting from FW, TODO: move to correct file */
enum _ENUM_CNM_CH_CONCURR_T {
	CNM_CH_CONCURR_MCC,
	CNM_CH_CONCURR_SCC_NEW,
	CNM_CH_CONCURR_SCC_CURR,
	CNM_CH_CONCURR_NUM
};

void
cnmGetChBound(IN uint8_t ucPrimaryCh, IN enum ENUM_CHANNEL_WIDTH eChannelWidth,
	      IN enum ENUM_CHNL_EXT eSCO, IN uint8_t ucChannelS1,
	      IN uint8_t ucChannelS2, OUT uint8_t *pucChLowBound1,
	      OUT uint8_t *pucChHighBound1, OUT uint8_t *pucChLowBound2,
	      OUT uint8_t *pucChHighBound2) {
	switch (eChannelWidth) {
	case CW_20_40MHZ:
		if (eSCO == CHNL_EXT_SCB) {
			*pucChLowBound1 = (ucPrimaryCh - 4);
			*pucChHighBound1 = ucPrimaryCh;
		} else if (eSCO == CHNL_EXT_SCA) {
			*pucChLowBound1 = ucPrimaryCh;
			*pucChHighBound1 = (ucPrimaryCh + 4);
		} else {
			*pucChLowBound1 = ucPrimaryCh;
			*pucChHighBound1 = ucPrimaryCh;
		}

		break;

	case CW_80MHZ:
		*pucChLowBound1 = (ucChannelS1 - 6);
		*pucChHighBound1 = (ucChannelS1 + 6);
		break;

	case CW_160MHZ:
		*pucChLowBound1 = (ucChannelS1 - 14);
		*pucChHighBound1 = (ucChannelS1 + 14);
		break;

	case CW_80P80MHZ:
		*pucChLowBound1 = (ucChannelS1 - 6);
		*pucChHighBound1 = (ucChannelS1 + 6);
		*pucChLowBound2 = (ucChannelS2 * 2 - ucChannelS1 -
				   6); /* S1 + (S2-S1)*2 = S2*2 - S1 */
		*pucChHighBound2 = (ucChannelS2 * 2 - ucChannelS1 +
				    6); /* S1 + (S2-S1)*2 = S2*2 - S1 */
		break;

	default:
		*pucChLowBound1 = ucPrimaryCh;
		*pucChHighBound1 = ucPrimaryCh;
		break;
	}
}

enum _ENUM_CNM_CH_CONCURR_T
cnmChConCurrType(IN uint8_t ucPrimaryChNew,
		 IN enum ENUM_CHANNEL_WIDTH eChannelWidthNew,
		 IN enum ENUM_CHNL_EXT eSCONew, IN uint8_t ucChannelS1New,
		 IN uint8_t ucChannelS2New, IN uint8_t ucPrimaryChCurr,
		 IN enum ENUM_CHANNEL_WIDTH eChannelWidthCurr,
		 IN enum ENUM_CHNL_EXT eSCOCurr, IN uint8_t ucChannelS1Curr,
		 IN uint8_t ucChannelS2Curr) {
	uint8_t ucChLowBound_New1;
	uint8_t ucChHighBound_New1;
	uint8_t ucChLowBound_New2;
	uint8_t ucChHighBound_New2;

	uint8_t ucChLowBound_Curr1;
	uint8_t ucChHighBound_Curr1;
	uint8_t ucChLowBound_Curr2;
	uint8_t ucChHighBound_Curr2;

	cnmGetChBound(ucPrimaryChNew, eChannelWidthNew, eSCONew, ucChannelS1New,
		      ucChannelS2New, &ucChLowBound_New1, &ucChHighBound_New1,
		      &ucChLowBound_New2, &ucChHighBound_New2);

	cnmGetChBound(ucPrimaryChCurr, eChannelWidthCurr, eSCOCurr,
		      ucChannelS1Curr, ucChannelS2Curr, &ucChLowBound_Curr1,
		      &ucChHighBound_Curr1, &ucChLowBound_Curr2,
		      &ucChHighBound_Curr2);

	if (eChannelWidthNew != CW_80P80MHZ &&
	    eChannelWidthCurr != CW_80P80MHZ) {

		if ((ucChLowBound_Curr1 >= ucChLowBound_New1) &&
		    (ucChHighBound_Curr1 <= ucChHighBound_New1)) {

			return CNM_CH_CONCURR_SCC_NEW;

		} else if ((ucChLowBound_New1 >= ucChLowBound_Curr1) &&
			   (ucChHighBound_New1 <= ucChHighBound_Curr1)) {

			return CNM_CH_CONCURR_SCC_CURR;
		}

	} else {

		if (eChannelWidthNew == CW_80P80MHZ &&
		    eChannelWidthCurr == CW_80P80MHZ) {

			if (ucChannelS2Curr ==
				    ucChannelS2New && /* Mirror == Mirror */
			    (ucChannelS1Curr == ucChannelS1New ||
			     ucChannelS1Curr ==
				     (ucChannelS2New * 2 - ucChannelS1New))) {
				/* Same BW */
				return CNM_CH_CONCURR_SCC_CURR;
			}

		} else if (eChannelWidthNew == CW_80P80MHZ) {

			switch (eChannelWidthCurr) {
			case CW_20_40MHZ:
				if (((ucChLowBound_Curr1 >=
				      ucChLowBound_New1) &&
				     (ucChHighBound_Curr1 <=
				      ucChHighBound_New1)) ||
				    ((ucChLowBound_Curr1 >=
				      ucChLowBound_New2) &&
				     (ucChHighBound_Curr1 <=
				      ucChHighBound_New2))) {

					/* curr is subset of new */
					return CNM_CH_CONCURR_SCC_NEW;
				}
				break;

			case CW_80MHZ:
				if (ucChannelS1Curr == ucChannelS1New ||
				    ucChannelS1Curr == (ucChannelS2New * 2 -
							ucChannelS1New)) {
					/* curr is subset of new */
					return CNM_CH_CONCURR_SCC_NEW;
				}
				break;

			case CW_160MHZ:
				if (ucChLowBound_New1 >= ucChLowBound_Curr1 &&
				    ucChLowBound_New2 >= ucChLowBound_Curr1 &&
				    ucChHighBound_New1 <= ucChHighBound_Curr1 &&
				    ucChHighBound_New2 <= ucChHighBound_Curr1) {

					return CNM_CH_CONCURR_SCC_CURR;
				}
				break;
			default:
				break;
			}

		} else if (eChannelWidthCurr == CW_80P80MHZ) {
			switch (eChannelWidthNew) {
			case CW_20_40MHZ:
				if (((ucChLowBound_New1 >=
				      ucChLowBound_Curr1) &&
				     (ucChHighBound_New1 <=
				      ucChHighBound_Curr1)) ||
				    ((ucChLowBound_New1 >=
				      ucChLowBound_Curr2) &&
				     (ucChHighBound_New1 <=
				      ucChHighBound_Curr2))) {
					/* curr is subset of new */
					return CNM_CH_CONCURR_SCC_CURR;
				}
				break;

			case CW_80MHZ:
				if (ucChannelS1New == ucChannelS1Curr ||
				    ucChannelS1New == (ucChannelS2Curr * 2 -
						       ucChannelS1Curr)) {
					/* curr is subset of new */
					return CNM_CH_CONCURR_SCC_CURR;
				}
				break;

			case CW_160MHZ:
				if (ucChLowBound_Curr1 >= ucChLowBound_New1 &&
				    ucChLowBound_Curr2 >= ucChLowBound_New1 &&
				    ucChHighBound_Curr1 <= ucChHighBound_New1 &&
				    ucChHighBound_Curr2 <= ucChHighBound_New1) {

					return CNM_CH_CONCURR_SCC_NEW;
				}
				break;
			default:
				break;
			}
		}
	}

	return CNM_CH_CONCURR_MCC;
}

struct _NAN_SCHEDULER_T *
nanGetScheduler(struct ADAPTER *prAdapter) {
	return &g_rNanScheduler;
}

struct _NAN_TIMELINE_MGMT_T *
nanGetTimelineMgmt(struct ADAPTER *prAdapter) {
	return &g_rNanTimelineMgmt;
}

struct _NAN_CRB_NEGO_CTRL_T *
nanGetNegoControlBlock(struct ADAPTER *prAdapter) {
	return &g_rNanSchNegoCtrl;
}

unsigned char
nanIsAllowedChannel(struct ADAPTER *prAdapter, uint8_t ucChnl) {
	enum ENUM_BAND eBand;
	struct _NAN_SCHEDULER_T *prNanScheduler;

	prNanScheduler = nanGetScheduler(prAdapter);

#if 0
	if (ucChnl < 36) {
		eBand = BAND_2G4;

		if (!prNanScheduler->fgEn2g)
			return FALSE;
	} else if (ucChnl < 100) {
		eBand = BAND_5G;

		if (!prNanScheduler->fgEn5gL)
			return FALSE;
	} else {
		eBand = BAND_5G;

		if (!prNanScheduler->fgEn5gH)
			return FALSE;
	}
#else
	if (ucChnl < 36) {
		eBand = BAND_2G4;

		if (!prNanScheduler->fgEn2g)
			return FALSE;
	} else {
		eBand = BAND_5G;

		if (!prNanScheduler->fgEn5gL && !prNanScheduler->fgEn5gH)
			return FALSE;
	}
#endif

	if (!rlmDomainIsLegalChannel(prAdapter, eBand, ucChnl))
		return FALSE;

	return TRUE;
}

unsigned char
nanIsDiscWindow(struct ADAPTER *prAdapter, uint16_t u2SlotIdx) {
	struct _NAN_SCHEDULER_T *prScheduler;

	prScheduler = nanGetScheduler(prAdapter);

	if (prScheduler->fgEn2g &&
	    (u2SlotIdx % NAN_SLOTS_PER_DW_INTERVAL == NAN_2G_DW_INDEX))
		return TRUE;
	else if (prScheduler->fgEn5gH &&
		 (u2SlotIdx % NAN_SLOTS_PER_DW_INTERVAL == NAN_5G_DW_INDEX))
		return TRUE;
	else if (prScheduler->fgEn5gL &&
		 (u2SlotIdx % NAN_SLOTS_PER_DW_INTERVAL == NAN_5G_DW_INDEX))
		return TRUE;

	return FALSE;
}

enum _ENUM_NAN_WINDOW_T
nanWindowType(struct ADAPTER *prAdapter, uint16_t u2SlotIdx) {
	struct _NAN_SCHEDULER_T *prScheduler;

	prScheduler = nanGetScheduler(prAdapter);

	if (nanIsDiscWindow(prAdapter, u2SlotIdx))
		return ENUM_NAN_DW;
	else if (nanQueryPrimaryChnlBySlot(prAdapter, u2SlotIdx, TRUE) == 0)
		return ENUM_NAN_IDLE_WINDOW;

	return ENUM_NAN_FAW;
}

void
nanSchedAvailAttrCtrlTimeout(struct ADAPTER *prAdapter, unsigned long ulParam) {
	struct _NAN_SCHEDULER_T *prScheduler;

	prScheduler = nanGetScheduler(prAdapter);

	prScheduler->u2NanCurrAvailAttrControlField = 0;

	nanSchedCmdUpdateAvailabilityCtrl(prAdapter);
}

struct _NAN_AVAILABILITY_DB_T *
nanSchedPeerAcquireAvailabilityDB(struct ADAPTER *prAdapter,
				  struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc,
				  uint8_t ucMapId) {
	struct _NAN_AVAILABILITY_DB_T *prNanAvailDB;
	uint32_t u4Idx;
	uint32_t u4InvalidIdx = NAN_NUM_AVAIL_DB;
	uint32_t u4EntryListPos;

	if (!prPeerSchDesc) {
		DBGLOG(NAN, ERROR, "prPeerSchDesc error!\n");
		return NULL;
	}

	for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
		prNanAvailDB = &prPeerSchDesc->arAvailAttr[u4Idx];
		if (prNanAvailDB->ucMapId == NAN_INVALID_MAP_ID) {
			if (u4InvalidIdx == NAN_NUM_AVAIL_DB)
				u4InvalidIdx = u4Idx;
		} else if (prNanAvailDB->ucMapId == ucMapId)
			return prNanAvailDB;
	}

	if (u4InvalidIdx != NAN_NUM_AVAIL_DB) {
		/* initialize new availability attribute */
		DBGLOG(NAN, INFO,
		       "alloc new availability for station idx:%d, mapID:%d\n",
		       u4InvalidIdx, ucMapId);
		prNanAvailDB = &prPeerSchDesc->arAvailAttr[u4InvalidIdx];
		prNanAvailDB->ucMapId = ucMapId;
		for (u4EntryListPos = 0;
		     u4EntryListPos < NAN_NUM_AVAIL_TIMELINE;
		     u4EntryListPos++) {

			prNanAvailDB->arAvailEntryList[u4EntryListPos]
				.fgActive = FALSE;
		}

		return prNanAvailDB;
	}

	return NULL;
}

void
nanSchedResetPeerSchDesc(struct ADAPTER *prAdapter,
			 struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc) {
	uint32_t u4Idx;

	kalMemZero(prPeerSchDesc, sizeof(struct _NAN_PEER_SCH_DESC_T));

	prPeerSchDesc->fgImmuNdlTimelineValid = FALSE;
	prPeerSchDesc->fgRangingTimelineValid = FALSE;

	for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
		prPeerSchDesc->arDevCapability[u4Idx].fgValid = FALSE;
		prPeerSchDesc->arDevCapability[u4Idx].ucMapId =
			NAN_INVALID_MAP_ID;
		prPeerSchDesc->arAvailAttr[u4Idx].ucMapId = NAN_INVALID_MAP_ID;
		prPeerSchDesc->arImmuNdlTimeline[u4Idx].ucMapId =
			NAN_INVALID_MAP_ID;
		prPeerSchDesc->arRangingTimeline[u4Idx].ucMapId =
			NAN_INVALID_MAP_ID;
	}
	prPeerSchDesc->arDevCapability[NAN_NUM_AVAIL_DB].fgValid = FALSE;
	prPeerSchDesc->arDevCapability[NAN_NUM_AVAIL_DB].ucMapId =
		NAN_INVALID_MAP_ID;

	prPeerSchDesc->u4QosMaxLatency = NAN_INVALID_QOS_MAX_LATENCY;
	prPeerSchDesc->u4QosMinSlots = NAN_INVALID_QOS_MIN_SLOTS;

	prPeerSchDesc->u4AvailAttrToken = 0;
	prPeerSchDesc->u4DevCapAttrToken = 0;
}

struct _NAN_PEER_SCH_DESC_T *
nanSchedAcquirePeerSchDescByNmi(struct ADAPTER *prAdapter,
		uint8_t *pucNmiAddr) {
	struct LINK *prPeerSchDescList;
	struct LINK *prFreePeerSchDescList;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDescNext;
	struct _NAN_PEER_SCH_DESC_T *prAgingPeerSchDesc;
	struct _NAN_SCHEDULER_T *prScheduler;

	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return NULL;
	}

	prScheduler = nanGetScheduler(prAdapter);

	prPeerSchDescList = &prScheduler->rPeerSchDescList;
	prFreePeerSchDescList = &prScheduler->rFreePeerSchDescList;
	prAgingPeerSchDesc = NULL;

	LINK_FOR_EACH_ENTRY_SAFE(prPeerSchDesc, prPeerSchDescNext,
				 prPeerSchDescList, rLinkEntry,
				 struct _NAN_PEER_SCH_DESC_T) {
		if (EQUAL_MAC_ADDR(prPeerSchDesc->aucNmiAddr, pucNmiAddr)) {
			LINK_REMOVE_KNOWN_ENTRY(prPeerSchDescList,
						prPeerSchDesc);

			LINK_INSERT_TAIL(prPeerSchDescList,
					 &prPeerSchDesc->rLinkEntry);
			DBGLOG(NAN, LOUD, "Find peer schedule desc %p [%d]\n",
			       prPeerSchDesc, __LINE__);
			return prPeerSchDesc;
		} else if ((prAgingPeerSchDesc == NULL) &&
			   (prPeerSchDesc->fgUsed == FALSE))
			prAgingPeerSchDesc = prPeerSchDesc;
	}

	LINK_REMOVE_HEAD(prFreePeerSchDescList, prPeerSchDesc,
			 struct _NAN_PEER_SCH_DESC_T *);
	if (prPeerSchDesc) {
		nanSchedResetPeerSchDesc(prAdapter, prPeerSchDesc);
		kalMemCopy(prPeerSchDesc->aucNmiAddr, pucNmiAddr, MAC_ADDR_LEN);

		LINK_INSERT_TAIL(prPeerSchDescList, &prPeerSchDesc->rLinkEntry);
	} else if (prAgingPeerSchDesc) {
		LINK_REMOVE_KNOWN_ENTRY(prPeerSchDescList, prAgingPeerSchDesc);

		nanSchedResetPeerSchDesc(prAdapter, prAgingPeerSchDesc);
		kalMemCopy(prAgingPeerSchDesc->aucNmiAddr, pucNmiAddr,
			   MAC_ADDR_LEN);

		LINK_INSERT_TAIL(prPeerSchDescList,
				 &prAgingPeerSchDesc->rLinkEntry);
		DBGLOG(NAN, LOUD, "Find peer schedule desc %p [%d]\n",
		       prPeerSchDesc, __LINE__);
		return prAgingPeerSchDesc;
	}

	DBGLOG(NAN, LOUD, "Find peer schedule desc %p [%d]\n", prPeerSchDesc,
	       __LINE__);
	return prPeerSchDesc;
}

void
nanSchedReleaseAllPeerSchDesc(struct ADAPTER *prAdapter) {
	struct LINK *prPeerSchDescList;
	struct LINK *prFreePeerSchDescList;
	struct _NAN_SCHEDULER_T *prNanScheduler;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return;
	}

	prNanScheduler = nanGetScheduler(prAdapter);
	prPeerSchDescList = &prNanScheduler->rPeerSchDescList;
	prFreePeerSchDescList = &prNanScheduler->rFreePeerSchDescList;

	while (!LINK_IS_EMPTY(prPeerSchDescList)) {
		LINK_REMOVE_HEAD(prPeerSchDescList, prPeerSchDesc,
				 struct _NAN_PEER_SCH_DESC_T *);
		if (prPeerSchDesc) {
			kalMemZero(prPeerSchDesc, sizeof(*prPeerSchDesc));
			LINK_INSERT_TAIL(prFreePeerSchDescList,
					 &prPeerSchDesc->rLinkEntry);
		} else {
			/* should not deliver to this function */
			DBGLOG(NAN, ERROR, "prPeerSchDesc error!\n");
			return;
		}
	}
}

struct _NAN_PEER_SCH_DESC_T *
nanSchedSearchPeerSchDescByNmi(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	struct LINK *prPeerSchDescList;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	struct _NAN_SCHEDULER_T *prNanScheduler;

	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return NULL;
	}
	prNanScheduler = nanGetScheduler(prAdapter);

	prPeerSchDescList = &prNanScheduler->rPeerSchDescList;

	LINK_FOR_EACH_ENTRY(prPeerSchDesc, prPeerSchDescList, rLinkEntry,
			    struct _NAN_PEER_SCH_DESC_T) {
		if (EQUAL_MAC_ADDR(prPeerSchDesc->aucNmiAddr, pucNmiAddr))
			return prPeerSchDesc;
	}

	return NULL;
}

struct _NAN_PEER_SCHEDULE_RECORD_T *
nanSchedGetPeerSchRecord(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	if (u4SchIdx < NAN_MAX_CONN_CFG)
		return &g_arNanPeerSchedRecord[u4SchIdx];

	return NULL;
}

struct _NAN_PEER_SCH_DESC_T *
nanSchedGetPeerSchDesc(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;

	prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRec)
		return prPeerSchRec->prPeerSchDesc;

	return NULL;
}

unsigned char
nanSchedPeerSchRecordIsValid(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;

	prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRec)
		return prPeerSchRec->fgActive;

	return FALSE;
}

void
nanSchedDumpPeerSchDesc(struct ADAPTER *prAdapter,
			struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc) {
	uint32_t u4Idx;
	struct _NAN_SCHEDULE_TIMELINE_T *prTimeline;

	if (prPeerSchDesc == NULL) {
		DBGLOG(NAN, INFO, "null peer sch desc\n");
		return;
	}

	nanSchedDbgDumpPeerAvailability(prAdapter, prPeerSchDesc->aucNmiAddr);

	if (prPeerSchDesc->rSelectedNdcCtrl.fgValid == TRUE) {
		nanUtilDump(prAdapter, "NDC ID",
			    prPeerSchDesc->rSelectedNdcCtrl.aucNdcId,
			    NAN_NDC_ATTRIBUTE_ID_LENGTH);

		for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
			prTimeline = &prPeerSchDesc->rSelectedNdcCtrl
					      .arTimeline[u4Idx];
			if (prTimeline->ucMapId == NAN_INVALID_MAP_ID)
				continue;

			nanUtilDump(prAdapter, "Selected NDC",
				    (uint8_t *)prTimeline->au4AvailMap,
				    sizeof(prTimeline->au4AvailMap));
		}
	}

	if (prPeerSchDesc->fgImmuNdlTimelineValid) {
		for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
			prTimeline = &prPeerSchDesc->arImmuNdlTimeline[u4Idx];

			if (prTimeline->ucMapId == NAN_INVALID_MAP_ID)
				continue;

			nanUtilDump(prAdapter, "ImmuNDL",
				    (uint8_t *)prTimeline->au4AvailMap,
				    sizeof(prTimeline->au4AvailMap));
		}
	}

	if (prPeerSchDesc->fgRangingTimelineValid) {
		for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
			prTimeline = &prPeerSchDesc->arRangingTimeline[u4Idx];

			if (prTimeline->ucMapId == NAN_INVALID_MAP_ID)
				continue;

			nanUtilDump(prAdapter, "Rang Map",
				    (uint8_t *)prTimeline->au4AvailMap,
				    sizeof(prTimeline->au4AvailMap));
		}
	}

	DBGLOG(NAN, INFO, "[QoS] MinSlot:%d, MaxLatency:%d\n",
	       prPeerSchDesc->u4QosMinSlots, prPeerSchDesc->u4QosMaxLatency);
}

uint32_t
nanSchedLookupPeerSchRecordIdx(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	uint32_t u4Idx;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;

	for (u4Idx = 0; u4Idx < NAN_MAX_CONN_CFG; u4Idx++) {
		prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4Idx);

		if (prPeerSchRec && prPeerSchRec->fgActive &&
		    (kalMemCmp(prPeerSchRec->aucNmiAddr, pucNmiAddr,
			       MAC_ADDR_LEN) == 0)) {

			DBGLOG(NAN, INFO, "Find peer schedule record %d\n",
			       u4Idx);
			return u4Idx;
		}
	}

	return NAN_INVALID_SCH_RECORD;
}

struct _NAN_PEER_SCHEDULE_RECORD_T *
nanSchedLookupPeerSchRecord(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	uint32_t u4Idx;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;

	for (u4Idx = 0; u4Idx < NAN_MAX_CONN_CFG; u4Idx++) {
		prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4Idx);

		if (prPeerSchRec && prPeerSchRec->fgActive &&
		    (kalMemCmp(prPeerSchRec->aucNmiAddr, pucNmiAddr,
			       MAC_ADDR_LEN) == 0)) {

			DBGLOG(NAN, INFO, "Find peer schedule record %d\n",
			       u4Idx);
			return prPeerSchRec;
		}
	}

	return NULL;
}

uint32_t
nanSchedQueryStaRecIdx(struct ADAPTER *prAdapter, uint32_t u4SchIdx,
		       uint32_t u4Idx) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRecord == NULL) {
		DBGLOG(NAN, ERROR, "Get Peer Sch Record %d error\n", u4SchIdx);
		return STA_REC_INDEX_NOT_FOUND;
	}

	return prPeerSchRecord->aucStaRecIdx[u4Idx];
}

uint32_t
nanSchedResetPeerSchedRecord(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	uint32_t u4Idx;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);

	if (!prPeerSchRecord) {
		DBGLOG(NAN, ERROR, "prPeerSchRecord error!\n");
		return WLAN_STATUS_FAILURE;
	}
	kalMemZero((uint8_t *)prPeerSchRecord, sizeof(*prPeerSchRecord));

	for (u4Idx = 0; u4Idx < NAN_MAX_SUPPORT_NDP_NUM; u4Idx++)
		prPeerSchRecord->aucStaRecIdx[u4Idx] = STA_REC_INDEX_NOT_FOUND;

	prPeerSchRecord->rCommImmuNdlTimeline.ucMapId = NAN_INVALID_MAP_ID;
	prPeerSchRecord->rCommRangingTimeline.ucMapId = NAN_INVALID_MAP_ID;
	prPeerSchRecord->rCommFawTimeline.ucMapId = NAN_INVALID_MAP_ID;
	prPeerSchRecord->prCommNdcCtrl = NULL;
	prPeerSchRecord->fgUseDataPath = FALSE;
	prPeerSchRecord->fgUseRanging = FALSE;

	prPeerSchRecord->u4FinalQosMaxLatency = NAN_INVALID_QOS_MAX_LATENCY;
	prPeerSchRecord->u4FinalQosMinSlots = NAN_INVALID_QOS_MIN_SLOTS;

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedAcquirePeerSchedRecord(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	uint32_t u4SchIdx;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	DBGLOG(NAN, INFO, "IN\n");

	for (u4SchIdx = 0; u4SchIdx < NAN_MAX_CONN_CFG; u4SchIdx++) {
		prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
		if (prPeerSchRecord->fgActive == FALSE)
			break;
	}

	if (u4SchIdx >= NAN_MAX_CONN_CFG)
		return NAN_INVALID_SCH_RECORD;

	prPeerSchDesc = nanSchedAcquirePeerSchDescByNmi(prAdapter, pucNmiAddr);

	if (!prPeerSchDesc) {
		DBGLOG(NAN, ERROR, "prPeerSchDesc error!\n");
		return NAN_INVALID_SCH_RECORD;
	}
	DBGLOG(NAN, LOUD, "Find peer schedule desc %p\n", prPeerSchDesc);
	prPeerSchDesc->fgUsed = TRUE;
	prPeerSchDesc->u4SchIdx = u4SchIdx;

	nanSchedResetPeerSchedRecord(prAdapter, u4SchIdx);
	prPeerSchRecord->fgActive = TRUE;
	kalMemCopy(prPeerSchRecord->aucNmiAddr, pucNmiAddr, MAC_ADDR_LEN);
	prPeerSchRecord->prPeerSchDesc = prPeerSchDesc;

	nanSchedCmdManagePeerSchRecord(prAdapter, u4SchIdx, TRUE);
	nanSchedCmdUpdatePeerCapability(prAdapter, u4SchIdx);

	return u4SchIdx;
}

uint32_t
nanSchedReleasePeerSchedRecord(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	DBGLOG(NAN, INFO, "IN\n");

	prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRecord->prPeerSchDesc)
		prPeerSchRecord->prPeerSchDesc->fgUsed = FALSE;
	nanSchedResetPeerSchedRecord(prAdapter, u4SchIdx);

	nanSchedCmdManagePeerSchRecord(prAdapter, u4SchIdx, FALSE);
	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedPeerGetAvailabilityDbIdx(struct ADAPTER *prAdapter,
				 struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc,
				 uint32_t u4MapId) {
	uint32_t u4Idx;

	for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
		if (prPeerSchDesc->arAvailAttr[u4Idx].ucMapId == u4MapId)
			return u4Idx;
	}

	return NAN_INVALID_AVAIL_DB_INDEX;
}

unsigned char
nanSchedPeerAvailabilityDbValidByDesc(
	struct ADAPTER *prAdapter, struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc,
	uint32_t u4AvailDbIdx) {
	struct _NAN_AVAILABILITY_DB_T *prAvailabilityDB;

	prAvailabilityDB = &prPeerSchDesc->arAvailAttr[u4AvailDbIdx];
	if (prAvailabilityDB->ucMapId == NAN_INVALID_MAP_ID)
		return FALSE;

	return TRUE;
}

unsigned char
nanSchedPeerAvailabilityDbValidByID(struct ADAPTER *prAdapter,
		uint32_t u4SchIdx, uint32_t u4AvailDbIdx) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;

	prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRec->prPeerSchDesc == NULL)
		return FALSE;

	return nanSchedPeerAvailabilityDbValidByDesc(
		prAdapter, prPeerSchRec->prPeerSchDesc, u4AvailDbIdx);
}

unsigned char
nanSchedPeerAvailabilityDbValid(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;
	uint32_t u4AvailDbIdx;

	prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRec->prPeerSchDesc == NULL)
		return FALSE;

	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB; u4AvailDbIdx++)
		if (nanSchedPeerAvailabilityDbValidByDesc(
			    prAdapter, prPeerSchRec->prPeerSchDesc,
			    u4AvailDbIdx))
			return TRUE;

	return FALSE;
}

uint32_t
nanSchedInit(struct ADAPTER *prAdapter) {
	uint32_t u4Idx;
	struct _NAN_SCHEDULER_T *prNanScheduler;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prNanScheduler = nanGetScheduler(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	if (prNanScheduler->fgInit == FALSE) {
		for (u4Idx = 0; u4Idx < NAN_MAX_CONN_CFG; u4Idx++) {
			prPeerSchRecord =
				nanSchedGetPeerSchRecord(prAdapter, u4Idx);
			kalMemZero(prPeerSchRecord,
				   sizeof(struct _NAN_PEER_SCHEDULE_RECORD_T));
		}

		kalMemZero((uint8_t *)prNegoCtrl, sizeof(*prNegoCtrl));
		kalMemZero((uint8_t *)prNanScheduler, sizeof(*prNanScheduler));
		kalMemZero((uint8_t *)prNanTimelineMgmt,
			   sizeof(*prNanTimelineMgmt));

		LINK_INITIALIZE(&prNanScheduler->rPeerSchDescList);
		LINK_INITIALIZE(&prNanScheduler->rFreePeerSchDescList);
		for (u4Idx = 0; u4Idx < NAN_NUM_PEER_SCH_DESC; u4Idx++) {
			kalMemZero(&prNanScheduler->arPeerSchDescArray[u4Idx],
				   sizeof(struct _NAN_PEER_SCH_DESC_T));
			LINK_INSERT_TAIL(
				&prNanScheduler->rFreePeerSchDescList,
				&prNanScheduler->arPeerSchDescArray[u4Idx]
					 .rLinkEntry);
		}

		cnmTimerInitTimer(prAdapter, &prNegoCtrl->rCrbNegoDispatchTimer,
				  nanSchedNegoDispatchTimeout, 0);
		cnmTimerInitTimer(prAdapter,
				  &prNanScheduler->rAvailAttrCtrlResetTimer,
				  nanSchedAvailAttrCtrlTimeout, 0);
	}

	prNanScheduler->fgInit = TRUE;

	prNanScheduler->u2NanAvailAttrControlField = 0;
	prNanScheduler->u2NanCurrAvailAttrControlField = 0;
	prNanScheduler->ucCommitDwInterval =
		prAdapter->rWifiVar.ucNanCommittedDw;
	prNanScheduler->fgAttrDevCapValid = FALSE;

	for (u4Idx = 0; u4Idx < NAN_MAX_NDC_RECORD; u4Idx++)
		prNanScheduler->arNdcCtrl[u4Idx].fgValid = FALSE;

	prNanTimelineMgmt->ucMapId = NAN_DEFAULT_MAP_ID;
	prNanTimelineMgmt->fgChkCondAvailability = FALSE;
	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx++) {
		prNanTimelineMgmt->arChnlList[u4Idx].fgValid = FALSE;
		prNanTimelineMgmt->arCondChnlList[u4Idx].fgValid = FALSE;
	}

	prNegoCtrl->eSyncSchUpdateCurrentState =
		ENUM_NAN_SYNC_SCH_UPDATE_STATE_IDLE;
	prNegoCtrl->eSyncSchUpdateLastState =
		ENUM_NAN_SYNC_SCH_UPDATE_STATE_IDLE;

	for (u4Idx = 0; u4Idx < NAN_MAX_CONN_CFG; u4Idx++)
		nanSchedReleasePeerSchedRecord(prAdapter, u4Idx);

	nanSchedReleaseAllPeerSchDesc(prAdapter);

	nanSchedConfigPhyParams(prAdapter);

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanUtilCalAttributeToken(struct _NAN_ATTR_HDR_T *prNanAttr) {
	uint8_t *pucPayload;
	uint32_t u4Token, u4Len;
	uint32_t u4Tail;

	pucPayload = prNanAttr->aucAttrBody;
	u4Len = prNanAttr->u2Length;
	u4Token = pucPayload[0];
	pucPayload++;
	u4Len--;

	while (u4Len > 4) {
		u4Token ^= *(uint32_t *)pucPayload;
		pucPayload += 4;
		u4Len -= 4;
	}

	u4Tail = 0;
	while (u4Len) {
		u4Tail <<= 8;
		u4Tail |= pucPayload[0];
		pucPayload++;
		u4Len--;
	}
	u4Token ^= u4Tail;

	return u4Token;
}

uint32_t
nanUtilCheckBitOneCnt(struct ADAPTER *prAdapter, uint8_t *pucBitMask,
		      uint32_t u4Size) {
	uint32_t u4Num;
	uint32_t u4Idx;

	u4Num = 0;
	for (u4Idx = 0; u4Idx < u4Size * 8; u4Idx++) {
		if (pucBitMask[u4Idx / 8] & BIT(u4Idx % 8))
			u4Num++;
	}

	return u4Num;
}

void
nanUtilDump(struct ADAPTER *prAdapter, uint8_t *pucMsg, uint8_t *pucContent,
	    uint32_t u4Length) {
	uint8_t aucBuf[16];

	DBGLOG(NAN, INFO, "%s, len:%d\n", pucMsg, u4Length);

	while (u4Length >= 16) {
		DBGLOG(NAN, INFO,
		       "%p: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		       pucContent, pucContent[0], pucContent[1], pucContent[2],
		       pucContent[3], pucContent[4], pucContent[5],
		       pucContent[6], pucContent[7], pucContent[8],
		       pucContent[9], pucContent[10], pucContent[11],
		       pucContent[12], pucContent[13], pucContent[14],
		       pucContent[15]);

		u4Length -= 16;
		pucContent += 16;
	}

	if (u4Length > 0) {
		kalMemZero(aucBuf, 16);
		kalMemCopy(aucBuf, pucContent, u4Length);

		DBGLOG(NAN, INFO,
		       "%p: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		       pucContent, aucBuf[0], aucBuf[1], aucBuf[2], aucBuf[3],
		       aucBuf[4], aucBuf[5], aucBuf[6], aucBuf[7], aucBuf[8],
		       aucBuf[9], aucBuf[10], aucBuf[11], aucBuf[12],
		       aucBuf[13], aucBuf[14], aucBuf[15]);
	}
}

/* pucTimeBitmapField:
 *       return Time Bitmap Field (must provide at least 67 bytes)
 *
 *   pu4TimeBitmapFieldLength:
 *       return real Time Bitmap Field total length
 */
uint32_t
nanParserGenTimeBitmapField(struct ADAPTER *prAdapter, uint32_t *pu4AvailMap,
			    uint8_t *pucTimeBitmapField,
			    uint32_t *pu4TimeBitmapFieldLength) {
	uint32_t u4RepeatInterval;
	uint32_t u4StartOffset;
	uint32_t u4BitDuration;
	uint32_t u4BitDurationChecked;
	uint32_t u4BitmapLength;
	uint32_t u4BitmapPos1, u4BitmapPos2;
	uint32_t u4CheckPos;
	unsigned char fgCheckDone;
	unsigned char fgBitSet, fgBitSetCheck;
	uint8_t aucTimeBitmapField[NAN_TIME_BITMAP_FIELD_MAX_LENGTH];
	uint16_t *pu2TimeBitmapControl;
	uint8_t *pucTimeBitmapLength;
	uint8_t *pucTimeBitmapValue;
	uint8_t *pucBitmap;
	uint16_t u2TimeBitmapControl;

	pucBitmap = (uint8_t *)&pu4AvailMap[0];

	for (u4StartOffset = 0; u4StartOffset < NAN_TIME_BITMAP_MAX_SIZE * 8;
	     u4StartOffset++) {
		if (pucBitmap[u4StartOffset / 8] & BIT(u4StartOffset % 8))
			break;
	}
#if CFG_NAN_SIGMA_TEST
	u4StartOffset = 0;
#endif

	u4RepeatInterval = ENUM_TIME_BITMAP_CTRL_PERIOD_8192;
	u4BitmapLength = NAN_TIME_BITMAP_MAX_SIZE;
	fgCheckDone = FALSE;
	do {
		u4BitmapLength >>= 1;
		u4BitmapPos1 = 0;
		u4BitmapPos2 = u4BitmapLength;

		for (u4CheckPos = 0; u4CheckPos < u4BitmapLength;
		     u4CheckPos++) {
			if (pucBitmap[u4BitmapPos1] !=
			    pucBitmap[u4BitmapPos2]) {
				fgCheckDone = TRUE;
				break;
			}

			u4BitmapPos1++;
			u4BitmapPos2++;
		}

		if (!fgCheckDone)
			u4RepeatInterval--;
	} while ((u4RepeatInterval > ENUM_TIME_BITMAP_CTRL_PERIOD_128) &&
		 !fgCheckDone);

	u4BitDuration = ENUM_TIME_BITMAP_CTRL_DURATION_128;
	u4CheckPos = u4StartOffset;
	while ((u4CheckPos < (1 << (u4RepeatInterval + 2))) &&
	       (u4BitDuration > ENUM_TIME_BITMAP_CTRL_DURATION_16)) {
		fgBitSet = (pucBitmap[u4CheckPos / 8] & (BIT(u4CheckPos % 8)))
				   ? TRUE
				   : FALSE;

		u4BitmapPos1 = u4CheckPos + 1;
		for (u4BitDurationChecked = (1 << u4BitDuration) - 1;
		     u4BitDurationChecked > 0; u4BitDurationChecked--) {
			if (u4BitmapPos1 >= (1 << (u4RepeatInterval + 2))) {
				u4BitDurationChecked = 0;
				break;
			}

			fgBitSetCheck = (pucBitmap[u4BitmapPos1 / 8] &
					 (BIT(u4BitmapPos1 % 8)))
						? TRUE
						: FALSE;
			if (fgBitSet != fgBitSetCheck)
				break;

			u4BitmapPos1++;
		}

		if (u4BitDurationChecked == 0)
			u4CheckPos += (1 << u4BitDuration);
		else
			u4BitDuration--;
	}
#if CFG_NAN_SIGMA_TEST
	u4BitDuration = ENUM_TIME_BITMAP_CTRL_DURATION_16;
#endif

	kalMemZero(aucTimeBitmapField, NAN_TIME_BITMAP_FIELD_MAX_LENGTH);
	pu2TimeBitmapControl = (uint16_t *)&aucTimeBitmapField[0];
	pucTimeBitmapLength = &aucTimeBitmapField[2];
	pucTimeBitmapValue = &aucTimeBitmapField[3];

	u2TimeBitmapControl =
		(u4BitDuration << NAN_TIME_BITMAP_CTRL_DURATION_OFFSET) |
		(u4RepeatInterval << NAN_TIME_BITMAP_CTRL_PERIOD_OFFSET) |
		(u4StartOffset << NAN_TIME_BITMAP_CTRL_STARTOFFSET_OFFSET);
	kalMemCopy(pu2TimeBitmapControl, &u2TimeBitmapControl, 2);

	u4CheckPos = u4StartOffset;
	u4BitmapPos1 = 0;
	while (u4CheckPos < (1 << (u4RepeatInterval + 2))) {
		fgBitSet = (pucBitmap[u4CheckPos / 8] & (BIT(u4CheckPos % 8)))
				   ? TRUE
				   : FALSE;
		if (fgBitSet) {
			pucTimeBitmapValue[u4BitmapPos1 / 8] |=
				(BIT(u4BitmapPos1 % 8));
			u4BitmapPos2 = u4BitmapPos1;
		}

		u4BitmapPos1++;
		u4CheckPos += (1 << u4BitDuration);
	}
	*pucTimeBitmapLength = 1 + u4BitmapPos2 / 8;
#if CFG_NAN_SIGMA_TEST
	if (*pucTimeBitmapLength < 4)
		*pucTimeBitmapLength = 4;
#endif

	*pu4TimeBitmapFieldLength = *pucTimeBitmapLength + 3;
	kalMemMove(pucTimeBitmapField, aucTimeBitmapField,
		   *pu4TimeBitmapFieldLength);

	DBGLOG(NAN, LOUD,
	       "BitDur:%d, RepeatPeriod:%d, StartOffset:%d, len:%d\n",
	       u4BitDuration, u4RepeatInterval, u4StartOffset,
	       *pucTimeBitmapLength);

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanParserGenChnlEntryField(struct ADAPTER *prAdapter,
			   union _NAN_BAND_CHNL_CTRL *prChnlCtrl,
			   struct _NAN_CHNL_ENTRY_T *prChnlEntry) {
	uint32_t u4Bw;
	uint8_t ucPrimaryChnl;
	uint8_t ucOperatingClass;
	uint8_t ucCenterChnl = 0;
	uint8_t ucChnlLowerBound = 0;

	ucOperatingClass = prChnlCtrl->rChannel.u4OperatingClass;
	ucPrimaryChnl = prChnlCtrl->rChannel.u4PrimaryChnl;

	u4Bw = nanRegGetBw(ucOperatingClass);
	prChnlEntry->ucOperatingClass = ucOperatingClass;

	if (u4Bw == 20 || u4Bw == 40) {
		prChnlEntry->ucPrimaryChnlBitmap = 0;
		prChnlEntry->u2AuxChannelBitmap = 0;
		nanRegGetChannelBitmap(ucOperatingClass, ucPrimaryChnl,
				       &prChnlEntry->u2ChannelBitmap);
	} else if (u4Bw == 80) {
		if (ucPrimaryChnl >= 36 && ucPrimaryChnl <= 48) {
			ucChnlLowerBound = 36;
			ucCenterChnl = 42;
		} else if (ucPrimaryChnl >= 52 && ucPrimaryChnl <= 64) {
			ucChnlLowerBound = 52;
			ucCenterChnl = 58;
		} else if (ucPrimaryChnl >= 100 && ucPrimaryChnl <= 112) {
			ucChnlLowerBound = 100;
			ucCenterChnl = 106;
		} else if (ucPrimaryChnl >= 116 && ucPrimaryChnl <= 128) {
			ucChnlLowerBound = 116;
			ucCenterChnl = 122;
		} else if (ucPrimaryChnl >= 132 && ucPrimaryChnl <= 144) {
			ucChnlLowerBound = 132;
			ucCenterChnl = 138;
		} else if (ucPrimaryChnl >= 149 && ucPrimaryChnl <= 161) {
			ucChnlLowerBound = 149;
			ucCenterChnl = 155;
		}
		prChnlEntry->ucPrimaryChnlBitmap =
			(1 << ((ucPrimaryChnl - ucChnlLowerBound) / 4));
		nanRegGetChannelBitmap(ucOperatingClass, ucCenterChnl,
				       &prChnlEntry->u2ChannelBitmap);

		if (prChnlCtrl->rChannel.u4AuxCenterChnl == 0) {
			prChnlEntry->u2AuxChannelBitmap = 0;
		} else {
			nanRegGetChannelBitmap(
				ucOperatingClass,
				prChnlCtrl->rChannel.u4AuxCenterChnl,
				&prChnlEntry->u2AuxChannelBitmap);
		}
	} else if (u4Bw == 160) {
		if (ucPrimaryChnl >= 36 && ucPrimaryChnl <= 64) {
			ucChnlLowerBound = 36;
			ucCenterChnl = 50;
		} else if (ucPrimaryChnl >= 100 && ucPrimaryChnl <= 128) {
			ucChnlLowerBound = 100;
			ucCenterChnl = 114;
		}
		prChnlEntry->ucPrimaryChnlBitmap =
			(1 << ((ucPrimaryChnl - ucChnlLowerBound) / 4));
		prChnlEntry->u2AuxChannelBitmap = 0;
		nanRegGetChannelBitmap(ucOperatingClass, ucCenterChnl,
				       &prChnlEntry->u2ChannelBitmap);
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanParserGenBandChnlEntryListField(
	struct ADAPTER *prAdapter,
	union _NAN_BAND_CHNL_CTRL *prBandChnlListCtrl, uint32_t u4NumOfList,
	uint8_t *pucBandChnlEntryListField,
	uint32_t *pu4BandChnlEntryListFieldLength) {
	struct _NAN_BAND_CHNL_LIST_T *prBandChnlList;
	struct _NAN_CHNL_ENTRY_T *prChnlEntry;
	uint8_t *pucPos;
	uint32_t u4BandIdMask;

	pucPos = pucBandChnlEntryListField;

	if (prBandChnlListCtrl->rBand.u4Type ==
	    NAN_BAND_CH_ENTRY_LIST_TYPE_BAND) {

		if (u4NumOfList != 1) {
			DBGLOG(NAN, ERROR, "u4NumOfList doesn't equal 1!\n");
			return WLAN_STATUS_FAILURE;
		}

		prBandChnlList = (struct _NAN_BAND_CHNL_LIST_T *)pucPos;
		prBandChnlList->ucNonContiguous = 0;
		prBandChnlList->ucType = NAN_BAND_CH_ENTRY_LIST_TYPE_BAND;
		prBandChnlList->ucRsvd = 0;
		prBandChnlList->ucNumberOfEntry = 0;
		pucPos += 1;

		u4BandIdMask = prBandChnlListCtrl->rBand.u4BandIdMask;
		if (u4BandIdMask & BIT(NAN_SUPPORTED_BAND_ID_2P4G)) {
			*pucPos = NAN_SUPPORTED_BAND_ID_2P4G;
			prBandChnlList->ucNumberOfEntry++;
			pucPos++;
		}
		if (u4BandIdMask & BIT(NAN_SUPPORTED_BAND_ID_5G)) {
			*pucPos = NAN_SUPPORTED_BAND_ID_5G;
			prBandChnlList->ucNumberOfEntry++;
			pucPos++;
		}

		*pu4BandChnlEntryListFieldLength =
			(pucPos - pucBandChnlEntryListField);
	} else {
		prBandChnlList = (struct _NAN_BAND_CHNL_LIST_T *)pucPos;
		prBandChnlList->ucNonContiguous = 0; /* Fixme */
		prBandChnlList->ucType =
			NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL;
		prBandChnlList->ucRsvd = 0;
		prBandChnlList->ucNumberOfEntry = 0;
		pucPos += 1;

		for (; u4NumOfList > 0;
		     u4NumOfList--, prBandChnlListCtrl += 1) {
			if (prBandChnlListCtrl->rBand.u4Type !=
			    NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL)
				continue;

			prChnlEntry = (struct _NAN_CHNL_ENTRY_T *)pucPos;
			nanParserGenChnlEntryField(
				prAdapter, prBandChnlListCtrl, prChnlEntry);

			prBandChnlList->ucNumberOfEntry++;

			if (prBandChnlList->ucNonContiguous == 0)
				pucPos += (sizeof(struct _NAN_CHNL_ENTRY_T) -
					   2 /* Aux channel bitmap */);
			else
				pucPos += sizeof(struct _NAN_CHNL_ENTRY_T);
		}

		*pu4BandChnlEntryListFieldLength =
			(pucPos - pucBandChnlEntryListField);
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanParserInterpretTimeBitmapField(struct ADAPTER *prAdapter,
		  uint16_t u2TimeBitmapCtrl,
		  uint8_t ucTimeBitmapLength,
		  uint8_t *pucTimeBitmap, uint32_t *pu4AvailMap) {
	uint32_t u4Duration;
	uint32_t u4Period;
	uint32_t u4StartOffset;
	uint32_t u4Repeat, u4Bit, u4Num;
	uint32_t u4BitPos;
	uint8_t ucLength;
	uint8_t *pucBitmap;
	uint32_t u4Run;

	u4Duration = (u2TimeBitmapCtrl & NAN_TIME_BITMAP_CTRL_DURATION) >>
		     NAN_TIME_BITMAP_CTRL_DURATION_OFFSET;
	u4Duration = 1 << u4Duration;
	u4Period = (u2TimeBitmapCtrl & NAN_TIME_BITMAP_CTRL_PERIOD) >>
		   NAN_TIME_BITMAP_CTRL_PERIOD_OFFSET;
	u4Period = 1 << (u4Period + 2);
	u4StartOffset = (u2TimeBitmapCtrl & NAN_TIME_BITMAP_CTRL_STARTOFFSET) >>
			NAN_TIME_BITMAP_CTRL_STARTOFFSET_OFFSET;

	DBGLOG(NAN, LOUD,
	       "BitDur:%d, RepeatPeriod:%d, StartOffset:%d, len:%d\n",
	       u4Duration, u4Period, u4StartOffset, ucTimeBitmapLength);

	u4Repeat = NAN_TOTAL_SLOT_WINDOWS / u4Period;
	u4Run = 0;

	do {
		u4BitPos = u4Run * u4Period + u4StartOffset;
		pucBitmap = pucTimeBitmap;
		ucLength = ucTimeBitmapLength;
		if (ucLength > (u4Period / 8))
			ucLength = u4Period / 8;

		do {
			for (u4Bit = 0; u4Bit < 8; u4Bit++) {
				if (*pucBitmap & BIT(u4Bit)) {
					for (u4Num = 0;
					     (u4Num < u4Duration) &&
					     (u4BitPos <
					      NAN_TOTAL_SLOT_WINDOWS);
					     u4Num++) {
						NAN_TIMELINE_SET(pu4AvailMap,
								 u4BitPos);

						u4BitPos++;
					}
				} else
					u4BitPos += u4Duration;
			}

			pucBitmap++;
			ucLength--;
		} while (ucLength);

		u4Run++;
	} while ((u4BitPos < NAN_TOTAL_SLOT_WINDOWS) && (u4Run < u4Repeat));

	return WLAN_STATUS_SUCCESS;
}

union _NAN_BAND_CHNL_CTRL
nanQueryPeerPotentialChnlInfoBySlot(
		struct ADAPTER *prAdapter, uint32_t u4SchIdx,
		uint32_t u4AvailDbIdx, uint16_t u2SlotIdx) {
	uint32_t u4Idx;
	uint32_t u4AvailType;
	struct _NAN_AVAILABILITY_DB_T *prAvailabilityDB;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntry;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	uint32_t u4ChnlIdx;
	union _NAN_BAND_CHNL_CTRL rSelChnl = g_rNullChnl;
	union _NAN_BAND_CHNL_CTRL *prCurrChnl;

	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, u4SchIdx);
	if (prPeerSchDesc == NULL)
		return g_rNullChnl;

	prAvailabilityDB = &prPeerSchDesc->arAvailAttr[u4AvailDbIdx];
	DBGLOG(NAN, LOUD, "ucAvailabilityDbIdx:%d, MapID:%d\n", u4AvailDbIdx,
	       prAvailabilityDB->ucMapId);

	if (prAvailabilityDB->ucMapId == NAN_INVALID_MAP_ID)
		return g_rNullChnl;

	for (u4Idx = 0; (u4Idx < NAN_NUM_AVAIL_TIMELINE); u4Idx++) {
		prNanAvailEntry = &prAvailabilityDB->arAvailEntryList[u4Idx];

		if (prNanAvailEntry->fgActive == FALSE)
			continue;

		if (!NAN_IS_AVAIL_MAP_SET(prNanAvailEntry->au4AvailMap,
					  u2SlotIdx))
			continue;

		if (prNanAvailEntry->arBandChnlCtrl[0].rChannel.u4Type ==
		    NAN_BAND_CH_ENTRY_LIST_TYPE_BAND)
			continue;

		u4AvailType =
			NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_POTN;
		if ((prNanAvailEntry->rEntryCtrl.rField.u2Type & u4AvailType) ==
		    0) {
			DBGLOG(NAN, LOUD,
			       "Entry:%d, Slot:%d, Type:%d not equal\n", u4Idx,
			       u2SlotIdx,
			       prNanAvailEntry->rEntryCtrl.rField.u2Type);
			continue;
		}

		prCurrChnl = &prNanAvailEntry->arBandChnlCtrl[0];
		for (u4ChnlIdx = 0;
		     u4ChnlIdx < prNanAvailEntry->ucNumBandChnlCtrl;
		     u4ChnlIdx++) {
			if (nanIsAllowedChannel(
				    prAdapter,
				    prCurrChnl->rChannel.u4PrimaryChnl)) {
				if (rSelChnl.rChannel.u4PrimaryChnl == 0)
					rSelChnl = *prCurrChnl;

				if (prCurrChnl->rChannel.u4PrimaryChnl < 36) {
					if (prCurrChnl->rChannel
						    .u4PrimaryChnl ==
					    g_r2gDwChnl.rChannel.u4PrimaryChnl)
						return *prCurrChnl;
				} else {
					if (prCurrChnl->rChannel
						    .u4PrimaryChnl ==
					    g_r5gDwChnl.rChannel.u4PrimaryChnl)
						return *prCurrChnl;
				}
			}
		}
	}

	return rSelChnl;
}

union _NAN_BAND_CHNL_CTRL
nanGetPeerPotentialChnlInfoBySlot(struct ADAPTER *prAdapter,
		uint32_t u4SchIdx, uint16_t u2SlotIdx) {
	uint32_t u4AvailDbIdx;
	union _NAN_BAND_CHNL_CTRL rSelChnlInfo;

	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
	     u4AvailDbIdx++) {
		rSelChnlInfo = nanQueryPeerPotentialChnlInfoBySlot(
			prAdapter, u4SchIdx, u4AvailDbIdx, u2SlotIdx);
		if (rSelChnlInfo.rChannel.u4PrimaryChnl != 0)
			return rSelChnlInfo;
	}

	return g_rNullChnl;
}

union _NAN_BAND_CHNL_CTRL
nanQueryPeerPotentialBandInfoBySlot(struct ADAPTER *prAdapter,
		uint32_t u4SchIdx, uint32_t u4AvailDbIdx,
		uint16_t u2SlotIdx) {
	uint32_t u4Idx;
	struct _NAN_AVAILABILITY_DB_T *prAvailabilityDB;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntry;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, u4SchIdx);
	if (prPeerSchDesc == NULL)
		return g_rNullChnl;

	prAvailabilityDB = &prPeerSchDesc->arAvailAttr[u4AvailDbIdx];
	DBGLOG(NAN, LOUD, "ucAvailabilityDbIdx:%d, MapID:%d\n", u4AvailDbIdx,
	       prAvailabilityDB->ucMapId);

	if (prAvailabilityDB->ucMapId == NAN_INVALID_MAP_ID)
		return g_rNullChnl;

	for (u4Idx = 0; (u4Idx < NAN_NUM_AVAIL_TIMELINE); u4Idx++) {
		prNanAvailEntry = &prAvailabilityDB->arAvailEntryList[u4Idx];

		if (prNanAvailEntry->fgActive == FALSE)
			continue;

		if (!NAN_IS_AVAIL_MAP_SET(prNanAvailEntry->au4AvailMap,
					  u2SlotIdx))
			continue;

		if (prNanAvailEntry->arBandChnlCtrl[0].rChannel.u4Type ==
		    NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL)
			continue;

		return prNanAvailEntry->arBandChnlCtrl[0];
	}

	return g_rNullChnl;
}

union _NAN_BAND_CHNL_CTRL
nanGetPeerPotentialBandInfoBySlot(struct ADAPTER *prAdapter, uint32_t u4SchIdx,
				  uint16_t u2SlotIdx) {
	uint32_t u4AvailDbIdx;
	union _NAN_BAND_CHNL_CTRL rSelBandInfo;

	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
	     u4AvailDbIdx++) {
		rSelBandInfo = nanQueryPeerPotentialBandInfoBySlot(
			prAdapter, u4SchIdx, u4AvailDbIdx, u2SlotIdx);
		if (rSelBandInfo.rBand.u4BandIdMask != 0)
			return rSelBandInfo;
	}

	return g_rNullChnl;
}

union _NAN_BAND_CHNL_CTRL
nanQueryChnlInfoBySlot(struct ADAPTER *prAdapter, uint16_t u2SlotIdx,
		       uint32_t **ppau4AvailMap, unsigned char fgCommitOrCond) {
	uint32_t u4Idx;
	uint32_t u4Valid;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimelineList;
	struct _NAN_SCHEDULER_T *prScheduler;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	if (ppau4AvailMap != NULL)
		*ppau4AvailMap = NULL;

	prScheduler = nanGetScheduler(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	if (fgCommitOrCond) {
		if (prScheduler->fgEn2g &&
		    (u2SlotIdx % NAN_SLOTS_PER_DW_INTERVAL == NAN_2G_DW_INDEX))
			return g_r2gDwChnl;
		else if (prScheduler->fgEn5gH &&
			 (u2SlotIdx % NAN_SLOTS_PER_DW_INTERVAL ==
			  NAN_5G_DW_INDEX))
			return g_r5gDwChnl;
		else if (prScheduler->fgEn5gL &&
			 (u2SlotIdx % NAN_SLOTS_PER_DW_INTERVAL ==
			  NAN_5G_DW_INDEX))
			return g_r5gDwChnl;

		prChnlTimelineList = prNanTimelineMgmt->arChnlList;
	} else {
		if (prNanTimelineMgmt->fgChkCondAvailability == FALSE)
			return g_rNullChnl;

		prChnlTimelineList = prNanTimelineMgmt->arCondChnlList;
	}

	/* FAW channel list */
	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx++) {
		if (prChnlTimelineList[u4Idx].fgValid == FALSE)
			continue;

		u4Valid = NAN_IS_AVAIL_MAP_SET(
			prChnlTimelineList[u4Idx].au4AvailMap, u2SlotIdx);
		if (u4Valid) {
			if (ppau4AvailMap != NULL)
				*ppau4AvailMap = &prChnlTimelineList[u4Idx]
							  .au4AvailMap[0];

			return prChnlTimelineList[u4Idx].rChnlInfo;
		}
	}

	return g_rNullChnl;
}

uint8_t
nanQueryPrimaryChnlBySlot(struct ADAPTER *prAdapter, uint16_t u2SlotIdx,
			  unsigned char fgCommitOrCond) {
	union _NAN_BAND_CHNL_CTRL rChnlInfo;

	rChnlInfo = nanQueryChnlInfoBySlot(prAdapter, u2SlotIdx, NULL,
					   fgCommitOrCond);
	return rChnlInfo.rChannel.u4PrimaryChnl;
}

union _NAN_BAND_CHNL_CTRL
nanGetChnlInfoBySlot(struct ADAPTER *prAdapter, uint16_t u2SlotIdx) {
	union _NAN_BAND_CHNL_CTRL rChnlInfo;

	rChnlInfo = nanQueryChnlInfoBySlot(prAdapter, u2SlotIdx, NULL, TRUE);
	if (rChnlInfo.rChannel.u4PrimaryChnl == 0)
		rChnlInfo = nanQueryChnlInfoBySlot(prAdapter, u2SlotIdx, NULL,
						   FALSE);

	return rChnlInfo;
}

uint32_t
nanGetPrimaryChnlBySlot(struct ADAPTER *prAdapter, uint16_t u2SlotIdx) {
	union _NAN_BAND_CHNL_CTRL rChnlInfo;

	rChnlInfo = nanGetChnlInfoBySlot(prAdapter, u2SlotIdx);

	return rChnlInfo.rChannel.u4PrimaryChnl;
}

union _NAN_BAND_CHNL_CTRL
nanQueryPeerChnlInfoBySlot(struct ADAPTER *prAdapter, uint32_t u4SchIdx,
			   uint32_t u4AvailDbIdx, uint16_t u2SlotIdx,
			   unsigned char fgCommitOrCond) {
	uint32_t u4Idx;
	uint32_t u4AvailType;
	struct _NAN_AVAILABILITY_DB_T *prAvailabilityDB;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntry;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, u4SchIdx);
	if (prPeerSchDesc == NULL)
		return g_rNullChnl;

	prAvailabilityDB = &prPeerSchDesc->arAvailAttr[u4AvailDbIdx];
	DBGLOG(NAN, LOUD, "ucAvailabilityDbIdx:%d, MapID:%d\n", u4AvailDbIdx,
	       prAvailabilityDB->ucMapId);

	if (prAvailabilityDB->ucMapId == NAN_INVALID_MAP_ID)
		return g_rNullChnl;

	for (u4Idx = 0; (u4Idx < NAN_NUM_AVAIL_TIMELINE); u4Idx++) {
		prNanAvailEntry = &prAvailabilityDB->arAvailEntryList[u4Idx];

		if (prNanAvailEntry->fgActive == FALSE)
			continue;

		if (!NAN_IS_AVAIL_MAP_SET(prNanAvailEntry->au4AvailMap,
					  u2SlotIdx))
			continue;

		if (fgCommitOrCond)
			u4AvailType =
				NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COMMIT;
		else
			u4AvailType =
				NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COND;

		if ((prNanAvailEntry->rEntryCtrl.rField.u2Type & u4AvailType) ==
		    0) {
			DBGLOG(NAN, LOUD,
			       "Entry:%d, Slot:%d, Type:%d not equal\n", u4Idx,
			       u2SlotIdx,
			       prNanAvailEntry->rEntryCtrl.rField.u2Type);
			continue;
		}

		return prNanAvailEntry->arBandChnlCtrl[0];
	}

	return g_rNullChnl;
}

uint8_t
nanQueryPeerPrimaryChnlBySlot(struct ADAPTER *prAdapter, uint32_t u4SchIdx,
			      uint32_t u4AvailDbIdx, uint16_t u2SlotIdx,
			      unsigned char fgCommitOrCond) {
	union _NAN_BAND_CHNL_CTRL rChnlInfo;

	rChnlInfo = nanQueryPeerChnlInfoBySlot(
		prAdapter, u4SchIdx, u4AvailDbIdx, u2SlotIdx, fgCommitOrCond);
	return rChnlInfo.rChannel.u4PrimaryChnl;
}

union _NAN_BAND_CHNL_CTRL
nanGetPeerChnlInfoBySlot(struct ADAPTER *prAdapter, uint32_t u4SchIdx,
			 uint32_t u4AvailDbIdx, uint16_t u2SlotIdx,
			 unsigned char fgChkRmtCondSlot) {
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;

	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, u4SchIdx);
	if (prPeerSchDesc == NULL)
		return g_rNullChnl;

	rRmtChnlInfo = g_rNullChnl;
	if (u4AvailDbIdx == NAN_NUM_AVAIL_DB) {
		for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
		     u4AvailDbIdx++) {
			if (nanSchedPeerAvailabilityDbValidByDesc(
				    prAdapter, prPeerSchDesc, u4AvailDbIdx) ==
			    FALSE)
				continue;

			rRmtChnlInfo = nanQueryPeerChnlInfoBySlot(
				prAdapter, u4SchIdx, u4AvailDbIdx, u2SlotIdx,
				TRUE);
			if ((rRmtChnlInfo.rChannel.u4PrimaryChnl == 0) &&
			    fgChkRmtCondSlot) {
				rRmtChnlInfo = nanQueryPeerChnlInfoBySlot(
					prAdapter, u4SchIdx, u4AvailDbIdx,
					u2SlotIdx, FALSE);
			}
			if (rRmtChnlInfo.rChannel.u4PrimaryChnl != 0)
				break;
		}
	} else {
		rRmtChnlInfo = nanQueryPeerChnlInfoBySlot(
			prAdapter, u4SchIdx, u4AvailDbIdx, u2SlotIdx, TRUE);
		if ((rRmtChnlInfo.rChannel.u4PrimaryChnl == 0) &&
		    fgChkRmtCondSlot) {
			rRmtChnlInfo = nanQueryPeerChnlInfoBySlot(
				prAdapter, u4SchIdx, u4AvailDbIdx, u2SlotIdx,
				FALSE);
		}
	}

	return rRmtChnlInfo;
}

uint32_t
nanGetPeerPrimaryChnlBySlot(struct ADAPTER *prAdapter, uint32_t u4SchIdx,
			    uint32_t u4AvailDbIdx, uint16_t u2SlotIdx,
			    unsigned char fgChkRmtCondSlot) {
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;

	rRmtChnlInfo = nanGetPeerChnlInfoBySlot(
		prAdapter, u4SchIdx, u4AvailDbIdx, u2SlotIdx, fgChkRmtCondSlot);

	return rRmtChnlInfo.rChannel.u4PrimaryChnl;
}

uint8_t
nanGetPeerMaxBw(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	uint8_t ucBw = 20;
	uint32_t u4Idx;
	struct _NAN_AVAILABILITY_DB_T *prAvailabilityDB;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntry;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	uint32_t u4AvailDbIdx;
	uint32_t u4SlotIdx;
	uint32_t u4AvailType;

	do {
		prPeerSchDesc =
			nanSchedSearchPeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			DBGLOG(NAN, INFO, "Cann't find peer schedule desc\n");
			break;
		}

		for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
		     u4AvailDbIdx++) {
			prAvailabilityDB =
				&prPeerSchDesc->arAvailAttr[u4AvailDbIdx];

			if (prAvailabilityDB->ucMapId == NAN_INVALID_MAP_ID)
				continue;

			for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
			     u4SlotIdx++) {
				for (u4Idx = 0;
				     (u4Idx < NAN_NUM_AVAIL_TIMELINE);
				     u4Idx++) {
					prNanAvailEntry =
						&prAvailabilityDB
							 ->arAvailEntryList
								 [u4Idx];

					if (prNanAvailEntry->fgActive == FALSE)
						continue;

					if (!NAN_IS_AVAIL_MAP_SET(
						    prNanAvailEntry
							    ->au4AvailMap,
						    u4SlotIdx))
						continue;

					u4AvailType = (
					NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COMMIT |
					NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COND);
					if ((prNanAvailEntry->rEntryCtrl.rField
						     .u2Type &
					     u4AvailType) == 0)
						continue;

					if (nanRegGetBw(prNanAvailEntry
						->arBandChnlCtrl[0]
						.rChannel.u4OperatingClass) >
					    ucBw) {
						ucBw = nanRegGetBw(
						prNanAvailEntry
						->arBandChnlCtrl[0].rChannel
						.u4OperatingClass);
						break;
					}
				}
			}
		}
	} while (FALSE);

	return ucBw;
}

uint8_t
nanGetPeerMinBw(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	uint8_t ucBw = 80;
	uint32_t u4Idx;
	struct _NAN_AVAILABILITY_DB_T *prAvailabilityDB;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntry;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	uint32_t u4AvailDbIdx;
	uint32_t u4SlotIdx;
	uint32_t u4AvailType;

	do {
		prPeerSchDesc =
			nanSchedSearchPeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			DBGLOG(NAN, INFO, "Cann't find peer schedule desc\n");
			break;
		}

		for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
		     u4AvailDbIdx++) {
			prAvailabilityDB =
				&prPeerSchDesc->arAvailAttr[u4AvailDbIdx];

			if (prAvailabilityDB->ucMapId == NAN_INVALID_MAP_ID)
				continue;

			for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
			     u4SlotIdx++) {
				for (u4Idx = 0;
				     (u4Idx < NAN_NUM_AVAIL_TIMELINE);
				     u4Idx++) {
					prNanAvailEntry =
					&prAvailabilityDB->arAvailEntryList
					    [u4Idx];

					if (prNanAvailEntry->fgActive == FALSE)
						continue;

					if (!NAN_IS_AVAIL_MAP_SET(
					    prNanAvailEntry->au4AvailMap,
					    u4SlotIdx))
						continue;

					u4AvailType =
				    (NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COMMIT |
				     NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COND);
					if ((prNanAvailEntry->rEntryCtrl.rField
						.u2Type & u4AvailType) == 0)
						continue;

					if (nanRegGetBw(
					    prNanAvailEntry->arBandChnlCtrl[0]
					    .rChannel.u4OperatingClass) <
						ucBw) {
						ucBw = nanRegGetBw(
						prNanAvailEntry
						->arBandChnlCtrl[0]
						.rChannel.u4OperatingClass);
						break;
					}
				}
			}
		}
	} while (FALSE);

	return ucBw;
}

uint32_t
nanGetPeerDevCapability(struct ADAPTER *prAdapter,
			enum _ENUM_NAN_DEVCAP_FIELD_T eField,
			uint8_t *pucNmiAddr, uint8_t ucMapID,
			uint32_t *pu4RetValue) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	uint32_t u4AvailabilityDbIdx;
	struct _NAN_DEVICE_CAPABILITY_T *prNanDevCapability;

	do {
		prPeerSchDesc =
			nanSchedSearchPeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			DBGLOG(NAN, INFO, "Cann't find peer schedule desc\n");
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		if (ucMapID != NAN_INVALID_MAP_ID) {
			u4AvailabilityDbIdx = nanSchedPeerGetAvailabilityDbIdx(
				prAdapter, prPeerSchDesc, ucMapID);
			if (u4AvailabilityDbIdx == NAN_INVALID_AVAIL_DB_INDEX) {
				rRetStatus = WLAN_STATUS_FAILURE;
				break;
			}
		} else {
			u4AvailabilityDbIdx = NAN_NUM_AVAIL_DB;
		}

		prNanDevCapability =
			&prPeerSchDesc->arDevCapability[u4AvailabilityDbIdx];
		if (prNanDevCapability->fgValid == FALSE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		switch (eField) {
		case ENUM_NAN_DEVCAP_CAPABILITY_SET:
			if (pu4RetValue)
				*pu4RetValue =
					prNanDevCapability->ucCapabilitySet;
			break;

		case ENUM_NAN_DEVCAP_TX_ANT_NUM:
			if (pu4RetValue)
				*pu4RetValue = prNanDevCapability->ucNumTxAnt;
			break;

		case ENUM_NAN_DEVCAP_RX_ANT_NUM:
			if (pu4RetValue)
				*pu4RetValue = prNanDevCapability->ucNumRxAnt;
			break;

		default:
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}
	} while (FALSE);

	return rRetStatus;
}

unsigned char
nanGetFeaturePeerNDPE(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	struct _NAN_DEVICE_CAPABILITY_T *prNanDevCapability;
	uint32_t u4Idx;

	do {
		prPeerSchDesc =
			nanSchedSearchPeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			DBGLOG(NAN, INFO, "Cann't find peer schedule desc\n");
			break;
		}

		for (u4Idx = 0; u4Idx < (NAN_NUM_AVAIL_DB + 1); u4Idx++) {
			prNanDevCapability =
				&prPeerSchDesc->arDevCapability[u4Idx];
			if (prNanDevCapability->fgValid == FALSE)
				continue;
			else if (
				(prNanDevCapability->ucCapabilitySet &
				 NAN_ATTR_DEVICE_CAPABILITY_CAP_SUPPORT_NDPE) !=
				0)
				return TRUE;
		}
	} while (FALSE);

	return FALSE;
}

unsigned char
nanGetFeatureNDPE(struct ADAPTER *prAdapter) {
	return prAdapter->rWifiVar.fgEnableNDPE;
}

uint32_t
nanSchedDbgDumpTimelineDb(struct ADAPTER *prAdapter, const char *pucFunction,
			  uint32_t u4Line) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Idx;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimeline;

	DBGLOG(NAN, INFO, "\n");
	DBGLOG(NAN, INFO, "Dump timeline DB [%s:%d]\n", pucFunction, u4Line);

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx++) {
		prChnlTimeline = &prNanTimelineMgmt->arChnlList[u4Idx];
		if (prChnlTimeline->fgValid == FALSE)
			continue;

		DBGLOG(NAN, INFO,
		       "[%d] Raw:0x%x, Commit Chnl:%d, Class:%d, Bw:%d\n",
		       u4Idx, prChnlTimeline->rChnlInfo.u4RawData,
		       prChnlTimeline->rChnlInfo.rChannel.u4PrimaryChnl,
		       prChnlTimeline->rChnlInfo.rChannel.u4OperatingClass,
		       nanRegGetBw(prChnlTimeline->rChnlInfo.rChannel
					   .u4OperatingClass));
		nanUtilDump(prAdapter, "AvailMap",
			    (uint8_t *)prChnlTimeline->au4AvailMap,
			    sizeof(prChnlTimeline->au4AvailMap));
	}

	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx++) {
		if (prNanTimelineMgmt->fgChkCondAvailability == FALSE)
			break;

		prChnlTimeline = &prNanTimelineMgmt->arCondChnlList[u4Idx];
		if (prChnlTimeline->fgValid == FALSE)
			continue;

		DBGLOG(NAN, INFO,
		       "[%d] Raw:0x%x, Cond Chnl:%d, Class:%d, Bw:%d\n", u4Idx,
		       prChnlTimeline->rChnlInfo.u4RawData,
		       prChnlTimeline->rChnlInfo.rChannel.u4PrimaryChnl,
		       prChnlTimeline->rChnlInfo.rChannel.u4OperatingClass,
		       nanRegGetBw(prChnlTimeline->rChnlInfo.rChannel
					   .u4OperatingClass));
		nanUtilDump(prAdapter, "AvailMap",
			    (uint8_t *)prChnlTimeline->au4AvailMap,
			    sizeof(prChnlTimeline->au4AvailMap));
	}

	return rRetStatus;
}

uint32_t
nanSchedDbgDumpPeerAvailability(struct ADAPTER *prAdapter,
		uint8_t *pucNmiAddr) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Idx, u4Idx1, u4Idx2;
	struct _NAN_AVAILABILITY_DB_T *prNanAvailAttr;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntry;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prPeerSchDesc = nanSchedSearchPeerSchDescByNmi(prAdapter, pucNmiAddr);
	if (prPeerSchDesc == NULL) {
		DBGLOG(NAN, INFO, "Cann't find peer schedule desc\n");
		return WLAN_STATUS_FAILURE;
	}

	DBGLOG(NAN, INFO, "\n");
	DBGLOG(NAN, INFO, "Dump %02x:%02x:%02x:%02x:%02x:%02x Availability\n",
	       prPeerSchDesc->aucNmiAddr[0], prPeerSchDesc->aucNmiAddr[1],
	       prPeerSchDesc->aucNmiAddr[2], prPeerSchDesc->aucNmiAddr[3],
	       prPeerSchDesc->aucNmiAddr[4], prPeerSchDesc->aucNmiAddr[5]);

	for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
		prNanAvailAttr = &prPeerSchDesc->arAvailAttr[u4Idx];
		if (prNanAvailAttr->ucMapId == NAN_INVALID_MAP_ID)
			continue;

		for (u4Idx1 = 0; u4Idx1 < NAN_NUM_AVAIL_TIMELINE; u4Idx1++) {
			prNanAvailEntry =
				&prNanAvailAttr->arAvailEntryList[u4Idx1];
			if (prNanAvailEntry->fgActive == FALSE)
				continue;

			if (prNanAvailEntry->arBandChnlCtrl[0].rInfo.u4Type ==
			    NAN_BAND_CH_ENTRY_LIST_TYPE_BAND)
				continue;

			DBGLOG(NAN, INFO,
			       "[%d][%d] MapID:%d, Ctrl:0x%x, ChnlRaw:0x%x, Class:%d, Bw:%d\n",
			       u4Idx, u4Idx1, prNanAvailAttr->ucMapId,
			       prNanAvailEntry->rEntryCtrl.u2RawData,
			       prNanAvailEntry->arBandChnlCtrl[0].u4RawData,
			       prNanAvailEntry->arBandChnlCtrl[0]
				       .rChannel.u4OperatingClass,
			       nanRegGetBw(prNanAvailEntry->arBandChnlCtrl[0]
						   .rChannel.u4OperatingClass));
			for (u4Idx2 = 0;
			     u4Idx2 < prNanAvailEntry->ucNumBandChnlCtrl;
			     u4Idx2++)
				DBGLOG(NAN, INFO, "[%d] PriChnl:%u\n", u4Idx2,
				       prNanAvailEntry->arBandChnlCtrl[u4Idx2]
					       .rChannel.u4PrimaryChnl);
			nanUtilDump(prAdapter, "[Map]",
				    (uint8_t *)prNanAvailEntry->au4AvailMap,
				    sizeof(prNanAvailEntry->au4AvailMap));
		}
	}

	return rRetStatus;
}

struct _NAN_NDC_CTRL_T *
nanSchedAcquireNdcCtrl(struct ADAPTER *prAdapter) {
	uint32_t u4Idx;
	struct _NAN_SCHEDULER_T *prScheduler;

	prScheduler = nanGetScheduler(prAdapter);

	for (u4Idx = 0; u4Idx < NAN_MAX_NDC_RECORD; u4Idx++) {
		if (prScheduler->arNdcCtrl[u4Idx].fgValid == FALSE)
			return &prScheduler->arNdcCtrl[u4Idx];
	}

	return NULL;
}

struct _NAN_NDC_CTRL_T *
nanSchedGetNdcCtrl(struct ADAPTER *prAdapter, uint8_t *pucNdcId) {
	uint32_t u4Idx;
	struct _NAN_SCHEDULER_T *prScheduler;

	prScheduler = nanGetScheduler(prAdapter);

	for (u4Idx = 0; u4Idx < NAN_MAX_NDC_RECORD; u4Idx++) {
		if (prScheduler->arNdcCtrl[u4Idx].fgValid == FALSE)
			continue;

		if ((pucNdcId == NULL) ||
		    (kalMemCmp(prScheduler->arNdcCtrl[u4Idx].aucNdcId, pucNdcId,
			       NAN_NDC_ATTRIBUTE_ID_LENGTH) == 0))
			return &prScheduler->arNdcCtrl[u4Idx];
	}

	return NULL;
}

enum _ENUM_CNM_CH_CONCURR_T
nanSchedChkConcurrOp(union _NAN_BAND_CHNL_CTRL rCurrChnlInfo,
		     union _NAN_BAND_CHNL_CTRL rNewChnlInfo) {
	uint8_t ucPrimaryChNew;
	enum ENUM_CHANNEL_WIDTH eChannelWidthNew;
	enum ENUM_CHNL_EXT eSCONew;
	uint8_t ucChannelS1New;
	uint8_t ucChannelS2New;
	uint8_t ucPrimaryChCurr;
	enum ENUM_CHANNEL_WIDTH eChannelWidthCurr;
	enum ENUM_CHNL_EXT eSCOCurr;
	uint8_t ucChannelS1Curr;
	uint8_t ucChannelS2Curr;

	if (nanRegConvertNanChnlInfo(rCurrChnlInfo, &ucPrimaryChCurr,
				     &eChannelWidthCurr, &eSCOCurr,
				     &ucChannelS1Curr,
				     &ucChannelS2Curr) != WLAN_STATUS_SUCCESS)
		return CNM_CH_CONCURR_MCC;

	if (nanRegConvertNanChnlInfo(
		    rNewChnlInfo, &ucPrimaryChNew, &eChannelWidthNew, &eSCONew,
		    &ucChannelS1New, &ucChannelS2New) != WLAN_STATUS_SUCCESS)
		return CNM_CH_CONCURR_MCC;

	return cnmChConCurrType(ucPrimaryChNew, eChannelWidthNew, eSCONew,
				ucChannelS1New, ucChannelS2New, ucPrimaryChCurr,
				eChannelWidthCurr, eSCOCurr, ucChannelS1Curr,
				ucChannelS2Curr);
}

struct _NAN_CHANNEL_TIMELINE_T *
nanSchedAcquireChnlTimeline(struct ADAPTER *prAdapter,
			    struct _NAN_CHANNEL_TIMELINE_T *prChnlTimelineList,
			    union _NAN_BAND_CHNL_CTRL *prChnlInfo) {
	uint32_t u4Idx;
	uint32_t u4InvalidIdx = NAN_TIMELINE_MGMT_CHNL_LIST_NUM;
	enum _ENUM_CNM_CH_CONCURR_T eConcurr;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimeline;

	prChnlTimeline = &prChnlTimelineList[0];
	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM;
	     u4Idx++, prChnlTimeline++) {
		if (prChnlTimeline->fgValid == FALSE) {
			if (u4InvalidIdx == NAN_TIMELINE_MGMT_CHNL_LIST_NUM)
				u4InvalidIdx = u4Idx;

			continue;
		}

		eConcurr = nanSchedChkConcurrOp(prChnlTimeline->rChnlInfo,
						*prChnlInfo);
		if (eConcurr == CNM_CH_CONCURR_MCC)
			continue;
		else if (eConcurr == CNM_CH_CONCURR_SCC_NEW)
			prChnlTimeline->rChnlInfo.u4RawData =
				prChnlInfo->u4RawData;

		return prChnlTimeline;
	}

	if (u4InvalidIdx != NAN_TIMELINE_MGMT_CHNL_LIST_NUM) {
		prChnlTimelineList[u4InvalidIdx].fgValid = TRUE;
		prChnlTimelineList[u4InvalidIdx].rChnlInfo.u4RawData =
			prChnlInfo->u4RawData;
		prChnlTimelineList[u4InvalidIdx].i4Num = 0;
		kalMemZero(
			prChnlTimelineList[u4InvalidIdx].au4AvailMap,
			sizeof(prChnlTimelineList[u4InvalidIdx].au4AvailMap));

		return &prChnlTimelineList[u4InvalidIdx];
	}

	return NULL;
}

union _NAN_BAND_CHNL_CTRL
nanSchedConvergeChnlInfo(struct ADAPTER *prAdapter,
			 union _NAN_BAND_CHNL_CTRL rChnlInfo) {
	union _NAN_BAND_CHNL_CTRL rSelChnlInfo;
	uint32_t u4MaxAllowedBw;

	u4MaxAllowedBw = nanSchedConfigGetAllowedBw(
		prAdapter, rChnlInfo.rChannel.u4PrimaryChnl);
	if (nanRegGetBw(rChnlInfo.rChannel.u4OperatingClass) !=
	    u4MaxAllowedBw) {
		rSelChnlInfo = nanRegGenNanChnlInfoByPriChannel(
			rChnlInfo.rChannel.u4PrimaryChnl, u4MaxAllowedBw);
	} else {
		rSelChnlInfo = rChnlInfo;
	}

	return rSelChnlInfo;
}

uint8_t
nanSchedChooseBestFromChnlBitmap(struct ADAPTER *prAdapter,
		 uint8_t ucOperatingClass,
		 uint16_t *pu2ChnlBitmap, unsigned char fgNonContBw,
		 uint8_t ucPriChnlBitmap) {
	uint8_t ucChnl;
	uint8_t ucFirstChnl;
	uint8_t ucPriChnl;

	ucPriChnl = ucFirstChnl = 0;
	do {
		ucChnl = nanRegGetPrimaryChannelByOrder(
			ucOperatingClass, pu2ChnlBitmap, fgNonContBw,
			ucPriChnlBitmap);
		if (ucChnl == REG_INVALID_INFO)
			break;

		if (ucFirstChnl == 0)
			ucFirstChnl = ucChnl;

		if (!nanIsAllowedChannel(prAdapter, ucChnl))
			continue;

		if (ucPriChnl == 0)
			ucPriChnl = ucChnl;

		if (ucChnl < 36) {
			if (ucChnl == g_r2gDwChnl.rChannel.u4PrimaryChnl) {
				ucPriChnl = ucChnl;
				break;
			}
		} else {
			if (ucChnl == g_r5gDwChnl.rChannel.u4PrimaryChnl) {
				ucPriChnl = ucChnl;
				break;
			}
		}
	} while (TRUE);

	if (ucPriChnl == 0)
		ucPriChnl = ucFirstChnl;

	return ucPriChnl;
}

union _NAN_BAND_CHNL_CTRL
nanSchedGetFixedChnlInfo(struct ADAPTER *prAdapter) {
	uint32_t u4Bw;
	union _NAN_BAND_CHNL_CTRL rSelChnlInfo = g_rNullChnl;

	if (prAdapter->rWifiVar.ucNanFixChnl != 0) {
		u4Bw = nanSchedConfigGetAllowedBw(
			prAdapter, prAdapter->rWifiVar.ucNanFixChnl);
		rSelChnlInfo = nanRegGenNanChnlInfoByPriChannel(
			prAdapter->rWifiVar.ucNanFixChnl, u4Bw);
	}

	return rSelChnlInfo;
}

uint32_t
nanSchedMergeAvailabileChnlList(struct ADAPTER *prAdapter,
				unsigned char fgCommitOrCond) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Idx1, u4Idx2;
	uint32_t u4DwIdx;
	enum _ENUM_CNM_CH_CONCURR_T eConcurr;
	union _NAN_BAND_CHNL_CTRL *prChnl1, *prChnl2;
	union _NAN_BAND_CHNL_CTRL rTargetChnl;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimelineList;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	if (fgCommitOrCond) {
		prChnlTimelineList = prNanTimelineMgmt->arChnlList;
	} else {
		if (prNanTimelineMgmt->fgChkCondAvailability == FALSE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			goto MERGE_CHNL_DONE;
		}

		prChnlTimelineList = prNanTimelineMgmt->arCondChnlList;
	}

	for (u4Idx1 = 0; u4Idx1 < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx1++) {
		if (prChnlTimelineList[u4Idx1].fgValid == FALSE)
			continue;

		prChnl1 = &prChnlTimelineList[u4Idx1].rChnlInfo;

		for (u4Idx2 = u4Idx1 + 1;
		     u4Idx2 < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx2++) {
			if (prChnlTimelineList[u4Idx2].fgValid == FALSE)
				continue;

			prChnl2 = &prChnlTimelineList[u4Idx2].rChnlInfo;

			eConcurr = nanSchedChkConcurrOp(*prChnl1, *prChnl2);
			if (eConcurr == CNM_CH_CONCURR_MCC)
				continue;
			else if (eConcurr == CNM_CH_CONCURR_SCC_NEW)
				rTargetChnl.u4RawData = prChnl2->u4RawData;
			else /* CNM_CH_CONCURR_SCC_CURR */
				rTargetChnl.u4RawData = prChnl1->u4RawData;

			prChnl1->u4RawData = rTargetChnl.u4RawData;
			for (u4DwIdx = 0; u4DwIdx < NAN_TOTAL_DW; u4DwIdx++)
				prChnlTimelineList[u4Idx1]
					.au4AvailMap[u4DwIdx] |=
					prChnlTimelineList[u4Idx2]
						.au4AvailMap[u4DwIdx];
			prChnlTimelineList[u4Idx1].i4Num =
			    nanUtilCheckBitOneCnt(
				prAdapter, (uint8_t *)
				prChnlTimelineList[u4Idx1].au4AvailMap,
				sizeof(prChnlTimelineList[u4Idx1].au4AvailMap));
			prChnlTimelineList[u4Idx2].fgValid = FALSE;
		}
	}

MERGE_CHNL_DONE:

	return rRetStatus;
}

uint32_t
nanSchedAddCrbToChnlList(struct ADAPTER *prAdapter,
			 union _NAN_BAND_CHNL_CTRL *prChnlInfo,
			 uint32_t u4StartOffset, uint32_t u4NumSlots,
			 enum _ENUM_TIME_BITMAP_CTRL_PERIOD_T eRepeatPeriod,
			 unsigned char fgCommitOrCond,
			 uint32_t au4RetMap[NAN_TOTAL_DW]) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Run, u4Idx, u4SlotIdx;
	unsigned char fgChanged = FALSE;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_SCHEDULER_T *prNanScheduler;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimelineList;
	struct _NAN_CHANNEL_TIMELINE_T *prTargetChnlTimeline;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNanScheduler = nanGetScheduler(prAdapter);

	do {
		if (fgCommitOrCond) {
			prChnlTimelineList = prNanTimelineMgmt->arChnlList;
		} else {
			if (prNanTimelineMgmt->fgChkCondAvailability == FALSE) {
				rRetStatus = WLAN_STATUS_FAILURE;
				break;
			}

			prChnlTimelineList = prNanTimelineMgmt->arCondChnlList;
		}

		prTargetChnlTimeline = nanSchedAcquireChnlTimeline(
			prAdapter, prChnlTimelineList, prChnlInfo);
		if (prTargetChnlTimeline == NULL) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		for (u4Run = 0; u4Run < (128 >> eRepeatPeriod); u4Run++) {
			for (u4Idx = 0; u4Idx < u4NumSlots; u4Idx++) {
				u4SlotIdx = (u4Run << (eRepeatPeriod + 2)) +
					    u4StartOffset + u4Idx;

				if (NAN_IS_AVAIL_MAP_SET(
					    prTargetChnlTimeline->au4AvailMap,
					    u4SlotIdx) == FALSE) {
					prTargetChnlTimeline->i4Num++;
					NAN_TIMELINE_SET(prTargetChnlTimeline
								 ->au4AvailMap,
							 u4SlotIdx);
					fgChanged = TRUE;
				}

				if (au4RetMap)
					NAN_TIMELINE_SET(au4RetMap, u4SlotIdx);
			}
		}

		if (fgChanged || !fgCommitOrCond) {
			prNanScheduler->u2NanAvailAttrControlField |=
				NAN_AVAIL_CTRL_COMMIT_CHANGED;
		}
	} while (FALSE);

	return rRetStatus;
}

uint32_t
nanSchedDeleteCrbFromChnlList(
	struct ADAPTER *prAdapter, uint32_t u4StartOffset, uint32_t u4NumSlots,
	enum _ENUM_TIME_BITMAP_CTRL_PERIOD_T eRepeatPeriod,
	unsigned char fgCommitOrCond) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4ChnlIdx, u4Run, u4Idx, u4SlotIdx;
	unsigned char fgChanged = FALSE;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimelineList;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_SCHEDULER_T *prNanScheduler;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNanScheduler = nanGetScheduler(prAdapter);

	if (fgCommitOrCond) {
		prChnlTimelineList = prNanTimelineMgmt->arChnlList;
	} else {
		if (prNanTimelineMgmt->fgChkCondAvailability == FALSE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			goto DELETE_CRB_DONE;
		}

		prChnlTimelineList = prNanTimelineMgmt->arCondChnlList;
	}

	for (u4ChnlIdx = 0; u4ChnlIdx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM;
	     u4ChnlIdx++) {
		if (prChnlTimelineList[u4ChnlIdx].fgValid == FALSE)
			continue;

		for (u4Run = 0; u4Run < (128 >> eRepeatPeriod); u4Run++) {
			for (u4Idx = 0; u4Idx < u4NumSlots; u4Idx++) {
				u4SlotIdx = (u4Run << (eRepeatPeriod + 2)) +
					    u4StartOffset;

				if (NAN_IS_AVAIL_MAP_SET(
					    prChnlTimelineList[u4ChnlIdx]
						    .au4AvailMap,
					    u4SlotIdx) == TRUE) {
					prChnlTimelineList[u4ChnlIdx].i4Num--;
					if (prChnlTimelineList[u4ChnlIdx]
						    .i4Num <= 0)
						prChnlTimelineList[u4ChnlIdx]
							.fgValid = FALSE;

					NAN_TIMELINE_UNSET(
						prChnlTimelineList[u4ChnlIdx]
							.au4AvailMap,
						u4SlotIdx);
					fgChanged = TRUE;
				}
			}
		}

		if (fgChanged && !fgCommitOrCond)
			prNanScheduler->u2NanAvailAttrControlField |=
				NAN_AVAIL_CTRL_COMMIT_CHANGED;
	}

DELETE_CRB_DONE:
	return rRetStatus;
}

void
nanSchedPeerPurgeOldCrb(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	uint32_t u4Idx;

	do {
		prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
		if (prPeerSchRecord == NULL)
			break;

		/* reserve necessary CRB */
		kalMemZero((uint8_t *)
			prPeerSchRecord->rCommFawTimeline.au4AvailMap,
			sizeof(prPeerSchRecord->rCommFawTimeline.au4AvailMap));
		if (prPeerSchRecord->fgUseDataPath == TRUE) {
			for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++)
				prPeerSchRecord->rCommFawTimeline
					.au4AvailMap[u4Idx] |=
					prPeerSchRecord->rCommImmuNdlTimeline
						.au4AvailMap[u4Idx];
			prPeerSchRecord->prCommNdcCtrl = NULL;
		}
		if (prPeerSchRecord->fgUseRanging == TRUE) {
			for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++)
				prPeerSchRecord->rCommFawTimeline
					.au4AvailMap[u4Idx] |=
					prPeerSchRecord->rCommRangingTimeline
						.au4AvailMap[u4Idx];
		}
	} while (FALSE);

	nanSchedReleaseUnusedCommitSlot(prAdapter);
}

void
nanSchedPeerDataPathConflict(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {}

void
nanSchedPeerRangingConflict(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	nanRangingScheduleViolation(prAdapter, pucNmiAddr);
}

uint32_t
nanSchedPeerInterpretScheduleEntryList(
	struct ADAPTER *prAdapter, struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc,
	struct _NAN_SCHEDULE_TIMELINE_T arTimeline[NAN_NUM_AVAIL_DB],
	struct _NAN_SCHEDULE_ENTRY_T *prScheduleEntryList,
	uint32_t u4ScheduleEntryListLength) {
	uint32_t rRetStatus = WLAN_STATUS_FAILURE;
	uint8_t *pucScheduleEntry;
	uint8_t *pucScheduleEntryListEnd;
	struct _NAN_SCHEDULE_ENTRY_T *prScheduleEntry;
	uint32_t u4Idx;
	uint32_t u4TargetIdx;

	for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
		arTimeline[u4Idx].ucMapId = NAN_INVALID_MAP_ID;
		kalMemZero(arTimeline[u4Idx].au4AvailMap,
			   sizeof(arTimeline[u4Idx].au4AvailMap));
	}

	pucScheduleEntry = (uint8_t *)prScheduleEntryList;
	pucScheduleEntryListEnd = pucScheduleEntry + u4ScheduleEntryListLength;
	for ((prScheduleEntry =
		      (struct _NAN_SCHEDULE_ENTRY_T *)pucScheduleEntry);
	     (pucScheduleEntry < pucScheduleEntryListEnd);
	     (pucScheduleEntry +=
	      (prScheduleEntry->ucTimeBitmapLength + 4
	       /* MapID(1)+TimeBitmapControl(2)+TimeBitmapLength(1) */))) {

		prScheduleEntry =
			(struct _NAN_SCHEDULE_ENTRY_T *)pucScheduleEntry;

		u4TargetIdx = NAN_NUM_AVAIL_DB;
		for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
			if (arTimeline[u4Idx].ucMapId ==
			    prScheduleEntry->ucMapID) {
				u4TargetIdx = u4Idx;
				break;
			} else if ((u4TargetIdx == NAN_NUM_AVAIL_DB) &&
				   arTimeline[u4Idx].ucMapId ==
					   NAN_INVALID_MAP_ID)
				u4TargetIdx = u4Idx;
		}

		if (u4TargetIdx >= NAN_NUM_AVAIL_DB)
			break;

		arTimeline[u4TargetIdx].ucMapId = prScheduleEntry->ucMapID;

		nanParserInterpretTimeBitmapField(
			prAdapter, prScheduleEntry->u2TimeBitmapControl,
			prScheduleEntry->ucTimeBitmapLength,
			prScheduleEntry->aucTimeBitmap,
			arTimeline[u4TargetIdx].au4AvailMap);

		rRetStatus = WLAN_STATUS_SUCCESS;
	}

	return rRetStatus;
}

uint32_t
nanSchedPeerChkQos(struct ADAPTER *prAdapter,
		   struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;
	struct _NAN_SCHEDULE_TIMELINE_T *prTimeline;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4DwIdx;
	uint32_t u4SchIdx;
	uint32_t u4QosMinSlots;
	uint32_t u4QosMaxLatency;
	uint32_t u4EmptySlots;
	uint32_t i4Latency;
	uint32_t u4Idx1;

	if (!prPeerSchDesc->fgUsed)
		return WLAN_STATUS_FAILURE;

	u4SchIdx = prPeerSchDesc->u4SchIdx;
	prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRec == NULL) {
		DBGLOG(NAN, ERROR, "No peer sch record\n");
		return WLAN_STATUS_FAILURE;
	}

	if (prPeerSchRec->fgUseDataPath == FALSE)
		return WLAN_STATUS_FAILURE;

	prTimeline = &prPeerSchRec->rCommFawTimeline;
	for (u4DwIdx = 0; u4DwIdx < NAN_TOTAL_DW; u4DwIdx++) {
		u4QosMinSlots = prPeerSchRec->u4FinalQosMinSlots;
		if (u4QosMinSlots > NAN_INVALID_QOS_MIN_SLOTS) {
			if (nanUtilCheckBitOneCnt(
				    prAdapter, (uint8_t *)
				    &prTimeline->au4AvailMap[u4DwIdx],
				    sizeof(uint32_t)) < u4QosMinSlots) {
				/* Qos min slot validation fail */
				return WLAN_STATUS_FAILURE;
			}
		}

		u4QosMaxLatency = prPeerSchRec->u4FinalQosMaxLatency;
		if (u4QosMaxLatency < NAN_INVALID_QOS_MAX_LATENCY) {
			if (u4QosMaxLatency == 0)
				u4QosMaxLatency =
					1; /* reserve 1 slot for DW window */

			u4EmptySlots = ~(prTimeline->au4AvailMap[u4DwIdx]);
			i4Latency = 0;

			for (u4Idx1 = 0; u4Idx1 < 32; u4Idx1++) {
				if (u4EmptySlots & BIT(u4Idx1)) {
					i4Latency++;
					/* Qos max latency validation fail */
					if (i4Latency > u4QosMaxLatency)
						return WLAN_STATUS_FAILURE;
				} else {
					i4Latency = 0;
				}
			}
		}
	}

	return rRetStatus;
}

uint32_t
nanSchedPeerChkTimeline(struct ADAPTER *prAdapter,
			struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc,
			struct _NAN_SCHEDULE_TIMELINE_T *prTimeline) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;
	union _NAN_BAND_CHNL_CTRL rLocalChnlInfo;
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;
	uint32_t u4SlotIdx;
	uint32_t u4AvailDbIdx;
	uint32_t u4SchIdx;
	unsigned char fgCheckOk;

	if (!prPeerSchDesc->fgUsed)
		return WLAN_STATUS_FAILURE;

	u4SchIdx = prPeerSchDesc->u4SchIdx;
	prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRec == NULL) {
		DBGLOG(NAN, ERROR, "No peer sch record\n");
		return WLAN_STATUS_FAILURE;
	}

	for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS; u4SlotIdx++) {
		if (!NAN_IS_AVAIL_MAP_SET(prTimeline->au4AvailMap, u4SlotIdx))
			continue;

		/* check local availability window */
		rLocalChnlInfo = nanQueryChnlInfoBySlot(prAdapter, u4SlotIdx,
							NULL, TRUE);
		if (rLocalChnlInfo.rChannel.u4PrimaryChnl == 0)
			return WLAN_STATUS_FAILURE;

		/* check peer's availability window */
		fgCheckOk = FALSE;

		for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
		     u4AvailDbIdx++) {
			if (nanSchedPeerAvailabilityDbValidByID(
				    prAdapter, u4SchIdx, u4AvailDbIdx) == FALSE)
				continue;

			rRmtChnlInfo = nanGetPeerChnlInfoBySlot(
				prAdapter, u4SchIdx, u4AvailDbIdx, u4SlotIdx,
				FALSE);
			if (rRmtChnlInfo.rChannel.u4PrimaryChnl == 0)
				continue;

			if (nanSchedChkConcurrOp(rLocalChnlInfo,
						 rRmtChnlInfo) !=
			    CNM_CH_CONCURR_MCC) {

				fgCheckOk = TRUE;
				break;
			}
		}

		if (!fgCheckOk)
			return WLAN_STATUS_FAILURE;
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedPeerChkDataPath(struct ADAPTER *prAdapter,
			struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec) {
	struct _NAN_SCHEDULE_TIMELINE_T *prTimeline;
	/* UINT_32 u4DwIdx; */
	uint32_t rRetStatus;

	do {
		rRetStatus = WLAN_STATUS_SUCCESS;
		if (!prPeerSchRec->fgUseDataPath)
			break;

#if 0
		prTimeline = &prPeerSchRec->rCommFawTimeline;
		for (u4DwIdx = 0; u4DwIdx < NAN_TOTAL_DW; u4DwIdx++) {
			if (nanUtilCheckBitOneCnt(prAdapter,
			(PUINT_8)&prTimeline->au4AvailMap[u4DwIdx],
			sizeof(UINT_32)) < prPeerSchRec->u4DefNdlNumSlots) {
				rRetStatus = WLAN_STATUS_FAILURE;
				break;
			}
		}
#endif

		if (prPeerSchRec->prCommNdcCtrl) {
			prTimeline =
				&prPeerSchRec->prCommNdcCtrl->arTimeline[0];
			rRetStatus = nanSchedPeerChkTimeline(
				prAdapter, prPeerSchRec->prPeerSchDesc,
				prTimeline);
			if (rRetStatus != WLAN_STATUS_SUCCESS)
				break;
		}

		prTimeline = &prPeerSchRec->rCommImmuNdlTimeline;
		rRetStatus = nanSchedPeerChkTimeline(
			prAdapter, prPeerSchRec->prPeerSchDesc, prTimeline);
		if (rRetStatus != WLAN_STATUS_SUCCESS)
			break;

		rRetStatus = nanSchedPeerChkQos(prAdapter,
						prPeerSchRec->prPeerSchDesc);
		if (rRetStatus != WLAN_STATUS_SUCCESS)
			break;
	} while (FALSE);

	return rRetStatus;
}

uint32_t
nanSchedPeerChkRanging(struct ADAPTER *prAdapter,
		       struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec) {
	struct _NAN_SCHEDULE_TIMELINE_T *prTimeline;
	uint32_t rRetStatus;

	do {
		rRetStatus = WLAN_STATUS_SUCCESS;
		if (!prPeerSchRec->fgUseRanging)
			break;

		prTimeline = &prPeerSchRec->rCommRangingTimeline;
		rRetStatus = nanSchedPeerChkTimeline(
			prAdapter, prPeerSchRec->prPeerSchDesc, prTimeline);
		if (rRetStatus != WLAN_STATUS_SUCCESS)
			break;
	} while (FALSE);

	return rRetStatus;
}

void
nanSchedPeerChkAvailability(struct ADAPTER *prAdapter,
			    struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;
	uint32_t u4SchIdx;
	uint32_t rRetStatus;

	if (!prPeerSchDesc->fgUsed)
		return;

	u4SchIdx = prPeerSchDesc->u4SchIdx;
	prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRec == NULL) {
		DBGLOG(NAN, ERROR, "No peer sch record\n");
		return;
	}

	/* Check Data Path CRB Violation */
	rRetStatus = nanSchedPeerChkDataPath(prAdapter, prPeerSchRec);
	if (rRetStatus != WLAN_STATUS_SUCCESS)
		nanSchedPeerDataPathConflict(prAdapter,
					     prPeerSchDesc->aucNmiAddr);

	/* Check Ranging CRB Violation */
	rRetStatus = nanSchedPeerChkRanging(prAdapter, prPeerSchRec);
	if (rRetStatus != WLAN_STATUS_SUCCESS)
		nanSchedPeerRangingConflict(prAdapter,
					    prPeerSchDesc->aucNmiAddr);
}

void
nanSchedPeerPrepareNegoState(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	prPeerSchRecord = nanSchedLookupPeerSchRecord(prAdapter, pucNmiAddr);
	if (prPeerSchRecord == NULL)
		return;
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	prPeerSchRecord->i4InNegoContext++;
	prNegoCtrl->i4InNegoContext++;

	DBGLOG(NAN, INFO,
	       "[%02x:%02x:%02x:%02x:%02x:%02x] i4InNegoContext:%d (%d)\n",
	       pucNmiAddr[0], pucNmiAddr[1], pucNmiAddr[2], pucNmiAddr[3],
	       pucNmiAddr[4], pucNmiAddr[5], prPeerSchRecord->i4InNegoContext,
	       prNegoCtrl->i4InNegoContext);
}

void
nanSchedPeerCompleteNegoState(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	prPeerSchRecord = nanSchedLookupPeerSchRecord(prAdapter, pucNmiAddr);
	if (prPeerSchRecord == NULL)
		return;
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	prPeerSchRecord->i4InNegoContext--;
	prNegoCtrl->i4InNegoContext--;

	DBGLOG(NAN, INFO,
	       "[%02x:%02x:%02x:%02x:%02x:%02x] i4InNegoContext:%d (%d)\n",
	       pucNmiAddr[0], pucNmiAddr[1], pucNmiAddr[2], pucNmiAddr[3],
	       pucNmiAddr[4], pucNmiAddr[5], prPeerSchRecord->i4InNegoContext,
	       prNegoCtrl->i4InNegoContext);
}

unsigned char
nanSchedPeerInNegoState(struct ADAPTER *prAdapter,
			struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	if (!prPeerSchDesc->fgUsed)
		return FALSE;

	prPeerSchRecord =
		nanSchedGetPeerSchRecord(prAdapter, prPeerSchDesc->u4SchIdx);

	if (!prPeerSchRecord) {
		DBGLOG(NAN, ERROR, "prPeerSchRecord error!\n");
		return FALSE;
	}

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	if (prNegoCtrl->eSyncSchUpdateCurrentState !=
	    ENUM_NAN_SYNC_SCH_UPDATE_STATE_IDLE)
		return TRUE;

#if 1
	if (prPeerSchRecord->i4InNegoContext > 0) {
		DBGLOG(NAN, INFO,
		       "PeeSchRecord:%d in NEGO state, i4InNegoContext:%d (%d)\n",
		       prPeerSchDesc->u4SchIdx,
		       prPeerSchRecord->i4InNegoContext,
		       prNegoCtrl->i4InNegoContext);
		return TRUE;
	}
#else
	if ((prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_IDLE) &&
	    (prNegoCtrl->u4SchIdx == prPeerSchDesc->u4SchIdx))
		return TRUE;
#endif

	return FALSE;
}

uint32_t
nanSchedPeerUpdateAvailabilityAttr(struct ADAPTER *prAdapter,
				   uint8_t *pucNmiAddr,
				   uint8_t *pucAvailabilityAttr) {
	struct _NAN_ATTR_NAN_AVAILABILITY_T *prAttrNanAvailibility;
	struct _NAN_AVAILABILITY_ENTRY_T *prAttrAvailEntry;
	struct _NAN_BAND_CHNL_LIST_T *prAttrBandChnlList;
	struct _NAN_CHNL_ENTRY_T *prAttrChnlEntry;
	struct _NAN_AVAILABILITY_DB_T *prNanAvailDB;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntry;
	uint16_t u2AttributeControl;
	uint8_t ucMapId;
	uint32_t u4EntryListPos;
	uint8_t *pucAvailEntry;
	uint8_t *pucAvailEntryEndPos;
	uint16_t u2EntryControl;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	unsigned char fgChnlType;
	unsigned char fgNonContinuousBw;
	uint32_t u4NumBandChnlEntries;
	uint8_t *pucBandChnlEntryList;
	uint16_t u2TimeBitmapControl;
	uint8_t ucTimeBitmapLength;
	uint8_t ucNumBandChnlCtrl;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	uint8_t ucPriChnl = 0;
	uint32_t u4Token;

	do {
		prAttrNanAvailibility = (struct _NAN_ATTR_NAN_AVAILABILITY_T *)
			pucAvailabilityAttr;

		prPeerSchDesc =
			nanSchedAcquirePeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		u4Token = nanUtilCalAttributeToken(
			(struct _NAN_ATTR_HDR_T *)pucAvailabilityAttr);
		if ((prPeerSchDesc->u4AvailAttrToken != 0) &&
		    (prPeerSchDesc->u4AvailAttrToken == u4Token))
			break;
		prPeerSchDesc->u4AvailAttrToken = u4Token;

		DBGLOG(NAN, INFO, "\n\n");
		DBGLOG(NAN, INFO, "------>\n");
		nanUtilDump(prAdapter, "[Peer Avail]", pucAvailabilityAttr,
			    prAttrNanAvailibility->u2Length + 3);

		u2AttributeControl = prAttrNanAvailibility->u2AttributeControl;
		ucMapId = (u2AttributeControl & NAN_AVAIL_CTRL_MAPID);

		prNanAvailDB = nanSchedPeerAcquireAvailabilityDB(
			prAdapter, prPeerSchDesc, ucMapId);
		if (prNanAvailDB == NULL) {
			rRetStatus = WLAN_STATUS_FAILURE;
			DBGLOG(NAN, ERROR,
			       "No Availability attribute record\n");
			break;
		}

		/* release old availability entries */
		for (u4EntryListPos = 0;
		     u4EntryListPos < NAN_NUM_AVAIL_TIMELINE;
		     u4EntryListPos++) {

			prNanAvailEntry =
				&prNanAvailDB->arAvailEntryList[u4EntryListPos];
			prNanAvailEntry->fgActive = FALSE;
		}
		u4EntryListPos = 0;

		pucAvailEntry = prAttrNanAvailibility->aucAvailabilityEntryList;
		pucAvailEntryEndPos = pucAvailEntry +
				      prAttrNanAvailibility->u2Length -
				      3 /* Seq ID(1) + Attribute Ctrl(2) */;

		do {
			prAttrAvailEntry = (struct _NAN_AVAILABILITY_ENTRY_T *)
				pucAvailEntry;
			u2EntryControl = prAttrAvailEntry->u2EntryControl;

			prNanAvailEntry =
				&prNanAvailDB->arAvailEntryList[u4EntryListPos];
			prNanAvailEntry->fgActive = TRUE;
			prNanAvailEntry->ucNumBandChnlCtrl = 0;
			prNanAvailEntry->rEntryCtrl.u2RawData = 0;
			kalMemZero(prNanAvailEntry->au4AvailMap,
				   sizeof(prNanAvailEntry->au4AvailMap));

			DBGLOG(NAN, LOUD, "[%d] Entry Control:0x%x\n",
			       u4EntryListPos, u2EntryControl);

			prNanAvailEntry->rEntryCtrl.u2RawData = u2EntryControl;

			if ((u2EntryControl &
			     NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT) ==
			    0) {
				/* all slots are available when timebitmap
				 * is not set
				 */
				prAttrBandChnlList =
				    (struct _NAN_BAND_CHNL_LIST_T *)
				    (prAttrAvailEntry
					->aucTimeBitmapAndBandChnlEntry);
				kalMemSet(prNanAvailEntry->au4AvailMap, 0xFF,
				    sizeof(prNanAvailEntry->au4AvailMap));
			} else {
				u2TimeBitmapControl = *(uint16_t *)(
					prAttrAvailEntry
					->aucTimeBitmapAndBandChnlEntry);
				ucTimeBitmapLength =
					prAttrAvailEntry
					->aucTimeBitmapAndBandChnlEntry[2];
				prAttrBandChnlList =
					(struct _NAN_BAND_CHNL_LIST_T *)
					(prAttrAvailEntry
					->aucTimeBitmapAndBandChnlEntry +
					3 + ucTimeBitmapLength);
				nanParserInterpretTimeBitmapField(
					prAdapter, u2TimeBitmapControl,
					ucTimeBitmapLength,
					&prAttrAvailEntry
					->aucTimeBitmapAndBandChnlEntry[3],
					prNanAvailEntry->au4AvailMap);
			}

			pucBandChnlEntryList = prAttrBandChnlList->aucEntry;
			fgChnlType = (prAttrBandChnlList->ucType ==
				      NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL);
			fgNonContinuousBw = prAttrBandChnlList->ucNonContiguous;
			u4NumBandChnlEntries =
				prAttrBandChnlList->ucNumberOfEntry;

			if (fgChnlType) {
				if (u4NumBandChnlEntries >
					NAN_NUM_BAND_CHNL_ENTRY)
					u4NumBandChnlEntries =
						NAN_NUM_BAND_CHNL_ENTRY;

				prNanAvailEntry->ucNumBandChnlCtrl = 0;
				ucNumBandChnlCtrl = 0;
				while (u4NumBandChnlEntries) {
					prAttrChnlEntry =
					(struct _NAN_CHNL_ENTRY_T *)
					pucBandChnlEntryList;

				/* only select one channel from Channel
				 * Bitmap in the Channel Entry
				 */
					ucPriChnl =
					nanSchedChooseBestFromChnlBitmap(
					    prAdapter,
					    prAttrChnlEntry->ucOperatingClass,
					    &prAttrChnlEntry->u2ChannelBitmap,
					    fgNonContinuousBw,
					    prAttrChnlEntry
						->ucPrimaryChnlBitmap);

					prNanAvailEntry->arBandChnlCtrl
					[ucNumBandChnlCtrl].rChannel.u4Type =
					NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL;
					prNanAvailEntry->arBandChnlCtrl
					[ucNumBandChnlCtrl]
					  .rChannel.u4OperatingClass =
					    prAttrChnlEntry->ucOperatingClass;
					prNanAvailEntry->arBandChnlCtrl
					  [ucNumBandChnlCtrl].rChannel
					    .u4PrimaryChnl = ucPriChnl;

					if ((fgNonContinuousBw) &&
					    (nanRegGetBw(prAttrChnlEntry
					    ->ucOperatingClass) == 160)) {
					prNanAvailEntry->arBandChnlCtrl
					[ucNumBandChnlCtrl].rChannel
					    .u4AuxCenterChnl =
					    nanRegGetChannelByOrder(
						prAttrChnlEntry
						    ->ucOperatingClass,
						&prAttrChnlEntry
						    ->u2AuxChannelBitmap);
					pucBandChnlEntryList += sizeof(
						struct _NAN_CHNL_ENTRY_T);
					} else {
						prNanAvailEntry->arBandChnlCtrl
						[ucNumBandChnlCtrl].rChannel
						.u4AuxCenterChnl = 0;
						pucBandChnlEntryList +=
					(sizeof(struct _NAN_CHNL_ENTRY_T) - 2);
					}

					if (ucPriChnl == 0) {
						u4NumBandChnlEntries--;
						continue;
					}

					DBGLOG(NAN, LOUD,
					"Ch:%d, Bw:%d, OpClass:%d\n", ucPriChnl,
					nanRegGetBw(
					prAttrChnlEntry->ucOperatingClass),
					prAttrChnlEntry->ucOperatingClass);

					u4NumBandChnlEntries--;
					prNanAvailEntry->ucNumBandChnlCtrl++;
					ucNumBandChnlCtrl++;
				}
			} else {
				prNanAvailEntry->ucNumBandChnlCtrl = 1;
				prNanAvailEntry->arBandChnlCtrl[0]
				.rBand.u4Type =
				NAN_BAND_CH_ENTRY_LIST_TYPE_BAND;
				prNanAvailEntry->arBandChnlCtrl[0]
				.rBand.u4BandIdMask = 0;

				while (u4NumBandChnlEntries) {
					prNanAvailEntry->arBandChnlCtrl[0]
					.rBand.u4BandIdMask |=
					BIT(*pucBandChnlEntryList);

					pucBandChnlEntryList++;
					u4NumBandChnlEntries--;
				}
			}

			pucAvailEntry = pucAvailEntry +
					prAttrAvailEntry->u2Length +
					2 /* length(2) */;
			u4EntryListPos++;
		} while ((pucAvailEntry < pucAvailEntryEndPos) &&
			(u4EntryListPos < NAN_NUM_AVAIL_TIMELINE));

		if (prPeerSchDesc->fgUsed) {
			nanSchedPeerUpdateCommonFAW(prAdapter,
						    prPeerSchDesc->u4SchIdx);
			if (!nanSchedNegoInProgress(prAdapter))
				nanSchedPeerChkAvailability(prAdapter,
					prPeerSchDesc);
		}

		nanSchedDbgDumpPeerAvailability(prAdapter, pucNmiAddr);
		DBGLOG(NAN, INFO, "<------\n");
	} while (FALSE);

	return rRetStatus;
}

uint32_t
nanSchedPeerUpdateDevCapabilityAttr(struct ADAPTER *prAdapter,
				    uint8_t *pucNmiAddr,
				    uint8_t *pucDevCapabilityAttr) {
	struct _NAN_ATTR_DEVICE_CAPABILITY_T *prAttrDevCapability;
	struct _NAN_DEVICE_CAPABILITY_T *prNanDevCapability;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4AvailabilityDbIdx;
	uint8_t ucMapID;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	uint32_t u4Token;

	do {
		prAttrDevCapability = (struct _NAN_ATTR_DEVICE_CAPABILITY_T *)
			pucDevCapabilityAttr;

		prPeerSchDesc =
			nanSchedAcquirePeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		u4Token = nanUtilCalAttributeToken(
			(struct _NAN_ATTR_HDR_T *)pucDevCapabilityAttr);
		if ((prPeerSchDesc->u4DevCapAttrToken != 0) &&
		    (prPeerSchDesc->u4DevCapAttrToken == u4Token))
			break;
		prPeerSchDesc->u4DevCapAttrToken = u4Token;

		DBGLOG(NAN, INFO, "\n\n");
		DBGLOG(NAN, INFO, "------>\n");
		nanUtilDump(prAdapter, "[Peer DevCap]", pucDevCapabilityAttr,
			    prAttrDevCapability->u2Length + 3);

		if (prAttrDevCapability->ucMapID & BIT(0)) {
			ucMapID = (prAttrDevCapability->ucMapID & BITS(1, 4)) >>
				  1;

			u4AvailabilityDbIdx = nanSchedPeerGetAvailabilityDbIdx(
				prAdapter, prPeerSchDesc, ucMapID);
			if (u4AvailabilityDbIdx == NAN_INVALID_AVAIL_DB_INDEX) {
				rRetStatus = WLAN_STATUS_FAILURE;
				break;
			}
		} else {
			ucMapID = NAN_INVALID_MAP_ID;
			/* for global device capability */
			u4AvailabilityDbIdx =
				NAN_NUM_AVAIL_DB;
		}

		prNanDevCapability =
			&prPeerSchDesc->arDevCapability[u4AvailabilityDbIdx];
		prNanDevCapability->fgValid = TRUE;
		prNanDevCapability->ucMapId = ucMapID;

		prNanDevCapability->ucCapabilitySet =
			prAttrDevCapability->ucCapabilities;
		prNanDevCapability->u2MaxChnlSwitchTime =
			prAttrDevCapability->u2MaxChannelSwitchTime;
		prNanDevCapability->ucSupportedBand =
			prAttrDevCapability->ucSupportedBands;
		prNanDevCapability->ucOperationMode =
			prAttrDevCapability->ucOperationMode;
		prNanDevCapability->ucNumRxAnt =
			(prAttrDevCapability->ucNumOfAntennas & BITS(4, 7)) >>
			4;
		prNanDevCapability->ucNumTxAnt =
			(prAttrDevCapability->ucNumOfAntennas & BITS(0, 3));
		prNanDevCapability->ucDw24g =
			(prAttrDevCapability->u2CommittedDWInfo &
			 NAN_COMMITTED_DW_INFO_24G) >>
			NAN_COMMITTED_DW_INFO_24G_OFFSET;
		prNanDevCapability->ucDw5g =
			(prAttrDevCapability->u2CommittedDWInfo &
			 NAN_COMMITTED_DW_INFO_5G) >>
			NAN_COMMITTED_DW_INFO_5G_OFFSET;
		prNanDevCapability->ucOvrDw24gMapId =
			(prAttrDevCapability->u2CommittedDWInfo &
			 NAN_COMMITTED_DW_INFO_24G_DW_OVERWRITE) >>
			NAN_COMMITTED_DW_INFO_24G_DW_OVERWRITE_OFFSET;
		prNanDevCapability->ucOvrDw5gMapId =
			(prAttrDevCapability->u2CommittedDWInfo &
			 NAN_COMMITTED_DW_INFO_5G_DW_OVERWRITE) >>
			NAN_COMMITTED_DW_INFO_5G_DW_OVERWRITE_OFFSET;

		DBGLOG(NAN, INFO,
		       "[Dev Capability] NMI=>%02x:%02x:%02x:%02x:%02x:%02x\n",
		       prPeerSchDesc->aucNmiAddr[0],
		       prPeerSchDesc->aucNmiAddr[1],
		       prPeerSchDesc->aucNmiAddr[2],
		       prPeerSchDesc->aucNmiAddr[3],
		       prPeerSchDesc->aucNmiAddr[4],
		       prPeerSchDesc->aucNmiAddr[5]);
		DBGLOG(NAN, INFO, "MapID:%d\n", prNanDevCapability->ucMapId);
		DBGLOG(NAN, INFO, "Capability Set:0x%x\n",
		       prNanDevCapability->ucCapabilitySet);
		DBGLOG(NAN, INFO, "Max Chnl Switch Time:%d\n",
		       prNanDevCapability->u2MaxChnlSwitchTime);
		DBGLOG(NAN, INFO, "Supported Band:0x%x\n",
		       prNanDevCapability->ucSupportedBand);
		DBGLOG(NAN, INFO, "Operation Mode:0x%x\n",
		       prNanDevCapability->ucOperationMode);
		DBGLOG(NAN, INFO, "Rx Ant:%d\n",
		       prNanDevCapability->ucNumRxAnt);
		DBGLOG(NAN, INFO, "Tx Ant:%d\n",
		       prNanDevCapability->ucNumTxAnt);
		DBGLOG(NAN, INFO, "DW 2.4G:%d\n", prNanDevCapability->ucDw24g);
		DBGLOG(NAN, INFO, "DW 5G:%d\n", prNanDevCapability->ucDw5g);

		if (prPeerSchDesc->fgUsed)
			nanSchedCmdUpdatePeerCapability(
				prAdapter, prPeerSchDesc->u4SchIdx);
		DBGLOG(NAN, INFO, "<------\n");
	} while (FALSE);

	return rRetStatus;
}

uint32_t
nanSchedPeerUpdateQosAttr(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
			  uint8_t *pucQosAttr) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_ATTR_NDL_QOS_T *prNdlQosAttr;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	DBGLOG(NAN, INFO, "\n\n");
	DBGLOG(NAN, INFO, "------>\n");

	do {
		prPeerSchDesc =
			nanSchedAcquirePeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		prNdlQosAttr = (struct _NAN_ATTR_NDL_QOS_T *)pucQosAttr;

		if (prNdlQosAttr->u2MaxLatency < prPeerSchDesc->u4QosMaxLatency)
			prPeerSchDesc->u4QosMaxLatency =
				prNdlQosAttr->u2MaxLatency;

		if (prNdlQosAttr->ucMinTimeSlot > prPeerSchDesc->u4QosMinSlots)
			prPeerSchDesc->u4QosMinSlots =
				prNdlQosAttr->ucMinTimeSlot;

		DBGLOG(NAN, INFO, "MinSlot:%d, MaxLatency:%d\n",
		       prPeerSchDesc->u4QosMinSlots,
		       prPeerSchDesc->u4QosMaxLatency);
	} while (FALSE);

	DBGLOG(NAN, INFO, "<------\n");

	return rRetStatus;
}

uint32_t
nanSchedPeerUpdateNdcAttr(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
			  uint8_t *pucNdcAttr) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4ScheduleEntryListLength;
	struct _NAN_ATTR_NDC_T *prAttrNdc;
	struct _NAN_NDC_CTRL_T *prNanNdcCtrl;
	struct _NAN_SCHEDULE_ENTRY_T *prScheduleEntryList;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	DBGLOG(NAN, INFO, "\n\n");
	DBGLOG(NAN, INFO, "------>\n");

	do {
		prPeerSchDesc =
			nanSchedAcquirePeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		prAttrNdc = (struct _NAN_ATTR_NDC_T *)pucNdcAttr;
		prScheduleEntryList = (struct _NAN_SCHEDULE_ENTRY_T *)
					      prAttrNdc->aucScheduleEntryList;
		u4ScheduleEntryListLength =
			prAttrNdc->u2Length -
			7 /* NdcID(6)+AttributeControl(1) */;

		/** Only handle selected NDC for CRB negotiation */
		if (!(prAttrNdc->ucAttributeControl &
		      NAN_ATTR_NDC_CTRL_SELECTED_FOR_NDL)) {
			DBGLOG(NAN, INFO, "Not Selected NDC\n");
			break;
		}

		nanUtilDump(prAdapter, "[Peer NDC]", (uint8_t *)prAttrNdc,
			    prAttrNdc->u2Length + 3);

		prNanNdcCtrl = &prPeerSchDesc->rSelectedNdcCtrl;
		kalMemZero(prNanNdcCtrl, sizeof(struct _NAN_NDC_CTRL_T));

		if (nanSchedPeerInterpretScheduleEntryList(
			    prAdapter, prPeerSchDesc, prNanNdcCtrl->arTimeline,
			    prScheduleEntryList,
			    u4ScheduleEntryListLength) == WLAN_STATUS_SUCCESS) {
			prNanNdcCtrl->fgValid = TRUE;
			kalMemCopy(prNanNdcCtrl->aucNdcId, prAttrNdc->aucNDCID,
				   NAN_NDC_ATTRIBUTE_ID_LENGTH);
		} else {
			prNanNdcCtrl->fgValid = FALSE;
		}
	} while (FALSE);

	DBGLOG(NAN, INFO, "<------\n");
	return rRetStatus;
}

uint32_t
nanSchedPeerUpdateUawAttr(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
			  uint8_t *pucUawAttr) {
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_ATTR_UNALIGNED_SCHEDULE_T *prAttrUaw;

	void *prCmdBuffer = NULL;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	uint8_t *pucUawBuf;

	do {
		prPeerSchRecord =
			nanSchedLookupPeerSchRecord(prAdapter, pucNmiAddr);
		if (prPeerSchRecord == NULL) {
			rStatus = WLAN_STATUS_FAILURE;
			break;
		}

		DBGLOG(NAN, INFO, "\n\n");
		DBGLOG(NAN, INFO, "------>\n");

		prAttrUaw = (struct _NAN_ATTR_UNALIGNED_SCHEDULE_T *)pucUawAttr;
		nanUtilDump(prAdapter, "[Peer UAW]", (uint8_t *)prAttrUaw,
			    prAttrUaw->u2Length + 3);

		u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
				 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
				 (MAC_ADDR_LEN + prAttrUaw->u2Length + 3);
		prCmdBuffer =
			cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);
		if (!prCmdBuffer) {
			rStatus = WLAN_STATUS_FAILURE;
			break;
		}

		prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

		prTlvCommon->u2TotalElementNum = 0;

		rStatus = nicAddNewTlvElement(
			NAN_CMD_UPDATE_PEER_UAW,
			(MAC_ADDR_LEN + prAttrUaw->u2Length + 3),
			u4CmdBufferLen, prCmdBuffer);
		if (rStatus != WLAN_STATUS_SUCCESS) {
			DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
			rStatus = WLAN_STATUS_FAILURE;
			break;
		}

		prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);
		if (prTlvElement == NULL) {
			DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
			rStatus = WLAN_STATUS_FAILURE;
			break;
		}

		pucUawBuf = prTlvElement->aucbody;
		kalMemCopy(pucUawBuf, pucNmiAddr, MAC_ADDR_LEN);
		kalMemCopy(pucUawBuf + MAC_ADDR_LEN, pucUawAttr,
			   (prAttrUaw->u2Length + 3));

		rStatus = wlanSendSetQueryCmd(
			prAdapter, CMD_ID_NAN_EXT_CMD, TRUE, FALSE, FALSE, NULL,
			nicCmdTimeoutCommon, u4CmdBufferLen,
			(uint8_t *)prCmdBuffer, NULL, 0);
		if (rStatus == WLAN_STATUS_PENDING)
			rStatus = WLAN_STATUS_SUCCESS;

		DBGLOG(NAN, INFO, "<------\n");
	} while (FALSE);

	if (prCmdBuffer)
		cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

/* [nanSchedPeerUpdateImmuNdlScheduleList]
 *
 *   Only allowed when it acquires the token to nego CRB.
 */
uint32_t
nanSchedPeerUpdateImmuNdlScheduleList(
	struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
	struct _NAN_SCHEDULE_ENTRY_T *prScheduleEntryList,
	uint32_t u4ScheduleEntryListLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	do {
		prPeerSchDesc =
			nanSchedAcquirePeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		nanUtilDump(prAdapter, "[Peer Immu NDL]",
			    (uint8_t *)prScheduleEntryList,
			    u4ScheduleEntryListLength);

		prPeerSchDesc->fgImmuNdlTimelineValid = FALSE;
		rRetStatus = nanSchedPeerInterpretScheduleEntryList(
			prAdapter, prPeerSchDesc,
			prPeerSchDesc->arImmuNdlTimeline, prScheduleEntryList,
			u4ScheduleEntryListLength);
		if (rRetStatus == WLAN_STATUS_SUCCESS) {
			prPeerSchDesc->fgImmuNdlTimelineValid = TRUE;

#if 0
{
			UINT_32 u4Idx = 0;

			for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
				nanUtilDump(prAdapter, "[Peer Immu NDL Bitmap]",
				(PUINT_8)prPeerSchDesc
				->arImmuNdlTimeline[u4Idx].au4AvailMap,
				sizeof(prPeerSchDesc->arImmuNdlTimeline[u4Idx].
				au4AvailMap));
			}
}
#endif
		}
	} while (FALSE);

	return rRetStatus;
}

/* [nanSchedPeerUpdateRangingScheduleList]
 *
 *   Only allowed when it acquires the token to nego CRB.
 */
uint32_t
nanSchedPeerUpdateRangingScheduleList(
	struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
	struct _NAN_SCHEDULE_ENTRY_T *prScheduleEntryList,
	uint32_t u4ScheduleEntryListLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	do {
		prPeerSchDesc =
			nanSchedAcquirePeerSchDescByNmi(prAdapter, pucNmiAddr);
		if (prPeerSchDesc == NULL) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		nanUtilDump(prAdapter, "[Peer Ranging]",
			    (uint8_t *)prScheduleEntryList,
			    u4ScheduleEntryListLength);

		prPeerSchDesc->fgRangingTimelineValid = FALSE;
		rRetStatus = nanSchedPeerInterpretScheduleEntryList(
			prAdapter, prPeerSchDesc,
			prPeerSchDesc->arRangingTimeline, prScheduleEntryList,
			u4ScheduleEntryListLength);
		if (rRetStatus == WLAN_STATUS_SUCCESS)
			prPeerSchDesc->fgRangingTimelineValid = TRUE;
	} while (FALSE);

	return rRetStatus;
}

void
nanSchedPeerUpdateCommonFAW(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	uint32_t u4SlotIdx;
	uint32_t u4AvailDbIdx;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_SCHEDULE_TIMELINE_T *prTimeline;
	union _NAN_BAND_CHNL_CTRL rLocalChnlInfo;
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;
	struct _NAN_SCHEDULER_T *prScheduler;
	enum ENUM_BAND eBand;

	prScheduler = nanGetScheduler(prAdapter);

	prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (prPeerSchRecord == NULL)
		return;

	if ((prPeerSchRecord->fgActive == FALSE) ||
	    ((prPeerSchRecord->fgUseDataPath == FALSE) &&
	     (prPeerSchRecord->fgUseRanging == FALSE)))
		return;

	prTimeline = &prPeerSchRecord->rCommFawTimeline;
	kalMemZero(prTimeline->au4AvailMap, sizeof(prTimeline->au4AvailMap));

	for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS; u4SlotIdx++) {
		if (nanIsDiscWindow(prAdapter, u4SlotIdx))
			continue;

		rLocalChnlInfo = nanQueryChnlInfoBySlot(prAdapter, u4SlotIdx,
							NULL, TRUE);
		if (rLocalChnlInfo.rChannel.u4PrimaryChnl == 0)
			continue;
		DBGLOG(NAN, LOUD, "[LOCAL] Slot:%d -> Chnl:%d\n", u4SlotIdx,
		       rLocalChnlInfo.rChannel.u4PrimaryChnl);

		for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
		     u4AvailDbIdx++) {
			if (nanSchedPeerAvailabilityDbValidByDesc(
				    prAdapter, prPeerSchRecord->prPeerSchDesc,
				    u4AvailDbIdx) == FALSE)
				continue;

			rRmtChnlInfo = nanGetPeerChnlInfoBySlot(
				prAdapter, u4SchIdx, u4AvailDbIdx, u4SlotIdx,
				TRUE);
			if (rRmtChnlInfo.rChannel.u4PrimaryChnl == 0)
				continue;
			DBGLOG(NAN, LOUD, "[RMT][%d] Slot:%d -> Chnl:%d\n",
			       u4AvailDbIdx, u4SlotIdx,
			       rRmtChnlInfo.rChannel.u4PrimaryChnl);

			if (nanSchedChkConcurrOp(rLocalChnlInfo,
						 rRmtChnlInfo) !=
			    CNM_CH_CONCURR_MCC) {
				DBGLOG(NAN, LOUD, "co-channel check pass\n");
				NAN_TIMELINE_SET(prTimeline->au4AvailMap,
						 u4SlotIdx);

				/* Update used band to peer schedule record */
				eBand =
				(rLocalChnlInfo.rChannel.u4PrimaryChnl < 36)
						? BAND_2G4 : BAND_5G;

				if (prPeerSchRecord->eBand == BAND_NULL) {
					prPeerSchRecord->eBand = eBand;
					DBGLOG(NAN, INFO,
					"Peer %d use band %d\n",
					u4SchIdx, prPeerSchRecord->eBand);
				} else if (prPeerSchRecord->eBand != eBand) {
					DBGLOG(NAN, ERROR,
					"Band conflict %d != %d\n",
						prPeerSchRecord->eBand, eBand);
				}
				break;
			}
		}
	}

	nanSchedCmdUpdateCRB(prAdapter, u4SchIdx);
}

uint32_t
nanSchedConfigGetAllowedBw(struct ADAPTER *prAdapter, uint8_t ucChannel) {
	enum _NAN_CHNL_BW_MAP eBwMap;
	uint32_t u4SupportedBw;

	eBwMap = prAdapter->rWifiVar.ucNanBandwidth;
	if (((ucChannel < 36) || (!prAdapter->rWifiVar.fgEnNanVHT)) &&
	    (eBwMap > NAN_CHNL_BW_40))
		eBwMap = NAN_CHNL_BW_40;

	switch (eBwMap) {
	default:
	case NAN_CHNL_BW_20:
		u4SupportedBw = 20;
		break;

	case NAN_CHNL_BW_40:
		u4SupportedBw = 40;
		break;

	case NAN_CHNL_BW_80:
		u4SupportedBw = 80;
		break;

	case NAN_CHNL_BW_160:
		u4SupportedBw = 160;
		break;
	}

	return u4SupportedBw;
}

uint32_t
nanSchedConfigDefNdlNumSlots(struct ADAPTER *prAdapter, uint32_t u4NumSlots) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	if (u4NumSlots < NAN_DEFAULT_NDL_QUOTA_LOW_BOUND)
		u4NumSlots = NAN_DEFAULT_NDL_QUOTA_LOW_BOUND;
	else if (u4NumSlots > NAN_DEFAULT_NDL_QUOTA_UP_BOUND)
		u4NumSlots = NAN_DEFAULT_NDL_QUOTA_UP_BOUND;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prNegoCtrl->u4DefNdlNumSlots = u4NumSlots;

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedConfigDefRangingNumSlots(struct ADAPTER *prAdapter,
				 uint32_t u4NumSlots) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	if (u4NumSlots < NAN_DEFAULT_RANG_QUOTA_LOW_BOUND)
		u4NumSlots = NAN_DEFAULT_RANG_QUOTA_LOW_BOUND;
	else if (u4NumSlots > NAN_DEFAULT_RANG_QUOTA_UP_BOUND)
		u4NumSlots = NAN_DEFAULT_RANG_QUOTA_UP_BOUND;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prNegoCtrl->u4DefRangingNumSlots = u4NumSlots;

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedConfigAllowedBand(struct ADAPTER *prAdapter, unsigned char fgEn2g,
			  unsigned char fgEn5gH, unsigned char fgEn5gL) {
	struct _NAN_SCHEDULER_T *prNanScheduler;
	/* whsu */
	/* UINT_8 ucDiscChnlBw = BW_20; */
	uint8_t ucDiscChnlBw = MAX_BW_20MHZ;

	prNanScheduler = nanGetScheduler(prAdapter);

	prNanScheduler->fgEn2g = fgEn2g;
	prNanScheduler->fgEn5gH = fgEn5gH;
	prNanScheduler->fgEn5gL = fgEn5gL;

	DBGLOG(NAN, INFO, "Allowed Band: %d, %d, %d\n", fgEn2g, fgEn5gH,
	       fgEn5gL);

	ucDiscChnlBw = prAdapter->rWifiVar.ucNanBandwidth;
	if ((!prAdapter->rWifiVar.fgEnNanVHT) &&
	    (ucDiscChnlBw > NAN_CHNL_BW_40))
		ucDiscChnlBw = NAN_CHNL_BW_40;

	g_r2gDwChnl.rChannel.u4Type = NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL;
	g_r2gDwChnl.rChannel.u4AuxCenterChnl = 0;
	g_r2gDwChnl.rChannel.u4PrimaryChnl = NAN_2P4G_DISC_CHANNEL;
	if (ucDiscChnlBw == NAN_CHNL_BW_20)
		g_r2gDwChnl.rChannel.u4OperatingClass =
			NAN_2P4G_DISC_CH_OP_CLASS;
	else
		g_r2gDwChnl.rChannel.u4OperatingClass =
			NAN_2P4G_BW40_DISC_CH_OP_CLASS;

	g_r5gDwChnl.rChannel.u4Type = NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL;
	g_r5gDwChnl.rChannel.u4AuxCenterChnl = 0;
	if (fgEn5gH) {
		g_r5gDwChnl.rChannel.u4PrimaryChnl = NAN_5G_HIGH_DISC_CHANNEL;

		if (ucDiscChnlBw == NAN_CHNL_BW_20)
			g_r5gDwChnl.rChannel.u4OperatingClass =
				NAN_5G_HIGH_DISC_CH_OP_CLASS;
		else if (ucDiscChnlBw == NAN_CHNL_BW_40)
			g_r5gDwChnl.rChannel.u4OperatingClass =
				NAN_5G_HIGH_BW40_DISC_CH_OP_CLASS;
		else
			g_r5gDwChnl.rChannel.u4OperatingClass =
				NAN_5G_HIGH_BW80_DISC_CH_OP_CLASS;
	} else if (fgEn5gL) {
		g_r5gDwChnl.rChannel.u4PrimaryChnl = NAN_5G_LOW_DISC_CHANNEL;

		if (ucDiscChnlBw == NAN_CHNL_BW_20)
			g_r5gDwChnl.rChannel.u4OperatingClass =
				NAN_5G_LOW_DISC_CH_OP_CLASS;
		else if (ucDiscChnlBw == NAN_CHNL_BW_40)
			g_r5gDwChnl.rChannel.u4OperatingClass =
				NAN_5G_LOW_BW40_DISC_CH_OP_CLASS;
		else
			g_r5gDwChnl.rChannel.u4OperatingClass =
				NAN_5G_LOW_BW80_DISC_CH_OP_CLASS;
	}

	if (fgEn2g)
		g_rPreferredChnl = g_r2gDwChnl;

	if (fgEn5gH)
		g_rPreferredChnl = g_r5gDwChnl;
	else if (fgEn5gL)
		g_rPreferredChnl = g_r5gDwChnl;

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedConfigPhyParams(struct ADAPTER *prAdapter) {
#define VHT_CAP_INFO_NSS_MAX 8

	struct WIFI_VAR *prWifiVar = &prAdapter->rWifiVar;

	struct _NAN_PHY_SETTING_T r2P4GPhySettings;
	struct _NAN_PHY_SETTING_T r5GPhySettings;
	struct _NAN_PHY_SETTING_T *prPhySettings;

	enum ENUM_PARAM_NAN_MODE_T eNanMode;
	const struct NON_HT_PHY_ATTRIBUTE *prLegacyPhyAttr;
	const struct NON_HT_ADHOC_MODE_ATTRIBUTE *prLegacyModeAttr;
	uint8_t ucLegacyPhyTp;
	int32_t i;
	uint8_t ucOffset;
	uint8_t ucNss;
	uint8_t ucMcsMap;
	uint8_t ucVhtCapInfoNssOfst[VHT_CAP_INFO_NSS_MAX] = {
		VHT_CAP_INFO_MCS_1SS_OFFSET, VHT_CAP_INFO_MCS_2SS_OFFSET,
		VHT_CAP_INFO_MCS_3SS_OFFSET, VHT_CAP_INFO_MCS_4SS_OFFSET,
		VHT_CAP_INFO_MCS_5SS_OFFSET, VHT_CAP_INFO_MCS_6SS_OFFSET,
		VHT_CAP_INFO_MCS_7SS_OFFSET, VHT_CAP_INFO_MCS_8SS_OFFSET
	};

	/* 2.4G */
	prPhySettings = &r2P4GPhySettings;

	eNanMode = NAN_MODE_MIXED_11BG;
	prLegacyModeAttr = &rNonHTNanModeAttr[eNanMode];
	ucLegacyPhyTp = (uint8_t)prLegacyModeAttr->ePhyTypeIndex;
	prLegacyPhyAttr = &rNonHTPhyAttributes[ucLegacyPhyTp];

	prPhySettings->ucPhyTypeSet =
		prWifiVar->ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11BGN;
	prPhySettings->ucNonHTBasicPhyType = ucLegacyPhyTp;
	if (prLegacyPhyAttr->fgIsShortPreambleOptionImplemented &&
	    (prWifiVar->ePreambleType == PREAMBLE_TYPE_SHORT ||
	     prWifiVar->ePreambleType == PREAMBLE_TYPE_AUTO))
		prPhySettings->fgUseShortPreamble = TRUE;
	else
		prPhySettings->fgUseShortPreamble = FALSE;
	prPhySettings->fgUseShortSlotTime =
		prLegacyPhyAttr->fgIsShortSlotTimeOptionImplemented;

	prPhySettings->u2OperationalRateSet =
		prLegacyPhyAttr->u2SupportedRateSet;
	prPhySettings->u2BSSBasicRateSet = prLegacyModeAttr->u2BSSBasicRateSet;
	/* Mask CCK 1M For Sco scenario except FDD mode */
	if (prAdapter->u4FddMode == FALSE)
		prPhySettings->u2BSSBasicRateSet &= ~RATE_SET_BIT_1M;
	prPhySettings->u2VhtBasicMcsSet = 0;

	prPhySettings->ucHtOpInfo1 = 0; /* FW configues SCN, SCA or SCB */
	prPhySettings->u2HtOpInfo2 = 0;
	prPhySettings->u2HtOpInfo3 = 0;
	prPhySettings->fgErpProtectMode = FALSE;
	prPhySettings->eHtProtectMode = HT_PROTECT_MODE_NONE;
	prPhySettings->eGfOperationMode = GF_MODE_DISALLOWED;
	prPhySettings->eRifsOperationMode = RIFS_MODE_DISALLOWED;

	/* 5G */
	prPhySettings = &r5GPhySettings;

	eNanMode = NAN_MODE_11A;
	prLegacyModeAttr = &rNonHTNanModeAttr[eNanMode];
	ucLegacyPhyTp = (uint8_t)prLegacyModeAttr->ePhyTypeIndex;
	prLegacyPhyAttr = &rNonHTPhyAttributes[ucLegacyPhyTp];

	prPhySettings->ucPhyTypeSet =
		prWifiVar->ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11ANAC;
	prPhySettings->ucNonHTBasicPhyType = ucLegacyPhyTp;
	if (prLegacyPhyAttr->fgIsShortPreambleOptionImplemented &&
	    (prWifiVar->ePreambleType == PREAMBLE_TYPE_SHORT ||
	     prWifiVar->ePreambleType == PREAMBLE_TYPE_AUTO))
		prPhySettings->fgUseShortPreamble = TRUE;
	else
		prPhySettings->fgUseShortPreamble = FALSE;
	prPhySettings->fgUseShortSlotTime =
		prLegacyPhyAttr->fgIsShortSlotTimeOptionImplemented;

	prPhySettings->u2OperationalRateSet =
		prLegacyPhyAttr->u2SupportedRateSet;
	prPhySettings->u2BSSBasicRateSet = prLegacyModeAttr->u2BSSBasicRateSet;
	/* Mask CCK 1M For Sco scenario except FDD mode */
	if (prAdapter->u4FddMode == FALSE)
		prPhySettings->u2BSSBasicRateSet &= ~RATE_SET_BIT_1M;

	/* Filled the VHT BW/S1/S2 and MCS rate set */
	ucNss = prAdapter->rWifiVar.ucNSS;
#if CFG_SISO_SW_DEVELOP
	if (IS_WIFI_5G_SISO(prAdapter))
		ucNss = 1;
#endif
	if (prPhySettings->ucPhyTypeSet & PHY_TYPE_BIT_VHT) {
		prPhySettings->u2VhtBasicMcsSet = 0;
		for (i = 0; i < VHT_CAP_INFO_NSS_MAX; i++) {
			ucOffset = ucVhtCapInfoNssOfst[i];

			if (i < ucNss)
				ucMcsMap = VHT_CAP_INFO_MCS_MAP_MCS9;
			else
				ucMcsMap = VHT_CAP_INFO_MCS_NOT_SUPPORTED;

			prPhySettings->u2VhtBasicMcsSet |=
				(ucMcsMap << ucOffset);
		}
	} else {
		prPhySettings->u2VhtBasicMcsSet = 0;
	}

	prPhySettings->ucHtOpInfo1 = 0; /* FW configues SCN, SCA or SCB */
	prPhySettings->u2HtOpInfo2 = 0;
	prPhySettings->u2HtOpInfo3 = 0;
	prPhySettings->fgErpProtectMode = FALSE;
	prPhySettings->eHtProtectMode = HT_PROTECT_MODE_NONE;
	prPhySettings->eGfOperationMode = GF_MODE_DISALLOWED;
	prPhySettings->eRifsOperationMode = RIFS_MODE_DISALLOWED;

	nanSchedCmdUpdatePhySettigns(prAdapter, &r2P4GPhySettings,
				     &r5GPhySettings);

	return WLAN_STATUS_SUCCESS;
}

void
nanSchedReleaseUnusedCommitSlot(struct ADAPTER *prAdapter) {
	uint32_t au4UsedAvailMap[NAN_TOTAL_DW];
	struct _NAN_SCHEDULE_TIMELINE_T *prTimeline;
	uint32_t u4Idx, u4Idx1;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimeline;
	uint32_t u4SlotIdx;
	uint32_t u4SchIdx;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_NDC_CTRL_T *prNdcCtrl;
	struct _NAN_SCHEDULER_T *prScheduler;

	prScheduler = nanGetScheduler(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	kalMemZero((uint8_t *)au4UsedAvailMap, sizeof(au4UsedAvailMap));

	for (u4SchIdx = 0; u4SchIdx < NAN_MAX_CONN_CFG; u4SchIdx++) {
		prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
		if (prPeerSchRecord == NULL)
			continue;

#if 1
		if (prPeerSchRecord->fgActive == FALSE)
			continue;

		if (prPeerSchRecord->fgUseDataPath == TRUE) {
			prTimeline = &prPeerSchRecord->rCommFawTimeline;
			for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++)
				au4UsedAvailMap[u4Idx] |=
					prTimeline->au4AvailMap[u4Idx];
		} else if (prPeerSchRecord->fgUseRanging == TRUE) {
			prTimeline = &prPeerSchRecord->rCommRangingTimeline;
			for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++)
				au4UsedAvailMap[u4Idx] |=
					prTimeline->au4AvailMap[u4Idx];
		}
#else
		if ((prPeerSchRecord->fgActive == FALSE) ||
		    ((prPeerSchRecord->fgUseDataPath == FALSE) &&
		     (prPeerSchRecord->fgUseRanging == FALSE)))
			continue;

		prTimeline = &prPeerSchRecord->rCommFawTimeline;
		for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++)
			au4UsedAvailMap[u4Idx] |=
				prTimeline->au4AvailMap[u4Idx];
#endif
	}

	for (u4Idx = 0; u4Idx < NAN_MAX_NDC_RECORD; u4Idx++) {
		prNdcCtrl = &prScheduler->arNdcCtrl[u4Idx];
		if (prNdcCtrl->fgValid == FALSE)
			continue;

		/* release the NDC if there's no any peer occupying it */
		for (u4SchIdx = 0; u4SchIdx < NAN_MAX_CONN_CFG; u4SchIdx++) {
			prPeerSchRecord =
				nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
			if (prPeerSchRecord == NULL)
				continue;

			if (prPeerSchRecord->fgActive == FALSE)
				continue;

			if (prPeerSchRecord->prCommNdcCtrl == prNdcCtrl)
				break;
		}
		if (u4SchIdx >= NAN_MAX_CONN_CFG) {
			prNdcCtrl->fgValid = FALSE;
			continue;
		}

		for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++)
			au4UsedAvailMap[u4Idx] |=
				prNdcCtrl->arTimeline[0].au4AvailMap[u4Idx];
	}

	/* consider custom FAW */
	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx++) {
		prChnlTimeline = &prNanTimelineMgmt->arCustChnlList[u4Idx];
		if (prChnlTimeline->fgValid == FALSE)
			continue;

		for (u4Idx1 = 0; u4Idx1 < NAN_TOTAL_DW; u4Idx1++)
			au4UsedAvailMap[u4Idx1] |=
				prChnlTimeline->au4AvailMap[u4Idx1];
	}

	for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS; u4SlotIdx++) {
		if (NAN_IS_AVAIL_MAP_SET(au4UsedAvailMap, u4SlotIdx) == TRUE)
			continue;

		nanSchedDeleteCrbFromChnlList(prAdapter, u4SlotIdx, 1,
					      ENUM_TIME_BITMAP_CTRL_PERIOD_8192,
					      TRUE);
	}

	/* nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__); */
}

void
nanSchedDropResources(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
		      enum _ENUM_NAN_NEGO_TYPE_T eType) {
	uint32_t u4SchIdx;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	uint32_t u4Idx;

	u4SchIdx = nanSchedLookupPeerSchRecordIdx(prAdapter, pucNmiAddr);
	if (u4SchIdx == NAN_INVALID_SCH_RECORD)
		return;

	prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	prPeerSchDesc = prPeerSchRecord->prPeerSchDesc;

	DBGLOG(NAN, INFO, "\n\n");
	DBGLOG(NAN, INFO, "------>\n");
	DBGLOG(NAN, INFO, "Drop %02x:%02x:%02x:%02x:%02x:%02x Type:%d\n",
	       pucNmiAddr[0], pucNmiAddr[1], pucNmiAddr[2], pucNmiAddr[3],
	       pucNmiAddr[4], pucNmiAddr[5], eType);

	switch (eType) {
	case ENUM_NAN_NEGO_DATA_LINK:
		prPeerSchRecord->fgUseDataPath = FALSE;
		prPeerSchRecord->u4FinalQosMaxLatency =
			NAN_INVALID_QOS_MAX_LATENCY;
		prPeerSchRecord->u4FinalQosMinSlots = NAN_INVALID_QOS_MIN_SLOTS;

		prPeerSchDesc->fgImmuNdlTimelineValid = FALSE;
		for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++)
			prPeerSchDesc->arImmuNdlTimeline[u4Idx].ucMapId =
				NAN_INVALID_MAP_ID;
		prPeerSchDesc->rSelectedNdcCtrl.fgValid = FALSE;
		prPeerSchDesc->u4QosMaxLatency = NAN_INVALID_QOS_MAX_LATENCY;
		prPeerSchDesc->u4QosMinSlots = NAN_INVALID_QOS_MIN_SLOTS;
		break;

	case ENUM_NAN_NEGO_RANGING:
		prPeerSchRecord->fgUseRanging = FALSE;

		prPeerSchDesc->fgRangingTimelineValid = FALSE;
		for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++)
			prPeerSchDesc->arRangingTimeline[u4Idx].ucMapId =
				NAN_INVALID_MAP_ID;
		break;

	default:
		break;
	}

	if (prPeerSchRecord->fgActive && !prPeerSchRecord->fgUseDataPath &&
	    !prPeerSchRecord->fgUseRanging) {
		nanSchedReleasePeerSchedRecord(prAdapter, u4SchIdx);

		/* release unused commit slot */
		if (!nanSchedNegoInProgress(prAdapter)) {
			nanSchedReleaseUnusedCommitSlot(prAdapter);
			nanSchedCmdUpdateAvailability(prAdapter);
		}
	}

	nanSchedDumpPeerSchDesc(prAdapter, prPeerSchDesc);
	nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
	DBGLOG(NAN, INFO, "<------\n");
}

void
nanSchedNegoDumpState(struct ADAPTER *prAdapter, uint8_t *pucFunc,
		      uint32_t u4Line) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	DBGLOG(NAN, INFO, "\n\n");
	DBGLOG(NAN, INFO, "------>\n");
	DBGLOG(NAN, INFO, "#%s@%d\n", pucFunc, u4Line);
	DBGLOG(NAN, INFO, "Role:%d, Type:%d, State:%d\n", prNegoCtrl->eRole,
	       prNegoCtrl->eType, prNegoCtrl->eState);

	do {
		prPeerSchRecord = nanSchedGetPeerSchRecord(
			prAdapter, prNegoCtrl->u4SchIdx);
		if (!prPeerSchRecord)
			break;

		prPeerSchDesc = prPeerSchRecord->prPeerSchDesc;
		if (!prPeerSchDesc)
			break;

		nanSchedDumpPeerSchDesc(prAdapter, prPeerSchDesc);
	} while (FALSE);

	nanSchedDbgDumpTimelineDb(prAdapter, pucFunc, u4Line);
	DBGLOG(NAN, INFO, "<------\n\n");
}

union _NAN_BAND_CHNL_CTRL
nanSchedNegoSelectChnlInfo(struct ADAPTER *prAdapter, uint32_t u4SlotIdx) {
	uint32_t u4SchIdx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	union _NAN_BAND_CHNL_CTRL rSelChnlInfo;
	union _NAN_BAND_CHNL_CTRL rSelBandInfo;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	struct _NAN_SCHEDULER_T *prNanScheduler;
	uint32_t u4AvailDbIdx;
	uint32_t u4BandIdMask;

	prNanScheduler = nanGetScheduler(prAdapter);

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	u4SchIdx = prNegoCtrl->u4SchIdx;
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, u4SchIdx);

	if (!prPeerSchDesc) {
		DBGLOG(NAN, ERROR, "prPeerSchDesc error!\n");
		return g_rPreferredChnl;
	}

	if (prNegoCtrl->eState == ENUM_NAN_CRB_NEGO_STATE_INITIATOR) {
		rSelChnlInfo = nanSchedGetFixedChnlInfo(prAdapter);
		if (rSelChnlInfo.rChannel.u4PrimaryChnl != 0)
			return rSelChnlInfo;
	}

	/* commit channel */
	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
	     u4AvailDbIdx++) {
		if (nanSchedPeerAvailabilityDbValidByDesc(
			    prAdapter, prPeerSchDesc, u4AvailDbIdx) == FALSE)
			continue;

		rSelChnlInfo = nanQueryPeerChnlInfoBySlot(
			prAdapter, u4SchIdx, u4AvailDbIdx, u4SlotIdx, TRUE);
		if (rSelChnlInfo.rChannel.u4PrimaryChnl == 0)
			continue;
		else if (!nanIsAllowedChannel(
				 prAdapter,
				 rSelChnlInfo.rChannel.u4PrimaryChnl))
			continue;
		else {
			rSelChnlInfo = nanSchedConvergeChnlInfo(prAdapter,
								rSelChnlInfo);
			if (rSelChnlInfo.rChannel.u4PrimaryChnl != 0)
				return rSelChnlInfo;
		}
	}

	/* conditional channel */
	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
	     u4AvailDbIdx++) {
		if (nanSchedPeerAvailabilityDbValidByDesc(
			    prAdapter, prPeerSchDesc, u4AvailDbIdx) == FALSE)
			continue;

		rSelChnlInfo = nanQueryPeerChnlInfoBySlot(
			prAdapter, u4SchIdx, u4AvailDbIdx, u4SlotIdx, FALSE);
		if (rSelChnlInfo.rChannel.u4PrimaryChnl == 0)
			continue;
		else if (!nanIsAllowedChannel(
				 prAdapter,
				 rSelChnlInfo.rChannel.u4PrimaryChnl))
			continue;
		else {
			rSelChnlInfo = nanSchedConvergeChnlInfo(prAdapter,
								rSelChnlInfo);
			if (rSelChnlInfo.rChannel.u4PrimaryChnl != 0)
				return rSelChnlInfo;
		}
	}

	/* potential channel */
	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
	     u4AvailDbIdx++) {
		rSelChnlInfo = nanQueryPeerPotentialChnlInfoBySlot(
			prAdapter, u4SchIdx, u4AvailDbIdx, u4SlotIdx);
		if (rSelChnlInfo.rChannel.u4PrimaryChnl == 0)
			continue;
		else if (!nanIsAllowedChannel(
				 prAdapter,
				 rSelChnlInfo.rChannel.u4PrimaryChnl))
			continue;
		else {
			rSelChnlInfo = nanSchedConvergeChnlInfo(prAdapter,
								rSelChnlInfo);
			if (rSelChnlInfo.rChannel.u4PrimaryChnl != 0)
				return rSelChnlInfo;
		}
	}

	/* potential band */
	u4BandIdMask = 0;
	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
	     u4AvailDbIdx++) {
		rSelBandInfo = nanQueryPeerPotentialBandInfoBySlot(
			prAdapter, u4SchIdx, u4AvailDbIdx, u4SlotIdx);
		u4BandIdMask |= rSelBandInfo.rBand.u4BandIdMask;
	}
	if (((u4BandIdMask & BIT(NAN_SUPPORTED_BAND_ID_5G)) != 0) &&
	    (prNanScheduler->fgEn5gH || prNanScheduler->fgEn5gL))
		return g_r5gDwChnl;
	else if (((u4BandIdMask & BIT(NAN_SUPPORTED_BAND_ID_2P4G)) != 0) &&
		 (prNanScheduler->fgEn2g))
		return g_r2gDwChnl;

	return g_rPreferredChnl;
}

uint32_t
nanSchedNegoDecideChnlInfoForEmptySlot(struct ADAPTER *prAdapter,
				       uint32_t au4EmptyMap[NAN_TOTAL_DW]) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4SlotIdx;
	union _NAN_BAND_CHNL_CTRL rSelChnlInfo;

	for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS; u4SlotIdx++) {
		if (NAN_IS_AVAIL_MAP_SET(au4EmptyMap, u4SlotIdx)) {
			rSelChnlInfo = nanSchedNegoSelectChnlInfo(prAdapter,
								  u4SlotIdx);
			if (rSelChnlInfo.rChannel.u4PrimaryChnl == 0) {
				DBGLOG(NAN, ERROR, "u4PrimaryChnl equals 0!\n");
				return WLAN_STATUS_FAILURE;
			}
			nanSchedAddCrbToChnlList(
				prAdapter, &rSelChnlInfo, u4SlotIdx, 1,
				ENUM_TIME_BITMAP_CTRL_PERIOD_8192, FALSE, NULL);
		}
	}

	return rRetStatus;
}

struct _NAN_PEER_SCHEDULE_RECORD_T *
nanSchedNegoSyncSchFindNextPeerSchRec(struct ADAPTER *prAdapter) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;
	uint32_t u4SchIdx;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	for (u4SchIdx = prNegoCtrl->u4SyncPeerSchRecIdx;
	     u4SchIdx < NAN_MAX_CONN_CFG; u4SchIdx++) {
		prPeerSchRec = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
		if ((prPeerSchRec == NULL) ||
		    (prPeerSchRec->fgActive == FALSE) ||
		    (prPeerSchRec->prPeerSchDesc == NULL))
			continue;

		prNegoCtrl->u4SyncPeerSchRecIdx =
			u4SchIdx + 1; /* point to next record */
		return prPeerSchRec;
	}

	prNegoCtrl->u4SyncPeerSchRecIdx = u4SchIdx;
	return NULL;
}

static enum _ENUM_NAN_SYNC_SCH_UPDATE_STATE_T
nanSchedNegoSyncSchUpdateFsmStep(
	IN struct ADAPTER *prAdapter,
	IN enum _ENUM_NAN_SYNC_SCH_UPDATE_STATE_T eNextState) {
	enum _ENUM_NAN_SYNC_SCH_UPDATE_STATE_T eLastState;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;
	struct _NAN_PARAMETER_NDL_SCH rNanUpdateSchParam;
	uint32_t rStatus;
	uint8_t *pucNmiAddr;
	uint32_t u4SchIdx;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	do {
		DBGLOG(NAN, INFO, "SCH UPDATE STATE: [%d] -> [%d]\n",
		       prNegoCtrl->eSyncSchUpdateCurrentState, eNextState);

		prNegoCtrl->eSyncSchUpdateLastState =
			prNegoCtrl->eSyncSchUpdateCurrentState;
		prNegoCtrl->eSyncSchUpdateCurrentState = eNextState;

		eLastState = eNextState;

		switch (eNextState) {
		case ENUM_NAN_SYNC_SCH_UPDATE_STATE_IDLE:
			/* stable state */

			/* check all peers to see if any one conflicts
			 * the new schedule
			 */
			nanSchedReleaseUnusedCommitSlot(prAdapter);
			nanSchedCmdUpdateAvailability(prAdapter);

			for (u4SchIdx = 0; u4SchIdx < NAN_MAX_CONN_CFG;
			     u4SchIdx++) {
				prPeerSchRec = nanSchedGetPeerSchRecord(
					prAdapter, u4SchIdx);
				if (!prPeerSchRec->fgActive ||
				    (prPeerSchRec->prPeerSchDesc == NULL))
					continue;

				nanSchedPeerChkAvailability(
					prAdapter, prPeerSchRec->prPeerSchDesc);
			}
			break;

		case ENUM_NAN_SYNC_SCH_UPDATE_STATE_PREPARE:
			if (prNegoCtrl->ucNegoTransNum == 0)
				eNextState =
					ENUM_NAN_SYNC_SCH_UPDATE_STATE_CHECK;
			break;

		case ENUM_NAN_SYNC_SCH_UPDATE_STATE_CHECK:
			if (prNegoCtrl->eSyncSchUpdateLastState !=
			    ENUM_NAN_SYNC_SCH_UPDATE_STATE_RUN) {
				nanSchedNegoApplyCustChnlList(prAdapter);
				prNegoCtrl->u4SyncPeerSchRecIdx =
					0; /* reset to first record */

				/* update availability schedule to FW */
				nanSchedCmdUpdateAvailability(prAdapter);
			}

			do {
				rStatus = WLAN_STATUS_SUCCESS;

				prPeerSchRec =
					nanSchedNegoSyncSchFindNextPeerSchRec(
						prAdapter);
				if (prPeerSchRec == NULL)
					eNextState =
					ENUM_NAN_SYNC_SCH_UPDATE_STATE_DONE;
				else {
#if 0
					/* check if schedule conflict */
					if ((nanSchedPeerChkDataPath(
					  prAdapter, prPeerSchRec)
					== WLAN_STATUS_SUCCESS) &&
					(nanSchedPeerChkRanging(
					prAdapter, prPeerSchRec)
					== WLAN_STATUS_SUCCESS)) {
						continue;
				    }
#endif

				    /* start schedule update */
				    pucNmiAddr = prPeerSchRec->prPeerSchDesc
							     ->aucNmiAddr;
				    DBGLOG(NAN, INFO,
					"Update Sch for %02x:%02x:%02x:%02x:%02x:%02x\n",
					pucNmiAddr[0], pucNmiAddr[1],
					pucNmiAddr[2], pucNmiAddr[3],
					pucNmiAddr[4], pucNmiAddr[5]);
				    kalMemZero(&rNanUpdateSchParam,
					sizeof(rNanUpdateSchParam));
				    kalMemCopy(rNanUpdateSchParam
					.aucPeerDataAddress,
					pucNmiAddr, MAC_ADDR_LEN);
				    rStatus = nanUpdateNdlSchedule(
					prAdapter, &rNanUpdateSchParam);
				    if (rStatus == WLAN_STATUS_SUCCESS)
					eNextState =
					    ENUM_NAN_SYNC_SCH_UPDATE_STATE_RUN;
				}
			} while (rStatus != WLAN_STATUS_SUCCESS);
			break;

		case ENUM_NAN_SYNC_SCH_UPDATE_STATE_RUN:
			if (prNegoCtrl->ucNegoTransNum == 0)
				eNextState =
					ENUM_NAN_SYNC_SCH_UPDATE_STATE_CHECK;
			break;

		case ENUM_NAN_SYNC_SCH_UPDATE_STATE_DONE:

			/* Fixme: send schedule update notification frames
			 * to all peers
			 */

			nanSchedDbgDumpTimelineDb(prAdapter, __func__,
						  __LINE__);

			eNextState = ENUM_NAN_SYNC_SCH_UPDATE_STATE_IDLE;
			break;

		default:
			break;
		}
	} while (eLastState != eNextState);

	return eNextState;
}

uint32_t
nanSchedNegoCustFawResetCmd(struct ADAPTER *prAdapter) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	kalMemZero(prNanTimelineMgmt->arCustChnlList,
		   sizeof(prNanTimelineMgmt->arCustChnlList));

	return rRetStatus;
}

uint32_t
nanSchedNegoCustFawConfigCmd(struct ADAPTER *prAdapter, uint8_t ucChnl,
			     uint32_t u4SlotBitmap) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	union _NAN_BAND_CHNL_CTRL rChnlInfo;
	uint32_t u4Bw;
	uint32_t u4Idx, u4DwIdx;
	uint32_t u4TargetIdx = NAN_TIMELINE_MGMT_CHNL_LIST_NUM;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimeline;
	struct _NAN_SCHEDULER_T *prScheduler;

	prScheduler = nanGetScheduler(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	u4Bw = nanSchedConfigGetAllowedBw(prAdapter, ucChnl);
	rChnlInfo = nanRegGenNanChnlInfoByPriChannel(ucChnl, u4Bw);

	if (prScheduler->fgEn2g)
		u4SlotBitmap &= ~(BIT(NAN_2G_DW_INDEX));
	if (prScheduler->fgEn5gH || prScheduler->fgEn5gL)
		u4SlotBitmap &= ~(BIT(NAN_5G_DW_INDEX));

	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx++) {
		prChnlTimeline = &prNanTimelineMgmt->arCustChnlList[u4Idx];
		if (prChnlTimeline->fgValid == FALSE) {
			if (u4TargetIdx == NAN_TIMELINE_MGMT_CHNL_LIST_NUM)
				u4TargetIdx = u4Idx;

			continue;
		}

		if (prChnlTimeline->rChnlInfo.rChannel.u4PrimaryChnl ==
		    ucChnl) {
			u4TargetIdx = u4Idx;
			continue;
		}

		for (u4DwIdx = 0; u4DwIdx < NAN_TOTAL_DW; u4DwIdx++)
			prChnlTimeline->au4AvailMap[u4DwIdx] &= (~u4SlotBitmap);
		prChnlTimeline->i4Num = nanUtilCheckBitOneCnt(
			prAdapter, (uint8_t *)prChnlTimeline->au4AvailMap,
			sizeof(prChnlTimeline->au4AvailMap));
		if (prChnlTimeline->i4Num == 0)
			prChnlTimeline->fgValid = FALSE;
	}

	if (u4TargetIdx != NAN_TIMELINE_MGMT_CHNL_LIST_NUM) {
		prChnlTimeline =
			&prNanTimelineMgmt->arCustChnlList[u4TargetIdx];
		prChnlTimeline->rChnlInfo = rChnlInfo;
		for (u4DwIdx = 0; u4DwIdx < NAN_TOTAL_DW; u4DwIdx++)
			prChnlTimeline->au4AvailMap[u4DwIdx] |= u4SlotBitmap;
		prChnlTimeline->i4Num = nanUtilCheckBitOneCnt(
			prAdapter, (uint8_t *)prChnlTimeline->au4AvailMap,
			sizeof(prChnlTimeline->au4AvailMap));
		if (prChnlTimeline->i4Num)
			prChnlTimeline->fgValid = TRUE;
	}

	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx++) {
		prChnlTimeline = &prNanTimelineMgmt->arCustChnlList[u4Idx];
		if (prChnlTimeline->fgValid == FALSE)
			continue;

		DBGLOG(NAN, INFO,
		       "[%d] Raw:0x%x, Cust Chnl:%d, Class:%d, Bw:%d\n", u4Idx,
		       prChnlTimeline->rChnlInfo.u4RawData,
		       prChnlTimeline->rChnlInfo.rChannel.u4PrimaryChnl,
		       prChnlTimeline->rChnlInfo.rChannel.u4OperatingClass,
		       nanRegGetBw(prChnlTimeline->rChnlInfo.rChannel
					   .u4OperatingClass));
		nanUtilDump(prAdapter, "AvailMap",
			    (uint8_t *)prChnlTimeline->au4AvailMap,
			    sizeof(prChnlTimeline->au4AvailMap));
	}

	return rRetStatus;
}

uint32_t
nanSchedNegoCustFawApplyCmd(struct ADAPTER *prAdapter) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;

	nanSchedNegoSyncSchUpdateFsmStep(
		prAdapter, ENUM_NAN_SYNC_SCH_UPDATE_STATE_PREPARE);

	return rRetStatus;
}

void
nanSchedNegoDispatchTimeout(struct ADAPTER *prAdapter, unsigned long ulParam) {
	uint32_t u4Idx;
	uint32_t u4SchIdx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	if (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_IDLE)
		return;

	DBGLOG(NAN, INFO, "Num:%d\n", prNegoCtrl->ucNegoTransNum);

	while (prNegoCtrl->ucNegoTransNum > 0) {
		u4Idx = prNegoCtrl->ucNegoTransHeadPos;
		prNegoCtrl->ucNegoTransHeadPos =
			(prNegoCtrl->ucNegoTransHeadPos + 1) %
			NAN_CRB_NEGO_MAX_TRANSACTION;
		prNegoCtrl->ucNegoTransNum--;

		u4SchIdx = nanSchedLookupPeerSchRecordIdx(
			prAdapter, prNegoCtrl->rNegoTrans[u4Idx].aucNmiAddr);
		if (u4SchIdx == NAN_INVALID_SCH_RECORD) {
			DBGLOG(NAN, ERROR,
				"u4SchIdx equals NAN_INVALID_SCH_RECORD!\n");
			return;
		}

		nanSchedNegoInitDb(prAdapter, u4SchIdx,
				   prNegoCtrl->rNegoTrans[u4Idx].eType,
				   prNegoCtrl->rNegoTrans[u4Idx].eRole);
		prNanTimelineMgmt->fgChkCondAvailability = TRUE;

		nanSchedNegoDumpState(prAdapter,
			(uint8_t *) __func__, __LINE__);

		prNegoCtrl->rNegoTrans[u4Idx].pfCallback(
			prAdapter, prNegoCtrl->rNegoTrans[u4Idx].aucNmiAddr,
			prNegoCtrl->rNegoTrans[u4Idx].eType,
			prNegoCtrl->rNegoTrans[u4Idx].eRole,
			prNegoCtrl->rNegoTrans[u4Idx].pvToken);
		break;
	}
}

void
nanSchedNegoInitDb(struct ADAPTER *prAdapter, uint32_t u4SchIdx,
		   enum _ENUM_NAN_NEGO_TYPE_T eType,
		   enum _ENUM_NAN_NEGO_ROLE_T eRole) {
	uint32_t u4Idx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	uint32_t u4DftNdlQosLatencyVal = NAN_INVALID_QOS_MAX_LATENCY;
	uint32_t u4DftNdlQosQuotaVal = NAN_INVALID_QOS_MIN_SLOTS;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	prNegoCtrl->u4SchIdx = u4SchIdx;
	prNegoCtrl->eRole = eRole;
	prNegoCtrl->eType = eType;
	if (eRole == ENUM_NAN_NEGO_ROLE_INITIATOR)
		prNegoCtrl->eState = ENUM_NAN_CRB_NEGO_STATE_INITIATOR;
	else {
		nanSchedPeerPurgeOldCrb(prAdapter, u4SchIdx);
		prNegoCtrl->eState = ENUM_NAN_CRB_NEGO_STATE_RESPONDER;
	}

	prNegoCtrl->rSelectedNdcCtrl.fgValid = FALSE;
	kalMemZero(&prNegoCtrl->rSelectedNdcCtrl,
		   sizeof(prNegoCtrl->rSelectedNdcCtrl));
	prNegoCtrl->rImmuNdlTimeline.ucMapId = NAN_INVALID_MAP_ID;
	prNegoCtrl->rRangingTimeline.ucMapId = NAN_INVALID_MAP_ID;

	prNegoCtrl->u4QosMaxLatency = NAN_INVALID_QOS_MAX_LATENCY;
	prNegoCtrl->u4QosMinSlots = NAN_INVALID_QOS_MIN_SLOTS;
	prNegoCtrl->u4NegoQosMaxLatency = NAN_INVALID_QOS_MAX_LATENCY;
	prNegoCtrl->u4NegoQosMinSlots = NAN_INVALID_QOS_MIN_SLOTS;

	kalMemZero(prNegoCtrl->au4AvailSlots,
		   sizeof(prNegoCtrl->au4AvailSlots));
	kalMemZero(prNegoCtrl->au4UnavailSlots,
		   sizeof(prNegoCtrl->au4UnavailSlots));
	kalMemZero(prNegoCtrl->au4CondSlots, sizeof(prNegoCtrl->au4CondSlots));

	prNanTimelineMgmt->fgChkCondAvailability = FALSE;
	for (u4Idx = 0; u4Idx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM; u4Idx++)
		prNanTimelineMgmt->arCondChnlList[u4Idx].fgValid = FALSE;

	if (prAdapter->rWifiVar.u2DftNdlQosLatencyVal)
		u4DftNdlQosLatencyVal =
			prAdapter->rWifiVar.u2DftNdlQosLatencyVal;

	if (prAdapter->rWifiVar.ucDftNdlQosQuotaVal)
		u4DftNdlQosQuotaVal = prAdapter->rWifiVar.ucDftNdlQosQuotaVal;

	nanSchedNegoAddQos(prAdapter, u4DftNdlQosQuotaVal,
			   u4DftNdlQosLatencyVal);
}

uint32_t
nanSchedNegoStart(
	struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
	enum _ENUM_NAN_NEGO_TYPE_T eType, enum _ENUM_NAN_NEGO_ROLE_T eRole,
	void (*pfCallback)(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
			   enum _ENUM_NAN_NEGO_TYPE_T eType,
			   enum _ENUM_NAN_NEGO_ROLE_T eRole, void *pvToken),
	void *pvToken) {
	uint32_t u4Idx;
	uint32_t u4SchIdx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	DBGLOG(NAN, INFO, "IN\n");

	u4SchIdx = nanSchedLookupPeerSchRecordIdx(prAdapter, pucNmiAddr);
	if (u4SchIdx == NAN_INVALID_SCH_RECORD)
		u4SchIdx =
			nanSchedAcquirePeerSchedRecord(prAdapter, pucNmiAddr);
	if (u4SchIdx == NAN_INVALID_SCH_RECORD)
		return WLAN_STATUS_FAILURE;

	if (prNegoCtrl->ucNegoTransNum >= NAN_CRB_NEGO_MAX_TRANSACTION)
		return WLAN_STATUS_FAILURE;

	u4Idx = (prNegoCtrl->ucNegoTransHeadPos + prNegoCtrl->ucNegoTransNum) %
		NAN_CRB_NEGO_MAX_TRANSACTION;
	prNegoCtrl->ucNegoTransNum++;

	kalMemCopy(prNegoCtrl->rNegoTrans[u4Idx].aucNmiAddr, pucNmiAddr,
		   MAC_ADDR_LEN);
	prNegoCtrl->rNegoTrans[u4Idx].eRole = eRole;
	prNegoCtrl->rNegoTrans[u4Idx].eType = eType;
	prNegoCtrl->rNegoTrans[u4Idx].pfCallback = pfCallback;
	prNegoCtrl->rNegoTrans[u4Idx].pvToken = pvToken;

	nanSchedPeerPrepareNegoState(prAdapter, pucNmiAddr);

	if (prNegoCtrl->eState == ENUM_NAN_CRB_NEGO_STATE_IDLE) {
		cnmTimerStopTimer(prAdapter,
				  &(prNegoCtrl->rCrbNegoDispatchTimer));
		cnmTimerStartTimer(prAdapter,
				   &(prNegoCtrl->rCrbNegoDispatchTimer), 1);
	}

	return WLAN_STATUS_PENDING;
}

void
nanSchedNegoStop(struct ADAPTER *prAdapter) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	DBGLOG(NAN, INFO, "IN\n");

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	prPeerSchRecord =
		nanSchedGetPeerSchRecord(prAdapter, prNegoCtrl->u4SchIdx);
	if (prPeerSchRecord->fgActive && !prPeerSchRecord->fgUseDataPath &&
	    !prPeerSchRecord->fgUseRanging) {
		nanSchedReleasePeerSchedRecord(prAdapter, prNegoCtrl->u4SchIdx);
	}

	prNegoCtrl->eState = ENUM_NAN_CRB_NEGO_STATE_IDLE;
	prNanTimelineMgmt->fgChkCondAvailability = FALSE;

	nanSchedNegoDumpState(prAdapter, (uint8_t *) __func__, __LINE__);
	nanSchedPeerCompleteNegoState(prAdapter, prPeerSchRecord->aucNmiAddr);

	cnmTimerStopTimer(prAdapter, &(prNegoCtrl->rCrbNegoDispatchTimer));
	if (prNegoCtrl->ucNegoTransNum > 0)
		cnmTimerStartTimer(prAdapter,
				   &(prNegoCtrl->rCrbNegoDispatchTimer), 1);
	else
		nanSchedNegoSyncSchUpdateFsmStep(
			prAdapter, prNegoCtrl->eSyncSchUpdateCurrentState);
}

unsigned char
nanSchedNegoInProgress(struct ADAPTER *prAdapter) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	if (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_IDLE)
		return TRUE;

	if (prNegoCtrl->i4InNegoContext) {
		DBGLOG(NAN, INFO, "In NEGO state, i4InNegoContext:%d\n",
		       prNegoCtrl->i4InNegoContext);
		return TRUE;
	}

	return FALSE;
}

uint32_t
nanSchedNegoApplyCustChnlList(struct ADAPTER *prAdapter) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Idx1;
	uint32_t u4SlotIdx;
	union _NAN_BAND_CHNL_CTRL rCustChnl, rCommitChnl;
	struct _NAN_CHANNEL_TIMELINE_T *prCustChnlTimeline;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	enum _ENUM_CNM_CH_CONCURR_T eConcurr;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	prCustChnlTimeline = &prNanTimelineMgmt->arCustChnlList[0];
	for (u4Idx1 = 0; u4Idx1 < NAN_TIMELINE_MGMT_CHNL_LIST_NUM;
	     u4Idx1++, prCustChnlTimeline++) {
		if (prCustChnlTimeline->fgValid == FALSE)
			continue;
		rCustChnl = prCustChnlTimeline->rChnlInfo;

		for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
		     u4SlotIdx++) {
			if (!NAN_IS_AVAIL_MAP_SET(
				    prCustChnlTimeline->au4AvailMap, u4SlotIdx))
				continue;

			rCommitChnl = nanQueryChnlInfoBySlot(
				prAdapter, u4SlotIdx, NULL, TRUE);
			if (rCommitChnl.rChannel.u4PrimaryChnl == 0) {
				rRetStatus = nanSchedAddCrbToChnlList(
					prAdapter, &rCustChnl, u4SlotIdx, 1,
					ENUM_TIME_BITMAP_CTRL_PERIOD_8192, TRUE,
					NULL);
				if (rRetStatus != WLAN_STATUS_SUCCESS)
					DBGLOG(NAN, INFO,
					       "nanSchedAddCrbToChnlList fail@%d\n",
					       __LINE__);
				continue;
			}

			eConcurr = nanSchedChkConcurrOp(rCustChnl, rCommitChnl);
			if (eConcurr == CNM_CH_CONCURR_MCC) {
				nanSchedDeleteCrbFromChnlList(
					prAdapter, u4SlotIdx, 1,
					ENUM_TIME_BITMAP_CTRL_PERIOD_8192,
					TRUE);
			}

			rRetStatus = nanSchedAddCrbToChnlList(
				prAdapter, &rCustChnl, u4SlotIdx, 1,
				ENUM_TIME_BITMAP_CTRL_PERIOD_8192, TRUE, NULL);
			if (rRetStatus != WLAN_STATUS_SUCCESS)
				DBGLOG(NAN, INFO,
				       "nanSchedAddCrbToChnlList fail@%d\n",
				       __LINE__);
		}
	}

	return rRetStatus;
}

uint32_t
nanSchedNegoValidateCondChnlList(struct ADAPTER *prAdapter) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4SlotIdx;
	uint32_t u4LocalChnl;
	uint32_t u4RmtChnl;
	uint32_t u4AvailDbIdx;
	uint32_t u4SchIdx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	u4SchIdx = prNegoCtrl->u4SchIdx;

	/* check if local & remote sides both in the same channel during
	 * the conditional window
	 */
	for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS; u4SlotIdx++) {
		u4LocalChnl =
			nanQueryPrimaryChnlBySlot(prAdapter, u4SlotIdx, FALSE);
		if (u4LocalChnl == 0)
			continue;

		/* Fixme */
		u4RmtChnl = 0;
		for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
		     u4AvailDbIdx++) {
			if (nanSchedPeerAvailabilityDbValidByID(
				    prAdapter, u4SchIdx, u4AvailDbIdx) == FALSE)
				continue;

			u4RmtChnl = nanQueryPeerPrimaryChnlBySlot(
				prAdapter, u4SchIdx, u4AvailDbIdx, u4SlotIdx,
				TRUE);
			if (u4RmtChnl == 0)
				u4RmtChnl = nanQueryPeerPrimaryChnlBySlot(
					prAdapter, u4SchIdx, u4AvailDbIdx,
					u4SlotIdx, FALSE);

			if (u4RmtChnl == u4LocalChnl)
				break;
		}

		if (u4RmtChnl != u4LocalChnl)
			nanSchedDeleteCrbFromChnlList(
				prAdapter, u4SlotIdx, 1,
				ENUM_TIME_BITMAP_CTRL_PERIOD_8192, FALSE);
	}

	return rRetStatus;
}

uint32_t
nanSchedNegoCommitCondChnlList(struct ADAPTER *prAdapter) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Idx1;
	uint32_t u4SlotIdx;
	union _NAN_BAND_CHNL_CTRL rCondChnl;
	struct _NAN_CHANNEL_TIMELINE_T *prCondChnlTimeline;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	/* check local & remote availability to remove conflict
	 * conditional windows
	 */
	nanSchedNegoValidateCondChnlList(prAdapter);

	/* merge remain conditional windows into committed windows */
	prCondChnlTimeline = &prNanTimelineMgmt->arCondChnlList[0];
	for (u4Idx1 = 0; u4Idx1 < NAN_TIMELINE_MGMT_CHNL_LIST_NUM;
	     u4Idx1++, prCondChnlTimeline++) {
		if (prCondChnlTimeline->fgValid == FALSE)
			continue;

		rCondChnl = prCondChnlTimeline->rChnlInfo;
		for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
		     u4SlotIdx++) {
			if (!NAN_IS_AVAIL_MAP_SET(
				    prCondChnlTimeline->au4AvailMap, u4SlotIdx))
				continue;

			/* remove CRB from conditional window list */
			nanSchedDeleteCrbFromChnlList(
				prAdapter, u4SlotIdx, 1,
				ENUM_TIME_BITMAP_CTRL_PERIOD_8192, FALSE);

			/* add CRB to committed window list */
			rRetStatus = nanSchedAddCrbToChnlList(
				prAdapter, &rCondChnl, u4SlotIdx, 1,
				ENUM_TIME_BITMAP_CTRL_PERIOD_8192, TRUE, NULL);
			if (rRetStatus != WLAN_STATUS_SUCCESS)
				DBGLOG(NAN, INFO,
				       "nanSchedAddCrbToChnlList fail@%d\n",
				       __LINE__);
		}
	}

	return rRetStatus;
}

enum _ENUM_CHNL_CHECK_T
nanSchedNegoChkChnlConflict(struct ADAPTER *prAdapter, uint32_t u4SlotIdx,
			    unsigned char fgChkRmtCondSlot) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t u4AvailDbIdx;
	uint32_t u4SchIdx;
	union _NAN_BAND_CHNL_CTRL rLocalChnlInfo;
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;
	unsigned char fgValidDB = FALSE;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	u4SchIdx = prNegoCtrl->u4SchIdx;

	rLocalChnlInfo = nanGetChnlInfoBySlot(prAdapter, u4SlotIdx);
	if (rLocalChnlInfo.rChannel.u4PrimaryChnl == 0)
		return ENUM_CHNL_CHECK_NOT_FOUND;

	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
	     u4AvailDbIdx++) {
		if (nanSchedPeerAvailabilityDbValidByID(prAdapter, u4SchIdx,
							u4AvailDbIdx) == FALSE)
			continue;

		fgValidDB = TRUE;

		rRmtChnlInfo = nanGetPeerChnlInfoBySlot(prAdapter, u4SchIdx,
							u4AvailDbIdx, u4SlotIdx,
							fgChkRmtCondSlot);
		if (rRmtChnlInfo.rChannel.u4PrimaryChnl == 0)
			return ENUM_CHNL_CHECK_PASS;

		if (nanSchedChkConcurrOp(rLocalChnlInfo, rRmtChnlInfo) !=
		    CNM_CH_CONCURR_MCC)
			return ENUM_CHNL_CHECK_PASS;
	}

	if (!fgValidDB && !fgChkRmtCondSlot)
		return ENUM_CHNL_CHECK_PASS;

	return ENUM_CHNL_CHECK_CONFLICT;
}

uint32_t
nanSchedNegoUpdateDatabase(struct ADAPTER *prAdapter,
			   unsigned char fgChkRmtCondSlot) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t u4Idx;
	uint32_t u4SlotIdx;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	enum _ENUM_CHNL_CHECK_T eChkChnlResult;
	uint32_t u4RmtChnl;
	uint32_t u4SchIdx;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	u4SchIdx = prNegoCtrl->u4SchIdx;

	kalMemZero(prNegoCtrl->au4AvailSlots,
		   sizeof(prNegoCtrl->au4AvailSlots));
	kalMemZero(prNegoCtrl->au4UnavailSlots,
		   sizeof(prNegoCtrl->au4UnavailSlots));

	for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS; u4SlotIdx++) {
		if (nanWindowType(prAdapter, u4SlotIdx) == ENUM_NAN_DW) {
			NAN_TIMELINE_SET(prNegoCtrl->au4UnavailSlots,
					 u4SlotIdx);
			continue;
		}

		eChkChnlResult = nanSchedNegoChkChnlConflict(
			prAdapter, u4SlotIdx, fgChkRmtCondSlot);

		if (eChkChnlResult == ENUM_CHNL_CHECK_NOT_FOUND)
			continue;
		else if (eChkChnlResult == ENUM_CHNL_CHECK_PASS)
			NAN_TIMELINE_SET(prNegoCtrl->au4AvailSlots, u4SlotIdx);
		else
			NAN_TIMELINE_SET(prNegoCtrl->au4UnavailSlots,
					 u4SlotIdx);
	}

	for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++) {
		prNegoCtrl->au4FreeSlots[u4Idx] =
			~(prNegoCtrl->au4AvailSlots[u4Idx] |
			  prNegoCtrl->au4UnavailSlots[u4Idx]);
		prNegoCtrl->au4FawSlots[u4Idx] =
			(prNegoCtrl->au4AvailSlots[u4Idx]);
	}

	DBGLOG(NAN, INFO, "fgChkRmtCondSlot:%d\n", fgChkRmtCondSlot);
	nanUtilDump(prAdapter, "au4AvailSlots",
		    (uint8_t *)prNegoCtrl->au4AvailSlots,
		    sizeof(prNegoCtrl->au4AvailSlots));
	nanUtilDump(prAdapter, "au4UnavailSlots",
		    (uint8_t *)prNegoCtrl->au4UnavailSlots,
		    sizeof(prNegoCtrl->au4UnavailSlots));
	nanUtilDump(prAdapter, "au4FawSlots",
		    (uint8_t *)prNegoCtrl->au4FawSlots,
		    sizeof(prNegoCtrl->au4FawSlots));
	nanUtilDump(prAdapter, "au4FreeSlots",
		    (uint8_t *)prNegoCtrl->au4FreeSlots,
		    sizeof(prNegoCtrl->au4FreeSlots));

	if (fgChkRmtCondSlot) {
		for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
		     u4SlotIdx++) {
			/* TODO: consider DW slot */
			u4RmtChnl = nanGetPeerPrimaryChnlBySlot(
				prAdapter, u4SchIdx, NAN_NUM_AVAIL_DB,
				u4SlotIdx, TRUE);
			if (u4RmtChnl != 0)
				continue;

			if (NAN_IS_AVAIL_MAP_SET(prNegoCtrl->au4FawSlots,
						 u4SlotIdx))
				NAN_TIMELINE_UNSET(prNegoCtrl->au4FawSlots,
						   u4SlotIdx);
			else if (NAN_IS_AVAIL_MAP_SET(prNegoCtrl->au4FreeSlots,
						      u4SlotIdx))
				NAN_TIMELINE_UNSET(prNegoCtrl->au4FreeSlots,
						   u4SlotIdx);
		}

		nanUtilDump(prAdapter, "final au4FawSlots",
			    (uint8_t *)prNegoCtrl->au4FawSlots,
			    sizeof(prNegoCtrl->au4FawSlots));
		nanUtilDump(prAdapter, "final au4FreeSlots",
			    (uint8_t *)prNegoCtrl->au4FreeSlots,
			    sizeof(prNegoCtrl->au4FreeSlots));
	}

	return rRetStatus;
}

uint32_t
nanSchedNegoGenQosCriteria(struct ADAPTER *prAdapter) {
	uint32_t u4Idx, u4Idx1;
	uint32_t u4SlotIdx;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	int32_t i4Num;
	uint32_t u4EmptySlots;
	uint32_t u4CondSlots;
	int32_t i4Latency, i4LatencyStart, i4LatencyEnd;
	uint32_t u4QosMinSlots;
	uint32_t u4QosMaxLatency;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	union _NAN_BAND_CHNL_CTRL rSelChnlInfo;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, prNegoCtrl->u4SchIdx);

	DBGLOG(NAN, INFO, "------>\n");
	DBGLOG(NAN, INFO, "Peer Qos spec MinSlots:%d, MaxLatency:%d\n",
	       prPeerSchDesc->u4QosMinSlots, prPeerSchDesc->u4QosMaxLatency);
	DBGLOG(NAN, INFO, "My Qos spec MinSlots:%d, MaxLatency:%d\n",
	       prNegoCtrl->u4QosMinSlots, prNegoCtrl->u4QosMaxLatency);

	/* negotiate min slots */
	if (prNegoCtrl->u4QosMinSlots > prPeerSchDesc->u4QosMinSlots)
		u4QosMinSlots = prNegoCtrl->u4QosMinSlots;
	else
		u4QosMinSlots = prPeerSchDesc->u4QosMinSlots;
	if (u4QosMinSlots > NAN_INVALID_QOS_MIN_SLOTS) {
		if (u4QosMinSlots < NAN_QOS_MIN_SLOTS_LOW_BOUND)
			u4QosMinSlots = NAN_QOS_MIN_SLOTS_LOW_BOUND;
		else if (u4QosMinSlots > NAN_QOS_MIN_SLOTS_UP_BOUND)
			u4QosMinSlots = NAN_QOS_MIN_SLOTS_UP_BOUND;

		prNegoCtrl->u4NegoQosMinSlots = u4QosMinSlots;
	}

	/* negotiate max latency */
	if (prNegoCtrl->u4QosMaxLatency > prPeerSchDesc->u4QosMaxLatency)
		u4QosMaxLatency = prPeerSchDesc->u4QosMaxLatency;
	else
		u4QosMaxLatency = prNegoCtrl->u4QosMaxLatency;
	if (u4QosMaxLatency < NAN_INVALID_QOS_MAX_LATENCY) {
		if (u4QosMaxLatency < NAN_QOS_MAX_LATENCY_LOW_BOUND)
			u4QosMaxLatency =
				NAN_QOS_MAX_LATENCY_LOW_BOUND;
			/* reserve 1 slot for DW window */
		else if (u4QosMaxLatency > NAN_QOS_MAX_LATENCY_UP_BOUND)
			u4QosMaxLatency = NAN_QOS_MAX_LATENCY_UP_BOUND;

		prNegoCtrl->u4NegoQosMaxLatency = u4QosMaxLatency;
	}

	for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++) {
		u4CondSlots = 0;

		/* step1. check QoS min slots */
		i4Num = 0;
		if ((u4QosMinSlots > NAN_INVALID_QOS_MIN_SLOTS) &&
		    (nanUtilCheckBitOneCnt(
			     prAdapter,
			     (uint8_t *)&prNegoCtrl->au4FawSlots[u4Idx],
			     sizeof(uint32_t)) < u4QosMinSlots)) {

			i4Num = u4QosMinSlots -
				nanUtilCheckBitOneCnt(
					prAdapter, (uint8_t *)&prNegoCtrl
							   ->au4FawSlots[u4Idx],
					sizeof(uint32_t));
			if ((i4Num > 0) &&
			    (nanUtilCheckBitOneCnt(
				 prAdapter,
				 (uint8_t *)&prNegoCtrl->au4FreeSlots[u4Idx],
				 sizeof(uint32_t)) < i4Num)) {

				DBGLOG(NAN, INFO, "MinSlots:%d, Lack:%d\n",
				       u4QosMinSlots, i4Num);
				rRetStatus = WLAN_STATUS_FAILURE;
				goto CHK_QOS_DONE;
			}
		}

		/* step2. check & quarantee QoS max latency */
		if (u4QosMaxLatency >= NAN_INVALID_QOS_MAX_LATENCY)
			goto CHK_QOS_LATENCY_DONE;

		u4EmptySlots = ~(prNegoCtrl->au4FawSlots[u4Idx]);
		i4Latency = 0;
		i4LatencyStart = i4LatencyEnd = -1;

		for (u4Idx1 = 0; u4Idx1 < 32; u4Idx1++) {
			if (u4EmptySlots & BIT(u4Idx1)) {
				if (i4Latency == 0) {
					i4Latency++;
					i4LatencyStart = i4LatencyEnd = u4Idx1;
				} else {
					i4Latency++;
					i4LatencyEnd = u4Idx1;
				}

				for (; (i4Latency > u4QosMaxLatency) &&
				       (i4LatencyStart <= i4LatencyEnd);
				     i4LatencyEnd--) {
					if (prNegoCtrl->au4UnavailSlots[u4Idx] &
					    BIT(i4LatencyEnd))
						continue;

					if (prNegoCtrl->au4FreeSlots[u4Idx] &
					    BIT(i4LatencyEnd)) {
						prNegoCtrl
							->au4FawSlots[u4Idx] |=
							BIT(i4LatencyEnd);
						u4CondSlots |=
							BIT(i4LatencyEnd);

						i4Num--;
						u4Idx1 = i4LatencyEnd;
						i4Latency = 0;
						i4LatencyStart = i4LatencyEnd =
							-1;
					}
				}

				if (i4Latency > u4QosMaxLatency) {
					rRetStatus = WLAN_STATUS_FAILURE;
					goto CHK_QOS_DONE;
				}
			} else {
				i4Latency = 0;
				i4LatencyStart = i4LatencyEnd = -1;
			}
		}
CHK_QOS_LATENCY_DONE:

		/* step3. quarantee QoS min slots */
		for (u4Idx1 = 0; (i4Num > 0) && (u4Idx1 < 32); u4Idx1++) {
			if (!(prNegoCtrl->au4FawSlots[u4Idx] & BIT(u4Idx1)) &&
			    (prNegoCtrl->au4FreeSlots[u4Idx] & BIT(u4Idx1))) {

				i4Num--;

				prNegoCtrl->au4FawSlots[u4Idx] |= BIT(u4Idx1);
				u4CondSlots |= BIT(u4Idx1);
			}
		}

		if (i4Num > 0) {
			rRetStatus = WLAN_STATUS_FAILURE;
			goto CHK_QOS_DONE;
		}

		/* step4. assign channel to conditional slots */
		for (u4Idx1 = 0; u4Idx1 < 32; u4Idx1++) {
			if (u4CondSlots & BIT(u4Idx1)) {
				u4SlotIdx = u4Idx * NAN_SLOTS_PER_DW_INTERVAL +
					    u4Idx1;
				rSelChnlInfo = nanSchedNegoSelectChnlInfo(
					prAdapter, u4SlotIdx);
				rRetStatus = nanSchedAddCrbToChnlList(
					prAdapter, &rSelChnlInfo, u4SlotIdx, 1,
					ENUM_TIME_BITMAP_CTRL_PERIOD_8192,
					FALSE, NULL);
				if (rRetStatus != WLAN_STATUS_SUCCESS)
					break;
			}
		}

		/* step5. merge availability entry for the
		 * same primary channel
		 */
		rRetStatus = nanSchedMergeAvailabileChnlList(prAdapter, FALSE);
	}

CHK_QOS_DONE:
	if (rRetStatus == WLAN_STATUS_SUCCESS)
		nanUtilDump(prAdapter, "QoS au4FawSlots",
			    (uint8_t *)prNegoCtrl->au4FawSlots,
			    sizeof(prNegoCtrl->au4FawSlots));
	else
		DBGLOG(NAN, ERROR, "can't satisfy Qos spec\n");
	DBGLOG(NAN, INFO, "<------\n");

	return rRetStatus;
}

uint32_t
nanSchedNegoChkQosSpec(struct ADAPTER *prAdapter,
		unsigned char fgChkRmtCondSlot) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, prNegoCtrl->u4SchIdx);

	if ((prNegoCtrl->u4QosMinSlots == NAN_INVALID_QOS_MIN_SLOTS) &&
	    (prNegoCtrl->u4QosMaxLatency == NAN_INVALID_QOS_MAX_LATENCY) &&
	    (prPeerSchDesc->u4QosMinSlots == NAN_INVALID_QOS_MIN_SLOTS) &&
	    (prPeerSchDesc->u4QosMaxLatency == NAN_INVALID_QOS_MAX_LATENCY))
		return WLAN_STATUS_SUCCESS;

	/* Update current slot status */
	nanSchedNegoUpdateDatabase(prAdapter, fgChkRmtCondSlot);

	/* Verify QoS requirement */
	return nanSchedNegoGenQosCriteria(prAdapter);
}

uint32_t
nanSchedNegoIsRmtAvailabilityConflict(struct ADAPTER *prAdapter) {
	uint32_t u4RetCode = 0;
	uint32_t u4Idx;
	uint32_t u4SlotIdx;
	uint32_t u4AvailDbIdx;
	uint32_t u4AvailEntryIdx;
	uint16_t u2Type;
	uint32_t u4SchIdx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_SCHEDULE_TIMELINE_T *prTimeline;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;
	struct _NAN_AVAILABILITY_DB_T *prAvailDb;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntry;
	struct _NAN_AVAILABILITY_TIMELINE_T *prNanAvailEntryTmp;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	u4SchIdx = prNegoCtrl->u4SchIdx;
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, u4SchIdx);

	for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
	     u4AvailDbIdx++) {
		prAvailDb = &prPeerSchDesc->arAvailAttr[u4AvailDbIdx];
		if (prAvailDb->ucMapId == NAN_INVALID_MAP_ID)
			continue;

		for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
		     u4SlotIdx++) {
			prNanAvailEntryTmp = NULL;

			for (u4AvailEntryIdx = 0;
			     u4AvailEntryIdx < NAN_NUM_AVAIL_TIMELINE;
			     u4AvailEntryIdx++) {
				prNanAvailEntry = &prAvailDb->arAvailEntryList
							   [u4AvailEntryIdx];
				if (prNanAvailEntry->fgActive == FALSE)
					continue;

				if (!NAN_IS_AVAIL_MAP_SET(
					    prNanAvailEntry->au4AvailMap,
					    u4SlotIdx))
					continue;

				u2Type = prNanAvailEntry->rEntryCtrl.rField
						 .u2Type;

				if (!(u2Type &
				      NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COMMIT) &&
				    !(u2Type &
				      NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COND))
					continue;

				if (prNanAvailEntryTmp == NULL) {
					prNanAvailEntryTmp = prNanAvailEntry;
					continue;
				}

				if (prNanAvailEntryTmp->arBandChnlCtrl[0]
					    .rChannel.u4PrimaryChnl !=
				    prNanAvailEntry->arBandChnlCtrl[0]
					    .rChannel.u4PrimaryChnl) {

					u4RetCode = -5;
					goto CHK_RMT_AVAIL_DONE;
				}
			}
		}
	}

	if (prPeerSchDesc->rSelectedNdcCtrl.fgValid == TRUE) {
		for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
			prTimeline = &prPeerSchDesc->rSelectedNdcCtrl
					      .arTimeline[u4Idx];
			if (prTimeline->ucMapId == NAN_INVALID_MAP_ID)
				continue;

			u4AvailDbIdx = nanSchedPeerGetAvailabilityDbIdx(
				prAdapter, prPeerSchDesc, prTimeline->ucMapId);
			if (u4AvailDbIdx == NAN_INVALID_AVAIL_DB_INDEX) {
				u4RetCode = -10;
				goto CHK_RMT_AVAIL_DONE;
			}

			for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
			     u4SlotIdx++) {
				if (!NAN_IS_AVAIL_MAP_SET(
					    prTimeline->au4AvailMap, u4SlotIdx))
					continue;

				if (nanQueryPeerPrimaryChnlBySlot(
					    prAdapter, u4SchIdx, u4AvailDbIdx,
					    u4SlotIdx, TRUE) != 0)
					continue;

				if (nanQueryPeerPrimaryChnlBySlot(
					    prAdapter, u4SchIdx, u4AvailDbIdx,
					    u4SlotIdx, FALSE) != 0)
					continue;

				u4RetCode = -15;
				goto CHK_RMT_AVAIL_DONE;
			}
		}
	}

	prTimeline = NULL;

	if (prNegoCtrl->eType == ENUM_NAN_NEGO_DATA_LINK) {
		if (prPeerSchDesc->fgImmuNdlTimelineValid == TRUE)
			prTimeline = &prPeerSchDesc->arImmuNdlTimeline[0];
	} else {
		if (prPeerSchDesc->fgRangingTimelineValid == TRUE)
			prTimeline = &prPeerSchDesc->arRangingTimeline[0];
	}

	if (prTimeline) {
		for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
			if (prTimeline->ucMapId == NAN_INVALID_MAP_ID)
				continue;

			u4AvailDbIdx = nanSchedPeerGetAvailabilityDbIdx(
				prAdapter, prPeerSchDesc, prTimeline->ucMapId);
			if (u4AvailDbIdx == NAN_INVALID_AVAIL_DB_INDEX) {
				u4RetCode = -20;
				goto CHK_RMT_AVAIL_DONE;
			}

			for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
			     u4SlotIdx++) {
				if (!NAN_IS_AVAIL_MAP_SET(
					    prTimeline->au4AvailMap, u4SlotIdx))
					continue;

				if (nanQueryPeerPrimaryChnlBySlot(
					    prAdapter, u4SchIdx, u4AvailDbIdx,
					    u4SlotIdx, TRUE) != 0)
					continue;

				if (nanQueryPeerPrimaryChnlBySlot(
					    prAdapter, u4SchIdx, u4AvailDbIdx,
					    u4SlotIdx, FALSE) != 0)
					continue;

				u4RetCode = -25;
				goto CHK_RMT_AVAIL_DONE;
			}

			prTimeline++;
		}
	}

CHK_RMT_AVAIL_DONE:
	DBGLOG(NAN, INFO, "Return:%d\n", u4RetCode);

	return u4RetCode;
}

/* check if remote CRB conflicts with local CRB */
unsigned char
nanSchedNegoIsRmtCrbConflict(
	struct ADAPTER *prAdapter,
	struct _NAN_SCHEDULE_TIMELINE_T arTimeline[NAN_NUM_AVAIL_DB],
	unsigned char *pfgEmptyMapSet, /* found local empty slot */
	uint32_t au4EmptyMap[NAN_TOTAL_DW]) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t u4AvailDbIdx;
	uint32_t u4SlotIdx;
	uint32_t u4Idx;
	struct _NAN_SCHEDULE_TIMELINE_T *prTimeline;
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;
	union _NAN_BAND_CHNL_CTRL rLocalChnlInfo;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prPeerSchRecord =
		nanSchedGetPeerSchRecord(prAdapter, prNegoCtrl->u4SchIdx);

	if (pfgEmptyMapSet && au4EmptyMap) {
		*pfgEmptyMapSet = FALSE;
		kalMemZero(au4EmptyMap, sizeof(uint32_t) * NAN_TOTAL_DW);
	}

	for (u4Idx = 0; u4Idx < NAN_NUM_AVAIL_DB; u4Idx++) {
		prTimeline = &arTimeline[u4Idx];
		if (prTimeline->ucMapId == NAN_INVALID_MAP_ID)
			continue;

		u4AvailDbIdx = nanSchedPeerGetAvailabilityDbIdx(
			prAdapter, prPeerSchRecord->prPeerSchDesc,
			prTimeline->ucMapId);
		if (u4AvailDbIdx == NAN_INVALID_AVAIL_DB_INDEX)
			return TRUE;

		for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
		     u4SlotIdx++) {
			if (!NAN_IS_AVAIL_MAP_SET(prTimeline->au4AvailMap,
						  u4SlotIdx))
				continue;

			rRmtChnlInfo = nanGetPeerChnlInfoBySlot(
				prAdapter, prNegoCtrl->u4SchIdx, u4AvailDbIdx,
				u4SlotIdx, TRUE);
			if (rRmtChnlInfo.rChannel.u4PrimaryChnl == 0) {
				DBGLOG(NAN, ERROR, "rmt channel invalid\n");
				return TRUE;
			}
			if (!nanIsAllowedChannel(
				    prAdapter,
				    rRmtChnlInfo.rChannel.u4PrimaryChnl)) {
				DBGLOG(NAN, WARN,
				       "rmt channel (%d) not allowed\n",
				       rRmtChnlInfo.rChannel.u4PrimaryChnl);
				return TRUE;
			}

			rLocalChnlInfo =
				nanGetChnlInfoBySlot(prAdapter, u4SlotIdx);
			if (rLocalChnlInfo.rChannel.u4PrimaryChnl == 0) {
				if (pfgEmptyMapSet && au4EmptyMap) {
					*pfgEmptyMapSet = TRUE;
					NAN_TIMELINE_SET(au4EmptyMap,
							 u4SlotIdx);
				}
				continue;
			}
/* Need to discuss why checking concurrent op. */
#if 0
			if (nanSchedChkConcurrOp(rLocalChnlInfo,
						 rRmtChnlInfo) ==
			    CNM_CH_CONCURR_MCC)
				return TRUE;
#endif
		}
	}

	return FALSE;
}

/* check if local CRB conflicts with remote CRB */
unsigned char
nanSchedNegoIsLocalCrbConflict(
	struct ADAPTER *prAdapter, struct _NAN_SCHEDULE_TIMELINE_T *prTimeline,
	unsigned char fgChkRmtCondSlot,
	unsigned char *pfgEmptyMapSet, /* found remote empty slot */
	uint32_t au4EmptyMap[NAN_TOTAL_DW]) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t u4SlotIdx;
	uint32_t u4AvailDbIdx;
	unsigned char fgConflict;
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;
	union _NAN_BAND_CHNL_CTRL rLocalChnlInfo;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	if (prTimeline->ucMapId == NAN_INVALID_MAP_ID) {
		DBGLOG(NAN, ERROR, "ucMapId equals NAN_INVALID_MAP_ID!\n");
		return TRUE;
	}

	if (!nanSchedPeerAvailabilityDbValid(prAdapter, prNegoCtrl->u4SchIdx))
		return TRUE;

	if (pfgEmptyMapSet && au4EmptyMap) {
		*pfgEmptyMapSet = FALSE;
		kalMemZero(au4EmptyMap, sizeof(uint32_t) * NAN_TOTAL_DW);
	}

	for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS; u4SlotIdx++) {
		if (!NAN_IS_AVAIL_MAP_SET(prTimeline->au4AvailMap, u4SlotIdx))
			continue;

		rLocalChnlInfo = nanGetChnlInfoBySlot(prAdapter, u4SlotIdx);
		if (rLocalChnlInfo.rChannel.u4PrimaryChnl == 0) {
			DBGLOG(NAN, ERROR, "u4PrimaryChnl equals 0!\n");
			return TRUE;
		}

		fgConflict = TRUE;

		for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
		     u4AvailDbIdx++) {
			if (nanSchedPeerAvailabilityDbValidByID(
				    prAdapter, prNegoCtrl->u4SchIdx,
				    u4AvailDbIdx) == FALSE)
				continue;

			rRmtChnlInfo = nanGetPeerChnlInfoBySlot(
				prAdapter, prNegoCtrl->u4SchIdx, u4AvailDbIdx,
				u4SlotIdx, fgChkRmtCondSlot);
			if (rRmtChnlInfo.rChannel.u4PrimaryChnl == 0)
				fgConflict = FALSE;
			else if (!nanIsAllowedChannel(
					 prAdapter,
					 rRmtChnlInfo.rChannel.u4PrimaryChnl))
				continue;

			if (nanSchedChkConcurrOp(rLocalChnlInfo,
						 rRmtChnlInfo) !=
			    CNM_CH_CONCURR_MCC) {
				fgConflict = FALSE;
				break;
			}
		}

		if (fgConflict == TRUE)
			return TRUE;

		if (u4AvailDbIdx >= NAN_NUM_AVAIL_DB) {
			/* cannot find a coherent rmt channel */
			if (pfgEmptyMapSet && au4EmptyMap) {
				*pfgEmptyMapSet = TRUE;
				NAN_TIMELINE_SET(au4EmptyMap, u4SlotIdx);
			}
		}
	}

	return FALSE;
}

unsigned char
nanSchedNegoIsCrbEqual(
	struct ADAPTER *prAdapter,
	struct _NAN_SCHEDULE_TIMELINE_T *prLocalTimeline,
	struct _NAN_SCHEDULE_TIMELINE_T arRmtTimeline[NAN_NUM_AVAIL_DB]) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t u4Idx, u4Idx1;
	struct _NAN_SCHEDULE_TIMELINE_T *prRmtTimeline;
	uint32_t u4Bitmask;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++) {
		u4Bitmask = 0;

		for (u4Idx1 = 0; u4Idx1 < NAN_NUM_AVAIL_DB; u4Idx1++) {
			prRmtTimeline = &arRmtTimeline[u4Idx1];
			if (prRmtTimeline->ucMapId == NAN_INVALID_MAP_ID)
				continue;

			u4Bitmask |= prRmtTimeline->au4AvailMap[u4Idx];
		}

		if (prLocalTimeline->au4AvailMap[u4Idx] != u4Bitmask)
			return FALSE;
	}

	return !nanSchedNegoIsRmtCrbConflict(prAdapter, arRmtTimeline, NULL,
					     NULL);
}

/* [nanSchedNegoMergeCrb]
 *
 *   It just merges the CRBs and doesn't check if CRB channel conflict.
 */
uint32_t
nanSchedNegoMergeCrb(
	struct ADAPTER *prAdapter,
	struct _NAN_SCHEDULE_TIMELINE_T *prLocalTimeline,
	struct _NAN_SCHEDULE_TIMELINE_T arRmtTimeline[NAN_NUM_AVAIL_DB]) {
	uint32_t u4Idx, u4Idx1;

	for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++) {
		for (u4Idx1 = 0; u4Idx1 < NAN_NUM_AVAIL_DB; u4Idx1++) {
			if (arRmtTimeline[u4Idx1].ucMapId != NAN_INVALID_MAP_ID)
				prLocalTimeline->au4AvailMap[u4Idx] |=
					arRmtTimeline[u4Idx1]
						.au4AvailMap[u4Idx];
		}
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedNegoAddQos(struct ADAPTER *prAdapter, uint32_t u4MinSlots,
		   uint32_t u4MaxLatency) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	if ((prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_INITIATOR) &&
	    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_RESPONDER)) {
		return WLAN_STATUS_FAILURE;
	}

	if (u4MaxLatency < NAN_INVALID_QOS_MAX_LATENCY) {
		if (u4MaxLatency < prNegoCtrl->u4QosMaxLatency)
			prNegoCtrl->u4QosMaxLatency = u4MaxLatency;

		if (prNegoCtrl->u4QosMaxLatency < NAN_QOS_MAX_LATENCY_LOW_BOUND)
			prNegoCtrl->u4QosMaxLatency =
				NAN_QOS_MAX_LATENCY_LOW_BOUND;
		else if (prNegoCtrl->u4QosMaxLatency >
			 NAN_QOS_MAX_LATENCY_UP_BOUND)
			prNegoCtrl->u4QosMaxLatency =
				NAN_QOS_MAX_LATENCY_UP_BOUND;
	}

	if (u4MinSlots > NAN_INVALID_QOS_MIN_SLOTS) {
		if (u4MinSlots > prNegoCtrl->u4QosMinSlots)
			prNegoCtrl->u4QosMinSlots = u4MinSlots;

		if (prNegoCtrl->u4QosMinSlots < NAN_QOS_MIN_SLOTS_LOW_BOUND)
			prNegoCtrl->u4QosMinSlots = NAN_QOS_MIN_SLOTS_LOW_BOUND;
		else if (prNegoCtrl->u4QosMinSlots > NAN_QOS_MIN_SLOTS_UP_BOUND)
			prNegoCtrl->u4QosMinSlots = NAN_QOS_MIN_SLOTS_UP_BOUND;
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedNegoAddNdcCrb(struct ADAPTER *prAdapter,
		      union _NAN_BAND_CHNL_CTRL *prChnlInfo,
		      uint32_t u4StartOffset, uint32_t u4NumSlots,
		      enum _ENUM_TIME_BITMAP_CTRL_PERIOD_T eRepeatPeriod) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Idx;
	uint32_t u4Run;
	uint32_t u4SlotIdx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_NDC_CTRL_T *prNdcCtrl;
	uint32_t u4SysTime;
	uint32_t au4AvailMap[NAN_TOTAL_DW];
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	do {
		if ((prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_INITIATOR) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_RESPONDER)) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		if ((prChnlInfo == NULL) ||
		    (prChnlInfo->rChannel.u4PrimaryChnl == 0)) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		if ((u4StartOffset + u4NumSlots) > (1 << (eRepeatPeriod + 2))) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}
		/* check if the CRB conflicts with existed status */
		for (u4Run = 0; (u4Run < (128 >> eRepeatPeriod)) &&
				(rRetStatus == WLAN_STATUS_SUCCESS);
		     u4Run++) {
			for (u4Idx = 0; u4Idx < u4NumSlots; u4Idx++) {
				u4SlotIdx = u4StartOffset +
					    (u4Run << (eRepeatPeriod + 2)) +
					    u4Idx;

				if ((nanQueryPrimaryChnlBySlot(prAdapter,
							       u4SlotIdx,
							       TRUE) != 0) &&
				    (nanQueryPrimaryChnlBySlot(
					     prAdapter, u4SlotIdx, TRUE) !=
				     prChnlInfo->rChannel.u4PrimaryChnl)) {
					rRetStatus = WLAN_STATUS_FAILURE;
					break;
				}

				if ((nanQueryPrimaryChnlBySlot(prAdapter,
							       u4SlotIdx,
							       FALSE) != 0) &&
				    (nanQueryPrimaryChnlBySlot(
					     prAdapter, u4SlotIdx, FALSE) !=
				     prChnlInfo->rChannel.u4PrimaryChnl)) {
					rRetStatus = WLAN_STATUS_FAILURE;
					break;
				}
			}
		}

		if (rRetStatus != WLAN_STATUS_SUCCESS)
			break;

		kalMemZero(au4AvailMap, sizeof(au4AvailMap));
		rRetStatus = nanSchedAddCrbToChnlList(
			prAdapter, prChnlInfo, u4StartOffset, u4NumSlots,
			eRepeatPeriod, FALSE, au4AvailMap);
		if (rRetStatus != WLAN_STATUS_SUCCESS)
			break;

		prNdcCtrl = &prNegoCtrl->rSelectedNdcCtrl;
		if (prNdcCtrl->fgValid == FALSE) {
			prNdcCtrl->fgValid = TRUE;
			u4SysTime = (uint32_t)kalGetTimeTick();
			kalMemCopy(prNdcCtrl->aucNdcId, (uint8_t *)&u4SysTime,
				   NAN_NDC_ATTRIBUTE_ID_LENGTH);
			kalMemCopy(prNdcCtrl->arTimeline[0].au4AvailMap,
				   (uint8_t *)au4AvailMap, sizeof(au4AvailMap));
			prNdcCtrl->arTimeline[0].ucMapId =
				prNanTimelineMgmt->ucMapId;
		} else {
			for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++)
				prNdcCtrl->arTimeline[0].au4AvailMap[u4Idx] |=
					au4AvailMap[u4Idx];
		}
	} while (FALSE);

	return rRetStatus;
}

uint32_t
nanSchedNegoGenDefCrb(struct ADAPTER *prAdapter,
		unsigned char fgChkRmtCondSlot) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t u4SlotIdx;
	uint32_t u4SlotOffset;
	uint32_t u4DwIdx;
	uint32_t u4AvailDbIdx;
	uint32_t u4DefCrbNum;
	uint32_t u4DefCrbNumBackup;
	uint32_t u4IterationNum;
	unsigned char fgFound;
	unsigned char fgPeerAvailMapValid;
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;
	union _NAN_BAND_CHNL_CTRL rLocalChnlInfo;
	union _NAN_BAND_CHNL_CTRL rSelChnlInfo;
	struct _NAN_SCHEDULE_TIMELINE_T *prSchedTimeline = NULL;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	uint32_t u4NanQuota;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	if ((prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_INITIATOR) &&
	    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_RESPONDER) &&
	    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_WAIT_RESP)) {
		rRetStatus = WLAN_STATUS_FAILURE;
		goto GEN_DFT_CRB_DONE;
	}

	if (prNegoCtrl->eType == ENUM_NAN_NEGO_DATA_LINK) {
		u4DefCrbNum = prNegoCtrl->u4DefNdlNumSlots;
	} else if (prNegoCtrl->eType == ENUM_NAN_NEGO_RANGING) {
		u4DefCrbNum = prNegoCtrl->u4DefRangingNumSlots;
		prSchedTimeline = &prNegoCtrl->rRangingTimeline;
	} else {
		rRetStatus = WLAN_STATUS_FAILURE;
		goto GEN_DFT_CRB_DONE;
	}

	if ((prSchedTimeline != NULL) &&
	    (prSchedTimeline->ucMapId == NAN_INVALID_MAP_ID)) {
		prSchedTimeline->ucMapId = prNanTimelineMgmt->ucMapId;
		kalMemZero(prSchedTimeline->au4AvailMap,
			   sizeof(prSchedTimeline->au4AvailMap));
	}

	fgPeerAvailMapValid = nanSchedPeerAvailabilityDbValid(
		prAdapter, prNegoCtrl->u4SchIdx);
	DBGLOG(NAN, INFO,
	       "StaIdx:%d, fgChkRmtCondSlot:%d, Num:%d, PeerAvailValid:%d\n",
	       prNegoCtrl->u4SchIdx, fgChkRmtCondSlot, u4DefCrbNum,
	       fgPeerAvailMapValid);

	if (fgChkRmtCondSlot && !fgPeerAvailMapValid) {
		rRetStatus = WLAN_STATUS_FAILURE;
		goto GEN_DFT_CRB_DONE;
	}

	u4NanQuota = NAN_SLOTS_PER_DW_INTERVAL;
	if (!fgChkRmtCondSlot)
		u4NanQuota -= prAdapter->rWifiVar.ucAisQuotaVal;

	/* try to allocate basic CRBs for every DW interval */
	u4DefCrbNumBackup = u4DefCrbNum;
	for (u4DwIdx = 0; u4DwIdx < NAN_TOTAL_DW; u4DwIdx++) {
		if (fgChkRmtCondSlot)
			u4SlotOffset = 1;
		else {
			u4SlotOffset =
				prAdapter->rWifiVar.ucDftQuotaStartOffset;
		}

		for ((u4DefCrbNum = u4DefCrbNumBackup), (u4IterationNum = 0);
		     (u4DefCrbNum && (u4IterationNum < u4NanQuota));
		     (u4SlotOffset = (u4SlotOffset + 1) % u4NanQuota),
		     (u4IterationNum++)) {

			u4SlotIdx = u4SlotOffset +
				    u4DwIdx * NAN_SLOTS_PER_DW_INTERVAL;

			if (nanWindowType(prAdapter, u4SlotIdx) == ENUM_NAN_DW)
				continue;

			rLocalChnlInfo =
				nanGetChnlInfoBySlot(prAdapter, u4SlotIdx);

			if (!fgPeerAvailMapValid) {
				if (rLocalChnlInfo.rChannel.u4PrimaryChnl ==
				    0) {
					rSelChnlInfo =
					nanSchedNegoSelectChnlInfo(
						prAdapter, u4SlotIdx);
					nanSchedAddCrbToChnlList(
						prAdapter, &rSelChnlInfo,
						u4SlotIdx, 1,
					ENUM_TIME_BITMAP_CTRL_PERIOD_8192,
						FALSE, NULL);
				}

				u4DefCrbNum--;

				if (prSchedTimeline)
					NAN_TIMELINE_SET(
						prSchedTimeline->au4AvailMap,
						u4SlotIdx);

				continue;
			}

			fgFound = FALSE;

			for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
			     u4AvailDbIdx++) {
				if (nanSchedPeerAvailabilityDbValidByID(
					    prAdapter, prNegoCtrl->u4SchIdx,
					    u4AvailDbIdx) == FALSE)
					continue;

				rRmtChnlInfo = nanGetPeerChnlInfoBySlot(
					prAdapter, prNegoCtrl->u4SchIdx,
					u4AvailDbIdx, u4SlotIdx,
					fgChkRmtCondSlot);
				DBGLOG(NAN, LOUD,
				       "Slot:%d, localChnl:%d, rmtChnl:%d\n",
				       u4SlotIdx,
				       rLocalChnlInfo.rChannel.u4PrimaryChnl,
				       rRmtChnlInfo.rChannel.u4PrimaryChnl);
				if ((rRmtChnlInfo.rChannel.u4PrimaryChnl ==
				     0) &&
				    !fgChkRmtCondSlot) {
					if (rLocalChnlInfo.rChannel
					    .u4PrimaryChnl == 0) {
						rSelChnlInfo =
						nanSchedNegoSelectChnlInfo(
							prAdapter,
							u4SlotIdx);
						nanSchedAddCrbToChnlList(
							prAdapter,
							&rSelChnlInfo,
							u4SlotIdx, 1,
					    ENUM_TIME_BITMAP_CTRL_PERIOD_8192,
							FALSE, NULL);
					}

					fgFound = TRUE;
					u4DefCrbNum--;
					break;
				} else if (rRmtChnlInfo.rChannel
					   .u4PrimaryChnl != 0) {
					if (!nanIsAllowedChannel(
					    prAdapter,
					    rRmtChnlInfo.rChannel
					    .u4PrimaryChnl))
						continue;

					if ((rLocalChnlInfo.rChannel
					     .u4PrimaryChnl != 0) &&
					    (nanSchedChkConcurrOp(
						rLocalChnlInfo,
						rRmtChnlInfo) !=
						CNM_CH_CONCURR_MCC)) {
						fgFound = TRUE;
						u4DefCrbNum--;
						break;
					} else if (rLocalChnlInfo.rChannel
						   .u4PrimaryChnl != 0) {
						continue;
					}

					/* no local channel allocation case */
					rSelChnlInfo = nanSchedConvergeChnlInfo(
						prAdapter, rRmtChnlInfo);
					if (rSelChnlInfo.rChannel.u4PrimaryChnl
						!= 0) {
						nanSchedAddCrbToChnlList(
						prAdapter,
						&rSelChnlInfo,
						u4SlotIdx, 1,
					    ENUM_TIME_BITMAP_CTRL_PERIOD_8192,
						FALSE, NULL);

						fgFound = TRUE;
						u4DefCrbNum--;
						break;
					}
				}
			}

			if ((fgFound == TRUE) && prSchedTimeline)
				NAN_TIMELINE_SET(prSchedTimeline->au4AvailMap,
						 u4SlotIdx);
		}

		if (u4DefCrbNum)
			rRetStatus = WLAN_STATUS_RESOURCES;
	}

GEN_DFT_CRB_DONE:

	nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
	return rRetStatus;
}

uint32_t
nanSchedNegoGenNdcCrb(struct ADAPTER *prAdapter) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Idx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_NDC_CTRL_T *prNdcCtrl;
	uint32_t u4SysTime;
	uint32_t u4SlotIdx;
	uint32_t u4Num;
	uint32_t u4SlotOffset;
	uint32_t u4DwIdx;
	uint32_t u4AvailDbIdx;
	union _NAN_BAND_CHNL_CTRL rLocalChnlInfo;
	union _NAN_BAND_CHNL_CTRL rRmtChnlInfo;
	union _NAN_BAND_CHNL_CTRL rSelChnlInfo;
	unsigned char fgConflict;
	struct _NAN_SCHEDULER_T *prScheduler;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prScheduler = nanGetScheduler(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	if ((prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_INITIATOR) &&
	    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_RESPONDER)) {
		rRetStatus = WLAN_STATUS_FAILURE;
		goto GEN_NDC_DONE;
	}

	/* Step1 check existed NDC CRB */
	for (u4Idx = 0; u4Idx < NAN_MAX_NDC_RECORD; u4Idx++) {
		prNdcCtrl = &prScheduler->arNdcCtrl[u4Idx];
		if (prNdcCtrl->fgValid == FALSE)
			continue;

		fgConflict = FALSE;

		for (u4SlotIdx = 0; (u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS);
		     u4SlotIdx++) {
			if (!NAN_IS_AVAIL_MAP_SET(
				    prNdcCtrl->arTimeline[0].au4AvailMap,
				    u4SlotIdx))
				continue;

			rLocalChnlInfo = nanQueryChnlInfoBySlot(
				prAdapter, u4SlotIdx, NULL, TRUE);
			if (rLocalChnlInfo.rChannel.u4PrimaryChnl == 0) {
				DBGLOG(NAN, ERROR,
				       "no committeed slot for NDC\n");
				fgConflict = TRUE;
				break;
			}

			for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
			     u4AvailDbIdx++) {
				if (nanSchedPeerAvailabilityDbValidByID(
					    prAdapter, prNegoCtrl->u4SchIdx,
					    u4AvailDbIdx) == FALSE)
					continue;

				rRmtChnlInfo = nanGetPeerChnlInfoBySlot(
					prAdapter, prNegoCtrl->u4SchIdx,
					u4AvailDbIdx, u4SlotIdx, TRUE);
				if (rRmtChnlInfo.rChannel.u4PrimaryChnl == 0)
					continue;

				if (nanSchedChkConcurrOp(rLocalChnlInfo,
							 rRmtChnlInfo) !=
				    CNM_CH_CONCURR_MCC)
					break;
			}

			if (u4AvailDbIdx >= NAN_NUM_AVAIL_DB) {
				fgConflict = TRUE;
				break;
			}
		}

		if (!fgConflict)
			break;
	}

	if (u4Idx < NAN_MAX_NDC_RECORD) {
		/* find a local NDC CRB */
		kalMemCopy((uint8_t *)&prNegoCtrl->rSelectedNdcCtrl,
		    (uint8_t *)prNdcCtrl, sizeof(struct _NAN_NDC_CTRL_T));
		nanUtilDump(prAdapter, "Reuse NDC ID", prNdcCtrl->aucNdcId,
		    NAN_NDC_ATTRIBUTE_ID_LENGTH);
		goto GEN_NDC_DONE;
	}

	/* Step2 allocate new NDC CRB */
	if (prScheduler->fgEn5gH || prScheduler->fgEn5gL)
		u4SlotOffset = NAN_5G_DW_INDEX + 1;
	else
		u4SlotOffset = NAN_2G_DW_INDEX + 1;

	if (prAdapter->rWifiVar.ucDftNdcStartOffset != 0)
		u4SlotOffset = prAdapter->rWifiVar.ucDftNdcStartOffset;

	for (u4Num = 0; u4Num < NAN_SLOTS_PER_DW_INTERVAL; u4Num++,
	    u4SlotOffset = (u4SlotOffset + 1) % NAN_SLOTS_PER_DW_INTERVAL) {

		if (nanIsDiscWindow(prAdapter, u4SlotOffset))
			continue;

		for (u4DwIdx = 0; u4DwIdx < NAN_TOTAL_DW; u4DwIdx++) {
			u4SlotIdx = u4DwIdx * NAN_SLOTS_PER_DW_INTERVAL +
				    u4SlotOffset;

			rLocalChnlInfo =
				nanGetChnlInfoBySlot(prAdapter, u4SlotIdx);
			if (rLocalChnlInfo.rChannel.u4PrimaryChnl == 0)
				continue; /* pass */

			if (!nanSchedPeerAvailabilityDbValid(
				    prAdapter, prNegoCtrl->u4SchIdx))
				continue; /* pass */

			fgConflict = TRUE;

			for (u4AvailDbIdx = 0; u4AvailDbIdx < NAN_NUM_AVAIL_DB;
			     u4AvailDbIdx++) {
				if (nanSchedPeerAvailabilityDbValidByID(
					    prAdapter, prNegoCtrl->u4SchIdx,
					    u4AvailDbIdx) == FALSE)
					continue;

				rRmtChnlInfo = nanGetPeerChnlInfoBySlot(
					prAdapter, prNegoCtrl->u4SchIdx,
					u4AvailDbIdx, u4SlotIdx, TRUE);
				if (rRmtChnlInfo.rChannel.u4PrimaryChnl == 0) {
					fgConflict = FALSE;
					break;
				}

				if (nanSchedChkConcurrOp(rLocalChnlInfo,
							 rRmtChnlInfo) !=
				    CNM_CH_CONCURR_MCC) {
					fgConflict = FALSE;
					break;
				}
			}

			if (fgConflict)
				break;
		}

		if (u4DwIdx >= NAN_TOTAL_DW)
			break;
	}

	if (u4Num < NAN_SLOTS_PER_DW_INTERVAL) {
		prNdcCtrl = &prNegoCtrl->rSelectedNdcCtrl;

		prNdcCtrl->fgValid = TRUE;
		kalMemZero(&prNdcCtrl->arTimeline[0].au4AvailMap,
			   sizeof(prNdcCtrl->arTimeline[0].au4AvailMap));
		u4SysTime = (uint32_t)kalGetTimeTick();
		kalMemCopy(prNdcCtrl->aucNdcId, (uint8_t *)&u4SysTime,
			   NAN_NDC_ATTRIBUTE_ID_LENGTH);
		prNdcCtrl->arTimeline[0].ucMapId = prNanTimelineMgmt->ucMapId;

		for (u4DwIdx = 0; u4DwIdx < NAN_TOTAL_DW; u4DwIdx++) {
			u4SlotIdx = u4DwIdx * NAN_SLOTS_PER_DW_INTERVAL +
				    u4SlotOffset;
			NAN_TIMELINE_SET(prNdcCtrl->arTimeline[0].au4AvailMap,
					 u4SlotIdx);

			rLocalChnlInfo =
				nanGetChnlInfoBySlot(prAdapter, u4SlotIdx);
			if (rLocalChnlInfo.rChannel.u4PrimaryChnl != 0)
				continue;

			/* Assign channel for new NDC */
			rSelChnlInfo = nanSchedNegoSelectChnlInfo(prAdapter,
								  u4SlotIdx);
			rRetStatus = nanSchedAddCrbToChnlList(
				prAdapter, &rSelChnlInfo, u4SlotIdx, 1,
				ENUM_TIME_BITMAP_CTRL_PERIOD_8192, FALSE, NULL);
			if (rRetStatus != WLAN_STATUS_SUCCESS)
				break;
		}

		nanUtilDump(prAdapter, "New NDC Map",
			    (uint8_t *)prNdcCtrl->arTimeline[0].au4AvailMap,
			    sizeof(prNdcCtrl->arTimeline[0].au4AvailMap));
		nanUtilDump(prAdapter, "NDC ID", prNdcCtrl->aucNdcId,
			    NAN_NDC_ATTRIBUTE_ID_LENGTH);
		/* nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__); */
	} else {
		rRetStatus = WLAN_STATUS_FAILURE;
	}

GEN_NDC_DONE:

	return rRetStatus;
}

uint32_t
nanSchedNegoAllocNdcCtrl(struct ADAPTER *prAdapter,
			 struct _NAN_NDC_CTRL_T *prSelectedNdcCtrl) {
	struct _NAN_NDC_CTRL_T *prNdcCtrl;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	do {
		prNdcCtrl = nanSchedGetNdcCtrl(prAdapter,
					       prSelectedNdcCtrl->aucNdcId);
		if ((prNdcCtrl != NULL) &&
		    (nanSchedNegoIsCrbEqual(
			     prAdapter, &prNdcCtrl->arTimeline[0],
			     prSelectedNdcCtrl->arTimeline) == FALSE)) {

			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		prNegoCtrl->rSelectedNdcCtrl.fgValid = TRUE;
		kalMemZero(
			&prNegoCtrl->rSelectedNdcCtrl.arTimeline[0].au4AvailMap,
			sizeof(prNegoCtrl->rSelectedNdcCtrl.arTimeline[0]
				       .au4AvailMap));
		kalMemCopy(prNegoCtrl->rSelectedNdcCtrl.aucNdcId,
			   prSelectedNdcCtrl->aucNdcId,
			   NAN_NDC_ATTRIBUTE_ID_LENGTH);
		prNegoCtrl->rSelectedNdcCtrl.arTimeline[0].ucMapId =
			prNanTimelineMgmt->ucMapId; /* use local MAP ID */
		nanSchedNegoMergeCrb(
			prAdapter, &prNegoCtrl->rSelectedNdcCtrl.arTimeline[0],
			prSelectedNdcCtrl->arTimeline);
		prNegoCtrl->rSelectedNdcCtrl.arTimeline[1].ucMapId =
			NAN_INVALID_MAP_ID;

		nanUtilDump(prAdapter, "Join NDC ID",
			    prSelectedNdcCtrl->aucNdcId,
			    NAN_NDC_ATTRIBUTE_ID_LENGTH);
	} while (FALSE);

	return rRetStatus;
}

uint32_t
nanSchedNegoResetControlInfo(struct ADAPTER *prAdapter,
			     enum _ENUM_NEGO_CTRL_RST_REASON_T eResetReason) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4SlotIdx;
	uint32_t u4LocalChnl;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, prNegoCtrl->u4SchIdx);

	switch (eResetReason) {
	case ENUM_NEGO_CTRL_RST_BY_WAIT_RSP_STATE:
		/* honor peer's NDC proposal in WAIT-RSP-STATE */
		prNegoCtrl->rSelectedNdcCtrl.fgValid = FALSE;

		for (u4SlotIdx = 0; u4SlotIdx < NAN_TOTAL_SLOT_WINDOWS;
		     u4SlotIdx++) {
			u4LocalChnl = nanQueryPrimaryChnlBySlot(
				prAdapter, u4SlotIdx, FALSE);
			if (u4LocalChnl == 0)
				continue;

			if ((prNegoCtrl->rImmuNdlTimeline.ucMapId !=
			     NAN_INVALID_MAP_ID) &&
			    (NAN_IS_AVAIL_MAP_SET(
				    prNegoCtrl->rImmuNdlTimeline.au4AvailMap,
				    u4SlotIdx))) {
				continue;
			}

			nanSchedDeleteCrbFromChnlList(
				prAdapter, u4SlotIdx, 1,
				ENUM_TIME_BITMAP_CTRL_PERIOD_8192, FALSE);
		}

		break;

	case ENUM_NEGO_CTRL_RST_PREPARE_CONFIRM_STATE:
		prPeerSchDesc->rSelectedNdcCtrl.fgValid = FALSE;

		if ((prNegoCtrl->eType == ENUM_NAN_NEGO_DATA_LINK) &&
		    (prPeerSchDesc->fgImmuNdlTimelineValid == TRUE)) {
			if (prNegoCtrl->rImmuNdlTimeline.ucMapId ==
			    NAN_INVALID_MAP_ID) {
				kalMemZero(&prNegoCtrl->rImmuNdlTimeline
						    .au4AvailMap,
					   sizeof(prNegoCtrl->rImmuNdlTimeline
							  .au4AvailMap));
				prNegoCtrl->rImmuNdlTimeline.ucMapId =
					prNanTimelineMgmt->ucMapId;
			}

			nanSchedNegoMergeCrb(prAdapter,
					     &prNegoCtrl->rImmuNdlTimeline,
					     prPeerSchDesc->arImmuNdlTimeline);

			prPeerSchDesc->fgImmuNdlTimelineValid = FALSE;
		}

		if ((prNegoCtrl->eType == ENUM_NAN_NEGO_RANGING) &&
		    (prPeerSchDesc->fgRangingTimelineValid == TRUE)) {
			if (prNegoCtrl->rRangingTimeline.ucMapId ==
			    NAN_INVALID_MAP_ID) {
				kalMemZero(&prNegoCtrl->rRangingTimeline
						    .au4AvailMap,
					   sizeof(prNegoCtrl->rRangingTimeline
							  .au4AvailMap));
				prNegoCtrl->rRangingTimeline.ucMapId =
					prNanTimelineMgmt->ucMapId;
			}

			nanSchedNegoMergeCrb(prAdapter,
					     &prNegoCtrl->rRangingTimeline,
					     prPeerSchDesc->arRangingTimeline);

			prPeerSchDesc->fgRangingTimelineValid = FALSE;
		}

		break;

	case ENUM_NEGO_CTRL_RST_PREPARE_WAIT_RSP_STATE:
		/* peer should propose the final NDC CRB in the last state
		 * (WAIT-RSP-STATE)
		 */
		prPeerSchDesc->rSelectedNdcCtrl.fgValid = FALSE;

		break;

	default:
		break;
	}

	return rRetStatus;
}

uint32_t
nanSchedNegoGenLocalCrbProposal(struct ADAPTER *prAdapter) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	DBGLOG(NAN, INFO, "\n\n");
	DBGLOG(NAN, INFO, "------>\n");

	nanSchedNegoDumpState(prAdapter, (uint8_t *) __func__, __LINE__);

	switch (prNegoCtrl->eState) {
	case ENUM_NAN_CRB_NEGO_STATE_INITIATOR:
		if (prNegoCtrl->eType == ENUM_NAN_NEGO_DATA_LINK) {
			/* Generate initial proposal */

			/* Determine local NDC schedule */
			if (prNegoCtrl->rSelectedNdcCtrl.fgValid == FALSE) {
				if (nanSchedNegoGenNdcCrb(prAdapter) ==
				    WLAN_STATUS_FAILURE)
					DBGLOG(NAN, WARN,
					       "gen local NDC fail\n");
			}

			/* check default CRB quota */
			nanSchedNegoGenDefCrb(prAdapter, FALSE);

			/* check QoS requirement */
			rRetStatus = nanSchedNegoChkQosSpec(prAdapter, FALSE);
			if (rRetStatus != WLAN_STATUS_SUCCESS)
				break;
		} else if (prNegoCtrl->eType == ENUM_NAN_NEGO_RANGING) {
			/* check default CRB quota */
			if (prNegoCtrl->rRangingTimeline.ucMapId ==
			    NAN_INVALID_MAP_ID)
				rRetStatus =
					nanSchedNegoGenDefCrb(prAdapter, FALSE);
		}
		break;

	default:
		/* wrong state */
		rRetStatus = WLAN_STATUS_FAILURE;
		break;
	}

	if (rRetStatus == WLAN_STATUS_SUCCESS) {
		nanSchedCmdUpdateAvailability(prAdapter);
		prNegoCtrl->eState = ENUM_NAN_CRB_NEGO_STATE_WAIT_RESP;
	} else {
		prNegoCtrl->eState = ENUM_NAN_CRB_NEGO_STATE_CONFIRM;
	}

	DBGLOG(NAN, INFO, "<------\n");

	return rRetStatus;
}

uint32_t
nanSchedNegoDataPathChkRmtCrbProposalForRspState(
	struct ADAPTER *prAdapter, uint32_t *pu4RejectCode) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4SchIdx;
	unsigned char fgCounterProposal = FALSE;
	uint32_t u4ReasonCode = NAN_REASON_CODE_RESERVED;
	unsigned char fgEmptyMapSet;
	uint32_t au4EmptyMap[NAN_TOTAL_DW];
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	u4SchIdx = prNegoCtrl->u4SchIdx;
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, u4SchIdx);

	/* Generate compliant/counter proposal */

	/* check if remote immutable NDL CRB conflict */
	if (prPeerSchDesc->fgImmuNdlTimelineValid == TRUE) {
		if (nanSchedNegoIsRmtCrbConflict(
			    prAdapter, prPeerSchDesc->arImmuNdlTimeline,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode = NAN_REASON_CODE_IMMUTABLE_UNACCEPTABLE;
			goto DATA_RESPONDER_STATE_DONE;
		} else {
			if (fgEmptyMapSet) {
				/* honor remote's channel sequence */
				nanSchedNegoDecideChnlInfoForEmptySlot(
					prAdapter, au4EmptyMap);
			}
		}
	}

	/* check if local immutable NDL CRB conflict */
	if (prNegoCtrl->rImmuNdlTimeline.ucMapId != NAN_INVALID_MAP_ID) {
		if (nanSchedNegoIsLocalCrbConflict(
			    prAdapter, &prNegoCtrl->rImmuNdlTimeline, FALSE,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode = NAN_REASON_CODE_IMMUTABLE_UNACCEPTABLE;
			goto DATA_RESPONDER_STATE_DONE;
		}

		if (nanSchedNegoIsLocalCrbConflict(
			    prAdapter, &prNegoCtrl->rImmuNdlTimeline, TRUE,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			/* expect the initiator can change conditional
			 * window to comply with immutable NDL
			 */
			fgCounterProposal = TRUE;
		} else {
			if (fgEmptyMapSet)
				fgCounterProposal = TRUE;
		}
	}

	/* Step1. Determine NDC CRB */
	if (prNegoCtrl->rSelectedNdcCtrl.fgValid == TRUE) {
		if ((prPeerSchDesc->rSelectedNdcCtrl.fgValid == TRUE) &&
		    (nanSchedNegoIsCrbEqual(
			     prAdapter,
			     &prNegoCtrl->rSelectedNdcCtrl.arTimeline[0],
			     prPeerSchDesc->rSelectedNdcCtrl.arTimeline) ==
		     FALSE)) {
			fgCounterProposal = TRUE;
		}

		if (nanSchedNegoIsLocalCrbConflict(
			    prAdapter,
			    &prNegoCtrl->rSelectedNdcCtrl.arTimeline[0], FALSE,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
			goto DATA_RESPONDER_STATE_DONE;
		}

		if (nanSchedNegoIsLocalCrbConflict(
			    prAdapter,
			    &prNegoCtrl->rSelectedNdcCtrl.arTimeline[0], TRUE,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			fgCounterProposal = TRUE;
		} else {
			if (fgEmptyMapSet)
				fgCounterProposal = TRUE;
		}
	} else if (prPeerSchDesc->rSelectedNdcCtrl.fgValid == TRUE) {
		if (nanSchedNegoIsRmtCrbConflict(
			    prAdapter,
			    prPeerSchDesc->rSelectedNdcCtrl.arTimeline,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {

			rRetStatus = nanSchedNegoGenNdcCrb(prAdapter);
			if (rRetStatus != WLAN_STATUS_SUCCESS) {
				u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
				goto DATA_RESPONDER_STATE_DONE;
			}

			fgCounterProposal = TRUE;
		} else {
			if (fgEmptyMapSet) {
				/* honor remote's channel sequence */
				nanSchedNegoDecideChnlInfoForEmptySlot(
					prAdapter, au4EmptyMap);
			}

			if (nanSchedNegoAllocNdcCtrl(
				    prAdapter,
				    &prPeerSchDesc->rSelectedNdcCtrl) !=
			    WLAN_STATUS_SUCCESS) {
				rRetStatus = WLAN_STATUS_FAILURE;
				u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
				goto DATA_RESPONDER_STATE_DONE;
			}
		}
	} else {
		rRetStatus = nanSchedNegoGenNdcCrb(prAdapter);
		if (rRetStatus != WLAN_STATUS_SUCCESS) {
			u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
			goto DATA_RESPONDER_STATE_DONE;
		}
	}

	if (fgCounterProposal) {
		/* generate default CRB quota */
		nanSchedNegoGenDefCrb(prAdapter, FALSE);

		/* check QoS requirement */
		rRetStatus = nanSchedNegoChkQosSpec(prAdapter, FALSE);
		if (rRetStatus != WLAN_STATUS_SUCCESS) {
			u4ReasonCode = NAN_REASON_CODE_QOS_UNACCEPTABLE;
			goto DATA_RESPONDER_STATE_DONE;
		}
	} else {
		/* check QoS requirement */
		rRetStatus = nanSchedNegoChkQosSpec(prAdapter, TRUE);
		if (rRetStatus != WLAN_STATUS_SUCCESS) {
			nanSchedDbgDumpTimelineDb(prAdapter, __func__,
						  __LINE__);

			fgCounterProposal = TRUE;

			/* generate default CRB quota */
			nanSchedNegoGenDefCrb(prAdapter, FALSE);

			/* check QoS requirement */
			rRetStatus = nanSchedNegoChkQosSpec(prAdapter, FALSE);
			if (rRetStatus != WLAN_STATUS_SUCCESS) {
				u4ReasonCode = NAN_REASON_CODE_QOS_UNACCEPTABLE;
				goto DATA_RESPONDER_STATE_DONE;
			}

		} else {
			/* check default CRB quota */
			nanSchedNegoGenDefCrb(prAdapter, TRUE);
		}
	}

DATA_RESPONDER_STATE_DONE:
	if ((rRetStatus == WLAN_STATUS_SUCCESS) && (fgCounterProposal == TRUE))
		rRetStatus = WLAN_STATUS_PENDING;

	if (pu4RejectCode != NULL)
		*pu4RejectCode = u4ReasonCode;

	return rRetStatus;
}

uint32_t
nanSchedNegoRangingChkRmtCrbProposalForRspState(struct ADAPTER *prAdapter,
						      uint32_t *pu4RejectCode) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4ReasonCode = NAN_REASON_CODE_RESERVED;
	unsigned char fgEmptyMapSet;
	uint32_t au4EmptyMap[NAN_TOTAL_DW];
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, prNegoCtrl->u4SchIdx);

	/* Step1. check initial ranging proposal */
	if (prPeerSchDesc->fgRangingTimelineValid == TRUE) {
		nanUtilDump(prAdapter, "Ranging Map",
			    (uint8_t *)prPeerSchDesc->arRangingTimeline[0]
				    .au4AvailMap,
			    sizeof(prPeerSchDesc->arRangingTimeline[0]
					   .au4AvailMap));
		if (nanSchedNegoIsRmtCrbConflict(
			    prAdapter, prPeerSchDesc->arRangingTimeline,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode =
				NAN_REASON_CODE_RANGING_SCHEDULE_UNACCEPTABLE;
			goto RANG_RESPONDER_STATE_DONE;
		} else {
			if (fgEmptyMapSet) {
				nanSchedNegoDecideChnlInfoForEmptySlot(
					prAdapter, au4EmptyMap);
			}
		}
	} else {
		rRetStatus = WLAN_STATUS_FAILURE;
		u4ReasonCode = NAN_REASON_CODE_RANGING_SCHEDULE_UNACCEPTABLE;
		goto RANG_RESPONDER_STATE_DONE;
	}

RANG_RESPONDER_STATE_DONE:
	if (pu4RejectCode != NULL)
		*pu4RejectCode = u4ReasonCode;

	return rRetStatus;
}

uint32_t
nanSchedNegoDataPathChkRmtCrbProposalForWaitRspState(struct ADAPTER *prAdapter,
						     uint32_t *pu4RejectCode) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4ReasonCode = NAN_REASON_CODE_RESERVED;
	unsigned char fgEmptyMapSet;
	uint32_t au4EmptyMap[NAN_TOTAL_DW];
	uint32_t u4SchIdx;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	u4SchIdx = prNegoCtrl->u4SchIdx;
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, u4SchIdx);

	nanSchedNegoResetControlInfo(prAdapter,
				     ENUM_NEGO_CTRL_RST_BY_WAIT_RSP_STATE);

	/* Generate confirm proposal */

	/* check if remote immutable NDL CRB conflict */
	if (prPeerSchDesc->fgImmuNdlTimelineValid == TRUE) {
		if (nanSchedNegoIsRmtCrbConflict(
			    prAdapter, prPeerSchDesc->arImmuNdlTimeline,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode = NAN_REASON_CODE_IMMUTABLE_UNACCEPTABLE;
			goto WAIT_RSP_STATE_DONE;
		} else {
			if (fgEmptyMapSet) {
				nanSchedNegoDecideChnlInfoForEmptySlot(
					prAdapter, au4EmptyMap);
			}
		}
	}
	/* check if local immutable NDL CRB conflict */
	if (prNegoCtrl->rImmuNdlTimeline.ucMapId != NAN_INVALID_MAP_ID) {
		if (nanSchedNegoIsLocalCrbConflict(
			    prAdapter, &prNegoCtrl->rImmuNdlTimeline, TRUE,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode = NAN_REASON_CODE_IMMUTABLE_UNACCEPTABLE;
			goto WAIT_RSP_STATE_DONE;
		} else {
			if (fgEmptyMapSet) {
				rRetStatus = WLAN_STATUS_FAILURE;
				u4ReasonCode =
					NAN_REASON_CODE_IMMUTABLE_UNACCEPTABLE;
				goto WAIT_RSP_STATE_DONE;
			}
		}
	}

	/* determine NDC CRB */
	if (prPeerSchDesc->rSelectedNdcCtrl.fgValid == TRUE) {
		if (nanSchedNegoIsRmtCrbConflict(
			    prAdapter,
			    prPeerSchDesc->rSelectedNdcCtrl.arTimeline,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
			goto WAIT_RSP_STATE_DONE;
		} else {
			if (fgEmptyMapSet) {
				nanSchedNegoDecideChnlInfoForEmptySlot(
					prAdapter, au4EmptyMap);
			}

			if (nanSchedNegoAllocNdcCtrl(
				    prAdapter,
				    &prPeerSchDesc->rSelectedNdcCtrl) !=
			    WLAN_STATUS_SUCCESS) {
				rRetStatus = WLAN_STATUS_FAILURE;
				u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
				goto WAIT_RSP_STATE_DONE;
			}
		}
	} else {
		rRetStatus = WLAN_STATUS_FAILURE;
		u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
		goto WAIT_RSP_STATE_DONE;
	}

	/* check QoS requirement */
	rRetStatus = nanSchedNegoChkQosSpec(prAdapter, TRUE);
	if (rRetStatus != WLAN_STATUS_SUCCESS) {
		rRetStatus = WLAN_STATUS_FAILURE;
		u4ReasonCode = NAN_REASON_CODE_QOS_UNACCEPTABLE;
		goto WAIT_RSP_STATE_DONE;
	}

	/* check default CRB quota */
	nanSchedNegoGenDefCrb(prAdapter, TRUE);

WAIT_RSP_STATE_DONE:

	if (pu4RejectCode != NULL)
		*pu4RejectCode = u4ReasonCode;

	return rRetStatus;
}

uint32_t
nanSchedNegoRangingChkRmtCrbProposalForWaitRspState(struct ADAPTER *prAdapter,
						    uint32_t *pu4RejectCode) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4ReasonCode = NAN_REASON_CODE_RESERVED;
	unsigned char fgEmptyMapSet;
	uint32_t au4EmptyMap[NAN_TOTAL_DW];
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_PEER_SCH_DESC_T *prPeerSchDesc;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prPeerSchDesc = nanSchedGetPeerSchDesc(prAdapter, prNegoCtrl->u4SchIdx);

	if (prPeerSchDesc->fgRangingTimelineValid == TRUE) {
		nanUtilDump(prAdapter, "Ranging Map",
			    (uint8_t *)prPeerSchDesc->arRangingTimeline[0]
				    .au4AvailMap,
			    sizeof(prPeerSchDesc->arRangingTimeline[0]
					   .au4AvailMap));
		if (nanSchedNegoIsRmtCrbConflict(
			    prAdapter, prPeerSchDesc->arRangingTimeline,
			    &fgEmptyMapSet, au4EmptyMap) == TRUE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode =
				NAN_REASON_CODE_RANGING_SCHEDULE_UNACCEPTABLE;
			goto WAIT_RSP_STATE_DONE;
		} else {
			if (fgEmptyMapSet) {
				nanSchedNegoDecideChnlInfoForEmptySlot(
					prAdapter, au4EmptyMap);
			}
		}
	} else {
		rRetStatus = WLAN_STATUS_FAILURE;
		u4ReasonCode = NAN_REASON_CODE_RANGING_SCHEDULE_UNACCEPTABLE;
		goto WAIT_RSP_STATE_DONE;
	}

WAIT_RSP_STATE_DONE:

	if (pu4RejectCode != NULL)
		*pu4RejectCode = u4ReasonCode;

	return rRetStatus;
}

uint32_t
nanSchedNegoChkRmtCrbProposal(struct ADAPTER *prAdapter,
			      uint32_t *pu4RejectCode) {
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4ReasonCode = NAN_REASON_CODE_RESERVED;
	struct _NAN_NDC_CTRL_T *prNdcCtrl;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);
	prPeerSchRec =
		nanSchedGetPeerSchRecord(prAdapter, prNegoCtrl->u4SchIdx);

	DBGLOG(NAN, INFO, "\n\n");
	DBGLOG(NAN, INFO, "------>\n");

	nanSchedNegoDumpState(prAdapter, (uint8_t *) __func__, __LINE__);

	if (nanSchedNegoIsRmtAvailabilityConflict(prAdapter) != 0) {
		u4ReasonCode = NAN_REASON_CODE_INVALID_AVAILABILITY;
		rRetStatus = WLAN_STATUS_FAILURE;
		goto RMT_PROPOSAL_DONE;
	}

	switch (prNegoCtrl->eState) {
	case ENUM_NAN_CRB_NEGO_STATE_RESPONDER:
		if (prNegoCtrl->eType == ENUM_NAN_NEGO_DATA_LINK) {
			rRetStatus =
			nanSchedNegoDataPathChkRmtCrbProposalForRspState(
			    prAdapter, &u4ReasonCode);
		} else if (prNegoCtrl->eType == ENUM_NAN_NEGO_RANGING) {
			rRetStatus =
			nanSchedNegoRangingChkRmtCrbProposalForRspState(
			    prAdapter, &u4ReasonCode);
		} else {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
			goto RMT_PROPOSAL_DONE;
		}
		break;

	case ENUM_NAN_CRB_NEGO_STATE_WAIT_RESP:
		if (prNegoCtrl->eType == ENUM_NAN_NEGO_DATA_LINK) {
			rRetStatus =
			nanSchedNegoDataPathChkRmtCrbProposalForWaitRspState(
			    prAdapter, &u4ReasonCode);
		} else if (prNegoCtrl->eType == ENUM_NAN_NEGO_RANGING) {
			rRetStatus =
			nanSchedNegoRangingChkRmtCrbProposalForWaitRspState(
			    prAdapter, &u4ReasonCode);
		} else {
			rRetStatus = WLAN_STATUS_FAILURE;
			u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
			goto RMT_PROPOSAL_DONE;
		}
		break;

	case ENUM_NAN_CRB_NEGO_STATE_CONFIRM:
		rRetStatus = WLAN_STATUS_SUCCESS;
		u4ReasonCode = NAN_REASON_CODE_RESERVED;
		break;

	default:
		/* wrong state */
		rRetStatus = WLAN_STATUS_FAILURE;
		u4ReasonCode = NAN_REASON_CODE_UNSPECIFIED;
		break;
	}

RMT_PROPOSAL_DONE:
	if (rRetStatus == WLAN_STATUS_PENDING) {
		rRetStatus = WLAN_STATUS_NOT_ACCEPTED;

		prNegoCtrl->eState = ENUM_NAN_CRB_NEGO_STATE_WAIT_RESP;
		nanSchedNegoResetControlInfo(
			prAdapter, ENUM_NEGO_CTRL_RST_PREPARE_WAIT_RSP_STATE);

		nanSchedCmdUpdateAvailability(prAdapter);
	} else if (rRetStatus == WLAN_STATUS_SUCCESS) {
		prNegoCtrl->eState = ENUM_NAN_CRB_NEGO_STATE_CONFIRM;
		nanSchedNegoResetControlInfo(
			prAdapter, ENUM_NEGO_CTRL_RST_PREPARE_CONFIRM_STATE);

		/* save negotiation result to peer sch record */
		if (prNegoCtrl->eType == ENUM_NAN_NEGO_DATA_LINK) {
			prNdcCtrl = nanSchedGetNdcCtrl(
				prAdapter,
				prNegoCtrl->rSelectedNdcCtrl.aucNdcId);
			if (prNdcCtrl == NULL) {
				prNdcCtrl = nanSchedAcquireNdcCtrl(prAdapter);
				if (prNdcCtrl == NULL) {
					rRetStatus = WLAN_STATUS_FAILURE;
					u4ReasonCode =
						NAN_REASON_CODE_UNSPECIFIED;
					goto RMT_PROPOSAL_DONE;
				}

				kalMemCopy(prNdcCtrl,
					   &prNegoCtrl->rSelectedNdcCtrl,
					   sizeof(struct _NAN_NDC_CTRL_T));
			}
			prPeerSchRec->prCommNdcCtrl = prNdcCtrl;

			prPeerSchRec->u4DefNdlNumSlots =
				prNegoCtrl->u4DefNdlNumSlots;

			kalMemCopy(&prPeerSchRec->rCommImmuNdlTimeline,
				   &prNegoCtrl->rImmuNdlTimeline,
				   sizeof(prNegoCtrl->rImmuNdlTimeline));

			prPeerSchRec->u4FinalQosMinSlots =
				prNegoCtrl->u4NegoQosMinSlots;
			prPeerSchRec->u4FinalQosMaxLatency =
				prNegoCtrl->u4NegoQosMaxLatency;

			prPeerSchRec->fgUseDataPath = TRUE;
		} else if (prNegoCtrl->eType == ENUM_NAN_NEGO_RANGING) {
			prPeerSchRec->u4DefRangingNumSlots =
				prNegoCtrl->u4DefRangingNumSlots;

			kalMemCopy(&prPeerSchRec->rCommRangingTimeline,
				   &prNegoCtrl->rRangingTimeline,
				   sizeof(prNegoCtrl->rRangingTimeline));

			prPeerSchRec->fgUseRanging = TRUE;
		}

		/* step1: commit conditional windows */
		nanSchedNegoCommitCondChnlList(prAdapter);
		prNanTimelineMgmt->fgChkCondAvailability = FALSE;
		/* step2: determine common FAW CRB */
		nanSchedPeerUpdateCommonFAW(prAdapter, prNegoCtrl->u4SchIdx);
		/* step3: release unused slots */
		nanSchedReleaseUnusedCommitSlot(prAdapter);

		nanSchedCmdUpdateAvailability(prAdapter);
	} else {
		prNegoCtrl->eState = ENUM_NAN_CRB_NEGO_STATE_CONFIRM;
	}

	if (pu4RejectCode != NULL)
		*pu4RejectCode = u4ReasonCode;

	nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
	DBGLOG(NAN, INFO, "Reason:%x\n", u4ReasonCode);
	DBGLOG(NAN, INFO, "<------\n");

	return rRetStatus;
}

uint32_t
nanSchedNegoGetImmuNdlScheduleList(struct ADAPTER *prAdapter,
				   uint8_t **ppucScheduleList,
				   uint32_t *pu4ScheduleListLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint8_t *pucPos = g_aucNanIEBuffer;
	uint32_t u4RetLength;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_SCHEDULE_ENTRY_T *prScheduleEntry;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	do {
		if ((prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_INITIATOR) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_RESPONDER) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_WAIT_RESP) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_CONFIRM)) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		if (prNegoCtrl->eType != ENUM_NAN_NEGO_DATA_LINK) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		if (prNegoCtrl->rImmuNdlTimeline.ucMapId !=
		    NAN_INVALID_MAP_ID) {
			prScheduleEntry =
				(struct _NAN_SCHEDULE_ENTRY_T *)pucPos;
			prScheduleEntry->ucMapID = prNanTimelineMgmt->ucMapId;
			pucPos += 1 /* MapID(1) */;
			nanParserGenTimeBitmapField(
				prAdapter,
				prNegoCtrl->rImmuNdlTimeline.au4AvailMap,
				pucPos, &u4RetLength);
			pucPos += u4RetLength;
		}
	} while (FALSE);

	if (ppucScheduleList)
		*ppucScheduleList = g_aucNanIEBuffer;
	if (pu4ScheduleListLength)
		*pu4ScheduleListLength = (pucPos - g_aucNanIEBuffer);

	return rRetStatus;
}

uint32_t
nanSchedNegoGetRangingScheduleList(struct ADAPTER *prAdapter,
				   uint8_t **ppucScheduleList,
				   uint32_t *pu4ScheduleListLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint8_t *pucPos = g_aucNanIEBuffer;
	uint32_t u4RetLength;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_SCHEDULE_ENTRY_T *prScheduleEntry;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	do {
		if ((prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_INITIATOR) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_RESPONDER) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_WAIT_RESP) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_CONFIRM)) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		if (prNegoCtrl->eType != ENUM_NAN_NEGO_RANGING) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		if (prNegoCtrl->rRangingTimeline.ucMapId !=
		    NAN_INVALID_MAP_ID) {
			prScheduleEntry =
				(struct _NAN_SCHEDULE_ENTRY_T *)pucPos;
			prScheduleEntry->ucMapID = prNanTimelineMgmt->ucMapId;
			pucPos += 1 /* MapID(1) */;
			nanParserGenTimeBitmapField(
				prAdapter,
				prNegoCtrl->rRangingTimeline.au4AvailMap,
				pucPos, &u4RetLength);
			pucPos += u4RetLength;
		}
	} while (FALSE);

	if (ppucScheduleList)
		*ppucScheduleList = g_aucNanIEBuffer;
	if (pu4ScheduleListLength)
		*pu4ScheduleListLength = (pucPos - g_aucNanIEBuffer);

	return rRetStatus;
}

uint32_t
nanSchedNegoGetQosAttr(struct ADAPTER *prAdapter, uint8_t **ppucQosAttr,
		       uint32_t *pu4QosLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint8_t *pucPos = g_aucNanIEBuffer;
	struct _NAN_ATTR_NDL_QOS_T *prNdlQosAttr;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;

	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	do {
		if ((prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_INITIATOR) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_RESPONDER) &&
		    (prNegoCtrl->eState != ENUM_NAN_CRB_NEGO_STATE_WAIT_RESP)) {
			rRetStatus = WLAN_STATUS_FAILURE;
			break;
		}

		if ((prNegoCtrl->u4QosMaxLatency <
		     NAN_INVALID_QOS_MAX_LATENCY) ||
		    (prNegoCtrl->u4QosMinSlots > NAN_INVALID_QOS_MIN_SLOTS)) {
			prNdlQosAttr = (struct _NAN_ATTR_NDL_QOS_T *)pucPos;
			pucPos += sizeof(struct _NAN_ATTR_NDL_QOS_T);

			prNdlQosAttr->ucAttrId = NAN_ATTR_ID_NDL_QOS;
			prNdlQosAttr->u2Length = 3;
			prNdlQosAttr->u2MaxLatency =
				prNegoCtrl->u4QosMaxLatency;
			prNdlQosAttr->ucMinTimeSlot = prNegoCtrl->u4QosMinSlots;
		}
	} while (FALSE);

	if (ppucQosAttr)
		*ppucQosAttr = g_aucNanIEBuffer;
	if (pu4QosLength)
		*pu4QosLength = (pucPos - g_aucNanIEBuffer);

	return rRetStatus;
}

uint32_t
nanSchedNegoGetSelectedNdcAttr(struct ADAPTER *prAdapter, uint8_t **ppucNdcAttr,
			       uint32_t *pu4NdcAttrLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint8_t *pucPos;
	uint32_t u4RetLength;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_ATTR_NDC_T *prNdcAttr;
	struct _NAN_SCHEDULE_ENTRY_T *prNanScheduleEntry;
	struct _NAN_NDC_CTRL_T *prNdcCtrl;

	do {
		pucPos = g_aucNanIEBuffer;

		prNegoCtrl = nanGetNegoControlBlock(prAdapter);

		prNdcCtrl = &prNegoCtrl->rSelectedNdcCtrl;
		if (prNdcCtrl->fgValid == FALSE) {
			rRetStatus = WLAN_STATUS_FAILURE;
			DBGLOG(NAN, ERROR,
			       "Can't find valid NDC control block\n");
			break;
		}

		kalMemZero(g_aucNanIEBuffer, NAN_IE_BUF_MAX_SIZE);

		prNdcAttr = (struct _NAN_ATTR_NDC_T *)pucPos;
		prNdcAttr->ucAttrId = NAN_ATTR_ID_NDC;
		kalMemCopy(prNdcAttr->aucNDCID, prNdcCtrl->aucNdcId,
			   NAN_NDC_ATTRIBUTE_ID_LENGTH);
		prNdcAttr->ucAttributeControl |=
			NAN_ATTR_NDC_CTRL_SELECTED_FOR_NDL;
		prNdcAttr->u2Length = 7; /* ID(6)+Attribute Control(1) */

		pucPos = prNdcAttr->aucScheduleEntryList;
		prNanScheduleEntry = (struct _NAN_SCHEDULE_ENTRY_T *)pucPos;
		prNanScheduleEntry->ucMapID = prNdcCtrl->arTimeline[0].ucMapId;
		pucPos++; /* Map ID(1) */
		nanParserGenTimeBitmapField(
			prAdapter, prNdcCtrl->arTimeline[0].au4AvailMap, pucPos,
			&u4RetLength);
		prNdcAttr->u2Length += (1 /*MapID*/ + u4RetLength);
		pucPos += u4RetLength;
	} while (FALSE);

	*ppucNdcAttr = g_aucNanIEBuffer;
	*pu4NdcAttrLength = (pucPos - g_aucNanIEBuffer);

	return rRetStatus;
}

uint32_t
nanSchedAddPotentialWindows(struct ADAPTER *prAdapter, uint8_t *pucBuf) {
#define NAN_POTENTIAL_BAND 0
#define NAN_POTENTIAL_CHANNEL 1
	uint8_t *pucPos;
	uint8_t *pucTmp;
	uint32_t u4EntryIdx;
	struct _NAN_AVAILABILITY_ENTRY_T *prAvailEntry;
	uint32_t u4RetLength;
	uint32_t u2EntryControl;
	uint32_t au4PotentialAvailMap[NAN_TOTAL_DW];
	struct _NAN_SCHEDULER_T *prScheduler;
	uint32_t u4Idx;
#if NAN_POTENTIAL_BAND
	union _NAN_BAND_CHNL_CTRL rPotentialBandInfo;
#endif
#if NAN_POTENTIAL_CHANNEL
	uint8_t *pucPotentialChnls;
	uint32_t u4PotentialChnlSize;
#endif
	struct _NAN_SPECIFIC_BSS_INFO_T *prNanSpecificBssInfo;
	struct BSS_INFO *prBssInfo;

	prScheduler = nanGetScheduler(prAdapter);
	prNanSpecificBssInfo =
		nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter,
					  prNanSpecificBssInfo->ucBssIndex);

	pucPos = pucBuf;

	kalMemSet(au4PotentialAvailMap, 0xFF, sizeof(au4PotentialAvailMap));
	if (prScheduler->fgEn2g) {
		for (u4EntryIdx = 0; u4EntryIdx < NAN_TOTAL_DW; u4EntryIdx++)
			NAN_TIMELINE_UNSET(
				au4PotentialAvailMap,
				u4EntryIdx * NAN_SLOTS_PER_DW_INTERVAL +
					NAN_2G_DW_INDEX);
	}
	if (prScheduler->fgEn5gH || prScheduler->fgEn5gL) {
		for (u4EntryIdx = 0; u4EntryIdx < NAN_TOTAL_DW; u4EntryIdx++)
			NAN_TIMELINE_UNSET(
				au4PotentialAvailMap,
				u4EntryIdx * NAN_SLOTS_PER_DW_INTERVAL +
					NAN_5G_DW_INDEX);
	}

/* potential channel */
#if NAN_POTENTIAL_CHANNEL
	pucTmp = pucPos;

	prAvailEntry = (struct _NAN_AVAILABILITY_ENTRY_T *)pucTmp;

	/* whsu */
	u2EntryControl =
		((prBssInfo->ucOpRxNss
		  << NAN_AVAIL_ENTRY_CTRL_RX_NSS_OFFSET) &
		 NAN_AVAIL_ENTRY_CTRL_RX_NSS) |
		((1
		  << NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT_OFFSET) &
		 NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT) |
		((3 << NAN_AVAIL_ENTRY_CTRL_USAGE_PREF_OFFSET) &
		 NAN_AVAIL_ENTRY_CTRL_USAGE_PREF) |
		((NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_POTN
		  << NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_OFFSET) &
		 NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE);
	prAvailEntry->u2EntryControl = u2EntryControl;
	pucPos += 4 /* length(2)+entry control(2) */;

	nanParserGenTimeBitmapField(prAdapter, au4PotentialAvailMap, pucPos,
				    &u4RetLength);
	pucPos += u4RetLength;

	*pucPos = ((prScheduler->u4NumOfPotentialChnlList
		    << NAN_BAND_CH_ENTRY_LIST_NUM_ENTRY_OFFSET) |
		   (NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL));
	pucPos++;
	for (u4Idx = 0; u4Idx < prScheduler->u4NumOfPotentialChnlList;
	     u4Idx++) {
		pucPotentialChnls =
			(uint8_t *)&prScheduler->arPotentialChnlList[u4Idx];
		u4PotentialChnlSize = 4;

		kalMemCopy(pucPos, pucPotentialChnls, u4PotentialChnlSize);
		pucPos += u4PotentialChnlSize;
	}

	prAvailEntry->u2Length = (pucPos - pucTmp) - 2 /* length(2) */;
#endif

/* potential band */
#if NAN_POTENTIAL_BAND
	rPotentialBandInfo.rBand.u4BandIdMask = 0;
	rPotentialBandInfo.rBand.u4Type = NAN_BAND_CH_ENTRY_LIST_TYPE_BAND;
	if (prScheduler->fgEn2g)
		rPotentialBandInfo.rBand.u4BandIdMask |=
			BIT(NAN_SUPPORTED_BAND_ID_2P4G);
	if (prScheduler->fgEn5gH || prScheduler->fgEn5gL)
		rPotentialBandInfo.rBand.u4BandIdMask |=
			BIT(NAN_SUPPORTED_BAND_ID_5G);

	if (rPotentialBandInfo.rBand.u4BandIdMask != 0) {
		pucTmp = pucPos;
		prAvailEntry = (struct _NAN_AVAILABILITY_ENTRY_T *)pucTmp;

		u2EntryControl =
			((prBssInfo->ucOpRxNss
			  << NAN_AVAIL_ENTRY_CTRL_RX_NSS_OFFSET) &
			 NAN_AVAIL_ENTRY_CTRL_RX_NSS) |
			((1
			  << NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT_OFFSET) &
			 NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT) |
			((3
			  << NAN_AVAIL_ENTRY_CTRL_USAGE_PREF_OFFSET) &
			 NAN_AVAIL_ENTRY_CTRL_USAGE_PREF) |
			((NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_POTN
			  << NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_OFFSET) &
			 NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE);
		prAvailEntry->u2EntryControl = u2EntryControl;
		pucPos += 4 /* length(2)+entry control(2) */;

		nanParserGenTimeBitmapField(prAdapter, au4PotentialAvailMap,
					    pucPos, &u4RetLength);
		pucPos += u4RetLength;

		nanParserGenBandChnlEntryListField(prAdapter,
						   &rPotentialBandInfo, 1,
						   pucPos, &u4RetLength);
		pucPos += u4RetLength;

		prAvailEntry->u2Length = (pucPos - pucTmp) - 2 /* length(2) */;
	}
#endif

	nanUtilDump(prAdapter, "Potential Windows", pucBuf, (pucPos - pucBuf));
	return (pucPos - pucBuf);
}

uint32_t
nanSchedGetAvailabilityAttr(struct ADAPTER *prAdapter,
			    uint8_t **ppucAvailabilityAttr,
			    uint32_t *pu4AvailabilityAttrLength) {
	uint8_t *pucPos;
	struct _NAN_ATTR_NAN_AVAILABILITY_T *prAvailAttr;
	struct _NAN_AVAILABILITY_ENTRY_T *prAvailEntry;
	uint32_t u4EntryIdx;
	struct _NAN_CHANNEL_TIMELINE_T *prChnlTimeline;
	uint32_t u4RetLength;
	uint32_t u2EntryControl;
	uint32_t u2EntryLength;
	struct _NAN_SCHEDULER_T *prScheduler;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_CRB_NEGO_CTRL_T *prNegoCtrl;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNanSpecificBssInfo;
	struct BSS_INFO *prBssInfo;

	prScheduler = nanGetScheduler(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);
	prNegoCtrl = nanGetNegoControlBlock(prAdapter);

	prNanSpecificBssInfo =
		nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter,
					  prNanSpecificBssInfo->ucBssIndex);

	prAvailAttr = (struct _NAN_ATTR_NAN_AVAILABILITY_T *)g_aucNanIEBuffer;
	kalMemZero(g_aucNanIEBuffer, NAN_IE_BUF_MAX_SIZE);

	prAvailAttr->ucAttrId = NAN_ATTR_ID_NAN_AVAILABILITY;
	prAvailAttr->u2AttributeControl =
		prScheduler->u2NanCurrAvailAttrControlField |
		(prNanTimelineMgmt->ucMapId & NAN_AVAIL_CTRL_MAPID);
	prAvailAttr->ucSeqID = prScheduler->ucNanAvailAttrSeqId;
	prAvailAttr->u2Length = 3; /* Sequence ID + Attriburte Control */

	pucPos = prAvailAttr->aucAvailabilityEntryList;

	/* Commit availability type */
	for (u4EntryIdx = 0, prChnlTimeline = &prNanTimelineMgmt->arChnlList[0];
	     u4EntryIdx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM;
	     u4EntryIdx++, prChnlTimeline++) {

		if (prChnlTimeline->fgValid == FALSE)
			continue;

		prAvailEntry = (struct _NAN_AVAILABILITY_ENTRY_T *)pucPos;

		u2EntryControl =
			((prBssInfo->ucOpRxNss
			  << NAN_AVAIL_ENTRY_CTRL_RX_NSS_OFFSET) &
			 NAN_AVAIL_ENTRY_CTRL_RX_NSS) |
			((1
			  << NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT_OFFSET) &
			 NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT) |
			((NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COMMIT
			  << NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_OFFSET) &
			 NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE);
		prAvailEntry->u2EntryControl = u2EntryControl;
		u2EntryLength = 2;

		pucPos += 4 /* length(2)+entry control(2) */;

		nanParserGenTimeBitmapField(prAdapter,
					    prChnlTimeline->au4AvailMap, pucPos,
					    &u4RetLength);
		u2EntryLength += u4RetLength;
		pucPos += u4RetLength;

		nanParserGenBandChnlEntryListField(prAdapter,
						   &prChnlTimeline->rChnlInfo,
						   1, pucPos, &u4RetLength);
		u2EntryLength += u4RetLength;
		pucPos += u4RetLength;

		prAvailEntry->u2Length = u2EntryLength;
		prAvailAttr->u2Length += (u2EntryLength + 2 /* length(2) */);
	}

	/* Conditional availability type */
	if (prNanTimelineMgmt->fgChkCondAvailability == TRUE) {
		prAvailAttr->u2AttributeControl |=
			NAN_AVAIL_CTRL_COMMIT_CHANGED;

		for (u4EntryIdx = 0,
		    prChnlTimeline = &prNanTimelineMgmt->arCondChnlList[0];
		     u4EntryIdx < NAN_TIMELINE_MGMT_CHNL_LIST_NUM;
		     u4EntryIdx++, prChnlTimeline++) {

			if (prChnlTimeline->fgValid == FALSE)
				continue;

			prAvailEntry =
				(struct _NAN_AVAILABILITY_ENTRY_T *)pucPos;

#if CFG_NAN_SIGMA_TEST
			if (prNegoCtrl->eType == ENUM_NAN_NEGO_RANGING) {
				u2EntryControl =
				((prBssInfo->ucOpRxNss <<
				NAN_AVAIL_ENTRY_CTRL_RX_NSS_OFFSET) &
				NAN_AVAIL_ENTRY_CTRL_RX_NSS) |
				  ((1 <<
				  NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT_OFFSET) &
				  NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT) |
				  ((NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COMMIT <<
				  NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_OFFSET) &
				  NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE);
			} else {
				u2EntryControl =
				((prBssInfo->ucOpRxNss <<
				NAN_AVAIL_ENTRY_CTRL_RX_NSS_OFFSET) &
				NAN_AVAIL_ENTRY_CTRL_RX_NSS) |
				 ((1 <<
				 NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT_OFFSET) &
				 NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT) |
				 ((NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COND <<
				 NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_OFFSET) &
				 NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE);
			}
#else
			u2EntryControl =
			    ((prBssInfo->ucOpRxNss <<
			    NAN_AVAIL_ENTRY_CTRL_RX_NSS_OFFSET) &
			    NAN_AVAIL_ENTRY_CTRL_RX_NSS) |
				((1 <<
				NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT_OFFSET) &
				NAN_AVAIL_ENTRY_CTRL_TBITMAP_PRESENT) |
				((NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_COND <<
				NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE_OFFSET) &
				 NAN_AVAIL_ENTRY_CTRL_AVAIL_TYPE);
#endif
			prAvailEntry->u2EntryControl = u2EntryControl;
			u2EntryLength = 2;

			pucPos += 4 /* length(2)+entry control(2) */;

			nanParserGenTimeBitmapField(prAdapter,
						    prChnlTimeline->au4AvailMap,
						    pucPos, &u4RetLength);
			u2EntryLength += u4RetLength;
			pucPos += u4RetLength;

			nanParserGenBandChnlEntryListField(
				prAdapter, &prChnlTimeline->rChnlInfo, 1,
				pucPos, &u4RetLength);
			u2EntryLength += u4RetLength;
			pucPos += u4RetLength;

			prAvailEntry->u2Length = u2EntryLength;
			prAvailAttr->u2Length +=
				(u2EntryLength + 2 /* length(2) */);
		}
	}

	/* add potential availability */
	u4RetLength = nanSchedAddPotentialWindows(prAdapter, pucPos);
	pucPos += u4RetLength;
	prAvailAttr->u2Length += u4RetLength;

	if (ppucAvailabilityAttr)
		*ppucAvailabilityAttr = g_aucNanIEBuffer;
	if (pu4AvailabilityAttrLength)
		*pu4AvailabilityAttrLength = (pucPos - g_aucNanIEBuffer);

	nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
	/* nanUtilDump(prAdapter, "Avail Attr",
	 *		g_aucNanIEBuffer, (pucPos-g_aucNanIEBuffer));
	 */

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSchedGetDevCapabilityAttr(struct ADAPTER *prAdapter,
		uint8_t **ppucDevCapAttr,
		uint32_t *pu4DevCapAttrLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_ATTR_DEVICE_CAPABILITY_T *prAttrDevCap;
	struct _NAN_SCHEDULER_T *prScheduler;
	struct _NAN_SPECIFIC_BSS_INFO_T *prNanSpecificBssInfo;
	struct BSS_INFO *prBssInfo;

	prScheduler = nanGetScheduler(prAdapter);
	prAttrDevCap = &prScheduler->rAttrDevCap;

	if (prScheduler->fgAttrDevCapValid) {
		if (ppucDevCapAttr)
			*ppucDevCapAttr = (uint8_t *)prAttrDevCap;
		if (pu4DevCapAttrLength)
			*pu4DevCapAttrLength =
				sizeof(struct _NAN_ATTR_DEVICE_CAPABILITY_T);

		return WLAN_STATUS_SUCCESS;
	}

	prAttrDevCap->ucAttrId = NAN_ATTR_ID_DEVICE_CAPABILITY;
	prAttrDevCap->u2Length = sizeof(struct _NAN_ATTR_DEVICE_CAPABILITY_T) -
				 3 /* ID(1)+Length(2) */;
	prAttrDevCap->ucMapID = 0; /* applied to all MAP */

	prAttrDevCap->u2CommittedDWInfo = 0;
	prAttrDevCap->u2CommittedDWInfo |=
		(prScheduler->ucCommitDwInterval
		 << NAN_COMMITTED_DW_INFO_24G_OFFSET);
	if (prScheduler->fgEn5gL || prScheduler->fgEn5gH)
		prAttrDevCap->u2CommittedDWInfo |=
			(prScheduler->ucCommitDwInterval
			 << NAN_COMMITTED_DW_INFO_5G_OFFSET);

	prAttrDevCap->ucSupportedBands = 0;
	if (prScheduler->fgEn2g)
		prAttrDevCap->ucSupportedBands |=
			BIT(NAN_SUPPORTED_BAND_ID_2P4G);
	if (prScheduler->fgEn5gL || prScheduler->fgEn5gH)
		prAttrDevCap->ucSupportedBands |= BIT(NAN_SUPPORTED_BAND_ID_5G);

	/* Fixme */
	prAttrDevCap->ucOperationMode = 0;

	prNanSpecificBssInfo =
		nanGetSpecificBssInfo(prAdapter, NAN_BSS_INDEX_BAND0);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter,
					  prNanSpecificBssInfo->ucBssIndex);
	prAttrDevCap->ucNumOfAntennas =
		(prBssInfo->ucOpRxNss << 4) | (prBssInfo->ucOpRxNss);
	DBGLOG(NAN, INFO, "Bss idx:%d, Nss:%d\n", prBssInfo->ucBssIndex,
	       prBssInfo->ucOpRxNss);

	prAttrDevCap->u2MaxChannelSwitchTime = g_u4MaxChnlSwitchTimeUs;

	prAttrDevCap->ucCapabilities = 0;
	if (prAdapter->rWifiVar.fgEnableNDPE)
		prAttrDevCap->ucCapabilities |=
			NAN_ATTR_DEVICE_CAPABILITY_CAP_SUPPORT_NDPE;

	if (ppucDevCapAttr)
		*ppucDevCapAttr = (uint8_t *)prAttrDevCap;
	if (pu4DevCapAttrLength)
		*pu4DevCapAttrLength =
			sizeof(struct _NAN_ATTR_DEVICE_CAPABILITY_T);

	prScheduler->fgAttrDevCapValid = TRUE;

	return rRetStatus;
}

uint32_t
nanSchedGetUnalignedScheduleAttr(struct ADAPTER *prAdapter,
				 uint8_t **ppucUnalignedScheduleAttr,
				 uint32_t *pu4UnalignedScheduleLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint8_t *pucPos = g_aucNanIEBuffer;

	if (ppucUnalignedScheduleAttr)
		*ppucUnalignedScheduleAttr = g_aucNanIEBuffer;
	if (pu4UnalignedScheduleLength)
		*pu4UnalignedScheduleLength = (pucPos - g_aucNanIEBuffer);

	return rRetStatus;
}

uint32_t
nanSchedGetExtWlanInfraAttr(struct ADAPTER *prAdapter,
			    uint8_t **ppucExtWlanInfraAttr,
			    uint32_t *pu4ExtWlanInfraLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint8_t *pucPos = g_aucNanIEBuffer;

	if (ppucExtWlanInfraAttr)
		*ppucExtWlanInfraAttr = g_aucNanIEBuffer;
	if (pu4ExtWlanInfraLength)
		*pu4ExtWlanInfraLength = (pucPos - g_aucNanIEBuffer);

	return rRetStatus;
}

uint32_t
nanSchedGetPublicAvailAttr(struct ADAPTER *prAdapter,
			   uint8_t **ppucPublicAvailAttr,
			   uint32_t *pu4PublicAvailLength) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	uint8_t *pucPos = g_aucNanIEBuffer;

	if (ppucPublicAvailAttr)
		*ppucPublicAvailAttr = g_aucNanIEBuffer;
	if (pu4PublicAvailLength)
		*pu4PublicAvailLength = (pucPos - g_aucNanIEBuffer);

	return rRetStatus;
}

uint32_t
nanSchedCmdUpdatePotentialChnlList(IN struct ADAPTER *prAdapter) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_SCHED_CMD_UPDATE_PONTENTIAL_CHNL_LIST_T
		*prCmdUpdatePontentialChnlList = NULL;
	struct _NAN_POTENTIAL_CHNL_MAP_T *prPotentialChnlMap;
	struct _NAN_POTENTIAL_CHNL_T *prPotentialChnlList;
	uint32_t u4Idx, u4Num;
	enum ENUM_BAND eBand;
	enum _NAN_CHNL_BW_MAP eBwMap;
	struct _NAN_SCHEDULER_T *prNanScheduler;
	union _NAN_BAND_CHNL_CTRL rFixChnl;

	prNanScheduler = nanGetScheduler(prAdapter);

	u4CmdBufferLen =
		sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
		sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
		sizeof(struct _NAN_SCHED_CMD_UPDATE_PONTENTIAL_CHNL_LIST_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(NAN, ERROR, "Memory allocation fail\n");
		return WLAN_STATUS_FAILURE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(
		NAN_CMD_UPDATE_POTENTIAL_CHNL_LIST,
		sizeof(struct _NAN_SCHED_CMD_UPDATE_PONTENTIAL_CHNL_LIST_T),
		u4CmdBufferLen, prCmdBuffer);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);
	if (prTlvElement == NULL) {
		DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prCmdUpdatePontentialChnlList =
		(struct _NAN_SCHED_CMD_UPDATE_PONTENTIAL_CHNL_LIST_T *)
			prTlvElement->aucbody;
	kalMemZero(prCmdUpdatePontentialChnlList,
		   sizeof(struct _NAN_SCHED_CMD_UPDATE_PONTENTIAL_CHNL_LIST_T));

	u4Num = 0;
	prPotentialChnlList = prCmdUpdatePontentialChnlList->arChnlList;

	for (prPotentialChnlMap = g_arPotentialChnlMap;
	     prPotentialChnlMap->ucPrimaryChnl != 0; prPotentialChnlMap++) {
		eBwMap = prAdapter->rWifiVar.ucNanBandwidth;
		if (((prPotentialChnlMap->ucPrimaryChnl < 36) ||
		     (!prAdapter->rWifiVar.fgEnNanVHT)) &&
		    (eBwMap > NAN_CHNL_BW_40))
			eBwMap = NAN_CHNL_BW_40;

		if (prPotentialChnlMap->ucPrimaryChnl < 36) {
			if (!prNanScheduler->fgEn2g)
				continue;

			eBand = BAND_2G4;
		} else if (prPotentialChnlMap->ucPrimaryChnl < 100) {
			if (!prNanScheduler->fgEn5gL)
				continue;

			eBand = BAND_5G;
		} else {
			if (!prNanScheduler->fgEn5gH)
				continue;

			eBand = BAND_5G;
		}

		while (prPotentialChnlMap->aucOperatingClass[eBwMap] == 0)
			eBwMap--;

		if (!rlmDomainIsLegalChannel(prAdapter, eBand,
					     prPotentialChnlMap->ucPrimaryChnl))
			continue;

		/* search the current potential channel list for the
		 * same group
		 */
		for (u4Idx = 0; u4Idx < u4Num; u4Idx++) {
			if ((prPotentialChnlList[u4Idx].ucOpClass ==
			     prPotentialChnlMap->aucOperatingClass[eBwMap]) &&
			    (prPotentialChnlList[u4Idx].ucPriChnlBitmap ==
			     prPotentialChnlMap->aucPriChnlMapIdx[eBwMap]))
				break;
		}

		if ((u4Idx == u4Num) && (u4Num < NAN_MAX_POTENTIAL_CHNL_LIST)) {
			u4Num++;

			prPotentialChnlList[u4Idx].ucOpClass =
				prPotentialChnlMap->aucOperatingClass[eBwMap];
			prPotentialChnlList[u4Idx].ucPriChnlBitmap =
				prPotentialChnlMap->aucPriChnlMapIdx[eBwMap];
		}

		if (u4Idx < u4Num)
			prPotentialChnlList[u4Idx].u2ChnlBitmap |=
				prPotentialChnlMap->au2ChnlMapIdx[eBwMap];
	}

	prCmdUpdatePontentialChnlList->u4Num = u4Num;

	rFixChnl = nanSchedGetFixedChnlInfo(prAdapter);
	if (rFixChnl.rChannel.u4PrimaryChnl != 0) {
		u4Num = 1;
		prCmdUpdatePontentialChnlList->u4Num = u4Num;
		prCmdUpdatePontentialChnlList->arChnlList[0].ucOpClass =
			rFixChnl.rChannel.u4OperatingClass;
		prCmdUpdatePontentialChnlList->arChnlList[0].ucPriChnlBitmap =
			0; /* Fixme */
		prCmdUpdatePontentialChnlList->arChnlList[0].u2ChnlBitmap = 0;
		nanRegGetChannelBitmap(
			prCmdUpdatePontentialChnlList->arChnlList[0].ucOpClass,
			rFixChnl.rChannel.u4PrimaryChnl,
			&prCmdUpdatePontentialChnlList->arChnlList[0]
				 .u2ChnlBitmap);
	}

#if 1
	for (u4Idx = 0; u4Idx < u4Num; u4Idx++) {
		DBGLOG(NAN, INFO,
		       "[%d] OpClass:%d, PriChnlBitmap:0x%x, ChnlBitmap:0x%x, Bw:%d\n",
		       u4Idx, prPotentialChnlList[u4Idx].ucOpClass,
		       prPotentialChnlList[u4Idx].ucPriChnlBitmap,
		       prPotentialChnlList[u4Idx].u2ChnlBitmap,
		       nanRegGetBw(prPotentialChnlList[u4Idx].ucOpClass));
	}
#endif

	prNanScheduler->u4NumOfPotentialChnlList =
		prCmdUpdatePontentialChnlList->u4Num;
	for (u4Idx = 0; u4Idx < prCmdUpdatePontentialChnlList->u4Num; u4Idx++) {
		prNanScheduler->arPotentialChnlList[u4Idx].ucOperatingClass =
			prCmdUpdatePontentialChnlList->arChnlList[u4Idx]
				.ucOpClass;
		prNanScheduler->arPotentialChnlList[u4Idx].u2ChannelBitmap =
			prCmdUpdatePontentialChnlList->arChnlList[u4Idx]
				.u2ChnlBitmap;
		prNanScheduler->arPotentialChnlList[u4Idx]
			.ucPrimaryChnlBitmap =
			prCmdUpdatePontentialChnlList->arChnlList[u4Idx]
				.ucPriChnlBitmap;
	}

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, nicCmdTimeoutCommon,
				      u4CmdBufferLen, (uint8_t *)prCmdBuffer,
				      NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

uint32_t
nanSchedCmdUpdateCRB(IN struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_SCHED_CMD_UPDATE_CRB_T *prCmdUpdateCRB = NULL;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (!prPeerSchRecord || prPeerSchRecord->fgActive == FALSE)
		return WLAN_STATUS_FAILURE;

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_SCHED_CMD_UPDATE_CRB_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(NAN, ERROR, "Memory allocation fail\n");
		return WLAN_STATUS_FAILURE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(
		NAN_CMD_UPDATE_CRB, sizeof(struct _NAN_SCHED_CMD_UPDATE_CRB_T),
		u4CmdBufferLen, prCmdBuffer);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);
	if (prTlvElement == NULL) {
		DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prCmdUpdateCRB =
		(struct _NAN_SCHED_CMD_UPDATE_CRB_T *)prTlvElement->aucbody;
	kalMemZero(prCmdUpdateCRB, sizeof(struct _NAN_SCHED_CMD_UPDATE_CRB_T));

	prCmdUpdateCRB->u4SchIdx = u4SchIdx;

	kalMemCopy(&prCmdUpdateCRB->rCommFawTimeline,
		   &prPeerSchRecord->rCommFawTimeline,
		   sizeof(prCmdUpdateCRB->rCommFawTimeline));

	if (prPeerSchRecord->fgUseDataPath) {
		prCmdUpdateCRB->fgUseDataPath = TRUE;
		prCmdUpdateCRB->rCommNdcCtrl = *prPeerSchRecord->prCommNdcCtrl;
	}

	if (prPeerSchRecord->fgUseRanging) {
		prCmdUpdateCRB->fgUseRanging = TRUE;
		kalMemCopy(&prCmdUpdateCRB->rCommRangingTimeline,
			   &prPeerSchRecord->rCommRangingTimeline,
			   sizeof(prCmdUpdateCRB->rCommRangingTimeline));
	}

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, nicCmdTimeoutCommon,
				      u4CmdBufferLen, (uint8_t *)prCmdBuffer,
				      NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

uint32_t
nanSchedCmdMapStaRecord(IN struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
			enum NAN_BSS_ROLE_INDEX eRoleIdx, uint8_t ucStaRecIdx,
			uint8_t ucNdpCxtId) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_SCHED_CMD_MAP_STA_REC_T *prCmdMapStaRec = NULL;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	prPeerSchRecord = nanSchedLookupPeerSchRecord(prAdapter, pucNmiAddr);
	if (!prPeerSchRecord)
		return WLAN_STATUS_FAILURE;

	prPeerSchRecord->aucStaRecIdx[ucNdpCxtId] = ucStaRecIdx;

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_SCHED_CMD_MAP_STA_REC_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(NAN, ERROR, "Memory allocation fail\n");
		return WLAN_STATUS_FAILURE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus =
		nicAddNewTlvElement(NAN_CMD_MAP_STA_RECORD,
				    sizeof(struct _NAN_SCHED_CMD_MAP_STA_REC_T),
				    u4CmdBufferLen, prCmdBuffer);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);
	if (prTlvElement == NULL) {
		DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prCmdMapStaRec =
		(struct _NAN_SCHED_CMD_MAP_STA_REC_T *)prTlvElement->aucbody;
	kalMemCopy(prCmdMapStaRec->aucNmiAddr, pucNmiAddr, MAC_ADDR_LEN);
	prCmdMapStaRec->eRoleIdx = eRoleIdx;
	prCmdMapStaRec->ucStaRecIdx = ucStaRecIdx;
	prCmdMapStaRec->ucNdpCxtId = ucNdpCxtId;

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, nicCmdTimeoutCommon,
				      u4CmdBufferLen, (uint8_t *)prCmdBuffer,
				      NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

uint32_t
nanSchedCmdUpdatePeerCapability(struct ADAPTER *prAdapter, uint32_t u4SchIdx) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_SCHED_CMD_UPDATE_PEER_CAPABILITY_T *prCmdUpdatePeerCap =
		NULL;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;
	uint8_t ucSupportedBands;
	uint32_t u4Idx;
	struct _NAN_DEVICE_CAPABILITY_T *prDevCapList;

	prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (!prPeerSchRecord || prPeerSchRecord->fgActive != TRUE ||
	    !prPeerSchRecord->prPeerSchDesc)
		return WLAN_STATUS_FAILURE;

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_SCHED_CMD_UPDATE_PEER_CAPABILITY_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);
	if (!prCmdBuffer) {
		DBGLOG(NAN, ERROR, "Memory allocation fail\n");
		return WLAN_STATUS_FAILURE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;
	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(
		NAN_CMD_UPDATE_PEER_CAPABILITY,
		sizeof(struct _NAN_SCHED_CMD_UPDATE_PEER_CAPABILITY_T),
		u4CmdBufferLen, prCmdBuffer);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);
	if (prTlvElement == NULL) {
		DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prCmdUpdatePeerCap = (struct _NAN_SCHED_CMD_UPDATE_PEER_CAPABILITY_T *)
				     prTlvElement->aucbody;

	ucSupportedBands = BIT(NAN_SUPPORTED_BAND_ID_2P4G);
	prDevCapList = prPeerSchRecord->prPeerSchDesc->arDevCapability;
	for (u4Idx = 0; u4Idx < (NAN_NUM_AVAIL_DB + 1);
		u4Idx++, prDevCapList++) {
		if (prDevCapList->fgValid)
			ucSupportedBands |= prDevCapList->ucSupportedBand;
	}

	prCmdUpdatePeerCap->ucSupportedBands = ucSupportedBands;

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, nicCmdTimeoutCommon,
				      u4CmdBufferLen, (uint8_t *)prCmdBuffer,
				      NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

uint32_t
nanSchedCmdManagePeerSchRecord(IN struct ADAPTER *prAdapter, uint32_t u4SchIdx,
			       unsigned char fgActivate) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_SCHED_CMD_MANAGE_PEER_SCH_REC_T *prCmdManagePeerSchRec =
		NULL;
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRecord;

	prPeerSchRecord = nanSchedGetPeerSchRecord(prAdapter, u4SchIdx);
	if (!prPeerSchRecord || prPeerSchRecord->fgActive != fgActivate)
		return WLAN_STATUS_FAILURE;

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_SCHED_CMD_MANAGE_PEER_SCH_REC_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(NAN, ERROR, "Memory allocation fail\n");
		return WLAN_STATUS_FAILURE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(
		NAN_CMD_MANAGE_PEER_SCH_RECORD,
		sizeof(struct _NAN_SCHED_CMD_MANAGE_PEER_SCH_REC_T),
		u4CmdBufferLen, prCmdBuffer);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);
	if (prTlvElement == NULL) {
		DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prCmdManagePeerSchRec = (struct _NAN_SCHED_CMD_MANAGE_PEER_SCH_REC_T *)
					prTlvElement->aucbody;
	kalMemZero(prCmdManagePeerSchRec,
		   sizeof(struct _NAN_SCHED_CMD_MANAGE_PEER_SCH_REC_T));
	prCmdManagePeerSchRec->u4SchIdx = u4SchIdx;
	prCmdManagePeerSchRec->fgActivate = fgActivate;
	kalMemCopy(prCmdManagePeerSchRec->aucNmiAddr,
		   prPeerSchRecord->aucNmiAddr, MAC_ADDR_LEN);

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, nicCmdTimeoutCommon,
				      u4CmdBufferLen, (uint8_t *)prCmdBuffer,
				      NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

uint32_t
nanSchedCmdUpdateAvailabilityDb(struct ADAPTER *prAdapter,
				unsigned char fgChkCondAvailability) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_T *prCmdUpdateAvailability =
		NULL;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;

	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(NAN, ERROR, "Memory allocation fail\n");
		return WLAN_STATUS_FAILURE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;
	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(
		NAN_CMD_UPDATE_AVAILABILITY,
		sizeof(struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_T),
		u4CmdBufferLen, prCmdBuffer);

	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);

	if (prTlvElement == NULL) {
		DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prCmdUpdateAvailability =
		(struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_T *)
			prTlvElement->aucbody;

	prCmdUpdateAvailability->ucMapId = prNanTimelineMgmt->ucMapId;
	prCmdUpdateAvailability->fgChkCondAvailability = fgChkCondAvailability;

	if (!fgChkCondAvailability) {
		kalMemCopy((uint8_t *)prCmdUpdateAvailability->arChnlList,
			   (uint8_t *)prNanTimelineMgmt->arChnlList,
			   sizeof(prCmdUpdateAvailability->arChnlList));
	} else {
		kalMemCopy((uint8_t *)prCmdUpdateAvailability->arChnlList,
			   (uint8_t *)prNanTimelineMgmt->arCondChnlList,
			   sizeof(prCmdUpdateAvailability->arChnlList));
	}

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, nicCmdTimeoutCommon,
				      u4CmdBufferLen, (uint8_t *)prCmdBuffer,
				      NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

uint32_t
nanSchedCmdUpdateAvailabilityCtrl(struct ADAPTER *prAdapter) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_CTRL_T *prCmdUpdateAvailCtrl =
		NULL;
	struct _NAN_SCHEDULER_T *prScheduler;

	prScheduler = nanGetScheduler(prAdapter);

	u4CmdBufferLen =
		sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
		sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
		sizeof(struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_CTRL_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(NAN, ERROR, "Memory allocation fail\n");
		return WLAN_STATUS_FAILURE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;
	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(
		NAN_CMD_UPDATE_AVAILABILITY_CTRL,
		sizeof(struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_CTRL_T),
		u4CmdBufferLen, prCmdBuffer);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);
	if (prTlvElement == NULL) {
		DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prCmdUpdateAvailCtrl =
		(struct _NAN_SCHED_CMD_UPDATE_AVAILABILITY_CTRL_T *)
			prTlvElement->aucbody;

	prCmdUpdateAvailCtrl->u2AvailAttrControlField =
		prScheduler->u2NanCurrAvailAttrControlField;
	prCmdUpdateAvailCtrl->ucAvailSeqID = prScheduler->ucNanAvailAttrSeqId;
	DBGLOG(NAN, INFO, "AvailAttr SeqID:%d, Ctrl:%x\n",
	       prCmdUpdateAvailCtrl->ucAvailSeqID,
	       prCmdUpdateAvailCtrl->u2AvailAttrControlField);

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, nicCmdTimeoutCommon,
				      u4CmdBufferLen, (uint8_t *)prCmdBuffer,
				      NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

uint32_t
nanSchedCmdUpdateAvailability(IN struct ADAPTER *prAdapter) {
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_TIMELINE_MGMT_T *prNanTimelineMgmt;
	struct _NAN_SCHEDULER_T *prScheduler;
	unsigned char fgChange = FALSE;

	prScheduler = nanGetScheduler(prAdapter);
	prNanTimelineMgmt = nanGetTimelineMgmt(prAdapter);

	do {
		if (prScheduler->u2NanAvailAttrControlField &
		    NAN_AVAIL_CTRL_CHECK_FOR_CHANGED) {
			prScheduler->ucNanAvailAttrSeqId++;
			fgChange = TRUE;
		}

		if (prScheduler->u2NanCurrAvailAttrControlField !=
		    prScheduler->u2NanAvailAttrControlField) {
			prScheduler->u2NanCurrAvailAttrControlField =
				prScheduler->u2NanAvailAttrControlField;
			fgChange = TRUE;
		}
		prScheduler->u2NanAvailAttrControlField = 0;

		if (fgChange) {
			cnmTimerStopTimer(
				prAdapter,
				&(prScheduler->rAvailAttrCtrlResetTimer));
			cnmTimerStartTimer(
				prAdapter,
				&(prScheduler->rAvailAttrCtrlResetTimer),
				CFG_NAN_AVAIL_CTRL_RESET_TIMEOUT);

			nanSchedCmdUpdateAvailabilityCtrl(prAdapter);
		}

		rStatus = nanSchedCmdUpdateAvailabilityDb(prAdapter, FALSE);
		if (rStatus != WLAN_STATUS_SUCCESS)
			break;

		if (prNanTimelineMgmt->fgChkCondAvailability)
			rStatus = nanSchedCmdUpdateAvailabilityDb(prAdapter,
								  TRUE);
	} while (FALSE);

	return rStatus;
}

uint32_t
nanSchedCmdUpdatePhySettigns(IN struct ADAPTER *prAdapter,
			     struct _NAN_PHY_SETTING_T *pr2P4GPhySettings,
			     struct _NAN_PHY_SETTING_T *pr5GPhySettings) {
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_SCHED_CMD_UPDATE_PHY_PARAM_T *prNanPhyParam;

	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_SCHED_CMD_UPDATE_PHY_PARAM_T);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(NAN, ERROR, "Memory allocation fail\n");
		return WLAN_STATUS_FAILURE;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;
	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(
		NAN_CMD_UPDATE_PHY_SETTING,
		sizeof(struct _NAN_SCHED_CMD_UPDATE_PHY_PARAM_T),
		u4CmdBufferLen, prCmdBuffer);

	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(NAN, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);

	if (prTlvElement == NULL) {
		DBGLOG(NAN, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return WLAN_STATUS_FAILURE;
	}

	prNanPhyParam = (struct _NAN_SCHED_CMD_UPDATE_PHY_PARAM_T *)
				prTlvElement->aucbody;
	prNanPhyParam->r2P4GPhySettings = *pr2P4GPhySettings;
	prNanPhyParam->r5GPhySettings = *pr5GPhySettings;

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, nicCmdTimeoutCommon,
				      u4CmdBufferLen, (uint8_t *)prCmdBuffer,
				      NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);

	return rStatus;
}

uint32_t
nanSchedEventScheduleConfig(struct ADAPTER *prAdapter, uint32_t u4SubEvent,
			    uint8_t *pucBuf) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_SCHED_EVENT_SCHEDULE_CONFIG_T *prEventScheduleConfig;
	struct WIFI_VAR *prWifiVar = &prAdapter->rWifiVar;

	prEventScheduleConfig =
		(struct _NAN_SCHED_EVENT_SCHEDULE_CONFIG_T *)pucBuf;

	nanSchedConfigAllowedBand(prAdapter, prEventScheduleConfig->fgEn2g,
				  prEventScheduleConfig->fgEn5gH,
				  prEventScheduleConfig->fgEn5gL);
	nanSchedConfigDefNdlNumSlots(prAdapter, prWifiVar->ucDftNdlQuotaVal);
	nanSchedConfigDefRangingNumSlots(prAdapter,
					 prWifiVar->ucDftRangQuotaVal);

	nanSchedCmdUpdatePotentialChnlList(prAdapter);

	return rRetStatus;
}

uint32_t
nanSchedEventNanAttr(struct ADAPTER *prAdapter, uint32_t u4SubEvent,
		     uint8_t *pucBuf) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_SCHED_EVENT_NAN_ATTR_T *prEventNanAttr;
	struct _NAN_ATTR_HDR_T *prAttrHdr;
	struct _NAN_NDL_INSTANCE_T *prNDL = NULL;
	struct _NAN_NDP_INSTANCE_T *prNDP = NULL;
	uint32_t u4Idx = 0;

	prEventNanAttr = (struct _NAN_SCHED_EVENT_NAN_ATTR_T *)pucBuf;
	prAttrHdr = (struct _NAN_ATTR_HDR_T *)prEventNanAttr->aucNanAttr;

	DBGLOG(NAN, INFO, "Nmi> %02x:%02x:%02x:%02x:%02x:%02x, SubEvent:%d\n",
		prEventNanAttr->aucNmiAddr[0], prEventNanAttr->aucNmiAddr[1],
		prEventNanAttr->aucNmiAddr[2], prEventNanAttr->aucNmiAddr[3],
		prEventNanAttr->aucNmiAddr[4], prEventNanAttr->aucNmiAddr[5],
		u4SubEvent);
#if 0
	nanUtilDump(prAdapter, "NAN Attribute",
		(PUINT_8)prAttrHdr, (prAttrHdr->u2Length + 3));
#endif

	switch (u4SubEvent) {
	case NAN_EVENT_ID_PEER_CAPABILITY:
		nanSchedPeerUpdateDevCapabilityAttr(prAdapter,
						    prEventNanAttr->aucNmiAddr,
						    prEventNanAttr->aucNanAttr);
		break;
	case NAN_EVENT_ID_PEER_AVAILABILITY:
		/* Skip update availability by event
		* if NDP negotiation is ongoing
		*/
		prNDL = nanDataUtilSearchNdlByMac(
			prAdapter, prEventNanAttr->aucNmiAddr);
		if (prNDL) {
			if (prNDL->prOperatingNDP)
				DBGLOG(NAN, INFO, "operating NDP %d\n",
				prNDL->prOperatingNDP->ucNDPID);

			if (prNDL->ucNDPNum) {
				for (u4Idx = 0;
					u4Idx < prNDL->ucNDPNum; u4Idx++) {
					prNDP = &(prNDL->arNDP[u4Idx]);
					DBGLOG(NAN, INFO,
						"NDP idx[%d] NDPID[%d] state[%d]\n",
						u4Idx, prNDP->ucNDPID,
						(prNDP
						->eCurrentNDPProtocolState));

					if ((prNDP->eCurrentNDPProtocolState
						!= NDP_IDLE) &&
						(prNDP->eCurrentNDPProtocolState
						!= NDP_NORMAL_TR)) {
						DBGLOG(NAN, INFO,
							"Skip due to peer under negotiation\n",
							u4Idx,
							prNDP->ucNDPID,
						(prNDP
						->eCurrentNDPProtocolState));
						return WLAN_STATUS_FAILURE;
					}
				}
			} else {
				DBGLOG(NAN, INFO,
					"No NDP found %d\n", prNDL->ucNDPNum);
			}
		} else {
			DBGLOG(NAN, INFO, "No NDL found\n");
		}

		nanSchedPeerUpdateAvailabilityAttr(prAdapter,
						   prEventNanAttr->aucNmiAddr,
						   prEventNanAttr->aucNanAttr);
		break;

	default:
		break;
	}

	return rRetStatus;
}

uint32_t
nanSchedEventDwInterval(struct ADAPTER *prAdapter, uint32_t u4SubEvent,
			uint8_t *pucBuf) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;
	struct _NAN_SCHED_EVENT_DW_INTERVAL_T *prEventDwInterval;
	struct _NAN_SCHEDULER_T *prScheduler;

	prEventDwInterval = (struct _NAN_SCHED_EVENT_DW_INTERVAL_T *)pucBuf;
	prScheduler = nanGetScheduler(prAdapter);
	prScheduler->ucCommitDwInterval = prEventDwInterval->ucDwInterval;

	return rRetStatus;
}

uint32_t
nanSchedulerEventDispatch(struct ADAPTER *prAdapter, uint32_t u4SubEvent,
			  uint8_t *pucBuf) {
	uint32_t rRetStatus = WLAN_STATUS_SUCCESS;

	DBGLOG(NAN, LOUD, "Evt:%d\n", u4SubEvent);

	switch (u4SubEvent) {
	case NAN_EVENT_ID_SCHEDULE_CONFIG:
		nanSchedEventScheduleConfig(prAdapter, u4SubEvent, pucBuf);
		break;

	case NAN_EVENT_ID_PEER_CAPABILITY:
	case NAN_EVENT_ID_PEER_AVAILABILITY:
		nanSchedEventNanAttr(prAdapter, u4SubEvent, pucBuf);
		break;
	case NAN_EVENT_ID_CRB_HANDSHAKE_TOKEN:
		break;
	case NAN_EVENT_DW_INTERVAL:
		nanSchedEventDwInterval(prAdapter, u4SubEvent, pucBuf);
		break;
	default:
		break;
	}

	return rRetStatus;
}

#define NAN_STATION_TEST_ADDRESS                                               \
	{ 0x22, 0x22, 0x22, 0x22, 0x22, 0x22 }

/* [0] ChnlRaw:0x247301, PriChnl:36
 * [Map], len:64
 * 0x02057668: 00 0e 00 00 00 0e 00 00 00 0e 00 00 00 0e 00 00
 * 0x02057678: 00 0e 00 00 00 0e 00 00 00 0e 00 00 00 0e 00 00
 * 0x02057688: 00 0e 00 00 00 0e 00 00 00 0e 00 00 00 0e 00 00
 * 0x02057698: 00 0e 00 00 00 0e 00 00 00 0e 00 00 00 0e 00 00
 *
 * [1] ChnlRaw:0xa17c01, PriChnl:161
 * [Map], len:64
 * 0x020576c4: 00 20 00 00 00 20 00 00 00 20 00 00 00 20 00 00
 * 0x020576d4: 00 20 00 00 00 20 00 00 00 20 00 00 00 20 00 00
 * 0x020576e4: 00 20 00 00 00 20 00 00 00 20 00 00 00 20 00 00
 * 0x020576f4: 00 20 00 00 00 20 00 00 00 20 00 00 00 20 00 00
 */
uint8_t g_aucPeerAvailabilityAttr[] = { 0x12, 0x1D, 0x0,  0x0,  0x1, 0x0, 0xb,
				       0x0,  0x1,  0x12, 0x58, 0x2, 0x1, 0x7,
				       0x11, 0x73, 0x1,  0x0,  0x0, 0xb, 0x0,
				       0x1,  0x12, 0x58, 0x3,  0x1, 0x1, 0x11,
				       0x7c, 0x8,  0x0,  0x0 };
uint8_t g_aucPeerAvailabilityAttr2[] = { 0x12, 0xc,  0x0,  0x1,  0x21,
					0x0,  0x7,  0x0,  0x1a, 0x0,
					0x11, 0x51, 0xff, 0x7,  0x0 };

/* commit Chnl:6
 * AvailMap, len:64
 * ffffffffc0c9158c: 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00
 * ffffffffc0c9159c: 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00
 * ffffffffc0c915ac: 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00
 * ffffffffc0c915bc: 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00
 * Commit Chnl:149
 * AvailMap, len:64
 * ffffffffc0c915d8: 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00
 * ffffffffc0c915e8: 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00
 * ffffffffc0c915f8: 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00
 * ffffffffc0c91608: 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00
 */
uint8_t g_aucPeerAvailabilityAttr3[] = {
	0x12, 0x2b, 0x0,  0x01, 0x11, 0x0,  0xb,  0x0,  0x1,  0x12, 0x18, 0x0,
	0x1,  0x1,  0x11, 0x51, 0x20, 0x0,  0x0,  0xb,  0x0,  0x1,  0x12, 0x18,
	0x2,  0x1,  0x1,  0x11, 0x7c, 0x01, 0x0,  0x0,  0xc,  0x0,  0x2,  0x12,
	0x58, 0x0,  0x4,  0x7f, 0xff, 0xff, 0x7f, 0x20, 0x02, 0x04
};

/* [0], Commit Chnl:6
 * AvailMap, len:64
 * ffffffffc0c915cc: 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00
 * ffffffffc0c915dc: 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00
 * ffffffffc0c915ec: 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00
 * ffffffffc0c915fc: 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00 00
 * [1], Commit Chnl:149
 * AvailMap, len:64
 * ffffffffc0c91618: 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00
 * ffffffffc0c91628: 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00
 * ffffffffc0c91638: 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00
 * ffffffffc0c91648: 00 01 00 00 00 01 00 00 00 01 00 00 00 01 00 00
 * [0], Cond Chnl:149
 * AvailMap, len:64
 * ffffffffc0c918c8: 02 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00
 * ffffffffc0c918d8: 02 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00
 * ffffffffc0c918e8: 02 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00
 * ffffffffc0c918f8: 02 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00
 */
uint8_t g_aucPeerAvailabilityAttr4[] = {
	0x12, 0x38, 0x00, 0x01, 0x11, 0x00, 0x0b, 0x00, 0x01, 0x12, 0x18, 0x00,
	0x01, 0x01, 0x11, 0x51, 0x20, 0x00, 0x00, 0x0b, 0x00, 0x01, 0x12, 0x18,
	0x02, 0x01, 0x01, 0x11, 0x7c, 0x01, 0x00, 0x00, 0x0b, 0x00, 0x04, 0x12,
	0x58, 0x00, 0x01, 0x01, 0x11, 0x7c, 0x01, 0x00, 0x00, 0x0c, 0x00, 0x02,
	0x12, 0x58, 0x00, 0x04, 0x7f, 0xff, 0xff, 0x7f, 0x20, 0x02, 0x04
};

uint8_t g_aucPeerAvailabilityAttr5[] = {
	0x12, 0x2f, 0x00, 0x01, 0x11, 0x00,

	0x0e, 0x00, 0x01, 0x12, 0x18, 0x00, 0x04, 0x01, 0x00, 0x00,
	0x00, 0x11, 0x7c, 0x01, 0x00, 0x00,

	0x1a, 0x00, 0x1a, 0x12, 0x18, 0x00, 0x04, 0xff, 0xfe, 0xff,
	0xff, 0x41, 0x7e, 0x03, 0x00, 0x00, 0x7f, 0x03, 0x00, 0x00,
	0x73, 0x0f, 0x00, 0x00, 0x51, 0xff, 0x07, 0x00
};

uint8_t g_aucRangSchEntryList1[] = { 0x1, 0x18, 0x0, 0x4, 0x1, 0x0, 0x0, 0x0 };

uint8_t g_aucPeerSelectedNdcAttr[] = { 0x13, 0xc, 0x0, 0x1,  0x2, 0x3, 0x4, 0x5,
				      0x6,  0x1, 0x1, 0x58, 0x3, 0x1, 0x1 };

/* ffffffffc0c918c8: 02 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00
 * ffffffffc0c918d8: 02 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00
 * ffffffffc0c918e8: 02 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00
 * ffffffffc0c918f8: 02 00 00 00 02 00 00 00 02 00 00 00 02 00 00 00
 */
uint8_t g_aucRangSchEntryList[] = { 0x1, 0x58, 0x0, 0x1, 0x1 };
uint8_t g_aucTestData[100];

#if 0
UINT_8 g_aucCase_5_3_3_DataReq_AvailAttr[] = {
	0x12, 0x20, 0x00, 0x00, 0x00, 0x00,
	0x0e, 0x00, 0x0c, 0x11, 0x18, 0x00,
	0x04, 0x00, 0x00, 0x00, 0x00, 0x11, 0x51,
	0x20, 0x00, 0x00, 0x0b, 0x00, 0x0a, 0x11,
	0x18, 0x00, 0x04, 0x7e, 0xfe, 0xff,
	0x7f, 0x10, 0x02};
#else
uint8_t g_aucCase_5_3_3_DataReq_AvailAttr[] = {
	0x12, 0x20, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x0c, 0x11, 0x18, 0x00,
	0x04, 0x7e, 0xfe, 0xff, 0x0f, 0x11, 0x51, 0x20, 0x00, 0x00, 0x0b, 0x00,
	0x0a, 0x11, 0x18, 0x00, 0x04, 0x7e, 0xfe, 0xff, 0x7f, 0x10, 0x02
};
#endif
uint8_t g_aucCase_5_3_3_DataReq_NdcAttr[] = {
	0x13, 0x0f, 0x00, 0x50, 0x6f, 0x9a, 0x01, 0x00, 0x00,
	0x01, 0x00, 0x18, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00
};

uint8_t g_aucCase_5_3_3_DataReq_ImmNdl[] = { 0x00, 0x18, 0x00, 0x04,
					    0x00, 0x00, 0x01, 0x00 };

uint8_t g_aucCase_5_3_1_Publish_AvailAttr[] = { 0x12, 0x0c, 0x00, 0x01, 0x20,
					       0x00, 0x07, 0x00, 0x1a, 0x00,
					       0x11, 0x51, 0xff, 0x07, 0x00 };

uint8_t g_aucCase_5_3_1_Publish_AvailAttr2[] = {
	0x12, 0x23, 0x00, 0x07, 0x00, 0x00, 0x0e, 0x00, 0x1a, 0x10,
	0x18, 0x00, 0x04, 0x00, 0x00, 0x80, 0xff, 0x11, 0x51, 0xff,
	0x07, 0x00, 0x0e, 0x00, 0x02, 0x10, 0x18, 0x00, 0x04, 0xfe,
	0xff, 0x7f, 0x00, 0x11, 0x51, 0x20, 0x00, 0x00
};

uint8_t g_aucCase_5_3_1_DataRsp_AvailAttr[] = {
	0x12, 0x23, 0x00, 0x08, 0xb0, 0x00, 0x0e, 0x00, 0x1a, 0x10,
	0x18, 0x00, 0x04, 0x00, 0x00, 0x80, 0xff, 0x11, 0x51, 0xff,
	0x07, 0x00, 0x0e, 0x00, 0x04, 0x10, 0x18, 0x00, 0x04, 0xfe,
	0xff, 0x7f, 0x00, 0x11, 0x51, 0x20, 0x00, 0x00
};

uint8_t g_aucCase_5_3_11_DataReq_Avail1Attr[] = {
	0x12, 0x31, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x0c, 0x11, 0x18,
	0x00, 0x04, 0x7e, 0x00, 0x00, 0x00, 0x11, 0x51, 0x20, 0x00, 0x00,
	0x0e, 0x00, 0x0c, 0x11, 0x18, 0x00, 0x04, 0x00, 0xfe, 0xff, 0x7f,
	0x11, 0x7d, 0x01, 0x00, 0x00, 0x0c, 0x00, 0x0a, 0x11, 0x18, 0x00,
	0x04, 0x7e, 0xfe, 0xff, 0x7f, 0x20, 0x02, 0x04
};

uint8_t g_aucCase_5_3_11_DataReq_Avail2Attr[] = {
	0x12, 0x31, 0x00, 0x00, 0x01, 0x00, 0x0e, 0x00, 0x0c, 0x11, 0x18,
	0x00, 0x04, 0x7e, 0x00, 0x00, 0x00, 0x11, 0x7d, 0x01, 0x00, 0x00,
	0x0e, 0x00, 0x0c, 0x11, 0x18, 0x00, 0x04, 0x00, 0xfe, 0xff, 0x7f,
	0x11, 0x51, 0x20, 0x00, 0x00, 0x0c, 0x00, 0x0a, 0x11, 0x18, 0x00,
	0x04, 0x7e, 0xfe, 0xff, 0x7f, 0x20, 0x02, 0x04
};

uint8_t g_aucCase_5_3_11_DataReq_NdcAttr[] = { 0x13, 0x0f, 0x00, 0x50, 0x6f,
					      0x9a, 0x01, 0x00, 0x00, 0x01,
					      0x00, 0x18, 0x00, 0x04, 0x00,
					      0x02, 0x00, 0x00 };

void
nanScheduleNegoTestFunc(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr,
			enum _ENUM_NAN_NEGO_TYPE_T eType,
			enum _ENUM_NAN_NEGO_ROLE_T eRole, void *pvToken) {
	uint8_t *pucBuf;
	uint32_t u4Length;
	uint32_t rRetStatus;
	uint32_t u4RejectCode;
	uint8_t aucNmiAddr[] = NAN_STATION_TEST_ADDRESS;
	uint8_t aucTestData[100];

	DBGLOG(NAN, INFO, "IN\n");

	if (pvToken == (void *)5) {
		nanSchedNegoAddNdcCrb(prAdapter, &g_r5gDwChnl, 16, 1,
				      ENUM_TIME_BITMAP_CTRL_PERIOD_512);
		nanSchedNegoGetSelectedNdcAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[NDC Attr]", pucBuf, u4Length);

		nanSchedNegoStop(prAdapter);
	} else if (pvToken == (void *)7) {
		nanSchedNegoGenLocalCrbProposal(prAdapter);

		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedNegoStop(prAdapter);
	} else if (pvToken == (void *)8) {
		nanSchedNegoAddQos(prAdapter, 10, 3);
		nanSchedNegoGenLocalCrbProposal(prAdapter);
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);

		nanSchedNegoStop(prAdapter);
	} else if (pvToken == (void *)9) {
		nanSchedNegoAddNdcCrb(prAdapter, &g_r2gDwChnl, 14, 1,
				      ENUM_TIME_BITMAP_CTRL_PERIOD_256);
		nanSchedNegoGenLocalCrbProposal(prAdapter);
		nanSchedNegoGetSelectedNdcAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Selected NDC]", pucBuf, u4Length);

		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedNegoStop(prAdapter);

	} else if (pvToken == (void *)10) {
		/* nanSchedNegoAddQos(prAdapter, 10, 3); */
		rRetStatus =
			nanSchedNegoChkRmtCrbProposal(prAdapter, &u4RejectCode);
		DBGLOG(NAN, INFO, "nanSchedNegoChkRmtCrbProposal: %x, %u\n",
		       rRetStatus, u4RejectCode);
		nanSchedNegoGetSelectedNdcAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[NDC Attr]", pucBuf, u4Length);
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedNegoStop(prAdapter);
	} else if (pvToken == (void *)11) {
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		nanSchedNegoStop(prAdapter);

	} else if (pvToken == (void *)14) {

		rRetStatus = nanSchedNegoGenLocalCrbProposal(prAdapter);
		DBGLOG(NAN, INFO, "nanSchedNegoGenLocalCrbProposal: %x\n",
		       rRetStatus);
		nanSchedNegoGetRangingScheduleList(prAdapter, &pucBuf,
						   &u4Length);
		nanUtilDump(prAdapter, "[Ranging]", pucBuf, u4Length);

		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);

		nanSchedNegoStop(prAdapter);

	} else if (pvToken == (void *)15) {
		rRetStatus =
			nanSchedNegoChkRmtCrbProposal(prAdapter, &u4RejectCode);
		DBGLOG(NAN, INFO, "nanSchedNegoChkRmtCrbProposal: %x, %u\n",
		       rRetStatus, u4RejectCode);
		nanSchedNegoGetRangingScheduleList(prAdapter, &pucBuf,
						   &u4Length);
		nanUtilDump(prAdapter, "[Ranging]", pucBuf, u4Length);
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedNegoStop(prAdapter);

	} else if (pvToken == (void *)18) {

		rRetStatus =
			nanSchedNegoChkRmtCrbProposal(prAdapter, &u4RejectCode);
		DBGLOG(NAN, INFO, "nanSchedNegoChkRmtCrbProposal: %x, %u\n",
		       rRetStatus, u4RejectCode);

		DBGLOG(NAN, INFO, "DUMP#5\n");
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);

		nanSchedNegoGetSelectedNdcAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[NDC Attr]", pucBuf, u4Length);

		nanSchedNegoGetImmuNdlScheduleList(prAdapter, &pucBuf,
						   &u4Length);
		nanUtilDump(prAdapter, "[Imm NDL Attr]", pucBuf, u4Length);

		nanSchedNegoStop(prAdapter);

	} else if (pvToken == (void *)19) {
		rRetStatus = nanSchedNegoGenLocalCrbProposal(prAdapter);
		DBGLOG(NAN, INFO, "nanSchedNegoGenLocalCrbProposal: %x\n",
		       rRetStatus);

		DBGLOG(NAN, INFO, "DUMP#2\n");
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedNegoStop(prAdapter);

	} else if (pvToken == (void *)20) {

		rRetStatus = nanSchedNegoGenLocalCrbProposal(prAdapter);
		DBGLOG(NAN, INFO, "nanSchedNegoGenLocalCrbProposal: %x\n",
		       rRetStatus);

		DBGLOG(NAN, INFO, "DUMP#4\n");
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);

		nanSchedNegoGetSelectedNdcAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[NDC Attr]", pucBuf, u4Length);

		nanSchedNegoStop(prAdapter);

	} else if (pvToken == (void *)23) {
		rRetStatus = nanSchedNegoGenLocalCrbProposal(prAdapter);
		DBGLOG(NAN, INFO, "nanSchedNegoGenLocalCrbProposal: %x\n",
		       rRetStatus);

		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);

		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr5,
			   sizeof(g_aucPeerAvailabilityAttr5));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		kalMemCopy(aucTestData, g_aucRangSchEntryList1,
			   sizeof(g_aucRangSchEntryList1));
		nanSchedPeerUpdateRangingScheduleList(
			prAdapter, aucNmiAddr,
			(struct _NAN_SCHEDULE_ENTRY_T *)aucTestData,
			sizeof(g_aucRangSchEntryList1));

		rRetStatus =
			nanSchedNegoChkRmtCrbProposal(prAdapter, &u4RejectCode);
		DBGLOG(NAN, INFO, "nanSchedNegoChkRmtCrbProposal: %x, %u\n",
		       rRetStatus, u4RejectCode);

		nanSchedNegoStop(prAdapter);
	} else if (pvToken == (void *)24) {
		rRetStatus =
			nanSchedNegoChkRmtCrbProposal(prAdapter, &u4RejectCode);
		DBGLOG(NAN, INFO, "nanSchedNegoChkRmtCrbProposal: %x, %u\n",
		       rRetStatus, u4RejectCode);

		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[AVAIL ATTR]", pucBuf, u4Length);

		nanSchedNegoGetSelectedNdcAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[NDC ATTR]", pucBuf, u4Length);

		nanSchedNegoStop(prAdapter);
	}
}

uint32_t
nanSchedSwDbg4(struct ADAPTER *prAdapter, uint32_t u4Data) /* 0x7426000d */
{
	uint32_t u4Ret = 0;
	uint8_t *pucBuf;
	uint32_t u4Length;
	uint8_t aucNmiAddr[] = NAN_STATION_TEST_ADDRESS;
	uint32_t au4Map[NAN_TOTAL_DW];
	uint32_t u4Idx;
	uint8_t aucTestData[100];
	uint32_t rRetStatus;
	struct _NAN_SCHEDULER_T *prNanScheduler;
	uint32_t u4TestDataLen;
	union _NAN_BAND_CHNL_CTRL rChnlInfo = {
		.rChannel.u4Type = NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL,
		.rChannel.u4OperatingClass = NAN_5G_LOW_DISC_CH_OP_CLASS,
		.rChannel.u4PrimaryChnl = 36,
		.rChannel.u4AuxCenterChnl = 0
	};
	uint8_t aucBitmap[4];

	prNanScheduler = nanGetScheduler(prAdapter);

	if (prNanScheduler->fgInit == FALSE)
		nanSchedInit(prAdapter);

	switch (u4Data) {
	case 0:
#if 0
		nanSchedConfigAllowedBand(prAdapter, TRUE, TRUE, TRUE);
#else
		nanSchedConfigAllowedBand(prAdapter, TRUE, FALSE, FALSE);
#endif
		nanSchedConfigDefNdlNumSlots(prAdapter, 3);
		nanSchedConfigDefRangingNumSlots(prAdapter, 1);
		break;

	case 1:
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
		break;

	case 2:
		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);
		break;

	case 3:
		kalMemZero(au4Map, sizeof(au4Map));
		for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++) {
			au4Map[u4Idx] |= (BIT(NAN_5G_DW_INDEX + 1));
			au4Map[u4Idx] |= (BIT(NAN_5G_DW_INDEX + 2));
			au4Map[u4Idx] |= (BIT(NAN_5G_DW_INDEX + 3));
			au4Map[u4Idx] |= (BIT(NAN_5G_DW_INDEX + 4));
		}

		nanUtilDump(prAdapter, "[Map]", (uint8_t *)au4Map,
			    sizeof(au4Map));
		nanParserGenTimeBitmapField(prAdapter, au4Map, g_aucNanIEBuffer,
					    &u4Length);
		nanUtilDump(prAdapter, "[TimeBitmap]", g_aucNanIEBuffer,
			    u4Length);
		break;

	case 4:
		kalMemZero(au4Map, sizeof(au4Map));
		for (u4Idx = 0; u4Idx < NAN_TOTAL_DW; u4Idx++)
			au4Map[u4Idx] |= (BIT(NAN_5G_DW_INDEX + 5));

		nanUtilDump(prAdapter, "[Map]", (uint8_t *)au4Map,
			    sizeof(au4Map));
		nanParserGenTimeBitmapField(prAdapter, au4Map, g_aucNanIEBuffer,
					    &u4Length);
		nanUtilDump(prAdapter, "[TimeBitmap]", g_aucNanIEBuffer,
			    u4Length);
		break;

	case 5:
		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr,
			   sizeof(g_aucPeerAvailabilityAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_INITIATOR,
				  nanScheduleNegoTestFunc, (void *)5);
		break;

	case 6:
#if 1
		rChnlInfo.rChannel.u4OperatingClass =
			NAN_5G_LOW_DISC_CH_OP_CLASS;
		rChnlInfo.rChannel.u4PrimaryChnl = 36;
		rRetStatus = nanSchedAddCrbToChnlList(
			prAdapter, &rChnlInfo, 9, 2,
			ENUM_TIME_BITMAP_CTRL_PERIOD_512, TRUE, NULL);
#endif

#if 1
		rChnlInfo.rChannel.u4OperatingClass =
			NAN_5G_HIGH_DISC_CH_OP_CLASS;
		rChnlInfo.rChannel.u4PrimaryChnl = 161;
		rRetStatus = nanSchedAddCrbToChnlList(
			prAdapter, &rChnlInfo, 13, 1,
			ENUM_TIME_BITMAP_CTRL_PERIOD_512, TRUE, NULL);
#endif
		break;

	case 7:
		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr,
			   sizeof(g_aucPeerAvailabilityAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_INITIATOR,
				  nanScheduleNegoTestFunc, (void *)7);
		break;

	case 8:
		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr,
			   sizeof(g_aucPeerAvailabilityAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_INITIATOR,
				  nanScheduleNegoTestFunc, (void *)8);

		break;

	case 9:
		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr,
			   sizeof(g_aucPeerAvailabilityAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_INITIATOR,
				  nanScheduleNegoTestFunc, (void *)9);
		break;

	case 10:
		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr,
			   sizeof(g_aucPeerAvailabilityAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_RESPONDER,
				  nanScheduleNegoTestFunc, (void *)10);
		break;

	case 11:
		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr2,
			   sizeof(g_aucPeerAvailabilityAttr2));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_RESPONDER,
				  nanScheduleNegoTestFunc, (void *)11);

		break;

	case 12:
		break;

	case 13:
		prNanScheduler->fgInit = FALSE;
		break;

	case 14:
		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr3,
			   sizeof(g_aucPeerAvailabilityAttr3));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		nanSchedNegoStart(prAdapter, aucNmiAddr, ENUM_NAN_NEGO_RANGING,
				  ENUM_NAN_NEGO_ROLE_INITIATOR,
				  nanScheduleNegoTestFunc, (void *)14);
		break;

	case 15:
		kalMemCopy(aucTestData, g_aucRangSchEntryList,
			   sizeof(g_aucRangSchEntryList));
		u4TestDataLen = sizeof(g_aucRangSchEntryList);
		nanSchedPeerUpdateRangingScheduleList(
			prAdapter, aucNmiAddr,
			(struct _NAN_SCHEDULE_ENTRY_T *)aucTestData,
			u4TestDataLen);

		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr4,
			   sizeof(g_aucPeerAvailabilityAttr4));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
		nanSchedNegoStart(prAdapter, aucNmiAddr, ENUM_NAN_NEGO_RANGING,
				  ENUM_NAN_NEGO_ROLE_RESPONDER,
				  nanScheduleNegoTestFunc, (void *)15);

		break;

	case 16:
		nanSchedInit(prAdapter);
		break;

	case 17:
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);
		break;

	case 18:
		DBGLOG(NAN, INFO, "DUMP#1\n");
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		kalMemCopy(aucTestData, g_aucCase_5_3_3_DataReq_AvailAttr,
			   sizeof(g_aucCase_5_3_3_DataReq_AvailAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);

		DBGLOG(NAN, INFO, "DUMP#2\n");
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		kalMemCopy(aucTestData, g_aucCase_5_3_3_DataReq_NdcAttr,
			   sizeof(g_aucCase_5_3_3_DataReq_NdcAttr));
		nanSchedPeerUpdateNdcAttr(prAdapter, aucNmiAddr, aucTestData);

#if 1
		nanSchedPeerUpdateImmuNdlScheduleList(
			prAdapter, aucNmiAddr,
			(struct _NAN_SCHEDULE_ENTRY_T *)
				g_aucCase_5_3_3_DataReq_ImmNdl,
			sizeof(g_aucCase_5_3_3_DataReq_ImmNdl));
#endif

		DBGLOG(NAN, INFO, "DUMP#3\n");
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		DBGLOG(NAN, INFO, "DUMP#4\n");
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_RESPONDER,
				  nanScheduleNegoTestFunc, (void *)18);
		break;

	case 19:
		DBGLOG(NAN, INFO, "DUMP#1\n");
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_INITIATOR,
				  nanScheduleNegoTestFunc, (void *)19);

		break;

	case 20:
		DBGLOG(NAN, INFO, "DUMP#1\n");
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		kalMemCopy(aucTestData, g_aucCase_5_3_1_Publish_AvailAttr,
			   sizeof(g_aucCase_5_3_1_Publish_AvailAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);

		DBGLOG(NAN, INFO, "DUMP#2\n");
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		DBGLOG(NAN, INFO, "DUMP#3\n");
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_INITIATOR,
				  nanScheduleNegoTestFunc, (void *)20);
		break;

	case 21:
		DBGLOG(NAN, INFO, "DUMP#1\n");
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		kalMemCopy(aucTestData, g_aucCase_5_3_1_DataRsp_AvailAttr,
			   sizeof(g_aucCase_5_3_1_DataRsp_AvailAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);

		DBGLOG(NAN, INFO, "DUMP#2\n");
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);
		break;

	case 22:
		nanSchedDbgDumpTimelineDb(prAdapter, __func__, __LINE__);
		nanSchedGetAvailabilityAttr(prAdapter, &pucBuf, &u4Length);
		nanUtilDump(prAdapter, "[Availability Attr]", pucBuf, u4Length);
		break;

	case 23:
		kalMemCopy(aucTestData, g_aucPeerAvailabilityAttr,
			   sizeof(g_aucPeerAvailabilityAttr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		nanSchedNegoStart(prAdapter, aucNmiAddr, ENUM_NAN_NEGO_RANGING,
				  ENUM_NAN_NEGO_ROLE_INITIATOR,
				  nanScheduleNegoTestFunc, (void *)23);
		break;

	case 24:
		kalMemCopy(aucTestData, g_aucCase_5_3_11_DataReq_Avail1Attr,
			   sizeof(g_aucCase_5_3_11_DataReq_Avail1Attr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);

		kalMemCopy(aucTestData, g_aucCase_5_3_11_DataReq_Avail2Attr,
			   sizeof(g_aucCase_5_3_11_DataReq_Avail2Attr));
		nanSchedPeerUpdateAvailabilityAttr(prAdapter, aucNmiAddr,
						   aucTestData);
		nanSchedDbgDumpPeerAvailability(prAdapter, aucNmiAddr);

		kalMemCopy(aucTestData, g_aucCase_5_3_11_DataReq_NdcAttr,
			   sizeof(g_aucCase_5_3_11_DataReq_NdcAttr));
		nanSchedPeerUpdateNdcAttr(prAdapter, aucNmiAddr, aucTestData);

		nanSchedNegoStart(prAdapter, aucNmiAddr,
				  ENUM_NAN_NEGO_DATA_LINK,
				  ENUM_NAN_NEGO_ROLE_RESPONDER,
				  nanScheduleNegoTestFunc, (void *)24);
		break;

	case 25:
		halPrintHifDbgInfo(prAdapter);
		break;

	case 26:
		aucBitmap[0] = 0xaa;
		aucBitmap[1] = aucBitmap[2] = aucBitmap[3] = 0;
		kalMemZero(aucTestData, sizeof(aucTestData));
		nanParserInterpretTimeBitmapField(
		    prAdapter, 0x0008, 4, aucBitmap, (uint32_t *)aucTestData);
		nanUtilDump(prAdapter, "time map", aucTestData, 64);
		break;

	default:
		break;
	}

	return u4Ret;
}

enum ENUM_BAND
nanSchedGetSchRecBandByMac(struct ADAPTER *prAdapter, uint8_t *pucNmiAddr) {
	struct _NAN_PEER_SCHEDULE_RECORD_T *prPeerSchRec;

	prPeerSchRec = nanSchedLookupPeerSchRecord(prAdapter, pucNmiAddr);
	if (prPeerSchRec)
		return prPeerSchRec->eBand;
	else
		return BAND_NULL;
}
