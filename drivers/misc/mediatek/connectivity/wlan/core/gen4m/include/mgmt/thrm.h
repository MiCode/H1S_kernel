/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _THRM_H
#define _THRM_H

#if (CFG_SUPPORT_POWER_THROTTLING == 1)

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
enum ENUM_THERMAL_PROTECT_TYPE {
	THERMAL_PROTECT_TYPE_NTX_CTRL = 0,
	THERMAL_PROTECT_TYPE_DUTY_CTRL,
	THERMAL_PROTECT_TYPE_RADIO_CTRL,
	THERMAL_PROTECT_TYPE_NUM
};

enum ENUM_THERMAL_PROTECT_TRIG_TYPE {
	THERMAL_PROTECT_LOW_TRIG = 0,
	THERMAL_PROTECT_HIGH_TRIG,
	THERMAL_PROTECT_TRIG_TYPE_NUM
};

enum ENUM_THERMAL_PROTECT_ACT_TYPE {
	THERMAL_PROTECT_ACT_TYPE_TRIG = 0,
	THERMAL_PROTECT_ACT_TYPE_RESTORE,
	THERMAL_PROTECT_ACT_TYPE_NUM
};

enum ENUM_THERMAL_PROTECT_EVENT_CATEGORY {
	THERMAL_PROTECT_EVENT_REASON_NOTIFY = 0x0,
	TXPOWER_EVENT_THERMAL_PROT_SHOW_INFO = 0x1,
	THERMAL_PROTECT_EVENT_DUTY_NOTIFY = 0x2,
	THERMAL_PROTECT_EVENT_RADIO_NOTIFY = 0x3,
	THERMAL_PROTECT_EVENT_MECH_INFO = 0x4,
	THERMAL_PROTECT_EVENT_DUTY_INFO = 0x5,
	THERMAL_PROTECT_EVENT_NUM
};

enum ENUM_THERMAL_PROTECT_MODE {
	THERMAL_PROTECT_MODE_DISABLED = 0,
	THERMAL_PROTECT_MODE_PASSIVE,
	THERMAL_PROTECT_MODE_AGGRESSIVE,
	THERMAL_PROTECT_MODE_FORCE_DUTY_50,
	THERMAL_PROTECT_MODE_NUM
};

enum ENUM_THERMAL_PROTECT_ACTION_CATEGORY {
	THERMAL_PROTECT_PARAMETER_CTRL = 0x0,
	THERMAL_PROTECT_BASIC_INFO = 0x1,
	THERMAL_PROTECT_ENABLE = 0x2,
	THERMAL_PROTECT_DISABLE = 0x3,
	THERMAL_PROTECT_DUTY_CONFIG = 0x4,
	THERMAL_PROTECT_MECH_INFO = 0x5,
	THERMAL_PROTECT_DUTY_INFO = 0x6,
	THERMAL_PROTECT_STATE_ACT = 0x7,
	THERMAL_PROTECT_ACTION_NUM
};

struct EXT_EVENT_THERMAL_PROTECT_DUTY_NOTIFY {
	uint8_t u1SubEventId;
	uint8_t u1BandIdx;
	uint8_t u1LevelIdx;
	uint8_t u1DutyPercent;
	int32_t i4Temp;
	enum ENUM_THERMAL_PROTECT_ACT_TYPE eProtectActType;
	uint8_t u1Reserved[3];
};

struct EXT_EVENT_THERMAL_PROTECT_RADIO_NOTIFY {
	uint8_t u1SubEventId;
	uint8_t u1BandIdx;
	uint8_t u1LevelIdx;
	uint8_t u1Reserved;
	int32_t i4Temp;
	enum ENUM_THERMAL_PROTECT_ACT_TYPE eProtectActType;
	uint8_t u1Reserved2[3];
};

struct EXT_EVENT_THERMAL_PROTECT_DUTY_INFO {
	uint8_t u1SubEventId;
	uint8_t u1BandIdx;
	uint8_t u1Duty0;
	uint8_t u1Duty1;
	uint8_t u1Duty2;
	uint8_t u1Duty3;
	uint8_t u1Reserved[2];
};

struct EXT_EVENT_THERMAL_PROTECT_MECH_INFO {
	uint8_t u1SubEventId;
	uint8_t u1BandIdx;
	uint8_t u1Reserved[2];
	uint8_t u1ProtectionType[THERMAL_PROTECT_TYPE_NUM];
	uint8_t u1Reserved2;
	uint8_t u1TriggerType[THERMAL_PROTECT_TYPE_NUM];
	uint8_t u1Reserved3;
	int32_t i4TriggerTemp[THERMAL_PROTECT_TYPE_NUM];
	int32_t i4RestoreTemp[THERMAL_PROTECT_TYPE_NUM];
	uint16_t u2RecheckTime[THERMAL_PROTECT_TYPE_NUM];
	uint8_t u1Reserved4[2];
	uint8_t u1State[THERMAL_PROTECT_TYPE_NUM];
	uint8_t u1Reserved6;
	bool fgEnable[THERMAL_PROTECT_TYPE_NUM];
	uint8_t u1Reserved7;
};

struct EXT_CMD_THERMAL_PROTECT_ENABLE {
	uint8_t sub_cmd_id;
	uint8_t band_idx;
	uint8_t protection_type;
	uint8_t trigger_type;
	int32_t trigger_temp;
	int32_t restore_temp;
	uint16_t recheck_time;
	uint16_t reserved[2];
};

struct EXT_CMD_THERMAL_PROTECT_DISABLE {
	uint8_t sub_cmd_id;
	uint8_t band_idx;
	uint8_t protection_type;
	uint8_t trigger_type;
};

struct EXT_CMD_THERMAL_PROTECT_DUTY_CFG {
	uint8_t sub_cmd_id;
	uint8_t band_idx;
	uint8_t level_idx;
	uint8_t duty;
};

struct EXT_CMD_THERMAL_PROTECT_INFO {
	uint8_t sub_cmd_id;
	uint8_t band_idx;
	uint8_t reserved[2];
};

struct EXT_CMD_THERMAL_PROTECT_DUTY_INFO {
	uint8_t sub_cmd_id;
	uint8_t band_idx;
	uint8_t reserved[2];
};

struct EXT_CMD_THERMAL_PROTECT_STATE_ACT {
	uint8_t sub_cmd_id;
	uint8_t band_idx;
	uint8_t protect_type;
	uint8_t trig_type;
	uint8_t state;
	uint8_t reserved[3];
};

struct THRM_PROT_CFG_CONTEXT {
	uint8_t ucLevel;
	uint8_t ucMode;
	int32_t i4TrigTemp;
	int32_t i4RestoreTemp;
};

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/
uint32_t thrmProtTempConfig(IN struct ADAPTER *prAdapter,
	IN int32_t i4TrigTemp, IN int32_t i4RestoreTemp);
void thrmProtEventHandler(IN struct ADAPTER *prAdapter, IN uint8_t *prEventBuf);
int thrmProtLvHandler(IN struct ADAPTER *prAdapter, IN uint8_t ucLevel);
void thrmInit(IN struct ADAPTER *prAdapter);

#endif
#endif /* _THRM_H */
