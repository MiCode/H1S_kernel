/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2021 MediaTek Inc.
 *
 */

#if (CFG_SUPPORT_POWER_THROTTLING == 1)

/*******************************************************************************
 *            C O M P I L E R	 F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *            E X T E R N A L	R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"
#include "thrm.h"

/*******************************************************************************
 *            C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *            F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

#define THRM_PROT_CHECKTIME_PASSIVE 30
#define THRM_PROT_CHECKTIME_AGGRESIVE 10
#define THRM_PROT_DUTY_FORCE_50 50
#define THRM_PROT_DUTY_NORMAL 100

/*******************************************************************************
 *            P R I V A T E  F U N C T I O N S
 *******************************************************************************
 */

uint32_t thrmProtEnable(IN struct ADAPTER *prAdapter, IN uint8_t ucBand,
			IN uint8_t ucProtType, IN uint8_t ucTrigType,
			IN int32_t i4TrigTemp, IN int32_t i4RestoreTemp,
			IN uint32_t u4CheckTime)
{
	struct EXT_CMD_THERMAL_PROTECT_ENABLE *ext_cmd_buf;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	ext_cmd_buf =  cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
		sizeof(struct EXT_CMD_THERMAL_PROTECT_ENABLE));

	if (!ext_cmd_buf)
		return WLAN_STATUS_FAILURE;

	ext_cmd_buf->band_idx = ucBand;
	ext_cmd_buf->protection_type = ucProtType;
	ext_cmd_buf->trigger_type = ucTrigType;
	ext_cmd_buf->trigger_temp = i4TrigTemp;
	ext_cmd_buf->restore_temp = i4RestoreTemp;
	ext_cmd_buf->recheck_time = u4CheckTime;
	ext_cmd_buf->sub_cmd_id = THERMAL_PROTECT_ENABLE;

	rStatus = wlanSendSetQueryExtCmd(prAdapter,
				CMD_ID_LAYER_0_EXT_MAGIC_NUM,
				EXT_CMD_ID_THERMAL_PROTECT,
				TRUE,
				FALSE,
				FALSE,
				NULL,
				NULL,
				sizeof(struct EXT_CMD_THERMAL_PROTECT_ENABLE),
				(uint8_t *) ext_cmd_buf, NULL,
				0);

	cnmMemFree(prAdapter, ext_cmd_buf);

	return rStatus;
}

uint32_t thrmProtDisable(IN struct ADAPTER *prAdapter, IN uint8_t ucBand,
		IN uint8_t ucProtType, IN uint8_t ucTrigType)
{
	struct EXT_CMD_THERMAL_PROTECT_DISABLE *ext_cmd_buf;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	ext_cmd_buf = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
		sizeof(struct EXT_CMD_THERMAL_PROTECT_DISABLE));

	if (!ext_cmd_buf)
		return WLAN_STATUS_FAILURE;

	ext_cmd_buf->band_idx = ucBand;
	ext_cmd_buf->protection_type = ucProtType;
	ext_cmd_buf->trigger_type = ucTrigType;
	ext_cmd_buf->sub_cmd_id = THERMAL_PROTECT_DISABLE;

	rStatus = wlanSendSetQueryExtCmd(prAdapter,
				CMD_ID_LAYER_0_EXT_MAGIC_NUM,
				EXT_CMD_ID_THERMAL_PROTECT,
				TRUE,
				FALSE,
				FALSE,
				NULL,
				NULL,
				sizeof(struct EXT_CMD_THERMAL_PROTECT_DISABLE),
				(uint8_t *) ext_cmd_buf, NULL,
				0);
	cnmMemFree(prAdapter, ext_cmd_buf);

	return rStatus;
}

uint32_t thrmProtDutyCfg(IN struct ADAPTER *prAdapter, IN uint8_t ucBand,
	IN uint8_t ucLevel, IN uint8_t ucDuty)
{
	struct EXT_CMD_THERMAL_PROTECT_DUTY_CFG *ext_cmd_buf;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	ext_cmd_buf = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
		sizeof(struct EXT_CMD_THERMAL_PROTECT_DUTY_CFG));

	if (!ext_cmd_buf)
		return WLAN_STATUS_FAILURE;

	ext_cmd_buf->band_idx = ucBand;
	ext_cmd_buf->level_idx = ucLevel;
	ext_cmd_buf->duty = ucDuty;
	ext_cmd_buf->sub_cmd_id = THERMAL_PROTECT_DUTY_CONFIG;

	rStatus = wlanSendSetQueryExtCmd(prAdapter,
				CMD_ID_LAYER_0_EXT_MAGIC_NUM,
				EXT_CMD_ID_THERMAL_PROTECT,
				TRUE,
				FALSE,
				FALSE,
				NULL,
				NULL,
				sizeof(struct EXT_CMD_THERMAL_PROTECT_DUTY_CFG),
				(uint8_t *) ext_cmd_buf, NULL,
				0);

	cnmMemFree(prAdapter, ext_cmd_buf);

	return rStatus;
}

uint32_t thrmProtStateAct(IN struct ADAPTER *prAdapter, IN uint8_t ucBand,
	IN uint8_t ucProtType, IN uint8_t ucTrigType, IN uint8_t ucState)
{
	struct EXT_CMD_THERMAL_PROTECT_STATE_ACT *ext_cmd_buf;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	ext_cmd_buf = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
		sizeof(struct EXT_CMD_THERMAL_PROTECT_STATE_ACT));

	if (!ext_cmd_buf)
		return WLAN_STATUS_FAILURE;

	ext_cmd_buf->band_idx = ucBand;
	ext_cmd_buf->protect_type = ucProtType;
	ext_cmd_buf->trig_type = ucTrigType;
	ext_cmd_buf->state = ucState;
	ext_cmd_buf->sub_cmd_id = THERMAL_PROTECT_STATE_ACT;

	rStatus = wlanSendSetQueryExtCmd(prAdapter,
			CMD_ID_LAYER_0_EXT_MAGIC_NUM,
			EXT_CMD_ID_THERMAL_PROTECT,
			TRUE,
			FALSE,
			FALSE,
			NULL,
			NULL,
			sizeof(struct EXT_CMD_THERMAL_PROTECT_STATE_ACT),
			(uint8_t *) ext_cmd_buf, NULL,
			0);
	cnmMemFree(prAdapter, ext_cmd_buf);

	return rStatus;
}

uint32_t thrmProtTempConfig(IN struct ADAPTER *prAdapter,
			IN int32_t i4TrigTemp, IN int32_t i4RestoreTemp)
{
	prAdapter->rThrmProtCfg.i4TrigTemp = i4TrigTemp;
	prAdapter->rThrmProtCfg.i4RestoreTemp = i4RestoreTemp;

	if (prAdapter->rThrmProtCfg.ucLevel == CONN_PWR_THR_LV_5)
		return WLAN_STATUS_SUCCESS;

	if (prAdapter->rThrmProtCfg.ucMode > THERMAL_PROTECT_MODE_DISABLED) {
		thrmProtDisable(prAdapter, ENUM_BAND_0,
			THERMAL_PROTECT_TYPE_DUTY_CTRL,
			THERMAL_PROTECT_HIGH_TRIG);
	}

	DBGLOG(NIC, INFO, "Enable thermal monitor, trigger: %d, restore: %d\n",
					prAdapter->rThrmProtCfg.i4TrigTemp,
					prAdapter->rThrmProtCfg.i4RestoreTemp);

	return thrmProtEnable(prAdapter, ENUM_BAND_0,
			THERMAL_PROTECT_TYPE_DUTY_CTRL,
			THERMAL_PROTECT_HIGH_TRIG,
			i4TrigTemp, i4RestoreTemp,
			THRM_PROT_CHECKTIME_PASSIVE);
}

void thrmProtEventHandler(IN struct ADAPTER *prAdapter, IN uint8_t *prBuf)
{
	uint8_t ucEventId = (uint8_t) *(prBuf);

	DBGLOG(NIC, INFO, "Thermal protect event: %d\n", ucEventId);

	switch (ucEventId) {
	case THERMAL_PROTECT_EVENT_DUTY_NOTIFY:
	{
		struct EXT_EVENT_THERMAL_PROTECT_DUTY_NOTIFY *prEvent;

		prEvent =
			(struct EXT_EVENT_THERMAL_PROTECT_DUTY_NOTIFY *) prBuf;
		DBGLOG(NIC, INFO, "Duty Event, level: %d, duty: %d, temp: %d\n",
			prEvent->u1LevelIdx, prEvent->u1DutyPercent,
			prEvent->i4Temp);

		if (prEvent->i4Temp > prAdapter->rThrmProtCfg.i4TrigTemp) {
			connsysPowerTempUpdate(CONN_PWR_MSG_TEMP_TOO_HIGH,
						prEvent->i4Temp);
			/* Transit to aggressive monitor mode.*/
			if (prAdapter->rThrmProtCfg.ucMode ==
					THERMAL_PROTECT_MODE_PASSIVE) {

				thrmProtDisable(prAdapter, ENUM_BAND_0,
					THERMAL_PROTECT_TYPE_DUTY_CTRL,
					THERMAL_PROTECT_HIGH_TRIG);

				thrmProtEnable(prAdapter, ENUM_BAND_0,
					THERMAL_PROTECT_TYPE_DUTY_CTRL,
					THERMAL_PROTECT_HIGH_TRIG,
					prAdapter->rThrmProtCfg.i4TrigTemp,
					prAdapter->rThrmProtCfg.i4RestoreTemp,
					THRM_PROT_CHECKTIME_AGGRESIVE);

				prAdapter->rThrmProtCfg.ucMode =
					THERMAL_PROTECT_MODE_AGGRESSIVE;
			}
		} else if (prAdapter->rThrmProtCfg.ucMode ==
				THERMAL_PROTECT_MODE_AGGRESSIVE &&
				prEvent->i4Temp <
				prAdapter->rThrmProtCfg.i4RestoreTemp) {

			connsysPowerTempUpdate(CONN_PWR_MSG_TEMP_RECOVERY,
						prEvent->i4Temp);

			thrmProtDisable(prAdapter, ENUM_BAND_0,
						THERMAL_PROTECT_TYPE_DUTY_CTRL,
						THERMAL_PROTECT_HIGH_TRIG);

			thrmProtEnable(prAdapter, ENUM_BAND_0,
					THERMAL_PROTECT_TYPE_DUTY_CTRL,
					THERMAL_PROTECT_HIGH_TRIG,
					prAdapter->rThrmProtCfg.i4TrigTemp,
					prAdapter->rThrmProtCfg.i4RestoreTemp,
					THRM_PROT_CHECKTIME_PASSIVE);

			prAdapter->rThrmProtCfg.ucMode =
				THERMAL_PROTECT_MODE_PASSIVE;

		}
		break;
	}
	default:
		break;
	}
}

int thrmProtLvHandler(IN struct ADAPTER *prAdapter, IN uint8_t ucLevel)
{
	uint8_t ucOrigLevel;

	ucOrigLevel = prAdapter->rThrmProtCfg.ucLevel;
	prAdapter->rThrmProtCfg.ucLevel = ucLevel;

	if (prAdapter->rThrmProtCfg.i4TrigTemp == 0)
		return 0;

	if (ucLevel == CONN_PWR_THR_LV_5) {
		if (prAdapter->rThrmProtCfg.ucMode ==
				THERMAL_PROTECT_MODE_FORCE_DUTY_50)
			return 0;

		/* Disable existing passive thermal monitor */
		if (prAdapter->rThrmProtCfg.ucMode ==
				THERMAL_PROTECT_MODE_PASSIVE)

			thrmProtDisable(prAdapter, ENUM_BAND_0,
					THERMAL_PROTECT_TYPE_DUTY_CTRL,
					THERMAL_PROTECT_HIGH_TRIG);

		/* Enable aggressive thermal monitor */
		if (prAdapter->rThrmProtCfg.ucMode <
				THERMAL_PROTECT_MODE_AGGRESSIVE)

			thrmProtEnable(prAdapter, ENUM_BAND_0,
				THERMAL_PROTECT_TYPE_DUTY_CTRL,
				THERMAL_PROTECT_HIGH_TRIG,
				prAdapter->rThrmProtCfg.i4TrigTemp,
				prAdapter->rThrmProtCfg.i4RestoreTemp,
				THRM_PROT_CHECKTIME_AGGRESIVE);

		prAdapter->rThrmProtCfg.ucMode =
			THERMAL_PROTECT_MODE_FORCE_DUTY_50;

		thrmProtDutyCfg(prAdapter, ENUM_BAND_0, 3,
					THRM_PROT_DUTY_FORCE_50);

		thrmProtStateAct(prAdapter, ENUM_BAND_0,
					THERMAL_PROTECT_TYPE_DUTY_CTRL,
					THERMAL_PROTECT_HIGH_TRIG, 3);

		DBGLOG(NIC, INFO, "Force enter thermal level 3, duty: %d\n",
			THRM_PROT_DUTY_FORCE_50);
	} else {
		if (ucOrigLevel == CONN_PWR_THR_LV_5)
			thrmProtDutyCfg(prAdapter, ENUM_BAND_0, 3,
						THRM_PROT_DUTY_NORMAL);
	}

	return 0;
}

void thrmInit(IN struct ADAPTER *prAdapter)
{
	/* Register power level handler */
	kalPwrLevelHdlrRegister(prAdapter, thrmProtLvHandler);
}

#endif
/* End of thrm.c */
