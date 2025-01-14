// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt)	"BATTERY_CHG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/thermal.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/soc/qcom/battery_charger.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include "charger_partition.h"
#include "qti_battery_charger.h"

ATOMIC_NOTIFIER_HEAD(pen_charge_state_notifier);

#define MSG_OWNER_BC			32778
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2

/* opcode for battery charger */
#define BC_SET_NOTIFY_REQ		0x04
#define BC_DISABLE_NOTIFY_REQ		0x05
#define BC_NOTIFY_IND			0x07
#define BC_BATTERY_STATUS_GET		0x30
#define BC_BATTERY_STATUS_SET		0x31
#define BC_USB_STATUS_GET(port_id)	(0x32 | (port_id) << 16)
#define BC_USB_STATUS_SET(port_id)	(0x33 | (port_id) << 16)
#define BC_WLS_STATUS_GET		0x34
#define BC_WLS_STATUS_SET		0x35
#define BC_XM_STATUS_GET		0x50
#define BC_XM_STATUS_SET		0x51
#define BC_XM_STATUS_UPLOAD		0x60
#define BC_SHIP_MODE_REQ_SET		0x36
#define BC_SHUTDOWN_REQ_SET		0x37
#define BC_WLS_FW_CHECK_UPDATE		0x40
#define BC_WLS_FW_PUSH_BUF_REQ		0x41
#define BC_WLS_FW_UPDATE_STATUS_RESP	0x42
#define BC_WLS_FW_PUSH_BUF_RESP		0x43
#define BC_WLS_FW_GET_VERSION		0x44
#define BC_SHUTDOWN_NOTIFY		0x47
#define BC_CHG_CTRL_LIMIT_EN		0x48
#define BC_HBOOST_VMAX_CLAMP_NOTIFY	0x79
#define BC_GENERIC_NOTIFY		0x80

/* Generic definitions */
#define MAX_STR_LEN			128
#define BC_WAIT_TIME_MS			1000
#define GLINK_CRASH_RESET_LTIME_MS	30000
#define GLINK_CRASH_RESET_HTIME_MS	300000
#define WLS_FW_PREPARE_TIME_MS		1000
#define WLS_FW_WAIT_TIME_MS		500
#define WLS_FW_UPDATE_TIME_MS		1000
#define WLS_FW_BUF_SIZE			128
#define DEFAULT_RESTRICT_FCC_UA		1000000

#define CHG_DEBUG_DATA_LEN	200
#define WIRELESS_CHIP_FW_VERSION_LEN	16
#define CHG_DFX_DATA_LEN 	 5

#define WIRELESS_UUID_LEN	24

#define BATTERY_DIGEST_LEN 32
#define BATTERY_SS_AUTH_DATA_LEN 4
#define BATTERY_DIGEST_LEN 32
#define BATTERY_SS_AUTH_DATA_LEN 4
#define USBPD_UVDM_SS_LEN		4
#define USBPD_UVDM_VERIFIED_LEN		1
#define MAX_THERMAL_LEVEL		16

#if IS_ENABLED(CONFIG_MIEV)
#include "miev/mievent.h"
#include <linux/string.h>
#endif

#define DFX_ID_CHG_PD_AUTHEN_FAIL	909001004
#define DFX_ID_CHG_CP_ENABLE_FAIL	909001005

#define DFX_ID_CHG_NONE_STANDARD_CHG	909002001
#define DFX_ID_CHG_CORROSION_DISCHARGE	909002002
#define DFX_ID_CHG_LPD_DISCHARGE	909002003
#define DFX_ID_CHG_CP_VBUS_OVP	909002004
#define DFX_ID_CHG_CP_IBUS_OCP	909002005
#define DFX_ID_CHG_CP_VBAT_OVP	909002006
#define DFX_ID_CHG_CP_IBAT_OCP	909002007

#define DFX_ID_CHG_BATTERY_CYCLECOUNT	909003001
#define DFX_ID_CHG_SMART_ENDURANCE	909003004

#define DFX_ID_CHG_FG_IIC_ERR	909005001
#define DFX_ID_CHG_CP_ERR	909005002
#define DFX_ID_CHG_BATT_LINKER_ABSENT	909005003
#define DFX_ID_CHG_CP_TDIE	909005004
#define DFX_ID_CHG_LOW_TEMP_DISCHARGING	909005007
#define DFX_ID_CHG_HIGH_TEMP_DISCHARGING	909005008
#define DFX_ID_CHG_DUAL_BATTERY_MISSING	909005009

#define DFX_ID_CHG_DUAL_BATTERY_AUTH_FAIL	909007002

#define DFX_ID_CHG_BATTERY_TEMP_HIGH	909009001
#define DFX_ID_CHG_BATTERY_TEMP_LOW		909009002
#define DFX_ID_CHG_ANTIBURN_FAIL	909009003
#define DFX_ID_CHG_ANTIBURN	909002012

#define DFX_ID_CHG_WLS_FASTCHG_FAIL	909011001
#define DFX_ID_CHG_WLS_FOD_LOW_POWER	909011002
#define DFX_ID_CHG_WLS_RX_OTP	909012001
#define DFX_ID_CHG_WLS_RX_OVP	909012002
#define DFX_ID_CHG_WLS_RX_OCP	909012003
#define DFX_ID_CHG_WLS_TRX_FOD	909012004
#define DFX_ID_CHG_WLS_TRX_OCP	909012005
#define DFX_ID_CHG_WLS_TRX_UVLO	909012006
#define DFX_ID_CHG_WLS_TRX_IIC_ERR	909013001
#define DFX_ID_CHG_RX_IIC_ERR	909013004

#define DFX_ID_CHG_BATTERY_VOLTAGE_DIFFERENCE	909014002

#define MAX_INFO_LENGTH 1024

#if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
static struct drm_panel *active_panel;
#endif
enum usb_port_id {
	USB_1_PORT_ID,
	USB_2_PORT_ID,
	NUM_USB_PORTS,
};

enum psy_type {
	PSY_TYPE_BATTERY,
	PSY_TYPE_USB,
	PSY_TYPE_USB_2,
	PSY_TYPE_WLS,
	PSY_TYPE_XM,
	PSY_TYPE_MAX,
};

enum ship_mode_type {
	SHIP_MODE_PMIC,
	SHIP_MODE_PACK_SIDE,
};

/* property ids */
enum battery_property_id {
	BATT_STATUS,
	BATT_HEALTH,
	BATT_PRESENT,
	BATT_CHG_TYPE,
	BATT_CAPACITY,
	BATT_SOH,
	BATT_VOLT_OCV,
	BATT_VOLT_NOW,
	BATT_VOLT_MAX,
	BATT_CURR_NOW,
	BATT_CHG_CTRL_LIM,
	BATT_CHG_CTRL_LIM_MAX,
	BATT_CONSTANT_CURRENT,
	BATT_TEMP,
	BATT_TECHNOLOGY,
	BATT_CHG_COUNTER,
	BATT_CYCLE_COUNT,
	BATT_CHG_FULL_DESIGN,
	BATT_CHG_FULL,
	BATT_MODEL_NAME,
	BATT_TTF_AVG,
	BATT_TTE_AVG,
	BATT_RESISTANCE,
	BATT_POWER_NOW,
	BATT_POWER_AVG,
	BATT_CHG_CTRL_EN,
	BATT_CHG_CTRL_START_THR,
	BATT_CHG_CTRL_END_THR,
	BATT_CURR_AVG,
	BATT_PROP_MAX,
};

enum usb_property_id {
	USB_ONLINE,
	USB_VOLT_NOW,
	USB_VOLT_MAX,
	USB_CURR_NOW,
	USB_CURR_MAX,
	USB_INPUT_CURR_LIMIT,
	USB_TYPE,
	USB_ADAP_TYPE,
	USB_MOISTURE_DET_EN,
	USB_MOISTURE_DET_STS,
	USB_TEMP,
	USB_REAL_TYPE,
	USB_TYPEC_COMPLIANT,
	USB_PROP_MAX,
};

enum wireless_property_id {
	WLS_ONLINE,
	WLS_VOLT_NOW,
	WLS_VOLT_MAX,
	WLS_CURR_NOW,
	WLS_CURR_MAX,
	WLS_TYPE,
	WLS_BOOST_EN,
	WLS_HBOOST_VMAX,
	WLS_INPUT_CURR_LIMIT,
	WLS_ADAP_TYPE,
	WLS_CONN_TEMP,
	WLS_PROP_MAX,
};

enum {
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP = 0x80,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5,
	QTI_POWER_SUPPLY_USB_TYPE_USB_FLOAT,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3_CLASSB,
};

enum uvdm_state {
	USBPD_UVDM_DISCONNECT,
	USBPD_UVDM_CHARGER_VERSION,
	USBPD_UVDM_CHARGER_VOLTAGE,
	USBPD_UVDM_CHARGER_TEMP,
	USBPD_UVDM_SESSION_SEED,
	USBPD_UVDM_AUTHENTICATION,
	USBPD_UVDM_VERIFIED,
	USBPD_UVDM_REMOVE_COMPENSATION,
	USBPD_UVDM_REVERSE_AUTHEN,
	USBPD_UVDM_CONNECT,
};

enum xm_property_id {
	XM_PROP_RESISTANCE_ID,
	XM_PROP_VERIFY_DIGEST,
	XM_PROP_CONNECTOR_TEMP,
#if defined(CONFIG_MI_N2N3ANTIBURN)
	XM_PROP_ANTI_BURN,
#endif
	XM_PROP_CONNECTOR_TEMP2,
	XM_PROP_AUTHENTIC,
	XM_PROP_BATTERY_ADAPT_POWER_MATCH,
	XM_PROP_CHIP_OK,
	XM_PROP_VBUS_DISABLE,
	XM_PROP_REAL_TYPE,
	XM_PROP_THERMAL_BOARD_TEMP,
	XM_PROP_THERMAL_SCENE,
	XM_PROP_NTC_ALARM,
	XM_PROP_CHARGER_USER_VALUE_MAP,
	XM_PROP_TWO_NTC_PARAMETER1,
	XM_PROP_TWO_NTC_PARAMETER2,
	XM_PROP_TWO_NTC_PARAMETER3,
	XM_PROP_TWO_NTC_PARAMETER4,
	XM_PROP_TWO_NTC_PARAMETER5,
	XM_PROP_TWO_NTC_PARAMETER6,
	XM_PROP_VERIFY_PROCESS,
	XM_PROP_VDM_CMD_CHARGER_VERSION,
	XM_PROP_VDM_CMD_CHARGER_VOLTAGE,
	XM_PROP_VDM_CMD_CHARGER_TEMP,
	XM_PROP_VDM_CMD_SESSION_SEED,
	XM_PROP_VDM_CMD_AUTHENTICATION,
	XM_PROP_VDM_CMD_VERIFIED,
	XM_PROP_VDM_CMD_REMOVE_COMPENSATION,
	XM_PROP_VDM_CMD_REVERSE_AUTHEN,
	XM_PROP_CURRENT_STATE,
	XM_PROP_ADAPTER_ID,
	XM_PROP_ADAPTER_SVID,
	XM_PROP_PD_VERIFED,
	XM_PROP_PDO2,
	XM_PROP_UVDM_STATE,
	XM_PROP_CHG_CFG,
	XM_PROP_CP_MODE,
	XM_PROP_BQ2597X_CHIP_OK,
	XM_PROP_BQ2597X_SLAVE_CHIP_OK,
	XM_PROP_BQ2597X_BUS_CURRENT,
	XM_PROP_BQ2597X_SLAVE_BUS_CURRENT,
	XM_PROP_BQ2597X_BUS_DELTA,
	XM_PROP_BQ2597X_BUS_VOLTAGE,
	XM_PROP_BQ2597X_BATTERY_PRESENT,
	XM_PROP_BQ2597X_SLAVE_BATTERY_PRESENT,
	XM_PROP_BQ2597X_BATTERY_VOLTAGE,
	XM_PROP_BQ2597X_BATTERY_TEMPERATURE,
	XM_PROP_DAM_OVPGATE,
	XM_PROP_MASTER_SMB1396_ONLINE,
	XM_PROP_MASTER_SMB1396_IIN,
	XM_PROP_SLAVE_SMB1396_ONLINE,
	XM_PROP_SLAVE_SMB1396_IIN,
	XM_PROP_SMB_IIN_DIFF,
	XM_PROP_CC_ORIENTATION,
	XM_PROP_INPUT_SUSPEND,
	XM_PROP_FASTCHGMODE,
	XM_PROP_NIGHT_CHARGING,
	XM_PROP_USBINTERFACE,
	XM_PROP_SOC_DECIMAL,
	XM_PROP_SOC_DECIMAL_RATE,
	XM_PROP_QUICK_CHARGE_TYPE,
	XM_PROP_APDO_MAX,
	XM_PROP_POWER_MAX,
	XM_PROP_DIE_TEMPERATURE,
	XM_PROP_SLAVE_DIE_TEMPERATURE,
	XM_PROP_WLS_START,
	XM_PROP_TX_MACL,
	XM_PROP_TX_MACH,
	XM_PROP_RX_CRL,
	XM_PROP_RX_CRH,
	XM_PROP_RX_CEP,
	XM_PROP_BT_STATE,
	XM_PROP_REVERSE_CHG_MODE,
	XM_PROP_REVERSE_CHG_STATE,
	XM_PROP_RX_VOUT,
	XM_PROP_RX_VRECT,
	XM_PROP_RX_IOUT,
	XM_PROP_TX_ADAPTER,
	XM_PROP_OP_MODE,
	XM_PROP_WLS_DIE_TEMP,
	XM_PROP_WLS_BIN,
	XM_PROP_WLSCHARGE_CONTROL_LIMIT,
	XM_PROP_WLS_QUICK_CHARGE_CONTROL_LIMIT,
	XM_PROP_FW_VER,
	XM_PROP_TX_UUID,
	XM_PROP_WLS_THERMAL_REMOVE,
	XM_PROP_CHG_DEBUG,
	XM_PROP_WLS_FW_STATE,
	XM_PROP_WLS_CAR_ADAPTER,
	XM_PROP_WLS_TX_SPEED,
	XM_PROP_WLS_FC_FLAG,
	XM_PROP_RX_SS,
	XM_PROP_RX_OFFSET,
	XM_PROP_RX_SLEEP_MODE,
	XM_PROP_LOW_INDUCTANCE_OFFSET,
	XM_PROP_SET_RX_SLEEP,
	XM_PROP_PEN_MACL,
	XM_PROP_PEN_MACH,
	XM_PROP_TX_IOUT,
	XM_PROP_TX_VOUT,
	XM_PROP_PEN_SOC,
	XM_PROP_PEN_HALL3,
	XM_PROP_PEN_HALL4,
	XM_PROP_PEN_HALL3_S,
	XM_PROP_PEN_HALL4_S,
	XM_PROP_PEN_PPE_HALL_N,
	XM_PROP_PEN_PPE_HALL_S,
	XM_PROP_PEN_TX_SS,
	XM_PROP_PEN_PLACE_ERR,
	XM_PROP_FAKE_SS,
	XM_PROP_WLS_END,
	XM_PROP_SHUTDOWN_DELAY,
	XM_PROP_FAKE_TEMP,
	XM_PROP_THERMAL_REMOVE,
	XM_PROP_TYPEC_MODE,
	XM_PROP_MTBF_CURRENT,
	XM_PROP_THERMAL_TEMP,
	XM_PROP_FB_BLANK_STATE,
	XM_PROP_SMART_BATT,
	XM_PROP_SMART_FV,
	XM_PROP_SHIPMODE_COUNT_RESET,
	XM_PROP_SPORT_MODE,
	XM_PROP_BATT_CONNT_ONLINE,
	XM_PROP_FAKE_CYCLE,
  	XM_PROP_AFP_TEMP,
	XM_PROP_CC_SHORT_VBUS,
	XM_PROP_OTG_UI_SUPPORT,
	XM_PROP_CID_STATUS,
	XM_PROP_CC_TOGGLE,
	XM_PROP_SMART_CHG,
	XM_PROP_LOW_FAST_PARA,
	XM_PROP_LOWFAST_SW_PARA,
	XM_PROP_SMART_SIC_MODE,
	XM_PROP_NVTFG_MONITOR_ISC,
	XM_PROP_NVTFG_MONITOR_SOA,
	XM_PROP_OVER_PEAK_FLAG,
	XM_PROP_CURRENT_DEVIATION,
	XM_PROP_POWER_DEVIATION,
	XM_PROP_AVERAGE_CURRENT,
	XM_PROP_AVERAGE_TEMPERATURE,
	XM_PROP_START_LEARNING,
	XM_PROP_STOP_LEARNING,
	XM_PROP_SET_LEARNING_POWER,
	XM_PROP_GET_LEARNING_POWER,
	XM_PROP_GET_LEARNING_POWER_DEV,
	XM_PROP_GET_LEARNING_TIME_DEV,
	XM_PROP_SET_CONSTANT_POWER,
	XM_PROP_GET_REMAINING_TIME,
	XM_PROP_SET_REFERANCE_POWER,
	XM_PROP_GET_REFERANCE_CURRENT,
	XM_PROP_GET_REFERANCE_POWER,
	XM_PROP_START_LEARNING_B,
	XM_PROP_STOP_LEARNING_B,
	XM_PROP_SET_LEARNING_POWER_B,
	XM_PROP_GET_LEARNING_POWER_B,
	XM_PROP_GET_LEARNING_POWER_DEV_B,
	XM_PROP_FG2_OVER_PEAK_FLAG,
	XM_PROP_FG2_CURRENT_DEVIATION,
	XM_PROP_FG2_POWER_DEVIATION,
	XM_PROP_FG2_AVERAGE_CURRENT,
	XM_PROP_FG2_AVERAGE_TEMPERATURE,
	XM_PROP_FG2_START_LEARNING,
	XM_PROP_FG2_STOP_LEARNING,
	XM_PROP_FG2_SET_LEARNING_POWER,
	XM_PROP_FG2_GET_LEARNING_POWER,
	XM_PROP_FG2_GET_LEARNING_POWER_DEV,
	XM_PROP_FG2_GET_LEARNING_TIME_DEV,
	XM_PROP_FG2_SET_CONSTANT_POWER,
	XM_PROP_FG2_GET_REMAINING_TIME,
	XM_PROP_FG2_SET_REFERANCE_POWER,
	XM_PROP_FG2_GET_REFERANCE_CURRENT,
	XM_PROP_FG2_GET_REFERANCE_POWER,
	XM_PROP_FG2_START_LEARNING_B,
	XM_PROP_FG2_STOP_LEARNING_B,
	XM_PROP_FG2_SET_LEARNING_POWER_B,
	XM_PROP_FG2_GET_LEARNING_POWER_B,
	XM_PROP_FG2_GET_LEARNING_POWER_DEV_B,
	XM_PROP_FG1_GET_DESIGN_CAPACITY,
	XM_PROP_FG2_GET_DESIGN_CAPACITY,
	XM_PROP_FG1_QMAX,
	XM_PROP_FG1_RM,
	XM_PROP_FG1_FCC,
	XM_PROP_FG1_SOH,
	XM_PROP_FG1_FCC_SOH,
	XM_PROP_FG1_CYCLE,
	XM_PROP_FG1_FAST_CHARGE,
	XM_PROP_FG1_CURRENT_MAX,
	XM_PROP_FG1_VOL_MAX,
	XM_PROP_FG1_TSIM,
	XM_PROP_FG1_TAMBIENT,
	XM_PROP_FG1_TREMQ,
	XM_PROP_FG1_TFULLQ,
	XM_PROP_FG1_RSOC,
	XM_PROP_FG1_AI,
	XM_PROP_FG1_CELL1_VOL,
	XM_PROP_FG1_CELL2_VOL,
	XM_PROP_FG1_TEMP_MAX,
	XM_PROP_FG2_TEMP_MAX,
	XM_PROP_FG1_TIME_HT,
	XM_PROP_FG1_TIME_OT,
	XM_PROP_FG1_SEAL_SET,
	XM_PROP_FG1_SEAL_STATE,
	XM_PROP_FG1_DF_CHECK,
	XM_PROP_SLAVE_CHIP_OK,
	XM_PROP_SLAVE_AUTHENTIC,
	XM_PROP_FG2_RM,
	XM_PROP_FG2_FCC,
	XM_PROP_FG2_SOH,
	XM_PROP_FG2_CYCLE,
	XM_PROP_FG2_FAST_CHARGE,
	XM_PROP_FG2_CURRENT_MAX,
	XM_PROP_FG2_VOL_MAX,
	XM_PROP_FG2_RSOC,
	XM_PROP_FG1_SOC,
	XM_PROP_FG2_SOC,
	XM_PROP_FG1_IBATT,
	XM_PROP_FG2_IBATT,
	XM_PROP_FG1_VOL,
	XM_PROP_FG2_VOL,
	XM_PROP_FG1_TEMP,
	XM_PROP_FG2_TEMP,
	XM_PROP_FG1_ORITEMP,
	XM_PROP_FG2_ORITEMP,	
	XM_PROP_FG2_AI,
	XM_PROP_FG1_FC,
	XM_PROP_FG2_FC,
	XM_PROP_FG2_QMAX,
	XM_PROP_FG2_TSIM,
	XM_PROP_FG2_TAMBIENT,
	XM_PROP_FG2_TREMQ,
	XM_PROP_FG2_TFULLQ,
	XM_PROP_FG1_IMAX,
	XM_PROP_DOT_TEST,
	XM_PROP_FG_VENDOR_ID,
	XM_PROP_PACK_VENDOR_ID,
	XM_PROP_CELL_VENDOR_ID,
	XM_PROP_DOD_COUNT,
	XM_PROP_DOUBLE85,
	XM_PROP_SOH_NEW,
	XM_PROP_REMOVE_TEMP_LIMIT,
	XM_PROP_HAS_DP,
	XM_PROP_LPD_CONTROL,
	XM_PROP_LPD_SBU1,
	XM_PROP_LPD_SBU2,
	XM_PROP_LPD_CC1,
	XM_PROP_LPD_CC2,
	XM_PROP_PLATE_SHOCK,
	XM_PROP_CALC_RVALUE,
	XM_PROP_LPD_DP,
	XM_PROP_LPD_DM,
	XM_PROP_LPD_UART_SLEEP,
	XM_PROP_LPD_CHARGING,
	XM_PROP_MAX_TEMP_OCCUR_TIME,
	XM_PROP_RUN_TIME,
	XM_PROP_MAX_TEMP_TIME,
	XM_PROP_HANDLE_STATE,
	XM_PROP_HANDLE_STOP_CHARGING,
	XM_PROP_SC760X_CHIP_OK,
	XM_PROP_SC760X_SLAVE_CHIP_OK,
	XM_PROP_SC760X_SLAVE_IBATT_LIMIT,
	XM_PROP_QBG_CURR,
#if defined(CONFIG_MI_O16U_EUPD) || defined(CONFIG_MI_O11_EMPTY_BATTERY)
	XM_PROP_IS_EU_MODEL,
	XM_PROP_PPS_PTF,
#endif
#if defined(CONFIG_MI_O11_EMPTY_BATTERY)
	XM_PROP_CP_FSW,
	XM_PROP_CP_IOUT,
#endif
	XM_PROP_LAST_NODE,
	XM_PROP_MAX,
};
enum fg_venodr{
	POWER_SUPPLY_VENDOR_BYD = 0,
	POWER_SUPPLY_VENDOR_COSLIGHT,
	POWER_SUPPLY_VENDOR_SUNWODA,
	POWER_SUPPLY_VENDOR_NVT,
	POWER_SUPPLY_VENDOR_SCUD,
	POWER_SUPPLY_VENDOR_TWS,
	POWER_SUPPLY_VENDOR_LISHEN,
	POWER_SUPPLY_VENDOR_DESAY,
};

enum cell_vendor{
	POWER_SUPPLY_CELL_VENDOR_SAMSUNG = 0,
	POWER_SUPPLY_CELL_VENDOR_SONY,
	POWER_SUPPLY_CELL_VENDOR_SANYO,
	POWER_SUPPLY_CELL_VENDOR_LG,
	POWER_SUPPLY_CELL_VENDOR_PANASONIC,
	POWER_SUPPLY_CELL_VENDOR_ATL,
	POWER_SUPPLY_CELL_VENDOR_BYD,
	POWER_SUPPLY_CELL_VENDOR_TWS,
	POWER_SUPPLY_CELL_VENDOR_LISHEN,
	POWER_SUPPLY_CELL_VENDOR_COSLIGHT,
	POWER_SUPPLY_CELL_VENDOR_LWN,
};

enum xm_chg_debug_type {
	CHG_WLS_DEBUG,
	CHG_ADSP_LOG,
	CHG_UI_SOH_DATA,
	CHG_SLAVE_UI_SOH_DATA,
	CHG_UI_SOH_SN_CODE,
	CHG_UI_SLAVE_SOH_SN_CODE,
	CHG_CLOUD_FOD_DATA,
	CHG_DEBUG_TYPE_MAX,
};

enum xm_chg_dfx_type {
	CHG_DFX_DEFAULT,
	CHG_DFX_FG_IIC_ERR,
	CHG_DFX_CP_ERR,
	CHG_DFX_BATT_LINKER_ABSENT,
	CHG_DFX_LPD_DISCHARGE,
	CHG_DFX_LPD_DISCHARGE_RESET,
	CHG_DFX_CORROSION_DISCHARGE,
	CHG_DFX_NONE_STANDARD_CHG,
	CHG_DFX_RX_IIC_ERR,
	CHG_DFX_CP_VBUS_OVP,
	CHG_DFX_CP_IBUS_OCP,
	CHG_DFX_CP_VBAT_OVP,
	CHG_DFX_CP_IBAT_OCP,
	CHG_DFX_PPS_AUTH_FAIL,
	CHG_DFX_CP_ENABLE_FAIL,
	CHG_DFX_BATTERY_CYCLECOUNT,
	CHG_DFX_SMART_ENDURANCE,
	CHG_DFX_LOW_TEMP_DISCHARGING,
	CHG_DFX_HIGH_TEMP_DISCHARGING,
	CHG_DFX_DUAL_BATTERY_MISSING,
	CHG_DFX_DUAL_BATTERY_AUTH_FAIL,
	CHG_DFX_BATTERY_TEMP_HIGH,
	CHG_DFX_BATTERY_TEMP_LOW,
	CHG_DFX_ANTIBURN_FAIL,
	CHG_DFX_ANTIBURN,
	CHG_DFX_BATTERY_VOLTAGE_DIFFERENCE,
	CHG_DFX_CP_TDIE,
	CHG_DFX_WLS_FASTCHG_FAIL,
	CHG_DFX_WLS_FOD_LOW_POWER,
	CHG_DFX_WLS_RX_OTP,
	CHG_DFX_WLS_RX_OVP,
	CHG_DFX_WLS_RX_OCP,
	CHG_DFX_WLS_TRX_FOD,
	CHG_DFX_WLS_TRX_OCP,
	CHG_DFX_WLS_TRX_UVLO,
	CHG_DFX_WLS_TRX_IIC_ERR,
	CHG_DFX_MAX_INDEX,
};

static const char *const xm_dfx_chg_report_text[] = {
	"DEFAULT_TEXT", "fgI2cErr", "cpErr",
	"battLinkerAbsent", "lpdDischarge", "lpdDischargeReset",
	"corrosionDischarge", "noneStandartChg","wlsRxI2CErr",
	"CpVbusOvp", "CpIbusOcp", "CpVbatOvp", "CpIbatOcp",
	"PdAuthFail", "CpEnFail", "chgBattCycle", "NotChgInLowTemp",
	"NotChgInHighTemp", "DualBattLinkerAbsent", "chgBattAuthFail",
	"TbatHot", "TbatCold", "AntiFail", "dualVbatDiff", "CpTdieHot",
	"WlsFastChgFail", "WlsQLow", "WlsRxOTP", "WlsRxOVP", "WlsRxOCP",
	"WlsTrxFod", "WlsTrxOCP", "WlsTrxUVLO", "WlsTrxI2cErr"
};


struct battery_charger_set_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			power_state;
	u32			low_capacity;
	u32			high_capacity;
};

struct battery_charger_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
};

struct battery_charger_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			property_id;
	u32			value;
};

struct battery_charger_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			value;
	u32			ret_code;
};

struct battery_model_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	char			model[MAX_STR_LEN];
};

struct wireless_fw_check_req {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
	u32			fw_size;
	u32			fw_crc;
};

struct wireless_fw_check_resp {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};

struct wireless_fw_push_buf_req {
	struct pmic_glink_hdr	hdr;
	u8			buf[WLS_FW_BUF_SIZE];
	u32			fw_chunk_id;
};

struct wireless_fw_push_buf_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_status;
};

struct wireless_fw_update_status {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_done;
};

struct wireless_fw_get_version_req {
	struct pmic_glink_hdr	hdr;
};

struct wireless_fw_get_version_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
};

struct battery_charger_ship_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			ship_mode_type;
};

struct battery_charger_chg_ctrl_msg {
	struct pmic_glink_hdr	hdr;
	u32			enable;
	u32			target_soc;
	u32			delta_soc;
};

struct wireless_tx_uuid_msg {
	struct pmic_glink_hdr	hdr;
	u32				property_id;
	u32				value;
	char			version[WIRELESS_UUID_LEN];
};

struct psy_state {
	struct power_supply	*psy;
	char			*model;
	const int		*map;
	u32			*prop;
	u32			prop_count;
	u32			opcode_get;
	u32			opcode_set;
};

/*add xiaomi msg*/
struct battery_charger_shutdown_req_msg {
	struct pmic_glink_hdr	hdr;
};

struct xm_ss_auth_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			data[BATTERY_SS_AUTH_DATA_LEN];
};

struct chg_debug_msg {
	struct pmic_glink_hdr   hdr;
	u32                     property_id;
	u8                      type;
	char                    data[CHG_DEBUG_DATA_LEN];
};

struct chg_dfx_report_msg {
	struct pmic_glink_hdr   hdr;
	u32                     property_id;
	u8                      type;
	int            data[CHG_DFX_DATA_LEN];
};

struct chg_prop_upload_msg {
	struct pmic_glink_hdr   hdr;
	u32                     property_id;
	u32                     property_value;
	bool                    upload;
};

struct wireless_chip_fw_msg {
	struct pmic_glink_hdr	hdr;
	u32				property_id;
	u32				value;
	char			version[WIRELESS_CHIP_FW_VERSION_LEN];
};

struct xm_set_wls_bin_req_msg {
  struct pmic_glink_hdr hdr;
  u32 property_id;
  u16 total_length;
  u8 serial_number;
  u8 fw_area;
  u8 wls_fw_bin[MAX_STR_LEN];
};

struct xm_verify_digest_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u8			digest[BATTERY_DIGEST_LEN];
	bool		slave_fg;
};

struct battery_chg_dev {
	struct device			*dev;
	struct class			battery_class;
	struct pmic_glink_client	*client;
	struct mutex			rw_lock;
	struct rw_semaphore		state_sem;
	struct completion		ack;
	struct completion		fw_buf_ack;
	struct completion		fw_update_ack;
	struct psy_state		psy_list[PSY_TYPE_MAX];
	struct dentry			*debugfs_dir;
	void				*notifier_cookie;
	u32				*thermal_levels;
	const char			*wls_fw_name;
	int				curr_thermal_level;
	int				curr_wlsthermal_level;
	int				curr_wls_quick_thermal_level;
	int				num_thermal_levels;
	int				shutdown_volt_mv;
	int				last_capacity;
	bool				fast_update_temp;
	atomic_t			state;
	struct work_struct		subsys_up_work;
	struct work_struct		usb_type_work;
	struct work_struct		battery_check_work;
	struct work_struct		lpd_status_work;
	struct delayed_work		charger_debug_info_print_work;
	struct delayed_work		xm_prop_change_work;
	struct delayed_work		xm_smart_prop_update_work;
	struct delayed_work		batt_update_work;
	struct delayed_work	    panel_notify_register_work;
	struct delayed_work	    glink_crash_num_work;
	bool				debug_work_en;
	bool				glink_crash_timeout_reset_flag;
	int				fake_soc;
	bool				block_tx;
	bool				ship_mode_en;
	bool				debug_battery_detected;
	bool				wls_fw_update_reqd;
	u32				wls_fw_version;
	u16				wls_fw_crc;
	u32				wls_fw_update_time_ms;
	struct notifier_block		reboot_notifier;
	struct notifier_block		shutdown_notifier;
	u32				thermal_fcc_ua;
	u32				restrict_fcc_ua;
	u32				last_fcc_ua;
	u32				usb_icl_ua[NUM_USB_PORTS];
	u32				thermal_fcc_step;
	bool				restrict_chg_en;
	u8				chg_ctrl_start_thr;
	u8				chg_ctrl_end_thr;
	u8				glink_crash_count;
	bool				chg_ctrl_en;
	bool				usb_active[NUM_USB_PORTS];
	bool				initialized;
	bool				notify_en;
	u32				reverse_chg_flag;
	u32				boost_mode;
	char			wireless_chip_fw_version[WIRELESS_CHIP_FW_VERSION_LEN];
	char			wireless_tx_uuid_version[WIRELESS_UUID_LEN];
	u8				*digest;
	u32				*ss_auth_data;
	char				wls_debug_data[CHG_DEBUG_DATA_LEN];
	char				ui_soh_data[CHG_DEBUG_DATA_LEN];
	char				ui_slave_soh_data[CHG_DEBUG_DATA_LEN];
	char				cloud_fod_data[CHG_DEBUG_DATA_LEN];
	bool				battery_auth;
	bool				slave_battery_auth;
	bool				slave_fg_verify_flag;
	int				mtbf_current;
	int				blank_state;
	bool			shutdown_delay_en;
	bool			report_power_absent;
	bool				read_capacity_timeout;
	int				thermal_board_temp;
	int				thermal_scene;
	struct				notifier_block chg_nb;
	bool				error_prop;
	bool				has_usb_2;
	bool				ut_test;
	bool				ut_test_region;
	const char			* cycle_volt;
	const char			* cycle_step_curr;
	const char			* temp_term_curr;
	const char			* thermal;
	const char			* cycle_volt_gl;
	const char			* cycle_step_curr_gl;
	const char			* thermal_gl;
	const char			* temp_term_curr_gl_nvt;
	const char			* temp_term_curr_gl_sunwoda;
	const char			* cycle_volt_cn;
	const char			* cycle_step_curr_cn;
	const char			* thermal_cn;
	const char			* temp_term_curr_cn_nvt;
	const char			* temp_term_curr_cn_sunwoda;

	struct work_struct pen_notifier_work;
};

static void xm_handle_adsp_dfx_report(struct chg_dfx_report_msg *Context);

BLOCKING_NOTIFIER_HEAD(charger_notifier);
EXPORT_SYMBOL_GPL(charger_notifier);

int charger_reg_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&charger_notifier, nb);
}
EXPORT_SYMBOL_GPL(charger_reg_notifier);

int charger_unreg_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&charger_notifier, nb);
}
EXPORT_SYMBOL_GPL(charger_unreg_notifier);

int charger_notifier_call_chain(unsigned long event, int val)
{
	return blocking_notifier_call_chain(&charger_notifier, event, &val);
}
EXPORT_SYMBOL_GPL(charger_notifier_call_chain);

static const int battery_prop_map[BATT_PROP_MAX] = {
	[BATT_STATUS]		= POWER_SUPPLY_PROP_STATUS,
	[BATT_HEALTH]		= POWER_SUPPLY_PROP_HEALTH,
	[BATT_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[BATT_CHG_TYPE]		= POWER_SUPPLY_PROP_CHARGE_TYPE,
	[BATT_CAPACITY]		= POWER_SUPPLY_PROP_CAPACITY,
	[BATT_VOLT_OCV]		= POWER_SUPPLY_PROP_VOLTAGE_OCV,
	[BATT_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[BATT_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[BATT_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[BATT_CHG_CTRL_LIM]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	[BATT_CHG_CTRL_LIM_MAX]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	[BATT_CONSTANT_CURRENT]	= POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	[BATT_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[BATT_TECHNOLOGY]	= POWER_SUPPLY_PROP_TECHNOLOGY,
	[BATT_CHG_COUNTER]	= POWER_SUPPLY_PROP_CHARGE_COUNTER,
	[BATT_CYCLE_COUNT]	= POWER_SUPPLY_PROP_CYCLE_COUNT,
	[BATT_CHG_FULL_DESIGN]	= POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	[BATT_CHG_FULL]		= POWER_SUPPLY_PROP_CHARGE_FULL,
	[BATT_MODEL_NAME]	= POWER_SUPPLY_PROP_MODEL_NAME,
	[BATT_TTF_AVG]		= POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	[BATT_TTE_AVG]		= POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	[BATT_POWER_NOW]	= POWER_SUPPLY_PROP_POWER_NOW,
	[BATT_POWER_AVG]	= POWER_SUPPLY_PROP_POWER_AVG,
	[BATT_CHG_CTRL_START_THR] = POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	[BATT_CHG_CTRL_END_THR]   = POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	[BATT_CURR_AVG]		= POWER_SUPPLY_PROP_CURRENT_AVG,
};

static const int usb_prop_map[USB_PROP_MAX] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
	[USB_TEMP]		= POWER_SUPPLY_PROP_TEMP,
};

static const int wls_prop_map[WLS_PROP_MAX] = {
	[WLS_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[WLS_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[WLS_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[WLS_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[WLS_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[WLS_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[WLS_CONN_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[WLS_BOOST_EN]		= POWER_SUPPLY_PROP_PRESENT,
};

/*add xm_power_supply text*/
static const char * const POWER_SUPPLY_VENDOR_TEXT[] = {
	[POWER_SUPPLY_VENDOR_BYD]		= "BYD",
	[POWER_SUPPLY_VENDOR_COSLIGHT]	= "COSLIGHT",
	[POWER_SUPPLY_VENDOR_SUNWODA]	= "SUNWODA",
	[POWER_SUPPLY_VENDOR_NVT]		= "NVT",
	[POWER_SUPPLY_VENDOR_SCUD]		= "SCUD",
	[POWER_SUPPLY_VENDOR_TWS]		= "TWS",
	[POWER_SUPPLY_VENDOR_DESAY]		= "DESAY",
};

static const char * const POWER_SUPPLY_CELL_VENDOR_TEXT[] = {
	[POWER_SUPPLY_CELL_VENDOR_SAMSUNG]		= "SAMSUNG",
	[POWER_SUPPLY_CELL_VENDOR_SONY]			= "SONY",
	[POWER_SUPPLY_CELL_VENDOR_SANYO]		= "SANYO",
	[POWER_SUPPLY_CELL_VENDOR_LG]			= "LG",
	[POWER_SUPPLY_CELL_VENDOR_PANASONIC]	= "PANASONIC",
	[POWER_SUPPLY_CELL_VENDOR_ATL]			= "ATL",
	[POWER_SUPPLY_CELL_VENDOR_BYD]			= "BYD",
	[POWER_SUPPLY_CELL_VENDOR_TWS]			= "TWS",
	[POWER_SUPPLY_CELL_VENDOR_LISHEN]		= "LISHEN",
	[POWER_SUPPLY_CELL_VENDOR_COSLIGHT]		= "COSLIGHT",
	[POWER_SUPPLY_CELL_VENDOR_LWN]			= "LWN",
};

static const int xm_prop_map[XM_PROP_MAX] = {};

static const char * const power_supply_usb_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

static const char *const qc_power_supply_usb_type_text[] = {
	"HVDCP", "HVDCP_3", "HVDCP_3P5", "USB_FLOAT", "HVDCP_3"
};

static const char * const qc_power_supply_wls_type_text[] = {
	"Unknown", "BPP", "EPP", "HPP"
};

static const char * const power_supply_usbc_text[] = {
	"Nothing attached",
	"Source attached (default current)",
	"Source attached (medium current)",
	"Source attached (high current)",
	"Non compliant",
	"Sink attached",
	"Powered cable w/ sink",
	"Debug Accessory",
	"Audio Adapter",
	"Powered cable w/o sink",
};

static RAW_NOTIFIER_HEAD(hboost_notifier);

int register_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&hboost_notifier, nb);
}
EXPORT_SYMBOL(register_hboost_event_notifier);

int unregister_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&hboost_notifier, nb);
}
EXPORT_SYMBOL(unregister_hboost_event_notifier);

int StringToHex(char *str, unsigned char *out, unsigned int *outlen)
{
	char *p = str;
	char high = 0, low = 0;
	int tmplen = strlen(p), cnt = 0;
	tmplen = strlen(p);
	while(cnt < (tmplen / 2))
	{
		high = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;
		low = (*(++ p) > '9' && ((*p <= 'F') || (*p <= 'f'))) ? *(p) - 48 - 7 : *(p) - 48;
		out[cnt] = ((high & 0x0f) << 4 | (low & 0x0f));
		p ++;
		cnt ++;
	}
	if(tmplen % 2 != 0) out[cnt] = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;

	if(outlen != NULL) *outlen = tmplen / 2 + tmplen % 2;

	return tmplen / 2 + tmplen % 2;
}

static int battery_chg_fw_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	down_read(&bcdev->state_sem);
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		up_read(&bcdev->state_sem);
		return -ENOTCONN;
	}

	reinit_completion(&bcdev->fw_buf_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->fw_buf_ack,
					msecs_to_jiffies(WLS_FW_WAIT_TIME_MS));
		if (!rc) {
			up_read(&bcdev->state_sem);
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	up_read(&bcdev->state_sem);
	return rc;
}

static int battery_chg_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;
	static u8 glink_crash_num = 0;
	struct battery_charger_req_msg *req_msg;

	/*
	 * When the subsystem goes down, it's better to return the last
	 * known values until it comes back up. Hence, return 0 so that
	 * pmic_glink_write() is not attempted until pmic glink is up.
	 */
	down_read(&bcdev->state_sem);
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		up_read(&bcdev->state_sem);
		return 0;
	}

	if (bcdev->debug_battery_detected && bcdev->block_tx) {
		up_read(&bcdev->state_sem);
		return 0;
	}

	mutex_lock(&bcdev->rw_lock);
	reinit_completion(&bcdev->ack);
	bcdev->error_prop = false;
	req_msg = data;
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		if(bcdev->glink_crash_timeout_reset_flag == true){
                  	bcdev->glink_crash_count++;
                  	if(bcdev->glink_crash_count > 3){
                        	glink_crash_num = 19;
                        } else {
                          	glink_crash_num = 0;
                        }
			bcdev->glink_crash_timeout_reset_flag = false;
		}

		if(glink_crash_num > 20){
			up_read(&bcdev->state_sem);
			mutex_unlock(&bcdev->rw_lock);
                  	if(bcdev->glink_crash_count > 3){
                          	bcdev->glink_crash_count = 4;
                        	schedule_delayed_work(&bcdev->glink_crash_num_work, msecs_to_jiffies(GLINK_CRASH_RESET_HTIME_MS));
                        } else {
                        	schedule_delayed_work(&bcdev->glink_crash_num_work, msecs_to_jiffies(GLINK_CRASH_RESET_LTIME_MS));
                        }
			return 0;
		} else {
			rc = wait_for_completion_timeout(&bcdev->ack,
					msecs_to_jiffies(BC_WAIT_TIME_MS));
		}

		if (!rc) {
			glink_crash_num++;
			if (len == sizeof(*req_msg)) {
				if (BATT_CAPACITY == req_msg->property_id)
					bcdev->read_capacity_timeout = true;
			}
			up_read(&bcdev->state_sem);
			mutex_unlock(&bcdev->rw_lock);
			if (bcdev->read_capacity_timeout)
				return 0;
			else
				return -ETIMEDOUT;
		} else {
                  	bcdev->glink_crash_count = 0;
			if (len == sizeof(*req_msg)) {
				if (BATT_CAPACITY == req_msg->property_id && bcdev->read_capacity_timeout)
					bcdev->read_capacity_timeout = false;
			}
		}
		rc = 0;

		if (bcdev->error_prop) {
			bcdev->error_prop = false;
			rc = -ENODATA;
		}
	}
	mutex_unlock(&bcdev->rw_lock);
	up_read(&bcdev->state_sem);

	return rc;
}

static int write_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u32 val)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = val;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = 0;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int get_property_id(struct psy_state *pst,
			enum power_supply_property prop)
{
	u32 i;

	for (i = 0; i < pst->prop_count; i++)
		if (pst->map[i] == prop)
			return i;

	return -ENOENT;
}

/*add pa and batt auth*/
static int write_ss_auth_prop_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u32* buff)
{
	struct xm_ss_auth_resp_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;
	memcpy(req_msg.data, buff, BATTERY_SS_AUTH_DATA_LEN*sizeof(u32));

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_ss_auth_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct xm_ss_auth_resp_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static void battery_chg_notify_enable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	if (!bcdev->notify_en) {
		req_msg.hdr.owner = MSG_OWNER_BC;
		req_msg.hdr.type = MSG_TYPE_NOTIFY;
		req_msg.hdr.opcode = BC_SET_NOTIFY_REQ;

		rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
		if (rc < 0)
			pr_err("Failed to enable notification rc=%d\n", rc);
		else
			bcdev->notify_en = true;
	}
}

static void battery_chg_state_cb(void *priv, enum pmic_glink_state state)
{
	struct battery_chg_dev *bcdev = priv;

	down_write(&bcdev->state_sem);
	if (!bcdev->initialized) {
		pr_warn("Driver not initialized, pmic_glink state %d\n", state);
		up_write(&bcdev->state_sem);
		return;
	}
	atomic_set(&bcdev->state, state);
	up_write(&bcdev->state_sem);

	if (state == PMIC_GLINK_STATE_UP)
		schedule_work(&bcdev->subsys_up_work);
	else if (state == PMIC_GLINK_STATE_DOWN)
		bcdev->notify_en = false;
}

/**
 * qti_battery_charger_get_prop() - Gets the property being requested
 *
 * @name: Power supply name
 * @prop_id: Property id to be read
 * @val: Pointer to value that needs to be updated
 *
 * Return: 0 if success, negative on error.
 */
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	int rc = 0;

	if (prop_id >= BATTERY_CHARGER_PROP_MAX)
		return -EINVAL;

	if (strcmp(name, "battery") && strcmp(name, "usb") &&
	    strcmp(name, "usb-2") && strcmp(name, "wireless"))
		return -EINVAL;

	psy = power_supply_get_by_name(name);
	if (!psy)
		return -ENODEV;

	bcdev = power_supply_get_drvdata(psy);
	if (!bcdev)
		return -ENODEV;

	power_supply_put(psy);

	switch (prop_id) {
	case BATTERY_RESISTANCE:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
		if (!rc)
			*val = pst->prop[BATT_RESISTANCE];
		break;
	default:
		break;
	}

	return rc;
}
EXPORT_SYMBOL(qti_battery_charger_get_prop);

static bool validate_message(struct battery_chg_dev *bcdev,
			struct battery_charger_resp_msg *resp_msg, size_t len)
{
	struct xm_verify_digest_resp_msg *verify_digest_resp_msg = (struct xm_verify_digest_resp_msg *)resp_msg;
	struct xm_ss_auth_resp_msg *ss_auth_resp_msg = (struct xm_ss_auth_resp_msg *)resp_msg;

	if (len == sizeof(*verify_digest_resp_msg) || len == sizeof(*ss_auth_resp_msg)) {
		return true;
	}

	if (len != sizeof(*resp_msg)) {
		pr_err("Incorrect response length %zu for opcode %#x\n", len,
			resp_msg->hdr.opcode);
		return false;
	}

	if (resp_msg->ret_code) {
		pr_err_ratelimited("Error in response for opcode %#x prop_id %u, rc=%d\n",
			resp_msg->hdr.opcode, resp_msg->property_id,
			(int)resp_msg->ret_code);
		bcdev->error_prop = true;
		return false;
	}

	return true;
}

#define MODEL_DEBUG_BOARD	"Debug_Board"
static void handle_message(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_resp_msg *resp_msg = data;
	struct battery_model_resp_msg *model_resp_msg = data;
	struct wireless_fw_check_resp *fw_check_msg;
	struct wireless_fw_push_buf_resp *fw_resp_msg;
	struct wireless_fw_update_status *fw_update_msg;
	struct wireless_fw_get_version_resp *fw_ver_msg;
	struct xm_verify_digest_resp_msg *verify_digest_resp_msg = data;
	struct xm_ss_auth_resp_msg *ss_auth_resp_msg = data;
	struct chg_debug_msg *chg_debug_data = data;
	struct chg_dfx_report_msg *chg_dfx_data = data;
	struct wireless_chip_fw_msg *wireless_chip_fw_resp_msg = data;
	struct wireless_tx_uuid_msg *wireless_tx_uuid_msg = data;
	struct psy_state *pst;
	bool ack_set = false;

	switch (resp_msg->hdr.opcode) {
	case BC_BATTERY_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

		/* Handle model response uniquely as it's a string */
		if (pst->model && len == sizeof(*model_resp_msg)) {
			memcpy(pst->model, model_resp_msg->model, MAX_STR_LEN);
			ack_set = true;
			bcdev->debug_battery_detected = !strcmp(pst->model,
					MODEL_DEBUG_BOARD);
			break;
		}

		/* Other response should be of same type as they've u32 value */
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_USB_STATUS_GET(USB_1_PORT_ID):
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_USB_STATUS_GET(USB_2_PORT_ID):
		if (!bcdev->has_usb_2) {
			pr_debug("opcode: %u for missing USB_2 port\n",
					resp_msg->hdr.opcode);
			break;
		}
		pst = &bcdev->psy_list[PSY_TYPE_USB_2];
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_XM_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_XM];

		/* Handle digest response uniquely as it's a string */
		if (bcdev->digest && len == sizeof(*verify_digest_resp_msg)) {
			memcpy(bcdev->digest, verify_digest_resp_msg->digest, BATTERY_DIGEST_LEN);
			ack_set = true;
			break;
		}
		if (bcdev->ss_auth_data && len == sizeof(*ss_auth_resp_msg)) {
			memcpy(bcdev->ss_auth_data, ss_auth_resp_msg->data, BATTERY_SS_AUTH_DATA_LEN*sizeof(u32));
			ack_set = true;
			break;
		}
		/* Handle model response uniquely as it's a string */
		if (len == sizeof(*wireless_chip_fw_resp_msg)) {
			memcpy(bcdev->wireless_chip_fw_version, wireless_chip_fw_resp_msg->version, WIRELESS_CHIP_FW_VERSION_LEN);
			ack_set = true;
			break;
		}
		if (len == sizeof(*wireless_tx_uuid_msg)) {
			memcpy(bcdev->wireless_tx_uuid_version, wireless_tx_uuid_msg->version, WIRELESS_UUID_LEN);
			ack_set = true;
			break;
		}

		/* handle DFX ADSP report  */
		if (len == sizeof(*chg_dfx_data)) {
			xm_handle_adsp_dfx_report(chg_dfx_data);
			ack_set = true;
			break;
		}

		if (len == sizeof(*chg_debug_data)) {
			if (chg_debug_data->type == CHG_ADSP_LOG) {
				;
			} else if (chg_debug_data->type == CHG_WLS_DEBUG) {
				memcpy(bcdev->wls_debug_data, chg_debug_data->data, CHG_DEBUG_DATA_LEN);
				ack_set = true;
			} else if (chg_debug_data->type == CHG_UI_SOH_DATA) {
				memset(bcdev->ui_soh_data, '\0', CHG_DEBUG_DATA_LEN);
				memcpy(bcdev->ui_soh_data, chg_debug_data->data, CHG_DEBUG_DATA_LEN);
				ack_set = true;
			} else if (chg_debug_data->type == CHG_UI_SOH_SN_CODE) {
				memset(bcdev->ui_soh_data, '\0', CHG_DEBUG_DATA_LEN);
				memcpy(bcdev->ui_soh_data, chg_debug_data->data, CHG_DEBUG_DATA_LEN);
				ack_set = true;
			} else if (chg_debug_data->type == CHG_SLAVE_UI_SOH_DATA) {
				memset(bcdev->ui_slave_soh_data, '\0', CHG_DEBUG_DATA_LEN);
				memcpy(bcdev->ui_slave_soh_data, chg_debug_data->data, CHG_DEBUG_DATA_LEN);
				ack_set = true;
			} else if (chg_debug_data->type == CHG_UI_SLAVE_SOH_SN_CODE) {
                                memset(bcdev->ui_slave_soh_data, '\0', CHG_DEBUG_DATA_LEN);
                                memcpy(bcdev->ui_slave_soh_data, chg_debug_data->data, CHG_DEBUG_DATA_LEN);
                                ack_set = true;
                        }else if (chg_debug_data->type == CHG_CLOUD_FOD_DATA) {
				memset(bcdev->cloud_fod_data, '\0', CHG_DEBUG_DATA_LEN);
				memcpy(bcdev->cloud_fod_data, chg_debug_data->data, CHG_DEBUG_DATA_LEN);
				ack_set = true;
			}
			break;
		}
		if (validate_message(bcdev, resp_msg, len) && resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}
		break;
	case BC_USB_STATUS_SET(USB_2_PORT_ID):
		if (!bcdev->has_usb_2) {
			pr_debug("opcode: %u for missing USB_2 port\n",
					resp_msg->hdr.opcode);
			break;
		}
		fallthrough;
	case BC_BATTERY_STATUS_SET:
	case BC_USB_STATUS_SET(USB_1_PORT_ID):
	case BC_WLS_STATUS_SET:
		if (validate_message(bcdev, data, len))
			ack_set = true;

		break;
	case BC_XM_STATUS_SET:
		if (validate_message(bcdev, data, len))
			ack_set = true;
		break;
	case BC_SET_NOTIFY_REQ:
	case BC_DISABLE_NOTIFY_REQ:
	case BC_SHUTDOWN_NOTIFY:
	case BC_SHIP_MODE_REQ_SET:
	case BC_CHG_CTRL_LIMIT_EN:
	case BC_SHUTDOWN_REQ_SET:
		/* Always ACK response for notify or ship_mode request */
		ack_set = true;
		break;
	case BC_WLS_FW_CHECK_UPDATE:
		if (len == sizeof(*fw_check_msg)) {
			fw_check_msg = data;
			if (fw_check_msg->ret_code == 1)
				bcdev->wls_fw_update_reqd = true;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_check_update\n",
				len);
		}
		break;
	case BC_WLS_FW_PUSH_BUF_RESP:
		if (len == sizeof(*fw_resp_msg)) {
			fw_resp_msg = data;
			if (fw_resp_msg->fw_update_status == 1)
				complete(&bcdev->fw_buf_ack);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_push_buf_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_UPDATE_STATUS_RESP:
		if (len == sizeof(*fw_update_msg)) {
			fw_update_msg = data;
			if (fw_update_msg->fw_update_done == 1)
				complete(&bcdev->fw_update_ack);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_update_status_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_GET_VERSION:
		if (len == sizeof(*fw_ver_msg)) {
			fw_ver_msg = data;
			bcdev->wls_fw_version = fw_ver_msg->fw_version;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_get_version\n",
				len);
		}
		break;
	default:
		pr_err("Unknown opcode: %u\n", resp_msg->hdr.opcode);
		break;
	}

	if (ack_set || bcdev->error_prop)
		complete(&bcdev->ack);
}

static struct power_supply_desc usb_psy_desc[NUM_USB_PORTS];

static void battery_chg_update_usb_type_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, usb_type_work);
	struct power_supply_desc *desc;
	struct psy_state *pst;
	int rc, i;

	for (i = 0; i < NUM_USB_PORTS; i++) {
		if (!bcdev->usb_active[i])
			continue;

		bcdev->usb_active[i] = false;

		switch (i) {
		case USB_1_PORT_ID:
			pst = &bcdev->psy_list[PSY_TYPE_USB];
			break;
		case USB_2_PORT_ID:
			pst = &bcdev->psy_list[PSY_TYPE_USB_2];
			break;
		default:
			pr_err_ratelimited("USB port %d not supported\n", i);
			continue;
		}

		desc = &usb_psy_desc[i];

		rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
		if (rc < 0) {
			pr_err("Failed to read USB_ADAP_TYPE rc=%d\n", rc);
			continue;
		}

		if (pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_SDP &&
		    pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_PD)
			bcdev->usb_icl_ua[i] = 0;

		switch (pst->prop[USB_ADAP_TYPE]) {
		if (bcdev->report_power_absent == 1
			&& pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_UNKNOWN) {
			desc->type = POWER_SUPPLY_TYPE_UNKNOWN;
			break;
		}
                case POWER_SUPPLY_TYPE_UNKNOWN:
			desc->type = POWER_SUPPLY_TYPE_UNKNOWN;
			break;
		case POWER_SUPPLY_USB_TYPE_SDP:
			desc->type = POWER_SUPPLY_TYPE_USB;
			break;
		case POWER_SUPPLY_USB_TYPE_DCP:
		case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
		case QTI_POWER_SUPPLY_USB_TYPE_HVDCP:
		case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3:
		case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5:
			desc->type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case POWER_SUPPLY_USB_TYPE_CDP:
			desc->type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case POWER_SUPPLY_USB_TYPE_ACA:
			desc->type = POWER_SUPPLY_TYPE_USB_ACA;
			break;
		case POWER_SUPPLY_USB_TYPE_C:
			desc->type = POWER_SUPPLY_TYPE_USB_TYPE_C;
			break;
		case POWER_SUPPLY_USB_TYPE_PD:
		case POWER_SUPPLY_USB_TYPE_PD_DRP:
		case POWER_SUPPLY_USB_TYPE_PD_PPS:
			desc->type = POWER_SUPPLY_TYPE_USB_PD;
			break;
		default:
			desc->type = POWER_SUPPLY_TYPE_USB;
			break;
		}
	}
}

static void battery_chg_check_status_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev,
					battery_check_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	struct psy_state *usb_pst = &bcdev->psy_list[PSY_TYPE_USB];
	struct psy_state *wireless_pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int rc;

	if(bcdev->report_power_absent && pst->prop[BATT_CAPACITY] >= 100)
		bcdev->report_power_absent = false;

	rc = read_property_id(bcdev, usb_pst, USB_ONLINE);
	if (rc < 0) {
		pr_err("Failed to read USB_ONLINE, rc=%d\n", rc);
		return;
	}

	rc = read_property_id(bcdev, wireless_pst, WLS_ONLINE);
	if (rc < 0) {
		pr_debug("Failed to read WLS_ONLINE, rc=%d\n", rc);
		return;
	}

	if (usb_pst->prop[USB_ONLINE] == 0 && wireless_pst->prop[WLS_ONLINE] == 0) {
		pr_debug("usb and wireless is not online\n");
		return;
	}

	rc = read_property_id(bcdev, pst, BATT_CAPACITY);
	if (rc < 0) {
		pr_err("Failed to read BATT_CAPACITY, rc=%d\n", rc);
		return;
	}

	if (DIV_ROUND_CLOSEST(pst->prop[BATT_CAPACITY], 100) > 0) {
		pr_debug("Battery SOC is > 0\n");
		return;
	}
	pr_err("battery_chg_check_status_work, report_power_absent=%d, glink_crash_count %d\n",
               bcdev->report_power_absent, bcdev->glink_crash_count);

	rc = read_property_id(bcdev, pst, BATT_VOLT_NOW);
	if (rc < 0) {
		pr_err("Failed to read BATT_VOLT_NOW, rc=%d\n", rc);
		return;
	}

	if (pst->prop[BATT_VOLT_NOW] / 1000 > bcdev->shutdown_volt_mv) {
		pr_debug("Battery voltage is > %d mV\n",
			bcdev->shutdown_volt_mv);
		return;
	}

	pr_emerg("Initiating a shutdown in 100 ms\n");
	msleep(100);

	bcdev->report_power_absent = true;
}

static void pen_charge_notifier_work(struct work_struct *work)
{
	int rc;
	int pen_charge_connect;
	static int pen_charge_connect_last_time  = -1;
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, pen_notifier_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];


	rc = read_property_id(bcdev, pst, XM_PROP_PEN_HALL3);
	if (rc < 0) {
		printk(KERN_ERR "%s:read_property_id XM_PROP_PEN_HALL3 err\n", __func__);
		return;
	}
	printk("%s:XM_PROP_PEN_HALL3 is %d\n", __func__, pst->prop[XM_PROP_PEN_HALL3]);

	rc = read_property_id(bcdev, pst, XM_PROP_PEN_HALL4);
	if (rc < 0) {
		printk(KERN_ERR "%s:read_property_id XM_PROP_PEN_HALL4 err\n", __func__);
		return;
	}
	printk("%s:XM_PROP_PEN_HALL4 is %d\n", __func__, pst->prop[XM_PROP_PEN_HALL4]);

	rc = read_property_id(bcdev, pst, XM_PROP_REVERSE_CHG_STATE);
	if (rc < 0) {
		printk(KERN_ERR "%s:read_property_id XM_PROP_REVERSE_CHG_STATE err\n", __func__);
		return;
	}
	printk("%s:XM_PROP_REVERSE_CHG_STATE is %d\n", __func__, pst->prop[XM_PROP_REVERSE_CHG_STATE]);

	pen_charge_connect = !(!!pst->prop[XM_PROP_PEN_HALL3] & !!pst->prop[XM_PROP_PEN_HALL4]);

	if (pen_charge_connect == 1) {
		if (pst->prop[XM_PROP_REVERSE_CHG_STATE] != 4 && pst->prop[XM_PROP_REVERSE_CHG_STATE] != 2)
			pen_charge_connect = 0;
	}

	if(pen_charge_connect_last_time != pen_charge_connect) {
		atomic_notifier_call_chain(&pen_charge_state_notifier, pen_charge_connect, NULL);
	} else {
		printk("%s:pen_charge_connect is %d, pen_charge_connect_last_time is %d, skip call chain\n", __func__, pen_charge_connect, pen_charge_connect_last_time);
	}
	pen_charge_connect_last_time = pen_charge_connect;
}

int pen_charge_state_notifier_register_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&pen_charge_state_notifier, nb);
}
EXPORT_SYMBOL(pen_charge_state_notifier_register_client);

int pen_charge_state_notifier_unregister_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&pen_charge_state_notifier, nb);
}
EXPORT_SYMBOL(pen_charge_state_notifier_unregister_client);

static void usb_chg_lpd_check_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
						struct battery_chg_dev,
						lpd_status_work);
	bool val = false;
	int rc = 0;

	if(!bcdev->blank_state)
		val = true;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_USB],
				USB_MOISTURE_DET_EN, val);
	if (rc < 0) {
		pr_err("lpd write failed %d\n", rc);
		return;
	}

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			XM_PROP_FB_BLANK_STATE, bcdev->blank_state);
	if (rc < 0) {
			pr_err("blank_state write failed %d\n", rc);
			return;
	}

	pr_err("blank_state = %d,enable lpd = %d\n", bcdev->blank_state, val);

}

#define BATT_UPDATE_PERIOD_10S		10
#define BATT_UPDATE_PERIOD_20S		20
#define BATT_UPDATE_PERIOD_8S		  8
#define BATT_UPDATE_PERIOD_5S		  5
static void xm_batt_update_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, batt_update_work.work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct psy_state *batt_pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	struct psy_state *usb_pst = &bcdev->psy_list[PSY_TYPE_USB];
	int interval = BATT_UPDATE_PERIOD_10S;
	int rc = 0;
	static u32 last_capacity = 0;

	rc = read_property_id(bcdev, batt_pst, BATT_CAPACITY);
	if ((batt_pst->prop[BATT_CAPACITY] / 100) < 15)
		interval = BATT_UPDATE_PERIOD_8S;
	rc = read_property_id(bcdev, pst, XM_PROP_THERMAL_TEMP);
	if (bcdev->blank_state)
		interval = BATT_UPDATE_PERIOD_20S;

	if(bcdev->fast_update_temp) {
          rc = read_property_id(bcdev, usb_pst, USB_ONLINE);
          if (rc < 0) {
                  pr_err("Failed to read USB_ONLINE, rc=%d\n", rc);
          }

          if(usb_pst->prop[USB_ONLINE] == 1) {
          interval = BATT_UPDATE_PERIOD_5S;
          }
        }
	if (batt_pst->prop[BATT_CAPACITY] != last_capacity) {
		last_capacity = batt_pst->prop[BATT_CAPACITY];
		power_supply_changed(batt_pst->psy);
	}
	pr_err("batt_update_work: batt_temp:%d blank_state:%d,interval:%d,soc:%d\n",
		pst->prop[XM_PROP_THERMAL_TEMP] * 100, bcdev->blank_state,interval,batt_pst->prop[BATT_CAPACITY] / 100);
	schedule_delayed_work(&bcdev->batt_update_work, interval * HZ);
}

static void xm_glink_crash_num_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, glink_crash_num_work.work);

	pr_err("glink crash, try to wait message\n");
	bcdev->glink_crash_timeout_reset_flag = true;

}

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_notify_msg *notify_msg = data;
	struct psy_state *pst = NULL;
	u32 hboost_vmax_mv, notification;

	if (len != sizeof(*notify_msg)) {
		pr_err("Incorrect response length %zu\n", len);
		return;
	}

	notification = notify_msg->notification;
	if ((notification & 0xffff) == BC_HBOOST_VMAX_CLAMP_NOTIFY) {
		hboost_vmax_mv = (notification >> 16) & 0xffff;
		raw_notifier_call_chain(&hboost_notifier, VMAX_CLAMP, &hboost_vmax_mv);
		return;
	}

	switch (notification) {
	case BC_BATTERY_STATUS_GET:
	case BC_GENERIC_NOTIFY:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		if (bcdev->shutdown_volt_mv > 0)
			schedule_work(&bcdev->battery_check_work);
		break;
	case BC_USB_STATUS_GET(USB_1_PORT_ID):
		bcdev->usb_active[USB_1_PORT_ID] = true;
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		schedule_work(&bcdev->usb_type_work);
		break;
	case BC_USB_STATUS_GET(USB_2_PORT_ID):
		if (!bcdev->has_usb_2) {
			pr_debug("notification: %u for missing USB_2 port\n",
					notification);
			break;
		}
		bcdev->usb_active[USB_2_PORT_ID] = true;
		pst = &bcdev->psy_list[PSY_TYPE_USB_2];
		schedule_work(&bcdev->usb_type_work);
		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		break;
	case BC_XM_STATUS_GET:
		schedule_delayed_work(&bcdev->xm_prop_change_work, 0);
		schedule_work(&bcdev->pen_notifier_work);
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		break;
	case BC_XM_STATUS_UPLOAD:
		schedule_delayed_work(&bcdev->xm_smart_prop_update_work, msecs_to_jiffies(1000));
		break;
	default:
		break;
	}

	if (pst && pst->psy) {
		/*
		 * For charger mode, keep the device awake at least for 50 ms
		 * so that device won't enter suspend when a non-SDP charger
		 * is removed. This would allow the userspace process like
		 * "charger" to be able to read power supply uevents to take
		 * appropriate actions (e.g. shutting down when the charger is
		 * unplugged).
		 */
		power_supply_changed(pst->psy);
		if (!bcdev->reverse_chg_flag)
			pm_wakeup_dev_event(bcdev->dev, 50, true);
	}
}

static int battery_chg_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct battery_chg_dev *bcdev = priv;

	down_read(&bcdev->state_sem);

	if (!bcdev->initialized) {
		pr_debug("Driver initialization failed: Dropping glink callback message: state %d\n",
			 bcdev->state);
		up_read(&bcdev->state_sem);
		return 0;
	}

	if (hdr->opcode == BC_NOTIFY_IND)
		handle_notification(bcdev, data, len);
	else
		handle_message(bcdev, data, len);

	up_read(&bcdev->state_sem);

	return 0;
}

static int wls_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int prop_id, rc;

	pval->intval = -ENODATA;

	if (prop == POWER_SUPPLY_PROP_PRESENT) {
		pval->intval = bcdev->boost_mode;
		return 0;
	}

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		if (pval->intval == 1 && bcdev->report_power_absent)
			pval->intval = 0;
		if (bcdev->debug_work_en == 0 && pval->intval == 1)
			schedule_delayed_work(&bcdev->charger_debug_info_print_work, 5 * HZ);
	}

	return 0;
}

typedef enum {
	POWER_SUPPLY_USB_REAL_TYPE_HVDCP2=0x80,
	POWER_SUPPLY_USB_REAL_TYPE_HVDCP3=0x81,
	POWER_SUPPLY_USB_REAL_TYPE_HVDCP3P5=0x82,
	POWER_SUPPLY_USB_REAL_TYPE_USB_FLOAT=0x83,
	POWER_SUPPLY_USB_REAL_TYPE_HVDCP3_CLASSB=0x84,
}power_supply_usb_type;

enum power_supply_quick_charge_type {
	QUICK_CHARGE_NORMAL = 0,		/* Charging Power <= 10W */
	QUICK_CHARGE_FAST,			/* 10W < Charging Power <= 20W */
	QUICK_CHARGE_FLASH,			/* 20W < Charging Power <= 30W */
	QUICK_CHARGE_TURBE,			/* 30W < Charging Power <= 50W */
	QUICK_CHARGE_SUPER,			/* Charging Power > 50W */
	QUICK_CHARGE_MAX,
};


struct quick_charge {
	int adap_type;
	enum power_supply_quick_charge_type adap_cap;
};

struct quick_charge adapter_cap[11] = {
	{ POWER_SUPPLY_USB_TYPE_SDP,        QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_TYPE_DCP,    QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_TYPE_CDP,    QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_TYPE_ACA,    QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_REAL_TYPE_USB_FLOAT,  QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_TYPE_PD,       QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_USB_REAL_TYPE_HVDCP2,    QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_USB_REAL_TYPE_HVDCP3,  QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_USB_REAL_TYPE_HVDCP3_CLASSB,  QUICK_CHARGE_FLASH },
	{ POWER_SUPPLY_USB_REAL_TYPE_HVDCP3P5,  QUICK_CHARGE_FLASH },
	{0, 0},
};

#define ADAPTER_NONE              0x0
#define ADAPTER_XIAOMI_QC3_20W    0x9
#define ADAPTER_XIAOMI_PD_20W     0xa
#define ADAPTER_XIAOMI_CAR_20W    0xb
#define ADAPTER_XIAOMI_PD_30W     0xc
#define ADAPTER_VOICE_BOX_30W     0xd
#define ADAPTER_XIAOMI_PD_50W     0xe
#define ADAPTER_XIAOMI_PD_60W     0xf
#define ADAPTER_XIAOMI_PD_100W    0x10

static ssize_t quick_charge_type_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int i = 0,verify_digiest = 0;
	int rc;
	u8 result = QUICK_CHARGE_NORMAL;
	enum power_supply_usb_type		real_charger_type = 0;
	int		batt_health;
	int		batt_status;
	u32 power_max;
	u32 batt_auth;
	u32 bap_match;
	int connector_temp;

#if defined(CONFIG_MI_WIRELESS)
	struct power_supply *wls_psy = NULL;
	union power_supply_propval val = {0, };
	int wls_present = 0;
#endif

	rc = read_property_id(bcdev, pst, BATT_HEALTH);
	if (rc < 0)
		return rc;
	batt_health = pst->prop[BATT_HEALTH];
	rc = read_property_id(bcdev, pst, BATT_STATUS);
	if (rc < 0)
		return rc;
	batt_status = pst->prop[BATT_STATUS];
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(bcdev, pst, USB_REAL_TYPE);
	if (rc < 0)
		return rc;
	real_charger_type = pst->prop[USB_REAL_TYPE];

	pst = &bcdev->psy_list[PSY_TYPE_XM];
	rc = read_property_id(bcdev, pst, XM_PROP_PD_VERIFED);
	verify_digiest = pst->prop[XM_PROP_PD_VERIFED];

	rc = read_property_id(bcdev, pst, XM_PROP_POWER_MAX);
	power_max = pst->prop[XM_PROP_POWER_MAX];

	rc = read_property_id(bcdev, pst, XM_PROP_AUTHENTIC);
	batt_auth = pst->prop[XM_PROP_AUTHENTIC];

	rc = read_property_id(bcdev, pst, XM_PROP_BATTERY_ADAPT_POWER_MATCH);
	bap_match = pst->prop[XM_PROP_BATTERY_ADAPT_POWER_MATCH];

	rc = read_property_id(bcdev, pst, XM_PROP_CONNECTOR_TEMP);
	connector_temp = pst->prop[XM_PROP_CONNECTOR_TEMP];
	if ((batt_health == POWER_SUPPLY_HEALTH_COLD) || (batt_health == POWER_SUPPLY_HEALTH_HOT) || (batt_auth == 0) || (bap_match == 0) ||
		((connector_temp > 600) && (batt_status == POWER_SUPPLY_STATUS_DISCHARGING))){
				result = QUICK_CHARGE_NORMAL;
	} else if (real_charger_type == POWER_SUPPLY_USB_TYPE_PD_PPS && verify_digiest ==1) {
		if(power_max >= 50)
			result = QUICK_CHARGE_SUPER;
		else
			result = QUICK_CHARGE_TURBE;
		}
	else if (real_charger_type == POWER_SUPPLY_USB_TYPE_PD_PPS)
		result = QUICK_CHARGE_FAST;
	else {
		while (adapter_cap[i].adap_type != 0) {
			if (real_charger_type == adapter_cap[i].adap_type) {
				result = adapter_cap[i].adap_cap;
			}
			i++;
		}
	}

#if defined(CONFIG_MI_WIRELESS)
	wls_psy = bcdev->psy_list[PSY_TYPE_WLS].psy;
	if (wls_psy != NULL) {
	rc = wls_psy_get_prop(wls_psy, POWER_SUPPLY_PROP_ONLINE, &val);
	if (!rc)
	      wls_present = val.intval;
	else
	      wls_present = 0;
	}
	if(wls_present) {
		if(power_max >= 30) {
			result = QUICK_CHARGE_SUPER;
		}
		else if(power_max == 20) {
			result = QUICK_CHARGE_FLASH;
		}
		else {
			result = QUICK_CHARGE_NORMAL;
		}
	}
#endif

	return scnprintf(buf, PAGE_SIZE, "%u", result);
}
static CLASS_ATTR_RO(quick_charge_type);

static ssize_t apdo_max_show(struct class *c,
                                        struct class_attribute *attr, char *buf)
{
        struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
                                                battery_class);
        struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
        int rc;

        rc = read_property_id(bcdev, pst, XM_PROP_APDO_MAX);
        if (rc < 0)
                return rc;

        return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_APDO_MAX]);
}
static CLASS_ATTR_RO(apdo_max);

static ssize_t soc_decimal_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SOC_DECIMAL);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_SOC_DECIMAL]);
}
static CLASS_ATTR_RO(soc_decimal);

static ssize_t soc_decimal_rate_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SOC_DECIMAL_RATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_SOC_DECIMAL_RATE]);
}
static CLASS_ATTR_RO(soc_decimal_rate);

static ssize_t smart_batt_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SMART_BATT, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t smart_batt_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SMART_BATT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SMART_BATT]);
}
static CLASS_ATTR_RW(smart_batt);

static ssize_t smart_fv_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SMART_FV, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t smart_fv_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SMART_FV);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SMART_FV]);
}
static CLASS_ATTR_RW(smart_fv);

static ssize_t night_charging_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;
	if (kstrtobool(buf, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_NIGHT_CHARGING, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t night_charging_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_NIGHT_CHARGING);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_NIGHT_CHARGING]);
}
static CLASS_ATTR_RW(night_charging);

static ssize_t usbinterface_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;
	if (kstrtobool(buf, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_USBINTERFACE, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t usbinterface_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_USBINTERFACE);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_USBINTERFACE]);
}
static CLASS_ATTR_RW(usbinterface);

void mievent_upload(int miev_code, int para_cnt,...)
{
	#if IS_ENABLED(CONFIG_MIEV)
    int i = 0;
	char* key;
	char* type;
	char* value_str;
	long val_long;

	
	va_list arg;

	struct misight_mievent *event = cdev_tevent_alloc(miev_code);
	va_start(arg,para_cnt);
	while (i < para_cnt) {
	    type = va_arg(arg, char*);
		key = va_arg(arg, char*);
		if (strcmp(type, "char") == 0) {
			value_str = va_arg(arg, char*);
			cdev_tevent_add_str(event, key, value_str);
		} else if (strcmp(type, "int") == 0) {
			val_long = va_arg(arg, int);
			cdev_tevent_add_int(event, key, val_long);
		} else {
			break;
		}
        i++;
	}

	va_end(arg);
	cdev_tevent_write(event);
	cdev_tevent_destroy(event);
	#endif

	return;
}

static void xm_handle_adsp_dfx_report(struct chg_dfx_report_msg *Context)
{
	u8 type = Context->type;

	if (type >= CHG_DFX_MAX_INDEX) {
		return;
	}

	switch (type) {
	case CHG_DFX_FG_IIC_ERR:
		mievent_upload(DFX_ID_CHG_FG_IIC_ERR, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_BATT_LINKER_ABSENT:
		mievent_upload(DFX_ID_CHG_BATT_LINKER_ABSENT, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_CP_TDIE:
		mievent_upload(DFX_ID_CHG_CP_TDIE, 3, "char", "chgStatInfo", xm_dfx_chg_report_text[type],
											"int", "masterTdie", Context->data[0], "int", "slaveTdie", Context->data[1]);
		break;
	case CHG_DFX_CP_ERR:
		mievent_upload(DFX_ID_CHG_CP_ERR, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_LPD_DISCHARGE:
		mievent_upload(DFX_ID_CHG_LPD_DISCHARGE, 1, "int", "lpdFlag", 1);
		break;
	case CHG_DFX_LPD_DISCHARGE_RESET:
		mievent_upload(DFX_ID_CHG_LPD_DISCHARGE, 1, "int","lpdFlag", 0);
		break;
	case CHG_DFX_CORROSION_DISCHARGE:
		mievent_upload(DFX_ID_CHG_CORROSION_DISCHARGE, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_NONE_STANDARD_CHG:
		mievent_upload(DFX_ID_CHG_NONE_STANDARD_CHG, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_RX_IIC_ERR:
		mievent_upload(DFX_ID_CHG_RX_IIC_ERR, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_CP_VBUS_OVP:
		mievent_upload(DFX_ID_CHG_CP_VBUS_OVP, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_CP_IBUS_OCP:
		mievent_upload(DFX_ID_CHG_CP_IBUS_OCP, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_CP_VBAT_OVP:
		mievent_upload(DFX_ID_CHG_CP_VBAT_OVP, 2, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
				                        	"int", "vbat", Context->data[0]);
		break;
	case CHG_DFX_CP_IBAT_OCP:
		mievent_upload(DFX_ID_CHG_CP_IBAT_OCP, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_PPS_AUTH_FAIL:
		mievent_upload(DFX_ID_CHG_PD_AUTHEN_FAIL, 2, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
				                            "int", "adapterId", Context->data[0]);
		break;
	case CHG_DFX_CP_ENABLE_FAIL:
		mievent_upload(DFX_ID_CHG_CP_ENABLE_FAIL, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_BATTERY_CYCLECOUNT:
		mievent_upload(DFX_ID_CHG_BATTERY_CYCLECOUNT, 2, "char", "chgStatInfo", xm_dfx_chg_report_text[type],
		                                   "int", "cycleCnt", Context->data[0]);
		break;
	case CHG_DFX_SMART_ENDURANCE:
		mievent_upload(DFX_ID_CHG_SMART_ENDURANCE, 2, "char", "chgStatInfo", xm_dfx_chg_report_text[type],
							"int", "soc", Context->data[0]);
		break;
	case CHG_DFX_LOW_TEMP_DISCHARGING:
		mievent_upload(DFX_ID_CHG_LOW_TEMP_DISCHARGING, 2, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
				                            "int", "tbat", Context->data[0]);
		break;
	case CHG_DFX_HIGH_TEMP_DISCHARGING:
		mievent_upload(DFX_ID_CHG_HIGH_TEMP_DISCHARGING, 2, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
				                            "int", "tbat", Context->data[0]);
		break;
	case CHG_DFX_DUAL_BATTERY_MISSING:
		mievent_upload(DFX_ID_CHG_DUAL_BATTERY_MISSING, 2, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
											"int", "status", Context->data[0]);
		break;
	case CHG_DFX_DUAL_BATTERY_AUTH_FAIL:
		mievent_upload(DFX_ID_CHG_DUAL_BATTERY_AUTH_FAIL, 2, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
				                        	"int", "status", Context->data[0]);
		break;
	case CHG_DFX_BATTERY_TEMP_HIGH:
		mievent_upload(DFX_ID_CHG_BATTERY_TEMP_HIGH, 3, "char", "chgStatInfo", xm_dfx_chg_report_text[type],
											"int", "tbat", Context->data[0], "int", "isCharging", Context->data[1]);
		break;
	case CHG_DFX_BATTERY_TEMP_LOW:
		mievent_upload(DFX_ID_CHG_BATTERY_TEMP_LOW, 3, "char", "chgStatInfo", xm_dfx_chg_report_text[type],
				                            "int", "tbat", Context->data[0], "int", "isCharging", Context->data[1]);
		break;
	case CHG_DFX_ANTIBURN_FAIL:
		mievent_upload(DFX_ID_CHG_ANTIBURN_FAIL, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_ANTIBURN:
		mievent_upload(DFX_ID_CHG_ANTIBURN, 2, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
											"int", "tconn", Context->data[0]);
		break;
	case CHG_DFX_BATTERY_VOLTAGE_DIFFERENCE:
		mievent_upload(DFX_ID_CHG_BATTERY_VOLTAGE_DIFFERENCE, 3, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
				                            "int", "chgBaseBattVol", Context->data[0], "int", "chgFlipBattVol", Context->data[1]);
		break;
	case CHG_DFX_WLS_FASTCHG_FAIL:
		mievent_upload(DFX_ID_CHG_WLS_FASTCHG_FAIL, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_WLS_FOD_LOW_POWER:
		mievent_upload(DFX_ID_CHG_WLS_FOD_LOW_POWER, 3, "char", "chgErrInfo", xm_dfx_chg_report_text[type],
				"int", "chgQBase", Context->data[0], "int", "chgQReal", Context->data[1]);
		break;
	case CHG_DFX_WLS_RX_OTP:
		mievent_upload(DFX_ID_CHG_WLS_RX_OTP, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_WLS_RX_OVP:
		mievent_upload(DFX_ID_CHG_WLS_RX_OVP, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_WLS_RX_OCP:
		mievent_upload(DFX_ID_CHG_WLS_RX_OCP, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_WLS_TRX_FOD:
		mievent_upload(DFX_ID_CHG_WLS_TRX_FOD, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_WLS_TRX_OCP:
		mievent_upload(DFX_ID_CHG_WLS_TRX_OCP, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_WLS_TRX_UVLO:
		mievent_upload(DFX_ID_CHG_WLS_TRX_UVLO, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	case CHG_DFX_WLS_TRX_IIC_ERR:
		mievent_upload(DFX_ID_CHG_WLS_TRX_IIC_ERR, 1, "char", "chgErrInfo", xm_dfx_chg_report_text[type]);
		break;
	default:
		;
	}
}

static void xm_handle_smartchg_scene_upload(struct battery_chg_dev *bcdev, int scene)
{
	char uevent_buf[MAX_STR_LEN] = { 0 };
	char *envp[2] = { uevent_buf, NULL };
	int ret = 0;
	struct psy_state *usb_pst = &bcdev->psy_list[PSY_TYPE_USB];
	int upload_scene = 0;
	static int last_upload_scene = 0;

	switch (scene) {
	case SMART_CHG_SCENE_PHONE:
	case SMART_CHG_SCENE_VIDEOCHAT:
	case SMART_CHG_SCENE_VIDEO:
	case SMART_CHG_SCENE_PER_VIDEO:
		upload_scene = scene;
		break;
	default:
		upload_scene = 0;
		break;
	}

	if (upload_scene != last_upload_scene) {
		ret = read_property_id(bcdev, usb_pst, USB_ONLINE);
		if (ret < 0) {
			pr_err("Failed to read USB_ONLINE, ret=%d\n", ret);
			return;
		}
		if (usb_pst->prop[USB_ONLINE] == 1) {
			snprintf(uevent_buf, MAX_STR_LEN, "MCA_SMARTCHG_SCENE=%d", upload_scene);
			last_upload_scene = upload_scene;
			ret = kobject_uevent_env(&bcdev->dev->kobj, KOBJ_CHANGE, envp);
			if (ret < 0)
				pr_err("notify uevent fail, ret=%d\n", ret);
		}
	}
}

static int wls_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	return 0;
}

static int wls_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	return 0;
}

static enum power_supply_property wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT,
};

static const struct power_supply_desc wls_psy_desc = {
	.name			= "wireless",
	.type			= POWER_SUPPLY_TYPE_WIRELESS,
	.properties		= wls_props,
	.num_properties		= ARRAY_SIZE(wls_props),
	.get_property		= wls_psy_get_prop,
	.set_property		= wls_psy_set_prop,
	.property_is_writeable	= wls_psy_prop_is_writeable,
};

static const char *get_wls_type_name(u32 wls_type)
{
	if (wls_type >= ARRAY_SIZE(qc_power_supply_wls_type_text))
		return "Unknown";

	return qc_power_supply_wls_type_text[wls_type];
}

static const char *get_usb_type_name(u32 usb_type)
{
	u32 i;

	if (usb_type >= QTI_POWER_SUPPLY_USB_TYPE_HVDCP &&
	    usb_type <= QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3_CLASSB) {
		for (i = 0; i < ARRAY_SIZE(qc_power_supply_usb_type_text);
		     i++) {
			if (i == (usb_type - QTI_POWER_SUPPLY_USB_TYPE_HVDCP))
				return qc_power_supply_usb_type_text[i];
		}
		return "Unknown";
	}

	for (i = 0; i < ARRAY_SIZE(power_supply_usb_type_text); i++) {
		if (i == usb_type)
			return power_supply_usb_type_text[i];
	}

	return "Unknown";
}

static int usb_psy_set_icl(struct battery_chg_dev *bcdev,
		struct psy_state *pst, u32 prop_id, int val)
{
	u32 temp;
	int rc;

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		return rc;
	}

	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_CDP:
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Input current limit (ICL) can be set by different clients. E.g. USB
	 * driver can request for a current of 500/900 mA depending on the
	 * port type. Also, clients like EUD driver can pass 0 or -22 to
	 * suspend or unsuspend the input for its use case.
	 */

	temp = val;
	if (val < 0)
		temp = UINT_MAX;

	rc = write_property_id(bcdev, pst, prop_id, temp);
	if (rc < 0) {
		;
	} else {

		if (!strcmp(pst->psy->desc->name, "usb-2"))
			bcdev->usb_icl_ua[USB_2_PORT_ID] = temp;
		else
			bcdev->usb_icl_ua[USB_1_PORT_ID] = temp;
	}

	return rc;
}

static int usb_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst;
	int prop_id, rc;

	if (!strcmp(psy->desc->name, "usb-2"))
		pst = &bcdev->psy_list[PSY_TYPE_USB_2];
	else
		pst = &bcdev->psy_list[PSY_TYPE_USB];

	pval->intval = -ENODATA;
	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];
	if (prop == POWER_SUPPLY_PROP_TEMP)
		pval->intval = DIV_ROUND_CLOSEST((int)pval->intval, 10);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
			if (pval->intval == 1 && bcdev->report_power_absent)
			pval->intval = 0;
 		if (bcdev->debug_work_en == 0 && pval->intval == 1)
 			schedule_delayed_work(&bcdev->charger_debug_info_print_work, 5 * HZ);
 	}

	return 0;
}

static int usb_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst;
	int prop_id, rc = 0;

	if (!strcmp(psy->desc->name, "usb-2"))
		pst = &bcdev->psy_list[PSY_TYPE_USB_2];
	else
		pst = &bcdev->psy_list[PSY_TYPE_USB];

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = usb_psy_set_icl(bcdev, pst, prop_id, pval->intval);
		break;
	default:
		break;
	}

	return rc;
}

static int usb_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_usb_type usb_psy_supported_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
};

static struct power_supply_desc usb_psy_desc[NUM_USB_PORTS] = {
	[USB_1_PORT_ID] = {
		.name			= "usb",
		.type			= POWER_SUPPLY_TYPE_USB,
		.properties		= usb_props,
		.num_properties		= ARRAY_SIZE(usb_props),
		.get_property		= usb_psy_get_prop,
		.set_property		= usb_psy_set_prop,
		.usb_types		= usb_psy_supported_types,
		.num_usb_types		= ARRAY_SIZE(usb_psy_supported_types),
		.property_is_writeable	= usb_psy_prop_is_writeable,
	},

	[USB_2_PORT_ID] = {
		.name			= "usb-2",
		.type			= POWER_SUPPLY_TYPE_USB,
		.properties		= usb_props,
		.num_properties		= ARRAY_SIZE(usb_props),
		.get_property		= usb_psy_get_prop,
		.set_property		= usb_psy_set_prop,
		.usb_types		= usb_psy_supported_types,
		.num_usb_types		= ARRAY_SIZE(usb_psy_supported_types),
		.property_is_writeable	= usb_psy_prop_is_writeable,
	}
};

#define CHARGE_CTRL_START_THR_MIN	50
#define CHARGE_CTRL_START_THR_MAX	95
#define CHARGE_CTRL_END_THR_MIN		55
#define CHARGE_CTRL_END_THR_MAX		100
#define CHARGE_CTRL_DELTA_SOC		5

static int battery_psy_set_charge_threshold(struct battery_chg_dev *bcdev,
					u32 target_soc, u32 delta_soc)
{
	struct battery_charger_chg_ctrl_msg msg = { { 0 } };
	int rc;

	if (!bcdev->chg_ctrl_en)
		return 0;

	if (target_soc > CHARGE_CTRL_END_THR_MAX)
		target_soc = CHARGE_CTRL_END_THR_MAX;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_CHG_CTRL_LIMIT_EN;
	msg.enable = 1;
	msg.target_soc = target_soc;
	msg.delta_soc = delta_soc;

	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_err("Failed to set charge_control thresholds, rc=%d\n", rc);
	else
		;

	return rc;
}

static int battery_psy_set_charge_end_threshold(struct battery_chg_dev *bcdev,
					int val)
{
	u32 delta_soc = CHARGE_CTRL_DELTA_SOC;
	int rc;

	if (val < CHARGE_CTRL_END_THR_MIN ||
	    val > CHARGE_CTRL_END_THR_MAX) {
		pr_err("Charge control end_threshold should be within [%u %u]\n",
			CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);
		return -EINVAL;
	}

	if (bcdev->chg_ctrl_start_thr && val > bcdev->chg_ctrl_start_thr)
		delta_soc = val - bcdev->chg_ctrl_start_thr;

	rc = battery_psy_set_charge_threshold(bcdev, val, delta_soc);
	if (rc < 0)
		pr_err("Failed to set charge control end threshold %u, rc=%d\n",
			val, rc);
	else
		bcdev->chg_ctrl_end_thr = val;

	return rc;
}

static int battery_psy_set_charge_start_threshold(struct battery_chg_dev *bcdev,
					int val)
{
	u32 target_soc, delta_soc;
	int rc;

	if (val < CHARGE_CTRL_START_THR_MIN ||
	    val > CHARGE_CTRL_START_THR_MAX) {
		pr_err("Charge control start_threshold should be within [%u %u]\n",
			CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);
		return -EINVAL;
	}

	if (val > bcdev->chg_ctrl_end_thr) {
		target_soc = val +  CHARGE_CTRL_DELTA_SOC;
		delta_soc = CHARGE_CTRL_DELTA_SOC;
	} else {
		target_soc = bcdev->chg_ctrl_end_thr;
		delta_soc = bcdev->chg_ctrl_end_thr - val;
	}

	rc = battery_psy_set_charge_threshold(bcdev, target_soc, delta_soc);
	if (rc < 0)
		pr_err("Failed to set charge control start threshold %u, rc=%d\n",
			val, rc);
	else
		bcdev->chg_ctrl_start_thr = val;

	return rc;
}

static int get_charge_control_en(struct battery_chg_dev *bcdev)
{
	int rc;

	rc = read_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_EN);
	if (rc < 0)
		pr_err("Failed to read the CHG_CTRL_EN, rc = %d\n", rc);
	else
		bcdev->chg_ctrl_en =
			bcdev->psy_list[PSY_TYPE_BATTERY].prop[BATT_CHG_CTRL_EN];

	return rc;
}

static int __battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					u32 fcc_ua)
{
	int rc;

	if (bcdev->restrict_chg_en) {
		fcc_ua = min_t(u32, fcc_ua, bcdev->restrict_fcc_ua);
		fcc_ua = min_t(u32, fcc_ua, bcdev->thermal_fcc_ua);
	}

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_LIM, fcc_ua);
	if (rc < 0) {
		;
	} else {
		bcdev->last_fcc_ua = fcc_ua;
	}

	return rc;
}

static int battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					int val)
{
	int rc;
	struct psy_state *pst = NULL;

	pst = &bcdev->psy_list[PSY_TYPE_XM];

	if (!bcdev->num_thermal_levels)
		return 0;

	if (bcdev->num_thermal_levels < 0) {
		return -EINVAL;
	}

	if (val < 0 || val > bcdev->num_thermal_levels)
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
			BATT_CHG_CTRL_LIM, val);

	bcdev->curr_thermal_level = val;

	return rc;
}

static int battery_psy_set_fcc(struct battery_chg_dev *bcdev, u32 prop_id, int val)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	u32 temp;
	int rc;

	temp = val;
	if (val < 0)
		temp = UINT_MAX;

	rc = write_property_id(bcdev, pst, prop_id, temp);

	return rc;
}

static bool check_batt_capacity_whether_glink_timeout(struct power_supply *psy)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc, vbat_mv;

	rc = read_property_id(bcdev, pst, BATT_VOLT_NOW);
	if (rc < 0) {
		vbat_mv = 3700;
	} else {
		vbat_mv = pst->prop[BATT_VOLT_NOW] / 1000;
	}
	if (vbat_mv > bcdev->shutdown_volt_mv + 50 && bcdev->read_capacity_timeout)
		return true;
	else
		return false;
}

static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int prop_id, rc;

	pval->intval = -ENODATA;

	if (prop == POWER_SUPPLY_PROP_TIME_TO_FULL_NOW)
		prop = POWER_SUPPLY_PROP_TIME_TO_FULL_AVG;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		pval->strval = pst->model;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = pst->prop[prop_id] / 100;
		if (pval->intval > 0)
			bcdev->last_capacity = pval->intval;
		if (!pval->intval) {
			if (check_batt_capacity_whether_glink_timeout(psy))
				pval->intval = bcdev->last_capacity;
		}
		if (bcdev->fake_soc >= 0 && bcdev->fake_soc <= 100)
			pval->intval = bcdev->fake_soc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 10);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		pval->intval = bcdev->curr_thermal_level;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		pval->intval = bcdev->num_thermal_levels;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = pst->prop[prop_id];
		if ((pval->intval == POWER_SUPPLY_STATUS_CHARGING && bcdev->report_power_absent)
                    || bcdev->glink_crash_count > 3)
			pval->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		pval->intval = pst->prop[prop_id];
		break;
	}

	return rc;
}

static int battery_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst =&bcdev->psy_list[PSY_TYPE_BATTERY];
	int prop_id, rc = 0;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return battery_psy_set_charge_current(bcdev, pval->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		rc = battery_psy_set_fcc(bcdev, prop_id, pval->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		return battery_psy_set_charge_start_threshold(bcdev,
								pval->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return battery_psy_set_charge_end_threshold(bcdev,
								pval->intval);
	default:
		return -EINVAL;
	}

	return 0;
}

static int battery_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
};

static int power_supply_read_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev = NULL;
	struct psy_state *pst = NULL;
	int batt_temp;
	static int last_temp = 0;
	ktime_t time_now;
	static ktime_t last_read_time;
	s64 delta;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	bcdev = power_supply_get_drvdata(psy);
	pst = &bcdev->psy_list[PSY_TYPE_XM];

	time_now = ktime_get();
	delta = ktime_ms_delta(time_now, last_read_time);


	batt_temp = pst->prop[XM_PROP_THERMAL_TEMP];
	last_read_time = time_now;

	*temp = batt_temp * 100;
	if (batt_temp!= last_temp) {
		last_temp = batt_temp;
	}
	return 0;
}

static struct thermal_zone_device_ops psy_tzd_ops = {
	.get_temp = power_supply_read_temp,
};

static const struct power_supply_desc batt_psy_desc = {
	.name			= "battery",
	.no_thermal		= true,
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= battery_props,
	.num_properties		= ARRAY_SIZE(battery_props),
	.get_property		= battery_psy_get_prop,
	.set_property		= battery_psy_set_prop,
	.property_is_writeable	= battery_psy_prop_is_writeable,
};

static int battery_chg_init_psy(struct battery_chg_dev *bcdev)
{
	struct power_supply_config psy_cfg = {};
	int rc;
	struct power_supply *psy;

	psy_cfg.drv_data = bcdev;
	psy_cfg.of_node = bcdev->dev->of_node;

	bcdev->psy_list[PSY_TYPE_BATTERY].psy =
		devm_power_supply_register(bcdev->dev, &batt_psy_desc,
						&psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy);
		bcdev->psy_list[PSY_TYPE_BATTERY].psy = NULL;
		return rc;
	}

	psy = bcdev->psy_list[PSY_TYPE_BATTERY].psy;
	psy->tzd = thermal_zone_device_register(psy->desc->name,
					0, 0, psy, &psy_tzd_ops, NULL, 0, 0);

	bcdev->psy_list[PSY_TYPE_USB].psy =
		devm_power_supply_register(bcdev->dev, &usb_psy_desc[USB_1_PORT_ID], &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_USB].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_USB].psy);
		bcdev->psy_list[PSY_TYPE_USB].psy = NULL;
		return rc;
	}

	if (bcdev->has_usb_2) {
		bcdev->psy_list[PSY_TYPE_USB_2].psy =
			devm_power_supply_register(bcdev->dev,
				&usb_psy_desc[USB_2_PORT_ID], &psy_cfg);
		if (IS_ERR(bcdev->psy_list[PSY_TYPE_USB_2].psy)) {
			rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_USB_2].psy);
			bcdev->psy_list[PSY_TYPE_USB_2].psy = NULL;
			return rc;
		}
	}

	bcdev->psy_list[PSY_TYPE_WLS].psy =
		devm_power_supply_register(bcdev->dev, &wls_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy);
		bcdev->psy_list[PSY_TYPE_WLS].psy = NULL;
		return rc;
	}


	return 0;
}

static void battery_chg_subsys_up_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, subsys_up_work);
	struct psy_state *pst;
	int rc;

	battery_chg_notify_enable(bcdev);

	msleep(200);

	if (bcdev->last_fcc_ua) {
		rc = __battery_psy_set_charge_current(bcdev,
				bcdev->last_fcc_ua);
		if (rc < 0)
			;
	}

	if (bcdev->usb_icl_ua[USB_1_PORT_ID]) {
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		rc = usb_psy_set_icl(bcdev, pst, USB_INPUT_CURR_LIMIT,
				bcdev->usb_icl_ua[USB_1_PORT_ID]);
		if (rc < 0)
			;
	}

	if (bcdev->has_usb_2 && bcdev->usb_icl_ua[USB_2_PORT_ID]) {
		pst = &bcdev->psy_list[PSY_TYPE_USB_2];
		rc = usb_psy_set_icl(bcdev, pst, USB_INPUT_CURR_LIMIT,
				bcdev->usb_icl_ua[USB_2_PORT_ID]);
		if (rc < 0)
			;
	}
}

static int wireless_fw_send_firmware(struct battery_chg_dev *bcdev,
					const struct firmware *fw)
{
	struct wireless_fw_push_buf_req msg = {};
	const u8 *ptr;
	u32 i, num_chunks, partial_chunk_size;
	int rc;

	num_chunks = fw->size / WLS_FW_BUF_SIZE;
	partial_chunk_size = fw->size % WLS_FW_BUF_SIZE;

	if (!num_chunks)
		return -EINVAL;

	ptr = fw->data;
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_WLS_FW_PUSH_BUF_REQ;

	for (i = 0; i < num_chunks; i++, ptr += WLS_FW_BUF_SIZE) {
		msg.fw_chunk_id = i + 1;
		memcpy(msg.buf, ptr, WLS_FW_BUF_SIZE);

		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	if (partial_chunk_size) {
		msg.fw_chunk_id = i + 1;
		memset(msg.buf, 0, WLS_FW_BUF_SIZE);
		memcpy(msg.buf, ptr, partial_chunk_size);

		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int wireless_fw_check_for_update(struct battery_chg_dev *bcdev,
					u32 version, size_t size)
{
	struct wireless_fw_check_req req_msg = {};

	bcdev->wls_fw_update_reqd = false;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_CHECK_UPDATE;
	req_msg.fw_version = version;
	req_msg.fw_size = size;
	req_msg.fw_crc = bcdev->wls_fw_crc;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

#define IDT9415_FW_MAJOR_VER_OFFSET		0x84
#define IDT9415_FW_MINOR_VER_OFFSET		0x86
#define IDT_FW_MAJOR_VER_OFFSET		0x94
#define IDT_FW_MINOR_VER_OFFSET		0x96
static int wireless_fw_update(struct battery_chg_dev *bcdev, bool force)
{
	const struct firmware *fw;
	struct psy_state *pst;
	u32 version;
	u16 maj_ver, min_ver;
	int rc;

	if (!bcdev->wls_fw_name) {
		return -EINVAL;
	}

	pm_stay_awake(bcdev->dev);

	/*
	 * Check for USB presence. If nothing is connected, check whether
	 * battery SOC is at least 50% before allowing FW update.
	 */
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(bcdev, pst, USB_ONLINE);
	if (rc < 0)
		goto out;

	if (bcdev->has_usb_2 && !pst->prop[USB_ONLINE]) {
		pst = &bcdev->psy_list[PSY_TYPE_USB_2];
		rc = read_property_id(bcdev, pst, USB_ONLINE);
		if (rc < 0)
			goto out;
	}

	if (!pst->prop[USB_ONLINE]) {
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_CAPACITY);
		if (rc < 0)
			goto out;

		if ((pst->prop[BATT_CAPACITY] / 100) < 50) {
			rc = -EINVAL;
			goto out;
		}
	}

	rc = firmware_request_nowarn(&fw, bcdev->wls_fw_name, bcdev->dev);
	if (rc) {
		goto out;
	}

	if (!fw || !fw->data || !fw->size) {
		rc = -EINVAL;
		goto release_fw;
	}

	if (fw->size < SZ_16K) {
		rc = -EINVAL;
		goto release_fw;
	}

	if (strstr(bcdev->wls_fw_name, "9412")) {
		maj_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MAJOR_VER_OFFSET));
		min_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MINOR_VER_OFFSET));
	} else {
		maj_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT9415_FW_MAJOR_VER_OFFSET));
		min_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT9415_FW_MINOR_VER_OFFSET));
	}
	version = maj_ver << 16 | min_ver;

	if (force)
		version = UINT_MAX;

	rc = wireless_fw_check_for_update(bcdev, version, fw->size);
	if (rc < 0) {
		goto release_fw;
	}

	if (!bcdev->wls_fw_update_reqd) {
		goto release_fw;
	}

	/* Wait for IDT to be setup by charger firmware */
	msleep(WLS_FW_PREPARE_TIME_MS);

	reinit_completion(&bcdev->fw_update_ack);
	rc = wireless_fw_send_firmware(bcdev, fw);
	if (rc < 0) {
		goto release_fw;
	}

	rc = wait_for_completion_timeout(&bcdev->fw_update_ack,
				msecs_to_jiffies(bcdev->wls_fw_update_time_ms));
	if (!rc) {
		rc = -ETIMEDOUT;
		goto release_fw;
	} else {
		rc = 0;
	}

release_fw:
	bcdev->wls_fw_crc = 0;
	release_firmware(fw);
out:
	pm_relax(bcdev->dev);

	return rc;
}

static ssize_t qti_charger_ro_show(struct class *c,
				struct class_attribute *attr, char *buf,
				int psy_type, int prop_id)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[psy_type];
	int rc;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", (int)pst->prop[prop_id]);
}

#define QTI_CHARGER_RO_SHOW(name, psy_type, prop_id)			\
	static ssize_t name##_show(struct class *c,			\
					struct class_attribute *attr,	\
					char *buf)			\
	{								\
		return qti_charger_ro_show(c, attr, buf, psy_type, prop_id); \
	}								\
	static CLASS_ATTR_RO(name)					\

#define QTI_CHARGER_RW_SHOW(name, psy_type, prop_id)			\
	static ssize_t name##_show(struct class *c,			\
					struct class_attribute *attr,	\
					char *buf)			\
	{								\
		return qti_charger_ro_show(c, attr, buf, psy_type, prop_id); \
	}								\
	static CLASS_ATTR_RW(name)					\

#define QTI_CHARGER_RW_PROP_SHOW(name, field)				\
	static ssize_t name##_show(struct class *c,			\
					struct class_attribute *attr,	\
					char *buf)			\
	{								\
		struct battery_chg_dev *bcdev = container_of(c,		\
				struct battery_chg_dev, battery_class);	\
		return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->field);	\
	}								\
	static CLASS_ATTR_RW(name)					\

#define QTI_CHARGER_TYPE_RO_SHOW(name, psy, psy_type, prop_id)		\
	static ssize_t name##_show(struct class *c,			\
					struct class_attribute *attr,	\
					char *buf)			\
	{								\
		struct battery_chg_dev *bcdev = container_of(c,		\
				struct battery_chg_dev, battery_class);	\
		struct psy_state *pst = &bcdev->psy_list[psy_type];	\
		int rc;							\
									\
		rc = read_property_id(bcdev, pst, prop_id);		\
		if (rc < 0)						\
			return rc;					\
									\
		return scnprintf(buf, PAGE_SIZE, "%s\n",		\
				get_##psy##_type_name(pst->prop[prop_id]));	\
	}								\
	static CLASS_ATTR_RO(name)					\

static ssize_t wireless_fw_update_time_ms_store(struct class *c,
				struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtou32(buf, 0, &bcdev->wls_fw_update_time_ms))
		return -EINVAL;

	return count;
}

QTI_CHARGER_RW_PROP_SHOW(wireless_fw_update_time_ms, wls_fw_update_time_ms);

static ssize_t wireless_fw_crc_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	u16 val;

	if (kstrtou16(buf, 0, &val) || !val)
		return -EINVAL;

	bcdev->wls_fw_crc = val;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_crc);

static ssize_t wireless_fw_version_show(struct class *c,
					struct class_attribute *attr,
					char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct wireless_fw_get_version_req req_msg = {};
	int rc;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_GET_VERSION;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%#x\n", bcdev->wls_fw_version);
}
static CLASS_ATTR_RO(wireless_fw_version);

static ssize_t wireless_fw_force_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, true);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_force_update);

static ssize_t wireless_fw_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, false);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_update);

QTI_CHARGER_TYPE_RO_SHOW(wireless_type, wls, PSY_TYPE_WLS, WLS_ADAP_TYPE);

static ssize_t wls_debug_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
							battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_WLS_DEBUG;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	memset(req_msg.data, '\0', sizeof(req_msg.data));
	strncpy(req_msg.data, buf, count);

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t wls_debug_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_WLS_DEBUG;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s", bcdev->wls_debug_data);
}
static CLASS_ATTR_RW(wls_debug);

static ssize_t charge_control_en_store(struct class *c,
				struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	if (val == bcdev->chg_ctrl_en)
		return count;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_EN, val);
	if (rc < 0) {
		return rc;
	}

	bcdev->chg_ctrl_en = val;

	return count;
}

static ssize_t charge_control_en_show(struct class *c,
				struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;

	rc = get_charge_control_en(bcdev);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->chg_ctrl_en);
}
static CLASS_ATTR_RW(charge_control_en);

QTI_CHARGER_RO_SHOW(usb_typec_compliant, PSY_TYPE_USB, USB_TYPEC_COMPLIANT);

QTI_CHARGER_TYPE_RO_SHOW(usb_real_type, usb, PSY_TYPE_USB, USB_REAL_TYPE);

QTI_CHARGER_RO_SHOW(usb_2_typec_compliant, PSY_TYPE_USB_2, USB_TYPEC_COMPLIANT);

QTI_CHARGER_TYPE_RO_SHOW(usb_2_real_type, usb, PSY_TYPE_USB_2, USB_REAL_TYPE);

static ssize_t restrict_cur_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (kstrtou32(buf, 0, &fcc_ua) || fcc_ua > bcdev->thermal_fcc_ua)
		return -EINVAL;

	prev_fcc_ua = bcdev->restrict_fcc_ua;
	bcdev->restrict_fcc_ua = fcc_ua;
	if (bcdev->restrict_chg_en) {
		rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
		if (rc < 0) {
			bcdev->restrict_fcc_ua = prev_fcc_ua;
			return rc;
		}
	}

	return count;
}
QTI_CHARGER_RW_PROP_SHOW(restrict_cur, restrict_fcc_ua);

static ssize_t restrict_chg_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->restrict_chg_en = val;
	rc = __battery_psy_set_charge_current(bcdev, bcdev->restrict_chg_en ?
			bcdev->restrict_fcc_ua : bcdev->thermal_fcc_ua);
	if (rc < 0)
		return rc;

	return count;
}
QTI_CHARGER_RW_PROP_SHOW(restrict_chg, restrict_chg_en);

static ssize_t fake_soc_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	bcdev->fake_soc = val;

	if (pst->psy)
		power_supply_changed(pst->psy);

	return count;
}
QTI_CHARGER_RW_PROP_SHOW(fake_soc, fake_soc);

static ssize_t wireless_boost_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_WLS],
				WLS_BOOST_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

QTI_CHARGER_RW_SHOW(wireless_boost_en, PSY_TYPE_WLS, WLS_BOOST_EN);

static ssize_t _moisture_detection_en_store(struct class *c,
					struct class_attribute *attr, enum psy_type type,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[type],
				USB_MOISTURE_DET_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t moisture_detection_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	return _moisture_detection_en_store(c, attr, PSY_TYPE_USB, buf, count);
}

QTI_CHARGER_RW_SHOW(moisture_detection_en, PSY_TYPE_USB, USB_MOISTURE_DET_EN);

static ssize_t moisture_detection_usb_2_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	return _moisture_detection_en_store(c, attr, PSY_TYPE_USB_2, buf, count);
}

QTI_CHARGER_RW_SHOW(moisture_detection_usb_2_en, PSY_TYPE_USB_2, USB_MOISTURE_DET_EN);

static ssize_t moisture_detection_status_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_STS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d",
			pst->prop[USB_MOISTURE_DET_STS]);
}
static CLASS_ATTR_RO(moisture_detection_status);

QTI_CHARGER_RO_SHOW(moisture_detection_usb_2_status, PSY_TYPE_USB_2, USB_MOISTURE_DET_STS);

QTI_CHARGER_RO_SHOW(resistance, PSY_TYPE_BATTERY, BATT_RESISTANCE);

QTI_CHARGER_RO_SHOW(soh, PSY_TYPE_BATTERY, BATT_SOH);

static ssize_t ship_mode_en_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	int rc =0;

 	if (kstrtobool(buf, &bcdev->ship_mode_en))
 		return -EINVAL;
 
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHIP_MODE_REQ_SET;
	msg.ship_mode_type = SHIP_MODE_PMIC;
	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_err("Failed to write ship mode: %d\n", rc);

	return count;
}

static ssize_t ship_mode_en_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->ship_mode_en);
}
static CLASS_ATTR_RW(ship_mode_en);

static ssize_t cp_mode_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int val = 0, rc = 0;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
							XM_PROP_CP_MODE, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t cp_mode_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_CP_MODE);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CP_MODE]);
}
static CLASS_ATTR_RW(cp_mode);

static ssize_t bq2597x_chip_ok_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_CHIP_OK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_CHIP_OK]);
}
static CLASS_ATTR_RO(bq2597x_chip_ok);

static ssize_t bq2597x_slave_chip_ok_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_SLAVE_CHIP_OK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_SLAVE_CHIP_OK]);
}
static CLASS_ATTR_RO(bq2597x_slave_chip_ok);

static ssize_t bq2597x_bus_current_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BUS_CURRENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BUS_CURRENT]);
}
static CLASS_ATTR_RO(bq2597x_bus_current);

static ssize_t bq2597x_slave_bus_current_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_SLAVE_BUS_CURRENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_SLAVE_BUS_CURRENT]);
}
static CLASS_ATTR_RO(bq2597x_slave_bus_current);

static ssize_t bq2597x_bus_delta_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BUS_DELTA);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BUS_DELTA]);
}
static CLASS_ATTR_RO(bq2597x_bus_delta);

static ssize_t bq2597x_bus_voltage_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BUS_VOLTAGE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BUS_VOLTAGE]);
}
static CLASS_ATTR_RO(bq2597x_bus_voltage);

static ssize_t bq2597x_battery_present_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BATTERY_PRESENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BATTERY_PRESENT]);
}
static CLASS_ATTR_RO(bq2597x_battery_present);

static ssize_t bq2597x_slave_battery_present_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_SLAVE_BATTERY_PRESENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_SLAVE_BATTERY_PRESENT]);
}
static CLASS_ATTR_RO(bq2597x_slave_battery_present);

static ssize_t bq2597x_battery_voltage_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BATTERY_VOLTAGE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BATTERY_VOLTAGE]);
}
static CLASS_ATTR_RO(bq2597x_battery_voltage);

static ssize_t bq2597x_battery_temp_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BATTERY_TEMPERATURE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_BQ2597X_BATTERY_TEMPERATURE]);
}
static CLASS_ATTR_RO(bq2597x_battery_temp);

static ssize_t dam_ovpgate_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_DAM_OVPGATE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t dam_ovpgate_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_DAM_OVPGATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_DAM_OVPGATE]);
}
static CLASS_ATTR_RW(dam_ovpgate);

static ssize_t real_type_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_REAL_TYPE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s\n", get_usb_type_name(pst->prop[XM_PROP_REAL_TYPE]));
}
static CLASS_ATTR_RO(real_type);

static ssize_t battcont_online_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_BATT_CONNT_ONLINE);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_BATT_CONNT_ONLINE]);
}
static CLASS_ATTR_RO(battcont_online);

static ssize_t connector_temp_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
						XM_PROP_CONNECTOR_TEMP, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t connector_temp_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CONNECTOR_TEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_CONNECTOR_TEMP]);
}
static CLASS_ATTR_RW(connector_temp);

static ssize_t connector_temp2_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CONNECTOR_TEMP2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_CONNECTOR_TEMP2]);
}
static CLASS_ATTR_RO(connector_temp2);

static ssize_t authentic_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->battery_auth = val;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
						XM_PROP_AUTHENTIC, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t authentic_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_AUTHENTIC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_AUTHENTIC]);
}
static CLASS_ATTR_RW(authentic);

static ssize_t bap_match_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
						XM_PROP_BATTERY_ADAPT_POWER_MATCH, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t bap_match_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BATTERY_ADAPT_POWER_MATCH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BATTERY_ADAPT_POWER_MATCH]);
}
static CLASS_ATTR_RW(bap_match);

static ssize_t double85_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
						XM_PROP_DOUBLE85, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t double85_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_DOUBLE85);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_DOUBLE85]);
}
static CLASS_ATTR_RW(double85);

static ssize_t soh_new_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SOH_NEW);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_SOH_NEW]);
}
static CLASS_ATTR_RO(soh_new);

static ssize_t remove_temp_limit_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
						XM_PROP_REMOVE_TEMP_LIMIT, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t remove_temp_limit_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_REMOVE_TEMP_LIMIT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_REMOVE_TEMP_LIMIT]);
}
static CLASS_ATTR_RW(remove_temp_limit);

static int write_verify_digest_prop_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u8* buff)
{
	struct xm_verify_digest_resp_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;
	req_msg.slave_fg = bcdev->slave_fg_verify_flag;
	memcpy(req_msg.digest, buff, BATTERY_DIGEST_LEN);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_verify_digest_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct xm_verify_digest_resp_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;
	req_msg.slave_fg = bcdev->slave_fg_verify_flag;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static ssize_t verify_slave_flag_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->slave_fg_verify_flag = val;

	return count;
}

static ssize_t verify_slave_flag_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%u\n", bcdev->slave_fg_verify_flag);
}
static CLASS_ATTR_RW(verify_slave_flag);


static ssize_t verify_digest_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	u8 random_1s[BATTERY_DIGEST_LEN + 1] = {0};
	char kbuf_1s[70] = {0};
	int rc;
	int i;


	memset(kbuf_1s, 0, sizeof(kbuf_1s));
	strncpy(kbuf_1s, buf, count - 1);
	StringToHex(kbuf_1s, random_1s, &i);
	rc = write_verify_digest_prop_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			XM_PROP_VERIFY_DIGEST, random_1s);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t verify_digest_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	u8 digest_buf[4];
	int i;
	int len;

	rc = read_verify_digest_property_id(bcdev, pst, XM_PROP_VERIFY_DIGEST);
	if (rc < 0)
		return rc;

	for (i = 0; i < BATTERY_DIGEST_LEN; i++) {
		memset(digest_buf, 0, sizeof(digest_buf));
		snprintf(digest_buf, sizeof(digest_buf) - 1, "%02x", bcdev->digest[i]);
		strlcat(buf, digest_buf, BATTERY_DIGEST_LEN * 2 + 1);
	}
	len = strlen(buf);
	buf[len] = '\0';
	return strlen(buf) + 1;
}
static CLASS_ATTR_RW(verify_digest);

static ssize_t chip_ok_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CHIP_OK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CHIP_OK]);
}
static CLASS_ATTR_RO(chip_ok);

static ssize_t resistance_id_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RESISTANCE_ID);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_RESISTANCE_ID]);
}
static CLASS_ATTR_RO(resistance_id);

static ssize_t input_suspend_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
						XM_PROP_INPUT_SUSPEND, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t input_suspend_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_INPUT_SUSPEND);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_INPUT_SUSPEND]);
}
static CLASS_ATTR_RW(input_suspend);

static ssize_t fastchg_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FASTCHGMODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_FASTCHGMODE]);
}
static CLASS_ATTR_RO(fastchg_mode);

static ssize_t cc_orientation_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CC_ORIENTATION);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CC_ORIENTATION]);
}
static CLASS_ATTR_RO(cc_orientation);

static ssize_t thermal_remove_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_THERMAL_REMOVE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t thermal_remove_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_THERMAL_REMOVE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_THERMAL_REMOVE]);
}
static CLASS_ATTR_RW(thermal_remove);

static ssize_t typec_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_TYPEC_MODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s\n", power_supply_usbc_text[pst->prop[XM_PROP_TYPEC_MODE]]);
}
static CLASS_ATTR_RO(typec_mode);

static ssize_t mtbf_current_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	bcdev->mtbf_current = val;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_MTBF_CURRENT, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t mtbf_current_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_MTBF_CURRENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_MTBF_CURRENT]);
}
static CLASS_ATTR_RW(mtbf_current);

static ssize_t fake_temp_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FAKE_TEMP, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fake_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FAKE_TEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FAKE_TEMP]);
}
static CLASS_ATTR_RW(fake_temp);

/*test*/
static ssize_t fake_cycle_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
		struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
												battery_class);
		int rc;
		int val;
		if(kstrtoint(buf, 10, &val))
					return -EINVAL;
			rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
									XM_PROP_FAKE_CYCLE, val);
			if(rc < 0)
					return rc;
			return count;
}
static ssize_t fake_cycle_show(struct class *c,
                                        struct class_attribute *attr, char *buf)
{
        struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
                                                battery_class);
        struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
        int rc;

        rc = read_property_id(bcdev, pst, XM_PROP_FAKE_CYCLE);
        if (rc < 0)
                return rc;

        return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FAKE_CYCLE]);
}
static CLASS_ATTR_RW(fake_cycle);

static ssize_t afp_temp_show(struct class *c,
                                          struct class_attribute *attr, char *buf)
  {
  	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
  											battery_class);
  	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
  	int rc;

  	rc = read_property_id(bcdev, pst, XM_PROP_AFP_TEMP);
  	if (rc < 0)
  			return rc;

  	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_AFP_TEMP]);
  }
  static CLASS_ATTR_RO(afp_temp);

static ssize_t shutdown_delay_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SHUTDOWN_DELAY);
	if (rc < 0)
		return rc;

	if (!bcdev->shutdown_delay_en ||
	    (bcdev->fake_soc > 0 && bcdev->fake_soc <= 100))
		pst->prop[XM_PROP_SHUTDOWN_DELAY] = 0;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_SHUTDOWN_DELAY]);
}

static ssize_t shutdown_delay_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	return count;
}

static CLASS_ATTR_RW(shutdown_delay);

static ssize_t shipmode_count_reset_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SHIPMODE_COUNT_RESET, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t shipmode_count_reset_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SHIPMODE_COUNT_RESET);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SHIPMODE_COUNT_RESET]);
}
static CLASS_ATTR_RW(shipmode_count_reset);

#if defined(CONFIG_MI_N2N3ANTIBURN)
static ssize_t anti_burn_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_ANTI_BURN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_ANTI_BURN]);
}
static CLASS_ATTR_RO(anti_burn);
#endif

static ssize_t cc_short_vbus_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CC_SHORT_VBUS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_CC_SHORT_VBUS]);
}

static CLASS_ATTR_RO(cc_short_vbus);

static ssize_t otg_ui_support_show(struct class *c,
                                        struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
											battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_OTG_UI_SUPPORT);
	if (rc < 0)
			return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_OTG_UI_SUPPORT]);
}
static CLASS_ATTR_RO(otg_ui_support);

static ssize_t cid_status_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CID_STATUS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_CID_STATUS]);
}
static CLASS_ATTR_RO(cid_status);

static ssize_t cc_toggle_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_CC_TOGGLE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t cc_toggle_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CC_TOGGLE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_CC_TOGGLE]);
}
static CLASS_ATTR_RW(cc_toggle);

static ssize_t smart_chg_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SMART_CHG, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t smart_chg_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SMART_CHG);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SMART_CHG]);
}

static CLASS_ATTR_RW(smart_chg);

static ssize_t smart_sic_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SMART_SIC_MODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SMART_SIC_MODE]);
}

static CLASS_ATTR_RO(smart_sic_mode);

static ssize_t charger_user_value_map_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_CHARGER_USER_VALUE_MAP, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t charger_user_value_map_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_CHARGER_USER_VALUE_MAP);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CHARGER_USER_VALUE_MAP]);
}
static CLASS_ATTR_RW(charger_user_value_map);

static ssize_t two_ntc_parameter1_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_TWO_NTC_PARAMETER1, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t two_ntc_parameter1_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_TWO_NTC_PARAMETER1);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_TWO_NTC_PARAMETER1]);
}
static CLASS_ATTR_RW(two_ntc_parameter1);

static ssize_t two_ntc_parameter2_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_TWO_NTC_PARAMETER2, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t two_ntc_parameter2_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_TWO_NTC_PARAMETER2);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_TWO_NTC_PARAMETER2]);
}
static CLASS_ATTR_RW(two_ntc_parameter2);

static ssize_t two_ntc_parameter3_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_TWO_NTC_PARAMETER3, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t two_ntc_parameter3_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_TWO_NTC_PARAMETER3);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_TWO_NTC_PARAMETER3]);
}
static CLASS_ATTR_RW(two_ntc_parameter3);

static ssize_t two_ntc_parameter4_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_TWO_NTC_PARAMETER4, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t two_ntc_parameter4_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_TWO_NTC_PARAMETER4);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_TWO_NTC_PARAMETER4]);
}
static CLASS_ATTR_RW(two_ntc_parameter4);

static ssize_t two_ntc_parameter5_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_TWO_NTC_PARAMETER5, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t two_ntc_parameter5_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_TWO_NTC_PARAMETER5);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_TWO_NTC_PARAMETER5]);
}
static CLASS_ATTR_RW(two_ntc_parameter5);

static ssize_t two_ntc_parameter6_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_TWO_NTC_PARAMETER6, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t two_ntc_parameter6_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_TWO_NTC_PARAMETER6);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_TWO_NTC_PARAMETER6]);
}
static CLASS_ATTR_RW(two_ntc_parameter6);

static ssize_t low_fast_para_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_LOW_FAST_PARA, val);
	if (rc < 0)
		return rc;

	return count;
}

static CLASS_ATTR_WO(low_fast_para);

static ssize_t lowfast_sw_para_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_LOWFAST_SW_PARA, val);
	if (rc < 0)
		return rc;

	return count;
}

static CLASS_ATTR_WO(lowfast_sw_para);

static ssize_t dot_test_store(struct class *c,
                                        struct class_attribute *attr,
                                        const char *buf, size_t count)
{
        struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
                                                battery_class);
        int rc;
        int val;

        if (kstrtoint(buf, 10, &val))
                return -EINVAL;

        rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
                                XM_PROP_DOT_TEST, val);
        if (rc < 0)
                return rc;

        return count;
}
static CLASS_ATTR_WO(dot_test);

static ssize_t lpd_control_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;


	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_LPD_CONTROL, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t lpd_control_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_CONTROL);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_LPD_CONTROL]);
}

static CLASS_ATTR_RW(lpd_control);

#if defined(CONFIG_MI_WIRELESS)
static int write_wls_bin_prop_id(struct battery_chg_dev *bcdev, struct psy_state *pst,
			u32 prop_id, u16 total_length, u8 serial_number, u8 fw_area, u8* buff)
{
	struct xm_set_wls_bin_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;
	req_msg.total_length = total_length;
	req_msg.serial_number = serial_number;
	req_msg.fw_area = fw_area;
	if(serial_number < total_length/MAX_STR_LEN)
		memcpy(req_msg.wls_fw_bin, buff, MAX_STR_LEN);
	else if(serial_number == total_length/MAX_STR_LEN)
		memcpy(req_msg.wls_fw_bin, buff, total_length - serial_number*MAX_STR_LEN);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static ssize_t wls_bin_store(struct class *c,
			struct class_attribute *attr,
			const char *buf, size_t count)
{

	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
				battery_class);
	int rc, retry, tmp_serial;
	static u16 total_length = 0;
	static u8 serial_number = 0;
	static u8 fw_area = 0;

	if( strncmp("length:", buf, 7 ) == 0 ) {
		if (kstrtou16( buf+7, 10, &total_length))
		      return -EINVAL;
		serial_number = 0;
	} else if( strncmp("area:", buf, 5 ) == 0 ) {
		if (kstrtou8( buf+5, 10, &fw_area))
		      return -EINVAL;
	}else {
		for( tmp_serial=0;
			(tmp_serial<(count+MAX_STR_LEN-1)/MAX_STR_LEN) && (serial_number<(total_length+MAX_STR_LEN-1)/MAX_STR_LEN);
			++tmp_serial,++serial_number)
		{
			for(retry = 0; retry < 3; ++retry )
			{
				rc = write_wls_bin_prop_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
							XM_PROP_WLS_BIN,
							total_length,
							serial_number,
							fw_area,
							(u8 *)buf+tmp_serial*MAX_STR_LEN);
				if (rc == 0)
				      break;
			}
		}
	}
	return count;
}
static CLASS_ATTR_WO(wls_bin);

static ssize_t wireless_chip_fw_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct wireless_chip_fw_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_FW_VER;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%s\n", bcdev->wireless_chip_fw_version);
}

static ssize_t wireless_chip_fw_store(struct class *c,
		 struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct wireless_chip_fw_msg req_msg = { { 0 } };
	int rc;
	u32 val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	req_msg.property_id = XM_PROP_FW_VER;
	req_msg.value = val;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_RW(wireless_chip_fw);

static ssize_t wireless_tx_uuid_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct wireless_tx_uuid_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_TX_UUID;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%s\n", bcdev->wireless_tx_uuid_version);
}
static CLASS_ATTR_RO(wireless_tx_uuid);

static ssize_t wireless_version_forcit_show(struct class *c,
                struct class_attribute *attr, char *buf)
{
    struct battery_chg_dev *bcdev =
            container_of(c, struct battery_chg_dev, battery_class);
    char *buffer = NULL;
    buffer = (char*)get_zeroed_page(GFP_KERNEL);
    if(!buffer)
        return 0;

    wireless_chip_fw_show(&(bcdev->battery_class), NULL, buffer);
    while (strncmp(buffer, "updating", strlen("updating")) == 0) {
        msleep(200);
        wireless_chip_fw_show(&(bcdev->battery_class), NULL, buffer);
    }
    if(strncmp(buffer, "00.00.00.00", strlen("00.00.00.00")) == 0
        || strstr(buffer, "fe") != 0) {
        return scnprintf(buf, PAGE_SIZE, "%s\n", "fail");
    } else {
        return scnprintf(buf, PAGE_SIZE, "%s\n", "pass");
    }
}
static CLASS_ATTR_RO(wireless_version_forcit);

static ssize_t wls_fw_state_show(struct class *c,
			struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
				battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_WLS_FW_STATE);
	if (rc < 0)
	      return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_WLS_FW_STATE]);
}
static CLASS_ATTR_RO(wls_fw_state);

static ssize_t wls_car_adapter_show(struct class *c,
			struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
				battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_WLS_CAR_ADAPTER);
	if (rc < 0)
	      return rc;
	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_WLS_CAR_ADAPTER]);
}
static CLASS_ATTR_RO(wls_car_adapter);

static ssize_t wls_tx_speed_store(struct class *c,
			struct class_attribute *attr,
			const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
				battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
	      return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_WLS_TX_SPEED, val);
	if (rc < 0)
	      return rc;
	return count;
}
static ssize_t wls_tx_speed_show(struct class *c,
			struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
				battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_WLS_TX_SPEED);
	if (rc < 0)
	      return rc;
	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_WLS_TX_SPEED]);
}
static CLASS_ATTR_RW(wls_tx_speed);

static ssize_t wls_fc_flag_show(struct class *c,
                       struct class_attribute *attr, char *buf)
{
       struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
                               battery_class);
       struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
       int rc;
       rc = read_property_id(bcdev, pst, XM_PROP_WLS_FC_FLAG);
       if (rc < 0)
             return rc;
       return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_WLS_FC_FLAG]);
}
static CLASS_ATTR_RO(wls_fc_flag);

static ssize_t tx_mac_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	u64 value = 0;

	rc = read_property_id(bcdev, pst, XM_PROP_TX_MACL);
	if (rc < 0)
		return rc;

	rc = read_property_id(bcdev, pst, XM_PROP_TX_MACH);
	if (rc < 0)
		return rc;
	value = pst->prop[XM_PROP_TX_MACH];
	value = (value << 32) + pst->prop[XM_PROP_TX_MACL];

	return scnprintf(buf, PAGE_SIZE, "%llx", value);
}
static CLASS_ATTR_RO(tx_mac);

static ssize_t rx_cr_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	u64 value = 0;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_CRL);
	if (rc < 0)
		return rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_CRH);
	if (rc < 0)
		return rc;
	value = pst->prop[XM_PROP_RX_CRH];
	value = (value << 32) + pst->prop[XM_PROP_RX_CRL];

	return scnprintf(buf, PAGE_SIZE, "%llx", value);
}
static CLASS_ATTR_RO(rx_cr);

static ssize_t rx_cep_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_CEP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%x", pst->prop[XM_PROP_RX_CEP]);
}
static CLASS_ATTR_RO(rx_cep);

static ssize_t bt_state_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_BT_STATE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t bt_state_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BT_STATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_BT_STATE]);
}
static CLASS_ATTR_RW(bt_state);

static ssize_t wlscharge_control_limit_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if(val == bcdev->curr_wlsthermal_level)
	      return count;

	if (bcdev->num_thermal_levels <= 0) {
		return -EINVAL;
	}

	if (val < 0 || val >= bcdev->num_thermal_levels)
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_WLSCHARGE_CONTROL_LIMIT, val);
	if (rc < 0)
		return rc;

	bcdev->curr_wlsthermal_level = val;

	return count;
}

static ssize_t wlscharge_control_limit_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_WLSCHARGE_CONTROL_LIMIT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_WLSCHARGE_CONTROL_LIMIT]);
}
static CLASS_ATTR_RW(wlscharge_control_limit);

static ssize_t wls_quick_chg_control_limit_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if(val == bcdev->curr_wls_quick_thermal_level)
	      return count;


	if (bcdev->num_thermal_levels <= 0) {
		return -EINVAL;
	}

	if (val < 0 || val >= bcdev->num_thermal_levels)
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_WLS_QUICK_CHARGE_CONTROL_LIMIT, val);
	if (rc < 0)
		return rc;

	bcdev->curr_wls_quick_thermal_level = val;

	return count;
}

static ssize_t wls_quick_chg_control_limit_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_WLS_QUICK_CHARGE_CONTROL_LIMIT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_WLS_QUICK_CHARGE_CONTROL_LIMIT]);
}
static CLASS_ATTR_RW(wls_quick_chg_control_limit);

#ifndef CONFIG_WIRELESS_REVERSE_CLOSE
static ssize_t reverse_chg_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	bcdev->boost_mode = val;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_REVERSE_CHG_MODE, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t reverse_chg_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_REVERSE_CHG_MODE);
	if (rc < 0)
		goto out;

	if (bcdev->reverse_chg_flag != pst->prop[XM_PROP_REVERSE_CHG_MODE]) {
		if (pst->prop[XM_PROP_REVERSE_CHG_MODE]) {
			pm_stay_awake(bcdev->dev);
		}
		else {
			pm_relax(bcdev->dev);
		}
		bcdev->reverse_chg_flag = pst->prop[XM_PROP_REVERSE_CHG_MODE];
	}

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_REVERSE_CHG_MODE]);

out:
	bcdev->reverse_chg_flag = 0;
	pm_relax(bcdev->dev);
	return rc;
}
static CLASS_ATTR_RW(reverse_chg_mode);

static ssize_t reverse_chg_state_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_REVERSE_CHG_STATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_REVERSE_CHG_STATE]);
}
static CLASS_ATTR_RO(reverse_chg_state);
#endif

static ssize_t set_rx_sleep_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, pst, XM_PROP_SET_RX_SLEEP, val);
	if (rc < 0)
		return rc;

	return count;
}
static ssize_t set_rx_sleep_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SET_RX_SLEEP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_SET_RX_SLEEP]);
}
static CLASS_ATTR_RW(set_rx_sleep);

static ssize_t rx_sleep_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, pst, XM_PROP_RX_SLEEP_MODE, val);

	if (rc < 0)
		return rc;

	return count;
}

static ssize_t rx_sleep_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_SLEEP_MODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_RX_SLEEP_MODE]);
}
static CLASS_ATTR_RW(rx_sleep_mode);

static ssize_t rx_vout_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_VOUT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_RX_VOUT]);
}
static CLASS_ATTR_RO(rx_vout);

static ssize_t rx_ss_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_SS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_RX_SS]);
}
static CLASS_ATTR_RO(rx_ss);

static ssize_t rx_offset_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, pst, XM_PROP_RX_OFFSET, val);

	if (rc < 0)
		return rc;

	return count;
}

static ssize_t rx_offset_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_OFFSET);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_RX_OFFSET]);
}
static CLASS_ATTR_RW(rx_offset);

static ssize_t rx_vrect_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_VRECT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_RX_VRECT]);
}
static CLASS_ATTR_RO(rx_vrect);

static ssize_t rx_iout_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RX_IOUT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_RX_IOUT]);
}
static CLASS_ATTR_RO(rx_iout);

static ssize_t tx_adapter_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_TX_ADAPTER);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_TX_ADAPTER]);
}
static CLASS_ATTR_RO(tx_adapter);

static ssize_t low_inductance_offset_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LOW_INDUCTANCE_OFFSET);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_LOW_INDUCTANCE_OFFSET]);
}
static CLASS_ATTR_RO(low_inductance_offset);

static ssize_t op_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_OP_MODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_OP_MODE]);
}
static CLASS_ATTR_RO(op_mode);


static ssize_t wls_die_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_WLS_DIE_TEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_WLS_DIE_TEMP]);
}
static CLASS_ATTR_RO(wls_die_temp);

static ssize_t wls_thermal_remove_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_WLS_THERMAL_REMOVE, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t wls_thermal_remove_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_WLS_THERMAL_REMOVE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_WLS_THERMAL_REMOVE]);
}
static CLASS_ATTR_RW(wls_thermal_remove);

// wls pen
#if defined(CONFIG_MI_PEN_WIRELESS)
static ssize_t pen_mac_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	u64 value = 0;
	u64 value_l = 0;
	u64 value_h = 0;
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_MACL);
	if (rc < 0)
		return rc;
	value_l = pst->prop[XM_PROP_PEN_MACL];
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_MACH);
	if (rc < 0)
		return rc;
	value_h = pst->prop[XM_PROP_PEN_MACH];

	if ((value_l == 0) && (value_h != 0)) {
		pr_err("Ignore: value_l:%x, value_h:%x.\n", value_l, value_h);
		value_h = 0;
	}

	value = (value_h << 32) + value_l;
	return scnprintf(buf, PAGE_SIZE, "%llx", value);
}
static CLASS_ATTR_RO(pen_mac);

static ssize_t tx_iout_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_TX_IOUT);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_TX_IOUT]);
}
static CLASS_ATTR_RO(tx_iout);

static ssize_t tx_vout_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_TX_VOUT);
	if (rc < 0)
		return rc;
	pr_err("tx_vout_show %d\n",pst->prop[XM_PROP_TX_VOUT]);
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_TX_VOUT]);
}
static CLASS_ATTR_RO(tx_vout);

static ssize_t pen_soc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_SOC);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_PEN_SOC]);
}
static CLASS_ATTR_RO(pen_soc);

static ssize_t pen_hall3_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_HALL3);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_PEN_HALL3]);
}
static CLASS_ATTR_RO(pen_hall3);

static ssize_t pen_hall4_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_HALL4);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_PEN_HALL4]);
}
static CLASS_ATTR_RO(pen_hall4);

static ssize_t pen_hall3_s_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_HALL3_S);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_PEN_HALL3_S]);
}
static CLASS_ATTR_RO(pen_hall3_s);

static ssize_t pen_hall4_s_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_HALL4_S);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_PEN_HALL4_S]);
}
static CLASS_ATTR_RO(pen_hall4_s);

static ssize_t pen_ppe_hall_n_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_PPE_HALL_N);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_PEN_PPE_HALL_N]);
}
static CLASS_ATTR_RO(pen_ppe_hall_n);

static ssize_t pen_ppe_hall_s_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_PEN_PPE_HALL_S);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_PEN_PPE_HALL_S]);
}
static CLASS_ATTR_RO(pen_ppe_hall_s);

static ssize_t pen_tx_ss_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_PEN_TX_SS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_PEN_TX_SS]);
}
static CLASS_ATTR_RO(pen_tx_ss);

static ssize_t pen_place_err_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_PEN_PLACE_ERR);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d", pst->prop[XM_PROP_PEN_PLACE_ERR]);
}
static CLASS_ATTR_RO(pen_place_err);

static ssize_t fake_ss_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FAKE_SS, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fake_ss_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FAKE_SS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_FAKE_SS]);
}
static CLASS_ATTR_RW(fake_ss);
#endif
#endif

static ssize_t fg_vendor_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG_VENDOR_ID);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG_VENDOR_ID]);
}
static CLASS_ATTR_RO(fg_vendor);

static ssize_t pack_vendor_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	const char* prop = NULL;
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_PACK_VENDOR_ID);
	if (rc < 0)
		return rc;
	prop = POWER_SUPPLY_VENDOR_TEXT[pst->prop[XM_PROP_PACK_VENDOR_ID]];
	return scnprintf(buf, PAGE_SIZE, "%s\n", prop);
}
static CLASS_ATTR_RO(pack_vendor);

static ssize_t cell_vendor_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	const char* prop = NULL;
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CELL_VENDOR_ID);
	if (rc < 0)
		return rc;
	prop = POWER_SUPPLY_CELL_VENDOR_TEXT[pst->prop[XM_PROP_CELL_VENDOR_ID]];
	return scnprintf(buf, PAGE_SIZE, "%s\n", prop);
}
static CLASS_ATTR_RO(cell_vendor);

static ssize_t dod_count_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_DOD_COUNT);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_DOD_COUNT]);
}

static ssize_t dod_count_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_DOD_COUNT, val);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_RW(dod_count);

static ssize_t fg1_qmax_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_QMAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_QMAX]);
}
static CLASS_ATTR_RO(fg1_qmax);

static ssize_t fg1_rm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_RM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_RM]);
}
static CLASS_ATTR_RO(fg1_rm);

static ssize_t fg1_fcc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_FCC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_FCC]);
}
static CLASS_ATTR_RO(fg1_fcc);

static ssize_t fg1_soh_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_SOH]);
}
static CLASS_ATTR_RO(fg1_soh);

static ssize_t fg1_rsoc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_RSOC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_RSOC]);
}
static CLASS_ATTR_RO(fg1_rsoc);

static ssize_t fg1_ai_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_AI);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_AI]);
}
static CLASS_ATTR_RO(fg1_ai);

static ssize_t fg1_fcc_soh_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_FCC_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_FCC_SOH]);
}
static CLASS_ATTR_RO(fg1_fcc_soh);

static ssize_t fg1_cycle_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_CYCLE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_CYCLE]);
}
static CLASS_ATTR_RO(fg1_cycle);

static ssize_t fg1_oritemp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_ORITEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_ORITEMP]);
}
static CLASS_ATTR_RO(fg1_oritemp);

static ssize_t fg1_fastcharge_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_FAST_CHARGE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_FAST_CHARGE]);
}
static CLASS_ATTR_RO(fg1_fastcharge);

static ssize_t fg1_current_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_CURRENT_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_CURRENT_MAX]);
}
static CLASS_ATTR_RO(fg1_current_max);

static ssize_t fg1_vol_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_VOL_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_VOL_MAX]);
}
static CLASS_ATTR_RO(fg1_vol_max);

static ssize_t fg1_tsim_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TSIM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TSIM]);
}
static CLASS_ATTR_RO(fg1_tsim);

static ssize_t fg1_tambient_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TAMBIENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TAMBIENT]);
}
static CLASS_ATTR_RO(fg1_tambient);

static ssize_t fg1_tremq_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TREMQ);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TREMQ]);
}
static CLASS_ATTR_RO(fg1_tremq);

static ssize_t fg1_tfullq_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TFULLQ);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TFULLQ]);
}
static CLASS_ATTR_RO(fg1_tfullq);

static ssize_t fg1_temp_max_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TEMP_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TEMP_MAX]);
}
static CLASS_ATTR_RO(fg1_temp_max);

static ssize_t fg2_temp_max_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_TEMP_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_TEMP_MAX]);
}
static CLASS_ATTR_RO(fg2_temp_max);

static ssize_t fg1_time_ot_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TIME_OT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TIME_OT]);
}
static CLASS_ATTR_RO(fg1_time_ot);

static ssize_t fg1_time_ht_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TIME_HT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TIME_HT]);
}
static CLASS_ATTR_RO(fg1_time_ht);

static ssize_t fg1_seal_set_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG1_SEAL_SET, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg1_seal_set_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_SEAL_SET);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_FG1_SEAL_SET]);
}
static CLASS_ATTR_RW(fg1_seal_set);

static ssize_t fg1_seal_state_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_SEAL_STATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_SEAL_STATE]);
}
static CLASS_ATTR_RO(fg1_seal_state);

static ssize_t fg1_df_check_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_DF_CHECK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_DF_CHECK]);
}
static CLASS_ATTR_RO(fg1_df_check);

static ssize_t fg1_imax_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_IMAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_IMAX]);
}
static CLASS_ATTR_RO(fg1_imax);

#define BSWAP_32(x) \
	(u32)((((u32)(x) & 0xff000000) >> 24) | \
			(((u32)(x) & 0x00ff0000) >> 8) | \
			(((u32)(x) & 0x0000ff00) << 8) | \
			(((u32)(x) & 0x000000ff) << 24))

static void usbpd_sha256_bitswap32(unsigned int *array, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		array[i] = BSWAP_32(array[i]);
	}
}

static void usbpd_request_vdm_cmd(struct battery_chg_dev *bcdev, enum uvdm_state cmd, unsigned int *data)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	u32 prop_id, val = 0;
	int rc;

	switch (cmd) {
	case USBPD_UVDM_CHARGER_VERSION:
		prop_id = XM_PROP_VDM_CMD_CHARGER_VERSION;
		break;
	case USBPD_UVDM_CHARGER_VOLTAGE:
		prop_id = XM_PROP_VDM_CMD_CHARGER_VOLTAGE;
		break;
	case USBPD_UVDM_CHARGER_TEMP:
		prop_id = XM_PROP_VDM_CMD_CHARGER_TEMP;
		break;
	case USBPD_UVDM_SESSION_SEED:
		prop_id = XM_PROP_VDM_CMD_SESSION_SEED;
		usbpd_sha256_bitswap32(data, USBPD_UVDM_SS_LEN);
		val = *data;
		break;
	case USBPD_UVDM_AUTHENTICATION:
		prop_id = XM_PROP_VDM_CMD_AUTHENTICATION;
		usbpd_sha256_bitswap32(data, USBPD_UVDM_SS_LEN);
		val = *data;
		break;
	case USBPD_UVDM_REVERSE_AUTHEN:
                prop_id = XM_PROP_VDM_CMD_REVERSE_AUTHEN;
                usbpd_sha256_bitswap32(data, USBPD_UVDM_SS_LEN);
                val = *data;
                break;
	case USBPD_UVDM_REMOVE_COMPENSATION:
		prop_id = XM_PROP_VDM_CMD_REMOVE_COMPENSATION;
		val = *data;
		break;
	case USBPD_UVDM_VERIFIED:
		prop_id = XM_PROP_VDM_CMD_VERIFIED;
		val = *data;
		break;
	default:
		prop_id = XM_PROP_VDM_CMD_CHARGER_VERSION;
		break;
	}

	if(cmd == USBPD_UVDM_SESSION_SEED || cmd == USBPD_UVDM_AUTHENTICATION || cmd == USBPD_UVDM_REVERSE_AUTHEN) {
		rc = write_ss_auth_prop_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				prop_id, data);
	}
	else
		rc = write_property_id(bcdev, pst, prop_id, val);
}


static ssize_t request_vdm_cmd_store(struct class *c,
					struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int cmd, ret;
	unsigned char buffer[64];
	unsigned char data[32];
	int ccount;

	ret = sscanf(buf, "%d,%s\n", &cmd, buffer);

	StringToHex(buffer, data, &ccount);
	usbpd_request_vdm_cmd(bcdev, cmd, (unsigned int *)data);
	return count;
}

/*pd verify*/
static ssize_t request_vdm_cmd_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	u32 prop_id = 0;
	int i;
	char data[16], str_buf[128] = {0};
	enum uvdm_state cmd;

	rc = read_property_id(bcdev, pst, XM_PROP_UVDM_STATE);
	if (rc < 0)
		return rc;

	cmd = pst->prop[XM_PROP_UVDM_STATE];

	switch (cmd){
	  case USBPD_UVDM_CHARGER_VERSION:
	  	prop_id = XM_PROP_VDM_CMD_CHARGER_VERSION;
		rc = read_property_id(bcdev, pst, prop_id);
		return snprintf(buf, PAGE_SIZE, "%d,%d", cmd, pst->prop[prop_id]);
	  	break;
	  case USBPD_UVDM_CHARGER_TEMP:
	  	prop_id = XM_PROP_VDM_CMD_CHARGER_TEMP;
		rc = read_property_id(bcdev, pst, prop_id);
		return snprintf(buf, PAGE_SIZE, "%d,%d", cmd, pst->prop[prop_id]);
	  	break;
	  case USBPD_UVDM_CHARGER_VOLTAGE:
	  	prop_id = XM_PROP_VDM_CMD_CHARGER_VOLTAGE;
		rc = read_property_id(bcdev, pst, prop_id);
		return snprintf(buf, PAGE_SIZE, "%d,%d", cmd, pst->prop[prop_id]);
	  	break;
	  case USBPD_UVDM_CONNECT:
	  case USBPD_UVDM_DISCONNECT:
	  case USBPD_UVDM_SESSION_SEED:
	  case USBPD_UVDM_VERIFIED:
	  case USBPD_UVDM_REMOVE_COMPENSATION:
	  case USBPD_UVDM_REVERSE_AUTHEN:
	  	return snprintf(buf, PAGE_SIZE, "%d,Null", cmd);
	  	break;
	  case USBPD_UVDM_AUTHENTICATION:
	  	prop_id = XM_PROP_VDM_CMD_AUTHENTICATION;
		rc = read_ss_auth_property_id(bcdev, pst, prop_id);
		if (rc < 0)
			return rc;
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++) {
			memset(data, 0, sizeof(data));
			snprintf(data, sizeof(data), "%08lx", bcdev->ss_auth_data[i]);
			strlcat(str_buf, data, sizeof(str_buf));
		}
		return snprintf(buf, PAGE_SIZE, "%d,%s", cmd, str_buf);
	  	break;
	  default:
		break;
	}

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[prop_id]);
}
static CLASS_ATTR_RW(request_vdm_cmd);

static const char * const usbpd_state_strings[] = {
	"UNKNOWN",
	"SNK_Startup",
	"SNK_Ready",
	"SRC_Ready",
};

#define USBPD_PE_STATE_PE_SRC_READY 5
#define USBPD_PE_STATE_PE_SNK_STARTUP 26
#define USBPD_PE_STATE_PE_SNK_READY 32


static ssize_t current_state_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CURRENT_STATE);
	if (rc < 0)
		return rc;
	if (pst->prop[XM_PROP_CURRENT_STATE] == USBPD_PE_STATE_PE_SNK_STARTUP)
		return snprintf(buf, PAGE_SIZE, "%s", usbpd_state_strings[1]);
	else if (pst->prop[XM_PROP_CURRENT_STATE] == USBPD_PE_STATE_PE_SNK_READY)
		return snprintf(buf, PAGE_SIZE, "%s", usbpd_state_strings[2]);
	else if (pst->prop[XM_PROP_CURRENT_STATE] == USBPD_PE_STATE_PE_SRC_READY)
		return snprintf(buf, PAGE_SIZE, "%s", usbpd_state_strings[3]);
	else
		return snprintf(buf, PAGE_SIZE, "%s", usbpd_state_strings[0]);

}
static CLASS_ATTR_RO(current_state);

static ssize_t adapter_id_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_ADAPTER_ID);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%08x", pst->prop[XM_PROP_ADAPTER_ID]);
}
static CLASS_ATTR_RO(adapter_id);

static ssize_t adapter_svid_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_ADAPTER_SVID);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%04x", pst->prop[XM_PROP_ADAPTER_SVID]);
}
static CLASS_ATTR_RO(adapter_svid);

static ssize_t pd_verifed_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_PD_VERIFED, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t pd_verifed_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_PD_VERIFED);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_PD_VERIFED]);
}
static CLASS_ATTR_RW(pd_verifed);

#if defined(CONFIG_MI_O16U_EUPD) || defined(CONFIG_MI_O11_EMPTY_BATTERY)
static ssize_t charger_partition_poweroffmode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	bool power_off_mode = 0;

	if (kstrtobool(buf, &power_off_mode))
		return -EINVAL;

	if (power_off_mode)
		charger_partition_set_prop(CHARGER_PARTITION_PROP_POWER_OFF_MODE, 1);
	else
		charger_partition_set_prop(CHARGER_PARTITION_PROP_POWER_OFF_MODE, 2);

	return count;
}

static ssize_t charger_partition_poweroffmode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	uint32_t power_off_mode = 0;

	charger_partition_get_prop(CHARGER_PARTITION_PROP_POWER_OFF_MODE, &power_off_mode);

	return scnprintf(buf, PAGE_SIZE, "%u\n", power_off_mode);
}
static CLASS_ATTR_RW(charger_partition_poweroffmode);
#endif

#if defined(CONFIG_MI_O16U_EUPD) || defined(CONFIG_MI_O11_EMPTY_BATTERY)
static ssize_t is_eu_model_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	if (val)
		charger_partition_set_prop(CHARGER_PARTITION_PROP_EU_MODE, 1);
	else
		charger_partition_set_prop(CHARGER_PARTITION_PROP_EU_MODE, 0);

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_IS_EU_MODEL, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t is_eu_model_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	uint32_t eu_mode = 0;

	charger_partition_get_prop(CHARGER_PARTITION_PROP_EU_MODE, &eu_mode);

	return scnprintf(buf, PAGE_SIZE, "%u\n", eu_mode);
}
static CLASS_ATTR_RW(is_eu_model);

static ssize_t pps_ptf_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_PPS_PTF, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t pps_ptf_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_PPS_PTF);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_PPS_PTF]);
}
static CLASS_ATTR_RW(pps_ptf);
/* add end */
#endif

#if defined(CONFIG_MI_O11_EMPTY_BATTERY)
static ssize_t cp_fsw_store(struct class *c,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_CP_FSW, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t cp_fsw_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CP_FSW);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CP_FSW]);
}
static CLASS_ATTR_RW(cp_fsw);

static ssize_t cp_iout_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CP_IOUT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CP_IOUT]);
}
static CLASS_ATTR_RO(cp_iout);
#endif

static ssize_t pdo2_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_PDO2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%08x\n", pst->prop[XM_PROP_PDO2]);
}
static CLASS_ATTR_RO(pdo2);

static ssize_t verify_process_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_VERIFY_PROCESS, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t verify_process_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_VERIFY_PROCESS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_VERIFY_PROCESS]);
}
static CLASS_ATTR_RW(verify_process);

#if defined (CONFIG_MI_MULTI_CHG_CFG)
static ssize_t chg_cfg_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CHG_CFG);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CHG_CFG]);
}
static CLASS_ATTR_RO(chg_cfg);
#endif

static ssize_t power_max_show(struct class *c,
			struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
				battery_class);
	struct psy_state *xm_pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	union power_supply_propval val = {0, };
	int rc, usb_present = 0, charge_present = 0, wls_present = 0;
	struct power_supply *usb_psy = NULL;
	int	batt_status;
#if defined(CONFIG_MI_WIRELESS)
	struct power_supply *wls_psy = NULL;
#endif
	usb_psy = bcdev->psy_list[PSY_TYPE_USB].psy;
	if (usb_psy != NULL) {
		rc = usb_psy_get_prop(usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (!rc)
			usb_present = val.intval;
		else
			usb_present = 0;
	}

	rc = read_property_id(bcdev, pst, BATT_STATUS);
	if (rc < 0)
		return rc;
	batt_status = pst->prop[BATT_STATUS];
	if (batt_status == POWER_SUPPLY_STATUS_CHARGING) {
		charge_present = 1;
	}else{
        	charge_present = 0;
        }

#if defined(CONFIG_MI_WIRELESS)
	wls_psy = bcdev->psy_list[PSY_TYPE_WLS].psy;
	if (wls_psy != NULL) {
		rc = wls_psy_get_prop(wls_psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (!rc)
			wls_present = val.intval;
		else
			wls_present = 0;
	}
#endif

	if (usb_present || wls_present || charge_present) {
		rc = read_property_id(bcdev, xm_pst, XM_PROP_POWER_MAX);
		if (rc < 0)
			return rc;
		return scnprintf(buf, PAGE_SIZE, "%u", xm_pst->prop[XM_PROP_POWER_MAX]);
	}
	return scnprintf(buf, PAGE_SIZE, "%u", 0);
}
static CLASS_ATTR_RO(power_max);

static ssize_t cloud_fod_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_CLOUD_FOD_DATA;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%s", bcdev->cloud_fod_data);
}

static ssize_t cloud_fod_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
							battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_CLOUD_FOD_DATA;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	memset(req_msg.data, '\0', sizeof(req_msg.data));
	strncpy(req_msg.data, buf, count);

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;
	return count;
}
static CLASS_ATTR_RW(cloud_fod);

static ssize_t last_node_show(struct class *c,
			struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LAST_NODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_LAST_NODE]);
}
static CLASS_ATTR_RO(last_node);

#if defined(CONFIG_MI_DTPT)
static ssize_t battmoni_isc_show(struct class *c, struct class_attribute *attr,
				 char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_NVTFG_MONITOR_ISC);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_NVTFG_MONITOR_ISC]);
}
static CLASS_ATTR_RO(battmoni_isc);

static ssize_t battmoni_soa_show(struct class *c, struct class_attribute *attr,
				 char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_NVTFG_MONITOR_SOA);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_NVTFG_MONITOR_SOA]);
}
static CLASS_ATTR_RO(battmoni_soa);

static ssize_t over_peak_flag_show(struct class *c,
				   struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_OVER_PEAK_FLAG);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_OVER_PEAK_FLAG]);
}
static CLASS_ATTR_RO(over_peak_flag);

static ssize_t current_deviation_show(struct class *c,
				      struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_CURRENT_DEVIATION);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_CURRENT_DEVIATION]);
}
static CLASS_ATTR_RO(current_deviation);

static ssize_t power_deviation_show(struct class *c,
				    struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_POWER_DEVIATION);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_POWER_DEVIATION]);
}
static CLASS_ATTR_RO(power_deviation);

static ssize_t average_current_show(struct class *c,
				    struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_AVERAGE_CURRENT);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_AVERAGE_CURRENT]);
}
static CLASS_ATTR_RO(average_current);

static ssize_t average_temp_show(struct class *c, struct class_attribute *attr,
				 char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_AVERAGE_TEMPERATURE);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_AVERAGE_TEMPERATURE]);
}
static CLASS_ATTR_RO(average_temp);

static ssize_t start_learn_store(struct class *c, struct class_attribute *attr,
				 const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_START_LEARNING, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t start_learn_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_START_LEARNING);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_START_LEARNING]);
}
static CLASS_ATTR_RW(start_learn);

static ssize_t stop_learn_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_STOP_LEARNING, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t stop_learn_show(struct class *c, struct class_attribute *attr,
			       char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_STOP_LEARNING);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_STOP_LEARNING]);
}
static CLASS_ATTR_RW(stop_learn);

static ssize_t set_learn_power_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_SET_LEARNING_POWER, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t set_learn_power_show(struct class *c,
				    struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_SET_LEARNING_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_SET_LEARNING_POWER]);
}
static CLASS_ATTR_RW(set_learn_power);

static ssize_t get_learn_power_show(struct class *c,
				    struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_GET_LEARNING_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_GET_LEARNING_POWER]);
}
static CLASS_ATTR_RO(get_learn_power);

static ssize_t get_learn_power_dev_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_GET_LEARNING_POWER_DEV);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_GET_LEARNING_POWER_DEV]);
}
static CLASS_ATTR_RO(get_learn_power_dev);

static ssize_t start_learn_b_store(struct class *c,
				   struct class_attribute *attr,
				   const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_START_LEARNING_B, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t start_learn_b_show(struct class *c, struct class_attribute *attr,
				  char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_START_LEARNING_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_START_LEARNING_B]);
}
static CLASS_ATTR_RW(start_learn_b);

static ssize_t stop_learn_b_store(struct class *c, struct class_attribute *attr,
				  const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_STOP_LEARNING_B, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t stop_learn_b_show(struct class *c, struct class_attribute *attr,
				 char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_STOP_LEARNING_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_STOP_LEARNING_B]);
}
static CLASS_ATTR_RW(stop_learn_b);

static ssize_t set_learn_power_b_store(struct class *c,
				       struct class_attribute *attr,
				       const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_SET_LEARNING_POWER_B, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t set_learn_power_b_show(struct class *c,
				      struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_SET_LEARNING_POWER_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_SET_LEARNING_POWER_B]);
}
static CLASS_ATTR_RW(set_learn_power_b);

static ssize_t get_learn_power_b_show(struct class *c,
				      struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_GET_LEARNING_POWER_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_GET_LEARNING_POWER_B]);
}
static CLASS_ATTR_RO(get_learn_power_b);

static ssize_t get_learn_power_dev_b_show(struct class *c,
					  struct class_attribute *attr,
					  char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_GET_LEARNING_POWER_DEV_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_GET_LEARNING_POWER_DEV_B]);
}
static CLASS_ATTR_RO(get_learn_power_dev_b);

static ssize_t get_learn_time_dev_show(struct class *c,
				       struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_GET_LEARNING_TIME_DEV);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_GET_LEARNING_TIME_DEV]);
}
static CLASS_ATTR_RO(get_learn_time_dev);

static ssize_t constant_power_store(struct class *c,
				    struct class_attribute *attr,
				    const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_SET_CONSTANT_POWER, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t constant_power_show(struct class *c,
				   struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_SET_CONSTANT_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_SET_CONSTANT_POWER]);
}
static CLASS_ATTR_RW(constant_power);

static ssize_t remaining_time_show(struct class *c,
				   struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_GET_REMAINING_TIME);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_GET_REMAINING_TIME]);
}
static CLASS_ATTR_RO(remaining_time);

static ssize_t referance_power_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_SET_REFERANCE_POWER, val);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t referance_power_show(struct class *c,
				    struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_SET_REFERANCE_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_SET_REFERANCE_POWER]);
}
static CLASS_ATTR_RW(referance_power);

static ssize_t nvt_referance_current_show(struct class *c,
					  struct class_attribute *attr,
					  char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_GET_REFERANCE_CURRENT);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_GET_REFERANCE_CURRENT]);
}
static CLASS_ATTR_RO(nvt_referance_current);

static ssize_t nvt_referance_power_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_GET_REFERANCE_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pst->prop[XM_PROP_GET_REFERANCE_POWER]);
}
static CLASS_ATTR_RO(nvt_referance_power);
#endif

#if defined(CONFIG_MI_ENABLE_DP)
static ssize_t has_dp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_HAS_DP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_HAS_DP]);
}
static CLASS_ATTR_RO(has_dp);
#endif

static ssize_t dis_uart_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_UART_SLEEP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_LPD_UART_SLEEP]);
}

static ssize_t dis_uart_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_LPD_UART_SLEEP, val);
	if (rc < 0)
		return rc;
	return count;
}
static CLASS_ATTR_RW(dis_uart);

static ssize_t lpd_charging_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_CHARGING);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_LPD_CHARGING]);
}

static ssize_t lpd_charging_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_LPD_CHARGING, val);
	if (rc < 0)
		return rc;
	return count;
}
static CLASS_ATTR_RW(lpd_charging);

static ssize_t sbu1_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_SBU1);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_LPD_SBU1]);
}

static ssize_t sbu1_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_LPD_SBU1, val);
	if (rc < 0)
		return rc;
	return count;
}
static CLASS_ATTR_RW(sbu1);

static ssize_t sbu2_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_SBU2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_LPD_SBU2]);
}
static CLASS_ATTR_RO(sbu2);


static ssize_t cc1_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_CC1);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_LPD_CC1]);
}
static CLASS_ATTR_RO(cc1);

static ssize_t cc2_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_CC2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_LPD_CC2]);
}
static CLASS_ATTR_RO(cc2);

static ssize_t plate_shock_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
		struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
												battery_class);
		int rc;
		int val;
		if(kstrtoint(buf, 10, &val))
					return -EINVAL;
			rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
									XM_PROP_PLATE_SHOCK, val);
			if(rc < 0)
					return rc;
			return count;
}
static ssize_t plate_shock_show(struct class *c,
                                        struct class_attribute *attr, char *buf)
{
        struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
                                                battery_class);
        struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
        int rc;

        rc = read_property_id(bcdev, pst, XM_PROP_PLATE_SHOCK);
        if (rc < 0)
                return rc;

        return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_PLATE_SHOCK]);
}
static CLASS_ATTR_RW(plate_shock);

static ssize_t ui_soh_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_UI_SOH_DATA;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s", bcdev->ui_soh_data);
}

static ssize_t ui_soh_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
							battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_UI_SOH_DATA;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	memset(req_msg.data, '\0', sizeof(req_msg.data));
	strncpy(req_msg.data, buf, count);

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;
	return count;
}
static CLASS_ATTR_RW(ui_soh);

static ssize_t ui_slave_soh_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_SLAVE_UI_SOH_DATA;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s", bcdev->ui_slave_soh_data);
}

static ssize_t ui_slave_soh_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;

	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_SLAVE_UI_SOH_DATA;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	memset(req_msg.data, '\0', sizeof(req_msg.data));
	strncpy(req_msg.data, buf, count);

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;
	return count;
}
static CLASS_ATTR_RW(ui_slave_soh);

static ssize_t soh_sn_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	struct chg_debug_msg req_msg = { { 0 } };
	int rc;
	req_msg.property_id = XM_PROP_CHG_DEBUG;
	req_msg.type = CHG_UI_SOH_SN_CODE;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;
	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%s", bcdev->ui_soh_data);
}
static CLASS_ATTR_RO(soh_sn);

static ssize_t fg2_soh_sn_show(struct class *c,
                                        struct class_attribute *attr, char *buf)
{
        struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
                                                battery_class);
        struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
        struct chg_debug_msg req_msg = { { 0 } };
        int rc;
        req_msg.property_id = XM_PROP_CHG_DEBUG;
        req_msg.type = CHG_UI_SLAVE_SOH_SN_CODE;
        req_msg.hdr.owner = MSG_OWNER_BC;
        req_msg.hdr.type = MSG_TYPE_REQ_RESP;
        req_msg.hdr.opcode = pst->opcode_get;
        rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
        if (rc < 0)
                return rc;
        return scnprintf(buf, PAGE_SIZE, "%s", bcdev->ui_slave_soh_data);
}
static CLASS_ATTR_RO(fg2_soh_sn);

static ssize_t calc_rvalue_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CALC_RVALUE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_CALC_RVALUE]);
}
static CLASS_ATTR_RO(calc_rvalue);

static ssize_t dp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_DP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_LPD_DP]);
}
static CLASS_ATTR_RO(dp);

static ssize_t dm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_LPD_DM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_LPD_DM]);
}
static CLASS_ATTR_RO(dm);

static ssize_t maxtemp_occurtime_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_MAX_TEMP_OCCUR_TIME);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_MAX_TEMP_OCCUR_TIME]);
}
static CLASS_ATTR_RO(maxtemp_occurtime);

static ssize_t runtime_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RUN_TIME);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_RUN_TIME]);
}
static CLASS_ATTR_RO(runtime);

static ssize_t maxtemptime_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_MAX_TEMP_TIME);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_MAX_TEMP_TIME]);
}
static CLASS_ATTR_RO(maxtemptime);

static ssize_t handle_state_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_HANDLE_STATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_HANDLE_STATE]);
}

static ssize_t handle_state_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_HANDLE_STATE, val);
	if (rc < 0)
		return rc;
	return count;
}
static CLASS_ATTR_RW(handle_state);

static ssize_t handle_stop_charging_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_HANDLE_STOP_CHARGING);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_HANDLE_STOP_CHARGING]);
}

static ssize_t handle_stop_charging_store(struct class *c,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev =
		container_of(c, struct battery_chg_dev, battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			       XM_PROP_HANDLE_STOP_CHARGING, val);
	if (rc < 0)
		return rc;
	return count;
}
static CLASS_ATTR_RW(handle_stop_charging);

static ssize_t thermal_board_temp_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, pst, XM_PROP_THERMAL_BOARD_TEMP, val);

	if (rc < 0)
		return rc;

	return count;
}

static ssize_t thermal_board_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_THERMAL_BOARD_TEMP);
	if (rc < 0)
		return rc;

        return scnprintf(buf, PAGE_SIZE, "%u", bcdev->thermal_board_temp);
	// return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_THERMAL_BOARD_TEMP]);
}
static CLASS_ATTR_RW(thermal_board_temp);

static ssize_t thermal_scene_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, pst, XM_PROP_THERMAL_SCENE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t thermal_scene_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_THERMAL_SCENE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_THERMAL_SCENE]);
}
static CLASS_ATTR_RW(thermal_scene);

static ssize_t ntc_alarm_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_NTC_ALARM, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t ntc_alarm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_NTC_ALARM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_NTC_ALARM]);
}
static CLASS_ATTR_RW(ntc_alarm);

static ssize_t sport_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SPORT_MODE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t sport_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SPORT_MODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SPORT_MODE]);
}
static CLASS_ATTR_RW(sport_mode);

#if defined(CONFIG_MI_SC760X)
static ssize_t sc760x_chip_ok_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SC760X_CHIP_OK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SC760X_CHIP_OK]);
}
static CLASS_ATTR_RO(sc760x_chip_ok);

static ssize_t sc760x_slave_chip_ok_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SC760X_SLAVE_CHIP_OK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SC760X_SLAVE_CHIP_OK]);
}
static CLASS_ATTR_RO(sc760x_slave_chip_ok);

static ssize_t sc760x_slave_ibatt_limit_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SC760X_SLAVE_IBATT_LIMIT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SC760X_SLAVE_IBATT_LIMIT]);
}
static ssize_t sc760x_slave_ibatt_limit_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SC760X_SLAVE_IBATT_LIMIT, val);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_RW(sc760x_slave_ibatt_limit);
#endif

static ssize_t qbg_curr_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_QBG_CURR);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_QBG_CURR]);
}
static CLASS_ATTR_RO(qbg_curr);

enum device_chg_cfg {
      CN_90W,
      CN_P01_90W,
      GL_120W,
      CN_45W,
};

static int ut_test_parse(struct battery_chg_dev *bcdev)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	enum device_chg_cfg chg_cfg = GL_120W;
	const char* prop = NULL;
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CHG_CFG);
	if (rc < 0) {
		return rc;
	}
	chg_cfg = pst->prop[XM_PROP_CHG_CFG];

	rc = read_property_id(bcdev, pst, XM_PROP_PACK_VENDOR_ID);
	if (rc < 0) {
		return rc;
	}
	prop = POWER_SUPPLY_VENDOR_TEXT[pst->prop[XM_PROP_PACK_VENDOR_ID]];

	if (chg_cfg == GL_120W) {
		bcdev->cycle_volt = bcdev->cycle_volt_gl;
		bcdev->cycle_step_curr = bcdev->cycle_step_curr_gl;
		bcdev->thermal = bcdev->thermal_gl;
		if (0 == strncmp(prop, "NVT", strlen("NVT")))
			bcdev->temp_term_curr = bcdev->temp_term_curr_gl_nvt;
		if (0 == strncmp(prop, "SUNWODA", strlen("SUNWODA")))
			bcdev->temp_term_curr = bcdev->temp_term_curr_gl_sunwoda;
	}
	else if (chg_cfg == CN_90W) {
		bcdev->cycle_volt = bcdev->cycle_volt_cn;
		bcdev->cycle_step_curr = bcdev->cycle_step_curr_cn;
		bcdev->thermal = bcdev->thermal_cn;
		if (0 == strncmp(prop, "NVT", strlen("NVT")))
			bcdev->temp_term_curr = bcdev->temp_term_curr_cn_nvt;
		if (0 == strncmp(prop, "SUNWODA", strlen("SUNWODA")))
			bcdev->temp_term_curr = bcdev->temp_term_curr_cn_sunwoda;
	}
	else if (chg_cfg == CN_45W) {
		if (0 == strncmp(prop, "NVT", strlen("NVT")))
			bcdev->temp_term_curr = bcdev->temp_term_curr_cn_nvt;
		if (0 == strncmp(prop, "SUNWODA", strlen("SUNWODA")))
			bcdev->temp_term_curr = bcdev->temp_term_curr_cn_sunwoda;
	}

	return 0;
}

static ssize_t ut_test_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	char str[MAX_INFO_LENGTH+1] = {0};

	if (bcdev->ut_test_region) {
		int rc;
		rc = ut_test_parse(bcdev);
	}

	if (bcdev->ut_test) {
		snprintf(str, 12, "cycle_volt,");
		snprintf(str + strlen(str), strlen(bcdev->cycle_volt)+1, bcdev->cycle_volt);

		snprintf(str + strlen(str), 17, "cycle_step_curr,");
		snprintf(str + strlen(str), strlen(bcdev->cycle_step_curr)+1, bcdev->cycle_step_curr);

		snprintf(str + strlen(str), 16, "temp_term_curr,");
		snprintf(str + strlen(str),  strlen(bcdev->temp_term_curr)+1, bcdev->temp_term_curr);

		snprintf(str + strlen(str), 9, "thermal,");
		snprintf(str + strlen(str),  strlen(bcdev->thermal)+1, bcdev->thermal);
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", str);
}
static CLASS_ATTR_RO(ut_test);

#if defined (CONFIG_DUAL_FUEL_GAUGE)
static ssize_t slave_chip_ok_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_SLAVE_CHIP_OK);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SLAVE_CHIP_OK]);
}
static CLASS_ATTR_RO(slave_chip_ok);

static ssize_t slave_authentic_store(struct class *c,
																					struct class_attribute *attr,
																					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
																																		battery_class);
		int rc;
		bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->slave_battery_auth = val;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SLAVE_AUTHENTIC, val);
	if (rc < 0)
		return rc;

	return count;
}
static ssize_t slave_authentic_show(struct class *c,
  																						struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SLAVE_AUTHENTIC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SLAVE_AUTHENTIC]);
}
static CLASS_ATTR_RW(slave_authentic);

static ssize_t fg2_rm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_RM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_RM]);
}
static CLASS_ATTR_RO(fg2_rm);

static ssize_t fg2_fcc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_FCC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_FCC]);
}
static CLASS_ATTR_RO(fg2_fcc);

static ssize_t fg2_soh_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_SOH]);
}
static CLASS_ATTR_RO(fg2_soh);

static ssize_t fg2_cycle_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_CYCLE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_CYCLE]);
}
static CLASS_ATTR_RO(fg2_cycle);

static ssize_t fg2_rsoc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_RSOC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_RSOC]);
}
static CLASS_ATTR_RO(fg2_rsoc);

static ssize_t fg2_fastcharge_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_FAST_CHARGE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_FAST_CHARGE]);
}
static CLASS_ATTR_RO(fg2_fastcharge);

static ssize_t fg2_current_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_CURRENT_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_CURRENT_MAX]);
}
static CLASS_ATTR_RO(fg2_current_max);

static ssize_t fg2_vol_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_VOL_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_VOL_MAX]);
}
static CLASS_ATTR_RO(fg2_vol_max);

static ssize_t fg1_soc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_SOC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_SOC]);
}
static CLASS_ATTR_RO(fg1_soc);

static ssize_t fg2_soc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_SOC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_SOC]);
}
static CLASS_ATTR_RO(fg2_soc);

static ssize_t fg1_ibatt_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_IBATT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_IBATT]);
}
static CLASS_ATTR_RO(fg1_ibatt);

static ssize_t fg2_ibatt_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_IBATT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_IBATT]);
}
static CLASS_ATTR_RO(fg2_ibatt);

static ssize_t fg1_vol_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_VOL);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_VOL]);
}
static CLASS_ATTR_RO(fg1_vol);

static ssize_t fg2_vol_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_VOL);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_VOL]);
}
static CLASS_ATTR_RO(fg2_vol);

static ssize_t fg1_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TEMP]);
}
static CLASS_ATTR_RO(fg1_temp);

static ssize_t fg2_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_TEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_TEMP]);
}
static CLASS_ATTR_RO(fg2_temp);

static ssize_t fg2_oritemp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_ORITEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_ORITEMP]);
}
static CLASS_ATTR_RO(fg2_oritemp);

static ssize_t fg2_ai_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_AI);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_AI]);
}
static CLASS_ATTR_RO(fg2_ai);

static ssize_t fg1_fc_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_FC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_FC]);
}
static CLASS_ATTR_RO(fg1_fc);

static ssize_t fg2_fc_show(struct class *c,
		struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c,
			struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_FC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_FC]);
}
static CLASS_ATTR_RO(fg2_fc);

static ssize_t fg2_qmax_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_QMAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_QMAX]);
}
static CLASS_ATTR_RO(fg2_qmax);

static ssize_t fg2_tsim_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_TSIM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_TSIM]);
}
static CLASS_ATTR_RO(fg2_tsim);

static ssize_t fg2_tambient_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_TAMBIENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_TAMBIENT]);
}
static CLASS_ATTR_RO(fg2_tambient);

static ssize_t fg2_tremq_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_TREMQ);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_TREMQ]);
}
static CLASS_ATTR_RO(fg2_tremq);

static ssize_t fg2_tfullq_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG2_TFULLQ);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_TFULLQ]);
}
static CLASS_ATTR_RO(fg2_tfullq);

#endif

#if defined(CONFIG_MI_DTPT) && defined(CONFIG_DUAL_FUEL_GAUGE)
static ssize_t fg2_over_peak_flag_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_OVER_PEAK_FLAG);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_OVER_PEAK_FLAG]);
}
static CLASS_ATTR_RO(fg2_over_peak_flag);
static ssize_t fg2_current_deviation_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_CURRENT_DEVIATION);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_CURRENT_DEVIATION]);
}
static CLASS_ATTR_RO(fg2_current_deviation);
static ssize_t fg2_power_deviation_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_POWER_DEVIATION);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_POWER_DEVIATION]);
}
static CLASS_ATTR_RO(fg2_power_deviation);
static ssize_t fg2_average_current_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_AVERAGE_CURRENT);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_AVERAGE_CURRENT]);
}
static CLASS_ATTR_RO(fg2_average_current);
static ssize_t fg2_average_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_AVERAGE_TEMPERATURE);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_AVERAGE_TEMPERATURE]);
}
static CLASS_ATTR_RO(fg2_average_temp);
static ssize_t fg2_start_learn_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG2_START_LEARNING, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t fg2_start_learn_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_START_LEARNING);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_START_LEARNING]);
}
static CLASS_ATTR_RW(fg2_start_learn);
static ssize_t fg2_stop_learn_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG2_STOP_LEARNING, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t fg2_stop_learn_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_STOP_LEARNING);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_STOP_LEARNING]);
}
static CLASS_ATTR_RW(fg2_stop_learn);
static ssize_t fg2_set_learn_power_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG2_SET_LEARNING_POWER, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t fg2_set_learn_power_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_SET_LEARNING_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_SET_LEARNING_POWER]);
}
static CLASS_ATTR_RW(fg2_set_learn_power);
static ssize_t fg2_get_learn_power_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_LEARNING_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_LEARNING_POWER]);
}
static CLASS_ATTR_RO(fg2_get_learn_power);
static ssize_t fg2_get_learn_power_dev_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_LEARNING_POWER_DEV);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_LEARNING_POWER_DEV]);
}
static CLASS_ATTR_RO(fg2_get_learn_power_dev);
static ssize_t fg2_start_learn_b_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG2_START_LEARNING_B, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t fg2_start_learn_b_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_START_LEARNING_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_START_LEARNING_B]);
}
static CLASS_ATTR_RW(fg2_start_learn_b);
static ssize_t fg2_stop_learn_b_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG2_STOP_LEARNING_B, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t fg2_stop_learn_b_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_STOP_LEARNING_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_STOP_LEARNING_B]);
}
static CLASS_ATTR_RW(fg2_stop_learn_b);
static ssize_t fg2_set_learn_power_b_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG2_SET_LEARNING_POWER_B, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t fg2_set_learn_power_b_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_SET_LEARNING_POWER_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_SET_LEARNING_POWER_B]);
}
static CLASS_ATTR_RW(fg2_set_learn_power_b);
static ssize_t fg2_get_learn_power_b_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_LEARNING_POWER_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_LEARNING_POWER_B]);
}
static CLASS_ATTR_RO(fg2_get_learn_power_b);
static ssize_t fg2_get_learn_power_dev_b_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_LEARNING_POWER_DEV_B);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_LEARNING_POWER_DEV_B]);
}
static CLASS_ATTR_RO(fg2_get_learn_power_dev_b);
static ssize_t fg2_get_learn_time_dev_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_LEARNING_TIME_DEV);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_LEARNING_TIME_DEV]);
}
static CLASS_ATTR_RO(fg2_get_learn_time_dev);
static ssize_t fg2_constant_power_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG2_SET_CONSTANT_POWER, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t fg2_constant_power_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_SET_CONSTANT_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_SET_CONSTANT_POWER]);
}
static CLASS_ATTR_RW(fg2_constant_power);
static ssize_t fg2_remaining_time_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_REMAINING_TIME);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_REMAINING_TIME]);
}
static CLASS_ATTR_RO(fg2_remaining_time);
static ssize_t fg2_referance_power_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG2_SET_REFERANCE_POWER, val);
	if (rc < 0)
		return rc;
	return count;
}
static ssize_t fg2_referance_power_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_SET_REFERANCE_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_SET_REFERANCE_POWER]);
}
static CLASS_ATTR_RW(fg2_referance_power);
static ssize_t fg2_nvt_referance_current_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_REFERANCE_CURRENT);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_REFERANCE_CURRENT]);
}
static CLASS_ATTR_RO(fg2_nvt_referance_current);
static ssize_t fg2_nvt_referance_power_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_REFERANCE_POWER);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_REFERANCE_POWER]);
}
static CLASS_ATTR_RO(fg2_nvt_referance_power);
static ssize_t fg1_design_capacity_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG1_GET_DESIGN_CAPACITY);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_GET_DESIGN_CAPACITY]);
}
static CLASS_ATTR_RO(fg1_design_capacity);
static ssize_t fg2_design_capacity_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	rc = read_property_id(bcdev, pst, XM_PROP_FG2_GET_DESIGN_CAPACITY);
	if (rc < 0)
		return rc;
	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG2_GET_DESIGN_CAPACITY]);
}
static CLASS_ATTR_RO(fg2_design_capacity);
#endif

static struct attribute *battery_class_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_wireless_fw_crc.attr,
	&class_attr_wireless_fw_update_time_ms.attr,
	&class_attr_wireless_type.attr,
	&class_attr_wls_debug.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
	&class_attr_charge_control_en.attr,
	&class_attr_cp_mode.attr,
	&class_attr_bq2597x_chip_ok.attr,
	&class_attr_bq2597x_slave_chip_ok.attr,
	&class_attr_bq2597x_bus_current.attr,
	&class_attr_bq2597x_slave_bus_current.attr,
	&class_attr_bq2597x_bus_delta.attr,
	&class_attr_bq2597x_bus_voltage.attr,
	&class_attr_bq2597x_battery_present.attr,
	&class_attr_bq2597x_slave_battery_present.attr,
	&class_attr_bq2597x_battery_voltage.attr,
	&class_attr_bq2597x_battery_temp.attr,
	&class_attr_dam_ovpgate.attr,
	&class_attr_real_type.attr,
	&class_attr_thermal_board_temp.attr,
	&class_attr_thermal_scene.attr,
	&class_attr_ntc_alarm.attr,
	&class_attr_battcont_online.attr,
	&class_attr_connector_temp.attr,
#if defined(CONFIG_MI_N2N3ANTIBURN)
	&class_attr_anti_burn.attr,
#endif
	&class_attr_connector_temp2.attr,
	&class_attr_authentic.attr,
	&class_attr_bap_match.attr,
	&class_attr_verify_slave_flag.attr,
	&class_attr_verify_digest.attr,
	&class_attr_chip_ok.attr,
	&class_attr_resistance_id.attr,
	&class_attr_input_suspend.attr,
	&class_attr_fastchg_mode.attr,
	&class_attr_cc_orientation.attr,
	&class_attr_typec_mode.attr,
	&class_attr_mtbf_current.attr,

	&class_attr_quick_charge_type.attr,
	&class_attr_apdo_max.attr,
	&class_attr_soc_decimal.attr,
	&class_attr_soc_decimal_rate.attr,
	&class_attr_smart_batt.attr,
	&class_attr_smart_fv.attr,
	&class_attr_night_charging.attr,
	&class_attr_usbinterface.attr,
	&class_attr_request_vdm_cmd.attr,
	&class_attr_current_state.attr,
	&class_attr_adapter_id.attr,
	&class_attr_adapter_svid.attr,
	&class_attr_pd_verifed.attr,
	&class_attr_pdo2.attr,
	&class_attr_verify_process.attr,
#if defined (CONFIG_MI_MULTI_CHG_CFG)
	&class_attr_chg_cfg.attr,
#endif
	&class_attr_power_max.attr,
	&class_attr_thermal_remove.attr,

	&class_attr_fake_temp.attr,
	&class_attr_fake_cycle.attr,
  	&class_attr_afp_temp.attr,
	&class_attr_shutdown_delay.attr,
	&class_attr_shipmode_count_reset.attr,
	&class_attr_sport_mode.attr,

	&class_attr_double85.attr,
	&class_attr_soh_new.attr,
	&class_attr_remove_temp_limit.attr,
	&class_attr_cc_short_vbus.attr,
	&class_attr_otg_ui_support.attr,
	&class_attr_cid_status.attr,
	&class_attr_cc_toggle.attr,
	&class_attr_smart_chg.attr,
	&class_attr_low_fast_para.attr,
	&class_attr_lowfast_sw_para.attr,
	&class_attr_smart_sic_mode.attr,
	&class_attr_charger_user_value_map.attr,
	&class_attr_two_ntc_parameter1.attr,
	&class_attr_two_ntc_parameter2.attr,
	&class_attr_two_ntc_parameter3.attr,
	&class_attr_two_ntc_parameter4.attr,
	&class_attr_two_ntc_parameter5.attr,
	&class_attr_two_ntc_parameter6.attr,
	&class_attr_dot_test.attr,
	&class_attr_lpd_control.attr,
#if defined(CONFIG_MI_WIRELESS)
	&class_attr_tx_mac.attr,
	&class_attr_rx_cr.attr,
	&class_attr_rx_cep.attr,
	&class_attr_bt_state.attr,
#ifndef CONFIG_WIRELESS_REVERSE_CLOSE
	&class_attr_reverse_chg_mode.attr,
	&class_attr_reverse_chg_state.attr,
#endif
	&class_attr_wireless_chip_fw.attr,
	&class_attr_wireless_tx_uuid.attr,
	&class_attr_wireless_version_forcit.attr,
	&class_attr_wls_bin.attr,
	&class_attr_rx_vout.attr,
	&class_attr_rx_vrect.attr,
	&class_attr_rx_iout.attr,
	&class_attr_tx_adapter.attr,
	&class_attr_op_mode.attr,
	&class_attr_wls_die_temp.attr,
	&class_attr_wlscharge_control_limit.attr,
	&class_attr_wls_quick_chg_control_limit.attr,
	&class_attr_wls_thermal_remove.attr,
	&class_attr_wls_fw_state.attr,
	&class_attr_wls_car_adapter.attr,
	&class_attr_wls_tx_speed.attr,
	&class_attr_wls_fc_flag.attr,
	&class_attr_rx_ss.attr,
	&class_attr_rx_offset.attr,
	&class_attr_rx_sleep_mode.attr,
	&class_attr_low_inductance_offset.attr,
	&class_attr_set_rx_sleep.attr,
#if defined(CONFIG_MI_PEN_WIRELESS)
	&class_attr_pen_mac.attr,
	&class_attr_tx_iout.attr,
	&class_attr_tx_vout.attr,
	&class_attr_pen_soc.attr,
	&class_attr_pen_hall3.attr,
	&class_attr_pen_hall4.attr,
	&class_attr_pen_hall3_s.attr,
	&class_attr_pen_hall4_s.attr,
	&class_attr_pen_ppe_hall_n.attr,
	&class_attr_pen_ppe_hall_s.attr,
	&class_attr_pen_tx_ss.attr,
	&class_attr_pen_place_err.attr,
	&class_attr_fake_ss.attr,
#endif
#endif

	&class_attr_fg_vendor.attr,
	&class_attr_pack_vendor.attr,
	&class_attr_cell_vendor.attr,
	&class_attr_fg1_qmax.attr,
	&class_attr_fg1_rm.attr,
	&class_attr_fg1_fcc.attr,
	&class_attr_fg1_soh.attr,
	&class_attr_fg1_rsoc.attr,
	&class_attr_fg1_ai.attr,
	&class_attr_fg1_fcc_soh.attr,
	&class_attr_fg1_cycle.attr,
	&class_attr_fg1_fastcharge.attr,
	&class_attr_fg1_current_max.attr,
	&class_attr_fg1_vol_max.attr,
	&class_attr_fg1_tsim.attr,
	&class_attr_fg1_tambient.attr,
	&class_attr_fg1_tremq.attr,
	&class_attr_fg1_tfullq.attr,
	&class_attr_fg1_temp_max.attr,
	&class_attr_fg2_temp_max.attr,
	&class_attr_fg1_time_ot.attr,
	&class_attr_fg1_time_ht.attr,
	&class_attr_fg1_seal_set.attr,
	&class_attr_fg1_seal_state.attr,
	&class_attr_fg1_df_check.attr,
	&class_attr_fg1_imax.attr,
	&class_attr_fg1_oritemp.attr,
#if defined(CONFIG_MI_DTPT)
	&class_attr_battmoni_isc.attr,
	&class_attr_battmoni_soa.attr,
	&class_attr_over_peak_flag.attr,
	&class_attr_current_deviation.attr,
	&class_attr_power_deviation.attr,
	&class_attr_average_current.attr,
	&class_attr_average_temp.attr,
	&class_attr_start_learn.attr,
	&class_attr_stop_learn.attr,
	&class_attr_set_learn_power.attr,
	&class_attr_get_learn_power.attr,
	&class_attr_get_learn_power_dev.attr,
	&class_attr_get_learn_time_dev.attr,
	&class_attr_constant_power.attr,
	&class_attr_remaining_time.attr,
	&class_attr_referance_power.attr,
	&class_attr_nvt_referance_current.attr,
	&class_attr_nvt_referance_power.attr,
	&class_attr_start_learn_b.attr,
	&class_attr_stop_learn_b.attr,
	&class_attr_set_learn_power_b.attr,
	&class_attr_get_learn_power_b.attr,
	&class_attr_get_learn_power_dev_b.attr,
#endif
#if defined (CONFIG_DUAL_FUEL_GAUGE)
	&class_attr_slave_chip_ok.attr,
	&class_attr_slave_authentic.attr,
	&class_attr_fg2_fcc.attr,
	&class_attr_fg2_rm.attr,
	&class_attr_fg2_soh.attr,
	&class_attr_fg2_cycle.attr,
	&class_attr_fg2_fastcharge.attr,
	&class_attr_fg2_current_max.attr,
	&class_attr_fg2_vol_max.attr,
	&class_attr_fg2_rsoc.attr,
	&class_attr_fg1_soc.attr,
	&class_attr_fg2_soc.attr,
	&class_attr_fg1_ibatt.attr,
	&class_attr_fg2_ibatt.attr,
	&class_attr_fg1_vol.attr,
	&class_attr_fg2_vol.attr,
	&class_attr_fg1_temp.attr,
	&class_attr_fg2_temp.attr,
	&class_attr_fg2_oritemp.attr,
	&class_attr_fg2_ai.attr,
	&class_attr_fg1_fc.attr,
	&class_attr_fg2_fc.attr,
	&class_attr_fg2_qmax.attr,
	&class_attr_fg2_tsim.attr,
	&class_attr_fg2_tambient.attr,
	&class_attr_fg2_tremq.attr,
	&class_attr_fg2_tfullq.attr,
#endif
#if defined(CONFIG_MI_DTPT) && defined(CONFIG_DUAL_FUEL_GAUGE)
	&class_attr_fg2_over_peak_flag.attr,
	&class_attr_fg2_current_deviation.attr,
	&class_attr_fg2_power_deviation.attr,
	&class_attr_fg2_average_current.attr,
	&class_attr_fg2_average_temp.attr,
	&class_attr_fg2_start_learn.attr,
	&class_attr_fg2_stop_learn.attr,
	&class_attr_fg2_set_learn_power.attr,
	&class_attr_fg2_get_learn_power.attr,
	&class_attr_fg2_get_learn_power_dev.attr,
	&class_attr_fg2_get_learn_time_dev.attr,
	&class_attr_fg2_constant_power.attr,
	&class_attr_fg2_remaining_time.attr,
	&class_attr_fg2_referance_power.attr,
	&class_attr_fg2_nvt_referance_current.attr,
	&class_attr_fg2_nvt_referance_power.attr,
	&class_attr_fg2_start_learn_b.attr,
	&class_attr_fg2_stop_learn_b.attr,
	&class_attr_fg2_set_learn_power_b.attr,
	&class_attr_fg2_get_learn_power_b.attr,
	&class_attr_fg2_get_learn_power_dev_b.attr,
	&class_attr_fg1_design_capacity.attr,
	&class_attr_fg2_design_capacity.attr,
#endif
#if defined(CONFIG_MI_ENABLE_DP)
	&class_attr_has_dp.attr,
#endif
	&class_attr_sbu1.attr,
	&class_attr_sbu2.attr,
	&class_attr_cc1.attr,
	&class_attr_cc2.attr,
	&class_attr_plate_shock.attr,
	&class_attr_ui_soh.attr,
	&class_attr_ui_slave_soh.attr,
	&class_attr_soh_sn.attr,
	&class_attr_fg2_soh_sn.attr,
	&class_attr_calc_rvalue.attr,
	&class_attr_dp.attr,
	&class_attr_dm.attr,
	&class_attr_dis_uart.attr,
	&class_attr_lpd_charging.attr,
	&class_attr_maxtemp_occurtime.attr,
	&class_attr_runtime.attr,
	&class_attr_maxtemptime.attr,
	&class_attr_handle_state.attr,
	&class_attr_handle_stop_charging.attr,
	&class_attr_dod_count.attr,
#if defined(CONFIG_MI_SC760X)
	&class_attr_sc760x_chip_ok.attr,
	&class_attr_sc760x_slave_chip_ok.attr,
	&class_attr_sc760x_slave_ibatt_limit.attr,
#endif
	&class_attr_qbg_curr.attr,
	&class_attr_ut_test.attr,
#if defined(CONFIG_MI_O16U_EUPD) || defined(CONFIG_MI_O11_EMPTY_BATTERY)
	&class_attr_charger_partition_poweroffmode.attr,
	&class_attr_is_eu_model.attr,
	&class_attr_pps_ptf.attr,
#endif
#if defined(CONFIG_MI_O11_EMPTY_BATTERY)
	&class_attr_cp_fsw.attr,
	&class_attr_cp_iout.attr,
#endif
	&class_attr_cloud_fod.attr,
	&class_attr_last_node.attr,
	NULL,
};
ATTRIBUTE_GROUPS(battery_class);

static struct attribute *battery_class_usb_2_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_usb_2_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_moisture_detection_usb_2_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_wireless_fw_crc.attr,
	&class_attr_wireless_fw_update_time_ms.attr,
	&class_attr_wireless_type.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_2_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
	&class_attr_usb_2_typec_compliant.attr,
	&class_attr_charge_control_en.attr,
	NULL,
};
ATTRIBUTE_GROUPS(battery_class_usb_2);

#ifdef CONFIG_DEBUG_FS
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev)
{
	int rc;
	struct dentry *dir;

	dir = debugfs_create_dir("battery_charger", NULL);
	if (IS_ERR(dir)) {
		rc = PTR_ERR(dir);
		return;
	}

	bcdev->debugfs_dir = dir;
	debugfs_create_bool("block_tx", 0600, dir, &bcdev->block_tx);
}
#else
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev) { }
#endif

static void generate_xm_charge_uvent(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, xm_prop_change_work.work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int prop_id, rc;

	dev_err(bcdev->dev,"%s+++", __func__);

	kobject_uevent_env(&bcdev->dev->kobj, KOBJ_CHANGE, NULL);

	prop_id = get_property_id(pst, POWER_SUPPLY_PROP_PRESENT);
	if (prop_id < 0)
		return;
	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return;
	bcdev->boost_mode = pst->prop[WLS_BOOST_EN];

	return;
}

static void generate_xm_smartchg_uvent(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, xm_smart_prop_update_work.work);
	char uevent_buf[MAX_STR_LEN] = { 0 };
	char *envp[2] = { uevent_buf, NULL };
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	dev_err(bcdev->dev,"%s+++", __func__);

	rc = read_property_id(bcdev, pst, XM_PROP_SMART_SIC_MODE);
	if (rc < 0) {
		return;
	}

	snprintf(uevent_buf, MAX_STR_LEN, "POWER_SUPPLY_SMART_SIC_MODE=%d", pst->prop[XM_PROP_SMART_SIC_MODE]);
	rc = kobject_uevent_env(&bcdev->dev->kobj, KOBJ_CHANGE, envp);
	if (rc < 0)
		pr_err("notify uevent fail, rc=%d\n", rc);
}

#define CHARGING_PERIOD_S		30
static void xm_charger_debug_info_print_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, charger_debug_info_print_work.work);
	struct power_supply *usb_psy = NULL;
	struct power_supply *wls_psy = NULL;
	int rc, usb_present = 0, wls_present = 0;
	int vbus_vol_uv = 0, ibus_ua = 0;
	int interval = CHARGING_PERIOD_S;
	union power_supply_propval val = {0, };
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];

	usb_psy = bcdev->psy_list[PSY_TYPE_USB].psy;
	if (usb_psy != NULL) {
		rc = usb_psy_get_prop(usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (!rc)
			usb_present = val.intval;
		else
			usb_present = 0;
	} else {
		return;
	}

#if defined(CONFIG_MI_WIRELESS)
	wls_psy = bcdev->psy_list[PSY_TYPE_WLS].psy;
	if (wls_psy != NULL) {
		rc = wls_psy_get_prop(wls_psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (!rc)
			wls_present = val.intval;
		else
			wls_present = 0;
	} else {
		wls_present = 0;
	}
#endif

	if ((usb_present == 1) || (wls_present == 1)) {

		rc = read_property_id(bcdev, pst, XM_PROP_FG_VENDOR_ID);
#if defined(CONFIG_MI_O16U_EUPD) || defined(CONFIG_MI_O11_EMPTY_BATTERY)
		/*add for EEA 100W*/
		rc = read_property_id(bcdev, pst, XM_PROP_IS_EU_MODEL);
		rc = read_property_id(bcdev, pst, XM_PROP_PPS_PTF);
#endif

		if (usb_present == 1) {
			rc = usb_psy_get_prop(usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
			if (!rc)
			      vbus_vol_uv = val.intval;
			else
			      vbus_vol_uv = 0;

			rc = usb_psy_get_prop(usb_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
			if (!rc)
			      ibus_ua = val.intval;
			else
			      ibus_ua = 0;

		} else if(wls_present == 1) {
			rc = wls_psy_get_prop(wls_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
			if (!rc)
			      vbus_vol_uv = val.intval;
			else
			      vbus_vol_uv = 0;

			rc = wls_psy_get_prop(wls_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
			if (!rc)
			      ibus_ua = val.intval;
			else
			      ibus_ua = 0;
#if defined(CONFIG_MI_WIRELESS)
			rc = read_property_id(bcdev, pst, XM_PROP_RX_VOUT);
			rc = read_property_id(bcdev, pst, XM_PROP_RX_IOUT);
			rc = read_property_id(bcdev, pst, XM_PROP_WLS_FC_FLAG);
#endif	
		}

		rc = read_property_id(bcdev, pst, XM_PROP_MTBF_CURRENT);
		if (!rc && pst->prop[XM_PROP_MTBF_CURRENT] != bcdev->mtbf_current) {
			rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
					XM_PROP_MTBF_CURRENT, bcdev->mtbf_current);
		}

		rc = read_property_id(bcdev, pst, XM_PROP_AUTHENTIC);
		if (!rc && !pst->prop[XM_PROP_AUTHENTIC] && bcdev->battery_auth) {
			rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
					XM_PROP_AUTHENTIC, bcdev->battery_auth);
		}

		interval = CHARGING_PERIOD_S;
		schedule_delayed_work(&bcdev->charger_debug_info_print_work, interval * HZ);
		bcdev->debug_work_en = 1;
	} else {
		bcdev->debug_work_en = 0;
	}
}

static int battery_chg_parse_dt(struct battery_chg_dev *bcdev)
{
	struct device_node *node = bcdev->dev->of_node;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	of_property_read_string(node, "qcom,wireless-fw-name",
				&bcdev->wls_fw_name);

	of_property_read_u32(node, "qcom,shutdown-voltage",
				&bcdev->shutdown_volt_mv);

	bcdev->num_thermal_levels = MAX_THERMAL_LEVEL;
	bcdev->thermal_fcc_ua = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	bcdev->shutdown_delay_en = of_property_read_bool(node, "mi,support-shutdown-delay");
	bcdev->fast_update_temp = of_property_read_bool(node, "mi,support-fast-update-timer");
	bcdev->ut_test = of_property_read_bool(node, "mi,support-ut-test");
	bcdev->ut_test_region = of_property_read_bool(node, "mi,support-ut-test-region");

	if (bcdev->ut_test_region) {
		of_property_read_string(node, "mi,cycle_volt_gl", &bcdev->cycle_volt_gl);
		of_property_read_string(node, "mi,cycle_step_curr_gl", &bcdev->cycle_step_curr_gl);
		of_property_read_string(node, "mi,thermal_gl", &bcdev->thermal_gl);
		of_property_read_string(node, "mi,temp_term_curr_gl_nvt", &bcdev->temp_term_curr_gl_nvt);
		of_property_read_string(node, "mi,temp_term_curr_gl_sunwoda", &bcdev->temp_term_curr_gl_sunwoda);

		of_property_read_string(node, "mi,cycle_volt_cn", &bcdev->cycle_volt_cn);
		of_property_read_string(node, "mi,cycle_step_curr_cn", &bcdev->cycle_step_curr_cn);
		of_property_read_string(node, "mi,thermal_cn", &bcdev->thermal_cn);
		of_property_read_string(node, "mi,temp_term_curr_cn_nvt", &bcdev->temp_term_curr_cn_nvt);
		of_property_read_string(node, "mi,temp_term_curr_cn_sunwoda", &bcdev->temp_term_curr_cn_sunwoda);
	}
	of_property_read_string(node, "mi,cycle_volt", &bcdev->cycle_volt);
	of_property_read_string(node, "mi,cycle_step_curr", &bcdev->cycle_step_curr);
	of_property_read_string(node, "mi,temp_term_curr", &bcdev->temp_term_curr);
	of_property_read_string(node, "mi,thermal", &bcdev->thermal);

	return 0;
}

#define FW_UPDATE_NUM 10
static int battery_chg_shutdown(struct notifier_block *nb, unsigned long code,
		void *unused)
{
	struct battery_charger_shutdown_req_msg msg = { { 0 } };
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     shutdown_notifier);

#if defined(CONFIG_MI_WIRELESS)
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
#endif

	int rc,shipmode,afp_temp;
  	char prop_buf1[10];
  	char prop_buf2[10];

  	shipmode_count_reset_show(&(bcdev->battery_class), NULL, prop_buf1);
  	if (kstrtoint(prop_buf1, 10, &shipmode))
  		return -EINVAL;
  	if (shipmode == 1)
  		pr_err("enter the ship mode,the result is: %d\n", shipmode);
  	else
  		pr_err("failed to enter the ship mode\n");

  	afp_temp_show(&(bcdev->battery_class), NULL, prop_buf2);
  	if (kstrtoint(prop_buf2, 10, &afp_temp))
  		return -EINVAL;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHUTDOWN_REQ_SET;

	if (code == SYS_POWER_OFF || code == SYS_RESTART) {
		;

#if defined(CONFIG_MI_WIRELESS)
		rc = read_property_id(bcdev, pst, XM_PROP_WLS_FW_STATE);
		if(rc < 0)
			pr_err("Failed to get wls_pst!\n");
		else
		{
			int fw_update_cnt = 0;
			while (pst->prop[XM_PROP_WLS_FW_STATE] == 2 && fw_update_cnt++ < FW_UPDATE_NUM)
			{
				msleep(1000);
				rc = read_property_id(bcdev, pst, XM_PROP_WLS_FW_STATE);
			}
		}
#endif

		rc = battery_chg_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			;
	}

	return NOTIFY_DONE;
}

static int battery_chg_ship_mode(struct notifier_block *nb, unsigned long code,
		void *unused)
{
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     reboot_notifier);
	int rc;

	if (!bcdev->ship_mode_en)
		return NOTIFY_DONE;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHIP_MODE_REQ_SET;
	msg.ship_mode_type = SHIP_MODE_PMIC;

	if (code == SYS_POWER_OFF) {
		rc = battery_chg_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			pr_emerg("Failed to write ship mode: %d\n", rc);
	}

	return NOTIFY_DONE;
}
#if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
static void screen_state_for_charge_callback (enum panel_event_notifier_tag tag,
			struct panel_event_notification *notification, void *data)
{
	struct battery_chg_dev *bcdev = data;

	if (!notification) {
		return;
	}

	if (notification->notif_data.early_trigger)
		return;

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_BLANK:
		//battery_chg_notify_disable(bcdev);
		bcdev->blank_state = 1; 
		schedule_work(&bcdev->lpd_status_work);
		break;
	case DRM_PANEL_EVENT_UNBLANK:
		//battery_chg_notify_enable(bcdev);
		bcdev->blank_state = 0;
		schedule_work(&bcdev->lpd_status_work);
		break;
	case DRM_PANEL_EVENT_FPS_CHANGE:
		return;
	default:
		break;
	}
}

static int charge_check_panel(struct device_node *np)
{
	int i;
	int count;
	struct device_node *pnode;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0) {
		return 0;
	}


	for (i = 0; i < count; i++) {
		pnode = of_parse_phandle(np, "panel", i);
        if (!pnode) {
			return 0;
		}
		panel = of_drm_find_panel(pnode);
		of_node_put(pnode);
		if (!IS_ERR(panel)) {
			active_panel = panel;
			return 0;
		}else{
			active_panel = NULL;
		}
	}
	return PTR_ERR(panel);
}


static void battery_register_panel_notifier_work(struct work_struct *work)
{
	struct device_node *pnode;
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, panel_notify_register_work.work);
	int error = 0;
	static int retry_count = 3;
    static void *cookie = NULL;

	pnode = of_find_node_by_name(NULL, "charge-screen");
	if (!pnode) {
		pr_err("%s ERROR: Cannot find node with panel!", __func__);
		return;
	}

	error = charge_check_panel(pnode);
	if (error == -EPROBE_DEFER)
		pr_err("%s ERROR: Cannot fine panel of node!", __func__);

	if (active_panel) {
		cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
				PANEL_EVENT_NOTIFIER_CLIENT_BATTERY_CHARGER, active_panel,
				screen_state_for_charge_callback, (void *)bcdev);
		if (IS_ERR(cookie))
			printk(KERN_ERR "%s:Failed to register for panel events\n", __func__);
		else
			printk(KERN_ERR "%s:panel_event_notifier_register register succeed\n", __func__);
	} else if(retry_count > 0) {
		printk(KERN_ERR "%s:active_panel is NULL Failed to register for panel events\n", __func__);
		retry_count--;
		schedule_delayed_work(&bcdev->panel_notify_register_work, msecs_to_jiffies(5000));

	}
    bcdev->notifier_cookie = cookie;

}
#endif
static int
battery_chg_get_max_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct battery_chg_dev *bcdev = tcd->devdata;

	*state = bcdev->num_thermal_levels;

	return 0;
}

static int
battery_chg_get_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct battery_chg_dev *bcdev = tcd->devdata;

	*state = bcdev->curr_thermal_level;

	return 0;
}

static int
battery_chg_set_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct battery_chg_dev *bcdev = tcd->devdata;

	return battery_psy_set_charge_current(bcdev, (int)state);
}

static const struct thermal_cooling_device_ops battery_tcd_ops = {
	.get_max_state = battery_chg_get_max_charge_cntl_limit,
	.get_cur_state = battery_chg_get_cur_charge_cntl_limit,
	.set_cur_state = battery_chg_set_cur_charge_cntl_limit,
};

#define MAX_UEVENT_LENGTH 50
static int add_xiaomi_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct battery_chg_dev *bcdev = platform_get_drvdata(pdev);

	char *prop_buf = NULL;
	char uevent_string[MAX_UEVENT_LENGTH+1];
	u32 i = 0;

#if defined(CONFIG_MI_PEN_WIRELESS)
	int val;
#endif

	prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!prop_buf)
		return 0;

	/*add our prop start*/
#if defined(CONFIG_MI_WIRELESS)
#ifndef CONFIG_WIRELESS_REVERSE_CLOSE
	reverse_chg_state_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_REVERSE_CHG_STATE=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	reverse_chg_mode_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_REVERSE_CHG_MODE=%s", prop_buf);
	add_uevent_var(env, uevent_string);
#endif

#if defined(CONFIG_MI_PEN_WIRELESS)
	reverse_chg_state_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_REVERSE_PEN_CHG_STATE=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	pen_hall3_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_PEN_HALL3=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	pen_hall4_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_PEN_HALL4=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	pen_soc_show( &(bcdev->battery_class), NULL, prop_buf);
	if (!kstrtoint(prop_buf, 10, &val)) {
		pr_err("kstrtoint pen_soc:%02x", val);
		if (val != 0xff) {
			snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_REVERSE_PEN_SOC=%d", val);
			add_uevent_var(env, uevent_string);
		}
	}

	pen_mac_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_PEN_MAC=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	pen_place_err_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_PEN_PLACE_ERR=%s", prop_buf);
	add_uevent_var(env, uevent_string);
#endif

	wls_fw_state_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_WLS_FW_STATE=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	wls_car_adapter_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_WLS_CAR_ADAPTER=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	tx_adapter_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_TX_ADAPTER=%s", prop_buf);
	add_uevent_var(env, uevent_string);


	rx_offset_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_RX_OFFSET=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	low_inductance_offset_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_LOW_INDUCTANCE_OFFSET=%s", prop_buf);
	add_uevent_var(env, uevent_string);

#endif

	soc_decimal_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_SOC_DECIMAL=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	soc_decimal_rate_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_SOC_DECIMAL_RATE=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	quick_charge_type_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_QUICK_CHARGE_TYPE=%s", prop_buf);
	add_uevent_var(env, uevent_string);


	shutdown_delay_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_SHUTDOWN_DELAY=%s", prop_buf);
	add_uevent_var(env, uevent_string);


	connector_temp_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_CONNECTOR_TEMP=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	moisture_detection_status_show( &(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_MOISTURE_DET_STS=%s", prop_buf);
	add_uevent_var(env, uevent_string);


	cc_short_vbus_show(&(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_CC_SHORT_VBUS=%s", prop_buf);
	add_uevent_var(env, uevent_string);

#if defined(CONFIG_MI_N2N3ANTIBURN)
	anti_burn_show(&(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_ANTI_BURN=%s", prop_buf);
	add_uevent_var(env, uevent_string);
#endif

	ntc_alarm_show(&(bcdev->battery_class), NULL, prop_buf);
	snprintf(uevent_string, MAX_UEVENT_LENGTH, "POWER_SUPPLY_NTC_ALARM=%s", prop_buf);
	add_uevent_var(env, uevent_string);

	dev_err(bcdev->dev," %s ", env->envp[env->envp_idx -1]);


	dev_err(bcdev->dev,"currnet uevent info :");
	for(i = 0; i < env->envp_idx; ++i){
#ifndef CONFIG_MI_PEN_WIRELESS
		if(i <= 9 || (i >= 12 && i <= 16 && i != 14))
			continue;
#endif
	    dev_err(bcdev->dev," %s ", env->envp[i]);
	}

	free_page((unsigned long)prop_buf);
	return 0;
}

static struct device_type dev_type_xiaomi_uevent = {
	.name = "dev_type_xiaomi_uevent",
	.uevent = add_xiaomi_uevent,
};

static int charger_notifier_event(struct notifier_block *notifier,
			unsigned long chg_event, void *val)
{
	struct battery_chg_dev *bcdev;
	struct power_supply	*batt_psy;
	int rc = 0;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("Failed to get battery supply\n");
		return -EPROBE_DEFER;
	}

	bcdev = power_supply_get_drvdata(batt_psy);
	if (!bcdev)
		return -ENODEV;

	switch (chg_event) {
	case THERMAL_BOARD_TEMP:
		bcdev->thermal_board_temp = *(int *)val;
		rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM], XM_PROP_THERMAL_BOARD_TEMP, bcdev->thermal_board_temp);
		if (rc < 0)
			return rc;
		break;
	case THERMAL_SCENE:
		bcdev->thermal_scene = *(int *)val;
		xm_handle_smartchg_scene_upload(bcdev, bcdev->thermal_scene);
		rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM], XM_PROP_THERMAL_SCENE, bcdev->thermal_scene);
		if (rc < 0)
			return rc;
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static int battery_chg_probe(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev;
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data = { };
	struct thermal_cooling_device *tcd;
	struct psy_state *pst;
	int rc, i;

	dev_err(dev, "battery_chg probe start\n");
	bcdev = devm_kzalloc(&pdev->dev, sizeof(*bcdev), GFP_KERNEL);
	if (!bcdev)
		return -ENOMEM;

	bcdev->psy_list[PSY_TYPE_BATTERY].map = battery_prop_map;
	bcdev->psy_list[PSY_TYPE_BATTERY].prop_count = BATT_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_get = BC_BATTERY_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_set = BC_BATTERY_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_USB].map = usb_prop_map;
	bcdev->psy_list[PSY_TYPE_USB].prop_count = USB_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_USB].opcode_get = BC_USB_STATUS_GET(USB_1_PORT_ID);
	bcdev->psy_list[PSY_TYPE_USB].opcode_set = BC_USB_STATUS_SET(USB_1_PORT_ID);
	bcdev->psy_list[PSY_TYPE_WLS].map = wls_prop_map;
	bcdev->psy_list[PSY_TYPE_WLS].prop_count = WLS_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_get = BC_WLS_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_set = BC_WLS_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_XM].map = xm_prop_map;
	bcdev->psy_list[PSY_TYPE_XM].prop_count = XM_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_XM].opcode_get = BC_XM_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_XM].opcode_set = BC_XM_STATUS_SET;
	bcdev->usb_active[USB_1_PORT_ID] = true;

	bcdev->has_usb_2 = of_property_read_bool(dev->of_node, "qcom,multiport-usb");
	if (bcdev->has_usb_2) {
		bcdev->psy_list[PSY_TYPE_USB_2].map = usb_prop_map;
		bcdev->psy_list[PSY_TYPE_USB_2].prop_count = USB_PROP_MAX;
		bcdev->psy_list[PSY_TYPE_USB_2].opcode_get = BC_USB_STATUS_GET(USB_2_PORT_ID);
		bcdev->psy_list[PSY_TYPE_USB_2].opcode_set = BC_USB_STATUS_SET(USB_2_PORT_ID);
		bcdev->usb_active[USB_2_PORT_ID] = true;
	}

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		bcdev->psy_list[i].prop =
			devm_kcalloc(&pdev->dev, bcdev->psy_list[i].prop_count,
					sizeof(u32), GFP_KERNEL);
		if (!bcdev->psy_list[i].prop)
			return -ENOMEM;
	}

	bcdev->psy_list[PSY_TYPE_BATTERY].model =
		devm_kzalloc(&pdev->dev, MAX_STR_LEN, GFP_KERNEL);
	if (!bcdev->psy_list[PSY_TYPE_BATTERY].model)
		return -ENOMEM;

	bcdev->digest=
		devm_kzalloc(&pdev->dev, BATTERY_DIGEST_LEN, GFP_KERNEL);
	if (!bcdev->digest)
		return -ENOMEM;
	bcdev->ss_auth_data=
		devm_kzalloc(&pdev->dev, BATTERY_SS_AUTH_DATA_LEN * sizeof(u32), GFP_KERNEL);
	if (!bcdev->ss_auth_data)
		return -ENOMEM;

	mutex_init(&bcdev->rw_lock);
	init_rwsem(&bcdev->state_sem);
	init_completion(&bcdev->ack);
	init_completion(&bcdev->fw_buf_ack);
	init_completion(&bcdev->fw_update_ack);
	INIT_WORK(&bcdev->subsys_up_work, battery_chg_subsys_up_work);
	INIT_WORK(&bcdev->usb_type_work, battery_chg_update_usb_type_work);
	INIT_WORK(&bcdev->battery_check_work, battery_chg_check_status_work);
	/* pen_connect_strategy start */
	INIT_WORK( &bcdev->pen_notifier_work, pen_charge_notifier_work);
	/* pen_connect_strategy end */
	INIT_WORK(&bcdev->lpd_status_work, usb_chg_lpd_check_work);
	INIT_DELAYED_WORK(&bcdev->charger_debug_info_print_work, xm_charger_debug_info_print_work);
	INIT_DELAYED_WORK( &bcdev->xm_prop_change_work, generate_xm_charge_uvent);
	INIT_DELAYED_WORK( &bcdev->xm_smart_prop_update_work, generate_xm_smartchg_uvent);
	INIT_DELAYED_WORK( &bcdev->batt_update_work, xm_batt_update_work);
	INIT_DELAYED_WORK( &bcdev->glink_crash_num_work, xm_glink_crash_num_work);

#if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
	INIT_DELAYED_WORK(&bcdev->panel_notify_register_work, battery_register_panel_notifier_work);
#endif
	bcdev->dev = dev;

	client_data.id = MSG_OWNER_BC;
	client_data.name = "battery_charger";
	client_data.msg_cb = battery_chg_callback;
	client_data.priv = bcdev;
	client_data.state_cb = battery_chg_state_cb;

	bcdev->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(bcdev->client)) {
		rc = PTR_ERR(bcdev->client);
		if (rc != -EPROBE_DEFER)
			dev_err(dev, "Error in registering with pmic_glink %d\n",
				rc);
		goto reg_error;
	}

	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_UP);
	/*
	 * This should be initialized here so that battery_chg_callback
	 * can run successfully when battery_chg_parse_dt() starts
	 * reading BATT_CHG_CTRL_LIM_MAX parameter and waits for a response.
	 */
	bcdev->initialized = true;
	up_write(&bcdev->state_sem);

	bcdev->reboot_notifier.notifier_call = battery_chg_ship_mode;
	bcdev->reboot_notifier.priority = 255;
	register_reboot_notifier(&bcdev->reboot_notifier);

	/* register shutdown notifier to do something before shutdown */
  	bcdev->shutdown_notifier.notifier_call = battery_chg_shutdown;
  	bcdev->shutdown_notifier.priority = 255;
  	register_reboot_notifier(&bcdev->shutdown_notifier);

	rc = battery_chg_parse_dt(bcdev);
	if (rc < 0) {
		dev_err(dev, "Failed to parse dt rc=%d\n", rc);
		goto error;
	}

	bcdev->restrict_fcc_ua = DEFAULT_RESTRICT_FCC_UA;
	platform_set_drvdata(pdev, bcdev);
	bcdev->fake_soc = -EINVAL;
	rc = battery_chg_init_psy(bcdev);
	if (rc < 0)
		goto error;
	bcdev->battery_class.name = "qcom-battery";

	if (bcdev->has_usb_2)
		bcdev->battery_class.class_groups = battery_class_usb_2_groups;
	else
		bcdev->battery_class.class_groups = battery_class_groups;

	rc = class_register(&bcdev->battery_class);
	if (rc < 0) {
		dev_err(dev, "Failed to create battery_class rc=%d\n", rc);
		goto error;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	tcd = devm_thermal_of_cooling_device_register(dev, dev->of_node,
			(char *)pst->psy->desc->name, bcdev, &battery_tcd_ops);
	if (IS_ERR_OR_NULL(tcd)) {
		rc = PTR_ERR_OR_ZERO(tcd);
		dev_err(dev, "Failed to register thermal cooling device rc=%d\n",
			rc);
		class_unregister(&bcdev->battery_class);
		goto error;
	}

	bcdev->wls_fw_update_time_ms = WLS_FW_UPDATE_TIME_MS;
	battery_chg_add_debugfs(bcdev);
	bcdev->notify_en = false;
	battery_chg_notify_enable(bcdev);
	device_init_wakeup(bcdev->dev, true);
	schedule_work(&bcdev->usb_type_work);

	schedule_delayed_work(&bcdev->charger_debug_info_print_work, 5 * HZ);
	schedule_delayed_work(&bcdev->batt_update_work, 0);
#if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
	schedule_delayed_work(&bcdev->panel_notify_register_work, msecs_to_jiffies(5000));
#endif
	bcdev->debug_work_en = 1;
	dev->type = &dev_type_xiaomi_uevent;
	strcpy(bcdev->wireless_chip_fw_version, "00.00.00.00");
	strcpy(bcdev->wireless_tx_uuid_version, "00.00.00.00");
	bcdev->battery_auth = false;
	bcdev->slave_fg_verify_flag = false;
	bcdev->mtbf_current = 0;
	bcdev->reverse_chg_flag = 0;
	bcdev->glink_crash_count = 0;
	bcdev->read_capacity_timeout = false;
	bcdev->glink_crash_timeout_reset_flag = false;

	bcdev->chg_nb.notifier_call = charger_notifier_event;
	charger_reg_notifier(&bcdev->chg_nb);

	dev_err(dev, "battery_chg probe done\n");
	rc = get_charge_control_en(bcdev);
	if (rc < 0)
		pr_debug("Failed to read charge_control_en, rc = %d\n", rc);

	return 0;
error:
	dev_err(dev, "battery_chg probe error\n");
	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_DOWN);
	bcdev->initialized = false;
	up_write(&bcdev->state_sem);

	pmic_glink_unregister_client(bcdev->client);
	cancel_work_sync(&bcdev->usb_type_work);
	cancel_work_sync(&bcdev->subsys_up_work);
	cancel_work_sync(&bcdev->battery_check_work);
#if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
	cancel_delayed_work_sync(&bcdev->panel_notify_register_work);
#endif
	complete(&bcdev->ack);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	unregister_reboot_notifier(&bcdev->shutdown_notifier);
reg_error:
 	dev_err(dev, "battery_chg probe error: pmic_glink_register_client FAILED\n");
	
	return rc;
}

static int battery_chg_remove(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev = platform_get_drvdata(pdev);

	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_DOWN);
	bcdev->initialized = false;
	up_write(&bcdev->state_sem);

	device_init_wakeup(bcdev->dev, false);
	debugfs_remove_recursive(bcdev->debugfs_dir);
	class_unregister(&bcdev->battery_class);
	pmic_glink_unregister_client(bcdev->client);
	cancel_work_sync(&bcdev->subsys_up_work);
	cancel_work_sync(&bcdev->usb_type_work);
	cancel_work_sync(&bcdev->battery_check_work);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	unregister_reboot_notifier(&bcdev->shutdown_notifier);
 #if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
	cancel_delayed_work_sync(&bcdev->panel_notify_register_work);
	if (active_panel && !IS_ERR(bcdev->notifier_cookie)) {
 		panel_event_notifier_unregister(bcdev->notifier_cookie);
	} else {
		printk(KERN_ERR "%s:prim_panel_event_notifier_unregister falt\n", __func__);
	}
 #endif

	return 0;
}

static const struct of_device_id battery_chg_match_table[] = {
	{ .compatible = "qcom,battery-charger" },
	{},
};

static struct platform_driver battery_chg_driver = {
	.driver = {
		.name = "qti_battery_charger",
		.of_match_table = battery_chg_match_table,
	},
	.probe = battery_chg_probe,
	.remove = battery_chg_remove,
};
module_platform_driver(battery_chg_driver);

MODULE_DESCRIPTION("QTI Glink battery charger driver");
MODULE_LICENSE("GPL v2");
