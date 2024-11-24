/*******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2016 MediaTek Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/
/*
 * Log: gl_vendor.h
 *
 * 10 14 2014
 * add vendor declaration
 *
 *
 */

#ifndef _GL_VENDOR_H
#define _GL_VENDOR_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>
#include <linux/can/netlink.h>
#include <net/netlink.h>
#include "wlan_lib.h"
#include "gl_wext.h"


/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define GOOGLE_OUI 0x001A11
#define OUI_QCA 0x001374
#define OUI_MTK 0x000CE7

#define NL80211_VENDOR_SUBCMD_GET_PREFER_FREQ_LIST 103
#define NL80211_VENDOR_SUBCMD_ACS 54
#define NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_STARTED 56
#define NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_FINISHED 57
#define NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_ABORTED 58
#define NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_NOP_FINISHED 59
#define NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_RADAR_DETECTED 60
#define NL80211_VENDOR_SUBCMD_DFS_CAPABILITY 11
#define NL80211_VENDOR_SUBCMD_GET_FEATURES 55
#define QCA_NL80211_VENDOR_SUBCMD_ROAM 64
#define QCA_WLAN_VENDOR_ATTR_SETBAND_VALUE 12
#define QCA_WLAN_VENDOR_ATTR_SETBAND_MASK 43
#define QCA_WLAN_VENDOR_ATTR_MAX 44
#define QCA_NL80211_VENDOR_SUBCMD_SETBAND 105
#define NL80211_VENDOR_SUBCMD_NAN 12
#define NL80211_VENDOR_SUBCMD_GET_APF_CAPABILITIES 14
#define NL80211_VENDOR_SUBCMD_SET_PACKET_FILTER 15
#define NL80211_VENDOR_SUBCMD_READ_PACKET_FILTER 16
#define NL80211_VENDOR_SUBCMD_NDP 81
#define NL80211_VENDOR_SUBCMD_GET_TRX_STATS 48
#define MTK_NL80211_OP_MODE_CHANGE 14
#define MTK_NL80211_TRIGGER_RESET 15

#define WIFI_VENDOR_ATTR_FEATURE_FLAGS 7
#define WIFI_VENDOR_DATA_OP_MODE_CHANGE(bssIdx, channelBw, TxNss, RxNss) \
	(uint32_t)((((bssIdx) << 24) + ((channelBw) << 16) + \
		((TxNss) << 8) + (RxNss)) & 0xffff)

enum NL80211_VENDOR_FEATURES {
	VENDOR_FEATURE_KEY_MGMT_OFFLOAD        = 0,
	VENDOR_FEATURE_SUPPORT_HW_MODE_ANY     = 1,
	VENDOR_FEATURE_OFFCHANNEL_SIMULTANEOUS = 2,
	VENDOR_FEATURE_P2P_LISTEN_OFFLOAD      = 3,
	VENDOR_FEATURE_OCE_STA                 = 4,
	VENDOR_FEATURE_OCE_AP                  = 5,
	VENDOR_FEATURE_OCE_STA_CFON            = 6,
	NUM_VENDOR_FEATURES /* keep last */
};

enum ANDROID_VENDOR_SUB_COMMAND {
	/* Don't use 0 as a valid subcommand */
	ANDROID_NL80211_SUBCMD_UNSPECIFIED,

	/* Define all vendor startup commands between 0x0 and 0x0FFF */
	ANDROID_NL80211_SUBCMD_WIFI_RANGE_START = 0x0001,
	ANDROID_NL80211_SUBCMD_WIFI_RANGE_END	= 0x0FFF,

	/* Define all GScan related commands between 0x1000 and 0x10FF */
	ANDROID_NL80211_SUBCMD_GSCAN_RANGE_START = 0x1000,
	ANDROID_NL80211_SUBCMD_GSCAN_RANGE_END	 = 0x10FF,

	/* Define all RTT related commands between 0x1100 and 0x11FF */
	ANDROID_NL80211_SUBCMD_RTT_RANGE_START = 0x1100,
	ANDROID_NL80211_SUBCMD_RTT_RANGE_END   = 0x11FF,

	ANDROID_NL80211_SUBCMD_LSTATS_RANGE_START = 0x1200,
	ANDROID_NL80211_SUBCMD_LSTATS_RANGE_END   = 0x12FF,

	/* Define all Logger related commands between 0x1400 and 0x14FF */
	ANDROID_NL80211_SUBCMD_DEBUG_RANGE_START = 0x1400,
	ANDROID_NL80211_SUBCMD_DEBUG_RANGE_END	 = 0x14FF,

	/* Define all wifi offload related commands between 0x1600 and 0x16FF */
	ANDROID_NL80211_SUBCMD_WIFI_OFFLOAD_RANGE_START = 0x1600,
	ANDROID_NL80211_SUBCMD_WIFI_OFFLOAD_RANGE_END	= 0x16FF,

	/* This is reserved for future usage */

};

enum WIFI_SUB_COMMAND {
	WIFI_SUBCMD_GET_CHANNEL_LIST = ANDROID_NL80211_SUBCMD_WIFI_RANGE_START,

	WIFI_SUBCMD_GET_FEATURE_SET,				/* 0x0002 */
	WIFI_SUBCMD_GET_FEATURE_SET_MATRIX,			/* 0x0003 */
	WIFI_SUBCMD_SET_PNO_RANDOM_MAC_OUI,			/* 0x0004 */
	WIFI_SUBCMD_NODFS_SET,					/* 0x0005 */
	WIFI_SUBCMD_SET_COUNTRY_CODE,				/* 0x0006 */
	WIFI_SUBCMD_SET_RSSI_MONITOR,				/* 0x0007 */

	/* Add more sub commands here */
	WIFI_SUBCMD_GET_ROAMING_CAPABILITIES,			/* 0x0008 */
	WIFI_SUBCMD_SET_ROAMING = 0x0009,			/* 0x0009 */
	WIFI_SUBCMD_CONFIG_ROAMING = 0x000a,			/* 0x000a */
	WIFI_SUBCMD_ENABLE_ROAMING,				/* 0x000b */
	WIFI_SUBCMD_SELECT_TX_POWER_SCENARIO,			/* 0x000c */
	WIFI_SUBCMD_SET_MULTISTA_PRIMARY_CONNECTION,		/* 0x000d */
	WIFI_SUBCMD_SET_MULTISTA_USE_CASE,			/* 0x000e */
	WIFI_SUBCMD_SET_SCAN_PARAM,				/* 0x000f */
};

enum RTT_SUB_COMMAND {
	RTT_SUBCMD_SET_CONFIG = ANDROID_NL80211_SUBCMD_RTT_RANGE_START,
	RTT_SUBCMD_CANCEL_CONFIG,
	RTT_SUBCMD_GETCAPABILITY,
};

enum LSTATS_SUB_COMMAND {
	LSTATS_SUBCMD_GET_INFO = ANDROID_NL80211_SUBCMD_LSTATS_RANGE_START,
};

/* moved from wifi_logger.cpp */
enum DEBUG_SUB_COMMAND {
	LOGGER_START_LOGGING = ANDROID_NL80211_SUBCMD_DEBUG_RANGE_START,
	LOGGER_GET_VER,
	LOGGER_DRIVER_MEM_DUMP,
};

enum WIFI_OFFLOAD_SUB_COMMAND {
	WIFI_OFFLOAD_START_MKEEP_ALIVE =
		ANDROID_NL80211_SUBCMD_WIFI_OFFLOAD_RANGE_START,
	WIFI_OFFLOAD_STOP_MKEEP_ALIVE,
};

enum MTK_WIFI_VENDOR_SUB_COMMAND {
	WIFI_SUBCMD_TRIGGER_RESET = 1,
};

enum WIFI_VENDOR_EVENT {
	GSCAN_EVENT_SIGNIFICANT_CHANGE_RESULTS,
	GSCAN_EVENT_HOTLIST_RESULTS_FOUND,
	GSCAN_EVENT_SCAN_RESULTS_AVAILABLE,
	GSCAN_EVENT_FULL_SCAN_RESULTS,
	RTT_EVENT_COMPLETE,
	GSCAN_EVENT_COMPLETE_SCAN,
	GSCAN_EVENT_HOTLIST_RESULTS_LOST,
	WIFI_EVENT_RSSI_MONITOR,
	WIFI_EVENT_DRIVER_ERROR,
	WIFI_EVENT_ACS,
	WIFI_EVENT_GENERIC_RESPONSE,
	WIFI_EVENT_BIGDATA_PIP,
	WIFI_EVENT_OP_MODE_CHANGE,
	WIFI_EVENT_DFS_OFFLOAD_CAC_STARTED,
	WIFI_EVENT_DFS_OFFLOAD_CAC_FINISHED,
	WIFI_EVENT_DFS_OFFLOAD_CAC_ABORTED,
	WIFI_EVENT_DFS_OFFLOAD_CAC_NOP_FINISHED,
	WIFI_EVENT_DFS_OFFLOAD_RADAR_DETECTED,
	WIFI_EVENT_RESET_TRIGGERED,
	WIFI_EVENT_SUBCMD_NAN,
	WIFI_EVENT_SUBCMD_NDP,
	/* Always add at the end.*/
};

enum WIFI_ATTRIBUTE {
	WIFI_ATTRIBUTE_BAND = 1,
	WIFI_ATTRIBUTE_NUM_CHANNELS,
	WIFI_ATTRIBUTE_CHANNEL_LIST,

	WIFI_ATTRIBUTE_NUM_FEATURE_SET,
	WIFI_ATTRIBUTE_FEATURE_SET,
	WIFI_ATTRIBUTE_PNO_RANDOM_MAC_OUI,
	WIFI_ATTRIBUTE_NODFS_VALUE,
	WIFI_ATTRIBUTE_COUNTRY_CODE,

	WIFI_ATTRIBUTE_ROAMING_CAPABILITIES,
	WIFI_ATTRIBUTE_ROAMING_BLACKLIST_NUM,
	WIFI_ATTRIBUTE_ROAMING_BLACKLIST_BSSID,
	WIFI_ATTRIBUTE_ROAMING_WHITELIST_NUM,
	WIFI_ATTRIBUTE_ROAMING_WHITELIST_SSID,
	WIFI_ATTRIBUTE_ROAMING_STATE,
	WIFI_ATTRIBUTE_TX_POWER_SCENARIO,
	WIFI_ATTRIBUTE_MAX,
};

enum WIFI_STATS_ATTRIBUTE {
	WIFI_ATTRIBUTE_STATS_TX  = 0,
	WIFI_ATTRIBUTE_STATS_RX,
	WIFI_ATTRIBUTE_STATS_CGS,
	WIFI_ATTRIBUTE_STATS_MAX,
};

enum WIFI_RSSI_MONITOR_ATTRIBUTE {
	WIFI_ATTRIBUTE_RSSI_MONITOR_INVALID	  = 0,
	WIFI_ATTRIBUTE_RSSI_MONITOR_MAX_RSSI      = 1,
	WIFI_ATTRIBUTE_RSSI_MONITOR_MIN_RSSI      = 2,
	WIFI_ATTRIBUTE_RSSI_MONITOR_START	  = 3,
	WIFI_ATTRIBUTE_RSSI_MONITOR_ATTRIBUTE_MAX
};

enum MULTISTA_ATTRIBUTE {
	MULTISTA_ATTRIBUTE_INVALID,
	MULTISTA_ATTRIBUTE_PRIMARY_IFACE,
	MULTISTA_ATTRIBUTE_USE_CASE,
	/* Add more attributes here */
	MULTISTA_ATTRIBUTE_MAX
};

enum LOGGER_ATTRIBUTE {
	LOGGER_ATTRIBUTE_INVALID    = 0,
	LOGGER_ATTRIBUTE_DRIVER_VER = 1,
	LOGGER_ATTRIBUTE_FW_VER     = 2,
	LOGGER_ATTRIBUTE_MAX	    = 3
};

enum RTT_ATTRIBUTE {
	RTT_ATTRIBUTE_CAPABILITIES = 1,

	RTT_ATTRIBUTE_TARGET_CNT = 10,
	RTT_ATTRIBUTE_TARGET_INFO,
	RTT_ATTRIBUTE_TARGET_MAC,
	RTT_ATTRIBUTE_TARGET_TYPE,
	RTT_ATTRIBUTE_TARGET_PEER,
	RTT_ATTRIBUTE_TARGET_CHAN,
	RTT_ATTRIBUTE_TARGET_PERIOD,
	RTT_ATTRIBUTE_TARGET_NUM_BURST,
	RTT_ATTRIBUTE_TARGET_NUM_FTM_BURST,
	RTT_ATTRIBUTE_TARGET_NUM_RETRY_FTM,
	RTT_ATTRIBUTE_TARGET_NUM_RETRY_FTMR,
	RTT_ATTRIBUTE_TARGET_LCI,
	RTT_ATTRIBUTE_TARGET_LCR,
	RTT_ATTRIBUTE_TARGET_BURST_DURATION,
	RTT_ATTRIBUTE_TARGET_PREAMBLE,
	RTT_ATTRIBUTE_TARGET_BW,
	RTT_ATTRIBUTE_RESULTS_COMPLETE = 30,
	RTT_ATTRIBUTE_RESULTS_PER_TARGET,
	RTT_ATTRIBUTE_RESULT_CNT,
	RTT_ATTRIBUTE_RESULT
};

enum LSTATS_ATTRIBUTE {
	LSTATS_ATTRIBUTE_STATS = 2,
};

enum WIFI_MKEEP_ALIVE_ATTRIBUTE {
	MKEEP_ALIVE_ATTRIBUTE_INVALID = 0,
	MKEEP_ALIVE_ATTRIBUTE_ID,
	MKEEP_ALIVE_ATTRIBUTE_IP_PKT_LEN,
	MKEEP_ALIVE_ATTRIBUTE_IP_PKT,
	MKEEP_ALIVE_ATTRIBUTE_SRC_MAC_ADDR,
	MKEEP_ALIVE_ATTRIBUTE_DST_MAC_ADDR,
	MKEEP_ALIVE_ATTRIBUTE_PERIOD_MSEC,
	MKEEP_ALIVE_ATTRIBUTE_MAX
};

#if (CFG_SUPPORT_APF == 1)
#define APF_VERSION		4
#define APF_MAX_PROGRAM_LEN	2048
#else
#define APF_VERSION		0
#define APF_MAX_PROGRAM_LEN	0
#endif

enum WIFI_APF_ATTRIBUTE {
	APF_ATTRIBUTE_INVALID = 0,
	APF_ATTRIBUTE_VERSION,
	APF_ATTRIBUTE_MAX_LEN,
	APF_ATTRIBUTE_PROGRAM,
	APF_ATTRIBUTE_PROGRAM_LEN,
	APF_ATTRIBUTE_MAX
};

/* QCA Vender CMD */
enum QCA_SET_BAND {
	QCA_SETBAND_AUTO,
	QCA_SETBAND_5G,
	QCA_SETBAND_2G,
	QCA_SETBAND_6G,
};

enum QCA_ATTR_ROAM_SUBCMD {
	QCA_ATTR_ROAM_SUBCMD_INVALID = 0,
	QCA_ATTR_ROAM_SUBCMD_SET_BLACKLIST_BSSID = 6,

	/* keep last */
	QCA_ATTR_ROAM_SUBCMD_AFTER_LAST,
	QCA_ATTR_ROAM_SUBCMD_MAX =
	QCA_ATTR_ROAM_SUBCMD_AFTER_LAST - 1,
};

enum QCA_ATTR_ROAMING_PARAMS {
	QCA_ATTR_ROAMING_PARAM_INVALID = 0,

	QCA_ATTR_ROAMING_SUBCMD = 1,
	QCA_ATTR_ROAMING_REQ_ID = 2,

	/* Attribute for set_blacklist bssid params */
	QCA_ATTR_ROAMING_PARAM_SET_BSSID_PARAMS = 18,
	QCA_ATTR_ROAMING_PARAM_SET_BSSID_PARAMS_NUM_BSSID = 19,
	QCA_ATTR_ROAMING_PARAM_SET_BSSID_PARAMS_BSSID = 20,

	/* keep last */
	QCA_ATTR_ROAMING_PARAM_AFTER_LAST,
	QCA_ATTR_ROAMING_PARAM_MAX =
	QCA_ATTR_ROAMING_PARAM_AFTER_LAST - 1,
};

enum QCA_ATTR_DFS_PARAMS {
	/* used by NL80211_VENDOR_SUBCMD_DFS_CAPABILITY */
	QCA_ATTR_DFS_CAPAB = 1,
};

enum WIFI_VENDOR_ATTR_PREFERRED_FREQ_LIST {
	WIFI_VENDOR_ATTR_PREFERRED_FREQ_LIST_INVALID,
	WIFI_VENDOR_ATTR_PREFERRED_FREQ_LIST_IFACE_TYPE,
	WIFI_VENDOR_ATTR_PREFERRED_FREQ_LIST_GET,
	WIFI_VENDOR_ATTR_PREFERRED_FREQ_LIST_LAST,
	WIFI_VENDOR_ATTR_PREFERRED_FREQ_LIST_MAX =
		WIFI_VENDOR_ATTR_PREFERRED_FREQ_LIST_LAST - 1
};

enum WIFI_VENDOR_ATTR_ACS {
	WIFI_VENDOR_ATTR_ACS_CHANNEL_INVALID = 0,
	WIFI_VENDOR_ATTR_ACS_PRIMARY_CHANNEL = 1,
	WIFI_VENDOR_ATTR_ACS_SECONDARY_CHANNEL = 2,
	WIFI_VENDOR_ATTR_ACS_HW_MODE = 3,
	WIFI_VENDOR_ATTR_ACS_HT_ENABLED = 4,
	WIFI_VENDOR_ATTR_ACS_HT40_ENABLED = 5,
	WIFI_VENDOR_ATTR_ACS_VHT_ENABLED = 6,
	WIFI_VENDOR_ATTR_ACS_CHWIDTH = 7,
	WIFI_VENDOR_ATTR_ACS_CH_LIST = 8,
	WIFI_VENDOR_ATTR_ACS_VHT_SEG0_CENTER_CHANNEL = 9,
	WIFI_VENDOR_ATTR_ACS_VHT_SEG1_CENTER_CHANNEL = 10,
	WIFI_VENDOR_ATTR_ACS_FREQ_LIST = 11,
	WIFI_VENDOR_ATTR_ACS_PRIMARY_FREQUENCY = 12,
	WIFI_VENDOR_ATTR_ACS_SECONDARY_FREQUENCY = 13,
	WIFI_VENDOR_ATTR_ACS_VHT_SEG0_CENTER_FREQUENCY = 14,
	WIFI_VENDOR_ATTR_ACS_VHT_SEG1_CENTER_FREQUENCY = 15,
	WIFI_VENDOR_ATTR_ACS_ACS_EDMG_ENABLED = 16,
	WIFI_VENDOR_ATTR_ACS_ACS_EDMG_CHANNEL = 17,

	/* keep last */
	WIFI_VENDOR_ATTR_ACS_AFTER_LAST,
	WIFI_VENDOR_ATTR_ACS_MAX =
		WIFI_VENDOR_ATTR_ACS_AFTER_LAST - 1
};
#if CFG_SUPPORT_NAN
enum WIFI_VENDOR_NAN {
	MTK_WLAN_VENDOR_ATTR_NAN = 2,
};
#endif

#define MAX_FW_ROAMING_BLACKLIST_SIZE	16
#define MAX_FW_ROAMING_WHITELIST_SIZE	8

#if CFG_SUPPORT_DATA_STALL
enum WIFI_DATA_STALL_ATTRIBUTE {
	WIFI_ATTRIBUTE_ERROR_REASON = 0,
};
#endif

#if CFG_SUPPORT_BIGDATA_PIP
enum WIFI_BIGDATA_PIP_ATTRIBUTE {
	WIFI_ATTRIBUTE_PIP_PAYLOAD = 0,
};
#endif

enum WIFI_SCAN_PARAMS_ATTRIBUTE {
	WIFI_ATTR_SCAN_IFACE_TYPE = 0,
	WIFI_ATTR_SCAN_ASSOC_TYPE,
	WIFI_ATTR_SCAN_TYPE,
	WIFI_ATTR_SCAN_PROBE_NUM,
	WIFI_ATTR_SCAN_ACTIVE_TIME,
	WIFI_ATTR_SCAN_PASSIVE_TIME,
	WIFI_ATTR_SCAN_HOME_TIME,
	WIFI_ATTR_SCAN_ACTIVE_N_CH_BACK,
	WIFI_ATTR_SCAN_PASSIVE_N_CH_BACK,
	WIFI_ATTR_SCAN_MAX
};

#if CFG_SUPPORT_DBDC
enum WIFI_OP_MODE_CHANGE_ATTRIBUTE {
	WIFI_ATTRIBUTE_OP_MODE_CHANGE = 0,
};
#endif

enum WIFI_RESET_TRIGGERED_ATTRIBUTE {
	WIFI_ATTRIBUTE_RESET_REASON = 1,
};

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */
#if CFG_SUPPORT_WAPI
extern uint8_t
keyStructBuf[1024];	/* add/remove key shared buffer */
#else
extern uint8_t
keyStructBuf[100];	/* add/remove key shared buffer */
#endif

extern const struct nla_policy mtk_scan_param_policy[
		WIFI_ATTR_SCAN_MAX + 1];
extern const struct nla_policy nla_parse_wifi_multista[
		MULTISTA_ATTRIBUTE_MAX + 1];
extern const struct nla_policy nla_parse_wifi_rssi_monitor[
		WIFI_ATTRIBUTE_RSSI_MONITOR_ATTRIBUTE_MAX + 1];
extern const struct nla_policy nla_parse_wifi_attribute[
		WIFI_ATTRIBUTE_MAX + 1];
extern const struct nla_policy nal_parse_wifi_setband[
		QCA_WLAN_VENDOR_ATTR_MAX + 1];
extern const struct nla_policy nla_get_version_policy[
		LOGGER_ATTRIBUTE_MAX + 1];
extern const struct nla_policy nla_parse_offloading_policy[
		MKEEP_ALIVE_ATTRIBUTE_PERIOD_MSEC + 1];
extern const struct nla_policy nla_get_preferred_freq_list_policy[
		WIFI_VENDOR_ATTR_PREFERRED_FREQ_LIST_MAX + 1];
extern const struct nla_policy nla_get_acs_policy[
		WIFI_VENDOR_ATTR_ACS_MAX + 1];

#if CFG_SUPPORT_MBO
extern const struct nla_policy qca_roaming_param_policy[
	QCA_ATTR_ROAMING_PARAM_MAX + 1];
#endif

extern const struct nla_policy nla_get_apf_policy[
		APF_ATTRIBUTE_MAX + 1];

/*******************************************************************************
 *                           MACROS
 *******************************************************************************
 */

#if KERNEL_VERSION(3, 5, 0) <= LINUX_VERSION_CODE
/*
 * #define NLA_PUT(skb, attrtype, attrlen, data) \
 *	do { \
 *		if (unlikely(nla_put(skb, attrtype, attrlen, data) < 0)) \
 *			goto nla_put_failure; \
 *	} while (0)
 *
 *#define NLA_PUT_TYPE(skb, type, attrtype, value) \
 *	do { \
 *		type __tmp = value; \
 *		NLA_PUT(skb, attrtype, sizeof(type), &__tmp); \
 *	} while (0)
 */
#define NLA_PUT(skb, attrtype, attrlen, data) \
	mtk_cfg80211_NLA_PUT(skb, attrtype, attrlen, data)

#define NLA_PUT_TYPE(skb, type, attrtype, value) \
	mtk_cfg80211_nla_put_type(skb, type, attrtype, value)

#define NLA_PUT_U8(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, NLA_PUT_DATE_U8, attrtype, value)

#define NLA_PUT_U16(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, NLA_PUT_DATE_U16, attrtype, value)

#define NLA_PUT_U32(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, NLA_PUT_DATE_U32, attrtype, value)

#define NLA_PUT_U64(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, NLA_PUT_DATE_U64, attrtype, value)

#endif

#if KERNEL_VERSION(4, 12, 0) <= CFG80211_VERSION_CODE
#define NLA_PARSE_NESTED(nlattr, maxtype, nla, policy)	\
	nla_parse_nested(nlattr, maxtype, nla, policy, NULL)
#define NLA_PARSE(tb, maxtype, head, len, policy) \
	nla_parse(tb, maxtype, head, len, policy, NULL)
#else
#define NLA_PARSE_NESTED(nlattr, maxtype, nla, policy)	\
	nla_parse_nested(nlattr, maxtype, nla, policy)
#define NLA_PARSE(tb, maxtype, head, len, policy) \
	nla_parse(tb, maxtype, head, len, policy)
#endif

/*******************************************************************************
 *				P R I V A T E   D A T A
 *
 *******************************************************************************
 */
struct PARAM_WIFI_CHANGE_RESULT {
	uint16_t flags;
	uint16_t channel;
	uint8_t bssid[6];	/* BSSID */
	int8_t rssi[8];	/* RSSI history in db */
};

struct PARAM_AP_THRESHOLD {
	uint8_t bssid[6];	/* AP BSSID */
	int32_t low;	/* low threshold */
	int32_t high;	/* high threshold */
	uint32_t channel;	/* channel hint */
};

#define STATS_LLS_MAX_NSS_NUM    2

#define STATS_LLS_CCK_NUM        4   /* 1M/2M/5.5M/11M */
#define STATS_LLS_OFDM_NUM       8   /* 6M/9M/12M/18M/24M/36M/48M/54M */
#define STATS_LLS_HT_NUM         16  /* MCS0~15 */
#define STATS_LLS_VHT_NUM        10  /* MCS0~9 */
#define STATS_LLS_HE_NUM         12  /* MCS0~11 */

#define STATS_LLS_MAX_CCK_BW_NUM  1  /* BW20 */
#define STATS_LLS_MAX_OFDM_BW_NUM 1  /* BW20 */
#define STATS_LLS_MAX_HT_BW_NUM   2  /* BW20/40 */
#define STATS_LLS_MAX_VHT_BW_NUM  3  /* BW20/40/80 */
#define STATS_LLS_MAX_HE_BW_NUM   4  /* BW20/40/80/160 */

#define STATS_LLS_MAX_CCK_NUM  (STATS_LLS_CCK_NUM)
#define STATS_LLS_MAX_OFDM_NUM (STATS_LLS_OFDM_NUM)
#define STATS_LLS_MAX_HT_NUM   \
	(STATS_LLS_HT_NUM * STATS_LLS_MAX_HT_BW_NUM)
#define STATS_LLS_MAX_VHT_NUM  \
	(STATS_LLS_VHT_NUM * STATS_LLS_MAX_VHT_BW_NUM * STATS_LLS_MAX_NSS_NUM)
#define STATS_LLS_MAX_HE_NUM   \
	(STATS_LLS_HE_NUM * STATS_LLS_MAX_HE_BW_NUM * STATS_LLS_MAX_NSS_NUM)


#define STATS_LLS_CH_NUM_2G4 14
#define STATS_LLS_CH_NUM_5G 32
#if CFG_SUPPORT_WIFI_6G
#define STATS_LLS_CH_NUM_6G 64
#else
#define STATS_LLS_CH_NUM_6G 0
#endif

#define STATS_LLS_CH_NUM (STATS_LLS_CH_NUM_2G4 +	\
			  STATS_LLS_CH_NUM_5G +		\
			  STATS_LLS_CH_NUM_6G)

#define STATS_LLS_RATE_NUM (STATS_LLS_MAX_CCK_NUM +	\
			    STATS_LLS_MAX_OFDM_NUM +	\
			    STATS_LLS_MAX_HT_NUM +	\
			    STATS_LLS_MAX_VHT_NUM +	\
			    STATS_LLS_MAX_HE_NUM)

/**
 * channel operating width
 */
enum ENUM_STATS_LLS_CHANNEL_WIDTH {
	STATS_LLS_WIFI_CHAN_WIDTH_20    = 0,
	STATS_LLS_WIFI_CHAN_WIDTH_40    = 1,
	STATS_LLS_WIFI_CHAN_WIDTH_80    = 2,
	STATS_LLS_WIFI_CHAN_WIDTH_160   = 3,
	STATS_LLS_WIFI_CHAN_WIDTH_80P80 = 4,
	STATS_LLS_WIFI_CHAN_WIDTH_5     = 5,
	STATS_LLS_WIFI_CHAN_WIDTH_10    = 6,
	STATS_LLS_WIFI_CHAN_WIDTH_INVALID = -1
};

/**
 * @width: channel width (20, 40, 80, 80+80, 160)
 * @center_freq: primary 20 MHz channel
 * @center_freq0: center frequency (MHz) first segment
 * @center_freq1: center frequency (MHz) second segment
 */
struct STATS_LLS_CHANNEL_INFO {
	enum ENUM_STATS_LLS_CHANNEL_WIDTH width;
	int32_t center_freq;
	int32_t center_freq0;
	int32_t center_freq1;
};

/**
 * @channel: channel
 * @on_time: msecs the radio is awake
 *           (32 bits number accruing over time)
 * @cca_busy_time: msecs the CCA register is busy
 *                 (32 bits number accruing over time)
 */
struct STATS_LLS_CHANNEL_STAT {
	struct STATS_LLS_CHANNEL_INFO channel;
	uint32_t on_time;
	uint32_t cca_busy_time;
};

/**
 * Statistics data reported for STATS_LLS_TAG_RADIO_STAT.
 *
 * @radio: wifi radio (if multiple radio supported)
 * @on_time: msecs the radio is awake (32 bits number accruing over time)
 * @tx_time: msecs the radio is transmitting (32 bits number accruing over time)
 * @num_tx_levels: number of radio transmit power levels (unavailable)
 * @tx_time_per_levels: pointer to an array of radio transmit per power levels
 *                      in msecs accured over time (unavailable)
 * @rx_time: msecs the radio is in active receive
 *           (32 bits number accruing over time)
 * @on_time_scan: msecs the radio is awake due to all scan
 *                (32 bits number accruing over time)
 * @on_time_nbd: msecs the radio is awake due to NAN
 *               (32 bits number accruing over time)
 * @on_time_gscan: msecs the radio is awake due to G?scan
 *                 (32 bits number accruing over time)
 * @on_time_roam_scan: msecs the radio is awake due to roam?scan
 *                     (32 bits number accruing over time)
 * @on_time_pno_scan: msecs the radio is awake due to PNO scan
 *                    (32 bits number accruing over time)
 * @on_time_hs20: msecs the radio is awake due to HS2.0 scans and GAS exchange
 *                (32 bits number accruing over time)
 * @num_channels: number of channels
 * @RESERVED: for padding since sizeof this structure returns 56 bytes.
 * @channels: channel statistics
 */
struct STATS_LLS_WIFI_RADIO_STAT {
	int32_t radio;
	uint32_t on_time;
	uint32_t tx_time;
	uint32_t num_tx_levels; /* 0 */
	uint32_t *tx_time_per_levels; /* NULL */
	uint32_t rx_time;
	uint32_t on_time_scan;
	uint32_t on_time_nbd;
	uint32_t on_time_gscan;
	uint32_t on_time_roam_scan;
	uint32_t on_time_pno_scan;
	uint32_t on_time_hs20;
	uint32_t num_channels;
	struct STATS_LLS_CHANNEL_STAT channels[0];
};


enum STATS_LLS_WIFI_RATE_PREAMBLE {
	LLS_MODE_OFDM,
	LLS_MODE_CCK,
	LLS_MODE_HT,
	LLS_MODE_VHT,
	LLS_MODE_HE,
	LLS_MODE_RESERVED,
};

/**
 * wifi rate
 *
 * @preamble: 0: OFDM, 1:CCK, 2:HT 3:VHT 4:HE 5..7 reserved
 * @nss: 0:1x1, 1:2x2, 3:3x3, 4:4x4
 * @bw: 0:20MHz, 1:40Mhz, 2:80Mhz, 3:160Mhz
 * @rateMcsIdx: OFDM/CCK rate code would be as per ieee std in the units of
 *              0.5mbps
 *              HT/VHT/HE it would be mcs index
 * @reserved: reserved
 * @bitrate: units of 100 Kbps
 */
struct STATS_LLS_WIFI_RATE {
	uint32_t preamble   :3;
	uint32_t nss        :2;
	uint32_t bw         :3;
	uint32_t rateMcsIdx :8;

	uint32_t reserved   :16;
	uint32_t bitrate;
};

/**
 * per rate statistics
 *
 * @rate: rate information
 * @tx_mpdu: number of successfully transmitted data pkts (ACK rcvd)
 * @rx_mpdu: number of received data pkts
 * @mpdu_lost: number of data packet losses (no ACK)
 * @retries: total number of data pkt retries
 * @retries_short: number of short data pkt retries
 * @retries_long: number of long data pkt retries
 */
struct STATS_LLS_RATE_STAT {
	struct STATS_LLS_WIFI_RATE rate;
	uint32_t tx_mpdu;
	uint32_t rx_mpdu;
	uint32_t mpdu_lost;
	uint32_t retries;
	uint32_t retries_short;
	uint32_t retries_long;
};

enum WIFI_MULTI_STA_USE_CASE {
	WIFI_DUAL_STA_TRANSIENT_PREFER_PRIMARY = 0,
	WIFI_DUAL_STA_NON_TRANSIENT_UNBIASED = 1,
	WIFI_DUAL_STA_MTK_LEGACY = 15,
};

enum ENUM_WIFI_CONNECTION_STATE {
	WIFI_DISCONNECTED = 0,
	WIFI_AUTHENTICATING = 1,
	WIFI_ASSOCIATING = 2,
	WIFI_ASSOCIATED = 3,
	WIFI_EAPOL_STARTED = 4,   /* if done by firmware/driver */
	WIFI_EAPOL_COMPLETED = 5, /* if done by firmware/driver */
};

enum ENUM_WIFI_ROAM_STATE {
	WIFI_ROAMING_IDLE = 0,
	WIFI_ROAMING_ACTIVE = 1,
};

enum ENUM_WIFI_INTERFACE_MODE {
	WIFI_INTERFACE_STA = 0,
	WIFI_INTERFACE_SOFTAP = 1,
	WIFI_INTERFACE_IBSS = 2,
	WIFI_INTERFACE_P2P_CLIENT = 3,
	WIFI_INTERFACE_P2P_GO = 4,
	WIFI_INTERFACE_NAN = 5,
	WIFI_INTERFACE_MESH = 6,
	WIFI_INTERFACE_TDLS = 7,
	WIFI_INTERFACE_UNKNOWN = -1
};

/**
 *
 * @mode: interface mode
 * @mac_addr[6]: interface mac address (self)
 * @state: connection state (valid for STA, CLI only)
 * @roaming: roaming state
 * @capabilities: WIFI_CAPABILITY_XXX (self)
 * @ssid[33]: null terminated SSID
 * @bssid[6]: bssid
 * @ap_country_str[3]: country string advertised by AP
 * @country_str[3]: country string for this association
 * @time_slicing_duty_cycle_percent: if this iface is being served using time
 *                     slicing on a radio with one or more ifaces (i.e MCC),
 *                     then the duty cycle assigned to this iface in %.
 *                     If not using time slicing (i.e SCC or DBS), set to 100.
 */
struct WIFI_INTERFACE_LINK_LAYER_INFO {
	enum ENUM_WIFI_INTERFACE_MODE mode;
	uint8_t mac_addr[6];
	enum ENUM_WIFI_CONNECTION_STATE state;
	enum ENUM_WIFI_ROAM_STATE roaming;
	uint32_t capabilities;
	uint8_t ssid[33];
	uint8_t bssid[6];
	uint8_t ap_country_str[3];
	uint8_t country_str[3];
	uint8_t time_slicing_duty_cycle_percent;
};

/**
 * access categories
 */
enum ENUM_STATS_LLS_AC {
	STATS_LLS_WIFI_AC_VO  = 0,
	STATS_LLS_WIFI_AC_VI  = 1,
	STATS_LLS_WIFI_AC_BE  = 2,
	STATS_LLS_WIFI_AC_BK  = 3,
	STATS_LLS_WIFI_AC_MAX = 4,
};

/* access categories */
enum WIFI_TRAFFIC_AC {
	WIFI_AC_VO = 0,
	WIFI_AC_VI = 1,
	WIFI_AC_BE = 2,
	WIFI_AC_BK = 3,
	WIFI_AC_MAX = 4,
};

/* wifi peer type */
enum ENUM_STATS_PEER_TYPE {
	STATS_LLS_WIFI_PEER_STA,
	STATS_LLS_WIFI_PEER_AP,
	STATS_LLS_WIFI_PEER_P2P_GO,
	STATS_LLS_WIFI_PEER_P2P_CLIENT,
	STATS_LLS_WIFI_PEER_NAN,
	STATS_LLS_WIFI_PEER_TDLS,
	STATS_LLS_WIFI_PEER_INVALID,
};

/**
 * per peer statistics
 *
 * @sta_count: station count
 * @chan_util: channel utilization
 * @reserved: reserved
 */
struct STATS_LLS_BSSLOAD_INFO {
	uint16_t sta_count;
	uint16_t chan_util;
	uint8_t reserved[4];
};

/**
 * Statistics data reported for STATS_LLS_TAG_PEER_STAT.
 *
 * @type: peer type (AP, TDLS, GO etc.)
 * @peer_mac_address[6]: mac address (unavailable)
 * @capabilities: peer WIFI_CAPABILITY_XXX (unavailable)
 * @bssload: STA count and CU
 * @num_rate: number of rates
 * @rate_stats: per rate statistics, number of entries  = num_rate
 */
struct STATS_LLS_PEER_INFO {
	enum ENUM_STATS_PEER_TYPE type;
	uint8_t peer_mac_address[6];
	uint32_t capabilities;
	struct STATS_LLS_BSSLOAD_INFO bssload;
	uint32_t num_rate;
	struct STATS_LLS_RATE_STAT rate_stats[]; /* structure revised */
};

/**
 * Statistics data reported for STATS_LLS_TAG_WMM_AC_STAT.
 *
 * @ac: access category (VI, VO, BE, BK)
 * @tx_mpdu: number of successfully transmitted unicast data pkts (ACK rcvd)
 * @rx_mpdu: number of received unicast data packets
 * @tx_mcast: number of successfully transmitted multicast data packets
 *            STA case: implies ACK received from AP for the unicast packet
 *            in which mcast pkt was sent
 * @rx_mcast: number of received multicast data packets
 * @rx_ampdu: number of received unicast a-mpdus;
 *              support of this counter is optional
 * @tx_ampdu: number of transmitted unicast a-mpdus;
 *              support of this counter is optional
 * @mpdu_lost: number of data pkt losses (no ACK)
 * @retries: total number of data pkt retries
 * @retries_short: number of short data pkt retries
 * @retries_long: number of long data pkt retries
 * @contention_time_min: data pkt min contention time (usecs)
 * @contention_time_max: data pkt max contention time (usecs)
 * @contention_time_avg: data pkt avg contention time (usecs)
 * @contention_num_samples: num of data pkts used for contention statistics
 */
struct STATS_LLS_WMM_AC_STAT {
	enum ENUM_STATS_LLS_AC ac;
	uint32_t tx_mpdu;
	uint32_t rx_mpdu;
	uint32_t tx_mcast;
	uint32_t rx_mcast;
	uint32_t rx_ampdu;
	uint32_t tx_ampdu;
	uint32_t mpdu_lost;
	uint32_t retries;
	uint32_t retries_short;
	uint32_t retries_long;
	uint32_t contention_time_min;
	uint32_t contention_time_max;
	uint32_t contention_time_avg;
	uint32_t contention_num_samples;
};

/**
 * interface statistics
 *
 * @iface: wifi interface
 * @info: current state of the interface
 * @beacon_rx: access point beacon received count from connected AP
 * @average_tsf_offset: average beacon offset encountered (beacon_TSF - TBTT)
 *               The average_tsf_offset field is used so as to calculate the
 *               typical beacon contention time on the channel as well may be
 *               used to debug beacon synchronization and related power
 *               consumption issue
 * @leaky_ap_detected: indicate that this AP typically leaks packets
 *                     beyond the driver guard time.
 * @leaky_ap_avg_num_frames_leaked: average number of frame leaked by AP
 *                                  after frame with PM bit set was ACK'ed by AP
 * @leaky_ap_guard_time: guard time currently in force
 *                       (when implementing IEEE power management based on
 *                       frame control PM bit), How long driver waits before
 *                       shutting down the radio and after receiving an ACK
 *                       for a data frame with PM bit set)
 * @mgmt_rx: access point mgmt frames received count from connected AP
 *           (including Beacon)
 * @mgmt_action_rx: action frames received count
 * @mgmt_action_tx: action frames transmit count
 * @rssi_mgmt: access Point Beacon and Management frames RSSI (averaged)
 * @rssi_data: access Point Data Frames RSSI (averaged) from connected AP
 * @rssi_ack: access Point ACK RSSI (averaged) from connected AP
 * @ac[WIFI_AC_MAX]: per ac data packet statistics
 * @num_peers: number of peers
 * @peer_info[]: per peer statistics
 */
struct STATS_LLS_WIFI_IFACE_STAT {
	void *iface;
	struct WIFI_INTERFACE_LINK_LAYER_INFO info;
	uint32_t beacon_rx;
	uint64_t average_tsf_offset;
	uint32_t leaky_ap_detected;
	uint32_t leaky_ap_avg_num_frames_leaked;
	uint32_t leaky_ap_guard_time;
	uint32_t mgmt_rx;
	uint32_t mgmt_action_rx;
	uint32_t mgmt_action_tx;
	int32_t rssi_mgmt;
	int32_t rssi_data;
	int32_t rssi_ack;
	struct STATS_LLS_WMM_AC_STAT ac[STATS_LLS_WIFI_AC_MAX];
	uint32_t num_peers;
	struct STATS_LLS_PEER_INFO peer_info[];
};

/* Shared EMI Memory layout */
struct PEER_INFO_RATE_STAT {
	struct STATS_LLS_PEER_INFO peer;
	struct STATS_LLS_RATE_STAT rate[STATS_LLS_RATE_NUM];
};

struct WIFI_RADIO_CHANNEL_STAT {
	struct STATS_LLS_WIFI_RADIO_STAT radio;
	struct STATS_LLS_CHANNEL_STAT channel[STATS_LLS_CH_NUM];
};

struct HAL_LLS_FULL_REPORT {
	struct STATS_LLS_WIFI_IFACE_STAT iface;
	struct PEER_INFO_RATE_STAT peer_info[CFG_STA_REC_NUM];
	struct WIFI_RADIO_CHANNEL_STAT radio[ENUM_BAND_NUM];
};

struct STATS_LLS_PEER_AP_REC {
	uint16_t sta_count;
	uint16_t chan_util;
	uint8_t mac_addr[ETH_ALEN];
};

/* RTT Capabilities */
struct PARAM_WIFI_RTT_CAPABILITIES {
	/* if 1-sided rtt data collection is supported */
	uint8_t rtt_one_sided_supported;
	/* if ftm rtt data collection is supported */
	uint8_t rtt_ftm_supported;
	/* if initiator supports LCI request. Applies to 2-sided RTT */
	uint8_t lci_support;
	/* if initiator supports LCR request. Applies to 2-sided RTT */
	uint8_t lcr_support;
	/* bit mask indicates what preamble is supported by initiator */
	uint8_t preamble_support;
	/* bit mask indicates what BW is supported by initiator */
	uint8_t bw_support;
};

enum ENUM_NLA_PUT_DATE_TYPE {
	NLA_PUT_DATE_U8 = 0,
	NLA_PUT_DATE_U16,
	NLA_PUT_DATE_U32,
	NLA_PUT_DATE_U64,
};

/* RSSI Monitoring */
struct PARAM_RSSI_MONITOR_T {
	bool enable;	/* 1=Start, 0=Stop*/
	int8_t max_rssi_value;
	int8_t min_rssi_value;
	uint8_t reserved[1];
	uint8_t reserved2[4]; /* reserved for MT6632 */
};

struct PARAM_RSSI_MONITOR_EVENT {
	uint8_t version;
	int8_t rssi;
	uint8_t BSSID[PARAM_MAC_ADDR_LEN];
};

/* Packet Keep Alive */
struct PARAM_PACKET_KEEPALIVE_T {
	bool enable;	/* 1=Start, 0=Stop*/
	uint8_t index;
	int16_t u2IpPktLen;
	uint8_t pIpPkt[256];
	uint8_t ucSrcMacAddr[PARAM_MAC_ADDR_LEN];
	uint8_t ucDstMacAddr[PARAM_MAC_ADDR_LEN];
	uint32_t u4PeriodMsec;
	uint8_t reserved[8]; /* reserved for MT6632 */
};

struct PARAM_BSS_MAC_OUI {
	uint8_t ucBssIndex;
	uint8_t ucMacOui[MAC_OUI_LEN];
};

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */



/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */
int mtk_cfg80211_NLA_PUT(struct sk_buff *skb, int attrtype,
			 int attrlen, const void *data);

int mtk_cfg80211_nla_put_type(struct sk_buff *skb,
			      enum ENUM_NLA_PUT_DATE_TYPE type, int attrtype,
			      const void *value);

int mtk_cfg80211_vendor_get_capabilities(struct wiphy
		*wiphy, struct wireless_dev *wdev,
		const void *data, int data_len);

int mtk_cfg80211_vendor_set_config(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   const void *data, int data_len);

int mtk_cfg80211_vendor_set_scan_config(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int data_len);

int mtk_cfg80211_vendor_set_significant_change(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);

int mtk_cfg80211_vendor_set_hotlist(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    const void *data, int data_len);

int mtk_cfg80211_vendor_enable_scan(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    const void *data, int data_len);

int mtk_cfg80211_vendor_enable_full_scan_results(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);

int mtk_cfg80211_vendor_get_scan_results(struct wiphy
		*wiphy, struct wireless_dev *wdev,
		const void *data, int data_len);

int mtk_cfg80211_vendor_get_channel_list(struct wiphy
		*wiphy, struct wireless_dev *wdev,
		const void *data, int data_len);

int mtk_cfg80211_vendor_set_country_code(struct wiphy
		*wiphy, struct wireless_dev *wdev,
		const void *data, int data_len);

int mtk_cfg80211_vendor_get_rtt_capabilities(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);

int mtk_cfg80211_vendor_llstats_get_info(struct wiphy
		*wiphy, struct wireless_dev *wdev,
		const void *data, int data_len);

int mtk_cfg80211_vendor_set_band(struct wiphy *wiphy,
				 struct wireless_dev *wdev,
				 const void *data, int data_len);

int mtk_cfg80211_vendor_set_scan_mac_oui(struct wiphy *wiphy,
				 struct wireless_dev *wdev,
				 const void *data, int data_len);

#if CFG_SUPPORT_MBO
int mtk_cfg80211_vendor_set_roaming_param(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);
#endif

int mtk_cfg80211_vendor_set_roaming_policy(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);

int mtk_cfg80211_vendor_get_roaming_capabilities(
	struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_config_roaming(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data,
				       int data_len);

int mtk_cfg80211_vendor_enable_roaming(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data,
				       int data_len);

int mtk_cfg80211_vendor_set_rssi_monitoring(
	struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_packet_keep_alive_start(
	struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_packet_keep_alive_stop(
	struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_get_version(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    const void *data, int data_len);

int mtk_cfg80211_vendor_event_generic_response(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	uint32_t len, uint8_t *data);

int mtk_cfg80211_vendor_get_supported_feature_set(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);

int mtk_cfg80211_vendor_event_rssi_beyond_range(
	struct wiphy *wiphy,
	struct wireless_dev *wdev, int rssi);

int mtk_cfg80211_vendor_set_tx_power_scenario(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);

int mtk_cfg80211_vendor_set_multista_primary_connection(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);

int mtk_cfg80211_vendor_set_multista_use_case(
		struct wiphy *wiphy, struct wireless_dev *wdev,
		const void *data, int data_len);

int mtk_cfg80211_vendor_get_preferred_freq_list(struct wiphy
		*wiphy, struct wireless_dev *wdev, const void *data,
		int data_len);

int mtk_cfg80211_vendor_acs(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_dfs_capability(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_get_features(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_get_apf_capabilities(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int data_len);

#if (CFG_SUPPORT_APF == 1)
int mtk_cfg80211_vendor_set_packet_filter(
	struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_read_packet_filter(
	struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int data_len);
#endif /* CFG_SUPPORT_APF */

int mtk_cfg80211_vendor_driver_memory_dump(struct wiphy *wiphy,
					   struct wireless_dev *wdev,
					   const void *data,
					   int data_len);

int mtk_cfg80211_vendor_set_scan_param(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int data_len);

int mtk_cfg80211_vendor_get_trx_stats(struct wiphy *wiphy,
					   struct wireless_dev *wdev,
					   const void *data,
					   int data_len);

int mtk_cfg80211_vendor_trigger_reset(
	struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int data_len);

int mtk_cfg80211_vendor_event_reset_triggered(
	uint32_t data);
#endif /* _GL_VENDOR_H */
