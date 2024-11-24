/******************************************************************************
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
 *****************************************************************************/
/*
 * Id: //Department/DaVinci/BRANCHES/MT6620_WIFI_DRIVER_V2_3/mgmt/rlm_domain.c#2
 */

/*! \file   "rlm_domain.c"
 *    \brief
 *
 */


/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"
#include "rlm_txpwr_init.h"

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT
/* dynamic tx power control */
char *g_au1TxPwrChlTypeLabel[] = {
	"NORMAL",
	"ALL",
	"RANGE",
	"2G4",
	"5G",
	"BANDEDGE_2G4",
	"BANDEDGE_5G",
	"5GBAND1",
	"5GBAND2",
	"5GBAND3",
	"5GBAND4",
};

char *g_au1TxPwrAppliedWayLabel[] = {
	"wifi on",
	"ioctl"
};

char *g_au1TxPwrOperationLabel[] = {
	"power level",
	"power offset"
};

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
typedef int32_t (*PFN_TX_PWR_TAG_PARA_FUNC) (
	char *, char *, uint8_t, struct TX_PWR_CTRL_ELEMENT *);

struct TX_PWR_TAG_TABLE {
	const char arTagNames[32];
	uint8_t ucTagParaNum;
	PFN_TX_PWR_TAG_PARA_FUNC pfnParseTagParaHandler;
} g_auTxPwrTagTable[] = {
	{
		"MIMO_1T",
		(POWER_ANT_BAND_NUM * POWER_ANT_NUM),
		txPwrParseTagMimo1T
	},
	{
		"MIMO_2T",
		(POWER_ANT_BAND_NUM * POWER_ANT_NUM),
		txPwrParseTagMimo2T
	},
	{
		"ALL_T",
		(POWER_ANT_BAND_NUM * POWER_ANT_NUM),
		txPwrParseTagAllT
	},
	{
		"ALL_T_6G",
		(POWER_ANT_6G_BAND_NUM * POWER_ANT_NUM),
		txPwrParseTagAllT6G
	}
};
#endif

#endif

#define PWR_BUF_LEN 1024

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */
#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT
/* dynamic tx power control */
char *g_au1TxPwrDefaultSetting[] = {
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
	"_SAR_PwrLevel;1;2;1;[2G4,,,,,,,,,,,,][5G,,,,,,,,,,,,]",
	"_G_Scenario;1;2;1;[ALL,,,,,,,,,,,,]",
	"_G_Scenario;2;2;1;[ALL,,,,,,,,,,,,]",
	"_G_Scenario;3;2;1;[ALL,,,,,,,,,,,,]",
	"_G_Scenario;4;2;1;[ALL,,,,,,,,,,,,]",
	"_G_Scenario;5;2;1;[ALL,,,,,,,,,,,,]",
#else
	"_SAR_PwrLevel;1;2;1;[2G4,,,,,,,,,][5G,,,,,,,,,]",
	"_G_Scenario;1;2;1;[2G4,28,28,24,28,24,,,,][5G,18]",
	"_G_Scenario;2;2;1;[ALL,,,,,,,,,]",
	"_G_Scenario;3;2;1;[ALL,,,,,,,,,]",
	"_G_Scenario;4;2;1;[ALL,,,,,,,,,]",
	"_G_Scenario;5;2;1;[ALL,,,,,,,,,]",
#endif /* CFG_SUPPORT_DYNA_TX_PWR_CTRL_OFDM_SETTING */
};
#endif
/* The following country or domain shall be set from host driver.
 * And host driver should pass specified DOMAIN_INFO_ENTRY to MT6620 as
 * the channel list of being a STA to do scanning/searching AP or being an
 * AP to choose an adequate channel if auto-channel is set.
 */

/* Define mapping tables between country code and its channel set
 */
static const uint16_t g_u2CountryGroup0[] = { COUNTRY_CODE_JP };

static const uint16_t g_u2CountryGroup1[] = {
	COUNTRY_CODE_AI, COUNTRY_CODE_AN, COUNTRY_CODE_AZ, COUNTRY_CODE_BM,
	COUNTRY_CODE_BW, COUNTRY_CODE_CK, COUNTRY_CODE_CO, COUNTRY_CODE_CU,
	COUNTRY_CODE_CX, COUNTRY_CODE_FM, COUNTRY_CODE_FO, COUNTRY_CODE_GD,
	COUNTRY_CODE_GG, COUNTRY_CODE_GI, COUNTRY_CODE_IM, COUNTRY_CODE_IR,
	COUNTRY_CODE_JE, COUNTRY_CODE_KH, COUNTRY_CODE_KI, COUNTRY_CODE_KP,
	COUNTRY_CODE_KY, COUNTRY_CODE_LB, COUNTRY_CODE_LK, COUNTRY_CODE_LR,
	COUNTRY_CODE_MH, COUNTRY_CODE_MM, COUNTRY_CODE_MN, COUNTRY_CODE_NF,
	COUNTRY_CODE_NI, COUNTRY_CODE_NU, COUNTRY_CODE_PM, COUNTRY_CODE_PN,
	COUNTRY_CODE_PS, COUNTRY_CODE_PW, COUNTRY_CODE_SD, COUNTRY_CODE_SS,
	COUNTRY_CODE_SY, COUNTRY_CODE_TL, COUNTRY_CODE_TT, COUNTRY_CODE_WS
};

static const uint16_t g_u2CountryGroup2[] = {
	COUNTRY_CODE_AW, COUNTRY_CODE_LA, COUNTRY_CODE_UG
};

static const uint16_t g_u2CountryGroup3[] = {
	COUNTRY_CODE_AD, COUNTRY_CODE_AL, COUNTRY_CODE_AR, COUNTRY_CODE_AT,
	COUNTRY_CODE_BA, COUNTRY_CODE_BE, COUNTRY_CODE_BG, COUNTRY_CODE_BY,
	COUNTRY_CODE_CH, COUNTRY_CODE_CV, COUNTRY_CODE_CY, COUNTRY_CODE_CZ,
	COUNTRY_CODE_DK, COUNTRY_CODE_DO, COUNTRY_CODE_DZ, COUNTRY_CODE_EE,
	COUNTRY_CODE_ES, COUNTRY_CODE_ET, COUNTRY_CODE_EU, COUNTRY_CODE_FI,
	COUNTRY_CODE_FK, COUNTRY_CODE_FR, COUNTRY_CODE_GE, COUNTRY_CODE_GF,
	COUNTRY_CODE_GH, COUNTRY_CODE_GP, COUNTRY_CODE_GR, COUNTRY_CODE_HK,
	COUNTRY_CODE_HR, COUNTRY_CODE_HU, COUNTRY_CODE_IE, COUNTRY_CODE_IQ,
	COUNTRY_CODE_IT, COUNTRY_CODE_KE, COUNTRY_CODE_KZ, COUNTRY_CODE_LC,
	COUNTRY_CODE_LI, COUNTRY_CODE_LS, COUNTRY_CODE_LT, COUNTRY_CODE_LU,
	COUNTRY_CODE_LV, COUNTRY_CODE_MC, COUNTRY_CODE_MD, COUNTRY_CODE_ME,
	COUNTRY_CODE_MF, COUNTRY_CODE_MK, COUNTRY_CODE_MQ, COUNTRY_CODE_MR,
	COUNTRY_CODE_MS, COUNTRY_CODE_MT, COUNTRY_CODE_MU, COUNTRY_CODE_MZ,
	COUNTRY_CODE_NA, COUNTRY_CODE_NL, COUNTRY_CODE_OM, COUNTRY_CODE_PF,
	COUNTRY_CODE_PH, COUNTRY_CODE_PL, COUNTRY_CODE_PT, COUNTRY_CODE_RE,
	COUNTRY_CODE_RO, COUNTRY_CODE_RS, COUNTRY_CODE_SE, COUNTRY_CODE_SG,
	COUNTRY_CODE_SI, COUNTRY_CODE_SK, COUNTRY_CODE_SM, COUNTRY_CODE_SN,
	COUNTRY_CODE_TC, COUNTRY_CODE_TF, COUNTRY_CODE_TR, COUNTRY_CODE_UA,
	COUNTRY_CODE_UZ, COUNTRY_CODE_VA, COUNTRY_CODE_VC, COUNTRY_CODE_VG,
	COUNTRY_CODE_VN, COUNTRY_CODE_YT, COUNTRY_CODE_ZA, COUNTRY_CODE_ZW
};

static const uint16_t g_u2CountryGroup4[] = {
	COUNTRY_CODE_AU, COUNTRY_CODE_NZ, COUNTRY_CODE_EC, COUNTRY_CODE_PY,
	COUNTRY_CODE_TH, COUNTRY_CODE_UY
};

static const uint16_t g_u2CountryGroup5[] = { COUNTRY_CODE_RU };

static const uint16_t g_u2CountryGroup6[] = {
	COUNTRY_CODE_CL, COUNTRY_CODE_EG, COUNTRY_CODE_IN, COUNTRY_CODE_AG,
	COUNTRY_CODE_BS, COUNTRY_CODE_BH, COUNTRY_CODE_BB, COUNTRY_CODE_BN,
	COUNTRY_CODE_MV, COUNTRY_CODE_PA, COUNTRY_CODE_ZM, COUNTRY_CODE_CN
};

static const uint16_t g_u2CountryGroup7[] = { COUNTRY_CODE_MY };

static const uint16_t g_u2CountryGroup8[] = { COUNTRY_CODE_NP };

static const uint16_t g_u2CountryGroup9[] = {
	COUNTRY_CODE_IL, COUNTRY_CODE_AM, COUNTRY_CODE_KW, COUNTRY_CODE_MA,
	COUNTRY_CODE_NE, COUNTRY_CODE_TN
};

static const uint16_t g_u2CountryGroup10[] = {
	COUNTRY_CODE_JO, COUNTRY_CODE_PG
};

static const uint16_t g_u2CountryGroup11[] = { COUNTRY_CODE_AF };

static const uint16_t g_u2CountryGroup12[] = { COUNTRY_CODE_NG };

static const uint16_t g_u2CountryGroup13[] = {
	COUNTRY_CODE_PK, COUNTRY_CODE_QA, COUNTRY_CODE_BF, COUNTRY_CODE_GY,
	COUNTRY_CODE_HT, COUNTRY_CODE_JM, COUNTRY_CODE_MO, COUNTRY_CODE_MW,
	COUNTRY_CODE_RW, COUNTRY_CODE_KN, COUNTRY_CODE_TZ, COUNTRY_CODE_BD
};

static const uint16_t g_u2CountryGroup14[] = { COUNTRY_CODE_ID };

static const uint16_t g_u2CountryGroup15[] = {
	COUNTRY_CODE_AO, COUNTRY_CODE_BZ, COUNTRY_CODE_BJ, COUNTRY_CODE_BT,
	COUNTRY_CODE_BO, COUNTRY_CODE_BI, COUNTRY_CODE_CM, COUNTRY_CODE_CF,
	COUNTRY_CODE_TD, COUNTRY_CODE_KM, COUNTRY_CODE_CD, COUNTRY_CODE_CG,
	COUNTRY_CODE_CI, COUNTRY_CODE_DJ, COUNTRY_CODE_GQ, COUNTRY_CODE_ER,
	COUNTRY_CODE_FJ, COUNTRY_CODE_GA, COUNTRY_CODE_GM, COUNTRY_CODE_GN,
	COUNTRY_CODE_GW, COUNTRY_CODE_RKS, COUNTRY_CODE_KG, COUNTRY_CODE_LY,
	COUNTRY_CODE_MG, COUNTRY_CODE_ML, COUNTRY_CODE_NR, COUNTRY_CODE_NC,
	COUNTRY_CODE_ST, COUNTRY_CODE_SC, COUNTRY_CODE_SL, COUNTRY_CODE_SB,
	COUNTRY_CODE_SO, COUNTRY_CODE_SR, COUNTRY_CODE_SZ, COUNTRY_CODE_TJ,
	COUNTRY_CODE_TG, COUNTRY_CODE_TO, COUNTRY_CODE_TM, COUNTRY_CODE_TV,
	COUNTRY_CODE_VU, COUNTRY_CODE_YE
};

static const uint16_t g_u2CountryGroup16[] = {
	COUNTRY_CODE_US, COUNTRY_CODE_CA, COUNTRY_CODE_TW
};

static const uint16_t g_u2CountryGroup17[] = {
	COUNTRY_CODE_DM, COUNTRY_CODE_SV
};

static const uint16_t g_u2CountryGroup18[] = {
	COUNTRY_CODE_MX, COUNTRY_CODE_VE
};

static const uint16_t g_u2CountryGroup19[] = {
	COUNTRY_CODE_DE, COUNTRY_CODE_GB, COUNTRY_CODE_IS, COUNTRY_CODE_SA,
	COUNTRY_CODE_KR, COUNTRY_CODE_MP, COUNTRY_CODE_NO
};

static const uint16_t g_u2CountryGroup20[] = {
	COUNTRY_CODE_AS, COUNTRY_CODE_CR, COUNTRY_CODE_GT, COUNTRY_CODE_GU,
	COUNTRY_CODE_PR, COUNTRY_CODE_VI, COUNTRY_CODE_BR
};

static const uint16_t g_u2CountryGroup21[] = {
	COUNTRY_CODE_HN
};

static const uint16_t g_u2CountryGroup22[] = {
	COUNTRY_CODE_AE
};

static const uint16_t g_u2CountryGroup23[] = {
	COUNTRY_CODE_PE
};



#if (CFG_SUPPORT_SINGLE_SKU == 1)
struct mtk_regd_control g_mtk_regd_control = {
	.en = FALSE,
	.state = REGD_STATE_UNDEFINED
};

#if CFG_SUPPORT_BW160
#define BW_5G 160
#define BW_6G 160
#else
#define BW_5G 80
#define BW_6G 80
#endif

#if (CFG_SUPPORT_SINGLE_SKU_LOCAL_DB == 1)
#if (CFG_SUPPORT_WIFI_6G == 1)
const struct ieee80211_regdomain default_regdom_ww = {
	.n_reg_rules = 5,
	.alpha2 = "99",
	.reg_rules = {
	/* channels 1..13 */
	REG_RULE_LIGHT(2412-10, 2472+10, 40, 0),
	/* channels 14 */
	REG_RULE_LIGHT(2484-10, 2484+10, 20, 0),
	/* channel 36..64 */
	REG_RULE_LIGHT(5150-10, 5350+10, BW_5G, 0),
	/* channel 100..165 */
	REG_RULE_LIGHT(5470-10, 5850+10, BW_5G, 0),
	/* 6G channel 1..17 */
	REG_RULE_LIGHT(5935-10, 7135+10, BW_6G, 0),
	}
};
#else
const struct ieee80211_regdomain default_regdom_ww = {
	.n_reg_rules = 4,
	.alpha2 = "99",
	.reg_rules = {
	/* channels 1..13 */
	REG_RULE_LIGHT(2412-10, 2472+10, 40, 0),
	/* channels 14 */
	REG_RULE_LIGHT(2484-10, 2484+10, 20, 0),
	/* channel 36..64 */
	REG_RULE_LIGHT(5150-10, 5350+10, BW_5G, 0),
	/* channel 100..165 */
	REG_RULE_LIGHT(5470-10, 5850+10, BW_5G, 0),
	}
};
#endif
#endif

struct TX_PWR_LIMIT_SECTION {
	uint8_t ucSectionNum;
	const char *arSectionNames[TX_PWR_LIMIT_SECTION_NUM];
} gTx_Pwr_Limit_Section[] = {
	{5,
	 {"legacy", "ht20", "ht40", "vht20", "offset"}
	},
	{9,
	 {"cck", "ofdm", "ht20", "ht40", "vht20", "vht40",
	  "vht80", "vht160", "txbf_backoff"}
	},
};


const u8 gTx_Pwr_Limit_Element_Num[][TX_PWR_LIMIT_SECTION_NUM] = {
	{7, 6, 7, 7, 5},
	{POWER_LIMIT_SKU_CCK_NUM, POWER_LIMIT_SKU_OFDM_NUM,
	 POWER_LIMIT_SKU_HT20_NUM, POWER_LIMIT_SKU_HT40_NUM,
	 POWER_LIMIT_SKU_VHT20_NUM, POWER_LIMIT_SKU_VHT40_NUM,
	 POWER_LIMIT_SKU_VHT80_NUM, POWER_LIMIT_SKU_VHT160_NUM,
	 POWER_LIMIT_TXBF_BACKOFF_PARAM_NUM},
};

const char *gTx_Pwr_Limit_Element[]
	[TX_PWR_LIMIT_SECTION_NUM]
	[TX_PWR_LIMIT_ELEMENT_NUM] = {
	{
		{"cck1_2", "cck_5_11", "ofdm6_9", "ofdm12_18", "ofdm24_36",
		 "ofdm48", "ofdm54"},
		{"mcs0_8", "mcs1_2_9_10", "mcs3_4_11_12", "mcs5_13", "mcs6_14",
		 "mcs7_15"},
		{"mcs0_8", "mcs1_2_9_10", "mcs3_4_11_12", "mcs5_13", "mcs6_14",
		 "mcs7_15", "mcs32"},
		{"mcs0", "mcs1_2", "mcs3_4", "mcs5_6", "mcs7", "mcs8", "mcs9"},
		{"lg40", "lg80", "vht40", "vht80", "vht160nc"},
	},
	{
		{"c1", "c2", "c5", "c11"},
		{"o6", "o9", "o12", "o18",
		 "o24", "o36", "o48", "o54"},
		{"m0", "m1", "m2", "m3", "m4", "m5", "m6", "m7"},
		{"m0", "m1", "m2", "m3", "m4", "m5", "m6", "m7", "m32"},
		{"m0", "m1", "m2", "m3", "m4", "m5", "m6", "m7", "m8", "m9"},
		{"m0", "m1", "m2", "m3", "m4", "m5", "m6", "m7", "m8", "m9"},
		{"m0", "m1", "m2", "m3", "m4", "m5", "m6", "m7", "m8", "m9"},
		{"m0", "m1", "m2", "m3", "m4", "m5", "m6", "m7", "m8", "m9"},
		{"2to1"},
	},
};

static const int8_t gTx_Pwr_Limit_2g_Ch[] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const int8_t gTx_Pwr_Limit_5g_Ch[] = {
	36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 100, 102,
	104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 132,
	134, 136, 138, 140, 142, 144, 149, 151, 153, 155, 157, 159, 161, 165};

#define TX_PWR_LIMIT_2G_CH_NUM (ARRAY_SIZE(gTx_Pwr_Limit_2g_Ch))
#define TX_PWR_LIMIT_5G_CH_NUM (ARRAY_SIZE(gTx_Pwr_Limit_5g_Ch))

u_int8_t g_bTxBfBackoffExists = FALSE;

#endif

struct DOMAIN_INFO_ENTRY arSupportedRegDomains[] = {
	{
		(uint16_t *) g_u2CountryGroup0, sizeof(g_u2CountryGroup0) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{82, BAND_2G4, CHNL_SPAN_5, 14, 1, FALSE}
			,			/* CH_SET_2G4_14_14 */
			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 11, TRUE}
			,			/* CH_SET_UNII_WW_100_140 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{125, BAND_NULL, 0, 0, 0, FALSE}
				/* CH_SET_UNII_UPPER_NA */
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup1, sizeof(g_u2CountryGroup1) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 12, TRUE}
			,			/* CH_SET_UNII_WW_100_144 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup2, sizeof(g_u2CountryGroup2) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 12, TRUE}
			,			/* CH_SET_UNII_WW_100_144 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 4, FALSE}
			,			/* CH_SET_UNII_UPPER_149_161 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup3, sizeof(g_u2CountryGroup3) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 11, TRUE}
			,			/* CH_SET_UNII_WW_100_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup4, sizeof(g_u2CountryGroup4) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, TRUE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 5, TRUE}
			,			/* CH_SET_UNII_WW_100_116 */
			{121, BAND_5G, CHNL_SPAN_20, 132, 3, TRUE}
			,			/* CH_SET_UNII_WW_132_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
						/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			,
			{131, BAND_NULL, 0, 0, 0, FALSE}
						/* 6G_CH_1_233 */
#endif
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup5, sizeof(g_u2CountryGroup5) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 132, 3, TRUE}
			,			/* CH_SET_UNII_WW_132_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup6, sizeof(g_u2CountryGroup6) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup7, sizeof(g_u2CountryGroup7) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 8, TRUE}
			,			/* CH_SET_UNII_WW_100_128 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_6G, CHNL_SPAN_20, 1, 59, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup8, sizeof(g_u2CountryGroup8) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_5G, CHNL_SPAN_20, 149, 4, FALSE}
			,			/* CH_SET_UNII_UPPER_149_161 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup9, sizeof(g_u2CountryGroup9) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_UPPER_NA */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup10, sizeof(g_u2CountryGroup10) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_MID_NA */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup11, sizeof(g_u2CountryGroup11) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_MID_NA */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_UPPER_NA */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup12, sizeof(g_u2CountryGroup12) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_LOW_NA */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 11, TRUE}
			,			/* CH_SET_UNII_WW_100_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup13, sizeof(g_u2CountryGroup13) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_LOW_NA */
			{118, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_MID_NA */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup14, sizeof(g_u2CountryGroup14) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */
			{115, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_LOW_NA */
			{118, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_MID_NA */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_5G, CHNL_SPAN_20, 149, 4, FALSE}
			,			/* CH_SET_UNII_UPPER_149_161 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup15, sizeof(g_u2CountryGroup15) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */
			{115, BAND_5G, CHNL_SPAN_20, 36, 4, TRUE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_NA */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_UPPER_NA */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup16, sizeof(g_u2CountryGroup16) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 11, FALSE}
			,			/* CH_SET_2G4_1_11 */
			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 12, TRUE}
			,			/* CH_SET_UNII_WW_100_144 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_6G, CHNL_SPAN_20, 1, 59, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup17, sizeof(g_u2CountryGroup17) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 11, FALSE}
			,			/* CH_SET_2G4_1_11 */
			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 5, TRUE}
			,			/* CH_SET_UNII_WW_100_116 */
			{121, BAND_5G, CHNL_SPAN_20, 132, 3, TRUE}
			,			/* CH_SET_UNII_WW_132_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
						/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			,
			{131, BAND_NULL, 0, 0, 0, FALSE}
						/* 6G_CH_1_233 */
#endif
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup18, sizeof(g_u2CountryGroup18) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 11, FALSE}
			,			/* CH_SET_2G4_1_11 */
			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_NULL, 0, 0, 0, FALSE}
			,			/* CH_SET_UNII_WW_NA */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup19, sizeof(g_u2CountryGroup19) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 11, TRUE}
			,			/* CH_SET_UNII_WW_100_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_6G, CHNL_SPAN_20, 1, 59, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup20, sizeof(g_u2CountryGroup20) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 12, TRUE}
			,			/* CH_SET_UNII_WW_100_144 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_6G, CHNL_SPAN_20, 1, 59, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup21, sizeof(g_u2CountryGroup21) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 11, FALSE}
			,			/* CH_SET_2G4_1_11 */
			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 5, TRUE}
			,			/* CH_SET_UNII_WW_100_116 */
			{121, BAND_5G, CHNL_SPAN_20, 132, 3, TRUE}
			,			/* CH_SET_UNII_WW_132_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
						/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			,
			{131, BAND_6G, CHNL_SPAN_20, 1, 59, FALSE}
						/* 6G_CH_1_233 */
#endif
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup22, sizeof(g_u2CountryGroup22) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 12, TRUE}
			,			/* CH_SET_UNII_WW_100_144 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 4, FALSE}
			,			/* CH_SET_UNII_UPPER_149_161 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_6G, CHNL_SPAN_20, 1, 59, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
	,
	{
		(uint16_t *) g_u2CountryGroup23, sizeof(g_u2CountryGroup23) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */

			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 5, TRUE}
			,			/* CH_SET_UNII_WW_100_116 */
			{121, BAND_5G, CHNL_SPAN_20, 132, 3, TRUE}
			,			/* CH_SET_UNII_WW_132_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
						/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			,
			{131, BAND_6G, CHNL_SPAN_20, 1, 59, FALSE}
						/* 6G_CH_1_233 */
#endif
		}
	}
	,
	{
		/* Note: Default group if no matched country code */
		NULL, 0,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 13, FALSE}
			,			/* CH_SET_2G4_1_13 */
			{115, BAND_5G, CHNL_SPAN_20, 36, 4, FALSE}
			,			/* CH_SET_UNII_LOW_36_48 */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 12, TRUE}
			,			/* CH_SET_UNII_WW_100_144 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 5, FALSE}
			,			/* CH_SET_UNII_UPPER_149_165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
			{131, BAND_NULL, 0, 0, 0, FALSE}
			,			/* 6G_CH_1_233 */
#endif
			{0, BAND_NULL, 0, 0, 0, FALSE}
		}
	}
};

static const uint16_t g_u2CountryGroup0_Passive[] = {
	COUNTRY_CODE_TW
};

struct DOMAIN_INFO_ENTRY arSupportedRegDomains_Passive[] = {
	{
		(uint16_t *) g_u2CountryGroup0_Passive,
		sizeof(g_u2CountryGroup0_Passive) / 2,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 0, FALSE}
			,			/* CH_SET_2G4_1_14_NA */
			{82, BAND_2G4, CHNL_SPAN_5, 14, 0, FALSE}
			,

			{115, BAND_5G, CHNL_SPAN_20, 36, 0, FALSE}
			,			/* CH_SET_UNII_LOW_NA */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 11, TRUE}
			,			/* CH_SET_UNII_WW_100_140 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 0, FALSE}
						/* CH_SET_UNII_UPPER_NA */
#if (CFG_SUPPORT_WIFI_6G == 1)
			,
			{131, BAND_6G, CHNL_SPAN_20, 1, 0, FALSE}
						/* 6G_CH_1_233 */
#endif
		}
	}
	,
	{
		/* Default passive scan channel table: ch52~64, ch100~144 */
		NULL,
		0,
		{
			{81, BAND_2G4, CHNL_SPAN_5, 1, 0, FALSE}
			,			/* CH_SET_2G4_1_14_NA */
			{82, BAND_2G4, CHNL_SPAN_5, 14, 0, FALSE}
			,

			{115, BAND_5G, CHNL_SPAN_20, 36, 0, FALSE}
			,			/* CH_SET_UNII_LOW_NA */
			{118, BAND_5G, CHNL_SPAN_20, 52, 4, TRUE}
			,			/* CH_SET_UNII_MID_52_64 */
			{121, BAND_5G, CHNL_SPAN_20, 100, 12, TRUE}
			,			/* CH_SET_UNII_WW_100_144 */
			{125, BAND_5G, CHNL_SPAN_20, 149, 0, FALSE}
						/* CH_SET_UNII_UPPER_NA */
#if (CFG_SUPPORT_WIFI_6G == 1)
			,
			{131, BAND_6G, CHNL_SPAN_20, 1, 0, FALSE}
						/* 6G_CH_1_233 */
#endif
		}
	}
};

#if (CFG_SUPPORT_PWR_LIMIT_COUNTRY == 1)
struct SUBBAND_CHANNEL g_rRlmSubBand[] = {

	{BAND_2G4, BAND_2G4_LOWER_BOUND, BAND_2G4_UPPER_BOUND, 1, 0} /* 2.4G */
	,
	{BAND_5G, UNII1_LOWER_BOUND, UNII1_UPPER_BOUND, 2, 0} /* 5G 36~50 */
	,
	{BAND_5G, UNII2A_LOWER_BOUND, UNII2A_UPPER_BOUND, 2, 0} /* 5G 52~64 */
	,
	{BAND_5G, UNII2C_LOWER_BOUND, UNII2C_UPPER_BOUND, 2, 0} /* 5G 100~144 */
	,
	{BAND_5G, UNII3_LOWER_BOUND, UNII3_UPPER_BOUND, 2, 0} /* 5G 149~165 */
#if (CFG_SUPPORT_WIFI_6G == 1)
	,
	{BAND_6G, UNII5_LOWER_BOUND, UNII5_UPPER_BOUND, 2, 0}  /* 6G 1~93 */
	,
	{BAND_6G, UNII6_LOWER_BOUND, UNII6_UPPER_BOUND, 2, 0}  /* 6G 97~115 */
	,
	{BAND_6G, UNII7_LOWER_BOUND, UNII7_UPPER_BOUND, 2, 0}  /* 6G 117~185 */
	,
	{BAND_6G, UNII8_LOWER_BOUND, UNII8_UPPER_BOUND, 2, 0}  /* 6G 187~233 */
#endif
};
#endif
/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in/out]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
struct DOMAIN_INFO_ENTRY *rlmDomainGetDomainInfo(struct ADAPTER *prAdapter)
{
#define REG_DOMAIN_GROUP_NUM  \
	(sizeof(arSupportedRegDomains) / sizeof(struct DOMAIN_INFO_ENTRY))
#define REG_DOMAIN_DEF_IDX	(REG_DOMAIN_GROUP_NUM - 1)

	struct DOMAIN_INFO_ENTRY *prDomainInfo;
	struct REG_INFO *prRegInfo;
	uint16_t u2TargetCountryCode;
	uint16_t i, j;

	ASSERT(prAdapter);

	if (prAdapter->prDomainInfo)
		return prAdapter->prDomainInfo;

	prRegInfo = &prAdapter->prGlueInfo->rRegInfo;

	DBGLOG(RLM, TRACE, "eRegChannelListMap=%d, u2CountryCode=0x%04x\n",
			   prRegInfo->eRegChannelListMap,
			   prAdapter->rWifiVar.u2CountryCode);

	/*
	 * Domain info can be specified by given idx of arSupportedRegDomains
	 * table, customized, or searched by country code,
	 * only one is set among these three methods in NVRAM.
	 */
	if (prRegInfo->eRegChannelListMap == REG_CH_MAP_TBL_IDX &&
	    prRegInfo->ucRegChannelListIndex < REG_DOMAIN_GROUP_NUM) {
		/* by given table idx */
		DBGLOG(RLM, TRACE, "ucRegChannelListIndex=%d\n",
		       prRegInfo->ucRegChannelListIndex);
		prDomainInfo = &arSupportedRegDomains
					[prRegInfo->ucRegChannelListIndex];
	} else if (prRegInfo->eRegChannelListMap == REG_CH_MAP_CUSTOMIZED) {
		/* by customized */
		prDomainInfo = &prRegInfo->rDomainInfo;
	} else {
		/* by country code */
		u2TargetCountryCode =
				prAdapter->rWifiVar.u2CountryCode;

		for (i = 0; i < REG_DOMAIN_GROUP_NUM; i++) {
			prDomainInfo = &arSupportedRegDomains[i];

			if ((prDomainInfo->u4CountryNum &&
			     prDomainInfo->pu2CountryGroup) ||
			    prDomainInfo->u4CountryNum == 0) {
				for (j = 0;
				     j < prDomainInfo->u4CountryNum;
				     j++) {
					if (prDomainInfo->pu2CountryGroup[j] ==
							u2TargetCountryCode)
						break;
				}
				if (j < prDomainInfo->u4CountryNum)
					break;	/* Found */
			}
		}

		/* If no matched country code,
		 * use the default regulatory domain
		 */
		if (i >= REG_DOMAIN_GROUP_NUM) {
			DBGLOG(RLM, INFO,
			       "No matched country code, use the default regulatory domain\n");
			prDomainInfo = &arSupportedRegDomains
							[REG_DOMAIN_DEF_IDX];
		}
	}

	prAdapter->prDomainInfo = prDomainInfo;
	return prDomainInfo;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Retrieve the supported channel list of specified band
 *
 * \param[in/out] eSpecificBand:   BAND_2G4, BAND_5G or BAND_NULL
 *                                 (both 2.4G and 5G)
 *                fgNoDfs:         whether to exculde DFS channels
 *                ucMaxChannelNum: max array size
 *                pucNumOfChannel: pointer to returned channel number
 *                paucChannelList: pointer to returned channel list array
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void
rlmDomainGetChnlList_V2(struct ADAPTER *prAdapter,
			enum ENUM_BAND eSpecificBand, u_int8_t fgNoDfs,
			uint8_t ucMaxChannelNum, uint8_t *pucNumOfChannel,
			struct RF_CHANNEL_INFO *paucChannelList)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	enum ENUM_BAND band;
	uint8_t max_count, i, ucNum;
	struct CMD_DOMAIN_CHANNEL *prCh;

	if (eSpecificBand == BAND_2G4) {
		i = 0;
		max_count = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ);
	} else if (eSpecificBand == BAND_5G) {
		i = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ);
		max_count = rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ) +
			rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ);
	}
#if (CFG_SUPPORT_WIFI_6G == 1)
	else if (eSpecificBand == BAND_6G) {
		i = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ)
			+ rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ);
		max_count = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ)
			+ rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ)
			+rlmDomainGetActiveChannelCount(KAL_BAND_6GHZ);
	}
#endif
	else {
		i = 0;
		max_count =
#if (CFG_SUPPORT_WIFI_6G == 1)
			rlmDomainGetActiveChannelCount(KAL_BAND_6GHZ) +
#endif
			rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ) +
			rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ);
	}

	ucNum = 0;
	for (; i < max_count; i++) {
		prCh = rlmDomainGetActiveChannels() + i;
		if (fgNoDfs && (prCh->eFlags & IEEE80211_CHAN_RADAR))
			continue; /*not match*/

		if (i < rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ))
			band = BAND_2G4;
#if (CFG_SUPPORT_WIFI_6G == 1)
		else if (i < rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ) +
			rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ))
			band = BAND_5G;
		else
			band = BAND_6G;
#else
		else
			band = BAND_5G;
#endif

		paucChannelList[ucNum].eBand = band;
		paucChannelList[ucNum].ucChannelNum = prCh->u2ChNum;

		ucNum++;
		if (ucMaxChannelNum == ucNum)
			break;
	}

	*pucNumOfChannel = ucNum;
#else
	*pucNumOfChannel = 0;
#endif
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Check if Channel supported by HW
 *
 * \param[in/out] eBand:          BAND_2G4, BAND_5G or BAND_NULL
 *                                (both 2.4G and 5G)
 *                ucNumOfChannel: channel number
 *
 * \return TRUE/FALSE
 */
/*----------------------------------------------------------------------------*/
u_int8_t rlmIsValidChnl(struct ADAPTER *prAdapter, uint8_t ucNumOfChannel,
			enum ENUM_BAND eBand)
{
	struct ieee80211_supported_band *channelList;
	int i, chSize;
	struct GLUE_INFO *prGlueInfo;
	struct wiphy *prWiphy;

	prGlueInfo = prAdapter->prGlueInfo;
	prWiphy = wlanGetWiphy();

#if (CFG_SUPPORT_WIFI_6G == 1)
	if (eBand == BAND_6G) {
		channelList = prWiphy->bands[KAL_BAND_6GHZ];
		chSize = channelList->n_channels;
	} else
#endif
	if (eBand == BAND_5G) {
		channelList = prWiphy->bands[KAL_BAND_5GHZ];
		chSize = channelList->n_channels;
	} else if (eBand == BAND_2G4) {
		channelList = prWiphy->bands[KAL_BAND_2GHZ];
		chSize = channelList->n_channels;
	} else
		return FALSE;

	for (i = 0; i < chSize; i++) {
		if ((channelList->channels[i]).hw_value == ucNumOfChannel)
			return TRUE;
	}
	return FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Retrieve the supported channel list of specified band
 *
 * \param[in/out] eSpecificBand:   BAND_2G4, BAND_5G or BAND_NULL
 *                                 (both 2.4G and 5G)
 *                fgNoDfs:         whether to exculde DFS channels
 *                ucMaxChannelNum: max array size
 *                pucNumOfChannel: pointer to returned channel number
 *                paucChannelList: pointer to returned channel list array
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void
rlmDomainGetChnlList(struct ADAPTER *prAdapter,
		     enum ENUM_BAND eSpecificBand, u_int8_t fgNoDfs,
		     uint8_t ucMaxChannelNum, uint8_t *pucNumOfChannel,
		     struct RF_CHANNEL_INFO *paucChannelList)
{
	uint8_t i, j, ucNum, ch;
	struct DOMAIN_SUBBAND_INFO *prSubband;
	struct DOMAIN_INFO_ENTRY *prDomainInfo;

	ASSERT(prAdapter);
	ASSERT(paucChannelList);
	ASSERT(pucNumOfChannel);

	if (regd_is_single_sku_en())
		return rlmDomainGetChnlList_V2(prAdapter, eSpecificBand,
					       fgNoDfs, ucMaxChannelNum,
					       pucNumOfChannel,
					       paucChannelList);

	/* If no matched country code, the final one will be used */
	prDomainInfo = rlmDomainGetDomainInfo(prAdapter);
	ASSERT(prDomainInfo);

	ucNum = 0;
	for (i = 0; i < MAX_SUBBAND_NUM; i++) {
		prSubband = &prDomainInfo->rSubBand[i];

		if (prSubband->ucBand == BAND_NULL ||
		    prSubband->ucBand >= BAND_NUM ||
		    (prSubband->ucBand == BAND_5G &&
		     !prAdapter->fgEnable5GBand))
			continue;

#if (CFG_SUPPORT_WIFI_6G == 1)
		if (prSubband->ucBand == BAND_6G && !prAdapter->fgIsHwSupport6G)
			continue;
#endif

		/* repoert to upper layer only non-DFS channel
		 * for ap mode usage
		 */
		if (fgNoDfs == TRUE && prSubband->fgDfs == TRUE)
			continue;

		if (eSpecificBand == BAND_NULL ||
		    prSubband->ucBand == eSpecificBand) {
			for (j = 0; j < prSubband->ucNumChannels; j++) {
				if (ucNum >= ucMaxChannelNum)
					break;

				ch = prSubband->ucFirstChannelNum +
				     j * prSubband->ucChannelSpan;
				if (!rlmIsValidChnl(prAdapter, ch,
						prSubband->ucBand)) {
					DBGLOG(RLM, INFO,
					       "Not support ch%d!\n", ch);
					continue;
				}
				paucChannelList[ucNum].eBand =
							prSubband->ucBand;
				paucChannelList[ucNum].ucChannelNum = ch;
				paucChannelList[ucNum].eDFS = prSubband->fgDfs;
				ucNum++;
			}
		}
	}

	*pucNumOfChannel = ucNum;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Retrieve DFS channels from 5G band
 *
 * \param[in/out] ucMaxChannelNum: max array size
 *                pucNumOfChannel: pointer to returned channel number
 *                paucChannelList: pointer to returned channel list array
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/
void rlmDomainGetDfsChnls(struct ADAPTER *prAdapter,
			  uint8_t ucMaxChannelNum, uint8_t *pucNumOfChannel,
			  struct RF_CHANNEL_INFO *paucChannelList)
{
	uint8_t i, j, ucNum, ch;
	struct DOMAIN_SUBBAND_INFO *prSubband;
	struct DOMAIN_INFO_ENTRY *prDomainInfo;

	ASSERT(prAdapter);
	ASSERT(paucChannelList);
	ASSERT(pucNumOfChannel);

	prDomainInfo = rlmDomainGetDomainInfo(prAdapter);
	ASSERT(prDomainInfo);

	ucNum = 0;
	for (i = 0; i < MAX_SUBBAND_NUM; i++) {
		prSubband = &prDomainInfo->rSubBand[i];

		if (prSubband->ucBand == BAND_5G) {
			if (!prAdapter->fgEnable5GBand)
				continue;

			if (prSubband->fgDfs == TRUE) {
				for (j = 0; j < prSubband->ucNumChannels; j++) {
					if (ucNum >= ucMaxChannelNum)
						break;

					ch = prSubband->ucFirstChannelNum +
					     j * prSubband->ucChannelSpan;
					if (!rlmIsValidChnl(prAdapter, ch,
							prSubband->ucBand)) {
						DBGLOG(RLM, INFO,
					       "Not support ch%d!\n", ch);
						continue;
					}

					paucChannelList[ucNum].eBand =
					    prSubband->ucBand;
					paucChannelList[ucNum].ucChannelNum =
					    ch;
					ucNum++;
				}
			}
		}
	}

	*pucNumOfChannel = ucNum;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Check whether input is dfs channel
 *
 * \param[in/out] ucChannel: channel to check
 *
 * \return true if dfs channel
 */
/*----------------------------------------------------------------------------*/
u_int8_t rlmDomainIsDfsChnls(struct ADAPTER *prAdapter, uint8_t ucChannel)
{
	uint8_t ucNumOfChannel = 0;
	struct RF_CHANNEL_INFO aucChannelList[64] = {};
	uint8_t ucCount = 0;

	rlmDomainGetDfsChnls(prAdapter, 64, &ucNumOfChannel, aucChannelList);
	for (; ucCount < ucNumOfChannel; ucCount++) {
		if (ucChannel == aucChannelList[ucCount].ucChannelNum)
			return TRUE;
	}

	return FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief
 *
 * @param[in]
 *
 * @return (none)
 */
/*----------------------------------------------------------------------------*/
void rlmDomainSendCmd(struct ADAPTER *prAdapter)
{
	if (!regd_is_single_sku_en())
		rlmDomainSendPassiveScanInfoCmd(prAdapter);
	rlmDomainSendDomainInfoCmd(prAdapter);
#if CFG_SUPPORT_PWR_LIMIT_COUNTRY
	rlmDomainSendPwrLimitCmd(prAdapter);
#endif
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief
 *
 * @param[in]
 *
 * @return (none)
 */
/*----------------------------------------------------------------------------*/
void rlmDomainSendDomainInfoCmd_V2(struct ADAPTER *prAdapter)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	u8 max_channel_count = 0;
	u32 buff_max_size, buff_valid_size;
	struct CMD_SET_DOMAIN_INFO_V2 *prCmd;
	struct CMD_DOMAIN_ACTIVE_CHANNEL_LIST *prChs;
	struct wiphy *pWiphy;


	pWiphy = wlanGetWiphy();
	if (pWiphy->bands[KAL_BAND_2GHZ] != NULL)
		max_channel_count += pWiphy->bands[KAL_BAND_2GHZ]->n_channels;
	if (pWiphy->bands[KAL_BAND_5GHZ] != NULL)
		max_channel_count += pWiphy->bands[KAL_BAND_5GHZ]->n_channels;
#if (CFG_SUPPORT_WIFI_6G == 1)
	if (pWiphy->bands[KAL_BAND_6GHZ] != NULL)
		max_channel_count += pWiphy->bands[KAL_BAND_6GHZ]->n_channels;
#endif
	if (max_channel_count == 0) {
		DBGLOG(RLM, ERROR, "%s, invalid channel count.\n", __func__);
		ASSERT(0);
	}


	buff_max_size = sizeof(struct CMD_SET_DOMAIN_INFO_V2) +
		max_channel_count * sizeof(struct CMD_DOMAIN_CHANNEL);

	prCmd = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, buff_max_size);
	if (!prCmd) {
		DBGLOG(RLM, ERROR, "Alloc cmd buffer failed\n");
		return;
	}
	prChs = &(prCmd->arActiveChannels);


	/*
	 * Fill in the active channels
	 */
	rlmExtractChannelInfo(max_channel_count, prChs);

	prCmd->u4CountryCode = rlmDomainGetCountryCode();
	prCmd->uc2G4Bandwidth = prAdapter->rWifiVar.uc2G4BandwidthMode;
	prCmd->uc5GBandwidth = prAdapter->rWifiVar.uc5GBandwidthMode;
	prCmd->uc6GBandwidth = prAdapter->rWifiVar.uc6GBandwidthMode;
	prCmd->aucPadding[0] = 0;

	buff_valid_size = sizeof(struct CMD_SET_DOMAIN_INFO_V2) +
		(prChs->u1ActiveChNum2g + prChs->u1ActiveChNum5g +
		prChs->u1ActiveChNum6g) *
		sizeof(struct CMD_DOMAIN_CHANNEL);

	DBGLOG(RLM, INFO,
	       "rlmDomainSendDomainInfoCmd_V2(), buff_valid_size = 0x%x\n",
	       buff_valid_size);


	/* Set domain info to chip */
	wlanSendSetQueryCmd(prAdapter, /* prAdapter */
			    CMD_ID_SET_DOMAIN_INFO, /* ucCID */
			    TRUE,  /* fgSetQuery */
			    FALSE, /* fgNeedResp */
			    FALSE, /* fgIsOid */
			    NULL,  /* pfCmdDoneHandler */
			    NULL,  /* pfCmdTimeoutHandler */
			    buff_valid_size,
			    (uint8_t *) prCmd, /* pucInfoBuffer */
			    NULL,  /* pvSetQueryBuffer */
			    0      /* u4SetQueryBufferLen */
	    );

	cnmMemFree(prAdapter, prCmd);
#endif
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief
 *
 * @param[in]
 *
 * @return (none)
 */
/*----------------------------------------------------------------------------*/
void rlmDomainSendDomainInfoCmd(struct ADAPTER *prAdapter)
{
	struct DOMAIN_INFO_ENTRY *prDomainInfo;
	struct CMD_SET_DOMAIN_INFO *prCmd;
	struct DOMAIN_SUBBAND_INFO *prSubBand;
	uint8_t i;

	if (regd_is_single_sku_en())
		return rlmDomainSendDomainInfoCmd_V2(prAdapter);


	prDomainInfo = rlmDomainGetDomainInfo(prAdapter);
	ASSERT(prDomainInfo);

	prCmd = cnmMemAlloc(prAdapter, RAM_TYPE_BUF,
			    sizeof(struct CMD_SET_DOMAIN_INFO));
	if (!prCmd) {
		DBGLOG(RLM, ERROR, "Alloc cmd buffer failed\n");
		return;
	}
	kalMemZero(prCmd, sizeof(struct CMD_SET_DOMAIN_INFO));

	prCmd->u2CountryCode =
		prAdapter->rWifiVar.u2CountryCode;
	prCmd->u2IsSetPassiveScan = 0;
	prCmd->uc2G4Bandwidth =
		prAdapter->rWifiVar.uc2G4BandwidthMode;
	prCmd->uc5GBandwidth =
		prAdapter->rWifiVar.uc5GBandwidthMode;
	prCmd->aucReserved[0] = 0;
	prCmd->aucReserved[1] = 0;

	for (i = 0; i < MAX_SUBBAND_NUM; i++) {
		prSubBand = &prDomainInfo->rSubBand[i];

		prCmd->rSubBand[i].ucRegClass = prSubBand->ucRegClass;
		prCmd->rSubBand[i].ucBand = prSubBand->ucBand;

		if (prSubBand->ucBand != BAND_NULL && prSubBand->ucBand
								< BAND_NUM) {
			prCmd->rSubBand[i].ucChannelSpan
						= prSubBand->ucChannelSpan;
			prCmd->rSubBand[i].ucFirstChannelNum
						= prSubBand->ucFirstChannelNum;
			prCmd->rSubBand[i].ucNumChannels
						= prSubBand->ucNumChannels;
		}
	}

	/* Set domain info to chip */
	wlanSendSetQueryCmd(prAdapter, /* prAdapter */
		CMD_ID_SET_DOMAIN_INFO, /* ucCID */
		TRUE,  /* fgSetQuery */
		FALSE, /* fgNeedResp */
		FALSE, /* fgIsOid */
		NULL,  /* pfCmdDoneHandler */
		NULL,  /* pfCmdTimeoutHandler */
		sizeof(struct CMD_SET_DOMAIN_INFO), /* u4SetQueryInfoLen */
		(uint8_t *) prCmd, /* pucInfoBuffer */
		NULL,  /* pvSetQueryBuffer */
		0      /* u4SetQueryBufferLen */
	    );

	cnmMemFree(prAdapter, prCmd);
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief
 *
 * @param[in]
 *
 * @return (none)
 */
/*----------------------------------------------------------------------------*/
void rlmDomainSendPassiveScanInfoCmd(struct ADAPTER *prAdapter)
{
#define REG_DOMAIN_PASSIVE_DEF_IDX	1
#define REG_DOMAIN_PASSIVE_GROUP_NUM \
	(sizeof(arSupportedRegDomains_Passive)	\
	 / sizeof(struct DOMAIN_INFO_ENTRY))

	struct DOMAIN_INFO_ENTRY *prDomainInfo;
	struct CMD_SET_DOMAIN_INFO *prCmd;
	struct DOMAIN_SUBBAND_INFO *prSubBand;
	uint16_t u2TargetCountryCode;
	uint8_t i, j;

	prCmd = cnmMemAlloc(prAdapter, RAM_TYPE_BUF,
			    sizeof(struct CMD_SET_DOMAIN_INFO));
	if (!prCmd) {
		DBGLOG(RLM, ERROR, "Alloc cmd buffer failed\n");
		return;
	}
	kalMemZero(prCmd, sizeof(struct CMD_SET_DOMAIN_INFO));

	prCmd->u2CountryCode = prAdapter->rWifiVar.u2CountryCode;
	prCmd->u2IsSetPassiveScan = 1;
	prCmd->uc2G4Bandwidth =
		prAdapter->rWifiVar.uc2G4BandwidthMode;
	prCmd->uc5GBandwidth =
		prAdapter->rWifiVar.uc5GBandwidthMode;
	prCmd->aucReserved[0] = 0;
	prCmd->aucReserved[1] = 0;

	DBGLOG(RLM, TRACE, "u2CountryCode=0x%04x\n",
	       prAdapter->rWifiVar.u2CountryCode);

	u2TargetCountryCode = prAdapter->rWifiVar.u2CountryCode;

	for (i = 0; i < REG_DOMAIN_PASSIVE_GROUP_NUM; i++) {
		prDomainInfo = &arSupportedRegDomains_Passive[i];

		for (j = 0; j < prDomainInfo->u4CountryNum; j++) {
			if (prDomainInfo->pu2CountryGroup[j] ==
						u2TargetCountryCode)
				break;
		}
		if (j < prDomainInfo->u4CountryNum)
			break;	/* Found */
	}

	if (i >= REG_DOMAIN_PASSIVE_GROUP_NUM)
		prDomainInfo = &arSupportedRegDomains_Passive
					[REG_DOMAIN_PASSIVE_DEF_IDX];

	for (i = 0; i < MAX_SUBBAND_NUM; i++) {
		prSubBand = &prDomainInfo->rSubBand[i];

		prCmd->rSubBand[i].ucRegClass = prSubBand->ucRegClass;
		prCmd->rSubBand[i].ucBand = prSubBand->ucBand;

		if (prSubBand->ucBand != BAND_NULL && prSubBand->ucBand
		    < BAND_NUM) {
			prCmd->rSubBand[i].ucChannelSpan =
						prSubBand->ucChannelSpan;
			prCmd->rSubBand[i].ucFirstChannelNum =
						prSubBand->ucFirstChannelNum;
			prCmd->rSubBand[i].ucNumChannels =
						prSubBand->ucNumChannels;
		}
	}

	/* Set passive scan channel info to chip */
	wlanSendSetQueryCmd(prAdapter, /* prAdapter */
		CMD_ID_SET_DOMAIN_INFO, /* ucCID */
		TRUE,  /* fgSetQuery */
		FALSE, /* fgNeedResp */
		FALSE, /* fgIsOid */
		NULL,  /* pfCmdDoneHandler */
		NULL,  /* pfCmdTimeoutHandler */
		sizeof(struct CMD_SET_DOMAIN_INFO), /* u4SetQueryInfoLen */
		(uint8_t *) prCmd, /* pucInfoBuffer */
		NULL,  /* pvSetQueryBuffer */
		0      /* u4SetQueryBufferLen */
	    );

	cnmMemFree(prAdapter, prCmd);
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in/out]
 *
 * \return TRUE  Legal channel
 *         FALSE Illegal channel for current regulatory domain
 */
/*----------------------------------------------------------------------------*/
u_int8_t rlmDomainIsLegalChannel_V2(struct ADAPTER *prAdapter,
				    enum ENUM_BAND eBand,
				    uint8_t ucChannel)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	uint8_t idx, start_idx, end_idx;
	struct CMD_DOMAIN_CHANNEL *prCh;

#if (CFG_SUPPORT_WIFI_6G == 1)
	if (eBand == BAND_6G) {
		start_idx = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ)
			+ rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ);
		end_idx = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ) +
			rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ) +
			rlmDomainGetActiveChannelCount(KAL_BAND_6GHZ);
	} else
#endif
	if (eBand == BAND_2G4) {
		start_idx = 0;
		end_idx = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ);
	} else {
		start_idx = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ);
		end_idx = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ) +
				rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ);
	}

	for (idx = start_idx; idx < end_idx; idx++) {
		prCh = rlmDomainGetActiveChannels() + idx;

		if (prCh->u2ChNum == ucChannel)
			return TRUE;
	}

	return FALSE;
#else
	return FALSE;
#endif
}

u_int8_t rlmDomainIsLegalChannel(struct ADAPTER *prAdapter,
				 enum ENUM_BAND eBand, uint8_t ucChannel)
{
	uint8_t i, j;
	struct DOMAIN_SUBBAND_INFO *prSubband;
	struct DOMAIN_INFO_ENTRY *prDomainInfo;

	if (regd_is_single_sku_en())
		return rlmDomainIsLegalChannel_V2(prAdapter, eBand, ucChannel);

	prDomainInfo = rlmDomainGetDomainInfo(prAdapter);
	ASSERT(prDomainInfo);

	for (i = 0; i < MAX_SUBBAND_NUM; i++) {
		prSubband = &prDomainInfo->rSubBand[i];

		if (prSubband->ucBand == BAND_5G && !prAdapter->fgEnable5GBand)
			continue;

#if (CFG_SUPPORT_WIFI_6G == 1)
		if (prSubband->ucBand == BAND_6G && !prAdapter->fgIsHwSupport6G)
			continue;
#endif

		if (prSubband->ucBand == eBand) {
			for (j = 0; j < prSubband->ucNumChannels; j++) {
				if ((prSubband->ucFirstChannelNum + j *
				    prSubband->ucChannelSpan) == ucChannel) {
					if (!rlmIsValidChnl(prAdapter,
						    ucChannel,
						    prSubband->ucBand)) {
						DBGLOG(RLM, INFO,
						       "Not support ch%d!\n",
						       ucChannel);
						return FALSE;
					} else
						return TRUE;

				}
			}
		}
	}

	return FALSE;
}

u_int8_t rlmDomainIsLegalDfsChannel(struct ADAPTER *prAdapter,
		enum ENUM_BAND eBand, uint8_t ucChannel)
{
	uint8_t i, j;
	struct DOMAIN_SUBBAND_INFO *prSubband;
	struct DOMAIN_INFO_ENTRY *prDomainInfo;

	prDomainInfo = rlmDomainGetDomainInfo(prAdapter);
	ASSERT(prDomainInfo);

	for (i = 0; i < MAX_SUBBAND_NUM; i++) {
		prSubband = &prDomainInfo->rSubBand[i];

		if (prSubband->ucBand == BAND_5G
			&& !prAdapter->fgEnable5GBand)
			continue;

		if (prSubband->ucBand == eBand
			&& prSubband->fgDfs == TRUE) {
			for (j = 0; j < prSubband->ucNumChannels; j++) {
				if ((prSubband->ucFirstChannelNum + j *
					prSubband->ucChannelSpan)
					== ucChannel) {
					return TRUE;
				}
			}
		}
	}

	return FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in/out]
 *
 * \return none
 */
/*----------------------------------------------------------------------------*/

uint32_t rlmDomainSupOperatingClassIeFill(uint8_t *pBuf)
{
	/*
	 *  The Country element should only be included for
	 *  Status Code 0 (Successful).
	 */
	uint32_t u4IeLen;
	uint8_t aucClass[12] = { 0x01, 0x02, 0x03, 0x05, 0x16, 0x17, 0x19, 0x1b,
		0x1c, 0x1e, 0x20, 0x21
	};

	/*
	 * The Supported Operating Classes element is used by a STA to
	 * advertise the operating classes that it is capable of operating
	 * with in this country.
	 * The Country element (see 8.4.2.10) allows a STA to configure its
	 * PHY and MAC for operation when the operating triplet of Operating
	 * Extension Identifier, Operating Class, and Coverage Class fields
	 * is present.
	 */
	SUP_OPERATING_CLASS_IE(pBuf)->ucId = ELEM_ID_SUP_OPERATING_CLASS;
	SUP_OPERATING_CLASS_IE(pBuf)->ucLength = 1 + sizeof(aucClass);
	SUP_OPERATING_CLASS_IE(pBuf)->ucCur = 0x0c;	/* 0x51 */
	kalMemCopy(SUP_OPERATING_CLASS_IE(pBuf)->ucSup, aucClass,
		   sizeof(aucClass));
	u4IeLen = (SUP_OPERATING_CLASS_IE(pBuf)->ucLength + 2);
	pBuf += u4IeLen;

	COUNTRY_IE(pBuf)->ucId = ELEM_ID_COUNTRY_INFO;
	COUNTRY_IE(pBuf)->ucLength = 6;
	COUNTRY_IE(pBuf)->aucCountryStr[0] = 0x55;
	COUNTRY_IE(pBuf)->aucCountryStr[1] = 0x53;
	COUNTRY_IE(pBuf)->aucCountryStr[2] = 0x20;
	COUNTRY_IE(pBuf)->arCountryStr[0].ucFirstChnlNum = 1;
	COUNTRY_IE(pBuf)->arCountryStr[0].ucNumOfChnl = 11;
	COUNTRY_IE(pBuf)->arCountryStr[0].cMaxTxPwrLv = 0x1e;
	u4IeLen += (COUNTRY_IE(pBuf)->ucLength + 2);

	return u4IeLen;
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief
 *
 * @param[in]
 *
 * @return (fgValid) : 0 -> inValid, 1 -> Valid
 */
/*----------------------------------------------------------------------------*/
u_int8_t rlmDomainCheckChannelEntryValid(struct ADAPTER *prAdapter,
				enum ENUM_BAND eBand, uint8_t ucCentralCh)
{
	u_int8_t fgValid = FALSE;
	uint8_t ucTemp = 0xff;
	uint8_t i;
	/*Check Power limit table channel efficient or not */

	/* CH50 is not located in any FCC subbands
	 * but it's a valid central channel for 160C
	 */
	if (eBand == BAND_5G && ucCentralCh == 50) {
		fgValid = TRUE;
		return fgValid;
	}

	for (i = POWER_LIMIT_2G4; i < POWER_LIMIT_SUBAND_NUM; i++) {
		if ((eBand == BAND_NULL || eBand == g_rRlmSubBand[i].eBand) &&
			(ucCentralCh >= g_rRlmSubBand[i].ucStartCh) &&
			(ucCentralCh <= g_rRlmSubBand[i].ucEndCh))
			ucTemp = (ucCentralCh - g_rRlmSubBand[i].ucStartCh) %
				 g_rRlmSubBand[i].ucInterval;
		if (ucTemp == 0)
			break;
	}

	if (ucTemp == 0)
		fgValid = TRUE;
	return fgValid;

}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in]
 *
 * \return
 */
/*----------------------------------------------------------------------------*/
uint8_t rlmDomainGetCenterChannel(enum ENUM_BAND eBand, uint8_t ucPriChannel,
				  enum ENUM_CHNL_EXT eExtend)
{
	uint8_t ucCenterChannel;

	if (eExtend == CHNL_EXT_SCA)
		ucCenterChannel = ucPriChannel + 2;
	else if (eExtend == CHNL_EXT_SCB)
		ucCenterChannel = ucPriChannel - 2;
	else
		ucCenterChannel = ucPriChannel;

	return ucCenterChannel;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *
 * \param[in]
 *
 * \return
 */
/*----------------------------------------------------------------------------*/
u_int8_t
rlmDomainIsValidRfSetting(struct ADAPTER *prAdapter,
			  enum ENUM_BAND eBand,
			  uint8_t ucPriChannel,
			  enum ENUM_CHNL_EXT eExtend,
			  enum ENUM_CHANNEL_WIDTH eChannelWidth,
			  uint8_t ucChannelS1, uint8_t ucChannelS2)
{
	uint8_t ucCenterCh = 0;
	uint8_t  ucUpperChannel;
	uint8_t  ucLowerChannel;
	u_int8_t fgValidChannel = TRUE;
	u_int8_t fgUpperChannel = TRUE;
	u_int8_t fgLowerChannel = TRUE;
	u_int8_t fgValidBW = TRUE;
	u_int8_t fgValidRfSetting = TRUE;
	uint32_t u4PrimaryOffset;

	/*DBG msg for Channel InValid */
	if (eChannelWidth == CW_20_40MHZ) {
		ucCenterCh = rlmDomainGetCenterChannel(eBand, ucPriChannel,
						       eExtend);

		/* Check Central Channel Valid or Not */
		fgValidChannel = rlmDomainCheckChannelEntryValid(prAdapter,
			eBand, ucCenterCh);
		if (fgValidChannel == FALSE)
			DBGLOG(RLM, WARN, "Rf20: CentralCh=%d\n", ucCenterCh);

		/* Check Upper Channel and Lower Channel */
		switch (eExtend) {
		case CHNL_EXT_SCA:
			ucUpperChannel = ucPriChannel + 4;
			ucLowerChannel = ucPriChannel;
			break;
		case CHNL_EXT_SCB:
			ucUpperChannel = ucPriChannel;
			ucLowerChannel = ucPriChannel - 4;
			break;
		default:
			ucUpperChannel = ucPriChannel;
			ucLowerChannel = ucPriChannel;
			break;
		}

		fgUpperChannel = rlmDomainCheckChannelEntryValid(prAdapter,
			eBand, ucUpperChannel);
		if (fgUpperChannel == FALSE)
			DBGLOG(RLM, WARN, "Rf20: UpperCh=%d\n", ucUpperChannel);

		fgLowerChannel = rlmDomainCheckChannelEntryValid(prAdapter,
			eBand, ucLowerChannel);
		if (fgLowerChannel == FALSE)
			DBGLOG(RLM, WARN, "Rf20: LowerCh=%d\n", ucLowerChannel);

		/* Check S1, S2 */
		if (ucChannelS2 != 0) {
			fgValidChannel = FALSE;
			DBGLOG(RLM, WARN, "Rf20: S1=%d, S2=%d\n",
				ucChannelS1, ucChannelS2);
		}
	} else if ((eChannelWidth == CW_80MHZ) ||
		   (eChannelWidth == CW_160MHZ)) {
		ucCenterCh = ucChannelS1;

		/* Check Central Channel Valid or Not */
		if (eChannelWidth != CW_160MHZ) {
			/* BW not check , ex: primary 36 and
			 * central channel 50 will fail the check
			 */
			fgValidChannel =
				rlmDomainCheckChannelEntryValid(prAdapter,
					eBand, ucCenterCh);
		}

		if (fgValidChannel == FALSE)
			DBGLOG(RLM, WARN, "Rf80/160C: CentralCh=%d\n",
			       ucCenterCh);
	} else if (eChannelWidth == CW_80P80MHZ) {
		ucCenterCh = ucChannelS1;

		fgValidChannel = rlmDomainCheckChannelEntryValid(prAdapter,
			eBand, ucCenterCh);

		if (fgValidChannel == FALSE)
			DBGLOG(RLM, WARN, "Rf160NC: CentralCh1=%d\n",
			       ucCenterCh);

		ucCenterCh = ucChannelS2;

		fgValidChannel = rlmDomainCheckChannelEntryValid(prAdapter,
			eBand, ucCenterCh);

		if (fgValidChannel == FALSE)
			DBGLOG(RLM, WARN, "Rf160NC: CentralCh2=%d\n",
			       ucCenterCh);

		/* Check Central Channel Valid or Not */
	} else {
		DBGLOG(RLM, ERROR, "Wrong BW =%d\n", eChannelWidth);
		fgValidChannel = FALSE;
	}

	/* Check BW Setting Correct or Not */
	if (eBand == BAND_2G4) {
		if (eChannelWidth != CW_20_40MHZ) {
			fgValidBW = FALSE;
			DBGLOG(RLM, WARN, "Rf: B=%d, W=%d\n",
			       eBand, eChannelWidth);
		}
	} else {
		if ((eChannelWidth == CW_80MHZ) ||
				(eChannelWidth == CW_80P80MHZ)) {
			u4PrimaryOffset = CAL_CH_OFFSET_80M(ucPriChannel,
							    ucChannelS1);
			if (u4PrimaryOffset > 4) {
				fgValidBW = FALSE;
				DBGLOG(RLM, WARN, "Rf: PriOffSet=%d, W=%d\n",
				       u4PrimaryOffset, eChannelWidth);
			}
			if (ucPriChannel == 165 && eBand == BAND_5G) {
				fgValidBW = FALSE;
				DBGLOG(RLM, WARN,
				       "Rf: PriOffSet=%d, W=%d C=%d\n",
				       u4PrimaryOffset, eChannelWidth,
				       ucPriChannel);
			}
		} else if (eChannelWidth == CW_160MHZ) {
			u4PrimaryOffset = CAL_CH_OFFSET_160M(ucPriChannel,
							     ucCenterCh);
			if (u4PrimaryOffset > 8) {
				fgValidBW = FALSE;
				DBGLOG(RLM, WARN,
				       "Rf: PriOffSet=%d, W=%d\n",
				       u4PrimaryOffset, eChannelWidth);
			}
		}
	}

	if ((fgValidBW == FALSE) || (fgValidChannel == FALSE) ||
	    (fgUpperChannel == FALSE) || (fgLowerChannel == FALSE))
		fgValidRfSetting = FALSE;

	return fgValidRfSetting;

}

#if (CFG_SUPPORT_SINGLE_SKU == 1)
/*
 * This function coverts country code from alphabet chars to u32,
 * the caller need to pass country code chars and do size check
 */
uint32_t rlmDomainAlpha2ToU32(uint8_t *pcAlpha2, uint8_t ucAlpha2Size)
{
	uint8_t ucIdx;
	uint32_t u4CountryCode = 0;

	if (ucAlpha2Size > TX_PWR_LIMIT_COUNTRY_STR_MAX_LEN) {
		DBGLOG(RLM, ERROR, "alpha2 size %d is invalid!(max: %d)\n",
			ucAlpha2Size, TX_PWR_LIMIT_COUNTRY_STR_MAX_LEN);
		ucAlpha2Size = TX_PWR_LIMIT_COUNTRY_STR_MAX_LEN;
	}

	for (ucIdx = 0; ucIdx < ucAlpha2Size; ucIdx++)
		u4CountryCode |= (pcAlpha2[ucIdx] << (ucIdx * 8));

	return u4CountryCode;
}

uint8_t rlmDomainTxPwrLimitGetTableVersion(
	uint8_t *pucBuf, uint32_t u4BufLen)
{
#define TX_PWR_LIMIT_VERSION_STR_LEN 7
#define TX_PWR_LIMIT_MAX_VERSION 1
	uint32_t u4TmpPos = 0;
	uint8_t ucVersion = 0;

	while (u4TmpPos < u4BufLen && pucBuf[u4TmpPos] != '<')
		u4TmpPos++;

	if (u4TmpPos >= (u4BufLen - TX_PWR_LIMIT_VERSION_STR_LEN))
		return ucVersion;

	if (kalStrnCmp(&pucBuf[u4TmpPos + 1], "Ver:", 4) == 0) {
		ucVersion = (pucBuf[u4TmpPos + 5] - '0') * 10 +
			(pucBuf[u4TmpPos + 6] - '0');
	}

	if (ucVersion > TX_PWR_LIMIT_MAX_VERSION)
		ucVersion = 0;

	return ucVersion;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Search the tx power limit setting range of the specified in the text
 *        file
 *
 * \param[IN] u4CountryCode The u32 type of the specified country.
 * \param[IN] pucBuf The content of the text file.
 * \param[IN] u4cBufLen End boundary of the text file.
 * \param[OUT] pu4CountryStart Store the start position of the desired country
 *             settings.
 * \param[OUT] pu4CountryEnd Store the end position of the desired country
 *             settings.
 *
 * \retval TRUE Success.
 * \retval FALSE Failure.
 */
/*----------------------------------------------------------------------------*/
u_int8_t rlmDomainTxPwrLimitGetCountryRange(
	uint32_t u4CountryCode, uint8_t *pucBuf, uint32_t u4BufLen,
	uint32_t *pu4CountryStart, uint32_t *pu4CountryEnd)
{
	uint32_t u4TmpPos = 0;
	char pcrCountryStr[TX_PWR_LIMIT_COUNTRY_STR_MAX_LEN + 1] = {0};
	uint8_t cIdx = 0;

	while (1) {
		while (u4TmpPos < u4BufLen && pucBuf[u4TmpPos] != '[')
			u4TmpPos++;

		u4TmpPos++; /* skip the '[' char */

		cIdx = 0;
		while ((u4TmpPos < u4BufLen) &&
			   (cIdx < TX_PWR_LIMIT_COUNTRY_STR_MAX_LEN) &&
			   (pucBuf[u4TmpPos] != ']')) {
			pcrCountryStr[cIdx++] = pucBuf[u4TmpPos];
			u4TmpPos++;
		}

		u4TmpPos++; /* skip the ']' char */

		if ((u4TmpPos >= u4BufLen) ||
		    (cIdx > TX_PWR_LIMIT_COUNTRY_STR_MAX_LEN))
			return FALSE;

		if (u4CountryCode ==
			rlmDomainAlpha2ToU32(pcrCountryStr, cIdx)) {
			DBGLOG(RLM, INFO,
				"Found TxPwrLimit table for CountryCode \"%s\"\n",
				pcrCountryStr);
			*pu4CountryStart = u4TmpPos;
			/* the location after char ']' */
			break;
		}
	}

	while (u4TmpPos < u4BufLen && pucBuf[u4TmpPos] != '[')
		u4TmpPos++;

	*pu4CountryEnd = u4TmpPos;

	return TRUE;
}

u_int8_t rlmDomainTxPwrLimitSearchSection(const char *pSectionName,
	uint8_t *pucBuf, uint32_t *pu4Pos, uint32_t u4BufEnd)
{
	uint32_t u4TmpPos = *pu4Pos;
	uint8_t uSectionNameLen = kalStrLen(pSectionName);

	while (1) {
		while (u4TmpPos < u4BufEnd && pucBuf[u4TmpPos] != '<')
			u4TmpPos++;

		u4TmpPos++; /* skip char '<' */

		if (u4TmpPos + uSectionNameLen >= u4BufEnd)
			return FALSE;

		if (kalStrnCmp(&pucBuf[u4TmpPos],
				pSectionName, uSectionNameLen) == 0) {

			/* Go to the end of section header line */
			while ((u4TmpPos < u4BufEnd) &&
				   (pucBuf[u4TmpPos] != '\n'))
				u4TmpPos++;

			*pu4Pos = u4TmpPos;

			break;
		}
	}

	return TRUE;
}

u_int8_t rlmDomainTxPwrLimitSectionEnd(uint8_t *pucBuf,
	const char *pSectionName, uint32_t *pu4Pos, uint32_t u4BufEnd)
{
	uint32_t u4TmpPos = *pu4Pos;
	char cTmpChar = 0;
	uint8_t uSectionNameLen = kalStrLen(pSectionName);

	while (u4TmpPos < u4BufEnd) {
		cTmpChar = pucBuf[u4TmpPos];

		/* skip blank lines */
		if (cTmpChar == ' ' || cTmpChar == '\t' ||
			cTmpChar == '\n' || cTmpChar == '\r') {
			u4TmpPos++;
			continue;
		}

		break;
	}

	/* 2 means '/' and '>' */
	if (u4TmpPos + uSectionNameLen + 2 >= u4BufEnd) {
		*pu4Pos = u4BufEnd;
		return FALSE;
	}

	if (pucBuf[u4TmpPos] != '<')
		return FALSE;

	if (pucBuf[u4TmpPos + 1] != '/' ||
		pucBuf[u4TmpPos + 2 + uSectionNameLen] != '>' ||
		kalStrnCmp(&pucBuf[u4TmpPos + 2],
			pSectionName, uSectionNameLen)) {

		*pu4Pos = u4TmpPos + uSectionNameLen + 2;
		return FALSE;
	}

	/* 3 means go to the location after '>' */
	*pu4Pos = u4TmpPos + uSectionNameLen + 3;
	return TRUE;
}

int8_t rlmDomainTxPwrLimitGetChIdx(
	struct TX_PWR_LIMIT_DATA *pTxPwrLimit, uint8_t ucChannel)
{
	int8_t cIdx = 0;

	for (cIdx = 0; cIdx < pTxPwrLimit->ucChNum; cIdx++)
		if (ucChannel ==
			pTxPwrLimit->rChannelTxPwrLimit[cIdx].ucChannel)
			return cIdx;

	DBGLOG(RLM, ERROR,
		"Can't find idx of channel %d in TxPwrLimit data\n",
		ucChannel);

	return -1;
}

u_int8_t rlmDomainTxPwrLimitIsTxBfBackoffSection(
	uint8_t ucVersion, uint8_t ucSectionIdx)
{
	if (ucVersion == 1 && ucSectionIdx == 8)
		return TRUE;

	return FALSE;
}
u_int8_t rlmDomainTxPwrLimitLoadChannelSetting(
	uint8_t ucVersion, uint8_t *pucBuf, uint32_t *pu4Pos, uint32_t u4BufEnd,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimit, uint8_t ucSectionIdx)
{
	uint32_t u4TmpPos = *pu4Pos;
	char cTmpChar = 0;
	struct CHANNEL_TX_PWR_LIMIT *prChTxPwrLimit = NULL;
	u_int8_t bNeg = FALSE;
	int8_t cLimitValue = 0, cChIdx = 0;
	uint8_t ucIdx = 0, ucChannel = 0;
	uint8_t ucElementNum =
		gTx_Pwr_Limit_Element_Num[ucVersion][ucSectionIdx];

	/* skip blank lines */
	while (u4TmpPos < u4BufEnd) {
		cTmpChar = pucBuf[u4TmpPos];

		if (cTmpChar == ' ' || cTmpChar == '\t' ||
			cTmpChar == '\n' || cTmpChar == '\r') {
			u4TmpPos++;
			continue;
		}

		break;
	}

	/* current is at the location of 'c',
	 * check remaining buf length for 'chxxx'
	 */
	if (u4TmpPos + 5 >= u4BufEnd) {
		DBGLOG(RLM, ERROR,
			"Invalid location of ch setting: %u/%u\n",
			u4TmpPos, u4BufEnd);
		return FALSE;
	}

	if (pucBuf[u4TmpPos] == 'c' && pucBuf[u4TmpPos + 1] == 'h') {
		ucChannel = (pucBuf[u4TmpPos + 2] - '0') * 100 +
					(pucBuf[u4TmpPos + 3] - '0') * 10 +
					(pucBuf[u4TmpPos + 4] - '0');
	} else { /* invalid format */
		*pu4Pos = u4TmpPos;
		DBGLOG(RLM, ERROR,
			"Invalid ch setting starting chars: %c%c\n",
			pucBuf[u4TmpPos], pucBuf[u4TmpPos + 1]);

		/* goto next line */
		while (*pu4Pos < u4BufEnd && pucBuf[*pu4Pos] != '\n')
			(*pu4Pos)++;

		return TRUE;
	}

	cChIdx = rlmDomainTxPwrLimitGetChIdx(pTxPwrLimit, ucChannel);

	if (cChIdx == -1) {
		*pu4Pos = u4TmpPos;
		DBGLOG(RLM, ERROR, "Invalid ch %u %c%c%c\n", ucChannel,
			pucBuf[u4TmpPos + 2],
			pucBuf[u4TmpPos + 3], pucBuf[u4TmpPos + 4]);

		/* goto next line */
		while (*pu4Pos < u4BufEnd && pucBuf[*pu4Pos] != '\n')
			(*pu4Pos)++;

		return TRUE;
	}

	u4TmpPos += 5;

	prChTxPwrLimit = &pTxPwrLimit->rChannelTxPwrLimit[cChIdx];

	/* read the channel TxPwrLimit settings */
	for (ucIdx = 0; ucIdx < ucElementNum; ucIdx++) {

		/* skip blank and comma */
		while (u4TmpPos < u4BufEnd) {
			cTmpChar = pucBuf[u4TmpPos];

			if ((cTmpChar == ' ') ||
				(cTmpChar == '\t') ||
				(cTmpChar == ',')) {
				u4TmpPos++;
				continue;
			}
			break;
		}

		if (u4TmpPos >= u4BufEnd) {
			*pu4Pos = u4BufEnd;
			DBGLOG(RLM, ERROR,
				"Invalid location of ch tx pwr limit val: %u/%u\n",
				u4TmpPos, u4BufEnd);
			return FALSE;
		}

		bNeg = FALSE;

		cTmpChar = pucBuf[u4TmpPos];

		if (cTmpChar == '-') {
			bNeg = TRUE;
			u4TmpPos++;
		} else {
			if (cTmpChar == 'x') {
				if (!rlmDomainTxPwrLimitIsTxBfBackoffSection(
					ucVersion, ucSectionIdx)) {
					prChTxPwrLimit->
						rTxPwrLimitValue
						[ucSectionIdx][ucIdx] =
						TX_PWR_LIMIT_MAX_VAL;
				} else {
					prChTxPwrLimit->rTxBfBackoff[ucIdx] =
						TX_PWR_LIMIT_MAX_VAL;
				}
				u4TmpPos++;
				continue;
			}
		}

		cLimitValue = 0;
		while (u4TmpPos < u4BufEnd) {
			cTmpChar = pucBuf[u4TmpPos];

			if (cTmpChar < '0' || cTmpChar > '9')
				break;

			cLimitValue = (cLimitValue * 10) + (cTmpChar - '0');
			u4TmpPos++;
		}

		if (bNeg)
			cLimitValue = -cLimitValue;
		if (!rlmDomainTxPwrLimitIsTxBfBackoffSection(
			ucVersion, ucSectionIdx)) {
			prChTxPwrLimit->rTxPwrLimitValue[ucSectionIdx][ucIdx] =
				cLimitValue;
		} else {
			prChTxPwrLimit->rTxBfBackoff[ucIdx] =
				cLimitValue;
		}
	}

	*pu4Pos = u4TmpPos;
	return TRUE;
}

void rlmDomainTxPwrLimitRemoveComments(
	uint8_t *pucBuf, uint32_t u4BufLen)
{
	uint32_t u4TmpPos = 0;
	char cTmpChar = 0;

	while (u4TmpPos < u4BufLen) {
		cTmpChar = pucBuf[u4TmpPos];

		if (cTmpChar == '#') {
			while (cTmpChar != '\n') {
				pucBuf[u4TmpPos] = ' ';

				u4TmpPos++;
				if (u4TmpPos >= u4BufLen)
					break;

				cTmpChar = pucBuf[u4TmpPos];
			}
		}
		u4TmpPos++;
	}
}

u_int8_t rlmDomainTxPwrLimitLoad(
	struct ADAPTER *prAdapter, uint8_t *pucBuf, uint32_t u4BufLen,
	uint8_t ucVersion, uint32_t u4CountryCode,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimitData)
{
	uint8_t uSecIdx = 0;
	uint8_t ucSecNum = gTx_Pwr_Limit_Section[ucVersion].ucSectionNum;
	uint32_t u4CountryStart = 0, u4CountryEnd = 0, u4Pos = 0;
	struct TX_PWR_LIMIT_SECTION *prSection =
		&gTx_Pwr_Limit_Section[ucVersion];
	uint8_t *prFileName = prAdapter->chip_info->prTxPwrLimitFile;


	if (!rlmDomainTxPwrLimitGetCountryRange(u4CountryCode, pucBuf,
		u4BufLen, &u4CountryStart, &u4CountryEnd)) {
		DBGLOG(RLM, ERROR, "Can't find specified table in %s\n",
			prFileName);
		return FALSE;
	}
	u4Pos = u4CountryStart;

	for (uSecIdx = 0; uSecIdx < ucSecNum; uSecIdx++) {
		const uint8_t *pSecName =
			prSection->arSectionNames[uSecIdx];
		if (!rlmDomainTxPwrLimitSearchSection(
				pSecName, pucBuf, &u4Pos,
				u4CountryEnd)) {
			DBGLOG(RLM, ERROR,
				"Can't find specified section %s in %s\n",
				pSecName,
				prFileName);
			continue;
		}

		DBGLOG(RLM, INFO, "Find specified section %s in %s\n",
			pSecName,
			prFileName);

		while (!rlmDomainTxPwrLimitSectionEnd(pucBuf,
			pSecName,
			&u4Pos, u4CountryEnd) &&
			u4Pos < u4CountryEnd) {
			if (!rlmDomainTxPwrLimitLoadChannelSetting(
				ucVersion, pucBuf, &u4Pos, u4CountryEnd,
				pTxPwrLimitData, uSecIdx))
				return FALSE;
			if (rlmDomainTxPwrLimitIsTxBfBackoffSection(
				ucVersion, uSecIdx))
				g_bTxBfBackoffExists = TRUE;
		}
	}

	DBGLOG(RLM, INFO, "Load %s finished\n", prFileName);
	return TRUE;
}

void rlmDomainTxPwrLimitSetChValues(
	uint8_t ucVersion,
	struct CMD_CHANNEL_POWER_LIMIT_V2 *pCmd,
	struct CHANNEL_TX_PWR_LIMIT *pChTxPwrLimit)
{
	uint8_t section = 0, e = 0;
	uint8_t ucElementNum = 0;

	pCmd->tx_pwr_dsss_cck = pChTxPwrLimit->rTxPwrLimitValue[0][0];
	pCmd->tx_pwr_dsss_bpsk = pChTxPwrLimit->rTxPwrLimitValue[0][1];

	/* 6M, 9M */
	pCmd->tx_pwr_ofdm_bpsk = pChTxPwrLimit->rTxPwrLimitValue[0][2];
	/* 12M, 18M */
	pCmd->tx_pwr_ofdm_qpsk = pChTxPwrLimit->rTxPwrLimitValue[0][3];
	/* 24M, 36M */
	pCmd->tx_pwr_ofdm_16qam = pChTxPwrLimit->rTxPwrLimitValue[0][4];
	pCmd->tx_pwr_ofdm_48m = pChTxPwrLimit->rTxPwrLimitValue[0][5];
	pCmd->tx_pwr_ofdm_54m = pChTxPwrLimit->rTxPwrLimitValue[0][6];

	/* MCS0*/
	pCmd->tx_pwr_ht20_bpsk = pChTxPwrLimit->rTxPwrLimitValue[1][0];
	/* MCS1, MCS2*/
	pCmd->tx_pwr_ht20_qpsk = pChTxPwrLimit->rTxPwrLimitValue[1][1];
	/* MCS3, MCS4*/
	pCmd->tx_pwr_ht20_16qam = pChTxPwrLimit->rTxPwrLimitValue[1][2];
	/* MCS5*/
	pCmd->tx_pwr_ht20_mcs5 = pChTxPwrLimit->rTxPwrLimitValue[1][3];
	/* MCS6*/
	pCmd->tx_pwr_ht20_mcs6 = pChTxPwrLimit->rTxPwrLimitValue[1][4];
	/* MCS7*/
	pCmd->tx_pwr_ht20_mcs7 = pChTxPwrLimit->rTxPwrLimitValue[1][5];

	/* MCS0*/
	pCmd->tx_pwr_ht40_bpsk = pChTxPwrLimit->rTxPwrLimitValue[2][0];
	/* MCS1, MCS2*/
	pCmd->tx_pwr_ht40_qpsk = pChTxPwrLimit->rTxPwrLimitValue[2][1];
	/* MCS3, MCS4*/
	pCmd->tx_pwr_ht40_16qam = pChTxPwrLimit->rTxPwrLimitValue[2][2];
	/* MCS5*/
	pCmd->tx_pwr_ht40_mcs5 = pChTxPwrLimit->rTxPwrLimitValue[2][3];
	/* MCS6*/
	pCmd->tx_pwr_ht40_mcs6 = pChTxPwrLimit->rTxPwrLimitValue[2][4];
	/* MCS7*/
	pCmd->tx_pwr_ht40_mcs7 = pChTxPwrLimit->rTxPwrLimitValue[2][5];
	/* MCS32*/
	pCmd->tx_pwr_ht40_mcs32 = pChTxPwrLimit->rTxPwrLimitValue[2][6];

	/* MCS0*/
	pCmd->tx_pwr_vht20_bpsk = pChTxPwrLimit->rTxPwrLimitValue[3][0];
	/* MCS1, MCS2*/
	pCmd->tx_pwr_vht20_qpsk = pChTxPwrLimit->rTxPwrLimitValue[3][1];
	/* MCS3, MCS4*/
	pCmd->tx_pwr_vht20_16qam = pChTxPwrLimit->rTxPwrLimitValue[3][2];
	/* MCS5, MCS6*/
	pCmd->tx_pwr_vht20_64qam = pChTxPwrLimit->rTxPwrLimitValue[3][3];
	pCmd->tx_pwr_vht20_mcs7 = pChTxPwrLimit->rTxPwrLimitValue[3][4];
	pCmd->tx_pwr_vht20_mcs8 = pChTxPwrLimit->rTxPwrLimitValue[3][5];
	pCmd->tx_pwr_vht20_mcs9 = pChTxPwrLimit->rTxPwrLimitValue[3][6];

	pCmd->tx_pwr_vht_40 = pChTxPwrLimit->rTxPwrLimitValue[4][2];
	pCmd->tx_pwr_vht_80 = pChTxPwrLimit->rTxPwrLimitValue[4][3];
	pCmd->tx_pwr_vht_160c = pChTxPwrLimit->rTxPwrLimitValue[4][5];
	pCmd->tx_pwr_vht_160nc = pChTxPwrLimit->rTxPwrLimitValue[4][4];
	pCmd->tx_pwr_lg_40 = pChTxPwrLimit->rTxPwrLimitValue[4][0];
	pCmd->tx_pwr_lg_80 = pChTxPwrLimit->rTxPwrLimitValue[4][1];


	DBGLOG(RLM, TRACE, "ch %d\n", pCmd->ucCentralCh);
	for (section = 0; section < TX_PWR_LIMIT_SECTION_NUM; section++) {
		struct TX_PWR_LIMIT_SECTION *pSection =
			&gTx_Pwr_Limit_Section[ucVersion];
		ucElementNum = gTx_Pwr_Limit_Element_Num[ucVersion][section];
		for (e = 0; e < ucElementNum; e++)
			DBGLOG(RLM, TRACE, "TxPwrLimit[%s][%s]= %d\n",
				pSection->arSectionNames[section],
				gTx_Pwr_Limit_Element[ucVersion][section][e],
				pChTxPwrLimit->rTxPwrLimitValue[section][e]);
	}
}

void rlmDomainTxPwrLimitPerRateSetChValues(
	uint8_t ucVersion,
	struct CMD_TXPOWER_CHANNEL_POWER_LIMIT_PER_RATE *pCmd,
	struct CHANNEL_TX_PWR_LIMIT *pChTxPwrLimit)
{
	uint8_t section = 0, e = 0, count = 0;
	uint8_t ucElementNum = 0;

	for (section = 0; section < TX_PWR_LIMIT_SECTION_NUM; section++) {
		if (rlmDomainTxPwrLimitIsTxBfBackoffSection(ucVersion, section))
			continue;
		ucElementNum = gTx_Pwr_Limit_Element_Num[ucVersion][section];
		for (e = 0; e < ucElementNum; e++) {
			pCmd->aucTxPwrLimit.i1PwrLimit[count] =
				pChTxPwrLimit->rTxPwrLimitValue[section][e];
			count++;
		}
	}

	DBGLOG(RLM, TRACE, "ch %d\n", pCmd->u1CentralCh);
	count = 0;
	for (section = 0; section < TX_PWR_LIMIT_SECTION_NUM; section++) {
		struct TX_PWR_LIMIT_SECTION *pSection =
			&gTx_Pwr_Limit_Section[ucVersion];
		if (rlmDomainTxPwrLimitIsTxBfBackoffSection(ucVersion, section))
			continue;
		ucElementNum = gTx_Pwr_Limit_Element_Num[ucVersion][section];
		for (e = 0; e < ucElementNum; e++) {
			DBGLOG(RLM, TRACE, "TxPwrLimit[%s][%s]= %d\n",
				pSection->arSectionNames[section],
				gTx_Pwr_Limit_Element[ucVersion][section][e],
				pCmd->aucTxPwrLimit.i1PwrLimit[count]);
			count++;
		}
	}
}

void rlmDomainTxPwrLimitSetValues(
	uint8_t ucVersion,
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT_V2 *pSetCmd,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimit)
{
	uint8_t ucIdx = 0;
	int8_t cChIdx = 0;
	struct CMD_CHANNEL_POWER_LIMIT_V2 *pCmd = NULL;
	struct CHANNEL_TX_PWR_LIMIT *pChTxPwrLimit = NULL;

	if (!pSetCmd || !pTxPwrLimit) {
		DBGLOG(RLM, ERROR, "Invalid TxPwrLimit request\n");
		return;
	}

	for (ucIdx = 0; ucIdx < pSetCmd->ucNum; ucIdx++) {
		pCmd = &(pSetCmd->rChannelPowerLimit[ucIdx]);
		cChIdx = rlmDomainTxPwrLimitGetChIdx(pTxPwrLimit,
			pCmd->ucCentralCh);
		if (cChIdx == -1) {
			DBGLOG(RLM, ERROR,
				"Invalid ch idx found while assigning values\n");
			continue;
		}
		pChTxPwrLimit = &pTxPwrLimit->rChannelTxPwrLimit[cChIdx];
		rlmDomainTxPwrLimitSetChValues(ucVersion, pCmd, pChTxPwrLimit);
	}
}

void rlmDomainTxPwrLimitPerRateSetValues(
	uint8_t ucVersion,
	struct CMD_SET_TXPOWER_COUNTRY_TX_POWER_LIMIT_PER_RATE *pSetCmd,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimit)
{
	uint8_t ucIdx = 0;
	int8_t cChIdx = 0;
	struct CMD_TXPOWER_CHANNEL_POWER_LIMIT_PER_RATE *pChPwrLimit = NULL;
	struct CHANNEL_TX_PWR_LIMIT *pChTxPwrLimit = NULL;

	if (pSetCmd == NULL)
		return;

	for (ucIdx = 0; ucIdx < pSetCmd->ucNum; ucIdx++) {
		pChPwrLimit = &(pSetCmd->rChannelPowerLimit[ucIdx]);
		cChIdx = rlmDomainTxPwrLimitGetChIdx(pTxPwrLimit,
			pChPwrLimit->u1CentralCh);

		if (cChIdx == -1) {
			DBGLOG(RLM, ERROR,
				"Invalid ch idx found while assigning values\n");
			continue;
		}
		pChTxPwrLimit = &pTxPwrLimit->rChannelTxPwrLimit[cChIdx];
		rlmDomainTxPwrLimitPerRateSetChValues(ucVersion,
			pChPwrLimit, pChTxPwrLimit);
	}
}

u_int8_t rlmDomainTxPwrLimitLoadFromFile(
	struct ADAPTER *prAdapter,
	uint8_t *pucConfigBuf, uint32_t *pu4ConfigReadLen)
{
#define TXPWRLIMIT_FILE_LEN 64
	u_int8_t bRet = TRUE;
	uint8_t *prFileName = prAdapter->chip_info->prTxPwrLimitFile;
	uint8_t aucPath[TXPWRLIMIT_FILE_LEN];

	kalMemZero(aucPath, sizeof(aucPath));
	kalSnprintf(aucPath, TXPWRLIMIT_FILE_LEN, "%s", prFileName);

	kalMemZero(pucConfigBuf, WLAN_TX_PWR_LIMIT_FILE_BUF_SIZE);
	*pu4ConfigReadLen = 0;

	if (wlanGetFileContent(
			prAdapter,
			aucPath,
			pucConfigBuf,
			WLAN_TX_PWR_LIMIT_FILE_BUF_SIZE,
			pu4ConfigReadLen) == 0) {
		/* ToDo:: Nothing */
	} else {
		bRet = FALSE;
		goto error;
	}

	if (pucConfigBuf[0] == '\0' || *pu4ConfigReadLen == 0) {
		bRet = FALSE;
		goto error;
	}

error:

	return bRet;
}

u_int8_t rlmDomainGetTxPwrLimit(
	uint32_t country_code,
	uint8_t *pucVersion,
	struct GLUE_INFO *prGlueInfo,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimitData)
{
	u_int8_t bRet = TRUE;
	uint8_t *pucConfigBuf = NULL;
	uint32_t u4ConfigReadLen = 0;

	pucConfigBuf = (uint8_t *) kalMemAlloc(
		WLAN_TX_PWR_LIMIT_FILE_BUF_SIZE, VIR_MEM_TYPE);

	if (!pucConfigBuf)
		return FALSE;

	bRet = rlmDomainTxPwrLimitLoadFromFile(prGlueInfo->prAdapter,
		pucConfigBuf, &u4ConfigReadLen);

	rlmDomainTxPwrLimitRemoveComments(pucConfigBuf, u4ConfigReadLen);
	*pucVersion = rlmDomainTxPwrLimitGetTableVersion(pucConfigBuf,
		u4ConfigReadLen);

	if (!rlmDomainTxPwrLimitLoad(prGlueInfo->prAdapter,
		pucConfigBuf, u4ConfigReadLen, *pucVersion,
		country_code, pTxPwrLimitData)) {
		bRet = FALSE;
		goto error;
	}

error:

	kalMemFree(pucConfigBuf,
		VIR_MEM_TYPE, WLAN_TX_PWR_LIMIT_FILE_BUF_SIZE);

	return bRet;
}

#endif

#if CFG_SUPPORT_PWR_LIMIT_COUNTRY

/*----------------------------------------------------------------------------*/
/*!
 * @brief Check if power limit setting is in the range [MIN_TX_POWER,
 *        MAX_TX_POWER]
 *
 * @param[in]
 *
 * @return (fgValid) : 0 -> inValid, 1 -> Valid
 */
/*----------------------------------------------------------------------------*/
u_int8_t
rlmDomainCheckPowerLimitValid(struct ADAPTER *prAdapter,
			      struct COUNTRY_POWER_LIMIT_TABLE_CONFIGURATION
						rPowerLimitTableConfiguration,
			      uint8_t ucPwrLimitNum)
{
	uint16_t i;
	u_int8_t fgValid = TRUE;
	int8_t *prPwrLimit;

	prPwrLimit = &rPowerLimitTableConfiguration.aucPwrLimit[0];

	for (i = 0; i < ucPwrLimitNum; i++, prPwrLimit++) {
		if (*prPwrLimit > MAX_TX_POWER || *prPwrLimit < MIN_TX_POWER) {
			fgValid = FALSE;
			break;	/*Find out Wrong Power limit */
		}
	}
	return fgValid;

}

/*----------------------------------------------------------------------------*/
/*!
 * @brief 1.Check if power limit configuration table valid(channel intervel)
 *	2.Check if power limit configuration/default table entry repeat
 *
 * @param[in]
 *
 * @return (none)
 */
/*----------------------------------------------------------------------------*/
void rlmDomainCheckCountryPowerLimitTable(struct ADAPTER *prAdapter)
{
#define PwrLmtConf g_rRlmPowerLimitConfiguration
	uint16_t i, j;
	uint16_t u2CountryCodeTable, u2CountryCodeCheck;
	u_int8_t fgChannelValid = FALSE;
	u_int8_t fgPowerLimitValid = FALSE;
	u_int8_t fgEntryRepetetion = FALSE;
	u_int8_t fgTableValid = TRUE;

	/*1.Configuration Table Check */
	for (i = 0; i < sizeof(PwrLmtConf) /
	     sizeof(struct COUNTRY_POWER_LIMIT_TABLE_CONFIGURATION); i++) {
		/*Table Country Code */
		WLAN_GET_FIELD_BE16(&PwrLmtConf[i].aucCountryCode[0],
				    &u2CountryCodeTable);

		/*<1>Repetition Entry Check */
		for (j = i + 1; j < sizeof(PwrLmtConf) /
		     sizeof(struct COUNTRY_POWER_LIMIT_TABLE_CONFIGURATION);
		     j++) {

			WLAN_GET_FIELD_BE16(&PwrLmtConf[j].aucCountryCode[0],
					    &u2CountryCodeCheck);
			if (((PwrLmtConf[i].ucCentralCh) ==
			     PwrLmtConf[j].ucCentralCh)
			    && (u2CountryCodeTable == u2CountryCodeCheck)) {
				fgEntryRepetetion = TRUE;
				DBGLOG(RLM, LOUD,
				       "Domain: Configuration Repetition CC=%c%c, Ch=%d\n",
				       PwrLmtConf[i].aucCountryCode[0],
				       PwrLmtConf[i].aucCountryCode[1],
				       PwrLmtConf[i].ucCentralCh);
			}
		}

		/*<2>Channel Number Interval Check */
		fgChannelValid =
		    rlmDomainCheckChannelEntryValid(prAdapter,
				BAND_NULL, PwrLmtConf[i].ucCentralCh);

		/*<3>Power Limit Range Check */
		fgPowerLimitValid =
		    rlmDomainCheckPowerLimitValid(prAdapter,
						  PwrLmtConf[i],
						  PWR_LIMIT_NUM);

		if (fgChannelValid == FALSE || fgPowerLimitValid == FALSE) {
			fgTableValid = FALSE;
			DBGLOG(RLM, LOUD,
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
				"Domain: CC=%c%c, Ch=%d, Limit: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d, Valid:%d,%d\n",
#else
				"Domain: CC=%c%c, Ch=%d, Limit: %d,%d,%d,%d,%d,%d,%d,%d,%d, Valid:%d,%d\n",
#endif
				PwrLmtConf[i].aucCountryCode[0],
				PwrLmtConf[i].aucCountryCode[1],
				PwrLmtConf[i].ucCentralCh,
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_CCK_L],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_CCK_H],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_OFDM_L],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_OFDM_H],
#else
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_CCK],
#endif
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_20M_L],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_20M_H],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_40M_L],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_40M_H],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_80M_L],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_80M_H],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_160M_L],
				PwrLmtConf[i].aucPwrLimit[PWR_LIMIT_160M_H],
				fgChannelValid,
				fgPowerLimitValid);
		}

		if (u2CountryCodeTable == COUNTRY_CODE_NULL) {
			DBGLOG(RLM, LOUD, "Domain: Full search down\n");
			break;	/*End of country table entry */
		}

	}

	if (fgEntryRepetetion == FALSE)
		DBGLOG(RLM, TRACE,
		       "Domain: Configuration Table no Repetiton.\n");

	/*Configuration Table no error */
	if (fgTableValid == TRUE)
		prAdapter->fgIsPowerLimitTableValid = TRUE;
	else
		prAdapter->fgIsPowerLimitTableValid = FALSE;

	/*2.Default Table Repetition Entry Check */
	fgEntryRepetetion = FALSE;
	for (i = 0; i < sizeof(g_rRlmPowerLimitDefault) /
	     sizeof(struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT); i++) {

		WLAN_GET_FIELD_BE16(&g_rRlmPowerLimitDefault[i].
							aucCountryCode[0],
				    &u2CountryCodeTable);

		for (j = i + 1; j < sizeof(g_rRlmPowerLimitDefault) /
		     sizeof(struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT); j++) {
			WLAN_GET_FIELD_BE16(&g_rRlmPowerLimitDefault[j].
							aucCountryCode[0],
					    &u2CountryCodeCheck);
			if (u2CountryCodeTable == u2CountryCodeCheck) {
				fgEntryRepetetion = TRUE;
				DBGLOG(RLM, LOUD,
				       "Domain: Default Repetition CC=%c%c\n",
				       g_rRlmPowerLimitDefault[j].
							aucCountryCode[0],
				       g_rRlmPowerLimitDefault[j].
							aucCountryCode[1]);
			}
		}
	}
	if (fgEntryRepetetion == FALSE)
		DBGLOG(RLM, TRACE, "Domain: Default Table no Repetiton.\n");
#undef PwrLmtConf
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief
 *
 * @param[in]
 *
 * @return (u2TableIndex) : if  0xFFFF -> No Table Match
 */
/*----------------------------------------------------------------------------*/
uint16_t rlmDomainPwrLimitDefaultTableDecision(struct ADAPTER *prAdapter,
					       uint16_t u2CountryCode)
{

	uint16_t i;
	uint16_t u2CountryCodeTable = COUNTRY_CODE_NULL;
	uint16_t u2TableIndex = POWER_LIMIT_TABLE_NULL;	/* No Table Match */

	/*Default Table Index */
	for (i = 0; i < sizeof(g_rRlmPowerLimitDefault) /
	     sizeof(struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT); i++) {

		WLAN_GET_FIELD_BE16(&g_rRlmPowerLimitDefault[i].
						aucCountryCode[0],
				    &u2CountryCodeTable);

		if (u2CountryCodeTable == u2CountryCode) {
			u2TableIndex = i;
			break;	/*match country code */
		} else if (u2CountryCodeTable == COUNTRY_CODE_NULL) {
			u2TableIndex = i;
			break;	/*find last one country- Default */
		}
	}

	return u2TableIndex;
}

/*----------------------------------------------------------------------------*/
/*!
 * @brief Fill power limit CMD by Power Limit Default Table(regulation)
 *
 * @param[in]
 *
 * @return (none)
 */
/*----------------------------------------------------------------------------*/
void
rlmDomainBuildCmdByDefaultTable(struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT
				*prCmd,
				uint16_t u2DefaultTableIndex)
{
	uint16_t i, k;
	struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT *prPwrLimitSubBand = NULL;
	struct CMD_CHANNEL_POWER_LIMIT *prPwrLimit = NULL;
	struct CMD_CHANNEL_POWER_LIMIT_HE *prPwrLmtHE = NULL;
	struct CMD_CHANNEL_POWER_LIMIT_HE_BW160 *prPwrLmtHEBW160 = NULL;
#if (CFG_SUPPORT_WIFI_6G == 1)
	struct CMD_CHANNEL_POWER_LIMIT_6E *prPwrLmt6E = NULL;
#endif
	enum ENUM_PWR_LIMIT_TYPE eType;
	int8_t startIndex, endIndex;

	int8_t cLmtBand = 0;

	ASSERT(prCmd);

	prPwrLimitSubBand = &g_rRlmPowerLimitDefault[u2DefaultTableIndex];
	eType = prCmd->ucLimitType;
	if (eType == PWR_LIMIT_TYPE_COMP_11AX)
		prPwrLmtHE = &prCmd->u.rChPwrLimtHE[0];
	else if (eType == PWR_LIMIT_TYPE_COMP_11AX_BW160)
		prPwrLmtHEBW160 = &prCmd->u.rChPwrLimtHEBW160[0];
#if (CFG_SUPPORT_WIFI_6G == 1)
	else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
				eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
				eType == PWR_LIMIT_TYPE_COMP_6E_3)
		prPwrLmt6E = &prCmd->u.rChPwrLimt6E[0];
#endif
	else
		prPwrLimit = &prCmd->u.rChannelPowerLimit[0];

	startIndex = POWER_LIMIT_2G4;
	endIndex = POWER_LIMIT_UNII3;
#if (CFG_SUPPORT_WIFI_6G == 1)
	if (eType == PWR_LIMIT_TYPE_COMP_6E_1) {
		startIndex = POWER_LIMIT_UNII5;
		endIndex = POWER_LIMIT_UNII5;
	} else if (eType == PWR_LIMIT_TYPE_COMP_6E_2) {
		startIndex = POWER_LIMIT_UNII6;
		endIndex = POWER_LIMIT_UNII7;
	} else if (eType == PWR_LIMIT_TYPE_COMP_6E_3) {
		startIndex = POWER_LIMIT_UNII8;
		endIndex = POWER_LIMIT_UNII8;
	}
#endif
	/*Build power limit cmd by default table information */

	for (i = startIndex; i <= endIndex; i++) {
		cLmtBand = prPwrLimitSubBand->aucPwrLimitSubBand[i];

		if (prPwrLimitSubBand->aucPwrLimitSubBand[i] > MAX_TX_POWER) {
			DBGLOG(RLM, WARN, "SubBand[%d] Pwr(%d) > Max (%d)",
				prPwrLimitSubBand->aucPwrLimitSubBand[i],
				MAX_TX_POWER);
			cLmtBand = MAX_TX_POWER;
		}

		for (k = g_rRlmSubBand[i].ucStartCh;
		     k <= g_rRlmSubBand[i].ucEndCh;
		     k += g_rRlmSubBand[i].ucInterval) {

			if (eType == PWR_LIMIT_TYPE_COMP_11AX)
				prPwrLmtHE->ucCentralCh = k;
			else if (eType == PWR_LIMIT_TYPE_COMP_11AX_BW160)
				prPwrLmtHEBW160->ucCentralCh = k;
#if (CFG_SUPPORT_WIFI_6G == 1)
			else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3)
				prPwrLmt6E->ucCentralCh = k;
#endif
			else
				prPwrLimit->ucCentralCh = k;

			if ((prPwrLimitSubBand->ucPwrUnit
						& BIT(i)) == 0) {

				if (eType == PWR_LIMIT_TYPE_COMP_11AX)
					kalMemSet(&prPwrLmtHE->cPwrLimitRU26L,
						cLmtBand,
						PWR_LIMIT_HE_NUM);
				else if (eType ==
					PWR_LIMIT_TYPE_COMP_11AX_BW160)
					kalMemSet(
					  &prPwrLmtHEBW160->cPwrLimitRU26L,
					  cLmtBand,
					  PWR_LIMIT_HE_BW160_NUM);
#if (CFG_SUPPORT_WIFI_6G == 1)
				else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3)
					kalMemSet(&prPwrLmt6E->cPwrLimitRU26L,
						cLmtBand,
						PWR_LIMIT_6E_NUM);
#endif
				else
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
					kalMemSet(&prPwrLimit->cPwrLimitCCK_L,
#else
					kalMemSet(&prPwrLimit->cPwrLimitCCK,
#endif
					cLmtBand,
					PWR_LIMIT_NUM);
			} else {
					/* ex: 40MHz power limit(mW\MHz)
					 *	= 20MHz power limit(mW\MHz) * 2
					 * ---> 40MHz power limit(dBm)
					 *	= 20MHz power limit(dBm) + 6;
					 */

				if (eType == PWR_LIMIT_TYPE_COMP_11AX) {
					/* BW20 */
					prPwrLmtHE->cPwrLimitRU26L = cLmtBand;
					prPwrLmtHE->cPwrLimitRU26H = cLmtBand;
					prPwrLmtHE->cPwrLimitRU26U = cLmtBand;

					prPwrLmtHE->cPwrLimitRU52L = cLmtBand;
					prPwrLmtHE->cPwrLimitRU52H = cLmtBand;
					prPwrLmtHE->cPwrLimitRU52U = cLmtBand;

					prPwrLmtHE->cPwrLimitRU106L = cLmtBand;
					prPwrLmtHE->cPwrLimitRU106H = cLmtBand;
					prPwrLmtHE->cPwrLimitRU106U = cLmtBand;

					prPwrLmtHE->cPwrLimitRU242L = cLmtBand;
					prPwrLmtHE->cPwrLimitRU242H = cLmtBand;
					prPwrLmtHE->cPwrLimitRU242U = cLmtBand;

				} else if (eType ==
					PWR_LIMIT_TYPE_COMP_11AX_BW160) {
					/* BW20 */
					prPwrLmtHEBW160->cPwrLimitRU26L =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU26H =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU26U =
						cLmtBand;

					prPwrLmtHEBW160->cPwrLimitRU52L =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU52H =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU52U =
						cLmtBand;

					prPwrLmtHEBW160->cPwrLimitRU106L =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU106H =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU106U =
						cLmtBand;

					prPwrLmtHEBW160->cPwrLimitRU242L =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU242H =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU242U =
						cLmtBand;
				}
#if (CFG_SUPPORT_WIFI_6G == 1)
				else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3) {
					/* BW20 */
					prPwrLmt6E->cPwrLimitRU26L = cLmtBand;
					prPwrLmt6E->cPwrLimitRU26H = cLmtBand;
					prPwrLmt6E->cPwrLimitRU26U = cLmtBand;

					prPwrLmt6E->cPwrLimitRU52L = cLmtBand;
					prPwrLmt6E->cPwrLimitRU52H = cLmtBand;
					prPwrLmt6E->cPwrLimitRU52U = cLmtBand;

					prPwrLmt6E->cPwrLimitRU106L = cLmtBand;
					prPwrLmt6E->cPwrLimitRU106H = cLmtBand;
					prPwrLmt6E->cPwrLimitRU106U = cLmtBand;

					prPwrLmt6E->cPwrLimitRU242L = cLmtBand;
					prPwrLmt6E->cPwrLimitRU242H = cLmtBand;
					prPwrLmt6E->cPwrLimitRU242U = cLmtBand;
				}
#endif
				else {
					/* BW20 */
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
					prPwrLimit->cPwrLimitCCK_L = cLmtBand;
					prPwrLimit->cPwrLimitCCK_H = cLmtBand;
					prPwrLimit->cPwrLimitOFDM_L = cLmtBand;
					prPwrLimit->cPwrLimitOFDM_H = cLmtBand;
#else
					prPwrLimit->cPwrLimitCCK = cLmtBand;
#endif /* CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING */
					prPwrLimit->cPwrLimit20L = cLmtBand;
					prPwrLimit->cPwrLimit20H = cLmtBand;
				}

				/* BW40 */
				cLmtBand += 6;

				if (cLmtBand > MAX_TX_POWER)
					cLmtBand = MAX_TX_POWER;

				if (eType == PWR_LIMIT_TYPE_COMP_11AX) {
					prPwrLmtHE->cPwrLimitRU484L = cLmtBand;
					prPwrLmtHE->cPwrLimitRU484H = cLmtBand;
					prPwrLmtHE->cPwrLimitRU484U = cLmtBand;
				} else if (eType ==
					PWR_LIMIT_TYPE_COMP_11AX_BW160) {
					prPwrLmtHEBW160->cPwrLimitRU484L =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU484H =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU484U =
						cLmtBand;
				}
#if (CFG_SUPPORT_WIFI_6G == 1)
				else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3) {
					prPwrLmt6E->cPwrLimitRU484L = cLmtBand;
					prPwrLmt6E->cPwrLimitRU484H = cLmtBand;
					prPwrLmt6E->cPwrLimitRU484U = cLmtBand;
				}
#endif
				else {
					prPwrLimit->cPwrLimit40L = cLmtBand;
					prPwrLimit->cPwrLimit40H = cLmtBand;
				}

				/* BW80 */
				cLmtBand += 6;
				if (cLmtBand > MAX_TX_POWER)
					cLmtBand = MAX_TX_POWER;

				if (eType == PWR_LIMIT_TYPE_COMP_11AX) {
					prPwrLmtHE->cPwrLimitRU996L = cLmtBand;
					prPwrLmtHE->cPwrLimitRU996H = cLmtBand;
					prPwrLmtHE->cPwrLimitRU996U = cLmtBand;
				} else if (eType ==
					PWR_LIMIT_TYPE_COMP_11AX_BW160) {
					prPwrLmtHEBW160->cPwrLimitRU996L =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU996H =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU996U =
						cLmtBand;
				}
#if (CFG_SUPPORT_WIFI_6G == 1)
				else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3) {
					prPwrLmt6E->cPwrLimitRU996L = cLmtBand;
					prPwrLmt6E->cPwrLimitRU996H = cLmtBand;
					prPwrLmt6E->cPwrLimitRU996U = cLmtBand;
				}
#endif
				else {
					prPwrLimit->cPwrLimit80L = cLmtBand;
					prPwrLimit->cPwrLimit80H = cLmtBand;
				}

					/* BW160 */
				cLmtBand += 6;
				if (cLmtBand > MAX_TX_POWER)
					cLmtBand = MAX_TX_POWER;
				if (eType == PWR_LIMIT_TYPE_COMP_11AX_BW160) {
					/* prPwrLmtHE do nothing */
					prPwrLmtHEBW160->cPwrLimitRU1992L =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU1992H =
						cLmtBand;
					prPwrLmtHEBW160->cPwrLimitRU1992U =
						cLmtBand;
				}
#if (CFG_SUPPORT_WIFI_6G == 1)
				else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3) {
					prPwrLmt6E->cPwrLimitRU1992L = cLmtBand;
					prPwrLmt6E->cPwrLimitRU1992H = cLmtBand;
					prPwrLmt6E->cPwrLimitRU1992U = cLmtBand;
				}
#endif
				else {
					prPwrLimit->cPwrLimit160L = cLmtBand;
					prPwrLimit->cPwrLimit160H = cLmtBand;
				}
			}
				/* save to power limit array per
				 * subband channel
				 */
			if (eType == PWR_LIMIT_TYPE_COMP_11AX)
				prPwrLmtHE++;
			else if (eType == PWR_LIMIT_TYPE_COMP_11AX_BW160)
				prPwrLmtHEBW160++;
#if (CFG_SUPPORT_WIFI_6G == 1)
			else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3)
				prPwrLmt6E++;
#endif
			else
				prPwrLimit++;

			prCmd->ucNum++;

			if (prCmd->ucNum >= MAX_CMD_SUPPORT_CHANNEL_NUM) {
				DBGLOG(RLM, WARN,
					"etype = %d, out of MAX CH Num\n",
					eType);
				break;
			}

		}
	}

	DBGLOG(RLM, TRACE, "Build Default Limit(%c%c)ChNum=%d,typde=%d\n",
				((prCmd->u2CountryCode &
				0xff00) >> 8),
				(prCmd->u2CountryCode &
				0x00ff),
				prCmd->ucNum,
				eType);

}

void rlmDomainCopyFromConfigTable(struct CMD_CHANNEL_POWER_LIMIT *prCmdPwrLimit,
	struct COUNTRY_POWER_LIMIT_TABLE_CONFIGURATION *prPwrLimitConfig)
{


#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
	kalMemCopy(&prCmdPwrLimit->cPwrLimitCCK_L,
		   &prPwrLimitConfig->aucPwrLimit[0],
		   PWR_LIMIT_NUM);
#else
	kalMemCopy(&prCmdPwrLimit->cPwrLimitCCK,
		   &prPwrLimitConfig->aucPwrLimit[0],
		   PWR_LIMIT_NUM);
#endif /* CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING */
}


/*----------------------------------------------------------------------------*/
/*!
 * @brief Fill power limit CMD by Power Limit Configurartion Table
 * (Bandedge and Customization)
 *
 * @param[in]
 *
 * @return (none)
 */
/*----------------------------------------------------------------------------*/
void rlmDomainBuildCmdByConfigTable(struct ADAPTER *prAdapter,
			struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd)
{
#define PwrLmtConf g_rRlmPowerLimitConfiguration
#define PwrLmtConfHE g_rRlmPowerLimitConfigurationHE
#define PwrLmtConfHEBW160 g_rRlmPowerLimitConfigurationHEBW160
#if (CFG_SUPPORT_WIFI_6G == 1)
#define PwrLmtConf6E g_rRlmPowerLimitConfiguration6E
#endif
	uint16_t i, k;
	uint16_t u2CountryCodeTable = COUNTRY_CODE_NULL;
	enum ENUM_PWR_LIMIT_TYPE eType;
	struct CMD_CHANNEL_POWER_LIMIT *prCmdPwrLimit;
	struct CMD_CHANNEL_POWER_LIMIT_HE *prCmdPwrLimtHE;
	struct CMD_CHANNEL_POWER_LIMIT_HE_BW160 *prCmdPwrLimtHEBW160;
#if (CFG_SUPPORT_WIFI_6G == 1)
	struct CMD_CHANNEL_POWER_LIMIT_6E *prCmdPwrLimt6E;
#endif
	u_int8_t fgChannelValid;
	uint8_t ucCentCh;
	uint8_t ucPwrLmitConfSize = sizeof(PwrLmtConf) /
		sizeof(struct COUNTRY_POWER_LIMIT_TABLE_CONFIGURATION);

	uint8_t ucPwrLmitConfSizeHE = sizeof(PwrLmtConfHE) /
		sizeof(struct COUNTRY_POWER_LIMIT_TABLE_CONFIGURATION_HE);
#if (CFG_SUPPORT_WIFI_6G == 1)
	uint8_t ucPwrLmitConfSize6E = sizeof(PwrLmtConf6E) /
		sizeof(struct COUNTRY_POWER_LIMIT_TABLE_CONFIGURATION_6E);
#endif
	eType = prCmd->ucLimitType;

	/*Build power limit cmd by configuration table information */
	for (k = 0; k < prCmd->ucNum; k++) {

		if (k >= MAX_CMD_SUPPORT_CHANNEL_NUM) {
			DBGLOG(RLM, ERROR, "out of MAX CH Num\n");
			return;
		}

		if (eType == PWR_LIMIT_TYPE_COMP_11AX) {
			prCmdPwrLimtHE = &prCmd->u.rChPwrLimtHE[k];
			ucCentCh = prCmdPwrLimtHE->ucCentralCh;

			for (i = 0; i < ucPwrLmitConfSizeHE ; i++) {

				WLAN_GET_FIELD_BE16(
					&PwrLmtConfHE[i].aucCountryCode[0],
					&u2CountryCodeTable);

				fgChannelValid =
					rlmDomainCheckChannelEntryValid(
						prAdapter,
						BAND_NULL,
						PwrLmtConfHE[i].ucCentralCh);

				if (u2CountryCodeTable == COUNTRY_CODE_NULL)
					break;	/*end of configuration table */
				else if (u2CountryCodeTable
					!= prCmd->u2CountryCode)
					continue;
				else if (fgChannelValid == FALSE)
					continue;
				else if (ucCentCh
					!= PwrLmtConfHE[i].ucCentralCh)
					continue;

				/* Cmd setting (Default table
				 * information) and Conf table
				 * has repetition channel entry,
				 * ex : Default table (ex: 2.4G,
				 *      limit = 20dBm) -->
				 *      ch1~14 limit =20dBm,
				 * Conf table (ex: ch1, limit =
				 *      22dBm) --> ch 1 = 22 dBm
				 * Cmd final setting --> ch1 =
				 *      22dBm, ch2~14 = 20dBm
				 */
				kalMemCopy(&prCmdPwrLimtHE->cPwrLimitRU26L,
					   &PwrLmtConfHE[i].aucPwrLimit,
					   PWR_LIMIT_HE_NUM);
			}
		} else if (eType == PWR_LIMIT_TYPE_COMP_11AX_BW160) {
			prCmdPwrLimtHEBW160 = &prCmd->u.rChPwrLimtHEBW160[k];
			ucCentCh = prCmdPwrLimtHEBW160->ucCentralCh;

			for (i = 0; i < ucPwrLmitConfSizeHE ; i++) {

				WLAN_GET_FIELD_BE16(
					&PwrLmtConfHEBW160[i].aucCountryCode[0],
					&u2CountryCodeTable);

				fgChannelValid =
					rlmDomainCheckChannelEntryValid(
					  prAdapter,
					  BAND_NULL,
					  PwrLmtConfHEBW160[i].ucCentralCh);

				if (u2CountryCodeTable == COUNTRY_CODE_NULL)
					break;	/*end of configuration table */
				else if (u2CountryCodeTable
					!= prCmd->u2CountryCode)
					continue;
				else if (fgChannelValid == FALSE)
					continue;
				else if (ucCentCh
					!= PwrLmtConfHEBW160[i].ucCentralCh)
					continue;

				/* Cmd setting (Default table
				 * information) and Conf table
				 * has repetition channel entry,
				 * ex : Default table (ex: 2.4G,
				 *      limit = 20dBm) -->
				 *      ch1~14 limit =20dBm,
				 * Conf table (ex: ch1, limit =
				 *      22dBm) --> ch 1 = 22 dBm
				 * Cmd final setting --> ch1 =
				 *      22dBm, ch2~14 = 20dBm
				 */
				kalMemCopy(&prCmdPwrLimtHEBW160->cPwrLimitRU26L,
					   &PwrLmtConfHEBW160[i].aucPwrLimit,
					   PWR_LIMIT_HE_BW160_NUM);
			}
		}
#if (CFG_SUPPORT_WIFI_6G == 1)
		else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3) {
			prCmdPwrLimt6E = &prCmd->u.rChPwrLimt6E[k];
			ucCentCh = prCmdPwrLimt6E->ucCentralCh;

			for (i = 0; i < ucPwrLmitConfSize6E ; i++) {

				WLAN_GET_FIELD_BE16(
					&PwrLmtConf6E[i].aucCountryCode[0],
					&u2CountryCodeTable);

				fgChannelValid =
				    rlmDomainCheckChannelEntryValid(prAdapter,
				    BAND_6G,
					PwrLmtConf6E[i].ucCentralCh);

				if (u2CountryCodeTable == COUNTRY_CODE_NULL)
					break;	/*end of configuration table */
				else if (u2CountryCodeTable
					!= prCmd->u2CountryCode)
					continue;
				else if (fgChannelValid == FALSE)
					continue;
				else if (ucCentCh
					!= PwrLmtConf6E[i].ucCentralCh)
					continue;

				kalMemCopy(&prCmdPwrLimt6E->cPwrLimitRU26L,
					   &PwrLmtConf6E[i].aucPwrLimit,
					   PWR_LIMIT_6E_NUM);
			}
		}
#endif
		else {
			prCmdPwrLimit = &prCmd->u.rChannelPowerLimit[k];
			ucCentCh = prCmdPwrLimit->ucCentralCh;

			for (i = 0; i < ucPwrLmitConfSize ; i++) {

				WLAN_GET_FIELD_BE16(
					&PwrLmtConf[i].aucCountryCode[0],
					&u2CountryCodeTable);

				fgChannelValid =
					rlmDomainCheckChannelEntryValid(
						prAdapter,
						BAND_NULL,
						PwrLmtConf[i].ucCentralCh);

				if (u2CountryCodeTable == COUNTRY_CODE_NULL)
					break;	/*end of configuration table */
				else if (u2CountryCodeTable
					!= prCmd->u2CountryCode)
					continue;
				else if (fgChannelValid == FALSE)
					continue;
				else if (ucCentCh != PwrLmtConf[i].ucCentralCh)
					continue;

				/* Cmd setting (Default table
				 * information) and Conf table
				 * has repetition channel entry,
				 * ex : Default table (ex: 2.4G,
				 *      limit = 20dBm) -->
				 *      ch1~14 limit =20dBm,
				 * Conf table (ex: ch1, limit =
				 *      22dBm) --> ch 1 = 22 dBm
				 * Cmd final setting --> ch1 =
				 *      22dBm, ch2~14 = 20dBm
				 */
				rlmDomainCopyFromConfigTable(
					prCmdPwrLimit,
					&PwrLmtConf[i]
				);
			}
		}
	}
#undef PwrLmtConf
#undef PwrLmtConfHE
#undef PwrLmtConfHEBW160
#if (CFG_SUPPORT_WIFI_6G == 1)
#undef PwrLmtConf6E
#endif
}

struct TX_PWR_LIMIT_DATA *
rlmDomainInitTxPwrLimitData(struct ADAPTER *prAdapter)
{
	uint8_t ch_cnt = 0;
	uint8_t ch_idx = 0;
	uint8_t band_idx = 0;
	uint8_t ch_num = 0;
	const int8_t *prChannelList = NULL;
	struct TX_PWR_LIMIT_DATA *pTxPwrLimitData = NULL;
	struct CHANNEL_TX_PWR_LIMIT *prChTxPwrLimit = NULL;

	pTxPwrLimitData =
		(struct TX_PWR_LIMIT_DATA *)
		kalMemAlloc(sizeof(struct TX_PWR_LIMIT_DATA),
			VIR_MEM_TYPE);

	if (!pTxPwrLimitData) {
		DBGLOG(RLM, ERROR,
			"Alloc buffer for TxPwrLimit main struct failed\n");
		return NULL;
	}

	pTxPwrLimitData->ucChNum =
		TX_PWR_LIMIT_2G_CH_NUM + TX_PWR_LIMIT_5G_CH_NUM;

	pTxPwrLimitData->rChannelTxPwrLimit =
		(struct CHANNEL_TX_PWR_LIMIT *)
		kalMemAlloc(sizeof(struct CHANNEL_TX_PWR_LIMIT) *
			(pTxPwrLimitData->ucChNum), VIR_MEM_TYPE);

	if (!pTxPwrLimitData->rChannelTxPwrLimit) {
		DBGLOG(RLM, ERROR,
			"Alloc buffer for TxPwrLimit ch values failed\n");

		kalMemFree(pTxPwrLimitData, VIR_MEM_TYPE,
			sizeof(struct TX_PWR_LIMIT_DATA));
		return NULL;
	}

	for (ch_idx = 0; ch_idx < pTxPwrLimitData->ucChNum; ch_idx++) {
		prChTxPwrLimit =
			&(pTxPwrLimitData->rChannelTxPwrLimit[ch_idx]);
		kalMemSet(prChTxPwrLimit->rTxPwrLimitValue,
			MAX_TX_POWER,
			sizeof(prChTxPwrLimit->rTxPwrLimitValue));
		kalMemSet(prChTxPwrLimit->rTxBfBackoff,
			MAX_TX_POWER,
			sizeof(prChTxPwrLimit->rTxBfBackoff));
	}

	ch_cnt = 0;
	for (band_idx = 0; band_idx < KAL_NUM_BANDS; band_idx++) {
		if (band_idx != KAL_BAND_2GHZ && band_idx != KAL_BAND_5GHZ)
			continue;

		prChannelList = (band_idx == KAL_BAND_2GHZ) ?
			gTx_Pwr_Limit_2g_Ch : gTx_Pwr_Limit_5g_Ch;

		ch_num =
			(band_idx == KAL_BAND_2GHZ) ?
				TX_PWR_LIMIT_2G_CH_NUM :
				TX_PWR_LIMIT_5G_CH_NUM;

		for (ch_idx = 0; ch_idx < ch_num; ch_idx++) {
			pTxPwrLimitData->rChannelTxPwrLimit[ch_cnt].ucChannel =
				prChannelList[ch_idx];
			++ch_cnt;
		}
	}

	return pTxPwrLimitData;
}

void
rlmDomainSendTxPwrLimitCmd(struct ADAPTER *prAdapter,
	uint8_t ucVersion,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimitData)
{
	uint8_t ch_cnt = 0;
	uint8_t ch_idx = 0;
	uint8_t band_idx = 0;
	const int8_t *prChannelList = NULL;
	uint32_t rStatus;
	uint32_t u4SetQueryInfoLen;
	uint32_t u4SetCmdTableMaxSize[KAL_NUM_BANDS] = {0};
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT_V2
		*prCmd[KAL_NUM_BANDS] = {0};
	struct CMD_CHANNEL_POWER_LIMIT_V2 *prCmdChPwrLimitV2 = NULL;
	uint32_t u4SetCountryCmdSize =
		sizeof(struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT_V2);
	uint32_t u4ChPwrLimitV2Size = sizeof(struct CMD_CHANNEL_POWER_LIMIT_V2);
	const uint8_t ucCmdBatchSize =
		prAdapter->chip_info->ucTxPwrLimitBatchSize;

	for (band_idx = 0; band_idx < KAL_NUM_BANDS; band_idx++) {
		if (band_idx != KAL_BAND_2GHZ && band_idx != KAL_BAND_5GHZ)
			continue;

		prChannelList = (band_idx == KAL_BAND_2GHZ) ?
			gTx_Pwr_Limit_2g_Ch : gTx_Pwr_Limit_5g_Ch;

		ch_cnt = (band_idx == KAL_BAND_2GHZ) ? TX_PWR_LIMIT_2G_CH_NUM :
			TX_PWR_LIMIT_5G_CH_NUM;

		if (!ch_cnt)
			continue;

		u4SetCmdTableMaxSize[band_idx] = u4SetCountryCmdSize +
			ch_cnt * u4ChPwrLimitV2Size;

		prCmd[band_idx] = cnmMemAlloc(prAdapter,
			RAM_TYPE_BUF, u4SetCmdTableMaxSize[band_idx]);

		if (!prCmd[band_idx]) {
			DBGLOG(RLM, ERROR, "Domain: no buf to send cmd\n");
			goto error;
		}

		/*initialize tw pwr table*/
		kalMemSet(prCmd[band_idx], MAX_TX_POWER,
			u4SetCmdTableMaxSize[band_idx]);

		prCmd[band_idx]->ucNum = ch_cnt;
		prCmd[band_idx]->eband =
			(band_idx == KAL_BAND_2GHZ) ?
				BAND_2G4 : BAND_5G;
		prCmd[band_idx]->countryCode = rlmDomainGetCountryCode();

		DBGLOG(RLM, INFO,
			"%s, active n_channels=%d, band=%d\n",
			__func__, ch_cnt, prCmd[band_idx]->eband);

		for (ch_idx = 0; ch_idx < ch_cnt; ch_idx++) {
			prCmdChPwrLimitV2 =
				&(prCmd[band_idx]->rChannelPowerLimit[ch_idx]);
			prCmdChPwrLimitV2->ucCentralCh =
				prChannelList[ch_idx];
		}
	}

	rlmDomainTxPwrLimitSetValues(ucVersion,
		prCmd[KAL_BAND_2GHZ], pTxPwrLimitData);
	rlmDomainTxPwrLimitSetValues(ucVersion,
		prCmd[KAL_BAND_5GHZ], pTxPwrLimitData);

	for (band_idx = 0; band_idx < KAL_NUM_BANDS; band_idx++) {
		uint8_t ucRemainChNum, i, ucTempChNum, prCmdBatchNum;
		uint32_t u4BufSize = 0;
		struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT_V2 *prTempCmd = NULL;
		enum ENUM_BAND eBand = (band_idx == KAL_BAND_2GHZ) ?
				BAND_2G4 : BAND_5G;
		uint16_t u2ChIdx = 0;

		if (!prCmd[band_idx])
			continue;

		ucRemainChNum = prCmd[band_idx]->ucNum;
		prCmdBatchNum = (ucRemainChNum +
			ucCmdBatchSize - 1) /
			ucCmdBatchSize;

		for (i = 0; i < prCmdBatchNum; i++) {
			if (i == prCmdBatchNum - 1)
				ucTempChNum = ucRemainChNum;
			else
				ucTempChNum = ucCmdBatchSize;

			u4BufSize = u4SetCountryCmdSize +
				ucTempChNum * u4ChPwrLimitV2Size;

			prTempCmd =
				cnmMemAlloc(prAdapter,
					RAM_TYPE_BUF, u4BufSize);

			if (!prTempCmd) {
				DBGLOG(RLM, ERROR,
					"Domain: no buf to send cmd\n");
				goto error;
			}

			/*copy partial tx pwr limit*/
			prTempCmd->ucNum = ucTempChNum;
			prTempCmd->eband = eBand;
			prTempCmd->countryCode = rlmDomainGetCountryCode();
			u2ChIdx = i * ucCmdBatchSize;
			kalMemCopy(&prTempCmd->rChannelPowerLimit[0],
				&prCmd[band_idx]->rChannelPowerLimit[u2ChIdx],
				ucTempChNum * u4ChPwrLimitV2Size);

			u4SetQueryInfoLen = u4BufSize;
			/* Update tx max. power info to chip */
			rStatus = wlanSendSetQueryCmd(prAdapter,
				CMD_ID_SET_COUNTRY_POWER_LIMIT,
				TRUE,
				FALSE,
				FALSE,
				NULL,
				NULL,
				u4SetQueryInfoLen,
				(uint8_t *) prTempCmd,
				NULL,
				0);

			cnmMemFree(prAdapter, prTempCmd);

			ucRemainChNum -= ucTempChNum;
		}
	}

error:
	for (band_idx = 0; band_idx < KAL_NUM_BANDS; band_idx++) {
		if (prCmd[band_idx])
			cnmMemFree(prAdapter, prCmd[band_idx]);
	}
}

u_int32_t rlmDomainInitTxPwrLimitPerRateCmd(
	struct ADAPTER *prAdapter,
	struct wiphy *prWiphy,
	struct CMD_SET_TXPOWER_COUNTRY_TX_POWER_LIMIT_PER_RATE *prCmd[])
{
	uint8_t ch_cnt = 0;
	uint8_t ch_idx = 0;
	uint8_t band_idx = 0;
	const int8_t *prChannelList = NULL;
	uint32_t u4SetCmdTableMaxSize[KAL_NUM_BANDS] = {0};
	uint32_t u4SetCountryTxPwrLimitCmdSize =
		sizeof(struct CMD_SET_TXPOWER_COUNTRY_TX_POWER_LIMIT_PER_RATE);
	uint32_t u4ChPwrLimitSize =
		sizeof(struct CMD_TXPOWER_CHANNEL_POWER_LIMIT_PER_RATE);
	struct CMD_TXPOWER_CHANNEL_POWER_LIMIT_PER_RATE *prChPwrLimit = NULL;

	for (band_idx = 0; band_idx < KAL_NUM_BANDS; band_idx++) {
		if (band_idx != KAL_BAND_2GHZ && band_idx != KAL_BAND_5GHZ)
			continue;

		prChannelList = (band_idx == KAL_BAND_2GHZ) ?
			gTx_Pwr_Limit_2g_Ch : gTx_Pwr_Limit_5g_Ch;

		ch_cnt = (band_idx == KAL_BAND_2GHZ) ? TX_PWR_LIMIT_2G_CH_NUM :
			TX_PWR_LIMIT_5G_CH_NUM;

		if (!ch_cnt)
			continue;

		u4SetCmdTableMaxSize[band_idx] = u4SetCountryTxPwrLimitCmdSize +
			ch_cnt * u4ChPwrLimitSize;

		prCmd[band_idx] = cnmMemAlloc(prAdapter,
			RAM_TYPE_BUF, u4SetCmdTableMaxSize[band_idx]);

		if (!prCmd[band_idx]) {
			DBGLOG(RLM, ERROR, "Domain: no buf to send cmd\n");
			return WLAN_STATUS_RESOURCES;
		}

		/*initialize tx pwr table*/
		kalMemSet(prCmd[band_idx]->rChannelPowerLimit, MAX_TX_POWER,
			ch_cnt * u4ChPwrLimitSize);

		prCmd[band_idx]->ucNum = ch_cnt;
		prCmd[band_idx]->eBand =
			(band_idx == KAL_BAND_2GHZ) ?
				BAND_2G4 : BAND_5G;
		prCmd[band_idx]->u4CountryCode = rlmDomainGetCountryCode();

		DBGLOG(RLM, INFO,
			"%s, active n_channels=%d, band=%d\n",
			__func__, ch_cnt, prCmd[band_idx]->eBand);

		for (ch_idx = 0; ch_idx < ch_cnt; ch_idx++) {
			prChPwrLimit =
				&(prCmd[band_idx]->
				  rChannelPowerLimit[ch_idx]);
			prChPwrLimit->u1CentralCh =	prChannelList[ch_idx];
		}

	}

	return WLAN_STATUS_SUCCESS;
}

void rlmDomainTxPwrLimitSendPerRateCmd(
	struct ADAPTER *prAdapter,
	struct CMD_SET_TXPOWER_COUNTRY_TX_POWER_LIMIT_PER_RATE *prCmd[]
)
{
	uint32_t rStatus;
	uint32_t u4SetQueryInfoLen;
	uint8_t band_idx = 0;
	uint32_t u4SetCountryTxPwrLimitCmdSize =
		sizeof(struct CMD_SET_TXPOWER_COUNTRY_TX_POWER_LIMIT_PER_RATE);
	uint32_t u4ChPwrLimitSize =
		sizeof(struct CMD_TXPOWER_CHANNEL_POWER_LIMIT_PER_RATE);
	const uint8_t ucCmdBatchSize =
		prAdapter->chip_info->ucTxPwrLimitBatchSize;

	for (band_idx = 0; band_idx < KAL_NUM_BANDS; band_idx++) {
		uint8_t ucRemainChNum, i, ucTempChNum, prCmdBatchNum;
		uint32_t u4BufSize = 0;
		struct CMD_SET_TXPOWER_COUNTRY_TX_POWER_LIMIT_PER_RATE
			*prTempCmd = NULL;
		enum ENUM_BAND eBand = (band_idx == KAL_BAND_2GHZ) ?
				BAND_2G4 : BAND_5G;
		uint16_t u2ChIdx = 0;
		u_int8_t bCmdFinished = FALSE;

		if (!prCmd[band_idx])
			continue;

		ucRemainChNum = prCmd[band_idx]->ucNum;
		prCmdBatchNum = (ucRemainChNum +
			ucCmdBatchSize - 1) /
			ucCmdBatchSize;

		for (i = 0; i < prCmdBatchNum; i++) {
			if (i == prCmdBatchNum - 1) {
				ucTempChNum = ucRemainChNum;
				bCmdFinished = TRUE;
			} else {
				ucTempChNum = ucCmdBatchSize;
			}

			u4BufSize = u4SetCountryTxPwrLimitCmdSize +
				ucTempChNum * u4ChPwrLimitSize;

			prTempCmd =
				cnmMemAlloc(prAdapter,
					RAM_TYPE_BUF, u4BufSize);

			if (!prTempCmd) {
				DBGLOG(RLM, ERROR,
					"Domain: no buf to send cmd\n");
				return;
			}

			/*copy partial tx pwr limit*/
			prTempCmd->ucNum = ucTempChNum;
			prTempCmd->eBand = eBand;
			prTempCmd->u4CountryCode =
				rlmDomainGetCountryCode();
			prTempCmd->bCmdFinished = bCmdFinished;
			u2ChIdx = i * ucCmdBatchSize;
			kalMemCopy(
				&prTempCmd->rChannelPowerLimit[0],
				&prCmd[band_idx]->
				 rChannelPowerLimit[u2ChIdx],
				ucTempChNum * u4ChPwrLimitSize);

			u4SetQueryInfoLen = u4BufSize;
			/* Update tx max. power info to chip */
			rStatus = wlanSendSetQueryCmd(prAdapter,
				CMD_ID_SET_COUNTRY_POWER_LIMIT_PER_RATE,
				TRUE,
				FALSE,
				FALSE,
				NULL,
				NULL,
				u4SetQueryInfoLen,
				(uint8_t *) prTempCmd,
				NULL,
				0);

			cnmMemFree(prAdapter, prTempCmd);

			ucRemainChNum -= ucTempChNum;
		}
	}
}

u_int32_t rlmDomainInitTxBfBackoffCmd(
	struct ADAPTER *prAdapter,
	struct wiphy *prWiphy,
	struct CMD_TXPWR_TXBF_SET_BACKOFF **prCmd
)
{
	uint8_t ucChNum = TX_PWR_LIMIT_2G_CH_NUM + TX_PWR_LIMIT_5G_CH_NUM;
	uint8_t ucChIdx = 0;
	uint8_t ucChCnt = 0;
	uint8_t ucBandIdx = 0;
	uint8_t ucAisIdx = 0;
	uint8_t ucCnt = 0;
	const int8_t *prChannelList = NULL;
	uint32_t u4SetCmdSize = sizeof(struct CMD_TXPWR_TXBF_SET_BACKOFF);
	struct CMD_TXPWR_TXBF_CHANNEL_BACKOFF *prChTxBfBackoff = NULL;

	if (ucChNum >= CMD_POWER_LIMIT_TABLE_SUPPORT_CHANNEL_NUM) {
		DBGLOG(RLM, ERROR, "ChNum %d should <= %d\n",
			ucChNum, CMD_POWER_LIMIT_TABLE_SUPPORT_CHANNEL_NUM);
		return WLAN_STATUS_FAILURE;
	}

	*prCmd = cnmMemAlloc(prAdapter,
		RAM_TYPE_BUF, u4SetCmdSize);

	if (!*prCmd) {
		DBGLOG(RLM, ERROR, "Domain: no buf to send cmd\n");
		return WLAN_STATUS_RESOURCES;
	}

	/*initialize backoff table*/
	kalMemSet((*prCmd)->rChannelTxBfBackoff, MAX_TX_POWER,
		sizeof((*prCmd)->rChannelTxBfBackoff));

	(*prCmd)->ucNum = ucChNum;
	(*prCmd)->ucBssIdx = prAdapter->prAisBssInfo[ucAisIdx]->ucBssIndex;

	for (ucBandIdx = 0; ucBandIdx < KAL_NUM_BANDS; ucBandIdx++) {
		if (ucBandIdx != KAL_BAND_2GHZ && ucBandIdx != KAL_BAND_5GHZ)
			continue;

		prChannelList = (ucBandIdx == KAL_BAND_2GHZ) ?
			gTx_Pwr_Limit_2g_Ch : gTx_Pwr_Limit_5g_Ch;
		ucChCnt = (ucBandIdx == KAL_BAND_2GHZ) ?
			TX_PWR_LIMIT_2G_CH_NUM : TX_PWR_LIMIT_5G_CH_NUM;

		for (ucChIdx = 0; ucChIdx < ucChCnt; ucChIdx++) {
			prChTxBfBackoff =
				&((*prCmd)->rChannelTxBfBackoff[ucCnt++]);
			prChTxBfBackoff->ucCentralCh =	prChannelList[ucChIdx];
		}
	}

	return WLAN_STATUS_SUCCESS;
}

void rlmDomainTxPwrTxBfBackoffSetValues(
	uint8_t ucVersion,
	struct CMD_TXPWR_TXBF_SET_BACKOFF *prTxBfBackoffCmd,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimitData)
{
	uint8_t ucIdx = 0;
	int8_t cChIdx = 0;
	struct CMD_TXPWR_TXBF_CHANNEL_BACKOFF *pChTxBfBackoff = NULL;
	struct CHANNEL_TX_PWR_LIMIT *pChTxPwrLimit = NULL;

	if (prTxBfBackoffCmd == NULL ||
		pTxPwrLimitData == NULL)
		return;

	for (ucIdx = 0; ucIdx < prTxBfBackoffCmd->ucNum; ucIdx++) {
		pChTxBfBackoff =
			&(prTxBfBackoffCmd->rChannelTxBfBackoff[ucIdx]);
		cChIdx = rlmDomainTxPwrLimitGetChIdx(pTxPwrLimitData,
			pChTxBfBackoff->ucCentralCh);

		if (cChIdx == -1) {
			DBGLOG(RLM, ERROR,
				"Invalid ch idx found while assigning values\n");
			return;
		}
		pChTxPwrLimit = &pTxPwrLimitData->rChannelTxPwrLimit[cChIdx];

		kalMemCopy(&pChTxBfBackoff->acTxBfBackoff,
			pChTxPwrLimit->rTxBfBackoff,
			sizeof(pChTxBfBackoff->acTxBfBackoff));
	}

	for (ucIdx = 0; ucIdx < prTxBfBackoffCmd->ucNum; ucIdx++) {
		pChTxBfBackoff =
			&(prTxBfBackoffCmd->rChannelTxBfBackoff[ucIdx]);

		DBGLOG(RLM, ERROR,
			"ch %d TxBf backoff 2to1 %d\n",
			pChTxBfBackoff->ucCentralCh,
			pChTxBfBackoff->acTxBfBackoff[0]);

	}

}

void rlmDomainTxPwrSendTxBfBackoffCmd(
	struct ADAPTER *prAdapter,
	struct CMD_TXPWR_TXBF_SET_BACKOFF *prCmd)
{
	uint32_t rStatus;
	uint32_t u4SetQueryInfoLen;
	uint32_t u4SetCmdSize = sizeof(struct CMD_TXPWR_TXBF_SET_BACKOFF);

	u4SetQueryInfoLen = u4SetCmdSize;
	/* Update tx max. power info to chip */
	rStatus = wlanSendSetQueryCmd(prAdapter,
		CMD_ID_SET_TXBF_BACKOFF,
		TRUE,
		FALSE,
		FALSE,
		NULL,
		NULL,
		u4SetQueryInfoLen,
		(uint8_t *) prCmd,
		NULL,
		0);

}

void
rlmDomainSendTxPwrLimitPerRateCmd(struct ADAPTER *prAdapter,
	uint8_t ucVersion,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimitData)
{
	struct wiphy *wiphy;
	uint8_t band_idx = 0;
	struct CMD_SET_TXPOWER_COUNTRY_TX_POWER_LIMIT_PER_RATE
		*prTxPwrLimitPerRateCmd[KAL_NUM_BANDS] = {0};

	wiphy = wlanGetWiphy();
	if (rlmDomainInitTxPwrLimitPerRateCmd(
		prAdapter, wiphy, prTxPwrLimitPerRateCmd) !=
		WLAN_STATUS_SUCCESS)
		goto error;

	rlmDomainTxPwrLimitPerRateSetValues(ucVersion,
		prTxPwrLimitPerRateCmd[KAL_BAND_2GHZ], pTxPwrLimitData);
	rlmDomainTxPwrLimitPerRateSetValues(ucVersion,
		prTxPwrLimitPerRateCmd[KAL_BAND_5GHZ], pTxPwrLimitData);
	rlmDomainTxPwrLimitSendPerRateCmd(prAdapter, prTxPwrLimitPerRateCmd);

error:
	for (band_idx = 0; band_idx < KAL_NUM_BANDS; band_idx++)
		if (prTxPwrLimitPerRateCmd[band_idx])
			cnmMemFree(prAdapter, prTxPwrLimitPerRateCmd[band_idx]);
}

void
rlmDomainSendTxBfBackoffCmd(struct ADAPTER *prAdapter,
	uint8_t ucVersion,
	struct TX_PWR_LIMIT_DATA *pTxPwrLimitData)
{
	struct wiphy *wiphy;
	struct CMD_TXPWR_TXBF_SET_BACKOFF
		*prTxBfBackoffCmd = NULL;

	wiphy = wlanGetWiphy();

	if (rlmDomainInitTxBfBackoffCmd(
		prAdapter, wiphy, &prTxBfBackoffCmd) !=
		WLAN_STATUS_SUCCESS)
		goto error;

	rlmDomainTxPwrTxBfBackoffSetValues(
		ucVersion, prTxBfBackoffCmd, pTxPwrLimitData);
	rlmDomainTxPwrSendTxBfBackoffCmd(prAdapter, prTxBfBackoffCmd);

error:

	if (prTxBfBackoffCmd)
		cnmMemFree(prAdapter, prTxBfBackoffCmd);
}

void rlmDomainSendPwrLimitCmd_V2(struct ADAPTER *prAdapter)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	uint8_t ucVersion = 0;
	struct TX_PWR_LIMIT_DATA *pTxPwrLimitData = NULL;

	DBGLOG(RLM, INFO, "rlmDomainSendPwrLimitCmd()\n");
	pTxPwrLimitData = rlmDomainInitTxPwrLimitData(prAdapter);

	if (!pTxPwrLimitData) {
		DBGLOG(RLM, ERROR,
			"Init TxPwrLimitData failed\n");
		goto error;
	}

	/*
	 * Get Max Tx Power from MT_TxPwrLimit.dat
	 */
	rlmDomainGetTxPwrLimit(rlmDomainGetCountryCode(),
		&ucVersion,
		prAdapter->prGlueInfo,
		pTxPwrLimitData);

	/* Prepare to send CMD to FW */
	if (ucVersion == 0) {
		rlmDomainSendTxPwrLimitCmd(prAdapter,
			ucVersion, pTxPwrLimitData);
	 } else if (ucVersion == 1) {
		rlmDomainSendTxPwrLimitPerRateCmd(prAdapter,
			ucVersion, pTxPwrLimitData);

		if (g_bTxBfBackoffExists)
			rlmDomainSendTxBfBackoffCmd(prAdapter,
				ucVersion, pTxPwrLimitData);

	} else {
		DBGLOG(RLM, WARN, "Unsupported TxPwrLimit dat version %u\n",
			ucVersion);
	}

error:
	if (pTxPwrLimitData && pTxPwrLimitData->rChannelTxPwrLimit)
		kalMemFree(pTxPwrLimitData->rChannelTxPwrLimit, VIR_MEM_TYPE,
			sizeof(struct CHANNEL_TX_PWR_LIMIT) *
			pTxPwrLimitData->ucChNum);

	if (pTxPwrLimitData)
		kalMemFree(pTxPwrLimitData, VIR_MEM_TYPE,
			sizeof(struct TX_PWR_LIMIT_DATA));
#endif
}

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT
/* dynamic tx power control: begin ********************************************/

char *txPwrGetString(char **pcContent, char *delim)
{
	char *result = NULL;

	if (pcContent == NULL)
		return NULL;

	result = kalStrSep(pcContent, delim);
	if ((pcContent == NULL) || (result == NULL) ||
	    ((result != NULL) && (kalStrLen(result) == 0)))
		return NULL;

	return result;
}

uint32_t txPwrParseNumber(char **pcContent, char *delim, uint8_t *op,
			  int8_t *value) {
	u_int8_t fgIsNegtive = FALSE;
	char *pcTmp = NULL;
	char *result = NULL;

	if ((!pcContent) || (*pcContent == NULL))
		return -1;

	if (**pcContent == '-') {
		fgIsNegtive = TRUE;
		*pcContent = *pcContent + 1;
	}
	pcTmp = *pcContent;

	result = kalStrSep(pcContent, delim);
	if (result == NULL) {
		return -1;
	} else if ((result != NULL) && (kalStrLen(result) == 0)) {
		if (fgIsNegtive)
			return -1;
		*value = 0;
		*op = 0;
	} else {
		if (kalkStrtou8(pcTmp, 0, value) != 0) {
			DBGLOG(RLM, ERROR,
			       "parse number error: invalid number [%s]\n",
			       pcTmp);
			return -1;
		}
		if (fgIsNegtive)
			*op = 2;
		else
			*op = 1;
	}

	return 0;
}

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
void txPwrParseTagDump(struct TX_PWR_CTRL_ELEMENT *pRecord)
{
	uint32_t i = 0, j = 0;

	for (i = 0; i < POWER_ANT_TAG_NUM; i++) {
		DBGLOG(RLM, TRACE, "Tag id (%d) :", i);
		for (j = 0; j < POWER_ANT_NUM; j++) {
			DBGLOG(RLM, TRACE, "[%d]",
				pRecord->aiPwrAnt[i].aiPwrAnt2G4[j]);
			DBGLOG(RLM, TRACE, "[%d]",
				pRecord->aiPwrAnt[i].aiPwrAnt5GB1[j]);
			DBGLOG(RLM, TRACE, "[%d]",
				pRecord->aiPwrAnt[i].aiPwrAnt5GB2[j]);
			DBGLOG(RLM, TRACE, "[%d]",
				pRecord->aiPwrAnt[i].aiPwrAnt5GB3[j]);
			DBGLOG(RLM, TRACE, "[%d]",
				pRecord->aiPwrAnt[i].aiPwrAnt5GB4[j]);
		}
	}
}

int32_t txPwrParseTagXXXT(
	char *pStart, char *pEnd, uint8_t cTagParaNum,
	struct TX_PWR_CTRL_ELEMENT *pRecord,
	enum ENUM_POWER_ANT_TAG eTag) {

	char *pCurent = NULL, *pContent = NULL;
	uint8_t i = 0, j = 0;
	uint8_t ucBandIdx = 0, ucAntIdx = 0;
	uint8_t op = 0, value = 0;
	int8_t backoff = 0;

	if (!pStart || !pEnd || !pRecord)
		return -1;

	if (cTagParaNum != (POWER_ANT_BAND_NUM * POWER_ANT_NUM))
		return -1;

	DBGLOG(RLM, TRACE, "pase tag Para (%s) to aiPwrAnt[%u]",
		pStart, eTag);

	pCurent = pStart;

	for (i = 0; i < cTagParaNum; i++) {

		if (!pCurent || pCurent >= pEnd)
			break;

		pContent = txPwrGetString(&pCurent, ",");

		if (!pContent) {
			DBGLOG(RLM, TRACE, "tag parameter format error: %s\n",
			       pStart);
			break;
		}

		if (txPwrParseNumber(&pContent, ",", &op, &value)) {
			DBGLOG(RLM, TRACE, "parse parameter error: %s\n",
			       pContent);
			break;
		}

		backoff = (op == 1) ? value : (0 - value);
		if (backoff < PWR_CFG_BACKOFF_MIN)
			backoff = PWR_CFG_BACKOFF_MIN;

		if (backoff > PWR_CFG_BACKOFF_MAX)
			backoff = PWR_CFG_BACKOFF_MAX;

		/* (wf02g, wf05g,  wf12g, wf15g, ...  wfx2g, wfx5g)
		 * i is parameter index, start from 0.
		 * i % POWER_ANT_BAND_NUM : 0 for 5g  1 for 2g
		 * i / POWER_ANT_BAND_NUM : is wf idx,  wfx
		 *
		 * for example:
		 * i = 3, it means the fourth parameter.
		 * i%POWER_ANT_BAND_NUM is 1 , so means 2g
		 * i/POWER_ANT_BAND_NUM is 1, so means wf1
		 * the i parameter is wf12g.
		 */
		ucBandIdx = i%POWER_ANT_BAND_NUM;
		ucAntIdx = i/POWER_ANT_BAND_NUM;
		switch (ucBandIdx) {
		case POWER_ANT_2G4_BAND:
			pRecord->aiPwrAnt[eTag].aiPwrAnt2G4[ucAntIdx]
				= backoff;
			break;
		case POWER_ANT_5G_BAND1:
			pRecord->aiPwrAnt[eTag].aiPwrAnt5GB1[ucAntIdx]
				= backoff;
			break;
		case POWER_ANT_5G_BAND2:
			pRecord->aiPwrAnt[eTag].aiPwrAnt5GB2[ucAntIdx]
				= backoff;
			break;
		case POWER_ANT_5G_BAND3:
			pRecord->aiPwrAnt[eTag].aiPwrAnt5GB3[ucAntIdx]
				= backoff;
			break;
		case POWER_ANT_5G_BAND4:
			pRecord->aiPwrAnt[eTag].aiPwrAnt5GB4[ucAntIdx]
				= backoff;
			break;
		default:
			DBGLOG(RLM, INFO, "Never happen: %s\n",
				pStart);
			return -1;
		}

		if (pCurent >= pEnd)
			break;

	}

	if (i != cTagParaNum) {
		DBGLOG(RLM, INFO, "parameter number error: %s\n",
			       pStart);
		for (j = 0; j < POWER_ANT_NUM; j++) {
			pRecord->aiPwrAnt[eTag].aiPwrAnt2G4[j] = 0;
			pRecord->aiPwrAnt[eTag].aiPwrAnt5GB1[j] = 0;
			pRecord->aiPwrAnt[eTag].aiPwrAnt5GB2[j] = 0;
			pRecord->aiPwrAnt[eTag].aiPwrAnt5GB3[j] = 0;
			pRecord->aiPwrAnt[eTag].aiPwrAnt5GB4[j] = 0;
		}
		return -1;
	}

	DBGLOG(RLM, TRACE, "[Success] Dump aiPwrAnt[%u] para: ", eTag);
	for (j = 0; j < POWER_ANT_NUM; j++)
		DBGLOG(RLM, TRACE, "[%d][%d][%d][%d][%d]",
			       pRecord->aiPwrAnt[eTag].aiPwrAnt2G4[j],
			       pRecord->aiPwrAnt[eTag].aiPwrAnt5GB1[j],
			       pRecord->aiPwrAnt[eTag].aiPwrAnt5GB2[j],
			       pRecord->aiPwrAnt[eTag].aiPwrAnt5GB3[j],
			       pRecord->aiPwrAnt[eTag].aiPwrAnt5GB4[j]);
	DBGLOG(RLM, TRACE, "\n");

	return 0;
}

int32_t txPwrParseTagMimo1T(
	char *pStart, char *pEnd, uint8_t cTagParaNum,
	struct TX_PWR_CTRL_ELEMENT *pRecord) {
	return txPwrParseTagXXXT(pStart, pEnd,
		cTagParaNum, pRecord, POWER_ANT_MIMO_1T);
}

int32_t txPwrParseTagMimo2T(
	char *pStart, char *pEnd, uint8_t cTagParaNum,
	struct TX_PWR_CTRL_ELEMENT *pRecord){
	return txPwrParseTagXXXT(pStart, pEnd,
		cTagParaNum, pRecord, POWER_ANT_MIMO_2T);
}

int32_t txPwrParseTagAllT(
	char *pStart, char *pEnd, uint8_t cTagParaNum,
	struct TX_PWR_CTRL_ELEMENT *pRecord){
	return txPwrParseTagXXXT(pStart, pEnd,
		cTagParaNum, pRecord, POWER_ANT_ALL_T);
}

int32_t txPwrParseTagAllT6G(
	char *pStart, char *pEnd, uint8_t cTagParaNum,
	struct TX_PWR_CTRL_ELEMENT *pRecord){

	char *pCurent = NULL, *pContent = NULL;
	uint8_t i = 0, j = 0;
	uint8_t ucBandIdx = 0, ucAntIdx = 0;
	uint8_t op = 0, value = 0;
	int8_t backoff = 0;
	enum ENUM_POWER_ANT_TAG eTag = POWER_ANT_ALL_T_6G;

	if (!pStart || !pEnd || !pRecord)
		return -1;

	if (cTagParaNum != (POWER_ANT_6G_BAND_NUM * POWER_ANT_NUM))
		return -1;

	DBGLOG(RLM, TRACE, "pase tag Para (%s) to aiPwrAnt[%u]",
		pStart, eTag);

	pCurent = pStart;

	for (i = 0; i < cTagParaNum; i++) {

		if (!pCurent || pCurent >= pEnd)
			break;

		pContent = txPwrGetString(&pCurent, ",");

		if (!pContent) {
			DBGLOG(RLM, TRACE, "tag parameter format error: %s\n",
			       pStart);
			break;
		}

		if (txPwrParseNumber(&pContent, ",", &op, &value)) {
			DBGLOG(RLM, TRACE, "parse parameter error: %s\n",
			       pContent);
			break;
		}

		backoff = (op == 1) ? value : (0 - value);
		if (backoff < PWR_CFG_BACKOFF_MIN)
			backoff = PWR_CFG_BACKOFF_MIN;

		if (backoff > PWR_CFG_BACKOFF_MAX)
			backoff = PWR_CFG_BACKOFF_MAX;

		ucBandIdx = i%POWER_ANT_6G_BAND_NUM;
		ucAntIdx = i/POWER_ANT_6G_BAND_NUM;
		switch (ucBandIdx) {
		case POWER_ANT_6G_BAND1:
			pRecord->aiPwrAnt[eTag].aiPwrAnt6GB1[ucAntIdx]
				= backoff;
			break;
		case POWER_ANT_6G_BAND2:
			pRecord->aiPwrAnt[eTag].aiPwrAnt6GB2[ucAntIdx]
				= backoff;
			break;
		case POWER_ANT_6G_BAND3:
			pRecord->aiPwrAnt[eTag].aiPwrAnt6GB3[ucAntIdx]
				= backoff;
			break;
		case POWER_ANT_6G_BAND4:
			pRecord->aiPwrAnt[eTag].aiPwrAnt6GB4[ucAntIdx]
				= backoff;
			break;
		default:
			DBGLOG(RLM, INFO, "Never happen: %s\n",
				pStart);
			return -1;
		}

		if (pCurent >= pEnd)
			break;

	}

	if (i != cTagParaNum) {
		DBGLOG(RLM, INFO, "parameter number error: %s\n",
			       pStart);
		for (j = 0; j < POWER_ANT_NUM; j++) {
			pRecord->aiPwrAnt[eTag].aiPwrAnt6GB1[j] = 0;
			pRecord->aiPwrAnt[eTag].aiPwrAnt6GB2[j] = 0;
			pRecord->aiPwrAnt[eTag].aiPwrAnt6GB3[j] = 0;
			pRecord->aiPwrAnt[eTag].aiPwrAnt6GB4[j] = 0;
		}
		return -1;
	}

	DBGLOG(RLM, TRACE, "[Success] Dump aiPwrAnt[%u] para: ", eTag);
	for (j = 0; j < POWER_ANT_NUM; j++)
		DBGLOG(RLM, TRACE, "[%d][%d][%d][%d]",
			       pRecord->aiPwrAnt[eTag].aiPwrAnt6GB1[j],
			       pRecord->aiPwrAnt[eTag].aiPwrAnt6GB2[j],
			       pRecord->aiPwrAnt[eTag].aiPwrAnt6GB3[j],
			       pRecord->aiPwrAnt[eTag].aiPwrAnt6GB4[j]);
	DBGLOG(RLM, TRACE, "\n");

	return 0;
}

int32_t txPwrParseTag(char *pTagStart, char *pTagEnd,
	struct TX_PWR_CTRL_ELEMENT *pRecord) {
	uint8_t i = 0;
	int32_t ret = 0;
	char *pCurent = NULL, *pNext = NULL;

	if (!pTagStart || !pTagEnd || !pRecord)
		return -1;

	DBGLOG(RLM, TRACE, "Pase new tag %s\n", pTagStart);

	pCurent = pTagStart;

	pNext = txPwrGetString(&pCurent, ",");
	if (!pNext) {
		DBGLOG(RLM, INFO,
			   "Pase tag name error, %s\n",
			   pTagStart);
		return -1;
	}

	for (i = 0;
		i < sizeof(g_auTxPwrTagTable)/sizeof(struct TX_PWR_TAG_TABLE);
		i++){

		DBGLOG(RLM, TRACE,
				"Parse tag name [%s] handler name[%s]\n", pNext,
				g_auTxPwrTagTable[i].arTagNames);

		if (kalStrCmp(pNext,
			g_auTxPwrTagTable[i].arTagNames) == 0){

			if (g_auTxPwrTagTable[i].pfnParseTagParaHandler
				== NULL) {
				DBGLOG(RLM, TRACE,
			       "No parse handler: tag name [%s]\n",
			       pNext);
				return -1;
			}
			ret = g_auTxPwrTagTable[i].pfnParseTagParaHandler(
				pCurent, pTagEnd,
				g_auTxPwrTagTable[i].ucTagParaNum,
				pRecord);

			txPwrParseTagDump(pRecord);

			return ret;
		}
	}

	DBGLOG(RLM, INFO,
		"Undefined tag name [%s]\n",
		pCurent);

	return -1;
}

int32_t txPwrOnPreParseAppendTag(
	struct TX_PWR_CTRL_ELEMENT *pRecord) {

	if (!pRecord)
		return -1;

	/* init aiPwrAnt to 0 for tag :
	  * MIMO_1T/MIMO_2T/ALL_T
	  */
	kalMemSet(&(pRecord->aiPwrAnt[0]), 0,
		POWER_ANT_TAG_NUM *
		sizeof(struct TX_PWR_CTRL_ANT_SETTING));

	/* Init pRecord for your new tag*/

	return 0;
}


int32_t txPwrParseAppendTag(char *pcStart,
	char *pcEnd, struct TX_PWR_CTRL_ELEMENT *pRecord) {

	char *pcContCur = NULL, *pcContTemp = NULL;
	uint32_t ucTagCount1 = 0, ucTagCount2 = 0;
	uint8_t i = 0;

	if (!pcStart || !pcEnd || !pRecord) {
		DBGLOG(RLM, INFO, "%p-%p-%p!\n",
			pcStart, pcEnd, pRecord);
		return -1;
	}

	/* parse power limit append tag to list. */
	pcContCur = pcStart;

	pcContTemp = pcContCur;

	while (pcContTemp <= pcEnd) {
		if ((*pcContTemp) == '<')
			ucTagCount1++;

		if ((*pcContTemp) == '>')
			ucTagCount2++;
		pcContTemp++;
	}

	if (ucTagCount1 != ucTagCount2) {
		DBGLOG(RLM, INFO,
		       "Wrong tag append %s!\n", pcStart);
		return -1;
	}

	if (ucTagCount1 == 0)
		return 0;

	DBGLOG(RLM, TRACE, "New config total %u tag append %s !\n",
		ucTagCount1, pcStart);

	for (i = 0; i < ucTagCount1; i++) {

		pcContTemp = txPwrGetString(&pcContCur, "<");

		if (!pcContCur || pcContCur > pcEnd) {
			DBGLOG(RLM, INFO,
		       "No content after '<' !\n");
			return -1;
		}

		/* verify there is > symbol */
		pcContTemp = txPwrGetString(&pcContCur, ">");
		if (!pcContTemp) {
			DBGLOG(RLM, INFO,
		       "Can not find content after '<' !\n");
			return -1;
		}

		if (!pcContCur || pcContCur > pcEnd) {
			DBGLOG(RLM, INFO,
			   "No content after '>' !\n");
			return -1;
		}

		if (txPwrParseTag(pcContTemp, pcContCur - 1, pRecord)) {
			DBGLOG(RLM, INFO,
			   "Parse one tag fail !\n");
			return -1;
		}

		DBGLOG(RLM, TRACE,
			   "Pase tag (%u/%u) success!\n", i + 1, ucTagCount1);
	}

	return 0;
}
#endif
void txPwrOperate(enum ENUM_TX_POWER_CTRL_TYPE eCtrlType,
		  int8_t *operand1, int8_t *operand2)
{
	switch (eCtrlType) {
	case PWR_CTRL_TYPE_WIFION_POWER_LEVEL:
	case PWR_CTRL_TYPE_IOCTL_POWER_LEVEL:
		if (*operand1 > *operand2)
			*operand1 = *operand2;
		break;
	case PWR_CTRL_TYPE_WIFION_POWER_OFFSET:
	case PWR_CTRL_TYPE_IOCTL_POWER_OFFSET:
		*operand1 += *operand2;
		break;
	default:
		break;
	}

	if (*operand1 > MAX_TX_POWER)
		*operand1 = MAX_TX_POWER;
	else if (*operand1 < MIN_TX_POWER)
		*operand1 = MIN_TX_POWER;
}

uint32_t txPwrArbitrator(enum ENUM_TX_POWER_CTRL_TYPE eCtrlType,
			 void *pvBuf,
			 struct TX_PWR_CTRL_CHANNEL_SETTING *prChlSetting,
			 enum ENUM_PWR_LIMIT_TYPE eType)
{
	struct CMD_CHANNEL_POWER_LIMIT *prPwrLimit;
	struct CMD_CHANNEL_POWER_LIMIT_HE *prPwrLimitHE;
	struct CMD_CHANNEL_POWER_LIMIT_HE_BW160 *prPwrLimitHEBW160;
	uint8_t rateIdxHe, rateIdx;
#if (CFG_SUPPORT_WIFI_6G == 1)
	struct CMD_CHANNEL_POWER_LIMIT_6E *prPwrLimit6E;
#endif
	int8_t *prRateOfs;

	if (eType == PWR_LIMIT_TYPE_COMP_11AX) {
		prPwrLimitHE = (struct CMD_CHANNEL_POWER_LIMIT_HE *) pvBuf;
		prRateOfs = &prPwrLimitHE->cPwrLimitRU26L;

		for (rateIdxHe = PWR_LIMIT_RU26_L;
			rateIdxHe < PWR_LIMIT_HE_NUM ; rateIdxHe++) {
			if (prChlSetting->opHE[rateIdxHe]
				!= PWR_CTRL_TYPE_NO_ACTION) {
				txPwrOperate(eCtrlType, prRateOfs + rateIdxHe,
				&prChlSetting->i8PwrLimitHE[rateIdxHe]);
			}
		}
	} else if (eType == PWR_LIMIT_TYPE_COMP_11AX_BW160) {
		prPwrLimitHEBW160 =
			(struct CMD_CHANNEL_POWER_LIMIT_HE_BW160 *) pvBuf;
		prRateOfs = &prPwrLimitHEBW160->cPwrLimitRU26L;

		for (rateIdxHe = PWR_LIMIT_RU26_L;
			rateIdxHe < PWR_LIMIT_HE_BW160_NUM ; rateIdxHe++) {
			if (prChlSetting->opHE[rateIdxHe]
				!= PWR_CTRL_TYPE_NO_ACTION) {
				txPwrOperate(eCtrlType, prRateOfs + rateIdxHe,
				&prChlSetting->i8PwrLimitHE[rateIdxHe]);
			}
		}
	}
#if (CFG_SUPPORT_WIFI_6G == 1)
	else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
				eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
				eType == PWR_LIMIT_TYPE_COMP_6E_3) {
		prPwrLimit6E = (struct CMD_CHANNEL_POWER_LIMIT_6E *) pvBuf;
		prRateOfs = &prPwrLimit6E->cPwrLimitRU26L;
		for (rateIdx = PWR_LIMIT_RU26_L;
			rateIdx < PWR_LIMIT_6E_NUM ; rateIdx++) {
			if (prChlSetting->op6E[rateIdx]
				!= PWR_CTRL_TYPE_NO_ACTION) {
				txPwrOperate(eCtrlType, prRateOfs + rateIdx,
					&prChlSetting->i8PwrLimit6E[rateIdx]);
			}
		}
	}
#endif
	else {
		prPwrLimit = (struct CMD_CHANNEL_POWER_LIMIT *) pvBuf;
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
		prRateOfs = &prPwrLimit->cPwrLimitCCK_L;
		for (rateIdx = PWR_LIMIT_CCK_L;
#else
		prRateOfs = &prPwrLimit->cPwrLimitCCK;

		for (rateIdx = PWR_LIMIT_CCK;
#endif
			rateIdx < PWR_LIMIT_NUM ; rateIdx++) {
			if (prChlSetting->op[rateIdx]
				!= PWR_CTRL_TYPE_NO_ACTION) {
				txPwrOperate(eCtrlType, prRateOfs + rateIdx,
				&prChlSetting->i8PwrLimit[rateIdx]);
			}
		}

	}

	return 0;
}

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
uint8_t txPwrIsAntTagSet(
	struct TX_PWR_CTRL_ELEMENT *prCurElement,
	enum ENUM_POWER_ANT_TAG tag) {
	uint8_t i = 0;

	if (tag >= POWER_ANT_TAG_NUM)
		return 0;
	for (i = 0; i < POWER_ANT_NUM; i++) {
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt2G4[i] != 0)
			return 1;
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt5GB1[i] != 0)
			return 1;
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt5GB2[i] != 0)
			return 1;
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt5GB3[i] != 0)
			return 1;
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt5GB4[i] != 0)
			return 1;
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt6GB1[i] != 0)
			return 1;
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt6GB2[i] != 0)
			return 1;
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt6GB3[i] != 0)
			return 1;
		if (prCurElement->aiPwrAnt[tag].aiPwrAnt6GB4[i] != 0)
			return 1;
	}
	return 0;
}

uint8_t txPwrIsAntTagNeedApply(
	struct TX_PWR_CTRL_ELEMENT *prCurElement) {
	uint8_t fgAllTSet = 0, fg1TSet = 0, fg2TSet = 0;

	if (!prCurElement)
		return 0;

	fgAllTSet = txPwrIsAntTagSet(prCurElement, POWER_ANT_ALL_T);
	fg1TSet = txPwrIsAntTagSet(prCurElement, POWER_ANT_MIMO_1T);
	fg2TSet = txPwrIsAntTagSet(prCurElement, POWER_ANT_MIMO_2T);

	if ((fgAllTSet && !fg1TSet && !fg2TSet)
		|| (!fgAllTSet && (fg1TSet || fg2TSet)))
		/* only MIMO_1T or MIMO_2T */
		return 1;

	DBGLOG(RLM, TRACE, "No need apply [%u-%u-%u]\n",
		fgAllTSet, fg1TSet, fg2TSet);

	return 0;
}

uint32_t txPwrApplyOneSettingPwrAnt(
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd,
	struct TX_PWR_CTRL_ELEMENT *prCurElement) {

	struct CMD_CHANNEL_POWER_LIMIT_ANT *prCmdPwrAnt = NULL;
	uint8_t i = 0, j = 0, k = 0;
	uint8_t fgAllTSet = 0, fg1TSet = 0, fg2TSet = 0;
	uint8_t fgAllT6GSet = 0;
	uint8_t tagIdx = 0;

	enum ENUM_PWR_LIMIT_TYPE eType;

	if (!prCurElement || !prCmd)
		return 1;

	eType = prCmd->ucLimitType;

	if (eType != PWR_LIMIT_TYPE_COMP_ANT_V2)
		return 0;

	prCmdPwrAnt = &(prCmd->u.rChPwrLimtAnt[0]);

	fgAllTSet = txPwrIsAntTagSet(prCurElement, POWER_ANT_ALL_T);
	fg1TSet = txPwrIsAntTagSet(prCurElement, POWER_ANT_MIMO_1T);
	fg2TSet = txPwrIsAntTagSet(prCurElement, POWER_ANT_MIMO_2T);
	fgAllT6GSet = txPwrIsAntTagSet(prCurElement, POWER_ANT_ALL_T_6G);

	/* check scenario reasonable */
	if (!(((fgAllTSet || fgAllT6GSet) && !fg1TSet && !fg2TSet)
		|| (!(fgAllTSet || fgAllT6GSet) && (fg1TSet || fg2TSet)))) {
		return 0;
	}
	/* TODO : refactory when fg1TSet/fg2TSet online */
	prCmd->ucNum = 0;

	if (fgAllTSet) {
		tagIdx = POWER_ANT_ALL_T;
		for (j = 0; j < POWER_ANT_BAND_NUM; j++) {
			for (k = 0; k < POWER_ANT_NUM; k++) {
				prCmdPwrAnt[i].cTagIdx = tagIdx;
				prCmdPwrAnt[i].cBandIdx = j;
				prCmdPwrAnt[i].cAntIdx = k;

				if (j == POWER_ANT_2G4_BAND) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T]
						.aiPwrAnt2G4[k]) {
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T]
							.aiPwrAnt2G4[k];
					}
				} else if (j == POWER_ANT_5G_BAND1) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T]
						.aiPwrAnt5GB1[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T]
							.aiPwrAnt5GB1[k];
				} else if (j == POWER_ANT_5G_BAND2) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T]
						.aiPwrAnt5GB2[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T]
							.aiPwrAnt5GB2[k];
				} else if (j == POWER_ANT_5G_BAND3) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T]
						.aiPwrAnt5GB3[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T]
							.aiPwrAnt5GB3[k];
				} else if (j == POWER_ANT_5G_BAND4) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T]
						.aiPwrAnt5GB4[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T]
							.aiPwrAnt5GB4[k];
				}
				i++;
			}
		}
	}

	if (fg1TSet) {
		tagIdx = POWER_ANT_MIMO_1T;
		for (j = 0; j < POWER_ANT_BAND_NUM; j++) {
			for (k = 0; k < POWER_ANT_NUM; k++) {
				prCmdPwrAnt[i].cTagIdx = tagIdx;
				prCmdPwrAnt[i].cBandIdx = j;
				prCmdPwrAnt[i].cAntIdx = k;

				if (j == POWER_ANT_2G4_BAND) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_1T]
						.aiPwrAnt2G4[k]) {
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_1T]
							.aiPwrAnt2G4[k];
					}
				} else if (j == POWER_ANT_5G_BAND1) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_1T]
						.aiPwrAnt5GB1[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_1T]
							.aiPwrAnt5GB1[k];
				} else if (j == POWER_ANT_5G_BAND2) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_1T]
						.aiPwrAnt5GB2[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_1T]
							.aiPwrAnt5GB2[k];
				} else if (j == POWER_ANT_5G_BAND3) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_1T]
						.aiPwrAnt5GB3[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_1T]
							.aiPwrAnt5GB3[k];
				} else if (j == POWER_ANT_5G_BAND4) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_1T]
						.aiPwrAnt5GB4[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_1T]
							.aiPwrAnt5GB4[k];
				}
				i++;
			}
		}
	}

	if (fg2TSet) {
		tagIdx = POWER_ANT_MIMO_2T;
		for (j = 0; j < POWER_ANT_BAND_NUM; j++) {
			for (k = 0; k < POWER_ANT_NUM; k++) {
				prCmdPwrAnt[i].cTagIdx = tagIdx;
				prCmdPwrAnt[i].cBandIdx = j;
				prCmdPwrAnt[i].cAntIdx = k;

				if (j == POWER_ANT_2G4_BAND) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_2T]
						.aiPwrAnt2G4[k]) {
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_2T]
							.aiPwrAnt2G4[k];
					}
				} else if (j == POWER_ANT_5G_BAND1) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_2T]
						.aiPwrAnt5GB1[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_2T]
							.aiPwrAnt5GB1[k];
				} else if (j == POWER_ANT_5G_BAND2) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_2T]
						.aiPwrAnt5GB2[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_2T]
							.aiPwrAnt5GB2[k];
				} else if (j == POWER_ANT_5G_BAND3) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_2T]
						.aiPwrAnt5GB3[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_2T]
							.aiPwrAnt5GB3[k];
				} else if (j == POWER_ANT_5G_BAND4) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_MIMO_2T]
						.aiPwrAnt5GB4[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_MIMO_2T]
							.aiPwrAnt5GB4[k];
				}
				i++;
			}
		}
	}

	if (fgAllT6GSet) {
		tagIdx = POWER_ANT_ALL_T_6G;
		for (j = 0; j < POWER_ANT_6G_BAND_NUM; j++) {
			for (k = 0; k < POWER_ANT_NUM; k++) {
				prCmdPwrAnt[i].cTagIdx = tagIdx;
				prCmdPwrAnt[i].cBandIdx = j;
				prCmdPwrAnt[i].cAntIdx = k;

				if (j == POWER_ANT_6G_BAND1) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T_6G]
						.aiPwrAnt6GB1[k]) {
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T_6G]
							.aiPwrAnt6GB1[k];
					}
				} else if (j == POWER_ANT_6G_BAND2) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T_6G]
						.aiPwrAnt6GB2[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T_6G]
							.aiPwrAnt6GB2[k];
				} else if (j == POWER_ANT_6G_BAND3) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T_6G]
						.aiPwrAnt6GB3[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T_6G]
							.aiPwrAnt6GB3[k];
				} else if (j == POWER_ANT_6G_BAND4) {
					if (prCmdPwrAnt[i].cValue >
						prCurElement->
						aiPwrAnt[POWER_ANT_ALL_T_6G]
						.aiPwrAnt6GB4[k])
						prCmdPwrAnt[i].cValue =
							prCurElement->
							aiPwrAnt
							[POWER_ANT_ALL_T_6G]
							.aiPwrAnt6GB4[k];
				}
				i++;
			}
		}
	}

	prCmd->ucNum = i;
	return 0;
}
#endif
uint32_t txPwrApplyOneSetting(struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd,
			      struct TX_PWR_CTRL_ELEMENT *prCurElement,
			      uint8_t *bandedgeParam)
{
	struct CMD_CHANNEL_POWER_LIMIT *prCmdPwrLimit = NULL;
	struct CMD_CHANNEL_POWER_LIMIT_HE *prCmdPwrLimitHE = NULL;
	struct CMD_CHANNEL_POWER_LIMIT_HE_BW160 *prCmdPwrLimitHEBW160 = NULL;
#if (CFG_SUPPORT_WIFI_6G == 1)
	struct CMD_CHANNEL_POWER_LIMIT_6E *prCmdPwrLimit6E = NULL;
	u_int8_t fgDoArbitrator6E;
#endif
	struct TX_PWR_CTRL_CHANNEL_SETTING *prChlSetting = NULL;
	uint8_t i, j, channel, channel2, channel3;
	u_int8_t fgDoArbitrator;
	enum ENUM_PWR_LIMIT_TYPE eType;

	ASSERT(prCmd);

	eType = prCmd->ucLimitType;

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	if (eType == PWR_LIMIT_TYPE_COMP_ANT_V2) {
		txPwrApplyOneSettingPwrAnt(prCmd, prCurElement);
		return 0;
	}
#endif

	for (i = 0; i < prCmd->ucNum; i++) {

		if (eType == PWR_LIMIT_TYPE_COMP_11AX) {
			prCmdPwrLimitHE = &prCmd->u.rChPwrLimtHE[i];
			channel = prCmdPwrLimitHE->ucCentralCh;
		} else if (eType == PWR_LIMIT_TYPE_COMP_11AX_BW160) {
			prCmdPwrLimitHEBW160 = &prCmd->u.rChPwrLimtHEBW160[i];
			channel = prCmdPwrLimitHEBW160->ucCentralCh;
		}
#if (CFG_SUPPORT_WIFI_6G == 1)
		else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3) {
			prCmdPwrLimit6E = &prCmd->u.rChPwrLimt6E[i];
			channel = prCmdPwrLimit6E->ucCentralCh;
		}
#endif
		else {
			prCmdPwrLimit = &prCmd->u.rChannelPowerLimit[i];
			channel = prCmdPwrLimit->ucCentralCh;
		}



		for (j = 0; j < prCurElement->settingCount; j++) {
			prChlSetting = &prCurElement->rChlSettingList[j];
			channel2 = prChlSetting->channelParam[0];
			channel3 = prChlSetting->channelParam[1];
			fgDoArbitrator = FALSE;
#if (CFG_SUPPORT_WIFI_6G == 1)
			fgDoArbitrator6E = FALSE;
#endif
			switch (prChlSetting->eChnlType) {
				case PWR_CTRL_CHNL_TYPE_NORMAL: {
					if (channel == channel2)
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_ALL: {
					fgDoArbitrator = TRUE;
#if (CFG_SUPPORT_WIFI_6G == 1)
					fgDoArbitrator6E = TRUE;
#endif
					break;
				}
				case PWR_CTRL_CHNL_TYPE_RANGE: {
					if ((channel >= channel2) &&
					    (channel <= channel3))
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_2G4: {
					if (channel <= 14)
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_5G: {
					if (channel > 14)
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_BANDEDGE_2G4: {
					if ((channel == *bandedgeParam) ||
					    (channel == *(bandedgeParam + 1)))
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_BANDEDGE_5G: {
					if ((channel == *(bandedgeParam + 2)) ||
					    (channel == *(bandedgeParam + 3)))
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_5G_BAND1: {
					if ((channel >=
						g_rRlmSubBand[1].ucStartCh) &&
						(channel <=
						g_rRlmSubBand[1].ucEndCh))
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_5G_BAND2: {
					if ((channel >=
						g_rRlmSubBand[2].ucStartCh) &&
						(channel <=
						g_rRlmSubBand[2].ucEndCh))
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_5G_BAND3: {
					if ((channel >=
						g_rRlmSubBand[3].ucStartCh) &&
						(channel <=
						g_rRlmSubBand[3].ucEndCh))
						fgDoArbitrator = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_5G_BAND4: {
					if ((channel >=
						g_rRlmSubBand[4].ucStartCh) &&
						(channel <=
						g_rRlmSubBand[4].ucEndCh))
						fgDoArbitrator = TRUE;
					break;
				}
#if (CFG_SUPPORT_WIFI_6G == 1)
				case PWR_CTRL_CHNL_TYPE_6G: {
					fgDoArbitrator6E = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_6G_NORMAL: {
					if (channel == channel2)
						fgDoArbitrator6E = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_6G_BAND1: {
					if ((channel >=
						g_rRlmSubBand[5].ucStartCh) &&
						(channel <=
						g_rRlmSubBand[5].ucEndCh))
						fgDoArbitrator6E = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_6G_BAND2: {
					if ((channel >=
						g_rRlmSubBand[6].ucStartCh) &&
						(channel <=
						g_rRlmSubBand[6].ucEndCh))
						fgDoArbitrator6E = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_6G_BAND3: {
					if ((channel >=
						g_rRlmSubBand[7].ucStartCh) &&
						(channel <=
						g_rRlmSubBand[7].ucEndCh))
						fgDoArbitrator6E = TRUE;
					break;
				}
				case PWR_CTRL_CHNL_TYPE_6G_BAND4: {
					if ((channel >=
						g_rRlmSubBand[8].ucStartCh) &&
						(channel <=
						g_rRlmSubBand[8].ucEndCh))
						fgDoArbitrator6E = TRUE;
					break;
				}
#endif
			}

			if (fgDoArbitrator) {
				if (eType == PWR_LIMIT_TYPE_COMP_11AX)
					txPwrArbitrator(prCurElement->eCtrlType,
							prCmdPwrLimitHE,
							prChlSetting,
							eType);
				else if (eType ==
					PWR_LIMIT_TYPE_COMP_11AX_BW160)
					txPwrArbitrator(prCurElement->eCtrlType,
							prCmdPwrLimitHEBW160,
							prChlSetting,
							eType);
				else if (eType == PWR_LIMIT_TYPE_COMP_11AC ||
					eType == PWR_LIMIT_TYPE_COMP_11AC_V2)
					txPwrArbitrator(prCurElement->eCtrlType,
							prCmdPwrLimit,
							prChlSetting,
							eType);
			}

#if (CFG_SUPPORT_WIFI_6G == 1)
			if (fgDoArbitrator6E) {
				if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3)
					txPwrArbitrator(prCurElement->eCtrlType,
							prCmdPwrLimit6E,
							prChlSetting,
							eType);
			}
#endif
		}

	}
	return 0;
}

uint32_t txPwrCtrlApplySettings(struct ADAPTER *prAdapter,
			struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd,
			uint8_t *bandedgeParam)
{
	struct LINK_ENTRY *prCur, *prNext;
	struct TX_PWR_CTRL_ELEMENT *element = NULL;
	struct LINK *aryprlist[2] = {
		&prAdapter->rTxPwr_DefaultList,
		&prAdapter->rTxPwr_DynamicList
	};
	int32_t i;

	/* show the tx power ctrl applied list */
	txPwrCtrlShowList(prAdapter, 1, "applied list");

	for (i = 0; i < ARRAY_SIZE(aryprlist); i++) {
		LINK_FOR_EACH_SAFE(prCur, prNext, aryprlist[i]) {
			element = LINK_ENTRY(prCur,
					struct TX_PWR_CTRL_ELEMENT, node);
			if (element->fgApplied == TRUE)
				txPwrApplyOneSetting(
					prCmd, element, bandedgeParam);
		}
	}

	return 0;
}

struct TX_PWR_CTRL_ELEMENT *txPwrCtrlStringToStruct(char *pcContent,
						    u_int8_t fgSkipHeader)
{
	struct TX_PWR_CTRL_ELEMENT *prCurElement = NULL;
	struct TX_PWR_CTRL_CHANNEL_SETTING *prTmpSetting;
	char acTmpName[MAX_TX_PWR_CTRL_ELEMENT_NAME_SIZE];
	char *pcContCur = NULL, *pcContCur2 = NULL, *pcContEnd = NULL;
	char *pcContTmp = NULL, *pcContNext = NULL, *pcContOld = NULL;
	uint32_t u4MemSize = sizeof(struct TX_PWR_CTRL_ELEMENT);
	uint32_t copySize = 0;
	uint8_t i, j, op, ucSettingCount = 0;
	uint8_t value, value2, count = 0;
	uint8_t ucAppliedWay, ucOperation = 0;
	char carySeperator[2] = { 0, 0 };

	char *pacParsePwrAC[PWR_CFG_PRAM_NUM_AC] = {
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
		"CCKL",
		"CCKH",
		"OFDML",
		"OFDMH",
#else
		"CCK",
#endif
		"HT20L",
		"HT20H",
		"HT40L",
		"HT40H",
		"HT80L",
		"HT80H",
		"HT160L",
		"HT160H"
		};
	char *pacParsePwrAX[PWR_LIMIT_HE_BW160_NUM] = {
		"RU26L",
		"RU26H",
		"RU26U",
		"RU52L",
		"RU52H",
		"RU52U",
		"RU106L",
		"RU106H",
		"RU106U",
		"RU242L",
		"RU242H",
		"RU242U",
		"RU484L",
		"RU484H",
		"RU484U",
		"RU996L",
		"RU996H",
		"RU996U",
		"RU1992L",
		"RU1992H",
		"RU1992U"
		};

	if (!pcContent) {
		DBGLOG(RLM, ERROR, "pcContent is null\n");
		return NULL;
	}

	pcContCur = pcContent;
	pcContEnd = pcContent + kalStrLen(pcContent);

	if (fgSkipHeader == TRUE)
		goto skipLabel;

	/* insert element into prTxPwrCtrlList */
	/* parse scenario name */
	kalMemZero(acTmpName, MAX_TX_PWR_CTRL_ELEMENT_NAME_SIZE);
	pcContOld = pcContCur;
	pcContTmp = txPwrGetString(&pcContCur, ";");
	if (!pcContTmp) {
		DBGLOG(RLM, ERROR, "parse scenario name error: %s\n",
		       pcContOld);
		return NULL;
	}
	copySize = kalStrLen(pcContTmp);
	if (copySize >= MAX_TX_PWR_CTRL_ELEMENT_NAME_SIZE)
		copySize = MAX_TX_PWR_CTRL_ELEMENT_NAME_SIZE - 1;
	kalMemCopy(acTmpName, pcContTmp, copySize);
	acTmpName[copySize] = 0;

	/* parese scenario sub index */
	pcContOld = pcContCur;
	if (txPwrParseNumber(&pcContCur, ";", &op, &value)) {
		DBGLOG(RLM, ERROR, "parse scenario sub index error: %s\n",
		       pcContOld);
		return NULL;
	}
	if ((op != 1) || (value < 0)) {
		DBGLOG(RLM, ERROR,
		       "parse scenario sub index error: op=%u, val=%d\n",
		       op, value);
		return NULL;
	}

	/* parese scenario applied way */
	pcContOld = pcContCur;
	if (txPwrParseNumber(&pcContCur, ";", &op, &ucAppliedWay)) {
		DBGLOG(RLM, ERROR, "parse applied way error: %s\n",
		       pcContOld);
		return NULL;
	}
	if ((ucAppliedWay < PWR_CTRL_TYPE_APPLIED_WAY_WIFION) ||
	    (ucAppliedWay > PWR_CTRL_TYPE_APPLIED_WAY_IOCTL)) {
		DBGLOG(RLM, ERROR,
		       "parse applied way error: value=%u\n",
		       ucAppliedWay);
		return NULL;
	}

	/* parese scenario applied type */
	pcContOld = pcContCur;
	if (txPwrParseNumber(&pcContCur, ";", &op, &ucOperation)) {
		DBGLOG(RLM, ERROR, "parse operation error: %s\n",
		       pcContOld);
		return NULL;
	}
	if ((ucOperation < PWR_CTRL_TYPE_OPERATION_POWER_LEVEL) ||
	    (ucOperation > PWR_CTRL_TYPE_OPERATION_POWER_OFFSET)) {
		DBGLOG(RLM, ERROR,
		       "parse operation error: value=%u\n",
		       ucOperation);
		return NULL;
	}

	switch (ucAppliedWay) {
	case PWR_CTRL_TYPE_APPLIED_WAY_WIFION:
		if (ucOperation == PWR_CTRL_TYPE_OPERATION_POWER_LEVEL)
			value2 = PWR_CTRL_TYPE_WIFION_POWER_LEVEL;
		else
			value2 = PWR_CTRL_TYPE_WIFION_POWER_OFFSET;
		break;
	case PWR_CTRL_TYPE_APPLIED_WAY_IOCTL:
		if (ucOperation == PWR_CTRL_TYPE_OPERATION_POWER_LEVEL)
			value2 = PWR_CTRL_TYPE_IOCTL_POWER_LEVEL;
		else
			value2 = PWR_CTRL_TYPE_IOCTL_POWER_OFFSET;
		break;
	}

skipLabel:
	/* decide how many channel setting */
	pcContOld = pcContCur;
	while (pcContCur <= pcContEnd) {
		if ((*pcContCur) == '[')
			ucSettingCount++;
		pcContCur++;
	}

	if (ucSettingCount == 0) {
		DBGLOG(RLM, ERROR,
		       "power ctrl channel setting is empty\n");
		return NULL;
	}

	/* allocate memory for control element */
	u4MemSize += (ucSettingCount == 1) ? 0 : (ucSettingCount - 1) *
			sizeof(struct TX_PWR_CTRL_CHANNEL_SETTING);
	prCurElement = (struct TX_PWR_CTRL_ELEMENT *)kalMemAlloc(
					u4MemSize, VIR_MEM_TYPE);
	if (!prCurElement) {
		DBGLOG(RLM, ERROR,
		       "alloc power ctrl element failed\n");
		return NULL;
	}

	/* assign values into control element */
	kalMemZero(prCurElement, u4MemSize);
	if (fgSkipHeader == FALSE) {
		kalMemCopy(prCurElement->name, acTmpName, copySize);
		prCurElement->index = (uint8_t)value;
		prCurElement->eCtrlType = (enum ENUM_TX_POWER_CTRL_TYPE)value2;
		if (prCurElement->eCtrlType <=
		    PWR_CTRL_TYPE_WIFION_POWER_OFFSET)
			prCurElement->fgApplied = TRUE;
	}
	prCurElement->settingCount = ucSettingCount;

	/* parse channel setting list */
	pcContCur = pcContOld + 1; /* skip '[' */

	for (i = 0; i < ucSettingCount; i++) {
		if (pcContCur >= pcContEnd) {
			DBGLOG(RLM, ERROR,
			       "parse error: out of bound\n");
			goto clearLabel;
		}

		prTmpSetting = &prCurElement->rChlSettingList[i];

		/* verify there is ] symbol */
		pcContNext = kalStrChr(pcContCur, ']');
		if (!pcContNext) {
			DBGLOG(RLM, ERROR,
			       "parse error: miss symbol ']', %s\n",
			       pcContCur);
			goto clearLabel;
		}

		/* verify this setting has 9 segment */
		pcContTmp = pcContCur;
		count = 0;
		while (pcContTmp < pcContNext) {
			if (*pcContTmp == ',')
				count++;
			pcContTmp++;
		}

		if ((count != PWR_CFG_PRAM_NUM_AC) &&
			(count != PWR_CFG_PRAM_NUM_AX) &&
#if (CFG_SUPPORT_WIFI_6G == 1)
			(count != PWR_LIMIT_6E_NUM) &&
#endif
			(count != PWR_CFG_PRAM_NUM_ALL_RATE) &&
			(count != PWR_LIMIT_HE_BW160_NUM + 1)) {
			DBGLOG(RLM, ERROR,
			       "parse error: not segments, %s\n",
			       pcContCur);
			goto clearLabel;
		}

		/* parse channel setting type */
		pcContOld = pcContCur;
		pcContTmp = txPwrGetString(&pcContCur, ",");
		if (!pcContTmp) {
			DBGLOG(RLM, ERROR,
			       "parse channel setting type error, %s\n",
			       pcContOld);
			goto clearLabel;
		/* "ALL" */
		} else if (kalStrCmp(pcContTmp,
				     PWR_CTRL_CHNL_TYPE_KEY_ALL) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_ALL;
		/* "2G4" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_2G4) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_2G4;
		/* "5G" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_5G) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_5G;
		/* "BANDEDGE2G4" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_BANDEDGE_2G4) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_BANDEDGE_2G4;
		/* "BANDEDGE5G" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_BANDEDGE_5G) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_BANDEDGE_5G;
		/* "5GBAND1" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_5G_BAND1) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_5G_BAND1;
		/* "5GBAND2" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_5G_BAND2) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_5G_BAND2;
		/* "5GBAND3" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_5G_BAND3) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_5G_BAND3;
		/* "5GBAND4" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_5G_BAND4) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_5G_BAND4;
#if (CFG_SUPPORT_WIFI_6G == 1)
		/* "6G" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_6G) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_6G;
		/* "6GBAND1" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_6G_BAND1) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_6G_BAND1;
		/* "5GBAND2" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_6G_BAND2) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_6G_BAND2;
		/* "5GBAND3" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_6G_BAND3) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_6G_BAND3;
		/* "5GBAND4" */
		else if (kalStrCmp(pcContTmp,
				   PWR_CTRL_CHNL_TYPE_KEY_6G_BAND4) == 0)
			prTmpSetting->eChnlType =
				PWR_CTRL_CHNL_TYPE_6G_BAND4;
#endif
		else {
			pcContCur2 = pcContOld;
			pcContTmp = txPwrGetString(&pcContCur2, "-");
			if (!pcContTmp) {
				DBGLOG(RLM, ERROR,
					"parse channel range error: %s\n");
				goto clearLabel;
			}
			if (pcContCur2 == NULL) { /* case: normal channel */
				if (kalkStrtou8(pcContOld, 0, &value) != 0) {
					DBGLOG(RLM, ERROR,
					       "parse channel error: %s\n",
					       pcContOld);
					goto clearLabel;
				}
				prTmpSetting->channelParam[0] = value;
				prTmpSetting->eChnlType =
						PWR_CTRL_CHNL_TYPE_NORMAL;
			} else { /* case: channel range */
				if (kalkStrtou8(pcContTmp, 0, &value) != 0) {
					DBGLOG(RLM, ERROR,
					       "parse first channel error, %s\n",
					       pcContTmp);
					goto clearLabel;
				}
				if (kalkStrtou8(pcContCur2, 0, &value2) != 0) {
					DBGLOG(RLM, ERROR,
					       "parse second channel error, %s\n",
					       pcContCur2);
					goto clearLabel;
				}
				prTmpSetting->channelParam[0] = value;
				prTmpSetting->channelParam[1] =	value2;
				prTmpSetting->eChnlType =
						PWR_CTRL_CHNL_TYPE_RANGE;
			}
		}

		if (count == PWR_CFG_PRAM_NUM_ALL_RATE) {
			/* parse all rate setting */
			pcContOld = pcContCur;
			if (txPwrParseNumber(&pcContCur, "]", &op, &value)) {
				DBGLOG(RLM, ERROR, "parse CCK error, %s\n",
					   pcContOld);
				goto clearLabel;
			}

			for (j = 0; j < PWR_CFG_PRAM_NUM_AC; j++) {
				prTmpSetting->op[j] = op;
				prTmpSetting->i8PwrLimit[j] = (op != 2) ?
							value : (0 - value);
			}

			for (j = 0; j < PWR_LIMIT_HE_BW160_NUM; j++) {
				prTmpSetting->opHE[j] = op;
				prTmpSetting->i8PwrLimitHE[j] = (op != 2) ?
							value : (0 - value);
			}
#if (CFG_SUPPORT_WIFI_6G == 1)
			for (j = 0; j < PWR_LIMIT_6E_NUM; j++) {
				prTmpSetting->op6E[j] = op;
				prTmpSetting->i8PwrLimit6E[j] = (op != 2) ?
							value : (0 - value);
			}
#endif
			goto skipLabel2;
		} else  if (count == PWR_CFG_PRAM_NUM_AC) {
			for (j = 0; j < PWR_CFG_PRAM_NUM_AC; j++) {
				/* parse cck/20L/20H .. 160L/160H setting */
				if (j == PWR_CFG_PRAM_NUM_AC - 1)
					carySeperator[0] = ']';
				else
					carySeperator[0] = ',';

				pcContOld = pcContCur;
				if (txPwrParseNumber(&pcContCur, carySeperator,
					&op, &value)) {
					DBGLOG(RLM, ERROR,
						"parse %s error, %s\n",
						pacParsePwrAC[j], pcContOld);
					goto clearLabel;
				}
				prTmpSetting->op[j] =
					(enum ENUM_TX_POWER_CTRL_VALUE_SIGN)op;
				prTmpSetting->i8PwrLimit[j] =
					(op != 2) ? value : (0 - value);
				if ((prTmpSetting->op[j]
					== PWR_CTRL_TYPE_POSITIVE) &&
					(ucOperation
					== PWR_CTRL_TYPE_OPERATION_POWER_OFFSET)
					) {
					DBGLOG(RLM, ERROR,
						"parse %s error, Power_Offset value cannot be positive: %u\n",
						pacParsePwrAC[j],
						value);
					goto clearLabel;
				}
			}

		} else if (count == PWR_CFG_PRAM_NUM_AX) {
			for (j = 0; j < PWR_CFG_PRAM_NUM_AX; j++) {
				/* parse RU26L ...  RU996U setting */
				if (j == PWR_CFG_PRAM_NUM_AX - 1)
					carySeperator[0] = ']';
				else
					carySeperator[0] = ',';

				pcContOld = pcContCur;
				if (txPwrParseNumber(&pcContCur, carySeperator,
					&op, &value)) {
					DBGLOG(RLM, ERROR,
						"parse %s error, %s\n",
						pacParsePwrAX[j],
						pcContOld);
					goto clearLabel;
				}
				prTmpSetting->opHE[j] =
					(enum ENUM_TX_POWER_CTRL_VALUE_SIGN)op;
				prTmpSetting->i8PwrLimitHE[j] =
					(op != 2) ? value : (0 - value);
				if ((prTmpSetting->opHE[j]
					== PWR_CTRL_TYPE_POSITIVE) &&
					(ucOperation
					== PWR_CTRL_TYPE_OPERATION_POWER_OFFSET)
					) {
					DBGLOG(RLM, ERROR,
						"parse %s error, Power_Offset value cannot be positive: %u\n",
						pacParsePwrAX[j],
						value);
					goto clearLabel;
				}
			}

		}
#if (CFG_SUPPORT_WIFI_6G == 1)
		else if (count == PWR_LIMIT_6E_NUM) {
			for (j = 0; j < PWR_LIMIT_6E_NUM; j++) {
				/* parse RU26L ...  RU996U setting */
				if (j == PWR_LIMIT_6E_NUM - 1)
					carySeperator[0] = ']';
				else
					carySeperator[0] = ',';

				pcContOld = pcContCur;
				if (txPwrParseNumber(&pcContCur, carySeperator,
					&op, &value)) {
					DBGLOG(RLM, ERROR,
						"parse %s error, %s\n",
						pacParsePwrAX[j],
						pcContOld);
					goto clearLabel;
				}
				prTmpSetting->op6E[j] =
					(enum ENUM_TX_POWER_CTRL_VALUE_SIGN)op;
				prTmpSetting->i8PwrLimit6E[j] =
					(op != 2) ? value : (0 - value);
				prTmpSetting->eChnlType =
					PWR_CTRL_CHNL_TYPE_6G_NORMAL;
				if ((prTmpSetting->op6E[j]
					== PWR_CTRL_TYPE_POSITIVE) &&
					(ucOperation
					== PWR_CTRL_TYPE_OPERATION_POWER_OFFSET)
					) {
					DBGLOG(RLM, ERROR,
						"parse %s error, Power_Offset value cannot be positive: %u\n",
						pacParsePwrAX[j],
						value);
					goto clearLabel;
				}
			}
		}
#endif
		else if (count == PWR_LIMIT_HE_BW160_NUM + 1) {

			/* first tag should be AX160 */
			pcContTmp = txPwrGetString(&pcContCur, ",");
			if (!pcContTmp || !kalStrCmp(pcContTmp,
				     PWR_CTRL_CHNL_TYPE_KEY_5G_AX160) == 0) {
				DBGLOG(RLM, ERROR,
					"parse AX160 error, %s\n", pcContTmp);
				goto clearLabel;
			}
			for (j = 0; j < PWR_LIMIT_HE_BW160_NUM; j++) {
				/* parse RU26L ...  RU996U setting */
				if (j == PWR_LIMIT_HE_BW160_NUM - 1)
					carySeperator[0] = ']';
				else
					carySeperator[0] = ',';

				pcContOld = pcContCur;
				if (txPwrParseNumber(&pcContCur, carySeperator,
					&op, &value)) {
					DBGLOG(RLM, ERROR,
						"parse %s error, %s\n",
						pacParsePwrAX[j],
						pcContOld);
					goto clearLabel;
				}
				prTmpSetting->opHE[j] =
					(enum ENUM_TX_POWER_CTRL_VALUE_SIGN)op;
				prTmpSetting->i8PwrLimitHE[j] =
					(op != 2) ? value : (0 - value);
				if ((prTmpSetting->opHE[j]
					== PWR_CTRL_TYPE_POSITIVE) &&
					(ucOperation
					== PWR_CTRL_TYPE_OPERATION_POWER_OFFSET)
					) {
					DBGLOG(RLM, ERROR,
						"parse %s error, Power_Offset value cannot be positive: %u\n",
						pacParsePwrAX[j],
						value);
					goto clearLabel;
				}
			}
		}

skipLabel2:
		pcContCur = pcContNext + 2;
	}

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	/* parse power limit append tag to list. */
	pcContCur -= 1;

	if (txPwrOnPreParseAppendTag(prCurElement)) {
		DBGLOG(RLM, INFO,
			"txPwrOnPreParseAppendTag fail.");
	}

	if (txPwrParseAppendTag(pcContCur, pcContEnd, prCurElement))
		DBGLOG(RLM, INFO, "txPwrParseAppendTag fail (%s).", pcContent);
#endif
	return prCurElement;

clearLabel:
	if (prCurElement != NULL)
		kalMemFree(prCurElement, VIR_MEM_TYPE, u4MemSize);

	return NULL;
}

/* filterType: 0:no filter, 1:fgEnable is TRUE */
int txPwrCtrlListSize(struct ADAPTER *prAdapter, uint8_t filterType)
{
	struct LINK_ENTRY *prCur, *prNext;
	struct TX_PWR_CTRL_ELEMENT *prCurElement = NULL;
	struct LINK *aryprlist[2] = {
		&prAdapter->rTxPwr_DefaultList,
		&prAdapter->rTxPwr_DynamicList
	};
	int i, count = 0;

	for (i = 0; i < ARRAY_SIZE(aryprlist); i++) {
		LINK_FOR_EACH_SAFE(prCur, prNext, aryprlist[i]) {
			prCurElement = LINK_ENTRY(prCur,
				struct TX_PWR_CTRL_ELEMENT, node);
			if ((filterType == 1) &&
			    (prCurElement->fgApplied != TRUE))
				continue;
			count++;
		}
	}

	return count;
}

/* filterType: 0:no filter, 1:fgApplied is TRUE */
void txPwrCtrlShowList(struct ADAPTER *prAdapter, uint8_t filterType,
		       char *message)
{
	struct LINK_ENTRY *prCur, *prNext;
	struct TX_PWR_CTRL_ELEMENT *prCurElement = NULL;
	struct LINK *aryprlist[2] = {
		&prAdapter->rTxPwr_DefaultList,
		&prAdapter->rTxPwr_DynamicList
	};
	uint8_t ucAppliedWay, ucOperation;
	int i, count = 0;

	if (filterType == 1)
		DBGLOG(RLM, TRACE, "Tx Power Ctrl List=[%s], Size=[%d]",
		       message, txPwrCtrlListSize(prAdapter, filterType));
	else
		DBGLOG(RLM, TRACE, "Tx Power Ctrl List=[%s], Size=[%d]",
		       message, txPwrCtrlListSize(prAdapter, filterType));

	for (i = 0; i < ARRAY_SIZE(aryprlist); i++) {
		LINK_FOR_EACH_SAFE(prCur, prNext, aryprlist[i]) {
			prCurElement = LINK_ENTRY(prCur,
					struct TX_PWR_CTRL_ELEMENT, node);
			if ((filterType == 1) &&
			    (prCurElement->fgApplied != TRUE))
				continue;

			switch (prCurElement->eCtrlType) {
			case PWR_CTRL_TYPE_WIFION_POWER_LEVEL:
				ucAppliedWay =
					PWR_CTRL_TYPE_APPLIED_WAY_WIFION;
				ucOperation =
					PWR_CTRL_TYPE_OPERATION_POWER_LEVEL;
				break;
			case PWR_CTRL_TYPE_WIFION_POWER_OFFSET:
				ucAppliedWay =
					PWR_CTRL_TYPE_APPLIED_WAY_WIFION;
				ucOperation =
					PWR_CTRL_TYPE_OPERATION_POWER_OFFSET;
				break;
			case PWR_CTRL_TYPE_IOCTL_POWER_LEVEL:
				ucAppliedWay =
					PWR_CTRL_TYPE_APPLIED_WAY_IOCTL;
				ucOperation =
					PWR_CTRL_TYPE_OPERATION_POWER_LEVEL;
				break;
			case PWR_CTRL_TYPE_IOCTL_POWER_OFFSET:
				ucAppliedWay =
					PWR_CTRL_TYPE_APPLIED_WAY_IOCTL;
				ucOperation =
					PWR_CTRL_TYPE_OPERATION_POWER_OFFSET;
				break;
			default:
				ucAppliedWay = 0;
				ucOperation = 0;
				break;
			}

			DBGLOG(RLM, TRACE,
			       "Tx Power Ctrl Element-%u: name=[%s], index=[%u], appliedWay=[%u:%s], operation=[%u:%s], ChlSettingCount=[%u]\n",
			       ++count, prCurElement->name, prCurElement->index,
			       ucAppliedWay,
			       g_au1TxPwrAppliedWayLabel[ucAppliedWay - 1],
			       ucOperation,
			       g_au1TxPwrOperationLabel[ucOperation - 1],
			       prCurElement->settingCount);

		}
	}
}

/* This function used to delete element by specifying name or index
 * if index is 0, deletion only according name
 * if index >= 1, deletion according name and index
 */
void _txPwrCtrlDeleteElement(struct ADAPTER *prAdapter,
			     uint8_t *name,
			     uint32_t index,
			     struct LINK *prTxPwrCtrlList)
{
	struct LINK_ENTRY *prCur, *prNext;
	struct TX_PWR_CTRL_ELEMENT *prCurElement = NULL;
	uint32_t u4MemSize = sizeof(struct TX_PWR_CTRL_ELEMENT);
	uint32_t u4MemSize2;
	uint32_t u4SettingSize = sizeof(struct TX_PWR_CTRL_CHANNEL_SETTING);
	uint8_t ucSettingCount;
	u_int8_t fgFind;

	LINK_FOR_EACH_SAFE(prCur, prNext, prTxPwrCtrlList) {
		fgFind = FALSE;
		prCurElement = LINK_ENTRY(prCur, struct TX_PWR_CTRL_ELEMENT,
					  node);
		if (kalStrCmp(prCurElement->name, name) == 0) {
			if (index == 0)
				fgFind = TRUE;
			else if (prCurElement->index == index)
				fgFind = TRUE;
			if (fgFind) {
				linkDel(prCur);
				if (prCurElement != NULL) {
					ucSettingCount =
						prCurElement->settingCount;
						u4MemSize2 = u4MemSize +
						((ucSettingCount == 1) ? 0 :
						(ucSettingCount - 1) *
						u4SettingSize);
					kalMemFree(prCurElement, VIR_MEM_TYPE,
						u4MemSize2);
				}
			}
		}
	}
}

void txPwrCtrlDeleteElement(struct ADAPTER *prAdapter,
			    uint8_t *name,
			    uint32_t index,
			    enum ENUM_TX_POWER_CTRL_LIST_TYPE eListType)
{
	if ((eListType == PWR_CTRL_TYPE_ALL_LIST) ||
	    (eListType == PWR_CTRL_TYPE_DEFAULT_LIST))
		_txPwrCtrlDeleteElement(prAdapter, name, index,
					&prAdapter->rTxPwr_DefaultList);

	if ((eListType == PWR_CTRL_TYPE_ALL_LIST) ||
	    (eListType == PWR_CTRL_TYPE_DYNAMIC_LIST))
		_txPwrCtrlDeleteElement(prAdapter, name, index,
					&prAdapter->rTxPwr_DynamicList);
}

struct TX_PWR_CTRL_ELEMENT *_txPwrCtrlFindElement(struct ADAPTER *prAdapter,
				 uint8_t *name, uint32_t index,
				 u_int8_t fgCheckIsApplied,
				 struct LINK *prTxPwrCtrlList)
{
	struct LINK_ENTRY *prCur, *prNext;
	struct TX_PWR_CTRL_ELEMENT *prCurElement = NULL;


	LINK_FOR_EACH_SAFE(prCur, prNext, prTxPwrCtrlList) {
		u_int8_t fgFind = FALSE;

		prCurElement = LINK_ENTRY(
			prCur, struct TX_PWR_CTRL_ELEMENT, node);
		if (kalStrCmp(prCurElement->name, name) == 0) {
			if ((!fgCheckIsApplied) ||
			    (fgCheckIsApplied &&
			    (prCurElement->fgApplied == TRUE))) {
				if (index == 0)
					fgFind = TRUE;
				else if (prCurElement->index == index)
					fgFind = TRUE;
			}
			if (fgFind)
				return prCurElement;
		}
	}
	return NULL;
}

struct TX_PWR_CTRL_ELEMENT *txPwrCtrlFindElement(struct ADAPTER *prAdapter,
				 uint8_t *name, uint32_t index,
				 u_int8_t fgCheckIsApplied,
				 enum ENUM_TX_POWER_CTRL_LIST_TYPE eListType)
{
	struct LINK *prTxPwrCtrlList = NULL;

	if (eListType == PWR_CTRL_TYPE_DEFAULT_LIST)
		prTxPwrCtrlList = &prAdapter->rTxPwr_DefaultList;
	if (eListType == PWR_CTRL_TYPE_DYNAMIC_LIST)
		prTxPwrCtrlList = &prAdapter->rTxPwr_DynamicList;
	if (prTxPwrCtrlList == NULL)
		return NULL;

	return _txPwrCtrlFindElement(prAdapter, name, index, fgCheckIsApplied,
				     prTxPwrCtrlList);
}

void txPwrCtrlAddElement(struct ADAPTER *prAdapter,
			 struct TX_PWR_CTRL_ELEMENT *prElement)
{
	struct LINK_ENTRY *prNode = &prElement->node;

	switch (prElement->eCtrlType) {
	case PWR_CTRL_TYPE_WIFION_POWER_LEVEL:
		linkAdd(prNode, &prAdapter->rTxPwr_DefaultList);
		break;
	case PWR_CTRL_TYPE_WIFION_POWER_OFFSET:
		linkAddTail(prNode, &prAdapter->rTxPwr_DefaultList);
		break;
	case PWR_CTRL_TYPE_IOCTL_POWER_LEVEL:
		linkAdd(prNode, &prAdapter->rTxPwr_DynamicList);
		break;
	case PWR_CTRL_TYPE_IOCTL_POWER_OFFSET:
		linkAddTail(prNode, &prAdapter->rTxPwr_DynamicList);
		break;
	}
}

void txPwrCtrlFileBufToList(struct ADAPTER *prAdapter, uint8_t *pucFileBuf)
{
	struct TX_PWR_CTRL_ELEMENT *prNewElement;
	char *oneLine;

	if (pucFileBuf == NULL)
		return;

	while ((oneLine = kalStrSep((char **)(&pucFileBuf), "\r\n"))
	       != NULL) {
		/* skip comment line and empty line */
		if ((oneLine[0] == '#') || (oneLine[0] == 0))
			continue;

		prNewElement = txPwrCtrlStringToStruct(oneLine, FALSE);
		if (prNewElement != NULL) {
			/* delete duplicated element
			 * by checking name and index
			 */
			txPwrCtrlDeleteElement(prAdapter,
				prNewElement->name, prNewElement->index,
				PWR_CTRL_TYPE_ALL_LIST);

			/* append to rTxPwr_List */
			txPwrCtrlAddElement(prAdapter, prNewElement);
		}
	}

	/* show the tx power ctrl list */
	txPwrCtrlShowList(prAdapter, 0, "config list, after loading cfg file");
}

void txPwrCtrlGlobalVariableToList(struct ADAPTER *prAdapter)
{
	struct TX_PWR_CTRL_ELEMENT *pcElement;
	char *ptr;
	int32_t i, u4MemSize;

	for (i = 0; i < ARRAY_SIZE(g_au1TxPwrDefaultSetting); i++) {
		/* skip empty line */
		if (g_au1TxPwrDefaultSetting[i][0] == 0)
			continue;
		u4MemSize = kalStrLen(g_au1TxPwrDefaultSetting[i]) + 1;
		ptr = (char *)kalMemAlloc(u4MemSize, VIR_MEM_TYPE);
		if (ptr == NULL) {
			DBGLOG(RLM, ERROR, "kalMemAlloc fail: %d\n", u4MemSize);
			continue;
		}
		kalMemCopy(ptr, g_au1TxPwrDefaultSetting[i], u4MemSize);
		*(ptr + u4MemSize - 1) = 0;
		pcElement = txPwrCtrlStringToStruct(ptr, FALSE);
		kalMemFree(ptr, VIR_MEM_TYPE, u4MemSize);
		if (pcElement != NULL) {
			/* delete duplicated element
			 * by checking name and index
			 */
			txPwrCtrlDeleteElement(prAdapter,
				pcElement->name, pcElement->index,
				PWR_CTRL_TYPE_ALL_LIST);

			/* append to rTxPwr_List */
			txPwrCtrlAddElement(prAdapter, pcElement);
		}
	}

	/* show the tx power ctrl cfg list */
	txPwrCtrlShowList(prAdapter, 0,
			  "config list, after loadding global variables");
}

void txPwrCtrlCfgFileToList(struct ADAPTER *prAdapter)
{
	uint8_t *pucConfigBuf;
	uint32_t u4ConfigReadLen = 0;

	pucConfigBuf = (uint8_t *)kalMemAlloc(WLAN_CFG_FILE_BUF_SIZE,
					      VIR_MEM_TYPE);
	kalMemZero(pucConfigBuf, WLAN_CFG_FILE_BUF_SIZE);
	if (pucConfigBuf) {
		if (kalRequestFirmware("txpowerctrl.cfg", pucConfigBuf,
		    WLAN_CFG_FILE_BUF_SIZE, &u4ConfigReadLen,
		    prAdapter->prGlueInfo->prDev) == 0) {
			/* ToDo:: Nothing */
		}

		if (pucConfigBuf[0] != '\0' && u4ConfigReadLen > 0
			&& u4ConfigReadLen < WLAN_CFG_FILE_BUF_SIZE) {
			pucConfigBuf[u4ConfigReadLen] = 0;
			txPwrCtrlFileBufToList(prAdapter, pucConfigBuf);
		} else
			DBGLOG(RLM, INFO,
			       "no txpowerctrl.cfg or file is empty\n");

		kalMemFree(pucConfigBuf, VIR_MEM_TYPE, WLAN_CFG_FILE_BUF_SIZE);
	}
}

void txPwrCtrlLoadConfig(struct ADAPTER *prAdapter)
{
	/* 1. add records from global tx power ctrl setting into cfg list */
	txPwrCtrlGlobalVariableToList(prAdapter);

	/* 2. update cfg list by txpowerctrl.cfg */
	txPwrCtrlCfgFileToList(prAdapter);

#if CFG_SUPPORT_PWR_LIMIT_COUNTRY
	/* 3. send setting to firmware */
	rlmDomainSendPwrLimitCmd(prAdapter);
#endif
}

void txPwrCtrlInit(struct ADAPTER *prAdapter)
{
	LINK_INITIALIZE(&prAdapter->rTxPwr_DefaultList);
	LINK_INITIALIZE(&prAdapter->rTxPwr_DynamicList);
}

void txPwrCtrlUninit(struct ADAPTER *prAdapter)
{
	struct LINK_ENTRY *prCur, *prNext;
	struct TX_PWR_CTRL_ELEMENT *prCurElement = NULL;
	struct LINK *aryprlist[2] = {
		&prAdapter->rTxPwr_DefaultList,
		&prAdapter->rTxPwr_DynamicList
	};
	uint32_t u4MemSize = sizeof(struct TX_PWR_CTRL_ELEMENT);
	uint32_t u4MemSize2;
	uint32_t u4SettingSize = sizeof(struct TX_PWR_CTRL_CHANNEL_SETTING);
	uint8_t ucSettingCount;
	int32_t i;

	for (i = 0; i < ARRAY_SIZE(aryprlist); i++) {
		LINK_FOR_EACH_SAFE(prCur, prNext, aryprlist[i]) {
			prCurElement = LINK_ENTRY(prCur,
					struct TX_PWR_CTRL_ELEMENT, node);
			linkDel(prCur);
			if (prCurElement) {
				ucSettingCount = prCurElement->settingCount;
					u4MemSize2 = u4MemSize +
					((ucSettingCount <= 1) ? 0 :
					(ucSettingCount - 1) * u4SettingSize);
				kalMemFree(prCurElement,
					VIR_MEM_TYPE, u4MemSize2);
			}
		}
	}
}
/* dynamic tx power control: end **********************************************/
#endif /* CFG_SUPPORT_DYNAMIC_PWR_LIMIT */

void rlmDomainShowPwrLimitPerCh(char *message,
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd)
{
	/* for print usage */
	struct CMD_CHANNEL_POWER_LIMIT *prPwrLmt = NULL;
	/* for print usage */
	struct CMD_CHANNEL_POWER_LIMIT_HE *prPwrLmtHE = NULL;
	struct CMD_CHANNEL_POWER_LIMIT_HE_BW160 *prPwrLmtHEBW160 = NULL;
#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	struct CMD_CHANNEL_POWER_LIMIT_ANT *prPwrLmtAnt = NULL;
#endif
#if (CFG_SUPPORT_WIFI_6G == 1)
	struct CMD_CHANNEL_POWER_LIMIT_6E *prPwrLmt6E = NULL;
#endif
	enum ENUM_PWR_LIMIT_TYPE eType;
	uint8_t i, j, k;
	char msgLimit[PWR_BUF_LEN];
	int msgOfs = 0;
	int8_t *prcRatePwr;

	ASSERT(prCmd);

	eType = prCmd->ucLimitType;

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	if (eType == PWR_LIMIT_TYPE_COMP_ANT_V2) {
		prPwrLmtAnt = &prCmd->u.rChPwrLimtAnt[0];
		DBGLOG(RLM, TRACE, "ANT Config #%d", prCmd->ucNum);
		for (i = 0; i < prCmd->ucNum; i++) {
			DBGLOG(RLM, TRACE, "%s ANT Config%d[%d:%d:%d:%d]\n",
				message,
				i,
				prPwrLmtAnt[i].cTagIdx,
				prPwrLmtAnt[i].cBandIdx,
				prPwrLmtAnt[i].cAntIdx,
				prPwrLmtAnt[i].cValue);

			if (prPwrLmtAnt[i].cTagIdx == -1)
				break;
		}

		return;
	}
#endif

	for (i = 0; i < prCmd->ucNum; i++) {

		if (i >= MAX_CMD_SUPPORT_CHANNEL_NUM) {
			DBGLOG(RLM, ERROR, "out of MAX CH Num\n");
			return;
		}

		kalMemZero(msgLimit, sizeof(char)*PWR_BUF_LEN);
		msgOfs = 0;
		if (eType == PWR_LIMIT_TYPE_COMP_11AX) {
			prPwrLmtHE = &prCmd->u.rChPwrLimtHE[i];
			prcRatePwr = &prPwrLmtHE->cPwrLimitRU26L;

			/*message head*/
			msgOfs += snprintf(msgLimit + msgOfs,
				PWR_BUF_LEN - msgOfs,
				"HE ch=%d,Limit=",
				prPwrLmtHE->ucCentralCh);

			/*message body*/
			for (j = PWR_LIMIT_RU26_L; j < PWR_LIMIT_HE_NUM ; j++)
				msgOfs += snprintf(msgLimit + msgOfs,
					PWR_BUF_LEN - msgOfs,
					"%d,",
					*(prcRatePwr + j));
			/*message tail*/
			if (msgOfs >= 1)
				msgLimit[msgOfs-1] = '\0';
			else
				msgLimit[0] = '\0';

			DBGLOG(RLM, TRACE, "%s:%s\n", message, msgLimit);

		} else if (eType == PWR_LIMIT_TYPE_COMP_11AX_BW160) {
			prPwrLmtHEBW160 = &prCmd->u.rChPwrLimtHEBW160[i];
			prcRatePwr = &prPwrLmtHEBW160->cPwrLimitRU26L;

			/*message head*/
			msgOfs += snprintf(msgLimit + msgOfs,
				PWR_BUF_LEN - msgOfs,
				"HE ch=%d,Limit=",
				prPwrLmtHEBW160->ucCentralCh);

			/*message body*/
			for (j = PWR_LIMIT_RU26_L;
				j < PWR_LIMIT_HE_BW160_NUM ; j++)
				msgOfs += snprintf(msgLimit + msgOfs,
					PWR_BUF_LEN - msgOfs,
					"%d,",
					*(prcRatePwr + j));
			/*message tail*/
			if (msgOfs >= 1)
				msgLimit[msgOfs-1] = '\0';
			else
				msgLimit[0] = '\0';

			DBGLOG(RLM, TRACE, "%s:%s\n", message, msgLimit);

		}
#if (CFG_SUPPORT_WIFI_6G == 1)
		else if (eType == PWR_LIMIT_TYPE_COMP_6E_1 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_2 ||
					eType == PWR_LIMIT_TYPE_COMP_6E_3) {
			prPwrLmt6E = &prCmd->u.rChPwrLimt6E[i];
			prcRatePwr = &prPwrLmt6E->cPwrLimitRU26L;

			/*message head*/
			msgOfs += snprintf(msgLimit + msgOfs,
				PWR_BUF_LEN - msgOfs,
				"6E ch=%d,Limit=",
				prPwrLmt6E->ucCentralCh);

			/*message body*/
			for (j = PWR_LIMIT_RU26_L; j < PWR_LIMIT_6E_NUM ; j++)
				msgOfs += snprintf(msgLimit + msgOfs,
					PWR_BUF_LEN - msgOfs,
					"%d,",
					*(prcRatePwr + j));
			/*message tail*/
			if (msgOfs >= 1)
				msgLimit[msgOfs-1] = '\0';
			else
				msgLimit[0] = '\0';

			DBGLOG(RLM, TRACE, "%s:%s\n", message, msgLimit);

		}
#endif
		else {
			prPwrLmt = &prCmd->u.rChannelPowerLimit[i];
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
			prcRatePwr = &prPwrLmt->cPwrLimitCCK_L;
#else
			prcRatePwr = &prPwrLmt->cPwrLimitCCK;
#endif
			/*message head*/
			msgOfs += snprintf(msgLimit + msgOfs,
				PWR_BUF_LEN - msgOfs,
				"legacy ch=%d,Limit=",
				prPwrLmt->ucCentralCh);

			/*message body*/
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
			for (k = PWR_LIMIT_CCK_L; k < PWR_LIMIT_NUM ; k++)
#else
			for (k = PWR_LIMIT_CCK; k < PWR_LIMIT_NUM ; k++)
#endif
				msgOfs += snprintf(msgLimit + msgOfs,
					PWR_BUF_LEN - msgOfs,
					"%d,",
					*(prcRatePwr + k));

			/*message tail*/
			if (msgOfs > 0)
				msgLimit[msgOfs-1] = '\0';
			else
				msgLimit[0] = '\0';

			DBGLOG(RLM, TRACE, "%s:%s\n", message, msgLimit);
		}
	}

}

void rlmDomainSendPwrLimitCmd(struct ADAPTER *prAdapter)
{
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd = NULL;
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmdHE = NULL;
#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmdAnt = NULL;
	uint32_t u4SetQueryInfoLenAnt = 0;
#endif
#if (CFG_SUPPORT_WIFI_6G == 1)
	/* 6E have many channel need to send by serval commnd */
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd6E_1 = NULL;
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd6E_2 = NULL;
	struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT *prCmd6E_3 = NULL;
#endif
	uint32_t rStatus;
	uint16_t u2DefaultTableIndex;
	uint32_t u4SetCmdTableMaxSize;
	uint32_t u4SetQueryInfoLen;
	uint8_t bandedgeParam[4] = { 0, 0, 0, 0 };
	struct DOMAIN_INFO_ENTRY *prDomainInfo;
	/* TODO : 5G band edge */
	prDomainInfo = rlmDomainGetDomainInfo(prAdapter);
	if (prDomainInfo) {
		bandedgeParam[0] = prDomainInfo->rSubBand[0].ucFirstChannelNum;
		bandedgeParam[1] = bandedgeParam[0] +
			prDomainInfo->rSubBand[0].ucNumChannels - 1;
	}

	if (regd_is_single_sku_en())
		return rlmDomainSendPwrLimitCmd_V2(prAdapter);


	u4SetCmdTableMaxSize =
	    sizeof(struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT);

	prCmd = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4SetCmdTableMaxSize);

	if (!prCmd) {
		DBGLOG(RLM, ERROR, "Domain: Alloc cmd buffer failed\n");
		goto err;
	}
	kalMemZero(prCmd, u4SetCmdTableMaxSize);

	prCmdHE = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4SetCmdTableMaxSize);

	if (!prCmdHE) {
		DBGLOG(RLM, ERROR, "Domain: Alloc cmd buffer failed\n");
		goto err;
	}
	kalMemZero(prCmdHE, u4SetCmdTableMaxSize);

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	prCmdAnt = cnmMemAlloc(prAdapter, RAM_TYPE_BUF,
		u4SetCmdTableMaxSize);

	if (!prCmdAnt) {
		DBGLOG(RLM, ERROR, "Domain: Alloc cmd buffer failed\n");
		goto err;
	}
	kalMemZero(prCmdAnt, u4SetCmdTableMaxSize);
#endif
#if (CFG_SUPPORT_WIFI_6G == 1)
	prCmd6E_1 = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4SetCmdTableMaxSize);

	if (!prCmd6E_1) {
		DBGLOG(RLM, ERROR, "Domain: Alloc cmd buffer failed\n");
		goto err;
	}
	kalMemZero(prCmd6E_1, u4SetCmdTableMaxSize);

	prCmd6E_2 = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4SetCmdTableMaxSize);

	if (!prCmd6E_2) {
		DBGLOG(RLM, ERROR, "Domain: Alloc cmd buffer failed\n");
		goto err;
	}
	kalMemZero(prCmd6E_2, u4SetCmdTableMaxSize);

	prCmd6E_3 = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4SetCmdTableMaxSize);

	if (!prCmd6E_3) {
		DBGLOG(RLM, ERROR, "Domain: Alloc cmd buffer failed\n");
		goto err;
	}
	kalMemZero(prCmd6E_3, u4SetCmdTableMaxSize);
#endif

	u2DefaultTableIndex =
	    rlmDomainPwrLimitDefaultTableDecision(prAdapter,
		prAdapter->rWifiVar.u2CountryCode);

	if (u2DefaultTableIndex == POWER_LIMIT_TABLE_NULL) {
		DBGLOG(RLM, ERROR,
					   "Can't find any table index!\n");
		goto err;
	}

	WLAN_GET_FIELD_BE16(&g_rRlmPowerLimitDefault
			    [u2DefaultTableIndex]
			    .aucCountryCode[0],
			    &prCmd->u2CountryCode);
	WLAN_GET_FIELD_BE16(&g_rRlmPowerLimitDefault
			    [u2DefaultTableIndex]
			    .aucCountryCode[0],
			    &prCmdHE->u2CountryCode);

	if (prCmd->u2CountryCode == COUNTRY_CODE_NULL)
		DBGLOG(RLM, TRACE,
			   "CC={0x00,0x00},Power Limit use Default setting!");


	/* Initialize channel number */
	prCmd->ucNum = 0;
#if (CFG_SUPPORT_DYNA_TX_PWR_CTRL_11AC_V2_SETTING == 1)
	prCmd->ucLimitType = PWR_LIMIT_TYPE_COMP_11AC_V2;
#else
	prCmd->ucLimitType = PWR_LIMIT_TYPE_COMP_11AC;
#endif
	prCmd->ucVersion = 1;

	prCmdHE->ucNum = 0;
	prCmdHE->fgPwrTblKeep = TRUE;
	prCmdHE->ucLimitType = PWR_LIMIT_TYPE_COMP_11AX;

	if (prAdapter->rWifiVar.ucSta5gBandwidth == MAX_BW_160MHZ)
		prCmdHE->ucLimitType = PWR_LIMIT_TYPE_COMP_11AX_BW160;

	prCmdHE->ucVersion = 1;

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	prCmdAnt->ucNum = 0;
	prCmdAnt->fgPwrTblKeep = TRUE;
	/* ANT number if PWR_LIMIT_TYPE_COMP_ANT*/
	prCmdAnt->ucLimitType = PWR_LIMIT_TYPE_COMP_ANT_V2;
	prCmdAnt->ucVersion = 1;
#endif
#if (CFG_SUPPORT_WIFI_6G == 1)
	prCmd6E_1->ucNum = 0;
	prCmd6E_1->fgPwrTblKeep = TRUE;
	prCmd6E_1->ucLimitType = PWR_LIMIT_TYPE_COMP_6E_1;

	prCmd6E_2->ucNum = 0;
	prCmd6E_2->fgPwrTblKeep = TRUE;
	prCmd6E_2->ucLimitType = PWR_LIMIT_TYPE_COMP_6E_2;

	prCmd6E_3->ucNum = 0;
	prCmd6E_3->fgPwrTblKeep = TRUE;
	prCmd6E_3->ucLimitType = PWR_LIMIT_TYPE_COMP_6E_3;
#endif
	/*<1>Command - default table information,
	 *fill all subband
	 */
	rlmDomainBuildCmdByDefaultTable(prCmd,
		u2DefaultTableIndex);

	/*<2>Command - configuration table information,
	 * replace specified channel
	 */
	rlmDomainBuildCmdByConfigTable(prAdapter,
		prCmd);

#if (CFG_SUPPORT_PWR_LIMIT_HE == 1)
	/*<1>Command - default table information,
	 *fill all subband
	 */
	rlmDomainBuildCmdByDefaultTable(prCmdHE,
		u2DefaultTableIndex);

	/*<2>Command - configuration table information,
	 * replace specified channel
	 */
	rlmDomainBuildCmdByConfigTable(prAdapter,
		prCmdHE);
#endif

#if (CFG_SUPPORT_WIFI_6G == 1)
		/*<1>Command - default table information,
		 *fill all subband
		 */
		rlmDomainBuildCmdByDefaultTable(prCmd6E_1,
			u2DefaultTableIndex);

		/*<2>Command - configuration table information,
		 * replace specified channel
		 */
		rlmDomainBuildCmdByConfigTable(prAdapter,
			prCmd6E_1);

		/*<1>Command - default table information,
		 *fill all subband
		 */
		rlmDomainBuildCmdByDefaultTable(prCmd6E_2,
			u2DefaultTableIndex);

		/*<2>Command - configuration table information,
		 * replace specified channel
		 */
		rlmDomainBuildCmdByConfigTable(prAdapter,
			prCmd6E_2);

		/*<1>Command - default table information,
		 *fill all subband
		 */
		rlmDomainBuildCmdByDefaultTable(prCmd6E_3,
			u2DefaultTableIndex);

		/*<2>Command - configuration table information,
		 * replace specified channel
		 */
		rlmDomainBuildCmdByConfigTable(prAdapter,
			prCmd6E_3);
#endif


	DBGLOG(RLM, TRACE,
	       "Domain: ValidCC=%c%c, PwrLimitCC=%c%c, PwrLimitChNum=%d\n",
	       (prAdapter->rWifiVar.u2CountryCode & 0xff00) >> 8,
	       (prAdapter->rWifiVar.u2CountryCode & 0x00ff),
	       ((prCmd->u2CountryCode & 0xff00) >> 8),
	       (prCmd->u2CountryCode & 0x00ff),
	       prCmd->ucNum);

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT
	rlmDomainShowPwrLimitPerCh("Old", prCmd);
	/* apply each setting into country channel power table */
	txPwrCtrlApplySettings(prAdapter, prCmd, bandedgeParam);
	/* show tx power table after applying setting */
	rlmDomainShowPwrLimitPerCh("Final", prCmd);

#if (CFG_SUPPORT_PWR_LIMIT_HE == 1)
	rlmDomainShowPwrLimitPerCh("Old", prCmdHE);
	txPwrCtrlApplySettings(prAdapter, prCmdHE, bandedgeParam);
	rlmDomainShowPwrLimitPerCh("Final", prCmdHE);
#endif /*#if (CFG_SUPPORT_PWR_LIMIT_HE == 1)*/

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	txPwrCtrlApplySettings(prAdapter, prCmdAnt, bandedgeParam);
	rlmDomainShowPwrLimitPerCh("Final", prCmdAnt);
#endif

#if (CFG_SUPPORT_WIFI_6G == 1)
	rlmDomainShowPwrLimitPerCh("Old", prCmd6E_1);
	txPwrCtrlApplySettings(prAdapter, prCmd6E_1, bandedgeParam);
	rlmDomainShowPwrLimitPerCh("Final", prCmd6E_1);

	rlmDomainShowPwrLimitPerCh("Old", prCmd6E_2);
	txPwrCtrlApplySettings(prAdapter, prCmd6E_2, bandedgeParam);
	rlmDomainShowPwrLimitPerCh("Final", prCmd6E_2);

	rlmDomainShowPwrLimitPerCh("Old", prCmd6E_3);
	txPwrCtrlApplySettings(prAdapter, prCmd6E_3, bandedgeParam);
	rlmDomainShowPwrLimitPerCh("Final", prCmd6E_3);
#endif
#endif

	u4SetQueryInfoLen =
		sizeof(struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT);

	/* Update domain info to chip */
	if (prCmd->ucNum <= MAX_CMD_SUPPORT_CHANNEL_NUM) {
		rStatus = wlanSendSetQueryCmd(prAdapter,	/* prAdapter */
				CMD_ID_SET_COUNTRY_POWER_LIMIT,	/* ucCID */
				TRUE,	/* fgSetQuery */
				FALSE,	/* fgNeedResp */
				FALSE,	/* fgIsOid */
				NULL,	/* pfCmdDoneHandler */
				NULL,	/* pfCmdTimeoutHandler */
				u4SetQueryInfoLen,	/* u4SetQueryInfoLen */
				(uint8_t *) prCmd,	/* pucInfoBuffer */
				NULL,	/* pvSetQueryBuffer */
				0	/* u4SetQueryBufferLen */
		    );
		if (rStatus != WLAN_STATUS_PENDING)
			DBGLOG(RLM, ERROR, "Power limit channel 0x%08x\n",
				rStatus);
#if (CFG_SUPPORT_PWR_LIMIT_HE == 1)
		rStatus = wlanSendSetQueryCmd(prAdapter,	/* prAdapter */
				CMD_ID_SET_COUNTRY_POWER_LIMIT,	/* ucCID */
				TRUE,	/* fgSetQuery */
				FALSE,	/* fgNeedResp */
				FALSE,	/* fgIsOid */
				NULL,	/* pfCmdDoneHandler */
				NULL,	/* pfCmdTimeoutHandler */
				u4SetQueryInfoLen,	/* u4SetQueryInfoLen */
				(uint8_t *) prCmdHE,	/* pucInfoBuffer */
				NULL,	/* pvSetQueryBuffer */
				0	/* u4SetQueryBufferLen */
		    );
		if (rStatus != WLAN_STATUS_PENDING)
			DBGLOG(RLM, ERROR, "Power limit channel HE 0x%08x\n",
				rStatus);

#endif
#if (CFG_SUPPORT_WIFI_6G == 1)
		rStatus = wlanSendSetQueryCmd(prAdapter,	/* prAdapter */
			CMD_ID_SET_COUNTRY_POWER_LIMIT,	/* ucCID */
			TRUE,	/* fgSetQuery */
			FALSE,	/* fgNeedResp */
			FALSE,	/* fgIsOid */
			NULL,	/* pfCmdDoneHandler */
			NULL,	/* pfCmdTimeoutHandler */
			u4SetQueryInfoLen,	/* u4SetQueryInfoLen */
			(uint8_t *) prCmd6E_1,	/* pucInfoBuffer */
			NULL,	/* pvSetQueryBuffer */
			0	/* u4SetQueryBufferLen */
	    );

		rStatus = wlanSendSetQueryCmd(prAdapter,	/* prAdapter */
			CMD_ID_SET_COUNTRY_POWER_LIMIT,	/* ucCID */
			TRUE,	/* fgSetQuery */
			FALSE,	/* fgNeedResp */
			FALSE,	/* fgIsOid */
			NULL,	/* pfCmdDoneHandler */
			NULL,	/* pfCmdTimeoutHandler */
			u4SetQueryInfoLen,	/* u4SetQueryInfoLen */
			(uint8_t *) prCmd6E_2,	/* pucInfoBuffer */
			NULL,	/* pvSetQueryBuffer */
			0	/* u4SetQueryBufferLen */
	    );

		rStatus = wlanSendSetQueryCmd(prAdapter,	/* prAdapter */
			CMD_ID_SET_COUNTRY_POWER_LIMIT,	/* ucCID */
			TRUE,	/* fgSetQuery */
			FALSE,	/* fgNeedResp */
			FALSE,	/* fgIsOid */
			NULL,	/* pfCmdDoneHandler */
			NULL,	/* pfCmdTimeoutHandler */
			u4SetQueryInfoLen,	/* u4SetQueryInfoLen */
			(uint8_t *) prCmd6E_3,	/* pucInfoBuffer */
			NULL,	/* pvSetQueryBuffer */
			0	/* u4SetQueryBufferLen */
	    );
#endif
	} else {
		DBGLOG(RLM, ERROR, "Domain: illegal power limit table\n");
	}

#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG

	u4SetQueryInfoLenAnt =
		sizeof(struct CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT);

	rStatus = wlanSendSetQueryCmd(prAdapter,	/* prAdapter */
				CMD_ID_SET_COUNTRY_POWER_LIMIT, /* ucCID */
				TRUE,	/* fgSetQuery */
				FALSE,	/* fgNeedResp */
				FALSE,	/* fgIsOid */
				NULL,	/* pfCmdDoneHandler */
				NULL,	/* pfCmdTimeoutHandler */
				u4SetQueryInfoLenAnt,	/* u4SetQueryInfoLen */
				(uint8_t *) prCmdAnt,	/* pucInfoBuffer */
				NULL,	/* pvSetQueryBuffer */
				0	/* u4SetQueryBufferLen */
			);
	if (rStatus != WLAN_STATUS_PENDING)
		DBGLOG(RLM, ERROR, "Power limit revise 0x%08x\n", rStatus);
#endif

err:
	cnmMemFree(prAdapter, prCmd);
	cnmMemFree(prAdapter, prCmdHE);
#if CFG_SUPPORT_DYNAMIC_PWR_LIMIT_ANT_TAG
	cnmMemFree(prAdapter, prCmdAnt);
#endif
#if (CFG_SUPPORT_WIFI_6G == 1)
	cnmMemFree(prAdapter, prCmd6E_1);
	cnmMemFree(prAdapter, prCmd6E_2);
	cnmMemFree(prAdapter, prCmd6E_3);
#endif
}
#endif
u_int8_t regd_is_single_sku_en(void)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	return g_mtk_regd_control.en;
#else
	return FALSE;
#endif
}

#if (CFG_SUPPORT_SINGLE_SKU == 1)
u_int8_t rlmDomainIsCtrlStateEqualTo(enum regd_state state)
{
	return (g_mtk_regd_control.state == state) ? TRUE : FALSE;
}

enum regd_state rlmDomainGetCtrlState(void)
{
	return g_mtk_regd_control.state;
}


void rlmDomainResetActiveChannel(void)
{
	g_mtk_regd_control.n_channel_active_2g = 0;
	g_mtk_regd_control.n_channel_active_5g = 0;
	g_mtk_regd_control.n_channel_active_6g = 0;
}

void rlmDomainAddActiveChannel(u8 band)

{
	if (band == KAL_BAND_2GHZ)
		g_mtk_regd_control.n_channel_active_2g += 1;
	else if (band == KAL_BAND_5GHZ)
		g_mtk_regd_control.n_channel_active_5g += 1;
#if (CFG_SUPPORT_WIFI_6G == 1)
	else if (band == KAL_BAND_6GHZ)
		g_mtk_regd_control.n_channel_active_6g += 1;
#endif
}

u8 rlmDomainGetActiveChannelCount(u8 band)
{
	if (band == KAL_BAND_2GHZ)
		return g_mtk_regd_control.n_channel_active_2g;
	else if (band == KAL_BAND_5GHZ)
		return g_mtk_regd_control.n_channel_active_5g;
#if (CFG_SUPPORT_WIFI_6G == 1)
	else if (band == KAL_BAND_6GHZ)
		return g_mtk_regd_control.n_channel_active_6g;
#endif
	else
		return 0;
}

struct CMD_DOMAIN_CHANNEL *rlmDomainGetActiveChannels(void)
{
	return g_mtk_regd_control.channels;
}

void rlmDomainSetDefaultCountryCode(void)
{
	g_mtk_regd_control.alpha2 = COUNTRY_CODE_WW;
}

void rlmDomainResetCtrlInfo(u_int8_t force)
{
	if ((g_mtk_regd_control.state == REGD_STATE_UNDEFINED) ||
	    (force == TRUE)) {
		memset(&g_mtk_regd_control, 0, sizeof(struct mtk_regd_control));

		g_mtk_regd_control.state = REGD_STATE_INIT;

		rlmDomainSetDefaultCountryCode();

#if (CFG_SUPPORT_SINGLE_SKU_LOCAL_DB == 1)
		g_mtk_regd_control.flag |= REGD_CTRL_FLAG_SUPPORT_LOCAL_REGD_DB;
#endif
	}
}

u_int8_t rlmDomainIsUsingLocalRegDomainDataBase(void)
{
#if (CFG_SUPPORT_SINGLE_SKU_LOCAL_DB == 1)
	return (g_mtk_regd_control.flag & REGD_CTRL_FLAG_SUPPORT_LOCAL_REGD_DB)
		? TRUE : FALSE;
#else
	return FALSE;
#endif
}

bool rlmDomainIsSameCountryCode(char *alpha2, u8 size_of_alpha2)
{
	u8 idx;
	u32 alpha2_hex = 0;

	for (idx = 0; idx < size_of_alpha2; idx++)
		alpha2_hex |= (alpha2[idx] << (idx * 8));

	return (rlmDomainGetCountryCode() == alpha2_hex) ? TRUE : FALSE;
}

void rlmDomainSetCountryCode(char *alpha2, u8 size_of_alpha2)
{
	u8 max;
	u8 buf_size;

	buf_size = sizeof(g_mtk_regd_control.alpha2);
	max = (buf_size < size_of_alpha2) ? buf_size : size_of_alpha2;

	g_mtk_regd_control.alpha2 = rlmDomainAlpha2ToU32(alpha2, max);
}

void rlmDomainSetDfsRegion(enum nl80211_dfs_regions dfs_region)
{
	g_mtk_regd_control.dfs_region = dfs_region;
}

enum nl80211_dfs_regions rlmDomainGetDfsRegion(void)
{
	return g_mtk_regd_control.dfs_region;
}

void rlmDomainSetDfsDbdcBand(enum ENUM_DBDC_BN eDBDCBand)
{
	g_mtk_regd_control.eDBDCBand = eDBDCBand;
}

enum ENUM_DBDC_BN rlmDomainGetDfsDbdcBand(void)
{
	return g_mtk_regd_control.eDBDCBand;
}

void rlmDomainSetTempCountryCode(char *alpha2, u8 size_of_alpha2)
{
	u8 idx, max;
	u8 buf_size;

	buf_size = sizeof(g_mtk_regd_control.tmp_alpha2);
	max = (buf_size < size_of_alpha2) ? buf_size : size_of_alpha2;

	g_mtk_regd_control.tmp_alpha2 = 0;

	for (idx = 0; idx < max; idx++)
		g_mtk_regd_control.tmp_alpha2 |= (alpha2[idx] << (idx * 8));

}

enum regd_state rlmDomainStateTransition(enum regd_state request_state,
					 struct regulatory_request *pRequest)
{
	enum regd_state next_state, old_state;
	bool the_same = 0;

	old_state = g_mtk_regd_control.state;
	next_state = REGD_STATE_INVALID;

	if (old_state == REGD_STATE_INVALID)
		DBGLOG(RLM, ERROR,
		       "%s(): invalid state. trasntion is not allowed.\n",
		       __func__);

	switch (request_state) {
	case REGD_STATE_SET_WW_CORE:
		if ((old_state == REGD_STATE_SET_WW_CORE) ||
		    (old_state == REGD_STATE_INIT) ||
		    (old_state == REGD_STATE_SET_COUNTRY_USER) ||
		    (old_state == REGD_STATE_SET_COUNTRY_IE))
			next_state = request_state;
		break;
	case REGD_STATE_SET_COUNTRY_USER:
		/* Allow user to set multiple times */
		if ((old_state == REGD_STATE_SET_WW_CORE) ||
		    (old_state == REGD_STATE_INIT) ||
		    (old_state == REGD_STATE_SET_COUNTRY_USER) ||
		    (old_state == REGD_STATE_SET_COUNTRY_IE))
			next_state = request_state;
		else
			DBGLOG(RLM, ERROR, "Invalid old state = %d\n",
			       old_state);
		break;
	case REGD_STATE_SET_COUNTRY_DRIVER:
		if (old_state == REGD_STATE_SET_COUNTRY_USER) {
			/*
			 * Error.
			 * Mixing using set_country_by_user and
			 * set_country_by_driver is not allowed.
			 */
			break;
		}

		next_state = request_state;
		break;
	case REGD_STATE_SET_COUNTRY_IE:
		next_state = request_state;
		break;
	default:
		break;
	}

	if (next_state == REGD_STATE_INVALID) {
		DBGLOG(RLM, ERROR,
		       "%s():  ERROR. trasntion to invalid state. o=%x, r=%x, s=%x\n",
		       __func__, old_state, request_state, the_same);
	} else
		DBGLOG(RLM, INFO, "%s():  trasntion to state = %x (old = %x)\n",
		__func__, next_state, g_mtk_regd_control.state);

	g_mtk_regd_control.state = next_state;

	return g_mtk_regd_control.state;
}

/**
 * rlmDomainChannelFlagString - Transform channel flags to readable string
 *
 * @ flags: the ieee80211_channel->flags for a channel
 * @ buf: string buffer to put the transformed string
 * @ buf_size: size of the buf
 **/
void rlmDomainChannelFlagString(u32 flags, char *buf, size_t buf_size)
{
	int32_t buf_written = 0;

	if (!flags || !buf || !buf_size)
		return;

	if (flags & IEEE80211_CHAN_DISABLED) {
		LOGBUF(buf, ((int32_t)buf_size), buf_written, "DISABLED ");
		/* If DISABLED, don't need to check other flags */
		return;
	}
	if (flags & IEEE80211_CHAN_PASSIVE_FLAG)
		LOGBUF(buf, ((int32_t)buf_size), buf_written,
		       IEEE80211_CHAN_PASSIVE_STR " ");
	if (flags & IEEE80211_CHAN_RADAR)
		LOGBUF(buf, ((int32_t)buf_size), buf_written, "RADAR ");
	if (flags & IEEE80211_CHAN_NO_HT40PLUS)
		LOGBUF(buf, ((int32_t)buf_size), buf_written, "NO_HT40PLUS ");
	if (flags & IEEE80211_CHAN_NO_HT40MINUS)
		LOGBUF(buf, ((int32_t)buf_size), buf_written, "NO_HT40MINUS ");
	if (flags & IEEE80211_CHAN_NO_80MHZ)
		LOGBUF(buf, ((int32_t)buf_size), buf_written, "NO_80MHZ ");
	if (flags & IEEE80211_CHAN_NO_160MHZ)
		LOGBUF(buf, ((int32_t)buf_size), buf_written, "NO_160MHZ ");
}

void rlmDomainParsingChannel(IN struct wiphy *pWiphy)
{
	u32 band_idx, ch_idx;
	u32 ch_count;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *chan;
	struct CMD_DOMAIN_CHANNEL *pCh;
	char chan_flag_string[64] = {0};
#if (CFG_SUPPORT_REGD_UPDATE_DISCONNECT_ALLOWED == 1)
	struct GLUE_INFO *prGlueInfo;
	bool fgDisconnection = FALSE;
	uint8_t ucChannelNum = 0;
	uint32_t rStatus, u4BufLen;
#endif

	if (!pWiphy) {
		DBGLOG(RLM, ERROR, "%s():  ERROR. pWiphy = NULL.\n", __func__);
		ASSERT(0);
		return;
	}

#if (CFG_SUPPORT_REGD_UPDATE_DISCONNECT_ALLOWED == 1)
	/* Retrieve connected channel */
	prGlueInfo = rlmDomainGetGlueInfo();
	if (prGlueInfo && kalGetMediaStateIndicated(prGlueInfo) ==
	    MEDIA_STATE_CONNECTED) {
		ucChannelNum =
			wlanGetChannelNumberByNetwork(prGlueInfo->prAdapter,
			   prGlueInfo->prAdapter->prAisBssInfo->ucBssIndex);
	}
#endif
	/*
	 * Ready to parse the channel for bands
	 */

	rlmDomainResetActiveChannel();

	ch_count = 0;
	for (band_idx = 0; band_idx < KAL_NUM_BANDS; band_idx++) {
		sband = pWiphy->bands[band_idx];
		if (!sband)
			continue;

		for (ch_idx = 0; ch_idx < sband->n_channels; ch_idx++) {
			chan = &sband->channels[ch_idx];
			pCh = (rlmDomainGetActiveChannels() + ch_count);
			/* Parse flags and get readable string */
			rlmDomainChannelFlagString(chan->flags,
						   chan_flag_string,
						   sizeof(chan_flag_string));

			if (chan->flags & IEEE80211_CHAN_DISABLED) {
				DBGLOG(RLM, INFO,
				       "channels[%d][%d]: ch%d (freq = %d) flags=0x%x [ %s]\n",
				    band_idx, ch_idx, chan->hw_value,
				    chan->center_freq, chan->flags,
				    chan_flag_string);
#if (CFG_SUPPORT_REGD_UPDATE_DISCONNECT_ALLOWED == 1)
				/* Disconnect AP in the end of this function*/
				if (chan->hw_value == ucChannelNum)
					fgDisconnection = TRUE;
#endif
				continue;
			}

			/* Allowable channel */
			if (ch_count == MAX_SUPPORTED_CH_COUNT) {
				DBGLOG(RLM, ERROR,
				       "%s(): no buffer to store channel information.\n",
				       __func__);
				break;
			}
#if (CFG_SUPPORT_WIFI_6G == 1)
			/* 6G only add PSC channel */
			if (band_idx == NL80211_BAND_6GHZ &&
				((chan->hw_value - 5) % 16) != 0) {
				continue;
			}
#endif

			rlmDomainAddActiveChannel(band_idx);

			DBGLOG(RLM, INFO,
			       "channels[%d][%d]: ch%d (freq = %d) flgs=0x%x [%s]\n",
				band_idx, ch_idx, chan->hw_value,
				chan->center_freq, chan->flags,
				chan_flag_string);

			pCh->u2ChNum = chan->hw_value;
			pCh->eFlags = chan->flags;

			ch_count += 1;
		}

	}
#if (CFG_SUPPORT_REGD_UPDATE_DISCONNECT_ALLOWED == 1)
	/* Disconnect with AP if connected channel is disabled in new country */
	if (fgDisconnection) {
		DBGLOG(RLM, STATE, "%s(): Disconnect! CH%d is DISABLED\n",
		    __func__, ucChannelNum);
		rStatus = kalIoctl(prGlueInfo, wlanoidSetDisassociate,
				   NULL, 0, FALSE, FALSE, TRUE, &u4BufLen);

		if (rStatus != WLAN_STATUS_SUCCESS)
			DBGLOG(RLM, WARN, "disassociate error:%lx\n", rStatus);
	}
#endif
}
void rlmExtractChannelInfo(u32 max_ch_count,
			   struct CMD_DOMAIN_ACTIVE_CHANNEL_LIST *prBuff)
{
	u32 ch_count, idx;
	struct CMD_DOMAIN_CHANNEL *pCh;

	prBuff->u1ActiveChNum2g = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ);
	prBuff->u1ActiveChNum5g = rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ);
#if (CFG_SUPPORT_WIFI_6G == 1)
	prBuff->u1ActiveChNum6g = rlmDomainGetActiveChannelCount(KAL_BAND_6GHZ);
#else
	prBuff->u1ActiveChNum6g = 0;
#endif
	ch_count = prBuff->u1ActiveChNum2g +
			prBuff->u1ActiveChNum5g +
			prBuff->u1ActiveChNum6g;

	if (ch_count > max_ch_count) {
		ch_count = max_ch_count;
		DBGLOG(RLM, WARN,
		       "%s(); active channel list is not a complete one.\n",
		       __func__);
	}

	for (idx = 0; idx < ch_count; idx++) {
		pCh = &(prBuff->arChannels[idx]);

		pCh->u2ChNum = (rlmDomainGetActiveChannels() + idx)->u2ChNum;
		pCh->eFlags = (rlmDomainGetActiveChannels() + idx)->eFlags;
	}

}

const struct ieee80211_regdomain
*rlmDomainSearchRegdomainFromLocalDataBase(char *alpha2)
{
#if (CFG_SUPPORT_SINGLE_SKU_LOCAL_DB == 1)
	u8 idx;
	const struct mtk_regdomain *prRegd;

	idx = 0;
	while (g_prRegRuleTable[idx]) {
		prRegd = g_prRegRuleTable[idx];

		if ((prRegd->country_code[0] == alpha2[0]) &&
			(prRegd->country_code[1] == alpha2[1]) &&
			(prRegd->country_code[2] == alpha2[2]) &&
			(prRegd->country_code[3] == alpha2[3]))
			return prRegd->prRegdRules;

		idx++;
	}

	DBGLOG(RLM, ERROR,
	       "%s(): Error, Cannot find the correct RegDomain. country = %s\n",
	       __func__, alpha2);
	DBGLOG(RLM, INFO, "    Set as default WW.\n");

	return &default_regdom_ww; /*default world wide*/
#else
	return NULL;
#endif
}


const struct ieee80211_regdomain *rlmDomainGetLocalDefaultRegd(void)
{
#if (CFG_SUPPORT_SINGLE_SKU_LOCAL_DB == 1)
	return &default_regdom_ww;
#else
	return NULL;
#endif
}
struct GLUE_INFO *rlmDomainGetGlueInfo(void)
{
	return g_mtk_regd_control.pGlueInfo;
}

bool rlmDomainIsEfuseUsed(void)
{
	return g_mtk_regd_control.isEfuseCountryCodeUsed;
}

uint8_t rlmDomainGetChannelBw(enum ENUM_BAND eBand, uint8_t channelNum)
{
	uint32_t ch_idx = 0, start_idx = 0, end_idx = 0;
	uint8_t channelBw = MAX_BW_80_80_MHZ;
	struct CMD_DOMAIN_CHANNEL *pCh;
	enum ENUM_BAND eChBand;

	end_idx = rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ)
			+ rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ)
#if (CFG_SUPPORT_WIFI_6G == 1)
			+ rlmDomainGetActiveChannelCount(KAL_BAND_6GHZ)
#endif
		;

	for (ch_idx = start_idx; ch_idx < end_idx; ch_idx++) {
		pCh = (rlmDomainGetActiveChannels() + ch_idx);

		if (ch_idx < rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ))
			eChBand = BAND_2G4;
#if (CFG_SUPPORT_WIFI_6G == 1)
		else if (ch_idx < rlmDomainGetActiveChannelCount(KAL_BAND_2GHZ)
			+ rlmDomainGetActiveChannelCount(KAL_BAND_5GHZ))
			eChBand = BAND_5G;
		else
			eChBand = BAND_6G;
#else
		else
			eChBand = BAND_5G;
#endif

		if ((eChBand != eBand) || (pCh->u2ChNum != channelNum))
			continue;

		DBGLOG(RLM, TRACE, "band=%d, ch=%d, flags=%x\n",
			eBand, channelNum, pCh->eFlags);

		/* Max BW */
		if ((pCh->eFlags & IEEE80211_CHAN_NO_160MHZ)
						== IEEE80211_CHAN_NO_160MHZ)
			channelBw = MAX_BW_80MHZ;
		if ((pCh->eFlags & IEEE80211_CHAN_NO_80MHZ)
						== IEEE80211_CHAN_NO_80MHZ)
			channelBw = MAX_BW_40MHZ;
		if ((pCh->eFlags & IEEE80211_CHAN_NO_HT40)
						== IEEE80211_CHAN_NO_HT40)
			channelBw = MAX_BW_20MHZ;
	}

	DBGLOG(RLM, TRACE, "ch=%d, BW=%d\n", channelNum, channelBw);
	return channelBw;
}
#endif

uint32_t rlmDomainExtractSingleSkuInfoFromFirmware(IN struct ADAPTER *prAdapter,
						   IN uint8_t *pucEventBuf)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	struct SINGLE_SKU_INFO *prSkuInfo =
			(struct SINGLE_SKU_INFO *) pucEventBuf;

	g_mtk_regd_control.en = TRUE;

	if (prSkuInfo->isEfuseValid) {
		if (!rlmDomainIsUsingLocalRegDomainDataBase()) {

			DBGLOG(RLM, ERROR,
			       "%s(): Error. In efuse mode, must use local data base.\n",
			       __func__);

			ASSERT(0);
			/* force using local db if getting
			 * country code from efuse
			 */
			return WLAN_STATUS_NOT_SUPPORTED;
		}

		rlmDomainSetCountryCode((char *) &prSkuInfo->u4EfuseCountryCode,
					sizeof(prSkuInfo->u4EfuseCountryCode));
		g_mtk_regd_control.isEfuseCountryCodeUsed = TRUE;

	}
#endif

	return WLAN_STATUS_SUCCESS;
}

void rlmDomainSendInfoToFirmware(IN struct ADAPTER *prAdapter)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	struct regulatory_request request;
	struct regulatory_request *prReq = NULL;

	if (!regd_is_single_sku_en())
		return; /*not support single sku*/

	if (g_mtk_regd_control.isEfuseCountryCodeUsed) {
		request.initiator = NL80211_REGDOM_SET_BY_DRIVER;
		prReq = &request;
	}

	g_mtk_regd_control.pGlueInfo = prAdapter->prGlueInfo;
	mtk_reg_notify(wlanGetWiphy(), prReq);
#endif
}

enum ENUM_CHNL_EXT rlmSelectSecondaryChannelType(struct ADAPTER *prAdapter,
						 enum ENUM_BAND band,
						 u8 primary_ch)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	u8 below_ch, above_ch;

	below_ch = primary_ch - CHNL_SPAN_20;
	above_ch = primary_ch + CHNL_SPAN_20;

	if (rlmDomainIsLegalChannel(prAdapter, band, above_ch))
		return CHNL_EXT_SCA;

	if (rlmDomainIsLegalChannel(prAdapter, band, below_ch))
		return CHNL_EXT_SCB;

#endif

	return CHNL_EXT_SCN;
}

void rlmDomainOidSetCountry(IN struct ADAPTER *prAdapter, char *country,
			    u8 size_of_country)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	struct regulatory_request request;
	kalMemZero(&request, sizeof(request));

	if (rlmDomainIsUsingLocalRegDomainDataBase()) {
		rlmDomainSetTempCountryCode(country, size_of_country);
		request.initiator = NL80211_REGDOM_SET_BY_DRIVER;
		mtk_reg_notify(wlanGetWiphy(), &request);
	} else {
		DBGLOG(RLM, INFO,
		       "%s(): Using driver hint to query CRDA getting regd.\n",
		       __func__);
		regulatory_hint(wlanGetWiphy(), country);
	}
#endif
}

u32 rlmDomainGetCountryCode(void)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	return g_mtk_regd_control.alpha2;
#else
	return 0;
#endif
}

u32 rlmDomainGetTempCountryCode(void)
{
#if (CFG_SUPPORT_SINGLE_SKU == 1)
	return g_mtk_regd_control.tmp_alpha2;
#else
	return 0;
#endif
}

void rlmDomainAssert(u_int8_t cond)
{
	/* bypass this check because single sku is not enable */
	if (!regd_is_single_sku_en())
		return;

	if (!cond) {
		WARN_ON(1);
		DBGLOG(RLM, ERROR, "[WARNING!!] RLM unexpected case.\n");
	}

}


