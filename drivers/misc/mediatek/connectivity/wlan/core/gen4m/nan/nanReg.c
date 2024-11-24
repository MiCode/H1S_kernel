/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include "precomp.h"

/* Table E4 - Global Operating Classes */
#define REG_MAX_SUPPORT_CHANNEL 13
#define REG_MAX_DB_SIZE 25

struct _NAN_CHNL_REG_INFO_T {
	uint8_t ucOperatingClass;

	uint8_t ucBw;

	enum ENUM_CHNL_EXT eSco;
	/* For 40M Bw to determine SCB or SCA */

	uint8_t aucSupportChnlList[REG_MAX_SUPPORT_CHANNEL];
	/* BW80, BW160=> Center Channel
	 * BW20, BW40=> Primary Channel
	 */
};

/*******************************************
 * Table E4 - Global Operating Classes
 *******************************************
 */
struct _NAN_CHNL_REG_INFO_T g_rNanRegInfo[REG_MAX_DB_SIZE] = {
	{ 81, 20, CHNL_EXT_SCN, { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 } },
	{ 82, 20, CHNL_EXT_SCN, { 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 83, 40, CHNL_EXT_SCA, { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0 } },
	{ 84, 40, CHNL_EXT_SCB, { 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 0, 0, 0 } },
	{ 115,
	  20,
	  CHNL_EXT_SCN,
	  { 36, 40, 44, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 116, 40, CHNL_EXT_SCA, { 36, 44, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 117, 40, CHNL_EXT_SCB, { 40, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 118,
	  20,
	  CHNL_EXT_SCN,
	  { 52, 56, 60, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 119, 40, CHNL_EXT_SCA, { 52, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 120, 40, CHNL_EXT_SCB, { 56, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 121,
	  20,
	  CHNL_EXT_SCN,
	  { 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 0 } },
	{ 122,
	  40,
	  CHNL_EXT_SCA,
	  { 100, 108, 116, 124, 132, 140, 0, 0, 0, 0, 0, 0, 0 } },
	{ 123,
	  40,
	  CHNL_EXT_SCB,
	  { 104, 112, 120, 128, 136, 144, 0, 0, 0, 0, 0, 0, 0 } },
	{ 124,
	  20,
	  CHNL_EXT_SCN,
	  { 149, 153, 157, 161, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 125,
	  20,
	  CHNL_EXT_SCN,
	  { 149, 153, 157, 161, 165, 169, 0, 0, 0, 0, 0, 0, 0 } },
	{ 126,
	  40,
	  CHNL_EXT_SCA,
	  { 149, 157, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 127,
	  40,
	  CHNL_EXT_SCB,
	  { 153, 161, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 128,
	  80,
	  CHNL_EXT_SCN,
	  { 42, 58, 106, 122, 138, 155, 0, 0, 0, 0, 0, 0, 0 } },
	{ 129,
	  160,
	  CHNL_EXT_SCN,
	  { 50, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
	{ 130,
	  80,
	  CHNL_EXT_SCN,
	  { 42, 58, 106, 122, 138, 155, 0, 0, 0, 0, 0, 0, 0 } },
	{ 0,
	  0,
	  0,
	  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	    0 } } /* should be the last one */
};

/*******************************************
 * Table E4 - Global Operating Classes
 *******************************************
 */
uint8_t
nanRegFindRecordIdx(uint8_t ucOperatingClass) {
	int i;

	for (i = 0; i < REG_MAX_DB_SIZE; i++) {
		if (g_rNanRegInfo[i].ucOperatingClass == 0)
			break;

		if (ucOperatingClass == g_rNanRegInfo[i].ucOperatingClass)
			return i;
	}

	return REG_MAX_DB_SIZE;
}

/* NAN 2.0 Spec, Table 85. Setting for Primary Channel Bitmap */
uint8_t
nanRegGet20MHzPrimaryChnlIndex(uint8_t ucOperatingClass,
			       uint8_t ucPriChnlBitmap) {
	int32_t i4Idx;

	for (i4Idx = 7; i4Idx >= 0; i4Idx--) {
		if ((ucPriChnlBitmap >> i4Idx))
			return i4Idx;
	}

	return 0;
}

uint8_t
nanRegGetChannelByOrder(uint8_t ucOperatingClass, uint16_t *pu2ChnlBitmap) {
	int i, j;

	i = nanRegFindRecordIdx(ucOperatingClass);
	if (i != REG_MAX_DB_SIZE) {
		for (j = 0; j < REG_MAX_SUPPORT_CHANNEL; j++) {
			if ((*pu2ChnlBitmap) & BIT(j)) {
				*pu2ChnlBitmap &= (~(BIT(j)));

				if (g_rNanRegInfo[i].aucSupportChnlList[j] != 0)
					return g_rNanRegInfo[i]
						.aucSupportChnlList[j];
			}
		}
	}

	return REG_INVALID_INFO;
}

uint32_t
nanRegGetChannelBitmap(uint8_t ucOperatingClass, uint8_t ucChannel,
		       uint16_t *pu2ChnlBitmap) {
	int i, j;
	uint8_t *pucBuf;

	pucBuf = (uint8_t *)pu2ChnlBitmap;
	i = nanRegFindRecordIdx(ucOperatingClass);
	if (i != REG_MAX_DB_SIZE) {
		for (j = 0; j < REG_MAX_SUPPORT_CHANNEL; j++) {
			if (g_rNanRegInfo[i].aucSupportChnlList[j] == ucChannel)
				pucBuf[j / 8] |= BIT(j % 8);
		}
	}

	return WLAN_STATUS_SUCCESS;
}

uint8_t
nanRegGetBw(uint8_t ucOperatingClass) {
	int i;

	i = nanRegFindRecordIdx(ucOperatingClass);
	if (i != REG_MAX_DB_SIZE)
		return g_rNanRegInfo[i].ucBw;

	return REG_INVALID_INFO;
}

enum ENUM_CHNL_EXT
nanRegGetSco(uint8_t ucOperatingClass) {
	int i;

	i = nanRegFindRecordIdx(ucOperatingClass);
	if (i != REG_MAX_DB_SIZE)
		return g_rNanRegInfo[i].eSco;

	return REG_INVALID_INFO;
}

uint8_t
nanRegGetPrimaryChannel(uint8_t ucChannel, uint8_t ucBw, uint8_t ucNonContBw,
			uint8_t ucPriChnlIdx) {
	if ((ucBw == 20) || (ucBw == 40))
		return ucChannel;
	else if ((ucBw == 160) && (ucNonContBw == 0))
		ucChannel = ucChannel - 14 + (ucPriChnlIdx * 4);
	else
		ucChannel = ucChannel - 6 + (ucPriChnlIdx * 4);

	return ucChannel;
}

uint8_t
nanRegGetPrimaryChannelByOrder(uint8_t ucOperatingClass,
		uint16_t *pu2ChnlBitmap, uint8_t ucNonContBw,
		uint8_t ucPriChnlBitmap) {
	uint32_t i, j;
	uint8_t *pucBuf;

	pucBuf = (uint8_t *)pu2ChnlBitmap;
	i = nanRegFindRecordIdx(ucOperatingClass);
	if (i != REG_MAX_DB_SIZE) {
		for (j = 0; j < REG_MAX_SUPPORT_CHANNEL; j++) {
			if (pucBuf[j / 8] & BIT(j % 8)) {
				pucBuf[j / 8] &= (~(BIT(j % 8)));
				if (g_rNanRegInfo[i].aucSupportChnlList[j] !=
				    0) {
					return nanRegGetPrimaryChannel(
						g_rNanRegInfo[i]
							.aucSupportChnlList[j],
						nanRegGetBw(ucOperatingClass),
						ucNonContBw,
						nanRegGet20MHzPrimaryChnlIndex(
							ucOperatingClass,
							ucPriChnlBitmap));
				}
			}
		}
	}

	return REG_INVALID_INFO;
}

uint8_t
nanRegGetCenterChnlByPriChnl(uint8_t ucOperatingClass, uint8_t ucPrimaryChnl) {
	uint32_t i, j;
	uint8_t ucBw;
	uint8_t ucRang = 0;
	uint8_t ucCenterChnl;
	uint8_t ucChnl;

	ucCenterChnl = REG_INVALID_INFO;

	i = nanRegFindRecordIdx(ucOperatingClass);
	if (i != REG_MAX_DB_SIZE) {
		ucBw = g_rNanRegInfo[i].ucBw;
		if (ucBw == 20)
			ucRang = 0;
		else if (ucBw == 40)
			ucRang = 0;
		else if (ucBw == 80)
			ucRang = 6;
		else if (ucBw == 160)
			ucRang = 14;

		for (j = 0; j < REG_MAX_SUPPORT_CHANNEL; j++) {
			ucChnl = g_rNanRegInfo[i].aucSupportChnlList[j];
			if (ucChnl == 0)
				break;

			if (((ucChnl - ucRang) <= ucPrimaryChnl) &&
			    (ucPrimaryChnl <= (ucChnl + ucRang)))
				break;
		}

		if (ucChnl != 0) {
			if (ucBw == 40) {
				if (nanRegGetSco(ucOperatingClass) ==
				    CHNL_EXT_SCA)
					ucCenterChnl = ucChnl + 2;
				else if (nanRegGetSco(ucOperatingClass) ==
					 CHNL_EXT_SCB)
					ucCenterChnl = ucChnl - 2;
			} else {
				ucCenterChnl = ucChnl;
			}
		}
	}

	return ucCenterChnl;
}

uint8_t
nanRegGetOperatingClass(uint8_t ucBw, uint8_t ucChannel,
			enum ENUM_CHNL_EXT eSco) {
	int i, j;

	for (i = 0; i < REG_MAX_DB_SIZE; i++) {
		if (g_rNanRegInfo[i].ucOperatingClass == 0)
			break;

		if ((g_rNanRegInfo[i].ucBw == ucBw) &&
		    (g_rNanRegInfo[i].eSco == eSco)) {
			for (j = 0; j < REG_MAX_SUPPORT_CHANNEL; j++) {
				if (g_rNanRegInfo[i].aucSupportChnlList[j] ==
				    ucChannel)
					return g_rNanRegInfo[i]
						.ucOperatingClass;
				else if (g_rNanRegInfo[i]
						 .aucSupportChnlList[j] == 0)
					break;
			}
		}
	}

	return REG_INVALID_INFO;
}

union _NAN_BAND_CHNL_CTRL
nanRegGenNanChnlInfo(uint8_t ucPriChannel,
		enum ENUM_CHANNEL_WIDTH eChannelWidth,
		enum ENUM_CHNL_EXT eSco, uint8_t ucChannelS1,
		uint8_t ucChannelS2) {
	union _NAN_BAND_CHNL_CTRL rChnlInfo;
	uint8_t ucOperatingClass = REG_INVALID_INFO;

	rChnlInfo.u4RawData = 0;

	if (eChannelWidth >= CW_80P80MHZ) {
		DBGLOG(NAN, ERROR, "eChannelWidth is over!\n");
		return rChnlInfo;
	}
	if (eSco == CHNL_EXT_RES) {
		DBGLOG(NAN, ERROR, "eSco equals CHNL_EXT_RES!\n");
		return rChnlInfo;
	}
	switch (eChannelWidth) {
	case CW_20_40MHZ:
		if (eSco == CHNL_EXT_SCN)
			ucOperatingClass =
				nanRegGetOperatingClass(20, ucPriChannel, eSco);
		else if ((eSco == CHNL_EXT_SCA) || (eSco == CHNL_EXT_SCB))
			ucOperatingClass =
				nanRegGetOperatingClass(40, ucPriChannel, eSco);
		break;

	case CW_80MHZ:
	case CW_160MHZ:
		ucOperatingClass =
			nanRegGetOperatingClass(80, ucChannelS1, eSco);
		break;

	case CW_80P80MHZ:
		/* Fixme */
		break;

	default:
		break;
	}

	if (ucOperatingClass != REG_INVALID_INFO) {
		rChnlInfo.rChannel.u4Type =
			NAN_BAND_CH_ENTRY_LIST_TYPE_CHNL;
		rChnlInfo.rChannel.u4OperatingClass = ucOperatingClass;
		rChnlInfo.rChannel.u4PrimaryChnl = ucPriChannel;
		rChnlInfo.rChannel.u4AuxCenterChnl = 0; /* Fixme */
	}

	return rChnlInfo;
}

union _NAN_BAND_CHNL_CTRL
nanRegGenNanChnlInfoByPriChannel(uint8_t ucPriChannel, uint8_t ucBw) {
	uint32_t u4Idx;
	enum ENUM_CHANNEL_WIDTH eChannelWidth;
	enum ENUM_CHNL_EXT eSco;
	uint8_t ucChannelS1;
	uint8_t ucChannelS2;
	unsigned char fgFound = FALSE;
	uint8_t ucCenterChnl;

	for (u4Idx = 0; (u4Idx < REG_MAX_DB_SIZE) && !fgFound; u4Idx++) {
		if (g_rNanRegInfo[u4Idx].ucOperatingClass == 0)
			break;

		if (g_rNanRegInfo[u4Idx].ucBw != ucBw)
			continue;

		ucCenterChnl = nanRegGetCenterChnlByPriChnl(
			g_rNanRegInfo[u4Idx].ucOperatingClass, ucPriChannel);
		if (ucCenterChnl != REG_INVALID_INFO) {
			fgFound = TRUE;
			break;
		}
	}

	if (!fgFound)
		return g_rNullChnl;

	eSco = nanRegGetSco(g_rNanRegInfo[u4Idx].ucOperatingClass);

	ucChannelS1 = ucChannelS2 = 0;
	if ((ucBw == 20) || (ucBw == 40)) {
		eChannelWidth = CW_20_40MHZ;
	} else if (ucBw == 80) {
		eChannelWidth = CW_80MHZ;
		ucChannelS1 = ucCenterChnl;
	} else { /* 160 */
		eChannelWidth = CW_160MHZ;
		ucChannelS1 = ucCenterChnl;
	}

	return nanRegGenNanChnlInfo(ucPriChannel, eChannelWidth, eSco,
				    ucChannelS1, ucChannelS2);
}

uint32_t
nanRegConvertNanChnlInfo(union _NAN_BAND_CHNL_CTRL rChnlInfo,
			 uint8_t *pucPriChannel,
			 enum ENUM_CHANNEL_WIDTH *peChannelWidth,
			 enum ENUM_CHNL_EXT *peSco, uint8_t *pucChannelS1,
			 uint8_t *pucChannelS2) {
	uint8_t ucBw;

	if (!pucPriChannel || !peChannelWidth || !peSco || !pucChannelS1 ||
	    !pucChannelS2)
		return WLAN_STATUS_FAILURE;

	ucBw = nanRegGetBw(rChnlInfo.rChannel.u4OperatingClass);
	if (ucBw == REG_INVALID_INFO)
		return WLAN_STATUS_FAILURE;

	*pucPriChannel = rChnlInfo.rChannel.u4PrimaryChnl;

	*peSco = nanRegGetSco(rChnlInfo.rChannel.u4OperatingClass);

	*pucChannelS1 = *pucChannelS2 = 0;
	if ((ucBw == 20) || (ucBw == 40)) {
		*peChannelWidth = CW_20_40MHZ;
	} else if ((ucBw == 80) && (rChnlInfo.rChannel.u4AuxCenterChnl == 0)) {
		*peChannelWidth = CW_80MHZ;
		*pucChannelS1 = nanRegGetCenterChnlByPriChnl(
			rChnlInfo.rChannel.u4OperatingClass,
			rChnlInfo.rChannel.u4PrimaryChnl);
	} else if (ucBw == 160) {
		*peChannelWidth = CW_160MHZ;
		*pucChannelS1 = nanRegGetCenterChnlByPriChnl(
			rChnlInfo.rChannel.u4OperatingClass,
			rChnlInfo.rChannel.u4PrimaryChnl);
	} else if ((ucBw == 80) && (rChnlInfo.rChannel.u4AuxCenterChnl != 0)) {
		*peChannelWidth = CW_80P80MHZ;
		*pucChannelS1 = nanRegGetCenterChnlByPriChnl(
			rChnlInfo.rChannel.u4OperatingClass,
			rChnlInfo.rChannel.u4PrimaryChnl);
		*pucChannelS2 = rChnlInfo.rChannel.u4AuxCenterChnl;
	}

	return WLAN_STATUS_SUCCESS;
}
