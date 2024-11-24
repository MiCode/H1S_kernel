/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _EHT_IE_H
#define _EHT_IE_H

#if (CFG_SUPPORT_802_11BE == 1)

/*
 * Refer to Draft P802.11be_D0.4 @20210419
 * ID value is not defined yet.
 * Assume use to use EXT ID.
 */
/* EHT Capabilities */
#define EID_EXT_EHT_CAPS 254
/* EHT Operation */
#define EID_EXT_EHT_OP 253

#define EHT_MAC_CAP_BYTE_NUM                            (2)
#define EHT_PHY_CAP_BYTE_NUM                            (8)

struct _IE_EHT_CAP_T {
	u_int8_t  ucId;
	u_int8_t  ucLength;
	u_int8_t  ucExtId;
	u_int8_t  ucEhtMacCap[EHT_MAC_CAP_BYTE_NUM];
	u_int8_t  ucEhtPhyCap[EHT_PHY_CAP_BYTE_NUM];
	u_int8_t  aucVarInfo[0];
} __KAL_ATTRIB_PACKED__;

#define EHT_OP_BYTE_NUM (1)

struct _IE_EHT_OP_T {
	u_int8_t  ucId;
	u_int8_t  ucLength;
	u_int8_t  ucExtId;
	u_int8_t  ucEhtOpParams[EHT_OP_BYTE_NUM];
	u_int8_t  aucVarInfo[0];
} __KAL_ATTRIB_PACKED__;


#define EHT_RESET_MAC_CAP(_aucMacCapInfo) \
	(memset(_aucMacCapInfo, 0, EHT_MAC_CAP_BYTE_NUM))

#define EHT_RESET_PHY_CAP(_aucPhyCapInfo) \
	(memset(_aucPhyCapInfo, 0, EHT_PHY_CAP_BYTE_NUM))

#define EHT_RESET_OP(_aucOpInfo) \
	(memset(_aucOpInfo, 0, EHT_OP_BYTE_NUM))

/* EHT MAC Capabilities Information field */
/*
 * Indicates support for NSEP priority access.
 */
#define EHT_MAC_CAP_NSEP_PRI_ACCESS BIT(0)

#define SET_EHT_MAC_CAP_NSEP_PRI_ACCESS(_aucMacCapInfo) \
	(_aucMacCapInfo[0] |= EHT_MAC_CAP_NSEP_PRI_ACCESS)

/*
 * Indicates support for receiving a frame with an EHT OM Control subfield.
 */
#define EHT_MAC_CAP_OM_CTRL BIT(1)

#define SET_EHT_MAC_CAP_OM_CTRL(_aucMacCapInfo) \
		(_aucMacCapInfo[0] |= EHT_MAC_CAP_OM_CTRL)

/*
 * Indicates support for transmitting or responding
 * to a TXOP sharing trigger frame that does not solicit TB PPDU.
 */
#define EHT_MAC_CAP_TXOP_SHARING BIT(2)

#define SET_EHT_MAC_CAP_TXOP_SHARING(_aucMacCapInfo) \
		(_aucMacCapInfo[0] |= EHT_MAC_CAP_TXOP_SHARING)

/*
 * Indicates support for transmitting or responding
 * to a TXOP sharing trigger frame that does not solicit TB PPDU.
 */
#define EHT_MAC_CAP_TXOP_SHARING BIT(2)

#define SET_EHT_MAC_CAP_TXOP_SHARING(_aucMacCapInfo) \
		(_aucMacCapInfo[0] |= EHT_MAC_CAP_TXOP_SHARING)

/*
 * Indicates support for the restricted TWT operation.
 */
#define EHT_MAC_CAP_RESTRICTED_TWT BIT(3)

#define SET_EHT_MAC_CAP_RESTRICTED_TWT(_aucMacCapInfo) \
		(_aucMacCapInfo[0] |= EHT_MAC_CAP_RESTRICTED_TWT)

/*
 * Indicates support for transmission and
 * reception of SCS Descriptor elements containing a TSPEC subelement.
 */
#define EHT_MAC_CAP_SCS BIT(4)

#define SET_EHT_MAC_CAP_SCS(_aucMacCapInfo) \
		(_aucMacCapInfo[0] |= EHT_MAC_CAP_SCS)

/*
 * For an AP, indicates support for receiving a frame
 * with an AAR Control subfield.
 * For a non-AP STA, indicates support for generating a frame
 * with an AAR Control subfield.
 */
#define EHT_MAC_CAP_AAR BIT(5)

#define SET_EHT_MAC_CAP_AAR(_aucMacCapInfo) \
		(_aucMacCapInfo[0] |= EHT_MAC_CAP_AAR)


/* EHT PHY Capabilities Information field */

/* Support Of MCS 15 subfield */
enum eht_mru_set_mcs15 {
	MRU_106TONE_OR_52TONE_W_26TONE = 1,
	MRU_484TONE_W_242TONE_IN_80M = (1 << 1),
	MRU_996TONE_TO_242TONE_IN_160M = (1 << 2),
	MRU_TRIPPLE_996TONE_IN_320M = (1 << 3),
};

/* Common Nominal Packet Padding */
enum eht_common_nominal_padd_us {
	COMMON_NOMINAL_PAD_0_US,
	COMMON_NOMINAL_PAD_8_US,
	COMMON_NOMINAL_PAD_16_US,
	COMMON_NOMINAL_PAD_20_US,
};

/* Maximum Number Of Supported EHT-LTFs */
enum eht_max_ltf {
	MAX_EHT_LTF_NUM_4,
	MAX_EHT_LTF_NUM_8,
};

enum eht_bf_caps {
	EHT_SU_BFER = 1,
	EHT_SU_BFEE = 1 << 1,
	EHT_MU_BFER = 1 << 2,
	EHT_BFEE_NG16_SU_FEEDBACK = 1 << 3,
	EHT_BFEE_NG32_SU_FEEDBACK = 1 << 4,
	EHT_BFEE_CODEBOOK_4_2_SU_FEEDBACK = 1 << 5,
	EHT_BFEE_CODEBOOK_7_5_SU_FEEDBACK = 1 << 6,
	EHT_TRIGED_SU_BF_FEEDBACK = 1 << 7,
	EHT_TRIGED_MU_BF_PATIAL_BW_FEEDBACK = 1 << 8,
};

enum eht_gi_caps {
	EHT_NDP_4X_EHT_LTF_3DOT2US_GI = 1,
	EHT_MU_PPDU_4X_EHT_LTF_DOT8US_GI = 1 << 1,
};

enum eht_mcs15_mru_support {
	EHT_MCS15_MRU_106_or_52_w_26_tone = 1,
	EHT_MCS15_MRU_484_w_242_tone_80M = 1 << 1,
	EHT_MCS15_MRU_996_to_242_tone_160M = 1 << 2,
	EHT_MCS15_MRU_3x996_tone_320M = 1 << 3,
};

enum eht_phy_caps {
	EHT_320M_6G = 1,
	EHT_242_TONE_RU_WT_20M = 1 << 1,
	EHT_PARTIAL_BW_UL_MI_MIMO  = 1 << 2,
	EHT_TRIGED_CQI_FEEDBACK = 1 << 3,
	EHT_PARTIAL_BW_DL_MU_MIMO = 1 << 4,
	EHT_PSR_BASED_SR = 1 << 5,
	EHT_POWER_BOOST_FACTOR = 1 << 6,
	EHT_NON_TRIGED_CQI_FEEDBACK = 1 << 7,
	EHT_TX_1024QAM_4096QAM_LE_242_TONE_RU = 1 << 8,
	EHT_RX_1024QAM_4096QAM_LE_242_TONE_RU = 1 << 9,
	EHT_PPE_THRLD_PRESENT = 1 << 10,
	EHT_EXTRA_LTF_SUPPORT = 1 << 11,
	EHT_DUP_6G = 1 << 12,
	EHT_20M_RX_NDP_W_WIDER_BW = 1 << 13,
};

struct eht_bf_info {
	enum eht_bf_caps bf_cap;
	uint8_t bfee_ss_le_eq_bw80;
	uint8_t bfee_ss_bw160;
	uint8_t bfee_ss_bw320;
	uint8_t snd_dim_le_eq_bw80;
	uint8_t snd_dim_bw160;
	uint8_t snd_dim_bw320;
	uint8_t bfee_max_nc;
	uint8_t max_ltf_num_su_non_ofdma;
	uint8_t max_ltf_num_mu;
};

struct GNU_PACKED eht_phy_capinfo {
	uint32_t phy_capinfo_1;
	uint32_t phy_capinfo_2;
};

 #define SET_CAP_BITS(_cap, _capinfo, _value)\
{\
	_capinfo &= ~_cap##_MASK;\
	_capinfo |= ((_value << _cap##_SHIFT)\
			& _cap##_MASK);\
}

#define GET_CAP_BITS(_cap, _capinfo)\
	(((_capinfo) & _cap##_MASK)\
		>> _cap##_SHIFT)

#define CAP_BITS_MASK(_cap)\
	BITS(_cap##_SHIFT, (_cap##_SHIFT + _cap##_BITS - 1))

/* phy_capinfo_1: bit0..bit31 */
#define DOT11BE_PHY_CAP_320M_6G BIT(1)
#define DOT11BE_PHY_CAP_242_TONE_RU_WT_20M BIT(2)
#define DOT11BE_PHY_CAP_NDP_4X_EHT_LTF_3DOT2US_GI BIT(3)
#define DOT11BE_PHY_CAP_PARTIAL_BW_UL_MU_MIMO BIT(4)
#define DOT11BE_PHY_CAP_SU_BFER BIT(5)
#define DOT11BE_PHY_CAP_SU_BFEE BIT(6)
#define DOT11BE_PHY_CAP_MU_BFER BIT(7)
#define DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M_BITS 3
#define DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M_SHIFT 7
#define DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M_MASK	\
	CAP_BITS_MASK(DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M)
#define SET_DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_BFEE_SS_LE_EQ_80M, _capinfo)
#define DOT11BE_PHY_CAP_BFEE_SS_160M_BITS 3
#define DOT11BE_PHY_CAP_BFEE_SS_160M_SHIFT 10
#define DOT11BE_PHY_CAP_BFEE_SS_160M_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_BFEE_SS_160M)
#define SET_DOT11BE_PHY_CAP_BFEE_160M(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_BFEE_SS_160M, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_BFEE_160M(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_BFEE_SS_160M, _capinfo)
#define DOT11BE_PHY_CAP_BFEE_SS_320M_BITS 3
#define DOT11BE_PHY_CAP_BFEE_SS_320M_SHIFT 13
#define DOT11BE_PHY_CAP_BFEE_SS_320M_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_BFEE_SS_320M)
#define SET_DOT11BE_PHY_CAP_BFEE_320M(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_BFEE_SS_320M, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_BFEE_320M(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_BFEE_SS_320M, _capinfo)
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M_BITS 3
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M_SHIFT 16
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M)
#define SET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_SOUND_DIM_NUM_LE_EQ_80M, _capinfo)
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M_BITS 3
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M_SHIFT 19
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M)
#define SET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_SOUND_DIM_NUM_160M, _capinfo)
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M_BITS 3
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M_SHIFT 22
#define DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M)
#define SET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_SOUND_DIM_NUM_320M, _capinfo)
#define DOT11BE_PHY_CAP_NG16_SU_FEEDBACK BIT(26)
/* TODO: name and definition do not match, table 9-322ap */
#define DOT11BE_PHY_CAP_NG32_SU_FEEDBACK BIT(27)
#define DOT11BE_PHY_CAP_CODEBOOK_4_2_SU_FEEDBACK BIT(28)
/* TODO: name and definition do not match, table 9-322ap */
#define DOT11BE_PHY_CAP_CODEBOOK_7_5_SU_FEEDBACK BIT(29)
#define DOT11BE_PHY_CAP_TRIGED_SU_BF_FEEDBACK BIT(30)
#define DOT11BE_PHY_CAP_TRIGED_MU_BF_PARTIAL_BW_FEEDBACK BIT(31)

/* phy_capinfo_2: bit32..bit63 */
#define DOT11BE_PHY_CAP_TRIGED_CQI_FEEDBACK BIT(0)
#define DOT11BE_PHY_CAP_PARTIAL_BW_DL_MU_MIMO BIT(1)
#define DOT11BE_PHY_CAP_PSR_BASED_SR BIT(2)
#define DOT11BE_PHY_CAP_POWER_BOOST_FACTOR BIT(3)
#define DOT11BE_PHY_CAP_EHT_MU_PPDU_4X_EHT_LTF_DOT8US_GI BIT(4)
#define DOT11BE_PHY_CAP_MAX_NC_BITS 3
#define DOT11BE_PHY_CAP_MAX_NC_SHIFT 4
#define DOT11BE_PHY_CAP_MAX_NC_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_MAX_NC)
#define SET_DOT11BE_PHY_CAP_MAX_NC(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_MAX_NC, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_MAX_NC(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_MAX_NC, _capinfo)
#define DOT11BE_PHY_CAP_NON_TRIGED_CQI_FEEDBACK BIT(8)
#define DOT11BE_PHY_CAP_TX_1024QAM_4096QAM_LE_242_TONE_RU BIT(9)
#define DOT11BE_PHY_CAP_RX_1024QAM_4096QAM_LE_242_TONE_RU BIT(10)
#define DOT11BE_PHY_CAP_PPE_THRLD_PRESENT BIT(11)
#define DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD_BITS 2
#define DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD_SHIFT 11
#define DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD)
#define SET_DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_COMMON_NOMINAL_PKT_PAD, _capinfo)
#define DOT11BE_PHY_CAP_EXTRA_MAX_EHT_LTF_NUM_SU BIT(13)
#define DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU_BITS 2
#define DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU_SHIFT 14
#define DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU)
#define SET_DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_SU, _capinfo)
#define DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU_BITS 2
#define DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU_SHIFT 16
#define DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU)
#define SET_DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_MAX_EHT_LTF_NUM_MU, _capinfo)
#define DOT11BE_PHY_CAP_MCS_15_BITS 4
#define DOT11BE_PHY_CAP_MCS_15_SHIFT 18
#define DOT11BE_PHY_CAP_MCS_15_MASK \
	CAP_BITS_MASK(DOT11BE_PHY_CAP_MCS_15)
#define SET_DOT11BE_PHY_CAP_MCS_15(_capinfo, _value) \
	SET_CAP_BITS(DOT11BE_PHY_CAP_MCS_15, _capinfo, _value)
#define GET_DOT11BE_PHY_CAP_MCS_15(_capinfo) \
	GET_CAP_BITS(DOT11BE_PHY_CAP_MCS_15, _capinfo)
#define DOT11BE_PHY_CAP_EHT_DUP_6G BIT(23)
#define DOT11BE_PHY_CAP_20M_RX_NDP_W_WIDER_BW BIT(24)

#endif /* CFG_SUPPORT_802_11BE == 1 */
#endif /* !_EHT_IE_H */
