/*
 * SPDX-License-Identifier: GPL-2.0
 * Copyright (c) 2019 MediaTek Inc.
 */

/*! \file   "ie_sort.c"
 *  \brief
 */

#include "precomp.h"

struct IE_ORDER_TABLE_INFO assoc_req_ie_order[] = {
	{3, ELEM_ID_SSID, 0}, /* SSID */
	/* Supported Rates and BSS Membership Selectors */
	{4, ELEM_ID_SUP_RATES, 0},
	/* Extended Supported Rates and BSS Membership Selectors */
	{5, ELEM_ID_EXTENDED_SUP_RATES, 0},
	/* Power Capability */
	{6, ELEM_ID_PWR_CAP, 0},
	/* Supported Channels */
	{7, ELEM_ID_SUP_CHS, 0},
	/* RSN */
	{8, ELEM_ID_RSN, 0},
	/* QoS Capability */
	{9, ELEM_ID_QOS_CAP, 0},
	/* RM Enabled Capabilities */
	{10, ELEM_ID_RRM_ENABLED_CAP, 0},
	/* Mobility Domain */
	{11, ELEM_ID_MOBILITY_DOMAIN, 0},
	/* Supported Operating Classes */
	{12, ELEM_ID_SUP_OPERATING_CLASS, 0},
	/* HT Capabilities */
	{13, ELEM_ID_HT_CAP, 0},
	/* 20/40 BSS Coexistence */
	{14, ELEM_ID_20_40_BSS_COEXISTENCE, 0},
	/* Extended Capabilities */
	{15, ELEM_ID_EXTENDED_CAP, 0},
	/* QoS Traffic Capability */
	{16, ELEM_ID_QOS_TRAFFIC_CAP, 0},
	/* TIM Broadcast Request */
	{17, ELEM_ID_TIM_BROADCAST_REQ, 0},
	/* Interworking */
	{18, ELEM_ID_INTERWORKING, 0},
	/* Multi-band */
	{19, ELEM_ID_MULTI_BAND, 0},
	/* DMG Capabilities */
	{20, ELEM_ID_DMG_CAP, 0},
	/* Multiple MAC Sublayers */
	{21, ELEM_ID_MULTI_MAC_SUBLAYERS, 0},
	/* VHT Capabilities */
	{22, ELEM_ID_VHT_CAP, 0},
	/* Operating Mode Notification */
	{23, ELEM_ID_OP_MODE, 0},
	/* FILS Session */
	{24, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_SESSION},
	/* FILS Public Key */
	{25, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_PUBLIC_KEY},
	/* FILS Key Confirmation */
	{26, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_KEY_CONFIRM},
	/* FILS HLP Container */
	{27, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_HLP_CONTAINER},
	/* FILS IP Address Assignment */
	{28, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_IP_ADDR_ASSIGN},
	/* TWT */
	{29, ELEM_ID_TWT, 0},
	/* AID Request */
	{30, ELEM_ID_AID_REQ, 0},
	/* S1G Capabilities */
	{31, ELEM_ID_S1G_CAP, 0},
	/* EL Operation */
	{32, ELEM_ID_EL_OP, 0},
	/* S1G Relay */
	{33, ELEM_ID_S1G_RELAY, 0},
	/* BSS Max Idle Period */
	{34, ELEM_ID_BSS_MAX_IDLE_PERIOD, 0},
	/* Header Compression */
	{35, ELEM_ID_HEADER_COMPRESSION, 0},
	/* MAD */
	{36, ELEM_ID_MAD, 0},
	/* Reachable Address */
	{37, ELEM_ID_REACHABLE_ADDR, 0},
	/* S1G Relay Activation */
	{38, ELEM_ID_S1G_RELAY_ACTIVATION, 0},
	/* CDMG Capabilities */
	{39, ELEM_ID_RESERVED, ELEM_EXT_ID_CDMG_CAP},
	/* CMMG Capabilities */
	{40, ELEM_ID_RESERVED, ELEM_EXT_ID_CMMG_CAP},
	/* GLK-GCR Parameter Set */
	{41, ELEM_ID_RESERVED, ELEM_EXT_ID_GLK_GCR_PARAM_SET},
	/* Fast BSS Transition */
	{42, ELEM_ID_FAST_TRANSITION, 0},
	/* RSN Extension */
	{43, ELEM_ID_RSNX, 0},
	/* Supplemental Class 2 Capabilities */
	{44, ELEM_ID_RESERVED, ELEM_EXT_ID_SUPPLEMENTAL_CLASS2_CAP},
	/* MSCS Descriptor */
	{45, ELEM_ID_RESERVED, ELEM_EXT_ID_MSCS_DESCRIPTOR},
	/* HE Capabilities */
	{46, ELEM_ID_RESERVED, ELEM_EXT_ID_HE_CAP},
	/* Channel Switch Timing */
	{47, ELEM_ID_CH_SWITCH_TIMING, 0},
	/* HE 6 GHz Band Capabilities */
	{48, ELEM_ID_RESERVED, ELEM_EXT_ID_HE_6G_BAND_CAP},
	/* UL MU Power Capabilities */
	{49, ELEM_ID_RESERVED, ELEM_EXT_ID_UL_MU_Power_CAP},
	/* TWT Constraint Parameters */
	{50, ELEM_ID_RESERVED, 0},
	/* BSS AC Access Delay/WAPI Parameter Set
	 * Note: This field is absent in order table of 802.11-2020
	 * but we need to handle it for WAPI connection
	 */
	{98, ELEM_ID_BSS_AC_ACCESS_DELAY, 0},
	/* Diffie-Hellman Parameter
	 * Note: This field is absent in order table of 802.11-2020
	 * but we need to handle it for OWE connection
	 */
	{99, ELEM_ID_RESERVED, ELEM_EXT_ID_DIFFIE_HELLMAN_PARAM},
	/* Vendor Specific: should be present in the last */
	{100, ELEM_ID_VENDOR, 0}
};

struct IE_ORDER_TABLE_INFO reassoc_req_ie_order[] = {
	{4, ELEM_ID_SSID, 0}, /* SSID */
	/* Supported Rates and BSS Membership Selectors */
	{5, ELEM_ID_SUP_RATES, 0},
	/* Extended Supported Rates and BSS Membership Selectors */
	{6, ELEM_ID_EXTENDED_SUP_RATES, 0},
	/* Power Capability */
	{7, ELEM_ID_PWR_CAP, 0},
	/* Supported Channels */
	{8, ELEM_ID_SUP_CHS, 0},
	/* RSN */
	{9, ELEM_ID_RSN, 0},
	/* QoS Capability */
	{10, ELEM_ID_QOS_CAP, 0},
	/* RM Enabled Capabilities */
	{11, ELEM_ID_RRM_ENABLED_CAP, 0},
	/* Mobility Domain */
	{12, ELEM_ID_MOBILITY_DOMAIN, 0},
	/* Fast BSS Transition */
	{13, ELEM_ID_FAST_TRANSITION, 0},
	/* Resource information container */
	{14, ELEM_ID_RESOURCE_INFO_CONTAINER, 0},
	/* Supported Operating Classes */
	{15, ELEM_ID_SUP_OPERATING_CLASS, 0},
	/* HT Capabilities */
	{16, ELEM_ID_HT_CAP, 0},
	/* 20/40 BSS Coexistence */
	{17, ELEM_ID_20_40_BSS_COEXISTENCE, 0},
	/* Extended Capabilities */
	{18, ELEM_ID_EXTENDED_CAP, 0},
	/* QoS Traffic Capability */
	{19, ELEM_ID_QOS_TRAFFIC_CAP, 0},
	/* TIM Broadcast Request */
	{20, ELEM_ID_TIM_BROADCAST_REQ, 0},
	/* FMS Request */
	{21, ELEM_ID_FMS_REQUEST, 0},
	/* DMS Request */
	{22, ELEM_ID_DMS_REQUEST, 0},
	/* Interworking */
	{23, ELEM_ID_INTERWORKING, 0},
	/* Multi-band */
	{24, ELEM_ID_MULTI_BAND, 0},
	/* DMG Capabilities */
	{25, ELEM_ID_DMG_CAP, 0},
	/* Multiple MAC Sublayers */
	{26, ELEM_ID_MULTI_MAC_SUBLAYERS, 0},
	/* VHT Capabilities */
	{27, ELEM_ID_VHT_CAP, 0},
	/* Operating Mode Notification */
	{28, ELEM_ID_OP_MODE, 0},
	/* FILS Session */
	{29, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_SESSION},
	/* FILS Public Key */
	{30, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_PUBLIC_KEY},
	/* FILS Key Confirmation */
	{31, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_KEY_CONFIRM},
	/* FILS HLP Container */
	{32, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_HLP_CONTAINER},
	/* FILS IP Address Assignment */
	{33, ELEM_ID_RESERVED, ELEM_EXT_ID_FILS_IP_ADDR_ASSIGN},
	/* TWT */
	{34, ELEM_ID_TWT, 0},
	/* AID Request */
	{35, ELEM_ID_AID_REQ, 0},
	/* S1G Capabilities */
	{36, ELEM_ID_S1G_CAP, 0},
	/* EL Operation */
	{37, ELEM_ID_EL_OP, 0},
	/* BSS Max Idle Period */
	{38, ELEM_ID_BSS_MAX_IDLE_PERIOD, 0},
	/* S1G Relay */
	{39, ELEM_ID_S1G_RELAY, 0},
	/* Header Compression */
	{40, ELEM_ID_HEADER_COMPRESSION, 0},
	/* MAD */
	{41, ELEM_ID_MAD, 0},
	/* Reachable Address */
	{42, ELEM_ID_REACHABLE_ADDR, 0},
	/* S1G Relay Activation */
	{43, ELEM_ID_S1G_RELAY_ACTIVATION, 0},
	/* CDMG Capabilities */
	{44, ELEM_ID_RESERVED, ELEM_EXT_ID_CDMG_CAP},
	/* CMMG Capabilities */
	{45, ELEM_ID_RESERVED, ELEM_EXT_ID_CMMG_CAP},
	/* OCI */
	{46, ELEM_ID_RESERVED, ELEM_EXT_ID_OCI},
	/* GLK-GCR Parameter Set */
	{47, ELEM_ID_RESERVED, ELEM_EXT_ID_GLK_GCR_PARAM_SET},
	/* RSN Extension */
	{48, ELEM_ID_RSNX, 0},
	/* Supplemental Class 2 Capabilities */
	{49, ELEM_ID_RESERVED, ELEM_EXT_ID_SUPPLEMENTAL_CLASS2_CAP},
	/* MSCS Descriptor */
	{50, ELEM_ID_RESERVED, ELEM_EXT_ID_MSCS_DESCRIPTOR},
	/* HE Capabilities */
	{51, ELEM_ID_RESERVED, ELEM_EXT_ID_HE_CAP},
	/* Channel Switch Timing */
	{52, ELEM_ID_CH_SWITCH_TIMING, 0},
	/* HE 6 GHz Band Capabilities */
	{53, ELEM_ID_RESERVED, ELEM_EXT_ID_HE_6G_BAND_CAP},
	/* UL MU Power Capabilities */
	{54, ELEM_ID_RESERVED, ELEM_EXT_ID_UL_MU_Power_CAP},
	/* BSS AC Access Delay/WAPI Parameter Set
	 * Note: This field is absent in order table of 802.11-2020
	 * but we need to handle it for WAPI connection
	 */
	{98, ELEM_ID_BSS_AC_ACCESS_DELAY, 0},
	/* Diffie-Hellman Parameter
	 * Note: This field is absent in order table of 802.11-2020
	 * but we need to handle it for OWE connection
	 */
	{99, ELEM_ID_RESERVED, ELEM_EXT_ID_DIFFIE_HELLMAN_PARAM},
	/* Vendor Specific: should be present in the last */
	{100, ELEM_ID_VENDOR, 0}
};

void sortAssocReqIE(IN struct ADAPTER *prAdapter,
			IN struct MSDU_INFO *prMsduInfo, IN uint8_t fgIsReAssoc)
{
	uint32_t beginOffset, start, end, frameLen, frameSearch, searchCount;
	uint32_t eid, exteid, orderIdx;
	uint16_t txAssocReqIENums;
	const uint8_t *primary_IE;
	struct MSDU_INFO *prMsduInfoInOrder;
	uint8_t  *pucBufferInOrder;

	prMsduInfoInOrder =
		cnmMgtPktAlloc(prAdapter, prMsduInfo->u2FrameLength);
	if (prMsduInfoInOrder == NULL) {
		DBGLOG(SAA, WARN,
			"No PKT_INFO_T for sending MLD STA. Don't reorder IE\n");
		return;
	}
	pucBufferInOrder =
		(uint8_t *) ((unsigned long) prMsduInfoInOrder->prPacket);

	end = prMsduInfo->u2FrameLength;
	if (fgIsReAssoc) {
		start = beginOffset =
		  OFFSET_OF(struct WLAN_REASSOC_REQ_FRAME, aucInfoElem[0]);
		txAssocReqIENums = sizeof(reassoc_req_ie_order) /
					sizeof(struct IE_ORDER_TABLE_INFO);
	} else {
		start = beginOffset =
		  OFFSET_OF(struct WLAN_ASSOC_REQ_FRAME, aucInfoElem[0]);
		txAssocReqIENums = sizeof(assoc_req_ie_order) /
					sizeof(struct IE_ORDER_TABLE_INFO);
	}
	frameLen = end - start;
	frameSearch = 0;
	searchCount = 0;
	orderIdx = 0;

	while (orderIdx < txAssocReqIENums) {
		if (fgIsReAssoc) {
			eid = reassoc_req_ie_order[orderIdx].eid;
			exteid = reassoc_req_ie_order[orderIdx].extid;
		} else {
			eid = assoc_req_ie_order[orderIdx].eid;
			exteid = assoc_req_ie_order[orderIdx].extid;
		}
		primary_IE = kalFindIeExtIE(eid, exteid,
			(uint8_t *)prMsduInfo->prPacket + start,
			(end - start));
		if (primary_IE) {
			frameSearch += IE_SIZE(primary_IE);
			if (frameSearch > frameLen)
				break;

			start = (primary_IE - (uint8_t *)prMsduInfo->prPacket)
				+ IE_SIZE(primary_IE);

			DBGLOG(RLM, LOUD,
				"find eid(%d) extid(%d)IE: id(%d), size(%d), start(%d), frameSearch(%d), frameLen(%d)\n",
				eid, exteid, IE_ID(primary_IE),
				IE_SIZE(primary_IE), start,
				frameSearch, frameLen);
			kalMemCopy(pucBufferInOrder, primary_IE,
					IE_SIZE(primary_IE));
			DBGLOG_MEM8(RLM, LOUD, pucBufferInOrder,
					IE_SIZE(primary_IE));
			pucBufferInOrder += IE_SIZE(primary_IE);
		} else {
			orderIdx++;
			/* no duplicated IE left, move back to the front */
			start = beginOffset;
		}
		searchCount++;
	}
	if (frameSearch != frameLen) {
		DBGLOG(RLM, WARN,
			"Cannot search all assoc req IE! Don't reorder IE\n");
	} else {
		/* copy ordered frame back to prMsduInfo */
		kalMemCopy((uint8_t *) ((unsigned long)
			prMsduInfo->prPacket + beginOffset),
			(uint8_t *) ((unsigned long)
			prMsduInfoInOrder->prPacket),
			frameLen);
	}
	cnmMgtPktFree(prAdapter, prMsduInfoInOrder);

}
