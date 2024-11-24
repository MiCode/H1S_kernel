/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * hostapd / WPA authenticator glue code
 * Copyright (c) 2002-2012, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */
#include "wpa_supp/FourWayHandShake.h"
#include "wpa_supp/src/ap/wpa_auth_glue.h"
#include "wpa_supp/src/ap/ieee802_1x.h"
#include "wpa_supp/src/eapol_auth/eapol_auth_sm_i.h"
/*#include "wpa_supp/wpaSuppCmdEvt.h"*/

/*====== DLM pre-allocation*/
struct hostapd_data g_rHapdData;
struct hostapd_iface g_rHapdIface;
struct hostapd_config g_rHapdConf;
struct hostapd_bss_config g_rHapdBssConf;

/*====== Patch point*/
void (*hostapd_config_defaults_bss)(struct hostapd_bss_config *bss) =
	_hostapd_config_defaults_bss;

int
secRxProcessEapol_AP(IN struct SW_RFB *prSwRfb, uint8_t *pucSendAddr) {
#ifndef CFG_SUPPORT_NAN
	int ret = 0;

	DBGLOG(RSN, INFO, ("[%s] Enter\n", __func__));

	/*wpa_hexdump(MSG_DEBUG, "prSwRfb->pucPayload:", */
	/*		prSwRfb->pucPayload, prSwRfb->u2PayloadLength);*/

	ret = ieee802_1x_receive(
		g_aprBssInfo[prSwRfb->ucBssIndex]->prHostapdData, pucSendAddr,
		prSwRfb->pucPayload, prSwRfb->u2PayloadLength);
	return ret;
#else
	return 0;
#endif
}

static int
ieee802_11_parse_vendor_specific(const u8 *pos, size_t elen,
				 struct ieee802_11_elems *elems,
				 int show_errors) {
	unsigned int oui;

	/* first 3 bytes in vendor specific information element are the IEEE
	 * OUI of the vendor. The following byte is used a vendor specific
	 * sub-type.
	 */
	if (elen < 4) {
		if (show_errors) {
			/*wpa_printf(MSG_MSGDUMP, "short vendor specific "
			*	"information element ignored (len=%lu)",
			*	(unsigned long) elen);
			*/
		}
		return -1;
	}

	oui = WPA_GET_BE24(pos);
	switch (oui) {
	case OUI_MICROSOFT:
		/* Microsoft/Wi-Fi information elements are further typed and
		 * subtyped
		 */
		switch (pos[3]) {
		case 1:
			/* Microsoft OUI (00:50:F2) with OUI Type 1:
			 * real WPA information element
			 */
			elems->wpa_ie = pos;
			elems->wpa_ie_len = elen;
			break;
		case WMM_OUI_TYPE:
			/* WMM information element */
			if (elen < 5) {
				wpa_printf(
					MSG_MSGDUMP,
					"short WMM information element ignored (len=%lu)",
					(unsigned long)elen);
				return -1;
			}
			switch (pos[4]) {
			case WMM_OUI_SUBTYPE_INFORMATION_ELEMENT:
			case WMM_OUI_SUBTYPE_PARAMETER_ELEMENT:
				/*
				 * Share same pointer since only one of these
				 * is used and they start with same data.
				 * Length field can be used to distinguish the
				 * IEs.
				 */
				elems->wmm = pos;
				elems->wmm_len = elen;
				break;
			case WMM_OUI_SUBTYPE_TSPEC_ELEMENT:
				elems->wmm_tspec = pos;
				elems->wmm_tspec_len = elen;
				break;
			default:
				wpa_printf(
					MSG_EXCESSIVE,
					"unknown WMM information element ignored (subtype=%d len=%lu)",
					pos[4], (unsigned long)elen);
				return -1;
			}
			break;
		case 4:
			/* Wi-Fi Protected Setup (WPS) IE */
			elems->wps_ie = pos;
			elems->wps_ie_len = elen;
			break;
		default:
			wpa_printf(
				MSG_EXCESSIVE,
				"Unknown Microsoft information element ignored (type=%d len=%lu)",
				pos[3], (unsigned long)elen);
			return -1;
		}
		break;

	case OUI_WFA:
		switch (pos[3]) {
		case P2P_OUI_TYPE:
			/* Wi-Fi Alliance - P2P IE */
			elems->p2p = pos;
			elems->p2p_len = elen;
			break;
		case WFD_OUI_TYPE:
			/* Wi-Fi Alliance - WFD IE */
			elems->wfd = pos;
			elems->wfd_len = elen;
			break;
		case HS20_INDICATION_OUI_TYPE:
			/* Hotspot 2.0 */
			elems->hs20 = pos;
			elems->hs20_len = elen;
			break;
		case HS20_OSEN_OUI_TYPE:
			/* Hotspot 2.0 OSEN */
			elems->osen = pos;
			elems->osen_len = elen;
			break;
		default:
			wpa_printf(
				MSG_MSGDUMP,
				"Unknown WFA information element ignored (type=%d len=%lu)",
				pos[3], (unsigned long)elen);
			return -1;
		}
		break;

	case OUI_BROADCOM:
		switch (pos[3]) {
		case VENDOR_HT_CAPAB_OUI_TYPE:
			elems->vendor_ht_cap = pos;
			elems->vendor_ht_cap_len = elen;
			break;
		case VENDOR_VHT_TYPE:
			if (elen > 4 && (pos[4] == VENDOR_VHT_SUBTYPE ||
					 pos[4] == VENDOR_VHT_SUBTYPE2)) {
				elems->vendor_vht = pos;
				elems->vendor_vht_len = elen;
			} else
				return -1;
			break;
		default:
			wpa_printf(
				MSG_EXCESSIVE,
				"Unknown Broadcom information element ignored (type=%d len=%lu)",
				pos[3], (unsigned long)elen);
			return -1;
		}
		break;

	case OUI_QCA:
		switch (pos[3]) {
		case QCA_VENDOR_ELEM_P2P_PREF_CHAN_LIST:
			elems->pref_freq_list = pos;
			elems->pref_freq_list_len = elen;
			break;
		default:
			wpa_printf(
				MSG_EXCESSIVE,
				"Unknown QCA information element ignored (type=%d len=%lu)",
				pos[3], (unsigned long)elen);
			return -1;
		}
		break;

	default:
		wpa_printf(
			MSG_EXCESSIVE,
			"unknown vendor specific information element ignored (vendor OUI%02x:%02x:%02x len=%lu)",
			pos[0], pos[1], pos[2], (unsigned long)elen);
		return -1;
	}

	return 0;
}

/**
 * ieee802_11_parse_elems - Parse information elements in management frames
 * @start: Pointer to the start of IEs
 * @len: Length of IE buffer in octets
 * @elems: Data structure for parsed elements
 * @show_errors: Whether to show parsing errors in debug log
 * Returns: Parsing result
 */
enum ParseRes
ieee802_11_parse_elems(const u8 *start, size_t len,
		       struct ieee802_11_elems *elems, int show_errors) {
	size_t left = len;
	const u8 *pos = start;
	int unknown = 0;

	os_memset(elems, 0, sizeof(*elems));

	while (left >= 2) {
		u8 id, elen;

		id = *pos++;
		elen = *pos++;
		left -= 2;

		if (elen > left) {
			if (show_errors) {
				wpa_printf(
					MSG_DEBUG,
					"IEEE 802.11 element parse failed (id=%d elen=%d left=%lu)",
					id, elen, (unsigned long)left);
				wpa_hexdump(MSG_MSGDUMP, "IEs", start, len);
			}
			return ParseFailed;
		}

		switch (id) {
		case WLAN_EID_SSID:
			if (elen > SSID_MAX_LEN) {
				wpa_printf(
					MSG_DEBUG,
					"Ignored too long SSID element (elen=%u)",
					elen);
				break;
			}
			elems->ssid = pos;
			elems->ssid_len = elen;
			break;
		case WLAN_EID_SUPP_RATES:
			elems->supp_rates = pos;
			elems->supp_rates_len = elen;
			break;
		case WLAN_EID_DS_PARAMS:
			if (elen < 1)
				break;
			elems->ds_params = pos;
			break;
		case WLAN_EID_CF_PARAMS:
		case WLAN_EID_TIM:
			break;
		case WLAN_EID_CHALLENGE:
			elems->challenge = pos;
			elems->challenge_len = elen;
			break;
		case WLAN_EID_ERP_INFO:
			if (elen < 1)
				break;
			elems->erp_info = pos;
			break;
		case WLAN_EID_EXT_SUPP_RATES:
			elems->ext_supp_rates = pos;
			elems->ext_supp_rates_len = elen;
			break;
		case WLAN_EID_VENDOR_SPECIFIC:
			if (ieee802_11_parse_vendor_specific(pos, elen, elems,
							     show_errors))
				unknown++;
			break;
		case WLAN_EID_RSN:
			elems->rsn_ie = pos;
			elems->rsn_ie_len = elen;
			break;
		case WLAN_EID_PWR_CAPABILITY:
			break;
		case WLAN_EID_SUPPORTED_CHANNELS:
			elems->supp_channels = pos;
			elems->supp_channels_len = elen;
			break;
		case WLAN_EID_MOBILITY_DOMAIN:
			if (elen < sizeof(struct rsn_mdie))
				break;
			elems->mdie = pos;
			elems->mdie_len = elen;
			break;
		case WLAN_EID_FAST_BSS_TRANSITION:
			if (elen < sizeof(struct rsn_ftie))
				break;
			elems->ftie = pos;
			elems->ftie_len = elen;
			break;
		case WLAN_EID_TIMEOUT_INTERVAL:
			if (elen != 5)
				break;
			elems->timeout_int = pos;
			break;
		case WLAN_EID_HT_CAP:
			if (elen < sizeof(struct ieee80211_ht_capabilities))
				break;
			elems->ht_capabilities = pos;
			break;
		case WLAN_EID_HT_OPERATION:
			if (elen < sizeof(struct ieee80211_ht_operation))
				break;
			elems->ht_operation = pos;
			break;
		case WLAN_EID_MESH_CONFIG:
			elems->mesh_config = pos;
			elems->mesh_config_len = elen;
			break;
		case WLAN_EID_MESH_ID:
			elems->mesh_id = pos;
			elems->mesh_id_len = elen;
			break;
		case WLAN_EID_PEER_MGMT:
			elems->peer_mgmt = pos;
			elems->peer_mgmt_len = elen;
			break;
		case WLAN_EID_VHT_CAP:
			if (elen < sizeof(struct ieee80211_vht_capabilities))
				break;
			elems->vht_capabilities = pos;
			break;
		case WLAN_EID_VHT_OPERATION:
			if (elen < sizeof(struct ieee80211_vht_operation))
				break;
			elems->vht_operation = pos;
			break;
		case WLAN_EID_VHT_OPERATING_MODE_NOTIFICATION:
			if (elen != 1)
				break;
			elems->vht_opmode_notif = pos;
			break;
		case WLAN_EID_LINK_ID:
			if (elen < 18)
				break;
			elems->link_id = pos;
			break;
		case WLAN_EID_INTERWORKING:
			elems->interworking = pos;
			elems->interworking_len = elen;
			break;
		case WLAN_EID_QOS_MAP_SET:
			if (elen < 16)
				break;
			elems->qos_map_set = pos;
			elems->qos_map_set_len = elen;
			break;
		case WLAN_EID_EXT_CAPAB:
			elems->ext_capab = pos;
			elems->ext_capab_len = elen;
			break;
		case WLAN_EID_BSS_MAX_IDLE_PERIOD:
			if (elen < 3)
				break;
			elems->bss_max_idle_period = pos;
			break;
		case WLAN_EID_SSID_LIST:
			elems->ssid_list = pos;
			elems->ssid_list_len = elen;
			break;
		case WLAN_EID_AMPE:
			elems->ampe = pos;
			elems->ampe_len = elen;
			break;
		case WLAN_EID_MIC:
			elems->mic = pos;
			elems->mic_len = elen;
			/* after mic everything is encrypted, so stop. */
			left = elen;
			break;
		case WLAN_EID_MULTI_BAND:
			if (elems->mb_ies.nof_ies >= MAX_NOF_MB_IES_SUPPORTED) {
				wpa_printf(
					MSG_MSGDUMP,
					"IEEE 802.11 element parse ignored MB IE (id=%d elen=%d)",
					id, elen);
				break;
			}

			elems->mb_ies.ies[elems->mb_ies.nof_ies].ie = pos;
			elems->mb_ies.ies[elems->mb_ies.nof_ies].ie_len = elen;
			elems->mb_ies.nof_ies++;
			break;
		default:
			unknown++;
			if (!show_errors)
				break;
			wpa_printf(
				MSG_MSGDUMP,
				"IEEE 802.11 element parse ignored unknown element (id=%d elen=%d)",
				id, elen);
			break;
		}

		left -= elen;
		pos += elen;
	}

	if (left)
		return ParseFailed;

	return unknown ? ParseUnknown : ParseOK;
}

struct wpabuf *
ieee802_11_vendor_ie_concat(const u8 *ies, size_t ies_len, u32 oui_type) {
	struct wpabuf *buf;
	const u8 *end, *pos, *ie;

	pos = ies;
	end = ies + ies_len;
	ie = NULL;

	while (pos + 1 < end) {
		if (pos + 2 + pos[1] > end)
			return NULL;
		if (pos[0] == WLAN_EID_VENDOR_SPECIFIC && pos[1] >= 4 &&
		    WPA_GET_BE32(&pos[2]) == oui_type) {
			ie = pos;
			break;
		}
		pos += 2 + pos[1];
	}

	if (ie == NULL)
		return NULL; /* No specified vendor IE found */

	buf = wpabuf_alloc(ies_len);
	if (buf == NULL)
		return NULL;

	/*
	 * There may be multiple vendor IEs in the message, so need to
	 * concatenate their data fields.
	 */
	while (pos + 1 < end) {
		if (pos + 2 + pos[1] > end)
			break;
		if (pos[0] == WLAN_EID_VENDOR_SPECIFIC && pos[1] >= 4 &&
		    WPA_GET_BE32(&pos[2]) == oui_type)
			wpabuf_put_data(buf, pos + 6, pos[1] - 4);
		pos += 2 + pos[1];
	}

	return buf;
}
void
hostapd_wpa_auth_set_ssid(struct hostapd_data *prHapdData, uint8_t *aucSSID,
			  uint8_t ucSSIDLen) {
	wpa_printf(MSG_DEBUG, "[%s] ", __func__);

	COPY_SSID(prHapdData->conf->ssid.ssid, prHapdData->conf->ssid.ssid_len,
		  aucSSID, ucSSIDLen);
	/*COPY_SSID(prHapdData->wps->ssid, prHapdData->wps->ssid_len, */
	/*	aucSSID, ucSSIDLen);*/
}

void
hostapd_wpa_auth_set_bssid(struct hostapd_data *prHapdData, uint8_t *aucBSSID) {
	wpa_printf(MSG_DEBUG, "[%s] ", __func__);

	COPY_MAC_ADDR(prHapdData->conf->bssid, aucBSSID);
}

void
hostapd_wpa_auth_set_ownmac(struct hostapd_data *prHapdData,
			    uint8_t *aucOwnMac) {
	wpa_printf(MSG_DEBUG, "[%s] ", __func__);

	COPY_MAC_ADDR(prHapdData->own_addr, aucOwnMac);
	COPY_MAC_ADDR(prHapdData->wpa_auth->addr, aucOwnMac);
}

void
hostapd_wpa_auth_conf(struct hostapd_bss_config *conf,
		      struct hostapd_config *iconf,
		      struct wpa_auth_config *wconf) {
	os_memset(wconf, 0, sizeof(*wconf));
	wconf->wpa = conf->wpa;
	wconf->wpa_key_mgmt = conf->wpa_key_mgmt;
	wconf->wpa_pairwise = conf->wpa_pairwise;
	wconf->wpa_group = conf->wpa_group;
	wconf->wpa_group_rekey = conf->wpa_group_rekey;
	wconf->wpa_strict_rekey = conf->wpa_strict_rekey;
	wconf->wpa_gmk_rekey = conf->wpa_gmk_rekey;
	wconf->wpa_ptk_rekey = conf->wpa_ptk_rekey;
	wconf->rsn_pairwise = conf->rsn_pairwise;
	wconf->eapol_version = conf->eapol_version;
#ifdef CONFIG_IEEE80211W
	wconf->ieee80211w = conf->ieee80211w;
	wconf->group_mgmt_cipher = conf->group_mgmt_cipher;
#endif /* CONFIG_IEEE80211W */
}

void
hostapd_wpa_auth_logger(void *ctx, const u8 *addr, int level, const char *txt) {
	/*TODO_CJ: integrate into printf*/
}

static void
hostapd_wpa_auth_disconnect(void *ctx, const u8 *addr, u16 reason) {
	/*struct hostapd_data *hapd = ctx;*/
	/*wpa_printf(MSG_DEBUG, "%s: WPA authenticator requests disconnect: "
	*		   "STA " MACSTR " reason %d",
	*		   __func__, MAC2STR(addr), reason);
	*/
	/*ap_sta_disconnect(hapd, NULL, addr, reason);    */
}

static int
hostapd_wpa_auth_mic_failure_report(void *ctx, const u8 *addr) {
	/*struct hostapd_data *hapd = ctx;
	 *return michael_mic_failure(hapd, addr, 0);
	 */
	return 0;
}

static void
hostapd_wpa_auth_psk_failure_report(void *ctx, const u8 *addr) {
#if 0
	struct hostapd_data *hapd = ctx;

	wpa_msg(hapd->msg_ctx, MSG_DEBUG, AP_STA_POSSIBLE_PSK_MISMATCH MACSTR,
			MAC2STR(addr));
#endif
}

static void
hostapd_wpa_auth_set_eapol(void *ctx, const u8 *addr, int var, int value) {

	struct hostapd_data *hapd = ctx;
	struct sta_info *sta = NULL; /*ap_get_sta(hapd, addr);*/

	if (sta == NULL)
		return;

	switch (var) {
	case WPA_EAPOL_portEnabled:
		ieee802_1x_notify_port_enabled(sta->eapol_sm, value);
		break;
	case WPA_EAPOL_portValid:
		/*ieee802_1x_notify_port_valid(sta->eapol_sm, value);*/
		break;
	case WPA_EAPOL_authorized:
		ieee802_1x_set_sta_authorized(hapd, sta, value);
		break;
	case WPA_EAPOL_portControl_Auto:
		if (sta->eapol_sm)
			sta->eapol_sm->portControl = Auto_auth;
		break;
	case WPA_EAPOL_keyRun:
		if (sta->eapol_sm)
			sta->eapol_sm->keyRun = value ? TRUE : FALSE;
		break;
	case WPA_EAPOL_keyAvailable:
		/*if (sta->eapol_sm)*/
		/*	sta->eapol_sm->eap_if->eapKeyAvailable =*/
		/*		value ? TRUE : FALSE;*/
		break;
	case WPA_EAPOL_keyDone:
		if (sta->eapol_sm)
			sta->eapol_sm->keyDone = value ? TRUE : FALSE;
		break;
	case WPA_EAPOL_inc_EapolFramesTx:
		if (sta->eapol_sm)
			sta->eapol_sm->dot1xAuthEapolFramesTx++;
		break;
	}
}

int
hostapd_wpa_sm_eapol_send(struct hostapd_data *hapd, const uint8_t *dest,
			  /* u16 proto,  const */ uint8_t *buf, size_t len) {
#ifndef CFG_SUPPORT_NAN
	struct MSDU_INFO *prMsduInfo;
	struct STA_RECORD *prStaRec = NULL;
	struct BSS_INFO *prBssInfo = NULL;
	uint8_t i, ucQosLen, fgMatch = FALSE;
	uint8_t *pucBuffer;
	uint8_t aucRfc1042Encap[6] = { 0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00 };

	for (i = 0; i < CFG_STA_REC_NUM; i++) {
		prStaRec = (struct STA_RECORD *)cnmGetStaRecByIndex(i);
		if (prStaRec->fgIsInUse) {
			if (EQUAL_MAC_ADDR(dest, prStaRec->aucMacAddr)) {
				fgMatch = TRUE;
				break;
			}
		}
	}

	if (!fgMatch)
		return -1;

	prBssInfo = g_aprBssInfo[prStaRec->ucBssIndex];

	ucQosLen = prStaRec->fgIsQoS ? 2 : 0;

	/* Attempt to allocate a buffer to compose a Eapol frame. */

	/*4 <1> Allocate a PKT_INFO_T for Frame*/
	/* Allocate a MSDU_INFO_T */
	prMsduInfo = txmAllocPkt(WLAN_MAC_MGMT_HEADER_LEN + ucQosLen + len +
								   LLC_LEN);
	if (prMsduInfo == NULL) {
		DBGLOG(RSN, WARN, ("No PKT_INFO_T for sending EAPoL frame.\n"));
		return WLAN_STATUS_RESOURCES;
	}

	/*4 <2> Compose Probe Request frame header and fixed fields */
	/* in MSDU_INfO_T.*/
	/* Compose Header and Fixed Field */
	secComposeEapolFrameHeader(txmGetPktBuffer(prMsduInfo),
				prStaRec->fgIsQoS, (uint8_t *)dest,
				prBssInfo->aucOwnMacAddr, hapd->conf->bssid);

	/*4 <3> Decide a proper Supported Rate Set and find the TX Rate */
	/* for Probe Req*/

	/*4 <4> Update information of MSDU_INFO_T*/
	txmSetMngPacket(prMsduInfo,
			(struct STA_RECORD *)prStaRec,
			(uint8_t)prStaRec->ucBssIndex,
			(txmGetPktBuffer(prMsduInfo)),
			WLAN_MAC_MGMT_HEADER_LEN + ucQosLen,
			(txmGetPktBuffer(prMsduInfo) +
			WLAN_MAC_MGMT_HEADER_LEN + ucQosLen),
			0,
			secEapolTxStatusNotification,
			MSDU_RATE_MODE_AUTO,
			NULL);

	/*4 <5> Compose the frame body's of the EAPoL  frame.*/
	pucBuffer = (uint8_t *)((uint32_t)prMsduInfo->pucPayload +
			  (uint32_t)prMsduInfo->u2PayloadLength);

	COPY_6_BYTE_FIELD(pucBuffer, aucRfc1042Encap);
	WLAN_SET_FIELD_BE16(pucBuffer + 6,  ETHER_TYPE_802_1X);
	kalMemCopy(pucBuffer + LLC_LEN, buf, len);

	prMsduInfo->u2PayloadLength += (LLC_LEN + len);

	/*txmConfigPktOption(prMsduInfo, MSDU_OPT_PROTECTED_FRAME, TRUE);*/

	/*4 <8> Inform TXM  to send this EAPoL frame.*/
	txmSendMngPackets(prMsduInfo);
#endif
	return 0;
}

static int
hostapd_wpa_auth_send_eapol(void *ctx, const u8 *addr, const u8 *data,
			    size_t data_len, int encrypt) {
	struct hostapd_data *hapd = ctx;

	return hostapd_wpa_sm_eapol_send(hapd, (uint8_t *)addr, (uint8_t *)data,
					 data_len);
}

static int
hostapd_wpa_auth_get_eapol(void *ctx, const u8 *addr, int var) {
	/*struct hostapd_data *hapd = ctx;*/
	struct sta_info *sta = NULL; /*ap_get_sta(hapd, addr);*/

	if (sta == NULL || sta->eapol_sm == NULL)
		return -1;
	switch (var) {
	case WPA_EAPOL_keyRun:
		return sta->eapol_sm->keyRun;
	case WPA_EAPOL_keyAvailable:
	/*return sta->eapol_sm->eap_if->eapKeyAvailable;*/
	default:
		return -1;
	}
}

const u8 *
hostapd_wpa_auth_get_psk(void *ctx, const u8 *addr, const u8 *p2p_dev_addr,
			 const u8 *prev_psk) {
	struct hostapd_data *hapd = ctx;
	const u8 *psk;

	wpa_printf(MSG_INFO, "[%s] Enter\n", __func__);

	psk = hapd->conf->ssid.wpa_psk->psk;

	/*dumpMemory8((PUINT_8)psk, PMK_LEN);*/

	return psk;
}

static int
hostapd_wpa_auth_get_msk(void *ctx, const u8 *addr, u8 *msk, size_t *len) {
	/*struct hostapd_data *hapd = ctx;*/
	const u8 *key = NULL;
	size_t keylen = 0;
	struct sta_info *sta;

	sta = NULL; /* ap_get_sta(hapd, addr) */
	if (sta == NULL) {
		wpa_printf(MSG_DEBUG, "AUTH_GET_MSK: Cannot find STA");
		return -1;
	}

	/*key = ieee802_1x_get_key(sta->eapol_sm, &keylen);*/
	if (key == NULL) {
		wpa_printf(MSG_DEBUG, "AUTH_GET_MSK: Key is null, eapol_sm: %p",
			   sta->eapol_sm);
		return -1;
	}

	if (keylen > *len)
		keylen = *len;
	os_memcpy(msk, key, keylen);
	*len = keylen;

	return 0;
}

/*extern void dumpCmdKey(P_CMD_802_11_KEY prCmdKey);*/
static int
hostapd_wpa_auth_set_key(void *ctx, int vlan_id, enum wpa_alg alg,
			 const u8 *addr, int idx, u8 *key, size_t key_len) {
#ifndef CFG_SUPPORT_NAN
	CMD_802_11_KEY rCmdkey;
	P_CMD_802_11_KEY prCmdkey = &rCmdkey;
	struct STA_RECORD *prStaRec = NULL;
	uint8_t aucBCAddr[] = BC_MAC_ADDR;
	struct BSS_INFO *prBssInfo;
	uint8_t ucEntry = WTBL_RESERVED_ENTRY;
	unsigned char fgIsBC = FALSE;
	/*struct wpa_authenticator* wpa_auth = */
	/*				(struct wpa_authenticator*)ctx;*/
	struct hostapd_data *hapd = (struct hostapd_data *)ctx;

	/*prBssInfo = g_aprBssInfo[wpa_auth->u1BssIdx];*/
	prBssInfo = g_aprBssInfo[hapd->u1BssIdx];

	if (os_memcmp(addr, aucBCAddr, ETH_ALEN) == 0) {
		wpa_printf(MSG_DEBUG, "[%s] broadcast addr", __func__);
		fgIsBC = TRUE;
	}

	/* Compose the common add key structure */
	kalMemZero(&rCmdkey, sizeof(CMD_802_11_KEY));

	rCmdkey.ucAddRemove = key_len ? 1 : 0;
	rCmdkey.ucTxKey = fgIsBC ? 0 : 1;
	rCmdkey.ucKeyType = fgIsBC ? 0 : 1;
	rCmdkey.ucIsAuthenticator = TRUE; /* AP role */

	if (fgIsBC)
		kalMemCopy(&rCmdkey.aucPeerAddr, prBssInfo->aucOwnMacAddr,
			   MAC_ADDR_LEN); /*Own AP*/
	else
		kalMemCopy(&rCmdkey.aucPeerAddr, addr,
			   MAC_ADDR_LEN); /*Remote STA*/

	rCmdkey.ucBssIndex = hapd->u1BssIdx;

	if (alg == WPA_ALG_CCMP)
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_CCMP;
	else if (alg == WPA_ALG_TKIP)
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_TKIP;
	else if (alg == WPA_ALG_IGTK)
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_BIP;
	else
		DBGLOG(RSN, INFO,
		       ("Not support the alg=%d at GTK offload\n", alg));

	rCmdkey.ucKeyId = idx;
	rCmdkey.ucKeyLen = key_len;

	if (!rCmdkey.ucKeyType) {
		if (prBssInfo->ucBMCWlanIndex >= MAX_WTBL_ENTRY_NUM) {
			wpa_printf(
				MSG_DEBUG,
				"[%s] WARN! Unknown ucBMCWlanIndex:%d, u1BssIdx:%d",
				__func__, prBssInfo->ucBMCWlanIndex,
				hapd->u1BssIdx);
			rCmdkey.ucWlanIndex = (MAX_WTBL_ENTRY_NUM - 1);
			/*ASSERT(FALSE);*/
		} else {
			rCmdkey.ucWlanIndex = prBssInfo->ucBMCWlanIndex;
		}
	} else {
		/*Coverity: dead code*/
		/*if(prStaRec!=NULL)
		*{
		*    rCmdkey.ucWlanIndex = prStaRec->ucWTEntry;
		*}
		*/
	}

	if (key != NULL) {
		if ((key_len == 32) &&
		    (rCmdkey.ucAlgorithmId == CIPHER_SUITE_TKIP) &&
		    (rCmdkey.ucIsAuthenticator == FALSE)) {
			/* Do this like driver do : mtk_cfg80211_add_key */
			kalMemCopy(&rCmdkey.aucKeyMaterial, key, 16);
			kalMemCopy(&rCmdkey.aucKeyMaterial[24], key + 16, 8);
			kalMemCopy(&rCmdkey.aucKeyMaterial[16], key + 24, 8);
		} else {
			kalMemCopy(&rCmdkey.aucKeyMaterial, key, key_len);
		}
	}

	/* End of Compose the common add key structure */

	/* Add Key */
	if (prCmdkey->ucAddRemove) {
		if (prCmdkey->ucWlanIndex >= MAX_WTBL_ENTRY_NUM) {
			DBGLOG(RSN, ERROR, ("Wrong wlan index\n"));
			/*ASSERT(FALSE);*/
		} else {
			/*nicPrivacySetKeyEntry(prCmdkey, */
			/*		prCmdkey->ucWlanIndex, prStaRec);*/
			/*phase1: driver cmd trigger it*/
			dumpCmdKey(prCmdkey);
			wpas_evt_cfg80211_add_key(prCmdkey);
		}

	} else { /* Remove Key */
		/*DBGLOG(RSN, INFO, ("[%s] Remove key\n", __func__));*/
		dumpCmdKey(prCmdkey);
		wpas_evt_cfg80211_add_key(prCmdkey);
	}
#endif
	return 0;
}

int
hostapd_wpa_auth_get_seqnum(void *ctx, const u8 *addr, int idx, u8 *seq) {
	/*By code review, */
	/* this always return zero since driver has no implement.*/
	return 0;
}

void
_hostapd_config_defaults_bss(struct hostapd_bss_config *bss) {
#ifndef CFG_SUPPORT_NAN
	bss->auth_algs = WPA_AUTH_ALG_OPEN | WPA_AUTH_ALG_SHARED;

	/* use key0 in individual key and key1 in broadcast key */
	bss->broadcast_key_idx_min = 1;
	bss->broadcast_key_idx_max = 2;
	bss->eap_reauth_period = 3600;

	bss->wpa = 2;
	bss->wpa_group_rekey = 600;
	bss->wpa_gmk_rekey = 86400;
	bss->wpa_key_mgmt = WPA_KEY_MGMT_PSK;
	bss->wpa_pairwise = WPA_CIPHER_CCMP;
	bss->wpa_group = WPA_CIPHER_CCMP;
	bss->rsn_pairwise = 0x10; /*CCMP*/
	bss->eap_server = 1;

	bss->eapol_version = EAPOL_VERSION;

	bss->pwd_group = 19; /* ECC: GF(p=256) */

#ifdef CONFIG_IEEE80211W
	bss->group_mgmt_cipher = WPA_CIPHER_AES_128_CMAC;
#endif /* CONFIG_IEEE80211W */

	bss->wps_state = WPS_STATE_CONFIGURED; /*or WPS_STATE_NOT_CONFIGURED?*/
	bss->force_per_enrollee_psk = 0;
#endif
}

void *
hostapd_wpa_auth_hapd_data_alloc(struct hostapd_data *prHapdData,
				 uint8_t u1BssIdx) {
	/*struct hostapd_data    *prHapdData = NULL;*/
	/*prHapdData = os_zalloc(sizeof(struct hostapd_data));*/
	/*prHapdData = &g_rHapdData;*/

	os_memset(prHapdData, 0, sizeof(struct hostapd_data));

	/*====== Need to allocate by ourselves*/
	prHapdData->u1BssIdx = u1BssIdx;

	/*prHapdData->iface = os_zalloc(sizeof(struct hostapd_iface));*/
	prHapdData->iface = &g_rHapdIface;
	os_memset(prHapdData->iface, 0, sizeof(struct hostapd_iface));

	/*prHapdData->iconf = os_zalloc(sizeof(struct hostapd_config));*/
	prHapdData->iconf = &g_rHapdConf;
	os_memset(prHapdData->iconf, 0, sizeof(struct hostapd_config));

	/*prHapdData->conf = os_zalloc(sizeof(struct hostapd_bss_config));*/
	prHapdData->conf = &g_rHapdBssConf;
	os_memset(prHapdData->conf, 0, sizeof(struct hostapd_bss_config));

	os_memset(&prHapdData->conf->ssid, 0, sizeof(struct hostapd_ssid));
	hostapd_config_defaults_bss(prHapdData->conf);

	/*====== Already allocate in other place*/
	/*
	*hapd->wpa_auth = wpa_init(hapd->own_addr, &_conf, &cb);
	* @hostapd_setup_wpa()
	*hapd->eapol_auth = eapol_auth_init(&conf, &cb);
	* @ieee802_1x_init()
	*hapd->wps = wps;
	* @hostapd_init_wps()
	*/

	return (void *)prHapdData;
}

int
hostapd_wpa_auth_for_each_sta(void *ctx, int (*cb)(struct wpa_state_machine *sm,
						   void *ctx),
			      void *cb_ctx) {

	struct hostapd_data *hapd = ctx;
	struct sta_info *sta;

	for (sta = hapd->sta_list; sta; sta = sta->next) {
		if (sta->wpa_sm && cb(sta->wpa_sm, cb_ctx))
			return 1;
	}
	return 0;
}

int
hostapd_setup_wpa(struct hostapd_data *hapd) {
	struct wpa_auth_config _conf;
	struct wpa_auth_callbacks cb;
	/*const u8 *wpa_ie;*/
	/*size_t wpa_ie_len;*/

	hostapd_wpa_auth_conf(hapd->conf, hapd->iconf, &_conf);
	if (hapd->iface->drv_flags & WPA_DRIVER_FLAGS_EAPOL_TX_STATUS)
		_conf.tx_status = 1;
	if (hapd->iface->drv_flags & WPA_DRIVER_FLAGS_AP_MLME)
		_conf.ap_mlme = 1;
	os_memset(&cb, 0, sizeof(cb));
	cb.ctx = hapd;
	cb.logger = hostapd_wpa_auth_logger;
	cb.disconnect = hostapd_wpa_auth_disconnect;
	cb.mic_failure_report = hostapd_wpa_auth_mic_failure_report;
	cb.psk_failure_report = hostapd_wpa_auth_psk_failure_report;
	cb.set_eapol = hostapd_wpa_auth_set_eapol;
	cb.get_eapol = hostapd_wpa_auth_get_eapol;

	cb.get_psk = hostapd_wpa_auth_get_psk;
	cb.get_msk = hostapd_wpa_auth_get_msk;
	cb.set_key = hostapd_wpa_auth_set_key;
	cb.get_seqnum = hostapd_wpa_auth_get_seqnum;
	cb.send_eapol = hostapd_wpa_auth_send_eapol;
	cb.for_each_sta = hostapd_wpa_auth_for_each_sta;
	/*cb.for_each_auth = hostapd_wpa_auth_for_each_auth;*/
	/*cb.send_ether = hostapd_wpa_auth_send_ether;*/

	hapd->wpa_auth = wpa_init(hapd->own_addr, &_conf, &cb, hapd->u1BssIdx);
	if (hapd->wpa_auth == NULL) {
		wpa_printf(MSG_ERROR, "WPA initialization failed.");
		return -1;
	}

	hapd->wpa_auth->u1BssIdx = hapd->u1BssIdx;

	wpa_printf(MSG_DEBUG,
		   "[%s] hapd->wpa_auth->u1BssIdx:%d, hapd->u1BssIdx:%d",
		   __func__, hapd->wpa_auth->u1BssIdx, hapd->u1BssIdx);

	return 0;
}

void
hostapd_deinit_wpa(struct hostapd_data *hapd) {
	if (hapd->wpa_auth) {
		wpa_deinit(hapd->wpa_auth);
		hapd->wpa_auth = NULL;
	}
	/*ieee802_1x_deinit(hapd);*/
}
