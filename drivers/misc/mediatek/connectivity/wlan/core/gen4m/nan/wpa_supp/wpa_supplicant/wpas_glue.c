/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * WPA Supplicant - Glue code to setup EAPOL and RSN modules
 * Copyright (c) 2003-2015, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */
#include "wpa_supp/FourWayHandShake.h"
/*#include "wpa_supp/wpa_supplicant/wps_supplicant.h"*/
/*#include "wpa_supp/wpaSuppCmdEvt.h"*/

/*====== DLM pre-allocation*/
struct wpa_sm_ctx g_rWpaSmCtx;
struct eapol_ctx g_rEapolCtx;

unsigned char g_fgEnableEncEapol = FALSE;

/*====== Extern*/
/*extern struct wpa_supplicant *wpa_s;
*extern void wpa_supplicant_rx_eapol(void *ctx, const u8 *src_addr,
					const u8 *buf, size_t len);
*extern void dumpCmdKey(P_CMD_802_11_KEY prCmdKey);
*extern struct wpa_sm *(*wpa_sm_init)(struct wpa_sm_ctx *ctx);
*/

/*====== Tx related*/
static u8 *
wpa_alloc_eapol(const struct wpa_supplicant *wpa_s, u8 type, const void *data,
		u16 data_len, size_t *msg_len, void **data_pos) {
	struct ieee802_1x_hdr *hdr;

	*msg_len = sizeof(*hdr) + data_len;
	hdr = os_malloc(*msg_len);
	if (hdr == NULL)
		return NULL;

	hdr->version = wpa_s->conf->eapol_version;
	hdr->type = type;
	hdr->length = host_to_be16(data_len);

	if (data)
		os_memcpy(hdr + 1, data, data_len);
	else
		os_memset(hdr + 1, 0, data_len);

	if (data_pos)
		*data_pos = hdr + 1;

	return (u8 *)hdr;
}

#ifndef CFG_SUPPORT_NAN
uint32_t
secEapolTxStatusNotification(IN struct MSDU_INFO *prMsduInfo,
			     IN uint32_t rTxDoneStatus) {
	struct BSS_INFO *prBssInfo = NULL;
	struct STA_RECORD *prStaRec = NULL;
	uint16_t u2FrameCtrl = 0;

	/*LITE_ASSERT(prMsduInfo);*/

	prStaRec = prMsduInfo->prStaRec;

	if ((!prStaRec) || (!prStaRec->fgIsInUse)) {
		DBGLOG(RSN, INFO, (
			"->secEapolTxStatusNotification(): STA_RECORD_T was invalid\n"
			));
		return WLAN_STATUS_SUCCESS;
	}

	/*LITE_ASSERT(prMsduInfo->ucBssIndex < BSS_INFO_NUM);*/

	prBssInfo = g_aprBssInfo[prMsduInfo->ucBssIndex];

	DBGLOG(RSN, INFO,
	       ("->secEapolTxStatusNotification(): 0x%08lx\n", rTxDoneStatus));

	return WLAN_STATUS_SUCCESS;
}
#endif

void
secComposeEapolFrameHeader(IN uint8_t *pucBuffer, IN unsigned char fgQos,
			   IN uint8_t aucPeerMACAddress[],
			   IN uint8_t aucMACAddress[],
			   IN uint8_t aucBSSIDAddress[]) {
	struct WLAN_MAC_MGMT_HEADER *prMacHeader;
	uint16_t u2FrameCtrl = MAC_FRAME_TYPE_DATA;

	/*ASSERT(pucBuffer && aucMACAddress);*/
	prMacHeader = (struct WLAN_MAC_MGMT_HEADER *)pucBuffer;

	/*4 <1> Compose the frame header of the ProbeRequest  frame.*/
	if (fgQos)
		u2FrameCtrl = MAC_FRAME_TYPE_QOS_DATA;

	if (g_fgEnableEncEapol && (wpa_s->wpa->ptk.installed)) {
		/*For KRACK test IOT issue*/
		HOST_PRINT((
			"[%s] Send EAPOL in enc, g_fgEnableEncEapol:%d, ptk.installed:%d\n",
			__func__, g_fgEnableEncEapol,
			wpa_s->wpa->ptk.installed));

		u2FrameCtrl |=
			MASK_FC_PROTECTED_FRAME;
		/* HW will also detect this bit for applying encryption */
	}

	if (os_memcmp(aucMACAddress, aucBSSIDAddress,
		      ETH_ALEN))	      /*zero if the same*/
		u2FrameCtrl |= MASK_FC_TO_DS; /*STA*/
	else
		u2FrameCtrl |= MASK_FC_FROM_DS; /*AP*/

	/* Fill the Frame Control field. */
	WLAN_SET_FIELD_16(&prMacHeader->u2FrameCtrl, u2FrameCtrl);

	/* Fill the DA field with broadcast address. */
	COPY_MAC_ADDR(prMacHeader->aucDestAddr, aucPeerMACAddress);

	/* Fill the SA field. */
	COPY_MAC_ADDR(prMacHeader->aucSrcAddr, aucMACAddress);

	/* Fill the BSSID field with the desired BSSID. */
	COPY_MAC_ADDR(prMacHeader->aucBSSID, aucBSSIDAddress);

	/* Clear the SEQ/FRAG_NO field. */
	prMacHeader->u2SeqCtrl = 0;

	/* Fill QoS */
	if (fgQos)
		memset(pucBuffer + WLAN_MAC_HEADER_LEN, 0, 2);

} /* end of secComposeEapolFrameHeader() */

int
wpa_sm_eapol_send(struct wpa_sm *sm, const uint8_t *dest,
		  /* u16 proto,  const */ uint8_t *buf, size_t len) {
#ifndef CFG_SUPPORT_NAN
	struct MSDU_INFO *prMsduInfo;
	struct STA_RECORD *prStaRec = NULL;
	struct BSS_INFO *prBssInfo = NULL;
	uint8_t i, ucQosLen, fgMatch = FALSE;
	uint8_t *pucBuffer;
	uint8_t aucRfc1042Encap[6] = { 0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00 };

	wpa_printf(MSG_INFO, "[%s] Enter\n", __func__);

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
	/*      in MSDU_INfO_T.*/
	/* Compose Header and Fixed Field */
	secComposeEapolFrameHeader(txmGetPktBuffer(prMsduInfo),
				   prStaRec->fgIsQoS, (uint8_t *)dest,
				   prBssInfo->aucOwnMacAddr, sm->bssid);

	/*4 <3> Decide a proper Supported Rate Set and find the TX Rate */
	/*      for Probe Req*/

	/*4 <4> Update information of MSDU_INFO_T*/
	txmSetMngPacket(
		prMsduInfo, (struct STA_RECORD *)prStaRec,
		(uint8_t)prStaRec->ucBssIndex, (txmGetPktBuffer(prMsduInfo)),
		WLAN_MAC_MGMT_HEADER_LEN + ucQosLen,
		(txmGetPktBuffer(prMsduInfo) + WLAN_MAC_MGMT_HEADER_LEN +
		 ucQosLen),
		0, secEapolTxStatusNotification, MSDU_RATE_MODE_AUTO, NULL);

	/*4 <5> Compose the frame body's of the EAPoL  frame.*/
	pucBuffer = (uint8_t *)((uint32_t)prMsduInfo->pucPayload +
			      (uint32_t)prMsduInfo->u2PayloadLength);

	COPY_6_BYTE_FIELD(pucBuffer, aucRfc1042Encap);
	WLAN_SET_FIELD_BE16(pucBuffer + 6, ETHER_TYPE_802_1X);
	kalMemCopy(pucBuffer + LLC_LEN, buf, len);

	prMsduInfo->u2PayloadLength += (LLC_LEN + len);

	/*txmConfigPktOption(prMsduInfo, MSDU_OPT_PROTECTED_FRAME, TRUE);*/

	/*4 <8> Inform TXM  to send this EAPoL frame.*/
	txmSendMngPackets(prMsduInfo);
#endif
	return 0;
}

/**
 * wpa_ether_send - Send Ethernet frame
 * @wpa_s: Pointer to wpa_supplicant data
 * @dest: Destination MAC address
 * @proto: Ethertype in host byte order
 * @buf: Frame payload starting from IEEE 802.1X header
 * @len: Frame payload length
 * Returns: >=0 on success, <0 on failure
 */
static int
wpa_ether_send(struct wpa_supplicant *wpa_s, const u8 *dest, u16 proto,
	       const u8 *buf, size_t len) {
	wpa_sm_eapol_send(wpa_s->wpa, (uint8_t *)dest, (uint8_t *)buf, len);
	return 0;
}

/**
 * wpa_supplicant_eapol_send - Send IEEE 802.1X EAPOL packet to Authenticator
 * @ctx: Pointer to wpa_supplicant data (wpa_s)
 * @type: IEEE 802.1X packet type (IEEE802_1X_TYPE_*)
 * @buf: EAPOL payload (after IEEE 802.1X header)
 * @len: EAPOL payload length
 * Returns: >=0 on success, <0 on failure
 *
 * This function adds Ethernet and IEEE 802.1X header and sends the EAPOL frame
 * to the current Authenticator.
 */
static int
wpa_supplicant_eapol_send(void *ctx, int type, const u8 *buf, size_t len) {
	struct wpa_supplicant *wpa_s = ctx;
	u8 *msg, *dst; /*bssid[ETH_ALEN];*/
	size_t msglen;
	int res;

	/* TODO: could add l2_packet_sendmsg that allows fragments to avoid
	 * extra copy here
	 */

	if (wpa_key_mgmt_wpa_psk(wpa_s->key_mgmt) ||
	    wpa_s->key_mgmt == WPA_KEY_MGMT_NONE) {
		/* Current SSID is not using IEEE 802.1X/EAP, so drop possible
		 * EAPOL frames (mainly, EAPOL-Start) from EAPOL state
		 * machines.
		 */
		wpa_printf(
			MSG_DEBUG,
			"WPA: drop TX EAPOL in non-IEEE 802.1X mode (type=%d len=%lu)",
			type, (unsigned long)len);
		return -1;
	}

	if (is_zero_ether_addr(wpa_s->bssid)) {
		wpa_printf(MSG_DEBUG,
			   "BSSID not set when trying to send an EAPOL frame");
		/*if (wpa_drv_get_bssid(wpa_s, bssid) == 0 &&
		*    !is_zero_ether_addr(bssid)) {
		*	dst = bssid;
		*	wpa_printf(MSG_DEBUG, "Using current BSSID " MACSTR
		*		   " from the driver as the EAPOL destination",
		*		   MAC2STR(dst));
		*} else
		*/
		{
			dst = wpa_s->last_eapol_src;
			/*wpa_printf(MSG_DEBUG, "Using the source address of the
			*		   last received EAPOL frame MACSTR as
			*		   the EAPOL destination",
			*		   MAC2STR(dst));
			*/
		}
	} else {
		/* BSSID was already set (from (Re)Assoc event, so use it as
		 * the EAPOL destination.
		 */
		dst = wpa_s->bssid;
	}

	msg = wpa_alloc_eapol(wpa_s, type, buf, len, &msglen, NULL);
	if (msg == NULL)
		return -1;

	wpa_printf(MSG_DEBUG, "TX EAPOL: dst=" MACSTR, MAC2STR(dst));
	wpa_hexdump(MSG_MSGDUMP, "TX EAPOL", msg, msglen);
	res = wpa_ether_send(wpa_s, dst, ETH_P_EAPOL, msg, msglen);
	os_free(msg);
	return res;
}

/*====== Rx related*/
#ifndef CFG_SUPPORT_NAN
int
secRxProcessEapol(IN struct SW_RFB *prSwRfb, uint8_t *pucSendAddr) {
	struct WLAN_MAC_MGMT_HEADER *prWlanMacHdr;

	wpa_printf(MSG_DEBUG, "[%s] Enter\n", __func__);

	prWlanMacHdr = (struct WLAN_MAC_MGMT_HEADER *)(prSwRfb->pucMacHeader);
	if (EQUAL_MAC_ADDR(wpa_s->wpa->bssid, pucSendAddr)) {
		wpa_supplicant_rx_eapol(
			NULL, pucSendAddr, prSwRfb->pucPayload /* + LLC_LEN */,
			prSwRfb->u2PayloadLength /* - LLC_LEN */);
		return TRUE;
	} /*else*/
	wpa_printf(MSG_WARNING,
		   "[%s] Warn! pucSendAddr is diffrent from aucBSSID\n",
		   __func__);
	wpa_hexdump(MSG_DEBUG, "pucSendAddr", pucSendAddr, MAC_ADDR_LEN);
	wpa_hexdump(MSG_DEBUG, "aucBSSID", prWlanMacHdr->aucBSSID,
		    MAC_ADDR_LEN);

	return FALSE;
}
#endif

/*====== Misc related*/
static void
wpa_supplicant_eapol_cb(struct eapol_sm *eapol, enum eapol_supp_result result,
			void *ctx) {}

static void
wpa_supplicant_notify_eapol_done(void *ctx) {
	struct wpa_supplicant *wpa_s = ctx;

	wpa_msg(wpa_s, MSG_DEBUG, "WPA: EAPOL processing complete");
	if (wpa_key_mgmt_wpa_ieee8021x(wpa_s->key_mgmt)) {
		wpa_supplicant_set_state(wpa_s, WPA_4WAY_HANDSHAKE);
	} else {
		wpa_supplicant_cancel_auth_timeout(wpa_s);
		wpa_supplicant_set_state(wpa_s, WPA_COMPLETED);
		/* wpas_evt_wpa_result(REPORT_EV_SUCCESS, 0, FALSE);*/
	}
}

u8 *
_wpa_alloc_eapol(void *wpa_s, u8 type, const void *data, u16 data_len,
		 size_t *msg_len, void **data_pos) {
	return wpa_alloc_eapol(wpa_s, type, data, data_len, msg_len, data_pos);
}

static int
_wpa_ether_send(void *wpa_s, const u8 *dest, u16 proto, const u8 *buf,
		size_t len) {
	return wpa_ether_send(wpa_s, dest, proto, buf, len);
}

void
_wpa_supplicant_cancel_auth_timeout(void *wpa_s) {
	wpa_supplicant_cancel_auth_timeout(wpa_s);
}

void
_wpa_supplicant_set_state(void *wpa_s, enum wpa_states state) {
	wpa_supplicant_set_state(wpa_s, state);
}

/**
 * wpa_supplicant_get_state - Get the connection state
 * @wpa_s: Pointer to wpa_supplicant data
 * Returns: The current connection state (WPA_*)
 */
static enum wpa_states
wpa_supplicant_get_state(struct wpa_supplicant *wpa_s) {
	return wpa_s->wpa_state;
}

enum wpa_states
_wpa_supplicant_get_state(void *wpa_s) {
	return wpa_supplicant_get_state(wpa_s);
}

void
_wpa_supplicant_deauthenticate(void *v_wpa_s, int reason_code) {
/*struct wpa_supplicant *wpa_s = v_wpa_s;*/
#ifdef CFG_SUPPORT_NAN
/*nanSecNotify4wayTerminate()   //TODO_CJ:ndpIdx compatibility*/
#else
	wpas_evt_notify_send_deauth(wpa_s->bssid, wpa_s->own_addr, reason_code,
				    FALSE, 0);
#endif
}

static void *
wpa_supplicant_get_network_ctx(void *wpa_s) {
	return wpa_supplicant_get_ssid(wpa_s);
}

int
wpa_supplicant_get_bssid(void *ctx, u8 *bssid) {
	struct wpa_supplicant *wpa_s = ctx;

	os_memcpy(bssid, wpa_s->bssid, ETH_ALEN);
	return 0;
}

/*Imitate secWpaRekeySetKey() for STA*/
static int
wpa_supplicant_set_key(void *_wpa_s, enum wpa_alg alg, const u8 *addr,
		       int key_idx, int set_tx, const u8 *seq, size_t seq_len,
		       const u8 *key, size_t key_len) {
#ifndef CFG_SUPPORT_NAN
	CMD_802_11_KEY rCmdkey;
	P_CMD_802_11_KEY prCmdkey = &rCmdkey;
	struct STA_RECORD *prStaRec = NULL;
	uint8_t aucBCAddr[] = BC_MAC_ADDR;
	struct BSS_INFO *prBssInfo;
	uint8_t ucEntry = WTBL_RESERVED_ENTRY;
	unsigned char fgIsBC = FALSE;
	struct wpa_supplicant *prWpa_s = (struct wpa_supplicant *)_wpa_s;

	if (os_memcmp(addr, aucBCAddr, ETH_ALEN) == 0) {
		wpa_printf(MSG_DEBUG, "[%s] broadcast addr", __func__);
		fgIsBC = TRUE;
	}

	prBssInfo = g_aprBssInfo[prWpa_s->u1BssIdx];
	prStaRec = rxmLookupStaRecIndexFromTA((uint8_t *)addr);

	/* Compose the common add key structure */
	kalMemZero(&rCmdkey, sizeof(CMD_802_11_KEY));

	rCmdkey.ucAddRemove = key_len ? 1 : 0;
	rCmdkey.ucTxKey = fgIsBC ? 0 : 1;
	rCmdkey.ucKeyType = fgIsBC ? 0 : 1;
	rCmdkey.ucIsAuthenticator = FALSE; /* STA role */

	if (fgIsBC) {
		kalMemCopy(&rCmdkey.aucPeerAddr, prBssInfo->aucBSSID,
			   MAC_ADDR_LEN); /*Remote AP*/
	} else {
		kalMemCopy(&rCmdkey.aucPeerAddr, addr,
			   MAC_ADDR_LEN); /*Remote AP*/
	}

	rCmdkey.ucBssIndex = prWpa_s->u1BssIdx;

	if (alg == WPA_ALG_CCMP)
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_CCMP;
	else if (alg == WPA_ALG_TKIP)
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_TKIP;
	else if (alg == WPA_ALG_IGTK) {
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_BIP;
		kalMemSet(&rCmdkey.aucPeerAddr, 0, MAC_ADDR_LEN);
	} else {
		DBGLOG(RSN, INFO,
		       ("Not support the alg=%d at GTK offload\n", alg));
	}
	rCmdkey.ucKeyId = key_idx;
	rCmdkey.ucKeyLen = key_len;

	if (!rCmdkey.ucKeyType) {
		if (prBssInfo->ucBMCWlanIndex >= MAX_WTBL_ENTRY_NUM) {
			wpa_printf(
				MSG_DEBUG,
				"[%s] WARN! Unknown ucBMCWlanIndex:%d, u1BssIdx:%d",
				__func__, prBssInfo->ucBMCWlanIndex,
				prWpa_s->u1BssIdx);
			rCmdkey.ucWlanIndex = (MAX_WTBL_ENTRY_NUM - 1);
			/*ASSERT(FALSE);*/
		} else
			rCmdkey.ucWlanIndex = prBssInfo->ucBMCWlanIndex;
	} else {
		if (prStaRec != NULL)
			rCmdkey.ucWlanIndex = prStaRec->ucWTEntry;
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
			/*phase1: driver cmd trigger it*/
			/*nicPrivacySetKeyEntry(prCmdkey, */
			/*		prCmdkey->ucWlanIndex, prStaRec);*/
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
wpa_supplicant_init_eapol(struct wpa_supplicant *wpa_s) {
	struct eapol_ctx *ctx;
	/*ctx = os_zalloc(sizeof(*ctx));*/
	ctx = &g_rEapolCtx;
	os_memset(ctx, 0, sizeof(struct eapol_ctx));

	/*if (ctx == NULL) {
	*	wpa_printf(MSG_ERROR, "Failed to allocate EAPOL context.");
	*	return -1;
	*}
	*/

	ctx->ctx = wpa_s;
	ctx->msg_ctx = wpa_s;
	ctx->eapol_send_ctx = wpa_s;
	ctx->eapol_done_cb = wpa_supplicant_notify_eapol_done;
	ctx->eapol_send = wpa_supplicant_eapol_send;
	/*ctx->wps = wpa_s->wps;*/
	ctx->cb = wpa_supplicant_eapol_cb;
	ctx->cert_in_cb = wpa_s->conf->cert_in_cb;
	ctx->cb_ctx = wpa_s;
	wpa_s->eapol = eapol_sm_init(ctx);
	if (wpa_s->eapol == NULL) {
		os_free(ctx);
		wpa_printf(MSG_ERROR,
			   "Failed to initialize EAPOL state machines.");
		return -1;
	}

	return 0;
}

int
wpa_supplicant_init_wpa(struct wpa_supplicant *wpa_s) {
	struct wpa_sm_ctx *ctx;

	/*ctx = os_zalloc(sizeof(*ctx));*/
	ctx = &g_rWpaSmCtx;
	os_memset(ctx, 0, sizeof(struct wpa_sm_ctx));

	/*if (ctx == NULL) {
	*	wpa_printf(MSG_ERROR, "Failed to allocate WPA context.");
	*	return -1;
	*}
	*/

	ctx->ctx = wpa_s;
	ctx->msg_ctx = wpa_s;
	ctx->set_state = _wpa_supplicant_set_state;
	ctx->get_state = _wpa_supplicant_get_state;
	ctx->deauthenticate = _wpa_supplicant_deauthenticate;
	ctx->set_key = wpa_supplicant_set_key;
	ctx->get_network_ctx = wpa_supplicant_get_network_ctx;
	ctx->get_bssid = wpa_supplicant_get_bssid;
	ctx->ether_send = _wpa_ether_send;
	ctx->alloc_eapol = _wpa_alloc_eapol;
	ctx->cancel_auth_timeout = _wpa_supplicant_cancel_auth_timeout;

	wpa_s->wpa = wpa_sm_init(ctx);
	if (wpa_s->wpa == NULL) {
		wpa_printf(MSG_ERROR, "Failed to initialize WPA state machine");
		os_free(ctx);
		return -1;
	}

	return 0;
}

void
wpa_supplicant_set_ssid(uint8_t *aucSSID, uint8_t ucSSIDLen) {
	COPY_SSID(wpa_s->current_ssid->ssid, wpa_s->current_ssid->ssid_len,
		  aucSSID, ucSSIDLen);
	COPY_SSID(wpa_s->current_bss->ssid, wpa_s->current_bss->ssid_len,
		  aucSSID, ucSSIDLen);
	COPY_SSID(wpa_s->wpa->ssid, wpa_s->wpa->ssid_len, aucSSID, ucSSIDLen);
	/*COPY_SSID(wpa_s->wps->ssid, */
	/*	wpa_s->wps->ssid_len, aucSSID, ucSSIDLen);*/
}

void
wpa_supplicant_set_bssid(struct wpa_supplicant *prWpa_s, uint8_t *aucBSSID) {
	/* COPY_MAC_ADDR(prWpa_s->current_bss->bssid, aucBSSID); */
	COPY_MAC_ADDR(prWpa_s->bssid, aucBSSID);
	COPY_MAC_ADDR(prWpa_s->wpa->bssid, aucBSSID);
}

void
wpa_supplicant_set_ownmac(struct wpa_supplicant *prWpa_s, uint8_t *aucOwnMac) {
	COPY_MAC_ADDR(prWpa_s->own_addr, aucOwnMac);
	COPY_MAC_ADDR(prWpa_s->wpa->own_addr, aucOwnMac);
}
