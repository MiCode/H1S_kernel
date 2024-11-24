/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * Internal WPA/RSN supplicant state machine definitions
 * Copyright (c) 2004-2015, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef WPA_I_H
#define WPA_I_H

#include "wpa_supp/src/rsn_supp/wpa.h"
#include "wpa_supp/src/utils/list.h"

struct wpa_eapol_key;

extern struct wpa_supplicant *wpa_s;

#define WPA_ASSERT(...)                                                        \
	do {                                                                   \
	} while (0)

/**
 * struct wpa_sm - Internal WPA state machine data
 */
struct wpa_sm {
	u8 pmk[PMK_LEN];
	size_t pmk_len;
	struct wpa_ptk ptk, tptk;
	int ptk_set, tptk_set;
	unsigned int msg_3_of_4_ok : 1;
	u8 snonce[WPA_NONCE_LEN];
	u8 anonce[WPA_NONCE_LEN]; /* ANonce from the last 1/4 msg */
	int renew_snonce;
	u8 rx_replay_counter[WPA_REPLAY_COUNTER_LEN];
	int rx_replay_counter_set;
	u8 request_counter[WPA_REPLAY_COUNTER_LEN];

	struct wpa_gtk gtk;
#ifdef CONFIG_IEEE80211W
	struct wpa_igtk igtk;
#endif /* CONFIG_IEEE80211W */

	struct eapol_sm *eapol; /* EAPOL state machine from upper level code */

	struct wpa_sm_ctx *ctx;

	void *network_ctx;
	int allowed_pairwise_cipher; /* bitfield of WPA_CIPHER_* */
	int eap_workaround;
	void *eap_conf_ctx;
	u8 ssid[32];
	size_t ssid_len;
	int wpa_ptk_rekey;
	int p2p;

	u8 own_addr[ETH_ALEN];
	const char *ifname;
	const char *bridge_ifname;
	u8 bssid[ETH_ALEN];

	unsigned int dot11RSNAConfigPMKLifetime;
	unsigned int dot11RSNAConfigPMKReauthThreshold;
	unsigned int dot11RSNAConfigSATimeout;

	unsigned int dot11RSNA4WayHandshakeFailures;

	/* Selected configuration (based on Beacon/ProbeResp WPA IE) */
	unsigned int proto;
	unsigned int pairwise_cipher;
	unsigned int group_cipher;
	unsigned int key_mgmt;
	unsigned int mgmt_group_cipher;

	int rsn_enabled; /* Whether RSN is enabled in configuration */
	int mfp;	 /* 0 = disabled, 1 = optional, 2 = mandatory */

	u8 *assoc_wpa_ie; /* Own WPA/RSN IE from (Re)AssocReq */
	size_t assoc_wpa_ie_len;
	u8 *ap_wpa_ie, *ap_rsn_ie;
	size_t ap_wpa_ie_len, ap_rsn_ie_len;

#ifdef CFG_SUPPORT_NAN
	u8 fgPtkKeyIdSet;
	u8 u1PtkKeyId;

	u8 au1Psk[PMK_LEN];
	u32 u4PskLen;

	void *pvNdp;
	u32 u4SelCipherType;
	u8 u1CurMsg;
	u8 u1MicCalState;
	bool fgIsTxDone;

	u8 *pu1TmpKdeAttrBuf; /*For Tx*/
	u32 u4TmpKdeAttrLen;

	u8 *pu1GetTxMsgBodyBuf;
	u32 u4GetTxMsgBodyLen;
	u8 *pu1GetTxMsgKdeBuf;

	bool fgIsAllocRxMsgForM1;
	u8 *pu1GetRxMsgBodyBuf;
	u32 u4GetRxMsgBodyLen;
	u8 *pu1GetRxMsgKdeBuf;
	u32 u4GetRxMsgKdeLen;

	u8 *pu1AuthTokenBuf;
	u8 *pu1M3MicMaterialBuf;
	u32 u4M3MicMaterialLen;

#endif
};

static inline void
wpa_sm_set_state(struct wpa_sm *sm, enum wpa_states state) {
	WPA_ASSERT(sm->ctx->set_state);
	sm->ctx->set_state(sm->ctx->ctx, state);
}

static inline enum wpa_states
wpa_sm_get_state(struct wpa_sm *sm) {
	WPA_ASSERT(sm->ctx->get_state);
	return sm->ctx->get_state(sm->ctx->ctx);
}

static inline void
wpa_sm_deauthenticate(struct wpa_sm *sm, int reason_code) {
	WPA_ASSERT(sm->ctx->deauthenticate);
	sm->ctx->deauthenticate(sm->ctx->ctx, reason_code);
}

static inline int
wpa_sm_set_key(struct wpa_sm *sm, enum wpa_alg alg, const u8 *addr, int key_idx,
	       int set_tx, const u8 *seq, size_t seq_len, const u8 *key,
	       size_t key_len) {
	WPA_ASSERT(sm->ctx->set_key);
	return sm->ctx->set_key(sm->ctx->ctx, alg, addr, key_idx, set_tx, seq,
				seq_len, key, key_len);
}

static inline void *
wpa_sm_get_network_ctx(struct wpa_sm *sm) {
	WPA_ASSERT(sm->ctx->get_network_ctx);
	return sm->ctx->get_network_ctx(sm->ctx->ctx);
}

static inline int
wpa_sm_get_bssid(struct wpa_sm *sm, u8 *bssid) {
	WPA_ASSERT(sm->ctx->get_bssid);
	return sm->ctx->get_bssid(sm->ctx->ctx, bssid);
}

static inline int
wpa_sm_ether_send(struct wpa_sm *sm, const u8 *dest, u16 proto, const u8 *buf,
		  size_t len) {
	WPA_ASSERT(sm->ctx->ether_send);
	return sm->ctx->ether_send(sm->ctx->ctx, dest, proto, buf, len);
}

static inline int
wpa_sm_get_beacon_ie(struct wpa_sm *sm) {
	return -1;
}

static inline void
wpa_sm_cancel_auth_timeout(struct wpa_sm *sm) {
	WPA_ASSERT(sm->ctx->cancel_auth_timeout);
	sm->ctx->cancel_auth_timeout(sm->ctx->ctx);
}

static inline u8 *
wpa_sm_alloc_eapol(struct wpa_sm *sm, u8 type, const void *data, u16 data_len,
		   size_t *msg_len, void **data_pos) {
	WPA_ASSERT(sm->ctx->alloc_eapol);
	return sm->ctx->alloc_eapol(sm->ctx->ctx, type, data, data_len, msg_len,
				    data_pos);
}

static inline int
wpa_sm_mark_authenticated(struct wpa_sm *sm, const u8 *target_ap) {
	if (sm->ctx->mark_authenticated)
		return sm->ctx->mark_authenticated(sm->ctx->ctx, target_ap);
	return -1;
}

void wpa_eapol_key_send_wpa(struct wpa_sm *sm, const u8 *kck, size_t kck_len,
			    int ver, const u8 *dest, u16 proto, u8 *msg,
			    size_t msg_len, u8 *key_mic);
int wpa_supplicant_send_2_of_4(struct wpa_sm *sm, const unsigned char *dst,
			       const struct wpa_eapol_key *key, int ver,
			       const u8 *nonce, const u8 *wpa_ie,
			       size_t wpa_ie_len, struct wpa_ptk *ptk);
int wpa_supplicant_send_4_of_4(struct wpa_sm *sm, const unsigned char *dst,
			       const struct wpa_eapol_key *key, u16 ver,
			       u16 key_info, struct wpa_ptk *ptk);

#endif /* WPA_I_H */
