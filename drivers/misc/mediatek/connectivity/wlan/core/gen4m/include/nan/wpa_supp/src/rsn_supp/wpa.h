/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * wpa_supplicant - WPA definitions
 * Copyright (c) 2003-2015, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef WPA_H
#define WPA_H

struct wpa_sm;
struct eapol_sm;
struct hostapd_freq_params;

struct wpa_sm_ctx {
	void *ctx;     /* pointer to arbitrary upper level context */
	void *msg_ctx; /* upper level context for wpa_msg() calls */

	void (*set_state)(void *ctx, enum wpa_states state);
	enum wpa_states (*get_state)(void *ctx);
	void (*deauthenticate)(void *ctx, int reason_code);
	int (*set_key)(void *ctx, enum wpa_alg alg, const u8 *addr, int key_idx,
		       int set_tx, const u8 *seq, size_t seq_len, const u8 *key,
		       size_t key_len);
	void *(*get_network_ctx)(void *ctx);
	int (*get_bssid)(void *ctx, u8 *bssid);
	int (*ether_send)(void *ctx, const u8 *dest, u16 proto, const u8 *buf,
			  size_t len);
	void (*cancel_auth_timeout)(void *ctx);
	u8 *(*alloc_eapol)(void *ctx, u8 type, const void *data, u16 data_len,
			   size_t *msg_len, void **data_pos);
	int (*mark_authenticated)(void *ctx, const u8 *target_ap);
};

enum wpa_sm_conf_params {
	RSNA_PMK_LIFETIME /* dot11RSNAConfigPMKLifetime */,
	RSNA_PMK_REAUTH_THRESHOLD /* dot11RSNAConfigPMKReauthThreshold */,
	RSNA_SA_TIMEOUT /* dot11RSNAConfigSATimeout */,
	WPA_PARAM_PROTO,
	WPA_PARAM_PAIRWISE,
	WPA_PARAM_GROUP,
	WPA_PARAM_KEY_MGMT,
	WPA_PARAM_MGMT_GROUP,
	WPA_PARAM_RSN_ENABLED,
	WPA_PARAM_MFP
};

struct rsn_supp_config {
	void *network_ctx;
	int peerkey_enabled;
	int allowed_pairwise_cipher; /* bitfield of WPA_CIPHER_* */
	int proactive_key_caching;
	int eap_workaround;
	void *eap_conf_ctx;
	const u8 *ssid;
	size_t ssid_len;
	int wpa_ptk_rekey;
	int p2p;
};

#ifndef CONFIG_NO_WPA

struct wpa_sm *_wpa_sm_init(struct wpa_sm_ctx *ctx);
void wpa_sm_deinit(struct wpa_sm *sm);
void wpa_sm_notify_assoc(struct wpa_sm *sm, const u8 *bssid);
void wpa_sm_notify_disassoc(struct wpa_sm *sm);
void wpa_sm_set_pmk(struct wpa_sm *sm, const u8 *pmk, size_t pmk_len,
		    const u8 *bssid);
void wpa_sm_set_eapol(struct wpa_sm *sm, struct eapol_sm *eapol);

int wpa_sm_set_param(struct wpa_sm *sm, enum wpa_sm_conf_params param,
		     unsigned int value);

int wpa_sm_get_status(struct wpa_sm *sm, char *buf, size_t buflen, int verbose);

void wpa_sm_key_request(struct wpa_sm *sm, int error, int pairwise);

int wpa_sm_rx_eapol_wpa(struct wpa_sm *sm, const u8 *src_addr, const u8 *buf,
			size_t len);

void wpa_sm_update_replay_ctr(struct wpa_sm *sm, const u8 *replay_ctr);

void wpa_sm_set_rx_replay_ctr(struct wpa_sm *sm, const u8 *rx_replay_counter);
void wpa_sm_set_ptk_kck_kek(struct wpa_sm *sm, const u8 *ptk_kck,
			    size_t ptk_kck_len, const u8 *ptk_kek,
			    size_t ptk_kek_len);

static inline int
wpa_sm_stkstart(struct wpa_sm *sm, const u8 *peer) {
	return -1;
}

static inline int
wpa_sm_rx_eapol_peerkey(struct wpa_sm *sm, const u8 *src_addr, const u8 *buf,
			size_t len) {
	return 0;
}
#endif /* CONFIG_NO_WPA */

void wpa_sm_backup_M1(const u8 *src_addr, const u8 *buf, size_t len);
void wpa_sm_free_M1(void);
int wpa_sm_drv_info_check(void);
void wpa_sm_drv_info_clean(void);
int wpa_sm_rmt_rsn_ie_saver_select(const u8 *au1SrcMacAddr);
void wpa_sm_rsn_ie_saver_init(void);

struct _WPA_OFFLOAD_BACKUP_4WAY_M1 {
	unsigned char au1MacAddr[6];
	unsigned char *au1Buf;
	unsigned long u4BufSize;
};

#ifdef CFG_SUPPORT_NAN
int wpa_supplicant_decrypt_key_data(struct wpa_sm *sm,
				    struct wpa_eapol_key *key, u16 ver,
				    u8 *key_data, size_t *key_data_len);

void wpa_supplicant_process_3_of_4(struct wpa_sm *sm,
				   const struct wpa_eapol_key *key, u16 ver,
				   const u8 *key_data, size_t key_data_len);

void wpa_supplicant_process_1_of_4(struct wpa_sm *sm,
				   const unsigned char *src_addr,
				   const struct wpa_eapol_key *key, u16 ver,
				   const u8 *key_data, size_t key_data_len);

void wpa_supplicant_process_1_of_2(struct wpa_sm *sm,
				   const unsigned char *src_addr,
				   const struct wpa_eapol_key *key,
				   const u8 *key_data, size_t key_data_len,
				   u16 ver);

int wpa_supplicant_verify_eapol_key_mic(struct wpa_sm *sm,
					struct wpa_eapol_key_192 *key, u16 ver,
					const u8 *buf, size_t len);

#endif

#endif /* WPA_H */
