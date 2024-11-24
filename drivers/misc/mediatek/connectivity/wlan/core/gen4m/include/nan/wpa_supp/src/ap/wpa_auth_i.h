/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * hostapd - IEEE 802.11i-2004 / WPA Authenticator: Internal definitions
 * Copyright (c) 2004-2015, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef WPA_AUTH_I_H
#define WPA_AUTH_I_H

#include "wpa_supp/src/ap/wpa_auth.h"
#include "wpa_supp/src/common/wpa_common.h"
#include "wpa_supp/src/utils/common.h"

/* max(dot11RSNAConfigGroupUpdateCount,dot11RSNAConfigPairwiseUpdateCount) */
#define RSNA_MAX_EAPOL_RETRIES 4

struct wpa_group;

struct wpa_stsl_negotiation {
	struct wpa_stsl_negotiation *next;
	u8 initiator[ETH_ALEN];
	u8 peer[ETH_ALEN];
};

struct wpa_state_machine {
	struct wpa_authenticator *wpa_auth;
	struct wpa_group *group;

	u8 addr[ETH_ALEN]; /*self own mac*/
	u8 p2p_dev_addr[ETH_ALEN];

	enum { WPA_PTK_INITIALIZE,
	       WPA_PTK_DISCONNECT,
	       WPA_PTK_DISCONNECTED,
	       WPA_PTK_AUTHENTICATION,
	       WPA_PTK_AUTHENTICATION2,
	       WPA_PTK_INITPMK,
	       WPA_PTK_INITPSK,
	       WPA_PTK_PTKSTART,
	       WPA_PTK_PTKCALCNEGOTIATING,
	       WPA_PTK_PTKCALCNEGOTIATING2,
	       WPA_PTK_PTKINITNEGOTIATING,
	       WPA_PTK_PTKINITDONE } wpa_ptk_state;

	enum { WPA_PTK_GROUP_IDLE = 0,
	       WPA_PTK_GROUP_REKEYNEGOTIATING,
	       WPA_PTK_GROUP_REKEYESTABLISHED,
	       WPA_PTK_GROUP_KEYERROR } wpa_ptk_group_state;

	bool Init;
	bool DeauthenticationRequest;
	bool AuthenticationRequest;
	bool ReAuthenticationRequest;
	bool Disconnect;
	int TimeoutCtr;
	int GTimeoutCtr;
	bool TimeoutEvt;
	bool EAPOLKeyReceived;
	bool EAPOLKeyPairwise;
	bool EAPOLKeyRequest;
	bool MICVerified;
	bool GUpdateStationKeys;
	u8 ANonce[WPA_NONCE_LEN];
	u8 SNonce[WPA_NONCE_LEN];
	u8 alt_SNonce[WPA_NONCE_LEN];
	u8 alt_replay_counter[WPA_REPLAY_COUNTER_LEN];
	u8 PMK[PMK_LEN];
	struct wpa_ptk PTK;
	bool PTK_valid;
	bool pairwise_set;
	int keycount;
	bool Pair;
	struct wpa_key_replay_counter {
		u8 counter[WPA_REPLAY_COUNTER_LEN];
		bool valid;
	} key_replay[RSNA_MAX_EAPOL_RETRIES],
		prev_key_replay[RSNA_MAX_EAPOL_RETRIES];
	bool PInitAKeys; /* WPA only, not in IEEE 802.11i */
	bool PTKRequest; /* not in IEEE 802.11i state machine */
	bool has_GTK;
	bool PtkGroupInit; /* init request for PTK Group state machine */

	u8 *last_rx_eapol_key; /* starting from IEEE 802.1X header */
	size_t last_rx_eapol_key_len;

	unsigned int changed : 1;
	unsigned int in_step_loop : 1;
	unsigned int pending_deinit : 1;
	unsigned int started : 1;
	unsigned int mgmt_frame_prot : 1;
	unsigned int rx_eapol_key_secure : 1;
	unsigned int update_snonce : 1;
	unsigned int alt_snonce_valid : 1;
	unsigned int is_wnmsleep : 1;

	u8 req_replay_counter[WPA_REPLAY_COUNTER_LEN];
	int req_replay_counter_used;

	u8 *wpa_ie;
	size_t wpa_ie_len;

	enum { WPA_VERSION_NO_WPA = 0 /* WPA not used */,
	       WPA_VERSION_WPA = 1 /* WPA / IEEE 802.11i/D3.0 */,
	       WPA_VERSION_WPA2 = 2 /* WPA2 / IEEE 802.11i */
	} wpa;
	int pairwise;     /* Pairwise cipher suite, WPA_CIPHER_* */
	int wpa_key_mgmt; /* the selected WPA_KEY_MGMT_* */

	int pending_1_of_4_timeout;

#ifdef CFG_SUPPORT_NAN
	u8 au1RmtAddr[6];
	u8 u1PtkKeyId;
	u8 fgPtkKeyIdSet;

	u8 au1Psk[PMK_LEN];
	u32 u4PskLen;

	u32 u4SelCipherType;

	void *pvNdp;
	u8 u1CurMsg;
	u8 u1MicCalState;
	bool fgIsTxDone;

	u8 *pu1TmpKdeAttrBuf; /* SEC gen buf */
	u32 u4TmpKdeAttrLen;

	u8 *pu1GetTxMsgBodyBuf; /* NDP gen buf */
	u32 u4GetTxMsgBodyLen;
	u8 *pu1GetTxMsgKdeBuf;

	u8 *pu1GetRxMsgBodyBuf; /* NDP rcv buf */
	u32 u4GetRxMsgBodyLen;
	u8 *pu1GetRxMsgKdeBuf;
	u32 u4GetRxMsgKdeLen;

	u8 *pu1AuthTokenBuf;
	u8 *pu1M3MicMaterialBuf;
	u32 u4M3MicMaterialLen;

#endif
};

#ifdef CFG_SUPPORT_NAN
extern struct wpa_state_machine
	g_arNanWpaAuthSm[NAN_MAX_SUPPORT_NDL_NUM *
			 NAN_MAX_SUPPORT_NDP_NUM]; /*TODO_CJ:def*/
#endif

/* per group key state machine data */
struct wpa_group {
	struct wpa_group *next;
	int vlan_id;

	bool GInit;
	int GKeyDoneStations;
	bool GTKReKey;
	int GTK_len;
	int GN, GM;
	bool GTKAuthenticator;
	u8 Counter[WPA_NONCE_LEN];

	enum { WPA_GROUP_GTK_INIT = 0,
	       WPA_GROUP_SETKEYS,
	       WPA_GROUP_SETKEYSDONE,
	       WPA_GROUP_FATAL_FAILURE } wpa_group_state;

	u8 GMK[WPA_GMK_LEN];
	u8 GTK[2][WPA_GTK_MAX_LEN];
	u8 GNonce[WPA_NONCE_LEN];
	bool changed;
	bool first_sta_seen;
	bool reject_4way_hs_for_entropy;
#ifdef CONFIG_IEEE80211W
	u8 IGTK[2][WPA_IGTK_MAX_LEN];
	int GN_igtk, GM_igtk;
#endif /* CONFIG_IEEE80211W */
	/* Number of references except those in struct wpa_group->next */
	unsigned int references;
};

/* per authenticator data */
struct wpa_authenticator {
	struct wpa_group *group;

	u32 dot11RSNAAuthenticationSuiteSelected;
	u32 dot11RSNAPairwiseCipherSelected;
	u32 dot11RSNAGroupCipherSelected;
	u32 dot11RSNAAuthenticationSuiteRequested; /* FIX: update */
	u32 dot11RSNAPairwiseCipherRequested;      /* FIX: update */
	u32 dot11RSNAGroupCipherRequested;	 /* FIX: update */
	unsigned int dot11RSNA4WayHandshakeFailures;

	struct wpa_auth_config conf;
	struct wpa_auth_callbacks cb;

	u8 *wpa_ie;
	size_t wpa_ie_len;

	u8 addr[ETH_ALEN];

	u8 u1BssIdx;
#ifdef CFG_SUPPORT_NAN
	void *pvNdp;
#endif
};

int wpa_write_rsn_ie(struct wpa_auth_config *conf, u8 *buf, size_t len,
		     const u8 *pmkid);
void wpa_auth_logger(struct wpa_authenticator *wpa_auth, const u8 *addr,
		     int level, const char *txt);
void wpa_auth_vlogger(struct wpa_authenticator *wpa_auth, const u8 *addr,
		      int level, const char *fmt, ...);
void __wpa_send_eapol(struct wpa_authenticator *wpa_auth,
		      struct wpa_state_machine *sm, int key_info,
		      const u8 *key_rsc, const u8 *nonce, const u8 *kde,
		      size_t kde_len, int keyidx, int encr, int force_version);
int wpa_auth_for_each_sta(struct wpa_authenticator *wpa_auth,
			  int (*cb)(struct wpa_state_machine *sm, void *ctx),
			  void *cb_ctx);
int wpa_auth_for_each_auth(struct wpa_authenticator *wpa_auth,
			   int (*cb)(struct wpa_authenticator *a, void *ctx),
			   void *cb_ctx);
#endif /* WPA_AUTH_I_H */
