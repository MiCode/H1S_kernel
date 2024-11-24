/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * hostapd / WPA authenticator glue code
 * Copyright (c) 2002-2009, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef WPA_AUTH_GLUE_H
#define WPA_AUTH_GLUE_H

extern void secComposeEapolFrameHeader(IN uint8_t *pucBuffer,
	IN unsigned char fgQos,
	IN uint8_t aucPeerMACAddress[],
	IN uint8_t aucMACAddress[],
	IN uint8_t aucBSSIDAddress[]);

extern uint32_t secEapolTxStatusNotification(IN struct MSDU_INFO *prMsduInfo,
					     IN uint32_t rTxDoneStatus);

#define AP_STA_POSSIBLE_PSK_MISMATCH "AP-STA-POSSIBLE-PSK-MISMATCH "
#define WPS_EVENT_REG_SUCCESS "WPS-REG-SUCCESS "

/** Driver is for a wired Ethernet interface */
#define WPA_DRIVER_FLAGS_WIRED 0x00000010

/** Driver indicates TX status events for EAPOL Data frames */
#define WPA_DRIVER_FLAGS_EAPOL_TX_STATUS 0x00010000

/** Driver expects user space implementation of MLME in AP mode */
#define WPA_DRIVER_FLAGS_AP_MLME 0x01000000

/* STA flags */
#define WLAN_STA_AUTH BIT(0)
#define WLAN_STA_ASSOC BIT(1)
#define WLAN_STA_AUTHORIZED BIT(5)
#define WLAN_STA_PENDING_POLL BIT(6) /* pending activity poll not ACKed */
#define WLAN_STA_SHORT_PREAMBLE BIT(7)
#define WLAN_STA_PREAUTH BIT(8)
#define WLAN_STA_WMM BIT(9)
#define WLAN_STA_MFP BIT(10)
#define WLAN_STA_HT BIT(11)
#define WLAN_STA_WPS BIT(12)
#define WLAN_STA_MAYBE_WPS BIT(13)
#define WLAN_STA_WDS BIT(14)
#define WLAN_STA_ASSOC_REQ_OK BIT(15)
#define WLAN_STA_WPS2 BIT(16)
#define WLAN_STA_GAS BIT(17)
#define WLAN_STA_VHT BIT(18)
#define WLAN_STA_WNM_SLEEP_MODE BIT(19)
#define WLAN_STA_VHT_OPMODE_ENABLED BIT(20)
#define WLAN_STA_VENDOR_VHT BIT(21)
#define WLAN_STA_PENDING_DISASSOC_CB BIT(29)
#define WLAN_STA_PENDING_DEAUTH_CB BIT(30)
#define WLAN_STA_NONERP BIT(31)

#define OUI_QCA 0x001374
enum qca_vendor_element_id {
	QCA_VENDOR_ELEM_P2P_PREF_CHAN_LIST = 0,
};

enum secpolicy {
	SECURITY_PLAINTEXT = 0,
	SECURITY_STATIC_WEP = 1,
	SECURITY_IEEE_802_1X = 2,
	SECURITY_WPA_PSK = 3,
	SECURITY_WPA = 4,
	SECURITY_OSEN = 5
};

struct hostapd_iface {
	u64 drv_flags;
};

struct hostapd_config {
	enum hostapd_hw_mode hw_mode; /* HOSTAPD_MODE_IEEE80211A, .. */
};

#define NUM_WEP_KEYS 4
struct hostapd_wep_keys {
	u8 idx;
	u8 *key[NUM_WEP_KEYS];
	size_t len[NUM_WEP_KEYS];
	int keys_set;
	size_t default_len; /* key length used for dynamic key generation */
};

struct hostapd_wpa_psk {
	struct hostapd_wpa_psk *next;
	int group;
	u8 psk[PMK_LEN];
	u8 addr[ETH_ALEN];
	u8 p2p_dev_addr[ETH_ALEN];
};

struct hostapd_ssid {
	u8 ssid[SSID_MAX_LEN];
	size_t ssid_len;
	unsigned int ssid_set : 1;
	unsigned int utf8_ssid : 1;
	unsigned int wpa_passphrase_set : 1;
	unsigned int wpa_psk_set : 1;

	char vlan[IFNAMSIZ + 1];
	enum secpolicy security_policy;

	struct hostapd_wpa_psk *wpa_psk;
	char *wpa_passphrase;
	char *wpa_psk_file;

	struct hostapd_wep_keys wep;

#define DYNAMIC_VLAN_DISABLED 0
#define DYNAMIC_VLAN_OPTIONAL 1
#define DYNAMIC_VLAN_REQUIRED 2
	int dynamic_vlan;
#define DYNAMIC_VLAN_NAMING_WITHOUT_DEVICE 0
#define DYNAMIC_VLAN_NAMING_WITH_DEVICE 1
#define DYNAMIC_VLAN_NAMING_END 2
	int vlan_naming;
#ifdef CONFIG_FULL_DYNAMIC_VLAN
	char *vlan_tagged_interface;
#endif /* CONFIG_FULL_DYNAMIC_VLAN */
};

struct hostapd_bss_config {
	int wpa;
	int wpa_key_mgmt;
	int wpa_pairwise;
	int wpa_group;
	int wpa_group_rekey;
	int wpa_strict_rekey;
	int wpa_gmk_rekey;
	int wpa_ptk_rekey;
	int rsn_pairwise;
	int eapol_version;
	enum mfp_options ieee80211w;
	int group_mgmt_cipher;
	u8 bssid[ETH_ALEN];

	int auth_algs; /* bitfield of allowed IEEE 802.11 authentication*/
	/* algorithms, WPA_AUTH_ALG_{OPEN,SHARED,LEAP} */

	int ieee802_1x; /* use IEEE 802.1X */
	int osen;

	int wps_state;
	int broadcast_key_idx_min, broadcast_key_idx_max;
	int eap_reauth_period;
	int eap_server;
	/* Use internal EAP server instead of external RADIUS server */
	char *eap_req_id_text;
	/* optional displayable message sent with EAP Request-Identity */
	size_t eap_req_id_text_len;
	int fragment_size;
	u16 pwd_group;
	int pbc_in_m1;
	int eapol_key_index_workaround;

#ifdef CONFIG_WPS
	int wps_independent;
	int ap_setup_locked;
	u8 uuid[16];
	char *wps_pin_requests;
	char *device_name;
	char *manufacturer;
	char *model_name;
	char *model_number;
	char *serial_number;
	u8 device_type[WPS_DEV_TYPE_LEN];
	char *config_methods;
	u8 os_version[4];
	char *ap_pin;
	int skip_cred_build;
	size_t extra_cred_len;
	int wps_cred_processing;
	int force_per_enrollee_psk;
	u8 *ap_settings;
	size_t ap_settings_len;
	struct wpabuf *wps_vendor_ext[MAX_WPS_VENDOR_EXTENSIONS];
#endif /* CONFIG_WPS */

	struct hostapd_ssid ssid;
	u8 wps_rf_bands; /* RF bands for WPS (WPS_RF_*) */
};

struct hostapd_data {
	struct hostapd_iface *iface;
	struct hostapd_config *iconf;
	struct hostapd_bss_config *conf;

	u8 own_addr[ETH_ALEN];

	int num_sta;		   /* number of entries in sta_list */
	struct sta_info *sta_list; /* STA info list head */
#define STA_HASH_SIZE 256
#define STA_HASH(sta) (sta[5])
	struct sta_info *sta_hash[STA_HASH_SIZE];

	struct wpa_authenticator *wpa_auth;
	struct eapol_authenticator *eapol_auth;
	/*struct wps_context *wps;*/
	void *msg_ctx; /* ctx for wpa_msg() calls */

	/*void (*wps_reg_success_cb)(void *ctx, const u8 *mac_addr,*/
	/*			   const u8 *uuid_e);*/
	/*void *wps_reg_success_cb_ctx;*/

	u8 *wpa_ie;
	size_t wpa_ie_len;

	uint8_t u1BssIdx;
};

struct sta_info {
	struct sta_info *next;  /* next entry in sta list */
	struct sta_info *hnext; /* next entry in hash table list */

	u8 addr[6];

	u32 flags; /* Bitfield of WLAN_STA_* */

	struct wpa_state_machine *wpa_sm;
	struct eapol_state_machine *eapol_sm;

	struct wpabuf *wps_ie; /* WPS IE from (Re)Association Request */
	struct wpabuf *p2p_ie; /* P2P IE from (Re)Association Request */

	u32 session_timeout; /* valid only if session_timeout_set == 1 */

	unsigned int remediation : 1;
	unsigned int hs20_deauth_requested : 1;
	unsigned int session_timeout_set : 1;

	/**
	 * req_ies - (Re)Association Request IEs
	 *
	 * If the driver generates WPA/RSN IE, this event data must be
	 * returned for WPA handshake to have needed information. If
	 * wpa_supplicant-generated WPA/RSN IE is used, this
	 * information event is optional.
	 *
	 * This should start with the first IE (fixed fields before IEs
	 * are not included).
	 */
	u8 *req_ies;
	size_t req_ies_len;

	u16 auth_alg;

	struct wps_data *wps_data_from_cmd_ap;

	bool fgIsInUse;

	u8 u1ArrIdx;
};

#define MAX_NOF_MB_IES_SUPPORTED 5
struct mb_ies_info {
	struct {
		const u8 *ie;
		u8 ie_len;
	} ies[MAX_NOF_MB_IES_SUPPORTED];
	u8 nof_ies;
};

/* Parsed Information Elements */
struct ieee802_11_elems {
	const u8 *ssid;
	const u8 *supp_rates;
	const u8 *ds_params;
	const u8 *challenge;
	const u8 *erp_info;
	const u8 *ext_supp_rates;
	const u8 *wpa_ie;
	const u8 *rsn_ie;
	const u8 *wmm; /* WMM Information or Parameter Element */
	const u8 *wmm_tspec;
	const u8 *wps_ie;
	const u8 *supp_channels;
	const u8 *mdie;
	const u8 *ftie;
	const u8 *timeout_int;
	const u8 *ht_capabilities;
	const u8 *ht_operation;
	const u8 *mesh_config;
	const u8 *mesh_id;
	const u8 *peer_mgmt;
	const u8 *vht_capabilities;
	const u8 *vht_operation;
	const u8 *vht_opmode_notif;
	const u8 *vendor_ht_cap;
	const u8 *vendor_vht;
	const u8 *p2p;
	const u8 *wfd;
	const u8 *link_id;
	const u8 *interworking;
	const u8 *qos_map_set;
	const u8 *hs20;
	const u8 *ext_capab;
	const u8 *bss_max_idle_period;
	const u8 *ssid_list;
	const u8 *osen;
	const u8 *ampe;
	const u8 *mic;
	const u8 *pref_freq_list;

	u8 ssid_len;
	u8 supp_rates_len;
	u8 challenge_len;
	u8 ext_supp_rates_len;
	u8 wpa_ie_len;
	u8 rsn_ie_len;
	u8 wmm_len; /* 7 = WMM Information; 24 = WMM Parameter */
	u8 wmm_tspec_len;
	u8 wps_ie_len;
	u8 supp_channels_len;
	u8 mdie_len;
	u8 ftie_len;
	u8 mesh_config_len;
	u8 mesh_id_len;
	u8 peer_mgmt_len;
	u8 vendor_ht_cap_len;
	u8 vendor_vht_len;
	u8 p2p_len;
	u8 wfd_len;
	u8 interworking_len;
	u8 qos_map_set_len;
	u8 hs20_len;
	u8 ext_capab_len;
	u8 ssid_list_len;
	u8 osen_len;
	u8 ampe_len;
	u8 mic_len;
	u8 pref_freq_list_len;
	struct mb_ies_info mb_ies;
};

enum ParseRes { ParseOK = 0, ParseUnknown = 1, ParseFailed = -1 };

enum ParseRes ieee802_11_parse_elems(const u8 *start, size_t len,
				     struct ieee802_11_elems *elems,
				     int show_errors);

struct wpabuf *ieee802_11_vendor_ie_concat(const u8 *ies, size_t ies_len,
					   u32 oui_type);

int hostapd_setup_wpa(struct hostapd_data *hapd);
void hostapd_reconfig_wpa(struct hostapd_data *hapd);
void hostapd_deinit_wpa(struct hostapd_data *hapd);
int hostapd_notif_assoc(struct hostapd_data *hapd, const u8 *addr,
			const u8 *req_ies, size_t req_ies_len, int reassoc);

struct sta_info *ap_get_sta(struct hostapd_data *hapd, const u8 *sta);
void ap_free_sta(struct hostapd_data *hapd, struct sta_info *sta);
void ap_sta_disconnect(struct hostapd_data *hapd, struct sta_info *sta,
		       const u8 *addr, u16 reason);
int ap_sta_is_authorized(struct sta_info *sta);
void ap_sta_set_authorized(struct hostapd_data *hapd, struct sta_info *sta,
			   int authorized);

void _hostapd_config_defaults_bss(struct hostapd_bss_config *bss);

#ifdef CFG_SUPPORT_NAN
const u8 *hostapd_wpa_auth_get_psk(void *ctx, const u8 *addr,
				   const u8 *p2p_dev_addr, const u8 *prev_psk);
int hostapd_wpa_auth_get_seqnum(void *ctx, const u8 *addr, int idx, u8 *seq);
void hostapd_wpa_auth_conf(struct hostapd_bss_config *conf,
			   struct hostapd_config *iconf,
			   struct wpa_auth_config *wconf);
int hostapd_wpa_auth_for_each_sta(void *ctx,
				  int (*cb)(struct wpa_state_machine *sm,
					    void *ctx),
				  void *cb_ctx);
void hostapd_wpa_auth_set_ownmac(struct hostapd_data *prHapdData,
				 uint8_t *aucOwnMac);

void hostapd_wpa_auth_set_bssid(struct hostapd_data *prHapdData,
				uint8_t *aucBSSID);

void *hostapd_wpa_auth_hapd_data_alloc(struct hostapd_data *prHapdData,
				       uint8_t u1BssIdx);

#endif

#endif /* WPA_AUTH_GLUE_H */
