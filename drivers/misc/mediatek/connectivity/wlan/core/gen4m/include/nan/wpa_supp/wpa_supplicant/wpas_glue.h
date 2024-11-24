/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * WPA Supplicant - Glue code to setup EAPOL and RSN modules
 * Copyright (c) 2003-2008, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef WPAS_GLUE_H
#define WPAS_GLUE_H

/*#include "wpa_supp/src/eap_peer/eap_config.h"*/
/*#include "wpa_supp/src/wps/wps.h"*/

extern struct wpa_supplicant *wpa_s;
extern void wpa_supplicant_rx_eapol(void *ctx, const u8 *src_addr,
				    const u8 *buf, size_t len);
/*extern void dumpCmdKey(P_CMD_802_11_KEY prCmdKey);*/
extern struct wpa_sm *(*wpa_sm_init)(struct wpa_sm_ctx *ctx);

enum wpa_ctrl_req_type;

#define WPA_CTRL_REQ "CTRL-REQ-"

/** Driver takes care of RSN 4-way handshake internally; */
/* PMK is configured with*/
/* struct wpa_driver_ops::set_key using alg = WPA_ALG_PMK */
#define WPA_DRIVER_FLAGS_4WAY_HANDSHAKE 0x00000008

/** Driver is for a wired Ethernet interface */
#define WPA_DRIVER_FLAGS_WIRED 0x00000010

/** Driver supports key management offload */
#define WPA_DRIVER_FLAGS_KEY_MGMT_OFFLOAD 0x0000000400000000ULL
struct wpa_config {
	int update_config;
	int cert_in_cb;
	int key_mgmt_offload;
	int okc;
	int eapol_version;

	/**
	 * uuid - Universally Unique IDentifier (UUID; see RFC 4122) for WPS
	 */
	u8 uuid[16];

	/**
	 * device_name - Device Name (WPS)
	 * User-friendly description of device; up to 32 octets encoded in
	 * UTF-8
	 */
	char device_name[32];

	/**
	 * manufacturer - Manufacturer (WPS)
	 * The manufacturer of the device (up to 64 ASCII characters)
	 */
	char manufacturer[64];

	/**
	 * model_name - Model Name (WPS)
	 * Model of the device (up to 32 ASCII characters)
	 */
	char model_name[32];

	/**
	 * model_number - Model Number (WPS)
	 * Additional device description (up to 32 ASCII characters)
	 */
	char model_number[32];

	/**
	 * serial_number - Serial Number (WPS)
	 * Serial number of the device (up to 32 characters)
	 */
	char serial_number[32];

	/**
	 * device_type - Primary Device Type (WPS)
	 */
	/*u8 device_type[WPS_DEV_TYPE_LEN];*/

	/**
	 * config_methods - Config Methods
	 *
	 * This is a space-separated list of supported WPS configuration
	 * methods. For example, "label virtual_display virtual_push_button
	 * keypad".
	 * Available methods: usba ethernet label display ext_nfc_token
	 * int_nfc_token nfc_interface push_button keypad
	 * virtual_display physical_display
	 * virtual_push_button physical_push_button.
	 */
	char config_methods[2];

	/**
	 * os_version - OS Version (WPS)
	 * 4-octet operating system version number
	 */
	u8 os_version[4];

	/**
	 * country - Country code
	 *
	 * This is the ISO/IEC alpha2 country code for which we are operating
	 * in
	 */
	char country[2];

	/**
	 * wps_cred_processing - Credential processing
	 *
	 *   0 = process received credentials internally
	 *   1 = do not process received credentials; just pass them over
	 *	ctrl_iface to external program(s)
	 *   2 = process received credentials internally and pass them over
	 *	ctrl_iface to external program(s)
	 */
	int wps_cred_processing;

#define MAX_SEC_DEVICE_TYPES 5
	/**
	 * sec_device_types - Secondary Device Types (P2P)
	 */
	/*u8 sec_device_type[MAX_SEC_DEVICE_TYPES][WPS_DEV_TYPE_LEN];*/
	int num_sec_device_types;

	struct wpabuf *wps_vendor_ext_m1;
};

struct wpa_ssid {
	int id;
	/*u8 *ssid;*/
	u8 ssid[SSID_MAX_LEN];
	size_t ssid_len;
	/*int peerkey;*/
	int pairwise_cipher;
	unsigned int eap_workaround;
	/*int proactive_key_caching;*/
	int wpa_ptk_rekey;

#ifdef IEEE8021X_EAPOL
#define EAPOL_FLAG_REQUIRE_KEY_UNICAST BIT(0)
#define EAPOL_FLAG_REQUIRE_KEY_BROADCAST BIT(1)
	/**
	 * eapol_flags - Bit field of IEEE 802.1X/EAPOL options (EAPOL_FLAG_*)
	 */
	int eapol_flags;

	/**
	 * eap - EAP peer configuration for this network
	 */
	struct eap_peer_config eap;
#endif /* IEEE8021X_EAPOL */
};

struct wpa_bss {
	struct dl_list list;
	u8 bssid[ETH_ALEN];
	u8 ssid[SSID_MAX_LEN];
	size_t ssid_len;
};

struct wpa_supplicant {
	struct wpa_config *conf;
	char *confname;
	unsigned char own_addr[ETH_ALEN];
	int pairwise_cipher;
	int group_cipher;
	int key_mgmt;
	struct wpa_sm *wpa;
	enum wpa_states wpa_state;
	struct wpa_ssid *current_ssid;
	struct wpa_bss *current_bss;
	u8 bssid[ETH_ALEN];
	struct dl_list bss;
	unsigned char last_eapol_src[ETH_ALEN];
	unsigned int eap_expected_failure : 1;
	/*struct wps_context  *wps;*/
	struct eapol_sm *eapol;
	u64 drv_flags;
	u8 pending_bssid[ETH_ALEN];
	int reassociate;
	unsigned int assoc_freq;
	int countermeasures;

	struct {
		struct hostapd_hw_modes *modes;
		u16 num_modes;
		u16 flags;
	} hw;

	u8 u1BssIdx;
	struct wps_data *wps_data_from_cmd_sta;
};

/*Porting from wpa_supplicant_i.h*/
void wpa_supplicant_set_state(struct wpa_supplicant *wpa_s,
			      enum wpa_states state);
void wpa_supplicant_cancel_auth_timeout(struct wpa_supplicant *wpa_s);
struct wpa_ssid *wpa_supplicant_get_ssid(struct wpa_supplicant *wpa_s);

int wpa_supplicant_init_eapol(struct wpa_supplicant *wpa_s);
int wpa_supplicant_init_wpa(struct wpa_supplicant *wpa_s);

#ifdef CFG_SUPPORT_NAN
void _wpa_supplicant_set_state(void *wpa_s, enum wpa_states state);
enum wpa_states _wpa_supplicant_get_state(void *wpa_s);
void _wpa_supplicant_deauthenticate(void *v_wpa_s, int reason_code);
int wpa_supplicant_get_bssid(void *ctx, u8 *bssid);
u8 *_wpa_alloc_eapol(void *wpa_s, u8 type, const void *data, u16 data_len,
		size_t *msg_len, void **data_pos);
void _wpa_supplicant_cancel_auth_timeout(void *wpa_s);
void wpa_supplicant_set_ownmac(struct wpa_supplicant *prWpa_s,
		uint8_t *aucOwnMac);
void wpa_supplicant_set_bssid(struct wpa_supplicant *prWpa_s,
		uint8_t *aucBSSID);
#endif

#endif /* WPAS_GLUE_H */
