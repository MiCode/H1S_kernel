/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * WPA Supplicant
 * Copyright (c) 2003-2015, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 *
 * This file implements functions for registering and unregistering
 * %wpa_supplicant interfaces. In addition, this file contains number of
 * functions for managing network connections.
 */

#include "wpa_supp/FourWayHandShake.h"
/*#include "wpa_supp/wpaSuppCmdEvt.h"*/

/*====== DLM pre-allocation*/
struct wpa_supplicant g_rWpaSupp;
struct wpa_ssid g_rWpaSsid;
struct wpa_bss g_rWpaBss;
struct wpa_config g_rWpaConf;

struct wpa_supplicant *wpa_s;

/*extern void hostapd_init(void);*/

void
wpa_supplicant_set_bssIndex(uint8_t u1BssIdx) {
	wpa_s->u1BssIdx = u1BssIdx;
}

void
wpa_supplicant_set_state(struct wpa_supplicant *wpa_s, enum wpa_states state) {
	/*struct wpa_supplicant *wpa_s = ctx;*/
	wpa_s->wpa_state = state;
}

/**
 * wpa_supplicant_deauthenticate - Deauthenticate the current connection
 * @wpa_s: Pointer to wpa_supplicant data
 * @reason_code: IEEE 802.11 reason code for the deauthenticate frame
 *
 * This function is used to request %wpa_supplicant to deauthenticate from the
 * current AP.
 */
void
wpa_supplicant_deauthenticate(struct wpa_supplicant *wpa_s, int reason_code) {
#ifndef CFG_SUPPORT_NAN
	wpas_evt_notify_send_deauth(wpa_s->bssid, wpa_s->own_addr, reason_code,
				    FALSE, 0);
#endif
}

void
wpa_supplicant_timeout(void *eloop_ctx, void *timeout_ctx) {
	struct wpa_supplicant *wpa_s = eloop_ctx;
	const u8 *bssid = wpa_s->bssid;

	if (is_zero_ether_addr(bssid))
		bssid = wpa_s->pending_bssid;
	wpa_msg(wpa_s, MSG_DEBUG, "Authentication with " MACSTR " timed out.",
		MAC2STR(bssid));
	wpa_sm_notify_disassoc(wpa_s->wpa);
	wpa_supplicant_deauthenticate(wpa_s, WLAN_REASON_DEAUTH_LEAVING);
	wpa_s->reassociate = 1;

	/*
	 * If we timed out, the AP or the local radio may be busy.
	 * So, wait a second until scanning again.
	 */
	/*wpa_supplicant_req_scan(wpa_s, 1, 0);*/
}

/**
 * wpa_supplicant_cancel_auth_timeout - Cancel authentication timeout
 * @wpa_s: Pointer to wpa_supplicant data
 *
 * This function is used to cancel authentication timeout scheduled with
 * wpa_supplicant_req_auth_timeout() and it is called when authentication has
 * been completed.
 */
void
wpa_supplicant_cancel_auth_timeout(struct wpa_supplicant *wpa_s) {
	wpa_dbg(wpa_s, MSG_DEBUG, "Cancelling authentication timeout");
	eloop_cancel_timeout(wpa_supplicant_timeout, wpa_s, NULL);
}

struct wpa_ssid *
wpa_supplicant_get_ssid(struct wpa_supplicant *wpa_s) {
	/*should be updated in formal path*/
	return wpa_s->current_ssid;
}

void
wpa_supplicant_rx_eapol(void *ctx, const u8 *src_addr, const u8 *buf,
			size_t len) {
#ifndef CFG_SUPPORT_NAN
	if (wpa_s->countermeasures) {
		wpa_printf(MSG_WARNING, "[%s] WPA: Countermeasures - dropped",
			   __func__);
		return;
	}

	/*STA-WPS*/
	if (!wpa_key_mgmt_wpa_psk(wpa_s->key_mgmt) &&
	    eapol_sm_rx_eapol(wpa_s->eapol, src_addr, buf, len) > 0) {
		return;
	}

	if (!(wpa_s->drv_flags & WPA_DRIVER_FLAGS_4WAY_HANDSHAKE)) {
		/*STA-WPA*/
		wpa_sm_rx_eapol(wpa_s->wpa, src_addr, buf, len);
	} else if (wpa_key_mgmt_wpa_ieee8021x(wpa_s->key_mgmt)) {
		/*
		 * Set portValid = TRUE here since we are going to skip 4-way
		 * handshake processing which would normally set portValid. We
		 * need this to allow the EAPOL state machines to be completed
		 * without going through EAPOL-Key handshake.
		 */
		eapol_sm_notify_portValid(wpa_s->eapol, TRUE);
	}
#endif
}

/**
 * wpa_supplicant_init - Initialize %wpa_supplicant
 * @params: Parameters for %wpa_supplicant
 * Returns: Pointer to global %wpa_supplicant data, or %NULL on failure
 *
 * This function is used to initialize %wpa_supplicant. After successful
 * initialization, the returned data pointer can be used to add and remove
 * network interfaces, and eventually, to deinitialize %wpa_supplicant.
 */
void
wpa_supplicant_init(void) {
	/*====== eap method register*/
	/*eap_register_methods();*/
}

/*extern int wpas_wps_init(struct wpa_supplicant *wpa_s);*/
void
wpa_supplicant_init_iface(void) {

	/*wpa_s = os_zalloc(sizeof(struct wpa_supplicant));*/
	wpa_s = &g_rWpaSupp;
	os_memset(wpa_s, 0, sizeof(struct wpa_supplicant));

	wpa_s->current_ssid = &g_rWpaSsid;
	os_memset(wpa_s->current_ssid, 0, sizeof(struct wpa_ssid));

	wpa_s->current_bss = &g_rWpaBss;
	os_memset(wpa_s->current_bss, 0, sizeof(struct wpa_bss));

	wpa_s->conf = &g_rWpaConf;
	os_memset(wpa_s->conf, 0, sizeof(struct wpa_config));
	wpa_s->conf->eapol_version = 2;

	/*====== wpa init*/
	if (wpa_supplicant_init_wpa(wpa_s) < 0)
		return;

	/*====== wps init*/
	/*if (wpas_wps_init(wpa_s))*/
	/*return;*/

	/*====== eapol init, including eap_peer init*/
	if (wpa_supplicant_init_eapol(wpa_s) < 0)
		return;
	wpa_sm_set_eapol(wpa_s->wpa, wpa_s->eapol);

	/*====== update settings*/
	eapol_sm_notify_portEnabled(wpa_s->eapol, FALSE);
	eapol_sm_notify_portValid(wpa_s->eapol, FALSE);
	wpa_supplicant_set_state(wpa_s, WPA_DISCONNECTED);
}

/**
 * wpa_bss_get_vendor_ie_multi - Fetch vendor IE data from a BSS entry
 * @bss: BSS table entry
 * @vendor_type: Vendor type (four octets starting the IE payload)
 * Returns: Pointer to the information element payload or %NULL if not found
 *
 * This function returns concatenated payload of possibly fragmented vendor
 * specific information elements in the BSS entry. The caller is responsible for
 * freeing the returned buffer.
 */
struct wpabuf *
wpa_bss_get_vendor_ie_multi(const struct wpa_bss *bss, u32 vendor_type) {
	/*fw doesn't handle ie*/
	return NULL;
}

/**
 * wpa_supplicant_initiate_eapol - Configure EAPOL state machine (On-the-fly)
 * @wpa_s: Pointer to wpa_supplicant data
 *
 * This function is used to configure EAPOL state machine based on the selected
 * authentication mode.
 */
void
wpa_supplicant_initiate_eapol(struct wpa_supplicant *wpa_s) {
#ifdef IEEE8021X_EAPOL
	struct eapol_config eapol_conf;
	struct wpa_ssid *ssid = wpa_s->current_ssid;

#ifdef CONFIG_IBSS_RSN
	if (ssid->mode == WPAS_MODE_IBSS &&
	    wpa_s->key_mgmt != WPA_KEY_MGMT_NONE &&
	    wpa_s->key_mgmt != WPA_KEY_MGMT_WPA_NONE) {
		/*
		 * RSN IBSS authentication is per-STA and we can disable the
		 * per-BSSID EAPOL authentication.
		 */
		eapol_sm_notify_portControl(wpa_s->eapol, ForceAuthorized);
		eapol_sm_notify_eap_success(wpa_s->eapol, TRUE);
		eapol_sm_notify_eap_fail(wpa_s->eapol, FALSE);
		return;
	}
#endif /* CONFIG_IBSS_RSN */

	eapol_sm_notify_eap_success(wpa_s->eapol, FALSE);
	eapol_sm_notify_eap_fail(wpa_s->eapol, FALSE);

	if (wpa_s->key_mgmt == WPA_KEY_MGMT_NONE ||
	    wpa_s->key_mgmt == WPA_KEY_MGMT_WPA_NONE)
		eapol_sm_notify_portControl(wpa_s->eapol, ForceAuthorized);
	else
		eapol_sm_notify_portControl(wpa_s->eapol, Auto);

	os_memset(&eapol_conf, 0, sizeof(eapol_conf));
	if (wpa_s->key_mgmt == WPA_KEY_MGMT_IEEE8021X_NO_WPA) {
		eapol_conf.accept_802_1x_keys = 1;
		eapol_conf.required_keys = 0;
		if (ssid->eapol_flags & EAPOL_FLAG_REQUIRE_KEY_UNICAST)
			eapol_conf.required_keys |= EAPOL_REQUIRE_KEY_UNICAST;

		if (ssid->eapol_flags & EAPOL_FLAG_REQUIRE_KEY_BROADCAST)
			eapol_conf.required_keys |= EAPOL_REQUIRE_KEY_BROADCAST;

		if (wpa_s->drv_flags & WPA_DRIVER_FLAGS_WIRED)
			eapol_conf.required_keys = 0;
	}
	/*eapol_conf.fast_reauth = wpa_s->conf->fast_reauth;*/
	eapol_conf.workaround = ssid->eap_workaround;
	eapol_conf.eap_disabled =
		!wpa_key_mgmt_wpa_ieee8021x(wpa_s->key_mgmt) &&
		wpa_s->key_mgmt != WPA_KEY_MGMT_IEEE8021X_NO_WPA &&
		wpa_s->key_mgmt != WPA_KEY_MGMT_WPS;
/*eapol_conf.external_sim = wpa_s->conf->external_sim;*/

#ifdef CONFIG_WPS
	if (wpa_s->key_mgmt == WPA_KEY_MGMT_WPS) {
		eapol_conf.wps |= EAPOL_LOCAL_WPS_IN_USE;
		if (wpa_s->current_bss) {
			struct wpabuf *ie;

			ie = wpa_bss_get_vendor_ie_multi(wpa_s->current_bss,
							 WPS_IE_VENDOR_TYPE);
			if (ie) {
				if (wps_is_20(ie))
					eapol_conf.wps |=
						EAPOL_PEER_IS_WPS20_AP;
				wpabuf_free(ie);
			}
		}
	}
#endif /* CONFIG_WPS */

	eapol_sm_notify_config(wpa_s->eapol, &ssid->eap, &eapol_conf);

/*ieee802_1x_alloc_kay_sm(wpa_s, ssid); we don't need*/
#endif /* IEEE8021X_EAPOL */
}

void
wpa_supplicant_deinit_iface(struct wpa_supplicant *wpa_s, int notify,
			    int terminate) {
	/*====== eapol deinit, including eap_peer deinit*/
	wpa_sm_set_eapol(wpa_s->wpa, NULL);
	eapol_sm_deinit(wpa_s->eapol);
	wpa_s->eapol = NULL;

	/*====== wpa deinit*/
	wpa_sm_deinit(wpa_s->wpa);
	wpa_s->wpa = NULL;

	/*wpas_wps_deinit(wpa_s);*/

	os_free(wpa_s);
}

void
wpa_supplicant_start(void) {
	wpa_printf(MSG_INFO, "[%s] Enter\n", __func__);

	/*====== Common*/
	wpa_supplicant_init();

	/*====== STA*/
	wpa_supplicant_init_iface();

	/*====== AP*/
	/*hostapd_init();*/
}
