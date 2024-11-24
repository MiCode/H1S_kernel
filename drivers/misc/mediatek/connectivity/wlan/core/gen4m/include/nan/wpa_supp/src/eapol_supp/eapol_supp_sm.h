/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * EAPOL supplicant state machines
 * Copyright (c) 2004-2012, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef EAPOL_SUPP_SM_H
#define EAPOL_SUPP_SM_H

#include "wpa_supp/src/common/defs.h"
#include "wpa_supp/src/utils/wpabuf.h"

enum PortStatus { Unauthorized, Authorized };
enum PortControl { Auto, ForceUnauthorized, ForceAuthorized };

/**
 * struct eapol_config - Per network configuration for EAPOL state machines
 */
struct eapol_config {
	/**
	 * accept_802_1x_keys - Accept IEEE 802.1X (non-WPA) EAPOL-Key frames
	 *
	 * This variable should be set to 1 when using EAPOL state machines
	 * with non-WPA security policy to generate dynamic WEP keys. When
	 * using WPA, this should be set to 0 so that WPA state machine can
	 * process the EAPOL-Key frames.
	 */
	int accept_802_1x_keys;

#define EAPOL_REQUIRE_KEY_UNICAST BIT(0)
#define EAPOL_REQUIRE_KEY_BROADCAST BIT(1)
	/**
	 * required_keys - Which EAPOL-Key packets are required
	 *
	 * This variable determines which EAPOL-Key packets are required before
	 * marking connection authenticated. This is a bit field of
	 * EAPOL_REQUIRE_KEY_UNICAST and EAPOL_REQUIRE_KEY_BROADCAST flags.
	 */
	int required_keys;

	/**
	 * workaround - Whether EAP workarounds are enabled
	 */
	unsigned int workaround;

	/**
	 * eap_disabled - Whether EAP is disabled
	 */
	int eap_disabled;

	/**
	 * external_sim - Use external processing for SIM/USIM operations
	 */
	int external_sim;

#define EAPOL_LOCAL_WPS_IN_USE BIT(0)
#define EAPOL_PEER_IS_WPS20_AP BIT(1)
	/**
	 * wps - Whether this connection is used for WPS
	 */
	int wps;
};

struct eapol_sm;

enum eapol_supp_result {
	EAPOL_SUPP_RESULT_FAILURE,
	EAPOL_SUPP_RESULT_SUCCESS,
	EAPOL_SUPP_RESULT_EXPECTED_FAILURE
};

/**
 * struct eapol_ctx - Global (for all networks) EAPOL state machine context
 */
struct eapol_ctx {
	/**
	 * ctx - Pointer to arbitrary upper level context
	 */
	void *ctx;

	/**
	 * cb - Function to be called when EAPOL negotiation has been completed
	 * @eapol: Pointer to EAPOL state machine data
	 * @result: Whether the authentication was completed successfully
	 * @ctx: Pointer to context data (cb_ctx)
	 *
	 * This optional callback function will be called when the EAPOL
	 * authentication has been completed. This allows the owner of the
	 * EAPOL state machine to process the key and terminate the EAPOL state
	 * machine. Currently, this is used only in RSN pre-authentication.
	 */
	void (*cb)(struct eapol_sm *eapol, enum eapol_supp_result result,
		   void *ctx);

	/**
	 * cb_ctx - Callback context for cb()
	 */
	void *cb_ctx;

	/**
	 * msg_ctx - Callback context for wpa_msg() calls
	 */
	void *msg_ctx;

	/**
	 * eapol_send_ctx - Callback context for eapol_send() calls
	 */
	void *eapol_send_ctx;

	/**
	 * eapol_done_cb - Function to be called at successful completion
	 * @ctx: Callback context (ctx)
	 *
	 * This function is called at the successful completion of EAPOL
	 * authentication. If dynamic WEP keys are used, this is called only
	 * after all the expected keys have been received.
	 */
	void (*eapol_done_cb)(void *ctx);

	/**
	 * eapol_send - Send EAPOL packets
	 * @ctx: Callback context (eapol_send_ctx)
	 * @type: EAPOL type (IEEE802_1X_TYPE_*)
	 * @buf: Pointer to EAPOL payload
	 * @len: Length of the EAPOL payload
	 * Returns: 0 on success, -1 on failure
	 */
	int (*eapol_send)(void *ctx, int type, const u8 *buf, size_t len);

	/**
	 * wps - WPS context data
	 *
	 * This is only used by EAP-WSC and can be left %NULL if not available.
	 */
	struct wps_context *wps;

	/**
	 * eap_param_needed - Notify that EAP parameter is needed
	 * @ctx: Callback context (ctx)
	 * @field: Field indicator (e.g., WPA_CTRL_REQ_EAP_IDENTITY)
	 * @txt: User readable text describing the required parameter
	 */
	void (*eap_param_needed)(void *ctx, enum wpa_ctrl_req_type field,
				 const char *txt);

	/**
	 * port_cb - Set port authorized/unauthorized callback (optional)
	 * @ctx: Callback context (ctx)
	 * @authorized: Whether the supplicant port is now in authorized state
	 */
	void (*port_cb)(void *ctx, int authorized);

	/**
	 * cert_cb - Notification of a peer certificate
	 * @ctx: Callback context (ctx)
	 * @depth: Depth in certificate chain (0 = server)
	 * @subject: Subject of the peer certificate
	 * @altsubject: Select fields from AltSubject of the peer certificate
	 * @num_altsubject: Number of altsubject values
	 * @cert_hash: SHA-256 hash of the certificate
	 * @cert: Peer certificate
	 */
	void (*cert_cb)(void *ctx, int depth, const char *subject,
			const char *altsubject[], int num_altsubject,
			const char *cert_hash, const struct wpabuf *cert);

	/**
	 * cert_in_cb - Include server certificates in callback
	 */
	int cert_in_cb;

	/**
	 * status_cb - Notification of a change in EAP status
	 * @ctx: Callback context (ctx)
	 * @status: Step in the process of EAP authentication
	 * @parameter: Step-specific parameter, e.g., EAP method name
	 */
	void (*status_cb)(void *ctx, const char *status, const char *parameter);
};

struct eap_peer_config;
struct ext_password_data;

#ifdef IEEE8021X_EAPOL
struct eapol_sm *eapol_sm_init(struct eapol_ctx *ctx);
void eapol_sm_deinit(struct eapol_sm *sm);
void eapol_sm_step(struct eapol_sm *sm);
int eapol_sm_get_status(struct eapol_sm *sm, char *buf, size_t buflen,
			int verbose);
int eapol_sm_get_mib(struct eapol_sm *sm, char *buf, size_t buflen);
void eapol_sm_configure(struct eapol_sm *sm, int heldPeriod, int authPeriod,
			int startPeriod, int maxStart);
int eapol_sm_rx_eapol(struct eapol_sm *sm, const u8 *src, const u8 *buf,
		      size_t len);
void eapol_sm_notify_tx_eapol_key(struct eapol_sm *sm);
void eapol_sm_notify_portEnabled(struct eapol_sm *sm, bool enabled);
void eapol_sm_notify_portValid(struct eapol_sm *sm, bool valid);
void eapol_sm_notify_eap_success(struct eapol_sm *sm, bool success);
void eapol_sm_notify_eap_fail(struct eapol_sm *sm, bool fail);
void eapol_sm_notify_config(struct eapol_sm *sm, struct eap_peer_config *config,
			    const struct eapol_config *conf);
int eapol_sm_get_key(struct eapol_sm *sm, u8 *key, size_t len);
const u8 *eapol_sm_get_session_id(struct eapol_sm *sm, size_t *len);
void eapol_sm_notify_logoff(struct eapol_sm *sm, bool logoff);
void eapol_sm_notify_cached(struct eapol_sm *sm);
void eapol_sm_notify_pmkid_attempt(struct eapol_sm *sm);
void eapol_sm_register_scard_ctx(struct eapol_sm *sm, void *ctx);
void eapol_sm_notify_portControl(struct eapol_sm *sm, PortControl portControl);
void eapol_sm_notify_ctrl_attached(struct eapol_sm *sm);
void eapol_sm_notify_ctrl_response(struct eapol_sm *sm);
void eapol_sm_request_reauth(struct eapol_sm *sm);
void eapol_sm_notify_lower_layer_success(struct eapol_sm *sm, int in_eapol_sm);
void eapol_sm_invalidate_cached_session(struct eapol_sm *sm);
const char *eapol_sm_get_method_name(struct eapol_sm *sm);
void eapol_sm_set_ext_pw_ctx(struct eapol_sm *sm,
			     struct ext_password_data *ext);
int eapol_sm_failed(struct eapol_sm *sm);
#else /* IEEE8021X_EAPOL */

static inline struct eapol_sm *
eapol_sm_init(struct eapol_ctx *ctx) {
	/*free(ctx);*/
	return (struct eapol_sm *)1;
}
static inline void
eapol_sm_deinit(struct eapol_sm *sm) {}
static inline void
eapol_sm_step(struct eapol_sm *sm) {}
static inline int
eapol_sm_get_status(struct eapol_sm *sm, char *buf, size_t buflen,
		    int verbose) {
	return 0;
}
static inline int
eapol_sm_get_mib(struct eapol_sm *sm, char *buf, size_t buflen) {
	return 0;
}
static inline void
eapol_sm_configure(struct eapol_sm *sm, int heldPeriod, int authPeriod,
		   int startPeriod, int maxStart) {}
static inline int
eapol_sm_rx_eapol(struct eapol_sm *sm, const u8 *src, const u8 *buf,
		  size_t len) {
	return 0;
}
static inline void
eapol_sm_notify_tx_eapol_key(struct eapol_sm *sm) {}
static inline void
eapol_sm_notify_portEnabled(struct eapol_sm *sm, bool enabled) {}
static inline void
eapol_sm_notify_portValid(struct eapol_sm *sm, bool valid) {}
static inline void
eapol_sm_notify_eap_success(struct eapol_sm *sm, bool success) {}
static inline void
eapol_sm_notify_eap_fail(struct eapol_sm *sm, bool fail) {}
static inline void
eapol_sm_notify_config(struct eapol_sm *sm, struct eap_peer_config *config,
		       struct eapol_config *conf) {}
static inline int
eapol_sm_get_key(struct eapol_sm *sm, u8 *key, size_t len) {
	return -1;
}
static inline const u8 *
eapol_sm_get_session_id(struct eapol_sm *sm, size_t *len) {
	return NULL;
}
static inline void
eapol_sm_notify_logoff(struct eapol_sm *sm, bool logoff) {}
static inline void
eapol_sm_notify_cached(struct eapol_sm *sm) {}
static inline void
eapol_sm_notify_pmkid_attempt(struct eapol_sm *sm) {}
#define eapol_sm_register_scard_ctx(sm, ctx)                                   \
	do {                                                                   \
	} while (0)
static inline void
eapol_sm_notify_portControl(struct eapol_sm *sm, enum PortControl portControl) {
}
static inline void
eapol_sm_notify_ctrl_attached(struct eapol_sm *sm) {}
static inline void
eapol_sm_notify_ctrl_response(struct eapol_sm *sm) {}
static inline void
eapol_sm_request_reauth(struct eapol_sm *sm) {}
static inline void
eapol_sm_notify_lower_layer_success(struct eapol_sm *sm, int in_eapol_sm) {}
static inline void
eapol_sm_invalidate_cached_session(struct eapol_sm *sm) {}
static inline const char *
eapol_sm_get_method_name(struct eapol_sm *sm) {
	return NULL;
}
static inline void
eapol_sm_set_ext_pw_ctx(struct eapol_sm *sm, struct ext_password_data *ext) {}
static inline int
eapol_sm_failed(struct eapol_sm *sm) {
	return 0;
}
static inline void
eapol_sm_erp_flush(struct eapol_sm *sm) {}
#endif /* IEEE8021X_EAPOL */

#endif /* EAPOL_SUPP_SM_H */
