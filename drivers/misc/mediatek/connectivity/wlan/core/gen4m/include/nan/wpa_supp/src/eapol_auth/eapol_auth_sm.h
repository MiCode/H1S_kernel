/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * IEEE 802.1X-2004 Authenticator - EAPOL state machine
 * Copyright (c) 2002-2015, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef EAPOL_AUTH_SM_H
#define EAPOL_AUTH_SM_H

#define EAPOL_SM_PREAUTH BIT(0)
#define EAPOL_SM_WAIT_START BIT(1)
#define EAPOL_SM_USES_WPA BIT(2)
#define EAPOL_SM_FROM_PMKSA_CACHE BIT(3)

struct eapol_auth_config {
	int eap_reauth_period;
	int wpa;
	int eap_server;
	void *msg_ctx;
	char *eap_req_id_text; /* a copy of this will be allocated */
	size_t eap_req_id_text_len;
	struct wps_context *wps;
	int fragment_size;
	u16 pwd_group;
	int pbc_in_m1;
	const u8 *server_id;
	size_t server_id_len;

	/* Opaque context pointer to owner data for callback functions */
	void *ctx;
};

struct eap_user;

enum eapol_logger_level {
	EAPOL_LOGGER_DEBUG,
	EAPOL_LOGGER_INFO,
	EAPOL_LOGGER_WARNING
};

enum eapol_event { EAPOL_AUTH_SM_CHANGE, EAPOL_AUTH_REAUTHENTICATE };

struct eapol_auth_cb {
	void (*eapol_send)(void *ctx, void *sta_ctx, u8 type, const u8 *data,
			   size_t datalen);
	void (*aaa_send)(void *ctx, void *sta_ctx, const u8 *data,
			 size_t datalen);
	void (*finished)(void *ctx, void *sta_ctx, int success, int preauth,
			 int remediation);
	int (*get_eap_user)(void *ctx, const u8 *identity, size_t identity_len,
			    int phase2, struct eap_user *user);
	int (*sta_entry_alive)(void *ctx, const u8 *addr);
	void (*logger)(void *ctx, const u8 *addr, enum eapol_logger_level level,
		       const char *txt);
	void (*set_port_authorized)(void *ctx, void *sta_ctx, int authorized);
	void (*abort_auth)(void *ctx, void *sta_ctx);
	void (*tx_key)(void *ctx, void *sta_ctx);
	void (*eapol_event)(void *ctx, void *sta_ctx, enum eapol_event type);
};

struct eapol_authenticator *eapol_auth_init(struct eapol_auth_config *conf,
					    struct eapol_auth_cb *cb,
					    int u1BssIdx);
void eapol_auth_deinit(struct eapol_authenticator *eapol);
struct eapol_state_machine *
eapol_auth_alloc(struct eapol_authenticator *eapol, const u8 *addr, int flags,
		 const struct wpabuf *assoc_wps_ie,
		 const struct wpabuf *assoc_p2p_ie, void *sta_ctx,
		 const char *identity, const char *radius_cui);
void eapol_auth_free(struct eapol_state_machine *sm);
void eapol_auth_step(struct eapol_state_machine *sm);
int eapol_auth_dump_state(struct eapol_state_machine *sm, char *buf,
			  size_t buflen);
int eapol_auth_eap_pending_cb(struct eapol_state_machine *sm, void *ctx);
void eapol_auth_reauthenticate(struct eapol_state_machine *sm);
int eapol_auth_set_conf(struct eapol_state_machine *sm, const char *param,
			const char *value);

#endif /* EAPOL_AUTH_SM_H */
