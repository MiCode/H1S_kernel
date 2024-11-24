/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * IEEE 802.1X-2004 Authenticator - EAPOL state machine (internal definitions)
 * Copyright (c) 2002-2009, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef EAPOL_AUTH_SM_I_H
#define EAPOL_AUTH_SM_I_H

#include "wpa_supp/src/common/defs.h"
#include "wpa_supp/src/eapol_auth/eapol_auth_sm.h"
/*#include "wpa_supp/src/eap_server/eap.h"*/

/* IEEE Std 802.1X-2004, Ch. 8.2 */

/*typedef enum { ForceUnauthorized = 1, ForceAuthorized = 3, Auto = 2 }
*	PortTypes;
*typedef enum { Unauthorized = 2, Authorized = 1 } PortState;
*/
enum PortTypes {
	ForceUnauthorized_auth = 1,
	ForceAuthorized_auth = 3,
	Auto_auth = 2
};
enum PortState { Unauthorized_auth = 2, Authorized_auth = 1 };

enum ControlledDirection { Both = 0, In = 1 };
/*typedef unsigned int Counter;*/

/**
 * struct eapol_authenticator - Global EAPOL authenticator data
 */
struct eapol_authenticator {
	struct eapol_auth_config conf;
	struct eapol_auth_cb cb;
};

/**
 * struct eapol_state_machine - Per-Supplicant Authenticator state machines
 */
struct eapol_state_machine {
	/* timers */
	int aWhile;
	int quietWhile;
	int reAuthWhen;

	/* global variables */
	bool authAbort;
	bool authFail;
	enum PortState authPortStatus;
	bool authStart;
	bool authTimeout;
	bool authSuccess;
	bool eapolEap;
	bool initialize;
	bool keyDone;
	bool keyRun;
	bool keyTxEnabled;
	enum PortTypes portControl;
	bool portValid;
	bool reAuthenticate;

	/* Port Timers state machine */
	/* 'Boolean tick' implicitly handled as registered timeout */

	/* Authenticator PAE state machine */
	enum { AUTH_PAE_INITIALIZE,
	       AUTH_PAE_DISCONNECTED,
	       AUTH_PAE_CONNECTING,
	       AUTH_PAE_AUTHENTICATING,
	       AUTH_PAE_AUTHENTICATED,
	       AUTH_PAE_ABORTING,
	       AUTH_PAE_HELD,
	       AUTH_PAE_FORCE_AUTH,
	       AUTH_PAE_FORCE_UNAUTH,
	       AUTH_PAE_RESTART } auth_pae_state;
	/* variables */
	bool eapolLogoff;
	bool eapolStart;
	enum PortTypes portMode;
	unsigned int reAuthCount;
	/* constants */
	unsigned int quietPeriod; /* default 60; 0..65535 */
#define AUTH_PAE_DEFAULT_quietPeriod 60
	unsigned int reAuthMax; /* default 2 */
#define AUTH_PAE_DEFAULT_reAuthMax 2
	/* counters */
	unsigned int authEntersConnecting;
	unsigned int authEapLogoffsWhileConnecting;
	unsigned int authEntersAuthenticating;
	unsigned int authAuthSuccessesWhileAuthenticating;
	unsigned int authAuthTimeoutsWhileAuthenticating;
	unsigned int authAuthFailWhileAuthenticating;
	unsigned int authAuthEapStartsWhileAuthenticating;
	unsigned int authAuthEapLogoffWhileAuthenticating;
	unsigned int authAuthReauthsWhileAuthenticated;
	unsigned int authAuthEapStartsWhileAuthenticated;
	unsigned int authAuthEapLogoffWhileAuthenticated;

	/* Backend Authentication state machine */
	enum { BE_AUTH_REQUEST,
	       BE_AUTH_RESPONSE,
	       BE_AUTH_SUCCESS,
	       BE_AUTH_FAIL,
	       BE_AUTH_TIMEOUT,
	       BE_AUTH_IDLE,
	       BE_AUTH_INITIALIZE,
	       BE_AUTH_IGNORE } be_auth_state;
	/* constants */
	unsigned int serverTimeout; /* default 30; 1..X */
#define BE_AUTH_DEFAULT_serverTimeout 30
	/* counters */
	unsigned int backendResponses;
	unsigned int backendAccessChallenges;
	unsigned int backendOtherRequestsToSupplicant;
	unsigned int backendAuthSuccesses;
	unsigned int backendAuthFails;

	/* Reauthentication Timer state machine */
	enum { REAUTH_TIMER_INITIALIZE,
	       REAUTH_TIMER_REAUTHENTICATE } reauth_timer_state;
	/* constants */
	unsigned int reAuthPeriod; /* default 3600 s */
	bool reAuthEnabled;

	/* Authenticator Key Transmit state machine */
	enum { AUTH_KEY_TX_NO_KEY_TRANSMIT,
	       AUTH_KEY_TX_KEY_TRANSMIT } auth_key_tx_state;

	/* Key Receive state machine */
	enum { KEY_RX_NO_KEY_RECEIVE, KEY_RX_KEY_RECEIVE } key_rx_state;
	/* variables */
	bool rxKey;

	/* Controlled Directions state machine */
	enum { CTRL_DIR_FORCE_BOTH, CTRL_DIR_IN_OR_BOTH } ctrl_dir_state;
	/* variables */
	enum ControlledDirection adminControlledDirections;
	enum ControlledDirection operControlledDirections;
	bool operEdge;

	/* Authenticator Statistics Table */
	unsigned int dot1xAuthEapolFramesRx;
	unsigned int dot1xAuthEapolFramesTx;
	unsigned int dot1xAuthEapolStartFramesRx;
	unsigned int dot1xAuthEapolLogoffFramesRx;
	unsigned int dot1xAuthEapolRespIdFramesRx;
	unsigned int dot1xAuthEapolRespFramesRx;
	unsigned int dot1xAuthEapolReqIdFramesTx;
	unsigned int dot1xAuthEapolReqFramesTx;
	unsigned int dot1xAuthInvalidEapolFramesRx;
	unsigned int dot1xAuthEapLengthErrorFramesRx;
	unsigned int dot1xAuthLastEapolFrameVersion;

	/* Other variables - not defined in IEEE 802.1X */
	u8 addr[ETH_ALEN]; /* Supplicant address */
	int flags;	 /* EAPOL_SM_* */

	/* EAPOL/AAA <-> EAP full authenticator interface */
	struct eap_eapol_interface *eap_if;

	/* TODO: check when the last messages can be released */
	u8 last_eap_id; /* last used EAP Identifier */
	u8 *identity;
	size_t identity_len;
	u8 eap_type_authsrv; /* EAP type of the last EAP packet from */
	/* Authentication server */
	u8 eap_type_supp; /* EAP type of the last EAP packet from Supplicant */

	/* Keys for encrypting and signing EAPOL-Key frames */
	u8 *eapol_key_sign;
	size_t eapol_key_sign_len;
	u8 *eapol_key_crypt;
	size_t eapol_key_crypt_len;

	struct eap_sm *eap;

	bool initializing; /* in process of initializing state machines */
	bool changed;

	struct eapol_authenticator *eapol;

	void *sta; /* station context pointer to use in callbacks */

	int remediation;

	bool fgIsInUse;
	u8 u1ArrIdx;
};

#endif /* EAPOL_AUTH_SM_I_H */
