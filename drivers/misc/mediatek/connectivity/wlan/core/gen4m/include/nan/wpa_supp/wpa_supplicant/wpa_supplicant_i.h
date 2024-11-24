/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * wpa_supplicant - Internal definitions
 * Copyright (c) 2003-2014, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef WPA_SUPPLICANT_I_H
#define WPA_SUPPLICANT_I_H

#include "wpa_supp/FourWayHandShake.h"

extern const char *const wpa_supplicant_version;
extern const char *const wpa_supplicant_license;
#ifndef CONFIG_NO_STDOUT_DEBUG
extern const char *const wpa_supplicant_full_license1;
extern const char *const wpa_supplicant_full_license2;
extern const char *const wpa_supplicant_full_license3;
extern const char *const wpa_supplicant_full_license4;
extern const char *const wpa_supplicant_full_license5;
#endif /* CONFIG_NO_STDOUT_DEBUG */

struct wpa_sm;
struct wpa_supplicant;
struct ibss_rsn;
struct scan_info;
struct wpa_bss;
struct wpa_scan_results;
struct hostapd_hw_modes;
struct wpa_driver_associate_params;

/*
 * Forward declarations of private structures used within the ctrl_iface
 * backends. Other parts of wpa_supplicant do not have access to data stored in
 * these structures.
 */
struct ctrl_iface_priv;
struct ctrl_iface_global_priv;
struct wpas_dbus_priv;

/**
 * struct wpa_interface - Parameters for wpa_supplicant_add_iface()
 */
struct wpa_interface {
	/**
	 * confname - Configuration name (file or profile) name
	 *
	 * This can also be %NULL when a configuration file is not used. In
	 * that case, ctrl_interface must be set to allow the interface to be
	 * configured.
	 */
	const char *confname;

	/**
	 * confanother - Additional configuration name (file or profile) name
	 *
	 * This can also be %NULL when the additional configuration file is not
	 * used.
	 */
	const char *confanother;

	/**
	 * ctrl_interface - Control interface parameter
	 *
	 * If a configuration file is not used, this variable can be used to
	 * set the ctrl_interface parameter that would have otherwise been read
	 * from the configuration file. If both confname and ctrl_interface are
	 * set, ctrl_interface is used to override the value from configuration
	 * file.
	 */
	const char *ctrl_interface;

	/**
	 * driver - Driver interface name, or %NULL to use the default driver
	 */
	const char *driver;

	/**
	 * driver_param - Driver interface parameters
	 *
	 * If a configuration file is not used, this variable can be used to
	 * set the driver_param parameters that would have otherwise been read
	 * from the configuration file. If both confname and driver_param are
	 * set, driver_param is used to override the value from configuration
	 * file.
	 */
	const char *driver_param;

	/**
	 * ifname - Interface name
	 */
	const char *ifname;

	/**
	 * bridge_ifname - Optional bridge interface name
	 *
	 * If the driver interface (ifname) is included in a Linux bridge
	 * device, the bridge interface may need to be used for receiving EAPOL
	 * frames. This can be enabled by setting this variable to enable
	 * receiving of EAPOL frames from an additional interface.
	 */
	const char *bridge_ifname;

	/**
	 * p2p_mgmt - Interface used for P2P management (P2P Device operations)
	 *
	 * Indicates whether wpas_p2p_init() must be called for this interface.
	 * This is used only when the driver supports a dedicated P2P Device
	 * interface that is not a network interface.
	 */
	int p2p_mgmt;
};

/**
 * struct wpa_params - Parameters for wpa_supplicant_init()
 */
struct wpa_params {
	/**
	 * daemonize - Run %wpa_supplicant in the background
	 */
	int daemonize;

	/**
	 * wait_for_monitor - Wait for a monitor program before starting
	 */
	int wait_for_monitor;

	/**
	 * pid_file - Path to a PID (process ID) file
	 *
	 * If this and daemonize are set, process ID of the background process
	 * will be written to the specified file.
	 */
	char *pid_file;

	/**
	 * wpa_debug_level - Debugging verbosity level (e.g., MSG_INFO)
	 */
	int wpa_debug_level;

	/**
	 * wpa_debug_show_keys - Whether keying material is included in debug
	 *
	 * This parameter can be used to allow keying material to be included
	 * in debug messages. This is a security risk and this option should
	 * not be enabled in normal configuration. If needed during
	 * development or while troubleshooting, this option can provide more
	 * details for figuring out what is happening.
	 */
	int wpa_debug_show_keys;

	/**
	 * wpa_debug_timestamp - Whether to include timestamp in debug messages
	 */
	int wpa_debug_timestamp;

	/**
	 * ctrl_interface - Global ctrl_iface path/parameter
	 */
	char *ctrl_interface;

	/**
	 * ctrl_interface_group - Global ctrl_iface group
	 */
	char *ctrl_interface_group;

	/**
	 * dbus_ctrl_interface - Enable the DBus control interface
	 */
	int dbus_ctrl_interface;

	/**
	 * wpa_debug_file_path - Path of debug file or %NULL to use stdout
	 */
	const char *wpa_debug_file_path;

	/**
	 * wpa_debug_syslog - Enable log output through syslog
	 */
	int wpa_debug_syslog;

	/**
	 * wpa_debug_tracing - Enable log output through Linux tracing
	 */
	int wpa_debug_tracing;

	/**
	 * override_driver - Optional driver parameter override
	 *
	 * This parameter can be used to override the driver parameter in
	 * dynamic interface addition to force a specific driver wrapper to be
	 * used instead.
	 */
	char *override_driver;

	/**
	 * override_ctrl_interface - Optional ctrl_interface override
	 *
	 * This parameter can be used to override the ctrl_interface parameter
	 * in dynamic interface addition to force a control interface to be
	 * created.
	 */
	char *override_ctrl_interface;

	/**
	 * entropy_file - Optional entropy file
	 *
	 * This parameter can be used to configure wpa_supplicant to maintain
	 * its internal entropy store over restarts.
	 */
	char *entropy_file;

#ifdef CONFIG_P2P
	/**
	 * conf_p2p_dev - Configuration file used to hold the
	 * P2P Device configuration parameters.
	 *
	 * This can also be %NULL. In such a case, if a P2P Device dedicated
	 * interfaces is created, the main configuration file will be used.
	 */
	char *conf_p2p_dev;
#endif /* CONFIG_P2P */
};

struct p2p_srv_bonjour {
	struct dl_list list;
	struct wpabuf *query;
	struct wpabuf *resp;
};

struct p2p_srv_upnp {
	struct dl_list list;
	u8 version;
	char *service;
};

/**
 * struct wpa_global - Internal, global data for all %wpa_supplicant interfaces
 *
 * This structure is initialized by calling wpa_supplicant_init() when starting
 * %wpa_supplicant.
 */
struct wpa_global {
	struct wpa_supplicant *ifaces;
	struct wpa_params params;
	struct ctrl_iface_global_priv *ctrl_iface;
	struct wpas_dbus_priv *dbus;
	void **drv_priv;
	size_t drv_count;
	struct os_time suspend_time;
	struct p2p_data *p2p;
	struct wpa_supplicant *p2p_init_wpa_s;
	struct wpa_supplicant *p2p_group_formation;
	struct wpa_supplicant *p2p_invite_group;
	u8 p2p_dev_addr[ETH_ALEN];
	struct os_reltime p2p_go_wait_client;
	struct dl_list p2p_srv_bonjour; /* struct p2p_srv_bonjour */
	struct dl_list p2p_srv_upnp;    /* struct p2p_srv_upnp */
	int p2p_disabled;
	int cross_connection;
	struct wpa_freq_range_list p2p_disallow_freq;
	struct wpa_freq_range_list p2p_go_avoid_freq;
	enum wpa_conc_pref {
		WPA_CONC_PREF_NOT_SET,
		WPA_CONC_PREF_STA,
		WPA_CONC_PREF_P2P
	} conc_pref;
	unsigned int p2p_per_sta_psk : 1;
	unsigned int p2p_fail_on_wps_complete : 1;
	unsigned int p2p_24ghz_social_channels : 1;
	unsigned int pending_p2ps_group : 1;
	unsigned int pending_group_iface_for_p2ps : 1;

#ifdef CONFIG_WIFI_DISPLAY
	int wifi_display;
#define MAX_WFD_SUBELEMS 10
	struct wpabuf *wfd_subelem[MAX_WFD_SUBELEMS];
#endif /* CONFIG_WIFI_DISPLAY */

	struct psk_list_entry *add_psk; /* From group formation */
};

/**
 * struct wpa_radio - Internal data for per-radio information
 *
 * This structure is used to share data about configured interfaces
 * (struct wpa_supplicant) that share the same physical radio, e.g., to allow
 * better coordination of offchannel operations.
 */
struct wpa_radio {
	char name[16]; /* from driver_ops get_radio_name() or empty if not*/
	/* available */
	unsigned int external_scan_running : 1;
	struct dl_list ifaces; /* struct wpa_supplicant::radio_list entries */
	struct dl_list work;   /* struct wpa_radio_work::list entries */
};

/**
 * struct wpa_radio_work - Radio work item
 */
struct wpa_radio_work {
	struct dl_list list;
	unsigned int freq; /* known frequency (MHz) or 0 for multiple/unknown */
	const char *type;
	struct wpa_supplicant *wpa_s;
	void (*cb)(struct wpa_radio_work *work, int deinit);
	void *ctx;
	unsigned int started : 1;
	struct os_reltime time;
};

int radio_add_work(struct wpa_supplicant *wpa_s, unsigned int freq,
		   const char *type, int next,
		   void (*cb)(struct wpa_radio_work *work, int deinit),
		   void *ctx);
void radio_work_done(struct wpa_radio_work *work);
void radio_remove_works(struct wpa_supplicant *wpa_s, const char *type,
			int remove_all);
void radio_work_check_next(struct wpa_supplicant *wpa_s);
struct wpa_radio_work *radio_work_pending(struct wpa_supplicant *wpa_s,
					  const char *type);

struct wpa_connect_work {
	unsigned int sme : 1;
	unsigned int bss_removed : 1;
	struct wpa_bss *bss;
	struct wpa_ssid *ssid;
};

int wpas_valid_bss_ssid(struct wpa_supplicant *wpa_s, struct wpa_bss *test_bss,
			struct wpa_ssid *test_ssid);
void wpas_connect_work_free(struct wpa_connect_work *cwork);
void wpas_connect_work_done(struct wpa_supplicant *wpa_s);

struct wpa_external_work {
	unsigned int id;
	char type[100];
	unsigned int timeout;
};

/**
 * offchannel_send_action_result - Result of offchannel send Action frame
 */
enum offchannel_send_action_result {
	OFFCHANNEL_SEND_ACTION_SUCCESS /**< Frame was send and acknowledged */,
	OFFCHANNEL_SEND_ACTION_NO_ACK /**< Frame was sent, but not acknowledged
				       */,
	OFFCHANNEL_SEND_ACTION_FAILED /**< Frame was not sent due to a failure
				       */
};

struct wps_ap_info {
	u8 bssid[ETH_ALEN];
	enum wps_ap_info_type {
		WPS_AP_NOT_SEL_REG,
		WPS_AP_SEL_REG,
		WPS_AP_SEL_REG_OUR
	} type;
	unsigned int tries;
	struct os_reltime last_attempt;
	unsigned int pbc_active;
	u8 uuid[WPS_UUID_LEN];
};

struct wpa_ssid_value {
	u8 ssid[SSID_MAX_LEN];
	size_t ssid_len;
};

#define WPA_FREQ_USED_BY_INFRA_STATION BIT(0)
#define WPA_FREQ_USED_BY_P2P_CLIENT BIT(1)

struct wpa_used_freq_data {
	int freq;
	unsigned int flags;
};

#define RRM_NEIGHBOR_REPORT_TIMEOUT 1 /* 1 second for AP to send a report */

/*
 * struct rrm_data - Data used for managing RRM features
 */
struct rrm_data {
	/* rrm_used - indication regarding the current connection */
	unsigned int rrm_used : 1;

	/*
	 * notify_neighbor_rep - Callback for notifying report requester
	 */
	void (*notify_neighbor_rep)(void *ctx, struct wpabuf *neighbor_rep);

	/*
	 * neighbor_rep_cb_ctx - Callback context
	 * Received in the callback registration, and sent to the callback
	 * function as a parameter.
	 */
	void *neighbor_rep_cb_ctx;

	/* next_neighbor_rep_token - Next request's dialog token */
	u8 next_neighbor_rep_token;
};

enum wpa_supplicant_test_failure {
	WPAS_TEST_FAILURE_NONE,
	WPAS_TEST_FAILURE_SCAN_TRIGGER,
};

/* wpa_supplicant.c */
void
wpa_supplicant_apply_ht_overrides(struct wpa_supplicant *wpa_s,
				  struct wpa_ssid *ssid,
				  struct wpa_driver_associate_params *params);
void
wpa_supplicant_apply_vht_overrides(struct wpa_supplicant *wpa_s,
				   struct wpa_ssid *ssid,
				   struct wpa_driver_associate_params *params);

int wpa_set_wep_keys(struct wpa_supplicant *wpa_s, struct wpa_ssid *ssid);
int wpa_supplicant_set_wpa_none_key(struct wpa_supplicant *wpa_s,
				    struct wpa_ssid *ssid);

int wpa_supplicant_reload_configuration(struct wpa_supplicant *wpa_s);

const char *wpa_supplicant_state_txt(enum wpa_states state);
int wpa_supplicant_update_mac_addr(struct wpa_supplicant *wpa_s);
int wpa_supplicant_driver_init(struct wpa_supplicant *wpa_s);
int wpa_supplicant_set_suites(struct wpa_supplicant *wpa_s, struct wpa_bss *bss,
			      struct wpa_ssid *ssid, u8 *wpa_ie,
			      size_t *wpa_ie_len);
void wpa_supplicant_associate(struct wpa_supplicant *wpa_s, struct wpa_bss *bss,
			      struct wpa_ssid *ssid);
void wpa_supplicant_set_non_wpa_policy(struct wpa_supplicant *wpa_s,
				       struct wpa_ssid *ssid);
void wpa_supplicant_initiate_eapol(struct wpa_supplicant *wpa_s);
void wpa_clear_keys(struct wpa_supplicant *wpa_s, const u8 *addr);
void wpa_supplicant_req_auth_timeout(struct wpa_supplicant *wpa_s, int sec,
				     int usec);
void wpa_supplicant_reinit_autoscan(struct wpa_supplicant *wpa_s);
void wpa_supplicant_set_state(struct wpa_supplicant *wpa_s,
			      enum wpa_states state);
struct wpa_ssid *wpa_supplicant_get_ssid(struct wpa_supplicant *wpa_s);
const char *wpa_supplicant_get_eap_mode(struct wpa_supplicant *wpa_s);
void wpa_supplicant_cancel_auth_timeout(struct wpa_supplicant *wpa_s);
void wpa_supplicant_deauthenticate(struct wpa_supplicant *wpa_s,
				   int reason_code);

void wpa_supplicant_enable_network(struct wpa_supplicant *wpa_s,
				   struct wpa_ssid *ssid);
void wpa_supplicant_disable_network(struct wpa_supplicant *wpa_s,
				    struct wpa_ssid *ssid);
void wpa_supplicant_select_network(struct wpa_supplicant *wpa_s,
				   struct wpa_ssid *ssid);
int wpas_set_pkcs11_engine_and_module_path(struct wpa_supplicant *wpa_s,
					   const char *pkcs11_engine_path,
					   const char *pkcs11_module_path);
int wpa_supplicant_set_ap_scan(struct wpa_supplicant *wpa_s, int ap_scan);
int wpa_supplicant_set_bss_expiration_age(struct wpa_supplicant *wpa_s,
					  unsigned int expire_age);
int wpa_supplicant_set_bss_expiration_count(struct wpa_supplicant *wpa_s,
					    unsigned int expire_count);
int wpa_supplicant_set_scan_interval(struct wpa_supplicant *wpa_s,
				     int scan_interval);
int wpa_supplicant_set_debug_params(struct wpa_global *global, int debug_level,
				    int debug_timestamp, int debug_show_keys);
void free_hw_features(struct wpa_supplicant *wpa_s);

void wpa_show_license(void);

struct wpa_supplicant *wpa_supplicant_add_iface(struct wpa_global *global,
						struct wpa_interface *iface,
						struct wpa_supplicant *parent);
int wpa_supplicant_remove_iface(struct wpa_global *global,
				struct wpa_supplicant *wpa_s, int terminate);
struct wpa_supplicant *wpa_supplicant_get_iface(struct wpa_global *global,
						const char *ifname);
struct wpa_global *wpa_supplicant_init(struct wpa_params *params);
int wpa_supplicant_run(struct wpa_global *global);
void wpa_supplicant_deinit(struct wpa_global *global);

int wpa_supplicant_scard_init(struct wpa_supplicant *wpa_s,
			      struct wpa_ssid *ssid);
void wpa_supplicant_terminate_proc(struct wpa_global *global);
void wpa_supplicant_rx_eapol(void *ctx, const u8 *src_addr, const u8 *buf,
			     size_t len);
void wpa_supplicant_update_config(struct wpa_supplicant *wpa_s);
void wpa_supplicant_clear_status(struct wpa_supplicant *wpa_s);
void wpas_connection_failed(struct wpa_supplicant *wpa_s, const u8 *bssid);
int wpas_driver_bss_selection(struct wpa_supplicant *wpa_s);
int wpas_is_p2p_prioritized(struct wpa_supplicant *wpa_s);
void wpas_auth_failed(struct wpa_supplicant *wpa_s, char *reason);
void wpas_clear_temp_disabled(struct wpa_supplicant *wpa_s,
			      struct wpa_ssid *ssid, int clear_failures);
int disallowed_bssid(struct wpa_supplicant *wpa_s, const u8 *bssid);
int disallowed_ssid(struct wpa_supplicant *wpa_s, const u8 *ssid,
		    size_t ssid_len);
void wpas_request_connection(struct wpa_supplicant *wpa_s);
int wpas_build_ext_capab(struct wpa_supplicant *wpa_s, u8 *buf, size_t buflen);
int wpas_update_random_addr(struct wpa_supplicant *wpa_s, int style);
int wpas_update_random_addr_disassoc(struct wpa_supplicant *wpa_s);
void add_freq(int *freqs, int *num_freqs, int freq);

void wpas_rrm_reset(struct wpa_supplicant *wpa_s);
void wpas_rrm_process_neighbor_rep(struct wpa_supplicant *wpa_s,
				   const u8 *report, size_t report_len);
int wpas_rrm_send_neighbor_rep_request(
	struct wpa_supplicant *wpa_s, const struct wpa_ssid *ssid,
	void (*cb)(void *ctx, struct wpabuf *neighbor_rep), void *cb_ctx);
void wpas_rrm_handle_link_measurement_request(struct wpa_supplicant *wpa_s,
					      const u8 *src, const u8 *frame,
					      size_t len, int rssi);

/**
 * wpa_supplicant_ctrl_iface_ctrl_rsp_handle - Handle a control response
 * @wpa_s: Pointer to wpa_supplicant data
 * @ssid: Pointer to the network block the reply is for
 * @field: field the response is a reply for
 * @value: value (ie, password, etc) for @field
 * Returns: 0 on success, non-zero on error
 *
 * Helper function to handle replies to control interface requests.
 */
int wpa_supplicant_ctrl_iface_ctrl_rsp_handle(struct wpa_supplicant *wpa_s,
					      struct wpa_ssid *ssid,
					      const char *field,
					      const char *value);

void ibss_mesh_setup_freq(struct wpa_supplicant *wpa_s,
			  const struct wpa_ssid *ssid,
			  struct hostapd_freq_params *freq);

/* events.c */
void wpa_supplicant_mark_disassoc(struct wpa_supplicant *wpa_s);
int wpa_supplicant_connect(struct wpa_supplicant *wpa_s,
			   struct wpa_bss *selected, struct wpa_ssid *ssid);
void wpa_supplicant_stop_countermeasures(void *eloop_ctx, void *sock_ctx);
void wpa_supplicant_delayed_mic_error_report(void *eloop_ctx, void *sock_ctx);
void wnm_bss_keep_alive_deinit(struct wpa_supplicant *wpa_s);
int wpa_supplicant_fast_associate(struct wpa_supplicant *wpa_s);
struct wpa_bss *wpa_supplicant_pick_network(struct wpa_supplicant *wpa_s,
					    struct wpa_ssid **selected_ssid);

/* eap_register.c */
int eap_register_methods(void);

/**
 * Utility method to tell if a given network is for persistent group storage
 * @ssid: Network object
 * Returns: 1 if network is a persistent group, 0 otherwise
 */
static inline int
network_is_persistent_group(struct wpa_ssid *ssid) {
	return ssid->disabled == 2 && ssid->p2p_persistent_group;
}

int wpas_network_disabled(struct wpa_supplicant *wpa_s, struct wpa_ssid *ssid);
int wpas_get_ssid_pmf(struct wpa_supplicant *wpa_s, struct wpa_ssid *ssid);

int wpas_init_ext_pw(struct wpa_supplicant *wpa_s);

void dump_freq_data(struct wpa_supplicant *wpa_s, const char *title,
		    struct wpa_used_freq_data *freqs_data, unsigned int len);

int get_shared_radio_freqs_data(struct wpa_supplicant *wpa_s,
				struct wpa_used_freq_data *freqs_data,
				unsigned int len);
int get_shared_radio_freqs(struct wpa_supplicant *wpa_s, int *freq_array,
			   unsigned int len);

void wpas_network_reenabled(void *eloop_ctx, void *timeout_ctx);

#ifdef CONFIG_FST

struct fst_wpa_obj;

void fst_wpa_supplicant_fill_iface_obj(struct wpa_supplicant *wpa_s,
				       struct fst_wpa_obj *iface_obj);

#endif /* CONFIG_FST */

#endif /* WPA_SUPPLICANT_I_H */
