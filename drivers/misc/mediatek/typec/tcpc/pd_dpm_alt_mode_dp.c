// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
#include <linux/delay.h>

#include "inc/tcpci.h"
#include "inc/pd_policy_engine.h"
#include "inc/pd_dpm_core.h"
#include "pd_dpm_prv.h"

/* Display Port DFP_U / UFP_U */


/* DP_Role : DFP_D & UFP_D Both or DFP_D only */

#define DP_CHECK_DP_CONNECTED_MATCH(a, b)	\
	((a|b) == DPSTS_BOTH_CONNECTED)

#define DP_DFP_U_CHECK_ROLE_CAP_MATCH(a, b)	\
	((MODE_DP_PORT_CAP(a)|MODE_DP_PORT_CAP(b)) == MODE_DP_BOTH)

#define DP_SELECT_CONNECTED(b)		((b == DPSTS_DFP_D_CONNECTED) ? \
		DPSTS_UFP_D_CONNECTED : DPSTS_DFP_D_CONNECTED)

/*
 * If we support ufp_d & dfp_d both, we should choose another role.
 * If we don't support both, check dp_connected valid or not
 */
static inline bool dp_update_dp_connected_one(struct pd_port *pd_port,
			uint32_t dp_connected, uint32_t dp_local_connected)
{
	bool valid_connected;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	if (dp_local_connected != DPSTS_BOTH_CONNECTED) {
		valid_connected = DP_CHECK_DP_CONNECTED_MATCH(
			dp_connected, dp_local_connected);
	} else {
		valid_connected = true;
		dp_data->local_status = DP_SELECT_CONNECTED(dp_connected);
	}

	return valid_connected;
}

/*
 * If we support ufp_d & dfp_d both, we should decide to use which role.
 * For dfp_u, the dp_connected is invalid, and re-send dp_status.
 * For ufp_u, the dp_connected is valid, and wait for dp_status from dfp_u
 *
 * If we don't support both, the dp_connected always is valid
 *
 */

static inline bool dp_update_dp_connected_both(struct pd_port *pd_port,
			uint32_t dp_local_connected, bool both_connected_valid)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	bool valid_connected = true;

	if (dp_local_connected == DPSTS_BOTH_CONNECTED) {
		dp_data->local_status = pd_port->dp_second_connected;
		valid_connected = both_connected_valid;
	}

	return valid_connected;
}

/* DP : DFP_U */
#if DP_DBG_ENABLE
static const char * const dp_dfp_u_state_name[] = {
	"dp_dfp_u_none",
	"dp_dfp_u_discover_id",
	"dp_dfp_u_discover_svids",
	"dp_dfp_u_discover_modes",
	"dp_dfp_u_discover_cable",
	"dp_dfp_u_enter_mode",
	"dp_dfp_u_status_update",
	"dp_dfp_u_wait_attention",
	"dp_dfp_u_configure",
	"dp_dfp_u_operation",
};
#endif /* DP_DBG_ENABLE */

static void dp_dfp_u_set_state(struct pd_port *pd_port, uint8_t state)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	dp_data->dfp_u_state = state;

	if (dp_data->dfp_u_state < DP_DFP_U_STATE_NR)
		DP_DBG("%s\n", dp_dfp_u_state_name[state]);
	else
		DP_DBG("dp_dfp_u_stop (%d)\n", state);
}

bool dp_dfp_u_notify_pe_startup(
		struct pd_port *pd_port, struct svdm_svid_data *svid_data)
{
	if (!pd_is_support_modal_operation(pd_port))
		return false;

	if (pd_port->dpm_caps & DPM_CAP_ATTEMPT_ENTER_DP_MODE)
		dp_dfp_u_set_state(pd_port, DP_DFP_U_DISCOVER_ID);

	return false;
}

bool dp_dfp_u_notify_pe_ready(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	DPM_DBG("DPM: %s\n", __func__);

	if (pd_port->data_role != PD_ROLE_DFP)
		return false;

	if (dp_data->dfp_u_state != DP_DFP_U_DISCOVER_MODES)
		return false;

	/* Check Cable later */
	pd_port->mode_svid = USB_SID_DISPLAYPORT;
	pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_DISCOVER_MODES);
	return true;
}

bool dp_notify_pe_shutdown(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data)
{
	if (svid_data->active_mode) {
		pd_send_vdm_exit_mode(pd_port, TCPC_TX_SOP,
			svid_data->svid, svid_data->active_mode);
	}

	return false;
}

bool dp_dfp_u_notify_discover_id(struct pd_port *pd_port,
	struct svdm_svid_data *svid_data, bool ack)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	uint32_t *payload = pd_get_msg_vdm_data_payload(pd_port);

	if (dp_data->dfp_u_state != DP_DFP_U_DISCOVER_ID)
		return false;

	if (!ack || !payload) {
		dp_dfp_u_set_state(pd_port,
				DP_DFP_U_ERR_DISCOVER_ID_NAK_TIMEOUT);
		return false;
	}

	if (payload[VDO_DISCOVER_ID_IDH] & PD_IDH_MODAL_SUPPORT)
		dp_dfp_u_set_state(pd_port, DP_DFP_U_DISCOVER_SVIDS);
	else
		dp_dfp_u_set_state(pd_port, DP_DFP_U_ERR_DISCOVER_ID_TYPE);

	return false;
}

bool dp_dfp_u_notify_discover_svids(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data, bool ack)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	if (dp_data->dfp_u_state != DP_DFP_U_DISCOVER_SVIDS)
		return false;

	if (!ack) {
		dp_dfp_u_set_state(pd_port,
			DP_DFP_U_ERR_DISCOVER_SVIDS_NAK_TIMEOUT);
		return false;
	}

	if (!svid_data->exist) {
		dp_dfp_u_set_state(pd_port, DP_DFP_U_ERR_DISCOVER_SVIDS_DP_SID);
		return false;
	}

	dp_dfp_u_set_state(pd_port, DP_DFP_U_DISCOVER_MODES);
	return false;
}

static inline bool is_dp_cap_valid(uint32_t dp_cap)
{
	if  ((((dp_cap >> 24) & 0x3f) == 0) && ((dp_cap & 0x00ffffff) != 0))
		return true;
	return false;
}

/* priority : D -> C -> E */
static inline int eval_dp_match_score(struct pd_port *pd_port,
	uint32_t local_mode, uint32_t remote_mode,
	uint32_t *local_dp_config, uint32_t *remote_dp_config)
{
	bool local_is_dp_src = false, local_is_preferred_role = false;
	uint32_t local_pin_assignment = 0, remote_pin_assignment = 0;
	uint32_t common_pin_assignment = 0, pin = 0, sig = DP_SIG_HBR3;
	int score = 0;

	if (!DP_DFP_U_CHECK_ROLE_CAP_MATCH(local_mode, remote_mode))
		return 0;

	if (((local_mode & MODE_DP_BOTH) == 0) ||
		((remote_mode & MODE_DP_BOTH) == 0))
		return 0;

	if (MODE_DP_PORT_CAP(local_mode) & MODE_DP_SRC &&
	    MODE_DP_PORT_CAP(remote_mode) & MODE_DP_SNK) {
		local_is_dp_src = true;
		local_is_preferred_role =
			pd_port->dp_second_connected == DPSTS_DFP_D_CONNECTED;
	}

	if (!local_is_preferred_role &&
	    MODE_DP_PORT_CAP(local_mode) & MODE_DP_SNK &&
	    MODE_DP_PORT_CAP(remote_mode) & MODE_DP_SRC) {
		local_is_dp_src = false;
		local_is_preferred_role =
			pd_port->dp_second_connected == DPSTS_UFP_D_CONNECTED;
	}

	if (local_is_dp_src) {
		local_pin_assignment = PD_DP_DFP_D_PIN_CAPS(local_mode);
		remote_pin_assignment = PD_DP_UFP_D_PIN_CAPS(remote_mode);
	} else {
		local_pin_assignment = PD_DP_UFP_D_PIN_CAPS(local_mode);
		remote_pin_assignment = PD_DP_DFP_D_PIN_CAPS(remote_mode);
	}

	common_pin_assignment = local_pin_assignment & remote_pin_assignment;
	if (common_pin_assignment & MODE_DP_PIN_D) {
		score = 3;
		pin = DP_PIN_ASSIGN_SUPPORT_D;
	} else if (common_pin_assignment & MODE_DP_PIN_C) {
		score = 2;
		pin = DP_PIN_ASSIGN_SUPPORT_C;
	} else if (common_pin_assignment & MODE_DP_PIN_E &&
		   !MODE_DP_RECEPT(remote_mode)) {
		score = 1;
		pin = DP_PIN_ASSIGN_SUPPORT_E;
	}
	if (pin) {
		if (local_is_dp_src) {
			*local_dp_config =
				VDO_DP_CFG(0, 0, pin, sig, DP_CONFIG_DFP_D);
			*remote_dp_config =
				VDO_DP_CFG(0, 0, pin, sig, DP_CONFIG_UFP_D);
		} else {
			*local_dp_config =
				VDO_DP_CFG(0, 0, pin, sig, DP_CONFIG_UFP_D);
			*remote_dp_config =
				VDO_DP_CFG(0, 0, pin, sig, DP_CONFIG_DFP_D);
		}
		if (local_is_preferred_role)
			score += 3;
	}

	return score;
}

static inline uint8_t dp_dfp_u_select_mode(struct pd_port *pd_port,
	struct dp_data *dp_data, struct svdm_svid_data *svid_data)
{
	uint32_t dp_local_mode, dp_remote_mode,
		 local_dp_config = 0, remote_dp_config = 0;
	struct svdm_mode *local, *remote;
	int i, j;
	int match_score, best_match_score = 0;
	int __maybe_unused local_index = -1;
	int remote_index = -1;
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	local = &svid_data->local_mode;
	remote = &svid_data->remote_mode;

	for (j = 0; j < local->mode_cnt; j++) {
		dp_local_mode = local->mode_vdo[j];
		if (!is_dp_cap_valid(dp_local_mode))
			continue;
		for (i = 0; i < remote->mode_cnt; i++) {
			dp_remote_mode = remote->mode_vdo[i];
			if (!is_dp_cap_valid(dp_remote_mode))
				continue;
			match_score = eval_dp_match_score(pd_port,
				dp_local_mode, dp_remote_mode,
				&local_dp_config, &remote_dp_config);
			if (match_score > best_match_score) {
				local_index = j;
				remote_index = i;
				dp_data->local_config = local_dp_config;
				dp_data->remote_config = remote_dp_config;
			}
		}
	}

#if DP_INFO_ENABLE
	for (i = 0; i < remote->mode_cnt; i++)
		DP_INFO("Mode%d=0x%08x\n", i, remote->mode_vdo[i]);

	DP_INFO("SelectMode:%d\n", remote_index);
#endif	/* DP_INFO_ENABLE */

	return remote_index + 1;
}

bool dp_dfp_u_notify_discover_modes(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data, bool ack)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	uint32_t mode = 0;
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	if (dp_data->dfp_u_state != DP_DFP_U_DISCOVER_MODES)
		return false;

	if (!ack) {
		dp_dfp_u_set_state(pd_port,
			DP_DFP_U_ERR_DISCOVER_MODES_NAK_TIMEROUT);
		return false;
	}

	if (svid_data->remote_mode.mode_cnt == 0) {
		dp_dfp_u_set_state(pd_port, DP_DFP_U_ERR_DISCOVER_MODES_DP_SID);
		return false;
	}

	pd_port->mode_obj_pos = dp_dfp_u_select_mode(
		pd_port, dp_data, svid_data);

	if (pd_port->mode_obj_pos == 0) {
		DPM_DBG("Can't find match mode\n");
		dp_dfp_u_set_state(pd_port, DP_DFP_U_ERR_DISCOVER_MODES_CAP);
		return false;
	}

	dp_data->cable_signal = DP_SIG_HBR3;
	dp_data->uhbr13_5 = false;
	dp_data->active_component = DP_CONFIG_AC_PASSIVE;

	mode = svid_data->remote_mode.mode_vdo[pd_port->mode_obj_pos-1];
	if (MODE_DP_VDO_VERSION(mode)) {
		pd_port->cable_mode_svid = 0;
		pd_port->cable_mode_obj_pos = 0;
		pd_port->cable_svid_to_discover = 0;
		pd_port->pe_data.discover_id_counter = 0;
		pd_port->pe_data.cable_discovered_state = CABLE_DISCOVERED_NONE;
		dp_dfp_u_set_state(pd_port, DP_DFP_U_DISCOVER_CABLE);
		dpm_reaction_set(pd_port, DPM_REACTION_DISCOVER_CABLE_FLOW);
	} else {
		dp_dfp_u_set_state(pd_port, DP_DFP_U_ENTER_MODE);
		pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_ENTER_MODE);
	}
	return true;
}

bool dp_dfp_u_notify_enter_mode(struct pd_port *pd_port,
	struct svdm_svid_data *svid_data, uint8_t ops, bool ack)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	if (dp_data->dfp_u_state != DP_DFP_U_ENTER_MODE)
		return false;

	if (!ack) {
		dp_dfp_u_set_state(pd_port,
				DP_DFP_U_ERR_ENTER_MODE_NAK_TIMEOUT);
		return false;
	}

	if (svid_data->active_mode == 0) {
		dp_dfp_u_set_state(pd_port, DP_DFP_U_ERR_ENTER_MODE_DP_SID);
		return false;
	}

	dp_data->local_status = pd_port->dp_first_connected;
	dp_dfp_u_set_state(pd_port, DP_DFP_U_STATUS_UPDATE);

#if CONFIG_USB_PD_DBG_DP_DFP_D_AUTO_UPDATE
	/*
	 * For Real Product,
	 * DFP_U should not send status_update until USB status is changed
	 *	From : "USB Mode, USB Configration"
	 *	To : "DisplayPlay Mode, USB Configration"
	 *
	 * After USB status is changed,
	 * please call following function to continue DFP_U flow.
	 * tcpm_dpm_dp_status_update(tcpc, 0, 0, NULL)
	 */

	pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_DP_STATUS_UPDATE);
	return true;
#else
	return false;
#endif	/* CONFIG_USB_PD_DBG_DP_DFP_D_AUTO_UPDATE */
}

bool dp_dfp_u_notify_exit_mode(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data, uint8_t ops)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	if (dp_data->dfp_u_state <= DP_DFP_U_ENTER_MODE)
		return false;

	if (svid_data->svid != USB_SID_DISPLAYPORT)
		return false;

	memset(dp_data, 0, sizeof(struct dp_data));
	dp_dfp_u_set_state(pd_port, DP_DFP_U_NONE);
	return false;
}

static inline bool dp_dfp_u_select_pin_mode(struct pd_port *pd_port)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	uint32_t dp_local_connected;
	uint32_t dp_mode[2], pin_cap[2];
	uint32_t pin_caps, sig = dp_data->cable_signal;
	struct svdm_svid_data *svid_data =
		dpm_get_svdm_svid_data(pd_port, USB_SID_DISPLAYPORT);
	uint8_t ver_min = pd_get_svdm_ver_min(pd_port, TCPC_TX_SOP);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	if (svid_data == NULL)
		return false;

	dp_mode[0] = SVID_DATA_LOCAL_MODE(svid_data, 0);
	dp_mode[1] = SVID_DATA_DFP_GET_ACTIVE_MODE(svid_data);

	dp_local_connected = PD_VDO_DPSTS_CONNECT(dp_data->local_status);

	switch (dp_local_connected) {
	case DPSTS_DFP_D_CONNECTED:
		pin_cap[0] = PD_DP_DFP_D_PIN_CAPS(dp_mode[0]);
		pin_cap[1] = PD_DP_UFP_D_PIN_CAPS(dp_mode[1]);
		break;
	case DPSTS_UFP_D_CONNECTED:
		pin_cap[0] = PD_DP_UFP_D_PIN_CAPS(dp_mode[0]);
		pin_cap[1] = PD_DP_DFP_D_PIN_CAPS(dp_mode[1]);
		break;
	default:
		DP_ERR("select_pin error1\n");
		return false;
	}

	PE_DBG("modes=0x%x 0x%x\n", dp_mode[0], dp_mode[1]);
	PE_DBG("pins=0x%x 0x%x\n", pin_cap[0], pin_cap[1]);

	pin_caps = pin_cap[0] & pin_cap[1];

	if (!pin_caps) {
		DP_ERR("select_pin error2\n");
		return false;
	}

	/* Priority */
	if ((pin_caps & (MODE_DP_PIN_C | MODE_DP_PIN_D)) ==
			(MODE_DP_PIN_C | MODE_DP_PIN_D)) {
		if (PD_VDO_DPSTS_MF_PREF(dp_data->remote_status))
			pin_caps = DP_PIN_ASSIGN_SUPPORT_D;
		else
			pin_caps = DP_PIN_ASSIGN_SUPPORT_C;
	} else if (pin_caps & MODE_DP_PIN_D)
		pin_caps = DP_PIN_ASSIGN_SUPPORT_D;
	else if (pin_caps & MODE_DP_PIN_C)
		pin_caps = DP_PIN_ASSIGN_SUPPORT_C;
	else if (pin_caps & MODE_DP_PIN_E &&
		 !MODE_DP_RECEPT(dp_mode[1]))
		pin_caps = DP_PIN_ASSIGN_SUPPORT_E;
	else {
		DP_ERR("select_pin error3\n");
		return false;
	}

	if (dp_local_connected == DPSTS_DFP_D_CONNECTED) {
		dp_data->local_config =
			VDO_DP_CFG(dp_data->active_component, dp_data->uhbr13_5,
				   pin_caps, sig, DP_CONFIG_DFP_D);
		dp_data->remote_config =
			VDO_DP_CFG(dp_data->active_component, dp_data->uhbr13_5,
				   pin_caps, sig, DP_CONFIG_UFP_D);
	} else {
		dp_data->local_config =
			VDO_DP_CFG(dp_data->active_component, dp_data->uhbr13_5,
				   pin_caps, sig, DP_CONFIG_UFP_D);
		dp_data->remote_config =
			VDO_DP_CFG(dp_data->active_component, dp_data->uhbr13_5,
				   pin_caps, sig, DP_CONFIG_DFP_D);
	}
	dp_data->local_config |= ver_min << DP_CFG_VDO_VERSION_SHFT;
	dp_data->remote_config |= ver_min << DP_CFG_VDO_VERSION_SHFT;

	return true;
}

static bool dp_dfp_u_request_dp_configuration(struct pd_port *pd_port)
{
	if (!dp_dfp_u_select_pin_mode(pd_port)) {
		dp_dfp_u_set_state(pd_port,
			DP_DFP_U_ERR_CONFIGURE_SELECT_MODE);
		return false;
	}

	tcpci_dp_notify_config_start(pd_port->tcpc);

	dp_dfp_u_set_state(pd_port, DP_DFP_U_CONFIGURE);
	pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_DP_CONFIG);
	return true;
}

static inline bool dp_dfp_u_update_dp_connected(struct pd_port *pd_port)
{
	bool valid_connected = false;
	uint32_t dp_connected, dp_local_connected;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	dp_connected = PD_VDO_DPSTS_CONNECT(dp_data->remote_status);
	dp_local_connected = PD_VDO_DPSTS_CONNECT(dp_data->local_status);

	switch (dp_connected) {
	case DPSTS_DFP_D_CONNECTED:
	case DPSTS_UFP_D_CONNECTED:
		valid_connected = dp_update_dp_connected_one(
			pd_port, dp_connected, dp_local_connected);

		if (!valid_connected) {
			dp_dfp_u_set_state(pd_port,
				DP_DFP_U_ERR_STATUS_UPDATE_ROLE);
		}
		break;

	case DPSTS_DISCONNECT:
		dp_dfp_u_set_state(pd_port, DP_DFP_U_WAIT_ATTENTION);
		break;

	case DPSTS_BOTH_CONNECTED:
		valid_connected = dp_update_dp_connected_both(
			pd_port, dp_local_connected, false);

		if (!valid_connected) {
			DP_INFO("BOTH_SEL_ONE\n");
			pd_put_tcp_vdm_event(pd_port,
				TCP_DPM_EVT_DP_STATUS_UPDATE);
		}
		break;
	}

	return valid_connected;
}

static bool dp_dfp_u_notify_dp_status_update(struct pd_port *pd_port, bool ack)
{
	bool oper_mode = false;
	bool valid_connected = true;
	uint32_t *ptr;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	switch (dp_data->dfp_u_state) {
	case DP_DFP_U_OPERATION:
		oper_mode = true;
		fallthrough;
	case DP_DFP_U_STATUS_UPDATE:
		break;

	default:
		return false;
	}

	if (!ack) {
		tcpci_dp_notify_status_update_done(tcpc, 0, false);
		dp_dfp_u_set_state(pd_port,
				DP_DFP_U_ERR_STATUS_UPDATE_NAK_TIMEOUT);
		return false;
	}

	if (dpm_vdm_get_svid(pd_port) != USB_SID_DISPLAYPORT) {
		dp_dfp_u_set_state(pd_port, DP_DFP_U_ERR_STATUS_UPDATE_DP_SID);
		return false;
	}

	ptr = pd_get_msg_vdm_data_payload(pd_port);
	if (!ptr)
		dp_data->remote_status = 0;
	else
		dp_data->remote_status = ptr[0];
	DP_INFO("dp_status: 0x%x\n", dp_data->remote_status);

	if (oper_mode) {
		tcpci_dp_notify_status_update_done(
				tcpc, dp_data->remote_status, ack);
	} else {
		valid_connected =
			dp_dfp_u_update_dp_connected(pd_port);
		if (valid_connected)
			valid_connected =
				dp_dfp_u_request_dp_configuration(pd_port);
	}
	return valid_connected;
}

static inline bool dp_ufp_u_auto_update(struct pd_port *pd_port)
{
	bool ret = false;
#if CONFIG_USB_PD_DBG_DP_UFP_U_AUTO_UPDATE
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	if (dp_data->dfp_u_state == DP_DFP_U_OPERATION)
		goto out;

	pd_port->mode_svid = USB_SID_DISPLAYPORT;
	dp_data->local_status |= DPSTS_DP_ENABLED | DPSTS_DP_HPD_STATUS;
	pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_DP_STATUS_UPDATE);
	ret = true;
out:
#endif	/* CONFIG_USB_PD_DBG_DP_UFP_U_AUTO_UPDATE */
	return ret;
}

static bool dp_dfp_u_notify_dp_configuration(struct pd_port *pd_port, bool ack)
{
	bool ret = false;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	if (ack) {
		ret = dp_ufp_u_auto_update(pd_port);
		dp_dfp_u_set_state(pd_port, DP_DFP_U_OPERATION);
	} else
		DP_ERR("config failed: 0x%0x\n", dp_data->remote_config);

	tcpci_dp_notify_config_done(tcpc,
		dp_data->local_config, dp_data->remote_config, ack);

	return ret;
}

bool dp_dfp_u_notify_attention(struct pd_port *pd_port,
	struct svdm_svid_data *svid_data)
{
	bool valid_connected = true;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	uint32_t *ptr;
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	ptr = pd_get_msg_vdm_data_payload(pd_port);
	if (!ptr)
		dp_data->remote_status = 0;
	else
		dp_data->remote_status = ptr[0];

	DP_INFO("dp_status: 0x%x\n", dp_data->remote_status);

	switch (dp_data->dfp_u_state) {
	case DP_DFP_U_WAIT_ATTENTION:
		valid_connected =
			dp_dfp_u_update_dp_connected(pd_port);
		if (valid_connected)
			valid_connected =
				dp_dfp_u_request_dp_configuration(pd_port);
		break;

	case DP_DFP_U_OPERATION:
		tcpci_dp_attention(tcpc, dp_data->remote_status);
		break;
	}

	return valid_connected;
}

bool dp_dfp_u_notify_discover_cable_id(struct pd_port *pd_port,
	struct svdm_svid_data *svid_data, bool ack,
	uint32_t *payload, uint8_t cnt)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	uint32_t idh = 0;

	if (dp_data->dfp_u_state != DP_DFP_U_DISCOVER_CABLE)
		return false;

	if (!ack || !payload || cnt < 4)
		goto next_state;

	idh = payload[VDO_DISCOVER_ID_IDH];
	dp_data->usb_signal =
		PD_VDO_CABLE_USB_SIGNAL(payload[VDO_DISCOVER_ID_CABLE]);
	dp_data->active_cable = false;
	switch (PD_IDH_PTYPE(idh)) {
	case IDH_PTYPE_PCABLE:
		if (!dp_data->usb_signal)
			goto next_state;
		return false;
	case IDH_PTYPE_ACABLE:
		dp_data->active_cable = true;
		return false;
	default:
		goto next_state;
	}
next_state:
	dpm_reaction_clear(pd_port, DPM_REACTION_DISCOVER_CABLE);
	dp_dfp_u_set_state(pd_port, DP_DFP_U_ENTER_MODE);
	pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_ENTER_MODE);
	return true;
}

bool dp_dfp_u_notify_discover_cable_svids(struct pd_port *pd_port,
	struct svdm_svid_data *svid_data, bool ack,
	uint32_t *payload, uint8_t cnt)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	struct svdm_svid_list *cable_svids = &svid_data->cable_svids;
	int i = 0, j = 0, k = 0;
	uint16_t svid[2];

	if (dp_data->dfp_u_state != DP_DFP_U_DISCOVER_CABLE)
		return false;

	if (!ack || !payload)
		goto next_state;

	for (i = 0; i < cnt; i++) {
		svid[0] = PD_VDO_SVID_SVID0(payload[i]);
		svid[1] = PD_VDO_SVID_SVID1(payload[i]);
		for (j = 0; j < 2; j++) {
			for (k = 0; k < cable_svids->cnt; k++) {
				if (dp_data->cable_svids_cnt >=
				    ARRAY_SIZE(dp_data->cable_svids))
					goto out;
				if (svid[j] == cable_svids->svids[k]) {
					dp_data->cable_svids[
						dp_data->cable_svids_cnt++] =
						svid[j];
					break;
				}
			}
		}
	}
out:
	/* simple sorting */
	for (k = 0, j = 0; k < cable_svids->cnt; k++) {
		svid[0] = cable_svids->svids[k];
		for (i = j; i < dp_data->cable_svids_cnt &&
			    j < dp_data->cable_svids_cnt; i++) {
			if (dp_data->cable_svids[i] != svid[0])
				continue;
			if (i == j) {
				j++;
				break;
			}
			/* swap */
			svid[1] = dp_data->cable_svids[j];
			dp_data->cable_svids[j] = dp_data->cable_svids[i];
			dp_data->cable_svids[i] = svid[1];
			j++;
			break;
		}
	}
	if (dp_data->cable_svids_cnt) {
		pd_port->cable_mode_svid =
			dp_data->cable_svids[dp_data->cable_svids_cnt-1];
		pd_port->cable_svid_to_discover = dp_data->cable_svids[0];
	}
	return !!dp_data->cable_svids_cnt;
next_state:
	if (!dp_data->active_cable) {
		dp_data->cable_signal |= DP_SIG_UHBR10;
		switch (dp_data->usb_signal) {
		case CABLE_USBSS_U32_GEN1:
		case CABLE_USBSS_U32_GEN2:
			break;
		default:
			dp_data->uhbr13_5 = true;
			dp_data->cable_signal |= DP_SIG_UHBR20;
			break;
		}
	}
	dpm_reaction_clear(pd_port, DPM_REACTION_DISCOVER_CABLE);
	dp_dfp_u_set_state(pd_port, DP_DFP_U_ENTER_MODE);
	pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_ENTER_MODE);
	return true;
}

bool dp_dfp_u_notify_discover_cable_modes(struct pd_port *pd_port,
	struct svdm_svid_data *svid_data, bool ack,
	uint32_t *payload, uint8_t cnt)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	uint32_t remote_pin = DP_CFG_PIN(dp_data->remote_config);
	uint32_t cable_pin = 0, mode = 0;
	bool remote_is_dp_src = PD_DP_CFG_DFP_D(dp_data->remote_config);
	int i = 0;
	uint16_t cable_svid = 0;
	uint8_t obj_pos = 0;

	if (dp_data->dfp_u_state != DP_DFP_U_DISCOVER_CABLE)
		return false;

	if (!ack || !payload || cnt < 1 ||
	    dp_data->cable_svids_idx >= dp_data->cable_svids_cnt)
		goto next_state;

	cable_svid = dp_data->cable_svids[dp_data->cable_svids_idx++];

	if (cable_svid == USB_SID_TBT) {
		mode = payload[0];
		if (mode & BIT(22)) {
			dp_data->active_component = DP_CONFIG_AC_ACTIVE_RETIMER;
		} else if (mode & BIT(25)) {
			dp_data->active_component =
				DP_CONFIG_AC_ACTIVE_REDRIVER;
		} else {
			if (mode & BIT(21))
				dp_data->active_component =
					DP_CONFIG_AC_OPTICAL;
			switch ((mode >> 16) & 0x7) {
			case 3:
				dp_data->uhbr13_5 = true;
				dp_data->cable_signal |= DP_SIG_UHBR20;
				fallthrough;
			case 1:
			case 2:
				dp_data->cable_signal |= DP_SIG_UHBR10;
				fallthrough;
			default:
				break;
			}
		}
		if (dp_data->cable_svids_idx >= dp_data->cable_svids_cnt)
			goto next_state;
		else {
			pd_port->cable_svid_to_discover =
				dp_data->cable_svids[dp_data->cable_svids_idx];
			return true;
		}
	}

	for (i = 0; i < cnt; i++) {
		mode = payload[i];
		cable_pin = remote_is_dp_src ? MODE_DP_PIN_DFP(mode) :
					       MODE_DP_PIN_UFP(mode);
		if (cable_pin & remote_pin) {
			obj_pos = i + 1;
			break;
		}
	}
	if (!obj_pos)
		goto next_state;
	pd_port->cable_mode_obj_pos = obj_pos;

	dp_data->cable_signal |= MODE_DP_SIGNAL_SUPPORT(mode);
	if (dp_data->cable_signal & DP_SIG_UHBR20)
		dp_data->uhbr13_5 = true;
	else
		dp_data->uhbr13_5 = dp_data->uhbr13_5 ||
				    MODE_DP_UHBR13_5(mode);
	dp_data->active_component |= MODE_DP_ACTIVE_COMPONENT(mode);
next_state:
	dpm_reaction_clear(pd_port, DPM_REACTION_DISCOVER_CABLE);
	dp_dfp_u_set_state(pd_port, DP_DFP_U_ENTER_MODE);
	pd_put_tcp_vdm_event(pd_port, TCP_DPM_EVT_ENTER_MODE);
	return true;
}

/* DP : UFP_U */

#if DPM_DBG_ENABLE
static const char * const dp_ufp_u_state_name[] = {
	"dp_ufp_u_none",
	"dp_ufp_u_startup",
	"dp_ufp_u_wait",
	"dp_ufp_u_operation",
};
#endif /* DPM_DBG_ENABLE */

static void dp_ufp_u_set_state(struct pd_port *pd_port, uint8_t state)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	dp_data->ufp_u_state = state;

	if (dp_data->ufp_u_state < DP_UFP_U_STATE_NR)
		DPM_DBG("%s\n", dp_ufp_u_state_name[state]);
	else
		DPM_DBG("dp_ufp_u_stop\n");
}

bool dp_ufp_u_request_enter_mode(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data, uint8_t ops)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	dp_data->local_status = pd_port->dp_first_connected;

	if (pd_port->dp_first_connected == DPSTS_DISCONNECT)
		dp_ufp_u_set_state(pd_port, DP_UFP_U_STARTUP);
	else
		dp_ufp_u_set_state(pd_port, DP_UFP_U_WAIT);

	return false;
}

bool dp_ufp_u_request_exit_mode(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data, uint8_t ops)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	memset(dp_data, 0, sizeof(struct dp_data));
	dp_ufp_u_set_state(pd_port, DP_UFP_U_NONE);

	return false;
}

static inline bool dp_ufp_u_update_dp_connected(struct pd_port *pd_port)
{
	bool valid_connected = false;
	uint32_t dp_connected, dp_local_connected;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	dp_connected = PD_VDO_DPSTS_CONNECT(dp_data->remote_status);
	dp_local_connected = PD_VDO_DPSTS_CONNECT(dp_data->local_status);

	switch (dp_connected) {
	case DPSTS_DFP_D_CONNECTED:
	case DPSTS_UFP_D_CONNECTED:
		valid_connected = dp_update_dp_connected_one(
			pd_port, dp_connected, dp_local_connected);
		break;

	case DPSTS_BOTH_CONNECTED:
		valid_connected = dp_update_dp_connected_both(
			pd_port, dp_local_connected, true);
		break;

	default:
		break;
	}

	return valid_connected;
}

static inline int dp_ufp_u_request_dp_status(struct pd_port *pd_port)
{
	bool ack;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	uint32_t *ptr;

	ptr = pd_get_msg_vdm_data_payload(pd_port);
	if (!ptr)
		dp_data->remote_status = 0;
	else
		dp_data->remote_status = ptr[0];

	switch (dp_data->ufp_u_state) {
	case DP_UFP_U_WAIT:
		ack = dp_ufp_u_update_dp_connected(pd_port);
		break;

	case DP_UFP_U_STARTUP:
	case DP_UFP_U_OPERATION:
		ack = true;
		tcpci_dp_status_update(
			pd_port->tcpc, dp_data->remote_status);
		break;

	default:
		ack = false;
		break;
	}

	if (ack) {
		dp_data->local_status |= DPSTS_DP_ENABLED;
		if (pd_port->dpm_caps & DPM_CAP_DP_PREFER_MF)
			dp_data->local_status |= DPSTS_DP_MF_PREF;

		return pd_reply_svdm_request(pd_port,
			CMDT_RSP_ACK, 1, &dp_data->local_status);
	} else {
		return dpm_vdm_reply_svdm_nak(pd_port);
	}
}

static bool dp_ufp_u_is_valid_dp_config(struct pd_port *pd_port,
					uint32_t dp_config)
{
	bool ret = false;
	uint32_t cfg_signal = DP_CFG_SIGNAL(dp_config);
	uint32_t cfg_pin = DP_CFG_PIN(dp_config);
	uint32_t local_mode;
	struct svdm_svid_data *svid_data =
		dpm_get_svdm_svid_data(pd_port, USB_SID_DISPLAYPORT);

	if (svid_data == NULL)
		return false;

	local_mode = SVID_DATA_LOCAL_MODE(svid_data, 0);

	switch (PD_DP_CFG_ROLE(dp_config)) {
	case DP_CONFIG_USB:
		ret = true;
		break;

	case DP_CONFIG_DFP_D:
		if (MODE_DP_PORT_CAP(local_mode) & MODE_DP_SRC &&
		    MODE_DP_SIGNAL_SUPPORT(local_mode) & cfg_signal &&
		    PD_DP_DFP_D_PIN_CAPS(local_mode) & cfg_pin)
			ret = true;
		break;

	case DP_CONFIG_UFP_D:
		if (MODE_DP_PORT_CAP(local_mode) & MODE_DP_SNK &&
		    MODE_DP_SIGNAL_SUPPORT(local_mode) & cfg_signal &&
		    PD_DP_UFP_D_PIN_CAPS(local_mode) & cfg_pin)
			ret = true;
		break;
	}

	return ret;
}

static inline void dp_ufp_u_auto_attention(struct pd_port *pd_port)
{
#if CONFIG_USB_PD_DBG_DP_UFP_U_AUTO_ATTENTION
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	pd_port->mode_svid = USB_SID_DISPLAYPORT;
	dp_data->local_status |= DPSTS_DP_ENABLED | DPSTS_DP_HPD_STATUS;
#endif	/* CONFIG_USB_PD_DBG_DP_UFP_U_AUTO_ATTENTION */
}

static inline int dp_ufp_u_request_dp_config(struct pd_port *pd_port)
{
	bool ack = false;
	uint32_t dp_config, *ptr;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);
	struct tcpc_device __maybe_unused *tcpc = pd_port->tcpc;

	ptr = pd_get_msg_vdm_data_payload(pd_port);
	if (!ptr)
		dp_config = 0;
	else
		dp_config = ptr[0];
	DPM_DBG("dp_config: 0x%x\n", dp_config);

	switch (dp_data->ufp_u_state) {
	case DP_UFP_U_STARTUP:
	case DP_UFP_U_WAIT:
	case DP_UFP_U_OPERATION:
		ack = dp_ufp_u_is_valid_dp_config(pd_port, dp_config);

		if (ack) {
			dp_data->local_config = dp_config;
			tcpci_dp_configure(tcpc, dp_config);
			dp_ufp_u_auto_attention(pd_port);
			dp_ufp_u_set_state(pd_port, DP_UFP_U_OPERATION);
		}
		break;
	}

	return dpm_vdm_reply_svdm_request(pd_port, ack);
}

static inline void dp_ufp_u_send_dp_attention(struct pd_port *pd_port)
{
	struct svdm_svid_data *svid_data;
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	switch (dp_data->ufp_u_state) {
	case DP_UFP_U_STARTUP:
	case DP_UFP_U_OPERATION:
		svid_data = dpm_get_svdm_svid_data(
				pd_port, USB_SID_DISPLAYPORT);
		PD_BUG_ON(svid_data == NULL);

		pd_send_vdm_dp_attention(pd_port, TCPC_TX_SOP,
			svid_data->active_mode, dp_data->local_status);
		break;

	default:
		VDM_STATE_DPM_INFORMED(pd_port);
		pd_notify_tcp_vdm_event_2nd_result(
			pd_port, TCP_DPM_RET_DENIED_NOT_READY);
		break;
	}
}

/* ---- UFP : DP Only ---- */

int pd_dpm_ufp_request_dp_status(struct pd_port *pd_port)
{
	return dp_ufp_u_request_dp_status(pd_port);
}

int pd_dpm_ufp_request_dp_config(struct pd_port *pd_port)
{
	return dp_ufp_u_request_dp_config(pd_port);
}

void pd_dpm_ufp_send_dp_attention(struct pd_port *pd_port)
{
	dp_ufp_u_send_dp_attention(pd_port);
}

/* ---- DFP : DP Only ---- */

void pd_dpm_dfp_send_dp_status_update(struct pd_port *pd_port)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	pd_send_vdm_dp_status(pd_port, TCPC_TX_SOP,
		pd_port->mode_obj_pos, 1, &dp_data->local_status);
}

void pd_dpm_dfp_inform_dp_status_update(
	struct pd_port *pd_port, bool ack)
{
	VDM_STATE_DPM_INFORMED(pd_port);
	dp_dfp_u_notify_dp_status_update(pd_port, ack);
}

void pd_dpm_dfp_send_dp_configuration(struct pd_port *pd_port)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	pd_send_vdm_dp_config(pd_port, TCPC_TX_SOP,
		pd_port->mode_obj_pos, 1, &dp_data->remote_config);
}

void pd_dpm_dfp_inform_dp_configuration(
	struct pd_port *pd_port, bool ack)
{
	VDM_STATE_DPM_INFORMED(pd_port);
	dp_dfp_u_notify_dp_configuration(pd_port, ack);
}

bool dp_reset_state(struct pd_port *pd_port, struct svdm_svid_data *svid_data)
{
	struct dp_data *dp_data = pd_get_dp_data(pd_port);

	memset(dp_data, 0, sizeof(struct dp_data));
	return false;
}

#define DEFAULT_DP_ROLE_CAP				(MODE_DP_SRC)
#define DEFAULT_DP_FIRST_CONNECTED		(DPSTS_DFP_D_CONNECTED)
#define DEFAULT_DP_SECOND_CONNECTED		(DPSTS_DFP_D_CONNECTED)

static const struct {
	const char *prop_name;
	const char *legacy_prop_name;
	uint32_t mode;
} supported_dp_pin_modes[] = {
	{ "pin-assignment,mode-c", "pin_assignment,mode_c", MODE_DP_PIN_C },
	{ "pin-assignment,mode-d", "pin_assignment,mode_d", MODE_DP_PIN_D },
	{ "pin-assignment,mode-e", "pin_assignment,mode_e", MODE_DP_PIN_E },
};

static const struct {
	const char *conn_mode;
	const char *legacy_conn_mode;
	uint32_t val;
} dp_connect_mode[] = {
	{"both", "both", DPSTS_BOTH_CONNECTED},
	{"dfp-d", "dfp_d", DPSTS_DFP_D_CONNECTED},
	{"ufp-d", "ufp_d", DPSTS_UFP_D_CONNECTED},
};

bool dp_parse_svid_data(
	struct pd_port *pd_port, struct svdm_svid_data *svid_data)
{
	struct device_node *np, *ufp_np, *dfp_np;
	const char *connection;
	uint32_t ufp_d_pin_cap = 0, dfp_d_pin_cap = 0;
	uint32_t ufp_d_pin = 0, dfp_d_pin = 0;
	uint32_t sig = DP_SIG_HBR3, receptacle = 1, usb2 = 0;
	int i = 0;

	np = of_find_node_by_name(
		pd_port->tcpc->dev.of_node, "displayport");
	if (np == NULL) {
		pr_err("%s get displayport data fail\n", __func__);
		return false;
	}

	pr_info("dp, svid\n");
	svid_data->svid = USB_SID_DISPLAYPORT;
	ufp_np = of_find_node_by_name(np, "ufp-d");
	if (!ufp_np)
		ufp_np = of_find_node_by_name(np, "ufp_d");

	dfp_np = of_find_node_by_name(np, "dfp-d");
	if (!dfp_np)
		dfp_np = of_find_node_by_name(np, "dfp_d");

	if (ufp_np) {
		pr_info("dp, ufp_np\n");
		for (i = 0; i < ARRAY_SIZE(supported_dp_pin_modes); i++) {
			if (of_property_read_bool(ufp_np,
						  supported_dp_pin_modes[i].prop_name) ||
			    of_property_read_bool(ufp_np,
						  supported_dp_pin_modes[i].legacy_prop_name))
				ufp_d_pin_cap |= supported_dp_pin_modes[i].mode;
		}
	}

	if (dfp_np) {
		pr_info("dp, dfp_np\n");
		for (i = 0; i < ARRAY_SIZE(supported_dp_pin_modes); i++) {
			if (of_property_read_bool(dfp_np,
						  supported_dp_pin_modes[i].prop_name) ||
			    of_property_read_bool(dfp_np,
						  supported_dp_pin_modes[i].legacy_prop_name))
				dfp_d_pin_cap |= supported_dp_pin_modes[i].mode;
		}
	}

	if (of_property_read_bool(np, "usbr20-not-used") ||
	    of_property_read_bool(np, "usbr20_not_used"))
		usb2 = 1;
	if (of_property_read_bool(np, "typec,receptacle"))
		receptacle = 1;
	svid_data->local_mode.mode_cnt = 1;
	if (receptacle) {
		ufp_d_pin = ufp_d_pin_cap;
		dfp_d_pin = dfp_d_pin_cap;
	} else {
		ufp_d_pin = dfp_d_pin_cap;
		dfp_d_pin = ufp_d_pin_cap;
	}
	svid_data->local_mode.mode_vdo[0] = VDO_MODE_DP(
		ufp_d_pin, dfp_d_pin,
		usb2, receptacle, sig, (ufp_d_pin_cap ? MODE_DP_SNK : 0)
		| (dfp_d_pin_cap ? MODE_DP_SRC : 0));

	pd_port->dp_first_connected = DEFAULT_DP_FIRST_CONNECTED;
	pd_port->dp_second_connected = DEFAULT_DP_SECOND_CONNECTED;

	if (of_property_read_string(np, "1st-connection", &connection) == 0 ||
	    of_property_read_string(np, "1st_connection", &connection) == 0) {
		pr_info("dp, 1st_connection\n");
		for (i = 0; i < ARRAY_SIZE(dp_connect_mode); i++) {
			if (strcasecmp(connection,
				       dp_connect_mode[i].conn_mode) == 0 ||
			    strcasecmp(connection,
				       dp_connect_mode[i].legacy_conn_mode) == 0) {
				pd_port->dp_first_connected = dp_connect_mode[i].val;
				break;
			}
		}
	}

	if (of_property_read_string(np, "2nd-connection", &connection) == 0 ||
	    of_property_read_string(np, "2nd_connection", &connection) == 0) {
		pr_info("dp, 2nd_connection\n");
		for (i = 0; i < ARRAY_SIZE(dp_connect_mode); i++) {
			if (strcasecmp(connection,
				       dp_connect_mode[i].conn_mode) == 0 ||
			    strcasecmp(connection,
				       dp_connect_mode[i].legacy_conn_mode) == 0) {
				pd_port->dp_second_connected = dp_connect_mode[i].val;
				break;
			}
		}
	}
	/* 2nd connection must not be BOTH */
	PD_BUG_ON(pd_port->dp_second_connected == DPSTS_BOTH_CONNECTED);
	/* UFP or DFP can't both be invalid */
	PD_BUG_ON(ufp_d_pin_cap == 0 && dfp_d_pin_cap == 0);
	if (pd_port->dp_first_connected == DPSTS_BOTH_CONNECTED) {
		PD_BUG_ON(ufp_d_pin_cap == 0);
		PD_BUG_ON(dfp_d_pin_cap == 0);
	}

	return true;
}
#endif	/* CONFIG_USB_POWER_DELIVERY */
