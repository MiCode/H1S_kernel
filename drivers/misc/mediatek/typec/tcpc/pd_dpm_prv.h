/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef PD_DPM_PRV_H_INCLUDED
#define PD_DPM_PRV_H_INCLUDED

#include <linux/of.h>
#include <linux/device.h>

#define SVID_DATA_LOCAL_MODE(svid_data, n)	\
		(svid_data->local_mode.mode_vdo[n])

#define SVID_DATA_REMOTE_MODE(svid_data, n) \
		(svid_data->remote_mode.mode_vdo[n])

#define SVID_DATA_DFP_GET_ACTIVE_MODE(svid_data)\
	SVID_DATA_REMOTE_MODE(svid_data, svid_data->active_mode-1)

#define SVID_DATA_UFP_GET_ACTIVE_MODE(svid_data)\
	SVID_DATA_LOCAL_MODE(svid_data, svid_data->active_mode-1)

struct svdm_svid_ops {
	const char *name;
	uint16_t svid;
	struct svdm_svid_list cable_svids;

	bool (*dfp_inform_id)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, bool ack);
	bool (*dfp_inform_svids)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, bool ack);
	bool (*dfp_inform_modes)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, bool ack);

	bool (*dfp_inform_enter_mode)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, uint8_t ops, bool ack);
	bool (*dfp_inform_exit_mode)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, uint8_t ops);

	bool (*dfp_inform_attention)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data);

	bool (*dfp_inform_cable_id)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, bool ack,
		uint32_t *payload, uint8_t cnt);
	bool (*dfp_inform_cable_svids)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, bool ack,
		uint32_t *payload, uint8_t cnt);
	bool (*dfp_inform_cable_modes)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, bool ack,
		uint32_t *payload, uint8_t cnt);

	bool (*ufp_request_enter_mode)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, uint8_t ops);
	bool (*ufp_request_exit_mode)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, uint8_t ops);

	bool (*notify_pe_startup)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data);
	bool (*notify_pe_ready)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data);
	bool (*notify_pe_shutdown)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data);

#if CONFIG_USB_PD_CUSTOM_VDM
	bool (*dfp_notify_uvdm)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data, bool ack);

	bool (*ufp_notify_uvdm)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data);
#endif

	bool (*reset_state)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data);

	bool (*parse_svid_data)(struct pd_port *pd_port,
		struct svdm_svid_data *svid_data);
};

static inline bool dpm_check_data_msg_event(
	struct pd_port *pd_port, uint8_t msg)
{
	return pd_event_data_msg_match(
		pd_get_curr_pd_event(pd_port), msg);
}

#if CONFIG_USB_PD_REV30
static inline bool dpm_check_ext_msg_event(
	struct pd_port *pd_port, uint8_t msg)
{
	return pd_event_ext_msg_match(
		pd_get_curr_pd_event(pd_port), msg);
}
#endif	/* CONFIG_USB_PD_REV30 */

static inline uint8_t dpm_vdm_get_ops(struct pd_port *pd_port)
{
	return pd_port->curr_vdm_ops;
}

static inline uint16_t dpm_vdm_get_svid(struct pd_port *pd_port)
{
	return pd_port->curr_vdm_svid;
}

static inline int dpm_vdm_reply_svdm_request(
		struct pd_port *pd_port, bool ack)
{
	return pd_reply_svdm_request_simply(
		pd_port, ack ? CMDT_RSP_ACK : CMDT_RSP_NAK);
}

static inline int dpm_vdm_reply_svdm_nak(struct pd_port *pd_port)
{
	return pd_reply_svdm_request_simply(pd_port, CMDT_RSP_NAK);
}

enum {
	GOOD_PW_NONE = 0,	/* both no GP */
	GOOD_PW_PARTNER,	/* partner has GP */
	GOOD_PW_LOCAL,		/* local has GP */
	GOOD_PW_BOTH,		/* both have GPs */
};

int dpm_check_good_power(struct pd_port *pd_port);

/* SVDM */

extern struct svdm_svid_data *
	dpm_get_svdm_svid_data(struct pd_port *pd_port, uint16_t svid);
extern struct svdm_svid_data *dpm_get_svdm_svid_data_via_cable_svids(
	struct pd_port *pd_port, uint16_t svid);

extern bool svdm_reset_state(struct pd_port *pd_port);
extern bool svdm_notify_pe_startup(struct pd_port *pd_port);

static inline bool svdm_notify_pe_ready(struct pd_port *pd_port)
{
	int i = 0;
	bool ret = false;
	struct svdm_svid_data *svid_data;

	for (i = 0; i < pd_port->svid_data_cnt; i++) {
		svid_data = &pd_port->svid_data[i];
		if (svid_data->ops && svid_data->ops->notify_pe_ready) {
			ret = svid_data->ops->notify_pe_ready(
						pd_port, svid_data);
			if (ret)
				break;
		}
	}

	return ret;
}

static inline bool svdm_notify_pe_shutdown(
	struct pd_port *pd_port)
{
	int i;
	struct svdm_svid_data *svid_data;

	for (i = 0; i < pd_port->svid_data_cnt; i++) {
		svid_data = &pd_port->svid_data[i];
		if (svid_data->ops && svid_data->ops->notify_pe_shutdown) {
			svid_data->ops->notify_pe_shutdown(
				pd_port, svid_data);
		}
	}

	return 0;
}

static inline bool svdm_dfp_inform_id(struct pd_port *pd_port, bool ack)
{
	int i;
	bool ret = false;
	struct svdm_svid_data *svid_data;

	for (i = 0; i < pd_port->svid_data_cnt && !ret; i++) {
		svid_data = &pd_port->svid_data[i];
		if (!svid_data->ops || !svid_data->ops->dfp_inform_id)
			continue;
		ret = svid_data->ops->dfp_inform_id(pd_port, svid_data, ack);
	}

	return ret;
}

static inline bool svdm_dfp_inform_svids(struct pd_port *pd_port, bool ack)
{
	int i;
	bool ret = false;
	struct svdm_svid_data *svid_data;

	for (i = 0; i < pd_port->svid_data_cnt && !ret; i++) {
		svid_data = &pd_port->svid_data[i];
		if (!svid_data->ops || !svid_data->ops->dfp_inform_svids)
			continue;
		ret = svid_data->ops->dfp_inform_svids(pd_port, svid_data, ack);
	}

	return ret;
}

static inline bool svdm_dfp_inform_modes(
		struct pd_port *pd_port, uint16_t svid, bool ack)
{
	bool ret = false;
	struct svdm_svid_data *svid_data;

	svid_data = dpm_get_svdm_svid_data(pd_port, svid);
	if (svid_data == NULL)
		goto out;

	if (svid_data->ops && svid_data->ops->dfp_inform_modes)
		ret = svid_data->ops->dfp_inform_modes(pd_port, svid_data, ack);
out:
	return ret;
}

static inline bool svdm_dfp_inform_enter_mode(
	struct pd_port *pd_port, uint16_t svid, uint8_t ops, bool ack)
{
	bool ret = false;
	struct svdm_svid_data *svid_data;

	svid_data = dpm_get_svdm_svid_data(pd_port, svid);
	if (svid_data == NULL)
		goto out;

	if (svid_data->ops && svid_data->ops->dfp_inform_enter_mode)
		ret = svid_data->ops->dfp_inform_enter_mode(
						pd_port, svid_data, ops, ack);
out:
	return ret;
}

static inline bool svdm_dfp_inform_exit_mode(
	struct pd_port *pd_port, uint16_t svid, uint8_t ops)
{
	bool ret = false;
	struct svdm_svid_data *svid_data;

	svid_data = dpm_get_svdm_svid_data(pd_port, svid);
	if (svid_data == NULL)
		goto out;

	if (svid_data->ops && svid_data->ops->dfp_inform_exit_mode)
		ret = svid_data->ops->dfp_inform_exit_mode(pd_port, svid_data, ops);
out:
	return ret;
}

static inline bool svdm_dfp_inform_attention(
	struct pd_port *pd_port, uint16_t svid)
{
	bool ret = false;
	struct svdm_svid_data *svid_data;

	svid_data = dpm_get_svdm_svid_data(pd_port, svid);
	if (svid_data == NULL)
		goto out;

	if (svid_data->ops && svid_data->ops->dfp_inform_attention)
		ret = svid_data->ops->dfp_inform_attention(pd_port, svid_data);
out:
	return ret;
}

static inline bool svdm_ufp_request_enter_mode(
	struct pd_port *pd_port, uint16_t svid, uint8_t ops)
{
	bool ret = false;
	struct svdm_svid_data *svid_data;

	svid_data = dpm_get_svdm_svid_data(pd_port, svid);
	if (svid_data == NULL)
		goto out;

	if (svid_data->ops && svid_data->ops->ufp_request_enter_mode)
		ret = svid_data->ops->ufp_request_enter_mode(pd_port, svid_data, ops);
out:
	return ret;
}

static inline bool svdm_ufp_request_exit_mode(
	struct pd_port *pd_port, uint16_t svid, uint8_t ops)
{
	bool ret = false;
	struct svdm_svid_data *svid_data;

	svid_data = dpm_get_svdm_svid_data(pd_port, svid);
	if (svid_data == NULL)
		goto out;

	if (svid_data->ops && svid_data->ops->ufp_request_exit_mode)
		ret = svid_data->ops->ufp_request_exit_mode(pd_port, svid_data, ops);
out:
	return ret;
}

static inline bool svdm_dfp_inform_cable_id(struct pd_port *pd_port, bool ack,
					    uint32_t *payload, uint8_t cnt)
{
	int i;
	bool ret = false;
	struct svdm_svid_data *svid_data;

	for (i = 0; i < pd_port->svid_data_cnt && !ret; i++) {
		svid_data = &pd_port->svid_data[i];
		if (!svid_data->ops || !svid_data->ops->dfp_inform_cable_id)
			continue;
		ret = svid_data->ops->dfp_inform_cable_id(pd_port, svid_data,
							  ack, payload, cnt);
	}

	return ret;
}

static inline bool svdm_dfp_inform_cable_svids(struct pd_port *pd_port,
					       bool ack, uint32_t *payload,
					       uint8_t cnt)
{
	int i;
	bool ret = false;
	struct svdm_svid_data *svid_data;

	for (i = 0; i < pd_port->svid_data_cnt && !ret; i++) {
		svid_data = &pd_port->svid_data[i];
		if (!svid_data->ops || !svid_data->ops->dfp_inform_cable_svids)
			continue;
		ret = svid_data->ops->dfp_inform_cable_svids(pd_port, svid_data,
							     ack, payload, cnt);
	}

	return ret;
}

static inline bool svdm_dfp_inform_cable_modes(struct pd_port *pd_port,
					       uint16_t svid, bool ack,
					       uint32_t *payload, uint8_t cnt)
{
	bool ret = false;
	struct svdm_svid_data *svid_data;

	svid_data = dpm_get_svdm_svid_data_via_cable_svids(pd_port, svid);
	if (svid_data == NULL)
		goto out;

	if (svid_data->ops && svid_data->ops->dfp_inform_cable_modes)
		ret = svid_data->ops->dfp_inform_cable_modes(pd_port, svid_data,
							     ack, payload, cnt);
out:
	return ret;
}

#endif /* PD_DPM_PRV_H_INCLUDED */
