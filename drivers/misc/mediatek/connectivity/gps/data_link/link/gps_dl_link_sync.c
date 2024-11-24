/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"
#include "gps_dl_time_tick.h"

#include "gps_each_link.h"
#include "gps_dl_hal.h"
#include "gps_dl_hal_api.h"
#include "gps_dl_hal_util.h"
#include "gps_dl_hw_api.h"
#include "gps_dl_isr.h"
#include "gps_dl_lib_misc.h"
#include "gps_dsp_fsm.h"
#include "gps_dl_osal.h"
#include "gps_dl_name_list.h"
#include "gps_dl_context.h"
#include "gps_dl_subsys_reset.h"


#define GDL_TEST_TRUE_AND_SET_FALSE(x, x_old) \
	do {                \
		x_old = x;      \
		if (x_old) {    \
			x = false;  \
	} } while (0)

#define GDL_TEST_FALSE_AND_SET_TRUE(x, x_old) \
	do {                \
		x_old = x;      \
		if (!x_old) {   \
			x = true;   \
	} } while (0)

enum GDL_RET_STATUS gps_dl_link_wait_on(struct gps_each_link_waitable *p, long *p_sigval)
{
#if GPS_DL_ON_LINUX
	long val;
	bool is_fired;

	p->waiting = true;
	/* TODO: check race conditions? */
	GDL_TEST_TRUE_AND_SET_FALSE(p->fired, is_fired);
	if (is_fired) {
		GDL_LOGD("waitable = %s, no wait return", gps_dl_waitable_type_name(p->type));
		p->waiting = false;
		return GDL_OKAY;
	}

	GDL_LOGD("waitable = %s, wait start", gps_dl_waitable_type_name(p->type));
	val = wait_event_interruptible(p->wq, p->fired);
	p->waiting = false;

	if (val) {
		GDL_LOGI("signaled by %ld", val);
		if (p_sigval)
			*p_sigval = val;
		p->waiting = false;
		return GDL_FAIL_SIGNALED;
	}

	p->fired = false;
	p->waiting = false;
	GDL_LOGD("waitable = %s, wait done", gps_dl_waitable_type_name(p->type));
	return GDL_OKAY;
#else
	return GDL_FAIL_NOT_SUPPORT;
#endif
}

enum GDL_RET_STATUS gps_dl_link_try_wait_on(enum gps_dl_link_id_enum link_id,
	enum gps_each_link_waitable_type type)
{
	struct gps_each_link *p_link;
	struct gps_each_link_waitable *p;
	bool is_fired;

	p_link = gps_dl_link_get(link_id);
	p = &p_link->waitables[type];

	GDL_TEST_TRUE_AND_SET_FALSE(p->fired, is_fired);
	if (is_fired) {
		GDL_LOGD("waitable = %s, okay", gps_dl_waitable_type_name(p->type));
		p->waiting = false;
		return GDL_OKAY;
	}

	return GDL_FAIL;
}

void gps_dl_link_wake_up(struct gps_each_link_waitable *p)
{
	bool is_fired = false;

	ASSERT_NOT_NULL(p, GDL_VOIDF());

	if (!p->waiting) {
		if (p->type == GPS_DL_WAIT_WRITE || p->type == GPS_DL_WAIT_READ) {
			/* normal case for read/write, not show warning */
			GDL_LOGD("waitable = %s, nobody waiting",
				gps_dl_waitable_type_name(p->type));
		} else {
			/* not return, just show warning */
			GDL_LOGW("waitable = %s, nobody waiting",
				gps_dl_waitable_type_name(p->type));
		}
	}

	GDL_TEST_FALSE_AND_SET_TRUE(p->fired, is_fired);
	GDL_LOGD("waitable = %s, fired = %d", gps_dl_waitable_type_name(p->type), is_fired);

	if (!is_fired) {
#if GPS_DL_ON_LINUX
		wake_up(&p->wq);
#else
#endif
	}
}

void gps_dl_link_open_wait(enum gps_dl_link_id_enum link_id, long *p_sigval)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	enum GDL_RET_STATUS gdl_ret;
	long sigval;

	gdl_ret = gps_dl_link_wait_on(&p->waitables[GPS_DL_WAIT_OPEN_CLOSE], &sigval);
	if (gdl_ret == GDL_FAIL_SIGNALED) {
		/*
		 * LINK_OPENING should be a temp state. if code arriving here,
		 * it means something block LINK_OPENING changing to LINK_OPENED
		 * use the api to dump and check whether it's blocked by conn infra driver operation
		 */
		gps_dl_hal_conn_infra_driver_debug_dump();
		if (p_sigval != NULL) {
			*p_sigval = sigval;
			return;
		}
	} else if (gdl_ret == GDL_FAIL_NOT_SUPPORT)
		; /* show warnning */
}

void gps_dl_link_open_ack(enum gps_dl_link_id_enum link_id, bool okay, bool hw_resume)
{
#if 0
	enum GDL_RET_STATUS gdl_ret;
	struct gdl_dma_buf_entry dma_buf_entry;
#endif
	struct gps_each_link *p = gps_dl_link_get(link_id);
	bool send_msg = false;

	GDL_LOGXD_ONF(link_id, "hw_resume = %d", hw_resume);

	gps_each_link_take_big_lock(link_id, GDL_LOCK_FOR_OPEN_DONE);
	if (gps_each_link_get_bool_flag(link_id, LINK_USER_OPEN) && okay) {
		GDL_LOGXW_ONF(link_id,
			"user still online, try to change to opened");

		/* Note: if pre_status not OPENING, it might be RESETTING, not handle it here */
		if (hw_resume)
			gps_each_link_change_state_from(link_id, LINK_RESUMING, LINK_OPENED);
		else
			gps_each_link_change_state_from(link_id, LINK_OPENING, LINK_OPENED);

		/* TODO: ack on DSP reset done */
#if 0
		/* if has pending data, can send it now */
		gdl_ret = gdl_dma_buf_get_data_entry(&p->tx_dma_buf, &dma_buf_entry);
		if (gdl_ret == GDL_OKAY)
			gps_dl_hal_a2d_tx_dma_start(link_id, &dma_buf_entry);
#endif
	} else {
		GDL_LOGXW_ONF(link_id,
			"okay = %d or user already offline, try to change to closing", okay);

		/* Note: if pre_status not OPENING, it might be RESETTING, not handle it here */
		if (gps_each_link_change_state_from(link_id, LINK_OPENING, LINK_CLOSING))
			send_msg = true;
	}

	gps_each_link_set_bool_flag(link_id, LINK_OPEN_RESULT_OKAY, okay);
	gps_dl_link_wake_up(&p->waitables[GPS_DL_WAIT_OPEN_CLOSE]);

	gps_each_link_give_big_lock(link_id);

	if (send_msg) {
		gps_dl_link_event_send(GPS_DL_EVT_LINK_CLOSE, link_id);
		gps_each_link_set_bool_flag(link_id, LINK_TO_BE_CLOSED, true);
	}
}

void gps_dl_link_close_wait(enum gps_dl_link_id_enum link_id, long *p_sigval)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	enum GDL_RET_STATUS gdl_ret;
	long sigval;

	gdl_ret = gps_dl_link_wait_on(&p->waitables[GPS_DL_WAIT_OPEN_CLOSE], &sigval);
	if (gdl_ret == GDL_FAIL_SIGNALED) {
		if (p_sigval != NULL) {
			*p_sigval = sigval;
			return;
		}
	} else if (gdl_ret == GDL_FAIL_NOT_SUPPORT)
		; /* show warnning */
}

void gps_dl_link_close_ack(enum gps_dl_link_id_enum link_id, bool hw_suspend)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);

	GDL_LOGXD_ONF(link_id, "hw_suspend = %d", hw_suspend);

	gps_each_link_take_big_lock(link_id, GDL_LOCK_FOR_CLOSE_DONE);
	if (hw_suspend)
		gps_each_link_change_state_from(link_id, LINK_SUSPENDING, LINK_SUSPENDED);
	else
		gps_each_link_change_state_from(link_id, LINK_CLOSING, LINK_CLOSED);

	gps_dl_link_wake_up(&p->waitables[GPS_DL_WAIT_OPEN_CLOSE]);
	gps_each_link_give_big_lock(link_id);
}

static bool gps_dl_link_try_to_clear_both_resetting_status(void)
{
	enum gps_dl_link_id_enum link_id;
	struct gps_each_link *p = NULL;

	for (link_id = 0; link_id < GPS_DATA_LINK_NUM; link_id++) {
		p = gps_dl_link_get(link_id);
		if (p->reset_level != GPS_DL_RESET_LEVEL_NONE)
			return false;
	}

	for (link_id = 0; link_id < GPS_DATA_LINK_NUM; link_id++) {
		p = gps_dl_link_get(link_id);

		if (p->sub_states.user_open)
			p->state_for_user = LINK_RESET_DONE;
		else
			p->state_for_user = LINK_CLOSED;
	}

	return true;
}

static void gps_dl_link_reset_ack_inner(enum gps_dl_link_id_enum link_id, bool post_conn_reset)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	enum gps_each_link_state_enum old_state, new_state;
	enum gps_each_link_reset_level old_level, new_level;
	bool user_still_open = false;
	bool both_clear_done = false;
	bool try_conn_infra_off = false;

	gps_each_link_take_big_lock(link_id, GDL_LOCK_FOR_RESET_DONE);
	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	old_state = p->state_for_user;
	old_level = p->reset_level;
	user_still_open = p->sub_states.user_open;

	switch (old_level) {
	case GPS_DL_RESET_LEVEL_GPS_SINGLE_LINK:
		p->reset_level = GPS_DL_RESET_LEVEL_NONE;
		if (p->sub_states.user_open)
			p->state_for_user = LINK_RESET_DONE;
		else
			p->state_for_user = LINK_CLOSED;
		break;

	case GPS_DL_RESET_LEVEL_CONNSYS:
		if (!post_conn_reset)
			break;
		p->reset_level = GPS_DL_RESET_LEVEL_NONE;
		both_clear_done = gps_dl_link_try_to_clear_both_resetting_status();
		try_conn_infra_off = true;
		break;

	case GPS_DL_RESET_LEVEL_GPS_SUBSYS:
		p->reset_level = GPS_DL_RESET_LEVEL_NONE;
		both_clear_done = gps_dl_link_try_to_clear_both_resetting_status();
		break;

	default:
		break;
	}

	new_state = p->state_for_user;
	new_level = p->reset_level;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	if (try_conn_infra_off) {
		/* During connsys resetting, conninfra_pwr_off may fail,
		 * it need to be called again when connsys reset done.
		 */
		gps_dl_hal_conn_infra_driver_off();
	}

	/* TODO: if both clear, show another link's log */
	GDL_LOGXE_STA(link_id,
		"state change: %s -> %s, level: %d -> %d, user = %d, post_reset = %d, both_clear = %d",
		gps_dl_link_state_name(old_state), gps_dl_link_state_name(new_state),
		old_level, new_level,
		user_still_open, post_conn_reset, both_clear_done);

	gps_each_link_give_big_lock(link_id);

	/* Note: for CONNSYS or GPS_SUBSYS RESET, here might be still RESETTING,
	 * if any other link not reset done (see both_clear_done print).
	 */
	gps_dl_link_wake_up(&p->waitables[GPS_DL_WAIT_RESET]);
}

void gps_dl_link_reset_ack(enum gps_dl_link_id_enum link_id)
{
	gps_dl_link_reset_ack_inner(link_id, false);
}

void gps_dl_link_on_post_conn_reset(enum gps_dl_link_id_enum link_id)
{
	gps_dl_link_reset_ack_inner(link_id, true);
}

