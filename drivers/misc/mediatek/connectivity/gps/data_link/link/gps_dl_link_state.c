/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"
#include "gps_dl_time_tick.h"

#include "gps_each_link.h"
#include "gps_dl_lib_misc.h"
#include "gps_dl_name_list.h"
#include "gps_dl_context.h"

void gps_each_link_set_bool_flag(enum gps_dl_link_id_enum link_id,
	enum gps_each_link_bool_state name, bool value)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);

	if (!p)
		return;

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	switch (name) {
	case LINK_TO_BE_CLOSED:
		p->sub_states.to_be_closed = value;
		break;
	case LINK_USER_OPEN:
		p->sub_states.user_open = value;
		break;
	case LINK_OPEN_RESULT_OKAY:
		p->sub_states.open_result_okay = value;
		break;
	case LINK_NEED_A2Z_DUMP:
		p->sub_states.need_a2z_dump = value;
		break;
	case LINK_SUSPEND_TO_CLK_EXT:
		p->sub_states.suspend_to_clk_ext = value;
		break;
	default:
		break; /* do nothing */
	}
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
}

bool gps_each_link_get_bool_flag(enum gps_dl_link_id_enum link_id,
	enum gps_each_link_bool_state name)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	bool value = false;

	if (!p)
		return false;

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	switch (name) {
	case LINK_TO_BE_CLOSED:
		value = p->sub_states.to_be_closed;
		break;
	case LINK_USER_OPEN:
		value = p->sub_states.user_open;
		break;
	case LINK_OPEN_RESULT_OKAY:
		value = p->sub_states.open_result_okay;
		break;
	case LINK_NEED_A2Z_DUMP:
		value = p->sub_states.need_a2z_dump;
		break;
	case LINK_SUSPEND_TO_CLK_EXT:
		value = p->sub_states.suspend_to_clk_ext;
		break;
	default:
		break; /* TODO: warning it */
	}
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	return value;
}

void gps_dl_link_set_ready_to_write(enum gps_dl_link_id_enum link_id, bool is_ready)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	if (p)
		p->sub_states.is_ready_to_write = is_ready;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
}

bool gps_dl_link_is_ready_to_write(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	bool ready = false;

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	if (p)
		ready = p->sub_states.is_ready_to_write;
	else
		ready = false;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	return ready;
}

void gps_each_link_set_active(enum gps_dl_link_id_enum link_id, bool is_active)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	if (p)
		p->sub_states.is_active = is_active;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
}

bool gps_each_link_is_active(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	bool ready = false;

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	if (p)
		ready = p->sub_states.is_active;
	else
		ready = false;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	return ready;
}

void gps_each_link_inc_session_id(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	int sid;

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	if (p->session_id >= GPS_EACH_LINK_SID_MAX)
		p->session_id = 1;
	else
		p->session_id++;
	sid = p->session_id;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	GDL_LOGXD(link_id, "new sid = %d, 1byte_mode = %d", sid, gps_dl_is_1byte_mode());
}

int gps_each_link_get_session_id(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	int sid;

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	sid = p->session_id;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	return sid;
}

enum gps_each_link_state_enum gps_each_link_get_state(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	enum gps_each_link_state_enum state;

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	state = p->state_for_user;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	return state;
}

void gps_each_link_set_state(enum gps_dl_link_id_enum link_id, enum gps_each_link_state_enum state)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	enum gps_each_link_state_enum pre_state;


	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	pre_state = p->state_for_user;
	p->state_for_user = state;
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	GDL_LOGXI_STA(link_id, "state change: %s -> %s",
		gps_dl_link_state_name(pre_state), gps_dl_link_state_name(state));
}

bool gps_each_link_change_state_from(enum gps_dl_link_id_enum link_id,
	enum gps_each_link_state_enum from, enum gps_each_link_state_enum to)
{
	bool is_okay = false;
	struct gps_each_link *p = gps_dl_link_get(link_id);
	enum gps_each_link_state_enum pre_state;
	enum gps_each_link_reset_level old_level, new_level;

	gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);
	pre_state = p->state_for_user;
	if (from == pre_state) {
		p->state_for_user = to;
		is_okay = true;

		if (to == LINK_RESETTING) {
			old_level = p->reset_level;
			if (old_level < GPS_DL_RESET_LEVEL_GPS_SINGLE_LINK)
				p->reset_level = GPS_DL_RESET_LEVEL_GPS_SINGLE_LINK;
			new_level = p->reset_level;
		}
	}
	gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_LINK_STATE);

	if (is_okay && (to == LINK_RESETTING)) {
		GDL_LOGXI_STA(link_id, "state change: %s -> %s, okay, level: %d -> %d",
			gps_dl_link_state_name(from), gps_dl_link_state_name(to),
			old_level, new_level);
	} else if (is_okay) {
		GDL_LOGXI_STA(link_id, "state change: %s -> %s, okay",
			gps_dl_link_state_name(from), gps_dl_link_state_name(to));
	} else {
		GDL_LOGXW_STA(link_id, "state change: %s -> %s, fail on pre_state = %s",
			gps_dl_link_state_name(from), gps_dl_link_state_name(to),
			gps_dl_link_state_name(pre_state));
	}

	return is_okay;
}

