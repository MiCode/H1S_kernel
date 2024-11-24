/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"

#include "gps_dl_context.h"
#include "gps_dl_hw_dep_api.h"
#include "gps_dl_hw_dep_macro.h"
#include "gps_dl_hw_priv_util.h"
#include "gps_dl_hw_semaphore.h"


#if GPS_DL_USE_BGF_SEL_SEMA
bool gps_dl_hw_take_conn_bgf_sel_hw_sema(unsigned int try_timeout_ms)
{
	bool okay;
	bool show_log;
	unsigned int poll_us, poll_max_us;
	unsigned int val;

	show_log = gps_dl_set_show_reg_rw_log(true);
	/* poll until value is expected or timeout */
	poll_us = 0;
	poll_max_us = POLL_US * 1000 * try_timeout_ms;
	okay = false;
	while (!okay) {
		val = GDL_HW_GET_CONN_INFRA_ENTRY(COS_SEMA_BGF_SEL_STA_ENTRY_FOR_GPS);
		/* 2bit value:
		 * 0 -> need waiting
		 * 1,3 -> okay; 2 -> already taken
		 */
		if (val != 0) {
			okay = true;
			break;
		}
		if (poll_us >= poll_max_us) {
			okay = false;
			break;
		}
		gps_dl_wait_us(POLL_INTERVAL_US);
		poll_us += POLL_INTERVAL_US;
	}
	gps_dl_set_show_reg_rw_log(show_log);

	if (!okay)
		GDL_LOGW("okay = %d", okay);
	else
		GDL_LOGD("okay = %d", okay);
	return okay;
}

void gps_dl_hw_give_conn_bgf_sel_hw_sema(void)
{
	bool show_log;

	show_log = gps_dl_set_show_reg_rw_log(true);
	GDL_HW_SET_CONN_INFRA_ENTRY(COS_SEMA_BGF_SEL_REL_ENTRY_FOR_GPS, 1);
	gps_dl_set_show_reg_rw_log(show_log);

	GDL_LOGD("");
}
#endif /* GPS_DL_USE_BGF_SEL_SEMA */

#if GPS_DL_HAS_PTA
bool gps_dl_hw_take_conn_coex_hw_sema(unsigned int try_timeout_ms)
{
	bool okay;
	bool show_log;
	unsigned int poll_us, poll_max_us;
	unsigned int val;

	show_log = gps_dl_set_show_reg_rw_log(true);
	/* poll until value is expected or timeout */
	poll_us = 0;
	poll_max_us = POLL_US * 1000 * try_timeout_ms;
	okay = false;
	while (!okay) {
		val = GDL_HW_GET_CONN_INFRA_ENTRY(COS_SEMA_COEX_STA_ENTRY_FOR_GPS);
		/* 2bit value:
		 * 0 -> need waiting
		 * 1,3 -> okay; 2 -> already taken
		 */
		if (val != 0) {
			okay = true;
			break;
		}
		if (poll_us >= poll_max_us) {
			okay = false;
			break;
		}
		gps_dl_wait_us(POLL_INTERVAL_US);
		poll_us += POLL_INTERVAL_US;
	}
	gps_dl_set_show_reg_rw_log(show_log);

	if (!okay)
		GDL_LOGW("okay = %d", okay);
	else
		GDL_LOGD("okay = %d", okay);
	return okay;
}

void gps_dl_hw_give_conn_coex_hw_sema(void)
{
	bool show_log;

	show_log = gps_dl_set_show_reg_rw_log(true);
	GDL_HW_SET_CONN_INFRA_ENTRY(COS_SEMA_COEX_REL_ENTRY_FOR_GPS, 1);
	gps_dl_set_show_reg_rw_log(show_log);

	GDL_LOGD("");
}
#endif /* GPS_DL_HAS_PTA */

bool gps_dl_hw_take_conn_rfspi_hw_sema(unsigned int try_timeout_ms)
{
	bool okay;
	bool show_log;
	unsigned int poll_us, poll_max_us;
	unsigned int val;

	show_log = gps_dl_set_show_reg_rw_log(true);
	/* poll until value is expected or timeout */
	poll_us = 0;
	poll_max_us = POLL_US * 1000 * try_timeout_ms;
	okay = false;
	while (!okay) {
		val = GDL_HW_GET_CONN_INFRA_ENTRY(COS_SEMA_RFSPI_STA_ENTRY_FOR_GPS);
		/* 2bit value:
		 * 0 -> need waiting
		 * 1,3 -> okay; 2 -> already taken
		 */
		if (val != 0) {
			okay = true;
			break;
		}
		if (poll_us >= poll_max_us) {
			okay = false;
			break;
		}
		gps_dl_wait_us(POLL_INTERVAL_US);
		poll_us += POLL_INTERVAL_US;
	}
	gps_dl_set_show_reg_rw_log(show_log);

	GDL_LOGI("okay = %d", okay);

	return okay;
}

void gps_dl_hw_give_conn_rfspi_hw_sema(void)
{
	bool show_log;

	show_log = gps_dl_set_show_reg_rw_log(true);
	GDL_HW_SET_CONN_INFRA_ENTRY(COS_SEMA_RFSPI_REL_ENTRY_FOR_GPS, 1);
	gps_dl_set_show_reg_rw_log(show_log);

	GDL_LOGI("");
}

