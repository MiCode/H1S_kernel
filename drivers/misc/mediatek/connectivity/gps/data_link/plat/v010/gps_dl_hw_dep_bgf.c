/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include "gps_dl_config.h"
#include "gps_dl_hw_ver.h"
#include "gps_dl_hw_dep_api.h"
#include "../gps_dl_hw_priv_util.h"
#include "gps_dl_hw_dep_macro.h"
#include "gps_dl_hw_api.h"

#if GPS_DL_HAS_CONNINFRA_DRV
#if GPS_DL_ON_LINUX
#include "conninfra.h"
#elif GPS_DL_ON_CTP
#include "conninfra_ext.h"
#endif
#endif

void gps_dl_hw_dep_gps_sw_request_peri_usage(bool request)
{
	/* Do nothing for MT6893 or MT6885 */
}

bool gps_dl_hw_dep_en_gps_func_and_poll_bgf_ack(void)
{
	bool poll_okay = false;

	/* Enable GPS function */
	GDL_HW_SET_GPS_FUNC_EN(1);

	/* bit24: BGFSYS_ON_TOP primary power ack */
	GDL_HW_POLL_CONN_INFRA_ENTRY(CONN_INFRA_RGU_BGFSYS_ON_TOP_PWR_ACK_ST_BGFSYS_ON_TOP_PWR_ACK, 1,
		POLL_DEFAULT, &poll_okay);
	if (!poll_okay) {
		GDL_LOGE("_fail_bgf_top_1st_pwr_ack_not_okay");
		goto _fail_bgf_top_1st_pwr_ack_not_okay;
	}

	/* bit25: BGFSYS_ON_TOP secondary power ack */
	GDL_HW_POLL_CONN_INFRA_ENTRY(CONN_INFRA_RGU_BGFSYS_ON_TOP_PWR_ACK_ST_AN_BGFSYS_ON_TOP_PWR_ACK_S, 1,
		POLL_DEFAULT, &poll_okay);
	if (!poll_okay) {
		GDL_LOGE("_fail_bgf_top_2nd_pwr_ack_not_okay");
		goto _fail_bgf_top_2nd_pwr_ack_not_okay;
	}
	return true; /* okay */

_fail_bgf_top_1st_pwr_ack_not_okay:
_fail_bgf_top_2nd_pwr_ack_not_okay:
	return false; /* fail */
}


bool gps_dl_hw_dep_poll_bgf_bus_and_gps_top_ack(void)
{
	bool poll_okay = false;
	int i;

	/* 0x18060240[3:0] == 4h'2 gps_top_off is GPS_ACTIVE state */
	GDL_HW_SET_CONN_INFRA_ENTRY(CONN_HOST_CSR_TOP_HOST2GPS_DEGUG_SEL_HOST2GPS_DEGUG_SEL, 0x80);
	for (i = 0; i < 3; i++) {
		GDL_HW_POLL_CONN_INFRA_ENTRY(CONN_HOST_CSR_TOP_GPS_CFG2HOST_DEBUG_GPS_CFG2HOST_DEBUG, 2,
			POLL_DEFAULT, &poll_okay);
		if (poll_okay)
			break;
		/*
		 * TODO:
		 * if (!gps_dl_reset_level_is_none()) break;
		 */
		if (i > 0)
			GDL_LOGW("_poll_gps_top_off_active, cnt = %d", i + 1);
	}
	if (!poll_okay) {
		GDL_LOGE("_fail_gps_top_off_active_not_okay");
		goto _fail_gps_top_off_active_not_okay;
	}

	/* 0x18c21010[31:0] bgf ip version */
	GDL_HW_POLL_GPS_ENTRY(BGF_GPS_CFG_BGF_IP_VERSION_BGFSYS_VERSION,
		GDL_HW_BGF_VER_CONNAC2_0, POLL_DEFAULT, &poll_okay);
	if (!poll_okay) {
		GDL_LOGE("_fail_bgf_ip_ver_not_okay");
		goto _fail_bgf_ip_ver_not_okay;
	}

	/* 0x18c21014[7:0] bgf ip cfg */
	GDL_HW_POLL_GPS_ENTRY(BGF_GPS_CFG_BGF_IP_CONFIG_BGFSYS_CONFIG, 0,
		POLL_DEFAULT, &poll_okay);
	if (!poll_okay) {
		GDL_LOGE("_fail_bgf_ip_cfg_not_okay");
		goto _fail_bgf_ip_cfg_not_okay;
	}
	return true;

_fail_gps_top_off_active_not_okay:
_fail_bgf_ip_ver_not_okay:
_fail_bgf_ip_cfg_not_okay:
	return false;
}

bool gps_dl_hw_dep_may_enable_bpll(void)
{
	/*mt6885 mt6893 do nothing*/
	/*avoid return check, return true*/
	return true;
}


void gps_dl_hw_dep_may_disable_bpll(void)
{
	/*mt6885 mt6893 do nothing*/
}

void gps_dl_hw_dep_may_set_bus_debug_flag(void)
{
#if (GPS_DL_HAS_CONNINFRA_DRV)
	/* conninfra driver add an API to do bellow step
	 * after connac2_0, it's an empty function and no need to call it
	 */
	conninfra_config_setup();
#else
	/* Enable conninfra bus hung detection */
	GDL_HW_WR_CONN_INFRA_REG(0x1800F000, 0xF000001C);
#endif

	/* host csr gps bus debug mode enable 0x18c60000 = 0x10 */
	GDL_HW_WR_GPS_REG(0x80060000, 0x10);
}

void gps_dl_hw_dep_may_remap_conn2ap_gps_peri(void)
{
	/*mt6885 mt6893 do nothing*/
}

bool gps_dl_hw_dep_may_check_conn_infra_restore_done(void)
{
	/*mt6885 mt6893 do nothing*/
	return true;
}

void gps_dl_hw_dep_may_set_conn_infra_l1_request(bool request)
{
	/*mt6885 mt6893 do nothing*/
}
