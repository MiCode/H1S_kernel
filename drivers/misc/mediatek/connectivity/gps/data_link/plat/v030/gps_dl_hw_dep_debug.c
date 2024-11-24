/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include "gps_dl_config.h"
#include "gps_dl_hw_ver.h"
#include "gps_dl_hw_dep_api.h"
#include "../gps_dl_hw_priv_util.h"
#include "gps_dl_hw_dep_macro.h"
#include "gps_dl_hw_semaphore.h"
#include "conn_infra/conn_vdnr_gen_on_debug_ctrl_ao.h"
#include "gps/gps_aon_top.h"
#if GPS_DL_HAS_CONNINFRA_DRV
#if GPS_DL_ON_LINUX
#include "conninfra.h"
#elif GPS_DL_ON_CTP
#include "conninfra_ext.h"
#endif
#endif

void gps_dl_hw_dep_dump_gps_pos_info(enum gps_dl_link_id_enum link_id)
{
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_RGU_ON_GPS_L1_CR_RGU_GPS_L1_ON_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_RGU_ON_GPS_L1_CR_RGU_GPS_L1_SOFT_RST_B_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_CFG_ON_GPS_L1_SLP_PWR_CTL_GPS_L1_SLP_PWR_CTL_CS_ADDR,
		BMASK_RW_FORCE_PRINT);

	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_RGU_ON_GPS_L5_CR_RGU_GPS_L5_ON_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_RGU_ON_GPS_L5_CR_RGU_GPS_L5_SOFT_RST_B_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_CFG_ON_GPS_L5_SLP_PWR_CTL_GPS_L5_SLP_PWR_CTL_CS_ADDR,
		BMASK_RW_FORCE_PRINT);

	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_AON_TOP_TCXO_MS_H_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_AON_TOP_TCXO_MS_L_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		GPS_AON_TOP_DSLEEP_CTL_ADDR,
		BMASK_RW_FORCE_PRINT);

}

void gps_dl_hw_dep_dump_host_csr_gps_info(void)
{
	int i;
	bool okay = false;

	okay = gps_dl_hw_take_conn_bgf_sel_hw_sema(100);
	if (!okay) {
		GDL_LOGE("gps_dl_hw_take_conn_bgf_sel_hw_sema fail");
		return;
	}

	/*2021.2.20 confirm with DE, gps use 0x180600ac[23:16], 0x1806023c[31:16]*/
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_HOST_CSR_TOP_HOST2BGF_DEGUG_SEL_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_HOST_CSR_TOP_BT_CFG2HOST_DEBUG_ADDR,
		BMASK_RW_FORCE_PRINT);

	for (i = 0xA20000; i <= 0xBA0000; i = i + 0x10000) {
		gps_dl_bus_wr_opt(GPS_DL_CONN_INFRA_BUS,
			CONN_HOST_CSR_TOP_HOST2BGF_DEGUG_SEL_ADDR, i,
			BMASK_RW_FORCE_PRINT);
		gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
			CONN_HOST_CSR_TOP_BT_CFG2HOST_DEBUG_ADDR,
			BMASK_RW_FORCE_PRINT);
	}

	gps_dl_hw_give_conn_bgf_sel_hw_sema();

}

static void gps_dl_hw_dump_host_csr_conninfra_info_inner(void)
{

	/* Due to RW_DO_CHECK might be enabled, not use
	 * GDL_HW_SET_CONN_INFRA_ENTRY and GDL_HW_GET_CONN_INFRA_ENTRY to avoid redundant print.
	 */
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT2_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT3_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT4_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT5_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT6_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT7_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT8_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT9_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT10_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT11_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT12_ADDR,
		BMASK_RW_FORCE_PRINT);

}

void gps_dl_hw_dep_dump_host_csr_conninfra_info(void)
{
	gps_dl_hw_dump_host_csr_conninfra_info_inner();
}

void gps_dl_hw_dep_may_do_bus_check_and_print(unsigned int host_addr)
{
	/* not do rw check because here is the checking */
	GDL_LOGI("for addr = 0x%08x", host_addr);

	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT10_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT11_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_VDNR_GEN_ON_DEBUG_CTRL_AO_CONN_INFRA_VDNR_GEN_ON_U_DEBUG_CTRL_AO_CONN_INFRA_ON_RESULT12_ADDR,
		BMASK_RW_FORCE_PRINT);
}

void gps_dl_hw_gps_dump_gps_rf_temp_cr(void)
{
#if GPS_DL_HAS_CONNINFRA_DRV
	unsigned int spi_data;
	int rd_status;

	/*GPS: read 0x500*/
	spi_data = 0;
	rd_status = conninfra_spi_read(SYS_SPI_GPS, 0x500, &spi_data);
	ASSERT_ZERO(rd_status, GDL_VOIDF());
	GDL_LOGW("normally dump spi_data(GPS:0x500) = 0x%x", spi_data);

	/*GPS: read 0x518*/
	spi_data = 0;
	rd_status = conninfra_spi_read(SYS_SPI_GPS, 0x518, &spi_data);
	ASSERT_ZERO(rd_status, GDL_VOIDF());
	GDL_LOGW("normally dump spi_data(GPS:0x518) = 0x%x", spi_data);
#else
	GDL_LOGE("no conninfra driver");
#endif
}

