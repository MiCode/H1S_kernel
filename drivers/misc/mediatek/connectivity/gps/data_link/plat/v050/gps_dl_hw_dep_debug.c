/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"

#include "gps_dl_hw_ver.h"
#include "gps_dl_hw_dep_api.h"
#include "gps_dl_hw_dep_macro.h"

#include "../gps_dl_hw_priv_util.h"
#include "conn_infra/conn_dbg_ctl.h"
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
		BGF_GPS_RGU_ON_GPS_L1_CR_RGU_GPS_L1_ON_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		BGF_GPS_RGU_ON_GPS_L1_CR_RGU_GPS_L1_SOFT_RST_B_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		BGF_GPS_CFG_ON_GPS_L1_SLP_PWR_CTL_GPS_L1_SLP_PWR_CTL_CS_ADDR,
		BMASK_RW_FORCE_PRINT);

	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		BGF_GPS_RGU_ON_GPS_L5_CR_RGU_GPS_L5_ON_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		BGF_GPS_RGU_ON_GPS_L5_CR_RGU_GPS_L5_SOFT_RST_B_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_GPS_BUS,
		BGF_GPS_CFG_ON_GPS_L5_SLP_PWR_CTL_GPS_L5_SLP_PWR_CTL_CS_ADDR,
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

	/*2021.5.7 confirm with DE, gps use 0x18023a04, 0x18023a00*/
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_DBG_CTL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_ADDR,
		BMASK_RW_FORCE_PRINT);
	gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
		CONN_DBG_CTL_BGF_MONFLAG_OFF_OUT_ADDR,
		BMASK_RW_FORCE_PRINT);

	for (i = 0xC0020301; i <= 0xC0020307; i = i + 0x1) {
		gps_dl_bus_wr_opt(GPS_DL_CONN_INFRA_BUS,
			CONN_DBG_CTL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_ADDR, i,
			BMASK_RW_FORCE_PRINT);
		gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
			CONN_DBG_CTL_BGF_MONFLAG_OFF_OUT_ADDR,
			BMASK_RW_FORCE_PRINT);
	}

	for (i = 0xC0020400; i <= 0xC0020406; i = i + 0x1) {
		gps_dl_bus_wr_opt(GPS_DL_CONN_INFRA_BUS,
			CONN_DBG_CTL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_ADDR, i,
			BMASK_RW_FORCE_PRINT);
		gps_dl_bus_rd_opt(GPS_DL_CONN_INFRA_BUS,
			CONN_DBG_CTL_BGF_MONFLAG_OFF_OUT_ADDR,
			BMASK_RW_FORCE_PRINT);
	}

}

#if 0
static void gps_dl_hw_dump_host_csr_conninfra_info_inner(unsigned int selection, int n)
{
	/* TODO */
}
#endif

void gps_dl_hw_dep_dump_host_csr_conninfra_info(void)
{
	/* TODO */
}

void gps_dl_hw_dep_may_do_bus_check_and_print(unsigned int host_addr)
{
	/* TODO */
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

