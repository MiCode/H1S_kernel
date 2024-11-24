/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "gps_dl_config.h"
#ifdef GPS_DL_ENABLE_MET
#include "../../hw/gps_dl_hw_priv_util.h"
#include "gps_dl_linux_reserved_mem.h"
#include "gps/bgf_gps_cfg_on.h"

void gps_dl_hw_dep_set_emi_write_range(unsigned int bus_emi_met_phy_addr)
{
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_START_ADDR_GPS_MET_START_ADDR, bus_emi_met_phy_addr);
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_END_ADDR_GPS_MET_END_ADDR,
		(bus_emi_met_phy_addr+GPS_MET_MEM_SIZE));
}

void gps_dl_hw_dep_set_ringbuffer_mode(unsigned int mode)
{
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_CTL1_GPS_MET_RING_BUF_MODE, mode);
}

void gps_dl_hw_dep_set_sampling_rate(unsigned int rate)
{
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_CTL0_GPS_MET_OSC_CNT_TARGET, rate);
}

void gps_dl_hw_dep_set_mask_signal(unsigned int mask_signal)
{
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_CTL0_GPS_MET_DATA_MASK, mask_signal);
}

void gps_dl_hw_dep_set_edge_detection(unsigned int edge)
{
	unsigned int edge_msb, edge_lsb;

	/*edge_msb:16 MSB */
	edge_msb = edge>>16;
	/*edge_lsb:16 LSB */
	edge_lsb = edge&0xFFFF;
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_DEGE_DET_GPS_MET_NEGEDGE_DET_EN, edge_lsb);
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_DEGE_DET_GPS_MET_POSEDGE_DET_EN, edge_msb);
}

void gps_dl_hw_dep_set_event_signal(unsigned int event_signal)
{
	unsigned int event_signal_msb, event_signal_lsb;

	/*event_signal_msb:16 MSB */
	event_signal_msb = event_signal>>16;
	/*event_signal_lsb:16 LSB */
	event_signal_lsb = event_signal&0xFFFF;
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_SIGNAL_SEL_GPS_MET_SIGNAL_SEL, event_signal_lsb);
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_SIGNAL_SEL_GPS_MET_TEMP_SIGNAL_SEL, event_signal_msb);
}

void gps_dl_hw_dep_set_event_select(unsigned int event_select)
{
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_EVENT_SEL0_GPS_MET_GPS_EVENT_SEL, event_select);
}

void gps_dl_hw_dep_enable_met(void)
{
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_CTL0_GPS_MET_CLK_EN, 1);
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_CTL0_GPS_MET_RECORD_EN, 1);
}

void gps_dl_hw_dep_disable_met(void)
{
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_CTL0_GPS_MET_CLK_EN, 0);
	GDL_HW_SET_GPS_ENTRY(BGF_GPS_CFG_ON_GPS_ON_MET_CTL0_GPS_MET_RECORD_EN, 0);
}

unsigned int gps_dl_hw_dep_get_met_read_ptr_addr(void)
{
	return gps_bus_to_host(BGF_GPS_CFG_ON_GPS_ON_MET_READ_PTR_ADDR);
}

unsigned int gps_dl_hw_dep_get_met_write_ptr_addr(void)
{
	return gps_bus_to_host(BGF_GPS_CFG_ON_GPS_ON_MET_WRITE_PTR_ADDR);
}

#endif /* GPS_DL_ENABLE_MET */

