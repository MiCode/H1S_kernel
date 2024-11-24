/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */


#include "gps_dl_config.h"
#ifdef GPS_DL_ENABLE_MET
#include "gps_dl_hal_met2_0.h"
#include "gps_dl_hw_dep_api.h"
#include "metlog.h"
#include "gps_dl_log.h"
#include "gps_dl_linux_plat_drv.h"
#include "gps_dl_linux_reserved_mem.h"
#include "gps_dl_hal_api.h"
#include "gps_each_link.h"
#include "gps_dl_name_list.h"
#include "conninfra.h"


struct gps_debug_met_contex g_gps_debug_met_contex;
static struct gps_debug_met_settings gps_debug_met_default_settings =  {
	/*ringbuffer mode = on*/
	.is_ringbuffer_mode = 1,
	/*not mast any siganl*/
	.mask_signal = 0,
	/*sample rate = 1M*/
	.sample_rate = 25,
	/*default L1, channel 1*/
	.event_select = 0x1,
	/*gps met debug message*/
	.event_signal = 0x33333210,
	/*detection*/
	.edge_detection = 0xffffffff,
};

int gps_debug_met_start(struct gps_debug_met_contex *contex)
{
	unsigned int p_mem_phy, p_mem_met_phy, bus_emi_met_phy_addr;
	unsigned int value;
	enum gps_each_link_state_enum state;
	struct conn_metlog_info metlog_info;
	int ret;

	/*1. check if we can start GPS MET*/
	/* 1.1 Check if GPS is opened*/
	state = gps_each_link_get_state(GPS_DATA_LINK_ID0);
	if (state != LINK_OPENED) {
		GDL_LOGE("gps state:%s is not opend, set MET fail\n", gps_dl_link_state_name(state));
		return -1;
	}

	/* 1.2 Check if MET is closed*/
	if (contex->status != GPS_DEBUG_MET_CLOSED) {
		GDL_LOGE("GPS MET start fail, MET Status = %d", contex->status);
		return -1;
	}

	/* 1.3 Check if EMI is correctly mapped*/
	p_mem_phy = g_gps_dl_res_emi.host_phys_addr;
	if (p_mem_phy == 0) {
		GDL_LOGE("MET get EMI phy mem fail\n");
		return -1;
	}
	p_mem_met_phy = p_mem_phy+offsetof(struct gps_dl_reserved_mem_layout, met_buf);

	if (gps_dl_emi_remap_phy_to_bus_addr(p_mem_met_phy, &bus_emi_met_phy_addr) == GDL_FAIL) {
		GDL_LOGE("MET remap EMI phy to bus addr fail\n");
		return -1;
	}

	/*2. Set parameters for EMI CRs and notify connifra*/
	/* 2.1 Set EMI Writing range*/
	gps_dl_hw_dep_set_emi_write_range(bus_emi_met_phy_addr);

	/* 2.2 Set ring buffer mode*/
	value = (contex->setting_bitmap&GPS_DEBUG_MET_SETTINGS_BUFFER_MODE_VALID ?
		contex->settings.is_ringbuffer_mode : gps_debug_met_default_settings.is_ringbuffer_mode);
	gps_dl_hw_dep_set_ringbuffer_mode(value);

	/* 2.3 Set sampling rate*/
	value = (contex->setting_bitmap&GPS_DEBUG_MET_SETTINGS_SAMPLE_RATE_VALID ?
		contex->settings.sample_rate : gps_debug_met_default_settings.sample_rate);
	gps_dl_hw_dep_set_sampling_rate(value);

	/* 2.4 Set mask signal*/
	value = (contex->setting_bitmap&GPS_DEBUG_MET_SETTINGS_MASK_SIGNAL_VALID ?
		contex->settings.mask_signal : gps_debug_met_default_settings.mask_signal);
	gps_dl_hw_dep_set_mask_signal(value);

	/* 2.5 Set edge detection*/
	value = (contex->setting_bitmap&GPS_DEBUG_MET_SETTINGS_EADE_DETECTION_VALID ?
		contex->settings.edge_detection : gps_debug_met_default_settings.edge_detection);
	gps_dl_hw_dep_set_edge_detection(value);

	/* 2.6 Select event signal Level 1*/
	value = (contex->setting_bitmap&GPS_DEBUG_MET_SETTINGS_EVENT_SIGNAL_VALID ?
		contex->settings.event_signal : gps_debug_met_default_settings.event_signal);
	gps_dl_hw_dep_set_event_signal(value);

	/*2.7 Event select Level 2*/
	value = (contex->setting_bitmap&GPS_DEBUG_MET_SETTINGS_EVENT_SELECT_VALID ?
		contex->settings.event_select : gps_debug_met_default_settings.event_select);
	gps_dl_hw_dep_set_event_select(value);

	/* 2.8 Enable MET*/
	gps_dl_hw_dep_enable_met();

	/* 2.9 Notify connifra and create MET thread*/
	metlog_info.type = CONNDRV_TYPE_GPS;
	metlog_info.read_cr = gps_dl_hw_dep_get_met_read_ptr_addr();
	metlog_info.write_cr = gps_dl_hw_dep_get_met_write_ptr_addr();
	metlog_info.met_base_ap = p_mem_met_phy;
	metlog_info.met_base_fw = bus_emi_met_phy_addr;
	metlog_info.met_size = GPS_MET_MEM_SIZE;
	metlog_info.output_len = GPS_DEBUG_MET_OUTPUT_LEN;

	ret = conn_metlog_start(&metlog_info);
	if (ret != 0)
		GDL_LOGE("Start MET notify conifra fail, error=%d\n", ret);
	else
		contex->status = GPS_DEBUG_MET_OPENED;

	/* 2.10 Clear all the settings for this session*/
	gps_debug_met_clear(contex);

	/*Dump setting for debug MET function*/
	GDL_LOGW("Start MET,");
	GDL_LOGW("bitmap :0x%.8X\n", contex->setting_bitmap);
	GDL_LOGW("buffer mode :0x%.2X\n", contex->settings.is_ringbuffer_mode);
	GDL_LOGW("sample rate :0x%.2X\n", contex->settings.sample_rate);
	GDL_LOGW("event select :0x%.2X\n", contex->settings.event_select);
	GDL_LOGW("mask signal :0x%.8X\n", contex->settings.mask_signal);
	GDL_LOGW("event signal :0x%.8X\n", contex->settings.event_signal);
	GDL_LOGW("edge detection :0x%.8X\n", contex->settings.edge_detection);

	return ret;
}

int gps_debug_met_stop(struct gps_debug_met_contex *contex)
{
	int ret;

	if (contex->status != GPS_DEBUG_MET_OPENED) {
		GDL_LOGE("GPS MET stop fail, MET Status = %d", contex->status);
		return -1;
	}
	gps_dl_hw_dep_disable_met();

	/*notify connifra and stop MET thread*/
	ret = conn_metlog_stop(CONNDRV_TYPE_GPS);
	if (ret != 0)
		GDL_LOGE("Stop MET notify conifra fail, error=%d\n", ret);
	else
		contex->status = GPS_DEBUG_MET_CLOSED;
	return ret;
}

/*clear all the settings*/
void gps_debug_met_clear(struct gps_debug_met_contex *contex)
{
	contex->setting_bitmap = 0;
}

int gps_debug_met_set_parameter(struct gps_debug_met_contex *contex,
	enum gps_debug_met_operator_index op, unsigned int value)
{
	int ret = 0;

	switch (op) {
	case GPS_DEBUG_OP_SET_BUFFER_MODE:
		contex->settings.is_ringbuffer_mode = value;
		contex->setting_bitmap |=  GPS_DEBUG_MET_SETTINGS_BUFFER_MODE_VALID;
		break;
	case GPS_DEBUG_OP_SET_SAMPLE_RATE:
		contex->settings.sample_rate = value;
		contex->setting_bitmap |=  GPS_DEBUG_MET_SETTINGS_SAMPLE_RATE_VALID;
		break;
	case GPS_DEBUG_OP_SET_MASK_SIGNAL:
		contex->settings.mask_signal = value;
		contex->setting_bitmap |=  GPS_DEBUG_MET_SETTINGS_MASK_SIGNAL_VALID;
		break;
	case GPS_DEBUG_OP_SET_EVENT_SIGNAL:
		contex->settings.event_signal = value;
		contex->setting_bitmap |=  GPS_DEBUG_MET_SETTINGS_EVENT_SIGNAL_VALID;
		break;
	case GPS_DEBUG_OP_SET_EDGE_DETECTION:
		contex->settings.edge_detection = value;
		contex->setting_bitmap |=  GPS_DEBUG_MET_SETTINGS_EADE_DETECTION_VALID;
		break;
	case GPS_DEBUG_OP_SET_EVENT_SELECT:
		contex->settings.event_select = value;
		contex->setting_bitmap |=  GPS_DEBUG_MET_SETTINGS_EVENT_SELECT_VALID;
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

#endif /* GPS_DL_ENABLE_MET */

