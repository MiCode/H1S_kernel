/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#ifndef _GPS_DL_HW_DEP_API_H
#define _GPS_DL_HW_DEP_API_H

#include "gps_dl_config.h"
#include "gps_dsp_fsm.h"

/*
 * BGF
 */
void gps_dl_hw_dep_gps_sw_request_peri_usage(bool request);
bool gps_dl_hw_dep_en_gps_func_and_poll_bgf_ack(void);
bool gps_dl_hw_dep_poll_bgf_bus_and_gps_top_ack(void);
void gps_dl_hw_dep_may_set_bus_debug_flag(void);
bool gps_dl_hw_dep_may_enable_bpll(void);
void gps_dl_hw_dep_may_disable_bpll(void);
void gps_dl_hw_dep_may_remap_conn2ap_gps_peri(void);
bool gps_dl_hw_dep_may_check_conn_infra_restore_done(void);
void gps_dl_hw_dep_may_set_conn_infra_l1_request(bool request);

/*
 * GPS
 */
bool gps_dl_hw_dep_set_dsp_on_and_poll_ack(enum gps_dl_link_id_enum link_id);
void gps_dl_hw_dep_set_dsp_off(enum gps_dl_link_id_enum link_id);
void gps_dl_hw_dep_cfg_dsp_mem(enum dsp_ctrl_enum ctrl);

/*
 * Debug
 */
void gps_dl_hw_dep_dump_gps_pos_info(enum gps_dl_link_id_enum link_id);
void gps_dl_hw_dep_dump_host_csr_gps_info(void);
void gps_dl_hw_dep_dump_host_csr_conninfra_info(void);

/* Only need when BMASK_RW_DO_CHECK active for debug purpose */
void gps_dl_hw_dep_may_do_bus_check_and_print(unsigned int host_addr);
void gps_dl_hw_dep_dump_gps_rf_temp_cr(void);

/*
 * MET2.0
 */
void gps_dl_hw_dep_set_emi_write_range(unsigned int bus_emi_met_phy_addr);
void gps_dl_hw_dep_set_ringbuffer_mode(unsigned int mode);
void gps_dl_hw_dep_set_sampling_rate(unsigned int rate);
void gps_dl_hw_dep_set_mask_signal(unsigned int mask_signal);
void gps_dl_hw_dep_set_edge_detection(unsigned int edge);
void gps_dl_hw_dep_set_event_signal(unsigned int event_signal);
void gps_dl_hw_dep_set_event_select(unsigned int event_select);
void gps_dl_hw_dep_enable_met(void);
void gps_dl_hw_dep_disable_met(void);
unsigned int gps_dl_hw_dep_get_met_read_ptr_addr(void);
unsigned int gps_dl_hw_dep_get_met_write_ptr_addr(void);
#endif

