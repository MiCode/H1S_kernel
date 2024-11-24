/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "gps_dl_config.h"
#include "gps_each_device.h"
#include "gps_each_link.h"
#include "gps_dl_osal.h"
#include "gps_dl_hist_rec2.h"
#include "gps_dl_time_tick.h"
#include "../gps_dl_hw_priv_util.h"
#include "gps/bgf_gps_dma.h"
#include "gps_dsp_fsm.h"

#define GDL_COUNT_REC_DATA(w_new, w_old, l)\
	((w_new >= w_old) ? (w_new - w_old) : (l + w_new - w_old))

struct gps_dl_hist_rec2_tick_item {
		unsigned long time_us;
		unsigned int full_bag_item;
		unsigned int transmit_item;
		unsigned int transmit_len_item;
	};

#define GPS_DL_HIST_REC_RW_LOG_ITEM_MAX (8)
struct gps_dl_hist_rec2_log_list {
		struct gps_dl_hist_rec2_tick_item  tick_items[GPS_DL_HIST_REC_RW_LOG_ITEM_MAX];
		unsigned int full_bag_rec_idx;
		unsigned int tick_item_idx;
		unsigned int tick_item_rec;
		unsigned int last_write_index;
	};


struct gps_dl_hist_rec2_log_list g_gps_dl_hist_rec_log_list[GPS_DATA_LINK_NUM];

struct gps_dl_hist_rec2_log_list *gps_dl_hist_rec2_log_list_get(enum gps_dl_link_id_enum link_id)
{
	if (link_id >= 0 && link_id < GPS_DATA_LINK_NUM)
		return &g_gps_dl_hist_rec_log_list[link_id];

	return NULL;
}


bool gps_dl_hist_rec2_data_routing_status;
void gps_dl_hist_rec2_enable_data_routing(void)
{
	gps_dl_hist_rec2_data_routing_status = true;
}

bool gps_dl_hist_rec2_get_data_routing_status(void)
{
	return gps_dl_hist_rec2_data_routing_status;
}

void gps_dl_hist_rec2_disable_data_routing(void)
{
	gps_dl_hist_rec2_data_routing_status = false;
}

void gps_dl_hist_rec2_data_routing(enum gps_dl_link_id_enum link_id,
	enum gps_dl_hist_rec2_status rec2_status)
{
	struct gps_dl_hist_rec2_tick_item *p_item;
	struct gps_dl_hist_rec2_log_list *p_list;
	struct gps_each_link *p_link = gps_dl_link_get(link_id);
	unsigned long start_us = 0, duration_us = 0;
	unsigned int transmit_len = 0;

	p_list = gps_dl_hist_rec2_log_list_get(link_id);

	if (p_list == NULL)
		return;

	if (DATA_TRANS_START == rec2_status) {
		p_list->tick_items[p_list->tick_item_idx].time_us = gps_dl_tick_get_us();
	} else if (DATA_TRANS_CONTINUE == rec2_status) {
		p_list->full_bag_rec_idx++;
	} else if (DATA_TRANS_END == rec2_status) {
		if (true == gps_dl_hist_rec2_get_data_routing_status()) {
			p_item = &p_list->tick_items[p_list->tick_item_idx];
			start_us = p_item->time_us;
			p_item->time_us = gps_dl_tick_get_us();
			p_item->full_bag_item = p_list->full_bag_rec_idx;
			p_item->transmit_item = p_list->tick_item_rec;

			p_item->transmit_len_item = GDL_COUNT_REC_DATA(p_link->rx_dma_buf.dma_working_entry.write_index,
				p_list->last_write_index,
				p_link->rx_dma_buf.dma_working_entry.buf_length);

			transmit_len = p_item->transmit_len_item;
			p_item->transmit_len_item -= p_item->full_bag_item * GPS_LIBMNL_READ_MAX;
			if (p_item->transmit_len_item == GPS_LIBMNL_READ_MAX) {
				p_item->full_bag_item++;
				p_item->transmit_len_item -= GPS_LIBMNL_READ_MAX;
			}
			p_list->last_write_index = p_link->rx_dma_buf.dma_working_entry.write_index;
			duration_us = p_item->time_us - start_us;

			if (GPS_DSP_ST_WORKING == gps_dsp_state_get(link_id)) {
				GDL_LOGXW_DRW(link_id, "i = %u, %u(512 * %u + %u), d_us = %lu",
					p_list->tick_items[p_list->tick_item_idx].transmit_item,
					transmit_len,
					p_list->tick_items[p_list->tick_item_idx].full_bag_item,
					p_list->tick_items[p_list->tick_item_idx].transmit_len_item,
					duration_us);
			}

			p_list->full_bag_rec_idx = 0;
			p_list->tick_item_idx++;
			p_list->tick_item_rec++;

			if (p_list->tick_item_idx == GPS_DL_HIST_REC_RW_LOG_ITEM_MAX) {
				p_list->tick_item_idx = 0;
				memset(&p_list->tick_items, 0, sizeof(p_list->tick_items));
			}
		} else {
				p_list->last_write_index = p_link->rx_dma_buf.dma_working_entry.write_index;
				p_list->full_bag_rec_idx = 0;
				memset(&p_list->tick_items, 0, sizeof(p_list->tick_items));
			}

	}

}


