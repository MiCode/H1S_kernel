/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"

#include "gps_each_link.h"
#include "gps_dl_name_list.h"
#include "gps_dl_hal.h"
#include "gps_dl_hw_api.h"

bool gps_dl_link_start_tx_dma_if_has_data(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p_link = gps_dl_link_get(link_id);
	struct gdl_dma_buf_entry dma_buf_entry;
	enum GDL_RET_STATUS gdl_ret;
	bool tx_dma_started = false;

	gdl_ret = gdl_dma_buf_get_data_entry(&p_link->tx_dma_buf, &dma_buf_entry);

	if (gdl_ret == GDL_OKAY) {
		/* wait until dsp recevie last data done or timeout(10ms)
		 * TODO: handle timeout case
		 */
		gps_dl_hw_poll_usrt_dsp_rx_empty(link_id);
		gps_dl_hal_a2d_tx_dma_claim_emi_usage(link_id, true);
		gps_dl_hal_a2d_tx_dma_start(link_id, &dma_buf_entry);
		tx_dma_started = true;
	} else {
		GDL_LOGD("gdl_dma_buf_get_data_entry ret = %s", gdl_ret_to_name(gdl_ret));
		tx_dma_started = false;
	}

	return tx_dma_started;
}

int gps_dl_link_get_clock_flag(void)
{
	return gps_dl_hal_get_clock_flag();
}

