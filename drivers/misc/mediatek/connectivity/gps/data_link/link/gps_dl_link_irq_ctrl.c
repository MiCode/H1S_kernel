/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"

#include "gps_each_link.h"
#include "gps_dl_isr.h"
#include "gps_dl_hal.h"


void gps_dl_link_irq_set(enum gps_dl_link_id_enum link_id, bool enable)
{
	struct gps_each_link *p_link = gps_dl_link_get(link_id);
	bool dma_working = false;
	bool pending_rx = false;
	bool bypass_unmask_irq = false;

	if (enable) {
		gps_dl_irq_each_link_unmask(link_id, GPS_DL_IRQ_TYPE_HAS_DATA, GPS_DL_IRQ_CTRL_FROM_THREAD);
		gps_dl_irq_each_link_unmask(link_id, GPS_DL_IRQ_TYPE_HAS_NODATA, GPS_DL_IRQ_CTRL_FROM_THREAD);

		/* check if MCUB ROM ready */
		if (gps_dl_test_mask_mcub_irq_on_open_get(link_id)) {
			GDL_LOGXE(link_id, "test mask mcub irq, not unmask irq and wait reset");
			gps_dl_hal_set_mcub_irq_dis_flag(link_id, true);
			gps_dl_test_mask_mcub_irq_on_open_set(link_id, false);
		} else if (!gps_dl_hal_mcub_flag_handler(link_id)) {
			GDL_LOGXE(link_id, "mcub_flag_handler not okay, not unmask irq and wait reset");
			gps_dl_hal_set_mcub_irq_dis_flag(link_id, true);
		} else {
			gps_dl_irq_each_link_unmask(link_id,
				GPS_DL_IRQ_TYPE_MCUB, GPS_DL_IRQ_CTRL_FROM_THREAD);
		}
	} else {
		if (gps_dl_hal_get_mcub_irq_dis_flag(link_id)) {
			GDL_LOGXW(link_id, "mcub irq already disable, bypass mask irq");
			gps_dl_hal_set_mcub_irq_dis_flag(link_id, false);
		} else {
			gps_dl_irq_each_link_mask(link_id,
				GPS_DL_IRQ_TYPE_MCUB, GPS_DL_IRQ_CTRL_FROM_THREAD);
		}

		bypass_unmask_irq = false;
		if (gps_dl_hal_get_irq_dis_flag(link_id, GPS_DL_IRQ_TYPE_HAS_DATA)) {
			GDL_LOGXW(link_id, "hasdata irq already disable, bypass mask irq");
			gps_dl_hal_set_irq_dis_flag(link_id, GPS_DL_IRQ_TYPE_HAS_DATA, false);
			bypass_unmask_irq = true;
		}

		gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_DMA_BUF);
		dma_working = p_link->rx_dma_buf.dma_working_entry.is_valid;
		pending_rx = p_link->rx_dma_buf.has_pending_rx;
		if (dma_working || pending_rx) {
			p_link->rx_dma_buf.has_pending_rx = false;
			gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_DMA_BUF);

			/* It means this irq has already masked, */
			/* DON'T mask again, otherwise twice unmask might be needed */
			GDL_LOGXW(link_id,
				"has dma_working = %d, pending rx = %d, bypass mask irq",
				dma_working, pending_rx);
		} else {
			gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_DMA_BUF);
			if (!bypass_unmask_irq) {
				gps_dl_irq_each_link_mask(link_id,
					GPS_DL_IRQ_TYPE_HAS_DATA, GPS_DL_IRQ_CTRL_FROM_THREAD);
			}
		}

		/* TODO: avoid twice mask need to be handled if HAS_CTRLD */
		gps_dl_irq_each_link_mask(link_id, GPS_DL_IRQ_TYPE_HAS_NODATA, GPS_DL_IRQ_CTRL_FROM_THREAD);
	}
}

