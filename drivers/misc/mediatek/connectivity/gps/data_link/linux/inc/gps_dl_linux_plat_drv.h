/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */

#ifndef _GPS_DL_LINUX_PLAT_DRV_H
#define _GPS_DL_LINUX_PLAT_DRV_H

#include "gps_dl_config.h"

#if GPS_DL_HAS_PLAT_DRV
#include <linux/io.h>
struct gps_dl_iomem_addr_map_entry {
	void __iomem *host_virt_addr;
	unsigned int host_phys_addr;
	unsigned int length;
};
#endif

#if GPS_DL_HAS_PLAT_DRV
int gps_dl_linux_plat_drv_register(void);
int gps_dl_linux_plat_drv_unregister(void);
void __iomem *gps_dl_host_addr_to_virt(unsigned int host_addr);
void gps_dl_update_status_for_md_blanking(bool gps_is_on);
void gps_dl_tia_gps_ctrl(bool gps_is_on);
void gps_dl_lna_pin_ctrl(enum gps_dl_link_id_enum link_id, bool dsp_is_on, bool force_en);
void gps_dl_reserved_mem_show_info(void);
void gps_dl_wake_lock_init(void);
void gps_dl_wake_lock_deinit(void);
void gps_dl_wake_lock_hold(bool hold);
#endif

extern unsigned int b13_gps_status_addr;
#endif /* _GPS_DL_LINUX_PLAT_DRV_H */

