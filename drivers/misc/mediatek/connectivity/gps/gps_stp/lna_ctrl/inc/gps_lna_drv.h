/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef _GPS_LNA_DRV_H
#define _GPS_LNA_DRV_H

#include "gps.h"
extern struct pinctrl *g_gps_lna_pinctrl_ptr;

int gps_lna_linux_plat_drv_register(void);
int gps_lna_linux_plat_drv_unregister(void);
void gps_lna_pin_ctrl(enum gps_data_link_id_enum link_id, bool dsp_is_on, bool force_en);
void gps_lna_update_status_for_md_blanking(bool gps_is_on);

#endif /* _GPS_DL_LINUX_PLAT_DRV_H */

