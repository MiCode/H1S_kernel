/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */

#ifndef _GPS_DL_HAL_UTIL_H
#define _GPS_DL_HAL_UTIL_H



/* bool gps_dl_hal_get_dma_int_status(enum gps_dl_hal_dma_ch_index channel); */


enum GPS_DL_HWCR_OWNER_ENUM {
	GPS_DL_HWCR_OWNER_POS,	/* for power on sequence */
	GPS_DL_HWCR_OWNER_DMA,	/* for dma control */
	GPS_DL_HWCR_OWNER_NUM
};


/* Must claim the owner before access hardware control registers */
enum GDL_RET_STATUS gps_dl_hwcr_access_claim(enum GPS_DL_HWCR_OWNER_ENUM owner);
enum GDL_RET_STATUS gps_dl_hwcr_access_disclaim(enum GPS_DL_HWCR_OWNER_ENUM owner);


#endif /* _GPS_DL_HAL_UTIL_H */

