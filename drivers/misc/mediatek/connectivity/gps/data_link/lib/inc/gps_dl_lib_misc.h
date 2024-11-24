/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */

#ifndef _GPS_DL_LIB_MISC_H
#define _GPS_DL_LIB_MISC_H

#include "gps_dl_config.h"
#include "gps_dl_base.h"
#if GPS_DL_ON_LINUX
#include <linux/types.h> /* for bool */
#endif

void gps_dl_hal_show_buf(unsigned char *tag,
	unsigned char *buf, unsigned int len);

bool gps_dl_hal_comp_buf_match(unsigned char *data_buf, unsigned int data_len,
	unsigned char *golden_buf, unsigned int golden_len, unsigned int data_shift);

#endif /* _GPS_DL_LIB_MISC_H */

