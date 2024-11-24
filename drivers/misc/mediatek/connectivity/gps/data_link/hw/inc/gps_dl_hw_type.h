/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */

#ifndef _GPS_DL_HW_TYPE_H
#define _GPS_DL_HW_TYPE_H

#include "gps_dl_config.h"

/* to import u32 series types */
#if GPS_DL_ON_LINUX
#include <linux/types.h>
#elif GPS_DL_ON_CTP
#include "kernel_to_ctp.h"
#endif

typedef u32 conn_reg;


#define GPS_DL_HW_INVALID_ADDR (0xFFFFFFFF)

#endif /* _GPS_DL_HW_TYPE_H */

