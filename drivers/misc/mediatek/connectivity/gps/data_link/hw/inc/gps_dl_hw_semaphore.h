/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#ifndef _GPS_DL_HW_SEMAPHORE_H
#define _GPS_DL_HW_SEMAPHORE_H
#include "gps_dl_config.h"

#if GPS_DL_USE_BGF_SEL_SEMA
bool gps_dl_hw_take_conn_bgf_sel_hw_sema(unsigned int try_timeout_ms);
void gps_dl_hw_give_conn_bgf_sel_hw_sema(void);
#endif
#if GPS_DL_HAS_PTA
bool gps_dl_hw_take_conn_coex_hw_sema(unsigned int timeout_ms);
void gps_dl_hw_give_conn_coex_hw_sema(void);
#endif
bool gps_dl_hw_take_conn_rfspi_hw_sema(unsigned int timeout_ms);
void gps_dl_hw_give_conn_rfspi_hw_sema(void);

#endif /* _GPS_DL_HW_SEMAPHORE_H */

