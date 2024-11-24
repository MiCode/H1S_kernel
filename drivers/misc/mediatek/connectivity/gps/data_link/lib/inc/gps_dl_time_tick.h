/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */

#ifndef _GPS_DL_TIME_TICK_H
#define _GPS_DL_TIME_TICK_H

#define GPS_DL_RW_NO_TIMEOUT (-1)
void gps_dl_sleep_us(unsigned int min_us, unsigned int max_us);
void gps_dl_wait_us(unsigned int us);
#define GDL_WAIT_US(Usec) gps_dl_wait_us(Usec)
unsigned long gps_dl_tick_get(void);
unsigned long gps_dl_tick_get_us(void);
unsigned long gps_dl_tick_get_ms(void);
int gps_dl_tick_delta_to_usec(unsigned int tick0, unsigned int tick1);

#endif /* _GPS_DL_TIME_TICK_H */

