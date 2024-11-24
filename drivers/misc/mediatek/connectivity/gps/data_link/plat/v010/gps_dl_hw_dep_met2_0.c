/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "gps_dl_config.h"
#ifdef GPS_DL_ENABLE_MET

void gps_dl_hw_dep_set_emi_write_range(unsigned int bus_emi_met_phy_addr)
{
	/* Do nothing for MT6893 or MT6885 */
}

void gps_dl_hw_dep_set_ringbuffer_mode(unsigned int mode)
{
	/* Do nothing for MT6893 or MT6885 */
}

void gps_dl_hw_dep_set_sampling_rate(unsigned int rate)
{
	/* Do nothing for MT6893 or MT6885 */
}

void gps_dl_hw_dep_set_mask_signal(unsigned int mask_signal)
{
	/* Do nothing for MT6893 or MT6885 */
}

void gps_dl_hw_dep_set_edge_detection(unsigned int edge)
{
	/* Do nothing for MT6893 or MT6885 */
}

void gps_dl_hw_dep_set_event_signal(unsigned int event_signal)
{
	/* Do nothing for MT6893 or MT6885 */
}

void gps_dl_hw_dep_set_event_select(unsigned int event_select)
{
	/* Do nothing for MT6893 or MT6885 */
}

void gps_dl_hw_dep_enable_met(void)
{
	/* Do nothing for MT6893 or MT6885 */
}

void gps_dl_hw_dep_disable_met(void)
{
	/* Do nothing for MT6893 or MT6885 */
}

unsigned int gps_dl_hw_dep_get_met_read_ptr_addr(void)
{
	/* Do nothing for MT6893 or MT6885 */
	return 1;
}

unsigned int gps_dl_hw_dep_get_met_write_ptr_addr(void)
{
	/* Do nothing for MT6893 or MT6885 */
	return 1;
}

#endif /* GPS_DL_ENABLE_MET */

