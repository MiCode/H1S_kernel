/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#ifndef _GPS_DL_HIST_REC2_H
#define _GPS_DL_HIST_REC2_H

#include "gps_dl_config.h"

enum gps_dl_hist_rec2_status {
	DATA_TRANS_START	= 0,
	DATA_TRANS_CONTINUE	= 1,
	DATA_TRANS_END	= 2,
	DATA_TRANS_NUm	= 3,
};

void gps_dl_hist_rec2_data_routing(enum gps_dl_link_id_enum link_id,
	enum gps_dl_hist_rec2_status rec2_status);
void gps_dl_hist_rec2_disable_data_routing(void);
void gps_dl_hist_rec2_enable_data_routing(void);

#endif /* _GPS_DL_HIST_REC2_H */

