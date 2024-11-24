/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */

#ifndef _GPS_DL_NAME_LIST_H
#define _GPS_DL_NAME_LIST_H

#include "gps_dl_config.h"

#include "gps_each_link.h"
#include "gps_dl_hal_api.h"
#include "gps_dsp_fsm.h"
#include "gps_dl_base.h"

const char *gps_dl_dsp_state_name(enum gps_dsp_state_t state);
const char *gps_dl_dsp_event_name(enum gps_dsp_event_t event);

const char *gps_dl_link_state_name(enum gps_each_link_state_enum state);
const char *gps_dl_link_event_name(enum gps_dl_link_event_id event);
const char *gps_dl_hal_event_name(enum gps_dl_hal_event_id event);

const char *gps_dl_waitable_type_name(enum gps_each_link_waitable_type type);

#endif /* _GPS_DL_NAME_LIST_H */

