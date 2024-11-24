/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _FW_LOG_ICS_H_
#define _FW_LOG_ICS_H_

#ifdef CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH
int fw_log_ics_init(void);
int fw_log_ics_deinit(void);
#endif /* CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH */

#endif /*_FW_LOG_ICS_H_*/
