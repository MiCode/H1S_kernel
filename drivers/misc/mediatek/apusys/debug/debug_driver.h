// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __DEBUG_DRIVER_H__
#define __DEBUG_DRIVER_H__

#define DEBUG_PREFIX "[apusys_dbg]"

#define DBG_LOG_ERR(x, args...) \
	pr_info(DEBUG_PREFIX "[error] %s " x, __func__, ##args)

static inline void apu_dbg_print(const char *fmt, ...)
{
}
int apusys_dump_init(struct device *dev);
void apusys_dump_exit(struct device *dev);

#endif /* __DEBUG_DRIVER_H__ */
