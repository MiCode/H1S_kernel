/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */

#ifndef _GPS_DL_LINUX_H
#define _GPS_DL_LINUX_H

#include "gps_dl_config.h"

#include <linux/io.h>
#include <linux/interrupt.h>

#if (GPS_DL_USE_MTK_SYNC_WRITE)
#include <sync_write.h>
#define gps_dl_linux_sync_writel(v, a) mt_reg_sync_writel(v, a)
#else
/* Add mb after writel to make sure it takes effect before next operation */
#define gps_dl_mb() mb()
#define gps_dl_linux_sync_writel(v, a)                          \
	do {                                                    \
		writel((v), (void __force __iomem *)((a)));     \
		gps_dl_mb();                                    \
	} while (0)
#endif

#include "gps_dl_isr.h"
#include "gps_each_link.h"

/* put linux proprietary items here otherwise put into gps_dl_osal.h */
irqreturn_t gps_dl_linux_irq_dispatcher(int irq, void *data);
int gps_dl_linux_irqs_register(struct gps_each_irq *p_irqs, int irq_num);
int gps_dl_linux_irqs_unregister(struct gps_each_irq *p_irqs, int irq_num);

#endif /* _GPS_DL_LINUX_H */

