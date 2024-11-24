/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __BTMTK_CHIP_IF_H__
#define __BTMTK_CHIP_IF_H__

#ifdef CHIP_IF_USB
#include "btmtk_usb.h"
#elif defined(CHIP_IF_SDIO)
#include "btmtk_sdio.h"
#elif defined(CHIP_IF_UART)
#include "btmtk_uart.h"
#elif defined(CHIP_IF_BTIF)
#include "btmtk_btif.h"
#define CFG_SUPPORT_BT_DL_WIFI_PATCH    0
#define CFG_SUPPORT_DVT                 0
#define CFG_SUPPORT_BLUEZ               0

#if (CONNAC20_CHIPID == 6885)
	#include "platform_mt6885.h"
#elif (CONNAC20_CHIPID == 6893)
	#include "platform_mt6893.h"
#elif (CONNAC20_CHIPID == 6877)
	#include "platform_mt6877.h"
#elif (CONNAC20_CHIPID == 6983)
	#include "platform_mt6983.h"
#elif (CONNAC20_CHIPID == 6879)
	#include "platform_mt6879.h"
#elif (CONNAC20_CHIPID == 6895)
	#include "platform_mt6895.h"
#endif
#endif

int btmtk_cif_register(void);
int btmtk_cif_deregister(void);

#endif /* __BTMTK_CHIP_IF_H__ */
