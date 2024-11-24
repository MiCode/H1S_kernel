/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef _PRE_CAL_H
#define _PRE_CAL_H

extern struct platform_device *g_prPlatDev;

extern void update_pre_cal_status(uint8_t fgIsPreCal);
extern int8_t get_pre_cal_status(void);
extern int32_t update_wr_mtx_down_up_status(uint8_t ucDownUp,
		uint8_t ucIsBlocking);
extern void wlanWakeLockInit(struct GLUE_INFO *prGlueInfo);
extern void wlanWakeLockUninit(struct GLUE_INFO *prGlueInfo);
extern struct wireless_dev *wlanNetCreate(void *pvData, void *pvDriverData);
extern void wlanNetDestroy(struct wireless_dev *prWdev);

int wlanPreCalPwrOn(void);
int wlanPreCal(void);
int wlanGetCalResultCb(uint32_t *pEmiCalOffset, uint32_t *pEmiCalSize);
uint32_t wlanPhyAction(struct ADAPTER *prAdapter);
uint8_t *wlanGetCalResult(uint32_t *prCalSize);
void wlanCalDebugCmd(uint32_t cmd, uint32_t para);

#endif
