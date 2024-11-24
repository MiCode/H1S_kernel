/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
 
#ifndef _BTMTK_DBG_EVENTS_IF_H_
#define _BTMTK_DBG_EVENTS_IF_H_

#define TP_ACT_PWR_ON 0
#define TP_ACT_PWR_OFF 1
#define TP_ACT_POLL 2
#define TP_ACT_WR_IN 3
#define TP_ACT_WR_OUT 4
#define TP_ACT_RD_IN 5
#define TP_ACT_RD_CB 6
#define TP_ACT_RD_OUT 7
#define TP_ACT_FWOWN_IN 8
#define TP_ACT_FWOWN_OUT 9
#define TP_ACT_DRVOWN_IN 10
#define TP_ACT_DRVOWN_OUT 11
#define TP_ACT_DPI_ENTER 12
#define TP_ACT_DPI_EXIT 13
#define TP_ACT_RST 14

#define TP_PAR_PASS 1
#define TP_PAR_FAIL 2
#define TP_PAR_RST_START 3
#define TP_PAR_RST_DUMP 4
#define TP_PAR_RST_OFF 5

void bt_dbg_tp_evt(unsigned int pkt_action,
		unsigned int parameter,
		unsigned int data_len,
		char *data);


#endif
