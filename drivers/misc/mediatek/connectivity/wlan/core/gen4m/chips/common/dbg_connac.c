/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2016 MediaTek Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
/******************************************************************************
 *[File]             dbg_connac.c
 *[Version]          v1.0
 *[Revision Date]    2015-09-08
 *[Author]
 *[Description]
 *    The program provides WIFI MAC Debug APIs
 *[Copyright]
 *    Copyright (C) 2015 MediaTek Incorporation. All Rights Reserved.
 ******************************************************************************/

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"

#include "pse.h"
#include "wf_ple.h"
#include "host_csr.h"
#include "dma_sch.h"
#include "mt_dmac.h"

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */
#if (CFG_SUPPORT_RA_GEN == 0)
static char *RATE_TBLE[] = {"B", "G", "N", "N_2SS", "AC", "AC_2SS", "N/A"};
#else
static char *RATE_TBLE[] = {"B", "G", "N", "N_2SS", "AC", "AC_2SS", "BG",
			    "N/A"};
static char *RA_STATUS_TBLE[] = {"INVALID", "POWER_SAVING", "SLEEP", "STANDBY",
				 "RUNNING", "N/A"};
static char *LT_MODE_TBLE[] = {"RSSI", "LAST_RATE", "TRACKING", "N/A"};
static char *SGI_UNSP_STATE_TBLE[] = {"INITIAL", "PROBING", "SUCCESS",
				      "FAILURE", "N/A"};
static char *BW_STATE_TBLE[] = {"UNCHANGED", "DOWN", "N/A"};
#endif
/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

void halShowPseInfo(IN struct ADAPTER *prAdapter)
{
#define BUF_SIZE 512

	uint32_t pse_buf_ctrl = 0, pg_sz, pg_num;
	uint32_t pse_stat = 0, pse_queue_empty_mask = 0, pg_flow_ctrl[16] = {0};
	uint32_t fpg_cnt, ffa_cnt, fpg_head, fpg_tail;
	uint32_t max_q, min_q, rsv_pg, used_pg;
	uint32_t i, page_offset, addr, value = 0, pos = 0;
	char *buf;

	HAL_MCR_RD(prAdapter, PSE_PBUF_CTRL, &pse_buf_ctrl);
	HAL_MCR_RD(prAdapter, PSE_QUEUE_EMPTY, &pse_stat);
	HAL_MCR_RD(prAdapter, PSE_QUEUE_EMPTY_MASK, &pse_queue_empty_mask);
	HAL_MCR_RD(prAdapter, PSE_FREEPG_CNT, &pg_flow_ctrl[0]);
	HAL_MCR_RD(prAdapter, PSE_FREEPG_HEAD_TAIL, &pg_flow_ctrl[1]);
	HAL_MCR_RD(prAdapter, PSE_PG_HIF0_GROUP, &pg_flow_ctrl[2]);
	HAL_MCR_RD(prAdapter, PSE_HIF0_PG_INFO, &pg_flow_ctrl[3]);
	HAL_MCR_RD(prAdapter, PSE_PG_HIF1_GROUP, &pg_flow_ctrl[4]);
	HAL_MCR_RD(prAdapter, PSE_HIF1_PG_INFO, &pg_flow_ctrl[5]);
	HAL_MCR_RD(prAdapter, PSE_PG_CPU_GROUP, &pg_flow_ctrl[6]);
	HAL_MCR_RD(prAdapter, PSE_CPU_PG_INFO, &pg_flow_ctrl[7]);
	HAL_MCR_RD(prAdapter, PSE_PG_LMAC0_GROUP, &pg_flow_ctrl[8]);
	HAL_MCR_RD(prAdapter, PSE_LMAC0_PG_INFO, &pg_flow_ctrl[9]);
	HAL_MCR_RD(prAdapter, PSE_PG_LMAC1_GROUP, &pg_flow_ctrl[10]);
	HAL_MCR_RD(prAdapter, PSE_LMAC1_PG_INFO, &pg_flow_ctrl[11]);
	HAL_MCR_RD(prAdapter, PSE_PG_LMAC2_GROUP, &pg_flow_ctrl[12]);
	HAL_MCR_RD(prAdapter, PSE_LMAC2_PG_INFO, &pg_flow_ctrl[13]);
	HAL_MCR_RD(prAdapter, PSE_PG_PLE_GROUP, &pg_flow_ctrl[14]);
	HAL_MCR_RD(prAdapter, PSE_PLE_PG_INFO, &pg_flow_ctrl[15]);

	/* Configuration Info */
	DBGLOG(HAL, INFO, "PSE Configuration Info:\n");

	HAL_MCR_RD(prAdapter, PSE_GC, &value);
	DBGLOG(HAL, INFO, "\tGC(0x82068000): 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, PSE_INT_STS, &value);
	DBGLOG(HAL, INFO, "\tINT_STS(0x82068024): 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, PSE_INT_ERR_STS, &value);
	DBGLOG(HAL, INFO, "\tINT_ERR_STS(0x82068028): 0x%08x\n", value);

	DBGLOG(HAL, INFO, "\tPacket Buffer Control(0x82068014): 0x%08x\n",
		pse_buf_ctrl);
	pg_sz = (pse_buf_ctrl & (0x1 << 31)) >> 31;
	DBGLOG(HAL, INFO,
		"\t\tPage Size=%d(%d bytes per page)\n",
		pg_sz, (pg_sz == 1 ? 256 : 128));
	page_offset  = (pse_buf_ctrl & 0x3FFFFFF) >> 17;
	DBGLOG(HAL, INFO,
		 "\t\tPage Offset=%d(in unit of 2KB)\n", page_offset);
	pg_num = (pse_buf_ctrl & PSE_TOTAL_PAGE_NUM_MASK);
	DBGLOG(HAL, INFO, "\t\tAvailable Total Page=%d pages\n", pg_num);
	/* Page Flow Control */
	DBGLOG(HAL, INFO, "PSE Page Flow Control:\n");

	DBGLOG(HAL, INFO,
		"\tFree page counter(0x82068100): 0x%08x\n", pg_flow_ctrl[0]);
	fpg_cnt = pg_flow_ctrl[0] & 0xfff;
	DBGLOG(HAL, INFO,
		"\t\tThe toal page number of free=0x%03x\n", fpg_cnt);
	ffa_cnt = (pg_flow_ctrl[0] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe free page numbers of free for all=0x%03x\n", ffa_cnt);
	DBGLOG(HAL, INFO,
		"\tFree page head and tail(0x82068104): 0x%08x\n",
		pg_flow_ctrl[1]);
	fpg_head = pg_flow_ctrl[1] & 0xfff;
	fpg_tail = (pg_flow_ctrl[1] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe tail/head page of free page list=0x%03x/0x%03x\n",
		fpg_tail, fpg_head);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of HIF0 group(0x82068110): 0x%08x\n",
		pg_flow_ctrl[2]);
	DBGLOG(HAL, INFO,
		"\tHIF0 group page status(0x82068114): 0x%08x\n",
		pg_flow_ctrl[3]);
	min_q = pg_flow_ctrl[2] & 0xfff;
	max_q = (pg_flow_ctrl[2] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of HIF0 group=0x%03x/0x%03x\n",
		max_q, min_q);
	rsv_pg = pg_flow_ctrl[3] & 0xfff;
	used_pg = (pg_flow_ctrl[3] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of HIF0 group=0x%03x/0x%03x\n",
		used_pg, rsv_pg);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of HIF1 group(0x82068118): 0x%08x\n",
		pg_flow_ctrl[4]);
	DBGLOG(HAL, INFO,
		"\tHIF1 group page status(0x8206811c): 0x%08x\n",
		pg_flow_ctrl[5]);
	min_q = pg_flow_ctrl[4] & 0xfff;
	max_q = (pg_flow_ctrl[4] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of HIF1 group=0x%03x/0x%03x\n",
		max_q, min_q);
	rsv_pg = pg_flow_ctrl[5] & 0xfff;
	used_pg = (pg_flow_ctrl[5] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of HIF1 group=0x%03x/0x%03x\n",
		used_pg, rsv_pg);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of CPU group(0x82068150): 0x%08x\n",
		pg_flow_ctrl[6]);
	DBGLOG(HAL, INFO,
		"\tCPU group page status(0x82068154): 0x%08x\n",
		pg_flow_ctrl[7]);
	min_q = pg_flow_ctrl[6] & 0xfff;
	max_q = (pg_flow_ctrl[6] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of CPU group=0x%03x/0x%03x\n",
		max_q, min_q);
	rsv_pg = pg_flow_ctrl[7] & 0xfff;
	used_pg = (pg_flow_ctrl[7] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of CPU group=0x%03x/0x%03x\n",
		used_pg, rsv_pg);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of LMAC0 group(0x82068170): 0x%08x\n",
		pg_flow_ctrl[8]);
	DBGLOG(HAL, INFO,
		"\tLMAC0 group page status(0x82068174): 0x%08x\n",
		pg_flow_ctrl[9]);
	min_q = pg_flow_ctrl[8] & 0xfff;
	max_q = (pg_flow_ctrl[8] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of LMAC0 group=0x%03x/0x%03x\n",
		max_q, min_q);
	rsv_pg = pg_flow_ctrl[9] & 0xfff;
	used_pg = (pg_flow_ctrl[9] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of LMAC0 group=0x%03x/0x%03x\n",
		used_pg, rsv_pg);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of LMAC1 group(0x82068178): 0x%08x\n",
		pg_flow_ctrl[10]);
	DBGLOG(HAL, INFO,
		"\tLMAC1 group page status(0x8206817c): 0x%08x\n",
		pg_flow_ctrl[11]);
	min_q = pg_flow_ctrl[10] & 0xfff;
	max_q = (pg_flow_ctrl[10] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of LMAC1 group=0x%03x/0x%03x\n",
		max_q, min_q);
	rsv_pg = pg_flow_ctrl[11] & 0xfff;
	used_pg = (pg_flow_ctrl[11] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of LMAC1 group=0x%03x/0x%03x\n",
		used_pg, rsv_pg);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of LMAC2 group(0x82068180): 0x%08x\n",
		pg_flow_ctrl[12]);
	DBGLOG(HAL, INFO,
		"\tLMAC2 group page status(0x82068184): 0x%08x\n",
		pg_flow_ctrl[13]);
	min_q = pg_flow_ctrl[12] & 0xfff;
	max_q = (pg_flow_ctrl[12] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of LMAC2 group=0x%03x/0x%03x\n",
		max_q, min_q);
	rsv_pg = pg_flow_ctrl[13] & 0xfff;
	used_pg = (pg_flow_ctrl[13] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of LMAC2 group=0x%03x/0x%03x\n",
		used_pg, rsv_pg);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of PLE group(0x82068190): 0x%08x\n",
		pg_flow_ctrl[14]);
	DBGLOG(HAL, INFO,
		"\tPLE group page status(0x82068194): 0x%08x\n",
		pg_flow_ctrl[15]);
	min_q = pg_flow_ctrl[14] & 0xfff;
	max_q = (pg_flow_ctrl[14] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of PLE group=0x%03x/0x%03x\n",
		max_q, min_q);
	rsv_pg = pg_flow_ctrl[15] & 0xfff;
	used_pg = (pg_flow_ctrl[15] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of PLE group=0x%03x/0x%03x\n",
		used_pg, rsv_pg);
	/* Queue Empty Status */
	DBGLOG(HAL, INFO, "PSE Queue Empty Status:\n");
	DBGLOG(HAL, INFO,
		"\tQUEUE_EMPTY(0x820680b0): 0x%08x\n",
		pse_stat);
	DBGLOG(HAL, INFO,
		"\tQUEUE_EMPTY_MASK(0x820680b4): 0x%08x\n",
		pse_queue_empty_mask);
	DBGLOG(HAL, INFO,
		"\t\tCPU Q0/1/2/3 empty=%d/%d/%d/%d\n",
		 pse_stat & 0x1, ((pse_stat & 0x2) >> 1),
		 ((pse_stat & 0x4) >> 2), ((pse_stat & 0x8) >> 3));
	DBGLOG(HAL, INFO,
		"\t\tHIF Q0/1/2/3 empty=%d/%d/%d/%d\n",
		 ((pse_stat & (0x1 << 16)) >> 16),
		 ((pse_stat & (0x1 << 17)) >> 17),
		 ((pse_stat & (0x1 << 18)) >> 18),
		 ((pse_stat & (0x1 << 19)) >> 19));
	DBGLOG(HAL, INFO,
		"\t\tLMAC TX Q empty=%d\n",
		 ((pse_stat & (0x1 << 24)) >> 24));
	DBGLOG(HAL, INFO,
		"\t\tRLS_Q empty=%d\n",
		 ((pse_stat & (0x1 << 31)) >> 31));

	buf = (char *) kalMemAlloc(BUF_SIZE, VIR_MEM_TYPE);
	if (buf) {
		kalMemZero(buf, BUF_SIZE);
		pos = 0;
		for (i = 0; i < PSE_PEEK_CR_NUM; i++) {
			addr = PSE_PEEK_CR_0 + PSE_PEEK_CR_OFFSET * i;
			HAL_MCR_RD(prAdapter, addr, &value);
			pos += kalSnprintf(buf + pos, 40,
				"PSE_PEEK_CR_%u[0x%08x/0x%08x] ",
				i, addr, value);
		}
		DBGLOG(HAL, INFO, "%s\n", buf);

		kalMemZero(buf, BUF_SIZE);
		pos = 0;
		for (i = 0; i < PSE_ENDEQ_NUM; i++) {
			addr = PSE_ENQ_0 + PSE_ENDEQ_OFFSET * i;
			HAL_MCR_RD(prAdapter, addr, &value);
			pos += kalSnprintf(buf + pos, 40,
				"PSE_ENQ_%u[0x%08x/0x%08x] ",
				i, addr, value);
		}
		DBGLOG(HAL, INFO, "%s\n", buf);

		kalMemZero(buf, BUF_SIZE);
		pos = 0;
		for (i = 0; i < PSE_ENDEQ_NUM; i++) {
			addr = PSE_DEQ_0 + PSE_ENDEQ_OFFSET * i;
			HAL_MCR_RD(prAdapter, addr, &value);
			pos += kalSnprintf(buf + pos, 40,
				"PSE_DEQ_%u[0x%08x/0x%08x] ",
				i, addr, value);
		}
		DBGLOG(HAL, INFO, "%s\n", buf);

		kalMemFree(buf, VIR_MEM_TYPE, BUF_SIZE);
	}

#undef BUF_SIZE
}

static int8_t *sta_ctrl_reg[] = {"ENABLE", "*DISABLE", "*PAUSE"};
static struct EMPTY_QUEUE_INFO Queue_Empty_info[] = {
	{"CPU Q0",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_0},
	{"CPU Q1",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_1},
	{"CPU Q2",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_2},
	{"CPU Q3",  ENUM_UMAC_CPU_PORT_1,     ENUM_UMAC_CTX_Q_3},
	{NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	{"ALTX Q0", ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_ALTX_0},
	{"BMC Q0",  ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_BMC_0},
	{"BCN Q0",  ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_BNC_0},
	{"PSMP Q0", ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_PSMP_0},
	{"ALTX Q1", ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_ALTX_1},
	{"BMC Q1",  ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_BMC_1},
	{"BCN Q1",  ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_BNC_1},
	{"PSMP Q1", ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_PSMP_1},
	{"NAF Q",   ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_NAF},
	{"NBCN Q",  ENUM_UMAC_LMAC_PORT_2,    ENUM_UMAC_LMAC_PLE_TX_Q_NBCN},
	{NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	{NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	{NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	{NULL, 0, 0}, {NULL, 0, 0}, {NULL, 0, 0},
	{"RLS Q",   ENUM_PLE_CTRL_PSE_PORT_3, ENUM_UMAC_PLE_CTRL_P3_Q_0X1E},
	{"RLS2 Q",  ENUM_PLE_CTRL_PSE_PORT_3, ENUM_UMAC_PLE_CTRL_P3_Q_0X1F}
};

/* =============================================================================
 *                         Debug Interrupt Interface v1
 * +---------------------------------------------------------------------------+
 * |Toggle|Rsv[30:28]|Ver[27:24]|Rsv[23:18]|BSSInx[17:14]|Mod[13:8]|Reason[7:0]|
 * +---------------------------------------------------------------------------+
 * =============================================================================
 */
uint32_t halGetPleInt(struct ADAPTER *prAdapter)
{
	uint32_t u4Val = 0;

	HAL_MCR_RD(prAdapter, PLE_TO_N9_INT, &u4Val);

	return u4Val;
}

void halSetPleInt(struct ADAPTER *prAdapter, bool fgTrigger,
		  uint32_t u4ClrMask, uint32_t u4SetMask)
{
	uint32_t u4Val = 0;

	HAL_MCR_RD(prAdapter, PLE_TO_N9_INT, &u4Val);

	if (fgTrigger) {
		u4Val = (~u4Val & PLE_TO_N9_INT_TOGGLE_MASK) |
			(u4Val & ~PLE_TO_N9_INT_TOGGLE_MASK);
	}

	u4Val &= ~u4ClrMask;
	u4Val |= u4SetMask;

	HAL_MCR_WR(prAdapter, PLE_TO_N9_INT, u4Val);
}

void halShowPleInfo(IN struct ADAPTER *prAdapter,
	u_int8_t fgDumpTxd)
{
#define BUF_SIZE 1024

	uint32_t ple_buf_ctrl[3] = {0}, pg_sz, pg_num, bit_field_1, bit_field_2;
	uint32_t ple_stat[17] = {0}, pg_flow_ctrl[6] = {0};
	uint32_t sta_pause[4] = {0}, dis_sta_map[4] = {0};
	uint32_t fpg_cnt, ffa_cnt, fpg_head, fpg_tail, hif_max_q, hif_min_q;
	uint32_t rpg_hif, upg_hif, cpu_max_q, cpu_min_q, rpg_cpu, upg_cpu;
	uint32_t i, j, addr, value = 0, pos = 0;
	char *buf;

	HAL_MCR_RD(prAdapter, PLE_PBUF_CTRL, &ple_buf_ctrl[0]);
	HAL_MCR_RD(prAdapter, PLE_RELEASE_CTRL, &ple_buf_ctrl[1]);
	HAL_MCR_RD(prAdapter, PLE_HIF_REPORT, &ple_buf_ctrl[2]);
	HAL_MCR_RD(prAdapter, PLE_QUEUE_EMPTY, &ple_stat[0]);
	HAL_MCR_RD(prAdapter, PLE_AC0_QUEUE_EMPTY_0, &ple_stat[1]);
	HAL_MCR_RD(prAdapter, PLE_AC0_QUEUE_EMPTY_1, &ple_stat[2]);
	HAL_MCR_RD(prAdapter, PLE_AC0_QUEUE_EMPTY_2, &ple_stat[3]);
	HAL_MCR_RD(prAdapter, PLE_AC0_QUEUE_EMPTY_3, &ple_stat[4]);
	HAL_MCR_RD(prAdapter, PLE_AC1_QUEUE_EMPTY_0, &ple_stat[5]);
	HAL_MCR_RD(prAdapter, PLE_AC1_QUEUE_EMPTY_1, &ple_stat[6]);
	HAL_MCR_RD(prAdapter, PLE_AC1_QUEUE_EMPTY_2, &ple_stat[7]);
	HAL_MCR_RD(prAdapter, PLE_AC1_QUEUE_EMPTY_3, &ple_stat[8]);
	HAL_MCR_RD(prAdapter, PLE_AC2_QUEUE_EMPTY_0, &ple_stat[9]);
	HAL_MCR_RD(prAdapter, PLE_AC2_QUEUE_EMPTY_1, &ple_stat[10]);
	HAL_MCR_RD(prAdapter, PLE_AC2_QUEUE_EMPTY_2, &ple_stat[11]);
	HAL_MCR_RD(prAdapter, PLE_AC2_QUEUE_EMPTY_3, &ple_stat[12]);
	HAL_MCR_RD(prAdapter, PLE_AC3_QUEUE_EMPTY_0, &ple_stat[13]);
	HAL_MCR_RD(prAdapter, PLE_AC3_QUEUE_EMPTY_1, &ple_stat[14]);
	HAL_MCR_RD(prAdapter, PLE_AC3_QUEUE_EMPTY_2, &ple_stat[15]);
	HAL_MCR_RD(prAdapter, PLE_AC3_QUEUE_EMPTY_3, &ple_stat[16]);
	HAL_MCR_RD(prAdapter, PLE_FREEPG_CNT, &pg_flow_ctrl[0]);
	HAL_MCR_RD(prAdapter, PLE_FREEPG_HEAD_TAIL, &pg_flow_ctrl[1]);
	HAL_MCR_RD(prAdapter, PLE_PG_HIF_GROUP, &pg_flow_ctrl[2]);
	HAL_MCR_RD(prAdapter, PLE_HIF_PG_INFO, &pg_flow_ctrl[3]);
	HAL_MCR_RD(prAdapter, PLE_PG_CPU_GROUP, &pg_flow_ctrl[4]);
	HAL_MCR_RD(prAdapter, PLE_CPU_PG_INFO, &pg_flow_ctrl[5]);
	HAL_MCR_RD(prAdapter, DIS_STA_MAP0, &dis_sta_map[0]);
	HAL_MCR_RD(prAdapter, DIS_STA_MAP1, &dis_sta_map[1]);
	HAL_MCR_RD(prAdapter, DIS_STA_MAP2, &dis_sta_map[2]);
	HAL_MCR_RD(prAdapter, DIS_STA_MAP3, &dis_sta_map[3]);
	HAL_MCR_RD(prAdapter, STATION_PAUSE0, &sta_pause[0]);
	HAL_MCR_RD(prAdapter, STATION_PAUSE1, &sta_pause[1]);
	HAL_MCR_RD(prAdapter, STATION_PAUSE2, &sta_pause[2]);
	HAL_MCR_RD(prAdapter, STATION_PAUSE3, &sta_pause[3]);
	/* Configuration Info */
	DBGLOG(HAL, INFO, "PLE Configuration Info:\n");

	HAL_MCR_RD(prAdapter, PLE_GC, &value);
	DBGLOG(HAL, INFO, "\tGC(0x82060000): 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, PLE_INT_STS, &value);
	DBGLOG(HAL, INFO, "\tINT_STS(0x82060024): 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, PLE_INT_ERR_STS, &value);
	DBGLOG(HAL, INFO, "\tINT_ERR_STS(0x82060028): 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, PLE_QUEUE_CMD_ERR_STS, &value);
	DBGLOG(HAL, INFO, "\tQUEUE_CMD_ERR_STS(0x82060550): 0x%08x\n", value);

	DBGLOG(HAL, INFO,
		"\tPacket Buffer Control(0x82060014): 0x%08x\n",
		ple_buf_ctrl[0]);
	pg_sz = (ple_buf_ctrl[0] & (0x1 << 31)) >> 31;
	DBGLOG(HAL, INFO,
		"\t\tPage Size=%d(%d bytes per page)\n",
		pg_sz, (pg_sz == 1 ? 128 : 64));
	DBGLOG(HAL, INFO,
		"\t\tPage Offset=%d(in unit of 2KB)\n",
		(ple_buf_ctrl[0] & (0xf << 17)) >> 17);
	pg_num = (ple_buf_ctrl[0] & 0xfff);
	DBGLOG(HAL, INFO,
		"\t\tTotal Page=%d pages\n", (ple_buf_ctrl[0] & 0xfff));
	DBGLOG(HAL, INFO,
		"\tRelease Control(0x82060030): 0x%08x\n", ple_buf_ctrl[1]);
	bit_field_1 = (ple_buf_ctrl[1] & 0x1f);
	bit_field_2 = ((ple_buf_ctrl[1] & (0x3 << 6)) >> 6);
	DBGLOG(HAL, INFO,
		"\t\tNormalTx Release Pid/Qid=%d/%d\n",
		bit_field_2, bit_field_1);
	bit_field_1 = ((ple_buf_ctrl[1] & (0x1f << 8)) >> 8);
	bit_field_2 = ((ple_buf_ctrl[1] & (0x3 << 14)) >> 14);
	DBGLOG(HAL, INFO,
		"\t\tDropTx Release Pid/Qid=%d/%d\n", bit_field_2, bit_field_1);
	bit_field_1 = ((ple_buf_ctrl[1] & (0x1f << 16)) >> 16);
	bit_field_2 = ((ple_buf_ctrl[1] & (0x3 << 22)) >> 22);
	DBGLOG(HAL, INFO,
		"\t\tBCN0 Release Pid/Qid=%d/%d\n", bit_field_2, bit_field_1);
	bit_field_1 = ((ple_buf_ctrl[1] & (0x1f << 24)) >> 24);
	bit_field_2 = ((ple_buf_ctrl[1] & (0x3 << 30)) >> 30);
	DBGLOG(HAL, INFO,
		"\t\tBCN1 Release Pid/Qid=%d/%d\n", bit_field_2, bit_field_1);
	DBGLOG(HAL, INFO,
		"\tHIF Report Control(0x82060034): 0x%08x\n", ple_buf_ctrl[2]);
	bit_field_1 = ((ple_buf_ctrl[2] & (0x1 << 1)) >> 1);
	DBGLOG(HAL, INFO,
		"\t\tHostReportQSel/HostReportDisable=%d/%d\n",
			  (ple_buf_ctrl[2] & 0x1), bit_field_1);
	/* Page Flow Control */
	DBGLOG(HAL, INFO, "PLE Page Flow Control:\n");
	DBGLOG(HAL, INFO,
		"\tFree page counter(0x82060100): 0x%08x\n", pg_flow_ctrl[0]);
	fpg_cnt = pg_flow_ctrl[0] & 0xfff;
	DBGLOG(HAL, INFO,
		"\t\tThe toal page number of free=0x%03x\n", fpg_cnt);
	ffa_cnt = (pg_flow_ctrl[0] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe free page numbers of free for all=0x%03x\n", ffa_cnt);
	DBGLOG(HAL, INFO,
		"\tFree page head and tail(0x82060104): 0x%08x\n",
		pg_flow_ctrl[1]);
	fpg_head = pg_flow_ctrl[1] & 0xfff;
	fpg_tail = (pg_flow_ctrl[1] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe tail/head page of free page list=0x%03x/0x%03x\n",
		fpg_tail, fpg_head);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of HIF group(0x82060110): 0x%08x\n",
		pg_flow_ctrl[2]);
	DBGLOG(HAL, INFO,
		"\tHIF group page status(0x82060114): 0x%08x\n",
		pg_flow_ctrl[3]);
	hif_min_q = pg_flow_ctrl[2] & 0xfff;
	hif_max_q = (pg_flow_ctrl[2] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of HIF group=0x%03x/0x%03x\n",
		hif_max_q, hif_min_q);
	rpg_hif = pg_flow_ctrl[3] & 0xfff;
	upg_hif = (pg_flow_ctrl[3] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of HIF group=0x%03x/0x%03x\n",
		upg_hif, rpg_hif);
	DBGLOG(HAL, INFO,
		"\tReserved page counter of CPU group(0x82060150): 0x%08x\n",
		pg_flow_ctrl[4]);
	DBGLOG(HAL, INFO,
		"\tCPU group page status(0x82060154): 0x%08x\n",
		pg_flow_ctrl[5]);
	cpu_min_q = pg_flow_ctrl[4] & 0xfff;
	cpu_max_q = (pg_flow_ctrl[4] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe max/min quota pages of CPU group=0x%03x/0x%03x\n",
		cpu_max_q, cpu_min_q);
	rpg_cpu = pg_flow_ctrl[5] & 0xfff;
	upg_cpu = (pg_flow_ctrl[5] & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
		"\t\tThe used/reserved pages of CPU group=0x%03x/0x%03x\n",
		upg_cpu, rpg_cpu);

	if (((ple_stat[0] & (0x1 << 24)) >> 24) == 0) {
		DBGLOG(HAL, INFO,
			"\tAC0_QUEUE_EMPTY0(0x82060300): 0x%08x\n",
			ple_stat[1]);
		DBGLOG(HAL, INFO,
			"\tAC1_QUEUE_EMPTY0(0x82060310): 0x%08x\n",
			ple_stat[5]);
		DBGLOG(HAL, INFO,
			"\tAC2_QUEUE_EMPTY0(0x82060320): 0x%08x\n",
			ple_stat[9]);
		DBGLOG(HAL, INFO,
			"\tAC3_QUEUE_EMPTY0(0x82060330): 0x%08x\n",
			ple_stat[13]);

		for (j = 0; j < 16; j = j + 4) {
			if (j % 4 == 0)
				DBGLOG(HAL, INFO, "\tNonempty AC%d Q of STA#:",
					j / 4);

			for (i = 0; i < 32; i++) {
				if (((ple_stat[j + 1] & (0x1 << i)) >> i) == 0)
					DBGLOG(HAL, INFO, "%d ",
						i + (j % 4) * 32);
			}
		}

		DBGLOG(HAL, INFO, "\n");
	}

	DBGLOG(HAL, INFO, "Nonempty Q info:\n");

	for (i = 0; i < 31; i++) {
		if (((ple_stat[0] & (0x1 << i)) >> i) == 0) {
			if (Queue_Empty_info[i].QueueName != NULL)
				DBGLOG(HAL, INFO, "\t%s: ",
					Queue_Empty_info[i].QueueName);
			else
				continue;
		}
	}

	for (j = 0; j < 16; j = j + 4) { /* show AC Q info */
		for (i = 0; i < 32; i++) {
			if (((ple_stat[j + 1] & (0x1 << i)) >> i) == 0) {
				uint32_t ac_num = j / 4, ctrl = 0;
				uint32_t sta_num = i + (j % 4) * 32;

				DBGLOG(HAL, INFO, "\tSTA%d AC%d: ",
					sta_num, ac_num);
				if (((sta_pause[j % 4] & 0x1 << i) >> i) == 1)
					ctrl = 2;

				if (((dis_sta_map[j % 4] & 0x1 << i) >> i) == 1)
					ctrl = 1;

				DBGLOG(HAL, INFO, " ctrl = %s",
					sta_ctrl_reg[ctrl]);
			}
		}
	}

	buf = (char *) kalMemAlloc(BUF_SIZE, VIR_MEM_TYPE);
	if (buf) {
		kalMemZero(buf, BUF_SIZE);
		for (i = 0; i < PLE_PEEK_CR_NUM; i++) {
			addr = PLE_PEEK_CR_0 + PLE_PEEK_CR_OFFSET * i;
			HAL_MCR_RD(prAdapter, addr, &value);
			pos += kalSnprintf(buf + pos, 40,
				"PLE_PEEK_CR_%u[0x%08x/0x%08x]%s",
				i, addr, value,
				i == (PLE_PEEK_CR_NUM - 1) ? "\n" : ", ");
		}
		DBGLOG(HAL, INFO, "%s\n", buf);
		kalMemFree(buf, VIR_MEM_TYPE, BUF_SIZE);
	}

#undef BUF_SIZE
}

void halShowDmaschInfo(IN struct ADAPTER *prAdapter)
{
	struct mt66xx_chip_info *prChipInfo;
	uint32_t value = 0;
	uint32_t ple_pkt_max_sz;
	uint32_t pse_pkt_max_sz;
	uint32_t max_quota;
	uint32_t min_quota;
	uint32_t rsv_cnt;
	uint32_t src_cnt;
	uint32_t pse_rsv_cnt = 0;
	uint32_t pse_src_cnt = 0;
	uint32_t odd_group_pktin_cnt = 0;
	uint32_t odd_group_ask_cnt = 0;
	uint32_t pktin_cnt;
	uint32_t ask_cnt;
	uint32_t total_src_cnt = 0;
	uint32_t total_rsv_cnt = 0;
	uint32_t ffa_cnt;
	uint32_t free_pg_cnt;
	uint32_t Group_Mapping_Q[16] = {0};
	uint32_t qmapping_addr = MT_HIF_DMASHDL_Q_MAP0;
	uint32_t status_addr = MT_HIF_DMASHDL_STATUS_RD_GP0;
	uint32_t quota_addr = MT_HIF_DMASHDL_GROUP0_CTRL;
	uint32_t pkt_cnt_addr = MT_HIF_DMASHDLRD_GP_PKT_CNT_0;
	uint32_t mapping_mask = 0xf;
	uint32_t mapping_offset = 0;
	uint32_t mapping_qidx;
	uint32_t groupidx = 0;
	uint8_t idx = 0;
	u_int8_t pktin_int_refill_ena;
	u_int8_t pdma_add_int_refill_ena;
	u_int8_t ple_add_int_refill_ena;
	u_int8_t ple_sub_ena;
	u_int8_t hif_ask_sub_ena;
	u_int8_t ple_txd_gt_max_size_flag_clr;
	uint32_t ple_rpg_hif;
	uint32_t ple_upg_hif;
	uint32_t pse_rpg_hif = 0;
	uint32_t pse_upg_hif = 0;
	uint8_t is_mismatch = FALSE;
	uint32_t u4BaseAddr;

	prChipInfo = prAdapter->chip_info;
	u4BaseAddr = prChipInfo->u4HifDmaShdlBaseAddr;
	for (mapping_qidx = 0; mapping_qidx < 32; mapping_qidx++) {
		uint32_t mapping_group;

		idx = 0;

		if (mapping_qidx == 0) {
			qmapping_addr = MT_HIF_DMASHDL_Q_MAP0;
			mapping_mask = 0xf;
			mapping_offset = 0;
		} else if ((mapping_qidx % 8) == 0) {
			qmapping_addr += 0x4;
			mapping_mask = 0xf;
			mapping_offset = 0;
		} else {
			mapping_offset += 4;
			mapping_mask = 0xf << mapping_offset;
		}

		HAL_MCR_RD(prAdapter, qmapping_addr, &value);
		mapping_group = (value & mapping_mask) >> mapping_offset;
		Group_Mapping_Q[mapping_group] |= 1 << mapping_qidx;
	}

	DBGLOG(HAL, INFO, "Dma scheduler info:\n");
	HAL_MCR_RD(prAdapter, MT_HIF_DMASHDL_CTRL_SIGNAL, &value);
	pktin_int_refill_ena =
		(value & DMASHDL_PKTIN_INT_REFILL_ENA) ? TRUE : FALSE;
	pdma_add_int_refill_ena =
		(value & DMASHDL_PDMA_ADD_INT_REFILL_ENA) ? TRUE : FALSE;
	ple_add_int_refill_ena =
		(value & DMASHDL_PLE_ADD_INT_REFILL_ENA) ? TRUE : FALSE;
	ple_sub_ena = (value & DMASHDL_PLE_SUB_ENA) ? TRUE : FALSE;
	hif_ask_sub_ena = (value & DMASHDL_HIF_ASK_SUB_ENA) ? TRUE : FALSE;
	ple_txd_gt_max_size_flag_clr =
		(value & DMASHDL_PLE_TXD_GT_MAX_SIZE_FLAG_CLR) ? TRUE : FALSE;
	DBGLOG(HAL, INFO, "DMASHDL Ctrl Signal(0x5000A018): 0x%08x\n", value);
	DBGLOG(HAL, INFO, "\tple_txd_gt_max_size_flag_clr(BIT0) = %d\n",
		ple_txd_gt_max_size_flag_clr);
	DBGLOG(HAL, INFO, "\thif_ask_sub_ena(BIT16) = %d\n", hif_ask_sub_ena);
	DBGLOG(HAL, INFO, "\tple_sub_ena(BIT17) = %d\n", ple_sub_ena);
	DBGLOG(HAL, INFO, "\tple_add_int_refill_ena(BIT29) = %d\n",
		ple_add_int_refill_ena);
	DBGLOG(HAL, INFO, "\tpdma_add_int_refill_ena(BIT30) = %d\n",
		pdma_add_int_refill_ena);
	DBGLOG(HAL, INFO, "\tpktin_int_refill(BIT31)_ena = %d\n",
		pktin_int_refill_ena);
	HAL_MCR_RD(prAdapter, MT_HIF_DMASHDL_PKT_MAX_SIZE, &value);
	ple_pkt_max_sz = GET_PLE_PKT_MAX_SIZE_NUM(value);
	pse_pkt_max_sz = GET_PSE_PKT_MAX_SIZE_NUM(value);
	DBGLOG(HAL, INFO,
		"DMASHDL Packet_max_size(0x5000A01c): 0x%08x\n", value);
	DBGLOG(HAL, INFO,
		"PLE/PSE packet max size=0x%03x/0x%03x\n",
		 ple_pkt_max_sz, pse_pkt_max_sz);
	HAL_MCR_RD(prAdapter, MT_HIF_DMASHDL_ERROR_FLAG_CTRL, &value);
	DBGLOG(HAL, INFO, "DMASHDL ERR FLAG CTRL(0x5000A09c): 0x%08x\n", value);
	HAL_MCR_RD(prAdapter, MT_HIF_DMASHDL_STATUS_RD, &value);
	ffa_cnt = (value & DMASHDL_FFA_CNT_MASK) >> DMASHDL_FFA_CNT_OFFSET;
	free_pg_cnt = (value & DMASHDL_FREE_PG_CNT_MASK) >>
		DMASHDL_FREE_PG_CNT_OFFSET;
	DBGLOG(HAL, INFO, "DMASHDL Status_RD(0x5000A100): 0x%08x\n", value);
	DBGLOG(HAL, INFO,
		"free page cnt = 0x%03x, ffa cnt = 0x%03x\n",
		free_pg_cnt, ffa_cnt);
	HAL_MCR_RD(prAdapter,
		CONN_HIF_DMASHDL_REFILL_CONTROL(u4BaseAddr), &value);
	DBGLOG(HAL, INFO,
		"DMASHDL ReFill Control(0x5000A010): 0x%08x\n", value);

	for (groupidx = 0; groupidx < 16; groupidx++) {
		DBGLOG(HAL, INFO, "Group %d info:", groupidx);
		HAL_MCR_RD(prAdapter, status_addr, &value);
		rsv_cnt = (value & DMASHDL_RSV_CNT_MASK) >>
			DMASHDL_RSV_CNT_OFFSET;
		src_cnt = (value & DMASHDL_SRC_CNT_MASK) >>
			DMASHDL_SRC_CNT_OFFSET;
		DBGLOG(HAL, INFO,
			"\tDMASHDL Status_RD_GP%d(0x%08x): 0x%08x\n",
			groupidx, status_addr, value);
		HAL_MCR_RD(prAdapter, quota_addr, &value);
		max_quota = (value & DMASHDL_MAX_QUOTA_MASK) >>
			DMASHDL_MAX_QUOTA_OFFSET;
		min_quota = (value & DMASHDL_MIN_QUOTA_MASK) >>
			DMASHDL_MIN_QUOTA_OFFSET;
		DBGLOG(HAL, INFO,
			"\tDMASHDL Group%d control(0x%08x): 0x%08x\n",
			groupidx, quota_addr, value);

		if ((groupidx & 0x1) == 0) {
			HAL_MCR_RD(prAdapter, pkt_cnt_addr, &value);
			DBGLOG(HAL, INFO,
			"\tDMASHDL RD_group_pkt_cnt_%d(0x%08x): 0x%08x\n",
			groupidx / 2, pkt_cnt_addr, value);
			odd_group_pktin_cnt = GET_ODD_GROUP_PKT_IN_CNT(value);
			odd_group_ask_cnt = GET_ODD_GROUP_ASK_CNT(value);
			pktin_cnt = GET_EVEN_GROUP_PKT_IN_CNT(value);
			ask_cnt = GET_EVEN_GROUP_ASK_CNT(value);
		} else {
			pktin_cnt = odd_group_pktin_cnt;
			ask_cnt = odd_group_ask_cnt;
		}

		DBGLOG(HAL, INFO,
			"\trsv_cnt = 0x%03x, src_cnt = 0x%03x\n",
			rsv_cnt, src_cnt);
		DBGLOG(HAL, INFO,
			"\tmax/min quota = 0x%03x/ 0x%03x\n",
			max_quota, min_quota);
		DBGLOG(HAL, INFO,
			"\tpktin_cnt = 0x%02x, ask_cnt = 0x%02x",
			pktin_cnt, ask_cnt);

		if (hif_ask_sub_ena && pktin_cnt != ask_cnt) {
			DBGLOG(HAL, INFO, ", mismatch!");
			is_mismatch = TRUE;
		}

		/* Group15 is for PSE */
		if (groupidx == 15 && Group_Mapping_Q[groupidx] == 0) {
			pse_src_cnt = src_cnt;
			pse_rsv_cnt = rsv_cnt;
			break;
		}

		DBGLOG(HAL, INFO, "\tMapping Qidx: 0x%x",
		       Group_Mapping_Q[groupidx]);

		total_src_cnt += src_cnt;
		total_rsv_cnt += rsv_cnt;
		status_addr = status_addr + 4;
		quota_addr = quota_addr + 4;

		if (groupidx & 0x1)
			pkt_cnt_addr = pkt_cnt_addr + 4;
	}

	DBGLOG(HAL, INFO, "\nCounter Check:\n");
	HAL_MCR_RD(prAdapter, PLE_HIF_PG_INFO, &value);
	ple_rpg_hif = value & 0xfff;
	ple_upg_hif = (value & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
	"PLE:\n\tThe used/reserved pages of PLE HIF group=0x%03x/0x%03x\n",
	ple_upg_hif, ple_rpg_hif);
	HAL_MCR_RD(prAdapter, PSE_HIF1_PG_INFO, &value);
	pse_rpg_hif = value & 0xfff;
	pse_upg_hif = (value & (0xfff << 16)) >> 16;
	DBGLOG(HAL, INFO,
	"PSE:\n\tThe used/reserved pages of PSE HIF group=0x%03x/0x%03x\n",
	pse_upg_hif, pse_rpg_hif);
	DBGLOG(HAL, INFO,
		"DMASHDL:\n\tThe total used pages of group0~14=0x%03x",
		 total_src_cnt);

	if (ple_upg_hif != total_src_cnt) {
		DBGLOG(HAL, INFO, ", mismatch!");
		is_mismatch = TRUE;
	}

	DBGLOG(HAL, INFO, "\n");
	DBGLOG(HAL, INFO,
		"\tThe total reserved pages of group0~14=0x%03x\n",
		total_rsv_cnt);
	DBGLOG(HAL, INFO,
		"\tThe total ffa pages of group0~14=0x%03x\n",
		ffa_cnt);
	DBGLOG(HAL, INFO,
		"\tThe total free pages of group0~14=0x%03x", free_pg_cnt);

	if (free_pg_cnt != total_rsv_cnt + ffa_cnt) {
		DBGLOG(HAL, INFO,
			", mismatch(total_rsv_cnt + ffa_cnt in DMASHDL)");
		is_mismatch = TRUE;
	}

	if (free_pg_cnt != ple_rpg_hif) {
		DBGLOG(HAL, INFO, ", mismatch(reserved pages in PLE)");
		is_mismatch = TRUE;
	}

	DBGLOG(HAL, INFO, "\n");
	DBGLOG(HAL, INFO, "\tThe used pages of group15=0x%03x", pse_src_cnt);

	if (pse_upg_hif != pse_src_cnt) {
		DBGLOG(HAL, INFO, ", mismatch!");
		is_mismatch = TRUE;
	}

	DBGLOG(HAL, INFO, "\n");
	DBGLOG(HAL, INFO,
		"\tThe reserved pages of group15=0x%03x", pse_rsv_cnt);

	if (pse_rpg_hif != pse_rsv_cnt) {
		DBGLOG(HAL, INFO, ", mismatch!");
		is_mismatch = TRUE;
	}

	DBGLOG(HAL, INFO, "\n");

	if (!is_mismatch)
		DBGLOG(HAL, INFO, "DMASHDL: no counter mismatch\n");
}

/* return: new pos */
int32_t halDumpRegToBuffer(struct ADAPTER *prAdapter,
	char *buf, uint32_t bufSize, int32_t pos,
	uint32_t reg)
{
#define __DUMP_STR__ "0x%08x:0x%08x "
#define DUMP_STR_LEN 23
	int len = 0;
	uint32_t value = 0;

	if (pos + DUMP_STR_LEN < bufSize) {
		HAL_MCR_RD(prAdapter, reg, &value);
		len = kalSnprintf(buf + pos, DUMP_STR_LEN,
			__DUMP_STR__,
			reg, value);
	} else {
		DBGLOG(HAL, ERROR,
			"buffer full bufSize:%d pos:%d\n",
			bufSize, pos);
		return -1;
	}

	return pos + len;
#undef __DUMP_STR__
#undef DUMP_STR_LEN
}

void halDumpRegInRange(struct ADAPTER *prAdapter,
	char *buf, uint32_t bufSize, uint32_t maxInLine,
	uint32_t start, uint32_t end, uint32_t offset)
{
	int i = 0;
	u_int8_t fgEnd = FALSE;
	int32_t pos = 0;
	uint32_t reg;

	ASSERT(start <= end);

	reg = start;
	while (!fgEnd) {
		for (i = 0; i < maxInLine;  i++) {
			pos = halDumpRegToBuffer(prAdapter,
				buf, bufSize, pos, reg);

			/* buffer full, early break */
			if (pos == -1)
				break;
			/* reach end reg, so break */
			if (reg >= end) {
				fgEnd = TRUE;
				break;
			}

			reg += offset;
			kalMdelay(1);
		}
		DBGLOG(HAL, INFO, "Dump CR: %s\n", buf);
		pos = 0;
	}
}

uint32_t halDumpRegInArray(struct ADAPTER *prAdapter,
	char *buf, uint32_t bufSize, int32_t pos,
	uint32_t pArr[], uint32_t elemSize, uint32_t maxSize)
{
	int i = 0;
	uint32_t reg;
	uint32_t ret;

	for (i = 0; i < maxSize;  i++) {
		reg = pArr[i];
		ret = halDumpRegToBuffer(prAdapter,
			buf, bufSize, pos, reg);
		/* buffer full, early break */
		if (ret == -1)
			break;
		pos = ret;
	}

	return pos;
}

void haldumpMacInfo(struct ADAPTER *prAdapter)
{
#define BUF_SIZE 1024
#define LOOP_COUNT 30
	uint32_t i = 0, j = 0, pos = 0;
	uint32_t u4RegValue1 = 0, u4RegValue2 = 0, u4RegValue3 = 0;
	uint32_t cr_band0[] = {
		0x820F0000, /* [25] clock enable */
		0x820F3080, /* [ 9: 8] RX/TX disable */
		0x82060028, /* PLE error interrupt */
		0x82068028, /* PSE error interrupt */
		0x820F4124, /* [18:16] TXV count */
		0x820F4130, /* TXV1 */
		0x820F4134, /* TXV2 */
		0x820F4138, /* TXV3 */
		0x820F413C, /* TXV4 */
		0x820F4140, /* TXV5 */
		0x820F4144, /* TXV6 */
		0x820F4148, /* TXV7 */
		0x820F2050, /* [12:8] ERP protection */
	};
	uint32_t cr_band0_loop[] = {
		0x820FD020, /* [15: 0] channel idle count */
		0x820F4128, /* [15: 0] TMAC FSM & CCA */
		0x820F20D0, /* AGG0 FSM */
		0x820F20D4, /* AGG1 FSM */
		0x820F20D8, /* AGG2 FSM */
		0x820F20DC, /* AGG3 FSM */
		0x820F3190, /* [15:14] TX start/slot idle toggle */
		0x82060220, /* Queue empty, Check [25:24][19:16][15:0] */
		0x82060114, /* HIF SRC count */
		0x82060154, /* N9 SRC count */
		0x820F0024, /* Abort condition */
	};
	uint32_t cr_band1[] = {
		0x820F0000, /* [24] clock enable */
		0x820F3080, /* [11:10] RX/TX disable */
		0x82060028, /* PLE error interrupt */
		0x82068028, /* PSE error interrupt */
		0x820F4124, /* [26:24] TXV count */
		0x820F414C, /* TXV1 */
		0x820F4150, /* TXV2 */
		0x820F4154, /* TXV3 */
		0x820F4158, /* TXV4 */
		0x820F415C, /* TXV5 */
		0x820F4160, /* TXV6 */
		0x820F4164, /* TXV7 */
		0x820F2050, /* [12:10] & [24] ERP protection */
	};
	uint32_t cr_band1_loop[] = {
		0x820FD220, /* [15: 0] channel idle count */
		0x820F4128, /* [31:16] TMAC FSM & CCA */
		0x820F20D0, /* AGG0 FSM */
		0x820F20D4, /* AGG1 FSM */
		0x820F20D8, /* AGG2 FSM */
		0x820F20DC, /* AGG3 FSM */
		0x820F3190, /* [31:30] TX start/slot idle toggle */
		0x82060220, /* Queue empty, Check [25:24][23:20][15:0] */
		0x82060114, /* HIF SRC count */
		0x82060154, /* N9 SRC count */
		0x820F0024, /* Abort condition */
	};
	uint32_t cr_arb_debug_flag_loop[] = {
		0xa5a5a4a4, /* BT RW(ARB RW) */
		0xd5d5d4d4, /* ARBtoAGG RW */
		0x92929595, /* [13]:RW==0? [4]:sleep_flag */
		0x99999999, /* PTA-ARB interface  */
		0x40404141, /* arb back-off state & ARB-UMAC interface */
		0xe0e0e5e5, /* ARB-UMAC interface */
		0x04040505, /* queue status */
	};
	uint32_t cr_arb_queue_sel_loop[] = {
		0x00000000,
		0x01010101,
		0x02020202,
		0x03030303,
		0x04040404,
		0x05050505,
		0x06060606,
		0x07070707,
		0x08080808,
		0x09090909,
		0x0A0A0A0A,
		0x0B0B0B0B,
		0x0C0C0C0C,
		0x0D0D0D0D,
		0x0E0E0E0E,
		0x0F0F0F0F,
		0x10101010,
		0x11111111,
		0x12121212,
		0x13131313,
	};

	char *buf = (char *) kalMemAlloc(BUF_SIZE, VIR_MEM_TYPE);

	DBGLOG(HAL, INFO, "Dump for band0\n");
	HAL_MCR_WR(prAdapter, 0x820F082C, 0xF);
	HAL_MCR_WR(prAdapter, 0x80025100, 0x1F);
	HAL_MCR_WR(prAdapter, 0x80025104, 0x07070707);
	HAL_MCR_WR(prAdapter, 0x80025108, 0x38383737);

	if (buf) {
		kalMemZero(buf, BUF_SIZE);
		pos = halDumpRegInArray(prAdapter, buf, BUF_SIZE,
			pos, cr_band0, sizeof(uint32_t),
			ARRAY_SIZE(cr_band0));

		DBGLOG(HAL, INFO, "Dump CR: %s\n", buf);
		pos = 0;

		kalMemZero(buf, BUF_SIZE);
		for (i = 0; i < LOOP_COUNT; i++) {
			pos = halDumpRegInArray(prAdapter, buf, BUF_SIZE,
				pos, cr_band0_loop, sizeof(uint32_t),
				ARRAY_SIZE(cr_band0_loop));
			DBGLOG(HAL, INFO, "Dump CR: %s\n", buf);
			pos = 0;
			kalMdelay(1);
		}
	}

	DBGLOG(HAL, INFO, "Dump for band1\n");
	HAL_MCR_WR(prAdapter, 0x820F082C, 0xF);
	HAL_MCR_WR(prAdapter, 0x80025100, 0x1F);
	HAL_MCR_WR(prAdapter, 0x80025104, 0x07070707);
	HAL_MCR_WR(prAdapter, 0x80025108, 0x38383737);

	if (buf) {
		kalMemZero(buf, BUF_SIZE);
		pos = halDumpRegInArray(prAdapter, buf, BUF_SIZE,
			pos, cr_band1, sizeof(uint32_t),
			ARRAY_SIZE(cr_band1));

		DBGLOG(HAL, INFO, "Dump CR: %s\n", buf);
		pos = 0;

		kalMemZero(buf, BUF_SIZE);
		for (i = 0; i < LOOP_COUNT; i++) {
			pos = halDumpRegInArray(prAdapter, buf, BUF_SIZE,
				pos, cr_band1_loop, sizeof(uint32_t),
				ARRAY_SIZE(cr_band1_loop));
			DBGLOG(HAL, INFO, "Dump CR: %s\n", buf);
			pos = 0;
			kalMdelay(1);
		}
	}

	DBGLOG(HAL, INFO, "Dump for ARB\n");
	HAL_MCR_RD(prAdapter, 0x820F3200, &u4RegValue1);
	HAL_MCR_RD(prAdapter, 0x820F3278, &u4RegValue2);
	HAL_MCR_RD(prAdapter, 0x820F327C, &u4RegValue3);
	DBGLOG(HAL, INFO,
		"ARB_DBG: B[0]Read 0x820F3200=0x%x Read 0x820F3278=0x%x Read 0x820F327C=0x%x\n",
		u4RegValue1, u4RegValue2, u4RegValue3);

	HAL_MCR_WR(prAdapter, 0x820F082C, 0xF);
	HAL_MCR_WR(prAdapter, 0x80025100, 0x1F);
	HAL_MCR_WR(prAdapter, 0x80025104, 0x04040404);
	for (i = 0; i < LOOP_COUNT; i++) {
		/* [15:11]:tx_que_num */
		HAL_MCR_WR(prAdapter, 0x80025108, 0x4d4dacac);
		HAL_MCR_RD(prAdapter, 0x820F0024, &u4RegValue1);
		DBGLOG(HAL, INFO,
			"ARB_DBG: B[0]Set 0x80025108 = 0x4d4dacac read 0x820F0024=0x%x\n",
			u4RegValue1);
		/* queue freeze flag */
		HAL_MCR_WR(prAdapter, 0x80025108, 0x6c6c6d6d);
		HAL_MCR_RD(prAdapter, 0x820F0024, &u4RegValue1);
		DBGLOG(HAL, INFO,
			"ARB_DBG: B[0]Set 0x80025108 = 0x6c6c6d6d read 0x820F0024=0x%x\n",
			u4RegValue1);
	}

	for (i = 0; i < LOOP_COUNT; i++) {
		/* ARB debug flags */
		for (j = 0; j < ARRAY_SIZE(cr_arb_debug_flag_loop); j++) {
			HAL_MCR_WR(prAdapter, 0x80025108,
				cr_arb_debug_flag_loop[j]);
			HAL_MCR_RD(prAdapter, 0x820F0024, &u4RegValue1);
			DBGLOG(HAL, INFO,
				"ARB_DBG: B[0]Set 0x80025108 = 0x%x read 0x820F0024=0x%x\n",
				cr_arb_debug_flag_loop[j], u4RegValue1);
		}
		/* ARB queue status */
		HAL_MCR_WR(prAdapter, 0x80025108, 0x04040505);
		for (j = 0; j < ARRAY_SIZE(cr_arb_queue_sel_loop); j++) {
			HAL_MCR_WR(prAdapter, 0x820f3060,
				cr_arb_queue_sel_loop[j]);
			HAL_MCR_RD(prAdapter, 0x820F0024, &u4RegValue1);
			DBGLOG(HAL, INFO,
				"ARB_DBG: B[0]Set 0x80025108 = 0x04040505 Set 0x820f3060 = 0x%x read 0x820F0024=0x%x\n",
				cr_arb_queue_sel_loop[j], u4RegValue1);
		}
	}

	if (buf)
		kalMemFree(buf, VIR_MEM_TYPE, BUF_SIZE);
#undef LOOP_COUNT
#undef BUF_SIZE
}

static char *q_idx_mcu_str[] = {"RQ0", "RQ1", "RQ2", "RQ3", "Invalid"};
static char *pkt_ft_str[] = {"cut_through", "store_forward",
	"cmd", "PDA_FW_Download"};
static char *hdr_fmt_str[] = {
	"Non-80211-Frame",
	"Command-Frame",
	"Normal-80211-Frame",
	"enhanced-80211-Frame",
};
static char *p_idx_str[] = {"LMAC", "MCU"};
static char *q_idx_lmac_str[] = {"WMM0_AC0", "WMM0_AC1", "WMM0_AC2", "WMM0_AC3",
	"WMM1_AC0", "WMM1_AC1", "WMM1_AC2", "WMM1_AC3",
	"WMM2_AC0", "WMM2_AC1", "WMM2_AC2", "WMM2_AC3",
	"WMM3_AC0", "WMM3_AC1", "WMM3_AC2", "WMM3_AC3",
	"Band0_ALTX", "Band0_BMC", "Band0_BNC", "Band0_PSMP",
	"Band1_ALTX", "Band1_BMC", "Band1_BNC", "Band1_PSMP",
	"Invalid"};

void halDumpTxdInfo(IN struct ADAPTER *prAdapter, uint8_t *tmac_info)
{
	struct TMAC_TXD_S *txd_s;
	struct TMAC_TXD_0 *txd_0;
	struct TMAC_TXD_1 *txd_1;
	uint8_t q_idx = 0;

	txd_s = (struct TMAC_TXD_S *)tmac_info;
	txd_0 = &txd_s->TxD0;
	txd_1 = &txd_s->TxD1;

	DBGLOG(HAL, INFO, "TMAC_TXD Fields:\n");
	DBGLOG(HAL, INFO, "\tTMAC_TXD_0:\n");
	DBGLOG(HAL, INFO, "\t\tPortID=%d(%s)\n",
			txd_0->p_idx, p_idx_str[txd_0->p_idx]);

	if (txd_0->p_idx == P_IDX_LMAC)
		q_idx = txd_0->q_idx % 0x18;
	else
		q_idx = ((txd_0->q_idx == TxQ_IDX_MCU_PDA) ?
			txd_0->q_idx : (txd_0->q_idx % 0x4));

	DBGLOG(HAL, INFO, "\t\tQueID=0x%x(%s %s)\n", txd_0->q_idx,
			 (txd_0->p_idx == P_IDX_LMAC ? "LMAC" : "MCU"),
			 txd_0->p_idx == P_IDX_LMAC ?
				q_idx_lmac_str[q_idx] : q_idx_mcu_str[q_idx]);
	DBGLOG(HAL, INFO, "\t\tTxByteCnt=%d\n", txd_0->TxByteCount);
	DBGLOG(HAL, INFO, "\t\tIpChkSumOffload=%d\n", txd_0->IpChkSumOffload);
	DBGLOG(HAL, INFO, "\t\tUdpTcpChkSumOffload=%d\n",
						txd_0->UdpTcpChkSumOffload);
	DBGLOG(HAL, INFO, "\t\tEthTypeOffset=%d\n", txd_0->EthTypeOffset);

	DBGLOG(HAL, INFO, "\tTMAC_TXD_1:\n");
	DBGLOG(HAL, INFO, "\t\twlan_idx=%d\n", txd_1->wlan_idx);
	DBGLOG(HAL, INFO, "\t\tHdrFmt=%d(%s)\n",
			 txd_1->hdr_format, hdr_fmt_str[txd_1->hdr_format]);
	DBGLOG(HAL, INFO, "\t\tHdrInfo=0x%x\n", txd_1->hdr_info);

	switch (txd_1->hdr_format) {
	case TMI_HDR_FT_NON_80211:
		DBGLOG(HAL, INFO,
			"\t\t\tMRD=%d, EOSP=%d, RMVL=%d, VLAN=%d, ETYP=%d\n",
			txd_1->hdr_info & (1 << TMI_HDR_INFO_0_BIT_MRD),
			txd_1->hdr_info & (1 << TMI_HDR_INFO_0_BIT_EOSP),
			txd_1->hdr_info & (1 << TMI_HDR_INFO_0_BIT_RMVL),
			txd_1->hdr_info & (1 << TMI_HDR_INFO_0_BIT_VLAN),
			txd_1->hdr_info & (1 << TMI_HDR_INFO_0_BIT_ETYP));
		break;

	case TMI_HDR_FT_CMD:
		DBGLOG(HAL, INFO, "\t\t\tRsvd=0x%x\n", txd_1->hdr_info);
		break;

	case TMI_HDR_FT_NOR_80211:
		DBGLOG(HAL, INFO, "\t\t\tHeader Len=%d(WORD)\n",
				 txd_1->hdr_info & TMI_HDR_INFO_2_MASK_LEN);
		break;

	case TMI_HDR_FT_ENH_80211:
		DBGLOG(HAL, INFO, "\t\t\tEOSP=%d, AMS=%d\n",
			txd_1->hdr_info & (1 << TMI_HDR_INFO_3_BIT_EOSP),
			txd_1->hdr_info & (1 << TMI_HDR_INFO_3_BIT_AMS));
		break;
	}

	DBGLOG(HAL, INFO, "\t\tTxDFormatType=%d(%s format)\n", txd_1->ft,
		(txd_1->ft == TMI_FT_LONG ?
		"Long - 8 DWORD" : "Short - 3 DWORD"));
	DBGLOG(HAL, INFO, "\t\ttxd_len=%d page(%d DW)\n",
		txd_1->txd_len == 0 ? 1 : 2, (txd_1->txd_len + 1) * 16);
	DBGLOG(HAL, INFO,
		"\t\tHdrPad=%d(Padding Mode: %s, padding bytes: %d)\n",
		txd_1->hdr_pad,
		((txd_1->hdr_pad & (TMI_HDR_PAD_MODE_TAIL << 1)) ?
		"tail" : "head"), (txd_1->hdr_pad & 0x1 ? 2 : 0));
	DBGLOG(HAL, INFO, "\t\tUNxV=%d\n", txd_1->UNxV);
	DBGLOG(HAL, INFO, "\t\tamsdu=%d\n", txd_1->amsdu);
	DBGLOG(HAL, INFO, "\t\tTID=%d\n", txd_1->tid);
	DBGLOG(HAL, INFO, "\t\tpkt_ft=%d(%s)\n",
			 txd_1->pkt_ft, pkt_ft_str[txd_1->pkt_ft]);
	DBGLOG(HAL, INFO, "\t\town_mac=%d\n", txd_1->OwnMacAddr);

	if (txd_s->TxD1.ft == TMI_FT_LONG) {
		struct TMAC_TXD_L *txd_l = (struct TMAC_TXD_L *)tmac_info;
		struct TMAC_TXD_2 *txd_2 = &txd_l->TxD2;
		struct TMAC_TXD_3 *txd_3 = &txd_l->TxD3;
		struct TMAC_TXD_4 *txd_4 = &txd_l->TxD4;
		struct TMAC_TXD_5 *txd_5 = &txd_l->TxD5;
		struct TMAC_TXD_6 *txd_6 = &txd_l->TxD6;

		DBGLOG(HAL, INFO, "\tTMAC_TXD_2:\n");
		DBGLOG(HAL, INFO, "\t\tsub_type=%d\n", txd_2->sub_type);
		DBGLOG(HAL, INFO, "\t\tfrm_type=%d\n", txd_2->frm_type);
		DBGLOG(HAL, INFO, "\t\tNDP=%d\n", txd_2->ndp);
		DBGLOG(HAL, INFO, "\t\tNDPA=%d\n", txd_2->ndpa);
		DBGLOG(HAL, INFO, "\t\tSounding=%d\n", txd_2->sounding);
		DBGLOG(HAL, INFO, "\t\tRTS=%d\n", txd_2->rts);
		DBGLOG(HAL, INFO, "\t\tbc_mc_pkt=%d\n", txd_2->bc_mc_pkt);
		DBGLOG(HAL, INFO, "\t\tBIP=%d\n", txd_2->bip);
		DBGLOG(HAL, INFO, "\t\tDuration=%d\n", txd_2->duration);
		DBGLOG(HAL, INFO, "\t\tHE(HTC Exist)=%d\n", txd_2->htc_vld);
		DBGLOG(HAL, INFO, "\t\tFRAG=%d\n", txd_2->frag);
		DBGLOG(HAL, INFO, "\t\tReamingLife/MaxTx time=%d\n",
			txd_2->max_tx_time);
		DBGLOG(HAL, INFO, "\t\tpwr_offset=%d\n", txd_2->pwr_offset);
		DBGLOG(HAL, INFO, "\t\tba_disable=%d\n", txd_2->ba_disable);
		DBGLOG(HAL, INFO, "\t\ttiming_measure=%d\n",
			txd_2->timing_measure);
		DBGLOG(HAL, INFO, "\t\tfix_rate=%d\n", txd_2->fix_rate);
		DBGLOG(HAL, INFO, "\tTMAC_TXD_3:\n");
		DBGLOG(HAL, INFO, "\t\tNoAck=%d\n", txd_3->no_ack);
		DBGLOG(HAL, INFO, "\t\tPF=%d\n", txd_3->protect_frm);
		DBGLOG(HAL, INFO, "\t\ttx_cnt=%d\n", txd_3->tx_cnt);
		DBGLOG(HAL, INFO, "\t\tremain_tx_cnt=%d\n",
			txd_3->remain_tx_cnt);
		DBGLOG(HAL, INFO, "\t\tsn=%d\n", txd_3->sn);
		DBGLOG(HAL, INFO, "\t\tpn_vld=%d\n", txd_3->pn_vld);
		DBGLOG(HAL, INFO, "\t\tsn_vld=%d\n", txd_3->sn_vld);
		DBGLOG(HAL, INFO, "\tTMAC_TXD_4:\n");
		DBGLOG(HAL, INFO, "\t\tpn_low=0x%x\n", txd_4->pn_low);
		DBGLOG(HAL, INFO, "\tTMAC_TXD_5:\n");
		DBGLOG(HAL, INFO, "\t\ttx_status_2_host=%d\n",
			txd_5->tx_status_2_host);
		DBGLOG(HAL, INFO, "\t\ttx_status_2_mcu=%d\n",
			txd_5->tx_status_2_mcu);
		DBGLOG(HAL, INFO, "\t\ttx_status_fmt=%d\n",
			txd_5->tx_status_fmt);

		if (txd_5->tx_status_2_host || txd_5->tx_status_2_mcu)
			DBGLOG(HAL, INFO, "\t\tpid=%d\n", txd_5->pid);

		if (txd_2->fix_rate)
			DBGLOG(HAL, INFO,
				"\t\tda_select=%d\n", txd_5->da_select);

		DBGLOG(HAL, INFO, "\t\tpwr_mgmt=0x%x\n", txd_5->pwr_mgmt);
		DBGLOG(HAL, INFO, "\t\tpn_high=0x%x\n", txd_5->pn_high);

		if (txd_2->fix_rate) {
			DBGLOG(HAL, INFO, "\tTMAC_TXD_6:\n");
			DBGLOG(HAL, INFO, "\t\tfix_rate_mode=%d\n",
				txd_6->fix_rate_mode);
			DBGLOG(HAL, INFO, "\t\tGI=%d(%s)\n", txd_6->gi,
				(txd_6->gi == 0 ? "LONG" : "SHORT"));
			DBGLOG(HAL, INFO, "\t\tldpc=%d(%s)\n", txd_6->ldpc,
				(txd_6->ldpc == 0 ? "BCC" : "LDPC"));
			DBGLOG(HAL, INFO, "\t\tTxBF=%d\n", txd_6->TxBF);
			DBGLOG(HAL, INFO, "\t\ttx_rate=0x%x\n", txd_6->tx_rate);
			DBGLOG(HAL, INFO, "\t\tant_id=%d\n", txd_6->ant_id);
			DBGLOG(HAL, INFO, "\t\tdyn_bw=%d\n", txd_6->dyn_bw);
			DBGLOG(HAL, INFO, "\t\tbw=%d\n", txd_6->bw);
		}
	}
}

void halShowTxdInfo(
	struct ADAPTER *prAdapter,
	u_int32_t fid)
{
	/*
	 * TODO: Follow connac 2x design and get TXD from PLE by FW.

	uint32_t txd_info[16] = {0};

	halGetPleTxdInfo(prAdapter, fid, txd_info);
	halDumpTxdInfo(prAdapter, txd_info);
	*/
}

int32_t halShowStatInfo(struct ADAPTER *prAdapter,
			IN char *pcCommand, IN int i4TotalLen,
			struct PARAM_HW_WLAN_INFO *prHwWlanInfo,
			struct PARAM_GET_STA_STATISTICS *prQueryStaStatistics,
			u_int8_t fgResetCnt, uint32_t u4StatGroup)
{
	int32_t i4BytesWritten = 0;
	int32_t rRssi;
	uint16_t u2LinkSpeed;
	uint32_t u4Per, u4RxPer[ENUM_BAND_NUM], u4AmpduPer[ENUM_BAND_NUM],
		 u4InstantPer;
	uint8_t ucDbdcIdx, ucSkipAr, ucStaIdx, ucNss;
	static uint32_t u4TotalTxCnt, u4TotalFailCnt;
	static uint32_t u4Rate1TxCnt, u4Rate1FailCnt;
	static uint32_t au4RxMpduCnt[ENUM_BAND_NUM] = {0};
	static uint32_t au4FcsError[ENUM_BAND_NUM] = {0};
	static uint32_t au4RxFifoCnt[ENUM_BAND_NUM] = {0};
	static uint32_t au4AmpduTxSfCnt[ENUM_BAND_NUM] = {0};
	static uint32_t au4AmpduTxAckSfCnt[ENUM_BAND_NUM] = {0};
	struct RX_CTRL *prRxCtrl;
	uint32_t u4InstantRxPer[ENUM_BAND_NUM];
	struct PARAM_CUSTOM_SW_CTRL_STRUCT rSwCtrlInfo;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	int16_t i2Wf0AvgPwr = 0, i2Wf1AvgPwr = 0;
	uint32_t u4BufLen = 0;
#if (CFG_SUPPORT_RA_GEN == 1)
	uint8_t ucRaTableNum = sizeof(RATE_TBLE) / sizeof(char *);
	uint8_t ucRaStatusNum = sizeof(RA_STATUS_TBLE) / sizeof(char *);
	uint8_t ucRaLtModeNum = sizeof(LT_MODE_TBLE) / sizeof(char *);
	uint8_t ucRaSgiUnSpStateNum = sizeof(SGI_UNSP_STATE_TBLE) /
								sizeof(char *);
	uint8_t ucRaBwStateNum = sizeof(BW_STATE_TBLE) / sizeof(char *);
	uint8_t ucAggRange[AGG_RANGE_SEL_NUM] = {0};
	uint32_t u4RangeCtrl_0, u4RangeCtrl_1;
	enum AGG_RANGE_TYPE_T eRangeType = ENUM_AGG_RANGE_TYPE_TX;
#endif
	uint8_t ucBssIndex = AIS_DEFAULT_INDEX;
	struct PARAM_LINK_SPEED_EX rLinkSpeed;

	ucSkipAr = prQueryStaStatistics->ucSkipAr;
	prRxCtrl = &prAdapter->rRxCtrl;
	ucNss = prAdapter->rWifiVar.ucNSS;

	if (ucSkipAr) {
#if (CFG_SUPPORT_RA_GEN == 1)
		u4TotalTxCnt += prQueryStaStatistics->u4TransmitCount;
		u4TotalFailCnt += prQueryStaStatistics->u4TransmitFailCount;
		u4Rate1TxCnt += prQueryStaStatistics->u4Rate1TxCnt;
		u4Rate1FailCnt += prQueryStaStatistics->u4Rate1FailCnt;

		u4Per = (u4Rate1TxCnt == 0) ?
			(0) : (1000 * (u4Rate1FailCnt) / (u4Rate1TxCnt));
		u4InstantPer = (prQueryStaStatistics->u4Rate1TxCnt == 0) ?
			(0) : (1000 * (prQueryStaStatistics->u4Rate1FailCnt) /
			(prQueryStaStatistics->u4Rate1TxCnt));
#else
		u4TotalTxCnt += prHwWlanInfo->rWtblTxCounter.u2CurBwTxCnt +
			prHwWlanInfo->rWtblTxCounter.u2OtherBwTxCnt;
		u4TotalFailCnt += prHwWlanInfo->rWtblTxCounter.u2CurBwFailCnt +
			prHwWlanInfo->rWtblTxCounter.u2OtherBwFailCnt;
		u4Rate1TxCnt += prHwWlanInfo->rWtblTxCounter.u2Rate1TxCnt;
		u4Rate1FailCnt += prHwWlanInfo->rWtblTxCounter.u2Rate1FailCnt;
		u4Per = (prHwWlanInfo->rWtblTxCounter.u2Rate1TxCnt == 0) ?
			(0) : (1000 * u4Rate1FailCnt / u4Rate1TxCnt);
		u4InstantPer =
			(prHwWlanInfo->rWtblTxCounter.u2Rate1TxCnt == 0) ? (0) :
			(1000 * (prHwWlanInfo->rWtblTxCounter.u2Rate1FailCnt) /
			(prHwWlanInfo->rWtblTxCounter.u2Rate1TxCnt));
#endif
	} else {
		u4Per = (prQueryStaStatistics->u4Rate1TxCnt == 0) ?
			(0) : (1000 * (prQueryStaStatistics->u4Rate1FailCnt) /
			(prQueryStaStatistics->u4Rate1TxCnt));
		u4InstantPer = (prQueryStaStatistics->ucPer == 0) ?
			(0) : (prQueryStaStatistics->ucPer);
	}

	for (ucDbdcIdx = 0; ucDbdcIdx < ENUM_BAND_NUM; ucDbdcIdx++) {
		au4RxMpduCnt[ucDbdcIdx] +=
		    prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4RxMpduCnt;
		au4FcsError[ucDbdcIdx] +=
		    prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4FcsError;
		au4RxFifoCnt[ucDbdcIdx] +=
		    prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4RxFifoFull;
		au4AmpduTxSfCnt[ucDbdcIdx] +=
		    prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4AmpduTxSfCnt;
		au4AmpduTxAckSfCnt[ucDbdcIdx] +=
		    prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4AmpduTxAckSfCnt;

		u4RxPer[ucDbdcIdx] =
		    ((au4RxMpduCnt[ucDbdcIdx] + au4FcsError[ucDbdcIdx]) == 0) ?
			(0) : (1000 * au4FcsError[ucDbdcIdx] /
			(au4RxMpduCnt[ucDbdcIdx] +
			au4FcsError[ucDbdcIdx]));

		u4AmpduPer[ucDbdcIdx] =
		    (au4AmpduTxSfCnt[ucDbdcIdx] == 0) ?
			(0) : (1000 * (au4AmpduTxSfCnt[ucDbdcIdx] -
			au4AmpduTxAckSfCnt[ucDbdcIdx]) /
			au4AmpduTxSfCnt[ucDbdcIdx]);

		u4InstantRxPer[ucDbdcIdx] =
			((prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4RxMpduCnt
			+ prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4FcsError)
			== 0) ? (0) : (1000 * prQueryStaStatistics->
			rMibInfo[ucDbdcIdx].u4FcsError /
			(prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4RxMpduCnt +
			prQueryStaStatistics->rMibInfo[ucDbdcIdx].u4FcsError));
	}

	rRssi = RCPI_TO_dBm(prQueryStaStatistics->ucRcpi);
	u2LinkSpeed = (prQueryStaStatistics->u2LinkSpeed == 0) ? 0 :
					prQueryStaStatistics->u2LinkSpeed / 2;

	/* =========== Group 0x0001 =========== */
	if (u4StatGroup & 0x0001) {
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%s", "\n----- STA Stat (Group 0x01) -----\n");

		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%d\n", "CurrTemperature", " = ",
			prQueryStaStatistics->ucTemperature);

		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%d\n", "Tx Total cnt", " = ",
			ucSkipAr ? (u4TotalTxCnt) :
				(prQueryStaStatistics->u4TransmitCount));

		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%d\n", "Tx Fail Cnt", " = ",
			ucSkipAr ? (u4TotalFailCnt) :
				(prQueryStaStatistics->u4TransmitFailCount));

		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%d\n", "Rate1 Tx Cnt", " = ",
			ucSkipAr ? (u4Rate1TxCnt) :
				(prQueryStaStatistics->u4Rate1TxCnt));

		if (ucSkipAr)
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d, PER = %d.%1d%%, instant PER = %d.%1d%%\n",
				"Rate1 Fail Cnt", " = ",
				u4Rate1FailCnt, u4Per/10, u4Per%10,
				u4InstantPer/10, u4InstantPer%10);
		else
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d, PER = %d.%1d%%, instant PER = %d%%\n",
				"Rate1 Fail Cnt", " = ",
				prQueryStaStatistics->u4Rate1FailCnt,
				u4Per/10, u4Per%10, u4InstantPer);

		if ((ucSkipAr) && (fgResetCnt)) {
			u4TotalTxCnt = 0;
			u4TotalFailCnt = 0;
			u4Rate1TxCnt = 0;
			u4Rate1FailCnt = 0;
		}
	}

	/* =========== Group 0x0002 =========== */
	if (u4StatGroup & 0x0002) {
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%s", "----- MIB Info (Group 0x02) -----\n");

		for (ucDbdcIdx = 0; ucDbdcIdx < ENUM_BAND_NUM; ucDbdcIdx++) {
			if (prAdapter->rWifiVar.fgDbDcModeEn)
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"[DBDC_%d] :\n", ucDbdcIdx);

			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d\n", "RX Success", " = ",
				au4RxMpduCnt[ucDbdcIdx]);

			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d, PER = %d.%1d%%, instant PER = %d.%1d%%\n",
				"RX with CRC", " = ", au4FcsError[ucDbdcIdx],
				u4RxPer[ucDbdcIdx] / 10,
				u4RxPer[ucDbdcIdx] % 10,
				u4InstantRxPer[ucDbdcIdx] / 10,
				u4InstantRxPer[ucDbdcIdx] % 10);

			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d\n", "RX drop FIFO full", " = ",
				au4RxFifoCnt[ucDbdcIdx]);

			if (!prAdapter->rWifiVar.fgDbDcModeEn)
				break;
		}

		if (fgResetCnt) {
			kalMemZero(au4RxMpduCnt, sizeof(au4RxMpduCnt));
			kalMemZero(au4FcsError, sizeof(au4RxMpduCnt));
			kalMemZero(au4RxFifoCnt, sizeof(au4RxMpduCnt));
			kalMemZero(au4AmpduTxSfCnt, sizeof(au4RxMpduCnt));
			kalMemZero(au4AmpduTxAckSfCnt, sizeof(au4RxMpduCnt));
		}
	}

	/* =========== Group 0x0004 =========== */
	if (u4StatGroup & 0x0004) {
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%s", "----- Last Rx Info (Group 0x04) -----\n");

		/* get Beacon RSSI */
		ucBssIndex = secGetBssIdxByWlanIdx
			(prAdapter, (uint8_t)(prHwWlanInfo->u4Index));

		rStatus = kalIoctlByBssIdx(prAdapter->prGlueInfo,
				   wlanoidQueryRssi, &rLinkSpeed,
				   sizeof(rLinkSpeed), TRUE, TRUE, TRUE,
				   &u4BufLen, ucBssIndex);
		if (rStatus != WLAN_STATUS_SUCCESS)
			DBGLOG(REQ, WARN, "unable to retrieve rssi\n");
		if (ucBssIndex < BSSID_NUM)
			rRssi = rLinkSpeed.rLq[ucBssIndex].cRssi;

		rSwCtrlInfo.u4Data = 0;
		rSwCtrlInfo.u4Id = CMD_SW_DBGCTL_ADVCTL_GET_ID + 1;
#if (CFG_SUPPORT_RA_GEN == 0)
		rStatus = kalIoctl(prAdapter->prGlueInfo,
				   wlanoidQuerySwCtrlRead, &rSwCtrlInfo,
				   sizeof(rSwCtrlInfo), TRUE, TRUE, TRUE,
				   &u4BufLen);
#endif
		DBGLOG(REQ, LOUD, "rStatus %u, rSwCtrlInfo.u4Data 0x%x\n",
		       rStatus, rSwCtrlInfo.u4Data);
		if (rStatus == WLAN_STATUS_SUCCESS) {
			i2Wf0AvgPwr = rSwCtrlInfo.u4Data & 0xFFFF;
			i2Wf1AvgPwr = (rSwCtrlInfo.u4Data >> 16) & 0xFFFF;

			i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%d %d\n", "NOISE", " = ",
					i2Wf0AvgPwr, i2Wf1AvgPwr);
		}

		/* Last RX Rate */
		i4BytesWritten += nicGetRxRateInfo(prAdapter,
			pcCommand + i4BytesWritten, i4TotalLen - i4BytesWritten,
			(uint8_t)(prHwWlanInfo->u4Index));

		/* Last RX RSSI */
		i4BytesWritten += nicRxGetLastRxRssi(prAdapter,
			pcCommand + i4BytesWritten, i4TotalLen - i4BytesWritten,
			(uint8_t)(prHwWlanInfo->u4Index));

		/* Last TX Resp RSSI */
		if (ucNss > 2)
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d %d %d %d\n",
				"Tx Response RSSI", " = ",
				RCPI_TO_dBm(
				    prHwWlanInfo->rWtblRxCounter.ucRxRcpi0),
				RCPI_TO_dBm(
				    prHwWlanInfo->rWtblRxCounter.ucRxRcpi1),
				RCPI_TO_dBm(
				    prHwWlanInfo->rWtblRxCounter.ucRxRcpi2),
				RCPI_TO_dBm(
				    prHwWlanInfo->rWtblRxCounter.ucRxRcpi3));
		else
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d %d\n", "Tx Response RSSI", " = ",
				RCPI_TO_dBm(
				    prHwWlanInfo->rWtblRxCounter.ucRxRcpi0),
				RCPI_TO_dBm(
				    prHwWlanInfo->rWtblRxCounter.ucRxRcpi1));

		/* Last Beacon RSSI */
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d\n", "Beacon RSSI", " = ", rRssi);
	}

	/* =========== Group 0x0008 =========== */
	if (u4StatGroup & 0x0008) {
		/* TxV */
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%s", "----- Last TX Info (Group 0x08) -----\n");

		for (ucDbdcIdx = 0; ucDbdcIdx < ENUM_BAND_NUM; ucDbdcIdx++) {
			if (prAdapter->rWifiVar.fgDbDcModeEn)
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"[DBDC_%d] :\n", ucDbdcIdx);

			i4BytesWritten += nicTxGetVectorInfo(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				&prQueryStaStatistics->rTxVector[ucDbdcIdx]);

			if (prQueryStaStatistics->rTxVector[ucDbdcIdx]
			    .u4TxV[0] == 0xFFFFFFFF)
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%s\n", "Chip Out TX Power",
					" = ", "N/A");
			else
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%ld.%1ld dBm\n",
					"Chip Out TX Power", " = ",
					TX_VECTOR_GET_TX_PWR(
					    &prQueryStaStatistics->rTxVector[
						ucDbdcIdx]) >> 1,
					5 * (TX_VECTOR_GET_TX_PWR(
					    &prQueryStaStatistics->rTxVector[
						ucDbdcIdx]) % 2));

			if (!prAdapter->rWifiVar.fgDbDcModeEn)
				break;
		}
	}

	/* =========== Group 0x0010 =========== */
	if (u4StatGroup & 0x0010) {
		/* RX Reorder */
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%s", "----- RX Reorder (Group 0x10) -----\n");
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%llu\n", "Rx reorder miss", " = ",
			RX_GET_CNT(prRxCtrl, RX_DATA_REORDER_MISS_COUNT));
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%llu\n", "Rx reorder within", " = ",
			RX_GET_CNT(prRxCtrl, RX_DATA_REORDER_WITHIN_COUNT));
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%llu\n", "Rx reorder ahead", " = ",
			RX_GET_CNT(prRxCtrl, RX_DATA_REORDER_AHEAD_COUNT));
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%llu\n", "Rx reorder behind", " = ",
			RX_GET_CNT(prRxCtrl, RX_DATA_REORDER_BEHIND_COUNT));
	}

	/* =========== Group 0x0020 =========== */
	if (u4StatGroup & 0x0020) {
		/* RA info */
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%s", "----- RA Info (Group 0x20) -----\n");

		/* Last TX Rate */
		i4BytesWritten += nicGetTxRateInfo(
			pcCommand + i4BytesWritten, i4TotalLen - i4BytesWritten,
			FALSE, prHwWlanInfo, prQueryStaStatistics);

		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten, "%-20s%s%d\n", "LinkSpeed",
			" = ", u2LinkSpeed);

		if (!prQueryStaStatistics->ucSkipAr) {
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%s\n", "RateTable", " = ",
#if (CFG_SUPPORT_RA_GEN == 1)
				prQueryStaStatistics->ucArTableIdx <
				(ucRaTableNum - 1) ?
				RATE_TBLE[prQueryStaStatistics->ucArTableIdx] :
				RATE_TBLE[ucRaTableNum - 1]);
#else
				prQueryStaStatistics->ucArTableIdx < 6 ?
				RATE_TBLE[prQueryStaStatistics->ucArTableIdx] :
				RATE_TBLE[6]);
#endif

			if (wlanGetStaIdxByWlanIdx(prAdapter,
				(uint8_t)(prHwWlanInfo->u4Index), &ucStaIdx) ==
				WLAN_STATUS_SUCCESS) {
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%d\n", "2G Support 256QAM TX",
					" = ",
#if (CFG_SUPPORT_RA_GEN == 1)
					((prAdapter->arStaRec[ucStaIdx].u4Flags
					& MTK_SYNERGY_CAP_SUPPORT_24G_MCS89) ||
					(prQueryStaStatistics->
					ucDynamicGband256QAMState == 2)) ?
					1 : 0);
#else
					(prAdapter->arStaRec[ucStaIdx].u4Flags &
					MTK_SYNERGY_CAP_SUPPORT_24G_MCS89) ?
					1 : 0);
#endif
			}

#if (CFG_SUPPORT_RA_GEN == 0)
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d%%\n", "Rate1 instantPer", " = ",
				u4InstantPer);
#endif

			if (prQueryStaStatistics->ucAvePer == 0xFF) {
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%s\n", "Train Down", " = ",
					"N/A");

				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%s\n", "Train Up", " = ",
					"N/A");
			} else {
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%d -> %d\n", "Train Down",
					" = ",
					(uint16_t)
					(prQueryStaStatistics->u2TrainDown
						& BITS(0, 7)),
					(uint16_t)
					((prQueryStaStatistics->u2TrainDown >>
						8) & BITS(0, 7)));

				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%d -> %d\n", "Train Up", " = ",
					(uint16_t)
					(prQueryStaStatistics->u2TrainUp
						& BITS(0, 7)),
					(uint16_t)
					((prQueryStaStatistics->u2TrainUp >> 8)
						& BITS(0, 7)));
			}

			if (prQueryStaStatistics->fgIsForceTxStream == 0)
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%s\n", "Force Tx Stream",
					" = ", "N/A");
			else
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%-20s%s%d\n", "Force Tx Stream", " = ",
					prQueryStaStatistics->
						fgIsForceTxStream);

			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%d\n", "Force SE off", " = ",
				prQueryStaStatistics->fgIsForceSeOff);

#if (CFG_SUPPORT_RA_GEN == 1)
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%s\n", "LtMode", " = ",
				prQueryStaStatistics->ucLowTrafficMode <
				(ucRaLtModeNum - 1) ?
				LT_MODE_TBLE[prQueryStaStatistics->
				ucLowTrafficMode] :
				LT_MODE_TBLE[ucRaLtModeNum - 1]);

			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%s\n", "SgiState", " = ",
				prQueryStaStatistics->ucDynamicSGIState <
				(ucRaSgiUnSpStateNum - 1) ?
				SGI_UNSP_STATE_TBLE[prQueryStaStatistics->
				ucDynamicSGIState] :
				SGI_UNSP_STATE_TBLE[ucRaSgiUnSpStateNum - 1]);

			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%s\n", "BwState", " = ",
				prQueryStaStatistics->ucDynamicBWState <
				(ucRaBwStateNum - 1) ?
				BW_STATE_TBLE[prQueryStaStatistics->
				ucDynamicBWState] :
				BW_STATE_TBLE[ucRaBwStateNum - 1]);

			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-20s%s%s\n", "NonSpState", " = ",
				prQueryStaStatistics->ucVhtNonSpRateState <
				(ucRaSgiUnSpStateNum - 1) ?
				SGI_UNSP_STATE_TBLE[prQueryStaStatistics->
				ucVhtNonSpRateState] :
				SGI_UNSP_STATE_TBLE[ucRaSgiUnSpStateNum - 1]);
#endif
		}

#if (CFG_SUPPORT_RA_GEN == 1)
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%d\n", "RunningCnt", " = ",
			prQueryStaStatistics->u2RaRunningCnt);

		prQueryStaStatistics->ucRaStatus &= ~0x80;
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%s\n", "Status", " = ",
			prQueryStaStatistics->ucRaStatus < (ucRaStatusNum - 1) ?
			RA_STATUS_TBLE[prQueryStaStatistics->ucRaStatus] :
			RA_STATUS_TBLE[ucRaStatusNum - 1]);

		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%d\n", "MaxAF", " = ",
			prHwWlanInfo->rWtblPeerCap.ucAmpduFactor);

		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s0x%x\n", "SpeIdx", " = ",
			prHwWlanInfo->rWtblPeerCap.ucSpatialExtensionIndex);
#endif
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-20s%s%d\n", "CBRN", " = ",
			prHwWlanInfo->rWtblPeerCap.ucChangeBWAfterRateN);

		/* Rate1~Rate8 */
		i4BytesWritten += nicGetTxRateInfo(
			pcCommand + i4BytesWritten, i4TotalLen - i4BytesWritten,
			TRUE, prHwWlanInfo, prQueryStaStatistics);
	}

	/* =========== Group 0x0040 =========== */
	if (u4StatGroup & 0x0040) {
#if (CFG_SUPPORT_RA_GEN == 1)
		u4RangeCtrl_0 = prQueryStaStatistics->u4AggRangeCtrl_0;
		u4RangeCtrl_1 = prQueryStaStatistics->u4AggRangeCtrl_1;
		eRangeType = (enum AGG_RANGE_TYPE_T)
					prQueryStaStatistics->ucRangeType;

		ucAggRange[0] = (((u4RangeCtrl_0) & AGG_RANGE_SEL_0_MASK) >>
						AGG_RANGE_SEL_0_OFFSET);
		ucAggRange[1] = (((u4RangeCtrl_0) & AGG_RANGE_SEL_1_MASK) >>
						AGG_RANGE_SEL_1_OFFSET);
		ucAggRange[2] = (((u4RangeCtrl_0) & AGG_RANGE_SEL_2_MASK) >>
						AGG_RANGE_SEL_2_OFFSET);
		ucAggRange[3] = (((u4RangeCtrl_0) & AGG_RANGE_SEL_3_MASK) >>
						AGG_RANGE_SEL_3_OFFSET);
		ucAggRange[4] = (((u4RangeCtrl_1) & AGG_RANGE_SEL_4_MASK) >>
						AGG_RANGE_SEL_4_OFFSET);
		ucAggRange[5] = (((u4RangeCtrl_1) & AGG_RANGE_SEL_5_MASK) >>
						AGG_RANGE_SEL_5_OFFSET);
		ucAggRange[6] = (((u4RangeCtrl_1) & AGG_RANGE_SEL_6_MASK) >>
						AGG_RANGE_SEL_6_OFFSET);

		/* Tx Agg */
		i4BytesWritten += kalScnprintf(
			pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%s%s%s", "------ ",
			(eRangeType != 0) ? (
				(eRangeType == ENUM_AGG_RANGE_TYPE_TRX) ?
				("TRX") : ("RX")) : ("TX"),
				" AGG (Group 0x40) -----\n");

		if (eRangeType == ENUM_AGG_RANGE_TYPE_TRX) {
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-6s%8d%5d%1s%2d%5d%1s%2d%5d%3s",
				" TX  :", ucAggRange[0] + 1,
				ucAggRange[0] + 2, "~", ucAggRange[1] + 1,
				ucAggRange[1] + 2, "~", ucAggRange[2] + 1,
				ucAggRange[2] + 2, "~64\n");

			for (ucDbdcIdx = 0; ucDbdcIdx < ENUM_BAND_NUM;
			     ucDbdcIdx++) {
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"DBDC%d:", ucDbdcIdx);
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%8d%8d%8d%8d\n",
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						0],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						1],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						2],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						3]);

				if (!prAdapter->rWifiVar.fgDbDcModeEn)
					break;
			}

			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%-6s%8d%5d%1s%2d%5d%1s%2d%5d%3s",
				" RX  :", ucAggRange[3] + 1,
				ucAggRange[3] + 2, "~", ucAggRange[4] + 1,
				ucAggRange[4] + 2, "~", ucAggRange[5] + 1,
				ucAggRange[5] + 2, "~64\n");

			for (ucDbdcIdx = 0; ucDbdcIdx < ENUM_BAND_NUM;
			     ucDbdcIdx++) {
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"DBDC%d:", ucDbdcIdx);
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%8d%8d%8d%8d\n",
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						4],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						5],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						6],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						7]);

				if (!prAdapter->rWifiVar.fgDbDcModeEn)
					break;
			}
		} else {
			i4BytesWritten += kalScnprintf(
			pcCommand + i4BytesWritten, i4TotalLen - i4BytesWritten,
				"%-6s%8d%5d%1s%2d%5d%1s%2d%5d%1s%2d%5d%1s%2d%5d%1s%2d%5d%1s%2d%5d%3s",
				"Range:", ucAggRange[0] + 1,
				ucAggRange[0] + 2, "~", ucAggRange[1] + 1,
				ucAggRange[1] + 2, "~", ucAggRange[2] + 1,
				ucAggRange[2] + 2, "~", ucAggRange[3] + 1,
				ucAggRange[3] + 2, "~", ucAggRange[4] + 1,
				ucAggRange[4] + 2, "~", ucAggRange[5] + 1,
				ucAggRange[5] + 2, "~", ucAggRange[6] + 1,
				ucAggRange[6] + 2, "~64\n");

			for (ucDbdcIdx = 0; ucDbdcIdx < ENUM_BAND_NUM;
			     ucDbdcIdx++) {
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"DBDC%d:", ucDbdcIdx);
				i4BytesWritten += kalScnprintf(
					pcCommand + i4BytesWritten,
					i4TotalLen - i4BytesWritten,
					"%8d%8d%8d%8d%8d%8d%8d%8d\n",
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						0],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						1],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						2],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						3],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						4],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						5],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						6],
					prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						7]);

				if (!prAdapter->rWifiVar.fgDbDcModeEn)
					break;
			}
		}
#else
		/* Tx Agg */
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%s", "------ TX AGG (Group 0x40) -----\n");
		i4BytesWritten += kalScnprintf(pcCommand + i4BytesWritten,
			i4TotalLen - i4BytesWritten,
			"%-12s%s", "Range:",
			"1     2~5     6~15    16~22    23~33    34~49    50~57    58~64\n");
		for (ucDbdcIdx = 0; ucDbdcIdx < ENUM_BAND_NUM; ucDbdcIdx++) {
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"DBDC%d:", ucDbdcIdx);
			i4BytesWritten += kalScnprintf(
				pcCommand + i4BytesWritten,
				i4TotalLen - i4BytesWritten,
				"%7d%8d%9d%9d%9d%9d%9d%9d\n",
				prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
					0],
				prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						1],
				prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						2],
				prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						3],
				prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						4],
				prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						5],
				prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						6],
				prQueryStaStatistics->
					rMibInfo[ucDbdcIdx].au2TxRangeAmpduCnt[
						7]);
			if (!prAdapter->rWifiVar.fgDbDcModeEn)
				break;
		}
#endif
	}

	return i4BytesWritten;
}

#ifdef CFG_SUPPORT_LINK_QUALITY_MONITOR
int connac_get_rx_rate_info(IN struct ADAPTER *prAdapter,
		IN uint8_t ucBssIdx,
		OUT uint32_t *pu4Rate, OUT uint32_t *pu4Nss,
		OUT uint32_t *pu4RxMode, OUT uint32_t *pu4FrMode,
		OUT uint32_t *pu4Sgi)
{
	struct STA_RECORD *prStaRec;
	uint32_t rxmode = 0, rate = 0, frmode = 0, sgi = 0, nsts = 0;
	uint32_t groupid = 0, stbc = 0, nss = 0;
	uint32_t u4RxVector0 = 0, u4RxVector1 = 0;
	uint8_t ucWlanIdx, ucStaIdx;

	if ((!pu4Rate) || (!pu4Nss) || (!pu4RxMode) || (!pu4FrMode) ||
		(!pu4Sgi))
		return -1;

	prStaRec = aisGetStaRecOfAP(prAdapter, ucBssIdx);
	if (prStaRec) {
		ucWlanIdx = prStaRec->ucWlanIndex;
	} else {
		//DBGLOG(SW4, ERROR, "prStaRecOfAP is null\n");
		return -1;
	}

	if (wlanGetStaIdxByWlanIdx(prAdapter, ucWlanIdx, &ucStaIdx) ==
		WLAN_STATUS_SUCCESS) {
		u4RxVector0 = prAdapter->arStaRec[ucStaIdx].u4RxVector0;
		u4RxVector1 = prAdapter->arStaRec[ucStaIdx].u4RxVector1;
		if ((u4RxVector0 == 0) || (u4RxVector1 == 0)) {
			DBGLOG(SW4, WARN, "RxVector1 or RxVector2 is 0\n");
			return -1;
		}
	} else {
		DBGLOG(SW4, ERROR, "wlanGetStaIdxByWlanIdx fail\n");
		return -1;
	}

	rate = (u4RxVector0 & RX_VT_RX_RATE_MASK) >> RX_VT_RX_RATE_OFFSET;
	nsts = ((u4RxVector1 & RX_VT_NSTS_MASK) >> RX_VT_NSTS_OFFSET);
	stbc = ((u4RxVector0 & RX_VT_STBC_MASK) >> RX_VT_STBC_OFFSET);
	rxmode = (u4RxVector0 & RX_VT_RX_MODE_MASK) >> RX_VT_RX_MODE_OFFSET;
	frmode = (u4RxVector0 & RX_VT_FR_MODE_MASK) >> RX_VT_FR_MODE_OFFSET;
	sgi = u4RxVector0 & RX_VT_SHORT_GI;
	groupid = (u4RxVector1 & RX_VT_GROUP_ID_MASK) >> RX_VT_GROUP_ID_OFFSET;

	if ((groupid == 0) || (groupid == 63))
		nsts += 1;

	nss = stbc ? (nsts >> 1) : nsts;

	sgi = (sgi == 0) ? 0 : 1;

	if (frmode >= 4) {
		DBGLOG(SW4, ERROR, "frmode error: %u\n", frmode);
		return -1;
	}

	*pu4Rate = rate;
	*pu4Nss = nss;
	*pu4RxMode = rxmode;
	*pu4FrMode = frmode;
	*pu4Sgi = sgi;

	DBGLOG(SW4, TRACE,
		   "rxmode=[%u], rate=[%u], bw=[%u], sgi=[%u], nss=[%u]\n",
		   rxmode, rate, frmode, sgi, nss
	);

	return 0;
}
#endif

