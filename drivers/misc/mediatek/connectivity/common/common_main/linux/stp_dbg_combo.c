/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "stp_dbg.h"
#include "stp_dbg_combo.h"
#include "stp_sdio.h"
#include "stp_core.h"

static _osal_inline_ INT32 stp_dbg_combo_put_dump_to_aee(VOID);
static _osal_inline_ INT32 stp_dbg_combo_put_dump_to_nl(VOID);

static PUINT8 combo_task_str[STP_DBG_TASK_ID_MAX] = {
	"Task_WMT",
	"Task_BT",
	"Task_Wifi",
	"Task_Tst",
	"Task_FM",
	"Task_GPS",
	"Task_FLP",
	"Task_BAL",
	"Task_Idle",
	"Task_DrvStp",
	"Task_DrvSdio",
	"Task_NatBt",
	"Task_DrvWifi",
	"Task_GPS"
};

INT32 const combo_legacy_task_id_adapter[STP_DBG_TASK_ID_MAX] = {
	STP_DBG_TASK_WMT,
	STP_DBG_TASK_BT,
	STP_DBG_TASK_WIFI,
	STP_DBG_TASK_TST,
	STP_DBG_TASK_FM,
	STP_DBG_TASK_IDLE,
	STP_DBG_TASK_WMT,
	STP_DBG_TASK_WMT,
	STP_DBG_TASK_WMT,
	STP_DBG_TASK_DRVSTP,
	STP_DBG_TASK_BUS,
	STP_DBG_TASK_NATBT,
	STP_DBG_TASK_DRVWIFI,
	STP_DBG_TASK_DRVGPS
};

static _osal_inline_ INT32 stp_dbg_combo_put_dump_to_aee(VOID)
{
	static UINT8 buf[2048];
	static UINT8 tmp[2048];

	UINT32 buf_len;
	STP_PACKET_T *pkt = NULL;
	STP_DBG_HDR_T *hdr = NULL;
	INT32 remain = 0;
	INT32 retry = 0;
	INT32 ret = 0;

	do {
		remain = stp_dbg_dmp_out_ex(&buf[0], &buf_len);
		if (buf_len > 0) {
			pkt = (STP_PACKET_T *) buf;
			hdr = &pkt->hdr;
			if (hdr->dbg_type == STP_DBG_FW_DMP) {
				if (pkt->hdr.len <= 1500) {
					tmp[pkt->hdr.len] = '\n';
					tmp[pkt->hdr.len + 1] = '\0';
					if (pkt->hdr.len < STP_DMP_SZ)
						osal_memcpy(&tmp[0], pkt->raw, pkt->hdr.len);
					else
						osal_memcpy(&tmp[0], pkt->raw, STP_DMP_SZ);
					ret = stp_dbg_aee_send(tmp, pkt->hdr.len, 0);
				} else {
					STP_DBG_PR_INFO("dump entry length is over long\n");
					osal_bug_on(0);
				}
				retry = 0;
			}
			retry = 0;
		} else {
			retry++;
			osal_sleep_ms(20);
		}
	} while ((remain > 0) || (retry < 10));

	return ret;
}

static _osal_inline_ INT32 stp_dbg_combo_put_dump_to_nl(VOID)
{
#define NUM_FETCH_ENTRY 8

	static UINT8 buf[2048];
	static UINT8 tmp[2048];

	UINT32 buf_len;
	STP_PACKET_T *pkt = NULL;
	STP_DBG_HDR_T *hdr = NULL;
	INT32 remain = 0;
	INT32 index = 0;
	INT32 retry = 0;
	INT32 ret = 0;
	INT32 len;

	index = 0;
	tmp[index++] = '[';
	tmp[index++] = 'M';
	tmp[index++] = ']';

	do {
		index = 3;
		remain = stp_dbg_dmp_out_ex(&buf[0], &buf_len);
		if (buf_len > 0) {
			pkt = (STP_PACKET_T *) buf;
			hdr = &pkt->hdr;
			len = pkt->hdr.len;
			osal_memcpy(&tmp[index], &len, 2);
			index += 2;
			if (hdr->dbg_type == STP_DBG_FW_DMP) {
				osal_memcpy(&tmp[index], pkt->raw, pkt->hdr.len);

				if (pkt->hdr.len <= 1500) {
					tmp[index + pkt->hdr.len] = '\n';
					tmp[index + pkt->hdr.len + 1] = '\0';

					/* pr_warn("\n%s\n+++\n", tmp); */
					ret = stp_dbg_dump_send_retry_handler((PINT8)&tmp, len);
					if (ret)
						break;

					/* schedule(); */
				} else {
					STP_DBG_PR_INFO("dump entry length is over long\n");
					osal_bug_on(0);
				}
				retry = 0;
			}
		} else {
			retry++;
			osal_sleep_ms(100);
		}
	} while ((remain > 0) || (retry < 2));

	return ret;
}

INT32 stp_dbg_combo_core_dump(INT32 dump_sink)
{
	INT32 ret = 0;

	switch (dump_sink) {
	case 0:
		STP_DBG_PR_INFO("coredump is disabled!\n");
		break;
	case 1:
		ret = stp_dbg_combo_put_dump_to_aee();
		break;
	case 2:
		ret = stp_dbg_combo_put_dump_to_nl();
		break;
	default:
		ret = -1;
		STP_DBG_PR_ERR("unknown sink %d\n", dump_sink);
	}

	return ret;
}

PUINT8 stp_dbg_combo_id_to_task(UINT32 id)
{
	UINT32 chip_id = mtk_wcn_wmt_chipid_query();
	UINT32 temp_id;

	if (id >= STP_DBG_TASK_ID_MAX) {
		STP_DBG_PR_ERR("task id(%d) overflow(%d)\n", id, STP_DBG_TASK_ID_MAX);
		return NULL;
	}

	switch (chip_id) {
	case 0x6632:
		temp_id = id;
		break;
	default:
		temp_id = combo_legacy_task_id_adapter[id];
		break;
	}

	return combo_task_str[temp_id];
}

INT32 stp_dbg_combo_poll_cpupcr(UINT32 times, UINT32 sleep, UINT32 cmd)
{
	INT32 i = 0;
	UINT32 value = 0x0;
	INT32 i_ret = 0;
	INT32 count = 0;
	INT32 chip_id = -1;
	UINT8 cccr_value = 0x0;
	UINT32 buffer[STP_DBG_CPUPCR_NUM];
	UINT64 sec_buffer[STP_DBG_CPUPCR_NUM];
	ULONG nsec_buffer[STP_DBG_CPUPCR_NUM];

	if (times > STP_DBG_CPUPCR_NUM)
		times = STP_DBG_CPUPCR_NUM;

	for (i = 0; i < times; i++) {
		stp_sdio_rw_retry(HIF_TYPE_READL, STP_SDIO_RETRY_LIMIT,
				g_stp_sdio_host_info.sdio_cltctx, SWPCDBGR, &value, 0);
		buffer[i] = value;
		osal_get_local_time(&(sec_buffer[i]),
				&(nsec_buffer[i]));
		if (sleep > 0)
			osal_sleep_ms(sleep);
	}

	if (cmd) {
		UINT8 str[DBG_LOG_STR_SIZE] = {""};
		PUINT8 p = str;
		INT32 str_len = 0;

		for (i = 0; i < STP_DBG_CPUPCR_NUM; i++) {
			if (sec_buffer[i] == 0 && nsec_buffer[i] == 0)
				continue;

			count++;
			if (count % 4 != 0) {
				str_len = osal_sprintf(p, "%llu.%06lu/0x%08x;",
						       sec_buffer[i],
						       nsec_buffer[i],
						       buffer[i]);
				p += str_len;
			} else {
				str_len = osal_sprintf(p, "%llu.%06lu/0x%08x;",
						       sec_buffer[i],
						       nsec_buffer[i],
						       buffer[i]);
				STP_DBG_PR_INFO("TIME/CPUPCR: %s\n", str);
				p = str;
			}
		}
		if (count % 4 != 0)
			STP_DBG_PR_INFO("TIME/CPUPCR: %s\n", str);

		chip_id = mtk_wcn_wmt_chipid_query();
		if (chip_id == 0x6632) {
			for (i = 0; i < 8; i++) {
				i_ret = mtk_wcn_hif_sdio_f0_readb(g_stp_sdio_host_info.sdio_cltctx,
						CCCR_F8 + i, &cccr_value);
				if (i_ret)
					STP_DBG_PR_ERR("read CCCR fail(%d), address(0x%x)\n",
							i_ret, CCCR_F8 + i);
				else
					STP_DBG_PR_INFO("read CCCR value(0x%x), address(0x%x)\n",
							cccr_value, CCCR_F8 + i);
				cccr_value = 0x0;
			}
		}
	}
	STP_DBG_PR_INFO("dump sdio register for debug\n");
	mtk_stp_dump_sdio_register();

	return 0;
}
