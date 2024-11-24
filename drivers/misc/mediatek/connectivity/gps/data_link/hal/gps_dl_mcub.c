/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */

#include "gps_dl_config.h"
#include "gps_dl_log.h"
#include "gps_dl_hw_api.h"
#include "gps_dl_hw_dep_macro.h"
#include "gps_dsp_fsm.h"
#include "gps_dl_hal.h"

const unsigned int c_gps_dsp_reg_list[GPS_DATA_LINK_NUM][GPS_DSP_REG_POLL_MAX] = {
	GPS_L1_REG_POLL_LIST,
	GPS_L5_REG_POLL_LIST
};

const unsigned int c_gps_dsp_reg_dbg_list[GPS_DATA_LINK_NUM][GPS_DSP_REG_DBG_POLL_MAX] = {
	GPS_L1_REG_DBG_POLL_LIST,
	GPS_L5_REG_DBG_POLL_LIST
};

struct gps_each_dsp_reg_read_context {
	bool poll_ongoing;
	int poll_index;
	unsigned int poll_addr;

	/* a "poll" means one register in c_gps_dsp_reg_list.
	 * a "round" means a round to poll all registers in c_gps_dsp_reg_list.
	 * sometimes for debug we need for several rounds
	 * to check the changing of the values of each register.
	 */
	unsigned int round_max;
	unsigned int round_index;
	const unsigned int *poll_list_ptr;
	int poll_list_len;
};

struct gps_each_dsp_reg_read_context g_gps_each_dsp_reg_read_context[GPS_DATA_LINK_NUM];

struct gps_each_dsp_reg_read_value g_gps_rec_dsp_reg[GPS_DATA_LINK_NUM];

enum GDL_RET_STATUS gps_each_dsp_reg_read_request(
	enum gps_dl_link_id_enum link_id, unsigned int reg_addr)
{
	enum GDL_RET_STATUS ret;

	ASSERT_LINK_ID(link_id, GDL_FAIL_INVAL);
	ret = gps_dl_hw_mcub_dsp_read_request(link_id, reg_addr);

	if (ret == GDL_OKAY) {
		g_gps_each_dsp_reg_read_context[link_id].poll_addr = reg_addr;
		g_gps_each_dsp_reg_read_context[link_id].poll_ongoing = true;
	}

	return ret;
}

void gps_each_dsp_reg_gourp_read_next(enum gps_dl_link_id_enum link_id, bool restart)
{
	unsigned int reg_addr;
	enum GDL_RET_STATUS ret;
	struct gps_each_dsp_reg_read_context *p_read_context = NULL;
	int i;

	ASSERT_LINK_ID(link_id, GDL_VOIDF());
	p_read_context = &g_gps_each_dsp_reg_read_context[link_id];
	if (restart) {
		p_read_context->poll_index = 0;
		p_read_context->round_index = 0;
	} else {
		p_read_context->poll_index++;
		if (p_read_context->poll_index >= p_read_context->poll_list_len) {
			p_read_context->round_index++;
			if (p_read_context->round_index >= p_read_context->round_max) {
				/* all polling end */
				return;
			}
			/* next round */
			p_read_context->poll_index = 0;
		}
	}

	i = p_read_context->poll_index;
	reg_addr = p_read_context->poll_list_ptr[i];
	ret = gps_each_dsp_reg_read_request(link_id, reg_addr);
	GDL_LOGXD(link_id, "i = %d/%d, addr = 0x%04x, status = %s",
		i, p_read_context->poll_list_len, reg_addr, gdl_ret_to_name(ret));
}

void gps_each_dsp_reg_read_ack(
	enum gps_dl_link_id_enum link_id, const struct gps_dl_hal_mcub_info *p_d2a)
{
	struct gps_each_dsp_reg_read_context *p_read_context = NULL;
	struct gps_each_dsp_reg_read_value *p_read_value = NULL;

	ASSERT_LINK_ID(link_id, GDL_VOIDF());

	p_read_context = &g_gps_each_dsp_reg_read_context[link_id];
	p_read_value = &g_gps_rec_dsp_reg[link_id];

	if (p_read_value->record_d2a_index >= GPS_DSP_REG_DBG_POLL_MAX) {
		GDL_LOGXW(link_id, "warning : poll_index : %d, write_index : %d, REG_DBG_POLL_MAX : %d",
			p_read_context->poll_index, p_read_value->record_d2a_index, GPS_DSP_REG_DBG_POLL_MAX + 1);
	} else {
		p_read_value->g_gps_rec_dsp_value[p_read_value->record_d2a_index] = p_d2a->dat0;
		p_read_value->record_d2a_index++;

		if (p_read_value->record_d2a_index >= p_read_context->poll_list_len) {
			p_read_value->g_gps_rec_dsp_value[p_read_value->record_d2a_index] = p_d2a->dat1;
			gps_dl_hal_show_dsp_reg("dsp_reg_normal", link_id, p_read_value->g_gps_rec_dsp_value,
				p_read_value->record_d2a_index + 1);
			p_read_value->record_d2a_index = 0;
		}
	}
	g_gps_each_dsp_reg_read_context[link_id].poll_ongoing = false;
	gps_each_dsp_reg_gourp_read_next(link_id, false);
}

void gps_each_dsp_reg_dump_if_any_rec(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_dsp_reg_read_value *p_read_value = NULL;

	ASSERT_LINK_ID(link_id, GDL_VOIDF());

	p_read_value = &g_gps_rec_dsp_reg[link_id];
	gps_dl_hal_show_dsp_reg("dsp_reg_force_dump", link_id, p_read_value->g_gps_rec_dsp_value,
		p_read_value->record_d2a_index);
	p_read_value->record_d2a_index = 0;
}

void gps_each_dsp_reg_gourp_read_start(enum gps_dl_link_id_enum link_id,
	bool dbg, unsigned int round_max)
{
	unsigned int a2d_flag;
	struct gps_dl_hal_mcub_info d2a;

	ASSERT_LINK_ID(link_id, GDL_VOIDF());

	if (g_gps_each_dsp_reg_read_context[link_id].poll_ongoing) {
		GDL_LOGXW(link_id, "n = %d/%d, addr = 0x%04x, seem busy, check it",
			g_gps_each_dsp_reg_read_context[link_id].poll_index + 1,
			g_gps_each_dsp_reg_read_context[link_id].poll_list_len,
			g_gps_each_dsp_reg_read_context[link_id].poll_addr);

		/* TODO: show hw status */
		a2d_flag = gps_dl_hw_get_mcub_a2d_flag(link_id);
		gps_dl_hw_get_mcub_info(link_id, &d2a);
		GDL_LOGXW(link_id, "a2d_flag = %d, d2a_flag = %d, d0 = 0x%04x, d1 = 0x%04x",
			a2d_flag, d2a.flag, d2a.dat0, d2a.dat1);

		if (a2d_flag & GPS_MCUB_A2DF_MASK_DSP_REG_READ_REQ ||
			d2a.flag & GPS_MCUB_D2AF_MASK_DSP_REG_READ_READY) {
			/* real busy, bypass */
			return;
		}
	}

	if (dbg) {
		g_gps_each_dsp_reg_read_context[link_id].poll_list_ptr =
			&c_gps_dsp_reg_dbg_list[link_id][0];
		g_gps_each_dsp_reg_read_context[link_id].poll_list_len = GPS_DSP_REG_DBG_POLL_MAX;
	} else {
		g_gps_each_dsp_reg_read_context[link_id].poll_list_ptr =
			&c_gps_dsp_reg_list[link_id][0];
		g_gps_each_dsp_reg_read_context[link_id].poll_list_len = GPS_DSP_REG_POLL_MAX;
	}
	g_gps_each_dsp_reg_read_context[link_id].round_max = round_max;
	gps_each_dsp_reg_gourp_read_next(link_id, true);
}

void gps_each_dsp_reg_gourp_read_init(enum gps_dl_link_id_enum link_id)
{
	ASSERT_LINK_ID(link_id, GDL_VOIDF());

	memset(&g_gps_each_dsp_reg_read_context[link_id], 0,
		sizeof(g_gps_each_dsp_reg_read_context[link_id]));
}

void gps_dl_hal_show_dsp_reg(unsigned char *tag, enum gps_dl_link_id_enum link_id,
	unsigned int *buf, unsigned int len)
{
	int base = 0, line_idx = 0;
	int line_len = 8;
	int left_len = len;

#define SHOW_BUF_FMT0 "[%s] len = %u"
#define SHOW_BUF_FMT1 SHOW_BUF_FMT0", data = %04x"
#define SHOW_BUF_FMT2 SHOW_BUF_FMT1" %04x"
#define SHOW_BUF_FMT3 SHOW_BUF_FMT2" %04x"
#define SHOW_BUF_FMT4 SHOW_BUF_FMT3" %04x"
#define SHOW_BUF_FMT5 SHOW_BUF_FMT4", %04x"
#define SHOW_BUF_FMT6 SHOW_BUF_FMT5" %04x"
#define SHOW_BUF_FMT7 SHOW_BUF_FMT6" %04x"
#define SHOW_BUF_FMT8 SHOW_BUF_FMT7" %04x"

#define SHOW_BUF_ARG0 do {GDL_LOGI(SHOW_BUF_FMT0, tag, len); } while (0)

#define SHOW_BUF_ARG1 do {GDL_LOGI(SHOW_BUF_FMT1, tag, len, buf[base+0]); } while (0)

#define SHOW_BUF_ARG2 do {GDL_LOGI(SHOW_BUF_FMT2, tag, len, buf[base+0], buf[base+1]); } while (0)

#define SHOW_BUF_ARG3 do {GDL_LOGI(SHOW_BUF_FMT3, tag, len, buf[base+0], buf[base+1], buf[base+2]) \
	; } while (0)

#define SHOW_BUF_ARG4 do {GDL_LOGI(SHOW_BUF_FMT4, tag, len, buf[base+0], buf[base+1], buf[base+2], \
	buf[base+3]); } while (0)

#define SHOW_BUF_ARG5 do {GDL_LOGI(SHOW_BUF_FMT5, tag, len, buf[base+0], buf[base+1], buf[base+2], \
	buf[base+3], buf[base+4]); } while (0)

#define SHOW_BUF_ARG6 do {GDL_LOGI(SHOW_BUF_FMT6, tag, len, buf[base+0], buf[base+1], buf[base+2], \
	buf[base+3], buf[base+4], buf[base+5]); } while (0)

#define SHOW_BUF_ARG7 do {GDL_LOGI(SHOW_BUF_FMT7, tag, len, buf[base+0], buf[base+1], buf[base+2], \
	buf[base+3], buf[base+4], buf[base+5], buf[base+6]); } while (0)

#define SHOW_BUF_ARG8 do {GDL_LOGI(SHOW_BUF_FMT8, tag, len, buf[base+0], buf[base+1], buf[base+2], \
	buf[base+3], buf[base+4], buf[base+5], buf[base+6], buf[base+7]); } while (0)

	for (left_len = len, base = 0, line_idx = 0;
		left_len > 0 && line_idx < 3;
		left_len -= 8, base += 8, line_idx++) {

		if (left_len > 8)
			line_len = 8;
		else
			line_len = left_len;

		switch (line_len) {
		/* case 0 is impossible */
#if 0
		case 0:
			SHOW_BUF_ARG0; break;
#endif
		case 1:
			SHOW_BUF_ARG1; break;
		case 2:
			SHOW_BUF_ARG2; break;
		case 3:
			SHOW_BUF_ARG3; break;
		case 4:
			SHOW_BUF_ARG4; break;
		case 5:
			SHOW_BUF_ARG5; break;
		case 6:
			SHOW_BUF_ARG6; break;
		case 7:
			SHOW_BUF_ARG7; break;
		default:
			SHOW_BUF_ARG8; break;
		}
	}
}

