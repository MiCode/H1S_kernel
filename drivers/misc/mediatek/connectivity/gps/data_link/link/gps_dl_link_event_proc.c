/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"
#include "gps_each_link.h"

#include "gps_dl_name_list.h"
#include "gps_dl_time_tick.h"
#include "gps_dl_hal.h"
#include "gps_dl_hal_api.h"
#include "gps_dl_hal_util.h"
#include "gps_dl_hw_api.h"
#include "gps_dl_isr.h"
#include "gps_dl_lib_misc.h"
#include "gps_dsp_fsm.h"
#include "gps_dl_osal.h"
#include "gps_dl_context.h"
#include "gps_dl_subsys_reset.h"
#if GPS_DL_HAS_PLAT_DRV
#include "gps_dl_linux_plat_drv.h"
#endif


void gps_dl_link_event_send(enum gps_dl_link_event_id evt,
	enum gps_dl_link_id_enum link_id)
{
#if GPS_DL_HAS_CTRLD
	struct gps_dl_osal_lxop *pOp = NULL;
	struct gps_dl_osal_signal *pSignal = NULL;
	int iRet;

	pOp = gps_dl_get_free_op();
	if (!pOp)
		return;

	pSignal = &pOp->signal;
	pSignal->timeoutValue = 0;/* send data need to wait ?ms */
	if (link_id < GPS_DATA_LINK_NUM) {
		pOp->op.opId = GPS_DL_OPID_LINK_EVENT_PROC;
		pOp->op.au4OpData[0] = link_id;
		pOp->op.au4OpData[1] = evt;
		pOp->op.op_enq = gps_dl_tick_get_ms();
		iRet = gps_dl_put_act_op(pOp);
	} else {
		gps_dl_put_op_to_free_queue(pOp);
		/*printf error msg*/
		return;
	}
#else
	gps_dl_link_event_proc(evt, link_id);
#endif
}

void gps_dl_link_event_proc(enum gps_dl_link_event_id evt,
	enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p_link = gps_dl_link_get(link_id);
	bool show_log = false;
	bool show_log2 = false;
	unsigned long j0, j1;
	int ret;
	enum gps_dsp_state_t dsp_state;

	j0 = gps_dl_tick_get();
	GDL_LOGXD_EVT(link_id, "evt = %s", gps_dl_link_event_name(evt));

	switch (evt) {
	case GPS_DL_EVT_LINK_OPEN:
		/* show_log = gps_dl_set_show_reg_rw_log(true); */
		gps_each_dsp_reg_gourp_read_init(link_id);
		gps_each_link_inc_session_id(link_id);
		gps_each_link_set_active(link_id, true);
		gps_each_link_set_bool_flag(link_id, LINK_NEED_A2Z_DUMP, false);

		ret = gps_dl_hal_conn_power_ctrl(link_id, 1);
		if (ret != 0) {
			gps_dl_link_open_ack(link_id, false, false);
			break;
		}

		ret = gps_dl_hal_link_power_ctrl(link_id, GPS_DL_HAL_POWER_ON);
		if (ret != 0) {
			gps_dl_hal_conn_power_ctrl(link_id, 0);
			gps_dl_link_open_ack(link_id, false, false);
			break;
		}

		gps_dsp_fsm(GPS_DSP_EVT_FUNC_ON, link_id);
		gps_dl_link_irq_set(link_id, true);
#if GPS_DL_NO_USE_IRQ
		gps_dl_wait_us(1000); /* wait 1ms */
#endif
		/* set ready to write before open ack, otherwise need to check pending tx data
		 * gps_dl_link_set_ready_to_write(link_id, true);
		 * move it to DSP reset done handler
		 */
		gps_dl_link_open_ack(link_id, true, false); /* TODO: ack on DSP reset done */
		/* gps_dl_set_show_reg_rw_log(show_log); */
		break;
	case GPS_DL_EVT_LINK_LEAVE_DPSTOP:
	case GPS_DL_EVT_LINK_LEAVE_DPSTOP2:
		/*leave deep stop mode with incorrect status*/
		if (GPS_DSP_ST_WAKEN_UP == gps_dsp_state_get(link_id)) {
			GDL_LOGXE(link_id, "not leave stop mode correct due to dsp state keep wakeup");
			gps_dl_link_open_ack(link_id, true, true);
			break;
		}

		gps_dl_link_pre_leave_dpstop_setting(link_id);
		gps_each_dsp_reg_gourp_read_init(link_id);
		gps_each_link_inc_session_id(link_id);
		gps_each_link_set_active(link_id, true);
		if (evt == GPS_DL_EVT_LINK_LEAVE_DPSTOP2)
			gps_dl_hal_set_deep_stop_mode_revert_for_mvcd(link_id, true);
		gps_each_link_set_bool_flag(link_id, LINK_NEED_A2Z_DUMP, false);
		ret = gps_dl_hal_link_power_ctrl(link_id, GPS_DL_HAL_LEAVE_DPSTOP);
		if (ret != 0)
			gps_dl_link_open_ack(link_id, false, true);
		else {
			gps_dsp_fsm(GPS_DSP_EVT_HW_STOP_EXIT, link_id);
			gps_dl_link_irq_set(link_id, true);
			gps_dl_link_open_ack(link_id, true, true);
		}
		if (evt == GPS_DL_EVT_LINK_LEAVE_DPSTOP2)
			gps_dl_hal_set_deep_stop_mode_revert_for_mvcd(link_id, false);
		break;
	case GPS_DL_EVT_LINK_LEAVE_DPSLEEP:
		gps_dl_hal_link_power_ctrl(link_id, GPS_DL_HAL_LEAVE_DPSLEEP);
		gps_dl_link_irq_set(link_id, true);
		break;
	case GPS_DL_EVT_LINK_ENTER_DPSLEEP:
		gps_dl_link_pre_off_setting(link_id);
		gps_dl_hal_link_power_ctrl(link_id, GPS_DL_HAL_ENTER_DPSLEEP);
		break;
	case GPS_DL_EVT_LINK_ENTER_DPSTOP:
		dsp_state = gps_dsp_state_get(link_id);
		if ((GPS_DSP_ST_WORKING != dsp_state) && (GPS_DSP_ST_RESET_DONE != dsp_state)) {
			/* TODO: ever working check */
			GDL_LOGXE(link_id, "not enter dpstop due to dsp state = %s",
				gps_dl_dsp_state_name(dsp_state));

			/* TODO: ack fail */
			gps_dl_link_close_ack(link_id, true);
			break;
		}

		if (GPS_DSP_ST_WORKING == dsp_state) {
			GDL_LOGXW(link_id, "enter dpstop with dsp state = %s",
				gps_dl_dsp_state_name(dsp_state));
		}

		gps_dl_hal_set_need_clk_ext_flag(link_id,
			gps_each_link_get_bool_flag(link_id, LINK_SUSPEND_TO_CLK_EXT));

		gps_dl_link_pre_off_setting(link_id);
		/* TODO: handle fail */
		gps_dl_hal_link_power_ctrl(link_id, GPS_DL_HAL_ENTER_DPSTOP);
		gps_dsp_fsm(GPS_DSP_EVT_HW_STOP_REQ, link_id);
		gps_each_link_context_clear(link_id);
#if GPS_DL_ON_LINUX
		gps_dma_buf_reset(&p_link->tx_dma_buf);
		gps_dma_buf_reset(&p_link->rx_dma_buf);
#endif
		gps_dl_link_close_ack(link_id, true);

		gps_dl_link_post_enter_dpstop_setting(link_id);
		break;
	case GPS_DL_EVT_LINK_DSP_ROM_READY_TIMEOUT:
		/* check again mcub not ready triggered */
		if (false)
			break; /* wait hal handle it */

		/* true: */
		if (!gps_each_link_change_state_from(link_id, LINK_OPENED, LINK_RESETTING)) {
			/* no handle it again */
			break;
		}
		/* TODO: go and do close */
	case GPS_DL_EVT_LINK_CLOSE:
	case GPS_DL_EVT_LINK_RESET_DSP:
	case GPS_DL_EVT_LINK_RESET_GPS:
	case GPS_DL_EVT_LINK_PRE_CONN_RESET:
		if (evt != GPS_DL_EVT_LINK_CLOSE)
			show_log = gps_dl_set_show_reg_rw_log(true);

		/* handle open fail case */
		if (!gps_each_link_get_bool_flag(link_id, LINK_OPEN_RESULT_OKAY)) {
			GDL_LOGXD(link_id, "not open okay, just power off for %s",
				gps_dl_link_event_name(evt));

			gps_each_link_set_active(link_id, false);
			gps_dl_hal_link_power_ctrl(link_id, GPS_DL_HAL_POWER_OFF);
			gps_dl_hal_conn_power_ctrl(link_id, 0);
			goto _close_or_reset_ack;
		}

		/* to avoid twice enter */
		if (GPS_DSP_ST_OFF == gps_dsp_state_get(link_id)) {
			GDL_LOGXD(link_id, "dsp state is off, do nothing for %s",
				gps_dl_link_event_name(evt));

			if (evt != GPS_DL_EVT_LINK_CLOSE)
				gps_dl_set_show_reg_rw_log(show_log);

			goto _close_or_reset_ack;
		} else if (GPS_DSP_ST_HW_STOP_MODE == gps_dsp_state_get(link_id)) {
			/* exit deep stop mode and turn off it
			 * before exit deep stop, need clear pwr stat to make sure dsp is in hold-on state
			 * after exit deep stop mode.
			 */
			gps_dl_link_pre_leave_dpstop_setting(link_id);
			gps_dl_hal_link_clear_hw_pwr_stat(link_id);
			gps_dl_hal_link_power_ctrl(link_id, GPS_DL_HAL_LEAVE_DPSTOP);
		} else {
			/* make sure current link's DMAs are stopped and mask the IRQs */
			gps_dl_link_pre_off_setting(link_id);
		}
		gps_dl_hal_set_need_clk_ext_flag(link_id, false);

		if (evt != GPS_DL_EVT_LINK_CLOSE) {
			/* try to dump host csr info if not normal close operation */
			if (gps_dl_conninfra_is_okay_or_handle_it(NULL, true))
				gps_dl_hw_dump_host_csr_gps_info(true);
		}

		if (gps_each_link_get_bool_flag(link_id, LINK_NEED_A2Z_DUMP)) {
			show_log2 = gps_dl_set_show_reg_rw_log(true);
			gps_dl_hw_do_gps_a2z_dump();
			gps_dl_set_show_reg_rw_log(show_log2);
		}

		gps_dl_hal_link_power_ctrl(link_id, GPS_DL_HAL_POWER_OFF);
		gps_dl_hal_conn_power_ctrl(link_id, 0);

		gps_dsp_fsm(GPS_DSP_EVT_FUNC_OFF, link_id);

		gps_each_link_context_clear(link_id);
#if GPS_DL_ON_LINUX
		gps_dma_buf_reset(&p_link->tx_dma_buf);
		gps_dma_buf_reset(&p_link->rx_dma_buf);
#endif

_close_or_reset_ack:
		if (evt != GPS_DL_EVT_LINK_CLOSE)
			gps_dl_set_show_reg_rw_log(show_log);

		if (GPS_DL_EVT_LINK_CLOSE == evt)
			gps_dl_link_close_ack(link_id, false); /* TODO: check fired race */
		else
			gps_dl_link_reset_ack(link_id);
		break;

	case GPS_DL_EVT_LINK_POST_CONN_RESET:
		gps_dl_link_on_post_conn_reset(link_id);
		break;

	case GPS_DL_EVT_LINK_WRITE:
		/* gps_dl_hw_print_usrt_status(link_id); */
		if (gps_dl_link_is_ready_to_write(link_id))
			gps_dl_link_start_tx_dma_if_has_data(link_id);
		else
			GDL_LOGXW(link_id, "too early writing");
		break;

	case GPS_DL_EVT_LINK_PRINT_HW_STATUS:
	case GPS_DL_EVT_LINK_PRINT_DATA_STATUS:
		if (!gps_each_link_is_active(link_id)) {
			GDL_LOGXW(link_id, "inactive, do not dump hw status");
			break;
		}

		gps_dma_buf_show(&p_link->rx_dma_buf, true);
		gps_dma_buf_show(&p_link->tx_dma_buf, true);
		if (!gps_dl_conninfra_is_okay_or_handle_it(NULL, true))
			break;

		show_log = gps_dl_set_show_reg_rw_log(true);
		if (evt == GPS_DL_EVT_LINK_PRINT_HW_STATUS) {
			gps_dl_hw_dump_host_csr_gps_info(true);
			gps_dl_hw_print_hw_status(link_id, true);
			gps_each_dsp_reg_gourp_read_start(link_id, true, 4);
		} else {
			gps_dl_hw_print_hw_status(link_id, false);
			gps_each_dsp_reg_gourp_read_start(link_id, true, 2);
		}
		gps_dl_set_show_reg_rw_log(show_log);
		break;

	case GPS_DL_EVT_LINK_DSP_FSM_TIMEOUT:
		gps_dsp_fsm(GPS_DSP_EVT_CTRL_TIMER_EXPIRE, link_id);
		break;
#if 0
	case GPS_DL_EVT_LINK_RESET_GPS:
		/* turn off GPS power directly */
		break;

	case GPS_DL_EVT_LINK_PRE_CONN_RESET:
		/* turn off Connsys power directly
		 * 1. no need to do anything, just make sure the message queue is empty
		 * 2. how to handle ctrld block issue
		 */
		/* gps_dl_link_open_ack(link_id); */
		break;
#endif
	default:
		break;
	}

	j1 =  gps_dl_tick_get();
	GDL_LOGXI_EVT(link_id, "evt = %s, dj = %lu", gps_dl_link_event_name(evt), j1 - j0);
}

void gps_dl_link_pre_off_setting(enum gps_dl_link_id_enum link_id)
{
	/*
	 * The order is important:
	 * 1. disallow write, avoiding to start dma
	 * 2. stop tx/rx dma and mask dma irq if it is last link
	 * 3. mask link's irqs
	 * 4. set inactive after all irq mask done
	 * (at this time isr can check inactive and unmask irq safely due to step 3 already mask irqs)
	 */
	gps_dl_link_set_ready_to_write(link_id, false);
	gps_dl_hal_link_confirm_dma_stop(link_id);
	gps_dl_link_irq_set(link_id, false);
	gps_each_link_set_active(link_id, false);
}

bool g_gps_dl_dpstop_release_wakelock_fg;
void gps_dl_link_post_enter_dpstop_setting(enum gps_dl_link_id_enum link_id)
{
	if (GPS_DSP_ST_HW_STOP_MODE == gps_dsp_state_get(GPS_DATA_LINK_ID0)
			&& (GPS_DSP_ST_HW_STOP_MODE == gps_dsp_state_get(GPS_DATA_LINK_ID1)
			|| GPS_DSP_ST_OFF == gps_dsp_state_get(GPS_DATA_LINK_ID1))) {
#if GPS_DL_HAS_PLAT_DRV
		gps_dl_wake_lock_hold(false);
#endif
		g_gps_dl_dpstop_release_wakelock_fg = true;
		GDL_LOGXW(link_id, "enter dpstop with wake_lock relased");
	}
}

void gps_dl_link_pre_leave_dpstop_setting(enum gps_dl_link_id_enum link_id)
{
	if (g_gps_dl_dpstop_release_wakelock_fg) {
#if GPS_DL_HAS_PLAT_DRV
		gps_dl_wake_lock_hold(true);
#endif
		g_gps_dl_dpstop_release_wakelock_fg = false;
		GDL_LOGXW(link_id, "exit dpstop with wake_lock holded");
	}
}

