/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"
#include "gps_dl_time_tick.h"

#include "gps_each_link.h"
#include "gps_dl_hal.h"
#include "gps_dl_hal_api.h"
#include "gps_dl_hal_util.h"
#include "gps_dl_hw_api.h"
#include "gps_dl_isr.h"
#include "gps_dl_lib_misc.h"
#include "gps_dsp_fsm.h"
#include "gps_dl_osal.h"
#include "gps_dl_name_list.h"
#include "gps_dl_context.h"
#include "gps_dl_subsys_reset.h"
#if GPS_DL_HAS_PLAT_DRV
#include "gps_dl_linux_plat_drv.h"
#endif

#include "linux/errno.h"


void gps_each_link_init(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);

	p->session_id = 0;
	gps_each_link_mutexes_init(p);
	gps_each_link_spin_locks_init(p);
	gps_each_link_set_active(link_id, false);
	gps_dl_link_set_ready_to_write(link_id, false);
	gps_each_link_context_init(link_id);
	gps_each_link_set_state(link_id, LINK_CLOSED);
}

void gps_each_link_deinit(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);

	gps_each_link_set_state(link_id, LINK_UNINIT);
	gps_each_link_mutexes_deinit(p);
	gps_each_link_spin_locks_deinit(p);
}

void gps_each_link_context_init(enum gps_dl_link_id_enum link_id)
{
	enum gps_each_link_waitable_type j;

	for (j = 0; j < GPS_DL_WAIT_NUM; j++)
		gps_dl_link_waitable_reset(link_id, j);
}

void gps_each_link_context_clear(enum gps_dl_link_id_enum link_id)
{
	gps_dl_link_waitable_reset(link_id, GPS_DL_WAIT_WRITE);
	gps_dl_link_waitable_reset(link_id, GPS_DL_WAIT_READ);
}

int gps_each_link_open(enum gps_dl_link_id_enum link_id)
{
	enum gps_each_link_state_enum state, state2;
	enum GDL_RET_STATUS gdl_ret;
	long sigval = 0;
	bool okay = false;
	int retval;
#if GPS_DL_ON_CTP
	/* Todo: is it need on LINUX? */
	struct gps_each_link *p_link = gps_dl_link_get(link_id);

	gps_dma_buf_align_as_byte_mode(&p_link->tx_dma_buf);
	gps_dma_buf_align_as_byte_mode(&p_link->rx_dma_buf);
#endif

#if 0
#if (GPS_DL_ON_LINUX && !GPS_DL_NO_USE_IRQ && !GPS_DL_HW_IS_MOCK)
	if (!p_link->sub_states.irq_is_init_done) {
		gps_dl_irq_init();
		p_link->sub_states.irq_is_init_done = true;
	}
#endif
#endif

	state = gps_each_link_get_state(link_id);

	switch (state) {
	case LINK_RESUMING:
	case LINK_SUSPENDED:
	case LINK_SUSPENDING:
		retval = -EAGAIN;
		break;

	case LINK_CLOSING:
	case LINK_RESETTING:
	case LINK_DISABLED:
		retval = -EAGAIN;
		break;

	case LINK_RESET_DONE:
		/* RESET_DONE stands for user space not close me */
		retval = -EBUSY; /* twice open not allowed */
		break;

	case LINK_OPENED:
	case LINK_OPENING:
		retval = -EBUSY;; /* twice open not allowed */
		break;

	case LINK_CLOSED:
		okay = gps_each_link_change_state_from(link_id, LINK_CLOSED, LINK_OPENING);
		if (!okay) {
			retval = -EBUSY;
			break;
		}

		/* TODO: simplify the flags */
		gps_each_link_set_bool_flag(link_id, LINK_TO_BE_CLOSED, false);
		gps_each_link_set_bool_flag(link_id, LINK_USER_OPEN, true);

		gps_dl_link_waitable_reset(link_id, GPS_DL_WAIT_OPEN_CLOSE);
		gps_dl_link_event_send(GPS_DL_EVT_LINK_OPEN, link_id);
		gps_dl_link_open_wait(link_id, &sigval);

		/* TODO: Check this mutex can be removed?
		 * the possible purpose is make it's atomic from LINK_USER_OPEN and LINK_OPEN_RESULT_OKAY.
		 */
		gps_each_link_take_big_lock(link_id, GDL_LOCK_FOR_OPEN);
		if (sigval != 0) {
			gps_each_link_set_bool_flag(link_id, LINK_USER_OPEN, false);

			gdl_ret = gps_dl_link_try_wait_on(link_id, GPS_DL_WAIT_OPEN_CLOSE);
			if (gdl_ret == GDL_OKAY) {
				okay = gps_each_link_change_state_from(link_id, LINK_OPENED, LINK_CLOSING);

				/* Change okay, need to send event to trigger close */
				if (okay) {
					gps_each_link_give_big_lock(link_id);
					gps_dl_link_event_send(GPS_DL_EVT_LINK_CLOSE, link_id);
					GDL_LOGXW_ONF(link_id,
						"sigval = %ld, corner case 1: close it", sigval);
					retval = -EBUSY;
					break;
				}

				/* Not change okay, state maybe RESETTING or RESET_DONE */
				state2 = gps_each_link_get_state(link_id);
				if (state2 == LINK_RESET_DONE)
					gps_each_link_set_state(link_id, LINK_CLOSED);

				gps_each_link_give_big_lock(link_id);
				GDL_LOGXW_ONF(link_id, "sigval = %ld, corner case 2: %s",
					sigval, gps_dl_link_state_name(state2));
				retval = -EBUSY;
				break;
			}

			gps_each_link_give_big_lock(link_id);
			GDL_LOGXW_ONF(link_id, "sigval = %ld, normal case", sigval);
			retval = -EINVAL;
			break;
		}

		okay = gps_each_link_get_bool_flag(link_id, LINK_OPEN_RESULT_OKAY);
		gps_each_link_give_big_lock(link_id);

		if (okay)
			retval = 0;
		else {
			gps_each_link_set_bool_flag(link_id, LINK_USER_OPEN, false);
			retval = -EBUSY;
		}
		break;

	default:
		retval = -EINVAL;
		break;
	}

	if (retval == 0) {
		GDL_LOGXD_ONF(link_id, "prev_state = %s, retval = %d",
			gps_dl_link_state_name(state), retval);
	} else {
		GDL_LOGXW_ONF(link_id, "prev_state = %s, retval = %d",
			gps_dl_link_state_name(state), retval);
	}

	return retval;
}

int gps_each_link_reset(enum gps_dl_link_id_enum link_id)
{
	/*
	 * - set each link resetting flag
	 */
	enum gps_each_link_state_enum state, state2;
	bool okay = false;
	int retval;

	state = gps_each_link_get_state(link_id);

	switch (state) {
	case LINK_OPENING:
	case LINK_CLOSING:
	case LINK_CLOSED:
	case LINK_DISABLED:
		retval = -EBUSY;
		break;

	case LINK_RESETTING:
	case LINK_RESET_DONE:
		retval = 0;
		break;

	case LINK_RESUMING:
	case LINK_SUSPENDING:
	case LINK_SUSPENDED:
	case LINK_OPENED:
_try_change_to_reset_again:
		okay = gps_each_link_change_state_from(link_id, state, LINK_RESETTING);
		if (!okay) {
			state2 = gps_each_link_get_state(link_id);

			/* Already reset or close, not trigger reseeting again */
			GDL_LOGXW_ONF(link_id, "state flip to %s - corner case",
				gps_dl_link_state_name(state2));

			/* -ing state may become -ed state, try change to reset again */
			if ((state == LINK_SUSPENDING && state2 == LINK_SUSPENDED) ||
				(state == LINK_RESUMING && state2 == LINK_OPENED)) {
				state = state2;
				goto _try_change_to_reset_again;
			}

			if (state2 == LINK_RESETTING || state2 == LINK_RESET_DONE)
				retval = 0;
			else
				retval = -EBUSY;
			break;
		}

		gps_each_link_set_bool_flag(link_id, LINK_IS_RESETTING, true);

		/* no need to wait reset ack
		 * TODO: make sure message send okay
		 */
		gps_dl_link_waitable_reset(link_id, GPS_DL_WAIT_RESET);
		gps_dl_link_event_send(GPS_DL_EVT_LINK_RESET_DSP, link_id);
		retval = 0;
		break;

	default:
		retval = -EINVAL;
		break;
	}

	/* wait until cttld thread ack the status */
	if (retval == 0) {
		GDL_LOGXD_ONF(link_id, "prev_state = %s, retval = %d",
			gps_dl_link_state_name(state), retval);
	} else {
		GDL_LOGXW_ONF(link_id, "prev_state = %s, retval = %d",
			gps_dl_link_state_name(state), retval);
	}

	return retval;
}

int gps_each_link_close_or_suspend_inner(enum gps_dl_link_id_enum link_id,
	enum gps_each_link_state_enum state,
	enum gps_each_link_close_or_suspend_op close_or_suspend_op)
{
	enum gps_each_link_state_enum state2;
	bool okay = false;
	int retval = 0;
	bool hw_suspend;

	hw_suspend = !!(close_or_suspend_op == GDL_DPSTOP || close_or_suspend_op == GDL_CLKEXT);
	gps_each_link_take_big_lock(link_id, GDL_LOCK_FOR_CLOSE);
	do {
		if (hw_suspend) {
			okay = gps_each_link_change_state_from(link_id, LINK_OPENED, LINK_SUSPENDING);
			if (!okay) {
				state2 = gps_each_link_get_state(link_id);
				gps_each_link_give_big_lock(link_id);
				GDL_LOGXW_ONF(link_id, "state check: %s, return hw suspend fail",
					gps_dl_link_state_name(state2));
				retval = -EINVAL;
				break;
			}

			gps_each_link_set_bool_flag(link_id,
				LINK_SUSPEND_TO_CLK_EXT, close_or_suspend_op == GDL_CLKEXT);
		} else {
			if (state == LINK_SUSPENDED) {
				okay = gps_each_link_change_state_from(
					link_id, LINK_SUSPENDED, LINK_CLOSING);
			} else {
				okay = gps_each_link_change_state_from(
					link_id, LINK_OPENED, LINK_CLOSING);
			}
			gps_each_link_set_bool_flag(link_id, LINK_USER_OPEN, false);
			if (!okay) {
				state2 = gps_each_link_get_state(link_id);
				if (state2 == LINK_RESET_DONE)
					gps_each_link_set_state(link_id, LINK_CLOSED);
				else {
					GDL_LOGXW_ONF(link_id, "state check: %s, return close ok",
						gps_dl_link_state_name(state2));
				}
				gps_each_link_give_big_lock(link_id);
				retval = 0;
				break;
			}
		}
		gps_each_link_give_big_lock(link_id);
	} while (0);
	return retval;
}

int gps_each_link_close_or_suspend(enum gps_dl_link_id_enum link_id,
	enum gps_each_link_close_or_suspend_op close_or_suspend_op)
{
	enum gps_each_link_state_enum state;
	long sigval = 0;
	int retval;
	bool hw_suspend;

	state = gps_each_link_get_state(link_id);
	hw_suspend = !!(close_or_suspend_op == GDL_DPSTOP || close_or_suspend_op == GDL_CLKEXT);

	switch (state) {
	case LINK_OPENING:
	case LINK_CLOSING:
	case LINK_CLOSED:
	case LINK_DISABLED:
		/* twice close */
		/* TODO: show user open flag */
		retval = -EINVAL;
		break;

	case LINK_SUSPENDING:
	case LINK_RESUMING:
	case LINK_RESETTING:
		if (hw_suspend) {
			if (state == LINK_SUSPENDING)
				retval = 0;
			else if (state == LINK_RESUMING)
				retval = -EBUSY;
			else
				retval = -EINVAL;
			break;
		}

		/* close on xxx-ing states: just recording user is not online
		 * ctrld will handle it on the end of xxx-ing handling
		 */
		gps_each_link_set_bool_flag(link_id, LINK_USER_OPEN, false);
		GDL_LOGXE_ONF(link_id, "state check: %s, return close ok", gps_dl_link_state_name(state));
		/* return okay to avoid twice close */
		retval = 0;
		break;

	case LINK_RESET_DONE:
		if (hw_suspend) {
			retval = -EINVAL;
			break;
		}
		gps_each_link_set_bool_flag(link_id, LINK_USER_OPEN, false);
		gps_each_link_set_state(link_id, LINK_CLOSED);
		retval = 0;
		break;

	case LINK_SUSPENDED:
	case LINK_OPENED:
		retval = gps_each_link_close_or_suspend_inner(link_id, state, close_or_suspend_op);
		if (retval != 0)
			break;

		/* clean the done(fired) flag before send and wait */
		gps_dl_link_waitable_reset(link_id, GPS_DL_WAIT_OPEN_CLOSE);
		if (hw_suspend)
			gps_dl_link_event_send(GPS_DL_EVT_LINK_ENTER_DPSTOP, link_id);
		else
			gps_dl_link_event_send(GPS_DL_EVT_LINK_CLOSE, link_id);

		/* set this status, hal proc will by pass the message from the link
		 * it can make LINK_CLOSE be processed faster
		 */
		gps_each_link_set_bool_flag(link_id, LINK_TO_BE_CLOSED, true);
		gps_dl_link_close_wait(link_id, &sigval);

		if (sigval) {
			retval = -EINVAL;
			break;
		}

		retval = 0;
		break;
	default:
		retval = -EINVAL;
		break;
	}

	if (retval == 0) {
		GDL_LOGXD_ONF(link_id, "prev_state = %s, retval = %d, op = %d",
			gps_dl_link_state_name(state), retval, close_or_suspend_op);
	} else {
		GDL_LOGXW_ONF(link_id, "prev_state = %s, retval = %d, op = %d",
			gps_dl_link_state_name(state), retval, close_or_suspend_op);
	}

	return retval;
}

int gps_each_link_close(enum gps_dl_link_id_enum link_id)
{
	return gps_each_link_close_or_suspend(link_id, GDL_CLOSE);
}
int gps_each_link_check(enum gps_dl_link_id_enum link_id, int reason)
{
	enum gps_each_link_state_enum state;
	enum gps_dl_link_event_id event;
	int retval = 0;

	state = gps_each_link_get_state(link_id);

	switch (state) {
	case LINK_OPENING:
	case LINK_CLOSING:
	case LINK_CLOSED:
	case LINK_DISABLED:
		break;

	case LINK_RESETTING:
#if 0
		if (rstflag == 1) {
			/* chip resetting */
			retval = -888;
		} else if (rstflag == 2) {
			/* chip reset end */
			retval = -889;
		} else {
			/* normal */
			retval = 0;
		}
#endif
		retval = -888;
		break;

	case LINK_RESET_DONE:
		retval = 889;
		break;

	case LINK_RESUMING:
	case LINK_SUSPENDING:
	case LINK_SUSPENDED:
	case LINK_OPENED:
		if (reason == 2)
			event = GPS_DL_EVT_LINK_PRINT_HW_STATUS;
		else if (reason == 4)
			event = GPS_DL_EVT_LINK_PRINT_DATA_STATUS;
		else
			break;

		/* if L1 trigger it, also print L5 status
		 * for this case, dump L5 firstly.
		 */
		if (link_id == GPS_DATA_LINK_ID0)
			gps_dl_link_event_send(event, GPS_DATA_LINK_ID1);

		gps_dl_link_event_send(event, link_id);
		break;

	default:
		break;
	}

	GDL_LOGXW_ONF(link_id, "prev_state = %s, reason = %d, retval = %d",
		gps_dl_link_state_name(state), reason, retval);

	return retval;
}

int gps_each_link_enter_dsleep(enum gps_dl_link_id_enum link_id)
{
	struct gps_each_link *p_link = gps_dl_link_get(link_id);

	gps_dl_link_event_send(GPS_DL_EVT_LINK_ENTER_DPSLEEP, link_id);
	gps_dma_buf_reset(&p_link->tx_dma_buf);
	gps_dma_buf_reset(&p_link->rx_dma_buf);
	return 0;
}

int gps_each_link_leave_dsleep(enum gps_dl_link_id_enum link_id)
{
#if GPS_DL_ON_CTP
	struct gps_each_link *p_link = gps_dl_link_get(link_id);

	gps_dma_buf_align_as_byte_mode(&p_link->tx_dma_buf);
	gps_dma_buf_align_as_byte_mode(&p_link->rx_dma_buf);
#endif
	gps_dl_link_event_send(GPS_DL_EVT_LINK_LEAVE_DPSLEEP, link_id);
	return 0;
}


int gps_each_link_hw_suspend(enum gps_dl_link_id_enum link_id, bool need_clk_ext)
{
	enum gps_each_link_close_or_suspend_op op;

	if (need_clk_ext)
		op = GDL_CLKEXT;
	else
		op = GDL_DPSTOP;
	return gps_each_link_close_or_suspend(link_id, op);
}

int gps_each_link_hw_resume(enum gps_dl_link_id_enum link_id, bool revert_for_mvcd)
{
	enum gps_each_link_state_enum state;
	long sigval = 0;
	bool okay = false;
	int retval;
#if GPS_DL_ON_CTP
	struct gps_each_link *p_link = gps_dl_link_get(link_id);
#endif

	state = gps_each_link_get_state(link_id);
	do {
		if (state != LINK_SUSPENDED) {
			retval = -EINVAL;
			break;
		}

		okay = gps_each_link_change_state_from(link_id, LINK_SUSPENDED, LINK_RESUMING);
		if (!okay) {
			retval = -EBUSY;
			break;
		}

		gps_each_link_set_bool_flag(link_id, LINK_TO_BE_CLOSED, false);
		gps_dl_link_waitable_reset(link_id, GPS_DL_WAIT_OPEN_CLOSE);
#if GPS_DL_ON_CTP
		gps_dma_buf_align_as_byte_mode(&p_link->tx_dma_buf);
		gps_dma_buf_align_as_byte_mode(&p_link->rx_dma_buf);
#endif
		if (revert_for_mvcd)
			gps_dl_link_event_send(GPS_DL_EVT_LINK_LEAVE_DPSTOP2, link_id);
		else
			gps_dl_link_event_send(GPS_DL_EVT_LINK_LEAVE_DPSTOP, link_id);

		gps_dl_link_open_wait(link_id, &sigval);
		if (sigval != 0) {
			GDL_LOGXW_ONF(link_id, "sigval = %ld", sigval);
			retval = -EBUSY;
			break;
		}

		okay = gps_each_link_get_bool_flag(link_id, LINK_OPEN_RESULT_OKAY);
		if (okay)
			retval = 0;
		else
			retval = -EBUSY;
	} while (0);

	if (retval == 0) {
		GDL_LOGXD_ONF(link_id, "prev_state = %s, retval = %d",
			gps_dl_link_state_name(state), retval);
	} else {
		GDL_LOGXW_ONF(link_id, "prev_state = %s, retval = %d",
			gps_dl_link_state_name(state), retval);
	}
	return retval;
}

/* TODO: determine return value type */
int gps_each_link_write(enum gps_dl_link_id_enum link_id,
	unsigned char *buf, unsigned int len)
{
	return gps_each_link_write_with_opt(link_id, buf, len, true);
}

#define GPS_DL_READ_SHOW_BUF_MAX_LEN (32)
int gps_each_link_write_with_opt(enum gps_dl_link_id_enum link_id,
	unsigned char *buf, unsigned int len, bool wait_tx_done)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	enum GDL_RET_STATUS gdl_ret;
	long sigval = 0;

	if (NULL == p)
		return -EINVAL;

	if (len > p->tx_dma_buf.len)
		return -EINVAL;

	if (gps_each_link_get_state(link_id) != LINK_OPENED) {
		GDL_LOGXW_DRW(link_id, "not opened, drop the write data len = %d", len);
		return -EBUSY;
	}

	if (len <= GPS_DL_READ_SHOW_BUF_MAX_LEN)
		gps_dl_hal_show_buf("wr_buf", buf, len);
	else
		GDL_LOGXD_DRW(link_id, "wr_buf, len = %d", len);

	while (1) {
		gdl_ret = gdl_dma_buf_put(&p->tx_dma_buf, buf, len);

		if (gdl_ret == GDL_OKAY) {
			gps_dl_link_event_send(GPS_DL_EVT_LINK_WRITE, link_id);
#if (GPS_DL_NO_USE_IRQ == 1)
			if (wait_tx_done) {
				do {
					gps_dl_hal_a2d_tx_dma_wait_until_done_and_stop_it(
						link_id, GPS_DL_RW_NO_TIMEOUT, false);
					gps_dl_hal_event_send(GPS_DL_HAL_EVT_A2D_TX_DMA_DONE, link_id);
					/* for case tx transfer_max > 0, GPS_DL_HAL_EVT_A2D_TX_DMA_DONE may */
					/* start anthor dma session again, need to loop again until all data done */
				} while (!gps_dma_buf_is_empty(&p->tx_dma_buf));
			}
#endif
			return 0;
		} else if (gdl_ret == GDL_FAIL_NOSPACE || gdl_ret == GDL_FAIL_BUSY ||
			gdl_ret == GDL_FAIL_NOENTRY) {
			/* TODO: */
			/* 1. note: BUSY stands for others thread is do write, it should be impossible */
			/* - If wait on BUSY, should wake up the waitings or return eno_again? */
			/* 2. note: NOSPACE stands for need wait for tx dma working done */
			gps_dma_buf_show(&p->tx_dma_buf, false);
			GDL_LOGXD(link_id,
				"wait due to gdl_dma_buf_put ret = %s", gdl_ret_to_name(gdl_ret));
			gdl_ret = gps_dl_link_wait_on(&p->waitables[GPS_DL_WAIT_WRITE], &sigval);
			if (gdl_ret == GDL_FAIL_SIGNALED)
				return -ERESTARTSYS;
		} else {
			gps_dma_buf_show(&p->tx_dma_buf, true);
			GDL_LOGXW(link_id,
				"fail due to gdl_dma_buf_put ret = %s", gdl_ret_to_name(gdl_ret));
			break;
		}
	}

	return -EFAULT;
}

int gps_each_link_read(enum gps_dl_link_id_enum link_id,
	unsigned char *buf, unsigned int len) {
	return gps_each_link_read_with_timeout(link_id, buf, len, GPS_DL_RW_NO_TIMEOUT, NULL);
}

int gps_each_link_read_with_timeout(enum gps_dl_link_id_enum link_id,
	unsigned char *buf, unsigned int len, int timeout_usec, bool *p_is_nodata)
{
	struct gps_each_link *p = gps_dl_link_get(link_id);
	enum GDL_RET_STATUS gdl_ret;
#if (GPS_DL_NO_USE_IRQ == 0)
	long sigval = 0;
#endif
	unsigned int data_len;

	if (NULL == p)
		return -EINVAL;

	while (1) {
		gdl_ret = gdl_dma_buf_get(&p->rx_dma_buf, buf, len, &data_len, p_is_nodata);

		if (gdl_ret == GDL_OKAY) {
			if (data_len <= GPS_DL_READ_SHOW_BUF_MAX_LEN)
				gps_dl_hal_show_buf("rd_buf", buf, data_len);
			else
				GDL_LOGXD_DRW(link_id, "rd_buf, len = %d", data_len);

			gps_each_link_spin_lock_take(link_id, GPS_DL_SPINLOCK_FOR_DMA_BUF);
			if (p->rx_dma_buf.has_pending_rx) {
				p->rx_dma_buf.has_pending_rx = false;
				gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_DMA_BUF);

				GDL_LOGXI_DRW(link_id, "has pending rx, trigger again");
				gps_dl_hal_event_send(GPS_DL_HAL_EVT_D2A_RX_HAS_DATA, link_id);
			} else
				gps_each_link_spin_lock_give(link_id, GPS_DL_SPINLOCK_FOR_DMA_BUF);

			return data_len;
		} else if (gdl_ret == GDL_FAIL_NODATA) {
			GDL_LOGXD_DRW(link_id, "gdl_dma_buf_get no data and wait");
#if (GPS_DL_NO_USE_IRQ == 1)
			gdl_ret = gps_dl_hal_wait_and_handle_until_usrt_has_data(
				link_id, timeout_usec);
			if (gdl_ret == GDL_FAIL_TIMEOUT)
				return -EAGAIN;

			gdl_ret = gps_dl_hal_wait_and_handle_until_usrt_has_nodata_or_rx_dma_done(
				link_id, timeout_usec, true);
			if (gdl_ret == GDL_FAIL_TIMEOUT)
				return -EAGAIN;
			continue;
#else
			gdl_ret = gps_dl_link_wait_on(&p->waitables[GPS_DL_WAIT_READ], &sigval);
			if (gdl_ret == GDL_FAIL_SIGNALED)
				return -ERESTARTSYS;
			else if (gdl_ret == GDL_FAIL_NOT_SUPPORT)
				return -EFAULT;
#endif
		} else {
			GDL_LOGXW_DRW(link_id, "gdl_dma_buf_get fail %s", gdl_ret_to_name(gdl_ret));
			return -EFAULT;
		}
	}

	return 0;
}

