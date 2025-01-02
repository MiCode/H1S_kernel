// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 *
 * Filename:
 * ---------
 *   mtk_charger.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of Battery charging
 *
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include <asm/setup.h>
#include "mtk_charger.h"
#include "mtk_battery.h"
#include <tcpm.h>

#define PE5_START_VBAT_MAX  4420
#define SW_JEITA_HW_TH_MV   -10

/* sw jeita */
void do_sw_jeita_state_machine(struct mtk_charger *info)
{
	struct sw_jeita_data *sw_jeita;
	int ibat = 0;
	int vbat = 0;
	int xm_ieoc = 300000;
	struct battery_info *bat = &(info->bat);
	struct power_supply *psy = NULL;
	struct mtk_battery *gm = NULL;

	sw_jeita = &info->sw_jeita;
	sw_jeita->charging = true;
	ibat = get_battery_current(info);	//ma
	vbat = get_battery_voltage(info);	//mv
	sw_jeita->cc = 2000000;	//ma
	sw_jeita->cv = 4480000; //mv

	psy = power_supply_get_by_name("battery");
	if (psy != NULL) {
		gm = (struct mtk_battery *)power_supply_get_drvdata(psy);
		if (gm != NULL){
			chr_err("get mtk_battery drvdata success in %s\n", __func__);
		}
	} else {
		return;
	}

	if (info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {	//pps
		switch (info->battery_temp) {
		case -100 ... -10:
			sw_jeita->charging = false;
			sw_jeita->cc = 0;	//<-10
			sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
			sw_jeita->sm = TEMP_BELOW_T0;
			break;

		case -9 ... 0:
			xm_ieoc = 300000;	//-9~0
			sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
			sw_jeita->cc = info->data.jeita_temp_below_t0_cc;
			sw_jeita->sm = TEMP_T0_TO_T1;
			if ((sw_jeita->pre_sm == TEMP_BELOW_T0) && (info->battery_temp < -8)) {
				sw_jeita->charging = false;
				sw_jeita->cc = 0;
				sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
				sw_jeita->sm = TEMP_BELOW_T0;
				chr_err("[%s]still low temp, not charging\n",__func__);
			}
			break;

		case 1 ... 5:
			xm_ieoc = 300000;
			sw_jeita->cv = info->data.jeita_temp_t0_to_t1_cv;
			sw_jeita->cc = info->data.jeita_temp_t0_to_t1_cc;
			sw_jeita->sm = TEMP_T1_TO_T2;
			break;

		case 6 ... 9:
			xm_ieoc = 300000;
			sw_jeita->cv = info->data.jeita_temp_t1_to_t2_cv;
			sw_jeita->cc = info->data.jeita_temp_t1_to_t2_cc;
			sw_jeita->sm = TEMP_T2_TO_T3;
			break;

		case 10 ... 14:
			xm_ieoc = 300000;
			sw_jeita->cv = info->data.jeita_temp_t2_to_t3_cv;
			sw_jeita->cc = info->data.jeita_temp_t2_to_t3_cc;
			sw_jeita->sm = TEMP_T3_TO_T4;
			break;

		case 15 ... 35:
			if (info->pd_adapter->verifed == true) { //xiaomi adapter
				if(bat->ffc) {
					xm_ieoc = bat->iterm * 1000;
				}
				sw_jeita->cv = bat->fv * 1000;
			} else {
				xm_ieoc = 300000;
				sw_jeita->cv = 4470000;
			}
			sw_jeita->sm = TEMP_T4_TO_T5;
			if ((sw_jeita->pre_sm == TEMP_T3_TO_T4) && (info->battery_temp < 17)) {
				xm_ieoc = 300000;
				sw_jeita->cv = info->data.jeita_temp_t2_to_t3_cv;
				sw_jeita->cc = info->data.jeita_temp_t2_to_t3_cc;
				sw_jeita->sm = TEMP_T3_TO_T4;
				chr_err("[%s]still cool, not fast charging\n",__func__);
			}
			break;

		case 36 ... 44:
			if (info->pd_adapter->verifed == true) { //xiaomi adapter
				if(bat->ffc) {
					xm_ieoc = bat->iterm * 1000;
				}
				sw_jeita->cv = bat->fv * 1000;
			} else {
				xm_ieoc = 300000;
				sw_jeita->cv = 4470000;
			}

			sw_jeita->sm = TEMP_T4_TO_T5;
			if ((sw_jeita->pre_sm == TEMP_T5_TO_T6) && (info->battery_temp > 43)) {
				xm_ieoc = 300000;
				sw_jeita->cv = info->data.jeita_temp_t5_to_t6_cv;
				sw_jeita->cc = info->data.jeita_temp_t5_to_t6_cc;
				sw_jeita->sm = TEMP_T5_TO_T6;
				chr_err("[%s]still hot, not fast charging\n",__func__);
			}
			break;

		case 45 ... 56:
			xm_ieoc = 300000;
			sw_jeita->cv = info->data.jeita_temp_t5_to_t6_cv;
			sw_jeita->cc = info->data.jeita_temp_t5_to_t6_cc;
			sw_jeita->sm = TEMP_T5_TO_T6;
			if ((sw_jeita->pre_sm == TEMP_ABOVE_T6) && (info->battery_temp > 55)) {
				sw_jeita->charging = false;
				sw_jeita->cc = 0;
				sw_jeita->cv = info->data.jeita_temp_t5_to_t6_cv;
				sw_jeita->sm = TEMP_ABOVE_T6;
				chr_err("[%s]still high temp, not charging\n",__func__);
			}
			break;

		case 57 ... 200:
			sw_jeita->charging = false;
			sw_jeita->cc = 0;
			sw_jeita->cv = info->data.jeita_temp_t5_to_t6_cv;
			sw_jeita->sm = TEMP_ABOVE_T6;
			break;
		default:
			sw_jeita->charging = false;
			sw_jeita->cc = 0;
			sw_jeita->cv = info->data.jeita_temp_above_t6_cv;
			sw_jeita->sm = TEMP_ABOVE_T6;
			chr_err("[SW_JEITA] The battery temperature is not within the range (%d) !!\n",info->battery_temp);
		}
	} else {		//not adpo(hvdcp,dcp,cdp,sdp,float)
		switch (info->battery_temp) {
		case -100 ... -10:
			sw_jeita->charging = false;
			sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
			sw_jeita->cc = 0;
			sw_jeita->sm = TEMP_BELOW_T0;
			break;

		case -9 ... 0:
			sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
			sw_jeita->cc = info->data.jeita_temp_below_t0_cc;
			sw_jeita->sm = TEMP_T0_TO_T1;
			if (sw_jeita->pre_sm == TEMP_BELOW_T0 && (info->battery_temp < -8)) {
				sw_jeita->charging = false;
				sw_jeita->cc = 0;
				sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
				sw_jeita->sm = TEMP_BELOW_T0;
				chr_err("[%s]still low temp, not charging\n",__func__);
			}
			break;

		case 1 ... 5:
			sw_jeita->cv = info->data.jeita_temp_t0_to_t1_cv;
			sw_jeita->cc = info->data.jeita_temp_t0_to_t1_cc;
			sw_jeita->sm = TEMP_T1_TO_T2;
			break;

		case 6 ... 9:
			sw_jeita->cv = info->data.jeita_temp_t1_to_t2_cv;
			sw_jeita->cc = info->data.jeita_temp_t1_to_t2_cc;
			sw_jeita->sm = TEMP_T2_TO_T3;
			break;

		case 10 ... 14:
			sw_jeita->cv = info->data.jeita_temp_t2_to_t3_cv;
			sw_jeita->cc = info->data.jeita_temp_t2_to_t3_cc;
			sw_jeita->sm = TEMP_T3_TO_T4;
			break;

		case 15 ... 34:
			sw_jeita->cv = bat->fv * 1000;
			sw_jeita->cc = info->data.jeita_temp_t3_to_t4_cc;
			sw_jeita->sm = TEMP_T4_TO_T5;
			break;

		case 35 ... 44:
			sw_jeita->cv = bat->fv * 1000;
			sw_jeita->cc = info->data.jeita_temp_t4_to_t5_cc;
			sw_jeita->sm = TEMP_T4_TO_T5;
			if ((sw_jeita->pre_sm == TEMP_T5_TO_T6) && (info->battery_temp > 43)) {
				sw_jeita->cv = info->data.jeita_temp_t5_to_t6_cv;
				sw_jeita->cc = info->data.jeita_temp_t5_to_t6_cc;
				sw_jeita->sm = TEMP_T5_TO_T6;
				chr_err("[%s]still hot, maintain TEMP_T5_TO_T6\n",__func__);
			}
			break;

		case 45 ... 56:
			sw_jeita->cv = info->data.jeita_temp_t5_to_t6_cv;
			sw_jeita->cc = info->data.jeita_temp_t5_to_t6_cc;
			sw_jeita->sm = TEMP_T5_TO_T6;
			if ((sw_jeita->pre_sm == TEMP_ABOVE_T6) && (info->battery_temp > 55)) {
				sw_jeita->charging = false;
				sw_jeita->cc = 0;
				sw_jeita->cv = info->data.jeita_temp_t5_to_t6_cv;
				sw_jeita->sm = TEMP_ABOVE_T6;
				chr_err("[%s]still high temp, not charging\n",__func__);
			}
			break;

		case 57 ... 200:
			sw_jeita->charging = false;
			sw_jeita->cc = 0;
			sw_jeita->cv = info->data.jeita_temp_above_t6_cv;
			sw_jeita->sm = TEMP_ABOVE_T6;
			break;
		default:
			sw_jeita->charging = false;
			sw_jeita->cv = info->data.jeita_temp_above_t6_cv;
			sw_jeita->cc = 0;
			sw_jeita->sm = TEMP_ABOVE_T6;
			chr_err("[SW_JEITA] The battery temperature is not within the range (%d) !!\n", info->battery_temp);
		}
		xm_ieoc = 300000;
	}

        if (sw_jeita->pre_sm != sw_jeita->sm) {
                if (sw_jeita->sm <= TEMP_T0_TO_T1) {
                        charger_dev_set_recharge_voltage(info->chg1_dev, 200*1000);
                } else {
                        charger_dev_set_recharge_voltage(info->chg1_dev, 100*1000);
                }
        }

	sw_jeita->pre_sm = sw_jeita->sm;

	if (!chg_alg_is_algo_running(info->alg[0]) &&
		sw_jeita->charging == true &&
		get_uisoc(info) <= 94 &&
		sw_jeita->sm == TEMP_T4_TO_T5 &&
		info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO &&
		(info->chg_data[DVCHG1_SETTING].thermal_input_current_limit > 2200000 ||
		info->chg_data[DVCHG1_SETTING].thermal_input_current_limit == -1) &&
		!info->cmd_discharging &&
		vbat <= PE5_START_VBAT_MAX &&
		!gm->smart_charge[SMART_CHG_NAVIGATION].active_status &&
		!info->bat.ffc_disable &&
		bat->ffc &&
		info->batt_verify &&
		!chg_alg_cp_charge_finished(info->alg[0])) {
		chg_alg_thermal_restart(info->alg[0], false);
		chg_alg_start_algo(info->alg[0]);
		msleep(5);
		chg_alg_thermal_restart(info->alg[0], true);
		chr_err("[%s]Return to normal temperature, restart pe50\n",__func__);
	}

	chr_err("[%s]thermal_input_current_limit=%d\n", __func__, info->chg_data[DVCHG1_SETTING].thermal_input_current_limit);

        if (info->diff_fv_val > 0) {//smart fv
                /*
                 * cycle count reduce cv at temp range 15 ~ 45,
                 * when in low temp (< 15) reduce smart_fv driectly
                 * in normal temp use the min of cycle_fv and smart fv
                 * in high temp (> 45) use 4.1V cv driectly
                 **/
                if (sw_jeita->sm >= TEMP_T5_TO_T6) {//high temp
                        chr_err("[%s]is high tmep, ignore smart_fv\n", __func__);
                } else if ((sw_jeita->sm == TEMP_T4_TO_T5) && (sw_jeita->cv <= 4480 * 1000)) {//in nomal cycle fv
                        sw_jeita->cv = min(bat->fv, (u32)(4480 - info->diff_fv_val)) * 1000;
                } else {//in ffc or low temp
                        sw_jeita->cv -= (info->diff_fv_val * 1000);
                }
        }

        if (bat->ffc == 0) {
                sw_jeita->cv += SW_JEITA_HW_TH_MV * 1000;
        }

	charger_dev_set_constant_voltage(info->chg1_dev, sw_jeita->cv);
	charger_dev_set_eoc_current(info->chg1_dev, xm_ieoc);
	chr_err("[SW_JEITA]sm:%d tmp:%d jeita_cv:%d jeita_cc:%d ibat:%d vbat:%d xm_ieoc:%d pd_type:%d fcc:%d\n",
			sw_jeita->sm, info->battery_temp, sw_jeita->cv, sw_jeita->cc, ibat, vbat, xm_ieoc, info->pd_type, bat->ffc);
}
