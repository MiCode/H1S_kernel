// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 *
 * Filename:
 * ---------
 *    mtk_basic_charger.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of Battery charging
 *
 * Author:
 * -------
 * Wy Chuang
 *
 */
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
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
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
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

#include "mtk_charger.h"
#include "mtk_battery.h"
#include "adapter_class.h"

#define BATTERY_VERIFY_SUCCESS 1
#define NONSTAND_CHG_CURRENT 1000000
#define DCP_CHARGER_CURRENT_LIMIT 1600000
#define OUTDOOR_CHARGER_CURRENT_LIMIT 1900000
#define PD_CHARGER_CURRENT_LIMIT 3150000
#define ACA_CHARGER_CURRENT_LIMIT 3150000
#define PPS_ABNORMAL_TEMP_INPUT_CURRENT_LIMIT 3000000
#define PPS_ABNORMAL_TEMP_CURRENT_LIMIT 3150000
#define PD30_VTA_RESET   5000	/* mV */
#define PD30_ITA_RESET   2000	/* mA */

static int _uA_to_mA(int uA)
{
	if (uA == -1)
		return -1;
	else
		return uA / 1000;
}

static void select_cv(struct mtk_charger *info)
{
	u32 constant_voltage;

        if (info->bat.ffc) {
                info->setting.cv = info->bat.fv * 1000;
                info->setting.iterm = info->bat.iterm;
                return;
        }

	if (info->enable_sw_jeita) {
		if (info->sw_jeita.cv != 0) {
			info->setting.cv = info->sw_jeita.cv;
			return;
		}
        }

	constant_voltage = info->data.battery_cv;
	info->setting.cv = constant_voltage;
}

static bool is_typec_adapter(struct mtk_charger *info)
{
	int rp;

	rp = adapter_dev_get_property(info->pd_adapter, TYPEC_RP_LEVEL);
	chr_info("is_typec_adapter:rp = %d\n",rp);
	if (info->pd_type == MTK_PD_CONNECT_TYPEC_ONLY_SNK &&
			rp != 500 &&
			info->chr_type != POWER_SUPPLY_TYPE_USB &&
			info->chr_type != POWER_SUPPLY_TYPE_USB_CDP &&
			info->chr_type != POWER_SUPPLY_TYPE_USB_ACA)
		return true;

	return false;
}

static bool support_fast_charging(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	int i = 0, state = 0;
	bool ret = false;

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		if (alg == NULL)
			continue;

		if (info->enable_fast_charging_indicator &&
		    ((alg->alg_id & info->fast_charging_indicator) == 0))
			continue;

		chg_alg_set_current_limit(alg, &info->setting);
		state = chg_alg_is_algo_ready(alg);
		chr_info("%s %s ret:%s\n", __func__, dev_name(&alg->dev),
			chg_alg_state_to_str(state));

		if (state == ALG_READY || state == ALG_RUNNING) {
			ret = true;
			break;
		}
	}
	return ret;
}

static bool select_charging_current_limit(struct mtk_charger *info,
	struct chg_limit_setting *setting)
{
	struct charger_data *pdata, *pdata2, *pdata_dvchg, *pdata_dvchg2;
	bool is_basic = false;
	u32 ichg1_min = 0, aicr1_min = 0, vbus_now = 0;
	int ret;
	bool id_flag = false;
	struct power_supply *psy;
	struct power_supply *bat_psy;
	struct mtk_battery * gm = NULL;
	union power_supply_propval val;
	union power_supply_propval val_capacity;
	int blankState = 0, low_fast_enable = 0;
	bool fast_flag = false;
	time64_t time_now = 0, delta_time = 0;
	static time64_t time_last = 0;
	int level;
	int thermal_vote_current;

	bat_psy = power_supply_get_by_name("battery");
	if (bat_psy != NULL) {
		gm = (struct mtk_battery *)power_supply_get_drvdata(bat_psy);
		if (gm != NULL){
			chr_err("get mtk_battery drvdata success in %s\n", __func__);
		}
	} else {
		return 0;
	}

	select_cv(info);

	pdata = &info->chg_data[CHG1_SETTING];
	pdata2 = &info->chg_data[CHG2_SETTING];
	pdata_dvchg = &info->chg_data[DVCHG1_SETTING];
	pdata_dvchg2 = &info->chg_data[DVCHG2_SETTING];
	if (info->usb_unlimited) {
		pdata->input_current_limit =
					info->data.ac_charger_input_current;
		pdata->charging_current_limit =
					info->data.ac_charger_current;
		is_basic = true;
		goto done;
	}

        //xiaomi mtbf currnet mA -> uA
        if (info->mtbf_current > 500) {
                pdata->input_current_limit = info->mtbf_current*1000;
                pdata->charging_current_limit = info->mtbf_current*1000;
                is_basic = true;
                goto done;
        }

	if (info->water_detected) {
		pdata->input_current_limit = info->data.usb_charger_current;
		pdata->charging_current_limit = info->data.usb_charger_current;
		is_basic = true;
		goto done;
	}

	if (((info->bootmode == 1) ||
	    (info->bootmode == 5)) && info->enable_meta_current_limit != 0) {
		pdata->input_current_limit = 200000; // 200mA
		is_basic = true;
		goto done;
	}

	if (info->atm_enabled == true
		&& (info->chr_type == POWER_SUPPLY_TYPE_USB ||
		info->chr_type == POWER_SUPPLY_TYPE_USB_CDP)
		) {
		pdata->input_current_limit = 500000; /* 500mA */
		pdata->charging_current_limit = 500000; /*500mA*/
		is_basic = true;
		goto done;
	}

	if (info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO && info->bat.ffc_disable) {
		 adapter_dev_set_cap_xm(info->pd_adapter, MTK_PD, 9000, 2000);
	}

        if (info->pd_type == MTK_PD_CONNECT_PE_READY_SNK || info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30) {
                if (info->pd_adapter->received_pdos[1] == 0)  { //is cc
                        pdata->input_current_limit = 1500000;
                        pdata->charging_current_limit = 1500000;
                        chr_info("PD adapter is CC, set 5V1.5A\n");
                } else { //PD chg adapter
                        vbus_now = get_vbus(info);
                        if (vbus_now < 7000 && !info->is_chg_done) {
                        	chr_info("PD adapter is CHG, vbus=%dmV\n", vbus_now);
                                adapter_dev_set_cap_xm(info->pd_adapter, MTK_PD, 9000, 2000);
                        }
                        pdata->input_current_limit = info->data.ac_charger_input_current;
                        pdata->charging_current_limit = PD_CHARGER_CURRENT_LIMIT;
                        chr_info("PD adapter is CHG, set 9V2A\n");
                }
                is_basic = true;
        } else if (info->chr_type == POWER_SUPPLY_TYPE_USB &&
	    info->usb_type == POWER_SUPPLY_USB_TYPE_SDP) {
                pdata->input_current_limit =
                        info->data.usb_charger_current;
                /* it can be larger */
                pdata->charging_current_limit =
                        info->data.usb_charger_current;
		is_basic = true;
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_CDP) {
		pdata->input_current_limit =
			info->data.charging_host_charger_current;
		pdata->charging_current_limit =
			info->data.charging_host_charger_current;
		is_basic = true;

	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_DCP) {
		if (gm->smart_charge[SMART_CHG_OUTDOOR_CHARGE].en_ret) {
			pdata->input_current_limit = OUTDOOR_CHARGER_CURRENT_LIMIT;
			pdata->charging_current_limit = OUTDOOR_CHARGER_CURRENT_LIMIT;
			if (!gm->smart_charge[SMART_CHG_OUTDOOR_CHARGE].active_status)
				gm->smart_charge[SMART_CHG_OUTDOOR_CHARGE].active_status = true;
		} else {
			pdata->input_current_limit = DCP_CHARGER_CURRENT_LIMIT;
			pdata->charging_current_limit = DCP_CHARGER_CURRENT_LIMIT;
		}
		if (info->config == DUAL_CHARGERS_IN_SERIES) {
			pdata2->input_current_limit =
				pdata->input_current_limit;
			pdata2->charging_current_limit = 2000000;
		}
                if (info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
			pdata->input_current_limit = PPS_ABNORMAL_TEMP_INPUT_CURRENT_LIMIT;
			pdata->charging_current_limit = PPS_ABNORMAL_TEMP_CURRENT_LIMIT;
		}
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB_ACA) {
		pdata->input_current_limit = ACA_CHARGER_CURRENT_LIMIT/2;
		pdata->charging_current_limit = ACA_CHARGER_CURRENT_LIMIT;
		is_basic = true;
	} else if (info->chr_type == POWER_SUPPLY_TYPE_USB &&
	    info->usb_type == POWER_SUPPLY_USB_TYPE_DCP) {
		/* NONSTANDARD_CHARGER */
		pdata->input_current_limit = NONSTAND_CHG_CURRENT;
		pdata->charging_current_limit = NONSTAND_CHG_CURRENT;

                if (info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
			pdata->input_current_limit = PPS_ABNORMAL_TEMP_INPUT_CURRENT_LIMIT;
			pdata->charging_current_limit = PPS_ABNORMAL_TEMP_CURRENT_LIMIT;
		}
		is_basic = true;
	} else {
		/*chr_type && usb_type cannot match above, set 500mA*/
		pdata->input_current_limit =
				info->data.usb_charger_current;
		pdata->charging_current_limit =
				info->data.usb_charger_current;
		is_basic = true;
	}

	if (!id_flag) {
		psy = power_supply_get_by_name("batt_verify");
		if (psy) {
			power_supply_get_property(psy, POWER_SUPPLY_PROP_AUTHENTIC, &val);
			if(val.intval == BATTERY_VERIFY_SUCCESS) {
				id_flag = true;
				info->batt_verify = true;
			} else {
				id_flag = false;
				info->batt_verify = false;
				chr_err("batt verify fail, limit charging power\n");
			}
		} else {
			info->batt_verify = false;
			chr_err("can't find batt verify, limit charging power\n");
		}
	}

	if (support_fast_charging(info) && id_flag)
		is_basic = false;
	else {
		is_basic = true;
		/* AICL */
		if (!info->disable_aicl)
			charger_dev_run_aicl(info->chg1_dev,
				&pdata->input_current_limit_by_aicl);
		if (info->enable_dynamic_mivr) {
			if (pdata->input_current_limit_by_aicl >
				info->data.max_dmivr_charger_current)
				pdata->input_current_limit_by_aicl =
					info->data.max_dmivr_charger_current;
		}
		if (is_typec_adapter(info)) {
			if (adapter_dev_get_property(info->pd_adapter, TYPEC_RP_LEVEL)
				== 3000) {
				pdata->input_current_limit = 3000000;
				pdata->charging_current_limit = 3000000;
				if (gm->smart_charge[SMART_CHG_OUTDOOR_CHARGE].en_ret) {
					pdata->input_current_limit = OUTDOOR_CHARGER_CURRENT_LIMIT;
					pdata->charging_current_limit = OUTDOOR_CHARGER_CURRENT_LIMIT;
					if (!gm->smart_charge[SMART_CHG_OUTDOOR_CHARGE].active_status)
						gm->smart_charge[SMART_CHG_OUTDOOR_CHARGE].active_status = true;
				} else {
					pdata->input_current_limit = DCP_CHARGER_CURRENT_LIMIT;
					pdata->charging_current_limit = DCP_CHARGER_CURRENT_LIMIT;
				}
			} else if (adapter_dev_get_property(info->pd_adapter,
				TYPEC_RP_LEVEL) == 1500) {
				pdata->input_current_limit = 1500000;
				pdata->charging_current_limit = 2000000;
			} else {
				chr_err("type-C: inquire rp error\n");
				pdata->input_current_limit = 500000;
				pdata->charging_current_limit = 500000;
			}

			chr_info("type-C:%d current:%d\n",
				info->pd_type,
				adapter_dev_get_property(info->pd_adapter,
					TYPEC_RP_LEVEL));
		}
	}

	chr_info("%s: pdata->input_current_limit: %d, pdata->charging_current_limit: %d\n",
				__func__,
				pdata->input_current_limit,
				pdata->charging_current_limit);

	if (info->enable_sw_jeita) {
		if (pdata->charging_current_limit >= info->sw_jeita.cc)
			pdata->charging_current_limit = info->sw_jeita.cc;
	}
	pr_info("charging_current_limit: %d, sw_jeita.cc: %d\n",
		pdata->charging_current_limit, info->sw_jeita.cc);

	sc_select_charging_current(info, pdata);

	if (pdata->thermal_charging_current_limit != -1) {
		if (pdata->thermal_charging_current_limit <=
			pdata->charging_current_limit) {
			pdata->charging_current_limit =
					pdata->thermal_charging_current_limit;
			info->setting.charging_current_limit1 =
					pdata->thermal_charging_current_limit;
		}
		pdata->thermal_throttle_record = true;
	} else
		info->setting.charging_current_limit1 = pdata->charging_current_limit;

        if (info->pd_sink_uA != -1 &&
				(info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO ||
				 info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30 ||
				 info->pd_type == MTK_PD_CONNECT_PE_READY_SNK)) {
                if (info->pd_sink_uA <= pdata->input_current_limit) {
                        pdata->input_current_limit =
					info->pd_sink_uA;
			info->setting.input_current_limit1 =
					pdata->input_current_limit;
                }
                pr_info("%s: lmt by pd_sink_uA = %d\n", __func__, info->pd_sink_uA);
        }

	if (pdata->thermal_input_current_limit != -1) {
		if (pdata->thermal_input_current_limit <=
			pdata->input_current_limit) {
			pdata->input_current_limit =
					pdata->thermal_input_current_limit;
			info->setting.input_current_limit1 =
					pdata->input_current_limit;
		}
		pdata->thermal_throttle_record = true;
	} else
		info->setting.input_current_limit1 = pdata->charging_current_limit;

	if (pdata2->thermal_charging_current_limit != -1) {
		if (pdata2->thermal_charging_current_limit <=
			pdata2->charging_current_limit) {
			pdata2->charging_current_limit =
					pdata2->thermal_charging_current_limit;
			info->setting.charging_current_limit2 =
					pdata2->charging_current_limit;
		}
	} else
		info->setting.charging_current_limit2 = info->sc.sc_ibat;

	if (pdata2->thermal_input_current_limit != -1) {
		if (pdata2->thermal_input_current_limit <=
			pdata2->input_current_limit) {
			pdata2->input_current_limit =
					pdata2->thermal_input_current_limit;
			info->setting.input_current_limit2 =
					pdata2->input_current_limit;
		}
	} else
		info->setting.input_current_limit2 = -1;

	if (is_basic == true && pdata->input_current_limit_by_aicl != -1
		&& !info->charger_unlimited
		&& !info->disable_aicl) {
		if (pdata->input_current_limit_by_aicl <
		    pdata->input_current_limit)
			pdata->input_current_limit =
					pdata->input_current_limit_by_aicl;
	}

	level = gm->thermal_level;
        info->bat.thermal_lv = gm->thermal_level;
	low_fast_enable = gm->smart_charge[SMART_CHG_LOW_FAST].en_ret;
	if (level != 0) {
		if (info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
			ret = power_supply_get_property(bat_psy, POWER_SUPPLY_PROP_CAPACITY, &val_capacity);
			if ((val_capacity.intval <= 40) && (info->first_low_plugin_flag) && low_fast_enable) {
				blankState = info->sm.screen_state;
				chr_err("%s level = %d, first_low_plugin_flag = %d, low_fast_enable = %d, blankState = %d, b_flag = %d\n",
					__func__, level, info->first_low_plugin_flag, low_fast_enable, blankState, info->b_flag);
				//info->sm.screen_state 0:bright, 1:black
				if ((info->b_flag == NORMAL || info->b_flag == BLACK) && !blankState) {  //black to bright
					info->b_flag = BLACK_TO_BRIGHT;
					time_last = ktime_get_seconds();
					fast_flag = true;
					chr_err("%s switch to bright time_last = %d\n", __func__, time_last);
				} else if ((info->b_flag == BLACK_TO_BRIGHT || info->b_flag == BRIGHT) && !blankState) {  //still bright
					info->b_flag = BRIGHT;
					time_now = ktime_get_seconds();
					delta_time = time_now - time_last;
					chr_err("%s still_bright time_now = %d, time_last = %d, delta_time = %d\n", __func__, time_now, time_last, delta_time);
					if(delta_time <= 15) {
						fast_flag = true;
						chr_err("%s still_bright delta_time = %d, stay fast\n", __func__, delta_time);
					} else {
						fast_flag = false;
						chr_err("%s still_bright delta_time = %d, exit fast\n", __func__, delta_time);
					}
				} else { //black
					info->b_flag = BLACK;
					fast_flag = true;
					chr_err("%s black stay fast\n", __func__, delta_time);
				}

				if (info->thermal_board_temp > 420){
					chr_err("%s, thermal_board_temp is above 42: %d\n", __func__, info->thermal_board_temp);
					fast_flag = false;
				}

				if (fast_flag) {  //stay fast strategy
					info->pps_fast_mode = true;
					thermal_vote_current = thermal_mitigation_pps_fast[level];
					/*
					if ((val_capacity.intval > 38) && (info->thermal_board_temp > 385)) {
						if(thermal_vote_current >= 5450000){
							thermal_vote_current -= 3300000;
						} else {
							thermal_vote_current = 2150000;
						}
						chr_err("%s stay fast but decrease 3, info->thermal_board_temp = %d, thermal_vote_current = %d\n", __func__, info->thermal_board_temp, thermal_vote_current);
					} else if ((val_capacity.intval > 35) && (info->thermal_board_temp > 395)) {
						if (thermal_vote_current >= 3950000) {
							thermal_vote_current -= 1800000;
						} else {
							thermal_vote_current = 2150000;
						}
						chr_err("%s stay fast but decrease 1.5, info->thermal_board_temp = %d, thermal_vote_current = %d\n", __func__, info->thermal_board_temp, thermal_vote_current);
					} else if ((val_capacity.intval > 30) && (info->thermal_board_temp > 409)) {
						if (thermal_vote_current >= 3150000) {
							thermal_vote_current -= 1000000;
						} else {
							thermal_vote_current = 2150000;
						}
						chr_err("%s stay fast but decrease 0.7, info->thermal_board_temp = %d, thermal_vote_current = %d\n", __func__, info->thermal_board_temp, thermal_vote_current);
					} else if ((val_capacity.intval < 30) && (info->thermal_board_temp > 409)) {
						if (thermal_vote_current >= 2650000) {
							thermal_vote_current -= 500000;
						} else {
							thermal_vote_current = 2150000;
						}
						chr_err("%s stay fast but decrease 0.5, info->thermal_board_temp = %d, thermal_vote_current = %d\n", __func__, info->thermal_board_temp, thermal_vote_current);
					} else if ((val_capacity.intval > 30) && (val_capacity.intval < 39) && (info->thermal_board_temp < 390)) {
						if(thermal_vote_current <= 4000000){
							thermal_vote_current += 2000000;
						} else {
							thermal_vote_current = 6000000;
						}
						chr_err("%s stay fast but add 1.5, info->thermal_board_temp = %d, thermal_vote_current = %d\n", __func__, info->thermal_board_temp, thermal_vote_current);
					} else if ((val_capacity.intval > 25) && ((val_capacity.intval <= 30)) && (info->thermal_board_temp < 400)) {
						if(thermal_vote_current <= 2000000){
							thermal_vote_current += 3000000;
						} else {
							thermal_vote_current = 6000000;
						}
						chr_err("%s stay fast but add 2, info->thermal_board_temp = %d, thermal_vote_current = %d\n", __func__, info->thermal_board_temp, thermal_vote_current);
					}*/
					chr_err("%s stay fast, thermal_vote_current = %d\n", __func__, thermal_vote_current);
				} else { //exit fast strategy
					info->pps_fast_mode = false;
					thermal_vote_current = pdata->thermal_charging_current_limit;
					chr_err("%s exit fast, thermal_current = %d\n", __func__, pdata_dvchg->thermal_input_current_limit);
				}
			} else { //capacity > 40, or low_fast was not triggered, use default strategy
				info->b_flag = NROMAL;
				info->pps_fast_mode = false;
				thermal_vote_current = pdata->thermal_charging_current_limit;
				chr_err("%s use default strategy, thermal_current = %d\n", __func__, pdata_dvchg->thermal_input_current_limit);
			}
		} else {
			thermal_vote_current = pdata->thermal_charging_current_limit;
		}
	} else {
		thermal_vote_current = pdata->thermal_charging_current_limit;
	}
	pdata_dvchg->thermal_input_current_limit = thermal_vote_current;
	chr_err("dvchg1_old:%d\n", pdata_dvchg->thermal_input_current_limit);
	if((info->fv_overvoltage_flag) && (pdata_dvchg->thermal_input_current_limit >= 3000))
		info->setting.input_current_limit_dvchg1 = pdata_dvchg->thermal_input_current_limit - 1000000;
	else
		info->setting.input_current_limit_dvchg1 = pdata_dvchg->thermal_input_current_limit;

done:

	ret = charger_dev_get_min_charging_current(info->chg1_dev, &ichg1_min);
	if (ret != -EOPNOTSUPP && pdata->charging_current_limit < ichg1_min) {
		pdata->charging_current_limit = 0;
		/* For TC_018, pleasae don't modify the format */
		chr_err("min_charging_current is too low %d %d\n",
			pdata->charging_current_limit, ichg1_min);
		is_basic = true;
	}

	ret = charger_dev_get_min_input_current(info->chg1_dev, &aicr1_min);
	if (ret != -EOPNOTSUPP && pdata->input_current_limit < aicr1_min) {
		pdata->input_current_limit = 0;
		/* For TC_018, pleasae don't modify the format */
		chr_err("min_input_current is too low %d %d\n",
			pdata->input_current_limit, aicr1_min);
		is_basic = true;
	}
	/* For TC_018, pleasae don't modify the format */
	chr_err("m:%d chg1:%d,%d,%d,%d chg2:%d,%d,%d,%d dvchg1:%d sc:%d %d %d type:%d:%d usb_unlimited:%d usbif:%d usbsm:%d aicl:%d atm:%d bm:%d b:%d\n",
		info->config,
		_uA_to_mA(pdata->thermal_input_current_limit),
		_uA_to_mA(pdata->thermal_charging_current_limit),
		_uA_to_mA(pdata->input_current_limit),
		_uA_to_mA(pdata->charging_current_limit),
		_uA_to_mA(pdata2->thermal_input_current_limit),
		_uA_to_mA(pdata2->thermal_charging_current_limit),
		_uA_to_mA(pdata2->input_current_limit),
		_uA_to_mA(pdata2->charging_current_limit),
		_uA_to_mA(pdata_dvchg->thermal_input_current_limit),
		info->sc.pre_ibat,
		info->sc.sc_ibat,
		info->sc.solution,
		info->chr_type, info->pd_type,
		info->usb_unlimited,
		IS_ENABLED(CONFIG_USBIF_COMPLIANCE), info->usb_state,
		pdata->input_current_limit_by_aicl, info->atm_enabled,
		info->bootmode, is_basic);

	return is_basic;
}

static int do_algorithm(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	struct charger_data *pdata;
	struct chg_alg_notify notify;
	struct mtk_battery *gm;
        struct power_supply *psy = NULL;
	bool is_basic = true;
	bool chg_done = false;
	int i;
	int ret;
	int val = 0;

	pdata = &info->chg_data[CHG1_SETTING];
	charger_dev_is_charging_done(info->chg1_dev, &chg_done);
	is_basic = select_charging_current_limit(info, &info->setting);

	if (info->is_chg_done != chg_done) {
		if (chg_done) {
			charger_dev_do_event(info->chg1_dev, EVENT_FULL, 0);
			info->polling_interval = CHARGING_FULL_INTERVAL;
			chr_err("%s battery full\n", __func__);
		} else {
			charger_dev_do_event(info->chg1_dev, EVENT_RECHARGE, 0);
			info->polling_interval = CHARGING_INTERVAL;
			chr_err("%s battery recharge\n", __func__);
		}
	}

	chr_err("%s is_basic:%d\n", __func__, is_basic);
	if (is_basic != true) {
		is_basic = true;
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;

			if (info->enable_fast_charging_indicator &&
			    ((alg->alg_id & info->fast_charging_indicator) == 0))
				continue;

			if (!info->enable_hv_charging ||
			    pdata->charging_current_limit == 0 ||
			    pdata->input_current_limit == 0) {
				chg_alg_get_prop(alg, ALG_MAX_VBUS, &val);
				if (val > 5000)
					chg_alg_stop_algo(alg);
				chr_err("%s: alg:%s alg_vbus:%d\n", __func__,
					dev_name(&alg->dev), val);
				continue;
			}

			if (chg_done != info->is_chg_done) {
				if (chg_done) {
					notify.evt = EVT_FULL;
					notify.value = 0;
				} else {
					notify.evt = EVT_RECHARGE;
					notify.value = 0;
				}
				chg_alg_notifier_call(alg, &notify);
				chr_err("%s notify:%d\n", __func__, notify.evt);
			}

			chg_alg_set_current_limit(alg, &info->setting);
			ret = chg_alg_is_algo_ready(alg);

			chr_err("%s %s ret:%s\n", __func__,
				dev_name(&alg->dev),
				chg_alg_state_to_str(ret));

			if (ret == ALG_INIT_FAIL || ret == ALG_TA_NOT_SUPPORT) {
				/* try next algorithm */
				continue;
			} else if (ret == ALG_TA_CHECKING || ret == ALG_DONE ||
						ret == ALG_NOT_READY) {
				/* wait checking , use basic first */
				is_basic = true;
				break;
			} else if (ret == ALG_READY || ret == ALG_RUNNING) {
				is_basic = false;
				//chg_alg_set_setting(alg, &info->setting);
				chg_alg_start_algo(alg);
				break;
			} else {
				chr_err("algorithm ret is error");
				is_basic = true;
			}
		}
	} else {
		if (info->enable_hv_charging != true ||
		    pdata->charging_current_limit == 0 ||
		    pdata->input_current_limit == 0) {
			for (i = 0; i < MAX_ALG_NO; i++) {
				alg = info->alg[i];
				if (alg == NULL)
					continue;

				chg_alg_get_prop(alg, ALG_MAX_VBUS, &val);
				if (val > 5000 && chg_alg_is_algo_running(alg))
					chg_alg_stop_algo(alg);

				chr_err("%s: Stop hv charging. en_hv:%d alg:%s alg_vbus:%d\n",
					__func__, info->enable_hv_charging,
					dev_name(&alg->dev), val);
			}
		}
	}

	if (is_basic == true) {
		charger_dev_set_input_current(info->chg1_dev,
			pdata->input_current_limit);
		charger_dev_set_charging_current(info->chg1_dev,
			pdata->charging_current_limit);

		chr_err("%s:old_cv=%d,cv=%d, old_iterm=%d,iterm=%d, vbat_mon_en=%d\n",
			__func__,
			info->old_cv,
			info->setting.cv,
                        info->old_iterm,
                        info->bat.iterm,
			info->setting.vbat_mon_en);
		if (info->old_cv == 0 || (info->old_cv != info->setting.cv)
		    || info->setting.vbat_mon_en == 0) {
			charger_dev_enable_6pin_battery_charging(
				info->chg1_dev, false);
			if (info->pd_type != MTK_PD_CONNECT_PE_READY_SNK_APDO)
				charger_dev_set_constant_voltage(info->chg1_dev, info->setting.cv);
			if (info->setting.vbat_mon_en && info->stop_6pin_re_en != 1)
				charger_dev_enable_6pin_battery_charging(
					info->chg1_dev, true);
			info->old_cv = info->setting.cv;
		} else {
			if (info->setting.vbat_mon_en && info->stop_6pin_re_en != 1) {
				info->stop_6pin_re_en = 1;
				charger_dev_enable_6pin_battery_charging(
					info->chg1_dev, true);
			}
		}

		if ((info->old_iterm == 0) || (info->old_iterm != info->bat.iterm)) {
				charger_dev_set_eoc_current(info->chg1_dev, (info->bat.iterm) * 1000);
				info->old_iterm = info->bat.iterm;
		}

		chr_info("chg_done=%d, info->is_chg_done=%d\n", chg_done, info->is_chg_done);
		if (chg_done != info->is_chg_done) {
			if (chg_done && (info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30
				|| info->pd_type == MTK_PD_CONNECT_PE_READY_SNK || info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO)) {
					adapter_dev_set_cap_xm(info->pd_adapter, MTK_PD_APDO_END, PD30_VTA_RESET, PD30_ITA_RESET);
					chr_err("charging done, reset pd adapter\n");
			}
		}
	}

	info->is_chg_done = chg_done;

	if (pdata->input_current_limit == 0 ||
	    pdata->charging_current_limit == 0)
		charger_dev_enable(info->chg1_dev, false);
	else {
		alg = get_chg_alg_by_name("pe5");
		ret = chg_alg_is_algo_ready(alg);
		psy = power_supply_get_by_name("battery");
		if (psy != NULL) {
			gm = (struct mtk_battery *)power_supply_get_drvdata(psy);
			if (gm != NULL){
				chr_err("get mtk_battery drvdata success in %s\n", __func__);
			}
		} else {
			return 0;
		}
		chr_err("xm %s enable charger ret = %d, active_status = %d\n", __func__, ret,
			gm->smart_charge[SMART_CHG_NAVIGATION].active_status);
		if (!(ret == ALG_READY || ret == ALG_RUNNING) && (info->bat.charge_full == false) && !gm->smart_charge[SMART_CHG_NAVIGATION].active_status) {
			charger_dev_enable(info->chg1_dev, true);
			chr_err("xm %s enable charger ret = %d\n", __func__, ret);
		}
	}

	if (info->chg1_dev != NULL)
		charger_dev_dump_registers(info->chg1_dev);

	if (info->chg2_dev != NULL)
		charger_dev_dump_registers(info->chg2_dev);

	return 0;
}

static int enable_charging(struct mtk_charger *info,
						bool en)
{
	int i;
	struct chg_alg_device *alg;
	struct mtk_battery *gm = NULL;
	struct power_supply *psy = NULL;

	chr_err("%s %d\n", __func__, en);

	psy = power_supply_get_by_name("battery");
	if (psy != NULL) {
		gm = (struct mtk_battery *)power_supply_get_drvdata(psy);
		if (gm != NULL){
			chr_err("get mtk_battery drvdata success in %s\n", __func__);
		}
	} else {
		return 0;
	}

	if (en == false) {
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;
			chg_alg_stop_algo(alg);
		}
		charger_dev_enable(info->chg1_dev, false);
		charger_dev_do_event(info->chg1_dev, EVENT_DISCHARGE, 0);
	} else {
		chr_err("xm in %s enable charger, active_status = %d\n", __func__,
			gm->smart_charge[SMART_CHG_NAVIGATION].active_status);
		if (!gm->smart_charge[SMART_CHG_NAVIGATION].active_status) {
			charger_dev_enable(info->chg1_dev, true);
			charger_dev_do_event(info->chg1_dev, EVENT_RECHARGE, 0);
		}
	}

	return 0;
}

static int charger_dev_event(struct notifier_block *nb, unsigned long event,
				void *v)
{
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	struct mtk_charger *info =
			container_of(nb, struct mtk_charger, chg1_nb);
	struct chgdev_notify *data = v;
	int i;

	chr_err("%s %lu\n", __func__, event);

	switch (event) {
	case CHARGER_DEV_NOTIFY_EOC:
		info->stop_6pin_re_en = 1;
		notify.evt = EVT_FULL;
		notify.value = 0;
		for (i = 0; i < 10; i++) {
			alg = info->alg[i];
			chg_alg_notifier_call(alg, &notify);
		}

		break;
	case CHARGER_DEV_NOTIFY_RECHG:
		pr_info("%s: recharge\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT:
		info->safety_timeout = true;
		pr_info("%s: safety timer timeout\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_VBUS_OVP:
		info->vbusov_stat = data->vbusov_stat;
		pr_info("%s: vbus ovp = %d\n", __func__, info->vbusov_stat);
		break;
	case CHARGER_DEV_NOTIFY_BATPRO_DONE:
		info->batpro_done = true;
		info->setting.vbat_mon_en = 0;
		notify.evt = EVT_BATPRO_DONE;
		notify.value = 0;
		for (i = 0; i < 10; i++) {
			alg = info->alg[i];
			chg_alg_notifier_call(alg, &notify);
		}
		pr_info("%s: batpro_done = %d\n", __func__, info->batpro_done);
		break;
	default:
		return NOTIFY_DONE;
	}

	if (info->chg1_dev->is_polling_mode == false)
		_wake_up_charger(info);

	return NOTIFY_DONE;
}

static int to_alg_notify_evt(unsigned long evt)
{
	switch (evt) {
	case CHARGER_DEV_NOTIFY_VBUS_OVP:
		return EVT_VBUSOVP;
	case CHARGER_DEV_NOTIFY_IBUSOCP:
		return EVT_IBUSOCP;
	case CHARGER_DEV_NOTIFY_IBUSUCP_FALL:
		return EVT_IBUSUCP_FALL;
	case CHARGER_DEV_NOTIFY_BAT_OVP:
		return EVT_VBATOVP;
	case CHARGER_DEV_NOTIFY_IBATOCP:
		return EVT_IBATOCP;
	case CHARGER_DEV_NOTIFY_VBATOVP_ALARM:
		return EVT_VBATOVP_ALARM;
	case CHARGER_DEV_NOTIFY_VBUSOVP_ALARM:
		return EVT_VBUSOVP_ALARM;
	case CHARGER_DEV_NOTIFY_VOUTOVP:
		return EVT_VOUTOVP;
	case CHARGER_DEV_NOTIFY_VDROVP:
		return EVT_VDROVP;
	default:
		return -EINVAL;
	}
}

static int dvchg1_dev_event(struct notifier_block *nb, unsigned long event,
			    void *data)
{
	struct mtk_charger *info =
		container_of(nb, struct mtk_charger, dvchg1_nb);
	int alg_evt = to_alg_notify_evt(event);

	chr_info("%s %ld", __func__, event);
	if (alg_evt < 0)
		return NOTIFY_DONE;
	mtk_chg_alg_notify_call(info, alg_evt, 0);
	return NOTIFY_OK;
}

static int dvchg2_dev_event(struct notifier_block *nb, unsigned long event,
			    void *data)
{
	struct mtk_charger *info =
		container_of(nb, struct mtk_charger, dvchg1_nb);
	int alg_evt = to_alg_notify_evt(event);

	chr_info("%s %ld", __func__, event);
	if (alg_evt < 0)
		return NOTIFY_DONE;
	mtk_chg_alg_notify_call(info, alg_evt, 0);
	return NOTIFY_OK;
}


int mtk_basic_charger_init(struct mtk_charger *info)
{

	info->algo.do_algorithm = do_algorithm;
	info->algo.enable_charging = enable_charging;
	info->algo.do_event = charger_dev_event;
	info->algo.do_dvchg1_event = dvchg1_dev_event;
	info->algo.do_dvchg2_event = dvchg2_dev_event;
	//info->change_current_setting = mtk_basic_charging_current;
	return 0;
}
