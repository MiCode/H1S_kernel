/*
 * Copyright (C) 2023 Nuvolta Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/version.h>

#define CONFIG_MTK_CLASS

#ifdef CONFIG_MTK_CLASS
#include "charger_class.h"
#endif /*CONFIG_MTK_CLASS*/

#include "nu2115.h"
#include "nu2115_reg.h"

#ifdef CONFIG_MTK_CLASS
static const struct charger_properties nu2115_chg_props = {
	.alias_name = "nu2115_chg",
};
#endif /*CONFIG_MTK_CLASS*/

/************************************************************************/
static int __nu2115_read_byte(struct nu2115 *chip, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		nu_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	if (reg == NU2115_REG_2F)
		nu_info("read reg 0x2f:%02X\n", *data);

	return 0;
}

static int __nu2115_write_byte(struct nu2115 *chip, int reg, u8 val)
{
	s32 ret;

	if (reg == NU2115_REG_2F)
		nu_info("write reg 0x2f:%02X\n", val);

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		nu_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int nu2115_read_byte(struct nu2115 *chip, u8 reg, u8 *data)
{
	int ret;

	if (chip->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&chip->i2c_rw_lock);
	ret = __nu2115_read_byte(chip, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int nu2115_write_byte(struct nu2115 *chip, u8 reg, u8 data)
{
	int ret;

	if (chip->skip_writes)
		return 0;

	mutex_lock(&chip->i2c_rw_lock);
	ret = __nu2115_write_byte(chip, reg, data);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int nu2115_update_bits(struct nu2115*chip, u8 reg,
		u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (chip->skip_reads || chip->skip_writes)
		return 0;

	mutex_lock(&chip->i2c_rw_lock);
	ret = __nu2115_read_byte(chip, reg, &tmp);
	if (ret) {
		nu_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __nu2115_write_byte(chip, reg, tmp);
	if (ret)
		nu_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&chip->i2c_rw_lock);
	return ret;
}

/*********************************************************************/
__maybe_unused static int nu2115_disable_cp_ts_detect(struct nu2115 *chip)
{
	int ret;

	nu_err("nu2115_disable_cp_ts_detect\n");
	ret = nu2115_write_byte(chip, NU2115_REG_0E,
			0x06);

	return ret;
}

__maybe_unused void nu2115_dump(struct nu2115 *chip)
{
	u8 addr;
	u8 val;
	int ret;
	for (addr = 0x0; addr <= 0x36; addr++) {
		ret = nu2115_read_byte(chip, addr, &val);
		if (ret == 0) {
			nu_err("nu2115 reg[%02X]=%02X \n", addr, val);
		}
	}
}

static int nu2115_init_device(struct nu2115 *chip);
__maybe_unused static int nu2115_enable_charge(struct nu2115 *chip, bool en)
{
	int ret;
	u8 val;

	if (en) {
		ret = nu2115_write_byte(chip, NU2115_REG_32, 0x70);
		val = NU2115_CHG_ENABLE;
	} else {
		val = NU2115_CHG_DISABLE;
	}

	val <<= NU2115_CHG_EN_SHIFT;

	nu_err("nu2115 charger %s\n", en == false ? "disable" : "enable");

	ret = nu2115_update_bits(chip, NU2115_REG_0E, NU2115_CHG_EN_MASK, val);

	if (en) {
		msleep(30);
		ret = nu2115_write_byte(chip, NU2115_REG_32, 0x50);
	}

	return ret;
}

__maybe_unused static int nu2115_check_charge_enabled(struct nu2115 *chip, bool *en)
{
	int ret;
	u8 val;

	ret = nu2115_read_byte(chip, NU2115_REG_0E, &val);
	nu_err(">>>reg [0x0E] = 0x%02x\n", val);
	if (!ret)
		*en = !!(val & NU2115_CHG_EN_MASK);

	return ret;
}

__maybe_unused static int nu2115_enable_wdt(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = NU2115_WATCHDOG_ENABLE;
	else
		val = NU2115_WATCHDOG_DISABLE;

	val <<= NU2115_WATCHDOG_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_0D,
			NU2115_WATCHDOG_DIS_MASK, val);

	return ret;
}

__maybe_unused static int nu2115_set_reg_reset(struct nu2115 *chip)
{
	int ret;
	u8 val = 1;

	val = NU2115_REG_RST_ENABLE;

	val <<= NU2115_REG_RST_DISABLE;

	ret = nu2115_update_bits(chip, NU2115_REG_0D,
			NU2115_REG_RST_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_enable_batovp(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = NU2115_BAT_OVP_DISABLE;
	else
		val = NU2115_BAT_OVP_ENABLE;

	val <<= NU2115_BAT_OVP_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_00,
			NU2115_BAT_OVP_DIS_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_batovp_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BAT_OVP_BASE)
		threshold = NU2115_BAT_OVP_BASE;

	val = (threshold - NU2115_BAT_OVP_BASE) *10/ NU2115_BAT_OVP_LSB;

	val <<= NU2115_BAT_OVP_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_00,
			NU2115_BAT_OVP_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_enable_batovp_alarm(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = NU2115_BAT_OVP_ALM_ENABLE;
	else
		val = NU2115_BAT_OVP_ALM_DISABLE;

	val <<= NU2115_BAT_OVP_ALM_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_01,
			NU2115_BAT_OVP_ALM_DIS_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_batovp_alarm_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BAT_OVP_ALM_BASE)
		threshold = NU2115_BAT_OVP_ALM_BASE;

	val = (threshold - NU2115_BAT_OVP_ALM_BASE) / NU2115_BAT_OVP_ALM_LSB;

	val <<= NU2115_BAT_OVP_ALM_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_01,
			NU2115_BAT_OVP_ALM_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_enable_batocp(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = NU2115_BAT_OCP_DISABLE;
	else
		val = NU2115_BAT_OCP_ENABLE;

	val <<= NU2115_BAT_OCP_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_02,
			NU2115_BAT_OCP_DIS_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_batocp_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BAT_OCP_BASE)
		threshold = NU2115_BAT_OCP_BASE;

	val = (threshold - NU2115_BAT_OCP_BASE) / NU2115_BAT_OCP_LSB;

	val <<= NU2115_BAT_OCP_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_02,
			NU2115_BAT_OCP_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_enable_batocp_alarm(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = NU2115_BAT_OCP_ALM_ENABLE;
	else
		val = NU2115_BAT_OCP_ALM_DISABLE;

	val <<= NU2115_BAT_OCP_ALM_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_03,
			NU2115_BAT_OCP_ALM_DIS_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_batocp_alarm_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BAT_OCP_ALM_BASE)
		threshold = NU2115_BAT_OCP_ALM_BASE;

	val = (threshold - NU2115_BAT_OCP_ALM_BASE) / NU2115_BAT_OCP_ALM_LSB;

	val <<= NU2115_BAT_OCP_ALM_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_03,
			NU2115_BAT_OCP_ALM_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_busovp_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BUS_OVP_BASE)
		threshold = NU2115_BUS_OVP_BASE;

	val = (threshold - NU2115_BUS_OVP_BASE) / NU2115_BUS_OVP_LSB;

	val <<= NU2115_BUS_OVP_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_07,
			NU2115_BUS_OVP_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_enable_busovp_alarm(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = NU2115_BUS_OVP_ALM_ENABLE;
	else
		val = NU2115_BUS_OVP_ALM_DISABLE;

	val <<= NU2115_BUS_OVP_ALM_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_08,
			NU2115_BUS_OVP_ALM_DIS_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_busovp_alarm_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BUS_OVP_ALM_BASE)
		threshold = NU2115_BUS_OVP_ALM_BASE;

	val = (threshold - NU2115_BUS_OVP_ALM_BASE) / NU2115_BUS_OVP_ALM_LSB;

	val <<= NU2115_BUS_OVP_ALM_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_08,
			NU2115_BUS_OVP_ALM_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_enable_busocp(struct nu2115 *chip, bool enable)
{
	int ret = 0;
	u8 val;

	if (enable)
		val = NU2115_BUS_OCP_DISABLE;
	else
		val = NU2115_BUS_OCP_ENABLE;

	val <<= NU2115_BUS_OCP_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_09,
			NU2115_BUS_OCP_DIS_MASK, val);

	return ret;
}

__maybe_unused static int nu2115_set_busocp_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BUS_OCP_BASE)
		threshold = NU2115_BUS_OCP_BASE;

	val = (threshold - NU2115_BUS_OCP_BASE) / (NU2115_BUS_OCP_LSB);

	val <<= NU2115_BUS_OCP_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_09,
			NU2115_BUS_OCP_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_enable_busocp_alarm(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = NU2115_BUS_OCP_ALM_ENABLE;
	else
		val = NU2115_BUS_OCP_ALM_DISABLE;

	val <<= NU2115_BUS_OCP_ALM_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_0A,
			NU2115_BUS_OCP_ALM_DIS_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_busocp_alarm_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BUS_OCP_ALM_BASE)
		threshold = NU2115_BUS_OCP_ALM_BASE;

	val = (threshold - NU2115_BUS_OCP_ALM_BASE) / NU2115_BUS_OCP_ALM_LSB;

	val <<= NU2115_BUS_OCP_ALM_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_0A,
			NU2115_BUS_OCP_ALM_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_enable_batucp_alarm(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = NU2115_BAT_UCP_ALM_ENABLE;
	else
		val = NU2115_BAT_UCP_ALM_DISABLE;

	val <<= NU2115_BAT_UCP_ALM_DIS_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_04,
			NU2115_BAT_UCP_ALM_DIS_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_batucp_alarm_th(struct nu2115 *chip, int threshold)
{
	int ret;
	u8 val;

	if (threshold < NU2115_BAT_UCP_ALM_BASE)
		threshold = NU2115_BAT_UCP_ALM_BASE;

	val = (threshold - NU2115_BAT_UCP_ALM_BASE) / NU2115_BAT_UCP_ALM_LSB;

	val <<= NU2115_BAT_UCP_ALM_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_04,
			NU2115_BAT_UCP_ALM_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_reg_ctrl_i2c(struct nu2115 *chip)
{
	int ret;

	ret = nu2115_write_byte(chip, 0xF7, 0x00);
	ret = nu2115_write_byte(chip, 0xF7, 0x78);
	ret = nu2115_write_byte(chip, 0xF7, 0x87);
	ret = nu2115_write_byte(chip, 0xF7, 0xAA);
	ret = nu2115_write_byte(chip, 0xF7, 0x55);
	ret = nu2115_write_byte(chip, 0x8C, 0x20);

	return ret;
}

__maybe_unused static int nu2115_set_acovp_th(struct nu2115 *chip, int threshold)
{
	int ret = 0;
	u8 val;

	if (threshold < NU2115_AC1_OVP_BASE)
		threshold = NU2115_AC1_OVP_BASE;

	if (threshold == NU2115_AC1_OVP_6P5V)
		val = 0x07;
	else
		val = (threshold - NU2115_AC1_OVP_BASE) /  NU2115_AC1_OVP_LSB;

	val <<= NU2115_AC1_OVP_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_05,
			NU2115_AC1_OVP_MASK, val);

	return ret;
}

__maybe_unused static int nu2115_enable_otg(struct nu2115 *chip, bool en)
{
	int ret = 0;
	u8 val;

	if (en)
		val = 0x02;
	else
		val = 0x00;

	ret = nu2115_write_byte(chip, NU2115_REG_2F, val);
	return ret;
}

__maybe_unused static int nu2115_check_enable_otg(struct nu2115 *chip)
{
	int ret;
	u8 val;

	ret = nu2115_read_byte(chip, NU2115_REG_2F, &val);
	nu_info(">>>reg [0x2F] = 0x%02x\n", val);
	if (!ret)
		ret = !!(val & NU2115_EN_OTG_MASK);
	return ret;
}

__maybe_unused static int nu2115_enable_acdrv1(struct nu2115 *chip, bool en)
{
	int ret = 0;
	u8 val;

	if (en)
		val = NU2115_ACDRV1_ENABLE;
	else
		val = NU2115_ACDRV1_DISABLE;

	val <<= NU2115_EN_ACDRV1_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_30,
			NU2115_EN_ACDRV1_MASK, val);

	return ret;
}

__maybe_unused static int nu2115_check_enable_acdrv1(struct nu2115 *chip, bool *enable)
{
	int ret;
	u8 val;

	ret = nu2115_read_byte(chip, NU2115_REG_30, &val);
	nu_info(">>>reg [0x30] = 0x%02x\n", val);
	if (!ret)
		*enable = !!(val & NU2115_EN_ACDRV1_MASK);
	return ret;
}

__maybe_unused static int nu2115_enable_acdrv2(struct nu2115 *chip, bool enable)
{
	int ret = 0;
	u8 val;

	if (enable)
		val = NU2115_ACDRV2_ENABLE;
	else
		val = NU2115_ACDRV2_DISABLE;

	val <<= NU2115_EN_ACDRV2_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_30,
			NU2115_EN_ACDRV2_MASK, val);

	return ret;
}

__maybe_unused static int nu2115_check_enable_acdrv2(struct nu2115 *chip, bool *enable)
{
	int ret;
	u8 val;

	ret = nu2115_read_byte(chip, NU2115_REG_30, &val);
	nu_info(">>>reg [0x30] = 0x%02x\n", val);
	if (!ret)
		*enable = !!(val & NU2115_EN_ACDRV2_MASK);
	return ret;
}

__maybe_unused static int nu2115_enable_adc(struct nu2115 *chip, bool enable)
{
	int ret;
	u8 val;

	dev_err(chip->dev,"nu2115 set adc :%d\n", enable);

	if (enable)
		val = NU2115_ADC_ENABLE;
	else
		val = NU2115_ADC_DISABLE;

	val <<= NU2115_ADC_EN_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_15,
			NU2115_ADC_EN_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_set_adc_scanrate(struct nu2115 *chip, bool oneshot)
{
	int ret;
	u8 val;

	if (oneshot)
		val = NU2115_ADC_RATE_ONESHOT;
	else
		val = NU2115_ADC_RATE_CONTINOUS;

	val <<= NU2115_ADC_RATE_SHIFT;

	ret = nu2115_update_bits(chip, NU2115_REG_15,
			NU2115_ADC_RATE_MASK, val);
	return ret;
}

__maybe_unused static int nu2115_get_adc_data(struct nu2115 *chip, int channel, int *data)
{
	int ret;
	u8 val_l, val_h;
	u16 val;
	struct power_supply *bat_psy = NULL;
	union power_supply_propval val_new;

	if(channel >= ADC_MAX_NUM) return -EINVAL;

	ret = nu2115_read_byte(chip, ADC_REG_BASE + (channel << 1), &val_h);
	ret = nu2115_read_byte(chip, ADC_REG_BASE + (channel << 1) + 1, &val_l);

	if (ret < 0)
		return ret;
	val = (val_h << 8) | val_l;

	if((channel == ADC_TSBUS) || (channel == ADC_TSBAT))
		val = val * 9766/100000;
	else if(channel == ADC_TDIE)
		val = val * 5/10;

	*data = val;

	if (channel == ADC_IBAT) {
		bat_psy = power_supply_get_by_name("battery");
		if (IS_ERR_OR_NULL(bat_psy)) {
			dev_info(chip->dev,"%s: failed to get battery psy\n", __func__);
			return 0;
		} else {
			ret = power_supply_get_property(bat_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val_new);
			*data = val_new.intval / 1000;
		}
	}

	nu_err("[nu2115_get_adc_data]:%d %d", channel, *data);

	return ret;
}

__maybe_unused static int nu2115_set_adc_scan(struct nu2115 *chip, int channel, bool enable)
{
	int ret;
	u8 reg;
	u8 mask;
	u8 shift;
	u8 val;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

	if (channel == ADC_IBUS) {
		reg = NU2115_REG_15;
		shift = NU2115_IBUS_ADC_DIS_SHIFT;
		mask = NU2115_IBUS_ADC_DIS_MASK;
	} else if (channel == ADC_VBUS) {
		reg = NU2115_REG_15;
		shift = NU2115_IBUS_ADC_DIS_SHIFT;
		mask = NU2115_IBUS_ADC_DIS_MASK;
	} else {
		reg = NU2115_REG_16;
		shift = 9 - channel;
		mask = 1 << shift;
	}

	if (enable)
		val = 0 << shift;
	else
		val = 1 << shift;

	ret = nu2115_update_bits(chip, reg, mask, val);

	return ret;
}

__maybe_unused static int nu2115_set_ss_timeout(struct nu2115 *chip, int timeout)
{
	int ret =0;
	u8 val;

	switch (timeout) {
		case 0:
			val = NU2115_SS_TIMEOUT_DISABLE;
			break;
		case 12:
			val = NU2115_SS_TIMEOUT_12P5MS;
			break;
		case 25:
			val = NU2115_SS_TIMEOUT_25MS;
			break;
		case 50:
			val = NU2115_SS_TIMEOUT_50MS;
			break;
		case 100:
			val = NU2115_SS_TIMEOUT_100MS;
			break;
		case 400:
			val = NU2115_SS_TIMEOUT_400MS;
			break;
		case 1500:
			val = NU2115_SS_TIMEOUT_1500MS;
			break;
		case 100000:
			val = NU2115_SS_TIMEOUT_100000MS;
			break;
		default:
			val = NU2115_SS_TIMEOUT_DISABLE;
			break;
	}

	val <<= NU2115_SS_TIMEOUT_SET_SHIFT;;

	ret = nu2115_update_bits(chip, NU2115_REG_2E,
			NU2115_SS_TIMEOUT_SET_MASK,
			val);

	return ret;
}

__maybe_unused static int nu2115_check_vbus_error_status(struct nu2115 *chip, uint32_t *state)
{
	int ret;
	u8 data;

	ret = nu2115_read_byte(chip, NU2115_REG_35, &data);
	if(ret == 0){
		chip->vbus_error_low = !!(data & (1 << NU2115_VBUS_ERRLO_STAT_SHIFT));
		chip->vbus_error_high = !!(data & (1 << NU2115_VBUS_ERRHI_STAT_SHIFT));
		nu_info("vbus_error_low:%d, vbus_error_high:%d \n", chip->vbus_error_low, chip->vbus_error_high);
	}

	if (chip->vbus_error_low != 0)
		*state |= BIT(VBUS_ERROR_L);

	if (chip->vbus_error_high != 0)
		*state |= BIT(VBUS_ERROR_H);

	return ret;
}

__maybe_unused static int nu2115_is_vbuslowerr(struct nu2115 *chip, bool *err)
{
	int ret;
	u8 data;

	ret = nu2115_read_byte(chip, NU2115_REG_35, &data);
	if(ret == 0){
		chip->vbus_error_low = !!(data & (1 << NU2115_VBUS_ERRLO_STAT_SHIFT));
		nu_info("vbus_error_low:%d,\n", chip->vbus_error_low);
	}

    *err = (bool)chip->vbus_error_low;

	return ret;
}


__maybe_unused static int nu2115_detect_device(struct nu2115 *chip)
{
	int ret;
	u8 data;

	ret = nu2115_read_byte(chip, NU2115_REG_31, &data);
	if (ret == 0) {
		chip->part_no = (data & NU2115_DEV_ID_MASK);
		chip->part_no >>= NU2115_DEV_ID_SHIFT;
	}

	return ret;
}

void nu2115_check_alarm_status(struct nu2115 *chip)
{
	int ret1;
	int ret2;
	u8 flag = 0;
	u8 stat1 = 0;
	u8 stat2 = 0;

	mutex_lock(&chip->data_lock);

	/*read to clear alarm flag*/
	ret1 = nu2115_read_byte(chip, NU2115_REG_10, &flag);
	if (!ret1 && flag)
		nu_dbg("INT_FLAG =0x%02X\n", flag);

	ret1 = nu2115_read_byte(chip, NU2115_REG_0F, &stat1);
	ret2 = nu2115_read_byte(chip, NU2115_REG_36, &stat2);

	if (!ret1 && !ret2 && stat1 != chip->prev_alarm && stat2 != chip->prev_alarm) {
		nu_dbg("INT_STAT = stat1 0X%02x, stat2 0X%02x\n", stat1, stat2);
		chip->prev_alarm = stat1;
		chip->bat_ovp_alarm = !!(stat1 & BAT_OVP_ALARM);
		chip->bat_ocp_alarm = !!(stat1 & BAT_OCP_ALARM);
		chip->bus_ovp_alarm = !!(stat1 & BUS_OVP_ALARM);
		chip->bus_ocp_alarm = !!(stat1 & BUS_OCP_ALARM);
		chip->batt_present  = !!(stat1 & VBAT_INSERT);
		chip->bat_ucp_alarm = !!(stat1 & BAT_UCP_ALARM);
		chip->vbus_present  = !!(stat2 & VBUS_INSERT);
	}

	mutex_unlock(&chip->data_lock);
}

void nu2115_check_fault_status(struct nu2115 *chip)
{
	int ret;
	u8 flag = 0;
	u8 stat = 0;
	bool changed = false;

	mutex_lock(&chip->data_lock);
	ret = nu2115_read_byte(chip, NU2115_REG_05, &stat);
	if (!ret && stat)
		nu_err("FAULT_STAT_05 = 0x%02X\n", stat);

	ret = nu2115_read_byte(chip, NU2115_REG_06, &stat);
	if (!ret && stat)
		nu_err("FAULT_STAT_06 = 0x%02X\n", stat);

	ret = nu2115_read_byte(chip, NU2115_REG_09, &stat);
	if (!ret && stat)
		nu_err("FAULT_STAT_09 = 0x%02X\n", stat);

	ret = nu2115_read_byte(chip, NU2115_REG_0A, &stat);
	if (!ret && stat)
		nu_err("FAULT_STAT_0a = 0x%02X\n", stat);

	ret = nu2115_read_byte(chip, NU2115_REG_0B, &stat);
	if (!ret && stat)
		nu_err("FAULT_STAT_0b = 0x%02X\n", stat);

	ret = nu2115_read_byte(chip, NU2115_REG_0C, &stat);
	if (!ret && stat)
		nu_err("FAULT_STAT_0c = 0x%02X\n", stat);

	ret = nu2115_read_byte(chip, NU2115_REG_12, &stat);
	if (!ret && stat)
		nu_err("FAULT_STAT_12 = 0x%02X\n", stat);

	ret = nu2115_read_byte(chip, NU2115_REG_13, &flag);
	if (!ret && flag)
		nu_err("FAULT_FLAG_13 = 0x%02X\n", flag);

	ret = nu2115_read_byte(chip, NU2115_REG_32, &stat);
	if (!ret && stat)
		nu_err("FAULT_FLAG_32 = 0x%02X\n", stat);

	ret = nu2115_read_byte(chip, NU2115_REG_35, &stat);
	if (!ret && stat)
		nu_err("FAULT_FLAG_35 = 0x%02X\n", stat);

	if (!ret && flag != chip->prev_fault) {
		changed = true;
		chip->prev_fault = flag;
		chip->bat_ovp_fault = !!(flag & BAT_OVP_FAULT);
		chip->bat_ocp_fault = !!(flag & BAT_OCP_FAULT);
		chip->bus_ovp_fault = !!(flag & BUS_OVP_FAULT);
		chip->bus_ocp_fault = !!(flag & BUS_OCP_FAULT);
		chip->bat_therm_fault = !!(flag & TS_BAT_FAULT);
		chip->bus_therm_fault = !!(flag & TS_BUS_FAULT);

		chip->bat_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
		chip->bus_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
	}

	mutex_unlock(&chip->data_lock);
}

static ssize_t nu2115_show_registers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct nu2115 *chip = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[400];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "nu2115");
	for (addr = 0x0; addr <= 0x36; addr++) {
		ret = nu2115_read_byte(chip, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
					"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t nu2115_store_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct nu2115 *chip = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x31)
		nu2115_write_byte(chip, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, 0660, nu2115_show_registers, nu2115_store_register);

static void nu2115_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}

__maybe_unused static int nu2115_init_adc(struct nu2115 *chip)
{
	nu2115_set_adc_scanrate(chip, false);
	nu2115_set_adc_scan(chip, ADC_IBUS, true);
	nu2115_set_adc_scan(chip, ADC_VBUS, true);
	nu2115_set_adc_scan(chip, ADC_VOUT, true);
	nu2115_set_adc_scan(chip, ADC_VBAT, true);
	nu2115_set_adc_scan(chip, ADC_IBAT, true);
	nu2115_set_adc_scan(chip, ADC_TSBUS, true);
	nu2115_set_adc_scan(chip, ADC_TSBAT, true);
	nu2115_set_adc_scan(chip, ADC_TDIE, true);
	nu2115_set_adc_scan(chip, ADC_VAC1, true);
	nu2115_set_adc_scan(chip, ADC_VAC2, true);

	return 0;
}

__maybe_unused static int nu2115_init_int_src(struct nu2115 *chip)
{
	return 0;
}

__maybe_unused static int nu2115_init_protection(struct nu2115 *chip)
{
	int ret;

	ret = nu2115_enable_batovp(chip, chip->cfg->bat_ovp_disable);
	nu_err("%s bat ovp %s\n",
			chip->cfg->bat_ovp_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = nu2115_enable_batocp(chip, chip->cfg->bat_ocp_disable);
	nu_err("%s bat ocp %s\n",
			chip->cfg->bat_ocp_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = nu2115_enable_busocp(chip, chip->cfg->bus_ocp_disable);
	nu_err("%s bus ocp %s\n",
			chip->cfg->bus_ocp_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = nu2115_set_batovp_th(chip, chip->cfg->bat_ovp_th);
	nu_err("set bat ovp th %d %s\n", chip->cfg->bat_ovp_th,
			!ret ? "successfully" : "failed");

	ret = nu2115_set_batocp_th(chip, chip->cfg->bat_ocp_th);
	nu_err("set bat ocp threshold %d %s\n", chip->cfg->bat_ocp_th,
			!ret ? "successfully" : "failed");

	ret = nu2115_set_batocp_alarm_th(chip, chip->cfg->bat_ocp_alm_th);
	nu_err("set bat ocp alarm threshold %d %s\n", chip->cfg->bat_ocp_alm_th,
			!ret ? "successfully" : "failed");

	ret = nu2115_set_busovp_th(chip, chip->cfg->bus_ovp_th);
	nu_err("set bus ovp threshold %d %s\n", chip->cfg->bus_ovp_th,
			!ret ? "successfully" : "failed");

	ret = nu2115_set_busovp_alarm_th(chip, chip->cfg->bus_ovp_alm_th);
	nu_err("set bus ovp alarm threshold %d %s\n", chip->cfg->bus_ovp_alm_th,
			!ret ? "successfully" : "failed");

	ret = nu2115_set_busocp_th(chip, chip->cfg->bus_ocp_th);
	nu_err("set bus ocp threshold %d %s\n", chip->cfg->bus_ocp_th,
			!ret ? "successfully" : "failed");

	ret = nu2115_set_busocp_alarm_th(chip, chip->cfg->bus_ocp_alm_th);
	nu_err("set bus ocp alarm th %d %s\n", chip->cfg->bus_ocp_alm_th,
			!ret ? "successfully" : "failed");

	ret = nu2115_set_reg_ctrl_i2c(chip);
	nu_err("ctlr i2c %s\n",!ret ? "successfully" : "failed");

	ret = nu2115_set_acovp_th(chip, chip->cfg->ac_ovp_th);
	nu_err("set ac ovp threshold %d %s\n", chip->cfg->ac_ovp_th,
			!ret ? "successfully" : "failed");
	return 0;
}

__maybe_unused static int nu2115_init_regulation(struct nu2115 *chip)
{
	return 0;
}

__maybe_unused static int nu2115_init_device(struct nu2115 *chip)
{
	nu2115_set_reg_reset(chip);
	nu2115_enable_wdt(chip, false);
	nu2115_disable_cp_ts_detect(chip);
	nu2115_set_batocp_alarm_th(chip, 12000);
	nu2115_set_batovp_alarm_th(chip, 5390);

	nu2115_init_protection(chip);
	nu2115_init_adc(chip);
	nu2115_init_int_src(chip);

	nu2115_init_regulation(chip);
	nu2115_write_byte(chip, NU2115_REG_00, 0x47);
	nu2115_write_byte(chip, NU2115_REG_33, 0x0D);
	nu2115_write_byte(chip, NU2115_REG_11, 0xF8);

	return 0;
}

__maybe_unused static int nu2115_set_present(struct nu2115 *chip, bool present)
{
	chip->usb_present = present;

	if (present)
		nu2115_init_device(chip);
	return 0;
}


static irqreturn_t nu2115_irq_handler(int irq, void *data)
{
	struct nu2115 *chip = data;

	dev_err(chip->dev,"INT OCCURED\n");

	nu2115_check_fault_status(chip);

	power_supply_changed(chip->psy);

	return IRQ_HANDLED;
}

__maybe_unused static int nu2115_register_interrupt(struct nu2115 *chip)
{
	int ret;

	if (gpio_is_valid(chip->irq_gpio)) {
		ret = gpio_request_one(chip->irq_gpio, GPIOF_DIR_IN,"nu2115_irq");
		if (ret) {
			dev_err(chip->dev,"failed to request nu2115_irq\n");
			return -EINVAL;
		}
		chip->irq = gpio_to_irq(chip->irq_gpio);
		if (chip->irq < 0) {
			dev_err(chip->dev,"failed to gpio_to_irq\n");
			return -EINVAL;
		}
	} else {
		dev_err(chip->dev,"irq gpio not provided\n");
		return -EINVAL;
	}

	if (chip->irq) {
		ret = devm_request_threaded_irq(&chip->client->dev, chip->irq,
				NULL, nu2115_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				nu2115_irq_name[chip->mode], chip);

		if (ret < 0) {
			dev_err(chip->dev,"request irq for irq=%d failed, ret =%d\n",
							chip->irq, ret);
			return ret;
		}
		enable_irq_wake(chip->irq);
	}

	return ret;
}
/********************interrupte end*************************************************/
#ifdef CONFIG_MTK_CLASS
static inline int to_nu2115_adc(enum adc_channel chan)
{
	switch (chan) {
	case ADC_CHANNEL_VBUS:
		return ADC_VBUS;
	case ADC_CHANNEL_VBAT:
		return ADC_VBAT;
	case ADC_CHANNEL_IBUS:
		return ADC_IBUS;
	case ADC_CHANNEL_IBAT:
		return ADC_IBAT;
	case ADC_CHANNEL_TEMP_JC:
		return ADC_TDIE;
	case ADC_CHANNEL_VOUT:
		return ADC_VOUT;
	default:
		break;
	}
	return ADC_MAX_NUM;
}

static int mtk_nu2115_set_enable(struct charger_device *chg_dev, bool enable)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	return nu2115_enable_charge(chip, enable);
}

static int mtk_nu2115_get_is_enable(struct charger_device *chg_dev, bool *enable)
{
	struct nu2115 *chip = charger_get_data(chg_dev);
	return nu2115_check_charge_enabled(chip, enable);
}

static int mtk_nu2115_get_adc_value(struct charger_device *chg_dev, enum adc_channel ch,
                int *min, int *max)
{
	struct nu2115 *chip = charger_get_data(chg_dev);

	nu2115_get_adc_data(chip, to_nu2115_adc(ch), max);

	if(ch != ADC_CHANNEL_TEMP_JC)
		*max = *max * 1000;

	if (min != max)
		*min = *max;

	return 0;
}

static int mtk_nu2115_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
    *min = *max = nu2115_adc_accuracy_tbl[to_nu2115_adc(chan)];
    return 0;
}

static int mtk_nu2115_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
    struct nu2115 *chip = charger_get_data(chg_dev);

    return nu2115_is_vbuslowerr(chip, err);
}

static int mtk_nu2115_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
    struct nu2115 *chip = charger_get_data(chg_dev);
    int mv;
    mv = uV / 1000;

    return nu2115_set_busovp_th(chip, mv);
}

static int mtk_nu2115_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
    struct nu2115 *chip = charger_get_data(chg_dev);
    int ma;
    ma = uA / 1000;

    return nu2115_set_busocp_th(chip, ma);
}

static int mtk_nu2115_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
    struct nu2115 *chip = charger_get_data(chg_dev);
    int ret;

    ret = nu2115_set_batovp_th(chip, uV/1000);
    if (ret < 0)
        return ret;

    return ret;
}

static int mtk_nu2115_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
    struct nu2115 *chip = charger_get_data(chg_dev);
    int ret;

    ret = nu2115_set_batocp_th(chip, uA/1000);
    if (ret < 0)
        return ret;

    return ret;
}

static int mtk_nu2115_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    return 0;
}

static int mtk_nu2115_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
    return 0;
}

static int mtk_nu2115_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    return 0;
}

static int mtk_nu2115_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
    return 0;
}

static int mtk_nu2115_init_chip(struct charger_device *chg_dev)
{
    struct nu2115 *chip = charger_get_data(chg_dev);

    return nu2115_init_device(chip);
}

static int mtk_nu2115_enable_cp_adc(struct charger_device *chg_dev, bool enable)
{
    struct nu2115 *chip = charger_get_data(chg_dev);

    return nu2115_enable_adc(chip, enable);
}

static int mtk_nu2115_enable_otg(struct charger_device *chg_dev, bool en)
{
    struct nu2115 *chip = charger_get_data(chg_dev);
    int ret = 0;

    if (en) {
        ret = nu2115_enable_otg(chip, true);
        ret |= nu2115_enable_acdrv1(chip, true);
    } else {
        ret = nu2115_enable_otg(chip, false);
        ret |= nu2115_enable_acdrv1(chip, false);
    }
    pr_info("%s:(%d), ret=%d\n", en, ret);
    return ret;
}

static const struct charger_ops nu2115_ops = {
    .enable = mtk_nu2115_set_enable,
    .is_enabled = mtk_nu2115_get_is_enable,
    .get_adc = mtk_nu2115_get_adc_value,
    .get_adc_accuracy = mtk_nu2115_get_adc_accuracy,
    .set_vbusovp = mtk_nu2115_set_vbusovp,
    .set_ibusocp = mtk_nu2115_set_ibusocp,
    .set_vbatovp = mtk_nu2115_set_vbatovp,
    .set_ibatocp = mtk_nu2115_set_ibatocp,
    .init_chip = mtk_nu2115_init_chip,
    .is_vbuslowerr = mtk_nu2115_is_vbuslowerr,
    .set_vbatovp_alarm = mtk_nu2115_set_vbatovp_alarm,
    .reset_vbatovp_alarm = mtk_nu2115_reset_vbatovp_alarm,
    .set_vbusovp_alarm = mtk_nu2115_set_vbusovp_alarm,
    .reset_vbusovp_alarm = mtk_nu2115_reset_vbusovp_alarm,
    .enable_cp_adc = mtk_nu2115_enable_cp_adc,
    .enable_otg = mtk_nu2115_enable_otg,
};
#endif /*CONFIG_MTK_CLASS*/

/************************psy start**************************************/
static enum power_supply_property nu2115_charger_props[] = {
		POWER_SUPPLY_PROP_ONLINE,
		POWER_SUPPLY_PROP_VOLTAGE_NOW,
		POWER_SUPPLY_PROP_CURRENT_NOW,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
		POWER_SUPPLY_PROP_TEMP,
};

static int nu2115_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct nu2115 *chip = power_supply_get_drvdata(psy);
	int result;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		nu2115_check_charge_enabled(chip, &chip->charge_enabled);
		val->intval = chip->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nu2115_get_adc_data(chip, ADC_VBUS, &result);
		if (!ret)
			chip->vbus_volt = result;
		val->intval = chip->vbus_volt;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = nu2115_get_adc_data(chip, ADC_IBUS, &result);
		if (!ret)
			chip->ibus_curr = result;
		val->intval = chip->ibus_curr;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = nu2115_get_adc_data(chip, ADC_VBAT, &result);
		if (!ret)
			chip->vbat_volt = result;
		val->intval = chip->vbat_volt;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = nu2115_get_adc_data(chip, ADC_IBAT, &result);
		if (!ret)
			chip->ibat_curr = result;
		val->intval = chip->ibat_curr;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = nu2115_get_adc_data(chip, ADC_TDIE, &result);
		if (!ret)
			chip->die_temp = result;
		val->intval = chip->die_temp;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nu2115_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
	struct nu2115 *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		nu2115_enable_charge(chip, val->intval);
		dev_info(chip->dev, "POWER_SUPPLY_PROP_ONLINE: %s\n",
				val->intval ? "enable" : "disable");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nu2115_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
	return 0;
}

static int nu2115_psy_register(struct nu2115 *chip)
{
	chip->psy_cfg.drv_data = chip;
	chip->psy_cfg.of_node = chip->dev->of_node;

	chip->psy_desc.name = nu2115_psy_name[chip->mode];

	chip->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	chip->psy_desc.properties = nu2115_charger_props;
	chip->psy_desc.num_properties = ARRAY_SIZE(nu2115_charger_props);
	chip->psy_desc.get_property = nu2115_charger_get_property;
	chip->psy_desc.set_property = nu2115_charger_set_property;
	chip->psy_desc.property_is_writeable = nu2115_charger_is_writeable;


	chip->psy = devm_power_supply_register(chip->dev,
			&chip->psy_desc, &chip->psy_cfg);
	if (IS_ERR(chip->psy)) {
		dev_err(chip->dev, "%s failed to register psy\n", __func__);
		return PTR_ERR(chip->psy);
	}

	dev_info(chip->dev, "%s power supply register successfully\n", chip->psy_desc.name);

	return 0;
}
/************************psy end**************************************/

static int nu2115_set_work_mode(struct nu2115 *chip, int mode)
{
	chip->mode = mode;

	dev_err(chip->dev,"work mode is %s\n", chip->mode == NU2115_STANDALONG
		? "standalone" : (chip->mode == NU2115_MASTER ? "master" : "slave"));

	return 0;
}


__maybe_unused static int nu2115_parse_dt(struct nu2115 *chip, struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node;

	chip->cfg = devm_kzalloc(dev, sizeof(struct nu2115_cfg),
			GFP_KERNEL);

	if (!chip->cfg)
		return -ENOMEM;

	ret = of_property_read_u32(np, "nuvolta,nu2115,bat-ovp-disable",
			&chip->cfg->bat_ovp_disable);
	ret = of_property_read_u32(np, "nuvolta,nu2115,bat-ocp-disable",
			&chip->cfg->bat_ocp_disable);
	ret = of_property_read_u32(np, "nuvolta,nu2115,bus-ocp-disable",
			&chip->cfg->bus_ocp_disable);

	ret = of_property_read_u32(np, "nuvolta,nu2115,bat-ovp-threshold",
			&chip->cfg->bat_ovp_th);
	if (ret) {
		nu_err("failed to read bat-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "nuvolta,nu2115,bat-ocp-threshold",
			&chip->cfg->bat_ocp_th);
	if (ret) {
		nu_err("failed to read bat-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "nuvolta,nu2115,bus-ovp-threshold",
			&chip->cfg->bus_ovp_th);
	if (ret) {
		nu_err("failed to read bus-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "nuvolta,nu2115,bus-ovp-alarm-threshold",
			&chip->cfg->bus_ovp_alm_th);
	if (ret) {
		nu_err("failed to read bus-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "nuvolta,nu2115,bus-ocp-threshold",
			&chip->cfg->bus_ocp_th);
	if (ret) {
		nu_err("failed to read bus-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "nuvolta,nu2115,bus-ocp-alarm-threshold",
			&chip->cfg->bus_ocp_alm_th);
	if (ret) {
		nu_err("failed to read bus-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "nuvolta,nu2115,ac-ovp-threshold",
			&chip->cfg->ac_ovp_th);
	if (ret) {
		nu_err("failed to read ac-ovp-threshold\n");
		return ret;
	}

	chip->irq_gpio = of_get_named_gpio(np, "nu2115,intr_gpio", 0);
	if (!gpio_is_valid(chip->irq_gpio)) {
		dev_err(chip->dev,"fail to valid gpio : %d\n", chip->irq_gpio);
		return -EINVAL;
	}

	if (of_property_read_string(np, "charger_name", &chip->chg_dev_name) < 0) {
		chip->chg_dev_name = "NU2115_CP";
	}

	return 0;
}

static struct of_device_id nu2115_charger_match_table[] = {
	{
		.compatible = "nuvolta,nu2115-standalone",
		.data = &nu2115_mode_data[NU2115_STANDALONG],
	},
	{
		.compatible = "nuvolta,nu2115-master",
		.data = &nu2115_mode_data[NU2115_MASTER],
	},

	{
		.compatible = "nuvolta,nu2115-slave",
		.data = &nu2115_mode_data[NU2115_SLAVE],
	},
	{},
};
MODULE_DEVICE_TABLE(of, nu2115_charger_match_table);

static int nu2115_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct nu2115 *chip;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	int ret;

	dev_err(&client->dev, "%s\n", __func__);

	chip =  devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}
	chip->dev = &client->dev;
	chip->client = client;

	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->data_lock);

	ret = nu2115_detect_device(chip);
	if (ret) {
		nu_err("No nu2115 device found!\n");
		ret = -ENODEV;
		goto err_detect_dev;
	}

	i2c_set_clientdata(client, chip);
	nu2115_create_device_node(&(client->dev));

	dev_err(chip->dev, "%s\n nu2115 ", __func__);
	match = of_match_node(nu2115_charger_match_table, node);
	if (match == NULL) {
		nu_err("device tree match not found!\n");
		goto err_match_node;
	}

	ret = nu2115_set_work_mode(chip, *(int *)match->data);
	if (ret) {
		dev_err(chip->dev,"Fail to set work mode!\n");
		goto err_set_mode;
	}

	ret = nu2115_parse_dt(chip, &client->dev);
	if (ret < 0) {
		dev_err(chip->dev, "%s parse dt failed(%d)\n", __func__, ret);
		goto err_parse_dt;
	}

	ret = nu2115_init_device(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s init device failed(%d)\n", __func__, ret);
		goto err_init_device;
	}

	ret = nu2115_enable_adc(chip, false);
	if (ret) {
		dev_err(chip->dev,"Fail to disable adc!\n");
		goto err_set_mode;
	}

	ret = nu2115_psy_register(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s psy register failed(%d)\n", __func__, ret);
		goto err_register_psy;
	}

	ret = nu2115_register_interrupt(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s register irq fail(%d)\n",
					__func__, ret);
		goto err_register_irq;
	}

#ifdef CONFIG_MTK_CLASS
    chip->chg_dev = charger_device_register(chip->chg_dev_name,
					      &client->dev, chip,
					      &nu2115_ops,
					      &nu2115_chg_props);
	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		ret = PTR_ERR(chip->chg_dev);
		dev_err(chip->dev,"Fail to register charger!\n");
        goto err_register_mtk_charger;
	}
#endif /*CONFIG_MTK_CLASS*/

	nu_err("nu2115 probe successfully, Part Num:%d\n!", chip->part_no);

	return 0;

#ifdef CONFIG_MTK_CLASS
err_register_mtk_charger:
#endif /*CONFIG_MTK_CLASS*/
err_register_psy:
err_register_irq:
err_init_device:
	power_supply_unregister(chip->psy);
err_detect_dev:
err_match_node:
err_set_mode:
err_parse_dt:
	devm_kfree(&client->dev, chip);
err_kzalloc:
	dev_err(&client->dev,"nu2115 probe fail\n");
	return ret;
}


static int nu2115_charger_remove(struct i2c_client *client)
{
	struct nu2115 *chip = i2c_get_clientdata(client);


	nu2115_enable_adc(chip, false);
	mutex_destroy(&chip->data_lock);
	mutex_destroy(&chip->i2c_rw_lock);

	power_supply_unregister(chip->psy);
	devm_kfree(&client->dev, chip);

	return 0;
}

static void nu2115_charger_shutdown(struct i2c_client *client)
{
	struct nu2115 *chip = i2c_get_clientdata(client);

	nu2115_enable_adc(chip, false);
}

#ifdef CONFIG_PM_SLEEP
static int nu2115_suspend(struct device *dev)
{
	struct nu2115 *chip = dev_get_drvdata(dev);

	dev_info(chip->dev, "Suspend successfully!");
	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);
	disable_irq(chip->irq);

	return 0;
}

static int nu2115_resume(struct device *dev)
{
	struct nu2115 *chip = dev_get_drvdata(dev);

	dev_info(chip->dev, "Resume successfully!");
	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);
	enable_irq(chip->irq);

	return 0;
}
static const struct dev_pm_ops nu2115_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nu2115_suspend, nu2115_resume)
};
#endif

static const struct i2c_device_id nu2115_charger_id[] = {
	{"nu2115-standalone", NU2115_STANDALONG},
	{},
};
MODULE_DEVICE_TABLE(i2c, nu2115_charger_id);

static struct i2c_driver nu2115_charger_driver = {
	.driver		= {
		.name	= "nu2115-charger",
		.owner	= THIS_MODULE,
		.of_match_table = nu2115_charger_match_table,
#ifdef CONFIG_PM_SLEEP
		.pm	= &nu2115_pm_ops,
#endif
	},
	.id_table	= nu2115_charger_id,
	.probe		= nu2115_charger_probe,
	.remove		= nu2115_charger_remove,
	.shutdown	= nu2115_charger_shutdown,
};

module_i2c_driver(nu2115_charger_driver);

MODULE_DESCRIPTION("Nuvolta NU2115 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("mick.ye@nuvoltatech.com");
