/* SPDX-License-Identifier: GPL-2.0 */
/*
 * hl7603.h
 *
 * boost ic driver
 *
 * Copyright (c) 2023-2023 Xiaomi Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#ifndef __HL7603_H
#define __HL7603_H

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
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/version.h>

#define CONFIG1     0x01

#define VOUT_REG    0x02
#define VOUT_REG_BASE   2850
#define VOUT_REG_MAX    5500
#define VOUT_REG_STEP   50

#define ILIMSET1    0x03

#define STATUS      0x05

struct boost_bypass_dev {
    struct device *dev;
    struct i2c_client *client;
    u32 vout_threshold;
};

#endif  /* __HL7603_H */