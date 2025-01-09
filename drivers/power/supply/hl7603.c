// SPDX-License-Identifier: GPL-2.0
/*
 * hl7603.c
 *
 * boost bypass ic driver
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

#include <linux/printk.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include "hl7603.h"

static int hl7603_parse_dt(struct boost_bypass_dev *bq)
{
    struct device_node *np = bq->dev->of_node;

    if (!np) {
        return -1;
    }
    of_property_read_u32(np, "vout_threshold", &bq->vout_threshold);
    return 0;

}

static int hl7603_set_voltage_threshold(struct boost_bypass_dev *bq, u32 vout_threshold)
{
    u8 val = 0;
    int ret;

    if ((vout_threshold > VOUT_REG_MAX) || vout_threshold < VOUT_REG_BASE) {
        return -1;
    }
    val = (vout_threshold - VOUT_REG_BASE) / VOUT_REG_STEP;

    ret = i2c_smbus_write_byte_data(bq->client, VOUT_REG, val);
    if (ret < 0) {
        return ret;
    }
    val = i2c_smbus_read_byte_data(bq->client, VOUT_REG);

    return ret;
}

static int hl7603_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct boost_bypass_dev *bq;

    bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
    if (!bq)
        return -ENOMEM;

    bq->client = client;
    bq->dev = &client->dev;
    i2c_set_clientdata(client, bq);
    ret = hl7603_parse_dt(bq);
    ret |= hl7603_set_voltage_threshold(bq, bq->vout_threshold);
    if (ret) {
        return ret;
    }

    return 0;
}

static void hl7603_remove(struct i2c_client *client)
{

}

static void hl7603_shutdown(struct i2c_client *client)
{

}

static int hl7603_suspend(struct device *dev)
{
    return 0;
}

static int hl7603_resume(struct device *dev)
{
    return 0;
}

static const struct dev_pm_ops hl7603_pm_ops = {
    .suspend    = hl7603_suspend,
    .resume     = hl7603_resume,
};

static const struct of_device_id hl7603_of_match[] = {
    { .compatible = "hl7603"},
    {},
};

MODULE_DEVICE_TABLE(of, hl7603_of_match);

static struct i2c_driver hl7603_driver = {
    .driver = {
        .name = "hl7603_boost_bypass",
        .of_match_table = hl7603_of_match,
        .pm = &hl7603_pm_ops,
    },
    .probe = hl7603_probe,
    .remove = hl7603_remove,
    .shutdown = hl7603_shutdown,
};
module_i2c_driver(hl7603_driver);

MODULE_AUTHOR("jinkai <jinaki1@xiaomi.com>");
MODULE_DESCRIPTION("hl7603 boot_bypass driver");
MODULE_LICENSE("GPL v2");