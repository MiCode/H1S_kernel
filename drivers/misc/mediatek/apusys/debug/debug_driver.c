// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/platform_device.h>
#include <linux/module.h>

#include "debug_driver.h"

#define APUDBG_DEV_NAME		"apu_debug"

static int debug_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_notice("%s in", __func__);

	ret = apusys_dump_init(&pdev->dev);
	if (ret) {
		DBG_LOG_ERR("failed to create debug dump attr node(devinfo).\n");
		goto out;
	}

	return 0;

out:
	pr_notice("debug probe error!!\n");

	return ret;
}

static int debug_remove(struct platform_device *pdev)
{
	apusys_dump_exit(&pdev->dev);
	return 0;
}

static struct platform_driver debug_driver = {
	.probe = debug_probe,
	.remove = debug_remove,
	.driver = {
		   .name = APUDBG_DEV_NAME,
		   .owner = THIS_MODULE,
	}
};

static struct platform_device debug_device = {
	.name = APUDBG_DEV_NAME,
	.id = 0,
};

static int __init debug_INIT(void)
{
	int ret = 0;

	pr_notice("%s debug driver init", __func__);

	ret = platform_driver_register(&debug_driver);
	if (ret != 0)
		pr_notice("failed to register debug driver");

	if (platform_device_register(&debug_device)) {
		DBG_LOG_ERR("failed to register debug_driver device");
		platform_driver_unregister(&debug_driver);
		return -ENODEV;
	}

	return ret;
}

static void __exit debug_EXIT(void)
{
	platform_driver_unregister(&debug_driver);
}

module_init(debug_INIT);
module_exit(debug_EXIT);
MODULE_LICENSE("GPL");
