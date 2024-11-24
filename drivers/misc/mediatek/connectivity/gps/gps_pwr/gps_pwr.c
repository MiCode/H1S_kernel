/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */


/*******************************************************************************
* Dependency
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <conn_power_throttling.h>
#include <linux/poll.h>
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

/******************************************************************************
 * Definition
******************************************************************************/
/* device name and major number */
#define GPS_PWR_DEVNAME            "gps_pwr"
/*******************************************************************************
* structure & enumeration
*******************************************************************************/
/*---------------------------------------------------------------------------*/
struct gps_pwr_dev {
	struct class *cls;
	struct device *dev;
	dev_t devno;
	struct cdev chdev;
};

/******************************************************************************
 * local variables
******************************************************************************/
/* static int flag; */
bool has_new_battery_level;
struct gps_pwr_dev *gps_pwr_devobj;
static wait_queue_head_t GPS_PWR_wq;
#define DRV_TYPE_GPS    2
enum conn_pwr_low_battery_level battery_level;
bool gps_pwr_open_state;

int gps_pwr_msg_cb(enum conn_pwr_event_type event_type, void *level)
{
	int ret = 0;

	if (event_type == CONN_PWR_EVENT_LEVEL) {
		pr_info("gps_pwr_msg_cb been called, level=%d, gps_pwr_open_state=%d\n",
			*(enum conn_pwr_low_battery_level *)level, gps_pwr_open_state);
		battery_level = *(enum conn_pwr_low_battery_level *)level;
		has_new_battery_level = true;
		wake_up_interruptible(&GPS_PWR_wq);
	}
	return ret;
}

/*---------------------------------------------------------------------------*/
long gps_pwr_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long retval = 0;

	return retval;
}

long gps_pwr_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gps_pwr_unlocked_ioctl(filp, cmd, arg);
}

/******************************************************************************/
/*****************************************************************************/
static int gps_pwr_open(struct inode *inode, struct file *file)
{
	int ret;

	gps_pwr_open_state = true;
	ret = conn_pwr_drv_pre_on(DRV_TYPE_GPS, &battery_level);
	has_new_battery_level = true;
	pr_info("conn_pwr_drv_pre_on ret = %d, battery_level = %d\n", ret, battery_level);
	return 0;
}

/*****************************************************************************/


/*****************************************************************************/
static int gps_pwr_close(struct inode *inode, struct file *file)
{
	int ret;

	gps_pwr_open_state = false;
	ret = conn_pwr_drv_post_off(DRV_TYPE_GPS);
	pr_info("conn_pwr_drv_post_off ret = %d\n", ret);
	return 0;
}

static ssize_t gps_pwr_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int retval;

	pr_info("gps_pwr read\n");

	if (count > sizeof(int))
		count = sizeof(int);

	if (copy_to_user(buf, (void *)&battery_level, sizeof(int))) {
		pr_info("gps_pwr_read failed,because copy_to_user error\n");
		retval = -EFAULT;
	} else {
		retval = sizeof(int);
		has_new_battery_level = false;
		return retval;
	}

	if (retval  ==  -EFAULT)
		goto OUT;

OUT:
	/*	pr_info("gps_pwr_read(): retval = %d\n", retval);*/
		return retval;

}
/******************************************************************************/
static ssize_t gps_pwr_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t retval = 0;

	return retval;
}
/*****************************************************************************/
static unsigned int gps_pwr_poll(struct file *file, poll_table *wait)
{
	unsigned int mask;

	mask = 0;

	poll_wait(file, &GPS_PWR_wq, wait);
	if (has_new_battery_level == true)
		mask = (POLLIN | POLLRDNORM);
	pr_info("gps_pwr_poll been return , mask = %d\n", mask);

	return mask;
}
/* Kernel interface */
static const struct file_operations gps_pwr_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gps_pwr_unlocked_ioctl,
	.compat_ioctl = gps_pwr_compat_ioctl,
	.open = gps_pwr_open,
	.read = gps_pwr_read,
	.write = gps_pwr_write,
	.release = gps_pwr_close,
	.poll = gps_pwr_poll,
};

static int gps_pwr_init(void)
{
	int ret = 0;
	int err = 0;

	gps_pwr_devobj = kzalloc(sizeof(*gps_pwr_devobj), GFP_KERNEL);
	if (gps_pwr_devobj == NULL) {
		err = -ENOMEM;
		ret = -ENOMEM;
		goto err_out;
	}

	pr_info("Registering gps_pwr chardev\n");
	ret = alloc_chrdev_region(&gps_pwr_devobj->devno, 0, 1, GPS_PWR_DEVNAME);
	if (ret) {
		pr_info("alloc_chrdev_region fail: %d\n", ret);
		err = -ENOMEM;
		goto err_out;
	} else {
		pr_info("major: %d, minor: %d\n", MAJOR(gps_pwr_devobj->devno), MINOR(gps_pwr_devobj->devno));
	}
	cdev_init(&gps_pwr_devobj->chdev, &gps_pwr_fops);
	gps_pwr_devobj->chdev.owner = THIS_MODULE;
	err = cdev_add(&gps_pwr_devobj->chdev, gps_pwr_devobj->devno, 1);
	if (err) {
		pr_info("cdev_add fail: %d\n", err);
		goto err_out;
	}
	gps_pwr_devobj->cls = class_create(THIS_MODULE, "gps_pwr");
	if (IS_ERR(gps_pwr_devobj->cls)) {
		pr_info("Unable to create class, err = %d\n", (int)PTR_ERR(gps_pwr_devobj->cls));
	goto err_out;
	}

	init_waitqueue_head(&GPS_PWR_wq);
	has_new_battery_level = false;
	gps_pwr_open_state = false;
	conn_pwr_register_event_cb(DRV_TYPE_GPS, (CONN_PWR_EVENT_CB)gps_pwr_msg_cb);

	gps_pwr_devobj->dev = device_create(gps_pwr_devobj->cls,
		NULL, gps_pwr_devobj->devno, gps_pwr_devobj, "gps_pwr");
	pr_info("GPS_PWR device init Done\n");
	return 0;

err_out:
	if (gps_pwr_devobj != NULL) {
		if (err == 0)
			cdev_del(&gps_pwr_devobj->chdev);
		if (ret == 0)
			unregister_chrdev_region(gps_pwr_devobj->devno, 1);
		kfree(gps_pwr_devobj);
		gps_pwr_devobj = NULL;
	}
	return -1;
}

/*****************************************************************************/
static void gps_pwr_exit(void)
{
	if (!gps_pwr_devobj) {
		pr_info("null pointer: %p\n", gps_pwr_devobj);
		return;
	}

	pr_info("Unregistering gps_pwr chardev\n");
	cdev_del(&gps_pwr_devobj->chdev);
	unregister_chrdev_region(gps_pwr_devobj->devno, 1);
	device_destroy(gps_pwr_devobj->cls, gps_pwr_devobj->devno);
	class_destroy(gps_pwr_devobj->cls);
	kfree(gps_pwr_devobj);
	gps_pwr_devobj = NULL;
	pr_info("Done\n");
}

int mtk_gps_pwr_init(void)
{
	pr_info("gps_pwr init begin");
	return gps_pwr_init();
}

void mtk_gps_pwr_exit(void)
{
	pr_info("gps_pwr exit begin");
	return gps_pwr_exit();
}
/*****************************************************************************/
module_init(mtk_gps_pwr_init);
module_exit(mtk_gps_pwr_exit);
/*****************************************************************************/
MODULE_AUTHOR("Tianfang li <Tianfang.Li@mediatek.com>");
MODULE_DESCRIPTION("GPS_PWR dev");
MODULE_LICENSE("GPL");


