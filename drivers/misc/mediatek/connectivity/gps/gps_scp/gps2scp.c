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
#include <conap_scp.h>
#include <linux/poll.h>
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

/******************************************************************************
 * Definition
******************************************************************************/
/* device name and major number */
#define GPS2SCP_DEVNAME            "gps2scp"
#ifdef GPS2SCP_TEST_ENABLE
#define GPS2SCP_DEVNAME_TEST       "gps2scp_test"
#endif
/*******************************************************************************
* structure & enumeration
*******************************************************************************/
/*---------------------------------------------------------------------------*/
struct gps2scp_dev {
	struct class *cls;
	struct device *dev;
	dev_t devno;
	struct cdev chdev;
};
/* typedef unsigned char   UINT8, *PUINT8, **PPUINT8; */

/******************************************************************************
 * local variables
******************************************************************************/
/* static int flag; */
static struct conap_scp_drv_cb gps2scp_cb;
#define GPS2SCP_BUFFER_SIZE (1024*10)
struct gps2scp_dev *gps2scp_devobj;
static unsigned char o_buf[GPS2SCP_BUFFER_SIZE];	/* output buffer of write() */
static struct semaphore wr_mtx, rd_mtx, ring_buf_mtx;
static wait_queue_head_t GPS2SCP_wq;
#define SCP2GPS_BUFF_SIZE       1024
#define SCP2GPS_BUFF_NUM  24
#define DRV_TYPE_GPS    1
struct scp2gps_data {
	int size;
	unsigned int msg_id;
	unsigned int data[SCP2GPS_BUFF_SIZE];
};
struct scp2gps_data_link {
	int msg_read;
	struct scp2gps_data data_link[SCP2GPS_BUFF_NUM];
	int msg_write;
};
struct scp2gps_data_link scp2gps_data_received;
#ifdef GPS2SCP_TEST_ENABLE
static unsigned char o_buf_test[GPS2SCP_BUFFER_SIZE];	/* output buffer of write() */
static struct semaphore wr_mtx_test, rd_mtx_test, ring_buf_mtx_test;
static wait_queue_head_t GPS2SCP_wq_test;
struct gps2scp_dev *gps2scp_devobj_test;
struct scp2gps_data_link scp2gps_data_received_test;

void save_scp2gps_data_test(unsigned int msg_id, unsigned int *buf, unsigned int size)
{
	down(&ring_buf_mtx_test);
	pr_info("save_scp2gps_data_test msg_write=%d , size=%d, msg_read =%d\n",
		scp2gps_data_received_test.msg_write, size, scp2gps_data_received_test.msg_read);
	memset(&scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_write], 0,
		sizeof(scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_write]));
	if (size < SCP2GPS_BUFF_SIZE)
		memcpy(&scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_write].data[0], buf, size);
	else
		memcpy(&scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_write].data[0],
			buf, (SCP2GPS_BUFF_SIZE - 1));
	pr_info("save_scp2gps_data_test msg_write=%d , size=%d, msg_read =%d\n",
		scp2gps_data_received_test.msg_write,
		scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_write].size,
		scp2gps_data_received_test.msg_read);
	scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_write].msg_id = msg_id;
	scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_write].size = size;

	scp2gps_data_received_test.msg_write++;
	if (scp2gps_data_received_test.msg_write >= SCP2GPS_BUFF_NUM)
		scp2gps_data_received_test.msg_write = 0;

	if (scp2gps_data_received_test.msg_write == scp2gps_data_received_test.msg_read) {
		scp2gps_data_received_test.msg_read++;
		if (scp2gps_data_received_test.msg_read >= SCP2GPS_BUFF_NUM)
			scp2gps_data_received_test.msg_read = 0;
	}
	up(&ring_buf_mtx_test);
}

ssize_t read_scp2gps_data_test(char __user *buf,  size_t count)
{
	int retval;
	int msg_read;
	int size;

	down(&ring_buf_mtx_test);
	pr_info("read_scp2gps_data_test msg_write=%d , size=%d, msg_read =%d\n",
		scp2gps_data_received_test.msg_write,
		scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_read].size,
		scp2gps_data_received_test.msg_read);
	msg_read = scp2gps_data_received_test.msg_read;
	size = scp2gps_data_received_test.data_link[msg_read].size;
	if (size > count)
		size = count;
	if (msg_read != scp2gps_data_received_test.msg_write) {
		/* we got something from STP driver */
		if (copy_to_user(buf, scp2gps_data_received_test.data_link[msg_read].data, size)) {
			pr_info("read_scp2gps_data failed,because copy_to_user error\n");
			retval = -EFAULT;
			up(&ring_buf_mtx_test);
		} else {
			retval = scp2gps_data_received_test.data_link[msg_read].size;
			scp2gps_data_received_test.msg_read++;
			if (scp2gps_data_received_test.msg_read >= SCP2GPS_BUFF_NUM)
				scp2gps_data_received_test.msg_read = 0;
			up(&ring_buf_mtx_test);
			pr_info("read_scp2gps_data_test msg_write=%d , size=%d, msg_read =%d\n",
				scp2gps_data_received_test.msg_write,
				scp2gps_data_received_test.data_link[scp2gps_data_received_test.msg_write].size,
				scp2gps_data_received_test.msg_read);
		}
		return retval;
	}
	up(&ring_buf_mtx_test);
	return 0;
}

#endif

void save_scp2gps_data(unsigned int msg_id, unsigned int *buf, unsigned int size)
{
	down(&ring_buf_mtx);
	pr_info("save_scp2gps_data msg_write=%d , size=%d, msg_read =%d\n",
		scp2gps_data_received.msg_write, size, scp2gps_data_received.msg_read);
	memset(&scp2gps_data_received.data_link[scp2gps_data_received.msg_write], 0,
		sizeof(scp2gps_data_received.data_link[scp2gps_data_received.msg_write]));
	if (size < SCP2GPS_BUFF_SIZE)
		memcpy(&scp2gps_data_received.data_link[scp2gps_data_received.msg_write].data[0], buf, size);
	else
		memcpy(&scp2gps_data_received.data_link[scp2gps_data_received.msg_write].data[0],
			buf, (SCP2GPS_BUFF_SIZE - 1));
	pr_info("save_scp2gps_data msg_write=%d , size=%d, msg_read =%d\n",
		scp2gps_data_received.msg_write,
		scp2gps_data_received.data_link[scp2gps_data_received.msg_write].size,
		scp2gps_data_received.msg_read);
	scp2gps_data_received.data_link[scp2gps_data_received.msg_write].msg_id = msg_id;
	scp2gps_data_received.data_link[scp2gps_data_received.msg_write].size = size;

	scp2gps_data_received.msg_write++;
	if (scp2gps_data_received.msg_write >= SCP2GPS_BUFF_NUM)
		scp2gps_data_received.msg_write = 0;

	if (scp2gps_data_received.msg_write == scp2gps_data_received.msg_read) {
		scp2gps_data_received.msg_read++;
		if (scp2gps_data_received.msg_read >= SCP2GPS_BUFF_NUM)
			scp2gps_data_received.msg_read = 0;
	}
	up(&ring_buf_mtx);
}

ssize_t read_scp2gps_data(char __user *buf,  size_t count)
{
	int retval;
	int msg_read;
	int size;

	down(&ring_buf_mtx);
	pr_info("read_scp2gps_data msg_write=%d , size=%d, msg_read =%d\n",
		scp2gps_data_received.msg_write,
		scp2gps_data_received.data_link[scp2gps_data_received.msg_read].size,
		scp2gps_data_received.msg_read);
	msg_read = scp2gps_data_received.msg_read;
	size = scp2gps_data_received.data_link[msg_read].size;
	if (size > count)
		size = count;
	if (msg_read != scp2gps_data_received.msg_write) {
		/* we got something from STP driver */
		if (copy_to_user(buf, scp2gps_data_received.data_link[msg_read].data, size)) {
			pr_info("read_scp2gps_data failed,because copy_to_user error\n");
			retval = -EFAULT;
			up(&ring_buf_mtx);
		} else {
			retval = scp2gps_data_received.data_link[msg_read].size;
			scp2gps_data_received.msg_read++;
			if (scp2gps_data_received.msg_read >= SCP2GPS_BUFF_NUM)
				scp2gps_data_received.msg_read = 0;
			up(&ring_buf_mtx);
			pr_info("read_scp2gps_data msg_write=%d , size=%d, msg_read =%d\n",
				scp2gps_data_received.msg_write,
				scp2gps_data_received.data_link[scp2gps_data_received.msg_write].size,
				scp2gps_data_received.msg_read);
		}
		return retval;
	}
	up(&ring_buf_mtx);
	return 0;
}
void gps2scp_msg_cb(unsigned int msg_id, unsigned int *buf, unsigned int size)
{
	pr_info("gps2scp_msg_cb been called, msg_id=%d,*buf=0x%p,size=%d\n", msg_id, buf, size);
	if (msg_id == 0) {
		save_scp2gps_data(msg_id, buf, size);
		wake_up_interruptible(&GPS2SCP_wq);
#ifdef GPS2SCP_TEST_ENABLE
	} else if (msg_id == 1) {
		save_scp2gps_data_test(msg_id, buf, size);
		wake_up_interruptible(&GPS2SCP_wq_test);
#endif
	}

}

void gps2scp_state_notify_cb(int state)
{
	pr_info("gps2scp_msg_cb been called , state = %d\n", state);
}

/*---------------------------------------------------------------------------*/
long gps2scp_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long retval = 0;

	return retval;
}

long gps2scp_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gps2scp_unlocked_ioctl(filp, cmd, arg);
}

/******************************************************************************/
/*****************************************************************************/
static int gps2scp_open(struct inode *inode, struct file *file)
{
	int conap_status;

	conap_status = conap_scp_is_drv_ready(DRV_TYPE_GPS);
	if (conap_status != 0) {
		gps2scp_cb.conap_scp_msg_notify_cb = gps2scp_msg_cb;
		gps2scp_cb.conap_scp_state_notify_cb = gps2scp_state_notify_cb;
		conap_scp_register_drv(DRV_TYPE_GPS, &gps2scp_cb);
	}
	conap_status = conap_scp_is_drv_ready(DRV_TYPE_GPS);
	if (conap_status == 0)
		pr_info("conap ready! conap_status = %d\n", conap_status);
	else
		pr_info("conap not ready! conap_status = %d\n", conap_status);

	pr_info("%s: gps2scp open major %d minor %d (pid %d)\n", __func__, imajor(inode), iminor(inode), current->pid);
	return 0;
}

/*****************************************************************************/


/*****************************************************************************/
static int gps2scp_close(struct inode *inode, struct file *file)
{
	int conap_status;

	conap_status = conap_scp_is_drv_ready(DRV_TYPE_GPS);
	if (conap_status == 0)
		conap_scp_unregister_drv(DRV_TYPE_GPS);

	conap_status = conap_scp_is_drv_ready(DRV_TYPE_GPS);
	if (conap_status == 0)
		pr_info("conap unregisetr fail! conap_status = %d\n", conap_status);
	else
		pr_info("conap unregisetr success! conap_status = %d\n", conap_status);

	memset(&scp2gps_data_received, 0, sizeof(scp2gps_data_received));

	pr_info("%s: gps2scp close major %d minor %d (pid %d)\n", __func__, imajor(inode), iminor(inode), current->pid);
	return 0;
}
#ifdef GPS2SCP_TEST_ENABLE
/*****************************************************************************/
static int gps2scp_open_test(struct inode *inode, struct file *file)
{
	int conap_status;

	conap_status = conap_scp_is_drv_ready(DRV_TYPE_GPS);
	if (conap_status != 0) {
		gps2scp_cb.conap_scp_msg_notify_cb = gps2scp_msg_cb;
		gps2scp_cb.conap_scp_state_notify_cb = gps2scp_state_notify_cb;
		conap_scp_register_drv(DRV_TYPE_GPS, &gps2scp_cb);
	}
	conap_status = conap_scp_is_drv_ready(DRV_TYPE_GPS);
	if (conap_status == 0)
		pr_info("conap ready! conap_status = %d\n", conap_status);
	else
		pr_info("conap not ready! conap_status = %d\n", conap_status);

	pr_info("%s: gps2scp open test major %d minor %d (pid %d)\n",
		__func__, imajor(inode), iminor(inode), current->pid);
	return 0;
}

/*****************************************************************************/


/*****************************************************************************/
static int gps2scp_close_test(struct inode *inode, struct file *file)
{
	int conap_status;

	conap_status = conap_scp_is_drv_ready(DRV_TYPE_GPS);
	if (conap_status == 0)
		conap_scp_unregister_drv(DRV_TYPE_GPS);

	conap_status = conap_scp_is_drv_ready(DRV_TYPE_GPS);
	if (conap_status == 0)
		pr_info("conap unregisetr fail! conap_status = %d\n", conap_status);
	else
		pr_info("conap unregisetr success! conap_status = %d\n", conap_status);


	memset(&scp2gps_data_received_test, 0, sizeof(scp2gps_data_received));

	pr_info("%s: gps2scp close test major %d minor %d (pid %d)\n",
		__func__, imajor(inode), iminor(inode), current->pid);
	return 0;
}

/******************************************************************************/
static ssize_t gps2scp_read_test(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int retval;

	pr_info("gps2scp read test\n");
	down(&rd_mtx_test);

	if (count > SCP2GPS_BUFF_SIZE)
		count = SCP2GPS_BUFF_SIZE;

	retval = read_scp2gps_data_test(buf, count);

	if (retval  ==  -EFAULT)
		goto OUT;

OUT:
		up(&rd_mtx_test);
	/*	pr_info("GPS_read(): retval = %d\n", retval);*/
		return retval;

}
#endif
static ssize_t gps2scp_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int retval;

	pr_info("gps2scp read\n");
	down(&rd_mtx);

	if (count > SCP2GPS_BUFF_SIZE)
		count = SCP2GPS_BUFF_SIZE;

	retval = read_scp2gps_data(buf, count);

	if (retval  ==  -EFAULT)
		goto OUT;

OUT:
		up(&rd_mtx);
	/*	pr_info("GPS_read(): retval = %d\n", retval);*/
		return retval;

}
/******************************************************************************/
#ifdef GPS2SCP_TEST_ENABLE
static ssize_t gps2scp_write_test(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int retval;
	int written = 0;
	int msg_id = 1;
	int copy_size = 0;

	down(&wr_mtx_test);
	pr_info("gps2scp write test\n");
	if (count > 0) {
		copy_size = (count < SCP2GPS_BUFF_SIZE) ? count : SCP2GPS_BUFF_SIZE;
		if (copy_from_user(&o_buf_test[0], &buf[0], copy_size)) {
			retval = -EFAULT;
			pr_info("gps2scp_write copy_from_user failed retval=%d\n", retval);
			goto out;
		}
	} else {
		retval = -EFAULT;
		pr_info("gps2scp_write target packet length:%zd is not allowed, retval = %d.\n", count, retval);
	}

	written = conap_scp_send_message(DRV_TYPE_GPS, msg_id, &o_buf_test[0], count);

	if (written == 0) {
		retval = copy_size;
		pr_info("conap_scp_send_message success count= %zd\n", count);
	} else {
		retval = -EFAULT;
		pr_info("conap_scp_send_message failed retval=%d , written=%d\n", retval, written);
	}

out:
	up(&wr_mtx_test);
	return retval;
}
#endif
static ssize_t gps2scp_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int retval;
	int written = 0;
	int msg_id = 0;
	int copy_size = 0;

	down(&wr_mtx);
	pr_info("gps2scp write\n");
	if (count > 0) {
		copy_size = (count < SCP2GPS_BUFF_SIZE) ? count : SCP2GPS_BUFF_SIZE;
		if (copy_from_user(&o_buf[0], &buf[0], copy_size)) {
			retval = -EFAULT;
			pr_info("gps2scp_write copy_from_user failed retval=%d\n", retval);
			goto out;
		}
	} else {
		retval = -EFAULT;
		pr_info("gps2scp_write target packet length:%zd is not allowed, retval = %d.\n", count, retval);
	}

	written = conap_scp_send_message(DRV_TYPE_GPS, msg_id, &o_buf[0], count);

	if (written == 0) {
		retval = copy_size;
		pr_info("conap_scp_send_message success count= %zd\n", count);
	} else {
		retval = -EFAULT;
		pr_info("conap_scp_send_message failed retval=%d , written=%d\n", retval, written);
	}

out:
	up(&wr_mtx);
	return retval;
}
/*****************************************************************************/
static unsigned int gps2scp_poll(struct file *file, poll_table *wait)
{
	unsigned int mask;

	mask = 0;
	pr_info("gps2scp_poll has enter\n");
	if (scp2gps_data_received.msg_read != scp2gps_data_received.msg_write) {
		mask = (POLLIN | POLLRDNORM);
		pr_info("gps2scp_poll already have data\n");
		return mask;
	}

	poll_wait(file, &GPS2SCP_wq, wait);

	if (scp2gps_data_received.msg_read != scp2gps_data_received.msg_write)
		mask = (POLLIN | POLLRDNORM);
	pr_info("gps2scp_poll been return , mask = %d\n", mask);

	return mask;
}
#ifdef GPS2SCP_TEST_ENABLE
static unsigned int gps2scp_test_poll(struct file *file, poll_table *wait)
{
	unsigned int mask;

	mask = 0;
	pr_info("gps2scp_test_poll has enter\n");
	if (scp2gps_data_received_test.msg_read != scp2gps_data_received_test.msg_write) {
		mask = (POLLIN | POLLRDNORM);
		pr_info("gps2scp_test_poll already have data\n");
		return mask;
	}

	poll_wait(file, &GPS2SCP_wq_test, wait);

	if (scp2gps_data_received_test.msg_read != scp2gps_data_received_test.msg_write)
		mask = (POLLIN | POLLRDNORM);

	pr_info("gps2scp_test_poll been return , mask = %d\n", mask);

	return mask;
}
#endif
/* Kernel interface */
static const struct file_operations gps2scp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gps2scp_unlocked_ioctl,
	.compat_ioctl = gps2scp_compat_ioctl,
	.open = gps2scp_open,
	.read = gps2scp_read,
	.write = gps2scp_write,
	.release = gps2scp_close,
	.poll = gps2scp_poll,
};
#ifdef GPS2SCP_TEST_ENABLE
static const struct file_operations gps2scp_fops_test = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gps2scp_unlocked_ioctl,
	.compat_ioctl = gps2scp_compat_ioctl,
	.open = gps2scp_open_test,
	.read = gps2scp_read_test,
	.write = gps2scp_write_test,
	.release = gps2scp_close_test,
	.poll = gps2scp_test_poll,
};
#endif

#ifdef GPS2SCP_TEST_ENABLE
static int gps2scp_test_init(void)
{
	int ret = 0;
	int err = 0;

	gps2scp_devobj_test = kzalloc(sizeof(*gps2scp_devobj_test), GFP_KERNEL);
	if (gps2scp_devobj_test == NULL) {
		err = -ENOMEM;
		ret = -ENOMEM;
		goto err_out;
	}

	pr_info("Registering gps2scp test chardev\n");
	ret = alloc_chrdev_region(&gps2scp_devobj_test->devno, 0, 1, GPS2SCP_DEVNAME_TEST);
	if (ret) {
		pr_info("alloc_chrdev_region fail: %d\n", ret);
		err = -ENOMEM;
		goto err_out;
	} else {
		pr_info("major: %d, minor: %d\n", MAJOR(gps2scp_devobj_test->devno), MINOR(gps2scp_devobj_test->devno));
	}
	cdev_init(&gps2scp_devobj_test->chdev, &gps2scp_fops_test);
	gps2scp_devobj_test->chdev.owner = THIS_MODULE;
	err = cdev_add(&gps2scp_devobj_test->chdev, gps2scp_devobj_test->devno, 1);
	if (err) {
		pr_info("cdev_add fail: %d\n", err);
		goto err_out;
	}
	gps2scp_devobj_test->cls = class_create(THIS_MODULE, "gps2scp_test");
	if (IS_ERR(gps2scp_devobj_test->cls)) {
		pr_info("Unable to create class, err = %d\n", (int)PTR_ERR(gps2scp_devobj_test->cls));
	goto err_out;
	}
	/* init_MUTEX(&wr_mtx); */
	sema_init(&wr_mtx_test, 1);
	/* init_MUTEX(&rd_mtx); */
	sema_init(&rd_mtx_test, 1);
	/* init_MUTEX(&ring_buf_mtx); */
	sema_init(&ring_buf_mtx_test, 1);
	init_waitqueue_head(&GPS2SCP_wq_test);

	memset(&scp2gps_data_received_test, 0, sizeof(scp2gps_data_received_test));

	gps2scp_devobj_test->dev = device_create(gps2scp_devobj_test->cls, NULL,
		gps2scp_devobj_test->devno, gps2scp_devobj_test, "gps2scp_test");
	pr_info("GPS2SCP test device init Done\n");
	return 0;

err_out:
	if (gps2scp_devobj_test != NULL) {
		if (err == 0)
			cdev_del(&gps2scp_devobj_test->chdev);
		if (ret == 0)
			unregister_chrdev_region(gps2scp_devobj_test->devno, 1);
		kfree(gps2scp_devobj_test);
		gps2scp_devobj_test = NULL;
	}
	return -1;
}
#endif
static int gps2scp_init(void)
{
	int ret = 0;
	int err = 0;

	gps2scp_devobj = kzalloc(sizeof(*gps2scp_devobj), GFP_KERNEL);
	if (gps2scp_devobj == NULL) {
		err = -ENOMEM;
		ret = -ENOMEM;
		goto err_out;
	}

	pr_info("Registering gps2scp chardev\n");
	ret = alloc_chrdev_region(&gps2scp_devobj->devno, 0, 1, GPS2SCP_DEVNAME);
	if (ret) {
		pr_info("alloc_chrdev_region fail: %d\n", ret);
		err = -ENOMEM;
		goto err_out;
	} else {
		pr_info("major: %d, minor: %d\n", MAJOR(gps2scp_devobj->devno), MINOR(gps2scp_devobj->devno));
	}
	cdev_init(&gps2scp_devobj->chdev, &gps2scp_fops);
	gps2scp_devobj->chdev.owner = THIS_MODULE;
	err = cdev_add(&gps2scp_devobj->chdev, gps2scp_devobj->devno, 1);
	if (err) {
		pr_info("cdev_add fail: %d\n", err);
		goto err_out;
	}
	gps2scp_devobj->cls = class_create(THIS_MODULE, "gps2scp");
	if (IS_ERR(gps2scp_devobj->cls)) {
		pr_info("Unable to create class, err = %d\n", (int)PTR_ERR(gps2scp_devobj->cls));
	goto err_out;
	}
	/* init_MUTEX(&wr_mtx); */
	sema_init(&wr_mtx, 1);
	/* init_MUTEX(&rd_mtx); */
	sema_init(&rd_mtx, 1);
	/* init_MUTEX(&ring_buf_mtx); */
	sema_init(&ring_buf_mtx, 1);
	init_waitqueue_head(&GPS2SCP_wq);

	memset(&scp2gps_data_received, 0, sizeof(scp2gps_data_received));

	gps2scp_devobj->dev = device_create(gps2scp_devobj->cls,
		NULL, gps2scp_devobj->devno, gps2scp_devobj, "gps2scp");
	pr_info("GPS2SCP device init Done\n");
	return 0;

err_out:
	if (gps2scp_devobj != NULL) {
		if (err == 0)
			cdev_del(&gps2scp_devobj->chdev);
		if (ret == 0)
			unregister_chrdev_region(gps2scp_devobj->devno, 1);
		kfree(gps2scp_devobj);
		gps2scp_devobj = NULL;
	}
	return -1;
}

/*****************************************************************************/
#ifdef GPS2SCP_TEST_ENABLE
static void gps2scp_test_exit(void)
{
	if (!gps2scp_devobj_test) {
		pr_info("null pointer: %p\n", gps2scp_devobj_test);
		return;
	}

	conap_scp_unregister_drv(DRV_TYPE_GPS);

	pr_info("Unregistering gps2scp test chardev\n");
	cdev_del(&gps2scp_devobj_test->chdev);
	unregister_chrdev_region(gps2scp_devobj_test->devno, 1);
	device_destroy(gps2scp_devobj_test->cls, gps2scp_devobj_test->devno);
	class_destroy(gps2scp_devobj_test->cls);
	kfree(gps2scp_devobj_test);
	gps2scp_devobj_test = NULL;
	pr_info("Done\n");
}
#endif
static void gps2scp_exit(void)
{
	if (!gps2scp_devobj) {
		pr_info("null pointer: %p\n", gps2scp_devobj);
		return;
	}

	conap_scp_unregister_drv(DRV_TYPE_GPS);

	pr_info("Unregistering gps2scp chardev\n");
	cdev_del(&gps2scp_devobj->chdev);
	unregister_chrdev_region(gps2scp_devobj->devno, 1);
	device_destroy(gps2scp_devobj->cls, gps2scp_devobj->devno);
	class_destroy(gps2scp_devobj->cls);
	kfree(gps2scp_devobj);
	gps2scp_devobj = NULL;
	pr_info("Done\n");
}

int mtk_gps2scp_init(void)
{
	pr_info("gps2scp init begin");
#ifdef GPS2SCP_TEST_ENABLE
	pr_info("gps2scp_test init begin");
	gps2scp_test_init();
#endif
	return gps2scp_init();
}

void mtk_gps2scp_exit(void)
{
	pr_info("gps2scp exit begin");
#ifdef GPS2SCP_TEST_ENABLE
	pr_info("gps2scp_test exit begin");
	gps2scp_exit();
#endif
	return gps2scp_exit();
}
#ifdef GPS2SCP_TEST_ENABLE
int mtk_gps2scp_test_init(void)
{
	pr_info("gps2scp init begin");
	return gps2scp_test_init();
}

void mtk_gps2scp_test_exit(void)
{
	pr_info("gps2scp exit begin");
	return gps2scp_test_exit();
}
#endif
/*****************************************************************************/
module_init(mtk_gps2scp_init);
module_exit(mtk_gps2scp_exit);
/*****************************************************************************/
MODULE_AUTHOR("Tianfang li <Tianfang.Li@mediatek.com>");
MODULE_DESCRIPTION("GPS2SCP stp dev");
MODULE_LICENSE("GPL");


