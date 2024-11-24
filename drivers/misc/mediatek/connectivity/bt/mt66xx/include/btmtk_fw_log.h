/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __BTMTK_FW_LOG_H__
#define __BTMTK_FW_LOG_H__

#define BT_FWLOG_IOC_MAGIC          (0xfc)
#define BT_FWLOG_IOC_ON_OFF      _IOW(BT_FWLOG_IOC_MAGIC, 0, int)
#define BT_FWLOG_IOC_SET_LEVEL   _IOW(BT_FWLOG_IOC_MAGIC, 1, int)
#define BT_FWLOG_IOC_GET_LEVEL   _IOW(BT_FWLOG_IOC_MAGIC, 2, int)
#define BT_FWLOG_OFF    0x00
#define BT_FWLOG_ON     0xFF

/* bluetooth kpi */
#define KPI_WITHOUT_TYPE	0

/* Device node */
#define BT_FWLOG_DEV_NODE	"fw_log_bt"

struct btmtk_fops_fwlog {
	dev_t g_devIDfwlog;
	struct cdev BT_cdevfwlog;
	wait_queue_head_t fw_log_inq;
	struct sk_buff_head fwlog_queue;
	struct class *pBTClass;
	struct device *pBTDevfwlog;
	spinlock_t fwlog_lock;
	u8 btmtk_bluetooth_kpi;
};

int btmtk_fops_initfwlog(void);
int btmtk_fops_exitfwlog(void);
void fw_log_bt_event_cb(void);
void fw_log_bt_state_cb(uint8_t state);
/** file_operations: stpbtfwlog */
int btmtk_fops_openfwlog(struct inode *inode, struct file *file);
int btmtk_fops_closefwlog(struct inode *inode, struct file *file);
ssize_t btmtk_fops_readfwlog(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t btmtk_fops_writefwlog(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
unsigned int btmtk_fops_pollfwlog(struct file *filp, poll_table *wait);
long btmtk_fops_unlocked_ioctlfwlog(struct file *filp, unsigned int cmd, unsigned long arg);
long btmtk_fops_compat_ioctlfwlog(struct file *filp, unsigned int cmd, unsigned long arg);
int btmtk_dispatch_fwlog(struct btmtk_dev *bdev, struct sk_buff *skb);
int btmtk_dispatch_fwlog_bluetooth_kpi(struct btmtk_dev *bdev, u8 *buf, int len, u8 type);

#endif /* __BTMTK_FW_LOG_H__ */
