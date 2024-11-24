/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "btmtk_main.h"
#include "btmtk_fw_log.h"

/*
 * BT Logger Tool will turn on/off Firmware Picus log, and set 3 log levels (Low, SQC and Debug)
 * For extention capability, driver does not check the value range.
 *
 * Combine log state and log level to below settings:
 * - 0x00: OFF
 * - 0x01: Low Power
 * - 0x02: SQC
 * - 0x03: Debug
 */


#if (FW_LOG_DEFAULT_ON == 0)
	#define BT_FWLOG_DEFAULT_LEVEL 0x00
#else
	#define BT_FWLOG_DEFAULT_LEVEL 0x02
#endif

/* CTD BT log function and log status */
static wait_queue_head_t BT_log_wq;
static struct semaphore ioctl_mtx;
static uint8_t g_bt_on = BT_FWLOG_OFF;
static uint8_t g_log_on = BT_FWLOG_OFF;
static uint8_t g_log_level = BT_FWLOG_DEFAULT_LEVEL;
static uint8_t g_log_current = BT_FWLOG_OFF;
/* For fwlog dev node setting */
static struct btmtk_fops_fwlog *g_fwlog;

const struct file_operations BT_fopsfwlog = {
	.open = btmtk_fops_openfwlog,
	.release = btmtk_fops_closefwlog,
	.read = btmtk_fops_readfwlog,
	.write = btmtk_fops_writefwlog,
	.poll = btmtk_fops_pollfwlog,
	.unlocked_ioctl = btmtk_fops_unlocked_ioctlfwlog,
	.compat_ioctl = btmtk_fops_compat_ioctlfwlog
};

__weak int32_t btmtk_intcmd_wmt_utc_sync(void)
{
	BTMTK_ERR("weak function %s not implement", __func__);
	return -1;
}

__weak int32_t btmtk_intcmd_set_fw_log(uint8_t flag)
{
	BTMTK_ERR("weak function %s not implement", __func__);
	return -1;
}

void fw_log_bt_state_cb(uint8_t state)
{
	uint8_t on_off;

	on_off = (state == FUNC_ON) ? BT_FWLOG_ON : BT_FWLOG_OFF;
	BTMTK_INFO("bt_on(0x%x) state(%d) on_off(0x%x)", g_bt_on, state, on_off);

	if (g_bt_on != on_off) {
		// changed
		if (on_off == BT_FWLOG_OFF) { // should turn off
			g_bt_on = BT_FWLOG_OFF;
			BTMTK_INFO("BT func off, no need to send hci cmd");
		} else {
			g_bt_on = BT_FWLOG_ON;
			if (g_log_current) {
				btmtk_intcmd_set_fw_log(g_log_current);
				btmtk_intcmd_wmt_utc_sync();
			}
		}
	}
}

void fw_log_bt_event_cb(void)
{
	BTMTK_DBG("fw_log_bt_event_cb");
	wake_up_interruptible(&BT_log_wq);
}

int btmtk_fops_initfwlog(void)
{
	static int BT_majorfwlog;
	dev_t devIDfwlog = MKDEV(BT_majorfwlog, 0);
	int ret = 0;
	int cdevErr = 0;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	BTMTK_INFO("%s: Start", __func__);

	if (g_fwlog == NULL) {
		g_fwlog = kzalloc(sizeof(*g_fwlog), GFP_KERNEL);
		if (!g_fwlog) {
			BTMTK_ERR("%s: alloc memory fail (g_data)", __func__);
			return -1;
		}
	}
	//if (is_mt66xx(g_sbdev->chip_id)) {
	if (bmain_info->hif_hook.log_init) {
		bmain_info->hif_hook.log_init();
		bmain_info->hif_hook.log_register_cb(fw_log_bt_event_cb);
		init_waitqueue_head(&BT_log_wq);
		sema_init(&ioctl_mtx, 1);
	} else {
		spin_lock_init(&g_fwlog->fwlog_lock);
		skb_queue_head_init(&g_fwlog->fwlog_queue);
		init_waitqueue_head(&(g_fwlog->fw_log_inq));
	}

	ret = alloc_chrdev_region(&devIDfwlog, 0, 1, BT_FWLOG_DEV_NODE);
	if (ret) {
		BTMTK_ERR("%s: fail to allocate chrdev", __func__);
		goto alloc_error;
	}

	BT_majorfwlog = MAJOR(devIDfwlog);

	cdev_init(&g_fwlog->BT_cdevfwlog, &BT_fopsfwlog);
	g_fwlog->BT_cdevfwlog.owner = THIS_MODULE;

	cdevErr = cdev_add(&g_fwlog->BT_cdevfwlog, devIDfwlog, 1);
	if (cdevErr)
		goto cdv_error;

	g_fwlog->pBTClass = class_create(THIS_MODULE, BT_FWLOG_DEV_NODE);
	if (IS_ERR(g_fwlog->pBTClass)) {
		BTMTK_ERR("%s: class create fail, error code(%ld)\n", __func__, PTR_ERR(g_fwlog->pBTClass));
		goto create_node_error;
	}

	g_fwlog->pBTDevfwlog = device_create(g_fwlog->pBTClass, NULL, devIDfwlog, NULL,
		BT_FWLOG_DEV_NODE);
	if (IS_ERR(g_fwlog->pBTDevfwlog)) {
		BTMTK_ERR("%s: device(stpbtfwlog) create fail, error code(%ld)", __func__,
			PTR_ERR(g_fwlog->pBTDevfwlog));
		goto create_node_error;
	}
	BTMTK_INFO("%s: BT_majorfwlog %d, devIDfwlog %d", __func__, BT_majorfwlog, devIDfwlog);

	g_fwlog->g_devIDfwlog = devIDfwlog;

	BTMTK_INFO("%s: End", __func__);
	return 0;

create_node_error:
	if (g_fwlog->pBTClass) {
		class_destroy(g_fwlog->pBTClass);
		g_fwlog->pBTClass = NULL;
	}

cdv_error:
	if (cdevErr == 0)
		cdev_del(&g_fwlog->BT_cdevfwlog);

	if (ret == 0)
		unregister_chrdev_region(devIDfwlog, 1);

alloc_error:
	kfree(g_fwlog);
	g_fwlog = NULL;

	return -1;
}

int btmtk_fops_exitfwlog(void)
{
	dev_t devIDfwlog = g_fwlog->g_devIDfwlog;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	BTMTK_INFO("%s: Start\n", __func__);

	//if (is_mt66xx(g_sbdev->chip_id))
	if (bmain_info->hif_hook.log_deinit)
		bmain_info->hif_hook.log_deinit();

	if (g_fwlog->pBTDevfwlog) {
		device_destroy(g_fwlog->pBTClass, devIDfwlog);
		g_fwlog->pBTDevfwlog = NULL;
	}

	if (g_fwlog->pBTClass) {
		class_destroy(g_fwlog->pBTClass);
		g_fwlog->pBTClass = NULL;
	}
	BTMTK_INFO("%s: pBTDevfwlog, pBTClass done\n", __func__);

	cdev_del(&g_fwlog->BT_cdevfwlog);
	unregister_chrdev_region(devIDfwlog, 1);
	BTMTK_INFO("%s: BT_chrdevfwlog driver removed.\n", __func__);

	kfree(g_fwlog);
	return 0;
}

ssize_t btmtk_fops_readfwlog(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int copyLen = 0;
	ulong flags = 0;
	struct sk_buff *skb = NULL;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	//if (is_mt66xx(g_sbdev->chip_id)) {
	if (bmain_info->hif_hook.log_read_to_user) {
		copyLen = bmain_info->hif_hook.log_read_to_user(buf, count);
		BTMTK_DBG("BT F/W log from Connsys, len %d", copyLen);
		return copyLen;
	}

	/* picus read a queue, it may occur performace issue */
	spin_lock_irqsave(&g_fwlog->fwlog_lock, flags);
	if (skb_queue_len(&g_fwlog->fwlog_queue))
		skb = skb_dequeue(&g_fwlog->fwlog_queue);

	spin_unlock_irqrestore(&g_fwlog->fwlog_lock, flags);
	if (skb == NULL)
		return 0;

	if (skb->len <= count) {
		if (copy_to_user(buf, skb->data, skb->len))
			BTMTK_ERR("%s: copy_to_user failed!", __func__);
		copyLen = skb->len;
	} else {
		BTMTK_DBG("%s: socket buffer length error(count: %d, skb.len: %d)",
			__func__, (int)count, skb->len);
	}
	kfree_skb(skb);

	return copyLen;
}
ssize_t btmtk_fops_writefwlog(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int i = 0, len = 0, ret = -1;
	int hci_idx = 0;
	int vlen = 0, index = 3;
	struct sk_buff *skb = NULL;
	int state = BTMTK_STATE_INIT;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	u8 *i_fwlog_buf = NULL;
	u8 *o_fwlog_buf = NULL;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();
	struct btmtk_dev **pp_bdev = btmtk_get_pp_bdev();

	/* only 7xxx will use writefwlog, 66xx not used */
	/*if (is_mt66xx(bdev->chip_id)) {
	 * BTMTK_WARN("%s: not implement!", __func__);
	 * return 0;
	 * }
	 */

	i_fwlog_buf = kmalloc(HCI_MAX_COMMAND_BUF_SIZE, GFP_KERNEL);
	if (!i_fwlog_buf) {
		BTMTK_ERR("%s: alloc i_fwlog_buf failed", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	o_fwlog_buf = kmalloc(HCI_MAX_COMMAND_SIZE, GFP_KERNEL);
	if (!o_fwlog_buf) {
		BTMTK_ERR("%s: alloc o_fwlog_buf failed", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (count > HCI_MAX_COMMAND_BUF_SIZE) {
		BTMTK_ERR("%s: your command is larger than maximum length, count = %zd",
				__func__, count);
		ret = -ENOMEM;
		goto exit;
	}

	memset(i_fwlog_buf, 0, HCI_MAX_COMMAND_BUF_SIZE);
	memset(o_fwlog_buf, 0, HCI_MAX_COMMAND_SIZE);
	if (copy_from_user(i_fwlog_buf, buf, count) != 0) {
		BTMTK_ERR("%s: Failed to copy data", __func__);
		ret = -ENODATA;
		goto exit;
	}

	/* For log level, EX: echo log_lvl=1 > /dev/stpbtfwlog */
	if (strncmp(i_fwlog_buf, "log_lvl=", strlen("log_lvl=")) == 0) {
		u8 val = *(i_fwlog_buf + strlen("log_lvl=")) - '0';

		if (val > BTMTK_LOG_LVL_MAX || val <= 0) {
			BTMTK_ERR("Got incorrect value for log level(%d)", val);
			count =  -EINVAL;
			goto exit;
		}
		btmtk_log_lvl = val;
		BTMTK_INFO("btmtk_log_lvl = %d", btmtk_log_lvl);
		ret = count;
		goto exit;
	}

	/* For bperf, EX: echo bperf=1 > /dev/stpbtfwlog */
	if (strncmp(i_fwlog_buf, "bperf=", strlen("bperf=")) == 0) {
		u8 val = *(i_fwlog_buf + strlen("bperf=")) - '0';

		g_fwlog->btmtk_bluetooth_kpi = val;
		BTMTK_INFO("%s: set bluetooth KPI feature(bperf) to %d", __func__, g_fwlog->btmtk_bluetooth_kpi);
		ret = count;
		goto exit;
	}

	if (strncmp(i_fwlog_buf, "whole chip reset", strlen("whole chip reset")) == 0) {
		BTMTK_INFO("whole chip reset start");
		bmain_info->whole_reset_flag = 1;
		schedule_work(&pp_bdev[hci_idx]->reset_waker);
		ret = count;
		goto exit;
	}
	if (strncmp(i_fwlog_buf, "subsys chip reset", strlen("subsys chip reset")) == 0) {
		BTMTK_INFO("subsys chip reset");
		schedule_work(&pp_bdev[hci_idx]->reset_waker);
		ret = count;
		goto exit;
	}

	/* hci input command format : echo 01 be fc 01 05 > /dev/stpbtfwlog */
	/* We take the data from index three to end. */
	for (i = 0; i < count; i++) {
		char *pos = i_fwlog_buf + i;
		char temp_str[3] = {'\0'};
		long res = 0;

		if (*pos == ' ' || *pos == '\t' || *pos == '\r' || *pos == '\n') {
			continue;
		} else if (*pos == '0' && (*(pos + 1) == 'x' || *(pos + 1) == 'X')) {
			i++;
			continue;
		} else if (!(*pos >= '0' && *pos <= '9') && !(*pos >= 'A' && *pos <= 'F')
			&& !(*pos >= 'a' && *pos <= 'f')) {
			BTMTK_ERR("%s: There is an invalid input(%c)", __func__, *pos);
			ret = -EINVAL;
			goto exit;
		}
		temp_str[0] = *pos;
		temp_str[1] = *(pos + 1);
		i++;
		ret = kstrtol(temp_str, 16, &res);
		if (ret == 0)
			o_fwlog_buf[len++] = (u8)res;
		else
			BTMTK_ERR("%s: Convert %s failed(%d)", __func__, temp_str, ret);
	}

	if (o_fwlog_buf[0] != HCI_COMMAND_PKT && o_fwlog_buf[0] != FWLOG_TYPE) {
		BTMTK_ERR("%s: Not support 0x%02X yet", __func__, o_fwlog_buf[0]);
		ret = -EPROTONOSUPPORT;
		goto exit;
	}
	/* check HCI command length */
	if (len > HCI_MAX_COMMAND_SIZE) {
		BTMTK_ERR("%s: command is larger than max buf size, length = %d", __func__, len);
		ret = -ENOMEM;
		goto exit;
	}

	skb = alloc_skb(count + BT_SKB_RESERVE, GFP_ATOMIC);
	if (!skb) {
		BTMTK_ERR("%s allocate skb failed!!", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	/* send HCI command */
	bt_cb(skb)->pkt_type = HCI_COMMAND_PKT;

	/* format */
	/* 0xF0 XX XX 00 01 AA 10 BB CC CC CC CC ... */
	/* XX XX total length */
	/* 00 : hci index setting type */
	/* AA hci index to indicate which hci send following command*/
	/* 10 : raw data type*/
	/* BB command length */
	/* CC command */
	if (o_fwlog_buf[0] == FWLOG_TYPE) {
		while (index < ((o_fwlog_buf[2] << 8) + o_fwlog_buf[1])) {
			switch (o_fwlog_buf[index]) {
			case FWLOG_HCI_IDX:    /* hci index */
				vlen = o_fwlog_buf[index + 1];
				hci_idx = o_fwlog_buf[index + 2];
				BTMTK_DBG("%s: send to hci%d", __func__, hci_idx);
				index += (FWLOG_ATTR_TL_SIZE + vlen);
				break;
			case FWLOG_TX:    /* tx raw data */
				vlen = o_fwlog_buf[index + 1];
				memcpy(skb->data, o_fwlog_buf + index + FWLOG_ATTR_TL_SIZE, vlen);
				skb->len = vlen;
				index = index + FWLOG_ATTR_TL_SIZE + vlen;
				break;
			default:
				BTMTK_WARN("Invalid opcode");
				ret = -1;
				goto free_skb;
			}
		}
	} else {
		memcpy(skb->data, o_fwlog_buf, len);
		skb->len = len;
		pp_bdev[hci_idx]->opcode_usr[0] = o_fwlog_buf[1];
		pp_bdev[hci_idx]->opcode_usr[1] = o_fwlog_buf[2];
	}

	/* won't send command if g_bdev not define */
	if (pp_bdev[hci_idx]->hdev == NULL) {
		BTMTK_DBG("pp_bdev[%d] not define", hci_idx);
		ret = count;
		goto free_skb;
	}

	state = btmtk_get_chip_state(pp_bdev[hci_idx]);
	if (state != BTMTK_STATE_WORKING) {
		BTMTK_WARN("%s: current is in suspend/resume/standby/dump/disconnect (%d).",
			__func__, state);
		ret = -EBADFD;
		goto free_skb;
	}

	fstate = btmtk_fops_get_state(pp_bdev[hci_idx]);
	if (fstate != BTMTK_FOPS_STATE_OPENED) {
		BTMTK_WARN("%s: fops is not open yet(%d)!", __func__, fstate);
		ret = -ENODEV;
		goto free_skb;
	}

	if (pp_bdev[hci_idx]->power_state == BTMTK_DONGLE_STATE_POWER_OFF) {
		BTMTK_WARN("%s: dongle state already power off, do not write", __func__);
		ret = -EFAULT;
		goto free_skb;
	}

	/* clean fwlog queue before enable picus log */
	if (skb_queue_len(&g_fwlog->fwlog_queue) && skb->data[0] == 0x01
			&& skb->data[1] == 0x5d && skb->data[2] == 0xfc && skb->data[4] == 0x00) {
		skb_queue_purge(&g_fwlog->fwlog_queue);
		BTMTK_INFO("clean fwlog_queue, skb_queue_len = %d", skb_queue_len(&g_fwlog->fwlog_queue));
	}

	btmtk_dispatch_fwlog_bluetooth_kpi(pp_bdev[hci_idx], skb->data, skb->len, KPI_WITHOUT_TYPE);

	ret = bmain_info->hif_hook.send_cmd(pp_bdev[hci_idx], skb, 0, 0, (int)BTMTK_TX_PKT_FROM_HOST);
	if (ret < 0) {
		BTMTK_ERR("%s failed!!", __func__);
		goto free_skb;
	} else
		BTMTK_INFO("%s: OK", __func__);

	BTMTK_INFO("%s: Write end(len: %d)", __func__, len);
	ret = count;
	goto exit;

free_skb:
	kfree_skb(skb);
	skb = NULL;
exit:
	kfree(i_fwlog_buf);
	kfree(o_fwlog_buf);

	return ret;	/* If input is correct should return the same length */
}

int btmtk_fops_openfwlog(struct inode *inode, struct file *file)
{
	BTMTK_INFO("%s: Start.", __func__);

	return 0;
}

int btmtk_fops_closefwlog(struct inode *inode, struct file *file)
{
	BTMTK_INFO("%s: Start.", __func__);

	return 0;
}

long btmtk_fops_unlocked_ioctlfwlog(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long retval = 0;
	uint8_t log_tmp = BT_FWLOG_OFF;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	/* only 66xx will use ioctlfwlog, 76xx not used */
	/* if (!is_mt66xx(g_sbdev->chip_id)) {
	 * BTMTK_WARN("%s: not implement!", __func__);
	 * return 0;
	 *}
	 */

	down(&ioctl_mtx);
	if (bmain_info->hif_hook.log_hold_sem)
		bmain_info->hif_hook.log_hold_sem();

	switch (cmd) {
	case BT_FWLOG_IOC_ON_OFF:
		/* Connsyslogger daemon dynamically enable/disable Picus log */
		BTMTK_INFO("[ON_OFF]arg(%lu) bt_on(0x%x) log_on(0x%x) level(0x%x) log_cur(0x%x)",
			       arg, g_bt_on, g_log_on, g_log_level, g_log_current);

		log_tmp = (arg == 0) ? BT_FWLOG_OFF : BT_FWLOG_ON;
		if (log_tmp != g_log_on) { // changed
			g_log_on = log_tmp;
			g_log_current = g_log_on & g_log_level;
			if (g_bt_on) {
				retval = btmtk_intcmd_set_fw_log(g_log_current);
				btmtk_intcmd_wmt_utc_sync();
			}
		}
		break;
	case BT_FWLOG_IOC_SET_LEVEL:
		/* Connsyslogger daemon dynamically set Picus log level */
		BTMTK_INFO("[SET_LEVEL]arg(%lu) bt_on(0x%x) log_on(0x%x) level(0x%x) log_cur(0x%x)",
			       arg, g_bt_on, g_log_on, g_log_level, g_log_current);

		log_tmp = (uint8_t)arg;
		if (log_tmp != g_log_level) {
			g_log_level = log_tmp;
			g_log_current = g_log_on & g_log_level;
			if (g_bt_on & g_log_on) {
				// driver on and log on
				retval = btmtk_intcmd_set_fw_log(g_log_current);
				btmtk_intcmd_wmt_utc_sync();
			}
		}
		break;
	case BT_FWLOG_IOC_GET_LEVEL:
		retval = g_log_level;
		BTMTK_INFO("[GET_LEVEL]return %ld", retval);
		break;
	default:
		BTMTK_ERR("Unknown cmd: 0x%08x", cmd);
		retval = -EOPNOTSUPP;
		break;
	}

	if (bmain_info->hif_hook.log_release_sem)
		bmain_info->hif_hook.log_release_sem();
	up(&ioctl_mtx);
	return retval;
}

long btmtk_fops_compat_ioctlfwlog(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return btmtk_fops_unlocked_ioctlfwlog(filp, cmd, arg);
}

unsigned int btmtk_fops_pollfwlog(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	//if (is_mt66xx(g_sbdev->chip_id)) {
	if (bmain_info->hif_hook.log_get_buf_size) {
		poll_wait(file, &BT_log_wq, wait);
		if (bmain_info->hif_hook.log_get_buf_size() > 0)
			mask = (POLLIN | POLLRDNORM);
	} else {
		poll_wait(file, &g_fwlog->fw_log_inq, wait);
		if (skb_queue_len(&g_fwlog->fwlog_queue) > 0)
			mask |= POLLIN | POLLRDNORM;			/* readable */
	}
	return mask;
}

static void btmtk_fwdump_wake_lock(void)
{
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	BTMTK_INFO("%s: enter", __func__);
	__pm_stay_awake(bmain_info->fwdump_ws);
	BTMTK_INFO("%s: exit", __func__);
}

static void btmtk_fwdump_wake_unlock(void)
{
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	BTMTK_INFO("%s: enter", __func__);
	__pm_relax(bmain_info->fwdump_ws);
	BTMTK_INFO("%s: exit", __func__);
}

static int btmtk_skb_enq_fwlog(struct btmtk_dev *bdev, void *src, u32 len, u8 type, struct sk_buff_head *queue)
{
	struct sk_buff *skb_tmp = NULL;
	ulong flags = 0;
	int retry = 10, index = FWLOG_TL_SIZE;

	do {
		skb_tmp = alloc_skb(len + FWLOG_PRSV_LEN, GFP_ATOMIC);
		if (skb_tmp != NULL)
			break;
		else if (retry <= 0) {
			pr_info("%s: alloc_skb return 0, error", __func__);
			return -ENOMEM;
		}
		pr_info("%s: alloc_skb return 0, error, retry = %d", __func__, retry);
	} while (retry-- > 0);

	if (type) {
		skb_tmp->data[0] = FWLOG_TYPE;
		/* 01 for dongle index */
		skb_tmp->data[index] = FWLOG_DONGLE_IDX;
		skb_tmp->data[index + 1] = sizeof(bdev->dongle_index);
		skb_tmp->data[index + 2] = bdev->dongle_index;
		index += (FWLOG_ATTR_RX_LEN_LEN + FWLOG_ATTR_TYPE_LEN);
		/* 11 for rx data*/
		skb_tmp->data[index] = FWLOG_RX;
		if (type == HCI_ACLDATA_PKT || type == HCI_EVENT_PKT || type == HCI_COMMAND_PKT) {
			skb_tmp->data[index + 1] = len & 0x00FF;
			skb_tmp->data[index + 2] = (len & 0xFF00) >> 8;
			skb_tmp->data[index + 3] = type;
			index += (HCI_TYPE_SIZE + FWLOG_ATTR_RX_LEN_LEN + FWLOG_ATTR_TYPE_LEN);
		} else {
			skb_tmp->data[index + 1] = len & 0x00FF;
			skb_tmp->data[index + 2] = (len & 0xFF00) >> 8;
			index += (FWLOG_ATTR_RX_LEN_LEN + FWLOG_ATTR_TYPE_LEN);
		}
		memcpy(&skb_tmp->data[index], src, len);
		skb_tmp->data[1] = (len + index - FWLOG_TL_SIZE) & 0x00FF;
		skb_tmp->data[2] = ((len + index - FWLOG_TL_SIZE) & 0xFF00) >> 8;
		skb_tmp->len = len + index;
	} else {
		memcpy(skb_tmp->data, src, len);
		skb_tmp->len = len;
	}


	spin_lock_irqsave(&g_fwlog->fwlog_lock, flags);
	skb_queue_tail(queue, skb_tmp);
	spin_unlock_irqrestore(&g_fwlog->fwlog_lock, flags);
	return 0;
}

int btmtk_dispatch_fwlog_bluetooth_kpi(struct btmtk_dev *bdev, u8 *buf, int len, u8 type)
{
	static u8 fwlog_blocking_warn;
	int ret = 0;

	if (g_fwlog->btmtk_bluetooth_kpi &&
		skb_queue_len(&g_fwlog->fwlog_queue) < FWLOG_BLUETOOTH_KPI_QUEUE_COUNT) {
		/* sent event to queue, picus tool will log it for bluetooth KPI feature */
		if (btmtk_skb_enq_fwlog(bdev, buf, len, type, &g_fwlog->fwlog_queue) == 0) {
			wake_up_interruptible(&g_fwlog->fw_log_inq);
			fwlog_blocking_warn = 0;
		}
	} else {
		if (fwlog_blocking_warn == 0) {
			fwlog_blocking_warn = 1;
			pr_info("btmtk_usb fwlog queue size is full(bluetooth_kpi)");
		}
	}
	return ret;
}

int btmtk_dispatch_fwlog(struct btmtk_dev *bdev, struct sk_buff *skb)
{
	static u8 fwlog_picus_blocking_warn;
	static u8 fwlog_fwdump_blocking_warn;
	int state = BTMTK_STATE_INIT;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	if ((bt_cb(skb)->pkt_type == HCI_ACLDATA_PKT) &&
			skb->data[0] == 0x6f &&
			skb->data[1] == 0xfc) {
		static int dump_data_counter;
		static int dump_data_length;

		state = btmtk_get_chip_state(bdev);
		if (state != BTMTK_STATE_FW_DUMP) {
			BTMTK_INFO("%s: FW dump begin", __func__);
			btmtk_hci_snoop_print_to_log();
			/* Print too much log, it may cause kernel panic. */
			dump_data_counter = 0;
			dump_data_length = 0;
			btmtk_set_chip_state(bdev, BTMTK_STATE_FW_DUMP);
			btmtk_fwdump_wake_lock();
		}

		dump_data_counter++;
		dump_data_length += skb->len;

		/* coredump */
		/* print dump data to console */
		if (dump_data_counter % 1000 == 0) {
			BTMTK_INFO("%s: FW dump on-going, total_packet = %d, total_length = %d",
					__func__, dump_data_counter, dump_data_length);
		}

		/* print dump data to console */
		if (dump_data_counter < 20)
			BTMTK_INFO("%s: FW dump data (%d): %s",
					__func__, dump_data_counter, &skb->data[4]);

		/* In the new generation, we will check the keyword of coredump (; coredump end)
		 * Such as : 79xx
		 */
		if (skb->data[skb->len - 4] == 'e' &&
			skb->data[skb->len - 3] == 'n' &&
			skb->data[skb->len - 2] == 'd') {
			/* This is the latest coredump packet. */
			BTMTK_INFO("%s: FW dump end, dump_data_counter = %d", __func__, dump_data_counter);
			/* TODO: Chip reset*/
			bmain_info->reset_stack_flag = HW_ERR_CODE_CORE_DUMP;
			btmtk_fwdump_wake_unlock();
		}

		if (skb_queue_len(&g_fwlog->fwlog_queue) < FWLOG_ASSERT_QUEUE_COUNT) {
			/* sent picus data to queue, picus tool will log it */
			if (btmtk_skb_enq_fwlog(bdev, skb->data, skb->len, 0, &g_fwlog->fwlog_queue) == 0) {
				wake_up_interruptible(&g_fwlog->fw_log_inq);
				fwlog_fwdump_blocking_warn = 0;
			}
		} else {
			if (fwlog_fwdump_blocking_warn == 0) {
				fwlog_fwdump_blocking_warn = 1;
				pr_info("btmtk fwlog queue size is full(coredump)");
			}
		}

		if (!bdev->bt_cfg.support_picus_to_host)
			return 1;
	} else if ((bt_cb(skb)->pkt_type == HCI_ACLDATA_PKT) &&
				(skb->data[0] == 0xff || skb->data[0] == 0xfe) &&
				skb->data[1] == 0x05 &&
				!bdev->bt_cfg.support_picus_to_host) {
		/* picus or syslog */
		if (skb_queue_len(&g_fwlog->fwlog_queue) < FWLOG_QUEUE_COUNT) {
			if (btmtk_skb_enq_fwlog(bdev, skb->data, skb->len,
				FWLOG_TYPE, &g_fwlog->fwlog_queue) == 0) {
				wake_up_interruptible(&g_fwlog->fw_log_inq);
				fwlog_picus_blocking_warn = 0;
			}
		} else {
			if (fwlog_picus_blocking_warn == 0) {
				fwlog_picus_blocking_warn = 1;
				pr_info("btmtk fwlog queue size is full(picus)");
			}
		}
		return 1;
	} else if ((bt_cb(skb)->pkt_type == HCI_EVENT_PKT) &&
			skb->data[0] == 0x0E &&
			bdev->opcode_usr[0] == skb->data[3] &&
			bdev->opcode_usr[1] == skb->data[4]) {
		BTMTK_INFO_RAW(skb->data, skb->len, "%s: Discard event from user hci command - ", __func__);
		bdev->opcode_usr[0] = 0;
		bdev->opcode_usr[1] = 0;
		return 1;
	}
	return 0;
}

