/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/input.h>
#include <linux/pm_wakeup.h>
#include <linux/reboot.h>
#include <linux/string.h>

#include "btmtk_define.h"
#include "btmtk_main.h"
#include "btmtk_fw_log.h"
#include "btmtk_chip_if.h"

#define MTKBT_UNSLEEPABLE_LOCK(x, y)	spin_lock_irqsave(x, y)
#define MTKBT_UNSLEEPABLE_UNLOCK(x, y)	spin_unlock_irqsave(x, y)

/* TODO, need to modify the state mutex for each hci dev*/
static DEFINE_MUTEX(btmtk_chip_state_mutex);
#define CHIP_STATE_MUTEX_LOCK()	mutex_lock(&btmtk_chip_state_mutex)
#define CHIP_STATE_MUTEX_UNLOCK()	mutex_unlock(&btmtk_chip_state_mutex)
static DEFINE_MUTEX(btmtk_fops_state_mutex);
#define FOPS_MUTEX_LOCK()	mutex_lock(&btmtk_fops_state_mutex)
#define FOPS_MUTEX_UNLOCK()	mutex_unlock(&btmtk_fops_state_mutex)

/**
 * Global parameters(mtkbt_)
 */
uint8_t btmtk_log_lvl = BTMTK_LOG_LVL_DEF;

/* To support dynamic mount of interface can be probed */
static int btmtk_intf_num = BT_MCU_MINIMUM_INTERFACE_NUM;
/* To allow g_bdev being sized from btmtk_intf_num setting */
static struct btmtk_dev **g_bdev;
struct btmtk_dev *g_sbdev;

/*btmtk main information*/
static struct btmtk_main_info main_info;


/* State machine table that clarify through each HIF events,
 * To specify HIF event on
 * Entering / End / Error
 */
static const struct btmtk_cif_state g_cif_state[] = {
	/* HIF_EVENT_PROBE */
	{BTMTK_STATE_PROBE, BTMTK_STATE_WORKING, BTMTK_STATE_DISCONNECT},
	/* HIF_EVENT_DISCONNECT */
	{BTMTK_STATE_DISCONNECT, BTMTK_STATE_DISCONNECT, BTMTK_STATE_DISCONNECT},
	/* HIF_EVENT_SUSPEND */
	{BTMTK_STATE_SUSPEND, BTMTK_STATE_SUSPEND, BTMTK_STATE_FW_DUMP},
	/* HIF_EVENT_RESUME */
	{BTMTK_STATE_RESUME, BTMTK_STATE_WORKING, BTMTK_STATE_FW_DUMP},
	/* HIF_EVENT_STANDBY */
	{BTMTK_STATE_STANDBY, BTMTK_STATE_STANDBY, BTMTK_STATE_FW_DUMP},
	/* BTMTK_STATE_FW_DUMP */
	{BTMTK_STATE_SUBSYS_RESET, BTMTK_STATE_WORKING, BTMTK_STATE_FW_DUMP},
	/* BTMTK_STATE_FW_DUMP */
	{BTMTK_STATE_FW_DUMP, BTMTK_STATE_DISCONNECT, BTMTK_STATE_FW_DUMP},
	/* BTMTK_STATE_FW_DUMP */
	{BTMTK_STATE_FW_DUMP, BTMTK_STATE_FW_DUMP, BTMTK_STATE_FW_DUMP},
};

__weak int btmtk_cif_register(void)
{
	BTMTK_ERR("weak function %s not implement", __func__);
	return -1;
}

__weak int btmtk_cif_deregister(void)
{
	BTMTK_ERR("weak function %s not implement", __func__);
	return -1;
}

__weak int btmtk_cif_send_calibration(struct btmtk_dev *bdev)
{
	BTMTK_ERR("weak function %s not implement", __func__);
	return -1;
}

__weak int btmtk_cif_rx_packet_handler(struct hci_dev *hdev, struct sk_buff *skb)
{
	BTMTK_ERR("weak function %s not implement", __func__);
	return -1;
}
void btmtk_do_gettimeofday(struct timespec64 *time)
{
	ktime_get_real_ts64(time);
}

static int btmtk_send_hci_tci_set_sleep_cmd_766x(struct btmtk_dev *bdev);

/*get 1 byte only*/
int btmtk_efuse_read(struct btmtk_dev *bdev, u16 addr, u8 *value)
{
	uint8_t efuse_r[READ_EFUSE_CMD_LEN] = {0x01, 0x6F, 0xFC, 0x0E,
				0x01, 0x0D, 0x0A, 0x00, 0x02, 0x04,
				0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00};/*4 sub block number(sub block 0~3)*/

	uint8_t efuse_r_event[READ_EFUSE_EVT_HDR_LEN] = {0x04, 0xE4, 0x1E, 0x02, 0x0D, 0x1A, 0x00, 02, 04};
	/*check event
	 *04 E4 LEN(1B) 02 0D LEN(2Byte) 02 04 ADDR(2Byte) VALUE(4B) ADDR(2Byte) VALUE(4Byte)
	 *ADDR(2Byte) VALUE(4B)  ADDR(2Byte) VALUE(4Byte)
	 */
	int ret = 0;
	uint8_t sub_block_addr_in_event = 0;
	uint16_t sub_block = (addr / 16) * 4;
	uint8_t temp = 0;

	efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET] = sub_block & 0xFF;
	efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 1] = (sub_block & 0xFF00) >> 8;
	efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 2] = (sub_block + 1) & 0xFF;
	efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 3] = ((sub_block + 1) & 0xFF00) >> 8;
	efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 4] = (sub_block + 2) & 0xFF;
	efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 5] = ((sub_block + 2) & 0xFF00) >> 8;
	efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 6] = (sub_block + 3) & 0xFF;
	efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 7] = ((sub_block + 3) & 0xFF00) >> 8;

	ret = btmtk_main_send_cmd(bdev,
			efuse_r, READ_EFUSE_CMD_LEN,
			efuse_r_event, READ_EFUSE_EVT_HDR_LEN,
			0, 0, BTMTK_TX_CMD_FROM_DRV);
	if (ret) {
		BTMTK_WARN("btmtk_main_send_cmd error");
		return ret;
	}

	if (memcmp(bdev->io_buf, efuse_r_event, READ_EFUSE_EVT_HDR_LEN) == 0) {
		/*compare rxbuf format ok, compare addr*/
		BTMTK_DBG("compare rxbuf format ok");
		if (efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET] == bdev->io_buf[9] &&
			efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 1] == bdev->io_buf[10] &&
			efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 2] == bdev->io_buf[15] &&
			efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 3] == bdev->io_buf[16] &&
			efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 4] == bdev->io_buf[21] &&
			efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 5] == bdev->io_buf[22] &&
			efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 6] == bdev->io_buf[27] &&
			efuse_r[READ_EFUSE_CMD_BLOCK_OFFSET + 7] == bdev->io_buf[28]) {

			BTMTK_DBG("address compare ok");
			/*Get value*/
			sub_block_addr_in_event = ((addr / 16) / 4);/*cal block num*/
			temp = addr % 16;
			BTMTK_DBG("address in block %d", temp);
			switch (temp) {
			case 0:
			case 1:
			case 2:
			case 3:
				*value = bdev->io_buf[11 + temp];
				break;
			case 4:
			case 5:
			case 6:
			case 7:
				*value = bdev->io_buf[17 + temp];
				break;
			case 8:
			case 9:
			case 10:
			case 11:
				*value = bdev->io_buf[22 + temp];
				break;

			case 12:
			case 13:
			case 14:
			case 15:
				*value = bdev->io_buf[34 + temp];
				break;
			}
		} else {
			BTMTK_WARN("address compare fail");
			ret = -1;
		}
	} else {
		BTMTK_WARN("compare rxbuf format fail");
		ret = -1;
	}

	return ret;
}
void btmtk_free_fw_cfg_struct(struct fw_cfg_struct *fw_cfg, int count)
{
	int i = 0;

	for (i = 0; i < count; i++) {
		if (fw_cfg[i].content) {
			BTMTK_INFO("%s:kfree %d", __func__, i);
			kfree(fw_cfg[i].content);
			fw_cfg[i].content = NULL;
			fw_cfg[i].length = 0;
		} else
			fw_cfg[i].length = 0;
	}
}

void btmtk_free_setting_file(struct btmtk_dev *bdev)
{
	BTMTK_INFO("%s begin", __func__);
	if (bdev == NULL) {
		BTMTK_ERR("%s: bdev == NULL", __func__);
		return;
	}

	btmtk_free_fw_cfg_struct(&bdev->bt_cfg.picus_filter, 1);
	btmtk_free_fw_cfg_struct(&bdev->bt_cfg.picus_enable, 1);
	btmtk_free_fw_cfg_struct(bdev->bt_cfg.phase1_wmt_cmd, PHASE1_WMT_CMD_COUNT);
	btmtk_free_fw_cfg_struct(bdev->bt_cfg.vendor_cmd, VENDOR_CMD_COUNT);

	memset(&bdev->bt_cfg, 0, sizeof(bdev->bt_cfg));
	/* reset pin initial value need to be -1, used to judge after
	 * disconnected before probe, can't do chip reset
	 */
	bdev->bt_cfg.dongle_reset_gpio_pin = -1;
}

static void btmtk_initialize_cfg_items(struct btmtk_dev *bdev)
{
	BTMTK_INFO("%s begin", __func__);
	if (bdev == NULL) {
		BTMTK_ERR("%s: bdev is NULL", __func__);
		return;
	}

	bdev->bt_cfg.dongle_reset_gpio_pin = 220;
	bdev->bt_cfg.support_dongle_reset = 0;
	bdev->bt_cfg.support_full_fw_dump = 0;
	bdev->bt_cfg.support_unify_woble = 1;
	bdev->bt_cfg.unify_woble_type = 0;
	bdev->bt_cfg.support_woble_by_eint = 0;
	bdev->bt_cfg.support_woble_for_bt_disable = 0;
	bdev->bt_cfg.support_woble_wakelock = 0;
	bdev->bt_cfg.reset_stack_after_woble = 0;
	bdev->bt_cfg.support_auto_picus = 0;
	bdev->bt_cfg.support_picus_to_host = 0;
	bdev->bt_cfg.support_bt_single_sku = 0;
	btmtk_free_fw_cfg_struct(&bdev->bt_cfg.picus_filter, 1);
	btmtk_free_fw_cfg_struct(&bdev->bt_cfg.picus_enable, 1);
	btmtk_free_fw_cfg_struct(bdev->bt_cfg.phase1_wmt_cmd, PHASE1_WMT_CMD_COUNT);
	btmtk_free_fw_cfg_struct(bdev->bt_cfg.vendor_cmd, VENDOR_CMD_COUNT);

	BTMTK_INFO("%s end", __func__);
}

unsigned char btmtk_get_chip_state(struct btmtk_dev *bdev)
{
	unsigned char state = BTMTK_STATE_INIT;

	CHIP_STATE_MUTEX_LOCK();
	state = bdev->interface_state;
	CHIP_STATE_MUTEX_UNLOCK();

	return state;
}

void btmtk_set_chip_state(struct btmtk_dev *bdev, unsigned char new_state)
{
	static const char * const state_msg[BTMTK_STATE_MSG_NUM] = {
		"UNKNOWN", "INIT", "DISCONNECT", "PROBE", "WORKING", "SUSPEND", "RESUME",
		"FW_DUMP", "STANDBY", "SUBSYS_RESET",
	};

	if (new_state >= BTMTK_STATE_MSG_NUM) {
		BTMTK_INFO("%s: new_state invalid(%d)", __func__, new_state);
		return;
	}

	BTMTK_INFO("%s: %s(%d) -> %s(%d)", __func__, state_msg[bdev->interface_state],
			bdev->interface_state, state_msg[new_state], new_state);

	CHIP_STATE_MUTEX_LOCK();
	bdev->interface_state = new_state;
	CHIP_STATE_MUTEX_UNLOCK();
}

unsigned char btmtk_fops_get_state(struct btmtk_dev *bdev)
{
	unsigned char state = BTMTK_FOPS_STATE_INIT;

	FOPS_MUTEX_LOCK();
	state = bdev->fops_state;
	FOPS_MUTEX_UNLOCK();

	return state;
}

void btmtk_fops_set_state(struct btmtk_dev *bdev, unsigned char new_state)
{
	static const char * const fstate_msg[BTMTK_FOPS_STATE_MSG_NUM] = {
		"UNKNOWN", "INIT", "OPENING", "OPENED", "CLOSING", "CLOSED",
	};

	if (new_state >= BTMTK_FOPS_STATE_MSG_NUM) {
		BTMTK_INFO("%s: new_state invalid(%d)", __func__, new_state);
		return;
	}

	BTMTK_INFO("%s: FOPS_%s(%d) -> FOPS_%s(%d)", __func__, fstate_msg[bdev->fops_state],
			bdev->fops_state, fstate_msg[new_state], new_state);
	FOPS_MUTEX_LOCK();
	bdev->fops_state = new_state;
	FOPS_MUTEX_UNLOCK();
}

unsigned long btmtk_kallsyms_lookup_name(const char *name)
{
	unsigned long ret = 0;

	// todo: need to unmask, kernel-5.10 not export, submit request to google
	//ret = kallsyms_lookup_name(name);
	if (ret) {
#ifdef CONFIG_ARM
#ifdef CONFIG_THUMB2_KERNEL
		/*set bit 0 in address for thumb mode*/
		ret |= 1;
#endif
#endif
	}
	return ret;
}

static void btmtk_main_info_initialize(void)
{
	memset(&main_info, 0, sizeof(main_info));
	main_info.hci_cmd_index = HCI_SNOOP_ENTRY_NUM - 1;
	main_info.hci_event_index = HCI_SNOOP_ENTRY_NUM - 1;
	main_info.hci_adv_event_index = HCI_SNOOP_ENTRY_NUM - 1;
	main_info.hci_acl_index = HCI_SNOOP_ENTRY_NUM - 1;

#ifdef CONFIG_MP_WAKEUP_SOURCE_SYSFS_STAT
	main_info.fwdump_ws = wakeup_source_register(NULL, "btmtk_fwdump_wakelock");
	main_info.woble_ws = wakeup_source_register(NULL, "btmtk_woble_wakelock");
	main_info.eint_ws = wakeup_source_register(NULL, "btevent_eint");
#else
	main_info.fwdump_ws = wakeup_source_register("btmtk_fwdump_wakelock");
	main_info.woble_ws = wakeup_source_register("btmtk_woble_wakelock");
	main_info.eint_ws = wakeup_source_register("btevent_eint");
#endif

	main_info.wmt_over_hci_header[0] = HCI_COMMAND_PKT;
	main_info.wmt_over_hci_header[1] = 0x6F;
	main_info.wmt_over_hci_header[2] = 0xFC;

	main_info.read_iso_packet_size_cmd[0] = HCI_COMMAND_PKT;
	main_info.read_iso_packet_size_cmd[1] = 0x98;
	main_info.read_iso_packet_size_cmd[2] = 0xFD;
	main_info.read_iso_packet_size_cmd[3] = 0x02;

}

struct btmtk_main_info *btmtk_get_main_info(void)
{
	return &main_info;
}

int btmtk_get_interface_num(void)
{
	return btmtk_intf_num;
}

struct btmtk_dev **btmtk_get_pp_bdev(void)
{
	return g_bdev;
}

void btmtk_hci_snoop_print_to_log(void)
{
	u8 counter, index;

	BTMTK_INFO("HCI Command Dump");
	BTMTK_INFO("Using A5 A5 to separator the head 32 bytes and the tail 32 bytes data");
	BTMTK_INFO("index(len)(timestamp:us) :HCI Command");

	if (main_info.hci_cmd_index >= (HCI_SNOOP_ENTRY_NUM - 1))
		index = 0;
	else
		index = main_info.hci_cmd_index + 1;
	for (counter = 0; counter < HCI_SNOOP_ENTRY_NUM; counter++) {
		if (main_info.hci_cmd_len[index] > 0)
			BTMTK_INFO_RAW(main_info.hci_cmd_buf[index], main_info.hci_cmd_len[index],
				"time(%u)-act_len(%d)-len(%d):", main_info.hci_cmd_timestamp[index],
				main_info.hci_cmd_actual_len[index], main_info.hci_cmd_len[index]);
		index++;
		if (index >= HCI_SNOOP_ENTRY_NUM)
			index = 0;
	}

	BTMTK_INFO("HCI Event Dump");
	BTMTK_INFO("Using A5 A5 to separator the head 32 bytes and the tail 32 bytes data");
	BTMTK_INFO("index(len)(timestamp:us) :HCI Event");
	if (main_info.hci_event_index >= (HCI_SNOOP_ENTRY_NUM - 1))
		index = 0;
	else
		index = main_info.hci_event_index + 1;
	for (counter = 0; counter < HCI_SNOOP_ENTRY_NUM; counter++) {
		if (main_info.hci_event_len[index] > 0)
			BTMTK_INFO_RAW(main_info.hci_event_buf[index], main_info.hci_event_len[index],
				"time(%u)-act_len(%d)-len(%d):", main_info.hci_event_timestamp[index],
				main_info.hci_event_actual_len[index], main_info.hci_event_len[index]);
		index++;
		if (index >= HCI_SNOOP_ENTRY_NUM)
			index = 0;
	}

	BTMTK_INFO("HCI ADV Event Dump");
	BTMTK_INFO("Using A5 A5 to separator the head 32 bytes and the tail 32 bytes data");
	BTMTK_INFO("index(len)(timestamp:us) :HCI ADV Event");
	if (main_info.hci_adv_event_index >= (HCI_SNOOP_ENTRY_NUM - 1))
		index = 0;
	else
		index = main_info.hci_adv_event_index + 1;
	for (counter = 0; counter < HCI_SNOOP_ENTRY_NUM; counter++) {
		if (main_info.hci_adv_event_len[index] > 0)
			BTMTK_INFO_RAW(main_info.hci_adv_event_buf[index], main_info.hci_adv_event_len[index],
				"time(%u)-act_len(%d)-len(%d):", main_info.hci_adv_event_timestamp[index],
				main_info.hci_adv_event_actual_len[index], main_info.hci_adv_event_len[index]);
		index++;
		if (index >= HCI_SNOOP_ENTRY_NUM)
			index = 0;
	}

	BTMTK_INFO("HCI ACL Dump");
	BTMTK_INFO("Using A5 A5 to separator the head 32 bytes and the tail 32 bytes data");
	BTMTK_INFO("index(len)(timestamp:us) :ACL");
	if (main_info.hci_acl_index >= (HCI_SNOOP_ENTRY_NUM - 1))
		index = 0;
	else
		index = main_info.hci_acl_index + 1;
	for (counter = 0; counter < HCI_SNOOP_ENTRY_NUM; counter++) {
		if (main_info.hci_acl_len[index] > 0) {
			BTMTK_INFO_RAW(main_info.hci_acl_buf[index], main_info.hci_acl_len[index],
				"time(%u)-act_len(%d)-len(%d):", main_info.hci_acl_timestamp[index],
				main_info.hci_acl_actual_len[index], main_info.hci_acl_len[index]);
		}
		index++;
		if (index >= HCI_SNOOP_ENTRY_NUM)
			index = 0;
	}
}

static unsigned int btmtk_hci_snoop_get_microseconds(void)
{
	struct timespec64 now = {0, 0};

	btmtk_do_gettimeofday(&now);
	return now.tv_sec * 1000000 + now.tv_nsec/1000;
}

void btmtk_hci_snoop_save_cmd(u32 len, u8 *buf)
{
	u32 copy_len = HCI_SNOOP_BUF_SIZE;
	u32 copy_tail_len = HCI_SNOOP_BUF_SIZE;
	u8 separator_char[SEPARATOR_LEN] = {0xA5, 0xA5};
	u8 *copy_tail_buf;

	if (buf && len > 0 && main_info.hci_cmd_index < HCI_SNOOP_ENTRY_NUM) {
		if (len < HCI_SNOOP_BUF_SIZE) {
			copy_len = len;
			copy_tail_len = 0;
		} else if (len > HCI_SNOOP_BUF_SIZE && len <= HCI_SNOOP_BUF_SIZE * 2)
			copy_tail_len = len - copy_len;

		main_info.hci_cmd_len[main_info.hci_cmd_index] = copy_len & 0xff;
		main_info.hci_cmd_actual_len[main_info.hci_cmd_index] = len & 0xffff;
		main_info.hci_cmd_timestamp[main_info.hci_cmd_index] = btmtk_hci_snoop_get_microseconds();
		memset(main_info.hci_cmd_buf[main_info.hci_cmd_index], 0, HCI_SNOOP_MAX_BUF_SIZE);
		memcpy(main_info.hci_cmd_buf[main_info.hci_cmd_index], buf, copy_len & 0xff);
		/* save less then 32 bytes data in the buffer tail, using A5 A5 to
		 * separator the head 32 bytes data and the tail 32 bytes data
		 */
		if (copy_tail_len > 0) {
			copy_tail_buf = buf + len - copy_tail_len;
			main_info.hci_cmd_len[main_info.hci_cmd_index] +=
				(copy_tail_len + SEPARATOR_LEN) & 0xff;
			memcpy(main_info.hci_cmd_buf[main_info.hci_cmd_index] + copy_len, separator_char,
				SEPARATOR_LEN);
			memcpy(main_info.hci_cmd_buf[main_info.hci_cmd_index] + copy_len + SEPARATOR_LEN,
				copy_tail_buf, copy_tail_len);
		}

		if (main_info.hci_cmd_index == 0)
			main_info.hci_cmd_index = HCI_SNOOP_ENTRY_NUM;
		main_info.hci_cmd_index--;
	}
}

void btmtk_hci_snoop_save_adv_event(u32 len, u8 *buf)
{
	u32 copy_len = HCI_SNOOP_BUF_SIZE;
	u32 copy_tail_len = HCI_SNOOP_BUF_SIZE;
	u8 separator_char[SEPARATOR_LEN] = {0xA5, 0xA5};
	u8 *copy_tail_buf;

	if (buf && len > 0 && main_info.hci_adv_event_index < HCI_SNOOP_ENTRY_NUM) {
		if (len < HCI_SNOOP_BUF_SIZE) {
			copy_len = len;
			copy_tail_len = 0;
		} else if (len > HCI_SNOOP_BUF_SIZE && len <= HCI_SNOOP_BUF_SIZE * 2)
			copy_tail_len = len - copy_len;

		main_info.hci_adv_event_len[main_info.hci_adv_event_index] = copy_len & 0xff;
		main_info.hci_adv_event_actual_len[main_info.hci_adv_event_index] = len & 0xffff;
		main_info.hci_adv_event_timestamp[main_info.hci_adv_event_index] = btmtk_hci_snoop_get_microseconds();
		memset(main_info.hci_adv_event_buf[main_info.hci_adv_event_index], 0, HCI_SNOOP_MAX_BUF_SIZE);
		memcpy(main_info.hci_adv_event_buf[main_info.hci_adv_event_index], buf, copy_len);
		/* save less then 32 bytes data in the buffer tail, using A5 A5 to
		 * separator the head 32 bytes data and the tail 32 bytes data
		 */
		if (copy_tail_len > 0) {
			copy_tail_buf = buf + len - copy_tail_len;
			main_info.hci_adv_event_len[main_info.hci_adv_event_index] +=
				(copy_tail_len + SEPARATOR_LEN) & 0xff;
			memcpy(main_info.hci_adv_event_buf[main_info.hci_adv_event_index] + copy_len, separator_char,
				SEPARATOR_LEN);
			memcpy(main_info.hci_adv_event_buf[main_info.hci_adv_event_index] + copy_len + SEPARATOR_LEN,
				copy_tail_buf, copy_tail_len);
		}

		if (main_info.hci_adv_event_index == 0)
			main_info.hci_adv_event_index = HCI_SNOOP_ENTRY_NUM;
		main_info.hci_adv_event_index--;
	}
}

void btmtk_hci_snoop_save_event(u32 len, u8 *buf)
{
	u32 copy_len = HCI_SNOOP_BUF_SIZE;
	u32 copy_tail_len = HCI_SNOOP_BUF_SIZE;
	u8 separator_char[SEPARATOR_LEN] = {0xA5, 0xA5};
	u8 *copy_tail_buf;

	if (buf && len > 0 && main_info.hci_event_index < HCI_SNOOP_ENTRY_NUM) {
		if (len < HCI_SNOOP_BUF_SIZE) {
			copy_len = len;
			copy_tail_len = 0;
		} else if (len > HCI_SNOOP_BUF_SIZE && len <= HCI_SNOOP_BUF_SIZE * 2)
			copy_tail_len = len - copy_len;

		main_info.hci_event_len[main_info.hci_event_index] = copy_len & 0xff;
		main_info.hci_event_actual_len[main_info.hci_event_index] = len & 0xffff;
		main_info.hci_event_timestamp[main_info.hci_event_index] = btmtk_hci_snoop_get_microseconds();
		memset(main_info.hci_event_buf[main_info.hci_event_index], 0, HCI_SNOOP_MAX_BUF_SIZE);
		memcpy(main_info.hci_event_buf[main_info.hci_event_index], buf, copy_len);
		/* save less then 32 bytes data in the buffer tail, using A5 A5 to
		 * separator the head 32 bytes data and the tail 32 bytes data
		 */
		if (copy_tail_len > 0) {
			copy_tail_buf = buf + len - copy_tail_len;
			main_info.hci_event_len[main_info.hci_event_index] +=
				(copy_tail_len + SEPARATOR_LEN) & 0xff;
			memcpy(main_info.hci_event_buf[main_info.hci_event_index] + copy_len, separator_char,
				SEPARATOR_LEN);
			memcpy(main_info.hci_event_buf[main_info.hci_event_index] + copy_len + SEPARATOR_LEN,
				copy_tail_buf, copy_tail_len);
		}

		if (main_info.hci_event_index == 0)
			main_info.hci_event_index = HCI_SNOOP_ENTRY_NUM;
		main_info.hci_event_index--;
	}
}

void btmtk_hci_snoop_save_acl(u32 len, u8 *buf)
{
	u32 copy_len = HCI_SNOOP_BUF_SIZE;
	u32 copy_tail_len = HCI_SNOOP_BUF_SIZE;
	u8 separator_char[SEPARATOR_LEN] = {0xA5, 0xA5};
	u8 *copy_tail_buf;

	if (buf && len > 0 && main_info.hci_acl_index < HCI_SNOOP_ENTRY_NUM) {
		if (len < HCI_SNOOP_BUF_SIZE) {
			copy_len = len;
			copy_tail_len = 0;
		} else if (len > HCI_SNOOP_BUF_SIZE && len <= HCI_SNOOP_BUF_SIZE * 2)
			copy_tail_len = len - copy_len;

		main_info.hci_acl_len[main_info.hci_acl_index] = copy_len & 0xff;
		main_info.hci_acl_actual_len[main_info.hci_acl_index] = len & 0xffff;
		main_info.hci_acl_timestamp[main_info.hci_acl_index] = btmtk_hci_snoop_get_microseconds();
		memset(main_info.hci_acl_buf[main_info.hci_acl_index], 0, HCI_SNOOP_MAX_BUF_SIZE);
		memcpy(main_info.hci_acl_buf[main_info.hci_acl_index], buf, copy_len & 0xff);
		/* save less then 32 bytes data in the buffer tail, using A5 A5 to
		 * separator the head 32 bytes data and the tail 32 bytes data
		 */
		if (copy_tail_len > 0) {
			copy_tail_buf = buf + len - copy_tail_len;
			main_info.hci_acl_len[main_info.hci_acl_index] +=
				(copy_tail_len + SEPARATOR_LEN) & 0xff;
			memcpy(main_info.hci_acl_buf[main_info.hci_acl_index] + copy_len, separator_char,
				SEPARATOR_LEN);
			memcpy(main_info.hci_acl_buf[main_info.hci_acl_index] + copy_len + SEPARATOR_LEN,
				copy_tail_buf, copy_tail_len);
		}

		if (main_info.hci_acl_index == 0)
			main_info.hci_acl_index = HCI_SNOOP_ENTRY_NUM;
		main_info.hci_acl_index--;
	}
}

void btmtk_hci_snoop_print(u32 len, const u8 *buf)
{
	u32 copy_len = HCI_SNOOP_BUF_SIZE;
	u32 copy_tail_len = HCI_SNOOP_BUF_SIZE;
	u8 separator_char[SEPARATOR_LEN] = {0xA5, 0xA5};
	const u8 *copy_tail_buf;
	u8 hci_snoop_buf[HCI_SNOOP_MAX_BUF_SIZE] = {0};
	u16 hci_snoop_len = 0;

	if (buf && len > 0) {
		if (len < HCI_SNOOP_BUF_SIZE) {
			copy_len = len;
			copy_tail_len = 0;
		} else if (len > HCI_SNOOP_BUF_SIZE && len <= HCI_SNOOP_BUF_SIZE * 2)
			copy_tail_len = len - copy_len;

		memcpy(hci_snoop_buf, buf, copy_len & 0xff);
		hci_snoop_len = copy_len & 0xff;

		/* save less then 32 bytes data in the buffer tail, using A5 A5 to
		 * separator the head 32 bytes data and the tail 32 bytes data
		 */
		if (copy_tail_len > 0) {
			copy_tail_buf = buf + len - copy_tail_len;
			hci_snoop_len += (copy_tail_len + SEPARATOR_LEN) & 0xff;
			memcpy(hci_snoop_buf + copy_len, separator_char, SEPARATOR_LEN);
			memcpy(hci_snoop_buf + copy_len + SEPARATOR_LEN,
				copy_tail_buf, copy_tail_len);
		}

		if (hci_snoop_len > 0)
			BTMTK_INFO_RAW(hci_snoop_buf, hci_snoop_len, "act_len(%d)-len(%d)-buf(%p):",
				len, hci_snoop_len, buf);
	}
}

/* HCI receive mechnism */
static inline struct sk_buff *h4_recv_buf(struct hci_dev *hdev,
					  struct sk_buff *skb,
					  const unsigned char *buffer,
					  int count,
					  const struct h4_recv_pkt *pkts,
					  int pkts_count)
{
	struct btmtk_dev *bdev = NULL;
	/* used for print debug log*/
	const unsigned char *buffer_dbg = buffer;
	int count_dbg = count;

	if (hdev == NULL || buffer == NULL) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return ERR_PTR(-EINVAL);
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL) {
		BTMTK_ERR("%s, bdev is invalid", __func__);
		return ERR_PTR(-EINVAL);
	}
	/* Check for error from previous call */
	if (IS_ERR(skb))
		skb = NULL;
	/* BTMTK_DBG("%s begin, count = %d", __func__, count); */

	while (count) {
		int i, len;

		if (!skb) {
			for (i = 0; i < pkts_count; i++) {
				if (buffer[0] != (&pkts[i])->type)
					continue;

				skb = bt_skb_alloc((&pkts[i])->maxlen,
						   GFP_ATOMIC);
				if (!skb) {
					BTMTK_ERR("%s, alloc skb failed!", __func__);
					return ERR_PTR(-ENOMEM);
				}

				hci_skb_pkt_type(skb) = (&pkts[i])->type;
				hci_skb_expect(skb) = (&pkts[i])->hlen;
				break;
			}

			/* Check for invalid packet type */
			if (!skb) {
				BTMTK_ERR("%s,skb is invalid, buffer[0] = %d!", __func__,
					buffer[0]);
				if (is_mt66xx(bdev->chip_id))
					btmtk_set_sleep(hdev, FALSE);
				else {
					btmtk_hci_snoop_print(count_dbg, buffer_dbg);
					btmtk_hci_snoop_print(count, buffer);
					btmtk_hci_snoop_print_to_log();
				}
				return ERR_PTR(-EILSEQ);
			}

			count -= 1;
			buffer += 1;
		}

		len = min_t(uint, hci_skb_expect(skb) - skb->len, count);
		memcpy(skb_put(skb, len), buffer, len);
		/* If kernel version > 4.x */
		/* skb_put_data(skb, buffer, len); */

		count -= len;
		buffer += len;

		/* BTMTK_DBG("%s skb->len = %d, %d", __func__, skb->len, hci_skb_expect(skb)); */

		/* Check for partial packet */
		if (skb->len < hci_skb_expect(skb))
			continue;

		for (i = 0; i < pkts_count; i++) {
			if (hci_skb_pkt_type(skb) == (&pkts[i])->type)
				break;
		}

		if (i >= pkts_count) {
			BTMTK_ERR("%s, pkt type is invalid!", __func__);
			if (is_mt66xx(bdev->chip_id))
				btmtk_set_sleep(hdev, FALSE);
			else {
				btmtk_hci_snoop_print(count_dbg, buffer_dbg);
				btmtk_hci_snoop_print(count, buffer);
				btmtk_hci_snoop_print_to_log();
			}
			kfree_skb(skb);
			return ERR_PTR(-EILSEQ);
		}

		if (skb->len == (&pkts[i])->hlen) {
			u16 dlen;

			/* BTMTK_DBG("%s begin, skb->len = %d, %d, %d", __func__, skb->len, */
			/* (&pkts[i])->hlen, (&pkts[i])->lsize); */
			switch ((&pkts[i])->lsize) {
			case 0:
				/* No variable data length */
				dlen = 0;
				break;
			case 1:
				/* Single octet variable length */
				dlen = skb->data[(&pkts[i])->loff];
				hci_skb_expect(skb) += dlen;

				if (skb_tailroom(skb) < dlen) {
					BTMTK_ERR("%s, skb_tailroom is not enough, dlen:%d!",
						__func__, dlen);
					if (is_mt66xx(bdev->chip_id))
						btmtk_set_sleep(hdev, FALSE);
					else {
						btmtk_hci_snoop_print(skb->len, skb->data);
						btmtk_hci_snoop_print(count_dbg, buffer_dbg);
						btmtk_hci_snoop_print(count, buffer);
						btmtk_hci_snoop_print_to_log();
					}
					kfree_skb(skb);
					return ERR_PTR(-EMSGSIZE);
				}
				break;
			case 2:
				/* Double octet variable length */
				dlen = get_unaligned_le16(skb->data +
							  (&pkts[i])->loff);
				/* parse ISO packet len*/
				if ((&pkts[i])->type == HCI_ISODATA_PKT) {
					unsigned char *cp = (unsigned char *)&dlen + 1;
					*cp = *cp & 0x3F;
				}
				hci_skb_expect(skb) += dlen;

				if (skb_tailroom(skb) < dlen) {
					BTMTK_ERR("%s, skb_tailroom is not enough in case 2, dlen:%d!",
						__func__, dlen);
					if (is_mt66xx(bdev->chip_id))
						btmtk_set_sleep(hdev, FALSE);
					else {
						btmtk_hci_snoop_print(skb->len, skb->data);
						btmtk_hci_snoop_print(count_dbg, buffer_dbg);
						btmtk_hci_snoop_print(count, buffer);
						btmtk_hci_snoop_print_to_log();
					}
					kfree_skb(skb);
					return ERR_PTR(-EMSGSIZE);
				}
				break;
			default:
				/* Unsupported variable length */
				BTMTK_ERR("%s, Unsupported variable length!", __func__);
				if (is_mt66xx(bdev->chip_id))
					btmtk_set_sleep(hdev, FALSE);
				else {
					btmtk_hci_snoop_print(count_dbg, buffer_dbg);
					btmtk_hci_snoop_print(count, buffer);
					btmtk_hci_snoop_print_to_log();
				}
				kfree_skb(skb);
				return ERR_PTR(-EILSEQ);
			}

			if (!dlen) {
				/* No more data, complete frame */
				(&pkts[i])->recv(hdev, skb);
				if (is_mt66xx(bdev->chip_id))
					btmtk_set_sleep(hdev, FALSE);
				skb = NULL;
			}
		} else {
			/* Complete frame */
			(&pkts[i])->recv(hdev, skb);
			if (is_mt66xx(bdev->chip_id))
				btmtk_set_sleep(hdev, FALSE);
			skb = NULL;
		}
	}

	return skb;
}

static const struct h4_recv_pkt mtk_recv_pkts[] = {
	{ H4_RECV_ACL,      .recv = btmtk_recv_acl },
	{ H4_RECV_SCO,      .recv = hci_recv_frame },
	{ H4_RECV_EVENT,    .recv = btmtk_recv_event },
	{ H4_RECV_ISO,      .recv = btmtk_recv_iso },
};

#if ENABLESTP
static inline struct sk_buff *mtk_add_stp(struct btmtk_dev *bdev, struct sk_buff *skb)
{
	struct mtk_stp_hdr *shdr;
	int dlen, err = 0, type = 0;
	u8 stp_crc[STP_CRC_LEN] = {0x00, 0x00};

	if (unlikely(skb_headroom(skb) < sizeof(*shdr)) ||
		(skb_tailroom(skb) < MTK_STP_TLR_SIZE)) {
		BTMTK_DBG("%s, add pskb_expand_head, headroom = %d, tailroom = %d",
				__func__, skb_headroom(skb), skb_tailroom(skb));

		err = pskb_expand_head(skb, sizeof(*shdr), MTK_STP_TLR_SIZE,
					   GFP_ATOMIC);
	}
	dlen = skb->len;
	shdr = (void *) skb_push(skb, sizeof(*shdr));
	shdr->prefix = 0x80;
	shdr->dlen = cpu_to_be16((dlen & 0x0fff) | (type << 12));
	shdr->cs = 0;
/*	Add the STP trailer
 *	kernel version > 4.20
 *	skb_put_zero(skb, MTK_STP_TLR_SIZE);
 *	kernel version < 4.20
 */
	skb_put(skb, STP_CRC_LEN);

	return skb;
}

static const unsigned char *
mtk_stp_split(struct btmtk_dev *bdev, const unsigned char *data, int count,
	      int *sz_h4)
{
	struct mtk_stp_hdr *shdr;

	/* The cursor is reset when all the data of STP is consumed out */
	if (!bdev->stp_dlen && bdev->stp_cursor >= 6) {
		bdev->stp_cursor = 0;
		BTMTK_ERR("reset cursor = %d\n", bdev->stp_cursor);
	}

	/* Filling pad until all STP info is obtained */
	while (bdev->stp_cursor < 6 && count > 0) {
		bdev->stp_pad[bdev->stp_cursor] = *data;
		pr_info("fill stp format (%02x, %d, %d)\n",
		   bdev->stp_pad[bdev->stp_cursor], bdev->stp_cursor, count);
		bdev->stp_cursor++;
		data++;
		count--;
	}

	/* Retrieve STP info and have a sanity check */
	if (!bdev->stp_dlen && bdev->stp_cursor >= 6) {
		shdr = (struct mtk_stp_hdr *)&bdev->stp_pad[2];
		bdev->stp_dlen = be16_to_cpu(shdr->dlen) & 0x0fff;
		pr_info("stp format (%02x, %02x)",
			   shdr->prefix, bdev->stp_dlen);

		/* Resync STP when unexpected data is being read */
		if (shdr->prefix != 0x80 || bdev->stp_dlen > 2048) {
			BTMTK_ERR("stp format unexpect (%02x, %02x)",
				   shdr->prefix, bdev->stp_dlen);
			BTMTK_ERR("reset cursor = %d\n", bdev->stp_cursor);
			bdev->stp_cursor = 2;
			bdev->stp_dlen = 0;
		}
	}

	/* Directly quit when there's no data found for H4 can process */
	if (count <= 0)
		return NULL;

	/* Tranlate to how much the size of data H4 can handle so far */
	*sz_h4 = min_t(int, count, bdev->stp_dlen);

	/* Update the remaining size of STP packet */
	bdev->stp_dlen -= *sz_h4;

	/* Data points to STP payload which can be handled by H4 */
	return data;
}
#endif

int btmtk_recv(struct hci_dev *hdev, const u8 *data, size_t count)
{
	struct btmtk_dev *bdev = NULL;
	const unsigned char *p_left = data;
	int sz_left = count;
	int err;
#if ENABLESTP
	const unsigned char **p_h4 = NULL;
	int sz_h4 = 0, adv = 0;
#endif

	if (hdev == NULL || data == NULL) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -EINVAL;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL) {
		BTMTK_ERR("%s, bdev is NULL!", __func__);
		return -EINVAL;
	}

	while (sz_left > 0) {
		/*  The serial data received from MT7622 BT controller is
		 *  at all time padded around with the STP header and tailer.
		 *
		 *  A full STP packet is looking like
		 *   -----------------------------------
		 *  | STP header  |  H:4   | STP tailer |
		 *   -----------------------------------
		 *  but it doesn't guarantee to contain a full H:4 packet which
		 *  means that it's possible for multiple STP packets forms a
		 *  full H:4 packet that means extra STP header + length doesn't
		 *  indicate a full H:4 frame, things can fragment. Whose length
		 *  recorded in STP header just shows up the most length the
		 *  H:4 engine can handle currently.
		 */
#if ENABLESTP
		p_h4 = mtk_stp_split(bdev, p_left, sz_left, &sz_h4);
		if (!p_h4)
			break;

		adv = p_h4 - p_left;
		sz_left -= adv;
		p_left += adv;
#endif

#if ENABLESTP
		bdev->rx_skb = h4_recv_buf(hdev, bdev->rx_skb, p_h4,
					   sz_h4, mtk_recv_pkts,
					   ARRAY_SIZE(mtk_recv_pkts));
#else
		bdev->rx_skb = h4_recv_buf(hdev, bdev->rx_skb, data,
					   count, mtk_recv_pkts,
					   ARRAY_SIZE(mtk_recv_pkts));
#endif

		if (IS_ERR(bdev->rx_skb)) {
			err = PTR_ERR(bdev->rx_skb);
			BTMTK_ERR("Frame reassembly failed (%d)", err);
			bdev->rx_skb = NULL;
			return err;
		}

#if ENABLESTP
		sz_left -= sz_h4;
		p_left += sz_h4;
#else
		sz_left -= count;
		p_left += count;
#endif
	}

	return 0;
}


int btmtk_recv_acl(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = NULL;

	if (hdev == NULL || skb == NULL) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -EINVAL;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL || bdev->workqueue == NULL) {
		BTMTK_ERR("%s, bdev or workqueue is invalid!", __func__);
		return -EINVAL;
	}

	skb_queue_tail(&bdev->rx_q, skb);
	queue_work(bdev->workqueue, &bdev->rx_work);

	/* remove it, if workqueue can't be scheduled, you can reuse it */
#if 0
	skip_pkt = btmtk_dispatch_fwlog(bdev, skb);
	if (skip_pkt == 0)
		err = hci_recv_frame(hdev, skb);
#endif
	return 0;
}


int btmtk_recv_event(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = NULL;

	if (hdev == NULL || skb == NULL) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -EINVAL;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL || bdev->workqueue == NULL) {
		BTMTK_ERR("%s, bdev or workqueue is invalid!", __func__);
		kfree_skb(skb);
		return -EINVAL;
	}

	/* Fix up the vendor event id with 0xff for vendor specific instead
	 * of 0xe4 so that event send via monitoring socket can be parsed
	 * properly.
	 */
	/* if (hdr->evt == 0xe4) {
	 * BTMTK_DBG("%s hdr->evt is %02x", __func__, hdr->evt);
	 * hdr->evt = HCI_EV_VENDOR;
	 * }
	 */

	BTMTK_DBG_RAW(skb->data, skb->len, "%s, recv evt(hci_recv_frame)", __func__);

	skb_queue_tail(&bdev->rx_q, skb);
	queue_work(bdev->workqueue, &bdev->rx_work);

	/* remove it, if workqueue can't be scheduled, you can reuse it */
#if 0
	skip_pkt = btmtk_dispatch_event(hdev, skb);
	if (skip_pkt == 0)
		err = hci_recv_frame(hdev, skb);

	if (err < 0) {
		BTMTK_ERR("%s hci_recv_failed, err = %d", __func__, err);
		goto err_out;
	}
#endif
	return 0;
}

int btmtk_recv_iso(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = NULL;

	if (hdev == NULL || skb == NULL) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -EINVAL;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL || bdev->workqueue == NULL) {
		BTMTK_ERR("%s, bdev or workqueue is invalid!", __func__);
		kfree_skb(skb);
		return -EINVAL;
	}

	skb_queue_tail(&bdev->rx_q, skb);
	queue_work(bdev->workqueue, &bdev->rx_work);

	return 0;
}

int btmtk_main_send_cmd(struct btmtk_dev *bdev, const uint8_t *cmd,
		const int cmd_len, const uint8_t *event, const int event_len, int delay,
		int retry, int pkt_type)
{
	struct sk_buff *skb = NULL;
	int ret = 0;
	int state = BTMTK_STATE_INIT;

	if (bdev == NULL || bdev->hdev == NULL ||
		cmd == NULL || cmd_len <= 0) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if (!is_mt66xx(bdev->chip_id) &&
		memcmp(cmd, main_info.wmt_over_hci_header, WMT_OVER_HCI_HEADER_SIZE) &&
		pkt_type != BTMTK_TX_ACL_FROM_DRV &&
		bdev->power_state != BTMTK_DONGLE_STATE_POWER_ON) {
		BTMTK_WARN("%s: chip power isn't on, ignore this command, state is %d",
			__func__, bdev->power_state);
		goto exit;
	}

	state = btmtk_get_chip_state(bdev);
	if (state == BTMTK_STATE_FW_DUMP) {
		BTMTK_WARN("%s: FW dumping ongoing, don't send any cmd to FW!!!", __func__);
		ret = -1;
		goto exit;
	}

	skb = alloc_skb(cmd_len + BT_SKB_RESERVE, GFP_ATOMIC);
	if (skb == NULL) {
		BTMTK_ERR("%s allocate skb failed!!", __func__);
		goto exit;
	}
	/* Reserv for core and drivers use */
	skb_reserve(skb, 7);
	bt_cb(skb)->pkt_type = HCI_COMMAND_PKT;
	memcpy(skb->data, cmd, cmd_len);
	skb->len = cmd_len;

#if ENABLESTP
	skb = mtk_add_stp(bdev, skb);
#endif
	/* wmt cmd and download fw patch using wmt cmd with USB interface, need use
	 * usb_control_msg to recv wmt event;
	 * other HIF don't use this method to recv wmt event
	 */

	ret = main_info.hif_hook.send_and_recv(bdev,
			skb,
			event, event_len,
			delay, retry, pkt_type);

	if (ret < 0)
		BTMTK_ERR("%s send_and_recv failed!!", __func__);

exit:
	BTMTK_DBG("%s end!!", __func__);
	return ret;
}

static int btmtk_check_need_load_rom_patch(struct btmtk_dev *bdev)
{
	u8 cmd[CHECK_LD_PATCH_CMD_LEN] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x17, 0x01, 0x00, 0x01 };
	u8 event[CHECK_LD_PATCH_EVT_HDR_LEN] = { 0x04, 0xE4, 0x05, 0x02, 0x17, 0x01, 0x00, /* 0x02 */ };
	int ret = -1;

	if (!bdev) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		ret = -EINVAL;
		return ret;
	}

	ret = btmtk_main_send_cmd(bdev,
			cmd, CHECK_LD_PATCH_CMD_LEN,
			event, CHECK_LD_PATCH_EVT_HDR_LEN,
			DELAY_TIMES, RETRY_TIMES, BTMTK_TX_CMD_FROM_DRV);
	/* can't get correct event */
	if (ret < 0)
		return PATCH_ERR;

	if (bdev->recv_evt_len == CHECK_LD_PATCH_EVT_HDR_LEN)
		return bdev->io_buf[CHECK_LD_PATCH_EVT_RESULT_OFFSET];

	return PATCH_ERR;
}

int btmtk_load_code_from_bin(u8 **image, char *bin_name, struct device *dev,
											u32 *code_len, u8 retry)
{
	const struct firmware *fw_entry = NULL;
	int err = 0;

	if (!bin_name) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -1;
	}

	do {
		err = request_firmware(&fw_entry, bin_name, dev);
		if (err == 0) {
			break;
		} else if (retry <= 0) {
			*image = NULL;
			BTMTK_INFO("%s: request_firmware %d times fail, maybe file not exist, err = %d",
				__func__, 10, err);
			return -1;
		}
		BTMTK_INFO("%s: request_firmware fail, maybe file not exist, err = %d, retry = %d",
			__func__, err, retry);
		msleep(100);
	} while (retry-- > 0);

	*image = vmalloc(ALIGN_4(fw_entry->size));
	if (*image == NULL) {
		*code_len = 0;
		BTMTK_ERR("%s: vmalloc failed!! error code = %d", __func__, err);
		return -1;
	}

	memcpy(*image, fw_entry->data, fw_entry->size);
	*code_len = fw_entry->size;

	release_firmware(fw_entry);
	return 0;
}

static void btmtk_print_bt_patch_info(struct btmtk_dev *bdev, u8 *fwbuf)
{
	struct _PATCH_HEADER *patchHdr = NULL;
	struct _Global_Descr *globalDesrc = NULL;

	if (fwbuf == NULL) {
		BTMTK_WARN("%s, fwbuf is NULL!", __func__);
		return;
	}

	patchHdr = (struct _PATCH_HEADER *)fwbuf;

	if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id))
		globalDesrc = (struct _Global_Descr *)(fwbuf + FW_ROM_PATCH_HEADER_SIZE);

	BTMTK_INFO("[btmtk] =============== Patch Info ==============");
	if (patchHdr) {
		BTMTK_INFO("[btmtk] Built Time = %s", patchHdr->ucDateTime);
		BTMTK_INFO("[btmtk] Hw Ver = 0x%04x", patchHdr->u2HwVer);
		BTMTK_INFO("[btmtk] Sw Ver = 0x%04x", patchHdr->u2SwVer);
		BTMTK_INFO("[btmtk] Magic Number = 0x%08x", patchHdr->u4MagicNum);

		BTMTK_INFO("[btmtk] Platform = %c%c%c%c",
				patchHdr->ucPlatform[0],
				patchHdr->ucPlatform[1],
				patchHdr->ucPlatform[2],
				patchHdr->ucPlatform[3]);
	} else
		BTMTK_WARN("%s, patchHdr is NULL!", __func__);

	if (globalDesrc) {
		BTMTK_INFO("[btmtk] Patch Ver = 0x%08x", globalDesrc->u4PatchVer);
		BTMTK_INFO("[btmtk] Section num = 0x%08x", globalDesrc->u4SectionNum);
	} else
		BTMTK_WARN("%s, globalDesrc is NULL!", __func__);
	BTMTK_INFO("[btmtk] =========================================");
}

static void btmtk_print_wifi_patch_info(struct btmtk_dev *bdev, u8 *fwbuf)
{
	struct _PATCH_HEADER *patchHdr = NULL;
	struct _Global_Descr *globalDesrc = NULL;

	if (fwbuf == NULL) {
		BTMTK_WARN("%s, fwbuf is NULL!", __func__);
		return;
	}

	patchHdr = (struct _PATCH_HEADER *)fwbuf;

	if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id))
		globalDesrc = (struct _Global_Descr *)(fwbuf + FW_ROM_PATCH_HEADER_SIZE);

	BTMTK_INFO("[btmtk] =============== Wifi Patch Info ==============");
	if (patchHdr) {
		BTMTK_INFO("[btmtk] Built Time = %s", patchHdr->ucDateTime);
		BTMTK_INFO("[btmtk] Hw Ver = 0x%04x",
			((patchHdr->u2HwVer & 0x00ff) << 8) | ((patchHdr->u2HwVer & 0xff00) >> 8));
		BTMTK_INFO("[btmtk] Sw Ver = 0x%04x",
			((patchHdr->u2SwVer & 0x00ff) << 8) | ((patchHdr->u2SwVer & 0xff00) >> 8));
		BTMTK_INFO("[btmtk] Magic Number = 0x%08x", be2cpu32(patchHdr->u4MagicNum));

		BTMTK_INFO("[btmtk] Platform = %c%c%c%c",
				patchHdr->ucPlatform[0],
				patchHdr->ucPlatform[1],
				patchHdr->ucPlatform[2],
				patchHdr->ucPlatform[3]);
	} else
		BTMTK_WARN("%s, patchHdr is NULL!", __func__);

	if (globalDesrc) {
		BTMTK_INFO("[btmtk] Patch Ver = 0x%08x",
			be2cpu32(globalDesrc->u4PatchVer));
		BTMTK_INFO("[btmtk] Section num = 0x%08x",
			be2cpu32(globalDesrc->u4SectionNum));
	} else
		BTMTK_WARN("%s, globalDesrc is NULL!", __func__);
	BTMTK_INFO("[btmtk] =========================================");
}

static int btmtk_send_wmt_download_cmd(struct btmtk_dev *bdev, u8 *cmd,
		int cmd_len, u8 *event, int event_len, struct _Section_Map *sectionMap,
		u8 fw_state, u8 dma_flag, bool patch_flag)
{
	int payload_len = 0;
	int ret = -1;
	int i = 0;
	u32 revert_SecSpec = 0;

	if (bdev == NULL || cmd == NULL || event == NULL || sectionMap == NULL) {
		BTMTK_ERR("%s: invalid parameter!", __func__);
		return ret;
	}

	/* need refine this cmd to mtk_wmt_hdr struct*/
	/* prepare HCI header */
	cmd[0] = 0x01;
	cmd[1] = 0x6F;
	cmd[2] = 0xFC;

	/* prepare WMT header */
	cmd[4] = 0x01;
	cmd[5] = 0x01; /* opcode */

	if (fw_state == 0) {
		/* prepare WMT DL cmd */
		payload_len = SEC_MAP_NEED_SEND_SIZE + 2;

		cmd[3] = (payload_len + 4) & 0xFF; /* length*/
		cmd[6] = payload_len & 0xFF;
		cmd[7] = (payload_len >> 8) & 0xFF;
		cmd[8] = 0x00; /* which is the FW download state 0 */
		cmd[9] = dma_flag; /* 1:using DMA to download, 0:using legacy wmt cmd*/
		cmd_len = SEC_MAP_NEED_SEND_SIZE + PATCH_HEADER_SIZE;

		if (patch_flag) {
			for (i = 0; i < SECTION_SPEC_NUM; i++) {
				revert_SecSpec = be2cpu32(sectionMap->u4SecSpec[i]);
				memcpy(&cmd[PATCH_HEADER_SIZE] + i * sizeof(u32), (u8 *)&revert_SecSpec, sizeof(u32));
			}
		} else
			memcpy(&cmd[PATCH_HEADER_SIZE], (u8 *)(sectionMap->u4SecSpec), SEC_MAP_NEED_SEND_SIZE);

		BTMTK_INFO_RAW(cmd, cmd_len, "%s: CMD:", __func__);

		ret = btmtk_main_send_cmd(bdev, cmd, cmd_len,
				event, event_len, DELAY_TIMES, RETRY_TIMES, BTMTK_TX_CMD_FROM_DRV);
		if (ret < 0) {
			BTMTK_ERR("%s: send wmd dl cmd failed, terminate!", __func__);
			return PATCH_ERR;
		}

		if (bdev->recv_evt_len >= event_len)
			return bdev->io_buf[PATCH_STATUS];

		return PATCH_ERR;
	}

	BTMTK_ERR("%s: fw state is error!", __func__);
	return ret;
}

static int btmtk_load_fw_patch_using_wmt_cmd(struct btmtk_dev *bdev,
		u8 *image, u8 *fwbuf, u8 *event, int event_len, u32 patch_len, int offset)
{
	int ret = 0;
	u32 cur_len = 0;
	s32 sent_len;
	int first_block = 1;
	u8 phase;
	int delay = PATCH_DOWNLOAD_PHASE1_2_DELAY_TIME;
	int retry = PATCH_DOWNLOAD_PHASE1_2_RETRY;

	if (bdev == NULL || image == NULL || fwbuf == NULL) {
		BTMTK_WARN("%s, invalid parameters!", __func__);
		ret = -1;
		goto exit;
	}

	/* loading rom patch */
	while (1) {
		s32 sent_len_max = UPLOAD_PATCH_UNIT - PATCH_HEADER_SIZE;

		sent_len = (patch_len - cur_len) >= sent_len_max ? sent_len_max : (patch_len - cur_len);

		if (sent_len > 0) {
			if (first_block == 1) {
				if (sent_len < sent_len_max)
					phase = PATCH_PHASE3;
				else
					phase = PATCH_PHASE1;
				first_block = 0;
			} else if (sent_len == sent_len_max) {
				if (patch_len - cur_len == sent_len_max)
					phase = PATCH_PHASE3;
				else
					phase = PATCH_PHASE2;
			} else {
				phase = PATCH_PHASE3;
			}


			/* prepare HCI header */
			image[0] = 0x02;
			image[1] = 0x6F;
			image[2] = 0xFC;
			image[3] = (sent_len + 5) & 0xFF;
			image[4] = ((sent_len + 5) >> 8) & 0xFF;

			/* prepare WMT header */
			image[5] = 0x01;
			image[6] = 0x01;
			image[7] = (sent_len + 1) & 0xFF;
			image[8] = ((sent_len + 1) >> 8) & 0xFF;

			image[9] = phase;
			memcpy(&image[10], fwbuf + offset + cur_len, sent_len);
			if (phase == PATCH_PHASE3) {
				delay = PATCH_DOWNLOAD_PHASE3_DELAY_TIME;
				retry = PATCH_DOWNLOAD_PHASE3_RETRY;
			}

			cur_len += sent_len;
			BTMTK_DBG("%s: sent_len = %d, cur_len = %d, phase = %d", __func__,
					sent_len, cur_len, phase);

			ret = btmtk_main_send_cmd(bdev, image, sent_len + PATCH_HEADER_SIZE,
					event, event_len, delay, retry, BTMTK_TX_ACL_FROM_DRV);
			if (ret < 0) {
				BTMTK_INFO("%s: send patch failed, terminate", __func__);
				goto exit;
			}
		} else
			break;
	}

exit:
	return ret;
}

void btmtk_send_hw_err_to_host(struct btmtk_dev *bdev)
{
	struct sk_buff *skb = NULL;
	u8 hwerr_event[HWERR_EVT_LEN] = { 0x04, 0x10, 0x01, 0xff };

	BTMTK_ERR("%s reset_stack_flag = %d!!", __func__, main_info.reset_stack_flag);
	if (main_info.reset_stack_flag) {
		skb = alloc_skb(HWERR_EVT_LEN + BT_SKB_RESERVE, GFP_ATOMIC);
		if (skb == NULL) {
			BTMTK_ERR("%s allocate skb failed!!", __func__);
		} else {
			hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
			skb->data[0] = hwerr_event[1];
			skb->data[1] = hwerr_event[2];
			skb->data[2] = main_info.reset_stack_flag;
			skb->len = HWERR_EVT_LEN - 1;
			BTMTK_DBG_RAW(skb->data, skb->len, "%s: hw err event:", __func__);
			hci_recv_frame(bdev->hdev, skb);
		}
	}
}

static int btmtk_send_fw_rom_patch_79xx(struct btmtk_dev *bdev,
		u8 *fwbuf, bool patch_flag)
{
	u8 *pos;
	int loop_count = 0;
	int ret = 0;
	u32 section_num = 0;
	u32 section_offset = 0;
	u32 dl_size = 0;
	int patch_status = 0;
	int retry = 20;
	u8 dma_flag = PATCH_DOWNLOAD_USING_WMT;
	struct _Section_Map *sectionMap;
	struct _Global_Descr *globalDescr;
	u8 event[LD_PATCH_EVT_LEN] = {0x04, 0xE4, 0x05, 0x02, 0x01, 0x01, 0x00, 0x00}; /* event[7] is status*/
#if LD_PATCH_TIME
	struct timespec64 tv_start, tv_bgf, tv_ilm;
	u32 dlt_dma = 0, dlt_all = 0;

	memset(&tv_start, 0, sizeof(tv_start));
	memset(&tv_bgf, 0, sizeof(tv_bgf));
	memset(&tv_ilm, 0, sizeof(tv_ilm));
	btmtk_do_gettimeofday(&tv_start);
#endif
	if (fwbuf == NULL) {
		BTMTK_WARN("%s, fwbuf is NULL!", __func__);
		ret = -1;
		goto exit;
	}

	globalDescr = (struct _Global_Descr *)(fwbuf + FW_ROM_PATCH_HEADER_SIZE);

	BTMTK_INFO("%s: loading rom patch...\n", __func__);

	if (patch_flag)
		section_num = be2cpu32(globalDescr->u4SectionNum);
	else
		section_num = globalDescr->u4SectionNum;
	BTMTK_INFO("%s: section_num = 0x%08x\n", __func__, section_num);

	pos = kmalloc(UPLOAD_PATCH_UNIT, GFP_ATOMIC);
	if (!pos) {
		BTMTK_ERR("%s: alloc memory failed", __func__);
		goto exit;
	}

	do {
		sectionMap = (struct _Section_Map *)(fwbuf + FW_ROM_PATCH_HEADER_SIZE +
				FW_ROM_PATCH_GD_SIZE + FW_ROM_PATCH_SEC_MAP_SIZE * loop_count);

		if (patch_flag) {
			/* wifi is big-endian */
			section_offset = be2cpu32(sectionMap->u4SecOffset);
			dl_size = be2cpu32(sectionMap->bin_info_spec.u4DLSize);
			dma_flag = be2cpu32(sectionMap->bin_info_spec.u4DLModeCrcType) & 0xFF;
		} else {
			/* BT is little-endian */
			section_offset = sectionMap->u4SecOffset;
			dl_size = sectionMap->bin_info_spec.u4DLSize;
			/*
			 * loop_count = 0: BGF patch
			 *              1: BT ILM
			 * only BT ILM support DL DMA for Buzzard
			 */
			dma_flag = le2cpu32(sectionMap->bin_info_spec.u4DLModeCrcType) & 0xFF;
		}
		BTMTK_INFO("%s: loop_count = %d, section_offset = 0x%08x, download patch_len = 0x%08x, dl mode = %d\n",
				__func__, loop_count, section_offset, dl_size, dma_flag);
		if (dl_size > 0) {
			retry = 20;
			do {
				patch_status = btmtk_send_wmt_download_cmd(bdev, pos, 0,
						event, LD_PATCH_EVT_LEN - 1, sectionMap, 0, dma_flag, patch_flag);
				BTMTK_INFO("%s: patch_status %d", __func__, patch_status);

				if (patch_status > PATCH_READY || patch_status == PATCH_ERR) {
					BTMTK_ERR("%s: patch_status error", __func__);
					ret = -1;
					goto err;
				} else if (patch_status == PATCH_READY) {
					BTMTK_INFO("%s: no need to load rom patch section%d", __func__, loop_count);
					goto next_section;
				} else if (patch_status == PATCH_IS_DOWNLOAD_BY_OTHER) {
					msleep(100);
					retry--;
				} else if (patch_status == PATCH_NEED_DOWNLOAD) {
					break;  /* Download ROM patch directly */
				}
			} while (retry > 0);

			if (patch_status == PATCH_IS_DOWNLOAD_BY_OTHER) {
				BTMTK_WARN("%s: Hold by another fun more than 2 seconds", __func__);
				ret = -1;
				goto err;
			}

			if (dma_flag == PATCH_DOWNLOAD_USING_DMA && main_info.hif_hook.dl_dma) {
				/* using DMA to download fw patch*/
				ret = main_info.hif_hook.dl_dma(bdev,
					pos, fwbuf,
					dl_size, section_offset);
				if (ret < 0) {
					BTMTK_ERR("%s: btmtk_load_fw_patch_using_dma failed!", __func__);
					goto err;
				}
			} else {
				/* using legacy wmt cmd to download fw patch */
				ret = btmtk_load_fw_patch_using_wmt_cmd(bdev, pos, fwbuf, event,
						LD_PATCH_EVT_LEN - 1, dl_size, section_offset);
				if (ret < 0) {
					BTMTK_ERR("%s: btmtk_load_fw_patch_using_wmt_cmd failed!", __func__);
					goto err;
				}
			}
		}
		/* FW Download finished */
		/* remove it, comment from fw dl owner
		 * if (patch_flag) {
		 * if (loop_count == section_num - 1) {
		 * mdelay(500);
		 * }
		 * }
		 */
#if LD_PATCH_TIME
		else {
			if (loop_count == 0) {
				btmtk_do_gettimeofday(&tv_bgf);
			} else if (loop_count == 1) {
				btmtk_do_gettimeofday(&tv_ilm);
				if (tv_bgf.tv_sec != 0 || tv_bgf.tv_nsec/1000 != 0) {
					if (tv_ilm.tv_sec >= tv_bgf.tv_sec)
						dlt_dma = (tv_ilm.tv_sec - tv_bgf.tv_sec) * 1000;
					else
						dlt_dma = (~(tv_bgf.tv_sec - tv_ilm.tv_sec) + 1) * 1000;

					dlt_dma += (tv_ilm.tv_nsec/1000 - tv_bgf.tv_nsec/1000) / 1000;
				}
				if (tv_ilm.tv_sec >= tv_start.tv_sec)
					dlt_all = (tv_ilm.tv_sec - tv_start.tv_sec) * 1000;
				else
					dlt_all = (~(tv_start.tv_sec - tv_ilm.tv_sec) + 1) * 1000;

				dlt_all += (tv_ilm.tv_nsec/1000 - tv_start.tv_nsec/1000) / 1000;

				BTMTK_INFO("LD PATCH 1 tv_start: tv_sec:%zu, tv_usec:%zu.",
					tv_start.tv_sec, tv_start.tv_nsec/1000);
				BTMTK_INFO("LD PATCH 2 tv_bgf: tv_sec:%zu, tv_usec:%zu.",
					tv_bgf.tv_sec, tv_bgf.tv_nsec/1000);
				BTMTK_INFO("LD PATCH 3 tv_ilm: tv_sec:%zu, tv_usec:%zu.",
					tv_ilm.tv_sec, tv_ilm.tv_nsec/1000);

				if (dlt_dma != 0)
					BTMTK_INFO("LD PATCH time: ILM_DMA:%ums, ALL:%ums.",
						dlt_dma, dlt_all);
				else
					BTMTK_INFO("LD PATCH time: ILM_DMA:%ums.",
						dlt_all);
			}
		}
#endif
next_section:
		continue;
	} while (++loop_count < section_num);

err:
	kfree(pos);
	pos = NULL;

exit:
	return ret;
}

int btmtk_load_rom_patch_79xx(struct btmtk_dev *bdev, bool patch_flag)
{
	int ret = 0;
	u8 *rom_patch = NULL;
	unsigned int rom_patch_len = 0;

	BTMTK_ERR("%s, patch_flag = %d!", __func__, patch_flag);


	if (!bdev) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -EINVAL;
	}

	if (patch_flag) {
		if (bdev->flavor)
			/* if flavor equals 1, it represent 7920, else it represent 7921*/
			(void)snprintf(bdev->rom_patch_bin_file_name, MAX_BIN_FILE_NAME_LEN,
					"WIFI_MT%04x_patch_mcu_1a_%x_hdr.bin",
					bdev->chip_id & 0xffff, (bdev->fw_version & 0xff) + 1);
		else
			(void)snprintf(bdev->rom_patch_bin_file_name, MAX_BIN_FILE_NAME_LEN,
					"WIFI_MT%04x_patch_mcu_1_%x_hdr.bin",
					bdev->chip_id & 0xffff, (bdev->fw_version & 0xff) + 1);
	}

	btmtk_load_code_from_bin(&rom_patch, bdev->rom_patch_bin_file_name, NULL,
							&rom_patch_len, 10);

	if (!rom_patch) {
		BTMTK_ERR("%s: please assign a rom patch(/etc/firmware/%s)or(/lib/firmware/%s)",
			__func__, bdev->rom_patch_bin_file_name, bdev->rom_patch_bin_file_name);
		ret = -1;
		goto err;
	}

	if (patch_flag)
		/*Display rom patch info*/
		btmtk_print_wifi_patch_info(bdev, rom_patch);
	else
		btmtk_print_bt_patch_info(bdev, rom_patch);

	ret = btmtk_send_fw_rom_patch_79xx(bdev, rom_patch, patch_flag);
	if (ret < 0) {
		BTMTK_ERR("%s, btmtk_send_fw_rom_patch_79xx failed!", __func__);
		goto err;
	}

	bdev->power_state = BTMTK_DONGLE_STATE_POWER_OFF;
	BTMTK_INFO("btmtk_load_rom_patch_79xx end");

err:
	if (rom_patch)
		vfree(rom_patch);
	return ret;
}

int btmtk_load_rom_patch_766x(struct btmtk_dev *bdev)
{
	u32 patch_len = 0;
	int ret = 0;
	int patch_status = 0;
	int retry = 20;
	u8 *pos = NULL;
	u8 event[LD_PATCH_EVT_LEN] = {0x04, 0xE4, 0x05, 0x02, 0x01, 0x01, 0x00, 0x00};
	u8 *rom_patch = NULL;
	unsigned int rom_patch_len = 0;
	struct _PATCH_HEADER *patchHdr;

	if (!bdev) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -EINVAL;
	}

	btmtk_load_code_from_bin(&rom_patch, bdev->rom_patch_bin_file_name, NULL,
							&rom_patch_len, 10);

	do {
		patch_status = btmtk_check_need_load_rom_patch(bdev);
		BTMTK_INFO("%s: patch_status %d", __func__, patch_status);

		if (patch_status > MT766X_PATCH_NEED_DOWNLOAD || patch_status == PATCH_ERR) {
			BTMTK_ERR("%s: patch_status error", __func__);
			ret = -1;
			goto err1;
		} else if (patch_status == MT766X_PATCH_READY) {
			BTMTK_INFO("%s: no need to load rom patch", __func__);
			goto patch_end;
		} else if (patch_status == MT766X_PATCH_IS_DOWNLOAD_BY_OTHER) {
			msleep(100);
			retry--;
		} else if (patch_status == MT766X_PATCH_NEED_DOWNLOAD) {
/* TODO*/
#if 0
			if (is_mt7663(g_card)) {
				if (btmtk_sdio_send_wmt_cfg())
					BTMTK_ERR("send wmt cfg failed!");
			}
#endif
			break;  /* Download ROM patch directly */
		}
	} while (retry > 0);

	if (patch_status == PATCH_IS_DOWNLOAD_BY_OTHER) {
		BTMTK_WARN("%s: Hold by another fun more than 2 seconds", __func__);
		ret = -1;
		goto err1;
	}

	patchHdr = (struct _PATCH_HEADER *)rom_patch;
	/*Display rom patch info*/
	btmtk_print_bt_patch_info(bdev, rom_patch);

	pos = kmalloc(UPLOAD_PATCH_UNIT, GFP_ATOMIC);
	if (!pos) {
		BTMTK_ERR("%s: alloc memory failed", __func__);
		ret = -1;
		goto err1;
	}

	patch_len = rom_patch_len - PATCH_INFO_SIZE;

	BTMTK_INFO("%s: loading rom patch...\n", __func__);
	BTMTK_INFO("%s: patch_len = %d\n", __func__, patch_len);
	ret = btmtk_load_fw_patch_using_wmt_cmd(bdev, pos, rom_patch, event,
			LD_PATCH_EVT_LEN - 1, patch_len, PATCH_INFO_SIZE);
	if (ret < 0) {
		BTMTK_ERR("%s, btmtk_send_fw_rom_patch_766x failed!", __func__);
		goto err0;
	}

	ret = btmtk_send_wmt_reset(bdev);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_send_wmt_reset failed!", __func__);
		goto err0;
	}
	BTMTK_INFO("%s: loading rom patch... Done", __func__);

	btmtk_send_hw_err_to_host(bdev);

patch_end:
	bdev->power_state = BTMTK_DONGLE_STATE_POWER_OFF;
	BTMTK_INFO("btmtk_load_rom_patch end");

err0:
	kfree(pos);
	pos = NULL;

err1:
	if (rom_patch)
		vfree(rom_patch);
	return ret;
}

/* need to remove after modify to using function pointer*/
__weak int32_t bgfsys_bt_patch_dl(void)
{
	BTMTK_ERR("No bgfsys_bt_patch_dl function");
	return -1;
}

/* need to remove after modify to using function pointer*/
__weak int32_t btmtk_set_sleep(struct hci_dev *hdev, u_int8_t need_wait)
{
	BTMTK_ERR("No btmtk_set_sleep function");
	return -1;
}

int btmtk_load_rom_patch_66xx(struct btmtk_dev *bdev)
{
	return bgfsys_bt_patch_dl();
}

int btmtk_load_rom_patch(struct btmtk_dev *bdev)
{
	int err = -1;

	if (!bdev || !bdev->hdev) {
		BTMTK_ERR("%s: invalid parameters!", __func__);
		return err;
	}

	if (is_mt7663(bdev->chip_id))
		err = btmtk_load_rom_patch_766x(bdev);
	else if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id)) {
		err = btmtk_load_rom_patch_79xx(bdev, BT_DOWNLOAD);
		if (err < 0) {
			BTMTK_ERR("%s: btmtk_load_rom_patch_79xx bt patch failed!", __func__);
			return err;
		}

#if CFG_SUPPORT_BT_DL_WIFI_PATCH
		err = btmtk_load_rom_patch_79xx(bdev, WIFI_DOWNLOAD);
		if (err < 0) {
			BTMTK_WARN("%s: btmtk_load_rom_patch_79xx wifi patch failed!", __func__);
			err = 0;
		}
#endif
	} else if (is_mt66xx(bdev->chip_id)) {
		err = btmtk_load_rom_patch_66xx(bdev);
	} else
		BTMTK_WARN("%s: unknown chip id (%d)", __func__, bdev->chip_id);

	return err;
}

struct btmtk_dev *btmtk_get_dev(void)
{
	int i = 0;
	struct btmtk_dev *tmp_bdev = NULL;

	BTMTK_INFO("%s", __func__);

	for (i = 0; i < btmtk_intf_num; i++) {
		/* Find empty slot for newly probe interface.
		 * Judged from load_rom_patch is done and
		 * Identified chip_id from cap_init.
		 */
		if (g_bdev[i]->hdev == NULL) {
			if (i == 0)
				g_bdev[i]->dongle_index = i;
			else
				g_bdev[i]->dongle_index = g_bdev[i - 1]->dongle_index + 1;

			/* reset pin initial value need to be -1, used to judge after
			 * disconnected before probe, can't do chip reset
			 */
			g_bdev[i]->bt_cfg.dongle_reset_gpio_pin = -1;
			tmp_bdev = g_bdev[i];

			/* Hook pre-defined table on state machine */
			g_bdev[i]->cif_state = (struct btmtk_cif_state *)g_cif_state;
			break;
		}
	}

	return tmp_bdev;
}

void btmtk_release_dev(struct btmtk_dev *bdev)
{
	int i = 0;
	struct btmtk_dev *tmp_bdev = NULL;

	BTMTK_INFO("%s", __func__);

	tmp_bdev = bdev;
	if (tmp_bdev != NULL) {
		for (i = 0; i < btmtk_intf_num; i++) {
			/* Find slot on probed interface.
			 * Judged from load_rom_patch is done and
			 * Identified chip_id from cap_init.
			 */
			if (memcmp(tmp_bdev, g_bdev[i], sizeof(*tmp_bdev)) == 0) {
				memset(tmp_bdev, 0, sizeof(*tmp_bdev));
				/* reset pin initial value need to be -1, used to judge after
				 * disconnected before probe, can't do chip reset
				 */
				bdev->bt_cfg.dongle_reset_gpio_pin = -1;

				tmp_bdev = NULL;
				break;
			}
		}
	}

}

struct btmtk_dev *btmtk_allocate_dev_memory(struct device *dev)
{
	struct btmtk_dev *bdev = NULL;
	size_t len = sizeof(*bdev);

	BTMTK_INFO("%s", __func__);

	if (dev != NULL)
		bdev = devm_kzalloc(dev, len, GFP_KERNEL);
	else
		bdev = kzalloc(len, GFP_KERNEL);

	if (!bdev)
		return NULL;

	btmtk_set_chip_state(bdev, BTMTK_STATE_INIT);

	return bdev;
}

void btmtk_free_dev_memory(struct device *dev, struct btmtk_dev *bdev)
{
	BTMTK_INFO("%s", __func__);

	if (bdev != NULL) {
		if (dev != NULL)
			devm_kfree(dev, bdev);
		else
			kfree(bdev);
	}
}

static int btmtk_calibration_flow(struct btmtk_dev *bdev)
{
	if (!bdev) {
		BTMTK_ERR("%s: bdev is NULL !", __func__);
		return -1;
	}

	btmtk_cif_send_calibration(bdev);
	BTMTK_INFO("%s done", __func__);
	return 0;
}

static int btmtk_send_hci_reset_cmd(struct btmtk_dev *bdev)
{
	u8 cmd[HCI_RESET_CMD_LEN] = { 0x01, 0x03, 0x0C, 0x00 };
	u8 event[HCI_RESET_EVT_LEN] = { 0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00 };
	int ret = -1;	/* if successful, 0 */

	if (!bdev) {
		BTMTK_ERR("%s: bdev is NULL !", __func__);
		return ret;
	}

	ret = btmtk_main_send_cmd(bdev,
			cmd, HCI_RESET_CMD_LEN,
			event, HCI_RESET_EVT_LEN,
			0, 0, BTMTK_TX_PKT_FROM_HOST);

	BTMTK_INFO("%s done", __func__);
	return ret;
}

int btmtk_send_wmt_reset(struct btmtk_dev *bdev)
{
	/* Support 7668 and 7663 */
	u8 cmd[WMT_RESET_CMD_LEN] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x07, 0x01, 0x00, 0x04 };
	/* To-Do, for event check */
	u8 event[WMT_RESET_EVT_LEN] = { 0x04, 0xE4, 0x05, 0x02, 0x07, 0x01, 0x00, 0x00 };
	int ret = -1;

	if (!bdev) {
		BTMTK_ERR("%s: bdev is NULL !", __func__);
		return ret;
	}
	if (bdev->power_state == BTMTK_DONGLE_STATE_POWER_OFF) {
		ret = btmtk_main_send_cmd(bdev,
			cmd, WMT_RESET_CMD_LEN,
			event, WMT_RESET_EVT_LEN,
			DELAY_TIMES, 0, BTMTK_TX_CMD_FROM_DRV);

		if (ret >= 0)
			bdev->power_state = BTMTK_DONGLE_STATE_POWER_ON;
		else
			bdev->power_state = BTMTK_DONGLE_STATE_ERROR;
	}

	if (bdev->power_state != BTMTK_DONGLE_STATE_POWER_ON) {
		BTMTK_WARN("%s: end of Incorrect state:%d", __func__, bdev->power_state);
		return -EBADFD;
	}

	BTMTK_INFO("%s done", __func__);
	return ret;
}

int btmtk_send_wmt_power_on_cmd(struct btmtk_dev *bdev)
{
	/* Support 7668 and 7663 and 7961 */
	u8 cmd[WMT_POWER_ON_CMD_LEN] = { 0x01, 0x6F, 0xFC, 0x06, 0x01, 0x06, 0x02, 0x00, 0x00, 0x01 };
	u8 event[WMT_POWER_ON_EVT_HDR_LEN] = { 0x04, 0xE4, 0x05, 0x02, 0x06, 0x01, 0x00 };	/* event[7] is key */
	int ret = -1, retry = RETRY_TIMES;

	if (!bdev) {
		BTMTK_ERR("%s: bdev is NULL !", __func__);
		return ret;
	}

retry_again:

	ret = btmtk_main_send_cmd(bdev,
			cmd, WMT_POWER_ON_CMD_LEN,
			event, WMT_POWER_ON_EVT_HDR_LEN,
			WMT_DELAY_TIMES, RETRY_TIMES, BTMTK_TX_CMD_FROM_DRV);
	if (ret < 0) {
		BTMTK_ERR("%s: failed(%d)", __func__, ret);
		bdev->power_state = BTMTK_DONGLE_STATE_ERROR;
		ret = -1;
	} else if (ret == 0 && bdev->recv_evt_len > 0) {
		switch (bdev->io_buf[WMT_POWER_ON_EVT_RESULT_OFFSET]) {
		case 0:			 /* successful */
			BTMTK_INFO("%s: OK", __func__);
			bdev->power_state = BTMTK_DONGLE_STATE_POWER_ON;
			break;
		case 2:			 /* TODO:retry */
			if (retry > 0) {
				/* comment from fw, we need to retry a sec until power on sucessfully. */
				retry--;
				BTMTK_INFO("%s: need to try again", __func__);
				mdelay(50);
				goto retry_again;
			}
			break;
		default:
			BTMTK_WARN("%s: Unknown result: %02X", __func__, bdev->io_buf[WMT_POWER_ON_EVT_RESULT_OFFSET]);
			bdev->power_state = BTMTK_DONGLE_STATE_ERROR;
			ret = -1;
			break;
		}
	}

	return ret;
}

int btmtk_send_wmt_power_off_cmd(struct btmtk_dev *bdev)
{
	/* Support 7668 and 7663 and 7961 */
	u8 cmd[WMT_POWER_OFF_CMD_LEN] = { 0x01, 0x6F, 0xFC, 0x06, 0x01, 0x06, 0x02, 0x00, 0x00, 0x00 };
	/* To-Do, for event check */
	u8 event[WMT_POWER_OFF_EVT_HDR_LEN] = { 0x04, 0xE4, 0x05, 0x02, 0x06, 0x01, 0x00 };
	int ret = -1;

	if (!bdev) {
		BTMTK_ERR("%s: bdev is NULL !", __func__);
		return ret;
	}

	if (bdev->power_state == BTMTK_DONGLE_STATE_POWER_OFF) {
		BTMTK_WARN("%s: power_state already power off", __func__);
		return 0;
	}

	ret = btmtk_main_send_cmd(bdev,
			cmd, WMT_POWER_OFF_CMD_LEN,
			event, WMT_POWER_OFF_EVT_HDR_LEN,
			DELAY_TIMES, RETRY_TIMES, BTMTK_TX_CMD_FROM_DRV);
	if (ret < 0) {
		BTMTK_ERR("%s: failed(%d)", __func__, ret);
		bdev->power_state = BTMTK_DONGLE_STATE_ERROR;
		return ret;
	}

	bdev->power_state = BTMTK_DONGLE_STATE_POWER_OFF;
	BTMTK_INFO("%s done", __func__);
	return ret;
}

/* Check power status, if power is off, try to set power on */
int btmtk_reset_power_on(struct btmtk_dev *bdev)
{
	if (bdev->power_state == BTMTK_DONGLE_STATE_POWER_OFF) {
		bdev->power_state = BTMTK_DONGLE_STATE_ERROR;
		if (btmtk_send_wmt_power_on_cmd(bdev) < 0)
			return -1;
		if (is_mt7663(bdev->chip_id)) {
			if (btmtk_send_hci_tci_set_sleep_cmd_766x(bdev) < 0)
				return -1;

			if (btmtk_send_hci_reset_cmd(bdev) < 0)
			return -1;
		}

		bdev->power_state = BTMTK_DONGLE_STATE_POWER_ON;
	}

	if (bdev->power_state != BTMTK_DONGLE_STATE_POWER_ON) {
		BTMTK_WARN("%s: end of Incorrect state:%d", __func__, bdev->power_state);
		return -1;
	}
	BTMTK_INFO("%s: end success", __func__);

	return 0;
}

int btmtk_picus_enable(struct btmtk_dev *bdev)
{
	u8 dft_enable_cmd[PICUS_ENABLE_CMD_LEN] = { 0x01, 0x5D, 0xFC, 0x04, 0x00, 0x00, 0x02, 0x02 };
	u8 *enable_cmd = NULL;
	u8 enable_event[PICUS_ENABLE_EVT_HDR_LEN] = { 0x04, 0x0E, 0x08, 0x01, 0x5D, 0xFC, 0x00, 0x00, 0x00 };
	int enable_len = 0;
	int ret = -1;	/* if successful, 0 */

	struct fw_cfg_struct *picus_setting = &bdev->bt_cfg.picus_enable;

	BTMTK_INFO("%s", __func__);

	if (picus_setting->content && picus_setting->length) {
		BTMTK_INFO("%s load picus from bt.cfg", __func__);
		enable_cmd = picus_setting->content;
		enable_len = picus_setting->length;
	} else {
		enable_cmd = dft_enable_cmd;
		enable_len = PICUS_ENABLE_CMD_LEN;
	}
	BTMTK_INFO_RAW(enable_cmd, enable_len, "%s: Send CMD:", __func__);

	if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id))
		ret = btmtk_main_send_cmd(bdev,
			enable_cmd, enable_len,
			enable_event, PICUS_ENABLE_EVT_HDR_LEN,
			DELAY_TIMES, RETRY_TIMES,
			BTMTK_TX_PKT_FROM_HOST);
	else
		BTMTK_WARN("%s: not support for 0x%x", __func__, bdev->chip_id);

	BTMTK_INFO("%s: ret %d", __func__, ret);
	return ret;
}

int btmtk_picus_disable(struct btmtk_dev *bdev)
{
	u8 dft_disable_cmd[PICUS_DISABLE_CMD_LEN] = { 0x01, 0x5D, 0xFC, 0x04, 0x00, 0x00, 0x02, 0x00 };
	u8 dft_disable_event[PICUS_DISABLE_EVT_HDR_LEN] = { 0x04, 0x0E, 0x08, 0x01, 0x5D, 0xFC, 0x00, 0x00, 0x00 };
	int ret = -1;	/* if successful, 0 */

	BTMTK_INFO("%s\n", __func__);

	if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id))
		ret = btmtk_main_send_cmd(bdev,
			dft_disable_cmd, PICUS_DISABLE_CMD_LEN,
			dft_disable_event, PICUS_DISABLE_EVT_HDR_LEN,
			DELAY_TIMES, RETRY_TIMES,
			BTMTK_TX_PKT_FROM_HOST);
	else
		BTMTK_WARN("%s: not support for 0x%x", __func__, bdev->chip_id);

	BTMTK_INFO("%s: ret %d", __func__, ret);
	return ret;
}

int btmtk_load_fw_cfg_setting(char *block_name, struct fw_cfg_struct *save_content,
		int counter, u8 *searchcontent, enum fw_cfg_index_len index_length)
{
	int ret = 0, i = 0;
	u16 temp_len = 0;
	u8 temp[TEMP_LEN]; /* save for total hex number */
	unsigned long parsing_result = 0;
	char *search_result = NULL;
	char *search_end = NULL;
	char search[SEARCH_LEN];
	char *next_block = NULL;
	char number[CHAR2HEX_SIZE + 1];	/* 1 is for '\0' */

	memset(search, 0, SEARCH_LEN);
	memset(temp, 0, TEMP_LEN);
	memset(number, 0, CHAR2HEX_SIZE + 1);

	if (searchcontent == NULL) {
		BTMTK_ERR("%s: Searchcontent is NULL", __func__);
		return -1;
	}

	/* search block name */
	for (i = 0; i < counter; i++) {
		temp_len = 0;
		if (index_length == FW_CFG_INX_LEN_2) /* EX: APCF01 */
			(void)snprintf(search, SEARCH_LEN, "%s%02d:", block_name, i);
		else if (index_length == FW_CFG_INX_LEN_3) /* EX: APCF001 */
			(void)snprintf(search, SEARCH_LEN, "%s%03d:", block_name, i);
		else
			(void)snprintf(search, SEARCH_LEN, "%s:", block_name);

		ret = 0;

		search_result = strstr((char *)searchcontent, search);
		if (search_result) {
			memset(temp, 0, TEMP_LEN);
			search_result = strstr(search_result, "0x");
			/* find next line as end of this command line, if NULL means last line */
			if (search_result == NULL) {
				BTMTK_ERR("%s: Search_result is NULL", __func__);
				continue;
			}
			next_block = strstr(search_result, ":");

			/* Add HCI packet type to front of each command/event */
			if (!memcmp(block_name, "APCF", sizeof("APCF")) ||
				!memcmp(block_name, "RADIOOFF", sizeof("RADIOOFF")) ||
				!memcmp(block_name, "RADIOON", sizeof("RADIOON")) ||
				!memcmp(block_name, "APCF_RESUME", sizeof("APCF_RESUME")) ||
				!memcmp(block_name, "VENDOR_CMD", sizeof("VENDOR_CMD")) ||
				!memcmp(block_name, "PHASE1_WMT_CMD", sizeof("PHASE1_WMT_CMD"))) {
				temp[0] = 0x01;
				temp_len++;
			} else if (!memcmp(block_name, "RADIOOFF_STATUS_EVENT", sizeof("RADIOOFF_STATUS_EVENT")) ||
				!memcmp(block_name, "RADIOOFF_COMPLETE_EVENT", sizeof("RADIOOFF_COMPLETE_EVENT")) ||
				!memcmp(block_name, "RADIOON_STATUS_EVENT", sizeof("RADIOON_STATUS_EVENT")) ||
				!memcmp(block_name, "RADIOON_COMPLETE_EVENT", sizeof("RADIOON_COMPLETE_EVENT"))) {
				temp[0] = 0x04;
				temp_len++;
			}

			do {
				if (search_result == NULL) {
					BTMTK_ERR("%s: search_result is NULL", __func__);
					break;
				}
				search_end = strstr(search_result, ",");
				if (search_end == NULL) {
					BTMTK_ERR("%s: Search_end is NULL", __func__);
					break;
				}

				if (search_end - search_result != CHAR2HEX_SIZE) {
					BTMTK_ERR("%s: Incorrect Format in %s", __func__, search);
					break;
				}

				memset(number, 0, CHAR2HEX_SIZE + 1);
				memcpy(number, search_result, CHAR2HEX_SIZE);
				ret = kstrtoul(number, 0, &parsing_result);
				if (ret == 0) {
					if (temp_len >= TEMP_LEN) {
						BTMTK_ERR("%s: %s data over %d", __func__, search, TEMP_LEN);
						break;
					}
					temp[temp_len] = parsing_result;
					temp_len++;
				} else {
					BTMTK_WARN("%s: %s kstrtoul fail: %d", __func__, search, ret);
					break;
				}
				search_result = strstr(search_end, "0x");
			} while (search_result < next_block || (search_result && next_block == NULL));
		} else
			BTMTK_DBG("%s: %s is not found in %d", __func__, search, i);

		if (temp_len && temp_len < TEMP_LEN) {
			BTMTK_INFO("%s: %s found & stored in %d", __func__, search, i);
			save_content[i].content = kzalloc(temp_len, GFP_KERNEL);
			if (save_content[i].content == NULL) {
				BTMTK_ERR("%s: Allocate memory fail(%d)", __func__, i);
				return -ENOMEM;
			}
			memcpy(save_content[i].content, temp, temp_len);
			save_content[i].length = temp_len;
			BTMTK_DBG_RAW(save_content[i].content, save_content[i].length, "%s", search);
		}
	}

	return ret;
}

int btmtk_load_code_from_setting_files(char *setting_file_name,
			struct device *dev, u32 *code_len, struct btmtk_dev *bdev)
{
	int err = 0;
	const struct firmware *fw_entry = NULL;

	*code_len = 0;

	if (bdev == NULL) {
		BTMTK_ERR("%s: g_data is NULL!!", __func__);
		err = -1;
		goto end;
	}

	BTMTK_INFO("%s: begin setting_file_name = %s", __func__, setting_file_name);
	err = request_firmware(&fw_entry, setting_file_name, dev);
	if (err != 0 || fw_entry == NULL) {
		BTMTK_INFO("%s: request_firmware fail, maybe file not exist, err = %d, fw_entry = %p",
				__func__, err, fw_entry);
		if (fw_entry)
			release_firmware(fw_entry);
		goto end;
	}

	BTMTK_INFO("%s: setting file request_firmware size %zu success", __func__, fw_entry->size);
	if (bdev->setting_file != NULL) {
		kfree(bdev->setting_file);
		bdev->setting_file = NULL;
	}
	bdev->setting_file = kzalloc(fw_entry->size + 1, GFP_KERNEL); /* alloc setting file memory */
	if (bdev->setting_file == NULL) {
		BTMTK_ERR("%s: kzalloc size %zu failed!!", __func__, fw_entry->size);
		release_firmware(fw_entry);
		err = -1;
		goto end;
	}

	memcpy(bdev->setting_file, fw_entry->data, fw_entry->size);
	bdev->setting_file[fw_entry->size] = '\0';

	*code_len = fw_entry->size;
	release_firmware(fw_entry);

	BTMTK_INFO("%s: setting_file len (%d) assign done", __func__, *code_len);
end:
	return err;
}

static bool btmtk_parse_bt_cfg_file(char *item_name,
		char *text, u8 *searchcontent)
{
	bool ret = true;
	int temp_len = 0;
	char search[SEARCH_LEN];
	char *ptr = NULL, *p = NULL;
	char *temp = text;

	if (text == NULL) {
		BTMTK_ERR("%s: text param is invalid!", __func__);
		ret = false;
		goto out;
	}

	memset(search, 0, SEARCH_LEN);
	(void)snprintf(search, SEARCH_LEN, "%s", item_name); /* EX: SUPPORT_UNIFY_WOBLE */
	p = ptr = strstr((char *)searchcontent, search);

	if (!ptr) {
		BTMTK_ERR("%s: Can't find %s\n", __func__, item_name);
		ret = false;
		goto out;
	}

	if (p > (char *)searchcontent) {
		p--;
		while ((*p == ' ') && (p != (char *)searchcontent))
			p--;
		if (*p == '#') {
			BTMTK_ERR("%s: It's invalid bt cfg item\n", __func__);
			ret = false;
			goto out;
		}
	}

	p = ptr + strlen(item_name) + 1;
	ptr = p;

	for (;;) {
		switch (*p) {
		case '\n':
			goto textdone;
		default:
			*temp++ = *p++;
			break;
		}
	}

textdone:
	temp_len = p - ptr;
	*temp = '\0';

out:
	return ret;
}

static void btmtk_bt_cfg_item_value_to_bool(char *item_value, bool *value)
{
	unsigned long text_value = 0;

	if (item_value == NULL) {
		BTMTK_ERR("%s: item_value is NULL!", __func__);
		return;
	}

	if (kstrtoul(item_value, 10, &text_value) == 0) {
		if (text_value == 1)
			*value = true;
		else
			*value = false;
	} else {
		BTMTK_WARN("%s: kstrtoul failed!", __func__);
	}
}

static void btmtk_load_bt_cfg_item(struct bt_cfg_struct *bt_cfg_content,
		u8 *searchcontent, struct btmtk_dev *bdev)
{
	bool ret = true;
	char text[TEXT_LEN]; /* save for search text */
	unsigned long text_value = 0;

	memset(text, 0, TEXT_LEN);
	ret = btmtk_parse_bt_cfg_file(BT_UNIFY_WOBLE, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_unify_woble);
		BTMTK_INFO("%s: bt_cfg_content->support_unify_woble = %d", __func__,
				bt_cfg_content->support_unify_woble);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_UNIFY_WOBLE);
	}

	ret = btmtk_parse_bt_cfg_file(BT_UNIFY_WOBLE_TYPE, text, searchcontent);
	if (ret) {
		if (kstrtoul(text, 10, &text_value) == 0)
			bt_cfg_content->unify_woble_type = text_value;
		else
			BTMTK_WARN("%s: kstrtoul failed %s!", __func__, BT_UNIFY_WOBLE_TYPE);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_UNIFY_WOBLE_TYPE);
	}

	BTMTK_INFO("%s: bt_cfg_content->unify_woble_type = %d", __func__,
			bt_cfg_content->unify_woble_type);

	ret = btmtk_parse_bt_cfg_file(BT_WOBLE_BY_EINT, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_woble_by_eint);
		BTMTK_INFO("%s: bt_cfg_content->support_woble_by_eint = %d", __func__,
					bt_cfg_content->support_woble_by_eint);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_WOBLE_BY_EINT);
	}

	ret = btmtk_parse_bt_cfg_file(BT_DONGLE_RESET_PIN, text, searchcontent);
	if (ret) {
		if (kstrtoul(text, 10, &text_value) == 0)
			bt_cfg_content->dongle_reset_gpio_pin = text_value;
		else
			BTMTK_WARN("%s: kstrtoul failed %s!", __func__, BT_DONGLE_RESET_PIN);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_DONGLE_RESET_PIN);
	}

	BTMTK_INFO("%s: bt_cfg_content->dongle_reset_gpio_pin = %d", __func__,
			bt_cfg_content->dongle_reset_gpio_pin);

	ret = btmtk_parse_bt_cfg_file(BT_RESET_DONGLE, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_dongle_reset);
		BTMTK_INFO("%s: bt_cfg_content->support_dongle_reset = %d", __func__,
				bt_cfg_content->support_dongle_reset);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_RESET_DONGLE);
	}

	ret = btmtk_parse_bt_cfg_file(BT_FULL_FW_DUMP, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_full_fw_dump);
		BTMTK_INFO("%s: bt_cfg_content->support_full_fw_dump = %d", __func__,
				bt_cfg_content->support_full_fw_dump);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_FULL_FW_DUMP);
	}

	ret = btmtk_parse_bt_cfg_file(BT_WOBLE_WAKELOCK, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_woble_wakelock);
		BTMTK_INFO("%s: bt_cfg_content->support_woble_wakelock = %d", __func__,
				bt_cfg_content->support_woble_wakelock);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_WOBLE_WAKELOCK);
	}

	ret = btmtk_parse_bt_cfg_file(BT_WOBLE_FOR_BT_DISABLE, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_woble_for_bt_disable);
		BTMTK_INFO("%s: bt_cfg_content->support_woble_for_bt_disable = %d", __func__,
				bt_cfg_content->support_woble_for_bt_disable);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_WOBLE_FOR_BT_DISABLE);
	}

	ret = btmtk_parse_bt_cfg_file(BT_RESET_STACK_AFTER_WOBLE, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->reset_stack_after_woble);
		BTMTK_INFO("%s: bt_cfg_content->reset_stack_after_woble = %d", __func__,
				bt_cfg_content->reset_stack_after_woble);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_RESET_STACK_AFTER_WOBLE);
	}

	ret = btmtk_parse_bt_cfg_file(BT_AUTO_PICUS, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_auto_picus);
		BTMTK_INFO("%s: bt_cfg_content->support_auto_picus = %d", __func__,
				bt_cfg_content->support_auto_picus);
		if (bt_cfg_content->support_auto_picus == true) {
			ret = btmtk_load_fw_cfg_setting(BT_AUTO_PICUS_FILTER,
					&bt_cfg_content->picus_filter, 1, searchcontent, FW_CFG_INX_LEN_NONE);
			if (ret)
				BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_AUTO_PICUS_FILTER);

			ret = btmtk_load_fw_cfg_setting(BT_AUTO_PICUS_ENABLE,
					&bt_cfg_content->picus_enable, 1, searchcontent, FW_CFG_INX_LEN_NONE);
			if (ret)
				BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_AUTO_PICUS_ENABLE);
		}
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_AUTO_PICUS);
	}

	ret = btmtk_parse_bt_cfg_file(BT_PICUS_TO_HOST, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_picus_to_host);
		BTMTK_INFO("%s: bt_cfg_content->support_picus_to_host = %d", __func__,
				bt_cfg_content->support_picus_to_host);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_PICUS_TO_HOST);
	}

	ret = btmtk_parse_bt_cfg_file(BT_SINGLE_SKU, text, searchcontent);
	if (ret) {
		btmtk_bt_cfg_item_value_to_bool(text, &bt_cfg_content->support_bt_single_sku);
		BTMTK_INFO("%s: bt_cfg_content->support_bt_single_sku = %d", __func__,
				bt_cfg_content->support_bt_single_sku);
	} else {
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_SINGLE_SKU);
	}

	ret = btmtk_load_fw_cfg_setting(BT_PHASE1_WMT_CMD, bt_cfg_content->phase1_wmt_cmd,
				PHASE1_WMT_CMD_COUNT, searchcontent, FW_CFG_INX_LEN_3);
	if (ret)
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_PHASE1_WMT_CMD);

	ret = btmtk_load_fw_cfg_setting(BT_VENDOR_CMD, bt_cfg_content->vendor_cmd,
				VENDOR_CMD_COUNT, searchcontent, FW_CFG_INX_LEN_3);
	if (ret)
		BTMTK_WARN("%s: search item %s is invalid!", __func__, BT_VENDOR_CMD);

	/* release setting file memory */
	if (bdev) {
		kfree(bdev->setting_file);
		bdev->setting_file = NULL;
	}
}

static void btmtk_load_bt_cfg(char *cfg_name, struct device *dev, struct btmtk_dev *bdev)
{
	u32 code_len = 0;

	if (btmtk_load_code_from_setting_files(cfg_name, dev, &code_len, bdev))
		BTMTK_ERR("btmtk_usb_load_code_from_setting_files failed!!");
	else
		btmtk_load_bt_cfg_item(&bdev->bt_cfg, bdev->setting_file, bdev);
}

#if ENABLESTP
static int btmtk_send_set_stp_cmd(struct btmtk_dev *bdev)
{
	u8 cmd[SET_STP_CMD_LEN] = { 0x01, 0x6F, 0xFC, 0x09, 0x01, 0x04, 0x05, 0x00, 0x03, 0x11, 0x0E, 0x00, 0x00};
	u8 event[SET_STP_EVT_LEN] = { 0x04, 0xE4, 0x06, 0x02, 0x04, 0x02, 0x00, 0x00, 0x03};
	int ret = 0;

	ret = btmtk_main_send_cmd(bdev,
			cmd, SET_STP_CMD_LEN,
			event, SET_STP_EVT_LEN,
			0, 0, BTMTK_TX_CMD_FROM_DRV);

	BTMTK_INFO("%s done", __func__);
	return ret;
}

static int btmtk_send_set_stp1_cmd(struct btmtk_dev *bdev)
{
	u8 cmd[SET_STP1_CMD_LEN] = {0x01, 0x6F, 0xFC, 0x0C,
		0x01, 0x08, 0x08, 0x00, 0x02, 0x01, 0x00, 0x01, 0x08, 0x00, 0x00, 0x80};
	u8 event[SET_STP1_EVT_LEN] = {0x04, 0xE4, 0x10, 0x02, 0x08,
		0x0C, 0x00, 0x00, 0x00, 0x00, 0x01, 0x08, 0x00, 0x00, 0x80, 0x63, 0x76, 0x00, 0x00};
	int ret = 0;

	ret = btmtk_main_send_cmd(bdev,
			cmd, SET_STP1_CMD_LEN,
			event, SET_STP1_EVT_LEN, 0, 0,
			BTMTK_TX_CMD_FROM_DRV);

	BTMTK_INFO("%s done", __func__);
	return ret;
}
#endif

static int btmtk_send_hci_tci_set_sleep_cmd_766x(struct btmtk_dev *bdev)
{
	u8 cmd[SET_SLEEP_CMD_LEN] = { 0x01, 0x7A, 0xFC, 0x07, 0x05, 0x40, 0x06, 0x40, 0x06, 0x00, 0x00 };
	u8 event[SET_SLEEP_EVT_LEN] = { 0x04, 0x0E, 0x04, 0x01, 0x7A, 0xFC, 0x00 };
	int ret = -1;

	if (!bdev) {
		BTMTK_ERR("%s: bdev is NULL !", __func__);
		return ret;
	}

	ret = btmtk_main_send_cmd(bdev,
		cmd, SET_SLEEP_CMD_LEN,
		event, SET_SLEEP_EVT_LEN,
		0, 0, BTMTK_TX_PKT_FROM_HOST);

	BTMTK_INFO("%s done", __func__);

	return ret;
}

int btmtk_cap_init(struct btmtk_dev *bdev)
{
	if (!bdev) {
		BTMTK_ERR("%s, bdev is NULL!", __func__);
		return -1;
	}
	/* Todo read wifi fw version
	 * int wifi_fw_ver;

	 * btmtk_cif_write_register(bdev, 0x7C4001C4, 0x00008800);
	 * btmtk_cif_read_register(bdev, 0x7c4f0004, &wifi_fw_ver);
	 * BTMTK_ERR("wifi fw_ver = %04X", wifi_fw_ver);
	 */

	main_info.hif_hook.reg_read(bdev, CHIP_ID, &bdev->chip_id);
	if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id)) {
		main_info.hif_hook.reg_read(bdev, FLAVOR, &bdev->flavor);
		main_info.hif_hook.reg_read(bdev, FW_VERSION, &bdev->fw_version);
	} else {
		BTMTK_ERR("Unknown Mediatek device(%04X)\n", bdev->chip_id);
		return -1;
	}

	BTMTK_INFO("%s: Chip ID = 0x%x", __func__, bdev->chip_id);
	BTMTK_INFO("%s: flavor = 0x%x", __func__, bdev->flavor);
	BTMTK_INFO("%s: FW Ver = 0x%x", __func__, bdev->fw_version);

	memset(bdev->rom_patch_bin_file_name, 0, MAX_BIN_FILE_NAME_LEN);
	if ((bdev->fw_version & 0xff) == 0xff) {
		BTMTK_ERR("%s: failed, wrong FW version : 0x%x !", __func__, bdev->fw_version);
		return -1;
	}

	/* Bin filename format : "BT_RAM_CODE_MT%04x_%x_%x_hdr.bin"
	 *  $$$$ : chip id
	 *  % : fw version & 0xFF + 1 (in HEX)
	 */
	bdev->flavor = (bdev->flavor & 0x00000080) >> 7;
	BTMTK_INFO("%s: flavor1 = 0x%x", __func__, bdev->flavor);

	/* if flavor equals 1, it represent 7920, else it represent 7921 */
	if (bdev->flavor)
		(void)snprintf(bdev->rom_patch_bin_file_name, MAX_BIN_FILE_NAME_LEN, "BT_RAM_CODE_MT%04x_1a_%x_hdr.bin",
				bdev->chip_id & 0xffff, (bdev->fw_version & 0xff) + 1);
	else
		(void)snprintf(bdev->rom_patch_bin_file_name, MAX_BIN_FILE_NAME_LEN, "BT_RAM_CODE_MT%04x_1_%x_hdr.bin",
				bdev->chip_id & 0xffff, (bdev->fw_version & 0xff) + 1);

	BTMTK_INFO("%s: rom patch file name is %s", __func__, bdev->rom_patch_bin_file_name);

	memcpy(bdev->bt_cfg_file_name, BT_CFG_NAME, sizeof(BT_CFG_NAME));
	memset(bdev->bdaddr, 0, BD_ADDRESS_SIZE);
	return 0;
}

static int btmtk_send_vendor_cfg(struct btmtk_dev *bdev)
{
	int ret = 0;
	u16 index = 0;
	uint8_t event[EVT_HDR_LEN] = { 0x04, 0x0E };

	BTMTK_INFO("%s enter", __func__);

	for (index = 0; index < VENDOR_CMD_COUNT; index++) {
		if (bdev->bt_cfg.vendor_cmd[index].content &&
			bdev->bt_cfg.vendor_cmd[index].length) {
			ret = btmtk_main_send_cmd(bdev,
				bdev->bt_cfg.vendor_cmd[index].content,
				bdev->bt_cfg.vendor_cmd[index].length,
				event, EVT_HDR_LEN,
				0, 0, BTMTK_TX_PKT_FROM_HOST);
			if (ret < 0) {
				BTMTK_ERR("%s: Send vendor cmd failed(%d)! Index: %d",
					__func__, ret, index);
				goto exit;
			}

			BTMTK_INFO_RAW(bdev->bt_cfg.vendor_cmd[index].content,
				bdev->bt_cfg.vendor_cmd[index].length, "send vendor cmd");
		}
	}

exit:
	BTMTK_INFO("%s exit", __func__);
	return ret;
}

static int btmtk_send_phase1_wmt_cfg(struct btmtk_dev *bdev)
{
	int ret = 0;
	u16 index = 0;
	uint8_t event[EVT_HDR_LEN] = { 0x04, 0xE4 };

	BTMTK_INFO("%s", __func__);

	for (index = 0; index < PHASE1_WMT_CMD_COUNT; index++) {
		if (bdev->bt_cfg.phase1_wmt_cmd[index].content &&
			bdev->bt_cfg.phase1_wmt_cmd[index].length) {
			ret = btmtk_main_send_cmd(bdev,
					bdev->bt_cfg.phase1_wmt_cmd[index].content,
					bdev->bt_cfg.phase1_wmt_cmd[index].length,
					event, EVT_HDR_LEN,
					DELAY_TIMES, RETRY_TIMES, BTMTK_TX_CMD_FROM_DRV);
			if (ret < 0) {
				BTMTK_ERR("%s: Send phase1 wmt cmd failed(%d)! Index: %d",
					__func__, ret, index);
				goto exit;
			}

			BTMTK_INFO_RAW(bdev->bt_cfg.phase1_wmt_cmd[index].content,
				bdev->bt_cfg.phase1_wmt_cmd[index].length, "send wmt cmd");
		}
	}

exit:
	BTMTK_INFO("%s exit", __func__);
	return ret;
}

int btmtk_send_init_cmds(struct btmtk_dev *bdev)
{
	int ret = -1;

	if (!bdev) {
		BTMTK_ERR("%s: bdev is NULL !", __func__);
		goto exit;
	}

	BTMTK_INFO("%s", __func__);

#if ENABLESTP
	btmtk_send_set_stp_cmd(bdev);
	btmtk_send_set_stp1_cmd(bdev);
#endif
	ret = btmtk_calibration_flow(bdev);
	if (ret < 0) {
		BTMTK_ERR("%s, btmtk_calibration_flow failed!", __func__);
		goto exit;
	}
	ret = btmtk_send_wmt_power_on_cmd(bdev);
	if (ret < 0) {
		if (bdev->power_state != BTMTK_DONGLE_STATE_POWER_ON) {
			BTMTK_ERR("%s, btmtk_send_wmt_power_on_cmd failed!", __func__);
			if (main_info.reset_stack_flag == HW_ERR_NONE)
				main_info.reset_stack_flag = HW_ERR_CODE_POWER_ON;
		}
		goto exit;
	}

	ret = btmtk_send_phase1_wmt_cfg(bdev);
	if (ret < 0) {
		BTMTK_ERR("btmtk_send_wmt_cfg failed");
		goto exit;
	}

	if (bdev->bt_cfg.support_auto_picus == true) {
		if (btmtk_picus_enable(bdev) < 0) {
			BTMTK_ERR("send picus filter param failed");
			ret = -1;
			goto exit;
		}
	}

	ret = btmtk_send_vendor_cfg(bdev);
	if (ret < 0) {
		BTMTK_ERR("btmtk_send_vendor_cfg failed");
		goto exit;
	}

	if (is_mt7663(bdev->chip_id))
		ret = btmtk_send_hci_tci_set_sleep_cmd_766x(bdev);

exit:
	return ret;
}


int btmtk_send_deinit_cmds(struct btmtk_dev *bdev)
{
	int ret = -1;

	if (!bdev) {
		BTMTK_ERR("%s: bdev is NULL !", __func__);
		return ret;
	}

	BTMTK_INFO("%s", __func__);

	if (bdev->bt_cfg.support_auto_picus == true) {
		if (btmtk_picus_disable(bdev) < 0) {
			BTMTK_ERR("send picus filter param failed");
			btmtk_send_assert_cmd(bdev);
			return -1;
		}
	}

	ret = btmtk_send_wmt_power_off_cmd(bdev);
	if (bdev->power_state != BTMTK_DONGLE_STATE_POWER_OFF) {
		BTMTK_WARN("Power off failed, reset it");
		if (main_info.reset_stack_flag == HW_ERR_NONE)
			main_info.reset_stack_flag = HW_ERR_CODE_POWER_OFF;
		btmtk_send_assert_cmd(bdev);
	}

	return ret;
}

int btmtk_send_assert_cmd(struct btmtk_dev *bdev)
{
	int ret = 0;
	int state;
	u8 cmd[ASSERT_CMD_LEN] = { 0x01, 0x6F, 0xFC, 0x05, 0x01, 0x02, 0x01, 0x00, 0x08 };
	struct sk_buff *skb = NULL;

	if (!bdev) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		ret = -EINVAL;
		goto exit;
	}

	state = btmtk_get_chip_state(bdev);
	if (state == BTMTK_STATE_FW_DUMP) {
		BTMTK_WARN("%s: FW dumping already!!!", __func__);
		return ret;
	}

	BTMTK_INFO("%s: send assert cmd", __func__);

	skb = alloc_skb(ASSERT_CMD_LEN + BT_SKB_RESERVE, GFP_ATOMIC);
	if (!skb) {
		BTMTK_ERR("%s allocate skb failed!!", __func__);
		goto exit;
	}
	bt_cb(skb)->pkt_type = HCI_COMMAND_PKT;
	memcpy(skb->data, cmd, ASSERT_CMD_LEN);
	skb->len = ASSERT_CMD_LEN;

	ret = main_info.hif_hook.send_cmd(bdev, skb, WMT_DELAY_TIMES, RETRY_TIMES, (int)BTMTK_TX_CMD_FROM_DRV);
	if (ret < 0)
		BTMTK_ERR("%s failed!!", __func__);
	else
		BTMTK_INFO("%s: OK", __func__);

exit:
	return ret;
}

static int btmtk_send_txpower_cmd(struct btmtk_dev *bdev)
{
	/**
	 *  TCI Set TX Power Command
	 *  01 2C FC 0C QQ 00 00 00 XX YY ZZ GG AA BB CC DD
	 *  QQ: EDR init TX power dbm // the value is equal to EDR MAX
	 *  XX: BLE TX power dbm
	 *  YY: EDR MAX TX power dbm
	 *  ZZ: Enable LV9
	 *  GG: 3db diff mode
	 *  AA: [5:4] Indicator // [5] 1: command send to BT1, [4] 1: command send to BT0
	 *      [3:0] Resolution // 0: 1dBm, 1: 0.5dBm, 2: 0.25dBm
	 *  BB: BLE 2M
	 *  CC: BLE S2
	 *  DD: BLE S8
	 */

	u8 cmd[TXPOWER_CMD_LEN] = { 0x01, 0x2C, 0xFC, 0x0C,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 event[TXPOWER_EVT_LEN] = { 0x04, 0x0E, 0x04, 0x01, 0x2C, 0xFC, 0x00 };
	int ret = 0;

	cmd[4] = (u8)main_info.PWS.EDR_Max;
	cmd[8] = (u8)main_info.PWS.BLE_1M;
	cmd[9] = (u8)main_info.PWS.EDR_Max;
	cmd[10] = (u8)main_info.PWS.LV9;
	cmd[11] = (u8)main_info.PWS.DM;
	cmd[12] = (u8)main_info.PWS.IR;
	cmd[13] = (u8)main_info.PWS.BLE_2M;
	cmd[14] = (u8)main_info.PWS.BLE_LR_S2;
	cmd[15] = (u8)main_info.PWS.BLE_LR_S8;

	ret = btmtk_main_send_cmd(bdev,
			cmd, TXPOWER_CMD_LEN,
			event, TXPOWER_EVT_LEN,
			0, 0, BTMTK_TX_CMD_FROM_DRV);

	if (ret < 0)
		BTMTK_ERR("%s failed!!", __func__);
	else
		BTMTK_INFO("%s: OK", __func__);

	return ret;
}

static int btmtk_set_power_value(char *str, int resolution, int is_edr)
{
	int power = ERR_PWR, integer = 0, decimal = 0, ret = 0;

	if (resolution == RES_DOT_25) {
		/* XX.YY => XX.YY/0.25 = XX*4 + YY/25 */
		if (strstr(str, ".")) {
			ret = sscanf(str, "%d.%d", &integer, &decimal);
			if (ret < 0)
				return ret;
			if (decimal != 25 && decimal != 75 && decimal != 5 && decimal != 50)
				return ERR_PWR;
			if (decimal == 5)
				decimal = 50;
			if (integer >= 0)
				power = integer * 4 + decimal / 25;
			else
				power = integer * 4 - decimal / 25;
		} else {
			ret = sscanf(str, "%d", &integer);
			if (ret < 0)
				return ret;
			power = integer * 4;
		}

		BTMTK_DBG("%s: power = %d", __func__, power);

		if (is_edr) {
			if (power > EDR_MAX_R2 || power < EDR_MIN_R2)
				return ERR_PWR;
			if (power >= EDR_MIN_LV9_R2)
				main_info.PWS.LV9 = 1;
		} else if (!is_edr && (power > BLE_MAX_R2 || power < BLE_MIN_R2))
			return ERR_PWR;
	} else if (resolution == RES_DOT_5) {
		/* XX.YY => XX.YY/0.5 = XX*2 + YY/5 */
		if (strstr(str, ".")) {
			ret = sscanf(str, "%d.%d", &integer, &decimal);
			if (ret < 0)
				return ret;
			if (decimal != 5)
				return ERR_PWR;
			if (integer >= 0)
				power = integer * 2 + decimal / 5;
			if (integer < 0)
				power = integer * 2 - decimal / 5;
		} else {
			ret = sscanf(str, "%d", &integer);
			if (ret < 0)
				return ret;
			power = integer * 2;
		}

		BTMTK_DBG("%s: power = %d", __func__, power);

		if (is_edr) {
			if (power > EDR_MAX_R1 || power < EDR_MIN_R1)
				return ERR_PWR;
			if (power >= EDR_MIN_LV9_R1)
				main_info.PWS.LV9 = 1;
		} else if (!is_edr && (power > BLE_MAX_R1 || power < BLE_MIN_R1))
			return ERR_PWR;
	} else if (resolution == RES_1) {
		ret = sscanf(str, "%d", &power);
		if (ret < 0)
			return ret;

		BTMTK_DBG("%s: power = %d", __func__, power);

		if (is_edr) {
			if (power > EDR_MAX || power < EDR_MIN)
				return ERR_PWR;
			if (power >= EDR_MIN_LV9)
				main_info.PWS.LV9 = 1;
		} else if (!is_edr && (power > BLE_MAX || power < BLE_MIN))
			return ERR_PWR;
	}

	return power;
}

static int btmtk_check_power_resolution(char *str)
{
	if (str == NULL)
		return -1;
	if (strstr(str, ".25") || strstr(str, ".75"))
		return RES_DOT_25;
	if (strstr(str, ".5"))
		return RES_DOT_5;
	if (!strstr(str, ".") || strstr(str, ".0"))
		return RES_1;
	return -1;
}

static void btmtk_init_power_setting_struct(void)
{
	main_info.PWS.BLE_1M = 0;
	main_info.PWS.EDR_Max = 0;
	main_info.PWS.LV9 = 0;
	main_info.PWS.DM = 0;
	main_info.PWS.IR = 0;
	main_info.PWS.BLE_2M = 0;
	main_info.PWS.BLE_LR_S2 = 0;
	main_info.PWS.BLE_LR_S8 = 0;
}

static int btmtk_parse_power_table(char *context)
{
	char *ptr = NULL;
	int step = 0, temp;
	int resolution;
	int power;

	if (context == NULL) {
		BTMTK_ERR("%s context is NULL", __func__);
		return -1;
	}

	BTMTK_INFO("%s", __func__);
	btmtk_init_power_setting_struct();

	/* Send to BT0? BT1? */
	if (strstr(context, "BT0")) {
		BTMTK_INFO("Parse power for BT0");
		main_info.PWS.IR |= 0x10;
		context += strlen("[BT0]");
	} else if (strstr(context, "BT1")) {
		BTMTK_INFO("Parse power for BT1");
		main_info.PWS.IR |= 0x20;
		context += strlen("[BT1]");
	} else {
		BTMTK_ERR("%s BT indicator error", __func__);
		return -1;
	}

	resolution = btmtk_check_power_resolution(context);
	if (resolution == -1) {
		BTMTK_ERR("Check resolution fail");
		return -1;
	}

	main_info.PWS.IR |= resolution;
	BTMTK_INFO("%s: resolution = %d", __func__, resolution);

	while ((ptr = strsep(&context, ",")) != NULL) {
		while (*ptr == '\t' || *ptr == ' ')
			ptr++;

		switch (step) {
		/* BR_EDR_PWR_MODE */
		case CHECK_SINGLE_SKU_PWR_MODE:
			if (kstrtoint(ptr, 0, &temp) == 0) {
				if (temp == 0 || temp == 1) {
					main_info.PWS.DM = temp;
					step++;
					continue;
				} else {
					BTMTK_ERR("PWR MODE value wrong");
					return -1;
				}
			} else {
				BTMTK_ERR("Read PWR MODE Fail");
				return -1;
			}
			break;
		/* Parse EDR MAX */
		case CHECK_SINGLE_SKU_EDR_MAX:
			power = btmtk_set_power_value(ptr, resolution, 1);
			if (power == ERR_PWR) {
				BTMTK_ERR("EDR MAX value wrong");
				return -1;
			}
			main_info.PWS.EDR_Max = power;
			step++;
			break;
		/* Parse BLE Default */
		case CHECK_SINGLE_SKU_BLE:
			power = btmtk_set_power_value(ptr, resolution, 0);
			if (power == ERR_PWR) {
				BTMTK_ERR("BLE value wrong");
				return -1;
			}
			main_info.PWS.BLE_1M = power;
			step++;
			break;
		/* Parse BLE 2M */
		case CHECK_SINGLE_SKU_BLE_2M:
			power = btmtk_set_power_value(ptr, resolution, 0);
			if (power == ERR_PWR) {
				BTMTK_ERR("BLE 2M value wrong");
				return -1;
			}
			main_info.PWS.BLE_2M = power;
			step++;
			break;
		/* Parse BLE long range S2 */
		case CHECK_SINGLE_SKU_BLE_LR_S2:
			power = btmtk_set_power_value(ptr, resolution, 0);
			if (power == ERR_PWR) {
				BTMTK_ERR("BLE LR S2 value wrong");
				return -1;
			}
			main_info.PWS.BLE_LR_S2 = power;
			step++;
			break;
		/* Parse BLE long range S8 */
		case CHECK_SINGLE_SKU_BLE_LR_S8:
			power = btmtk_set_power_value(ptr, resolution, 0);
			if (power == ERR_PWR) {
				BTMTK_ERR("BLE LR S8 value wrong");
				return -1;
			}
			main_info.PWS.BLE_LR_S8 = power;
			step++;
			break;
		default:
			BTMTK_ERR("%s step is wrong: %d", __func__, step);
			break;
		}
		continue;
	}

	return step;
}

static void btmtk_send_txpower_cmd_to_all_interface(void)
{
	int i, ret;
	struct btmtk_dev *bdev = NULL;

	for (i = 0; i < btmtk_intf_num; i++) {
		if (g_bdev[i]->hdev != NULL) {
			bdev = g_bdev[i];
			BTMTK_INFO("send to %d", i);
			ret = btmtk_send_txpower_cmd(bdev);
			if (ret < 0)
				BTMTK_ERR("Device %d send txpower cmd fail", i);
		}
	}
}

static void btmtk_requset_country_cb(const struct firmware *fw, void *context)
{
	char *ptr, *data, *p_data = NULL;
	char *country = NULL;
	int ret = 0;
	bool find_country = false;
	bool read_next = false;

	if (fw == NULL) {
		BTMTK_ERR("fw is NULL");
		return;
	}

	BTMTK_INFO("%s request %s success", __func__, DEFAULT_COUNTRY_TABLE_NAME);
	p_data = data = kzalloc(fw->size, GFP_KERNEL);
	if (data == NULL) {
		BTMTK_WARN("%s allocate memory fail (data)", __func__);
		goto exit;
	}

	memcpy(data, fw->data, fw->size);
	while ((ptr = strsep(&p_data, "\n")) != NULL) {
		/* If the '#' in front of the line, ignore this line */
		if (*ptr == '#')
			continue;

		/* Set power for BT1 */
		if (read_next) {
			if (strncmp(ptr, "[BT1]", 5) == 0) {
				ret = btmtk_parse_power_table(ptr);
				if (ret != CHECK_SINGLE_SKU_ALL) {
					BTMTK_ERR("Parse power fail, ret = %d", ret);
					break;
				}

				btmtk_send_txpower_cmd_to_all_interface();
			} else {
				BTMTK_INFO("No power data for BT1");
			}
			break;
		}

		if (find_country) {
			ret = btmtk_parse_power_table(ptr);
			/* Check if the next line has power value for BT1 */
			read_next = true;
			if (ret != CHECK_SINGLE_SKU_ALL) {
				BTMTK_ERR("Parse power fail, ret = %d", ret);
				continue;
			}

			btmtk_send_txpower_cmd_to_all_interface();
			continue;
		}

		while ((country = strsep(&ptr, ",[]")) != NULL) {
			if (strlen(country) != COUNTRY_CODE_LEN)
				continue;
			if (strcmp(country, main_info.PWS.country_code) == 0) {
				find_country = true;
				break;
			}
		}
	}
	kfree(data);

	if (find_country == false)
		BTMTK_ERR("Can't find country in the table");

exit:
	release_firmware(fw);
}

static int btmtk_load_country_table(struct btmtk_dev *bdev)
{
	int err = 0;

	if (bdev->country_file_name)
		err = request_firmware_nowait(THIS_MODULE, true,
			bdev->country_file_name, NULL, GFP_KERNEL, NULL,
			btmtk_requset_country_cb);
	else
		BTMTK_WARN("%s country_file_name is null", __func__);

	return err;
}

void btmtk_set_country_code_from_wifi(char *code)
{
	int i;
	struct btmtk_dev *bdev = NULL;

	if (!code)
		return;

	if (strlen(code) == COUNTRY_CODE_LEN) {
		BTMTK_INFO("%s country code is %s", __func__, code);
		memcpy(main_info.PWS.country_code, code, sizeof(main_info.PWS.country_code));
		for (i = 0; i < btmtk_intf_num; i++) {
			if (g_bdev[i]->hdev != NULL) {
				bdev = g_bdev[i];
				if (bdev->bt_cfg.support_bt_single_sku) {
					btmtk_load_country_table(bdev);
					break;
				}
			}
		}
	} else {
		BTMTK_INFO("%s country code is not valid", __func__);
	}
}
EXPORT_SYMBOL_GPL(btmtk_set_country_code_from_wifi);


/**
 * Kernel HCI Interface Registeration
 */
static int bt_flush(struct hci_dev *hdev)
{
	struct btmtk_dev *bdev =  hci_get_drvdata(hdev);

	if (is_mt66xx(bdev->chip_id))
		return main_info.hif_hook.flush(bdev);
	else
		return 0;
}

static int bt_close(struct hci_dev *hdev)
{
	int ret = -1;
	int state = BTMTK_STATE_INIT;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	struct btmtk_dev *bdev = NULL;

	if (!hdev) {
		BTMTK_ERR("%s: invalid parameters!", __func__);
		return ret;
	}

	bdev = hci_get_drvdata(hdev);
	if (!bdev) {
		BTMTK_ERR("%s: bdev is invalid!", __func__);
		return ret;
	}

	fstate = btmtk_fops_get_state(bdev);
	if (fstate != BTMTK_FOPS_STATE_OPENED) {
		BTMTK_WARN("%s: fops is not allow close(%d)", __func__, fstate);
		goto err;
	}
	btmtk_fops_set_state(bdev, BTMTK_FOPS_STATE_CLOSING);

	state = btmtk_get_chip_state(bdev);
	if (state != BTMTK_STATE_WORKING && state != BTMTK_STATE_STANDBY) {
		/* If hif disconnect occurs,
		 * it will call cif_mutex_lock and release hci device.
		 * Release hci device will call bt_close.
		 * It must return with this,
		 * otherwise the below cif_mutex_lock will cause deadlock
		 */
		BTMTK_WARN("%s: not in working state and standby state(%d).", __func__, state);
		goto exit;
	}

	BTMTK_INFO("%s, enter", __func__);

	if (main_info.hif_hook.cif_mutex_lock)
		main_info.hif_hook.cif_mutex_lock(bdev);

	if (!is_mt66xx(bdev->chip_id)) {
		state = btmtk_get_chip_state(bdev);
		if (state != BTMTK_STATE_WORKING && state != BTMTK_STATE_STANDBY) {
			/* It's for the case that
			 * bt_close and hif_disconnect occur at the same time
			 */
			BTMTK_WARN("%s: not in working state and standby state(%d).", __func__, state);
			goto unlock;
		}

#if CFG_SUPPORT_DVT
		/* Don't send init cmd for DVT
		 * Such as Lowpower DVT
		 */
		bdev->power_state = BTMTK_DONGLE_STATE_POWER_OFF;
#else
		if (state != BTMTK_STATE_STANDBY) {
			ret = btmtk_send_deinit_cmds(bdev);
			if (ret < 0) {
				BTMTK_ERR("%s, btmtk_send_deinit_cmds failed", __func__);
				goto unlock;
			}
		}
#endif /* CFG_SUPPORT_DVT */
	}

	/* Flush RX works */
	flush_work(&bdev->rx_work);

	/* Drop queues */
	skb_queue_purge(&bdev->rx_q);

	main_info.hif_hook.close(hdev);

unlock:
	if (main_info.hif_hook.cif_mutex_unlock)
		main_info.hif_hook.cif_mutex_unlock(bdev);
exit:
	btmtk_fops_set_state(bdev, BTMTK_FOPS_STATE_CLOSED);

err:
	main_info.reset_stack_flag = HW_ERR_NONE;

	BTMTK_INFO("%s: end, reset_stack_flag = %d", __func__, main_info.reset_stack_flag);
	return 0;
}

unsigned long dummy_func(const char *ptr) {
	return 0;
}

static int bt_open(struct hci_dev *hdev)
{
	int ret = -1;
	int state = BTMTK_STATE_INIT;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	struct btmtk_dev *bdev = NULL;
	void (*rlm_get_alpha2)(char *);
	const char *wifi_func_name = "rlm_get_alpha2";
	char alpha2[5];

	BTMTK_INFO("%s: MTK BT Driver Version : %s", __func__, VERSION);

	if (!hdev) {
		BTMTK_ERR("%s: invalid parameters!", __func__);
		return -EFAULT;
	}

	bdev = hci_get_drvdata(hdev);
	if (!bdev) {
		BTMTK_ERR("%s: bdev is invalid", __func__);
		return -EFAULT;
	}

	state = btmtk_get_chip_state(bdev);
	if (state == BTMTK_STATE_INIT || state == BTMTK_STATE_DISCONNECT) {
		ret = -EAGAIN;
		goto failed;
	}

	if (state != BTMTK_STATE_WORKING && state != BTMTK_STATE_STANDBY) {
		BTMTK_WARN("%s: not in working state and standby state(%d).", __func__, state);
		ret = -ENODEV;
		goto failed;
	}

	fstate = btmtk_fops_get_state(bdev);
	if (fstate == BTMTK_FOPS_STATE_OPENED) {
		BTMTK_WARN("%s: fops opened!", __func__);
		ret = -EIO;
		goto failed;
	}

	if ((fstate == BTMTK_FOPS_STATE_CLOSING) ||
		(fstate == BTMTK_FOPS_STATE_OPENING)) {
		BTMTK_WARN("%s: fops open/close is on-going !", __func__);
		ret = -EAGAIN;
		goto failed;
	}

	BTMTK_INFO("%s", __func__);
	btmtk_fops_set_state(bdev, BTMTK_FOPS_STATE_OPENING);
	ret = main_info.hif_hook.open(hdev);
	if (ret < 0) {
		BTMTK_ERR("%s, cif_open failed", __func__);
		goto failed;
	}

	if (!is_mt66xx(bdev->chip_id)) {
#if CFG_SUPPORT_DVT
		/* Don't send init cmd for DVT
		 * Such as Lowpower DVT
		 */
		bdev->power_state = BTMTK_DONGLE_STATE_POWER_ON;
#else
		ret = btmtk_send_init_cmds(bdev);
		if (ret < 0) {
			BTMTK_ERR("%s, btmtk_send_init_cmds failed", __func__);
			goto failed;
		}
#endif /* CFG_SUPPORT_DVT */

		if (main_info.hif_hook.open_done)
			main_info.hif_hook.open_done(bdev);
	}

	btmtk_fops_set_state(bdev, BTMTK_FOPS_STATE_OPENED);
	main_info.reset_stack_flag = HW_ERR_NONE;

	if (bdev->bt_cfg.support_bt_single_sku) {
		// todo: need to unmask, kernel-5.10 not export, submit request to google
		//rlm_get_alpha2 = (void *)kallsyms_lookup_name(wifi_func_name);
		rlm_get_alpha2 = (void*)dummy_func(wifi_func_name);

		if (rlm_get_alpha2) {
			rlm_get_alpha2(alpha2);
			if (strlen(alpha2) == COUNTRY_CODE_LEN) {
				BTMTK_INFO("Wifi set country code %s", alpha2);
				memcpy(main_info.PWS.country_code, alpha2, sizeof(main_info.PWS.country_code));
			} else {
				BTMTK_ERR("Country code length is wrong");
			}
		} else {
			BTMTK_INFO("Wifi didn't set country code");
		}

		main_info.PWS.country_code[COUNTRY_CODE_LEN] = '\0';
		if (strcmp(main_info.PWS.country_code, "") != 0)
			btmtk_load_country_table(bdev);
	}

	return 0;

failed:
	btmtk_fops_set_state(bdev, BTMTK_FOPS_STATE_CLOSED);

	return ret;
}

static int bt_setup(struct hci_dev *hdev)
{
	int ret = 0;
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);

	BTMTK_INFO("%s", __func__);

	if (is_mt66xx(bdev->chip_id)) {
		ret = main_info.hif_hook.open(hdev);
		if (ret)
			BTMTK_ERR("%s: fail", __func__);
		return ret;
	}
	return 0;
}

static int bt_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	int ret = -1;
	int state = BTMTK_STATE_INIT;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	/* parsing commands */
	u8 fw_coredump_cmd[FW_COREDUMP_CMD_LEN] = { 0x01, 0x5B, 0xFD, 0x00 };
	u8 reset_cmd[HCI_RESET_CMD_LEN] = { 0x01, 0x03, 0x0C, 0x00 };
	struct btmtk_dev *bdev = NULL;

	if (hdev == NULL || skb == NULL) {
		BTMTK_ERR("%s, invalid parameters!", __func__);
		return -ENODEV;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL) {
		BTMTK_ERR("%s, bdev is invalid!", __func__);
		return -ENODEV;
	}

	fstate = btmtk_fops_get_state(bdev);
	if (fstate != BTMTK_FOPS_STATE_OPENED) {
		BTMTK_WARN("%s: fops is not open yet(%d)!", __func__, fstate);
		ret = -ENODEV;
		goto exit;
	}

	state = btmtk_get_chip_state(bdev);
	if (state != BTMTK_STATE_WORKING) {
		BTMTK_WARN("%s: chip state is %d.", __func__, state);
		if (state == BTMTK_STATE_DISCONNECT) {
			ret = -ENODEV;
		} else {
			msleep(3000);
			ret = -EAGAIN;
		}
		goto exit;
	}

	if (bdev->power_state == BTMTK_DONGLE_STATE_POWER_OFF) {
		BTMTK_WARN("%s: dongle state already power off, do not write", __func__);
		ret = -EFAULT;
		goto exit;
	}

	if (main_info.reset_stack_flag) {
		BTMTK_WARN("%s: reset_stack_flag (%d)!", __func__, main_info.reset_stack_flag);
		ret = -EFAULT;
		goto exit;
	}

	if (!is_mt66xx(bdev->chip_id))
		btmtk_dispatch_fwlog_bluetooth_kpi(bdev, skb->data, skb->len, hci_skb_pkt_type(skb));

	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);

#if ENABLESTP
	skb = mtk_add_stp(bdev, skb);
#endif

	if (!is_mt66xx(bdev->chip_id)) {
		/* For Ble ISO packet size */
		if (memcmp(skb->data, main_info.read_iso_packet_size_cmd,
			READ_ISO_PACKET_SIZE_CMD_HDR_LEN) == 0) {
			bdev->iso_threshold = skb->data[READ_ISO_PACKET_SIZE_CMD_HDR_LEN] +
						(skb->data[READ_ISO_PACKET_SIZE_CMD_HDR_LEN + 1]  << 8);
			BTMTK_INFO("%s: Ble iso pkt size is %d", __func__, bdev->iso_threshold);
		}

		if (hci_skb_pkt_type(skb) == HCI_COMMAND_PKT) {
			/* save hci cmd pkt for debug */
			btmtk_hci_snoop_save_cmd(skb->len, skb->data);
			if (skb->len == FW_COREDUMP_CMD_LEN &&
				!memcmp(skb->data, fw_coredump_cmd, FW_COREDUMP_CMD_LEN)) {
				BTMTK_INFO("%s: Dongle FW Assert Triggered by BT Stack!", __func__);
				btmtk_hci_snoop_print_to_log();
			} else if (skb->len == HCI_RESET_CMD_LEN &&
					!memcmp(skb->data, reset_cmd, HCI_RESET_CMD_LEN))
				BTMTK_INFO("%s: got command: 0x03 0C 00 (HCI_RESET)", __func__);
			}

		ret = main_info.hif_hook.send_cmd(bdev, skb, 0, 0, (int)BTMTK_TX_PKT_FROM_HOST);
		if (ret < 0)
			BTMTK_ERR("%s failed!!", __func__);
	} else {
		ret = main_info.hif_hook.send_cmd(bdev, skb, 0, 5, (int)BTMTK_TX_PKT_FROM_HOST);
	}

exit:
	return ret;
}

void btmtk_reg_hif_hook(struct hif_hook_ptr *hook)
{
	memcpy(&main_info.hif_hook, hook, sizeof(struct hif_hook_ptr));
}

static void btmtk_rx_work(struct work_struct *work)
{
	int err = 0, skip_pkt = 0;
	struct btmtk_dev *bdev = container_of(work, struct btmtk_dev, rx_work);
	struct sk_buff *skb;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	int state = 0;

	BTMTK_DBG("%s enter", __func__);

	while ((skb = skb_dequeue(&bdev->rx_q))) {
		if (!is_mt66xx(bdev->chip_id)) {
			/* BTMTK_DBG_RAW(skb->data, skb->len, "%s, recv evt", __func__); */
			skip_pkt = btmtk_dispatch_fwlog(bdev, skb);
			if (skip_pkt != 0) {
				/* kfree_skb should be moved to btmtk_dispach_pkt */
				kfree_skb(skb);
				continue;
			}
		}

		if (hci_skb_pkt_type(skb) == HCI_EVENT_PKT) {
			/* save hci evt pkt for debug */
			if (skb->data[0] == 0x3E)
				btmtk_hci_snoop_save_adv_event(skb->len, skb->data);
			else
				btmtk_hci_snoop_save_event(skb->len, skb->data);

			if (main_info.hif_hook.event_filter(bdev, skb)) {
				/* Drop by driver, don't send to stack */
				kfree_skb(skb);
				continue;
			}
		} else if (hci_skb_pkt_type(skb) == HCI_ACLDATA_PKT) {
			/* save hci acl pkt for debug, not include picus log and coredump*/
			btmtk_hci_snoop_save_acl(skb->len, skb->data);
		}

		fstate = btmtk_fops_get_state(bdev);
		if (fstate != BTMTK_FOPS_STATE_OPENED) {
			/* BT close case, drop by driver, don't send to stack */
			kfree_skb(skb);
			continue;
		}

		if (is_mt66xx(bdev->chip_id)) {
			err = btmtk_cif_rx_packet_handler(bdev->hdev, skb);
		} else {
			/* for bluetooth kpi */
			btmtk_dispatch_fwlog_bluetooth_kpi(bdev, skb->data, skb->len, hci_skb_pkt_type(skb));
			/* Can't send to stack when is not WORKING */
			state = btmtk_get_chip_state(bdev);
			if (state != BTMTK_STATE_WORKING) {
				kfree_skb(skb);
				continue;
			}

			err = hci_recv_frame(bdev->hdev, skb);
		}
		if (err < 0) {
			if (err != -ENXIO)
				BTMTK_ERR("%s btmtk_rx_work failed, err = %d", __func__, err);
			return;
		}
	}
}

void btmtk_free_hci_device(struct btmtk_dev *bdev, int hci_bus_type)
{
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;

	if (!bdev)
		return;

	BTMTK_INFO("%s Begin", __func__);
	/* Flush RX works */
	flush_work(&bdev->rx_work);

	/* Drop queues */
	skb_queue_purge(&bdev->rx_q);
	destroy_workqueue(bdev->workqueue);

	if (bdev->hdev) {
		hci_free_dev(bdev->hdev);
		bdev->hdev = NULL;
	}

	fstate = btmtk_fops_get_state(bdev);
	if (fstate == BTMTK_FOPS_STATE_OPENED || fstate == BTMTK_FOPS_STATE_CLOSING) {
		BTMTK_WARN("%s: fstate = %d , set reset_stack_flag", __func__, fstate);
		if (main_info.reset_stack_flag == HW_ERR_NONE)
			main_info.reset_stack_flag = HW_ERR_CODE_USB_DISC;
	}

	bdev->chip_reset = 0;
	BTMTK_INFO("%s End", __func__);
}

int btmtk_allocate_hci_device(struct btmtk_dev *bdev, int hci_bus_type)
{
	struct hci_dev *hdev;
	int err = 0;

	if (!bdev) {
		BTMTK_ERR("%s, bdev is NULL!", __func__);
		err = -EINVAL;
		goto end;
	}

	BTMTK_INFO("%s", __func__);
	/* Add hci device */
	hdev = hci_alloc_dev();
	if (!hdev) {
		BTMTK_ERR("%s, hdev is NULL!", __func__);
		err = -ENOMEM;
		goto end;
	}

	hdev->bus = hci_bus_type;
	hci_set_drvdata(hdev, bdev);

	/* HCI_PRIMARY = 0x00 */
	hdev->dev_type = 0x00;

	bdev->hdev = hdev;

	/* register hci callback */
	hdev->open	   = bt_open;
	hdev->close    = bt_close;
	hdev->flush    = bt_flush;
	hdev->send	   = bt_send_frame;
	hdev->setup    = bt_setup;

	init_waitqueue_head(&bdev->p_wait_event_q);

	/* rx_work init */
	INIT_WORK(&bdev->rx_work, btmtk_rx_work);
	skb_queue_head_init(&bdev->rx_q);
	bdev->workqueue = alloc_workqueue("BTMTK_RX_WQ", WQ_HIGHPRI | WQ_UNBOUND |
					  WQ_MEM_RECLAIM, 1);
	if (!bdev->workqueue) {
		BTMTK_ERR("%s, bdev->workqueue is NULL!", __func__);
		err = -ENOMEM;
		goto err0;
	}

	BTMTK_INFO("%s done", __func__);
	return 0;

err0:
	hci_free_dev(hdev);
	hdev = NULL;
end:
	return err;
}

int btmtk_register_hci_device(struct btmtk_dev *bdev)
{
	struct hci_dev *hdev;
	int err = 0;
	int ret = 0;

	hdev = bdev->hdev;

	err = hci_register_dev(hdev);
	/* After hci_register_dev completed
	 * It will set dev_flags to HCI_SETUP
	 * That cause vendor_lib create socket failed
	 */
	if (err < 0) {
		BTMTK_INFO("%s can't register", __func__);
		goto exit;
	}

#if CFG_SUPPORT_BLUEZ

#else
#if (KERNEL_VERSION(4, 4, 0) > LINUX_VERSION_CODE)
		ret = test_and_clear_bit(HCI_SETUP, &hdev->dev_flags);
#else
		ret = hci_dev_test_and_clear_flag(hdev, HCI_SETUP);
#endif
		if (ret)
			BTMTK_INFO("%s, the bit value returned is %d", __func__, ret);
		else
			BTMTK_INFO("%s, the bit value returned is %d", __func__, ret);

#endif /* CFG_SUPPORT_BLUEZ */

exit:
	return err;
}

int btmtk_deregister_hci_device(struct btmtk_dev *bdev)
{
	int err = 0;

	if (bdev && bdev->hdev)
		hci_unregister_dev(bdev->hdev);

	return err;
}

static int btmtk_main_allocate_memory(struct btmtk_dev *bdev)
{
	int err = -1;

	BTMTK_INFO("%s Begin", __func__);

	if (bdev->rom_patch_bin_file_name == NULL) {
		bdev->rom_patch_bin_file_name = kzalloc(MAX_BIN_FILE_NAME_LEN, GFP_KERNEL);
		if (!bdev->rom_patch_bin_file_name) {
			BTMTK_ERR("%s: alloc memory fail (bdev->rom_patch_bin_file_name)", __func__);
			goto end;
		}
	}

	if (bdev->io_buf == NULL) {
		bdev->io_buf = kzalloc(IO_BUF_SIZE, GFP_KERNEL);
		if (!bdev->io_buf) {
			BTMTK_ERR("%s: alloc memory fail (bdev->io_buf)", __func__);
			goto err2;
		}
	}

	if (bdev->bt_cfg_file_name == NULL) {
		bdev->bt_cfg_file_name = kzalloc(MAX_BIN_FILE_NAME_LEN, GFP_KERNEL);
		if (!bdev->bt_cfg_file_name) {
			BTMTK_ERR("%s: alloc memory fail (bdev->bt_cfg_file_name)", __func__);
			goto err1;
		}
	}

	if (bdev->country_file_name == NULL) {
		bdev->country_file_name = kzalloc(MAX_BIN_FILE_NAME_LEN, GFP_KERNEL);
		if (!bdev->country_file_name) {
			BTMTK_ERR("%s: alloc memory fail (bdev->country_file_name)", __func__);
			goto err0;
		}
	}
	BTMTK_INFO("%s Done", __func__);
	return 0;

err0:
	kfree(bdev->bt_cfg_file_name);
	bdev->bt_cfg_file_name = NULL;
err1:
	kfree(bdev->io_buf);
	bdev->io_buf = NULL;
err2:
	kfree(bdev->rom_patch_bin_file_name);
	bdev->rom_patch_bin_file_name = NULL;
end:
	return err;
}

static void btmtk_main_free_memory(struct btmtk_dev *bdev)
{
	kfree(bdev->rom_patch_bin_file_name);
	bdev->rom_patch_bin_file_name = NULL;

	kfree(bdev->bt_cfg_file_name);
	bdev->bt_cfg_file_name = NULL;

	kfree(bdev->country_file_name);
	bdev->country_file_name = NULL;

	kfree(bdev->io_buf);
	bdev->io_buf = NULL;

	BTMTK_INFO("%s: Success", __func__);
}

int btmtk_main_cif_initialize(struct btmtk_dev *bdev, int hci_bus)
{
	int err = 0;

	err = btmtk_main_allocate_memory(bdev);
	if (err < 0) {
		BTMTK_ERR("btmtk_main_allocate_memory failed!");
		goto end;
	}

	btmtk_initialize_cfg_items(bdev);

	err = btmtk_allocate_hci_device(bdev, hci_bus);
	if (err < 0) {
		BTMTK_ERR("btmtk_allocate_hci_device failed!");
		goto free_mem;
	}

	err = btmtk_cap_init(bdev);
	if (err < 0) {
		BTMTK_ERR("btmtk_cap_init failed!");
		goto free_hci_dev;
	}

	btmtk_load_bt_cfg(bdev->bt_cfg_file_name, bdev->intf_dev, bdev);

	(void)snprintf(bdev->country_file_name, MAX_BIN_FILE_NAME_LEN,
			DEFAULT_COUNTRY_TABLE_NAME);

	return 0;

free_hci_dev:
	btmtk_free_hci_device(bdev, hci_bus);
free_mem:
	btmtk_main_free_memory(bdev);
end:
	return err;
}

void btmtk_main_cif_uninitialize(struct btmtk_dev *bdev, int hci_bus)
{
	btmtk_free_setting_file(bdev);
	btmtk_free_hci_device(bdev, hci_bus);
	btmtk_main_free_memory(bdev);
}

int btmtk_main_cif_disconnect_notify(struct btmtk_dev *bdev, int hci_bus)
{
	btmtk_deregister_hci_device(bdev);
	btmtk_main_cif_uninitialize(bdev, hci_bus);

	bdev->power_state = BTMTK_DONGLE_STATE_POWER_OFF;
	btmtk_release_dev(bdev);

	return 0;
}

#if (USE_DEVICE_NODE != 1)
static int btmtk_reboot_notify(struct notifier_block *nb,
			unsigned long event, void *unused)
{
	int ret = 0;
	int i = 0;
	int cif_event = 0;
	unsigned char fstate = 0;
	int state = 0;
	struct btmtk_cif_state *cif_state = NULL;
	struct btmtk_dev *bdev = NULL;

	BTMTK_INFO("%s: btmtk_reboot_notify(%d)", __func__, (int)event);

	if (event == SYS_RESTART) {
		BTMTK_INFO("%s: enter", __func__);
		for (i = 0; i < btmtk_intf_num; i++) {
			/* Find valid dev for already probe interface. */
			if (g_bdev[i]->hdev != NULL) {
				bdev = g_bdev[i];

				fstate = btmtk_fops_get_state(bdev);
				if (fstate != BTMTK_FOPS_STATE_OPENED) {
					BTMTK_WARN("%s: fops is not opened(%d)", __func__, fstate);
					continue;
				}

				state = btmtk_get_chip_state(bdev);
				if (state != BTMTK_STATE_WORKING) {
					BTMTK_WARN("%s: not in working(%d).", __func__, state);
					continue;
				}

				cif_event = HIF_EVENT_DISCONNECT;
				if (BTMTK_CIF_IS_NULL(bdev, cif_event)) {
					/* Error */
					BTMTK_WARN("%s parameter is NULL", __func__);
					continue;
				}

				cif_state = &bdev->cif_state[cif_event];
				/* Set Entering state */
				btmtk_set_chip_state((void *)bdev, cif_state->ops_enter);

				btmtk_fops_set_state(bdev, BTMTK_FOPS_STATE_CLOSING);

				if (main_info.hif_hook.cif_mutex_lock)
					main_info.hif_hook.cif_mutex_lock(bdev);

				ret = btmtk_send_deinit_cmds(bdev);
				if (ret < 0)
					BTMTK_ERR("%s, btmtk_send_deinit_cmds failed", __func__);
#if 0
				/* Flush RX works */
				flush_work(&bdev->rx_work);

				/* Drop queues */
				skb_queue_purge(&bdev->rx_q);
#endif
				main_info.hif_hook.close(bdev->hdev);

				if (main_info.hif_hook.cif_mutex_unlock)
					main_info.hif_hook.cif_mutex_unlock(bdev);

				btmtk_fops_set_state(bdev, BTMTK_FOPS_STATE_CLOSED);

				/* Set End/Error state */
				if (ret == 0)
					btmtk_set_chip_state((void *)bdev, cif_state->ops_end);
				else
					btmtk_set_chip_state((void *)bdev, cif_state->ops_error);
			}
		}
	}

	return 0;
}

static struct notifier_block btmtk_reboot_notifier = {
	.notifier_call = btmtk_reboot_notify,
	.next = NULL,
	.priority = 0,
};
#endif // USE_DEVICE_NODE

static int main_init(void)
{
	int i = 0;

	BTMTK_INFO("%s", __func__);

	/* Check if user changes default minimum supported intf count */
	if (btmtk_intf_num < BT_MCU_MINIMUM_INTERFACE_NUM) {
		btmtk_intf_num = BT_MCU_MINIMUM_INTERFACE_NUM;
		BTMTK_WARN("%s minimum interface is %d", __func__, btmtk_intf_num);
	}

	BTMTK_INFO("%s supported intf count <%d>", __func__, btmtk_intf_num);

	BTMTK_INFO("%s: Register reboot_notifier callback success.", __func__);
	/* Is it necessary? bt_close will be called by reboot. */
#if (USE_DEVICE_NODE != 1)
	register_reboot_notifier(&btmtk_reboot_notifier);
#endif // USE_DEVICE_NODE
	g_bdev = kzalloc((sizeof(*g_bdev) * btmtk_intf_num), GFP_KERNEL);
	if (!g_bdev) {
		BTMTK_WARN("%s insufficient memory", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < btmtk_intf_num; i++) {
		g_bdev[i] = btmtk_allocate_dev_memory(NULL);
		if (g_bdev[i]) {
			/* BTMTK_STATE_UNKNOWN instead? */
			/* btmtk_set_chip_state(g_bdev[i], BTMTK_STATE_INIT); */

			/* BTMTK_FOPS_STATE_UNKNOWN instead? */
			btmtk_fops_set_state(g_bdev[i], BTMTK_FOPS_STATE_INIT);
		} else {
			return -ENOMEM;
		}
	}

	// Set global variable for btif interface
	g_sbdev = g_bdev[0];

	btmtk_main_info_initialize();

	return 0;
}

static int main_exit(void)
{
	int i = 0;

	BTMTK_INFO("%s releasing intf count <%d>", __func__, btmtk_intf_num);

	if (g_bdev == NULL) {
		BTMTK_WARN("%s g_data is NULL", __func__);
		return 0;
	}

	BTMTK_INFO("%s: Unregister reboot_notifier callback success.", __func__);
	/* Is it necessary? bt_close will be called by reboot. */
#if (USE_DEVICE_NODE != 1)
	unregister_reboot_notifier(&btmtk_reboot_notifier);
#endif // USE_DEVICE_NODE

	wakeup_source_unregister(main_info.fwdump_ws);
	wakeup_source_unregister(main_info.woble_ws);
	wakeup_source_unregister(main_info.eint_ws);

	for (i = 0; i < btmtk_intf_num; i++) {
		if (g_bdev[i] != NULL)
			btmtk_free_dev_memory(NULL, g_bdev[i]);
	}

	kfree(g_bdev);
	return 0;
}

/**
 * Kernel Module init/exit Functions
 */

int __init main_driver_init(void)
{
	int ret = 0;
	int i;

	/* Mediatek Driver Version */
	BTMTK_INFO("%s: MTK BT Driver Version : %s", __func__, VERSION);

	ret = main_init();
	if (ret < 0)
		return ret;

	for (i = 0; i < btmtk_intf_num; i++)
		btmtk_set_chip_state(g_bdev[i], BTMTK_STATE_DISCONNECT);

	ret = btmtk_cif_register();
	if (ret < 0) {
		BTMTK_ERR("*** USB registration failed(%d)! ***", ret);
		main_exit();
		return ret;
	}

	ret = btmtk_fops_initfwlog();
	if (ret < 0) {
		BTMTK_ERR("*** STPBTFWLOG registration failed(%d)! ***", ret);
		main_exit();
		return ret;
	}

	if (main_info.hif_hook.init)
		ret = main_info.hif_hook.init();

	BTMTK_INFO("%s: Done", __func__);
	return ret;
}

void main_driver_exit(void)
{
	BTMTK_INFO("%s", __func__);

	if (main_info.hif_hook.exit)
		main_info.hif_hook.exit();

	btmtk_fops_exitfwlog();
	btmtk_cif_deregister();

	main_exit();
}

#ifndef MTK_WCN_REMOVE_KERNEL_MODULE
module_init(main_driver_init);
module_exit(main_driver_exit);
#endif

/**
 * Module Common Information
 */
MODULE_DESCRIPTION("Mediatek Bluetooth Driver");
MODULE_VERSION(VERSION SUBVER);
MODULE_LICENSE("GPL");
module_param(btmtk_intf_num, int, 0444);
