/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


//---------------------------------------------------------------------------
#include <mtk-bt/LD_usbbt.h>
#include <mtk-bt/LD_btmtk_usb.h>
#include <mtk-bt/errno.h>
#include <drvPM.h>

/*============================================================================*/
/* Local Configuration */
/*============================================================================*/

#define LD_VERSION "4.0.20110301"

#define BUFFER_SIZE  (1024 * 4)	/* Size of RX Queue */
#define BT_SEND_HCI_CMD_BEFORE_SUSPEND 1
#define LD_SUPPORT_FW_DUMP 0
#define LD_BT_ALLOC_BUF 0
#define LD_NOT_FIX_BUILD_WARN 0

#define FIDX 0x5A	/* Unify WoBLE APCF Filtering Index */
#define BUZZARD_FIDX 0x0A	/* Unify WoBLE APCF Filtering Index */
#define FIDX_OFFSET_RADIO_OFF 19	/* Unify WoBLE APCF Filtering Index offset in radio_off cmd*/
#define FIDX_OFFSET_APCF 5	/* Unify WoBLE APCF Filtering Index offset in apcf related cmd*/

#define WOBLE_LOG_OFFSET_76XX 37
#define WOBLE_LOG_VALUE_76XX 0x31

/*============================================================================*/
/* Global Variable */
/*============================================================================*/
static char driver_version[64] = { 0 };
static unsigned char probe_counter = 0;
static volatile int metaMode;
static volatile int metaCount;
/* 0: False; 1: True */
static int isbtready;
static int isUsbDisconnet;
static volatile int is_assert = 0;
static u8 u8WoBTW = PM_SOURCE_DISABLE;

/*============================================================================*/
/* Extern Functions */
/*============================================================================*/
extern int snprintf(char *str, size_t size, const char *fmt, ...);

static inline int is_mt7630(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffff0000) == 0x76300000);
}

static inline int is_mt7650(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffff0000) == 0x76500000);
}

static inline int is_mt7632(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffff0000) == 0x76320000);
}

static inline int is_mt7662(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffff0000) == 0x76620000);
}

static inline int is_mt7662T(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffffffff) == 0x76620100);
}

static inline int is_mt7632T(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffffffff) == 0x76320100);
}

static inline int is_mt7668(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffff) == 0x7668);
}

static inline int is_mt7663(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffff) == 0x7663);
}

static inline int is_mt7961(struct LD_btmtk_usb_data *data)
{
	return ((data->chip_id & 0xffff) == 0x7961);
}

static inline int is_support_unify_woble(struct LD_btmtk_usb_data *data)
{
	if (data->bt_cfg.support_unify_woble) {
		if (is_mt7668(data) || is_mt7663(data) || is_mt7961(data))
			return 1;
		else
			return 0;
	} else {
		return 0;
	}
}

/*============================================================================*/
/* Internal Functions */
/*============================================================================*/
static int btmtk_usb_io_read32(struct LD_btmtk_usb_data *data, u32 reg, u32 *val)
{
	u8 request = data->r_request;
	int ret;

	ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_RX_EP, request,
			DEVICE_VENDOR_REQUEST_IN, 0, (u16)reg, data->io_buf, sizeof(u32),
			CONTROL_TIMEOUT_JIFFIES);

	if (ret < 0)
	{
		*val = 0xffffffff;
		usb_debug("error(%d), reg=%x, value=%x\n", ret, reg, *val);
		return ret;
	}

	os_memmove(val, data->io_buf, sizeof(u32));
	*val = le32_to_cpu(*val);
	return 0;
}

static int btmtk_usb_io_read32_7xxx(struct LD_btmtk_usb_data *data, u32 reg, u32 *val)
{
	int ret = -1;
	__le16 reg_high;
	__le16 reg_low;

	reg_high = ((reg >> 16) & 0xFFFF);
	reg_low = (reg & 0xFFFF);

	ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_RX_EP, 0x63,
			DEVICE_VENDOR_REQUEST_IN, reg_high, reg_low, data->io_buf, sizeof(u32),
			CONTROL_TIMEOUT_JIFFIES);
	if (ret < 0) {
		*val = 0xFFFFFFFF;
		usb_debug("error(%d), reg=%X, value=%X\n", ret, reg, *val);
		return ret;
	}

	os_memmove(val, data->io_buf, sizeof(u32));
	*val = le32_to_cpu(*val);
	return 0;
}

static int btmtk_usb_io_write32(struct LD_btmtk_usb_data *data, u32 reg, u32 val)
{
	u16 value, index;
	u8 request = data->w_request;
	mtkbt_dev_t *udev = data->udev;
	int ret;

	index = (u16) reg;
	value = val & 0x0000ffff;

	ret = data->hcif->usb_control_msg(udev, MTKBT_CTRL_TX_EP, request, DEVICE_VENDOR_REQUEST_OUT,
			value, index, NULL, 0, CONTROL_TIMEOUT_JIFFIES);

	if (ret < 0)
	{
		usb_debug("error(%d), reg=%x, value=%x\n", ret, reg, val);
		return ret;
	}

	index = (u16) (reg + 2);
	value = (val & 0xffff0000) >> 16;

	ret = data->hcif->usb_control_msg(udev, MTKBT_CTRL_TX_EP, request, DEVICE_VENDOR_REQUEST_OUT,
				value, index, NULL, 0, CONTROL_TIMEOUT_JIFFIES);

	if (ret < 0)
	{
		usb_debug("error(%d), reg=%x, value=%x\n", ret, reg, val);
		return ret;
	}
	if (ret > 0)
	{
		ret = 0;
	}
	return ret;
}

static int btmtk_usb_send_wmt_cmd(struct LD_btmtk_usb_data *data, const u8 *cmd,
		const int cmd_len, const u8 *event, const int event_len, u32 delay, u8 retry)
{
	int ret = -1;
	BOOL check = FALSE;

	if (!data || !data->hcif || !data->io_buf || !cmd) {
		usb_debug("incorrect cmd pointer\n");
		return -1;
	}
	if (event != NULL && event_len > 0)
		check = TRUE;

	/* send WMT command */
	ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_TX_EP, 0x01,
			DEVICE_CLASS_REQUEST_OUT, 0x30, 0x00, (void *)cmd, cmd_len,
			CONTROL_TIMEOUT_JIFFIES);
	if (ret < 0) {
		usb_debug("command send failed(%d)\n", ret);
		return ret;
	}

	if (event_len == -1) {
		/* If event_len is -1, DO NOT read event, since FW wouldn't feedback */
		return 0;
	}

retry_get:
	MTK_MDELAY(delay);

	/* check WMT event */
	ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_RX_EP, 0x01,
			DEVICE_VENDOR_REQUEST_IN, 0x30, 0x00, data->io_buf, LD_BT_MAX_EVENT_SIZE,
			CONTROL_TIMEOUT_JIFFIES);
	if (ret < 0) {
		usb_debug("event get failed(%d)\n", ret);
		if (check == TRUE)
			return ret;
		else
			return 0;
	}

	if (check == TRUE) {
		if (ret >= event_len && memcmp(event, data->io_buf, event_len) == 0) {
			return ret;
		} else if (retry > 0) {
			usb_debug("retry to get event(%d)\n", retry);
			retry--;
			goto retry_get;
		} else {
			usb_debug("can't get expect event\n");
			usb_debug_raw(event, event_len, "EXPECT:");
			usb_debug_raw(data->io_buf, ret, "RCV:");
		}
	} else {
		if (ret > 0) {
			usb_debug_raw(cmd, cmd_len, "CMD:");
			usb_debug_raw(data->io_buf, ret, "EVT:");
			return 0;
		} else if (retry > 0) {
			usb_debug("retry to get event(%d)\n", retry);
			retry--;
			goto retry_get;
		} else {
			usb_debug("can't get expect event\n");
		}
	}
	return -1;
}

static int btmtk_usb_send_hci_cmd(struct LD_btmtk_usb_data *data, u8 *cmd,
		const int cmd_len, const u8 *event, const int event_len)
{
	/** @RETURN
	 *	length if event compare successfully.,
	 *	0 if doesn't check event.,
	 *	< 0 if error.
	 */
#define USB_CTRL_IO_TIMO 100
#define USB_INTR_MSG_TIMO 2000
	int ret = -1;
	int len = 0;
	int i = 0;
	u8 retry = 0;
	BOOL check = FALSE;

	if (!data || !data->hcif || !data->io_buf || !cmd) {
		usb_debug("incorrect cmd pointer\n");
		return -1;
	}
	if (event != NULL && event_len > 0)
		check = TRUE;

	/* send HCI command */
	ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_TX_EP, 0,
			DEVICE_CLASS_REQUEST_OUT, 0, 0, (u8 *)cmd, cmd_len, USB_CTRL_IO_TIMO);
	if (ret < 0) {
		usb_debug("send command failed: %d\n", ret);
		return ret;
	}

	if (event_len == -1) {
		/* If event_len is -1, DO NOT read event, since FW wouldn't feedback */
		return 0;
	}

	/* check HCI event */
	do {
		memset(data->io_buf, 0, LD_BT_MAX_EVENT_SIZE);
		ret = data->hcif->usb_interrupt_msg(data->udev, MTKBT_INTR_EP, data->io_buf,
				LD_BT_MAX_EVENT_SIZE, &len, USB_INTR_MSG_TIMO);
		if (ret < 0) {
			usb_debug("event get failed: %d\n", ret);
			if (check == TRUE) return ret;
			else return 0;
		}

		if (check == TRUE) {
			if (len >= event_len) {
				for (i = 0; i < event_len; i++) {
					if (event[i] != data->io_buf[i])
						break;
				}
			} else {
				usb_debug("event length is not match(%d/%d)\n", len, event_len);
			}
			if (i != event_len) {
				usb_debug("got unknown event(%d)\n", len);
			} else {
				return len; /* actually read length */
			}
			MTK_MDELAY(10);
			++retry;
		}
		usb_debug("try get event again\n");
	} while (retry < 3);
	return -1;
}

static int btmtk_usb_send_hci_suspend_cmd(struct LD_btmtk_usb_data *data)
{
	int ret = -1;
#if SUPPORT_HISENSE_WoBLE
	u8 cmd[] = {0xC9, 0xFC, 0x02, 0x01, 0x0D}; // for Hisense WoBLE

	usb_debug("issue wake up command for Hisense\n");
#else
	u8 cmd[] = {0xC9, 0xFC, 0x0D, 0x01, 0x0E, 0x00, 0x05, 0x43,
		0x52, 0x4B, 0x54, 0x4D, 0x20, 0x04, 0x32, 0x00};

	usb_debug("issue wake up command for '0E: MTK WoBLE Ver2'\n");
#endif

	ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), NULL, -1);
	if (ret < 0) {
		usb_debug("error(%d)\n", ret);
		return ret;
	}
	usb_debug("send suspend cmd OK\n");
	return 0;
}

static int btmtk_usb_send_hci_reset_cmd(struct LD_btmtk_usb_data *data)
{
	u8 cmd[] = { 0x03, 0x0C, 0x00 };
	u8 event[] = { 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00 };
	int ret = -1;

	ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), event, sizeof(event));
	if (ret < 0) {
		usb_debug("failed(%d)\n", ret);
	} else {
		usb_debug("OK\n");
	}

	return ret;
}

static int btmtk_usb_send_hci_set_ce_cmd(struct LD_btmtk_usb_data *data)
{
	u8 cmd[] = { 0xD1, 0xFC, 0x04, 0x0C, 0x07, 0x41, 0x00 };
	u8 event[] = { 0x0E, 0x08, 0x01, 0xD1, 0xFC, 0x00 };
	int ret = -1;

	ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), event, sizeof(event));
	if (ret < 0) {
		usb_debug("failed(%d)\n", ret);

	} else if (ret == sizeof(event) + 4) {
		if (data->io_buf[6] & 0x01) {
			usb_debug("warning, 0x41070c[0] is 1!\n");
			ret = 0;
		} else {
			u8 cmd2[11] = { 0xD0, 0xFC, 0x08, 0x0C, 0x07, 0x41, 0x00 };

			cmd2[7] = data->io_buf[6] | 0x01;
			cmd2[8] = data->io_buf[7];
			cmd2[9] = data->io_buf[8];
			cmd2[10] = data->io_buf[9];

			ret = btmtk_usb_send_hci_cmd(data, cmd2, sizeof(cmd2), NULL, 0);
			if (ret < 0) {
				usb_debug("write 0x41070C failed(%d)\n", ret);
			} else {
				usb_debug("OK\n");
				ret = 0;
			}
		}
	} else {
		usb_debug("failed, incorrect response length(%d)\n", ret);
		return -1;
	}

	return ret;
}

static int btmtk_usb_send_check_rom_patch_result_cmd(struct LD_btmtk_usb_data *data)
{
	/* Send HCI Reset */
	{
		int ret = 0;
		unsigned char buf[8] = { 0 };
		buf[0] = 0xD1;
		buf[1] = 0xFC;
		buf[2] = 0x04;
		buf[3] = 0x00;
		buf[4] = 0xE2;
		buf[5] = 0x40;
		buf[6] = 0x00;
		ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_TX_EP,0x0, DEVICE_CLASS_REQUEST_OUT,
					0x00, 0x00, buf, 0x07, 100);
		if (ret < 0)
		{
			usb_debug("error1(%d)\n", ret);
			return ret;
		}
	}
	/* Get response of HCI reset */
	{
		int ret = 0;
		unsigned char buf[LD_BT_MAX_EVENT_SIZE] = { 0 };
		int actual_length = 0;
		ret = data->hcif->usb_interrupt_msg(data->udev, MTKBT_INTR_EP, buf, LD_BT_MAX_EVENT_SIZE,
				&actual_length, USB_INTR_MSG_TIMO);
		if (ret < 0)
		{
			usb_debug("error2(%d)\n", ret);
			return ret;
		}
		usb_debug("Check rom patch result : ");

		if (buf[6] == 0 && buf[7] == 0 && buf[8] == 0 && buf[9] == 0)
		{
			usb_debug("NG\n");
		}
		else
		{
			usb_debug("OK\n");
		}
	}
	return 0;
}

static int btmtk_usb_switch_iobase(struct LD_btmtk_usb_data *data, int base)
{
	int ret = 0;

	switch (base)
	{
		case SYSCTL:
			data->w_request = 0x42;
			data->r_request = 0x47;
			break;
		case WLAN:
			data->w_request = 0x02;
			data->r_request = 0x07;
			break;

		default:
			return -EINVAL;
	}

	return ret;
}

static void btmtk_usb_cap_init(struct LD_btmtk_usb_data *data)
{
	unsigned char *str_end ;

	usb_debug("chip id = %x\n", data->chip_id);

	if (data->chip_id == 0)
		btmtk_usb_io_read32(data, 0x00, &data->chip_id);
	if (data->chip_id == 0)
		btmtk_usb_io_read32_7xxx(data, 0x80000008, &data->chip_id);

	if (is_mt7630(data) || is_mt7650(data)) {
		data->need_load_fw = 1;
		data->need_load_rom_patch = 0;
		data->fw_header_image = NULL;
		data->fw_bin_file_name = (unsigned char*)strdup("mtk/mt7650.bin");
		data->fw_len = 0;

	} else if (is_mt7662T(data) || is_mt7632T(data)) {
		usb_debug("btmtk:This is 7662T chip\n");
		data->need_load_fw = 0;
		data->need_load_rom_patch = 1;
		os_memcpy(data->rom_patch_bin_file_name, "mt7662t_patch_e1_hdr.bin", 24);
		data->rom_patch_offset = 0xBC000;
		data->rom_patch_len = 0;

	} else if (is_mt7632(data) || is_mt7662(data)) {
		usb_debug("btmtk:This is 7662 chip\n");
		data->need_load_fw = 0;
		data->need_load_rom_patch = 1;
		os_memcpy(data->rom_patch_bin_file_name, "mt7662_patch_e3_hdr.bin", 23);
		data->rom_patch_offset = 0x90000;
		data->rom_patch_len = 0;

	} else if (is_mt7668(data) || is_mt7663(data)) {
		unsigned int fw_ver = 0;

		btmtk_usb_io_read32_7xxx(data, 0x80000004, &fw_ver);
		if ((fw_ver & 0xFF) != 0xFF) {
			data->need_load_fw = 0;
			data->need_load_rom_patch = 1;

			/* Bin filename format : "mt$$$$_patch_e%.bin"
			 *  $$$$ : chip id
			 *  % : fw version & 0xFF + 1 (in HEX)
			 */
			snprintf((char *)data->rom_patch_bin_file_name, MAX_BIN_FILE_NAME_LEN, "mt%04x_patch_e%x_hdr.bin",
					data->chip_id & 0xFFFF, (fw_ver & 0xFF) + 1);
			usb_debug("patch name: %s\n", data->rom_patch_bin_file_name);
			data->rom_patch_len = 0;
		} else {
			usb_debug("Incorrect firmware version: 0xFF");
			return;
		}

		if (is_mt7668(data)) {
			memcpy(data->woble_setting_file_name, WOBLE_SETTING_FILE_NAME_7668,
				sizeof(WOBLE_SETTING_FILE_NAME_7668));
			usb_debug("woble setting file name is %s\n", WOBLE_SETTING_FILE_NAME_7668);
		} else if (is_mt7663(data)) {
			memcpy(data->woble_setting_file_name, WOBLE_SETTING_FILE_NAME_7663,
				sizeof(WOBLE_SETTING_FILE_NAME_7663));
			usb_debug("woble setting file name is %s\n", WOBLE_SETTING_FILE_NAME_7663);
		} else {
			memcpy(data->woble_setting_file_name, WOBLE_SETTING_FILE_NAME,
				sizeof(WOBLE_SETTING_FILE_NAME));
			usb_debug("woble setting file name is %s\n", WOBLE_SETTING_FILE_NAME);
		}
	} else {
		btmtk_usb_io_read32_7xxx(data, BUZZARD_CHIP_ID, &data->chip_id);
		if (is_mt7961(data)) {
			btmtk_usb_io_read32_7xxx(data, BUZZARD_FLAVOR, &data->flavor);
			btmtk_usb_io_read32_7xxx(data, BUZZARD_FW_VERSION, &data->fw_version);
		} else {
			usb_debug("Unknown Mediatek device(%04X)\n", data->chip_id);
			return;
		}

		usb_debug("Chip ID = 0x%x\n", data->chip_id);
		usb_debug("flavor = 0x%x\n", data->flavor);
		usb_debug("FW Ver = 0x%x\n", data->fw_version);

		memset(data->rom_patch_bin_file_name, 0, MAX_BIN_FILE_NAME_LEN);
		if ((data->fw_version & 0xff) == 0xff) {
			usb_debug("Wrong FW version : 0x%x !", data->fw_version);
			return;
		}

		data->need_load_fw = 0;
		data->need_load_rom_patch = 1;
		/* Bin filename format : "BT_RAM_CODE_MT%04x_%x_%x_hdr.bin"
		 *  $$$$ : chip id
		 *  % : fw version & 0xFF + 1 (in HEX)
		 */
		data->flavor = (data->flavor & 0x00000080) >> 7;
		/* if flavor equals 1, it represent 7920, else it represent 7921 */
		if (data->flavor)
			snprintf((char *)data->rom_patch_bin_file_name, MAX_BIN_FILE_NAME_LEN,
				"BT_RAM_CODE_MT%04x_1a_%x_hdr.bin", data->chip_id & 0xffff,
				(data->fw_version & 0xff) + 1);
		else
			snprintf(data->rom_patch_bin_file_name, MAX_BIN_FILE_NAME_LEN,
				"BT_RAM_CODE_MT%04x_1_%x_hdr.bin",
				data->chip_id & 0xffff, (data->fw_version & 0xff) + 1);

		if(strlen((char *)data->rom_patch_bin_file_name) > MAX_BIN_FILE_NAME_LEN){
			str_end = data->rom_patch_bin_file_name + MAX_BIN_FILE_NAME_LEN;
			*str_end='\0';
		}
		usb_debug("patch name: %s\n", data->rom_patch_bin_file_name);
		data->rom_patch_len = 0;

		memcpy(data->woble_setting_file_name, WOBLE_SETTING_FILE_NAME_7961,
			sizeof(WOBLE_SETTING_FILE_NAME_7961));
		usb_debug("woble setting file name is %s", WOBLE_SETTING_FILE_NAME_7961);
	}
}

#if CRC_CHECK
static u16 checksume16(u8 *pData, int len)
{
	int sum = 0;

	while (len > 1)
	{
		sum += *((u16 *) pData);

		pData = pData + 2;

		if (sum & 0x80000000)
		{
			sum = (sum & 0xFFFF) + (sum >> 16);
		}
		len -= 2;
	}

	if (len)
		sum += *((u8 *) pData);

	while (sum >> 16)
	{
		sum = (sum & 0xFFFF) + (sum >> 16);
	}

	return ~sum;
}

static int btmtk_usb_chk_crc(struct LD_btmtk_usb_data *data, u32 checksum_len)
{
	int ret = 0;
	mtkbt_dev_t *udev = data->udev;

	usb_debug("\n");

	os_memmove(data->io_buf, &data->rom_patch_offset, 4);
	os_memmove(&data->io_buf[4], &checksum_len, 4);

	ret = data->hcif->usb_control_msg(udev, MTKBT_CTRL_TX_EP,0x1, DEVICE_VENDOR_REQUEST_OUT,
				0x20, 0x00, data->io_buf, 8, CONTROL_TIMEOUT_JIFFIES);

	if (ret < 0)
	{
		usb_debug("error(%d)\n", ret);
	}

	return ret;
}

static u16 btmtk_usb_get_crc(struct LD_btmtk_usb_data *data)
{
	int ret = 0;
	mtkbt_dev_t *udev = data->udev;
	u16 crc, count = 0;

	usb_debug("\n");

	while (1)
	{
		ret =
			data->hcif->usb_control_msg(udev, MTKBT_CTRL_RX_EP, 0x01, DEVICE_VENDOR_REQUEST_IN,
					0x21, 0x00, data->io_buf, 2, CONTROL_TIMEOUT_JIFFIES);

		if (ret < 0)
		{
			crc = 0xFFFF;
			usb_debug("error(%d)\n", ret);
		}

		os_memmove(&crc, data->io_buf, 2);

		crc = le16_to_cpu(crc);

		if (crc != 0xFFFF)
			break;

		MTK_MDELAY(100);

		if (count++ > 100)
		{
			usb_debug("Query CRC over %d times\n", count);
			break;
		}
	}

	return crc;
}
#endif /* CRC_CHECK */

static int btmtk_usb_send_wmt_reset_cmd(struct LD_btmtk_usb_data *data)
{
	/* reset command */
	u8 cmd[] = { 0x6F, 0xFC, 0x05, 0x01, 0x07, 0x01, 0x00, 0x04 };
	u8 event[] = { 0xE4, 0x05, 0x02, 0x07, 0x01, 0x00, 0x00 };
	int ret = -1;

	ret = btmtk_usb_send_wmt_cmd(data, cmd, sizeof(cmd), event, sizeof(event), 20, 0);
	if (ret < 0) {
		usb_debug("Check reset wmt result : NG\n");
	} else {
		usb_debug("Check reset wmt result : OK\n");
		ret = 0;
	}

	return ret;
}

static int btmtk_usb_send_wmt_cfg(struct LD_btmtk_usb_data *data)
{
	int ret = 0;
	int index = 0;

	usb_debug("send wmt cmd!\n");

	for (index = 0; index < WMT_CMD_COUNT; index++) {
		if (data->bt_cfg.wmt_cmd[index].content && data->bt_cfg.wmt_cmd[index].length) {
			ret = btmtk_usb_send_wmt_cmd(data, data->bt_cfg.wmt_cmd[index].content,
					data->bt_cfg.wmt_cmd[index].length, NULL, 0, 100, 10);
			if (ret < 0) {
				usb_debug("Send wmt cmd failed(%d)! Index: %d\n", ret, index);
				return ret;
			}
		}
	}

	return ret;
}

static u16 btmtk_usb_get_rom_patch_result(struct LD_btmtk_usb_data *data)
{
	int ret = 0;

	ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_RX_EP, 0x01,
				DEVICE_VENDOR_REQUEST_IN, 0x30, 0x00, data->io_buf, 7,
				CONTROL_TIMEOUT_JIFFIES);

	if (ret < 0)
	{
		usb_debug("error(%d)\n", ret);
	}

	if (data->io_buf[0] == 0xe4 &&
		data->io_buf[1] == 0x05 &&
		data->io_buf[2] == 0x02 &&
		data->io_buf[3] == 0x01 &&
		data->io_buf[4] == 0x01 &&
		data->io_buf[5] == 0x00 &&
		data->io_buf[6] == 0x00)
	{
		//usb_debug("Get rom patch result : OK\n");
	}
	else
	{
		usb_debug("Get rom patch result : NG\n");
	}
	return ret;
}

/**
 * Only for load rom patch function, tmp_str[15] is '\n'
 */
#define SHOW_FW_DETAILS(s)												\
	usb_debug("%s = %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", s,				\
			tmp_str[0], tmp_str[1], tmp_str[2], tmp_str[3],				\
			tmp_str[4], tmp_str[5], tmp_str[6], tmp_str[7],				\
			tmp_str[8], tmp_str[9], tmp_str[10], tmp_str[11],			\
			tmp_str[12], tmp_str[13], tmp_str[14], tmp_str[15])

static int btmtk_usb_load_rom_patch(struct LD_btmtk_usb_data *data)
{
	u32 loop = 0;
	u32 value;
	s32 sent_len;
	int ret = 0;
	u32 patch_len = 0;
	u32 cur_len = 0;
	int real_len = 0;
	int first_block = 1;
	unsigned char phase;
	void *buf;
	char *pos;
	unsigned char *tmp_str;

	//usb_debug("begin\n");
load_patch_protect:
	btmtk_usb_switch_iobase(data, WLAN);
	btmtk_usb_io_read32(data, SEMAPHORE_03, &value);
	loop++;

	if ((value & 0x01) == 0x00)
	{
		if (loop < 1000)
		{
			MTK_MDELAY(1);
			goto load_patch_protect;
		}
		else
		{
			usb_debug("btmtk_usb_load_rom_patch ERR! Can't get semaphore! Continue\n");
		}
	}

	btmtk_usb_switch_iobase(data, SYSCTL);

	btmtk_usb_io_write32(data, 0x1c, 0x30);

	btmtk_usb_switch_iobase(data, WLAN);

	/* check ROM patch if upgrade */
	if ((MT_REV_GTE(data, mt7662, REV_MT76x2E3)) || (MT_REV_GTE(data, mt7632, REV_MT76x2E3)))
	{
		btmtk_usb_io_read32(data, CLOCK_CTL, &value);
		if ((value & 0x01) == 0x01)
		{
			usb_debug("btmtk_usb_load_rom_patch : no need to load rom patch\n");
			btmtk_usb_send_hci_reset_cmd(data);
			goto error;
		}
	}
	else
	{
		btmtk_usb_io_read32(data, COM_REG0, &value);
		if ((value & 0x02) == 0x02)
		{
			usb_debug("btmtk_usb_load_rom_patch : no need to load rom patch\n");
			btmtk_usb_send_hci_reset_cmd(data);
			goto error;
		}
	}

	buf = os_kzalloc(UPLOAD_PATCH_UNIT, MTK_GFP_ATOMIC);
	if (!buf)
	{
		ret = -ENOMEM;
		goto error;
	}
	pos = buf;

	LD_load_code_from_bin(&data->rom_patch, (char *)data->rom_patch_bin_file_name, NULL,
			data->udev, &data->rom_patch_len);

	if (!data->rom_patch)
	{
		usb_debug("please assign a rom patch(/vendor/firmware/%s)or(/lib/firmware/%s)\n",
				data->rom_patch_bin_file_name,
				data->rom_patch_bin_file_name);
		ret = -1;
		goto error1;
	}

	tmp_str = data->rom_patch;
	SHOW_FW_DETAILS("FW Version");
	SHOW_FW_DETAILS("build Time");

	tmp_str = data->rom_patch + 16;
	usb_debug("platform = %c%c%c%c\n", tmp_str[0], tmp_str[1], tmp_str[2], tmp_str[3]);

	tmp_str = data->rom_patch + 20;
	usb_debug("HW/SW version = %c%c%c%c\n", tmp_str[0], tmp_str[1], tmp_str[2], tmp_str[3]);

	tmp_str = data->rom_patch + 24;
	usb_debug("Patch version = %c%c%c%c\n", tmp_str[0], tmp_str[1], tmp_str[2], tmp_str[3]);

	usb_debug("\nloading rom patch...\n");

	cur_len = 0x00;
	patch_len = data->rom_patch_len - PATCH_INFO_SIZE;

	/* loading rom patch */
	while (1)
	{
		s32 sent_len_max = UPLOAD_PATCH_UNIT - PATCH_HEADER_SIZE_BULK_EP;
		real_len = 0;
		sent_len =
			(patch_len - cur_len) >= sent_len_max ? sent_len_max : (patch_len - cur_len);

		//usb_debug("patch_len = %d\n", patch_len);
		//usb_debug("cur_len = %d\n", cur_len);
		//usb_debug("sent_len = %d\n", sent_len);

		if (sent_len > 0)
		{
			if (first_block == 1)
			{
				if (sent_len < sent_len_max)
					phase = PATCH_PHASE3;
				else
					phase = PATCH_PHASE1;
				first_block = 0;
			}
			else if (sent_len == sent_len_max)
			{
				if (patch_len - cur_len == sent_len_max)
					phase = PATCH_PHASE3;
				else
					phase = PATCH_PHASE2;
			}
			else
			{
				phase = PATCH_PHASE3;
			}

			/* prepare HCI header */
			pos[0] = 0x6F;
			pos[1] = 0xFC;
			pos[2] = (sent_len + 5) & 0xFF;
			pos[3] = ((sent_len + 5) >> 8) & 0xFF;

			/* prepare WMT header */
			pos[4] = 0x01;
			pos[5] = 0x01;
			pos[6] = (sent_len + 1) & 0xFF;
			pos[7] = ((sent_len + 1) >> 8) & 0xFF;

			pos[8] = phase;

			os_memcpy(&pos[9], data->rom_patch + PATCH_INFO_SIZE + cur_len, sent_len);

			//usb_debug("sent_len + PATCH_HEADER_SIZE = %d, phase = %d\n",
					//sent_len + PATCH_HEADER_SIZE, phase);

			ret = data->hcif->usb_bulk_msg(data->udev, MTKBT_BULK_TX_EP, buf, sent_len + PATCH_HEADER_SIZE_BULK_EP, &real_len, 0);

			if (ret)
			{
				usb_debug("upload rom_patch err: %d\n", ret);
				goto error1;
			}

			MTK_MDELAY(1);

			cur_len += sent_len;

		}
		else
		{
			usb_debug("loading rom patch... Done\n");
			break;
		}
	}

	MTK_MDELAY(20);
	ret = btmtk_usb_get_rom_patch_result(data);
	MTK_MDELAY(20);

	/* Send Checksum request */
	#if CRC_CHECK
	int total_checksum = checksume16(data->rom_patch + PATCH_INFO_SIZE, patch_len);
	btmtk_usb_chk_crc(data, patch_len);
	MTK_MDELAY(20);
	if (total_checksum != btmtk_usb_get_crc(data))
	{
		usb_debug("checksum fail!, local(0x%x) <> fw(0x%x)\n", total_checksum,
				btmtk_usb_get_crc(data));
		ret = -1;
		goto error1;
	}
	else
	{
		usb_debug("crc match!\n");
	}
	#endif
	MTK_MDELAY(20);
	/* send check rom patch result request */
	btmtk_usb_send_check_rom_patch_result_cmd(data);
	MTK_MDELAY(20);
	/* CHIP_RESET */
	ret = btmtk_usb_send_wmt_reset_cmd(data);
	MTK_MDELAY(20);
	/* BT_RESET */
	btmtk_usb_send_hci_reset_cmd(data);
	/* for WoBLE/WoW low power */
	btmtk_usb_send_hci_set_ce_cmd(data);

error1:
	os_kfree(buf);

error:
	btmtk_usb_io_write32(data, SEMAPHORE_03, 0x1);
	//usb_debug("end\n");
	return ret;
}

static int btmtk_usb_send_wmt_power_on_cmd_7668(struct LD_btmtk_usb_data *data)
{
	u8 count = 0;	/* retry 3 times */
	u8 cmd[] = { 0x6F, 0xFC, 0x06, 0x01, 0x06, 0x02, 0x00, 0x00, 0x01 };
	u8 event[] = { 0xE4, 0x05, 0x02, 0x06, 0x01, 0x00 };	/* event[6] is key */
	int ret = -1;	/* if successful, 0 */

	do {
		ret = btmtk_usb_send_wmt_cmd(data, cmd, sizeof(cmd), event, sizeof(event), 100, 10);
		if (ret < 0) {
			usb_debug("failed(%d)\n", ret);
		} else if (ret == sizeof(event) + 1) {
			switch (data->io_buf[6]) {
			case 0:		/* successful */
				usb_debug("OK\n");
				ret = 0;
				break;
			case 2:		/* retry */
				usb_debug("Try again\n");
				continue;
			default:
				usb_debug("Unknown result: %02X\n", data->io_buf[6]);
				return -1;
			}
		} else {
			usb_debug("failed, incorrect response length(%d)\n", ret);
			return -1;
		}
	} while (++count < 3 && ret > 0);

	return ret;
}

static int btmtk_usb_send_hci_tci_set_sleep_cmd_7668(struct LD_btmtk_usb_data *data)
{
	u8 cmd[] = { 0x7A, 0xFC, 0x07, 0x05, 0x40, 0x06, 0x40, 0x06, 0x00, 0x00 };
	u8 event[] = { 0x0E, 0x04, 0x01, 0x7A, 0xFC, 0x00 };
	int ret = -1;	/* if successful, 0 */

	ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), event, sizeof(event));
	if (ret < 0) {
		usb_debug("failed(%d)\n", ret);
	} else {
		usb_debug("OK\n");
		ret = 0;
	}

	return ret;
}

static int btmtk_usb_get_vendor_cap(struct LD_btmtk_usb_data *data)
{
	u8 cmd[] = { 0x53, 0xFD, 0x00 };
	u8 event[6] = { 0x0E, 0x12, 0x01, 0x53, 0xFD, 0x00, /* ... */ };
	int ret = -1;

	// TODO: should not compare whole event
	ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), event, sizeof(event));
	if (ret < 0) {
		usb_debug("Failed(%d)\n", ret);
	} else {
		usb_debug("OK\n");
		ret = 0;
	}

	return ret;
}

static int btmtk_usb_send_apcf_reserved_79xx(struct LD_btmtk_usb_data *data)
{
	int ret = 0;
	u8 reserve_apcf_cmd[] = { 0xC9, 0xFC, 0x05, 0x01, 0x30, 0x02, 0x61, 0x02 };
	u8 reserve_apcf_event[] = { 0xE6, 0x02, 0x08, 0x11 };

	ret = btmtk_usb_send_hci_cmd(data, reserve_apcf_cmd, sizeof(reserve_apcf_cmd),
		reserve_apcf_event, sizeof(reserve_apcf_event));
	if (ret < 0 ) {
		usb_debug("Failed(%d)\n", ret);
		return ret;
	}

	return ret;
}

static int btmtk_usb_send_read_bdaddr(struct LD_btmtk_usb_data *data)
{
	u8 cmd[] = { 0x09, 0x10, 0x00 };
	u8 event[] = { 0x0E, 0x0A, 0x01, 0x09, 0x10, 0x00, /* 6 bytes are BDADDR */ };
	int ret = -1;

	ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), event, sizeof(event));
	if (ret < 0 || ret != 12 /* Event actual length */) {
		usb_debug("Failed(%d)\n", ret);
		return ret;
	}

	os_memcpy(data->local_addr, data->io_buf + 6, BD_ADDR_LEN);
	usb_debug("ADDR: %02X-%02X-%02X-%02X-%02X-%02X\n",
			data->local_addr[5], data->local_addr[4], data->local_addr[3],
			data->local_addr[2], data->local_addr[1], data->local_addr[0]);
	ret = 0;

	return ret;
}

static int btmtk_usb_set_apcf(struct LD_btmtk_usb_data *data, BOOL bin_file)
{
	int i = 0, ret = -1;
	// Legacy RC pattern
	u8 manufacture_data[] = { 0x57, 0xFD, 0x27, 0x06, 0x00, FIDX,
		0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x43, 0x52, 0x4B, 0x54, 0x4D,   /* manufacturer data */
		0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; /* mask */
	u8 filter_cmd[] = { 0x57, 0xFD, 0x0A, 0x01, 0x00, FIDX, 0x20, 0x00,
		0x00, 0x00, 0x01, 0x80, 0x00 };
	u8 event[] = { 0x0E, 0x07, 0x01, 0x57, 0xFD, 0x00, /* ... */ };

	if (is_mt7961(data)) {
		manufacture_data[FIDX_OFFSET_APCF] = BUZZARD_FIDX;
		filter_cmd[FIDX_OFFSET_APCF] = BUZZARD_FIDX;
	}

	if (bin_file) {
		if (data->wake_dev_len) {
			/* wake_on_ble.conf using 90(0x5A-FIDX) as filter_index */
			u8 pos = 0;
			u8 broadcast_addr[] = { 0x57, 0xFD, 0x0A, 0x02, 0x00, FIDX,
				0x55, 0x44, 0x33, 0x22, 0x11, 0x00,	// ADDRESS
				0x00 };	// 0: Public, 1: Random
			u8 adv_pattern[] = { 0x57, 0xFD, 0x15, 0x06, 0x00, FIDX,
				0x71, 0x01,				// VID
				0x04, 0x11,				// PID
				0x00, 0x00, 0x00, 0x00,	// IR key code
				0x00,					// sequence number
				0xFF, 0xFF,				// mask~
				0xFF, 0xFF,
				0x00, 0x00, 0x00, 0x00,
				0x00};

			// BDADDR
			for (i = 0; i < data->wake_dev[1]; i++) {
				broadcast_addr[11] = data->wake_dev[2 + i * BD_ADDR_LEN + 0];
				broadcast_addr[10] = data->wake_dev[2 + i * BD_ADDR_LEN + 1];
				broadcast_addr[9] = data->wake_dev[2 + i * BD_ADDR_LEN + 2];
				broadcast_addr[8] = data->wake_dev[2 + i * BD_ADDR_LEN + 3];
				broadcast_addr[7] = data->wake_dev[2 + i * BD_ADDR_LEN + 4];
				broadcast_addr[6] = data->wake_dev[2 + i * BD_ADDR_LEN + 5];
				ret = btmtk_usb_send_hci_cmd(data, broadcast_addr, sizeof(broadcast_addr),
						event, sizeof(event));
				if (ret < 0) {
					usb_debug("Set broadcast address fail\n");
					continue;
				}
				// mask broadcast address as a filter condition
				filter_cmd[6] = 0x21;
			}
			usb_debug("There are %d broadcast address filter(s) from %s\n", i, WAKE_DEV_RECORD);

			/** VID/PID in conf is LITTLE endian, but PID in ADV is BIG endian */
			pos = 2 + data->wake_dev[1] * 6;
			for (i = 0; i < data->wake_dev[pos]; i++) {
				adv_pattern[6] = data->wake_dev[pos + (i * 4) + 1];
				adv_pattern[7] = data->wake_dev[pos + (i * 4) + 2];
				adv_pattern[9] = data->wake_dev[pos + (i * 4) + 3];
				adv_pattern[8] = data->wake_dev[pos + (i * 4) + 4];
				ret = btmtk_usb_send_hci_cmd(data, adv_pattern, sizeof(adv_pattern),
						event, sizeof(event));
				if (ret < 0) {
					usb_debug("Set advertising patten fail\n");
					return ret;
				}
			}
			usb_debug("There are %d manufacture data filter(s) from %s\n", i, WAKE_DEV_RECORD);

			// Filtering parameters
			ret = btmtk_usb_send_hci_cmd(data, filter_cmd, sizeof(filter_cmd),
					event, sizeof(event));
			if (ret < 0) {
				usb_debug("Set filtering parm fail\n");
				return ret;
			}

		// if wake_on_ble.conf exist, no need use default woble_setting.bin
		} else {
			// woble_setting.bin
			usb_debug("Set APCF filter from woble_setting.bin\n");
			for (i = 0; i < WOBLE_SETTING_COUNT; i++) {
				if (!data->woble_setting_apcf[i].length)
					continue;

				if ((data->woble_setting_apcf_fill_mac[i].content[0] == 1) &&
					data->woble_setting_apcf_fill_mac_location[i].length) {
					/* need add BD addr to apcf cmd */
					memcpy(data->woble_setting_apcf[i].content +
						(*data->woble_setting_apcf_fill_mac_location[i].content),
						data->local_addr, BD_ADDR_LEN);
				}

				usb_debug_raw(data->woble_setting_apcf[i].content,
						data->woble_setting_apcf[i].length,
						"Send woble_setting_apcf[%d] ", i);

				ret = btmtk_usb_send_hci_cmd(data, data->woble_setting_apcf[i].content,
						data->woble_setting_apcf[i].length, event, sizeof(event));
				if (ret < 0) {
					usb_debug("Set apcf_cmd[%d] data fail\n", i);
					return ret;
				}
			}
		}
	} else {
		// Use default
		usb_debug("Using default APCF filter\n");
		os_memcpy(manufacture_data + 9, data->local_addr, BD_ADDR_LEN);
		ret = btmtk_usb_send_hci_cmd(data, manufacture_data,
				sizeof(manufacture_data), event, sizeof(event));
		if (ret < 0) {
			usb_debug("Set manufacture data fail\n");
			return ret;
		}

		ret = btmtk_usb_send_hci_cmd(data, filter_cmd, sizeof(filter_cmd),
				event, sizeof(event));
		if (ret < 0) {
			usb_debug("Set manufacture data fail\n");
			return ret;
		}
	}
	return 0;
}

static int btmtk_usb_check_need_load_patch_7668(struct LD_btmtk_usb_data *data)
{
	u8 cmd[] = { 0x6F, 0xFC, 0x05, 0x01, 0x17, 0x01, 0x00, 0x01 };
	u8 event[] = { 0xE4, 0x05, 0x02, 0x17, 0x01, 0x00, /* 0x02 */ };	/* event[6] is key */
	int ret = -1;

	ret = btmtk_usb_send_wmt_cmd(data, cmd, sizeof(cmd), event, sizeof(event), 20, 0);
	/* can't get correct event */
	if (ret < 0) {
		usb_debug("check need load patch or not fail(%d)\n", ret);
		return PATCH_ERR;
	}

	if (ret >= sizeof(event) + 1) {
		usb_debug("%s: return len is %d\n", __func__, ret);
		return data->io_buf[6];
	}

	return PATCH_ERR;
}

static int btmtk_usb_load_partial_rom_patch_7668(struct LD_btmtk_usb_data *data,
		u32 patch_len, int offset)
{
	u8 *pos = NULL;
	u8 phase = 0;
	s32 sent_len = 0;
	u32 cur_len = 0;
	int real_len = 0;
	int first_block = 1;
	int ret = 0;
	void *buf = NULL;

	buf = os_kzalloc(UPLOAD_PATCH_UNIT, MTK_GFP_ATOMIC);
	if (!buf) {
		return -ENOMEM;
	}
	pos = buf;

	/* loading rom patch */
	while (1) {
		s32 sent_len_max = UPLOAD_PATCH_UNIT - PATCH_HEADER_SIZE_BULK_EP;

		real_len = 0;
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
			pos[0] = 0x6F;
			pos[1] = 0xFC;
			pos[2] = (sent_len + 5) & 0xFF;
			pos[3] = ((sent_len + 5) >> 8) & 0xFF;

			/* prepare WMT header */
			pos[4] = 0x01;
			pos[5] = 0x01;
			pos[6] = (sent_len + 1) & 0xFF;
			pos[7] = ((sent_len + 1) >> 8) & 0xFF;

			pos[8] = phase;

			os_memcpy(&pos[9], data->rom_patch + offset + cur_len, sent_len);
			//usb_debug("sent_len = %d, cur_len = %d, phase = %d\n", sent_len, cur_len, phase);

			ret = data->hcif->usb_bulk_msg(data->udev, MTKBT_BULK_TX_EP, buf,
					sent_len + PATCH_HEADER_SIZE_BULK_EP, &real_len, 0);
			if (ret) {
				usb_debug("upload rom_patch err: %d\n", ret);
				ret = -1;
				goto free;
			}
			cur_len += sent_len;
			MTK_MDELAY(1);
			btmtk_usb_get_rom_patch_result(data);
			MTK_MDELAY(1);

		} else {
			usb_debug("loading rom patch... Done\n");
			break;
		}
		os_memset(buf, 0, UPLOAD_PATCH_UNIT);
	}
free:
	os_kfree(buf);
	buf = NULL;

	return ret;
}

static int btmtk_usb_load_rom_patch_7668(struct LD_btmtk_usb_data *data)
{
	int ret = 0;
	int patch_status = 0;
	int retry = 20;
	unsigned char *tmp_str = NULL;
	BOOL sysram3 = FALSE;
	u32 patch_len = 0;

	LD_load_code_from_bin(&data->rom_patch, (char *)data->rom_patch_bin_file_name,
			NULL, data->udev, &data->rom_patch_len);
	if (!data->rom_patch || !data->rom_patch_len) {
		usb_debug("please assign a rom patch from (/etc/firmware/%s) or (/lib/firmware/%s)\n",
				data->rom_patch_bin_file_name, data->rom_patch_bin_file_name);
		return -1;
	}

	if (is_mt7668(data))
		sysram3 = data->rom_patch_len > (PATCH_INFO_SIZE + PATCH_LEN_ILM) ? TRUE : FALSE;

	do {
		patch_status = btmtk_usb_check_need_load_patch_7668(data);
		usb_debug("patch_status: %d, retry: %d\n", patch_status, retry);
		if (patch_status > PATCH_NEED_DOWNLOAD || patch_status == PATCH_ERR) {
			usb_debug("%s: patch_status error\n", __func__);
			return -1;
		} else if (patch_status == PATCH_READY) {
			if (sysram3 == TRUE) {
				usb_debug("%s: Prepare to load sysram3\n", __func__);
				goto sysram;
			}
			usb_debug("%s: No need to load ROM patch\n", __func__);
			return 0;
		} else if (patch_status == PATCH_IS_DOWNLOAD_BY_OTHER) {
			MTK_MDELAY(100);
			retry--;
		} else if (patch_status == PATCH_NEED_DOWNLOAD) {
			if (is_mt7663(data)) {
				ret = btmtk_usb_send_wmt_cfg(data);
				if (ret < 0) {
					usb_debug("send wmt cmd failed(%d)\n", ret);
					return ret;
				}
			}
			break; /* Download ROM patch directly */
		}
	} while (retry > 0);

	if (patch_status == PATCH_IS_DOWNLOAD_BY_OTHER) {
		usb_debug("Hold by another fun more than 2 seconds");
		return -1;
	}

	tmp_str = data->rom_patch;
	SHOW_FW_DETAILS("FW Version");
	SHOW_FW_DETAILS("build Time");

	tmp_str = data->rom_patch + 16;
	usb_debug("platform = %c%c%c%c\n", tmp_str[0], tmp_str[1], tmp_str[2], tmp_str[3]);

	tmp_str = data->rom_patch + 20;
	usb_debug("HW/SW version = %c%c%c%c\n", tmp_str[0], tmp_str[1], tmp_str[2], tmp_str[3]);

	tmp_str = data->rom_patch + 24;

	usb_debug("%s: loading rom patch of ILM\n", __func__);
	patch_len = sysram3 ? PATCH_LEN_ILM : (data->rom_patch_len - PATCH_INFO_SIZE);
	ret = btmtk_usb_load_partial_rom_patch_7668(data, patch_len, PATCH_INFO_SIZE);
	if (ret < 0)
			return ret;

	/* CHIP_RESET, ROM patch would be reactivated.
	 * Currently, wmt reset is only for ILM rom patch, and there are also
	 * some preparations need to be done in FW for loading sysram3 patch...
	 */
	MTK_MDELAY(20);
	ret = btmtk_usb_send_wmt_reset_cmd(data);
	if (ret < 0)
		return ret;
	MTK_MDELAY(20);

sysram:
	if (sysram3) {
		usb_debug("%s: loading rom patch of sysram3\n", __func__);
		patch_len = data->rom_patch_len - PATCH_INFO_SIZE - PATCH_LEN_ILM - PATCH_INFO_SIZE;
		ret = btmtk_usb_load_partial_rom_patch_7668(data, patch_len,
				PATCH_INFO_SIZE + PATCH_LEN_ILM + PATCH_INFO_SIZE);
	}
	return ret;
}

static void btmtk_print_bt_patch_info(struct LD_btmtk_usb_data *data)
{
	struct _PATCH_HEADER *patchHdr = NULL;
	struct _Global_Descr *globalDesrc = NULL;

	if (data->rom_patch == NULL) {
		usb_debug("data->rom_patch is NULL!\n");
		return;
	}

	patchHdr = (struct _PATCH_HEADER *)data->rom_patch;

	if (is_mt7961(data))
		globalDesrc = (struct _Global_Descr *)(data->rom_patch + FW_ROM_PATCH_HEADER_SIZE);

	usb_debug("[btmtk] =============== Patch Info ==============\n");
	if (patchHdr) {
		usb_debug("[btmtk] Built Time = %s\n", patchHdr->ucDateTime);
		usb_debug("[btmtk] Hw Ver = 0x%04x\n", patchHdr->u2HwVer);
		usb_debug("[btmtk] Sw Ver = 0x%04x\n", patchHdr->u2SwVer);
		usb_debug("[btmtk] Magic Number = 0x%08x\n", patchHdr->u4MagicNum);

		usb_debug("[btmtk] Platform = %c%c%c%c\n",
				patchHdr->ucPlatform[0],
				patchHdr->ucPlatform[1],
				patchHdr->ucPlatform[2],
				patchHdr->ucPlatform[3]);
	} else
		usb_debug("patchHdr is NULL!\n");

	if (globalDesrc) {
		usb_debug("[btmtk] Patch Ver = 0x%08x\n", globalDesrc->u4PatchVer);
		usb_debug("[btmtk] Section num = 0x%08x\n", globalDesrc->u4SectionNum);
	} else
		usb_debug("globalDesrc is NULL!\n");
	usb_debug("[btmtk] =========================================\n");
}

int btmtk_cif_send_control_out(struct LD_btmtk_usb_data *data,
		const uint8_t *cmd, const int cmd_len, int delay, int retry)
{
	int ret = 0;

	if (data == NULL || data->udev == NULL || data->io_buf == NULL || cmd == NULL) {
		usb_debug("%s: incorrect cmd pointer\n", __func__);
		ret = -1;
		return ret;
	}

	/* send WMT command */
	ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_TX_EP, 0x01,
			DEVICE_CLASS_REQUEST_OUT, 0x30, 0x00, (void *)cmd, cmd_len,
			CONTROL_TIMEOUT_JIFFIES);
	if (ret < 0) {
		usb_debug("command send failed(%d)\n", ret);
		return ret;
	}

	return ret;
}

int btmtk_cif_send_bulk_out(struct LD_btmtk_usb_data *data,
		const uint8_t *cmd, const int cmd_len)
{
	int ret = 0;
	int real_len = 0;

	if (data == NULL || data->udev == NULL || cmd == NULL) {
		usb_debug("%s: incorrect cmd pointer\n", __func__);
		ret = -1;
		return ret;
	}

	ret = data->hcif->usb_bulk_msg(data->udev, MTKBT_BULK_TX_EP, cmd, cmd_len, &real_len, 0);
	if (ret < 0) {
		usb_debug("command send failed(%d)\n", ret);
		return ret;
	}

	return ret;
}

int btmtk_cif_send_cmd(struct LD_btmtk_usb_data *data, const uint8_t *cmd,
		const int cmd_len, int delay, int retry, int endpoint)
{
	int ret = -1;

	if (endpoint == BTMTK_EP_TYPE_OUT_CMD) {
		/* handle wmt cmd from driver */
		ret = btmtk_cif_send_control_out(data, cmd, cmd_len,
				delay, retry);
	} else if (endpoint == BTMTK_EP_TPYE_OUT_ACL) {
		/* bulk out for load rom patch*/
		ret = btmtk_cif_send_bulk_out(data, cmd, cmd_len);
	}

	return ret;
}

int btmtk_cif_recv_evt(struct LD_btmtk_usb_data *data, int delay, int retry)
{
	int ret = -1;	/* if successful, 0 */

	if (!data) {
		usb_debug("%s: data == NULL!\n", __func__);
		return ret;
	}

	if (!data->udev) {
		usb_debug("%s: invalid parameters!\n", __func__);
		return ret;
	}

get_response_again:
	/* ms delay */
	MTK_MDELAY(delay);

	/* check WMT event */
	memset(data->io_buf, 0, LD_BT_MAX_EVENT_SIZE);
	ret = data->hcif->usb_control_msg(data->udev, MTKBT_CTRL_RX_EP,
			0x01, DEVICE_VENDOR_REQUEST_IN, 0x30, 0x00, data->io_buf,
			LD_BT_MAX_EVENT_SIZE, CONTROL_TIMEOUT_JIFFIES);

	if (ret < 0) {
		usb_debug("%s: event get failed(%d)\n", __func__, ret);
		return ret;
	}

	if (ret > 0) {
		//usb_debug_raw(data->io_buf, ret, "%s OK: EVT:", __func__);
		return ret; /* return read length */
	} else if (retry > 0) {
		usb_debug("%s: Trying to get response... (%d)\n", __func__, ret);
		retry--;
		goto get_response_again;
	} else
		usb_debug("%s NG: do not got response:(%d)\n", __func__, ret);

	return -1;
}

int btmtk_compare_evt(struct LD_btmtk_usb_data *data, const uint8_t *event,
		int event_len, int recv_evt_len)
{
	int ret = -1;

	if (data && data->io_buf && event && recv_evt_len >= event_len) {
		if (memcmp(data->io_buf, event, event_len) == 0) {
			ret = 0;
			goto exit;
		} else {
			usb_debug("%s compare fail\n", __func__);
			usb_debug_raw(event, event_len, "%s: event_need_compare:", __func__);
			usb_debug_raw(data->io_buf, recv_evt_len, "%s: RCV:", __func__);
			goto exit;
		}
	} else
		usb_debug("%s invalid parameter!\n", __func__);

exit:
	return ret;
}

int btmtk_main_send_cmd(struct LD_btmtk_usb_data *data, const uint8_t *cmd,
		const int cmd_len, const uint8_t *event, const int event_len, int delay,
		int retry, int endpoint)
{
	int ret = 0;

	if (data == NULL || cmd == NULL) {
		usb_debug("%s, invalid parameters!\n", __func__);
		ret = -EINVAL;
		return ret;
	}

	ret = btmtk_cif_send_cmd(data, cmd, cmd_len, delay, retry, endpoint);
	if (ret < 0) {
		usb_debug("%s btmtk_cif_send_cmd failed!!\n", __func__);
		return ret;
	}

	/* wmt cmd and download fw patch using wmt cmd with USB interface, need use
	 * usb_control_msg to recv wmt event;
	 */
	if (event && (endpoint == BTMTK_EP_TYPE_OUT_CMD || endpoint == BTMTK_EP_TPYE_OUT_ACL)) {
		data->recv_evt_len = btmtk_cif_recv_evt(data, delay, retry);
		if (data->recv_evt_len < 0) {
			usb_debug("%s btmtk_cif_recv_evt failed!!\n", __func__);
			ret = -1;
			return ret;
		}
		ret = btmtk_compare_evt(data, event, event_len, data->recv_evt_len);
	}

	return ret;
}

static int btmtk_send_wmt_download_cmd(struct LD_btmtk_usb_data *data,
		u8 *cmd, int cmd_len, u8 *event, int event_len, struct _Section_Map *sectionMap,
		u8 fw_state, u8 dma_flag)
{
	int payload_len = 0;
	int ret = -1;

	if (data == NULL || cmd == NULL || event == NULL || sectionMap == NULL) {
		usb_debug("%s: invalid parameter!", __func__);
		return ret;
	}

	/* need refine this cmd to mtk_wmt_hdr struct*/
	/* prepare HCI header */
	cmd[0] = 0x6F;
	cmd[1] = 0xFC;

	/* prepare WMT header */
	cmd[3] = 0x01;
	cmd[4] = 0x01; /* opcode */

	if (fw_state == 0) {
		/* prepare WMT DL cmd */
		payload_len = SEC_MAP_NEED_SEND_SIZE + 2;

		cmd[2] = (payload_len + WMT_HEADER_LEN) & 0xFF; /* length*/
		cmd[5] = payload_len & 0xFF;
		cmd[6] = (payload_len >> 8) & 0xFF;
		cmd[7] = 0x00; /* which is the FW download state 0 */
		cmd[8] = dma_flag; /* 1:using DMA to download, 0:using legacy wmt cmd*/
		cmd_len = SEC_MAP_NEED_SEND_SIZE + PATCH_HEADER_SIZE_BULK_EP;

		memcpy(&cmd[PATCH_HEADER_SIZE_BULK_EP], (u8 *)(sectionMap->u4SecSpec), SEC_MAP_NEED_SEND_SIZE);

		ret = btmtk_main_send_cmd(data, cmd, cmd_len,
				event, event_len, PATCH_DOWNLOAD_CMD_DELAY_TIME, PATCH_DOWNLOAD_CMD_RETRY, BTMTK_EP_TYPE_OUT_CMD);
		if (ret < 0) {
			usb_debug("%s: send wmd dl cmd failed, terminate!\n", __func__);
			return PATCH_ERR;
		}

		if (data->recv_evt_len >= event_len)
			return data->io_buf[PATCH_STATUS];

		return PATCH_ERR;
	} else
		usb_debug("%s: fw state is error!\n", __func__);

	return ret;
}

static int btmtk_load_fw_patch_using_wmt_cmd(struct LD_btmtk_usb_data *data,
		u8 *image, u8 *event, int event_len, u32 patch_len, int offset)
{
/* Using Bulk EP will cause Buzzard no response
 * Need check later
 */
#define DL_PATCH_BY_BULK_EP 0
	int ret = 0;
	u32 cur_len = 0;
	s32 sent_len;
	int first_block = 1;
	u8 phase;
	int delay = PATCH_DOWNLOAD_PHASE1_2_DELAY_TIME;
	int retry = PATCH_DOWNLOAD_PHASE1_2_RETRY;
#if DL_PATCH_BY_BULK_EP
	s32 sent_hdr_len = PATCH_HEADER_SIZE_BULK_EP;
	int ep = BTMTK_EP_TPYE_OUT_ACL;
#else
	s32 sent_hdr_len = PATCH_HEADER_SIZE_CTRL_EP;
	int ep = BTMTK_EP_TYPE_OUT_CMD;
#endif

	if (data == NULL || image == NULL || data->rom_patch == NULL) {
		usb_debug("%s, invalid parameters!\n", __func__);
		ret = -1;
		goto exit;
	}
	/* loading rom patch */
	while (1) {
		s32 sent_len_max = UPLOAD_PATCH_UNIT - sent_hdr_len;

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

#if DL_PATCH_BY_BULK_EP
			/* prepare HCI header */
			image[0] = 0x6F;
			image[1] = 0xFC;
			image[2] = (sent_len + LOAD_PATCH_PHASE_LEN + WMT_HEADER_LEN) & 0xFF;
			image[3] = ((sent_len + LOAD_PATCH_PHASE_LEN + WMT_HEADER_LEN) >> 8) & 0xFF;

			/* prepare WMT header */
			image[4] = 0x01;
			image[5] = 0x01;
			image[6] = (sent_len + LOAD_PATCH_PHASE_LEN) & 0xFF;
			image[7] = ((sent_len + LOAD_PATCH_PHASE_LEN) >> 8) & 0xFF;

			image[8] = phase;
#else
			/* prepare HCI header */
			image[0] = 0x6F;
			image[1] = 0xFC;
			image[2] = sent_len + LOAD_PATCH_PHASE_LEN + WMT_HEADER_LEN;

			/* prepare WMT header */
			image[3] = 0x01;
			image[4] = 0x01;
			image[5] = (sent_len + LOAD_PATCH_PHASE_LEN) & 0xFF;
			image[6] = ((sent_len + LOAD_PATCH_PHASE_LEN) >> 8) & 0xFF;

			image[7] = phase;
#endif
			memcpy(&image[sent_hdr_len], data->rom_patch + offset + cur_len, sent_len);
			if (phase == PATCH_PHASE3) {
				delay = PATCH_DOWNLOAD_PHASE3_DELAY_TIME;
				retry = PATCH_DOWNLOAD_PHASE3_RETRY;
			}

			cur_len += sent_len;
			ret = btmtk_main_send_cmd(data, image, sent_len + sent_hdr_len,
					event, event_len, delay, retry, ep);
			if (ret < 0) {
				usb_debug("%s: send patch failed, terminate\n", __func__);
				goto exit;
			}
		} else
			break;
	}

exit:
	return ret;
}

static int btmtk_send_fw_rom_patch_79xx(struct LD_btmtk_usb_data *data)
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
	u8 event[] = {0xE4, 0x05, 0x02, 0x01, 0x01, 0x00, 0x00}; /* event[6] is status*/

	if (data->rom_patch == NULL) {
		usb_debug("data->rom_patch is NULL!");
		ret = -1;
		goto exit;
	}

	globalDescr = (struct _Global_Descr *)(data->rom_patch + FW_ROM_PATCH_HEADER_SIZE);

	usb_debug("%s: loading rom patch...\n", __func__);

	section_num = globalDescr->u4SectionNum;
	usb_debug("%s: section_num = 0x%08x\n", __func__, section_num);

	pos = os_kzalloc(UPLOAD_PATCH_UNIT, MTK_GFP_ATOMIC);
	if (!pos) {
		usb_debug("%s: alloc memory failed\n", __func__);
		ret = -1;
		goto exit;
	}

	do {
		sectionMap = (struct _Section_Map *)(data->rom_patch + FW_ROM_PATCH_HEADER_SIZE +
				FW_ROM_PATCH_GD_SIZE + FW_ROM_PATCH_SEC_MAP_SIZE * loop_count);

		section_offset = sectionMap->u4SecOffset;
		dl_size = sectionMap->bin_info_spec.u4DLSize;

		usb_debug("%s: loop_count = %d, section_offset = 0x%08x, download patch_len = 0x%08x\n",
				__func__, loop_count, section_offset, dl_size);

		if (dl_size > 0) {
			retry = 20;
			do {
				patch_status = btmtk_send_wmt_download_cmd(data, pos, 0,
						event, sizeof(event) - 1, sectionMap, 0, dma_flag);
				usb_debug("%s: patch_status %d\n", __func__, patch_status);

				if (patch_status > BUZZARD_PATCH_READY || patch_status == PATCH_ERR) {
					usb_debug("%s: patch_status error\n", __func__);
					ret = -1;
					goto err;
				} else if (patch_status == BUZZARD_PATCH_READY) {
					usb_debug("%s: no need to load rom patch section%d\n", __func__, loop_count);
					goto next_section;
				} else if (patch_status == BUZZARD_PATCH_IS_DOWNLOAD_BY_OTHER) {
					MTK_MDELAY(100);
					retry--;
				} else if (patch_status == BUZZARD_PATCH_NEED_DOWNLOAD) {
					break;  /* Download ROM patch directly */
				}
			} while (retry > 0);

			if (patch_status == BUZZARD_PATCH_IS_DOWNLOAD_BY_OTHER) {
				usb_debug("%s: Hold by another fun more than 2 seconds\n", __func__);
				ret = -1;
				goto err;
			}

			if (dma_flag == PATCH_DOWNLOAD_USING_DMA) {
				/* using DMA to download fw patch*/
			} else {
				/* using legacy wmt cmd to download fw patch */
				ret = btmtk_load_fw_patch_using_wmt_cmd(data, pos, event,
						sizeof(event) - 1, dl_size, section_offset);
				if (ret < 0) {
					usb_debug("%s: btmtk_load_fw_patch_using_wmt_cmd failed!\n", __func__);
					goto err;
				}
			}
		}

next_section:
		continue;
	} while (++loop_count < section_num);

err:
	os_kfree(pos);
	pos = NULL;

exit:
	usb_debug("%s: loading rom patch... Done\n", __func__);
	return ret;
}

int btmtk_load_rom_patch_79xx(struct LD_btmtk_usb_data *data)
{
	int ret = 0;

	LD_load_code_from_bin(&data->rom_patch, (char *)data->rom_patch_bin_file_name,
				NULL, data->udev, &data->rom_patch_len);
	if (!data->rom_patch || !data->rom_patch_len) {
		usb_debug("please assign a rom patch from (/etc/firmware/%s) or (/lib/firmware/%s)\n",
				data->rom_patch_bin_file_name, data->rom_patch_bin_file_name);
		return -1;
	}

	btmtk_print_bt_patch_info(data);

	ret = btmtk_send_fw_rom_patch_79xx(data);
	if (ret < 0) {
		usb_debug("%s, btmtk_send_fw_rom_patch_79xx failed!\n", __func__);
		return ret;
	}

	usb_debug("btmtk_load_rom_patch_79xx end\n");

	return ret;
}

static int btmtk_usb_load_fw_cfg_setting(char *block_name, struct fw_cfg_struct *save_content,
		int counter, u8 *searchcontent, enum fw_cfg_index_len index_length)
{
	int ret = 0, i = 0;
	int temp_len = 0;
	u8 temp[260]; /* save for total hex number */
	char *search_result = NULL;
	char *search_end = NULL;
	char search[32];
	char *next_block = NULL;
#define CHAR2HEX_SIZE	4
	char number[CHAR2HEX_SIZE + 1];	/* 1 is for '\0' */

	memset(search, 0, sizeof(search));
	memset(temp, 0, sizeof(temp));
	memset(number, 0, sizeof(number));

	/* search block name */
	for (i = 0; i < counter; i++) {
		temp_len = 0;
		if (index_length == FW_CFG_INX_LEN_2) /* EX: APCF01 */
			snprintf(search, sizeof(search), "%s%02d:", block_name, i);
		else if (index_length == FW_CFG_INX_LEN_3) /* EX: APCF001 */
			snprintf(search, sizeof(search), "%s%03d:", block_name, i);
		else
			snprintf(search, sizeof(search), "%s:", block_name);
		search_result = strstr((char *)searchcontent, search);

		if (search_result) {
			memset(temp, 0, sizeof(temp));
			search_result = strstr(search_result, "0x");
			/* find next line as end of this command line, if NULL means last line */
			next_block = strstr(search_result, ":");

			do {
				search_end = strstr(search_result, ",");
				if (search_end - search_result != CHAR2HEX_SIZE) {
					usb_debug("Incorrect Format in %s\n", search);
					break;
				}

				memset(number, 0, sizeof(number));
				memcpy(number, search_result, CHAR2HEX_SIZE);

				temp[temp_len++] = strtol(number, &number, 0);
				search_result = strstr(search_end, "0x");
			} while (search_result < next_block || (search_result && next_block == NULL));
		}

		if (temp_len && temp_len < sizeof(temp)) {
			usb_debug("%s found & stored in %d\n", search, i);
			save_content[i].content = os_kzalloc(temp_len, MTK_GFP_ATOMIC);
			if (save_content[i].content == NULL) {
				usb_debug("Allocate memory fail(%d)\n", i);
				return -ENOMEM;
			}
			memcpy(save_content[i].content, temp, temp_len);
			save_content[i].length = temp_len;
		}
	}

	return ret;
}

int btmtk_usb_load_woble_setting(struct LD_btmtk_usb_data *data)
{
	int err = 0;
	BOOL woble_setting_bin = FALSE;
	BOOL wake_on_ble_conf = FALSE;

	if (!data)
		return -EINVAL;

	/* For woble_setting.bin */
	if (data->setting_file != NULL) {
		os_kfree(data->setting_file);
		data->setting_file = NULL;
	}
	data->setting_file_len = 0;

	LD_load_code_from_bin(&data->setting_file, data->woble_setting_file_name,
		NULL, data->udev, &data->setting_file_len);

	if (data->setting_file == NULL || data->setting_file_len == 0) {
		usb_debug("Please make sure %s in the /vendor/firmware\n",
				data->woble_setting_file_name);
		woble_setting_bin = FALSE;
	} else {
		err = btmtk_usb_load_fw_cfg_setting("APCF", data->woble_setting_apcf,
			WOBLE_SETTING_COUNT, data->setting_file, FW_CFG_INX_LEN_2);
		if (err)
			goto LOAD_ERR;

		err = btmtk_usb_load_fw_cfg_setting("APCF_ADD_MAC",
				data->woble_setting_apcf_fill_mac, WOBLE_SETTING_COUNT,
				data->setting_file, FW_CFG_INX_LEN_2);
		if (err)
			goto LOAD_ERR;

		err = btmtk_usb_load_fw_cfg_setting("APCF_ADD_MAC_LOCATION",
				data->woble_setting_apcf_fill_mac_location, WOBLE_SETTING_COUNT,
				data->setting_file, FW_CFG_INX_LEN_2);
		if (err)
			goto LOAD_ERR;

		err = btmtk_usb_load_fw_cfg_setting("RADIOOFF", &data->woble_setting_radio_off,
				1, data->setting_file, FW_CFG_INX_LEN_2);
		if (err)
			goto LOAD_ERR;

		switch (data->bt_cfg.unify_woble_type) {
		case 0:
			err = btmtk_usb_load_fw_cfg_setting("WAKEUP_TYPE_LEGACY",
					&data->woble_setting_wakeup_type, 1, data->setting_file, FW_CFG_INX_LEN_2);
			break;
		case 1:
			err = btmtk_usb_load_fw_cfg_setting("WAKEUP_TYPE_WAVEFORM",
					&data->woble_setting_wakeup_type, 1, data->setting_file, FW_CFG_INX_LEN_2);
			break;
		case 2:
			err = btmtk_usb_load_fw_cfg_setting("WAKEUP_TYPE_IR",
					&data->woble_setting_wakeup_type, 1, data->setting_file, FW_CFG_INX_LEN_2);
			break;
		default:
			usb_debug("unify_woble_type unknown(%d)\n", data->bt_cfg.unify_woble_type);
		}
		if (err) {
			usb_debug("Parse unify_woble_type(%d) failed\n", data->bt_cfg.unify_woble_type);
			goto LOAD_ERR;
		}

		err = btmtk_usb_load_fw_cfg_setting("RADIOOFF_COMPLETE_EVENT",
				&data->woble_setting_radio_off_comp_event, 1, data->setting_file,
				FW_CFG_INX_LEN_2);

LOAD_ERR:
		if (data->setting_file) {
			os_kfree(data->setting_file);
			data->setting_file = NULL;
			data->setting_file_len = 0;
		}

		if (err) {
			woble_setting_bin = FALSE;
			usb_debug("err(%d), woble_setting_bin(%d)\n", err, woble_setting_bin);
		} else
			woble_setting_bin = TRUE;
	}

	/* For wake_on_ble.conf */
	data->wake_dev = NULL;
	data->wake_dev_len = 0;

	LD_load_code_from_bin(&data->wake_dev, WAKE_DEV_RECORD, WAKE_DEV_RECORD_PATH,
			data->udev, &data->wake_dev_len);
	if (data->wake_dev == NULL || data->wake_dev_len == 0) {
		usb_debug("There is no DEVICE RECORD for wake-up\n");
		wake_on_ble_conf = FALSE;
	} else {
		// content check
		if (data->wake_dev[0] != data->wake_dev_len || data->wake_dev_len < 3) {
			usb_debug("Incorrect total length on %s\n", WAKE_DEV_RECORD);
			data->wake_dev_len = 0;
			os_kfree(data->wake_dev);
			data->wake_dev = NULL;
			wake_on_ble_conf = FALSE;
		} else {
			wake_on_ble_conf = TRUE;
		}
	}

	if (woble_setting_bin == FALSE && wake_on_ble_conf == FALSE)
		return -ENOENT;
	return 0;
}

static int btmtk_usb_set_unify_woble(struct LD_btmtk_usb_data *data)
{
	int ret = -1;
	u8 *radio_off = NULL;
	int length = 0;
	// Filter Index: 0x5A
	u8 cmd[] = { 0xC9, 0xFC, 0x14, 0x01, 0x20, 0x02, 0x00, 0x01, 0x02, 0x01,
		0x00, 0x05, 0x10, 0x01, 0x00, 0x40, 0x06, 0x02, 0x40, FIDX, 0x02,
		0x41, 0x0F};
	u8 event[] = { 0xE6, 0x02, 0x08, 0x00 };

	if (is_mt7961(data)) {
		cmd[FIDX_OFFSET_RADIO_OFF] = BUZZARD_FIDX;
	}

	if (data->woble_setting_radio_off.length && is_support_unify_woble(data)) {
		/* start to send radio off cmd from woble setting file */
		length = data->woble_setting_radio_off.length +
				data->woble_setting_wakeup_type.length;
		radio_off = os_kzalloc(length, MTK_GFP_ATOMIC);
		if (!radio_off) {
			usb_debug("alloc memory fail (radio_off)\n");
			ret = -ENOMEM;
			return ret;
		}

		memcpy(radio_off, data->woble_setting_radio_off.content,
			data->woble_setting_radio_off.length);
		if (data->woble_setting_wakeup_type.length) {
			memcpy(radio_off + data->woble_setting_radio_off.length,
				data->woble_setting_wakeup_type.content,
				data->woble_setting_wakeup_type.length);
			radio_off[2] += data->woble_setting_wakeup_type.length;
		}

		usb_debug_raw(radio_off, length, "Send radio off");
		ret = btmtk_usb_send_hci_cmd(data, radio_off,
				length,
				data->woble_setting_radio_off_comp_event.content,
				data->woble_setting_radio_off_comp_event.length);
		if (ret < 0) {
			usb_debug("btmtk_usb_send_hci_cmd return fail %d\n", ret);
			return ret;
		}
	}else { /* use default */
		usb_debug("use default radio off cmd\n");
		ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), event, sizeof(event));
		if (ret < 0)
			usb_debug("Failed(%d)\n", ret);
	}
	return ret;
}

static void btmtk_usb_free_fw_cfg_struct(struct fw_cfg_struct *fw_cfg, int count)
{
	int i = 0;

	for (i = 0; i < count; i++) {
		if (fw_cfg[i].content) {
			os_kfree(fw_cfg[i].content);
			fw_cfg[i].content = NULL;
			fw_cfg[i].length = 0;
		} else
			fw_cfg[i].length = 0;
	}
	return;
}

static void btmtk_usb_free_setting_file(struct LD_btmtk_usb_data *data)
{
	btmtk_usb_free_fw_cfg_struct(data->woble_setting_apcf, WOBLE_SETTING_COUNT);
	btmtk_usb_free_fw_cfg_struct(data->woble_setting_apcf_fill_mac, WOBLE_SETTING_COUNT);
	btmtk_usb_free_fw_cfg_struct(data->woble_setting_apcf_fill_mac_location, WOBLE_SETTING_COUNT);
	btmtk_usb_free_fw_cfg_struct(&data->woble_setting_radio_off, 1);
	btmtk_usb_free_fw_cfg_struct(&data->woble_setting_radio_off_comp_event, 1);
	btmtk_usb_free_fw_cfg_struct(data->bt_cfg.wmt_cmd, WMT_CMD_COUNT);

	memset(&data->bt_cfg, 0, sizeof(data->bt_cfg));

	if (data->woble_setting_file_name) {
		os_kfree(data->woble_setting_file_name);
		data->woble_setting_file_name = NULL;
	}
	return;
}

static int btmtk_usb_parse_bt_cfg_file(char *item_name, char *text, u8 *searchcontent)
{
	int ret = 0;
	int temp_len = 0;
	char search[32];
	char *ptr = NULL, *p = NULL;
	char *temp = text;

	if (text == NULL) {
		usb_debug("text param is invalid!\n");
		ret = false;
		goto out;
	}

	memset(search, 0, sizeof(search));
	snprintf(search, sizeof(search), "%s", item_name); /* EX: SUPPORT_UNIFY_WOBLE */
	p = ptr = strstr((char *)searchcontent, search);

	if (!ptr) {
		usb_debug("Can't find %s\n", item_name);
		ret = -1;
		goto out;
	}

	if (p > (char *)searchcontent) {
		p--;
		while ((*p == ' ') && (p != (char *)searchcontent))
			p--;
		if (*p == '#') {
			usb_debug("It's invalid bt cfg item\n");
			ret = -1;
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

static int btmtk_usb_load_bt_cfg_item(struct bt_cfg_struct *bt_cfg_content, u8 *searchcontent)
{
	int ret = 0;
	char text[128]; /* save for search text */

	memset(text, 0, sizeof(text));
	ret = btmtk_usb_parse_bt_cfg_file(BT_UNIFY_WOBLE, text, searchcontent);
	if (!ret) {
		bt_cfg_content->support_unify_woble = strtol(text, &text, 10);
		usb_debug("bt_cfg_content->support_unify_woble = %d\n", bt_cfg_content->support_unify_woble);
	} else {
		usb_debug("search item %s is invalid!\n", BT_UNIFY_WOBLE);
	}

	ret = btmtk_usb_parse_bt_cfg_file(BT_UNIFY_WOBLE_TYPE, text, searchcontent);
	if (!ret) {
		bt_cfg_content->unify_woble_type = strtol(text, &text, 10);
		usb_debug("bt_cfg_content->unify_woble_type = %d\n", bt_cfg_content->unify_woble_type);
	} else {
		usb_debug("search item %s is invalid!\n", BT_UNIFY_WOBLE_TYPE);
	}

	ret = btmtk_usb_load_fw_cfg_setting(BT_WMT_CMD, bt_cfg_content->wmt_cmd,
			WMT_CMD_COUNT, searchcontent, FW_CFG_INX_LEN_3);
	if (ret)
		usb_debug("%s: search item %s is invalid!", __func__, BT_WMT_CMD);
	return ret;
}

static int btmtk_usb_load_bt_cfg(char *cfg_name, struct LD_btmtk_usb_data *data)
{
	int ret = 0;

	if (!data)
		return -EINVAL;

	/* For woble_setting.bin */
	if (data->setting_file != NULL) {
		os_kfree(data->setting_file);
		data->setting_file = NULL;
	}
	data->setting_file_len = 0;

	LD_load_code_from_bin(&data->setting_file, BT_CFG_NAME, NULL, data->udev, &data->setting_file_len);

	if (data->setting_file == NULL || data->setting_file_len == 0) {
		usb_debug("Please make sure %s in the /vendor/firmware\n", BT_CFG_NAME);
		return -ENOMEM;
	}

	ret = btmtk_usb_load_bt_cfg_item(&data->bt_cfg, data->setting_file);
	if (ret) {
		usb_debug("btmtk_usb_load_bt_cfg_item error return %d\n", ret);
		return ret;
	}

	/* release setting file memory */
	if (data->setting_file != NULL) {
		os_kfree(data->setting_file);
		data->setting_file = NULL;
		data->setting_file_len = 0;
	}
	return ret;
}

static void btmtk_usb_initialize_cfg_items(struct LD_btmtk_usb_data *data)
{
	if (data == NULL) {
		usb_debug("btmtk data NULL!\n");
		return;
	}

	data->bt_cfg.support_unify_woble = 1;
	data->bt_cfg.unify_woble_type = 0;
	btmtk_usb_free_fw_cfg_struct(data->bt_cfg.wmt_cmd, WMT_CMD_COUNT);
}

/*============================================================================*/
/* Interface Functions */
/*============================================================================*/
int  Ldbtusb_getBtWakeT(struct LD_btmtk_usb_data *data){
	//struct LD_btmtk_usb_data *data = BT_INST(dev)->priv_data;
	int ret = -1;
	if (!data) {
		usb_debug("btmtk data NULL!\n");
		return ret;
	}
	if (is_mt7668(data)) {
		usb_debug("is_mt7668(data \n");
		//resume
		u8 buf[] = {0xC9, 0xFC, 0x05, 0x01, 0x21, 0x02, 0x00, 0x00};
		u8 event[] = { 0xe6, 0x02, 0x08, 0x01 };
		ret = btmtk_usb_send_hci_cmd(data, buf, sizeof(buf), event, sizeof(event));

		if (ret < 0)
		{
			usb_debug("%s error1(%d)\n", __func__, ret);
			return ret;
		}
		u8 cmd[] = { 0xCE, 0xFC, 0x04, 0x03, 0x00, 0x00, 0x00};
		u8 event1[] = { 0xe8 };
		ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), event1, sizeof(event1));
		int len = 0;
		u8 retry = 0;
		u8 returnEvent = 0xe8;
		usb_debug("\n");
		do {
			ret = data->hcif->usb_interrupt_msg(data->udev, MTKBT_INTR_EP, data->io_buf,
				LD_BT_MAX_EVENT_SIZE, &len, USB_INTR_MSG_TIMO);
			if (ret < 0) {
				usb_debug("event get failed: %d\n", ret);
			}
			if (returnEvent == data->io_buf[0]) {//get response from fw
				usb_debug("mode %d\n",data->io_buf[WOBLE_LOG_OFFSET_76XX]);
				if(data->io_buf[WOBLE_LOG_OFFSET_76XX] == WOBLE_LOG_VALUE_76XX) {//get soundmode bt wake type
					//Xgimi_Set_MusicMode(TRUE);
				}
				break;
			} else {
				usb_debug("\n");
			}
			MTK_MDELAY(10);
			++retry;
			usb_debug("try get event again\n");
		} while (retry < 5);
	} else {
		usb_debug("is_mt7662 data \n");
		btmtk_usb_send_hci_reset_cmd(data);
		u8 buf[] = {0xC9, 0xFC, 0x05, 0x01, 0x21, 0x02, 0x00, 0x00};
		u8 event[] = { 0xe6, 0x02, 0x08, 0x01 };
		ret = btmtk_usb_send_hci_cmd(data, buf, sizeof(buf), event, sizeof(event));

		if (ret < 0) {
			usb_debug("%s error1(%d)\n", __func__, ret);
			return ret;
		}
		u8 cmd[] = { 0x4B, 0xFC, 0x04, 0x03, 0x00, 0x00, 0x00};
		u8 event1[] = { 0x0f  };
		ret = btmtk_usb_send_hci_cmd(data, cmd, sizeof(cmd), event1, sizeof(event1));
		int len = 0;
		u8 retry = 0;
		u8 returnEvent = 0xe8;

		usb_debug("after send\n");
		do {
			ret = data->hcif->usb_interrupt_msg(data->udev, MTKBT_INTR_EP, data->io_buf,
				LD_BT_MAX_EVENT_SIZE, &len, USB_INTR_MSG_TIMO);
			if (ret < 0) {
				usb_debug("event get failed: %d\n", ret);
			}
			if (returnEvent == data->io_buf[0]) {//get response from fw
				usb_debug("mode %d\n",data->io_buf[WOBLE_LOG_OFFSET_76XX]);
				if(data->io_buf[WOBLE_LOG_OFFSET_76XX] == WOBLE_LOG_VALUE_76XX){//get soundmode bt wake type
					//Xgimi_Set_MusicMode(TRUE);
				}
				break;
			} else {
				usb_debug("\n");
			}
			MTK_MDELAY(10);
			++retry;
			usb_debug("try get event again\n");
			} while (retry < 5);
	}
	return 0;
}

u8 LD_btmtk_usb_getWoBTW(void)
{
	usb_debug("LD_btmtk_usb_getWoBTW, u8WoBTW = %d\n", u8WoBTW);
	return u8WoBTW;
}

void LD_btmtk_usb_SetWoble(mtkbt_dev_t *dev)
{
	struct LD_btmtk_usb_data *data = BT_INST(dev)->priv_data;
	int ret = -1;
	u8WoBTW = PM_SOURCE_DISABLE;

	usb_debug("\n");
	if (!data) {
		usb_debug("btmtk data NULL!\n");
		return;
	}

	if (is_mt7668(data) || is_mt7663(data) || is_mt7961(data)) {
		if (is_support_unify_woble(data)) {
			if (data->bt_cfg.unify_woble_type == UNIFY_WOBLE_WAVEFORM) {
				/* BTW */
				u8WoBTW = 1;
			}

			/* Power on sequence */
			btmtk_usb_send_wmt_power_on_cmd_7668(data);
			if (!is_mt7961(data)) {
				btmtk_usb_send_hci_tci_set_sleep_cmd_7668(data);
				btmtk_usb_send_hci_reset_cmd(data);
				btmtk_usb_get_vendor_cap(data);
 			} else {
				btmtk_usb_send_apcf_reserved_79xx(data);
			}

			btmtk_usb_send_read_bdaddr(data);
			ret = btmtk_usb_load_woble_setting(data);
			if (ret) {
				usb_debug("Using lagecy WoBLE setting(%d)!!!\n", ret);
				btmtk_usb_set_apcf(data, FALSE);
			} else
				btmtk_usb_set_apcf(data, TRUE);
			btmtk_usb_set_unify_woble(data);
		}
	} else {
		btmtk_usb_send_hci_suspend_cmd(data);
	}

	// Clean & free buffer
	btmtk_usb_free_setting_file(data);

	if (data->wake_dev) {
		os_kfree(data->wake_dev);
		data->wake_dev = NULL;
		data->wake_dev_len = 0;
	}

	return;
}

int LD_btmtk_usb_probe(mtkbt_dev_t *dev,int flag)
{
	struct LD_btmtk_usb_data *data;
	int  ret=0, err = 0;

	usb_debug("=========================================\n");
	usb_debug("Mediatek Bluetooth USB driver ver %s\n", LD_VERSION);
	usb_debug("=========================================\n");
	os_memcpy(driver_version, LD_VERSION, sizeof(LD_VERSION));
	probe_counter++;
	isbtready = 0;
	is_assert = 0;
	//usb_debug("LD_btmtk_usb_probe begin\n");
	usb_debug("probe_counter = %d\n", probe_counter);

	data = os_kzalloc(sizeof(*data), MTK_GFP_ATOMIC);
	if (!data) {
		usb_debug("[ERR] end Error 1\n");
		return -ENOMEM;
	}

	data->hcif = BT_INST(dev)->hci_if;

	data->cmdreq_type = USB_TYPE_CLASS;

	data->udev = dev;

	data->meta_tx = 0;

	data->chip_id = dev->chipid;

	data->io_buf = os_kmalloc(LD_BT_MAX_EVENT_SIZE, MTK_GFP_ATOMIC);
	if (!data->io_buf) {
		usb_debug("Can't allocate memory io_buf\n");
		os_kfree(data);
		return -ENOMEM;
	}

	data->rom_patch_bin_file_name = os_kzalloc(MAX_BIN_FILE_NAME_LEN, MTK_GFP_ATOMIC);
	if (!data->rom_patch_bin_file_name) {
		usb_debug("Can't allocate memory rom_patch_bin_file_name\n");
		os_kfree(data->io_buf);
		os_kfree(data);
		return -ENOMEM;
	}

	if (data->woble_setting_file_name == NULL) {
		data->woble_setting_file_name = os_kzalloc(MAX_BIN_FILE_NAME_LEN, MTK_GFP_ATOMIC);
		if (!data->woble_setting_file_name) {
			usb_debug("Can't allocate memory woble_setting_file_name\n");
			os_kfree(data->rom_patch_bin_file_name);
			os_kfree(data->io_buf);
			os_kfree(data);
			return -ENOMEM;
		}
	}

	btmtk_usb_switch_iobase(data, WLAN);
	btmtk_usb_initialize_cfg_items(data);

	/* clayton: according to the chip id, load f/w or rom patch */
	usb_debug("data address = %p\n", data);
	btmtk_usb_cap_init(data);

	if(flag == 1){
		ret = Ldbtusb_getBtWakeT(data);
		os_kfree(data->io_buf);
		os_kfree(data);
		return ret;
	}

    /* load bt.cfg */
	btmtk_usb_load_bt_cfg(BT_CFG_NAME, data);

	if (data->need_load_rom_patch) {
		if (is_mt7668(data) || is_mt7663(data))
			err = btmtk_usb_load_rom_patch_7668(data);
		else if (is_mt7961(data))
			err = btmtk_load_rom_patch_79xx(data);
		else
			err = btmtk_usb_load_rom_patch(data);
		//btmtk_usb_send_hci_suspend_cmd(data);
		if (err < 0) {
			if (data->io_buf) os_kfree(data->io_buf);
			if (data->rom_patch_bin_file_name) os_kfree(data->rom_patch_bin_file_name);
			if (data->woble_setting_file_name) os_kfree(data->woble_setting_file_name);
			os_kfree(data);
			usb_debug("[ERR] end Error 2\n");
			return err;
		}
	}

	// Clean & free buffer
	if (data->rom_patch_bin_file_name)
		os_kfree(data->rom_patch_bin_file_name);


	isUsbDisconnet = 0;
	BT_INST(dev)->priv_data = data;
	isbtready = 1;

	//usb_debug("btmtk_usb_probe end\n");
	return 0;
}

void LD_btmtk_usb_disconnect(mtkbt_dev_t *dev)
{
	struct LD_btmtk_usb_data *data = BT_INST(dev)->priv_data;

	usb_debug("\n");

	if (!data)
		return;

	isbtready = 0;
	metaCount = 0;

	if (data->need_load_rom_patch)
		os_kfree(data->rom_patch);

	if (data->need_load_fw)
		os_kfree(data->fw_image);

	usb_debug("unregister bt irq\n");

	isUsbDisconnet = 1;
	usb_debug("btmtk: stop all URB\n");
	os_kfree(data->io_buf);
	os_kfree(data);
}

