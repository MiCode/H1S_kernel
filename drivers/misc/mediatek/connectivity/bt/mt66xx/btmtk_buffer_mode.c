/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "btmtk_define.h"
#include "btmtk_main.h"
#include "btmtk_buffer_mode.h"

static struct btmtk_buffer_mode_struct btmtk_buffer_mode;

static int btmtk_buffer_mode_check_auto_mode(struct btmtk_buffer_mode_struct *buffer_mode)
{
	u16 addr = 1;
	u8 value = 0;

	if (buffer_mode->efuse_mode != AUTO_MODE)
		return 0;

	if (btmtk_efuse_read(buffer_mode->bdev, addr, &value)) {
		BTMTK_WARN("read fail");
		BTMTK_WARN("Use EEPROM Bin file mode");
		buffer_mode->efuse_mode = BIN_FILE_MODE;
		return -EIO;
	}

	if (value == ((buffer_mode->bdev->chip_id & 0xFF00) >> 8)) {
		BTMTK_WARN("get efuse[1]: 0x%02x", value);
		BTMTK_WARN("use efuse mode");
		buffer_mode->efuse_mode = EFUSE_MODE;
	} else {
		BTMTK_WARN("get efuse[1]: 0x%02x", value);
		BTMTK_WARN("Use EEPROM Bin file mode");
		buffer_mode->efuse_mode = BIN_FILE_MODE;
	}

	return 0;
}

static int btmtk_buffer_mode_parse_mode(uint8_t *buf, size_t buf_size)
{
	int efuse_mode = EFUSE_MODE;
	char *p_buf = NULL;
	char *ptr = NULL, *p = NULL;

	if (!buf) {
		BTMTK_WARN("buf is null");
		return efuse_mode;
	} else if (buf_size < (strlen(BUFFER_MODE_SWITCH_FIELD) + 2)) {
		BTMTK_WARN("incorrect buf size(%d)", (int)buf_size);
		return efuse_mode;
	}

	p_buf = kmalloc(buf_size + 1, GFP_KERNEL);
	if (!p_buf)
		return efuse_mode;
	memcpy(p_buf, buf, buf_size);
	p_buf[buf_size] = '\0';

	/* find string */
	p = ptr = strstr(p_buf, BUFFER_MODE_SWITCH_FIELD);
	if (!ptr) {
		BTMTK_ERR("Can't find %s", BUFFER_MODE_SWITCH_FIELD);
		goto out;
	}

	if (p > p_buf) {
		p--;
		while ((*p == ' ') && (p != p_buf))
			p--;
		if (*p == '#') {
			BTMTK_ERR("It's not EEPROM - Bin file mode");
			goto out;
		}
	}

	/* check access mode */
	ptr += (strlen(BUFFER_MODE_SWITCH_FIELD) + 1);
	BTMTK_WARN("It's EEPROM bin mode: %c", *ptr);
	efuse_mode = *ptr - '0';
	if (efuse_mode > AUTO_MODE)
		efuse_mode = EFUSE_MODE;
out:
	kfree(p_buf);
	return efuse_mode;
}

static int btmtk_buffer_mode_set_addr(struct btmtk_buffer_mode_struct *buffer_mode)
{
	u8 cmd[SET_ADDRESS_CMD_LEN] = {0x01, 0x1A, 0xFC, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	u8 event[SET_ADDRESS_EVT_LEN] = {0x04, 0x0E, 0x04, 0x01, 0x1A, 0xFC, 0x00};
	int ret = 0;

	if (buffer_mode->bt0_mac[0] == 0x00 && buffer_mode->bt0_mac[1] == 0x00
		&& buffer_mode->bt0_mac[2] == 0x00 && buffer_mode->bt0_mac[3] == 0x00
		&& buffer_mode->bt0_mac[4] == 0x00 && buffer_mode->bt0_mac[5] == 0x00) {
		BTMTK_WARN("BDAddr is Zero, not set");
	} else {
		cmd[SET_ADDRESS_CMD_PAYLOAD_OFFSET + 5] = buffer_mode->bt0_mac[0];
		cmd[SET_ADDRESS_CMD_PAYLOAD_OFFSET + 4] = buffer_mode->bt0_mac[1];
		cmd[SET_ADDRESS_CMD_PAYLOAD_OFFSET + 3] = buffer_mode->bt0_mac[2];
		cmd[SET_ADDRESS_CMD_PAYLOAD_OFFSET + 2] = buffer_mode->bt0_mac[3];
		cmd[SET_ADDRESS_CMD_PAYLOAD_OFFSET + 1] = buffer_mode->bt0_mac[4];
		cmd[SET_ADDRESS_CMD_PAYLOAD_OFFSET] = buffer_mode->bt0_mac[5];

		BTMTK_INFO_RAW(cmd, SET_ADDRESS_CMD_LEN, "%s: Send", __func__);
		ret = btmtk_main_send_cmd(buffer_mode->bdev,
				cmd, SET_ADDRESS_CMD_LEN,
				event, SET_ADDRESS_EVT_LEN,
				0, 0, BTMTK_TX_CMD_FROM_DRV);
	}

	BTMTK_INFO("%s done", __func__);
	return ret;
}

static int btmtk_buffer_mode_set_radio(struct btmtk_buffer_mode_struct *buffer_mode)
{
	u8 cmd[SET_RADIO_CMD_LEN] = {0x01, 0x2C, 0xFC, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	u8 event[SET_RADIO_EVT_LEN] = {0x04, 0x0E, 0x04, 0x01, 0x2C, 0xFC, 0x00};
	int ret = 0;

	cmd[SET_RADIO_CMD_EDR_DEF_OFFSET] = buffer_mode->bt0_radio.radio_0 & 0x3F;		/* edr_init_pwr */
	cmd[SET_RADIO_CMD_BLE_OFFSET] = buffer_mode->bt0_radio.radio_2 & 0x3F;		/* ble_default_pwr */
	cmd[SET_RADIO_CMD_EDR_MAX_OFFSET] = buffer_mode->bt0_radio.radio_1 & 0x3F;		/* edr_max_pwr */
	cmd[SET_RADIO_CMD_EDR_MODE_OFFSET] = (buffer_mode->bt0_radio.radio_0 & 0xC0) >> 6;	/* edr_pwr_mode */

	BTMTK_INFO_RAW(cmd, SET_RADIO_CMD_LEN, "%s: Send", __func__);
	ret = btmtk_main_send_cmd(buffer_mode->bdev,
			cmd, SET_RADIO_CMD_LEN,
			event, SET_RADIO_EVT_LEN,
			0, 0, BTMTK_TX_CMD_FROM_DRV);

	BTMTK_INFO("%s done", __func__);
	return ret;
}

static int btmtk_buffer_mode_set_group_boundary(struct btmtk_buffer_mode_struct *buffer_mode)
{
	u8 cmd[SET_GRP_CMD_LEN] = {0x01, 0xEA, 0xFC, 0x09, 0x02, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	u8 event[SET_GRP_EVT_LEN] = {0x04, 0x0E, 0x04, 0x01, 0xEA, 0xFC, 0x00};
	int ret = 0;

	memcpy(&cmd[SET_GRP_CMD_PAYLOAD_OFFSET], buffer_mode->bt0_ant0_grp_boundary, BUFFER_MODE_GROUP_LENGTH);

	BTMTK_INFO_RAW(cmd, SET_GRP_CMD_LEN, "%s: Send", __func__);
	ret = btmtk_main_send_cmd(buffer_mode->bdev,
			cmd, SET_GRP_CMD_LEN,
			event, SET_GRP_EVT_LEN,
			0, 0, BTMTK_TX_CMD_FROM_DRV);

	BTMTK_INFO("%s done", __func__);
	return ret;
}

static int btmtk_buffer_mode_set_power_offset(struct btmtk_buffer_mode_struct *buffer_mode)
{
	u8 cmd[SET_PWR_OFFSET_CMD_LEN] = {0x01, 0xEA, 0xFC, 0x0A,
		0x02, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	u8 event[SET_PWR_OFFSET_EVT_LEN] = {0x04, 0x0E, 0x04, 0x01, 0xEA, 0xFC, 0x00};
	int ret = 0;

	memcpy(&cmd[SET_PWR_OFFSET_CMD_PAYLOAD_OFFSET], buffer_mode->bt0_ant0_pwr_offset, BUFFER_MODE_CAL_LENGTH);

	BTMTK_INFO_RAW(cmd, SET_PWR_OFFSET_CMD_LEN, "%s: Send", __func__);
	ret = btmtk_main_send_cmd(buffer_mode->bdev,
			cmd, SET_PWR_OFFSET_CMD_LEN,
			event, SET_PWR_OFFSET_EVT_LEN,
			0, 0, BTMTK_TX_CMD_FROM_DRV);

	BTMTK_INFO("%s done", __func__);
	return ret;
}

int btmtk_buffer_mode_send(struct btmtk_buffer_mode_struct *buffer_mode)
{
	int ret = 0;

	if (buffer_mode == NULL) {
		BTMTK_INFO("buffer_mode is NULL, not support");
		return -EIO;
	}

	if (btmtk_buffer_mode_check_auto_mode(buffer_mode)) {
		BTMTK_ERR("check auto mode failed");
		return -EIO;
	}

	if (buffer_mode->efuse_mode == BIN_FILE_MODE) {
		ret = btmtk_buffer_mode_set_addr(buffer_mode);
		if (ret < 0)
			BTMTK_ERR("set addr failed");

		ret = btmtk_buffer_mode_set_radio(buffer_mode);
		if (ret < 0)
			BTMTK_ERR("set radio failed");

		ret = btmtk_buffer_mode_set_group_boundary(buffer_mode);
		if (ret < 0)
			BTMTK_ERR("set group_boundary failed");

		ret = btmtk_buffer_mode_set_power_offset(buffer_mode);
		if (ret < 0)
			BTMTK_ERR("set power_offset failed");
	}
	return 0;
}

void btmtk_buffer_mode_initialize(struct btmtk_dev *bdev, struct btmtk_buffer_mode_struct **buffer_mode)
{
	int ret = 0;
	u32 code_len = 0;

	btmtk_buffer_mode.bdev = bdev;
	ret = btmtk_load_code_from_setting_files(BUFFER_MODE_SWITCH_FILE, bdev->intf_dev, &code_len, bdev);

	btmtk_buffer_mode.efuse_mode = btmtk_buffer_mode_parse_mode(bdev->setting_file, code_len);
	if (btmtk_buffer_mode.efuse_mode == EFUSE_MODE)
		return;

	if (bdev->flavor)
		(void)snprintf(btmtk_buffer_mode.file_name, MAX_BIN_FILE_NAME_LEN, "EEPROM_MT%04x_1a.bin",
				bdev->chip_id & 0xffff);
	else
		(void)snprintf(btmtk_buffer_mode.file_name, MAX_BIN_FILE_NAME_LEN, "EEPROM_MT%04x_1.bin",
				bdev->chip_id & 0xffff);

	ret = btmtk_load_code_from_setting_files(btmtk_buffer_mode.file_name, bdev->intf_dev, &code_len, bdev);
	if (ret < 0) {
		BTMTK_ERR("set load %s failed", btmtk_buffer_mode.file_name);
		return;
	}

	memcpy(btmtk_buffer_mode.bt0_mac, &bdev->setting_file[BT0_MAC_OFFSET],
			BUFFER_MODE_MAC_LENGTH);
	memcpy(btmtk_buffer_mode.bt1_mac, &bdev->setting_file[BT1_MAC_OFFSET],
			BUFFER_MODE_MAC_LENGTH);
	memcpy(&btmtk_buffer_mode.bt0_radio, &bdev->setting_file[BT0_RADIO_OFFSET],
			BUFFER_MODE_RADIO_LENGTH);
	memcpy(&btmtk_buffer_mode.bt1_radio, &bdev->setting_file[BT1_RADIO_OFFSET],
			BUFFER_MODE_RADIO_LENGTH);
	memcpy(btmtk_buffer_mode.bt0_ant0_grp_boundary, &bdev->setting_file[BT0_GROUP_ANT0_OFFSET],
			BUFFER_MODE_GROUP_LENGTH);
	memcpy(btmtk_buffer_mode.bt0_ant1_grp_boundary, &bdev->setting_file[BT0_GROUP_ANT1_OFFSET],
			BUFFER_MODE_GROUP_LENGTH);
	memcpy(btmtk_buffer_mode.bt1_ant0_grp_boundary, &bdev->setting_file[BT1_GROUP_ANT0_OFFSET],
			BUFFER_MODE_GROUP_LENGTH);
	memcpy(btmtk_buffer_mode.bt1_ant1_grp_boundary, &bdev->setting_file[BT1_GROUP_ANT1_OFFSET],
			BUFFER_MODE_GROUP_LENGTH);
	memcpy(btmtk_buffer_mode.bt0_ant0_pwr_offset, &bdev->setting_file[BT0_CAL_ANT0_OFFSET],
			BUFFER_MODE_CAL_LENGTH);
	memcpy(btmtk_buffer_mode.bt0_ant1_pwr_offset, &bdev->setting_file[BT0_CAL_ANT1_OFFSET],
			BUFFER_MODE_CAL_LENGTH);
	memcpy(btmtk_buffer_mode.bt1_ant0_pwr_offset, &bdev->setting_file[BT1_CAL_ANT0_OFFSET],
			BUFFER_MODE_CAL_LENGTH);
	memcpy(btmtk_buffer_mode.bt1_ant1_pwr_offset, &bdev->setting_file[BT1_CAL_ANT1_OFFSET],
			BUFFER_MODE_CAL_LENGTH);

	*buffer_mode = &btmtk_buffer_mode;
}

