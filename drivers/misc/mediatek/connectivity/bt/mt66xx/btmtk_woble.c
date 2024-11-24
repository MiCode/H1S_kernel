/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/input.h>
#include <linux/pm_wakeup.h>
#include <linux/interrupt.h>

#include "btmtk_woble.h"

static int is_support_unify_woble(struct btmtk_dev *bdev)
{
	if (bdev->bt_cfg.support_unify_woble) {
		if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id) || is_mt7663(bdev->chip_id))
			return 1;
		else
			return 0;
	} else {
		return 0;
	}
}

static void btmtk_woble_wake_lock(struct btmtk_dev *bdev)
{
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	if (bdev->bt_cfg.support_woble_wakelock) {
		BTMTK_INFO("%s: enter", __func__);
		__pm_stay_awake(bmain_info->woble_ws);
		BTMTK_INFO("%s: exit", __func__);
	}
}

void btmtk_woble_wake_unlock(struct btmtk_dev *bdev)
{
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	if (bdev->bt_cfg.support_woble_wakelock) {
		BTMTK_INFO("%s: enter", __func__);
		__pm_relax(bmain_info->woble_ws);
		BTMTK_INFO("%s: exit", __func__);
	}
}


static int btmtk_send_woble_apcf_reserved(struct btmtk_dev *bdev)
{
	u8 reserve_apcf_cmd[RES_APCF_CMD_LEN] = { 0x01, 0xC9, 0xFC, 0x05, 0x01, 0x30, 0x02, 0x61, 0x02 };
	u8 reserve_apcf_event[RES_APCF_EVT_LEN] = { 0x04, 0xE6, 0x02, 0x08, 0x11 };
	int ret = -1;

	if (bdev == NULL) {
		BTMTK_ERR("%s: Incorrect bdev", __func__);
		return ret;
	}

	if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id))
		ret = btmtk_main_send_cmd(bdev, reserve_apcf_cmd, RES_APCF_CMD_LEN,
			reserve_apcf_event, RES_APCF_EVT_LEN, 0, 0,
			BTMTK_TX_PKT_FROM_HOST);
	else
		BTMTK_WARN("%s: not support for 0x%x", __func__, bdev->chip_id);

	BTMTK_INFO("%s: ret %d", __func__, ret);
	return ret;
}

static int btmtk_send_woble_read_BDADDR_cmd(struct btmtk_dev *bdev)
{
	u8 cmd[READ_ADDRESS_CMD_LEN] = { 0x01, 0x09, 0x10, 0x00 };
	u8 event[READ_ADDRESS_EVT_HDR_LEN] = { 0x04, 0x0E, 0x0A, 0x01, 0x09, 0x10, 0x00, /* AA, BB, CC, DD, EE, FF */ };
	int i;
	int ret = -1;

	BTMTK_INFO("%s: begin", __func__);
	if (bdev == NULL || bdev->io_buf == NULL) {
		BTMTK_ERR("%s: Incorrect bdev", __func__);
		return ret;
	}

	for (i = 0; i < BD_ADDRESS_SIZE; i++) {
		if (bdev->bdaddr[i] != 0) {
			ret = 0;
			goto done;
		}
	}

	ret = btmtk_main_send_cmd(bdev,
			cmd, READ_ADDRESS_CMD_LEN,
			event, READ_ADDRESS_EVT_HDR_LEN,
			0, 0, BTMTK_TX_PKT_FROM_HOST);
	/*BD address will get in btmtk_rx_work*/
	if (ret < 0)
		BTMTK_ERR("%s: failed(%d)", __func__, ret);

done:
	BTMTK_INFO("%s, end, ret = %d", __func__, ret);
	return ret;
}


static int btmtk_send_unify_woble_suspend_default_cmd(struct btmtk_dev *bdev)
{
	u8 cmd[WOBLE_ENABLE_DEFAULT_CMD_LEN] = { 0x01, 0xC9, 0xFC, 0x24, 0x01, 0x20, 0x02, 0x00, 0x01,
		0x02, 0x01, 0x00, 0x05, 0x10, 0x00, 0x00, 0x40, 0x06,
		0x02, 0x40, 0x0A, 0x02, 0x41, 0x0F, 0x05, 0x24, 0x20,
		0x04, 0x32, 0x00, 0x09, 0x26, 0xC0, 0x12, 0x00, 0x00,
		0x12, 0x00, 0x00, 0x00};
	u8 event[WOBLE_ENABLE_DEFAULT_EVT_LEN] = { 0x04, 0xE6, 0x02, 0x08, 0x00 };
	int ret = 0;	/* if successful, 0 */

	BTMTK_INFO("%s: begin", __func__);
	ret = btmtk_main_send_cmd(bdev,
			cmd, WOBLE_ENABLE_DEFAULT_CMD_LEN,
			event, WOBLE_ENABLE_DEFAULT_EVT_LEN,
			0, 0, BTMTK_TX_PKT_FROM_HOST);
	if (ret < 0)
		BTMTK_ERR("%s: failed(%d)", __func__, ret);

	BTMTK_INFO("%s: end. ret = %d", __func__, ret);
	return ret;
}


static int btmtk_send_unify_woble_resume_default_cmd(struct btmtk_dev *bdev)
{
	u8 cmd[WOBLE_DISABLE_DEFAULT_CMD_LEN] = { 0x01, 0xC9, 0xFC, 0x05, 0x01, 0x21, 0x02, 0x00, 0x00 };
	u8 event[WOBLE_DISABLE_DEFAULT_EVT_LEN] = { 0x04, 0xE6, 0x02, 0x08, 0x01 };
	int ret = 0;	/* if successful, 0 */

	BTMTK_INFO("%s: begin", __func__);
	ret = btmtk_main_send_cmd(bdev,
			cmd, WOBLE_DISABLE_DEFAULT_CMD_LEN,
			event, WOBLE_DISABLE_DEFAULT_EVT_LEN,
			0, 0, BTMTK_TX_PKT_FROM_HOST);
	if (ret < 0)
		BTMTK_ERR("%s: failed(%d)", __func__, ret);

	BTMTK_INFO("%s: end. ret = %d", __func__, ret);
	return ret;
}


static int btmtk_send_woble_suspend_cmd(struct btmtk_dev *bdev)
{
	/* radio off cmd with wobx_mode_disable, used when unify woble off */
	u8 radio_off_cmd[RADIO_OFF_CMD_LEN] = { 0x01, 0xC9, 0xFC, 0x05, 0x01, 0x20, 0x02, 0x00, 0x00 };
	u8 event[RADIO_OFF_EVT_LEN] = { 0x04, 0xE6, 0x02, 0x08, 0x00 };
	int ret = 0;	/* if successful, 0 */

	BTMTK_INFO("%s: not support woble, send radio off cmd", __func__);
	ret = btmtk_main_send_cmd(bdev,
			radio_off_cmd, RADIO_OFF_CMD_LEN,
			event, RADIO_OFF_EVT_LEN,
			0, 0, BTMTK_TX_PKT_FROM_HOST);
	if (ret < 0)
		BTMTK_ERR("%s: failed(%d)", __func__, ret);

	return ret;
}

static int btmtk_send_woble_resume_cmd(struct btmtk_dev *bdev)
{
	/* radio on cmd with wobx_mode_disable, used when unify woble off */
	u8 radio_on_cmd[RADIO_ON_CMD_LEN] = { 0x01, 0xC9, 0xFC, 0x05, 0x01, 0x21, 0x02, 0x00, 0x00 };
	u8 event[RADIO_ON_EVT_LEN] = { 0x04, 0xE6, 0x02, 0x08, 0x01 };
	int ret = 0;	/* if successful, 0 */

	BTMTK_INFO("%s: begin", __func__);
	ret = btmtk_main_send_cmd(bdev,
			radio_on_cmd, RADIO_ON_CMD_LEN,
			event, RADIO_ON_EVT_LEN,
			0, 0, BTMTK_TX_PKT_FROM_HOST);
	if (ret < 0)
		BTMTK_ERR("%s: failed(%d)", __func__, ret);

	return ret;
}

static int btmtk_set_Woble_APCF_filter_parameter(struct btmtk_dev *bdev)
{
	u8 cmd[APCF_FILTER_CMD_LEN] = { 0x01, 0x57, 0xFD, 0x0A,
		0x01, 0x00, 0x0A, 0x20, 0x00, 0x20, 0x00, 0x01, 0x80, 0x00 };
	u8 event[APCF_FILTER_EVT_HDR_LEN] = { 0x04, 0x0E, 0x07,
		0x01, 0x57, 0xFD, 0x00, 0x01/*, 00, 63*/ };
	int ret = -1;

	BTMTK_INFO("%s: begin", __func__);
	ret = btmtk_main_send_cmd(bdev, cmd, APCF_FILTER_CMD_LEN,
		event, APCF_FILTER_EVT_HDR_LEN, 0, 0, BTMTK_TX_PKT_FROM_HOST);
	if (ret < 0)
		BTMTK_ERR("%s: end ret %d", __func__, ret);
	else
		ret = 0;

	BTMTK_INFO("%s: end ret=%d", __func__, ret);
	return ret;
}

/**
 * Set APCF manufacturer data and filter parameter
 */
static int btmtk_set_Woble_APCF(struct btmtk_woble *bt_woble)
{
	u8 manufactur_data[APCF_CMD_LEN] = { 0x01, 0x57, 0xFD, 0x27, 0x06, 0x00, 0x0A,
		0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x43, 0x52, 0x4B, 0x54, 0x4D,
		0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	u8 event[APCF_EVT_HDR_LEN] = { 0x04, 0x0E, 0x07, 0x01, 0x57, 0xFD, 0x00, /* 0x06 00 63 */ };
	int ret = -1;
	u8 i = 0;
	struct btmtk_dev *bdev = bt_woble->bdev;

	BTMTK_INFO("%s: woble_setting_apcf[0].length %d",
			__func__, bt_woble->woble_setting_apcf[0].length);

	/* start to send apcf cmd from woble setting file */
	if (bt_woble->woble_setting_apcf[0].length) {
		for (i = 0; i < WOBLE_SETTING_COUNT; i++) {
			if (!bt_woble->woble_setting_apcf[i].length)
				continue;

			BTMTK_INFO("%s: apcf_fill_mac[%d].content[0] = 0x%02x", __func__, i,
					bt_woble->woble_setting_apcf_fill_mac[i].content[0]);
			BTMTK_INFO("%s: apcf_fill_mac_location[%d].length = %d", __func__, i,
					bt_woble->woble_setting_apcf_fill_mac_location[i].length);

			if ((bt_woble->woble_setting_apcf_fill_mac[i].content[0] == 1) &&
				bt_woble->woble_setting_apcf_fill_mac_location[i].length) {
				/* need add BD addr to apcf cmd */
				memcpy(bt_woble->woble_setting_apcf[i].content +
					(*bt_woble->woble_setting_apcf_fill_mac_location[i].content + 1),
					bdev->bdaddr, BD_ADDRESS_SIZE);
				BTMTK_INFO("%s: apcf[%d], add local BDADDR to location %d", __func__, i,
						(*bt_woble->woble_setting_apcf_fill_mac_location[i].content));
			}

			BTMTK_INFO_RAW(bt_woble->woble_setting_apcf[i].content, bt_woble->woble_setting_apcf[i].length,
				"Send woble_setting_apcf[%d] ", i);
			ret = btmtk_main_send_cmd(bdev, bt_woble->woble_setting_apcf[i].content,
				bt_woble->woble_setting_apcf[i].length, event, APCF_EVT_HDR_LEN, 0, 0,
				BTMTK_TX_PKT_FROM_HOST);
			if (ret < 0) {
				BTMTK_ERR("%s: manufactur_data error ret %d", __func__, ret);
				return ret;
			}
		}
	} else { /* use default */
		BTMTK_INFO("%s: use default manufactur data", __func__);
		memcpy(manufactur_data + 10, bdev->bdaddr, BD_ADDRESS_SIZE);
		ret = btmtk_main_send_cmd(bdev, manufactur_data, APCF_CMD_LEN,
			event, APCF_EVT_HDR_LEN, 0, 0, BTMTK_TX_PKT_FROM_HOST);
		if (ret < 0) {
			BTMTK_ERR("%s: manufactur_data error ret %d", __func__, ret);
			return ret;
		}

		ret = btmtk_set_Woble_APCF_filter_parameter(bdev);
	}

	BTMTK_INFO("%s: end ret=%d", __func__, ret);
	return 0;
}

static int btmtk_set_Woble_Radio_Off(struct btmtk_woble *bt_woble)
{
	int ret = -1;
	int length = 0;
	char *radio_off = NULL;
	struct btmtk_dev *bdev = bt_woble->bdev;

	BTMTK_INFO("%s: woble_setting_radio_off.length %d", __func__,
		bt_woble->woble_setting_radio_off.length);
	if (bt_woble->woble_setting_radio_off.length) {
		/* start to send radio off cmd from woble setting file */
		length = bt_woble->woble_setting_radio_off.length +
				bt_woble->woble_setting_wakeup_type.length;
		radio_off = kzalloc(length, GFP_KERNEL);
		if (!radio_off) {
			BTMTK_ERR("%s: alloc memory fail (radio_off)",
				__func__);
			ret = -ENOMEM;
			goto Finish;
		}

		memcpy(radio_off,
			bt_woble->woble_setting_radio_off.content,
			bt_woble->woble_setting_radio_off.length);
		if (bt_woble->woble_setting_wakeup_type.length) {
			memcpy(radio_off + bt_woble->woble_setting_radio_off.length,
				bt_woble->woble_setting_wakeup_type.content,
				bt_woble->woble_setting_wakeup_type.length);
			radio_off[3] += bt_woble->woble_setting_wakeup_type.length;
		}

		BTMTK_INFO_RAW(radio_off, length, "Send radio off");
		ret = btmtk_main_send_cmd(bdev, radio_off, length,
			bt_woble->woble_setting_radio_off_comp_event.content,
			bt_woble->woble_setting_radio_off_comp_event.length, 0, 0,
			BTMTK_TX_PKT_FROM_HOST);

		kfree(radio_off);
		radio_off = NULL;
	} else { /* use default */
		BTMTK_INFO("%s: use default radio off cmd", __func__);
		ret = btmtk_send_unify_woble_suspend_default_cmd(bdev);
	}

Finish:
	BTMTK_INFO("%s, end ret=%d", __func__, ret);
	return ret;
}

static int btmtk_set_Woble_Radio_On(struct btmtk_woble *bt_woble)
{
	int ret = -1;
	struct btmtk_dev *bdev = bt_woble->bdev;

	BTMTK_INFO("%s: woble_setting_radio_on.length %d", __func__,
		bt_woble->woble_setting_radio_on.length);
	if (bt_woble->woble_setting_radio_on.length) {
		/* start to send radio on cmd from woble setting file */
		BTMTK_INFO_RAW(bt_woble->woble_setting_radio_on.content,
			bt_woble->woble_setting_radio_on.length, "send radio on");

		ret = btmtk_main_send_cmd(bdev, bt_woble->woble_setting_radio_on.content,
			bt_woble->woble_setting_radio_on.length,
			bt_woble->woble_setting_radio_on_comp_event.content,
			bt_woble->woble_setting_radio_on_comp_event.length, 0, 0,
			BTMTK_TX_PKT_FROM_HOST);
	} else { /* use default */
		BTMTK_WARN("%s: use default radio on cmd", __func__);
		ret = btmtk_send_unify_woble_resume_default_cmd(bdev);
	}

	BTMTK_INFO("%s, end ret=%d", __func__, ret);
	return ret;
}

static int btmtk_del_Woble_APCF_index(struct btmtk_dev *bdev)
{
	u8 cmd[APCF_DELETE_CMD_LEN] = { 0x01, 0x57, 0xFD, 0x03, 0x01, 0x01, 0x0A };
	u8 event[APCF_DELETE_EVT_HDR_LEN] = { 0x04, 0x0e, 0x07, 0x01, 0x57, 0xfd, 0x00, 0x01, /* 00, 63 */ };
	int ret = -1;

	BTMTK_INFO("%s, enter", __func__);
	ret = btmtk_main_send_cmd(bdev,
			cmd, APCF_DELETE_CMD_LEN,
			event, APCF_DELETE_EVT_HDR_LEN,
			0, 0, BTMTK_TX_PKT_FROM_HOST);
	if (ret < 0)
		BTMTK_ERR("%s: got error %d", __func__, ret);

	BTMTK_INFO("%s, end", __func__);
	return ret;
}


static int btmtk_set_Woble_APCF_Resume(struct btmtk_woble *bt_woble)
{
	u8 event[APCF_RESUME_EVT_HDR_LEN] = { 0x04, 0x0e, 0x07, 0x01, 0x57, 0xfd, 0x00 };
	u8 i = 0;
	int ret = -1;
	struct btmtk_dev *bdev = bt_woble->bdev;

	BTMTK_INFO("%s, enter, bt_woble->woble_setting_apcf_resume[0].length= %d",
			__func__, bt_woble->woble_setting_apcf_resume[0].length);
	if (bt_woble->woble_setting_apcf_resume[0].length) {
		BTMTK_INFO("%s: handle leave woble apcf from file", __func__);
		for (i = 0; i < WOBLE_SETTING_COUNT; i++) {
			if (!bt_woble->woble_setting_apcf_resume[i].length)
				continue;

			BTMTK_INFO_RAW(bt_woble->woble_setting_apcf_resume[i].content,
				bt_woble->woble_setting_apcf_resume[i].length,
				"%s: send apcf resume %d:", __func__, i);

			ret = btmtk_main_send_cmd(bdev,
				bt_woble->woble_setting_apcf_resume[i].content,
				bt_woble->woble_setting_apcf_resume[i].length,
				event, APCF_RESUME_EVT_HDR_LEN,
				0, 0, BTMTK_TX_PKT_FROM_HOST);
			if (ret < 0) {
				BTMTK_ERR("%s: Send apcf resume fail %d", __func__, ret);
				return ret;
			}
		}
	} else { /* use default */
		BTMTK_WARN("%s: use default apcf resume cmd", __func__);
		ret = btmtk_del_Woble_APCF_index(bdev);
		if (ret < 0)
			BTMTK_ERR("%s: btmtk_del_Woble_APCF_index return fail %d", __func__, ret);
	}
	BTMTK_INFO("%s, end", __func__);
	return ret;
}


static int btmtk_load_woble_setting(char *bin_name,
		struct device *dev, u32 *code_len, struct btmtk_woble *bt_woble)
{
	int err;
	struct btmtk_dev *bdev = bt_woble->bdev;

	*code_len = 0;

	err = btmtk_load_code_from_setting_files(bin_name, dev, code_len, bdev);
	if (err) {
		BTMTK_ERR("woble_setting btmtk_load_code_from_setting_files failed!!");
		goto LOAD_END;
	}

	err = btmtk_load_fw_cfg_setting("APCF",
			bt_woble->woble_setting_apcf, WOBLE_SETTING_COUNT, bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	err = btmtk_load_fw_cfg_setting("APCF_ADD_MAC",
			bt_woble->woble_setting_apcf_fill_mac, WOBLE_SETTING_COUNT,
			bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	err = btmtk_load_fw_cfg_setting("APCF_ADD_MAC_LOCATION",
			bt_woble->woble_setting_apcf_fill_mac_location, WOBLE_SETTING_COUNT,
			bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	err = btmtk_load_fw_cfg_setting("RADIOOFF", &bt_woble->woble_setting_radio_off, 1,
			bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	switch (bdev->bt_cfg.unify_woble_type) {
	case 0:
		err = btmtk_load_fw_cfg_setting("WAKEUP_TYPE_LEGACY", &bt_woble->woble_setting_wakeup_type, 1,
			bdev->setting_file, FW_CFG_INX_LEN_2);
		break;
	case 1:
		err = btmtk_load_fw_cfg_setting("WAKEUP_TYPE_WAVEFORM", &bt_woble->woble_setting_wakeup_type, 1,
			bdev->setting_file, FW_CFG_INX_LEN_2);
		break;
	case 2:
		err = btmtk_load_fw_cfg_setting("WAKEUP_TYPE_IR", &bt_woble->woble_setting_wakeup_type, 1,
			bdev->setting_file, FW_CFG_INX_LEN_2);
		break;
	default:
		BTMTK_WARN("%s: unify_woble_type unknown(%d)", __func__, bdev->bt_cfg.unify_woble_type);
	}
	if (err)
		BTMTK_WARN("%s: Parse unify_woble_type(%d) failed", __func__, bdev->bt_cfg.unify_woble_type);

	err = btmtk_load_fw_cfg_setting("RADIOOFF_STATUS_EVENT",
			&bt_woble->woble_setting_radio_off_status_event, 1, bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	err = btmtk_load_fw_cfg_setting("RADIOOFF_COMPLETE_EVENT",
			&bt_woble->woble_setting_radio_off_comp_event, 1, bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	err = btmtk_load_fw_cfg_setting("RADIOON",
			&bt_woble->woble_setting_radio_on, 1, bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	err = btmtk_load_fw_cfg_setting("RADIOON_STATUS_EVENT",
			&bt_woble->woble_setting_radio_on_status_event, 1, bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	err = btmtk_load_fw_cfg_setting("RADIOON_COMPLETE_EVENT",
			&bt_woble->woble_setting_radio_on_comp_event, 1, bdev->setting_file, FW_CFG_INX_LEN_2);
	if (err)
		goto LOAD_END;

	err = btmtk_load_fw_cfg_setting("APCF_RESUME",
			bt_woble->woble_setting_apcf_resume, WOBLE_SETTING_COUNT, bdev->setting_file, FW_CFG_INX_LEN_2);

LOAD_END:
	/* release setting file memory */
	if (bdev) {
		kfree(bdev->setting_file);
		bdev->setting_file = NULL;
	}

	if (err)
		BTMTK_ERR("%s: error return %d", __func__, err);

	return err;
}

static void btmtk_check_wobx_debug_log(struct btmtk_dev *bdev)
{
	/* 0xFF, 0xFF, 0xFF, 0xFF is log level */
	u8 cmd[CHECK_WOBX_DEBUG_CMD_LEN] = { 0X01, 0xCE, 0xFC, 0x04, 0xFF, 0xFF, 0xFF, 0xFF };
	u8 event[CHECK_WOBX_DEBUG_EVT_HDR_LEN] = { 0x04, 0xE8 };
	int ret = -1;

	BTMTK_INFO("%s: begin", __func__);

	ret = btmtk_main_send_cmd(bdev,
		cmd, CHECK_WOBX_DEBUG_CMD_LEN,
		event, CHECK_WOBX_DEBUG_EVT_HDR_LEN,
		0, 0, BTMTK_TX_PKT_FROM_HOST);
	if (ret < 0)
		BTMTK_ERR("%s: failed(%d)", __func__, ret);

	/* Driver just print event to kernel log in rx_work,
	 * Please reference wiki to know what it is.
	 */
}


static int btmtk_handle_leaving_WoBLE_state(struct btmtk_woble *bt_woble)
{
	int ret = -1;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	struct btmtk_dev *bdev = bt_woble->bdev;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	BTMTK_INFO("%s: begin", __func__);
	fstate = btmtk_fops_get_state(bdev);
	if (!bdev->bt_cfg.support_woble_for_bt_disable) {
		if (fstate != BTMTK_FOPS_STATE_OPENED) {
			BTMTK_WARN("%s: fops is not opened, return", __func__);
			return 0;
		}
	}

	if (fstate != BTMTK_FOPS_STATE_OPENED) {
		BTMTK_WARN("%s: fops is not open yet(%d), need to start traffic before leaving woble",
				__func__, fstate);
		/* start traffic to recv event*/
		ret = bmain_info->hif_hook.open(bdev->hdev);
		if (ret < 0) {
			BTMTK_ERR("%s, cif_open failed", __func__);
			goto Finish;
		}
	}

	if (is_support_unify_woble(bdev)) {
		ret = btmtk_set_Woble_Radio_On(bt_woble);
		if (ret < 0)
			goto Finish;

		ret = btmtk_set_Woble_APCF_Resume(bt_woble);
		if (ret < 0)
			goto Finish;
	} else {
		/* radio on cmd with wobx_mode_disable, used when unify woble off */
		ret = btmtk_send_woble_resume_cmd(bdev);
	}

Finish:
	if (ret < 0) {
		BTMTK_INFO("%s: woble_resume_fail!!!", __func__);
	} else {
		/* It's wobx debug log method. */
		btmtk_check_wobx_debug_log(bdev);

		if (fstate != BTMTK_FOPS_STATE_OPENED) {
			ret = btmtk_send_deinit_cmds(bdev);
			if (ret < 0) {
				BTMTK_ERR("%s, btmtk_send_deinit_cmds failed", __func__);
				goto exit;
			}

			BTMTK_WARN("%s: fops is not open(%d), need to stop traffic after leaving woble",
					__func__, fstate);
			/* stop traffic to stop recv data from fw*/
			ret = bmain_info->hif_hook.close(bdev->hdev);
			if (ret < 0) {
				BTMTK_ERR("%s, cif_close failed", __func__);
				goto exit;
			}
		} else
			bdev->power_state = BTMTK_DONGLE_STATE_POWER_ON;
		BTMTK_INFO("%s: success", __func__);
	}

exit:
	BTMTK_INFO("%s: end", __func__);
	return ret;
}

static int btmtk_handle_entering_WoBLE_state(struct btmtk_woble *bt_woble)
{
	int ret = -1;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	int state = BTMTK_STATE_INIT;
	struct btmtk_dev *bdev = bt_woble->bdev;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	BTMTK_INFO("%s: begin", __func__);

	fstate = btmtk_fops_get_state(bdev);
	if (!bdev->bt_cfg.support_woble_for_bt_disable) {
		if (fstate != BTMTK_FOPS_STATE_OPENED) {
			BTMTK_WARN("%s: fops is not open yet(%d)!, return", __func__, fstate);
			return 0;
		}
	}

	state = btmtk_get_chip_state(bdev);
	if (state == BTMTK_STATE_FW_DUMP) {
		BTMTK_WARN("%s: FW dumping ongoing, don't send any cmd to FW!!!", __func__);
		goto Finish;
	}

	if (bdev->chip_reset || bdev->subsys_reset) {
		BTMTK_ERR("%s chip_reset is %d, subsys_reset is %d", __func__,
			bdev->chip_reset, bdev->subsys_reset);
		goto Finish;
	}

	/* Power on first if state is power off */
	ret = btmtk_reset_power_on(bdev);
	if (ret < 0) {
		BTMTK_ERR("%s: reset power_on fail return", __func__);
		goto Finish;
	}

	if (fstate != BTMTK_FOPS_STATE_OPENED) {
		BTMTK_WARN("%s: fops is not open yet(%d), need to start traffic before enter woble",
				__func__, fstate);
		/* start traffic to recv event*/
		ret = bmain_info->hif_hook.open(bdev->hdev);
		if (ret < 0) {
			BTMTK_ERR("%s, cif_open failed", __func__);
			goto Finish;
		}
	}
	if (is_support_unify_woble(bdev)) {
		do {
			typedef ssize_t (*func) (u16 u16Key, const char *buf, size_t size);
			char *func_name = "MDrv_PM_Write_Key";
			func pFunc = NULL;
			ssize_t sret = 0;
			u8 buf = 0;

			pFunc = (func) btmtk_kallsyms_lookup_name(func_name);
			if (pFunc && bdev->bt_cfg.unify_woble_type == 1) {
				buf = 1;
				sret = pFunc(PM_KEY_BTW, &buf, sizeof(u8));
				BTMTK_INFO("%s: Invoke %s, buf = %d, sret = %zd", __func__,
					func_name, buf, sret);

			} else {
				BTMTK_WARN("%s: No Exported Func Found [%s]", __func__, func_name);
			}
		} while (0);

		ret = btmtk_send_woble_apcf_reserved(bdev);
		if (ret < 0)
			goto STOP_TRAFFIC;

		ret = btmtk_send_woble_read_BDADDR_cmd(bdev);
		if (ret < 0)
			goto STOP_TRAFFIC;

		ret = btmtk_set_Woble_APCF(bt_woble);
		if (ret < 0)
			goto STOP_TRAFFIC;

		ret = btmtk_set_Woble_Radio_Off(bt_woble);
		if (ret < 0)
			goto STOP_TRAFFIC;
	} else {
		/* radio off cmd with wobx_mode_disable, used when unify woble off */
		ret = btmtk_send_woble_suspend_cmd(bdev);
	}

STOP_TRAFFIC:
	if (fstate != BTMTK_FOPS_STATE_OPENED) {
		BTMTK_WARN("%s: fops is not open(%d), need to stop traffic after enter woble",
				__func__, fstate);
		/* stop traffic to stop recv data from fw*/
		ret = bmain_info->hif_hook.close(bdev->hdev);
		if (ret < 0) {
			BTMTK_ERR("%s, cif_close failed", __func__);
			goto Finish;
		}
	}

Finish:
	if (ret) {
		bdev->power_state = BTMTK_DONGLE_STATE_ERROR;
		btmtk_woble_wake_lock(bdev);
	}

	BTMTK_INFO("%s: end ret = %d, power_state =%d", __func__, ret, bdev->power_state);
	return ret;
}

int btmtk_woble_suspend(struct btmtk_woble *bt_woble)
{
	int ret = 0;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	struct btmtk_dev *bdev = bt_woble->bdev;

	BTMTK_INFO("%s: enter", __func__);

	fstate = btmtk_fops_get_state(bdev);

	if (!is_support_unify_woble(bdev) && (fstate != BTMTK_FOPS_STATE_OPENED)) {
		BTMTK_WARN("%s: when not support woble, in bt off state, do nothing!", __func__);
		goto exit;
	}

	ret = btmtk_handle_entering_WoBLE_state(bt_woble);
	if (ret)
		BTMTK_ERR("%s: btmtk_handle_entering_WoBLE_state return fail %d", __func__, ret);

	if (bdev->bt_cfg.support_woble_by_eint) {
		if (bt_woble->wobt_irq != 0 && atomic_read(&(bt_woble->irq_enable_count)) == 0) {
			BTMTK_INFO("enable BT IRQ:%d", bt_woble->wobt_irq);
			irq_set_irq_wake(bt_woble->wobt_irq, 1);
			enable_irq(bt_woble->wobt_irq);
			atomic_inc(&(bt_woble->irq_enable_count));
		} else
			BTMTK_INFO("irq_enable count:%d", atomic_read(&(bt_woble->irq_enable_count)));
	}

exit:
	BTMTK_INFO("%s: end", __func__);
	return ret;
}

int btmtk_woble_resume(struct btmtk_woble *bt_woble)
{
	int ret = -1;
	unsigned char fstate = BTMTK_FOPS_STATE_INIT;
	struct btmtk_dev *bdev = bt_woble->bdev;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	BTMTK_INFO("%s: enter", __func__);
	fstate = btmtk_fops_get_state(bdev);

	if (!is_support_unify_woble(bdev) && (fstate != BTMTK_FOPS_STATE_OPENED)) {
		BTMTK_WARN("%s: when not support woble, in bt off state, do nothing!", __func__);
		goto exit;
	}

	if (bdev->power_state == BTMTK_DONGLE_STATE_ERROR) {
		BTMTK_INFO("%s: In BTMTK_DONGLE_STATE_ERROR(Could suspend caused), do assert", __func__);
		btmtk_send_assert_cmd(bdev);
		ret = -EBADFD;
		goto exit;
	}


	if (bdev->bt_cfg.support_woble_by_eint) {
		if (bt_woble->wobt_irq != 0 && atomic_read(&(bt_woble->irq_enable_count)) == 1) {
			BTMTK_INFO("disable BT IRQ:%d", bt_woble->wobt_irq);
			atomic_dec(&(bt_woble->irq_enable_count));
			disable_irq_nosync(bt_woble->wobt_irq);
		} else
			BTMTK_INFO("irq_enable count:%d", atomic_read(&(bt_woble->irq_enable_count)));
	}

	ret = btmtk_handle_leaving_WoBLE_state(bt_woble);
	if (ret < 0) {
		BTMTK_ERR("%s: btmtk_handle_leaving_WoBLE_state return fail %d", __func__, ret);
		/* avoid rtc to to suspend again, do FW dump first */
		btmtk_woble_wake_lock(bdev);
		btmtk_send_assert_cmd(bdev);
		goto exit;
	}

	if (bdev->bt_cfg.reset_stack_after_woble
		&& bmain_info->reset_stack_flag == HW_ERR_NONE
		&& fstate == BTMTK_FOPS_STATE_OPENED)
		bmain_info->reset_stack_flag = HW_ERR_CODE_RESET_STACK_AFTER_WOBLE;

	btmtk_send_hw_err_to_host(bdev);
	BTMTK_INFO("%s: end(%d), reset_stack_flag = %d, fstate = %d", __func__, ret,
			bmain_info->reset_stack_flag, fstate);

exit:
	BTMTK_INFO("%s: end", __func__);
	return ret;
}

static irqreturn_t btmtk_woble_isr(int irq, struct btmtk_woble *bt_woble)
{
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	BTMTK_DBG("%s begin", __func__);
	disable_irq_nosync(bt_woble->wobt_irq);
	atomic_dec(&(bt_woble->irq_enable_count));
	BTMTK_INFO("disable BT IRQ, call wake lock");
	__pm_wakeup_event(bmain_info->eint_ws, WAIT_POWERKEY_TIMEOUT);

	input_report_key(bt_woble->WoBLEInputDev, KEY_WAKEUP, 1);
	input_sync(bt_woble->WoBLEInputDev);
	input_report_key(bt_woble->WoBLEInputDev, KEY_WAKEUP, 0);
	input_sync(bt_woble->WoBLEInputDev);
	BTMTK_DBG("%s end", __func__);
	return IRQ_HANDLED;
}

static int btmtk_RegisterBTIrq(struct btmtk_woble *bt_woble)
{
	struct device_node *eint_node = NULL;
	int interrupts[2];

	BTMTK_DBG("%s begin", __func__);
	eint_node = of_find_compatible_node(NULL, NULL, "mediatek,woble_eint");
	if (eint_node) {
		BTMTK_INFO("Get woble_eint compatible node");
		bt_woble->wobt_irq = irq_of_parse_and_map(eint_node, 0);
		BTMTK_INFO("woble_irq number:%d", bt_woble->wobt_irq);
		if (bt_woble->wobt_irq) {
			of_property_read_u32_array(eint_node, "interrupts",
						   interrupts, ARRAY_SIZE(interrupts));
			bt_woble->wobt_irqlevel = interrupts[1];
			if (request_irq(bt_woble->wobt_irq, (void *)btmtk_woble_isr,
					bt_woble->wobt_irqlevel, "woble-eint", bt_woble->bdev))
				BTMTK_INFO("WOBTIRQ LINE NOT AVAILABLE!!");
			else {
				BTMTK_INFO("disable BT IRQ");
				disable_irq_nosync(bt_woble->wobt_irq);
			}

		} else
			BTMTK_INFO("can't find woble_eint irq");

	} else {
		bt_woble->wobt_irq = 0;
		BTMTK_INFO("can't find woble_eint compatible node");
	}

	BTMTK_DBG("%s end", __func__);
	return 0;
}

static int btmtk_woble_input_init(struct btmtk_woble *bt_woble)
{
	int ret = 0;

	bt_woble->WoBLEInputDev = input_allocate_device();
	if (!bt_woble->WoBLEInputDev || IS_ERR(bt_woble->WoBLEInputDev)) {
		BTMTK_ERR("input_allocate_device error");
		return -ENOMEM;
	}

	bt_woble->WoBLEInputDev->name = "WOBLE_INPUT_DEVICE";
	bt_woble->WoBLEInputDev->id.bustype = BUS_HOST;
	bt_woble->WoBLEInputDev->id.vendor = 0x0002;
	bt_woble->WoBLEInputDev->id.product = 0x0002;
	bt_woble->WoBLEInputDev->id.version = 0x0002;

	__set_bit(EV_KEY, bt_woble->WoBLEInputDev->evbit);
	__set_bit(KEY_WAKEUP, bt_woble->WoBLEInputDev->keybit);

	ret = input_register_device(bt_woble->WoBLEInputDev);
	if (ret < 0) {
		input_free_device(bt_woble->WoBLEInputDev);
		BTMTK_ERR("input_register_device %d", ret);
		return ret;
	}

	return ret;
}

static void btmtk_woble_input_deinit(struct btmtk_woble *bt_woble)
{
	if (bt_woble->WoBLEInputDev) {
		input_unregister_device(bt_woble->WoBLEInputDev);
		input_free_device(bt_woble->WoBLEInputDev);
		bt_woble->WoBLEInputDev = NULL;
	}
}

static void btmtk_free_woble_setting_file(struct btmtk_woble *bt_woble)
{
	btmtk_free_fw_cfg_struct(bt_woble->woble_setting_apcf, WOBLE_SETTING_COUNT);
	btmtk_free_fw_cfg_struct(bt_woble->woble_setting_apcf_fill_mac, WOBLE_SETTING_COUNT);
	btmtk_free_fw_cfg_struct(bt_woble->woble_setting_apcf_fill_mac_location, WOBLE_SETTING_COUNT);
	btmtk_free_fw_cfg_struct(bt_woble->woble_setting_apcf_resume, WOBLE_SETTING_COUNT);
	btmtk_free_fw_cfg_struct(&bt_woble->woble_setting_radio_off, 1);
	btmtk_free_fw_cfg_struct(&bt_woble->woble_setting_radio_off_status_event, 1);
	btmtk_free_fw_cfg_struct(&bt_woble->woble_setting_radio_off_comp_event, 1);
	btmtk_free_fw_cfg_struct(&bt_woble->woble_setting_radio_on, 1);
	btmtk_free_fw_cfg_struct(&bt_woble->woble_setting_radio_on_status_event, 1);
	btmtk_free_fw_cfg_struct(&bt_woble->woble_setting_radio_on_comp_event, 1);
	btmtk_free_fw_cfg_struct(&bt_woble->woble_setting_wakeup_type, 1);

	bt_woble->woble_setting_len = 0;

	kfree(bt_woble->woble_setting_file_name);
	bt_woble->woble_setting_file_name = NULL;
}

int btmtk_woble_initialize(struct btmtk_dev *bdev, struct btmtk_woble *bt_woble)
{
	int err = 0;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();

	bt_woble->bdev = bdev;
	/* Need to add Woble flow */
	if (is_support_unify_woble(bdev)) {
		if (bt_woble->woble_setting_file_name == NULL) {
			bt_woble->woble_setting_file_name = kzalloc(MAX_BIN_FILE_NAME_LEN, GFP_KERNEL);
			if (!bt_woble->woble_setting_file_name) {
				BTMTK_ERR("%s: alloc memory fail (bt_woble->woble_setting_file_name)", __func__);
				err = -1;
				goto end;
			}
		}

		if (is_mt7663(bdev->chip_id))
			memcpy(bt_woble->woble_setting_file_name, WOBLE_SETTING_FILE_NAME_7663,
				sizeof(WOBLE_SETTING_FILE_NAME_7663));

		if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id))
			memcpy(bt_woble->woble_setting_file_name, WOBLE_SETTING_FILE_NAME_7961,
				sizeof(WOBLE_SETTING_FILE_NAME_7961));

		BTMTK_INFO("%s: woble setting file name is %s", __func__, bt_woble->woble_setting_file_name);

		btmtk_load_woble_setting(bt_woble->woble_setting_file_name,
			bdev->intf_dev,
			&bt_woble->woble_setting_len,
			bt_woble);
		/* if reset_stack is true, when chip reset is done, we need to power on chip to do
		 * reset stack
		 */
		if (bmain_info->reset_stack_flag) {
			err = btmtk_reset_power_on(bdev);
			if (err < 0) {
				BTMTK_ERR("reset power on failed!");
				goto err;
			}
		}
	}

	if (bdev->bt_cfg.support_woble_by_eint) {
		btmtk_woble_input_init(bt_woble);
		btmtk_RegisterBTIrq(bt_woble);
	}

	return 0;

err:
	btmtk_free_woble_setting_file(bt_woble);
end:
	return err;
}

void btmtk_woble_uninitialize(struct btmtk_woble *bt_woble)
{
	struct btmtk_dev *bdev = bt_woble->bdev;

	if (bdev == NULL) {
		BTMTK_ERR("%s: bdev == NULL", __func__);
		return;
	}

	BTMTK_INFO("%s begin", __func__);
	if (bdev->bt_cfg.support_woble_by_eint) {
		if (bt_woble->wobt_irq != 0 && atomic_read(&(bt_woble->irq_enable_count)) == 1) {
			BTMTK_INFO("disable BT IRQ:%d", bt_woble->wobt_irq);
			atomic_dec(&(bt_woble->irq_enable_count));
			disable_irq_nosync(bt_woble->wobt_irq);
		} else
			BTMTK_INFO("irq_enable count:%d", atomic_read(&(bt_woble->irq_enable_count)));
		free_irq(bt_woble->wobt_irq, bdev);

		btmtk_woble_input_deinit(bt_woble);
	}

	btmtk_free_woble_setting_file(bt_woble);
	bt_woble->bdev = NULL;
}
