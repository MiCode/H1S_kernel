/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#ifndef __BTMTK_BUFFER_MODE_H__
#define __BTMTK_BUFFER_MODE_H__

#include "btmtk_main.h"

#define BUFFER_MODE_SWITCH_FILE	"wifi.cfg"
#define BUFFER_MODE_SWITCH_FIELD	"EfuseBufferModeCal"
#define BUFFER_MODE_CFG_FILE		"EEPROM_MT%X_1.bin"
#define EFUSE_MODE			0
#define BIN_FILE_MODE			1
#define AUTO_MODE			2

#define SET_ADDRESS_CMD_LEN 10
#define SET_ADDRESS_EVT_LEN 7
#define SET_ADDRESS_CMD_PAYLOAD_OFFSET 4

#define SET_RADIO_CMD_LEN 12
#define SET_RADIO_EVT_LEN 7
#define SET_RADIO_CMD_EDR_DEF_OFFSET 4
#define SET_RADIO_CMD_BLE_OFFSET 8
#define SET_RADIO_CMD_EDR_MAX_OFFSET 9
#define SET_RADIO_CMD_EDR_MODE_OFFSET 11

#define SET_GRP_CMD_LEN 13
#define SET_GRP_EVT_LEN 7
#define SET_GRP_CMD_PAYLOAD_OFFSET 8

#define SET_PWR_OFFSET_CMD_LEN 14
#define SET_PWR_OFFSET_EVT_LEN 7
#define SET_PWR_OFFSET_CMD_PAYLOAD_OFFSET 8

#define BUFFER_MODE_MAC_LENGTH	6
#define BT0_MAC_OFFSET			0x139
#define BT1_MAC_OFFSET			0x13F

#define BUFFER_MODE_RADIO_LENGTH	4
#define BT0_RADIO_OFFSET		0x145
#define BT1_RADIO_OFFSET		0x149

#define BUFFER_MODE_GROUP_LENGTH	5
#define BT0_GROUP_ANT0_OFFSET		0x984
#define BT0_GROUP_ANT1_OFFSET		0x9BE
#define BT1_GROUP_ANT0_OFFSET		0x9A1
#define BT1_GROUP_ANT1_OFFSET		0x9DB

#define BUFFER_MODE_CAL_LENGTH	6
#define BT0_CAL_ANT0_OFFSET		0x96C
#define BT0_CAL_ANT1_OFFSET		0x9A6
#define BT1_CAL_ANT0_OFFSET		0x989
#define BT1_CAL_ANT1_OFFSET		0x9C3

struct btmtk_buffer_mode_radio_struct {
	u8 radio_0;	/* bit 0-5:edr_init_pwr, 6-7:edr_pwr_mode */
	u8 radio_1;	/* bit 0-5:edr_max_pwr, 6-7:reserved */
	u8 radio_2;	/* bit 0-5:ble_default_pwr, 6-7:reserved */
	u8 radio_3;	/* reserved */
};

struct btmtk_buffer_mode_struct {
	struct btmtk_dev *bdev;

	unsigned char		file_name[MAX_BIN_FILE_NAME_LEN];
	int			efuse_mode;

	u8 bt0_mac[BUFFER_MODE_MAC_LENGTH];
	u8 bt1_mac[BUFFER_MODE_MAC_LENGTH];
	struct btmtk_buffer_mode_radio_struct bt0_radio;
	struct btmtk_buffer_mode_radio_struct bt1_radio;
	u8 bt0_ant0_grp_boundary[BUFFER_MODE_GROUP_LENGTH];
	u8 bt0_ant1_grp_boundary[BUFFER_MODE_GROUP_LENGTH];
	u8 bt1_ant0_grp_boundary[BUFFER_MODE_GROUP_LENGTH];
	u8 bt1_ant1_grp_boundary[BUFFER_MODE_GROUP_LENGTH];
	u8 bt0_ant0_pwr_offset[BUFFER_MODE_CAL_LENGTH];
	u8 bt0_ant1_pwr_offset[BUFFER_MODE_CAL_LENGTH];
	u8 bt1_ant0_pwr_offset[BUFFER_MODE_CAL_LENGTH];
	u8 bt1_ant1_pwr_offset[BUFFER_MODE_CAL_LENGTH];
};

int btmtk_buffer_mode_send(struct btmtk_buffer_mode_struct *buffer_mode);
void btmtk_buffer_mode_initialize(struct btmtk_dev *bdev, struct btmtk_buffer_mode_struct **buffer_mode);
#endif /* __BTMTK_BUFFER_MODE_H__ */

