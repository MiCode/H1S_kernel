/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __LD_BTMTK_USB_H__
#define __LD_BTMTK_USB_H__

#include <mtk-bt/LD_usbbt.h>

/* Memory map for MTK BT */
//#if 0
/* SYS Control */
#define SYSCTL	0x400000

/* WLAN */
#define WLAN		0x410000

/* MCUCTL */
#define CLOCK_CTL		0x0708
#define INT_LEVEL		0x0718
#define COM_REG0		0x0730
#define SEMAPHORE_00	0x07B0
#define SEMAPHORE_01	0x07B4
#define SEMAPHORE_02	0x07B8
#define SEMAPHORE_03	0x07BC

/* Chip definition */

#define CONTROL_TIMEOUT_JIFFIES		(300)
#define DEVICE_VENDOR_REQUEST_OUT	0x40
#define DEVICE_VENDOR_REQUEST_IN	0xc0
#define DEVICE_CLASS_REQUEST_OUT	0x20
#define DEVICE_CLASS_REQUEST_IN		0xa0

#define BTUSB_MAX_ISOC_FRAMES	10
#define BTUSB_INTR_RUNNING	0
#define BTUSB_BULK_RUNNING	1
#define BTUSB_ISOC_RUNNING	2
#define BTUSB_SUSPENDING	3
#define BTUSB_DID_ISO_RESUME	4

/* ROM Patch */
#define PATCH_HCI_HEADER_SIZE_BULK_EP 4
#define PATCH_HCI_HEADER_SIZE_CTRL_EP 3
#define PATCH_WMT_HEADER_SIZE 5
#define PATCH_HEADER_SIZE_BULK_EP (PATCH_WMT_HEADER_SIZE + PATCH_HCI_HEADER_SIZE_BULK_EP)
#define PATCH_HEADER_SIZE_CTRL_EP (PATCH_WMT_HEADER_SIZE + PATCH_HCI_HEADER_SIZE_CTRL_EP)
#define UPLOAD_PATCH_UNIT 512
#define PATCH_INFO_SIZE 30
#define PATCH_PHASE1 1
#define PATCH_PHASE2 2
#define PATCH_PHASE3 3
#define PATCH_LEN_ILM (192 * 1024)

#define BUZZARD_CHIP_ID 0x70010200
#define BUZZARD_FLAVOR 0x70010020
#define BUZZARD_FW_VERSION 0x80021004

/**
 * 0: patch download is not complete/BT get patch semaphore fail (WiFi get semaphore success)
 * 1: patch download is complete
 * 2: patch download is not complete/BT get patch semaphore success
 */
#define PATCH_ERR -1
#define PATCH_IS_DOWNLOAD_BY_OTHER 0
#define PATCH_READY 1
#define PATCH_NEED_DOWNLOAD 2

#define MAX_BIN_FILE_NAME_LEN 64
#define LD_BT_MAX_EVENT_SIZE 260
#define BD_ADDR_LEN 6

#define WOBLE_SETTING_FILE_NAME_7961 "woble_setting_7961.bin"
#define WOBLE_SETTING_FILE_NAME_7668 "woble_setting_7668.bin"
#define WOBLE_SETTING_FILE_NAME_7663 "woble_setting_7663.bin"
#define WOBLE_SETTING_FILE_NAME "woble_setting.bin"
#define BT_CFG_NAME "bt.cfg"
#define BT_UNIFY_WOBLE "SUPPORT_UNIFY_WOBLE"
#define BT_UNIFY_WOBLE_TYPE "UNIFY_WOBLE_TYPE"
#define BT_WMT_CMD "WMT_CMD"

#define WMT_CMD_COUNT 255

#define WAKE_DEV_RECORD		 "wake_on_ble.conf"
#define WAKE_DEV_RECORD_PATH	"misc/bluedroid"
#define APCF_SETTING_COUNT	10
#define WOBLE_SETTING_COUNT	10

/* It is for mt7961 download rom patch*/
#define FW_ROM_PATCH_HEADER_SIZE	32
#define FW_ROM_PATCH_GD_SIZE	64
#define FW_ROM_PATCH_SEC_MAP_SIZE	64
#define SEC_MAP_NEED_SEND_SIZE	52
#define PATCH_STATUS	6
#define SECTION_SPEC_NUM	13
#define WMT_HEADER_LEN 4
#define LOAD_PATCH_PHASE_LEN 1

/* this for 79XX need download patch staus
 * 0:
 * patch download is not complete, BT driver need to download patch
 * 1:
 * patch is downloading by Wifi,BT driver need to retry until status = PATCH_READY
 * 2:
 * patch download is complete, BT driver no need to download patch
 */
#define BUZZARD_PATCH_ERR -1
#define BUZZARD_PATCH_NEED_DOWNLOAD 0
#define BUZZARD_PATCH_IS_DOWNLOAD_BY_OTHER 1
#define BUZZARD_PATCH_READY 2

/* 0:
 * using legacy wmt cmd/evt to download fw patch, usb/sdio just support 0 now
 * 1:
 * using DMA to download fw patch
 */
#define PATCH_DOWNLOAD_USING_WMT 0
#define PATCH_DOWNLOAD_USING_DMA 1


#define PATCH_DOWNLOAD_CMD_DELAY_TIME 5
#define PATCH_DOWNLOAD_CMD_RETRY 0
#define PATCH_DOWNLOAD_PHASE1_2_DELAY_TIME 1
#define PATCH_DOWNLOAD_PHASE1_2_RETRY 5
#define PATCH_DOWNLOAD_PHASE3_DELAY_TIME 20
#define PATCH_DOWNLOAD_PHASE3_RETRY 20


enum {
	BTMTK_EP_TYPE_OUT_CMD = 0,	/*EP type out for hci cmd and wmt cmd */
	BTMTK_EP_TPYE_OUT_ACL,	/* EP type out for acl pkt with load rompatch */
};

typedef enum {
	TYPE_APCF_CMD,
} woble_setting_type;

enum fw_cfg_index_len {
	FW_CFG_INX_LEN_NONE = 0,
	FW_CFG_INX_LEN_2 = 2,
	FW_CFG_INX_LEN_3 = 3,
};

struct fw_cfg_struct {
	u8	*content;	/* APCF content or radio off content */
	int	length;		/* APCF content or radio off content of length */
};

#define UNIFY_WOBLE_LEGACY 0
#define UNIFY_WOBLE_WAVEFORM 1
struct bt_cfg_struct {
	u8	support_unify_woble;	/* support unify woble or not */
	u8	unify_woble_type;	/* 0: legacy. 1: waveform. 2: IR */
	struct fw_cfg_struct wmt_cmd[WMT_CMD_COUNT];
};

struct LD_btmtk_usb_data {
	mtkbt_dev_t *udev; /* store the usb device informaiton */

	unsigned long flags;
	int meta_tx;
	HC_IF *hcif;

	u8 cmdreq_type;

	unsigned int sco_num;
	int isoc_altsetting;
	int suspend_count;

	/* request for different io operation */
	u8 w_request;
	u8 r_request;

	/* io buffer for usb control transfer */
	unsigned char *io_buf;

	unsigned char *fw_image;
	unsigned char *fw_header_image;
	unsigned char *fw_bin_file_name;

	unsigned char *rom_patch;
	unsigned char *rom_patch_header_image;
	unsigned char *rom_patch_bin_file_name;
	u32 chip_id;
	unsigned int	flavor;
	unsigned int	fw_version;
	u8 need_load_fw;
	u8 need_load_rom_patch;
	u32 rom_patch_offset;
	u32 rom_patch_len;
	u32 fw_len;
	int recv_evt_len;

	u8 local_addr[BD_ADDR_LEN];
	char *woble_setting_file_name;
	u8 *setting_file;
	u32 setting_file_len;
	u8 *wake_dev;   /* ADDR:NAP-UAP-LAP, VID/PID:Both Little endian */
	u32 wake_dev_len;
	struct fw_cfg_struct		woble_setting_apcf[WOBLE_SETTING_COUNT];
	struct fw_cfg_struct		woble_setting_apcf_fill_mac[WOBLE_SETTING_COUNT];
	struct fw_cfg_struct		woble_setting_apcf_fill_mac_location[WOBLE_SETTING_COUNT];

	struct fw_cfg_struct		woble_setting_radio_off;
	struct fw_cfg_struct		woble_setting_wakeup_type;
	/* complete event */
	struct fw_cfg_struct		woble_setting_radio_off_comp_event;

	struct bt_cfg_struct bt_cfg;
};

struct _PATCH_HEADER {
	u8 ucDateTime[16];
	u8 ucPlatform[4];
	u16 u2HwVer;
	u16 u2SwVer;
	u32 u4MagicNum;
};

struct _Global_Descr {
	u32 u4PatchVer;
	u32 u4SubSys;
	u32 u4FeatureOpt;
	u32 u4SectionNum;
};

struct _Section_Map {
	u32 u4SecType;
	u32 u4SecOffset;
	u32 u4SecSize;
	union {
		u32 u4SecSpec[SECTION_SPEC_NUM];
		struct {
			u32 u4DLAddr;
			u32 u4DLSize;
			u32 u4SecKeyIdx;
			u32 u4AlignLen;
			u32 reserved[9];
		}bin_info_spec;
	};
};

u8 LD_btmtk_usb_getWoBTW(void);
int LD_btmtk_usb_probe(mtkbt_dev_t *dev, int flag);
void LD_btmtk_usb_disconnect(mtkbt_dev_t *dev);
void LD_btmtk_usb_SetWoble(mtkbt_dev_t *dev);
int Ldbtusb_getBtWakeT(struct LD_btmtk_usb_data *data);


#define REV_MT76x2E3		0x0022

#define MT_REV_LT(_data, _chip, _rev) \
	is_##_chip(_data) && (((_data)->chip_id & 0x0000ffff) < (_rev))

#define MT_REV_GTE(_data, _chip, _rev) \
	is_##_chip(_data) && (((_data)->chip_id & 0x0000ffff) >= (_rev))

/*
 *  Load code method
 */
enum LOAD_CODE_METHOD {
	BIN_FILE_METHOD,
	HEADER_METHOD,
};
#endif
