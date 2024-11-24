/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#ifndef __BTMTK_DEFINE_H__
#define __BTMTK_DEFINE_H__


#include <linux/version.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/kallsyms.h>
#include <linux/device.h>
#include <asm/unaligned.h>

/* Define for proce node */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

/* Define for whole chip reset */
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/kthread.h>
#include <linux/freezer.h>

/** Driver version */
#define VERSION "7.0.2020121701"
#define SUBVER ":turnkey"


#define ENABLESTP FALSE
#define BTMTKUART_TX_STATE_ACTIVE	1
#define BTMTKUART_TX_STATE_WAKEUP	2
#define BTMTK_TX_WAIT_VND_EVT		3
#define BTMTKUART_REQUIRED_WAKEUP	4
#define BTMTKUART_REQUIRED_DOWNLOAD	5
#define BTMTK_TX_SKIP_VENDOR_EVT	6

#define BTMTKUART_RX_STATE_ACTIVE	1
#define BTMTKUART_RX_STATE_WAKEUP	2
#define BTMTKUART_RX_STATE_RESET	3

/**
 * Maximum rom patch file name length
 */
#define MAX_BIN_FILE_NAME_LEN 64

/**
 * Type definition
 */
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif

#ifndef UNUSED
	#define UNUSED(x) (void)(x)
#endif

#ifndef ALIGN_4
#define ALIGN_4(_value)             (((_value) + 3) & ~3u)
#endif /* ALIGN_4 */

#ifndef ALIGN_8
#define ALIGN_8(_value)             (((_value) + 7) & ~7u)
#endif /* ALIGN_4 */

/* This macro check the DW alignment of the input value.
 * _value - value of address need to check
 */
#ifndef IS_ALIGN_4
#define IS_ALIGN_4(_value)          (((_value) & 0x3) ? FALSE : TRUE)
#endif /* IS_ALIGN_4 */

#ifndef IS_NOT_ALIGN_4
#define IS_NOT_ALIGN_4(_value)      (((_value) & 0x3) ? TRUE : FALSE)
#endif /* IS_NOT_ALIGN_4 */

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))


/**
 * Log and level definition
 */
#define BTMTK_LOG_LVL_ERR	1
#define BTMTK_LOG_LVL_WARN	2
#define BTMTK_LOG_LVL_INFO	3
#define BTMTK_LOG_LVL_DBG	4
#define BTMTK_LOG_LVL_MAX	BTMTK_LOG_LVL_DBG
#define BTMTK_LOG_LVL_DEF	BTMTK_LOG_LVL_INFO	/* default setting */

#define HCI_SNOOP_ENTRY_NUM	30
#define HCI_SNOOP_BUF_SIZE	32
#define HCI_SNOOP_MAX_BUF_SIZE	66
#define WMT_OVER_HCI_HEADER_SIZE	3
#define READ_ISO_PACKET_CMD_SIZE	4

extern uint8_t btmtk_log_lvl;

#define BTMTK_ERR(fmt, ...)	 \
	do { if (btmtk_log_lvl >= BTMTK_LOG_LVL_ERR) pr_info("[btmtk_err] ***"fmt"***\n", ##__VA_ARGS__); } while (0)
#define BTMTK_WARN(fmt, ...)	\
	do { if (btmtk_log_lvl >= BTMTK_LOG_LVL_WARN) pr_info("[btmtk_warn] "fmt"\n", ##__VA_ARGS__); } while (0)
#define BTMTK_INFO(fmt, ...)	\
	do { if (btmtk_log_lvl >= BTMTK_LOG_LVL_INFO) pr_info("[btmtk_info] "fmt"\n", ##__VA_ARGS__); } while (0)
#define BTMTK_DBG(fmt, ...)	 \
	do { if (btmtk_log_lvl >= BTMTK_LOG_LVL_DBG) pr_info("[btmtk_dbg] "fmt"\n", ##__VA_ARGS__); } while (0)

#define BTMTK_INFO_RAW(p, l, fmt, ...)						\
	do {	\
		if (btmtk_log_lvl >= BTMTK_LOG_LVL_INFO) {	\
			int cnt_ = 0;	\
			int len_ = (l <= HCI_SNOOP_MAX_BUF_SIZE ? l : HCI_SNOOP_MAX_BUF_SIZE);	\
			uint8_t raw_buf[HCI_SNOOP_MAX_BUF_SIZE * 5 + 10];	\
			const unsigned char *ptr = p;	\
			for (cnt_ = 0; cnt_ < len_; ++cnt_) {	\
				if (snprintf(raw_buf+5*cnt_, 6, "0x%02X ", ptr[cnt_]) < 0) {	\
					pr_info("snprintf error\n");	\
					break;	\
				}	\
			}	\
			raw_buf[5*cnt_] = '\0';	\
			if (l <= HCI_SNOOP_MAX_BUF_SIZE) {	\
				pr_cont("[btmtk_info] "fmt"%s\n", ##__VA_ARGS__, raw_buf);	\
			} else {	\
				pr_cont("[btmtk_info] "fmt"%s (prtail)\n", ##__VA_ARGS__, raw_buf);	\
			}	\
		}	\
	} while (0)

#define BTMTK_DBG_RAW(p, l, fmt, ...)						\
	do {	\
		if (btmtk_log_lvl >= BTMTK_LOG_LVL_DBG) {	\
			int cnt_ = 0;	\
			int len_ = (l <= HCI_SNOOP_MAX_BUF_SIZE ? l : HCI_SNOOP_MAX_BUF_SIZE);	\
			uint8_t raw_buf[HCI_SNOOP_MAX_BUF_SIZE * 5 + 10];	\
			const unsigned char *ptr = p;	\
			for (cnt_ = 0; cnt_ < len_; ++cnt_) {	\
				if (snprintf(raw_buf+5*cnt_, 6, "0x%02X ", ptr[cnt_]) < 0) {	\
					pr_info("snprintf error\n");	\
					break;	\
				}	\
			}	\
			raw_buf[5*cnt_] = '\0';	\
			if (l <= HCI_SNOOP_MAX_BUF_SIZE) {	\
				pr_cont("[btmtk_debug] "fmt"%s\n", ##__VA_ARGS__, raw_buf);	\
			} else {	\
				pr_cont("[btmtk_debug] "fmt"%s (prtail)\n", ##__VA_ARGS__, raw_buf); \
			}	\
		}	\
	} while (0)

#define BTMTK_CIF_IS_NULL(bdev, cif_event) \
	(!bdev || !(&bdev->cif_state[cif_event]))

/**
 *
 * HCI packet type
 */
#define MTK_HCI_COMMAND_PKT		0x01
#define MTK_HCI_ACLDATA_PKT		0x02
#define MTK_HCI_SCODATA_PKT		0x03
#define MTK_HCI_EVENT_PKT		0x04
#define HCI_ISO_PKT			0x05
#define HCI_ISO_PKT_HEADER_SIZE	4
#define HCI_ISO_PKT_WITH_ACL_HEADER_SIZE	5

/**
 * ROM patch related
 */
#define PATCH_HCI_HEADER_SIZE	4
#define PATCH_WMT_HEADER_SIZE	5
/*
 * Enable STP
 * HCI+WMT+STP = 4 + 5 + 1(phase) +(4=STP_HEADER + 2=CRC)
#define PATCH_HEADER_SIZE	16
 */
/*#ifdef ENABLESTP
 * #define PATCH_HEADER_SIZE	(PATCH_HCI_HEADER_SIZE + PATCH_WMT_HEADER_SIZE + 1 + 6)
 * #define UPLOAD_PATCH_UNIT	916
 * #define PATCH_INFO_SIZE		30
 *#else
 */
#define PATCH_HEADER_SIZE	(PATCH_HCI_HEADER_SIZE + PATCH_WMT_HEADER_SIZE + 1)
/* TODO, If usb use 901 patch unit size, download patch will timeout
 * because the timeout has been set to 1s
 */
#define UPLOAD_PATCH_UNIT	2048
#define PATCH_INFO_SIZE		30
/*#endif*/
#define PATCH_PHASE1		1
#define PATCH_PHASE2		2
#define PATCH_PHASE3		3

/* It is for mt7961 download rom patch*/
#define FW_ROM_PATCH_HEADER_SIZE	32
#define FW_ROM_PATCH_GD_SIZE	64
#define FW_ROM_PATCH_SEC_MAP_SIZE	64
#define SEC_MAP_NEED_SEND_SIZE	52
#define PATCH_STATUS	7


#define IO_BUF_SIZE		(HCI_MAX_EVENT_SIZE > 256 ? HCI_MAX_EVENT_SIZE : 256)
#define EVENT_COMPARE_SIZE	64

#define SECTION_SPEC_NUM	13

#define BD_ADDRESS_SIZE 6
#define PHASE1_WMT_CMD_COUNT 255
#define VENDOR_CMD_COUNT 255


#define BT_CFG_NAME "bt.cfg"
#define BT_UNIFY_WOBLE "SUPPORT_UNIFY_WOBLE"
#define BT_UNIFY_WOBLE_TYPE "UNIFY_WOBLE_TYPE"
#define BT_WOBLE_BY_EINT "SUPPORT_WOBLE_BY_EINT"
#define BT_DONGLE_RESET_PIN "BT_DONGLE_RESET_GPIO_PIN"
#define BT_RESET_DONGLE "SUPPORT_DONGLE_RESET"
#define BT_FULL_FW_DUMP "SUPPORT_FULL_FW_DUMP"
#define BT_WOBLE_WAKELOCK "SUPPORT_WOBLE_WAKELOCK"
#define BT_WOBLE_FOR_BT_DISABLE "SUPPORT_WOBLE_FOR_BT_DISABLE"
#define BT_RESET_STACK_AFTER_WOBLE "RESET_STACK_AFTER_WOBLE"
#define BT_AUTO_PICUS "SUPPORT_AUTO_PICUS"
#define BT_AUTO_PICUS_FILTER "PICUS_FILTER_COMMAND"
#define BT_AUTO_PICUS_ENABLE "PICUS_ENABLE_COMMAND"
#define BT_PICUS_TO_HOST "SUPPORT_PICUS_TO_HOST"
#define BT_PHASE1_WMT_CMD "PHASE1_WMT_CMD"
#define BT_VENDOR_CMD "VENDOR_CMD"
#define BT_SINGLE_SKU "SUPPORT_BT_SINGLE_SKU"


#define PM_KEY_BTW (0x0015) /* Notify PM the unify woble type */

/**
 * Disable RESUME_RESUME
 */
#ifndef BT_DISABLE_RESET_RESUME
#define BT_DISABLE_RESET_RESUME 0
#endif

enum fw_cfg_index_len {
	FW_CFG_INX_LEN_NONE = 0,
	FW_CFG_INX_LEN_2 = 2,
	FW_CFG_INX_LEN_3 = 3,
};

struct fw_cfg_struct {
	char	*content;	/* APCF content or radio off content */
	u32	length;		/* APCF content or radio off content of length */
};

struct bt_cfg_struct {
	bool	support_unify_woble;	/* support unify woble or not */
	bool	support_woble_by_eint;		/* support woble by eint or not */
	bool	support_dongle_reset;		/* support chip reset or not */
	bool	support_full_fw_dump;		/* dump full fw coredump or not */
	bool	support_woble_wakelock;		/* support when woble error, do wakelock or not */
	bool	support_woble_for_bt_disable;		/* when bt disable, support enter susend or not */
	bool	reset_stack_after_woble;	/* support reset stack to re-connect IOT after resume */
	bool	support_auto_picus;			/* support enable PICUS automatically */
	struct fw_cfg_struct picus_filter;	/* support on PICUS filter command customization */
	struct fw_cfg_struct picus_enable;	/* support on PICUS enable command customization */
	bool	support_picus_to_host;			/* support picus log to host (boots/bluedroid) */
	int	dongle_reset_gpio_pin;		/* BT_DONGLE_RESET_GPIO_PIN number */
	unsigned int	unify_woble_type;	/* 0: legacy. 1: waveform. 2: IR */
	struct fw_cfg_struct phase1_wmt_cmd[PHASE1_WMT_CMD_COUNT];
	struct fw_cfg_struct vendor_cmd[VENDOR_CMD_COUNT];
	bool	support_bt_single_sku;
};

#define WIFI_DOWNLOAD	TRUE
#define BT_DOWNLOAD	FALSE

#define SWAP32(x) \
	((u32) (\
	(((u32) (x) & (u32) 0x000000ffUL) << 24) | \
	(((u32) (x) & (u32) 0x0000ff00UL) << 8) | \
	(((u32) (x) & (u32) 0x00ff0000UL) >> 8) | \
	(((u32) (x) & (u32) 0xff000000UL) >> 24)))

/* Endian byte swapping codes */
#ifdef __LITTLE_ENDIAN
#define cpu2le32(x) ((uint32_t)(x))
#define le2cpu32(x) ((uint32_t)(x))
#define cpu2be32(x) SWAP32((x))
#define be2cpu32(x) SWAP32((x))
#else
#define cpu2le32(x) SWAP32((x))
#define le2cpu32(x) SWAP32((x))
#define cpu2be32(x) ((uint32_t)(x))
#define be2cpu32(x) ((uint32_t)(x))
#endif

#define FW_VERSION	0x80021004
#define CHIP_ID	0x70010200
#define FLAVOR	0x70010020


#endif /* __BTMTK_DEFINE_H__ */
