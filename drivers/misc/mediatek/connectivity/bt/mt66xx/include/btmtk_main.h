/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __BTMTK_MAIN_H__
#define __BTMTK_MAIN_H__
#include "btmtk_define.h"

#define DEFAULT_COUNTRY_TABLE_NAME "btPowerTable.dat"

//static inline struct sk_buff *mtk_add_stp(struct btmtk_dev *bdev, struct sk_buff *skb);

#define hci_dev_test_and_clear_flag(hdev, nr)  test_and_clear_bit((nr), (hdev)->dev_flags)

/* h4_recv */
#define hci_skb_pkt_type(skb) bt_cb((skb))->pkt_type
#define hci_skb_expect(skb) bt_cb((skb))->expect
#define hci_skb_opcode(skb) bt_cb((skb))->hci.opcode

/* HCI bus types */
#define HCI_VIRTUAL	0
#define HCI_USB		1
#define HCI_PCCARD	2
#define HCI_UART	3
#define HCI_RS232	4
#define HCI_PCI		5
#define HCI_SDIO	6
#define HCI_SPI		7
#define HCI_I2C		8
#define HCI_SMD		9

#define HCI_TYPE_SIZE	1

/* this for 7663 need download patch staus
 * 0:
 * patch download is not complete/BT get patch semaphore fail (WiFi get semaphore success)
 * 1:
 * patch download is complete
 * 2:
 * patch download is not complete/BT get patch semaphore success
 */
#define MT766X_PATCH_IS_DOWNLOAD_BY_OTHER 0
#define MT766X_PATCH_READY 1
#define MT766X_PATCH_NEED_DOWNLOAD 2

/* this for 79XX need download patch staus
 * 0:
 * patch download is not complete, BT driver need to download patch
 * 1:
 * patch is downloading by Wifi,BT driver need to retry until status = PATCH_READY
 * 2:
 * patch download is complete, BT driver no need to download patch
 */
#define PATCH_ERR -1
#define PATCH_NEED_DOWNLOAD 0
#define PATCH_IS_DOWNLOAD_BY_OTHER 1
#define PATCH_READY 2

/* 0:
 * using legacy wmt cmd/evt to download fw patch, usb/sdio just support 0 now
 * 1:
 * using DMA to download fw patch
 */
#define PATCH_DOWNLOAD_USING_WMT 0
#define PATCH_DOWNLOAD_USING_DMA 1

#define PATCH_DOWNLOAD_PHASE1_2_DELAY_TIME 1
#define PATCH_DOWNLOAD_PHASE1_2_RETRY 5
#define PATCH_DOWNLOAD_PHASE3_DELAY_TIME 20
#define PATCH_DOWNLOAD_PHASE3_RETRY 20

/* * delay and retrey for main_send_cmd */
#define WMT_DELAY_TIMES 100
#define DELAY_TIMES 20
#define RETRY_TIMES 20

/* Expected minimum supported interface */
#define BT_MCU_MINIMUM_INTERFACE_NUM	4

/* Bus event */
#define HIF_EVENT_PROBE		0
#define HIF_EVENT_DISCONNECT	1
#define HIF_EVENT_SUSPEND	2
#define HIF_EVENT_RESUME	3
#define HIF_EVENT_STANDBY	4
#define HIF_EVENT_SUBSYS_RESET	5
#define HIF_EVENT_WHOLE_CHIP_RESET	6
#define HIF_EVENT_FW_DUMP	7


#define CHAR2HEX_SIZE	4

/**
 * For chip reset pin
 */
#define RESET_PIN_SET_LOW_TIME		100

/* stpbtfwlog setting */
#define FWLOG_QUEUE_COUNT			(400 * BT_MCU_MINIMUM_INTERFACE_NUM)
#define FWLOG_ASSERT_QUEUE_COUNT		45000
#define FWLOG_BLUETOOTH_KPI_QUEUE_COUNT		400
#define HCI_MAX_COMMAND_SIZE			255
#define HCI_MAX_COMMAND_BUF_SIZE		(HCI_MAX_COMMAND_SIZE * 3)
//#define HCI_MAX_ISO_SIZE	340

/* fwlog information define */
#define FWLOG_TYPE		0xF0
#define FWLOG_LEN_SIZE		2
#define FWLOG_TL_SIZE		(HCI_TYPE_SIZE + FWLOG_LEN_SIZE)
#define FWLOG_ATTR_TYPE_LEN	1
#define FWLOG_ATTR_LEN_LEN	1
#define FWLOG_ATTR_RX_LEN_LEN	2
#define FWLOG_ATTR_TL_SIZE	(FWLOG_ATTR_TYPE_LEN + FWLOG_ATTR_LEN_LEN)

#define FWLOG_HCI_IDX		0x00
#define FWLOG_DONGLE_IDX	0x01
#define FWLOG_TX		0x10
#define FWLOG_RX		0x11

/* total fwlog info len */
#define FWLOG_PRSV_LEN		32

#define COUNTRY_CODE_LEN	2


#define EDR_MIN		-32
#define EDR_MAX		20
#define EDR_MIN_LV9	13
#define BLE_MIN		-29
#define BLE_MAX		20
#define EDR_MIN_R1	-64
#define EDR_MAX_R1	40
#define EDR_MIN_LV9_R1	26
#define BLE_MIN_R1	-58
#define BLE_MAX_R1	40
#define EDR_MIN_R2	-128
#define EDR_MAX_R2	80
#define EDR_MIN_LV9_R2	52
#define BLE_MIN_R2	-116
#define BLE_MAX_R2	80

#define ERR_PWR		-9999

#define WAIT_POWERKEY_TIMEOUT 5000

#define SEPARATOR_LEN 2
#define STP_CRC_LEN 2
#define TEMP_LEN 260
#define SEARCH_LEN 32
#define TEXT_LEN 128

/* CMD&Event sent by driver */
#define READ_EFUSE_CMD_LEN 18
#define READ_EFUSE_EVT_HDR_LEN 9
#define READ_EFUSE_CMD_BLOCK_OFFSET 10

#define CHECK_LD_PATCH_CMD_LEN 9
#define CHECK_LD_PATCH_EVT_HDR_LEN 7
#define CHECK_LD_PATCH_EVT_RESULT_OFFSET 6	/* need confirm later */

#define HWERR_EVT_LEN 4

#define LD_PATCH_EVT_LEN 8

#define HCI_RESET_CMD_LEN 4
#define HCI_RESET_EVT_LEN 7

#define WMT_RESET_CMD_LEN 9
#define WMT_RESET_EVT_LEN 8

#define WMT_POWER_ON_CMD_LEN 10
#define WMT_POWER_ON_EVT_HDR_LEN 7
#define WMT_POWER_ON_EVT_RESULT_OFFSET 7

#define WMT_POWER_OFF_CMD_LEN 10
#define WMT_POWER_OFF_EVT_HDR_LEN 7
#define WMT_POWER_OFF_EVT_RESULT_OFFSET 7

#define PICUS_ENABLE_CMD_LEN 8
#define PICUS_ENABLE_EVT_HDR_LEN 9

#define PICUS_DISABLE_CMD_LEN 8
#define PICUS_DISABLE_EVT_HDR_LEN 9

#define RES_APCF_CMD_LEN 9
#define RES_APCF_EVT_LEN 5

#define READ_ADDRESS_CMD_LEN 4
#define READ_ADDRESS_EVT_HDR_LEN 7

#define WOBLE_ENABLE_DEFAULT_CMD_LEN 40
#define WOBLE_ENABLE_DEFAULT_EVT_LEN 5

#define WOBLE_DISABLE_DEFAULT_CMD_LEN 9
#define WOBLE_DISABLE_DEFAULT_EVT_LEN 5

#define RADIO_OFF_CMD_LEN 9
#define RADIO_OFF_EVT_LEN 5

#define RADIO_ON_CMD_LEN 9
#define RADIO_ON_EVT_LEN 5

#define APCF_FILTER_CMD_LEN 14
#define APCF_FILTER_EVT_HDR_LEN 8

#define APCF_CMD_LEN 43
#define APCF_EVT_HDR_LEN 7

#define APCF_DELETE_CMD_LEN 7
#define APCF_DELETE_EVT_HDR_LEN 8

#define APCF_RESUME_EVT_HDR_LEN 7

#define CHECK_WOBX_DEBUG_CMD_LEN 8
#define CHECK_WOBX_DEBUG_EVT_HDR_LEN 2

#define SET_STP_CMD_LEN 13
#define SET_STP_EVT_LEN 9

#define SET_STP1_CMD_LEN 16
#define SET_STP1_EVT_LEN 19

#define SET_SLEEP_CMD_LEN 11
#define SET_SLEEP_EVT_LEN 7

#define EVT_HDR_LEN 2

#define ASSERT_CMD_LEN 9

#define TXPOWER_CMD_LEN 16
#define TXPOWER_EVT_LEN 7

#define FW_COREDUMP_CMD_LEN 4
#define HCI_RESET_CMD_LEN 4
#define READ_ISO_PACKET_SIZE_CMD_HDR_LEN 4

#ifndef LD_PATCH_TIME
#define LD_PATCH_TIME 0
#endif

enum {
	RES_1 = 0,
	RES_DOT_5,
	RES_DOT_25
};

enum {
	CHECK_SINGLE_SKU_PWR_MODE	= 0,
	CHECK_SINGLE_SKU_EDR_MAX,
	CHECK_SINGLE_SKU_BLE,
	CHECK_SINGLE_SKU_BLE_2M,
	CHECK_SINGLE_SKU_BLE_LR_S2,
	CHECK_SINGLE_SKU_BLE_LR_S8,
	CHECK_SINGLE_SKU_ALL
};

enum {
	DISABLE_LV9 = 0,
	ENABLE_LV9
};

enum {
	DIFF_MODE_3DB = 0,
	DIFF_MODE_0DB
};

struct btmtk_cif_state {
	unsigned char ops_enter;
	unsigned char ops_end;
	unsigned char ops_error;
};

enum TX_TYPE {
	BTMTK_TX_CMD_FROM_DRV = 0,	/* send hci cmd and wmt cmd by driver */
	BTMTK_TX_ACL_FROM_DRV,	/* send acl pkt with load rompatch by driver */
	BTMTK_TX_PKT_FROM_HOST,	/* send pkt from host, include acl and hci */
};

enum bt_state {
	FUNC_OFF = 0,
	TURNING_ON = 1,
	PRE_ON_AFTER_CAL = 2,
	FUNC_ON = 3,
	RESET_START = 4,
	RESET_END = 5
};

struct bt_power_setting {
	int8_t EDR_Max;
	int8_t LV9;
	int8_t DM;
	int8_t IR;
	int8_t BLE_1M;
	int8_t BLE_2M;
	int8_t BLE_LR_S2;
	int8_t BLE_LR_S8;
	char country_code[COUNTRY_CODE_LEN + 1];
};

enum {
	BTMTK_DONGLE_STATE_UNKNOWN,
	BTMTK_DONGLE_STATE_POWER_ON,
	BTMTK_DONGLE_STATE_POWER_OFF,
	BTMTK_DONGLE_STATE_ERROR,
};

enum {
	HW_ERR_NONE = 0x00,
	HW_ERR_CODE_CHIP_RESET = 0xF0,
	HW_ERR_CODE_USB_DISC = 0xF1,
	HW_ERR_CODE_CORE_DUMP = 0xF2,
	HW_ERR_CODE_POWER_ON = 0xF3,
	HW_ERR_CODE_POWER_OFF = 0xF4,
	HW_ERR_CODE_SET_SLEEP_CMD = 0xF5,
	HW_ERR_CODE_RESET_STACK_AFTER_WOBLE = 0xF6,
};

/* Please keep sync with btmtk_set_state function */
enum {
	/* BTMTK_STATE_UNKNOWN = 0, */
	BTMTK_STATE_INIT = 1,
	BTMTK_STATE_DISCONNECT,
	BTMTK_STATE_PROBE,
	BTMTK_STATE_WORKING,
	BTMTK_STATE_SUSPEND,
	BTMTK_STATE_RESUME,
	BTMTK_STATE_FW_DUMP,
	BTMTK_STATE_STANDBY,
	BTMTK_STATE_SUBSYS_RESET,

	BTMTK_STATE_MSG_NUM
};

/* Please keep sync with btmtk_fops_set_state function */
enum {
	/* BTMTK_FOPS_STATE_UNKNOWN = 0, */
	BTMTK_FOPS_STATE_INIT = 1,
	BTMTK_FOPS_STATE_OPENING,	/* during opening */
	BTMTK_FOPS_STATE_OPENED,	/* open in fops_open */
	BTMTK_FOPS_STATE_CLOSING,	/* during closing */
	BTMTK_FOPS_STATE_CLOSED,	/* closed */

	BTMTK_FOPS_STATE_MSG_NUM
};

enum {
	BTMTK_EVENT_COMPARE_STATE_UNKNOWN,
	BTMTK_EVENT_COMPARE_STATE_NOTHING_NEED_COMPARE,
	BTMTK_EVENT_COMPARE_STATE_NEED_COMPARE,
	BTMTK_EVENT_COMPARE_STATE_COMPARE_SUCCESS,
};

struct h4_recv_pkt {
	u8  type;	/* Packet type */
	u8  hlen;	/* Header length */
	u8  loff;	/* Data length offset in header */
	u8  lsize;	/* Data length field size */
	u16 maxlen;	/* Max overall packet length */
	int (*recv)(struct hci_dev *hdev, struct sk_buff *skb);
};

#pragma pack(1)
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
			u32 u4SecType;
			u32 u4DLModeCrcType;
			u32 u4Crc;
			u32 reserved[6];
		} bin_info_spec;
	};
};
#pragma pack()

#define H4_RECV_ACL \
	.type = HCI_ACLDATA_PKT, \
	.hlen = HCI_ACL_HDR_SIZE, \
	.loff = 2, \
	.lsize = 2, \
	.maxlen = HCI_MAX_FRAME_SIZE \

#define H4_RECV_SCO \
	.type = HCI_SCODATA_PKT, \
	.hlen = HCI_SCO_HDR_SIZE, \
	.loff = 2, \
	.lsize = 1, \
	.maxlen = HCI_MAX_SCO_SIZE

#define H4_RECV_EVENT \
	.type = HCI_EVENT_PKT, \
	.hlen = HCI_EVENT_HDR_SIZE, \
	.loff = 1, \
	.lsize = 1, \
	.maxlen = HCI_MAX_EVENT_SIZE

/* yumin todo */
// TODO: replace by kernel constant if kernel support new spec
#define HCI_ISODATA_PKT		0x05
#define HCI_ISO_HDR_SIZE	4
#define H4_RECV_ISO \
	.type = HCI_ISODATA_PKT, \
	.hlen = HCI_ISO_HDR_SIZE, \
	.loff = 2, \
	.lsize = 2, \
	.maxlen = HCI_MAX_FRAME_SIZE

struct btmtk_dev {
	struct hci_dev	*hdev;
	unsigned long	hdev_flags;
	unsigned long	flags;
	void *intf_dev;
	void *cif_dev;

	struct work_struct	work;
	struct work_struct	waker;
	struct work_struct	reset_waker;

	int	recv_evt_len;
	int	tx_in_flight;
	spinlock_t	txlock;
	spinlock_t	rxlock;

	struct sk_buff	*evt_skb;
	struct sk_buff	*sco_skb;

	/* For ble iso packet size */
	int iso_threshold;

	unsigned int	sco_num;
	int	isoc_altsetting;

	int	suspend_count;

	/* For tx queue */
	unsigned long	tx_state;

	/* For rx queue */
	struct workqueue_struct	*workqueue;
	struct sk_buff_head	rx_q;
	struct work_struct	rx_work;
	struct sk_buff		*rx_skb;

	wait_queue_head_t	p_wait_event_q;

	unsigned int	subsys_reset;
	unsigned int	chip_reset;
	unsigned char	*rom_patch_bin_file_name;
	unsigned int	chip_id;
	unsigned int	flavor;
	unsigned int	fw_version;
	unsigned char	dongle_index;
	unsigned char	power_state;
	unsigned char	fops_state;
	unsigned char	interface_state;
	struct btmtk_cif_state *cif_state;

	/* io buffer for usb control transfer */
	unsigned char	*io_buf;

	unsigned char	*setting_file;
	unsigned char	bdaddr[BD_ADDRESS_SIZE];

	unsigned char		*bt_cfg_file_name;
	struct bt_cfg_struct	bt_cfg;

	/* single sku */
	unsigned char		*country_file_name;
	u8 opcode_usr[2];
};

typedef int (*cif_bt_init_ptr)(void);
typedef void (*cif_bt_exit_ptr)(void);
typedef int (*cif_open_ptr)(struct hci_dev *hdev);
typedef int (*cif_close_ptr)(struct hci_dev *hdev);
typedef int (*cif_reg_read_ptr)(struct btmtk_dev *bdev, u32 reg, u32 *val);
typedef int (*cif_reg_write_ptr)(struct btmtk_dev *bdev, u32 reg, u32 val);
typedef int (*cif_send_cmd_ptr)(struct btmtk_dev *bdev, struct sk_buff *skb,
		int delay, int retry, int pkt_type);
typedef int (*cif_send_and_recv_ptr)(struct btmtk_dev *bdev,
		struct sk_buff *skb,
		const uint8_t *event, const int event_len,
		int delay, int retry, int pkt_type);
typedef int (*cif_event_filter_ptr)(struct btmtk_dev *bdev, struct sk_buff *skb);
typedef int (*cif_subsys_reset_ptr)(struct btmtk_dev *bdev);
typedef int (*cif_whole_reset_ptr)(struct btmtk_dev *bdev);
typedef void (*cif_chip_reset_notify_ptr)(struct btmtk_dev *bdev);
typedef void (*cif_mutex_lock_ptr)(struct btmtk_dev *bdev);
typedef void (*cif_mutex_unlock_ptr)(struct btmtk_dev *bdev);
typedef int (*cif_flush_ptr)(struct btmtk_dev *bdev);
typedef void (*cif_log_init_ptr)(void);
typedef void (*cif_log_register_cb_ptr)(void (*func)(void));
typedef ssize_t (*cif_log_read_to_user_ptr)(char __user *buf, size_t count);
typedef unsigned int (*cif_log_get_buf_size_ptr)(void);
typedef void (*cif_log_deinit_ptr)(void);
typedef void (*cif_log_hold_sem_ptr)(void);
typedef void (*cif_log_release_sem_ptr)(void);
typedef void (*cif_open_done_ptr)(struct btmtk_dev *bdev);
typedef int (*cif_dl_dma_ptr)(struct btmtk_dev *bdev, u8 *image,
		u8 *fwbuf, int section_dl_size, int section_offset);

struct hif_hook_ptr {
	cif_bt_init_ptr			init;
	cif_bt_exit_ptr			exit;
	cif_open_ptr			open;
	cif_close_ptr			close;
	cif_reg_read_ptr		reg_read;
	cif_reg_write_ptr		reg_write;
	cif_send_cmd_ptr		send_cmd;
	cif_send_and_recv_ptr		send_and_recv;
	cif_event_filter_ptr		event_filter;
	cif_subsys_reset_ptr		subsys_reset;
	cif_whole_reset_ptr		whole_reset;
	cif_chip_reset_notify_ptr	chip_reset_notify;
	cif_mutex_lock_ptr		cif_mutex_lock;
	cif_mutex_unlock_ptr		cif_mutex_unlock;
	cif_flush_ptr				flush;
	cif_log_init_ptr			log_init;
	cif_log_register_cb_ptr		log_register_cb;
	cif_log_read_to_user_ptr	log_read_to_user;
	cif_log_get_buf_size_ptr	log_get_buf_size;
	cif_log_deinit_ptr			log_deinit;
	cif_log_hold_sem_ptr		log_hold_sem;
	cif_log_release_sem_ptr		log_release_sem;
	cif_open_done_ptr		open_done;
	cif_dl_dma_ptr			dl_dma;
};

struct btmtk_main_info {
	int whole_reset_flag;
	u8 reset_stack_flag;
	struct wakeup_source *fwdump_ws;
	struct wakeup_source *woble_ws;
	struct wakeup_source *eint_ws;
	struct hif_hook_ptr hif_hook;
	struct bt_power_setting PWS;
	/* save Hci Snoop for debug*/
	u8 hci_cmd_buf[HCI_SNOOP_ENTRY_NUM][HCI_SNOOP_MAX_BUF_SIZE];
	u8 hci_cmd_len[HCI_SNOOP_ENTRY_NUM];
	u16 hci_cmd_actual_len[HCI_SNOOP_ENTRY_NUM];
	unsigned int hci_cmd_timestamp[HCI_SNOOP_ENTRY_NUM];
	u8 hci_cmd_index;

	u8 hci_event_buf[HCI_SNOOP_ENTRY_NUM][HCI_SNOOP_MAX_BUF_SIZE];
	u8 hci_event_len[HCI_SNOOP_ENTRY_NUM];
	u16 hci_event_actual_len[HCI_SNOOP_ENTRY_NUM];
	unsigned int hci_event_timestamp[HCI_SNOOP_ENTRY_NUM];
	u8 hci_event_index;

	u8 hci_adv_event_buf[HCI_SNOOP_ENTRY_NUM][HCI_SNOOP_MAX_BUF_SIZE];
	u8 hci_adv_event_len[HCI_SNOOP_ENTRY_NUM];
	u16 hci_adv_event_actual_len[HCI_SNOOP_ENTRY_NUM];
	unsigned int hci_adv_event_timestamp[HCI_SNOOP_ENTRY_NUM];
	u8 hci_adv_event_index;


	u8 hci_acl_buf[HCI_SNOOP_ENTRY_NUM][HCI_SNOOP_MAX_BUF_SIZE];
	u8 hci_acl_len[HCI_SNOOP_ENTRY_NUM];
	u16 hci_acl_actual_len[HCI_SNOOP_ENTRY_NUM];
	unsigned int hci_acl_timestamp[HCI_SNOOP_ENTRY_NUM];
	u8 hci_acl_index;

	u8 wmt_over_hci_header[WMT_OVER_HCI_HEADER_SIZE];
	u8 read_iso_packet_size_cmd[READ_ISO_PACKET_CMD_SIZE];
};

static inline int is_mt7922(u32 chip_id)
{
	chip_id &= 0xFFFF;
	if (chip_id == 0x7922)
		return 1;
	return 0;
}

static inline int is_mt7961(u32 chip_id)
{
	chip_id &= 0xFFFF;
	if (chip_id == 0x7961)
		return 1;
	return 0;
}

static inline int is_mt7663(u32 chip_id)
{
	chip_id &= 0xFFFF;
	if (chip_id == 0x7663)
		return 1;
	return 0;
}

static inline int is_mt66xx(u32 chip_id)
{
	chip_id &= 0xFFFF;
	if (chip_id == 0x6631 || chip_id == 0x6635)
		return 1;
	return 0;
}

unsigned char btmtk_get_chip_state(struct btmtk_dev *bdev);
void btmtk_set_chip_state(struct btmtk_dev *bdev, unsigned char new_state);
int btmtk_allocate_hci_device(struct btmtk_dev *bdev, int hci_bus_type);
void btmtk_free_hci_device(struct btmtk_dev *bdev, int hci_bus_type);
int btmtk_register_hci_device(struct btmtk_dev *bdev);
int btmtk_deregister_hci_device(struct btmtk_dev *bdev);
int btmtk_recv(struct hci_dev *hdev, const u8 *data, size_t count);
int btmtk_recv_event(struct hci_dev *hdev, struct sk_buff *skb);
int btmtk_recv_acl(struct hci_dev *hdev, struct sk_buff *skb);
int btmtk_recv_iso(struct hci_dev *hdev, struct sk_buff *skb);
int btmtk_send_init_cmds(struct btmtk_dev *hdev);
int btmtk_send_deinit_cmds(struct btmtk_dev *hdev);
int btmtk_send_wmt_reset(struct btmtk_dev *hdev);
int btmtk_load_rom_patch_766x(struct btmtk_dev *hdev);
int btmtk_load_rom_patch(struct btmtk_dev *bdev);
struct btmtk_dev *btmtk_get_dev(void);
void btmtk_reset_waker(struct work_struct *work);
int btmtk_cap_init(struct btmtk_dev *bdev);
struct btmtk_main_info *btmtk_get_main_info(void);
int btmtk_get_interface_num(void);
int btmtk_reset_power_on(struct btmtk_dev *bdev);

void btmtk_send_hw_err_to_host(struct btmtk_dev *bdev);
void btmtk_free_setting_file(struct btmtk_dev *bdev);

unsigned char btmtk_fops_get_state(struct btmtk_dev *bdev);
void btmtk_fops_set_state(struct btmtk_dev *bdev, unsigned char new_state);

void btmtk_hci_snoop_save_cmd(u32 len, u8 *buf);
void btmtk_hci_snoop_save_event(u32 len, u8 *buf);
void btmtk_hci_snoop_save_adv_event(u32 len, u8 *buf);
void btmtk_hci_snoop_save_acl(u32 len, u8 *buf);
void btmtk_hci_snoop_print(u32 len, const u8 *buf);
void btmtk_hci_snoop_print_to_log(void);
unsigned long btmtk_kallsyms_lookup_name(const char *name);
void btmtk_do_gettimeofday(struct timespec64 *tv);
void btmtk_reg_hif_hook(struct hif_hook_ptr *hook);
int btmtk_main_cif_initialize(struct btmtk_dev *bdev, int hci_bus);
void btmtk_main_cif_uninitialize(struct btmtk_dev *bdev, int hci_bus);
int btmtk_main_cif_disconnect_notify(struct btmtk_dev *bdev, int hci_bus);
int btmtk_load_code_from_bin(u8 **image, char *bin_name,
					 struct device *dev, u32 *code_len, u8 retry);
int btmtk_main_send_cmd(struct btmtk_dev *bdev, const uint8_t *cmd,
		const int cmd_len, const uint8_t *event, const int event_len, int delay,
		int retry, int pkt_type);
int btmtk_load_code_from_setting_files(char *setting_file_name,
			struct device *dev, u32 *code_len, struct btmtk_dev *bdev);
int btmtk_load_fw_cfg_setting(char *block_name, struct fw_cfg_struct *save_content,
		int counter, u8 *searchcontent, enum fw_cfg_index_len index_length);
int btmtk_send_assert_cmd(struct btmtk_dev *bdev);
void btmtk_free_fw_cfg_struct(struct fw_cfg_struct *fw_cfg, int count);
struct btmtk_dev **btmtk_get_pp_bdev(void);


int32_t btmtk_set_sleep(struct hci_dev *hdev, u_int8_t need_wait);
int32_t bgfsys_bt_patch_dl(void);
int btmtk_efuse_read(struct btmtk_dev *bdev, u16 addr, u8 *value);
void btmtk_set_country_code_from_wifi(char *code);
#endif /* __BTMTK_MAIN_H__ */
