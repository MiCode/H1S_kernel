/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _BTMTK_BTIF_H_
#define _BTMTK_BTIF_H_

#include <net/bluetooth/bluetooth.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/workqueue.h>
#include <linux/pm_qos.h>

#include "conninfra.h"
#include "conn_power_throttling.h"
#include "btmtk_define.h"
#include "btmtk_main.h"

#undef __init
#undef __exit
#define __init
#define __exit

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/
/* Preamble length for HCI Command:
**      2-bytes for opcode and 1 byte for length
*/
#define HCI_CMD_PREAMBLE_SIZE   3

/* Preamble length for ACL Data:
**      2-byte for Handle and 2 byte for length
*/
#define HCI_ACL_PREAMBLE_SIZE   4

/* Preamble length for SCO Data:
**      2-byte for Handle and 1 byte for length
*/
#define HCI_SCO_PREAMBLE_SIZE   3

/* Preamble length for HCI Event:
**      1-byte for opcode and 1 byte for length
*/
#define HCI_EVT_PREAMBLE_SIZE   2

#define HCI_CMD_HDR_LEN			(HCI_CMD_PREAMBLE_SIZE + 1)
#define HCI_EVT_HDR_LEN			(HCI_EVT_PREAMBLE_SIZE)

/* WMT Event */
#define WMT_EVT_OFFSET			(HCI_EVT_HDR_LEN)
#define WMT_EVT_HDR_LEN			(4)

#define WAKE_LOCK_NAME_SIZE		(16)


#define MT_BGF2AP_BTIF_WAKEUP_IRQ_ID 	312 /* temp */
#define MT_BGF2AP_SW_IRQ_ID          	271 /* temp */

#define IRQ_NAME_SIZE			(20)
#define MAX_STATE_MONITORS		(2)
#define RING_BUFFER_SIZE		(16384)

#define MAX_DUMP_DATA_SIZE		(20)
#define MAX_DUMP_QUEUE_SIZE		(100)

/* BT HCI Cmd/Evt */
#define HCI_EVT_COMPLETE_EVT		0x0E
#define HCI_EVT_STATUS_EVT			0x0F
#define HCI_EVT_CC_STATUS_SUCCESS	0x00
#define HCI_CMD_DY_ADJ_PWR_QUERY 	0x01
#define HCI_CMD_DY_ADJ_PWR_SET		0x02

/* sleep parameter */
#define USLEEP_1MS_L			(1000)
#define USLEEP_1MS_H			(1100)
#define USLEEP_5MS_L			(5000)
#define USLEEP_5MS_H			(5200)

/* function return value*/
#define RET_PWRON_WHOLE_CHIP_RESET	(99)
#define RET_SWIRQ_ST_FAIL			(-98)

#undef SUPPORT_BT_THREAD
#define SUPPORT_BT_THREAD	(1)
#define SUPPORT_COREDUMP	(1)

#define DRIVER_CMD_CHECK	(1)

typedef void (*BT_RX_EVENT_CB) (void);
typedef int (*BT_RX_EVT_HANDLER_CB) (uint8_t *buf, int len);

enum wmt_evt_result {
	WMT_EVT_SUCCESS,
	WMT_EVT_FAIL,
	WMT_EVT_INVALID,
	WMT_EVT_SKIP // normal packet, not for wmt cmd
};

typedef enum wmt_pkt_dir {
	WMT_PKT_DIR_HOST_TO_CHIP = 1, /* command */
	WMT_PKT_DIR_CHIP_TO_HOST = 2  /* event */
} WMT_PKT_DIR_T;

typedef enum wmt_opcode {
	WMT_OPCODE_FUNC_CTRL = 0x06,
	WMT_OPCODE_RF_CAL = 0x14,
	WMT_OPCODE_0XF0 = 0xF0,
	WMT_OPCODE_ANT_EFEM = 0x55,
	WMT_OPCODE_MAX
} WMT_OPCODE_T;

enum wmt_blank_state {
	// wmt parameter: 0(screen off) / 1(screen on)
	WMT_PARA_SCREEN_OFF = 0,
	WMT_PARA_SCREEN_ON = 1
};

enum bt_irq_type {
	BGF2AP_BTIF_WAKEUP_IRQ,
	BGF2AP_SW_IRQ,
	BT_CONN2AP_SW_IRQ,
	BGF2AP_IRQ_MAX
};

struct bt_irq_ctrl {
	uint32_t irq_num;
	uint8_t name[IRQ_NAME_SIZE];
	u_int8_t active;
	spinlock_t lock;
	unsigned long flags;
};

struct btif_deepidle_ctrl {
	struct semaphore sem;
	struct workqueue_struct *task;
	struct delayed_work work;
	u_int8_t is_dpidle;
};

enum bt_reset_level {
	RESET_LEVEL_NONE,
	RESET_LEVEL_0_5,	/* subsys reset */
	RESET_LEVEL_0,		/* whole chip reset */
	RESET_LEVEL_MAX
};


struct bt_wake_lock {
	struct wakeup_source *ws;
	uint8_t name[WAKE_LOCK_NAME_SIZE];
#if (PM_QOS_CONTROL == 1)
	struct pm_qos_request qos_req;
#endif
};

enum bt_psm_state {
	PSM_ST_SLEEP,
	PSM_ST_NORMAL_TR,
	PSM_ST_MAX
};

struct bt_psm_ctrl {
	enum bt_psm_state state;
	u_int8_t sleep_flag;  /* sleep notify */
	u_int8_t wakeup_flag; /* wakeup notify */
	struct completion comp; /* request completion */
	u_int8_t result; /* request result */
	struct bt_wake_lock wake_lock;
	u_int8_t force_on;
};

enum bt_direction_type {
	NONE,
	TX,
	RX
};

struct bt_dump_packet {
	enum bt_direction_type 	direction_type;
	u_int16_t 				data_length;
	u_int8_t 				data[MAX_DUMP_DATA_SIZE];
	struct timespec64 		kerneltime;
	struct timespec64 		time;
};

struct bt_dump_queue {
	struct bt_dump_packet	queue[MAX_DUMP_QUEUE_SIZE];
	int32_t 				index;
	bool 					full;
	spinlock_t				lock;
};


#define BT_BTIF_DUMP_OWN_CR		0x01
#define BT_BTIF_DUMP_REG		0x02
#define BT_BTIF_DUMP_LOG		0x04
#define BT_BTIF_DUMP_DMA		0x08
#define BT_BTIF_DUMP_ALL		0x0F

struct sched_param {
	int sched_priority;
};

struct bt_dbg_st {
	bool rt_thd_enable;
	uint8_t rx_buf_ctrl;
};

typedef void (*BT_STATE_CHANGE_CB) (uint8_t state);

struct wmt_pkt_param {
	/* Extensible union */
	union {
		struct {
			uint8_t subsys; /* subsys type:
					 00 - BT
					 01 - FM (obsolete)
					 02 - GPS (obsolete)
					 03 - WIFI (obsolete) */
			uint8_t on; /* 00 - off, 01 - on */
		} func_ctrl_cmd;

		struct {
			uint8_t status;
		} func_ctrl_evt;

		struct {
			uint8_t subop; /* sub operation:
					01 - calibration start (obsolete)
					02 - restore indication (obsolete)
					03 - backup */
		} rf_cal_cmd;

		struct {
			uint8_t status;
			uint8_t subop;
			uint8_t start_addr[4]; /* start address of cal data, little endian */
			uint8_t data_len[2];   /* cal data length, little endian */
			uint8_t ready_addr[4]; /* ready bit address, little endian */
		} rf_cal_evt;

		uint8_t evt_buf[16];
	} u;
};

struct bt_rf_cal_data_backup {
	uint32_t start_addr; /* calibration data start address on sysram */
	uint32_t ready_addr; /* ready bit address on sysram to mark calibration data is ready */
	uint8_t *p_cache_buf; /* buffer to cache the calibration data */
	uint16_t cache_len; /* cached data length */
};

struct bt_ring_buffer_mgmt {
	uint8_t buf[RING_BUFFER_SIZE];
	uint32_t write_idx;
	uint32_t read_idx;
	spinlock_t lock;
};

/*
 * WMT Packet Format
 *
 * NOTICE:
 *   It is recommanded to define each field in bytes, otherwise we should consider the
 *   memory alignment by compiler and use __attribute__((__packed__)) if needed.
 */
struct wmt_pkt_hdr {
	uint8_t dir;
	uint8_t opcode;
	uint8_t param_len[2]; /* 2 bytes length, little endian */
};


struct wmt_pkt {
	struct wmt_pkt_hdr hdr;
	struct wmt_pkt_param params;
};

struct bt_internal_cmd {
	uint8_t waiting_event;
	uint16_t pending_cmd_opcode;
	uint8_t wmt_opcode;
	struct completion comp; /* command completion */
	u_int8_t result; /* command result */
	struct wmt_pkt_param wmt_event_params;
};

#if (DRIVER_CMD_CHECK == 1)
struct bt_cmd_node {
	uint16_t opcode;
	struct bt_cmd_node *next;
};

struct bt_cmd_queue {
	uint8_t size;
	struct bt_cmd_node *head, *tail;
	spinlock_t lock;
};
#endif

struct bt_DyPwr_st {
	int8_t dy_max_dbm;
	int8_t dy_min_dbm;
	int8_t lp_bdy_dbm;
	int8_t fw_sel_dbm;
	BT_RX_EVT_HANDLER_CB cb;
	/* Power Throttling Feature */
	enum conn_pwr_low_battery_level lp_cur_lv;
};

struct internal_trx_st {
	spinlock_t lock;
	unsigned long flag;
	BT_RX_EVT_HANDLER_CB cb;
	uint16_t opcode;
	bool send_to_stack;
	uint16_t buf_len;
	uint8_t buf[256];
	struct completion comp;
};

struct btmtk_btif_dev {
	/* BT state machine */
	u_int8_t			bt_state;

	/* Power state */
	struct bt_psm_ctrl	psm;

	/* Reset relative */
	struct completion 	rst_comp;
	enum bt_reset_level	rst_level;
	u_int8_t		rst_count;
	u_int8_t		rst_flag;
	wait_queue_head_t	rst_onoff_waitq;

	u_int8_t		do_recal;
	u_int8_t		bt_precal_state;
	struct bt_rf_cal_data_backup cal_data;

#ifdef SUPPORT_BT_THREAD
	/* thread */
	struct task_struct	*tx_thread;
	wait_queue_head_t	tx_waitq;

	/* Tx quque */
	struct sk_buff_head	tx_queue;
#endif

#if (USE_DEVICE_NODE == 1)
	struct bt_ring_buffer_mgmt rx_buffer;
#endif
	u_int8_t		rx_ind; /* RX indication from Firmware */
	u_int8_t		bgf2ap_ind; /* FW log / reset indication */
	u_int8_t		bt_conn2ap_ind; /* BGF bus hang indication */

	/* context for current pending command */
	struct bt_internal_cmd	internal_cmd;
	/* For internal command (generated in driver) handling */
	u_int8_t		event_intercept;

#if (DRIVER_CMD_CHECK == 1)
	/* command response queue */
	struct bt_cmd_queue 	cmd_queue;
	uint8_t			cmd_timeout_count;
	bool			cmd_timeout_check;
#endif
	/* driver dump queue*/
	struct bt_dump_queue	dump_queue;
	/* cif info */
	struct platform_device	*pdev;

	/* coredump handle */
	void 			*coredump_handle;

	/* state change callback */
	BT_STATE_CHANGE_CB	state_change_cb[MAX_STATE_MONITORS];

	/* call back to notify upper layer RX data is available */
	BT_RX_EVENT_CB		rx_event_cb;

	/* initernal trx function: send particular command,
	get command complete event and run callback function */
	struct internal_trx_st internal_trx;

	/* DynamicAdjustTxPower function*/
	struct bt_DyPwr_st dy_pwr;

	/* sempaphore to control close */
	struct semaphore halt_sem;
	struct semaphore internal_cmd_sem;
	struct semaphore cmd_tout_sem;

	/* blank status */
	int32_t		blank_state;

	/* btif deep idle ctrl */
	struct btif_deepidle_ctrl btif_dpidle_ctrl;

};

#define BTMTK_GET_DEV(bdev) (&bdev->pdev->dev)

/* IRQ APIs */
int bt_request_irq(enum bt_irq_type irq_type);
void bt_enable_irq(enum bt_irq_type irq_type);
void bt_disable_irq(enum bt_irq_type irq_type);
void bt_free_irq(enum bt_irq_type irq_type);
void bt_trigger_reset(void);
int bt_chip_reset_flow(enum bt_reset_level rst_level,
			     enum consys_drv_type drv,
			     char *reason);
void bt_bgf2ap_irq_handler(void);
void bt_conn2ap_irq_handler(void);

/* external functions */
int BT_init(void);
void BT_exit(void);
int32_t btmtk_wcn_btif_open(void);
int32_t btmtk_wcn_btif_close(void);
int32_t btmtk_tx_thread(void * arg);
void btmtk_cif_dump_fw_no_rsp(unsigned int flag);
void btmtk_cif_dump_rxd_backtrace(void);
void btmtk_reset_init(void);
void bt_notify_state(void);
int32_t btmtk_intcmd_query_thermal(void);

static inline void bt_wake_lock_init(struct bt_wake_lock *plock)
{
	if (plock) {
		plock->ws = wakeup_source_register(NULL, plock->name);
		if (!plock->ws)
			BTMTK_ERR("ERROR NO MEM\n");
#if (PM_QOS_CONTROL == 1)
		else
			pm_qos_add_request(&plock->qos_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
#endif
	}
}

static inline void bt_wake_lock_deinit(struct bt_wake_lock *plock)
{
	if (plock && plock->ws) {
		wakeup_source_unregister(plock->ws);
#if (PM_QOS_CONTROL == 1)
		pm_qos_remove_request(&plock->qos_req);
#endif
	}
}

static inline void bt_hold_wake_lock(struct bt_wake_lock *plock)
{
	if (plock && plock->ws) {
		__pm_stay_awake(plock->ws);
#if (PM_QOS_CONTROL == 1)
		pm_qos_update_request(&plock->qos_req, 1000);
#endif
	}
}

static inline void bt_hold_wake_lock_timeout(struct bt_wake_lock *plock, uint32_t ms)
{
	if (plock && plock->ws) {
		__pm_wakeup_event(plock->ws, ms);
#if (PM_QOS_CONTROL == 1)
		//pm_qos_update_request_timeout(&plock->qos_req, 1000, ms * 1000);
#endif
	}
}

static inline void bt_release_wake_lock(struct bt_wake_lock *plock)
{
	if (plock && plock->ws) {
		__pm_relax(plock->ws);
#if (PM_QOS_CONTROL == 1)
		pm_qos_update_request(&plock->qos_req, PM_QOS_DEFAULT_VALUE);
#endif
	}
}

static inline void bt_psm_init(struct bt_psm_ctrl *psm)
{
	init_completion(&psm->comp);
	strncpy(psm->wake_lock.name, "bt_psm", 6);
	psm->wake_lock.name[6] = '\0';
	bt_wake_lock_init(&psm->wake_lock);
}

static inline void bt_psm_deinit(struct bt_psm_ctrl *psm)
{
	bt_wake_lock_deinit(&psm->wake_lock);
}

static inline int osal_strtol(const char *str, unsigned int adecimal, long *res)
{
	if (sizeof(long) == 4)
		return kstrtou32(str, adecimal, (unsigned int *) res);
	else
		return kstrtol(str, adecimal, res);
}

/**
 * Send cmd dispatch evt
 */
#define HCI_EV_VENDOR			0xff

int32_t btmtk_set_power_on(struct hci_dev *hdev, u_int8_t for_precal);
int32_t btmtk_set_power_off(struct hci_dev *hdev, u_int8_t for_precal);
int btmtk_btif_internal_trx (uint8_t *buf, uint32_t count, BT_RX_EVT_HANDLER_CB cb, bool send_to_stack, bool is_blocking);
int btmtk_inttrx_DynamicAdjustTxPower_cb(uint8_t *buf, int len);
int btmtk_inttrx_DynamicAdjustTxPower(uint8_t mode, int8_t set_val, BT_RX_EVT_HANDLER_CB cb, bool is_blocking);
int32_t btmtk_intcmd_wmt_calibration(struct hci_dev *hdev);
int32_t btmtk_intcmd_wmt_blank_status(struct hci_dev *hdev, int32_t blank);
int32_t btmtk_intcmd_wmt_utc_sync(void);
int32_t btmtk_intcmd_set_fw_log(uint8_t flag);
int32_t btmtk_send_data(struct hci_dev *hdev, uint8_t *buf, uint32_t count);
bool bt_pwrctrl_support(void);
void bt_pwrctrl_pre_on(void);
void bt_pwrctrl_post_off(void);
int bt_dev_dbg_init(void);
int bt_dev_dbg_deinit(void);
void bthost_debug_print(void);
void btmtk_connsys_log_init(void);
void btmtk_connsys_log_register_event_cb(void (*func)(void));
void btmtk_connsys_log_deinit(void);
void btmtk_connsys_log_hold_sem(void);
void btmtk_connsys_log_release_sem(void);
ssize_t btmtk_connsys_log_read_to_user(char __user *buf, size_t count);
unsigned int btmtk_connsys_log_get_buf_size(void);
int btmtk_cif_send_calibration(struct btmtk_dev *bdev);
int btmtk_btif_send_cmd(struct btmtk_dev *bdev, struct sk_buff *skb, int delay,
	int retry, int pkt_type);

#if (USE_DEVICE_NODE == 1)
uint8_t btmtk_rx_data_valid(void);
int32_t btmtk_receive_data(struct hci_dev *hdev, uint8_t *buf, uint32_t count);
void btmtk_rx_flush(void);
void btmtk_register_rx_event_cb(struct hci_dev *hdev, BT_RX_EVENT_CB cb);
int main_driver_init(void);
void main_driver_exit(void);

void rx_queue_initialize(void);
void rx_queue_destroy(void);
uint8_t is_rx_queue_empty(void);
int32_t rx_skb_enqueue(struct sk_buff *skb);
void rx_dequeue(uint8_t *buffer, uint32_t size, uint32_t *plen);
void rx_queue_flush(void);
#endif

#if (DRIVER_CMD_CHECK == 1)
void cmd_list_initialize(void);
bool cmd_list_check(uint16_t opcode);
bool cmd_list_remove(uint16_t opcode);
bool cmd_list_append (uint16_t opcode);
void cmd_list_destory(void);
bool cmd_list_isempty(void);

bool cmd_workqueue_init(void);
void cmd_workqueue_exit(void);
void update_command_response_workqueue(void);
#endif

void dump_queue_initialize(void);
void add_dump_packet(const uint8_t *buffer, const uint32_t length, enum bt_direction_type type);
void show_all_dump_packet(void);

#endif
