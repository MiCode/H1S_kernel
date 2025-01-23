#ifndef FP_ULTRA_H
#define FP_ULTRA_H

#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/time.h>

#define DTS_IPC_GPIO				"xiaomi,gpio_ipc"
#define DTS_WUHB_GPIO				"xiaomi,gpio_wuhb"
#define DTS_INTR2_GPIO				"xiaomi,gpio_intr2"

#define FP_IPC_DEV_NAME "qbt_ipc"
#define FP_WUHB_DEV_NAME "qbt_fd"

#define FD_GPIO_ACTIVE_LOW 0  // FIXME: Need to get this flag value from devicetree

#define QBT_INPUT_DEV_VERSION 0x0100

#define MAX_FW_EVENTS 128

/*
 * enum qbt_finger_events -
 *      enumeration of qbt finger events
 * @QBT_EVENT_FINGER_UP - finger up detected
 * @QBT_EVENT_FINGER_DOWN - finger down detected
 * @QBT_EVENT_FINGER_MOVE - finger move detected
 */
enum qbt_finger_events {
	QBT_EVENT_FINGER_UP,
	QBT_EVENT_FINGER_DOWN,
	QBT_EVENT_FINGER_MOVE
};


/*
 * enum qbt_fw_event -
 *      enumeration of firmware events
 * @FW_EVENT_FINGER_DOWN - finger down detected
 * @FW_EVENT_FINGER_UP - finger up detected
 * @FW_EVENT_IPC - an IPC from the firmware is pending
 */
enum qbt_fw_event {
	FW_EVENT_FINGER_DOWN = 1,
	FW_EVENT_FINGER_UP = 2,
	FW_EVENT_IPC = 3,
};


struct fw_ipc_info {
	int gpio;
	int irq;
	bool irq_enabled;
	struct work_struct work;
    bool work_init;
};

struct finger_detect_gpio {
	int gpio;
	int active_low;
	int irq;
	struct work_struct work;
	int last_gpio_state;
	int event_reported;
	bool irq_enabled;
    bool work_init;
};

struct ipc_event {
	enum qbt_fw_event ev;
};

struct fd_event {
	struct timespec64 timestamp;
	int X;
	int Y;
	int id;
	int state;
	bool touch_valid;
};

struct fd_userspace_buf {
	uint32_t num_events;
	struct fd_event fd_events[MAX_FW_EVENTS];
};

/*
 * struct qbt_wuhb_connected_status -
 *		used to query whether WUHB INT line is connected
 * @is_wuhb_connected - if non-zero, WUHB INT line is connected
 */
struct qbt_wuhb_connected_status {
	_Bool is_wuhb_connected;
};


#endif