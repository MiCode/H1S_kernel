#ifndef __BATT_AUTH_CLASS__
#define __BATT_AUTH_CLASS__

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>

struct auth_device {
	const char *name;
	const struct auth_ops *ops;
	raw_spinlock_t io_lock;
	struct device dev;
	void *drv_data;
	struct gpio_desc *gpiod;
};

#define to_auth_device(obj) container_of(obj, struct auth_device, dev)

//battery pagedata
#define NVT_SN1         0x4e
#define NVT_SN2         0x56
#define SWD_SN1         0x53
#define SWD_SN2         0x4C
#define GY_SN1          0x47
#define GY_SN2          0x45

//battery supply
#define BATTERY_VENDOR_FIRST      0
#define BATTERY_VENDOR_SECOND     1
#define BATTERY_VENDOR_THIRD      2
#define BATTERY_VENDOR_UNKNOW    0xff

static char* battery_name_txt[] = {
        [BATTERY_VENDOR_FIRST] = "NVT",
        [BATTERY_VENDOR_SECOND] = "SWD",
        [BATTERY_VENDOR_THIRD] = "COS",
        [BATTERY_VENDOR_UNKNOW] = "UNKNOWN",
};

struct auth_ops {
	int (*set_cycle_count) (struct auth_device * auth_dev, u32 get_cycle, u32 set_cycle);
	int (*get_cycle_count) (struct auth_device * auth_dev, u32 * cycle);
	int (*get_ui_soh) (struct auth_device *auth_dev, u8 *ui_soh_data, int len);
	int (*set_ui_soh) (struct auth_device *auth_dev, u8 *ui_soh_data, int len, int raw_soh);
	int (*get_raw_soh) (struct auth_device *auth_dev);
	int (*set_raw_soh) (struct auth_device *auth_dev, int raw_soh);
};

int auth_device_set_cycle(struct auth_device *auth_dev, u32 set_cycle, u32 get_cycle);
int auth_device_get_cycle_count(struct auth_device *auth_dev, u32 * cycle);
int auth_device_get_ui_soh(struct auth_device *auth_dev, u8 *ui_soh_data, int len);
int auth_device_set_ui_soh(struct auth_device *auth_dev, u8 *ui_soh_data, int len, int raw_soh);
int auth_device_get_raw_soh(struct auth_device *auth_dev);
int auth_device_set_raw_soh(struct auth_device *auth_dev, int raw_soh);

struct auth_device *auth_device_register(const char *name,
					 struct device *parent,
					 void *devdata,
					 const struct auth_ops *ops);

void auth_device_unregister(struct auth_device *auth_dev);
struct auth_device *get_batt_auth_by_name(const char *name);
#endif				/* __BATT_AUTH_CLASS__ */