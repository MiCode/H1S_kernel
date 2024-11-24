/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __LD_USBBT_H__
#define __LD_USBBT_H__
#include <common.h>
#include <malloc.h>
#include <usb.h>
#include <MsTypes.h>

#define MTKBT_CTRL_TX_EP 0
#define MTKBT_CTRL_RX_EP 1
#define MTKBT_INTR_EP 2
#define MTKBT_BULK_TX_EP 3
#define MTKBT_BULK_RX_EP 4

#define USB_INTR_MSG_TIMO 2000

#define MTK_GFP_ATOMIC 1

#define CRC_CHECK 0
#define BT_USB_PORT "bt_usb_port"

#define BTLDER				"[BT-LOADER] "
#define USB_TYPE_STANDARD	(0x00 << 5)
#define USB_TYPE_CLASS		(0x01 << 5)
#define USB_TYPE_VENDOR		(0x02 << 5)
#define USB_TYPE_RESERVED	(0x03 << 5)


#define usb_debug(fmt,...) printf("%s: "fmt, __func__, ##__VA_ARGS__)
#define usb_debug_raw(p, l, fmt, ...) \
	do { \
		int raw_count = 0; \
		const unsigned char *ptr = p; \
		printf("%s: "fmt, __func__, ##__VA_ARGS__); \
		for (raw_count = 0; raw_count < l; ++raw_count) \
			printf(" %02X", ptr[raw_count]); \
			printf("\n"); \
	} while (0)

#define os_kmalloc(size,flags) malloc(size)
#define os_kfree(ptr) free(ptr)

#define MTK_UDELAY(x)  udelay(x)
#define MTK_MDELAY(x)  mdelay(x)


//#define btusbdev_t struct usb_interface
#define btusbdev_t struct usb_device

#undef NULL
#define NULL ((void *)0)
#define s32 signed int
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef unsigned int UINT32;
typedef signed int INT32;
typedef unsigned char UINT8;
typedef unsigned long ULONG;
typedef unsigned char BOOL;

typedef struct __USBBT_DEVICE__ mtkbt_dev_t;

typedef struct {
	int (*usb_bulk_msg) (mtkbt_dev_t *dev, u32 epType, u8 *data, int size, int* realsize, int timeout);
	int (*usb_control_msg) (mtkbt_dev_t *dev, u32 epType, u8 request, u8 requesttype, u16 value, u16 index,
			u8 *data, int data_length, int timeout);
	int (*usb_interrupt_msg)(mtkbt_dev_t *dev, u32 epType, u8 *data, int size, int* realsize, int timeout);
} HC_IF;

struct __USBBT_DEVICE__
{
	void *priv_data;
	btusbdev_t* intf;
	struct usb_device *udev;
	struct usb_endpoint_descriptor *intr_ep;
	struct usb_endpoint_descriptor *bulk_tx_ep;
	struct usb_endpoint_descriptor *bulk_rx_ep;
	struct usb_endpoint_descriptor *isoc_tx_ep;
	struct usb_endpoint_descriptor *isoc_rx_ep;
	HC_IF *hci_if;
	int  (*connect)(btusbdev_t *dev, int flag);
	void (*disconnect)(btusbdev_t *dev);
	int  (*SetWoble)(btusbdev_t *dev);
	u32 chipid;
};//mtkbt_dev_t;

#define BT_INST(dev) (dev)

u8 LDbtusb_getWoBTW(void);
int Ldbtusb_connect (btusbdev_t *dev, int falg);
VOID *os_memcpy(VOID *dst, const VOID *src, UINT32 len);
VOID *os_memmove(VOID *dest, const void *src,UINT32 len);
VOID *os_memset(VOID *s, int c, size_t n);
VOID *os_kzalloc(size_t size, unsigned int flags);

void LD_load_code_from_bin(unsigned char **image, char *bin_name, char *path, mtkbt_dev_t *dev,u32 *code_len);

int do_setMtkBT(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
int do_getMtkBTWakeT(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);

#endif
