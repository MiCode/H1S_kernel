/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __LD_USBBT_H__
#define __LD_USBBT_H__
//#include <common.h>
//#include "usb_def.h"
//#include <MsTypes.h>

#include "types.h"
#include "loader_if.h"
#include "usb_type.h"

struct _usb_vid_pid_
{
    unsigned short vid;
    unsigned short pid;
    char name[10];
};
typedef struct _usb_vid_pid_ usb_vid_pid;

#define MTKBT_CTRL_TX_EP 0
#define MTKBT_CTRL_RX_EP 1
#define MTKBT_INTR_EP 2
#define MTKBT_BULK_TX_EP 3
#define MTKBT_BULK_RX_EP 4

#define MTK_GFP_ATOMIC 1

#define CRC_CHECK 0

#define BTLDER				"[BT-LOADER] "
#define USB_TYPE_STANDARD	(0x00 << 5)
#define USB_TYPE_CLASS		(0x01 << 5)
#define USB_TYPE_VENDOR		(0x02 << 5)
#define USB_TYPE_RESERVED	(0x03 << 5)


#define usb_debug(fmt,...) Printf("%s: "fmt, __func__, ##__VA_ARGS__)
#define usb_debug_raw(p, l, fmt, ...) \
	do { \
		int raw_count = 0; \
		const unsigned char *ptr = p; \
		Printf("%s: "fmt, __func__, ##__VA_ARGS__); \
		for (raw_count = 0; raw_count < l; ++raw_count) \
			Printf(" %02X", ptr[raw_count]); \
			Printf("\n"); \
	} while (0)

#define os_kmalloc(size,flags) x_mem_alloc(size)
#define os_kfree(ptr) x_mem_free(ptr)

#define MTK_UDELAY(x)  HAL_Delay_us(x)
#define MTK_MDELAY(x)  HAL_Delay_us(x*1000)

//#define btusbdev_t struct usb_interface
#define btusbdev_t struct usbdev

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
	struct usbdev *idev;
	endpoint_t *intr_ep;
	endpoint_t *bulk_tx_ep;
	endpoint_t *bulk_rx_ep;
	endpoint_t *isoc_tx_ep;
	endpoint_t *isoc_rx_ep;
	HC_IF *hci_if;
	int  (*connect)(btusbdev_t *dev, int flag);
	void (*disconnect)(btusbdev_t *dev);
	int  (*SetWoble)(btusbdev_t *dev);
};//mtkbt_dev_t;

#define BT_INST(dev) (dev)

u8 LDbtusb_getWoBTW(void);
int Ldbtusb_connect (btusbdev_t *dev,int falg);
VOID *os_memcpy(VOID *dst, const VOID *src, UINT32 len);
VOID *os_memmove(VOID *dest, const void *src,UINT32 len);
VOID *os_memset(VOID *s, int c, size_t n);
VOID *os_kzalloc(size_t size, unsigned int flags);
void LD_load_code_from_bin(unsigned char **image, char *bin_name, char *path, mtkbt_dev_t *dev,u32 *code_len);

void do_setMtkBT(usbdev_t *dev);
int getMtkBTWakeT(void);

#endif
