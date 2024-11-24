/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#ifndef _BTMTK_USB_H_
#define _BTMTK_USB_H_
#include <linux/usb.h>
#include "btmtk_define.h"
#include "btmtk_main.h"
#include "btmtk_woble.h"

#define HCI_MAX_COMMAND_SIZE	255
#define URB_MAX_BUFFER_SIZE	(4*1024)

#define BT0_MCU_INTERFACE_NUM 0
#define BT1_MCU_INTERFACE_NUM 3
#define BT_MCU_INTERFACE_NUM_MAX 4
#define BT_MCU_NUM_MAX 2

typedef int (*pdwnc_func) (u8 fgReset);
typedef int (*reset_func_ptr2) (unsigned int gpio, int init_value);
typedef int (*set_gpio_low)(u8 gpio);
typedef int (*set_gpio_high)(u8 gpio);

/**
 * Send cmd dispatch evt
 */
#define HCI_EV_VENDOR			0xff
#define HCI_USB_IO_BUF_SIZE		256


/* UHW CR mapping */
#define BT_MISC 0x70002510
#define BT_SUBSYS_RST 0x70002610
#define UDMA_INT_STA_BT 0x74000024
#define UDMA_INT_STA_BT1 0x74000308
#define BT_WDT_STATUS 0x740003A0
#define EP_RST_OPT 0x74011890
#define EP_RST_IN_OUT_OPT 0x00010001

struct btmtk_cif_chip_reset {
	/* For Whole chip reset */
	pdwnc_func pf_pdwndFunc;
	reset_func_ptr2 pf_resetFunc2;
	set_gpio_low pf_lowFunc;
	set_gpio_high pf_highFunc;
};

struct btmtk_usb_dev {
	struct usb_endpoint_descriptor	*intr_ep;
	/* EP10 OUT */
	struct usb_endpoint_descriptor	*intr_iso_tx_ep;
	/* EP10 IN */
	struct usb_endpoint_descriptor	*intr_iso_rx_ep;
	/* BULK CMD EP1 OUT or EP 11 OUT */
	struct usb_endpoint_descriptor	*bulk_cmd_tx_ep;
	/* EP15 in for reset */
	struct usb_endpoint_descriptor	*reset_intr_ep;
	struct usb_endpoint_descriptor	*bulk_tx_ep;
	struct usb_endpoint_descriptor	*bulk_rx_ep;
	struct usb_endpoint_descriptor	*isoc_tx_ep;
	struct usb_endpoint_descriptor	*isoc_rx_ep;

	struct usb_device	*udev;
	struct usb_interface	*intf;
	struct usb_interface	*isoc;
	struct usb_interface	*iso_channel;


	struct usb_anchor	tx_anchor;
	struct usb_anchor	intr_anchor;
	struct usb_anchor	bulk_anchor;
	struct usb_anchor	isoc_anchor;
	struct usb_anchor	ctrl_anchor;
	struct usb_anchor	ble_isoc_anchor;

	__u8	cmdreq_type;
	__u8	cmdreq;

	int new_isoc_altsetting;
	int new_isoc_altsetting_interface;

	unsigned char	*o_usb_buf;

	unsigned char	*urb_intr_buf;
	unsigned char	*urb_bulk_buf;
	unsigned char	*urb_ble_isoc_buf;
	struct btmtk_woble	bt_woble;
};
#endif
