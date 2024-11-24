/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/quirks.h>
#include <linux/firmware.h>
#include <asm/unaligned.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include "btmtk_define.h"
#include "btmtk_usb.h"
#include "btmtk_main.h"
#include "btmtk_woble.h"

static struct usb_driver btusb_driver;
static struct btmtk_cif_chip_reset reset_func;
static int intf_to_idx[BT_MCU_INTERFACE_NUM_MAX] = {0, -1, -1, 1};
static struct btmtk_usb_dev g_usb_dev[BT_MCU_MINIMUM_INTERFACE_NUM][BT_MCU_NUM_MAX];

static const struct usb_device_id btusb_table[] = {
	/* Mediatek MT7961 */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7961, 0xe0, 0x01, 0x01) },
	/* Mediatek MT7915 */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7915, 0xe0, 0x01, 0x01) },
	/* Mediatek MT7663 */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7663, 0xe0, 0x01, 0x01) },
	/* Mediatek MT7922 */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7922, 0xe0, 0x01, 0x01) },

	{ }	/* Terminating entry */
};

static char event_need_compare[EVENT_COMPARE_SIZE] = {0};
static char event_need_compare_len;
static char event_compare_status;

static DEFINE_MUTEX(btmtk_usb_ops_mutex);
#define USB_OPS_MUTEX_LOCK()	mutex_lock(&btmtk_usb_ops_mutex)
#define USB_OPS_MUTEX_UNLOCK()	mutex_unlock(&btmtk_usb_ops_mutex)

MODULE_DEVICE_TABLE(usb, btusb_table);

/* remove #define BTUSB_MAX_ISOC_FRAMES	10
 * ISCO_FRAMES max is 24
 */
#define BTUSB_MAX_ISOC_FRAMES	24

#define BTUSB_INTR_RUNNING	0
#define BTUSB_BULK_RUNNING	1
#define BTUSB_ISOC_RUNNING	2
#define BTUSB_SUSPENDING	3
#define BTUSB_DID_ISO_RESUME	4
#define BTUSB_BLE_ISOC_RUNNING	5

#define DEVICE_VENDOR_REQUEST_IN	0xc0
#define DEVICE_CLASS_REQUEST_OUT	0x20
#define USB_CTRL_IO_TIMO		100

#define BTMTK_IS_BT_0_INTF(ifnum_base) \
	(ifnum_base == BT0_MCU_INTERFACE_NUM)

#define BTMTK_IS_BT_1_INTF(ifnum_base) \
	(ifnum_base == BT1_MCU_INTERFACE_NUM)

#define BTMTK_CIF_GET_DEV_PRIV(bdev, intf, ifnum_base) \
	do { \
		bdev = usb_get_intfdata(intf); \
		ifnum_base = intf->cur_altsetting->desc.bInterfaceNumber; \
	} while (0)

static int btmtk_cif_allocate_memory(struct btmtk_usb_dev *cif_dev);
static void btmtk_cif_free_memory(struct btmtk_usb_dev *cif_dev);
static int btmtk_cif_write_uhw_register(struct btmtk_dev *bdev, u32 reg, u32 val);
static int btmtk_cif_read_uhw_register(struct btmtk_dev *bdev, u32 reg, u32 *val);

static int btmtk_usb_send_and_recv(struct btmtk_dev *bdev,
		struct sk_buff *skb,
		const uint8_t *event, const int event_len,
		int delay, int retry, int pkt_type);
static void btmtk_usb_chip_reset_notify(struct btmtk_dev *bdev);
static int btmtk_usb_event_filter(struct btmtk_dev *bdev, struct sk_buff *skb);
static int btmtk_usb_send_cmd(struct btmtk_dev *bdev, struct sk_buff *skb,
		int delay, int retry, int pkt_type);
static int btmtk_usb_read_register(struct btmtk_dev *bdev, u32 reg, u32 *val);
static int btmtk_usb_write_register(struct btmtk_dev *bdev, u32 reg, u32 val);

static void btmtk_usb_cif_mutex_lock(struct btmtk_dev *bdev)
{
	USB_OPS_MUTEX_LOCK();
}

static void btmtk_usb_cif_mutex_unlock(struct btmtk_dev *bdev)
{
	USB_OPS_MUTEX_UNLOCK();
}

static inline void btusb_free_frags(struct btmtk_dev *bdev)
{
	unsigned long flags;

	spin_lock_irqsave(&bdev->rxlock, flags);

	kfree_skb(bdev->evt_skb);
	bdev->evt_skb = NULL;

	kfree_skb(bdev->sco_skb);
	bdev->sco_skb = NULL;

	spin_unlock_irqrestore(&bdev->rxlock, flags);
}

static int btusb_recv_isoc(struct btmtk_dev *bdev, void *buffer, int count)
{
	struct sk_buff *skb;
	int err = 0;

	spin_lock(&bdev->rxlock);
	skb = bdev->sco_skb;

	while (count) {
		int len;

		if (!skb) {
			skb = bt_skb_alloc(HCI_MAX_SCO_SIZE, GFP_ATOMIC);
			if (!skb) {
				err = -ENOMEM;
				break;
			}

			hci_skb_pkt_type(skb) = HCI_SCODATA_PKT;
			hci_skb_expect(skb) = HCI_SCO_HDR_SIZE;
		}

		len = min_t(uint, hci_skb_expect(skb), count);
		memcpy(skb_put(skb, len), buffer, len);

		count -= len;
		buffer += len;
		hci_skb_expect(skb) -= len;

		if (skb->len == HCI_SCO_HDR_SIZE) {
			/* Complete SCO header */
			hci_skb_expect(skb) = hci_sco_hdr(skb)->dlen;

			if (skb_tailroom(skb) < hci_skb_expect(skb)) {
				kfree_skb(skb);
				skb = NULL;

				err = -EILSEQ;
				break;
			}
		}

		if (!hci_skb_expect(skb)) {
			/* Complete frame */
			hci_recv_frame(bdev->hdev, skb);
			skb = NULL;
		}
	}

	bdev->sco_skb = skb;
	spin_unlock(&bdev->rxlock);

	return err;
}

static void btusb_intr_complete(struct urb *urb)
{
	struct hci_dev *hdev = NULL;
	struct btmtk_dev *bdev = NULL;
	struct btmtk_usb_dev *cif_dev = NULL;
	int err;
	u8 *buf;
	static u8 intr_blocking_usb_warn;

	if (urb == NULL) {
		BTMTK_ERR("%s: ERROR, urb is NULL!", __func__);
		return;
	}

	hdev = urb->context;
	if (hdev == NULL) {
		BTMTK_ERR("%s: ERROR, hdev is NULL!", __func__);
		return;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL) {
		BTMTK_ERR("%s: ERROR, bdev is NULL!", __func__);
		return;
	}

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (cif_dev == NULL) {
		BTMTK_ERR("%s: ERROR, cif_dev is NULL!", __func__);
		return;
	}

	BTMTK_DBG("%s urb %p status %d count %d", hdev->name, urb, urb->status,
	       urb->actual_length);

	if (urb->status != 0 && intr_blocking_usb_warn < 10) {
		intr_blocking_usb_warn++;
		BTMTK_WARN("%s: urb %p urb->status %d count %d", __func__,
			urb, urb->status, urb->actual_length);
	} else if (urb->status == 0 && urb->actual_length != 0)
		intr_blocking_usb_warn = 0;

	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;

		if (!cif_dev->urb_intr_buf) {
			BTMTK_ERR("%s: bdev->urb_intr_buf is NULL!", __func__);
			return;
		}

		buf = urb->transfer_buffer;
		if (urb->actual_length >= URB_MAX_BUFFER_SIZE ||
			(urb->actual_length != (buf[1] + 2) && urb->actual_length > 1)) {
			BTMTK_ERR("%s: urb->actual_length is invalid, buf[1] = %d!",
				__func__, buf[1]);
			btmtk_hci_snoop_print(urb->actual_length, urb->transfer_buffer);
			goto intr_resub;
		}
		memset(cif_dev->urb_intr_buf, 0, URB_MAX_BUFFER_SIZE);
		cif_dev->urb_intr_buf[0] = HCI_EVENT_PKT;
		memcpy(cif_dev->urb_intr_buf + 1, urb->transfer_buffer, urb->actual_length);

		BTMTK_DBG("%s ,urb->actual_length = %d", __func__, urb->actual_length);
		BTMTK_DBG_RAW(cif_dev->urb_intr_buf, urb->actual_length + 1, "%s, recv evt", __func__);
		BTMTK_DBG_RAW(urb->transfer_buffer, urb->actual_length, "%s, recv evt", __func__);
		if (cif_dev->urb_intr_buf[1] == 0xFF && urb->actual_length == 1) {
			/* We can't use usb_control_msg in interrupt.
			 * If you use usb_control_msg , it will cause crash.
			 * Receive a bytes 0xFF from controller, it's WDT interrupt to driver.
			 * WDT interrupt is a mechanism to do L0.5 reset.
			 */
			schedule_work(&bdev->reset_waker);
			goto intr_resub;
		}

#if 0
		/* need to remove after SQC done*/
		if (buf[0] == 0x3E)
			btmtk_hci_snoop_save_adv_event(urb->actual_length, urb->transfer_buffer);
		else
			btmtk_hci_snoop_save_event(urb->actual_length, urb->transfer_buffer);
#endif
		err = btmtk_recv(hdev, cif_dev->urb_intr_buf, urb->actual_length + 1);
		if (err) {
			BTMTK_ERR("%s corrupted event packet, urb_intr_buf = %p, transfer_buffer = %p",
				hdev->name, cif_dev->urb_intr_buf, urb->transfer_buffer);
			btmtk_hci_snoop_print(urb->actual_length, urb->transfer_buffer);
			btmtk_hci_snoop_print(urb->actual_length + 1, cif_dev->urb_intr_buf);
			hdev->stat.err_rx++;
		}
	} else if (urb->status == -ENOENT) {
		BTMTK_INFO("%s: urb->status is ENOENT!", __func__);
		return;
	}

intr_resub:
	if (!test_bit(BTUSB_INTR_RUNNING, &bdev->flags)) {
		BTMTK_INFO("%s: test_bit is not running!", __func__);
		return;
	}

	usb_mark_last_busy(cif_dev->udev);
	usb_anchor_urb(urb, &cif_dev->intr_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected
		 */
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p failed to resubmit (%d)",
			       hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static int btusb_submit_intr_reset_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;
	struct btmtk_usb_dev *cif_dev = NULL;

	/* If WDT reset happened, fw will send a bytes (FF) to host */
	BTMTK_DBG("%s", hdev->name);

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (!cif_dev->reset_intr_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;
	/* Default size is 16 */
	/* size = le16_to_cpu(data->intr_ep->wMaxPacketSize); */
	/* 7663 & 7668 & Buzzard Endpoint description.
	 * bEndpointAddress     0x8f  EP 15 IN
	 * wMaxPacketSize     0x0001  1x 1 bytes
	 */
	size = le16_to_cpu(HCI_MAX_EVENT_SIZE);

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvintpipe(cif_dev->udev, cif_dev->reset_intr_ep->bEndpointAddress);

	/* fw issue, we need to submit urb with a byte
	 * If driver set size = le16_to_cpu(HCI_MAX_EVENT_SIZE) to usb_fill_int_urb
	 * We can't get interrupt callback from bus.
	 */
	usb_fill_int_urb(urb, cif_dev->udev, pipe, buf, 1,
			 btusb_intr_complete, hdev, cif_dev->reset_intr_ep->bInterval);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &cif_dev->intr_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p submission failed (%d)",
				   hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_mtk_wmt_recv(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = NULL;
	struct sk_buff *skb;
	int err;

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;

	BTMTK_DBG("%s : %s urb %p status %d count %d", __func__, hdev->name, urb, urb->status,
	       urb->actual_length);

	if (urb->status == 0 && urb->actual_length > 0) {
		BTMTK_DBG_RAW(urb->transfer_buffer, urb->actual_length, "%s, recv evt", __func__);
		hdev->stat.byte_rx += urb->actual_length;
		skb = bt_skb_alloc(HCI_MAX_EVENT_SIZE, GFP_ATOMIC);
		if (!skb) {
			BTMTK_ERR("%s skb is null!", __func__);
			hdev->stat.err_rx++;
			goto exit;
		}

		if (urb->actual_length >= HCI_MAX_EVENT_SIZE) {
			BTMTK_ERR("%s urb->actual_length is invalid!", __func__);
			BTMTK_INFO_RAW(urb->transfer_buffer, urb->actual_length,
				"urb->actual_length:%d, urb->transfer_buffer:%p",
				urb->actual_length, urb->transfer_buffer);
			kfree_skb(skb);
			hdev->stat.err_rx++;
			goto exit;
		}
		hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
		memcpy(skb_put(skb, urb->actual_length), urb->transfer_buffer, urb->actual_length);
		BTMTK_DBG_RAW(skb->data, skb->len, "%s, skb recv evt", __func__);

		hci_recv_frame(hdev, skb);
		return;
	} else if (urb->status == -ENOENT) {
		/* Avoid suspend failed when usb_kill_urb */
		goto exit;
	}

	usb_mark_last_busy(cif_dev->udev);

	/* The URB complete handler is still called with urb->actual_length = 0
	 * when the event is not available, so we should keep re-submitting
	 * URB until WMT event returns, Also, It's necessary to wait some time
	 * between the two consecutive control URBs to relax the target device
	 * to generate the event. Otherwise, the WMT event cannot return from
	 * the device successfully.
	 */
	udelay(100);

	usb_anchor_urb(urb, &cif_dev->ctrl_anchor);
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		kfree(urb->setup_packet);
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected
		 */
		if (err != -EPERM && err != -ENODEV)
			usb_unanchor_urb(urb);
	}

	return;

exit:
	kfree(urb->setup_packet);
}

static int btusb_submit_wmt_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = NULL;
	struct usb_ctrlrequest *dr;
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;
	unsigned int ifnum_base;

	BTMTK_DBG("%s : %s", __func__, hdev->name);

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(HCI_MAX_EVENT_SIZE);

	dr = kmalloc(sizeof(*dr), GFP_KERNEL);
	if (!dr) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;

	ifnum_base = cif_dev->intf->cur_altsetting->desc.bInterfaceNumber;

	if (BTMTK_IS_BT_0_INTF(ifnum_base)) {
		dr->bRequestType = 0xC0;
		dr->bRequest     = 0x01;
		dr->wIndex       = 0;
		dr->wValue       = 0x30;
		dr->wLength      = __cpu_to_le16(size);
	} else if (BTMTK_IS_BT_1_INTF(ifnum_base)) {
		dr->bRequestType = 0xA1;
		dr->bRequest     = 0x01;
		dr->wIndex       = 0x03;
		dr->wValue       = 0x30;
		dr->wLength      = __cpu_to_le16(size);
	}

	pipe = usb_rcvctrlpipe(cif_dev->udev, 0);

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf) {
		kfree(dr);
		usb_free_urb(urb);
		return -ENOMEM;
	}

	usb_fill_control_urb(urb, cif_dev->udev, pipe, (void *)dr,
			     buf, size, btusb_mtk_wmt_recv, hdev);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &cif_dev->ctrl_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p submission failed (%d)",
					hdev->name, urb, -err);
		kfree(dr);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static int btusb_submit_intr_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = NULL;
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	BTMTK_DBG("%s", hdev->name);

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;

	if (!cif_dev->intr_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	/* size = le16_to_cpu(data->intr_ep->wMaxPacketSize); */
	/* 7663 & 7668 & Buzzard Endpoint description.
	 * bEndpointAddress     0x81  EP 1 IN
	 * wMaxPacketSize     0x0010  1x 16 bytes
	 */
	size = le16_to_cpu(HCI_MAX_EVENT_SIZE);
	BTMTK_INFO("%s: maximum packet size:%d", __func__, size);

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvintpipe(cif_dev->udev, cif_dev->intr_ep->bEndpointAddress);

	usb_fill_int_urb(urb, cif_dev->udev, pipe, buf, size,
			 btusb_intr_complete, hdev, cif_dev->intr_ep->bInterval);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &cif_dev->intr_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p submission failed (%d)",
			       hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_bulk_complete(struct urb *urb)
{
	struct hci_dev *hdev = NULL;
	struct btmtk_dev *bdev = NULL;
	struct btmtk_usb_dev *cif_dev = NULL;
	int err;
	u8 *buf;
	u16 len = 0;
	static u8 block_bulkin_usb_warn;

	if (urb == NULL) {
		BTMTK_ERR("%s: ERROR, urb is NULL!", __func__);
		return;
	}

	hdev = urb->context;
	if (hdev == NULL) {
		BTMTK_ERR("%s: ERROR, hdev is NULL!", __func__);
		return;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL) {
		BTMTK_ERR("%s: ERROR, bdev is NULL!", __func__);
		return;
	}

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (cif_dev == NULL) {
		BTMTK_ERR("%s: ERROR, cif_dev is NULL!", __func__);
		return;
	}

	if (urb->status != 0 && block_bulkin_usb_warn < 10) {
		block_bulkin_usb_warn++;
		BTMTK_INFO("%s: urb %p urb->status %d count %d", __func__, urb,
			urb->status, urb->actual_length);
	} else if (urb->status == 0 && urb->actual_length != 0)
		block_bulkin_usb_warn = 0;

	/*
	 * This flag didn't support in kernel 4.x
	 * Driver will remove it
	 * if (!test_bit(HCI_RUNNING, &hdev->flags))
	 * return;
	 */
	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;
		if (!cif_dev->urb_bulk_buf) {
			BTMTK_ERR("%s: bdev->urb_bulk_buf is NULL!", __func__);
			return;
		}

		buf = urb->transfer_buffer;
		len = buf[2] + ((buf[3] << 8) & 0xff00);
		if (urb->actual_length >= URB_MAX_BUFFER_SIZE ||
			urb->actual_length != len + 4) {
			BTMTK_ERR("%s urb->actual_length is invalid, len = %d!", __func__, len);
			btmtk_hci_snoop_print(urb->actual_length, urb->transfer_buffer);
			goto bulk_resub;
		}
		memset(cif_dev->urb_bulk_buf, 0, URB_MAX_BUFFER_SIZE);
		cif_dev->urb_bulk_buf[0] = HCI_ACLDATA_PKT;
		memcpy(cif_dev->urb_bulk_buf + 1, urb->transfer_buffer, urb->actual_length);

		/* BTMTK_DBG_RAW(bdev->urb_bulk_buf, urb->actual_length + 1, "%s, recv from bulk", __func__); */
#if 0
		/* need to remove after SQC done*/
		btmtk_hci_snoop_save_acl(urb->actual_length, urb->transfer_buffer);
#endif
		err = btmtk_recv(hdev, cif_dev->urb_bulk_buf, urb->actual_length + 1);
		if (err) {
			BTMTK_ERR("%s corrupted ACL packet, urb_bulk_buf = %p, transfer_buffer = %p",
				hdev->name, cif_dev->urb_bulk_buf, urb->transfer_buffer);
			btmtk_hci_snoop_print(urb->actual_length, urb->transfer_buffer);
			btmtk_hci_snoop_print(urb->actual_length + 1, cif_dev->urb_bulk_buf);
			hdev->stat.err_rx++;
		}
	} else if (urb->status == -ENOENT) {
		/* Avoid suspend failed when usb_kill_urb */
		BTMTK_INFO("%s urb %p status %d count %d", hdev->name, urb, urb->status,
			urb->actual_length);
		return;
	}

bulk_resub:
	if (!test_bit(BTUSB_BULK_RUNNING, &bdev->flags)) {
		BTMTK_INFO("%s test flag failed", __func__);
		return;
	}

	usb_anchor_urb(urb, &cif_dev->bulk_anchor);
	usb_mark_last_busy(cif_dev->udev);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected
		 */
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p failed to resubmit (%d)",
			       hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static int btusb_submit_bulk_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = NULL;
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size = HCI_MAX_FRAME_SIZE;

	BTMTK_DBG("%s", hdev->name);

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;

	if (!cif_dev->bulk_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvbulkpipe(cif_dev->udev, cif_dev->bulk_rx_ep->bEndpointAddress);

	usb_fill_bulk_urb(urb, cif_dev->udev, pipe, buf, size,
			  btusb_bulk_complete, hdev);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_mark_last_busy(cif_dev->udev);
	usb_anchor_urb(urb, &cif_dev->bulk_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p submission failed (%d)",
			       hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_ble_isoc_complete(struct urb *urb)
{
	struct hci_dev *hdev = NULL;
	struct btmtk_dev *bdev = NULL;
	struct btmtk_usb_dev *cif_dev = NULL;
	int err;
	u8 *isoc_buf;
	int isoc_pkt_len;

	/*
	 * This flag didn't support in kernel 4.x
	 * Driver will remove it
	 * if (!test_bit(HCI_RUNNING, &hdev->flags))
	 * return;
	 */
	if (urb == NULL) {
		BTMTK_ERR("%s: ERROR, urb is NULL!", __func__);
		return;
	}

	hdev = urb->context;
	if (hdev == NULL) {
		BTMTK_ERR("%s: ERROR, hdev is NULL!", __func__);
		return;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL) {
		BTMTK_ERR("%s: ERROR, bdev is NULL!", __func__);
		return;
	}

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (cif_dev == NULL) {
		BTMTK_ERR("%s: ERROR, cif_dev is NULL!", __func__);
		return;
	}

	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;
		isoc_buf = urb->transfer_buffer;

		if (!cif_dev->urb_ble_isoc_buf) {
			BTMTK_ERR("%s: bdev->urb_ble_isoc_buf is NULL!", __func__);
			return;
		}
		isoc_pkt_len = isoc_buf[2] + (isoc_buf[3] << 8) + HCI_ISO_PKT_HEADER_SIZE;

		/* Skip padding */
		BTMTK_DBG("%s: isoc_pkt_len = %d, urb->actual_length = %d", __func__, isoc_pkt_len, urb->actual_length);
		if (isoc_pkt_len == HCI_ISO_PKT_HEADER_SIZE) {
			BTMTK_DBG("%s: goto ble_iso_resub", __func__);
			goto ble_iso_resub;
		}

		if (urb->actual_length + HCI_ISO_PKT_WITH_ACL_HEADER_SIZE > URB_MAX_BUFFER_SIZE) {
			BTMTK_ERR("%s urb->actual_length is invalid!", __func__);
			btmtk_hci_snoop_print(urb->actual_length, urb->transfer_buffer);
			goto ble_iso_resub;
		}
		/* It's mtk specific heade for stack
		 * hci layered didn't support 0x05 for ble iso, it will drop the packet type with 0x05
		 * Driver will replace 0x05 to 0x02
		 * header format : 0x02 0x00 0x44 xx xx + isoc packet header & payload
		 */
		memset(cif_dev->urb_ble_isoc_buf, 0, URB_MAX_BUFFER_SIZE);
		cif_dev->urb_ble_isoc_buf[0] = HCI_ACLDATA_PKT;
		cif_dev->urb_ble_isoc_buf[1] = 0x00;
		cif_dev->urb_ble_isoc_buf[2] = 0x44;
		cif_dev->urb_ble_isoc_buf[3] = (isoc_pkt_len & 0x00ff);
		cif_dev->urb_ble_isoc_buf[4] = (isoc_pkt_len >> 8);
		memcpy(cif_dev->urb_ble_isoc_buf + HCI_ISO_PKT_WITH_ACL_HEADER_SIZE,
			urb->transfer_buffer, isoc_pkt_len + HCI_ISO_PKT_HEADER_SIZE);

		BTMTK_DBG_RAW(cif_dev->urb_ble_isoc_buf,
			isoc_pkt_len + HCI_ISO_PKT_WITH_ACL_HEADER_SIZE,
			"%s: raw data is :", __func__);

		err = btmtk_recv(hdev, cif_dev->urb_ble_isoc_buf,
			isoc_pkt_len + HCI_ISO_PKT_WITH_ACL_HEADER_SIZE);
		if (err) {
			BTMTK_ERR("%s corrupted ACL packet", hdev->name);
			hdev->stat.err_rx++;
		}
	} else if (urb->status == -ENOENT) {
		BTMTK_INFO("%s: urb->status is ENOENT!", __func__);
		return;
	}

ble_iso_resub:
	if (!test_bit(BTUSB_BLE_ISOC_RUNNING, &bdev->flags)) {
		BTMTK_INFO("%s: bdev->flags is RUNNING!", __func__);
		return;
	}

	usb_anchor_urb(urb, &cif_dev->ble_isoc_anchor);
	usb_mark_last_busy(cif_dev->udev);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected
		 */
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p failed to resubmit (%d)",
			       hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static int btusb_submit_intr_ble_isoc_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = NULL;
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;

	if (!cif_dev->intr_iso_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;
	/* Default size is 16 */
	/* size = le16_to_cpu(data->intr_ep->wMaxPacketSize); */
	/* we need to consider the wMaxPacketSize in BLE ISO */
	size = le16_to_cpu(2000);

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvintpipe(cif_dev->udev, cif_dev->intr_iso_rx_ep->bEndpointAddress);
	BTMTK_INFO("btusb_submit_intr_iso_urb : polling  0x%02X",  cif_dev->intr_iso_rx_ep->bEndpointAddress);

	usb_fill_int_urb(urb, cif_dev->udev, pipe, buf, size,
			 btusb_ble_isoc_complete, hdev, cif_dev->intr_iso_rx_ep->bInterval);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &cif_dev->ble_isoc_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p submission failed (%d)",
				   hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_isoc_complete(struct urb *urb)
{
	struct hci_dev *hdev = NULL;
	struct btmtk_dev *bdev = NULL;
	struct btmtk_usb_dev *cif_dev = NULL;
	int i, err;

	if (urb == NULL) {
		BTMTK_ERR("%s: ERROR, urb is NULL!", __func__);
		return;
	}

	hdev = urb->context;
	if (hdev == NULL) {
		BTMTK_ERR("%s: ERROR, hdev is NULL!", __func__);
		return;
	}

	bdev = hci_get_drvdata(hdev);
	if (bdev == NULL) {
		BTMTK_ERR("%s: ERROR, bdev is NULL!", __func__);
		return;
	}

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (cif_dev == NULL) {
		BTMTK_ERR("%s: ERROR, cif_dev is NULL!", __func__);
		return;
	}


	BTMTK_DBG("%s urb %p status %d count %d", hdev->name, urb, urb->status,
	       urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {
			unsigned int offset = urb->iso_frame_desc[i].offset;
			unsigned int length = urb->iso_frame_desc[i].actual_length;

			if (urb->iso_frame_desc[i].status)
				continue;

			hdev->stat.byte_rx += length;

			if (btusb_recv_isoc(bdev, urb->transfer_buffer + offset,
					    length) < 0) {
				BTMTK_ERR("%s corrupted SCO packet", hdev->name);
				hdev->stat.err_rx++;
			}
		}
	} else if (urb->status == -ENOENT) {
		BTMTK_INFO("%s: urb->status is ENOENT!", __func__);
		return;
	}

	if (!test_bit(BTUSB_ISOC_RUNNING, &bdev->flags)) {
		BTMTK_INFO("%s: bdev->flags is RUNNING!", __func__);
		return;
	}

	usb_anchor_urb(urb, &cif_dev->isoc_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected
		 */
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p failed to resubmit (%d)",
			       hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static inline void __fill_isoc_descriptor(struct urb *urb, int len, int mtu)
{
	int i, offset = 0;

	BTMTK_DBG("len %d mtu %d", len, mtu);

	for (i = 0; i < BTUSB_MAX_ISOC_FRAMES && len >= mtu;
					i++, offset += mtu, len -= mtu) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = mtu;
	}

	if (len && i < BTUSB_MAX_ISOC_FRAMES) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = len;
		i++;
	}

	urb->number_of_packets = i;
}

static int btusb_submit_isoc_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = NULL;
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	BTMTK_DBG("%s", hdev->name);
	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;

	if (!cif_dev->isoc_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(cif_dev->isoc_rx_ep->wMaxPacketSize) *
						BTUSB_MAX_ISOC_FRAMES;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvisocpipe(cif_dev->udev, cif_dev->isoc_rx_ep->bEndpointAddress);

	usb_fill_int_urb(urb, cif_dev->udev, pipe, buf, size, btusb_isoc_complete,
			 hdev, cif_dev->isoc_rx_ep->bInterval);

	urb->transfer_flags = URB_FREE_BUFFER | URB_ISO_ASAP;

	__fill_isoc_descriptor(urb, size,
			       le16_to_cpu(cif_dev->isoc_rx_ep->wMaxPacketSize));

	usb_anchor_urb(urb, &cif_dev->isoc_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p submission failed (%d)",
			       hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	unsigned long flags;

	BTMTK_DBG("%s urb %p status %d count %d", hdev->name, urb, urb->status,
	       urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (!urb->status)
		hdev->stat.byte_tx += urb->transfer_buffer_length;
	else
		hdev->stat.err_tx++;

done:
	spin_lock_irqsave(&bdev->txlock, flags);
	bdev->tx_in_flight--;
	spin_unlock_irqrestore(&bdev->txlock, flags);

	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static void btusb_isoc_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;

	BTMTK_DBG("%s urb %p status %d count %d", hdev->name, urb, urb->status,
	       urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (!urb->status)
		hdev->stat.byte_tx += urb->transfer_buffer_length;
	else
		hdev->stat.err_tx++;

done:
	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static int btmtk_usb_open(struct hci_dev *hdev)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	int err;
	unsigned int ifnum_base;

	BTMTK_INFO("%s enter!", __func__);

	BTMTK_DBG("%s", hdev->name);

	err = usb_autopm_get_interface(cif_dev->intf);
	if (err < 0)
		return err;

	cif_dev->intf->needs_remote_wakeup = 1;

	if (test_and_set_bit(BTUSB_INTR_RUNNING, &bdev->flags))
		goto done;

	ifnum_base = cif_dev->intf->cur_altsetting->desc.bInterfaceNumber;

	if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id)) {
		BTMTK_INFO("%s 7961 submit urb\n", __func__);
		if (BTMTK_IS_BT_0_INTF(ifnum_base)) {
			if (cif_dev->reset_intr_ep) {
				err = btusb_submit_intr_reset_urb(hdev, GFP_KERNEL);
				if (err < 0)
					goto failed;
			} else
				BTMTK_INFO("%s, reset_intr_ep missing, don't submit_intr_reset_urb!",
					__func__);

			if (cif_dev->intr_iso_rx_ep) {
				err = btusb_submit_intr_ble_isoc_urb(hdev, GFP_KERNEL);
				if (err < 0) {
					usb_kill_anchored_urbs(&cif_dev->ble_isoc_anchor);
					goto failed;
				}
			} else
				BTMTK_INFO("%s, intr_iso_rx_ep missing, don't submit_intr_ble_isoc_urb!",
					__func__);
		} else if (BTMTK_IS_BT_1_INTF(ifnum_base)) {
			/*need to do in bt_open in btmtk_main.c */
			/* btmtk_usb_send_power_on_cmd_7668(hdev); */
		}
	}
	err = btusb_submit_intr_urb(hdev, GFP_KERNEL);
	if (err < 0)
		goto failed;

	err = btusb_submit_bulk_urb(hdev, GFP_KERNEL);
	if (err < 0) {
		usb_kill_anchored_urbs(&cif_dev->intr_anchor);
		goto failed;
	}


	set_bit(BTUSB_BULK_RUNNING, &bdev->flags);

done:
	usb_autopm_put_interface(cif_dev->intf);
	return 0;

failed:
	clear_bit(BTUSB_INTR_RUNNING, &bdev->flags);
	usb_autopm_put_interface(cif_dev->intf);
	return err;
}

static void btusb_stop_traffic(struct btmtk_usb_dev *cif_dev)
{
	usb_kill_anchored_urbs(&cif_dev->intr_anchor);
	usb_kill_anchored_urbs(&cif_dev->bulk_anchor);
	usb_kill_anchored_urbs(&cif_dev->isoc_anchor);
	usb_kill_anchored_urbs(&cif_dev->ctrl_anchor);
	usb_kill_anchored_urbs(&cif_dev->ble_isoc_anchor);
}

static int btmtk_usb_close(struct hci_dev *hdev)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	int err;

	BTMTK_INFO("%s enter!", __func__);

	BTMTK_DBG("%s", hdev->name);

	cancel_work_sync(&bdev->work);
	cancel_work_sync(&bdev->waker);
	cancel_work_sync(&bdev->reset_waker);

	clear_bit(BTUSB_BLE_ISOC_RUNNING, &bdev->flags);
	clear_bit(BTUSB_ISOC_RUNNING, &bdev->flags);
	clear_bit(BTUSB_BULK_RUNNING, &bdev->flags);
	clear_bit(BTUSB_INTR_RUNNING, &bdev->flags);

	btusb_stop_traffic(cif_dev);
	btusb_free_frags(bdev);

	err = usb_autopm_get_interface(cif_dev->intf);
	if (err < 0)
		goto failed;

	cif_dev->intf->needs_remote_wakeup = 0;
	usb_autopm_put_interface(cif_dev->intf);

failed:
	return 0;
}

/* Maybe will be used in the future*/
#if 0
static int btusb_flush(struct hci_dev *hdev)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);

	BTMTK_DBG("%s", hdev->name);

	usb_kill_anchored_urbs(&bdev->tx_anchor);
	btusb_free_frags(bdev);

	return 0;
}
#endif

static struct urb *alloc_intr_iso_urb(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct urb *urb;
	unsigned int pipe;

	if (!cif_dev->intr_iso_tx_ep)
		return ERR_PTR(-ENODEV);

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return ERR_PTR(-ENOMEM);

	pipe = usb_sndintpipe(cif_dev->udev, cif_dev->intr_iso_tx_ep->bEndpointAddress);

	usb_fill_int_urb(urb, cif_dev->udev, pipe,
			  skb->data, skb->len, btusb_tx_complete, skb, 1);

	skb->dev = (void *)hdev;

	return urb;
}

static struct urb *alloc_ctrl_bgf1_urb(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct usb_ctrlrequest *dr;
	struct urb *urb;
	unsigned int pipe;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return ERR_PTR(-ENOMEM);

	BTMTK_DBG("%s\n", __func__);
	dr = kmalloc(sizeof(*dr), GFP_KERNEL);
	if (!dr) {
		usb_free_urb(urb);
		return ERR_PTR(-ENOMEM);
	}

	dr->bRequestType = 0x21;
	dr->bRequest	 = 0x00;
	dr->wIndex	   = 3;
	dr->wValue	   = 0;
	dr->wLength	  = __cpu_to_le16(skb->len);

	pipe = usb_sndctrlpipe(cif_dev->udev, 0x00);

	usb_fill_control_urb(urb, cif_dev->udev, pipe, (void *)dr,
				 skb->data, skb->len, btusb_tx_complete, skb);

	skb->dev = (void *)hdev;

	return urb;
}

static struct urb *alloc_bulk_cmd_urb(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct urb *urb;
	unsigned int pipe;

	BTMTK_DBG("%s start\n", __func__);
	if (!cif_dev->bulk_cmd_tx_ep)
		return ERR_PTR(-ENODEV);

	BTMTK_DBG("%s\n", __func__);
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return ERR_PTR(-ENOMEM);

	pipe = usb_sndbulkpipe(cif_dev->udev, cif_dev->bulk_cmd_tx_ep->bEndpointAddress);

	usb_fill_bulk_urb(urb, cif_dev->udev, pipe,
			  skb->data, skb->len, btusb_tx_complete, skb);

	skb->dev = (void *)hdev;

	return urb;
}

static struct urb *alloc_ctrl_urb(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct usb_ctrlrequest *dr;
	struct urb *urb;
	unsigned int pipe;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return ERR_PTR(-ENOMEM);

	dr = kmalloc(sizeof(*dr), GFP_KERNEL);
	if (!dr) {
		usb_free_urb(urb);
		return ERR_PTR(-ENOMEM);
	}

	dr->bRequestType = cif_dev->cmdreq_type;
	dr->bRequest     = cif_dev->cmdreq;
	dr->wIndex       = 0;
	dr->wValue       = 0;
	dr->wLength      = __cpu_to_le16(skb->len);

	pipe = usb_sndctrlpipe(cif_dev->udev, 0x00);

	usb_fill_control_urb(urb, cif_dev->udev, pipe, (void *)dr,
			     skb->data, skb->len, btusb_tx_complete, skb);

	skb->dev = (void *)hdev;

	return urb;
}

static struct urb *alloc_bulk_urb(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct urb *urb;
	unsigned int pipe;

	if (!cif_dev->bulk_tx_ep)
		return ERR_PTR(-ENODEV);

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return ERR_PTR(-ENOMEM);

	pipe = usb_sndbulkpipe(cif_dev->udev, cif_dev->bulk_tx_ep->bEndpointAddress);

	usb_fill_bulk_urb(urb, cif_dev->udev, pipe,
			  skb->data, skb->len, btusb_tx_complete, skb);

	skb->dev = (void *)hdev;

	return urb;
}

static struct urb *alloc_isoc_urb(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct urb *urb;
	unsigned int pipe;

	if (!cif_dev->isoc_tx_ep)
		return ERR_PTR(-ENODEV);

	urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, GFP_KERNEL);
	if (!urb)
		return ERR_PTR(-ENOMEM);

	pipe = usb_sndisocpipe(cif_dev->udev, cif_dev->isoc_tx_ep->bEndpointAddress);

	usb_fill_int_urb(urb, cif_dev->udev, pipe,
			 skb->data, skb->len, btusb_isoc_tx_complete,
			 skb, cif_dev->isoc_tx_ep->bInterval);

	urb->transfer_flags  = URB_ISO_ASAP;

	__fill_isoc_descriptor(urb, skb->len,
			       le16_to_cpu(cif_dev->isoc_tx_ep->wMaxPacketSize));

	skb->dev = (void *)hdev;

	return urb;
}

static int submit_tx_urb(struct hci_dev *hdev, struct urb *urb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	int err;

	usb_anchor_urb(urb, &cif_dev->tx_anchor);

	err = usb_submit_urb(urb, GFP_KERNEL);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BTMTK_ERR("%s urb %p submission failed (%d)",
			       hdev->name, urb, -err);
		kfree(urb->setup_packet);
		usb_unanchor_urb(urb);
	} else {
		usb_mark_last_busy(cif_dev->udev);
	}

	usb_free_urb(urb);
	return err;
}

static int submit_or_queue_tx_urb(struct hci_dev *hdev, struct urb *urb)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	unsigned long flags;
	bool suspending;

	spin_lock_irqsave(&bdev->txlock, flags);
	suspending = test_bit(BTUSB_SUSPENDING, &bdev->flags);
	if (!suspending)
		bdev->tx_in_flight++;
	spin_unlock_irqrestore(&bdev->txlock, flags);

	if (!suspending)
		return submit_tx_urb(hdev, urb);

	schedule_work(&bdev->waker);

	usb_free_urb(urb);
	return 0;
}

static int btusb_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct urb *urb = NULL;
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	unsigned int ifnum_base;
	int ret = 0;
	struct sk_buff *iso_skb = NULL;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();
#ifdef CFG_SUPPORT_HW_DVT
	struct sk_buff *evt_skb;
	uint8_t notify_alt_evt[NOTIFY_ALT_EVT_LEN] = {0x04, 0x0E, 0x04, 0x01, 0x03, 0x0c, 0x00};
	u16 crBaseAddr = 0, crRegOffset = 0;
#endif

	if (skb->len <= 0) {
		ret = -EFAULT;
		BTMTK_ERR("%s: target packet length:%zu is not allowed", __func__, (size_t)skb->len);
	}

	ifnum_base = cif_dev->intf->cur_altsetting->desc.bInterfaceNumber;

	skb_pull(skb, 1);
	BTMTK_DBG_RAW(skb->data, skb->len, "%s, send_frame, type = %d", __func__,
		hci_skb_pkt_type(skb));
	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
#ifdef CFG_SUPPORT_HW_DVT
		if (skb->len > 7) {
			if (skb->data[0] == 0x6f && skb->data[1] == 0xfc &&
					skb->data[2] == 0x06 && skb->data[3] == 0x01 &&
					skb->data[4] == 0xff && skb->data[5] == 0x03 &&
					skb->data[6] == 0x00 && skb->data[7] == 0x00) {
				/* return evt to upper layered */
				evt_skb = skb_copy(skb, GFP_KERNEL);
				bt_cb(evt_skb)->pkt_type = notify_alt_evt[0];
				memcpy(evt_skb->data, &notify_alt_evt[1], NOTIFY_ALT_EVT_LEN - 1);
				evt_skb->len = NOTIFY_ALT_EVT_LEN - 1;
				/* After set alternate setting, we will return evt to boots */
				hci_recv_frame(hdev, evt_skb);
				hdev->conn_hash.sco_num++;
				bdev->sco_num = hdev->conn_hash.sco_num;
				cif_dev->new_isoc_altsetting = skb->data[8];
				BTMTK_INFO("alt_setting = %d, new_isoc_altsetting_interface = %d\n",
						cif_dev->new_isoc_altsetting, cif_dev->new_isoc_altsetting_interface);
				schedule_work(&bdev->work);
				msleep(20);
				kfree_skb(skb);
				skb = NULL;
				return 0;
			} else if (skb->data[0] == 0x6f && skb->data[1] == 0xfc &&
					skb->data[2] == 0x07 && skb->data[3] == 0x01 &&
					skb->data[4] == 0xff && skb->data[5] == 0x03 &&
					skb->data[6] == 0x00 && skb->data[7] == 0x00 && skb->data[9] == 0x00) {
				evt_skb = skb_copy(skb, GFP_KERNEL);
				bt_cb(evt_skb)->pkt_type = notify_alt_evt[0];
				memcpy(evt_skb->data, &notify_alt_evt[1], NOTIFY_ALT_EVT_LEN - 1);
				evt_skb->len = NOTIFY_ALT_EVT_LEN - 1;
				/* After set alternate setting, we will return evt to boots */
				hci_recv_frame(hdev, evt_skb);
				/* if sco_num == 0, btusb_work will set alternate setting to zero */
				hdev->conn_hash.sco_num--;
				bdev->sco_num = hdev->conn_hash.sco_num;
				cif_dev->new_isoc_altsetting_interface = skb->data[8];
				BTMTK_INFO("alt_setting to = %d, new_isoc_altsetting_interface = %d\n",
						cif_dev->new_isoc_altsetting, cif_dev->new_isoc_altsetting_interface);
				schedule_work(&bdev->work);
				/* If we don't sleep 50ms, it will failed to set alternate setting to zero */
				msleep(50);
				kfree_skb(skb);
				skb = NULL;
				return 0;
			} else if (skb->data[0] == 0x6f && skb->data[1] == 0xfc &&
					skb->data[2] == 0x09 && skb->data[3] == 0x01 &&
					skb->data[4] == 0xff && skb->data[5] == 0x05 &&
					skb->data[6] == 0x00 && skb->data[7] == 0x01) {
				BTMTK_INFO("read CR skb->data = %02x %02x %02x %02x\n", skb->data[8],
					skb->data[9], skb->data[10], skb->data[11]);
				crBaseAddr = (skb->data[8]<<8) + skb->data[9];
				crRegOffset = (skb->data[10]<<8) + skb->data[11];
				BTMTK_INFO("base + offset = %04x %04x\n", crBaseAddr, crRegOffset);
				memset(bdev->io_buf, 0, IO_BUF_SIZE);
				ret = usb_control_msg(cif_dev->udev, usb_rcvctrlpipe(cif_dev->udev, 0),
						1, 0xDE, crBaseAddr, crRegOffset,
						bdev->io_buf, 4, USB_CTRL_IO_TIMO);
				if (ret < 0)
					BTMTK_ERR("read CR(%04X[%04X]) FAILED\n", crBaseAddr, crRegOffset);
				else
					BTMTK_INFO("read CR(%04X[%04X]) value = 0x%02x%02x%02x%02x\n",
						crBaseAddr, crRegOffset,
						bdev->io_buf[3], bdev->io_buf[2],
						bdev->io_buf[1], bdev->io_buf[0]);
				kfree_skb(skb);
				skb = NULL;
				return 0;
			} else if (skb->data[0] == 0x6f && skb->data[1] == 0xfc &&
					skb->data[2] == 0x0D && skb->data[3] == 0x01 &&
					skb->data[4] == 0xff && skb->data[5] == 0x09 &&
					skb->data[6] == 0x00 && skb->data[7] == 0x02) {
				crBaseAddr = (skb->data[8] << 8) + skb->data[9];
				crRegOffset = (skb->data[10] << 8) + skb->data[11];
				BTMTK_INFO("base + offset = %04x %04x\n", crBaseAddr, crRegOffset);
				memset(cif_dev->o_usb_buf, 0, HCI_MAX_COMMAND_SIZE);
				cif_dev->o_usb_buf[0] = skb->data[12];
				cif_dev->o_usb_buf[1] = skb->data[13];
				cif_dev->o_usb_buf[2] = skb->data[14];
				cif_dev->o_usb_buf[3] = skb->data[15];
				ret = usb_control_msg(cif_dev->udev, usb_sndctrlpipe(cif_dev->udev, 0),
						2, 0x5E, crBaseAddr, crRegOffset,
						cif_dev->o_usb_buf, 4, USB_CTRL_IO_TIMO);
				if (ret < 0)
					BTMTK_ERR("write CR(%04X[%04X]) FAILED\n", crBaseAddr, crRegOffset);
				else
					BTMTK_INFO("write CR(%04X[%04X]) value = 0x%02x%02x%02x%02x\n",
						crBaseAddr, crRegOffset,
						cif_dev->o_usb_buf[3], cif_dev->o_usb_buf[2],
						cif_dev->o_usb_buf[1], cif_dev->o_usb_buf[0]);
				kfree_skb(skb);
				skb = NULL;
				return 0;
			}
		}
#endif

		/* For wmt cmd/evt */
		if (!memcmp(skb->data, &bmain_info->wmt_over_hci_header[1], WMT_OVER_HCI_HEADER_SIZE - 1)) {
			skb_push(skb, 1);
			skb->data[0] = MTK_HCI_COMMAND_PKT;
			BTMTK_DBG_RAW(skb->data, skb->len, "%s, 6ffc send_frame", __func__);
			btmtk_usb_send_cmd(bdev, skb, WMT_DELAY_TIMES, RETRY_TIMES, BTMTK_TX_CMD_FROM_DRV);
			btusb_submit_wmt_urb(hdev, GFP_KERNEL);
			return 0;
		}

		if (BTMTK_IS_BT_0_INTF(ifnum_base)) {
			if ((is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id)) &&
					cif_dev->bulk_cmd_tx_ep)
				urb = alloc_bulk_cmd_urb(hdev, skb);
			else
				urb = alloc_ctrl_urb(hdev, skb);
		} else if (BTMTK_IS_BT_1_INTF(ifnum_base)) {
			if (is_mt7922(bdev->chip_id) || is_mt7961(bdev->chip_id)) {
				if (cif_dev->bulk_cmd_tx_ep) {
					UNUSED(alloc_ctrl_bgf1_urb);
					urb = alloc_bulk_cmd_urb(hdev, skb);
				} else
					urb = alloc_ctrl_bgf1_urb(hdev, skb);
			} else if (is_mt7663(bdev->chip_id)) {
				urb = alloc_ctrl_urb(hdev, skb);
			} else {
				BTMTK_ERR("%s: chip_id(%d) is invalid", __func__, bdev->chip_id);
				return -ENODEV;
			}
		} else {
			BTMTK_ERR("%s: ifnum_base(%d) is invalid", __func__, ifnum_base);
			return -ENODEV;
		}

		if (IS_ERR(urb)) {
			kfree_skb(skb);
			skb = NULL;
			return PTR_ERR(urb);
		}

		hdev->stat.cmd_tx++;
		return submit_or_queue_tx_urb(hdev, urb);

	case HCI_ACLDATA_PKT:
		if (skb->data[0] == 0x00 && skb->data[1] == 0x44) {
			if (cif_dev->iso_channel && bdev->iso_threshold) {
				int isoc_pkt_len = 0;
				int isoc_pkt_padding = 0;

				skb_pull(skb, 4);
				isoc_pkt_len = skb->data[2] + (skb->data[3] << 8) + HCI_ISO_PKT_HEADER_SIZE;
				isoc_pkt_padding = bdev->iso_threshold - isoc_pkt_len;

				if (skb_tailroom(skb) < isoc_pkt_padding) {
					/* hci driver alllocate the size of skb that is to samll, need re-allocate */
					iso_skb = alloc_skb(HCI_MAX_ISO_SIZE + BT_SKB_RESERVE, GFP_ATOMIC);
					if (!iso_skb) {
						BTMTK_ERR("%s allocate skb failed!!", __func__);
						kfree_skb(skb);
						skb = NULL;
						return -ENOMEM;
					}
					/* copy skb data into iso_skb */
					skb_copy_bits(skb, 0, skb_put(iso_skb, skb->len), skb->len);
					memset(skb_put(iso_skb, isoc_pkt_padding), 0, isoc_pkt_padding);

					/* After call back, bt drive will free iso_skb */
					urb = alloc_intr_iso_urb(hdev, iso_skb);
					BTMTK_DBG_RAW(iso_skb->data, iso_skb->len, "%s, it's ble iso packet",
						__func__);
					/* It's alloc by hci drver, bt driver must be free it. */
					kfree_skb(skb);
					skb = NULL;
					if (IS_ERR(urb)) {
						kfree_skb(iso_skb);
						iso_skb = NULL;
						return PTR_ERR(urb);
					}
				} else {
					memset(skb_put(skb, isoc_pkt_padding), 0, isoc_pkt_padding);
					urb = alloc_intr_iso_urb(hdev, skb);
					BTMTK_DBG_RAW(skb->data, skb->len, "%s, it's ble iso packet",
						__func__);
					if (IS_ERR(urb)) {
						kfree_skb(skb);
						skb = NULL;
						return PTR_ERR(urb);
					}
				}
			} else {
				BTMTK_WARN("btusb_send_frame send iso data, but iso channel not exit");
				/* if iso channel not exist, we need to drop iso data then free the skb */
				kfree_skb(skb);
				skb = NULL;
				return 0;
			}
		} else {
			urb = alloc_bulk_urb(hdev, skb);
			if (IS_ERR(urb)) {
				kfree_skb(skb);
				skb = NULL;
				return PTR_ERR(urb);
			}
		}
		hdev->stat.acl_tx++;
		return submit_or_queue_tx_urb(hdev, urb);

	case HCI_SCODATA_PKT:
		if (hci_conn_num(hdev, SCO_LINK) < 1) {
			BTMTK_INFO("btusb_send_frame hci_conn sco link = %d", hci_conn_num(hdev, SCO_LINK));
			/* We need to study how to solve this in hw_dvt case.*/
#ifndef CFG_SUPPORT_HW_DVT
			kfree_skb(skb);
			skb = NULL;
			return -ENODEV;
#endif
		}

		urb = alloc_isoc_urb(hdev, skb);
		if (IS_ERR(urb)) {
			kfree_skb(skb);
			skb = NULL;
			return PTR_ERR(urb);
		}

		hdev->stat.sco_tx++;
		return submit_tx_urb(hdev, urb);
	}

	kfree_skb(skb);
	skb = NULL;
	return -EILSEQ;
}

static int btmtk_usb_load_fw_patch_using_dma(struct btmtk_dev *bdev, u8 *image,
		u8 *fwbuf, int section_dl_size, int section_offset)
{
	int cur_len = 0;
	int ret = 0;
	s32 sent_len;
	u8 dl_done_cmd[LD_PATCH_CMD_LEN] = {0x01, 0x6F, 0xFC, 0x05, 0x01, 0x01, 0x01, 0x00, PATCH_PHASE3};
	u8 event[LD_PATCH_EVT_LEN] = {0x04, 0xE4, 0x05, 0x02, 0x01, 0x01, 0x00, 0x00}; /* event[7] is status*/

	if (bdev == NULL || image == NULL || fwbuf == NULL) {
		BTMTK_ERR("%s: invalid parameters!", __func__);
		ret = -1;
		goto exit;
	}

	BTMTK_INFO("%s: loading rom patch... start", __func__);
	while (1) {
		sent_len = (section_dl_size - cur_len) >= (UPLOAD_PATCH_UNIT - HCI_TYPE_SIZE) ?
				(UPLOAD_PATCH_UNIT - HCI_TYPE_SIZE) : (section_dl_size - cur_len);

		if (sent_len > 0) {
			/* btmtk_cif_send_bulk_out will send from image[1],
			 * image[0] will be ingored
			 */
			image[0] = HCI_ACLDATA_PKT;
			memcpy(&image[HCI_TYPE_SIZE], fwbuf + section_offset + cur_len, sent_len);
			BTMTK_DBG("%s: sent_len = %d, cur_len = %d", __func__,
					sent_len, cur_len);
			ret = btmtk_main_send_cmd(bdev,
					image, sent_len + HCI_TYPE_SIZE,
					NULL, -1,
					0, 0, BTMTK_TX_ACL_FROM_DRV);
			if (ret < 0) {
				BTMTK_ERR("%s: send patch failed, terminate", __func__);
				goto exit;
			}
			cur_len += sent_len;
		} else
			break;
	}

	BTMTK_INFO_RAW(dl_done_cmd, LD_PATCH_CMD_LEN, "%s: send dl cmd - ", __func__);
	ret = btmtk_main_send_cmd(bdev, dl_done_cmd, LD_PATCH_CMD_LEN,
			event, LD_PATCH_EVT_LEN,
			DELAY_TIMES, RETRY_TIMES, BTMTK_TX_CMD_FROM_DRV);
	if (ret < 0)
		BTMTK_ERR("%s: send wmd dl cmd failed, terminate!", __func__);
	BTMTK_INFO("%s: loading rom patch... Done", __func__);

exit:
	return ret;
}

static void btusb_notify(struct hci_dev *hdev, unsigned int evt)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);

	BTMTK_DBG("%s evt %d", hdev->name, evt);

	if (hci_conn_num(hdev, SCO_LINK) != bdev->sco_num) {
		bdev->sco_num = hci_conn_num(hdev, SCO_LINK);
		schedule_work(&bdev->work);
	}
}

static inline int __set_isoc_interface(struct hci_dev *hdev, int altsetting)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct usb_interface *intf = cif_dev->isoc;
	struct usb_endpoint_descriptor *ep_desc;
	int i, err;
	unsigned int ifnum_base;

	if (!cif_dev->isoc)
		return -ENODEV;

	ifnum_base = cif_dev->intf->cur_altsetting->desc.bInterfaceNumber;
	if (BTMTK_IS_BT_0_INTF(ifnum_base))
		cif_dev->new_isoc_altsetting_interface = 1;
	else if (BTMTK_IS_BT_1_INTF(ifnum_base))
		cif_dev->new_isoc_altsetting_interface = 4;
	err = usb_set_interface(cif_dev->udev, cif_dev->new_isoc_altsetting_interface, altsetting);
	BTMTK_DBG("setting interface alt = %d, interface = %d", altsetting, cif_dev->new_isoc_altsetting_interface);

	if (err < 0) {
		BTMTK_ERR("%s setting interface failed (%d)", hdev->name, -err);
		return err;
	}

	bdev->isoc_altsetting = altsetting;

	cif_dev->isoc_tx_ep = NULL;
	cif_dev->isoc_rx_ep = NULL;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!cif_dev->isoc_tx_ep && usb_endpoint_is_isoc_out(ep_desc)) {
			cif_dev->isoc_tx_ep = ep_desc;
			continue;
		}

		if (!cif_dev->isoc_rx_ep && usb_endpoint_is_isoc_in(ep_desc)) {
			cif_dev->isoc_rx_ep = ep_desc;
			continue;
		}
	}

	if (!cif_dev->isoc_tx_ep || !cif_dev->isoc_rx_ep) {
		BTMTK_ERR("%s invalid SCO descriptors", hdev->name);
		return -ENODEV;
	}

	return 0;
}

static void btusb_work(struct work_struct *work)
{
	struct btmtk_dev *bdev = container_of(work, struct btmtk_dev, work);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct hci_dev *hdev = bdev->hdev;
	int new_alts;
	int err;
	unsigned long flags;

	if (bdev->sco_num > 0) {
		if (!test_bit(BTUSB_DID_ISO_RESUME, &bdev->flags)) {
			err = usb_autopm_get_interface(cif_dev->isoc ? cif_dev->isoc : cif_dev->intf);
			if (err < 0) {
				clear_bit(BTUSB_ISOC_RUNNING, &bdev->flags);
				usb_kill_anchored_urbs(&cif_dev->isoc_anchor);
				return;
			}

			set_bit(BTUSB_DID_ISO_RESUME, &bdev->flags);
		}

#ifdef CFG_SUPPORT_HW_DVT
		new_alts = cif_dev->new_isoc_altsetting;
#else
		if (hdev->voice_setting & 0x0020) {
			static const int alts[3] = { 2, 4, 5 };

			new_alts = alts[bdev->sco_num - 1];
		} else {
			new_alts = bdev->sco_num;
		}
#endif

		clear_bit(BTUSB_ISOC_RUNNING, &bdev->flags);
		usb_kill_anchored_urbs(&cif_dev->isoc_anchor);

		/* When isochronous alternate setting needs to be
		 * changed, because SCO connection has been added
		 * or removed, a packet fragment may be left in the
		 * reassembling state. This could lead to wrongly
		 * assembled fragments.
		 *
		 * Clear outstanding fragment when selecting a new
		 * alternate setting.
		 */
		spin_lock_irqsave(&bdev->rxlock, flags);
		kfree_skb(bdev->sco_skb);
		bdev->sco_skb = NULL;
		spin_unlock_irqrestore(&bdev->rxlock, flags);

		if (__set_isoc_interface(hdev, new_alts) < 0)
			return;

		if (!test_and_set_bit(BTUSB_ISOC_RUNNING, &bdev->flags)) {
			if (btusb_submit_isoc_urb(hdev, GFP_KERNEL) < 0)
				clear_bit(BTUSB_ISOC_RUNNING, &bdev->flags);
			else
				btusb_submit_isoc_urb(hdev, GFP_KERNEL);
		}
	} else {
		clear_bit(BTUSB_ISOC_RUNNING, &bdev->flags);
		usb_kill_anchored_urbs(&cif_dev->isoc_anchor);
		BTMTK_INFO("%s set alt to zero", __func__);
		__set_isoc_interface(hdev, 0);
		if (test_and_clear_bit(BTUSB_DID_ISO_RESUME, &bdev->flags))
			usb_autopm_put_interface(cif_dev->isoc ? cif_dev->isoc : cif_dev->intf);
	}
}

static void btusb_waker(struct work_struct *work)
{
	struct btmtk_dev *bdev = container_of(work, struct btmtk_dev, waker);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	int err;

	err = usb_autopm_get_interface(cif_dev->intf);
	if (err < 0)
		return;

	usb_autopm_put_interface(cif_dev->intf);
}

static int btmtk_usb_toggle_rst_pin(struct btmtk_dev *bdev)
{
	struct device_node *node;
	int rst_pin_num = 0;

	if (!bdev) {
		BTMTK_WARN("%s: bdev is NULL!", __func__);
		return -1;
	}
	if (bdev->bt_cfg.dongle_reset_gpio_pin == -1) {
		BTMTK_WARN("%s: bt driver is not ready, please don't call chip reset!", __func__);
		return -1;
	}

	BTMTK_INFO("%s: begin", __func__);

	bdev->chip_reset = 1;
	/* Initialize the interface specific function pointers */
	reset_func.pf_pdwndFunc = (pdwnc_func) btmtk_kallsyms_lookup_name("PDWNC_SetBTInResetState");
	if (reset_func.pf_pdwndFunc)
		BTMTK_INFO("%s: Found PDWNC_SetBTInResetState", __func__);
	else
		BTMTK_WARN("%s: No Exported Func Found PDWNC_SetBTInResetState", __func__);

	reset_func.pf_resetFunc2 = (reset_func_ptr2) btmtk_kallsyms_lookup_name("mtk_gpio_set_value");
	if (!reset_func.pf_resetFunc2)
		BTMTK_ERR("%s: No Exported Func Found mtk_gpio_set_value", __func__);
	else
		BTMTK_INFO("%s: Found mtk_gpio_set_value", __func__);

	reset_func.pf_lowFunc = (set_gpio_low) btmtk_kallsyms_lookup_name("MDrv_GPIO_Set_Low");
	reset_func.pf_highFunc = (set_gpio_high) btmtk_kallsyms_lookup_name("MDrv_GPIO_Set_High");
	if (!reset_func.pf_lowFunc || !reset_func.pf_highFunc)
		BTMTK_WARN("%s: No Exported Func Found MDrv_GPIO_Set_Low or High", __func__);
	else
		BTMTK_INFO("%s: Found MDrv_GPIO_Set_Low & MDrv_GPIO_Set_High", __func__);

	if (reset_func.pf_pdwndFunc) {
		BTMTK_INFO("%s: Invoke PDWNC_SetBTInResetState(%d)", __func__, 1);
		reset_func.pf_pdwndFunc(1);
	} else
		BTMTK_INFO("%s: No Exported Func Found PDWNC_SetBTInResetState", __func__);

	if (reset_func.pf_resetFunc2) {
		rst_pin_num = bdev->bt_cfg.dongle_reset_gpio_pin;
		BTMTK_INFO("%s: Invoke bdev->pf_resetFunc2(%d,%d)", __func__, rst_pin_num, 0);
		reset_func.pf_resetFunc2(rst_pin_num, 0);
		msleep(RESET_PIN_SET_LOW_TIME);
		BTMTK_INFO("%s: Invoke bdev->pf_resetFunc2(%d,%d)", __func__, rst_pin_num, 1);
		reset_func.pf_resetFunc2(rst_pin_num, 1);
		goto exit;
	}

	node = of_find_compatible_node(NULL, NULL, "mstar,gpio-wifi-ctl");
	if (node) {
		if (of_property_read_u32(node, "wifi-ctl-gpio", &rst_pin_num) == 0) {
			if (reset_func.pf_lowFunc && reset_func.pf_highFunc) {
				BTMTK_INFO("%s: Invoke bdev->pf_lowFunc(%d)", __func__, rst_pin_num);
				reset_func.pf_lowFunc(rst_pin_num);
				msleep(RESET_PIN_SET_LOW_TIME);
				BTMTK_INFO("%s: Invoke bdev->pf_highFunc(%d)", __func__, rst_pin_num);
				reset_func.pf_highFunc(rst_pin_num);
				goto exit;
			}
		} else
			BTMTK_WARN("%s, failed to obtain wifi control gpio\n", __func__);
	} else {
		if (reset_func.pf_lowFunc && reset_func.pf_highFunc) {
			rst_pin_num = bdev->bt_cfg.dongle_reset_gpio_pin;
			BTMTK_INFO("%s: Invoke bdev->pf_lowFunc(%d)", __func__, rst_pin_num);
			reset_func.pf_lowFunc(rst_pin_num);
			msleep(RESET_PIN_SET_LOW_TIME);
			BTMTK_INFO("%s: Invoke bdev->pf_highFunc(%d)", __func__, rst_pin_num);
			reset_func.pf_highFunc(rst_pin_num);
			goto exit;
		}
	}

	/* use linux kernel common api */
	do {
		struct device_node *node;
		int mt76xx_reset_gpio = bdev->bt_cfg.dongle_reset_gpio_pin;

		node = of_find_compatible_node(NULL, NULL, "mediatek,connectivity-combo");
		if (node) {
			mt76xx_reset_gpio = of_get_named_gpio(node, "mt76xx-reset-gpio", 0);
			if (gpio_is_valid(mt76xx_reset_gpio))
				BTMTK_INFO("%s: Get chip reset gpio(%d)", __func__, mt76xx_reset_gpio);
			else
				mt76xx_reset_gpio = bdev->bt_cfg.dongle_reset_gpio_pin;
		}

		BTMTK_INFO("%s: Invoke Low(%d)", __func__, mt76xx_reset_gpio);
		gpio_direction_output(mt76xx_reset_gpio, 0);
		msleep(RESET_PIN_SET_LOW_TIME);
		BTMTK_INFO("%s: Invoke High(%d)", __func__, mt76xx_reset_gpio);
		gpio_direction_output(mt76xx_reset_gpio, 1);
		goto exit;
	} while (0);

exit:
	BTMTK_INFO("%s: end", __func__);
	return 0;
}

static int btmtk_usb_subsys_reset(struct btmtk_dev *bdev)
{
	int val, retry = 10;

	cancel_work_sync(&bdev->work);
	cancel_work_sync(&bdev->waker);

	clear_bit(BTUSB_BLE_ISOC_RUNNING, &bdev->flags);
	clear_bit(BTUSB_ISOC_RUNNING, &bdev->flags);
	clear_bit(BTUSB_BULK_RUNNING, &bdev->flags);
	clear_bit(BTUSB_INTR_RUNNING, &bdev->flags);
	bdev->sco_num = 0;

	btusb_stop_traffic((struct btmtk_usb_dev *)bdev->cif_dev);

	/* For reset */
	btmtk_cif_write_uhw_register(bdev, EP_RST_OPT, EP_RST_IN_OUT_OPT);

	/* read interrupt EP15 CR */
	btmtk_cif_read_uhw_register(bdev, BT_WDT_STATUS, &val);

	/* Write Reset CR to 1 */
	btmtk_cif_write_uhw_register(bdev, BT_SUBSYS_RST, 1);

	btmtk_cif_write_uhw_register(bdev, UDMA_INT_STA_BT, 0x000000FF);
	btmtk_cif_read_uhw_register(bdev, UDMA_INT_STA_BT, &val);
	btmtk_cif_write_uhw_register(bdev, UDMA_INT_STA_BT1, 0x000000FF);
	btmtk_cif_read_uhw_register(bdev, UDMA_INT_STA_BT1, &val);

	/* Write Reset CR to 0 */
	btmtk_cif_write_uhw_register(bdev, BT_SUBSYS_RST, 0);

	/* Read reset CR */
	btmtk_cif_read_uhw_register(bdev, BT_SUBSYS_RST, &val);

	do {
		/* polling re-init CR */
		btmtk_cif_read_uhw_register(bdev, BT_MISC, &val);
		BTMTK_INFO("%s: reg=%x, value=0x%08x", __func__, BT_MISC, val);
		if ((val & 0x00000300) == 0x00000300) {
			/* L0.5 reset done */
			BTMTK_INFO("%s: Do L0.5 reset sucessfully.", __func__);
			goto Finish;
		} else {
			BTMTK_INFO("%s: polling MCU-init done CR", __func__);
		}
		msleep(100);
	} while (retry-- > 0);

	/* L0.5 reset failed, do whole chip reset */
	return -1;

Finish:
	return 0;
}

static int btmtk_usb_enter_standby(void)
{
	int ret = 0;
	int i = 0;
	int cif_event = 0;
	struct btmtk_cif_state *cif_state = NULL;
	struct btmtk_dev *bdev = NULL;
	struct btmtk_usb_dev *cif_dev = NULL;
	struct btmtk_woble *bt_woble = NULL;
	int intf_num = btmtk_get_interface_num();
	struct btmtk_dev **pp_bdev = btmtk_get_pp_bdev();

	BTMTK_INFO("%s: enter", __func__);
	for (i = 0; i < intf_num; i++) {
		/* Find valid dev for already probe interface. */
		if (pp_bdev[i]->hdev != NULL) {
			bdev = pp_bdev[i];
			cif_dev = (struct btmtk_usb_dev *)bdev;
			bt_woble = &cif_dev->bt_woble;

			/* Retrieve current HIF event state */
			cif_event = HIF_EVENT_STANDBY;
			if (BTMTK_CIF_IS_NULL(bdev, cif_event)) {
				/* Error */
				BTMTK_WARN("%s parameter is NULL", __func__);
				return -ENODEV;
			}

			cif_state = &bdev->cif_state[cif_event];

			btmtk_usb_cif_mutex_lock(bdev);
			/* Set Entering state */
			btmtk_set_chip_state((void *)bdev, cif_state->ops_enter);

			/* Do HIF events */
			ret = btmtk_woble_suspend(bt_woble);

			/* Set End/Error state */
			if (ret == 0)
				btmtk_set_chip_state((void *)bdev, cif_state->ops_end);
			else
				btmtk_set_chip_state((void *)bdev, cif_state->ops_error);

			btmtk_usb_cif_mutex_unlock(bdev);

			if (ret)
				break;
		}
	}

	BTMTK_INFO("%s: end", __func__);
	return ret;
}

static int btusb_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ep_desc;
	struct btmtk_dev *bdev = NULL;
	struct btmtk_usb_dev *cif_dev = NULL;
	unsigned int ifnum_base;
	int i, err = 0;

	ifnum_base = intf->cur_altsetting->desc.bInterfaceNumber;
	BTMTK_DBG("intf %p id %p, interfacenum = %d", intf, id, ifnum_base);

	bdev = usb_get_intfdata(intf);
	if (!bdev) {
		BTMTK_ERR("[ERR] bdev is NULL");
		err = -ENOMEM;
		goto end;
	}
	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		/* reset_intr_ep must be initialized before intr_ep,
		 * otherwise its address may be the intr_ep address
		 */
		if (!cif_dev->reset_intr_ep && ep_desc->bEndpointAddress == 0x8f &&
			usb_endpoint_is_int_in(ep_desc)) {
			BTMTK_INFO("intr_reset_rx__ep i = %d  Endpoints 0x%02X, number_of_endpoints=%d",
				i, ep_desc->bEndpointAddress, intf->cur_altsetting->desc.bNumEndpoints);
			cif_dev->reset_intr_ep = ep_desc;
			continue;
		}

		/* bulk_cmd_tx_ep must be initialized before bulk_tx_ep,
		 * otherwise its address will be the bulk_tx_ep address
		 */
		if (!cif_dev->bulk_cmd_tx_ep && usb_endpoint_is_bulk_out(ep_desc) &&
			(ep_desc->bEndpointAddress == 0x01 || ep_desc->bEndpointAddress == 0x0b)) {
			cif_dev->bulk_cmd_tx_ep = ep_desc;
			BTMTK_INFO(" bulk_cmd_tx_ep i = %d  Endpoints 0x%02X, number_of_endpoints=%d",
				i, ep_desc->bEndpointAddress, intf->cur_altsetting->desc.bNumEndpoints);
			continue;
		}

		if (!cif_dev->intr_ep && usb_endpoint_is_int_in(ep_desc)) {
			cif_dev->intr_ep = ep_desc;
			BTMTK_INFO("intr_rx_ep i = %d  Endpoints 0x%02X, number_of_endpoints=%d",
				i, ep_desc->bEndpointAddress, intf->cur_altsetting->desc.bNumEndpoints);
			continue;
		}

		if (!cif_dev->bulk_tx_ep && usb_endpoint_is_bulk_out(ep_desc)) {
			cif_dev->bulk_tx_ep = ep_desc;
			BTMTK_INFO("bulk_tx_ep i = %d  Endpoints 0x%02X, number_of_endpoints=%d",
				i, ep_desc->bEndpointAddress, intf->cur_altsetting->desc.bNumEndpoints);
			continue;
		}

		if (!cif_dev->bulk_rx_ep && usb_endpoint_is_bulk_in(ep_desc)) {
			cif_dev->bulk_rx_ep = ep_desc;
			BTMTK_INFO("bulk_rx_ep i = %d  Endpoints 0x%02X, number_of_endpoints=%d",
				i, ep_desc->bEndpointAddress, intf->cur_altsetting->desc.bNumEndpoints);
			continue;
		}
	}

	if (!cif_dev->intr_ep || !cif_dev->bulk_tx_ep || !cif_dev->bulk_rx_ep) {
		BTMTK_ERR("[ERR] intr_ep or bulk_tx_ep or bulk_rx_ep is NULL");
		err = -ENODEV;
		goto end;
	}

	cif_dev->cmdreq_type = USB_TYPE_CLASS;
	cif_dev->cmdreq = 0x00;


	cif_dev->udev = interface_to_usbdev(intf);
	cif_dev->intf = intf;
	bdev->intf_dev = &cif_dev->udev->dev;

	INIT_WORK(&bdev->work, btusb_work);
	INIT_WORK(&bdev->waker, btusb_waker);
	/* it's for L0/L0.5 reset */
	INIT_WORK(&bdev->reset_waker, btmtk_reset_waker);
	init_usb_anchor(&cif_dev->tx_anchor);
	spin_lock_init(&bdev->txlock);

	init_usb_anchor(&cif_dev->intr_anchor);
	init_usb_anchor(&cif_dev->bulk_anchor);
	init_usb_anchor(&cif_dev->isoc_anchor);
	init_usb_anchor(&cif_dev->ctrl_anchor);
	init_usb_anchor(&cif_dev->ble_isoc_anchor);
	spin_lock_init(&bdev->rxlock);

	err = btmtk_cif_allocate_memory(cif_dev);
	if (err < 0) {
		BTMTK_ERR("[ERR] btmtk_cif_allocate_memory failed!");
		goto end;
	}

	err = btmtk_main_cif_initialize(bdev, HCI_USB);
	if (err < 0) {
		BTMTK_ERR("[ERR] btmtk_main_cif_initialize failed!");
		goto free_mem;
	}

	/* only usb interface need this callback to allocate isoc trx endpoint
	 * There is no need for other interface such as sdio to use this function
	 */
	bdev->hdev->notify = btusb_notify;

	SET_HCIDEV_DEV(bdev->hdev, &cif_dev->intf->dev);

	if (BTMTK_IS_BT_0_INTF(ifnum_base))
		err = btmtk_load_rom_patch(bdev);
	else
		BTMTK_INFO("interface = %d, don't download patch", ifnum_base);

	if (err < 0) {
		BTMTK_ERR("btmtk load rom patch failed!");
		goto deinit1;
	}

	/* For reset */
	btmtk_cif_write_uhw_register(bdev, EP_RST_OPT, 0x00010001);

	/* Interface numbers are hardcoded in the specification */
	if (BTMTK_IS_BT_0_INTF(ifnum_base)) {
		cif_dev->isoc = usb_ifnum_to_if(cif_dev->udev, 1);

		BTMTK_INFO("set interface number 2 for iso ");
		cif_dev->iso_channel = usb_ifnum_to_if(cif_dev->udev, 2);
		usb_set_interface(cif_dev->udev, 2, 1);
		if (cif_dev->iso_channel) {
			for (i = 0; i < cif_dev->iso_channel->cur_altsetting->desc.bNumEndpoints; i++) {
				ep_desc = &cif_dev->iso_channel->cur_altsetting->endpoint[i].desc;

				if (!cif_dev->intr_iso_tx_ep && usb_endpoint_is_int_out(ep_desc)) {
					cif_dev->intr_iso_tx_ep = ep_desc;
					BTMTK_INFO("intr_iso_tx_ep i = %d\t"
						"Endpoints 0x%02X, number_of_endpoints=%d",
						i, ep_desc->bEndpointAddress,
						intf->cur_altsetting->desc.bNumEndpoints);
					continue;
				}

				if (!cif_dev->intr_iso_rx_ep && usb_endpoint_is_int_in(ep_desc)) {
					cif_dev->intr_iso_rx_ep = ep_desc;
					BTMTK_INFO("intr_iso_rx_ep i = %d\t"
						"Endpoints 0x%02X, number_of_endpoints=%d",
						i, ep_desc->bEndpointAddress,
						intf->cur_altsetting->desc.bNumEndpoints);
					continue;
				}
			}

			err = usb_driver_claim_interface(&btusb_driver,
							 cif_dev->iso_channel, bdev);
			if (err < 0)
				goto deinit1;
		}
	} else if (BTMTK_IS_BT_1_INTF(ifnum_base)) {
		BTMTK_INFO("interface number = 3, set interface number 4");
		cif_dev->isoc = usb_ifnum_to_if(cif_dev->udev, 4);
	}

	if (cif_dev->isoc) {
		err = usb_driver_claim_interface(&btusb_driver,
						 cif_dev->isoc, bdev);
		if (err < 0)
			goto deinit1;
	}

	/* dongle_index - 1 since BT1 is in same interface */
	if (BTMTK_IS_BT_1_INTF(ifnum_base))
		bdev->dongle_index--;
	BTMTK_DBG("%s: bdev->dongle_index = %d ", __func__, bdev->dongle_index);

	usb_set_intfdata(intf, bdev);

	err = btmtk_woble_initialize(bdev, &cif_dev->bt_woble);
	if (err < 0) {
		BTMTK_ERR("btmtk_woble_initialize failed!");
		goto deinit1;
	}

	btmtk_woble_wake_unlock(bdev);

#if CFG_SUPPORT_BLUEZ
	err = btmtk_send_init_cmds(bdev);
	if (err < 0) {
		BTMTK_ERR("%s, btmtk_send_init_cmds failed, err = %d", __func__, err);
		goto free_setting;
	}
#endif /* CFG_SUPPORT_BLUEZ */

	err = btmtk_register_hci_device(bdev);
	if (err < 0) {
		BTMTK_ERR("btmtk_register_hci_device failed!");
		goto free_setting;
	}

	return 0;

free_setting:
	btmtk_woble_uninitialize(&cif_dev->bt_woble);
deinit1:
	btmtk_main_cif_uninitialize(bdev, HCI_USB);
free_mem:
	btmtk_cif_free_memory(cif_dev);
end:
	return err;
}

static void btusb_disconnect(struct usb_interface *intf)
{
	struct btmtk_dev *bdev = NULL;
	struct btmtk_usb_dev *cif_dev = NULL;
	struct hci_dev *hdev;

	BTMTK_DBG("intf %p", intf);
	bdev = usb_get_intfdata(intf);
	if (!bdev) {
		BTMTK_WARN("%s: bdev is NULL!", __func__);
		return;
	}

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (!cif_dev) {
		BTMTK_WARN("%s: cif_dev is NULL!", __func__);
		return;
	}

	hdev = bdev->hdev;
	usb_set_intfdata(cif_dev->intf, NULL);

	if (cif_dev->isoc)
		usb_set_intfdata(cif_dev->isoc, NULL);

	if (cif_dev->iso_channel)
		usb_set_intfdata(cif_dev->iso_channel, NULL);

	if (intf == cif_dev->intf) {
		if (cif_dev->isoc)
			usb_driver_release_interface(&btusb_driver, cif_dev->isoc);
		if (cif_dev->iso_channel)
			usb_driver_release_interface(&btusb_driver, cif_dev->iso_channel);
	} else if (intf == cif_dev->isoc) {
		usb_driver_release_interface(&btusb_driver, cif_dev->intf);
	} else if (intf == cif_dev->iso_channel) {
		usb_driver_release_interface(&btusb_driver, cif_dev->intf);
	}

	btmtk_woble_uninitialize(&cif_dev->bt_woble);
	btmtk_cif_free_memory(cif_dev);

	btmtk_main_cif_disconnect_notify(bdev, HCI_USB);
}

#ifdef CONFIG_PM
static int btusb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct btmtk_dev *bdev = usb_get_intfdata(intf);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct btmtk_woble *bt_woble = &cif_dev->bt_woble;
	int ret = 0;

	BTMTK_DBG("intf %p", intf);

	if (bdev->suspend_count++) {
		BTMTK_WARN("%s: Has suspended. suspend_count: %d end", __func__, bdev->suspend_count);
		return 0;
	}

#if CFG_SUPPORT_DVT
	BTMTK_INFO("%s: SKIP Driver woble_suspend flow", __func__);
#else
	ret = btmtk_woble_suspend(bt_woble);
	if (ret < 0)
		BTMTK_ERR("%s: btmtk_woble_suspend return fail %d", __func__, ret);
#endif

	spin_lock_irq(&bdev->txlock);
	if (!(PMSG_IS_AUTO(message) && bdev->tx_in_flight)) {
		set_bit(BTUSB_SUSPENDING, &bdev->flags);
		spin_unlock_irq(&bdev->txlock);
	} else {
		spin_unlock_irq(&bdev->txlock);
		bdev->suspend_count--;
		return -EBUSY;
	}

	cancel_work_sync(&bdev->work);

	btusb_stop_traffic(cif_dev);
	usb_kill_anchored_urbs(&cif_dev->tx_anchor);

	BTMTK_INFO("%s end, suspend_count = %d", __func__, bdev->suspend_count);

	return ret;
}

static int btusb_resume(struct usb_interface *intf)
{
	struct btmtk_dev *bdev = usb_get_intfdata(intf);
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	struct hci_dev *hdev = bdev->hdev;
	struct btmtk_woble *bt_woble = &cif_dev->bt_woble;
	int err = 0;
	unsigned int ifnum_base = intf->cur_altsetting->desc.bInterfaceNumber;

	BTMTK_INFO("%s begin", __func__);

	if (--bdev->suspend_count) {
		BTMTK_WARN("%s: bdev->suspend_count %d, return 0", __func__,
				bdev->suspend_count);
		return 0;
	}

	/* need to remove it when BT off, need support woble case*/
	/* if (!test_bit(HCI_RUNNING, &hdev->flags)) {
	 * BTMTK_WARN("%s: hdev flags is not hci running. return", __func__);
	 * goto done;
	 * }
	 */

	/* when BT off, BTUSB_INTR_RUNNING will be clear,
	 * so we need to start traffic in btmtk_woble_resume when BT off
	 */
	if (test_bit(BTUSB_INTR_RUNNING, &bdev->flags)) {
		err = btusb_submit_intr_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_INTR_RUNNING, &bdev->flags);
			goto done;
		}

		if (is_mt7961(bdev->chip_id) && BTMTK_IS_BT_0_INTF(ifnum_base)) {
			BTMTK_INFO("%s 7961 submit urb\n", __func__);
			if (cif_dev->reset_intr_ep) {
				err = btusb_submit_intr_reset_urb(hdev, GFP_KERNEL);
				if (err < 0) {
					clear_bit(BTUSB_INTR_RUNNING, &bdev->flags);
					goto done;
				}
			} else
				BTMTK_INFO("%s, reset_intr_ep missing, don't submit_intr_reset_urb!",
					__func__);

			if (cif_dev->intr_iso_rx_ep) {
				err = btusb_submit_intr_ble_isoc_urb(hdev, GFP_KERNEL);
				if (err < 0) {
					usb_kill_anchored_urbs(&cif_dev->ble_isoc_anchor);
					clear_bit(BTUSB_INTR_RUNNING, &bdev->flags);
					goto done;
				}
			} else
				BTMTK_INFO("%s, intr_iso_rx_ep missing, don't submit_intr_ble_isoc_urb!",
					__func__);
		}
	}

	if (test_bit(BTUSB_BULK_RUNNING, &bdev->flags)) {
		err = btusb_submit_bulk_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_BULK_RUNNING, &bdev->flags);
			goto done;
		}

		btusb_submit_bulk_urb(hdev, GFP_NOIO);
	}

	if (test_bit(BTUSB_ISOC_RUNNING, &bdev->flags)) {
		if (btusb_submit_isoc_urb(hdev, GFP_NOIO) < 0)
			clear_bit(BTUSB_ISOC_RUNNING, &bdev->flags);
		else
			btusb_submit_isoc_urb(hdev, GFP_NOIO);
	}

	if (test_bit(BTUSB_BLE_ISOC_RUNNING, &bdev->flags)) {
		if (btusb_submit_intr_ble_isoc_urb(hdev, GFP_NOIO) < 0)
			clear_bit(BTUSB_BLE_ISOC_RUNNING, &bdev->flags);
		else
			btusb_submit_intr_ble_isoc_urb(hdev, GFP_NOIO);
	}

	spin_lock_irq(&bdev->txlock);
	clear_bit(BTUSB_SUSPENDING, &bdev->flags);
	spin_unlock_irq(&bdev->txlock);
	schedule_work(&bdev->work);

#if CFG_SUPPORT_DVT
	BTMTK_INFO("%s: SKIP Driver woble_resume flow", __func__);
#else
	err = btmtk_woble_resume(bt_woble);
	if (err < 0) {
		BTMTK_ERR("%s: btmtk_woble_resume return fail %d", __func__, err);
		goto done;
	}
#endif

	BTMTK_INFO("%s end", __func__);

	return 0;

done:
	spin_lock_irq(&bdev->txlock);
	clear_bit(BTUSB_SUSPENDING, &bdev->flags);
	spin_unlock_irq(&bdev->txlock);

	return err;
}
#endif

static int btmtk_cif_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	int ret = -1;
	int cif_event = 0;
	unsigned int ifnum_base;
	struct btmtk_cif_state *cif_state = NULL;
	struct btmtk_dev *bdev = NULL;

	/* Mediatek Driver Version */
	BTMTK_INFO("%s: MTK BT Driver Version : %s", __func__, VERSION);

	/* USB interface only.
	 * USB will need to identify thru descriptor's interface numbering.
	 */
	ifnum_base = intf->cur_altsetting->desc.bInterfaceNumber;
	BTMTK_DBG("intf %p id %p, interfacenum = %d", intf, id, ifnum_base);

	/* interface numbers are hardcoded in the spec */
	if (ifnum_base != BT0_MCU_INTERFACE_NUM &&
		ifnum_base != BT1_MCU_INTERFACE_NUM)
		return -ENODEV;

	/* Retrieve priv data and set to interface structure */
	bdev = btmtk_get_dev();
	usb_set_intfdata(intf, bdev);
	bdev->cif_dev = &g_usb_dev[bdev->dongle_index][intf_to_idx[ifnum_base]];

	/* Retrieve current HIF event state */
	cif_event = HIF_EVENT_PROBE;
	if (BTMTK_CIF_IS_NULL(bdev, cif_event)) {
		/* Error */
		BTMTK_WARN("%s intf[%d] priv setting is NULL", __func__, ifnum_base);
		return -ENODEV;
	}

	cif_state = &bdev->cif_state[cif_event];

	/* Set Entering state */
	btmtk_set_chip_state((void *)bdev, cif_state->ops_enter);

	/* Do HIF events */
	ret = btusb_probe(intf, id);

	/* Set End/Error state */
	if (ret == 0)
		btmtk_set_chip_state((void *)bdev, cif_state->ops_end);
	else
		btmtk_set_chip_state((void *)bdev, cif_state->ops_error);

	return ret;
}

static void btmtk_cif_disconnect(struct usb_interface *intf)
{
	int cif_event = 0;
	unsigned int ifnum_base;
	struct btmtk_cif_state *cif_state = NULL;
	struct btmtk_dev *bdev = NULL;
	struct btmtk_usb_dev *cif_dev = NULL;

	BTMTK_CIF_GET_DEV_PRIV(bdev, intf, ifnum_base);

	/* Retrieve current HIF event state */
	cif_event = HIF_EVENT_DISCONNECT;
	if (BTMTK_CIF_IS_NULL(bdev, cif_event)) {
		/* Error */
		BTMTK_WARN("%s intf[%d] priv setting is NULL", __func__, ifnum_base);
		return;
	}

	cif_state = &bdev->cif_state[cif_event];

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (!cif_dev) {
		BTMTK_WARN("%s: cif_dev is NULL!", __func__);
		return;
	}

	clear_bit(BTUSB_INTR_RUNNING, &bdev->flags);
	clear_bit(BTUSB_BULK_RUNNING, &bdev->flags);
	clear_bit(BTUSB_ISOC_RUNNING, &bdev->flags);
	clear_bit(BTUSB_BLE_ISOC_RUNNING, &bdev->flags);

	btusb_stop_traffic(cif_dev);

	btmtk_usb_cif_mutex_lock(bdev);
	/* Set Entering state */
	btmtk_set_chip_state((void *)bdev, cif_state->ops_enter);

	/* Do HIF events */
	btusb_disconnect(intf);

	/* Set End/Error state */
	btmtk_set_chip_state((void *)bdev, cif_state->ops_end);
	btmtk_usb_cif_mutex_unlock(bdev);
}

#ifdef CONFIG_PM
static int btmtk_cif_suspend(struct usb_interface *intf, pm_message_t message)
{
	int ret = 0;
	unsigned int ifnum_base;
	int cif_event = 0;
	struct btmtk_cif_state *cif_state = NULL;
	struct btmtk_dev *bdev = NULL;
	int state = BTMTK_STATE_INIT;

	BTMTK_INFO("%s, enter", __func__);
	BTMTK_CIF_GET_DEV_PRIV(bdev, intf, ifnum_base);

	state = btmtk_get_chip_state(bdev);
	/* Retrieve current HIF event state */
	if (state == BTMTK_STATE_FW_DUMP) {
		BTMTK_WARN("%s: FW dumping ongoing, don't dos suspend flow!!!", __func__);
		cif_event = HIF_EVENT_FW_DUMP;
	} else
		cif_event = HIF_EVENT_SUSPEND;

	if (BTMTK_IS_BT_0_INTF(ifnum_base) || BTMTK_IS_BT_1_INTF(ifnum_base)) {
		if (BTMTK_CIF_IS_NULL(bdev, cif_event)) {
			/* Error */
			BTMTK_WARN("%s intf[%d] priv setting is NULL", __func__, ifnum_base);
			return -ENODEV;
		}

		cif_state = &bdev->cif_state[cif_event];

		/* Set Entering state */
		btmtk_set_chip_state((void *)bdev, cif_state->ops_enter);

		/* Do HIF events */
		ret = btusb_suspend(intf, message);

		/* Set End/Error state */
		if (ret == 0)
			btmtk_set_chip_state((void *)bdev, cif_state->ops_end);
		else
			btmtk_set_chip_state((void *)bdev, cif_state->ops_error);
	} else
		BTMTK_INFO("%s, interface num is for isoc interface, do't do suspend!", __func__);

	BTMTK_INFO("%s, end. ret = %d", __func__, ret);
	return ret;
}

static int btmtk_cif_resume(struct usb_interface *intf)
{
	int ret = 0;
	unsigned int ifnum_base;
	int cif_event = 0;
	struct btmtk_cif_state *cif_state = NULL;
	struct btmtk_dev *bdev = NULL;

	BTMTK_INFO("%s, enter", __func__);
	BTMTK_CIF_GET_DEV_PRIV(bdev, intf, ifnum_base);

	if (BTMTK_IS_BT_0_INTF(ifnum_base) || BTMTK_IS_BT_1_INTF(ifnum_base)) {
		/* Retrieve current HIF event state */
		cif_event = HIF_EVENT_RESUME;
		if (BTMTK_CIF_IS_NULL(bdev, cif_event)) {
			/* Error */
			BTMTK_WARN("%s intf[%d] priv setting is NULL", __func__, ifnum_base);
			return -ENODEV;
		}

		cif_state = &bdev->cif_state[cif_event];

		/* Set Entering state */
		btmtk_set_chip_state((void *)bdev, cif_state->ops_enter);

		/* Do HIF events */
		ret = btusb_resume(intf);

		/* Set End/Error state */
		if (ret == 0)
			btmtk_set_chip_state((void *)bdev, cif_state->ops_end);
		else
			btmtk_set_chip_state((void *)bdev, cif_state->ops_error);
	} else
		BTMTK_INFO("%s, interface num is for isoc interface, do't do resume!", __func__);

	BTMTK_INFO("%s, end. ret = %d", __func__, ret);
	return ret;
}
#endif	/* CONFIG_PM */

#if !BT_DISABLE_RESET_RESUME
static int btmtk_cif_reset_resume(struct usb_interface *intf)
{
	BTMTK_INFO("%s: Call resume directly", __func__);
	return btmtk_cif_resume(intf);
}
#endif

static struct usb_driver btusb_driver = {
	.name		= "btusb",
	.probe		= btmtk_cif_probe,
	.disconnect	= btmtk_cif_disconnect,
#ifdef CONFIG_PM
	.suspend	= btmtk_cif_suspend,
	.resume		= btmtk_cif_resume,
#endif
#if !BT_DISABLE_RESET_RESUME
	.reset_resume = btmtk_cif_reset_resume,
#endif
	.id_table	= btusb_table,
	.supports_autosuspend = 1,
	.disable_hub_initiated_lpm = 1,
};

int btmtk_cif_register(void)
{
	int retval = 0;
	struct hif_hook_ptr hook;

	BTMTK_INFO("%s", __func__);

	/* register system power off callback function. */
	do {
		typedef void (*func_ptr) (int (*f) (void));
		char *func_name = "RegisterPdwncCallback";
		func_ptr pFunc = (func_ptr) btmtk_kallsyms_lookup_name(func_name);

		if (pFunc) {
			BTMTK_INFO("%s: Register Pdwnc callback success.", __func__);
			pFunc(&btmtk_usb_enter_standby);
		} else
			BTMTK_WARN("%s: No Exported Func Found [%s], just skip!", __func__, func_name);
	} while (0);

	memset(&hook, 0, sizeof(struct hif_hook_ptr));
	hook.open = btmtk_usb_open;
	hook.close = btmtk_usb_close;
	hook.reg_read = btmtk_usb_read_register;
	hook.reg_write = btmtk_usb_write_register;
	hook.send_cmd = btmtk_usb_send_cmd;
	hook.send_and_recv = btmtk_usb_send_and_recv;
	hook.event_filter = btmtk_usb_event_filter;
	hook.subsys_reset = btmtk_usb_subsys_reset;
	hook.whole_reset = btmtk_usb_toggle_rst_pin;
	hook.chip_reset_notify = btmtk_usb_chip_reset_notify;
	hook.cif_mutex_lock = btmtk_usb_cif_mutex_lock;
	hook.cif_mutex_unlock = btmtk_usb_cif_mutex_unlock;
	hook.dl_dma = btmtk_usb_load_fw_patch_using_dma;
	btmtk_reg_hif_hook(&hook);

	retval = usb_register(&btusb_driver);
	if (retval)
		BTMTK_ERR("*** USB registration fail(%d)! ***", retval);
	else
		BTMTK_INFO("%s, usb registration success!", __func__);
	return retval;
}

int btmtk_cif_deregister(void)
{
	BTMTK_INFO("%s", __func__);
	usb_deregister(&btusb_driver);
	BTMTK_INFO("%s: Done", __func__);
	return 0;
}

static int btmtk_cif_allocate_memory(struct btmtk_usb_dev *cif_dev)
{
	int err = -1;

	BTMTK_INFO("%s Begin", __func__);

	if (cif_dev->o_usb_buf == NULL) {
		cif_dev->o_usb_buf = kzalloc(HCI_MAX_COMMAND_SIZE, GFP_KERNEL);
		if (!cif_dev->o_usb_buf) {
			BTMTK_ERR("%s: alloc memory fail (bdev->o_usb_buf)", __func__);
			goto end;
		}
	}

	if (cif_dev->urb_intr_buf == NULL) {
		cif_dev->urb_intr_buf = kzalloc(URB_MAX_BUFFER_SIZE, GFP_KERNEL);
		if (!cif_dev->urb_intr_buf) {
			BTMTK_ERR("%s: alloc memory fail (bdev->urb_intr_buf)", __func__);
			goto err2;
		}
	}
	if (cif_dev->urb_bulk_buf == NULL) {
		cif_dev->urb_bulk_buf = kzalloc(URB_MAX_BUFFER_SIZE, GFP_KERNEL);
		if (!cif_dev->urb_bulk_buf) {
			BTMTK_ERR("%s: alloc memory fail (bdev->urb_bulk_buf)", __func__);
			goto err1;
		}
	}
	if (cif_dev->urb_ble_isoc_buf == NULL) {
		cif_dev->urb_ble_isoc_buf = kzalloc(URB_MAX_BUFFER_SIZE, GFP_KERNEL);
		if (!cif_dev->urb_ble_isoc_buf) {
			BTMTK_ERR("%s: alloc memory fail (bdev->urb_ble_isoc_buf)", __func__);
			goto err0;
		}
	}

	BTMTK_INFO("%s End", __func__);
	return 0;

err0:
	kfree(cif_dev->urb_bulk_buf);
	cif_dev->urb_bulk_buf = NULL;
err1:
	kfree(cif_dev->urb_intr_buf);
	cif_dev->urb_intr_buf = NULL;
err2:
	kfree(cif_dev->o_usb_buf);
	cif_dev->o_usb_buf = NULL;
end:
	return err;
}

static void btmtk_cif_free_memory(struct btmtk_usb_dev *cif_dev)
{
	if (!cif_dev) {
		BTMTK_ERR("%s: bdev is NULL!", __func__);
		return;
	}

	kfree(cif_dev->o_usb_buf);
	cif_dev->o_usb_buf = NULL;

	kfree(cif_dev->urb_intr_buf);
	cif_dev->urb_intr_buf = NULL;

	kfree(cif_dev->urb_bulk_buf);
	cif_dev->urb_bulk_buf = NULL;

	kfree(cif_dev->urb_ble_isoc_buf);
	cif_dev->urb_ble_isoc_buf = NULL;

	memset(cif_dev, 0, sizeof(struct btmtk_usb_dev));

	BTMTK_INFO("%s: Success", __func__);
}

static int btmtk_cif_write_uhw_register(struct btmtk_dev *bdev, u32 reg, u32 val)
{
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	int ret = -1;
	__le16 reg_high;
	__le16 reg_low;
	u8 reset_buf[4];

	reg_high = ((reg >> 16) & 0xffff);
	reg_low = (reg & 0xffff);

	reset_buf[0] = (val & 0x00ff);
	reset_buf[1] = ((val >> 8) & 0x00ff);
	reset_buf[2] = ((val >> 16) & 0x00ff);
	reset_buf[3] = ((val >> 24) & 0x00ff);

	memcpy(cif_dev->o_usb_buf, reset_buf, sizeof(reset_buf));
	ret = usb_control_msg(cif_dev->udev, usb_sndctrlpipe(cif_dev->udev, 0),
			0x02,						/* bRequest */
			0x5E,						/* bRequestType */
			reg_high,					/* wValue */
			reg_low,					/* wIndex */
			cif_dev->o_usb_buf,
			sizeof(reset_buf), USB_CTRL_IO_TIMO);

	BTMTK_DBG("%s: high=%x, reg_low=%x, val=%x", __func__, reg_high, reg_low, val);
	BTMTK_DBG("%s: reset_buf = %x %x %x %x", __func__, reset_buf[3], reset_buf[2], reset_buf[1], reset_buf[0]);

	if (ret < 0) {
		val = 0xffffffff;
		BTMTK_ERR("%s: error(%d), reg=%x, value=%x", __func__, ret, reg, val);
		return ret;
	}
	return 0;
}

static int btmtk_cif_read_uhw_register(struct btmtk_dev *bdev, u32 reg, u32 *val)
{
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	int ret = -1;
	__le16 reg_high;
	__le16 reg_low;

	reg_high = ((reg >> 16) & 0xffff);
	reg_low = (reg & 0xffff);

	memset(bdev->io_buf, 0, IO_BUF_SIZE);
	ret = usb_control_msg(cif_dev->udev, usb_rcvctrlpipe(cif_dev->udev, 0),
			0x01,						/* bRequest */
			0xDE,						/* bRequestType */
			reg_high,					/* wValue */
			reg_low,					/* wIndex */
			bdev->io_buf,
			4, USB_CTRL_IO_TIMO);

	if (ret < 0) {
		*val = 0xffffffff;
		BTMTK_ERR("%s: error(%d), reg=%x, value=0x%08x", __func__, ret, reg, *val);
		return ret;
	}

	memmove(val, bdev->io_buf, sizeof(u32));
	*val = le32_to_cpu(*val);

	BTMTK_DBG("%s: reg=%x, value=0x%08x", __func__, reg, *val);

	return 0;
}

static int btmtk_usb_read_register(struct btmtk_dev *bdev, u32 reg, u32 *val)
{
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	int ret = -1;
	__le16 reg_high;
	__le16 reg_low;

	reg_high = ((reg >> 16) & 0xffff);
	reg_low = (reg & 0xffff);

	memset(bdev->io_buf, 0, IO_BUF_SIZE);
	ret = usb_control_msg(cif_dev->udev, usb_rcvctrlpipe(cif_dev->udev, 0),
			0x63,						/* bRequest */
			DEVICE_VENDOR_REQUEST_IN,	/* bRequestType */
			reg_high,					/* wValue */
			reg_low,					/* wIndex */
			bdev->io_buf,
			sizeof(u32), USB_CTRL_IO_TIMO);

	if (ret < 0) {
		*val = 0xffffffff;
		BTMTK_ERR("%s: error(%d), reg=%x, value=%x", __func__, ret, reg, *val);
		return ret;
	}

	memmove(val, bdev->io_buf, sizeof(u32));
	*val = le32_to_cpu(*val);

	return 0;
}

static int btmtk_usb_write_register(struct btmtk_dev *bdev, u32 reg, u32 val)
{
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	int ret = -1;
	__le16 reg_high;
	__le16 reg_low;
	u8 buf[4];

	reg_high = ((reg >> 16) & 0xffff);
	reg_low = (reg & 0xffff);

	buf[0] = 0;
	buf[1] = 0;
	buf[2] = (val & 0x00ff);
	buf[3] = ((val >> 8) & 0x00ff);

	memcpy(cif_dev->o_usb_buf, buf, sizeof(buf));
	ret = usb_control_msg(cif_dev->udev, usb_sndctrlpipe(cif_dev->udev, 0),
			0x66,						/* bRequest */
			0x40,	/* bRequestType */
			reg_high,					/* wValue */
			reg_low,					/* wIndex */
			cif_dev->o_usb_buf,
			sizeof(buf), USB_CTRL_IO_TIMO);

	BTMTK_DBG("%s: buf = %x %x %x %x", __func__, buf[3], buf[2], buf[1], buf[0]);

	if (ret < 0) {
		val = 0xffffffff;
		BTMTK_ERR("%s: error(%d), reg=%x, value=%x", __func__, ret, reg, val);
		return ret;
	}

	return 0;
}

static void btmtk_cif_load_rom_patch_complete(struct urb *urb)
{
	struct completion *sent_to_mcu_done = (struct completion *)urb->context;

	complete(sent_to_mcu_done);
}

int btmtk_cif_send_control_out(struct btmtk_dev *bdev, struct sk_buff *skb, int delay, int retry)
{
	struct btmtk_usb_dev *cif_dev = NULL;
	int ret = 0;
	unsigned int ifnum_base;

	if (bdev == NULL || bdev->hdev == NULL || bdev->io_buf == NULL || skb == NULL ||
		skb->len > HCI_MAX_COMMAND_SIZE || skb->len <= 0) {
		BTMTK_ERR("%s: incorrect parameter", __func__);
		ret = -1;
		goto exit;
	}

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (cif_dev->udev == NULL || cif_dev->o_usb_buf == NULL) {
		BTMTK_ERR("%s: cif_dev is invalid", __func__);
		ret = -1;
		goto exit;
	}

	ifnum_base = cif_dev->intf->cur_altsetting->desc.bInterfaceNumber;

	/* send wmt command */
	memcpy(cif_dev->o_usb_buf, skb->data + 1, skb->len - 1);
	BTMTK_INFO_RAW(skb->data + 1, skb->len - 1, "%s: cmd:", __func__);
	if (BTMTK_IS_BT_0_INTF(ifnum_base))
		ret = usb_control_msg(cif_dev->udev, usb_sndctrlpipe(cif_dev->udev, 0),
				0x01, DEVICE_CLASS_REQUEST_OUT, 0x30, 0x00, (void *)cif_dev->o_usb_buf,
				skb->len - 1, USB_CTRL_IO_TIMO);
	else if (BTMTK_IS_BT_1_INTF(ifnum_base))
		ret = usb_control_msg(cif_dev->udev, usb_sndctrlpipe(cif_dev->udev, 0),
				0x00, 0x21, 0x00, 0x03, (void *)cif_dev->o_usb_buf, skb->len - 1, USB_CTRL_IO_TIMO);

	if (ret < 0) {
		BTMTK_ERR("%s: command send failed(%d)", __func__, ret);
		goto exit;
	}
exit:
	kfree_skb(skb);
	skb = NULL;
	return ret;
}

static int btmtk_cif_send_bulk_out(struct btmtk_dev *bdev, struct sk_buff *skb)
{
	int ret = 0;
	struct urb *urb;
	unsigned int pipe;
	struct completion sent_to_mcu_done;
	void *buf;
	struct btmtk_usb_dev *cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		ret = -ENOMEM;
		goto exit;
	}

	/* why need to alloc dma buffer??*/
	buf = usb_alloc_coherent(cif_dev->udev, UPLOAD_PATCH_UNIT, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		ret = -ENOMEM;
		goto exit;
	}
	init_completion(&sent_to_mcu_done);

	pipe = usb_sndbulkpipe(cif_dev->udev, cif_dev->bulk_tx_ep->bEndpointAddress);

	memcpy(buf, skb->data + 1, skb->len - 1);
	usb_fill_bulk_urb(urb,
			cif_dev->udev,
			pipe,
			buf,
			skb->len - 1,
			(usb_complete_t)btmtk_cif_load_rom_patch_complete,
			&sent_to_mcu_done);

	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret < 0) {
		BTMTK_ERR("%s: submit urb failed (%d)", __func__, ret);
		goto error;
	}

	if (!wait_for_completion_timeout
			(&sent_to_mcu_done, msecs_to_jiffies(1000))) {
		usb_kill_urb(urb);
		BTMTK_ERR("%s: upload rom_patch timeout", __func__);
		ret = -ETIME;
		goto error;
	}

error:
	usb_free_coherent(cif_dev->udev, UPLOAD_PATCH_UNIT, buf, urb->transfer_dma);
exit:
	usb_free_urb(urb);
	kfree_skb(skb);
	skb = NULL;
	return ret;
}

int btmtk_usb_send_cmd(struct btmtk_dev *bdev, struct sk_buff *skb,
		int delay, int retry, int pkt_type)
{
	int ret = -1;

	if (pkt_type == BTMTK_TX_CMD_FROM_DRV) {
		/* handle wmt cmd from driver */
		ret = btmtk_cif_send_control_out(bdev, skb, delay, retry);
	} else if (pkt_type == BTMTK_TX_ACL_FROM_DRV) {
		/* bulk out for load rom patch*/
		ret = btmtk_cif_send_bulk_out(bdev, skb);
	} else if (pkt_type == BTMTK_TX_PKT_FROM_HOST) {
		/* handle hci cmd and acl pkt from host, handle hci cmd from driver */
		ret = btusb_send_frame(bdev->hdev, skb);
	}

	return ret;
}

static int btmtk_cif_recv_evt(struct btmtk_dev *bdev, int delay, int retry)
{
	struct btmtk_usb_dev *cif_dev = NULL;
	int ret = -1;	/* if successful, 0 */
	unsigned int ifnum_base;

	if (bdev == NULL) {
		BTMTK_ERR("%s: bdev == NULL!\n", __func__);
		return ret;
	}

	cif_dev = (struct btmtk_usb_dev *)bdev->cif_dev;
	if (!cif_dev->udev || !bdev->hdev) {
		BTMTK_ERR("%s: invalid parameters!\n", __func__);
		return ret;
	}

	ifnum_base = cif_dev->intf->cur_altsetting->desc.bInterfaceNumber;
get_response_again:
	/* ms delay */
	mdelay(delay);

	/* check WMT event */
	memset(bdev->io_buf, 0, IO_BUF_SIZE);
	bdev->io_buf[0] = HCI_EVENT_PKT;
	if (BTMTK_IS_BT_0_INTF(ifnum_base))
		ret = usb_control_msg(cif_dev->udev, usb_rcvctrlpipe(cif_dev->udev, 0),
				0x01, DEVICE_VENDOR_REQUEST_IN, 0x30, 0x00, bdev->io_buf + 1,
				HCI_USB_IO_BUF_SIZE, USB_CTRL_IO_TIMO);
	else if (BTMTK_IS_BT_1_INTF(ifnum_base))
		ret = usb_control_msg(cif_dev->udev, usb_rcvctrlpipe(cif_dev->udev, 0),
				0x01, 0xA1, 0x30, 0x03, bdev->io_buf + 1, HCI_USB_IO_BUF_SIZE,
				USB_CTRL_IO_TIMO);

	if (ret < 0) {
		BTMTK_ERR("%s: event get failed(%d)", __func__, ret);
		return ret;
	}

	if (ret > 0) {
		BTMTK_DBG_RAW(bdev->io_buf, ret + 1, "%s OK: EVT:", __func__);
		return ret + 1; /* return read length */
	} else if (retry > 0) {
		BTMTK_WARN("%s: Trying to get response... (%d)", __func__, ret);
		retry--;
		goto get_response_again;
	} else
		BTMTK_ERR("%s NG: do not got response:(%d)", __func__, ret);

	return -1;
}

int btmtk_usb_send_and_recv(struct btmtk_dev *bdev,
		struct sk_buff *skb,
		const uint8_t *event, const int event_len,
		int delay, int retry, int pkt_type)
{
	unsigned long comp_event_timo = 0, start_time = 0;
	int ret = 0;

	if (bdev == NULL) {
		BTMTK_ERR("%s: bdev == NULL!\n", __func__);
		return ret;
	}

	if ((pkt_type == BTMTK_TX_CMD_FROM_DRV || pkt_type == BTMTK_TX_ACL_FROM_DRV)) {
		BTMTK_DBG_RAW(skb->data, skb->len, "%s, send, len = %d", __func__, skb->len);

		ret = btmtk_usb_send_cmd(bdev, skb, delay, retry, pkt_type);
		if (ret < 0) {
			BTMTK_ERR("%s btmtk_usb_send_cmd failed!!", __func__);
			goto fw_assert;
		}

		if (event && event_len > 0) {
			bdev->recv_evt_len = btmtk_cif_recv_evt(bdev, delay, retry);
			if (bdev->recv_evt_len < 0) {
				BTMTK_ERR("%s btmtk_cif_recv_evt failed!!", __func__);
				ret = -1;
				goto fw_assert;
			}

			if (bdev->io_buf && bdev->recv_evt_len >= event_len) {
				if (memcmp(bdev->io_buf, event, event_len) == 0) {
					ret = 0;
					goto exit;
				}
			}
			BTMTK_INFO("%s compare fail\n", __func__);
			BTMTK_INFO_RAW(event, event_len, "%s: event_need_compare:", __func__);
			BTMTK_INFO_RAW(bdev->io_buf, bdev->recv_evt_len, "%s: RCV:", __func__);
			ret = -1;
		} else {
			ret = 0;
		}
	} else {
		if (event) {
			if (event_len > EVENT_COMPARE_SIZE) {
				BTMTK_ERR("%s, event_len (%d) > EVENT_COMPARE_SIZE(%d), error",
					__func__, event_len, EVENT_COMPARE_SIZE);
				ret = -1;
				goto exit;
			}
			event_compare_status = BTMTK_EVENT_COMPARE_STATE_NEED_COMPARE;
			memcpy(event_need_compare, event + 1, event_len - 1);
			event_need_compare_len = event_len - 1;

			start_time = jiffies;
			/* check hci event /wmt event for SDIO/UART interface, check hci
			 * event for USB interface
			 */
			comp_event_timo = jiffies + msecs_to_jiffies(WOBLE_COMP_EVENT_TIMO);
			BTMTK_INFO("event_need_compare_len %d, event_compare_status %d",
				event_need_compare_len, event_compare_status);
		} else {
			event_compare_status = BTMTK_EVENT_COMPARE_STATE_COMPARE_SUCCESS;
		}

		BTMTK_DBG_RAW(skb->data, skb->len, "%s, send, len = %d", __func__, skb->len);

		ret = btmtk_usb_send_cmd(bdev, skb, delay, retry, pkt_type);
		if (ret < 0) {
			BTMTK_ERR("%s btmtk_sdio_send_cmd failed!!", __func__);
			goto fw_assert;
		}

		do {
			/* check if event_compare_success */
			if (event_compare_status == BTMTK_EVENT_COMPARE_STATE_COMPARE_SUCCESS) {
				ret = 0;
				break;
			}
			usleep_range(10, 100);
		} while (time_before(jiffies, comp_event_timo));

		event_compare_status = BTMTK_EVENT_COMPARE_STATE_NOTHING_NEED_COMPARE;
	}
	goto exit;
fw_assert:
	btmtk_send_assert_cmd(bdev);
exit:
	return ret;
}

void btmtk_usb_chip_reset_notify(struct btmtk_dev *bdev)
{
	cancel_work_sync(&bdev->work);
	cancel_work_sync(&bdev->waker);
}

int btmtk_usb_event_filter(struct btmtk_dev *bdev, struct sk_buff *skb)
{
	u8 read_address_event[READ_ADDRESS_EVT_HDR_LEN] = {0x04, 0x0E, 0x0A, 0x01, 0x09, 0x10, 0x00 };

	if (event_compare_status == BTMTK_EVENT_COMPARE_STATE_NEED_COMPARE &&
		skb->len >= event_need_compare_len) {
		if (memcmp(skb->data, &read_address_event[1], READ_ADDRESS_EVT_HDR_LEN - 1) == 0
			&& (skb->len == (READ_ADDRESS_EVT_HDR_LEN - 1 + BD_ADDRESS_SIZE))) {
			memcpy(bdev->bdaddr, &skb->data[READ_ADDRESS_EVT_PAYLOAD_OFFSET - 1], BD_ADDRESS_SIZE);
			BTMTK_INFO("GET BDADDR = %02X:%02X:%02X:%02X:%02X:%02X",
				bdev->bdaddr[0], bdev->bdaddr[1], bdev->bdaddr[2],
				bdev->bdaddr[3], bdev->bdaddr[4], bdev->bdaddr[5]);

			event_compare_status = BTMTK_EVENT_COMPARE_STATE_COMPARE_SUCCESS;
		} else if (memcmp(skb->data, event_need_compare,
					event_need_compare_len) == 0) {
			/* if it is wobx debug event, just print in kernel log, drop it
			 * by driver, don't send to stack
			 */
			if (skb->data[0] == WOBLE_DEBUG_EVT_TYPE)
				BTMTK_INFO_RAW(skb->data, skb->len, "%s: wobx debug log:", __func__);

			event_compare_status = BTMTK_EVENT_COMPARE_STATE_COMPARE_SUCCESS;
			BTMTK_INFO("%s, compare success", __func__);
		} else {
			BTMTK_INFO("%s compare fail", __func__);
			BTMTK_INFO_RAW(event_need_compare, event_need_compare_len,
				"%s: event_need_compare:", __func__);
			BTMTK_INFO_RAW(skb->data, skb->len, "%s: skb->data:", __func__);
			return 0;
		}

		return 1;
	}

	return 0;
}
