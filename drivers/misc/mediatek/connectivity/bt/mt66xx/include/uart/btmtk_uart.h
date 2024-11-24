/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _BTMTK_UART_H_
#define _BTMTK_UART_H_
#include "btmtk_define.h"


#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/serial.h>

#define HCI_HEADER_LEN	4

struct mtk_stp_hdr {
	u8	prefix;
	__be16	dlen;
	u8	cs;
} __packed;
#define MTK_STP_TLR_SIZE	2
#define STP_HEADER_LEN	4
#define STP_HEADER_CRC_LEN	2


struct btmtk_uart_dev {
	struct hci_dev	   *hdev;
	struct tty_struct *tty;
	unsigned long	hdev_flags;

	/* For tx queue */
	struct sk_buff		*tx_skb;
	unsigned long		tx_state;

	/* For rx queue */
	struct sk_buff		*rx_skb;
	unsigned long		rx_state;

	struct sk_buff		*evt_skb;
	wait_queue_head_t p_wait_event_q;

	unsigned int		subsys_reset;

	u8	stp_pad[6];
	u8	stp_cursor;
	u16	stp_dlen;
};


/**
 * Maximum rom patch file name length
 */
#define MAX_BIN_FILE_NAME_LEN 32

#define N_MTK        (15+1)
/**
 * Upper layeard IOCTL
 */
#define HCIUARTSETPROTO _IOW('U', 200, int)
#define HCIUARTSETBAUD _IOW('U', 201, int)
#define HCIUARTGETBAUD _IOW('U', 202, int)
#define HCIUARTSETSTP _IOW('U', 203, int)
#define HCIUARTLOADPATCH _IOW('U', 204, int)
#define HCIUARTSETWAKEUP _IOW('U', 205, int)

/**
 * Send cmd dispatch evt
 */
#define RETRY_TIMES 10
#define HCI_EV_VENDOR			0xff

#define N_MTK        (15+1)

int btmtk_cif_send_calibration(struct hci_dev *hdev);
#endif

