/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#ifndef _LINUX_SERIAL_H
#define _LINUX_SERIAL_H

struct serial_struct {
	int	type;
	int	line;
	unsigned int	port;
	int	irq;
	int	flags;
	int	xmit_fifo_size;
	int	custom_divisor;
	int	baud_base;
	unsigned short	close_delay;
	char	io_type;
	char	reserved_char[1];
	int	hub6;
	unsigned short	closing_wait;
	unsigned short	closing_wait2;
	unsigned char	*iomem_base;
	unsigned short	iomem_reg_shift;
	unsigned int	port_high;
	unsigned long	iomap_base;
};

#define ASYNCB_HUP_NOTIFY	 0
#define ASYNCB_FOURPORT		 1
#define ASYNCB_SAK		 2
#define ASYNCB_SPLIT_TERMIOS	 3
#define ASYNCB_SPD_HI		 4
#define ASYNCB_SPD_VHI		 5
#define ASYNCB_SKIP_TEST	 6
#define ASYNCB_AUTO_IRQ		 7
#define ASYNCB_SESSION_LOCKOUT	 8
#define ASYNCB_PGRP_LOCKOUT	 9
#define ASYNCB_CALLOUT_NOHUP	10
#define ASYNCB_HARDPPS_CD	11
#define ASYNCB_SPD_SHI		12
#define ASYNCB_LOW_LATENCY	13
#define ASYNCB_BUGGY_UART	14
#define ASYNCB_AUTOPROBE	15

#define ASYNC_HUP_NOTIFY	(1U << ASYNCB_HUP_NOTIFY)
#define ASYNC_SUSPENDED		(1U << ASYNCB_SUSPENDED)
#define ASYNC_FOURPORT		(1U << ASYNCB_FOURPORT)
#define ASYNC_SAK		(1U << ASYNCB_SAK)
#define ASYNC_SPLIT_TERMIOS	(1U << ASYNCB_SPLIT_TERMIOS)
#define ASYNC_SPD_HI		(1U << ASYNCB_SPD_HI)
#define ASYNC_SPD_VHI		(1U << ASYNCB_SPD_VHI)
#define ASYNC_SKIP_TEST		(1U << ASYNCB_SKIP_TEST)
#define ASYNC_AUTO_IRQ		(1U << ASYNCB_AUTO_IRQ)
#define ASYNC_SESSION_LOCKOUT	(1U << ASYNCB_SESSION_LOCKOUT)
#define ASYNC_PGRP_LOCKOUT	(1U << ASYNCB_PGRP_LOCKOUT)
#define ASYNC_CALLOUT_NOHUP	(1U << ASYNCB_CALLOUT_NOHUP)
#define ASYNC_HARDPPS_CD	(1U << ASYNCB_HARDPPS_CD)
#define ASYNC_SPD_SHI		(1U << ASYNCB_SPD_SHI)
#define ASYNC_LOW_LATENCY	(1U << ASYNCB_LOW_LATENCY)
#define ASYNC_BUGGY_UART	(1U << ASYNCB_BUGGY_UART)
#define ASYNC_AUTOPROBE		(1U << ASYNCB_AUTOPROBE)

#define ASYNC_FLAGS		((1U << ASYNCB_LAST_USER) - 1)
#define ASYNC_USR_MASK		(ASYNC_SPD_HI|ASYNC_SPD_VHI| \
		ASYNC_CALLOUT_NOHUP|ASYNC_SPD_SHI|ASYNC_LOW_LATENCY)
#define ASYNC_SPD_CUST		(ASYNC_SPD_HI|ASYNC_SPD_VHI)
#define ASYNC_SPD_WARP		(ASYNC_SPD_HI|ASYNC_SPD_SHI)
#define ASYNC_SPD_MASK		(ASYNC_SPD_HI|ASYNC_SPD_VHI|ASYNC_SPD_SHI)

#define ASYNC_INITIALIZED	(1U << ASYNCB_INITIALIZED)
#define ASYNC_NORMAL_ACTIVE	(1U << ASYNCB_NORMAL_ACTIVE)
#define ASYNC_BOOT_AUTOCONF	(1U << ASYNCB_BOOT_AUTOCONF)
#define ASYNC_CLOSING		(1U << ASYNCB_CLOSING)
#define ASYNC_CTS_FLOW		(1U << ASYNCB_CTS_FLOW)
#define ASYNC_CHECK_CD		(1U << ASYNCB_CHECK_CD)
#define ASYNC_SHARE_IRQ		(1U << ASYNCB_SHARE_IRQ)
#define ASYNC_CONS_FLOW		(1U << ASYNCB_CONS_FLOW)
#define ASYNC_BOOT_ONLYMCA	(1U << ASYNCB_BOOT_ONLYMCA)
#define ASYNC_INTERNAL_FLAGS	(~((1U << ASYNCB_FIRST_KERNEL) - 1))

#endif /* _LINUX_SERIAL_H */
