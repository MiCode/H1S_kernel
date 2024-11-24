/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __HOST_CSR_H__
#define __HOST_CSR_H__

#define HOST_CSR_DRIVER_OWN_INFO				0x7000
#define HOST_CSR_BASE						0x7100

/* MCU programming Counter  info (no sync) */
#define HOST_CSR_MCU_PORG_COUNT				(HOST_CSR_BASE + 0x04)

/* RGU Info */
#define HOST_CSR_RGU					(HOST_CSR_BASE + 0x08)

/* HIF_BUSY / CIRQ / WFSYS_ON info */
#define HOST_CSR_HIF_BUSY_CORQ_WFSYS_ON			(HOST_CSR_BASE + 0x0C)

/* Pinmux/mon_flag info */
#define HOST_CSR_PINMUX_MON_FLAG			(HOST_CSR_BASE + 0x10)

/* Bit[5] mcu_pwr_stat */
#define HOST_CSR_MCU_PWR_STAT				(HOST_CSR_BASE + 0x14)

/* Bit[15] fw_own_stat */
#define HOST_CSR_FW_OWN_SET				(HOST_CSR_BASE + 0x18)

/* MCU SW Mailbox */
#define HOST_CSR_MCU_SW_MAILBOX_0			(HOST_CSR_BASE + 0x1C)
#define HOST_CSR_MCU_SW_MAILBOX_1			(HOST_CSR_BASE + 0x20)
#define HOST_CSR_MCU_SW_MAILBOX_2			(HOST_CSR_BASE + 0x24)
#define HOST_CSR_MCU_SW_MAILBOX_3			(HOST_CSR_BASE + 0x28)

/* Conn_cfg_on info */
#define HOST_CSR_CONN_CFG_ON				(HOST_CSR_BASE + 0x2C)

/* Get conn_cfg_on IRQ ENA/IRQ Status 0xC_1170~0xC_1174 */
#define HOST_CSR_IRQ_STA				0xC1170
#define HOST_CSR_IRQ_ENA				0xC1174

/* Get mcu_cfg PC programming Counter log info 0x0_2450~0x0_24D0 */
#define HOST_CSR_MCU_PROG_COUNT				0x2450

#if (CFG_ENABLE_HOST_BUS_TIMEOUT == 1)
#define HOST_CSR_BUS_TIMOUT_CTRL_ADDR	(HOST_CSR_DRIVER_OWN_INFO + 0x44)
#define HOST_CSR_AP2CONN_AHB_HADDR	(HOST_CSR_DRIVER_OWN_INFO + 0x4C)
#endif

/* Modem LP control */
#define HOST_CSR_CONN_HIF_ON_MD_LPCTL_ADDR (HOST_CSR_DRIVER_OWN_INFO + 0x30)
/* Modem interrupt enable */
#define HOST_CSR_CONN_HIF_ON_MD_IRQ_STAT_ADDR (HOST_CSR_DRIVER_OWN_INFO + 0x34)
/* Modem interrupt status */
#define HOST_CSR_CONN_HIF_ON_MD_IRQ_ENA_ADDR (HOST_CSR_DRIVER_OWN_INFO + 0x38)

/*
* AP2CONN_ADDR_MAP1[31..16]
* Mapping [0x180A_xxxx] to [ap2conn_addr_map0[15:0], xxxx]
*
* AP2CONN_ADDR_MAP0[15..0]
* Mapping [0x180D_xxxx] to [ap2conn_addr_map1[15:0], xxxx]
*/
#define CONN_HIF_ON_ADDR_REMAP1					0x700C
#define AP2CONN_ADDR_MAP0					0xD0000
#define AP2CONN_ADDR_MAP1					0xA0000
/*
* AP2CONN_ADDR_MAP3[31..16]
* Mapping [0x180F_xxxx] to [ap2conn_addr_map3[15:0], xxxx]
*
* AP2CONN_ADDR_MAP2[15..0]
* Mapping [0x1804_xxxx] to [ap2conn_addr_map2[15:0], xxxx]
*/
#define CONN_HIF_ON_ADDR_REMAP2					0x7010
#define AP2CONN_ADDR_MAP2					0x40000
#define AP2CONN_ADDR_MAP3					0xF0000

#endif /* __HOST_CSR_H__ */
