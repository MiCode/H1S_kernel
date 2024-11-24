/*
 * Copyright (C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef FM_EXT_API_H
#define FM_EXT_API_H

#include <linux/platform_device.h>

#include "fm_interface.h"

enum fm_spi_speed {
	FM_SPI_SPEED_26M,
	FM_SPI_SPEED_64M,
	FM_SPI_SPEED_MAX
};

enum fm_reg_map {
	CONN_TCR_CKMCTL,
	CONN_TCR_FMAUD_SET,
	CONN_TCR_FI2SCK_DIV_CNT,
	CONN_INFRA_WAKEPU_FM,
	CONN_HW_VER,
	OSC_MASK,
	PLL_STATUS,
	CONN_INFRA_CFG_RC_STATUS,
	ADIE_CTL,
	CKGEN_BUS,
	FM_CTRL_MEM_SWCTL_PDN,
	RG_DIG_EN_02,
	SPI_CRTL,
	FM_CTRL,
	WB_SLP_TOP_CK_4,
	CONN_INFRA_CFG_PWRCTRL1,
	FM_REG_MAP_MAX
};
#define CONN_INFRA_CFG_FM_PWRCTRL0 CONN_TCR_CKMCTL

enum fm_base_addr {
	CONN_RF_SPI_MST_REG,
	CONN_HOST_CSR_TOP,
	MCU_CFG_CONSYS,
	FM_BASE_ADDR_MAX
};

#define FM_REG_NO_USED 0x00000000

struct fm_ext_interface {
	void (*eint_handler)(void);
	void (*eint_cb)(void);
	void (*enable_eint)(void);
	void (*disable_eint)(void);
	int (*stp_send_data)(unsigned char *buf, unsigned int len);
	int (*stp_recv_data)(unsigned char *buf, unsigned int len);
	int (*stp_register_event_cb)(void *cb);
	int (*wmt_msgcb_reg)(void *data);
	int (*wmt_func_on)(void);
	int (*wmt_func_off)(void);
	unsigned int (*wmt_ic_info_get)(void);
	int (*wmt_chipid_query)(void);
	int (*get_hw_version)(void);
	unsigned char (*get_top_index)(void);
	unsigned int (*get_get_adie)(void);
	bool (*is_aoc_support)(void);
	int (*spi_clock_switch)(enum fm_spi_speed speed);
	bool (*is_bus_hang)(void);
	int (*spi_hopping)(void);
	int (*disable_spi_hopping)(void);
	void (*host_reg_dump)(void);
	int (*host_pre_on)(void);
	int (*host_post_on)(void);
	int (*host_pre_off)(void);
	int (*host_post_off)(void);
	signed int (*low_ops_register)(struct fm_callback *cb, struct fm_basic_interface *bi);
	signed int (*low_ops_unregister)(struct fm_basic_interface *bi);
	signed int (*rds_ops_register)(struct fm_basic_interface *bi, struct fm_rds_interface *ri);
	signed int (*rds_ops_unregister)(struct fm_rds_interface *ri);

	struct platform_driver *drv;
	unsigned int irq_id;
	unsigned int family_id;
	unsigned int conn_id;
	unsigned int reg_map[FM_REG_MAP_MAX];
	unsigned int base_addr[FM_BASE_ADDR_MAX];
	unsigned int base_size[FM_BASE_ADDR_MAX];
};

#endif /* FM_EXT_API_H */
