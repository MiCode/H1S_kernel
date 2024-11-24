/*
 * Copyright (C) 2016 MediaTek Inc.
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
/*! \file
*    \brief  Declaration of library functions
*
*    Any definitions in this file will be shared among GLUE Layer and internal Driver Stack.
*/
/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

#ifdef DFT_TAG
#undef DFT_TAG
#endif
#define DFT_TAG "[WMT-CONSYS-HW]"

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include <connectivity_build_in_adapter.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/memblock.h>
#include <linux/platform_device.h>
#include "connsys_debug_utility.h"
#ifdef CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH
#include "fw_log_wmt.h"
#endif
#include "osal_typedef.h"
#include "mt6785.h"
#include "mtk_wcn_consys_hw.h"
#include "wmt_ic.h"
#include "wmt_lib.h"
#include "wmt_plat.h"
#include "stp_dbg.h"

#ifdef CONFIG_MTK_EMI
#include <mt_emi_api.h>
#endif

#if CONSYS_PMIC_CTRL_ENABLE
#include <mtk_pmic_api_buck.h>
#include <upmu_common.h>
#include <linux/regulator/consumer.h>
#endif

#ifdef CONFIG_MTK_HIBERNATION
#include <mtk_hibernate_dpm.h>
#endif

#include <linux/of_reserved_mem.h>

#include <mtk_clkbuf_ctl.h>

/* Direct path */
#include <mtk_ccci_common.h>
#if WMT_DEVAPC_DBG_SUPPORT
#include <devapc_public.h>
#endif
/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
static INT32 consys_clock_buffer_ctrl(MTK_WCN_BOOL enable);
static VOID consys_hw_reset_bit_set(MTK_WCN_BOOL enable);
static VOID consys_hw_spm_clk_gating_enable(VOID);
static INT32 consys_hw_power_ctrl(MTK_WCN_BOOL enable);
static INT32 consys_ahb_clock_ctrl(MTK_WCN_BOOL enable);
static INT32 polling_consys_chipid(VOID);
static VOID consys_acr_reg_setting(VOID);
static VOID consys_afe_reg_setting(VOID);
static INT32 consys_hw_vcn18_ctrl(MTK_WCN_BOOL enable);
static VOID consys_vcn28_hw_mode_ctrl(UINT32 enable);
static INT32 consys_hw_vcn28_ctrl(UINT32 enable);
static INT32 consys_hw_wifi_vcn33_ctrl(UINT32 enable);
static INT32 consys_hw_bt_vcn33_ctrl(UINT32 enable);
static UINT32 consys_soc_chipid_get(VOID);
static INT32 consys_emi_mpu_set_region_protection(VOID);
static UINT32 consys_emi_set_remapping_reg(VOID);
static INT32 bt_wifi_share_v33_spin_lock_init(VOID);
static INT32 consys_clk_get_from_dts(struct platform_device *pdev);
static INT32 consys_pmic_get_from_dts(struct platform_device *pdev);
static INT32 consys_read_irq_info_from_dts(struct platform_device *pdev,
		PINT32 irq_num, PUINT32 irq_flag);
static INT32 consys_read_reg_from_dts(struct platform_device *pdev);
static UINT32 consys_read_cpupcr(VOID);
static INT32 consys_poll_cpupcr_dump(UINT32 times, UINT32 sleep_ms);
static VOID force_trigger_assert_debug_pin(VOID);
static INT32 consys_co_clock_type(VOID);
static P_CONSYS_EMI_ADDR_INFO consys_soc_get_emi_phy_add(VOID);
static VOID consys_set_if_pinmux(MTK_WCN_BOOL enable);
static INT32 consys_dl_rom_patch(UINT32 ip_ver, UINT32 fw_ver);
static VOID consys_set_dl_rom_patch_flag(INT32 flag);
static INT32 consys_dedicated_log_path_init(struct platform_device *pdev);
static VOID consys_dedicated_log_path_deinit(VOID);
static INT32 consys_check_reg_readable(VOID);
static INT32 consys_emi_coredump_remapping(UINT8 __iomem **addr, UINT32 enable);
static INT32 consys_reset_emi_coredump(UINT8 __iomem *addr);
static VOID consys_ic_clock_fail_dump(VOID);
static INT32 consys_is_connsys_reg(UINT32 addr);
static INT32 consys_is_host_csr(SIZE_T addr);
static INT32 consys_dump_osc_state(P_CONSYS_STATE state);
static VOID consys_set_pdma_axi_rready_force_high(UINT32 enable);
static VOID consys_set_mcif_emi_mpu_protection(MTK_WCN_BOOL enable);
/*
 * If 1: this platform supports calibration backup/restore.
 * otherwise: 0
 */
static INT32 consys_calibration_backup_restore_support(VOID);
#if WMT_DEVAPC_DBG_SUPPORT
static VOID consys_devapc_violation_cb(VOID);
#endif
static VOID consyc_register_devapc_cb(VOID);
static UINT64 consys_get_options(VOID);

static INT32 dump_conn_mcu_pc_log_wrapper(VOID);
static INT32 consys_cmd_tx_timeout_dump(VOID);
static INT32 consys_cmd_rx_timeout_dump(VOID);
static INT32 consys_coredump_timeout_dump(VOID);
static INT32 consys_assert_timeout_dump(VOID);
static INT32 consys_before_chip_reset_dump(VOID);

static INT32 consys_jtag_set_for_mcu(VOID);
static UINT32 consys_jtag_flag_ctrl(UINT32 enable);

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/
/* CCF part */
static struct clk *clk_scp_conn_main;	/*ctrl conn_power_on/off */
struct clk *clk_infracfg_ao_ccif4_ap_cg;       /* For direct path */

/* PMIC part */
#if CONSYS_PMIC_CTRL_ENABLE
static struct regulator *reg_VCN18;
static struct regulator *reg_VCN33_1;
static struct regulator *reg_VCN33_2;
#endif

static EMI_CTRL_STATE_OFFSET mtk_wcn_emi_state_off = {
	.emi_apmem_ctrl_state = EXP_APMEM_CTRL_STATE,
	.emi_apmem_ctrl_host_sync_state = EXP_APMEM_CTRL_HOST_SYNC_STATE,
	.emi_apmem_ctrl_host_sync_num = EXP_APMEM_CTRL_HOST_SYNC_NUM,
	.emi_apmem_ctrl_chip_sync_state = EXP_APMEM_CTRL_CHIP_SYNC_STATE,
	.emi_apmem_ctrl_chip_sync_num = EXP_APMEM_CTRL_CHIP_SYNC_NUM,
	.emi_apmem_ctrl_chip_sync_addr = EXP_APMEM_CTRL_CHIP_SYNC_ADDR,
	.emi_apmem_ctrl_chip_sync_len = EXP_APMEM_CTRL_CHIP_SYNC_LEN,
	.emi_apmem_ctrl_chip_print_buff_start = EXP_APMEM_CTRL_CHIP_PRINT_BUFF_START,
	.emi_apmem_ctrl_chip_print_buff_len = EXP_APMEM_CTRL_CHIP_PRINT_BUFF_LEN,
	.emi_apmem_ctrl_chip_print_buff_idx = EXP_APMEM_CTRL_CHIP_PRINT_BUFF_IDX,
	.emi_apmem_ctrl_chip_int_status = EXP_APMEM_CTRL_CHIP_INT_STATUS,
	.emi_apmem_ctrl_chip_paded_dump_end = EXP_APMEM_CTRL_CHIP_PAGED_DUMP_END,
	.emi_apmem_ctrl_host_outband_assert_w1 = EXP_APMEM_CTRL_HOST_OUTBAND_ASSERT_W1,
	.emi_apmem_ctrl_chip_page_dump_num = EXP_APMEM_CTRL_CHIP_PAGE_DUMP_NUM,
	.emi_apmem_ctrl_assert_flag = EXP_APMEM_CTRL_ASSERT_FLAG,
};

static CONSYS_EMI_ADDR_INFO mtk_wcn_emi_addr_info = {
	.emi_phy_addr = CONSYS_EMI_FW_PHY_BASE,
	.paged_trace_off = CONSYS_EMI_PAGED_TRACE_OFFSET,
	.paged_dump_off = CONSYS_EMI_PAGED_DUMP_OFFSET,
	.full_dump_off = CONSYS_EMI_FULL_DUMP_OFFSET,
	.emi_remap_offset = CONSYS_EMI_MAPPING_OFFSET,
	.p_ecso = &mtk_wcn_emi_state_off,
	.emi_core_dump_offset = CONSYS_EMI_COREDUMP_OFFSET,
	.pda_dl_patch_flag = 1,
	.emi_met_size = (32*KBYTE),
	.emi_met_data_offset = CONSYS_EMI_MET_DATA_OFFSET,
};

WMT_CONSYS_IC_OPS consys_ic_ops_mt6785 = {
	.consys_ic_clock_buffer_ctrl = consys_clock_buffer_ctrl,
	.consys_ic_hw_reset_bit_set = consys_hw_reset_bit_set,
	.consys_ic_hw_spm_clk_gating_enable = consys_hw_spm_clk_gating_enable,
	.consys_ic_hw_power_ctrl = consys_hw_power_ctrl,
	.consys_ic_ahb_clock_ctrl = consys_ahb_clock_ctrl,
	.polling_consys_ic_chipid = polling_consys_chipid,
	.consys_ic_acr_reg_setting = consys_acr_reg_setting,
	.consys_ic_afe_reg_setting = consys_afe_reg_setting,
	.consys_ic_hw_vcn18_ctrl = consys_hw_vcn18_ctrl,
	.consys_ic_vcn28_hw_mode_ctrl = consys_vcn28_hw_mode_ctrl,
	.consys_ic_hw_vcn28_ctrl = consys_hw_vcn28_ctrl,
	.consys_ic_hw_wifi_vcn33_ctrl = consys_hw_wifi_vcn33_ctrl,
	.consys_ic_hw_bt_vcn33_ctrl = consys_hw_bt_vcn33_ctrl,
	.consys_ic_soc_chipid_get = consys_soc_chipid_get,
	.consys_ic_emi_mpu_set_region_protection = consys_emi_mpu_set_region_protection,
	.consys_ic_emi_set_remapping_reg = consys_emi_set_remapping_reg,
	.ic_bt_wifi_share_v33_spin_lock_init = bt_wifi_share_v33_spin_lock_init,
	.consys_ic_clk_get_from_dts = consys_clk_get_from_dts,
	.consys_ic_pmic_get_from_dts = consys_pmic_get_from_dts,
	.consys_ic_read_irq_info_from_dts = consys_read_irq_info_from_dts,
	.consys_ic_read_reg_from_dts = consys_read_reg_from_dts,
	.consys_ic_read_cpupcr = consys_read_cpupcr,
	.consys_ic_poll_cpupcr_dump = consys_poll_cpupcr_dump,
	.ic_force_trigger_assert_debug_pin = force_trigger_assert_debug_pin,
	.consys_ic_co_clock_type = consys_co_clock_type,
	.consys_ic_soc_get_emi_phy_add = consys_soc_get_emi_phy_add,
	.consys_ic_set_if_pinmux = consys_set_if_pinmux,
	.consys_ic_set_dl_rom_patch_flag = consys_set_dl_rom_patch_flag,
	.consys_ic_dedicated_log_path_init = consys_dedicated_log_path_init,
	.consys_ic_dedicated_log_path_deinit = consys_dedicated_log_path_deinit,
	.consys_ic_check_reg_readable = consys_check_reg_readable,
	.consys_ic_emi_coredump_remapping = consys_emi_coredump_remapping,
	.consys_ic_reset_emi_coredump = consys_reset_emi_coredump,
	.consys_ic_clock_fail_dump = consys_ic_clock_fail_dump,
	.consys_ic_is_connsys_reg = consys_is_connsys_reg,
	.consys_ic_is_host_csr = consys_is_host_csr,
	.consys_ic_dump_osc_state = consys_dump_osc_state,
	.consys_ic_set_pdma_axi_rready_force_high = consys_set_pdma_axi_rready_force_high,
	.consys_ic_set_mcif_emi_mpu_protection = consys_set_mcif_emi_mpu_protection,
	.consys_ic_calibration_backup_restore = consys_calibration_backup_restore_support,
	.consys_ic_register_devapc_cb = consyc_register_devapc_cb,
	.consys_ic_get_options = consys_get_options,

	/* debug dump */
	.consys_ic_cmd_tx_timeout_dump = consys_cmd_tx_timeout_dump,
	.consys_ic_cmd_rx_timeout_dump = consys_cmd_rx_timeout_dump,
	.consys_ic_coredump_timeout_dump = consys_coredump_timeout_dump,
	.consys_ic_assert_timeout_dump = consys_assert_timeout_dump,
	.consys_ic_before_chip_reset_dump = consys_before_chip_reset_dump,

	.consys_ic_pc_log_dump = dump_conn_mcu_pc_log_wrapper,

	.consys_ic_jtag_set_for_mcu = consys_jtag_set_for_mcu,
	.consys_ic_jtag_flag_ctrl = consys_jtag_flag_ctrl,
};

static const struct connlog_emi_config connsys_fw_log_parameter = {
	.emi_offset = 0x36500,
	.emi_size_total = (192*1024),/* 192KB */
	.emi_size_mcu = (16*1024),
	.emi_size_wifi = (64*1024),
	.emi_size_bt = (64*1024),
	.emi_size_gps = (32*1024),
};

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/
static INT32 rom_patch_dl_flag = 1;
static UINT32 gJtagCtrl;

#if WMT_DEVAPC_DBG_SUPPORT
static struct devapc_vio_callbacks devapc_handle = {
	.id = INFRA_SUBSYS_CONN,
	.debug_dump = consys_devapc_violation_cb,
};
#endif

#if CONSYS_ENALBE_SET_JTAG
#define JTAG_ADDR1_BASE 0x10005000
#define JTAG_ADDR2_BASE 0x11F20000
#define AP2CONN_JTAG_2WIRE_OFFSET 0xF00
#endif

static INT32 consys_jtag_set_for_mcu(VOID)
{
	INT32 ret = 0;
#if CONSYS_ENALBE_SET_JTAG
	UINT32 tmp = 0;
	PVOID addr = 0;
	PVOID remap_addr1 = 0;
	PVOID remap_addr2 = 0;

	if (gJtagCtrl) {
		WMT_PLAT_PR_INFO("WCN jtag set for mcu start...\n");

		switch (gJtagCtrl) {
		case 1:
			remap_addr1 = ioremap(JTAG_ADDR1_BASE, 0x1000);
			if (remap_addr1 == 0) {
				WMT_PLAT_PR_ERR("remap jtag_addr1 fail!\n");
				ret = -1;
				goto error;
			}

			remap_addr2 = ioremap(JTAG_ADDR2_BASE, 0x100);
			if (remap_addr2 == 0) {
				WMT_PLAT_PR_ERR("remap jtag_addr2 fail!\n");
				ret = -1;
				goto error;
			}

			/* 7-wire jtag pinmux setting*/
			/* PAD AUX Function Selection */
			addr = remap_addr1 + 0x310;
			tmp = readl(addr);
			tmp = tmp & 0xffffff00;
			tmp = tmp | 0x66;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			addr = remap_addr1 + 0x330;
			tmp = readl(addr);
			tmp = tmp & 0x00ffffff;
			tmp = tmp | 0x66000000;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			addr = remap_addr1 + 0x340;
			tmp = readl(addr);
			tmp = tmp & 0xfffff000;
			tmp = tmp | 0x666;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			/* PAD Driving Selection */
			addr = remap_addr2;
			tmp = readl(addr);
			tmp = tmp & 0xfffc01ff;
			tmp = tmp | 0x3fe00;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			/* PAD PULL Selection */
			addr = remap_addr2 + 0x40;
			tmp = readl(addr);
			tmp = tmp & 0xfffc07ff;
			tmp = tmp | 0x3f800;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			break;
		case 2:
			remap_addr1 = ioremap(JTAG_ADDR1_BASE, 0x1000);
			if (remap_addr1 == 0) {
				WMT_PLAT_PR_ERR("remap jtag_addr1 fail!\n");
				ret = -1;
				goto error;
			}

			remap_addr2 = ioremap(0x11C20000, 0x100);
			if (remap_addr2 == 0) {
				WMT_PLAT_PR_ERR("remap jtag_addr2 fail!\n");
				ret = -1;
				goto error;
			}

			/* 2-wire jtag pinmux setting*/
			addr = remap_addr1 + 0x3F0;
			tmp = readl(addr);
			tmp = tmp & 0x0fffffff;
			tmp = tmp | 0x60000000;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			addr = remap_addr1 + 0x400;
			tmp = readl(addr);
			tmp = tmp & 0xfffffff0;
			tmp = tmp | 0x6;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			/* Driving Selection */

			addr = remap_addr2;
			tmp = readl(addr);
			tmp = tmp & 0xfffff1ff;
			tmp = tmp | 0x600;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			/* PULL Selection */
			addr = remap_addr2 + 0x30;
			tmp = readl(addr);
			tmp = tmp & 0xffffff3f;
			tmp = tmp | 0xc0;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);
			break;
		default:
			WMT_PLAT_PR_INFO("unsupported options!\n");
		}

	}

error:
	if (remap_addr1)
		iounmap(remap_addr1);
	if (remap_addr2)
		iounmap(remap_addr2);
#endif
	return ret;
}

static UINT32 consys_jtag_flag_ctrl(UINT32 enable)
{
	WMT_PLAT_PR_INFO("%s jtag set for MCU\n", enable ? "enable" : "disable");
	gJtagCtrl = enable;
	return 0;
}

static INT32 consys_clk_get_from_dts(struct platform_device *pdev)
{
	clk_scp_conn_main = devm_clk_get(&pdev->dev, "conn");
	if (IS_ERR(clk_scp_conn_main)) {
		WMT_PLAT_PR_ERR("[CCF]cannot get clk_scp_conn_main clock.\n");
		return PTR_ERR(clk_scp_conn_main);
	}
	WMT_PLAT_PR_DBG("[CCF]clk_scp_conn_main=%p\n", clk_scp_conn_main);

	clk_infracfg_ao_ccif4_ap_cg = devm_clk_get(&pdev->dev, "ccif");
	if (IS_ERR(clk_infracfg_ao_ccif4_ap_cg)) {
		WMT_PLAT_PR_ERR("[CCF]cannot get clk_infracfg_ao_ccif4_ap_cg clock.\n");
		return PTR_ERR(clk_infracfg_ao_ccif4_ap_cg);
	}
	WMT_PLAT_PR_DBG("[CCF]clk_infracfg_ao_ccif4_ap_cg=%p\n", clk_infracfg_ao_ccif4_ap_cg);

	return 0;
}

static INT32 consys_pmic_get_from_dts(struct platform_device *pdev)
{
#if CONSYS_PMIC_CTRL_ENABLE
	reg_VCN18 = regulator_get(&pdev->dev, "vcn18");
	if (!reg_VCN18)
		WMT_PLAT_PR_ERR("Regulator_get VCN_1V8 fail\n");
	reg_VCN33_1 = regulator_get(&pdev->dev, "vcn33_1_wifi");
	if (!reg_VCN33_1)
		WMT_PLAT_PR_ERR("Regulator_get VCN33_1 fail\n");
	reg_VCN33_2 = regulator_get(&pdev->dev, "vcn33_2_wifi");
	if (!reg_VCN33_2)
		WMT_PLAT_PR_ERR("Regulator_get VCN33_2 fail\n");
#endif
	return 0;
}

static INT32 consys_co_clock_type(VOID)
{
	return 0;
}

static INT32 consys_clock_buffer_ctrl(MTK_WCN_BOOL enable)
{
	if (enable)
		KERNEL_clk_buf_ctrl(CLK_BUF_CONN, true);	/*open XO_WCN*/
	else
		KERNEL_clk_buf_ctrl(CLK_BUF_CONN, false);	/*close XO_WCN*/

	return 0;
}

static VOID consys_set_if_pinmux(MTK_WCN_BOOL enable)
{
	UINT8 *consys_if_pinmux_reg_base = NULL;
	UINT8 *consys_if_pinmux_driving_base = NULL;

	/* Switch D die pinmux for connecting A die */
	consys_if_pinmux_reg_base = ioremap_nocache(CONSYS_IF_PINMUX_REG_BASE, 0x1000);
	if (!consys_if_pinmux_reg_base) {
		WMT_PLAT_PR_ERR("consys_if_pinmux_reg_base(%x) ioremap fail\n",
				CONSYS_IF_PINMUX_REG_BASE);
		return;
	}

	consys_if_pinmux_driving_base = ioremap_nocache(CONSYS_IF_PINMUX_DRIVING_BASE, 0x100);
	if (!consys_if_pinmux_driving_base) {
		WMT_PLAT_PR_ERR("consys_if_pinmux_driving_base(%x) ioremap fail\n",
				CONSYS_IF_PINMUX_DRIVING_BASE);
		if (consys_if_pinmux_reg_base)
			iounmap(consys_if_pinmux_reg_base);
		return;
	}

	if (enable) {
		CONSYS_REG_WRITE(consys_if_pinmux_reg_base + CONSYS_IF_PINMUX_01_OFFSET,
				(CONSYS_REG_READ(consys_if_pinmux_reg_base +
						CONSYS_IF_PINMUX_01_OFFSET) &
				CONSYS_IF_PINMUX_01_MASK) | CONSYS_IF_PINMUX_01_VALUE);
		CONSYS_REG_WRITE(consys_if_pinmux_reg_base + CONSYS_IF_PINMUX_02_OFFSET,
				(CONSYS_REG_READ(consys_if_pinmux_reg_base +
						CONSYS_IF_PINMUX_02_OFFSET) &
				CONSYS_IF_PINMUX_02_MASK) | CONSYS_IF_PINMUX_02_VALUE);
		/* set pinmux driving to 2mA */
		CONSYS_REG_WRITE(consys_if_pinmux_driving_base + CONSYS_IF_PINMUX_DRIVING_OFFSET,
				(CONSYS_REG_READ(consys_if_pinmux_driving_base +
				CONSYS_IF_PINMUX_DRIVING_OFFSET) &
				CONSYS_IF_PINMUX_DRIVING_MASK) | CONSYS_IF_PINMUX_DRIVING_VALUE);
	} else {
		CONSYS_REG_WRITE(consys_if_pinmux_reg_base + CONSYS_IF_PINMUX_01_OFFSET,
				CONSYS_REG_READ(consys_if_pinmux_reg_base +
				CONSYS_IF_PINMUX_01_OFFSET) & CONSYS_IF_PINMUX_01_MASK);
		CONSYS_REG_WRITE(consys_if_pinmux_reg_base + CONSYS_IF_PINMUX_02_OFFSET,
				CONSYS_REG_READ(consys_if_pinmux_reg_base +
				CONSYS_IF_PINMUX_02_OFFSET) & CONSYS_IF_PINMUX_02_MASK);
	}

	if (consys_if_pinmux_reg_base)
		iounmap(consys_if_pinmux_reg_base);
	if (consys_if_pinmux_driving_base)
		iounmap(consys_if_pinmux_driving_base);
}

static VOID consys_hw_reset_bit_set(MTK_WCN_BOOL enable)
{
	UINT32 consys_ver_id = 0;
	UINT32 cnt = 0;

	if ((conn_reg.ap_rgu_base == 0) || (conn_reg.mcu_base == 0) ||
			(conn_reg.mcu_conn_hif_on_base == 0))
		return;

	if (enable) {
		/*3.assert CONNSYS CPU SW reset  0x10007018 "[12]=1'b1  [31:24]=8'h88 (key)" */
		CONSYS_REG_WRITE((conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET),
				CONSYS_REG_READ(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET) |
				CONSYS_CPU_SW_RST_BIT | CONSYS_CPU_SW_RST_CTRL_KEY);
	} else {
		/*16.deassert CONNSYS CPU SW reset 0x10007018 "[12]=1'b0 [31:24] =8'h88 (key)" */
		CONSYS_REG_WRITE(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET,
				(CONSYS_REG_READ(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET) &
				 ~CONSYS_CPU_SW_RST_BIT) | CONSYS_CPU_SW_RST_CTRL_KEY);

		consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_base + 0x600);
		while (consys_ver_id != 0x1D1E) {
			if (cnt > 10)
				break;
			consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_base + 0x600);
			WMT_PLAT_PR_INFO("0x18002600(0x%x)\n", consys_ver_id);
			WMT_PLAT_PR_INFO("0x1800216c(0x%x)\n",
					CONSYS_REG_READ(conn_reg.mcu_base + 0x16c));
			WMT_PLAT_PR_INFO("0x18007104(0x%x)\n",
					CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base +
					CONSYS_CPUPCR_OFFSET));
			msleep(20);
			cnt++;
		}
	}
}

static VOID consys_hw_spm_clk_gating_enable(VOID)
{
}

static INT32 consys_hw_power_ctrl(MTK_WCN_BOOL enable)
{
#if CONSYS_PWR_ON_OFF_API_AVAILABLE
	INT32 iRet = 0;
#else
	INT32 value = 0;
	INT32 i = 0;
#endif

	if (enable) {
#if CONSYS_PWR_ON_OFF_API_AVAILABLE
		iRet = clk_prepare_enable(clk_infracfg_ao_ccif4_ap_cg);
		if (iRet) {
			WMT_PLAT_PR_ERR("clk_prepare_enable(clk_infracfg_ao_ccif4_ap_cg) fail(%d)\n", iRet);
			return iRet;
		}
		WMT_PLAT_PR_DBG("clk_prepare_enable(clk_infracfg_ao_ccif4_ap_cg) ok\n");

		iRet = clk_prepare_enable(clk_scp_conn_main);
		if (iRet) {
			WMT_PLAT_PR_ERR("clk_prepare_enable(clk_scp_conn_main) fail(%d)\n", iRet);
			return iRet;
		}
		WMT_PLAT_PR_DBG("clk_prepare_enable(clk_scp_conn_main) ok\n");

		if (conn_reg.infra_ao_pericfg_base != 0) {
			WMT_PLAT_PR_DBG("CON_STA_REG = %x\n",
				CONSYS_REG_READ(
					conn_reg.infra_ao_pericfg_base +
					INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG));

			/* Set CON_PWR_ON bit (CON_STA_REG[0]) */
			CONSYS_REG_WRITE_RANGE((conn_reg.infra_ao_pericfg_base +
				INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG),
				1, 1, 0);

			WMT_PLAT_PR_DBG("CON_STA_REG = %x\n",
				CONSYS_REG_READ(
					conn_reg.infra_ao_pericfg_base +
					INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG));
		}
#else
		/* turn on SPM clock gating enable PWRON_CONFG_EN 0x10006000 32'h0b160001 */
		CONSYS_REG_WRITE((conn_reg.spm_base + CONSYS_PWRON_CONFG_EN_OFFSET),
				 (CONSYS_REG_READ(conn_reg.spm_base +
				 CONSYS_PWRON_CONFG_EN_OFFSET) &
				 0x0000FFFF) | CONSYS_PWRON_CONFG_EN_VALUE);

		/*write assert "conn_top_on" primary part power on,
		 *set "connsys_on_domain_pwr_on"=1
		 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_SPM_PWR_ON_BIT);
		/*read check "conn_top_on" primary part power status,
		 *check "connsys_on_domain_pwr_ack"=1
		 */
		value = CONSYS_PWR_ON_ACK_BIT &
			CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_OFFSET);
		while (value == 0)
			value = CONSYS_PWR_ON_ACK_BIT &
				CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_OFFSET);
		/*write assert "conn_top_on" secondary part power on,
		 *set "connsys_on_domain_pwr_on_s"=1
		 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_SPM_PWR_ON_S_BIT);
		/*read check "conn_top_on" secondary part power status,
		 *check "connsys_on_domain_pwr_ack_s"=1
		 */
		value = CONSYS_PWR_ON_ACK_S_BIT &
			CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_S_OFFSET);
		while (value == 0)
			value = CONSYS_PWR_ON_ACK_S_BIT &
				CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_S_OFFSET);
		/*write turn on AP-to-CONNSYS bus clock, set "conn_clk_dis"=0 (apply this for bus
		 *clock toggling)
		 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) &
				 ~CONSYS_CLK_CTRL_BIT);
		/*wait 1us*/
		udelay(1);
		/*de-assert "conn_top_on" isolation, set "connsys_iso_en"=0 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) &
				 ~CONSYS_SPM_PWR_ISO_S_BIT);

		/*de-assert CONNSYS S/W reset (TOP RGU CR), set "ap_sw_rst_b"=1 */
		CONSYS_REG_WRITE(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET,
				(CONSYS_REG_READ(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET) &
				 ~CONSYS_SW_RST_BIT) | CONSYS_CPU_SW_RST_CTRL_KEY);

		/*de-assert CONNSYS S/W reset (SPM CR), set "ap_sw_rst_b"=1 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) &
				 CONSYS_SPM_PWR_RST_BIT);

		/*wait 0.5ms*/
		udelay(500);
		/*Turn off AHB RX bus sleep protect (AP2CONN AHB Bus protect)
		 *(apply this for INFRA AHB bus accessing when CONNSYS had been turned on)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AHB_RX_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base +
				 CONSYS_AHB_RX_PROT_EN_OFFSET) & ~CONSYS_AHB_RX_PROT_MASK);
		value = ~CONSYS_AHB_RX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AHB_RX_PROT_STA_OFFSET);
		i = 0;
		while (value == 0 || i > 10) {
			value = ~CONSYS_AP2CONN_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AHB_RX_PROT_STA_OFFSET);
			i++;
		}

		/*Turn off AXI Rx bus sleep protect (CONN2AP AXI Rx Bus protect)
		 *(disable sleep protection when CONNSYS had been turned on)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base +
				 CONSYS_AXI_RX_PROT_EN_OFFSET) & ~CONSYS_AXI_RX_PROT_MASK);
		value = ~CONSYS_AXI_RX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_STA_OFFSET);
		i = 0;
		while (value == 0 || i > 10) {
			value = ~CONSYS_AXI_RX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AXI_RX_PROT_STA_OFFSET);
			i++;
		}

		/*Turn off AXI TX bus sleep protect (CONN2AP AXI Tx Bus protect)
		 *(disable sleep protection when CONNSYS had been turned on)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base +
				 CONSYS_AXI_RX_PROT_EN_OFFSET) & ~CONSYS_AXI_TX_PROT_MASK);
		value = ~CONSYS_AXI_TX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_STA_OFFSET);
		i = 0;
		while (value == 0 || i > 10) {
			value = ~CONSYS_AXI_TX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AXI_RX_PROT_STA_OFFSET);
			i++;
		}

		/*Turn off AHB TX bus sleep protect (AP2CONN AHB Bus protect)
		 *(apply this for INFRA AHB bus accessing when CONNSYS had been turned on)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base +
				 CONSYS_AXI_RX_PROT_EN_OFFSET) & ~CONSYS_AHB_TX_PROT_MASK);
		value = ~CONSYS_AHB_TX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_STA_OFFSET);
		i = 0;
		while (value == 0 || i > 10) {
			value = ~CONSYS_AHB_TX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AXI_RX_PROT_STA_OFFSET);
			i++;
		}
		/*wait 5ms*/
		mdelay(5);
#endif /* CONSYS_PWR_ON_OFF_API_AVAILABLE */
	} else {
#if CONSYS_PWR_ON_OFF_API_AVAILABLE
		if (conn_reg.infra_ao_pericfg_base != 0) {
			WMT_PLAT_PR_DBG("CON_STA_REG = %x\n",
				CONSYS_REG_READ(
					conn_reg.infra_ao_pericfg_base +
					INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG));

			/* Clean CON_STA_REG
			 * when power off or reset connsys
			 */
			CONSYS_REG_WRITE((conn_reg.infra_ao_pericfg_base +
				INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG), 0);

			WMT_PLAT_PR_DBG("CON_STA_REG = %x\n",
				CONSYS_REG_READ(
					conn_reg.infra_ao_pericfg_base +
					INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG));
		}

		clk_disable_unprepare(clk_scp_conn_main);
		WMT_PLAT_PR_DBG("clk_disable_unprepare(clk_scp_conn_main) calling\n");

		/* Clean CCIF4 ACK status */
		/* Wait 100us to make sure all ongoing tx interrupt could be
		 * reset.
		 * According to profiling result, the time between read
		 * register and send tx interrupt is less than 20 us.
		 */
		udelay(100);
		if (conn_reg.ap_pccif4_base != 0) {
			CONSYS_REG_WRITE((conn_reg.ap_pccif4_base +
				INFRASYS_COMMON_AP2MD_PCCIF4_AP_PCCIF_ACK_OFFSET),
				0xFF);
			WMT_PLAT_PR_DBG("AP_PCCIF_ACK = %x\n",
				CONSYS_REG_READ(
					conn_reg.ap_pccif4_base +
					INFRASYS_COMMON_AP2MD_PCCIF4_AP_PCCIF_ACK_OFFSET));
		}

		clk_disable_unprepare(clk_infracfg_ao_ccif4_ap_cg);
		WMT_PLAT_PR_DBG("clk_disable_unprepare(clk_infracfg_ao_ccif4_ap_cg) calling\n");

#else
		/* Turn on AHB bus sleep protect (AP2CONN AHB Bus protect)
		 * (apply this for INFRA AXI bus protection to prevent bus hang
		 * when CONNSYS had been turned off)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AHBAXI_PROT_EN_OFFSET,
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AHBAXI_PROT_EN_OFFSET) &
				 CONSYS_AP2CONN_PROT_MASK);
		value = CONSYS_AP2CONN_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AHBAXI_PROT_STA_OFFSET);
		i = 0;
		while (value == CONSYS_AP2CONN_PROT_MASK || i > 10) {
			value = CONSYS_AP2CONN_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AHBAXI_PROT_STA_OFFSET);
			i++;
		}
		/* Turn on AXI Tx bus sleep protect (CONN2AP AXI Tx Bus protect)
		 * (apply this for INFRA AXI bus protection to prevent bus hang
		 * when CONNSYS had been turned off)
		 * Note : Should turn on AXI Tx sleep protection first.
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AHBAXI_PROT_EN_OFFSET,
				CONSYS_REG_READ(conn_reg.topckgen_base +
				 CONSYS_AHBAXI_PROT_EN_OFFSET) &
				CONSYS_TX_PROT_MASK);
		value = CONSYS_TX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AHBAXI_PROT_STA_OFFSET);
		i = 0;
		while (value == CONSYS_TX_PROT_MASK || i > 10) {
			value = CONSYS_TX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AHBAXI_PROT_STA_OFFSET);
			i++;
		}
		/* Turn on AXI Rx bus sleep protect (CONN2AP AXI RX Bus protect)
		 * (apply this for INFRA AXI bus protection to prevent bus hang
		 * when CONNSYS had been turned off)
		 * Note : Should turn on AXI Rx sleep protection
		 * after AXI Tx sleep protection has been turn on.
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AHBAXI_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base +
				 CONSYS_AHBAXI_PROT_EN_OFFSET) &
				 CONSYS_RX_PROT_MASK);
		value = CONSYS_RX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AHBAXI_PROT_STA_OFFSET);
		i = 0;
		while (value == CONSYS_RX_PROT_MASK || i > 10) {
			value = CONSYS_RX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AHBAXI_PROT_STA_OFFSET);
			i++;
		}

		/*assert "conn_top_on" isolation, set "connsys_iso_en"=1 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_SPM_PWR_ISO_S_BIT);
		/*assert CONNSYS S/W reset (TOP RGU CR), set "ap_sw_rst_b"=0 */
		CONSYS_REG_WRITE(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET,
				(CONSYS_REG_READ(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET) &
				 CONSYS_SW_RST_BIT) | CONSYS_CPU_SW_RST_CTRL_KEY);
		/*assert CONNSYS S/W reset (SPM CR), set "ap_sw_rst_b"=0 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 ~CONSYS_SPM_PWR_RST_BIT);
		/*write conn_clk_dis=1, disable connsys clock  0x1000632C [4]  1'b1 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_CLK_CTRL_BIT);
		/*wait 1us      */
		udelay(1);
		/*write conn_top1_pwr_on=0, power off conn_top1 0x1000632C [3:2] 2'b00 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) &
				 ~(CONSYS_SPM_PWR_ON_BIT | CONSYS_SPM_PWR_ON_S_BIT));
#endif /* CONSYS_PWR_ON_OFF_API_AVAILABLE */
	}

#if CONSYS_PWR_ON_OFF_API_AVAILABLE
	return iRet;
#else
	return 0;
#endif
}

static INT32 consys_ahb_clock_ctrl(MTK_WCN_BOOL enable)
{
	return 0;
}

static INT32 polling_consys_chipid(VOID)
{
	UINT32 retry = 10;
	UINT32 consys_ver_id = 0;
	UINT32 consys_hw_ver = 0;
	UINT32 consys_fw_ver = 0;
	UINT8 *consys_reg_base = NULL;
	UINT32 value = 0;

	if ((conn_reg.mcu_top_misc_off_base == 0) || (conn_reg.mcu_base == 0) ||
			(conn_reg.mcu_conn_hif_pdma_base == 0))
		return 0;

	/*12.poll CONNSYS CHIP ID until chipid is returned */
	while (retry-- > 0) {
		consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_top_misc_off_base +
				CONSYS_IP_VER_OFFSET);
		if (consys_ver_id == CONSYS_IP_VER_ID) {
			WMT_PLAT_PR_INFO("retry(%d)consys version id(0x%08x)\n",
					retry, consys_ver_id);
			consys_hw_ver = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HW_ID_OFFSET);
			WMT_PLAT_PR_INFO("consys HW version id(0x%x)\n", consys_hw_ver & 0xFFFF);
			consys_fw_ver = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_FW_ID_OFFSET);
			WMT_PLAT_PR_INFO("consys FW version id(0x%x)\n", consys_fw_ver & 0xFFFF);

			consys_dl_rom_patch(consys_ver_id, consys_fw_ver);
			break;
		}
		WMT_PLAT_PR_ERR("Read CONSYS version id(0x%08x)", consys_ver_id);
		msleep(20);
	}
	consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_top_misc_off_base + CONSYS_CONF_ID_OFFSET);
	WMT_PLAT_PR_INFO("consys configuration id(0x%x)\n", consys_ver_id & 0xF);
	consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HW_ID_OFFSET);
	WMT_PLAT_PR_INFO("consys HW version id(0x%x)\n", consys_ver_id & 0xFFFF);
	consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_FW_ID_OFFSET);
	WMT_PLAT_PR_INFO("consys FW version id(0x%x)\n", consys_ver_id & 0xFFFF);

	if (wmt_plat_soc_co_clock_flag_get()) {
		consys_reg_base = ioremap_nocache(CONSYS_COCLOCK_STABLE_TIME_BASE, 0x100);
		if (consys_reg_base) {
			/**
			 * 1. set CR "vcore ready stable time" "XO initial stable time" and
			 *    "XO bg stable time" to optimize the wakeup time after sleep
			 * 2. clear "source clock enable ack to XO state" mask
			 */
			value = CONSYS_REG_READ(consys_reg_base);
			value = (value & CONSYS_COCLOCK_STABLE_TIME_MASK) |
					CONSYS_COCLOCK_STABLE_TIME;
			CONSYS_REG_WRITE(consys_reg_base, value);
			value = CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_ACK_ENABLE_OFFSET);
			value = (value & 0xffff00fe) | 0x600;
			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_ACK_ENABLE_OFFSET, value);
			iounmap(consys_reg_base);
		} else
			WMT_PLAT_PR_ERR("connsys co_clock stable time base(0x%x) ioremap fail!\n",
					  CONSYS_COCLOCK_STABLE_TIME_BASE);
	}

	/* CONN2EMI HW slpprot control enable: 0x18004160[4] 1'b1 */
	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_pdma_base + CONSYS_SLPPROT_CONTROL_OFFSET,
			(CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base +
			CONSYS_SLPPROT_CONTROL_OFFSET) &
			CONSYS_SLPPROT_CONTROL_MASK) | CONSYS_SLPPROT_CONTROL_VALUE);

	/* toppose_restore_done rollback */
	consys_reg_base = ioremap_nocache(CONSYS_TOPPOSE_RESTORE_ADDRESS, 0x100);
	if (consys_reg_base) {
		CONSYS_REG_WRITE(consys_reg_base,
				(CONSYS_REG_READ(consys_reg_base) &
				CONSYS_TOPPOSE_RESTORE_MASK) | CONSYS_TOPPOSE_RESTORE_VALUE);
		iounmap(consys_reg_base);
	}

	/* update WPLL setting for WPLL issue@LT */
	consys_reg_base = ioremap_nocache(CONSYS_WPLL_SETTING_ADDRESS, 0x100);
	if (consys_reg_base) {
		CONSYS_REG_WRITE(consys_reg_base,
				(CONSYS_REG_READ(consys_reg_base) &
				CONSYS_WPLL_SETTING_MASK) | CONSYS_WPLL_SETTING_VALUE);
		iounmap(consys_reg_base);
	}

	return 0;
}

static VOID consys_acr_reg_setting(VOID)
{
	WMT_PLAT_PR_INFO("No need to do acr");
}

static VOID consys_afe_reg_setting(VOID)
{
	WMT_PLAT_PR_INFO("No need to do afe");
}

static INT32 consys_hw_vcn18_ctrl(MTK_WCN_BOOL enable)
{
#if CONSYS_PMIC_CTRL_ENABLE
	if (enable) {
		/*Set VCN18_SW_OP_EN as 1*/
		KERNEL_pmic_ldo_vcn18_lp(SW, 1, 1, SW_OFF);
		/*Set VCN18_SW_EN as 1 and set votage as 1V8*/
		if (reg_VCN18) {
			regulator_set_voltage(reg_VCN18, 1800000, 1800000);
			if (regulator_enable(reg_VCN18))
				WMT_PLAT_PR_ERR("enable VCN18 fail\n");
			else
				WMT_PLAT_PR_DBG("enable VCN18 ok\n");
		}
		/*Set VCN18 SW_LP as 0*/
		KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN18_LP, 0);
		/*Set VCN18 as low-power mode(1), HW0_OP_EN as 1, HW0_OP_CFG as HW_LP(1)*/
		KERNEL_pmic_ldo_vcn18_lp(SRCLKEN0, 1, 1, HW_LP);

		/*1.AP power on VCN_3V3 LDO (with PMIC_WRAP API) VCN_3V3  */
		/*default vcn33_1 SW mode*/
		if (reg_VCN33_1) {
			regulator_set_voltage(reg_VCN33_1, 3300000, 3300000);
			if (regulator_enable(reg_VCN33_1))
				WMT_PLAT_PR_ERR("WMT do VCN33_1 PMIC on fail!\n");
		}

	} else {
		if (reg_VCN33_1)
			regulator_disable(reg_VCN33_1);
		/* delay 300us */
		udelay(300);
		/*AP power off MT6359 VCN_1V8 LDO */
		if (reg_VCN18) {
			if (regulator_disable(reg_VCN18))
				WMT_PLAT_PR_ERR("disable VCN_1V8 fail!\n");
			else
				WMT_PLAT_PR_DBG("disable VCN_1V8 ok\n");
		}
	}
#endif
	return 0;
}

static VOID consys_vcn28_hw_mode_ctrl(UINT32 enable)
{
#if CONSYS_PMIC_CTRL_ENABLE
	if (enable)
		/*Set VCN33_2_SW_OP_EN as 1*/
		KERNEL_pmic_ldo_vcn33_2_lp(SW, 1, 1, SW_OFF);
	else
		/*Set VCN33_2 as low-power mode(1), HW0_OP_EN as 0, HW0_OP_CFG as HW_OFF(0)*/
		KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN0, 1, 0, HW_OFF);
#endif
}

static INT32 consys_hw_vcn28_ctrl(UINT32 enable)
{
#if CONSYS_PMIC_CTRL_ENABLE
	INT32 result;

	if (enable) {
		/*in co-clock mode,need to turn on vcn33_2 when fm on */
		/*Set VCN33_2_SW_OP_EN as 1*/
		KERNEL_pmic_ldo_vcn33_2_lp(SW, 1, 1, SW_OFF);
		/*Set VCN33_2_SW_EN as 1 and set votage as 2V8*/
		if (reg_VCN33_2) {
			regulator_set_voltage(reg_VCN33_2, 2800000, 2800000);
			result = regulator_enable(reg_VCN33_2);
		}
		/*Set VCN33_2 SW_LP as 0*/
		KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_2_LP, 0);
		/*Set VCN33_2 as low-power mode(1), HW0_OP_EN as 1, HW0_OP_CFG as HW_OFF(0)*/
		KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN0, 1, 1, HW_OFF);
		WMT_PLAT_PR_INFO("turn on vcn33_2 for fm/gps usage in co-clock mode\n");
	} else {
		/*in co-clock mode,need to turn off vcn33_2 when fm off */
		/*Set VCN33_2 as low-power mode(1), HW0_OP_EN as 0, HW0_OP_CFG as HW_OFF(0)*/
		KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN0, 1, 0, HW_OFF);
		if (reg_VCN33_2)
			regulator_disable(reg_VCN33_2);
		WMT_PLAT_PR_INFO("turn off vcn33_2 for fm/gps usage in co-clock mode\n");
	}
#endif
	return 0;
}

static INT32 consys_hw_bt_vcn33_ctrl(UINT32 enable)
{
	return 0;
}

static INT32 consys_hw_wifi_vcn33_ctrl(UINT32 enable)
{
	return 0;
}

static INT32 consys_emi_mpu_set_region_protection(VOID)
{
#ifdef CONFIG_MTK_EMI
	struct emi_region_info_t region_info;

	/*set MPU for EMI share Memory */
	WMT_PLAT_PR_INFO("setting MPU for EMI share memory\n");

	region_info.start = gConEmiPhyBase;
	region_info.end = gConEmiPhyBase + gConEmiSize - 1;
	region_info.region = 27;
	SET_ACCESS_PERMISSION(region_info.apc, LOCK,
			FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
			FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
			NO_PROTECTION, FORBIDDEN, NO_PROTECTION);
	emi_mpu_set_protection(&region_info);
#endif
	return 0;
}

static UINT32 consys_emi_set_remapping_reg(VOID)
{
	/* For direct path */
	phys_addr_t mdPhy = 0;
	INT32 size = 0;

	mtk_wcn_emi_addr_info.emi_ap_phy_addr = gConEmiPhyBase;
	mtk_wcn_emi_addr_info.emi_size = gConEmiSize;

	if (conn_reg.topckgen_base == 0)
		return 0;

	/*EMI Registers remapping*/
	CONSYS_REG_WRITE_OFFSET_RANGE(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET,
					  gConEmiPhyBase, 0, 16, 20);
	WMT_PLAT_PR_INFO("CONSYS_EMI_MAPPING dump in restore cb(0x%08x)\n",
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET));

	/*Perisys Configuration Registers remapping*/
	CONSYS_REG_WRITE_OFFSET_RANGE(conn_reg.topckgen_base + CONSYS_EMI_PERI_MAPPING_OFFSET,
					  0x10003000, 0, 16, 20);
	WMT_PLAT_PR_INFO("PERISYS_MAPPING dump in restore cb(0x%08x)\n",
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_PERI_MAPPING_OFFSET));

	/*Modem Configuration Registers remapping*/
#ifdef CONFIG_MTK_ECCCI_DRIVER
	mdPhy = get_smem_phy_start_addr(MD_SYS1, SMEM_USER_RAW_MD_CONSYS, &size);
#endif
	if (size == 0)
		WMT_PLAT_PR_INFO("MD direct path is not supported\n");
	else {
		CONSYS_REG_WRITE_OFFSET_RANGE(
			conn_reg.topckgen_base + CONSYS_EMI_AP_MD_OFFSET,
			mdPhy, 0, 16, 20);
		WMT_PLAT_PR_INFO("MD_MAPPING dump in restore cb(0x%08x)\n",
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_AP_MD_OFFSET));
	}
	mtk_wcn_emi_addr_info.emi_direct_path_ap_phy_addr = mdPhy;
	mtk_wcn_emi_addr_info.emi_direct_path_size = size;

	mtk_wcn_emi_addr_info.emi_ram_bt_buildtime_offset =
			CONSYS_EMI_RAM_BT_BUILDTIME_OFFSET;
	mtk_wcn_emi_addr_info.emi_ram_wifi_buildtime_offset =
			CONSYS_EMI_RAM_WIFI_BUILDTIME_OFFSET;
	mtk_wcn_emi_addr_info.emi_ram_mcu_buildtime_offset =
			CONSYS_EMI_RAM_MCU_BUILDTIME_OFFSET;
	mtk_wcn_emi_addr_info.emi_patch_mcu_buildtime_offset =
			CONSYS_EMI_PATCH_MCU_BUILDTIME_OFFSET;

	return 0;
}

static INT32 bt_wifi_share_v33_spin_lock_init(VOID)
{
	return 0;
}

static INT32 consys_read_irq_info_from_dts(struct platform_device *pdev, PINT32 irq_num,
		PUINT32 irq_flag)
{
	struct device_node *node;

	node = pdev->dev.of_node;
	if (node) {
		*irq_num = irq_of_parse_and_map(node, 0);
		*irq_flag = irq_get_trigger_type(*irq_num);
		WMT_PLAT_PR_INFO("get irq id(%d) and irq trigger flag(%d) from DT\n", *irq_num,
				   *irq_flag);
	} else {
		WMT_PLAT_PR_ERR("[%s] can't find CONSYS compatible node\n", __func__);
		return -1;
	}

	return 0;
}

static INT32 consys_read_reg_from_dts(struct platform_device *pdev)
{
	INT32 iRet = -1;
	struct device_node *node = NULL;

	node = pdev->dev.of_node;
	if (node) {
		/* registers base address */
		conn_reg.mcu_base = (SIZE_T) of_iomap(node, MCU_BASE_INDEX);
		conn_reg.ap_rgu_base = (SIZE_T) of_iomap(node, TOP_RGU_BASE_INDEX);
		conn_reg.topckgen_base = (SIZE_T) of_iomap(node, INFRACFG_AO_BASE_INDEX);
		conn_reg.spm_base = (SIZE_T) of_iomap(node, SPM_BASE_INDEX);
		conn_reg.mcu_conn_hif_on_base = (SIZE_T) of_iomap(node, MCU_CONN_HIF_ON_BASE_INDEX);
		conn_reg.mcu_top_misc_off_base = (SIZE_T) of_iomap(node,
				MCU_TOP_MISC_OFF_BASE_INDEX);
		conn_reg.mcu_cfg_on_base = (SIZE_T) of_iomap(node, MCU_CFG_ON_BASE_INDEX);
		conn_reg.mcu_cirq_base = (SIZE_T) of_iomap(node, MCU_CIRQ_BASE_INDEX);
		conn_reg.mcu_top_misc_on_base = (SIZE_T) of_iomap(node, MCU_TOP_MISC_ON_BASE_INDEX);
		conn_reg.mcu_conn_hif_pdma_base = (SIZE_T) of_iomap(node,
				MCU_CONN_HIF_PDMA_BASE_INDEX);
		conn_reg.ap_pccif4_base = (SIZE_T) of_iomap(node, AP_PCCIF4_BASE_INDEX);
		conn_reg.infra_ao_pericfg_base = (SIZE_T) of_iomap(node,
				INFRA_AO_PERICFG_BASE_INDEX);

		WMT_PLAT_PR_DBG("Get base mcu(0x%zx), rgu(0x%zx), topckgen(0x%zx), spm(0x%zx)\n",
				conn_reg.mcu_base, conn_reg.ap_rgu_base,
				conn_reg.topckgen_base, conn_reg.spm_base);
		WMT_PLAT_PR_DBG("Get base hif_on(0x%zx), misc_off(0x%zx), cfg_on(0x%zx)\n",
				conn_reg.mcu_top_misc_off_base,
				conn_reg.mcu_conn_hif_on_base,
				conn_reg.mcu_cfg_on_base);
		WMT_PLAT_PR_DBG("Get base cirq(0x%zx), top_misc_on(0x%zx), hif_pdma(0x%zx)\n",
				conn_reg.mcu_top_misc_on_base,
				conn_reg.mcu_cirq_base,
				conn_reg.mcu_conn_hif_pdma_base);
		WMT_PLAT_PR_DBG("Get base ap_pccif4(0x%zx), infra_ao_pericfg(0x%zx)\n",
				conn_reg.ap_pccif4_base,
				conn_reg.infra_ao_pericfg_base);
	} else {
		WMT_PLAT_PR_ERR("[%s] can't find CONSYS compatible node\n", __func__);
		return iRet;
	}

	return 0;
}

static VOID force_trigger_assert_debug_pin(VOID)
{
	UINT32 value = 0;

	if (conn_reg.infra_ao_pericfg_base != 0) {
		WMT_PLAT_PR_DBG("CON_STA_REG = %x\n",
			CONSYS_REG_READ(
				conn_reg.infra_ao_pericfg_base +
				INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG));

		/* clear CON_PWR_ON & CON_SW_READY bit (CON_STA_REG[0], CON_STA_REG[1]) */
		value = CONSYS_REG_READ(conn_reg.infra_ao_pericfg_base +
			INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG);
		value = value & (~INFRASYS_COMMON_AP2MD_CON_PWR_ON_CON_SW_READY_MASK);
		CONSYS_REG_WRITE(conn_reg.infra_ao_pericfg_base +
			INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG,
			value);

		WMT_PLAT_PR_DBG("CON_STA_REG = %x\n",
			CONSYS_REG_READ(
				conn_reg.infra_ao_pericfg_base +
				INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG));
	}

	if (conn_reg.topckgen_base == 0)
		return;

	CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AP2CONN_OSC_EN_OFFSET,
			CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AP2CONN_OSC_EN_OFFSET) & ~CONSYS_AP2CONN_WAKEUP_BIT);
	WMT_PLAT_PR_INFO("enable:dump CONSYS_AP2CONN_OSC_EN_REG(0x%x)\n",
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AP2CONN_OSC_EN_OFFSET));
	usleep_range(64, 96);
	CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AP2CONN_OSC_EN_OFFSET,
			CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AP2CONN_OSC_EN_OFFSET) | CONSYS_AP2CONN_WAKEUP_BIT);
	WMT_PLAT_PR_INFO("disable:dump CONSYS_AP2CONN_OSC_EN_REG(0x%x)\n",
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AP2CONN_OSC_EN_OFFSET));
}

static UINT32 consys_read_cpupcr(VOID)
{
	if (conn_reg.mcu_conn_hif_on_base == 0)
		return 0;

	return CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONSYS_CPUPCR_OFFSET);
}

static INT32 consys_poll_cpupcr_dump(UINT32 times, UINT32 sleep_ms)
{
	UINT64 ts;
	ULONG nsec;
	INT32 str_len = 0, i;
	char str[DBG_LOG_STR_SIZE] = {""};
	unsigned int remain = DBG_LOG_STR_SIZE;
	char *p = NULL;

	p = str;
	for (i = 0; i < times; i++) {
		osal_get_local_time(&ts, &nsec);
		str_len = snprintf(p, remain, "%llu.%06lu/0x%08x;", ts, nsec,
								consys_read_cpupcr());
		if (str_len < 0) {
			WMT_PLAT_PR_WARN("%s snprintf fail", __func__);
			continue;
		}
		p += str_len;
		remain -= str_len;

		if (sleep_ms > 0)
			osal_sleep_ms(sleep_ms);
	}
	WMT_PLAT_PR_INFO("TIME/CPUPCR: %s", str);
	return 0;
}

static UINT32 consys_soc_chipid_get(VOID)
{
	return PLATFORM_SOC_CHIP;
}

static P_CONSYS_EMI_ADDR_INFO consys_soc_get_emi_phy_add(VOID)
{
	return &mtk_wcn_emi_addr_info;
}

static INT32 consys_dl_rom_patch(UINT32 ip_ver, UINT32 fw_ver)
{
	if (rom_patch_dl_flag) {
		if (mtk_wcn_soc_rom_patch_dwn(ip_ver, fw_ver) == 0)
			rom_patch_dl_flag = 1;
	}

	return 0;
}

static VOID consys_set_dl_rom_patch_flag(INT32 flag)
{
	rom_patch_dl_flag = flag;
}

static INT32 consys_dedicated_log_path_init(struct platform_device *pdev)
{
	struct device_node *node;
	UINT32 irq_num;
	UINT32 irq_flag;
	INT32 iret = -1;
	struct connlog_irq_config irq_config;

	memset(&irq_config, 0, sizeof(struct connlog_irq_config));
	node = pdev->dev.of_node;
	if (node) {
		irq_num = irq_of_parse_and_map(node, 2);
		irq_flag = irq_get_trigger_type(irq_num);
		WMT_PLAT_PR_INFO("get conn2ap_sw_irq id(%d) and irq trigger flag(%d) from DT\n",
				irq_num, irq_flag);
	} else {
		WMT_PLAT_PR_ERR("[%s] can't find CONSYS compatible node\n", __func__);
		return iret;
	}

	irq_config.irq_num = irq_num;
	irq_config.irq_flag = irq_flag;
	irq_config.irq_callback = NULL;

	connsys_dedicated_log_path_apsoc_init(
		gConEmiPhyBase, &connsys_fw_log_parameter, &irq_config);
#ifdef CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH
	fw_log_wmt_init();
#endif
	return 0;
}

static VOID consys_dedicated_log_path_deinit(VOID)
{
#ifdef CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH
	fw_log_wmt_deinit();
#endif
	connsys_dedicated_log_path_apsoc_deinit();
}

static INT32 consys_check_reg_readable(VOID)
{
	INT32 can_read = 0;
	UINT32 value = 0;

	if (conn_reg.mcu_cfg_on_base != 0 &&
	    conn_reg.mcu_top_misc_on_base != 0) {
		/*check connsys clock and sleep status*/
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base, CONSYS_CLOCK_CHECK_VALUE);
		udelay(1000);
		value = CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base);
		if ((value & CONSYS_HCLK_CHECK_BIT) &&
		    (value & CONSYS_OSCCLK_CHECK_BIT))
			can_read = 1;
	}

	if (!can_read)
		WMT_PLAT_PR_ERR("connsys clock check fail 0x18007000(0x%x)\n", value);

	return can_read;
}

static INT32 consys_emi_coredump_remapping(UINT8 __iomem **addr, UINT32 enable)
{
	if (enable) {
		*addr = ioremap_nocache(gConEmiPhyBase + CONSYS_EMI_COREDUMP_OFFSET,
				CONSYS_EMI_MEM_SIZE);
		if (*addr) {
			WMT_PLAT_PR_INFO("COREDUMP EMI mapping OK virtual(0x%p) physical(0x%x)\n",
					*addr, (UINT32) gConEmiPhyBase +
					CONSYS_EMI_COREDUMP_OFFSET);
			memset_io(*addr, 0, CONSYS_EMI_MEM_SIZE);
		} else {
			WMT_PLAT_PR_ERR("EMI mapping fail\n");
			return -1;
		}
	} else {
		if (*addr) {
			iounmap(*addr);
			*addr = NULL;
		}
	}
	return 0;
}

static INT32 consys_reset_emi_coredump(UINT8 __iomem *addr)
{
	if (!addr) {
		WMT_PLAT_PR_ERR("get virtual address fail\n");
		return -1;
	}
	WMT_PLAT_PR_INFO("Reset EMI(0xF0068000 ~ 0xF0068400) and (0xF0070400 ~ 0xF0078400)\n");
	/* reset 0xF0068000 ~ 0xF0068400 (1K) */
	memset_io(addr, 0, 0x400);
	/* reset 0xF0070400 ~ 0xF0078400 (32K)  */
	memset_io(addr + CONSYS_EMI_PAGED_DUMP_OFFSET, 0, 0x8000);
	return 0;
}

static INT32 consys_is_connsys_reg(UINT32 addr)
{
	if (addr > 0x18000000 && addr < 0x180FFFFF) {
		if (addr >= 0x18007000 && addr <= 0x18007FFF)
			return 0;

		return 1;
	}

	return 0;
}

static INT32 consys_is_host_csr(SIZE_T addr)
{
	SIZE_T start_offset = 0x000;
	SIZE_T end_offset = 0xFFF;

	if (addr >= (CONN_HIF_ON_BASE_ADDR + start_offset) &&
		addr <= (CONN_HIF_ON_BASE_ADDR + end_offset))
		return 1;

	if (conn_reg.mcu_conn_hif_on_base != 0 &&
		addr >= (conn_reg.mcu_conn_hif_on_base + start_offset) &&
		addr <= (conn_reg.mcu_conn_hif_on_base + end_offset))
		return 1;

	return 0;
}

static VOID consys_ic_clock_fail_dump(VOID)
{
	UINT8 *addr;
	char *buffer, *temp;
	INT32 size = 1024;

	if (conn_reg.mcu_base == 0)
		return;

	/* make sure buffer size is big enough */
	buffer = osal_malloc(size);
	if (!buffer)
		return;

	temp = buffer;
	temp += sprintf(temp, "CONN_HIF_TOP_MISC=0x%08x CONN_HIF_BUSY_STATUS=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_TOP_MISC),
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_BUSY_STATUS));

	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_HIF_DBG_IDX, 0x3333);
	temp += sprintf(temp, "Write CONSYS_HIF_DBG_IDX to 0x3333\n");

	temp += sprintf(temp, "CONSYS_HIF_DBG_PROBE=0x%08x CONN_HIF_TOP_MISC=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_DBG_PROBE),
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_TOP_MISC));

	temp += sprintf(temp, "CONN_HIF_BUSY_STATUS=0x%08x CONN_HIF_PDMA_BUSY_STATUS=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_BUSY_STATUS),
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_PDMA_BUSY_STATUS));

	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_HIF_DBG_IDX, 0x2222);
	temp += sprintf(temp, "Write CONSYS_HIF_DBG_IDX to 0x2222\n");

	temp += sprintf(temp, "CONSYS_HIF_DBG_PROBE=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_DBG_PROBE));

	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_HIF_DBG_IDX, 0x3333);
	temp += sprintf(temp, "Write CONSYS_HIF_DBG_IDX to 0x3333\n");

	temp += sprintf(temp, "CONSYS_HIF_DBG_PROBE=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_DBG_PROBE));

	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_HIF_DBG_IDX, 0x4444);
	temp += sprintf(temp, "Write CONSYS_HIF_DBG_IDX to 0x4444\n");

	temp += sprintf(temp, "CONSYS_HIF_DBG_PROBE=0x%08x CONN_MCU_EMI_CONTROL=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HIF_DBG_PROBE),
		CONSYS_REG_READ(conn_reg.mcu_base + CONN_MCU_EMI_CONTROL));
	temp += sprintf(temp, "EMI_CONTROL_DBG_PROBE=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + EMI_CONTROL_DBG_PROBE));
	temp += sprintf(temp, "CONN_MCU_CLOCK_CONTROL=0x%08x CONN_MCU_BUS_CONTROL=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_CLOCK_CONTROL),
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_BUS_CONTROL));

	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_DEBUG_SELECT, 0x003e3d00);
	temp += sprintf(temp, "Write CONSYS_DEBUG_SELECT to 0x003e3d00\n");

	temp += sprintf(temp, "CONN_MCU_DEBUG_STATUS=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_DEBUG_STATUS));

	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_DEBUG_SELECT, 0x00403f00);
	temp += sprintf(temp, "Write CONSYS_DEBUG_SELECT to 0x00403f00\n");

	temp += sprintf(temp, "CONN_MCU_DEBUG_STATUS=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_DEBUG_STATUS));

	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_DEBUG_SELECT, 0x00424100);
	temp += sprintf(temp, "Write CONSYS_DEBUG_SELECT to 0x00424100\n");

	temp += sprintf(temp, "CONN_MCU_DEBUG_STATUS=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_DEBUG_STATUS));

	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_DEBUG_SELECT, 0x00444300);
	temp += sprintf(temp, "Write CONSYS_DEBUG_SELECT to 0x00444300\n");

	temp += sprintf(temp, "CONN_MCU_DEBUG_STATUS=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_DEBUG_STATUS));
	WMT_PLAT_PR_ERR("%s length = %d", buffer, osal_strlen(buffer));

	temp = buffer;
	addr = ioremap_nocache(0x180bc000, 0x100);
	/* conn2ap axi master sleep prot info */
	temp += sprintf(temp, "0x180bc010=0x%08x\n", CONSYS_REG_READ(addr + 0x10));
	/* conn_mcu2ap axi master sleep prot info */
	temp += sprintf(temp, "0x180bc014=0x%08x\n", CONSYS_REG_READ(addr + 0x14));
	/* conn2ap axi gals bus info */
	temp += sprintf(temp, "0x180bc018=0x%08x\n", CONSYS_REG_READ(addr + 0x18));
	/* conn2ap mux4to1 debug info */
	temp += sprintf(temp, "0x180bc01c=0x%08x\n", CONSYS_REG_READ(addr + 0x1c));
	/* conn_hif_off bus busy info */
	temp += sprintf(temp, "0x180bc020=0x%08x\n", CONSYS_REG_READ(addr + 0x20));
	iounmap(addr);

	addr = ioremap_nocache(0x10001B20, 0x100);
	/* 0x1020E804 */
	temp += sprintf(temp, "0x10001B20=0x%08x\n", CONSYS_REG_READ(addr));

	addr = ioremap_nocache(0x1800713c, 0x100);
	/* conn_hif_on misc info */
	temp += sprintf(temp, "0x1800713c=0x%08x\n", CONSYS_REG_READ(addr));
	iounmap(addr);

	addr = ioremap_nocache(0x180c1144, 0x100);
	/* conn_on_host debug flag */
	temp += sprintf(temp, "0x180c1144=0x%08x\n", CONSYS_REG_READ(addr));
	iounmap(addr);

	addr = ioremap_nocache(0x1020E804, 0x100);
	/* 0x1020E804 */
	temp += sprintf(temp, "0x1020E804=0x%08x\n", CONSYS_REG_READ(addr));
	iounmap(addr);

	WMT_PLAT_PR_ERR("%s length = %d", buffer, osal_strlen(buffer));
	osal_free(buffer);
}

static INT32 consys_dump_osc_state(P_CONSYS_STATE state)
{
	CONSYS_REG_WRITE(CONN_CFG_ON_CONN_ON_HOST_MAILBOX_MCU_ADDR, 0x1);
	CONSYS_REG_WRITE(CONN_CFG_ON_CONN_ON_MON_CTL_ADDR, 0x80000001);
	CONSYS_REG_WRITE(CONN_CFG_ON_CONN_ON_MON_SEL0_ADDR, 0x03020100);
	CONSYS_REG_WRITE(CONN_CFG_ON_CONN_ON_MON_SEL1_ADDR, 0x07060504);
	CONSYS_REG_WRITE(CONN_CFG_ON_CONN_ON_MON_SEL2_ADDR, 0x0b0a0908);
	CONSYS_REG_WRITE(CONN_CFG_ON_CONN_ON_MON_SEL3_ADDR, 0x0f0e0d0c);
	CONSYS_REG_WRITE(CONN_CFG_ON_CONN_ON_DBGSEL_ADDR, 0x3);
	state->lp[0] = (UINT32)CONN_CFG_ON_CONN_ON_MON_FLAG_RECORD_MAPPING_AP_ADDR;
	state->lp[1] = CONSYS_REG_READ(CONN_CFG_ON_CONN_ON_MON_FLAG_RECORD_ADDR);
	WMT_PLAT_PR_INFO("0x%08x: 0x%x\n", state->lp[0], state->lp[1]);
	CONSYS_REG_WRITE(CONN_CFG_ON_CONN_ON_HOST_MAILBOX_MCU_ADDR, 0x0);

	return MTK_WCN_BOOL_TRUE;
}

static VOID consys_set_pdma_axi_rready_force_high(UINT32 enable)
{
	if (conn_reg.mcu_conn_hif_pdma_base == 0)
		return;

	if (enable)
		CONSYS_SET_BIT(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_PDMA_AXI_RREADY,
				CONSYS_PDMA_AXI_RREADY_MASK);
	else if ((CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_PDMA_AXI_RREADY) &
		 CONSYS_PDMA_AXI_RREADY_MASK) != 0)
		CONSYS_CLR_BIT(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_PDMA_AXI_RREADY,
			       CONSYS_PDMA_AXI_RREADY_MASK);
}

static VOID consys_set_mcif_emi_mpu_protection(MTK_WCN_BOOL enable)
{
	WMT_PLAT_PR_INFO("No need setup region 23 for this project.\n");
}

static INT32 consys_calibration_backup_restore_support(VOID)
{
	return 1;
}

#if WMT_DEVAPC_DBG_SUPPORT
static VOID consys_devapc_violation_cb(VOID)
{
	/**
	 * Don't use wmt_lib_trigger_assert() because it will invoke vmalloc and then cause KE since
	 * this callback is supposed to be invoked in DEVAPC exception hanlder.
	 */
	wmt_lib_trigger_assert_keyword_delay(WMTDRV_TYPE_WMT, 46, "DEVAPC Violation");
}
#endif

static VOID consyc_register_devapc_cb(VOID)
{
#if WMT_DEVAPC_DBG_SUPPORT
	register_devapc_vio_callback(&devapc_handle);
#endif
}

static UINT64 consys_get_options(VOID)
{
	UINT64 options = OPT_WIFI_LTE_COEX |
			OPT_BT_TSSI_FROM_WIFI_CONFIG_NEW_OPID |
			OPT_COEX_CONFIG_ADJUST |
			OPT_COEX_CONFIG_ADJUST_NEW_FLAG |
			OPT_WIFI_LTE_COEX_TABLE_3 |
			OPT_NORMAL_PATCH_DWN_3 |
			OPT_PATCH_CHECKSUM;
	return options;
}

INT32 dump_conn_mcu_pc_log_wrapper(VOID)
{
	return dump_conn_mcu_pc_log_mt6785("");
}

static INT32 consys_common_dump(const char *trg_str)
{
	int ret = 0;

	ret += dump_conn_debug_dump_mt6785(trg_str);
	ret += dump_conn_mcu_debug_flag_mt6785(trg_str);
	ret += dump_conn_mcu_apb0_bus_mt6785(trg_str);
	ret += dump_conn_mcu_apb1_bus_mt6785(trg_str);
	ret += dump_conn_mcu_pc_log_mt6785(trg_str);
	ret += dump_conn_cfg_on_debug_signal_mt6785(trg_str);
	ret += dump_conn_cfg_on_register_mt6785(trg_str);
	ret += dump_conn_cmdbt_debug_signal_mt6785(trg_str);
	ret += dump_conn_emi_detect_mt6785(trg_str);
	ret += dump_conn_slp_protect_debug_mt6785(trg_str);
	ret += dump_conn_spm_r13_mt6785(trg_str);
	ret += dump_conn_bus_timeout_debug_mt6785(trg_str);
	ret += dump_conn_ILM_corrupt_issue_debug_mt6785(trg_str);

	return ret;
}

INT32 consys_cmd_tx_timeout_dump(VOID)
{
	return consys_common_dump("tx_timeout");
}

INT32 consys_cmd_rx_timeout_dump(VOID)
{
	return consys_common_dump("rx_timeout");
}

INT32 consys_coredump_timeout_dump(VOID)
{
	return consys_common_dump("coredump_timeout");
}

INT32 consys_assert_timeout_dump(VOID)
{
	return consys_common_dump("assert_timeout");
}

INT32 consys_before_chip_reset_dump(VOID)
{
	return consys_common_dump("before_chip_reset");
}
