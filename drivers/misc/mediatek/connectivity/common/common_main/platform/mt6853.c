/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
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

#define CONSYS_ENABLE_EMI_MPU 1
#define	REGION_CONN	27

#define	DOMAIN_AP	0
#define	DOMAIN_CONN	2

#define	PRIMARY_ADIE	0x6631
#define	SECONDARY_ADIE	0x6635

/* if clock of TCXO is controlled by GPIO, CLK_CTRL_TCXOENA_REQ should be 1. */
#define CLK_CTRL_TCXOENA_REQ 0

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
#include "mt6853.h"
#include "mtk_wcn_consys_hw.h"
#include "wmt_ic.h"
#include "wmt_lib.h"
#include "wmt_step.h"
#include "wmt_plat.h"
#include "stp_dbg.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
#include <soc/mediatek/emi.h>
#else
#include <memory/mediatek/emi.h>
#endif

#if CONSYS_PMIC_CTRL_ENABLE
#include <linux/regulator/consumer.h>
#if (COMMON_KERNEL_PMIC_SUPPORT)
#include <linux/mfd/mt6397/core.h>
#include <linux/regmap.h>
#else
#include <mtk_pmic_api_buck.h>
#include <upmu_common.h>
#endif
#endif

#ifdef CONFIG_MTK_HIBERNATION
#include <mtk_hibernate_dpm.h>
#endif

#include <linux/of_reserved_mem.h>

/* Direct path */
#include <mtk_ccci_common.h>
#include <linux/gpio.h>
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
static INT32 consys_hw_vcn_ctrl_after_idle(VOID);
static UINT32 consys_soc_chipid_get(VOID);
static INT32 consys_adie_chipid_detect(VOID);
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
static INT32 consys_emi_coredump_remapping(UINT8 __iomem **addr, UINT32 enable);
static INT32 consys_reset_emi_coredump(UINT8 __iomem *addr);
static INT32 consys_check_reg_readable(VOID);
static VOID consys_ic_clock_fail_dump(VOID);
static INT32 consys_is_connsys_reg(UINT32 addr);
static INT32 consys_is_host_csr(SIZE_T addr);
static INT32 consys_dump_osc_state(P_CONSYS_STATE state);
static VOID consys_set_pdma_axi_rready_force_high(UINT32 enable);
static VOID consys_set_mcif_emi_mpu_protection(MTK_WCN_BOOL enable);
#if (COMMON_KERNEL_CLK_SUPPORT)
static MTK_WCN_BOOL consys_need_store_pdev(VOID);
static UINT32 consys_store_pdev(struct platform_device *pdev);
#endif
/*
 * If 1: this platform supports calibration backup/restore.
 * otherwise: 0
 */
static INT32 consys_calibration_backup_restore_support(VOID);
#if WMT_DEVAPC_DBG_SUPPORT
static VOID consys_devapc_violation_cb(VOID);
#endif
static VOID consyc_register_devapc_cb(VOID);
static INT32 consys_is_rc_mode_enable(VOID);
static VOID consys_emi_entry_address(VOID);
static VOID consys_set_xo_osc_ctrl(VOID);
static VOID consys_identify_adie(VOID);
static VOID consys_wifi_ctrl_setting(VOID);
static VOID consys_bus_timeout_config(VOID);
static VOID consys_set_access_emi_hw_mode(VOID);
static INT32 consys_dump_gating_state(P_CONSYS_STATE state);
static INT32 consys_sleep_info_enable_ctrl(UINT32 enable);
static INT32 consys_sleep_info_read_ctrl(WMT_SLEEP_COUNT_TYPE type, PUINT64 sleep_counter, PUINT64 sleep_timer);
static INT32 consys_sleep_info_clear(VOID);
static VOID consys_conn2ap_sw_irq_clear(VOID);
static UINT64 consys_get_options(VOID);

static INT32 dump_conn_mcu_pc_log_wrapper(VOID);
static INT32 consys_cmd_tx_timeout_dump(VOID);
static INT32 consys_cmd_rx_timeout_dump(VOID);
static INT32 consys_coredump_timeout_dump(VOID);
static INT32 consys_assert_timeout_dump(VOID);
static INT32 consys_before_chip_reset_dump(VOID);
static INT32 consys_ipi_timeout_dump(VOID);

static INT32 consys_jtag_set_for_mcu(VOID);
static UINT32 consys_jtag_flag_ctrl(UINT32 enable);

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/
/* CCF part */
#if (!COMMON_KERNEL_CLK_SUPPORT)
static struct clk *clk_scp_conn_main;	/*ctrl conn_power_on/off */
#endif
static struct clk *clk_infracfg_ao_ccif4_ap_cg;       /* For direct path */

#if (COMMON_KERNEL_CLK_SUPPORT)
static struct platform_device *connsys_pdev;
#endif

/* PMIC part */
#if CONSYS_PMIC_CTRL_ENABLE
static struct regulator *reg_VCN13;
static struct regulator *reg_VCN18;
static struct regulator *reg_VCN33_1_BT;
static struct regulator *reg_VCN33_1_WIFI;
static struct regulator *reg_VCN33_2_WIFI;
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
	.emi_apmem_ctrl_chip_check_sleep = EXP_APMEM_CTRL_CHIP_CHECK_SLEEP,
};

static CONSYS_EMI_ADDR_INFO mtk_wcn_emi_addr_info = {
	.emi_phy_addr = CONSYS_EMI_FW_PHY_BASE,
	.paged_trace_off = CONSYS_EMI_PAGED_TRACE_OFFSET,
	.paged_dump_off = CONSYS_EMI_PAGED_DUMP_OFFSET,
	.full_dump_off = CONSYS_EMI_FULL_DUMP_OFFSET,
	.emi_remap_offset = CONSYS_EMI_MAPPING_OFFSET,
	.p_ecso = &mtk_wcn_emi_state_off,
	.pda_dl_patch_flag = 1,
	.emi_met_size = (32*KBYTE),
	.emi_met_data_offset = CONSYS_EMI_MET_DATA_OFFSET,
	.emi_core_dump_offset = CONSYS_EMI_COREDUMP_OFFSET,
};

WMT_CONSYS_IC_OPS consys_ic_ops_mt6853 = {
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
	.consys_ic_hw_vcn_ctrl_after_idle = consys_hw_vcn_ctrl_after_idle,
	.consys_ic_soc_chipid_get = consys_soc_chipid_get,
	.consys_ic_adie_chipid_detect = consys_adie_chipid_detect,
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
	.consys_ic_emi_coredump_remapping = consys_emi_coredump_remapping,
	.consys_ic_reset_emi_coredump = consys_reset_emi_coredump,
	.consys_ic_check_reg_readable = consys_check_reg_readable,
	.consys_ic_clock_fail_dump = consys_ic_clock_fail_dump,
	.consys_ic_is_connsys_reg = consys_is_connsys_reg,
	.consys_ic_is_host_csr = consys_is_host_csr,
	.consys_ic_dump_osc_state = consys_dump_osc_state,
	.consys_ic_set_pdma_axi_rready_force_high = consys_set_pdma_axi_rready_force_high,
	.consys_ic_set_mcif_emi_mpu_protection = consys_set_mcif_emi_mpu_protection,
	.consys_ic_calibration_backup_restore = consys_calibration_backup_restore_support,
	.consys_ic_register_devapc_cb = consyc_register_devapc_cb,
	.consys_ic_emi_entry_address = consys_emi_entry_address,
	.consys_ic_set_xo_osc_ctrl = consys_set_xo_osc_ctrl,
	.consys_ic_identify_adie = consys_identify_adie,
	.consys_ic_wifi_ctrl_setting = consys_wifi_ctrl_setting,
	.consys_ic_bus_timeout_config = consys_bus_timeout_config,
	.consys_ic_set_access_emi_hw_mode = consys_set_access_emi_hw_mode,
	.consys_ic_dump_gating_state = consys_dump_gating_state,
	.consys_ic_sleep_info_enable_ctrl = consys_sleep_info_enable_ctrl,
	.consys_ic_sleep_info_read_ctrl = consys_sleep_info_read_ctrl,
	.consys_ic_sleep_info_clear = consys_sleep_info_clear,
	.consys_ic_get_options = consys_get_options,

	/* debug dump */
	.consys_ic_cmd_tx_timeout_dump = consys_cmd_tx_timeout_dump,
	.consys_ic_cmd_rx_timeout_dump = consys_cmd_rx_timeout_dump,
	.consys_ic_coredump_timeout_dump = consys_coredump_timeout_dump,
	.consys_ic_assert_timeout_dump = consys_assert_timeout_dump,
	.consys_ic_before_chip_reset_dump = consys_before_chip_reset_dump,
	.consys_ic_ipi_timeout_dump = consys_ipi_timeout_dump,

	.consys_ic_pc_log_dump = dump_conn_mcu_pc_log_wrapper,
#if (COMMON_KERNEL_CLK_SUPPORT)
	.consys_ic_need_store_pdev = consys_need_store_pdev,
	.consys_ic_store_pdev = consys_store_pdev,
#endif

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
static OSAL_UNSLEEPABLE_LOCK g_pwr_off_lock;
static atomic_t g_power_on = ATOMIC_INIT(0);

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
#define JTAG_ADDR2_BASE 0x11c00000
#endif

static INT32 consys_jtag_set_for_mcu(VOID)
{
#if 0
#if CONSYS_ENALBE_SET_JTAG
	INT32 ret = 0;
	UINT32 tmp = 0;
	PVOID addr = 0;
	PVOID remap_addr1 = 0;
	PVOID remap_addr2 = 0;

	if (gJtagCtrl) {
		remap_addr1 = ioremap(JTAG_ADDR1_BASE, 0x1000);
		if (remap_addr1 == 0) {
			WMT_PLAT_PR_INFO("remap jtag_addr1 fail!\n");
			ret = -1;
			goto error;
		}

		remap_addr2 = ioremap(JTAG_ADDR2_BASE, 0x100);
		if (remap_addr2 == 0) {
			WMT_PLAT_PR_INFO("remap jtag_addr2 fail!\n");
			ret = -1;
			goto error;
		}

		WMT_PLAT_PR_INFO("WCN jtag set for mcu start...\n");
		switch (gJtagCtrl) {
		case 1:
			/* 7-wire jtag pinmux setting*/
			/* PAD AUX Function Selection */
			addr = remap_addr1 + 0x360;
			tmp = readl(addr);
			tmp = tmp & 0xfff0f;
			tmp = tmp | 0x66600060;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			addr = remap_addr1 + 0x370;
			tmp = readl(addr);
			tmp = tmp & 0xfffff000;
			tmp = tmp | 0x666;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			/* PAD Driving Selection */
			addr = remap_addr2 + 0x0;
			tmp = readl(addr);
			tmp = tmp & 0xf80001f8;
			tmp = tmp | 0x36db603;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			/* PAD IES */
			addr = remap_addr2 + 0x10;
			tmp = readl(addr);
			tmp = tmp & 0xfffffe06;
			tmp = tmp | 0xf1;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);

			break;
		case 2:
			/* 2-wire jtag pinmux setting*/
			addr = remap_addr1 + 0x400;
			tmp = readl(addr);
			tmp = tmp & 0xfffff00f;
			tmp = tmp | 0x430;
			writel(tmp, addr);
			tmp = readl(addr);
			WMT_PLAT_PR_INFO("(RegAddr, RegVal):(0x%p, 0x%x)\n", addr, tmp);
			break;
		default:
			WMT_PLAT_PR_INFO("unsupported options!\n");
		}

	}
#endif

error:
	if (remap_addr1)
		iounmap(remap_addr1);
	if (remap_addr2)
		iounmap(remap_addr2);
	return ret;
#else
	WMT_PLAT_PR_INFO("No support switch to JTAG!\n");

	return 0;
#endif
}

static UINT32 consys_jtag_flag_ctrl(UINT32 enable)
{
	WMT_PLAT_PR_INFO("%s jtag set for MCU\n", enable ? "enable" : "disable");
	gJtagCtrl = enable;

	return 0;
}

static INT32 consys_clk_get_from_dts(struct platform_device *pdev)
{
#if (!COMMON_KERNEL_CLK_SUPPORT)
	clk_scp_conn_main = devm_clk_get(&pdev->dev, "conn");
	if (IS_ERR(clk_scp_conn_main)) {
		WMT_PLAT_PR_INFO("[CCF]cannot get clk_scp_conn_main clock.\n");
		return PTR_ERR(clk_scp_conn_main);
	}
	WMT_PLAT_PR_DBG("[CCF]clk_scp_conn_main=%p\n", clk_scp_conn_main);
#endif
	clk_infracfg_ao_ccif4_ap_cg = devm_clk_get(&pdev->dev, "ccif");
	if (IS_ERR(clk_infracfg_ao_ccif4_ap_cg)) {
		WMT_PLAT_PR_INFO("[CCF]cannot get clk_infracfg_ao_ccif4_ap_cg clock.\n");
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
		WMT_PLAT_PR_INFO("Regulator_get VCN_1V8 fail\n");
	reg_VCN13 = regulator_get(&pdev->dev, "vcn13");
	if (!reg_VCN13)
		WMT_PLAT_PR_INFO("Regulator_get VCN_1V3 fail\n");
	reg_VCN33_1_BT = regulator_get(&pdev->dev, "vcn33_1_bt");
	if (!reg_VCN33_1_BT)
		WMT_PLAT_PR_INFO("Regulator_get VCN33_1_BT fail\n");
	reg_VCN33_1_WIFI = regulator_get(&pdev->dev, "vcn33_1_wifi");
	if (!reg_VCN33_1_WIFI)
		WMT_PLAT_PR_INFO("Regulator_get VCN33_1_WIFI fail\n");
	reg_VCN33_2_WIFI = regulator_get(&pdev->dev, "vcn33_2_wifi");
	if (!reg_VCN33_2_WIFI)
		WMT_PLAT_PR_INFO("Regulator_get VCN33_2_WIFI fail\n");
#endif

	return 0;
}

static INT32 consys_co_clock_type(VOID)
{
	return 0;
}

static INT32 consys_clock_buffer_ctrl(MTK_WCN_BOOL enable)
{
#if 0
	if (enable)
		KERNEL_clk_buf_ctrl(CLK_BUF_CONN, true);	/*open XO_WCN*/
	else
		KERNEL_clk_buf_ctrl(CLK_BUF_CONN, false);	/*close XO_WCN*/
#endif
	return 0;
}

static VOID consys_set_if_pinmux(MTK_WCN_BOOL enable)
{
	UINT8 *consys_if_pinmux_reg_base = NULL;
	UINT8 *consys_if_pinmux_driving_base = NULL;

	/* Switch D die pinmux for connecting A die */
	consys_if_pinmux_reg_base = ioremap(CONSYS_IF_PINMUX_REG_BASE, 0x1000);
	if (!consys_if_pinmux_reg_base) {
		WMT_PLAT_PR_INFO("consys_if_pinmux_reg_base(%x) ioremap fail\n",
				CONSYS_IF_PINMUX_REG_BASE);
		return;
	}

	consys_if_pinmux_driving_base = ioremap(CONSYS_IF_PINMUX_DRIVING_BASE, 0x100);
	if (!consys_if_pinmux_driving_base) {
		WMT_PLAT_PR_INFO("consys_if_pinmux_driving_base(%x) ioremap fail\n",
				CONSYS_IF_PINMUX_DRIVING_BASE);
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
		CONSYS_REG_WRITE(consys_if_pinmux_driving_base + CONSYS_IF_PINMUX_DRIVING_OFFSET_1,
				(CONSYS_REG_READ(consys_if_pinmux_driving_base +
				CONSYS_IF_PINMUX_DRIVING_OFFSET_1) &
				CONSYS_IF_PINMUX_DRIVING_MASK_1) | CONSYS_IF_PINMUX_DRIVING_VALUE_1);
		CONSYS_REG_WRITE(consys_if_pinmux_driving_base + CONSYS_IF_PINMUX_DRIVING_OFFSET_2,
				(CONSYS_REG_READ(consys_if_pinmux_driving_base +
				CONSYS_IF_PINMUX_DRIVING_OFFSET_2) &
				CONSYS_IF_PINMUX_DRIVING_MASK_2) | CONSYS_IF_PINMUX_DRIVING_VALUE_2);
#if CLK_CTRL_TCXOENA_REQ
		if (wmt_plat_soc_co_clock_flag_get() == 0) {
			CONSYS_REG_WRITE(consys_if_pinmux_reg_base + CONSYS_CLOCK_TCXO_MODE_OFFSET,
					(CONSYS_REG_READ(consys_if_pinmux_reg_base + CONSYS_CLOCK_TCXO_MODE_OFFSET) &
					CONSYS_CLOCK_TCXO_MODE_MASK) | CONSYS_CLOCK_TCXO_MODE_VALUE);
			WMT_PLAT_PR_INFO("set GPIO137 pinmux for TCXO mode (Aux5)\n");
		}
#endif
	} else {
		CONSYS_REG_WRITE(consys_if_pinmux_reg_base + CONSYS_IF_PINMUX_01_OFFSET,
				CONSYS_REG_READ(consys_if_pinmux_reg_base +
				CONSYS_IF_PINMUX_01_OFFSET) & CONSYS_IF_PINMUX_01_MASK);
		CONSYS_REG_WRITE(consys_if_pinmux_reg_base + CONSYS_IF_PINMUX_02_OFFSET,
				CONSYS_REG_READ(consys_if_pinmux_reg_base +
				CONSYS_IF_PINMUX_02_OFFSET) & CONSYS_IF_PINMUX_02_MASK);
#if CLK_CTRL_TCXOENA_REQ
		if (wmt_plat_soc_co_clock_flag_get() == 0) {
			CONSYS_REG_WRITE(consys_if_pinmux_reg_base + CONSYS_CLOCK_TCXO_MODE_OFFSET,
					CONSYS_REG_READ(consys_if_pinmux_reg_base + CONSYS_CLOCK_TCXO_MODE_OFFSET) &
					CONSYS_CLOCK_TCXO_MODE_MASK);
		}
#endif
	}

	iounmap(consys_if_pinmux_reg_base);
	iounmap(consys_if_pinmux_driving_base);
}

static VOID consys_hw_reset_bit_set(MTK_WCN_BOOL enable)
{
	UINT32 consys_ver_id = 0;
	UINT32 cnt = 0;
	UINT8 *consys_reg_base = NULL;

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
		/* check CONNSYS power-on completion
		 * (polling "0x8000_0600[31:0]" == 0x1D1E and each polling interval is "1ms")
		 * (apply this for guarantee that CONNSYS CPU goes to "cos_idle_loop")
		 */
		consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_COM_REG0);
		while (consys_ver_id != 0x1D1E) {
			if (cnt > 10)
				break;
			consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_COM_REG0);
			WMT_PLAT_PR_INFO("0x18002600(0x%x)\n", consys_ver_id);
			WMT_PLAT_PR_INFO("0x1800216c(0x%x)\n",
					CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_SW_DBG_CTL));
			WMT_PLAT_PR_INFO("0x18007104(0x%x)\n",
					CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base +
					CONSYS_CPUPCR_OFFSET));
			msleep(20);
			cnt++;
		}

		if (mtk_wcn_consys_get_adie_chipid() == SECONDARY_ADIE) {
			/* if(MT6635) CONN_WF_CTRL2 swtich to CONN mode */
			consys_reg_base = ioremap(CONSYS_IF_PINMUX_REG_BASE, 0x1000);
			if (!consys_reg_base) {
				WMT_PLAT_PR_INFO("consys_if_pinmux_reg_base(%x) ioremap fail\n",
						CONSYS_IF_PINMUX_REG_BASE);
				return;
			}
			CONSYS_REG_WRITE(consys_reg_base + CONSYS_WF_CTRL2_03_OFFSET,
					(CONSYS_REG_READ(consys_reg_base + CONSYS_WF_CTRL2_03_OFFSET) &
					CONSYS_WF_CTRL2_03_MASK) | CONSYS_WF_CTRL2_CONN_MODE);
			iounmap(consys_reg_base);
		}
	}
}

static VOID consys_hw_spm_clk_gating_enable(VOID)
{
	/* turn on SPM clock (apply this for SPM's CONNSYS power control related CR accessing) */
	CONSYS_REG_WRITE((conn_reg.spm_base + CONSYS_PWRON_CONFG_EN_OFFSET),
			CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWRON_CONFG_EN_OFFSET) |
			CONSYS_SPM_PWR_ON_CLK_BIT | CONSYS_SPM_PWR_ON_CLK_CTRL_KEY);
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
			WMT_PLAT_PR_INFO("clk_prepare_enable(clk_infracfg_ao_ccif4_ap_cg) fail(%d)\n", iRet);
			return iRet;
		}
		WMT_PLAT_PR_DBG("clk_prepare_enable(clk_infracfg_ao_ccif4_ap_cg) ok\n");
#if (COMMON_KERNEL_CLK_SUPPORT)
		iRet = pm_runtime_get_sync(&connsys_pdev->dev);
		if (iRet)
			WMT_PLAT_PR_INFO("pm_runtime_get_sync() fail(%d)\n", iRet);
		else
			WMT_PLAT_PR_INFO("pm_runtime_get_sync() CONSYS ok\n");

		iRet = device_init_wakeup(&connsys_pdev->dev, true);
		if (iRet)
			WMT_PLAT_PR_INFO("device_init_wakeup(true) fail.\n");
		else
			WMT_PLAT_PR_INFO("device_init_wakeup(true) CONSYS ok\n");
#else
		iRet = clk_prepare_enable(clk_scp_conn_main);
		if (iRet) {
			WMT_PLAT_PR_INFO("clk_prepare_enable(clk_scp_conn_main) fail(%d)\n", iRet);
			return iRet;
		}
		WMT_PLAT_PR_DBG("clk_prepare_enable(clk_scp_conn_main) ok\n");
#endif
		if (conn_reg.infra_ao_pericfg_base != 0) {
			WMT_PLAT_PR_DBG("CON_STA_REG = %x\n",
				CONSYS_REG_READ(
					conn_reg.infra_ao_pericfg_base +
					INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG));

			/* Set CON_PWR_ON bit (CON_STA_REG[0]) */
			CONSYS_REG_WRITE(conn_reg.infra_ao_pericfg_base +
				INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG,
				0x00000001);

			WMT_PLAT_PR_DBG("CON_STA_REG = %x\n",
				CONSYS_REG_READ(
					conn_reg.infra_ao_pericfg_base +
					INFRASYS_COMMON_AP2MD_PCCIF4_AP_PERI_AP_CCU_CONFIG));
		}
#else
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
		i = 0;
		while (value == 0 && i < 10) {
			value = CONSYS_PWR_ON_ACK_BIT &
				CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_OFFSET);
			udelay(500);
			i++;
		}
		if (value == 0)
			WMT_PLAT_PR_INFO("[POS polling info] >> check connsys_on_domain_pwr_ack status fail\n");

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
		i = 0;
		while (value == 0 && i < 10) {
			value = CONSYS_PWR_ON_ACK_S_BIT &
				CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_S_OFFSET);
			udelay(500);
			i++;
		}
		if (value == 0)
			WMT_PLAT_PR_INFO("[POS polling info] >> check connsys_on_domain_pwr_ack_s status fail\n");

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

		/* check "conn_top_off" primary part power status,
		 * check "connsys_off_domain_pwr_ack"=1
		 * (polling "10 times" and each polling interval is "0.5ms")
		 */
		value = CONSYS_SPM_CON_TOP_OFF_CHECK_BIT &
			CONSYS_REG_READ(conn_reg.spm_base + CONSYS_SPM_CON_TOP_OFF_CHECK_OFFSET);
		i = 0;
		while (value == 0 && i < 10) {
			value = CONSYS_SPM_CON_TOP_OFF_CHECK_BIT &
				CONSYS_REG_READ(conn_reg.spm_base + CONSYS_SPM_CON_TOP_OFF_CHECK_OFFSET);
			udelay(500);
			i++;
		}
		if (value == 0)
			WMT_PLAT_PR_INFO("[POS polling info] >> check connsys_off_domain_pwr_ack status fail\n");

		/*wait 0.5ms*/
		udelay(500);

		/*Turn off AHB RX bus sleep protect (AP2CONN AHB Bus protect)
		 *(apply this for INFRA AHB bus accessing when CONNSYS had been turned on)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AHB_RX_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AHB_RX_PROT_EN_OFFSET) &
				 CONSYS_AHB_RX_PROT_MASK);

		value = ~CONSYS_AHB_RX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AHB_RX_PROT_STA_OFFSET);
		i = 0;
		while (value == 0 && i < 10) {
			value = ~CONSYS_AHB_RX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AHB_RX_PROT_STA_OFFSET);
			i++;
		}
		if (value == 0)
			WMT_PLAT_PR_INFO("[POS polling info] >> check AHB RX bus sleep protect turn off fail\n");

		/*Turn off AXI Rx bus sleep protect (CONN2AP AXI Rx Bus protect)
		 *(disable sleep protection when CONNSYS had been turned on)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET) &
				 CONSYS_AXI_RX_PROT_MASK);

		value = ~CONSYS_AXI_RX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_STA_OFFSET);
		i = 0;
		while (value == 0 && i < 10) {
			value = ~CONSYS_AXI_RX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AXI_RX_PROT_STA_OFFSET);
			i++;
		}
		if (value == 0)
			WMT_PLAT_PR_INFO("[POS polling info] >> check AXI Rx bus sleep protect turn off fail\n");

		/*Turn off AXI TX bus sleep protect (CONN2AP AXI Tx Bus protect)
		 *(disable sleep protection when CONNSYS had been turned on)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET) &
				 CONSYS_AXI_TX_PROT_MASK);

		value = ~CONSYS_AXI_TX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_STA_OFFSET);
		i = 0;
		while (value == 0 && i < 10) {
			value = ~CONSYS_AXI_TX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AXI_RX_PROT_STA_OFFSET);
			i++;
		}
		if (value == 0)
			WMT_PLAT_PR_INFO("[POS polling info] >> check AXI Tx bus sleep protect turn off fail\n");

		/*Turn off AHB TX bus sleep protect (AP2CONN AHB Bus protect)
		 *(apply this for INFRA AHB bus accessing when CONNSYS had been turned on)
		 */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_EN_OFFSET) &
				 CONSYS_AHB_TX_PROT_MASK);

		value = ~CONSYS_AHB_TX_PROT_MASK &
			CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_AXI_RX_PROT_STA_OFFSET);
		i = 0;
		while (value == 0 && i < 10) {
			value = ~CONSYS_AHB_TX_PROT_MASK &
				CONSYS_REG_READ(conn_reg.topckgen_base +
				CONSYS_AXI_RX_PROT_STA_OFFSET);
			i++;
		}
		if (value == 0)
			WMT_PLAT_PR_INFO("[POS polling info] >> check AHB TX bus sleep protect turn off fail\n");
#endif /* CONSYS_PWR_ON_OFF_API_AVAILABLE */

		atomic_set(&g_power_on, 1);
		/*wait 5ms*/
		mdelay(5);
	} else {
		osal_lock_unsleepable_lock(&g_pwr_off_lock);
		atomic_set(&g_power_on, 0);
		osal_unlock_unsleepable_lock(&g_pwr_off_lock);

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
#if (COMMON_KERNEL_CLK_SUPPORT)
		iRet = device_init_wakeup(&connsys_pdev->dev, false);
		if (iRet)
			WMT_PLAT_PR_INFO("device_init_wakeup(false) fail.\n");
		else
			WMT_PLAT_PR_INFO("device_init_wakeup(false) CONSYS ok\n");

		iRet = pm_runtime_put_sync(&connsys_pdev->dev);
		if (iRet)
			WMT_PLAT_PR_INFO("pm_runtime_put_sync() fail.\n");
		else
			WMT_PLAT_PR_INFO("pm_runtime_put_sync() CONSYS ok\n");
#else
		clk_disable_unprepare(clk_scp_conn_main);
		WMT_PLAT_PR_DBG("clk_disable_unprepare(clk_scp_conn_main) calling\n");
#endif
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

	if ((conn_reg.mcu_top_misc_off_base == 0) || (conn_reg.mcu_base == 0))
		return 0;

	/*12.poll CONNSYS CHIP ID until chipid is returned  0x10070000 */
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

			/* TODO: NEW API for ROM patch download */
			if (mtk_wcn_consys_get_adie_chipid())
				consys_dl_rom_patch(consys_ver_id, consys_fw_ver);
			break;
		}
		WMT_PLAT_PR_INFO("Read CONSYS version id(0x%08x)", consys_ver_id);
		msleep(20);
	}

	if (retry <= 0) {
#if defined(KERNEL_clk_buf_show_status_info)
		KERNEL_clk_buf_show_status_info();  /* dump clock buffer */
#endif
		return -1;
	}

	consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_top_misc_off_base + CONSYS_CONF_ID_OFFSET);
	WMT_PLAT_PR_INFO("consys configuration id(0x%x)\n", consys_ver_id & 0xF);
	consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_HW_ID_OFFSET);
	WMT_PLAT_PR_INFO("consys HW version id(0x%x)\n", consys_ver_id & 0xFFFF);
	consys_ver_id = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_FW_ID_OFFSET);
	WMT_PLAT_PR_INFO("consys FW version id(0x%x)\n", consys_ver_id & 0xFFFF);

	return 0;
}

static VOID consys_set_access_emi_hw_mode(VOID)
{
	if (conn_reg.mcu_top_misc_on_base == 0)
		return;

	/* conn2ap access EMI hw mode with slpprotect ctrl */
	CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONSYS_ACCESS_EMI_HW_MODE_OFFSET,
		CONSYS_REG_READ(conn_reg.mcu_top_misc_on_base + CONSYS_ACCESS_EMI_HW_MODE_OFFSET) |
		(0x1 << 0));
}

static VOID consys_acr_reg_setting(VOID)
{
	WMT_PLAT_PR_INFO("No need to do acr");
}

static VOID consys_afe_reg_setting(VOID)
{
	UINT8 *consys_afe_wbg_reg_base = NULL;
	/* AFE WBG CR (if needed),
	 * note that this CR must be backuped and restored by command batch engine
	 * 0x180B_3010[31:0]  0x00000000
	 * 0x180B_3018[31:0]  0x144B0160
	 * 0x180B_3040[31:0]  0x10990C13
	 * 0x180B_3058[31:0]  0xCD258051
	 * 0x180B_3078[31:0]  0xC5258251
	 * 0x180B_3094[31:0]  0xC5258251
	 * 0x180B_3100[31:0]  0x10990C13
	 */
	consys_afe_wbg_reg_base = ioremap(CONSYS_AFE_WBG_REG_BASE, 0x200);
	if (consys_afe_wbg_reg_base) {
		CONSYS_REG_WRITE(consys_afe_wbg_reg_base + CONSYS_AFE_WBG_REG_AFE_01_OFFSET,
			CONSYS_AFE_WBG_REG_AFE_01_VALUE);
		CONSYS_REG_WRITE(consys_afe_wbg_reg_base + CONSYS_AFE_WBG_REG_RCK_01_OFFSET,
			CONSYS_AFE_WBG_REG_RCK_01_VALUE);
		CONSYS_REG_WRITE(consys_afe_wbg_reg_base + CONSYS_AFE_WBG_REG_GL1_01_OFFSET,
			CONSYS_AFE_WBG_REG_GL1_01_VALUE);
		CONSYS_REG_WRITE(consys_afe_wbg_reg_base + CONSYS_AFE_WBG_REG_BT_TX_03_OFFSET,
			CONSYS_AFE_WBG_REG_BT_TX_03_VALUE);
		CONSYS_REG_WRITE(consys_afe_wbg_reg_base + CONSYS_AFE_WBG_REG_WF0_TX_03_OFFSET,
			CONSYS_AFE_WBG_REG_WF0_TX_03_VALUE);
		CONSYS_REG_WRITE(consys_afe_wbg_reg_base + CONSYS_AFE_WBG_REG_WF1_TX_03_OFFSET,
			CONSYS_AFE_WBG_REG_WF1_TX_03_VALUE);
		CONSYS_REG_WRITE(consys_afe_wbg_reg_base + CONSYS_AFE_WBG_REG_GL5_01_OFFSET,
			CONSYS_AFE_WBG_REG_GL5_01_VALUE);
		iounmap(consys_afe_wbg_reg_base);
	} else {
		WMT_PLAT_PR_INFO("CONSYS_AFE_WBG_REG_BASE(0x%x) ioremap fail!\n",
			CONSYS_AFE_WBG_REG_BASE);
	}
}

static VOID consys_emi_entry_address(VOID)
{
	/* EMI entry address (define by CONNSYS MCU SE)
	 * 0x1800_2504[31:0] 0xF011_0000
	 * 0x1800_2508[31:0] 0xF01D_0000
	 */
	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_EMI_BT_ENTRY_OFFSET,
			CONSYS_EMI_BT_ENTRY_ADDRESS);
	CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_EMI_WIFI_ENTRY_OFFSET,
			CONSYS_EMI_WIFI_ENTRY_ADDRESS);
}

static VOID consys_set_xo_osc_ctrl(VOID)
{
	UINT8 *consys_reg_base = NULL;
	UINT32 value = 0;
	UINT32 xo_value;

	consys_reg_base = ioremap(CONSYS_COCLOCK_STABLE_TIME_BASE, 0x100);
	if (consys_reg_base) {
		if (!consys_is_rc_mode_enable()) {
		/* set CR "XO initial stable time" and "XO bg stable time"
		 * to optimize the wakeup time after sleep
		 * clear "source clock enable ack to XO state" mask
		 */
			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET) &
				~CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_BIT);

			if (wmt_plat_soc_co_clock_flag_get()) {
				value = CONSYS_REG_READ(consys_reg_base);
				value = (value & CONSYS_COCLOCK_STABLE_TIME_MASK) | CONSYS_COCLOCK_STABLE_TIME;
				CONSYS_REG_WRITE(consys_reg_base, value);

				value = CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_ACK_ENABLE_OFFSET);
				value = (value & CONSYS_COCLOCK_ACK_ENABLE_MAST) | CONSYS_COCLOCK_ACK_ENABLE_VALUE;
				value = value & (~CONSYS_COCLOCK_ACK_ENABLE_BIT);
				CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_ACK_ENABLE_OFFSET, value);
			}
		} else {
			if (wmt_plat_soc_co_clock_flag_get())
				xo_value = CONSYS_COCLOCK_RC_CTL_1_XO_VALUE;
			else
				xo_value = CONSYS_COCLOCK_RC_CTL_1_XO_TCXO_VALUE;

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_1_GPS_XO_OFFSET,
				xo_value);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_GPS_ACK_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_GPS_ACK_OFFSET) &
				~CONSYS_COCLOCK_RC_CTL_0_ACK_BIT);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_1_BT_XO_OFFSET,
				xo_value);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_BT_ACK_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_BT_ACK_OFFSET) &
				~CONSYS_COCLOCK_RC_CTL_0_ACK_BIT);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_1_WF_XO_OFFSET,
				xo_value);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_WF_ACK_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_WF_ACK_OFFSET) &
				~CONSYS_COCLOCK_RC_CTL_0_ACK_BIT);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_1_TOP_XO_OFFSET,
				xo_value);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_ACK_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_ACK_OFFSET) &
				~CONSYS_COCLOCK_RC_CTL_0_ACK_BIT);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET) |
				CONSYS_COCLOCK_RC_CTL_0_GPS_OSC_RC_EN_BIT);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET) |
				CONSYS_COCLOCK_RC_CTL_0_BT_OSC_RC_EN_BIT);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET) |
				CONSYS_COCLOCK_RC_CTL_0_WF_OSC_RC_EN_BIT);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET) |
				CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_BIT_2);

			CONSYS_REG_WRITE(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET,
				CONSYS_REG_READ(consys_reg_base + CONSYS_COCLOCK_RC_CTL_0_TOP_OSC_RC_EN_OFFSET) &
				~CONSYS_COCLOCK_RC_CTL_0_OSC_LEGACY_EN_BIT);
		}
		iounmap(consys_reg_base);
	} else {
			WMT_PLAT_PR_INFO("connsys co_clock stable time base(0x%x) ioremap fail!\n",
					  CONSYS_COCLOCK_STABLE_TIME_BASE);
	}
}

static VOID consys_identify_adie(VOID)
{
	UINT8 *consys_reg_base = NULL;

	consys_reg_base = ioremap(CONSYS_IDENTIFY_ADIE_CR_ADDRESS, 0x100);
	if (consys_reg_base) {
		if (mtk_wcn_consys_get_adie_chipid() == SECONDARY_ADIE)
			CONSYS_REG_WRITE(consys_reg_base,
				CONSYS_REG_READ(consys_reg_base) &
				~CONSYS_IDENTIFY_ADIE_ENABLE_BIT);
		else
			CONSYS_REG_WRITE(consys_reg_base,
				CONSYS_REG_READ(consys_reg_base) |
				CONSYS_IDENTIFY_ADIE_ENABLE_BIT);
		iounmap(consys_reg_base);
	} else {
		WMT_PLAT_PR_INFO("CONSYS_IDENTIFY_ADIE_CR_ADDRESS(0x%x) ioremap fail!\n",
			CONSYS_IDENTIFY_ADIE_CR_ADDRESS);
	}
}

static VOID consys_wifi_ctrl_setting(VOID)
{
	UINT8 *reg_base = NULL;

	if (mtk_wcn_consys_get_adie_chipid() != SECONDARY_ADIE)
		return;

	/* if(MT6635)
	 * CONN_WF_CTRL2 swtich to GPIO mode, GPIO output value
	 * before patch download swtich back to CONN mode.
	 */
	reg_base = ioremap(CONSYS_IF_PINMUX_REG_BASE, 0x1000);
	if (reg_base) {
		CONSYS_REG_WRITE(reg_base + CONSYS_WF_CTRL2_01_OFFSET,
				(CONSYS_REG_READ(reg_base + CONSYS_WF_CTRL2_01_OFFSET) &
				CONSYS_WF_CTRL2_01_MASK) | CONSYS_WF_CTRL2_01_VALUE);
		CONSYS_REG_WRITE(reg_base + CONSYS_WF_CTRL2_02_OFFSET,
				(CONSYS_REG_READ(reg_base + CONSYS_WF_CTRL2_02_OFFSET) &
				CONSYS_WF_CTRL2_02_MASK) | CONSYS_WF_CTRL2_02_VALUE);
		CONSYS_REG_WRITE(reg_base + CONSYS_WF_CTRL2_03_OFFSET,
				(CONSYS_REG_READ(reg_base + CONSYS_WF_CTRL2_03_OFFSET) &
				CONSYS_WF_CTRL2_03_MASK) | CONSYS_WF_CTRL2_GPIO_MODE);
		iounmap(reg_base);
	} else {
		WMT_PLAT_PR_INFO("consys_if_pinmux_reg_base(0x%x) ioremap fail!\n",
			CONSYS_IF_PINMUX_REG_BASE);
	}
}

static VOID consys_bus_timeout_config(VOID)
{
	UINT8 *consys_reg_base = NULL;

	/* connsys bus time out configure,  enable AHB bus timeout */
	consys_reg_base = ioremap(CONSYS_AHB_TIMEOUT_EN_ADDRESS, 0x100);
	if (consys_reg_base) {
		CONSYS_REG_WRITE(consys_reg_base, CONSYS_AHB_TIMEOUT_EN_VALUE);
		iounmap(consys_reg_base);
	} else {
		WMT_PLAT_PR_INFO("CONSYS_AHB_TIMEOUT_EN_ADDRESS(0x%x) ioremap fail!\n",
				  CONSYS_AHB_TIMEOUT_EN_ADDRESS);
	}
}

static VOID consys_hw_vcn33_primary_rc_mode_enable(VOID)
{
	WMT_PLAT_PR_INFO("Turn on reg_VCN33_1_BT in RC mode\n");
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/*  PMRC_EN[7][6][5][4] HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 7);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 7);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 4);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 4);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 5);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 5);
		/* SW_LP =0 */
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN33_1_LP_ADDR,
			PMIC_RG_LDO_VCN33_1_LP_MASK << PMIC_RG_LDO_VCN33_1_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN33_1_LP_SHIFT);
	}
#else
	/*  PMRC_EN[7][6][5][4] HW_OP_EN = 1, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN7, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN4, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN6, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN5, 0, 1, HW_OFF);
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_1_LP, 0);
#endif
	if (reg_VCN33_1_BT) {
		regulator_set_voltage(reg_VCN33_1_BT, 3300000, 3300000);
		if (regulator_enable(reg_VCN33_1_BT))
			WMT_PLAT_PR_INFO("WMT do WIFI PMIC on fail!\n");
	}
}

static VOID consys_hw_vcn33_primary_legacy_mode_enable(VOID)
{
	WMT_PLAT_PR_INFO("Turn on reg_VCN33_1_BT in legacy mode\n");

#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/* HW_OP_EN = 0, HW_OP_CFG = 0 */
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 0);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 1 << 0);
		/* SW_LP =0 */
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN33_1_LP_ADDR,
			PMIC_RG_LDO_VCN33_1_LP_MASK << PMIC_RG_LDO_VCN33_1_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN33_1_LP_SHIFT);
	}
#else
	/* HW_OP_EN = 0, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN0, 1, 1, HW_LP);
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_1_LP, 0);
#endif
	/*Set VCN33_1_SW_EN as 1 and set votage as 3V3*/
	if (reg_VCN33_1_BT) {
		regulator_set_voltage(reg_VCN33_1_BT, 3300000, 3300000);
		if (regulator_enable(reg_VCN33_1_BT))
			WMT_PLAT_PR_INFO("WMT do WIFI PMIC on fail!\n");
	}
}

static VOID consys_hw_vcn13_secondary_rc_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/*  PMRC_EN[7][6][5][4] HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_EN_SET_ADDR, 1 << 7);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_CFG_SET_ADDR, 0 << 7);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_EN_SET_ADDR, 1 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_CFG_SET_ADDR, 0 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_EN_SET_ADDR, 1 << 5);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_CFG_SET_ADDR, 0 << 5);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_EN_SET_ADDR, 1 << 4);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_CFG_SET_ADDR, 0 << 4);
		/* SW_LP =1 */
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN13_LP_ADDR,
			PMIC_RG_LDO_VCN13_LP_MASK << PMIC_RG_LDO_VCN13_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN13_LP_SHIFT);
	}
#else
	/*  PMRC_EN[7][6][5][4] HW_OP_EN = 1, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn13_lp(SRCLKEN7, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn13_lp(SRCLKEN6, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn13_lp(SRCLKEN5, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn13_lp(SRCLKEN4, 0, 1, HW_OFF);
	/* SW_LP =1 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN13_LP, 0);
#endif

	if (reg_VCN13) {
		regulator_set_voltage(reg_VCN13, 1300000, 1300000);
		if (regulator_enable(reg_VCN13))
			WMT_PLAT_PR_INFO("enable VCN13 fail\n");
		else
			WMT_PLAT_PR_DBG("enable VCN13 ok\n");
	}
}

static VOID consys_hw_vcn13_secondary_legacy_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/* HW_OP_EN = 1, HW_OP_CFG = 1 */
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_EN_SET_ADDR, 1 << 0);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN13_OP_CFG_SET_ADDR, 1 << 0);
		/* SW_LP=0 */
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN13_LP_ADDR,
			PMIC_RG_LDO_VCN13_LP_MASK << PMIC_RG_LDO_VCN13_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN13_LP_SHIFT);
	}
#else
	/* HW_OP_EN = 1, HW_OP_CFG = 1 */
	KERNEL_pmic_ldo_vcn13_lp(SRCLKEN0, 1, 1, HW_LP);
	/* SW_LP=0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN13_LP, 0);
#endif

	if (reg_VCN13) {
		regulator_set_voltage(reg_VCN13, 1300000, 1300000);
		if (regulator_enable(reg_VCN13))
			WMT_PLAT_PR_INFO("enable VCN13 fail\n");
		else
			WMT_PLAT_PR_DBG("enable VCN13 ok\n");
	}
}

static VOID consys_hw_vcn18_rc_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	WMT_PLAT_PR_INFO("Turn on VCN18 in RC mode\n");
	if (g_regmap) {
		/*  PMRC_EN[7][6][5][4] HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_EN_SET_ADDR, 1 << 7);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_CFG_SET_ADDR, 0 << 7);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_EN_SET_ADDR, 1 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_CFG_SET_ADDR, 0 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_EN_SET_ADDR, 1 << 5);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_CFG_SET_ADDR, 0 << 5);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_EN_SET_ADDR, 1 << 4);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_CFG_SET_ADDR, 0 << 4);
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN18_LP_ADDR,
			PMIC_RG_LDO_VCN18_LP_MASK << PMIC_RG_LDO_VCN18_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN18_LP_SHIFT);
		}
#else
	/*  PMRC_EN[7][6][5][4] HW_OP_EN = 1, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn18_lp(SRCLKEN7, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn18_lp(SRCLKEN6, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn18_lp(SRCLKEN5, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn18_lp(SRCLKEN4, 0, 1, HW_OFF);

	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN18_LP, 0);
#endif
}

static VOID consys_hw_vcn18_legacy_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_EN_SET_ADDR, 1 << 0);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN18_OP_CFG_SET_ADDR, 1 << 0);
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN18_LP_ADDR,
			PMIC_RG_LDO_VCN18_LP_MASK << PMIC_RG_LDO_VCN18_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN18_LP_SHIFT);
	}
#else
	/* HW_OP_EN = 1, HW_OP_CFG = 1 */
	KERNEL_pmic_ldo_vcn18_lp(SRCLKEN0, 1, 1, HW_LP);
	/* SW_LP=0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN18_LP, 0);
#endif
}

static INT32 consys_hw_vcn18_ctrl(MTK_WCN_BOOL enable)
{
#if CONSYS_PMIC_CTRL_ENABLE
	if (enable) {
		if (consys_is_rc_mode_enable()) {
			/* RC mode */
			WMT_PLAT_PR_INFO("Turn on VCN18 in RC mode\n");
			/* VCN18 op_mode = 0 */
			consys_hw_vcn18_rc_mode_enable();
			/*Set VCN18_SW_EN as 1 and set votage as 1V8*/
			if (reg_VCN18) {
				regulator_set_voltage(reg_VCN18, 1800000, 1800000);
				if (regulator_enable(reg_VCN18))
					WMT_PLAT_PR_INFO("enable VCN18 fail\n");
				else
					WMT_PLAT_PR_DBG("enable VCN18 ok\n");
			}
			if (mtk_wcn_consys_get_adie_chipid() == PRIMARY_ADIE)
				consys_hw_vcn33_primary_rc_mode_enable();
			else if (mtk_wcn_consys_get_adie_chipid() == SECONDARY_ADIE)
				consys_hw_vcn13_secondary_rc_mode_enable();
		} else {
			WMT_PLAT_PR_INFO("Turn on VCN18 in legacy mode\n");
			consys_hw_vcn18_legacy_mode_enable();

			/*Set VCN18_SW_EN as 1 and set votage as 1V8*/
			if (reg_VCN18) {
				regulator_set_voltage(reg_VCN18, 1800000, 1800000);
				if (regulator_enable(reg_VCN18))
					WMT_PLAT_PR_INFO("enable VCN18 fail\n");
				else
					WMT_PLAT_PR_DBG("enable VCN18 ok\n");
			}
			/* Legacy mode */
			if (mtk_wcn_consys_get_adie_chipid() == PRIMARY_ADIE)
				consys_hw_vcn33_primary_legacy_mode_enable();
			else if (mtk_wcn_consys_get_adie_chipid() == SECONDARY_ADIE)
				consys_hw_vcn13_secondary_legacy_mode_enable();
		}
	} else {
		if (mtk_wcn_consys_get_adie_chipid() == PRIMARY_ADIE) {
			WMT_PLAT_PR_INFO("Turn off VCN33\n");
			if (reg_VCN33_1_BT) {
				if (regulator_disable(reg_VCN33_1_BT))
					WMT_PLAT_PR_INFO("disable VCN33 fail!\n");
			}
		} else if (mtk_wcn_consys_get_adie_chipid() == SECONDARY_ADIE) {
			WMT_PLAT_PR_INFO("Turn off VCN13\n");
			if (reg_VCN13) {
				if (regulator_disable(reg_VCN13))
					WMT_PLAT_PR_INFO("disable VCN13 fail!\n");
			}
		}
		/* delay 300us */
		udelay(300);

		WMT_PLAT_PR_INFO("Turn off VCN18\n");
		if (reg_VCN18) {
			if (regulator_disable(reg_VCN18))
				WMT_PLAT_PR_INFO("disable VCN18 fail!\n");
		}
	}
#endif
	return 0;
}

static VOID consys_vcn28_hw_mode_ctrl(UINT32 enable)
{
	/* 6635 not supported */
}

static VOID consys_hw_vcn28_rc_mode(UINT32 enable)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (!g_regmap)
		return;

	/* HW_OP_EN = 1, HW_OP_CFG = 0 */
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_EN_SET_ADDR, enable << 7);
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_CFG_SET_ADDR, 0 << 7);
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_EN_SET_ADDR, enable << 4);
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_CFG_SET_ADDR, 0 << 4);
	if (wmt_plat_soc_co_clock_flag_get() == 0) {
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_EN_SET_ADDR, enable << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_CFG_SET_ADDR, 0 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_EN_SET_ADDR, enable << 5);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_CFG_SET_ADDR, 0 << 5);
	}

	regmap_update_bits(g_regmap,
		PMIC_RG_LDO_VCN33_2_LP_ADDR,
		PMIC_RG_LDO_VCN33_2_LP_MASK << PMIC_RG_LDO_VCN33_2_LP_SHIFT,
		0 << PMIC_RG_LDO_VCN33_2_LP_SHIFT);
#else
	/* HW_OP_EN = 1, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN7, 0, enable, HW_OFF);
	KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN4, 0, enable, HW_OFF);
	if (wmt_plat_soc_co_clock_flag_get() == 0) {
		KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN6, 0, enable, HW_OFF);
		KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN5, 0, enable, HW_OFF);
	}
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_2_LP, 0);
#endif
}

static VOID consys_hw_vcn28_legacy_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (!g_regmap)
		return;

	/* HW_OP_EN = 1, HW_OP_CFG = 0 */
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_EN_SET_ADDR, 1 << 0);
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_CFG_SET_ADDR, 0 << 0);
	/* SW_LP =0 */
	regmap_update_bits(g_regmap,
		PMIC_RG_LDO_VCN33_2_LP_ADDR,
		PMIC_RG_LDO_VCN33_2_LP_MASK << PMIC_RG_LDO_VCN33_2_LP_SHIFT,
		0 << PMIC_RG_LDO_VCN33_2_LP_SHIFT);
#else
	/* HW_OP_EN = 1, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN0, 1, 1, HW_OFF);
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_2_LP, 0);
#endif
}

static INT32 consys_hw_vcn28_ctrl(UINT32 enable)
{
#if CONSYS_PMIC_CTRL_ENABLE
	if (mtk_wcn_consys_get_adie_chipid() == SECONDARY_ADIE)
		/* 6635 not supported */
		return 0;

	if (consys_is_rc_mode_enable()) {
		WMT_PLAT_PR_INFO("Configure reg_VCN33_2 in RC mode\n");
		consys_hw_vcn28_rc_mode(enable);

		if (enable && reg_VCN33_2_WIFI)
			regulator_set_voltage(reg_VCN33_2_WIFI, 2800000, 2800000);
	} else {
		if (enable) {
			WMT_PLAT_PR_INFO("Turn on reg_VCN33_2_WIFI in legacy mode\n");
			consys_hw_vcn28_legacy_mode_enable();

			/*Set VCN33_1_SW_EN as 1 and set votage as 2.8V*/
			if (reg_VCN33_2_WIFI) {
				regulator_set_voltage(reg_VCN33_2_WIFI, 2800000, 2800000);
				if (regulator_enable(reg_VCN33_2_WIFI))
					WMT_PLAT_PR_INFO("WMT do WIFI PMIC on fail!\n");
			}
		} else {
			if (reg_VCN33_2_WIFI) {
				regulator_disable(reg_VCN33_2_WIFI);
				WMT_PLAT_PR_INFO("turn off vcn33_2_WIFI\n");
			}
		}
	}
#endif
	return 0;
}

static INT32 consys_hw_set_vcn13_vcn18_lp_mode(INT8 lp_mode)
{
	if (consys_is_rc_mode_enable()) {
		if (lp_mode == 1)
			udelay(50);
#if (!COMMON_KERNEL_PMIC_SUPPORT)
		KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN18_LP, lp_mode);
		KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN13_LP, lp_mode);
#else
		if (g_regmap) {
			regmap_update_bits(g_regmap,
				PMIC_RG_LDO_VCN18_LP_ADDR,
				PMIC_RG_LDO_VCN18_LP_MASK << PMIC_RG_LDO_VCN18_LP_SHIFT,
				lp_mode << PMIC_RG_LDO_VCN18_LP_SHIFT);
			regmap_update_bits(g_regmap,
				PMIC_RG_LDO_VCN13_LP_ADDR,
				PMIC_RG_LDO_VCN13_LP_MASK << PMIC_RG_LDO_VCN13_LP_SHIFT,
				lp_mode << PMIC_RG_LDO_VCN13_LP_SHIFT);
		}
#endif
		if (lp_mode == 0)
			udelay(50);
	}
	return 0;
}

static VOID consys_hw_bt_vcn33_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/* Set VS2 to 1.4625V */
		regmap_update_bits(g_regmap,
			PMIC_RG_BUCK_VS2_VOSEL_ADDR,
			PMIC_RG_BUCK_VS2_VOSEL_MASK << PMIC_RG_BUCK_VS2_VOSEL_SHIFT,
			0x35 << PMIC_RG_BUCK_VS2_VOSEL_SHIFT);
		/* request VS2 to 1.4625V by VS2 VOTER (use bit 4) */
		regmap_update_bits(g_regmap,
			PMIC_RG_BUCK_VS2_VOTER_EN_SET_ADDR,
			PMIC_RG_BUCK_VS2_VOTER_EN_SET_MASK << PMIC_RG_BUCK_VS2_VOTER_EN_SET_SHIFT,
			0x10 << PMIC_RG_BUCK_VS2_VOTER_EN_SET_SHIFT);
		/* Set VS2 sleep voltage to 1.375V */
		regmap_update_bits(g_regmap,
			PMIC_RG_BUCK_VS2_VOSEL_SLEEP_ADDR,
			PMIC_RG_BUCK_VS2_VOSEL_SLEEP_MASK << PMIC_RG_BUCK_VS2_VOSEL_SLEEP_SHIFT,
			0x2e << PMIC_RG_BUCK_VS2_VOSEL_SLEEP_SHIFT);
		/* Set VCN13 to 1.37V */
		regmap_update_bits(g_regmap,
			PMIC_RG_VCN13_VOCAL_ADDR,
			PMIC_RG_VCN13_VOCAL_MASK << PMIC_RG_VCN13_VOCAL_SHIFT,
			0x7 << PMIC_RG_VCN13_VOCAL_SHIFT);
	}
#else
	/* Set VS2 to 1.4625V */
	KERNEL_pmic_set_register_value(PMIC_RG_BUCK_VS2_VOSEL, 0x35);
	/* request VS2 to 1.4625V by VS2 VOTER (use bit 4) */
	KERNEL_pmic_set_register_value(PMIC_RG_BUCK_VS2_VOTER_EN_SET, 0x10);
	/* Set VS2 sleep voltage to 1.375V */
	KERNEL_pmic_set_register_value(PMIC_RG_BUCK_VS2_VOSEL_SLEEP, 0x2e);
	/* Set VCN13 to 1.37V */
	KERNEL_pmic_set_register_value(PMIC_RG_VCN13_VOCAL, 0x7);
#endif
}

static VOID consys_hw_bt_vcn33_rc_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/*  PMRC_EN[6][5]  HW_OP_EN = 1, HW_OP_CFG = 0  */
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 5);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 5);
		/* SW_LP =0 */
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN33_1_LP_ADDR,
			PMIC_RG_LDO_VCN33_1_LP_MASK << PMIC_RG_LDO_VCN33_1_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN33_1_LP_SHIFT);
	}
#else
	/*  PMRC_EN[6][5]  HW_OP_EN = 1, HW_OP_CFG = 0  */
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN6, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN5, 0, 1, HW_OFF);
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_1_LP, 0);
#endif
}

static VOID consys_hw_bt_vcn33_legacy_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/* HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 0);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 0);
		/* SW_LP =0 */
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN33_1_LP_ADDR,
			PMIC_RG_LDO_VCN33_1_LP_MASK << PMIC_RG_LDO_VCN33_1_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN33_1_LP_SHIFT);
	}
#else
	/* HW_OP_EN = 1, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN0, 1, 1, HW_OFF);
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_1_LP, 0);
#endif
}

static VOID consys_hw_bt_vcn33_disable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/* restore VCN13 to 1.3V */
		regmap_update_bits(g_regmap,
			PMIC_RG_VCN13_VOCAL_ADDR,
			PMIC_RG_VCN13_VOCAL_MASK << PMIC_RG_VCN13_VOCAL_SHIFT,
			0 << PMIC_RG_VCN13_VOCAL_SHIFT);
		/* clear bit 4 of VS2 VOTER then VS2 can restore to 1.35V */
		regmap_update_bits(g_regmap,
			PMIC_RG_BUCK_VS2_VOTER_EN_CLR_ADDR,
			PMIC_RG_BUCK_VS2_VOTER_EN_CLR_MASK << PMIC_RG_BUCK_VS2_VOTER_EN_CLR_SHIFT,
			0x10 << PMIC_RG_BUCK_VS2_VOTER_EN_CLR_SHIFT);
		/* restore VS2 sleep voltage to 1.35V */
		regmap_update_bits(g_regmap,
			PMIC_RG_BUCK_VS2_VOSEL_SLEEP_ADDR,
			PMIC_RG_BUCK_VS2_VOSEL_SLEEP_MASK << PMIC_RG_BUCK_VS2_VOSEL_SLEEP_SHIFT,
			0x2c << PMIC_RG_BUCK_VS2_VOSEL_SLEEP_SHIFT);
	}
	consys_hw_set_vcn13_vcn18_lp_mode(1);

	if (consys_is_rc_mode_enable()) {
		WMT_PLAT_PR_INFO("Do nothing for reg_VCN33_1_BT under RC mode when disable\n");
	} else {
		/*Set VCN33_1 as low-power mode(1), HW0_OP_EN as 0, HW0_OP_CFG as HW_OFF(0)*/
		if (g_regmap) {
			regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 0 << 0);
			regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 0);
		}
		if (reg_VCN33_1_BT)
			regulator_disable(reg_VCN33_1_BT);
	}
#else
	/* restore VCN13 to 1.3V */
	KERNEL_pmic_set_register_value(PMIC_RG_VCN13_VOCAL, 0);
	/* clear bit 4 of VS2 VOTER then VS2 can restore to 1.35V */
	KERNEL_pmic_set_register_value(PMIC_RG_BUCK_VS2_VOTER_EN_CLR, 0x10);
	/* restore VS2 sleep voltage to 1.35V */
	KERNEL_pmic_set_register_value(PMIC_RG_BUCK_VS2_VOSEL_SLEEP, 0x2c);

	consys_hw_set_vcn13_vcn18_lp_mode(1);

	if (consys_is_rc_mode_enable()) {
		WMT_PLAT_PR_INFO("Do nothing for reg_VCN33_1_BT under RC mode when disable\n");
	} else {
		/*Set VCN33_1 as low-power mode(1), HW0_OP_EN as 0, HW0_OP_CFG as HW_OFF(0)*/
		KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN0, 1, 0, HW_OFF);
		if (reg_VCN33_1_BT)
			regulator_disable(reg_VCN33_1_BT);
	}
#endif
}

static INT32 consys_hw_bt_vcn33_ctrl(UINT32 enable)
{
#if CONSYS_PMIC_CTRL_ENABLE
	if (mtk_wcn_consys_get_adie_chipid() != SECONDARY_ADIE)
		/* 6631 not supported */
		return 0;
	if (enable) {
		/* Switch to ON mode to avoid OC */
		consys_hw_set_vcn13_vcn18_lp_mode(0);
		consys_hw_bt_vcn33_enable();
		consys_hw_set_vcn13_vcn18_lp_mode(1);

		if (consys_is_rc_mode_enable()) {
			WMT_PLAT_PR_INFO("Turn on reg_VCN33_1_BT in RC mode\n");
			consys_hw_bt_vcn33_rc_mode_enable();
			if (reg_VCN33_1_BT)
				regulator_set_voltage(reg_VCN33_1_BT, 3300000, 3300000);
		} else {
			WMT_PLAT_PR_INFO("Turn on reg_VCN33_1_BT in legacy mode\n");
			consys_hw_bt_vcn33_legacy_mode_enable();

			/*Set VCN33_1_SW_EN as 1 and set votage as 3V3*/
			if (reg_VCN33_1_BT) {
				regulator_set_voltage(reg_VCN33_1_BT, 3300000, 3300000);
				if (regulator_enable(reg_VCN33_1_BT))
					WMT_PLAT_PR_INFO("WMT do WIFI PMIC on fail!\n");
			}
		}
		WMT_PLAT_PR_DBG("WMT do BT PMIC on\n");
	} else {
		/*do BT PMIC off */
		consys_hw_set_vcn13_vcn18_lp_mode(0);
		consys_hw_bt_vcn33_disable();
		WMT_PLAT_PR_DBG("WMT do BT PMIC off\n");
	}
#endif
	return 0;
}

static VOID consys_hw_wifi_vcn33_rc_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 5);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 5);
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN33_1_LP_ADDR,
			PMIC_RG_LDO_VCN33_1_LP_MASK << PMIC_RG_LDO_VCN33_1_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN33_1_LP_SHIFT);

		regulator_set_voltage(reg_VCN33_1_WIFI, 3300000, 3300000);

		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_EN_SET_ADDR, 1 << 6);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_CFG_SET_ADDR, 0 << 6);
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN33_2_LP_ADDR,
			PMIC_RG_LDO_VCN33_2_LP_MASK << PMIC_RG_LDO_VCN33_2_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN33_2_LP_SHIFT);

		regulator_set_voltage(reg_VCN33_2_WIFI, 3300000, 3300000);
	}
#else
	/*  PMRC_EN[6][5]  HW_OP_EN = 1, HW_OP_CFG = 0  */
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN6, 0, 1, HW_OFF);
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN5, 0, 1, HW_OFF);

	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_1_LP, 0);

	regulator_set_voltage(reg_VCN33_1_WIFI, 3300000, 3300000);

	/*  PMRC_EN[6]  HW_OP_EN = 1, HW_OP_CFG = 0  */
	KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN6, 0, 1, HW_OFF);
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_2_LP, 0);
	regulator_set_voltage(reg_VCN33_2_WIFI, 3300000, 3300000);
#endif
}

static VOID consys_hw_wifi_vcn33_legacy_mode_enable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (g_regmap) {
		/* HW_OP_EN = 1, HW_OP_CFG = 0 */
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 1 << 0);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 0);
		/* SW_LP =0 */
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN33_1_LP_ADDR,
			PMIC_RG_LDO_VCN33_1_LP_MASK << PMIC_RG_LDO_VCN33_1_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN33_1_LP_SHIFT);

		/*Set VCN33_1_SW_EN as 1 and set votage as 3V3*/
		if (reg_VCN33_1_WIFI) {
			regulator_set_voltage(reg_VCN33_1_WIFI, 3300000, 3300000);
			if (regulator_enable(reg_VCN33_1_WIFI))
				WMT_PLAT_PR_INFO("WMT do WIFI PMIC on fail!\n");
		}
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_EN_SET_ADDR, 1 << 0);
		regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_CFG_SET_ADDR, 0 << 0);
		regmap_update_bits(g_regmap,
			PMIC_RG_LDO_VCN33_2_LP_ADDR,
			PMIC_RG_LDO_VCN33_2_LP_MASK << PMIC_RG_LDO_VCN33_2_LP_SHIFT,
			0 << PMIC_RG_LDO_VCN33_2_LP_SHIFT);
		/*Set VCN33_1_SW_EN as 1 and set votage as 3V3*/
		if (reg_VCN33_2_WIFI) {
			regulator_set_voltage(reg_VCN33_2_WIFI, 3300000, 3300000);
			if (regulator_enable(reg_VCN33_2_WIFI))
				WMT_PLAT_PR_INFO("WMT do WIFI PMIC on fail!\n");
		}

	}
#else
	/* HW_OP_EN = 1, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN0, 1, 1, HW_OFF);
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_1_LP, 0);

	/*Set VCN33_1_SW_EN as 1 and set votage as 3V3*/
	if (reg_VCN33_1_WIFI) {
		regulator_set_voltage(reg_VCN33_1_WIFI, 3300000, 3300000);
		if (regulator_enable(reg_VCN33_1_WIFI))
			WMT_PLAT_PR_INFO("WMT do WIFI PMIC on fail!\n");
	}
	/* HW_OP_EN = 1, HW_OP_CFG = 0 */
	KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN0, 1, 1, HW_OFF);
	/* SW_LP =0 */
	KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_2_LP, 0);

	/*Set VCN33_1_SW_EN as 1 and set votage as 3V3*/
	if (reg_VCN33_2_WIFI) {
		regulator_set_voltage(reg_VCN33_2_WIFI, 3300000, 3300000);
		if (regulator_enable(reg_VCN33_2_WIFI))
			WMT_PLAT_PR_INFO("WMT do WIFI PMIC on fail!\n");
	}
#endif
}

static VOID consys_hw_wifi_vcn33_legacy_mode_disable(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (!g_regmap)
		return;
	/*Set VCN33_1 as low-power mode(1), HW0_OP_EN as 0, HW0_OP_CFG as HW_OFF(0)*/
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_EN_SET_ADDR, 0 << 0);
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_1_OP_CFG_SET_ADDR, 0 << 0);
	if (reg_VCN33_1_WIFI)
		regulator_disable(reg_VCN33_1_WIFI);

	/*Set VCN33_2 as low-power mode(1), HW0_OP_EN as 0, HW0_OP_CFG as HW_OFF(0)*/
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_EN_SET_ADDR, 0 << 0);
	regmap_write(g_regmap, PMIC_RG_LDO_VCN33_2_OP_CFG_SET_ADDR, 0 << 0);
	if (reg_VCN33_2_WIFI)
		regulator_disable(reg_VCN33_2_WIFI);
#else
	/*Set VCN33_1 as low-power mode(1), HW0_OP_EN as 0, HW0_OP_CFG as HW_OFF(0)*/
	KERNEL_pmic_ldo_vcn33_1_lp(SRCLKEN0, 1, 0, HW_OFF);
	if (reg_VCN33_1_WIFI)
		regulator_disable(reg_VCN33_1_WIFI);

	/*Set VCN33_2 as low-power mode(1), HW0_OP_EN as 0, HW0_OP_CFG as HW_OFF(0)*/
	KERNEL_pmic_ldo_vcn33_2_lp(SRCLKEN0, 1, 0, HW_OFF);
	if (reg_VCN33_2_WIFI)
		regulator_disable(reg_VCN33_2_WIFI);
#endif
}

static INT32 consys_hw_wifi_vcn33_ctrl(UINT32 enable)
{
#if CONSYS_PMIC_CTRL_ENABLE
	if (mtk_wcn_consys_get_adie_chipid() != SECONDARY_ADIE)
		/* 6631 not supported */
		return 0;
	if (enable) {
		if (consys_is_rc_mode_enable()) {
			WMT_PLAT_PR_INFO("Turn on reg_VCN33_1_WIFI in RC mode\n");
			WMT_PLAT_PR_INFO("Turn on reg_VCN33_2_WIFI in RC mode\n");
			consys_hw_wifi_vcn33_rc_mode_enable();
		} else {
			WMT_PLAT_PR_INFO("Turn on reg_VCN33_1_WIFI in legacy mode\n");
			consys_hw_wifi_vcn33_legacy_mode_enable();
		}
		WMT_PLAT_PR_DBG("WMT do WIFI PMIC on\n");
	} else {
		/*do WIFI PMIC off */
		/*switch WIFI PALDO control from HW mode to SW mode:0x418[14]-->0x0 */
		if (consys_is_rc_mode_enable())
			WMT_PLAT_PR_INFO("Do nothing for VCN33_1 under RC mode when disable\n");
		else
			consys_hw_wifi_vcn33_legacy_mode_disable();
		WMT_PLAT_PR_DBG("WMT do WIFI PMIC off\n");
	}
#endif
	return 0;
}

static INT32 consys_hw_vcn_ctrl_after_idle(VOID)
{
#if (COMMON_KERNEL_PMIC_SUPPORT)
	if (consys_is_rc_mode_enable() && g_regmap) {
		regmap_update_bits(g_regmap,
		PMIC_RG_LDO_VCN18_LP_ADDR,
		PMIC_RG_LDO_VCN18_LP_MASK << PMIC_RG_LDO_VCN18_LP_SHIFT,
			1 << PMIC_RG_LDO_VCN18_LP_SHIFT);

		if (mtk_wcn_consys_get_adie_chipid() == PRIMARY_ADIE) {
			regmap_update_bits(g_regmap,
				PMIC_RG_LDO_VCN33_1_LP_ADDR,
				PMIC_RG_LDO_VCN33_1_LP_MASK << PMIC_RG_LDO_VCN33_1_LP_SHIFT,
				1 << PMIC_RG_LDO_VCN33_1_LP_SHIFT);
		} else if (mtk_wcn_consys_get_adie_chipid() == SECONDARY_ADIE) {
			regmap_update_bits(g_regmap,
				PMIC_RG_LDO_VCN13_LP_ADDR,
				PMIC_RG_LDO_VCN13_LP_MASK << PMIC_RG_LDO_VCN13_LP_SHIFT,
				1 << PMIC_RG_LDO_VCN13_LP_SHIFT);
		}
	}
#else
	if (consys_is_rc_mode_enable()) {
		KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN18_LP, 1);

		if (mtk_wcn_consys_get_adie_chipid() == PRIMARY_ADIE)
			KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN33_1_LP, 1);
		else if (mtk_wcn_consys_get_adie_chipid() == SECONDARY_ADIE)
			KERNEL_pmic_set_register_value(PMIC_RG_LDO_VCN13_LP, 1);
	}
#endif
	return 0;
}

static INT32 consys_emi_mpu_set_region_protection(VOID)
{
#if CONSYS_ENABLE_EMI_MPU
	struct emimpu_region_t region;
	unsigned long long start = gConEmiPhyBase;
	unsigned long long end = gConEmiPhyBase + gConEmiSize - 1;

	mtk_emimpu_init_region(&region, REGION_CONN);
	mtk_emimpu_set_addr(&region, start, end);
	mtk_emimpu_set_apc(&region, DOMAIN_AP, MTK_EMIMPU_NO_PROTECTION);
	mtk_emimpu_set_apc(&region, DOMAIN_CONN, MTK_EMIMPU_NO_PROTECTION);
	mtk_emimpu_set_protection(&region);
	mtk_emimpu_free_region(&region);

	WMT_PLAT_PR_INFO("setting MPU for EMI share memory\n");
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
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
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
		WMT_PLAT_PR_INFO("[%s] can't find CONSYS compatible node\n", __func__);
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
		WMT_PLAT_PR_DBG("Get mcu register base(0x%zx)\n", conn_reg.mcu_base);
		conn_reg.ap_rgu_base = (SIZE_T) of_iomap(node, TOP_RGU_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get ap_rgu register base(0x%zx)\n", conn_reg.ap_rgu_base);
		conn_reg.topckgen_base = (SIZE_T) of_iomap(node, INFRACFG_AO_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get topckgen register base(0x%zx)\n", conn_reg.topckgen_base);
		conn_reg.spm_base = (SIZE_T) of_iomap(node, SPM_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get spm register base(0x%zx)\n", conn_reg.spm_base);
		conn_reg.mcu_conn_hif_on_base = (SIZE_T) of_iomap(node, MCU_CONN_HIF_ON_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get mcu_conn_hif_on register base(0x%zx)\n",
				conn_reg.mcu_conn_hif_on_base);
		conn_reg.mcu_top_misc_off_base = (SIZE_T) of_iomap(node,
				MCU_TOP_MISC_OFF_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get mcu_top_misc_off register base(0x%zx)\n",
				conn_reg.mcu_top_misc_off_base);
		conn_reg.mcu_cfg_on_base = (SIZE_T) of_iomap(node, MCU_CFG_ON_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get mcu_cfg_on register base(0x%zx)\n", conn_reg.mcu_cfg_on_base);
		conn_reg.mcu_cirq_base = (SIZE_T) of_iomap(node, MCU_CIRQ_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get mcu cirq  register base(0x%zx)\n", conn_reg.mcu_cirq_base);
		conn_reg.mcu_top_misc_on_base = (SIZE_T) of_iomap(node, MCU_TOP_MISC_ON_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get mcu_top_misc_on register base(0x%zx)\n",
				conn_reg.mcu_top_misc_on_base);
		conn_reg.mcu_conn_hif_pdma_base = (SIZE_T) of_iomap(node, MCU_CONN_HIF_PDMA_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get mcu_conn_hif_pdma register base(0x%zx)\n",
				conn_reg.mcu_conn_hif_pdma_base);
		conn_reg.ap_pccif4_base = (SIZE_T) of_iomap(node, AP_PCCIF4_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get ap_pccif4 register base(0x%zx)\n",
				conn_reg.ap_pccif4_base);
		conn_reg.infra_ao_pericfg_base = (SIZE_T) of_iomap(node, INFRA_AO_PERICFG_BASE_INDEX);
		WMT_PLAT_PR_DBG("Get infra_ao_pericfg register base(0x%zx)\n",
				conn_reg.infra_ao_pericfg_base);
	} else {
		WMT_PLAT_PR_INFO("[%s] can't find CONSYS compatible node\n", __func__);
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

	if (mtk_consys_check_reg_readable_by_addr(conn_reg.mcu_conn_hif_on_base + CONSYS_CPUPCR_OFFSET) == 0)
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

static UINT32 consys_adie_chipid_checking_flow(UINT32 adie)
{
	UINT8 *conn_rf_spi_base = NULL;
	UINT32 chipid = 0;
	UINT32 retry = 10;

	conn_rf_spi_base = ioremap(CONN_RF_SPI_BASE, 0x100);
	if (!conn_rf_spi_base) {
		WMT_PLAT_PR_INFO("ioumap 0x180c6000 fail");
		return 0;
	}

	if (reg_VCN18) {
		regulator_set_voltage(reg_VCN18, 1800000, 1800000);
		if (regulator_enable(reg_VCN18))
			WMT_PLAT_PR_INFO("enable VCN18 fail\n");
		else
			WMT_PLAT_PR_DBG("enable VCN18 ok\n");
	}
	udelay(300);
	if ((reg_VCN13) && (adie == SECONDARY_ADIE)) {
		regulator_set_voltage(reg_VCN13, 1300000, 1300000);
		if (regulator_enable(reg_VCN13))
			WMT_PLAT_PR_INFO("enable VCN13 fail\n");
		else
			WMT_PLAT_PR_DBG("enable VCN13 ok\n");
	}
	if (wmt_plat_soc_co_clock_flag_get() == 0)
		consys_hw_vcn28_ctrl(ENABLE);
	consys_set_if_pinmux(ENABLE);
	udelay(150);
	consys_hw_reset_bit_set(ENABLE);
	consys_hw_spm_clk_gating_enable();
	consys_hw_power_ctrl(ENABLE);
	udelay(10);
	polling_consys_chipid();
	consys_set_access_emi_hw_mode();
	consys_afe_reg_setting();
	consys_set_xo_osc_ctrl();

	CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONN_ON_ADIE_CTL_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_top_misc_on_base + CONN_ON_ADIE_CTL_OFFSET)
			| 0x1);

	while (retry-- > 0) {
		if ((CONSYS_REG_READ(conn_rf_spi_base) & 0x1) == 0)
			break;
		udelay(500);
	}

	if (adie == PRIMARY_ADIE)
		CONSYS_REG_WRITE(conn_rf_spi_base + SPI_TOP_ADDR, 0x0000b024);
	else if (adie == SECONDARY_ADIE)
		CONSYS_REG_WRITE(conn_rf_spi_base + SPI_TOP_ADDR, 0x0000b02C);
	CONSYS_REG_WRITE(conn_rf_spi_base + SPI_TOP_WDAT, 0);

	retry = 10;
	while (retry-- > 0) {
		if ((CONSYS_REG_READ(conn_rf_spi_base) & 0x20) == 0)
			break;
		udelay(500);
	}

	chipid = CONSYS_REG_READ(conn_rf_spi_base + SPI_TOP_RDAT) >> 16;

	CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONN_ON_ADIE_CTL_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_top_misc_on_base + CONN_ON_ADIE_CTL_OFFSET)
			& 0x0);

	iounmap(conn_rf_spi_base);

	consys_hw_power_ctrl(DISABLE);
	consys_set_if_pinmux(DISABLE);
	if (wmt_plat_soc_co_clock_flag_get() == 0)
		consys_hw_vcn28_ctrl(DISABLE);

	if ((reg_VCN13) && (adie == SECONDARY_ADIE)) {
		if (regulator_disable(reg_VCN13))
			WMT_PLAT_PR_INFO("disable VCN13 fail!\n");
	}
	udelay(300);
	if (reg_VCN18) {
		if (regulator_disable(reg_VCN18))
			WMT_PLAT_PR_INFO("disable VCN18 fail!\n");
	}

	if ((reg_VCN13) && (chipid == PRIMARY_ADIE)) {
		/* release VCN13 resource if never used to prevent customerized issue */
		regulator_put(reg_VCN13);
		reg_VCN13 = NULL;
	}

	return chipid;
}

static INT32 consys_adie_chipid_detect(VOID)
{
	INT32 chipid = -1;
	UINT32 retry = 3;

	while ((retry-- > 0) && (chipid == -1)) {
		if (consys_adie_chipid_checking_flow(PRIMARY_ADIE) == PRIMARY_ADIE)
			chipid = PRIMARY_ADIE;
		else if (consys_adie_chipid_checking_flow(SECONDARY_ADIE) == SECONDARY_ADIE)
			chipid = SECONDARY_ADIE;
	}
	WMT_PLAT_PR_INFO("A-die chip id = %x\n", chipid);

	return chipid;
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
		else
			return -1;
	}

	return 0;
}

static VOID consys_set_dl_rom_patch_flag(INT32 flag)
{
	rom_patch_dl_flag = flag;
}

static VOID consys_conn2ap_sw_irq_clear(VOID)
{
	UINT32 ret;

	if (!conn_reg.mcu_base)
		return;

	if (osal_trylock_unsleepable_lock(&g_pwr_off_lock) == 0) {
		WMT_PLAT_PR_INFO("fail to get pwr off lock\n");
		return;
	}

	if (atomic_read(&g_power_on) == 1) {
		/* clear 0x1800214c[27:24] */
		CONSYS_REG_WRITE(conn_reg.mcu_base + CONN2AP_SW_IRQ_CLR_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_base + CONN2AP_SW_IRQ_CLR_OFFSET)
			| (0xf << 24));

		ret = CONSYS_REG_READ(conn_reg.mcu_base + CONN2AP_SW_IRQ_OFFSET) & (0xf << 24);
		if (ret > 0)
			WMT_PLAT_PR_INFO("CONN2AP_SW_IRQ[27:24] = 0x%x\n", ret);
	}

	osal_unlock_unsleepable_lock(&g_pwr_off_lock);
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
		WMT_PLAT_PR_INFO("[%s] can't find CONSYS compatible node\n", __func__);
		return iret;
	}
	osal_unsleepable_lock_init(&g_pwr_off_lock);

	irq_config.irq_num = irq_num;
	irq_config.irq_flag = irq_flag;
	irq_config.irq_callback = consys_conn2ap_sw_irq_clear;

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
	osal_unsleepable_lock_deinit(&g_pwr_off_lock);
}

static INT32 consys_emi_coredump_remapping(UINT8 __iomem **addr, UINT32 enable)
{
	if (enable) {
		*addr = ioremap(gConEmiPhyBase + CONSYS_EMI_COREDUMP_OFFSET,
				CONSYS_EMI_COREDUMP_MEM_SIZE);
		if (*addr) {
			WMT_PLAT_PR_INFO("COREDUMP EMI mapping OK virtual(0x%p) physical(0x%x)\n",
					*addr, (UINT32) gConEmiPhyBase +
					CONSYS_EMI_COREDUMP_OFFSET);
			memset_io(*addr, 0, CONSYS_EMI_COREDUMP_MEM_SIZE);
		} else {
			WMT_PLAT_PR_INFO("EMI mapping fail\n");
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
		WMT_PLAT_PR_INFO("get virtual address fail\n");
		return -1;
	}
	WMT_PLAT_PR_INFO("Reset EMI(0xF0068000 ~ 0xF0107FFF)\n");
	memset_io(addr, 0, CONSYS_EMI_COREDUMP_MEM_SIZE);
	return 0;
}

static INT32 consys_check_reg_readable(VOID)
{
	INT32 can_read = 0;
	UINT32 value = 0;

	if (conn_reg.mcu_cfg_on_base != 0 &&
	    conn_reg.mcu_top_misc_on_base != 0) {
		/*check connsys clock and sleep status*/
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base, CONSYS_CLOCK_CHECK_VALUE);
		udelay(200);
		value = CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base);
		if ((value & CONSYS_HCLK_CHECK_BIT) &&
		    (value & CONSYS_OSCCLK_CHECK_BIT))
			can_read = 1;
	}

	if (!can_read)
		WMT_PLAT_PR_INFO("connsys clock check fail 0x18007000(0x%x)\n", value);

	return can_read;
}

static VOID consys_ic_clock_fail_dump(VOID)
{
#if 0
	UINT8 *addr;
#endif
	char *temp;
	char buffer[1024] = {""};

	if (conn_reg.mcu_base == 0 ||
			conn_reg.mcu_conn_hif_pdma_base == 0 ||
			conn_reg.mcu_conn_hif_on_base == 0)
		return;

	temp = buffer;
	temp += sprintf(temp, "CONN_HIF_TOP_MISC=0x%08x CONN_HIF_BUSY_STATUS=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_TOP_MISC),
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_BUSY_STATUS));

	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_DBG_IDX, 0x3333);
	temp += sprintf(temp, "Write CONSYS_HIF_DBG_IDX to 0x3333\n");

	temp += sprintf(temp, "CONSYS_HIF_DBG_PROBE=0x%08x CONN_HIF_TOP_MISC=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_DBG_PROBE),
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_TOP_MISC));

	temp += sprintf(temp, "CONN_HIF_BUSY_STATUS=0x%08x CONN_HIF_PDMA_BUSY_STATUS=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_BUSY_STATUS),
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_PDMA_BUSY_STATUS));

	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_DBG_IDX, 0x2222);
	temp += sprintf(temp, "Write CONSYS_HIF_DBG_IDX to 0x2222\n");

	temp += sprintf(temp, "CONSYS_HIF_DBG_PROBE=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_DBG_PROBE));

	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_DBG_IDX, 0x3333);
	temp += sprintf(temp, "Write CONSYS_HIF_DBG_IDX to 0x3333\n");

	temp += sprintf(temp, "CONSYS_HIF_DBG_PROBE=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_DBG_PROBE));

	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_DBG_IDX, 0x4444);
	temp += sprintf(temp, "Write CONSYS_HIF_DBG_IDX to 0x4444\n");

	temp += sprintf(temp, "CONSYS_HIF_DBG_PROBE=0x%08x CONN_MCU_EMI_CONTROL=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_DBG_PROBE),
		CONSYS_REG_READ(conn_reg.mcu_base + CONN_MCU_EMI_CONTROL));
	temp += sprintf(temp, "EMI_CONTROL_DBG_PROBE=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + EMI_CONTROL_DBG_PROBE));
	temp += sprintf(temp, "CONN_MCU_CLOCK_CONTROL=0x%08x CONN_MCU_BUS_CONTROL=0x%08x\n",
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_CLOCK_CONTROL),
		CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_BUS_CONTROL));
#if 0
	temp = buffer;
	addr = ioremap(0x180bc000, 0x100);
	if (addr) {
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
	} else {
		WMT_PLAT_PR_INFO("0x180bc000 ioremap fail!\n");
	}

	addr = ioremap(0x10001B20, 0x100);
	if (addr) {
		/* 0x10001B20 */
		temp += sprintf(temp, "0x10001B20=0x%08x\n", CONSYS_REG_READ(addr));
		iounmap(addr);
	} else {
		WMT_PLAT_PR_INFO("0x10001B20 ioremap fail!\n");
	}

	/* conn_hif_on misc info */
	temp += sprintf(temp, "0x1800713c=0x%08x\n", CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + 0x13c));

	addr = ioremap(0x180c1144, 0x100);
	if (addr) {
		/* conn_on_host debug flag */
		temp += sprintf(temp, "0x180c1144=0x%08x\n", CONSYS_REG_READ(addr));
		iounmap(addr);
	} else {
		WMT_PLAT_PR_INFO("0x180c1144 ioremap fail!\n");
	}

	addr = ioremap(0x1020E804, 0x100);
	if (addr) {
		/* 0x1020E804 */
		temp += sprintf(temp, "0x1020E804=0x%08x\n", CONSYS_REG_READ(addr));
		iounmap(addr);
	} else {
		WMT_PLAT_PR_INFO("0x1020E804 ioremap fail!\n");
	}
#endif
	WMT_PLAT_PR_INFO("%s length = %d", buffer, osal_strlen(buffer));
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

static INT32 consys_dump_osc_state(P_CONSYS_STATE state)
{
#define MCU_OSC_EN_BIT		6
#define BT_OSC_EN_BIT		8
#define GPS_OSC_EN_BIT		9
#define WIFI_OSC_EN_BIT		12
#define FM_OSC_EN_BIT		7
#define AP2CONN_OSC_EN_BIT	5

	UINT32 value = 0;
	UINT8 strbuf[DBG_LOG_STR_SIZE] = {""};
	UINT32 len = 0;

	UINT32 mcu_osc_en = 0;
	UINT32 bt_osc_en = 0;
	UINT32 gps_osc_en = 0;
	UINT32 wifi_osc_en = 0;
	UINT32 fm_osc_en = 0;
	UINT32 ap2conn_osc_en = 0;

	if (conn_reg.mcu_cfg_on_base != 0 &&
	    conn_reg.mcu_top_misc_on_base != 0) {

		CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONN_CFG_ON_MON_CTL_ADDR_OFFSET, 0x80000001);
		CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONN_CFG_ON_MON_SEL0_ADDR_OFFSET, 0x03020100);
		CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONN_CFG_ON_MON_SEL1_ADDR_OFFSET, 0x07060504);
		CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONN_CFG_ON_MON_SEL2_ADDR_OFFSET, 0x0b0a0908);
		CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONN_CFG_ON_MON_SEL3_ADDR_OFFSET, 0x0f0e0d0c);
		CONSYS_REG_WRITE(conn_reg.mcu_top_misc_on_base + CONN_CFG_ON_DBGSEL_ADDR_OFFSET, 0x3);
		value = CONSYS_REG_READ(conn_reg.mcu_top_misc_on_base + CONN_CFG_ON_MON_FLAG_RECORD_ADDR_OFFSET);

		state->lp[0] = (UINT32)0x180c1340;
		state->lp[1] = value;

		if (value != 0 && value != 0xdeadfeed) {
			mcu_osc_en = (value >> MCU_OSC_EN_BIT) & 0x1;
			bt_osc_en = (value >> BT_OSC_EN_BIT) & 0x1;
			gps_osc_en = (value >> GPS_OSC_EN_BIT) & 0x1;
			wifi_osc_en = (value >> WIFI_OSC_EN_BIT) & 0x1;
			fm_osc_en = (value >> FM_OSC_EN_BIT) & 0x1;
			ap2conn_osc_en = (value >> AP2CONN_OSC_EN_BIT) & 0x1;
		}

		len += osal_sprintf(strbuf + len, "0x%08x: 0x%x", state->lp[0], state->lp[1]);
		if ((mcu_osc_en + bt_osc_en + gps_osc_en + wifi_osc_en + fm_osc_en + ap2conn_osc_en) >= 1) {
			len += osal_sprintf(strbuf + len, " (%s%s%s%s%s%s",
					((mcu_osc_en == 1) ? "MCU," : ""), ((bt_osc_en == 1) ? "BT," : ""),
					((gps_osc_en == 1) ? "GPS," : ""), ((wifi_osc_en == 1) ? "WIFI," : ""),
					((fm_osc_en == 1) ? "FM," : ""), ((ap2conn_osc_en == 1) ? "AP2CONN," : ""));
			len += osal_sprintf(strbuf + len - 1, ")");
		}
		WMT_PLAT_PR_INFO("%s\n", strbuf);
	}
	return MTK_WCN_BOOL_TRUE;
}

static INT32 consys_dump_gating_state(P_CONSYS_STATE state)
{
	UINT32 value = 0;
	UINT8 strbuf[DBG_LOG_STR_SIZE] = {""};
	UINT32 len = 0;

	UINT32 magic_number = 0;
	UINT32 clock_hif_ctrl = 0;
	UINT32 clock_umac_ctrl = 0;
	UINT32 sleep_disable_mode = 0;
	UINT32 subsys_clk_req_stutus = 0;
	UINT32 clk_rate = 0;

	if (conn_reg.mcu_conn_hif_on_base == 0)
		return MTK_WCN_BOOL_FALSE;

	value = CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_MCU_MAILBOX_DBG);

	if (state) {
		state->gating[0] = (UINT32)0x18007120;
		state->gating[1] = value;
		osal_memset(&(state->sw_state), 0, sizeof(state->sw_state));
	}

	magic_number = (value >> 0) & 0xF;
	if (magic_number == 0x7) {
		clock_hif_ctrl = (value >> 4) & 0x1;
		clock_umac_ctrl = (value >> 5) & 0x1;
		sleep_disable_mode = (value >> 8) & 0xFF;
		subsys_clk_req_stutus = (value >> 16) & 0xFF;
		clk_rate = (value >> 24) & 0xFF;
	}

	len += osal_sprintf(strbuf + len, "0x%08x: 0x%x", 0x18007120, value);
	if (magic_number == 0x7) {
		if (clock_hif_ctrl == 0x0 && clock_umac_ctrl == 0x0 && sleep_disable_mode == 0x0 &&
				subsys_clk_req_stutus == 0x0 && clk_rate <= 0x1a) {
			len += osal_sprintf(strbuf + len, " (sleep)");
			if (state) {
				state->sw_state.is_gating = 0;
				state->sw_state.clock_mcu = clk_rate;
			}
		} else {
			len += osal_sprintf(strbuf + len,
				" (gating, hif_clk:0x%01x, umac_clk:0x%01x, slp_dis:0x%02x, subsys_clk:0x%02x, clk_rate:0x%02x)",
				clock_hif_ctrl, clock_umac_ctrl, sleep_disable_mode, subsys_clk_req_stutus, clk_rate);
			if (state) {
				state->sw_state.is_gating = 1;
				state->sw_state.clock_hif_ctrl = clock_hif_ctrl;
				state->sw_state.clock_umac_ctrl = clock_umac_ctrl;
				state->sw_state.resource_disable_sleep = sleep_disable_mode;
				state->sw_state.sub_system = subsys_clk_req_stutus;
				state->sw_state.clock_mcu = clk_rate;
			}
		}
	} else {
		len += osal_sprintf(strbuf + len, " (no gating info found)");
	}
	WMT_PLAT_PR_INFO("%s\n", strbuf);

	return MTK_WCN_BOOL_TRUE;
}

static VOID consys_set_pdma_axi_rready_force_high(UINT32 enable)
{
	if (conn_reg.mcu_conn_hif_pdma_base == 0)
		return;

	if (enable)
		CONSYS_SET_BIT(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_PDMA_AXI_RREADY,
				CONSYS_HIF_PDMA_AXI_RREADY_MASK);
	else if ((CONSYS_REG_READ(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_PDMA_AXI_RREADY) &
		 CONSYS_HIF_PDMA_AXI_RREADY_MASK) != 0)
		CONSYS_CLR_BIT(conn_reg.mcu_conn_hif_pdma_base + CONSYS_HIF_PDMA_AXI_RREADY,
			       CONSYS_HIF_PDMA_AXI_RREADY_MASK);
}

static VOID consys_set_mcif_emi_mpu_protection(MTK_WCN_BOOL enable)
{
	WMT_PLAT_PR_INFO("Setup region 23 for domain 0 as %s\n", enable ? "FORBIDDEN" : "SEC_R_NSEC_R");
	/* emi_mpu_set_single_permission(23, 0, enable ? FORBIDDEN : SEC_R_NSEC_R); */
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

static INT32 consys_is_rc_mode_enable(VOID)
{
#if CONSYS_RC_MODE_ENABLE
	INT32 value;
	UINT8 *reg_base;

	reg_base = ioremap(SRCLKEN_RC_BASE, 0x10);
	/* Since RC mode is default on, return 1 if ioremap is failed */
	if (!reg_base)
		return 1;

	value = SRCLKEN_RC_CENTRAL_CFG1_BIT &
		CONSYS_REG_READ(reg_base + SRCLKEN_RC_CENTRAL_CFG1);

	iounmap(reg_base);
	return value;
#else
	return 0;
#endif
}

static UINT32 consys_sleep_info_is_enable(VOID)
{
	UINT32 ctrl_enable = 0;
	UINT32 host_ctrl_enable = 0;

	if (conn_reg.mcu_conn_hif_on_base == 0)
		return 0;

	ctrl_enable = (CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base +
			CONN_HOST_CR_SLEEP_CNT_OFFSET) >> 0) & 0x1;

	if (ctrl_enable == 0)
		return 0;

	host_ctrl_enable = (CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base +
			CONN_HOST_CR_SLEEP_CNT_OFFSET) >> 5) & 0x1;

	if (host_ctrl_enable == 0)
		return 0;

	return 1;
}

INT32 consys_sleep_info_clear(VOID)
{
	if (conn_reg.mcu_conn_hif_on_base == 0)
		return -1;

	/* set xx_in_sleep_clr = 1 */
	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET,
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET) |
		(0x1F << 5));

	/* wait 100us */
	udelay(100);

	/* set xx_in_sleep_clr = 0 */
	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET,
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET) &
		~(0x1F << 5));

	return 0;
}

INT32 consys_sleep_info_enable_ctrl(UINT32 enable)
{
	if (conn_reg.mcu_conn_hif_on_base == 0)
		return -1;

	WMT_PLAT_PR_DBG("consys_sleep_info_enable_ctrl, enable=[%d]\n", enable);

	if (enable == 1) {
		/* set sleep_cnt_en = 1 (function enable) */
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET) |
			(0x1 << 0));

		/* wait 100us */
		udelay(100);

		/* set cr_host_slp_cnt_host_ctl_en = 1 (driver only) */
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET) |
			(0x1 << 5));

		/* set xx_in_sleep_clr = 1 */
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET) |
			(0x1F << 5));

		/* set xx_in_sleep_stop = 0 */
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET) &
			~(0x1F << 0));

		/* wait 100us */
		udelay(100);

		/* set xx_in_sleep_clr = 0 */
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET) &
			~(0x1F << 5));
	} else {
		/* set xx_in_sleep_stop = 1 */
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_SLEEP_INFO_CTRL_OFFSET) |
			(0x1F << 0));

		/* set cr_host_slp_cnt_host_ctl_en = 0 (driver only) */
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET) &
			~(0x1 << 5));

		/* wait 100us */
		udelay(100);

		/* set sleep_cnt_en = 0 (function disable) */
		CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET,
			CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET) &
			~(0x1 << 0));
	}

	return 0;
}

INT32 consys_sleep_info_read_ctrl(WMT_SLEEP_COUNT_TYPE type, PUINT64 sleep_counter, PUINT64 sleep_timer)
{
	/*
	 * type = 0 --> WMT_SLEEP_COUNT_TOP
	 * type = 1 --> WMT_SLEEP_COUNT_MCU
	 * type = 2 --> WMT_SLEEP_COUNT_BT
	 * type = 3 --> WMT_SLEEP_COUNT_WF
	 * type = 4 --> WMT_SLEEP_COUNT_GPS
	 */

	UINT32 in_sleep_state = 0;
	UINT32 value_sleep_timer_rd = 0;
	UINT32 value_sleep_counter_rd = 0;
	UINT32 value_final_counter = 0;

	if (conn_reg.mcu_conn_hif_on_base == 0)
		return -1;

	if (type >= WMT_SLEEP_COUNT_MAX)
		return -2;

	if (sleep_counter == NULL && sleep_timer == NULL)
		return -3;

	if (!consys_sleep_info_is_enable())
		return -4;

	WMT_PLAT_PR_DBG("type=[%d]\n", type);

	/* set sleep_cnt_sel = top/mcu/bt/wf/gps */
	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET,
			(CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET) &
			0xFFFFFFF1) | (type * 2));

	/* set sleep_cnt_rd_trig = 1 */
	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET,
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET) |
		(0x1 << 4));

	/* wait 40us */
	udelay(40);

	/* set sleep_cnt_rd_trig = 0 */
	CONSYS_REG_WRITE(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET,
		CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base + CONN_HOST_CR_SLEEP_CNT_OFFSET) &
		~(0x1 << 4));

	/* wait 100us */
	udelay(100);

	if (sleep_counter) {
		/* read sleep_cnt_counter */
		value_sleep_counter_rd = CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base +
				CONN_SLEEP_INFO_READ_CTRL_COUNT_OFFSET);
		in_sleep_state = (CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base +
				CONN_SLEEP_INFO_CTRL_OFFSET) >> (14 - type)) & 0x1;

		if (value_sleep_counter_rd == 0) {
			value_final_counter = 0;
		} else {
			if ((type == WMT_SLEEP_COUNT_BT || type == WMT_SLEEP_COUNT_WF) && in_sleep_state == 1)
				value_final_counter = value_sleep_counter_rd + 1;
			else
				value_final_counter = value_sleep_counter_rd;
		}

		if (value_final_counter > 0)
			value_final_counter--;

		*sleep_counter = value_final_counter;
		WMT_PLAT_PR_DBG("sleep_counter_rd=[%d], in_sleep_state=[%d], final_counter=[%d]\n",
				value_sleep_counter_rd, in_sleep_state, value_final_counter);
	}

	if (sleep_timer) {
		/* read sleep_cnt_timer */
		value_sleep_timer_rd = CONSYS_REG_READ(conn_reg.mcu_conn_hif_on_base +
				CONN_SLEEP_INFO_READ_CTRL_TIMER_OFFSET);
		*sleep_timer = value_sleep_timer_rd;
		WMT_PLAT_PR_DBG("sleep_timer_rd=[%d]\n", value_sleep_timer_rd);
	}

	return 0;
}

#if (COMMON_KERNEL_CLK_SUPPORT)
static MTK_WCN_BOOL consys_need_store_pdev(VOID)
{
	return MTK_WCN_BOOL_TRUE;
}

static UINT32 consys_store_pdev(struct platform_device *pdev)
{
	connsys_pdev = pdev;
	return 0;
}
#endif

static UINT64 consys_get_options(VOID)
{
	UINT64 options = OPT_QUERY_ADIE |
			OPT_WIFI_LTE_COEX |
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
	return dump_conn_mcu_pc_log_mt6853("");
}

static INT32 consys_common_dump(const char *trg_str)
{
	int ret = 0;

	ret += dump_conn_mcu_pc_log_mt6853(trg_str);

	ret += dump_conn_debug_dump_mt6853(trg_str);
	ret += dump_conn_mcu_debug_flag_mt6853(trg_str);
	ret += dump_conn_mcu_ahb_bus_hang_layer1_mt6853(trg_str);
	ret += dump_conn_mcu_ahb_bus_hang_layer2_mt6853(trg_str);
	ret += dump_conn_mcu_ahb_bus_hang_layer3_mt6853(trg_str);
	ret += dump_conn_mcu_ahb_bus_hang_layer4_mt6853(trg_str);
	ret += dump_conn_mcu_ahb_timeout_info_mt6853(trg_str);
	ret += dump_conn_bus_hang_debug_mt6853(trg_str);
	ret += dump_conn_mcu_apb_timeout_info_mt6853(trg_str);
	ret += dump_conn_apb_bus0_hang_mt6853(trg_str);
	ret += dump_conn_apb_bus1_hang_mt6853(trg_str);
	ret += dump_conn_apb_bus2_hang_mt6853(trg_str);
	ret += dump_conn_emi_ctrl_host_csr_mt6853(trg_str);
	ret += dump_conn_mcu_confg_emi_ctrl_mt6853(trg_str);
	ret += dump_conn_mcu_cpu_probe_mt6853(trg_str);
	ret += dump_conn_mcu_ahb_probe_mt6853(trg_str);
	ret += dump_conn_mcu_idlm_prot_prob_mt6853(trg_str);
	ret += dump_conn_mcu_wf_cmdbt_ram_prob_mt6853(trg_str);
	ret += dump_conn_mcu_pda_dbg_flag_mt6853(trg_str);
	ret += dump_conn_mcu_sysram_prb_mt6853(trg_str);
	ret += dump_conn_mcu_confg_mt6853(trg_str);
	ret += dump_conn_mcu_i_eidlm_mt6853(trg_str);
	ret += dump_conn_mcu_dma_mt6853(trg_str);
	ret += dump_conn_mcu_tcm_prob_mt6853(trg_str);
	ret += dump_conn_mcu_met_prob_mt6853(trg_str);
	ret += dump_conn_mcusys_n9_mt6853(trg_str);
	ret += dump_conn_mcu_uart_dbg_loop_mt6853(trg_str);
	ret += dump_conn_cfg_on_Debug_Signal_mt6853(trg_str);
	ret += dump_conn_cfg_on_register_mt6853(trg_str);
	ret += dump_conn_cmdbt_debug_signal_mt6853(trg_str);
	ret += dump_conn_cmdbt_register_mt6853(trg_str);
	ret += dump_conn_emi_detect_mt6853(trg_str);
	ret += dump_conn_cmdbt_debug_mt6853(trg_str);
	ret += dump_conn_hif_reg_debug_mt6853(trg_str);
	ret += dump_conn_mcu_confg_bus_hang_reg_mt6853(trg_str);
	ret += dump_wf_pdma_reg_debug_mt6853(trg_str);
	ret += dump_conn_to_EMI_bus_path_mt6853(trg_str);

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

INT32 consys_ipi_timeout_dump(VOID)
{
	return consys_common_dump("ipi_timeout");
}

