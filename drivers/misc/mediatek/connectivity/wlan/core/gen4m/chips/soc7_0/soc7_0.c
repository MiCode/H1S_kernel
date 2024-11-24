/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*! \file   soc7_0.c
*    \brief  Internal driver stack will export
*    the required procedures here for GLUE Layer.
*
*    This file contains all routines which are exported
     from MediaTek 802.11 Wireless LAN driver stack to GLUE Layer.
*/

#ifdef SOC7_0

#include "precomp.h"
#include "soc7_0.h"
#include "coda/soc7_0/conn_host_csr_top.h"
#include "coda/soc7_0/conn_infra_cfg.h"
#include "coda/soc7_0/conn_infra_cfg_on.h"
#include "coda/soc7_0/conn_infra_clkgen_top.h"
#include "coda/soc7_0/conn_infra_rgu_on.h"
#include "coda/soc7_0/conn_wt_slp_ctl_reg.h"
#include "coda/soc7_0/wf_hif_dmashdl_top.h"
#include "coda/soc7_0/wf_mcu_bus_cr.h"
#include "coda/soc7_0/wf_mcu_confg_ls.h"
#include "coda/soc7_0/wf_mcusys_infra_bus_full_u_debug_ctrl_ao_wfmcu_pwa_debug_ctrl_ao.h"
#include "coda/soc7_0/wf_ple_top.h"
#include "coda/soc7_0/wf_pse_top.h"
#include "coda/soc7_0/wf_top_cfg.h"
#include "coda/soc7_0/wf_top_cfg_on.h"
#include "coda/soc7_0/wf_top_slpprot_on.h"
#include "coda/soc7_0/wf_wfdma_host_dma0.h"
#include "coda/soc7_0/ap2wf_conn_infra_on_ccif4.h"
#include "coda/soc7_0/conn_semaphore.h"
#include "hal_dmashdl_soc7_0.h"

#define CFG_SUPPORT_VCODE_VDFS 1

#if (CFG_SUPPORT_VCODE_VDFS == 1)
#include <linux/pm_qos.h>

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#endif /*#ifndef CFG_SUPPORT_VCODE_VDFS*/


/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

#define SOC7_0_FILE_NAME_TOTAL 8
#define SOC7_0_FILE_NAME_MAX 64

static uint8_t *soc7_0_apucFwName[] = {
	(uint8_t *) CFG_FW_FILENAME "_MT",
	NULL
};

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
static void soc7_0_ConstructFirmwarePrio(struct GLUE_INFO *prGlueInfo,
	uint8_t **apucNameTable, uint8_t **apucName,
	uint8_t *pucNameIdx, uint8_t ucMaxNameIdx);

static uint8_t soc7_0SetRxRingHwAddr(struct RTMP_RX_RING *prRxRing,
		struct BUS_INFO *prBusInfo, uint32_t u4SwRingIdx);

static bool soc7_0WfdmaAllocRxRing(struct GLUE_INFO *prGlueInfo,
		bool fgAllocMem);

static void soc7_0asicConnac2xProcessTxInterrupt(
		struct ADAPTER *prAdapter);

static void soc7_0asicConnac2xProcessRxInterrupt(
	struct ADAPTER *prAdapter);

static void soc7_0WfdmaRxRingExtCtrl(
	struct GLUE_INFO *prGlueInfo,
	struct RTMP_RX_RING *rx_ring,
	u_int32_t index);

static void soc7_0asicConnac2xWfdmaManualPrefetch(
	struct GLUE_INFO *prGlueInfo);

static void soc7_0ReadIntStatus(struct ADAPTER *prAdapter,
		uint32_t *pu4IntStatus);

static void configIntMask(struct GLUE_INFO *prGlueInfo,
		u_int8_t enable);

static void soc7_0asicConnac2xWpdmaConfig(struct GLUE_INFO *prGlueInfo,
		u_int8_t enable, bool fgResetHif);

static void soc7_0EnableFwDlMode(struct ADAPTER *prAdapter);

static int soc7_0_CheckBusHang(void *adapter, uint8_t ucWfResetEnable);
static void soc7_0_DumpBusHangCr(struct ADAPTER *prAdapter);

#if (CFG_SUPPORT_CONNINFRA == 1)
static int soc7_0_ConnacPccifon(void);
static int soc7_0_ConnacPccifoff(void);
#endif

static bool soc7_0_get_sw_interrupt_status(struct ADAPTER *prAdapter,
	uint32_t *status);

static int wf_pwr_on_consys_mcu(void);
static int wf_pwr_off_consys_mcu(void);

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/
#if (CFG_SUPPORT_VCODE_VDFS == 1)

#if (KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE)
/* Implementation for kernel-5.4 */
struct regulator *dvfsrc_vcore_power;
#else
static struct pm_qos_request wifi_req;
#endif

#endif /* #if (CFG_SUPPORT_VCODE_VDFS == 1) */


struct ECO_INFO soc7_0_eco_table[] = {
	/* HW version,  ROM version,    Factory version */
	{0x00, 0x00, 0xA, 0x1},	/* E1 */
	{0x00, 0x00, 0x0, 0x0}	/* End of table */
};

uint8_t *apucsoc7_0FwName[] = {
	(uint8_t *) CFG_FW_FILENAME "_soc7_0",
	NULL
};

#if defined(_HIF_PCIE)
struct PCIE_CHIP_CR_MAPPING soc7_0_bus2chip_cr_mapping[] = {
	/* chip addr, bus addr, range */
	{0x830c0000, 0x00000, 0x1000}, /* WF_MCU_BUS_CR_REMAP */
	{0x54000000, 0x02000, 0x1000},  /* WFDMA PCIE0 MCU DMA0 */
	{0x55000000, 0x03000, 0x1000},  /* WFDMA PCIE0 MCU DMA1 */
	{0x56000000, 0x04000, 0x1000},  /* WFDMA reserved */
	{0x57000000, 0x05000, 0x1000},  /* WFDMA MCU wrap CR */
	{0x58000000, 0x06000, 0x1000},  /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{0x59000000, 0x07000, 0x1000},  /* WFDMA PCIE1 MCU DMA1 */
	{0x820c0000, 0x08000, 0x4000},  /* WF_UMAC_TOP (PLE) */
	{0x820c8000, 0x0c000, 0x2000},  /* WF_UMAC_TOP (PSE) */
	{0x820cc000, 0x0e000, 0x2000},  /* WF_UMAC_TOP (PP) */
	{0x820e0000, 0x20000, 0x0400},  /* WF_LMAC_TOP BN0 (WF_CFG) */
	{0x820e1000, 0x20400, 0x0200},  /* WF_LMAC_TOP BN0 (WF_TRB) */
	{0x820e2000, 0x20800, 0x0400},  /* WF_LMAC_TOP BN0 (WF_AGG) */
	{0x820e3000, 0x20c00, 0x0400},  /* WF_LMAC_TOP BN0 (WF_ARB) */
	{0x820e4000, 0x21000, 0x0400},  /* WF_LMAC_TOP BN0 (WF_TMAC) */
	{0x820e5000, 0x21400, 0x0800},  /* WF_LMAC_TOP BN0 (WF_RMAC) */
	{0x820ce000, 0x21c00, 0x0200},  /* WF_LMAC_TOP (WF_SEC) */
	{0x820e7000, 0x21e00, 0x0200},  /* WF_LMAC_TOP BN0 (WF_DMA) */
	{0x820cf000, 0x22000, 0x1000},  /* WF_LMAC_TOP (WF_PF) */
	{0x820e9000, 0x23400, 0x0200},  /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
	{0x820ea000, 0x24000, 0x0200},  /* WF_LMAC_TOP BN0 (WF_ETBF) */
	{0x820eb000, 0x24200, 0x0400},  /* WF_LMAC_TOP BN0 (WF_LPON) */
	{0x820ec000, 0x24600, 0x0200},  /* WF_LMAC_TOP BN0 (WF_INT) */
	{0x820ed000, 0x24800, 0x0800},  /* WF_LMAC_TOP BN0 (WF_MIB) */
	{0x820ca000, 0x26000, 0x2000},  /* WF_LMAC_TOP BN0 (WF_MUCOP) */
	{0x820d0000, 0x30000, 0x10000}, /* WF_LMAC_TOP (WF_WTBLON) */
	{0x40000000, 0x70000, 0x10000}, /* WF_UMAC_SYSRAM */
	{0x00400000, 0x80000, 0x10000}, /* WF_MCU_SYSRAM */
	{0x00410000, 0x90000, 0x10000}, /* WF_MCU_SYSRAM (configure register) */
	{0x820f0000, 0xa0000, 0x0400},  /* WF_LMAC_TOP BN1 (WF_CFG) */
	{0x820f1000, 0xa0600, 0x0200},  /* WF_LMAC_TOP BN1 (WF_TRB) */
	{0x820f2000, 0xa0800, 0x0400},  /* WF_LMAC_TOP BN1 (WF_AGG) */
	{0x820f3000, 0xa0c00, 0x0400},  /* WF_LMAC_TOP BN1 (WF_ARB) */
	{0x820f4000, 0xa1000, 0x0400},  /* WF_LMAC_TOP BN1 (WF_TMAC) */
	{0x820f5000, 0xa1400, 0x0800},  /* WF_LMAC_TOP BN1 (WF_RMAC) */
	{0x820f7000, 0xa1e00, 0x0200},  /* WF_LMAC_TOP BN1 (WF_DMA) */
	{0x820f9000, 0xa3400, 0x0200},  /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
	{0x820fa000, 0xa4000, 0x0200},  /* WF_LMAC_TOP BN1 (WF_ETBF) */
	{0x820fb000, 0xa4200, 0x0400},  /* WF_LMAC_TOP BN1 (WF_LPON) */
	{0x820fc000, 0xa4600, 0x0200},  /* WF_LMAC_TOP BN1 (WF_INT) */
	{0x820fd000, 0xa4800, 0x0800},  /* WF_LMAC_TOP BN1 (WF_MIB) */
	{0x820cc000, 0xa5000, 0x2000},  /* WF_LMAC_TOP BN1 (WF_MUCOP) */
	{0x820c4000, 0xa8000, 0x4000},  /* WF_LMAC_TOP BN1 (WF_MUCOP) */
	{0x820b0000, 0xae000, 0x1000},  /* [APB2] WFSYS_ON */
	{0x80020000, 0xb0000, 0x10000}, /* WF_TOP_MISC_OFF */
	{0x81020000, 0xc0000, 0x10000}, /* WF_TOP_MISC_ON */
	{0x7c020000, 0xd0000, 0x10000}, /* CONN_INFRA, wfdma */
	{0x7c500000, 0x50000, 0x10000}, /* CONN_INFRA, dyn mem map */
	{0x7c060000, 0xe0000, 0x10000}, /* CONN_INFRA, conn_host_csr_top */
	{0x7c000000, 0xf0000, 0x10000}, /* CONN_INFRA */
	{0x0, 0x0, 0x0} /* End */
};
#elif defined(_HIF_AXI)
struct PCIE_CHIP_CR_MAPPING soc7_0_bus2chip_cr_mapping[] = {
	/* chip addr, bus addr, range */
	{0x830c0000, 0x400000, 0x1000},    /* WF_MCU_BUS_CR_REMAP */
	{0x54000000, 0x402000, 0x1000},    /* WFDMA PCIE0 MCU DMA0 */
	{0x55000000, 0x403000, 0x1000},    /* WFDMA PCIE0 MCU DMA1 */
	{0x56000000, 0x404000, 0x1000},    /* WFDMA reserved */
	{0x57000000, 0x405000, 0x1000},    /* WFDMA MCU wrap CR */
	{0x58000000, 0x406000, 0x1000},    /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{0x59000000, 0x407000, 0x1000},    /* WFDMA PCIE1 MCU DMA1 */
	{0x820c0000, 0x408000, 0x4000},    /* WF_UMAC_TOP (PLE) */
	{0x820c8000, 0x40c000, 0x2000},    /* WF_UMAC_TOP (PSE) */
	{0x820cc000, 0x40e000, 0x2000},    /* WF_UMAC_TOP (PP) */
	{0x83000000, 0x410000, 0x10000},   /* WF_PHY_MAP3 */
	{0x820e0000, 0x420000, 0x0400},    /* WF_LMAC_TOP BN0 (WF_CFG) */
	{0x820e1000, 0x420400, 0x0200},    /* WF_LMAC_TOP BN0 (WF_TRB) */
	{0x820e2000, 0x420800, 0x0400},    /* WF_LMAC_TOP BN0 (WF_AGG) */
	{0x820e3000, 0x420c00, 0x0400},    /* WF_LMAC_TOP BN0 (WF_ARB) */
	{0x820e4000, 0x421000, 0x0400},    /* WF_LMAC_TOP BN0 (WF_TMAC) */
	{0x820e5000, 0x421400, 0x0800},    /* WF_LMAC_TOP BN0 (WF_RMAC) */
	{0x820ce000, 0x421c00, 0x0200},    /* WF_LMAC_TOP (WF_SEC) */
	{0x820e7000, 0x421e00, 0x0200},    /* WF_LMAC_TOP BN0 (WF_DMA) */
	{0x820cf000, 0x422000, 0x1000},    /* WF_LMAC_TOP (WF_PF) */
	{0x820e9000, 0x423400, 0x0200},    /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
	{0x820ea000, 0x424000, 0x0200},    /* WF_LMAC_TOP BN0 (WF_ETBF) */
	{0x820eb000, 0x424200, 0x0400},    /* WF_LMAC_TOP BN0 (WF_LPON) */
	{0x820ec000, 0x424600, 0x0200},    /* WF_LMAC_TOP BN0 (WF_INT) */
	{0x820ed000, 0x424800, 0x0800},    /* WF_LMAC_TOP BN0 (WF_MIB) */
	{0x820ca000, 0x426000, 0x2000},    /* WF_LMAC_TOP BN0 (WF_MUCOP) */
	{0x820d0000, 0x430000, 0x10000},   /* WF_LMAC_TOP (WF_WTBLON) */
	{0x83080000, 0x450000, 0x10000},   /* WF_PHY_MAP1 */
	{0x83090000, 0x460000, 0x10000},   /* WF_PHY_MAP2 */
	{0xE0200000, 0x470000, 0x10000},   /* WF_UMAC_SYSRAM */
	{0x00400000, 0x480000, 0x10000},   /* WF_MCU_SYSRAM */
	{0x00410000, 0x490000, 0x10000},   /* WF_MCU_SYSRAM (config register) */
	{0x820f0000, 0x4a0000, 0x0400},    /* WF_LMAC_TOP BN1 (WF_CFG) */
	{0x820f1000, 0x4a0600, 0x0200},    /* WF_LMAC_TOP BN1 (WF_TRB) */
	{0x820f2000, 0x4a0800, 0x0400},    /* WF_LMAC_TOP BN1 (WF_AGG) */
	{0x820f3000, 0x4a0c00, 0x0400},    /* WF_LMAC_TOP BN1 (WF_ARB) */
	{0x820f4000, 0x4a1000, 0x0400},    /* WF_LMAC_TOP BN1 (WF_TMAC) */
	{0x820f5000, 0x4a1400, 0x0800},    /* WF_LMAC_TOP BN1 (WF_RMAC) */
	{0x820f7000, 0x4a1e00, 0x0200},    /* WF_LMAC_TOP BN1 (WF_DMA) */
	{0x820f9000, 0x4a3400, 0x0200},    /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
	{0x820fa000, 0x4a4000, 0x0200},    /* WF_LMAC_TOP BN1 (WF_ETBF) */
	{0x820fb000, 0x4a4200, 0x0400},    /* WF_LMAC_TOP BN1 (WF_LPON) */
	{0x820fc000, 0x4a4600, 0x0200},    /* WF_LMAC_TOP BN1 (WF_INT) */
	{0x820fd000, 0x4a4800, 0x0800},    /* WF_LMAC_TOP BN1 (WF_MIB) */
	{0x820c4000, 0x4a8000, 0x4000},    /* WF_LMAC_TOP BN1 (WF_UWTBL) */
	{0x820b0000, 0x4ae000, 0x1000},    /* [APB2] WFSYS_ON */
	{0x80020000, 0x4b0000, 0x10000},   /* WF_TOP_MISC_OFF */
	{0x81020000, 0x4c0000, 0x10000},   /* WF_TOP_MISC_ON */
	{0x80010000, 0x4d4000, 0x1000},    /* WF_AXIDMA */
	{0x83010000, 0x4e0000, 0x10000},   /* WF_PHY_MAP4 */
	{0x88000000, 0x4f0000, 0x10000},   /* WF_MCU_CFG_LS */
	{0x7c000000, 0x000000, 0x1000000}, /* CONN_INFRA */
	{0x0, 0x0, 0x0} /* End */
};
#endif

struct wfdma_group_info soc7_0_wfmda_host_tx_group[] = {
	{"P0T0:AP DATA0", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING0_CTRL0_ADDR, true},
	{"P0T1:AP DATA1", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING1_CTRL0_ADDR, true},
	{"P0T2:AP DATA2", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING2_CTRL0_ADDR, true},
	{"P0T15:AP CMD", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING15_CTRL0_ADDR, true},
	{"P0T16:FWDL", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING16_CTRL0_ADDR, true},
	{"P0T8:MD DATA0", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING8_CTRL0_ADDR},
	{"P0T9:MD DATA1", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING9_CTRL0_ADDR},
	{"P0T10:MD DATA2", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING10_CTRL0_ADDR},
	{"P0T14:MD CMD", WF_WFDMA_HOST_DMA0_WPDMA_TX_RING14_CTRL0_ADDR},
};

struct wfdma_group_info soc7_0_wfmda_host_rx_group[] = {
	{"P0R0:AP DATA0", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL0_ADDR, true},
	{"P0R2:AP EVENT", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING2_CTRL0_ADDR, true},
	{"P0R1:AP DATA1", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING1_CTRL0_ADDR, true},
	{"P0R3:AP TDONE", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING3_CTRL0_ADDR, true},
	{"P0R4:MD DATA0", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING4_CTRL0_ADDR},
	{"P0R5:MD DATA1", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING5_CTRL0_ADDR},
	{"P0R6:MD EVENT", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING6_CTRL0_ADDR},
	{"P0R7:MD TDONE", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING7_CTRL0_ADDR},
};

struct pse_group_info rSoc7_0_pse_group[] = {
	{"HIF0(TX data)", WF_PSE_TOP_PG_HIF0_GROUP_ADDR,
		WF_PSE_TOP_HIF0_PG_INFO_ADDR},
	{"HIF1(Talos CMD)", WF_PSE_TOP_PG_HIF1_GROUP_ADDR,
		WF_PSE_TOP_HIF1_PG_INFO_ADDR},
#if 0
	{"HIF2", WF_PSE_TOP_PG_HIF2_GROUP_ADDR,
		WF_PSE_TOP_HIF2_PG_INFO_ADDR},
#endif
	{"CPU(I/O r/w)",  WF_PSE_TOP_PG_CPU_GROUP_ADDR,
		WF_PSE_TOP_CPU_PG_INFO_ADDR},
	{"PLE(host report)",  WF_PSE_TOP_PG_PLE_GROUP_ADDR,
		WF_PSE_TOP_PLE_PG_INFO_ADDR},
	{"PLE1(SPL report)", WF_PSE_TOP_PG_PLE1_GROUP_ADDR,
		WF_PSE_TOP_PLE1_PG_INFO_ADDR},
	{"LMAC0(RX data)", WF_PSE_TOP_PG_LMAC0_GROUP_ADDR,
			WF_PSE_TOP_LMAC0_PG_INFO_ADDR},
	{"LMAC1(RX_VEC)", WF_PSE_TOP_PG_LMAC1_GROUP_ADDR,
			WF_PSE_TOP_LMAC1_PG_INFO_ADDR},
	{"LMAC2(TXS)", WF_PSE_TOP_PG_LMAC2_GROUP_ADDR,
			WF_PSE_TOP_LMAC2_PG_INFO_ADDR},
	{"LMAC3(TXCMD/RXRPT)", WF_PSE_TOP_PG_LMAC3_GROUP_ADDR,
			WF_PSE_TOP_LMAC3_PG_INFO_ADDR},
	{"MDP",  WF_PSE_TOP_PG_MDP_GROUP_ADDR,
			WF_PSE_TOP_MDP_PG_INFO_ADDR},
#if 0
	{"MDP1", WF_PSE_TOP_PG_MDP1_GROUP_ADDR,
		WF_PSE_TOP_MDP1_PG_INFO_ADDR},
	{"MDP2", WF_PSE_TOP_PG_MDP2_GROUP_ADDR,
		WF_PSE_TOP_MDP2_PG_INFO_ADDR},
#endif
};

struct BUS_INFO soc7_0_bus_info = {
#if defined(_HIF_PCIE) || defined(_HIF_AXI)
	.top_cfg_base = SOC7_0_TOP_CFG_BASE,

	/* host_dma0 for TXP */
	.host_dma0_base = WF_WFDMA_HOST_DMA0_BASE,
	.host_int_status_addr = WF_WFDMA_HOST_DMA0_HOST_INT_STA_ADDR,

	.host_int_txdone_bits =
		(CONNAC2X_WFDMA_TX_DONE_INT0 | CONNAC2X_WFDMA_TX_DONE_INT1 |
		CONNAC2X_WFDMA_TX_DONE_INT2 | CONNAC2X_WFDMA_TX_DONE_INT3 |
		CONNAC2X_WFDMA_TX_DONE_INT16 | CONNAC2X_WFDMA_TX_DONE_INT17),
	.host_int_rxdone_bits =
		(CONNAC2X_WFDMA_RX_DONE_INT0 | CONNAC2X_WFDMA_RX_DONE_INT1 |
		 CONNAC2X_WFDMA_RX_DONE_INT2 | CONNAC2X_WFDMA_RX_DONE_INT3),

	.host_tx_ring_base = WF_WFDMA_HOST_DMA0_WPDMA_TX_RING0_CTRL0_ADDR,
	.host_tx_ring_ext_ctrl_base =
		WF_WFDMA_HOST_DMA0_WPDMA_TX_RING0_EXT_CTRL_ADDR,
	.host_tx_ring_cidx_addr = WF_WFDMA_HOST_DMA0_WPDMA_TX_RING0_CTRL2_ADDR,
	.host_tx_ring_didx_addr = WF_WFDMA_HOST_DMA0_WPDMA_TX_RING0_CTRL3_ADDR,
	.host_tx_ring_cnt_addr = WF_WFDMA_HOST_DMA0_WPDMA_TX_RING0_CTRL1_ADDR,

	.host_rx_ring_base = WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL0_ADDR,
	.host_rx_ring_ext_ctrl_base =
		WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_EXT_CTRL_ADDR,
	.host_rx_ring_cidx_addr = WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL2_ADDR,
	.host_rx_ring_didx_addr = WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL3_ADDR,
	.host_rx_ring_cnt_addr = WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL1_ADDR,

	.bus2chip = soc7_0_bus2chip_cr_mapping,
	.max_static_map_addr = 0x000f0000,

	.tx_ring_fwdl_idx = CONNAC2X_FWDL_TX_RING_IDX,
	.tx_ring_cmd_idx = 15,
	.tx_ring0_data_idx = 0,
	.tx_ring1_data_idx = 1,
	.tx_ring2_data_idx = 2,
	.fw_own_clear_addr = CONNAC2X_BN0_IRQ_STAT_ADDR,
	.fw_own_clear_bit = PCIE_LPCR_FW_CLR_OWN,
	.fgCheckDriverOwnInt = FALSE,
	.u4DmaMask = 36,
#if defined(_HIF_PCIE)
	.pcie2ap_remap_2 = CONN_INFRA_CFG_PCIE2AP_REMAP_2_ADDR,
#endif
	.ap2wf_remap_1 = CONN_INFRA_CFG_AP2WF_REMAP_1_ADDR,
	.wfmda_host_tx_group = soc7_0_wfmda_host_tx_group,
	.wfmda_host_tx_group_len = ARRAY_SIZE(soc7_0_wfmda_host_tx_group),
	.wfmda_host_rx_group = soc7_0_wfmda_host_rx_group,
	.wfmda_host_rx_group_len = ARRAY_SIZE(soc7_0_wfmda_host_rx_group),
	.wfmda_wm_tx_group = NULL,
	.wfmda_wm_tx_group_len = 0,
	.wfmda_wm_rx_group = NULL,
	.wfmda_wm_rx_group_len = 0,
	.prDmashdlCfg = &rSOC7_0_DmashdlCfg,
	.prPleTopCr = &rSoc7_0_PleTopCr,
	.prPseTopCr = &rSoc7_0_PseTopCr,
	.prPpTopCr = &rSoc7_0_PpTopCr,
	.prPseGroup = rSoc7_0_pse_group,
	.u4PseGroupLen = ARRAY_SIZE(rSoc7_0_pse_group),
	.pdmaSetup = soc7_0asicConnac2xWpdmaConfig,
	.enableInterrupt = asicConnac2xEnablePlatformIRQ,
	.disableInterrupt = asicConnac2xDisablePlatformIRQ,
	.disableSwInterrupt = asicConnac2xDisablePlatformSwIRQ,
	.processTxInterrupt = soc7_0asicConnac2xProcessTxInterrupt,
	.processRxInterrupt = soc7_0asicConnac2xProcessRxInterrupt,
	.tx_ring_ext_ctrl = asicConnac2xWfdmaTxRingExtCtrl,
	.rx_ring_ext_ctrl = soc7_0WfdmaRxRingExtCtrl,
	/* null wfdmaManualPrefetch if want to disable manual mode */
	.wfdmaManualPrefetch = soc7_0asicConnac2xWfdmaManualPrefetch,
	.lowPowerOwnRead = asicConnac2xLowPowerOwnRead,
	.lowPowerOwnSet = asicConnac2xLowPowerOwnSet,
	.lowPowerOwnClear = asicConnac2xLowPowerOwnClear,
	.wakeUpWiFi = asicWakeUpWiFi,
	.processSoftwareInterrupt = asicConnac2xProcessSoftwareInterrupt,
	.softwareInterruptMcu = asicConnac2xSoftwareInterruptMcu,
	.hifRst = asicConnac2xHifRst,
	.initPcieInt = NULL,
	.devReadIntStatus = soc7_0ReadIntStatus,
	.DmaShdlInit = soc7_0DmashdlInit,
	.setRxRingHwAddr = soc7_0SetRxRingHwAddr,
	.wfdmaAllocRxRing = soc7_0WfdmaAllocRxRing,
	.enableFwDlMode = soc7_0EnableFwDlMode,
#endif /*_HIF_PCIE || _HIF_AXI */
};

#if CFG_ENABLE_FW_DOWNLOAD
struct FWDL_OPS_T soc7_0_fw_dl_ops = {
	.constructFirmwarePrio = soc7_0_ConstructFirmwarePrio,
	.constructPatchName = NULL,
	.downloadPatch = NULL,
	.downloadFirmware = wlanConnacFormatDownload,
#if (CFG_DOWNLOAD_DYN_MEMORY_MAP == 1)
	.downloadByDynMemMap = downloadImgByDynMemMap,
#else
	.downloadByDynMemMap = NULL,
#endif
	.getFwInfo = wlanGetConnacFwInfo,
	.getFwDlInfo = asicGetFwDlInfo,
#if (CFG_SUPPORT_PRE_ON_PHY_ACTION == 1)
	.phyAction = wlanPhyAction,
#else
	.phyAction = NULL,
#endif
};
#endif /* CFG_ENABLE_FW_DOWNLOAD */

struct TX_DESC_OPS_T soc7_0_TxDescOps = {
	.fillNicAppend = fillNicTxDescAppend,
	.fillHifAppend = fillTxDescAppendByHostV2,
	.fillTxByteCount = fillConnac2xTxDescTxByteCount,
};

struct RX_DESC_OPS_T soc7_0_RxDescOps = {};

struct CHIP_DBG_OPS soc7_0_DebugOps = {
	.showPdmaInfo = connac2x_show_wfdma_info,
	.showPseInfo = connac2x_show_pse_info,
	.showPleInfo = connac2x_show_ple_info,
	.showTxdInfo = connac2x_show_txd_Info,
	.showWtblInfo = connac2x_show_wtbl_info,
	.showUmacFwtblInfo = connac2x_show_umac_wtbl_info,
	.showCsrInfo = NULL,
	.showDmaschInfo = connac2x_show_dmashdl_info,
	.getFwDebug = connac2x_get_ple_int,
	.setFwDebug = connac2x_set_ple_int,
	.showHifInfo = NULL,
	.printHifDbgInfo = NULL,
	.show_rx_rate_info = connac2x_show_rx_rate_info,
	.show_rx_rssi_info = connac2x_show_rx_rssi_info,
	.show_stat_info = connac2x_show_stat_info,
#ifdef CFG_SUPPORT_LINK_QUALITY_MONITOR
	.get_rx_rate_info = soc7_0_get_rx_rate_info,
#endif
	.show_wfdma_dbg_probe_info = soc7_0_show_wfdma_dbg_probe_info,
	.show_wfdma_wrapper_info = soc7_0_show_wfdma_wrapper_info,
#if CFG_SUPPORT_LLS
	.get_rx_link_stats = soc7_0_get_rx_link_stats,
#endif
};


#if CFG_SUPPORT_QA_TOOL
struct ATE_OPS_T soc7_0_AteOps = {
	/* ICapStart phase out , wlan_service instead */
	.setICapStart = connacSetICapStart,
	/* ICapStatus phase out , wlan_service instead */
	.getICapStatus = connacGetICapStatus,
	/* CapIQData phase out , wlan_service instead */
	.getICapIQData = connacGetICapIQData,
	.getRbistDataDumpEvent = nicExtEventICapIQData,
	.icapRiseVcoreClockRate = soc7_0_icapRiseVcoreClockRate,
	.icapDownVcoreClockRate = soc7_0_icapDownVcoreClockRate,
};
#endif /* CFG_SUPPORT_QA_TOOL */



struct mt66xx_chip_info mt66xx_chip_info_soc7_0 = {
	.bus_info = &soc7_0_bus_info,
#if CFG_ENABLE_FW_DOWNLOAD
	.fw_dl_ops = &soc7_0_fw_dl_ops,
#endif /* CFG_ENABLE_FW_DOWNLOAD */
#if CFG_SUPPORT_QA_TOOL
	.prAteOps = &soc7_0_AteOps,
#endif /* CFG_SUPPORT_QA_TOOL */
	.prTxDescOps = &soc7_0_TxDescOps,
	.prRxDescOps = &soc7_0_RxDescOps,
	.prDebugOps = &soc7_0_DebugOps,
	.chip_id = SOC7_0_CHIP_ID,
	.should_verify_chip_id = FALSE,
	.sw_sync0 = CONNAC2x_CONN_CFG_ON_CONN_ON_MISC_ADDR,
	.sw_ready_bits = WIFI_FUNC_NO_CR4_READY_BITS,
	.sw_ready_bit_offset =
		CONNAC2x_CONN_CFG_ON_CONN_ON_MISC_DRV_FM_STAT_SYNC_SHFT,
	.patch_addr = SOC7_0_PATCH_START_ADDR,
	.is_support_cr4 = FALSE,
	.is_support_wacpu = FALSE,
	.txd_append_size = SOC7_0_TX_DESC_APPEND_LENGTH,
	.rxd_size = SOC7_0_RX_DESC_LENGTH,
	.init_evt_rxd_size = SOC7_0_RX_DESC_LENGTH,
	.pse_header_length = CONNAC2X_NIC_TX_PSE_HEADER_LENGTH,
	.init_event_size = CONNAC2X_RX_INIT_EVENT_LENGTH,
	.eco_info = soc7_0_eco_table,
	.isNicCapV1 = FALSE,
	.top_hcr = CONNAC2X_TOP_HCR,
	.top_hvr = CONNAC2X_TOP_HVR,
	.top_fvr = CONNAC2X_TOP_FVR,
	.arb_ac_mode_addr = SOC7_0_ARB_AC_MODE_ADDR,
	.custom_oid_interface_version = MTK_CUSTOM_OID_INTERFACE_VERSION,
	.em_interface_version = MTK_EM_INTERFACE_VERSION,
	.asicCapInit = asicConnac2xCapInit,
#if CFG_ENABLE_FW_DOWNLOAD
	.asicEnableFWDownload = NULL,
#endif /* CFG_ENABLE_FW_DOWNLOAD */
	.asicGetChipID = asicGetChipID,
	.downloadBufferBin = NULL,
	.is_support_hw_amsdu = TRUE,
	.is_support_asic_lp = TRUE,
	.is_support_wfdma1 = FALSE,
	.is_support_nvram_fragment = TRUE,
	.asicWfdmaReInit = asicConnac2xWfdmaReInit,
	.asicWfdmaReInit_handshakeInit = asicConnac2xWfdmaDummyCrWrite,
	.group5_size = sizeof(struct HW_MAC_RX_STS_GROUP_5),
	.u4LmacWtblDUAddr = CONNAC2X_WIFI_LWTBL_BASE,
	.u4UmacWtblDUAddr = CONNAC2X_WIFI_UWTBL_BASE,
	.wmmcupwron = wf_pwr_on_consys_mcu,
	.wmmcupwroff = wf_pwr_off_consys_mcu,
#if (CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1)
	.pwrondownload = soc7_0_wlanPowerOnDownload,
#else
	.pwrondownload = NULL,
#endif
	.triggerfwassert = soc7_0_Trigger_fw_assert,
#if (CFG_SUPPORT_CONNINFRA == 1)
	.coexpccifon = soc7_0_ConnacPccifon,
	.coexpccifoff = soc7_0_ConnacPccifoff,
	.get_sw_interrupt_status = soc7_0_get_sw_interrupt_status,
	.chip_capability = BIT(CHIP_CAPA_FW_LOG_TIME_SYNC),
#endif
	.checkbushang = soc7_0_CheckBusHang,
	.dumpBusHangCr = soc7_0_DumpBusHangCr,
#if (CFG_SUPPORT_PRE_ON_PHY_ACTION == 1)
	.getCalResult = wlanGetCalResult,
	.calDebugCmd = wlanCalDebugCmd,
#endif
	.cmd_max_pkt_size = CFG_TX_MAX_PKT_SIZE, /* size 1600 */
	.isSupportMddpAOR = true,
};

struct mt66xx_hif_driver_data mt66xx_driver_data_soc7_0 = {
	.chip_info = &mt66xx_chip_info_soc7_0,
};

void soc7_0_icapRiseVcoreClockRate(void)
{

#if (CFG_SUPPORT_VCODE_VDFS == 1)
	int value = 0;

#if (KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE)
	/* Implementation for kernel-5.4 */
	struct mt66xx_hif_driver_data *prDriverData =
		get_platform_driver_data();
	struct mt66xx_chip_info *prChipInfo;
	void *pdev;

	prChipInfo = ((struct mt66xx_hif_driver_data *)prDriverData)
		->chip_info;
	pdev = (void *)prChipInfo->pdev;

	dvfsrc_vcore_power = regulator_get(
		&((struct platform_device *)pdev)->dev, "dvfsrc-vcore");

	/* Enable VCore to 0.725 */
	regulator_set_voltage(dvfsrc_vcore_power, 725000, INT_MAX);
#else
	/* init */
	if (!pm_qos_request_active(&wifi_req))
		pm_qos_add_request(&wifi_req, PM_QOS_VCORE_OPP,
						PM_QOS_VCORE_OPP_DEFAULT_VALUE);

	/* update Vcore */
	pm_qos_update_request(&wifi_req, 0);
#endif

	DBGLOG(HAL, STATE, "icapRiseVcoreClockRate done\n");

	/* Seq2: update clock rate sel bus clock to 213MHz */

	/* 0x1801_2050[6:4]=3'b111 */
	wf_ioremap_read(WF_CONN_INFA_BUS_CLOCK_RATE, &value);
	value |= 0x00000070;
	wf_ioremap_write(WF_CONN_INFA_BUS_CLOCK_RATE, value);

	/* Seq3: enable clock select sw mode */

	/* 0x1801_2050[0]=1'b1 */
	wf_ioremap_read(WF_CONN_INFA_BUS_CLOCK_RATE, &value);
	value |= 0x1;
	wf_ioremap_write(WF_CONN_INFA_BUS_CLOCK_RATE, value);

#else
	DBGLOG(HAL, STATE, "icapRiseVcoreClockRate skip\n");
#endif  /*#ifndef CFG_BUILD_X86_PLATFORM*/
}

void soc7_0_icapDownVcoreClockRate(void)
{

#if (CFG_SUPPORT_VCODE_VDFS == 1)
	int value = 0;

#if (KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE)
	/* Implementation for kernel-5.4 */
	struct mt66xx_chip_info *prChipInfo;
	struct mt66xx_hif_driver_data *prDriverData =
		get_platform_driver_data();
	void *pdev;

	prChipInfo = ((struct mt66xx_hif_driver_data *)prDriverData)
		->chip_info;
	pdev = (void *)prChipInfo->pdev;

	dvfsrc_vcore_power = regulator_get(
		&((struct platform_device *)pdev)->dev, "dvfsrc-vcore");

	/* resume to default Vcore value */
	regulator_set_voltage(dvfsrc_vcore_power, 575000, INT_MAX);
#else
	/*init*/
	if (!pm_qos_request_active(&wifi_req))
		pm_qos_add_request(&wifi_req, PM_QOS_VCORE_OPP,
						PM_QOS_VCORE_OPP_DEFAULT_VALUE);

	/*restore to default Vcore*/
	pm_qos_update_request(&wifi_req,
		PM_QOS_VCORE_OPP_DEFAULT_VALUE);
#endif

	/*disable VCore to normal setting*/
	DBGLOG(HAL, STATE, "icapDownVcoreClockRate done!\n");

	/* Seq2: update clock rate sel bus clock to default value */

	/* 0x1801_2050[6:4]=3'b000 */
	wf_ioremap_read(WF_CONN_INFA_BUS_CLOCK_RATE, &value);
	value &= ~(0x00000070);
	wf_ioremap_write(WF_CONN_INFA_BUS_CLOCK_RATE, value);

	/* Seq3: disble clock select sw mode */

	/* 0x1801_2050[0]=1'b0 */
	wf_ioremap_read(WF_CONN_INFA_BUS_CLOCK_RATE, &value);
	value &= ~(0x1);
	wf_ioremap_write(WF_CONN_INFA_BUS_CLOCK_RATE, value);

#else
	DBGLOG(HAL, STATE, "icapDownVcoreClockRate skip\n");
#endif  /*#ifndef CFG_BUILD_X86_PLATFORM*/
}


static void soc7_0_ConstructFirmwarePrio(struct GLUE_INFO *prGlueInfo,
	uint8_t **apucNameTable, uint8_t **apucName,
	uint8_t *pucNameIdx, uint8_t ucMaxNameIdx)
{
	int ret = 0;
	uint8_t ucIdx = 0;
	uint8_t aucFlavor[2] = {0};

	kalGetFwFlavor(&aucFlavor[0]);

	for (ucIdx = 0; apucsoc7_0FwName[ucIdx]; ucIdx++) {
		if ((*pucNameIdx + 3) >= ucMaxNameIdx) {
			/* the table is not large enough */
			DBGLOG(INIT, ERROR,
				"kalFirmwareImageMapping >> file name array is not enough.\n");
			ASSERT(0);
			continue;
		}

		/* Type 1. WIFI_RAM_CODE_soc7_0_1_1.bin */
		ret = kalSnprintf(*(apucName + (*pucNameIdx)),
				CFG_FW_NAME_MAX_LEN, "%s_%u%s_%u.bin",
				apucsoc7_0FwName[ucIdx],
				CFG_WIFI_IP_SET,
				aucFlavor,
				1);
		if (ret >= 0 && ret < CFG_FW_NAME_MAX_LEN)
			(*pucNameIdx) += 1;
		else
			DBGLOG(INIT, ERROR,
					"[%u] kalSnprintf failed, ret: %d\n",
					__LINE__, ret);
	}
}

static uint8_t soc7_0SetRxRingHwAddr(struct RTMP_RX_RING *prRxRing,
		struct BUS_INFO *prBusInfo, uint32_t u4SwRingIdx)
{
	uint32_t offset = 0;

	/*
	 * RX_RING_DATA_IDX_0   (RX_Ring0) - Band0 Rx Data
	 * RX_RING_DATA1_IDX_2 (RX_Ring1) - Band1 Rx Data
	 * RX_RING_EVT_IDX_1    (RX_Ring2) - Band0 Tx Free Done Event / Rx Event
	 * RX_RING_TXDONE0_IDX_3 (RX_Ring3) - Band1 Tx Free Done Event
	*/
	switch (u4SwRingIdx) {
	case RX_RING_EVT_IDX_1:
		offset = 2 * MT_RINGREG_DIFF;
		break;
	case RX_RING_DATA1_IDX_2:
		offset = 1 * MT_RINGREG_DIFF;
		break;
	case RX_RING_DATA_IDX_0:
	case RX_RING_TXDONE0_IDX_3:
		offset = u4SwRingIdx * MT_RINGREG_DIFF;
		break;
	default:
		return FALSE;
	}

	prRxRing->hw_desc_base = prBusInfo->host_rx_ring_base + offset;
	prRxRing->hw_cidx_addr = prBusInfo->host_rx_ring_cidx_addr + offset;
	prRxRing->hw_didx_addr = prBusInfo->host_rx_ring_didx_addr + offset;
	prRxRing->hw_cnt_addr = prBusInfo->host_rx_ring_cnt_addr + offset;

	return TRUE;
}

static bool soc7_0WfdmaAllocRxRing(struct GLUE_INFO *prGlueInfo,
		bool fgAllocMem)
{
	/* Band1 Data Rx path */
	if (!halWpdmaAllocRxRing(prGlueInfo,
			RX_RING_DATA1_IDX_2, RX_RING0_SIZE,
			RXD_SIZE, CFG_RX_MAX_PKT_SIZE, fgAllocMem)) {
		DBGLOG(HAL, ERROR, "AllocRxRing[2] fail\n");
		return false;
	}
	/* Band0 Tx Free Done Event */
	if (!halWpdmaAllocRxRing(prGlueInfo,
			RX_RING_TXDONE0_IDX_3, RX_RING1_SIZE,
			RXD_SIZE, RX_BUFFER_AGGRESIZE, fgAllocMem)) {
		DBGLOG(HAL, ERROR, "AllocRxRing[3] fail\n");
		return false;
	}
	return true;
}

static void soc7_0asicConnac2xProcessTxInterrupt(
		struct ADAPTER *prAdapter)
{
	struct GL_HIF_INFO *prHifInfo = &prAdapter->prGlueInfo->rHifInfo;
	union WPDMA_INT_STA_STRUCT rIntrStatus;

	rIntrStatus = (union WPDMA_INT_STA_STRUCT)prHifInfo->u4IntStatus;
	if (rIntrStatus.field_conn2x_single.wfdma0_tx_done_16)
		halWpdmaProcessCmdDmaDone(
			prAdapter->prGlueInfo, TX_RING_FWDL_IDX_4);

	if (rIntrStatus.field_conn2x_single.wfdma0_tx_done_17)
		halWpdmaProcessCmdDmaDone(
			prAdapter->prGlueInfo, TX_RING_CMD_IDX_3);

	if (rIntrStatus.field_conn2x_single.wfdma0_tx_done_0) {
		halWpdmaProcessDataDmaDone(
			prAdapter->prGlueInfo, TX_RING_DATA0_IDX_0);
		kalSetTxEvent2Hif(prAdapter->prGlueInfo);
	}

	if (rIntrStatus.field_conn2x_single.wfdma0_tx_done_1) {
		halWpdmaProcessDataDmaDone(
			prAdapter->prGlueInfo, TX_RING_DATA1_IDX_1);
		kalSetTxEvent2Hif(prAdapter->prGlueInfo);
	}

	if (rIntrStatus.field_conn2x_single.wfdma0_tx_done_2) {
		halWpdmaProcessDataDmaDone(
			prAdapter->prGlueInfo, TX_RING_DATA2_IDX_2);
		kalSetTxEvent2Hif(prAdapter->prGlueInfo);
	}
}

static void soc7_0asicConnac2xProcessRxInterrupt(
	struct ADAPTER *prAdapter)
{
	struct GL_HIF_INFO *prHifInfo = &prAdapter->prGlueInfo->rHifInfo;
	union WPDMA_INT_STA_STRUCT rIntrStatus;

	rIntrStatus = (union WPDMA_INT_STA_STRUCT)prHifInfo->u4IntStatus;

	if (rIntrStatus.field_conn2x_single.wfdma0_rx_done_0 ||
	    (prAdapter->u4NoMoreRfb & BIT(RX_RING_DATA_IDX_0)))
		halRxReceiveRFBs(prAdapter, RX_RING_DATA_IDX_0, TRUE);

	if (rIntrStatus.field_conn2x_single.wfdma0_rx_done_1 ||
	    (prAdapter->u4NoMoreRfb & BIT(RX_RING_DATA1_IDX_2)))
		halRxReceiveRFBs(prAdapter, RX_RING_DATA1_IDX_2, TRUE);

	if (rIntrStatus.field_conn2x_single.wfdma0_rx_done_2 ||
	    (prAdapter->u4NoMoreRfb & BIT(RX_RING_EVT_IDX_1)))
		halRxReceiveRFBs(prAdapter, RX_RING_EVT_IDX_1, FALSE);

	if (rIntrStatus.field_conn2x_single.wfdma0_rx_done_3 ||
	    (prAdapter->u4NoMoreRfb & BIT(RX_RING_TXDONE0_IDX_3)))
		halRxReceiveRFBs(prAdapter, RX_RING_TXDONE0_IDX_3, FALSE);
}

static void soc7_0SetMDRXRingPriorityInterrupt(struct ADAPTER *prAdapter)
{
	u_int32_t val = 0;

	HAL_MCR_RD(prAdapter,
		WF_WFDMA_HOST_DMA0_WPDMA_INT_RX_PRI_SEL_ADDR, &val);
	val |= BITS(4, 7);
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA0_WPDMA_INT_RX_PRI_SEL_ADDR, val);
}

static void soc7_0WfdmaRxRingExtCtrl(
	struct GLUE_INFO *prGlueInfo,
	struct RTMP_RX_RING *rx_ring,
	u_int32_t index)
{
	struct ADAPTER *prAdapter;
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	uint32_t ext_offset;

	prAdapter = prGlueInfo->prAdapter;
	prChipInfo = prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;

	switch (index) {
	case RX_RING_EVT_IDX_1:
		ext_offset = 2 * 4;
		break;
	case RX_RING_DATA_IDX_0:
		ext_offset = 0;
		break;
	case RX_RING_DATA1_IDX_2:
		ext_offset = 1 * 4;
		break;
	case RX_RING_TXDONE0_IDX_3:
		ext_offset = 3 * 4;
		break;
	default:
		DBGLOG(RX, ERROR, "Error index=%d\n", index);
		return;
	}

	rx_ring->hw_desc_base_ext =
		prBusInfo->host_rx_ring_ext_ctrl_base + ext_offset;

	HAL_MCR_WR(prAdapter, rx_ring->hw_desc_base_ext,
		   CONNAC2X_RX_RING_DISP_MAX_CNT);

	asicConnac2xWfdmaRxRingBasePtrExtCtrl(prGlueInfo,
		rx_ring, index);
}

static void soc7_0asicConnac2xWfdmaManualPrefetch(
	struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	u_int32_t val = 0;
	uint32_t u4WrVal = 0x00000004, u4Addr = 0;

	HAL_MCR_RD(prAdapter, WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_ADDR, &val);
	/* disable prefetch offset calculation auto-mode */
	val &=
	~WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_CSR_DISP_BASE_PTR_CHAIN_EN_MASK;
	HAL_MCR_WR(prAdapter, WF_WFDMA_HOST_DMA0_WPDMA_GLO_CFG_ADDR, val);

	/* Rx ring */
	for (u4Addr = WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA0_WPDMA_RX_RING3_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	/* MD Rx ring */
	for (u4Addr = WF_WFDMA_HOST_DMA0_WPDMA_RX_RING4_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA0_WPDMA_RX_RING7_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	/* Tx ring */
	for (u4Addr = WF_WFDMA_HOST_DMA0_WPDMA_TX_RING0_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA0_WPDMA_TX_RING2_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	for (u4Addr = WF_WFDMA_HOST_DMA0_WPDMA_TX_RING15_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA0_WPDMA_TX_RING16_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	/* MD Tx ring */
	for (u4Addr = WF_WFDMA_HOST_DMA0_WPDMA_TX_RING8_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA0_WPDMA_TX_RING10_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}
	HAL_MCR_WR(prAdapter,
		   WF_WFDMA_HOST_DMA0_WPDMA_TX_RING14_EXT_CTRL_ADDR,
		   u4WrVal);
	u4WrVal += 0x00400000;

	soc7_0SetMDRXRingPriorityInterrupt(prAdapter);

	/* reset dma TRX idx */
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA0_WPDMA_RST_DTX_PTR_ADDR, 0xFFFFFFFF);
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA0_WPDMA_RST_DRX_PTR_ADDR, 0xFFFFFFFF);
}

static void soc7_0ReadIntStatus(struct ADAPTER *prAdapter,
		uint32_t *pu4IntStatus)
{
	uint32_t u4RegValue = 0, u4WrValue = 0;
	struct GL_HIF_INFO *prHifInfo = &prAdapter->prGlueInfo->rHifInfo;
	struct BUS_INFO *prBusInfo = prAdapter->chip_info->bus_info;

	*pu4IntStatus = 0;

	HAL_MCR_RD(prAdapter,
		WF_WFDMA_HOST_DMA0_HOST_INT_STA_ADDR, &u4RegValue);

	if (HAL_IS_CONNAC2X_EXT_RX_DONE_INTR(
		    u4RegValue, prBusInfo->host_int_rxdone_bits)) {
		*pu4IntStatus |= WHISR_RX0_DONE_INT;
		u4WrValue |= (u4RegValue & prBusInfo->host_int_rxdone_bits);
	}

	if (HAL_IS_CONNAC2X_EXT_TX_DONE_INTR(
		    u4RegValue, prBusInfo->host_int_txdone_bits)) {
		*pu4IntStatus |= WHISR_TX_DONE_INT;
		u4WrValue |= (u4RegValue & prBusInfo->host_int_txdone_bits);
	}

	if (u4RegValue & CONNAC_MCU_SW_INT) {
		*pu4IntStatus |= WHISR_D2H_SW_INT;
		u4WrValue |= (u4RegValue & CONNAC_MCU_SW_INT);
	}

	prHifInfo->u4IntStatus = u4RegValue;

	/* clear interrupt */
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA0_HOST_INT_STA_ADDR, u4WrValue);
}

static void configIntMask(struct GLUE_INFO *prGlueInfo,
		u_int8_t enable)
{
	union WPDMA_INT_MASK IntMask;
	uint32_t u4Addr = 0, u4Val = 0;

	u4Addr = enable ? WF_WFDMA_HOST_DMA0_HOST_INT_ENA_SET_ADDR :
		WF_WFDMA_HOST_DMA0_HOST_INT_ENA_CLR_ADDR;

	IntMask.word = 0;
	IntMask.field_conn2x_single.wfdma0_rx_done_0 = 1;
	IntMask.field_conn2x_single.wfdma0_rx_done_1 = 1;
	IntMask.field_conn2x_single.wfdma0_rx_done_2 = 1;
	IntMask.field_conn2x_single.wfdma0_rx_done_3 = 1;
	IntMask.field_conn2x_single.wfdma0_tx_done_0 = 1;
	IntMask.field_conn2x_single.wfdma0_tx_done_1 = 1;
	IntMask.field_conn2x_single.wfdma0_tx_done_2 = 1;
	IntMask.field_conn2x_single.wfdma0_tx_done_17 = 1;
	IntMask.field_conn2x_single.wfdma0_tx_done_16 = 1;
	IntMask.field_conn2x_single.wfdma0_mcu2host_sw_int_en = 1;

	HAL_MCR_WR(prGlueInfo->prAdapter, u4Addr, IntMask.word);

	HAL_MCR_RD(prGlueInfo->prAdapter,
		   WF_WFDMA_HOST_DMA0_HOST_INT_ENA_ADDR, &u4Val);

	DBGLOG(HAL, TRACE,
	       "HOST_INT_STA(0x%08x):0x%08x, En:%u, Word:0x%08x\n",
	       WF_WFDMA_HOST_DMA0_HOST_INT_ENA_ADDR,
	       u4Val,
	       enable,
	       IntMask.word);
}

static void soc7_0asicConnac2xWpdmaConfig(struct GLUE_INFO *prGlueInfo,
		u_int8_t enable, bool fgResetHif)
{
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	union WPDMA_GLO_CFG_STRUCT GloCfg;
	uint32_t u4DmaCfgCr;
	struct BUS_INFO *prBusInfo =
			prGlueInfo->prAdapter->chip_info->bus_info;

	asicConnac2xWfdmaControl(prGlueInfo, 0, enable);
	u4DmaCfgCr = asicConnac2xWfdmaCfgAddrGet(prGlueInfo, 0);
	HAL_MCR_RD(prAdapter, u4DmaCfgCr, &GloCfg.word);

	configIntMask(prGlueInfo, enable);

	if (enable) {
		u4DmaCfgCr = asicConnac2xWfdmaCfgAddrGet(prGlueInfo, 0);
		GloCfg.field_conn2x.tx_dma_en = 1;
		GloCfg.field_conn2x.rx_dma_en = 1;
		GloCfg.field_conn2x.csr_wfdma_dummy_reg = 1;
		GloCfg.field_conn.pdma_addr_ext_en =
			(prBusInfo->u4DmaMask > 32) ? 1 : 0;
		HAL_MCR_WR(prAdapter, u4DmaCfgCr, GloCfg.word);
	}
}

int soc7_0_Trigger_fw_assert(void)
{
	int ret = 0;
	int value = 0;
	uint32_t waitRet = 0;
	struct ADAPTER *prAdapter = NULL;
	struct GLUE_INFO *prGlueInfo = NULL;

	WIPHY_PRIV(wlanGetWiphy(), prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;

	if (g_IsWfsysBusHang == TRUE) {
		DBGLOG(HAL, INFO,
			"Already trigger conninfra whole chip reset.\n");
		return 0;
	}
	DBGLOG(HAL, INFO, "Trigger fw assert start.\n");
	wf_ioremap_read(WF_TRIGGER_AP2CONN_EINT, &value);
	value &= 0xFFFFFF7F;
	ret = wf_ioremap_write(WF_TRIGGER_AP2CONN_EINT, value);
	waitRet = wait_for_completion_timeout(&g_triggerComp,
			MSEC_TO_JIFFIES(WIFI_TRIGGER_ASSERT_TIMEOUT));
	if (waitRet > 0) {
		/* Case 1: No timeout. */
		DBGLOG(INIT, INFO, "Trigger assert successfully.\n");
	} else {
		/* Case 2: timeout */
		DBGLOG(INIT, ERROR,
			"Trigger assert more than 2 seconds, need to trigger rst self\n");
	}
#if (CFG_SUPPORT_CONNINFRA == 1)
	kalSetRstEvent();
#endif
	wf_ioremap_read(WF_TRIGGER_AP2CONN_EINT, &value);
	value |= 0x80;
	ret = wf_ioremap_write(WF_TRIGGER_AP2CONN_EINT, value);

	return ret;
}

static void soc7_0EnableFwDlMode(struct ADAPTER *prAdapter)
{
	uint32_t val = 0;

	HAL_MCR_RD(prAdapter, WF_WFDMA_HOST_DMA0_PDA_CONFG_ADDR, &val);
	val |= BIT(31);
	HAL_MCR_WR(prAdapter, WF_WFDMA_HOST_DMA0_PDA_CONFG_ADDR, val);
}

static int wake_up_conninfra_off(void)
{
	uint32_t value = 0;
	uint32_t polling_count;

	/* Wakeup conn_infra off
	 * Address: 0x1806_01A4[0]
	 * Data: 1'b1
	 * Action: write
	 */
	wf_ioremap_read(CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_ADDR, &value);
	value |= CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_CONN_INFRA_WAKEPU_WF_MASK;
	wf_ioremap_write(CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_ADDR, value);

	/* wait 200 us to avoid fake ready */
	udelay(200);

	/* Check CONNSYS version ID
	 * (polling "10 times" for specific project code
	 * and each polling interval is "1ms")
	 * Address: 0x1801_1000[31:0]
	 * Data: 0x02050100
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_CFG_IP_VERSION_ADDR, &value);
	polling_count = 0;
	while (value != SOC7_CONNSYS_VERSION_ID) {
		if (polling_count > 10) {
			DBGLOG(INIT, ERROR, "Polling CONNSYS version ID fail.\n");
			return -1;
		}
		udelay(1000);
		wf_ioremap_read(CONN_INFRA_CFG_IP_VERSION_ADDR, &value);
		polling_count++;
	}

	/* Check CONN_INFRA cmdbt restore done
	 * (polling "10 times" for specific project code
	 * and each polling interval is "0.5ms")
	 * Address: 0x1800_1210[16]
	 * Data: 1'b1
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_CFG_PWRCTRL1_CONN_INFRA_RDY_ADDR, &value);
	polling_count = 0;
	while ((value & CONN_INFRA_CFG_ON_CONN_INFRA_CFG_PWRCTRL1_CONN_INFRA_RDY_MASK) == 0) {
		if (polling_count > 10) {
			DBGLOG(INIT, ERROR, "Polling CONN_INFRA cmdbt restore done fail.\n");
			return -1;
		}
		udelay(500);
		wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_CFG_PWRCTRL1_CONN_INFRA_RDY_ADDR, &value);
		polling_count++;
	}

	return 0;
}

static void set_wf_monflg_on_mailbox_wf(void)
{
	uint32_t u4Val = 0;

	/* Set wf_monflg_on for polling wf mailbox from host side CR
	 * Address: 0x1806_0B00[0] 0x1806_0B04[4:0]
	 * Data: 1'b1 5'b01100
	 * Action: write
	 */
	wf_ioremap_read(CONN_HOST_CSR_TOP_WF_ON_MONFLG_EN_FR_HIF_ADDR, &u4Val);
	u4Val |= CONN_HOST_CSR_TOP_WF_ON_MONFLG_EN_FR_HIF_WF_ON_MONFLG_EN_FR_HIF_MASK;
	wf_ioremap_write(CONN_HOST_CSR_TOP_WF_ON_MONFLG_EN_FR_HIF_ADDR, u4Val);

	wf_ioremap_read(CONN_HOST_CSR_TOP_WF_ON_MONFLG_SEL_FR_HIF_ADDR, &u4Val);
	u4Val &= ~CONN_HOST_CSR_TOP_WF_ON_MONFLG_SEL_FR_HIF_WF_ON_MONFLG_SEL_FR_HIF_MASK;
	u4Val |= 0xc;
	wf_ioremap_write(CONN_HOST_CSR_TOP_WF_ON_MONFLG_SEL_FR_HIF_ADDR, u4Val);
}

static int wf_pwr_on_consys_mcu(void)
{
	int ret = 0;
	int check;
	uint32_t value = 0;
	uint32_t polling_count;

	DBGLOG(INIT, INFO, "wmmcu power-on start.\n");

	ret = wake_up_conninfra_off();
	if (ret)
		return ret;

	/* PTA clock on
	 * Address: 0x1801_2064、0x1801_2074
	 * Data: 0x01010101、0x00000101
	 * Action: write
	 */
	value = CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_CONN_CO_EXT_PTA_HCLK_CKEN_M0_MASK |
			CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_CONN_CO_EXT_PTA_OSC_CKEN_M0_MASK |
			CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_CONN_CO_EXT_UART_PTA_HCLK_CKEN_M0_MASK |
			CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_CONN_CO_EXT_UART_PTA_OSC_CKEN_M0_MASK;
	wf_ioremap_write(CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_ADDR, value);
	value = CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_1_SET_CONN_BSI_CNS_HCLK_CKEN_M0_MASK |
			CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_1_SET_CONN_CO_EXT_FDD_COEX_HCLKCKEN_M0_MASK;
	wf_ioremap_write(CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_1_SET_ADDR, value);

	/* Assert CONNSYS WM CPU SW reset
	 * (apply this for default value patching)
	 * Address: 0x1800_0120[0]
	 * Data: 1'b0
	 * Action: write
	 */
	wf_ioremap_read(CONN_INFRA_RGU_ON_WFSYS_CPU_SW_RST_B_ADDR, &value);
	value &= ~CONN_INFRA_RGU_ON_WFSYS_CPU_SW_RST_B_WFSYS_CPU_SW_RST_B_MASK;
	wf_ioremap_write(CONN_INFRA_RGU_ON_WFSYS_CPU_SW_RST_B_ADDR, value);

	/* Turn off "conn_infra to wfsys"/wfsys to conn_infra/wfdma2conn" bus sleep protect
	 * Address: 0x1800_1440[0]
	 * Data: 1'b0
	 * Action: write
	 */
	wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_CTRL_ADDR, &value);
	value &= ~CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_CTRL_CFG_CONN_WF_SLP_PROT_SW_EN_MASK;
	wf_ioremap_write(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_CTRL_ADDR, value);

	/* Turn on wfsys_top_on
	 * Address: 0x1800_0010[31:16] 0x1800_0010[7]
	 * Data: [31:16] = 16'h5746(key) [7] = 1'b1
	 * Action: write
	 */
	wf_ioremap_read(CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_ADDR, &value);
	value &= ~(CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_WFSYS_ON_TOP_WRITE_KEY_MASK |
		CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_WFSYS_ON_TOP_PWR_ON_MASK);
	value |= WFSYS_ON_TOP_WRITE_KEY;
	value |= CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_WFSYS_ON_TOP_PWR_ON_MASK;
	wf_ioremap_write(CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_ADDR, value);

	/* Polling wfsys_rgu_off_hreset_rst_b
	 * Address: 0x1806_0A10[30]
	 * Data: 1'b1
	 * Action: polling
	 */
	wf_ioremap_read(CONN_HOST_CSR_TOP_CONNSYS_PWR_STATES_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & BIT(30)) == 0) {
		if (polling_count > 10) {
			ret = -1;
			DBGLOG(INIT, ERROR,
				"Polling wfsys rgu off fail. (0x%x)\n",
				value);
			return ret;
		}
		udelay(500);
		wf_ioremap_read(CONN_HOST_CSR_TOP_CONNSYS_PWR_STATES_ADDR,
				&value);
		polling_count++;
	}

	/* Check "conn_infra to wfsys"/wfsys to conn_infra" bus sleep protect turn off
	 * (polling "100 times" and each polling interval is "0.5ms")
	 * If conn2wf protect turn off fail, power on fail.
	 * (DRV access wfsys CR will trigger bus hang, because bus transaction will queue at slpprot.)
	 * Address: 0x1800_1444[29] 0x1800_1444[31]
	 * Data: 1'b0 1'b0
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & (CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_WF2CONN_SLP_PROT_RDY_MASK |
			CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_CONN2WF_SLP_PROT_RDY_MASK)) != 0) {
		if (polling_count > 100) {
			DBGLOG(INIT, ERROR,
				"Polling WFSYS TO CONNINFRA SLEEP PROTECT fail. (0x%x)\n",
				value);
			ret = -1;
			return ret;
		}
		udelay(500);
		wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_ADDR, &value);
		polling_count++;
	}

	/* Check WFDMA2CONN AXI TX bus sleep protect turn off
	 * (polling "100 times")
	 * If protect turn off fail, print error message
	 * Address: 0x184C_300C[23]
	 * Data: 1'b0
	 * Action: polling
	 */
	wf_ioremap_read(WF_TOP_SLPPROT_ON_STATUS_READ_ADDR, &value);
	polling_count = 0;
	while ((value & WF_TOP_SLPPROT_ON_STATUS_READ_ro_slpprot_en_source_1_MASK) != 0) {
		if (polling_count > 100) {
			DBGLOG(INIT, ERROR,
				"Polling WFDMA TO CONNINFRA SLEEP PROTECT EN 1 fail. (0x%x)\n",
				value);
			ret = -1;
			return ret;
		}
		udelay(500);
		wf_ioremap_read(WF_TOP_SLPPROT_ON_STATUS_READ_ADDR, &value);
		polling_count++;
	}

	/* Check WFDMA2CONN AXI RX bus sleep protect turn off
	 * (polling "100 times")
	 * If protect turn off fail, print error message
	 * Address: 0x184C_300C[21]
	 * Data: 1'b0
	 * Action: polling
	 */
	wf_ioremap_read(WF_TOP_SLPPROT_ON_STATUS_READ_ADDR, &value);
	polling_count = 0;
	while ((value & WF_TOP_SLPPROT_ON_STATUS_READ_ro_slpprot_en_source_2_MASK) != 0) {
		if (polling_count > 100) {
			DBGLOG(INIT, ERROR,
				"Polling WFDMA TO CONNINFRA SLEEP PROTECT EN 2 fail. (0x%x)\n",
				value);
			ret = -1;
			return ret;
		}
		udelay(500);
		wf_ioremap_read(WF_TOP_SLPPROT_ON_STATUS_READ_ADDR, &value);
		polling_count++;
	}

	/* Check WFSYS version ID
	 * (polling "10 times" for specific project code and each polling interval is "0.5ms")
	 * Address: 0x184B_0010[31:0]
	 * Data: 32'h02040100
	 * Action: polling
	 */
	wf_ioremap_read(WF_TOP_CFG_IP_VERSION_ADDR, &value);
	polling_count = 0;
	while (value != SOC7_WFSYS_VERSION_ID) {
		if (polling_count > 10) {
			DBGLOG(INIT, ERROR, "Polling WFSYS version ID fail.\n");
			ret = -1;
			return ret;
		}
		udelay(500);
		wf_ioremap_read(WF_TOP_CFG_IP_VERSION_ADDR, &value);
		polling_count++;
	}

	/* Set wfsys bus timeout value (ahb apb timeout)
	 * Address: 0x184F_0440[7:0]
	 * Data: 8'h03
	 * Action: write
	 */
	wf_ioremap_read(WF_MCU_CONFG_LS_BUSHANGCR_ADDR, &value);
	value &= ~WF_MCU_CONFG_LS_BUSHANGCR_BUS_HANG_TIME_LIMIT_MASK;
	value |= 0x00000003;
	wf_ioremap_write(WF_MCU_CONFG_LS_BUSHANGCR_ADDR, value);

	/* Enable wfsys bus timeout (ahb apb timeout)
	 * Address: 0x184F_0440[28] 0x184F_0440[31]
	 * Data: 1'b1 1'b1
	 * Action: write
	 */
	wf_ioremap_read(WF_MCU_CONFG_LS_BUSHANGCR_ADDR, &value);
	value |= (WF_MCU_CONFG_LS_BUSHANGCR_BUS_HANG_DEBUG_EN_MASK |
		WF_MCU_CONFG_LS_BUSHANGCR_BUS_HANG_DEBUG_CG_EN_MASK);
	wf_ioremap_write(WF_MCU_CONFG_LS_BUSHANGCR_ADDR, value);

	/* Set conn2wf remapping window to wf debug_ctrl_ao CR
	 * Address: 0x1840_0120
	 * Data: 32'h810F0000
	 * Action: write
	 */
	wf_ioremap_write(WF_MCU_BUS_CR_AP2WF_REMAP_1_ADDR,
		WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_DEBUG_CTRL_AO_BASE);

	/* Enable debug clock (debug ctrl ao)
	 * Address: 0x1850_0000[3]
	 * Data: 1'b1
	 * Action: write
	 */
	wf_ioremap_read(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, &value);
	value |= WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_DEBUG_CTRL_AO_WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_CTRL0_debug_cken_MASK;
	wf_ioremap_write(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, value);

	/* Reset wfsys bus timeout value (debug ctrl ao)
	 * Address: 0x1850_0000[9]
	 * Data: 1'b0
	 * Action: write
	 */
	wf_ioremap_read(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, &value);
	value |= WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_DEBUG_CTRL_AO_WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_CTRL0_timeout_clr_MASK;
	wf_ioremap_write(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, value);

	/* Reset wfsys bus timeout value (debug ctrl ao)
	 * Address: 0x1850_0000[9]
	 * Data: 1'b0
	 * Action: write
	 */
	wf_ioremap_read(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, &value);
	value &= ~WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_DEBUG_CTRL_AO_WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_CTRL0_timeout_clr_MASK;
	wf_ioremap_write(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, value);

	/* Set wfsys bus timeout value (debug ctrl ao)
	 * Address: 0x1850_0000[31:16]
	 * Data: 16'h04F5
	 * Action: write
	 */
	wf_ioremap_read(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, &value);
	value &= ~WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_DEBUG_CTRL_AO_WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_CTRL0_timeout_thres_MASK;
	value |= 0x04F50000;
	wf_ioremap_write(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, value);

	/* Mask wfdma+umac busy signal (debug ctrl ao)
	 * Address: 0x1850_000C[8]
	 * Data: 1'b1
	 * Action: write
	 */
	wf_ioremap_read(DEBUG_CTRL_AO_WFMCU_PWA_CTRL3, &value);
	value |= 0x00000100;
	wf_ioremap_write(DEBUG_CTRL_AO_WFMCU_PWA_CTRL3, value);

	/* Enable wfsys bus timeout (debug ctrl ao)
	 * Address: 0x1850_0000[4] 0x1850_0000[3] 0x1850_0000[2]
	 * Data: 1'b1 1'b1 1'b1
	 * Action: write
	 */
	wf_ioremap_read(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, &value);
	value |= (WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_DEBUG_CTRL_AO_WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_CTRL0_debug_en_MASK |
		WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_DEBUG_CTRL_AO_WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_CTRL0_debug_cken_MASK |
		WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_DEBUG_CTRL_AO_WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_WFMCU_PWA_CTRL0_debug_en_debugtop_MASK);
	wf_ioremap_write(DEBUG_CTRL_AO_WFMCU_PWA_CTRL0, value);

	/* Setup CONNSYS firmware in EMI */
#if (CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1)
	soc7_0_wlanPowerOnInit();
#endif

	set_wf_monflg_on_mailbox_wf();

	/* De-assert WFSYS CPU SW reset
	 * Address: 0x1800_0120[0]
	 * Data: 1'b1
	 * Action: write
	 */
	wf_ioremap_read(CONN_INFRA_RGU_ON_WFSYS_CPU_SW_RST_B_ADDR, &value);
	value |= CONN_INFRA_RGU_ON_WFSYS_CPU_SW_RST_B_WFSYS_CPU_SW_RST_B_MASK;
	wf_ioremap_write(CONN_INFRA_RGU_ON_WFSYS_CPU_SW_RST_B_ADDR, value);

	/* Check CONNSYS power-on completion
	 * (polling "0x1806_0B10[31:0]" == 0x1D1E and each polling interval is "1ms")
	 * (apply this for guarantee that CONNSYS CPU goes to "cos_idle_loop")
	 * ([NOTE] this setting could be changed at different CONNSYS ROM code)
	 * Action: polling
	 */
	check = 0;
	polling_count = 0;
	while (TRUE) {
		if (polling_count > 1000) {
			check = -1;
			ret = -1;
			break;
		}
		/* pooling mailbox as backup */
		wf_ioremap_read(CONN_HOST_CSR_TOP_WF_ON_MONFLG_OUT_ADDR, &value);
		if (value == CONNSYS_ROM_DONE_CHECK)
			break;
		polling_count++;
		udelay(1000);
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Check CONNSYS power-on completion fail, 0x%08x=[0x%08x]\n",
			CONN_HOST_CSR_TOP_WF_ON_MONFLG_OUT_ADDR,
			value);

		wf_ioremap_read(WF_TOP_CFG_ON_ROMCODE_INDEX_ADDR, &value);
		DBGLOG(INIT, ERROR,
			"Check CONNSYS power-on completion fail, 0x%08x=[0x%08x]\n",
			WF_TOP_CFG_ON_ROMCODE_INDEX_ADDR,
			value);
		return ret;
	}

	/* Disable conn_infra off domain force on
	 * Address: 0x1806_01A4[0]
	 * Data: 1'b0
	 * Action: write
	 */
	wf_ioremap_read(CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_ADDR, &value);
	value &= ~CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_CONN_INFRA_WAKEPU_WF_MASK;
	wf_ioremap_write(CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_ADDR, value);

	DBGLOG(INIT, INFO, "wmmcu power-on done.\n");
	return ret;
}

static int wf_pwr_off_consys_mcu(void)
{
#define MAX_WAIT_COREDUMP_COUNT 10

	int check;
	int value = 0;
	int ret = 0;
	int polling_count;
	int retryCount = 0;

#if (CFG_ANDORID_CONNINFRA_COREDUMP_SUPPORT == 1)
	while (g_IsNeedWaitCoredump) {
		kalMsleep(100);
		retryCount++;
		if (retryCount >= MAX_WAIT_COREDUMP_COUNT) {
			DBGLOG(INIT, WARN,
				"Coredump spend long time, retryCount = %d\n",
				retryCount);
		}
	}
#endif

	DBGLOG(INIT, INFO, "wmmcu power-off start.\n");

	ret = wake_up_conninfra_off();
	if (ret)
		return ret;

	/* Turn on "conn_infra to wfsys"/wfsys to conn_infra/wfdma2conn" bus sleep protect
	 * Address: 0x1800_1440[0]
	 * Data: 1'b1
	 * Action: write
	 */
	wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_CTRL_ADDR, &value);
	value |= CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_CTRL_CFG_CONN_WF_SLP_PROT_SW_EN_MASK;
	wf_ioremap_write(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_CTRL_ADDR, value);

	/* Check WFDMA2CONN AXI RX bus sleep protect turn on
	 * (polling "100 times")
	 * Address: 0x1800_1444[25]
	 * Data: 1'b1
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_WFDMA2CONN_SLP_PROT_RDY_MASK) == 0) {
		if (polling_count > 100) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_ADDR, &value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling WFDMA TO CONNINFRA SLP PROT fail. (0x%x)\n",
			value);
	}

	/* Check "conn_infra to wfsys"/wfsys to conn_infra" bus sleep protect turn on
	 * (polling "100 times" and each polling interval is "0.5ms")
	 * If conn2wf protect turn on fail, power off fail.
	 * (DRV access wfsys CR will trigger bus hang, because bus transaction wiil queue at slpprot. )
	 * Address: 0x1800_1444[29] 0x1800_1444[31]
	 * Data: 1'b1 1'b1
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & (CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_WF2CONN_SLP_PROT_RDY_MASK |
			CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_WF2CONN_SLP_PROT_HW_EN_MASK)) == 0) {
		if (polling_count > 100) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(CONN_INFRA_CFG_ON_CONN_INFRA_WF_SLP_STATUS_ADDR, &value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling WFSYS TO CONNINFRA SLP PROT fail. (0x%x)\n",
			value);
	}

	/* Turn off wfsys_top_on
	 * Address: 0x1800_0010[31:16] 0x1800_0010[7]
	 * Data: [31:16] = 16'h5746(key) [7] = 1'b0
	 * Action: write
	 */
	wf_ioremap_read(CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_ADDR, &value);
	value &= ~(CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_WFSYS_ON_TOP_WRITE_KEY_MASK |
		CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_WFSYS_ON_TOP_PWR_ON_MASK);
	value |= WFSYS_ON_TOP_WRITE_KEY;
	wf_ioremap_write(CONN_INFRA_RGU_ON_WFSYS_ON_TOP_PWR_CTL_ADDR, value);

	/* Polling wfsys_rgu_off_hreset_rst_b
	 * Address: 0x1806_0A10[30]
	 * Data: 1'b0
	 * Action: polling
	 */
	wf_ioremap_read(CONN_HOST_CSR_TOP_CONNSYS_PWR_STATES_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & BIT(30)) != 0) {
		if (polling_count > 10) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(CONN_HOST_CSR_TOP_CONNSYS_PWR_STATES_ADDR,
				&value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling wfsys rgu off fail. (0x%x)\n",
			value);
		return ret;
	}

	/* Release WFSYS semaphore
	 * 0x1807_0200[0]=1'b1
	 * 0x1807_0204[0]=1'b1
	 * 0x1807_0208[0]=1'b1
	 * 0x1807_020C[0]=1'b1
	 * 0x1807_0210[0]=1'b1
	 * 0x1807_0214[0]=1'b1
	 * 0x1807_0218[0]=1'b1
	 * 0x1807_021C[0]=1'b1
	 * 0x1807_0220[0]=1'b1
	 * 0x1807_0224[0]=1'b1
	 * 0x1807_0228[0]=1'b1
	 * 0x1807_022C[0]=1'b1
	 * 0x1807_0230[0]=1'b1
	 * 0x1807_0234[0]=1'b1
	 * 0x1807_0238[0]=1'b1
	 * 0x1807_023C[0]=1'b1
	 * 0x1807_0240[0]=1'b1
	 * 0x1807_0244[0]=1'b1
	 * 0x1807_0248[0]=1'b1
	 * 0x1807_024C[0]=1'b1
	 * 0x1807_0250[0]=1'b1
	 * 0x1807_0254[0]=1'b1
	 * 0x1807_0258[0]=1'b1
	 * 0x1807_025C[0]=1'b1
	 * 0x1807_0260[0]=1'b1
	 * Action: write
	 */
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA00_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA00_M0_OWN_REL_CONN_SEMA00_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA00_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA01_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA01_M0_OWN_REL_CONN_SEMA01_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA01_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA02_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA02_M0_OWN_REL_CONN_SEMA02_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA02_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA03_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA03_M0_OWN_REL_CONN_SEMA03_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA03_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA04_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA04_M0_OWN_REL_CONN_SEMA04_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA04_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA05_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA05_M0_OWN_REL_CONN_SEMA05_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA05_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA06_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA06_M0_OWN_REL_CONN_SEMA06_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA06_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA07_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA07_M0_OWN_REL_CONN_SEMA07_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA07_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA08_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA08_M0_OWN_REL_CONN_SEMA08_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA08_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA09_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA09_M0_OWN_REL_CONN_SEMA09_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA09_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA10_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA10_M0_OWN_REL_CONN_SEMA10_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA10_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA11_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA11_M0_OWN_REL_CONN_SEMA11_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA11_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA12_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA12_M0_OWN_REL_CONN_SEMA12_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA12_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA13_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA13_M0_OWN_REL_CONN_SEMA13_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA13_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA14_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA14_M0_OWN_REL_CONN_SEMA14_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA14_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA15_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA15_M0_OWN_REL_CONN_SEMA15_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA15_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA16_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA16_M0_OWN_REL_CONN_SEMA16_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA16_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA17_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA17_M0_OWN_REL_CONN_SEMA17_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA17_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA18_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA18_M0_OWN_REL_CONN_SEMA18_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA18_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA19_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA19_M0_OWN_REL_CONN_SEMA19_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA19_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA20_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA20_M0_OWN_REL_CONN_SEMA20_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA20_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA21_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA21_M0_OWN_REL_CONN_SEMA21_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA21_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA22_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA22_M0_OWN_REL_CONN_SEMA22_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA22_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA23_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA23_M0_OWN_REL_CONN_SEMA23_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA23_M0_OWN_REL_ADDR, value);
	wf_ioremap_read(CONN_SEMAPHORE_CONN_SEMA24_M0_OWN_REL_ADDR, &value);
	value |= CONN_SEMAPHORE_CONN_SEMA24_M0_OWN_REL_CONN_SEMA24_M0_OWN_REL_MASK;
	wf_ioremap_write(CONN_SEMAPHORE_CONN_SEMA24_M0_OWN_REL_ADDR, value);

	/* Disable A-die top_ck_en_1
	 * Address: 0x18003124[0]
	 * Data: 1'b0
	 * Action: write
	 */
	wf_ioremap_read(CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_ADDR, &value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_ADDR, value);

	/* Disable A-die top_ck_en_1
	 * Address: 0x18003124[1]
	 * Data: 1'b1
	 * Action: polling
	 */
	wf_ioremap_read(CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_WB_SLP_TOP_CK_1_BSY_MASK) != 0) {
		if (polling_count > 100) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_ADDR, &value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling A-die top_ck_en_1 off fail. (0x%x)\n",
			value);
	}

	/* Clear wf_emi_req
	 * Address: 0x18011114[0]
	 * step 1. write 1'b1
	 * step 2. write 1'b0
	 */
	wf_ioremap_read(CONN_INFRA_CFG_EMI_CTL_WF_ADDR, &value);
	value |= 0x00000001;
	wf_ioremap_write(CONN_INFRA_CFG_EMI_CTL_WF_ADDR, value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(CONN_INFRA_CFG_EMI_CTL_WF_ADDR, value);

	/* Clear wf_infra_req
	 * Address: 0x18011114[5]
	 * step 1. write 1'b1
	 * step 2. write 1'b0
	 */
	wf_ioremap_read(CONN_INFRA_CFG_EMI_CTL_WF_ADDR, &value);
	value |= 0x00000020;
	wf_ioremap_write(CONN_INFRA_CFG_EMI_CTL_WF_ADDR, value);
	value &= 0xFFFFFFDF;
	wf_ioremap_write(CONN_INFRA_CFG_EMI_CTL_WF_ADDR, value);

	/* PTA clock off
	 * Address: 0x1801_2068、0x1801_2078
	 * Data: 0x01010101、0x00000101
	 * Action: write
	 */
	value = CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_CONN_CO_EXT_PTA_HCLK_CKEN_M0_MASK |
			CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_CONN_CO_EXT_PTA_OSC_CKEN_M0_MASK |
			CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_CONN_CO_EXT_UART_PTA_HCLK_CKEN_M0_MASK |
			CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_SET_CONN_CO_EXT_UART_PTA_OSC_CKEN_M0_MASK;
	wf_ioremap_write(CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_0_CLR_ADDR, value);
	value = CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_1_SET_CONN_BSI_CNS_HCLK_CKEN_M0_MASK |
			CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_1_SET_CONN_CO_EXT_FDD_COEX_HCLKCKEN_M0_MASK;
	wf_ioremap_write(CONN_INFRA_CLKGEN_TOP_CKGEN_COEX_1_CLR_ADDR, value);

	/* release conn_infra force on
	 * Address: 0x1806_01A4[0]
	 * Data: 1'b0
	 * Action: write
	 */
	wf_ioremap_read(CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_ADDR, &value);
	value &= ~CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_CONN_INFRA_WAKEPU_WF_MASK;
	wf_ioremap_write(CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_ADDR, value);

	return ret;
}

#if (CFG_SUPPORT_CONNINFRA == 1)
static int soc7_0_ConnacPccifon(void)
{
	int ret = 0;

	/* clear bit 16:18 */
	wf_ioremap_write(0x10001BF0, BITS(16, 18));

	return ret;
}

static int soc7_0_ConnacPccifoff(void)
{
	int ret = 0;

	/* clear bit 16:18 */
	wf_ioremap_write(0x10001BF0, BITS(16, 18));

	/*reset WiFi power on status to MD*/
	ret = wf_ioremap_write(0x1024c014, 0x0ff);

	return ret;
}
#endif

#if (CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1)
void *
soc7_0_kalFirmwareImageMapping(
			IN struct GLUE_INFO *prGlueInfo,
			OUT void **ppvMapFileBuf,
			OUT uint32_t *pu4FileLength,
			IN enum ENUM_IMG_DL_IDX_T eDlIdx)
{
	uint8_t **apucNameTable = NULL;
	uint8_t *apucName[SOC7_0_FILE_NAME_TOTAL +
					  1]; /* extra +1, for the purpose of
					       * detecting the end of the array
					       */
	uint8_t idx = 0, max_idx,
		aucNameBody[SOC7_0_FILE_NAME_TOTAL][SOC7_0_FILE_NAME_MAX],
		sub_idx = 0;
	struct mt66xx_chip_info *prChipInfo =
			prGlueInfo->prAdapter->chip_info;
	uint8_t aucFlavor[2] = {0};

	DEBUGFUNC("kalFirmwareImageMapping");

	ASSERT(prGlueInfo);
	ASSERT(ppvMapFileBuf);
	ASSERT(pu4FileLength);

	*ppvMapFileBuf = NULL;
	*pu4FileLength = 0;
	kalGetFwFlavor(&aucFlavor[0]);

	do {
		/* <0.0> Get FW name prefix table */
		switch (eDlIdx) {
		case IMG_DL_IDX_N9_FW:
			apucNameTable = soc7_0_apucFwName;
			break;

		case IMG_DL_IDX_MCU_ROM_EMI:
			break;

		case IMG_DL_IDX_WIFI_ROM_EMI:
			break;

		default:
			ASSERT(0);
			break;
		}

		/* <0.2> Construct FW name */
		memset(apucName, 0, sizeof(apucName));

		/* magic number 1: reservation for detection
		 * of the end of the array
		 */
		max_idx = (sizeof(apucName) / sizeof(uint8_t *)) - 1;

		idx = 0;
		apucName[idx] = (uint8_t *)(aucNameBody + idx);

		if (eDlIdx == IMG_DL_IDX_PATCH) {
			/* construct the file name for patch */
			/* soc7_0_patch_wmmcu_1_1_hdr.bin */
			if (prChipInfo->fw_dl_ops->constructPatchName)
				prChipInfo->fw_dl_ops->constructPatchName(
					prGlueInfo, apucName, &idx);
			else
				kalSnprintf(apucName[idx], SOC7_0_FILE_NAME_MAX,
					"soc7_0_patch_wmmcu_1_%x_hdr.bin",
					wlanGetEcoVersion(
						prGlueInfo->prAdapter));
			idx += 1;
		} else if (eDlIdx == IMG_DL_IDX_MCU_ROM_EMI) {
			/* construct the file name for MCU ROM EMI */
			/* soc7_0_ram_wmmcu_1_1_hdr.bin */
			kalSnprintf(apucName[idx], SOC7_0_FILE_NAME_MAX,
				"soc7_0_ram_wmmcu_%u%s_%x_hdr.bin",
				CFG_WIFI_IP_SET,
				aucFlavor,
				wlanGetEcoVersion(
					prGlueInfo->prAdapter));

			idx += 1;
		} else if (eDlIdx == IMG_DL_IDX_WIFI_ROM_EMI) {
			/* construct the file name for WiFi ROM EMI */
			/* soc7_0_ram_wifi_1_1_hdr.bin */
			kalSnprintf(apucName[idx], SOC7_0_FILE_NAME_MAX,
				"soc7_0_ram_wifi_%u%s_%x_hdr.bin",
				CFG_WIFI_IP_SET,
				aucFlavor,
				wlanGetEcoVersion(
					prGlueInfo->prAdapter));

			idx += 1;
		} else {
			for (sub_idx = 0; sub_idx < max_idx; sub_idx++)
				apucName[sub_idx] =
					(uint8_t *)(aucNameBody + sub_idx);

			if (prChipInfo->fw_dl_ops->constructFirmwarePrio)
				prChipInfo->fw_dl_ops->constructFirmwarePrio(
					prGlueInfo, apucNameTable, apucName,
					&idx, max_idx);
			else
				kalConstructDefaultFirmwarePrio(
					prGlueInfo, apucNameTable, apucName,
					&idx, max_idx);
		}

		/* let the last pointer point to NULL
		 * so that we can detect the end of the array in
		 * kalFirmwareOpen().
		 */
		apucName[idx] = NULL;

		apucNameTable = apucName;

		/* <1> Open firmware */
		if (kalFirmwareOpen(prGlueInfo,
				    apucNameTable) != WLAN_STATUS_SUCCESS)
			break;
		{
			uint32_t u4FwSize = 0;
			void *prFwBuffer = NULL;
			/* <2> Query firmare size */
			kalFirmwareSize(prGlueInfo, &u4FwSize);
			/* <3> Use vmalloc for allocating large memory trunk */
			prFwBuffer = vmalloc(ALIGN_4(u4FwSize));
			/* <4> Load image binary into buffer */
			if (kalFirmwareLoad(prGlueInfo, prFwBuffer, 0,
					    &u4FwSize) != WLAN_STATUS_SUCCESS) {
				vfree(prFwBuffer);
				kalFirmwareClose(prGlueInfo);
				break;
			}
			/* <5> write back info */
			*pu4FileLength = u4FwSize;
			*ppvMapFileBuf = prFwBuffer;

			return prFwBuffer;
		}
	} while (FALSE);

	return NULL;
}

uint32_t soc7_0_wlanImageSectionDownloadStage(
	IN struct ADAPTER *prAdapter, IN void *pvFwImageMapFile,
	IN uint32_t u4FwImageFileLength, IN uint8_t ucSectionNumber,
	IN enum ENUM_IMG_DL_IDX_T eDlIdx)
{
	uint32_t u4SecIdx, u4Offset = 0;
	uint32_t u4Addr, u4Len, u4DataMode = 0;
	u_int8_t fgIsEMIDownload = FALSE;
	u_int8_t fgIsNotDownload = FALSE;
	uint32_t u4Status = WLAN_STATUS_SUCCESS;
	struct mt66xx_chip_info *prChipInfo = prAdapter->chip_info;
	struct patch_dl_target target;
	struct PATCH_FORMAT_T *prPatchHeader;
	struct ROM_EMI_HEADER *prRomEmiHeader;
	struct FWDL_OPS_T *prFwDlOps;

	prFwDlOps = prChipInfo->fw_dl_ops;

	/* 3a. parse file header for decision of
	 * divided firmware download or not
	 */
	if (eDlIdx == IMG_DL_IDX_PATCH) {
		prPatchHeader = pvFwImageMapFile;
		if (prPatchHeader->u4PatchVersion == PATCH_VERSION_MAGIC_NUM) {
			wlanImageSectionGetPatchInfoV2(prAdapter,
				pvFwImageMapFile,
				u4FwImageFileLength,
				&u4DataMode,
				&target);
			DBGLOG(INIT, INFO,
				"FormatV2 num_of_regoin[%d] datamode[0x%08x]\n",
				target.num_of_region, u4DataMode);
		} else {
			wlanImageSectionGetPatchInfo(prAdapter,
				pvFwImageMapFile,
					     u4FwImageFileLength,
					     &u4Offset, &u4Addr,
					     &u4Len, &u4DataMode);
			DBGLOG(INIT, INFO,
		"FormatV1 DL Offset[%u] addr[0x%08x] len[%u] datamode[0x%08x]\n",
		       u4Offset, u4Addr, u4Len, u4DataMode);
		}

		if (prPatchHeader->u4PatchVersion == PATCH_VERSION_MAGIC_NUM)
			u4Status = wlanDownloadSectionV2(prAdapter,
				u4DataMode, eDlIdx, &target);
		else
/* For dynamic memory map::Begin */
#if (CFG_DOWNLOAD_DYN_MEMORY_MAP == 1)
			u4Status = prFwDlOps->downloadByDynMemMap(
						prAdapter, u4Addr, u4Len,
						pvFwImageMapFile
							+ u4Offset,
							eDlIdx);
#else
			u4Status = wlanDownloadSection(
							prAdapter,
							u4Addr,
							u4Len,
							u4DataMode,
							pvFwImageMapFile
								+ u4Offset,
						       eDlIdx);
#endif
/* For dynamic memory map::End */
#if (CFG_SUPPORT_CONNINFRA == 1)
		/* Set datecode to EMI */
		wlanDownloadEMISection(prAdapter,
			WMMCU_ROM_PATCH_DATE_ADDR,
			DATE_CODE_SIZE, prPatchHeader->aucBuildDate);
#endif

	} else if (eDlIdx == IMG_DL_IDX_MCU_ROM_EMI) {
		prRomEmiHeader = (struct ROM_EMI_HEADER *)pvFwImageMapFile;

		DBGLOG(INIT, INFO,
			"DL MCU ROM EMI %s\n",
			(char *)prRomEmiHeader->ucDateTime);

		u4Addr = prRomEmiHeader->u4PatchAddr;

		u4Len = u4FwImageFileLength - sizeof(struct ROM_EMI_HEADER);

		u4Offset = sizeof(struct ROM_EMI_HEADER);

		u4Status = wlanDownloadEMISection(prAdapter,
					u4Addr, u4Len,
					pvFwImageMapFile + u4Offset);
		/* Set datecode to EMI */
		wlanDownloadEMISection(prAdapter,
			WMMCU_MCU_ROM_EMI_DATE_ADDR,
			DATE_CODE_SIZE,
			prRomEmiHeader->ucDateTime);
	} else {
		for (u4SecIdx = 0; u4SecIdx < ucSectionNumber;
		     u4SecIdx++, u4Offset += u4Len) {
			prChipInfo->fw_dl_ops->getFwInfo(prAdapter, u4SecIdx,
				eDlIdx, &u4Addr,
				&u4Len, &u4DataMode, &fgIsEMIDownload,
				&fgIsNotDownload);

			DBGLOG(INIT, INFO,
			       "DL Offset[%u] addr[0x%08x] len[%u] datamode[0x%08x]\n",
			       u4Offset, u4Addr, u4Len, u4DataMode);

			if (fgIsNotDownload)
				continue;
			else if (fgIsEMIDownload)
				u4Status = wlanDownloadEMISection(prAdapter,
					u4Addr, u4Len,
					pvFwImageMapFile + u4Offset);
/* For dynamic memory map:: Begin */
#if (CFG_DOWNLOAD_DYN_MEMORY_MAP == 1)
			else if ((u4DataMode &
				DOWNLOAD_CONFIG_ENCRYPTION_MODE) == 0) {
				/* Non-encrypted F/W region,
				 * use dynamic memory mapping for download
				 */
				u4Status = prFwDlOps->downloadByDynMemMap(
					prAdapter,
					u4Addr,
					u4Len,
					pvFwImageMapFile + u4Offset,
					eDlIdx);
			}
#endif
/* For dynamic memory map:: End */
			else
				u4Status = wlanDownloadSection(prAdapter,
					u4Addr, u4Len,
					u4DataMode,
					pvFwImageMapFile + u4Offset, eDlIdx);

			/* escape from loop if any pending error occurs */
			if (u4Status == WLAN_STATUS_FAILURE)
				break;
		}
	}

	return u4Status;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Wlan power on download function. This function prepare the job
 *  during power on stage to download MCU ROM EMI
 *
 * \retval 0 Success
 * \retval negative value Failed
 */
/*----------------------------------------------------------------------------*/
uint32_t soc7_0_wlanPowerOnDownload(
	IN struct ADAPTER *prAdapter,
	IN uint8_t ucDownloadItem)
{
	uint32_t u4FwSize = 0;
	void *prFwBuffer = NULL;
	uint32_t u4Status;

	if (!prAdapter)
		return WLAN_STATUS_FAILURE;

	DBGLOG_LIMITED(INIT, INFO,
		"Power on download start(%d)\n", ucDownloadItem);

	switch (ucDownloadItem) {
	case ENUM_WLAN_POWER_ON_DOWNLOAD_EMI:
		/* Download MCU ROM EMI*/
		soc7_0_kalFirmwareImageMapping(prAdapter->prGlueInfo,
			&prFwBuffer, &u4FwSize, IMG_DL_IDX_MCU_ROM_EMI);

		if (prFwBuffer == NULL) {
			DBGLOG(INIT, WARN, "FW[%u] load error!\n",
			       IMG_DL_IDX_MCU_ROM_EMI);
			return WLAN_STATUS_FAILURE;
		}

		u4Status = soc7_0_wlanImageSectionDownloadStage(
			prAdapter, prFwBuffer, u4FwSize, 1,
			IMG_DL_IDX_MCU_ROM_EMI);

		kalFirmwareImageUnmapping(
			prAdapter->prGlueInfo, NULL, prFwBuffer);

		DBGLOG_LIMITED(INIT, INFO, "Power on download mcu ROM EMI %s\n",
			(u4Status == WLAN_STATUS_SUCCESS) ? "pass" : "failed");

		break;

	default:
		return WLAN_STATUS_NOT_SUPPORTED;
	}

	DBGLOG_LIMITED(INIT, INFO, "Power on download end[%d].\n", u4Status);

	return u4Status;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Wlan power on init function. This function do the job in the
 *  power on stage to download MCU ROM EMI
 *
 *  It is to simulate wlanProbe() with the minimum effort to complete
 *  ROM EMI + ROM patch download.
 *
 * \retval 0 Success
 * \retval negative value Failed
 */
/*----------------------------------------------------------------------------*/
int32_t soc7_0_wlanPowerOnInit(void)
{
	void *pvData;
	void *pvDriverData = (void *)&mt66xx_driver_data_soc7_0;

	int32_t i4Status = 0;
	enum ENUM_POWER_ON_INIT_FAIL_REASON {
		NET_CREATE_FAIL = 0,
		ROM_PATCH_DOWNLOAD_FAIL,
		POWER_ON_INIT_DONE,
		FAIL_REASON_NUM
	} eFailReason;
	struct wireless_dev *prWdev = NULL;
	struct GLUE_INFO *prGlueInfo = NULL;
	struct ADAPTER *prAdapter = NULL;
	struct mt66xx_chip_info *prChipInfo;

	DBGLOG(INIT, INFO, "wlanPowerOnInit::begin\n");

	eFailReason = POWER_ON_INIT_DONE;

	prChipInfo = ((struct mt66xx_hif_driver_data *)pvDriverData)
				->chip_info;
	pvData = (void *)prChipInfo->pdev;

	if (fgSimplifyResetFlow) {
		WIPHY_PRIV(wlanGetWiphy(), prGlueInfo);
		prAdapter = prGlueInfo->prAdapter;

		if (prChipInfo->pwrondownload) {
			DBGLOG_LIMITED(INIT, INFO,
				"[Wi-Fi PWR On] EMI download Start\n");

			if (prChipInfo->pwrondownload(prAdapter,
				ENUM_WLAN_POWER_ON_DOWNLOAD_EMI) !=
				WLAN_STATUS_SUCCESS)
				i4Status = -ROM_PATCH_DOWNLOAD_FAIL;

			DBGLOG_LIMITED(INIT, INFO,
				"[Wi-Fi PWR On] EMI download End\n");
		}
	} else {
		prWdev = wlanNetCreate(pvData, pvDriverData);

		if (prWdev == NULL) {
			DBGLOG(INIT, ERROR,
				"[Wi-Fi PWR On] No memory for dev and its private\n");

			i4Status = -NET_CREATE_FAIL;
		} else {
			/* Set the ioaddr to HIF Info */
			WIPHY_PRIV(prWdev->wiphy, prGlueInfo);

			prAdapter = prGlueInfo->prAdapter;

			if (prChipInfo->pwrondownload) {
				DBGLOG_LIMITED(INIT, INFO,
					"[Wi-Fi PWR On] EMI download Start\n");

				if (prChipInfo->pwrondownload(prAdapter,
				    ENUM_WLAN_POWER_ON_DOWNLOAD_EMI) !=
					WLAN_STATUS_SUCCESS)
					i4Status = -ROM_PATCH_DOWNLOAD_FAIL;

				DBGLOG_LIMITED(INIT, INFO,
					"[Wi-Fi PWR On] EMI download End\n");
			}

			wlanWakeLockUninit(prGlueInfo);
		}

		wlanNetDestroy(prWdev);
	}

	return i4Status;
}
#endif

static void soc7_0_DumpWfsyscpupcr(struct ADAPTER *prAdapter)
{
#define CPUPCR_LOG_NUM	5
#define CPUPCR_BUF_SZ	50

	uint32_t i = 0;
	uint32_t var_pc = 0;
	uint32_t var_lp = 0;
	uint64_t log_sec = 0;
	uint64_t log_nsec = 0;
	char log_buf_pc[CPUPCR_LOG_NUM][CPUPCR_BUF_SZ];
	char log_buf_lp[CPUPCR_LOG_NUM][CPUPCR_BUF_SZ];

	for (i = 0; i < CPUPCR_LOG_NUM; i++) {
		log_sec = local_clock();
		log_nsec = do_div(log_sec, 1000000000)/1000;
		HAL_MCR_RD(prAdapter,
			   CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_ADDR,
			   &var_pc);
		HAL_MCR_RD(prAdapter,
			   CONN_DBG_CTL_WF_MCU_GPR_BUS_DBGOUT_LOG_ADDR,
			   &var_lp);

		kalSnprintf(log_buf_pc[i],
			    CPUPCR_BUF_SZ,
			    "%llu.%06llu/0x%08x;",
			    log_sec,
			    log_nsec,
			    var_pc);

		kalSnprintf(log_buf_lp[i],
			    CPUPCR_BUF_SZ,
			    "%llu.%06llu/0x%08x;",
			    log_sec,
			    log_nsec,
			    var_lp);
	}

	DBGLOG(HAL, INFO, "wm pc=%s%s%s%s%s\n",
	       log_buf_pc[0],
	       log_buf_pc[1],
	       log_buf_pc[2],
	       log_buf_pc[3],
	       log_buf_pc[4]);

	DBGLOG(HAL, INFO, "wm lp=%s%s%s%s%s\n",
	       log_buf_lp[0],
	       log_buf_lp[1],
	       log_buf_lp[2],
	       log_buf_lp[3],
	       log_buf_lp[4]);
}

static void soc7_0_DumpPcLrLog(struct ADAPTER *prAdapter)
{
#define	HANG_PC_LOG_NUM			32
	uint32_t u4WrAddr, u4RdAddr, u4DbgAddr, u4Index, i;
	uint32_t u4Value = 0;
	uint32_t u4RegVal = 0;
	uint32_t log[HANG_PC_LOG_NUM];

	DBGLOG(HAL, LOUD,
		"Host_CSR - dump PC log / LR log");

	memset(log, 0, HANG_PC_LOG_NUM);

	/* PC log */
	u4DbgAddr = 0x18023604;
	u4WrAddr = 0x1802360C;
	u4RdAddr = 0x18023610;

	/* WM PC log output */
	connac2x_DbgCrRead(prAdapter, u4DbgAddr, &u4Value);
	u4RegVal = u4Value & BITS(1, 31);
	connac2x_DbgCrWrite(prAdapter, u4DbgAddr, u4RegVal);

	/* choose 33th PC log buffer to read current pc log buffer index */
	connac2x_DbgCrRead(prAdapter, u4WrAddr, &u4Value);
	u4RegVal = 0x3f | (u4Value & BITS(6, 31));
	connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4RegVal);

	/* read current pc log buffer index */
	connac2x_DbgCrRead(prAdapter, u4RdAddr, &u4Value);
	u4Index = ((u4Value & BITS(17, 21)) >> 17) + 1;

	for (i = 0; i < HANG_PC_LOG_NUM; i++) {
		u4Index++;
		if (u4Index == HANG_PC_LOG_NUM)
			u4Index = 0;

		connac2x_DbgCrRead(prAdapter, u4WrAddr, &u4Value);
		u4RegVal = u4Index | (u4Value & BITS(6, 31));
		connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4RegVal);
		connac2x_DbgCrRead(prAdapter, u4RdAddr, &log[i]);
	}

	/* restore */
	connac2x_DbgCrRead(prAdapter, u4WrAddr, &u4Value);
	u4RegVal = 0x3f | (u4Value & BITS(6, 31));
	connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4RegVal);

	connac2x_dump_format_memory32(log, HANG_PC_LOG_NUM, "PC log");

	/* GPR log */
	u4DbgAddr = 0x18023604;
	u4WrAddr = 0x18023614;
	u4RdAddr = 0x18023608;

	/* WM GPR log output */
	connac2x_DbgCrRead(prAdapter, u4DbgAddr, &u4Value);
	u4RegVal = (u4Value & BITS(3, 31)) | (u4Value & BIT(0));
	connac2x_DbgCrWrite(prAdapter, u4DbgAddr, u4RegVal);

	/* choose 33th gpr log buffer to read current gpr log buffer index */
	connac2x_DbgCrRead(prAdapter, u4WrAddr, &u4Value);
	u4RegVal = 0x3f | (u4Value & BITS(6, 31));
	connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4RegVal);

	/* read current gpr log buffer index */
	connac2x_DbgCrRead(prAdapter, u4RdAddr, &u4Value);
	u4Index = ((u4Value & BITS(17, 21)) >> 17) + 1;

	for (i = 0; i < HANG_PC_LOG_NUM; i++) {
		u4Index++;
		if (u4Index == HANG_PC_LOG_NUM)
			u4Index = 0;

		connac2x_DbgCrRead(prAdapter, u4WrAddr, &u4Value);
		u4RegVal = u4Index | (u4Value & BITS(6, 31));
		connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4RegVal);
		connac2x_DbgCrRead(prAdapter, u4RdAddr, &log[i]);
	}

	/* restore */
	connac2x_DbgCrRead(prAdapter, u4WrAddr, &u4Value);
	u4RegVal = 0x3f | (u4Value & BITS(6, 31));
	connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4RegVal);

	connac2x_dump_format_memory32(log, HANG_PC_LOG_NUM, "GPR log");
}

static void soc7_0_DumpN10CoreReg(struct ADAPTER *prAdapter)
{
#define	HANG_N10_CORE_LOG_NUM	38
	uint32_t u4WrSelAddr, u4WrIdxAddr, u4RdAddr, i;
	uint32_t u4Value = 0, u4Backup = 0;
	uint32_t log[HANG_N10_CORE_LOG_NUM];

	DBGLOG(HAL, LOUD,
		"Host_CSR - read N10 core register");

	memset(log, 0, HANG_N10_CORE_LOG_NUM);

	u4RdAddr = 0x18023608;
	u4WrIdxAddr = 0x18023614;
	u4WrSelAddr = 0x18023620;

	connac2x_DbgCrRead(prAdapter, u4WrSelAddr, &u4Backup);
	connac2x_DbgCrRead(prAdapter, u4WrIdxAddr, &u4Value);
	u4Value = 0x3f | (u4Value & BITS(6, 31));

	for (i = 0; i < HANG_N10_CORE_LOG_NUM; i++) {
		connac2x_DbgCrWrite(prAdapter, u4WrSelAddr, i);
		connac2x_DbgCrWrite(prAdapter, u4WrIdxAddr, u4Value);
		connac2x_DbgCrRead(prAdapter, u4RdAddr, &log[i]);
	}

	/* restore */
	connac2x_DbgCrWrite(prAdapter, u4WrSelAddr, u4Backup);

	connac2x_dump_format_memory32(
		log, HANG_N10_CORE_LOG_NUM, "N10 core register");
}

static void soc7_0_DumpWfsysSleepWakeupDebug(struct ADAPTER *prAdapter)
{
	uint32_t u4WrVal = 0, u4Val = 0, u4Idx, u4RdAddr, u4WrAddr;
	uint32_t au4List[] = {
		0x00000000, 0x00000001, 0x00000010, 0x00000012,
		0x00000017, 0x00000018, 0x0000001C, 0x0000001D
	};

	/* enable monflg */
	connac2x_DbgCrWrite(prAdapter, 0x18060B00, 0x00000001);

	u4WrAddr = 0x18060B04;
	u4RdAddr = 0x18060B10;
	for (u4Idx = 0; u4Idx < ARRAY_SIZE(au4List); u4Idx++) {
		u4WrVal = au4List[u4Idx];
		connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4WrVal);
		connac2x_DbgCrRead(prAdapter, u4RdAddr, &u4Val);
		DBGLOG(HAL, ERROR,
		       "\tW 0x%08x=[0x%08x], 0x%08x=[0x%08x]\n",
		       u4WrAddr, u4WrVal, u4RdAddr, u4Val);
	}
}

static void soc7_0_DumpDebugCtrlAoCr(struct ADAPTER *prAdapter)
{
	uint32_t u4Val = 0, u4Addr = 0, u4Idx;

	/* CONN2WF remapping
	 * 0x1840_0120 = 32'h810F0000
	 */
	u4Addr = WF_MCU_BUS_CR_AP2WF_REMAP_1;
	u4Val = WF_MCUSYS_INFRA_BUS_FULL_U_DEBUG_CTRL_AO_BASE;
	DBGLOG(HAL, ERROR, "WR 0x%08x=[0x%08x]\n", u4Addr, u4Val);
	wf_ioremap_write(u4Addr, u4Val);

	u4Addr = 0x18500000;
	connac2x_DbgCrRead(prAdapter, u4Addr, &u4Val);
	DBGLOG(HAL, ERROR, "0x%08x=[0x%08x]\n", u4Addr, u4Val);

	/* READ debug information from debug_ctrl_ao CR
	 * dump DEBUG_CTRL_RESULT_2~18 (0x1850_0408~0x1850_0448)
	 */
	u4Addr = 0x18500408;
	for (u4Idx = 2; u4Idx <= 18; u4Idx++) {
		connac2x_DbgCrRead(prAdapter, u4Addr, &u4Val);
		DBGLOG(HAL, ERROR, "0x%08x=[0x%08x]\n", u4Addr, u4Val);
		u4Addr += 0x04;
	}
}

static void soc7_0_DumpConnDbgCtrl(struct ADAPTER *prAdapter)
{
	uint32_t u4WrVal = 0, u4Val = 0, u4Idx, u4RdAddr, u4WrAddr;
	uint32_t u4RdValue2 = 0, u4RegVal = 0, u4WrAddr2;

	u4WrAddr = 0x18023628;
	u4RdAddr = 0x18023608;
	u4WrVal = 0x00010001;
	u4WrAddr2 = 0x18023604;

	connac2x_DbgCrRead(prAdapter, u4WrAddr2, &u4RdValue2);
	u4RegVal = BIT(2) | (u4RdValue2 & BITS(3, 31));
	connac2x_DbgCrWrite(prAdapter, u4WrAddr2, u4RegVal);

	for (u4Idx = 0; u4Idx < 15; u4Idx++) {
		connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4WrVal);
		connac2x_DbgCrRead(prAdapter, u4RdAddr, &u4Val);
		DBGLOG(HAL, ERROR,
		       "\tW 0x%08x=[0x%08x], 0x%08x=[0x%08x]\n",
		       u4WrAddr, u4WrVal, u4RdAddr, u4Val);
		u4WrVal += 0x00010000;
	}
	u4WrVal = 0x00010002;
	connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4WrVal);
	connac2x_DbgCrRead(prAdapter, u4RdAddr, &u4Val);
	DBGLOG(HAL, ERROR,
	       "\tW 0x%08x=[0x%08x], 0x%08x=[0x%08x]\n",
	       u4WrAddr, u4WrVal, u4RdAddr, u4Val);
	u4WrVal = 0x00010003;
	connac2x_DbgCrWrite(prAdapter, u4WrAddr, u4WrVal);
	connac2x_DbgCrRead(prAdapter, u4RdAddr, &u4Val);
	DBGLOG(HAL, ERROR,
	       "\tW 0x%08x=[0x%08x], 0x%08x=[0x%08x]\n",
	       u4WrAddr, u4WrVal, u4RdAddr, u4Val);

	u4RegVal = (u4RdValue2 & BITS(3, 31));
	connac2x_DbgCrWrite(prAdapter, u4WrAddr2, u4RegVal);
}

static void soc7_0_DumpAhbApbTimeout(struct ADAPTER *prAdapter)
{
	uint32_t u4Val = 0, u4Addr = 0;

	/* To check ahb apb timeout setting
	 * Read 0x184F_0440
	 */
	u4Addr = WF_MCU_CONFG_LS_BUSHANGCR_ADDR;
	connac2x_DbgCrRead(prAdapter, u4Addr, &u4Val);
	DBGLOG(HAL, ERROR, "0x%08x=[0x%08x]\n", u4Addr, u4Val);

	/* To get the timeout address
	 * Read 0x184F_0444
	 */
	u4Addr = WF_MCU_CONFG_LS_BUSHANGADDR_ADDR;
	connac2x_DbgCrRead(prAdapter, u4Addr, &u4Val);
	DBGLOG(HAL, ERROR, "0x%08x=[0x%08x]\n", u4Addr, u4Val);

	/* To get the cmd information
	 * Read 0x184F_0430
	 */
	u4Addr = WF_MCU_CONFG_LS_BUSHANGCTRLA_ADDR;
	connac2x_DbgCrRead(prAdapter, u4Addr, &u4Val);
	DBGLOG(HAL, ERROR, "0x%08x=[0x%08x]\n", u4Addr, u4Val);

	/* To get bus hang bus transaction id
	 * Read 0x184F_044C
	 */
	u4Addr = WF_MCU_CONFG_LS_BUSHANGID_ADDR;
	connac2x_DbgCrRead(prAdapter, u4Addr, &u4Val);
	DBGLOG(HAL, ERROR, "0x%08x=[0x%08x]\n", u4Addr, u4Val);

	/* To get bus hang node index
	 * Read 0x184F_0450
	 */
	u4Addr = WF_MCU_CONFG_LS_BUSHANGBUS_ADDR;
	connac2x_DbgCrRead(prAdapter, u4Addr, &u4Val);
	DBGLOG(HAL, ERROR, "0x%08x=[0x%08x]\n", u4Addr, u4Val);
}

static void soc7_0_DumpOtherCr(struct ADAPTER *prAdapter)
{
#define	HANG_OTHER_LOG_NUM		2
	uint32_t u4Val = 0;

	DBGLOG(HAL, INFO,
		"Host_CSR - mailbox and other CRs");

	connac2x_DbgCrRead(NULL, 0x18060010, &u4Val);
	DBGLOG(INIT, INFO, "0x18060010=[0x%08x]\n", u4Val);
	connac2x_DbgCrRead(NULL, 0x180600f0, &u4Val);
	DBGLOG(INIT, INFO, "0x180600f0=[0x%08x]\n", u4Val);

	set_wf_monflg_on_mailbox_wf();

	/* pooling host_mailbox_wf status */
	wf_ioremap_read(CONN_HOST_CSR_TOP_WF_ON_MONFLG_OUT_ADDR, &u4Val);
	DBGLOG(INIT, INFO, "0x%08x=[0x%08x]\n",
		CONN_HOST_CSR_TOP_WF_ON_MONFLG_OUT_ADDR,
		u4Val);

	/* Power_check */
	soc7_0_DumpWfsysSleepWakeupDebug(prAdapter);

	/* WFSYS BUS debug method */
	soc7_0_DumpDebugCtrlAoCr(prAdapter);
	soc7_0_DumpConnDbgCtrl(prAdapter);
	soc7_0_DumpAhbApbTimeout(prAdapter);

	/* MCIF_MD_STATUS_CR */
	connac2x_DbgCrRead(NULL, 0x10001BF4, &u4Val);
	DBGLOG(INIT, INFO, "MD_AOR_STATUS 0x10001BF4=[%x]\n", u4Val);

	/* Dump WFDMA CR */
	connac2x_DbgCrRead(NULL, 0x184be008, &u4Val);
	DBGLOG(INIT, INFO, "WFDMA clock 0x184be008=[%x]\n", u4Val);
	connac2x_DbgCrRead(NULL, 0x184c0800, &u4Val);
	DBGLOG(INIT, INFO, "WFDMA rst 0x184c0800=[%x]\n", u4Val);
	connac2x_DumpCrRange(prAdapter, 0x18024200, 7, "WFDMA 0x18024200");
	connac2x_DumpCrRange(prAdapter, 0x18024300, 16, "WFDMA 0x18024300");
	connac2x_DumpCrRange(prAdapter, 0x18024380, 16, "WFDMA x18024380");
	connac2x_DumpCrRange(prAdapter, 0x180243E0, 12, "WFDMA 0x180243E0");
	connac2x_DumpCrRange(prAdapter, 0x18024500, 16, "WFDMA 0x18024500");
	connac2x_DumpCrRange(prAdapter, 0x18024540, 16, "WFDMA 0x18024540");
	connac2x_DumpCrRange(prAdapter, 0x18024600, 16, "WFDMA 0x18024600");
	connac2x_DumpCrRange(prAdapter, 0x18024640, 1, "WFDMA 0x18024640");
	connac2x_DumpCrRange(prAdapter, 0x18024680, 8, "WFDMA 0x18024680");
	connac2x_DumpCrRange(prAdapter, 0x18027044, 1, "WFDMA 0x18027044");
	connac2x_DumpCrRange(prAdapter, 0x18027050, 1, "WFDMA 0x18027050");
	connac2x_DumpCrRange(prAdapter, 0x18027074, 3, "WFDMA 0x18027074");
	connac2x_DumpCrRange(prAdapter, 0x1802750C, 4, "WFDMA 0x1802750C");
	connac2x_DumpCrRange(prAdapter, 0x18027520, 5, "WFDMA 0x18027520");
}

/* need to dump AXI Master related CR 0x1802750C ~ 0x18027530*/
static void soc7_0_DumpAXIMasterDebugCr(struct ADAPTER *prAdapter)
{
#define AXI_MASTER_DUMP_CR_START 0x1802750C
#define	AXI_MASTER_DUMP_CR_NUM 10

	uint32_t u4RegVal = 0, u4Idx;
	uint32_t u4Value[AXI_MASTER_DUMP_CR_NUM] = {0};

	u4RegVal = AXI_MASTER_DUMP_CR_START;
	for (u4Idx = 0 ; u4Idx < AXI_MASTER_DUMP_CR_NUM; u4Idx++) {
		connac2x_DbgCrRead(prAdapter, u4RegVal, &u4Value[u4Idx]);
		u4RegVal += 4;
	}

	connac2x_dump_format_memory32(u4Value,
		AXI_MASTER_DUMP_CR_NUM,
		"HW AXI BUS debug CR start[0x1802750C]");

} /* soc7_0_DumpAXIMasterDebugCr */

/* Dump Flow :
 *	1) dump WFDMA / AXI Master CR
 */
static void soc7_0_DumpWFDMACr(struct ADAPTER *prAdapter)
{
	/* Dump Host side WFDMACR */
	connac2x_show_wfdma_info_by_type(prAdapter, WFDMA_TYPE_HOST, 1);
	connac2x_show_wfdma_dbg_flag_log(prAdapter, WFDMA_TYPE_HOST, 1);
	soc7_0_DumpAXIMasterDebugCr(prAdapter);
	connac2x_show_wfdma_desc(prAdapter);
} /* soc7_0_DumpWFDMAHostCr */

static void soc7_0_DumpHostCr(struct ADAPTER *prAdapter)
{
	soc7_0_DumpWfsyscpupcr(prAdapter);
	soc7_0_DumpPcLrLog(prAdapter);
	soc7_0_DumpN10CoreReg(prAdapter);
	soc7_0_DumpOtherCr(prAdapter);
	soc7_0_DumpWFDMACr(prAdapter);
}

static void soc7_0_DumpBusHangCr(struct ADAPTER *prAdapter)
{
	conninfra_is_bus_hang();
	soc7_0_DumpHostCr(prAdapter);
}

static int soc7_0_CheckBusHang(void *adapter, uint8_t ucWfResetEnable)
{
	struct ADAPTER *prAdapter = (struct ADAPTER *) adapter;
	int ret = 1;
	int conninfra_read_ret = 0;
	int conninfra_hang_ret = 0;
	uint8_t conninfra_reset = FALSE;
	uint32_t u4Value = 0;
	uint32_t u4WfdmaRstVal = 0;
	uint32_t u4WfdmaClockVal = 0;

	if (prAdapter == NULL)
		DBGLOG(HAL, INFO, "prAdapter NULL\n");
	do {
/*
 * 1. Check "AP2CONN_INFRA ON step is ok"
 *   & Check "AP2CONN_INFRA OFF step is ok"
 */
		conninfra_read_ret = conninfra_reg_readable();
		if (!conninfra_read_ret) {
			DBGLOG(HAL, ERROR,
				"conninfra_reg_readable fail(%d)\n",
				conninfra_read_ret);
			conninfra_hang_ret = conninfra_is_bus_hang();
			if (conninfra_hang_ret > 0) {
				conninfra_reset = TRUE;

				DBGLOG(HAL, ERROR,
					"conninfra_is_bus_hang, Chip reset\n");
			} else {
				/*
				* not readable, but no_hang or rst_ongoing
				* => no reset and return fail
				*/
				ucWfResetEnable = FALSE;
			}

			break;
		}
/*
 * 2. Check conn2wf sleep protect
` *  - 0x1800_1444[29] (sleep protect enable ready), should be 1'b0
 */
		connac2x_DbgCrRead(prAdapter, 0x18001444, &u4Value);
		if (u4Value & BIT(29)) {
			DBGLOG(HAL, ERROR, "0x18001444[29]=1'b1\n");
			break;
		}
/*
 * 3. Read WF IP version
` *  - Read 0x184B_0010 = 02040100
 */
		wf_ioremap_read(0x184B0010, &u4Value);
		if (u4Value != 0x02040100) {
			DBGLOG(HAL, ERROR, "0x184B_0100 != 02040100\n");
			break;
		}
/*
 * 4. Check wf_mcusys bus hang irq status
` *  - 0x1802_362C[0] =1'b0
 */
		connac2x_DbgCrRead(prAdapter, 0x1802362C, &u4Value);
		if (u4Value & BIT(0)) {
			set_wf_monflg_on_mailbox_wf();

			kalMsleep(20);

			/* Pooling mailbox to check bus timeout status
			 * Address: 0x1806_0B10[4:0]
			 * [4] n9 read channel (*0:idle, 1:busy )
			 * [3] n9 write channel (*0:idle, 1:busy )
			 * [2] s2 (*1:idle, 0:busy)
			 * [1] s0 (*1:idle, 0:busy)
			 * [0] is all safe (*1:0x0A11_5AFE, 0:latched address)
			 * Data: 5'b00111
			 * Action: read
			 */
			wf_ioremap_read(CONN_HOST_CSR_TOP_WF_ON_MONFLG_OUT_ADDR,
				&u4Value);

			if (u4Value & 0x0000001f != 0x00000007) {
				DBGLOG(HAL, ERROR, "0x1802_362C[0]=1'b1\n");
				DBGLOG(HAL, ERROR, "0x1806_0B10=0x%x\n",
					u4Value);
				break;
			}
		}

		DBGLOG(HAL, TRACE, "Bus hang check: Done\n");

		ret = 0;
	} while (FALSE);

	if (ret > 0) {
		if (conninfra_reg_readable_for_coredump() == 1 ||
			((conninfra_hang_ret != CONNINFRA_ERR_RST_ONGOING) &&
			(conninfra_hang_ret != CONNINFRA_INFRA_BUS_HANG) &&
			(conninfra_hang_ret !=
				CONNINFRA_AP2CONN_RX_SLP_PROT_ERR) &&
			(conninfra_hang_ret !=
				CONNINFRA_AP2CONN_TX_SLP_PROT_ERR) &&
			(conninfra_hang_ret != CONNINFRA_AP2CONN_CLK_ERR)))
			soc7_0_DumpHostCr(prAdapter);

		if (conninfra_reset) {
			g_IsWfsysBusHang = TRUE;
			glResetWholeChipResetTrigger("bus hang");
		} else if (ucWfResetEnable) {
			g_IsWfsysBusHang = TRUE;
			glResetWholeChipResetTrigger("wifi bus hang");
		}
	} else {
		connac2x_DbgCrRead(NULL, 0x184be008, &u4WfdmaClockVal);
		connac2x_DbgCrRead(NULL, 0x184c0800, &u4WfdmaRstVal);
		if (u4WfdmaClockVal == 0xdead0003 ||
			u4WfdmaRstVal == 0xdead0003) {
			soc7_0_DumpHostCr(prAdapter);
		} else if (u4WfdmaClockVal == 0xdead0001 ||
			u4WfdmaRstVal == 0xdead0001) {
			DBGLOG(INIT, ERROR,
				"clk 0x184be008=[%x] rst 0x184c0800=[%x]\n",
				u4WfdmaClockVal, u4WfdmaRstVal);
		} else if ((u4WfdmaClockVal & BIT(26)) &&
			!(u4WfdmaClockVal & BIT(9))) {
			DBGLOG(INIT, ERROR,
				"clk 0x184be008=[%x] rst 0x184c0800=[%x]\n",
				u4WfdmaClockVal, u4WfdmaRstVal);
		} else if (!(u4WfdmaRstVal & BIT(2)) ||
			!(u4WfdmaRstVal & BIT(3))) {
			DBGLOG(INIT, ERROR,
				"clk 0x184be008=[%x] rst 0x184c0800=[%x]\n",
				u4WfdmaClockVal, u4WfdmaRstVal);
		}
	}

	return ret;
}

static bool soc7_0_get_sw_interrupt_status(struct ADAPTER *prAdapter,
	uint32_t *status)
{
	int check = 0;
	uint32_t value = 0;
	uint32_t sw_int_value = 0;

	check = wake_up_conninfra_off();
	if (check)
		return false;

	wf_ioremap_read(AP2WF_CONN_INFRA_ON_CCIF4_AP2WF_PCCIF_RCHNUM_ADDR,
		&sw_int_value);
	wf_ioremap_write(AP2WF_CONN_INFRA_ON_CCIF4_AP2WF_PCCIF_ACK_ADDR,
		sw_int_value);

	/* Disable conn_infra off domain force on 0x180601A4[0] = 1'b0 */
	wf_ioremap_read(CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_ADDR, &value);
	value &= ~CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_CONN_INFRA_WAKEPU_WF_MASK;
	wf_ioremap_write(CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_WF_ADDR, value);

	*status = sw_int_value;
	return true;
}

#endif  /* soc7_0 */
