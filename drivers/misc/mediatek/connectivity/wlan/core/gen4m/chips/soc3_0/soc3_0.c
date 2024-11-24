/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2016 MediaTek Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
/*! \file   soc3_0.c
*    \brief  Internal driver stack will export
*    the required procedures here for GLUE Layer.
*
*    This file contains all routines which are exported
     from MediaTek 802.11 Wireless LAN driver stack to GLUE Layer.
*/

#ifdef SOC3_0

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/
#define CFG_SUPPORT_VCODE_VDFS	1

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

#include "coda/soc3_0/wf_wfdma_host_dma0.h"
#include "coda/soc3_0/wf_wfdma_host_dma1.h"
#include "coda/soc3_0/wf_wfdma_mcu_dma0.h"
#include "coda/soc3_0/wf_wfdma_mcu_dma1.h"
#include "coda/soc3_0/wf_pse_top.h"

#include "coda/soc3_0/conn_infra_cfg.h"

#include "precomp.h"
#include "gl_rst.h"
#include "gl_kal.h"
#include "soc3_0.h"
#include "hal_dmashdl_soc3_0.h"
#include <linux/platform_device.h>
#include <connectivity_build_in_adapter.h>
#include <linux/mfd/mt6359p/registers.h>
#include <linux/regmap.h>
#if (CFG_SUPPORT_VCODE_VDFS == 1)
#include <linux/pm_qos.h>
#endif /*#ifndef CFG_BUILD_X86_PLATFORM*/

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

#define CONN_MCU_CONFG_BASE                0x88000000
#define CONN_MCU_CONFG_COM_REG0_ADDR       (CONN_MCU_CONFG_BASE + 0x200)

#define PATCH_SEMAPHORE_COMM_REG 0
#define PATCH_SEMAPHORE_COMM_REG_PATCH_DONE 1	/* bit0 is for patch. */

#define SW_WORKAROUND_FOR_WFDMA_ISSUE_HWITS00009838 1

#define SOC3_0_FILE_NAME_TOTAL 8
#define SOC3_0_FILE_NAME_MAX 64
/* this is workaround for AXE, do not sync back to trunk*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
static void soc3_0_triggerInt(struct GLUE_INFO *prGlueInfo);
static void soc3_0_getIntSta(struct GLUE_INFO *prGlueInfo,  uint32_t *pu4Sta);
static bool soc3_0_get_sw_interrupt_status(struct ADAPTER *prAdapter,
	uint32_t *status);
static int wf_pwr_on_consys_mcu(void);
static int wf_pwr_off_consys_mcu(void);

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/
#if (CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1)
static uint8_t *soc3_0_apucFwName[] = {
	(uint8_t *) CFG_FW_FILENAME "_MT",
	NULL
};

static uint8_t *soc3_0_apucCr4FwName[] = {
	(uint8_t *) CFG_CR4_FW_FILENAME "_MT",
	NULL
};
#endif

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/
#if (CFG_SUPPORT_VCODE_VDFS == 1)
#if (KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE)
/* Implementation for kernel-5.4 */
#else
static struct pm_qos_request wifi_req;
#endif
#endif

bool gCoAntVFE28En = FALSE;

uint8_t *apucsoc3_0FwName[] = {
	(uint8_t *) CFG_FW_FILENAME "_soc3_0",
	NULL
};

struct ECO_INFO soc3_0_eco_table[] = {
	/* HW version,  ROM version,    Factory version */
	{0x00, 0x00, 0xA, 0x1}, /* E1 */
	{0x00, 0x00, 0x0, 0x0}	/* End of table */
};

#if defined(_HIF_PCIE)
struct PCIE_CHIP_CR_MAPPING soc3_0_bus2chip_cr_mapping[] = {
	/* chip addr, bus addr, range */
	{0x54000000, 0x02000, 0x1000}, /* WFDMA PCIE0 MCU DMA0 */
	{0x55000000, 0x03000, 0x1000}, /* WFDMA PCIE0 MCU DMA1 */
	{0x56000000, 0x04000, 0x1000}, /* WFDMA reserved */
	{0x57000000, 0x05000, 0x1000}, /* WFDMA MCU wrap CR */
	{0x58000000, 0x06000, 0x1000}, /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{0x59000000, 0x07000, 0x1000}, /* WFDMA PCIE1 MCU DMA1 */
	{0x820c0000, 0x08000, 0x4000}, /* WF_UMAC_TOP (PLE) */
	{0x820c8000, 0x0c000, 0x2000}, /* WF_UMAC_TOP (PSE) */
	{0x820cc000, 0x0e000, 0x2000}, /* WF_UMAC_TOP (PP) */
	{0x820e0000, 0x20000, 0x0400}, /* WF_LMAC_TOP BN0 (WF_CFG) */
	{0x820e1000, 0x20400, 0x0200}, /* WF_LMAC_TOP BN0 (WF_TRB) */
	{0x820e2000, 0x20800, 0x0400}, /* WF_LMAC_TOP BN0 (WF_AGG) */
	{0x820e3000, 0x20c00, 0x0400}, /* WF_LMAC_TOP BN0 (WF_ARB) */
	{0x820e4000, 0x21000, 0x0400}, /* WF_LMAC_TOP BN0 (WF_TMAC) */
	{0x820e5000, 0x21400, 0x0800}, /* WF_LMAC_TOP BN0 (WF_RMAC) */
	{0x820ce000, 0x21c00, 0x0200}, /* WF_LMAC_TOP (WF_SEC) */
	{0x820e7000, 0x21e00, 0x0200}, /* WF_LMAC_TOP BN0 (WF_DMA) */
	{0x820cf000, 0x22000, 0x1000}, /* WF_LMAC_TOP (WF_PF) */
	{0x820e9000, 0x23400, 0x0200}, /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
	{0x820ea000, 0x24000, 0x0200}, /* WF_LMAC_TOP BN0 (WF_ETBF) */
	{0x820eb000, 0x24200, 0x0400}, /* WF_LMAC_TOP BN0 (WF_LPON) */
	{0x820ec000, 0x24600, 0x0200}, /* WF_LMAC_TOP BN0 (WF_INT) */
	{0x820ed000, 0x24800, 0x0800}, /* WF_LMAC_TOP BN0 (WF_MIB) */
	{0x820ca000, 0x26000, 0x2000}, /* WF_LMAC_TOP BN0 (WF_MUCOP) */
	{0x820d0000, 0x30000, 0x10000}, /* WF_LMAC_TOP (WF_WTBLON) */
	{0x40000000, 0x70000, 0x10000}, /* WF_UMAC_SYSRAM */
	{0x00400000, 0x80000, 0x10000}, /* WF_MCU_SYSRAM */
	{0x00410000, 0x90000, 0x10000}, /* WF_MCU_SYSRAM (configure register) */
	{0x820f0000, 0xa0000, 0x0400}, /* WF_LMAC_TOP BN1 (WF_CFG) */
	{0x820f1000, 0xa0600, 0x0200}, /* WF_LMAC_TOP BN1 (WF_TRB) */
	{0x820f2000, 0xa0800, 0x0400}, /* WF_LMAC_TOP BN1 (WF_AGG) */
	{0x820f3000, 0xa0c00, 0x0400}, /* WF_LMAC_TOP BN1 (WF_ARB) */
	{0x820f4000, 0xa1000, 0x0400}, /* WF_LMAC_TOP BN1 (WF_TMAC) */
	{0x820f5000, 0xa1400, 0x0800}, /* WF_LMAC_TOP BN1 (WF_RMAC) */
	{0x820f7000, 0xa1e00, 0x0200}, /* WF_LMAC_TOP BN1 (WF_DMA) */
	{0x820f9000, 0xa3400, 0x0200}, /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
	{0x820fa000, 0xa4000, 0x0200}, /* WF_LMAC_TOP BN1 (WF_ETBF) */
	{0x820fb000, 0xa4200, 0x0400}, /* WF_LMAC_TOP BN1 (WF_LPON) */
	{0x820fc000, 0xa4600, 0x0200}, /* WF_LMAC_TOP BN1 (WF_INT) */
	{0x820fd000, 0xa4800, 0x0800}, /* WF_LMAC_TOP BN1 (WF_MIB) */
	{0x820c4000, 0xa8000, 0x4000}, /* WF_LMAC_TOP (WF_UWTBL)  */
	{0x820b0000, 0xae000, 0x1000}, /* [APB2] WFSYS_ON */
	{0x80020000, 0xb0000, 0x10000}, /* WF_TOP_MISC_OFF */
	{0x81020000, 0xc0000, 0x10000}, /* WF_TOP_MISC_ON */
	{0x7c500000, 0x50000, 0x10000}, /* CONN_INFRA, dyn mem map */
	{0x7c020000, 0xd0000, 0x10000}, /* CONN_INFRA, wfdma */
	{0x7c060000, 0xe0000, 0x10000}, /* CONN_INFRA, conn_host_csr_top */
	{0x7c000000, 0xf0000, 0x10000}, /* CONN_INFRA */
	{0x0, 0x0, 0x0} /* End */
};
#elif defined(_HIF_AXI)
struct PCIE_CHIP_CR_MAPPING soc3_0_bus2chip_cr_mapping[] = {
	/* chip addr, bus addr, range */
	{0x54000000, 0x402000, 0x1000}, /* WFDMA PCIE0 MCU DMA0 */
	{0x55000000, 0x403000, 0x1000}, /* WFDMA PCIE0 MCU DMA1 */
	{0x56000000, 0x404000, 0x1000}, /* WFDMA reserved */
	{0x57000000, 0x405000, 0x1000}, /* WFDMA MCU wrap CR */
	{0x58000000, 0x406000, 0x1000}, /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{0x59000000, 0x407000, 0x1000}, /* WFDMA PCIE1 MCU DMA1 */
	{0x820c0000, 0x408000, 0x4000}, /* WF_UMAC_TOP (PLE) */
	{0x820c8000, 0x40c000, 0x2000}, /* WF_UMAC_TOP (PSE) */
	{0x820cc000, 0x40e000, 0x2000}, /* WF_UMAC_TOP (PP) */
	{0x820e0000, 0x420000, 0x0400}, /* WF_LMAC_TOP BN0 (WF_CFG) */
	{0x820e1000, 0x420400, 0x0200}, /* WF_LMAC_TOP BN0 (WF_TRB) */
	{0x820e2000, 0x420800, 0x0400}, /* WF_LMAC_TOP BN0 (WF_AGG) */
	{0x820e3000, 0x420c00, 0x0400}, /* WF_LMAC_TOP BN0 (WF_ARB) */
	{0x820e4000, 0x421000, 0x0400}, /* WF_LMAC_TOP BN0 (WF_TMAC) */
	{0x820e5000, 0x421400, 0x0800}, /* WF_LMAC_TOP BN0 (WF_RMAC) */
	{0x820ce000, 0x421c00, 0x0200}, /* WF_LMAC_TOP (WF_SEC) */
	{0x820e7000, 0x421e00, 0x0200}, /* WF_LMAC_TOP BN0 (WF_DMA) */
	{0x820cf000, 0x422000, 0x1000}, /* WF_LMAC_TOP (WF_PF) */
	{0x820e9000, 0x423400, 0x0200}, /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
	{0x820ea000, 0x424000, 0x0200}, /* WF_LMAC_TOP BN0 (WF_ETBF) */
	{0x820eb000, 0x424200, 0x0400}, /* WF_LMAC_TOP BN0 (WF_LPON) */
	{0x820ec000, 0x424600, 0x0200}, /* WF_LMAC_TOP BN0 (WF_INT) */
	{0x820ed000, 0x424800, 0x0800}, /* WF_LMAC_TOP BN0 (WF_MIB) */
	{0x820ca000, 0x426000, 0x2000}, /* WF_LMAC_TOP BN0 (WF_MUCOP) */
	{0x820d0000, 0x430000, 0x10000}, /* WF_LMAC_TOP (WF_WTBLON) */
	{0x40000000, 0x470000, 0x10000}, /* WF_UMAC_SYSRAM */
	{0x00400000, 0x480000, 0x10000}, /* WF_MCU_SYSRAM */
	{0x00410000, 0x490000, 0x10000}, /* WF_MCU_SYSRAM (config register) */
	{0x820f0000, 0x4a0000, 0x0400}, /* WF_LMAC_TOP BN1 (WF_CFG) */
	{0x820f1000, 0x4a0600, 0x0200}, /* WF_LMAC_TOP BN1 (WF_TRB) */
	{0x820f2000, 0x4a0800, 0x0400}, /* WF_LMAC_TOP BN1 (WF_AGG) */
	{0x820f3000, 0x4a0c00, 0x0400}, /* WF_LMAC_TOP BN1 (WF_ARB) */
	{0x820f4000, 0x4a1000, 0x0400}, /* WF_LMAC_TOP BN1 (WF_TMAC) */
	{0x820f5000, 0x4a1400, 0x0800}, /* WF_LMAC_TOP BN1 (WF_RMAC) */
	{0x820f7000, 0x4a1e00, 0x0200}, /* WF_LMAC_TOP BN1 (WF_DMA) */
	{0x820f9000, 0x4a3400, 0x0200}, /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
	{0x820fa000, 0x4a4000, 0x0200}, /* WF_LMAC_TOP BN1 (WF_ETBF) */
	{0x820fb000, 0x4a4200, 0x0400}, /* WF_LMAC_TOP BN1 (WF_LPON) */
	{0x820fc000, 0x4a4600, 0x0200}, /* WF_LMAC_TOP BN1 (WF_INT) */
	{0x820fd000, 0x4a4800, 0x0800}, /* WF_LMAC_TOP BN1 (WF_MIB) */
	{0x820c4000, 0x4a8000, 0x4000}, /* WF_LMAC_TOP (WF_UWTBL)  */
	{0x820b0000, 0x4ae000, 0x1000}, /* [APB2] WFSYS_ON */
	{0x80020000, 0x4b0000, 0x10000}, /* WF_TOP_MISC_OFF */
	{0x81020000, 0x4c0000, 0x10000}, /* WF_TOP_MISC_ON */
	{0x7c500000, 0x500000, 0x10000}, /* CONN_INFRA, dyn mem map */
	{0x7c000000, 0x00000,  0x10000}, /* CONN_INFRA, conn_infra_on */
	{0x7c020000, 0x20000,  0x10000}, /* CONN_INFRA, wfdma */
	{0x7c050000, 0x50000,  0x10000}, /* CONN_INFRA, conn infra sysram */
	{0x7c060000, 0x60000,  0x10000}, /* CONN_INFRA, conn_host_csr_top */
	{0x0, 0x0, 0x0} /* End */
};
#endif

struct wfdma_group_info soc3_0_wfmda_host_tx_group[] = {
	{"P1T0:AP DATA0", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING0_CTRL0_ADDR, true},
	{"P1T1:AP DATA1", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING1_CTRL0_ADDR, true},
	{"P1T2:AP DATA2", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING2_CTRL0_ADDR, true},
	{"P1T17:AP CMD", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING17_CTRL0_ADDR, true},
	{"P1T16:FWDL", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING16_CTRL0_ADDR, true},
	{"P1T8:MD DATA0", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING8_CTRL0_ADDR},
	{"P1T9:MD DATA1", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING9_CTRL0_ADDR},
	{"P1T10:MD DATA2", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING10_CTRL0_ADDR},
	{"P1T18:MD CMD", WF_WFDMA_HOST_DMA1_WPDMA_TX_RING18_CTRL0_ADDR},
};

struct wfdma_group_info soc3_0_wfmda_host_rx_group[] = {
	{"P0R0:AP DATA0", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_CTRL0_ADDR, true},
	{"P1R0:AP EVENT", WF_WFDMA_HOST_DMA1_WPDMA_RX_RING0_CTRL0_ADDR, true},
	{"P0R1:AP DATA1", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING1_CTRL0_ADDR, true},
	{"P0R2:AP TDONE0", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING2_CTRL0_ADDR},
	{"P0R3:AP TDONE1", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING3_CTRL0_ADDR},
	{"P0R4:MD DATA0", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING4_CTRL0_ADDR},
	{"P0R5:MD DATA1", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING5_CTRL0_ADDR},
	{"P0R6:MD TDONE0", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING6_CTRL0_ADDR},
	{"P0R7:MD TDONE1", WF_WFDMA_HOST_DMA0_WPDMA_RX_RING7_CTRL0_ADDR},
	{"P1R1:MD EVENT", WF_WFDMA_HOST_DMA1_WPDMA_RX_RING1_CTRL0_ADDR},
};

struct wfdma_group_info soc3_0_wfmda_wm_tx_group[] = {
	{"P0T0:DATA", WF_WFDMA_MCU_DMA0_WPDMA_TX_RING0_CTRL0_ADDR},
	{"P1T0:AP EVENT", WF_WFDMA_MCU_DMA1_WPDMA_TX_RING0_CTRL0_ADDR},
	{"P1T1:MD EVENT", WF_WFDMA_MCU_DMA1_WPDMA_TX_RING1_CTRL0_ADDR},
};

struct wfdma_group_info soc3_0_wfmda_wm_rx_group[] = {
	{"P0R0:DATA", WF_WFDMA_MCU_DMA0_WPDMA_RX_RING0_CTRL0_ADDR},
	{"P0R1:TXDONE", WF_WFDMA_MCU_DMA0_WPDMA_RX_RING1_CTRL0_ADDR},
	{"P1R0:FWDL", WF_WFDMA_MCU_DMA1_WPDMA_RX_RING0_CTRL0_ADDR},
	{"P1R1:AP CMD", WF_WFDMA_MCU_DMA1_WPDMA_RX_RING1_CTRL0_ADDR},
	{"P1R2:MD CMD", WF_WFDMA_MCU_DMA1_WPDMA_RX_RING2_CTRL0_ADDR},
};

struct pse_group_info rSoc3_0_pse_group[] = {
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

static uint8_t soc3_0SetRxRingHwAddr(struct RTMP_RX_RING *prRxRing,
		struct BUS_INFO *prBusInfo, uint32_t u4SwRingIdx)
{
	uint32_t offset = 0;
	bool fgIsWfdma1 = false;

	/*
	 * RX_RING_EVT_IDX_1     (RX_Ring0) - Rx Event
	 * RX_RING_DATA_IDX_0    (RX_Ring0) - Band0 Rx Data
	 * RX_RING_DATA1_IDX_2   (RX_Ring1) - Band1 Rx Data
	 * RX_RING_TXDONE0_IDX_3 (RX_Ring2) - Band0 Tx Free Done Event
	 * RX_RING_TXDONE1_IDX_4 (RX_Ring3) - Band1 Tx Free Done Event
	*/
	switch (u4SwRingIdx) {
	case RX_RING_EVT_IDX_1:
		offset = 0;
		fgIsWfdma1 = true;
		break;
	case RX_RING_DATA_IDX_0:
		offset = 0;
		break;
	case RX_RING_DATA1_IDX_2:
	case RX_RING_TXDONE0_IDX_3:
	case RX_RING_TXDONE1_IDX_4:
		offset = (u4SwRingIdx - 1) * MT_RINGREG_DIFF;
		break;
	default:
		return FALSE;
	}

	if (fgIsWfdma1) {
		prRxRing->hw_desc_base =
			prBusInfo->host_wfdma1_rx_ring_base + offset;
		prRxRing->hw_cidx_addr =
			prBusInfo->host_wfdma1_rx_ring_cidx_addr + offset;
		prRxRing->hw_didx_addr =
			prBusInfo->host_wfdma1_rx_ring_didx_addr + offset;
		prRxRing->hw_cnt_addr =
			prBusInfo->host_wfdma1_rx_ring_cnt_addr + offset;
	} else {
		prRxRing->hw_desc_base =
			prBusInfo->host_rx_ring_base + offset;
		prRxRing->hw_cidx_addr =
			prBusInfo->host_rx_ring_cidx_addr + offset;
		prRxRing->hw_didx_addr =
			prBusInfo->host_rx_ring_didx_addr + offset;
		prRxRing->hw_cnt_addr =
			prBusInfo->host_rx_ring_cnt_addr + offset;
	}

	return TRUE;
}

static bool soc3_0WfdmaAllocRxRing(
	struct GLUE_INFO *prGlueInfo,
	bool fgAllocMem)
{
	if (!halWpdmaAllocRxRing(prGlueInfo, RX_RING_DATA1_IDX_2,
			RX_RING0_SIZE, RXD_SIZE, RX_BUFFER_AGGRESIZE,
			fgAllocMem)) {
		DBGLOG(HAL, ERROR, "AllocWfdmaRxRing fail\n");
		return false;
	}
	if (!halWpdmaAllocRxRing(prGlueInfo, RX_RING_TXDONE0_IDX_3,
			RX_RING1_SIZE, RXD_SIZE, RX_BUFFER_AGGRESIZE,
			fgAllocMem)) {
		DBGLOG(HAL, ERROR, "AllocWfdmaRxRing fail\n");
		return false;
	}
	if (!halWpdmaAllocRxRing(prGlueInfo, RX_RING_TXDONE1_IDX_4,
			RX_RING1_SIZE, RXD_SIZE, RX_BUFFER_AGGRESIZE,
			fgAllocMem)) {
		DBGLOG(HAL, ERROR, "AllocWfdmaRxRing fail\n");
		return false;
	}
	return true;
}

static void soc3_0asicConnac2xInterruptSettings(
	struct ADAPTER *prAdapter,
	IN u_int8_t enable)
{
	union soc3_0_WPDMA_INT_MASK IntMask;
	ASSERT(prAdapter);

	if (enable) {
		IntMask.word = 0;

		IntMask.field_wfdma0_ena.wfdma0_rx_done_0 = 1;
		IntMask.field_wfdma0_ena.wfdma0_rx_done_1 = 1;
		IntMask.field_wfdma0_ena.wfdma0_rx_done_2 = 1;
		IntMask.field_wfdma0_ena.wfdma0_rx_done_3 = 1;
		HAL_MCR_WR(prAdapter,
			WF_WFDMA_HOST_DMA0_HOST_INT_ENA_SET_ADDR, IntMask.word);

		IntMask.word = 0;

		IntMask.field_wfdma1_ena.wfdma1_tx_done_0 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_1 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_2 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_16 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_17 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_coherent = 1;
		IntMask.field_wfdma1_ena.wfdma1_rx_done_0 = 1;
		IntMask.field_wfdma1_ena.wfdma1_mcu2host_sw_int_en = 1;
		HAL_MCR_WR(prAdapter,
			WF_WFDMA_HOST_DMA1_HOST_INT_ENA_SET_ADDR, IntMask.word);
	} else {

		IntMask.word = 0;
		IntMask.field_wfdma0_ena.wfdma0_rx_done_0 = 1;
		IntMask.field_wfdma0_ena.wfdma0_rx_done_1 = 1;
		IntMask.field_wfdma0_ena.wfdma0_rx_done_2 = 1;
		IntMask.field_wfdma0_ena.wfdma0_rx_done_3 = 1;
		HAL_MCR_WR(prAdapter,
			WF_WFDMA_HOST_DMA0_HOST_INT_ENA_CLR_ADDR, IntMask.word);

		IntMask.word = 0;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_0 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_1 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_2 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_16 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_done_17 = 1;
		IntMask.field_wfdma1_ena.wfdma1_tx_coherent = 0;
		IntMask.field_wfdma1_ena.wfdma1_rx_done_0 = 1;
		IntMask.field_wfdma1_ena.wfdma1_mcu2host_sw_int_en = 1;
		HAL_MCR_WR(prAdapter,
			WF_WFDMA_HOST_DMA1_HOST_INT_ENA_CLR_ADDR, IntMask.word);
	}

}

static void soc3_0asicConnac2xWfdmaControl(
	struct GLUE_INFO *prGlueInfo,
	u_int8_t ucDmaIdx,
	u_int8_t enable)
{
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	union WPDMA_GLO_CFG_STRUCT GloCfg;
	uint32_t u4DmaCfgCr;
	uint32_t u4DmaRstDtxPtrCr;
	uint32_t u4DmaRstDrxPtrCr;

	ASSERT(ucDmaIdx < CONNAC2X_MAX_WFDMA_COUNT);
	u4DmaCfgCr = asicConnac2xWfdmaCfgAddrGet(prGlueInfo, ucDmaIdx);
	u4DmaRstDtxPtrCr =
		asicConnac2xWfdmaIntRstDtxPtrAddrGet(prGlueInfo, ucDmaIdx);
	u4DmaRstDrxPtrCr =
		asicConnac2xWfdmaIntRstDrxPtrAddrGet(prGlueInfo, ucDmaIdx);

	HAL_MCR_RD(prAdapter, u4DmaCfgCr, &GloCfg.word);
	if (enable == TRUE) {
		GloCfg.field_conn2x.pdma_bt_size = 3;
		GloCfg.field_conn2x.tx_wb_ddone = 1;
		GloCfg.field_conn2x.fifo_little_endian = 1;
		GloCfg.field_conn2x.clk_gate_dis = 1;
		GloCfg.field_conn2x.omit_tx_info = 1;
		if (ucDmaIdx == 1)
			GloCfg.field_conn2x.omit_rx_info = 1;
		GloCfg.field_conn2x.csr_disp_base_ptr_chain_en = 1;
		GloCfg.field_conn2x.omit_rx_info_pfet2 = 1;
	} else {
		GloCfg.field_conn2x.tx_dma_en = 0;
		GloCfg.field_conn2x.rx_dma_en = 0;
		GloCfg.field_conn2x.csr_disp_base_ptr_chain_en = 0;
		GloCfg.field_conn2x.omit_tx_info = 0;
		GloCfg.field_conn2x.omit_rx_info = 0;
		GloCfg.field_conn2x.omit_rx_info_pfet2 = 0;
	}
	HAL_MCR_WR(prAdapter, u4DmaCfgCr, GloCfg.word);

	if (!enable) {
		asicConnac2xWfdmaWaitIdle(prGlueInfo, ucDmaIdx, 100, 1000);
		/* Reset DMA Index */
		HAL_MCR_WR(prAdapter, u4DmaRstDtxPtrCr, 0xFFFFFFFF);
		HAL_MCR_WR(prAdapter, u4DmaRstDrxPtrCr, 0xFFFFFFFF);
	}
}

void soc3_0asicConnac2xWpdmaConfig(
	struct GLUE_INFO *prGlueInfo,
	u_int8_t enable,
	bool fgResetHif)
{
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	union WPDMA_GLO_CFG_STRUCT GloCfg[CONNAC2X_MAX_WFDMA_COUNT] = {0};
	uint32_t u4DmaCfgCr = 0;
	uint32_t idx, u4DmaNum = 1;
	struct mt66xx_chip_info *chip_info = prAdapter->chip_info;
	struct BUS_INFO *prBusInfo =
			prGlueInfo->prAdapter->chip_info->bus_info;

	if (chip_info->is_support_wfdma1)
		u4DmaNum++;

	for (idx = 0; idx < u4DmaNum; idx++) {
		soc3_0asicConnac2xWfdmaControl(prGlueInfo, idx, enable);
		u4DmaCfgCr = asicConnac2xWfdmaCfgAddrGet(prGlueInfo, idx);
		HAL_MCR_RD(prAdapter, u4DmaCfgCr, &GloCfg[idx].word);
	}

	/* set interrupt Mask */
	soc3_0asicConnac2xInterruptSettings(prAdapter, enable);

	if (enable) {
		for (idx = 0; idx < u4DmaNum; idx++) {
			u4DmaCfgCr =
				asicConnac2xWfdmaCfgAddrGet(prGlueInfo, idx);
			GloCfg[idx].field_conn2x.tx_dma_en = 1;
			GloCfg[idx].field_conn2x.rx_dma_en = 1;
			GloCfg[idx].field_conn2x.pdma_addr_ext_en =
				(prBusInfo->u4DmaMask > 32) ? 1 : 0;
			HAL_MCR_WR(prAdapter, u4DmaCfgCr, GloCfg[idx].word);
		}
	}
}

void soc3_0ReadExtIntStatus(
	struct ADAPTER *prAdapter,
	uint32_t *pu4IntStatus)
{
	uint32_t u4RegValue = 0;
	uint32_t ap_write_value = 0;
	struct GL_HIF_INFO *prHifInfo = &prAdapter->prGlueInfo->rHifInfo;
	struct BUS_INFO *prBusInfo = prAdapter->chip_info->bus_info;

	*pu4IntStatus = 0;

	HAL_MCR_RD(prAdapter,
		CONNAC2X_WPDMA_EXT_INT_STA(
			prBusInfo->host_ext_conn_hif_wrap_base),
		&u4RegValue);

	if (HAL_IS_CONNAC2X_EXT_RX_DONE_INTR(u4RegValue,
		prBusInfo->host_int_rxdone_bits)) {
		*pu4IntStatus |= WHISR_RX0_DONE_INT;
		ap_write_value |= (u4RegValue &
			prBusInfo->host_int_rxdone_bits);
	}

	if (HAL_IS_CONNAC2X_EXT_TX_DONE_INTR(u4RegValue,
		prBusInfo->host_int_txdone_bits)) {
		ap_write_value |= (u4RegValue &
			prBusInfo->host_int_txdone_bits);
		*pu4IntStatus |= WHISR_TX_DONE_INT;
	}

	if (u4RegValue & CONNAC2X_EXT_WFDMA1_TX_COHERENT_INT) {
		*pu4IntStatus |= WHISR_ABNORMAL_INT;
		ap_write_value |=
			(u4RegValue & CONNAC2X_EXT_WFDMA1_TX_COHERENT_INT);
	}

	if (prHifInfo->ulIntFlag & HIF_FLAG_SW_WFDMA_INT)
		*pu4IntStatus |= WHISR_TX_DONE_INT;

	if (u4RegValue & CONNAC_MCU_SW_INT) {
		*pu4IntStatus |= WHISR_D2H_SW_INT;
		ap_write_value |= (u4RegValue & CONNAC_MCU_SW_INT);
	}

	prHifInfo->u4IntStatus = u4RegValue;

	/* clear interrupt */
	HAL_MCR_WR(prAdapter,
		CONNAC2X_WPDMA_EXT_INT_STA(
			prBusInfo->host_ext_conn_hif_wrap_base),
		ap_write_value);

}

void soc3_0asicConnac2xProcessTxInterrupt(IN struct ADAPTER *prAdapter)
{
	struct GL_HIF_INFO *prHifInfo = &prAdapter->prGlueInfo->rHifInfo;
	struct BUS_INFO *prBusInfo = prAdapter->chip_info->bus_info;
	struct SW_WFDMA_INFO *prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;
	union WPDMA_INT_STA_STRUCT rIntrStatus;

	if (test_and_clear_bit(HIF_FLAG_SW_WFDMA_INT_BIT,
			       &prHifInfo->ulIntFlag)) {
		if (prSwWfdmaInfo->rOps.processDmaDone)
			prSwWfdmaInfo->rOps.
				processDmaDone(prAdapter->prGlueInfo);
		else
			DBGLOG(HAL, ERROR, "SwWfdma ops unsupported!");
	}

	rIntrStatus = (union WPDMA_INT_STA_STRUCT)prHifInfo->u4IntStatus;
	if (rIntrStatus.field_conn2x_ext.wfdma1_tx_done_16)
		halWpdmaProcessCmdDmaDone(prAdapter->prGlueInfo,
			TX_RING_FWDL_IDX_4);

	if (rIntrStatus.field_conn2x_ext.wfdma1_tx_done_17)
		halWpdmaProcessCmdDmaDone(prAdapter->prGlueInfo,
			TX_RING_CMD_IDX_3);

	if (rIntrStatus.field_conn2x_ext.wfdma1_tx_done_0) {
		halWpdmaProcessDataDmaDone(prAdapter->prGlueInfo,
			TX_RING_DATA0_IDX_0);

		kalSetTxEvent2Hif(prAdapter->prGlueInfo);
	}

	if (rIntrStatus.field_conn2x_ext.wfdma1_tx_done_1) {
		halWpdmaProcessDataDmaDone(prAdapter->prGlueInfo,
			TX_RING_DATA1_IDX_1);

		kalSetTxEvent2Hif(prAdapter->prGlueInfo);
	}

	if (rIntrStatus.field_conn2x_ext.wfdma1_tx_done_2) {
		halWpdmaProcessDataDmaDone(prAdapter->prGlueInfo,
			TX_RING_DATA2_IDX_2);

		kalSetTxEvent2Hif(prAdapter->prGlueInfo);
	}

}

void soc3_0asicConnac2xProcessRxInterrupt(
	struct ADAPTER *prAdapter)
{
	struct GL_HIF_INFO *prHifInfo = &prAdapter->prGlueInfo->rHifInfo;
	union WPDMA_INT_STA_STRUCT rIntrStatus;

	rIntrStatus = (union WPDMA_INT_STA_STRUCT)prHifInfo->u4IntStatus;
	if (rIntrStatus.field_conn2x_ext.wfdma1_rx_done_0 ||
		(prAdapter->u4NoMoreRfb & BIT(RX_RING_EVT_IDX_1)))
		halRxReceiveRFBs(prAdapter, RX_RING_EVT_IDX_1, FALSE);

	if (rIntrStatus.field_conn2x_ext.wfdma0_rx_done_0 ||
		(prAdapter->u4NoMoreRfb & BIT(RX_RING_DATA_IDX_0)))
		halRxReceiveRFBs(prAdapter, RX_RING_DATA_IDX_0, TRUE);

	if (rIntrStatus.field_conn2x_ext.wfdma0_rx_done_1 ||
		(prAdapter->u4NoMoreRfb & BIT(RX_RING_DATA1_IDX_2)))
		halRxReceiveRFBs(prAdapter, RX_RING_DATA1_IDX_2, TRUE);

	if (rIntrStatus.field_conn2x_ext.wfdma0_rx_done_2 ||
		(prAdapter->u4NoMoreRfb & BIT(RX_RING_TXDONE0_IDX_3)))
		halRxReceiveRFBs(prAdapter, RX_RING_TXDONE0_IDX_3, TRUE);

	if (rIntrStatus.field_conn2x_ext.wfdma0_rx_done_3 ||
		(prAdapter->u4NoMoreRfb & BIT(RX_RING_TXDONE1_IDX_4)))
		halRxReceiveRFBs(prAdapter, RX_RING_TXDONE1_IDX_4, TRUE);
}

void soc3_0ProcessAbnormalInterrupt(struct ADAPTER *prAdapter)
{
	struct GL_HIF_INFO *prHifInfo = &prAdapter->prGlueInfo->rHifInfo;
	union WPDMA_INT_STA_STRUCT rIntrStatus;

	rIntrStatus = (union WPDMA_INT_STA_STRUCT)prHifInfo->u4IntStatus;
	if (rIntrStatus.field_conn2x_ext.wfdma1_tx_coherent) {
		DBGLOG(HAL, ERROR, "wfdma1 tx coherent, trigger SER\n");
		halProcessAbnormalInterrupt(prAdapter);
	}
}

static void soc3_0SetMDRXRingPriorityInterrupt(struct ADAPTER *prAdapter)
{
	u_int32_t val = 0;

	HAL_MCR_RD(prAdapter,
		WF_WFDMA_HOST_DMA0_WPDMA_INT_RX_PRI_SEL_ADDR, &val);
	val |= BITS(4, 7);
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA0_WPDMA_INT_RX_PRI_SEL_ADDR, val);
	HAL_MCR_RD(prAdapter,
		WF_WFDMA_HOST_DMA1_WPDMA_INT_RX_PRI_SEL_ADDR, &val);
	val |= BIT(1);
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA1_WPDMA_INT_RX_PRI_SEL_ADDR, val);
}

void soc3_0asicConnac2xWfdmaManualPrefetch(
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

	HAL_MCR_RD(prAdapter, WF_WFDMA_HOST_DMA1_WPDMA_GLO_CFG_ADDR, &val);
	/* disable prefetch offset calculation auto-mode */
	val &=
	~WF_WFDMA_HOST_DMA1_WPDMA_GLO_CFG_CSR_DISP_BASE_PTR_CHAIN_EN_MASK;
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA1_WPDMA_GLO_CFG_ADDR, val);

	/* Rx ring */
	for (u4Addr = WF_WFDMA_HOST_DMA0_WPDMA_RX_RING0_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA0_WPDMA_RX_RING7_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	for (u4Addr = WF_WFDMA_HOST_DMA1_WPDMA_RX_RING0_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA1_WPDMA_RX_RING2_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	/* Tx ring */
	for (u4Addr = WF_WFDMA_HOST_DMA1_WPDMA_TX_RING0_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA1_WPDMA_TX_RING3_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	for (u4Addr = WF_WFDMA_HOST_DMA1_WPDMA_TX_RING8_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA1_WPDMA_TX_RING11_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	for (u4Addr = WF_WFDMA_HOST_DMA1_WPDMA_TX_RING16_EXT_CTRL_ADDR;
	     u4Addr <= WF_WFDMA_HOST_DMA1_WPDMA_TX_RING19_EXT_CTRL_ADDR;
	     u4Addr += 0x4) {
		HAL_MCR_WR(prAdapter, u4Addr, u4WrVal);
		u4WrVal += 0x00400000;
	}

	soc3_0SetMDRXRingPriorityInterrupt(prAdapter);

	/* reset dma TRX idx */
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA0_WPDMA_RST_DTX_PTR_ADDR, 0xFFFFFFFF);
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA1_WPDMA_RST_DTX_PTR_ADDR, 0xFFFFFFFF);
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA0_WPDMA_RST_DRX_PTR_ADDR, 0xFFFFFFFF);
	HAL_MCR_WR(prAdapter,
		WF_WFDMA_HOST_DMA1_WPDMA_RST_DRX_PTR_ADDR, 0xFFFFFFFF);

#if defined(_HIF_AXI)
    /*Bypass BID check*/
	soc3_0_WfdmaAxiCtrl(prAdapter);
#endif

}

void soc3_0_WfdmaAxiCtrl(
	struct ADAPTER *prAdapter)
{
	uint32_t u4Val = 0;

	HAL_MCR_RD(prAdapter, WFDMA_AXI0_R2A_CTRL_0, &u4Val);
	u4Val |= BID_CHK_BYP_EN_MASK;
	HAL_MCR_WR(prAdapter, WFDMA_AXI0_R2A_CTRL_0, u4Val);

}

void soc3_0_ConstructPatchName(struct GLUE_INFO *prGlueInfo,
	uint8_t **apucName, uint8_t *pucNameIdx)
{
	int ret = 0;
	uint8_t aucFlavor[2] = {0};

	kalGetFwFlavor(&aucFlavor[0]);

	ret = kalSnprintf(apucName[(*pucNameIdx)],
			SOC3_0_FILE_NAME_MAX,
			"soc3_0_patch_wmmcu_%u%s_%x_hdr.bin",
			CFG_WIFI_IP_SET,
			aucFlavor,
			wlanGetEcoVersion(prGlueInfo->prAdapter));

	if (ret < 0 || ret >= CFG_FW_NAME_MAX_LEN)
		DBGLOG(INIT, ERROR, "kalSnprintf failed, ret: %d\n", ret);
}

struct BUS_INFO soc3_0_bus_info = {
#if defined(_HIF_PCIE) || defined(_HIF_AXI)
	.top_cfg_base = SOC3_0_TOP_CFG_BASE,

	/* host_dma0 for TXP */
	.host_dma0_base = CONNAC2X_HOST_WPDMA_0_BASE,
	/* host_dma1 for TXD and host cmd to WX_CPU */
	.host_dma1_base = CONNAC2X_HOST_WPDMA_1_BASE,
	.host_ext_conn_hif_wrap_base = CONNAC2X_HOST_EXT_CONN_HIF_WRAP,
	.host_int_status_addr =
		CONNAC2X_WPDMA_EXT_INT_STA(CONNAC2X_HOST_EXT_CONN_HIF_WRAP),

	.host_int_txdone_bits = (CONNAC2X_EXT_WFDMA1_TX_DONE_INT0
				| CONNAC2X_EXT_WFDMA1_TX_DONE_INT1
				| CONNAC2X_EXT_WFDMA1_TX_DONE_INT2
				| CONNAC2X_EXT_WFDMA1_TX_DONE_INT3
				| CONNAC2X_EXT_WFDMA1_TX_DONE_INT4
				| CONNAC2X_EXT_WFDMA1_TX_DONE_INT5
				| CONNAC2X_EXT_WFDMA1_TX_DONE_INT6
				| CONNAC2X_EXT_WFDMA1_TX_DONE_INT16
				| CONNAC2X_EXT_WFDMA1_TX_DONE_INT17),
	.host_int_rxdone_bits = (CONNAC2X_EXT_WFDMA1_RX_DONE_INT0
				| CONNAC2X_EXT_WFDMA0_RX_DONE_INT0
				| CONNAC2X_EXT_WFDMA0_RX_DONE_INT1
				| CONNAC2X_EXT_WFDMA0_RX_DONE_INT2
				| CONNAC2X_EXT_WFDMA0_RX_DONE_INT3
				),

	.host_tx_ring_base =
		CONNAC2X_TX_RING_BASE(CONNAC2X_HOST_WPDMA_1_BASE),
	.host_tx_ring_ext_ctrl_base =
		CONNAC2X_TX_RING_EXT_CTRL_BASE(CONNAC2X_HOST_WPDMA_1_BASE),
	.host_tx_ring_cidx_addr =
		CONNAC2X_TX_RING_CIDX(CONNAC2X_HOST_WPDMA_1_BASE),
	.host_tx_ring_didx_addr =
		CONNAC2X_TX_RING_DIDX(CONNAC2X_HOST_WPDMA_1_BASE),
	.host_tx_ring_cnt_addr =
		CONNAC2X_TX_RING_CNT(CONNAC2X_HOST_WPDMA_1_BASE),

	.host_rx_ring_base =
		CONNAC2X_RX_RING_BASE(CONNAC2X_HOST_WPDMA_0_BASE),
	.host_rx_ring_ext_ctrl_base =
		CONNAC2X_RX_RING_EXT_CTRL_BASE(CONNAC2X_HOST_WPDMA_0_BASE),
	.host_rx_ring_cidx_addr =
		CONNAC2X_RX_RING_CIDX(CONNAC2X_HOST_WPDMA_0_BASE),
	.host_rx_ring_didx_addr =
		CONNAC2X_RX_RING_DIDX(CONNAC2X_HOST_WPDMA_0_BASE),
	.host_rx_ring_cnt_addr =
		CONNAC2X_RX_RING_CNT(CONNAC2X_HOST_WPDMA_0_BASE),

	.host_wfdma1_rx_ring_base =
		CONNAC2X_WFDMA1_RX_RING_BASE(CONNAC2X_HOST_WPDMA_1_BASE),
	.host_wfdma1_rx_ring_cidx_addr =
		CONNAC2X_WFDMA1_RX_RING_CIDX(CONNAC2X_HOST_WPDMA_1_BASE),
	.host_wfdma1_rx_ring_didx_addr =
		CONNAC2X_WFDMA1_RX_RING_DIDX(CONNAC2X_HOST_WPDMA_1_BASE),
	.host_wfdma1_rx_ring_cnt_addr =
		CONNAC2X_WFDMA1_RX_RING_CNT(CONNAC2X_HOST_WPDMA_1_BASE),
	.host_wfdma1_rx_ring_ext_ctrl_base =
		CONNAC2X_WFDMA1_RX_RING_EXT_CTRL_BASE(
			CONNAC2X_HOST_WPDMA_1_BASE),

	.bus2chip = soc3_0_bus2chip_cr_mapping,
#if defined(_HIF_PCIE)
	.max_static_map_addr = 0x000f0000,
#elif defined(_HIF_AXI)
	.max_static_map_addr = 0x00700000,
#endif
	.tx_ring_fwdl_idx = CONNAC2X_FWDL_TX_RING_IDX,
	.tx_ring_cmd_idx = CONNAC2X_CMD_TX_RING_IDX,
	.tx_ring0_data_idx = 0,
	.tx_ring1_data_idx = 1,
	.tx_ring2_data_idx = 2,
	.fw_own_clear_addr = CONNAC2X_BN0_IRQ_STAT_ADDR,
	.fw_own_clear_bit = PCIE_LPCR_FW_CLR_OWN,
	.fgCheckDriverOwnInt = FALSE,
	.u4DmaMask = 32,
#if defined(_HIF_PCIE)
	.pcie2ap_remap_2 = CONN_INFRA_CFG_PCIE2AP_REMAP_2_ADDR,
#endif
	.ap2wf_remap_1 = CONN_INFRA_CFG_AP2WF_REMAP_1_ADDR,
	.wfmda_host_tx_group = soc3_0_wfmda_host_tx_group,
	.wfmda_host_tx_group_len = ARRAY_SIZE(soc3_0_wfmda_host_tx_group),
	.wfmda_host_rx_group = soc3_0_wfmda_host_rx_group,
	.wfmda_host_rx_group_len = ARRAY_SIZE(soc3_0_wfmda_host_rx_group),
	.wfmda_wm_tx_group = soc3_0_wfmda_wm_tx_group,
	.wfmda_wm_tx_group_len = ARRAY_SIZE(soc3_0_wfmda_wm_tx_group),
	.wfmda_wm_rx_group = soc3_0_wfmda_wm_rx_group,
	.wfmda_wm_rx_group_len = ARRAY_SIZE(soc3_0_wfmda_wm_rx_group),
	.prDmashdlCfg = &rMT6885DmashdlCfg,
	.prPleTopCr = &rSoc3_0_PleTopCr,
	.prPseTopCr = &rSoc3_0_PseTopCr,
	.prPpTopCr = &rSoc3_0_PpTopCr,
	.prPseGroup = rSoc3_0_pse_group,
	.u4PseGroupLen = ARRAY_SIZE(rSoc3_0_pse_group),
	.pdmaSetup = soc3_0asicConnac2xWpdmaConfig,
	.enableInterrupt = asicConnac2xEnablePlatformIRQ,
	.disableInterrupt = asicConnac2xDisablePlatformIRQ,
	.disableSwInterrupt = asicConnac2xDisablePlatformSwIRQ,
	.processTxInterrupt = soc3_0asicConnac2xProcessTxInterrupt,
	.processRxInterrupt = soc3_0asicConnac2xProcessRxInterrupt,
	.processAbnormalInterrupt = soc3_0ProcessAbnormalInterrupt,
	.tx_ring_ext_ctrl = asicConnac2xWfdmaTxRingExtCtrl,
	.rx_ring_ext_ctrl = asicConnac2xWfdmaRxRingExtCtrl,
	/* null wfdmaManualPrefetch if want to disable manual mode */
	.wfdmaManualPrefetch = soc3_0asicConnac2xWfdmaManualPrefetch,
	.lowPowerOwnRead = asicConnac2xLowPowerOwnRead,
	.lowPowerOwnSet = asicConnac2xLowPowerOwnSet,
	.lowPowerOwnClear = asicConnac2xLowPowerOwnClear,
	.wakeUpWiFi = asicWakeUpWiFi,
	.processSoftwareInterrupt = asicConnac2xProcessSoftwareInterrupt,
	.softwareInterruptMcu = asicConnac2xSoftwareInterruptMcu,
	.hifRst = asicConnac2xHifRst,

	.initPcieInt = NULL,
	.devReadIntStatus = soc3_0ReadExtIntStatus,
	.DmaShdlInit = mt6885DmashdlInit,
	.setRxRingHwAddr = soc3_0SetRxRingHwAddr,
	.wfdmaAllocRxRing = soc3_0WfdmaAllocRxRing,

	.rSwWfdmaInfo = {
		.rOps = {
			.init = halSwWfdmaInit,
			.uninit = halSwWfdmaUninit,
			.enable = halSwWfdmaEn,
			.reset = halSwWfdmaReset,
			.backup = halSwWfdmaBackup,
			.restore = halSwWfdmaRestore,
			.getCidx = halSwWfdmaGetCidx,
			.setCidx = halSwWfdmaSetCidx,
			.getDidx = halSwWfdmaGetDidx,
			.writeCmd = halSwWfdmaWriteCmd,
			.processDmaDone = halSwWfdmaProcessDmaDone,
			.triggerInt = soc3_0_triggerInt,
			.getIntSta = soc3_0_getIntSta,
			.dumpDebugLog = halSwWfdmaDumpDebugLog,
		},
		.fgIsEnSwWfdma = false,
		.fgIsEnAfterFwdl = false,
		.u4EmiOffsetAddr = 0x7c0537fc,
		.u4EmiOffsetBase = 0,
		.u4EmiOffsetMask = 0xFFFFFF,
		.u4EmiOffset = 0x974FFC,
		.u4CcifStartAddr = 0x1024D008,
		.u4CcifTchnumAddr = 0x1024D00C,
		.u4CcifChlNum = 4,
		.u4CpuIdx = 0,
		.u4DmaIdx = 0,
		.u4MaxCnt = TX_RING_SIZE,
	},
#endif			/*_HIF_PCIE || _HIF_AXI */
};

#if CFG_ENABLE_FW_DOWNLOAD
struct FWDL_OPS_T soc3_0_fw_dl_ops = {
#if (CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1)
	.constructFirmwarePrio = soc3_0_ConstructFirmwarePrio,
#else
	.constructFirmwarePrio = NULL,
#endif
	.constructPatchName = soc3_0_ConstructPatchName,
	.downloadPatch = wlanDownloadPatch,
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
#endif			/* CFG_ENABLE_FW_DOWNLOAD */

struct TX_DESC_OPS_T soc3_0_TxDescOps = {
	.fillNicAppend = fillNicTxDescAppend,
	.fillHifAppend = fillTxDescAppendByHostV2,
	.fillTxByteCount = fillConnac2xTxDescTxByteCount,
};

struct RX_DESC_OPS_T soc3_0_RxDescOps = {
};

struct CHIP_DBG_OPS soc3_0_debug_ops = {
	.showPdmaInfo = connac2x_show_wfdma_info,
	.showPseInfo = connac2x_show_pse_info,
	.showPleInfo = connac2x_show_ple_info,
	.showTxdInfo = connac2x_show_txd_Info,
	.showWtblInfo = connac2x_show_wtbl_info,
	.showUmacFwtblInfo = connac2x_show_umac_wtbl_info,
	.showCsrInfo = NULL,
	.showDmaschInfo = connac2x_show_dmashdl_info,
	.dumpMacInfo = soc3_0_dump_mac_info,
	.dumpTxdInfo = connac2x_dump_tmac_info,
	.getFwDebug = connac2x_get_ple_int,
	.setFwDebug = connac2x_set_ple_int,
	.showHifInfo = NULL,
	.printHifDbgInfo = NULL,
	.show_rx_rate_info = connac2x_show_rx_rate_info,
	.show_rx_rssi_info = connac2x_show_rx_rssi_info,
	.show_stat_info = connac2x_show_stat_info,
	.show_wfdma_dbg_probe_info = soc3_0_show_wfdma_dbg_probe_info,
#ifdef CFG_SUPPORT_LINK_QUALITY_MONITOR
	.get_rx_rate_info = connac2x_get_rx_rate_info
#endif
};

#if CFG_SUPPORT_QA_TOOL
struct ATE_OPS_T soc3_0_AteOps = {
	/*ICapStart phase out , wlan_service instead*/
	.setICapStart = connacSetICapStart,
	/*ICapStatus phase out , wlan_service instead*/
	.getICapStatus = connacGetICapStatus,
	/*CapIQData phase out , wlan_service instead*/
	.getICapIQData = connacGetICapIQData,
	.getRbistDataDumpEvent = nicExtEventICapIQData,
	.icapRiseVcoreClockRate = soc3_0_icapRiseVcoreClockRate,
	.icapDownVcoreClockRate = soc3_0_icapDownVcoreClockRate
};
#endif


/* Litien code refine to support multi chip */
struct mt66xx_chip_info mt66xx_chip_info_soc3_0 = {
	.bus_info = &soc3_0_bus_info,
#if CFG_ENABLE_FW_DOWNLOAD
	.fw_dl_ops = &soc3_0_fw_dl_ops,
#endif				/* CFG_ENABLE_FW_DOWNLOAD */
#if CFG_SUPPORT_QA_TOOL
	.prAteOps = &soc3_0_AteOps,
#endif /*CFG_SUPPORT_QA_TOOL*/
	.prDebugOps = &soc3_0_debug_ops,
	.prTxDescOps = &soc3_0_TxDescOps,
	.prRxDescOps = &soc3_0_RxDescOps,
	.chip_id = SOC3_0_CHIP_ID,
	.should_verify_chip_id = FALSE,
	.sw_sync0 = SOC3_0_SW_SYNC0,
	.sw_ready_bits = WIFI_FUNC_NO_CR4_READY_BITS,
	.sw_ready_bit_offset = SOC3_0_SW_SYNC0_RDY_OFFSET,
	.patch_addr = SOC3_0_PATCH_START_ADDR,
	.is_support_cr4 = FALSE,
	.is_support_wacpu = FALSE,
	.txd_append_size = SOC3_0_TX_DESC_APPEND_LENGTH,
	.rxd_size = SOC3_0_RX_DESC_LENGTH,
	.init_evt_rxd_size = SOC3_0_RX_DESC_LENGTH,
	.pse_header_length = CONNAC2X_NIC_TX_PSE_HEADER_LENGTH,
	.init_event_size = CONNAC2X_RX_INIT_EVENT_LENGTH,
	.eco_info = soc3_0_eco_table,
	.isNicCapV1 = FALSE,
	.top_hcr = CONNAC2X_TOP_HCR,
	.top_hvr = CONNAC2X_TOP_HVR,
	.top_fvr = CONNAC2X_TOP_FVR,
	.arb_ac_mode_addr = SOC3_0_ARB_AC_MODE_ADDR,
	.custom_oid_interface_version = MTK_CUSTOM_OID_INTERFACE_VERSION,
	.em_interface_version = MTK_EM_INTERFACE_VERSION,
	.asicCapInit = asicConnac2xCapInit,
#if CFG_ENABLE_FW_DOWNLOAD
	.asicEnableFWDownload = NULL,
#endif				/* CFG_ENABLE_FW_DOWNLOAD */
	.asicGetChipID = asicGetChipID,
	.downloadBufferBin = NULL,
	.is_support_hw_amsdu = TRUE,
	.is_support_asic_lp = TRUE,
	.is_support_wfdma1 = TRUE,
	.get_rxv_from_rxrpt = TRUE,
	.is_support_nvram_fragment = TRUE,
	.asicWfdmaReInit = asicConnac2xWfdmaReInit,
	.asicWfdmaReInit_handshakeInit = asicConnac2xWfdmaDummyCrWrite,
	.group5_size = sizeof(struct HW_MAC_RX_STS_GROUP_5),
	.u4LmacWtblDUAddr = CONNAC2X_WIFI_LWTBL_BASE,
	.u4UmacWtblDUAddr = CONNAC2X_WIFI_UWTBL_BASE,
	.wmmcupwron = wf_pwr_on_consys_mcu,
	.wmmcupwroff = wf_pwr_off_consys_mcu,
#if (CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1)
	.pwrondownload = soc3_0_wlanPowerOnDownload,
#else
	.pwrondownload = NULL,
#endif
	.triggerfwassert = soc3_0_Trigger_fw_assert,
	.dumpwfsyscpupcr = connac2x_DumpWfsyscpupcr,

	.coantSetWiFi = wlanCoAntWiFi,
	.coantSetMD = wlanCoAntMD,
	.coantVFE28En = wlanCoAntVFE28En,
	.coantVFE28Dis = wlanCoAntVFE28Dis,
#if (CFG_SUPPORT_CONNINFRA == 1)
	.coexpccifon = wlanConnacPccifon,
	.coexpccifoff = wlanConnacPccifoff,
	.get_sw_interrupt_status = soc3_0_get_sw_interrupt_status,
#endif
#if (CFG_SUPPORT_PRE_ON_PHY_ACTION == 1)
	.getCalResult = wlanGetCalResult,
	.calDebugCmd = wlanCalDebugCmd,
#endif
	.checkbushang = soc3_0_CheckBusHang,
	.dumpBusHangCr = soc3_0_DumpBusHangCr,
	.cmd_max_pkt_size = CFG_TX_MAX_PKT_SIZE, /* size 1600 */
};

struct mt66xx_hif_driver_data mt66xx_driver_data_soc3_0 = {
	.chip_info = &mt66xx_chip_info_soc3_0,
};

void soc3_0_DumpWfsyscpupcr(struct ADAPTER *prAdapter)
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

	if (prAdapter == NULL)
		return;

	for (i = 0; i < CPUPCR_LOG_NUM; i++) {
		log_sec = local_clock();
		log_nsec = do_div(log_sec, 1000000000)/1000;
		HAL_MCR_RD(prAdapter, WFSYS_CPUPCR_ADDR, &var_pc);
		HAL_MCR_RD(prAdapter, WFSYS_LP_ADDR, &var_lp);

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

void soc3_0_DumpWfsysdebugflag(void)
{
	uint32_t i = 0, u4Value = 0;
	uint32_t RegValue = 0x000F0001;

	for (i = 0; i < 15; i++) {
		wf_ioremap_write(0x18060128, RegValue);
		wf_ioremap_read(0x18060148, &u4Value);
		DBGLOG(HAL, INFO,
			"Bus hang dump: 0x18060148 = 0x%08x after 0x%08x\n",
			u4Value, RegValue);
		RegValue -= 0x10000;
	}
	RegValue = 0x00030002;
	for (i = 0; i < 3; i++) {
		wf_ioremap_write(0x18060128, RegValue);
		wf_ioremap_read(0x18060148, &u4Value);
		DBGLOG(HAL, INFO,
			"Bus hang dump: 0x18060148 = 0x%08x after 0x%08x\n",
			u4Value, RegValue);
		RegValue -= 0x10000;
	}
		RegValue = 0x00040003;
	for (i = 0; i < 4; i++) {
		wf_ioremap_write(0x18060128, RegValue);
		wf_ioremap_read(0x18060148, &u4Value);
		DBGLOG(HAL, INFO,
			"Bus hang dump: 0x18060148 = 0x%08x after 0x%08x\n",
			u4Value, RegValue);
		RegValue -= 0x10000;
	}

}

void soc3_0_DumpWfsysInfo(void)
{
	int value = 0;
	int value_2 = 0;
	int i;

	for (i = 0; i < 5; i++) {
		wf_ioremap_read(0x18060204, &value);
		wf_ioremap_read(0x18060208, &value_2);
		DBGLOG(HAL, INFO,
			"MCU PC: 0x%08x, MCU LP: 0x%08x\n", value, value_2);
	}
}

int soc3_0_Trigger_fw_assert(void)
{
	int ret = 0;
	int value = 0;
	uint32_t waitRet = 0;
	struct ADAPTER *prAdapter = NULL;
	struct GLUE_INFO *prGlueInfo = NULL;

	WIPHY_PRIV(wlanGetWiphy(), prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;

	soc3_0_CheckBusHang(NULL, FALSE);
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
		soc3_0_DumpWfsysInfo();
		DBGLOG(INIT, INFO, "Trigger assert successfully.\n");
	} else {
		/* Case 2: timeout */
		DBGLOG(INIT, ERROR,
			"Trigger assert more than 2 seconds, need to trigger rst self\n");
		soc3_0_DumpWfsysInfo();
		soc3_0_DumpWfsysdebugflag();
	}
#if (CFG_SUPPORT_CONNINFRA == 1)
		kalSetRstEvent();
#endif
	wf_ioremap_read(WF_TRIGGER_AP2CONN_EINT, &value);
	value |= 0x80;
	ret = wf_ioremap_write(WF_TRIGGER_AP2CONN_EINT, value);

	return ret;
}

void soc3_0_CheckBusHangUT(void)
{
#define BUS_HANG_UT_WAIT_COUNT 30

	struct ADAPTER *prAdapter = NULL;
	struct GLUE_INFO *prGlueInfo = NULL;
	uint32_t count = 0;
	uint32_t u4Value = 0;
	uint32_t RegValue = 0;

	WIPHY_PRIV(wlanGetWiphy(), prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;

	HAL_MCR_RD(prAdapter, 0x7c00162c, &u4Value);
	RegValue = u4Value | BIT(0);
	HAL_MCR_WR(prAdapter, 0x7c00162c, RegValue);

	while (count < BUS_HANG_UT_WAIT_COUNT) {
		HAL_MCR_RD(prAdapter, 0x7c00162c, &u4Value);
		DBGLOG(HAL, INFO, "%s: 0x7c00162c = 0x%08x\n",
				__func__, u4Value);

		if ((u4Value&BIT(3)) == BIT(3)) {
			DBGLOG(HAL, ERROR, "%s: 0x7c00162c = 0x%08x\n",
					__func__, u4Value);
			break;
		}
		count++;
	}

	/* Trigger Hang */
	HAL_MCR_RD(prAdapter, 0x7c060000, &u4Value);
}

static uint32_t soc3_0_DumpHwDebugFlagSub(struct ADAPTER *prAdapter,
	uint32_t RegValue)
{
	uint32_t u4Cr;
	uint32_t u4Value = 0;

	u4Cr = 0x1806009C;
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

	u4Cr = 0x1806021C;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);

	return u4Value;
}

static void soc3_0_DumpHwDebugFlag(struct ADAPTER *prAdapter)
{
#define	HANG_HW_FLAG_NUM		7

	uint32_t u4Cr;
	uint32_t RegValue = 0;
	uint32_t log[HANG_HW_FLAG_NUM];

	DBGLOG(HAL, LOUD,
		"Host_CSR - dump all HW Debug flag");

	u4Cr = 0x18060094;
	RegValue = 0x00139CE7;
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

	log[0] = soc3_0_DumpHwDebugFlagSub(prAdapter, 0x366CD932);
	log[1] = soc3_0_DumpHwDebugFlagSub(prAdapter, 0x36AD5A34);
	log[2] = soc3_0_DumpHwDebugFlagSub(prAdapter, 0x36EDDB36);
	log[3] = soc3_0_DumpHwDebugFlagSub(prAdapter, 0x3972E54A);
	log[4] = soc3_0_DumpHwDebugFlagSub(prAdapter, 0x39B3664C);
	log[5] = soc3_0_DumpHwDebugFlagSub(prAdapter, 0x7C387060);
	log[6] = soc3_0_DumpHwDebugFlagSub(prAdapter, 0x7C78F162);

	connac2x_dump_format_memory32(log, HANG_HW_FLAG_NUM, "HW Debug flag");
}

static void soc3_0_DumpPcLrLog(struct ADAPTER *prAdapter)
{
#define	HANG_PC_LOG_NUM			32
	uint32_t u4Cr, u4Index, i;
	uint32_t u4Value = 0;
	uint32_t RegValue = 0;
	uint32_t log[HANG_PC_LOG_NUM];

	DBGLOG(HAL, LOUD,
		"Host_CSR - dump PC log / LR log");

	memset(log, 0, HANG_PC_LOG_NUM);

	/* PC log
	* dbg_pc_log_sel	Write	0x1806_0090 [7:2]	6'h20
	*  choose 33th pc log buffer to read current pc log buffer index
	* read pc from host CR	Read	0x1806_0204 [21:17]
	*  read current pc log buffer index
	* dbg_pc_log_sel	Write	0x1806_0090 [7:2]	index
	*  set pc log buffer index to read pc log
	* read pc from host CR	Read	0x1806_0204 [31:0]
	*  read pc log of the specific index
	*/

	u4Cr = 0x18060090;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = (0x20<<2) | (u4Value&BITS(0, 1)) | (u4Value&BITS(8, 31));
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

	u4Cr = 0x18060204;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	u4Index = ((u4Value&BITS(17, 21)) >> 17) + 1;

	for (i = 0; i < HANG_PC_LOG_NUM; i++) {

		u4Index++;

		if (u4Index == HANG_PC_LOG_NUM)
			u4Index = 0;

		u4Cr = 0x18060090;
		connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
		RegValue = (u4Index<<2) | (u4Value&BITS(0, 1)) |
			(u4Value&BITS(8, 31));
		connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

		u4Cr = 0x18060204;
		connac2x_DbgCrRead(prAdapter, u4Cr, &log[i]);
	}

	/* restore */
	u4Cr = 0x18060090;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = (0x3F<<2) | (u4Value&BITS(0, 1)) | (u4Value&BITS(8, 31));
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

	connac2x_dump_format_memory32(log, HANG_PC_LOG_NUM, "PC log");

	/* GPR log */

	u4Cr = 0x18060090;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = (0x20<<8) | (u4Value&BITS(0, 7)) | (u4Value&BITS(14, 31));
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

	u4Cr = 0x18060208;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	u4Index = ((u4Value&BITS(17, 21)) >> 17) + 1;

	for (i = 0; i < HANG_PC_LOG_NUM; i++) {

		u4Index++;

		if (u4Index == HANG_PC_LOG_NUM)
			u4Index = 0;

		u4Cr = 0x18060090;
		connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
		RegValue = (u4Index<<8) | (u4Value&BITS(0, 7)) |
			(u4Value&BITS(14, 31));
		connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

		u4Cr = 0x18060208;
		connac2x_DbgCrRead(prAdapter, u4Cr, &log[i]);
	}

	/* restore */
	u4Cr = 0x18060090;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = (0x3F<<8) | (u4Value&BITS(0, 7)) | (u4Value&BITS(14, 31));
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

	connac2x_dump_format_memory32(log, HANG_PC_LOG_NUM, "GPR log");
}

static void soc3_0_DumpN10CoreReg(struct ADAPTER *prAdapter)
{
#define	HANG_N10_CORE_LOG_NUM	38
	uint32_t u4Cr, i;
	uint32_t u4Value = 0;
	uint32_t RegValue = 0;
	uint32_t log[HANG_N10_CORE_LOG_NUM];

	DBGLOG(HAL, LOUD,
		"Host_CSR - read N10 core register");

	memset(log, 0, HANG_N10_CORE_LOG_NUM);

/*
*	[31:26]: gpr_index_sel (set different sets of gpr) = 0
*	[13:8]: gpr buffer index setting
*		(set as 0x3F to select the current selected GPR)
*/
	u4Cr = 0x18060090;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);

	u4Value = (0x3F<<8) | (u4Value&BITS(0, 7)) | (u4Value&BITS(14, 31));

	for (i = 0; i < HANG_N10_CORE_LOG_NUM; i++) {

		RegValue = (i<<26) | (u4Value&BITS(0, 25));

		u4Cr = 0x18060090;
		connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

		u4Cr = 0x18060208;
		connac2x_DbgCrRead(prAdapter, u4Cr, &log[i]);
	}

	/* restore */
	u4Cr = 0x18060090;
	RegValue = (30<<26) | (u4Value&BITS(0, 25));
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

	connac2x_dump_format_memory32(
		log, HANG_N10_CORE_LOG_NUM, "N10 core register");
}


static int soc3_0_wakeupConninfra(void)
{
	int check;
	int value = 0;
	int ret = 0;
	int conninfra_hang_ret = 0;
	unsigned int polling_count;

	/* Wakeup conn_infra off write 0x180601A4[0] = 1'b1 */
	wf_ioremap_read(CONN_INFRA_WAKEUP_WF_ADDR, &value);
	value |= 0x00000001;
	wf_ioremap_write(CONN_INFRA_WAKEUP_WF_ADDR, value);

	/* Check AP2CONN slpprot ready
	 * (polling "10 times" and each polling interval is "1ms")
	 * Address: 0x1806_0184[5]
	 * Data: 1'b0
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_ON2OFF_SLP_PROT_ACK_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & 0x00000020) != 0) {
		if (polling_count > 10) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(1000);
		wf_ioremap_read(CONN_INFRA_ON2OFF_SLP_PROT_ACK_ADDR, &value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR, "Polling  AP2CONN slpprot ready fail.\n");
		return ret;
	}

	if (!conninfra_reg_readable()) {
		DBGLOG(HAL, ERROR,
			"conninfra_reg_readable fail\n");

		conninfra_hang_ret = conninfra_is_bus_hang();

		if (conninfra_hang_ret > 0) {

			DBGLOG(HAL, ERROR,
				"conninfra_is_bus_hang, Chip reset\n");
		}
		return -1;
	}

	/* Check CONNSYS version ID
	 * (polling "10 times" and each polling interval is "1ms")
	 * Address: 0x1800_1000[31:0]
	 * Data: 0x2001_0000
	 * Action: polling
	 */
	wf_ioremap_read(CONN_HW_VER_ADDR, &value);
	check = 0;
	polling_count = 0;
	while (value != kalGetConnsysVerId()) {
		if (polling_count > 10) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(1000);
		wf_ioremap_read(CONN_HW_VER_ADDR, &value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR, "Polling CONNSYS version ID fail.\n");
		return ret;
	}

	return ret;
}

static void soc3_0_disableConninfraForceOn(void)
{
	int value = 0;

	/* Disable conn_infra off domain force on 0x180601A4[0] = 1'b0 */
	wf_ioremap_read(CONN_INFRA_WAKEUP_WF_ADDR, &value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(CONN_INFRA_WAKEUP_WF_ADDR, value);
}

static void soc3_0_DumpOtherCr(struct ADAPTER *prAdapter)
{
#define	HANG_OTHER_LOG_NUM		2

	connac2x_DumpCrRange(NULL,
		CONN_MCU_CONFG_ON_HOST_MAILBOX_WF_ADDR,
		HANG_OTHER_LOG_NUM,
		"mailbox and other CRs");
	connac2x_DumpCrRange(NULL, 0x180602c0, 8, "DBG_DUMMY");
	connac2x_DumpCrRange(NULL, 0x180602e0, 4, "BT_CSR_DUMMY");
	connac2x_DumpCrRange(NULL, 0x180602f0, 4, "WF_CSR_DUMMY");

	if (soc3_0_wakeupConninfra() == 0) {
		DBGLOG(INIT, ERROR, "wake up conninfra.\n");
		connac2x_DumpCrRange(NULL, 0x18052900, 16,
				     "conninfra Sysram BT");
		connac2x_DumpCrRange(NULL, 0x18053000, 16,
				     "conninfra Sysram WF");
	}

	soc3_0_disableConninfraForceOn();
}

static void soc3_0_DumpSpecifiedWfTop(struct ADAPTER *prAdapter)
{
#define	HANG_TOP_LOG_NUM		2

	uint32_t u4Cr;
	uint32_t u4Value = 0;
	uint32_t RegValue = 0;
	uint32_t log[HANG_TOP_LOG_NUM];

	DBGLOG(HAL, LOUD,
		"Host_CSR - specified WF TOP monflg on");

	memset(log, 0, HANG_TOP_LOG_NUM);

/* 0x1806009C[28]=1	write	enable wf_mcu_misc */

	u4Cr = 0x1806009C;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = u4Value | BIT(28);
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

/* 0x1806009C[27:0]=0xC387060	write	select {FLAG_4[9:2],FLAG_27[9:2]} */

	u4Cr = 0x1806009C;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = 0xC387060 | (u4Value&BITS(28, 31));
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

/* 0x18060094[20]=1	write	enable wf_monflg_on */

	u4Cr = 0x18060094;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = u4Value | BIT(20);
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

/* 0x18060094[19:0]=0x39CE7	write	select wf_mcusys_dbg */

	u4Cr = 0x18060094;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = 0x39CE7 | (u4Value&BITS(20, 31));
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

/* 0x1806_021c[31:0]	Read	get {FLAG_4[9:2],FLAG_27[9:2]} */

	u4Cr = 0x1806021c;
	connac2x_DbgCrRead(prAdapter, u4Cr, &log[0]);

/* 0x1806009C[27:0]=0xC78F162	write	select {FlAG_23[7:0],FlAG_6[9:2]} */

	u4Cr = 0x1806009C;
	connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
	RegValue = 0xC78F162 | (u4Value&BITS(28, 31));
	connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);


/* 0x1806_021c[31:0]	Read	get {FlAG_23[7:0],FlAG_6[9:2]} */

	u4Cr = 0x1806021c;
	connac2x_DbgCrRead(prAdapter, u4Cr, &log[1]);

	connac2x_dump_format_memory32(log, HANG_TOP_LOG_NUM,
		"specified WF TOP monflg on");
}


/* check wsys bus hang or not */
/* return = TRUE (bus is hang), FALSE (bus is not hang) */
/* write  0x18060128  = 0x000B0001 */
/* read and check 0x18060148 bit[9:8] */
/* if [9:8]=2'b00 (wsys not hang), others is hang */

static int IsWsysBusHang(struct ADAPTER *prAdapter)
{
	uint32_t u4Value = 0;
	uint32_t u4WriteDebugValue;

	u4WriteDebugValue = 0x000B0001;
	connac2x_DbgCrWrite(prAdapter, 0x18060128, u4WriteDebugValue);
	connac2x_DbgCrRead(prAdapter, 0x18060148, &u4Value);

	u4Value &= BITS(8, 9);
	return u4Value;
} /* soc3_0_IsWsysBusHang */

#if 0
/* PP CR: 0x820CCXXX remap to Base + 0x40e000  */
static void DumpPPDebugCr(struct ADAPTER *prAdapter)
{
	uint32_t ReadRegValue[4];
	uint32_t u4Value[4];

	/* 0x820CC0F0 : PP DBG_CTRL */
	ReadRegValue[0] = 0x820CC0F0;
	HAL_MCR_RD(prAdapter, ReadRegValue[0], &u4Value[0]);

	/* 0x820CC0F8 : PP DBG_CS0 */
	ReadRegValue[1] = 0x820CC0F8;
	HAL_MCR_RD(prAdapter, ReadRegValue[1], &u4Value[1]);

	/* 0x820CC0FC : PP DBG_CS1 */
	ReadRegValue[2] = 0x820CC0FC;
	HAL_MCR_RD(prAdapter, ReadRegValue[2], &u4Value[2]);

	/* 0x820CC100 : PP DBG_CS2 */
	ReadRegValue[3] = 0x820CC100;
	HAL_MCR_RD(prAdapter, ReadRegValue[3], &u4Value[3]);

	DBGLOG(HAL, INFO,
	"PP[0x%08x]=0x%08x,[0x%08x]=0x%08x,[0x%08x]=0x%08x,[0x%08x]=0x%08x,",
		ReadRegValue[0], u4Value[0],
		ReadRegValue[1], u4Value[1],
		ReadRegValue[2], u4Value[2],
		ReadRegValue[3], u4Value[3]);
}

/* PP CR: 0x820CCXXX remap to Base + 0x40e000  */
static void DumpPPDebugCr_without_adapter(void)
{
	uint32_t ReadRegValue[4];
	uint32_t u4Value[4];

	/* 0x820CC0F0 : PP DBG_CTRL */
	ReadRegValue[0] = 0x1840E0F0;
	wf_ioremap_read(ReadRegValue[0], &u4Value[0]);

	/* 0x820CC0F8 : PP DBG_CS0 */
	ReadRegValue[1] = 0x1840E0F8;
	wf_ioremap_read(ReadRegValue[1], &u4Value[1]);

	/* 0x820CC0FC : PP DBG_CS1 */
	ReadRegValue[2] = 0x1840E0FC;
	wf_ioremap_read(ReadRegValue[2], &u4Value[2]);

	/* 0x820CC100 : PP DBG_CS2 */
	ReadRegValue[3] = 0x1840E100;
	wf_ioremap_read(ReadRegValue[3], &u4Value[3]);

	DBGLOG(HAL, INFO,
	"PP[0x%08x]=0x%08x,[0x%08x]=0x%08x,[0x%08x]=0x%08x,[0x%08x]=0x%08x,",
		ReadRegValue[0], u4Value[0],
		ReadRegValue[1], u4Value[1],
		ReadRegValue[2], u4Value[2],
		ReadRegValue[3], u4Value[3]);
}
#endif

/* need to dump AXI Master related CR 0x1802750C ~ 0x18027530*/
static void DumpAXIMasterDebugCr(struct ADAPTER *prAdapter)
{
#define AXI_MASTER_DUMP_CR_START 0x1802750C
#define	AXI_MASTER_DUMP_CR_NUM 10

	uint32_t ReadRegValue = 0;
	uint32_t u4Value[AXI_MASTER_DUMP_CR_NUM] = {0};
	uint32_t i;

	ReadRegValue = AXI_MASTER_DUMP_CR_START;
	for (i = 0 ; i < AXI_MASTER_DUMP_CR_NUM; i++) {
		connac2x_DbgCrRead(prAdapter, ReadRegValue, &u4Value[i]);
		ReadRegValue += 4;
	}

	connac2x_dump_format_memory32(u4Value,
		AXI_MASTER_DUMP_CR_NUM,
		"HW AXI BUS debug CR start[0x1802750C]");

} /* soc3_0_DumpAXIMasterDebugCr */

/* Dump Flow :								*/
/*	1) dump WFDMA / AXI Master CR				*/
/*	2) check wsys bus is hang					*/
/*		- if not hang dump WM WFDMA & PP CR	*/
void soc3_0_DumpWFDMACr(struct ADAPTER *prAdapter)
{
	/* Dump Host side WFDMACR */
	bool bShowWFDMA_type = FALSE;
	int32_t ret = 0;

	if (prAdapter == NULL)
		soc3_0_show_wfdma_info_by_type_without_adapter(bShowWFDMA_type);
	else
		soc3_0_show_wfdma_info_by_type(prAdapter, bShowWFDMA_type);

	DumpAXIMasterDebugCr(prAdapter);

	ret = IsWsysBusHang(prAdapter);
	/* ret =0 is readable, wsys not bus hang */
	#if 0 /* TODO */
	if (ret == 0) {
		bShowWFDMA_type = TRUE;
		if (prAdapter == NULL) {
			soc3_0_show_wfdma_info_by_type_without_adapter(
				bShowWFDMA_type);

			DumpPPDebugCr_without_adapter();
		} else {
			soc3_0_show_wfdma_info_by_type(prAdapter,
				bShowWFDMA_type);

			DumpPPDebugCr(prAdapter);
		}
	} else {
		DBGLOG(HAL, INFO,
		"Wifi bus hang(0x%08x), can't dump wsys CR\n", ret);
	}
	#endif

} /* soc3_0_DumpWFDMAHostCr */

static void soc3_0_DumpHostCr(struct ADAPTER *prAdapter, bool fgIsReadable)
{
	soc3_0_DumpWfsyscpupcr(prAdapter);	/* first dump */
	soc3_0_DumpPcLrLog(prAdapter);
	soc3_0_DumpN10CoreReg(prAdapter);
	soc3_0_DumpOtherCr(prAdapter);
	soc3_0_DumpHwDebugFlag(prAdapter);
	soc3_0_DumpSpecifiedWfTop(prAdapter);
	soc3_0_DumpWFDMACr(prAdapter);
} /* soc3_0_DumpHostCr */

void soc3_0_DumpBusHangCr(struct ADAPTER *prAdapter)
{
	conninfra_is_bus_hang();
	soc3_0_DumpHostCr(prAdapter, conninfra_reg_readable());
}

int soc3_0_CheckBusHang(void *adapter, uint8_t ucWfResetEnable)
{
	int ret = 1;
	int conninfra_read_ret = 0;
	int conninfra_hang_ret = 0;
	uint8_t conninfra_reset = FALSE;
	uint32_t u4Cr = 0;
	uint32_t u4Value = 0;
	uint32_t RegValue = 0;
	struct ADAPTER *prAdapter = (struct ADAPTER *) adapter;

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

		if ((prAdapter != NULL) && (prAdapter->fgIsFwDownloaded)) {
/*
* 2. Check MCU wake up and setting mux sel done CR (mailbox)
*  - 0x1806_0260[31] should be 1'b1  (FW view 0x8900_0100[31])
*/

			u4Cr = CONN_MCU_CONFG_ON_HOST_MAILBOX_WF_ADDR;
			connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
			if ((u4Value & BIT(31)) != BIT(31)) {
				DBGLOG(HAL, ERROR,
					"Bus hang check: 0x%08x = 0x%08x\n",
					u4Cr, u4Value);
				break;
			}

/*
* 3. Check wf_mcusys bus hang irq status (need set debug ctrl enable first,
*     ref sheet "Debug ctrl setting")
*  - if(bus hang), then
*  a) WF MCU: wf_mcusys_vdnr_timeout_irq_b, FW: Trigger subssy reset /
*     Whole Chip Reset(conn2ap hang)
*  b) Driver dump CRs of sheet "Debug ctrl setting"
*
* Write Address : 0x1806_009c[6:0]	, Data : 0x60	default 0x00
* Write Address : 0x1806_009c[28]	, Data : 0x1	default 0x0
*  (wf_mcu_dbg enable)
* Write Address : 0x1806_0094[4:0]	, Data : 0x7	default 0x0
*  (switch monflg mux)
* Write Address : 0x1806_0094[20]	, Data : 0x1	default 0x1
*  (enable wf monflg debug)
* Read Address : 0x1806_021c[0] shoulde be 1'b0
*/

			u4Cr = 0x1806009c;
			connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
			RegValue = (u4Value&BITS(7, 31)) | 0x60;
			RegValue = RegValue | BIT(28);
			connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

			u4Cr = 0x18060094;
			connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
			RegValue = (u4Value&BITS(5, 31)) | 0x7;
			RegValue = RegValue | BIT(20);
			connac2x_DbgCrWrite(prAdapter, u4Cr, RegValue);

			u4Cr = 0x1806021c;
			connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);

			if ((u4Value&BIT(0)) == BIT(0)) {
				DBGLOG(HAL, ERROR,
					"Bus hang check: 0x%08x = 0x%08x\n",
					u4Cr, u4Value);

				u4Cr = 0x1806009c;
				connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
				DBGLOG(HAL, ERROR,
					"Bus hang check: 0x%08x = 0x%08x\n",
					u4Cr, u4Value);

				u4Cr = 0x18060094;
				connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
				DBGLOG(HAL, ERROR,
					"Bus hang check: 0x%08x = 0x%08x\n",
					u4Cr, u4Value);

				/* check false alarm */
				u4Cr = CONN_MCU_CONFG_ON_HOST_MAILBOX_WF_ADDR;
				connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);
				DBGLOG(HAL, ERROR,
					"Bus hang check: 0x%08x = 0x%08x\n",
					u4Cr, u4Value);
				if ((u4Value & 0x0001FF87) != 0x00000001) {
					if (((u4Value & 0x00019E00) != 0) ||
					    ((u4Value & 0x00000180) == 0))
						break;
				}
			}
		} else {
			DBGLOG(HAL, INFO,
				"Before fgIsFwDownloaded\n");
		}

/*
* 4. Check conn2wf sleep protect
*  - 0x1800_1620[3] (sleep protect enable ready), should be 1'b0
*/
		u4Cr = 0x18001620;
		connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);

		if ((u4Value&BIT(3)) == BIT(3)) {
			DBGLOG(HAL, ERROR,
				"Bus hang check: 0x%08x = 0x%08x\n",
				u4Cr, u4Value);

			u4Cr = 0x18060010;
			connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);

			DBGLOG(HAL, ERROR,
				"Bus hang check: 0x%08x = 0x%08x\n",
				u4Cr, u4Value);

			u4Cr = 0x180600f0;
			connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);

			DBGLOG(HAL, ERROR,
				"Bus hang check: 0x%08x = 0x%08x\n",
				u4Cr, u4Value);
			break;
		}

/*
* 5. check wfsys bus clock
*  - 0x1806_0000[15] , 1: means bus no clock, 0: ok
*/
		u4Cr = 0x18060000;
		connac2x_DbgCrRead(prAdapter, u4Cr, &u4Value);

		if ((u4Value&BIT(15)) == BIT(15)) {
			DBGLOG(HAL, ERROR,
				"Bus hang check: 0x%08x = 0x%08x\n",
				u4Cr, u4Value);
			break;
		}

		DBGLOG(HAL, TRACE, "Bus hang check: Done\n");

		ret = 0;
	} while (FALSE);

	if (ret > 0) {

		/* check again for dump log */
		conninfra_is_bus_hang();

		if ((conninfra_hang_ret != CONNINFRA_ERR_RST_ONGOING) &&
			(conninfra_hang_ret != CONNINFRA_INFRA_BUS_HANG) &&
			(conninfra_hang_ret !=
				CONNINFRA_AP2CONN_RX_SLP_PROT_ERR) &&
			(conninfra_hang_ret !=
				CONNINFRA_AP2CONN_TX_SLP_PROT_ERR) &&
			(conninfra_hang_ret != CONNINFRA_AP2CONN_CLK_ERR))
			soc3_0_DumpHostCr(prAdapter, conninfra_read_ret);

		if (conninfra_reset) {
			g_IsWfsysBusHang = TRUE;
			glResetWholeChipResetTrigger("bus hang");
		} else if (ucWfResetEnable) {
			g_IsWfsysBusHang = TRUE;
			glResetWholeChipResetTrigger("wifi bus hang");
		}
	}

	return ret;
}

void soc3_0_DumpBusHangdebuglog(void)
{
	uint32_t u4Value = 0;
	uint32_t RegValue;

	RegValue = 0x00020002;
	wf_ioremap_write(0x18060128, RegValue);
	wf_ioremap_read(0x18060148, &u4Value);
	DBGLOG(HAL, INFO,
			"dump: 0x18060148 = 0x%08x after 0x%08x\n",
			u4Value, RegValue);
	wf_ioremap_read(0x18001a00, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x18001a00 = 0x%08x\n", u4Value);
	wf_ioremap_read(0x1800c00c, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x1800c00c = 0x%08x\n", u4Value);

}
void soc3_0_DumpPwrStatedebuglog(void)
{
	uint32_t u4Value = 0;

	wf_ioremap_read(0x18070400, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x18070400 = 0x%08x\n", u4Value);
	wf_ioremap_read(0x18071400, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x18071400 = 0x%08x\n", u4Value);
	wf_ioremap_read(0x18072400, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x18072400 = 0x%08x\n", u4Value);
	wf_ioremap_read(0x18073400, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x18073400 = 0x%08x\n", u4Value);
	wf_ioremap_read(0x180602cc, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x180602cc = 0x%08x\n", u4Value);
	wf_ioremap_read(0x18000110, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x18000110 = 0x%08x\n", u4Value);
	wf_ioremap_read(0x184c0880, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x184c0880 = 0x%08x\n", u4Value);
	wf_ioremap_read(0x184c08d0, &u4Value);
	DBGLOG(HAL, INFO,
		"dump: 0x184c08d0 = 0x%08x\n", u4Value);
}

static int wf_pwr_on_consys_mcu(void)
{
	int check;
	int value = 0;
	int ret = 0;
	int conninfra_hang_ret = 0;
	unsigned int polling_count;

	DBGLOG(INIT, INFO, "wmmcu power-on start.\n");
	ret = soc3_0_wakeupConninfra();
	if (ret != 0)
		return ret;
	soc3_0_DumpBusHangdebuglog();

	/* Assert CONNSYS WM CPU SW reset write 0x18000010[0] = 1'b0*/
	wf_ioremap_read(WFSYS_CPU_SW_RST_B_ADDR, &value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(WFSYS_CPU_SW_RST_B_ADDR, value);

	/* bus clock ctrl */
	conninfra_bus_clock_ctrl(CONNDRV_TYPE_WIFI, CONNINFRA_BUS_CLOCK_ALL, 1);
	/* Turn on wfsys_top_on
	 * 0x18000000[31:16] = 0x57460000,
	 * 0x18000000[7] = 1'b1
	 */
	wf_ioremap_read(WFSYS_ON_TOP_PWR_CTL_ADDR, &value);
	value &= 0x0000FF7F;
	value |= 0x57460080;
	wf_ioremap_write(WFSYS_ON_TOP_PWR_CTL_ADDR, value);
	/* Polling wfsys_rgu_off_hreset_rst_b
	 * (polling "10 times" and each polling interval is "0.5ms")
	 * Address: 0x1806_02CC[2] (TOP_DBG_DUMMY_3_CONNSYS_PWR_STATUS[2])
	 * Data: 1'b1
	 * Action: polling
	 */

	wf_ioremap_read(TOP_DBG_DUMMY_3_CONNSYS_PWR_STATUS_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & 0x00000004) == 0) {
		if (polling_count > 10) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(TOP_DBG_DUMMY_3_CONNSYS_PWR_STATUS_ADDR,
				&value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling wfsys rgu off fail. (0x%x)\n",
			value);
		return ret;
	}

	if (!conninfra_reg_readable()) {
		DBGLOG(HAL, ERROR,
			"conninfra_reg_readable fail\n");

		conninfra_hang_ret = conninfra_is_bus_hang();

		if (conninfra_hang_ret > 0) {

			DBGLOG(HAL, ERROR,
				"conninfra_is_bus_hang, Chip reset\n");
		}
		return -1;
	}
	/* Turn off "conn_infra to wfsys"/ wfsys to conn_infra" bus
	 * sleep protect 0x18001620[0] = 1'b0
	 */
	wf_ioremap_read(CONN_INFRA_WF_SLP_CTRL_R_ADDR, &value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(CONN_INFRA_WF_SLP_CTRL_R_ADDR, value);
	/* Polling WF_SLP_CTRL ready
	 * (polling "100 times" and each polling interval is "0.5ms")
	 * Address: 0x1800_1620[3] (CONN_INFRA_WF_SLP_CTRL_R_OFFSET[3])
	 * Data: 1'b0
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_WF_SLP_CTRL_R_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & 0x00000008) != 0) {
		if (polling_count > 100) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(CONN_INFRA_WF_SLP_CTRL_R_ADDR, &value);
		polling_count++;
		DBGLOG(INIT, ERROR, "WF_SLP_CTRL (0x%x) (%d)\n",
			value,
			polling_count);
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling WFSYS TO CONNINFRA SLEEP PROTECT fail. (0x%x)\n",
			value);
		return ret;
	}
	/* Turn off "wf_dma to conn_infra" bus sleep protect
	 * 0x18001624[0] = 1'b0
	 */
	wf_ioremap_read(CONN_INFRA_WFDMA_SLP_CTRL_R_ADDR, &value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(CONN_INFRA_WFDMA_SLP_CTRL_R_ADDR, value);
	/* Polling wfsys_rgu_off_hreset_rst_b
	 * (polling "100 times" and each polling interval is "0.5ms")
	 * Address: 0x1800_1624[3] (CONN_INFRA_WFDMA_SLP_CTRL_R_OFFSET[3])
	 * Data: 1'b0
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_WFDMA_SLP_CTRL_R_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & 0x00000008) != 0) {
		if (polling_count > 100) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(CONN_INFRA_WFDMA_SLP_CTRL_R_ADDR, &value);
		polling_count++;
		DBGLOG(INIT, ERROR, "WFDMA_SLP_CTRL (0x%x) (%d)\n",
			value,
			polling_count);
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling WFDMA TO CONNINFRA SLEEP PROTECT RDY fail. (0x%x)\n",
			value);
		return ret;
	}

	/*WF_VDNR_EN_ADDR 0x1800_E06C[0] =1'b1*/
	wf_ioremap_read(WF_VDNR_EN_ADDR, &value);
	value |= 0x00000001;
	wf_ioremap_write(WF_VDNR_EN_ADDR, value);

	/* Check WFSYS version ID (Polling) */
	wf_ioremap_read(WFSYS_VERSION_ID_ADDR, &value);
	check = 0;
	polling_count = 0;
	while (value != WFSYS_VERSION_ID) {
		if (polling_count > 10) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(WFSYS_VERSION_ID_ADDR, &value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR, "Polling CONNSYS version ID fail.\n");
		return ret;
	}

	/* Setup CONNSYS firmware in EMI */
#if (CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1)
	soc3_0_wlanPowerOnInit(ENUM_WLAN_POWER_ON_DOWNLOAD_EMI);
#endif

	/* Default value update 2: EMI entry address */
	wf_ioremap_write(CONN_CFG_AP2WF_REMAP_1_ADDR, CONN_MCU_CONFG_HS_BASE);
	wf_ioremap_write(WF_DYNAMIC_BASE+MCU_EMI_ENTRY_OFFSET, 0x00000000);
	wf_ioremap_write(WF_DYNAMIC_BASE+WF_EMI_ENTRY_OFFSET, 0x00000000);

	/* Reset WFSYS semaphore 0x18000018[0] = 1'b0 */
	wf_ioremap_read(WFSYS_SW_RST_B_ADDR, &value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(WFSYS_SW_RST_B_ADDR, value);

	/* De-assert WFSYS CPU SW reset 0x18000010[0] = 1'b1 */
	wf_ioremap_read(WFSYS_CPU_SW_RST_B_ADDR, &value);
	value |= 0x00000001;
	wf_ioremap_write(WFSYS_CPU_SW_RST_B_ADDR, value);

	/* Check CONNSYS power-on completion
	 * Polling "100 times" and each polling interval is "0.5ms"
	 * Polling 0x81021604[31:0] = 0x00001D1E
	 */
	wf_ioremap_read(WF_ROM_CODE_INDEX_ADDR, &value);
	check = 0;
	polling_count = 0;
	while (value != CONNSYS_ROM_DONE_CHECK) {
		if (polling_count > 100) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(1000);
		wf_ioremap_read(WF_ROM_CODE_INDEX_ADDR, &value);
		polling_count++;
	}
	if (check != 0) {
		soc3_0_DumpWfsysInfo();
		soc3_0_DumpPwrStatedebuglog();
		soc3_0_DumpWfsysdebugflag();
		DBGLOG(INIT, ERROR,
			"Check CONNSYS power-on completion fail.\n");
		return ret;
	}

	conninfra_config_setup();

	/* bus clock ctrl */
	conninfra_bus_clock_ctrl(CONNDRV_TYPE_WIFI, CONNINFRA_BUS_CLOCK_ALL, 0);

	soc3_0_disableConninfraForceOn();

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
	ret = soc3_0_wakeupConninfra();
	if (ret != 0)
		return ret;

	/* mask wf2conn slpprot idle
	 * 0x1800F004[0] = 1'b1
	 */
	wf_ioremap_read(WF2CONN_SLPPROT_IDLE_ADDR, &value);
	value |= 0x00000001;
	wf_ioremap_write(WF2CONN_SLPPROT_IDLE_ADDR, value);

	/* Turn on "conn_infra to wfsys"/ wfsys to conn_infra" bus sleep protect
	 * 0x18001620[0] = 1'b1
	 */
	wf_ioremap_read(CONN_INFRA_WF_SLP_CTRL_R_ADDR, &value);
	value |= 0x00000001;
	wf_ioremap_write(CONN_INFRA_WF_SLP_CTRL_R_ADDR, value);

	/* Polling WF_SLP_CTRL ready
	 * (polling "100 times" and each polling interval is "0.5ms")
	 * Address: 0x1800_1620[3] (CONN_INFRA_WF_SLP_CTRL_R_OFFSET[3])
	 * Data: 1'b1
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_WF_SLP_CTRL_R_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & 0x00000008) == 0) {
		if (polling_count > 100) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(CONN_INFRA_WF_SLP_CTRL_R_ADDR, &value);
		polling_count++;
		DBGLOG(INIT, ERROR, "WF_SLP_CTRL (0x%x) (%d)\n",
			value,
			polling_count);
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling WFSYS TO CONNINFRA SLEEP PROTECT fail. (0x%x)\n",
			value);
		soc3_0_DumpWfsysInfo();
		soc3_0_DumpWfsysdebugflag();
	}
	/* Turn off "wf_dma to conn_infra" bus sleep protect
	 * 0x18001624[0] = 1'b1
	 */
	wf_ioremap_read(CONN_INFRA_WFDMA_SLP_CTRL_R_ADDR, &value);
	value |= 0x00000001;
	wf_ioremap_write(CONN_INFRA_WFDMA_SLP_CTRL_R_ADDR, value);

	/* Polling wfsys_rgu_off_hreset_rst_b
	 * (polling "100 times" and each polling interval is "0.5ms")
	 * Address: 0x1800_1624[3] (CONN_INFRA_WFDMA_SLP_CTRL_R_OFFSET[3])
	 * Data: 1'b1
	 * Action: polling
	 */
	wf_ioremap_read(CONN_INFRA_WFDMA_SLP_CTRL_R_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & 0x00000008) == 0) {
		if (polling_count > 100) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(CONN_INFRA_WFDMA_SLP_CTRL_R_ADDR, &value);
		polling_count++;
		DBGLOG(INIT, ERROR, "WFDMA_SLP_CTRL (0x%x) (%d)\n",
			value,
			polling_count);
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling WFDMA TO CONNINFRA SLEEP PROTECT RDY fail. (0x%x)\n",
			value);
		soc3_0_DumpWfsysInfo();
		soc3_0_DumpWfsysdebugflag();
	}

	/* Reset WFSYS semaphore 0x18000018[0] = 1'b0 */
	wf_ioremap_read(WFSYS_SW_RST_B_ADDR, &value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(WFSYS_SW_RST_B_ADDR, value);

	/* bus clock ctrl */
	conninfra_bus_clock_ctrl(CONNDRV_TYPE_WIFI, CONNINFRA_BUS_CLOCK_ALL, 0);

	/* Turn off wfsys_top_on
	 * 0x18000000[31:16] = 0x57460000,
	 * 0x18000000[7] = 1'b0
	 */
	wf_ioremap_read(WFSYS_ON_TOP_PWR_CTL_ADDR, &value);
	value &= 0x0000FF7F;
	value |= 0x57460000;
	wf_ioremap_write(WFSYS_ON_TOP_PWR_CTL_ADDR, value);
	/* Polling wfsys_rgu_off_hreset_rst_b
	 * (polling "10 times" and each polling interval is "0.5ms")
	 * Address: 0x1806_02CC[2] (TOP_DBG_DUMMY_3_CONNSYS_PWR_STATUS[2])
	 * Data: 1'b0
	 * Action: polling
	 */

	wf_ioremap_read(TOP_DBG_DUMMY_3_CONNSYS_PWR_STATUS_ADDR, &value);
	check = 0;
	polling_count = 0;
	while ((value & 0x00000004) != 0) {
		if (polling_count > 10) {
			check = -1;
			ret = -1;
			break;
		}
		udelay(500);
		wf_ioremap_read(TOP_DBG_DUMMY_3_CONNSYS_PWR_STATUS_ADDR,
				&value);
		polling_count++;
	}
	if (check != 0) {
		DBGLOG(INIT, ERROR,
			"Polling wfsys rgu off fail. (0x%x)\n",
			value);
		return ret;
	}

	/* unmask wf2conn slpprot idle
	 * 0x1800F004[0] = 1'b0
	 */
	wf_ioremap_read(WF2CONN_SLPPROT_IDLE_ADDR, &value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(WF2CONN_SLPPROT_IDLE_ADDR, value);

	/* Toggle WFSYS EMI request 0x18001c14[0] = 1'b1 -> 1'b0 */
	wf_ioremap_read(CONN_INFRA_WFSYS_EMI_REQ_ADDR, &value);
	value |= 0x00000001;
	wf_ioremap_write(CONN_INFRA_WFSYS_EMI_REQ_ADDR, value);
	value &= 0xFFFFFFFE;
	wf_ioremap_write(CONN_INFRA_WFSYS_EMI_REQ_ADDR, value);

	/*Disable A-die top_ck_en (use common API)(clear driver & FW resource)*/
	conninfra_adie_top_ck_en_off(CONNSYS_ADIE_CTL_FW_WIFI);

	soc3_0_DumpBusHangdebuglog();
	soc3_0_disableConninfraForceOn();
	return ret;
}

void wlanCoAntVFE28En(IN struct ADAPTER *prAdapter)
{
	struct WIFI_CFG_PARAM_STRUCT *prNvramSettings;
	struct REG_INFO *prRegInfo;
	u_int8_t fgCoAnt;

	if (g_NvramFsm != NVRAM_STATE_READY) {
		DBGLOG(INIT, INFO, "CoAntVFE28 NVRAM Not ready\n");
		return;
	}

	ASSERT(prAdapter);
	prRegInfo = &prAdapter->prGlueInfo->rRegInfo;
	ASSERT(prRegInfo);
	prNvramSettings = prRegInfo->prNvramSettings;
	ASSERT(prNvramSettings);

	fgCoAnt = prNvramSettings->ucSupportCoAnt;

	if (fgCoAnt) {
		if (gCoAntVFE28En == FALSE) {
#if (KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE)
			/* Implementation for kernel-5.4 */
#elif (KERNEL_VERSION(4, 15, 0) <= CFG80211_VERSION_CODE)
			regmap_write(g_regmap,
				MT6359_LDO_VFE28_OP_EN_SET, 0x1 << 8);
			regmap_write(g_regmap,
				MT6359_LDO_VFE28_OP_CFG_CLR, 0x1 << 8);
#else
			KERNEL_pmic_ldo_vfe28_lp(8, 0, 1, 0);
#endif
			DBGLOG(INIT, INFO, "CoAntVFE28 PMIC Enable\n");
			gCoAntVFE28En = TRUE;
		} else {
			DBGLOG(INIT, INFO, "CoAntVFE28 PMIC Already Enable\n");
		}
	} else {
		DBGLOG(INIT, INFO, "Not Support CoAnt Enable\n");
	}
}

void wlanCoAntVFE28Dis(void)
{
	if (gCoAntVFE28En == TRUE) {
#if (KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE)
		/* Implementation for kernel-5.4 */
#elif (KERNEL_VERSION(4, 15, 0) <= CFG80211_VERSION_CODE)
		regmap_write(g_regmap, MT6359_LDO_VFE28_OP_EN_CLR, 0x1 << 8);
		regmap_write(g_regmap, MT6359_LDO_VFE28_OP_CFG_CLR, 0x1 << 8);
		regmap_write(g_regmap, MT6359_LDO_VFE28_OP_CFG_CLR, 0x1 << 8);
#else
		KERNEL_pmic_ldo_vfe28_lp(8, 0, 0, 0);
#endif
		DBGLOG(INIT, INFO, "CoAntVFE28 PMIC Disable\n");
		gCoAntVFE28En = FALSE;
	} else {
		DBGLOG(INIT, INFO, "CoAntVFE28 PMIC Already Disable\n");
	}
}

void wlanCoAntWiFi(void)
{
	uint32_t u4GPIO10 = 0x0;

	wf_ioremap_read(0x100053a0, &u4GPIO10);
	u4GPIO10 |= 0x20000;
	wf_ioremap_write(0x100053a0, u4GPIO10);

}

void wlanCoAntMD(void)
{
	uint32_t u4GPIO10 = 0x0;

	wf_ioremap_read(0x100053a0, &u4GPIO10);
	u4GPIO10 |= 0x10000;
	wf_ioremap_write(0x100053a0, u4GPIO10);

}


#if (CFG_SUPPORT_CONNINFRA == 1)
int wlanConnacPccifon(void)
{
	int ret = 0;

	/*reset WiFi power on status to MD*/
	wf_ioremap_write(0x10003314, 0x00);
	/*set WiFi power on status to MD*/
	wf_ioremap_write(0x10003314, 0x01);
       /*
	*Ccif4 (ccif_md2conn_wf):
	*write cg gate 0x1000_10C0[28] & [29] (write 1 set)
	*write cg gate 0x1000_10C4[28] & [29] (write 1 clear)
	*Connsys/AP is used bit 28,md is used bit 29
	*default value is 0,clk enable
	*Set cg must set both bit[28] [29], and clk turn off
	*Clr cg set either bit[28][29], and clk turn on

       *Enable PCCIF4 clock
       *HW auto control, so no need to turn on or turn off
	*wf_ioremap_read(0x100010c4, &reg);
	*reg |= BIT(28);
	*ret = wf_ioremap_write(0x100010c4,reg);
	*/
	return ret;
}

int wlanConnacPccifoff(void)
{
	int ret = 0;

	/*reset WiFi power on status to MD*/
	ret = wf_ioremap_write(0x10003314, 0x00);
	/*reset WiFi power on status to MD*/
	ret = wf_ioremap_write(0x1024c014, 0x0ff);

	/*
	*Ccif4 (ccif_md2conn_wf):
	*write cg gate 0x1000_10C0[28] & [29] (write 1 set)
	*write cg gate 0x1000_10C4[28] & [29] (write 1 clear)
	*Connsys/AP is used bit 28, md is used bit 29
	*default value is 0, clk enable
	*Set cg must set both bit[28] [29], and clk turn off
	*Clr cg set either bit[28][29], and clk turn on

       *Disable PCCIF4 clock
	*HW auto control, so no need to turn on or turn off
	*wf_ioremap_read(0x100010c0, &reg);
	*reg |= BIT(28);
	*ret = wf_ioremap_write(0x100010c0,reg);
	*/
	return ret;
}
#endif

void soc3_0_icapRiseVcoreClockRate(void)
{
	int value = 0;

	/*2 update Clork Rate*/
	/*0x1000123C[20]=1,218Mhz*/
	wf_ioremap_read(WF_CONN_INFA_BUS_CLOCK_RATE, &value);
	value |= 0x00010000;
	wf_ioremap_write(WF_CONN_INFA_BUS_CLOCK_RATE, value);
#if (CFG_SUPPORT_VCODE_VDFS == 1)
	/*Enable VCore to 0.725*/

#if (KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE)
		/* Implementation for kernel-5.4 */
#else
	/*init*/
	if (!pm_qos_request_active(&wifi_req))
		pm_qos_add_request(&wifi_req, PM_QOS_VCORE_OPP,
						PM_QOS_VCORE_OPP_DEFAULT_VALUE);

	/*update Vcore*/
	pm_qos_update_request(&wifi_req, 0);
#endif

	DBGLOG(HAL, STATE, "icapRiseVcoreClockRate done\n");
#else
	DBGLOG(HAL, STATE, "icapRiseVcoreClockRate skip\n");
#endif  /*#ifndef CFG_BUILD_X86_PLATFORM*/
}

void soc3_0_icapDownVcoreClockRate(void)
{


	int value = 0;

	/*2 update Clork Rate*/
	/*0x1000123C[20]=0,156Mhz*/
	wf_ioremap_read(WF_CONN_INFA_BUS_CLOCK_RATE, &value);
	value &= ~(0x00010000);
	wf_ioremap_write(WF_CONN_INFA_BUS_CLOCK_RATE, value);
#if (CFG_SUPPORT_VCODE_VDFS == 1)
#if (KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE)
	/* Implementation for kernel-5.4 */
#else
	/*init*/
	if (!pm_qos_request_active(&wifi_req))
		pm_qos_add_request(&wifi_req, PM_QOS_VCORE_OPP,
						PM_QOS_VCORE_OPP_DEFAULT_VALUE);

	/*restore to default Vcore*/
	pm_qos_update_request(&wifi_req,
		PM_QOS_VCORE_OPP_DEFAULT_VALUE);

	/*disable VCore to normal setting*/
	DBGLOG(HAL, STATE, "icapDownVcoreClockRate done!\n");
#endif
#else
	DBGLOG(HAL, STATE, "icapDownVcoreClockRate skip\n");

#endif  /*#ifndef CFG_BUILD_X86_PLATFORM*/

}

#if (CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1)
#pragma message("SOC3_0::CFG_POWER_ON_DOWNLOAD_EMI_ROM_PATCH == 1")
void soc3_0_ConstructFirmwarePrio(struct GLUE_INFO *prGlueInfo,
	uint8_t **apucNameTable, uint8_t **apucName,
	uint8_t *pucNameIdx, uint8_t ucMaxNameIdx)
{
	int ret = 0;
	uint8_t ucIdx = 0;
	uint8_t aucFlavor[2] = {0};

	kalGetFwFlavor(&aucFlavor[0]);

	for (ucIdx = 0; apucsoc3_0FwName[ucIdx]; ucIdx++) {
		if ((*pucNameIdx + 3) >= ucMaxNameIdx) {
			/* the table is not large enough */
			DBGLOG(INIT, ERROR,
				"kalFirmwareImageMapping >> file name array is not enough.\n");
			ASSERT(0);
			continue;
		}

		/* Type 1. WIFI_RAM_CODE_soc1_0_1_1.bin */
		ret = kalSnprintf(*(apucName + (*pucNameIdx)),
				CFG_FW_NAME_MAX_LEN, "%s_%u%s_%u.bin",
				apucsoc3_0FwName[ucIdx],
				CFG_WIFI_IP_SET,
				aucFlavor,
				1);
		if (ret >= 0 && ret < CFG_FW_NAME_MAX_LEN)
			(*pucNameIdx) += 1;
		else
			DBGLOG(INIT, ERROR,
					"[%u] kalSnprintf failed, ret: %d\n",
					__LINE__, ret);

		/* Type 2. WIFI_RAM_CODE_soc1_0_1_1 */
		ret = kalSnprintf(*(apucName + (*pucNameIdx)),
				CFG_FW_NAME_MAX_LEN, "%s_%u%s_%u",
				apucsoc3_0FwName[ucIdx],
				CFG_WIFI_IP_SET,
				aucFlavor,
				1);
		if (ret >= 0 && ret < CFG_FW_NAME_MAX_LEN)
			(*pucNameIdx) += 1;
		else
			DBGLOG(INIT, ERROR,
					"[%u] kalSnprintf failed, ret: %d\n",
					__LINE__, ret);

		/* Type 3. WIFI_RAM_CODE_soc1_0 */
		ret = kalSnprintf(*(apucName + (*pucNameIdx)),
				CFG_FW_NAME_MAX_LEN, "%s",
				apucsoc3_0FwName[ucIdx]);
		if (ret >= 0 && ret < CFG_FW_NAME_MAX_LEN)
			(*pucNameIdx) += 1;
		else
			DBGLOG(INIT, ERROR,
					"[%u] kalSnprintf failed, ret: %d\n",
					__LINE__, ret);

		/* Type 4. WIFI_RAM_CODE_soc1_0.bin */
		ret = kalSnprintf(*(apucName + (*pucNameIdx)),
				CFG_FW_NAME_MAX_LEN, "%s.bin",
				apucsoc3_0FwName[ucIdx]);
		if (ret >= 0 && ret < CFG_FW_NAME_MAX_LEN)
			(*pucNameIdx) += 1;
		else
			DBGLOG(INIT, ERROR,
					"[%u] kalSnprintf failed, ret: %d\n",
					__LINE__, ret);
	}
}

void *
soc3_0_kalFirmwareImageMapping(
			IN struct GLUE_INFO *prGlueInfo,
			OUT void **ppvMapFileBuf,
			OUT uint32_t *pu4FileLength,
			IN enum ENUM_IMG_DL_IDX_T eDlIdx)
{
	uint8_t **apucNameTable = NULL;
	uint8_t *apucName[SOC3_0_FILE_NAME_TOTAL +
					  1]; /* extra +1, for the purpose of
					       * detecting the end of the array
					       */
	uint8_t idx = 0, max_idx,
		aucNameBody[SOC3_0_FILE_NAME_TOTAL][SOC3_0_FILE_NAME_MAX],
		sub_idx = 0;
	struct mt66xx_chip_info *prChipInfo =
			prGlueInfo->prAdapter->chip_info;
	/* uint32_t chip_id = prChipInfo->chip_id; */
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
			apucNameTable = soc3_0_apucFwName;
			break;

		case IMG_DL_IDX_CR4_FW:
			apucNameTable = soc3_0_apucCr4FwName;
			break;

		case IMG_DL_IDX_PATCH:
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
			/* soc3_0_patch_wmmcu_1_1_hdr.bin */
			if (prChipInfo->fw_dl_ops->constructPatchName)
				prChipInfo->fw_dl_ops->constructPatchName(
					prGlueInfo, apucName, &idx);
			else
				kalSnprintf(apucName[idx], SOC3_0_FILE_NAME_MAX,
					"soc3_0_patch_wmmcu_1_%x_hdr.bin",
					wlanGetEcoVersion(
						prGlueInfo->prAdapter));
			idx += 1;
		} else if (eDlIdx == IMG_DL_IDX_MCU_ROM_EMI) {
			/* construct the file name for MCU ROM EMI */
			/* soc3_0_ram_wmmcu_1_1_hdr.bin */
			kalSnprintf(apucName[idx], SOC3_0_FILE_NAME_MAX,
				"soc3_0_ram_wmmcu_%u%s_%x_hdr.bin",
				CFG_WIFI_IP_SET,
				aucFlavor,
				wlanGetEcoVersion(
					prGlueInfo->prAdapter));

			idx += 1;
		} else if (eDlIdx == IMG_DL_IDX_WIFI_ROM_EMI) {
			/* construct the file name for WiFi ROM EMI */
			/* soc3_0_ram_wifi_1_1_hdr.bin */
			kalSnprintf(apucName[idx], SOC3_0_FILE_NAME_MAX,
				"soc3_0_ram_wifi_%u%s_%x_hdr.bin",
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

uint32_t soc3_0_wlanImageSectionDownloadStage(
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

	} else if ((eDlIdx == IMG_DL_IDX_MCU_ROM_EMI) ||
				(eDlIdx == IMG_DL_IDX_WIFI_ROM_EMI)) {
		prRomEmiHeader = (struct ROM_EMI_HEADER *)pvFwImageMapFile;

		DBGLOG(INIT, INFO,
			"DL %s ROM EMI %s\n",
			(eDlIdx == IMG_DL_IDX_MCU_ROM_EMI) ?
				"MCU":"WiFi",
			(char *)prRomEmiHeader->ucDateTime);

		u4Addr = prRomEmiHeader->u4PatchAddr;

		u4Len = u4FwImageFileLength - sizeof(struct ROM_EMI_HEADER);

		u4Offset = sizeof(struct ROM_EMI_HEADER);

		u4Status = wlanDownloadEMISection(prAdapter,
					u4Addr, u4Len,
					pvFwImageMapFile + u4Offset);
#if (CFG_SUPPORT_CONNINFRA == 1)
		/* Set datecode to EMI */
		if (eDlIdx == IMG_DL_IDX_MCU_ROM_EMI)
			wlanDownloadEMISection(prAdapter,
				WMMCU_MCU_ROM_EMI_DATE_ADDR,
				DATE_CODE_SIZE, prRomEmiHeader->ucDateTime);
		else
			wlanDownloadEMISection(prAdapter,
				WMMCU_WIFI_ROM_EMI_DATE_ADDR,
				DATE_CODE_SIZE, prRomEmiHeader->ucDateTime);
#endif
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

/* WiFi power on download MCU/WiFi ROM EMI + ROM patch */
/*----------------------------------------------------------------------------*/
/*!
 * \brief Wlan power on download function. This function prepare the job
 *  during power on stage:
 *  [1]Download MCU + WiFi ROM EMI
 *  [2]Download ROM patch
 *
 * \retval 0 Success
 * \retval negative value Failed
 */
/*----------------------------------------------------------------------------*/
uint32_t soc3_0_wlanPowerOnDownload(
	IN struct ADAPTER *prAdapter,
	IN uint8_t ucDownloadItem)
{
	uint32_t u4FwSize = 0;
	void *prFwBuffer = NULL;
	uint32_t u4Status;
#if CFG_SUPPORT_COMPRESSION_FW_OPTION
	uint8_t ucIsCompressed;
#endif
	if (!prAdapter)
		return WLAN_STATUS_FAILURE;


	DBGLOG_LIMITED(INIT, INFO,
		"Power on download start(%d)\n", ucDownloadItem);

	switch (ucDownloadItem) {
	case ENUM_WLAN_POWER_ON_DOWNLOAD_EMI:
		/* Download MCU ROM EMI*/
		soc3_0_kalFirmwareImageMapping(prAdapter->prGlueInfo,
			&prFwBuffer, &u4FwSize, IMG_DL_IDX_MCU_ROM_EMI);

		if (prFwBuffer == NULL) {
			DBGLOG(INIT, WARN, "FW[%u] load error!\n",
			       IMG_DL_IDX_MCU_ROM_EMI);
			return WLAN_STATUS_FAILURE;
		}

		u4Status = soc3_0_wlanImageSectionDownloadStage(
			prAdapter, prFwBuffer, u4FwSize, 1,
			IMG_DL_IDX_MCU_ROM_EMI);

		kalFirmwareImageUnmapping(
			prAdapter->prGlueInfo, NULL, prFwBuffer);

		DBGLOG_LIMITED(INIT, INFO, "Power on download mcu ROM EMI %s\n",
			(u4Status == WLAN_STATUS_SUCCESS) ? "pass" : "failed");

		/* Download WiFi ROM EMI*/
		if (u4Status == WLAN_STATUS_SUCCESS) {
			soc3_0_kalFirmwareImageMapping(prAdapter->prGlueInfo,
				&prFwBuffer, &u4FwSize,
				IMG_DL_IDX_WIFI_ROM_EMI);

			if (prFwBuffer == NULL) {
				DBGLOG(INIT, WARN, "FW[%u] load error!\n",
				       IMG_DL_IDX_WIFI_ROM_EMI);
				return WLAN_STATUS_FAILURE;
			}

			u4Status = soc3_0_wlanImageSectionDownloadStage(
				prAdapter, prFwBuffer, u4FwSize, 1,
				IMG_DL_IDX_WIFI_ROM_EMI);

			kalFirmwareImageUnmapping(
				prAdapter->prGlueInfo, NULL, prFwBuffer);

			DBGLOG_LIMITED(INIT, INFO,
				"Power on download WiFi ROM EMI %s\n",
				(u4Status == WLAN_STATUS_SUCCESS)
					? "pass" : "failed");
		}

		break;

	case ENUM_WLAN_POWER_ON_DOWNLOAD_ROM_PATCH:
		prAdapter->rVerInfo.fgPatchIsDlByDrv = FALSE;

		soc3_0_kalFirmwareImageMapping(prAdapter->prGlueInfo,
			&prFwBuffer, &u4FwSize, IMG_DL_IDX_PATCH);

		if (prFwBuffer == NULL) {
			DBGLOG(INIT, WARN, "FW[%u] load error!\n",
			       IMG_DL_IDX_PATCH);
			return WLAN_STATUS_FAILURE;
		}

#if (CFG_ROM_PATCH_NO_SEM_CTRL == 0)
#pragma message("ROM code supports SEM-CTRL for ROM patch download")
		if (wlanPatchIsDownloaded(prAdapter)) {
			kalFirmwareImageUnmapping(prAdapter->prGlueInfo, NULL,
						  prFwBuffer);
			DBGLOG_LIMITED(INIT, INFO,
				"No need to download patch\n");
			return WLAN_STATUS_SUCCESS;
		}
#else
#pragma message("ROM code supports no SEM-CTRL for ROM patch download")
#endif

		/* Patch DL */
#if CFG_SUPPORT_COMPRESSION_FW_OPTION
		u4Status = wlanCompressedImageSectionDownloadStage(
			prAdapter, prFwBuffer, u4FwSize, 1,
			IMG_DL_IDX_PATCH, &ucIsCompressed, NULL);
#else
		u4Status = soc3_0_wlanImageSectionDownloadStage(
			prAdapter, prFwBuffer, u4FwSize, 1, IMG_DL_IDX_PATCH);
#endif

/* Dynamic memory map::Begin */
#if (CFG_DOWNLOAD_DYN_MEMORY_MAP == 1)
		if (u4Status == WLAN_STATUS_SUCCESS)
			wlanPatchDynMemMapSendComplete(prAdapter);
		else if (u4Status == WLAN_STATUS_NOT_ACCEPTED)
			u4Status = WLAN_STATUS_SUCCESS; /* already download*/
#else
		wlanPatchSendComplete(prAdapter);
#endif
/* Dynamic memory map::End */

		kalFirmwareImageUnmapping(
			prAdapter->prGlueInfo, NULL, prFwBuffer);

		prAdapter->rVerInfo.fgPatchIsDlByDrv = TRUE;

		break;

	default:
		return WLAN_STATUS_NOT_SUPPORTED;
	}

	DBGLOG_LIMITED(INIT, INFO, "Power on download end[%d].\n", u4Status);

	return u4Status;
}

/* WiFi power on init for MCU/WiFi ROM EMI + ROM patch download */
/*----------------------------------------------------------------------------*/
/*!
 * \brief Wlan power on init function. This function do the job in the
 *  power on stage, they are:
 *  [1]Download MCU + WiFi ROM EMI
 *  [2]Download ROM patch
 *
 *  It is to simulate wlanProbe() with the minimum effort to complete
 *  ROM EMI + ROM patch download.
 *
 * \retval 0 Success
 * \retval negative value Failed
 */
/*----------------------------------------------------------------------------*/
int32_t soc3_0_wlanPowerOnInit(
	enum ENUM_WLAN_POWER_ON_DOWNLOAD eDownloadItem)
{
/*
*	pvData     data passed by bus driver init function
*		_HIF_EHPI: NULL
*		_HIF_SDIO: sdio bus driver handle
*	see hifAxiProbe() for more detail...
*	pfWlanProbe((void *)prPlatDev,
*			(void *)prPlatDev->id_entry->driver_data)
*/
	void *pvData;
	void *pvDriverData = (void *)&mt66xx_driver_data_soc3_0;

	int32_t i4Status = 0;
	enum ENUM_POWER_ON_INIT_FAIL_REASON {
		BUS_INIT_FAIL = 0,
		NET_CREATE_FAIL,
		BUS_SET_IRQ_FAIL,
		ALLOC_ADAPTER_MEM_FAIL,
		DRIVER_OWN_FAIL,
		INIT_ADAPTER_FAIL,
		INIT_HIFINFO_FAIL,
		ROM_PATCH_DOWNLOAD_FAIL,
		POWER_ON_INIT_DONE,
		FAIL_REASON_NUM
	} eFailReason;
	uint32_t i = 0, j = 0;
	int32_t i4DevIdx = 0;
	struct wireless_dev *prWdev = NULL;
	struct GLUE_INFO *prGlueInfo = NULL;
	struct ADAPTER *prAdapter = NULL;
	struct mt66xx_chip_info *prChipInfo;
	struct FWDL_OPS_T *prFwDlOps;

	DBGLOG(INIT, INFO, "wlanPowerOnInit::begin\n");

	eFailReason = FAIL_REASON_NUM;

	do {
		prChipInfo = ((struct mt66xx_hif_driver_data *)pvDriverData)
					->chip_info;
		pvData = (void *)prChipInfo->pdev;

		if (eDownloadItem == ENUM_WLAN_POWER_ON_DOWNLOAD_EMI) {
			if (fgSimplifyResetFlow) {
				WIPHY_PRIV(wlanGetWiphy(), prGlueInfo);
				prAdapter = prGlueInfo->prAdapter;

				if (prChipInfo->pwrondownload) {
					DBGLOG_LIMITED(INIT, INFO,
						"[Wi-Fi PWR On] EMI download Start\n");

					if (prChipInfo->pwrondownload(
						prAdapter, eDownloadItem)
						!= WLAN_STATUS_SUCCESS)
						i4Status =
						-ROM_PATCH_DOWNLOAD_FAIL;

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

					if (prChipInfo->pwrondownload(
						prAdapter, eDownloadItem)
						!= WLAN_STATUS_SUCCESS) {
						i4Status =
						-ROM_PATCH_DOWNLOAD_FAIL;
					}

					DBGLOG_LIMITED(INIT, INFO,
					"[Wi-Fi PWR On] EMI download End\n");
				}

				wlanWakeLockUninit(prGlueInfo);
			}

			wlanNetDestroy(prWdev);
			}

			return i4Status;
		}

		/* [1]Copy from wlanProbe()::Begin */
		/* Initialize the IO port of the interface */
		/* GeorgeKuo: pData has different meaning for _HIF_XXX:
		* _HIF_EHPI: pointer to memory base variable, which will be
		*      initialized by glBusInit().
		* _HIF_SDIO: bus driver handle
		*/

		/* Remember to call glBusRelease() in wlanPowerOnDeinit() */
		if (glBusInit(pvData) == FALSE) {
			DBGLOG(INIT, ERROR,
				"[Wi-Fi PWR On] glBusInit() fail\n");

			i4Status = -EIO;

			eFailReason = BUS_INIT_FAIL;

			break;
		}

#if (CFG_SUPPORT_TRACE_TC4 == 1)
		wlanDebugTC4Init();
#endif

		/* Create network device, Adapter, KalInfo,
		*       prDevHandler(netdev)
		*/
		prWdev = wlanNetCreate(pvData, pvDriverData);

		if (prWdev == NULL) {
			DBGLOG(INIT, ERROR,
				"[Wi-Fi PWR On] No memory for dev and its private\n");

			i4Status = -ENOMEM;

			eFailReason = NET_CREATE_FAIL;

			break;
		}

		/* Set the ioaddr to HIF Info */
		WIPHY_PRIV(prWdev->wiphy, prGlueInfo);

		/* Should we need this??? to be conti... */
		gPrDev = prGlueInfo->prDevHandler;

		/* Setup IRQ */
		i4Status = glBusSetIrq(prWdev->netdev, NULL, prGlueInfo);

		if (i4Status != WLAN_STATUS_SUCCESS) {
			DBGLOG(INIT, ERROR, "[Wi-Fi PWR On] Set IRQ error\n");

			eFailReason = BUS_SET_IRQ_FAIL;

			break;
		}

		prGlueInfo->i4DevIdx = i4DevIdx;

		prAdapter = prGlueInfo->prAdapter;
		/* [1]Copy from wlanProbe()::End */

		/* [2]Copy from wlanProbe()->wlanOnPreAdapterStart()::Begin */
		/* Init Chip Capability */
		prChipInfo = prAdapter->chip_info;

		if (prChipInfo->asicCapInit)
			prChipInfo->asicCapInit(prAdapter);

		/* Trigger the action of switching Pwr state to drv_own */
		prAdapter->fgIsFwOwn = TRUE;

		nicpmWakeUpWiFi(prAdapter);
		/* [2]Copy from wlanProbe()->wlanOnPreAdapterStart()::End */

		/* [3]Copy from
		* wlanProbe()
		*	->wlanAdapterStart()
		*		->wlanOnPreAllocAdapterMem()::Begin
		*/
#if 0  /* Sample's gen4m code base doesn't support */
		prAdapter->u4HifDbgFlag = 0;
		prAdapter->u4HifChkFlag = 0;
		prAdapter->u4NoMoreRfb = 0;
#endif

		prAdapter->u4OwnFailedCount = 0;
		prAdapter->u4OwnFailedLogCount = 0;

		/* Additional with chip reset optimize*/
		prAdapter->ucCmdSeqNum = 0;
		prAdapter->u4PwrCtrlBlockCnt = 0;

		QUEUE_INITIALIZE(&(prAdapter->rPendingCmdQueue));
#if CFG_SUPPORT_MULTITHREAD
		QUEUE_INITIALIZE(&prAdapter->rTxCmdQueue);
		QUEUE_INITIALIZE(&prAdapter->rTxCmdDoneQueue);
#if CFG_FIX_2_TX_PORT
		QUEUE_INITIALIZE(&prAdapter->rTxP0Queue);
		QUEUE_INITIALIZE(&prAdapter->rTxP1Queue);
#else
		for (i = 0; i < BSS_DEFAULT_NUM; i++)
			for (j = 0; j < TX_PORT_NUM; j++)
				QUEUE_INITIALIZE(&prAdapter->rTxPQueue[i][j]);
#endif
		QUEUE_INITIALIZE(&prAdapter->rRxQueue);
		QUEUE_INITIALIZE(&prAdapter->rTxDataDoneQueue);
#endif

		/* reset fgIsBusAccessFailed */
		fgIsBusAccessFailed = FALSE;

		/* Allocate mandatory resource for TX/RX */
		if (nicAllocateAdapterMemory(prAdapter)
			!= WLAN_STATUS_SUCCESS) {
			DBGLOG(INIT, ERROR,
				"[Wi-Fi PWR On] nicAllocateAdapterMemory Error!\n");

			i4Status = -ENOMEM;

			eFailReason = ALLOC_ADAPTER_MEM_FAIL;
/*
*#if CFG_ENABLE_KEYWORD_EXCEPTION_MECHANISM & 0
*			mtk_wcn_wmt_assert_keyword(WMTDRV_TYPE_WIFI,
*			"[Wi-Fi PWR On] nicAllocateAdapterMemory Error!");
*#endif
*/
			break;
		}

		/* should we need this?  to be conti... */
		prAdapter->u4OsPacketFilter = PARAM_PACKET_FILTER_SUPPORTED;

		/* WLAN driver acquire LP own */
		DBGLOG(INIT, TRACE, "[Wi-Fi PWR On] Acquiring LP-OWN\n");

		ACQUIRE_POWER_CONTROL_FROM_PM(prAdapter);

		DBGLOG(INIT, TRACE, "[Wi-Fi PWR On] Acquiring LP-OWN-end\n");

		if (prAdapter->fgIsFwOwn == TRUE) {
			DBGLOG(INIT, ERROR,
				"[Wi-Fi PWR On] nicpmSetDriverOwn() failed!\n");

			eFailReason = DRIVER_OWN_FAIL;
/*
*#if CFG_ENABLE_KEYWORD_EXCEPTION_MECHANISM & 0
*			mtk_wcn_wmt_assert_keyword(WMTDRV_TYPE_WIFI,
*				"[Wi-Fi PWR On] nicpmSetDriverOwn() failed!");
*#endif
*/
			break;
		}

		/* Initialize the Adapter:
		*       verify chipset ID, HIF init...
		*       the code snippet just do the copy thing
		*/
		if (nicInitializeAdapter(prAdapter) != WLAN_STATUS_SUCCESS) {
			DBGLOG(INIT, ERROR,
				"[Wi-Fi PWR On] nicInitializeAdapter failed!\n");

			eFailReason = INIT_ADAPTER_FAIL;

			break;
		}

		/* Do the post NIC init adapter:
		* copy only the mandatory task
		* in wlanOnPostNicInitAdapter(prAdapter, FALSE)::Begin
		*/
		nicInitSystemService(prAdapter, FALSE);

		/* Initialize Tx */
		nicTxInitialize(prAdapter);

		/* Initialize Rx */
		nicRxInitialize(prAdapter);
		/* Do the post NIC init adapter:
		* copy only the mandatory task
		* in wlanOnPostNicInitAdapter(prAdapter, FALSE)::End
		*/

		/* HIF SW info initialize */
		if (!halHifSwInfoInit(prAdapter)) {
			DBGLOG(INIT, ERROR,
				"[Wi-Fi PWR On] halHifSwInfoInit failed!\n");

			eFailReason = INIT_HIFINFO_FAIL;

			break;
		}

		/* Enable HIF  cut-through to N9 mode */
		HAL_ENABLE_FWDL(prAdapter, TRUE);

		wlanSetChipEcoInfo(prAdapter);

		/* should we open it, to be conti */
		/* wlanOnPostInitHifInfo(prAdapter); */

		/* Disable interrupt, download is done by polling mode only */
		nicDisableInterrupt(prAdapter);

		/* Initialize Tx Resource to fw download state */
		nicTxInitResetResource(prAdapter);

		/* MCU ROM EMI +
		* WiFi ROM EMI + ROM patch download goes over here::Begin
		*/
		/* assiggned in wlanNetCreate() */
		prChipInfo = prAdapter->chip_info;

		/* It is configured in mt66xx_chip_info_soc3.fw_dl_ops */
		prFwDlOps = prChipInfo->fw_dl_ops;

		/* No need to check F/W ready bit,
		* since we are downloading MCU ROM EMI
		* + WiFi ROM EMI + ROM patch
		*/
		/*
		*DBGLOG(INIT, INFO,
		*	"wlanDownloadFW:: Check ready_bits(=0x%x)\n",
		*	prChipInfo->sw_ready_bits);
		*
		*HAL_WIFI_FUNC_READY_CHECK(prAdapter,
		*	prChipInfo->sw_ready_bits, &fgReady);
		*/

		if (prChipInfo->pwrondownload) {
			HAL_ENABLE_FWDL(prAdapter, TRUE);

			DBGLOG_LIMITED(INIT, INFO,
				"[Wi-Fi PWR On] download Start\n");

			if (prChipInfo->pwrondownload(prAdapter, eDownloadItem)
				!= WLAN_STATUS_SUCCESS) {
				eFailReason = ROM_PATCH_DOWNLOAD_FAIL;

				HAL_ENABLE_FWDL(prAdapter, FALSE);

				break;
			}

			DBGLOG_LIMITED(INIT, INFO,
				"[Wi-Fi PWR On] download End\n");

			HAL_ENABLE_FWDL(prAdapter, FALSE);
		}
		/* MCU ROM EMI + WiFi ROM EMI
		* + ROM patch download goes over here::End
		*/

		eFailReason = POWER_ON_INIT_DONE;
		/* [3]Copy from wlanProbe()
		*		->wlanAdapterStart()
		*			->wlanOnPreAllocAdapterMem()::End
		*/
	} while (FALSE);

	switch (eFailReason) {
	case BUS_INIT_FAIL:
		break;

	case NET_CREATE_FAIL:
#if (CFG_SUPPORT_TRACE_TC4 == 1)
		wlanDebugTC4Uninit();  /* Uninit for TC4 debug */
#endif

		/* We should call this, although nothing is inside */
		glBusRelease(pvData);

		break;

	case BUS_SET_IRQ_FAIL:
#if (CFG_SUPPORT_TRACE_TC4 == 1)
		wlanDebugTC4Uninit();  /* Uninit for TC4 debug */
#endif

		/* We should call this, although nothing is inside */
		glBusRelease(pvData);

		wlanWakeLockUninit(prGlueInfo);

		wlanNetDestroy(prWdev);

		break;

	case ALLOC_ADAPTER_MEM_FAIL:
	case DRIVER_OWN_FAIL:
	case INIT_ADAPTER_FAIL:
		/* Should we set Onwership to F/W for advanced debug???
		* to be conti...
		*/
		/* nicpmSetFWOwn(prAdapter, FALSE); */

		glBusFreeIrq(prWdev->netdev,
			*((struct GLUE_INFO **)netdev_priv(prWdev->netdev)));

#if (CFG_SUPPORT_TRACE_TC4 == 1)
		wlanDebugTC4Uninit();  /* Uninit for TC4 debug */
#endif

		/* We should call this, although nothing is inside */
		glBusRelease(pvData);

		wlanWakeLockUninit(prGlueInfo);

		wlanNetDestroy(prWdev);

		break;

	case INIT_HIFINFO_FAIL:
		nicRxUninitialize(prAdapter);

		nicTxRelease(prAdapter, FALSE);

		/* System Service Uninitialization */
		nicUninitSystemService(prAdapter);

		/* Should we set Onwership to F/W for advanced debug???
		* to be conti...
		*/
		/* nicpmSetFWOwn(prAdapter, FALSE); */

		glBusFreeIrq(prWdev->netdev,
			*((struct GLUE_INFO **)netdev_priv(prWdev->netdev)));

#if (CFG_SUPPORT_TRACE_TC4 == 1)
		wlanDebugTC4Uninit();  /* Uninit for TC4 debug */
#endif

		/* We should call this, although nothing is inside */
		glBusRelease(pvData);

		wlanWakeLockUninit(prGlueInfo);

		wlanNetDestroy(prWdev);

		break;

	case ROM_PATCH_DOWNLOAD_FAIL:
	case POWER_ON_INIT_DONE:
		HAL_ENABLE_FWDL(prAdapter, FALSE);

		nicRxUninitialize(prAdapter);

		nicTxRelease(prAdapter, FALSE);

		/* System Service Uninitialization */
		nicUninitSystemService(prAdapter);

		/* Should we set Onwership to F/W for advanced debug???
		* to be conti...
		*/
		/* nicpmSetFWOwn(prAdapter, FALSE); */

		glBusFreeIrq(prWdev->netdev,
			*((struct GLUE_INFO **)netdev_priv(prWdev->netdev)));

#if (CFG_SUPPORT_TRACE_TC4 == 1)
		wlanDebugTC4Uninit();  /* Uninit for TC4 debug */
#endif

		/* We should call this, although nothing is inside */
		glBusRelease(pvData);

		wlanWakeLockUninit(prGlueInfo);

		wlanNetDestroy(prWdev);

		break;

	default:
		break;
	}

	DBGLOG(INIT, INFO, "wlanPowerOnInit::end\n");

	return i4Status;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Wlan power on deinit function. This function revert whatever
 * has been altered in the
 * power on stage to restore to the most original state.
 *
 * \param[in] void
 *
 * \return (none)
 */
/*----------------------------------------------------------------------------*/
/*
*static void wlanPowerOnDeinit(void)
*{
*
*}
*/
#endif

static void soc3_0_triggerInt(struct GLUE_INFO *prGlueInfo)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	/* trigger ccif interrupt */
	if (wf_ioremap_write(prSwWfdmaInfo->u4CcifTchnumAddr,
			     prSwWfdmaInfo->u4CcifChlNum))
		DBGLOG(HAL, ERROR, "ioremap write fail!\n");
}

static void soc3_0_getIntSta(struct GLUE_INFO *prGlueInfo,  uint32_t *pu4Sta)
{
	struct mt66xx_chip_info *prChipInfo;
	struct BUS_INFO *prBusInfo;
	struct SW_WFDMA_INFO *prSwWfdmaInfo;

	prChipInfo = prGlueInfo->prAdapter->chip_info;
	prBusInfo = prChipInfo->bus_info;
	prSwWfdmaInfo = &prBusInfo->rSwWfdmaInfo;

	wf_ioremap_read(prSwWfdmaInfo->u4CcifStartAddr, pu4Sta);
}

static bool soc3_0_get_sw_interrupt_status(struct ADAPTER *prAdapter,
	uint32_t *status)
{
	struct BUS_INFO *prBusInfo = NULL;
	uint32_t value = 0;

	ASSERT(prAdapter);
	prBusInfo = prAdapter->chip_info->bus_info;

	if (!conninfra_reg_readable())
		return false;

	HAL_MCR_WR(prAdapter,
		prBusInfo->ap2wf_remap_1,
		CONN_MCU_CONFG_HS_BASE);
	HAL_MCR_RD(prAdapter,
		(CONN_INFRA_CFG_AP2WF_BUS_ADDR + 0xc0),
		&value);
	HAL_MCR_WR(prAdapter,
		(CONN_INFRA_CFG_AP2WF_BUS_ADDR + 0xc8),
		value);

	*status = value;
	return true;
}

#endif				/* soc3_0 */
