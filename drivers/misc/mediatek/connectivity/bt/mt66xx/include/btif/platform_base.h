/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _PLATFORM_BASE_H
#define _PLATFORM_BASE_H

#include "conninfra.h"

#define BT_CR_DUMP_BUF_SIZE	(1024)
#define FW_NAME_LEN		(64)
#define PATCH_FILE_NUM		2

static uint8_t g_dump_cr_buffer[BT_CR_DUMP_BUF_SIZE];
#if (CUSTOMER_FW_UPDATE == 1)
extern uint8_t g_fwp_names[PATCH_FILE_NUM][2[FW_NAME_LEN];
#else
extern uint8_t g_fwp_names[PATCH_FILE_NUM][1][FW_NAME_LEN];
#endif

#define FLAVOR_NONE	'0'

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/
#ifndef BIT
#define BIT(n)                          (1UL << (n))
#endif /* BIT */

#ifndef BITS
/* bits range: for example BITS(16,23) = 0xFF0000
 *   ==>  (BIT(m)-1)   = 0x0000FFFF     ~(BIT(m)-1)   => 0xFFFF0000
 *   ==>  (BIT(n+1)-1) = 0x00FFFFFF
 */
#define BITS(m, n)                      (~(BIT(m)-1) & ((BIT(n) - 1) | BIT(n)))
#endif /* BIT */

/*
 * This macro returns the byte offset of a named field in a known structure
 * type.
 * _type - structure name,
 * _field - field name of the structure
 */
#ifndef OFFSET_OF
#define OFFSET_OF(_type, _field)    ((unsigned long)&(((_type *)0)->_field))
#endif /* OFFSET_OF */

/*
 * This macro returns the base address of an instance of a structure
 * given the type of the structure and the address of a field within the
 * containing structure.
 * _addr_of_field - address of a field within the structure,
 * _type - structure name,
 * _field - field name of the structure
 */
#ifndef ENTRY_OF
#define ENTRY_OF(_addr_of_field, _type, _field) \
	((_type *)((unsigned char *)(_addr_of_field) - (unsigned char *)OFFSET_OF(_type, _field)))
#endif /* ENTRY_OF */

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
struct consys_reg_base_addr {
	unsigned long vir_addr;
	unsigned long phy_addr;
	unsigned long long size;
};

enum bt_base_addr_index {
	CONN_INFRA_RGU_BASE_INDEX = 0,
	CONN_INFRA_CFG_BASE_INDEX = 1,
	CONN_INFRA_SYS_BASE_INDEX = 2,
	SPM_BASE_INDEX		  = 3,

	BGFSYS_BASE_INDEX	  = 4,
	BGFSYS_INFO_BASE_INDEX	  = 5,

	INFRACFG_AO_BASE_INDEX    = 6,
#if 0
	CONN_INFRA_CFG_CCIF_BASE  = 7,
	BGF2MD_BASE_INDEX	  = 8,
#endif

	CONSYS_BASE_ADDR_MAX
};

struct bt_base_addr {
	struct consys_reg_base_addr reg_base_addr[CONSYS_BASE_ADDR_MAX];
};

extern struct bt_base_addr bt_reg;

#define CON_REG_INFRA_RGU_ADDR		bt_reg.reg_base_addr[CONN_INFRA_RGU_BASE_INDEX].vir_addr /* 0x18000000 0x1000 */
#define CON_REG_INFRA_CFG_ADDR		bt_reg.reg_base_addr[CONN_INFRA_CFG_BASE_INDEX].vir_addr /* 0x18001000 0x1000 */
#define CON_REG_INFRA_SYS_ADDR		bt_reg.reg_base_addr[CONN_INFRA_SYS_BASE_INDEX].vir_addr /* 0x18050000 0x1000 */
#define CON_REG_SPM_BASE_ADDR 		bt_reg.reg_base_addr[SPM_BASE_INDEX].vir_addr		 /* 0x18060000 0x1000 */
#define BGF_REG_BASE_ADDR		bt_reg.reg_base_addr[BGFSYS_BASE_INDEX].vir_addr	 /* 0x18800000 0x1000 */
#define BGF_REG_INFO_BASE_ADDR		bt_reg.reg_base_addr[BGFSYS_INFO_BASE_INDEX].vir_addr    /* 0x18812000 0x1000 */
#define CON_REG_INFRACFG_AO_ADDR	bt_reg.reg_base_addr[INFRACFG_AO_BASE_INDEX].vir_addr    /* 0x10001000 0x1000 */
#if 0
#define CON_REG_INFRA_CCIF_ADDR		bt_reg.reg_base_addr[CONN_INFRA_CFG_CCIF_BASE].vir_addr
#define BGF2MD_BASE_ADDR		bt_reg.reg_base_addr[BGF2MD_BASE_INDEX].vir_addr
#endif


#define SET_BIT(addr, bit) \
			(*((volatile uint32_t *)(addr))) |= ((uint32_t)bit)
#define CLR_BIT(addr, bit) \
			(*((volatile uint32_t *)(addr))) &= ~((uint32_t)bit)
#define REG_READL(addr) \
			readl((volatile uint32_t *)(addr))
#define REG_WRITEL(addr, val) \
			writel(val, (volatile uint32_t *)(addr))

#define CAN_DUMP_HOST_CSR(reason) \
	(reason != CONNINFRA_INFRA_BUS_HANG && \
	 reason != CONNINFRA_AP2CONN_RX_SLP_PROT_ERR && \
	 reason != CONNINFRA_AP2CONN_TX_SLP_PROT_ERR && \
	 reason != CONNINFRA_AP2CONN_CLK_ERR)

#define POS_POLLING_RTY_LMT		100
#define IDLE_LOOP_RTY_LMT		100
#define CAL_READY_BIT_PATTERN		0x5AD02EA5

/*********************************************************************
*
* Global variable
*
**********************************************************************
*/

static uint32_t g_sw_irq_status = 0;

/*********************************************************************
*
* Utility APIs
*
**********************************************************************
*/

static uint32_t inline bt_read_cr(uint32_t addr)
{
	uint32_t value = 0;
	uint8_t *base = ioremap(addr, 0x10);

	if (base == NULL) {
		BTMTK_ERR("%s: remapping 0x%08x fail", addr);
	} else {
		value = REG_READL(base);
		iounmap(base);
	}
	return value;
}

static void inline bt_write_cr(uint32_t addr, uint32_t value, bool is_set_bit)
{
	uint32_t *base = ioremap(addr, 0x10);

	if (base == NULL) {
		BTMTK_ERR("%s: remapping 0x%08x fail", addr);
	} else {
		if (is_set_bit)
			*base |= value; // set bit to CR
		else
			*base = value; // directly write value to CR
		iounmap(base);
	}
}

static void inline bt_dump_memory8(uint8_t *buf, uint32_t len)
{
	uint32_t i = 0;
	uint8_t *pos = NULL, *end = NULL;
	int32_t ret = 0;

	memset(g_dump_cr_buffer, 0, BT_CR_DUMP_BUF_SIZE);
	pos = &g_dump_cr_buffer[0];
	end = pos + BT_CR_DUMP_BUF_SIZE - 1;

	BTMTK_INFO("%s: length = (%d)", __func__, len);
	for (i = 0; i <= len; i++) {
		ret = snprintf(pos, (end - pos + 1), "%02x ", buf[i]);
		if (ret < 0 || ret >= (end - pos + 1))
			break;
		pos += ret;

		if ((i & 0xF) == 0xF || i == len - 1) {
			BTMTK_INFO("%s", g_dump_cr_buffer);
			memset(g_dump_cr_buffer, 0, BT_CR_DUMP_BUF_SIZE);
			pos = &g_dump_cr_buffer[0];
			end = pos + BT_CR_DUMP_BUF_SIZE - 1;
		}
	}

}

static inline u_int8_t fwp_has_flavor_bin(uint8_t *flavor)
{
	#define TARGET_KEY "flavor_bin"
	u_int8_t ret = FALSE;
	const char *str;
	struct device_node *node = NULL;
	node = of_find_compatible_node(NULL, NULL, "mediatek,bt");
	if (node) {
		if (of_property_read_string(node, TARGET_KEY, &str)) {
			BTMTK_INFO("%s: get %s: fail", __func__, TARGET_KEY);
		} else {
			*flavor = *str;
			BTMTK_INFO("%s: get %s: %c", __func__, TARGET_KEY, *flavor);
			ret = TRUE;
		}
	} else
		BTMTK_INFO("%s: get dts[mediatek,bt] fail!", __func__);
	return ret;
}

static inline void compose_fw_name(u_int8_t has_flavor, uint8_t flavor,
					   const uint8_t *bin_mcu_name,
					   const uint8_t *bin_bt_name)
{
	if (has_flavor) {
		if (snprintf(g_fwp_names[0][0], FW_NAME_LEN, "%s%c_1_hdr.bin", bin_mcu_name, flavor) < 0)
			BTMTK_ERR("%s: has_flavor[0][0]", __func__);

		if (snprintf(g_fwp_names[1][0], FW_NAME_LEN, "%s%c_1_hdr.bin", bin_bt_name, flavor) < 0)
			BTMTK_ERR("%s: has_flavor[1][0]", __func__);
	} else	{
		if (snprintf(g_fwp_names[0][0], FW_NAME_LEN, "%s_1_hdr.bin", bin_mcu_name) < 0)
			BTMTK_ERR("%s: no_flavor[0][0]", __func__);
		if (snprintf(g_fwp_names[1][0], FW_NAME_LEN, "%s_1_hdr.bin", bin_bt_name) < 0)
			BTMTK_ERR("%s: no_flavor[1][0]", __func__);
	}

#if (CUSTOMER_FW_UPDATE == 1)
	if (has_flavor) {
		if (snprintf(g_fwp_names[0][1], FW_NAME_LEN, "%s%c_1_hdr-u.bin", bin_mcu_name, flavor) < 0)
			BTMTK_ERR("%s: has_flavor[0][1]", __func__);
		if (snprintf(g_fwp_names[1][1], FW_NAME_LEN, "%s%c_1_hdr-u.bin", bin_bt_name, flavor) < 0)
			BTMTK_ERR("%s: has_flavor[1][1]", __func__);
	} else	{
		if (snprintf(g_fwp_names[0][1], FW_NAME_LEN, "%s_1_hdr.bin", bin_mcu_name) < 0)
			BTMTK_ERR("%s: no_flavor[0][1]", __func__);
		if (snprintf(g_fwp_names[1][1], FW_NAME_LEN, "%s_1_hdr.bin", bin_bt_name) < 0)
			BTMTK_ERR("%s: no_flavor[1][1]", __func__);
	}
#endif
}


#endif
