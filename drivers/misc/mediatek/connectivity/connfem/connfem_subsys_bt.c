// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "connfem.h"

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"][BT]" fmt

/*******************************************************************************
 *			    D A T A   T Y P E S
 ******************************************************************************/


/*******************************************************************************
 *		    F U N C T I O N   D E C L A R A T I O N S
 ******************************************************************************/
static void *bt_epaelna_flags_get(void);
static struct connfem_epaelna_flag_tbl_entry *bt_epaelna_flags_tbl_get(void);

/*******************************************************************************
 *			    P U B L I C   D A T A
 ******************************************************************************/
struct connfem_epaelna_subsys_cb cfm_bt_epaelna_cb = {
	.flags_get = bt_epaelna_flags_get,
	.flags_tbl_get = bt_epaelna_flags_tbl_get
};

/*******************************************************************************
 *			   P R I V A T E   D A T A
 ******************************************************************************/
static struct connfem_epaelna_flags_bt bt_epaelna_flags;

static struct connfem_epaelna_flag_tbl_entry bt_epaelna_flags_map[] = {
	{"bypass",	&bt_epaelna_flags.bypass},
	{"epa_elna",	&bt_epaelna_flags.epa_elna},
	{"epa",		&bt_epaelna_flags.epa},
	{"elna",	&bt_epaelna_flags.elna},
	{NULL, NULL}
};

/*******************************************************************************
 *			      F U N C T I O N S
 ******************************************************************************/
static void *bt_epaelna_flags_get(void)
{
	return &bt_epaelna_flags;
}

static struct connfem_epaelna_flag_tbl_entry *bt_epaelna_flags_tbl_get(void)
{
	return bt_epaelna_flags_map;
}
