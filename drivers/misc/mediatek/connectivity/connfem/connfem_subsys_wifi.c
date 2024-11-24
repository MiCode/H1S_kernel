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
#define pr_fmt(fmt) "["KBUILD_MODNAME"][WF]" fmt

/*******************************************************************************
 *			    D A T A   T Y P E S
 ******************************************************************************/


/*******************************************************************************
 *		    F U N C T I O N   D E C L A R A T I O N S
 ******************************************************************************/
static void *wf_epaelna_flags_get(void);
static struct connfem_epaelna_flag_tbl_entry *wf_epaelna_flags_tbl_get(void);

/*******************************************************************************
 *			    P U B L I C   D A T A
 ******************************************************************************/
struct connfem_epaelna_subsys_cb cfm_wf_epaelna_cb = {
	.flags_get = wf_epaelna_flags_get,
	.flags_tbl_get = wf_epaelna_flags_tbl_get
};

/*******************************************************************************
 *			   P R I V A T E   D A T A
 ******************************************************************************/
static struct connfem_epaelna_flags_wifi wf_epaelna_flags;

static struct connfem_epaelna_flag_tbl_entry wf_epaelna_flags_map[] = {
	{"open-loop",	&wf_epaelna_flags.open_loop},
	{"laa",		&wf_epaelna_flags.laa},
	{NULL, NULL}
};

/*******************************************************************************
 *			      F U N C T I O N S
 ******************************************************************************/
static void *wf_epaelna_flags_get(void)
{
	return &wf_epaelna_flags;
}

static struct connfem_epaelna_flag_tbl_entry *wf_epaelna_flags_tbl_get(void)
{
	return wf_epaelna_flags_map;
}
