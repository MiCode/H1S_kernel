// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include "connfem.h"

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"][TEST]" fmt


/*******************************************************************************
 *			    D A T A   T Y P E S
 ******************************************************************************/


/*******************************************************************************
 *		    F U N C T I O N   D E C L A R A T I O N S
 ******************************************************************************/


/*******************************************************************************
 *			    P U B L I C   D A T A
 ******************************************************************************/


/*******************************************************************************
 *			   P R I V A T E   D A T A
 ******************************************************************************/


/*******************************************************************************
 *			      F U N C T I O N S
 ******************************************************************************/
void connfem_test(void)
{
	struct connfem_epaelna_fem_info fem_info;
	struct connfem_epaelna_pin_info pin_info;
	struct connfem_epaelna_laa_pin_info laa_pin_info;
	struct connfem_epaelna_flags_wifi wf_flags;
	struct connfem_epaelna_flags_bt bt_flags;
	void *flags = NULL;
	unsigned int num_flags = 0;
	char **flags_names = NULL;
	int err = 0;

	memset(&fem_info, 0, sizeof(fem_info));
	memset(&pin_info, 0, sizeof(pin_info));
	memset(&laa_pin_info, 0, sizeof(laa_pin_info));
	memset(&wf_flags, 0, sizeof(wf_flags));
	memset(&bt_flags, 0, sizeof(bt_flags));

	pr_info("%s++", __func__);

	pr_info("connfem_ctx %p", connfem_ctx);

	if (connfem_ctx) {
		pr_info("connfem_ctx->epaelna");
		cfm_epaelna_config_dump(&connfem_ctx->epaelna);
	}

	pr_info("connfem_is_available(NONE) %d",
		connfem_is_available(CONNFEM_TYPE_NONE));

	pr_info("connfem_is_available(EPAELNA) %d",
		connfem_is_available(CONNFEM_TYPE_EPAELNA));

	pr_info("connfem_is_available(NUM) %d",
		connfem_is_available(CONNFEM_TYPE_NUM));


	pr_info("connfem_epaelna_get_fem_info >>>");
	err = connfem_epaelna_get_fem_info(&fem_info);
	pr_info("<<< err:%d", err);
	cfm_epaelna_feminfo_dump(&fem_info);


	pr_info("connfem_epaelna_get_pin_info >>>");
	err = connfem_epaelna_get_pin_info(&pin_info);
	pr_info("<<< err:%d", err);
	cfm_epaelna_pininfo_dump(&pin_info);


	pr_info("connfem_epaelna_laa_get_pin_info >>>");
	err = connfem_epaelna_laa_get_pin_info(&laa_pin_info);
	pr_info("<<< err:%d", err);
	cfm_epaelna_laainfo_dump(&laa_pin_info);


	pr_info("connfem_epaelna_get_flags(NONE) >>>");
	err = connfem_epaelna_get_flags(CONNFEM_SUBSYS_NONE, flags);
	pr_info("<<< err:%d", err);

	pr_info("connfem_epaelna_get_flags(WIFI) >>>");
	err = connfem_epaelna_get_flags(CONNFEM_SUBSYS_WIFI, &wf_flags);
	pr_info("<<< err:%d", err);
	cfm_epaelna_flags_obj_dump(CONNFEM_SUBSYS_WIFI, &wf_flags);

	pr_info("connfem_epaelna_get_flags(BT) >>>");
	err = connfem_epaelna_get_flags(CONNFEM_SUBSYS_BT, &bt_flags);
	pr_info("<<< err:%d", err);
	cfm_epaelna_flags_obj_dump(CONNFEM_SUBSYS_BT, &bt_flags);

	pr_info("connfem_epaelna_get_flags(NUM) >>>");
	err = connfem_epaelna_get_flags(CONNFEM_SUBSYS_NUM, flags);
	pr_info("<<< err:%d", err);


	num_flags = 0;
	flags_names = NULL;
	pr_info("connfem_epaelna_get_flags_names(NONE) >>>");
	err = connfem_epaelna_get_flags_names(CONNFEM_SUBSYS_NONE,
					      &num_flags, &flags_names);
	pr_info("<<< err:%d, num:%d, names:%p", err, num_flags, flags_names);

	num_flags = 0;
	flags_names = NULL;
	pr_info("connfem_epaelna_get_flags_names(WIFI) >>>");
	err = connfem_epaelna_get_flags_names(CONNFEM_SUBSYS_WIFI,
					      &num_flags, &flags_names);
	pr_info("<<< err:%d, num:%d, names:%p", err, num_flags, flags_names);
	cfm_epaelna_flags_name_entries_dump(CONNFEM_SUBSYS_WIFI,
					    num_flags, flags_names);

	num_flags = 0;
	flags_names = NULL;
	pr_info("connfem_epaelna_get_flags_names(BT) >>>");
	err = connfem_epaelna_get_flags_names(CONNFEM_SUBSYS_BT,
					      &num_flags, &flags_names);
	pr_info("<<< err:%d, num:%d, names:%p", err, num_flags, flags_names);
	cfm_epaelna_flags_name_entries_dump(CONNFEM_SUBSYS_BT,
					    num_flags, flags_names);

	num_flags = 0;
	flags_names = NULL;
	pr_info("connfem_epaelna_get_flags_names(NUM) >>>");
	err = connfem_epaelna_get_flags_names(CONNFEM_SUBSYS_NUM,
					      &num_flags, &flags_names);
	pr_info("<<< err:%d, num:%d, names:%p", err, num_flags, flags_names);

	pr_info("%s--", __func__);
}
