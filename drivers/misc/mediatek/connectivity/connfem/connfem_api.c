// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include "connfem.h"

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"][API]" fmt

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
bool connfem_is_available(enum connfem_type fem_type)
{
	if (!connfem_ctx) {
		pr_info("[WARN] %s, No ConnFem context", __func__);
		return false;
	}

	switch (fem_type) {
	case CONNFEM_TYPE_EPAELNA:
		pr_info("Available:%d", connfem_ctx->epaelna.available);
		cfm_epaelna_feminfo_dump(&connfem_ctx->epaelna.fem_info);

		return connfem_ctx->epaelna.available;

	default:
		pr_info("[WARN] %s, Unknown fem_type: %d", __func__, fem_type);
		break;
	}

	return false;
}
EXPORT_SYMBOL(connfem_is_available);

int connfem_epaelna_get_fem_info(struct connfem_epaelna_fem_info *fem_info)
{
	if (!fem_info) {
		pr_info("[WARN] %s, input parameter is NULL", __func__);
		return -EINVAL;
	}

	memset(fem_info, 0, sizeof(*fem_info));

	if (!connfem_ctx) {
		pr_info("[WARN] %s, No ConnFem context", __func__);
		return -EOPNOTSUPP;
	}

	memcpy(fem_info, &connfem_ctx->epaelna.fem_info, sizeof(*fem_info));

	pr_info("GetFemInfo");
	cfm_epaelna_feminfo_dump(fem_info);

	return 0;
}
EXPORT_SYMBOL(connfem_epaelna_get_fem_info);

int connfem_epaelna_get_pin_info(struct connfem_epaelna_pin_info *pin_info)
{
	if (!pin_info) {
		pr_info("[WARN] %s, input parameter is NULL", __func__);
		return -EINVAL;
	}

	memset(pin_info, 0, sizeof(*pin_info));

	if (!connfem_ctx) {
		pr_info("[WARN] %s, No ConnFem context", __func__);
		return -EOPNOTSUPP;
	}

	memcpy(pin_info, &connfem_ctx->epaelna.pin_cfg.pin_info,
	       sizeof(*pin_info));

	pr_info("GetPinInfo");
	cfm_epaelna_pininfo_dump(pin_info);

	return 0;
}
EXPORT_SYMBOL(connfem_epaelna_get_pin_info);

int connfem_epaelna_laa_get_pin_info(
		struct connfem_epaelna_laa_pin_info *laa_pin_info)
{
	if (!laa_pin_info) {
		pr_info("[WARN] %s, input parameter is NULL", __func__);
		return -EINVAL;
	}

	memset(laa_pin_info, 0, sizeof(*laa_pin_info));

	if (!connfem_ctx) {
		pr_info("[WARN] %s, No ConnFem context", __func__);
		return -EOPNOTSUPP;
	}

	memcpy(laa_pin_info, &connfem_ctx->epaelna.pin_cfg.laa_pin_info,
	       sizeof(*laa_pin_info));

	/* Assign chip id to connfem_epaelna_laa_pin_info
	 * due to FW (COEX LAA 4x4 module) need this information.
	 */
	laa_pin_info->chip_id = connfem_ctx->id;

	pr_info("GetLaaPinInfo");
	cfm_epaelna_laainfo_dump(laa_pin_info);

	return 0;
}
EXPORT_SYMBOL(connfem_epaelna_laa_get_pin_info);

int connfem_epaelna_get_flags(enum connfem_subsys subsys, void *flags)
{
	if (subsys <= CONNFEM_SUBSYS_NONE || subsys >= CONNFEM_SUBSYS_NUM) {
		pr_info("[WARN] %s, invalid subsys %d",
			__func__, subsys);
		return -EINVAL;
	}

	if (!flags) {
		pr_info("[WARN] %s, input parameter is NULL", __func__);
		return -EINVAL;
	}

	if (!connfem_ctx) {
		pr_info("[WARN] %s, No ConnFem context", __func__);
		return -EOPNOTSUPP;
	}

	if (!connfem_ctx->epaelna.flags_cfg[subsys].obj) {
		pr_info("[WARN] %s, subsys %d '%s' flags is NULL",
			__func__, subsys, cfm_subsys_name[subsys]);
		return -EINVAL;
	}

	switch (subsys) {
	case CONNFEM_SUBSYS_WIFI:
		memcpy(flags, connfem_ctx->epaelna.flags_cfg[subsys].obj,
		       sizeof(struct connfem_epaelna_flags_wifi));
		break;

	case CONNFEM_SUBSYS_BT:
		memcpy(flags, connfem_ctx->epaelna.flags_cfg[subsys].obj,
		       sizeof(struct connfem_epaelna_flags_bt));
		break;

	default:
		pr_info("%s, unknown subsys: %d", __func__, subsys);
		return -EINVAL;
	}

	pr_info("GetFlags");
	cfm_epaelna_flags_obj_dump(subsys, flags);

	return 0;
}
EXPORT_SYMBOL(connfem_epaelna_get_flags);

int connfem_epaelna_get_flags_names(enum connfem_subsys subsys,
			unsigned int *num_flags, char ***names)
{
	if (subsys <= CONNFEM_SUBSYS_NONE || subsys >= CONNFEM_SUBSYS_NUM) {
		pr_info("[WARN] %s, invalid subsys %d",
			__func__, subsys);
		return -EINVAL;
	}

	if (!num_flags || !names) {
		pr_info("[WARN] %s, input parameter is NULL, (%p, %p)",
			__func__, num_flags, names);
		return -EINVAL;
	}

	*num_flags = 0;
	*names = NULL;

	if (!connfem_ctx) {
		pr_info("[WARN] %s, No ConnFem context", __func__);
		return -EOPNOTSUPP;
	}

	if (!connfem_ctx->epaelna.flags_cfg[subsys].names ||
	    !connfem_ctx->epaelna.flags_cfg[subsys].name_entries) {
		pr_info("[WARN] %s, subsys %d '%s' names is NULL",
			__func__, subsys, cfm_subsys_name[subsys]);
		return -EINVAL;
	}

	*num_flags = connfem_ctx->epaelna.flags_cfg[subsys].names->cnt;
	*names = connfem_ctx->epaelna.flags_cfg[subsys].name_entries;

	pr_info("GetFlagsNames");
	cfm_epaelna_flags_name_entries_dump(subsys, *num_flags, *names);

	return 0;
}
EXPORT_SYMBOL(connfem_epaelna_get_flags_names);
