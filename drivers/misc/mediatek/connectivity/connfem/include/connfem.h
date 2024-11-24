/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 * The header is provided for connfem kernel module internal used.
 */

#ifndef __CONNFEM_H__
#define __CONNFEM_H__

#include "connfem_api.h"
#include "connfem_container.h"
#include "connfem_dt.h"
#include "connfem_epaelna.h"

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#define CFM_IOC_MAGIC		0xCF

#define CFM_IOC_IS_AVAILABLE	_IOR(CFM_IOC_MAGIC, \
				     0x00, struct cfm_ioc_is_available)

#define CFM_IOC_EPA_FN_STAT	_IOR(CFM_IOC_MAGIC, \
				     0x01, struct cfm_ioc_epa_fn_stat)

#define CFM_IOC_EPA_FN		_IOR(CFM_IOC_MAGIC, \
				     0x02, struct cfm_ioc_epa_fn)

#define CFM_IOC_EPA_INFO	_IOR(CFM_IOC_MAGIC, \
				     0x03, struct cfm_ioc_epa_info)

#define CFM_PARAM_EPAELNA_HWID_INVALID	0xFFFFFFFF

/*******************************************************************************
 *			    D A T A   T Y P E S
 ******************************************************************************/
struct connfem_context {
	unsigned int id;
	struct platform_device *pdev;
	struct cfm_dt_context dt;
	struct cfm_epaelna_config epaelna;
};

struct cfm_ioc_is_available {
	/* IN */
	enum connfem_type fem_type;
	enum connfem_subsys subsys;

	/* OUT */
	bool is_available;
};

struct cfm_ioc_epa_fn_stat {
	/* IN */
	enum connfem_subsys subsys;

	/* OUT */
	unsigned int cnt;
	unsigned int entry_sz;
};

struct cfm_ioc_epa_fn {
	/* IN */
	enum connfem_subsys subsys;

	/* IN/OUT */
	uint64_t names;	/* struct cfm_container* */
};

struct cfm_ioc_epa_info {
	/* OUT - Aligned with connfem_epaelna_fem_info */
	unsigned int id;
	struct connfem_part part[CONNFEM_PORT_NUM];
	char part_name[CONNFEM_PORT_NUM][CONNFEM_PART_NAME_SIZE];
};

/*******************************************************************************
 *			    P U B L I C   D A T A
 ******************************************************************************/
extern struct connfem_context *connfem_ctx;

extern char *cfm_subsys_name[CONNFEM_SUBSYS_NUM];

/*******************************************************************************
 *			      F U N C T I O N S
 ******************************************************************************/
extern bool connfem_is_internal(void);
extern void cfm_context_free(struct connfem_context *cfm);
extern unsigned int cfm_param_epaelna_hwid(void);

#endif /* __CONNFEM_H__ */
