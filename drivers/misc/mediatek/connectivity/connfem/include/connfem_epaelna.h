/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __CONNFEM_EPAELNA_H__
#define __CONNFEM_EPAELNA_H__

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/


/*******************************************************************************
 *			    D A T A   T Y P E S
 ******************************************************************************/
struct cfm_epaelna_flags_config {
	/* Pointer to the subsys specific flags data structure
	 *	- wifi: struct connfem_epaelna_flags_wifi *
	 *	- bt  : struct connfem_epaelna_flags_bt *
	 */
	void *obj;

	/* ConnFem Container for flags names buffer management */
	struct cfm_container *names;

	/* String pointers to each of the flags name entry.
	 * Size of array can be retrieved by names->cnt
	 */
	char **name_entries;
};

struct cfm_epaelna_pin_config {
	struct connfem_epaelna_pin_info pin_info;
	struct connfem_epaelna_laa_pin_info laa_pin_info;
};

struct cfm_epaelna_config {
	bool available;

	struct connfem_epaelna_fem_info fem_info;

	struct cfm_epaelna_pin_config pin_cfg;

	struct cfm_epaelna_flags_config flags_cfg[CONNFEM_SUBSYS_NUM];
};

/*******************************************************************************
 *			    P U B L I C   D A T A
 ******************************************************************************/
extern struct connfem_epaelna_subsys_cb cfm_wf_epaelna_cb;
extern struct connfem_epaelna_subsys_cb cfm_bt_epaelna_cb;

/*******************************************************************************
 *			      F U N C T I O N S
 ******************************************************************************/
extern int cfm_epaelna_feminfo_populate(
		struct cfm_dt_epaelna_context *dt,
		struct connfem_epaelna_fem_info *result);

extern int cfm_epaelna_pincfg_populate(
		struct cfm_dt_epaelna_pctl_data_context *pctl_data,
		struct cfm_epaelna_pin_config *result);

extern int cfm_epaelna_flags_populate(
		struct cfm_dt_epaelna_flags_context *dt_flags,
		struct cfm_epaelna_flags_config *result);

extern void cfm_epaelna_config_free(struct cfm_epaelna_config *cfg,
				    bool free_all);

extern void cfm_epaelna_flags_free(struct cfm_epaelna_flags_config *flags);

extern void cfm_epaelna_config_dump(struct cfm_epaelna_config *cfg);

extern void cfm_epaelna_feminfo_dump(struct connfem_epaelna_fem_info *fem_info);

extern void cfm_epaelna_pininfo_dump(struct connfem_epaelna_pin_info *pin_info);

extern void cfm_epaelna_laainfo_dump(struct connfem_epaelna_laa_pin_info *laa);

extern void cfm_epaelna_flags_dump(enum connfem_subsys subsys,
				   struct cfm_epaelna_flags_config *flags);

extern void cfm_epaelna_flags_obj_dump(enum connfem_subsys subsys,
				       void *flags_obj);

extern void cfm_epaelna_flags_names_dump(enum connfem_subsys subsys,
					 struct cfm_container *names);

extern void cfm_epaelna_flags_name_entries_dump(enum connfem_subsys subsys,
						unsigned int cnt,
						char **name_entries);

#endif /* __CONNFEM_EPAELNA_H__ */
