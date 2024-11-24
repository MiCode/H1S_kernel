/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __CONNFEM_CONTAINER_H__
#define __CONNFEM_CONTAINER_H__

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/


/*******************************************************************************
 *			    D A T A   T Y P E S
 ******************************************************************************/
struct cfm_container {
	unsigned int cnt;
	unsigned int entry_sz;
	char buffer[0];
};

/*******************************************************************************
 *			    P U B L I C   D A T A
 ******************************************************************************/


/*******************************************************************************
 *			      F U N C T I O N S
 ******************************************************************************/
extern struct cfm_container *cfm_container_alloc(unsigned int cnt,
						 unsigned int entry_sz);

extern void cfm_container_free(struct cfm_container *cont);

extern void *cfm_container_entry(struct cfm_container *container,
				 unsigned int idx);

extern void **cfm_container_entries(struct cfm_container *cont);

extern void cfm_container_entries_free(void **entries);

#endif /* __CONNFEM_CONTAINER_H__ */
