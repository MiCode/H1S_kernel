// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "connfem_container.h"

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/


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
struct cfm_container *cfm_container_alloc(unsigned int cnt,
					  unsigned int entry_sz)
{
	struct cfm_container *cont;
	unsigned int sz;

	sz = sizeof(struct cfm_container) + (cnt * entry_sz);
	cont = kmalloc(sz, GFP_KERNEL);
	if (!cont) {
		pr_info("[WARN] Failed to alloc container %u + %ux%u = %u",
			(unsigned int)sizeof(struct cfm_container),
			cnt, entry_sz, sz);
		return NULL;
	}
	memset(cont, 0, sz);

	cont->cnt = cnt;
	cont->entry_sz = entry_sz;

	return cont;
};

void cfm_container_free(struct cfm_container *cont)
{
	kfree(cont);
}

void *cfm_container_entry(struct cfm_container *cont, unsigned int idx)
{
	if (cont) {
		if (idx < cont->cnt)
			return &cont->buffer[idx * cont->entry_sz];

		pr_info("[WARN] ConnFem container index out-of-bound: %u >= %u",
			idx, cont->cnt);
	}
	return NULL;
}

void **cfm_container_entries(struct cfm_container *cont)
{
	unsigned int i;
	void **entries;

	if (!cont || cont->cnt == 0) {
		pr_info("Container is empty");
		return NULL;
	}

	entries = kcalloc(cont->cnt, sizeof(*entries), GFP_KERNEL);
	if (!entries) {
		pr_info("[WARN] Failed to alloc %u entries of %u",
			cont->cnt, (unsigned int)sizeof(*entries));
		return NULL;
	}

	for (i = 0; i < cont->cnt; i++)
		entries[i] = cfm_container_entry(cont, i);

	return entries;
}

void cfm_container_entries_free(void **entries)
{
	kfree(entries);
}
