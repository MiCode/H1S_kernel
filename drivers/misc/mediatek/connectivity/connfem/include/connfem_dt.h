/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __CONNFEM_DT_H__
#define __CONNFEM_DT_H__

/*******************************************************************************
 *				M A C R O S
 ******************************************************************************/
/* ConnFem Device Tree Node/Parameter Names */
#define CFM_DT_NODE_EPAELNA		"epa_elna"
#define CFM_DT_NODE_EPAELNA_MTK		"epa_elna_mtk"
#define CFM_DT_NODE_WIFI		"wifi"
#define CFM_DT_NODE_BT			"bt"
#define CFM_DT_PROP_PARTS		"parts"
#define CFM_DT_NODE_HWID		"hwid"
#define CFM_DT_PROP_GPIO		"gpio"
#define CFM_DT_NODE_PMIC		"pmic"
#define CFM_DT_PROP_CHANNEL_NAME	"channel_name"
#define CFM_DT_PROP_IO_CHANNEL_NAMES	"io-channel-names"
#define CFM_DT_PROP_RANGE_PREFIX	"range-"
#define CFM_DT_PROP_FLAGS_PREFIX	"flags-"
#define CFM_DT_PROP_PINCTRL_PREFIX	"pinctrl-"
#define CFM_DT_PROP_PINMUX		"pinmux"
#define CFM_DT_PROP_MAPPING		"mapping"
#define CFM_DT_PROP_LAA_PINMUX		"laa-pinmux"

#define CFM_DT_PARTS_NOFEM		"nofem"

/* format: 'GBandPart_ABandPart' */
#define CFM_DT_PCTL_STATE_NAME_SIZE	\
			((CONNFEM_PART_NAME_SIZE * CONNFEM_PORT_NUM) + \
			 (CONNFEM_PORT_NUM - 1) /* Number of _*/ \
			 + 1) /* Zero-terminate */

#define CFM_DT_MAPPING_SIZE		3
#define CFM_DT_MAPPING_POLARITY_INDEX	2

#define CFM_DT_LAA_PINMUX_SIZE		2
#define CFM_DT_LAA_PINMUX_WF_INDEX	0
#define CFM_DT_LAA_PINMUX_MD_INDEX	1

#define CFM_DT_GET_PIN_NO(x) ((x) >> 8)
#define CFM_DT_GET_PIN_FUNC(x) ((x) & 0xF)


/*******************************************************************************
 *			    D A T A   T Y P E S
 ******************************************************************************/
struct cfm_dt_epaelna_pctl_state_context {
	char name[CFM_DT_PCTL_STATE_NAME_SIZE];
	unsigned int index;

	/* Max 10 digits (2^32) */
	char prop_name[sizeof(CFM_DT_PROP_PINCTRL_PREFIX) + 10 + 1];

	/* Pointers to nodes described in 'pinctrl-n' property */
	unsigned int np_cnt;
	struct device_node **np;
};

struct cfm_dt_epaelna_pctl_data_context {
	/* Pointer to the pinctrl data node that contains the 'pinmux' prop */
	struct device_node *np;

	/* Number of CFM_DT_MAPPING_SIZE entries */
	unsigned int pin_cnt;

	/* Number of CFM_DT_LAA_PINMUX_SIZE entries */
	unsigned int laa_cnt;
};

struct cfm_dt_epaelna_pctl_context {
	struct cfm_dt_epaelna_pctl_state_context state;
};

struct cfm_dt_epaelna_flags_context {
	/* Pointer to the 'flags-n' node in each subsys */
	struct device_node *np[CONNFEM_SUBSYS_NUM];

	/* The composed 'flags-n' node name.
	 * n has max 10 digits (2^32).
	 */
	char node_name[sizeof(CFM_DT_PROP_FLAGS_PREFIX) + 10 + 1];
};

struct cfm_dt_epaelna_context {
	unsigned int hwid;
	struct device_node *parts_np[CONNFEM_PORT_NUM]; /* selected 'parts' */

	struct cfm_dt_epaelna_pctl_context pctl;

	struct cfm_dt_epaelna_flags_context flags;
};

struct cfm_dt_context {
	struct cfm_dt_epaelna_context epaelna;
};

struct connfem_context;

/*******************************************************************************
 *			    P U B L I C   D A T A
 ******************************************************************************/


/*******************************************************************************
 *			      F U N C T I O N S
 ******************************************************************************/
extern int cfm_dt_parse(struct connfem_context *cfm);
extern void cfm_dt_free(struct cfm_dt_context *dt);

#endif /* __CONNFEM_DT_H__ */
