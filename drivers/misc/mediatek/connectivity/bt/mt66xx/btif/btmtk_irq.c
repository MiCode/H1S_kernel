/*  SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of.h>

#include "btmtk_chip_if.h"
#include "conninfra.h"
#include "connsys_debug_utility.h"

/*******************************************************************************
*				C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*			       D A T A	 T Y P E S
********************************************************************************
*/

/*******************************************************************************
*			      P U B L I C   D A T A
********************************************************************************
*/
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
unsigned long long irq_timer[12] = {0};
#endif

/*******************************************************************************
*			     P R I V A T E   D A T A
********************************************************************************
*/
extern struct btmtk_dev *g_sbdev;
static struct bt_irq_ctrl bgf2ap_btif_wakeup_irq = {.name = "BTIF_WAKEUP_IRQ"};
static struct bt_irq_ctrl bgf2ap_sw_irq = {.name = "BGF_SW_IRQ"};
static struct bt_irq_ctrl bt_conn2ap_sw_irq = {.name = "BUS_SW_IRQ"};
static struct bt_irq_ctrl *bt_irq_table[BGF2AP_IRQ_MAX];
static struct work_struct rst_trigger_work;


/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*			       F U N C T I O N S
********************************************************************************
*/

/* bt_reset_work
 *
 *    A work thread that handles BT subsys reset request
 *
 * Arguments:
 *    [IN] work
 *
 * Return Value:
 *    N/A
 *
 */
static void bt_reset_work(struct work_struct *work)
{
	BTMTK_INFO("Trigger subsys reset");
	bt_chip_reset_flow(RESET_LEVEL_0_5, CONNDRV_TYPE_BT, "BT Subsys reset");
}

/* bt_trigger_reset
 *
 *    Trigger reset (could be subsys or whole chip reset)
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *    N/A
 *
 */
void bt_trigger_reset(void)
{
	int32_t ret = conninfra_is_bus_hang();
	BTMTK_INFO("%s: conninfra_is_bus_hang ret = %d", __func__, ret);

	if (ret > 0)
		conninfra_trigger_whole_chip_rst(CONNDRV_TYPE_BT, "bus hang");
	else if (ret == CONNINFRA_ERR_RST_ONGOING)
		BTMTK_INFO("whole chip reset is onging, skip subsys reset");
	else
		schedule_work(&rst_trigger_work);
}

/* bt_bgf2ap_irq_handler
 *
 *    Handling BGF2AP_SW_IRQ, include FW log & chip reset
 *    Please be noticed this handler is running in bt thread
 *    not interrupt thread
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *    N/A
 *
 */
void bt_bgf2ap_irq_handler(void)
{
	int32_t bgf_status = 0, count = 5;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	cif_dev->bgf2ap_ind = FALSE;

	/* wake up conn_infra off */
	if(bgfsys_check_conninfra_ready())
		return;

	/* Read IRQ status CR to identify what happens */
	bgf_status = bgfsys_get_sw_irq_status();

	/* release conn_infra force on */
	CLR_BIT(CONN_INFRA_WAKEUP_BT, BIT(0));

	if (bgf_status == RET_SWIRQ_ST_FAIL)
		return;

	if (!(bgf_status & BGF_FW_LOG_NOTIFY)) {
		BTMTK_INFO("bgf_status = 0x%08x", bgf_status);
	}else{
		BTMTK_DBG("bgf_status = 0x%08x", bgf_status);
	}

	if (bgf_status == 0xDEADFEED) {
		bt_dump_bgfsys_all();
		bt_enable_irq(BGF2AP_SW_IRQ);
	} else if (bgf_status & BGF_SUBSYS_CHIP_RESET) {
		if (cif_dev->rst_level != RESET_LEVEL_NONE)
			complete(&cif_dev->rst_comp);
		else
			schedule_work(&rst_trigger_work);
	} else if (bgf_status & BGF_FW_LOG_NOTIFY) {
		/* FW notify host to get FW log */
		connsys_log_irq_handler(CONN_DEBUG_TYPE_BT);
		while(count--){};
		bt_enable_irq(BGF2AP_SW_IRQ);
	} else if (bgf_status &  BGF_WHOLE_CHIP_RESET) {
		conninfra_trigger_whole_chip_rst(CONNDRV_TYPE_BT, "FW trigger");
	} else {
		bt_enable_irq(BGF2AP_SW_IRQ);
	}
}

/* bt_conn2ap_irq_handler
 *
 *    Handling BT_CONN2AP_SW_IRQ, include BGF bus hang. And dump SSPM TIMER
 *    Please be noticed this handler is running in bt thread
 *    not interrupt thread
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *    N/A
 *
 */
void bt_conn2ap_irq_handler(void)
{
	uint32_t value = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	cif_dev->bt_conn2ap_ind = FALSE;
	value = bt_read_cr(BT_SSPM_TIMER);
	BTMTK_INFO("%s: [SSPM] [0x%08x] = [0x%08x]", __func__, BT_SSPM_TIMER, value);
	bt_trigger_reset();
}
/* btmtk_reset_init()
 *
 *    Inint work thread for subsys chip reset
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     N/A
 *
 */
void btmtk_reset_init(void)
{
	INIT_WORK(&rst_trigger_work, bt_reset_work);
}

/* btmtk_irq_handler()
 *
 *    IRQ handler, process following types IRQ
 *    BGF2AP_BTIF_WAKEUP_IRQ   - this IRQ indicates that FW has data to transmit
 *    BGF2AP_SW_IRQ            - this indicates that fw assert / fw log
 *
 * Arguments:
 *    [IN] irq - IRQ number
 *    [IN] arg -
 *
 * Return Value:
 *    returns IRQ_HANDLED for handled IRQ, IRQ_NONE otherwise
 *
 */
static irqreturn_t btmtk_irq_handler(int irq, void * arg)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	irq_timer[0] = sched_clock();
#endif
	if (irq == bgf2ap_btif_wakeup_irq.irq_num) {
		if (cif_dev->rst_level == RESET_LEVEL_NONE) {
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
			irq_timer[1] = sched_clock();
#endif
			bt_disable_irq(BGF2AP_BTIF_WAKEUP_IRQ);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
			irq_timer[7] = sched_clock();
#endif
			cif_dev->rx_ind = TRUE;
			cif_dev->psm.sleep_flag = FALSE;
			wake_up_interruptible(&cif_dev->tx_waitq);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
			irq_timer[10] = sched_clock();
			if (irq_timer[10] - irq_timer[1] > 5000000){
				BTMTK_ERR("btif: start1[%llu] b_dis2[%llu] in_dis3[%llu] b_lock4[%llu] a_lock5[%llu] b_unlock6[%llu] a_unlock7[%llu] a_dis8[%llu] end11[%llu]", irq_timer[0], irq_timer[1], irq_timer[2], irq_timer[3], irq_timer[4], irq_timer[5], irq_timer[6], irq_timer[7], irq_timer[10]);
			}
#endif
		}
		return IRQ_HANDLED;
	} else if (irq == bgf2ap_sw_irq.irq_num) {
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		irq_timer[8] = sched_clock();
#endif
		bt_disable_irq(BGF2AP_SW_IRQ);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		irq_timer[9] = sched_clock();
#endif
		cif_dev->bgf2ap_ind = TRUE;
		wake_up_interruptible(&cif_dev->tx_waitq);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		irq_timer[11] = sched_clock();
		if (irq_timer[11] - irq_timer[8] > 5000000){
			BTMTK_ERR("sw: start1[%llu] b_dis9[%llu] in_dis3[%llu] b_lock4[%llu] a_lock5[%llu] b_unlock6[%llu] a_unlock7[%llu] a_dis10[%llu] end11[%llu]", irq_timer[0], irq_timer[8], irq_timer[2], irq_timer[3], irq_timer[4], irq_timer[5], irq_timer[6], irq_timer[9], irq_timer[11]);
		}
#endif
		return IRQ_HANDLED;
	} else if (irq == bt_conn2ap_sw_irq.irq_num) {
		bt_disable_irq(BT_CONN2AP_SW_IRQ);
		cif_dev->bt_conn2ap_ind = TRUE;
		wake_up_interruptible(&cif_dev->tx_waitq);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/* bt_request_irq()
 *
 *    Request IRQ
 *
 * Arguments:
 *    [IN] irq_type - IRQ type
 *
 * Return Value:
 *    returns 0 for success, fail otherwise
 *
 */
int32_t bt_request_irq(enum bt_irq_type irq_type)
{
	uint32_t irq_num = 0;
	int32_t ret = 0;
	unsigned long irq_flags = 0;
	struct bt_irq_ctrl *pirq = NULL;
	struct device_node *node = NULL;

	switch (irq_type) {
	case BGF2AP_BTIF_WAKEUP_IRQ:
		node = of_find_compatible_node(NULL, NULL, "mediatek,bt");
		if (node) {
			irq_num = irq_of_parse_and_map(node, 0);
			BTMTK_DBG("irqNum of BGF2AP_BTIF_WAKEUP_IRQ = %d", irq_num);
		}
		else
			BTMTK_ERR("WIFI-OF: get bt device node fail");

		irq_flags = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		pirq = &bgf2ap_btif_wakeup_irq;
		break;
	case BGF2AP_SW_IRQ:
		node = of_find_compatible_node(NULL, NULL, "mediatek,bt");
		if (node) {
			irq_num = irq_of_parse_and_map(node, 1);
			BTMTK_DBG("irqNum of BGF2AP_SW_IRQ = %d", irq_num);
		}
		else
			BTMTK_ERR("WIFI-OF: get bt device node fail");
		irq_flags = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		pirq = &bgf2ap_sw_irq;
		break;
	case BT_CONN2AP_SW_IRQ:
		node = of_find_compatible_node(NULL, NULL, "mediatek,bt");
		if (node) {
			irq_num = irq_of_parse_and_map(node, 2);
			BTMTK_DBG("irqNum of BT_CONN2AP_SW_IRQ = %d", irq_num);
		}
		else
			BTMTK_ERR("WIFI-OF: get bt device node fail");
		irq_flags = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		pirq = &bt_conn2ap_sw_irq;
		break;
	default:
		BTMTK_ERR("Invalid irq_type %d!", irq_type);
		return -EINVAL;
	}

	pirq->irq_num = irq_num;
	spin_lock_init(&pirq->lock);
	pirq->active = TRUE;
	ret = request_irq(irq_num, btmtk_irq_handler, irq_flags,
			  pirq->name, pirq);
	if (ret) {
		BTMTK_ERR("Request %s (%u) failed! ret(%d)", pirq->name, irq_num, ret);
		pirq->active = FALSE;
		return ret;
	}

	ret = enable_irq_wake(irq_num);
	if (ret) {
		BTMTK_ERR("enable_irq_wake %s (%u) failed! ret(%d)", pirq->name, irq_num, ret);
	}

	BTMTK_INFO("Request %s (%u) succeed, pirq = %p, flag = 0x%08x", pirq->name, irq_num, pirq, irq_flags);
	bt_irq_table[irq_type] = pirq;

	return 0;
}

/* bt_enable_irq()
 *
 *    Enable IRQ
 *
 * Arguments:
 *    [IN] irq_type - IRQ type
 *
 * Return Value:
 *    N/A
 *
 */
void bt_enable_irq(enum bt_irq_type irq_type)
{
	struct bt_irq_ctrl *pirq;

	if (irq_type >= BGF2AP_IRQ_MAX) {
		BTMTK_ERR("Invalid irq_type %d!", irq_type);
		return;
	}

	pirq = bt_irq_table[irq_type];
	if (pirq) {
		spin_lock_irqsave(&pirq->lock, pirq->flags);
		if (!pirq->active) {
			enable_irq(pirq->irq_num);
			pirq->active = TRUE;
		}
		spin_unlock_irqrestore(&pirq->lock, pirq->flags);
	}
}

/* bt_disable_irq()
 *
 *    Disable IRQ
 *
 * Arguments:
 *    [IN] irq_type - IRQ type
 *
 * Return Value:
 *    N/A
 *
 */
void bt_disable_irq(enum bt_irq_type irq_type)
{
	struct bt_irq_ctrl *pirq;

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	irq_timer[2] = sched_clock();
#endif
	if (irq_type >= BGF2AP_IRQ_MAX) {
		BTMTK_ERR("Invalid irq_type %d!", irq_type);
		return;
	}

	pirq = bt_irq_table[irq_type];
	if (pirq) {
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		irq_timer[3] = sched_clock();
#endif
		spin_lock_irqsave(&pirq->lock, pirq->flags);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		irq_timer[4] = sched_clock();
#endif
		if (pirq->active) {
			disable_irq_nosync(pirq->irq_num);
			pirq->active = FALSE;
		}
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		irq_timer[5] = sched_clock();
#endif
		spin_unlock_irqrestore(&pirq->lock, pirq->flags);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		irq_timer[6] = sched_clock();
#endif
	}
}


/* bt_disable_irq()
 *
 *    Release IRQ and de-register IRQ
 *
 * Arguments:
 *    [IN] irq_type - IRQ type
 *
 * Return Value:
 *    N/A
 *
 */
void bt_free_irq(enum bt_irq_type irq_type)
{
	struct bt_irq_ctrl *pirq;

	if (irq_type >= BGF2AP_IRQ_MAX) {
		BTMTK_ERR("Invalid irq_type %d!", irq_type);
		return;
	}

	pirq = bt_irq_table[irq_type];
	if (pirq) {
		disable_irq_wake(pirq->irq_num);
		free_irq(pirq->irq_num, pirq);
		pirq->active = FALSE;
		bt_irq_table[irq_type] = NULL;
	}
}
