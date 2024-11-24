/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#include <linux/kthread.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/timer.h>
#include <linux/suspend.h>
#include <linux/module.h>

#include "btmtk_define.h"
#include "btmtk_chip_if.h"
#include "btmtk_dbg_tp_evt_if.h"
#include "conninfra.h"
#include "mtk_btif_exp.h"
#include "connectivity_build_in_adapter.h"
#include "connsys_debug_utility.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
#include "mtk_disp_notify.h"
#endif

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/
#define LOG TRUE
#define MTKBT_BTIF_NAME			"mtkbt_btif"
#define BTIF_OWNER_NAME 		"CONSYS_BT"

#define LPCR_POLLING_RTY_LMT		4096 /* 16*n */
#define LPCR_MASS_DUMP_LMT		4000
#define BTIF_IDLE_WAIT_TIME		32 /* ms */

#define MAX_RESET_COUNT			(3)
/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/
uint8_t *p_conn_infra_base_addr;
uint8_t *p_bgfsys_base_addr;
extern bool g_bt_trace_pt;
extern struct btmtk_dev *g_sbdev;

static unsigned long g_btif_id; /* The user identifier to operate btif */
struct task_struct *g_btif_rxd_thread;
struct btmtk_btif_dev g_btif_dev;
struct bt_dbg_st g_bt_dbg_st;
static struct work_struct internal_trx_work;

static struct platform_driver mtkbt_btif_driver = {
	.driver = {
		.name = MTKBT_BTIF_NAME,
		.owner = THIS_MODULE,
	},
	.probe = NULL,
	.remove = NULL,
};

#if (USE_DEVICE_NODE == 0)
static struct platform_device *mtkbt_btif_device;
#endif

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
/* bt_reg_init
 *
 *    Remapping control registers
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int32_t bt_reg_init(void)
{
	int32_t ret = -1;
	struct device_node *node = NULL;
	struct consys_reg_base_addr *base_addr = NULL;
	struct resource res;
	int32_t flag, i = 0;

	node = of_find_compatible_node(NULL, NULL, "mediatek,bt");
	if (node) {
		for (i = 0; i < CONSYS_BASE_ADDR_MAX; i++) {
			base_addr = &bt_reg.reg_base_addr[i];

			ret = of_address_to_resource(node, i, &res);
			if (ret) {
				BTMTK_ERR("Get Reg Index(%d) failed", i);
				return -1;
			}

			base_addr->phy_addr = res.start;
			base_addr->vir_addr =
				(unsigned long) of_iomap(node, i);
			of_get_address(node, i, &(base_addr->size), &flag);

			BTMTK_DBG("Get Index(%d) phy(0x%zx) baseAddr=(0x%zx) size=(0x%zx)",
				i, base_addr->phy_addr, base_addr->vir_addr,
				base_addr->size);
		}

	} else {
		BTMTK_ERR("[%s] can't find CONSYS compatible node\n", __func__);
		return -1;
	}
	return 0;
}

/* bt_reg_deinit
 *
 *    Release the memory of remapping address
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *    N/A
 *
 */
int32_t bt_reg_deinit(void)
{
	int i = 0;

	for (i = 0; i < CONSYS_BASE_ADDR_MAX; i++) {
		if (bt_reg.reg_base_addr[i].vir_addr) {
			iounmap((void __iomem*)bt_reg.reg_base_addr[i].vir_addr);
			bt_reg.reg_base_addr[i].vir_addr = 0;
		}
	}
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
int btmtk_disp_notify_cb(struct notifier_block *nb, unsigned long value, void *v)
{
	int *data = (int *)v;
	int32_t new_state = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	/*
		MTK_DISP_EARLY_EVENT_BLANK: This event will happen before lcm suspend/resume
		MTK_DISP_EVENT_BLANK: This event will happen after lcm suspend/resume
		MTK_DISP_BLANK_UNBLANK: which means display resume (power on), 0x0
		MTK_DISP_BLANK_POWERDOWN: which mean display suspend (power off), 0x1
	*/

	BTMTK_INFO("%s: value[%ld], data[%d]", __func__, value, *data);
	if (value == MTK_DISP_EARLY_EVENT_BLANK) {
		switch (*data) {
			case MTK_DISP_BLANK_UNBLANK:
				new_state = WMT_PARA_SCREEN_ON;
				break;
			case MTK_DISP_BLANK_POWERDOWN:
				new_state = WMT_PARA_SCREEN_OFF;
				break;
			default:
				goto end;
		}

		if(cif_dev->bt_state == FUNC_ON) {
			BTMTK_INFO("%s: blank state [%ld]->[%ld], and send cmd", __func__, cif_dev->blank_state, new_state);
			cif_dev->blank_state = new_state;
			btmtk_intcmd_wmt_blank_status(g_sbdev->hdev, cif_dev->blank_state);
		} else {
			BTMTK_INFO("%s: blank state [%ld]->[%ld]", __func__, cif_dev->blank_state, new_state);
			cif_dev->blank_state = new_state;
		}
	}
end:
	BTMTK_DBG("%s: end", __func__);
	return 0;
}

struct notifier_block btmtk_disp_notifier = {
	.notifier_call = btmtk_disp_notify_cb,
};

#else
static struct notifier_block bt_fb_notifier;
static int btmtk_fb_notifier_callback(struct notifier_block
				*self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int32_t blank = *(int32_t *)evdata->data;
	int32_t new_state = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if ((event != FB_EVENT_BLANK))
		return 0;

	switch (blank) {
		case FB_BLANK_UNBLANK:
			new_state = WMT_PARA_SCREEN_ON;
			break;
		case FB_BLANK_POWERDOWN:
			new_state = WMT_PARA_SCREEN_OFF;
			break;
		default:
			goto end;
	}

	if(cif_dev->bt_state == FUNC_ON) {
		BTMTK_INFO("%s: blank state [%ld]->[%ld], and send cmd", __func__, cif_dev->blank_state, new_state);
		cif_dev->blank_state = new_state;
		btmtk_intcmd_wmt_blank_status(g_sbdev->hdev, cif_dev->blank_state);
	} else {
		BTMTK_INFO("%s: blank state [%ld]->[%ld]", __func__, cif_dev->blank_state, new_state);
		cif_dev->blank_state = new_state;
	}
end:
	return 0;
}

static int btmtk_fb_notify_register(void)
{
	int32_t ret;

	bt_fb_notifier.notifier_call = btmtk_fb_notifier_callback;

	ret = fb_register_client(&bt_fb_notifier);
	if (ret)
		BTMTK_ERR("Register wlan_fb_notifier failed:%d\n", ret);
	else
		BTMTK_DBG("Register wlan_fb_notifier succeed\n");

	return ret;
}

static void btmtk_fb_notify_unregister(void)
{
	fb_unregister_client(&bt_fb_notifier);
}
#endif

static struct notifier_block bt_pm_notifier;
static int btmtk_pm_notifier_callback(struct notifier_block *nb,
		unsigned long event, void *dummy)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;


	switch (event) {
		case PM_SUSPEND_PREPARE:
			BTMTK_INFO("%s: bt_state[%d], event[%ld]",
					__func__, cif_dev->bt_state, event);
		case PM_POST_SUSPEND:
			if(cif_dev->bt_state == FUNC_ON) {
				bt_dump_bgfsys_suspend_wakeup_debug();
				/* bt_dump_bgfsys_top_common_flag(); */
				bthost_debug_print();
			}
			break;
		default:
			break;
	}
	return 0;
}

static int btmtk_pm_notify_register(void)
{
	int32_t ret;

	bt_pm_notifier.notifier_call = btmtk_pm_notifier_callback;

	ret = register_pm_notifier(&bt_pm_notifier);
	if (ret)
		BTMTK_ERR("Register bt_pm_notifier failed:%d\n", ret);
	else
		BTMTK_DBG("Register bt_pm_notifier succeed\n");

	return ret;
}

static void btmtk_pm_notify_unregister(void)
{
	unregister_pm_notifier(&bt_pm_notifier);
}


/* btmtk_cif_dump_fw_no_rsp
 *
 *    Dump fw/driver own cr and btif cr if
 *    fw response timeout
 *
 * Arguments:
 *
 *
 * Return Value:
 *     N/A
 *
 */
void btmtk_cif_dump_fw_no_rsp(unsigned int flag)
{
	BTMTK_WARN("%s! [%u]", __func__, flag);
	if (!g_btif_id) {
		BTMTK_ERR("NULL BTIF ID reference!");
	} else {
		if (flag & BT_BTIF_DUMP_OWN_CR)
			bt_dump_cif_own_cr();

		if (flag & BT_BTIF_DUMP_REG)
			mtk_wcn_btif_dbg_ctrl(g_btif_id, BTIF_DUMP_BTIF_REG);

		if (flag & BT_BTIF_DUMP_LOG)
			mtk_wcn_btif_dbg_ctrl(g_btif_id, BTIF_DUMP_LOG);

		if (flag & BT_BTIF_DUMP_DMA)
			mtk_wcn_btif_dbg_ctrl(g_btif_id, BTIF_DUMP_DMA_VFIFO);
	}
}


void btmtk_cif_dump_rxd_backtrace(void)
{
	if (!g_btif_rxd_thread)
		BTMTK_ERR("g_btif_rxd_thread == NULL");
	else
		KERNEL_show_stack(g_btif_rxd_thread, NULL);
}


/* btmtk_cif_dump_btif_tx_no_rsp
 *
 *    Dump btif cr if fw no response yet
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     N/A
 *
 */
void btmtk_cif_dump_btif_tx_no_rsp(void)
{
	BTMTK_INFO("%s", __func__);
	if (!g_btif_id) {
		BTMTK_ERR("NULL BTIF ID reference!");
	} else {
		mtk_wcn_btif_dbg_ctrl(g_btif_id, BTIF_DUMP_BTIF_IRQ);
	}
}

/*
Sample Code to get time diff:

	struct timeval time_cur, time_pre;
	unsigned long time_ms_val;
	do_gettimeofday(&time_pre);

	...do the task you want to calculate time diff...

	do_gettimeofday(&time_cur);
	time_ms_val = (time_cur.tv_sec - time_pre.tv_sec) * 1000;
	time_ms_val += ((time_cur.tv_usec - time_pre.tv_usec) / 1000);
	BTMTK_DBG("time_ms_val[%ld]", time_ms_val);
*/

/* btmtk_cif_fw_own_clr()
 *
 *    Ask FW to wakeup from sleep state, driver should invoke this function
 *    before communitcate with FW/HW
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 */
static int32_t btmtk_cif_fw_own_clr(void)
{
	uint32_t lpctl_cr;
	int32_t retry = LPCR_POLLING_RTY_LMT;

	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_DRVOWN_IN, 0, 0, NULL);
	do {
		/* assume wait interval 0.5ms each time,
		 * wait maximum total 7ms to query status
		 */
		if ((retry & 0xF) == 0) { /* retry % 16 == 0 */
			if (((retry < LPCR_POLLING_RTY_LMT && retry >= LPCR_MASS_DUMP_LMT) || (retry == 2048) || (retry == 32)) &&
				((retry & 0x1F) == 0)) {
				BTMTK_WARN("[DRV_OWN] failed in %d ms, retry[%d]", (LPCR_POLLING_RTY_LMT - retry) / 2, retry);
				bt_dump_cif_own_cr();
				REG_WRITEL(BGF_LPCTL, BGF_HOST_CLR_FW_OWN_B);
				BTMTK_WARN("[DRV_OWN] dump after write:");
				bt_dump_cif_own_cr();
			} else
				REG_WRITEL(BGF_LPCTL, BGF_HOST_CLR_FW_OWN_B);
		}

		lpctl_cr = REG_READL(BGF_LPCTL);
		if (!(lpctl_cr & BGF_OWNER_STATE_SYNC_B)) {
			REG_WRITEL(BGF_IRQ_STAT, BGF_IRQ_FW_OWN_CLR_B);
			break;
		}

		usleep_range(500, 550);
		retry --;
	} while (retry > 0);

	if (retry == 0) {
		BTMTK_ERR("[DRV_OWN] (Wakeup) failed!");
		if (g_bt_trace_pt)
			bt_dbg_tp_evt(TP_ACT_DRVOWN_OUT, TP_PAR_FAIL, 0, NULL);
		bt_dump_cif_own_cr();

		/* dump cpupcr, 10 times with 1ms interval */
		bt_dump_cpupcr(10 , 5);
		return -1;
	}

	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_DRVOWN_OUT, TP_PAR_PASS, 0, NULL);
	BTMTK_DBG("[DRV_OWN] (Wakeup) success, retry[%d]", retry);
	return 0;
}

/* btmtk_cif_fw_own_set()
 *
 *    Ask FW to sleep for power saving
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int32_t btmtk_cif_fw_own_set(void)
{
	uint32_t irqstat_cr;
	int32_t retry = LPCR_POLLING_RTY_LMT;

	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_FWOWN_IN, 0, 0, NULL);
	do {
		if ((retry & 0xF) == 0) { /* retry % 16 == 0 */
			if (((retry < LPCR_POLLING_RTY_LMT && retry >= LPCR_MASS_DUMP_LMT) || (retry == 2048) || (retry == 32)) &&
				((retry & 0x1F) == 0)) {
				BTMTK_WARN("[FW_OWN] failed in %d ms, retry[%d]", (LPCR_POLLING_RTY_LMT - retry) / 2, retry);
				bt_dump_cif_own_cr();
				REG_WRITEL(BGF_LPCTL, BGF_HOST_SET_FW_OWN_B);
				BTMTK_WARN("[FW_OWN] dump after write:");
				bt_dump_cif_own_cr();
			} else
				REG_WRITEL(BGF_LPCTL, BGF_HOST_SET_FW_OWN_B);
		}

		/*
		 * As current H/W design, BGF_LPCTL.OWNER_STATE_SYNC bit will be automatically
		 * asserted by H/W once driver set FW own, not waiting for F/W's ack,
		 * which means F/W may still transmit data to host at that time.
		 * So we cannot check this bit for FW own done identity, use BGF_IRQ_FW_OWN_SET
		 * bit for instead, even on polling mode and the interrupt is masked.
		 *
		 * This is a work-around, H/W will change design on ECO.
		 */
		irqstat_cr = REG_READL(BGF_IRQ_STAT2);
		if (irqstat_cr & BGF_IRQ_FW_OWN_SET_B) {
			/* Write 1 to clear IRQ */
			REG_WRITEL(BGF_IRQ_STAT2, BGF_IRQ_FW_OWN_SET_B);
			break;
		}

		usleep_range(500, 550);
		retry --;
	} while (retry > 0);

	if (retry == 0) {
		BTMTK_ERR("[FW_OWN] (Sleep) failed!");
		if (g_bt_trace_pt)
			bt_dbg_tp_evt(TP_ACT_FWOWN_OUT, TP_PAR_FAIL, 0, NULL);
		bt_dump_cif_own_cr();

		/* dump cpupcr, 10 times with 1ms interval */
		bt_dump_cpupcr(10 , 5);
		return -1;
	}

	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_FWOWN_OUT, TP_PAR_PASS, 0, NULL);
	BTMTK_DBG("[FW_OWN] (Sleep) success, retry[%d]", retry);
	return 0;
}

void bt_notify_state(void)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	int32_t i = 0;

	for(i = 0; i < MAX_STATE_MONITORS; i++)
		if (cif_dev->state_change_cb[i])
			cif_dev->state_change_cb[i](cif_dev->bt_state);
}

#if (USE_DEVICE_NODE == 0)
/* bt_report_hw_error()
 *
 *    Insert an event to stack to report error while recovering from chip reset
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
void bt_report_hw_error()
{
	const uint8_t HCI_EVT_HW_ERROR[] = {0x04, 0x10, 0x01, 0x00};
	btmtk_recv(g_sbdev->hdev, HCI_EVT_HW_ERROR, sizeof(HCI_EVT_HW_ERROR));
}
#endif

int bt_chip_reset_flow(enum bt_reset_level rst_level,
			     enum consys_drv_type drv,
			     char *reason)
{
	u_int8_t is_1st_reset = 0;
	uint8_t retry = 15;
	int32_t dump_property = 0; /* default = 0 */
	int32_t ret = 0;
	struct btmtk_dev *bdev = hci_get_drvdata(g_sbdev->hdev);
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_RST, TP_PAR_RST_START, 0, NULL);
	/* dump debug message */
	show_all_dump_packet();
	btmtk_cif_dump_fw_no_rsp(BT_BTIF_DUMP_ALL);

	is_1st_reset = (cif_dev->rst_count == 0) ? TRUE : FALSE;
	cif_dev->rst_count++;
	while (cif_dev->rst_level != RESET_LEVEL_NONE && retry > 0) {
		msleep(200);
		retry--;
	}

	if (retry == 0) {
		BTMTK_ERR("Unable to trigger pre_chip_rst due to unfinish previous reset");
		return -1;
	}

	cif_dev->rst_level = rst_level;
	cif_dev->rst_flag = TRUE;

	if (is_1st_reset) {
		/*
		 * 1. Trigger BT PAD_EINT to force FW coredump,
		 *    trigger coredump only if the first time reset
		 *    (compare with the case of subsys reset fail)
		 */
		CLR_BIT(BGF_PAD_EINT, BIT(9));
		bt_enable_irq(BGF2AP_SW_IRQ);

		/* 2. Wait for IRQ */
		if (!wait_for_completion_timeout(&cif_dev->rst_comp, msecs_to_jiffies(2000))) {
#if SUPPORT_COREDUMP
			dump_property = CONNSYS_DUMP_PROPERTY_NO_WAIT;
#endif
			BTMTK_ERR("uanble to get reset IRQ in 2000ms");
		}

		/* 3. Reset PAD_INT CR */
		SET_BIT(BGF_PAD_EINT, BIT(9));
#if SUPPORT_COREDUMP
		/* 4. Do coredump, only do this while BT is on */
		down(&cif_dev->halt_sem);
		if (cif_dev->bt_state != RESET_START && cif_dev->bt_state != FUNC_OFF) {
			if (g_bt_trace_pt)
				bt_dbg_tp_evt(TP_ACT_RST, TP_PAR_RST_DUMP, 0, NULL);
			bt_dump_bgfsys_debug_cr();
			connsys_coredump_start(cif_dev->coredump_handle, dump_property, drv, reason);
		} else
			BTMTK_WARN("BT state [%d], skip coredump", cif_dev->bt_state);
		up(&cif_dev->halt_sem);
#endif
	}

	/* Dump assert reason, only for subsys reset */
	if (rst_level == RESET_LEVEL_0_5) {
		phys_addr_t emi_ap_phy_base;
		uint8_t *dump_msg_addr;
		uint8_t msg[256] = {0};

		conninfra_get_phy_addr(&emi_ap_phy_base, NULL);

		dump_msg_addr = ioremap(emi_ap_phy_base + 0x3B000, 0x100);
		if (dump_msg_addr) {
			memcpy(msg, dump_msg_addr, 0xFF);
			iounmap(dump_msg_addr);
			msg[0xFF] = 0;
			BTMTK_INFO("FW Coredump Msg: [%s]", msg);
		} else {
			BTMTK_ERR("uanble to remap dump msg addr");
		}
	}

	/* set flag for reset during bt on/off */
	cif_dev->rst_flag = FALSE;
	wake_up_interruptible(&cif_dev->rst_onoff_waitq);

	/* directly return if following cases
	 * 1. BT is already off
	 * 2. another reset is already done whole procdure and back to normal
	 */
	if (cif_dev->bt_state == FUNC_OFF || cif_dev->rst_level == RESET_LEVEL_NONE) {
		BTMTK_INFO("BT is already off or reset success, skip power off [%d, %d]",
			cif_dev->bt_state, cif_dev->rst_level);
		return 0;
	}

	cif_dev->bt_state = RESET_START;
	bt_notify_state();

	/* 5. Turn off BT */
	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_RST, TP_PAR_RST_OFF, 0, NULL);
	btmtk_fops_set_state(bdev, BTMTK_FOPS_STATE_OPENED); // to comform to the common part state
	ret = g_sbdev->hdev->close(g_sbdev->hdev);
#if (USE_DEVICE_NODE == 0)
	bt_report_hw_error();
#endif
	return ret;
}


/*******************************************************************************
*                   C A L L   B A C K   F U N C T I O N S
********************************************************************************
*/
/* bt_pre_chip_rst_handler()
 *
 *    Pre-action of chip reset (before HW power off), driver should ask
 *    conninfra to do coredump and then turn off bt driver
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int bt_pre_chip_rst_handler(enum consys_drv_type drv, char *reason)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	// skip reset flow if bt is not on
	if (cif_dev->bt_state == FUNC_OFF)
		return 0;
	else
		return bt_chip_reset_flow(RESET_LEVEL_0, drv, reason);
}

/* bt_post_chip_rst_handler()
 *
 *    Post-action of chip reset (after HW power on), turn on BT
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int bt_post_chip_rst_handler(void)
{
	bt_notify_state();
	return 0;
}

/* bt_do_pre_power_on()
 *
 * Only set the flag to pre-calibration mode here
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 always (for success)
 *
 */
static int bt_do_pre_power_on(void)
{
	return btmtk_set_power_on(g_sbdev->hdev, TRUE);
}

/* bt_do_calibration()
 *
 *    calibration flow is control by conninfra driver, driver should implement
 *    the function of calibration callback, here what driver do is send cmd to
 *    FW to get calibration data (BT calibration is done in BT func on) and
 *    backup the calibration data
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int32_t bt_do_calibration(void)
{
	int32_t ret = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if(cif_dev->bt_state != FUNC_ON) {
		BTMTK_ERR("%s: bt is not on, skip calibration", __func__);
		return CONNINFRA_CB_RET_CAL_FAIL_POWER_OFF;
	}

	ret = btmtk_cif_send_calibration(g_sbdev);
	/* return -1 means driver unable to recv calibration event and reseting
	 * In such case, we don't have to turn off bt, it will be handled by
	 * reset thread */
	if (ret != -1 && cif_dev->bt_precal_state == FUNC_OFF)
		btmtk_set_power_off(g_sbdev->hdev, TRUE);

	if (ret == -1) {
		BTMTK_ERR("%s: error return in recving calibration event, reset", __func__);
		return CONNINFRA_CB_RET_CAL_FAIL_POWER_ON;
	} else if (ret) {
		BTMTK_ERR("%s: error return in sent calibration cmd", __func__);
		return (cif_dev->bt_precal_state) ? CONNINFRA_CB_RET_CAL_FAIL_POWER_ON :
						   CONNINFRA_CB_RET_CAL_FAIL_POWER_OFF;
	} else
		return (cif_dev->bt_precal_state) ? CONNINFRA_CB_RET_CAL_PASS_POWER_ON :
						   CONNINFRA_CB_RET_CAL_PASS_POWER_OFF;

}

void btmtk_time_change_notify(void)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if(cif_dev->bt_state == FUNC_ON)
		btmtk_intcmd_wmt_utc_sync();
}

/* sub_drv_ops_cb
 *
 *    All callbacks needs by conninfra driver, 3 types of callback functions
 *    1. Chip reset functions
 *    2. Calibration functions
 *    3. Thermal query functions (todo)
 *
 */
static struct sub_drv_ops_cb bt_drv_cbs =
{
	.rst_cb.pre_whole_chip_rst = bt_pre_chip_rst_handler,
	.rst_cb.post_whole_chip_rst = bt_post_chip_rst_handler,
	.pre_cal_cb.pwr_on_cb = bt_do_pre_power_on,
	.pre_cal_cb.do_cal_cb = bt_do_calibration,
	.thermal_qry = btmtk_intcmd_query_thermal,
	.time_change_notify = btmtk_time_change_notify,
};

/* bt_receive_data_cb
 *
 *    Callback function register to BTIF while BTIF receiving data from FW
 *
 * Arguments:
 *     [IN] buf    pointer to incoming data buffer
 *     [IN] count  length of incoming data
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int32_t bt_receive_data_cb(uint8_t *buf, uint32_t count)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_RD_CB, 0, count, buf);
	BTMTK_DBG_RAW(buf, count, "%s: len[%d] RX: ", __func__, count);
	add_dump_packet(buf, count, RX);
	cif_dev->psm.sleep_flag = FALSE;
	return btmtk_recv(g_sbdev->hdev, buf, count);
}

#if SUPPORT_COREDUMP
/* bt_coredump_cb
 *
 *    coredump API to check bt cr status
 *    Access bgf bus won't hang, so checking conninfra bus
 *
 */
static struct coredump_event_cb bt_coredump_cb =
{
	.reg_readable = conninfra_reg_readable_for_coredump,
	.poll_cpupcr = bt_dump_cpupcr,
};
#endif


/*******************************************************************************
*                        bt power throttling feature
********************************************************************************
*/
bool bt_pwrctrl_support(void)
{
	return TRUE;
}

int bt_pwrctrl_level_change_cb(enum conn_pwr_event_type type, void *data)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	int8_t set_val = cif_dev->dy_pwr.dy_max_dbm;

	BTMTK_INFO("%s", __func__);
	switch(type) {
	case CONN_PWR_EVENT_LEVEL:
		cif_dev->dy_pwr.lp_cur_lv = *((int *) data);
		BTMTK_INFO("%s: lp_cur_bat_lv = %d", __func__, cif_dev->dy_pwr.lp_cur_lv);
		btmtk_inttrx_DynamicAdjustTxPower(HCI_CMD_DY_ADJ_PWR_SET, set_val, NULL, FALSE);
		break;
	case CONN_PWR_EVENT_MAX_TEMP:
		BTMTK_ERR("Unsupport now");
		break;
	default:
		BTMTK_ERR("Uknown type for power throttling callback");
		break;
	}

	return 0;
}

void bt_pwrctrl_pre_on(void)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if (!bt_pwrctrl_support())
		return;

	memset(&cif_dev->dy_pwr, 0x00, sizeof(cif_dev->dy_pwr));
	cif_dev->dy_pwr.lp_cur_lv = CONN_PWR_THR_LV_0;
	conn_pwr_drv_pre_on(CONN_PWR_DRV_BT, &cif_dev->dy_pwr.lp_cur_lv);
	BTMTK_INFO("%s: lp_cur_bat_lv = %d", __func__, cif_dev->dy_pwr.lp_cur_lv);
}

void bt_pwrctrl_post_off(void)
{
	if (!bt_pwrctrl_support())
		return;

	conn_pwr_drv_post_off(CONN_PWR_DRV_BT);
}

void bt_pwrctrl_register_evt(void)
{
	if (!bt_pwrctrl_support())
		return;

	BTMTK_INFO("%s", __func__);
	/* Register callbacks for power throttling feature */
	conn_pwr_register_event_cb(CONN_PWR_DRV_BT, (CONN_PWR_EVENT_CB)bt_pwrctrl_level_change_cb);
}

/*******************************************************************************
*                        B T I F  F U N C T I O N S
********************************************************************************
*/
#if SUPPORT_BT_THREAD
static void btmtk_btif_enter_deep_idle(struct work_struct *pwork)
{
	int32_t ret = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct btif_deepidle_ctrl *idle_ctrl = &cif_dev->btif_dpidle_ctrl;

	down(&idle_ctrl->sem);
	BTMTK_DBG("[DP_IDLE] idle = [%d]", idle_ctrl->is_dpidle);
	if (idle_ctrl->is_dpidle) {
		ret = mtk_wcn_btif_dpidle_ctrl(g_btif_id, BTIF_DPIDLE_ENABLE);
		bt_release_wake_lock(&cif_dev->psm.wake_lock);
		idle_ctrl->is_dpidle = (ret) ? FALSE : TRUE;

		if (g_bt_trace_pt)
			bt_dbg_tp_evt(TP_ACT_DPI_ENTER, (ret == 0 ? TP_PAR_PASS : TP_PAR_FAIL), 0, NULL);
		if (ret)
			BTMTK_ERR("[DP_IDLE] BTIF enter dpidle failed(%d)", ret);
		else
			BTMTK_DBG("[DP_IDLE] BTIF enter dpidle success");
	} else
		BTMTK_DBG("[DP_IDLE] idle is set to false, skip this time");
	up(&idle_ctrl->sem);
}

/* btmtk_btif_dpidle_ctrl
 *
 *
 *     Ask btif to wakeup / sleep
 *
 * Arguments:
 *     enable    0 to wakeup BTIF, sleep otherwise
 *
 * Return Value:
 *     0 if success, otherwise error code
 */
static int32_t btmtk_btif_dpidle_ctrl(u_int8_t enable)
{
	int32_t ret = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct btif_deepidle_ctrl *idle_ctrl = &cif_dev->btif_dpidle_ctrl;
	down(&idle_ctrl->sem);

	if (!g_btif_id) {
		BTMTK_ERR("[DP_IDLE] NULL BTIF ID reference!");
		return -1;
	}

	if(idle_ctrl->task != NULL) {
		BTMTK_DBG("[DP_IDLE] enable = %d", enable);
		/* 1. Remove active timer, and remove a unschedule timer does no harm */
		cancel_delayed_work(&idle_ctrl->work);

		/* 2. Check enable or disable */
		if (!enable) {
			/* disable deep idle, call BTIF api directly */
			bt_hold_wake_lock(&cif_dev->psm.wake_lock);
			ret = mtk_wcn_btif_dpidle_ctrl(g_btif_id, BTIF_DPIDLE_DISABLE);
			idle_ctrl->is_dpidle = (ret) ? TRUE : FALSE;

			if (g_bt_trace_pt)
				bt_dbg_tp_evt(TP_ACT_DPI_EXIT, (ret == 0 ? TP_PAR_PASS : TP_PAR_FAIL), 0, NULL);
			if (ret)
				BTMTK_ERR("[DP_IDLE] BTIF exit dpidle failed(%d)", ret);
			else
				BTMTK_DBG("[DP_IDLE] BTIF exit dpidle success");
		} else {
			BTMTK_DBG("[DP_IDLE] create timer for enable deep idle");
			idle_ctrl->is_dpidle = TRUE;
			/* enable deep idle, schedule a timer */
			queue_delayed_work(idle_ctrl->task, &idle_ctrl->work,
						(BTIF_IDLE_WAIT_TIME * HZ) >> 10);
		}
	} else {
		BTMTK_INFO("[DP_IDLE] idle_ctrl->task already cancelled!");
	}

	up(&idle_ctrl->sem);
	return ret;
}

#endif

int btmtk_cif_rx_packet_handler(struct hci_dev *hdev, struct sk_buff *skb)
{
	int err = 0;
#if (USE_DEVICE_NODE == 0)
	err = hci_recv_frame(hdev, skb);
#else
	err = rx_skb_enqueue(skb);
#endif
	return err;
}

/* _check_wmt_evt_over_hci
 *
 *    Check incoming event (RX data) if it's a WMT(vendor) event
 *    (won't send to stack)
 *
 * Arguments:
 *    [IN] buffer   - event buffer
 *    [IN] len      - event length
 *    [IN] expected_op - OPCODE that driver is waiting for (calle assigned)
 *    [IN] p_evt_params - event parameter
 *
 * Return Value:
 *    return check
 *    WMT_EVT_SUCCESS - get expected event
 *    WMT_EVT_FAIL    - otherwise
 */
static int32_t _check_wmt_evt_over_hci(
		uint8_t *buffer,
		uint16_t len,
		uint8_t  expected_op,
		struct wmt_pkt_param *p_evt_params)
{
	struct wmt_pkt *p_wmt_evt;
	uint8_t opcode, sub_opcode;
	uint8_t status = 0xFF; /* reserved value for check error */
	uint16_t param_len = 0;

	/* Sanity check */
	if (len < (HCI_EVT_HDR_LEN + WMT_EVT_HDR_LEN)) {
		BTMTK_ERR("Incomplete packet len %d for WMT event!", len);
		goto check_error;
	}

	p_wmt_evt = (struct wmt_pkt *)&buffer[WMT_EVT_OFFSET];
	if (p_wmt_evt->hdr.dir != WMT_PKT_DIR_CHIP_TO_HOST) {
		BTMTK_ERR("WMT direction %x error!", p_wmt_evt->hdr.dir);
		goto check_error;
	}

	opcode = p_wmt_evt->hdr.opcode;
	if (opcode != expected_op) {
		BTMTK_ERR("WMT OpCode is unexpected! opcode[0x%02X], expect[0x%02X]", opcode, expected_op);
		goto check_error;
	}

	param_len = (p_wmt_evt->hdr.param_len[1] << 8) | (p_wmt_evt->hdr.param_len[0]);
	/* Sanity check */
	if (len < (HCI_EVT_HDR_LEN + WMT_EVT_HDR_LEN + param_len)) {
		BTMTK_ERR("Incomplete packet len %d for WMT event!", len);
		goto check_error;
	}

	switch (opcode) {
	case WMT_OPCODE_FUNC_CTRL:
		if (param_len != sizeof(p_wmt_evt->params.u.func_ctrl_evt)) {
			BTMTK_ERR("Unexpected event param len %d for WMT OpCode 0x%x!",
				      param_len, opcode);
			break;
		}
		status = p_wmt_evt->params.u.func_ctrl_evt.status;
		break;

	case WMT_OPCODE_RF_CAL:
		sub_opcode = p_wmt_evt->params.u.rf_cal_evt.subop;

		if (sub_opcode != 0x03) {
			BTMTK_ERR("Unexpected subop 0x%x for WMT OpCode 0x%x!",
				      sub_opcode, opcode);
			break;
		}

		if (param_len != sizeof(p_wmt_evt->params.u.rf_cal_evt)) {
			BTMTK_ERR("Unexpected event param len %d for WMT OpCode 0x%x!",
				      param_len, opcode);
			break;
		}
		status = p_wmt_evt->params.u.rf_cal_evt.status;
		break;
	case WMT_OPCODE_0XF0:
	case WMT_OPCODE_ANT_EFEM:
		status = 0x00; // todo: need more check?
		break;
	default:
		BTMTK_ERR("Unknown WMT OpCode 0x%x!", opcode);
		break;
	}

	if (status != 0xFF) {
		memcpy(p_evt_params, &p_wmt_evt->params, param_len);
		return (status == 0) ? WMT_EVT_SUCCESS : WMT_EVT_FAIL;
	}

check_error:
	BTMTK_DBG_RAW(buffer, len, "Dump data: ");
	return WMT_EVT_INVALID;
}

/* btmtk_dispatch_event
 *
 *    Handler of vendor event
 *
 * Arguments:
 *    [IN] hdev     - hci_device as control structure during BT life cycle
 *    [IN] skb      - packet that carries vendor event
 *
 * Return Value:
 *    return check  - 0 for checking success, -1 otherwise
 *
 */
int32_t btmtk_dispatch_event(struct sk_buff *skb)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_internal_cmd *p_inter_cmd = &cif_dev->internal_cmd;
	uint8_t event_code = skb->data[0];

	BTMTK_DBG("%s: event code=[0x%02x], length = %d", __func__, event_code, skb->len);

	if (event_code == 0xE4) {
		/* WMT event */
		p_inter_cmd->result = _check_wmt_evt_over_hci(skb->data,
					      skb->len,
					      p_inter_cmd->wmt_opcode,
					      &p_inter_cmd->wmt_event_params);

		//p_inter_cmd->result = (ret == WMT_EVT_SUCCESS) ? 1: 0;
	} else if(event_code == 0x0E && skb->len == 10 &&
		  skb->data[3] == 0x91 && skb->data[4] == 0xFD) {
		/* Sepcial case for thermal event, currently FW thermal event
		 * is not a typical WMT event, we have to do it separately
		 */
		memcpy((void*)&p_inter_cmd->wmt_event_params.u, skb->data, skb->len);
		p_inter_cmd->result = WMT_EVT_SUCCESS;
	} else {
		/* Not WMT event */
		p_inter_cmd->result = WMT_EVT_SKIP;
	}

	//return (p_inter_cmd->result == 1) ? 0 : -1;
	return p_inter_cmd->result;
}


void btmtk_check_event_timeout(struct sk_buff *skb)
{
	u8 event_code = skb->data[0];
	u16 cmd_opcode;

	if (event_code == HCI_EVT_COMPLETE_EVT) {
		cmd_opcode = (skb->data[3] << 8) | skb->data[4];

		if (cmd_list_check(cmd_opcode) == FALSE) {
			BTMTK_ERR("%s No match command %4X", __func__, cmd_opcode);
		} else {
			cmd_list_remove(cmd_opcode);
			update_command_response_workqueue();
		}
	} else if (event_code == HCI_EVT_STATUS_EVT) {
		cmd_opcode = (skb->data[4] << 8) | skb->data[5];

		if (cmd_list_check(cmd_opcode) == FALSE) {
			BTMTK_ERR("%s No match command %4X", __func__, cmd_opcode);
		} else {
			cmd_list_remove(cmd_opcode);
			update_command_response_workqueue();
		}
	}
}

int32_t btmtk_send_data(struct hci_dev *hdev, uint8_t *buf, uint32_t count)
{
	struct sk_buff *skb = NULL;
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_btif_dev *cif_dev = bdev->cif_dev;

	skb = alloc_skb(count + BT_SKB_RESERVE, GFP_KERNEL);
	if (skb == NULL) {
		BTMTK_ERR("%s allocate skb failed!!", __func__);
		return -1;
	}

	/* Reserv for core and drivers use */
	skb_reserve(skb, BT_SKB_RESERVE);
	bt_cb(skb)->pkt_type = buf[0];
	memcpy(skb->data, buf, count);
	skb->len = count;

	skb_queue_tail(&cif_dev->tx_queue, skb);
	wake_up_interruptible(&cif_dev->tx_waitq);

	return count;
}

/* btmtk_wcn_btif_open

 *
 *     Called when line discipline changed to HCI_UART.
 *
 * Arguments:
 *     tty    pointer to tty info structure
 * Return Value:
 *     0 if success, otherwise error code
 */
int32_t btmtk_wcn_btif_open(void)
{
	int32_t ret = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct btif_deepidle_ctrl *idle_ctrl = &cif_dev->btif_dpidle_ctrl;

	/* 1. Open BTIF */
	ret = mtk_wcn_btif_open(BTIF_OWNER_NAME, &g_btif_id);
	if (ret) {
		BTMTK_ERR("BT open BTIF failed(%d)", ret);
		return -1;
	}

#if SUPPORT_BT_THREAD
	idle_ctrl->is_dpidle = FALSE;
	idle_ctrl->task = create_singlethread_workqueue("dpidle_task");
	if (!idle_ctrl->task){
		BTMTK_ERR("fail to idle_ctrl->task ");
		return -1;
	}
	INIT_DELAYED_WORK(&idle_ctrl->work, btmtk_btif_enter_deep_idle);
#endif

	/* 2. Register rx callback */
	ret = mtk_wcn_btif_rx_cb_register(g_btif_id, (MTK_WCN_BTIF_RX_CB)bt_receive_data_cb);
	if (ret) {
		BTMTK_ERR("Register rx cb to BTIF failed(%d)", ret);
		mtk_wcn_btif_close(g_btif_id);
		return -1;
	}

	g_btif_rxd_thread = mtk_btif_exp_rx_thread_get(g_btif_id);

#if SUPPORT_BT_THREAD
	/* 3. Enter deeple idle */
	ret = mtk_wcn_btif_dpidle_ctrl(g_btif_id, TRUE);
	if (ret) {
		BTMTK_ERR("BTIF enter dpidle failed(%d)", ret);
		mtk_wcn_btif_close(g_btif_id);
		return -1;
	}
#endif

	BTMTK_DBG("BT open BTIF OK");
	return 0;
}

/* btmtk_wcn_btif_close()
 *
 *    Close btif
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 */
int32_t btmtk_wcn_btif_close()
{
	int32_t ret = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if (!g_btif_id) {
		BTMTK_ERR("NULL BTIF ID reference!");
		return 0;
	}

#if SUPPORT_BT_THREAD
	if(cif_dev->btif_dpidle_ctrl.task != NULL) {
		cancel_delayed_work(&cif_dev->btif_dpidle_ctrl.work);
		flush_workqueue(cif_dev->btif_dpidle_ctrl.task);
		down(&cif_dev->btif_dpidle_ctrl.sem);
		destroy_workqueue(cif_dev->btif_dpidle_ctrl.task);
		up(&cif_dev->btif_dpidle_ctrl.sem);
		cif_dev->btif_dpidle_ctrl.task = NULL;
	}
#endif

	ret = mtk_wcn_btif_close(g_btif_id);
	g_btif_id = 0;
	bt_release_wake_lock(&cif_dev->psm.wake_lock);

	if (ret) {
		BTMTK_ERR("BT close BTIF failed(%d)", ret);
		return -1;
	}
	BTMTK_DBG("BT close BTIF OK");
	return 0;

}

int btmtk_btif_open(struct hci_dev *hdev)
{
	int ret = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	bt_hold_wake_lock(&cif_dev->psm.wake_lock);
	ret = btmtk_set_power_on(hdev, FALSE);
	bt_release_wake_lock(&cif_dev->psm.wake_lock);
	return ret;
}

int btmtk_btif_close(struct hci_dev *hdev)
{
	int ret = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	bt_hold_wake_lock(&cif_dev->psm.wake_lock);
	ret = btmtk_set_power_off(hdev, FALSE);
	bt_release_wake_lock(&cif_dev->psm.wake_lock);
	return ret;
}

int btmtk_btif_send_cmd_if(struct btmtk_dev *bdev, struct sk_buff *skb, int delay, int retry, int pkt_type)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

#if SUPPORT_BT_THREAD
	skb_queue_tail(&cif_dev->tx_queue, skb);
	wake_up_interruptible(&cif_dev->tx_waitq);
#else
	btmtk_btif_send_cmd(bdev, skb, delay, retry, pkt_type);
	kfree_skb(skb);
#endif
	return 0;
}

int btmtk_btif_send_and_recv(struct btmtk_dev *bdev,
		struct sk_buff *skb,
		const uint8_t *event, const int event_len,
		int delay, int retry, int pkt_type)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	int ret = 0;
	int tx_state = pkt_type; // use for tx_state control

	set_bit(tx_state, &bdev->tx_state);
#if SUPPORT_BT_THREAD
	skb_queue_tail(&cif_dev->tx_queue, skb);
	wake_up_interruptible(&cif_dev->tx_waitq);
#else
	btmtk_btif_send_cmd_if(bdev, skb, 0, 5, (int)BTMTK_TX_CMD_FROM_DRV);
#endif
	ret = wait_event_timeout(bdev->p_wait_event_q,
				bdev->evt_skb != NULL || tx_state == BTMTK_TX_SKIP_VENDOR_EVT, 2*HZ);
	BTMTK_INFO("%s, ret = %d", __func__, ret);

	if (tx_state == BTMTK_TX_WAIT_VND_EVT) {
		if (bdev->evt_skb)
			kfree_skb(bdev->evt_skb);
		bdev->evt_skb = NULL;
	}
	return ret;
}

/* btmtk_btif_send_cmd
 *
 *    Public API of BT TX
 *
 * Arguments:
 *    [IN] bdev     - hci_device as control structure during BT life cycle
 *    [IN] skb      - skb data buffer
 *    [IN] delay    - NOT USE in BTIF driver (ignore)
 *    [IN] retry    - retry count
 *    [IN] endpoint - NOT USE in BTIF driver (ignore)
 *
 * Return Value:
 *    write length of TX
 */
int btmtk_btif_send_cmd(struct btmtk_dev *bdev, struct sk_buff *skb, int delay, int retry, int pkt_type)
{
	int32_t ret = -1;
	uint32_t wr_count = 0;
	uint8_t *cmd = skb->data;
	int32_t cmd_len = skb->len;
	int32_t tx_len = skb->len;
	int32_t _retry = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)bdev->cif_dev;
	uint8_t *tp_ptr = (uint8_t *)cmd;

	BTMTK_DBG_RAW(cmd, cmd_len, "%s: len[%d] TX: ", __func__, cmd_len);

	if (!g_btif_id) {
		BTMTK_ERR("NULL BTIF ID reference!");
		return -1;
	}

	add_dump_packet(cmd, cmd_len, TX);

#if (DRIVER_CMD_CHECK == 1)
		// To do: need to check if list_append will affect throughput
		if (cmd[0] == 0x01) {
			if ((cmd[1] == 0x5D && cmd[2] == 0xFC) ||
				(cmd[1] == 0x6F && cmd[2] == 0xFC) ||
				cif_dev->cmd_timeout_check == FALSE ) {
				/* Skip these cmd:
				fw will not return response, or response with other format */
			} else {
				u16 cmd_opcode = (cmd[1] << 8) | cmd[2];
				BTMTK_DBG("%s add opcode %4X in cmd queue", __func__,cmd_opcode);
				cmd_list_append(cmd_opcode);
				update_command_response_workqueue();
			}
		}
#endif

	while (tx_len > 0 && _retry < retry) {
		if (_retry++ > 0)
			usleep_range(USLEEP_5MS_L, USLEEP_5MS_H);

		ret = mtk_wcn_btif_write(g_btif_id, cmd, tx_len);
		if (ret < 0) {
			BTMTK_ERR("BTIF write failed(%d) on retry(%d)", ret, _retry-1);
			return -1;
		}
		tx_len -= ret;
		wr_count += ret;
		cmd += ret;
	}

	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_WR_OUT, 0, cmd_len, tp_ptr);
	return ret;
}

static void btmtk_btif_internal_trx_work(struct work_struct *work)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	btmtk_send_data(g_sbdev->hdev, cif_dev->internal_trx.buf, cif_dev->internal_trx.buf_len);
	if (!wait_for_completion_timeout(&cif_dev->internal_trx.comp, msecs_to_jiffies(2000)))
		BTMTK_ERR("[internal trx] wait event timeout!");
	BTMTK_INFO("[internal trx] complete");
	cif_dev->internal_trx.cb = NULL;
}


int btmtk_btif_internal_trx (uint8_t *buf, uint32_t count,
									BT_RX_EVT_HANDLER_CB cb,
									bool send_to_stack,
									bool is_blocking)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	spin_lock_irqsave(&cif_dev->internal_trx.lock, cif_dev->internal_trx.flag);
	if (cif_dev->internal_trx.cb) {
		BTMTK_ERR("unable to do internal trx before previous trx done");
		return -1;
	}
	cif_dev->internal_trx.cb = cb;
	spin_unlock_irqrestore(&cif_dev->internal_trx.lock, cif_dev->internal_trx.flag);

	cif_dev->internal_trx.opcode = *(buf + 1) + (*(buf + 2) << 8);
	cif_dev->internal_trx.send_to_stack = send_to_stack;
	if (count > 256) {
		BTMTK_ERR("internal command buffer larger than provided");
		return -1;
	}
	memcpy(cif_dev->internal_trx.buf, buf, count);
	cif_dev->internal_trx.buf_len = count;
	BTMTK_INFO_RAW(buf, count, "[internal trx] len[%d] TX: ", count);
	if (is_blocking) {
		btmtk_send_data(g_sbdev->hdev, cif_dev->internal_trx.buf, cif_dev->internal_trx.buf_len);
		if (!wait_for_completion_timeout(&cif_dev->internal_trx.comp, msecs_to_jiffies(2000)))
			BTMTK_ERR("[internal trx] wait event timeout!");
		BTMTK_INFO("[internal trx] complete");
		cif_dev->internal_trx.cb = NULL;
	} else
		schedule_work(&internal_trx_work);
	return 0;
}

int btmtk_btif_event_filter(struct btmtk_dev *bdev, struct sk_buff *skb)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

#if (DRIVER_CMD_CHECK == 1)
	if (cif_dev->cmd_timeout_check == TRUE)
		btmtk_check_event_timeout(skb);
#endif

	/* When someone waits for the WMT event, the skb is being cloned
	 * and being processed the events from there then.
	 */
	if (test_bit(BTMTK_TX_WAIT_VND_EVT, &bdev->tx_state)) {
		/* check is corresponding wmt event */
		if (btmtk_dispatch_event(skb) != WMT_EVT_SKIP) {
			bdev->evt_skb = skb_copy(skb, GFP_KERNEL);
			if (!bdev->evt_skb) {
				BTMTK_ERR("%s: WMT event copy to evt_skb failed", __func__);
				return 1; // this event is filtered
			}

			if (test_and_clear_bit(BTMTK_TX_WAIT_VND_EVT, &bdev->tx_state)) {
				BTMTK_DBG("%s: clear bit BTMTK_TX_WAIT_VND_EVT", __func__);
				wake_up(&bdev->p_wait_event_q);
				BTMTK_DBG("%s: wake_up p_wait_event_q", __func__);
			} else {
				BTMTK_ERR("%s: test_and_clear_bit(BTMTK_TX_WAIT_VND_EVT) fail", __func__);
				if (bdev->evt_skb)
					kfree_skb(bdev->evt_skb);
				bdev->evt_skb = NULL;
			}
			return 1; // this event is filtered
		} else {
			// may be normal packet, continue put skb to rx queue
			BTMTK_INFO("%s: may be normal packet!", __func__);
		}
	}

	/* handle command complete evt if callback is registered */
	if (cif_dev->internal_trx.cb != NULL) {
		if (skb->data[0] == HCI_EVT_COMPLETE_EVT && \
			cif_dev->internal_trx.opcode == skb->data[3] + (skb->data[4] << 8)) {
			BTMTK_INFO_RAW(skb->data, (int)skb->len, "[internal trx] len[%d] RX: ", (int)skb->len);
			cif_dev->internal_trx.cb(skb->data, (int)skb->len);
			complete(&cif_dev->internal_trx.comp);
			if (!cif_dev->internal_trx.send_to_stack)
				return -1; // do not send to stack
		}
	}

	return 0;
}

static int btmtk_btif_flush(struct btmtk_dev *bdev)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)bdev->cif_dev;

#if SUPPORT_BT_THREAD
	skb_queue_purge(&cif_dev->tx_queue);
#endif
	return 0;
}

int btmtk_cif_send_calibration(struct btmtk_dev *bdev)
{
	return btmtk_intcmd_wmt_calibration(bdev->hdev);
}

/* btmtk_cif_probe
 *
 *     Probe function of BT driver with BTIF HAL, initialize variable after
 *     driver installed
 *
 * Arguments:
 *     [IN] pdev    platform device pointer after driver installed
 *
 * Return Value:
 *     0 if success, otherwise error code
 */
static int btmtk_cif_probe(struct platform_device *pdev)
{
	struct btmtk_btif_dev *cif_dev = NULL;

	/* 1. allocate global context data */
	g_sbdev->cif_dev = &g_btif_dev;
	cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	cif_dev->pdev = pdev;
	g_sbdev->chip_id = 0x6631;
	if (bt_reg_init()) {
		BTMTK_ERR("%s: Error allocating memory remap");
		return -1;
	}

	/* 2. Init HCI device */
	btmtk_allocate_hci_device(g_sbdev, HCI_UART);
#if (USE_DEVICE_NODE == 0)
	SET_HCIDEV_DEV(g_sbdev->hdev, BTMTK_GET_DEV(cif_dev));
	btmtk_register_hci_device(g_sbdev);
#endif

	/* 3. Init power manager */
	bt_psm_init(&cif_dev->psm);

	/* 4. Init completion */
	init_completion(&cif_dev->rst_comp);
	init_completion(&cif_dev->internal_trx.comp);

	/* 5. Init semaphore */
	sema_init(&cif_dev->halt_sem, 1);
	sema_init(&cif_dev->internal_cmd_sem, 1);
	sema_init(&cif_dev->btif_dpidle_ctrl.sem, 1);
	sema_init(&cif_dev->cmd_tout_sem, 1);

#if SUPPORT_BT_THREAD
	/* 6. Init tx_queue of thread */
	skb_queue_head_init(&cif_dev->tx_queue);
#endif

#if SUPPORT_COREDUMP
	/* 7. Init coredump */
	cif_dev->coredump_handle = connsys_coredump_init(CONN_DEBUG_TYPE_BT, &bt_coredump_cb);
#endif

	/* 8. Register screen on/off & suspend/wakup notify callback */
	cif_dev->blank_state = WMT_PARA_SCREEN_ON;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	mtk_disp_notifier_register("btmtk_disp_notifier", &btmtk_disp_notifier);
#else
	btmtk_fb_notify_register();
#endif
	btmtk_pm_notify_register();

	/* 9. Init debug interface */
	bt_dev_dbg_init();

	/* 10. Init rst_onoff_waitq */
	init_waitqueue_head(&cif_dev->rst_onoff_waitq);

	/* 11. Init internal trx work */
	INIT_WORK(&internal_trx_work, btmtk_btif_internal_trx_work);

	/* Register callbacks to conninfra driver */
	conninfra_sub_drv_ops_register(CONNDRV_TYPE_BT, &bt_drv_cbs);
	bt_pwrctrl_register_evt();

	/* Runtime malloc patch names */
	fwp_get_patch_names();

	/* Set ICB cif state */
	btmtk_set_chip_state((void *)g_sbdev, BTMTK_STATE_WORKING);

	BTMTK_INFO("%s: Done", __func__);
	return 0;
}

/* btmtk_cif_remove
 *
 *     Remove function of BT driver with BTIF HAL, de-initialize variable after
 *     driver being removed
 *
 * Arguments:
 *     [IN] pdev    platform device pointer
 *
 * Return Value:
 *     0 if success, otherwise error code
 */
static int btmtk_cif_remove(struct platform_device *pdev)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	conninfra_sub_drv_ops_unregister(CONNDRV_TYPE_BT);

	bt_dev_dbg_deinit();

	/* Unregister screen on/off & suspend/wakup notify callback */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	mtk_disp_notifier_unregister(&btmtk_disp_notifier);
#else
	btmtk_fb_notify_unregister();
#endif
	btmtk_pm_notify_unregister();

	bt_psm_deinit(&cif_dev->psm);

#if (USE_DEVICE_NODE == 0)
	btmtk_deregister_hci_device(g_sbdev);
	btmtk_free_hci_device(g_sbdev, HCI_UART);
#endif

#if SUPPORT_COREDUMP
	if (!cif_dev->coredump_handle)
		connsys_coredump_deinit(cif_dev->coredump_handle);
#endif

	bt_reg_deinit();

	return 0;
}

/* btmtk_cif_register
 *
 *    BT driver with BTIF hal has its own device, this API should be invoked
 *    by BT main driver when installed
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 */
int btmtk_cif_register(void)
{
	int ret = -1;
	struct hif_hook_ptr hook;

	mtkbt_btif_driver.probe = btmtk_cif_probe;
	mtkbt_btif_driver.remove = btmtk_cif_remove;

	memset(&hook, 0, sizeof(struct hif_hook_ptr));
	hook.init = BT_init;
	hook.exit = BT_exit;
	hook.open = btmtk_btif_open;
	hook.close = btmtk_btif_close;
	hook.send_cmd = btmtk_btif_send_cmd_if;
	hook.send_and_recv = btmtk_btif_send_and_recv;
	hook.event_filter = btmtk_btif_event_filter;
	hook.flush = btmtk_btif_flush;
	hook.log_init = btmtk_connsys_log_init;
	hook.log_register_cb = btmtk_connsys_log_register_event_cb;
	hook.log_read_to_user = btmtk_connsys_log_read_to_user;
	hook.log_get_buf_size = btmtk_connsys_log_get_buf_size;
	hook.log_deinit = btmtk_connsys_log_deinit;
	hook.log_hold_sem = btmtk_connsys_log_hold_sem;
	hook.log_release_sem = btmtk_connsys_log_release_sem;
	btmtk_reg_hif_hook(&hook);

#if (USE_DEVICE_NODE == 1)
	ret = btmtk_cif_probe(NULL);
	if (ret)
		return -1;
	rx_queue_initialize();
#else
	ret = platform_driver_register(&mtkbt_btif_driver);
	BTMTK_INFO("platform_driver_register ret = %d", ret);

	mtkbt_btif_device = platform_device_alloc(MTKBT_BTIF_NAME, 0);
	if (mtkbt_btif_device == NULL) {
		platform_driver_unregister(&mtkbt_btif_driver);
		BTMTK_ERR("platform_device_alloc device fail");
		return -1;
	}
	ret = platform_device_add(mtkbt_btif_device);
	if (ret) {
		platform_driver_unregister(&mtkbt_btif_driver);
		BTMTK_ERR("platform_device_add fail");

		return -1;
	}
#endif
	BTMTK_INFO("%s: Done", __func__);
	return 0;
}

/* btmtk_cif_deregister
 *
 *    BT driver with BTIF hal has its own device, this API should be invoked
 *    by BT main driver when removed
 *
 * Arguments:
 *     N/A
 *
 * Return Value:
 *     0
 */
int btmtk_cif_deregister(void)
{
	btmtk_wcn_btif_close();

#if (USE_DEVICE_NODE == 1)
	rx_queue_destroy();
#else
	platform_driver_unregister(&mtkbt_btif_driver);
	platform_device_unregister(mtkbt_btif_device);
#endif

	return 0;
}

#if SUPPORT_BT_THREAD
/* bt_tx_wait_for_msg
 *
 *    Check needing action of current bt status to wake up bt thread
 *
 * Arguments:
 *    [IN] bdev     - bt driver control strcuture
 *
 * Return Value:
 *    return check  - 1 for waking up bt thread, 0 otherwise
 *
 */
static u_int8_t bt_tx_wait_for_msg(struct btmtk_dev *bdev)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	/* Ignore other cases for reset situation, and wait for close */
	if (cif_dev->bt_state == RESET_START)
		return kthread_should_stop();
	else {
		BTMTK_DBG("skb [%d], rx_ind [%d], bgf2ap_ind [%d], bt_conn2ap_ind [%d], sleep_flag [%d], wakeup_flag [%d], force_on [%d]",
				skb_queue_empty(&cif_dev->tx_queue), cif_dev->rx_ind,
				cif_dev->bgf2ap_ind, cif_dev->bt_conn2ap_ind, 
				cif_dev->psm.sleep_flag,
				cif_dev->psm.wakeup_flag,
				cif_dev->psm.force_on);
		return (!skb_queue_empty(&cif_dev->tx_queue)
			|| cif_dev->rx_ind
			|| cif_dev->bgf2ap_ind
			|| cif_dev->bt_conn2ap_ind
			|| (!cif_dev->psm.force_on && cif_dev->psm.sleep_flag) // only check sleep_flag if force_on is FALSE
			|| cif_dev->psm.wakeup_flag
			|| kthread_should_stop());
	}
}

/* btmtk_tx_thread
 *
 *    Internal bt thread handles driver's TX / wakeup / sleep flow
 *
 * Arguments:
 *    [IN] arg
 *
 * Return Value:
 *    N/A
 *
 */
int32_t btmtk_tx_thread(void * arg)
{
	struct btmtk_dev *bdev = (struct btmtk_dev *)arg;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_psm_ctrl *psm = &cif_dev->psm;
	int32_t sleep_ret = 0, wakeup_ret = 0, ret, ii;
	struct sk_buff *skb;
	int skb_len;
	char state_tag[20] = "";

	BTMTK_INFO("%s start running...", __func__);
	do {
		strncpy(state_tag, (psm->state == PSM_ST_SLEEP ? "[ST_SLEEP]" : "[ST_NORMAL]"), 15);
		BTMTK_DBG("%s -- wait_event_interruptible", state_tag);
		wait_event_interruptible(cif_dev->tx_waitq, bt_tx_wait_for_msg(bdev));
		BTMTK_DBG("%s -- wakeup", state_tag);
		if (kthread_should_stop()) {
			BTMTK_INFO("%s %s should stop now...", state_tag, __func__);
			if (psm->state == PSM_ST_NORMAL_TR)
				bt_release_wake_lock(&psm->wake_lock);
			break;
		}

		/* handling BUS SW IRQ */
		if (cif_dev->bt_conn2ap_ind && BT_SSPM_TIMER != 0) {
			bt_conn2ap_irq_handler();
			continue;
		}

		/* handling SW IRQ */
		if (cif_dev->bgf2ap_ind) {
			bt_bgf2ap_irq_handler();
			/* reset bgf2ap_ind flag move into bt_bgf2ap_irq_handler */
			continue;
		}

		switch (psm->state) {
		case PSM_ST_SLEEP:
			if (psm->sleep_flag) {
				psm->sleep_flag = FALSE;
				complete(&psm->comp);
				psm->result = sleep_ret;
				continue;
			}
			/*
			 *  TX queue has pending data to send,
			 *    or F/W pull interrupt to indicate there's data to host,
			 *    or there's a explicit wakeup request.
			 *
			 *  All need to execute the Wakeup procedure.
			 */
			btmtk_btif_dpidle_ctrl(FALSE);

			bt_disable_irq(BGF2AP_BTIF_WAKEUP_IRQ);
			wakeup_ret = btmtk_cif_fw_own_clr();
			if (wakeup_ret) {
				/*
				 * Special case handling:
				 * if FW is asserted, FW OWN clr must fail,
				 * so we can assume that FW is asserted from driver view
				 * and trigger reset directly
				 */
				bt_enable_irq(BGF2AP_BTIF_WAKEUP_IRQ);
				btmtk_btif_dpidle_ctrl(TRUE);
				/* check current bt_state to prevent from conflict
				 * resetting b/w subsys reset & whole chip reset
				 */
				if (cif_dev->bt_state == FUNC_ON) {
					BTMTK_ERR("%s FATAL: btmtk_cif_fw_own_clr error!! going to reset", state_tag);
					bt_trigger_reset();
				} else
					BTMTK_WARN("%s bt_state[%d] is not FUNC_ON, skip reset", state_tag, cif_dev->bt_state);
				break;
			} else {
				/* BGFSYS is awake and ready for data transmission */
				psm->state = PSM_ST_NORMAL_TR;
			}
			break;

		case PSM_ST_NORMAL_TR:
			if (psm->wakeup_flag) {
				psm->wakeup_flag = FALSE;
				complete(&psm->comp);
				psm->result = wakeup_ret;
			}

			if (cif_dev->rx_ind) {
				BTMTK_DBG("%s wakeup by BTIF_WAKEUP_IRQ", state_tag);
				/* Just reset the flag, F/W will send data to host after FW own clear */
				cif_dev->rx_ind = FALSE;
			}

			/*
			 *  Dequeue and send TX pending packets to bus
			 */
			 while(!skb_queue_empty(&cif_dev->tx_queue)) {
				skb = skb_dequeue(&cif_dev->tx_queue);
				if(skb == NULL)
					continue;

				/*
				 * make a copy of skb->len ot prevent skb being
				 * free after sending and recv event from FW
				 */
				skb_len = skb->len;
				// dump cr if it is fw_assert cmd
				if (skb_len >= 4 && skb->data[0] == 0x01 && skb->data[1] == 0x5B &&
				    skb->data[2] == 0xFD && skb->data[3] == 0x00) {
					kfree_skb(skb);
					skb_queue_purge(&cif_dev->tx_queue);
					bt_trigger_reset();
					break;
				}

#if (DRIVER_CMD_CHECK == 1)
				// enable driver check command timeout mechanism
				if (skb_len >= 3 && skb->data[0] == 0x01 && skb->data[1] == 0x1B &&
					skb->data[2] == 0xFD ) {
					kfree_skb(skb);
					BTMTK_INFO("%s enable check command timeout function", state_tag);
					cif_dev->cmd_timeout_check = TRUE;
					continue;
				}
#endif

				ret = btmtk_btif_send_cmd(bdev, skb, 0, 5, 0);
				kfree_skb(skb);
				if ((ret < 0) || (ret != skb_len)) {
					BTMTK_ERR("%s FATAL: TX packet error!! (%u/%d)", state_tag, skb_len, ret);
					break;
				}

				if (psm->sleep_flag) {
					BTMTK_DBG("%s more data to send and wait for event, ignore sleep", state_tag);
					psm->sleep_flag = FALSE;
				}
			}

			/*
			 *  If Quick PS mode is enabled,
			 *    or there's a explicit sleep request.
			 *
			 *  We need to excecute the Sleep procedure.
			 *
			 *  For performance consideration, donot try to enter sleep during BT func on.
			 *
			 *  [20191108]
			 *  - do not sleep if there's pending cmd waiting for event
			 *  [20191119]
			 *  - add check if btif has pending data
			 */
			if (cif_dev->bt_state == FUNC_ON && cmd_list_isempty() &&
			   psm->sleep_flag && !psm->force_on) {

				// wait if btif tx is not finish yet
				for (ii = 0; ii < 5; ii++) {
					if (mtk_btif_is_tx_complete(g_btif_id) > 0)
						break;
					else
						usleep_range(USLEEP_1MS_L, USLEEP_1MS_H);
					if (ii == 4)
						BTMTK_INFO("%s mtk_btif_is_tx_complete run 5 times", state_tag);
				}
				// re-run while loop
				if (ii == 4) {
					BTMTK_INFO("%s mtk_btif_is_tx_complete run 5 times", state_tag);
					break;
				}

				sleep_ret = btmtk_cif_fw_own_set();
				if (sleep_ret) {
					if (cif_dev->bt_state == FUNC_ON) {
						BTMTK_ERR("%s FATAL: btmtk_cif_fw_own_set error!! going to reset", state_tag);
						bt_trigger_reset();
					} else
						BTMTK_WARN("%s bt_state [%d] is not FUNC_ON, skip reset", state_tag, cif_dev->bt_state);
					break;
				} else {
					bt_enable_irq(BGF2AP_BTIF_WAKEUP_IRQ);
					btmtk_btif_dpidle_ctrl(TRUE);
					psm->state = PSM_ST_SLEEP;
				}
			}
			break;

		default:
			BTMTK_ERR("FATAL: Unknown state %d!!", psm->state);
			break;
		}
	} while(1);

	return 0;
}
#endif

/*******************************************************************************
*                   BT FW LOG F U N C T I O N S
********************************************************************************
*/

void btmtk_connsys_log_init(void)
{
	connsys_log_init(CONN_DEBUG_TYPE_BT);
}

void btmtk_connsys_log_register_event_cb(void (*func)(void))
{
	connsys_log_register_event_cb(CONN_DEBUG_TYPE_BT, func);
}

void btmtk_connsys_log_deinit(void)
{
	connsys_log_deinit(CONN_DEBUG_TYPE_BT);
}

ssize_t btmtk_connsys_log_read_to_user(char __user *buf, size_t count)
{
	return connsys_log_read_to_user(CONN_DEBUG_TYPE_BT, buf, count);
}

unsigned int btmtk_connsys_log_get_buf_size(void)
{
	return connsys_log_get_buf_size(CONN_DEBUG_TYPE_BT);
}

void btmtk_connsys_log_hold_sem(void)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	down(&cif_dev->halt_sem);
}

void btmtk_connsys_log_release_sem(void)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	up(&cif_dev->halt_sem);
}

