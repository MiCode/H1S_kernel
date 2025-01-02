#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>

#include <asm/setup.h>
#include "mtk_charger.h"
#include "mtk_battery.h"
#include "tcpci_typec.h"
#include "adapter_class.h"

static int log_level = 2;

#define sysfs_err(fmt, ...)     \
do {                                    \
    if (log_level > 0)                  \
        printk(KERN_ERR "[lc_charger_sysfs]" fmt, ##__VA_ARGS__);   \
}while (0)

#define sysfs_info(fmt, ...)    \
do {                                    \
    if (log_level > 1)          \
        printk(KERN_ERR "[lc_charger_sysfs]" fmt, ##__VA_ARGS__);    \
}while (0)

#define sysfs_dbg(fmt, ...)    \
do {                                        \
    if (log_level >=2 )          \
        printk(KERN_ERR "[lc_charger_sysfs]" fmt, ##__VA_ARGS__);    \
}while (0)

#define CQR_VBAT_MIM       3750000
#define CQR_VBAT_MAX       4300000
#define CQR_READ_VBAT_TIMES 5
#define CQR_VBAT_OUT_RANGE_CNT 2
#define SHUTDOWN_DELAY_VOL 3420
#define USB_MAX_CURRENT 500000 //uA
#define MODEL_NAME "O17_5110mah_45w"

struct sysfs_desc {
	struct mtk_charger *info;
	struct mtk_battery *gm;
	struct charger_device *chg_dev;

	struct tcpc_device *tcpc;

	struct power_supply *chg_psy;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;

	struct delayed_work shutdown_dwork;

	struct notifier_block tcpc_nb;

	struct task_struct *soc_decimal_task;
	bool wakeup_thread;
	wait_queue_head_t wq;
	bool shutdown_delay;

	struct notifier_block psy_nb;
};

static struct sysfs_desc *g_desc;

static const char *const real_type_name[] = {
	"Unknown", "USB", "USB_CDP", "USB_FLOAT",
	"USB_DCP", "USB_HVDCP", "USB_PD", "USB_PPS",
};

static const char *const typec_mode_name[] = {
	"Nothing attached", "Source attached", "Sink attached",
	"Audio Adapter", "Debug Accessory",
};

static const char *const batt_type_name[] = {
	"COSMX", "SWD", "ATL",
};

static const char *const cp_vendor_name[] = {
	"Unknown", "sc8541", "nu2115",
};

enum chr_type {
	CHARGER_UNKNOWN = 0,
	STANDARD_HOST,		/* USB : 450mA */
	CHARGING_HOST,
	NONSTANDARD_CHARGER,	/* AC : 450mA~1A */
	STANDARD_CHARGER,	/* AC : ~1A */
	HVDCP_CHARGER,		/* AC: QC charger */
	PD_CAHRGER,		/* AC: PD charger */
	PPS_CAHRGER,		/* AC: PPS charger */
	APPLE_2_1A_CHARGER,	/* 2.1A apple charger */
	APPLE_1_0A_CHARGER,	/* 1A apple charger */
	APPLE_0_5A_CHARGER,	/* 0.5A apple charger */
	WIRELESS_CHARGER,
};

static int shutdown_delay;
static void shutdown_delay_handler(struct work_struct *work)
{
	struct sysfs_desc *desc;
	struct mtk_battery *gm;

	char sd_str[32];
	char *envp[] = { sd_str, NULL };

	desc = container_of(to_delayed_work(work),
			    struct sysfs_desc, shutdown_dwork);

	gm = desc->gm;

	if (gm->bs_data.bat_capacity <= 1) {
		if (gm->bs_data.bat_batt_vol <= SHUTDOWN_DELAY_VOL &&
		gm->bs_data.bat_status != POWER_SUPPLY_STATUS_CHARGING)
			shutdown_delay = true;
		else if (gm->bs_data.bat_status == POWER_SUPPLY_STATUS_CHARGING &&
		shutdown_delay)
			shutdown_delay = false;
		else
			shutdown_delay = false;
	} else
		shutdown_delay = false;

	if (desc->shutdown_delay != shutdown_delay) {
		sysfs_dbg("pre status:%d cur status: %d\n", 
				desc->shutdown_delay, shutdown_delay);

		sprintf(envp[0], "POWER_SUPPLY_SHUTDOWN_DELAY=%d",
			shutdown_delay);
		kobject_uevent_env(&desc->bat_psy->dev.kobj,
				   KOBJ_CHANGE, envp);

		desc->shutdown_delay  = shutdown_delay;
	}

	mod_delayed_work(system_wq,
			 &(desc->shutdown_dwork), msecs_to_jiffies(2000));
}

static ssize_t shutdown_delay_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return sprintf(buf, "%d\n", shutdown_delay);
}

static struct device_attribute shutdown_delay_attr =
__ATTR(shutdown_delay, 0444, shutdown_delay_show, NULL);

static void wake_up_soc_decimal_task(struct sysfs_desc *desc)
{
	desc->wakeup_thread = true;
	wake_up_interruptible(&desc->wq);
}

static int quick_chr_type;
static int last_quick_chr_type;

enum power_supply_quick_charge_type {
	QUICK_CHARGE_NORMAL = 0,		/* Charging Power <= 10W */
	QUICK_CHARGE_FAST,			/* 10W < Charging Power <= 20W */
	QUICK_CHARGE_FLASH,			/* 20W < Charging Power <= 30W */
	QUICK_CHARGE_TURBE,			/* 30W < Charging Power <= 50W */
	QUICK_CHARGE_SUPER,			/* Charging Power > 50W */
	QUICK_CHARGE_MAX,
};

static int psy_change_noti(struct notifier_block *nb,
			       unsigned long event, void *v)
{
	struct power_supply *psy = v;
	struct sysfs_desc *desc;
	char chr_type_str[64];
	char *envp[] = { chr_type_str, NULL };
	int ret = 0;

	desc = container_of(nb, struct sysfs_desc, psy_nb);

	if (event != PSY_EVENT_PROP_CHANGED || !desc || !psy)
		return NOTIFY_OK;

	pr_info("%s psy->desc->name(%s)\n", __func__, psy->desc->name);
	pr_info("%s desc->info->chr_type(%d)\n", __func__, desc->info->chr_type);
	pr_info("%s desc->info->pd_type(%d)\n", __func__, desc->info->pd_type);

        switch (desc->info->chr_type) {
	case POWER_SUPPLY_TYPE_USB:
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB_DCP:
		quick_chr_type = QUICK_CHARGE_NORMAL;
		break;
	case POWER_SUPPLY_TYPE_USB_ACA:
		quick_chr_type = QUICK_CHARGE_FAST;
		break;
	default:
		break;
	}

	if (desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO)
		quick_chr_type = QUICK_CHARGE_TURBE;

	if (desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK
		|| desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30)
			quick_chr_type = QUICK_CHARGE_FAST;

	if(desc->info->battery_temp < 5 || desc->info->battery_temp > 48)
		quick_chr_type = QUICK_CHARGE_NORMAL;

	if(quick_chr_type != last_quick_chr_type){
		sysfs_info("%s real_type(%d), batt_temp(%d), quick_charge_type(%d), last_quick_charge_type(%d) \n",
					__func__, desc->info->chr_type, desc->info->battery_temp, quick_chr_type, last_quick_chr_type);
		if (desc->info->pd_type	== MTK_PD_CONNECT_PE_READY_SNK_APDO)
			wake_up_soc_decimal_task(desc);
		else
			desc->wakeup_thread = false;

		sprintf(envp[0], "POWER_SUPPLY_QUICK_CHARGE_TYPE=%d", quick_chr_type);
		ret = kobject_uevent_env(&(desc->info->chg_psy->dev.kobj), KOBJ_CHANGE, envp);
		if (ret)
			sysfs_err("%s send uevent fail(%d)\n", __func__, ret);
		last_quick_chr_type = quick_chr_type;
	}

	return 0;
}

static int get_uisoc_decimal_rate(struct sysfs_desc *desc, int *val)
{
	static int mtk_soc_decimal_rate[24] = {0,32,10,30,20,28,30,28,40,28,50,28,60,28,70,28,80,28,90,26,95,10,99,5};
	static int *dec_rate_seq = &mtk_soc_decimal_rate[0];
	static int dec_rate_len = 24;
	int i, soc = 0;

	for (i = 0; i < dec_rate_len; i += 2) {
		if (soc < dec_rate_seq[i]) {
			*val = dec_rate_seq[i - 1];
			return soc;
		}
	}
	*val = dec_rate_seq[dec_rate_len - 1];
	return soc;
}
static void get_uisoc_decimal(struct sysfs_desc *desc, int *val)
{
	int dec_rate, soc_dec, soc, hal_soc;
	static int last_val = 0, last_soc_dec = 0, last_hal_soc = 0;

	hal_soc = desc->gm->ui_soc ;

	soc_dec = desc->gm->fg_cust_data.ui_old_soc % 100;
	soc = get_uisoc_decimal_rate(desc, &dec_rate);

	if (soc_dec >= 0 && soc_dec < (50 - dec_rate))
		*val = soc_dec + 50;
	else if (soc_dec >= (50 - dec_rate) && soc_dec < 50)
		*val = soc_dec + 50 - dec_rate;
	else
		*val = soc_dec -50;
	if (last_hal_soc == hal_soc) {
		if ((last_val > *val && hal_soc != soc) || (last_soc_dec == soc_dec && hal_soc == soc)) {
			if (last_val > 50)
				*val = last_val + (100 - last_val - dec_rate) / 2;
			else
				*val = last_val + dec_rate / 4;
		} else if (last_val > *val) {
			*val = last_val;
		}
	}
	if (last_val != *val)
		last_val = *val;
	if (last_soc_dec != soc_dec)
		last_soc_dec = soc_dec;
	if (last_hal_soc != hal_soc)
		last_hal_soc = hal_soc;
}

static int soc_decimal_threadfn(void *param)
{
	int ret;
	int soc_decimal;
	int soc_decimal_rate;
	char soc_decimal_str[64];
	char soc_decimal_rate_str[64];
	char *envp[] = {soc_decimal_str,
					soc_decimal_rate_str, NULL};
	struct sysfs_desc *desc = (struct sysfs_desc *)param;

	while(!kthread_should_stop()) {
			wait_event_interruptible(desc->wq, desc->wakeup_thread);

			get_uisoc_decimal(desc, &soc_decimal);
			get_uisoc_decimal_rate(desc, &soc_decimal_rate);

			sprintf( envp[0],"POWER_SUPPLY_SOC_DECIMAL=%d", soc_decimal);
			sprintf( envp[1],"POWER_SUPPLY_SOC_DECIMAL_RATE=%d", soc_decimal_rate);

			ret = kobject_uevent_env(&desc->bat_psy->dev.kobj,
					KOBJ_CHANGE, envp);
			if (ret < 0){
					sysfs_err("send uevent fail");

					return -1;
			}

			msleep(100);
	}
	return 0;
}

static ssize_t quick_charge_type_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	return sprintf(buf, "%d\n", quick_chr_type);
}

static struct device_attribute quick_charge_type_attr =
__ATTR(quick_charge_type, 0444, quick_charge_type_show, NULL);

static ssize_t real_type_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int chr_type = CHARGER_UNKNOWN;

	switch (g_desc->info->chr_type) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
		chr_type = CHARGER_UNKNOWN;
		break;
	case POWER_SUPPLY_TYPE_USB:
		if (g_desc->info->usb_type == POWER_SUPPLY_USB_TYPE_DCP &&
			g_desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK){
			chr_type = PD_CAHRGER;
		} else if (g_desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30 ||
			g_desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK) {
			chr_type = PD_CAHRGER;
		} else if (g_desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO){
			chr_type = PPS_CAHRGER;
		} else if (g_desc->info->usb_type == POWER_SUPPLY_USB_TYPE_DCP) {
			chr_type = NONSTANDARD_CHARGER;
		} else {
			chr_type = STANDARD_HOST;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		chr_type = CHARGING_HOST;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		chr_type = STANDARD_CHARGER;
		if (g_desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30 ||
			g_desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK) {
			chr_type = PD_CAHRGER;
		} else if (g_desc->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO) {
			chr_type = PPS_CAHRGER;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_ACA:
		chr_type = HVDCP_CHARGER;
		break;
	default:
		chr_type = NONSTANDARD_CHARGER;
	}

	g_desc->info->real_type = chr_type;

	sysfs_info("real_type(%d) %s\n", chr_type, real_type_name[chr_type]);

	return sprintf(buf, "%s\n", real_type_name[chr_type]);
}

static struct device_attribute real_type_attr =
__ATTR(real_type, 0444, real_type_show, NULL);

#if IS_ENABLED(CONFIG_TCPC_CLASS)
static int typec_cc_orientation_handler(struct tcpc_device *tcpc)
{
	int typec_cc_orientation = 0;

	tcpci_get_cc(tcpc);

	if (typec_get_cc1() == 0 && typec_get_cc2() == 0)
		typec_cc_orientation = 0;
	else if (typec_get_cc2() == 0)
		typec_cc_orientation = 1;
	else if (typec_get_cc1() == 0)
		typec_cc_orientation = 2;

	return typec_cc_orientation;
}

static ssize_t typec_cc_orientation_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct sysfs_desc *desc;
	struct power_supply *psy;

	psy = dev_get_drvdata(dev);
	desc = power_supply_get_drvdata(psy);

	return sprintf(buf, "%d\n",
		       typec_cc_orientation_handler(desc->tcpc));
}

static struct device_attribute typec_cc_orientation_attr =
__ATTR(typec_cc_orientation, 0444, typec_cc_orientation_show, NULL);

static ssize_t typec_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sysfs_desc *desc;
	struct power_supply *psy;
	struct tcpc_device *tcpc;

	psy = dev_get_drvdata(dev);
	desc = power_supply_get_drvdata(psy);
	tcpc = desc->tcpc;
	if (tcpc->typec_attach_new > ARRAY_SIZE(typec_mode_name))
		return sprintf(buf, "%s\n", "Unknown");
	return sprintf(buf, "%s\n",
		       typec_mode_name[tcpc->typec_attach_new]);
}

static struct device_attribute typec_mode_attr =
__ATTR(typec_mode, 0444, typec_mode_show, NULL);
#endif				/* CONFIG_TCPC_CLASS */

static ssize_t usb_otg_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int res = 0;
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	struct tcpc_device *tcpc;

	tcpc = g_desc->tcpc;

	if (tcpc->typec_attach_new == TYPEC_ATTACHED_SRC)
		res = 1;
#endif

	return sprintf(buf, "%d\n", res);
}

static struct device_attribute usb_otg_attr =
__ATTR(usb_otg, 0444, usb_otg_show, NULL);

static ssize_t shipmode_count_reset_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_desc->info->ship_mode);
}

static ssize_t shipmode_count_reset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int i = 0;
	struct power_supply *psy;
	union power_supply_propval val;
	int shipmode_cnt = 0;

	if (kstrtobool(buf, &g_desc->info->ship_mode)) {
		sysfs_err("parsing number fail\n");
		return -EINVAL;
	}

	sysfs_info("%s before %d\n", __func__, g_desc->info->ship_mode);

	psy = power_supply_get_by_name("battery");
	if (psy) {
		for (i = 0; i < CQR_READ_VBAT_TIMES; i++) {
			ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
			if (ret) {
				sysfs_err("get vbat fail\n");
				continue;
			}

			msleep(1);

			if (val.intval < CQR_VBAT_MIM || val.intval > CQR_VBAT_MAX) {
				shipmode_cnt++;
				sysfs_info("%s %d %d %d %d %d\n", 
					__func__, g_desc->info->ship_mode, val.intval, CQR_VBAT_MAX, CQR_VBAT_MIM, shipmode_cnt);
			}
		}
	}

	if (shipmode_cnt >= CQR_VBAT_OUT_RANGE_CNT) {
		g_desc->info->ship_mode = false;
		sysfs_info("%s after %d\n", __func__, g_desc->info->ship_mode);
		return -EINVAL;
	}

	return count;
}

static struct device_attribute shipmode_count_reset_attr =
__ATTR(shipmode_count_reset, 0644, shipmode_count_reset_show, shipmode_count_reset_store);

static ssize_t input_suspend_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	bool enable = false;

	enable = g_desc->info->cmd_discharging;

	return sprintf(buf, "%d\n", enable);
}

static ssize_t input_suspend_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	bool enable = false;

	if (kstrtobool(buf, &enable)) {
		sysfs_err("parsing number fail\n");
		return -EINVAL;
	}

	if (enable == 1) {
		g_desc->info->cmd_discharging = true;
		charger_dev_enable(g_desc->info->chg1_dev, false);
		charger_dev_set_input_current(g_desc->info->chg1_dev, 100000);
		charger_dev_do_event(g_desc->info->chg1_dev,
			EVENT_DISCHARGE, 0);
	} else if (enable == 0) {
		g_desc->info->cmd_discharging = false;
		charger_dev_enable(g_desc->info->chg1_dev, true);
		charger_dev_do_event(g_desc->info->chg1_dev,
			EVENT_RECHARGE, 0);
	}

	sysfs_err("input_suspend_store = %d\n", g_desc->info->cmd_discharging);

	/* notifier to mtk battery */
	power_supply_changed(g_desc->info->psy1);

	return count;
}

static struct device_attribute input_suspend_attr =
__ATTR(input_suspend, 0644, input_suspend_show, input_suspend_store);

static ssize_t chip_ok_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	bool chip_ok_status = false;
	union power_supply_propval val;
	struct power_supply *psy = 
			power_supply_get_by_name("batt_verify");

	if (psy) {
		ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_AUTHENTIC, &val);
		if (ret < 0)
			chip_ok_status = false;

		if (val.intval)
			chip_ok_status = true;
		else
			chip_ok_status = false;
	}
	else
		chip_ok_status = false;

	return sprintf(buf, "%d\n", chip_ok_status);
}

static struct device_attribute chip_ok_attr =
__ATTR(chip_ok, 0444, chip_ok_show, NULL);

static ssize_t soh_sn_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	char sn[33];
	union power_supply_propval val;
	struct power_supply *psy = 
			power_supply_get_by_name("batt_verify");

	memset(sn, '\0', 33);
	if (psy) {
		ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_MODEL_NAME, &val);
		pr_err("soh_sn %s\n", val.strval);

		if (ret < 0 || IS_ERR_OR_NULL(val.strval)){
			goto error;
		}else{
			strcpy(sn, val.strval);
			pr_err("soh_sn %s\n", val.strval);
		}
	}else{
		pr_err("%s get batt_verify error\n", __func__);
	}
error:
pr_err("%s get batt_verify error\n", __func__);
	return sprintf(buf, "%s\n", sn);
}

static struct device_attribute soh_sn_attr =
__ATTR(soh_sn, 0444, soh_sn_show, NULL);

static ssize_t calc_rvalue_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char calc_rvalue[33];
	memset(calc_rvalue, '0', 33);
	calc_rvalue[32] = '\0';
	return sprintf(buf, "%s\n", calc_rvalue);
}

static struct device_attribute calc_rvalue_attr =
__ATTR(calc_rvalue, 0444, calc_rvalue_show, NULL);

static ssize_t batt_id_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 id = 0;
	struct power_supply *psy;
	union power_supply_propval val;

	psy = power_supply_get_by_name("batt_verify");
	if (psy) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_SCOPE,
					&val);
		if (ret)
			id = 0xff;

		id = val.intval;
	}
	else
		id = 0xff;

	return sprintf(buf, "%d\n", id);
}

static struct device_attribute batt_id_attr =
__ATTR(batt_id, 0444, batt_id_show, NULL);

static ssize_t battery_type_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 id = 0;
	struct power_supply *psy;
	union power_supply_propval val;

	psy = power_supply_get_by_name("batt_verify");
	if (psy) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_SCOPE,
					&val);
		if (ret)
			id = 0xff;

		id = val.intval;
	} else {
		id = 0xff;
	}

	if (id < 0 || id > 2)
		return sprintf(buf, "%s\n", "Unknown");

	return sprintf(buf, "%s\n", batt_type_name[id]);
}

static struct device_attribute battery_type_attr =
__ATTR(battery_type, 0444, battery_type_show, NULL);

static ssize_t model_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", MODEL_NAME);
}

static struct device_attribute model_name_attr =
__ATTR(model_name, 0444, model_name_show, NULL);

static ssize_t RSOC_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;
	int rsoc;
	struct power_supply *bat_psy;

	bat_psy= power_supply_get_by_name("battery");
	if (!bat_psy) {
		sysfs_err("%s get battery psy fail\n", __func__);
		return -PTR_ERR(bat_psy);
	}
	gm = power_supply_get_drvdata(bat_psy);
	if (gm == NULL) {
		sysfs_err("[%s]can not get gm\n", __func__);
		rsoc = 50;
		return sprintf(buf, "%d\n", rsoc);
	}
	rsoc = gm->soc;
	if (rsoc > 100) {
		rsoc = 100;
	} else if (rsoc < 0) {
		rsoc = 0;
	}
	sysfs_info("%s: %d\n",
		__func__, rsoc);
	return sprintf(buf, "%d\n", rsoc);
}

static struct device_attribute RSOC_attr =
__ATTR(RSOC, 0444, RSOC_show, NULL);

static ssize_t soh_show(
       struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;
	int soh;
	int ret;
	int raw_soh;
	int cycle_count;
	struct power_supply *bat_psy;

	bat_psy= power_supply_get_by_name("battery");
	if (!bat_psy) {
		sysfs_err("%s get battery psy fail\n", __func__);
		return -PTR_ERR(bat_psy);
	}
	gm = power_supply_get_drvdata(bat_psy);
	if (gm == NULL) {
		sysfs_err("[%s]can not get gm\n", __func__);
		soh = 100;
		return sprintf(buf, "%d\n", soh);
	}
	soh = ((gm->aging_factor + 99) / 100);
	if (soh > 100) {
		soh = 100;
	} else if (soh < 0) {
		soh = 0;
	}
	raw_soh = batt_auth_get_raw_soh();
	cycle_count = batt_auth_get_cycle_count();
	if ((cycle_count < 200 && raw_soh < 95) || (cycle_count < 2500 && raw_soh < 75)) {
		pr_err("%s raw_soh=%d\n", __func__, raw_soh);
		raw_soh = soh;
		ret = batt_auth_set_raw_soh(raw_soh);
		if (ret < 0) {
			pr_err("%s raw_soh calibration failed\n", __func__);
		}
	}
	if (raw_soh < 0) {
		sysfs_err("%s get_raw_soh fail\n", __func__);
		gm->soh = soh;
	} else {
		gm->soh = soh < raw_soh ? soh : raw_soh;
		sysfs_err("%s soh:%d, raw_soh:%d\n", __func__, soh, raw_soh);
	}
	sysfs_info("%s:soh: %d\n",
		__func__, gm->soh);
	return sprintf(buf, "%d\n", gm->soh);
}

static struct device_attribute soh_attr =
__ATTR(soh, 0444, soh_show, NULL);

static ssize_t soh_new_show(
       struct device *dev, struct device_attribute *attr, char *buf)
{
       int soh;
       struct mtk_battery *gm;
       struct power_supply *bat_psy;

       bat_psy= power_supply_get_by_name("battery");
       if (!bat_psy) {
            sysfs_err("%s get battery psy fail\n", __func__);
            return -PTR_ERR(bat_psy);
       }
       gm = power_supply_get_drvdata(bat_psy);
       if (gm == NULL) {
            soh = 0;
            sysfs_err("[%s]can not get gm\n", __func__);
            return sprintf(buf, "%d\n", soh);
       }

       soh = gm->soh;

       sysfs_info("%s:soh new: %d\n", __func__, soh);
       return sprintf(buf, "%d\n", soh);
}

static struct device_attribute soh_new_attr =
__ATTR(soh_new, 0444, soh_new_show, NULL);

static ssize_t apdo_max_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	if (g_desc->info->pd_type != MTK_PD_CONNECT_PE_READY_SNK_APDO)
		return sprintf(buf, "%d\n", 0);
	return sprintf(buf, "%d\n", g_desc->info->pd_adapter->apdo_max / 1000000);
}
static struct device_attribute apdo_max_attr =
__ATTR(apdo_max, 0444, apdo_max_show, NULL);

static ssize_t pd_verifed_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_desc->info->pd_adapter->verifed);
}
static struct device_attribute pd_verifed_attr =
__ATTR(pd_verifed, 0444, pd_verifed_show, NULL);

static ssize_t mtbf_current_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val)) {
		sysfs_err("get buf error %s\n", buf);
		return -EINVAL;
	}
        g_desc->info->mtbf_current = val;
        sysfs_info("set mtbf_current = %dmA\n", val);

	return count;
}

static ssize_t mtbf_current_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_desc->info->mtbf_current);
}
static struct device_attribute mtbf_current_attr =
__ATTR(mtbf_current, 0644, mtbf_current_show, mtbf_current_store);


static ssize_t ffc_disable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val)) {
		sysfs_err("get buf error %s\n", buf);
		return -EINVAL;
	}
	if (val)
		g_desc->info->bat.ffc_disable = true;
	else
		g_desc->info->bat.ffc_disable = false;
	sysfs_info("set ffc_disable = %d\n", val);

	return count;
}

static ssize_t ffc_disable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_desc->info->bat.ffc_disable);
}
static struct device_attribute ffc_disable_attr =
__ATTR(ffc_disable, 0644, ffc_disable_show, ffc_disable_store);

static ssize_t cp_vendor_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct power_supply *cp_psy = NULL;
	int cp_id = 0;

	cp_psy = power_supply_get_by_name("sc-cp-standalone");
	if (IS_ERR_OR_NULL(cp_psy)) {
		cp_psy = power_supply_get_by_name("nu2115-standalone");
		if (IS_ERR_OR_NULL(cp_psy)) {
			chr_err("%s cp psy fail\n", __func__);
			cp_id = 0;
		}else{
			cp_id = 2;
		}
	} else {
		cp_id = 1;
	}

	return sprintf(buf, "%s\n", cp_vendor_name[cp_id]);
}
static struct device_attribute cp_vendor_attr =
__ATTR(cp_vendor, 0444, cp_vendor_show, NULL);

static ssize_t main_chg_vendor_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "Mediatek");
}
static struct device_attribute main_chg_vendor_attr =
__ATTR(main_chg_vendor, 0444, main_chg_vendor_show, NULL);

static ssize_t cp_bus_voltage_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int vbus_total = 0;
	struct power_supply *psy;
	union power_supply_propval val;

	psy = power_supply_get_by_name("mtk-master-charger");
	if (psy) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&val);
		if (ret)
			vbus_total = 0;

		vbus_total = val.intval;
	} else {
		vbus_total = 0;
	}

	return sprintf(buf, "%d\n", vbus_total);
}
static struct device_attribute cp_bus_voltage_attr =
__ATTR(cp_bus_voltage, 0444, cp_bus_voltage_show, NULL);

static ssize_t cp_bus_current_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int cp_ibus = 0;
	struct power_supply *psy;
	union power_supply_propval val;
	psy = power_supply_get_by_name("sc-cp-standalone");
	if (IS_ERR_OR_NULL(psy)) {
		psy = power_supply_get_by_name("nu2115-standalone");
		if (IS_ERR_OR_NULL(psy)) {
			sysfs_err("%s cp psy fail\n", __func__);
			return 0;
		} else {
			ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val); //ma
			cp_ibus = val.intval;
		}
	} else {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val); //ma
		cp_ibus = val.intval;
	}
	return sprintf(buf, "%d\n", cp_ibus);
}
static struct device_attribute cp_bus_current_attr =
__ATTR(cp_bus_current, 0444, cp_bus_current_show, NULL);

static ssize_t resistance_id_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 500000);
}
static struct device_attribute resistance_id_attr =
__ATTR(resistance_id, 0444, resistance_id_show, NULL);

static int psy_bms_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int ret = 0;
	psy = power_supply_get_by_name("batt_verify");

	if (IS_ERR_OR_NULL(psy)) {
		pr_err("%s get batt_verify error\n", __func__);
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_AUTHENTIC:
		ret = power_supply_get_property(psy, psp, val);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct attribute *bms_psy_attrs[] = {
	&soh_attr.attr,
	&soh_new_attr.attr,
	NULL,
};

static struct attribute *usb_psy_attrs[] = {
	&quick_charge_type_attr.attr,
	&real_type_attr.attr,
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	&typec_cc_orientation_attr.attr,
	&typec_mode_attr.attr,
#endif				/* CONFIG_TCPC_CLASS */
	&usb_otg_attr.attr,
	&apdo_max_attr.attr,
	&pd_verifed_attr.attr,
	&mtbf_current_attr.attr,
	&ffc_disable_attr.attr,
	&cp_vendor_attr.attr,
	&main_chg_vendor_attr.attr,
	&cp_bus_voltage_attr.attr,
	&cp_bus_current_attr.attr,
	NULL,
};

static struct attribute *bat_psy_attrs[] = {
	&shutdown_delay_attr.attr,
	&input_suspend_attr.attr,
	&chip_ok_attr.attr,
	&batt_id_attr.attr,
	&shipmode_count_reset_attr.attr,
	&battery_type_attr.attr,
	&model_name_attr.attr,
	&RSOC_attr.attr,
	&resistance_id_attr.attr,
	&mtbf_current_attr.attr,
	&soh_sn_attr.attr,
	&calc_rvalue_attr.attr,
	NULL,
};

static const struct attribute_group usb_psy_group = {
	.attrs = usb_psy_attrs,
};

static const struct attribute_group bat_psy_group = {
	.attrs = bat_psy_attrs,
};

static const struct attribute_group bms_psy_group = {
	.attrs = bms_psy_attrs,
};

static int sysfs_setup_files(struct sysfs_desc *desc)
{
	int ret;

	if (!desc->usb_psy || !desc->bat_psy || !desc->info) {
		sysfs_err("%s find psy fail\n", __func__);
		ret = -EINVAL;
		goto _out;
	}

	ret = sysfs_create_group(&(desc->bat_psy->dev.kobj),
				 &bat_psy_group);
	if (ret) {
		sysfs_err("%s create battery node fail(%d)\n",
			  __func__, ret);
		goto _out;
	}

	ret = sysfs_create_group(&(desc->usb_psy->dev.kobj),
				 &usb_psy_group);
	if (ret) {
		sysfs_err("%s create usb node fail(%d)\n", __func__, ret);
		goto _out;
	}

	ret = sysfs_create_group(&(desc->bms_psy->dev.kobj),
				 &bms_psy_group);
	if (ret) {
		sysfs_err("%s create bms node fail(%d)\n", __func__, ret);
		goto _out;
	}

	return 0;

_out:
	return ret;
}

static enum power_supply_property usb_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int psy_usb_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	//struct sysfs_desc *desc = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (IS_ERR_OR_NULL(g_desc) || IS_ERR_OR_NULL(g_desc->info->chg1_dev))
			val->intval = 0;
		else
			val->intval = charger_dev_get_online(g_desc->info->chg1_dev);
		sysfs_err("%s online=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = psy->desc->type;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = USB_MAX_CURRENT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

struct power_supply_desc usb_psy_desc = {
	.name = "usb",
	.properties = usb_psy_properties,
	.num_properties = ARRAY_SIZE(usb_psy_properties),
	.type = POWER_SUPPLY_TYPE_USB,
	.get_property = psy_usb_get_property,
};

static int init_psy_tcpc(struct sysfs_desc *desc)
{
	struct power_supply_config cfg = {
		.drv_data = desc,
	};

	desc->usb_psy = power_supply_register(&(desc->info->pdev->dev),
					      &usb_psy_desc, &cfg);
	if (IS_ERR(desc->usb_psy)) {
		sysfs_err("%s register usb psy fail(%d)\n", __func__,
			  PTR_ERR(desc->usb_psy));
		return -PTR_ERR(desc->usb_psy);
	}

	desc->bat_psy = power_supply_get_by_name("battery");
	if (!desc->bat_psy) {
		sysfs_err("%s get battery psy fail\n", __func__);
		return -PTR_ERR(desc->bat_psy);
	}

	desc->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!desc->tcpc) {
		sysfs_err("%s get typec device fail\n", __func__);
		return -PTR_ERR(desc->tcpc);
	}

	desc->gm = power_supply_get_drvdata(desc->bat_psy);
	if (!desc->gm) {
		sysfs_err("%s get battery info fail\n", __func__);
		return -PTR_ERR(desc->gm);
	}

	return 0;
}

static enum power_supply_property bms_psy_properties[] = {
	POWER_SUPPLY_PROP_AUTHENTIC,
};
struct power_supply_desc bms_psy_desc = {
	.name = "bms",
	.properties = bms_psy_properties,
	.num_properties = ARRAY_SIZE(bms_psy_properties),
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property = psy_bms_get_property,
};

static int init_psy_bms(struct sysfs_desc *desc)
{
	struct power_supply_config cfg = {
		.drv_data = desc,
	};

	desc->bms_psy = power_supply_register(&(desc->info->pdev->dev),
					      &bms_psy_desc, &cfg);
	if (IS_ERR(desc->bms_psy)) {
		sysfs_err("%s register bms psy fail(%d)\n", __func__,
			  PTR_ERR(desc->bms_psy));
		return -PTR_ERR(desc->bms_psy);
	}
	return 0;
}

static int __init lc_charger_sysfs_init(void)
{
	int ret;
	struct sysfs_desc *desc;
	struct power_supply *main_psy;

	sysfs_info("%s\n", __func__);

	desc = kzalloc(sizeof(struct sysfs_desc), GFP_KERNEL);
	if (!desc) {
		sysfs_err("%s alloc desc mem fail\n", __func__);
		ret = -ENOMEM;
		goto alloc_err;
	}

	main_psy = power_supply_get_by_name("mtk-master-charger");
	if (!main_psy) {
		sysfs_err("%s get main charger psy fail\n", __func__);
		ret = -EINVAL;
		goto main_psy_err;
	}

	desc->info = power_supply_get_drvdata(main_psy);

	ret = init_psy_tcpc(desc);
	if (ret)
		goto psy_tcpc_err;

	ret = init_psy_bms(desc);
	if (ret)
		goto psy_bms_err;
	INIT_DELAYED_WORK(&(desc->shutdown_dwork), shutdown_delay_handler);
	schedule_delayed_work(&(desc->shutdown_dwork),
			      msecs_to_jiffies(2000));

	ret = sysfs_setup_files(desc);
	if (ret)
		goto sysfs_setup_err;

	desc->psy_nb.notifier_call = psy_change_noti;
	ret = power_supply_reg_notifier(&desc->psy_nb);
	if (ret < 0) {
		sysfs_err("%s register psy notifier fail(%d)\n", __func__,
			  ret);
		ret = -EINVAL;
		goto main_psy_err;
	}

	desc->wakeup_thread = false;
	init_waitqueue_head(&desc->wq);
	desc->soc_decimal_task = kthread_run(soc_decimal_threadfn, desc,
							"soc_decimal_task");
	if (IS_ERR(desc->soc_decimal_task)) {
			ret = -PTR_ERR(desc->soc_decimal_task);
			sysfs_err("%s run task fail(%d)\n", __func__, ret);
	}
	sysfs_info("%s init success\n", __func__);
	g_desc = desc;

	return 0;

sysfs_setup_err:
	cancel_delayed_work(&(desc->shutdown_dwork));
psy_tcpc_err:
psy_bms_err:
main_psy_err:
	kfree(desc);
alloc_err:
	return ret;
}

static void __exit lc_charger_sysfs_exit(void)
{

}

late_initcall_sync(lc_charger_sysfs_init);
module_exit(lc_charger_sysfs_exit);

MODULE_LICENSE("GPL");
