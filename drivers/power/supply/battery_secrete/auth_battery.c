#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include "battery_auth_class.h"

#define SECRET_IC "secret_ic="
#define BATTERY_ID "battery_id="
#define MI_AUTH_RESULT "mi_auth_result="
#define BATT_SN "batt_sn="

static const char *auth_device_name[] = {
	"unknown",
	"main_suppiler",
	"second_supplier",
	"third_supplier",
	"fourth_supplier",
};

char batt_sn[33];

enum {
	MAIN_SUPPLY = 0,
	SECEON_SUPPLY,
	THIRD_SUPPLY,
	FOURTH_SUPPLY,
	MAX_SUPPLY,
};

struct auth_data {
	struct auth_device *auth_dev[MAX_SUPPLY];

	struct power_supply *verify_psy;
	struct power_supply_desc desc;
	struct delayed_work dwork;

	bool auth_result;

	u8 battery_id;

	int secret_ic;

	u32 cycle_count;

};

static struct auth_data *g_info;

static enum power_supply_property verify_props[] = {
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};


#ifdef MODULE
static char __chg_cmdline[1024];
static char *chg_cmdline = __chg_cmdline;

const char *chg_get_cmd(void)
{
	struct device_node * of_chosen = NULL;
	char *bootargs = NULL;

	if (__chg_cmdline[0] != 0)
		return chg_cmdline;

	of_chosen = of_find_node_by_path("/chosen");
	if (of_chosen) {
		bootargs = (char *)of_get_property(
					of_chosen, "bootargs", NULL);
		if (!bootargs)
			pr_err("%s: failed to get bootargs\n", __func__);
		else {
			strncpy(__chg_cmdline, bootargs, 512);
			pr_err("%s: bootargs: %s\n", __func__, bootargs);
		}
	} else
		pr_err("%s: failed to get /chosen \n", __func__);

	return chg_cmdline;
}

#else
const char *chg_get_cmd(void)
{
	return saved_command_line;
}
#endif

int  batt_auth_get_cycle_count(void){
	int secret_ic;
	int get_cycle_count = 0;

	pr_err("%s \n", __func__);
	if (IS_ERR_OR_NULL(g_info)) {
		pr_err("g_info is null\n");
		return -1;
	}
	secret_ic = g_info->secret_ic;
	if (secret_ic == MAX_SUPPLY)
		secret_ic = 1;
	if(secret_ic && (!IS_ERR_OR_NULL(g_info->auth_dev[secret_ic]))){
		auth_device_get_cycle_count(g_info->auth_dev[secret_ic], &get_cycle_count);
		pr_err("%s get_cycle_count:%d\n", __func__, get_cycle_count);
		if (g_info->cycle_count < get_cycle_count) {
			pr_err("g_info->cycle_count:%d\n", g_info->cycle_count);
			g_info->cycle_count = get_cycle_count;
		}
		pr_err("%s g_info->cycle_count:%d\n", __func__, g_info->cycle_count);
		return g_info->cycle_count;
	}
	return -1;
}
EXPORT_SYMBOL(batt_auth_get_cycle_count);

int batt_auth_set_cycle_count(int set_count, int get_count)
{
	int ret = 0;
	int secret_ic;
	pr_err("%s \n", __func__);
	if (IS_ERR_OR_NULL(g_info)) {
		pr_err("g_info is null\n");
		return -1;
	}
	secret_ic = g_info->secret_ic;
	if (secret_ic == MAX_SUPPLY)
		secret_ic = 1;
	if(secret_ic && (g_info->auth_dev[secret_ic] != NULL)){
		ret = auth_device_set_cycle(g_info->auth_dev[secret_ic], set_count, get_count);
		pr_err("%s index:%d, set_count:%d, get_count:%d, ret:%d\n",
				__func__, secret_ic, set_count, get_count, ret);
		if (ret < 0)
			return -1;
		else
			return set_count;
	}
	return -1;
}
EXPORT_SYMBOL(batt_auth_set_cycle_count);

int batt_auth_get_raw_soh(void)
{
	int raw_soh = 0;
	int secret_ic = 0;

	pr_err("%s \n", __func__);
	if (!g_info) {
		pr_err("%s g_info is null, fail\n", __func__);
		return -1;
	}
	secret_ic = g_info->secret_ic;
	if (secret_ic == MAX_SUPPLY)
		secret_ic = 1;

	if (g_info->auth_dev[secret_ic]) {
		raw_soh = auth_device_get_raw_soh(g_info->auth_dev[secret_ic]);
		pr_err("%s index:%d, raw_soh:%d\n", __func__, secret_ic, raw_soh);
		return raw_soh;
	} else
		return -1;
}
EXPORT_SYMBOL(batt_auth_get_raw_soh);

int batt_auth_set_raw_soh(int raw_soh)
{
	int ret = 0;
	int secret_ic = 0;

	pr_err("%s \n", __func__);
	if (!g_info) {
		pr_err("%s g_info is null, fail\n", __func__);
		return -1;
	}
	secret_ic = g_info->secret_ic;
	if (secret_ic == MAX_SUPPLY)
		secret_ic = 1;

	if (g_info->auth_dev[secret_ic]) {
		ret = auth_device_set_raw_soh(g_info->auth_dev[secret_ic], raw_soh);
		pr_err("%s index:%d, raw_soh:%d, ret:%d\n", __func__, secret_ic, raw_soh, ret);
		return ret;
	} else
		return -1;
}
EXPORT_SYMBOL(batt_auth_set_raw_soh);

static int verify_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	pr_info("%s:%d\n", __func__, psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_AUTHENTIC:
		val->intval = g_info->auth_result;
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = g_info->battery_id;
		pr_info("%s: battery_id:%d\n", __func__, g_info->battery_id);
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = auth_device_name[g_info->secret_ic];
		pr_info("%s: secret_ic:%d\n", __func__, g_info->secret_ic);
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = batt_sn;
		pr_info("%s: batt_sn:%s\n", __func__, batt_sn);
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = batt_auth_get_cycle_count();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int batt_auth_get_ui_soh(u8 *ui_soh_data, int len)
{
	int ret = 0;
	int secret_ic = 0;

	if (!g_info) {
		pr_err("%s g_info is null, fail\n", __func__);
		return -1;
	}
	secret_ic = g_info->secret_ic;
	if (secret_ic == MAX_SUPPLY)
		secret_ic = 1;

	if (g_info->auth_dev[secret_ic]) {
		ret = auth_device_get_ui_soh(g_info->auth_dev[secret_ic], ui_soh_data, len);
		pr_err("%s index:%d, ret:%d\n", __func__, secret_ic, ret);
		return 0;
	} else
		return -1;
}
EXPORT_SYMBOL(batt_auth_get_ui_soh);

int batt_auth_set_ui_soh(u8 *ui_soh_data, int len, int raw_soh)
{
	int ret = 0;
	int secret_ic = 0;

	if (!g_info) {
		pr_err("%s g_info is null, fail\n", __func__);
		return -1;
	}
	secret_ic = g_info->secret_ic;
	if (secret_ic == MAX_SUPPLY)
		secret_ic = 1;

	if (g_info->auth_dev[secret_ic]) {
		ret = auth_device_set_ui_soh(g_info->auth_dev[secret_ic], ui_soh_data, len, raw_soh);
		pr_err("%s index:%d, len:%d, ret:%d\n", __func__, secret_ic, len, ret);
		return 0;
	} else
		return -1;
}
EXPORT_SYMBOL(batt_auth_set_ui_soh);

static int lc_auth_battery_probe(struct platform_device *pdev)
{
	//struct auth_data *info;
	struct power_supply_config cfg = { };
	char *rootfsmtd_ptr = NULL;
	int temp_value = 0;
	int i = 0;

	pr_info("%s enter\n", __func__);

	memset(batt_sn, '\0', 33);
	g_info = kzalloc(sizeof(*g_info), GFP_KERNEL);
	if (!g_info)

		return -ENOMEM;

	for (i = 0; i < MAX_SUPPLY; i++)
		g_info->auth_dev[i] = get_batt_auth_by_name(auth_device_name[i]);

	cfg.drv_data = g_info;
	g_info->desc.name = "batt_verify";
	g_info->desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	g_info->desc.properties = verify_props;
	g_info->desc.num_properties = ARRAY_SIZE(verify_props);
	g_info->desc.get_property = verify_get_property;
	g_info->verify_psy =
	    power_supply_register(NULL, &(g_info->desc), &cfg);
	if (!(g_info->verify_psy)) {
		pr_err("%s register verify psy fail\n", __func__);
	}

	g_info->battery_id = 0xff;
	//g_info = (struct auth_data *)malloc(sizeof(struct auth_data));
	//g_info = info;

	rootfsmtd_ptr = strstr(chg_get_cmd(), SECRET_IC);
	if (rootfsmtd_ptr) {
		sscanf(rootfsmtd_ptr, SECRET_IC"%d", &temp_value);
		g_info->secret_ic = temp_value;
		pr_err("auth_battery: Found kernel commandline option 'secret_ic=%d'\n", g_info->secret_ic);
	}

	rootfsmtd_ptr = strstr(chg_get_cmd(), BATT_SN);
	if (rootfsmtd_ptr) {
		sscanf(rootfsmtd_ptr, BATT_SN"%s", batt_sn);
		pr_err("auth_battery: Found kernel commandline option 'batt_sn=%s'\n", batt_sn);
	}

	rootfsmtd_ptr = strstr(chg_get_cmd(), BATTERY_ID);
	if (rootfsmtd_ptr) {
		sscanf(rootfsmtd_ptr, BATTERY_ID"%d", &temp_value);
		g_info->battery_id = temp_value;
		pr_err("auth_battery: Found kernel commandline option 'battery_id=%d'\n", g_info->battery_id);
	}
	pr_info("%s: battery_id:%d\n", __func__, g_info->battery_id);

	rootfsmtd_ptr = strstr(chg_get_cmd(), MI_AUTH_RESULT);
	if (rootfsmtd_ptr) {
		sscanf(rootfsmtd_ptr, MI_AUTH_RESULT"%d", &temp_value);
		g_info->auth_result = temp_value;
		pr_err("auth_battery: Found kernel commandline option 'auth_result=%d'\n", g_info->auth_result);
	}
	pr_info("%s: secret_ic:%d\n", __func__, g_info->secret_ic);

	return 0;
}

static int lc_auth_battery_remove(struct platform_device *pdev)
{
	power_supply_unregister(g_info->verify_psy);
	kfree(g_info);

	return 0;
}

static const struct of_device_id auth_battery_dt_match[] = {
	{.compatible = "lc,auth-battery"},
	{},
};

static struct platform_driver lc_auth_battery_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lc,auth-battery",
		.of_match_table = auth_battery_dt_match,
	},
	.probe = lc_auth_battery_probe,
	.remove = lc_auth_battery_remove,
};

static int __init auth_battery_init(void)
{
	pr_err("%s enter\n", __func__);
	return platform_driver_register(&lc_auth_battery_driver);
}
static void __exit auth_battery_exit(void)
{
	pr_err("%s enter\n", __func__);
	platform_driver_unregister(&lc_auth_battery_driver);
}

module_init(auth_battery_init);
module_exit(auth_battery_exit);
MODULE_LICENSE("GPL");
