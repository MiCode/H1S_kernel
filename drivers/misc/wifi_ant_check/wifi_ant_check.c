/*************************************************************************
 @Author: liuqi
 @Created Time : Fri 12 Apr 2024 03:19:00 PM CST
 @File Name: wifi_ant_check.c
 @Description:
 ************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <net/sock.h>
#include <net/netlink.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <net/genetlink.h>
#include <linux/nl80211.h>
#include <linux/netdevice.h>

#define DEVICE_NAME "wifi_ant_check"
#define NETLINK_WIFI_SAR_LIMIT 25
#define MAX_MSGSIZE 1024
#define LC_WIFI_SAR_LIMIT_NETLINK_PORTID 100

#define DSI_HEAD "0"
#define DSI_BODY "1"
#define DSI_ANT "2"

static struct class *wifi_ant_check_class;
static struct device *class_dev;

static unsigned int gpio_pin;
static bool gpio_state = false;
static bool enable_state = true;
static int irq_no;

struct pm_data {
	struct device *dev;
	int debounce_time;
	struct delayed_work debounce_work;
	atomic_t pm_count;
};

struct sock *nl_sk = NULL;


static int send_sar_enable_vendor_cmd(int pid, char *dsi_choice)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int len = NLMSG_SPACE(MAX_MSGSIZE);
	int ret;

	skb = nlmsg_new(len, GFP_KERNEL);
	if (!skb) {
		pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to allocate skb!\n", __func__, __LINE__);
		return -1;
	}

	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, strlen(dsi_choice), 0);

	NETLINK_CB(skb).portid = 0;
    NETLINK_CB(skb).dst_group = 0;

	strncpy(nlmsg_data(nlh), dsi_choice, strlen(dsi_choice));
	pr_info("[WIFI_ANT_CHECK] %s-%ld: send dsi choice %s to app.\n", __func__, __LINE__, dsi_choice);

    ret = netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret < 0) {
		pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to send message to user process, ret=%d.\n", __func__, __LINE__, ret);
	}
	return 0;
}

static void recv_msg_from_userspace(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;
	int pid;
	//char *msg;

	nlh = nlmsg_hdr(skb);
	pid = nlh->nlmsg_pid; // Get client pid
	pr_info("[WIFI_ANT_CHECK] %s-%ld: Client PID=%d has been connected.\n", __func__, __LINE__, pid);

#if 0
	// TODO: move to lc_wifi_sar_client.
	// Get msg from audio(wifi sar logic)
	msg = NLMSG_DATA(nlh);

	if (strcmp(msg, "1") == 0) {
		send_sar_enable_vendor_cmd(LC_WIFI_SAR_LIMIT_NETLINK_PORTID, BDF_CHOICE_1);
	} else if (strcmp(msg, "0") == 0){
		send_sar_enable_vendor_cmd(LC_WIFI_SAR_LIMIT_NETLINK_PORTID, BDF_CHOICE_0);
	}
#endif
}

static void wifi_pm_stay_awake(struct pm_data *pm_data)
{
	if (atomic_inc_return(&pm_data->pm_count) > 1) {
		atomic_set(&pm_data->pm_count, 1);
		return;
	}

	pr_info("[WIFI_ANT_CHECK] PM stay awake, count: %d\n", atomic_read(&pm_data->pm_count));
	pm_stay_awake(pm_data->dev);
}

static void wifi_pm_relax(struct pm_data *pm_data)
{
	int r = atomic_dec_return(&pm_data->pm_count);

	WARN_ON(r < 0);

	if (r != 0)
		return;

	pr_info("[WIFI_ANT_CHECK] PM relax, count: %d\n", atomic_read(&pm_data->pm_count));
	pm_relax(pm_data->dev);
}

static void gpio_debounce_work(struct work_struct *work)
{
	int ret = -1;
	struct pm_data *pm_data =
			container_of(work, struct pm_data, debounce_work.work);

	gpio_state = gpio_get_value(gpio_pin);
    pr_info("GPIO interrupt occurred. New state: %d\n", gpio_state);

	// irq continue if enable_state == true
	if (1 == enable_state) {
		// send msg to wifi driver.
		pr_info("[WIFI_ANT_CHECK] %s-%ld: The wifi ant check function has been enabled.\n", __func__, __LINE__);
		if (1 == gpio_state) {
			ret = send_sar_enable_vendor_cmd(LC_WIFI_SAR_LIMIT_NETLINK_PORTID, DSI_ANT);
        } else {
			ret = send_sar_enable_vendor_cmd(LC_WIFI_SAR_LIMIT_NETLINK_PORTID, DSI_HEAD);
		}
	} else {
		pr_err("[WIFI_ANT_CHECK] %s-%ld: The wifi ant check function has been disabled.\n", __func__, __LINE__);
	}
    pr_info("[WIFI_ANT_CHECK] %s-%ld: Irq handler func return: %d\n", __func__, __LINE__, ret);

	wifi_pm_relax(pm_data);
}


static irqreturn_t irq_handler_func(int irq, void *irq_data)
{
	struct pm_data *pm_data = irq_data;

	pr_info("[WIFI_ANT_CHECK] irq [%d] triggered.\n", irq);

	wifi_pm_stay_awake(pm_data);

	mod_delayed_work(system_wq, &pm_data->debounce_work, msecs_to_jiffies(pm_data->debounce_time));

    return IRQ_HANDLED;
}

// read GPIO status
static ssize_t gpio_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", gpio_state);
}

// read enable status
static ssize_t enable_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", enable_state);
}

// write enable status
static ssize_t enable_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    sscanf(buf, "%du", &val);
    enable_state = val ? true : false;
    pr_info("[WIFI_ANT_CHECK] %s-%ld: Wifi ant check function state set to %d.\n",__func__, __LINE__, enable_state);
    return count;
}


// define device attributes
static DEVICE_ATTR(gpio_state, S_IRUGO, gpio_state_show, NULL);
static DEVICE_ATTR(enable_state, S_IRUGO|S_IWUSR, enable_state_show, enable_state_store);


struct netlink_kernel_cfg cfg = {
	.input = recv_msg_from_userspace,
};

static int wifi_ant_check_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct pm_data *pm_data;
	int ret;

	pm_data = devm_kzalloc(dev, sizeof(struct pm_data), GFP_KERNEL);
	if (!pm_data) {
		return -ENOMEM;
	}

	// Get wifi_ant_check node
	gpio_pin = of_get_named_gpio(np, "gpio", 0);
	if (gpio_pin < 0) {
		pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to get wifi ant check GPIO pin.\n", __func__, __LINE__);
		return gpio_pin;
	}
	if (of_property_read_u32(np, "debounce-time", &pm_data->debounce_time)) {
		pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to get debounce-time, use default.\n");
		pm_data->debounce_time = 50;
	}

	// gpio request
	ret = devm_gpio_request(dev, gpio_pin, DEVICE_NAME);
	if (ret) {
		pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to request GPIO %d.\n", __func__, __LINE__, gpio_pin);
		return ret;
	}

	// get gpio init state
	gpio_state = gpio_get_value(gpio_pin);

	// irq request
	irq_no = gpio_to_irq(gpio_pin);
	if (irq_no < 0) {
		pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to get IRQ number.\n", __func__, __LINE__);
		return irq_no;
	}

	INIT_DELAYED_WORK(&pm_data->debounce_work, gpio_debounce_work);


	// irq register
	ret = devm_request_threaded_irq(dev, irq_no, NULL, irq_handler_func, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT, DEVICE_NAME, pm_data);
	if (ret < 0) {
        pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to request IRQ.\n", __func__, __LINE__);
		class_destroy(wifi_ant_check_class);
        return ret;
    }

	ret = device_init_wakeup(dev, true);
	if (ret) {
		pr_err("Failed to configure device as wakeup %d.\n", ret);
		return ret;
	}

	ret = dev_pm_set_wake_irq(dev, irq_no);
	if (ret) {
		pr_err("Failed to set wake irq %d.\n", ret);
		return ret;
	}

    // create wifi_ant_check class
    wifi_ant_check_class = class_create(THIS_MODULE, "wifi_ant_check");
    if (IS_ERR(wifi_ant_check_class)) {
        pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to create class.\n", __func__, __LINE__);
        return PTR_ERR(wifi_ant_check_class);
    }

	// create class
	class_dev = device_create(wifi_ant_check_class, NULL, 0, NULL, DEVICE_NAME);
    if (IS_ERR(class_dev)) {
        pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to create device.\n", __func__, __LINE__);
        class_destroy(wifi_ant_check_class);
        return PTR_ERR(class_dev);
    }

	// create device attributes
    ret = device_create_file(class_dev, &dev_attr_gpio_state);
    if (ret) {
        pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to create gpio_state device file.\n", __func__, __LINE__);
        device_destroy(wifi_ant_check_class, 0);
        class_destroy(wifi_ant_check_class);
        return ret;
    }
    ret = device_create_file(class_dev, &dev_attr_enable_state);
    if (ret) {
        pr_err("[WIFI_ANT_CHECK] %s-%ld: Failed to create enable_state device file.\n", __func__, __LINE__);
        device_destroy(wifi_ant_check_class, 0);
        class_destroy(wifi_ant_check_class);
        return ret;
    }

	nl_sk = netlink_kernel_create(&init_net, NETLINK_WIFI_SAR_LIMIT, &cfg);
	if (!nl_sk) {
		pr_err("[WIFI_ANT_CHECK] %s-%ld: Can not create netlink socket error.\n", __func__, __LINE__);
		return PTR_ERR(nl_sk);
	}

	platform_set_drvdata(pdev, pm_data);
	pr_info("[WIFI_ANT_CHECK] %s-%ld: GPIO Interrupt driver initialized.\n", __func__, __LINE__);
	return 0;
}

static int wifi_ant_check_remove(struct platform_device *pdev)
{
	struct pm_data *pm_data = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&pm_data->debounce_work);
	dev_pm_clear_wake_irq(pm_data->dev);
	device_init_wakeup(&pdev->dev, false);

    // destory device attribute
    device_remove_file(class_dev, &dev_attr_gpio_state);
	device_remove_file(class_dev, &dev_attr_enable_state);
    // destory class node
    device_destroy(wifi_ant_check_class, 0);
    // destroy class
    class_destroy(wifi_ant_check_class);

	if (nl_sk)
		netlink_kernel_release(nl_sk);

    pr_info("[WIFI_ANT_CHECK] %s-%ld: GPIO interrupt driver exited.\n", __func__, __LINE__);
    return 0;
}


static const struct of_device_id wifi_ant_check_dt_ids[] = {
    { .compatible = "wifi_ant_check", },
    { /* sentinel */ },
};

static struct platform_driver wifi_ant_check_driver = {
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = of_match_ptr(wifi_ant_check_dt_ids),
    },
    .probe = wifi_ant_check_probe,
    .remove = wifi_ant_check_remove,
};
module_platform_driver(wifi_ant_check_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("iliuqi");
MODULE_DESCRIPTION("GPIO Interrupt Driver which use by wifi ant check!");
