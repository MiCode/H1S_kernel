/*
 * gpio_control.c
 *
 * Copyright (C) 2024 Xiaomi Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <asm/setup.h>
#include <linux/bitops.h>
#include <linux/kobject.h>
#include <linux/gfp_types.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/dev_printk.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/sysfs.h>

#define N801_USED_GPIO_NUMS 3

enum {
  	XIAOMI_GPIO_SOC2UP_BIOS = 0,
  	XIAOMI_GPIO_UP2SOC_BIOS,
  	XIAOMI_GPIO_MCU2SOC_1PPS,
};


static const char * const xiaomi_gpio_names[] = {
    "gpio-soc2up_bios",
    "gpio-up2soc_bios",
    "gpio-mcu2soc_1pps",
};


  struct gpio_data {
    int irq;
  	int gpio_num;
  	int gpio_status;
  };


struct xiaomi_gpio_data {
    struct device *dev;
    struct gpio_data *data;
};

static ssize_t gpio_xiaomi_get_general(struct device *device, int gpio_type, char *buf){

    int value = -1;
    int gpio = 0;
    struct xiaomi_gpio_data * xiaomi_data;
    xiaomi_data = dev_get_drvdata(device);
    gpio = xiaomi_data->data[gpio_type].gpio_num;

    if(gpio_is_valid(gpio)){
        value = gpio_get_value(gpio);
    }else{
        dev_err(device, "xiaomi %s: unable to get gpio %d!\n", __func__, gpio);
    }

    dev_info(device, "%s, gpio_type [%d] get, value [%d]\n", __func__, gpio_type, value);

    return sysfs_emit(buf, "%d\n", value);
}

static ssize_t gpio_xiaomi_set_general(struct device *device, int gpio_type, const char *buf, size_t count){
    int rc = -1;
    int gpio = 0;
    struct xiaomi_gpio_data *xiaomi_data;
    xiaomi_data =dev_get_drvdata(device);
    gpio = xiaomi_data->data[gpio_type].gpio_num;
    if(!strncmp(buf, "1", strlen("1"))){
        if(gpio_is_valid(gpio)){
            rc = gpio_direction_output(gpio, 1);
            if (rc){
                dev_err(device, "xiaomi %s: fail to set gpio %d !\n", __func__, gpio);
                goto ret;
            }
        }else{
                dev_err(device, "xiaomi %s: unable to get gpio %d!\n", __func__, gpio);
        }
    }else if(!strncmp(buf, "0", strlen("0"))){
        if (gpio_is_valid(gpio)) {
                rc = gpio_direction_output(gpio, 0);
                if (rc) {
                    dev_err(device, "xiaomi %s: fail to set gpio %d !\n", __func__, gpio);
                    goto ret;
                }
        }else{
                dev_err(device, "xiaomi %s: unable to get gpio %d!\n", __func__, gpio);
        }
    }else{
            rc = -EINVAL;
            dev_err(device, "xiaomi %s: invalid input %s!only 0 or 1 is valid.\n", __func__, buf);
    }
ret:
    dev_info(device, "%s, gpio_type [%d] set, rc [%d], buf = %s\n", __func__, gpio_type, rc, buf);
    return count;
}

static ssize_t soc2up_bios_show(struct device *device, struct device_attribute *attr, char *buf)
{

    return gpio_xiaomi_get_general(device, XIAOMI_GPIO_SOC2UP_BIOS, buf);
}


static ssize_t soc2up_bios_store(struct device *device, struct device_attribute *attr, const char *buf, size_t count)
{
    return gpio_xiaomi_set_general(device, XIAOMI_GPIO_SOC2UP_BIOS, buf, count);
}

static DEVICE_ATTR_RW(soc2up_bios);

static ssize_t up2soc_bios_show(struct device *device, struct device_attribute *attr, char *buf)
{

    return gpio_xiaomi_get_general(device, XIAOMI_GPIO_UP2SOC_BIOS, buf);
}


static ssize_t up2soc_bios_store(struct device *device, struct device_attribute *attr, const char *buf, size_t count)
{
    return gpio_xiaomi_set_general(device, XIAOMI_GPIO_UP2SOC_BIOS, buf, count);
}


static DEVICE_ATTR_RW(up2soc_bios);

static ssize_t mcu2soc_bios_show(struct device *device, struct device_attribute *attr, char *buf)
{

    return gpio_xiaomi_get_general(device, XIAOMI_GPIO_MCU2SOC_1PPS, buf);
}


static ssize_t mcu2soc_bios_store(struct device *device, struct device_attribute *attr, const char *buf, size_t count)
{

    return gpio_xiaomi_set_general(device, XIAOMI_GPIO_MCU2SOC_1PPS, buf, count);
}

static DEVICE_ATTR_RW(mcu2soc_bios);


static struct attribute *g[] = {
    &dev_attr_soc2up_bios.attr,
    &dev_attr_up2soc_bios.attr,
    &dev_attr_mcu2soc_bios.attr,
    NULL,
};

static struct attribute_group attribute_group = {
	.attrs = g,
};

static int gpio_xiaomi_request_gpio(struct platform_device *pdev,
    struct xiaomi_gpio_data * xiaomi_data,const char* gpio_name){
    int idx = -1;
    int gpio_num = -1;
    int ret = 0;
    struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;

    for(int i = 0; i < N801_USED_GPIO_NUMS; i++){
        if(!strcmp(gpio_name, xiaomi_gpio_names[i])) {
  		    idx = i;
  			break;
  	    }
    }
    if(idx == -1) {
        dev_err(dev, "Invalid gpio config name: %s",gpio_name);
        return -1;
    }
    gpio_num = of_get_named_gpio(np, gpio_name, 0);
    if(gpio_num < 0){
        dev_err(dev, "Failed to get gpio %s, error: %d.\n", gpio_name, gpio_num);
    }
    ret = devm_gpio_request(dev, gpio_num, gpio_name);
    if (ret) {
        dev_err(dev, "Request gpio failed %s, error: %d.\n",  gpio_name, gpio_num);
        return ret;
    }
    dev_dbg(dev, "xiaomi_gpio %s, #%u.\n", gpio_name, gpio_num);
    xiaomi_data->data[idx].gpio_num = gpio_num;
    if((!strcmp(gpio_name,xiaomi_gpio_names[0])) || (!strcmp(gpio_name,xiaomi_gpio_names[2]))){
        gpio_direction_input(gpio_num);
    }
    if(idx == XIAOMI_GPIO_UP2SOC_BIOS){
        gpio_direction_output(gpio_num, 0);
    }
    return ret;
}


static int gpio_xiaomi_probe(struct platform_device *pdev){

    int ret = 0;
    struct device *dev = &pdev->dev;
    struct xiaomi_gpio_data *xiaomi_data;

    pr_info("%s.\n", __func__);

    xiaomi_data = devm_kzalloc(dev, sizeof(struct xiaomi_gpio_data), GFP_KERNEL);
    if(!xiaomi_data){
        dev_err(dev, "Memor allocation error!.\n");
        return -ENOMEM;
    }

    xiaomi_data->data = devm_kcalloc(dev, N801_USED_GPIO_NUMS, sizeof(struct gpio_data), GFP_KERNEL);
    if(!xiaomi_data->data ){
        dev_err(dev, "Memor allocation error!.\n");
        return -ENOMEM;
    }
    platform_set_drvdata(pdev, xiaomi_data);
    for(int i=0; i < N801_USED_GPIO_NUMS; i++){
            gpio_xiaomi_request_gpio(pdev, xiaomi_data, xiaomi_gpio_names[i]);
    }
    ret = sysfs_create_group(&dev->kobj, &attribute_group);
    if(ret < 0) {
        dev_err(dev, "Failed to create sysfs node.\n");
        return -EINVAL;
    }
    return ret;

}

static int gpio_xiaomi_remove(struct platform_device *pdev){

    return 0;
}


#if defined(CONFIG_OF)
static const struct of_device_id gpio_xiaomi_of_match[] = {
    { .compatible = "kernel,gpio-xiaomi", },
    {},
};
#endif

static struct platform_driver gpio_xiaomi_driver = {
    .driver = {
        .name = "gpio-xiaomi",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(gpio_xiaomi_of_match),
    },
    .probe = gpio_xiaomi_probe,
    .remove = gpio_xiaomi_remove,
};


static int __init gpio_xiaomi_driver_init(void){
    int err;
    pr_info("%s.\n", __func__);
    err = platform_driver_register(&gpio_xiaomi_driver);
    if (err) {
         pr_err("%s error: %d\n", __func__, err);
    }

    return err;
}
static void __exit gpio_xiaomi_driver_exit(void){
    platform_driver_unregister(&gpio_xiaomi_driver);
}


module_init(gpio_xiaomi_driver_init);
module_exit(gpio_xiaomi_driver_exit);
MODULE_AUTHOR("fengqi@xiaomi.com");
MODULE_LICENSE("GPL v2");