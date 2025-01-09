// MIUI ADD: Power_PowerSaveModeV2
/*
* Copyright (C) 2024 Xiaomi Inc.
*/
// #define DEBUG
#include <linux/sysfs.h>
#include "powersave.h"

#define LIMIT_BUFFER_SIZE 128

struct kobject *ps_kobj;
static char limit_buf[128];
unsigned int boost = 0;

ssize_t limit_buf_show(struct kobject *kobj,struct kobj_attribute *attr, char *buf)
{
    return snprintf(buf, LIMIT_BUFFER_SIZE, limit_buf);
}

ssize_t limit_buf_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t n)
{
    int ret;
    ret = snprintf(limit_buf, LIMIT_BUFFER_SIZE, buf);
    pr_debug("%s %s: limit_buf: %s", __FILE__, __func__, limit_buf);
    limit_buf_update(buf, n);
    return ret;
}
load_attr(limit_buf);

ssize_t boost_show(struct kobject *kobj,struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", boost);
}

ssize_t boost_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t n)
{
    unsigned int val;
    pr_debug("%s %s: buf: %s", __FILE__, __func__, buf);
    if (kstrtouint(buf, 10, &val)) {
        pr_debug("%s %s: kstrtouint err: %d", __FILE__, __func__, kstrtouint(buf, 10, &val));
        return -EINVAL;
    }
    boost = val;
    pr_debug("%s %s: boost: %u", __FILE__, __func__, boost);
    boost_update(boost == 1);
    return n;
}
load_attr(boost);

struct attribute * g[] = {
    &boost_attr.attr,
    &limit_buf_attr.attr,
    NULL,
};
const struct attribute_group attr_group = {
    .attrs = g,
};
const struct attribute_group *attr_groups[] = {
    &attr_group,
    NULL,
};

int init_ps_core(void) {
    int ret = 0;
    ret = init_soc_policy_control();
    if (ret < 0) {
        return -ENOMEM;
    }

    ps_kobj = kobject_create_and_add("powersave", NULL);
    if (!ps_kobj) {
        pr_err("%s %s: kobject_create_and_add err",  __FILE__, __func__);
        return -ENOMEM;
    }
    ret = sysfs_create_groups(ps_kobj, attr_groups);
    if (ret) {
        pr_err("%s %s: sysfs_create_groups err: %d",  __FILE__, __func__, ret);
        return -ENOMEM;
    }

    return 0;
}

void exit_ps_core(void) {
    if (ps_kobj) {
        sysfs_remove_groups(ps_kobj, attr_groups);
        kobject_del(ps_kobj);
    }
    ps_kobj = NULL;
    exit_soc_policy_control();
}

static int ps_init(void)
{
    init_ps_core();
    return 0;
}

static void __exit ps_exit(void)
{
    exit_ps_core();
}

module_init(ps_init);
module_exit(ps_exit);
MODULE_LICENSE("GPL");
// END Power_PowerSaveModeV2
