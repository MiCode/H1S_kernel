// MIUI ADD: Power_UnionPowerCore
/*
* Copyright (C) 2024 Xiaomi Inc.
*/
// #define DEBUG

#include <linux/proc_fs.h>
#include "unionpower.h"

static struct proc_dir_entry *up_root;
struct kobject *union_power_kobj;
bool frame_jank = false;

ssize_t frame_jank_show(struct kobject *kobj,struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", frame_jank);
}
ssize_t frame_jank_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t n)
{
    return n;
}
load_attr(frame_jank);

struct attribute * g[] = {
    &frame_jank_attr.attr,
    NULL,
};
const struct attribute_group attr_group = {
    .attrs = g,
};
const struct attribute_group *attr_groups[] = {
    &attr_group,
    NULL,
};

void union_power_sysfs_notify(const char *attr) {
    sysfs_notify(union_power_kobj, NULL, attr);
}

int init_up_core(void)
{
    int ret = 0;
    union_power_kobj = kobject_create_and_add("unionpower", NULL);
    if (!union_power_kobj)
        return -ENOMEM;
    ret = sysfs_create_groups(union_power_kobj, attr_groups);
    if (ret)
        return -ENOMEM;
    init_frame_load_monitor();
    up_root = proc_mkdir("unionpower", NULL);
    if (unlikely(!up_root)) {
        return -EFAULT;
    }
    //init_frame_monitor();
    init_ioctl(up_root);
    return 0;
}

void exit_up_core(void)
{
    exit_ioctl();
    if (likely(up_root)) {
        proc_remove(up_root);
    }
    up_root = NULL;
    exit_frame_load_monitor();
    sysfs_remove_groups(union_power_kobj, attr_groups);
    kobject_del(union_power_kobj);
    union_power_kobj = NULL;
}

static int up_init(void)
{
    init_up_core();
    return 0;
}

static void __exit up_exit(void)
{
    exit_up_core();
}

module_init(up_init);
module_exit(up_exit);
MODULE_LICENSE("GPL");
// END Power_UnionPowerCore
