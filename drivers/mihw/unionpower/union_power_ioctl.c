// MIUI ADD: Power_UnionPowerCore
/*
* Copyright (C) 2024 Xiaomi Inc.
*/
// #define DEBUG
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include "unionpower.h"
int FRAME_MONITOR_ENABLE = 0; // ns
module_param(FRAME_MONITOR_ENABLE, int, 0644);
MODULE_PARM_DESC(FRAME_MONITOR_ENABLE, "FRAME_MONITOR_ENABLE");
int monitor_enable(void) {
    return FRAME_MONITOR_ENABLE == 1;
}
void (*notify_buffer_work_fp)(enum WORK_PUSH_TYPE type, int call_from, int pid, unsigned long long frame_time, unsigned long long frame_interval, unsigned long long frame_id);
struct proc_dir_entry *pe;
static int device_show(struct seq_file *m, void *v)
{
    return 0;
}
static int device_open(struct inode *inode, struct file *file)
{
    return single_open(file, device_show, inode->i_private);
}
static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    ssize_t ret = 0;
    struct _BUFFER_PACKAGE *msg = NULL, *msgUM = (struct _BUFFER_PACKAGE *)arg;
    struct _BUFFER_PACKAGE smsg;
    if (!monitor_enable()) {
        return ret;
    }
    msg = &smsg;
    if (__copy_from_user(msg, msgUM, sizeof(struct _BUFFER_PACKAGE))) {
        return EFAULT;
    }
    switch (cmd) {
    case BUFFER_QUEUE:
        if (notify_buffer_work_fp) {
            notify_buffer_work_fp(NOTIFIER_QUEUE, current->pid, msg->pid, timespec64_to_ns(&msg->current_timespec), 0, msg->frame_id);
        }
        break;
    case BUFFER_VSYNC:
        if (current->pid != msg->pid) {
            return ret;
        }
        if (notify_buffer_work_fp)
            notify_buffer_work_fp(NOTIFIER_VSYNC, current->pid, msg->pid, timespec64_to_ns(&msg->current_timespec), msg->frame_interval, msg->frame_id);
        break;
    default:
        pr_debug("%s %d: unknown cmd %x\n", __FILE__, __LINE__, cmd);
        ret = -1;
    }
    return ret;
}
static const struct proc_ops FOPS = {
    .proc_compat_ioctl = device_ioctl,
    .proc_ioctl = device_ioctl,
    .proc_open = device_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
};
int init_ioctl(struct proc_dir_entry *parent)
{
    pe = proc_create("ioctl", 0664, parent, &FOPS);
    if (unlikely(!pe)) {
        pr_debug("%s %s: Creating file node", __FILE__, __FUNCTION__);
        return -ENOMEM;
    } else {
        return 0;
    }
}
void exit_ioctl(void)
{
    if (likely(pe)) {
        proc_remove(pe);
        pe = NULL;
    }
}
// END Power_UnionPowerCore
