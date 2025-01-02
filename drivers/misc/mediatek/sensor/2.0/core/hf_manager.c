// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#define pr_fmt(fmt) "[hf_manager]" fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/bitmap.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched_clock.h>
#include <linux/log2.h>

/*awinic bob add start*/
#include <linux/vmalloc.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/time.h>
/*awinic bob add end*/

/* awinic bob add start */
//#define AW_USB_PLUG_CAIL

#ifdef AW_USB_PLUG_CAIL
#include <linux/vmalloc.h>
#include <linux/notifier.h>
#include <linux/usb.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
#define USB_POWER_SUPPLY_NAME   "charger"
#else
#define USB_POWER_SUPPLY_NAME   "usb"
#endif

#define AW_SAR_CONFIG_MTK_CHARGER

#endif
/* awinic bob add end */

#include "hf_manager.h"

extern void set_lct_tp_proximity_switch_status(bool en);//TP调用虚拟距感统一接口

static int major;
static struct class *hf_manager_class;
static struct task_struct *task;

/*awinic bob add start*/
static volatile uint32_t awinic_debug_data[3] = { 0 };
static wait_queue_head_t g_aw_sar_wait;
static struct hf_device *aw_g_device = NULL;
struct mutex g_aw_sar_lock;

/*awinic bob add end*/

struct coordinate {
    int8_t sign[3];
    uint8_t map[3];
};

static const struct coordinate coordinates[] = {
    { { 1, 1, 1}, {0, 1, 2} },
    { { -1, 1, 1}, {1, 0, 2} },
    { { -1, -1, 1}, {0, 1, 2} },
    { { 1, -1, 1}, {1, 0, 2} },

    { { -1, 1, -1}, {0, 1, 2} },
    { { 1, 1, -1}, {1, 0, 2} },
    { { 1, -1, -1}, {0, 1, 2} },
    { { -1, -1, -1}, {1, 0, 2} },
};

static DECLARE_BITMAP(sensor_list_bitmap, SENSOR_TYPE_SENSOR_MAX);
static struct hf_core hfcore;

#define print_s64(l) (((l) == S64_MAX) ? -1 : (l))
static int hf_manager_find_client(struct hf_core *core,
        struct hf_manager_event *event);

static void init_hf_core(struct hf_core *core)
{
    int i = 0;

    mutex_init(&core->manager_lock);
    INIT_LIST_HEAD(&core->manager_list);
    for (i = 0; i < SENSOR_TYPE_SENSOR_MAX; ++i) {
        core->state[i].delay = S64_MAX;
        core->state[i].latency = S64_MAX;
        core->state[i].start_time = S64_MAX;
    }

    spin_lock_init(&core->client_lock);
    INIT_LIST_HEAD(&core->client_list);

    mutex_init(&core->device_lock);
    INIT_LIST_HEAD(&core->device_list);

    kthread_init_worker(&core->kworker);
}

void coordinate_map(unsigned char direction, int32_t *data)
{
    int32_t temp[3] = {0};

    if (direction >= ARRAY_SIZE(coordinates))
        return;

    temp[coordinates[direction].map[0]] =
        coordinates[direction].sign[0] * data[0];
    temp[coordinates[direction].map[1]] =
        coordinates[direction].sign[1] * data[1];
    temp[coordinates[direction].map[2]] =
        coordinates[direction].sign[2] * data[2];

    data[0] = temp[0];
    data[1] = temp[1];
    data[2] = temp[2];
}
EXPORT_SYMBOL_GPL(coordinate_map);

static bool filter_event_by_timestamp(struct hf_client_fifo *hf_fifo,
        struct hf_manager_event *event)
{
    if (hf_fifo->last_time_stamp[event->sensor_type] ==
            event->timestamp) {
        return true;
    }
    hf_fifo->last_time_stamp[event->sensor_type] = event->timestamp;
    return false;
}

static int hf_manager_report_event(struct hf_client *client,
        struct hf_manager_event *event)
{
    unsigned long flags;
    unsigned int next = 0;
    int64_t hang_time = 0;
    const int64_t max_hang_time = 1000000000LL;
    struct hf_client_fifo *hf_fifo = &client->hf_fifo;

    spin_lock_irqsave(&hf_fifo->buffer_lock, flags);
    if (unlikely(hf_fifo->buffull == true)) {
        hang_time = ktime_get_boottime_ns() - hf_fifo->hang_begin;
        if (hang_time >= max_hang_time) {
            /* reset buffer */
            hf_fifo->buffull = false;
            hf_fifo->head = 0;
            hf_fifo->tail = 0;
            pr_err_ratelimited("[%s][%d:%d] buffer reset %lld\n",
                client->proc_comm, client->leader_pid,
                client->ppid, hang_time);
        } else {
            pr_err_ratelimited("[%s][%d:%d] buffer full %d %lld\n",
                client->proc_comm, client->leader_pid,
                client->ppid, event->sensor_type,
                event->timestamp);
            spin_unlock_irqrestore(&hf_fifo->buffer_lock, flags);
            wake_up_interruptible(&hf_fifo->wait);
            /*
             * must return -EAGAIN when buffer full,
             * tell caller retry send data some times later.
             */
            return -EAGAIN;
        }
    }
    /* only data action run filter event */
    if (likely(event->action == DATA_ACTION) &&
            unlikely(filter_event_by_timestamp(hf_fifo, event))) {
        pr_err_ratelimited("[%s][%d:%d] buffer filter %d %lld\n",
            client->proc_comm, client->leader_pid,
            client->ppid, event->sensor_type, event->timestamp);
        spin_unlock_irqrestore(&hf_fifo->buffer_lock, flags);
        /*
         * must return 0 when timestamp filtered, tell caller data
         * already in buffer, don't need send again.
         */
        return 0;
    }



    hf_fifo->buffer[hf_fifo->head++] = *event;
    hf_fifo->head &= hf_fifo->bufsize - 1;
    /* remain 1 count */
    next = hf_fifo->head + 1;
    next &= hf_fifo->bufsize - 1;
    if (unlikely(next == hf_fifo->tail)) {
        hf_fifo->buffull = true;
        if (hf_fifo->hang_begin > hf_fifo->client_active) {
            hang_time = hf_fifo->hang_begin -
                hf_fifo->client_active;
            if (hang_time < max_hang_time)
                hf_fifo->hang_begin = ktime_get_boottime_ns();
        } else {
            hf_fifo->hang_begin = ktime_get_boottime_ns();
        }
    }
    spin_unlock_irqrestore(&hf_fifo->buffer_lock, flags);

    wake_up_interruptible(&hf_fifo->wait);
    return 0;
}

static void hf_manager_io_schedule(struct hf_manager *manager,
        int64_t timestamp)
{
    if (!atomic_read(&manager->io_enabled))
        return;
    set_interrupt_timestamp(manager, timestamp);
    if (READ_ONCE(manager->hf_dev->device_bus) == HF_DEVICE_IO_ASYNC)
        tasklet_schedule(&manager->io_work_tasklet);
    else if (READ_ONCE(manager->hf_dev->device_bus) == HF_DEVICE_IO_SYNC)
        kthread_queue_work(&manager->core->kworker,
            &manager->io_kthread_work);
}

static int hf_manager_io_report(struct hf_manager *manager,
        struct hf_manager_event *event)
{
    /* must return 0 when sensor_type exceed and no need to retry */
    if (unlikely(event->sensor_type >= SENSOR_TYPE_SENSOR_MAX)) {
        pr_err_ratelimited("Report failed, %u exceed max\n",
            event->sensor_type);
        return 0;
    }
    return hf_manager_find_client(manager->core, event);
}

static void hf_manager_io_complete(struct hf_manager *manager)
{
    clear_bit(HF_MANAGER_IO_IN_PROGRESS, &manager->flags);
}

static void hf_manager_io_sample(struct hf_manager *manager)
{
    int retval;

    if (!manager->hf_dev || !manager->hf_dev->sample)
        return;

    if (!test_and_set_bit(HF_MANAGER_IO_IN_PROGRESS, &manager->flags)) {
        retval = manager->hf_dev->sample(manager->hf_dev);
        if (retval) {
            clear_bit(HF_MANAGER_IO_IN_PROGRESS,
                  &manager->flags);
        }
    }
}

static void hf_manager_io_tasklet(unsigned long data)
{
    struct hf_manager *manager = (struct hf_manager *)data;

    hf_manager_io_sample(manager);
}

static void hf_manager_io_kthread_work(struct kthread_work *work)
{
    struct hf_manager *manager =
        container_of(work, struct hf_manager, io_kthread_work);

    hf_manager_io_sample(manager);
}

static void hf_manager_sched_sample(struct hf_manager *manager,
        int64_t timestamp)
{
    hf_manager_io_schedule(manager, timestamp);
}

static enum hrtimer_restart hf_manager_io_poll(struct hrtimer *timer)
{
    struct hf_manager *manager =
        (struct hf_manager *)container_of(timer,
            struct hf_manager, io_poll_timer);

    hf_manager_sched_sample(manager, ktime_get_boottime_ns());
    hrtimer_forward_now(&manager->io_poll_timer,
        ns_to_ktime(atomic64_read(&manager->io_poll_interval)));
    return HRTIMER_RESTART;
}

static void hf_manager_io_interrupt(struct hf_manager *manager,
        int64_t timestamp)
{
    hf_manager_sched_sample(manager, timestamp);
}

int hf_device_register(struct hf_device *device)
{
    struct hf_core *core = &hfcore;

    INIT_LIST_HEAD(&device->list);
    device->ready = false;
    mutex_lock(&core->device_lock);
    list_add(&device->list, &core->device_list);
    mutex_unlock(&core->device_lock);

    return 0;
}
EXPORT_SYMBOL_GPL(hf_device_register);

void hf_device_unregister(struct hf_device *device)
{
    struct hf_core *core = &hfcore;

    mutex_lock(&core->device_lock);
    list_del(&device->list);
    mutex_unlock(&core->device_lock);
    device->ready = false;
}
EXPORT_SYMBOL_GPL(hf_device_unregister);

int hf_manager_create(struct hf_device *device)
{
    uint8_t sensor_type = 0;
    int i = 0, err = 0;
    uint32_t gain = 0;
    struct hf_manager *manager = NULL;

    pr_err("mtkdebug:2 %s\n",__func__);
    if (!device || !device->dev_name ||
            !device->support_list || !device->support_size)
        return -EINVAL;

    pr_err("mtkdebug:3 %s\n",__func__);
    manager = kzalloc(sizeof(*manager), GFP_KERNEL);
    if (!manager)
        return -ENOMEM;
    pr_err("mtkdebug:4 %s\n",__func__);
    manager->hf_dev = device;
    manager->core = &hfcore;
    device->manager = manager;

    atomic_set(&manager->io_enabled, 0);
    atomic64_set(&manager->io_poll_interval, S64_MAX);

    clear_bit(HF_MANAGER_IO_IN_PROGRESS, &manager->flags);
    clear_bit(HF_MANAGER_IO_READY, &manager->flags);

    if (device->device_poll == HF_DEVICE_IO_POLLING) {
        hrtimer_init(&manager->io_poll_timer,
            CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        manager->io_poll_timer.function = hf_manager_io_poll;
    } else if (device->device_poll == HF_DEVICE_IO_INTERRUPT) {
        manager->interrupt = hf_manager_io_interrupt;
    }
    manager->report = hf_manager_io_report;
    manager->complete = hf_manager_io_complete;

    if (device->device_bus == HF_DEVICE_IO_ASYNC)
        tasklet_init(&manager->io_work_tasklet,
            hf_manager_io_tasklet, (unsigned long)manager);
    else if (device->device_bus == HF_DEVICE_IO_SYNC)
        kthread_init_work(&manager->io_kthread_work,
            hf_manager_io_kthread_work);

    for (i = 0; i < device->support_size; ++i) {
        sensor_type = device->support_list[i].sensor_type;
        gain = device->support_list[i].gain;
        if (unlikely(sensor_type >= SENSOR_TYPE_SENSOR_MAX || !gain)) {
            pr_err("Device:%s register failed, %u invalid gain\n",
                device->dev_name, sensor_type);
            err = -EINVAL;
            goto out_err;
        }
        if (test_and_set_bit(sensor_type, sensor_list_bitmap)) {
            pr_err("Device:%s register failed, %u repeat\n",
                device->dev_name, sensor_type);
            err = -EBUSY;
            goto out_err;
        }
    }

    INIT_LIST_HEAD(&manager->list);
    mutex_lock(&manager->core->manager_lock);
    list_add(&manager->list, &manager->core->manager_list);
    mutex_unlock(&manager->core->manager_lock);

    mutex_lock(&manager->core->device_lock);
    manager->hf_dev->ready = true;
    mutex_unlock(&manager->core->device_lock);

    return 0;
out_err:
    kfree(manager);
    device->manager = NULL;
    return err;
}
EXPORT_SYMBOL_GPL(hf_manager_create);

void hf_manager_destroy(struct hf_manager *manager)
{
    uint8_t sensor_type = 0;
    int i = 0;
    struct hf_device *device = NULL;

    if (!manager || !manager->hf_dev || !manager->hf_dev->support_list)
        return;

    device = manager->hf_dev;
    for (i = 0; i < device->support_size; ++i) {
        sensor_type = device->support_list[i].sensor_type;
        if (unlikely(sensor_type >= SENSOR_TYPE_SENSOR_MAX)) {
            pr_err("Device:%s unregister failed, %u exceed max\n",
                device->dev_name, sensor_type);
            continue;
        }
        clear_bit(sensor_type, sensor_list_bitmap);
    }
    mutex_lock(&manager->core->manager_lock);
    list_del(&manager->list);
    mutex_unlock(&manager->core->manager_lock);
    if (device->device_poll == HF_DEVICE_IO_POLLING)
        hrtimer_cancel(&manager->io_poll_timer);
    if (device->device_bus == HF_DEVICE_IO_ASYNC)
        tasklet_kill(&manager->io_work_tasklet);
    else if (device->device_bus == HF_DEVICE_IO_SYNC)
        kthread_flush_work(&manager->io_kthread_work);

    while (test_bit(HF_MANAGER_IO_IN_PROGRESS, &manager->flags))
        cpu_relax();

    kfree(manager);
}
EXPORT_SYMBOL_GPL(hf_manager_destroy);

int hf_device_register_manager_create(struct hf_device *device)
{
    int ret = 0;

    ret = hf_device_register(device);
    if (ret < 0)
        return ret;
    return hf_manager_create(device);
}
EXPORT_SYMBOL_GPL(hf_device_register_manager_create);

void hf_device_unregister_manager_destroy(struct hf_device *device)
{
    hf_manager_destroy(device->manager);
    hf_device_unregister(device);
}
EXPORT_SYMBOL_GPL(hf_device_unregister_manager_destroy);

static void down_sample_update(struct hf_core *core, uint8_t sensor_type)
{
    unsigned long flags;
    struct hf_client *client = NULL;
    struct sensor_state *request = NULL;
    int64_t min_delay = core->state[sensor_type].delay;

    spin_lock_irqsave(&core->client_lock, flags);
    list_for_each_entry(client, &core->client_list, list) {
        request = &client->request[sensor_type];
        if (request->enable && request->down_sample) {
            request->down_sample_cnt = 0;
            if (min_delay)
                request->down_sample_div =
                    div64_s64(request->delay, min_delay);
            else
                request->down_sample_div = 0;
        }
    }
    spin_unlock_irqrestore(&core->client_lock, flags);
}

static inline bool down_sample_estimate(struct sensor_state *request)
{
    if (!request->down_sample)
        return false;

    if (!request->down_sample_div) {
        request->down_sample_cnt = 0;
        return false;
    }
    if (++request->down_sample_cnt >= request->down_sample_div) {
        request->down_sample_cnt = 0;
        return false;
    }
    return true;
}

static int hf_manager_distinguish_event(struct hf_client *client,
        struct hf_manager_event *event)
{
    int err = 0;
    struct sensor_state *request = &client->request[event->sensor_type];

    switch (event->action) {
    case DATA_ACTION:
        /* must relay on enable status client requested */
        if (request->enable && !down_sample_estimate(request) &&
                (event->timestamp > request->start_time))
            err = hf_manager_report_event(client, event);
        break;
    case FLUSH_ACTION:
        /* must relay on flush count client requested */
        if (request->flush > 0) {
            err = hf_manager_report_event(client, event);
            /* return < 0, don't decrease flush count */
            if (err < 0)
                return err;
            request->flush--;
        }
        break;
    case BIAS_ACTION:
        /* relay on status client requested, don't check return */
        if (request->bias)
            hf_manager_report_event(client, event);
        break;
    case CALI_ACTION:
        /* cali on status client requested, don't check return */
        if (request->cali)
            hf_manager_report_event(client, event);
        break;
    case TEMP_ACTION:
        /* temp on status  client requested, don't check return */
        if (request->temp)
            hf_manager_report_event(client, event);
        break;
    case TEST_ACTION:
        /* test on status client requested, don't check return */
        if (request->test)
            hf_manager_report_event(client, event);
        break;
    case RAW_ACTION:
        /* raw on status client requested, don't check return */
        if (request->raw)
            hf_manager_report_event(client, event);
        break;
    default:
        pr_err("Report %u failed, unknown action %u\n",
            event->sensor_type, event->action);
        /* unknown action must return 0 */
        err = 0;
        break;
    }
    return err;
}

static int hf_manager_find_client(struct hf_core *core,
        struct hf_manager_event *event)
{
    int err = 0;
    unsigned long flags;
    struct hf_client *client = NULL;

    /*awinic bob add start*/
    if (event->action == DATA_ACTION && event->sensor_type == SENSOR_TYPE_SAR) {
                pr_info("sar rxh event->word[0]:0x%02x, event->word[1]:0x%02x, event->word[2]:0x%02x,event->word[3]:%d,event->word[4]:%d,event->word[5]:%d,event->word[6]:%d\n,",
                event->word[0], event->word[1], event->word[2],event->word[3],event->word[4],event->word[5],event->word[6]);
        if (event->word[0] == 0xff) {
            mutex_lock(&g_aw_sar_lock);
            awinic_debug_data[1] = event->word[1];
            awinic_debug_data[2] = event->word[2];
            awinic_debug_data[0] = event->word[0];
            mutex_unlock(&g_aw_sar_lock);

            wake_up_interruptible(&g_aw_sar_wait);

            return 0;
        }
    }
    /*awinic bob add end*/

    spin_lock_irqsave(&core->client_lock, flags);
    list_for_each_entry(client, &core->client_list, list) {
        /* must (err |=), collect all err to decide retry */
        err |= hf_manager_distinguish_event(client, event);
    }
    spin_unlock_irqrestore(&core->client_lock, flags);

    return err;
}

static struct hf_manager *hf_manager_find_manager(struct hf_core *core,
        uint8_t sensor_type)
{
    int i = 0;
    struct hf_manager *manager = NULL;
    struct hf_device *device = NULL;

    list_for_each_entry(manager, &core->manager_list, list) {
        device = READ_ONCE(manager->hf_dev);
        if (!device || !device->support_list)
            continue;
        for (i = 0; i < device->support_size; ++i) {
            if (sensor_type == device->support_list[i].sensor_type)
                return manager;
        }
    }
    pr_err("Failed to find manager, %u unregistered\n", sensor_type);
    return NULL;
}

static inline void hf_manager_save_update_enable(struct hf_client *client,
        struct hf_manager_cmd *cmd, struct sensor_state *old)
{
    unsigned long flags;
    struct hf_manager_batch *batch = (struct hf_manager_batch *)cmd->data;
    struct sensor_state *request = &client->request[cmd->sensor_type];

    spin_lock_irqsave(&client->core->client_lock, flags);
    /* only enable disable update action delay and latency */
    if (cmd->action == HF_MANAGER_SENSOR_ENABLE) {
        /*
         * NOTE: save significant parameter to old
         * remember mustn't save flush bias raw etc
         * down_sample_cnt and down_sample_div mustn't save due to
         * own_sample_update called in hf_manager_device_enable
         * when enable disable and batch device success
         */
        old->enable = request->enable;
        old->down_sample = request->down_sample;
        old->delay = request->delay;
        old->latency = request->latency;
        old->start_time = request->start_time;
        /* update new */
        if (!request->enable)
            request->start_time = ktime_get_boottime_ns();
        request->enable = true;
        request->down_sample = cmd->down_sample;
        request->delay = batch->delay;
        request->latency = batch->latency;
    } else if (cmd->action == HF_MANAGER_SENSOR_DISABLE) {
        request->enable = false;
        request->down_sample = false;
        request->delay = S64_MAX;
        request->latency = S64_MAX;
        request->start_time = S64_MAX;
    }
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_restore_enable(struct hf_client *client,
        struct hf_manager_cmd *cmd, struct sensor_state *old)
{
    unsigned long flags;
    struct sensor_state *request = &client->request[cmd->sensor_type];

    spin_lock_irqsave(&client->core->client_lock, flags);
    if (cmd->action == HF_MANAGER_SENSOR_ENABLE) {
        /*
         * NOTE: restore significant parameter from old
         * remember mustn't restore flush bias raw etc
         * down_sample_cnt and down_sample_div mustn't restore due to
         * down_sample_update called in hf_manager_device_enable
         * when enable disable and batch device success
         */
        request->enable = old->enable;
        request->down_sample = old->down_sample;
        request->delay = old->delay;
        request->latency = old->latency;
        request->start_time = old->start_time;
    } else if (cmd->action == HF_MANAGER_SENSOR_DISABLE) {
        request->enable = false;
        request->down_sample = false;
        request->delay = S64_MAX;
        request->latency = S64_MAX;
        request->start_time = S64_MAX;
    }
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_inc_flush(struct hf_client *client,
        uint8_t sensor_type)
{
    unsigned long flags;

    spin_lock_irqsave(&client->core->client_lock, flags);
    client->request[sensor_type].flush++;
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_dec_flush(struct hf_client *client,
        uint8_t sensor_type)
{
    unsigned long flags;

    spin_lock_irqsave(&client->core->client_lock, flags);
    if (client->request[sensor_type].flush > 0)
        client->request[sensor_type].flush--;
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_update_bias(struct hf_client *client,
        uint8_t sensor_type, bool enable)
{
    unsigned long flags;

    spin_lock_irqsave(&client->core->client_lock, flags);
    client->request[sensor_type].bias = enable;
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_update_cali(struct hf_client *client,
        uint8_t sensor_type, bool enable)
{
    unsigned long flags;

    spin_lock_irqsave(&client->core->client_lock, flags);
    client->request[sensor_type].cali = enable;
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_update_temp(struct hf_client *client,
        uint8_t sensor_type, bool enable)
{
    unsigned long flags;

    spin_lock_irqsave(&client->core->client_lock, flags);
    client->request[sensor_type].temp = enable;
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_update_test(struct hf_client *client,
        uint8_t sensor_type, bool enable)
{
    unsigned long flags;

    spin_lock_irqsave(&client->core->client_lock, flags);
    client->request[sensor_type].test = enable;
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_update_raw(struct hf_client *client,
        uint8_t sensor_type, bool enable)
{
    unsigned long flags;

    spin_lock_irqsave(&client->core->client_lock, flags);
    client->request[sensor_type].raw = enable;
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static inline void hf_manager_clear_raw(struct hf_client *client,
        uint8_t sensor_type)
{
    unsigned long flags;

    spin_lock_irqsave(&client->core->client_lock, flags);
    client->request[sensor_type].raw = false;
    spin_unlock_irqrestore(&client->core->client_lock, flags);
}

static void hf_manager_find_best_param(struct hf_core *core,
        uint8_t sensor_type, bool *action,
        int64_t *delay, int64_t *latency)
{
    unsigned long flags;
    struct hf_client *client = NULL;
    struct sensor_state *request = NULL;
    bool tmp_enable = false;
    int64_t tmp_delay = S64_MAX;
    int64_t tmp_latency = S64_MAX;
    const int64_t max_latency_ns = 2000000000000LL;

    spin_lock_irqsave(&core->client_lock, flags);
    list_for_each_entry(client, &core->client_list, list) {
        request = &client->request[sensor_type];
        if (request->enable) {
            tmp_enable = true;
            if (request->delay < tmp_delay)
                tmp_delay = request->delay;
            if (request->latency < tmp_latency)
                tmp_latency = request->latency;
        }
    }
    spin_unlock_irqrestore(&core->client_lock, flags);
    *action = tmp_enable;
    *delay = tmp_delay > 0 ? tmp_delay : 0;
    tmp_latency = tmp_latency > 0 ? tmp_latency : 0;
    *latency = tmp_latency < max_latency_ns ? tmp_latency : max_latency_ns;

#ifdef HF_MANAGER_DEBUG
    if (tmp_enable)
        pr_notice("Find best command %u %u %lld %lld\n",
            sensor_type, tmp_enable, tmp_delay, tmp_latency);
    else
        pr_notice("Find best command %u %u\n",
            sensor_type, tmp_enable);
#endif
}

static inline bool device_rebatch(struct hf_core *core, uint8_t sensor_type,
            int64_t best_delay, int64_t best_latency)
{
    if (core->state[sensor_type].delay != best_delay ||
            core->state[sensor_type].latency != best_latency) {
        core->state[sensor_type].delay = best_delay;
        core->state[sensor_type].latency = best_latency;
        return true;
    }
    return false;
}

static inline bool device_reenable(struct hf_core *core, uint8_t sensor_type,
        bool best_enable)
{
    if (core->state[sensor_type].enable != best_enable) {
        core->state[sensor_type].enable = best_enable;
        return true;
    }
    return false;
}

static inline bool device_redisable(struct hf_core *core, uint8_t sensor_type,
        bool best_enable, int64_t best_delay, int64_t best_latency)
{
    if (core->state[sensor_type].enable != best_enable) {
        core->state[sensor_type].enable = best_enable;
        core->state[sensor_type].delay = best_delay;
        core->state[sensor_type].latency = best_latency;
        return true;
    }
    return false;
}

static inline void device_state_save(struct hf_core *core,
        uint8_t sensor_type, struct sensor_state *old)
{
    /* save enable delay and latency to old */
    old->enable = core->state[sensor_type].enable;
    old->delay = core->state[sensor_type].delay;
    old->latency = core->state[sensor_type].latency;
}

static inline void device_state_restore(struct hf_core *core,
        uint8_t sensor_type, struct sensor_state *old)
{
    /*
     * restore enable delay and latency
     * remember must not restore bias raw etc
     */
    core->state[sensor_type].enable = old->enable;
    core->state[sensor_type].delay = old->delay;
    core->state[sensor_type].latency = old->latency;
}

static int64_t device_poll_min_interval(struct hf_device *device)
{
    int i = 0;
    uint8_t j = 0;
    int64_t interval = S64_MAX;
    struct hf_core *core = device->manager->core;

    for (i = 0; i < device->support_size; ++i) {
        j = device->support_list[i].sensor_type;
        if (core->state[j].enable) {
            if (core->state[j].delay < interval)
                interval = core->state[j].delay;
        }
    }
    return interval;
}

static void device_poll_trigger(struct hf_device *device, bool enable)
{
    int64_t min_interval = S64_MAX;
    struct hf_manager *manager = device->manager;

    WARN_ON(enable && !atomic_read(&manager->io_enabled));
    min_interval = device_poll_min_interval(device);
    WARN_ON(atomic_read(&manager->io_enabled) && min_interval == S64_MAX);
    if (atomic64_read(&manager->io_poll_interval) == min_interval)
        return;
    atomic64_set(&manager->io_poll_interval, min_interval);
    if (atomic_read(&manager->io_enabled))
        hrtimer_start(&manager->io_poll_timer,
            ns_to_ktime(min_interval), HRTIMER_MODE_REL);
    else
        hrtimer_cancel(&manager->io_poll_timer);
}

static int hf_manager_device_enable(struct hf_device *device,
        uint8_t sensor_type)
{
    int err = 0;
    struct sensor_state old;
    struct hf_manager *manager = device->manager;
    struct hf_core *core = device->manager->core;
    bool best_enable = false;
    int64_t best_delay = S64_MAX;
    int64_t best_latency = S64_MAX;

    if (!device->enable || !device->batch)
        return -EINVAL;

    hf_manager_find_best_param(core, sensor_type, &best_enable,
        &best_delay, &best_latency);

    if (best_enable) {
        device_state_save(core, sensor_type, &old);
        if (device_rebatch(core, sensor_type,
                best_delay, best_latency)) {
            err = device->batch(device, sensor_type,
                best_delay, best_latency);
            /* handle error to return when batch fail */
            if (err < 0) {
                device_state_restore(core, sensor_type, &old);
                goto out;
            }
        }
        if (device_reenable(core, sensor_type, best_enable)) {
            /* must update io_enabled before enable */
            atomic_inc(&manager->io_enabled);
            err = device->enable(device, sensor_type, best_enable);
            /* handle error to clear prev request */
            if (err < 0) {
                atomic_dec_if_positive(&manager->io_enabled);
                /*
                 * rebatch success and enable fail.
                 * update prev request from old.
                 */
                device_state_restore(core, sensor_type, &old);
                goto out;
            }
        }
        if (device->device_poll == HF_DEVICE_IO_POLLING)
            device_poll_trigger(device, best_enable);
    } else {
        if (device_redisable(core, sensor_type, best_enable,
                best_delay, best_latency)) {
            atomic_dec_if_positive(&manager->io_enabled);
            err = device->enable(device, sensor_type, best_enable);
            /*
             * disable fail no need to handle error.
             * run next to update hrtimer or tasklet.
             */
        }
        if (device->device_poll == HF_DEVICE_IO_POLLING)
            device_poll_trigger(device, best_enable);
        if (device->device_bus == HF_DEVICE_IO_ASYNC &&
                !atomic_read(&manager->io_enabled))
            tasklet_kill(&manager->io_work_tasklet);
    }

out:
    /*
     * enable, batch or disable success we update down sample.
     */
    if (!err)
        down_sample_update(core, sensor_type);
    return err;
}

static int hf_manager_device_flush(struct hf_device *device,
        uint8_t sensor_type)
{
    if (!device->flush)
        return -EINVAL;

    return device->flush(device, sensor_type);
}

static int hf_manager_device_calibration(struct hf_device *device,
        uint8_t sensor_type)
{
    if (device->calibration){
        return device->calibration(device, sensor_type);
        }
    return 0;
}

static int hf_manager_device_config_cali(struct hf_device *device,
        uint8_t sensor_type, void *data, uint8_t length)
{
    if (device->config_cali)
        return device->config_cali(device, sensor_type, data, length);
    return 0;
}

static int hf_manager_device_selftest(struct hf_device *device,
        uint8_t sensor_type)
{
    if (device->selftest)
        return device->selftest(device, sensor_type);
    return 0;
}

static int hf_manager_device_rawdata(struct hf_device *device,
        uint8_t sensor_type)
{
    int err = 0;
    unsigned long flags;
    struct hf_core *core = device->manager->core;
    struct hf_client *client = NULL;
    struct sensor_state *request = NULL;
    bool best_enable = false;

    if (!device->rawdata)
        return 0;

    spin_lock_irqsave(&core->client_lock, flags);
    list_for_each_entry(client, &core->client_list, list) {
        request = &client->request[sensor_type];
        if (request->raw)
            best_enable = true;
    }
    spin_unlock_irqrestore(&core->client_lock, flags);

    if (core->state[sensor_type].raw == best_enable)
        return 0;
    core->state[sensor_type].raw = best_enable;
    err = device->rawdata(device, sensor_type, best_enable);
    if (err < 0)
        core->state[sensor_type].raw = false;
    return err;
}

static int hf_manager_device_info(struct hf_client *client,
        uint8_t sensor_type, struct sensor_info *info)
{
    int i = 0;
    int ret = 0;
    struct hf_manager *manager = NULL;
    struct hf_device *device = NULL;
    struct sensor_info *si = NULL;

    mutex_lock(&client->core->manager_lock);
    manager = hf_manager_find_manager(client->core, sensor_type);
    if (!manager) {
        ret = -EINVAL;
        goto err_out;
    }
    device = manager->hf_dev;
    if (!device || !device->support_list ||
            !device->support_size) {
        ret = -EINVAL;
        goto err_out;
    }
    for (i = 0; i < device->support_size; ++i) {
        if (device->support_list[i].sensor_type ==
                sensor_type) {
            si = &device->support_list[i];
            break;
        }
    }
    if (!si) {
        ret = -EINVAL;
        goto err_out;
    }
    *info = *si;

err_out:
    mutex_unlock(&client->core->manager_lock);
    return ret;
}

static int hf_manager_custom_cmd(struct hf_client *client,
        uint8_t sensor_type, struct custom_cmd *cust_cmd)
{
    struct hf_manager *manager = NULL;
    struct hf_device *device = NULL;
    int ret = 0;

    if (cust_cmd->tx_len > sizeof(cust_cmd->data) ||
        cust_cmd->rx_len > sizeof(cust_cmd->data))
        return -EINVAL;

    mutex_lock(&client->core->manager_lock);
    manager = hf_manager_find_manager(client->core, sensor_type);
    if (!manager) {
        ret = -EINVAL;
        goto err_out;
    }
    device = manager->hf_dev;
    if (!device || !device->dev_name) {
        ret = -EINVAL;
        goto err_out;
    }
    if (device->custom_cmd)
        ret = device->custom_cmd(device, sensor_type, cust_cmd);

err_out:
    mutex_unlock(&client->core->manager_lock);
    return ret;
}

static int hf_manager_drive_device(struct hf_client *client,
        struct hf_manager_cmd *cmd)
{
    int err = 0;
    struct sensor_state old;
    struct hf_manager *manager = NULL;
    struct hf_device *device = NULL;
    struct hf_core *core = client->core;
    uint8_t sensor_type = cmd->sensor_type;

    if (unlikely(sensor_type >= SENSOR_TYPE_SENSOR_MAX))
        return -EINVAL;

    if (unlikely(cmd->length > sizeof(cmd->data)))
        return -EINVAL;

    mutex_lock(&core->manager_lock);
    manager = hf_manager_find_manager(core, sensor_type);
    if (!manager) {
        mutex_unlock(&core->manager_lock);
        return -EINVAL;
    }
    device = manager->hf_dev;
    if (!device || !device->dev_name) {
        mutex_unlock(&core->manager_lock);
        return -EINVAL;
    }

#ifdef HF_MANAGER_DEBUG
    pr_notice("Drive device:%s command %u %u\n",
        device->dev_name, cmd->sensor_type, cmd->action);
#endif

    switch (cmd->action) {
    case HF_MANAGER_SENSOR_ENABLE:
    case HF_MANAGER_SENSOR_DISABLE:
        //awinic bob add
        if (sensor_type == SENSOR_TYPE_SAR) {
            aw_g_device = device;
        }
        //awinic bob end
        if(cmd->action == HF_MANAGER_SENSOR_ENABLE && sensor_type == 8){
            set_lct_tp_proximity_switch_status(1);//打开TP模拟距感功能
            pr_err("longcheer enable proxmity");
        }
        else if(cmd->action == HF_MANAGER_SENSOR_DISABLE && sensor_type == 8){
            set_lct_tp_proximity_switch_status(0);//关闭TP模拟距感功能
            pr_err("longcheer disable proxmity");
        }
        hf_manager_save_update_enable(client, cmd, &old);
        err = hf_manager_device_enable(device, sensor_type);
        if (err < 0)
            hf_manager_restore_enable(client, cmd, &old);
        break;
    case HF_MANAGER_SENSOR_FLUSH:
        hf_manager_inc_flush(client, sensor_type);
        err = hf_manager_device_flush(device, sensor_type);
        if (err < 0)
            hf_manager_dec_flush(client, sensor_type);
        break;
    case HF_MANAGER_SENSOR_ENABLE_CALI:
        err = hf_manager_device_calibration(device, sensor_type);
        break;
    case HF_MANAGER_SENSOR_CONFIG_CALI:
        err = hf_manager_device_config_cali(device,
            sensor_type, cmd->data, cmd->length);

        /*awinic bob add start*/
        pr_info("sar sensor_type:%d length:%d\n",
                sensor_type, cmd->length);
        /*awinic bob add end*/

        break;
    case HF_MANAGER_SENSOR_SELFTEST:
        err = hf_manager_device_selftest(device, sensor_type);
        break;
    case HF_MANAGER_SENSOR_RAWDATA:
        hf_manager_update_raw(client, sensor_type, cmd->data[0]);
        err = hf_manager_device_rawdata(device, sensor_type);
        if (err < 0)
            hf_manager_clear_raw(client, sensor_type);
        break;
    default:
        pr_err("Unknown action %u\n", cmd->action);
        err = -EINVAL;
        break;
    }
    mutex_unlock(&core->manager_lock);
    return err;
}

static int hf_manager_get_sensor_info(struct hf_client *client,
        uint8_t sensor_type, struct sensor_info *info)
{
    return hf_manager_device_info(client, sensor_type, info);
}

struct hf_client *hf_client_create(void)
{
    unsigned long flags;
    struct hf_client *client = NULL;
    struct hf_client_fifo *hf_fifo = NULL;

    client = kzalloc(sizeof(*client), GFP_KERNEL);
    if (!client)
        goto err_out;

    /* record process id and thread id for debug */
    strlcpy(client->proc_comm, current->comm, sizeof(client->proc_comm));
    client->leader_pid = current->group_leader->pid;
    client->pid = current->pid;
    client->core = &hfcore;

#ifdef HF_MANAGER_DEBUG
    pr_notice("Client create\n");
#endif

    INIT_LIST_HEAD(&client->list);

    hf_fifo = &client->hf_fifo;
    hf_fifo->head = 0;
    hf_fifo->tail = 0;
    hf_fifo->bufsize = roundup_pow_of_two(HF_CLIENT_FIFO_SIZE);
    hf_fifo->buffull = false;
    spin_lock_init(&hf_fifo->buffer_lock);
    init_waitqueue_head(&hf_fifo->wait);
    hf_fifo->buffer =
        kcalloc(hf_fifo->bufsize, sizeof(*hf_fifo->buffer),
            GFP_KERNEL);
    if (!hf_fifo->buffer)
        goto err_free;

    spin_lock_init(&client->request_lock);

    spin_lock_irqsave(&client->core->client_lock, flags);
    list_add(&client->list, &client->core->client_list);
    spin_unlock_irqrestore(&client->core->client_lock, flags);

    return client;
err_free:
    kfree(client);
err_out:
    return NULL;
}
EXPORT_SYMBOL_GPL(hf_client_create);

void hf_client_destroy(struct hf_client *client)
{
    unsigned long flags;

#ifdef HF_MANAGER_DEBUG
    pr_notice("Client destroy\n");
#endif
    spin_lock_irqsave(&client->core->client_lock, flags);
    list_del(&client->list);
    spin_unlock_irqrestore(&client->core->client_lock, flags);

    kfree(client->hf_fifo.buffer);
    kfree(client);
}
EXPORT_SYMBOL_GPL(hf_client_destroy);

int hf_client_find_sensor(struct hf_client *client, uint8_t sensor_type)
{
    if (unlikely(sensor_type >= SENSOR_TYPE_SENSOR_MAX))
        return -EINVAL;
    if (!test_bit(sensor_type, sensor_list_bitmap))
        return -EINVAL;
    return 0;
}
EXPORT_SYMBOL_GPL(hf_client_find_sensor);

int hf_client_get_sensor_info(struct hf_client *client,
        uint8_t sensor_type, struct sensor_info *info)
{
    if (unlikely(sensor_type >= SENSOR_TYPE_SENSOR_MAX))
        return -EINVAL;
    if (!test_bit(sensor_type, sensor_list_bitmap))
        return -EINVAL;
    return hf_manager_device_info(client, sensor_type, info);
}
EXPORT_SYMBOL_GPL(hf_client_get_sensor_info);

int hf_client_request_sensor_cali(struct hf_client *client,
        uint8_t sensor_type, unsigned int cmd, bool status)
{
    if (unlikely(sensor_type >= SENSOR_TYPE_SENSOR_MAX))
        return -EINVAL;
    if (!test_bit(sensor_type, sensor_list_bitmap))
        return -EINVAL;
    switch (cmd) {
    case HF_MANAGER_REQUEST_BIAS_DATA:
        hf_manager_update_bias(client, sensor_type, status);
        break;
    case HF_MANAGER_REQUEST_CALI_DATA:
        hf_manager_update_cali(client, sensor_type, status);
        break;
    case HF_MANAGER_REQUEST_TEMP_DATA:
        hf_manager_update_temp(client, sensor_type, status);
        break;
    case HF_MANAGER_REQUEST_TEST_DATA:
        hf_manager_update_test(client, sensor_type, status);
        break;
    default:
        pr_err("Unknown command %u\n", cmd);
        return -EINVAL;
    }
    return 0;
}
EXPORT_SYMBOL_GPL(hf_client_request_sensor_cali);

int hf_client_control_sensor(struct hf_client *client,
        struct hf_manager_cmd *cmd)
{
    return hf_manager_drive_device(client, cmd);
}
EXPORT_SYMBOL_GPL(hf_client_control_sensor);

static int fetch_next(struct hf_client_fifo *hf_fifo,
                  struct hf_manager_event *event)
{
    unsigned long flags;
    int have_event;

    spin_lock_irqsave(&hf_fifo->buffer_lock, flags);
    have_event = hf_fifo->head != hf_fifo->tail;
    if (have_event) {
        *event = hf_fifo->buffer[hf_fifo->tail++];
        hf_fifo->tail &= hf_fifo->bufsize - 1;
        hf_fifo->buffull = false;
        hf_fifo->client_active = ktime_get_boottime_ns();
    }
    spin_unlock_irqrestore(&hf_fifo->buffer_lock, flags);
    return have_event;
}

/* timeout: MAX_SCHEDULE_TIMEOUT or msecs_to_jiffies(ms) */
int hf_client_poll_sensor_timeout(struct hf_client *client,
        struct hf_manager_event *data, int count, long timeout)
{
    long ret = 0;
    int read = 0;
    struct hf_client_fifo *hf_fifo = &client->hf_fifo;

    /* ret must be long to fill timeout(MAX_SCHEDULE_TIMEOUT) */
    ret = wait_event_interruptible_timeout(hf_fifo->wait,
        hf_fifo->head != hf_fifo->tail, timeout);

    if (!ret)
        return -ETIMEDOUT;
    if (ret < 0)
        return ret;

    for (;;) {
        if (hf_fifo->head == hf_fifo->tail)
            return 0;
        if (count == 0)
            break;
        while (read < count &&
            fetch_next(hf_fifo, &data[read])) {
            read++;
        }
        if (read)
            break;
    }
    return read;
}
EXPORT_SYMBOL_GPL(hf_client_poll_sensor_timeout);

int hf_client_custom_cmd(struct hf_client *client,
        uint8_t sensor_type, struct custom_cmd *cust_cmd)
{
    if (unlikely(sensor_type >= SENSOR_TYPE_SENSOR_MAX))
        return -EINVAL;
    if (!test_bit(sensor_type, sensor_list_bitmap))
        return -EINVAL;
    return hf_manager_custom_cmd(client, sensor_type, cust_cmd);
}
EXPORT_SYMBOL_GPL(hf_client_custom_cmd);

static int hf_manager_open(struct inode *inode, struct file *filp)
{
    struct hf_client *client = hf_client_create();

    if (!client)
        return -ENOMEM;

    filp->private_data = client;
    nonseekable_open(inode, filp);
    return 0;
}

static int hf_manager_release(struct inode *inode, struct file *filp)
{
    struct hf_client *client = filp->private_data;

    filp->private_data = NULL;
    hf_client_destroy(client);
    return 0;
}

static ssize_t hf_manager_read(struct file *filp,
        char __user *buf, size_t count, loff_t *f_pos)
{
    struct hf_client *client = filp->private_data;
    struct hf_client_fifo *hf_fifo = &client->hf_fifo;
    struct hf_manager_event event;
    size_t read = 0;

    if (count != 0 && count < sizeof(struct hf_manager_event))
        return -EINVAL;

    for (;;) {
        if (hf_fifo->head == hf_fifo->tail)
            return 0;
        if (count == 0)
            break;
        while (read + sizeof(event) <= count &&
                fetch_next(hf_fifo, &event)) {
            if (copy_to_user(buf + read, &event, sizeof(event)))
                return -EFAULT;
            read += sizeof(event);
        }
        if (read)
            break;
    }
    return read;
}

static ssize_t hf_manager_write(struct file *filp,
        const char __user *buf, size_t count, loff_t *f_pos)
{
    struct hf_manager_cmd cmd;
    struct hf_client *client = filp->private_data;

    memset(&cmd, 0, sizeof(cmd));

    if (count != sizeof(struct hf_manager_cmd))
        return -EINVAL;

    if (copy_from_user(&cmd, buf, count))
        return -EFAULT;

    return hf_manager_drive_device(client, &cmd);
}

static unsigned int hf_manager_poll(struct file *filp,
        struct poll_table_struct *wait)
{
    struct hf_client *client = filp->private_data;
    struct hf_client_fifo *hf_fifo = &client->hf_fifo;
    unsigned int mask = 0;

    client->ppid = current->pid;

    poll_wait(filp, &hf_fifo->wait, wait);

    if (hf_fifo->head != hf_fifo->tail)
        mask |= POLLIN | POLLRDNORM;

    return mask;
}

/*awinic bob add start*/
void *aw_memdup_user(const void __user *src, size_t len)
{
    void *p = NULL;

    /*
     * Always use GFP_KERNEL, since copy_from_user() can sleep and
     * cause pagefault, which makes it pointless to use GFP_NOFS
     * or GFP_ATOMIC.
     */
    p = vzalloc(len);
    if (!p) {
        pr_err("sar vzalloc err!");
        return p;
    }

    if (copy_from_user(p, src, len)) {
        pr_err("sar copy_from_user src err!");
        vfree(p);
    }

    return p;
}
/*awinic bob add end*/

static long hf_manager_ioctl(struct file *filp,
            unsigned int cmd, unsigned long arg)
{
    struct hf_client *client = filp->private_data;
    unsigned int size = _IOC_SIZE(cmd);
    void __user *ubuf = (void __user *)arg;
    uint8_t sensor_type = 0;
    struct ioctl_packet packet;
    struct sensor_info info;
    struct custom_cmd *cust_cmd = NULL;
    struct hf_device *device = NULL;

    /*awinic bob add start*/
    struct SAR_SENSOR_DATA aw_sar_sensor_data;
    struct aw_i2c_data *i2c_data;
    unsigned char __user **data_ptrs;
    int i = 0;
//    int err = 0;
    int j = 0;
    uint8_t buf[50] = { 0 };
    /*awinic bob add end*/

    memset(&packet, 0, sizeof(packet));

    /*awinic bob add start*/
    if (cmd != HF_AW_MANAGER_REQUEST_READ_STATUS) {
        if (size != sizeof(struct ioctl_packet))
            return -EINVAL;

        if (copy_from_user(&packet, ubuf, sizeof(packet)))
            return -EFAULT;
    }
    /*
    if (size != sizeof(struct ioctl_packet))
        return -EINVAL;

    if (copy_from_user(&packet, ubuf, sizeof(packet)))
        return -EFAULT;
    */
    /*awinic bob add end*/
    sensor_type = packet.sensor_type;
    if (unlikely(sensor_type >= SENSOR_TYPE_SENSOR_MAX))
        return -EINVAL;

    switch (cmd) {
    case HF_MANAGER_REQUEST_REGISTER_STATUS:
        packet.status = test_bit(sensor_type, sensor_list_bitmap);
        if (copy_to_user(ubuf, &packet, sizeof(packet)))
            return -EFAULT;
        break;
    case HF_MANAGER_REQUEST_BIAS_DATA:
        hf_manager_update_bias(client, sensor_type, packet.status);
        break;
    case HF_MANAGER_REQUEST_CALI_DATA:
        hf_manager_update_cali(client, sensor_type, packet.status);
        break;
    case HF_MANAGER_REQUEST_TEMP_DATA:
        hf_manager_update_temp(client, sensor_type, packet.status);
        break;
    case HF_MANAGER_REQUEST_TEST_DATA:
        hf_manager_update_test(client, sensor_type, packet.status);
        break;
    case HF_MANAGER_REQUEST_SENSOR_INFO:
        if (!test_bit(sensor_type, sensor_list_bitmap))
            return -EINVAL;
        memset(&info, 0, sizeof(info));
        if (hf_manager_get_sensor_info(client, sensor_type, &info))
            return -EINVAL;
        if (sizeof(packet.byte) < sizeof(info))
            return -EINVAL;
        memcpy(packet.byte, &info, sizeof(info));
        if (copy_to_user(ubuf, &packet, sizeof(packet)))
            return -EFAULT;
        break;
    case HF_MANAGER_REQUEST_CUST_DATA:
        if (!test_bit(sensor_type, sensor_list_bitmap))
            return -EINVAL;
        if (sizeof(packet.byte) < sizeof(*cust_cmd))
            return -EINVAL;
        cust_cmd = (struct custom_cmd *)packet.byte;
        if (hf_manager_custom_cmd(client, sensor_type, cust_cmd))
            return -EINVAL;
        if (copy_to_user(ubuf, &packet, sizeof(packet)))
            return -EFAULT;
        break;
    case HF_MANAGER_REQUEST_READY_STATUS:
        mutex_lock(&client->core->device_lock);
        packet.status = true;
        list_for_each_entry(device, &client->core->device_list, list) {
            if (!READ_ONCE(device->ready)) {
                pr_err_ratelimited("Device:%s not ready\n",
                    device->dev_name);
                packet.status = false;
                break;
            }
        }
        mutex_unlock(&client->core->device_lock);
        if (copy_to_user(ubuf, &packet, sizeof(packet)))
            return -EFAULT;
        break;
    /*awinic bob add start*/
    case HF_AW_MANAGER_REQUEST_READ_STATUS:
    //    pr_info("sar HF_AW_MANAGER_REQUEST_READ_STATUS enter\n");
        if (copy_from_user(&aw_sar_sensor_data,
                (struct SAR_SENSOR_DATA __user *)arg,
                sizeof(aw_sar_sensor_data))) {
            pr_err("sar copy_from_user err!\n");
            return -EFAULT;
        }
        //pr_info("sar data num: %d\n", aw_sar_sensor_data.num);

        i2c_data = (struct aw_i2c_data *)aw_memdup_user(aw_sar_sensor_data.data,
                aw_sar_sensor_data.num * sizeof(struct aw_i2c_data));
        if (i2c_data == NULL) {
            pr_err("sar aw_memdup_user err!\n");
            return -1;
        }

        data_ptrs = vzalloc(aw_sar_sensor_data.num * sizeof(u8 __user *));
        if (data_ptrs == NULL) {
            pr_err("sar vzalloc err\n");
            vfree(i2c_data);
            return -1;
        }

        for (i = 0; i < aw_sar_sensor_data.num; i++) {
            if (i2c_data[i].len > 256) {
                pr_err("sar i2c_data[i].len > 256 err!\n");
                goto free_cfg_data_hanld;
            }

            data_ptrs[i] = (unsigned char __user *)i2c_data[i].buf;
            i2c_data[i].buf = aw_memdup_user(data_ptrs[i], i2c_data[i].len);
            if (i2c_data[i].buf == NULL) {
                goto free_cfg_data_hanld;
                break;
            }
        }

        if (aw_sar_sensor_data.num == 1) {
            struct hf_manager_cmd manager_cmd;

            manager_cmd.sensor_type = SENSOR_TYPE_SAR;
            manager_cmd.action = HF_MANAGER_SENSOR_CONFIG_CALI;
            manager_cmd.length = i2c_data[0].len;
            memcpy(manager_cmd.data, &i2c_data[0].buf[0], i2c_data[0].len);
            //pr_info("sar  %d %d\n", i2c_data[0].buf[0], i2c_data[0].len);

            hf_manager_drive_device(client, &manager_cmd);
        } else if (aw_sar_sensor_data.num == 2) {
            struct hf_manager_cmd manager_cmd;
            int32_t ret = 0;

            manager_cmd.sensor_type = SENSOR_TYPE_SAR;
            manager_cmd.action = HF_MANAGER_SENSOR_CONFIG_CALI;
            manager_cmd.length = i2c_data[0].len;
            memcpy(manager_cmd.data, &i2c_data[0].buf[0], i2c_data[0].len);

            ret = hf_manager_drive_device(client, &manager_cmd);
            //pr_info("sar ret = %d, len = %d\n", ret, i2c_data[0].len);
    
            wait_event_interruptible_timeout(g_aw_sar_wait,
                        (awinic_debug_data[0] == 0xff) ? true : false,
                        msecs_to_jiffies(1000));
            mutex_lock(&g_aw_sar_lock);
            awinic_debug_data[0] = 0;

            //pr_info("sar awinic_debug_data[0]:0x%x awinic_debug_data[1]:0x%x awinic_debug_data[2]:0x%x",
            //    awinic_debug_data[0], awinic_debug_data[1], awinic_debug_data[2]);
            snprintf(buf, 50, "0x%02x 0x%02x 0x%02x 0x%02x ",
                                (awinic_debug_data[1] >> 8) & 0xff,
                                (awinic_debug_data[1] >> 0) & 0xff,
                                (awinic_debug_data[2] >> 8) & 0xff,
                                (awinic_debug_data[2] >> 0) & 0xff);
            pr_info("rxh addr:0x%02x%02x %s\n", i2c_data[0].buf[2], i2c_data[0].buf[3], buf);
            if (copy_to_user(data_ptrs[1], buf, strlen(buf) + 1)) {
                pr_err("sar copy_to_user err");
            }
            mutex_unlock(&g_aw_sar_lock);

        }
free_cfg_data_hanld:
        for (j = 0; j < aw_sar_sensor_data.num; j++) {
            if (i2c_data[j].buf != NULL) {
                vfree(i2c_data[j].buf);
            }
        }
        vfree(data_ptrs);
        vfree(i2c_data);
        return 0;
/*awinic bob add end*/
    default:
        pr_err("Unknown command %u\n", cmd);
        return -EINVAL;
    }
    return 0;
}

static const struct file_operations hf_manager_fops = {
    .owner          = THIS_MODULE,
    .open           = hf_manager_open,
    .release        = hf_manager_release,
    .read           = hf_manager_read,
    .write          = hf_manager_write,
    .poll           = hf_manager_poll,
    .unlocked_ioctl = hf_manager_ioctl,
    .compat_ioctl   = hf_manager_ioctl,
};

static int hf_manager_proc_show(struct seq_file *m, void *v)
{
    int i = 0, j = 0, k = 0;
    uint8_t sensor_type = 0;
    unsigned long flags;
    struct hf_core *core = (struct hf_core *)m->private;
    struct hf_manager *manager = NULL;
    struct hf_client *client = NULL;
    struct hf_device *device = NULL;
    const unsigned int debug_len = 4096;
    uint8_t *debug_buffer = NULL;

    seq_puts(m, "**************************************************\n");
    seq_puts(m, "Manager List:\n");
    mutex_lock(&core->manager_lock);
    j = 1;
    k = 1;
    list_for_each_entry(manager, &core->manager_list, list) {
        device = READ_ONCE(manager->hf_dev);
        if (!device || !device->support_list)
            continue;
        seq_printf(m, "%d. manager:[%d,%lld]\n", j++,
            atomic_read(&manager->io_enabled),
            print_s64((int64_t)atomic64_read(
                &manager->io_poll_interval)));
        seq_printf(m, " device:%s poll:%s bus:%s online\n",
            device->dev_name,
            device->device_poll ? "io_polling" : "io_interrupt",
            device->device_bus ? "io_async" : "io_sync");
        for (i = 0; i < device->support_size; ++i) {
            sensor_type = device->support_list[i].sensor_type;
            seq_printf(m, "  (%d) type:%u info:[%u,%s,%s]\n",
                k++,
                sensor_type,
                device->support_list[i].gain,
                device->support_list[i].name,
                device->support_list[i].vendor);
        }
    }
    mutex_unlock(&core->manager_lock);

    seq_puts(m, "**************************************************\n");
    seq_puts(m, "Client List:\n");
    spin_lock_irqsave(&core->client_lock, flags);
    j = 1;
    k = 1;
    list_for_each_entry(client, &core->client_list, list) {
        seq_printf(m, "%d. client:%s pid:[%d:%d,%d] online\n",
            j++,
            client->proc_comm,
            client->leader_pid,
            client->pid,
            client->ppid);
        for (i = 0; i < SENSOR_TYPE_SENSOR_MAX; ++i) {
            if (!client->request[i].enable)
                continue;
            seq_printf(m, " (%d) type:%d param:[%lld,%lld,%lld]",
                k++,
                i,
                client->request[i].delay,
                client->request[i].latency,
                client->request[i].start_time);
            seq_printf(m, " ds:[%u,%u,%u]\n",
                client->request[i].down_sample,
                client->request[i].down_sample_cnt,
                client->request[i].down_sample_div);
        }
    }
    spin_unlock_irqrestore(&core->client_lock, flags);

    seq_puts(m, "**************************************************\n");
    seq_puts(m, "Active List:\n");
    mutex_lock(&core->manager_lock);
    j = 1;
    for (i = 0; i < SENSOR_TYPE_SENSOR_MAX; ++i) {
        if (!core->state[i].enable)
            continue;
        seq_printf(m, "%d. type:%d param:[%lld,%lld]\n",
            j++,
            i,
            core->state[i].delay,
            core->state[i].latency);
    }
    mutex_unlock(&core->manager_lock);

    seq_puts(m, "**************************************************\n");
    mutex_lock(&core->manager_lock);
    debug_buffer = kzalloc(debug_len, GFP_KERNEL);
    list_for_each_entry(manager, &core->manager_list, list) {
        device = READ_ONCE(manager->hf_dev);
        if (!device || !device->support_list || !device->debug)
            continue;
        if (device->debug(device, SENSOR_TYPE_INVALID, debug_buffer,
                debug_len) > 0) {
            seq_printf(m, "Debug Sub Module: %s\n",
                device->dev_name);
            seq_printf(m, "%s\n", debug_buffer);
        }
    }
    kfree(debug_buffer);
    mutex_unlock(&core->manager_lock);
    return 0;
}

static int hf_manager_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, hf_manager_proc_show, PDE_DATA(inode));
}

static const struct proc_ops hf_manager_proc_fops = {
    .proc_open           = hf_manager_proc_open,
    .proc_release        = single_release,
    .proc_read           = seq_read,
    .proc_lseek         = seq_lseek,
};

/* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL

struct aw_sar_ps {
    bool ps_is_present;
    struct work_struct ps_notify_work;
    struct notifier_block ps_notif;
};

static void aw_sar_ps_notify_callback_work(struct work_struct *work)
{
    pr_info("sar Usb insert,going to force calibrate\n");
    //mtk_nanohub_calibration_to_hub(ID_SAR);

    if (aw_g_device != NULL) {
        hf_manager_device_calibration(aw_g_device, SENSOR_TYPE_SAR);
    } else {
        pr_info("sar aw_g_device is NUll error!\n");
    }
}

static int aw_sar_ps_get_state(struct power_supply *psy, bool *present)
{
    union power_supply_propval pval = { 0 };
    int retval;

#ifdef AW_SAR_CONFIG_MTK_CHARGER
    retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE,
            &pval);
#else
    retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
            &pval);
#endif
    if (retval) {
        pr_err("sar %s psy get property failed\n", psy->desc->name);
        return retval;
    }
    //pr_info("sar pys name:%s\n",  psy->desc->name);
    if (strcmp(psy->desc->name, "charger") == 0) {
//    if (strcmp(psy->desc->name, "mtk-master-charger") == 0) {
        *present = (pval.intval) ? true : false;
        pr_info("sar %s is %s\n", psy->desc->name,
                (*present) ? "present" : "not present");
    }

    return 0;
}

static int aw_sar_ps_notify_callback(struct notifier_block *self,
        unsigned long event, void *p)
{
    struct aw_sar_ps *aw_sar_ps_to_cail = container_of(self, struct aw_sar_ps, ps_notif);
    struct power_supply *psy = p;
    bool present;
    int retval;
    //pr_info("sar %s\n", __func__);
    //pr_info("sar %s name:%s\n", __func__, psy->desc->name);
    if ((event == PSY_EVENT_PROP_CHANGED)
         && psy->desc->get_property && psy->desc->name){
        //pr_info("sar1 %s\n", __func__);
        retval = aw_sar_ps_get_state(psy, &present);
        if (retval) {
            return retval;
        }
        if (event == PSY_EVENT_PROP_CHANGED) {
            if (aw_sar_ps_to_cail->ps_is_present == present) {
                pr_err("sar ps present state not change\n");
                return 0;
            }
        }
        aw_sar_ps_to_cail->ps_is_present = present;
        schedule_work(&aw_sar_ps_to_cail->ps_notify_work);
    }

    return 0;
}

static int aw_sar_ps_notify_init(struct aw_sar_ps *aw_sar_ps_to_cail)
{
    struct power_supply *psy = NULL;
    int ret = 0;

    //pr_info("%s enter\n", __func__);
    INIT_WORK(&aw_sar_ps_to_cail->ps_notify_work, aw_sar_ps_notify_callback_work);
    aw_sar_ps_to_cail->ps_notif.notifier_call = aw_sar_ps_notify_callback;
    ret = power_supply_reg_notifier(&aw_sar_ps_to_cail->ps_notif);
    if (ret) {
        pr_err("sar Unable to register ps_notifier: %d\n", ret);
        return -1;
    }
    //psy = power_supply_get_by_name(USB_POWER_SUPPLY_NAME);
    psy = power_supply_get_by_name("charger");
    if (psy) {
        ret = aw_sar_ps_get_state(psy, &aw_sar_ps_to_cail->ps_is_present);
        if (ret) {
            pr_err("sar psy get property failed rc=%d\n", ret);
            goto free_ps_notifier;
        }
    } else {
        pr_err("sar psy is NULL error!\n");
    }
    return 0;

free_ps_notifier:
    power_supply_unreg_notifier(&aw_sar_ps_to_cail->ps_notif);

    return -1;
}
#endif
/* awinic bob add end*/


static int __init hf_manager_init(void)
{
    int ret = 0;
    struct device *dev;
    struct sched_param param = { .sched_priority = MAX_RT_PRIO / 2 };
    /* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL
    int err = 0;
    struct aw_sar_ps *aw_sar_ps_to_cail = NULL;
    aw_sar_ps_to_cail = kzalloc(sizeof(*aw_sar_ps_to_cail), GFP_KERNEL);
    if (!aw_sar_ps_to_cail) {
        err = -ENOMEM;
        goto exit_aw_kfree;
    }
#endif
    /* awinic bob add end */

    pr_info("sar enter\n");

    init_hf_core(&hfcore);

    major = register_chrdev(0, "hf_manager", &hf_manager_fops);
    if (major < 0) {
        pr_err("Unable to get major\n");
        ret = major;
        goto err_exit;
    }

    hf_manager_class = class_create(THIS_MODULE, "hf_manager");
    if (IS_ERR(hf_manager_class)) {
        pr_err("Failed to create class\n");
        ret = PTR_ERR(hf_manager_class);
        goto err_chredev;
    }

    dev = device_create(hf_manager_class, NULL, MKDEV(major, 0),
        NULL, "hf_manager");
    if (IS_ERR(dev)) {
        pr_err("Failed to create device\n");
        ret = PTR_ERR(dev);
        goto err_class;
    }

    if (!proc_create_data("hf_manager", 0440, NULL,
            &hf_manager_proc_fops, &hfcore))
        pr_err("Failed to create proc\n");


    /* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL
    pr_err("sar usb_plug_cail\n");
    ret = aw_sar_ps_notify_init(aw_sar_ps_to_cail);
    if (ret < 0) {
        pr_err("sar error creating power supply notify\n");
        goto exit_ps_notify;
    }
#endif
    /* awinic bob add end */


    task = kthread_run(kthread_worker_fn,
            &hfcore.kworker, "hf_manager");
    if (IS_ERR(task)) {
        pr_err("Failed to create kthread\n");
        ret = PTR_ERR(task);
        goto err_device;
    }
    sched_setscheduler(task, SCHED_FIFO, &param);

    //awinic bob add
    init_waitqueue_head(&g_aw_sar_wait);
    mutex_init(&g_aw_sar_lock);

    return 0;
/* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL
exit_ps_notify:
    power_supply_unreg_notifier(&aw_sar_ps_to_cail->ps_notif);
#endif
/* awinic bob add end */
err_device:
    device_destroy(hf_manager_class, MKDEV(major, 0));
err_class:
    class_destroy(hf_manager_class);
err_chredev:
    unregister_chrdev(major, "hf_manager");
/* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL
exit_aw_kfree:
    kfree(aw_sar_ps_to_cail);
    aw_sar_ps_to_cail = NULL;
#endif
/* awinic bob add end */

err_exit:
    return ret;
}

static void __exit hf_manager_exit(void)
{
    kthread_stop(task);
    device_destroy(hf_manager_class, MKDEV(major, 0));
    class_destroy(hf_manager_class);
    unregister_chrdev(major, "hf_manager");
}

subsys_initcall(hf_manager_init);
module_exit(hf_manager_exit);

MODULE_DESCRIPTION("high frequency manager");
MODULE_AUTHOR("Hongxu Zhao <hongxu.zhao@mediatek.com>");
MODULE_LICENSE("GPL v2");
