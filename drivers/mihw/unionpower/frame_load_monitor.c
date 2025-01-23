// MIUI ADD: Power_UnionPowerCore
/*
* Copyright (C) 2024 Xiaomi Inc.
*/
// #define DEBUG
#include <asm/page.h>
#include <linux/math.h>
#include <linux/ktime.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "unionpower.h"

int VSYNC_GAP = 10000; // ns
module_param(VSYNC_GAP, int, 0644);
MODULE_PARM_DESC(VSYNC_GAP, "VSYNC_GAP");

int VSYNC_TIMEOUT = 10; // times
module_param(VSYNC_TIMEOUT, int, 0644);
MODULE_PARM_DESC(VSYNC_TIMEOUT, "VSYNC_TIMEOUT");

int JANK_TIMEOUT = 16600 * 5;
module_param(JANK_TIMEOUT, int, 0644);
MODULE_PARM_DESC(JANK_TIMEOUT, "JANK_TIMEOUT");

int JANK_LIMIT = 0; // times
module_param(JANK_LIMIT, int, 0644);
MODULE_PARM_DESC(JANK_LIMIT, "JANK_LIMIT");

int NICE_MORE_LIMIT = 3; // times
module_param(NICE_MORE_LIMIT, int, 0644);
MODULE_PARM_DESC(NICE_MORE_LIMIT, "NICE_MORE_LIMIT");

struct workqueue_struct *queue_buffer_wq;
struct list_head buffer_record_list;
static struct mutex list_lock;

void *push_alloc_atomic(int i32Size) {
    void *pvBuf;
    if (i32Size <= PAGE_SIZE)
        pvBuf = kmalloc(i32Size, GFP_ATOMIC);
    else
        pvBuf = vmalloc(i32Size);
    return pvBuf;
}

void push_free(void *pvBuf, int i32Size) {
    if (!pvBuf)
        return;
    if (i32Size <= PAGE_SIZE)
        kfree(pvBuf);
    else
        vfree(pvBuf);
}

bool fpsChange(int cur, int last) {
    int cur_fps;
    bool hasChange = true;
    if (cur == last || abs(cur - last) < 500) {
        return false;
    }
    if (cur < 8130) // 123-144
        cur_fps = FPS_144;
    else if (cur < 10753) { // 93-123
        cur_fps = FPS_120;
    } else if (cur < 16129) { // 62-93
        cur_fps = FPS_90;
    } else if (cur < 31250) { // 32-60
        cur_fps = FPS_60;
    } else { // 30
        cur_fps = FPS_30;
    }
    switch (cur_fps) {
    case FPS_144:
        hasChange = last >= 8130;
        break;
    case FPS_120:
        hasChange = last >= 10753 || last < 8130;
        break;
    case FPS_90:
        hasChange = last >= 16129 || last < 10753;
        break;
    case FPS_60:
        hasChange = last >= 31250 || last < 16129;
        break;
    case FPS_30:
        hasChange = last < 31250;
        break;
    default:
        break;
    }
    return hasChange;
}

void init_buffer(struct buffer_record *pos, int pid, __s64 vsync_interval, __s64 vsync_time, __s64 vsync_id) {
    pos->pid = pid;
    pos->render_thread = 0;
    pos->vsync_count = 1;
    pos->vsync_record_list[0].vsync_duration = vsync_interval;
    pos->vsync_record_list[0].vsync_delay = false;
    pos->vsync_record_list[0].vsync_id = vsync_id;
    pos->vsync_record_list[0].vsync_interval = vsync_interval;
    pos->vsync_record_list[0].vsync_time = vsync_time;
    pos->jank_happend = false;
    pos->frame_count = 0;
    pos->continus_nice = 0;
    pos->continus_jank = 0;
    pos->history_buffer_interval = vsync_interval;
    pr_debug("%s %s: pid %d, render_thread %d %llu %llu",
        __FILE__, __func__, pos->pid, pos->render_thread, vsync_time, vsync_interval);
}

void refresh_frame_load(bool new_status, __s64 cur_time) {
    struct buffer_record *pos = NULL;
    int last_index;
    ktime_t cpm_time;
    bool total_status = false;
    if (new_status == frame_jank) {
        return;
    }
    if (new_status && !frame_jank) {
        pr_debug("%s %s: frame_jank update: %d", __FILE__, __func__, new_status);
        frame_jank = new_status;
        union_power_sysfs_notify("frame_jank");
        return;
    }
    cpm_time = ktime_sub(cur_time, JANK_TIMEOUT != 0 ? JANK_TIMEOUT : 16600 * 5);
    pr_debug("%s %s: new_status %d cpmtime %llu", __FILE__, __func__, new_status, cpm_time);
    list_for_each_entry(pos, &buffer_record_list, list) {
        if (pos->jank_happend) {
            last_index = (pos->vsync_count - 1) % RECORD_SIZE;
            pr_debug("%s %s: pid %d jank_happed %llu ", __FILE__, __func__, pos->pid, pos->vsync_record_list[last_index].vsync_time - pos->history_buffer_interval);
            if (cpm_time <= pos->vsync_record_list[last_index].vsync_time - pos->history_buffer_interval) {
                total_status = true;
                break;
            }
        }
    }
    if (total_status != frame_jank) {
        pr_debug("%s %s: frame_jank update: %d", __FILE__, __func__, total_status);
        frame_jank = total_status;
        union_power_sysfs_notify("frame_jank");
    }
}

void update_vsync(struct buffer_record *pos, __u64 render_thread, __s64 vsync_interval, __s64 vsync_time, __s64 vsync_id, bool jank) {
    int cur_index;
    struct vsync_record last_vsync;
    pos->render_thread = render_thread;
    last_vsync = pos->vsync_record_list[(pos->vsync_count - 1) % RECORD_SIZE];
    if (pos->vsync_count - pos->frame_count >= 5) {
        cur_index = 1;
        pos->vsync_record_list[0].vsync_duration = last_vsync.vsync_duration;
        pos->vsync_record_list[0].vsync_delay = last_vsync.vsync_delay;
        pos->vsync_record_list[0].vsync_id = last_vsync.vsync_id;
        pos->vsync_record_list[0].vsync_interval = last_vsync.vsync_interval;
        pos->vsync_record_list[0].vsync_time = last_vsync.vsync_time;
        pos->vsync_count = 2;
        pos->frame_count = 0;
        pos->continus_nice = 0;
        pos->continus_jank = 0;
        pr_debug("%s %s: %d %d reset cycle, frame_id %lld", __FILE__, __func__, pos->pid, pos->render_thread, vsync_id);
    } else {
        cur_index = (pos->vsync_count) % RECORD_SIZE;
        pos->vsync_count++;
    }
    pos->vsync_record_list[cur_index].vsync_duration = ktime_sub(vsync_time, last_vsync.vsync_time);
    pos->vsync_record_list[cur_index].vsync_delay = pos->frame_count != pos->vsync_count -1 && pos->vsync_record_list[cur_index].vsync_duration > ktime_add(last_vsync.vsync_interval, VSYNC_GAP);
    pos->vsync_record_list[cur_index].vsync_id = vsync_id;
    pos->vsync_record_list[cur_index].vsync_interval = vsync_interval;
    pos->vsync_record_list[cur_index].vsync_time = vsync_time;
    pr_debug("%s %s: %d %d, vsync_count %d, frame_count %d, vsync_time %lld, vsync_interval %lld vsync_delay %d frame_id %lld",
        __FILE__, __func__, pos->pid, pos->render_thread, pos->vsync_count, pos->frame_count, vsync_time, vsync_interval, pos->vsync_record_list[cur_index].vsync_delay, vsync_id);

    if (!jank && pos->vsync_record_list[cur_index].vsync_delay) {
        pos->history_buffer_interval = pos->vsync_record_list[cur_index].vsync_duration;
        pos->jank_happend = true;
    } else {
        pos->jank_happend = jank;
    }
    refresh_frame_load(pos->jank_happend, vsync_time);
}

void update_buffer(struct buffer_record *pos, __s64 frame_time_us, __s64 frame_id) {
    int current_index, last_index;
    bool jank_status;
    ktime_t queue_buffer_interval;
    struct vsync_record cur_vsync;

    if (pos->frame_count > 0) {
        last_index = (pos->frame_count - 1) % RECORD_SIZE;
        cur_vsync = pos->vsync_record_list[last_index];
        if (cur_vsync.vsync_id == frame_id) {
            queue_buffer_interval = ktime_sub(frame_time_us, cur_vsync.vsync_time);
            pos->history_buffer_interval = ktime_divns(ktime_add(cur_vsync.vsync_duration, queue_buffer_interval), 2);
            pos->last_queue_buffer_time[last_index] = frame_time_us;
            if (queue_buffer_interval >= cur_vsync.vsync_interval) {
                if (pos->continus_nice > 0) {
                    pos->continus_jank = 1;
                    pos->continus_nice = 0;
                }
            }
            pr_debug("%s %s: %d %d, vsync_time %lld, vsync_interval %lld vsync_delay %d frame_id %lld",
                __FILE__, __func__, pos->pid, pos->render_thread, cur_vsync.vsync_time, cur_vsync.vsync_interval, cur_vsync.vsync_delay, cur_vsync.vsync_id);
            pr_debug("%s %s: %d %d, frame_count %d, continus_nice %d, continus_jank %d, last_frame_time %lld, his %lld interval %lld",
                __FILE__, __func__, pos->pid, pos->render_thread, pos->frame_count, pos->continus_nice, pos->continus_jank, pos->last_queue_buffer_time[last_index], pos->history_buffer_interval, queue_buffer_interval);
            jank_status = cur_vsync.vsync_delay || (pos->continus_jank > JANK_LIMIT && (pos->continus_nice < NICE_MORE_LIMIT));
            if (jank_status != pos->jank_happend) {
                pos->jank_happend = jank_status;
            }
            refresh_frame_load(jank_status, cur_vsync.vsync_time);
            return;
        }
    }

    current_index = pos->frame_count % RECORD_SIZE;
    cur_vsync = pos->vsync_record_list[current_index];
    while (pos->frame_count < pos->vsync_count && cur_vsync.vsync_id != frame_id) {
        pos->last_queue_buffer_time[current_index] = cur_vsync.vsync_time;
        pos->history_buffer_interval = ktime_divns(ktime_add(pos->history_buffer_interval, cur_vsync.vsync_interval), 2);
        if (!cur_vsync.vsync_delay) {
            pos->continus_nice ++;
            pos->history_buffer_interval = ktime_divns(ktime_add(pos->history_buffer_interval, cur_vsync.vsync_interval), 2);
        } else {
            pos->history_buffer_interval = ktime_divns(ktime_add(pos->history_buffer_interval, cur_vsync.vsync_duration), 2);
            if (pos->continus_nice > 0) {
                pos->continus_jank = 1;
                pos->continus_nice = 0;
            } else {
                pos->continus_jank ++;
            }
        }
        pos->frame_count++;
        pr_debug("%s %s: %d %d, vsync_time %lld, vsync_interval %lld vsync_delay %d frame_id %lld skip",
            __FILE__, __func__, pos->pid, pos->render_thread, cur_vsync.vsync_time, cur_vsync.vsync_interval, cur_vsync.vsync_delay, cur_vsync.vsync_id);
        current_index = pos->frame_count % RECORD_SIZE;
        cur_vsync = pos->vsync_record_list[current_index];
    }

    if (pos->frame_count >= pos->vsync_count || cur_vsync.vsync_id != frame_id) {
        return;
    }

    pos->last_queue_buffer_time[current_index] = frame_time_us;
    last_index = (pos->frame_count - 1) % RECORD_SIZE;
    queue_buffer_interval = ktime_sub(frame_time_us, cur_vsync.vsync_time);
    pos->history_buffer_interval = ktime_divns(ktime_add(pos->history_buffer_interval, queue_buffer_interval), 2);
    if (queue_buffer_interval < cur_vsync.vsync_interval) {
        pos->continus_nice ++;
    } else {
        if (pos->continus_nice > 0) {
            pos->continus_jank = 1;
            pos->continus_nice = 0;
        } else {
            pos->continus_jank ++;
        }
    }
    pos->frame_count++;
    pr_debug("%s %s: %d %d, vsync_time %lld, vsync_interval %lld vsync_delay %d frame_id %lld",
        __FILE__, __func__, pos->pid, pos->render_thread, cur_vsync.vsync_time, cur_vsync.vsync_interval, cur_vsync.vsync_delay, cur_vsync.vsync_id);
    pr_debug("%s %s: %d %d, frame_count %d, continus_nice %d, continus_jank %d, last_frame_time %lld, his %lld interval %lld",
        __FILE__, __func__, pos->pid, pos->render_thread, pos->frame_count, pos->continus_nice, pos->continus_jank, pos->last_queue_buffer_time[current_index], pos->history_buffer_interval, queue_buffer_interval);
    jank_status = cur_vsync.vsync_delay || (pos->continus_jank > JANK_LIMIT && (pos->continus_nice < NICE_MORE_LIMIT));
    if (jank_status != pos->jank_happend) {
        pos->jank_happend = jank_status;
    }
    refresh_frame_load(jank_status, cur_vsync.vsync_time);

}

static void on_notify_vsync(int pid, __s64 frameTime, __s64 frameInterval, __s64 vsync_id) {
    struct buffer_record *pos = NULL;
    struct task_struct *task = NULL;
    struct pid *pid_struct = NULL;
    struct vsync_record last_vsync;
    ktime_t current_us, interval_us;
    current_us = ktime_to_us(frameTime);
    interval_us = ktime_to_us(frameInterval);
    mutex_lock(&list_lock);
    list_for_each_entry(pos, &buffer_record_list, list) {
        if (pos->pid == pid) {
            update_vsync(pos, pos->render_thread, interval_us, current_us, vsync_id, pos->jank_happend);
            mutex_unlock(&list_lock);
            return;
        }
    }
    pid_struct = find_get_pid(pid);
    if (pid_struct == NULL) {
        pr_err("%s fail to find_get_pid\n", __func__);
        mutex_unlock(&list_lock);
        return;
    }
    task = get_pid_task(pid_struct, PIDTYPE_PID);
    if (task == NULL) {
        pr_err("%s fail to get pid_task\n", __func__);
        put_pid(pid_struct);
        mutex_unlock(&list_lock);
        return;
    }
    list_for_each_entry(pos, &buffer_record_list, list) {
        last_vsync = pos->vsync_record_list[(pos->vsync_count - 1) % RECORD_SIZE];
        if (pos->pid != pid && pos->frame_count >= pos->vsync_count && current_us - last_vsync.vsync_time > max(pos->history_buffer_interval, last_vsync.vsync_interval) * VSYNC_TIMEOUT) {
            if (pos->jank_happend) {
                pos->jank_happend = false;
                refresh_frame_load(false, current_us);
            }
            init_buffer(pos, pid, interval_us, current_us, vsync_id);
            mutex_unlock(&list_lock);
            return;
        }
    }
    pos = kzalloc(sizeof(*pos), GFP_KERNEL);
    if (!pos) {
        mutex_unlock(&list_lock);
        return;
    }
    memset(pos, 0, sizeof(struct buffer_record));
    init_buffer(pos, pid, interval_us, current_us, vsync_id);
    list_add_tail(&(pos->list), &buffer_record_list);
    mutex_unlock(&list_lock);
}

static void on_notify_queue_buffer(int pid, __u64 call_from, __s64 frame_time, __s64 frame_id) {
    struct buffer_record *pos = NULL;
    ktime_t frame_time_us = ktime_to_us(frame_time);

    mutex_lock(&list_lock);
    list_for_each_entry(pos, &buffer_record_list, list) {
        if (pos->pid == pid && pos->render_thread == call_from) {
            update_buffer(pos, frame_time_us, frame_id);
            mutex_unlock(&list_lock);
            return;
        }
    }
    list_for_each_entry(pos, &buffer_record_list, list) {
        if (pos->pid == pid && pos->render_thread == 0) {
            pos->render_thread = call_from;
            update_buffer(pos, frame_time_us, frame_id);
            mutex_unlock(&list_lock);
            return;
        }
    }

    mutex_unlock(&list_lock);
}

static void queue_buffer_work(struct work_struct *psWork) {
    struct task_struct *task = NULL;
    struct pid *pid_struct = NULL;
    struct WORK_MSG *workMsg = container_of(psWork, struct WORK_MSG, sWork);

    if (!workMsg) {
        pr_debug("[unionpower] err\n");
        return;
    }
    switch(workMsg->pushType) {
        case NOTIFIER_QUEUE:
            pid_struct = find_get_pid(workMsg->call_from);
            if (pid_struct == NULL) {
                pr_err("%s fail to find_get_pid\n", __func__);
                return;
            }
            task = get_pid_task(pid_struct, PIDTYPE_PID);
            if (task == NULL) {
                pr_err("%s fail to get pid_task\n", __func__);
                put_pid(pid_struct);
                return;
            }
            if (!strcmp(task->comm, "RenderThread")) {
                on_notify_queue_buffer(workMsg->pid, workMsg->call_from, workMsg->frameTime, workMsg->frameId);
            }
            break;
        case NOTIFIER_VSYNC:
            on_notify_vsync(workMsg->pid, workMsg->frameTime, workMsg->frameInterval, workMsg->frameId);
            break;
        default:
            pr_debug("[unionpower] unhandle %d\n", workMsg->pushType);
    }
    mutex_lock(&list_lock);
    mutex_unlock(&list_lock);
    push_free(workMsg, sizeof(struct WORK_MSG));
}

void notify_buffer_work(enum WORK_PUSH_TYPE type, int call_from, int pid,  unsigned long long frame_time, unsigned long long frame_interval, unsigned long long frame_id) {
    struct WORK_MSG *workMsg = NULL;

    workMsg = (struct WORK_MSG *)push_alloc_atomic(sizeof(struct WORK_MSG));
    if (!workMsg) {
        pr_debug("[unionpower] OOM\n");
        return;
    }
    if (!queue_buffer_wq) {
        pr_debug("[unionpower] NULL work queue\n");
        push_free(workMsg, sizeof(struct WORK_MSG));
        return;
    }
    workMsg->pushType = type;
    workMsg->call_from = call_from;
    workMsg->pid = pid;
    workMsg->frameId = frame_id;
    workMsg->frameTime = frame_time;
    workMsg->frameInterval = frame_interval;
    INIT_WORK(&workMsg->sWork, queue_buffer_work);
    queue_work(queue_buffer_wq, &workMsg->sWork);
}

int init_frame_load_monitor(void) {
    int ret = 0;

    queue_buffer_wq =
        alloc_ordered_workqueue("%s", WQ_MEM_RECLAIM | WQ_HIGHPRI, "union_power_buffer_wq");
    if (queue_buffer_wq == NULL)
        return -EFAULT;
    mutex_init(&list_lock);
    INIT_LIST_HEAD(&buffer_record_list);
    notify_buffer_work_fp = notify_buffer_work;
    return ret;
}

void exit_frame_load_monitor(void) {
    struct list_head *pos, *n;
    struct buffer_record *tmp;
    notify_buffer_work_fp = NULL;
    if (queue_buffer_wq) {
        destroy_workqueue(queue_buffer_wq);
        queue_buffer_wq = NULL;
    }
    list_for_each_safe(pos, n, &buffer_record_list) {
        tmp = list_entry(pos, struct buffer_record, list);
        list_del_init(pos);
        kfree(tmp);
    }
}
// END Power_UnionPowerCore