// MIUI ADD: Power_UnionPowerCore
#ifndef UNIONPOWER_H
#define UNIONPOWER_H
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/time.h>
#define RECORD_SIZE 5
#define FPS_30 30
#define FPS_60 60
#define FPS_90 90
#define FPS_120 120
#define FPS_144 144
#define load_attr(_name) \
static struct kobj_attribute _name##_attr = {   \
    .attr   =   {                           \
            .name = __stringify(_name),     \
            .mode = 0664,                   \
    },                                      \
    .show   =   _name##_show,               \
    .store  =   _name##_store,              \
}
struct _BUFFER_PACKAGE {
    __u32 pid;
    __u32 start;
    __u64 frame_time;
    __u64 frame_id; 
    __s64 frame_interval;
    struct timespec64 current_timespec;
};
#define BUFFER_QUEUE                  _IOW('g', 1,  struct _BUFFER_PACKAGE)
#define BUFFER_VSYNC                  _IOW('g', 5,  struct _BUFFER_PACKAGE)
enum WORK_PUSH_TYPE {
    NOTIFIER_QUEUE        = 0x01,
    NOTIFIER_VSYNC        = 0x05,
};
struct WORK_MSG {
    enum WORK_PUSH_TYPE pushType;
    __u32 call_from;
    __u32 pid;
    __u64 frameId;
    __s64 frameTime;
    __s64 frameInterval;
    struct work_struct sWork;
};
struct vsync_record {
    bool vsync_delay;
    __s64 vsync_id;
    __s64 vsync_interval;
    __s64 vsync_time;
    __s64 vsync_duration;
};
struct buffer_record {
    int pid;
    int render_thread;
    bool jank_happend;
    int vsync_count;
    struct vsync_record vsync_record_list[RECORD_SIZE];
    __s64 last_queue_buffer_time[RECORD_SIZE];
    __s64 history_buffer_interval;
    int target_fps;
    int frame_count;
    int continus_nice;
    int continus_jank;
    struct list_head list;
};
extern bool frame_jank;
extern void union_power_sysfs_notify(const char *attr);
extern int init_frame_load_monitor(void);
extern void exit_frame_load_monitor(void);
extern void (*notify_buffer_work_fp)(enum WORK_PUSH_TYPE type, int call_from, int pid, unsigned long long frame_time, unsigned long long frame_interval, unsigned long long frame_id);
//extern int init_frame_monitor(void);
//extern void exit_frame_monitor(void);
extern int init_ioctl(struct proc_dir_entry *parent);
extern void exit_ioctl(void);
#endif
// END Power_UnionPowerCore
