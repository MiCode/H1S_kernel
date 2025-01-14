
// MIUI ADD: Performance_UnfairMem
#define pr_fmt(fmt) "unfair_mem: " fmt
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/compat.h>
#include <trace/hooks/mm.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
struct proc_dir_entry *mimem_rootdir;
#define MAX_PIDS 5
enum BOOST_SCENE {
    UNDEFINE = 0,
    APP_START_EXIT,
    APP_UX,
    MAX_SCENT
};
static enum BOOST_SCENE boost_ing ;
static int pid_array[MAX_PIDS];
static int msf_pid;

static void mi_get_page_wmark_boost(void *nouse,
            unsigned int alloc_flags, unsigned long *page_wmark)
{
    pid_t current_pid ;
    bool boost =false;

    current_pid = current->pid;
    if(boost_ing == APP_START_EXIT ){
        for(int i = 0; i < MAX_PIDS && pid_array[i] !=0; ++i) {
            if(current_pid == pid_array[i]){
            boost =true;
            break;
            }
        }

        if(boost){
            *page_wmark = *page_wmark -*page_wmark/4;
            //printk(KERN_ERR "boost current_pid = %d ,page_wmark == %ld\n",
            //        current_pid, *page_wmark);
        }
    }
}

static int mimem_scene_show(struct seq_file *m, void *v)
{

    int i;
    seq_printf(m, "BOOST_SCENE: %d\n",boost_ing);
    seq_printf(m, "%lu PID Array:\n",ARRAY_SIZE(pid_array));
    for (i = 0; i < MAX_PIDS && pid_array[i] !=0; i++) {
            seq_printf(m, "%d ", pid_array[i]);
    }
    seq_printf(m, "\n");
    return 0;
}

static int mimem_scene_open(struct inode *inode, struct file *file)
{
    return single_open(file, mimem_scene_show, NULL);
}

static ssize_t mimem_sf_pid_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
    char buf[128];
    int ret,pid;

    if (count >= (sizeof(buf)-1))
        return -ENOSPC;

    ret = copy_from_user(buf, ubuf, count);
    if (ret)
        return -EFAULT;

    buf[count] = '\0';

    ret = sscanf(buf, "%d ", &pid);
    if (ret != 1)
        return -EINVAL;

    msf_pid = pid;

    return count;
}

static int mimem_sf_pid_show(struct seq_file *m, void *v)
{
    seq_printf(m, "surfaceflinger PID :\n");
    seq_printf(m, "%d ", msf_pid);
    seq_printf(m, "\n");

    return 0;
}

static int mimem_sf_pid_open(struct inode *inode, struct file *file)
{
    return single_open(file, mimem_sf_pid_show, NULL);
}

static ssize_t mimem_scene_write(struct file *filp,
                const char __user *ubuf, size_t count, loff_t *ppos)
{
    char buf[128];
    int ret;
    int scene;

    if (count >= (sizeof(buf)-1))
        return -ENOSPC;

    ret = copy_from_user(buf, ubuf, count);
    if (ret)
        return -EFAULT;

    buf[count] = '\0';

    ret = sscanf(buf, "%d", &scene);
    if (ret != 1)
        return -EINVAL;

    boost_ing = scene;

    if(boost_ing == APP_START_EXIT){
       memset(pid_array, 0, MAX_PIDS * sizeof(pid_t));

       ret = sscanf(buf, "%d %d %d %d %d",&boost_ing,
                        &pid_array[0], &pid_array[1],&pid_array[2],&pid_array[3]);
       if(ret < 2){
            boost_ing = UNDEFINE;
            pr_err("set /proc/mi_mem_engine/scene_tid  failed\n");
            return -ENOSPC;
       }


        if(msf_pid)
            pid_array[MAX_PIDS-1] = msf_pid;
    }

    return count;
}

static const struct proc_ops scene_tid_proc_fops = {
        .proc_open   = mimem_scene_open,
        .proc_read   = seq_read,
        .proc_write   = mimem_scene_write,
        .proc_lseek   = seq_lseek,
        .proc_release   = single_release,
};

static const struct proc_ops sf_pid_proc_fops = {
        .proc_open   = mimem_sf_pid_open,
        .proc_read   = seq_read,
        .proc_write   = mimem_sf_pid_write,
        .proc_lseek   = seq_lseek,
        .proc_release   = single_release,
};

static int __init unfair_mem_init(void)
{

    struct proc_dir_entry *mimem_scene_entry = NULL;
    struct proc_dir_entry *mimem_sf_pid_entry = NULL;
    printk(KERN_ERR "in %s\n", __func__);
    register_trace_android_vh_get_page_wmark(mi_get_page_wmark_boost, NULL);

    boost_ing = UNDEFINE;
    mimem_rootdir = proc_mkdir("mi_mem_engine", NULL);
    if (!mimem_rootdir){
        pr_err("create /proc/mi_mem_engine failed\n");
    }else {
        mimem_scene_entry = proc_create("scene_tid",
                              0664, mimem_rootdir, &scene_tid_proc_fops);
        if (!mimem_scene_entry)
            pr_err("create /proc/mi_mem_engine/scene_tid failed\n");

        mimem_sf_pid_entry = proc_create("sf_pid",
                              0664, mimem_rootdir, &sf_pid_proc_fops);
        if (!mimem_sf_pid_entry)
            pr_err("create /proc/mi_mem_engine/sf_pid failed\n");
    }
    return 0;
}
static void __exit unfair_mem_exit(void)
{
    unregister_trace_android_vh_get_page_wmark(mi_get_page_wmark_boost, NULL);
    printk(KERN_ERR "in %s\n", __func__);
}
module_init(unfair_mem_init);
module_exit(unfair_mem_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("huzhongyang");
MODULE_DESCRIPTION("A moudle to adjust min watermark for important task.");
// END Performance_UnfairMem
