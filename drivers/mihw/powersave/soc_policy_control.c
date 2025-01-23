// MIUI ADD: Power_PowerSaveModeV2
/*
* Copyright (C) 2024 Xiaomi Inc.
*/
// #define DEBUG
#include <asm/page.h>
#include <linux/ctype.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/list.h>
#include <linux/pm_qos.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include "powersave.h"

#define MAX_BUFF 1024
#define CPU_NUMBER 8

struct freq_table {
    u32 frequency;
};

struct cpufreq_device {
    int id;
    unsigned int max_freq_index;
    unsigned int freq_index;
    struct freq_table *freq_table;
    struct cpufreq_policy *policy;
    struct freq_qos_request qos_req;
    struct list_head list;
};

enum MSG_TYPE {
    NOTIFIER_LIMIT        = 0x01,
    NOTIFIER_BOOST        = 0x02,
};

struct POLICY_MSG {
    enum MSG_TYPE msg_type;
    bool boosted;
    unsigned int target_freq_rate[CPU_NUMBER];
    struct work_struct sWork;
};

static LIST_HEAD(cpufreq_dev_list);
static DEFINE_MUTEX(cpufreq_list_lock);

bool boosted = false;
bool limit_valid = false;
struct workqueue_struct *policy_workqueue;

unsigned int atoi(const char* s) {
    int i = 0;
    while (isdigit(*s))
        i = i * 10 + (*s++ - '0');
    return i;
}

unsigned int find_next_max(struct cpufreq_frequency_table *table, unsigned int freq) {
    struct cpufreq_frequency_table *pos;
    unsigned int i = 0;
    cpufreq_for_each_valid_entry(pos, table) {
        if (pos->frequency > i && pos->frequency < freq) {
            i = pos->frequency;
        }
    }
    return i;
}

unsigned int match_freq_index(struct freq_table *freq_table, unsigned int max_index, unsigned int rate) {
    unsigned int index = 0;
    if (rate >= 100 || rate < 0) {
        return index;
    }
    index = max_index * (100 - rate) / 100;
    pr_debug("%s %s: max_index %u rate %u index %d", __FILE__, __func__, max_index, rate, index);
    return index;
}

void *msg_alloc_atomic(int i32Size) {
    void *pvBuf;
    if (i32Size <= PAGE_SIZE)
        pvBuf = kmalloc(i32Size, GFP_ATOMIC);
    else
        pvBuf = vmalloc(i32Size);
    return pvBuf;
}

void msg_free(void *pvBuf, int i32Size) {
    if (!pvBuf)
        return;
    if (i32Size <= PAGE_SIZE)
        kfree(pvBuf);
    else
        vfree(pvBuf);
}

static void on_policy_update(struct work_struct *work) {
    struct cpufreq_device *dev, *tmp;
    struct POLICY_MSG *msg = container_of(work, struct POLICY_MSG, sWork);
    if (!msg) {
        pr_debug("%s %s: msg err", __FILE__, __func__);
        return;
    }
    switch(msg->msg_type) {
        case NOTIFIER_LIMIT:
            mutex_lock(&cpufreq_list_lock);
            limit_valid = false;
            list_for_each_entry_safe(dev, tmp, &cpufreq_dev_list, list) {
                dev->freq_index = match_freq_index(dev->freq_table, dev->max_freq_index, msg->target_freq_rate[dev->id]);
                limit_valid |= dev->freq_index != 0;
                if (!boosted && dev->freq_index <= dev->max_freq_index) {
                    freq_qos_update_request(&dev->qos_req, dev->freq_table[dev->freq_index].frequency);
                    pr_debug("%s %s: cpu%d %u", __FILE__, __func__, dev->id, dev->freq_table[dev->freq_index].frequency);
                }
            }
            mutex_unlock(&cpufreq_list_lock);
            break;
        case NOTIFIER_BOOST:
            if (boosted != msg->boosted) {
                mutex_lock(&cpufreq_list_lock);
                boosted = msg->boosted;
                if (limit_valid) {
                    list_for_each_entry_safe(dev, tmp, &cpufreq_dev_list, list) {
                        if (dev->freq_index <= dev->max_freq_index) {
                            freq_qos_update_request(&dev->qos_req, msg->boosted ? dev->freq_table[0].frequency : dev->freq_table[dev->freq_index].frequency);
                            pr_debug("%s %s: cpu%d %u, boost %d", __FILE__, __func__, dev->id, msg->boosted ? dev->freq_table[0].frequency : dev->freq_table[dev->freq_index].frequency, msg->boosted);
                        }
                    }
                }
                mutex_unlock(&cpufreq_list_lock);
            }
            break;
    }
    msg_free(msg, sizeof(struct POLICY_MSG));
}

// fmt: cpu<num> <float>[ cpu<num> <float>]
void parse_limit_buf(unsigned int *rates, const char *buf) {
    char *token = NULL;
    struct cpufreq_policy *policy;
    for (int i = 0; i < CPU_NUMBER; i++) {
        rates[i] = 100;
    }
    token = strstr(buf, "cpu");
    while (token) {
        int cpu;
        token = token + strlen("cpu");
        cpu = token[0] - '0';
        do {
            token++;
        } while (token[0] == ' ');
        if (token && cpu < CPU_NUMBER) {
            rates[cpu] = atoi(token);
            pr_debug("%s %s: cpu: %d rate: %d%%", __FILE__, __func__, cpu, atoi(token));
        } else {
            rates[cpu] = 100;
        }
        token = strstr(token, "cpu");
    }

    for (int i = 0; i < CPU_NUMBER; i++) {
        unsigned int cpu_first, cpu_last;
        pr_debug("%s %s: cpu%d %u", __FILE__, __func__, i, rates[i]);
        policy = cpufreq_cpu_get(i);
        if (!policy) {
            continue;
        }
        cpu_first = cpumask_first(policy->related_cpus);
        cpu_last = cpumask_weight(policy->related_cpus) + cpu_first;
        pr_debug("%s %s: cpu_first %u cpu_last %u", __FILE__, __func__, cpu_first, cpu_last);
        for (int j = cpu_first + 1; j < cpu_last; j++) {
            if (rates[j] == 100) {
                rates[j] = rates[cpu_first];
            }
        }
        i = cpu_last - 1;
    }
}

void limit_buf_update(const char *buf, size_t n) {
    struct POLICY_MSG *msg = NULL;
    pr_debug("%s %s: limit_buf_update(%lu) %s", __FILE__, __func__, n, buf);
    msg = (struct POLICY_MSG *) msg_alloc_atomic(sizeof(struct POLICY_MSG));
    if (!msg) {
        pr_debug("%s %s: msg err", __FILE__, __func__);
        return;
    }
    msg->msg_type = NOTIFIER_LIMIT;
    parse_limit_buf(msg->target_freq_rate, buf);
    if (!policy_workqueue) {
        pr_debug("%s %s: msg err NULL work queue\n", __FILE__, __func__);
        msg_free(msg, sizeof(struct POLICY_MSG));
        return;
    }
    INIT_WORK(&msg->sWork, on_policy_update);
    queue_work(policy_workqueue, &msg->sWork);
    return;
}

void boost_update(bool boost) {
    struct POLICY_MSG *msg = NULL;
    pr_debug("%s %s: boost %d boosted %d", __FILE__, __func__, boost, boosted);
    msg = (struct POLICY_MSG *) msg_alloc_atomic(sizeof(struct POLICY_MSG));
    if (!msg) {
        pr_debug("%s %s: msg err", __FILE__, __func__);
        return;
    }
    msg->msg_type = NOTIFIER_BOOST;
    msg->boosted = boost;
    if (!policy_workqueue) {
        pr_debug("%s %s: msg err NULL work queue\n", __FILE__, __func__);
        msg_free(msg, sizeof(struct POLICY_MSG));
        return;
    }
    INIT_WORK(&msg->sWork, on_policy_update);
    queue_work(policy_workqueue, &msg->sWork);
    return;
    return;
}

int init_soc_policy_control(void) {
    int cpu, ret = 0;
    struct cpufreq_policy *policy;
    for_each_possible_cpu(cpu) {
        unsigned int i, freq;
        struct cpufreq_device *dev;
        char buf[MAX_BUFF];
        policy = cpufreq_cpu_get(cpu);
        if (!policy) {
            pr_err("%s %s: cpufreq_cpu_get failed for CPU %d\n", __FILE__, __func__, cpu);
            return -1;
        }
        i = cpufreq_table_count_valid_entries(policy);
        if (!i) {
            pr_debug("%s %s: cpufreq_table_count_valid_entries failed\n", __FILE__, __func__);
            return -1;
        }
        dev = kzalloc(sizeof(*dev), GFP_KERNEL);
        if (!dev) {
            pr_err("%s %s: kzalloc failed for CPU %d\n", __FILE__, __func__, cpu);
            return -1;
        }
        dev->id = cpu;
        dev->max_freq_index = i - 1;
        dev->freq_index = 0;
        dev->policy = policy;
        dev->freq_table = kmalloc_array(i, sizeof(*dev->freq_table), GFP_KERNEL);
        if (!dev->freq_table) {
            kfree(dev);
            pr_err("%s %s: kmalloc_array failed for CPU %d\n", __FILE__, __func__, cpu);
            return -1;
        }
        snprintf(buf, MAX_BUFF, "cpu%d", cpu);
        for (i = 0, freq = -1; i <= dev->max_freq_index; i++) {
            freq = find_next_max(policy->freq_table, freq);
            dev->freq_table[i].frequency = freq;
            snprintf(buf, MAX_BUFF, "%s %u", buf, freq);
        }
        pr_debug("%s %s: %s", __FILE__, __func__, buf);
        ret = freq_qos_add_request(&policy->constraints, &dev->qos_req, FREQ_QOS_MAX, dev->freq_table[0].frequency);
        if (ret < 0) {
            kfree(dev->freq_table);
            kfree(dev);
            pr_err("%s %s: Failed to add freq constraint for CPU %d (%d)\n", __FILE__, __func__, cpu, ret);
            return ret;
        }
        mutex_lock(&cpufreq_list_lock);
        list_add(&dev->list, &cpufreq_dev_list);
        mutex_unlock(&cpufreq_list_lock);
    }

    policy_workqueue =
        alloc_ordered_workqueue("%s", WQ_MEM_RECLAIM | WQ_HIGHPRI, "powersave_wq");
    if (policy_workqueue == NULL)
        return -EFAULT;

    return ret;
}

void exit_soc_policy_control(void) {
    struct cpufreq_device *dev, *tmp;
    if (policy_workqueue) {
        destroy_workqueue(policy_workqueue);
        policy_workqueue = NULL;
    }
    mutex_lock(&cpufreq_list_lock);
    list_for_each_entry_safe(dev, tmp, &cpufreq_dev_list, list) {
        freq_qos_remove_request(&dev->qos_req);
        cpufreq_cpu_put(dev->policy);
        kfree(dev->freq_table);
        kfree(dev);
    }
    mutex_unlock(&cpufreq_list_lock);
}
// END Power_PowerSaveModeV2