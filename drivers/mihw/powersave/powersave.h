// MIUI ADD: Power_PowerSaveModeV2
#ifndef POWERSAVE_H
#define POWERSAVE_H

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>

#define load_attr(_name) \
static struct kobj_attribute _name##_attr = {   \
    .attr   =   {                           \
            .name = __stringify(_name),     \
            .mode = 0664,                   \
    },                                      \
    .show   =   _name##_show,               \
    .store  =   _name##_store,              \
}

extern int init_soc_policy_control(void);
extern void exit_soc_policy_control(void);
extern void limit_buf_update(const char *buf, size_t n);
extern void boost_update(bool boost);

#endif
// END Power_PowerSaveModeV2
