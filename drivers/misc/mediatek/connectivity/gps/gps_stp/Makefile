# drivers/barcelona/gps/Makefile
#
# Makefile for the Barcelona GPS driver.
#
# Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
# Author: Dimitry Andric <dimitry.andric@tomtom.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.

###############################################################################
ifeq ($(CONFIG_MTK_GPS_SUPPORT), y)

ifndef TOP
    TOP := $(srctree)/..
endif
ifeq ($(TARGET_BUILD_VARIANT),$(filter $(TARGET_BUILD_VARIANT),userdebug user))
    ldflags-y += -s
endif

# Force build fail on modpost warning
KBUILD_MODPOST_FAIL_ON_WARNINGS := y
###############################################################################

# only WMT align this design flow, but gps use this also.
#ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE

ifeq ($(CONFIG_ARM64), y)
    ccflags-y += -D CONFIG_MTK_WCN_ARM64
endif

ifeq ($(CONFIG_MTK_CONN_LTE_IDC_SUPPORT),y)
    ccflags-y += -D WMT_IDC_SUPPORT=1
else
    ccflags-y += -D WMT_IDC_SUPPORT=0
endif
ccflags-y += -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT

#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/freqhopping
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/freqhopping/$(MTK_PLATFORM)
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/emi/submodule
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/emi/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/common
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/mach
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1/$(MTK_PLATFORM)
#ccflags-y += -I$(srctree)/drivers/devfreq
###############################################################################

MODULE_NAME := gps_drv_stp
obj-m += $(MODULE_NAME).o

GPS_DRV_CONTROL_LNA := y
GPS_SRC_FOLDER := $(srctree)/drivers/misc/mediatek/connectivity/gps/gps_stp
ccflags-y += -DMTK_GENERIC_HAL
ccflags-y += -DCONFIG_GPSL5_SUPPORT

WMT_SRC_FOLDER := $(srctree)/drivers/misc/mediatek/connectivity/common
ccflags-y += -DSOC_CO_CLOCK_FLAG=0
ccflags-y += -DWMT_CREATE_NODE_DYNAMIC=0
ccflags-y += -DREMOVE_MK_NODE=1

ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/include
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/linux/include
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/core/include
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/platform/include
ifneq ($(CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH),)
ccflags-y += -I$(WMT_SRC_FOLDER)/debug_utility
endif
ifeq ($(GPS_DRV_CONTROL_LNA),y)
ccflags-y += -I$(GPS_SRC_FOLDER)/lna_ctrl/inc
endif
ccflags-y += -I$(GPS_SRC_FOLDER)/

        $(MODULE_NAME)-objs += stp_chrdev_gps.o

        $(MODULE_NAME)-objs += stp_chrdev_gps2.o

ifeq ($(GPS_DRV_CONTROL_LNA),y)
        $(MODULE_NAME)-objs += lna_ctrl/src/gps_lna_drv.o
endif
ifneq ($(CONFIG_MTK_GPS_EMI),)
$(MODULE_NAME)-objs += gps_emi.o
endif
ifneq ($(CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH),)
$(MODULE_NAME)-objs += fw_log_gps.o
endif

endif #ifeq ($(CONFIG_MTK_GPS_SUPPORT), y)
# EOF
