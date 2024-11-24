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
$(warning build in gps_dl)
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

#ifeq ($(CONFIG_ARM64), y)
#    ccflags-y += -D CONFIG_MTK_WCN_ARM64
#endif

#ifeq ($(CONFIG_MTK_CONN_LTE_IDC_SUPPORT),y)
#    ccflags-y += -D WMT_IDC_SUPPORT=1
#else
#    ccflags-y += -D WMT_IDC_SUPPORT=0
#endif
#ccflags-y += -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT

#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/freqhopping
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/freqhopping/$(MTK_PLATFORM)
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/emi/submodule
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/emi/$(MTK_PLATFORM)
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/common
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/mach
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1/$(MTK_PLATFORM)
#ccflags-y += -I$(srctree)/drivers/devfreq
###############################################################################

MODULE_NAME := gps_drv_dl_$(GPS_PLATFORM)
obj-m += $(MODULE_NAME).o

GPS_DRV_CONTROL_LNA := n
GPS_DL_HAS_MOCK := n
GPS_SRC_FOLDER := $(srctree)/drivers/misc/mediatek/connectivity/gps/data_link
ccflags-y += -DMTK_GENERIC_HAL

ccflags-y += -I$(GPS_SRC_FOLDER)/plat/$(GPS_PLATFORM)
ccflags-y += -I$(GPS_SRC_FOLDER)/plat/$(GPS_PLATFORM)/inc
ccflags-y += -I$(GPS_SRC_FOLDER)/plat/$(GPS_PLATFORM)/hw/inc

CONNINFRA_SRC_FOLDER := $(srctree)/drivers/misc/mediatek/connectivity/conninfra
ccflags-y += -I$(CONNINFRA_SRC_FOLDER)/include
ccflags-y += -I$(CONNINFRA_SRC_FOLDER)/debug_utility/metlog
ccflags-y += -DGPS_DL_HAS_CONNINFRA_DRV=1


ccflags-y += -I$(GPS_SRC_FOLDER)/inc
ccflags-y += -I$(GPS_SRC_FOLDER)/linux/inc
ccflags-y += -I$(GPS_SRC_FOLDER)/link/inc
ccflags-y += -I$(GPS_SRC_FOLDER)/lib/inc
ccflags-y += -I$(GPS_SRC_FOLDER)/hal/inc
ccflags-y += -I$(GPS_SRC_FOLDER)/hw/inc
ccflags-y += -I$(GPS_SRC_FOLDER)/data_link_mock/mock/inc

$(MODULE_NAME)-objs += gps_dl_context.o
$(MODULE_NAME)-objs += gps_dl_module.o
$(MODULE_NAME)-objs += lib/gps_dl_dma_buf.o
$(MODULE_NAME)-objs += lib/gps_dl_lib_misc.o
$(MODULE_NAME)-objs += lib/gps_dl_hist_rec.o
$(MODULE_NAME)-objs += lib/gps_dl_hist_rec2.o
$(MODULE_NAME)-objs += lib/gps_dl_time_tick.o
$(MODULE_NAME)-objs += lib/gps_dl_name_list.o

$(MODULE_NAME)-objs += hw/gps_dl_hw_conn_infra.o
$(MODULE_NAME)-objs += hw/gps_dl_hw_semaphore.o
$(MODULE_NAME)-objs += hw/gps_dl_hw_bgf.o
$(MODULE_NAME)-objs += hw/gps_dl_hw_gps.o
$(MODULE_NAME)-objs += hw/gps_dl_hw_power_ctrl.o
$(MODULE_NAME)-objs += hw/gps_dl_hw_usrt_apb.o
$(MODULE_NAME)-objs += hw/gps_dl_hw_util.o

$(MODULE_NAME)-objs += hal/gps_dl_hal.o
$(MODULE_NAME)-objs += hal/gps_dl_hal_util.o
$(MODULE_NAME)-objs += hal/gps_dsp_fsm.o
$(MODULE_NAME)-objs += hal/gps_dl_power_ctrl.o
$(MODULE_NAME)-objs += hal/gps_dl_isr.o
$(MODULE_NAME)-objs += hal/gps_dl_dma.o
$(MODULE_NAME)-objs += hal/gps_dl_mcub.o
$(MODULE_NAME)-objs += hal/gps_dl_zbus.o
$(MODULE_NAME)-objs += hal/gps_dl_conn_infra.o
$(MODULE_NAME)-objs += hal/gps_dl_hal_met2_0.o


$(MODULE_NAME)-objs += link/gps_dl_link_event_proc.o
$(MODULE_NAME)-objs += link/gps_dl_link_hal_ctrl.o
$(MODULE_NAME)-objs += link/gps_dl_link_irq_ctrl.o
$(MODULE_NAME)-objs += link/gps_dl_link_state.o
$(MODULE_NAME)-objs += link/gps_dl_link_sync.o
$(MODULE_NAME)-objs += link/gps_dl_link_util.o
$(MODULE_NAME)-objs += link/gps_dl_subsys_reset.o
$(MODULE_NAME)-objs += gps_each_link.o

$(MODULE_NAME)-objs += linux/gps_data_link_devices.o
$(MODULE_NAME)-objs += linux/gps_each_device.o
$(MODULE_NAME)-objs += linux/gps_dl_linux.o
$(MODULE_NAME)-objs += linux/gps_dl_linux_plat_drv.o
$(MODULE_NAME)-objs += linux/gps_dl_linux_reserved_mem.o
$(MODULE_NAME)-objs += linux/gps_dl_emi.o
$(MODULE_NAME)-objs += linux/gps_dl_ctrld.o
$(MODULE_NAME)-objs += linux/gps_dl_procfs.o
$(MODULE_NAME)-objs += linux/gps_dl_osal.o

$(MODULE_NAME)-objs += plat/$(GPS_PLATFORM)/gps_dl_hw_dep_bgf.o
$(MODULE_NAME)-objs += plat/$(GPS_PLATFORM)/gps_dl_hw_dep_gps.o
$(MODULE_NAME)-objs += plat/$(GPS_PLATFORM)/gps_dl_hw_dep_debug.o
$(MODULE_NAME)-objs += plat/$(GPS_PLATFORM)/gps_dl_hw_dep_met2_0.o

ifeq ($(GPS_DL_HAS_MOCK),y)
$(MODULE_NAME)-objs += data_link_mock/mock/gps_mock_mvcd.o
$(MODULE_NAME)-objs += data_link_mock/mock/gps_mock_hal.o
ccflags-y += -DGPS_DL_HAS_MOCK=1
endif

endif #ifeq ($(CONFIG_MTK_GPS_SUPPORT), y)
# EOF
