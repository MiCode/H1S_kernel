LOCAL_PATH := $(call my-dir)

ifeq ($(MTK_GPS_SUPPORT), yes)

include $(CLEAR_VARS)
LOCAL_MODULE := gps_drv_dl_$(GPS_PLATFORM).ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := mtk
LOCAL_INIT_RC := init.gps_drv.rc
LOCAL_REQUIRED_MODULES := conninfra.ko
include $(MTK_KERNEL_MODULE)
$(warning GPS_PLATFORM=$(GPS_PLATFORM))
GPS_OPTS := GPS_PLATFORM=$(GPS_PLATFORM)
$(linked_module): OPTS += $(GPS_OPTS)
endif
