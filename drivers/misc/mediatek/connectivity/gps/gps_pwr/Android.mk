
LOCAL_PATH := $(call my-dir)

ifeq ($(MTK_GPS_SUPPORT), yes)

include $(CLEAR_VARS)
LOCAL_MODULE := gps_pwr.ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := mtk
LOCAL_INIT_RC := init.gps_pwr.rc

include $(MTK_KERNEL_MODULE)

endif

