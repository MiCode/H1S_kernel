ifeq ($(GPS_CHIP_ID), common)
LOCAL_PATH_INCLUDE := $(call my-dir)

include $(LOCAL_PATH_INCLUDE)/gps_stp/Android.mk

ifneq ($(wildcard $(LOCAL_PATH_INCLUDE)/data_link/plat/v010),)
GPS_PLATFORM := v010
include $(LOCAL_PATH_INCLUDE)/data_link/Android.mk
endif

ifneq ($(wildcard $(LOCAL_PATH_INCLUDE)/data_link/plat/v030),)
GPS_PLATFORM := v030
include $(LOCAL_PATH_INCLUDE)/data_link/Android.mk
endif

ifneq ($(wildcard $(LOCAL_PATH_INCLUDE)/data_link/plat/v050),)
GPS_PLATFORM := v050
include $(LOCAL_PATH_INCLUDE)/data_link/Android.mk
endif

ifneq ($(wildcard $(LOCAL_PATH_INCLUDE)/gps_scp),)
include $(LOCAL_PATH_INCLUDE)/gps_scp/Android.mk
endif

ifneq ($(wildcard $(LOCAL_PATH_INCLUDE)/gps_pwr),)
include $(LOCAL_PATH_INCLUDE)/gps_pwr/Android.mk
endif

$(warning GPS_CHIP_ID = common)
else

$(warning GPS_CHIP_ID != common)
LOCAL_PATH := $(call my-dir)

ifeq ($(MTK_GPS_SUPPORT), yes)

include $(CLEAR_VARS)
LOCAL_MODULE := gps_drv.ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := mtk
LOCAL_INIT_RC := init.gps_drv.rc

ifneq (,$(filter mt6877 mt6879 mt6885 mt6893 mt6983,$(TARGET_BOARD_PLATFORM)))
#Only set dependency to conninfra.ko when CONSYS_CHIP in list.
ifneq (,$(filter CONSYS_6877 CONSYS_6879 CONSYS_6885 CONSYS_6893 CONSYS_6983,$(MTK_COMBO_CHIP)))
LOCAL_REQUIRED_MODULES := conninfra.ko
else
$(warning TARGET_BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM), MTK_COMBO_CHIP=$(MTK_COMBO_CHIP))
$(warning gps_drv.ko does not claim the requirement for conninfra.ko)
endif
else
LOCAL_REQUIRED_MODULES := wmt_drv.ko
endif

include $(MTK_KERNEL_MODULE)

endif

endif
