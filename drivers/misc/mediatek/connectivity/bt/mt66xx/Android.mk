LOCAL_PATH := $(call my-dir)
LOCAL_PATH_B := $(LOCAL_PATH)

BT_PLATFORM:=$(subst MTK_CONSYS_MT,,$(MTK_BT_CHIP))  
$(info [BT_Drv] MTK_BT_SUPPORT = $(MTK_BT_SUPPORT))
$(info [BT_Drv] MTK_BT_CHIP = $(MTK_BT_CHIP))

ifeq ($(strip $(MTK_BT_SUPPORT)), yes)
ifneq (true,$(strip $(TARGET_NO_KERNEL)))
  # connac1x
  LOG_TAG := [BT_Drv][wmt]
  BT_PLATFORM := connac1x
  include $(LOCAL_PATH_B)/wmt/Android.mk

  # connac20
  LOG_TAG := [BT_Drv][btif]
  BT_PLATFORM := 6885
  include $(LOCAL_PATH_B)/btif/Android.mk
  BT_PLATFORM := 6893
  include $(LOCAL_PATH_B)/btif/Android.mk
  BT_PLATFORM := 6877
  include $(LOCAL_PATH_B)/btif/Android.mk
  BT_PLATFORM := 6983
  include $(LOCAL_PATH_B)/btif/Android.mk
  BT_PLATFORM := 6879
  include $(LOCAL_PATH_B)/btif/Android.mk
  BT_PLATFORM := 6895
  include $(LOCAL_PATH_B)/btif/Android.mk
endif
endif

#dirs := btif
#include $(call all-named-subdir-makefiles, $(dirs))
