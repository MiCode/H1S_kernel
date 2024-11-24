LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := bt_drv_$(BT_PLATFORM).ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := mtk
LOCAL_INIT_RC := init.bt_drv.rc
LOCAL_REQUIRED_MODULES := wmt_drv.ko
include $(MTK_KERNEL_MODULE)

BT_OPTS := BT_PLATFORM=$(BT_PLATFORM) LOG_TAG=$(LOG_TAG)
#ifeq ($(BT_PLATFORM), $(filter $(BT_PLATFORM), 6873 6853 6833))
#  BT_OPTS += CFG_BT_PM_QOS_CONTROL=y
#endif
$(info $(LOG_TAG) BT_OPTS = $(BT_OPTS))
$(linked_module): OPTS += $(BT_OPTS)


