LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := bt_drv_$(BT_PLATFORM).ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := mtk
LOCAL_INIT_RC := init.bt_drv.rc
LOCAL_SRC_FILES := $(patsubst $(LOCAL_PATH)/%,%,$(shell find $(LOCAL_PATH) -type f -name '*.[cho]')) Makefile
LOCAL_REQUIRED_MODULES := conninfra.ko
LOCAL_REQUIRED_MODULES += connfem.ko
include $(MTK_KERNEL_MODULE)
BT_OPTS := BT_PLATFORM=$(BT_PLATFORM) LOG_TAG=$(LOG_TAG)
$(info $(LOG_TAG) BT_OPTS = $(BT_OPTS))
$(linked_module): OPTS += $(BT_OPTS)


