LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := connfem.ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := mtk

LOCAL_INIT_RC := init.connfem.rc

include $(MTK_KERNEL_MODULE)
