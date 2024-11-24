LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
$(info [fm_drv:Include.mk] BUILD_CONNAC2 = $(BUILD_CONNAC2))
$(info [fm_drv:Include.mk] FM_CHIP_ID = $(FM_CHIP_ID))
$(info [fm_drv:Include.mk] FM_CHIP = $(FM_CHIP))
$(info [fm_drv:Include.mk] FM_PLAT = $(FM_PLAT))

MODULE_NAME := fmradio_drv_$(FM_PLAT)
LOCAL_INIT_RC := init.fmradio_drv.rc
LOCAL_MODULE := $(MODULE_NAME).ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := mtk
ifeq ($(BUILD_CONNAC2), true)
    LOCAL_REQUIRED_MODULES := conninfra.ko
else
    LOCAL_REQUIRED_MODULES := wmt_drv.ko
endif

FM_OPTS := CONFIG_FM_USER_LOAD=1

# $(FM_CHIP_ID) should be empty in LD 2.0
FM_OPTS += MODULE_NAME=$(MODULE_NAME) CFG_BUILD_CONNAC2=$(BUILD_CONNAC2) CFG_FM_CHIP_ID=$(FM_CHIP_ID) CFG_FM_CHIP=$(FM_CHIP) CFG_FM_PLAT=$(FM_PLAT)

include $(MTK_KERNEL_MODULE)

$(linked_module): OPTS += $(FM_OPTS)

