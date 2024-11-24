###############################################################################
# Bluetooth character device driver

###############################################################################

ifndef TOP
    TOP := $(srctree)/..
endif

#ifneq ($(CFG_BT_PM_QOS_CONTROL),)
#    ccflags-y += -D PM_QOS_CONTROL=1
#else
#    ccflags-y += -D PM_QOS_CONTROL=0
#endif

# Force build fail on modpost warning
KBUILD_MODPOST_FAIL_ON_WARNINGS := y

ccflags-y += -D FW_LOG_DEFAULT_ON=0


# platform
ccflags-y += -D CONNAC1X_CHIPID=$(BT_PLATFORM)
$(info $(LOG_TAG) BT_PLATFORM = $(BT_PLATFORM))
$(info $(LOG_TAG) srctree = $(srctree))


###############################################################################
# To add WMT dependent Macro and header file, will be removed later

ccflags-y += -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt$(BT_PLATFORM)/include

ifeq ($(CONFIG_MTK_CONN_LTE_IDC_SUPPORT),y)
    ccflags-y += -D WMT_IDC_SUPPORT=1
else
    ccflags-y += -D WMT_IDC_SUPPORT=0
endif

###############################################################################
# To include BT driver dependent header file

WMT_SRC_FOLDER := $(srctree)/drivers/misc/mediatek/connectivity/common
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/include
ccflags-y += -I$(WMT_SRC_FOLDER)/common_main/linux/include
ifneq ($(CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH),)
ccflags-y += -I$(WMT_SRC_FOLDER)/debug_utility
endif
ccflags-y += -I$(src)/../include/btif

###############################################################################

MODULE_NAME := bt_drv_$(BT_PLATFORM)
ifeq ($(CONFIG_WLAN_DRV_BUILD_IN),y)
  obj-y += $(MODULE_NAME).o
else
  obj-m += $(MODULE_NAME).o
endif

ccflags-y += -D CREATE_NODE_DYNAMIC=1

$(MODULE_NAME)-objs += stp_chrdev_bt.o
$(MODULE_NAME)-objs += btmtk_dbg.o btmtk_dbg_tp_evt_if.o
ifneq ($(CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH),)
$(MODULE_NAME)-objs += fw_log_bt.o
endif
