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
$(warning GPS_CHIP_ID != common)
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

ccflags-y += -I$(srctree)/drivers/misc/mediatek/conn_scp/conap_scp
###############################################################################
GPS2SCP_TEST_SUPPORT :=n
MODULE_NAME := gps_scp
obj-m += $(MODULE_NAME).o

$(MODULE_NAME)-objs += gps2scp.o
ifeq ($(GPS2SCP_TEST_SUPPORT),y)
ccflags-y += -DGPS2SCP_TEST_ENABLE
endif
$(warning gps_scp has been build)

endif #ifeq ($(CONFIG_MTK_GPS_SUPPORT), y)
# EOF
