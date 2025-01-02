#
# Makefile for the fingerprint device.
#
xiaomi_fp-objs := fp_driver.o fp_netlink.o fp_platform.o
obj-$(CONFIG_XIAOMI_FINGERPRINT) := xiaomi_fp.o

#add some mtk makefile
# Each configuration option enables a list of files.
#ifneq ($(MTK_PLATFORM),)
#include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
subdir-ccflags-y += -I$(srctree)/drivers/tee/teei/510/tz_driver/include/
subdir-ccflags-y += -I$(srctree)/drivers/tee/teei/510/common/include/
#endif

ifeq ($(filter "duchamp", $(CONFIG_BUILD_PRODUCT_NAME)),)
$(warning "start compling fp_driver")
CDEFINES += -DCONFIG_FP_MTK_PLATFORM
KBUILD_CPPFLAGS += $(CDEFINES)
endif
