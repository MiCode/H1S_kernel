# SPDX-License-Identifier: GPL-2.0
ifeq ($(strip $(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE)), y)
dtb-y += $(addsuffix .dtb, $(notdir $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))))
endif

always          := $(dtb-y)
subdir-y        := $(dts-dirs)
clean-files     := *.dtb
