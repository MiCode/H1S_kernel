# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2021 Mediatek Inc.

CFG_FM_CHIP := mt6631

$(info [fm_drv:Makefile] M = $(M))
$(info [fm_drv:Makefile] FM_CHIP_ID = $(FM_CHIP_ID))
$(info [fm_drv:Makefile] FM_CHIP = $(FM_CHIP))
$(info [fm_drv:Makefile] FM_PLAT = $(FM_PLAT))
$(info [fm_drv:Makefile] CFG_FM_PLAT = $(CFG_FM_PLAT))
$(info [fm_drv:Makefile] CFG_BUILD_CONNAC2 = $(CFG_BUILD_CONNAC2))
$(info [fm_drv:Makefile] CFG_FM_CHIP_ID = $(CFG_FM_CHIP_ID))
$(info [fm_drv:Makefile] CFG_FM_CHIP = $(CFG_FM_CHIP))

define build_kernel_modules
    $(info [fm_drv:Makefile:build] CFG_FM_PLAT = $(1))
    $(info [fm_drv:Makefile:build] CFG_BUILD_CONNAC2 = $(2))
    $(info [fm_drv:Makefile:build] CFG_FM_CHIP_ID = $(3))
    $(info [fm_drv:Makefile:build] CFG_FM_CHIP = $(4))
    $(MAKE) -C $(KERNEL_SRC) M=$(M)/Build/$(1) modules $(KBUILD_OPTIONS) CFG_FM_PLAT=$(1) CFG_BUILD_CONNAC2=$(2) CFG_FM_CHIP_ID=$(3) CFG_FM_CHIP=$(4) KBUILD_EXTRA_SYMBOLS="$(5)"
endef

define install_kernel_modules
   $(MAKE) M=$(M)/Build/$(1) -C $(KERNEL_SRC) modules_install
endef

define clean_kernel_modules
   $(MAKE) -C $(KERNEL_SRC) M=$(M)/Build/$(1) clean
endef

extra_symbols := $(abspath $(O)/../vendor/mediatek/kernel_modules/connectivity/common/Module.symvers)
extra_symbols += $(abspath $(O)/../vendor/mediatek/kernel_modules/connectivity/conninfra/Module.symvers)

ifneq ($(wildcard $(abspath $(O)/../vendor/mediatek/kernel_modules/connectivity/common/Module.symvers)) ,)
    $(info [fm_drv:Makefile] wmt ready)
endif
ifneq ($(wildcard $(abspath $(O)/../vendor/mediatek/kernel_modules/connectivity/conninfra/Module.symvers)) ,)
    $(info [fm_drv:Makefile] conninfra ready)
endif

all:
ifdef CFG_FM_CHIP_ID
	$(info [fm_drv:Makefile] make LD = 1.0)
	$(call build_kernel_modules,$(CFG_FM_PLAT),$(CFG_BUILD_CONNAC2),$(CFG_FM_CHIP_ID),$(CFG_FM_CHIP),$(extra_symbols))
	#$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)
else
	$(info [fm_drv:Makefile] make LD = 2.0)
ifneq ($(wildcard $(abspath $(O)/../vendor/mediatek/kernel_modules/connectivity/conninfra/Module.symvers)) ,)
	$(call build_kernel_modules,connac2x,true,,mt6635,$(extra_symbols))
endif
	$(call build_kernel_modules,mt6631_6635,false,,,$(extra_symbols))
	$(call build_kernel_modules,mt6631,false,,mt6631,$(extra_symbols))
	$(call build_kernel_modules,mt6635,false,,mt6635,$(extra_symbols))
	# legacy chip
	$(call build_kernel_modules,soc,false,,mt6580,$(extra_symbols))
	$(call build_kernel_modules,mt6625,false,,mt6625,$(extra_symbols))
	$(call build_kernel_modules,mt6627,false,,mt6627,$(extra_symbols))
	$(call build_kernel_modules,mt6630,false,,mt6630,$(extra_symbols))
endif

modules_install:
ifdef CFG_FM_CHIP_ID
	$(info [fm_drv:Makefile] install LD = 1.0)
	$(call install_kernel_modules,$(CFG_FM_PLAT),$(CFG_BUILD_CONNAC2),$(CFG_FM_CHIP_ID),$(CFG_FM_CHIP),$(extra_symbols))
	#$(MAKE) M=$(M) -C $(KERNEL_SRC) modules_install
else
	$(info [fm_drv:Makefile] install LD = 2.0)
ifneq ($(wildcard $(abspath $(O)/../vendor/mediatek/kernel_modules/connectivity/conninfra/Module.symvers)) ,)
	$(call install_kernel_modules,connac2x,true,,mt6635,$(extra_symbols))
endif
	$(call install_kernel_modules,mt6631_6635,false,,,$(extra_symbols))
	$(call install_kernel_modules,mt6631,false,,mt6631,$(extra_symbols))
	$(call install_kernel_modules,mt6635,false,,mt6635,$(extra_symbols))
	# legacy chip
	$(call install_kernel_modules,soc,false,,mt6580,$(extra_symbols))
	$(call install_kernel_modules,mt6625,false,,mt6625,$(extra_symbols))
	$(call install_kernel_modules,mt6627,false,,mt6627,$(extra_symbols))
	$(call install_kernel_modules,mt6630,false,,mt6630,$(extra_symbols))
endif

clean:
ifdef CFG_FM_CHIP_ID
	$(info [fm_drv:Makefile] clean LD = 1.0)
	$(call clean_kernel_modules,$(CFG_FM_PLAT),$(CFG_BUILD_CONNAC2),$(CFG_FM_CHIP_ID),$(CFG_FM_CHIP),$(extra_symbols))
	#$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
else
	$(info [fm_drv:Makefile] clean LD = 2.0)
ifneq ($(wildcard $(abspath $(O)/../vendor/mediatek/kernel_modules/connectivity/conninfra/Module.symvers)) ,)
	$(call clean_kernel_modules,connac2x,true,,mt6635,$(extra_symbols))
endif
	$(call clean_kernel_modules,mt6631_6635,false,,,$(extra_symbols))
	$(call clean_kernel_modules,mt6631,false,,mt6631,$(extra_symbols))
	$(call clean_kernel_modules,mt6635,false,,mt6635,$(extra_symbols))
	# legacy chip
	$(call clean_kernel_modules,soc,false,,mt6580,$(extra_symbols))
	$(call clean_kernel_modules,mt6625,false,,mt6625,$(extra_symbols))
	$(call clean_kernel_modules,mt6627,false,,mt6627,$(extra_symbols))
	$(call clean_kernel_modules,mt6630,false,,mt6630,$(extra_symbols))
endif
