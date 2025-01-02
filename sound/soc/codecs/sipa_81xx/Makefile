# SIA PA object

COMPILE_OPTION = $(CONFIG_SND_SOC_SIPA_81XX)
ENABLE_SIA_TUNING = $(CONFIG_SND_SOC_SIPA_TUNING)
PLATFORM_TYPE = mtk
EXTRA_CFLAGS += $(call cc-option,-Wno-date-time,)
MODULE_NAME = snd-soc-sipa
SIPA_TUNING_MODULE = snd-soc-sipa-tuning

$(MODULE_NAME)-objs := \
		sipa.o \
		sipa_regmap.o \
		sipa_aux_dev_if.o \
		sipa_91xx.o \
		sipa_parameter.o

$(SIPA_TUNING_MODULE)-objs := sipa_tuning_misc.o

ifeq ($(COMPILE_OPTION),m)
EXTRA_CFLAGS += -DSIPA_COMPILE_TO_MODULE
endif

ifeq ($(PLATFORM_TYPE),qcom_elite)
$(SIPA_TUNING_MODULE)-objs += sipa_afe.o
endif

ifeq ($(PLATFORM_TYPE),mtk)
$(SIPA_TUNING_MODULE)-objs += sipa_tuning_if.o # sipa_tuning_if_dev.o # sia81xx_ipi.o
endif

ifeq ($(PLATFORM_TYPE),qcom_spf)
$(SIPA_TUNING_MODULE)-objs += sipa_tuning_if.o # sipa_tuning_if_dev.o # sia81xx_ipi.o
endif

obj-$(COMPILE_OPTION) += $(MODULE_NAME).o

obj-$(ENABLE_SIA_TUNING) += $(SIPA_TUNING_MODULE).o

