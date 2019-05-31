# User Test
#------------------------------------------

USER_APP_C  =   $(wildcard  ./*.c)

include ../gapoc_bsp_sources.mk


PLPBRIDGE_FLAGS += -ftdi

ifeq ($(MBED), 1)
TEST_C = $(USER_APP_C) $(GAPOC_BSP_C)
include $(GAP_SDK_HOME)/tools/rules/mbed_rules.mk
else ifeq ($(FREE), 1)
APP_SRC = $(USER_APP_C) $(GAPOC_BSP_C)
include $(GAP_SDK_HOME)/tools/rules/freeRTOS_rules.mk
endif
