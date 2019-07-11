# User Test
#------------------------------------------

USER_APP_C  =   $(wildcard  ./*.c)

ifeq ($(MBED), 1)
MBED = 1
FREE = 0
else
MBED = 0
FREE = 1
endif
include ../gapoc_bsp_sources.mk


PLPBRIDGE_FLAGS += -ftdi

ifeq ($(MBED), 1)
TEST_C = $(USER_APP_C) $(GAPOC_BSP_C)
include $(GAP_SDK_HOME)/tools/rules/mbed_rules.mk
else ifeq ($(FREE), 1)
ifeq ($(PMSIS), 1)
FREERTOS_FLAGS += -DUSE_PMSIS=1
USER_APP_C +=  ../../GAPOC_BSP/GAPOC_BSP_NINA/nina_b112.c
endif
FREERTOS_FLAGS += -DDBG=1
APP_SRC = $(USER_APP_C) $(GAPOC_BSP_C)
include $(GAP_SDK_HOME)/tools/rules/freeRTOS_rules.mk
endif
