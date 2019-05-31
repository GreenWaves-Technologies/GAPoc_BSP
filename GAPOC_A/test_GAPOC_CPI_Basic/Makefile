# User Test
#------------------------------------------

USER_APP_C  =   $(wildcard  ./*.c)

include ../gapoc_bsp_sources.mk

# This is to store files on PC via the bridge
# The figure after -fileIO is optional (2 by default); it's a latency because program must be loaded first  
PLPBRIDGE_FLAGS += -fileIO 4

#This is to use OLIMEX JTAG probe:
PLPBRIDGE_FLAGS += -ftdi

ifeq ($(MBED), 1)
TEST_C = $(USER_APP_C) $(GAPOC_BSP_C)
include $(GAP_SDK_HOME)/tools/rules/mbed_rules.mk
else ifeq ($(FREE), 1)
APP_SRC = $(USER_APP_C) $(GAPOC_BSP_C)
include $(GAP_SDK_HOME)/tools/rules/freeRTOS_rules.mk
endif
