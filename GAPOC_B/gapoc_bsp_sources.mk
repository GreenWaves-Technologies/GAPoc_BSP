# Paths to GAPOC BSP include files
INC +=  ..
INC +=  ../../GAPOC_BSP/GAPOC_BSP_GENERAL
INC +=  ../../GAPOC_BSP/GAPOC_BSP_NINA
INC +=  ../../GAPOC_BSP/GAPOC_BSP_ADAFRUIT_LCD

# Paths to GAPOC BSP source files
GAPOC_PLATFORM_C = $(wildcard ../*.c)
GENERAL_C = $(wildcard ../../GAPOC_BSP/GAPOC_BSP_GENERAL/*.c)
NINA_C =    $(wildcard ../../GAPOC_BSP/GAPOC_BSP_NINA/*.c)
ADAFRUIT_LCD_C =    $(wildcard ../../GAPOC_BSP/GAPOC_BSP_ADAFRUIT_LCD/*.c)
GAPOC_BSP_C = $(GAPOC_PLATFORM_C) $(GENERAL_C) $(MT9V034_C) $(NINA_C) $(ADAFRUIT_LCD_C)
