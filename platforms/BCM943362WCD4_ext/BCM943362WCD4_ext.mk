#
# Copyright 2016, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

NAME := Platform_BCM943362WCD4_ext

WLAN_CHIP            := 43362
WLAN_CHIP_REVISION   := A2
WLAN_CHIP_FAMILY     := 43362
HOST_MCU_FAMILY      := STM32F2xx
HOST_MCU_VARIANT     := STM32F2x5
HOST_MCU_PART_NUMBER := STM32F215RGT6

ifndef BUS
BUS := SDIO
endif

# BCM943362WCD4_ext-specific make targets
EXTRA_TARGET_MAKEFILES +=$(SOURCE_ROOT)platforms/BCM943362WCD4_ext/BCM943362WCD4_ext_targets.mk

VALID_BUSES := SDIO SPI

# Global includes
GLOBAL_INCLUDES  := .

# Global defines
# HSE_VALUE = STM32 crystal frequency = 26MHz (needed to make UART work correctly)
GLOBAL_DEFINES += HSE_VALUE=26000000
GLOBAL_DEFINES += $$(if $$(NO_CRLF_STDIO_REPLACEMENT),,CRLF_STDIO_REPLACEMENT)

# Components
$(NAME)_COMPONENTS += drivers/spi_flash

# Source files
$(NAME)_SOURCES := platform.c


# WICED APPS
# APP0 and FILESYSTEM_IMAGE are reserved main app and resources file system
# FR_APP :=
# DCT_IMAGE :=
# OTA_APP :=
FILESYSTEM_IMAGE := $(OUTPUT_DIR)/filesystem.bin
# WIFI_FIRMWARE :=
# APP0 :=
# APP1 :=
# APP2 :=

# WICED APPS LOOKUP TABLE
APPS_LUT_HEADER_LOC := 0x0000
APPS_START_SECTOR := 1
ifneq ($(APP),bootloader)
ifneq ($(MAIN_COMPONENT_PROCESSING),1)
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | IMPORTANT NOTES                                                                                     | )
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | Wi-Fi MAC Address                                                                                   | )
$(info |    The target Wi-Fi MAC address is defined in <WICED-SDK>/generated_mac_address.txt                 | )
$(info |    Ensure each target device has a unique address.                                                  | )
$(info +-----------------------------------------------------------------------------------------------------+ )
$(info | MCU & Wi-Fi Power Save                                                                              | )
$(info |    It is *critical* that applications using WICED Powersave API functions connect an accurate 32kHz | )
$(info |    reference clock to the sleep clock input pin of the WLAN chip. Please read the WICED Powersave   | )
$(info |    Application Note located in the documentation directory if you plan to use powersave features.   | )
$(info +-----------------------------------------------------------------------------------------------------+ )
endif
endif
