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

NAME := Bluetooth_Manufacturing_Test_Application

$(NAME)_SOURCES    := bt_mfg_test.c

$(NAME)_COMPONENTS += drivers/bluetooth/mfg_test

GLOBAL_DEFINES     += WICED_DISABLE_STDIO     # Disable default STDIO. Manufacturing Test uses raw UART
GLOBAL_DEFINES     += BT_BUS_RX_FIFO_SIZE=512 # Set Bluetooth UART RX FIFO size to large enough number

GLOBAL_DEFINES     += NO_WIFI_FIRMWARE        # Don't need WiFi FW

VALID_PLATFORMS    := BCM9WCDPLUS114 \
                      BCM943340WCD1 \
                      BCM9WCD1AUDIO \
                      BCM943909WCD* \
                      BCM943903WCD1_1* \
                      BCM9WCD2REFAD.BCM943438WLPTH_2 \
                      BCM94343WWCD1 \
                      BCM94343WWCD2 \
                      BCM943438WCD1 \
                      BCM943907WAE2_1

INVALID_PLATFORMS  += BCM943907WCD2*
