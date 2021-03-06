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

NAME := Lib_apollo_bt_service

$(NAME)_SOURCES := \
					apollo_bt_main_service.c \
					apollo_bt_a2dp_sink.c \
					apollo_bt_a2dp_sink_profiling.c \
					apollo_bt_a2dp_decoder.c \
					apollo_bt_remote_control.c \
					apollo_bt_nv.c \
					apollo_config_gatt_server.c \
					bluetooth_cfg_dual_mode.c \
					bluetooth_sdp_db.c \
					mem_pool.c

GLOBAL_INCLUDES += .

$(NAME)_COMPONENTS := drivers/bluetooth/dual_mode
$(NAME)_COMPONENTS += libraries/audio/codec/codec_framework
$(NAME)_COMPONENTS += audio/apollo/apollocore

GLOBAL_DEFINES  += BUILDCFG
# this needs to be FALSE to allow app to override the BTEWICED
# linkkey management
GLOBAL_DEFINES  += BTM_INTERNAL_LINKKEY_STORAGE_INCLUDED=FALSE
GLOBAL_DEFINES  += BT_AUDIO_USE_MEM_POOL
#GLOBAL_DEFINES  += MEM_POOL_DEBUG

$(NAME)_INCLUDES   += \
					../apollocore \
					../apollo_streamer \
					../../../drivers/bluetooth \
                    ../../../drivers/bluetooth/include \
                    ../../../drivers/bluetooth/BTE \
                    ../../../drivers/bluetooth/BTE/WICED \
                    ../../../drivers/bluetooth/BTE/Components/stack/include \
                    ../../../drivers/bluetooth/BTE/Projects/bte/main \
                    ../../codec/codec_framework/include
