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

NAME := App_Audio_Player

#INCLUDE_MP3_DECODER	:= 1	# NOT working yet
#INCLUDE_ALAW_DECODER	:= 1	# NOT working yet

#WICED_ENABLE_TRACEX := 1
GLOBAL_DEFINES     += WICED_DISABLE_WATCHDOG
GLOBAL_DEFINES     += WICED_DISABLE_TLS
GLOBAL_DEFINES     += WICED_DISABLE_MCU_POWERSAVE
GLOBAL_DEFINES     += WPRINT_ENABLE_APP_DEBUG
GLOBAL_DEFINES     += WPRINT_ENABLE_LIB_DEBUG

# ENABLE for ethernet support
#$(NAME)_DEFINES   += MFG_TEST_ENABLE_ETHERNET_SUPPORT

APPLICATION_DCT    := audio_player_dct.c

GLOBAL_DEFINES     += APPLICATION_STACK_SIZE=12000

$(NAME)_SOURCES    := audio_player.c	\
					  audio_player_config.c		\
					  audio_player_dct.c		\
					  audio_player_http.c		\
					  audio_player_util.c		\
					  audio_player_flac.c		\
					  audio_player_wav.c


$(NAME)_COMPONENTS := audio/apollo/audio_render \
					  audio/codec/FLAC \
                      utilities/command_console \
                      utilities/command_console/wifi \
                      utilities/command_console/dct \
                      utilities/wiced_log \
                      filesystems/wicedfs \
					  protocols/DNS \
					  protocols/HTTP

# MP3 specific files & libs
ifeq (1,$(INCLUDE_MP3_DECODER))
$(NAME)_SOURCES    += wiced_mp3_interface.c
$(NAME)_COMPONENTS += audio/codec/MP3
endif

# aLaw uLaw specific files & libs
ifeq (1,$(INCLUDE_ALAW_DECODER))
$(NAME)_SOURCES    += wiced_aLawuLaw_interface.c
$(NAME)_COMPONENTS += audio/codec/aLaw_uLaw
endif


#GLOBAL_DEFINES     += CONSOLE_ENABLE_WL
ifneq (,$(findstring CONSOLE_ENABLE_WL,$(GLOBAL_DEFINES)))
# wl commands which dump a lot of data require big buffers.
GLOBAL_DEFINES   += WICED_PAYLOAD_MTU=8320
$(NAME)_COMPONENTS += test/wl_tool
endif

WIFI_CONFIG_DCT_H  := wifi_config_dct.h

ifdef WICED_ENABLE_TRACEX
$(info apollo_audio using tracex lib)
GLOBAL_DEFINES     += WICED_TRACEX_BUFFER_DDR_OFFSET=0x0
GLOBAL_DEFINES     += WICED_TRACEX_BUFFER_SIZE=0x200000
$(NAME)_COMPONENTS += test/TraceX
endif

GLOBAL_DEFINES     += RX_PACKET_POOL_SIZE=10


GLOBAL_DEFINES     += WICED_USE_AUDIO


VALID_OSNS_COMBOS  := ThreadX-NetX_Duo
VALID_PLATFORMS    := BCM943909WCD* BCM943907*
INVALID_PLATFORMS  := BCM943907AEVAL*
