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

NAME := Lib_audio_render

AUDIO_RENDER_LIBRARY_NAME :=audio_render.$(RTOS).$(NETWORK).$(HOST_ARCH).$(BUILD_TYPE).a

ifneq ($(wildcard $(CURDIR)$(AUDIO_RENDER_LIBRARY_NAME)),)
$(info Using PREBUILT:  $(AUDIO_RENDER_LIBRARY_NAME))
$(NAME)_PREBUILT_LIBRARY :=$(AUDIO_RENDER_LIBRARY_NAME)
else
# Build from source (Broadcom internal)
$(info Building SRC:  $(AUDIO_RENDER_LIBRARY_NAME))
include $(CURDIR)audio_render_src.mk
endif # ifneq ($(wildcard $(CURDIR)$(AUDIO_RENDER_LIBRARY_NAME)),)

GLOBAL_INCLUDES += .

$(NAME)_CFLAGS :=

# Do not enable the timing log unless PLATFORM_DDR_BASE is defined.
#GLOBAL_DEFINES     += AUDIO_RENDER_TIMING_LOG

ifneq (,$(findstring AUDIO_RENDER_TIMING_LOG,$(GLOBAL_DEFINES)))
$(info Enabling audio render timing log.)
ifdef WICED_ENABLE_TRACEX
$(info WARNING: TraceX and timing log both enabled!)
endif

GLOBAL_DEFINES     += AUDIO_RENDER_BUFFER_DDR_OFFSET=0x0
GLOBAL_DEFINES     += AUDIO_RENDER_BUFFER_SIZE=0x200000
endif

ifdef WICED_ENABLE_TRACEX
GLOBAL_DEFINES     += AUDIO_RENDER_ENABLE_TRACE_EVENTS
endif

