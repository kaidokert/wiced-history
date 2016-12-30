/*
 * Copyright 2016, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/*
 * Defines internal configuration of the BCM943907WCD2 board
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  OTA Support
 *  NOTE:
 *  1. Only apply when OTA2 is not enabled.
 *  2. Following line must be located ahead of platform_config_bsp_default.h to override PLATFORM_NO_SFLASH_WRITE.
 */
#ifndef OTA2_SUPPORT
#define PLATFORM_HAS_OTA
#endif

/*
 * Below configuration file defines default BSP settings.
 * Put here platform specific configuration parameters to override these default ones.
 */
#include "platform_config_bsp_default.h"

/*
 * Below configuration file defines default WICED settings.
 * To change settings replace below included file with its contents.
 */
#include "platform_config_wiced_default.h"

#ifdef __cplusplus
} /* extern "C" */
#endif
