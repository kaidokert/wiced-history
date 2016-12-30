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
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#ifndef OFFSETOF
#define OFFSETOF( type, member )  ( (uintptr_t)&((type *)0)->member )
#endif /* OFFSETOF */

#define GET_CURRENT_ADDRESS_FAILED  ((void*)0xffffffff)

#ifndef PLATFORM_SECUREDCT_ENABLED
#define PLATFORM_SECUREDCT_ENABLED 0
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/* used in computing CRC of DCT in-place */
typedef enum
{
    WICED_DCT_CRC_NOT_IN_RANGE = 0,
    WICED_DCT_CRC_IN_HEADER,         /* Bootloader SDK-3.5.2, SDK-3.6.0, SDK-3.6.1, SDK-3.6.2 */
    WICED_DCT_CRC_IN_VERSION,        /* Bootloader SDK-3.6.4+ */

} wiced_dct_header_in_address_range_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/* DCT section */
typedef enum
{
    DCT_APP_SECTION,
    DCT_SECURITY_SECTION,
    DCT_MFG_INFO_SECTION,
    DCT_WIFI_CONFIG_SECTION,
    DCT_INTERNAL_SECTION, /* Do not use in apps */
    DCT_ETHERNET_CONFIG_SECTION,
    DCT_NETWORK_CONFIG_SECTION,
    DCT_BT_CONFIG_SECTION,
    DCT_P2P_CONFIG_SECTION,
    DCT_OTA2_CONFIG_SECTION,
    DCT_VERSION_SECTION, /* Do not use in apps */
} dct_section_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* we need the crc32 value to determine the validity of the DCT... */
uint32_t wiced_dct_get_crc_from_flash( uint32_t dct_start_addr, uint32_t dct_end_addr,
                                              wiced_dct_header_in_address_range_t header_in_range,
                                              uint32_t crc_start );

/* check minimum validity of platform_dct_header_t (no CRC or sequence) */
wiced_result_t wiced_dct_minimal_check_dct_header( platform_dct_header_t *header);
/* check minimum validity of platform_dct_header_t (no CRC or sequence) */
wiced_result_t wiced_dct_minimal_check_dct_version( platform_dct_version_t *version);

/*
 * Validate the DCT and determine the DCT's SDK version
 */
wiced_dct_sdk_ver_t wiced_dct_validate_and_determine_version(uint32_t device_start_address, uint32_t device_end_address, int *initial_write, int *sequence );

wiced_result_t wiced_dct_read_directly          ( void* info_ptr, uint32_t address, uint32_t size );
wiced_result_t wiced_dct_read_with_copy         ( void* info_ptr, dct_section_t section, uint32_t offset, uint32_t size );
wiced_result_t wiced_dct_write                  ( const void* info_ptr, dct_section_t section, uint32_t offset, uint32_t size );
void*          wiced_dct_get_current_address    ( dct_section_t section );
wiced_result_t wiced_dct_erase_non_current_dct( uint32_t non_current_dct );


#if (DCT_BOOTLOADER_SDK_VERSION >= DCT_BOOTLOADER_SDK_3_1_1)
wiced_result_t wiced_dct_restore_factory_reset  ( void );
wiced_result_t wiced_dct_get_app_header_location( uint8_t app_id, image_location_t* app_header_location );
wiced_result_t wiced_dct_set_app_header_location( uint8_t app_id, image_location_t* app_header_location );
#endif /* (DCT_BOOTLOADER_SDK_VERSION >= DCT_BOOTLOADER_SDK_3_1_1) */

/* for updating old DCT header structure to the current version
 *
 * @param[IN]   xxxx_destination     - ptr to destination struct - current SDK version
 * @param[IN]   xxxx_source          - ptr to source struct      - current which SDK version??? TODO
 *
 * return:  WICED_SUCCESS
 *          WICED_ERROR
 */
extern wiced_result_t wiced_dct_update_header_to_current           (platform_dct_header_t* dct_hdr_destination, platform_dct_header_t* dct_hdr_source);
extern wiced_result_t wiced_dct_update_mfg_info_to_current         (platform_dct_mfg_info_t* mfg_info_destination, platform_dct_mfg_info_t* mfg_info_source);
extern wiced_result_t wiced_dct_update_security_to_current         (platform_dct_security_t* security_credentials_destination, platform_dct_security_t*  security_credentials_source);
extern wiced_result_t wiced_dct_update_security_private_key_to_current (char*  private_destination, char*  private_source);
extern wiced_result_t wiced_dct_update_security_certificate_to_current (char*  certificate_destination, char*  certificate_source);
extern wiced_result_t wiced_dct_update_security_cooee_key_to_current   (uint8_t*  cooee_key_destination, uint8_t*  cooee_key_source);
extern wiced_result_t wiced_dct_update_wifi_config_to_current      (platform_dct_wifi_config_t* wifi_config_destination, platform_dct_wifi_config_t* wifi_config_source);
extern wiced_result_t wiced_dct_update_ethernet_config_to_current  (platform_dct_ethernet_config_t* ethernet_config_destination, platform_dct_ethernet_config_t* ethernet_config_source);
extern wiced_result_t wiced_dct_update_network_config_to_current   (platform_dct_network_config_t* network_config_destination, bootloader_dct_network_config_t* network_config_source);
extern wiced_result_t wiced_dct_update_bt_config_to_current        (platform_dct_bt_config_t* bt_config_destination, bootloader_dct_bt_config_t* bt_config_source);
extern wiced_result_t wiced_dct_update_p2p_config_to_current       (platform_dct_p2p_config_t* p2p_config_destination, platform_dct_p2p_config_t* p2p_config_source);
extern wiced_result_t wiced_dct_update_ota2_config_to_current      (platform_dct_ota2_config_t* ota2_config_destination, bootloader_dct_ota2_config_t* ota2_config_source);
extern wiced_result_t wiced_dct_update_version_to_current          (platform_dct_version_t* dct_version_destination, platform_dct_version_t* dct_version_source);

#if defined(OTA2_SUPPORT)
/* used by OTA2, lives in wiced_dct_external_ota2.c  and  wiced_dct_internal_ota2.c */
wiced_result_t wiced_dct_ota2_save_copy         ( uint8_t boot_type );
wiced_result_t wiced_dct_ota2_read_saved_copy   ( void* info_ptr, dct_section_t section, uint32_t offset, uint32_t size );

/* used by OTA2, lives in wiced_dct_external_common.c  and  wiced_dct_internal_common.c */
wiced_result_t wiced_dct_ota2_erase_save_area_and_copy_dct( uint32_t det_loc );
CRC_TYPE wiced_dct_ota2_get_crc(uint32_t dct_start_addr, uint32_t dct_end_addr);

#endif

#ifdef __cplusplus
} /*extern "C" */
#endif
