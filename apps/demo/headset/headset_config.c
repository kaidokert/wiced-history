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

/** @file
 *
 */

#include "wiced.h"
#include "wiced_log.h"

#include "headset_config.h"
#include "headset_dct.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t headset_config_load_dct_wifi(headset_dct_collection_t* dct_tables);
static wiced_result_t headset_config_load_dct_network(headset_dct_collection_t* dct_tables);
static wiced_result_t headset_config_unload_dct_wifi(headset_dct_collection_t* dct_tables);
static wiced_result_t headset_config_unload_dct_network(headset_dct_collection_t* dct_tables);

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t headset_config_load_dct_wifi(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    result = wiced_dct_read_lock((void**)&dct_tables->dct_wifi, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t));
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Can't get WIFi configuration!\n");
    }
    return result;
}

static wiced_result_t headset_config_load_dct_network(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    result = wiced_dct_read_lock( (void**)&dct_tables->dct_network, WICED_TRUE, DCT_NETWORK_CONFIG_SECTION, 0, sizeof(platform_dct_network_config_t) );
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Can't get Network configuration!\n");
    }
    return result;
}

wiced_result_t headset_config_init(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    /* wifi */
    result = headset_config_load_dct_wifi(dct_tables);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    /* network */
    result = headset_config_load_dct_network(dct_tables);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    /* App */
    result = wiced_dct_read_lock( (void**)&dct_tables->dct_app, WICED_TRUE, DCT_APP_SECTION, 0, sizeof(headset_dct_t));
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Can't get App configuration!\n");
        return result;
    }

    return result;
}

static wiced_result_t headset_config_unload_dct_wifi(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result = WICED_SUCCESS;

    if (dct_tables != NULL && dct_tables->dct_wifi != NULL)
    {
        result = wiced_dct_read_unlock(dct_tables->dct_wifi, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            dct_tables->dct_wifi = NULL;
        }
        else
        {
            wiced_log_msg(WICED_LOG_ERR, "Can't Free/Release WiFi Configuration !\n");
        }
    }

    return result;
}

static wiced_result_t headset_config_unload_dct_network(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result = WICED_SUCCESS;

    if (dct_tables != NULL && dct_tables->dct_network != NULL)
    {
        result = wiced_dct_read_unlock(dct_tables->dct_network, WICED_TRUE);
        if (result == WICED_SUCCESS)
        {
            dct_tables->dct_network = NULL;
        }
        else
        {
            wiced_log_msg(WICED_LOG_ERR, "Can't Free/Release Network Configuration !\n");
        }
    }

    return result;
}

wiced_result_t headset_config_deinit(headset_dct_collection_t* dct_tables)
{
    wiced_result_t result;

    result = wiced_dct_read_unlock(dct_tables->dct_app, WICED_TRUE);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Can't Free/Release App Configuration !\n");
        return result;
    }

    result = headset_config_unload_dct_network(dct_tables);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    result = headset_config_unload_dct_wifi(dct_tables);
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    return result;
}

