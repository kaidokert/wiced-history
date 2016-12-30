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

#include "wiced.h"
#include "headset_dct.h"
#include "headset_gmrender.h"

#include "wiced_log.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define DEFAULT_LISTENING_PORT                  ( 49494 )
#define DEFAULT_UUID                            "37ddf93a-6644-4fe3-953c-5feccfc72990"
#define DEFAULT_FRIENDLY_NAME                   "GMediaRender"

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
static int headset_gmrender_audio_client_event_cb(audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg);

/*************************************************************
 * Event callback for audio client
 *
 * Main purpose of this function is to save WLAN power with watermark feature
 */
static int headset_gmrender_audio_client_event_cb( audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg )
{
    headset_gmrender_context_t* gmrender_context = NULL;
    wiced_interface_t wlan_interface = WICED_STA_INTERFACE;
    powersave_dct_t pwrsave_params = {0};

    UNUSED_PARAMETER(handle);
    UNUSED_PARAMETER(arg);

    if (userdata == NULL)
    {
        return WICED_ERROR;
    }

    /* Get an userdata; WLAN interface and WLAN powersave mode */
    gmrender_context = (headset_gmrender_context_t *)userdata;
    wlan_interface = gmrender_context->audio_userdata.wlan_interface;
    memcpy(&pwrsave_params, &gmrender_context->audio_userdata.powersave_params, sizeof(powersave_dct_t));

    switch(event)
    {
        /* Hits high watermark
         * means there are many http buffers to be processed so we can enter to WLAN powersave mode */
        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH:
            if (pwrsave_params.wlan_pm_mode == PM1_POWERSAVE_MODE)
            {
                wiced_wifi_enable_powersave_interface(wlan_interface);
            }
            else if (pwrsave_params.wlan_pm_mode == PM2_POWERSAVE_MODE)
            {
                wiced_wifi_enable_powersave_with_throughput_interface(pwrsave_params.wlan_pm2_ret_on_high, wlan_interface);
            }
            /* else: No Powersave mode, nothing to do */
            break;

        /* Hits low watermark
         * means we don't have enough http buffers so should get out from WLAN powrsave mode */
        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW:
            if (pwrsave_params.wlan_pm_mode == PM1_POWERSAVE_MODE)
            {
                wiced_wifi_disable_powersave_interface(wlan_interface);
            }
            else if (pwrsave_params.wlan_pm_mode == PM2_POWERSAVE_MODE)
            {
                wiced_wifi_enable_powersave_with_throughput_interface(pwrsave_params.wlan_pm2_ret_on_low, wlan_interface);
            }
            /* else: No Powersave mode, nothing to do */
            break;

        /* TODO: Need a process audio event here? See other event types in AUDIO_CLIENT_EVENT_T */

        default:
            break;
    }

    return 0;
}

/*************************************************************
 * Init gmediarender parameters
 *
 */
wiced_result_t headset_gmrender_init( headset_gmrender_context_t** _gmrender_context, audio_dct_t* audio_params, powersave_dct_t* pwrsave_params, wiced_interface_t interface )
{
    wiced_result_t result = WICED_SUCCESS;
    headset_gmrender_context_t* gmrender_context = NULL;

    /* Init main context and userdata for audio client */
    gmrender_context = calloc_named("headset_gmrender", 1, sizeof(headset_gmrender_context_t));

    if (gmrender_context == NULL ||
        audio_params == NULL ||
        pwrsave_params == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to init gmrender context\n");
        result = WICED_ERROR;
        return result;
    }

    /* Init gmrender audio user data */
    gmrender_context->audio_userdata.wlan_interface = interface;
    memcpy(&gmrender_context->audio_userdata.powersave_params, pwrsave_params, sizeof(powersave_dct_t));

    /* Gmediarender parameters */
    gmrender_context->service_params.interface      = interface;
    gmrender_context->service_params.listening_port = DEFAULT_LISTENING_PORT;
    gmrender_context->service_params.uuid           = DEFAULT_UUID;
    gmrender_context->service_params.friendly_name  = DEFAULT_FRIENDLY_NAME;

    gmrender_context->service_params.audio_client_params.interface           = interface;
    gmrender_context->service_params.audio_client_params.event_cb            = headset_gmrender_audio_client_event_cb;
    gmrender_context->service_params.audio_client_params.device_id           = PLATFORM_DEFAULT_AUDIO_OUTPUT;

    /* Tunable parameters */
    gmrender_context->service_params.audio_client_params.userdata            = (void *)gmrender_context;
    gmrender_context->service_params.audio_client_params.data_buffer_num     = audio_params->http_buffer_num;
    gmrender_context->service_params.audio_client_params.audio_buffer_num    = audio_params->audio_buffer_num;
    gmrender_context->service_params.audio_client_params.audio_buffer_size   = audio_params->audio_buffer_size;
    gmrender_context->service_params.audio_client_params.data_threshold_high = audio_params->http_threshold_high;
    gmrender_context->service_params.audio_client_params.data_threshold_low  = audio_params->http_threshold_low;
    gmrender_context->service_params.audio_client_params.volume              = audio_params->volume;

    *_gmrender_context = gmrender_context;

    return result;
}

/*************************************************************
 * Deinit gmediarender handle
 *
 */
wiced_result_t headset_gmrender_deinit( headset_gmrender_context_t* gmrender_context )
{
    wiced_result_t result = WICED_SUCCESS;

    if (gmrender_context == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "No gmrender handle! Failed to deinit gmediarender\n");
        result = WICED_ERROR;
        return result;
    }

    headset_gmrender_stop(gmrender_context);

    if (gmrender_context != NULL)
    {
        free(gmrender_context);
        gmrender_context = NULL;
    }

    return result;
}

/*************************************************************
 * To start the gmediarender
 *
 */
wiced_result_t headset_gmrender_start( headset_gmrender_context_t* gmrender_context )
{
    wiced_result_t result = WICED_ERROR;

    if (gmrender_context == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "No gmrender handle! Failed to stop gmediarender\n");
        return WICED_ERROR;
    }

    result = gmediarender_service_start( &gmrender_context->service_params, &gmrender_context->service_handle );

    if (result == WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_NOTICE, "Gmediarender is started\n");
    }
    else
    {
        wiced_log_msg(WICED_LOG_ERR, "Failed to start Gmediarender\n");
    }

    return result;
}

/*************************************************************
 * To stop the gmediarender
 *
 */
wiced_result_t headset_gmrender_stop( headset_gmrender_context_t* gmrender_context )
{
    wiced_result_t result = WICED_ERROR;

    if (gmrender_context == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "No gmrender handle! Failed to stop gmediarender\n");
        return WICED_ERROR;
    }

    result = gmediarender_service_stop( gmrender_context->service_handle );

    if (result == WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_NOTICE, "Gmediarender is stopped\n");
    }
    else
    {
        wiced_log_msg(WICED_LOG_ERR, "Failed to stop Gmediarender\n");
    }

    return result;
}
