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

/** @headset_wlan.c
 */
#include "wiced.h"
#include "headset.h"
#include "headset_dct.h"
#include "headset_wlan.h"
#include "hashtable.h"

#include "connection_manager.h"
#include "wiced_log.h"

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
 *               Static Function Declarations
 ******************************************************/
static wiced_result_t headset_wlan_init_service       (void* arg);
static wiced_result_t headset_wlan_deinit_service     (void* arg);
static wiced_result_t headset_wlan_connect            (wiced_interface_t wlan_iface);
static wiced_result_t headset_wlan_disconnect         (wiced_interface_t wlan_iface);
static wiced_result_t headset_wlan_add_service        (wiced_interface_t* wlan_iface, headset_gmrender_context_t* gmrender_context);
static wiced_result_t headset_wlan_prevent_service    (wiced_app_service_t* service);
static wiced_result_t headset_wlan_allow_service      (wiced_app_service_t* service);
static wiced_result_t headset_wlan_button_handler     (app_service_action_t action);
static wiced_result_t headset_wlan_connect_service    (void* arg);
static wiced_result_t headset_wlan_disconnect_service (void* arg);
static void           headset_wlan_p2p_connection_cb  (connection_p2p_result_t result);
static void           headset_wlan_link_down_cb       (void);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/*************************************************************
 * Init WLAN service
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_wlan_init_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Gets an argument pointer for future use, but not used for now */
    UNUSED_PARAMETER(arg);

    /* Init WICED WLAN */
    HEADSET_CHECK_RESULT(wiced_wlan_connectivity_init());

    /* TODO: Init other WLAN functions here */

    wiced_log_msg(WICED_LOG_DEBUG0, "Success to init Headset WLAN!\n");
    return result;
_exit:
    wiced_log_msg(WICED_LOG_ERR, "Failed to init Headset WLAN!\n");
    return result;
}

/*************************************************************
 * Deinit WLAN service
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_wlan_deinit_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Gets an argument pointer for future use, but not used for now */
    UNUSED_PARAMETER(arg);

    /* Deinit WICED WLAN */
    HEADSET_CHECK_RESULT(wiced_wlan_connectivity_deinit());

    /* TODO: DeInit other WLAN functions here */

    wiced_log_msg(WICED_LOG_DEBUG0, "Success to Deinit Headset WLAN!\n");
    return result;
_exit:
    wiced_log_msg(WICED_LOG_ERR, "Failed to Deinit Headset WLAN!\n");
    return result;
}

/*************************************************************
 * WiFi connection request
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t headset_wlan_connect(wiced_interface_t wlan_iface)
{
    wiced_result_t result = WICED_SUCCESS;

    wiced_log_msg(WICED_LOG_DEBUG0, "Starting WiFi on %s Interface\n",
                                    wlan_iface == 0 ? "STA":
                                    wlan_iface == 1 ? "AP":
                                    wlan_iface == 2 ? "P2P":
                                    "Unknown");

    /* Make a pending state until the disconnection is done */
    HEADSET_CHECK_RESULT(app_set_current_service(SERVICE_WLAN));
    HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_PENDING));

    /* Turn RED LED on to indicate the headset is trying to connect to wifi */
    HEADSET_LED_WLAN_CONNECTING

    if (wlan_iface == WICED_P2P_INTERFACE)
    {
        /* Start WiFi direct group client */
        HEADSET_CHECK_RESULT(connection_launch(CONNECTION_P2P_GC_LEGACY));
        /* Register a result callback for p2p connection
         * The p2p thread will return a connection result to this callback */
        connection_register_p2p_result_callback(headset_wlan_p2p_connection_cb);
    }
    else
    {
        /* Only for AP interface */
        wiced_ip_setting_t headset_ip_settings =
        {
            INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
            INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
            INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
        };

        /* Only for STA interface */
        if (wlan_iface == WICED_STA_INTERFACE)
        {
            HEADSET_CHECK_RESULT(wiced_network_register_link_callback(NULL, headset_wlan_link_down_cb, WICED_STA_INTERFACE));
        }

        /* Bring up the wlan network, blocked until the DHCP is done */
        HEADSET_CHECK_RESULT(wiced_network_up_default(&wlan_iface, &headset_ip_settings));

        /* Set a service state to Enabled */
        HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_ENABLED));

        /* Notify that the WLAN is connected */
        HEADSET_LED_WLAN_CONNECTED

        wiced_log_msg(WICED_LOG_NOTICE, "Headset WLAN connected! : %02X\n", result);
    }

    return result;
_exit: /* Error handling */
    headset_wlan_disconnect(wlan_iface);
    return result;
}

/*************************************************************
 * WiFi Direct disconnection request
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t headset_wlan_disconnect(wiced_interface_t wlan_iface)
{
    wiced_result_t result = WICED_SUCCESS;

    if (wlan_iface == WICED_P2P_INTERFACE)
    {
        HEADSET_CHECK_RESULT(connection_kill(CONNECTION_P2P_GC_LEGACY));
        connection_register_p2p_result_callback(NULL);
    }
    else
    {
        HEADSET_CHECK_RESULT(wiced_network_down(wlan_iface));
        HEADSET_CHECK_RESULT(wiced_network_deregister_link_callback(NULL, headset_wlan_link_down_cb, WICED_STA_INTERFACE));
    }

    /* Disable WLAN service */
    HEADSET_CHECK_RESULT(app_disable_service(SERVICE_WLAN));
    /* Turn off the LEDs */
    HEADSET_LED_OFF

_exit:
    return result;
}

/*************************************************************
 * Add WLAN service with callbacks
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_wlan_add_service(wiced_interface_t* wlan_iface, headset_gmrender_context_t* gmrender_context)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t cell;

    cell.priority                   = SERVICE_WLAN_PRIORITY;
    cell.type                       = SERVICE_WLAN;
    cell.state                      = SERVICE_DISABLED;

    cell.prevent_service            = headset_wlan_prevent_service;
    cell.allow_service              = headset_wlan_allow_service;
    cell.button_handler             = headset_wlan_button_handler;
    cell.arg1                       = (void*) wlan_iface;
    cell.arg2                       = (void*) gmrender_context;

    result = wiced_add_entry(&cell);

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Failed to add WLAN Service entry [Error:%d]\n", result);
        return result;
    }

    /* Deinit the WLAN by default
     * Will be init by user input (button) when user wants to start WLAN headset */
    result = headset_wlan_deinit_service(NULL);

    return result;
}

/*************************************************************
 * WLAN connect service callback
 *
 */
static wiced_result_t headset_wlan_connect_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t* service = NULL;
    headset_gmrender_context_t* gmrender_context = NULL;
    wiced_interface_t wlan_iface = WICED_STA_INTERFACE;

    service = (wiced_app_service_t *) arg;

    if( service == NULL ||
        service->type != SERVICE_WLAN ||
        service->arg1 == NULL ||
        service->arg2 == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Invalid service\n");
        return WICED_ERROR;
    }

    wlan_iface = *(wiced_interface_t *) service->arg1;
    gmrender_context = (headset_gmrender_context_t *) service->arg2;

    /* Start wlan connection */
    HEADSET_CHECK_RESULT(headset_wlan_connect(wlan_iface));

    /* Start gmediarender */
    if (wlan_iface == WICED_P2P_INTERFACE)
    {
        /* Wait an asynchronous connection event from p2p thread */
        if (headset_worker_wait_for_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT) == WICED_SUCCESS)
        {
            /* Set a service state to Enabled */
            HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_ENABLED));

            /* Notify that the WLAN is connected */
            HEADSET_LED_WLAN_CONNECTED

            /* Disable WLAN Poewrsave by default
             * The HTTP buffer watermark feature will contorl the WLAN PM dynamically to save more power */
            HEADSET_CHECK_RESULT(wiced_wifi_disable_powersave_interface(wlan_iface));

            /* Start Gmediarender */
            HEADSET_CHECK_RESULT(headset_gmrender_start(gmrender_context));
        }
        else
        {
            wiced_log_msg(WICED_LOG_ERR, "Failed to connect P2P session\n");
        }
    }
    else
    {
        /* Disable WLAN Poewrsave by default
         * The HTTP buffer watermark feature will contorl the WLAN PM dynamically to save more power */
        HEADSET_CHECK_RESULT(wiced_wifi_disable_powersave_interface(wlan_iface));

        /* Start Gmediarender */
        HEADSET_CHECK_RESULT(headset_gmrender_start(gmrender_context));
    }

_exit:
    return result;
}

/*************************************************************
 * WLAN disconnect service callback
 *
 */
static wiced_result_t headset_wlan_disconnect_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t* service = NULL;
    headset_gmrender_context_t* gmrender_context = NULL;
    wiced_interface_t wlan_iface = WICED_STA_INTERFACE;

    service = (wiced_app_service_t *) arg;

    if( service == NULL ||
        service->type != SERVICE_WLAN ||
        service->arg1 == NULL ||
        service->arg2 == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Invalid service\n");
        return WICED_ERROR;
    }

    wlan_iface = *(wiced_interface_t *) service->arg1;
    gmrender_context = (headset_gmrender_context_t *) service->arg2;

    /* Make a pending state until the disconnection is done */
    HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_PENDING));

    /* Stop gmediarender */
    HEADSET_CHECK_RESULT(headset_gmrender_stop(gmrender_context));

    /* Stop wlan connection */
    HEADSET_CHECK_RESULT(headset_wlan_disconnect(wlan_iface));

_exit:
    return result;
}

/*************************************************************
 * WLAN prevent service callback
 *
 */
static wiced_result_t headset_wlan_prevent_service(wiced_app_service_t *service)
{
    wiced_result_t result = WICED_SUCCESS;

    if( service == NULL || service->type != SERVICE_WLAN)
    {
        wiced_log_msg(WICED_LOG_ERR, "[WLAN] Invalid argument\n");
        return WICED_ERROR;
    }

    if( service->state == SERVICE_PLAYING_AUDIO )
    {
        /* TODO: Service is prevented while playing audio */
    }

    HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_PREVENTED));

    return result;
_exit:
    return result;
}

/*************************************************************
 * WLAN allow service callback
 *
 */
static wiced_result_t headset_wlan_allow_service(wiced_app_service_t *service)
{
    wiced_result_t result = WICED_SUCCESS;

    if( service == NULL || service->type != SERVICE_WLAN )
    {
        wiced_log_msg(WICED_LOG_ERR, "[WLAN] Invalid argument\n");
        return WICED_ERROR;
    }

    if( service->state == SERVICE_PREEMPTED || service->state == SERVICE_PREVENTED )
    {
        /* TODO: Servie is allowed from prevent */
    }

    HEADSET_CHECK_RESULT(app_set_current_service(SERVICE_WLAN));
    HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_IDLE));

    return result;
_exit:
    return result;
}

/*************************************************************
 * WLAN STA Link down handler
 *
 */
static void headset_wlan_link_down_cb(void)
{
    wiced_app_service_t* service = NULL;

    service = wiced_get_entry(SERVICE_WLAN);

    if (service == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Failed to get service entry\n");
        return;
    }

    headset_worker_push_work(WICED_WLAN_DISCONNECT, headset_wlan_disconnect_service, (void *)service, WICED_FALSE);
    headset_worker_push_work(WICED_WLAN_DEINIT, headset_wlan_deinit_service, (void *)service, WICED_FALSE);
    wiced_log_msg(WICED_LOG_NOTICE, "Headset WLAN disconnected!\n");
}

/*************************************************************
 * WLAN P2P connection events handler
 *
 */
static void headset_wlan_p2p_connection_cb(connection_p2p_result_t result)
{
    wiced_app_service_t* service = NULL;

    service = wiced_get_entry(SERVICE_WLAN);

    if (service == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Failed to get service entry\n");
        return;
    }

    switch(result)
    {
        case CONNECTION_P2P_CONNECTED:
            /* To notify the connection result to caller of headset_wlan_connect */
            headset_worker_signal_for_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT, WICED_SUCCESS);
            wiced_log_msg(WICED_LOG_NOTICE, "Headset WLAN connected! : %02X\n", result);

            break;
        case CONNECTION_P2P_DISCONNECTED:
            if (app_get_service_state(SERVICE_WLAN) == SERVICE_PENDING ||
                app_get_service_state(SERVICE_WLAN) == SERVICE_DISABLED)
            {
                /* To avoid fw link event while doing p2p discovery */
                break;
            }
            /* Make a pending state immediately here.
             * When the P2P GO disconnect session, we P2P GC doesn't have a way to know that disconnection event.
             * So we're catching this event from FW's link event; takes several secs.
             * This FW link event is delivered 2~3 times in a short time so make the state to pending immediately to block duplicate event */
            app_set_service_state(SERVICE_WLAN, SERVICE_PENDING);
            /* To disconnect and deinit wlan connection */
            headset_worker_push_work(WICED_WLAN_DISCONNECT, headset_wlan_disconnect_service, (void *)service, WICED_FALSE);
            headset_worker_push_work(WICED_WLAN_DEINIT, headset_wlan_deinit_service, (void *)service, WICED_FALSE);
            wiced_log_msg(WICED_LOG_NOTICE, "Headset WLAN disconnected! : %02X\n", result);

            break;
        case CONNECTION_P2P_FAILED:
            /* To notify the connection result to caller of headset_wlan_connect */
            headset_worker_signal_for_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT, WICED_FALSE);
            /* To disconnect and deinit wlan connection */
            headset_worker_push_work(WICED_WLAN_DISCONNECT, headset_wlan_disconnect_service, (void *)service, WICED_ERROR);
            headset_worker_push_work(WICED_WLAN_DEINIT, headset_wlan_deinit_service, (void *)service, WICED_FALSE);
            wiced_log_msg(WICED_LOG_ERR, "Headset WLAN failed to connect: %02X\n", result);

            /* TODO: Need a retry concept? */
            break;
        default:
            wiced_log_msg(WICED_LOG_ERR, "Unknown P2P result ! : %02X\n", result);
            break;
    }
}

/*************************************************************
 * BUTTON events handler
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_wlan_button_handler(app_service_action_t action)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t* service = NULL;

    service = wiced_get_entry(SERVICE_WLAN);

    if (service == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Failed to get service entry\n");
        return result;
    }

    /* TODO: map buttons to UPnP functions */
    switch( action )
    {
        case ACTION_PAUSE_PLAY:
            break;
        case ACTION_FORWARD:
            break;
        case ACTION_BACKWARD:
            break;
        case ACTION_VOLUME_UP:
            break;
        case ACTION_VOLUME_DOWN:
            break;
        case ACTION_WLAN_INIT:
            /* Init WLAN (Turn on the WLAN) */
            headset_worker_push_work(WICED_WLAN_INIT, headset_wlan_init_service, (void *)service, WICED_FALSE);
            break;
        case ACTION_WLAN_DEINIT:
            /* Deinit WLAN (Turn off the WLAN) */
            headset_worker_push_work(WICED_WLAN_DEINIT, headset_wlan_deinit_service, (void *)service, WICED_FALSE);
            break;
        case ACTION_WLAN_CONNECT:
            /* To connect WLAN and start gmediarender if the WLAN link is up
             * Need to process asynchronous connection events */
            headset_worker_push_work(WICED_WLAN_CONNECT, headset_wlan_connect_service, (void *)service, WICED_FALSE);
            break;
        case ACTION_WLAN_DISCONNECT:
            /* To disconnect WLAN and stop gmediarender */
            headset_worker_push_work(WICED_WLAN_DISCONNECT, headset_wlan_disconnect_service, (void *)service, WICED_FALSE);
            break;
        default:
            break;
    }

    return result;
}

/*************************************************************
 * Start WLAN function
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t headset_wlan_application_start(wiced_interface_t* wlan_iface, headset_gmrender_context_t* gmrender_context)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Add WLAN service entry */
    HEADSET_CHECK_RESULT(headset_wlan_add_service(wlan_iface, gmrender_context));

_exit:
    return result;
}

/*************************************************************
 * Stop WLAN function
 *
 */
wiced_result_t headset_wlan_application_stop(wiced_interface_t wlan_iface)
{
    wiced_result_t result = WICED_SUCCESS;

    HEADSET_CHECK_RESULT(headset_wlan_deinit_service(NULL));

_exit:
    return result;
}

