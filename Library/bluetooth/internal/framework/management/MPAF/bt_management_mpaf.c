/**
 * Copyright 2014, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 *
 */

#include "wiced.h"
#include "wiced_bt.h"
#include "bt_bus.h"
#include "bt_mpaf.h"
#include "bt_packet_internal.h"
#include "bt_management_mpaf.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define BT_DEVICE_NAME_MAX_LENGTH 21

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

static void mpaf_system_event_handler( mpaf_event_opcode_t event, bt_packet_t* packet, mpaf_event_params_t* params );
static void mpaf_sdp_event_handler   ( mpaf_event_opcode_t event, bt_packet_t* packet, mpaf_event_params_t* params );

/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_queue_t mpaf_event_mailbox;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t bt_management_mpaf_init( void )
{
    /* Initialise event mailbox */
    if ( wiced_rtos_init_queue( &mpaf_event_mailbox, "MPAF event mailbox", sizeof(bt_packet_t*), 1 ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error initialising MPAF event mailbox\r\n") );
        return WICED_ERROR;
    }

    /* Initialise MPAF client */
    if ( bt_mpaf_init( mpaf_system_event_handler ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error initialising MPAF transport protocol\r\n") );
        return WICED_ERROR;
    }

    /* Register SDP callback */
    if ( bt_mpaf_register_sdp_callback( mpaf_sdp_event_handler ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error registering MPAF SDP callback\r\n") );
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_result_t bt_management_mpaf_deinit( void )
{
    /* Deinit MPAF */
    if ( bt_mpaf_deinit() != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error deinitialising MPAF transport protocol\r\n") );
        return WICED_ERROR;
    }

    /* Register SDP callback */
    if ( bt_mpaf_register_sdp_callback( NULL ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error deregistering MPAF SDP callback\r\n") );
        return WICED_ERROR;
    }

    /* Initialise event mailbox */
    if ( wiced_rtos_deinit_queue( &mpaf_event_mailbox ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error deinitialising MPAF event mailbox\r\n") );
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_bool_t bt_management_mpaf_is_device_connectable( void )
{
    bt_packet_t*         packet;
    mpaf_event_params_t* params;

    if ( bt_mpaf_get_controller_status( ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error getting Bluetooth controller status\r\n") );
        return WICED_ERROR;
    }

    /* Wait on event queue for MPAF_COMMAND_COMPLETE event */
    if ( bt_management_mpaf_wait_for_event( &packet ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error waiting for MPAF_COMMAND_COMPLETE event\r\n") );
        return WICED_ERROR;
    }

    params = (mpaf_event_params_t*)packet->data_start;

    if ( params->command_complete.return_params.get_controller_status.status == MPAF_SUCCESS )
    {
        bt_packet_pool_free_packet( packet );

        if ( ( params->command_complete.return_params.get_controller_status.controller_status & MPAF_CONTROLLER_STATUS_CONNECTABLE ) != 0 )
        {
            return WICED_TRUE;
        }
        else
        {
            return WICED_FALSE;
        }
    }
    else
    {
        bt_packet_pool_free_packet( packet );
        return WICED_FALSE;
    }
}

wiced_bool_t bt_management_mpaf_is_device_discoverable( void )
{
    bt_packet_t*         packet;
    mpaf_event_params_t* params;

    if ( bt_mpaf_get_controller_status( ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error getting Bluetooth controller status\r\n") );
        return WICED_ERROR;
    }

    /* Wait on event queue for MPAF_COMMAND_COMPLETE event */
    if ( bt_management_mpaf_wait_for_event( &packet ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("Error waiting for MPAF_COMMAND_COMPLETE event\r\n") );
        return WICED_ERROR;
    }

    params = (mpaf_event_params_t*)packet->data_start;

    if ( params->command_complete.return_params.get_controller_status.status == MPAF_SUCCESS )
    {
        bt_packet_pool_free_packet( packet );

        if ( ( params->command_complete.return_params.get_controller_status.controller_status & MPAF_CONTROLLER_STATUS_DISCOVERABLE ) != 0 )
        {
            return WICED_TRUE;
        }
        else
        {
            return WICED_FALSE;
        }
    }
    else
    {
        bt_packet_pool_free_packet( packet );
        return WICED_FALSE;
    }
}

wiced_result_t bt_management_mpaf_wait_for_event( bt_packet_t** packet )
{
    return wiced_rtos_pop_from_queue( &mpaf_event_mailbox, (void*)packet, WICED_NEVER_TIMEOUT );
}

wiced_result_t bt_management_mpaf_notify_event_received( bt_packet_t* packet )
{
    return wiced_rtos_push_to_queue( &mpaf_event_mailbox, (void*)&packet, WICED_NEVER_TIMEOUT );
}

static void mpaf_system_event_handler( mpaf_event_opcode_t event, bt_packet_t* packet, mpaf_event_params_t* params )
{
    if ( event == MPAF_COMMAND_COMPLETE && params->command_complete.command_opcode == MPAF_GET_CONTROLLER_STATUS )
    {
        bt_management_mpaf_notify_event_received( packet );
    }
    else
    {
        /* Packet is not needed anymore. Delete! */
        bt_packet_pool_free_packet( packet );
    }
}

static void mpaf_sdp_event_handler( mpaf_event_opcode_t event, bt_packet_t* packet, mpaf_event_params_t* params )
{
    if ( event == MPAF_COMMAND_COMPLETE )
    {
        bt_management_mpaf_notify_event_received( packet );
    }
    else
    {
        /* Packet is not needed anymore. Delete! */
        bt_packet_pool_free_packet( packet );
    }
}
