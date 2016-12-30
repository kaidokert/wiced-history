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

/* main.c - Main program routines
 *
 * Copyright (C) 2005-2007   Ivo Clarysse
 *
 * This file is part of GMediaRender.
 *
 * GMediaRender is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * GMediaRender is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GMediaRender; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

#define _GNU_SOURCE

#ifdef HAVE_CONFIG_H
    #include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wiced_result.h"
#include "wiced_wifi.h"
#include "gmediarender.h"
#include "audio_client.h"

#include <threadutil/inc/ithread.h>
// For version strings of upnp
#include <upnp/inc/upnpconfig.h>

#include "logging.h"
#include "output.h"
#include "upnp.h"
#include "upnp_control.h"
#include "upnp_device.h"
#include "upnp_renderer.h"
#include "upnp_transport.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define GMEDIARENDER_SERVICE_ID ( 0xBEF0A345 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct gmediarender_service_s
{
    uint32_t                       id;
    struct upnp_device_descriptor* upnp_renderer;
    struct upnp_device*            device;
    char                           ip_address_string[ 16 ];
} gmediarender_service_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

#if defined( ENABLE_TRANSPORT_LOGGING ) || defined( ENABLE_CONTROL_LOGGING )
static void log_variable_change( void* userdata, int var_num,
                                 const char* variable_name,
                                 const char* old_value,
                                 const char* variable_value )
{
    const char* category = (const char*) userdata;
    int         needs_newline = variable_value[ strlen( variable_value ) - 1 ] != '\n';
    // Silly terminal codes. Set to empty strings if not needed.
    const char* var_start = Log_color_allowed( ) ? "\033[1m\033[34m" : "";
    const char* var_end = Log_color_allowed( ) ? "\033[0m" : "";

    Log_info( category, "%s%s%s: %s%s",
              var_start, variable_name, var_end,
              variable_value, needs_newline ? "\n" : "" );
}
#endif /* ENABLE_TRANSPORT_LOGGING || ENABLE_CONTROL_LOGGING */


static void init_logging( const char* log_file )
{
    char version[ 1024 ];

    snprintf( version, sizeof( version ), "[ gmediarender %s "
                                          "(libupnp-%s) ]",
              GM_COMPILE_VERSION, UPNP_VERSION_STRING );

    Log_init( log_file );
    Log_info( "main", "%s log started %s", PACKAGE_STRING, version );
}


void ipv4_to_string( const wiced_ip_address_t* addr, char* addr_str, uint32_t addr_str_size )
{
    uint8_t ipv4[ 4 ];

    memcpy( ipv4, &addr->ip.v4, sizeof( addr->ip.v4 ) );
    snprintf( addr_str, addr_str_size, "%u.%u.%u.%u", ipv4[ 3 ], ipv4[ 2 ], ipv4[ 1 ], ipv4[ 0 ] );
    Log_info( "main", "Using IPv4 address %s", addr_str );
}


wiced_result_t gmediarender_service_start( gmediarender_service_params_t* params, gmediarender_service_ref* service_handle )
{
    wiced_result_t          result  = WICED_ERROR;
    gmediarender_service_t* phandle = NULL;
    int                     rc;
    wiced_ip_address_t      ipv4;

    if ( ( params->listening_port != 0 ) && ( ( params->listening_port < 49152 ) || ( params->listening_port > 65535 ) ) )
    {
        // Somewhere obscure internally in libupnp, they clamp the
        // port to be outside of the IANA range, so at least 49152.
        // Instead of surprising the user by ignoring lower port
        // numbers, complain loudly.
        Log_error( "main", "ERROR: listening port needs to be in range [49152..65535] (but was set to %hu)", params->listening_port );
        result = WICED_BADARG;
        goto _exit;
    }

    phandle = calloc( 1, sizeof( gmediarender_service_t ) );
    if ( phandle == NULL )
    {
        Log_error( "main", "ERROR: calloc() failed !" );
        result = WICED_OUT_OF_HEAP_SPACE;
        goto _exit;
    }
    phandle->id = GMEDIARENDER_SERVICE_ID;

    init_logging( NULL );

    phandle->upnp_renderer = upnp_renderer_descriptor( params->friendly_name, params->uuid );
    if ( phandle->upnp_renderer == NULL )
    {
        Log_error( "main", "ERROR: Failed to get renderer descriptor" );
        result = WICED_ERROR;
        goto _exit;
    }

    result = wiced_ip_get_ipv4_address( params->interface, &ipv4 );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, Log_error( "main", "ERROR: wiced_ip_get_ipv4_address() failed !" ) );
    ipv4_to_string( &ipv4, phandle->ip_address_string, sizeof( phandle->ip_address_string ) );

    output_add_options( &params->audio_client_params );

    rc = output_init( NULL );
    if ( rc != 0 )
    {
        Log_error( "main", "ERROR: Failed to initialize Output subsystem" );
        result = WICED_ERROR;
        goto _exit;
    }

    phandle->device = upnp_device_init( phandle->upnp_renderer, params->interface, phandle->ip_address_string, params->listening_port );
    if ( phandle->device == NULL )
    {
        Log_error( "main", "ERROR: Failed to initialize UPnP device" );
        result = WICED_ERROR;
        goto _exit;
    }

    upnp_transport_init( phandle->device );
    upnp_control_init( phandle->device );

    if (Log_info_enabled())
    {
#ifdef ENABLE_TRANSPORT_LOGGING
        upnp_transport_register_variable_listener(log_variable_change, (void*) "transport");
#endif
#ifdef ENABLE_CONTROL_LOGGING
        upnp_control_register_variable_listener(log_variable_change, (void*) "control");
#endif
    }

    Log_info("main", "Ready for rendering.");

    *service_handle = phandle;

    result = WICED_SUCCESS;

_exit:
    if ( result != WICED_SUCCESS )
    {
        free( phandle );
    }
    return result;
}


wiced_result_t gmediarender_service_stop( gmediarender_service_ref service_handle )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( service_handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( service_handle->id != GMEDIARENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        result = WICED_BADARG;
        goto _exit;
    }

    upnp_control_deinit();
    upnp_transport_deinit();
    output_deinit();
    upnp_device_shutdown( service_handle->device );
    service_handle->id = 0;

_exit:
    return result;
}
