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
 * Powercycle Application
 *
 * This application snippet demonstrates how to minimise power
 * consumption for a connected device. The app initialises the
 * WICED device, connects to and pings the network, then
 * powers off the Wi-Fi device for a fixed time period. This
 * process repeats forever. Please read the WICED Powersave
 * Application Note to familiarise yourself with the WICED
 * powersave implementation before using this application.
 *
 * Features demonstrated
 *  - Wi-Fi client mode
 *  - ICMP ping
 *  - MCU powersave
 *  - Wi-Fi powersave
 *  - Wi-Fi poweroff
 *
 * Application Instructions
 *   1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *      in the wifi_config_dct.h header file to match your Wi-Fi access point
 *   2. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *
 * The application loops through the following sequence forever:
 *      - displays WICED startup information on the terminal
 *      - joins the AP specified in wifi_config_dct.h
 *      - turns on MCU & Wi-Fi powersave
 *      - sends an ICMP ping to the gateway, and
 *      - enters low power mode for WIFI_SLEEP_TIME seconds
 *
 * NOTES :
 *  1. The Wi-Fi device can only be powered down if the WICED hardware
 *     platform has the ability to cut power to the Wi-Fi chip.
 *  2. Wi-Fi powerdown does *NOT* currently work for FreeRTOS/LwIP
 *     since LwIP does not provide a clean way to terminate the
 *     network thread
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define WIFI_SLEEP_TIME     (5 * SECONDS)

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

static wiced_result_t send_ping( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    while (1)
    {
        /* Initialise the WICED device */
        wiced_init();

        /* Configure the device */
        /* wiced_configure_device( app_config ); */  /* Config bypassed in local makefile and wifi_config_dct.h */

        /* Bring up the network interface and connect to the Wi-Fi network */
        wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

        /* Enable MCU powersave */
        /* WARNING: Please read the WICED Powersave API documentation before enabling MCU powersave */
        /* wiced_platform_mcu_enable_powersave(); */

        /* Enable Wi-Fi powersave */
        wiced_wifi_enable_powersave();

        /* Send an ICMP ping to the gateway */
        send_ping( );

        /* De-initialise the Wi-Fi interface which powers down the Wi-Fi device */
        WPRINT_APP_INFO(( "Wi-Fi Powerdown\n\n" ));
        wiced_deinit( );

        /* Sleep for a while */
        wiced_rtos_delay_milliseconds( WIFI_SLEEP_TIME );
    }
}


static wiced_result_t send_ping( void )
{
    const uint32_t     ping_timeout = 1000;
    uint32_t           elapsed_ms;
    wiced_result_t     status;
    wiced_ip_address_t ping_target_ip;

    wiced_ip_get_gateway_address( WICED_STA_INTERFACE, &ping_target_ip );
    status = wiced_ping( WICED_STA_INTERFACE, &ping_target_ip, ping_timeout, &elapsed_ms );

    if ( status == WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Ping Reply %lums\n", elapsed_ms ));
    }
    else if ( status == WICED_TIMEOUT )
    {
        WPRINT_APP_INFO(( "Ping timeout\n" ));
    }
    else
    {
        WPRINT_APP_INFO(( "Ping error\n" ));
    }

    return WICED_SUCCESS;
}
