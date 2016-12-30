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

/** @file U8G Library HW Function
 *
 */

#include "u8g_arm.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
/* #define U8G_I2C_USE_43909_TIMING */
#define RETRIES 5

#ifdef U8G_I2C_USE_43909_TIMING
#define USEC_DELAY_COUNT_1 49
#define USEC_DELAY_COUNT_10 780
#define MSEC_DELAY_COUNT_1 65538
#define MSEC_DELAY_COUNT_FACTOR 53000
#endif

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
 *               Variables Definitions
 ******************************************************/

static wiced_i2c_device_t i2c_display;

/******************************************************
 *               Function Definitions
 ******************************************************/

void u8g_init_wiced_i2c_device(wiced_i2c_device_t* device)
{
    i2c_display = *device;
}

/* These delay functions were measured with a scope on BCM943909WCD1_3.
 * They may be used for 43909 specific debugging or reference. */
#ifdef U8G_I2C_USE_43909_TIMING
void u8g_Delay(uint16_t number_of_milliseconds)
{
    int counter = 0;

    for (counter = 0; counter < MSEC_DELAY_COUNT_1; ++counter)
    {
        asm("nop");
    }
    if (number_of_milliseconds-- > 1)
    {
        for (counter = 0; counter < MSEC_DELAY_COUNT_FACTOR * number_of_milliseconds; ++counter)
        {
            asm("nop");
        }
    }
}

void u8g_MicroDelay(void)
{
    int counter = 0;

    for (counter = 0; counter < USEC_DELAY_COUNT_1; ++counter)
    {
        asm("nop");
    }
}

void u8g_10MicroDelay(void)
{
    int counter = 0;

    for (counter = 0; counter < USEC_DELAY_COUNT_10; ++counter)
    {
        asm("nop");
    }
}
#else
void u8g_Delay(uint16_t milliseconds)
{
    wiced_rtos_delay_milliseconds(milliseconds);
}

void u8g_MicroDelay(void)
{
    wiced_rtos_delay_microseconds(1);
}

void u8g_10MicroDelay(void)
{
    wiced_rtos_delay_microseconds(10);
}
#endif


uint8_t u8g_com_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
    static uint8_t control = 0;
#ifndef U8G_I2C_USE_REPEAT_START
    static uint8_t started = 0;
#endif
    wiced_result_t result;

    switch(msg)
    {
        /* Initialize I2C controller and probe device */
        case U8G_COM_MSG_INIT:
            if (wiced_i2c_init(&i2c_display) != WICED_SUCCESS)
            {
                WPRINT_LIB_INFO( ("Failed to initialize I2C core\n") );
            }
            if (wiced_i2c_probe_device(&i2c_display, RETRIES) != WICED_TRUE)
            {
                WPRINT_LIB_INFO( ("I2C device not found\n") );
            }
            u8g_MicroDelay();
            break;

        /* Switch between data and command mode */
        case U8G_COM_MSG_ADDRESS:
            /* Command Mode - 0x80 indicates a single command byte will be sent
             * Command Mode - 0x00 indicates a series of command bytes will be sent */
            if (arg_val == 0)
            {
#ifdef U8G_I2C_USE_REPEAT_START
                control = 0x00;
                result = wiced_i2c_write(&i2c_display, WICED_I2C_START_FLAG, &control, 1);
                if (result != WICED_SUCCESS)
                {
                    WPRINT_LIB_INFO( ("Control-Byte write failed\n") );
                }
#else
                control = 0x80;
                started = 0;
#endif
            }
            /* Data Mode - 0x40 indicates a series of data bytes will be sent */
            else
            {
                control = 0x40;
            }
            u8g_MicroDelay();
            break;

        /* Write single command byte to device */
        case U8G_COM_MSG_WRITE_BYTE:
#ifndef U8G_I2C_USE_REPEAT_START
            if (!started)
            {
                wiced_i2c_write(&i2c_display, WICED_I2C_START_FLAG, &control, sizeof control);
                started = 1;
            }
            else
            {
                wiced_i2c_write(&i2c_display, 0, &control, sizeof control);
            }
#endif

            result = wiced_i2c_write(&i2c_display, 0, &arg_val, 1);
            if (result != WICED_SUCCESS)
            {
                WPRINT_LIB_INFO( ("Single-Byte write failed\n") );
            }

            u8g_MicroDelay();

            break;

        /* Write a sequence of data bytes to device */
        case U8G_COM_MSG_WRITE_SEQ:
        case U8G_COM_MSG_WRITE_SEQ_P:
#ifndef U8G_I2C_USE_REPEAT_START
            wiced_i2c_write(&i2c_display, 0, &control, sizeof control);
#else
            result = wiced_i2c_write(&i2c_display, WICED_I2C_REPEATED_START_FLAG, &control, 1);
            if (result != WICED_SUCCESS)
            {
                WPRINT_LIB_INFO( ("Repeat-Start write failed\n") );
            }
#endif

            result = wiced_i2c_write(&i2c_display, WICED_I2C_STOP_FLAG, arg_ptr, arg_val);
            if (result != WICED_SUCCESS)
            {
                WPRINT_LIB_INFO( ("Multiple-Byte write failed\n") );
            }

            u8g_MicroDelay();
            break;
    }

    return 1;
}
