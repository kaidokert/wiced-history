/*
 * Copyright 2015, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */
#pragma once

/** @file
 *  Defines the SDIO part of the WICED Platform Interface.
 *
 *  Provides constants and prototypes for functions that
 *  enable Wiced to use an SDIO bus on a particular hardware platform.
 */

#include <stdint.h>
#include "wwd_constants.h"
#include "wwd_buffer.h"
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 * @cond       Constants
 ******************************************************/

/*
 * SDIO specific constants
 */
typedef enum
{
    SDIO_CMD_0  =  0,
    SDIO_CMD_3  =  3,
    SDIO_CMD_5  =  5,
    SDIO_CMD_7  =  7,
    SDIO_CMD_52 = 52,
    SDIO_CMD_53 = 53,
    __MAX_VAL   = 64
} sdio_command_t;

typedef enum
{
    SDIO_BLOCK_MODE = ( 0 << 2 ), /* These are STM32 implementation specific */
    SDIO_BYTE_MODE  = ( 1 << 2 )  /* These are STM32 implementation specific */
} sdio_transfer_mode_t;

typedef enum
{
    SDIO_1B_BLOCK    =  1,
    SDIO_2B_BLOCK    =  2,
    SDIO_4B_BLOCK    =  4,
    SDIO_8B_BLOCK    =  8,
    SDIO_16B_BLOCK   =  16,
    SDIO_32B_BLOCK   =  32,
    SDIO_64B_BLOCK   =  64,
    SDIO_128B_BLOCK  =  128,
    SDIO_256B_BLOCK  =  256,
    SDIO_512B_BLOCK  =  512,
    SDIO_1024B_BLOCK = 1024,
    SDIO_2048B_BLOCK = 2048
} sdio_block_size_t;


typedef enum
{
    RESPONSE_NEEDED,
    NO_RESPONSE
} sdio_response_needed_t;



/******************************************************
 *             Global declarations
 ******************************************************/


/******************************************************
 *             Structures
 ******************************************************/

/** @endcond */

/** @addtogroup platif Platform Interface
 *  @{
 */

/** @name SDIO Bus Functions
 *  Functions that enable WICED to use an SDIO bus
 *  on a particular hardware platform.
 */
/**@{*/


/******************************************************
 *             Function declarations
 ******************************************************/


/**
 * Transfers SDIO data
 *
 * Implemented in the WICED Platform interface, which is specific to the
 * platform in use.
 * WICED uses this function as a generic way to transfer data
 * across an SDIO bus.
 * Please refer to the SDIO specification.
 *
 * @param direction         : Direction of transfer - Write = to Wi-Fi device,
 *                                                    Read  = from Wi-Fi device
 * @param command           : The SDIO command number
 * @param mode              : Indicates whether transfer will be byte mode or block mode
 * @param block_size        : The block size to use (if using block mode transfer)
 * @param argument          : The argument of the particular SDIO command
 * @param data              : A pointer to the data buffer used to transmit or receive
 * @param data_size         : The length of the data buffer
 * @param response_expected : Indicates if a response is expected - RESPONSE_NEEDED = Yes
 *                                                                  NO_RESPONSE     = No
 * @param response  : A pointer to a variable which will receive the SDIO response.
 *                    Can be null if the caller does not care about the response value.
 *
 * @return WWD_SUCCESS if successful, otherwise an error code
 */
extern wwd_result_t host_platform_sdio_transfer( wwd_bus_transfer_direction_t direction, sdio_command_t command, sdio_transfer_mode_t mode, sdio_block_size_t block_size, uint32_t argument, /*@null@*/ uint32_t* data, uint16_t data_size, sdio_response_needed_t response_expected, /*@out@*/ /*@null@*/ uint32_t* response );


/**
 * Performs SDIO enumeration
 *
 * This needs to be called if the WLAN chip is reset
 *
 */
extern wwd_result_t host_platform_sdio_enumerate( void );

/**
 * Switch SDIO bus to high speed mode
 *
 * When SDIO starts, it must be in a low speed mode
 * This function switches it to high speed mode (up to 50MHz)
 *
 */
extern void host_platform_enable_high_speed_sdio( void );

/**
 * SDIO interrupt handler
 *
 * This function is implemented by Wiced and must be called
 * from the interrupt vector
 *
 */
extern void sdio_irq( void );


/**
 * Unmasks the bus interrupt
 *
 * This function is called by WICED to unmask the bus interrupt
 * on host platforms that must mask off the bus interrupt to
 * allow processing of the existing interrupt.
 */
extern wwd_result_t host_platform_unmask_sdio_interrupt( void );


#ifndef  WICED_DISABLE_MCU_POWERSAVE

/**
 * SDIO Out-of-band interrupt handler
 *
 * This function should enable the detection of an external interrupt
 * from the GPIO0 pin of the WLAN chip.
 *
 */
extern wwd_result_t host_enable_oob_interrupt( void );

/**
 * Get OOB interrupt pin (WLAN GPIO0 or GPIO1)
 */
extern uint8_t host_platform_get_oob_interrupt_pin( void );

#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */
/** @} */
/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
