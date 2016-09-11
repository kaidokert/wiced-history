/*
 * Copyright 2013, Broadcom Corporation
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

#include "sam4s_platform.h"
#include "platform_sleep.h"
#include "wiced_defaults.h"
#include "wiced_utilities.h"
#include "crt0.h"
#include "core_cm4.h"
#include "RTOS/wwd_rtos_interface.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define CYCLES_TO_MS( cycles ) ( ( ( cycles ) * 1000 * RTT_CLOCK_PRESCALER ) / RTT_CLOCK_HZ )
#define MS_TO_CYCLES( ms )     ( ( ( ms ) * RTT_CLOCK_HZ ) / ( 1000 * RTT_CLOCK_PRESCALER ) )

/******************************************************
 *                    Constants
 ******************************************************/

#ifdef WICED_DISABLE_MCU_POWERSAVE
#undef WAIT_MODE_SUPPORT
#endif /* ifndef WICED_DISABLE_MCU_POWERSAVE */

#define RTT_CLOCK_PRESCALER          (4) /*                                                                                        */
#define RTT_CLOCK_HZ             (32768) /* 32K / prescaler                                                                        */
#define RTT_MAX_CYCLES           (64000) /* 7.8125 seconds                                                                         */
#define RC_OSC_DELAY_CYCLES         (15) /* Cycles required to stabilise clock after exiting WAIT mode                             */
#define JTAG_DEBUG_SLEEP_DELAY_MS (3000) /* Delay in ms to give OpenOCD enough time to halt the CPU before the CPU enter WAIT mode */

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

#ifdef WAIT_MODE_SUPPORT
static unsigned long  wait_mode_power_down_hook        ( unsigned long delay_ms );
extern void           init_clocks                      ( void );
extern void           RTT_irq                          ( void );
#else
static unsigned long  idle_power_down_hook             ( unsigned long delay_ms );
#endif
extern void           vApplicationIdleSleepHook        ( void );
extern wiced_result_t host_platform_bus_enter_powersave( void );
extern wiced_result_t host_platform_bus_exit_powersave ( void );


/******************************************************
 *               Variables Definitions
 ******************************************************/

#ifdef WAIT_MODE_SUPPORT
static volatile uint32_t     system_io_backup_value      = 0;
static volatile uint32_t     sam4s_clock_needed_counter  = 0;
static volatile wiced_bool_t wake_up_interrupt_triggered = WICED_FALSE;
#endif

/******************************************************
 *            WICED Function Definitions
 ******************************************************/


/******************************************************
 *            SAM4S Function Definitions
 ******************************************************/

wiced_result_t sam4s_powersave_init( void )
{
    /* Initialise all pins to be input pull-up to save power */
    ioport_enable_port( IOPORT_PIOA,   0xffffffffU );
    ioport_set_port_mode( IOPORT_PIOA, 0xffffffffU, IOPORT_MODE_PULLUP );
    ioport_set_port_dir( IOPORT_PIOA,  0xffffffffU, IOPORT_DIR_INPUT );

    ioport_enable_port( IOPORT_PIOB,   0xffffffffU );
    ioport_set_port_mode( IOPORT_PIOB, 0xffffffffU, IOPORT_MODE_PULLUP );
    ioport_set_port_dir( IOPORT_PIOB,  0xffffffffU, IOPORT_DIR_INPUT );

#ifdef ID_PIOC
    /* On setup on SAM4S variants which PIOC is avaiable */
    ioport_enable_port( IOPORT_PIOC,   0xffffffffU );
    ioport_set_port_mode( IOPORT_PIOC, 0xffffffffU, IOPORT_MODE_PULLUP );
    ioport_set_port_dir( IOPORT_PIOC,  0xffffffffU, IOPORT_DIR_INPUT );
#endif

#ifdef WAIT_MODE_SUPPORT
    NVIC_DisableIRQ( RTT_IRQn );
    NVIC_ClearPendingIRQ( RTT_IRQn );
    NVIC_SetPriority( RTT_IRQn, SAM4S_RTT_IRQ_PRIO );
    NVIC_EnableIRQ( RTT_IRQn );
    pmc_set_fast_startup_input( PMC_FSMR_RTTAL );
#endif

    return WICED_SUCCESS;
}

wiced_result_t sam4s_powersave_clocks_needed( void )
{
#ifdef WAIT_MODE_SUPPORT
    /* Atomic operation starts */
    DISABLE_INTERRUPTS;

    /* Increment counter to indicate CPU clock is needed preventing CPU from entering WAIT mode */
    sam4s_clock_needed_counter++;

    /* Atomic operation ends */
    ENABLE_INTERRUPTS;
#endif
    return WICED_SUCCESS;
}

wiced_result_t sam4s_powersave_clocks_not_needed( void )
{
#ifdef WAIT_MODE_SUPPORT
    /* Atomic operation starts */
    DISABLE_INTERRUPTS;

    /* Decrement counter only if it's not 0 */
    sam4s_clock_needed_counter -= ( sam4s_clock_needed_counter > 0 ) ? 1 : 0;

    /* Atomic operation ends */
    ENABLE_INTERRUPTS;
#endif
    return WICED_SUCCESS;
}

wiced_result_t sam4s_powersave_enable_wakeup_pin( const sam4s_wakeup_pin_config_t* config )
{
    if ( config->is_wakeup_pin == WICED_TRUE )
    {
        /* Enable wake-up pin */
        pmc_set_fast_startup_input( 1 << config->wakeup_pin_number );

        if ( config->trigger == IOPORT_SENSE_RISING )
        {
            /* Set polarity of corresponding wake-up pin bit to active-high */
            PMC->PMC_FSPR |=  (uint32_t)( 1 << config->wakeup_pin_number );
        }
        else
        {
            /* Set polarity of corresponding wake-up pin bit to active-low */
            PMC->PMC_FSPR &= ~(uint32_t)( 1 << config->wakeup_pin_number );
        }

        return WICED_SUCCESS;
    }
    else
    {
        return WICED_ERROR;
    }
}

wiced_result_t sam4s_powersave_disable_wakeup_pin( const sam4s_wakeup_pin_config_t* config )
{
    if ( config->is_wakeup_pin == WICED_TRUE )
    {
        pmc_clr_fast_startup_input( 1 << config->wakeup_pin_number );
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_ERROR;
    }
}

void sam4s_powersave_wake_up_interrupt_notify( void )
{
#ifdef WAIT_MODE_SUPPORT
    wake_up_interrupt_triggered = WICED_TRUE;
#endif
}

/******************************************************
 *      RTOS Powersave Hook Function Definitions
 ******************************************************/

unsigned long platform_power_down_hook( unsigned long delay_ms )
{
#ifdef WAIT_MODE_SUPPORT
    return wait_mode_power_down_hook( delay_ms );
#else /* WAIT_MODE_SUPPORT */
    return idle_power_down_hook( delay_ms );
#endif /* WAIT_MODE_SUPPORT */
}

void platform_idle_hook( void )
{
    __asm("wfi");
}

void vApplicationIdleSleepHook( void )
{
    __asm("wfi");
}

/******************************************************
 *        Interrupt Service Routine Definitions
 ******************************************************/

#ifdef WAIT_MODE_SUPPORT
void RTT_irq( void )
{
    __set_PRIMASK( 1 );
    rtt_get_status( RTT );
    rtt_disable_interrupt( RTT, RTT_MR_ALMIEN );
}
#endif

/******************************************************
 *            Static Function Definitions
 ******************************************************/

#ifdef WAIT_MODE_SUPPORT
static unsigned long wait_mode_power_down_hook( unsigned long delay_ms )
{
    wiced_bool_t jtag_enabled       = ( ( CoreDebug ->DHCSR & CoreDebug_DEMCR_TRCENA_Msk ) != 0 ) ? WICED_TRUE : WICED_FALSE;
    wiced_bool_t jtag_delay_elapsed = ( host_rtos_get_time() > JTAG_DEBUG_SLEEP_DELAY_MS ) ? WICED_TRUE : WICED_FALSE;
    uint32_t     elapsed_cycles     = 0;

    /* Criteria to enter WAIT mode
     * 1. Clock needed counter is 0 and no JTAG debugging
     * 2. Clock needed counter is 0, in JTAG debugging session, and WICED system tick has progressed over 3 seconds.
     *    This is to give OpenOCD enough time to poke the JTAG tap before the CPU enters WAIT mode.
     */
    if ( ( sam4s_clock_needed_counter == 0 ) && ( ( jtag_enabled == WICED_FALSE ) || ( ( jtag_enabled == WICED_TRUE ) && ( jtag_delay_elapsed == WICED_TRUE ) ) ) )
    {
        uint32_t total_sleep_cycles;
        uint32_t total_delay_cycles;

        /* Start real-time timer */
        rtt_init( RTT, RTT_CLOCK_PRESCALER );

        /* Start atomic operation */
        DISABLE_INTERRUPTS;

        /* Ensure deep sleep bit is enabled, otherwise system doesn't go into deep sleep */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        /* Disable SysTick */
        SysTick->CTRL &= ( ~( SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk ) );

        /* End atomic operation */
        ENABLE_INTERRUPTS;

        /* Expected total time CPU executing in this function (including WAIT mode time) */
        total_sleep_cycles = MS_TO_CYCLES( delay_ms );

        /* Total cycles in WAIT mode loop */
        total_delay_cycles = ( total_sleep_cycles / RTT_MAX_CYCLES + 1 ) * RC_OSC_DELAY_CYCLES + WAIT_MODE_ENTER_DELAY_CYCLES + WAIT_MODE_EXIT_DELAY_CYCLES;

        if ( total_sleep_cycles > total_delay_cycles )
        {
            /* Adjust total sleep cycle to exclude exit delay */
            total_sleep_cycles -= WAIT_MODE_EXIT_DELAY_CYCLES;

            /* Prepare platform specific settings before entering powersave */
        	platform_enter_powersave();

            /* Prepare WLAN bus before entering powersave */
            host_platform_bus_enter_powersave();

            /* Disable brownout detector */
            supc_disable_brownout_detector( SUPC );

            /* Backup system I/0 functions and set all to GPIO to save power */
            system_io_backup_value = matrix_get_system_io();
            matrix_set_system_io( 0x0CF0 );

            /* Switch Master Clock to Main Clock (internal fast RC oscillator) */
            pmc_switch_mck_to_mainck( PMC_PCK_PRES_CLK_1 );

            /* Switch on internal fast RC oscillator, switch Main Clock source to internal fast RC oscillator and disables external fast crystal */
            pmc_switch_mainck_to_fastrc( CKGR_MOR_MOSCRCF_4_MHz );

            /* Disable external fast crystal */
            pmc_osc_disable_xtal( 0 );

            /* Disable PLLA */
            pmc_disable_pllack( );

            /* This above process introduces certain delay. Add delay to the elapsed cycles */
            elapsed_cycles += rtt_read_timer_value( RTT );

            while ( wake_up_interrupt_triggered == WICED_FALSE  && elapsed_cycles < total_sleep_cycles )
            {
                uint32_t current_sleep_cycles = total_sleep_cycles - elapsed_cycles;

                /* Start real-time timer and alarm */
                rtt_init( RTT, RTT_CLOCK_PRESCALER );
                rtt_write_alarm_time( RTT, ( current_sleep_cycles > RTT_MAX_CYCLES ) ? RTT_MAX_CYCLES - RC_OSC_DELAY_CYCLES : current_sleep_cycles - RC_OSC_DELAY_CYCLES );

                /* Enter WAIT mode */
                pmc_enable_waitmode();

                /* Clear wake-up status */
                rtt_get_status( RTT );

                /* Add sleep time to the elapsed cycles */
                elapsed_cycles += rtt_read_timer_value( RTT );
            }

            /* Re-enable real-time timer to time clock reinitialisation delay */
            rtt_init( RTT, RTT_CLOCK_PRESCALER );

            /* Reinit fast clock. This takes ~19ms, but the timing has been compensated */
            init_clocks();

            /* Disable unused clock to save power */
            pmc_osc_disable_fastrc();

            /* Restore system I/O pins */
            matrix_set_system_io( system_io_backup_value );

            /* Restore WLAN bus */
            host_platform_bus_exit_powersave();

            /* Restore platform-specific settings */
            platform_exit_powersave();

            /* Add clock reinitialisation delay to elapsed cycles */
            elapsed_cycles += rtt_read_timer_value( RTT );

            /* Disable RTT to save power */
            RTT->RTT_MR = (uint32_t)( 1 << 20 );
        }
    }

    /* Start atomic operation */
    DISABLE_INTERRUPTS;

    /* Switch SysTick back on */
    SysTick->CTRL |= ( SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk );

    /* Clear flag indicating interrupt triggered by wake up pin */
    wake_up_interrupt_triggered = WICED_FALSE;

    /* End atomic operation */
    ENABLE_INTERRUPTS;

    /* Return total time in milliseconds */
    return CYCLES_TO_MS( elapsed_cycles );
}
#else /* WAIT_MODE_SUPPORT */
static unsigned long idle_power_down_hook( unsigned long delay_ms  )
{
    UNUSED_PARAMETER( delay_ms );
    ENABLE_INTERRUPTS;
    __asm("wfi");
    return 0;
}
#endif /* WAIT_MODE_SUPPORT */
