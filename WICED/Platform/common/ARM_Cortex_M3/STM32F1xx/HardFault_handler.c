/*
 * Copyright 2013, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */
#include "stm32f10x.h"
#include "core_cm3.h"

#ifdef DEBUG
#define DEBUG_HARDFAULT
#endif /* ifdef DEBUG */

/* Bit Definitions for SCB_CFSR */
#define  SCB_CFSR_UNALIGNED                  ((uint32_t)0x01000000)

#ifdef __GNUC__
#define TRIGGER_BREAKPOINT() __asm__("bkpt")
#elif defined ( __IAR_SYSTEMS_ICC__ )
#define TRIGGER_BREAKPOINT() __asm("bkpt 0")
#endif


typedef struct exception_stacked_registers_struct
{
    /* Stacked registers */
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R12;
    uint32_t LR;
    uint32_t PC;  /* (Return Address) */
    uint32_t PSR;
} exception_stacked_registers_t;

typedef enum EXC_RETURN_enum
{
    HANDLER_MSP_MSP = 0xF1, /* Return to Handler mode. Exception return gets state from MSP. Execution uses MSP after return. */
    THREAD_MSP_MSP  = 0xF9, /* Return to Thread mode.  Exception return gets state from MSP. Execution uses MSP after return. */
    THREAD_PSP_PSP  = 0xFD  /* Return to Thread mode.  Exception return gets state from PSP. Execution uses PSP after return. */
} EXC_RETURN_t;

void HardFaultException_handler( uint32_t MSP, uint32_t PSP, uint32_t LR );


#ifdef DEBUG_HARDFAULT
#ifdef __GNUC__
#pragma GCC optimize ("O0")
#endif /* ifdef __GNUC__ */

void HardFaultException_handler( uint32_t MSP, uint32_t PSP, uint32_t LR )
{
    exception_stacked_registers_t*  stackframe;
    uint32_t MMFAR = 0;
    uint32_t BFAR = 0;

    /* Get the Link Register value which contains the EXC_RETURN code */
    EXC_RETURN_t EXC_RETURN = LR & 0xff;

    /* The location of the stack frame of the offending code is indicated by the EXC_RETURN code */
    if ( ( EXC_RETURN & 0x00000004 ) != 0 )
    {
        stackframe = (exception_stacked_registers_t*) PSP;
    }
    else
    {
        stackframe = (exception_stacked_registers_t*) MSP;
    }
    (void) stackframe; /* may be unused */

    /* Disable interrupts - this is so that when debugger continues, it will go to caller, not an interrupt routine */
    /* This will mean the system cannot run properly when returning */
    __set_PRIMASK( 0x01 );

    /* Find cause of hardfault */
    if ( ( SCB->HFSR & SCB_HFSR_VECTTBL_Msk ) != 0 )
    {
        TRIGGER_BREAKPOINT(); /* Vector Table Hard Fault - Bus fault during vector table read during exception processing. */
    }
    else if ( ( SCB->HFSR & SCB_HFSR_FORCED_Msk ) != 0 )
    {
        /* Hard Fault is an escalated fault that was not handled */
        /* Need to read the other fault status registers */


        if ( ( SCB->CFSR & SCB_CFSR_MMARVALID    ) != 0 )
        {
            /* Memory Management Fault address register is valid - read it. */
            MMFAR = SCB->MMFAR;
        }

        if ( ( SCB->CFSR & SCB_CFSR_BFARVALID    ) != 0 )
        {
            /* Bus Fault address register is valid - read it. */
            BFAR = SCB->BFAR;
        }

        if ( ( SCB->CFSR & SCB_CFSR_IACCVIOL ) != 0 )
        {
            /* Memory Management Fault */
            TRIGGER_BREAKPOINT();  /* Instruction Access Violation - Attempt to execute an instruction from a region marked Execute Never */
            (void) stackframe->LR; /* Check this variable for the jump instruction that jumped to an invalid region */
            (void) stackframe->PC; /* Check this variable for the location that was attempted to be executed */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */

        }
        else if ( ( SCB->CFSR & SCB_CFSR_DACCVIOL     ) != 0 )
        {
            /* Memory Management Fault */
            TRIGGER_BREAKPOINT();  /* Data Access Violation */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
            (void) MMFAR;           /* Check this variable for the address of the attempted access */
            /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_MUNSTKERR    ) != 0 )
        {
            /* Memory Management Fault */
            TRIGGER_BREAKPOINT();  /* Unstacking fault returning from an exception - stack possibly corrupted during exception handler */
                                   /* New stackframe is not saved in this case */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_MSTKERR      ) != 0 )
        {
            /* Memory Management Fault */
            TRIGGER_BREAKPOINT();  /* Stacking fault whilst entering an exception - probably a bad stack pointer */
                                   /* Stack frame may be incorrect due to bad stack pointer */

        }
        else if ( ( SCB->CFSR & SCB_CFSR_IBUSERR      ) != 0 )
        {
            /* Bus Fault */
            TRIGGER_BREAKPOINT();  /* Instruction Bus Error whilst fetching an instruction*/
        }
        else if ( ( SCB->CFSR & SCB_CFSR_PRECISERR    ) != 0 )
        {
            /* Bus Fault */
            TRIGGER_BREAKPOINT();  /* Precise Data Bus Error - i.e. Data Bus fault at well defined location */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
            (void) BFAR;           /* Check this variable for the faulting address */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_IMPRECISERR  ) != 0 )
        {
            /* Bus Fault */
            TRIGGER_BREAKPOINT();  /* Imprecise Data Bus Error - i.e. Data Bus fault occurred but details have been lost due to priorities delaying processing of the fault */
                                   /* No fault details are available in this case*/
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_UNSTKERR     ) != 0 )
        {
            /* Bus Fault */
            TRIGGER_BREAKPOINT();  /* Unstacking fault returning from an exception - stack possibly corrupted during exception handler */
                                   /* New stackframe is not saved in this case */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_STKERR       ) != 0 )
        {
            /* Bus Fault */
            TRIGGER_BREAKPOINT();  /* Stacking fault whilst entering an exception - probably a bad stack pointer */
                                   /* Stack frame may be incorrect due to bad stack pointer */

        }
        else if ( ( SCB->CFSR & SCB_CFSR_UNDEFINSTR   ) != 0 )
        {
            /* Usage Fault */
            TRIGGER_BREAKPOINT();  /* Undefined Instruction Usage fault - probably corrupted memory in code space */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_INVSTATE     ) != 0 )
        {
            /* Usage Fault */
            TRIGGER_BREAKPOINT();  /* Invalid State usage fault - Illegal use of EPSR was attempted */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_INVPC        ) != 0 )
        {
            /* Usage Fault */
            TRIGGER_BREAKPOINT();  /* Invalid PC load usage fault - the EXC_RETURN value in LR was invalid on return from an exception - possibly stack corruption in exception */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_NOCP         ) != 0 )
        {
            /* Usage Fault */
            TRIGGER_BREAKPOINT();  /* No Coprocessor usage fault - coprocessor instruction attempted on processor without support for them */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_UNALIGNED   ) != 0 )
        {
            /* Usage Fault */
            TRIGGER_BREAKPOINT();  /* Unaligned access usage fault - Unaligned access whilst UNALIGN_TRP bit of SCB_CCR is set, or any unaligned access to LDM, STM, LDRD or STRD */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else if ( ( SCB->CFSR & SCB_CFSR_DIVBYZERO    ) != 0 )
        {
            /* Usage Fault */
            TRIGGER_BREAKPOINT();  /* Divide by zero usage fault */
            (void) stackframe->PC; /* Check this variable for the location of the offending instruction */
                                   /* You may try stepping past the return of this handler, which may return near the location of the error */
        }
        else
        {
            /* Unknown Fault */
            TRIGGER_BREAKPOINT();
            /* You may try stepping past the return of this handler, which may return near the location of the error */
        }

    }
    else
    {
        /* Unknown Hard Fault cause */
        TRIGGER_BREAKPOINT();
        /* You may try stepping past the return of this handler, which may return near the location of the error */
    }

    (void) MMFAR; /* This is for debug usage and need not be used programmatically */
    (void) BFAR; /* This is for debug usage and need not be used programmatically */
}
#ifdef __GNUC__
#pragma GCC reset_options
#endif /* ifdef __GNUC__ */

#else /* ifdef DEBUG_HARDFAULT */

void HardFaultException_handler( uint32_t MSP, uint32_t PSP, uint32_t LR )
{
    (void) MSP;
    (void) PSP;
    (void) LR;
}

#endif /* ifdef DEBUG_HARDFAULT */
