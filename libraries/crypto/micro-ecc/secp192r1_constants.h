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
#pragma once

#include "configuration.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#if (uECC_WORD_SIZE == 1)

#define uECC_WORDS    24
#define uECC_N_WORDS  24

#define Curve_P {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define Curve_B {0xB1, 0xB9, 0x46, 0xC1, 0xEC, 0xDE, 0xB8, 0xFE, 0x49, 0x30, 0x24, 0x72, 0xAB, 0xE9, 0xA7, 0x0F, 0xE7, 0x80, 0x9C, 0xE5, 0x19, 0x05, 0x21, 0x64}
#define Curve_G { {0x12, 0x10, 0xFF, 0x82, 0xFD, 0x0A, 0xFF, 0xF4, 0x00, 0x88, 0xA1, 0x43, 0xEB, 0x20, 0xBF, 0x7C, 0xF6, 0x90, 0x30, 0xB0, 0x0E, 0xA8, 0x8D, 0x18}, \
                  {0x11, 0x48, 0x79, 0x1E, 0xA1, 0x77, 0xF9, 0x73, 0xD5, 0xCD, 0x24, 0x6B, 0xED, 0x11, 0x10, 0x63, 0x78, 0xDA, 0xC8, 0xFF, 0x95, 0x2B, 0x19, 0x07} }
#define Curve_N {0x31, 0x28, 0xD2, 0xB4, 0xB1, 0xC9, 0x6B, 0x14, 0x36, 0xF8, 0xDE, 0x99, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

#elif (uECC_WORD_SIZE == 4)

#define uECC_WORDS    6
#define uECC_N_WORDS  6

#define Curve_P {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_B {0xC146B9B1, 0xFEB8DEEC, 0x72243049, 0x0FA7E9AB, 0xE59C80E7, 0x64210519}
#define Curve_G { {0x82FF1012, 0xF4FF0AFD, 0x43A18800, 0x7CBF20EB, 0xB03090F6, 0x188DA80E}, {0x1E794811, 0x73F977A1, 0x6B24CDD5, 0x631011ED, 0xFFC8DA78, 0x07192B95} }
#define Curve_N {0xB4D22831, 0x146BC9B1, 0x99DEF836, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}

#elif (uECC_WORD_SIZE == 8)

#define uECC_WORDS    3
#define uECC_N_WORDS  3

#define Curve_P {0xFFFFFFFFFFFFFFFFull, 0xFFFFFFFFFFFFFFFEull, 0xFFFFFFFFFFFFFFFFull}
#define Curve_B {0xFEB8DEECC146B9B1ull, 0x0FA7E9AB72243049ull, 0x64210519E59C80E7ull}
#define Curve_G { {0xF4FF0AFD82FF1012ull, 0x7CBF20EB43A18800ull, 0x188DA80EB03090F6ull}, {0x73F977A11E794811ull, 0x631011ED6B24CDD5ull, 0x07192B95FFC8DA78ull} }
#define Curve_N {0x146BC9B1B4D22831ull, 0xFFFFFFFF99DEF836ull, 0xFFFFFFFFFFFFFFFFull}

#endif /* (uECC_WORD_SIZE == 8) */

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


#ifdef __cplusplus
} /* extern "C" */
#endif
