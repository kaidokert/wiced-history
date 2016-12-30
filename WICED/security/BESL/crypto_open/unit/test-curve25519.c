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

/*
 * Portable C implementation from https://code.google.com/p/curve25519-donna/
 * Original implementation from http://cr.yp.to/ecdh.html
 */

/*
test-curve25519 version 20050915
D. J. Bernstein
Public domain.

Tiny modifications by agl
*/

#include <stdio.h>
#include <string.h>
#include "curve25519.h"

#ifdef CURVE25519_EXTREMELY_LONG_TEST
const int loop_max=10000;
const char e1_expected[32]    = { 0x4f, 0xaf, 0x81, 0x19, 0x08, 0x69, 0xfd, 0x74,
                                  0x2a, 0x33, 0x69, 0x1b, 0x0e, 0x08, 0x24, 0xd5,
                                  0x7e, 0x03, 0x29, 0xf4, 0xdd, 0x28, 0x19, 0xf5,
                                  0xf3, 0x2d, 0x13, 0x0f, 0x12, 0x96, 0xb5, 0x00,
                                };
const char e2k_expected[32]   = { 0x05, 0xae, 0xc1, 0x3f, 0x92, 0x28, 0x6f, 0x3a,
                                  0x78, 0x1c, 0xca, 0xe9, 0x89, 0x95, 0xa3, 0xb9,
                                  0xe0, 0x54, 0x47, 0x70, 0xbc, 0x7d, 0xe8, 0x53,
                                  0xb3, 0x8f, 0x91, 0x00, 0x48, 0x9e, 0x3e, 0x79,
                                };
const char e1e2k_expected[32] = { 0xcd, 0x6e, 0x82, 0x69, 0x10, 0x4e, 0xb5, 0xaa,
                                  0xee, 0x88, 0x6b, 0xd2, 0x07, 0x1f, 0xba, 0x88,
                                  0xbd, 0x13, 0x86, 0x14, 0x75, 0x51, 0x6b, 0xc2,
                                  0xcd, 0x2b, 0x6e, 0x00, 0x5e, 0x80, 0x50, 0x64,
                                };

#elif defined ( CURVE25519_LONG_TEST ) || defined( LONG_UNIT_TEST )
const int loop_max=200;
const char e1_expected[32]    = { 0xBC, 0x71, 0x12, 0xCD, 0xE0, 0x3F, 0x97, 0xEF,
                                  0x70, 0x08, 0xCA, 0xD1, 0xBD, 0xC5, 0x6B, 0xE3,
                                  0xC6, 0xA1, 0x03, 0x7D, 0x74, 0xCC, 0xEB, 0x37,
                                  0x12, 0xE9, 0x20, 0x68, 0x71, 0xDC, 0xF6, 0x54,
                                };
const char e2k_expected[32]   = { 0xdd, 0x8f, 0xa2, 0x54, 0xfb, 0x60, 0xbd, 0xb5,
                                  0x14, 0x2f, 0xe0, 0x5b, 0x1f, 0x5d, 0xe4, 0x4d,
                                  0x8e, 0x3e, 0xe1, 0xa6, 0x3c, 0x7d, 0x14, 0x27,
                                  0x4e, 0xa5, 0xd4, 0xc6, 0x7f, 0x06, 0x54, 0x67,
                                };
const char e1e2k_expected[32] = { 0x7d, 0xdb, 0x98, 0xbd, 0x89, 0x02, 0x5d, 0x23,
                                  0x47, 0x77, 0x6b, 0x33, 0x90, 0x1b, 0x3e, 0x7e,
                                  0xc0, 0xee, 0x98, 0xcb, 0x22, 0x57, 0xa4, 0x54,
                                  0x5c, 0x0c, 0xfb, 0x2c, 0xa3, 0xe1, 0x81, 0x2b,
                                };


#else
const int loop_max=5;
const char e1_expected[32]    = { 0xF0, 0x7A, 0xD3, 0xF8, 0x2B, 0xB4, 0xE3, 0x43,
                                  0xAA, 0x88, 0xD6, 0xDB, 0xD7, 0xAF, 0x6F, 0xE9,
                                  0xBF, 0x08, 0xD2, 0x09, 0x71, 0x13, 0xE0, 0x8E,
                                  0x31, 0x01, 0x53, 0x0A, 0xCD, 0xFC, 0x38, 0x21,
                                };

const char e2k_expected[32]   = { 0x97, 0x86, 0xCC, 0x9D, 0x9F, 0x2F, 0x02, 0xA0,
                                  0x22, 0x03, 0x18, 0x67, 0x5A, 0x08, 0x5C, 0xE2,
                                  0x4B, 0xFA, 0x1E, 0x2B, 0x59, 0xBB, 0xF3, 0x97,
                                  0xD7, 0x80, 0x9B, 0x26, 0xAA, 0xF4, 0x2A, 0x1E,
                                };

const char e1e2k_expected[32] = { 0x6F, 0xBA, 0x98, 0xC4, 0xF2, 0x94, 0x1C, 0x28,
                                  0xA5, 0xC8, 0x9E, 0x87, 0xEE, 0xDB, 0x81, 0xAD,
                                  0x35, 0xE1, 0x99, 0x60, 0x53, 0x1E, 0x79, 0x2D,
                                  0x9F, 0xF7, 0xAF, 0x18, 0xF4, 0xCB, 0xB2, 0x4A,
                                };

#endif


int curve25519_self_test( void )
{

    int loop;
    int i;
    unsigned char e1k[32];
    unsigned char e2k[32];
    unsigned char e1e2k[32];
    unsigned char e2e1k[32];
    unsigned char e1[32] = {3};
    unsigned char e2[32] = {5};
    unsigned char k[32] = {9};



    for (loop = 0;loop < loop_max;++loop)
    {
        curve25519(e1k,e1,k);
        curve25519(e2e1k,e2,e1k);
        curve25519(e2k,e2,k);
        curve25519(e1e2k,e1,e2k);

        if ( 0 != memcmp( e1e2k, e2e1k, 32 ) )
        {
            printf("curve25519 fail\n");
            return -1;
        }
        if (loop == loop_max-1)
        {
            break;
        }
        for (i = 0;i < 32;++i) e1[i] ^= e2k[i];
        for (i = 0;i < 32;++i) e2[i] ^= e1k[i];
        for (i = 0;i < 32;++i) k[i] ^= e1e2k[i];
    }

    if ( ( 0 != memcmp( e1, e1_expected, 32 ) ) ||
         ( 0 != memcmp( e2k, e2k_expected, 32 ) ) ||
         ( 0 != memcmp( e1e2k, e1e2k_expected, 32 ) ) )
    {
        printf("curve25519 fail\n");
        return -1;
    }

    return 0;
}
