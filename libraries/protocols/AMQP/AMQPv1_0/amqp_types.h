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
 *  AMQP frame APIs and types.
 *
 *  Internal types not to be included directly by applications.
 */
#pragma once

#include "wiced.h"
#include "amqp_helper_function.h"

#ifdef __cplusplus
extern "C"
{
#endif


typedef enum AMQP_TYPE_TAG
{
    AMQP_TYPE_NULL,
    AMQP_TYPE_BOOL,
    AMQP_TYPE_UBYTE,
    AMQP_TYPE_USHORT,
    AMQP_TYPE_UINT,
    AMQP_TYPE_ULONG,
    AMQP_TYPE_BYTE,
    AMQP_TYPE_SHORT,
    AMQP_TYPE_INT,
    AMQP_TYPE_LONG,
    AMQP_TYPE_FLOAT,
    AMQP_TYPE_DOUBLE,
    AMQP_TYPE_CHAR,
    AMQP_TYPE_TIMESTAMP,
    AMQP_TYPE_UUID,
    AMQP_TYPE_BINARY,
    AMQP_TYPE_STRING,
    AMQP_TYPE_SYMBOL,
    AMQP_TYPE_LIST,
    AMQP_TYPE_MAP,
    AMQP_TYPE_ARRAY,
    AMQP_TYPE_DESCRIBED,
    AMQP_TYPE_COMPOSITE,
    AMQP_TYPE_UNKNOWN
} AMQP_TYPE;

typedef union AMQP_VALUE_UNION_T AMQP_VALUE_UNION;

typedef struct AMQP_LIST_ELEMENT_T* AMQP_VALUE;

typedef unsigned char uuid[ 16 ];

typedef struct AMQP_STRING_T
{
        char* str;
} AMQP_STRING;

typedef struct AMQP_SYMBOL_T
{
        char* str;
} AMQP_SYMBOL;

typedef struct AMQP_BINARY_T
{
        const void* bytes;
        uint32_t length;
} AMQP_BINARY;

/******************************************************
 *                   Function prototypes
 ******************************************************/
AMQP_VALUE amqpvalue_create_null( void );

AMQP_VALUE amqpvalue_create_boolean( uint8_t value );
int amqpvalue_get_boolean( AMQP_VALUE value, uint8_t* uint8_t_value );
AMQP_VALUE amqpvalue_create_ubyte( unsigned char value );
int amqpvalue_get_ubyte( AMQP_VALUE value, unsigned char* ubyte_value );
AMQP_VALUE amqpvalue_create_ushort( uint16_t value );
int amqpvalue_get_ushort( AMQP_VALUE value, uint16_t* ushort_value );
AMQP_VALUE amqpvalue_create_uint( uint32_t value );
int amqpvalue_get_uint( AMQP_VALUE value, uint32_t* uint_value );
AMQP_VALUE amqpvalue_create_ulong( uint64_t value );
int amqpvalue_get_ulong( AMQP_VALUE value, uint64_t* ulong_value );
AMQP_VALUE amqpvalue_create_byte( char value );
int amqpvalue_get_byte( AMQP_VALUE value, char* byte_value );
AMQP_VALUE amqpvalue_create_short( int16_t value );
int amqpvalue_get_short( AMQP_VALUE value, int16_t* short_value );
AMQP_VALUE amqpvalue_create_int( int32_t value );
int amqpvalue_get_int( AMQP_VALUE value, int32_t* int_value );
AMQP_VALUE amqpvalue_create_long( int64_t value );
int amqpvalue_get_long( AMQP_VALUE value, int64_t* long_value );
AMQP_VALUE amqpvalue_create_float( float value );
int amqpvalue_get_float( AMQP_VALUE value, float* float_value );
AMQP_VALUE amqpvalue_create_double( double value );
int amqpvalue_get_double( AMQP_VALUE value, double* double_value );
/* Codes_SRS_AMQPVALUE_01_024: [1.6.16 char A single Unicode character.] */
AMQP_VALUE amqpvalue_create_char( uint32_t value );
int amqpvalue_get_char( AMQP_VALUE value, uint32_t* char_value );
AMQP_VALUE amqpvalue_create_timestamp( int64_t value );
int amqpvalue_get_timestamp( AMQP_VALUE value, int64_t* timestamp_value );
AMQP_VALUE amqpvalue_create_uuid( uuid value );
int amqpvalue_get_uuid( AMQP_VALUE value, uuid* uuid_value );
AMQP_VALUE amqpvalue_create_binary( AMQP_BINARY value );
int amqpvalue_get_binary( AMQP_VALUE value, AMQP_BINARY* binary_value );
AMQP_VALUE amqpvalue_create_string( const char* value );
int amqpvalue_get_string( AMQP_VALUE value, const char** string_value );
AMQP_VALUE amqpvalue_create_symbol( const char* value );
int amqpvalue_get_symbol( AMQP_VALUE value, const char** symbol_value );
AMQP_VALUE amqpvalue_create_list( void );
int amqpvalue_get_list_item_count( AMQP_VALUE value, uint32_t* size );

#ifdef __cplusplus
} /* extern "C" */
#endif
