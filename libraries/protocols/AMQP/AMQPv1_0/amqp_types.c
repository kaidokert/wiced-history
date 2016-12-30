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
 *  I/O functions
 *
 *  Provides functions for sending and receiving to the network for use by
 *  framing layer.
 */
#include "wiced.h"
#include "amqp_types.h"

/******************************************************
 *                      Macros
 ******************************************************/

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

typedef struct AMQP_LIST_T
{
    AMQP_VALUE *list_element;
    uint32_t count;
} AMQP_LIST;
typedef struct AMQP_MAP_KEY_VALUE_PAIR_T
{
    AMQP_VALUE key;
    AMQP_VALUE value;
} AMQP_MAP_KEY_VALUE_PAIR;

typedef struct AMQP_MAP_T
{
    AMQP_MAP_KEY_VALUE_PAIR* pairs;
    uint32_t pair_count;
} AMQP_MAP;

typedef struct AMQP_ARRAY_T
{
    AMQP_VALUE items;
    uint32_t count;
} AMQP_ARRAY;

union AMQP_VALUE_UNION_T
{
    //DESCRIBED_VALUE described_value;
    unsigned char ubyte_value;
    uint16_t ushort_value;
    uint32_t uint_value;
    uint64_t ulong_value;
    char byte_value;
    int16_t short_value;
    int32_t int_value;
    int64_t long_value;
    uint8_t bool_value;
    float float_value;
    double double_value;
    uint32_t char_value;
    int64_t timestamp_value;
    uuid uuid_value;
    AMQP_STRING string_value;
    AMQP_BINARY binary_value;
    AMQP_LIST list_value;
    AMQP_MAP map_value;
    AMQP_ARRAY array_value;
    AMQP_SYMBOL symbol_value;
};

typedef struct AMQP_LIST_ELEMENT_T
{
    AMQP_TYPE type;
    AMQP_VALUE_UNION value;
} AMQP_LIST_ELEMENT;

/*sequence primitive type*/
typedef int32_t amqp_sequence_t;

/*miliseconds primitive type*/
typedef uint32_t amqp_millis_t;

/*seconds primitive type*/
typedef uint32_t amqp_seconds_t;

/*timestamp primitive type*/
typedef int64_t amqp_timestamp_t;

/*char primitive type*/
typedef uint32_t amqp_char_t;

/* decimal 32 primitive type*/
typedef uint32_t amqp_decimal32_t;

/*decimal 64 primitive type*/
typedef uint64_t amqp_decimal64_t;

/*decimal 128 primitive type*/
typedef struct
{
    char bytes[ 16 ];
} amqp_decimal128_t;

/*uuid primitive type*/
typedef struct
{
    char bytes[ 16 ];
} amqp_uuid_t;

/*bytes primitive type*/
typedef struct
{
    size_t size;
    const char *start;
} amqp_bytes_t;

/******************************************************
 *                   Function prototypes
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
void* amqpalloc_malloc( size_t size );
void amqpalloc_free( void* ptr );

void* amqpalloc_malloc( size_t size )
{
    void* result;
    result = malloc( size );
    return result;
}

void amqpalloc_free( void* ptr )
{
    free( ptr );
}

AMQP_VALUE amqpvalue_create_null( void )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {

        result->type = AMQP_TYPE_NULL;
    }
    return result;
}

AMQP_VALUE amqpvalue_create_boolean( uint8_t value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {

        result->type = AMQP_TYPE_BOOL;
        result->value.bool_value = value;
    }

    return result;
}

int amqpvalue_get_boolean( AMQP_VALUE value, uint8_t* bool_value )
{
    int result;

    if ( ( value == NULL ) || ( bool_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_BOOL )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for bool value = [%d]\r\n", result ) );
        }
        else
        {
            *bool_value = value_data->value.bool_value;
            AMQP_DEBUG( ( "[AMQP DEBUG] :ubyte value = [%d]\r\n", *bool_value ) );
            result = 0;
        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_ubyte( unsigned char value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_UBYTE;
        result->value.ubyte_value = value;
    }

    return result;
}

int amqpvalue_get_ubyte( AMQP_VALUE value, unsigned char* ubyte_value )
{
    int result;

    if ( ( value == NULL ) || ( ubyte_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        /* Codes_SRS_AMQPVALUE_01_037: [If the type of the value is not ubyte (was not created with amqpvalue_create_ubyte), then amqpvalue_get_ubyte shall return a non-zero value.] */
        if ( value_data->type != AMQP_TYPE_UBYTE )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for ubyte value = [%d]\r\n", result ) );
        }
        else
        {
            *ubyte_value = value_data->value.ubyte_value;
            AMQP_DEBUG( ( "[AMQP DEBUG] :ubyte value = [%c]\r\n", *ubyte_value ) );
            result = 0;
        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_ushort( uint16_t value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {

        result->type = AMQP_TYPE_USHORT;
        result->value.ushort_value = value;
    }
    return result;
}

int amqpvalue_get_ushort( AMQP_VALUE value, uint16_t* ushort_value )
{
    int result;

    if ( ( value == NULL ) || ( ushort_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_USHORT )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for ushort value = [%d]\r\n", result ) );
        }
        else
        {
            *ushort_value = value_data->value.ushort_value;

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : uint value. is = [%d]\r\n", *ushort_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_uint( uint32_t value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_UINT;
        result->value.uint_value = value;
    }
    return result;
}

int amqpvalue_get_uint( AMQP_VALUE value, uint32_t* uint_value )
{
    int result;

    if ( ( value == NULL ) || ( uint_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );

    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_UINT )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for uint value = [%d]\r\n", result ) );
        }
        else
        {
            *uint_value = value_data->value.uint_value;
            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : uint value. is = [%ld]\r\n", *uint_value ) );
        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_ulong( uint64_t value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_ULONG;
        result->value.ulong_value = value;
    }
    return result;
}

int amqpvalue_get_ulong( AMQP_VALUE value, uint64_t* ulong_value )
{
    int result;

    if ( ( value == NULL ) || ( ulong_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_ULONG )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for ulong value = [%d]\r\n", result ) );
        }
        else
        {
            *ulong_value = value_data->value.ulong_value;

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : ulong value. is = [%lld]\r\n", *ulong_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_byte( char value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_BYTE;
        result->value.byte_value = value;
    }
    return result;
}

int amqpvalue_get_byte( AMQP_VALUE value, char* byte_value )
{
    int result;

    if ( ( value == NULL ) || ( byte_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_BYTE )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for ubyte value = [%d]\r\n", result ) );
        }
        else
        {
            *byte_value = value_data->value.byte_value;
            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : byte value. is = [%d]\r\n", *byte_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_short( int16_t value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {

        result->type = AMQP_TYPE_SHORT;
        result->value.short_value = value;
    }
    return result;
}

int amqpvalue_get_short( AMQP_VALUE value, int16_t* short_value )
{
    int result;

    if ( ( value == NULL ) || ( short_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_SHORT )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for ubyte value = [%d]\r\n", result ) );
        }
        else
        {
            *short_value = value_data->value.short_value;

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : short value. is = [%d]\r\n", *short_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_int( int32_t value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_INT;
        result->value.int_value = value;
    }
    return result;
}

int amqpvalue_get_int( AMQP_VALUE value, int32_t* int_value )
{
    int result;

    if ( ( value == NULL ) || ( int_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_INT )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for int value = [%d]\r\n", result ) );
        }
        else
        {
            *int_value = value_data->value.int_value;

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : int value. is = [%ld]\r\n", *int_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_long( int64_t value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_LONG;
        result->value.long_value = value;
    }
    return result;
}

int amqpvalue_get_long( AMQP_VALUE value, int64_t* long_value )
{
    int result;

    if ( ( value == NULL ) || ( long_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        /* Codes_SRS_AMQPVALUE_01_078: [If the type of the value is not long (was not created with amqpvalue_create_long), then amqpvalue_get_long shall return a non-zero value.] */
        if ( value_data->type != AMQP_TYPE_LONG )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for long value = [%d]\r\n", result ) );
        }
        else
        {
            *long_value = value_data->value.long_value;
            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : long value. is = [%lld]\r\n", *long_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_float( float value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_FLOAT;
        result->value.float_value = value;
    }
    return result;
}

int amqpvalue_get_float( AMQP_VALUE value, float* float_value )
{
    int result;

    if ( ( value == NULL ) || ( float_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_FLOAT )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for float value = [%d]\r\n", result ) );
        }
        else
        {
            *float_value = value_data->value.float_value;

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : float value. is = [%f]\r\n", (double)*float_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_double( double value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_DOUBLE;
        result->value.double_value = value;
    }
    return result;
}

int amqpvalue_get_double( AMQP_VALUE value, double* double_value )
{
    int result;

    if ( ( value == NULL ) || ( double_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_DOUBLE )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for double value = [%d]\r\n", result ) );
        }
        else
        {
            *double_value = value_data->value.double_value;

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : double value. is = [%f]\r\n", *double_value ) );

        }
    }

    return result;
}

/* Codes_SRS_AMQPVALUE_01_024: [1.6.16 char A single Unicode character.] */
AMQP_VALUE amqpvalue_create_char( uint32_t value )
{
    AMQP_VALUE result;

    /* Codes_SRS_AMQPVALUE_01_098: [If the code point value is outside of the allowed range [0, 0x10FFFF] then amqpvalue_create_char shall return NULL.] */
    if ( value > 0x10FFFF )
    {
        result = NULL;
        AMQP_DEBUG( ( "[AMQP DEBUG] :Invalid input value\r\n" ) );

    }
    else
    {
        result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
        /* Codes_SRS_AMQPVALUE_01_093: [If allocating the AMQP_VALUE fails then amqpvalue_create_char shall return NULL.] */
        if ( result != NULL )
        {

            result->type = AMQP_TYPE_CHAR;
            result->value.char_value = value;
        }
    }

    return result;
}

int amqpvalue_get_char( AMQP_VALUE value, uint32_t* char_value )
{
    int result;

    if ( ( value == NULL ) || ( char_value == NULL ) )
    {
        AMQP_DEBUG( ( "[AMQP DEBUG] :check input arguments\r\n" ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_CHAR )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for char value = [%d]\r\n", result ) );
        }
        else
        {
            *char_value = value_data->value.char_value;
            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : char value. is = [%ld]\r\n", *char_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_timestamp( int64_t value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    /* Codes_SRS_AMQPVALUE_01_108: [If allocating the AMQP_VALUE fails then amqpvalue_create_timestamp shall return NULL.] */
    if ( result != NULL )
    {
                result->type = AMQP_TYPE_TIMESTAMP;
        result->value.timestamp_value = value;
    }
    return result;
}

int amqpvalue_get_timestamp( AMQP_VALUE value, int64_t* timestamp_value )
{
    int result;

    if ( ( value == NULL ) || ( timestamp_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_TIMESTAMP )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for timestamp value = [%d]\r\n", result ) );
        }
        else
        {
            *timestamp_value = value_data->value.timestamp_value;

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : timestamp value. is = [%lld]\r\n", *timestamp_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_uuid( uuid value )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_UUID;
        (void) memcpy( &result->value.uuid_value, value, 16 );
    }
    return result;
}

int amqpvalue_get_uuid( AMQP_VALUE value, uuid* uuid_value )
{
    int result;

    if ( ( value == NULL ) || ( uuid_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_UUID )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for uuid value = [%d]\r\n", result ) );
        }
        else
        {
            (void) memcpy( *uuid_value, value_data->value.uuid_value, 16 );

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] : uuid value. is = [%s]\r\n",(char*) uuid_value ) );

        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_binary( AMQP_BINARY value )
{
    AMQP_VALUE result;
    if ( ( value.bytes == NULL ) && ( value.length > 0 ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
        result = NULL;
    }
    else
    {
        result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
        if ( result != NULL )
        {
            result->type = AMQP_TYPE_BINARY;
            if ( value.length > 0 )
            {
                result->value.binary_value.bytes = amqpalloc_malloc( value.length );
            }
            else
            {
                result->value.binary_value.bytes = NULL;
            }

            result->value.binary_value.length = value.length;

            if ( ( result->value.binary_value.bytes == NULL ) && ( value.length > 0 ) )
            {
                amqpalloc_free( result );
                result = NULL;
            }
            else
            {
                if ( value.length > 0 )
                {
                    (void) memcpy( (void*) result->value.binary_value.bytes, value.bytes, value.length );
                }
            }
        }
    }
    return result;
}

int amqpvalue_get_binary( AMQP_VALUE value, AMQP_BINARY* binary_value )
{
    int result;

    if ( ( value == NULL ) || ( binary_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;
        if ( value_data->type != AMQP_TYPE_BINARY )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for binary value = [%d]\r\n", result ) );
        }
        else
        {
            binary_value->length = value_data->value.binary_value.length;
            binary_value->bytes = value_data->value.binary_value.bytes;
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG]  binary data bytes %.*s  for TOPIC : %.*s\n\n", (int) binary_value->length, binary_value->bytes ) );
            result = 0;
        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_string( const char* value )
{
    AMQP_VALUE result;
    if ( value == NULL )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : INPUT IS NULL \r\n" ) );

        result = NULL;
    }
    else
    {
        size_t length = strlen( value );

        result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
        if ( result != NULL )
        {
            result->type = AMQP_TYPE_STRING;
            result->value.string_value.str = amqpalloc_malloc( length + 1 );
            if ( result->value.string_value.str == NULL )
            {
                amqpalloc_free( result );
                result = NULL;
            }
            else
            {
                (void) strcpy( result->value.string_value.str, value );
            }
        }
    }

    return result;
}

int amqpvalue_get_string( AMQP_VALUE value, const char** string_value )
{
    int result;

    if ( ( value == NULL ) || ( string_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;

        if ( value_data->type != AMQP_TYPE_STRING )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for string value = [%d]\r\n", result ) );
        }
        else
        {
            *string_value = value_data->value.string_value.str;

            result = 0;
        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_symbol( const char* value )
{
    AMQP_VALUE result;
    if ( value == NULL )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : INPUT IS NULL \r\n" ) );
        result = NULL;
    }
    else
    {
        size_t length = strlen( value );
        if ( length > UINT32_MAX )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : string too long to be represented as a symbol \r\n" ) );

            result = NULL;
        }
        else
        {
            result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
            if ( result == NULL )
            {
                result = NULL;
                AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Cannot allocate memory for AMQP value \r\n" ) );

            }
            else
            {
                result->type = AMQP_TYPE_SYMBOL;
                result->value.symbol_value.str = (char*) amqpalloc_malloc( length + 1 );
                if ( result->value.symbol_value.str == NULL )
                {
                    AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Cannot allocate memory for symbol string \r\n" ) );

                    amqpalloc_free( result );
                    result = NULL;
                }
                else
                {
                    (void) memcpy( result->value.symbol_value.str, value, length + 1 );
                }
            }
        }
    }

    return result;
}

int amqpvalue_get_symbol( AMQP_VALUE value, const char** symbol_value )
{
    int result;

    if ( ( value == NULL ) || ( symbol_value == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;

        if ( value_data->type != AMQP_TYPE_SYMBOL )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for symbol value = [%d]\r\n", result ) );
        }
        else
        {
            *symbol_value = value_data->value.symbol_value.str;
            result = 0;
        }
    }

    return result;
}

AMQP_VALUE amqpvalue_create_list( void )
{
    AMQP_VALUE result = (AMQP_VALUE) amqpalloc_malloc( sizeof(AMQP_VALUE) );
    if ( result != NULL )
    {
        result->type = AMQP_TYPE_LIST;

        result->value.list_value.count = 0;
        result->value.list_value.list_element = NULL;

    }

    return result;
}

int amqpvalue_get_list_item_count( AMQP_VALUE value, uint32_t* size )
{
    int result;

    if ( ( value == NULL ) || ( size == NULL ) )
    {
        AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] : Check the input arguments. debug at line number = [%d]\r\n", __LINE__ ) );
    }
    else
    {
        AMQP_VALUE value_data = (AMQP_VALUE) value;

        if ( value_data->type != AMQP_TYPE_LIST )
        {
            AMQP_DEBUG_ERROR( ( "[AMQP DEBUG] :type field mismatch for list value = [%d]\r\n", result ) );
        }
        else
        {
            *size = value_data->value.list_value.count;

            result = 0;
            AMQP_DEBUG( ( "[AMQP DEBUG] :list count is  = [%ld]\r\n", *size ) );

        }
    }

    return result;
}

//
//int amqpvalue_set_list_item(AMQP_VALUE value, uint32_t index, AMQP_VALUE list_item_value)
//{
//
//}
//
//AMQP_VALUE amqpvalue_get_list_item(AMQP_VALUE value, size_t index)
//{
//
//}
//

/*-------------------Encoding---------------------*/

