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

#include "dtls_types.h"
#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif


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

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/*****************************************************************************/
/** @addtogroup dtls       DTLS Security
 *  @ingroup ipcoms
 *
 * Security initialisation functions for DTLS enabled connections (Datagram Transport Layer Security)
 *
 *  @{
 */
/*****************************************************************************/

/*****************************************************************************/
/** Initialises a simple DTLS context handle
 *
 * @param[out] context     : A pointer to a wiced_dtls_context_t context object that will be initialised
 * @param[out] identity    : A pointer to a wiced_tls_identity_t object that will be initialised
 *
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_dtls_init_context( wiced_dtls_context_t* context, wiced_dtls_identity_t* identity, const char* peer_cn );


/** Initialises a DTLS identity using a supplied certificate and private key
 *
 * @param[out] identity          : A pointer to a wiced_tls_identity_t object that will be initialised
 * @param[in] private_key        : The server private key in binary format
 * @param[in] certificate_data   : The server x509 certificate in PEM or DER format
 * @param[in] certificate_length : The length of the certificate
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_dtls_init_identity( wiced_dtls_identity_t* identity, const char* private_key, const uint32_t key_length, const uint8_t* certificate_data, uint32_t certificate_length );

/** DeiInitialises a DTLS identity
 *
 * @param[in] identity    : A pointer to a wiced_dtls_identity_t object that will be de-initialised
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_dtls_deinit_identity( wiced_dtls_identity_t* dtls_identity);

/** De-initialise a previously inited DTLS context
 *
 * @param[in,out] context : a pointer to a wiced_dtls_context_t object
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_dtls_deinit_context( wiced_dtls_context_t* context );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
