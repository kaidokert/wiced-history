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

/* Originally taken from TropicSSL
 * https://gitorious.org/tropicssl/
 * commit: 92bb3462dfbdb4568c92be19e8904129a17b1eed
 * Whitespace converted (Tab to 4 spaces, LF to CRLF)
 * int arguments/returns changed to int32_t
 * key length changed to measure in bits
 * Rename aes_context_t elements to meaningful names
 * Change AES_ENCRYPT/AES_DECRYPT to enum
 */

/**
 * \file aes.h
 *
 *  Based on XySSL: Copyright (C) 2006-2008  Christophe Devine
 *
 *  Copyright (C) 2009  Paul Bakker <polarssl_maintainer at polarssl dot org>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the names of PolarSSL or XySSL nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TROPICSSL_AES_H
#define TROPICSSL_AES_H

#include "crypto_structures.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * \brief          AES key schedule (encryption)
     *
     * \param ctx      AES context to be initialized
     * \param key      encryption key
     * \param keysize_bits  must be 128, 192 or 256
     */
    void aes_setkey_enc(aes_context_t * ctx, const unsigned char *key, uint32_t keysize_bits);

    /**
     * \brief          AES key schedule (decryption)
     *
     * \param ctx      AES context to be initialized
     * \param key      decryption key
     * \param keysize_bits  must be 128, 192 or 256
     */
    void aes_setkey_dec(aes_context_t * ctx, const unsigned char *key, uint32_t keysize_bits);

    /**
     * \brief          AES-ECB block encryption/decryption
     *
     * \param ctx      AES context
     * \param mode     AES_ENCRYPT or AES_DECRYPT
     * \param input    16-byte input block
     * \param output   16-byte output block
     */
    void aes_crypt_ecb(aes_context_t * ctx,
               aes_mode_type_t mode,
               const unsigned char input[16],
               unsigned char output[16]);

    /**
     * \brief          AES-CBC buffer encryption/decryption
     *
     * \param ctx      AES context
     * \param mode     AES_ENCRYPT or AES_DECRYPT
     * \param length   length of the input data
     * \param iv       initialization vector (updated after use)
     * \param input    buffer holding the input data
     * \param output   buffer holding the output data
     */
    void aes_crypt_cbc(aes_context_t * ctx,
               aes_mode_type_t mode,
               uint32_t length,
               unsigned char iv[16],
               const unsigned char *input,
               unsigned char *output);

    /**
     * \brief          AES-CFB128 buffer encryption/decryption
     *
     * \param ctx      AES context
     * \param mode     AES_ENCRYPT or AES_DECRYPT
     * \param length   length of the input data
     * \param iv_off   offset in IV (updated after use)
     * \param iv       initialization vector (updated after use)
     * \param input    buffer holding the input data
     * \param output   buffer holding the output data
     */
    void aes_crypt_cfb128(aes_context_t * ctx,
                  aes_mode_type_t mode,
                  uint32_t length,
                  uint32_t* iv_off,
                  unsigned char iv[16],
                  const unsigned char *input,
                  unsigned char *output);

    /**
     * \brief          Checkup routine
     *
     * \return         0 if successful, or 1 if the test failed
     */
    int32_t aes_self_test(int32_t verbose);

    int32_t aes_padded_ccm_ctr_self_test(int32_t verbose);

#ifdef __cplusplus
}
#endif
#endif                /* aes.h */
