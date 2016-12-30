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

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced_audio.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_CLIENT_RENDER_VOLUME_MIN     (0)
#define AUDIO_CLIENT_RENDER_VOLUME_MAX     (100)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_CLIENT_RENDER_CMD_STOP = 0,
    AUDIO_CLIENT_RENDER_CMD_FLUSH,
    AUDIO_CLIENT_RENDER_CMD_PAUSE,
    AUDIO_CLIENT_RENDER_CMD_RESUME,
    AUDIO_CLIENT_RENDER_CMD_VOLUME,
    AUDIO_CLIENT_RENDER_CMD_SET_EFFECT

} AUDIO_CLIENT_RENDER_CMD_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/**
 * Audio buffer structure.
 */

typedef struct audio_client_render_buf_s
{
    struct audio_client_render_buf_s* next;

    uint8_t* data_buf;
    uint32_t buf_length;
    uint32_t data_offset;
    uint32_t data_length;
} audio_client_render_buf_t;

/**
 * Callback for playback complete.
 */

typedef wiced_result_t (*audio_client_render_cb_t)(void* session_ptr);

/******************************************************
 *                    Structures
 ******************************************************/

/**
 * Configuration parameters for starting audio client render.
 */

typedef struct
{
    uint32_t                    buffer_nodes;           /* Number of buffer nodes for audio render to allocate                      */
    uint32_t                    buffer_size;            /* Size of each buffer node for audio render to allocate                    */
    platform_audio_device_id_t  device_id;              /* Audio device identifier                                                  */
    void*                       session_ptr;            /* Application session pointer passed back in the following callbacks       */
    audio_client_render_cb_t    playback_complete_cb;   /* Playback complete callback                                               */
} audio_client_render_params_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

typedef struct audio_client_render_s *audio_client_render_ref;

/** Initialize the audio client render library.
 *
 * @param[in] params : Pointer to the audio configuration parameters.
 *
 * @return Pointer to the audio client render instance or NULL
 */
audio_client_render_ref audio_client_render_init(audio_client_render_params_t* params);

/** Deinitialize the audio client render library.
 *
 * @param[in] audio  : Pointer to the audio render instance.
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_client_render_deinit(audio_client_render_ref audio);

/** Configure the audio client render audio format.
 *
 * @param[in] audio  : Pointer to the audio render instance.
 * @param[in] config : Pointer to the audio configuration.
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_client_render_configure(audio_client_render_ref audio, wiced_audio_config_t* config);

/** Send a command to the audio client render.
 *
 * @param[in] audio  : Pointer to the audio render instance.
 * @param[in] cmd    : The command to process.
 * @param[in] arg    : Pointer to command specific argument
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_client_render_command(audio_client_render_ref audio, AUDIO_CLIENT_RENDER_CMD_T cmd, void* arg);

/** Get an audio buffer from the audio client render library.
 *
 * @param[in] audio    : Pointer to the audio render instance.
 * @param[in/out] buf  : Address of pointer to an audio buffer.
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_client_render_buffer_get(audio_client_render_ref audio, audio_client_render_buf_t **buf);

/** Push an audio buffer to the audio client render library.
 * Note: Pushing a NULL buffering tells audio_render that no more
 * buffers will be sent for the current playback session. Audio_render
 * will then invoke the playback completion callback once the buffered
 * audio has finished playing.
 *
 * @param[in] audio  : Pointer to the audio render instance.
 * @param[in] buf    : Pointer to the audio buffer.
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_client_render_buffer_push(audio_client_render_ref audio, audio_client_render_buf_t* buf);

#ifdef __cplusplus
} /* extern "C" */
#endif
