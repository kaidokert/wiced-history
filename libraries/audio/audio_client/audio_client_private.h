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

/**
 * @file
 *
 * File Audio Client Internal Definitions
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced_tcpip.h"
#include "wiced_rtos.h"
#include "audio_client.h"
#include "audio_client_render.h"

/******************************************************
 *                     Macros
 ******************************************************/

#define CHECK_FOR_THRESHOLD_HIGH_EVENT(client)                                          \
    do                                                                                  \
    {                                                                                   \
        if (!client->threshold_high_sent && client->params.data_threshold_high > 0)     \
        {                                                                               \
            int in_use;                                                                 \
                                                                                        \
            in_use = client->data_buf_widx - client->data_buf_ridx;                     \
            if (in_use < 0)                                                             \
            {                                                                           \
                in_use += client->params.data_buffer_num;                               \
            }                                                                           \
            if (in_use >= client->params.data_threshold_high)                           \
            {                                                                           \
                client->threshold_high_sent = WICED_TRUE;                               \
                client->threshold_low_sent  = WICED_FALSE;                              \
                client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH, NULL); \
            }                                                                           \
        }                                                                               \
    } while (0)

#define CHECK_FOR_THRESHOLD_LOW_EVENT(client)                                           \
    do                                                                                  \
    {                                                                                   \
        if (!client->threshold_low_sent && client->params.data_threshold_high > 0)      \
        {                                                                               \
            int in_use;                                                                 \
                                                                                        \
            in_use = client->data_buf_widx - client->data_buf_ridx;                     \
            if (in_use < 0)                                                             \
            {                                                                           \
                in_use += client->params.data_buffer_num;                               \
            }                                                                           \
            if (in_use <= client->params.data_threshold_low && client->http_content_read < client->http_content_length) \
            {                                                                           \
                client->threshold_high_sent = WICED_FALSE;                              \
                client->threshold_low_sent  = WICED_TRUE;                               \
                client->params.event_cb(client, client->params.userdata, AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW, NULL); \
            }                                                                           \
        }                                                                               \
    } while (0)

/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_CLIENT_TAG_VALID              0x05EAf00D
#define AUDIO_CLIENT_TAG_INVALID            0xDEADBEEF

#define AUDIO_CLIENT_THREAD_PRIORITY        5
#define AUDIO_CLIENT_THREAD_STACK_SIZE      (2 * 4096)

#define AUDIO_CLIENT_DEFAULT_HTTP_PORT      (80)

#define AUDIO_CLIENT_HOSTNAME_LEN           (40)
#define AUDIO_CLIENT_PATH_LEN               (128)

#define AUDIO_CLIENT_HTTP_BUF_SIZE          (1024)

#define AUDIO_CLIENT_DATA_BUF_SIZE          (1500)

#define AUDIO_CLIENT_CONTENT_TYPE_SIZE      (128)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_CLIENT_CODEC_NULL = 0,
    AUDIO_CLIENT_CODEC_FLAC,
    AUDIO_CLIENT_CODEC_WAV,
    AUDIO_CLIENT_CODEC_AAC,

    AUDIO_CLIENT_CODEC_MAX
} AUDIO_CLIENT_CODEC_T;

typedef enum
{
    AUDIO_CLIENT_STATE_IDLE = 0,
    AUDIO_CLIENT_STATE_PLAY,
    AUDIO_CLIENT_STATE_PAUSE,

} AUDIO_CLIENT_STATE_T;

typedef enum
{
    AUDIO_CLIENT_EVENT_SHUTDOWN             = (1 <<  0),
    AUDIO_CLIENT_EVENT_PLAY                 = (1 <<  1),
    AUDIO_CLIENT_EVENT_STOP                 = (1 <<  2),
    AUDIO_CLIENT_EVENT_PAUSE                = (1 <<  3),
    AUDIO_CLIENT_EVENT_RESUME               = (1 <<  4),
    AUDIO_CLIENT_EVENT_VOLUME               = (1 <<  5),
    AUDIO_CLIENT_EVENT_SEEK                 = (1 <<  6),
    AUDIO_CLIENT_EVENT_EFFECT               = (1 <<  7),

    AUDIO_CLIENT_EVENT_HTTP_HEADER_COMPLETE = (1 << 15),

    AUDIO_CLIENT_EVENT_HTTP_ERROR           = (1 << 19),
    AUDIO_CLIENT_EVENT_HTTP_THREAD_DONE     = (1 << 20),
    AUDIO_CLIENT_EVENT_DECODER_THREAD_DONE  = (1 << 21),
    AUDIO_CLIENT_EVENT_PLAYBACK_COMPLETE    = (1 << 22),
} AUDIO_CLIENT_EVENTS_T;

#define AUDIO_CLIENT_ALL_EVENTS       (-1)

typedef enum
{
    HTTP_EVENT_TCP_DATA             = (1 <<  0),
    HTTP_EVENT_SEEK                 = (1 <<  1),
} HTTP_EVENTS_T;

#define HTTP_ALL_EVENTS       (-1)

typedef enum
{
    DECODER_EVENT_AUDIO_DATA        = (1 <<  0),
    DECODER_EVENT_FLUSH             = (1 <<  1),
} DECODER_EVENTS_T;

#define DECODER_ALL_EVENTS       (-1)

typedef enum
{
    DECODER_IOCTL_INFO = 0,             /* Arg is pointer to audio_client_stream_info_t                     */
    DECODER_IOCTL_GET_SEEK_POSITION,    /* Arg in is pointer to uint32_t (time in ms), out is byte position */
    DECODER_IOCTL_SET_POSITION,         /* Arg is new stream position in bytes                              */

    DECODER_IOCTL_MAX
} DECODER_IOCTL_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint64_t        stream_current_sample;  /* last played sample                   */
    uint64_t        stream_total_samples;   /* total samples in stream (if known)   */
    uint32_t        stream_sample_rate;     /* sample rate                          */
    uint8_t         stream_channels;        /* number of channels in source stream  */
    uint8_t         stream_bps;             /* bits per sample in source stream     */
} audio_client_stream_info_t;


typedef struct
{
    int         inuse;
    int         buflen;
    int         bufused;
    uint8_t     buf[AUDIO_CLIENT_DATA_BUF_SIZE];
} data_buf_t;


struct audio_client_s;

typedef wiced_result_t (*audio_client_decoder_start_t)(struct audio_client_s* client);
typedef wiced_result_t (*audio_client_decoder_stop_t)(struct audio_client_s* client);
typedef wiced_result_t (*audio_client_decoder_ioctl_t)(struct audio_client_s* client, DECODER_IOCTL_T ioctl, void* arg);

typedef struct
{
    audio_client_decoder_start_t            decoder_start;
    audio_client_decoder_stop_t             decoder_stop;
    audio_client_decoder_ioctl_t            decoder_ioctl;
} audio_client_decoder_api_t;


typedef struct audio_client_s
{
    uint32_t                    tag;
    AUDIO_CLIENT_STATE_T        state;

    wiced_thread_t              client_thread;
    wiced_thread_t*             client_thread_ptr;

    audio_client_params_t       params;
    int                         new_volume;

    wiced_event_flags_t         events;

    wiced_tcp_socket_t          socket;
    wiced_tcp_socket_t*         socket_ptr;

    AUDIO_CLIENT_CODEC_T        audio_codec;        /* which codec are we playing? */
    wiced_time_t                play_start_time;    /* when the last play started */

    wiced_bool_t                seek_in_progress;
    uint32_t                    seek_request_ms;
    uint32_t                    seek_position;

    uint32_t                    effect_mode;

    /*
     * HTTP information.
     */

    char*                       pending_uri;
    char                        hostname[AUDIO_CLIENT_HOSTNAME_LEN];
    char                        path[AUDIO_CLIENT_PATH_LEN];
    int                         port;
    char                        http_buf[AUDIO_CLIENT_HTTP_BUF_SIZE];           /* HTTP temporary buffer */
    int                         http_buf_idx;
    char                        http_content_type[AUDIO_CLIENT_CONTENT_TYPE_SIZE];

    wiced_thread_t              http_thread;
    wiced_thread_t*             http_thread_ptr;
    wiced_event_flags_t         http_events;

    wiced_bool_t                http_done;
    wiced_bool_t                http_error;
    wiced_bool_t                http_first_kick;
    wiced_bool_t                http_need_header;
    wiced_bool_t                http_in_header;
    uint8_t                     http_header_idx;
    uint8_t                     http_body_idx;
    wiced_bool_t                http_range_requests;                            /* Server supports range requests */
    uint32_t                    http_content_length;
    uint32_t                    http_content_read;
    uint32_t                    packets_read;
    uint32_t                    initial_buffer_count;

    wiced_bool_t                http_load_file;
    uint8_t*                    http_file_data;
    uint32_t                    http_file_idx;

    /*
     * Received data buffers.
     */

    data_buf_t                  *data_bufs;
    int                         data_buf_widx;
    int                         data_buf_ridx;

    wiced_bool_t                threshold_high_sent;                            /* Threshold high event sent    */
    wiced_bool_t                threshold_low_sent;                             /* Threshold low event sent     */

    /*
     * Decoder information.
     */

    audio_client_decoder_api_t  decoders[AUDIO_CLIENT_CODEC_MAX];
    wiced_bool_t                decoder_done;
    wiced_event_flags_t         decoder_events;

    wiced_thread_t              decoder_thread;
    wiced_thread_t*             decoder_thread_ptr;

    /*
     * Audio render information.
     */

    audio_client_render_ref     audio;

    /*
     * Handle for decoder specific information.
     */

    void*                       decoder_handle;

    uint8_t                     client_thread_stack_buffer[AUDIO_CLIENT_THREAD_STACK_SIZE];

} audio_client_t;


/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t audio_client_http_reader_stop(audio_client_t* client);
wiced_result_t audio_client_http_reader_start(audio_client_t* client);

#ifdef __cplusplus
} /*extern "C" */
#endif
