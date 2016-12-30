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

#include "wiced_result.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "platform_audio.h"
#include "wiced_log.h"

#include "audio_client_render.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define NUM_NSECONDS_IN_SECOND                      (1000000000LL)
#define NUM_USECONDS_IN_SECOND                      (1000000)
#define NUM_NSECONDS_IN_MSECOND                     (1000000)
#define NUM_MSECONDS_IN_SECOND                      (1000)
#define MAX_WICED_AUDIO_INIT_ATTEMPTS               (5)
#define INTERVAL_BETWEEN_WICED_AUDIO_INIT_ATTEMPTS  (10)

#define WICED_AUDIO_PERIOD_SIZE                     (256)
#define WICED_AUDIO_NUM_PERIOD_BUFFERS              (40)

#define AUDIO_RENDER_THREAD_STACK_SIZE              (8 * 1024)
#define AUDIO_RENDER_THREAD_PRIORITY                (3)
#define AUDIO_RENDER_THREAD_NAME                    ("audio render thread")

#define AUDIO_TAG_VALID        0x0C0FFEE0
#define AUDIO_TAG_INVALID      0xDEADBEEF

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_EVENT_SHUTDOWN           = (1 << 0),
    AUDIO_EVENT_PROCESS_BUFFER     = (1 << 1),
    AUDIO_EVENT_FLUSH              = (1 << 2),
    AUDIO_EVENT_STOP               = (1 << 3),
    AUDIO_EVENT_PAUSE              = (1 << 4),
    AUDIO_EVENT_RESUME             = (1 << 5),
} AUDIO_EVENTS_T;

#define AUDIO_ALL_EVENTS       (-1)

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct buf_list_s
{
    struct buf_list_s *next;
    audio_client_render_buf_t buf;
} buf_list_t;

typedef struct audio_client_render_s
{
    uint32_t                            tag;
    wiced_bool_t                        quit;
    wiced_bool_t                        flush_in_progress;
    wiced_bool_t                        err_msg;

    wiced_event_flags_t                 events;
    wiced_mutex_t                       mutex;

    void*                               session_ptr;
    audio_client_render_cb_t            playback_complete_cb;
    wiced_bool_t                        completion_notify;

    platform_audio_device_id_t          device_id;
    wiced_audio_config_t                config;
    wiced_audio_session_ref             sh;
    wiced_bool_t                        audio_running;
    wiced_bool_t                        audio_configured;
    wiced_bool_t                        audio_paused;
    int                                 period_size;
    uint32_t                            bytes_buffered;
    double                              minimum_volume;
    double                              maximum_volume;
    uint8_t                             cur_volume;                 /* 0 - 100 */
    uint32_t                            latency;

    wiced_thread_t                      audio_thread;
    wiced_thread_t*                     audio_thread_ptr;
    uint8_t                             stack[AUDIO_RENDER_THREAD_STACK_SIZE];

    /*
     * Buffer management.
     */

    uint32_t                            num_buffer_nodes;
    uint32_t                            buffer_data_size;
    uint32_t                            num_buffer_used;
    uint32_t                            buffer_bytes_avail;
    audio_client_render_buf_t*          buffer_list;
    audio_client_render_buf_t*          buffer_free_list;
    audio_client_render_buf_t*          buffer_head;
    audio_client_render_buf_t*          buffer_tail;
    uint8_t*                            buffer_data;

    uint64_t                            audio_frames_played;

} audio_client_render_t;


/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static inline void perform_audio_stop(audio_client_render_t* audio)
{
    if (audio->audio_running)
    {
        wiced_audio_stop(audio->sh);
        audio->audio_running  = WICED_FALSE;
        audio->bytes_buffered = 0;
    }
}


static inline void release_head_buffer(audio_client_render_t* audio)
{
    audio_client_render_buf_t* buf_ptr;

    /*
     * Note that this routine assumes that the mutex is locked.
     */

    buf_ptr = audio->buffer_head;
    if (buf_ptr == NULL)
    {
        return;
    }

    /*
     * Remove it from the list.
     */

    audio->buffer_head = audio->buffer_head->next;
    if (audio->buffer_head == NULL)
    {
        audio->buffer_tail        = NULL;
        audio->buffer_bytes_avail = 0;
    }

    /*
     * And put this node back on the free list.
     */

    buf_ptr->next           = audio->buffer_free_list;
    audio->buffer_free_list = buf_ptr;
    audio->num_buffer_used--;
}


static void flush_buffer_list(audio_client_render_t* audio)
{
    wiced_result_t result;
    audio_client_render_buf_t* buf_ptr;
    audio_client_render_buf_t* next;

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to lock audio mutex for flush %d\n", result);
        return;
    }

    /*
     * Traverse the current buffer list and release all queued buffers.
     */

    for (buf_ptr = audio->buffer_head; buf_ptr != NULL; buf_ptr = next)
    {
        next = buf_ptr->next;

        /*
         * And put it back on the free list.
         */

        buf_ptr->next           = audio->buffer_free_list;
        audio->buffer_free_list = buf_ptr;
    }

    audio->buffer_head        = NULL;
    audio->buffer_tail        = NULL;
    audio->num_buffer_used    = 0;
    audio->buffer_bytes_avail = 0;

    wiced_rtos_unlock_mutex(&audio->mutex);
}


static void perform_flush(audio_client_render_t* audio)
{
    audio->flush_in_progress = WICED_TRUE;

    /*
     * Start by stopping the current audio playback.
     */

    perform_audio_stop(audio);

    /*
     * And now flush the buffer list.
     */

    flush_buffer_list(audio);

    audio->flush_in_progress = WICED_FALSE;
}


static void process_buffer(audio_client_render_t* audio)
{
    wiced_result_t result;
    audio_client_render_buf_t* buf_ptr;
    uint32_t wait_time;
    uint32_t buffer_bytes_avail;
    uint32_t events;
    uint8_t* ptr;
    uint16_t size;
    uint16_t audio_buf_size;
    int bytes;

    /*
     * If the audio is paused than just return.
     */

    if (audio->audio_paused)
    {
        return;
    }

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
        return;
    }

    buffer_bytes_avail = audio->buffer_bytes_avail;
    wiced_rtos_unlock_mutex(&audio->mutex);

    while (buffer_bytes_avail >= audio->period_size)
    {
        /* Wait till at least one period is available for writing */
        wait_time = (uint32_t)((double)((NUM_USECONDS_IN_SECOND / (double)audio->config.sample_rate) * (double)audio->period_size) / (double)NUM_MSECONDS_IN_SECOND);
        result = wiced_audio_wait_buffer(audio->sh, audio->period_size, wait_time + 2);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WICED_LOG_WARNING, "No period (%lu)\n", audio->buffer_bytes_avail);

            /* Must do a recovery there */
            perform_audio_stop(audio);
        }

        /*
         * Get the audio buffer.
         */

        audio_buf_size = audio->period_size;
        result = wiced_audio_get_buffer(audio->sh, &ptr, &audio_buf_size);
        if (result != WICED_SUCCESS)
        {
            /* Potential underrun. */
            wiced_log_msg(WICED_LOG_ERR, "No audio buffer chunk available! (%d)\n", buffer_bytes_avail);
            perform_audio_stop(audio);
            break;
        }

        /*
         * We need to fill the buffer now.
         * Grab the mutex.
         */

        result = wiced_rtos_lock_mutex(&audio->mutex);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
            break;
        }

        size = 0;
        while (size < audio_buf_size && audio->buffer_head != NULL)
        {
            buf_ptr = audio->buffer_head;
            bytes   = MIN(audio_buf_size - size, buf_ptr->data_length);
            memcpy(&ptr[size], &buf_ptr->data_buf[buf_ptr->data_offset], bytes);

            size                      += bytes;
            audio->buffer_bytes_avail -= bytes;
            buffer_bytes_avail         = audio->buffer_bytes_avail;
            buf_ptr->data_length      -= bytes;

            audio->audio_frames_played += bytes / audio->config.frame_size;

            /*
             * Are we done with this data buffer?
             */

            if (buf_ptr->data_length == 0)
            {
                release_head_buffer(audio);
            }
            else
            {
                buf_ptr->data_offset += bytes;
            }
        }

        wiced_rtos_unlock_mutex(&audio->mutex);

        /*
         * And let the audio driver have the buffer.
         */

        result = wiced_audio_release_buffer(audio->sh, size);
        if (result == WICED_ERROR)
        {
            wiced_log_msg(WICED_LOG_INFO, "Audio buffer underrun occurred after fill! (%d)\n", result);
            perform_audio_stop(audio);
            break;
        }

        audio->bytes_buffered += size;
        if (audio->audio_running == WICED_FALSE)
        {
            if (audio->bytes_buffered >= audio->period_size * 4)
            {
                /* Only start when there's actual data. */
                wiced_log_msg(WICED_LOG_INFO, "Starting audio\n");
                result = wiced_audio_start(audio->sh);
                if (result == WICED_SUCCESS)
                {
                    audio->audio_running = WICED_TRUE;
                }
                else
                {
                    wiced_log_msg(WICED_LOG_ERR, "Audio did not start\n");
                }
            }
        }

        events = 0;
        result = wiced_rtos_wait_for_event_flags(&audio->events, (AUDIO_ALL_EVENTS & ~AUDIO_EVENT_PROCESS_BUFFER), &events, WICED_FALSE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
        if (result == WICED_SUCCESS && events != 0)
        {
            /*
             * We have a pending event notification. Return to our main event loop.
             */

            break;
        }
    }

    if (audio->buffer_bytes_avail >= audio->period_size)
    {
        /*
         * More audio is available. Make sure that we continue to process it.
         */

        wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_PROCESS_BUFFER);
    }
    else if (audio->completion_notify)
    {
        /*
         * Less than a period of audio left and we've been asked to provide
         * completion notification. Do it now and flush the buffer list in case
         * we have a partial buffer left over.
         */

        if (audio->playback_complete_cb != NULL)
        {
            audio->playback_complete_cb(audio->session_ptr);
        }
        audio->completion_notify = WICED_FALSE;
        flush_buffer_list(audio);
    }
}


static void audio_client_render_thread(uint32_t context)
{
    audio_client_render_t* audio = (audio_client_render_t* )context;
    wiced_result_t result;
    uint32_t events;

    wiced_log_msg(WICED_LOG_INFO, "Audio render thread begin\n");

    while (audio->quit == WICED_FALSE)
    {
        events = 0;
        result = wiced_rtos_wait_for_event_flags(&audio->events, AUDIO_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & AUDIO_EVENT_SHUTDOWN)
        {
            break;
        }

        if (events & AUDIO_EVENT_FLUSH)
        {
            perform_flush(audio);
        }

        if (events & AUDIO_EVENT_STOP)
        {
            perform_audio_stop(audio);
            perform_flush(audio);
            audio->completion_notify = WICED_FALSE;
            audio->audio_paused      = WICED_FALSE;
        }

        if (events & AUDIO_EVENT_PAUSE)
        {
            audio->audio_paused = WICED_TRUE;
        }

        if (events & AUDIO_EVENT_RESUME)
        {
            audio->audio_paused = WICED_FALSE;
            process_buffer(audio);
        }

        if (events & AUDIO_EVENT_PROCESS_BUFFER)
        {
            process_buffer(audio);
        }
    }

    if (audio->sh != NULL)
    {
        wiced_audio_stop(audio->sh);
    }

    wiced_log_msg(WICED_LOG_INFO, "Audio render thread end\n");

    WICED_END_OF_CURRENT_THREAD();
}


static void audio_client_render_shutdown(audio_client_render_ref audio)
{
    if (audio == NULL || audio->tag != AUDIO_TAG_VALID)
    {
        return;
    }

    audio->tag = AUDIO_TAG_INVALID;

    /*
     * Shutdown the main thread.
     */

    if (audio->audio_thread_ptr != NULL)
    {
        audio->quit = WICED_TRUE;

        wiced_rtos_thread_force_awake(&audio->audio_thread);
        wiced_rtos_thread_join(&audio->audio_thread);
        wiced_rtos_delete_thread(&audio->audio_thread);

        audio->audio_thread_ptr = NULL;
    }

    if (audio->sh != NULL)
    {
        wiced_audio_deinit(audio->sh);
    }

    /*
     * Free the buffer list.
     */

    if (audio->buffer_list != NULL)
    {
        /*
         * Release any in-use buffers before freeing the list.
         */

        flush_buffer_list(audio);
        free(audio->buffer_list);
        audio->buffer_list      = NULL;
        audio->buffer_free_list = NULL;
    }

    if (audio->buffer_data != NULL)
    {
        free(audio->buffer_data);
        audio->buffer_data = NULL;
    }

    wiced_rtos_deinit_event_flags(&audio->events);
    wiced_rtos_deinit_mutex(&audio->mutex);

    free(audio);
}


audio_client_render_ref audio_client_render_init(audio_client_render_params_t* params)
{
    audio_client_render_ref audio;
    wiced_result_t result;
    audio_client_render_buf_t* buf_ptr;
    int i;
    uint8_t attempts = 0;

    if (params == NULL)
    {
        return NULL;
    }

    /*
     * Allocate our main structure.
     */

    audio = (audio_client_render_t* )calloc_named("audio_render", 1, sizeof(audio_client_render_t));
    if (audio == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Audio allocation failure\n");
        return NULL;
    }

    audio->session_ptr          = params->session_ptr;
    audio->playback_complete_cb = params->playback_complete_cb;

    /*
     * Allocate and setup the audio buffers.
     */

    audio->num_buffer_nodes = params->buffer_nodes;
    audio->buffer_data_size = params->buffer_size;
    audio->buffer_list      = (audio_client_render_buf_t*)calloc(audio->num_buffer_nodes, sizeof(audio_client_render_buf_t));
    if (audio->buffer_list == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to create audio buffer list\n");
        goto error;
    }

    audio->buffer_data = malloc(audio->num_buffer_nodes * audio->buffer_data_size);
    if (audio->buffer_data == NULL)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to create audio buffer data\n");
        goto error;
    }

    buf_ptr = audio->buffer_list;
    for (i = 0; i < audio->num_buffer_nodes - 1; ++i)
    {
        buf_ptr[i].next     = &buf_ptr[i + 1];
        buf_ptr[i].data_buf = &audio->buffer_data[i * audio->buffer_data_size];
    }
    buf_ptr[i].data_buf = &audio->buffer_data[i * audio->buffer_data_size];

    audio->buffer_free_list = audio->buffer_list;

    /*
     * Initialize our mutex.
     */

    result = wiced_rtos_init_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to create audio mutex %d\n", result);
        goto error;
    }

    /*
     * Create our event flags.
     */

    result = wiced_rtos_init_event_flags(&audio->events);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to create audio event flags %d\n", result);
        goto error;
    }

    audio->quit = WICED_FALSE;

    /*
     * Initialize the audio output device.
     */

    audio->device_id = params->device_id;
    do
    {
        result = wiced_audio_init(audio->device_id, &audio->sh, 0);
    } while ((result != WICED_SUCCESS) && (++attempts < MAX_WICED_AUDIO_INIT_ATTEMPTS ) && (wiced_rtos_delay_milliseconds(INTERVAL_BETWEEN_WICED_AUDIO_INIT_ATTEMPTS) == WICED_SUCCESS));

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to initialize audio device %d (%d)\n", audio->device_id, result);
        goto error;
    }

    /*
     * Create the main render thread.
     */

    result = wiced_rtos_create_thread_with_stack(&audio->audio_thread, AUDIO_RENDER_THREAD_PRIORITY, AUDIO_RENDER_THREAD_NAME,
                                                 audio_client_render_thread, audio->stack, AUDIO_RENDER_THREAD_STACK_SIZE, audio);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to create audio render thread %d\n", result);
        goto error;
    }

    audio->audio_thread_ptr = &audio->audio_thread;
    audio->tag              = AUDIO_TAG_VALID;

    return audio;

error:
    audio_client_render_shutdown(audio);
    return NULL;
}


wiced_result_t audio_client_render_deinit(audio_client_render_ref audio)
{
    if (audio == NULL || audio->tag != AUDIO_TAG_VALID)
    {
        return WICED_BADARG;
    }

    audio_client_render_shutdown(audio);

    return WICED_SUCCESS;
}


wiced_result_t audio_client_render_configure(audio_client_render_ref audio, wiced_audio_config_t* config)
{
    wiced_result_t result;
    uint8_t attempts = 0;

    if (audio == NULL || audio->tag != AUDIO_TAG_VALID || config == NULL)
    {
        return WICED_BADARG;
    }

    if (audio->audio_configured)
    {
        /*
         * Is the configuration the same as what we already have? If so then we're OK.
         */

        if (config->bits_per_sample == audio->config.bits_per_sample && config->channels == audio->config.channels &&
            config->frame_size == audio->config.frame_size && config->sample_rate == audio->config.sample_rate)
        {
            return WICED_SUCCESS;
        }

        /*
         * We need to reinitialize the audio device.
         */

        perform_flush(audio);
        wiced_audio_deinit(audio->sh);
        audio->audio_configured = WICED_FALSE;

        do
        {
            result = wiced_audio_init(audio->device_id, &audio->sh, 0);
        } while ((result != WICED_SUCCESS) && (++attempts < MAX_WICED_AUDIO_INIT_ATTEMPTS ) && (wiced_rtos_delay_milliseconds(INTERVAL_BETWEEN_WICED_AUDIO_INIT_ATTEMPTS) == WICED_SUCCESS));

        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WICED_LOG_ERR, "Unable to reinitialize audio device %d (%d)\n", audio->device_id, result);
            return WICED_ERROR;
        }
    }

    /*
     * Save the configuration.
     */

    audio->config = *config;

    /*
     * Set the period size to use with the audio driver. We want it to be a multiple of the frame size.
     */

    audio->period_size = WICED_AUDIO_PERIOD_SIZE;
    if (audio->period_size % audio->config.frame_size != 0)
    {
        audio->period_size += audio->config.frame_size - (audio->period_size % audio->config.frame_size);
    }

    result = wiced_audio_update_period_size(audio->sh, audio->period_size);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to update audio period size %d\n", result);
        return result;
    }

    result = wiced_audio_create_buffer(audio->sh, audio->period_size * WICED_AUDIO_NUM_PERIOD_BUFFERS, NULL, NULL);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to create audio buffer %d\n", result);
        return result;
    }

    result = wiced_audio_configure(audio->sh, &audio->config);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to configure audio output %d\n", result);
        return result;
    }

    /*
     * Retrieve the volume range for this device and set the current volume.
     */

    wiced_audio_get_volume_range(audio->sh, &audio->minimum_volume, &audio->maximum_volume);
    audio->cur_volume = MIN(audio->config.volume, 100);
    wiced_audio_set_volume(audio->sh, (double)audio->cur_volume * ((audio->maximum_volume - audio->minimum_volume) / 100.0) + audio->minimum_volume);

    audio->audio_configured = WICED_TRUE;

    wiced_log_msg(WICED_LOG_NOTICE, "Audio config is %d channels, %lu kHz, %d bps, frame size %u\r\n",
                  audio->config.channels, audio->config.sample_rate, audio->config.bits_per_sample, audio->config.frame_size);

    return WICED_SUCCESS;
}


wiced_result_t audio_client_render_command(audio_client_render_ref audio, AUDIO_CLIENT_RENDER_CMD_T cmd, void* arg)
{
    wiced_result_t result = WICED_SUCCESS;
    int volume;
    double volume_in_db;
    uint32_t effect_mode;

    if (audio == NULL || audio->tag != AUDIO_TAG_VALID)
    {
        return WICED_BADARG;
    }

    switch (cmd)
    {
        case AUDIO_CLIENT_RENDER_CMD_STOP:
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_STOP);
            break;

        case AUDIO_CLIENT_RENDER_CMD_FLUSH:
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_FLUSH);
            break;

        case AUDIO_CLIENT_RENDER_CMD_PAUSE:
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_PAUSE);
            break;

        case AUDIO_CLIENT_RENDER_CMD_RESUME:
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_RESUME);
            break;

        case AUDIO_CLIENT_RENDER_CMD_VOLUME:
            volume = MIN((int)arg, 100);
            audio->cur_volume = MAX(volume, 0);

            if (audio->sh != NULL)
            {
                volume_in_db = (double)audio->cur_volume * ((audio->maximum_volume - audio->minimum_volume) / 100.0) + audio->minimum_volume;
                return wiced_audio_set_volume(audio->sh, volume_in_db);
            }
            break;

        case AUDIO_CLIENT_RENDER_CMD_SET_EFFECT:
            effect_mode = (uint32_t)arg;

            if (audio->sh != NULL)
            {
                result = wiced_audio_set_effect(audio->sh, effect_mode);
                if (result == WICED_SUCCESS)
                {
                    wiced_log_msg(WICED_LOG_INFO, "Audio effect mode: %d\n", effect_mode);
                }
                else
                {
                    wiced_log_msg(WICED_LOG_ERR, "Unable to set audio effect mode [mode: %d][error: %d]\n", effect_mode, result);
                }
                return result;
            }
            break;

        default:
            return WICED_BADARG;
    }

    return WICED_SUCCESS;
}


wiced_result_t audio_client_render_buffer_get(audio_client_render_ref audio, audio_client_render_buf_t **buf)
{
    wiced_result_t result = WICED_SUCCESS;

    if (audio == NULL || audio->tag != AUDIO_TAG_VALID || buf == NULL)
    {
        return WICED_BADARG;
    }

    /*
     * Grab the mutex.
     */

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
        return WICED_ERROR;
    }

    if (audio->buffer_free_list != NULL)
    {
        /*
         * Grab a node off of the free list.
         */

        *buf = audio->buffer_free_list;
        audio->buffer_free_list = audio->buffer_free_list->next;
        audio->num_buffer_used++;

        (*buf)->next        = NULL;
        (*buf)->buf_length  = audio->buffer_data_size;
        (*buf)->data_offset = 0;
        (*buf)->data_length = 0;

        /*
         * Unlock the mutex.
         */

        wiced_rtos_unlock_mutex(&audio->mutex);

        audio->err_msg = WICED_FALSE;
    }
    else
    {
        if (audio->err_msg == WICED_FALSE)
        {
            wiced_log_msg(WICED_LOG_DEBUG1, "No free buffers in audio list (%lu used out of %lu)  bytes_queued %lu\r\n",
                          audio->num_buffer_used, audio->num_buffer_nodes, audio->buffer_bytes_avail);
            audio->err_msg = WICED_TRUE;
        }
        result = WICED_ERROR;

        /*
         * Don't forget to unlock the mutex before we return.
         */

        wiced_rtos_unlock_mutex(&audio->mutex);
    }

    return result;
}


wiced_result_t audio_client_render_buffer_push(audio_client_render_ref audio, audio_client_render_buf_t *buf)
{
    wiced_result_t result = WICED_SUCCESS;

    if (audio == NULL || audio->tag != AUDIO_TAG_VALID)
    {
        return WICED_BADARG;
    }

    if (buf == NULL)
    {
        /*
         * We're being told that no more data is coming. We want to notify the controlling thread when we're
         * done with what's in the pipeline.
         */

        if (audio->audio_configured == WICED_TRUE)
        {
            audio->completion_notify = WICED_TRUE;
            wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_PROCESS_BUFFER);
        }

        return WICED_SUCCESS;
    }

    /*
     * Grab the mutex.
     */

    result = wiced_rtos_lock_mutex(&audio->mutex);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WICED_LOG_ERR, "Unable to lock audio mutex %d\n", result);
        return WICED_ERROR;
    }

    if (audio->audio_configured != WICED_TRUE)
    {
        wiced_log_msg(WICED_LOG_ERR, "Audio format not configured, buffer discarded\n");
        result = WICED_ERROR;

        /*
         * Put the buffer back on the free list.
         */

        buf->next               = audio->buffer_free_list;
        audio->buffer_free_list = buf;
        audio->num_buffer_used--;
    }
    else
    {
        audio->buffer_bytes_avail += buf->data_length;

        /*
         * Add it to the list of buffers to be processed.
         */

        if (audio->buffer_head == NULL)
        {
            audio->buffer_head = buf;
            audio->buffer_tail = buf;
        }
        else
        {
            audio->buffer_tail->next = buf;
            audio->buffer_tail       = buf;
        }
        buf->next = NULL;

        /*
         * Unlock the mutex before we kick the audio render thread. When the main thread
         * is a higher priority, we'll end up with a priority inversion and extra context
         * switches.
         */

        wiced_rtos_unlock_mutex(&audio->mutex);

        /*
         * Now tell the main loop that there is a buffer to process.
         */

        wiced_rtos_set_event_flags(&audio->events, AUDIO_EVENT_PROCESS_BUFFER);
        audio->err_msg = WICED_FALSE;
    }

    return result;
}
