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

/* output_wiced_audio.c - Output module using the WICED audio framework
 * derived from output_gstreamer.c - Output module for GStreamer
 *
 * Copyright (C) 2005-2007   Ivo Clarysse
 *
 * Adapted to gstreamer-0.10 2006 David Siorpaes
 *
 * This file is part of GMediaRender.
 *
 * GMediaRender is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * GMediaRender is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GMediaRender; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wiced_rtos.h"
#include "audio_client.h"

#include "logging.h"
#include "upnp_connmgr.h"
#include "output_module.h"
#include "output_wiced_audio.h"

#define NSECS_PER_SEC_FLOAT                   (1000000000.0)
#define AUDIO_CLIENT_IOCTL_STOP_TIMEOUT_MSECS (10000)
#define UPNP_CONTROL_VOL_MIN_DB               (-60.0)
#define UPNP_CONTROL_VOL_MID_DB               (-20.0)
#define UPNP_CONTROL_VOL_MAX_DB               (+0.0)
#define UPNP_CONTROL_VOL_MIN                  (0)
#define UPNP_CONTROL_VOL_MID                  (50)
#define UPNP_CONTROL_VOL_MAX                  (100)

#define AUDIO_PLAYER_NUM_HTTP_BUFFERS         (200)
#define AUDIO_PLAYER_NUM_AUDIO_BUFFERS        (80)
#define AUDIO_PLAYER_SIZE_AUDIO_BUFFERS       (2048)

struct track_time_info
{
	int64_t duration;
	int64_t position;
};

typedef struct
{
    char                     *uri;
    char                     *next_uri;
    struct SongMetaData      song_meta;
    output_transition_cb_t   play_trans_callback;
    output_update_meta_cb_t  meta_update_callback;
    struct track_time_info   last_known_time;
    audio_client_params_t    audio_client_params;
    audio_client_ref         audio_client_handle;
    AUDIO_CLIENT_EVENT_T     playback_status;
    double                   volume_in_db;
    audio_client_event       caller_event_cbf;
    void                     *caller_userdata;
} wiced_audio_output_ctx_t;

static wiced_audio_output_ctx_t output_ctx =
{
    .uri                  = NULL,
    .next_uri             = NULL,
    .play_trans_callback  = NULL,
    .meta_update_callback = NULL,
    .last_known_time      = {0, 0},
    .audio_client_handle  = NULL,
    .playback_status      = AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED,
    .volume_in_db         = -6.0,
};

static int audio_client_event_callback(audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg)
{
    wiced_result_t            result;
    wiced_audio_output_ctx_t *output_ctx_ptr = (wiced_audio_output_ctx_t *)userdata;

    UNUSED_PARAMETER(handle);

    switch(event)
    {
        case AUDIO_CLIENT_EVENT_ERROR:
            output_ctx_ptr->playback_status = event;
            Log_error("wiced_audio", "audio_client_event_callback() error %d", (int)arg);
            break;

        case AUDIO_CLIENT_EVENT_CONNECTED:
            Log_info("wiced_audio", "audio_client_event_callback() CONNECTED");
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_STARTED:
            output_ctx_ptr->playback_status = event;
            Log_info("wiced_audio", "audio_client_event_callback() STARTED");
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED:
            output_ctx_ptr->playback_status = event;
            Log_info("wiced_audio", "audio_client_event_callback() STOPPED");
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_EOS:
            output_ctx_ptr->playback_status = AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED;
            Log_info("wiced_audio", "audio_client_event_callback() EOS");
            if ( output_ctx.next_uri == NULL )
            {
                if ( output_ctx.play_trans_callback != NULL )
                {
                    output_ctx.play_trans_callback(PLAY_STOPPED);
                }
            }
            else
            {
                free(output_ctx.uri);
                output_ctx.uri = output_ctx.next_uri;
                output_ctx.next_uri = NULL;
                result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_PLAY, output_ctx.uri);
                if ( result != WICED_SUCCESS )
                {
                    Log_error("wiced_audio", "audio_client_ioctl(AUDIO_CLIENT_IOCTL_PLAY) failed with %d !", result);
                }
                else if ( output_ctx.play_trans_callback != NULL )
                {
                    output_ctx.play_trans_callback(PLAY_STARTED_NEXT_STREAM);
                }
            }
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED:
            output_ctx_ptr->playback_status = event;
            Log_info("wiced_audio", "audio_client_event_callback() PAUSED");
            break;

        case AUDIO_CLIENT_EVENT_HTTP_COMPLETE:
            Log_info("wiced_audio", "audio_client_event_callback() COMPLETE");
            break;

        default:
            break;
    }

    if ( output_ctx.caller_event_cbf != NULL )
    {
        output_ctx.caller_event_cbf(NULL, output_ctx.caller_userdata, event, arg);
    }

    return 0;
}

static void output_wiced_audio_set_next_uri(const char *uri)
{
	Log_info("wiced_audio", "Set next uri to '%s'", uri);
	free(output_ctx.next_uri);
	output_ctx.next_uri = ( (uri != NULL) && (uri[0] != '\0') ) ? strdup(uri) : NULL;
}

static void output_wiced_audio_set_uri(const char *uri, output_update_meta_cb_t meta_cb)
{
	Log_info("wiced_audio", "Set uri to '%s'", uri);
	free(output_ctx.uri);
	output_ctx.uri = ( (uri != NULL) && (uri[0] != '\0') ) ? strdup(uri) : NULL;
	output_ctx.meta_update_callback = meta_cb;
	SongMetaData_clear(&output_ctx.song_meta);
}

static int output_wiced_audio_play(output_transition_cb_t callback)
{
    int                  rc     = 0;
    wiced_result_t       result;
    AUDIO_CLIENT_IOCTL_T cmd;
    void                 *arg;

    output_ctx.play_trans_callback = callback;

    if (output_ctx.playback_status == AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED)
    {
        cmd = AUDIO_CLIENT_IOCTL_RESUME;
        arg = NULL;
    }
    else
    {
        if ( output_ctx.playback_status != AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED )
        {
            result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_STOP, NULL);
            if (result != WICED_SUCCESS)
            {
                Log_error("wiced_audio", "audio_client_ioctl(AUDIO_CLIENT_IOCTL_STOP) failed with %d !", result);
            }
        }

        cmd = AUDIO_CLIENT_IOCTL_PLAY;
        arg = output_ctx.uri;
    }

    result = audio_client_ioctl(output_ctx.audio_client_handle, cmd, arg);
    if (result != WICED_SUCCESS)
    {
        Log_error("wiced_audio", "audio_client_ioctl(%s) failed with %d !",
                  ((cmd == AUDIO_CLIENT_IOCTL_PLAY) ? "AUDIO_CLIENT_IOCTL_PLAY" : "AUDIO_CLIENT_IOCTL_PAUSE"),
                  (int)result);
    }

    return rc;
}

static int output_wiced_audio_stop(void)
{
    int            rc     = 0;
    wiced_result_t result;

    result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_STOP, NULL);
    if (result != WICED_SUCCESS)
    {
        Log_error("wiced_audio", "audio_client_ioctl(AUDIO_CLIENT_IOCTL_STOP) failed with %d !", result);
        rc = -1;
    }

    return rc;
}

static int output_wiced_audio_pause(void)
{
    int            rc     = 0;
    wiced_result_t result;

    result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_PAUSE, NULL);
    if (result != WICED_SUCCESS)
    {
        Log_error("wiced_audio", "audio_client_ioctl(AUDIO_CLIENT_IOCTL_PAUSE) failed with %d !", result);
        rc = -1;
    }

    return rc;
}

static int output_wiced_audio_seek(int64_t position_nanos)
{
    Log_error("wiced_audio", "Seek to %lld msecs not supported !", position_nanos/1000000LL);
    return -1;
}

static int output_wiced_audio_get_position(int64_t *track_duration, int64_t *track_pos)
{
    int rc = 0;

	*track_duration = output_ctx.last_known_time.duration;
	*track_pos      = output_ctx.last_known_time.position;

	if ( (output_ctx.playback_status == AUDIO_CLIENT_EVENT_PLAYBACK_STARTED) || (output_ctx.playback_status == AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED) )
	{
	    wiced_result_t            result;
	    audio_client_track_info_t info;

	    result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_TRACK_INFO, &info);
	    if (result != WICED_SUCCESS)
	    {
	        Log_error("wiced_audio", "audio_client_ioctl(AUDIO_CLIENT_IOCTL_TRACK_INFO) failed with %d !", result);
	    }
	    else
	    {
	        if ( info.total_samples != 0 && info.sample_rate != 0 )
	        {
	            *track_duration = (int64_t)((double)((NSECS_PER_SEC_FLOAT / (double)info.sample_rate) * (double)info.total_samples));
	        }
	        else
	        {
	            Log_error("wiced_audio", "Got 0 (duration) !");
	            *track_duration = 0;
	        }

	        if ( info.current_sample != 0 && info.sample_rate != 0 )
	        {
	            *track_pos      = (int64_t)((double)((NSECS_PER_SEC_FLOAT / (double)info.sample_rate) * (double)info.current_sample));
	        }
	        else
	        {
	            Log_error("wiced_audio", "Got 0 (position) !");
	            *track_pos      = 0;
	        }
	    }
	}

	output_ctx.last_known_time.duration = *track_duration;
	output_ctx.last_known_time.position = *track_pos;

	return rc;
}

static int output_wiced_audio_get_volume(float *v)
{
    int rc = 0;

	*v = exp(output_ctx.volume_in_db / 20 * log(10));
	Log_info("wiced_audio", "Query volume fraction: %f", *v);

	return rc;
}

static int output_wiced_audio_set_volume(float value, int level)
{
    int            rc     = 0;
    wiced_result_t result;

    output_ctx.volume_in_db = 20 * log(value) / log(10);
    output_ctx.audio_client_params.volume = level;
    Log_info("wiced_audio", "Set volume fraction/dB to %f/%f/%d", value, output_ctx.volume_in_db, output_ctx.audio_client_params.volume);
    result = audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_SET_VOLUME, (void *)output_ctx.audio_client_params.volume);
    if (result != WICED_SUCCESS)
    {
        Log_error("wiced_audio", "audio_client_ioctl(AUDIO_CLIENT_IOCTL_SET_VOLUME) failed with %d !", result);
    }

    return rc;
}

static int output_wiced_audio_get_mute(int *m)
{
    int rc = 0;

    *m = 0;

	return rc;
}

static int output_wiced_audio_set_mute(int m)
{
    int rc = 0;

	Log_info("wiced_audio", "Set mute to %s", m ? "on" : "off");

	return rc;
}

static int output_wiced_audio_add_options(void *ctx)
{
    memcpy(&output_ctx.audio_client_params, ctx, sizeof(output_ctx.audio_client_params));
    if ( output_ctx.audio_client_params.data_buffer_num == 0 )
    {
        output_ctx.audio_client_params.data_buffer_num = AUDIO_PLAYER_NUM_HTTP_BUFFERS;
    }

    if ( output_ctx.audio_client_params.audio_buffer_num == 0 )
    {
        output_ctx.audio_client_params.audio_buffer_num = AUDIO_PLAYER_NUM_AUDIO_BUFFERS;
    }

    if ( output_ctx.audio_client_params.audio_buffer_size == 0 )
    {
        output_ctx.audio_client_params.audio_buffer_size = AUDIO_PLAYER_SIZE_AUDIO_BUFFERS;
    }

    /* preserve caller original callback function and user data */
    output_ctx.caller_userdata              = output_ctx.audio_client_params.userdata;
    output_ctx.caller_event_cbf             = output_ctx.audio_client_params.event_cb;
    output_ctx.audio_client_params.userdata = &output_ctx;
    output_ctx.audio_client_params.event_cb = audio_client_event_callback;

    return 0;
}

static double volume_level_to_decibel(int volume)
{
    if (volume < UPNP_CONTROL_VOL_MIN)
    {
        volume = UPNP_CONTROL_VOL_MIN_DB;
    }

    if (volume > UPNP_CONTROL_VOL_MAX)
    {
        volume = UPNP_CONTROL_VOL_MAX_DB;
    }

    if (volume < UPNP_CONTROL_VOL_MAX / 2)
    {
        return UPNP_CONTROL_VOL_MIN_DB + (UPNP_CONTROL_VOL_MID_DB - UPNP_CONTROL_VOL_MIN_DB) / UPNP_CONTROL_VOL_MID * volume;
    }
    else
    {
        return UPNP_CONTROL_VOL_MID_DB + ((UPNP_CONTROL_VOL_MAX_DB - UPNP_CONTROL_VOL_MID_DB) / (UPNP_CONTROL_VOL_MAX - UPNP_CONTROL_VOL_MID) * (volume - UPNP_CONTROL_VOL_MID));
    }
}

static int output_wiced_audio_init(void)
{
    int rc = 0;

    SongMetaData_init(&output_ctx.song_meta);

    /* register supported MIME types */
    register_mime_type("audio/flac");
    register_mime_type("audio/x-wav");
    register_mime_type("audio/wav");
    register_mime_type("audio/L8");
    register_mime_type("audio/L16");
    register_mime_type("audio/L20");
    register_mime_type("audio/L24");
    register_mime_type("audio/mp4");
    register_mime_type("audio/m4a");
    register_mime_type("audio/x-m4a");

    output_ctx.audio_client_handle = audio_client_init(&output_ctx.audio_client_params);
    if (output_ctx.audio_client_handle == NULL)
    {
        Log_error("wiced_audio", "audio_client_init() failed !");
        rc = -1;
        goto _exit;
    }

    output_wiced_audio_set_mute(0);
    output_ctx.volume_in_db = volume_level_to_decibel(output_ctx.audio_client_params.volume);
    output_wiced_audio_set_volume(exp(output_ctx.volume_in_db / 20 * log(10)), output_ctx.audio_client_params.volume);

 _exit:
    return rc;
}

static int output_wiced_audio_deinit(void)
{
    int rc = 0;

    audio_client_ioctl(output_ctx.audio_client_handle, AUDIO_CLIENT_IOCTL_STOP, NULL);
    audio_client_deinit(output_ctx.audio_client_handle);
    free(output_ctx.uri);
    free(output_ctx.next_uri);
    memset(&output_ctx, 0, sizeof(output_ctx));

    return rc;
}

struct output_module wiced_audio_output =
{
    .shortname    = "wiced_audio",
	.description  = "WICED audio framework",
	.init         = output_wiced_audio_init,
	.deinit       = output_wiced_audio_deinit,
	.add_options  = output_wiced_audio_add_options,
	.set_uri      = output_wiced_audio_set_uri,
	.set_next_uri = output_wiced_audio_set_next_uri,
	.play         = output_wiced_audio_play,
	.stop         = output_wiced_audio_stop,
	.pause        = output_wiced_audio_pause,
	.seek         = output_wiced_audio_seek,
	.get_position = output_wiced_audio_get_position,
	.get_volume   = output_wiced_audio_get_volume,
	.set_volume   = output_wiced_audio_set_volume,
	.get_mute     = output_wiced_audio_get_mute,
	.set_mute     = output_wiced_audio_set_mute,
};
