/*
* Copyright 2015, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

#pragma once

#include "wiced_rtos.h" /* for wiced_mutex_t */
#include "wiced_audio.h"
#include "wiced_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define ak4961_ext_slave                        { ak4961_sext, 0, 0 }
#define ak4961_pll_slave                        { ak4961_spll, 1, 0 }

#define AK4961_SOURCE_PORT_SDTI1A               (&ak4961_source_port_sdti1a)
#define AK4961_SOURCE_PORT_SDTI1B               (&ak4961_source_port_sdti1b)
#define AK4961_SOURCE_PORT_SDTI1C               (&ak4961_source_port_sdti1c)
#define AK4961_SOURCE_PORT_SDTI1D               (&ak4961_source_port_sdti1d)
#define AK4961_SOURCE_PORT_SDTI2                (&ak4961_source_port_sdti2)
#define AK4961_SOURCE_PORT_SDTI3                (&ak4961_source_port_sdti3)
#define AK4961_SOURCE_PORT_SDTI4                (&ak4961_source_port_sdti4)
#define AK4961_SOURCE_PORT_ADC1                 (&ak4961_source_port_adc1)
#define AK4961_SOURCE_PORT_ADC2                 (&ak4961_source_port_adc2)

#define AK4961_SINK_PORT_SDTO1A                 (&ak4961_sink_port_sdto1a)
#define AK4961_SINK_PORT_SDTO1B                 (&ak4961_sink_port_sdto1b)
#define AK4961_SINK_PORT_SDTO2                  (&ak4961_sink_port_sdto2)
#define AK4961_SINK_PORT_SDTO3                  (&ak4961_sink_port_sdto3)
#define AK4961_SINK_PORT_SDTO4                  (&ak4961_sink_port_sdto4)
#define AK4961_SINK_PORT_DAC1                   (&ak4961_sink_port_dac1)
#define AK4961_SINK_PORT_DAC2                   (&ak4961_sink_port_dac2)

#define AK4961_DMIC_CHANNEL_ENABLE_LEFT         (1U << 4)
#define AK4961_DMIC_CHANNEL_ENABLE_RIGHT        (1U << 5)

#define AK4961_DAC_HP_AMP_GAIN_0DB_DEFAULT      (0x15)
#define AK4961_DAC_LOUT_AMP_GAIN_0DB_DEFAULT    (0x05)
#define AK4961_ADC_MIC_AMP_GAIN_0DB_DEFAULT     (0x00)
#define AK4961_DAC_DIGITAL_VOLUME_0DB_DEFAULT   (0x19)

#define AK4961_MIC_POWER_OUTPUT_VOLTAGE_DEFAULT (AK4961_MIC_POWER_OUTPUT_VOLTAGE_2V8)

#define AK4961_DMIC_POLARITY_DEFAULT            (AK4961_DMIC_POLARITY_LCH_LO_RCH_HI)

/* Helpers to initialize ak4961_device_route structure. */
#define AK4961_DAC1_HP_INITIALIZER(_INTF_PTR_) \
    { (_INTF_PTR_), ak4961_dac_route, &ak4961_dac1_hp_route }
#define AK4961_DAC2_LOUT2_INITIALIZER(_INTF_PTR_) \
    { (_INTF_PTR_), ak4961_dac_route, &ak4961_dac2_lout2_route }
#define AK4961_DAC2_LOUT2_DIFF_INITIALIZER(_INTF_PTR_) \
    { (_INTF_PTR_), ak4961_dac_route, &ak4961_dac2_lout2_diff_route }
#define AK4961_ADC1_MIC_INITIALIZER(_INTF_PTR_) \
    { (_INTF_PTR_), ak4961_adc_mic_route, &ak4961_adc1_mic_route }
#define AK4961_ADC1_DMIC1_INITIALIZER(_INTF_PTR_) \
    { (_INTF_PTR_), ak4961_adc_dmic_route, &ak4961_adc1_dmic1_route }
#define AK4961_ADC2_DMIC2_INITIALIZER(_INTF_PTR_) \
    { (_INTF_PTR_), ak4961_adc_dmic_route, &ak4961_adc2_dmic2_route }

#define AK4961_DAC_AUDIO_DEVICE_INTERFACE_INITIALIZER( ... )            \
{                                                                       \
    .audio_device_init              = ak4961_init,                      \
    .audio_device_deinit            = ak4961_deinit,                    \
    .audio_device_configure         = ak4961_configure,                 \
    .audio_device_start_streaming   = ak4961_start_play,                \
    .audio_device_stop_streaming    = ak4961_stop_play,                 \
    .audio_device_set_volume        = ak4961_set_playback_volume,       \
    .audio_device_get_volume_range  = ak4961_get_playback_volume_range, \
    ## __VA_ARGS__                                                      \
}

#define AK4961_ADC_AUDIO_DEVICE_INTERFACE_INITIALIZER( ... )            \
{                                                                       \
    .audio_device_init              = ak4961_init,                      \
    .audio_device_deinit            = ak4961_deinit,                    \
    .audio_device_configure         = ak4961_configure,                 \
    .audio_device_start_streaming   = ak4961_start_play,                \
    .audio_device_stop_streaming    = ak4961_stop_play,                 \
    .audio_device_set_volume        = ak4961_set_capture_volume,        \
    .audio_device_get_volume_range  = ak4961_get_capture_volume_range,  \
    ## __VA_ARGS__                                                      \
}

/******************************************************
 *            Constants
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/* Enumerations. */
typedef enum ak4961_device_id ak4961_device_id_t;
typedef enum ak4961_adc_input_select ak4961_adc_input_select_t;
typedef enum ak4961_dac_output_select ak4961_dac_output_select_t;
typedef enum ak4961_sync_domain_select ak4961_sync_domain_select_t;
typedef enum ak4961_mic_power_supply ak4961_mic_power_supply_t;
typedef enum ak4961_route_id ak4961_route_id_t;
typedef enum ak4961_mic_power_output_voltage ak4961_mic_power_output_voltage_t;
typedef enum ak4961_dmic_polarity ak4961_dmic_polarity_t;


/* Structures. */
typedef struct ak4961_clock_settings ak4961_clock_settings_t;
typedef struct ak4961_device_cmn_data ak4961_device_cmn_data_t;
typedef struct ak4961_device_route ak4961_device_route_t;
typedef struct ak4961_device_data ak4961_device_data_t;
typedef struct ak4961_adc_analog_input ak4961_adc_analog_input_t;
typedef struct ak4961_route_data ak4961_route_data_t;
typedef struct ak4961_dac_route_data ak4961_dac_route_data_t;
typedef struct ak4961_adc_route_data ak4961_adc_route_data_t;
typedef struct ak4961_source_port ak4961_source_port_t;
typedef struct ak4961_sink_port ak4961_sink_port_t;
typedef struct ak4961_device_runtime_data ak4961_device_runtime_data_t;
typedef struct ak4961_adc_analog ak4961_adc_analog_t;
typedef struct ak4961_adc_digital ak4961_adc_digital_t;
typedef struct ak4961_dac_route ak4961_dac_route_t;
typedef struct ak4961_adc_route ak4961_adc_route_t;

/* Unions. */
typedef union ak4961_adc_type ak4961_adc_type_t;

/* Functions. */
typedef wiced_result_t (ak4961_rfn_t)(ak4961_device_data_t *, wiced_bool_t);
typedef wiced_result_t (ak4961_ckcfg_fn_t)(ak4961_device_data_t *dd, wiced_audio_config_t *config, uint32_t mclk);


/******************************************************
 *            Enumerations
 ******************************************************/

enum ak4961_route_id
{
    AK4961_ROUTE_ID_0                       = 0,
    AK4961_ROUTE_ID_1                       = 1,
    AK4961_ROUTE_ID_2                       = 2,
    AK4961_ROUTE_ID_3                       = 3,
    AK4961_ROUTE_ID_4                       = 4,
    AK4961_ROUTE_ID_5                       = 5,
    AK4961_ROUTE_ID_6                       = 6,
    AK4961_ROUTE_ID_7                       = 7,

    /* Not a route id! */
    AK4961_ROUTE_ID_MAX,
};

enum ak4961_sync_domain_select
{
    AK4961_SYNC_DOMAIN_SELECT_NA            = 0,
    AK4961_SYNC_DOMAIN_SELECT_1             = 1,
    AK4961_SYNC_DOMAIN_SELECT_2             = 2,
    AK4961_SYNC_DOMAIN_SELECT_3             = 3,
    AK4961_SYNC_DOMAIN_SELECT_4             = 4,
    AK4961_SYNC_DOMAIN_SELECT_5             = 5,
    AK4961_SYNC_DOMAIN_SELECT_6             = 6,
    AK4961_SYNC_DOMAIN_SELECT_7             = 7,
};

enum ak4961_adc_input_select
{
    AK4961_ADC_INPUT_SELECT_AIN1            = 0,
    AK4961_ADC_INPUT_SELECT_AIN2            = 1,
    AK4961_ADC_INPUT_SELECT_AIN3            = 2,
    AK4961_ADC_INPUT_SELECT_AIN4            = 3,
    AK4961_ADC_INPUT_SELECT_AIN5            = 4,
    AK4961_ADC_INPUT_SELECT_AIN6            = 5,
    AK4961_ADC_INPUT_SELECT_MUTE            = 6,
};

enum ak4961_mic_power_supply
{
    AK4961_MIC_POWER_SUPPLY_MPWR1A          = 0,
    AK4961_MIC_POWER_SUPPLY_MPWR1B          = 1,
    AK4961_MIC_POWER_SUPPLY_MPWR1C          = 2,
    AK4961_MIC_POWER_SUPPLY_MPWR2           = 3,
    /* Not a value; used to specify a single-ended MIC. */
    AK4961_MIC_POWER_SUPPLY_MAX,
};

enum ak4961_mic_power_output_voltage
{
    AK4961_MIC_POWER_OUTPUT_VOLTAGE_2V8     = 0,
    AK4961_MIC_POWER_OUTPUT_VOLTAGE_2V5     = 1,
    AK4961_MIC_POWER_OUTPUT_VOLTAGE_1V8     = 2,
    AK4961_MIC_POWER_OUTPUT_VOLTAGE_AVDD1   = 3,
    /* Not a value. */
    AK49961_MIC_POWER_OUTPUT_VOLTAGE_MAX,
};

enum ak4961_dac_output_select
{
    AK4961_DAC_OUTPUT_SELECT_MUTE           = 0,
    AK4961_DAC_OUTPUT_SELECT_LCH            = 1,
    AK4961_DAC_OUTPUT_SELECT_RCH            = 2,
    AK4961_DAC_OUTPUT_SELECT_LCH_RCH        = 3,
    AK4961_DAC_OUTPUT_SELECT_LCH_DIV_2      = 5,
    AK4961_DAC_OUTPUT_SELECT_RCH_DIV_2      = 6,
    AK4961_DAC_OUTPUT_SELECT_LCH_RCH_DIV_2  = 7,
};

enum ak4961_dmic_polarity
{
    AK4961_DMIC_POLARITY_LCH_LO_RCH_HI      = 0,
    AK4961_DMIC_POLARITY_RCH_LO_LCH_HI      = 1,
    /* Not a value. */
    AK4961_DMIC_POLARITY_MAX,
};

/******************************************************
 *             Structures
 ******************************************************/

/* Common structure shared amongst routing data types. */
struct ak4961_route_data
{
    /* AK4961_ROUTE_ID_XXX */
    ak4961_route_id_t               id;

    const ak4961_device_route_t     *device_route;
};

struct ak4961_adc_analog_input
{
    /* AK4961_ADC_INPUT_AIN1-6 */
    ak4961_adc_input_select_t       adc_input_select;

    /* For AK4961_ADC_INPUT_AIN1-6, use AK4961_MIC_POWER_SUPPLY_XXX. */
    ak4961_mic_power_supply_t       power_supply;

    /*  Table 21.
     *  Applicable when !AK4961_MIC_POWER_SUPPLY_MAX.
    */
    ak4961_mic_power_output_voltage_t   output_voltage;

    /* Table 18. */
    uint8_t                         amp_gain;
};

struct ak4961_dac_route_data
{
    /* Must be first. */
    ak4961_route_data_t             base;

    /* AK4961_SOURCE_PORT_XXX */
    const ak4961_source_port_t      *source_port;

    /* AK4961_SYNC_DOMAIN_SELECT_XXX */
    ak4961_sync_domain_select_t     sync_domain_select;

    /* AK4961_DAC_OUTPUT_SELECT_XXX */
    ak4961_dac_output_select_t      output_left_select;
    ak4961_dac_output_select_t      output_right_select;

    /* Table 36/40/44. */
    uint8_t                         amp_gain;

    /* Digital Volume: Table 32. */
    uint8_t                         digital_volume;
};

struct ak4961_adc_analog
{
    const ak4961_adc_analog_input_t *input_left;
    const ak4961_adc_analog_input_t *input_right;
};

struct ak4961_adc_digital
{
    uint8_t                         lch_enabled : 1;
    uint8_t                         rch_enabled : 1;
    ak4961_dmic_polarity_t          polarity;
};

union ak4961_adc_type
{
    ak4961_adc_digital_t            digital;
    ak4961_adc_analog_t             analog;
};

struct ak4961_adc_route_data
{
    /* Must be first. */
    ak4961_route_data_t             base;

    /* AK4961_ROUTE_ID_XXX */
    const ak4961_route_id_t         id;

    /* AK4961_SINK_PORT_XXX */
    const ak4961_sink_port_t        *sink_port;

    /* AK4961_SYNC_DOMAIN_SELECT_XXX */
    ak4961_sync_domain_select_t     sync_domain_select;

    const ak4961_adc_type_t         type;
};

struct ak4961_clock_settings
{
    ak4961_ckcfg_fn_t               *fn;
    uint8_t                         pll_enab        : 1;
    uint8_t                         is_frame_master : 1;
};

struct ak4961_device_cmn_data
{
    wiced_i2c_device_t *            i2c_data;
    ak4961_clock_settings_t         ck;
    wiced_gpio_t                    pdn;
    wiced_gpio_t                    switcher_3v3_ps_enable;
    wiced_gpio_t                    switcher_2v_enable;
    wiced_gpio_t                    ldo_1v8_enable;
    ak4961_device_runtime_data_t    *rtd;
};

struct ak4961_device_data
{
    /* ak4961_XXX_route_data_t */
    const void                      *route;

    ak4961_device_cmn_data_t        *cmn;

    wiced_i2s_t                     data_port;
};

/* Codec device runtime information (private).
 * Declare one per device.
*/
struct ak4961_device_runtime_data
{
    wiced_mutex_t                   lock;

    /* This structure is initialized and ready for use. */
    uint8_t                         rdy   : 1;

    /* Route in-use flags. */
    uint8_t                         init  : AK4961_ROUTE_ID_MAX;
    uint8_t                         cfg   : AK4961_ROUTE_ID_MAX;

    /* Power initialization flags. */
    uint8_t                         pm3_1 : AK4961_ROUTE_ID_MAX;
    uint8_t                         pm3_3 : AK4961_ROUTE_ID_MAX;

    /* Shared configuration (duplex). */
    uint8_t                         bits_per_sample;
    uint8_t                         channels;
    uint32_t                        sample_rate;
};

/* Initialize with helpers. */
struct ak4961_device_route
{
    wiced_audio_device_interface_t  *adi;
    ak4961_rfn_t                    *fn;
    /* ak4961_XXX_route_t */
    const void                      *instance;
};


/******************************************************
 *             Variable declarations
 ******************************************************/

extern const ak4961_source_port_t ak4961_source_port_sdti1a;
extern const ak4961_source_port_t ak4961_source_port_sdti1b;
extern const ak4961_source_port_t ak4961_source_port_sdti1c;
extern const ak4961_source_port_t ak4961_source_port_sdti1d;
extern const ak4961_source_port_t ak4961_source_port_sdti2;
extern const ak4961_source_port_t ak4961_source_port_sdti3;
extern const ak4961_source_port_t ak4961_source_port_sdti4;

extern const ak4961_sink_port_t ak4961_sink_port_sdto1a;
extern const ak4961_sink_port_t ak4961_sink_port_sdto1b;
extern const ak4961_sink_port_t ak4961_sink_port_sdto2;
extern const ak4961_sink_port_t ak4961_sink_port_sdto3;
extern const ak4961_sink_port_t ak4961_sink_port_sdto4;
extern const ak4961_sink_port_t ak4961_sink_port_dac1;
extern const ak4961_sink_port_t ak4961_sink_port_dac2;

extern const ak4961_dac_route_t ak4961_dac1_hp_route;
extern const ak4961_dac_route_t ak4961_dac2_lout2_route;
extern const ak4961_dac_route_t ak4961_dac2_lout2_diff_route;
extern const ak4961_adc_route_t ak4961_adc1_mic_route;
extern const ak4961_adc_route_t ak4961_adc1_dmic1_route;
extern const ak4961_adc_route_t ak4961_adc2_dmic2_route;

/******************************************************
 *             Function declarations
 ******************************************************/

wiced_result_t ak4961_platform_configure( ak4961_device_data_t *device_data, uint32_t mclk, uint32_t fs, uint8_t width );

wiced_result_t ak4961_device_register( ak4961_device_data_t *device_data, const char *name );

/* Don't use these functions directly; use macros provided. */
ak4961_ckcfg_fn_t ak4961_sext;
ak4961_ckcfg_fn_t ak4961_spll;
wiced_result_t ak4961_dac_route(ak4961_device_data_t *, wiced_bool_t );
wiced_result_t ak4961_adc_mic_route(ak4961_device_data_t *, wiced_bool_t );
wiced_result_t ak4961_adc_dmic_route(ak4961_device_data_t *, wiced_bool_t );

wiced_result_t ak4961_init( void* device_data );
wiced_result_t ak4961_deinit( void* device_data );
wiced_result_t ak4961_configure( void* device_data, wiced_audio_config_t* config, uint32_t* mclk );
wiced_result_t ak4961_start_play( void* device_data );
wiced_result_t ak4961_stop_play( void* device_data );
wiced_result_t ak4961_set_playback_volume( void* device_data, double decibles );
wiced_result_t ak4961_get_playback_volume_range( void* device_data, double* min_volume_decibels, double* max_volume_decibels);
wiced_result_t ak4961_set_capture_volume( void* device_data, double decibles );
wiced_result_t ak4961_get_capture_volume_range( void* device_data, double* min_volume_decibels, double* max_volume_decibels);

#ifdef __cplusplus
} /* extern "C" */
#endif