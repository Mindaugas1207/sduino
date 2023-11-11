/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _LINE_SENSOR_HW_H
#define _LINE_SENSOR_HW_H

#include "pico/stdlib.h"
#include "spi_adc_hw.h"
#include "IS31FL3218.h"

#define LINE_SENSOR_HW_OK PICO_OK
#define LINE_SENSOR_HW_ERROR PICO_ERROR_GENERIC

#define LINE_SENSOR_HW_NUM_SENSORS (15U)
#define LINE_SENSOR_HW_OFFSET (1U)
#define LINE_SENSOR_HW_STRB_CH (17U)
#define LINE_SENSOR_HW_STATUS_CH (17U)
#define LINE_SENSOR_HW_LEFT_CH (16U)
#define LINE_SENSOR_HW_RIGHT_CH (0U)

#define LINE_SENSOR_HW_EMITTER_EN_MASK ((((1U << LINE_SENSOR_HW_NUM_SENSORS) - 1U) << LINE_SENSOR_HW_OFFSET) | (1U << LINE_SENSOR_HW_STRB_CH))

#ifdef __cplusplus
extern "C"
{
#endif

static const uint ADC_TO_POS_MAP[LINE_SENSOR_HW_NUM_SENSORS] = {14U, 0U, 13U, 1U, 12U, 2U, 11U, 3U, 10U, 4U, 9U, 5U, 8U, 6U, 7U};

typedef struct
{
    spi_adc_hw_inst_t spi_adc_hw;
    IS31FL3218_inst_t emitter_hw;
    IS31FL3218_inst_t led_hw;
} line_sensor_hw_inst_t;

static int line_sensor_hw_init(line_sensor_hw_inst_t *inst)
{
    if (spi_adc_hw_init(&inst->spi_adc_hw) != SPI_ADC_HW_OK)
        return LINE_SENSOR_HW_ERROR;
    if (IS31FL3218_init(&inst->emitter_hw) != IS31FL3218_OK)
        return LINE_SENSOR_HW_ERROR;
    if (IS31FL3218_init(&inst->led_hw) != IS31FL3218_OK)
        return LINE_SENSOR_HW_ERROR;

    for (int i = 0; i < LINE_SENSOR_HW_NUM_SENSORS; i++)
    {
        IS31FL3218_set_channel_pwm(&inst->emitter_hw, i + LINE_SENSOR_HW_OFFSET, 0xFF);
    }

    IS31FL3218_set_channel_pwm(&inst->emitter_hw, LINE_SENSOR_HW_STRB_CH, 0xFF);
    IS31FL3218_set_channels_enable(&inst->emitter_hw, 0U);
    if (IS31FL3218_write_data(&inst->emitter_hw) != SPI_ADC_HW_OK)
        return LINE_SENSOR_HW_ERROR;

    return LINE_SENSOR_HW_OK;
}

static int line_sensor_hw_enable(line_sensor_hw_inst_t *inst)
{
    if (IS31FL3218_write_channels_enable(&inst->emitter_hw, LINE_SENSOR_HW_EMITTER_EN_MASK) != SPI_ADC_HW_OK)
        return LINE_SENSOR_HW_ERROR;
    if (spi_adc_hw_enable(&inst->spi_adc_hw) != SPI_ADC_HW_OK)
        return LINE_SENSOR_HW_ERROR;

    return LINE_SENSOR_HW_OK;
}

static int line_sensor_hw_disable(line_sensor_hw_inst_t *inst)
{
    if (spi_adc_hw_disable(&inst->spi_adc_hw) != SPI_ADC_HW_OK)
        return LINE_SENSOR_HW_ERROR;
    if (IS31FL3218_write_channels_enable(&inst->emitter_hw, 0U) != SPI_ADC_HW_OK)
        return LINE_SENSOR_HW_ERROR;
    if (IS31FL3218_write_channels_enable(&inst->led_hw, 0U) != SPI_ADC_HW_OK)
        return LINE_SENSOR_HW_ERROR;

    return LINE_SENSOR_HW_OK;
}

static inline int line_sensor_hw_set_emitter_enable(line_sensor_hw_inst_t *inst, bool _Enabled)
{
    return IS31FL3218_write_channels_enable(&inst->emitter_hw, _Enabled ? LINE_SENSOR_HW_EMITTER_EN_MASK : 0U);
}

static inline void line_sensor_hw_set_led_power(line_sensor_hw_inst_t *inst, int ch, float _LedPower)
{
    uint pwm = (uint)(_LedPower * IS31FL3218_GAMMA_64_STEP_MAX);
    IS31FL3218_set_channel_pwm(&inst->led_hw, ch, IS31FL3218_GAMMA_64[pwm > IS31FL3218_GAMMA_64_STEP_MAX ? IS31FL3218_GAMMA_64_STEP_MAX : pwm]);
}

static inline void line_sensor_hw_set_led_enable(line_sensor_hw_inst_t *inst, int ch, bool _Enabled)
{
    IS31FL3218_set_channel_enable(&inst->led_hw, ch, _Enabled);
}

static inline void line_sensor_hw_toggle_led_enable(line_sensor_hw_inst_t *inst, int ch)
{
    IS31FL3218_toggle_channel_enable(&inst->led_hw, ch);
}

static inline int line_sensor_hw_led_update(line_sensor_hw_inst_t *inst)
{
    return IS31FL3218_write_data(&inst->led_hw);
}

static inline int line_sensor_hw_read(line_sensor_hw_inst_t *inst, uint _output[LINE_SENSOR_HW_NUM_SENSORS])
{
    uint16_t buffer[SPI_ADC_BUFFER_SIZE];
    if (spi_adc_hw_read(&inst->spi_adc_hw, buffer) != SPI_ADC_HW_OK)
        return LINE_SENSOR_HW_ERROR;

    for (uint i = 0U; i < LINE_SENSOR_HW_NUM_SENSORS; i++)
        _output[ADC_TO_POS_MAP[i]] = buffer[i * 2U + 1U];

    return LINE_SENSOR_HW_OK;
}

#ifdef __cplusplus
}
#endif

#endif
