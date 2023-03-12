/*
 * port.h
 *
 *  Created on: Oct 15, 2022
 *      Author: minda
 */

#ifndef INCLUDE_LINE_SENSOR_HW_H_
#define INCLUDE_LINE_SENSOR_HW_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "padc.h"
#include "IS31FL3218.h"
#include "port.h"

#define LINE_SENSOR_HW_NUM_SENSORS 15

#define LINE_SENSOR_HW_OFFSET 1
#define LINE_SENSOR_HW_STRB_CH (17 - LINE_SENSOR_HW_OFFSET)
#define LINE_SENSOR_HW_STATUS_CH (17 - LINE_SENSOR_HW_OFFSET)
#define LINE_SENSOR_HW_LEFT_CH (16 - LINE_SENSOR_HW_OFFSET)
#define LINE_SENSOR_HW_RIGHT_CH (0 - LINE_SENSOR_HW_OFFSET)

#define LINE_SENSOR_HW_OK IS31FL3218_OK
#define LINE_SENSOR_HW_ERROR IS31FL3218_ERROR

static const int PADC_TO_POS_MAP[LINE_SENSOR_HW_NUM_SENSORS] = {14, 0, 13, 1, 12, 2, 11, 3, 10, 4, 9, 5, 8, 6, 7};

typedef struct {
    port_i2c_t* port_inst;
	IS31FL3218_inst_t sensor_driver_inst;
	IS31FL3218_inst_t led_driver_inst;
} line_sesnor_hw_inst_t;

static int line_sensor_hw_init(line_sesnor_hw_inst_t *inst)
{
    // PADC init
    if (padc_init(30000000, 30000, PADC_DEFAULT_CLK_PIN, PADC_DEFAULT_DATA_PIN, PADC_DEFAULT_CNV_PIN, PADC_DEFAULT_RESET_PIN) == PADC_ERROR) return LINE_SENSOR_HW_ERROR;

    // IR LED driver init
    if (IS31FL3218_init(&inst->sensor_driver_inst, inst->port_inst, PORT_CHANNEL_0) == IS31FL3218_ERROR) return LINE_SENSOR_HW_ERROR;
    if (IS31FL3218_enable(&inst->sensor_driver_inst) == IS31FL3218_ERROR) return LINE_SENSOR_HW_ERROR;

    // Indicator LED driver init
    if (IS31FL3218_init(&inst->led_driver_inst, inst->port_inst, PORT_CHANNEL_1) == IS31FL3218_ERROR) return LINE_SENSOR_HW_ERROR;
    if (IS31FL3218_enable(&inst->led_driver_inst) == IS31FL3218_ERROR) return LINE_SENSOR_HW_ERROR;

    return LINE_SENSOR_HW_OK;
}

static inline void line_sensor_hw_set_emitter_power(line_sesnor_hw_inst_t *inst, int ch, float _EmitterPower)
{
    uint pwm = (uint)(_EmitterPower * IS31FL3218_PWM_MAX);
    IS31FL3218_set_channel_pwm(&inst->sensor_driver_inst, ch + LINE_SENSOR_HW_OFFSET, pwm > IS31FL3218_PWM_MAX ? IS31FL3218_PWM_MAX : pwm);
}

static inline void line_sensor_hw_set_emitter_power_pwm(line_sesnor_hw_inst_t *inst, int ch, uint pwm)
{
    IS31FL3218_set_channel_pwm(&inst->sensor_driver_inst, ch + LINE_SENSOR_HW_OFFSET, pwm);
}

static inline void line_sensor_hw_set_emitter_enable(line_sesnor_hw_inst_t *inst, int ch, bool _Enabled)
{
    IS31FL3218_set_channel_enable(&inst->sensor_driver_inst, ch + LINE_SENSOR_HW_OFFSET, _Enabled);
}

static inline void line_sensor_hw_emitter_update(line_sesnor_hw_inst_t *inst)
{
    IS31FL3218_write_data(&inst->sensor_driver_inst);
}

static inline void line_sensor_hw_set_led_power(line_sesnor_hw_inst_t *inst, int ch, float _LedPower)
{
    uint pwm = (uint)(_LedPower * IS31FL3218_GAMMA_64_STEP_MAX);
    IS31FL3218_set_channel_pwm(&inst->led_driver_inst, ch + LINE_SENSOR_HW_OFFSET, IS31FL3218_GAMMA_64[pwm > IS31FL3218_GAMMA_64_STEP_MAX ? IS31FL3218_GAMMA_64_STEP_MAX : pwm]);
}

static inline void line_sensor_hw_set_led_power_pwm(line_sesnor_hw_inst_t *inst, int ch, uint pwm)
{
    IS31FL3218_set_channel_pwm(&inst->led_driver_inst, ch + LINE_SENSOR_HW_OFFSET, pwm);
}

static inline void line_sensor_hw_set_led_enable(line_sesnor_hw_inst_t *inst, int ch, bool _Enabled)
{
    IS31FL3218_set_channel_enable(&inst->led_driver_inst, ch + LINE_SENSOR_HW_OFFSET, _Enabled);
}

static inline void line_sensor_hw_toggle_led_enable(line_sesnor_hw_inst_t *inst, int ch)
{
    IS31FL3218_toggle_channel_enable(&inst->led_driver_inst, ch + LINE_SENSOR_HW_OFFSET);
}

static inline void line_sensor_hw_led_update(line_sesnor_hw_inst_t *inst)
{
    IS31FL3218_write_data(&inst->led_driver_inst);
}

static inline void line_sensor_hw_start_read()
{
    padc_start();
}

static inline void line_sensor_hw_stop_read()
{
    padc_stop();
}

static inline bool line_sensor_hw_is_read_ongoing()
{
    return padc_is_read_ongoing();
}

static inline bool line_sensor_hw_is_new_data_available()
{
    return true;//padc_new_data_available();
}

static inline void line_sensor_hw_read(uint _output[LINE_SENSOR_HW_NUM_SENSORS])
{
    uint16_t buffer[PADC_BUFFER_SIZE];
    padc_read_blocking(buffer);

    // printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
    //     buffer[2],  buffer[3],  buffer[6],  buffer[7],  buffer[10], buffer[11],
    //     buffer[14], buffer[15], buffer[18], buffer[19], buffer[22], buffer[23],
    //     buffer[26], buffer[27], buffer[30], buffer[28], buffer[29],
    //     buffer[24], buffer[25], buffer[20], buffer[21], buffer[16], buffer[17],
    //     buffer[12], buffer[13], buffer[8],  buffer[9],  buffer[4],  buffer[5],
    //     buffer[0],  buffer[1]
    // );
    // sleep_ms(100);
    for (int i = 0; i < LINE_SENSOR_HW_NUM_SENSORS; i++)
        _output[PADC_TO_POS_MAP[i]] = buffer[i*2 + 1];
}

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_LINE_SENSOR_HW_H_ */
