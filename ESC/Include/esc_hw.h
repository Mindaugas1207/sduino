/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _ESC_HW_H
#define _ESC_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <algorithm>

#define ESC_HW_OK PICO_OK
#define ESC_HW_ERROR PICO_ERROR_GENERIC

typedef struct {
    uint pin;
    uint32_t frequency;
    uint32_t period;
} esc_hw_inst_t;

static int esc_hw_init(esc_hw_inst_t *inst)
{
    gpio_set_function(inst->pin, GPIO_FUNC_PWM);
    uint pwm = pwm_gpio_to_slice_num(inst->pin);
    pwm_set_clkdiv(pwm, clock_get_hz (clk_sys) / inst->frequency);
    pwm_set_wrap(pwm, inst->period);
    pwm_set_gpio_level(inst->pin, 0);
    pwm_set_enabled(pwm, true);

    return ESC_HW_OK;
}

static int esc_hw_set_pulse(esc_hw_inst_t *inst, uint value)
{
    pwm_set_gpio_level(inst->pin, value);
    return ESC_HW_OK;
}

#ifdef __cplusplus
}
#endif

#endif