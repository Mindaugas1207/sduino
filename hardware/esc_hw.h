/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _ESC_HW_H
#define _ESC_HW_H

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define ESC_HW_OK PICO_OK
#define ESC_HW_ERROR PICO_ERROR_GENERIC

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint Pin;
    uint32_t Frequency;
    uint32_t Period;
} esc_hw_inst_t;

static int esc_hw_init(esc_hw_inst_t *inst)
{
    gpio_set_function(inst->Pin, GPIO_FUNC_PWM);
    uint pwm = pwm_gpio_to_slice_num(inst->Pin);
    pwm_set_clkdiv(pwm, (double)clock_get_hz(clk_sys) / inst->Frequency);
    pwm_set_wrap(pwm, inst->Period);
    pwm_set_gpio_level(inst->Pin, 0);
    pwm_set_enabled(pwm_gpio_to_slice_num(inst->Pin), true);

    return ESC_HW_OK;
}

static int esc_hw_enable(esc_hw_inst_t *inst)
{
    pwm_set_enabled(pwm_gpio_to_slice_num(inst->Pin), true);

    return ESC_HW_OK;
}

static int esc_hw_disable(esc_hw_inst_t *inst)
{
    pwm_set_enabled(pwm_gpio_to_slice_num(inst->Pin), false);

    return ESC_HW_OK;
}

static inline int esc_hw_set_pulse(esc_hw_inst_t *inst, uint value)
{
    pwm_set_gpio_level(inst->Pin, value);

    return ESC_HW_OK;
}

#ifdef __cplusplus
}
#endif

#endif