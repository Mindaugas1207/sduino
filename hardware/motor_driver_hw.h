/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _MOTOR_DRIVER_HW_H
#define _MOTOR_DRIVER_HW_H

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_DRIVER_HW_OK PICO_OK
#define MOTOR_DRIVER_HW_ERROR PICO_ERROR_GENERIC

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint PinA;
    uint PinB;
    uint32_t Frequency;
    uint32_t Period;
} motor_driver_hw_inst_t;

static int motor_driver_hw_init(motor_driver_hw_inst_t *inst)
{
    gpio_set_function(inst->PinA, GPIO_FUNC_PWM);
    gpio_set_function(inst->PinB, GPIO_FUNC_PWM);
    uint pwmA = pwm_gpio_to_slice_num(inst->PinA);
    uint pwmB = pwm_gpio_to_slice_num(inst->PinB);
    pwm_set_clkdiv(pwmA, (double)clock_get_hz (clk_sys) / inst->Frequency);
    pwm_set_clkdiv(pwmB, (double)clock_get_hz (clk_sys) / inst->Frequency);
    pwm_set_wrap(pwmA, inst->Period);
    pwm_set_wrap(pwmB, inst->Period);
    pwm_set_gpio_level(inst->PinA, 0);
    pwm_set_gpio_level(inst->PinB, 0);
    pwm_set_enabled(pwmA, true);
    pwm_set_enabled(pwmB, true);

    return MOTOR_DRIVER_HW_OK;
}

static inline int motor_driver_hw_set_pwm(motor_driver_hw_inst_t *inst, uint pwmA, uint pwmB)
{
    pwm_set_gpio_level(inst->PinA, pwmA);
    pwm_set_gpio_level(inst->PinB, pwmB);

    return MOTOR_DRIVER_HW_OK;
}

#ifdef __cplusplus
}
#endif

#endif