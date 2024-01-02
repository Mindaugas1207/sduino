/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _ENCODER_HW_H
#define _ENCODER_HW_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "encoder.pio.h"

#define ENCODER_HW_OK PICO_OK
#define ENCODER_HW_ERROR PICO_ERROR_GENERIC

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    PIO Pio;
    uint Pio_sm;
    uint PinA;
    uint PinB;
} encoder_hw_inst_t;

static int encoder_hw_init(encoder_hw_inst_t *inst)
{

    pio_sm_config c = encoder_program_get_default_config(0);
    // pio_sm_set_pindirs_with_mask(inst->Pio, inst->Pio_sm, 0, (1u << inst->PinA) | (1u << inst->PinB));
    gpio_set_function(inst->PinA, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    uint pwm = pwm_gpio_to_slice_num(inst->PinA);
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv_int_frac(&cfg, 1, 0);
    pwm_config_set_wrap(&cfg, 0xFFFF);
    pwm_set_counter(pwm, 0);
    pwm_init(pwm, &cfg, false);

    gpio_set_dir(inst->PinA, GPIO_IN);
    gpio_set_dir(inst->PinB, GPIO_IN);
    gpio_pull_up(inst->PinA);
    gpio_pull_up(inst->PinB);

    sm_config_set_in_pins(&c, inst->PinA);
    sm_config_set_jmp_pin(&c, inst->PinA); // for JMP
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

    sm_config_set_clkdiv_int_frac(&c, 1, 0);

    pio_sm_init(inst->Pio, inst->Pio_sm, 0, &c);

    return ENCODER_HW_OK;
}

static int encoder_hw_enable(encoder_hw_inst_t *inst)
{
    pio_sm_set_enabled(inst->Pio, inst->Pio_sm, true);
    pwm_set_enabled(pwm_gpio_to_slice_num(inst->PinA), true);

    return ENCODER_HW_OK;
}

static int encoder_hw_disable(encoder_hw_inst_t *inst)
{
    pio_sm_set_enabled(inst->Pio, inst->Pio_sm, false);
    pwm_set_enabled(pwm_gpio_to_slice_num(inst->PinA), false);

    return ENCODER_HW_OK;
}

// Get pulse count
static inline uint32_t encoder_get_pulse_count(encoder_hw_inst_t *inst)
{
    uint pwm = pwm_gpio_to_slice_num(inst->PinA);
    uint32_t value = pwm_get_counter(pwm);
    pwm_set_counter(pwm, 0);

    return value;
}

static inline int32_t encoder_get_step_count(encoder_hw_inst_t *inst)
{
    int32_t result;
    int n;

    // if the FIFO has N entries, we fetch them to drain the FIFO,
    // plus one entry which will be guaranteed to not be stale
    n = pio_sm_get_rx_fifo_level(inst->Pio, inst->Pio_sm) + 1;
    while (n > 0)
    {
        result = pio_sm_get_blocking(inst->Pio, inst->Pio_sm);
        n--;
    }

    return result;
}

#ifdef __cplusplus
}
#endif

#endif