/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _ENCODER_HW_H
#define _ENCODER_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "encoder.pio.h"

#define ENCODER_HW_OK PICO_OK
#define ENCODER_HW_ERROR PICO_ERROR_GENERIC

typedef struct {
    PIO  pio;
    uint pio_sm;
    uint pinA;
    uint pinB;
} encoder_hw_inst_t;

static int encoder_hw_init(encoder_hw_inst_t *inst)
{
    pio_sm_config c = encoder_program_get_default_config(0);

    pio_sm_set_pindirs_with_mask(inst->pio, inst->pio_sm, 0, (1u << inst->pinA) | (1u << inst->pinB));
    gpio_pull_up(inst->pinA);
    gpio_pull_up(inst->pinB);

    sm_config_set_in_pins(&c, inst->pinA);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

    sm_config_set_clkdiv_int_frac(&c, 1, 0);

    pio_sm_init(inst->pio, inst->pio_sm, 0, &c);

    pio_sm_set_enabled(inst->pio, inst->pio_sm, true);
    
    return ENCODER_HW_OK;
}

typedef struct {
    uint pin;
    uint pwm;
} freqcnt_hw_inst_t;

static int freqcnt_init(freqcnt_hw_inst_t *inst)
{
    gpio_set_function(inst->pin, GPIO_FUNC_PWM);
    //inst->pwm = pwm_gpio_to_slice_num(inst->pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv_int_frac(&cfg, 1, 0);
    pwm_config_set_wrap(&cfg, 0xFFFF);
    pwm_set_counter(inst->pwm, 0);
    pwm_init(inst->pwm, &cfg, true);

    return ENCODER_HW_OK;
}

// Get pulse count
static uint32_t freqcnt_get_pulse_count(freqcnt_hw_inst_t *inst)
{
    uint32_t value = pwm_get_counter(inst->pwm);
    pwm_set_counter(inst->pwm, 0);

    return value;
}

static int32_t encoder_get_step_count(encoder_hw_inst_t *inst)
{
    int32_t ret;
    int n;

    // if the FIFO has N entries, we fetch them to drain the FIFO,
    // plus one entry which will be guaranteed to not be stale
    n = pio_sm_get_rx_fifo_level(inst->pio, inst->pio_sm) + 1;
    while (n > 0) {
        ret = pio_sm_get_blocking(inst->pio, inst->pio_sm);
        n--;
    }

    return ret;
}

#ifdef __cplusplus
}
#endif

#endif