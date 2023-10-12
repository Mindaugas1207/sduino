/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _ENCODER_HW_H
#define _ENCODER_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/dma.h"
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
    uint pinF;
    uint pwm;
    uint32_t fsampling;

    int32_t step_count;
    uint32_t pwm_count;
    absolute_time_t timestamp;
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

    float div = (float)clock_get_hz(clk_sys) / inst->fsampling;
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(inst->pio, inst->pio_sm, 0, &c);

    if ((inst->pinB > inst->pinA ? inst->pinB - inst->pinA : inst->pinA - inst->pinB) == 1)
    {
        pio_sm_exec(inst->pio, inst->pio_sm, pio_encode_set(pio_x, 0));
    }
    else
    {
        pio_sm_exec(inst->pio, inst->pio_sm, pio_encode_set(pio_x, 1));
    }

    gpio_set_function(inst->pinF, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv_int_frac(&cfg, 1, 0);
    pwm_config_set_wrap(&cfg, 0xFFFF);
    pwm_set_counter(inst->pwm, 0);
    pwm_init(inst->pwm, &cfg, true);

    pio_sm_set_enabled(inst->pio, inst->pio_sm, true);
    
    return ENCODER_HW_OK;
}

void encoder_read(encoder_hw_inst_t *inst)
{
    uint64_t deltaT = to_us_since_boot(get_absolute_time()) - to_us_since_boot(inst->timestamp);

    uint32_t value = pwm_get_counter(inst->pwm);
    pwm_set_counter(inst->pwm, 0);

    inst->timestamp = get_absolute_time();
        
        
        inst->pwm_count = value;


        //new_value0 = quadrature_encoder_get_count(pio1, 0);
        //new_value1 = quadrature_encoder_get_count(pio1, 1);metic delta will always
    // be correct even when new_value wraps around MAXINT / MININT
    
    //sleep_ms(100);
}

// Get pulse count
uint32_t encoder_get_pulse_count(encoder_hw_inst_t *inst)
{
    uint32_t value = pwm_get_counter(inst->pwm);
    pwm_set_counter(inst->pwm, 0);

    inst->pwm_count = value;

    return value;
}

int32_t encoder_get_step_count(encoder_hw_inst_t *inst)
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

    inst->step_count = ret;

    return ret;
}

#ifdef __cplusplus
}
#endif

#endif