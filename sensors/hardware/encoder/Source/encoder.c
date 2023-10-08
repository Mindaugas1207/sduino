
#include "encoder.h"

void encoder_init(PIO pio, uint sm, uint pinA, uint pinB, uint32_t fsampling)
{
    pio_sm_config c = encoder_program_get_default_config(0);

    pio_sm_set_pindirs_with_mask(pio, sm, 0, (1u << pinA) | (1u << pinB));
    gpio_pull_up(pinA);
    gpio_pull_up(pinB);

    sm_config_set_in_pins(&c, pinA);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

    float div = (float)clock_get_hz(clk_sys) / fsampling;
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, 0, &c);

    if ((pinB > pinA ? pinB - pinA : pinA - pinB) == 1)
    {
        pio_sm_exec(pio, sm, pio_encode_set(pio_x, 0));
    }
    else
    {
        pio_sm_exec(pio, sm, pio_encode_set(pio_x, 1));
    }
    

    pio_sm_set_enabled(pio, sm, true);
}

int32_t penc_get_count(PIO pio, uint sm)
{
    uint ret;
    int n;

    // if the FIFO has N entries, we fetch them to drain the FIFO,
    // plus one entry which will be guaranteed to not be stale
    n = pio_sm_get_rx_fifo_level(pio, sm) + 1;
    while (n > 0) {
        ret = pio_sm_get_blocking(pio, sm);
        n--;
    }
    return ret;
}
