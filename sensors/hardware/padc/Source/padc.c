
#include "padc.h"

static void padc_handler(void);

padc_inst_t padc_inst;

int padc_init(uint32_t fspi, uint32_t fsampling, uint clk_pin, uint data_pin, uint cnv_pin, uint reset_pin)
{
    padc_inst.fspi = fspi;
    padc_inst.fsampling = fsampling;
    padc_inst.clk_pin = clk_pin;
    padc_inst.data_pin = data_pin;
    padc_inst.cnv_pin = cnv_pin;
    padc_inst.reset_pin = reset_pin;
    padc_inst.Stop = false;
    padc_inst.Enabled = false;

    queue_init(&padc_inst.sample_fifo, sizeof(uint16_t)*PADC_BUFFER_SIZE, PADC_FIFO_SIZE);

    uint prog_offs_sm0 = pio_add_program(PADC_PIO, &padc_sm0_program);
    pio_sm_config c_sm0 = padc_sm0_program_get_default_config(prog_offs_sm0);
    uint prog_offs_sm1 = pio_add_program(PADC_PIO, &padc_sm1_program);
    pio_sm_config c_sm1 = padc_sm1_program_get_default_config(prog_offs_sm1);
    
    float div_sm0 = clock_get_hz (clk_sys) / ((padc_sm0_T_CLKL + padc_sm0_T_CLKH + 2) * fspi);
    float div_sm1 = clock_get_hz (clk_sys) / (2 * fsampling);

    sm_config_set_clkdiv(&c_sm0, div_sm0);
    sm_config_set_clkdiv(&c_sm1, div_sm1);

    sm_config_set_in_pins(&c_sm0, data_pin);
    sm_config_set_sideset_pins(&c_sm0, cnv_pin); // clk_pin = cnv_pin+1
    sm_config_set_in_shift(&c_sm0, false, true, __PADC_DATA_BITS);
    sm_config_set_fifo_join(&c_sm0, PIO_FIFO_JOIN_RX);
    sm_config_set_sideset_pins(&c_sm1, reset_pin);

    pio_gpio_init(PADC_PIO, clk_pin);
    pio_gpio_init(PADC_PIO, data_pin);
    pio_gpio_init(PADC_PIO, cnv_pin);
    pio_gpio_init(PADC_PIO, reset_pin);

    pio_sm_set_pins_with_mask(PADC_PIO, PADC_PIO_SM0, (1u << clk_pin), (1u << clk_pin) | (1u << cnv_pin));
    pio_sm_set_pindirs_with_mask(PADC_PIO, PADC_PIO_SM0, (1u << clk_pin) | (1u << cnv_pin), (1u << clk_pin) | (1u << data_pin) | (1u << cnv_pin));
    pio_sm_set_pins_with_mask(PADC_PIO, PADC_PIO_SM1, (1u << reset_pin), (1u << reset_pin));
    pio_sm_set_pindirs_with_mask(PADC_PIO, PADC_PIO_SM1, (1u << reset_pin), (1u << reset_pin));

    gpio_set_slew_rate(clk_pin, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(clk_pin, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(cnv_pin, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(cnv_pin, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_pulls(data_pin, true, false);

    hw_set_bits(&PADC_PIO->input_sync_bypass, 1u << data_pin);

    pio_sm_init(PADC_PIO, PADC_PIO_SM0, prog_offs_sm0, &c_sm0);
    pio_sm_init(PADC_PIO, PADC_PIO_SM1, prog_offs_sm1, &c_sm1);

    dma_channel_config dma_c = dma_channel_get_default_config(PADC_DMA_CH);
    channel_config_set_read_increment(&dma_c, false);
    channel_config_set_write_increment(&dma_c, true);
    channel_config_set_transfer_data_size(&dma_c, DMA_SIZE_16);
    channel_config_set_dreq(&dma_c, pio_get_dreq(PADC_PIO, PADC_PIO_SM0, false));

    dma_channel_configure(PADC_DMA_CH, &dma_c, padc_inst.dma_buffer, &PADC_PIO->rxf[PADC_PIO_SM0], PADC_BUFFER_SIZE, true);

    irq_set_exclusive_handler(PIO0_IRQ_0, padc_handler);

    pio_set_irq0_source_enabled(PADC_PIO, pis_interrupt0, false);
    irq_set_enabled(PIO0_IRQ_0, false);

    pio_set_sm_mask_enabled(PADC_PIO, (1 << PADC_PIO_SM0) | (1 << PADC_PIO_SM1), true);

    return PADC_OK;
}

static void padc_handler(void)
{
    //gpio_put(SDUINO_INTERNAL_LED_PIN, true);
    queue_try_add(&padc_inst.sample_fifo, padc_inst.dma_buffer);
    dma_channel_set_write_addr(PADC_DMA_CH, padc_inst.dma_buffer, true);
    if (padc_inst.Stop)
    {
        pio_set_irq0_source_enabled(PADC_PIO, pis_interrupt0, false);
        irq_set_enabled(PIO0_IRQ_0, false);
        padc_inst.Enabled = false;
    }
    else
    {
        pio_interrupt_clear(PADC_PIO, 0); //restart sm
        padc_inst.Enabled = true;
    }
        
    //gpio_put(SDUINO_INTERNAL_LED_PIN, false);
}

bool padc_is_read_ongoing()
{
    return dma_channel_is_busy(PADC_DMA_CH) || !pio_interrupt_get(PADC_PIO,0);
}

bool padc_new_data_available()
{
    return !queue_is_empty(&padc_inst.sample_fifo);
}

void padc_start()
{
    // padc_inst.Stop = true; //false
    // dma_channel_set_write_addr(PADC_DMA_CH, padc_inst.dma_buffer, true);
    // pio_set_irq0_source_enabled(PADC_PIO, pis_interrupt0, true);
    // irq_set_enabled(PIO0_IRQ_0, true);
}

void padc_stop()
{
    // padc_inst.Stop = true;
    // while (padc_inst.Enabled) tight_loop_contents();
    // uint16_t buffer[PADC_BUFFER_SIZE];
    // while (queue_try_remove(&padc_inst.sample_fifo, buffer)) tight_loop_contents();
}

bool padc_read(uint16_t buffer[PADC_BUFFER_SIZE])
{
    return queue_try_remove(&padc_inst.sample_fifo, buffer);
}

void padc_read_blocking(uint16_t buffer[PADC_BUFFER_SIZE])
{
    dma_channel_set_write_addr(PADC_DMA_CH, buffer, true);
    pio_interrupt_clear(PADC_PIO, 0); //start sm
    while (padc_is_read_ongoing()) tight_loop_contents();
}
