/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _SPI_ADC_HW_H
#define _SPI_ADC_HW_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "spi_adc.pio.h"
#include "string.h"

#define SPI_ADC_HW_OK PICO_OK
#define SPI_ADC_HW_ERROR PICO_ERROR_GENERIC

#define SPI_ADC_BUFFER_SIZE __SPI_ADC_CNV_COUNT + 1

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    PIO Pio;
    uint Pio_sm_0;
    uint Pio_sm_1;
    uint DMA_ch;
    uint Pin_CLK;
    uint Pin_DATA;
    uint Pin_CNV;
    uint Pin_RESET;
    uint32_t fspi;
    uint32_t fcnv;
    uint16_t DMA_buffer[SPI_ADC_BUFFER_SIZE];
} spi_adc_hw_inst_t;

static int spi_adc_hw_init(spi_adc_hw_inst_t *inst)
{
    // queue_init(&padc_inst.sample_fifo, sizeof(uint16_t)*PADC_BUFFER_SIZE, PADC_FIFO_SIZE);

    uint prog_offs_sm0 = pio_add_program(inst->Pio, &spi_adc_sm0_program);
    pio_sm_config c_sm0 = spi_adc_sm0_program_get_default_config(prog_offs_sm0);
    uint prog_offs_sm1 = pio_add_program(inst->Pio, &spi_adc_sm1_program);
    pio_sm_config c_sm1 = spi_adc_sm1_program_get_default_config(prog_offs_sm1);

    sm_config_set_clkdiv(&c_sm0, (double)clock_get_hz(clk_sys) / (2 * inst->fspi));
    sm_config_set_clkdiv(&c_sm1, (double)clock_get_hz(clk_sys) / (2 * inst->fcnv));

    

    sm_config_set_in_pins(&c_sm0, inst->Pin_DATA);
    sm_config_set_set_pins(&c_sm0, inst->Pin_CNV, 1);
    sm_config_set_sideset_pins(&c_sm0, inst->Pin_CLK);
    sm_config_set_in_shift(&c_sm0, false, true, __SPI_ADC_BIT_COUNT);
    sm_config_set_fifo_join(&c_sm0, PIO_FIFO_JOIN_RX);
    sm_config_set_set_pins(&c_sm1, inst->Pin_RESET, 1);
    
    pio_gpio_init(inst->Pio, inst->Pin_CLK);
    pio_gpio_init(inst->Pio, inst->Pin_DATA);
    pio_gpio_init(inst->Pio, inst->Pin_CNV);
    pio_gpio_init(inst->Pio, inst->Pin_RESET);

    gpio_set_dir(inst->Pin_CLK, GPIO_OUT);
    gpio_set_dir(inst->Pin_DATA, GPIO_IN);
    gpio_set_dir(inst->Pin_CNV, GPIO_OUT);
    gpio_set_dir(inst->Pin_RESET, GPIO_OUT);

    // gpio_put(inst->Pin_CLK, true);
    // gpio_put(inst->Pin_CNV, false);
    // gpio_put(inst->Pin_RESET, true);

    pio_sm_set_pins_with_mask(inst->Pio, inst->Pio_sm_0, (1u << inst->Pin_CLK), (1u << inst->Pin_CLK) | (1u << inst->Pin_CNV));
    pio_sm_set_pindirs_with_mask(inst->Pio, inst->Pio_sm_0, (1u << inst->Pin_CLK) | (1u << inst->Pin_CNV), (1u << inst->Pin_CLK) | (1u << inst->Pin_DATA) | (1u << inst->Pin_CNV));
    pio_sm_set_pins_with_mask(inst->Pio, inst->Pio_sm_1, (1u << inst->Pin_RESET), (1u << inst->Pin_RESET));
    pio_sm_set_pindirs_with_mask(inst->Pio, inst->Pio_sm_1, (1u << inst->Pin_RESET), (1u << inst->Pin_RESET));

    gpio_set_slew_rate(inst->Pin_CLK, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(inst->Pin_CLK, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(inst->Pin_CNV, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(inst->Pin_CNV, GPIO_DRIVE_STRENGTH_12MA);
    gpio_pull_up(inst->Pin_DATA);
    hw_set_bits(&inst->Pio->input_sync_bypass, 1u << inst->Pin_DATA);

    pio_sm_init(inst->Pio, inst->Pio_sm_0, prog_offs_sm0, &c_sm0);
    pio_sm_init(inst->Pio, inst->Pio_sm_1, prog_offs_sm1, &c_sm1);

    dma_channel_config dma_c = dma_channel_get_default_config(inst->DMA_ch);
    channel_config_set_read_increment(&dma_c, false);
    channel_config_set_write_increment(&dma_c, true);
    channel_config_set_transfer_data_size(&dma_c, DMA_SIZE_16);
    channel_config_set_dreq(&dma_c, pio_get_dreq(inst->Pio, inst->Pio_sm_0, false));

    dma_channel_configure(inst->DMA_ch, &dma_c, inst->DMA_buffer, &inst->Pio->rxf[inst->Pio_sm_0], SPI_ADC_BUFFER_SIZE, true);

    // irq_set_exclusive_handler(inst->Pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0, padc_handler);

    // pio_set_irq0_source_mask_enabled(inst->Pio, 1u << ((int)pis_interrupt0 + __SPI_ADC_FLAG_IRQ), true);
    // irq_set_enabled(inst->Pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0, true);

    return SPI_ADC_HW_OK;
}

static int spi_adc_hw_enable(spi_adc_hw_inst_t *inst)
{
    pio_set_sm_mask_enabled(inst->Pio, (1 << inst->Pio_sm_0) | (1 << inst->Pio_sm_1), false);
    pio_restart_sm_mask(inst->Pio, (1 << inst->Pio_sm_0) | (1 << inst->Pio_sm_1));
    pio_sm_drain_tx_fifo(inst->Pio, inst->Pio_sm_1);
    dma_channel_set_write_addr(inst->DMA_ch, inst->DMA_buffer, true);
    pio_set_sm_mask_enabled(inst->Pio, (1 << inst->Pio_sm_0) | (1 << inst->Pio_sm_1), true);

    while (!pio_interrupt_get(inst->Pio, __SPI_ADC_FLAG_IRQ))
    {
        tight_loop_contents();
    }

    
    pio_interrupt_clear(inst->Pio, __SPI_ADC_FLAG_IRQ); // start sm
    pio_sm_put_blocking(inst->Pio, inst->Pio_sm_1, __SPI_ADC_CNV_COUNT);

    return SPI_ADC_HW_OK;
}

static int spi_adc_hw_disable(spi_adc_hw_inst_t *inst)
{
    pio_set_sm_mask_enabled(inst->Pio, (1 << inst->Pio_sm_0) | (1 << inst->Pio_sm_1), false);
    dma_channel_abort(inst->DMA_ch);

    return SPI_ADC_HW_OK;
}

static inline bool spi_adc_hw_is_busy(spi_adc_hw_inst_t *inst)
{
    return dma_channel_is_busy(inst->DMA_ch) || !pio_interrupt_get(inst->Pio, __SPI_ADC_FLAG_IRQ);
}

static inline int spi_adc_hw_read(spi_adc_hw_inst_t *inst, uint16_t result[SPI_ADC_BUFFER_SIZE])
{
    while (spi_adc_hw_is_busy(inst))
    {
        //printf("dma %d pio %d %d %d\n", dma_channel_is_busy(inst->DMA_ch), pio_interrupt_get(inst->Pio, 0), pio_interrupt_get(inst->Pio, 1), pio_interrupt_get(inst->Pio, 6));
        tight_loop_contents();
    }

    //printf("read ok\n");

    // GetData
    memcpy(result, inst->DMA_buffer, SPI_ADC_BUFFER_SIZE * sizeof(uint16_t));

    // Restart
    dma_channel_set_write_addr(inst->DMA_ch, inst->DMA_buffer, true);
    pio_interrupt_clear(inst->Pio, __SPI_ADC_FLAG_IRQ); // start sm
    pio_sm_put_blocking(inst->Pio, inst->Pio_sm_1, __SPI_ADC_CNV_COUNT);

    return SPI_ADC_HW_OK;
}

#ifdef __cplusplus
}
#endif

#endif
