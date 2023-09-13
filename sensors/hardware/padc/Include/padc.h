/*
 * 2022-11-02, Minduagas Mikalauskas.
 */

#ifndef _PADC_H
#define _PADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/util/queue.h"
#include "padc.pio.h"

#define PADC_DEFAULT_CLK_PIN 6
#define PADC_DEFAULT_CNV_PIN 5
#define PADC_DEFAULT_DATA_PIN 4
#define PADC_DEFAULT_RESET_PIN 2

#define PADC_BUFFER_SIZE __PADC_CNV_COUNT
#define PADC_FIFO_SIZE 10

#define PADC_PIO pio0
#define PADC_PIO_SM0 0
#define PADC_PIO_SM1 1
#define PADC_DMA_CH 0

#define PADC_OK 0
#define PADC_ERROR -1

typedef struct {
    uint16_t dma_buffer[PADC_BUFFER_SIZE];
    queue_t sample_fifo;
    uint32_t fspi;
    uint32_t fsampling;
    uint clk_pin;
    uint data_pin;
    uint cnv_pin;
    uint reset_pin;
    volatile bool Stop;
    volatile bool Enabled;
} padc_inst_t;

int padc_init(uint32_t fspi, uint32_t fsampling, uint clk_pin, uint data_pin, uint cnv_pin, uint reset_pin);
bool padc_is_read_ongoing();
bool padc_new_data_available();
void padc_start();
void padc_stop();
bool padc_read(uint16_t buffer[PADC_BUFFER_SIZE]);
void padc_read_blocking(uint16_t buffer[PADC_BUFFER_SIZE]);

#ifdef __cplusplus
}
#endif

#endif
// /*
//  * 2022-11-02, Minduagas Mikalauskas.
//  */

// #ifndef _PADC_H
// #define _PADC_H

// #ifdef __cplusplus
// extern "C" {
// #endif

// #include "pico/stdlib.h"
// #include "hardware/dma.h"
// #include "hardware/pio.h"
// #include "hardware/clocks.h"
// #include "pico/util/queue.h"
// #include "padc.pio.h"

// #define PADC_DEFAULT_CLK_PIN 6
// #define PADC_DEFAULT_CNV_PIN 5
// #define PADC_DEFAULT_DATA_PIN 4
// #define PADC_DEFAULT_RESET_PIN 2

// #define PADC_BUFFER_SIZE __PADC_CNV_COUNT
// #define PADC_FIFO_SIZE 10

// #define PADC_PIO pio0
// #define PADC_PIO_SM0 0
// #define PADC_PIO_SM1 1
// #define PADC_DMA_CH 0

// #define PADC_OK 0
// #define PADC_ERROR -1

// typedef struct {
//     uint16_t dma_buffer[PADC_BUFFER_SIZE];
//     queue_t sample_fifo;
//     uint32_t fspi;
//     uint32_t fsampling;
//     uint clk_pin;
//     uint data_pin;
//     uint cnv_pin;
//     uint reset_pin;
//     uint prog_offs_sm0;
//     volatile bool Stop;
//     volatile bool Enabled;
//     volatile bool Busy;
// } padc_inst_t;

// int padc_init(uint32_t fspi, uint32_t fsampling, uint clk_pin, uint data_pin, uint cnv_pin, uint reset_pin);
// bool padc_is_read_ongoing();
// bool padc_new_data_available();
// void padc_start();
// void padc_stop();
// bool padc_read(uint16_t buffer[PADC_BUFFER_SIZE]);
// void padc_read_blocking(uint16_t buffer[PADC_BUFFER_SIZE]);
// #ifdef __cplusplus
// }
// #endif

// #endif