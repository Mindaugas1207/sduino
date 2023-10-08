/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _ENCODER_H
#define _ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "encoder.pio.h"

#define PENC_OK 0
#define PENC_ERROR -1

void encoder_init(PIO pio, uint sm, uint pinA, uint pinB, uint32_t fsampling);
int32_t quadrature_encoder_get_count(PIO pio, uint sm);

#ifdef __cplusplus
}
#endif

#endif