/*
 * 2022-11-02, Minduagas Mikalauskas.
 */

#ifndef _PI4MSD5V9540B_H
#define _PI4MSD5V9540B_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define PI4MSD5V9540B_DEFAULT_ADDRESS 0x70
#define PI4MSD5V9540B_CHANNEL_ANY -0x5
#define PI4MSD5V9540B_CHANNEL_NONE -0x4
#define PI4MSD5V9540B_CHANNEL_0 0x0
#define PI4MSD5V9540B_CHANNEL_1 0x1

#define PI4MSD5V9540B_CMD_SET_CHANNEL_NONE 0x0
#define PI4MSD5V9540B_CMD_SET_CHANNEL_0 0x4
#define PI4MSD5V9540B_CMD_SET_CHANNEL_1 0x5

#define PI4MSD5V9540B_OK PICO_OK
#define PI4MSD5V9540B_ERROR PICO_ERROR_GENERIC

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
    uint8_t channel;
} PI4MSD5V9540B_inst_t;

int PI4MSD5V9540B_init(PI4MSD5V9540B_inst_t *inst, i2c_inst_t *i2c, uint8_t address);
int PI4MSD5V9540B_get_channel(PI4MSD5V9540B_inst_t *inst);
int PI4MSD5V9540B_set_channel(PI4MSD5V9540B_inst_t *inst, int8_t channel);

#ifdef __cplusplus
}
#endif

#endif
