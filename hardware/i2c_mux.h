/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _I2C_MUX_H
#define _I2C_MUX_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define PI4MSD5V9540B_DEFAULT_ADDRESS (0x70)
#define I2C_MUX_CHANNEL_NONE (0)
#define I2C_MUX_CHANNEL_0 (0x4)
#define I2C_MUX_CHANNEL_1 (0x5)
#define I2C_MUX_CHANNEL_ANY (0xFF)

#define I2C_MUX_OK PICO_OK
#define I2C_MUX_ERROR PICO_ERROR_GENERIC

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    i2c_inst_t *i2c;
    uint8_t address;
    uint8_t channel;
} i2c_mux_inst_t;

static inline int i2c_mux_get_channel(i2c_mux_inst_t *inst)
{
    uint8_t channel = 0;
    
    if (i2c_read_blocking(inst->i2c, inst->address, &channel, 1, false) == PICO_ERROR_GENERIC)
        return I2C_MUX_ERROR;

    inst->channel = channel;

    return inst->channel;
}

static inline int i2c_mux_set_channel(i2c_mux_inst_t *inst, uint8_t channel)
{
    if (channel == inst->channel || channel == I2C_MUX_CHANNEL_ANY)
        return I2C_MUX_OK;

    if (i2c_write_blocking(inst->i2c, inst->address, &channel, 1, false) == PICO_ERROR_GENERIC)
        return I2C_MUX_ERROR;

    inst->channel = channel;

    return I2C_MUX_OK;
}

static int i2c_mux_init(i2c_mux_inst_t *inst)
{
    if (inst->i2c == NULL)
        return I2C_MUX_ERROR;

    inst->channel = I2C_MUX_CHANNEL_ANY;

    if (i2c_mux_get_channel(inst) == I2C_MUX_ERROR)
        return I2C_MUX_ERROR;

    if (i2c_mux_set_channel(inst, I2C_MUX_CHANNEL_NONE) == I2C_MUX_ERROR)
        return I2C_MUX_ERROR;

    return I2C_MUX_OK;
}

#ifdef __cplusplus
}
#endif

#endif
