/*
 * 2022-11-02, Minduagas Mikalauskas.
 */

#ifndef _PCA9548A_H
#define _PCA9548A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define PCA9548A_DEFAULT_ADDRESS 0x70
#define PCA9548A_CHANNEL_ANY -0x5
#define PCA9548A_CHANNEL_NONE -0x4
#define PCA9548A_CHANNEL_0 0x0
#define PCA9548A_CHANNEL_1 0x1
#define PCA9548A_CHANNEL_2 0x2
#define PCA9548A_CHANNEL_3 0x3
#define PCA9548A_CHANNEL_4 0x4
#define PCA9548A_CHANNEL_5 0x5
#define PCA9548A_CHANNEL_6 0x6
#define PCA9548A_CHANNEL_7 0x7

#define PCA9548A_CMD_SET_CHANNEL_NONE 0x0
#define PCA9548A_CMD_SET_CHANNEL_0 (1 << 0)
#define PCA9548A_CMD_SET_CHANNEL_1 (1 << 1)
#define PCA9548A_CMD_SET_CHANNEL_2 (1 << 2)
#define PCA9548A_CMD_SET_CHANNEL_3 (1 << 3)
#define PCA9548A_CMD_SET_CHANNEL_4 (1 << 4)
#define PCA9548A_CMD_SET_CHANNEL_5 (1 << 5)
#define PCA9548A_CMD_SET_CHANNEL_6 (1 << 6)
#define PCA9548A_CMD_SET_CHANNEL_7 (1 << 7)

#define PCA9548A_OK PICO_OK
#define PCA9548A_ERROR PICO_ERROR_GENERIC

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
    uint8_t channel;
} PCA9548A_inst_t;

int PCA9548A_init(PCA9548A_inst_t *inst, i2c_inst_t *i2c, uint8_t address);
int PCA9548A_get_channel(PCA9548A_inst_t *inst);
int PCA9548A_set_channel(PCA9548A_inst_t *inst, int8_t channel);

#ifdef __cplusplus
}
#endif

#endif
