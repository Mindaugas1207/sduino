
#include "PCA9548A.h"

#define CHANNEL_TO_CMD(ch) (ch != PCA9548A_CHANNEL_NONE ? (PCA9548A_CMD_SET_CHANNEL_0 << ch) : PCA9548A_CMD_SET_CHANNEL_NONE)
#define CMD_TO_CHANNEL(cmd) (cmd != PCA9548A_CMD_SET_CHANNEL_NONE ? cmd : PCA9548A_CHANNEL_NONE)

int PCA9548A_init(PCA9548A_inst_t *inst, i2c_inst_t *i2c, uint8_t address)
{
    inst->i2c = i2c;
    inst->address = address;
    inst->channel = 0xFF;

    uint8_t rxdata;
	if (i2c_read_blocking(i2c, address, &rxdata, 1, false) == PICO_ERROR_GENERIC)
        return PCA9548A_ERROR;
    if (PCA9548A_set_channel(inst, PCA9548A_CHANNEL_NONE) == PCA9548A_ERROR)
        return PCA9548A_ERROR;

    return PCA9548A_OK;
}

int PCA9548A_get_channel(PCA9548A_inst_t *inst)
{
    uint8_t cmd = 0;
    if (i2c_read_blocking(inst->i2c, inst->address, &cmd, 1, false) == PICO_ERROR_GENERIC) 
        return PCA9548A_ERROR;
    return CMD_TO_CHANNEL(cmd);
}

int PCA9548A_set_channel(PCA9548A_inst_t *inst, int8_t channel)
{
    if(channel == inst->channel || channel == PCA9548A_CHANNEL_ANY) return PCA9548A_OK;
	inst->channel = channel;
    uint8_t cmd = CHANNEL_TO_CMD(channel);
    if (i2c_write_blocking(inst->i2c, inst->address, &cmd, 1, false) == PICO_ERROR_GENERIC)
        return PCA9548A_ERROR;
    return PCA9548A_OK;
}
