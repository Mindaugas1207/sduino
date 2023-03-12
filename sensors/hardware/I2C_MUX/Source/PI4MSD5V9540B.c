
#include "PI4MSD5V9540B.h"

#define CHANNEL_TO_CMD(ch) (ch + PI4MSD5V9540B_CMD_SET_CHANNEL_0)
#define CMD_TO_CHANNEL(cmd) (cmd + PI4MSD5V9540B_CHANNEL_NONE)

int PI4MSD5V9540B_init(PI4MSD5V9540B_inst_t *inst, i2c_inst_t *i2c, uint8_t address)
{
    inst->i2c = i2c;
    inst->address = address;
    inst->channel = 0xFF;

    uint8_t rxdata;
	if (i2c_read_blocking(i2c, address, &rxdata, 1, false) == PICO_ERROR_GENERIC)
        return PI4MSD5V9540B_ERROR;
    if (PI4MSD5V9540B_set_channel(inst, PI4MSD5V9540B_CHANNEL_NONE) == PI4MSD5V9540B_ERROR)
        return PI4MSD5V9540B_ERROR;

    return PI4MSD5V9540B_OK;
}

int PI4MSD5V9540B_get_channel(PI4MSD5V9540B_inst_t *inst)
{
    uint8_t cmd = 0;
    if (i2c_read_blocking(inst->i2c, inst->address, &cmd, 1, false) == PICO_ERROR_GENERIC) 
        return PI4MSD5V9540B_ERROR;
    return CMD_TO_CHANNEL(cmd);
}

int PI4MSD5V9540B_set_channel(PI4MSD5V9540B_inst_t *inst, int8_t channel)
{
    if(channel == inst->channel || channel == PI4MSD5V9540B_CHANNEL_ANY) return PI4MSD5V9540B_OK;
	inst->channel = channel;
    uint8_t cmd = CHANNEL_TO_CMD(channel);
    if (i2c_write_blocking(inst->i2c, inst->address, &cmd, 1, false) == PICO_ERROR_GENERIC)
        return PI4MSD5V9540B_ERROR;
    return PI4MSD5V9540B_OK;
}
