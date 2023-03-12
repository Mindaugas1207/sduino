
#include "IS31FL3218.h"

int IS31FL3218_init(IS31FL3218_inst_t *inst, port_i2c_t *port_inst, int8_t port_channel)
{
    inst->port_inst = port_inst;
    inst->port_channel = port_channel;
    inst->data.reg = IS31FL3218_PWM_1_REG;
    for (int i = 0; i < IS31FL3218_CHANNEL_COUNT; i++ ) inst->data.pwm[i] = 0;
    inst->data.ctrl[0] = 0;
    inst->data.ctrl[1] = 0;
    inst->data.ctrl[2] = 0;
    inst->data.update = 0;

    if (IS31FL3218_reset(inst) == IS31FL3218_ERROR) return IS31FL3218_ERROR;

    return IS31FL3218_OK;
}


