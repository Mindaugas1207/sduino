/*
 * 2022-11-02, Minduagas Mikalauskas.
 */

#ifndef _IS31FL3218_H
#define _IS31FL3218_H

#ifdef __cplusplus
extern "C" {
#endif

#include "port.h"

#define IS31FL3218_ADDRESS 0x54

// Registers

#define IS31FL3218_CHANNEL_COUNT 18
#define IS31FL3218_SHUTDOWN_REG 0x00
#define IS31FL3218_PWM_1_REG 0x01
#define IS31FL3218_PWM_2_REG 0x02
#define IS31FL3218_PWM_3_REG 0x03
#define IS31FL3218_PWM_4_REG 0x04
#define IS31FL3218_PWM_5_REG 0x05
#define IS31FL3218_PWM_6_REG 0x06
#define IS31FL3218_PWM_7_REG 0x07
#define IS31FL3218_PWM_8_REG 0x08
#define IS31FL3218_PWM_9_REG 0x09
#define IS31FL3218_PWM_10_REG 0x0A
#define IS31FL3218_PWM_11_REG 0x0B
#define IS31FL3218_PWM_12_REG 0x0C
#define IS31FL3218_PWM_13_REG 0x0D
#define IS31FL3218_PWM_14_REG 0x0E
#define IS31FL3218_PWM_15_REG 0x0F
#define IS31FL3218_PWM_16_REG 0x10
#define IS31FL3218_PWM_17_REG 0x11
#define IS31FL3218_PWM_18_REG 0x12
#define IS31FL3218_LED_CONTROL_1_REG 0x13
#define IS31FL3218_LED_CONTROL_2_REG 0x14
#define IS31FL3218_LED_CONTROL_3_REG 0x15
#define IS31FL3218_UPDATE_REG 0x16
#define IS31FL3218_RESET_REG 0x17

//

#define IS31FL3218_OK PORT_OK
#define IS31FL3218_ERROR PORT_ERROR

static const uint8_t IS31FL3218_GAMMA_32[] = { 0,1,2,4,6,10,13,18,22,28,33,39,46,53,61,69,78,86,96,106,116,126,138,149,161,173,186,199,212,226,240,255 };
static const uint8_t IS31FL3218_GAMMA_64[] = { 0,1,2,3,4,5,6,7,8,10,12,14,16,18,20,22,24,26,29,32,35,38,41,44,47,50,53,57,61,65,69,73,77,81,85,89,94,99,104,109,114,119,124,129,134,140,146,152,158,164,170,176,182,188,195,202,209,216,223,230,237,244,251,255 };
#define IS31FL3218_GAMMA_32_STEPS sizeof(IS31FL3218_GAMMA_32)
#define IS31FL3218_GAMMA_64_STEPS sizeof(IS31FL3218_GAMMA_64)
#define IS31FL3218_GAMMA_32_STEP_MAX (sizeof(IS31FL3218_GAMMA_32)-1)
#define IS31FL3218_GAMMA_64_STEP_MAX (sizeof(IS31FL3218_GAMMA_64)-1)
#define IS31FL3218_PWM_MAX 255

struct __attribute__((__packed__)) IS31FL3218_data_s {
    uint8_t reg;
    uint8_t pwm[IS31FL3218_CHANNEL_COUNT];
    uint8_t ctrl[3];
    uint8_t update;
};

typedef struct {
    port_i2c_t *port_inst;
    int8_t port_channel;
    struct IS31FL3218_data_s data;
} IS31FL3218_inst_t;

int IS31FL3218_init(IS31FL3218_inst_t *inst, port_i2c_t *port_inst, int8_t port_channel);

static inline int IS31FL3218_enable(IS31FL3218_inst_t *inst)
{ 
    uint8_t buff[] = {IS31FL3218_SHUTDOWN_REG, 0x01};
    if (port_i2c_write(inst->port_inst, inst->port_channel, IS31FL3218_ADDRESS, buff, sizeof(buff)) == PORT_ERROR) return IS31FL3218_ERROR;
    return IS31FL3218_OK;
}

static inline int IS31FL3218_disable(IS31FL3218_inst_t *inst)
{
    uint8_t buff[] = {IS31FL3218_SHUTDOWN_REG, 0x00};
    if (port_i2c_write(inst->port_inst, inst->port_channel, IS31FL3218_ADDRESS, buff, sizeof(buff)) == PORT_ERROR) return IS31FL3218_ERROR;
    return IS31FL3218_OK;
}

static inline int IS31FL3218_reset(IS31FL3218_inst_t *inst)
{
    uint8_t buff[] = {IS31FL3218_RESET_REG, 0x00};
    if (port_i2c_write(inst->port_inst, inst->port_channel, IS31FL3218_ADDRESS, buff, sizeof(buff)) == PORT_ERROR) return IS31FL3218_ERROR;
    return IS31FL3218_OK;
}

static inline int IS31FL3218_update(IS31FL3218_inst_t *inst)
{
    uint8_t buff[] = {IS31FL3218_UPDATE_REG, 0x00};
    if (port_i2c_write(inst->port_inst, inst->port_channel, IS31FL3218_ADDRESS, buff, sizeof(buff)) == PORT_ERROR) return IS31FL3218_ERROR;
    return IS31FL3218_OK;
}

static inline int IS31FL3218_write_data(IS31FL3218_inst_t *inst)
{
    if (port_i2c_write(inst->port_inst, inst->port_channel, IS31FL3218_ADDRESS, (uint8_t*)&inst->data, sizeof(inst->data)) == PORT_ERROR) return IS31FL3218_ERROR;
    return IS31FL3218_OK;
}

static inline void IS31FL3218_set_channel_pwm(IS31FL3218_inst_t *inst, uint8_t channel, uint8_t pwmValue)
{
    inst->data.pwm[channel] = pwmValue;
}

static inline void IS31FL3218_set_channel_enable(IS31FL3218_inst_t *inst, uint8_t channel, bool enable) 
{
    int idx = channel / 6;
    int ci = channel - idx * 6;
    inst->data.ctrl[idx] = (inst->data.ctrl[idx] & ~(1 << ci)) | (enable << ci);
}

static inline void IS31FL3218_toggle_channel_enable(IS31FL3218_inst_t *inst, uint8_t channel) 
{
    int idx = channel / 6;
    int ci = channel - idx * 6;
    inst->data.ctrl[idx] = inst->data.ctrl[idx] ^ (1 << ci);
}

static inline void IS31FL3218_set_channels_enable(IS31FL3218_inst_t *inst, uint32_t mask)
{
    inst->data.ctrl[0] = mask;
    inst->data.ctrl[1] = mask >> 6;
    inst->data.ctrl[2] = mask >> 12;
}

#ifdef __cplusplus
}
#endif

#endif
