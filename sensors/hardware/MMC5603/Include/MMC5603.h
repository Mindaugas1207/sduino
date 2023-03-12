/*
 * port.h
 *
 *  Created on: Oct 15, 2022
 *      Author: minda
 */

#ifndef INCLUDE_MMC5603_H_
#define INCLUDE_MMC5603_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "port.h"

#define MMC5603_DEFAULT_ADDRESS 0x30
#define MMC5603_CHIP_ID 0x10

#define MMC5603_OUT_X0_REG 0x00
#define MMC5603_OUT_X1_REG 0x01
#define MMC5603_OUT_Y0_REG 0x02
#define MMC5603_OUT_Y1_REG 0x03
#define MMC5603_OUT_Z0_REG 0x04
#define MMC5603_OUT_Z1_REG 0x05
#define MMC5603_OUT_X2_REG 0x06
#define MMC5603_OUT_Y2_REG 0x07
#define MMC5603_OUT_Z2_REG 0x08
#define MMC5603_OUT_TEMP_REG 0x09
#define MMC5603_STATUS1_REG 0x18
#define MMC5603_ODR_REG 0x1A
#define MMC5603_CTRL0_REG 0x1B
#define MMC5603_CTRL1_REG 0x1C
#define MMC5603_CTRL2_REG 0x1D
#define MMC5603_ST_X_TH_REG 0x1E
#define MMC5603_ST_Y_TH_REG 0x1F
#define MMC5603_ST_Z_TH_REG 0x20
#define MMC5603_ST_X_REG 0x27
#define MMC5603_ST_Y_REG 0x28
#define MMC5603_ST_Z_REG 0x29
#define MMC5603_PRODUCT_ID_REG 0x39

#define MMC5603_STATUS1_MEAS_T_DONE (1<<7)
#define MMC5603_STATUS1_MEAS_M_DONE (1<<6)
#define MMC5603_STATUS1_SAT_SENSOR (1<<5)
#define MMC5603_STATUS1_OTP_READ_DONE (1<<4)

#define MMC5603_CTRL0_CMM_FREQ_EN (1<<7)
#define MMC5603_CTRL0_AUTO_ST_EN (1<<6)
#define MMC5603_CTRL0_AUTO_SR_EN (1<<5)
#define MMC5603_CTRL0_DO_RESET (1<<4)
#define MMC5603_CTRL0_DO_SET (1<<3)
#define MMC5603_CTRL0_START_MDT (1<<2)
#define MMC5603_CTRL0_TAKE_MEAS_T (1<<1)
#define MMC5603_CTRL0_TAKE_MEAS_M (1<<0)

#define MMC5603_CTRL1_SW_RESET (1<<7)
#define MMC5603_CTRL1_ST_ENM (1<<6)
#define MMC5603_CTRL1_ST_ENP (1<<5)
#define MMC5603_CTRL1_Z_INHIBIT (1<<4)
#define MMC5603_CTRL1_Y_INHIBIT (1<<3)
#define MMC5603_CTRL1_X_INHIBIT (1<<2)
#define MMC5603_CTRL1_BW1 (1<<1)
#define MMC5603_CTRL1_BW0 (1<<0)

#define MMC5603_CTRL2_HPOWER (1<<7)
#define MMC5603_CTRL2_CMM_EN (1<<4)
#define MMC5603_CTRL2_EN_PRD_SET (1<<3)
#define MMC5603_CTRL2_PRD_SET2 (1<<2)
#define MMC5603_CTRL2_PRD_SET1 (1<<1)
#define MMC5603_CTRL2_PRD_SET0 (1<<0)

#define MMC5603_DATA_READY (PORT_OK + 1)
#define MMC5603_DATA_NOT_READY (PORT_OK)
#define MMC5603_OK PORT_OK
#define MMC5603_ERROR PORT_ERROR

struct __attribute__((__packed__)) MMC5603_data_s {
    uint8_t out_data[10];
    uint8_t STATUS;
    uint8_t ODR;
    uint8_t CTRL0;
    uint8_t CTRL1;
    uint8_t CTRL2;
    uint8_t STXTH;
    uint8_t STYTH;
    uint8_t STZTH;
    uint8_t STX;
    uint8_t STY;
    uint8_t STZ;
    uint8_t PID;
};

typedef struct {
    port_i2c_t *port_inst;
    int8_t port_channel;
    uint8_t device_address;
    struct MMC5603_data_s reg_data;
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
    } mag;
    float Temperature;
} MMC5603_inst_t;

int MMC5603_init(MMC5603_inst_t *inst, port_i2c_t *port_inst, int8_t port_channel, uint8_t device_address);
int MMC5603_GetStatus(MMC5603_inst_t *inst, uint8_t *pStatus);
int MMC5603_NewDataReady(MMC5603_inst_t *inst);
int MMC5603_StartMeas(MMC5603_inst_t *inst);
int MMC5603_Configure(MMC5603_inst_t *inst);
int MMC5603_ReadData(MMC5603_inst_t *inst);
int MMC5603_ReadDataBlocking(MMC5603_inst_t *inst);

// mx = (MAG_INST.X - MAG_INST.Xbias) * MAG_INST.Xscale * (100.0f / 16384.0f);
// my = (MAG_INST.Y - MAG_INST.Ybias) * MAG_INST.Yscale * (100.0f / 16384.0f);
// mz = (MAG_INST.Z - MAG_INST.Zbias) * MAG_INST.Zscale * (100.0f / 16384.0f);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_MMC5603_H_ */
