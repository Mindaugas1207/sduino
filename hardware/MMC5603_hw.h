
#ifndef MMC5603_HW_H
#define MMC5603_HW_H

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

#define MMC5603_DATA_READY (PICO_OK)
#define MMC5603_DATA_NOT_READY (PICO_OK + 1)
#define MMC5603_OK PICO_OK
#define MMC5603_ERROR PICO_ERROR_GENERIC

#ifdef __cplusplus
extern "C" {
#endif

struct __attribute__((__packed__)) MMC5603_hw_data_s
{
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
    port_device_t port_device;
    struct MMC5603_hw_data_s reg_data;
} MMC5603_hw_inst_t;

typedef struct
{
    double X, Y, Z;
    double Temperature;
} MMC5603_hw_data_t;

static int MMC5603_hw_WriteByte(MMC5603_hw_inst_t *inst, uint8_t index, uint8_t data)
{
	int status = PORT_OK;
    uint8_t buff[] = {index, data};

    status |= port_write(&inst->port_device, buff, sizeof(buff));

    if (status != PORT_OK) return MMC5603_ERROR;
    return MMC5603_OK;
}

static int MMC5603_hw_ReadByte(MMC5603_hw_inst_t *inst, uint8_t index, uint8_t *data)
{
	int status = PORT_OK;

    status |= port_write(&inst->port_device, &index, sizeof(index));
	status |= port_read(&inst->port_device, data, sizeof(uint8_t));

    if (status != PORT_OK) return MMC5603_ERROR;
    return MMC5603_OK;
}

static int MMC5603_hw_init(MMC5603_hw_inst_t *inst)
{
    int status = MMC5603_OK;

    status |= MMC5603_hw_WriteByte(inst, MMC5603_CTRL1_REG, MMC5603_CTRL1_SW_RESET);
    port_delay(50);
    uint8_t tmp = MMC5603_OUT_X0_REG;
    status |= port_write(&inst->port_device, &tmp, sizeof(tmp));
	status |= port_read(&inst->port_device, (uint8_t*)&inst->reg_data, sizeof(MMC5603_hw_data_s));

    inst->reg_data.ODR = 255;
    inst->reg_data.CTRL0 = 0;//MMC5603_CTRL0_AUTO_SR_EN;
    inst->reg_data.CTRL1 = MMC5603_CTRL1_BW0 | MMC5603_CTRL1_BW1;
    inst->reg_data.CTRL2 = MMC5603_CTRL2_HPOWER;

    status |= MMC5603_hw_WriteByte(inst, MMC5603_ODR_REG, inst->reg_data.ODR);
    status |= MMC5603_hw_WriteByte(inst, MMC5603_CTRL0_REG, inst->reg_data.CTRL0);
    status |= MMC5603_hw_WriteByte(inst, MMC5603_CTRL1_REG, inst->reg_data.CTRL1);
    status |= MMC5603_hw_WriteByte(inst, MMC5603_CTRL2_REG, inst->reg_data.CTRL2);

    inst->reg_data.CTRL0 |= MMC5603_CTRL0_CMM_FREQ_EN;
    status |= MMC5603_hw_WriteByte(inst, MMC5603_CTRL0_REG, inst->reg_data.CTRL0);
    inst->reg_data.CTRL2 |= MMC5603_CTRL2_CMM_EN;
    status |= MMC5603_hw_WriteByte(inst, MMC5603_CTRL2_REG, inst->reg_data.CTRL2);

    MMC5603_hw_WriteByte(inst, MMC5603_CTRL0_REG, inst->reg_data.CTRL0 | MMC5603_CTRL0_TAKE_MEAS_M);

    return status;
}

static int MMC5603_hw_new_data_available(MMC5603_hw_inst_t *inst)
{
    int status = MMC5603_DATA_NOT_READY;
    uint8_t statusreg = 0;
    status |= MMC5603_hw_ReadByte(inst, MMC5603_STATUS1_REG, &statusreg);
    if (statusreg & MMC5603_STATUS1_MEAS_M_DONE)
    {
        return MMC5603_DATA_READY;
    }
    
    return status;
}

static int MMC5603_hw_read_data(MMC5603_hw_inst_t *inst, MMC5603_hw_data_t *result)
{
    int status = PORT_OK;
    uint8_t tmp = MMC5603_OUT_X0_REG;
    status |= port_write(&inst->port_device, &tmp, sizeof(tmp));
	status |= port_read(&inst->port_device, (uint8_t*)&inst->reg_data, 11);

    if (status != PORT_OK) return MMC5603_ERROR;

    int32_t x, y, z;
    int16_t t;

    x = (uint32_t)inst->reg_data.out_data[0] << 12
      | (uint32_t)inst->reg_data.out_data[1] << 4
      | (uint32_t)inst->reg_data.out_data[6] >> 4;

    y = (uint32_t)inst->reg_data.out_data[2] << 12
      | (uint32_t)inst->reg_data.out_data[3] << 4
      | (uint32_t)inst->reg_data.out_data[7] >> 4;

    z = (uint32_t)inst->reg_data.out_data[4] << 12
      | (uint32_t)inst->reg_data.out_data[5] << 4
      | (uint32_t)inst->reg_data.out_data[8] >> 4;

    t = (int16_t)inst->reg_data.out_data[9];

    result->X = x - (1 << (20 - 1));
    result->Y = y - (1 << (20 - 1));
    result->Z = z - (1 << (20 - 1));
    result->Temperature = t * 0.8f - 75.0;

    return MMC5603_OK;
}

// mx = (MAG_INST.X - MAG_INST.Xbias) * MAG_INST.Xscale * (100.0f / 16384.0f);
// my = (MAG_INST.Y - MAG_INST.Ybias) * MAG_INST.Yscale * (100.0f / 16384.0f);
// mz = (MAG_INST.Z - MAG_INST.Zbias) * MAG_INST.Zscale * (100.0f / 16384.0f);

#ifdef __cplusplus
}
#endif

#endif /* MMC5603_HW_H */
