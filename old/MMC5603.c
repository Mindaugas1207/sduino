
#include "MMC5603.h"
#include "stdio.h"
#include "pico/stdlib.h"
#include "time.h"

int MMC5603_WriteByte(MMC5603_inst_t *inst, uint8_t index, uint8_t data);
int MMC5603_ReadByte(MMC5603_inst_t *inst, uint8_t index, uint8_t *data);
int MMC5603_ReadAllRegs(MMC5603_inst_t *inst);
int MMC5603_Reset(MMC5603_inst_t *inst);

int MMC5603_init(MMC5603_inst_t *inst)
{
    int status = MMC5603_OK;

    status |= MMC5603_Reset(inst);
    port_delay(50);
    status |= MMC5603_ReadAllRegs(inst);
    status |= MMC5603_Configure(inst);
    return status;
}

int MMC5603_WriteByte(MMC5603_inst_t *inst, uint8_t index, uint8_t data)
{
	int status = MMC5603_OK;
    uint8_t buff[] = {index, data};

    status |= port_write(&inst->port_device, buff, sizeof(buff));

    if (status < MMC5603_OK) return MMC5603_ERROR;
    return MMC5603_OK;
}

int MMC5603_ReadByte(MMC5603_inst_t *inst, uint8_t index, uint8_t *data)
{
	int status = MMC5603_OK;

    status |= port_write(&inst->port_device, &index, sizeof(index));
	status |= port_read(&inst->port_device, data, sizeof(uint8_t));

    if (status < MMC5603_OK) return MMC5603_ERROR;
    return MMC5603_OK;
}

int MMC5603_ReadMulti(MMC5603_inst_t *inst, uint8_t index, uint8_t *data, uint32_t size)
{
	int status = MMC5603_OK;

    status |= port_write(&inst->port_device, &index, sizeof(index));
	status |= port_read(&inst->port_device, data, size);

    if (status < MMC5603_OK) return MMC5603_ERROR;
    return MMC5603_OK;
}

int MMC5603_GetStatus(MMC5603_inst_t *inst, uint8_t *pStatus)
{
    return MMC5603_ReadByte(inst, MMC5603_STATUS1_REG, pStatus);
}

int MMC5603_Reset(MMC5603_inst_t *inst)
{
    return MMC5603_WriteByte(inst, MMC5603_CTRL1_REG, MMC5603_CTRL1_SW_RESET);
}

int MMC5603_StartMeas(MMC5603_inst_t *inst)
{
    return MMC5603_WriteByte(inst, MMC5603_CTRL0_REG, inst->reg_data.CTRL0 | MMC5603_CTRL0_TAKE_MEAS_M);
}

int MMC5603_ReadAllRegs(MMC5603_inst_t *inst)
{
    int status = MMC5603_OK;
    status |= MMC5603_ReadMulti(inst, MMC5603_OUT_X0_REG,(uint8_t*)&inst->reg_data, sizeof(inst->reg_data));
    return status;
}

int MMC5603_Configure(MMC5603_inst_t *inst)
{
    int status = MMC5603_OK;

    inst->reg_data.ODR = 255;
    inst->reg_data.CTRL0 = 0;//MMC5603_CTRL0_AUTO_SR_EN;
    inst->reg_data.CTRL1 = MMC5603_CTRL1_BW0 | MMC5603_CTRL1_BW1;
    inst->reg_data.CTRL2 = MMC5603_CTRL2_HPOWER;

    status |= MMC5603_WriteByte(inst, MMC5603_ODR_REG, inst->reg_data.ODR);
    status |= MMC5603_WriteByte(inst, MMC5603_CTRL0_REG, inst->reg_data.CTRL0);
    status |= MMC5603_WriteByte(inst, MMC5603_CTRL1_REG, inst->reg_data.CTRL1);
    status |= MMC5603_WriteByte(inst, MMC5603_CTRL2_REG, inst->reg_data.CTRL2);

    inst->reg_data.CTRL0 |= MMC5603_CTRL0_CMM_FREQ_EN;
    status |= MMC5603_WriteByte(inst, MMC5603_CTRL0_REG, inst->reg_data.CTRL0);
    inst->reg_data.CTRL2 |= MMC5603_CTRL2_CMM_EN;
    status |= MMC5603_WriteByte(inst, MMC5603_CTRL2_REG, inst->reg_data.CTRL2);


    return status;
}

int MMC5603_NewDataReady(MMC5603_inst_t *inst)
{
    int status = MMC5603_OK;
    uint8_t statusreg = 0;
    status |= MMC5603_GetStatus(inst, &statusreg);
    if ((statusreg & (MMC5603_STATUS1_MEAS_M_DONE)) == (MMC5603_STATUS1_MEAS_M_DONE))
    {
        return MMC5603_DATA_READY;
    }
    
    return status;
}

int MMC5603_ReadData(MMC5603_inst_t *inst)
{
    int status = MMC5603_OK;
    status |= MMC5603_ReadMulti(inst, MMC5603_OUT_X0_REG, (uint8_t*)&inst->reg_data, 11);
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

    inst->mag.x = x - (1 << (20 - 1));
    inst->mag.y = y - (1 << (20 - 1));
    inst->mag.z = z - (1 << (20 - 1));
    inst->Temperature = t * 0.8f - 75.0;

    return status;
}

int MMC5603_ReadDataBlocking(MMC5603_inst_t *inst)
{
    int status = MMC5603_OK;

    status = MMC5603_StartMeas(inst);

    while (status == MMC5603_OK)
        status = MMC5603_NewDataReady(inst);

    status |= MMC5603_ReadData(inst);
    return status;
}
