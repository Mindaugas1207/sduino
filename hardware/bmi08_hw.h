/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef BMI088_HW_H
#define BMI088_HW_H

#include "port.h"
#include "bmi08x.h"
#include "bmi08x_defs.h"

#define BMI08X_READ_WRITE_LEN  UINT8_C(46)

/*! BMI085 shuttle id */
#define BMI085_SHUTTLE_ID      UINT16_C(0x46)

/*! BMI088 shuttle id */
#define BMI088_SHUTTLE_ID      UINT16_C(0x66)

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08X_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
static BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    port_spi_dev_t* intf = (port_spi_dev_t*)intf_ptr;
    return port_spi_read_mem(intf->port, intf->dev_num, reg_addr, reg_data, length);
}
/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08X_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
static BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    port_spi_dev_t* intf = (port_spi_dev_t*)intf_ptr;
    return port_spi_write_mem(intf->port, intf->dev_num, reg_addr, (uint8_t *)reg_data, length);
}
/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us    : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
static void bmi08x_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    port_delay_us(period);
}

static int BMI088_Init(struct bmi08x_dev *inst, port_spi_dev_t *gyro_spi_dev, port_spi_dev_t *accel_spi_dev)
{
    int status = BMI08X_OK;

    inst->intf = BMI08X_SPI_INTF;
    inst->read = bmi08x_spi_read;
    inst->write = bmi08x_spi_write;
    inst->variant = BMI088_VARIANT;

    inst->intf_ptr_accel = accel_spi_dev;
    inst->intf_ptr_gyro = gyro_spi_dev;
    inst->delay_us = bmi08x_delay_us;
    inst->read_write_len = BMI08X_READ_WRITE_LEN;

    status |= bmi08a_init(inst); //init accelerometer
    status |= bmi08g_init(inst); //init gyro

    status |= bmi08a_soft_reset(inst); //reset accelerometer

    inst->read_write_len = 32; //Read/write length

    status |= bmi08a_load_config_file(inst); //load cfg file

    inst->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    status |= bmi08a_set_power_mode(inst); //set accel power mode

    inst->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
    status |= bmi08g_set_power_mode(inst); //set gyro power mode

    inst->accel_cfg.range = BMI088_ACCEL_RANGE_24G; //accel range
    inst->gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS; //gyro range
    
    //Sync
    struct bmi08x_data_sync_cfg sync_cfg;

    /* Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
    sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ;

    status |= bmi08a_configure_data_synchronization(sync_cfg, inst);

    //interrupts
    struct bmi08x_int_cfg int_cfg;

    int_cfg.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
    int_cfg.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
    int_cfg.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_cfg.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_cfg.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    int_cfg.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
    int_cfg.accel_int_config_2.int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY;
    int_cfg.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_cfg.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_cfg.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    int_cfg.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
    int_cfg.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
    int_cfg.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
    int_cfg.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_cfg.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    int_cfg.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
    int_cfg.gyro_int_config_2.int_type = BMI08X_GYRO_INT_DATA_RDY;
    int_cfg.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
    int_cfg.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_cfg.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    status |= bmi08a_set_data_sync_int_config(&int_cfg, inst);

    return status;
}

static int BMI088_ReadData(struct bmi08x_dev *inst, int16_t gyro[3], int16_t accel[3])
{
    bmi08x_sensor_data gdata, adata;

    int status = bmi08a_get_synchronized_data(&gdata, &adata, inst);

    gyro[0] = gdata.x;
    gyro[1] = gdata.y;
    gyro[2] = gdata.z;
    accel[0] = adata.x;
    accel[1] = adata.y;
    accel[2] = adata.z;

    return status;
}

//#define DUMY_OFFSET 1
//#define BMI08X_GYRO_INT3_LVL_POS 0
//extern const uint8_t bmi08x_config_file[];

// void BMI088_ReadData2(port_spi_t *port, uint gyro_dev_num, uint accel_dev_num, int16_t results[6])
// {

//     uint8_t data[12];

//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_GP_0, data, DUMY_OFFSET + 11);

//     results[0] = (int16_t)((data[DUMY_OFFSET + 1] << 8) | data[DUMY_OFFSET + 0]); /* Data in X axis */
//     results[1] = (int16_t)((data[DUMY_OFFSET + 3] << 8) | data[DUMY_OFFSET + 2]); /* Data in Y axis */
//     results[2] = (int16_t)((data[DUMY_OFFSET + 10] << 8) | data[DUMY_OFFSET + 9]); /* Data in Z axis */

//     port_spi_read_mem(port, gyro_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_GYRO_X_LSB, data, 6);

//     results[3] = (int16_t)((data[1] << 8) | data[0]); /* Data in X axis */
//     results[4] = (int16_t)((data[3] << 8) | data[2]); /* Data in Y axis */
//     results[5] = (int16_t)((data[5] << 8) | data[4]); /* Data in Z axis */
// }






// bool BMI088_init2(port_spi_t *port, uint gyro_dev_num, uint accel_dev_num)
// {
//     uint8_t data[7];

//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_CHIP_ID, data, DUMY_OFFSET + 1);
//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_CHIP_ID, data, DUMY_OFFSET + 1);
//     if (data[DUMY_OFFSET + 0] != BMI088_ACCEL_CHIP_ID)
//         return false;
//     port_spi_read_mem(port, gyro_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_GYRO_CHIP_ID, data, 1);
//     if (data[0] != BMI08X_GYRO_CHIP_ID)
//         return false;
//     data[0] = BMI08X_SOFT_RESET_CMD;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_SOFTRESET, data, 1);
//     port_delay_us(1000);
//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_CHIP_ID, data, DUMY_OFFSET + 1);
//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_CHIP_ID, data, DUMY_OFFSET + 1);
//     if (data[DUMY_OFFSET + 0] != BMI088_ACCEL_CHIP_ID)
//         return false;

//     data[0] = 0;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_PWR_CONF, data, 1);
//     port_delay_us(450);
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_INIT_CTRL, data, 1);
//     data[0] = 0;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_RESERVED_5B, data, 1);
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_RESERVED_5C, data, 1);
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_FEATURE_CFG, bmi08x_config_file, BMI08X_CONFIG_STREAM_SIZE);
//     data[0] = 1;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_INIT_CTRL, data, 1);
//     port_delay_us(BMI08X_ASIC_INIT_TIME_MS * 1000);
//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_INTERNAL_STAT, data, DUMY_OFFSET + 1);
//     if (data[DUMY_OFFSET + 0] != 1)
//         return false;
//     data[0] = BMI08X_ACCEL_PM_ACTIVE;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_PWR_CONF, data, 1);
//     port_delay_us(5 * 1000);
//     data[0] = BMI08X_ACCEL_POWER_ENABLE;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_PWR_CTRL, data, 1);
//     port_delay_us(5 * 1000);
//     data[0] = BMI08X_GYRO_PM_NORMAL;
//     port_spi_write_mem(port, gyro_dev_num, BMI08X_REG_GYRO_LPM1, data, 1);
//     port_delay_us(30 * 1000);

//     data[0] = (BMI08X_ACCEL_BW_NORMAL << 4) | BMI08X_ACCEL_ODR_800_HZ;
//     data[1] = BMI088_ACCEL_RANGE_24G;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_CONF, data, 2);
//     data[0] = BMI08X_GYRO_RANGE_2000_DPS;
//     data[1] = BMI08X_GYRO_BW_116_ODR_1000_HZ;
//     port_spi_write_mem(port, gyro_dev_num, BMI08X_REG_GYRO_RANGE, data, 2);

//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_FEATURE_CFG, data, DUMY_OFFSET + 6);
//     data[DUMY_OFFSET + 4] = BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ & 0xFF;
//     data[DUMY_OFFSET + 5] = BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ >> 8;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_FEATURE_CFG, &data[DUMY_OFFSET], 6);
//     port_delay_us(100 * 1000);

//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_INT1_IO_CONF, data, DUMY_OFFSET + 1);
//     data[DUMY_OFFSET + 0] &= ~(BMI08X_ACCEL_INT_EDGE_MASK | BMI08X_ACCEL_INT_LVL_MASK | BMI08X_ACCEL_INT_OD_MASK | BMI08X_ACCEL_INT_IO_MASK | BMI08X_ACCEL_INT_IN_MASK);
//     data[DUMY_OFFSET + 0] |= (BMI08X_INT_ACTIVE_HIGH << BMI08X_ACCEL_INT_LVL_POS) |
//                              (BMI08X_INT_MODE_PUSH_PULL << BMI08X_ACCEL_INT_OD_POS) |
//                              (BMI08X_ENABLE << BMI08X_ACCEL_INT_EDGE_POS) |
//                              (BMI08X_ENABLE << BMI08X_ACCEL_INT_IN_POS) |
//                              (BMI08X_DISABLE << BMI08X_ACCEL_INT_IO_POS);
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_INT1_IO_CONF, &data[DUMY_OFFSET], 1);

//     data[0] = BMI08X_ACCEL_INTA_ENABLE;
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_INT2_MAP, data, 1);

//     port_spi_read_mem(port, accel_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_INT2_IO_CONF, data, DUMY_OFFSET + 1);
//     data[DUMY_OFFSET + 0] &= ~(BMI08X_ACCEL_INT_LVL_MASK | BMI08X_ACCEL_INT_OD_MASK | BMI08X_ACCEL_INT_IO_MASK | BMI08X_ACCEL_INT_IN_MASK);
//     data[DUMY_OFFSET + 0] |= (BMI08X_INT_ACTIVE_HIGH << BMI08X_ACCEL_INT_LVL_POS) |
//                              (BMI08X_INT_MODE_PUSH_PULL << BMI08X_ACCEL_INT_OD_POS) |
//                              (BMI08X_DISABLE << BMI08X_ACCEL_INT_IN_POS) |
//                              (BMI08X_ENABLE << BMI08X_ACCEL_INT_IO_POS);
//     port_spi_write_mem(port, accel_dev_num, BMI08X_REG_ACCEL_INT2_IO_CONF, &data[DUMY_OFFSET], 1);

//     port_spi_read_mem(port, gyro_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_GYRO_INT3_INT4_IO_MAP, data, 1);
//     data[0] &= ~(BMI08X_GYRO_INT3_MAP_MASK | BMI08X_GYRO_INT4_MAP_MASK);
//     data[0] |= (BMI08X_ENABLE << BMI08X_GYRO_INT3_MAP_POS) | (BMI08X_DISABLE << BMI08X_GYRO_INT4_MAP_POS);
//     port_spi_write_mem(port, gyro_dev_num, BMI08X_REG_GYRO_INT3_INT4_IO_MAP, data, 1);

//     port_spi_read_mem(port, gyro_dev_num, BMI08X_SPI_RD_MASK | BMI08X_REG_GYRO_INT3_INT4_IO_CONF, data, 1);
//     data[0] &= ~(BMI08X_GYRO_INT3_LVL_MASK | BMI08X_GYRO_INT3_OD_MASK | BMI08X_GYRO_INT4_LVL_MASK | BMI08X_GYRO_INT4_OD_MASK);
//     data[0] |= (BMI08X_INT_MODE_PUSH_PULL << BMI08X_GYRO_INT3_LVL_POS) |
//                (BMI08X_INT_ACTIVE_HIGH << BMI08X_GYRO_INT3_OD_POS) |
//                (BMI08X_INT_MODE_PUSH_PULL << BMI08X_GYRO_INT4_LVL_POS) |
//                (BMI08X_INT_ACTIVE_HIGH << BMI08X_GYRO_INT4_OD_POS);
//     port_spi_write_mem(port, gyro_dev_num, BMI08X_REG_GYRO_INT3_INT4_IO_CONF, data, 1);
//     data[0] = BMI08X_GYRO_DRDY_INT_DISABLE_VAL;
//     port_spi_write_mem(port, gyro_dev_num, BMI08X_REG_GYRO_INT_CTRL, data, 1);

//     return true;
// }



//accel dummy read chip id // BMI08X_REG_ACCEL_CHIP_ID
//accel read chip id // BMI08X_REG_ACCEL_CHIP_ID
//check id // BMI085_ACCEL_CHIP_ID // BMI088_ACCEL_CHIP_ID
//gyro read chip id // BMI08X_REG_GYRO_CHIP_ID
//check id // BMI08X_GYRO_CHIP_ID

//accel set reset // BMI08X_REG_ACCEL_SOFTRESET -> BMI08X_SOFT_RESET_CMD
//delay 1ms
//accel dummy read chip id // BMI08X_REG_ACCEL_CHIP_ID
//accel read chip id // BMI08X_REG_ACCEL_CHIP_ID
//check id // BMI085_ACCEL_CHIP_ID // BMI088_ACCEL_CHIP_ID

//load cfg
//bmi08x_config_file
//Dis pwr save mode // BMI08X_REG_ACCEL_PWR_CONF -> 0
//delay 450 us
//Dis cfg loading // BMI08X_REG_ACCEL_INIT_CTRL -> 0
//index = 0; index < BMI08X_CONFIG_STREAM_SIZE; index+=len
//uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
//uint8_t asic_lsb = ((index / 2) & 0x0F);
//write BMI08X_REG_ACCEL_RESERVED_5B -> lsb
//write BMI08X_REG_ACCEL_RESERVED_5C -> msb
//write BMI08X_REG_ACCEL_FEATURE_CFG -> stream_data+index, len
//En cfg loading // BMI08X_REG_ACCEL_INIT_CTRL -> 1
//delay BMI08X_ASIC_INIT_TIME_MS ms
//Check cfg // BMI08X_REG_ACCEL_INTERNAL_STAT == 1
//write BMI08X_REG_ACCEL_PWR_CONF -> BMI08X_ACCEL_PM_ACTIVE
//delay 5 ms
//write BMI08X_REG_ACCEL_PWR_CTRL -> BMI08X_ACCEL_POWER_ENABLE
//delay 5 ms
//gyro
//write BMI08X_REG_GYRO_LPM1 -> BMI08X_GYRO_PM_NORMAL
//delay 30 ms

/*
case BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ:
    dev->accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
    dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
    dev->gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
    dev->gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
    break;
case BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ:
    dev->accel_cfg.odr = BMI08X_ACCEL_ODR_800_HZ;
    dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
    dev->gyro_cfg.odr = BMI08X_GYRO_BW_116_ODR_1000_HZ;
    dev->gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;
    break;
case BMI08X_ACCEL_DATA_SYNC_MODE_400HZ:
    dev->accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;
    dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
    dev->gyro_cfg.odr = BMI08X_GYRO_BW_47_ODR_400_HZ;
    dev->gyro_cfg.bw = BMI08X_GYRO_BW_47_ODR_400_HZ;
    break;
*/
//accel
//data[0] = (BMI08X_ACCEL_BW_NORMAL << 4) | BMI08X_ACCEL_ODR_800_HZ
//data[1] = BMI088_ACCEL_RANGE_24G
//write BMI08X_REG_ACCEL_CONF -> data, 2
//gyro
//write BMI08X_REG_GYRO_BANDWIDTH -> BMI08X_GYRO_BW_116_ODR_1000_HZ
//write BMI08X_REG_GYRO_RANGE -> BMI08X_GYRO_RANGE_2000_DPS

//accel
//read BMI08X_REG_ACCEL_FEATURE_CFG -> data, 6
//data[4] = BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ & 0xFF;
//data[5] = BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ >> 8;
//write BMI08X_REG_ACCEL_FEATURE_CFG -> data, 6
//delay 100 ms

//ints
//accel 1
//read BMI08X_REG_ACCEL_INT1_IO_CONF -> data
//data &= ~(BMI08X_ACCEL_INT_EDGE_MASK | BMI08X_ACCEL_INT_LVL_MASK | BMI08X_ACCEL_INT_OD_MASK | BMI08X_ACCEL_INT_IO_MASK | BMI08X_ACCEL_INT_IN_MASK)
//data = (BMI08X_INT_ACTIVE_HIGH << BMI08X_ACCEL_INT_LVL_POS) |
//       (BMI08X_INT_MODE_PUSH_PULL << BMI08X_ACCEL_INT_OD_POS) |
//       (BMI08X_ENABLE << BMI08X_ACCEL_INT_EDGE_POS) |
//       (BMI08X_ENABLE << BMI08X_ACCEL_INT_IN_POS) |
//       (0 << BMI08X_ACCEL_INT_IO_POS)
//write BMI08X_REG_ACCEL_INT1_IO_CONF -> data

//accel 2
//write BMI08X_REG_ACCEL_INT2_MAP -> BMI08X_ACCEL_INTA_ENABLE
//read BMI08X_REG_ACCEL_INT2_IO_CONF -> data
//data &= ~(BMI08X_ACCEL_INT_LVL_MASK | BMI08X_ACCEL_INT_OD_MASK | BMI08X_ACCEL_INT_IO_MASK | BMI08X_ACCEL_INT_IN_MASK)
//data |= (BMI08X_INT_ACTIVE_HIGH << BMI08X_ACCEL_INT_LVL_POS) |
//       (BMI08X_INT_MODE_PUSH_PULL << BMI08X_ACCEL_INT_OD_POS) |
//       (0 << BMI08X_ACCEL_INT_IN_POS) |
//       (BMI08X_ENABLE << BMI08X_ACCEL_INT_IO_POS)
//write BMI08X_REG_ACCEL_INT2_IO_CONF -> data

//gyro
//read BMI08X_REG_GYRO_INT3_INT4_IO_MAP -> data
//data = BMI08X_ENABLE << BMI08X_GYRO_INT3_MAP_POS
//data = BMI08X_DISABLE << BMI08X_GYRO_INT4_MAP_POS
//write BMI08X_REG_GYRO_INT3_INT4_IO_MAP -> data
//read BMI08X_REG_GYRO_INT3_INT4_IO_CONF -> data
//data &= ~(BMI08X_GYRO_INT3_LVL_MASK | BMI08X_GYRO_INT3_OD_MASK | BMI08X_GYRO_INT4_LVL_MASK | BMI08X_GYRO_INT4_OD_MASK)
//data |= (BMI08X_INT_MODE_PUSH_PULL << BMI08X_GYRO_INT3_LVL_POS) |
//        (BMI08X_INT_ACTIVE_HIGH << BMI08X_GYRO_INT3_OD_POS) |
//        (BMI08X_INT_MODE_PUSH_PULL << BMI08X_GYRO_INT4_LVL_POS) |
//        (BMI08X_INT_ACTIVE_HIGH << BMI08X_GYRO_INT4_OD_POS)
//write BMI08X_REG_GYRO_INT3_INT4_IO_CONF -> data
//write BMI08X_REG_GYRO_INT_CTRL -> BMI08X_GYRO_DRDY_INT_DISABLE_VAL



#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* COMMON_H */
