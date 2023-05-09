/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdio.h>
#include "bmi08_spi.h"

/******************************************************************************/
/*!                       Macro definitions                                   */

#define BMI08X_READ_WRITE_LEN  UINT8_C(46)

/*! BMI085 shuttle id */
#define BMI085_SHUTTLE_ID      UINT16_C(0x46)

/*! BMI088 shuttle id */
#define BMI088_SHUTTLE_ID      UINT16_C(0x66)

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * SPI read function map to COINES platform
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    port_spi_dev_t* intf = (port_spi_dev_t*)intf_ptr;
    (void)intf_ptr;
    return port_spi_read_mem(intf->port, intf->dev_num, reg_addr, reg_data, length);
}

/*!
 * SPI write function map to COINES platform
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    port_spi_dev_t* intf = (port_spi_dev_t*)intf_ptr;
    (void)intf_ptr;
    return port_spi_write_mem(intf->port, intf->dev_num, reg_addr, (uint8_t *)reg_data, length);
}

/*!
 * Delay function map to COINES platform
 */
void bmi08x_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    port_delay_us(period);
}

int BMI088_init(BMI08x_inst_t *inst, port_spi_t *gyro_port, port_spi_t *accel_port, uint gyro_dev_num, uint accel_dev_num)
{
    int status = BMI08X_OK;
    struct bmi08x_dev *dev_inst = &inst->device;

    inst->gyro_port.port = gyro_port;
    inst->accel_port.port = accel_port;
    inst->gyro_port.dev_num = gyro_dev_num;
    inst->accel_port.dev_num = accel_dev_num;

    dev_inst->intf = BMI08X_SPI_INTF;
    dev_inst->read = bmi08x_spi_read;
    dev_inst->write = bmi08x_spi_write;
    dev_inst->variant = BMI088_VARIANT;

    dev_inst->intf_ptr_accel = &inst->accel_port;
    dev_inst->intf_ptr_gyro = &inst->gyro_port;
    dev_inst->delay_us = bmi08x_delay_us;
    dev_inst->read_write_len = BMI08X_READ_WRITE_LEN;

    status |= bmi08a_init(dev_inst); //init accelerometer
    status |= bmi08g_init(dev_inst); //init gyro

    status |= bmi08a_soft_reset(dev_inst); //reset accelerometer

    dev_inst->read_write_len = 32; //Read/write length

    status |= bmi08a_load_config_file(dev_inst); //load cfg file

    dev_inst->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    status |= bmi08a_set_power_mode(dev_inst); //set accel power mode

    dev_inst->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
    status |= bmi08g_set_power_mode(dev_inst); //set gyro power mode

    dev_inst->accel_cfg.range = BMI088_ACCEL_RANGE_24G; //accel range
    dev_inst->gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS; //gyro range

    /* Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
    inst->sync_config.mode = BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ;

    status |= bmi08a_configure_data_synchronization(inst->sync_config, dev_inst);

    //interrupts

    inst->int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
    inst->int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
    inst->int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    inst->int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    inst->int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    inst->int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
    inst->int_config.accel_int_config_2.int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY;
    inst->int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    inst->int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    inst->int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    inst->int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
    inst->int_config.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
    inst->int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
    inst->int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    inst->int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    inst->int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
    inst->int_config.gyro_int_config_2.int_type = BMI08X_GYRO_INT_DATA_RDY;
    inst->int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
    inst->int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    inst->int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    status |= bmi08a_set_data_sync_int_config(&inst->int_config, dev_inst);

    return status;
}

int BMI088_ReadData(BMI08x_inst_t *inst)
{
    int status = BMI08X_OK;

    status |= bmi08a_get_synchronized_data(&inst->accel, &inst->gyro, &inst->device);

    return status;
}
