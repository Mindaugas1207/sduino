/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef COMMON_H
#define COMMON_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "bmi08x.h"
#include "port.h"

typedef struct {
    struct bmi08x_dev device;
    struct bmi08x_data_sync_cfg sync_config;
    struct bmi08x_int_cfg int_config;
    struct bmi08x_sensor_data accel;
    struct bmi08x_sensor_data gyro;
    port_spi_dev_t gyro_port;
    port_spi_dev_t accel_port;
} BMI08x_inst_t;

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
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

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
BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

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
void bmi08x_delay_us(uint32_t period, void *intf_ptr);

int BMI088_init(BMI08x_inst_t *inst, port_spi_t *gyro_port, port_spi_t *accel_port, uint gyro_dev_num, uint accel_dev_num);

int BMI088_ReadData(BMI08x_inst_t *inst);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* COMMON_H */
