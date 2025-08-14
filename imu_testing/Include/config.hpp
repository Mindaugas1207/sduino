
#ifndef INC_CONFIG_HPP_
#define INC_CONFIG_HPP_

#include "main.hpp"

inline const HardwareConfig HWConfig = {
    .Sduino = {
        .I2C_BaudRate  = 400 * 1000,
        .SPI_BaudRate  = 10 * 1000 * 1000,
        .UART_BaudRate = 115200,
        .LED_Pin = SDUINO_INTERNAL_LED_PIN
    },
    .IMU0 = {
        .BMI08X_Device = {
            .intf_accel = {
                .hw_intf = spi_internal,
                .hw_intf_index = SDUINO_INTERNAL_IMU_ACCEL_CS_PIN,
                .hw_type = PORT_SPI
            },
            .intf_gyro  = {
                .hw_intf = spi_internal,
                .hw_intf_index = SDUINO_INTERNAL_IMU_GYRO_CS_PIN,
                .hw_type = PORT_SPI
            },
            .sync_mode = BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ,
        },
        .Pin = SDUINO_INTERNAL_IMU_INT_PIN
    }
};

inline const LineFollowerConfig DefaultConfig = {
    .IMU0 = {
        .CalibrationTime = 5 * 1000 * 1000,
        .FilterBeta = 0.02,
        .GyroBias = {0, 0, 0},
        .AccelBias = {0, 0, 0},
        .Calibrated = false
    }
};

#endif
