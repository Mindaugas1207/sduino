
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
    .I2C_Mux = {
        .i2c = i2c_internal,
        .address = PI4MSD5V9540B_DEFAULT_ADDRESS
    },
    .MotorDriverA = {
        .PinA = SDUINO_INTERNAL_DRV_A_IN1_PIN,
        .PinB = SDUINO_INTERNAL_DRV_A_IN2_PIN,
        .Frequency = 125000000,
        .Period = 0xFFFF
    },
    .MotorDriverB = {
        .PinA = SDUINO_INTERNAL_DRV_B_IN1_PIN,
        .PinB = SDUINO_INTERNAL_DRV_B_IN2_PIN,
        .Frequency = 125000000,
        .Period = 0xFFFF
    },
    .EncoderA = {
        .Pio = pio1,
        .Pio_sm = 0,
        .PinA = MOTOR_A_ENCODER_A_PIN,
        .PinB = MOTOR_A_ENCODER_B_PIN
    },
    .EncoderB = {
        .Pio = pio1,
        .Pio_sm = 1,
        .PinA = MOTOR_B_ENCODER_A_PIN,
        .PinB = MOTOR_B_ENCODER_B_PIN
    },
    .ESC0 = {
        .Pin = ESC_PWM_PIN,
        .Frequency = 1000000,
        .Period = 20000
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
    },
    .LineSensor0 = {
        .spi_adc_hw = {
            .Pio = pio0,
            .Pio_sm_0 = 0,
            .Pio_sm_1 = 1,
            .DMA_ch = 0,
            .Pin_CLK = 3,
            .Pin_DATA = 4,
            .Pin_CNV = 5,
            .Pin_RESET = 6,
            .fspi = 1000000,//20000000,
            .fcnv = 1000
        },
        .emitter_hw = {
            .port_device = {
                .hw_intf = &I2C_Mux,
                .hw_intf_index = I2C_MUX_CHANNEL_0,
                .hw_address = IS31FL3218_ADDRESS,
                .hw_type = PORT_I2C_MUX
            }
        },
        .led_hw = {
            .port_device = {
                .hw_intf = &I2C_Mux,
                .hw_intf_index = I2C_MUX_CHANNEL_1,
                .hw_address = IS31FL3218_ADDRESS,
                .hw_type = PORT_I2C_MUX
            }
        }
    }
};

inline const LineFollowerConfig DefaultConfig = {
    .MotorDriverA = {
        .Min = 0,
        .Max = 1
    },
    .MotorDriverB = {
        .Min = 0,
        .Max = 1
    },
    .EncoderA = {
        .SamplingPeriod = 100 * 1000,
        .RPM_min = 100,
        .RPM_max = 3000,
        .PPR = 7,
        .Reduction = 4,
        .Oersampling = 4,
        .WheelDiameter = 22.0 / 1000,
    },
    .EncoderB = {
        .SamplingPeriod = 100 * 1000,
        .RPM_min = 100,
        .RPM_max = 3000,
        .PPR = 7,
        .Reduction = 4,
        .Oersampling = 4,
        .WheelDiameter = 22.0 / 1000,
    },
    .ESC0 = {
        .RampUpPeriod = 200 * 1000,
        .Min = 0.05,
        .Max = 0.10,
        .Power = 0.2,
        .StartingPower = 0.1
    },
    .IMU0 = {
        .CalibrationTime = 5 * 1000 * 1000,
        .FilterBeta = 0.01,
        .Calibrated = false
    },
    .LineSensor0 = {
        .CalibrationTime = 4 * 1000 * 1000,
        .Calibrated = false
    }
};

#endif
