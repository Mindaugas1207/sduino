
#ifndef INC_MAIN_HPP_
#define INC_MAIN_HPP_

#include "stdio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#include "nvm.hpp"
#include "interface.hpp"
#include "port.h"
#include "motor_driver.hpp"
#include "imu.hpp"
#include "pid.hpp"
#include "sensorx.h"
#include "esc.hpp"
#include "encoder.hpp"
#include "hardware/adc.h"
#include "line_sensor_hw.h"
#include "line_sensor.hpp"
#include "distance_sensor.hpp"
#include "pico/util/queue.h"

#include "led.hpp"

#include "functional"
#include <algorithm>
#include <string>

struct SduinoConfig
{
    uint I2C_BaudRate;
    uint SPI_BaudRate;
    uint UART_BaudRate;
    uint LED_Pin;
};

struct HardwareConfig
{
    SduinoConfig Sduino;
    imu_hw_inst_t IMU0;
};

struct LineFollowerConfig
{
    IMU<double>::Config IMU0;
    //-----------------------------------//
    uint64_t LockCode;
};

int SduinoInit(const SduinoConfig& config);
void Init(void);

inline LED LED0;
inline IMU<double> IMU0;

#endif
