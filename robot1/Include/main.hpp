
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
    i2c_mux_inst_t I2C_Mux;
    motor_driver_hw_inst_t MotorDriverA, MotorDriverB;
    encoder_hw_inst_t EncoderA, EncoderB;
    esc_hw_inst_t ESC0;
    imu_hw_inst_t IMU0;
    line_sensor_hw_inst_t LineSensor0;
};

struct LineFollowerConfig
{
    MotorDriver::Config MotorDriverA, MotorDriverB;
    Encoder<double>::Config EncoderA, EncoderB;
    ESC::Config ESC0;
    IMU<double>::Config IMU0;
    LineSensor<double>::Config LineSensor0;
    //-----------------------------------//
    uint64_t LockCode;
};

int SduinoInit(const SduinoConfig& config);
void Init(void);

void SaveConfig(void);
void LoadConfig(void);
void ReloadConfig(void);
void EraseConfig(void);
int SetVariable(int _Enum, float _Value);
std::tuple<int, float> GetVariable(int _Enum);

constexpr auto ESC_PWM_PIN = 29U; //pwm5 A
constexpr auto MOTOR_B_ENCODER_A_PIN = 23U; //pwm0 b
constexpr auto MOTOR_B_ENCODER_B_PIN = 24U; //pwm1 b
constexpr auto MOTOR_A_ENCODER_A_PIN = 1U; // reik pakeist i 23 pwm3 b
constexpr auto MOTOR_A_ENCODER_B_PIN = 2U; //pwm4 b reik pakeist i 24 pwm4 a
constexpr auto NVM_CONFG_LOCK_CODE = 0xACE9FBD117E3B907;

inline NVM_s NVM;
inline Interface_s Interface;

inline i2c_mux_inst_t I2C_Mux;
inline LED LED0;
inline MotorDriver MotorDriverA, MotorDriverB;
inline Encoder<double> EncoderA, EncoderB;
inline ESC ESC0;
inline IMU<double> IMU0;
inline LineSensor<double> LineSensor0;

inline LineFollowerConfig Config0;

#endif
