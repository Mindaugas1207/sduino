
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
    i2c_mux_inst_t I2C_Mux;
    motor_driver_hw_inst_t MotorDriverA, MotorDriverB;
    encoder_hw_inst_t EncoderA, EncoderB;
    esc_hw_inst_t ESC0;
    imu_hw_inst_t IMU0;
    line_sensor_hw_inst_t LineSensor0;
    VL53L0X_Dev_t DistanceSensor0;
};

struct LineFollowerSysConfig
{
    float M_Speed;
    float ESC_Speed;
    float Kp;
    float Kd;
    float Max_Speed;
    float Ramp_Speed;
    float Ramp_SpeedDown;
    uint Wall_Th;
    float Wall_Speed;
    float Wall_Angle;
    uint Wall_time1;
    uint Wall_time2;
    uint loop_time;
};

struct LineFollowerConfig
{
    MotorDriver::Config MotorDriverA, MotorDriverB;
    Encoder<double>::Config EncoderA, EncoderB;
    ESC::Config ESC0;
    IMU<double>::Config IMU0;
    LineSensor<double>::Config LineSensor0;
    LineFollowerSysConfig LFCFG;
    //-----------------------------------//
    uint64_t LockCode;
};



struct LineFollowerSys
{
    bool Start;
    bool Stop;
    bool Sleep;
    bool ESC_Start;
    bool NVM_Erase_OK;
    bool NVM_Load_OK;
    bool NVM_ReLoad_OK;
    bool NVM_Save_OK;
    bool Plyta_doing;
    bool Plyta_done;

    LineFollowerSysConfig Config;
};

int SduinoInit(const SduinoConfig& config);
void Init(void);

void ESC_Start(void);
void ESC_Stop(void);

void Start(void);
void Stop(void);
void Wakeup(void);
void Sleep(void);

void SaveConfig(void);
void LoadConfig(void);
void ReloadConfig(void);
void EraseConfig(void);
int SetVariable(int _Enum, float _Value);
std::tuple<int, float> GetVariable(int _Enum);

constexpr auto ESC_PWM_PIN = 29U;
constexpr auto MOTOR_B_ENCODER_A_PIN = 23U;
constexpr auto MOTOR_B_ENCODER_B_PIN = 24U;
constexpr auto MOTOR_A_ENCODER_A_PIN = 1U;
constexpr auto MOTOR_A_ENCODER_B_PIN = 2U;
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
inline DistanceSensor DistanceSensor0;

inline LineFollowerConfig Config0;
inline LineFollowerSys LFSYS;

#endif
