
#ifndef INC_LINE_FOLLOWER_HPP_
#define INC_LINE_FOLLOWER_HPP_

#include "stdio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#include "nvm.hpp"
#include "interface.hpp"
#include "port.h"
#include "motor_driver.hpp"
#include "line_sensor.hpp"
#include "imu.hpp"
#include "pid.hpp"

#include "functional"
#include <algorithm>

class LineFollower_s
{
    const uint32_t NVM_CONFG_LOCK_CODE = 0xAABBCCDD;
    bool NVM_CONFIG_OK = false;
    bool Start = false;
    bool Stop = false;
    bool Awake = false;
    bool NVM_LOAD_OK = false;
    bool NVM_SAVE_OK = false;
    bool NVM_ERASE_OK = false;

    enum InterfaceVariables{
    //HEX FORMAT 16bit
        /*  Commands variables            = 0x0000,*/
        CMD_NONE                          = 0x0000,
        CMD_START                         = 0x0001,
        CMD_STOP                          = 0x0002,
        CMD_LINE_CALIBRATE                = 0x0003,
        CMD_IMU_CALIBRATE                 = 0x0004,
        CMD_NVM_ERASE                     = 0x0005,
        CMD_NVM_LOAD                      = 0x0006,
        CMD_NVM_SAVE                      = 0x0007,
        CMD_LINE_SLEEP                    = 0x0008,
        CMD_LINE_WAKEUP                   = 0x0009,
        /*  Common variables              = 0x0100,*/
        VAR_SPEED                         = 0x0100,
        VAR_TURN_POS                      = 0x0101,
        VAR_TURN_NEG                      = 0x0102,
        VAR_TURN_BIAS_POS                 = 0x0103,
        VAR_TURN_BIAS_NEG                 = 0x0104,
        VAR_SPEED_DOWN                    = 0x0105,
        /*  IMU variables                 = 0x0200,*/
        /*  Line variables                = 0x0220,*/
        VAR_LINE_EMITTER_POWER            = 0x0220,
        VAR_LINE_LED_BRIGHTNESS           = 0x0221,
        VAR_LINE_TURN_GAIN_1              = 0x0222,
        VAR_LINE_TURN_GAIN_2              = 0x0223,
        /*  Drive variables               = 0x0240,*/
        VAR_DRIVE0_PWM_WRAP_VALUE         = 0x0240,
        VAR_DRIVE0_PWM_CLK_DIV            = 0x0241,
        VAR_DRIVE0_BIAS                   = 0x0242,
        VAR_DRIVE0_DEADZONE               = 0x0243,
        VAR_DRIVE1_PWM_WRAP_VALUE         = 0x0244,
        VAR_DRIVE1_PWM_CLK_DIV            = 0x0245,
        VAR_DRIVE1_BIAS                   = 0x0246,
        VAR_DRIVE1_DEADZONE               = 0x0247,
        /*  PID0 variables                = 0x0260,*/
        VAR_PID0_SETPOINT                 = 0x0260,
        VAR_PID0_GAIN                     = 0x0261,
        VAR_PID0_INTEGRAL_TIME            = 0x0262,
        VAR_PID0_INTEGRAL_LIMIT           = 0x0263,
        VAR_PID0_INTEGRAL_RATE_LIMIT      = 0x0264,
        VAR_PID0_INTEGRAL_ANTI_WINDUP     = 0x0265,
        VAR_PID0_DERIVATIVE_TIME          = 0x0266,
        VAR_PID0_DERIVATIVE_CUTOFF        = 0x0267,
        VAR_PID0_OUTPUT_LIMIT             = 0x0268,
        VAR_PID0_DEADZONE                 = 0x0269,
        /*  PID1 variables                = 0x0280,*/
        VAR_PID1_SETPOINT                 = 0x0280,
        VAR_PID1_GAIN                     = 0x0281,
        VAR_PID1_INTEGRAL_TIME            = 0x0282,
        VAR_PID1_INTEGRAL_LIMIT           = 0x0283,
        VAR_PID1_INTEGRAL_RATE_LIMIT      = 0x0284,
        VAR_PID1_INTEGRAL_ANTI_WINDUP     = 0x0285,
        VAR_PID1_DERIVATIVE_TIME          = 0x0286,
        VAR_PID1_DERIVATIVE_CUTOFF        = 0x0287,
        VAR_PID1_OUTPUT_LIMIT             = 0x0288,
        VAR_PID1_DEADZONE                 = 0x0289,
        /*  PID2 variables                = 0x02A0,*/
        VAR_PID2_SETPOINT                 = 0x02A0,
        VAR_PID2_GAIN                     = 0x02A1,
        VAR_PID2_INTEGRAL_TIME            = 0x02A2,
        VAR_PID2_INTEGRAL_LIMIT           = 0x02A3,
        VAR_PID2_INTEGRAL_RATE_LIMIT      = 0x02A4,
        VAR_PID2_INTEGRAL_ANTI_WINDUP     = 0x02A5,
        VAR_PID2_DERIVATIVE_TIME          = 0x02A6,
        VAR_PID2_DERIVATIVE_CUTOFF        = 0x02A7,
        VAR_PID2_OUTPUT_LIMIT             = 0x02A8,
        VAR_PID2_DEADZONE                 = 0x02A9,
        /*  PID3 variables                = 0x02C0,*/
        VAR_PID3_SETPOINT                 = 0x02C0,
        VAR_PID3_GAIN                     = 0x02C1,
        VAR_PID3_INTEGRAL_TIME            = 0x02C2,
        VAR_PID3_INTEGRAL_LIMIT           = 0x02C3,
        VAR_PID3_INTEGRAL_RATE_LIMIT      = 0x02C4,
        VAR_PID3_INTEGRAL_ANTI_WINDUP     = 0x02C5,
        VAR_PID3_DERIVATIVE_TIME          = 0x02C6,
        VAR_PID3_DERIVATIVE_CUTOFF        = 0x02C7,
        VAR_PID3_OUTPUT_LIMIT             = 0x02C8,
        VAR_PID3_DEADZONE                 = 0x02C9,

        CMD_FIRST                         = CMD_NONE,
        CMD_LAST                          = CMD_LINE_WAKEUP,
        VAR_FIRST                         = VAR_SPEED,
        VAR_LAST                          = VAR_PID3_DEADZONE,
        INTERFACE_ENUM_FIRST              = CMD_FIRST,
        INTERFACE_ENUM_LAST               = VAR_LAST + 1,
        INTERFACE_ENUM_SIZE               = VAR_LAST
    };

    struct Config_s
    {
        uint32_t StateFlags;
        IMU_s::Config_t IMU;
        LineSensor_s::Config_t LineSensor;
        MotorDriver_s::Config_t DriveA;
        MotorDriver_s::Config_t DriveB;
        PID_s::Config_t PID_DriveA;
        PID_s::Config_t PID_DriveB;
        PID_s::Config_t PID_Cruise;
        PID_s::Config_t PID_Steering;
        float ForwardSpeed;
        float TurnBiasPositive;
        float TurnBiasNegative;
        float TurnMaxPositive;
        float TurnMaxNegative;
        float SpeedDown;
        uint32_t LockCode;
        void setLockCode(uint32_t _LockCode) { LockCode = _LockCode; }
        uint32_t getLockCode() { return LockCode; }
    };
    absolute_time_t PrintTime;
    absolute_time_t SleepTime;
    int EncoderPulseCountA;
    int EncoderPulseCountB;
    float EncoderSpeedA;
    float EncoderSpeedB;
public:
    Config_s Config;

    std::tuple<int, float> get(int _Enum);
    int set(int _Enum, float _Value);

    const std::function<std::tuple<int, float>(int)> f_getCallback = [this](int _Enum) { return this->get(_Enum); };
    const std::function<int(int, float)> f_setCallback = [this](int _Enum, float _Value) { return this->set(_Enum, _Value); };

    void init(void);
    void save(void);
    void load(void);
    void erase(void);
    void start(void);
    void stop(void);
    void stop_error(void);

    void sleep(void);
    void wakeup(void);

    void run(void);
    void computeControl(absolute_time_t _TimeNow);

    void motorControl(absolute_time_t _TimeNow);
};

class Blink_s
{
    uint Pin;
    bool Enabled;
    bool State;
    uint32_t OnTime;
    uint32_t OffTime;
    absolute_time_t Timeout;
public:
    void init(uint _Pin)
    {
        Pin = _Pin;
        gpio_init(Pin);
        gpio_set_dir(Pin, GPIO_OUT);
    }

    void start(uint32_t _OnTime, uint32_t _OffTime)
    {
        OnTime = _OnTime;
        OffTime = _OffTime;
        State = false;
        Enabled = true;
        gpio_put(Pin, false);
        Timeout = make_timeout_time_ms(OffTime);
    }

    void stop()
    {
        State = false;
        Enabled = false;
        gpio_put(Pin, false);
    }

    void process(absolute_time_t _TimeNow)
    {
        if (Enabled && to_us_since_boot(_TimeNow) > to_us_since_boot(Timeout))
        {
            if (State)
            {
                gpio_put(Pin, false);
                State = false;
                Timeout = delayed_by_ms(_TimeNow, OffTime);
            }
            else
            {
                gpio_put(Pin, true);
                State = true;
                Timeout = delayed_by_ms(_TimeNow, OnTime);
            }
            
        }
    }
};

#endif

