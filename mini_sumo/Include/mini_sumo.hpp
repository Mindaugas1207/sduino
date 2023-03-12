
#ifndef INC_MINI_SUMO_HPP_
#define INC_MINI_SUMO_HPP_

//#define XSHUT_1 29
#define XSHUT_1_PIN 23
#define XSHUT_2_PIN 24
#define XSHUT_3_PIN 25
#define STRT_PIN 1
#define KILL_PIN 29
#define NUM_SENSORS 4
#define SENSOR_INT_PIN 6
#define SERVO_PIN 5
#define EDGE_SENSOR_LEFT_PIN 2
#define EDGE_SENSOR_CENTER_PIN 4
#define EDGE_SENSOR_RIGHT_PIN 3

#include "stdio.h"
#include "pico/stdlib.h"

#include "nvm.hpp"
#include "interface.hpp"
#include "port.h"
#include "motor_driver.hpp"
#include "imu.hpp"
#include "pid.hpp"
#include "sensorx.h"

#include "functional"

class mini_sumo_s
{
    const uint32_t NVM_CONFG_LOCK_CODE = 0xAABBCCDD;
    bool NVM_CONFIG_OK = false;
    bool Start = false;
    bool Stop = false;
    bool Awake = false;

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
        VAR_TURN_BIAS                     = 0x0103,
        /*  Motor variables               = 0x0140,*/
        VAR_MOTORS_PMW_MAX                = 0x0140,
        VAR_MOTORS_CLK_DIV                = 0x0141,
        VAR_MOTORS_RAMP_STEP              = 0x0142,
        VAR_MOTORS_RAMP_SLOPE             = 0x0143,
        VAR_MOTORS_RAMP_STEP_TIME         = 0x0144,
        VAR_MOTORS_SPEED_MAX_POS          = 0x0145,
        VAR_MOTORS_SPEED_MAX_NEG          = 0x0146,
        /*  Line variables                = 0x0160,*/
        VAR_LINE_EMITTER_POWER            = 0x0160,
        VAR_LINE_LED_BRIGHTNESS           = 0x0161,
        VAR_LINE_DERIVATIVE_TIME          = 0x0162,
        VAR_LINE_TIMEOUT_TIME             = 0x0163,
        VAR_LINE_UPDATE_TIME              = 0x0164,
        VAR_LINE_CALIBRATION_TIME         = 0x0165,
        VAR_LINE_CHANGE_FILTER            = 0x0166,
        VAR_LINE_CHANGE_MULITPLIER        = 0x0167,
        VAR_LINE_FEATURE_TIME             = 0x0168,
        VAR_LINE_TURN_MAX                 = 0x0169,
        /*  PID0 variables                = 0x0180,*/
        VAR_PID0_MODE                     = 0x0180,
        VAR_PID0_KP                       = 0x0181,
        VAR_PID0_KD                       = 0x0182,
        VAR_PID0_KI                       = 0x0183,
        VAR_PID0_GAIN                     = 0x0184,
        VAR_PID0_OUTPUT_LIMIT             = 0x0185,
        VAR_PID0_SAMPLING_TIME            = 0x0186,
        VAR_PID0_INTEGRAL_TIME            = 0x0187,
        VAR_PID0_INTEGRAL_LIMIT           = 0x0188,
        VAR_PID0_INTEGRAL_RATE_LIMIT      = 0x0189,
        VAR_PID0_INTEGRAL_ANTI_WINDUP     = 0x018A,
        VAR_PID0_DERIVATIVE_TIME          = 0x018B,
        VAR_PID0_DERIVATIVE_CUTOFF        = 0x018C,
        VAR_PID0_SETPOINT                 = 0x018D,
        /*  IMU variables                 = 0x0200,*/
        VAR_IMU_YAW                       = 0x0200,

        CMD_FIRST                         = CMD_NONE,
        CMD_LAST                          = CMD_LINE_WAKEUP,
        VAR_FIRST                         = VAR_SPEED,
        VAR_LAST                          = VAR_IMU_YAW,
        INTERFACE_ENUM_FIRST              = CMD_FIRST,
        INTERFACE_ENUM_LAST               = VAR_LAST,
        INTERFACE_ENUM_SIZE               = VAR_LAST
    };

    struct Config_s
    {
        uint32_t StateFlags;
        PID_s::Config_t PID0;
        MotorDriver_s::Config_t MotorA;
        MotorDriver_s::Config_t MotorB;
        IMU_s::Config_t IMU;
        float ForwardSpeed;
        float TurnPositive;
        float TurnNegative;
        float TurnBias;
        uint32_t LockCode;
        void setLockCode(uint32_t _LockCode) { LockCode = _LockCode; }
        uint32_t getLockCode() { return LockCode; }
    };
    absolute_time_t PrintTime;
    absolute_time_t StartTimeout;
    bool RemoteStateChange = false;
    bool StartTimedOut = false;
    float yaw = 0.0f;
public:
    Config_s Config;

    std::tuple<float, bool> get(int _Enum);
    void set(int _Enum, float _Value);

    const std::function<std::tuple<float, bool>(int)> f_getCallback = [this](int _Enum) { return this->get(_Enum); };
    const std::function<void(int, float)> f_setCallback = [this](int _Enum, float _Value) { this->set(_Enum, _Value); };

    void init(void);
    void save(void);
    void load(void);

    void setSpeed(float Left, float Rigth);
    void start(void);
    void stop(void);
    void stop_error(void);
    
    void turnLeft(void);
    void turnRight(void);

    void sleep(void);
    void wakeup(void);

    void run(void);
    void computeControl(absolute_time_t _TimeNow);

    void getRemoteStart(void);
    int lineRead(void);
    //void vlx_multi_init(void);
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

    void set(bool _State)
    {
        State = _State;
        gpio_put(Pin, State);
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
