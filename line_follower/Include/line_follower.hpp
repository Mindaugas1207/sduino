
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
#include "sensorx.h"
#include "esc.hpp"

#include "functional"
#include <algorithm>
#include <string>

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
    bool Start_ESC = false;
    bool Stop_ESC = false;
    bool SleepAllowed = false;

    int iX = 0;
    int iY = 0;

    float iX_f = 0.0f;
    float iY_f = 0.0f;
    float iYaw = 0.0f;

    int iPulseR = 0;
    int iPulseL = 0;
    float iSpeedR = 0.0f;
    float iSpeedL = 0.0f;

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
        CMD_ESC_ARM                       = 0x000A,
        CMD_ESC_START                     = 0x000B,
        CMD_ESC_STOP                      = 0x000C,
        VAR_BAT_VOLTAGE                   = 0x000D,
        /*  Common variables              = 0x0100,*/
        VAR_SPEED                         = 0x0100,
        VAR_ESC_SPEED                     = 0x0101,
        VAR_STEERING_GAIN                 = 0x0102,
        VAR_STEERING_GAIN2                = 0x0103,
        VAR_STEERING_GAIN3                = 0x0104,
        VAR_STEERING_GAIN4                = 0x0105,
        VAR_STEERING_GAIN5                = 0x0106,
        VAR_STEERING_GAIN6                = 0x0107,
        VAR_CRUISE_GAIN                   = 0x0108,
        VAR_CRUISE_GAIN2                  = 0x0109,
        VAR_CRUISE_GAIN3                  = 0x010A,
        VAR_CRUISE_GAIN4                  = 0x010B,
        VAR_CRUISE_GAIN5                  = 0x010C,
        VAR_CRUISE_GAIN6                  = 0x010D,
        VAR_BRAKE_TIMEOUT                 = 0x010E,
        /*  IMU variables                 = 0x0200,*/
        /*  Line variables                = 0x0220,*/
        VAR_LINE_EMITTER_POWER            = 0x0220,
        VAR_LINE_LED_BRIGHTNESS           = 0x0221,
        VAR_LINE_TURN_GAIN_1              = 0x0222,
        VAR_LINE_TURN_GAIN_2              = 0x0223,
        CMD_LINE_TOGGLE_DISPLAY           = 0x0224,
        CMD_LINE_TOGGLE_LIVE              = 0x0225,
        VAR_LINE_OFFSET_DECAY             = 0x0226,
        VAR_LINE_AVERAGE_DECAY            = 0x0227,
        VAR_LINE_ANALOG_WIDTH             = 0x0228,
        VAR_LINE_OFFSET_THRESHOLD         = 0x0229,
        VAR_LINE_LEFT_THRESHOLD           = 0x022A,
        VAR_LINE_RIGHT_THRESHOLD          = 0x022B,
        VAR_LINE_CENTER_TIMEOUT           = 0x022C,
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
        VAR_PID0_GAIN                     = 0x0260,
        VAR_PID0_INTEGRAL_TIME            = 0x0261,
        VAR_PID0_INTEGRAL_LIMIT           = 0x0262,
        VAR_PID0_INTEGRAL_RATE_LIMIT      = 0x0263,
        VAR_PID0_DERIVATIVE_TIME          = 0x0264,
        VAR_PID0_DERIVATIVE_CUTOFF        = 0x0265,
        VAR_PID0_DEADZONE                 = 0x0266,

        CMD_FIRST                         = CMD_NONE,
        CMD_LAST                          = CMD_LINE_WAKEUP,
        VAR_FIRST                         = VAR_SPEED,
        VAR_LAST                          = VAR_PID0_DEADZONE,
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
        PID_s::Config_t PID_Drives;
        float ForwardSpeed;
        float EscSpeed;
        float SteeringGain;
        float CruiseGain;
        float SteeringGain2;
        float CruiseGain2;
        float SteeringGain3;
        float CruiseGain3;
        float SteeringGain4;
        float CruiseGain4;
        float SteeringGain5;
        float CruiseGain5;
        float SteeringGain6;
        float CruiseGain6;
        uint32_t BrakeTimeout;
        uint32_t LockCode;
        void setLockCode(uint32_t _LockCode) { LockCode = _LockCode; }
        uint32_t getLockCode() { return LockCode; }
    };
    absolute_time_t PrintTime;
    absolute_time_t SleepTime;
    absolute_time_t StartTime;
    absolute_time_t BrakeTime;
    bool ReturningOnLine = false;
    bool BrakeStart = false;
    bool BrakeEnd = false;
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

    void ESC_Start();
    void ESC_Stop();

    void sleep(void);
    void wakeup(void);

    void run(void);
    void computeControl(absolute_time_t _TimeNow);

    void motorControl(absolute_time_t _TimeNow);

    // #define xs(x) __XSTRING(x)

    // #define STRINGIFY_IMPL(s) #s
    // #define STRINGIFY(s) STRINGIFY_IMPL(s)
    // #define ARG1_IMPL(a, ...) a
    // #define ARG1(...) ARG1_IMPL(__VA_ARGS__, 0)
    // #define DumpStr(...) DumpString(STRINGIFY(ARG1(__VA_ARGS__)),__VA_ARGS__)
    
    // void DumpString(const char* varname, char* var, int optionalvar=0) {
    //     printf("%s : '%s'\n", varname, var);
    //     printf("blah: %d", optionalvar);
    // }

    // int rangeAdd(char * File, const char * VarName, uint VarId, const char * Min, const char * Max, const char * Step)
    // {
    //     return sprintf(File, "<div class=\"range-wrap\" id=\"%X\">"
    //                          "<span class =\"label\">%s</span>"
	// 	                     "<input type=\"range\" class=\"range\" min=\"%s\" max=\"%s\" value=\"0\" step=\"%s\">"
	// 	                     "<output class=\"bubble\">"
    //                          "</output></div>",
    //                          VarId, VarName, Min, Max, Step);
    // }

    // #define RANGE_ADD(File, VarName, Min, Max, Step) sprintf(File, "<div class=\"range-wrap\" id=\"%X\">"\
    //                                                                "<span class =\"label\">" xs(VarName) "</span>"\
    //                                                                "<input type=\"range\" class=\"range\" min=\"" xs(Min) "\" max=\"" xs(Max) "\" value=\"0\" step=\"" xs(Step) "\">"\
    //                                                                "<output class=\"bubble\">"\
    //                                                                "</output></div>", VarName)

    //#define RANGE_STRING()

    // int tabBegin(char * File, const char * TabName)
    // {
    //     return sprintf(File, "<div id=\"%s\" class=\"tabcontent\">", TabName);
    // }

    // int tabEnd(char * File)
    // {
    //     return sprintf(File, "</div>");
    // }

    // void interfaceFileConstruct()
    // {
    //     char buffer[255];
    //     //tabBegin(buffer, "PID0");
    //     RANGE_ADD(buffer, 0, 0, 1, 0.001);
    //     //rangeAdd(buffer, __XSTRING(VAR_PID0_SETPOINT), VAR_PID0_SETPOINT, "0", "1", "0.001");
    //     tabEnd(buffer);
    // }
    // constexpr auto ccatc(std::string, const char s2[])
    // {
    //     return cat;
    // }

    
    
    // void fileNewTabBegin(char file[], char TabName[])
    // {
    //     constexpr const char ff[] = "help";
    //     constexpr char stc[100];
    //     stc += ff;

    //     int n = sprintf(file, "<div id=\"%s\" class=\"tabcontent\">", TabName);
    // }
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

