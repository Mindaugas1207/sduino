
#ifndef INC_MOTOR_DRIVER_HPP_
#define INC_MOTOR_DRIVER_HPP_

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <algorithm>

#define MOTOR_DRIVER_OK 0
#define MOTOR_DRIVER_ERROR -1

#define DEFAULT_PWM_WRAP_VALUE 0xFFFF
#define DEFAULT_PWM_CLK_DIV 1.0f
//3.8f

#define SERVO_PWM_CLKDIV 125.0f
#define SERVO_PWM_PERIOD 20000U
#define SERVO_PWM_MIN 1000
#define SERVO_PWM_MAX 2000

class MotorDriver_s {
    struct Config_s {
        uint PWM_PIN1;
        uint PWM_PIN2;
        uint PWM_WRAP_VALUE;
        float PWM_CLK_DIV;
        float Bias;
        float DeadZone;
    };

    uint PWM_PIN1;
    uint PWM_PIN2;
    uint PWM_WRAP_VALUE;
    float PWM_CLK_DIV;

    uint PWM1;
    uint PWM2;
    float Duty;
    float Bias;
    float DeadZone;
    uint16_t PWM1_Value;
    uint16_t PWM2_Value;
    bool Enabled;
public:
    typedef Config_s Config_t;

    int init(uint _PWM_PIN1, uint _PWM_PIN2, uint _PWM_WRAP_VALUE, float _PWM_CLK_DIV);
    int init(uint _PWM_PIN1, uint _PWM_PIN2);
    
    void setPWM_PIN1(uint _PWM_PIN1);
    uint getPWM_PIN1() { return PWM_PIN1; }
    void setPWM_PIN2(uint _PWM_PIN2);
    uint getPWM_PIN2() { return PWM_PIN2; }
    void setPWM_WRAP_VALUE(uint _PWM_WRAP_VALUE);
    uint getPWM_WRAP_VALUE() { return PWM_WRAP_VALUE; }
    void setPWM_CLK_DIV(float _PWM_CLK_DIV);
    float getPWM_CLK_DIV() { return PWM_CLK_DIV; }

    void setDuty(float _Duty);
    float getDuty() { return Duty; }
    void setBias(float _Bias) { Bias = _Bias; };
    float getBias() { return Bias; }
    void setDeadZone(float _DeadZone) { DeadZone = _DeadZone; };
    float getDeadZone() { return DeadZone; }

    void loadConfiguration(Config_t _Config);
    Config_t getConfiguration();

    void start();
    void stop();
    void brake();
    void process(absolute_time_t _TimeNow);
};

class ESC_s {
    uint PWM_PIN;
    uint PWM;
    float Speed;
    float StartSpeed;
    uint PWM_Value;
    uint PWM_Value_Start;
    bool Enabled;
    bool SpinUp;
    uint32_t SpinUpTime;
    absolute_time_t SpinUpTimeStart;
public:
    int init(uint _PWM_PIN);
    void setSpeed(float _Speed);
    float getSpeed() { return Speed; };
    void start(float _StartSpeed, int _SpintUpTime);
    void stop();
    void process(absolute_time_t _TimeNow);
};

#endif
