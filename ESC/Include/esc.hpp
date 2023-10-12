
#ifndef INC_ESC_HPP_
#define INC_ESC_HPP_

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <algorithm>

#define SERVO_PWM_CLKDIV 125.0f
#define SERVO_PWM_PERIOD 20000U
#define SERVO_PWM_MIN 1000
#define SERVO_PWM_MAX 2000

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
