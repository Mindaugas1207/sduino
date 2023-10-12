
#include "esc.hpp"

int ESC_s::init(uint _PWM_PIN)
{
    PWM_PIN = _PWM_PIN;
    gpio_set_function(_PWM_PIN, GPIO_FUNC_PWM);
    PWM = pwm_gpio_to_slice_num(_PWM_PIN);
    pwm_set_clkdiv(PWM, SERVO_PWM_CLKDIV);
    pwm_set_wrap(PWM, SERVO_PWM_PERIOD);
    pwm_set_gpio_level(PWM_PIN, SERVO_PWM_MIN);
    pwm_set_enabled(PWM, true);
    SpinUp = false;
    Enabled = false;
    Speed = 0.0f;
    StartSpeed = 0.0f;
    SpinUpTime = 1;
    return true;
}

void ESC_s::setSpeed(float _Speed)
{
    Speed = _Speed;
}

// void ESC_s::setAngle(float _Angle)
// {
//     uint pwm = SERVO_PWM_MIN * (_Angle / 180.0f) + SERVO_PWM_MIN;
//     pwm = pwm > SERVO_PWM_MAX ? SERVO_PWM_MAX : (pwm < SERVO_PWM_MIN ? SERVO_PWM_MIN : pwm);
//     pwm_set_gpio_level(Out_pin, pwm);
// }

void ESC_s::start(float _StartSpeed, int _SpintUpTime)
{
    if (Enabled) return;
    SpinUpTimeStart = get_absolute_time();
    SpinUpTime = _SpintUpTime;
    SpinUp = true;
    StartSpeed = _StartSpeed;
    Enabled = true;
}

void ESC_s::stop()
{
    pwm_set_gpio_level(PWM_PIN, SERVO_PWM_MIN);
    Enabled = false;
}

void ESC_s::process(absolute_time_t _TimeNow)
{
    if (Enabled)
    {
        float prop = 0.0f;

        if (SpinUp)
        {
            uint64_t diff = to_us_since_boot(_TimeNow) - to_us_since_boot(SpinUpTimeStart);

            if (diff < SpinUpTime)
            {
                prop = (((float)diff / SpinUpTime) * (Speed - StartSpeed)) + StartSpeed;
            }
            else
            {
                SpinUp = false;
            }
        }
        else
        {
            prop = Speed;
        }


        float tmp = std::clamp(prop, StartSpeed, Speed);
        int32_t pwm = (SERVO_PWM_MAX - SERVO_PWM_MIN) * tmp + SERVO_PWM_MIN;
        pwm = pwm > SERVO_PWM_MAX ? SERVO_PWM_MAX : (pwm < SERVO_PWM_MIN ? SERVO_PWM_MIN : pwm);
        PWM_Value = pwm;
        pwm_set_gpio_level(PWM_PIN, PWM_Value);
    }
}