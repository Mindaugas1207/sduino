
#include "motor_driver.hpp"

int servo_hw_init(float _clkdiv, uint _pwm_max, uint _out_pin)
{
    gpio_set_function(_out_pin, GPIO_FUNC_PWM);
    uint out_slice = pwm_gpio_to_slice_num(_out_pin);
    pwm_set_clkdiv(out_slice, _clkdiv);
    pwm_set_wrap(out_slice, _pwm_max-1);
    pwm_set_enabled(out_slice, true);
    pwm_set_gpio_level(_out_pin, 0);

    return MOTOR_DRIVER_OK;
}

int MotorDriver_s::init(uint _PWM_PIN1, uint _PWM_PIN2, uint _PWM_WRAP_VALUE, float _PWM_CLK_DIV)
{
    setPWM_PIN1(_PWM_PIN1);
    setPWM_PIN2(_PWM_PIN2);
    setPWM_WRAP_VALUE(_PWM_WRAP_VALUE);
    setPWM_CLK_DIV(_PWM_CLK_DIV);
    Enabled = false;
    return MOTOR_DRIVER_OK;
}

int MotorDriver_s::init(uint _PWM_PIN1, uint _PWM_PIN2)
{
    return init(_PWM_PIN1, _PWM_PIN2, DEFAULT_PWM_WRAP_VALUE, DEFAULT_PWM_CLK_DIV);
}

void MotorDriver_s::setPWM_PIN1(uint _PWM_PIN1)
{
    PWM_PIN1 = _PWM_PIN1;
    gpio_set_function(_PWM_PIN1, GPIO_FUNC_PWM);
    PWM1 = pwm_gpio_to_slice_num(_PWM_PIN1);
}

void MotorDriver_s::setPWM_PIN2(uint _PWM_PIN2)
{
    PWM_PIN2 = _PWM_PIN2;
    gpio_set_function(_PWM_PIN2, GPIO_FUNC_PWM);
    PWM2 = pwm_gpio_to_slice_num(_PWM_PIN2);
}

void MotorDriver_s::setPWM_WRAP_VALUE(uint _PWM_WRAP_VALUE)
{
    PWM_WRAP_VALUE = _PWM_WRAP_VALUE;
    pwm_set_wrap(PWM1, _PWM_WRAP_VALUE);
    pwm_set_wrap(PWM2, _PWM_WRAP_VALUE);
}

void MotorDriver_s::setPWM_CLK_DIV(float _PWM_CLK_DIV)
{
    PWM_CLK_DIV = _PWM_CLK_DIV;
    pwm_set_clkdiv(PWM1, _PWM_CLK_DIV);
    pwm_set_clkdiv(PWM2, _PWM_CLK_DIV);
}

void MotorDriver_s::setDuty(float _Duty)
{
    if (_Duty > DeadZone) Duty = std::max(_Duty, Bias);
    else Duty = 0.0f;
    int32_t pwm = Duty * PWM_WRAP_VALUE;
    int32_t max = PWM_WRAP_VALUE;

    if (pwm > 0) {
        PWM1_Value = max;
        PWM2_Value = pwm > max ? 0 : max - pwm;
    }
    else if (pwm < 0) {
        PWM1_Value = -pwm > max ? 0 : max + pwm;
        PWM2_Value = max;
    }
    else {
        PWM1_Value = max;
        PWM2_Value = max;
    }
}

void MotorDriver_s::loadConfiguration(Config_t _Config)
{
    setPWM_PIN1(_Config.PWM_PIN1);
    setPWM_PIN2(_Config.PWM_PIN2);
    setPWM_WRAP_VALUE(_Config.PWM_WRAP_VALUE);
    setPWM_CLK_DIV(_Config.PWM_CLK_DIV);
}

MotorDriver_s::Config_t MotorDriver_s::getConfiguration()
{
    return {
        .PWM_PIN1 = getPWM_PIN1(),
        .PWM_PIN2 = getPWM_PIN2(),
        .PWM_WRAP_VALUE = getPWM_WRAP_VALUE(),
        .PWM_CLK_DIV = getPWM_CLK_DIV(),
    };
}

void MotorDriver_s::start()
{
    pwm_set_enabled(PWM1, true);
    pwm_set_enabled(PWM2, true);
    Enabled = true;
}

void MotorDriver_s::stop()
{
    Duty = 0.0f;
    PWM1_Value = 0;
    PWM2_Value = 0;
    pwm_set_gpio_level(PWM_PIN1, 0);
    pwm_set_gpio_level(PWM_PIN2, 0);

    //pwm_set_enabled(PWM1, false);
    //pwm_set_enabled(PWM2, false);
    Enabled = false;
}

void MotorDriver_s::process(absolute_time_t _TimeNow)
{
    if (Enabled)
    {
        pwm_set_gpio_level(PWM_PIN1, PWM1_Value);
        pwm_set_gpio_level(PWM_PIN2, PWM2_Value);
    }
}

int Servo_s::init(uint _Out_pin)
{
    Out_pin = _Out_pin;
    return servo_hw_init(SERVO_PWM_CLKDIV, SERVO_PWM_PERIOD, Out_pin);
}

void Servo_s::setSpeed(float _Speed)
{
    uint pwm = SERVO_PWM_MIN * _Speed + SERVO_PWM_MIN;
    pwm = pwm > SERVO_PWM_MAX ? SERVO_PWM_MAX : (pwm < SERVO_PWM_MIN ? SERVO_PWM_MIN : pwm);
    pwm_set_gpio_level(Out_pin, pwm);
}

void Servo_s::setAngle(float _Angle)
{
    uint pwm = SERVO_PWM_MIN * (_Angle / 180.0f) + SERVO_PWM_MIN;
    pwm = pwm > SERVO_PWM_MAX ? SERVO_PWM_MAX : (pwm < SERVO_PWM_MIN ? SERVO_PWM_MIN : pwm);
    pwm_set_gpio_level(Out_pin, pwm);
}

