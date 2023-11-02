
#ifndef INC_MOTOR_DRIVER_HPP_
#define INC_MOTOR_DRIVER_HPP_

#include "motor_driver_hw.h"

#define MOTOR_DRIVER_OK PICO_OK
#define MOTOR_DRIVER_ERROR PICO_ERROR_GENERIC

class MotorDriver_s {
    struct Config_s
    {
        motor_driver_hw_inst_t Driver_hw;
        float Min;
        float Max;
    };

    motor_driver_hw_inst_t Driver_hw;

    float Min;
    float Max;
    float Power;
    float Setpoint;

    bool Enabled;
    absolute_time_t TimeStamp;
public:
    typedef Config_s Config_t;

    void  SetPower(float _Power) { Power = _Power; }
    float GetPower(float _Power) { return Power; }

    int Init(void)
    {
        if (motor_driver_hw_init(&Driver_hw) != MOTOR_DRIVER_HW_OK) return MOTOR_DRIVER_ERROR;

        Stop();

        return MOTOR_DRIVER_OK;
    }
    
    int Init(Config_t _Config)
    {
        LoadConfiguration(_Config);

        return Init();
    }

    int Start(absolute_time_t time)
    {
        if (Enabled) return MOTOR_DRIVER_OK;

        Setpoint = -Power; //this forces update to run even in case the power and setpoint are equal
        Enabled = true;
        TimeStamp = time;

        return MOTOR_DRIVER_OK;
    }

    int Update(absolute_time_t time)
    {
        if (Enabled)
        {
            if (Setpoint != Power)
            {
                Setpoint = Power;
                uint max = Driver_hw.Period;
                uint pwmA = max, pwmB = max;
                int pwm = ((Max - Min) * std::clamp(Setpoint, 0.0f, 1.0f) + Min) * Driver_hw.Period;

                if (pwm > 0) {
                    pwm = (int)max - pwm;
                    pwmB = pwm < 0 ? 0 : pwm;
                }
                else if (pwm < 0) {
                    pwm = (int)max + pwm;
                    pwmA = pwm < 0 ? 0 : pwm;
                }

                motor_driver_hw_set_pwm(&Driver_hw, pwmA, pwmB);
            }
        }

        return MOTOR_DRIVER_OK;
    }

    int Stop(void)
    {
        motor_driver_hw_set_pwm(&Driver_hw, 0U, 0U);

        Enabled = false;

        return MOTOR_DRIVER_OK;
    }

    int Brake(void)
    {
        motor_driver_hw_set_pwm(&Driver_hw, Driver_hw.Period, Driver_hw.Period);

        Enabled = false;

        return MOTOR_DRIVER_OK;
    }

    void LoadConfiguration(Config_t _Config)
    {
        Driver_hw = _Config.Driver_hw;
        Min = _Config.Min;
        Max = _Config.Max;
    }

	Config_t GetConfiguration(void)
    {
        return {
            .Driver_hw = Driver_hw,
            .Min = Min,
            .Max = Max
        };
    }
};

#endif
