
#ifndef INC_MOTOR_DRIVER_HPP_
#define INC_MOTOR_DRIVER_HPP_

#include "motor_driver_hw.h"
#include "time_hw.h"

#define MOTOR_DRIVER_OK PICO_OK
#define MOTOR_DRIVER_ERROR PICO_ERROR_GENERIC

class MotorDriver
{
    motor_driver_hw_inst_t Driver_hw;

    float Min;
    float Max;
    float Power;
    float Setpoint;

    bool Enabled;
    uint64_t TimeStamp;
    
public:
    struct Config
    {
        float Min;
        float Max;
    };

    void   SetPower(const float& power) { Power = power; }
    float& GetPower(void)               { return Power;  }

    int Init(const motor_driver_hw_inst_t& hw)
    {
        Driver_hw = hw;
        if (motor_driver_hw_init(&Driver_hw) != MOTOR_DRIVER_HW_OK) return MOTOR_DRIVER_ERROR;

        Stop();

        return MOTOR_DRIVER_OK;
    }
    
    int Init(const motor_driver_hw_inst_t& hw, const Config& config)
    {
        if (Init(hw) != MOTOR_DRIVER_OK) return MOTOR_DRIVER_ERROR;

        LoadConfig(config);

        Stop();

        return MOTOR_DRIVER_OK;
    }

    int Start(const uint64_t& time = TIME_U64())
    {
        if (Enabled) return MOTOR_DRIVER_OK;

        motor_driver_hw_enable(&Driver_hw);

        Setpoint = -Power; //this forces update to run even in case the power and setpoint are equal
        Enabled = true;
        TimeStamp = time;

        return MOTOR_DRIVER_OK;
    }

    int Update(const uint64_t& time = TIME_U64())
    {
        if (Enabled)
        {
            if (Setpoint != Power)
            {
                Setpoint = Power;
                uint max = Driver_hw.Period;
                uint pwmA = max, pwmB = max;
                int pwm = ((Max - Min) * std::clamp(std::abs(Setpoint), 0.0f, 1.0f) + Min) * Driver_hw.Period;

                if (Setpoint > 0) {
                    pwm = (int)max - pwm;
                    pwmB = pwm < 0 ? 0 : pwm;
                }
                else if (Setpoint < 0) {
                    pwm = (int)max - pwm;
                    pwmA = pwm < 0 ? 0 : pwm;
                }

                motor_driver_hw_set_pwm(&Driver_hw, pwmA, pwmB);
                //printf("%d, %f, %d, %f, %f\n", pwm, Setpoint, max, Max, Min);
            }
        }

        return MOTOR_DRIVER_OK;
    }

    int Stop(void)
    {
        motor_driver_hw_set_pwm(&Driver_hw, 0U, 0U);
        motor_driver_hw_disable(&Driver_hw);

        Enabled = false;

        return MOTOR_DRIVER_OK;
    }

    int Brake(void)
    {
        motor_driver_hw_set_pwm(&Driver_hw, Driver_hw.Period, Driver_hw.Period);

        Enabled = false;

        return MOTOR_DRIVER_OK;
    }

    void LoadConfig(const Config& config)
    {
        Min = config.Min;
        Max = config.Max;
    }

	Config GetConfig(void)
    {
        return {
            .Min = Min,
            .Max = Max
        };
    }
};

#endif
