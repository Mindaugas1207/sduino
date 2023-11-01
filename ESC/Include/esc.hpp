
#ifndef INC_ESC_HPP_
#define INC_ESC_HPP_

#include "esc_hw.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <algorithm>

// #define SERVO_PWM_CLKDIV 125.0f
// #define SERVO_PWM_PERIOD 20000U
// #define SERVO_PWM_MIN 1000
// #define SERVO_PWM_MAX 2000

#define ESC_OK PICO_OK
#define ESC_RAMP (ESC_OK + 1)
#define ESC_FINAL (ESC_OK + 2)
#define ESC_IDLE (ESC_OK + 3)
#define ESC_ERROR PICO_ERROR_GENERIC

class ESC_s {
    struct Config_s
    {
        esc_hw_inst_t Esc_hw;
        absolute_time_t RampUpPeriod;
        uint PWM_min;
        uint PWM_max;
        float Speed;
        float StartSpeed;
    };

    esc_hw_inst_t Esc_hw;
    absolute_time_t RampUpPeriod;
    uint PWM_min;
    uint PWM_max;

    float Speed;
    float StartSpeed;

    bool Enabled;
    bool RampUp;
    absolute_time_t TimeStamp;
public:
    typedef Config_s Config_t;

    void  SetSpeed(float _Speed) { Speed = _Speed; }
    float GetSpeed(float _Speed) { return Speed; }

    int Init(void)
    {
        if (esc_hw_init(&Esc_hw) != ESC_HW_OK) return ESC_ERROR;

        return ESC_OK;
    }
    
    int Init(Config_t _Config)
    {
        LoadConfiguration(_Config);

        return Init();
    }

    int Start(absolute_time_t time)
    {
        if (Enabled) return ESC_OK;
        TimeStamp = time;
        RampUp = true;
        Enabled = true;
        return ESC_OK;
    }

    int Update(absolute_time_t time)
    {
        if (Enabled)
        {
            float prop = 0.0f;

            if (RampUp)
            {
                auto dt = absolute_time_diff_us(TimeStamp, time);

                if (dt < RampUpPeriod)
                {
                    prop = (((float)dt / RampUpPeriod) * (Speed - StartSpeed)) + StartSpeed;
                }
                else
                {
                    RampUp = false;
                    prop = Speed;
                }
            }
            else
            {
                prop = Speed;
            }

            uint pwm = (PWM_max - PWM_min) * std::clamp(prop, StartSpeed, Speed) + PWM_min;
            esc_hw_set_pulse(&Esc_hw, std::clamp(pwm, PWM_min, PWM_max));

            return RampUp ? ESC_RAMP : ESC_FINAL;
        }

        esc_hw_set_pulse(&Esc_hw, PWM_min);
        return ESC_OK;
    }

    int Stop(void)
    {
        esc_hw_set_pulse(&Esc_hw, PWM_min);
        Enabled = false;
        return ESC_OK;
    }

    void LoadConfiguration(Config_t _Config)
    {
        Esc_hw = _Config.Esc_hw;
        RampUpPeriod = _Config.RampUpPeriod;
        PWM_min = _Config.PWM_min;
        PWM_max = _Config.PWM_max;
        Speed = _Config.Speed;
        StartSpeed = _Config.StartSpeed;
    }

	Config_t GetConfiguration(void)
    {
        return {
            .Esc_hw = Esc_hw,
            .RampUpPeriod = RampUpPeriod,
            .PWM_min = PWM_min,
            .PWM_max = PWM_max,
            .Speed = Speed,
            .StartSpeed = StartSpeed
        };
    }
};

#endif
