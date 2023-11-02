
#ifndef INC_ESC_HPP_
#define INC_ESC_HPP_

#include "esc_hw.h"

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
        float Min;
        float Max;
        float Power;
        float StartingPower;
    };

    esc_hw_inst_t Esc_hw;
    absolute_time_t RampUpPeriod;

    float Min;
    float Max;
    float Power;
    float Setpoint;
    float StartingPower;

    bool Enabled;
    bool RampUp;
    absolute_time_t TimeStamp;
public:
    typedef Config_s Config_t;

    void  SetPower(float _Power) { Power = _Power; }
    float GetPower(float _Power) { return Power; }

    int Init(void)
    {
        if (esc_hw_init(&Esc_hw) != ESC_HW_OK) return ESC_ERROR;

        Stop();
        
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
        
        Setpoint = -Power; //this forces update to run even in case the power and setpoint are equal
        Enabled = true;
        RampUp = true;
        TimeStamp = time;

        return ESC_OK;
    }

    int Update(absolute_time_t time)
    {
        if (Enabled)
        {
            float _Setpoint = 0.0f;

            if (RampUp)
            {
                auto dt = absolute_time_diff_us(TimeStamp, time);

                if (dt < RampUpPeriod)
                {
                    _Setpoint = (((float)dt / RampUpPeriod) * (Power - StartingPower)) + StartingPower;
                }
                else
                {
                    RampUp = false;
                    _Setpoint = Power;
                }
            }
            else
            {
                _Setpoint = Power;
            }

            if (Setpoint != _Setpoint)
            {
                Setpoint = _Setpoint;
                int pwm = ((Max - Min) * std::clamp(Setpoint, 0.0f, 1.0f) + Min) * Esc_hw.Period;
                esc_hw_set_pulse(&Esc_hw, std::clamp(pwm, 0, (int)Esc_hw.Period));
            }

            return RampUp ? ESC_RAMP : ESC_FINAL;
        }

        return ESC_OK;
    }

    int Stop(void)
    {
        esc_hw_set_pulse(&Esc_hw, Min * Esc_hw.Period);

        Enabled = false;

        return ESC_OK;
    }

    void LoadConfiguration(Config_t _Config)
    {
        Esc_hw = _Config.Esc_hw;
        RampUpPeriod = _Config.RampUpPeriod;
        Min = _Config.Min;
        Max = _Config.Max;
        Power = _Config.Power;
        StartingPower = _Config.StartingPower;
    }

	Config_t GetConfiguration(void)
    {
        return {
            .Esc_hw = Esc_hw,
            .RampUpPeriod = RampUpPeriod,
            .Min = Min,
            .Max = Max,
            .Power = Power,
            .StartingPower = StartingPower
        };
    }
};

#endif
