
#ifndef INC_ESC_HPP_
#define INC_ESC_HPP_

#include "esc_hw.h"
#include "time_hw.h"

#define ESC_OK PICO_OK
#define ESC_RAMP (ESC_OK + 1)
#define ESC_FINAL (ESC_OK + 2)
#define ESC_IDLE (ESC_OK + 3)
#define ESC_ERROR PICO_ERROR_GENERIC

class ESC
{
    esc_hw_inst_t Esc_hw;
    uint64_t RampUpPeriod;

    float Min;
    float Max;
    float Power;
    float Setpoint;
    float StartingPower;

    bool Enabled;
    bool RampUp;
    uint64_t TimeStamp;

public:
    struct Config
    {
        uint64_t RampUpPeriod;
        float Min;
        float Max;
        float Power;
        float StartingPower;
    };

    void   SetPower(const float& power) { Power = power; }
    float& GetPower(void)               { return Power;  }

    int Init(const esc_hw_inst_t& hw)
    {
        Esc_hw = hw;
        if (esc_hw_init(&Esc_hw) != ESC_HW_OK) return ESC_ERROR;

        Stop();
        
        return ESC_OK;
    }
    
    int Init(const esc_hw_inst_t& hw, const Config& config)
    {
        if (Init(hw) != ESC_OK) return ESC_ERROR;

        LoadConfig(config);

        Stop();
        
        return ESC_OK;
    }

    int Start(const uint64_t& time = TIME_U64())
    {
        if (Enabled) return ESC_OK;

        //esc_hw_enable(&Esc_hw);
        
        Setpoint = -Power; //this forces update to run even in case the power and setpoint are equal
        Enabled = true;
        RampUp = true;
        TimeStamp = time;

        return ESC_OK;
    }

    int Update(const uint64_t& time = TIME_U64())
    {
        if (Enabled)
        {
            float _Setpoint = 0.0f;

            if (RampUp)
            {
                auto dt = time - TimeStamp;

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
        //esc_hw_disable(&Esc_hw);

        Enabled = false;

        return ESC_OK;
    }

    void LoadConfig(const Config& config)
    {
        RampUpPeriod = config.RampUpPeriod;
        Min = config.Min;
        Max = config.Max;
        Power = config.Power;
        StartingPower = config.StartingPower;
    }

	Config GetConfig(void)
    {
        return {
            .RampUpPeriod = RampUpPeriod,
            .Min = Min,
            .Max = Max,
            .Power = Power,
            .StartingPower = StartingPower
        };
    }
};

#endif
