
#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "encoder_hw.h"
#include "time_hw.h"

#define ENCODER_OK PICO_OK
#define ENCODER_ERROR PICO_ERROR_GENERIC

template<typename T>
class Encoder
{
    encoder_hw_inst_t Encoder_hw;
    uint64_t SamplingPeriod;

    int RPM_min;
    int RPM_max;
    int PPR;//7
    int Reduction;//4
    int Oersampling;//4
    T WheelDiameter;//22.0f/1000

    T WheelLength; //WheelDiameter * M_PI
    T Ratio;

    bool Enabled;
    uint64_t TimeStamp;

public:
    struct Config
    {
        uint64_t SamplingPeriod;
        int RPM_min;
        int RPM_max;
        int PPR;//7
        int Reduction;//4
        int Oersampling;//4
        T WheelDiameter;//22.0f/1000
    };

    int32_t StepsLast;
    int32_t StepsDelta;
    uint32_t CountsLast;
    uint32_t CountsDelta;
    T RPM;
    T RPM_;
    T RPS;
    T RPS_;
    T MPS;
    T MPS_;

    int Init(const encoder_hw_inst_t& hw)
    {
        Encoder_hw = hw;
        if (encoder_hw_init(&Encoder_hw) != ENCODER_HW_OK) return ENCODER_ERROR;

        Stop();
        
        return ENCODER_OK;
    }

    int Init(const encoder_hw_inst_t& hw, const Config& config)
    {
        if (Init(hw) != ENCODER_OK) return ENCODER_ERROR;

        LoadConfig(config);

        return ENCODER_OK;
    }

    int Start(const uint64_t& time = TIME_U64())
    {
        if (Enabled) return ENCODER_OK;

        encoder_hw_enable(&Encoder_hw);

        TimeStamp = time;
        Enabled = true;

        return ENCODER_OK;
    }

    int Update(const uint64_t& time = TIME_U64())
    {
        if (Enabled)
        {
            auto dt = (time - TimeStamp);
            if (dt > SamplingPeriod)
            {
                auto steps = encoder_get_step_count(&Encoder_hw);
                auto counts = encoder_get_pulse_count(&Encoder_hw);

                StepsDelta = steps - StepsLast;
                CountsDelta = counts;
                auto tmp = Ratio / dt;

                RPS  = StepsDelta  * tmp;
                RPS_ = CountsDelta * tmp * 4;
                RPM  = RPS  * 60;
                RPM_ = RPS_ * 60;

                MPS  = RPS * WheelLength;
                MPS_ = RPS_ * WheelLength;

                StepsLast = steps;
                CountsLast = counts;
                TimeStamp = time;
            }
        }

        return ENCODER_OK;
    }

    int Stop(void)
    {
        encoder_hw_disable(&Encoder_hw);

        Enabled = false;

        return ENCODER_OK;
    }

    void LoadConfig(const Config& config)
    {
        SamplingPeriod = config.SamplingPeriod;
        RPM_min = config.RPM_min;
        RPM_max = config.RPM_max;
        PPR = config.PPR;//7
        Reduction = config.Reduction;//4
        Oersampling = config.Oersampling;//4
        WheelDiameter = config.WheelDiameter;//22.0f/1000

        Ratio = 1000000 / (T)(PPR * Oersampling * Reduction);
        WheelLength = WheelDiameter * M_PI;
    }

	Config GetConfig(void)
    {
        return {
            .SamplingPeriod = SamplingPeriod,
            .RPM_min = RPM_min,
            .RPM_max = RPM_max,
            .PPR = PPR,
            .Reduction = Reduction,
            .Oersampling = Oersampling,
            .WheelDiameter = WheelDiameter
        };
    }
};



#endif
