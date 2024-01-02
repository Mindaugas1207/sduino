
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

    // T GetDeltaT(const uint64_t& time)
    // {
    //     T dT = (T)(time - TimeStamp) / 1000000;
    //     TimeStamp = time;

    //     return dT;
    // }

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

    // int Update(const uint64_t& time = TIME_U64())
    // {
    //     return Update(time, GetDeltaT(time));
    // }

    // std::tuple<float, float, float> XYoffsetEncoders(auto dR, auto dL, float A0)
    // {
    //     auto A = dR - dL;
    //     auto E = (float)(ENCODER_BASE_LENGTH / 2.0) * (dR + dL);
    //     auto B = E / A;
    //     float AX, AY, C;
    //     if (std::isinf(B) || std::isnan(B))
    //     {
    //         B = (float)(ENCODER_PULSE_TO_LENGTH / 2.0) * (dR + dL);
    //         C = 0.0f;
    //         AX = std::cos(A0);
    //         AY = std::sin(A0);
    //     }
    //     else
    //     {
    //         C = (float)(ENCODER_PULSE_TO_LENGTH / ENCODER_BASE_LENGTH) * A;
    //         AX =   std::sin(C + A0) - std::sin(A0);
    //         AY = -(std::cos(C + A0) - std::cos(A0));
    //     }
        
    //     return {B * AX, B * AY, C};
    // }

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

                RPS  = StepsDelta  * Ratio;
                RPS_ = CountsDelta * Ratio * 4;
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

        Ratio = (SamplingPeriod / (T)(PPR * Oersampling * Reduction)) / 1000000;
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
