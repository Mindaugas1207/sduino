
#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "stdio.h"
#include "pico/stdlib.h"
#include "encoder_hw.h"
#include "math.h"

#define ENCODER_OK PICO_OK
#define ENCODER_NEWDATA (ENCODER_OK + 1)
#define ENCODER_ERROR PICO_ERROR_GENERIC

class Encoder_s
{
    struct Config_s
    {
        encoder_hw_inst_t Encoder_hw;
        freqcnt_hw_inst_t Freqcnt_hw;
        absolute_time_t SamplingPeriod;
        int RPM_min;
        int RPM_max;
        int PPR;//7
        int Reduction;//4
        int Oersampling;//4
        float WheelDiameter;//22.0f/1000
    };
    encoder_hw_inst_t Encoder_hw;
    freqcnt_hw_inst_t Freqcnt_hw;
    absolute_time_t SamplingPeriod;

    int RPM_min;
    int RPM_max;
    int PPR;//7
    int Reduction;//4
    int Oersampling;//4
    float WheelDiameter;//22.0f/1000

    float WheelLength; //WheelDiameter * M_PI
    float Ratio;

    float RPM;
    float RPM_;
    float RPS;
    float RPS_;
    float MPS;
    float MPS_;

    int32_t StepsLast;
    int32_t StepsDelta;
    uint32_t CountsLast;
    uint32_t CountsDelta;

    absolute_time_t TimeStamp;
public:
    typedef Config_s Config_t;

    int Init(void)
    {
        if (encoder_hw_init(&Encoder_hw) != ENCODER_HW_OK) return ENCODER_ERROR;
        if (freqcnt_init(&Freqcnt_hw) != ENCODER_HW_OK) return ENCODER_ERROR;

        Ratio = 1000000.0f / (PPR * Oersampling * Reduction);
        WheelLength = WheelDiameter * M_PI;
        
        return ENCODER_OK;
    }

    int Init(Config_t _Config)
    {
        LoadConfiguration(_Config);

        return Init();
    }

    int Start(void)
    {
        return ENCODER_OK;
    }

    int Update(absolute_time_t time)
    {
        auto dt = absolute_time_diff_us(TimeStamp, time);
        if (dt > SamplingPeriod)
        {
            auto steps = encoder_get_step_count(&Encoder_hw);
            auto counts = freqcnt_get_pulse_count(&Freqcnt_hw);

            StepsDelta = steps - StepsLast;
            CountsDelta = counts - CountsLast;
            auto tmp = Ratio / dt;

            RPS  = StepsDelta  * tmp;
            RPS_ = CountsDelta * tmp;
            RPM  = 60 * RPS;
            RPM_ = 60 * RPS_;

            MPS  = RPS * WheelLength;
            MPS_ = RPS_ * WheelLength;

            StepsLast = steps;
            CountsLast = counts;
            TimeStamp = time;

            return ENCODER_NEWDATA;
        }

        return ENCODER_OK;
    }

    int Stop(void)
    {
        return ENCODER_OK;
    }

    void LoadConfiguration(Config_t _Config)
    {
        Encoder_hw = _Config.Encoder_hw;
        Freqcnt_hw = _Config.Freqcnt_hw;
        SamplingPeriod = _Config.SamplingPeriod;
        RPM_min = _Config.RPM_min;
        RPM_max = _Config.RPM_max;
        PPR = _Config.PPR;//7
        Reduction = _Config.Reduction;//4
        Oersampling = _Config.Oersampling;//4
        WheelDiameter = _Config.WheelDiameter;//22.0f/1000
    }

	Config_t GetConfiguration(void)
    {
        return {
            .Encoder_hw = Encoder_hw,
            .Freqcnt_hw = Freqcnt_hw,
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
