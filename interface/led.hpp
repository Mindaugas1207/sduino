
#ifndef LED_HPP_
#define LED_HPP_

#include "pico/stdlib.h"
#include "time_hw.h"

#define LED_OK PICO_OK
#define LED_ERROR PICO_ERROR_GENERIC

class LED
{
    enum LED_Modes
    {
        NORMAL,
        BLINK,
        PULSE
    };

    uint HW_Pin;
    LED_Modes Mode;

    uint32_t OnTime;
    uint32_t OffTime;
    uint32_t N_Pulses;


    uint32_t PulseCount;

    

    bool Enabled;
    bool PulseEn;
    bool State;
    uint64_t TimeStamp;

    void SetState(const bool& state)
    {
        State = state;
        gpio_put(HW_Pin, State);
    }

public:

    int Init(const uint& hw_pin)
    {
        HW_Pin = hw_pin;
        Mode   = NORMAL;

        gpio_init(HW_Pin);
        gpio_set_dir(HW_Pin, GPIO_OUT);

        Stop();

        return LED_OK;
    }

    void Set(const bool& value)
    {
        Mode  = NORMAL;
        State = value;
    }

    void Set(const uint32_t& onTime, const uint32_t& offTime, const uint64_t& time = TIME_U64())
    {
        Mode    = BLINK;
        OnTime  = onTime * 1000;
        OffTime = offTime * 1000;
        TimeStamp = time;
        SetState(false);
    }

    void Set(const uint32_t& onTime, const uint32_t& offTime, const uint32_t& nPulses, const uint64_t& time = TIME_U64() )
    {
        Mode       = PULSE;
        OnTime     = onTime * 1000;
        OffTime    = offTime * 1000;
        N_Pulses   = nPulses;
        PulseCount = 0;
        PulseEn    = true;
        TimeStamp = time;
        SetState(false);
    }

    void Toggle(void)
    {
        Mode   = NORMAL;
        State != State;
    }

    int Start(const uint64_t& time = TIME_U64())
    {
        if (Enabled) return LED_OK;

        Enabled = true;
        TimeStamp = time;

        return LED_OK;
    }

    int Update(const uint64_t& time = TIME_U64())
    {
        if (Enabled)
        {
            switch (Mode)
            {
            case NORMAL:
            SetState(State);
            break;
            case BLINK:
            DoBlink(time);
            break;
            case PULSE:
            DoPulse(time);
            break;
            default:
                return LED_ERROR;
            }
        }

        return LED_OK;
    }

    void DoBlink(const uint64_t& time = TIME_U64())
    {
        if (State && (time - TimeStamp) > OnTime)
        {
            TimeStamp = time;
            SetState(false);
        }
        else if (!State && (time - TimeStamp) > OffTime)
        {
            TimeStamp = time;
            SetState(true);
        }
    }

    void DoPulse(const uint64_t& time = TIME_U64())
    {
        if (!PulseEn) return;

        if (State && (time - TimeStamp) > OnTime)
        {
            TimeStamp = time;
            SetState(false);
        }
        else if (!State && (time - TimeStamp) > OffTime)
        {
            TimeStamp = time;
            if (PulseCount < N_Pulses)
            {
                PulseCount++;
                SetState(true);
            }
            else
            {
                PulseEn = false;
                SetState(false);
            }
        }
    }

    int Stop(void)
    {
        Enabled = false;
        PulseEn = false;
        PulseCount = 0;
        SetState(false);
        
        return LED_OK;
    }
};

#endif
