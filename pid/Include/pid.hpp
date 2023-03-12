
#ifndef INC_PID_HPP_
#define INC_PID_HPP_

#include "pico/stdlib.h"
#include <algorithm>

class PID_s {
private:
    struct Config_s {
        float SetPoint;
        float Gain;
        float IntegralTime;
        float IntegralLimit;
        float IntegralRateLimit;
        float IntegralAntiWindup;
        float DerivativeTime;
        float DerivativeCutoff;
        float OutputLimit;
        float DeadZone;
    };

    float SetPoint;
    float Gain;
    float IntegralTime;
    float IntegralLimit;
    float IntegralRateLimit;
    float IntegralAntiWindup;
    float DerivativeTime;
    float DerivativeCutoff;
    float OutputLimit;
    float DeadZone;
    
    float Output;
    float Error;
    float LastInput;
    float LastError;
    float Integral;
    float IntegralTimeReciprocal;
    float Derivative;
    float DeltaTime;

    absolute_time_t LastTime;
public:
    typedef Config_s Config_t;

    void reset(void)
    {
        Output = 0.0f;
        Error = 0.0f;
        LastError = 0.0f;
        LastInput = 0.0f;
        Integral = 0.0f;
        Derivative = 0.0f;
    }

    void setSetPoint(float _SetPoint) { SetPoint = _SetPoint; }
    float getSetPoint(void) { return SetPoint; }
    void setGain(float _Gain) { Gain = _Gain; }
    float getGain(void) { return Gain; }
    void setIntegralTime(float _IntegralTime) {
        IntegralTime = _IntegralTime;
        IntegralTimeReciprocal = _IntegralTime != 0.0f ? 1.0f / _IntegralTime : 0.0f;
        Integral *= _IntegralTime;
    }
    float getIntegralTime(void) { return IntegralTime; }
    void setIntegralLimit(float _IntegralLimit) { IntegralLimit = _IntegralLimit; }
    float getIntegralLimit(void) { return IntegralLimit; }
    void setIntegralRateLimit(float _IntegralRateLimit) { IntegralRateLimit = _IntegralRateLimit; }
    float getIntegralRateLimit(void) { return IntegralRateLimit; }
    void setIntegralAntiWindup(float _IntegralAntiWindup) { IntegralAntiWindup = _IntegralAntiWindup; }
    float getIntegralAntiWindup(void) { return IntegralAntiWindup; }
    void setDerivativeTime(float _DerivativeTime) { DerivativeTime = _DerivativeTime; }
    float getDerivativeTime(void) { return DerivativeTime; }
    void setDerivativeCutoff(float _DerivativeCutoff) { DerivativeCutoff = _DerivativeCutoff; }
    float getDerivativeCutoff(void) { return DerivativeCutoff; }
    void setOutputLimit(float _OutputLimit) { OutputLimit = _OutputLimit; }
    float getOutputLimit(void) { return OutputLimit; }
    void setDeadZone(float _DeadZone) { DeadZone = _DeadZone; }
    float getDeadZone(void) { return DeadZone; }
    
    float getError(void) { return Error; }
    float getIntegral(void) { return Integral; }
    float getDerivative(void) { return Derivative; }
    float getDeltaTime(void) { return DeltaTime; }
    float getOutput(void) { return Output; }

    void loadConfiguration(Config_t _Config)
    {
        setSetPoint(_Config.SetPoint);
        setGain(_Config.Gain);
        setIntegralTime(_Config.IntegralTime);
        setIntegralLimit(_Config.IntegralLimit);
        setIntegralRateLimit(_Config.IntegralRateLimit);
        setIntegralAntiWindup(_Config.IntegralAntiWindup);
        setDerivativeTime(_Config.DerivativeTime);
        setDerivativeCutoff(_Config.DerivativeCutoff);
        setOutputLimit(_Config.OutputLimit);
        setDeadZone(_Config.DeadZone);
    }

    Config_t getConfiguration() {
        return {
            .SetPoint = getSetPoint(),
            .Gain = getGain(),
            .IntegralTime = getIntegralTime(),
            .IntegralLimit = getIntegralLimit(),
            .IntegralRateLimit = getIntegralRateLimit(),
            .IntegralAntiWindup = getIntegralAntiWindup(),
            .DerivativeTime = getDerivativeTime(),
            .DerivativeCutoff = getDerivativeCutoff(),
            .OutputLimit = getOutputLimit(),
            .DeadZone = getDeadZone(),
        };
    }

    float compute(float _Input, absolute_time_t _TimeNow)
    {
        DeltaTime = absolute_time_diff_us(LastTime, _TimeNow) / 1000000.0f;
        LastTime = _TimeNow;

        if (DeadZone != 0.0f && SetPoint < DeadZone && SetPoint > -DeadZone && _Input < DeadZone && _Input > -DeadZone)
        {
            Error = 0.0f;
            Integral = 0.0f;
            Derivative = 0.0f;
            Output = 0.0f;
        }
        else
        {
            Error = SetPoint - _Input;
            //Integral
            Integral = std::clamp(Integral + std::clamp(Error, -IntegralRateLimit, IntegralRateLimit), -IntegralLimit, IntegralLimit);
        
            //Derivative
            Derivative = (1.0f - DerivativeCutoff) * Derivative + DerivativeCutoff * (_Input - LastInput); //Derivative on measurement, with moving average filter
            //PID
            Output = Error;
            Output += DerivativeTime != 0.0f ? Derivative * DerivativeTime / DeltaTime : 0.0f;
            Output += IntegralTimeReciprocal != 0.0f ? Integral * IntegralTimeReciprocal * DeltaTime : 0.0f;
            Output *= Gain;
            //Output limit
            Output = std::clamp(Output, -OutputLimit, OutputLimit);
        }
        
        LastError = Error;
        LastInput = _Input;
        return Output;
    }
};

#endif
