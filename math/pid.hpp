
#ifndef INC_PID_HPP_
#define INC_PID_HPP_

#include "pico/stdlib.h"
#include "vmath.hpp"

template<typename T>
struct PID
{
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

    uint64_t TimeStamp;

    // void reset(void)
    // {
    //     Output = 0.0f;
    //     Error = 0.0f;
    //     LastError = 0.0f;
    //     LastInput = 0.0f;
    //     Integral = 0.0f;
    //     Derivative = 0.0f;
    // }

    // void setSetPoint(float _SetPoint) { SetPoint = _SetPoint; }
    // float getSetPoint(void) { return SetPoint; }
    // void setGain(float _Gain) { Gain = _Gain; }
    // float getGain(void) { return Gain; }
    // void setIntegralTime(float _IntegralTime) {
    //     IntegralTime = _IntegralTime;
    //     IntegralTimeReciprocal = _IntegralTime != 0.0f ? 1.0f / _IntegralTime : 0.0f;
    //     Integral *= _IntegralTime;
    // }
    // float getIntegralTime(void) { return IntegralTime; }
    // void setIntegralLimit(float _IntegralLimit) { IntegralLimit = _IntegralLimit; }
    // float getIntegralLimit(void) { return IntegralLimit; }
    // void setIntegralRateLimit(float _IntegralRateLimit) { IntegralRateLimit = _IntegralRateLimit; }
    // float getIntegralRateLimit(void) { return IntegralRateLimit; }
    // void setIntegralAntiWindup(float _IntegralAntiWindup) { IntegralAntiWindup = _IntegralAntiWindup; }
    // float getIntegralAntiWindup(void) { return IntegralAntiWindup; }
    // void setDerivativeTime(float _DerivativeTime) { DerivativeTime = _DerivativeTime; }
    // float getDerivativeTime(void) { return DerivativeTime; }
    // void setDerivativeCutoff(float _DerivativeCutoff) { DerivativeCutoff = _DerivativeCutoff; }
    // float getDerivativeCutoff(void) { return DerivativeCutoff; }
    // void setOutputLimit(float _OutputLimit) { OutputLimit = _OutputLimit; }
    // float getOutputLimit(void) { return OutputLimit; }
    // void setDeadZone(float _DeadZone) { DeadZone = _DeadZone; }
    // float getDeadZone(void) { return DeadZone; }
    
    // float getError(void) { return Error; }
    // float getIntegral(void) { return Integral; }
    // float getDerivative(void) { return Derivative; }
    // float getDeltaTime(void) { return DeltaTime; }
    // float getOutput(void) { return Output; }

    // float compute(float _Input, float dT)
    // {
    //     DeltaTime = (T)(time - TimeStamp) / 1000000;
    //     TimeStamp = time;

    //     if (DeadZone != 0.0f && SetPoint < DeadZone && SetPoint > -DeadZone && _Input < DeadZone && _Input > -DeadZone)
    //     {
    //         Error = 0.0f;
    //         Integral = 0.0f;
    //         Derivative = 0.0f;
    //         Output = 0.0f;
    //     }
    //     else
    //     {
    //         Error = SetPoint - _Input;
    //         //Integral
    //         float tint = IntegralTime != 0.0f ? Error * IntegralTimeReciprocal * DeltaTime : 0.0f;
    //         Integral = std::clamp(Integral + tint, -IntegralLimit, IntegralLimit);// + std::clamp(Error, -IntegralRateLimit, IntegralRateLimit)

    //         //Derivative
    //         float dint = DerivativeTime != 0.0f ? DerivativeCutoff * (_Input - LastInput) * DerivativeTime / DeltaTime : 0.0f;
    //         Derivative = (1.0f - DerivativeCutoff) * Derivative + dint; //Derivative on measurement, with moving average filter
    //             // printf("int: %.6f, der: %.6f\n",Integral,Derivative
    //             // );
    //             // sleep_ms(200);

    //         //PID
    //         Output =  Error;
    //         Output += Derivative;
    //         Output += Integral;
    //         Output *= Gain;
    //         //Output limit
    //         Output = std::clamp(Output, -OutputLimit, OutputLimit);
    //     }
        
    //     LastError = Error;
    //     LastInput = _Input;
    //     return Output;
    // }
};

#endif
