
#ifndef INC_PID_HPP_
#define INC_PID_HPP_

#include "pico/stdlib.h"
#include "vmath.hpp"

template<typename T>
class PID
{
    T Gain;
    T IntegralTime;
    T DerivativeTime;
    T IntegralTimeReciprocal;
    
    T InputFilter;
    T IntegralLimit;
    T DerivativeCutoff;
    T OutputMin;
    T OutputMax;

    T LastInput;
    T InputDeltaAvg;
    T Integral;
    T Derivative;
public:
    void reset(void)
    {
        LastInput = 0;
        InputDeltaAvg = 0;
        Integral = 0;
        Derivative = 0;
    }

    void setGain(T value) { Gain = value; }
    T getGain(void) { return Gain; }
    void setInputFilter(T value) { InputFilter = value; }
    T getInputFilter(void) { return InputFilter; }
    void setIntegralTime(T value)
    {
        IntegralTime = value;
        IntegralTimeReciprocal = value != (T)0.0 ? (T)1.0 / value : 0;
        Integral *= value;
    }
    T getIntegralTime(void) { return IntegralTime; }
    void setDerivativeTime(T value) { DerivativeTime = value; }
    T getDerivativeTime(void) { return DerivativeTime; }
    void setIntegralLimit(T value) { IntegralLimit = value; }
    T getIntegralLimit(void) { return IntegralLimit; }
    void setDerivativeCutoff(T value) { DerivativeCutoff = value; }
    T getDerivativeCutoff(void) { return DerivativeCutoff; }
    void setOutputMin(T value) { OutputMin = value; }
    T getOutputMin(void) { return OutputMin; }
    void setOutputMax(T value) { OutputMax = value; }
    T getOutputMax(void) { return OutputMax; }

    T Compute(const T setpoint, const T input, const T dt)
    {
        T error = setpoint - input;
        T dInput = input - LastInput;
        LastInput = input;

        InputDeltaAvg = dInput * InputFilter + InputDeltaAvg * ((T)1.0 - InputFilter);

        T dIntegral = error * IntegralTimeReciprocal * dt;
        Integral = std::clamp(Integral + dIntegral, -IntegralLimit, IntegralLimit);

        //Derivative
        T dDerivative = DerivativeCutoff * InputDeltaAvg * DerivativeTime / dt;
        Derivative = ((T)1.0 - DerivativeCutoff) * Derivative + dDerivative; //Derivative on measurement, with moving average filter

        T output = error;
        output += Derivative;
        output += Integral;
        output *= Gain;
        return std::clamp(output, OutputMin, OutputMax);
    }

};

#endif
