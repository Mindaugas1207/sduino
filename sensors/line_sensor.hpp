
#ifndef LINE_SENSOR_HPP_
#define LINE_SENSOR_HPP_

#include "line_sensor_hw.h"
#include "vmath.hpp"
#include <array>
#include <tuple>
#include <limits>
#include "time_hw.h"


#define LINE_SENSOR_OK PICO_OK
#define LINE_SENSOR_ERROR PICO_ERROR_GENERIC

#define LINE_SENSOR_NUM_SENSORS LINE_SENSOR_HW_NUM_SENSORS
#define LINE_SENSOR_THRESHOLD_CLEARANCE (0.1)

#define LINE_SENSOR_AUTO_CALIBRATE (true)

typedef enum {
    BLACK,
    WHITE
} color_t;

template <typename T>
class IRSensor
{
    uint Min, Max;
    T Ratio;

    T ComputeRatio(uint& min, uint& max) { return Ratio = min != max ? (T)(1.0) / (T)(max - min) : (T)(1.0); }

public:
    struct Config { uint Min, Max; };

    T Value;
    color_t Color;

    void Init()
    {
        ResetConfig();
        Value = (T)(0.0);
        Color = WHITE;
    }

    void Init(const Config& config)
    {
        LoadCalibration(config);
        Value = (T)(0.0);
        Color = WHITE;
    }

    T Compute(const uint& input)
    {
        uint raw = std::clamp(input, Min, Max);

        Value = (raw - Min) * Ratio;

        Color = Value > (T)(0.5 + LINE_SENSOR_THRESHOLD_CLEARANCE) ? BLACK : (Value < (T)(0.5 - LINE_SENSOR_THRESHOLD_CLEARANCE) ? WHITE : Color);

        return Value;
    }

    void ResetConfig(void)
    {
        Min = std::numeric_limits<uint>::max();
        Max = std::numeric_limits<uint>::min();
        Ratio = 1.0f;
    }

    void Calibrate(const uint& input)
    {
        Min = std::min(input, Min);
        Max = std::max(input, Max);
        Ratio = ComputeRatio(Min, Max);
    }

    void LoadConfig(const Config& config)
    {
        Min = config.Min;
        Max = config.Max;
        Ratio = ComputeRatio(Min, Max);
    }

    Config GetConfig(void) { return {Min, Max}; }
};

template <typename T, size_t _SensorCount = LINE_SENSOR_NUM_SENSORS>
class LineSensor
{
    line_sensor_hw_inst_t LineSensor_hw;
    uint32_t CalibrationTime;

    std::array<IRSensor<T>, _SensorCount> Sensors;

    //std::array<uint, _SensorCount> RawData;

    bool Enabled;
    bool Calibrated;
    bool CalibrationStarted;
    uint64_t TimeStamp;
public:
    struct Config
    {
        std::template array<typename IRSensor<T>::Config, _SensorCount> SensorCalibration;
        uint32_t CalibrationTime;
        bool Calibrated;
    };

    std::array<uint, _SensorCount> RawData;

    size_t SensorCount(void) { return _SensorCount; }

    int Init(const line_sensor_hw_inst_t& hw)
    {
        LineSensor_hw = hw;
        if (line_sensor_hw_init(&LineSensor_hw) != LINE_SENSOR_HW_OK) return LINE_SENSOR_ERROR;

        for(IRSensor<T> &s: Sensors)
            s.Init();

        Stop();

        return LINE_SENSOR_OK;
    }
    
    int Init(const line_sensor_hw_inst_t& hw, const Config& config)
    {
        if (Init(hw) != LINE_SENSOR_OK) return LINE_SENSOR_ERROR;

        LoadConfig(config);

        return LINE_SENSOR_OK;
    }

    bool IsCalibrated(void) { return Calibrated; }

    int Start(const uint64_t& time = TIME_U64())
    {
        if (Enabled) return LINE_SENSOR_OK;

        line_sensor_hw_enable(&LineSensor_hw);

        Enabled = true;
        TimeStamp = time;

        return LINE_SENSOR_OK;
    }

    int Update(const uint64_t& time = TIME_U64())
    {
        if (Enabled)
        {
            if (line_sensor_hw_read(&LineSensor_hw, RawData.data()) != LINE_SENSOR_HW_OK) return LINE_SENSOR_ERROR;

            if (Calibrated)
                Compute(time);
            else if (CalibrationStarted)
                RunCalibration(time);
            else if (LINE_SENSOR_AUTO_CALIBRATE)
                StartCalibration(time);
        }

        return LINE_SENSOR_OK;
    }

    void Compute(const uint64_t& time = TIME_U64())
    {
        for(uint i = 0; i < _SensorCount; i++)
            Sensors[i].Compute(RawData[i]);
    }

    void StartCalibration(const uint64_t& time = TIME_U64())
    {
        for(IRSensor<T> &s: Sensors)
            s.ResetConfig();

        TimeStamp = time;
        Calibrated = false;
        CalibrationStarted = true;
    }

    void RunCalibration(const uint64_t& time = TIME_U64())
    {
        if (time - TimeStamp < CalibrationTime)
        {
            for(uint i = 0; i < _SensorCount; i++)
                Sensors[i].Calibrate(RawData[i]);
        }
        else
        {
            Calibrated = true;
            CalibrationStarted = false;
        }
    }

    int Stop(void)
    {
        line_sensor_hw_disable(&LineSensor_hw);

        Enabled = false;
        
        return LINE_SENSOR_OK;
    }

    void LoadConfig(const Config& config)
    {
        if (config.Calibrated)
        {
            for(uint i = 0; i < _SensorCount; i++)
                Sensors[i].LoadConfig(config.SensorCalibration[i]);
        }
        
        CalibrationTime = config.CalibrationTime;
        Calibrated      = config.Calibrated;
    }

	Config GetConfig(void)
    {
        Config result;

        for(uint i = 0; i < _SensorCount; i++)
            result.SensorCalibration[i] = Sensors[i].GetConfig();

        result.CalibrationTime = CalibrationTime;
        result.Calibrated = Calibrated;

        return result;
    }
};

#endif
