
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
    uint32_t LedTime;
    uint32_t CalibrationTime;
    uint32_t CalibrationBlinkTime;

    std::array<IRSensor<T>, _SensorCount> Sensors;

    int lpos = 0;

    //std::array<uint, _SensorCount> RawData;

    bool Enabled;
    bool Calibrated;
    bool CalibrationStarted;
    bool DisplayAnalog;
    uint64_t TimeStamp;
    uint64_t TimeStampCalibration;
    uint64_t TimeStampCalibrationBlink;

public:
    struct Config
    {
        std::template array<typename IRSensor<T>::Config, _SensorCount> SensorCalibration;
        uint32_t CalibrationTime;
        bool Calibrated;
    };

    std::array<uint, _SensorCount> RawData;

    size_t SensorCount(void) { return _SensorCount; }

    IRSensor<T>& operator[](const std::size_t& i)             { return Sensors[i]; }
    const IRSensor<T>& operator[](const std::size_t& i) const { return Sensors[i]; }

    void SetDisplayAnalog(bool value) { DisplayAnalog = value; }
    bool GetDisplayAnalog(void) { return DisplayAnalog; }

    void SetLpos(int lps) {lpos = lps;}

    int Init(const line_sensor_hw_inst_t& hw)
    {
        LineSensor_hw = hw;
        if (line_sensor_hw_init(&LineSensor_hw) != LINE_SENSOR_HW_OK) return LINE_SENSOR_ERROR;

        for(IRSensor<T> &s: Sensors)
            s.Init();

        CalibrationBlinkTime = 50000;
        LedTime = 20000;

        DisplayAnalog = false;

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
        TimeStampCalibration = time;

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

        //UpdateLeds(time);
    }

    void UpdateLeds(const uint64_t& time = TIME_U64())
    {
        if (time - TimeStamp > LedTime)
        {
            TimeStamp = time;

            if (DisplayAnalog)
            {
                for(uint i = 0; i < _SensorCount; i++)
                {
                    line_sensor_hw_set_led_power(&LineSensor_hw, i + LINE_SENSOR_HW_OFFSET, Sensors[i].Value);
                    line_sensor_hw_set_led_enable(&LineSensor_hw, i + LINE_SENSOR_HW_OFFSET, true);
                }
            }
            else
            {
                for(uint i = 0; i < _SensorCount; i++)
                {
                    line_sensor_hw_set_led_power(&LineSensor_hw, i + LINE_SENSOR_HW_OFFSET, 0U);
                    line_sensor_hw_set_led_enable(&LineSensor_hw, i + LINE_SENSOR_HW_OFFSET, true);
                }

                line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_LEFT_CH, 0U);
                line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_RIGHT_CH, 0U);
                line_sensor_hw_set_led_enable(&LineSensor_hw, LINE_SENSOR_HW_LEFT_CH, true);
                line_sensor_hw_set_led_enable(&LineSensor_hw, LINE_SENSOR_HW_RIGHT_CH, true);
            }

            //line_sensor_hw_set_led_power(&LineSensor_hw, lpos + LINE_SENSOR_HW_OFFSET, 125U);

            line_sensor_hw_led_update(&LineSensor_hw);
        }
            // if (LineTurn != LINE_NO_TURN)
            // {
            //     auto br = LedBrightness;
            //     if (CenterLineIndex > LINE_SENSOR_CENTER_INDEX)
            //     {
            //         for(auto i = CenterLineIndex; i > LINE_SENSOR_CENTER_INDEX; i--)
            //             Sensors[i].setLedPower(br);
            //     }
            //     else if (CenterLineIndex < LINE_SENSOR_CENTER_INDEX)
            //     {
            //         for(auto i = CenterLineIndex; i < LINE_SENSOR_CENTER_INDEX; i++)
            //             Sensors[i].setLedPower(br);
            //     }
                
            // }
            // Sensors[CenterLineIndex].setLedPower(LedBrightness);

        
        

        // else if (Position > LINE_SENSOR_LAST_VALUE){}
        //     line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_LEFT_CH, abs(LineHeading) / LINE_SENSOR_TURN90_VALUE);
        // else if (Position < -LINE_SENSOR_LAST_VALUE){}
        //     line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_RIGHT_CH,  abs(LineHeading) / LINE_SENSOR_TURN90_VALUE);

    }

    void StartCalibration(void)
    {
        Calibrated = false;
        CalibrationStarted = false;
    }

    void StartCalibration(const uint64_t& time)
    {
        for(IRSensor<T> &s: Sensors)
            s.ResetConfig();

        line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_LEFT_CH, 1);
        line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_RIGHT_CH, 1);
        line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_STATUS_CH, 1);
        line_sensor_hw_set_led_enable(&LineSensor_hw, LINE_SENSOR_HW_LEFT_CH, false);
        line_sensor_hw_set_led_enable(&LineSensor_hw, LINE_SENSOR_HW_RIGHT_CH, false);
        line_sensor_hw_set_led_enable(&LineSensor_hw, LINE_SENSOR_HW_STATUS_CH, true);

        line_sensor_hw_led_update(&LineSensor_hw);

        TimeStampCalibration = time;
        TimeStampCalibrationBlink = time;
        Calibrated = false;
        CalibrationStarted = true;
    }

    void RunCalibration(const uint64_t& time = TIME_U64())
    {
        if (time - TimeStampCalibration < CalibrationTime)
        {
            if (time - TimeStampCalibrationBlink > CalibrationBlinkTime)
            {
                TimeStampCalibrationBlink = time;
                line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_STATUS_CH, 1);
                line_sensor_hw_toggle_led_enable(&LineSensor_hw, LINE_SENSOR_HW_STATUS_CH);
            }

            for(uint i = 0; i < _SensorCount; i++)
            {
                Sensors[i].Calibrate(RawData[i]);
                line_sensor_hw_set_led_power(&LineSensor_hw, i + LINE_SENSOR_HW_OFFSET, Sensors[i].Compute(RawData[i]));
                line_sensor_hw_set_led_enable(&LineSensor_hw, i + LINE_SENSOR_HW_OFFSET, true);
            }

            line_sensor_hw_led_update(&LineSensor_hw);
        }
        else
        {
            line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_STATUS_CH, 0);
            line_sensor_hw_set_led_enable(&LineSensor_hw, LINE_SENSOR_HW_STATUS_CH, false);

            for(uint i = 0; i < _SensorCount; i++)
            {
                line_sensor_hw_set_led_power(&LineSensor_hw, i + LINE_SENSOR_HW_OFFSET, 0);
                line_sensor_hw_set_led_enable(&LineSensor_hw, i + LINE_SENSOR_HW_OFFSET, false);
            }

            line_sensor_hw_led_update(&LineSensor_hw);

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
