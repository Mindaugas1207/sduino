
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
#define LINE_SENSOR_THRESHOLD (0.5)
#define LINE_SENSOR_THRESHOLD_CLEARANCE (0.1)
#define LINE_SENSOR_SEGMENT_HISTERYSIS (1)
#define LINE_SENSOR_TURN_THRESHOLD (4)
#define LINE_SENSOR_TURN_RATIO_THRESHOLD (0.2)
#define LINE_SENSOR_VALUE_TO_ANGLE_RAD 0.304539116001953

#define LINE_SENSOR_AUTO_CALIBRATE (true)

typedef enum {
    COLOR_NONE,
    COLOR_BLACK,
    COLOR_WHITE
} color_t;

typedef enum
{
    TURN_NONE = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2
} LineTurnDirection_t;

template <typename T>
struct LineSegment
{
    int Start; //inclusive
    int End; //inclusive
    color_t Color;

    LineSegment(const uint& start, const uint& end, const color_t& color) : Start(start), End(end), Color(color) {}
    LineSegment(const uint& point, const color_t& color) : Start(point), End(point), Color(color) {}
    LineSegment(void) : Start(0), End(-1), Color(COLOR_NONE) {}

    bool Contains(const int& point) const
    {
        return point >= Start && point <= End;
    }
    bool Contains(const T& point) const
    {
        return point >= Start && point <= End;
    }
    bool Contains(const LineSegment& segment) const
    {
        return segment.Start >= Start && segment.End <= End; //asumes indexes can not be reversed
    }
    bool Excludes(const LineSegment& segment) const
    {
        return segment.End < Start || segment.Start > End;
    }
    bool Overlaps(const LineSegment& segment) const
    {
        return Contains(segment.Start) || Contains(segment.End);
    }
    bool Equals(const LineSegment& segment) const
    {
        return segment.Start == Start && segment.End == End && segment.Color == Color;
    }
    uint Overlap(const LineSegment& segment) const
    {
        if (segment.End < Start || segment.Start > End) return 0;
        else if (segment.Start >= Start && segment.End >  End) return End - segment.Start + 1;
        else if (segment.Start <  Start && segment.End <= End) return segment.End - Start + 1;
        else if (segment.Start <  Start && segment.End >  End) return Width();
        else return segment.Width();
    }
    bool IsCloser(const LineSegment& segment, const T& to) const
    {
        return std::abs(Center() - to) < std::abs(segment.Center() - to);
    }
    bool SameSide(const LineSegment& segment, const T& to) const
    {
        T c = Center();
        T sc = segment.Center();
        return (c <= to && sc <= to + (T)LINE_SENSOR_SEGMENT_HISTERYSIS) || (c >= to && sc >= to - (T)LINE_SENSOR_SEGMENT_HISTERYSIS);
    }
    int Width(void) const
    {
        return End - Start + 1;
    }
    T Center(void) const
    {
        return ((End - Start) * (T)0.5) + Start;
    }
    bool Exists(void) const
    {
        return Color != COLOR_NONE;
    }
};

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
        Color = COLOR_WHITE;
    }

    void Init(const Config& config)
    {
        LoadCalibration(config);
        Value = (T)(0.0);
        Color = COLOR_WHITE;
    }

    T Compute(const uint& input)
    {
        uint raw = std::clamp(input, Min, Max);

        Value = (raw - Min) * Ratio;

        Color = Value > (T)(LINE_SENSOR_THRESHOLD + LINE_SENSOR_THRESHOLD_CLEARANCE) ? COLOR_BLACK : (Value < (T)(LINE_SENSOR_THRESHOLD - LINE_SENSOR_THRESHOLD_CLEARANCE) ? COLOR_WHITE : Color);

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

template <typename T, size_t _SensorCount = LINE_SENSOR_NUM_SENSORS, size_t _LedCount = LINE_SENSOR_HW_NUM_POS_LEDS>
class LineSensor
{
    line_sensor_hw_inst_t LineSensor_hw;
    T LedMaxBrightness;
    uint32_t LedTime;
    uint32_t CalibrationTime;
    uint32_t CalibrationBlinkTime;

    std::array<IRSensor<T>, _SensorCount> Sensors;

    //std::array<float, _LedCount> LedValues;

    //std::array<uint, _SensorCount> RawData;

    bool Enabled;
    bool Calibrated;
    bool CalibrationStarted;
    bool IndicatorLedState;
    uint64_t TimeStamp;
    uint64_t TimeStampCalibration;
    uint64_t TimeStampCalibrationBlink;

    std::vector<LineSegment<T>> Segments;
    LineSegment<T> CenterSegment;
    LineTurnDirection_t TurnDirection;
    LineTurnDirection_t LastTurn;
    bool Detected;
    color_t LineColor;
    T Center;
    T AverageCenter;

    bool DisplayAnalog;

public:
    struct Config
    {
        std::template array<typename IRSensor<T>::Config, _SensorCount> SensorCalibration;
        T LedMaxBrightness;
        uint32_t CalibrationTime;
        bool Calibrated;
    };

    std::array<uint, _SensorCount> RawData;

    size_t SensorCount(void) const { return _SensorCount; }
    size_t LedCount(void) const { return _LedCount; }

    IRSensor<T>& operator[](const std::size_t& i)             { return Sensors[i]; }
    const IRSensor<T>& operator[](const std::size_t& i) const { return Sensors[i]; }

    void SetDisplayAnalog(bool enable)
    {
        DisplayAnalog = enable;
    }

    bool IsDetected(void) const
    {
        return Detected;
    }

    T LineCenter(void) const
    {
        return Center;
    }

    LineTurnDirection_t GetTurnDirection(void) const
    {
        return TurnDirection;
    }

    //void SetLed(const std::size_t& i, const float& value)
    //{
    //    LedValues[i] = value;
    //}

    int Init(const line_sensor_hw_inst_t& hw)
    {
        LineSensor_hw = hw;
        if (line_sensor_hw_init(&LineSensor_hw) != LINE_SENSOR_HW_OK) return LINE_SENSOR_ERROR;

        for(IRSensor<T> &s: Sensors)
            s.Init();

        //for(int i = 0; i < LedValues.size(); i++)
        //    LedValues[i] = 0;

        CalibrationBlinkTime = 50000;
        LedTime = 20000;

        IndicatorLedState = false;

        Segments.reserve(_SensorCount);
        TurnDirection = TURN_NONE;
        LastTurn = TURN_NONE;
        Detected = false;
        LineColor = COLOR_BLACK;
        DisplayAnalog = false;
        Center = 0;
        AverageCenter = 0;

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

        IndicatorLedState = true;

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

            UpdateLeds(time);
        }

        return LINE_SENSOR_OK;
    }

    void Compute(const uint64_t& time = TIME_U64())
    {
        for(uint i = 0; i < _SensorCount; i++)
            Sensors[i].Compute(RawData[i]);
        
        LineSegment<T> nearest = FindSegments(Segments, CenterSegment, LineColor);
        if (nearest.Exists())
        {
            if (CenterSegment.Exists())
            {
                uint overlap = nearest.Overlap(CenterSegment);
                LineTurnDirection_t turn = TURN_NONE;
                LineSegment<T> center = GetCenterFromOverlap(nearest, CenterSegment);
                if (center.Exists())
                {
                    turn = FindTurn(nearest, center);
                    if (turn != TURN_NONE) LastTurn = turn;
                    CenterSegment = center;
                }
                else
                {
                    //nera persidengimo
                    CenterSegment = nearest;
                }
            }
            else
            {
                //Pries tai nebuvo centro
                CenterSegment = nearest;
            }

            AverageCenter = ComputeAverageCenter();
            
            TurnDirection = TURN_NONE;
            Detected = true;
        }
        else
        {
            //nieko nemato
            //do left right, gyro
            if (LastTurn == TURN_LEFT)
            {
                CenterSegment = LineSegment<T>(-1, -1, LineColor);
                TurnDirection = TURN_LEFT;
                LastTurn = TURN_NONE;
            }
            else if (LastTurn == TURN_RIGHT)
            {
                CenterSegment = LineSegment<T>(_SensorCount, _SensorCount, LineColor);
                TurnDirection = TURN_RIGHT;
                LastTurn = TURN_NONE;
            }
            else if (TurnDirection == TURN_NONE) //nebuvo LEFT, RIGHT bandom ziureti i praita vidurki
            {
                if (AverageCenter < (((T)_SensorCount / 2) + (T)0.5))
                {
                    CenterSegment = LineSegment<T>(-1, -1, LineColor);
                }
                else
                {
                    CenterSegment = LineSegment<T>(_SensorCount, _SensorCount, LineColor);
                }
            }

            Detected = false;
        }

        Center = (_SensorCount - 2 * CenterSegment.Center()) * LINE_SENSOR_VALUE_TO_ANGLE_RAD / _SensorCount;
    }

    void UpdateLeds(const uint64_t& time = TIME_U64())
    {
        if (time - TimeStamp > LedTime)
        {
            TimeStamp = time;

            line_sensor_hw_set_led_power(&LineSensor_hw, LINE_SENSOR_HW_STATUS_CH, IndicatorLedState ? 1 : 0);

            if (DisplayAnalog || CalibrationStarted)
            {
                for(uint i = 0; i < _SensorCount; i++)
                    line_sensor_hw_set_led_power(&LineSensor_hw, i + 1, Sensors[i].Value * LedMaxBrightness);
                line_sensor_hw_set_led_power(&LineSensor_hw, 0, 0);
                line_sensor_hw_set_led_power(&LineSensor_hw, _SensorCount, 0);
            }
            else
            {
                for (auto i = 0; i < _SensorCount; i++)
                    line_sensor_hw_set_led_power(&LineSensor_hw, i + 1, Sensors[i].Color == COLOR_BLACK ? (0.1f * LedMaxBrightness) : 0.0f );

                for (auto i = CenterSegment.Start; i <= CenterSegment.End; i++)
                    line_sensor_hw_set_led_power(&LineSensor_hw, i + 1, 0.3f * LedMaxBrightness);

                int cled = std::clamp(CenterSegment.Center(), (T)1.0, (T)_SensorCount);

                line_sensor_hw_set_led_power(&LineSensor_hw, cled + 1, LedMaxBrightness);
            }

            line_sensor_hw_led_update(&LineSensor_hw);
        }
    }

    void StartCalibration(void)
    {
        Calibrated = false;
        CalibrationStarted = false;
        IndicatorLedState = false;
    }

    void StartCalibration(const uint64_t& time)
    {
        for(IRSensor<T> &s: Sensors)
            s.ResetConfig();

        TimeStampCalibration = time;
        TimeStampCalibrationBlink = time;
        Calibrated = false;
        CalibrationStarted = true;
        IndicatorLedState = false;
    }

    void RunCalibration(const uint64_t& time = TIME_U64())
    {
        if (time - TimeStampCalibration < CalibrationTime)
        {
            for(uint i = 0; i < _SensorCount; i++)
            {
                Sensors[i].Calibrate(RawData[i]);
                Sensors[i].Compute(RawData[i]);
            }

            if (time - TimeStampCalibrationBlink > CalibrationBlinkTime)
            {
                TimeStampCalibrationBlink = time;
                IndicatorLedState = !IndicatorLedState;
            }
        }
        else
        {
            Calibrated = true;
            CalibrationStarted = false;
            IndicatorLedState = true;
        }
    }

    int Stop(void)
    {
        line_sensor_hw_disable(&LineSensor_hw);

        IndicatorLedState = false;

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
        
        LedMaxBrightness = config.LedMaxBrightness;
        CalibrationTime = config.CalibrationTime;
        Calibrated      = config.Calibrated;
    }

	Config GetConfig(void)
    {
        Config result;

        for(uint i = 0; i < _SensorCount; i++)
            result.SensorCalibration[i] = Sensors[i].GetConfig();

        result.LedMaxBrightness = LedMaxBrightness;
        result.CalibrationTime = CalibrationTime;
        result.Calibrated = Calibrated;

        return result;
    }

private:
    bool IsSegmentNewCenter(const LineSegment<T>& newSegment, const LineSegment<T>& nearestSegment, const LineSegment<T>& currentCenterSegment, const color_t& lineColor) const
    {
        if (newSegment.Color != lineColor) return false;
        if (!newSegment.Exists()) return false;
        if (!nearestSegment.Exists()) return true;
        if (!currentCenterSegment.Exists()) return true;

        uint current_overlap = nearestSegment.Overlap(currentCenterSegment);
        uint new_overlap = newSegment.Overlap(currentCenterSegment);
        
        const T center = (T)_SensorCount / 2;

        if (new_overlap > 0)
        {
            if (new_overlap > current_overlap) return true;
            else if (new_overlap == current_overlap) return newSegment.IsCloser(nearestSegment, center);
        }
        else if (current_overlap == 0)
        {
            bool same_side_new = newSegment.SameSide(currentCenterSegment, center);
            bool same_side_current = nearestSegment.SameSide(currentCenterSegment, center);
            bool new_is_closer = newSegment.IsCloser(nearestSegment, center);
            return (same_side_current && same_side_new && new_is_closer) || (!same_side_current && (same_side_new || new_is_closer));
        }

        return false;
    }

    LineSegment<T> FindSegments(std::vector<LineSegment<T>>& segments, const LineSegment<T>& currentCenterSegment, const color_t& lineColor)
    {
        LineSegment<T> nearestSegment;
        color_t currentColor = Sensors[0].Color;
        segments.clear();
        int i_last = 0;
        int n = _SensorCount;
        T centerIndex = (T)n / 2;
        int end_index = n - 1;
        for (auto i = 0; i < n; i++)
        {
            const color_t color = Sensors[i].Color;
            if (color != currentColor)
            {
                LineSegment<T> segment = LineSegment<T>(i_last, i - 1, currentColor);
                segments.push_back(segment);
                i_last = i;
                currentColor = color;

                if (IsSegmentNewCenter(segment, nearestSegment, currentCenterSegment, lineColor)) nearestSegment = segment;
            }
            
            if (i == n - 1)
            {
                LineSegment<T> segment = LineSegment<T>(i_last, i, color);
                segments.push_back(segment);
                if (IsSegmentNewCenter(segment, nearestSegment, currentCenterSegment, lineColor)) nearestSegment = segment;
            }
        }

        return nearestSegment;
    }

    T ComputeAverageCenter(void)
    {
        T sum = 0;
        T div = 0;
        for(uint i = 0; i < _SensorCount; i++)
        {
            if (Sensors[i].Color == LineColor)
            {
                sum += i + 1;
                div += 1;
            }
        }

        return div != 0 ? sum / div : 0;
    }

    LineTurnDirection_t FindTurn(const LineSegment<T>& nearestSegment, const LineSegment<T>& centerSegment) const
    {
        T offsetLow = 0;
        T offsetHigh = 0;

        T center = centerSegment.Center();
        if (!nearestSegment.Contains(center)) return TURN_NONE; //segment doesnt contain point

        offsetLow = center - nearestSegment.Start;
        offsetHigh = nearestSegment.End - center;

        if (offsetHigh > offsetLow)
        {
            T ratio = (offsetHigh - offsetLow) / (offsetHigh + offsetLow);

            if (ratio >= (T)LINE_SENSOR_TURN_RATIO_THRESHOLD && offsetHigh > (T)LINE_SENSOR_TURN_THRESHOLD)
            {
                return TURN_LEFT;
            }
        }
        else
        {
            T ratio = (offsetLow - offsetHigh) / (offsetHigh + offsetLow);

            if (ratio >= (T)LINE_SENSOR_TURN_RATIO_THRESHOLD && offsetLow > (T)LINE_SENSOR_TURN_THRESHOLD)
            {
                return TURN_RIGHT;
            }
        }
        return TURN_NONE;
    }

    LineSegment<T> GetCenterFromOverlap(const LineSegment<T>& nearestSegment, const LineSegment<T>& currentCenterSegment) const
    {
        if (!nearestSegment.Exists()) return LineSegment<T>();
        uint c_width = currentCenterSegment.Width();
        T c_center = currentCenterSegment.Center();
        T n_center = nearestSegment.Center();
        if (nearestSegment.Overlap(currentCenterSegment) > 0)
        {
            if (nearestSegment.Width() <= c_width)
            {
                return nearestSegment;
            }
            else if (c_center < n_center)
            {
                int start = nearestSegment.Start;
                int end = nearestSegment.Start + c_width - 1;
                if (end >= _SensorCount)
                {
                    end = _SensorCount - 1;
                    start = end - c_width + 1;
                }

                return LineSegment<T>(start, end, nearestSegment.Color);
            }
            else if (c_center > n_center)
            {
                int start = nearestSegment.End - c_width + 1;
                int end = nearestSegment.End;
                if (start < 0)
                {
                    start = 0;
                    end = start + c_width - 1;
                }

                return LineSegment<T>(start, end, nearestSegment.Color);
            }
            else
            {
                return currentCenterSegment;
            }
        }
        else
        {
            return LineSegment<T>();
        }
    }
};

#endif
