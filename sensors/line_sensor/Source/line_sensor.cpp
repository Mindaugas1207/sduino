
#include "line_sensor.hpp"

int Sensor_s::init(uint _Pos, line_sesnor_hw_inst_t *_hw_inst)
{
    Pos = _Pos;
    hw_inst = _hw_inst;
    Min = std::numeric_limits<uint>::max();
    Max = std::numeric_limits<uint>::min();
    Raw = 0u;
    Offset = 0.0f;
    Span = 1.0f;
    ThresholdHigh = 0.5f + LINE_SENSOR_THRESHOLD_CLEARANCE;
    ThresholdLow = 0.5f - LINE_SENSOR_THRESHOLD_CLEARANCE;
    Value = 0.0f;
    Detected = false;

    setEmitterEnable(false);
    setEmitterPower(0u);

    setLedEnable(false);
    setLedPower(0u);

    return LINE_SENSOR_OK;
}

void Sensor_s::setEmitterEnable(bool _Enabled) { line_sensor_hw_set_emitter_enable(hw_inst, Pos, _Enabled); }
void Sensor_s::setEmitterPower(float _Power) { line_sensor_hw_set_emitter_power(hw_inst, Pos, _Power); }
void Sensor_s::setEmitterPower(uint _PWM) { line_sensor_hw_set_emitter_power_pwm(hw_inst, Pos, _PWM); }

void Sensor_s::setLedEnable(bool _Enabled) { line_sensor_hw_set_led_enable(hw_inst, Pos, _Enabled); }
void Sensor_s::setLedPower(float _Power) { line_sensor_hw_set_led_power(hw_inst, Pos, _Power); }
void Sensor_s::setLedPower(uint _PWM) { line_sensor_hw_set_led_power_pwm(hw_inst, Pos, _PWM); }

void Sensor_s::startCalibration(void)
{
    Min = std::numeric_limits<uint>::max();
    Max = std::numeric_limits<uint>::min();
    Offset = 0.0f;
    Span = 1.0f;
}

void Sensor_s::calibrate(uint & _Input)
{
    Min = std::min(_Input, Min);
    Max = std::max(_Input, Max);

    Offset = (float)Min;
    Span = Max != Min ? (float)(Max - Min) : 1.0f;
}

std::tuple<float, bool> Sensor_s::compute(uint & _Input)
{
    Raw = std::min(_Input, Max);
    Value = ((float)Raw - Offset) / Span;
    Detected = Value > ThresholdHigh ? true : (Value < ThresholdLow ? false : Detected);
    return {Value, Detected};
}

void Sensor_s::loadConfiguration(Config_t _Config)
{
    Min = _Config.Min;
    Max = _Config.Max;

    Offset = (float)Min;
    Span = Max != Min ? (float)(Max - Min) : 1.0f;
}

Sensor_s::Config_t Sensor_s::getConfiguration(void)
{
    return {
        .Min = Min,
        .Max = Max,
    }; 
}

int LineSensor_s::init(line_sesnor_hw_inst_t *_hw_inst) {
    hw_inst = _hw_inst;
    Position = 0.0f;
    OffsetAverage = 0.0f;
    LastFrameHeading = 0.0f;
    LineHeading = 0.0f;
    CenterLineIndex = LINE_SENSOR_CENTER_INDEX;
    Calibrated = false;
    Detected = false;
    SensorTimedOut = false;
    Available = false;
    DisplayAnalog = false;
    LiveUpdate = false;
    LineRange = LINE_CENTER;
    LineTurn = LINE_NO_TURN;

    if (line_sensor_hw_init(hw_inst) == LINE_SENSOR_HW_ERROR) return LINE_SENSOR_ERROR;

    for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++) { Sensors[i].init(i, hw_inst); }

    LedUpdateTimeout = get_absolute_time();
    LedBlinkTimeout = get_absolute_time();
    CalibrationTimeout = get_absolute_time();
    SensorTimeout = get_absolute_time();

    setEmittersPower(0.0f);
    setLedsBrightness(0.0f);
    setTurnGain1(1.0f);
    setTurnGain2(1.0f);

    return LINE_SENSOR_OK;
}

void LineSensor_s::setEmittersEnable(bool _Enabled)
{
    for(auto & sensor : Sensors) sensor.setEmitterEnable(_Enabled);
    line_sensor_hw_set_emitter_enable(hw_inst, LINE_SENSOR_STROBE_EMITER, _Enabled);
    line_sensor_hw_emitter_update(hw_inst);
}

void LineSensor_s::setEmittersPower(float _Power)
{
    EmittersPower = _Power;
    for(auto & sensor : Sensors) sensor.setEmitterPower(_Power);
    line_sensor_hw_set_emitter_power(hw_inst, LINE_SENSOR_STROBE_EMITER, _Power);
    line_sensor_hw_emitter_update(hw_inst);
}

float LineSensor_s::getEmittersPower() { return EmittersPower; }

void LineSensor_s::setLedsEnable(bool _Enabled)
{
    for(auto & sensor : Sensors) sensor.setLedEnable(_Enabled);
    line_sensor_hw_set_led_enable(hw_inst, LINE_SENSOR_EDGE_LED_FIRST, _Enabled);
    line_sensor_hw_set_led_enable(hw_inst, LINE_SENSOR_EDGE_LED_LAST, _Enabled);
    line_sensor_hw_led_update(hw_inst);
}

void LineSensor_s::setLedsBrightness(float _Brightness)
{
    LedBrightness = _Brightness;
    line_sensor_hw_set_led_power(hw_inst, LINE_SENSOR_STATUS_LED, _Brightness);
}

float LineSensor_s::getLedsBrightness() { return LedBrightness; }

void LineSensor_s::calibrate(std::array<uint, LINE_SENSOR_NUM_SENSORS> & _Input, absolute_time_t _TimeNow)
{
    for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++)
    {
        Sensors[i].calibrate(_Input[i]);
    }

    if (to_us_since_boot(_TimeNow) > to_us_since_boot(CalibrationTimeout))
    {
        Calibrated = true;
        
        // for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++)
        // {
        //     auto cfg = Sensors[i].getConfiguration();
        //     printf("(%d, %d) ", cfg.Max, cfg.Min);
        // }
        // printf("\n");
    }
}

void LineSensor_s::startCalibration(uint32_t duration_ms)
{
    for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++)
    {
        Sensors[i].startCalibration();
    }

    CalibrationTimeout = make_timeout_time_us(1000 * duration_ms);   
    Calibrated = false;
}

void LineSensor_s::startCalibration()
{
    for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++)
    {
        Sensors[i].startCalibration();
    }

    CalibrationTimeout = make_timeout_time_us(1000 * LINE_SENSOR_CALIBRATION_TIME);   
    Calibrated = false;
}

void LineSensor_s::start(void) { line_sensor_hw_start_read(); }
void LineSensor_s::stop(void) { line_sensor_hw_stop_read(); }

bool LineSensor_s::process(absolute_time_t _TimeNow)
{
    Available = line_sensor_hw_is_new_data_available();

    if (!Available)
        return false;
    
    std::array<uint, LINE_SENSOR_NUM_SENSORS> buffer;
    line_sensor_hw_read(buffer.data());

    if (!Calibrated)
        calibrate(buffer, _TimeNow);
    
    std::tie(Position, Detected) = compute(_TimeNow, buffer);
    return true;
}

std::tuple<float, bool> LineSensor_s::compute(absolute_time_t _TimeNow, std::array<uint, LINE_SENSOR_NUM_SENSORS> & _input)
{
    int _ClosestEdge = CenterLineIndex;
    int _ClosestDistance = std::numeric_limits<int>::max();
    int _EdgeCount = 0;
    //Find all edges
    for(auto i = LINE_SENSOR_FIRST_INDEX; i <= LINE_SENSOR_LAST_INDEX; i++)
    {
        auto [_Value, _Detected] = Sensors[i].compute(_input[i]);

        if (!_Detected)
            continue;
        
        _EdgeCount++;

        int _EdgeDistance = 2 * std::abs(CenterLineIndex - i) + std::abs(LINE_SENSOR_CENTER_INDEX - i);
        if (_EdgeDistance < _ClosestDistance)
        {
            _ClosestEdge = i;
            _ClosestDistance = _EdgeDistance;
        }
    }

    float _Position = 0.0f;
    bool _Detected = Detected;
    auto _EdgeDetected = _EdgeCount > 0;
    auto _CenterDetected = Sensors[CenterLineIndex].getDetected();
    auto _InRange = (CenterLineIndex < LINE_SENSOR_CENTER_INDEX && _ClosestEdge < LINE_SENSOR_CENTER_INDEX) ||
                    (CenterLineIndex > LINE_SENSOR_CENTER_INDEX && _ClosestEdge > LINE_SENSOR_CENTER_INDEX);
    auto _TimedOut = to_us_since_boot(_TimeNow) > to_us_since_boot(CenterLineTimeout);
    auto _TurnDeteced = LineTurn != LINE_NO_TURN;
    if (_EdgeDetected)
    {
        if ((Detected && (!_CenterDetected || _TimedOut) && !_TurnDeteced) ||
            (!Detected && ( _CenterDetected || _InRange )))
        {
            if (Detected && CenterLineIndex == _ClosestEdge && _TimedOut && _CenterDetected)
            {
                CenterLineIndex = CenterLineIndex < LINE_SENSOR_CENTER_INDEX ? CenterLineIndex + 1 :
                                  CenterLineIndex > LINE_SENSOR_CENTER_INDEX ? CenterLineIndex - 1 :
                                  LINE_SENSOR_CENTER_INDEX;
            }
            else
            {
                CenterLineIndex = _ClosestEdge;
            }
            CenterLineTimeout = delayed_by_us(_TimeNow, LINE_SENSOR_CENTERLINE_TIMEOUT_US);
            _CenterDetected = true;

            if (CenterLineIndex < LINE_SENSOR_INDEX_RIGHT_TH)
            {
                LineRange = LINE_RIGHT;
            }
            else if (CenterLineIndex > LINE_SENSOR_INDEX_LEFT_TH)
            {
                LineRange = LINE_LEFT;
            }
            else
            {
                LineRange = LINE_CENTER;
            }

            _Detected = true;
        }

        if (_Detected && _CenterDetected)
        {
            int AnalogLineStart = std::max(CenterLineIndex - LINE_SENSOR_ANALOG_WIDTH / 2, LINE_SENSOR_FIRST_INDEX);
            int AnalogLineEnd = std::min(CenterLineIndex + LINE_SENSOR_ANALOG_WIDTH / 2, LINE_SENSOR_LAST_INDEX);
            float AnalogLinePosition = 0.0f;
            float AnalogLineDev = 0.0f;

            for (auto i = AnalogLineStart; i <= AnalogLineEnd; i++)
            {
                float Svalue = Sensors[i].getValue();
                AnalogLinePosition += Svalue * (i + 1);
                AnalogLineDev += Svalue;
            }

            AnalogLinePosition = ((float)LINE_SENSOR_CENTER_INDEX - (AnalogLinePosition / AnalogLineDev) + 1.0f) / (float)LINE_SENSOR_CENTER_INDEX;

            int OffsetCenterLow = 0;
            int OffsetCenterHigh = 0;
            int OffsetDifference;
            for (auto i = CenterLineIndex - 1; i >= LINE_SENSOR_FIRST_INDEX; i--)
                if (Sensors[i].getDetected())
                    OffsetCenterLow ++;

            for (auto i = CenterLineIndex + 1; i <= LINE_SENSOR_LAST_INDEX; i++)
                if (Sensors[i].getDetected())
                    OffsetCenterHigh ++;

            OffsetDifference = OffsetCenterLow - OffsetCenterHigh;

            if (OffsetAverage < LINE_SENSOR_STEP_VALUE / 2.0f && OffsetAverage > -LINE_SENSOR_STEP_VALUE / 2.0f)
                OffsetAverage = 0.0f;

            if (OffsetDifference > LINE_SENSOR_OFFSET_THRESHOLD)
                OffsetAverage += LINE_SENSOR_STEP_VALUE;
            else if (OffsetDifference < -LINE_SENSOR_OFFSET_THRESHOLD)
                OffsetAverage -= LINE_SENSOR_STEP_VALUE;
            else
                OffsetAverage *= LINE_SENSOR_OFFSET_DECAY_COEF;

            if (OffsetAverage < -(LINE_SENSOR_STEP_VALUE + LINE_SENSOR_STEP_VALUE * 0.1f))
                LineTurn = LINE_TURN_LEFT;
            else if (OffsetAverage > (LINE_SENSOR_STEP_VALUE + LINE_SENSOR_STEP_VALUE * 0.1f))
                LineTurn = LINE_TURN_RIGHT;
            else
                LineTurn = LINE_NO_TURN;

            // if (LineTurn != LINE_NO_TURN)
            //     TLock = true;

            _Position = AnalogLinePosition;

            // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(DBG_PrintTimeout) || !Detected)
            // {
            //     printf("cidx: %d, ce: %d, low: %d, high: %d, diff: %d, avg: %f, Turn: %s\n",
            //         CenterLineIndex,
            //         _ClosestEdge,
            //         OffsetCenterLow,
            //         OffsetCenterHigh, 
            //         OffsetDifference,
            //         OffsetAverage,
            //         LineTurn == LineSensor_s::LineTurn_t::LINE_NO_TURN ? "NONE  " :
            //         LineTurn == LineSensor_s::LineTurn_t::LINE_TURN_LEFT ? "LEFT  " : "RIGHT "
            //     );
            //     DBG_PrintTimeout = make_timeout_time_ms(200);
            // }
        }
    }
    else
    {
        switch (LineTurn)
        {
        case LINE_NO_TURN:
            switch (LineRange)
            {
            case LINE_CENTER:
                _Position = Position;
                break;
            case LINE_LEFT:
                _Position = -LINE_SENSOR_EDGE_VALUE;
                break;
            case LINE_RIGHT:
                _Position =  LINE_SENSOR_EDGE_VALUE;
                break;
            }
            break;
        case LINE_TURN_LEFT:
            LineRange = LINE_LEFT;
            CenterLineIndex = LINE_SENSOR_LAST_INDEX;
            _Position = -LINE_SENSOR_TURN90_VALUE;
            break;
        case LINE_TURN_RIGHT:
            LineRange = LINE_RIGHT;
            CenterLineIndex = LINE_SENSOR_FIRST_INDEX;
            _Position =  LINE_SENSOR_TURN90_VALUE;
            break;
        }

        OffsetAverage *= LINE_SENSOR_OFFSET_DECAY_COEF;
        CenterLineTimeout = delayed_by_us(_TimeNow, LINE_SENSOR_CENTERLINE_TIMEOUT_US);
        //TLock = false;
        _Detected = false;
        // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(DBG_PrintTimeout) || Detected)
        // {
        //     printf("cidx: %d, ce: %d, avg: %f, Turn: %s\n",
        //         CenterLineIndex,
        //         _ClosestEdge,
        //         OffsetAverage,
        //         LineTurn == LineSensor_s::LineTurn_t::LINE_NO_TURN ? "NONE  " :
        //         LineTurn == LineSensor_s::LineTurn_t::LINE_TURN_LEFT ? "LEFT  " : "RIGHT "
        //     );
        //     DBG_PrintTimeout = make_timeout_time_ms(200);
        // }
    }

    return {_Position, _Detected};
}

float unwrap(float previous_angle, float new_angle) {
    float d = new_angle - previous_angle;
    d = d > M_PI ? d - (float)(2 * M_PI) : (d < -M_PI ? d + (float)(2 * M_PI) : d);
    return previous_angle + d;
}

float LineSensor_s::computeLineHeading(float _FrameHeading)
{
    float h = unwrap(ph, _FrameHeading);
    ph = h;
    
    if (Detected)
    {
        LastFrameHeading = h;
        LineHeading = Position;
    }
    else
    {
        auto HeadingChange = (h - LastFrameHeading) * LINE_SENSOR_ANGLE_RAD_TO_VALUE;
        if (Position < (LINE_SENSOR_EDGE_VALUE - LINE_SENSOR_STEP_VALUE / 2) && Position > -(LINE_SENSOR_EDGE_VALUE - LINE_SENSOR_STEP_VALUE / 2))
        {
            //off sensor (center)
            if (Position > 0.0f)
                LineHeading = std::clamp(HeadingChange + Position, -LINE_SENSOR_EDGE_VALUE/2, LINE_SENSOR_EDGE_VALUE/2);
            else
                LineHeading = std::clamp(HeadingChange + Position, -LINE_SENSOR_EDGE_VALUE/2, LINE_SENSOR_EDGE_VALUE/2);
        }
        else if (Position < (LINE_SENSOR_TURN90_VALUE - LINE_SENSOR_STEP_VALUE / 2) && Position > -(LINE_SENSOR_TURN90_VALUE - LINE_SENSOR_STEP_VALUE / 2))
        {
            
            //off edge
            if (Position > 0.0f)
                LineHeading = std::clamp(HeadingChange + LINE_SENSOR_EDGE_VALUE * TurnGain1, LINE_SENSOR_EDGE_VALUE/2, LINE_SENSOR_TURN90_VALUE);
            else
                LineHeading = std::clamp(HeadingChange - LINE_SENSOR_EDGE_VALUE * TurnGain1, -LINE_SENSOR_TURN90_VALUE, -LINE_SENSOR_EDGE_VALUE/2);
        }
        else
        {
            //90*
            if (Position > 0.0f)
                LineHeading = std::clamp(HeadingChange + LINE_SENSOR_TURN90_VALUE * TurnGain2, LINE_SENSOR_EDGE_VALUE, LINE_SENSOR_TURN90_VALUE);
            else
                LineHeading = std::clamp(HeadingChange - LINE_SENSOR_TURN90_VALUE * TurnGain2, -LINE_SENSOR_TURN90_VALUE, -LINE_SENSOR_EDGE_VALUE);
        }
        
    }

    return LineHeading;
}

void LineSensor_s::updateLeds(absolute_time_t _TimeNow)
{
    // Indicators
    if (to_us_since_boot(_TimeNow) < to_us_since_boot(LedUpdateTimeout) && !LiveUpdate)
        return;

    if (DisplayAnalog || !Calibrated)
    {
        for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++)
            Sensors[i].setLedPower(Sensors[i].getValue() * LedBrightness);
    }
    else
    {
        for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++)
            Sensors[i].setLedPower(0U);
        if (Detected)
            Sensors[CenterLineIndex].setLedPower(LedBrightness);
    }

    line_sensor_hw_set_led_power_pwm(hw_inst, LINE_SENSOR_EDGE_LED_FIRST, 0U);
    line_sensor_hw_set_led_power_pwm(hw_inst, LINE_SENSOR_EDGE_LED_LAST, 0U);
    line_sensor_hw_set_led_enable(hw_inst, LINE_SENSOR_STATUS_LED, false);
    
    if (!Calibrated)
    {
        if (to_us_since_boot(_TimeNow) > to_us_since_boot(LedBlinkTimeout))
        {
            line_sensor_hw_toggle_led_enable(hw_inst, LINE_SENSOR_STATUS_LED);
            LedBlinkTimeout = delayed_by_us(_TimeNow, 1000000 / LINE_SENSOR_LED_BLINK_FREQUENCY_CALIB);
        }
    }
    else if (SensorTimedOut)
    {
        line_sensor_hw_set_led_power(hw_inst, LINE_SENSOR_EDGE_LED_FIRST, LedBrightness);
        line_sensor_hw_set_led_power(hw_inst, LINE_SENSOR_EDGE_LED_LAST, LedBrightness);
    }
    else if (Position > LINE_SENSOR_LAST_VALUE)
        line_sensor_hw_set_led_power(hw_inst, LINE_SENSOR_EDGE_LED_FIRST, abs(LineHeading) / LINE_SENSOR_TURN90_VALUE);
    else if (Position < -LINE_SENSOR_LAST_VALUE)
        line_sensor_hw_set_led_power(hw_inst, LINE_SENSOR_EDGE_LED_LAST,  abs(LineHeading) / LINE_SENSOR_TURN90_VALUE);
    
    LedUpdateTimeout = delayed_by_us(_TimeNow, 1000000 / LINE_SENSOR_LED_UPDATE_FREQUENCY);
    line_sensor_hw_led_update(hw_inst);
}

void LineSensor_s::loadConfiguration(Config_t _Config)
{
    setEmittersPower(_Config.EmittersPower);
    setLedsBrightness(_Config.LedBrightness);
    setTurnGain1(_Config.TurnGain1);
    setTurnGain2(_Config.TurnGain2);

    Calibrated = _Config.Calibrated;

    if (!Calibrated)
        return;
    
    for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++)
        Sensors[i].loadConfiguration(_Config.SensorConfig[i]);
}

LineSensor_s::Config_t LineSensor_s::getConfiguration(void)
{
    Config_t _Config = {
        .EmittersPower = getEmittersPower(),
        .LedBrightness = getLedsBrightness(),
        .TurnGain1 = getTurnGain1(),
        .TurnGain2 = getTurnGain2(),
        .Calibrated = Calibrated,
    };

    for(auto i = 0; i < LINE_SENSOR_NUM_SENSORS; i++)
        _Config.SensorConfig[i] = Sensors[i].getConfiguration();

    return _Config;
}

