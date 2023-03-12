
#ifndef INC_LINE_SENSOR_HPP_
#define INC_LINE_SENSOR_HPP_

#include "stdio.h"
#include "pico/stdlib.h"
#include "line_sensor_hw.h"
#include <array>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <tuple>


#define LINE_SENSOR_NUM_SENSORS LINE_SENSOR_HW_NUM_SENSORS
#define LINE_SENSOR_CENTER_INDEX (LINE_SENSOR_NUM_SENSORS / 2)
#define LINE_SENSOR_FIRST_INDEX 0
#define LINE_SENSOR_LAST_INDEX (LINE_SENSOR_NUM_SENSORS - 1)
#define LINE_SENSOR_LAST_VALUE 1.0f
#define LINE_SENSOR_STEP_VALUE (2.0f * LINE_SENSOR_LAST_VALUE / LINE_SENSOR_LAST_INDEX)
#define LINE_SENSOR_MAX_VALUE (LINE_SENSOR_LAST_VALUE * 35.0f)
#define LINE_SENSOR_OFFLINE_MIN_VALUE (LINE_SENSOR_LAST_VALUE + LINE_SENSOR_STEP_VALUE)
#define LINE_SENSOR_EDGE_VALUE (LINE_SENSOR_LAST_VALUE + LINE_SENSOR_STEP_VALUE)
#define LINE_SENSOR_MAX_RIGHT LINE_SENSOR_MAX_VALUE
#define LINE_SENSOR_MAX_LEFT -LINE_SENSOR_MAX_VALUE
#define LINE_SENSOR_EDGE_LED_FIRST LINE_SENSOR_HW_RIGHT_CH
#define LINE_SENSOR_EDGE_LED_LAST LINE_SENSOR_HW_LEFT_CH
#define LINE_SENSOR_STROBE_EMITER LINE_SENSOR_HW_STRB_CH
#define LINE_SENSOR_STATUS_LED LINE_SENSOR_HW_STATUS_CH
#define LINE_SENSOR_THRESHOLD_CLEARANCE 0.1f
#define LINE_SENSOR_ANALOG_WIDTH 5
#define LINE_SENSOR_OFFSET_DECAY_COEF 0.95f
#define LINE_SENSOR_OFFSET_THRESHOLD 3
#define LINE_SENSOR_LED_UPDATE_FREQUENCY 30
#define LINE_SENSOR_LED_BLINK_FREQUENCY_CALIB 10
#define LINE_SENSOR_CALIBRATION_TIME 3500
#define LINE_SENSOR_TIMEOUT_TIME 1000

#define LINE_SENSOR_MAX_ANGLE_RAD 1.570796326794900f
#define LINE_SENSOR_POS_TO_ANGLE_RAD 0.043505588000279f
#define LINE_SENSOR_VALUE_TO_ANGLE_RAD 0.304539116001953f
#define LINE_SENSOR_ANGLE_RAD_TO_POS (1 / LINE_SENSOR_POS_TO_ANGLE_RAD)
#define LINE_SENSOR_ANGLE_RAD_TO_VALUE (1 / LINE_SENSOR_VALUE_TO_ANGLE_RAD)
#define LINE_SENSOR_LAST_ANGLE_RAD (LINE_SENSOR_LAST_VALUE * LINE_SENSOR_VALUE_TO_ANGLE_RAD)
#define LINE_SENSOR_EDGE_ANGLE_RAD (LINE_SENSOR_EDGE_VALUE * LINE_SENSOR_VALUE_TO_ANGLE_RAD)
#define LINE_SENSOR_TURN90_ANGLE_RAD ((float)(M_PI / 2))
#define LINE_SENSOR_TURN90_VALUE (LINE_SENSOR_TURN90_ANGLE_RAD * LINE_SENSOR_ANGLE_RAD_TO_VALUE)




#define LINE_SENSOR_OK PICO_OK
#define LINE_SENSOR_ERROR PICO_ERROR_GENERIC

class Sensor_s {
    struct Config_s {
        uint Min;
        uint Max;
    };

    line_sesnor_hw_inst_t *hw_inst;
    float Offset;
    float Span;
    float ThresholdHigh;
    float ThresholdLow;
    float Value;
    uint Pos;
    uint Raw;
    uint Min;
    uint Max;
    bool Detected;
public:
    typedef Config_s Config_t;

    int init(uint _Pos, line_sesnor_hw_inst_t *_hw_inst);

    float & getValue(void) { return Value; }
    bool & getDetected(void) { return Detected; }

    void setEmitterEnable(bool _Enabled);
	void setEmitterPower(float _Power);
    void setEmitterPower(uint _PWM);

    void setLedEnable(bool _Enabled);
    void setLedPower(float _Power);
    void setLedPower(uint _PWM);

    void startCalibration(void);
    void calibrate(uint & _Input);
	
    std::tuple<float, bool> compute(uint & _Input);

    void loadConfiguration(Config_t _Config);
    Config_t getConfiguration(void);
};

class LineSensor_s {
    struct Config_s {
        std::array<Sensor_s::Config_t, LINE_SENSOR_NUM_SENSORS> SensorConfig;
        float EmittersPower;
        float LedBrightness;
        float TurnGain1;
        float TurnGain2;
        bool Calibrated;
    };

	std::array<Sensor_s, LINE_SENSOR_NUM_SENSORS> Sensors;
    line_sesnor_hw_inst_t *hw_inst;
    absolute_time_t LedUpdateTimeout;
    absolute_time_t LedBlinkTimeout;
    absolute_time_t CalibrationTimeout;
    absolute_time_t SensorTimeout;

    float Position;
    float OffsetAverage;
    float EmittersPower;
    float LedBrightness;
    float TurnGain1;
    float TurnGain2;
    float LastFrameHeading;
    float LineHeading;
    float ph;

    int CenterLineIndex;
    int LastCenterLineIndex;
    
    bool Detected;
    bool SensorTimedOut;
    bool Available;
    bool Calibrated;
public:
    typedef Config_s Config_t;

    int init(line_sesnor_hw_inst_t *_hw_inst);

    bool isCalibrated(void) { return Calibrated; }
    bool available(void) { return Available; }
    bool isTimedOut(void) { return SensorTimedOut; }

    void setTurnGain1(float _TurnGain1) { TurnGain1 = _TurnGain1; }
    float getTurnGain1() { return TurnGain1; }
    void setTurnGain2(float _TurnGain2) { TurnGain2 = _TurnGain2; }
    float getTurnGain2() { return TurnGain2; }
	
    void setEmittersEnable(bool _Enabled);
    void setEmittersPower(float _EmittersPower);
    float getEmittersPower();

    void setLedsEnable(bool _Enabled);
    void setLedsBrightness(float _Brightness);
    float getLedsBrightness();

    void calibrate(std::array<uint, LINE_SENSOR_NUM_SENSORS> & _Input, absolute_time_t _TimeNow);
    void startCalibration(uint32_t _Duration_ms);
    void startCalibration();

    void start(void);
    void stop(void);
    void process(absolute_time_t _TimeNow);

    std::tuple<float, bool> compute(std::array<uint, LINE_SENSOR_NUM_SENSORS> & _input);
    float computeLineHeading(float _FrameHeading);
    bool isDetected(void) {return Detected;};

    void updateLeds(absolute_time_t _TimeNow);

    void loadConfiguration(Config_t _Config);
	Config_t getConfiguration(void);
};



#endif
