
#ifndef INC_DISTANCE_SENSOR_HPP_
#define INC_DISTANCE_SENSOR_HPP_

#include "sensorx.h"

#define DISTANCE_SENSOR_OK PICO_OK
#define DISTANCE_SENSOR_ERROR PICO_ERROR_GENERIC

class DistanceSensor
{
    VL53L0X_Dev_t sensor_hw;

    uint16_t Value = 0;

    bool Enabled;
    uint64_t TimeStamp;
    
public:
    struct Config
    {
        
    };

    int Init(const VL53L0X_Dev_t& hw)
    {
        sensor_hw = hw;
        if (SENSORX_VL53L0X_Init(&sensor_hw) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;

        VL53L0X_Config_t config;

        if (SENSORX_VL53L0X_GetConfig(&sensor_hw, &config) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;
        if (SENSORX_VL53L0X_RunCalibrations(&sensor_hw, &config, VL53L0X_CALIBRATE_SPADS) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;
        if (SENSORX_VL53L0X_RunCalibrations(&sensor_hw, &config, VL53L0X_CALIBRATE_REF) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;

        config.RANGING_MODE = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
        config.INTERRUPT.MODE = VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY;
        config.INTERRUPT.POLARITY = VL53L0X_INTERRUPTPOLARITY_HIGH;

        if (SENSORX_VL53L0X_SetConfig(&sensor_hw, &config) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;
        if (SENSORX_VL53L0X_PrintAll(&sensor_hw, &config) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;

        gpio_init(sensor_hw.Pin);
        gpio_set_dir(sensor_hw.Pin, GPIO_IN);
        gpio_set_pulls(sensor_hw.Pin, false, true);

        Stop();

        return DISTANCE_SENSOR_OK;
    }
    
    int Init(const VL53L0X_Dev_t& hw, const Config& config)
    {
        if (Init(hw) != DISTANCE_SENSOR_OK) return DISTANCE_SENSOR_ERROR;

        LoadConfig(config);

        return DISTANCE_SENSOR_OK;
    }

    uint16_t GetDistance(void) { return Value; }

    int Start(const uint64_t& time = TIME_U64())
    {
        if (Enabled) return DISTANCE_SENSOR_OK;

        if (SENSORX_VL53L0X_StartMeasurement(&sensor_hw) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;

        Enabled = true;
        TimeStamp = time;

        return DISTANCE_SENSOR_OK;
    }

    int Update(const uint64_t& time = TIME_U64())
    {
        if (Enabled)
        {
            if (gpio_get(sensor_hw.Pin))
            {
                int error = SENSORX_OK;
                VL53L0X_RangingMeasurementData_t data;
                error = SENSORX_VL53L0X_GetMeasurementData(&sensor_hw, &data);
                error = SENSORX_VL53L0X_ClearInterrupt(&sensor_hw);
                if(error != SENSORX_OK) return DISTANCE_SENSOR_ERROR;
                
                //SENSORX_VL53L0X_PrintMeasurementData(&sensor_hw, data);
                Value = data.RangeStatus == VL53L0X_RANGESTATUS_RANGEVALID ? data.RangeMilliMeter : 10000;
            }
        }

        return DISTANCE_SENSOR_OK;
    }

    int Stop(void)
    {
        if (SENSORX_VL53L0X_StopMeasurement(&sensor_hw) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;
        if (SENSORX_VL53L0X_ClearInterrupt(&sensor_hw) != SENSORX_OK) return DISTANCE_SENSOR_ERROR;

        Enabled = false;
        
        return DISTANCE_SENSOR_OK;
    }

    void LoadConfig(const Config& config)
    {
    }

	Config GetConfig(void)
    {
        return {
        };
    }
};

#endif
