
#ifndef INC_IMU_HPP_
#define INC_IMU_HPP_

#include "imu_hw.h"
#include "vmath.hpp"
#include "time_hw.h"

#define IMU_OK PICO_OK
#define IMU_ERROR PICO_ERROR_GENERIC

#define IMU_AUTO_CALIBRATE (true)

template<typename T>
class IMU
{
    imu_hw_inst_t IMU_hw;
    uint32_t CalibrationTime;

    struct {
        vmath::vect_t<T> Raw;
        vmath::vect_t<T> Value;
        vmath::vect_t<T> Bias;
        vmath::vect_t<T> Accum;
    } Accelerometer;

    struct {
        vmath::vect_t<T> Raw;
        vmath::vect_t<T> Value;
        vmath::vect_t<T> Bias;
        vmath::vect_t<T> Accum;
    } Gyroscope;

    vmath::euler_t<T> Orientation;

    int AccumulatorSamples;

    vmath::MadgwickFilter<T> Filter;

    bool Enabled;
    bool Calibrated;
    bool CalibrationStarted;
    uint64_t TimeStamp;
    uint64_t TimeStampCalibration;

    T GetDeltaT(const uint64_t& time)
    {
        T dT = (T)(time - TimeStamp) / 1000000;
        TimeStamp = time;

        return dT;
    }

    void Compute(const uint64_t& time, const T& deltaT)
    {
        Gyroscope.Value     = Gyroscope.Raw - Gyroscope.Bias;
        Accelerometer.Value = Accelerometer.Raw - Accelerometer.Bias;
        
        auto quat = Filter.Compute(Gyroscope.Value, Accelerometer.Value, deltaT);
        
        Orientation = quat.EulerAngles();
    }

    void RunCalibration(const uint64_t& time)
    {
        if (time - TimeStampCalibration < CalibrationTime)
        {
            Gyroscope.Accum     += Gyroscope.Raw;
            Accelerometer.Accum += Accelerometer.Raw;
            AccumulatorSamples++;
        }
        else
        {
            Gyroscope.Bias     = Gyroscope.Accum     / AccumulatorSamples;
            Accelerometer.Bias = Accelerometer.Accum / AccumulatorSamples;

            if(Accelerometer.Bias.Z > (T)G_EARTH / 3) Accelerometer.Bias.Z -= (T)G_EARTH;  // Remove gravity from the z-axis accelerometer bias calculation
            else Accelerometer.Bias.Z += (T)G_EARTH;

            Calibrated = true;
            CalibrationStarted = false;
            
            //printf("DBG:IMU_ACCEL_BIAS(%f,%f,%f)\n", Accelerometer.Bias.X, Accelerometer.Bias.Y, Accelerometer.Bias.Z);
            //printf("DBG:IMU_GYRO_BIAS(%f,%f,%f)\n", Gyroscope.Bias.X, Gyroscope.Bias.Y, Gyroscope.Bias.Z);
        }
    }
    
public:
    struct Config
    {
        uint32_t CalibrationTime;
        T FilterBeta;
        vmath::vect_t<T> GyroBias;
        vmath::vect_t<T> AccelBias;
        bool Calibrated;
    };

    int Init(const imu_hw_inst_t& hw)
    {
        IMU_hw = hw;
        if (imu_hw_init(&IMU_hw) != BMI08X_OK) return IMU_ERROR;

        Filter.Init();

        Stop();

        return IMU_OK;
    }
    
    int Init(const imu_hw_inst_t& hw, const Config& config)
    {
        if (Init(hw) != IMU_OK) return IMU_ERROR;

        LoadConfig(config);

        return IMU_OK;
    }

    auto GetOrientation(void) { return Orientation; }
    bool IsCalibrated(void)   { return Calibrated;  }

    int Start(const uint64_t& time = TIME_U64())
    {
        if (Enabled) return IMU_OK;

        //imu_hw_enable(&IMU_hw);

        Reset();

        Enabled = true;
        TimeStamp = time;
        TimeStampCalibration = time;

        return IMU_OK;
    }

    int Update(const uint64_t& time = TIME_U64())
    {
        return Update(time, GetDeltaT(time));
    }

    int Update(const uint64_t& time, const T& deltaT)
    {
        if (Enabled)
        {
            imu_hw_data_t data;
            if (imu_hw_read_data(&IMU_hw, &data) == IMU_HW_OK)
            {
                Gyroscope.Raw     = { data.Gyroscope.X,     data.Gyroscope.Y,     data.Gyroscope.Z     };
                Accelerometer.Raw = { data.Accelerometer.X, data.Accelerometer.Y, data.Accelerometer.Z };

                if (CalibrationStarted)
                {
                    RunCalibration(time);
                }
            }

            if (Calibrated)
            {
                Compute(time, deltaT);
            }
            else if (!CalibrationStarted && IMU_AUTO_CALIBRATE)
            {
                StartCalibration(time);
            }
        }

        return IMU_OK;
    }

    void StartCalibration(void)
    {
        Calibrated = false;
        CalibrationStarted = false;
    }

    void StartCalibration(const uint64_t& time)
    {
        Reset();

        TimeStampCalibration = time;
        Calibrated = false;
        CalibrationStarted = true;
        AccumulatorSamples = 0;

        Gyroscope.Accum = 0;
        Accelerometer.Accum = 0;

        //printf("DBG:IMU_CALIBRATION_START\n");
    }

    int Stop(void)
    {
        //imu_hw_disable(&IMU_hw);

        Enabled = false;
        
        return IMU_OK;
    }

    int Reset(void)
    {
        CalibrationStarted = false;
        Filter.Reset();
        Gyroscope.Value = 0;
        Accelerometer.Value = 0;

        return IMU_OK;
    }

    void LoadConfig(const Config& config)
    {
        CalibrationTime     = config.CalibrationTime;
        Gyroscope.Bias      = config.GyroBias;
        Accelerometer.Bias  = config.AccelBias;
        Calibrated          = config.Calibrated;

        Filter.SetBeta(config.FilterBeta);
    }

	Config GetConfig(void)
    {
        return {
            .CalibrationTime = CalibrationTime,
            .FilterBeta          = Filter.GetBeta(),
            .GyroBias            = Gyroscope.Bias,
            .AccelBias           = Accelerometer.Bias,
            .Calibrated          = Calibrated
        };
    }
};

#endif
