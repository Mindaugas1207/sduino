
/*
 * port.h
 *
 *  Created on: Oct 15, 2022
 *      Author: minda
 */

#ifndef INCLUDE_IMU_H_
#define INCLUDE_IMU_H_

#include "stdio.h"
#include "pico/stdlib.h"
#include "port.h"
#include "bmi08x.h"
#include "bmi08_spi.h"
#include "MMC5603.h"
#include "SensorFusion.h"
#include "math.h"
#include <limits>
#include <tuple>
#include <array>
//#include "Dense"
#ifdef IMU_USE_MAG
#define IMU_NDATA_CALIB 12
#define IMU_NDATA 9
#else
#define IMU_NDATA_CALIB 9
#define IMU_NDATA 6
#endif

#define G_EARTH 9.81492f

class IMU_s {

    typedef struct {float Roll; float Pitch; float Yaw;} euler_t;

    typedef struct vect_s {
        float X; float Y; float Z;
        vect_s& operator+= (const vect_s& rhs)
        {
            this->X += rhs.X;
            this->Y += rhs.Y;
            this->Z += rhs.Z;
            return *this;
        }
        vect_s& operator*= (const float& rhs)
        {
            this->X *= rhs;
            this->Y *= rhs;
            this->Z *= rhs;
            return *this;
        }
        vect_s& operator/= (const float& rhs)
        {
            this->X /= rhs;
            this->Y /= rhs;
            this->Z /= rhs;
            return *this;
        }
        friend vect_s operator+ (vect_s lhs, const vect_s& rhs)
        {
            lhs += rhs;
            return lhs;
        }
        friend vect_s operator* (vect_s lhs, const float& rhs)
        {
            lhs *= rhs;
            return lhs;
        }
        float length()
        {
            return sqrt(this->X * this->X + this->Y * this->Y + this->Z * this->Z);
        }
    } vect_t;

    typedef struct {float W; float X; float Y; float Z;} quat_t;

    typedef std::array<std::array<float, 3>, 3> matrix_t;

    struct Config_s{
        struct {
            vect_t Bias;
            vect_t Mean;
        } Accelerometer;

        struct {
            vect_t Bias;
            vect_t Mean;
        } Gyroscope;
#ifdef IMU_USE_MAG
        struct {
            vect_t Bias;
            vect_t Scale;
            vect_t Mean;
        } Magnetometer;
#endif
    };

    SF Fusion;

    int AccumulatorSamples;
    const uint32_t BiasCalibrationTime_ms = 5000;
    

    bool Calibrated;
    bool CalibrationStarted;
    bool FusionStarted;
    bool BiasCalibrated;

    absolute_time_t LastTime;
    absolute_time_t CalibrationTimeout;
    absolute_time_t PrintTimeout;
    struct {
        vect_t Raw;
        vect_t Value;
        vect_t Bias;
        vect_t Mean;
        vect_t Accum;
    } Accelerometer;

    struct {
        vect_t Raw;
        vect_t Value;
        vect_t Bias;
        vect_t Mean;
        vect_t Accum;
    } Gyroscope;

#ifdef IMU_USE_MAG
    const uint32_t MagCalibrationTime_ms = 20000;
    struct {
        vect_t Raw;
        vect_t Value;
        vect_t Bias;
        vect_t Scale;
        vect_t Mean;
        vect_t Accum;
        vect_t Max;
        vect_t Min;
        bool Calibrated;
    } Magnetometer;
#endif
    //Eigen::Matrix<float, 9, 1> e_x;
    //Eigen::Matrix<float, 9, 9> e_P;
    //Eigen::Matrix<float, 9, 1> E_X;
    absolute_time_t LastTimeF;
    vect_t Velocity = {0.0f,0.0f,0.0f};
    vect_t LastVelocity = {0.0f,0.0f,0.0f};
    vect_t LastDisplacement = {0.0f,0.0f,0.0f};
    vect_t Last_Acceleration = {0.0f,0.0f,0.0f};
    vect_t Last_Instantaneous_Acceleration = {0.0f,0.0f,0.0f};
    vect_t Average_Acceleration = {0.0f,0.0f,0.0f};
    euler_t Orientation;
    
    void ftest_init();
    //Eigen::Matrix<float, 9, 1> ftest(float accelerationX, float accelerationY, float accelerationZ, float dt, float sigma_a, float sigma_m);

    euler_t getEulerAngles(matrix_t& _Rotation);
    matrix_t getRotationMatrix(quat_t& _Quaternion);
    vect_t rotateVector(vect_t& _Vector, matrix_t& _Rotation);
    matrix_t getMatrixTranspose(matrix_t& _Matrix);
    float getMatrixDeterminant(matrix_t& _Matrix);

public:
    IMU_s() : Fusion() {};

    typedef Config_s Config_t;

    bool init(void *hw0_inst, void *hw1_inst);
    void reset();
    void startCalibration();
    void biasCalibrationInit();
#ifdef IMU_USE_MAG
    void magCalibrationInit();
    void getMagMaxMin();
    void calculateMagCal();
#endif
    void accumulate();
    void calculateMean();
    void calculateBias();
    void calibrationRun(absolute_time_t _TimeNow);
    void loadConfiguration(Config_t & calib);
    void loadDefaultConfiguration();
    Config_t getConfiguration();

    bool isCalibrated();

    bool readSensors();
    void compute(bool _NewData, absolute_time_t _TimeNow);

    //std::tuple<float, float, float> getAcceleration();
    std::tuple<float, float, float> getOrientation();
    std::tuple<float, float, float, float> getQuaternion();

    void startAsyncProcess();
    bool runAsyncProcess(absolute_time_t _TimeNow);
};

#endif /* INCLUDE_IMU_H_ */
