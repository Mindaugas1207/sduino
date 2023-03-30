
#include "imu.hpp"

constexpr float ACCEL_LSB_TO_MPS2(float raw) { return (raw * 24.0f * G_EARTH) / (float)(1 << (16 - 1)); }
constexpr float GYRO_LSB_TO_RPS(float raw) { return (raw * (float)(2000.0f * M_PI)) / (180.0f * (float)(1 << (16 - 1))); }
constexpr float MAG_LSB_TO_UT(float raw) { return (raw * 1000.0f) / (float)(1 << (14)); }

volatile bool data_sync_int = true;

BMI08x_inst_t BMI088dev;
MMC5603_inst_t MMC5603dev;

static void imu_hw_callback(uint gpio, uint32_t events);
inline bool imu_hw_new_data_available();
bool imu_hw_init(void *hw0_inst, void *hw1_inst);
inline void imu_hw_start();
inline void imu_hw_stop();
inline void imu_hw_read(float _output[IMU_NDATA]);

static void imu_hw_callback(uint gpio, uint32_t events)
{
    data_sync_int = true;
}

inline bool imu_hw_new_data_available()
{
    return data_sync_int || gpio_get(SDUINO_INTERNAL_IMU_INT_PIN);
}

bool imu_hw_init(void *hw0_inst, void *hw1_inst)
{
#ifdef IMU_USE_MAG
    port_i2c_t * i2c_inst = (port_i2c_t*)hw0_inst;
#endif
    port_spi_t * spi_inst = (port_spi_t*)hw1_inst;

    uint gyro_dev = port_spi_add_device(spi_inst, SDUINO_INTERNAL_IMU_GYRO_CS_PIN);
    uint accel_dev = port_spi_add_device(spi_inst, SDUINO_INTERNAL_IMU_ACCEL_CS_PIN);

    if (BMI088_init(&BMI088dev, spi_inst, spi_inst, gyro_dev, accel_dev) != BMI08X_OK) {
        printf("BMI INIT FAIL\n");
        return false;
    }
#ifdef IMU_USE_MAG
    if (MMC5603_init(&MMC5603dev, i2c_inst, PORT_CHANNEL_ANY, MMC5603_DEFAULT_ADDRESS) != MMC5603_OK) {
        printf("MMC INIT FAIL\n");
        return false;
    }
#endif
    gpio_init(SDUINO_INTERNAL_IMU_INT_PIN);
    gpio_set_dir(SDUINO_INTERNAL_IMU_INT_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(SDUINO_INTERNAL_IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, &imu_hw_callback);
    
    return true;
}

inline void imu_hw_start()
{
    gpio_set_irq_enabled(SDUINO_INTERNAL_IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true);
    data_sync_int = true;
}

inline void imu_hw_stop()
{
    gpio_set_irq_enabled(SDUINO_INTERNAL_IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, false);
}

inline void imu_hw_read(float _output[IMU_NDATA])
{
    data_sync_int = false;
    BMI088_ReadData(&BMI088dev);
#ifdef IMU_USE_MAG
    //MMC5603_ReadDataBlocking(&MMC5603dev);;
    //if (MMC5603_NewDataReady(&MMC5603dev) == MMC5603_DATA_READY)
    MMC5603_ReadDataBlocking(&MMC5603dev);
#endif
    _output[0] = ACCEL_LSB_TO_MPS2((float)BMI088dev.accel.x);
    _output[1] = ACCEL_LSB_TO_MPS2((float)BMI088dev.accel.y);
    _output[2] = ACCEL_LSB_TO_MPS2((float)BMI088dev.accel.z);
    _output[3] = GYRO_LSB_TO_RPS((float)BMI088dev.gyro.x);
    _output[4] = GYRO_LSB_TO_RPS((float)BMI088dev.gyro.y);
    _output[5] = GYRO_LSB_TO_RPS((float)BMI088dev.gyro.z);
#ifdef IMU_USE_MAG
    _output[6] = MAG_LSB_TO_UT((float)MMC5603dev.mag.x);
    _output[7] = MAG_LSB_TO_UT((float)MMC5603dev.mag.y);
    _output[8] = MAG_LSB_TO_UT((float)MMC5603dev.mag.z);

    //printf("X:% .6f, Y:% .6f, Z:% .6f\n", _output[6], _output[7], _output[8]);
#endif
}


void IMU_s::biasCalibrationInit()
{
    ftest_init();

    Accelerometer.Accum.X = 0.0f;
    Accelerometer.Accum.Y = 0.0f;
    Accelerometer.Accum.Z = 0.0f;
    Gyroscope.Accum.X = 0.0f;
    Gyroscope.Accum.Y = 0.0f;
    Gyroscope.Accum.Z = 0.0f;
#ifdef IMU_USE_MAG
    Magnetometer.Accum.X = 0.0f;
    Magnetometer.Accum.Y = 0.0f;
    Magnetometer.Accum.Z = 0.0f;
#endif

    AccumulatorSamples = 0;
    CalibrationTimeout = make_timeout_time_ms(BiasCalibrationTime_ms);
}

#ifdef IMU_USE_MAG
void IMU_s::magCalibrationInit()
{
    Magnetometer.Min.X = std::numeric_limits<float>::max();
    Magnetometer.Max.X = std::numeric_limits<float>::lowest();
    Magnetometer.Min.Y = std::numeric_limits<float>::max();
    Magnetometer.Max.Y = std::numeric_limits<float>::lowest();
    Magnetometer.Min.Z = std::numeric_limits<float>::max();
    Magnetometer.Max.Z = std::numeric_limits<float>::lowest();
    
    CalibrationTimeout = make_timeout_time_ms(MagCalibrationTime_ms);
}
#endif

bool IMU_s::readSensors()
{
    if (imu_hw_new_data_available())
    {
        float buffer[IMU_NDATA];
        imu_hw_read(buffer);
        Accelerometer.Raw.X = buffer[0];
        Accelerometer.Raw.Y = buffer[1];
        Accelerometer.Raw.Z = buffer[2];
        Gyroscope.Raw.X = buffer[3];
        Gyroscope.Raw.Y = buffer[4];
        Gyroscope.Raw.Z = buffer[5];
#ifdef IMU_USE_MAG
        Magnetometer.Raw.X = buffer[6];
        Magnetometer.Raw.Y = buffer[7];
        Magnetometer.Raw.Z = buffer[8];
#endif

        return true;
    }

    return false;
}

std::tuple<float, float, float> IMU_s::getOrientation()
{
    return {Orientation.Roll, Orientation.Pitch, Orientation.Yaw};
}

// std::tuple<float, float, float> IMU_s::getAcceleration()
// {
//     return {Acceleration.X, Acceleration.Y, Acceleration.Z};
// }

std::tuple<float, float, float, float> IMU_s::getQuaternion()
{
    return Fusion.getQuaternion();
}

bool IMU_s::init(void *hw0_inst, void *hw1_inst)
{
    Calibrated = false;
    CalibrationStarted = false;
    FusionStarted = false;
    BiasCalibrated = false;
    AccumulatorSamples = 0;
#ifdef IMU_USE_MAG
    Magnetometer.Calibrated = false;
#endif
    LastTime = get_absolute_time();
    CalibrationTimeout = get_absolute_time();
    return imu_hw_init(hw0_inst, hw1_inst);
}

void IMU_s::startCalibration()
{
    Calibrated = false;
    CalibrationStarted = false;
    FusionStarted = false;
    BiasCalibrated = false;
    AccumulatorSamples = 0;
#ifdef IMU_USE_MAG
    Magnetometer.Calibrated = false;
#endif
    LastTime = get_absolute_time();
    CalibrationTimeout = get_absolute_time();
}

void IMU_s::startAsyncProcess()
{
    imu_hw_start();
}

bool IMU_s::runAsyncProcess(absolute_time_t _TimeNow)
{
    bool new_data = readSensors();
    if (!Calibrated)
    {
        if (new_data) {
            calibrationRun(_TimeNow);
        }
    }
    else
    {
        compute(new_data, _TimeNow);
        
        return true;
    }

    return false;
}

void IMU_s::calibrationRun(absolute_time_t _TimeNow)
{   
    if (!CalibrationStarted)
    {
        biasCalibrationInit();
        FusionStarted = false;
        CalibrationStarted = true;
        printf("DBG:IMU_CALIBRATION_START\n");
    }
    else if (!BiasCalibrated)
    {
        if (to_us_since_boot(_TimeNow) > to_us_since_boot(CalibrationTimeout))
        {
            calculateMean();
            calculateBias();
            BiasCalibrated = true;
#ifdef IMU_USE_MAG
            magCalibrationInit();
#endif
            printf("DBG:IMU_BIAS_OK\n");
            gpio_put(SDUINO_INTERNAL_LED_PIN, 1);
        }
        else
            accumulate();
    }
#ifdef IMU_USE_MAG
    else if (!Magnetometer.Calibrated)
    {
        if (to_us_since_boot(_TimeNow) > to_us_since_boot(CalibrationTimeout))
        {
            calculateMagCal();
            Magnetometer.Calibrated = true;
            printf("DBG:IMU_MAG_OK\n");
            gpio_put(SDUINO_INTERNAL_LED_PIN, 0);
        }
        else
            getMagMaxMin();
    }
#endif
    else if (!FusionStarted)
    {
        Fusion.init();
#ifdef IMU_USE_MAG
        //FusionStarted = Fusion.initQuat(-Accelerometer.Mean.X, Accelerometer.Mean.Y, Accelerometer.Mean.Z,
        //                                -Magnetometer.Mean.X, -Magnetometer.Mean.Y, -Magnetometer.Mean.Z);
        FusionStarted = true;
#else
        FusionStarted = true;
#endif
        LastTime = get_absolute_time();
    }
    else {
        printf("DBG:IMU_ACCEL_MEAN(%f,%f,%f)\n", Accelerometer.Mean.X, Accelerometer.Mean.Y, Accelerometer.Mean.Z);
        printf("DBG:IMU_ACCEL_BIAS(%f,%f,%f)\n", Accelerometer.Bias.X, Accelerometer.Bias.Y, Accelerometer.Bias.Z);
        printf("DBG:IMU_GYRO_MEAN(%f,%f,%f)\n", Gyroscope.Mean.X, Gyroscope.Mean.Y, Gyroscope.Mean.Z);
        printf("DBG:IMU_GYRO_BIAS(%f,%f,%f)\n", Gyroscope.Bias.X, Gyroscope.Bias.Y, Gyroscope.Bias.Z);
#ifdef IMU_USE_MAG
        printf("Mag Max %f, %f, %f\n", Magnetometer.Max.X, Magnetometer.Max.Y, Magnetometer.Max.Z);
        printf("Mag Min %f, %f, %f\n", Magnetometer.Min.X, Magnetometer.Min.Y, Magnetometer.Min.Z);
        printf("Mag mean %f, %f, %f\n", Magnetometer.Mean.X, Magnetometer.Mean.Y, Magnetometer.Mean.Z);
        printf("Mag bias %f, %f, %f\n", Magnetometer.Bias.X, Magnetometer.Bias.Y, Magnetometer.Bias.Z);
        printf("Mag Scale %f, %f, %f\n", Magnetometer.Scale.X, Magnetometer.Scale.Y, Magnetometer.Scale.Z);
#endif
        Calibrated = true;
        printf("DBG:IMU_CALIBRATION_OK\n");
    }
}

bool IMU_s::isCalibrated()
{
    return Calibrated;
}

void IMU_s::loadConfiguration(Config_t & calib)
{
    CalibrationStarted = true;
    Accelerometer.Bias = calib.Accelerometer.Bias;
    Accelerometer.Mean = calib.Accelerometer.Mean;

    Gyroscope.Bias = calib.Gyroscope.Bias;
    Gyroscope.Mean = calib.Gyroscope.Mean;
    BiasCalibrated = true;

#ifdef IMU_USE_MAG
    Magnetometer.Bias = calib.Magnetometer.Bias;
    Magnetometer.Mean = calib.Magnetometer.Mean;
    Magnetometer.Scale = calib.Magnetometer.Scale;
    Magnetometer.Calibrated = true;
#endif
    FusionStarted = false;
    Calibrated = false;
}

void IMU_s::loadDefaultConfiguration()
{
    
}

IMU_s::Config_t IMU_s::getConfiguration()
{
    Config_t data;

    data.Accelerometer.Bias = Accelerometer.Bias;
    data.Accelerometer.Mean = Accelerometer.Mean;

    data.Gyroscope.Bias = Gyroscope.Bias;
    data.Gyroscope.Mean = Gyroscope.Mean;

#ifdef IMU_USE_MAG
    data.Magnetometer.Bias = Magnetometer.Bias;
    data.Magnetometer.Mean = Magnetometer.Mean;
    data.Magnetometer.Scale = Magnetometer.Scale;
#endif

    return data;
}

void IMU_s::compute(bool _NewData, absolute_time_t _TimeNow)
{
    //printf("Raw %f, %f, %f, %f, %f, %f, %f, %f, %f\n", Accelerometer.Raw.X, Accelerometer.Raw.Y, Accelerometer.Raw.Z, Gyroscope.Raw.X, Gyroscope.Raw.Y, Gyroscope.Raw.Z, Magnetometer.Raw.X, Magnetometer.Raw.Y, Magnetometer.Raw.Z);
    Accelerometer.Value.X = Accelerometer.Raw.X - Accelerometer.Bias.X;
    Accelerometer.Value.Y = Accelerometer.Raw.Y - Accelerometer.Bias.Y;
    Accelerometer.Value.Z = Accelerometer.Raw.Z - Accelerometer.Bias.Z;

    Gyroscope.Value.X = Gyroscope.Raw.X - Gyroscope.Bias.X;
    Gyroscope.Value.Y = Gyroscope.Raw.Y - Gyroscope.Bias.Y;
    Gyroscope.Value.Z = Gyroscope.Raw.Z - Gyroscope.Bias.Z;

#ifdef IMU_USE_MAG
    Magnetometer.Value.X = (Magnetometer.Raw.X - Magnetometer.Bias.X) * Magnetometer.Scale.X;
    Magnetometer.Value.Y = (Magnetometer.Raw.Y - Magnetometer.Bias.Y) * Magnetometer.Scale.Y;
    Magnetometer.Value.Z = (Magnetometer.Raw.Z - Magnetometer.Bias.Z) * Magnetometer.Scale.Z;

    //Magnetometer.Value.X = (Magnetometer.Raw.X - Magnetometer.Bias.X);
    //Magnetometer.Value.Y = (Magnetometer.Raw.Y - Magnetometer.Bias.Y);
    //Magnetometer.Value.Z = (Magnetometer.Raw.Z - Magnetometer.Bias.Z);

    // Magnetometer.Value.X = (Magnetometer.Raw.X);
    // Magnetometer.Value.Y = (Magnetometer.Raw.Y);
    // Magnetometer.Value.Z = (Magnetometer.Raw.Z);
#endif
    float dt = ((absolute_time_diff_us(LastTime, _TimeNow)) / 1000000.0f); // set integration time by time elapsed since last filter update
    LastTime = _TimeNow;
#ifdef IMU_USE_MAG
    // Fusion.MadgwickUpdate(
    //      Gyroscope.Value.X,
    //     -Gyroscope.Value.Y,
    //     -Gyroscope.Value.Z,
    //     -Accelerometer.Value.X,
    //      Accelerometer.Value.Y,
    //      Accelerometer.Value.Z,
    //     -Magnetometer.Value.X,
    //      Magnetometer.Value.Y,
    //     -Magnetometer.Value.Z,
    //      dt);
    Fusion.MadgwickUpdate_kriswiner(
        -Accelerometer.Value.X,
         Accelerometer.Value.Y,
         Accelerometer.Value.Z,
         Gyroscope.Value.X,
        -Gyroscope.Value.Y,
        -Gyroscope.Value.Z,
        -Magnetometer.Value.X,
         Magnetometer.Value.Y,
        -Magnetometer.Value.Z,
         dt);
#else
    Fusion.MadgwickUpdate(
         Gyroscope.Value.X,
         Gyroscope.Value.Y,
         Gyroscope.Value.Z,
         Accelerometer.Value.X,
         Accelerometer.Value.Y,
         Accelerometer.Value.Z,
         dt);
#endif
    auto [w, x, y, z] = Fusion.getQuaternion();
    quat_t Quaternion = {w, x, y, z};
    matrix_t Rotation;
    // //matrix_t Transposition;
    // vect_t Instantaneous_Acceleration;
    
    // //vect_t Displacement;
    // //float alpha_a = 0.1f;
    // //float A;
    // //vect_t AngularVelocity;
    

    //Get rotation (frame to world)
    Rotation = getRotationMatrix(Quaternion);
    // //A = getMatrixDeterminant(Rotation);
    // //Get transposition (reverse rotation)
    // //Transposition = getMatrixTranspose(Rotation);

    // // a = dv / dt => dv = a * dt, v += dv
    // // v = ds / dt => ds = v * dt, s += ds
    // //Acceleration (m/s/s)
    // if (_NewData)
    // {
    //     Instantaneous_Acceleration = rotateVector(Accelerometer.Value, Rotation); //Rotate from local frame to world
    //     Instantaneous_Acceleration.Z -= G_EARTH;
    //     Average_Acceleration = Instantaneous_Acceleration + Last_Instantaneous_Acceleration;
    //     Average_Acceleration /= 2.0f;
    //     Last_Instantaneous_Acceleration = Instantaneous_Acceleration;

    //     float dtF = ((absolute_time_diff_us(LastTimeF, _TimeNow)) / 1000000.0f); // set integration time by time elapsed since last filter update
    //     LastTimeF = _TimeNow;

    //     E_X = ftest(Average_Acceleration.X, Average_Acceleration.Y, Average_Acceleration.Z, dtF, 0.1f, 0.1f);
    // }

    

    

    // //Velocity (m/s)
    // //Velocity += Average_Acceleration * dt;
    // //Displacement (m)
    // //Displacement = Velocity * dt + LastDisplacement * 0.99f;
    // //LastDisplacement = Displacement;
    // //Orientation in Euler angles (rad)
    Orientation = getEulerAngles(Rotation);
    // //Angular Velocity (rad/s)
    // //AngularVelocity = rotateVector(Gyroscope.Value, Rotation); //Rotate from local frame to world
    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTimeout))
    // {
    //     //printf("O> R:% .9f, P:% .9f, Y:% .9f\n", Orientation.Roll * 180.0f / M_PI, Orientation.Pitch * 180.0f / M_PI, Orientation.Yaw * 180.0f / M_PI);
    //     // printf("m> % .9f, % .9f, % .9f\n"
    //     //        "   % .9f, % .9f, % .9f\n"
    //     //        "   % .9f, % .9f, % .9f\n"
    //     //        "A> % .9f\n",
    //     //        Rotation[0][0], Rotation[0][1], Rotation[0][2],
    //     //        Rotation[1][0], Rotation[1][1], Rotation[1][2],
    //     //        Rotation[2][0], Rotation[2][1], Rotation[2][2],
    //     //        A);
    //     //printf("s> X:% .9f, Y:% .9f, Z:% .9f\n", Accelerometer.Value.X, Accelerometer.Value.Y, Accelerometer.Value.Z);
    //     //printf("SF: dt % .9f\n", dt);
    //     //printf("ev % .9f\n", E_velocity);
    //     //printf("A> X:% .9f, Y:% .9f, Z:% .9f\n", Average_Acceleration.X, Average_Acceleration.Y, Average_Acceleration.Z);
    //     //printf("V> X:% .9f, Y:% .9f, Z:% .9f\n", Velocity.X, Velocity.Y, Velocity.Z);
    //     printf("P> X:% .9f, Y:% .9f, Z:% .9f\n"
    //            "V> X:% .9f, Y:% .9f, Z:% .9f\n"
    //            "A> X:% .9f, Y:% .9f, Z:% .9f\n", E_X(0), E_X(3), E_X(6),
    //                                              E_X(1), E_X(4), E_X(7),
    //                                              E_X(2), E_X(5), E_X(8));
    //     //printf("S> X:% .9f, Y:% .9f, Z:% .9f\n", Displacement.X, Displacement.Y, Displacement.Z);
    //     //printf("f> X:% .9f, Y:% .9f, Z:% .9f\n", AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z);
    //     PrintTimeout = make_timeout_time_ms(50);
    // }
}



void IMU_s::ftest_init()
{
    e_x << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
    e_P << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;
}

Eigen::Matrix<float, 9, 1> IMU_s::ftest(float accelerationX, float accelerationY, float accelerationZ, float dt, float sigma_a, float sigma_m)
{
    using namespace Eigen;
    // State transition matrix
    Matrix<float, 9, 9> F;
    F << 1.0f,   dt, 0.5f*dt*dt, 0.0f, 0.0f,       0.0f, 0.0f, 0.0f,       0.0f,
         0.0f, 1.0f,         dt, 0.0f, 0.0f,       0.0f, 0.0f, 0.0f,       0.0f,
         0.0f, 0.0f,       1.0f, 0.0f, 0.0f,       0.0f, 0.0f, 0.0f,       0.0f,
         0.0f, 0.0f,       0.0f, 1.0f,   dt, 0.5f*dt*dt, 0.0f, 0.0f,       0.0f,
         0.0f, 0.0f,       0.0f, 0.0f, 1.0f,         dt, 0.0f, 0.0f,       0.0f,
         0.0f, 0.0f,       0.0f, 0.0f, 0.0f,       1.0f, 0.0f, 0.0f,       0.0f,
         0.0f, 0.0f,       0.0f, 0.0f, 0.0f,       0.0f, 1.0f,   dt, 0.5f*dt*dt,
         0.0f, 0.0f,       0.0f, 0.0f, 0.0f,       0.0f, 0.0f, 1.0f,         dt,
         0.0f, 0.0f,       0.0f, 0.0f, 0.0f,       0.0f, 0.0f, 0.0f,       1.0f;

    // Measurement matrix
    Matrix<float, 3, 9> H;
    H << 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    // Process noise covariance
    float dt4 = dt*dt*dt*dt;
    float dt3 = dt*dt*dt;
    float dt2 = dt*dt;
    Matrix<float, 9, 9> Q;
    Q << dt4/4, dt3/2, dt2/2,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,
         dt3/2,   dt2,    dt,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,
         dt3/2,    dt,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,
         0.0f,   0.0f,  0.0f, dt4/4, dt3/2, dt2/2,  0.0f,  0.0f,  0.0f,
         0.0f,   0.0f,  0.0f, dt3/2,   dt2,    dt,  0.0f,  0.0f,  0.0f,
         0.0f,   0.0f,  0.0f, dt3/2,    dt,  1.0f,  0.0f,  0.0f,  0.0f,
         0.0f,   0.0f,  0.0f,  0.0f,  0.0f,  0.0f, dt4/4, dt3/2, dt2/2,
         0.0f,   0.0f,  0.0f,  0.0f,  0.0f,  0.0f, dt3/2,   dt2,    dt,
         0.0f,   0.0f,  0.0f,  0.0f,  0.0f,  0.0f, dt3/2,    dt,  1.0f;

    Q *= sigma_a * sigma_a;

    // Measurement noise covariance
    Matrix<float, 3, 3> R;
    R << sigma_m*sigma_m, 0.0f, 0.0f,
         0.0f, sigma_m*sigma_m, 0.0f,
         0.0f, 0.0f, sigma_m*sigma_m;

    // Initial state
    // see ftest_init()

    // Initial covariance
    // see ftest_init()

    // Predict step
    e_x = F * e_x;
    e_P = F * e_P * F.transpose().eval() + Q;

    // Measurement update step
    Matrix<float, 3, 1> z;
    z << accelerationX, accelerationY, accelerationZ;
    Matrix<float, 3, 1> y = z - H * e_x;
    Matrix<float, 3, 3> S = H * e_P * H.transpose().eval() + R;
    Matrix<float, 9, 3> K = e_P * H.transpose().eval() * S.inverse();
    e_x = e_x + K * y;
    Matrix<float, 9, 9> tm = (Matrix<float, 9, 9>::Identity() - K * H);
    e_P = tm * e_P * tm.transpose().eval() + K * R * K.transpose().eval();

    // Estimate of velocity is the first element of the state vector
    //printf("V> % .9f, % .9f, % .9f, % .9f, % .9f, % .9f\n", e_x(0), e_x(1), e_x(2), e_x(3), e_x(4), e_x(5));
    return e_x;
}

IMU_s::euler_t IMU_s::getEulerAngles(matrix_t& _Rotation)
{
    euler_t _Result;

    _Result.Roll  = atan2f(_Rotation[2][1], _Rotation[2][2]);
    _Result.Yaw   = atan2f(_Rotation[1][0], _Rotation[0][0]);
	_Result.Pitch = asinf(-2.0f * (_Rotation[2][0]));

    return _Result;
}

IMU_s::matrix_t IMU_s::getRotationMatrix(quat_t& _Quaternion)
{
    matrix_t _Matrix;
    float xx, yy, zz, xy, xz, yz, xw, yw, zw;

    xx = _Quaternion.X * _Quaternion.X;
    xy = _Quaternion.X * _Quaternion.Y;
    xz = _Quaternion.X * _Quaternion.Z;
    xw = _Quaternion.X * _Quaternion.W;
    yy = _Quaternion.Y * _Quaternion.Y;
    yz = _Quaternion.Y * _Quaternion.Z;
    yw = _Quaternion.Y * _Quaternion.W;
    zz = _Quaternion.Z * _Quaternion.Z;
    zw = _Quaternion.Z * _Quaternion.W;
    
    _Matrix[0][0] = (0.5f -  yy - zz);
    _Matrix[0][1] =         (xy - zw);
    _Matrix[0][2] =         (xz + yw);

    _Matrix[1][0] =         (xy + zw);
    _Matrix[1][1] = (0.5f -  xx - zz);
    _Matrix[1][2] =         (yz - xw);

    _Matrix[2][0] =         (xz - yw);
    _Matrix[2][1] =         (yz + xw);
    _Matrix[2][2] = (0.5f -  xx - yy);

    return _Matrix;
}

IMU_s::vect_t IMU_s::rotateVector(vect_t& _Vector, matrix_t& _Rotation)
{
    vect_t _Result;

    _Result.X = _Vector.X * _Rotation[0][0] + _Vector.Y * _Rotation[0][1] + _Vector.Z * _Rotation[0][2];
    _Result.Y = _Vector.X * _Rotation[1][0] + _Vector.Y * _Rotation[1][1] + _Vector.Z * _Rotation[1][2];
    _Result.Z = _Vector.X * _Rotation[2][0] + _Vector.Y * _Rotation[2][1] + _Vector.Z * _Rotation[2][2];

    _Result *= 2.0f;

    return _Result;
}

IMU_s::matrix_t IMU_s::getMatrixTranspose(matrix_t& _Matrix)
{
    matrix_t _Result;

    _Result[0][0] = _Matrix[0][0];
    _Result[0][1] = _Matrix[1][0];
    _Result[0][2] = _Matrix[2][0];

    _Result[1][0] = _Matrix[0][1];
    _Result[1][1] = _Matrix[1][1];
    _Result[1][2] = _Matrix[2][1];

    _Result[2][0] = _Matrix[0][2];
    _Result[2][1] = _Matrix[1][2];
    _Result[2][2] = _Matrix[2][2];

    return _Result;
}

float IMU_s::getMatrixDeterminant(matrix_t& _Matrix)
{
    float _Result = 0.0f;
    _Result = _Matrix[0][0] * (_Matrix[1][1] * _Matrix[2][2] - _Matrix[1][2] * _Matrix[2][1]);
    _Result -= _Matrix[0][1] * (_Matrix[1][0] * _Matrix[2][2] - _Matrix[1][2] * _Matrix[2][0]);
    _Result += _Matrix[0][2] * (_Matrix[1][0] * _Matrix[2][1] - _Matrix[1][1] * _Matrix[2][0]);
    return _Result;
}

#ifdef IMU_USE_MAG
void IMU_s::getMagMaxMin()
{
    // printf("Mag Raw %f, %f, %f\n", Magnetometer.Raw.X, Magnetometer.Raw.Y, Magnetometer.Raw.Z);
    // printf("Mag Max %f, %f, %f\n", Magnetometer.Max.X, Magnetometer.Max.Y, Magnetometer.Max.Z);
    // printf("Mag Min %f, %f, %f\n", Magnetometer.Min.X, Magnetometer.Min.Y, Magnetometer.Min.Z);
    
    Magnetometer.Min.X = std::min(Magnetometer.Raw.X, Magnetometer.Min.X);
    Magnetometer.Max.X = std::max(Magnetometer.Raw.X, Magnetometer.Max.X);
    Magnetometer.Min.Y = std::min(Magnetometer.Raw.Y, Magnetometer.Min.Y);
    Magnetometer.Max.Y = std::max(Magnetometer.Raw.Y, Magnetometer.Max.Y);
    Magnetometer.Min.Z = std::min(Magnetometer.Raw.Z, Magnetometer.Min.Z);
    Magnetometer.Max.Z = std::max(Magnetometer.Raw.Z, Magnetometer.Max.Z);
    
}
#endif

#ifdef IMU_USE_MAG
void IMU_s::calculateMagCal()
{
    // Get hard iron correction
    Magnetometer.Bias.X  = (Magnetometer.Max.X + Magnetometer.Min.X) / 2.0f;
    Magnetometer.Bias.Y  = (Magnetometer.Max.Y + Magnetometer.Min.Y) / 2.0f;
    Magnetometer.Bias.Z  = (Magnetometer.Max.Z + Magnetometer.Min.Z) / 2.0f;
    
    // Get soft iron correction estimate
    float _ScaleX = (Magnetometer.Max.X - Magnetometer.Min.X) / 2.0f;
    float _ScaleY = (Magnetometer.Max.Y - Magnetometer.Min.Y) / 2.0f;
    float _ScaleZ = (Magnetometer.Max.Z - Magnetometer.Min.Z) / 2.0f;

    float avg = _ScaleX + _ScaleY + _ScaleZ;
    avg /= 3.0;

    Magnetometer.Scale.X = avg / _ScaleX;
    Magnetometer.Scale.Y = avg / _ScaleY;
    Magnetometer.Scale.Z = avg / _ScaleZ;
}
#endif

void IMU_s::accumulate()
{
    Accelerometer.Accum.X += Accelerometer.Raw.X;
    Accelerometer.Accum.Y += Accelerometer.Raw.Y;
    Accelerometer.Accum.Z += Accelerometer.Raw.Z;
    Gyroscope.Accum.X += Gyroscope.Raw.X;
    Gyroscope.Accum.Y += Gyroscope.Raw.Y;
    Gyroscope.Accum.Z += Gyroscope.Raw.Z;
#ifdef IMU_USE_MAG
    Magnetometer.Accum.X += Magnetometer.Raw.X;
    Magnetometer.Accum.Y += Magnetometer.Raw.Y;
    Magnetometer.Accum.Z += Magnetometer.Raw.Z;
#endif
    AccumulatorSamples++;
}

void IMU_s::calculateMean()
{
    Accelerometer.Mean.X = Accelerometer.Accum.X / AccumulatorSamples;
    Accelerometer.Mean.Y = Accelerometer.Accum.Y / AccumulatorSamples;
    Accelerometer.Mean.Z = Accelerometer.Accum.Z / AccumulatorSamples;
    Gyroscope.Mean.X = Gyroscope.Accum.X / AccumulatorSamples;
    Gyroscope.Mean.Y = Gyroscope.Accum.Y / AccumulatorSamples;
    Gyroscope.Mean.Z = Gyroscope.Accum.Z / AccumulatorSamples;
#ifdef IMU_USE_MAG
    Magnetometer.Mean.X = Magnetometer.Accum.X / AccumulatorSamples;
    Magnetometer.Mean.Y = Magnetometer.Accum.Y / AccumulatorSamples;
    Magnetometer.Mean.Z = Magnetometer.Accum.Z / AccumulatorSamples;
#endif
}

void IMU_s::calculateBias()
{
    Accelerometer.Bias.X = Accelerometer.Mean.X;
    Accelerometer.Bias.Y = Accelerometer.Mean.Y;
    Accelerometer.Bias.Z = Accelerometer.Mean.Z;
    Gyroscope.Bias.X = Gyroscope.Mean.X;
    Gyroscope.Bias.Y = Gyroscope.Mean.Y;
    Gyroscope.Bias.Z = Gyroscope.Mean.Z;
    //Accelerometer.Bias.Z -= G_EARTH;
    if(Accelerometer.Bias.Z > G_EARTH / 3.0f) Accelerometer.Bias.Z -= G_EARTH;  // Remove gravity from the z-axis accelerometer bias calculation
    else Accelerometer.Bias.Z += G_EARTH;
}


