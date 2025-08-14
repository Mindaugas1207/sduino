/**
 * 2022-11-01, Minduagas Mikalauskas.
 */

#include "main.hpp"
#include "config.hpp"

void ControlUpdate(const uint64_t& time);

int main()
{
    Init();

    while (true)
    {
        uint64_t time = TIME_U64();
        LED0.Update(time);
        IMU0.Update(time);
        
        ControlUpdate(time);
    }

    return 0;
}

uint64_t prtime = 0;

void ControlUpdate(const uint64_t& time = TIME_U64())
{
    if (time - prtime > 500 * 1000) {
        prtime = time;
    //     auto orient = IMU0.GetOrientation();
        auto rorient = IMU0.GetRawOrientation();
        auto orient = IMU0.GetOrientation();
        //auto accel = IMU0.GetRawAccel();
        //auto gyro = IMU0.GetRawGyro();
        printf("R:% .2f, P:% .2f, Y:% .2f\n", rorient.Roll() * 180 / M_PI, rorient.Pitch() * 180 / M_PI, rorient.Yaw() * 180 / M_PI);
        printf("R:% .2f, P:% .2f, Y:% .2f\n", orient.Roll() * 180 / M_PI, orient.Pitch() * 180 / M_PI, orient.Yaw() * 180 / M_PI);
        //printf("AX:% .2f, AY:% .2f, AZ:% .2f\n", accel.X, accel.Y, accel.Z);
        //printf("GX:% .2f, GY:% .2f, GZ:% .2f\n", gyro.X, gyro.Y, gyro.Z);
    //     printf("EA> CNT %ld, STP %ld, RPM %f, RPM_ %f\n",
    //     EncoderA.CountsLast, EncoderA.StepsLast, EncoderA.RPM, EncoderA.RPM_);
                                            
    //     printf("EB> CNT %ld, STP %ld, RPM %f, RPM_ %f\n",
    //     EncoderB.CountsLast, EncoderB.StepsLast, EncoderB.RPM, EncoderB.RPM_);

    //     printf("MA> PWR %f\n",
    //     MotorDriverA.GetPower());

    //     printf("MB> PWR %f\n",
    //     MotorDriverB.GetPower());
        
    //     char buffer[255];
    //     sprintf(buffer, "% .2f");

    //     int idx = sprintf(buffer, "LS> ");

    //     for (int i = 0; i < LineSensor0.SensorCount(); i++)
    //     {
    //         idx += sprintf(buffer + idx, "% .2f %c,", LineSensor0[i].Value, LineSensor0[i].Color == BLACK ? 'B' : 'W');
    //     }

    //     idx--;

    //     sprintf(buffer + idx, "\n");

    //     printf(buffer);

    //     //printf("DS> %d, P> %f, LP> %f, Y> %f, lY> %f, dY> %f\n", DistanceSensor0.GetDistance(), Position, LastPosition, imu_yaw, last_yaw, dyaw);
    }
}

//X state matrix
//P covariance matrix;
//Q system noise;
//F transition matrix;
//H observation matrix;
//y sensor signal (with noise);
//R sensor variance;

//[vx = vx + ax * dt,
// vy = vy + ay * dt,
// w ]

//[vx,
// vy,
// w  ]

//[ax,
// ay,
// w  ]

// void Prediction()
// {
//     double X, P, Q, F;
//     double FT; //transpose F
//     X = F*X;
//     P = F*P*FT + Q;
// }

// void Update()
// {
//     double X, P, y, R, H;
//     double HT; //transpose H
//     double inn = y - H*X;
//     double S = H*P*HT + R;
//     double K = P*HT / S;
//     X = X + K*inn;
//     P = P - K*H*P;
// }

int SduinoInit(const SduinoConfig& config)
{
    int status = PICO_OK;
    //USB
    stdio_init_all();

    //
    while (!stdio_usb_connected()) { sleep_ms(500); }

    //I2C
    i2c_init(i2c_internal, config.I2C_BaudRate);
    gpio_set_function(SDUINO_INTERNAL_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDUINO_INTERNAL_I2C_SCL_PIN, GPIO_FUNC_I2C);

    //SPI
    spi_init(spi_internal, config.SPI_BaudRate);
    gpio_set_function(SDUINO_INTERNAL_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SDUINO_INTERNAL_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SDUINO_INTERNAL_SPI_TX_PIN, GPIO_FUNC_SPI);

    gpio_init(SDUINO_INTERNAL_BMP_CS_PIN);
    gpio_init(SDUINO_INTERNAL_IMU_ACCEL_CS_PIN);
    gpio_init(SDUINO_INTERNAL_IMU_GYRO_CS_PIN);
    gpio_set_dir(SDUINO_INTERNAL_BMP_CS_PIN, GPIO_OUT);
    gpio_set_dir(SDUINO_INTERNAL_IMU_ACCEL_CS_PIN, GPIO_OUT);
    gpio_set_dir(SDUINO_INTERNAL_IMU_GYRO_CS_PIN, GPIO_OUT);
    gpio_put(SDUINO_INTERNAL_BMP_CS_PIN, true);
    gpio_put(SDUINO_INTERNAL_IMU_ACCEL_CS_PIN, true);
    gpio_put(SDUINO_INTERNAL_IMU_GYRO_CS_PIN, true);

    // gpio_init(25);
    // gpio_set_dir(25, GPIO_IN);
    // gpio_set_pulls(25, true, false);

    //UART
    uart_init(uart_internal, config.UART_BaudRate);
    uart_set_fifo_enabled(uart_internal, true);
    uart_set_translate_crlf(uart_internal, false);
    uart_set_format(uart_internal, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart_internal, false, false);
    gpio_set_function(SDUINO_INTERNAL_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SDUINO_INTERNAL_UART_RX_PIN, GPIO_FUNC_UART);

    //ADC
    //adc_init();
    //adc_set_temp_sensor_enabled(true);


    //PIO
    //pio_add_program(pio1, &encoder_program);

    return status;
}

void Init(void)
{
    int status = PICO_OK;
    bool canStart = true;
    status = SduinoInit(HWConfig.Sduino);
    if (status != PICO_OK) { canStart = false; printf("DBG:Sduino->Init error [%d].\n", status); }
    status = LED0.Init(HWConfig.Sduino.LED_Pin);
    if (status != LED_OK) { canStart = false; printf("DBG:LED0->Init error [%d].\n", status); }
    status = IMU0.Init(HWConfig.IMU0, DefaultConfig.IMU0);
    if (status != IMU_OK) { canStart = false; printf("DBG:IMU0->Init error [%d].\n", status); }
    // status = DistanceSensor0.Init(HWConfig.DistanceSensor0, {});
    // if (status != DISTANCE_SENSOR_OK) { canStart = false; printf("DBG:DistanceSensor0->Init error [%d].\n", status); }

    if (LED0.Start() != LED_OK) { printf("DBG:LED0->Start error.\n"); }
    if (IMU0.Start() != IMU_OK) { printf("DBG:IMU0->Start error.\n"); }

    if (!canStart)
    {
        LED0.Set(100, 400);
        for (;;)
        {
            tight_loop_contents();
        }
    }

    LED0.Set(100, 900);

    //DEBUG TESTS
    //uint64_t time = TIME_U64();
    // uint64_t prtime = time;
    // //LED
    // LED0.Start(time);
    // LED0.Set(200*1000,200*1000,10,time);
    // while(1) {
    //     LED0.Update();
    // }
    //MotorDrivers
    // MotorDriverA.Start(time);
    // MotorDriverB.Start(time);
    // MotorDriverA.SetPower(-0.2f);
    // MotorDriverB.SetPower(0.2f);
    // while(1) {
    //     MotorDriverA.Update();
    //     MotorDriverB.Update();
    // }
    // //ESC
    // sleep_ms(000);
    // ESC0.Start(time);
    // ESC0.SetPower(0.1f);
    // while(1) {
    //     ESC0.Update();
    // }
    // //Encoders
    // EncoderA.Start(time);
    // EncoderB.Start(time);
    // while(1) {
    //     time = TIME_U64();
    //     EncoderA.Update(time);
    //     EncoderB.Update(time);
    //     if (time - prtime > 100 * 1000)
    //     {
    //         printf("EncoderA: CNT %ld, STP %ld, RPM %f, RPM_ %f; EncoderB: CNT %ld, STP %ld, RPM %f, RPM_ %f\n",
    //                 EncoderA.CountsLast, EncoderA.StepsLast, EncoderA.RPM, EncoderA.RPM_,
    //                 EncoderB.CountsLast, EncoderB.StepsLast, EncoderB.RPM, EncoderB.RPM_);
    //         prtime = time;
    //     }
    // }
    // //IMU
    // IMU0.Start(time);
    // while(1) {
    //     time = TIME_U64();
    //     IMU0.Update(time);
    //     if (time - prtime > 100 * 1000) {
    //         //printf("G> X:% .9f, Y:% .9f, Z:% .9f\n", Gyroscope.Value.X, Gyroscope.Value.Y, Gyroscope.Value.Z);
    //         //printf("A> X:% .9f, Y:% .9f, Z:% .9f\n", Accelerometer.Value.X, Accelerometer.Value.Y, Accelerometer.Value.Z);
    //         //printf("Q> W:% .9F, X:% .9f, Y:% .9f, Z:% .9f\n", quat.W, quat.X, quat.Y, quat.Z);
    //         auto orient = IMU0.GetOrientation();
    //         printf("O> R:% .9f, P:% .9f, Y:% .9f\n", orient.Roll() * 180.0 / M_PI, orient.Pitch() * 180.0 / M_PI, orient.Yaw() * 180.0 / M_PI);
    //         prtime = time;
    //     }
    // }
    // //LineSensor
    // LineSensor0.Start(time);
    // while(1) {
    //     time = TIME_U64();
    //     LineSensor0.Update(time);
    //     if (time - prtime > 100 * 1000) {
    //         printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", LineSensor0.RawData[0], LineSensor0.RawData[1], LineSensor0.RawData[2], LineSensor0.RawData[3],
    //                    LineSensor0.RawData[4], LineSensor0.RawData[5], LineSensor0.RawData[6], LineSensor0.RawData[7],
    //                    LineSensor0.RawData[8], LineSensor0.RawData[9], LineSensor0.RawData[10], LineSensor0.RawData[11],
    //                    LineSensor0.RawData[12], LineSensor0.RawData[13], LineSensor0.RawData[14]);

    //         prtime = time;
    //     }
    // }
    // //DistanceSensor
    // DistanceSensor0.Start(time);
    // while(1) {
    //     time = TIME_U64();
    //     DistanceSensor0.Update(time);
    //     LED0.Set(gpio_get(25));
    //     LED0.Update(time);
    // }
}

