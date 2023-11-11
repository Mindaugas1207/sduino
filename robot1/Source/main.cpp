/**
 * 2022-11-01, Minduagas Mikalauskas.
 */

#include "main.hpp"
#include "config.hpp"

#define DRV_R 473.5f
constexpr auto MOTOR_MIN_RPM = 260;
//constexpr auto MOTOR_NOMINAL_RPM = 2800;
//constexpr auto MOTOR_MAX_RPM = 2800;
// constexpr auto MOTOR_MAX_RPM = 2800; //2800, 4300
// constexpr auto ENCODER_REDUCTION = 5.4;//1:5.4, 4.4
// constexpr auto ENCODER_PPR_HAL = 11;//11, 1? 13?
// constexpr auto ENCODER_WHEEL_DIAMETER_MM = 25;
// constexpr auto ENCODER_WHEEL_DIAMETER = 0.025;
constexpr auto MOTOR_MAX_RPM = 3850; //2800, 4300
constexpr auto ENCODER_REDUCTION = 4;//1:5.4, 4.4
constexpr auto ENCODER_PPR_HAL = 7;//11, 1? 13?
constexpr auto ENCODER_WHEEL_DIAMETER_MM = 22;
constexpr auto ENCODER_WHEEL_DIAMETER = 0.022;
#ifdef MOTORDEF1
constexpr auto MOTOR_MAX_RPM = 4300; //2800, 4300
constexpr auto ENCODER_REDUCTION = 4.4;//1:5.4, 4.4
constexpr auto ENCODER_PPR_HAL = 11;//11, 1? 13?
constexpr auto ENCODER_WHEEL_DIAMETER_MM = 32;
#endif
constexpr auto ENCODER_EPP_HAL = 4;
constexpr auto ENCODER_PPR = ENCODER_PPR_HAL * ENCODER_EPP_HAL * ENCODER_REDUCTION;
constexpr auto ENCODER_MAX_PULSE_PERIOD_US = (float)(60 * 1000000 / MOTOR_MIN_RPM / ENCODER_PPR);
constexpr auto ENCODER_MIN_PULSE_PERIOD_US = (float)(60 * 1000000 / MOTOR_MAX_RPM / ENCODER_PPR);
constexpr auto ENCODER_CONVERSION_BOT = (float)(1.0 / (int)ENCODER_MAX_PULSE_PERIOD_US);
constexpr auto ENCODER_CONVERSION_TOP = (float)(1.0 / (int)ENCODER_MIN_PULSE_PERIOD_US);
constexpr auto ENCODER_CONVERSION_SPAN = ENCODER_CONVERSION_TOP - ENCODER_CONVERSION_BOT;
constexpr auto ENCODER_WHEEL_LENGTH_MM = ENCODER_WHEEL_DIAMETER_MM * M_PI;
constexpr auto ENCODER_WHEEL_LENGTH = ENCODER_WHEEL_DIAMETER * M_PI;
constexpr auto ENCODER_PULSE_TO_MM = (float)(ENCODER_WHEEL_LENGTH_MM / ENCODER_PPR);
constexpr auto ENCODER_BASE_LENGTH_MM = 184;

constexpr auto ENCODER_PULSE_TO_LENGTH = ENCODER_WHEEL_LENGTH / ENCODER_PPR;
constexpr auto ENCODER_PULSE_TO_LENGTH_MM = ENCODER_WHEEL_LENGTH_MM / ENCODER_PPR;
constexpr auto ENCODER_BASE_LENGTH = 0.184;

/* DRV 1
 * MOTOR     A - 15
 * MOTOR     B -  9
 * MOTOR  PWMA - PWM7B
 * MOTOR  PWMB - PWM4B
 * ENC       A - 23
 * ENC       B - 24
 * ENC PWM CNT - PWM3
 * ENC PWM TIM - PWM2
 */
// constexpr uint DRIVE1_MOTOR_PINA = 15;
// constexpr uint DRIVE1_MOTOR_PINB = 9;
// constexpr uint DRIVE1_MOTOR_PWMA = 7;
// constexpr uint DRIVE1_MOTOR_PWMB = 4;
// constexpr uint DRIVE1_ENCODER_PINA = 23;
// constexpr uint DRIVE1_ENCODER_PINB = 24;
// constexpr uint DRIVE1_ENCODER_PWM = 3;
/* DRV 2
 * MOTOR     A - 14
 * MOTOR     B -  8
 * MOTOR  PWMA - PWM7A
 * MOTOR  PWMB - PWM4A
 * ENC       A -  1
 * ENC       B -  3
 * ENC PWM CNT - PWM0
 * ENC PWM TIM - PWM1
 */
// constexpr uint DRIVE2_MOTOR_PINA = 14;
// constexpr uint DRIVE2_MOTOR_PINB = 8;
// constexpr uint DRIVE2_MOTOR_PWMA = 7;
// constexpr uint DRIVE2_MOTOR_PWMB = 4;
// constexpr uint DRIVE2_ENCODER_PINA = 1;
// constexpr uint DRIVE2_ENCODER_PINB = 3;
// constexpr uint DRIVE2_ENCODER_PWM = 0;

int main()
{

    Init();

    while (true)
    {
        uint64_t time = TIME_U64();
        LED0.Update(time);
        Interface.Process();
        MotorDriverA.Update(time);
        MotorDriverB.Update(time);
        EncoderA.Update(time);
        EncoderB.Update(time);
        ESC0.Update(time);
        IMU0.Update(time);
        LineSensor0.Update(time);

        


    }

    return 0;
}

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

    I2C_Mux = HWConfig.I2C_Mux;
    status = i2c_mux_init(&I2C_Mux);
    if (status != PICO_OK) printf("DBG:I2C_Mux->Init error [%d].\n", status);

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

    //UART
    uart_init(uart_internal, config.UART_BaudRate);
    uart_set_fifo_enabled(uart_internal, true);
    uart_set_translate_crlf(uart_internal, false);
    uart_set_format(uart_internal, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart_internal, false, false);
    gpio_set_function(SDUINO_INTERNAL_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SDUINO_INTERNAL_UART_RX_PIN, GPIO_FUNC_UART);

    //ADC
    adc_init();
    //adc_set_temp_sensor_enabled(true);


    //PIO
    pio_add_program(pio1, &encoder_program);

    return status;
}

void Init(void)
{
    int status = PICO_OK;
    bool canStart = true;
    status = SduinoInit(HWConfig.Sduino);
    if (status != PICO_OK) { canStart = false; printf("DBG:Sduino->Init error [%d].\n", status); }

    Interface.Init(GetVariable, SetVariable);
    NVM.init(FLASH_SECTOR_SIZE, &Config0, sizeof(Config0));

    LoadConfig();
    
    LED0.Init(HWConfig.Sduino.LED_Pin);
    if (status != LED_OK) { canStart = false; printf("DBG:LED0->Init error [%d].\n", status); }

    status = MotorDriverA.Init(HWConfig.MotorDriverA, Config0.MotorDriverA);
    if (status != MOTOR_DRIVER_OK) { canStart = false; printf("DBG:MotorDriverA->Init error [%d].\n", status); }
    status = MotorDriverB.Init(HWConfig.MotorDriverB, Config0.MotorDriverB);
    if (status != MOTOR_DRIVER_OK) { canStart = false; printf("DBG:MotorDriverB->Init error [%d].\n", status); }
    status = EncoderA.Init(HWConfig.EncoderA, Config0.EncoderA);
    if (status != ENCODER_OK) { canStart = false; printf("DBG:EncoderA->Init error [%d].\n", status); }
    status = EncoderB.Init(HWConfig.EncoderB, Config0.EncoderB);
    if (status != ENCODER_OK) { canStart = false; printf("DBG:EncoderB->Init error [%d].\n", status); }
    status = ESC0.Init(HWConfig.ESC0, Config0.ESC0);
    if (status != ESC_OK) { canStart = false; printf("DBG:ESC0->Init error [%d].\n", status); }
    status = IMU0.Init(HWConfig.IMU0, Config0.IMU0);
    if (status != IMU_OK) { canStart = false; printf("DBG:IMU0->Init error [%d].\n", status); }
    status = LineSensor0.Init(HWConfig.LineSensor0, Config0.LineSensor0);
    if (status != LINE_SENSOR_OK) { canStart = false; printf("DBG:LineSensor0->Init error [%d].\n", status); }

    

    if (!canStart)
    {
        for (;;)
        {
            tight_loop_contents();
        }
    }
    uint64_t time = TIME_U64();
    uint64_t prtime = time;
    //LED0.Start(time);
    
    //LineSensor0.Start(time);

    //DEBUG TESTS
    //LED
    LED0.Start(time);
    // LED0.Set(200*1000,200*1000,10,time);
    // while(1) {
    //     LED0.Update();
    // }
    //MotorDrivers
    // MotorDriverA.Start(time);
    // MotorDriverB.Start(time);
    // MotorDriverA.SetPower(0.1f);
    // MotorDriverB.SetPower(0.1f);
    // while(1) {
    //     MotorDriverA.Update();
    //     MotorDriverB.Update();
    // }
    //ESC
    // sleep_ms(000);
    // ESC0.Start(time);
    // ESC0.SetPower(0.1f);
    // while(1) {
    //     ESC0.Update();
    // }
    //Encoders
    // EncoderA.Start(time);
    // EncoderB.Start(time);
    // while(1) {
    //     EncoderA.Update();
    //     EncoderB.Update();
    //     if (time - prtime > 100 * 1000)
    //     {
    //         printf("EncoderA: CNT %ld, STP %ld, RPM %f, RPM_ %f; EncoderB: CNT %ld, STP %ld, RPM %f, RPM_ %f\n",
    //                 EncoderA.CountsLast, EncoderA.StepsLast, EncoderA.RPM, EncoderA.RPM_,
    //                 EncoderB.CountsLast, EncoderB.StepsLast, EncoderB.RPM, EncoderB.RPM_);
    //         prtime = time;
    //     }
    // }
    //IMU
    // IMU0.Start(time);
    // while(1) {
    //     IMU0.Update();
    //     if (time - prtime > 100 * 1000) {
    //         //printf("G> X:% .9f, Y:% .9f, Z:% .9f\n", Gyroscope.Value.X, Gyroscope.Value.Y, Gyroscope.Value.Z);
    //         //printf("A> X:% .9f, Y:% .9f, Z:% .9f\n", Accelerometer.Value.X, Accelerometer.Value.Y, Accelerometer.Value.Z);
    //         //printf("Q> W:% .9F, X:% .9f, Y:% .9f, Z:% .9f\n", quat.W, quat.X, quat.Y, quat.Z);
    //         auto orient = IMU0.GetOrientation();
    //         printf("O> R:% .9f, P:% .9f, Y:% .9f\n", orient.Roll() * 180.0 / M_PI, orient.Pitch() * 180.0 / M_PI, orient.Yaw() * 180.0 / M_PI);
    //         prtime = time;
    //     }
    // }
    //LineSensor
    LineSensor0.Start(time);
    while(1) {
        LineSensor0.Update();
        if (time - prtime > 100 * 1000) {
            printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", LineSensor0.RawData[0], LineSensor0.RawData[1], LineSensor0.RawData[2], LineSensor0.RawData[3],
                       LineSensor0.RawData[4], LineSensor0.RawData[5], LineSensor0.RawData[6], LineSensor0.RawData[7],
                       LineSensor0.RawData[8], LineSensor0.RawData[9], LineSensor0.RawData[10], LineSensor0.RawData[11],
                       LineSensor0.RawData[12], LineSensor0.RawData[13], LineSensor0.RawData[14]);

            prtime = time;
        }
    }
}

// void ESC_Start()
// {
//     if (Start_ESC) return;
//     Start_ESC = true;
//     Stop_ESC = false;
//     ESC0.Start(to_us_since_boot(get_absolute_time()));
// }

// void ESC_Stop()
// {
//     Start_ESC = false;
//     Stop_ESC = true;
//     ESC0.Stop();
// }

// void DriversStop() {
//     MotorDriverA.Stop();
//     MotorDriverB.Stop();
//     PID_Tracking.reset();
//     PID_DriveA.reset();
//     PID_DriveB.reset();
// }

// void DriversBrake() {
//     MotorDriverA.Brake();
//     MotorDriverB.Brake();
//     PID_Tracking.reset();
//     PID_DriveA.reset();
//     PID_DriveB.reset();
// }

// void DriversStart() {
//     MotorDriverA.Start(0);
//     MotorDriverB.Start(0);
// }

// void sleep(void)
// {
//     //if (!LineSensor.isCalibrated() || !IMU.IsCalibrated()) return;
//     DriversStop();
//     //LineSensor.stop();
//     Awake = false;
//     LED0.Set(false);
//     printf("DBG:SLEEP\n");
// }
// void wakeup(void)
// {
//     //LineSensor.start();
//     IMU0.Reset();
//     Awake = true;
//     LED0.Set(500, 500);
//     SleepTime = make_timeout_time_ms(10000);
//     printf("DBG:WAKEUP\n");
// }

// void start(void)
// {
//     if (Start) return;
//     wakeup();
//     Start = true;
//     Stop = false;
//     LED0.Set(50, 50);
//     DriversStart();
// }

// void stop(void)
// {
//     DriversBrake();
//     ESC_Stop();
//     Start = false;
//     Stop = true;
//     LED0.Set(1000, 1000);
//     SleepTime = make_timeout_time_ms(10000);
//     SleepAllowed = true;
// }

// void stop_error(void)
// {
//     DriversBrake();
//     ESC_Stop();
//     Start = false;
//     Stop = true;
//     sleep_ms(1000);
//     DriversStop();
//     LED0.Set(100, 100);
// }

