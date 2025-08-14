/**
 * 2022-11-01, Minduagas Mikalauskas.
 */

#include "main.hpp"
#include "config.hpp"
#include "MMC5603_hw.h"

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
#define LINE_SENSOR_LED_UPDATE_FREQUENCY 30
#define LINE_SENSOR_LED_BLINK_FREQUENCY_CALIB 10
#define LINE_SENSOR_CALIBRATION_TIME 3500
#define LINE_SENSOR_TIMEOUT_TIME 500
#define LINE_SENSOR_MAX_ANGLE_RAD 1.570796326794900
#define LINE_SENSOR_POS_TO_ANGLE_RAD 0.043505588000279
#define LINE_SENSOR_ANGLE_RAD_TO_POS (1 / LINE_SENSOR_POS_TO_ANGLE_RAD)
#define LINE_SENSOR_ANGLE_RAD_TO_VALUE (1 / LINE_SENSOR_VALUE_TO_ANGLE_RAD)
#define LINE_SENSOR_LAST_ANGLE_RAD (LINE_SENSOR_LAST_VALUE * LINE_SENSOR_VALUE_TO_ANGLE_RAD)
#define LINE_SENSOR_EDGE_ANGLE_RAD (LINE_SENSOR_EDGE_VALUE * LINE_SENSOR_VALUE_TO_ANGLE_RAD)
#define LINE_SENSOR_TURN90_ANGLE_RAD ((float)(M_PI / 2))
#define LINE_SENSOR_TURN90_VALUE (LINE_SENSOR_LAST_VALUE * 5.0f)

uint64_t prtime;
uint64_t pidtime;
uint64_t plytatime;
double last_yaw;

void ControlUpdate(const uint64_t& time);

int main()
{

    Init();
    
    

    while (true)
    {
        uint64_t time = TIME_U64();
        LED0.Update(time);
        
        MotorDriverA.Update(time);
        MotorDriverB.Update(time);
        EncoderA.Update(time);
        EncoderB.Update(time);
        ESC0.Update(time);
        IMU0.Update(time);
        LineSensor0.SetDisplayAnalog(LFSYS.LINE_ANALOG);
        LineSensor0.Update(time);
        //DistanceSensor0.Update(time);
        
        ControlUpdate(time);

        Interface.Process();
    }

    return 0;
}

// std::tuple<float, float, float> XYoffsetEncoders(auto dR, auto dL, float A0)
//     {
//         auto A = dR - dL;
//         auto E = (float)(ENCODER_BASE_LENGTH / 2.0) * (dR + dL);
//         auto B = E / A;
//         float AX, AY, C;
//         if (std::isinf(B) || std::isnan(B))
//         {
//             B = (float)(ENCODER_PULSE_TO_LENGTH / 2.0) * (dR + dL);
//             C = 0.0f;
//             AX = std::cos(A0);
//             AY = std::sin(A0);
//         }
//         else
//         {
//             C = (float)(ENCODER_PULSE_TO_LENGTH / ENCODER_BASE_LENGTH) * A;
//             AX =   std::sin(C + A0) - std::sin(A0);
//             AY = -(std::cos(C + A0) - std::cos(A0));
//         }
        
//         return {B * AX, B * AY, C};
//     }

void ControlUpdate(const uint64_t& time = TIME_U64())
{
    auto orient = IMU0.GetOrientation();
    auto yaw = orient.Yaw();
    float Position = 0;

    auto VL = EncoderA.GetVelocity();
    auto VR = EncoderB.GetVelocity();

    double V = (VR + VL) / 2;
    double W = (VR - VL) / ENCODER_BASE_LENGTH;

    LFSYS.SpeedL = VL;
    LFSYS.SpeedR = VR;
    LFSYS.SpeedV = V;
    LFSYS.SpeedW = W;

    switch (LineSensor0.GetTurnDirection())
    {
    case TURN_LEFT:
        Position = std::clamp(yaw - last_yaw - (M_PI / 2), -M_PI, M_PI);
        break;
    case TURN_RIGHT:
        Position = std::clamp(yaw - last_yaw + (M_PI / 2), -M_PI, M_PI);
        break;
    case TURN_NONE: //Matom linija ir galim ja sekti, arba nematom linijos ir sekam paskutinia puse
    default:
        last_yaw = yaw;
        Position = LineSensor0.LineCenter();
        break;
    }
    
    if (time - pidtime > LFSYS.Config.loop_time)
    {
        float dt = (float)(time - pidtime) / 1000000;

        float fspeed = LFSYS.Config.M_Speed;
        float maxSpeed = LFSYS.Config.Max_Speed;
        float esc_speed = LFSYS.Config.ESC_Speed;

        float pid = PID_Main.Compute(0, Position, dt);

        float pwrA = std::clamp(fspeed - pid, -maxSpeed, maxSpeed);
        float pwrB = std::clamp(fspeed + pid, -maxSpeed, maxSpeed);

        if (LFSYS.Start && !LFSYS.Stop)
        {
            MotorDriverA.SetPower(-PID_MotorA.Compute(pwrA * MOTOR_MAX_RPM, EncoderA.GetRPM(), dt));
            MotorDriverB.SetPower(PID_MotorB.Compute(pwrB * MOTOR_MAX_RPM, EncoderB.GetRPM(), dt));
            ESC0.SetPower(esc_speed);
        }
    }

    

    //if (time - prtime > 500 * 1000) {
    //     prtime = time;
    //     auto orient = IMU0.GetOrientation();
    //     printf("OR> R:% .2f, P:% .2f, Y:% .2f\n", orient.Roll(), orient.Pitch(), orient.Yaw());
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
    //}
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



void ESC_Start(void)
{
    if (ESC0.Start() != ESC_OK) { printf("DBG:ESC0->Start error.\n"); }
    LED0.Set(100, 100);

    LFSYS.ESC_Start = true;
}

void ESC_Stop(void)
{
    if (ESC0.Stop() != ESC_OK) { printf("DBG:ESC0->Stop error.\n"); }
    LED0.Set(500, 500);

    LFSYS.ESC_Start = false;
}

void Start(void)
{
    Wakeup();
    uint64_t time = TIME_U64();

    if (MotorDriverA.Start(time) != MOTOR_DRIVER_OK) { printf("DBG:MotorDriverA->Start error.\n"); }
    if (MotorDriverB.Start(time) != MOTOR_DRIVER_OK) { printf("DBG:MotorDriverB->Start error.\n"); }
    if (ESC0.Start(time) != ESC_OK) { printf("DBG:ESC0->Start error.\n"); }
    //if (DistanceSensor0.Start(time) != DISTANCE_SENSOR_OK) { printf("DBG:DistanceSensor0->Start error.\n"); }

    LED0.Set(100, 100);

    LFSYS.Sleep = false;
    LFSYS.Start = true;
    LFSYS.Stop  = false;
}

void Stop(void)
{
    if (MotorDriverA.Brake() != MOTOR_DRIVER_OK) { printf("DBG:MotorDriverA->Brake error.\n"); }
    if (MotorDriverB.Brake() != MOTOR_DRIVER_OK) { printf("DBG:MotorDriverB->Brake error.\n"); }
    if (ESC0.Stop() != ESC_OK) { printf("DBG:ESC0->Stop error.\n"); }

    LED0.Set(1000, 1000);

    LFSYS.Sleep = false;
    LFSYS.Start = false;
    LFSYS.Stop  = true;
}

void Wakeup(void)
{
    uint64_t time = TIME_U64();

    if (EncoderA.Start(time) != ENCODER_OK) { printf("DBG:EncoderA->Start error.\n"); }
    if (EncoderB.Start(time) != ENCODER_OK) { printf("DBG:EncoderB->Start error.\n"); }
    if (IMU0.Start(time) != IMU_OK) { printf("DBG:IMU0->Start error.\n"); }
    if (LineSensor0.Start(time) != LINE_SENSOR_OK) { printf("DBG:LineSensor0->Start error.\n"); }
    //if (DistanceSensor0.Start(time) != DISTANCE_SENSOR_OK) { printf("DBG:DistanceSensor0->Start error.\n"); }

    LED0.Set(100, 900);

    LFSYS.Sleep = false;
    LFSYS.Start = false;
    LFSYS.Stop  = false;
}

void Sleep(void)
{
    Stop();
    if (EncoderA.Stop() != ENCODER_OK) { printf("DBG:EncoderA->Stop error.\n"); }
    if (EncoderB.Stop() != ENCODER_OK) { printf("DBG:EncoderB->Stop error.\n"); }
    if (IMU0.Stop() != IMU_OK) { printf("DBG:IMU0->Stop error.\n"); }
    if (LineSensor0.Stop() != LINE_SENSOR_OK) { printf("DBG:LineSensor0->Stop error.\n"); }
    //if (DistanceSensor0.Stop() != DISTANCE_SENSOR_OK) { printf("DBG:DistanceSensor0->Stop error.\n"); }

    LED0.Set(100, 900);

    LFSYS.Sleep = true;
    LFSYS.Start = false;
    LFSYS.Stop  = false;
}

int SduinoInit(const SduinoConfig& config)
{
    int status = PICO_OK;
    //USB
    stdio_init_all();

    //
    //while (!stdio_usb_connected()) { sleep_ms(500); }

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
    adc_init();
    //adc_set_temp_sensor_enabled(true);


    //PIO
    pio_add_program(pio1, &encoder_program);

    return status;
}

void sduino_adc_read()
{
    float Volts[5];
    /* 12-bit conversion, assume max value == ADC_VREF == 3.08 V */
    const float conversionFactor = 3.035f / 4095.0f;

    for (uint i = 0; i < 3; i++)
    {
        adc_select_input(i);
        Volts[i] = adc_read() * conversionFactor;
        //float Ia = Volts[1] * 1500 / DRV_R;
    }
    //BatteryVoltage = adc_read() * conversionFactor * 7.2397;
    //float temperature = 27.0f - (Volts[4] - 0.706f) / 0.001721f;
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

    LFSYS.Config = Config0.LFCFG;
    
    status = LED0.Init(HWConfig.Sduino.LED_Pin);
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
    // status = DistanceSensor0.Init(HWConfig.DistanceSensor0, {});
    // if (status != DISTANCE_SENSOR_OK) { canStart = false; printf("DBG:DistanceSensor0->Init error [%d].\n", status); }
    

    LFSYS.Start = false;
    LFSYS.Stop  = false;
    LFSYS.Sleep = true;
    LFSYS.ESC_Start = false;
    LFSYS.LINE_ANALOG = false;

    if (LED0.Start() != LED_OK) { printf("DBG:LED0->Start error.\n"); }

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

