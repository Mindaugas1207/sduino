#include "hardware/adc.h"
#include "line_follower.hpp"


NVM_s NVM;
port_spi_t spi_port;
port_i2c_t i2c_port;
Interface_s Interface;
line_sesnor_hw_inst_t LineSensorHW;
LineSensor_s LineSensor;;
MotorDriver_s DriveA;
MotorDriver_s DriveB;
PID_s PID_DriveA;
PID_s PID_DriveB;
PID_s PID_Cruise;
PID_s PID_Steering;
IMU_s IMU;

Blink_s LED0;

void sduino_adc_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
}

#define DRV_R 473.5f
constexpr auto MOTOR_B_ENCODER_A_PIN = 1U;
constexpr auto MOTOR_B_ENCODER_B_PIN = 3U;
constexpr auto MOTOR_A_ENCODER_A_PIN = 23U;
constexpr auto MOTOR_A_ENCODER_B_PIN = 24U;
constexpr auto MOTOR_B_ENCODER_PINS_MASK = 1U << MOTOR_B_ENCODER_A_PIN | 1U << MOTOR_B_ENCODER_B_PIN;
constexpr auto MOTOR_A_ENCODER_PINS_MASK = 1U << MOTOR_A_ENCODER_A_PIN | 1U << MOTOR_A_ENCODER_B_PIN;
constexpr auto MOTOR_MIN_RPM = 60;
//constexpr auto MOTOR_NOMINAL_RPM = 2800;
constexpr auto MOTOR_MAX_RPM = 2800;
constexpr auto ENCODER_PPR = 237.6f;
constexpr auto ENCODER_MAX_PULSE_PERIOD_US = 60 * 1000000 / MOTOR_MIN_RPM / ENCODER_PPR;
constexpr auto ENCODER_MIN_PULSE_PERIOD_US = 60 * 1000000 / MOTOR_MAX_RPM / ENCODER_PPR;
constexpr auto ENCODER_UPDATE_PERIOD_US = 50000;
constexpr auto ENCODER_UPDATE_RETRY_PERIOD_US = 30;
constexpr auto ENCODER_SAMPLE_BUFFER_SIZE = 128;
constexpr auto ENCODER_CONVERSION_BOT = 1.0f / (int)ENCODER_MAX_PULSE_PERIOD_US;
constexpr auto ENCODER_CONVERSION_TOP = 1.0f / (int)ENCODER_MIN_PULSE_PERIOD_US;
constexpr auto ENCODER_CONVERSION_SPAN = ENCODER_CONVERSION_TOP - ENCODER_CONVERSION_BOT;

struct EncoderData_s {
    int PulseCountA, PulseCountB, PulsePeriodSumA, PulsePeriodSumB, PulseDirectionA, PulseDirectionB;
};

constexpr auto ENCODER_QUEUE_SIZE = 10U;
constexpr auto ENCODER_QUEUE_DATA_SIZE = sizeof(EncoderData_s);

queue_t gEncoderQueue;

std::tuple<EncoderData_s, bool> getEncoderData()
{
    EncoderData_s NewData;

    bool Ok = queue_try_remove(&gEncoderQueue, &NewData);

    return {NewData, Ok};
}

void __not_in_flash_func(encoderProcess)()
{
    multicore_lockout_victim_init();

    auto getStateAB = [](){
        auto gpio_state = gpio_get_all();
        auto PulseStateA = (gpio_state & MOTOR_A_ENCODER_PINS_MASK) >> MOTOR_A_ENCODER_A_PIN;
        auto PulseStateB = (gpio_state & MOTOR_B_ENCODER_PINS_MASK) >> MOTOR_B_ENCODER_A_PIN;
        PulseStateB = (PulseStateB >> 1 | PulseStateB) & 3;
        return std::tuple<uint, uint>{PulseStateA, PulseStateB};
    };

    auto getPulseDirection = [](uint LastPulseState, uint NewPulseState){
        switch (LastPulseState << 2 | NewPulseState)
        {
            case 0b0010: case 0b0100: case 0b1011: case 0b1101: return +1;
            case 0b0001: case 0b0111: case 0b1000: case 0b1110: return -1;
            default: return 0;
        }
    };

    auto getTimeStamp = [](){
        auto hi = timer_hw->timerawh;
        auto lo = timer_hw->timerawl;
        auto next_hi = timer_hw->timerawh;
        if (hi != next_hi) hi = next_hi;
        return (uint64_t)hi << 32u | lo;
    };

    std::size_t SampleIndexA = 0, SampleIndexB = 0;
    std::array<int, ENCODER_SAMPLE_BUFFER_SIZE> PulsePeriodSamplesA, PulsePeriodSamplesB;
    PulsePeriodSamplesA.fill(ENCODER_MAX_PULSE_PERIOD_US); PulsePeriodSamplesB.fill(ENCODER_MAX_PULSE_PERIOD_US);

    uint64_t LastTimeStampA, LastTimeStampB, TimeOutTimeStampA, TimeOutTimeStampB, UpdateTimeStamp;
    LastTimeStampA = LastTimeStampB = TimeOutTimeStampA = TimeOutTimeStampB = UpdateTimeStamp = getTimeStamp();

    auto [LastPulseStateA, LastPulseStateB] = getStateAB();

    int PulseCountA = 0, PulseCountB = 0, PulseDirectionA = 0, PulseDirectionB = 0, PulsePeriodSumA, PulsePeriodSumB;
    PulsePeriodSumA = PulsePeriodSumB = ENCODER_MAX_PULSE_PERIOD_US * ENCODER_SAMPLE_BUFFER_SIZE;

    for (;;)
    {
        auto NewTimeStamp = getTimeStamp();
        auto [NewPulseStateA, NewPulseStateB] = getStateAB();
        
        if (NewPulseStateA != LastPulseStateA)
        {
            PulseDirectionA = getPulseDirection(LastPulseStateA, NewPulseStateA);
            PulseCountA += PulseDirectionA;
            
            PulsePeriodSumA -= PulsePeriodSamplesA[SampleIndexA];
            PulsePeriodSumA += PulsePeriodSamplesA[SampleIndexA ++] = NewTimeStamp - LastTimeStampA;
            if (SampleIndexA >= ENCODER_SAMPLE_BUFFER_SIZE) SampleIndexA = 0;

            LastPulseStateA = NewPulseStateA;
            LastTimeStampA = NewTimeStamp;
            TimeOutTimeStampA = NewTimeStamp + ENCODER_MAX_PULSE_PERIOD_US;
        }
        else if (NewTimeStamp > TimeOutTimeStampA)
        {
            PulsePeriodSumA -= PulsePeriodSamplesA[SampleIndexA];
            PulsePeriodSumA += PulsePeriodSamplesA[SampleIndexA ++] = ENCODER_MAX_PULSE_PERIOD_US;
            if (SampleIndexA >= ENCODER_SAMPLE_BUFFER_SIZE) SampleIndexA = 0;

            LastTimeStampA = NewTimeStamp;
            TimeOutTimeStampA = NewTimeStamp + ENCODER_MAX_PULSE_PERIOD_US;
        }

        if (NewPulseStateB != LastPulseStateB)
        {
            PulseDirectionB = getPulseDirection(LastPulseStateB, NewPulseStateB);
            PulseCountB += PulseDirectionB;

            PulsePeriodSumB -= PulsePeriodSamplesB[SampleIndexB];
            PulsePeriodSumB += PulsePeriodSamplesB[SampleIndexB ++] = NewTimeStamp - LastTimeStampB;
            if (SampleIndexB >= ENCODER_SAMPLE_BUFFER_SIZE) SampleIndexB = 0;

            LastPulseStateB = NewPulseStateB;
            LastTimeStampB = NewTimeStamp;
            TimeOutTimeStampB = NewTimeStamp + ENCODER_MAX_PULSE_PERIOD_US;
        }
        else if (NewTimeStamp > TimeOutTimeStampB)
        {
            PulsePeriodSumB -= PulsePeriodSamplesB[SampleIndexB];
            PulsePeriodSumB += PulsePeriodSamplesB[SampleIndexB ++] = ENCODER_MAX_PULSE_PERIOD_US;
            if (SampleIndexB >= ENCODER_SAMPLE_BUFFER_SIZE) SampleIndexB = 0;

            LastTimeStampB = NewTimeStamp;
            TimeOutTimeStampB = NewTimeStamp + ENCODER_MAX_PULSE_PERIOD_US;
        }

        if (NewTimeStamp > UpdateTimeStamp)
        {
            EncoderData_s NewData = {
                .PulseCountA = PulseCountA,
                .PulseCountB = PulseCountB,
                .PulsePeriodSumA = PulsePeriodSumA,
                .PulsePeriodSumB = PulsePeriodSumB,
                .PulseDirectionA = PulseDirectionA,
                .PulseDirectionB = PulseDirectionB,
            };

            if (queue_try_add(&gEncoderQueue, &NewData))
                UpdateTimeStamp = NewTimeStamp + ENCODER_UPDATE_PERIOD_US;
            else
                UpdateTimeStamp = NewTimeStamp + ENCODER_UPDATE_RETRY_PERIOD_US;
        }
    }
}

void encodersInit()
{
    gpio_init(MOTOR_A_ENCODER_A_PIN);
    gpio_set_dir(MOTOR_A_ENCODER_A_PIN, GPIO_IN);
    gpio_init(MOTOR_A_ENCODER_B_PIN);
    gpio_set_dir(MOTOR_A_ENCODER_B_PIN, GPIO_IN);

    gpio_init(MOTOR_B_ENCODER_A_PIN);
    gpio_set_dir(MOTOR_B_ENCODER_A_PIN, GPIO_IN);
    gpio_init(MOTOR_B_ENCODER_B_PIN);
    gpio_set_dir(MOTOR_B_ENCODER_B_PIN, GPIO_IN);

    queue_init(&gEncoderQueue, ENCODER_QUEUE_DATA_SIZE, ENCODER_QUEUE_SIZE);

    multicore_launch_core1(encoderProcess);
}

void ButterworthLowpassFilter0050SixthOrder(const float src[], float dest[], int size)
{ 	
	const int NZEROS = 6;
	const int NPOLES = 6;
	const float GAIN = 1.165969038e+05;

	float xv[NZEROS+1] = {0.0}, yv[NPOLES+1] = {0.0};

    for (int i = 0; i < size; i++)
  	{ 
		xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; xv[5] = xv[6]; 
        xv[6] = src[i] / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; yv[5] = yv[6]; 
        yv[6] =   (xv[0] + xv[6]) + 6.0 * (xv[1] + xv[5]) + 15.0 * (xv[2] + xv[4])
                     + 20.0 * xv[3]
                     + ( -0.2951724313 * yv[0]) + (  2.1290387500 * yv[1])
                     + ( -6.4411118810 * yv[2]) + ( 10.4690788930 * yv[3])
                     + ( -9.6495177287 * yv[4]) + (  4.7871354989 * yv[5]);
        dest[i] = yv[6];
    }
}

    // sduino_adc_init();

    // uint16_t Buffer[5];
    // float Volts[5];
    // const float conversionFactor = 3.035f / 4095.0f;

    //float avgI = 0.0f;

    //absolute_time_t ptime = get_absolute_time();

        // adc_select_input(1);
        // Buffer[1] = adc_read();
        // Volts[1] = (float)Buffer[1] * conversionFactor;
        // float Ia = Volts[1] * 1500 / DRV_R;

        // avgI = avgI * 0.5f + Ia * 0.5f;

        // for (uint i = 0; i < 5; i++)
        // {
        //     adc_select_input(i);
        //     Buffer[i] = adc_read();
        //     Volts[i] = (float)Buffer[i] * conversionFactor;
        // }

        // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(ptime))
        // {
        //     //printf("%d %d\n", m1_cnt, m2_cnt);
        //     printf("Ia %.3f A\n", avgI);
        
        //     ptime = make_timeout_time_ms(500);
        // }

// void sduino_adc_test()
// {
//     uint16_t Buffer[5];
//     float Volts[5];
//     /* 12-bit conversion, assume max value == ADC_VREF == 3.08 V */
//     const float conversionFactor = 3.035f / 4095.0f;

//     for (uint i = 0; i < 5; i++)
//     {
//         adc_select_input(i);
//         Buffer[i] = adc_read();
//         Volts[i] = (float)Buffer[i] * conversionFactor;
//     }



//     //float temperature = 27.0f - (Volts[4] - 0.706f) / 0.001721f;
//     //printf("AIN0 = %.9f, AIN1 = %.9f, AIN2 = %.9f, AIN3 = %.9f, AIN4 = %.9f, TC = %.02f C\n", Volts[0], Volts[1], Volts[2]*7.2397, Volts[3], Volts[4], temperature);
// }

void DriversStop() {
    DriveA.stop();
    DriveB.stop();
    PID_DriveA.reset();
    PID_DriveB.reset();
}

void DriversStart() {
    DriveA.start();
    DriveB.start();
}

void LineFollower_s::sleep(void)
{
    LineSensor.setEmittersEnable(false);
    LineSensor.setLedsEnable(false);
    LineSensor.stop();
    Awake = false;
    printf("DBG:SLEEP\n");
}

void LineFollower_s::wakeup(void)
{
    LineSensor.setEmittersEnable(true);
    LineSensor.setLedsEnable(true);
    LineSensor.start();
    Awake = true;
    SleepTime = make_timeout_time_ms(10000);
    printf("DBG:WAKEUP\n");
}

void LineFollower_s::start(void)
{
    wakeup();
    Start = true;
    Stop = false;
    LED0.stop();
    DriversStart();
    printf("DBG:START\n");
}

void LineFollower_s::stop(void)
{
    DriversStop();
    Start = false;
    Stop = true;
    LED0.start(1000, 1000);
    // MotorA.stopBeep();
    // MotorB.stopBeep();
    SleepTime = make_timeout_time_ms(10000);
    printf("DBG:STOP\n");
}

void LineFollower_s::stop_error(void)
{
    DriversStop();
    Start = false;
    Stop = true;
    sleep_ms(500);
    LED0.start(500, 500);
    // MotorA.startBeep(500, 500);
    // MotorB.startBeep(500, 500);
    printf("DBG:STOP_ERROR\n");
}

void LineFollower_s::init(void)
{
    LED0.init(SDUINO_INTERNAL_LED_PIN);
    stdio_init_all();
   
    while (!stdio_usb_connected()) { sleep_ms(500); }

    i2c_init(i2c_internal, 400 * 1000);
    gpio_set_function(SDUINO_INTERNAL_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDUINO_INTERNAL_I2C_SCL_PIN, GPIO_FUNC_I2C);

    spi_init(spi_internal, 10000 * 1000);
    gpio_set_function(SDUINO_INTERNAL_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SDUINO_INTERNAL_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SDUINO_INTERNAL_SPI_TX_PIN, GPIO_FUNC_SPI);

    gpio_init(SDUINO_INTERNAL_BMP_CS_PIN);
    gpio_set_dir(SDUINO_INTERNAL_BMP_CS_PIN, GPIO_OUT);
    gpio_init(SDUINO_INTERNAL_IMU_ACCEL_CS_PIN);
    gpio_set_dir(SDUINO_INTERNAL_IMU_ACCEL_CS_PIN, GPIO_OUT);
    gpio_init(SDUINO_INTERNAL_IMU_GYRO_CS_PIN);
    gpio_set_dir(SDUINO_INTERNAL_IMU_GYRO_CS_PIN, GPIO_OUT);

    gpio_put(SDUINO_INTERNAL_BMP_CS_PIN, 1);
    gpio_put(SDUINO_INTERNAL_IMU_ACCEL_CS_PIN, 1);
    gpio_put(SDUINO_INTERNAL_IMU_GYRO_CS_PIN, 1);

    uart_init(uart_internal, 115200);

    gpio_set_function(SDUINO_INTERNAL_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SDUINO_INTERNAL_UART_RX_PIN, GPIO_FUNC_UART);

    uart_set_fifo_enabled(uart_internal, true);
    uart_set_translate_crlf(uart_internal, false);
    uart_set_format(uart_internal, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart_internal, false, false);

    EncoderPulseCountA = 0;
    EncoderPulseCountB = 0;
    EncoderSpeedA = 0.0f;
    EncoderSpeedB = 0.0f;
    encodersInit();

    Interface.Init(f_getCallback, f_setCallback);
    NVM.init(FLASH_SECTOR_SIZE, &Config, sizeof(Config));

    DriveA.init(SDUINO_INTERNAL_DRV_A_IN1_PIN, SDUINO_INTERNAL_DRV_A_IN2_PIN);
    DriveB.init(SDUINO_INTERNAL_DRV_B_IN1_PIN, SDUINO_INTERNAL_DRV_B_IN2_PIN);

    if (port_i2c_init(&i2c_port, i2c_internal, PORT_MUX_DEFAULT_ADDRESS) == PORT_ERROR) printf("DBG:PORT_I2C_INIT->FAIL\n");
    if (port_spi_init(&spi_port, spi_internal) == PORT_ERROR) printf("DBG:PORT_SPI_INIT->FAIL\n");

    LineSensorHW.port_inst = &i2c_port;
    if (LineSensor.init(&LineSensorHW) == LINE_SENSOR_ERROR) {
        printf("DBG:LINE_SENSOR_INIT->FAIL\n");
        while (true) { tight_loop_contents(); }
    }

    if(!IMU.init(&i2c_port, &spi_port)) {
        printf("DBG:IMU_INIT->FAIL\n");
        while (true) { tight_loop_contents(); }
    }
    load();

    IMU.startAsyncProcess();
    wakeup();

    stop();
    //sleep();
}

void LineFollower_s::save(void)
{
    Config = {
        .IMU = IMU.getConfiguration(),
        .LineSensor = LineSensor.getConfiguration(),
        .DriveA = DriveA.getConfiguration(),
        .DriveB = DriveB.getConfiguration(),
        .PID_DriveA = PID_DriveA.getConfiguration(),
        .PID_DriveB = PID_DriveB.getConfiguration(),
        .PID_Cruise = PID_Cruise.getConfiguration(),
        .PID_Steering = PID_Steering.getConfiguration(),
    };

    Config.setLockCode(NVM_CONFG_LOCK_CODE);
    NVM.program();
    NVM_SAVE_OK = true;
}

void LineFollower_s::load(void)
{
    NVM_CONFIG_OK = false;
    NVM.load();
    if (Config.getLockCode() == NVM_CONFG_LOCK_CODE)
    {
        //Load IMU
        IMU.loadConfiguration(Config.IMU);
        //Load Line Sensor
        LineSensor.loadConfiguration(Config.LineSensor);
        //Load Drivers
        DriveA.loadConfiguration(Config.DriveA);
        DriveB.loadConfiguration(Config.DriveB);
        //Load PID_DriveA
        PID_DriveA.loadConfiguration(Config.PID_DriveA);
        PID_DriveB.loadConfiguration(Config.PID_DriveB);
        PID_Cruise.loadConfiguration(Config.PID_Cruise);
        PID_Steering.loadConfiguration(Config.PID_Steering);
        
        NVM_CONFIG_OK = true;
        NVM_LOAD_OK = true;
    }
    else
    {
        //NVM empty or corrupted
        //Load Default Configurations

        PID_s::Config_t DrivePID = {
            .SetPoint = 0.0f,
            .Gain = 0.8f,
            .IntegralTime = 0.1f,
            .IntegralLimit = 2.5f,
            .IntegralRateLimit = 1.0f,
            .IntegralAntiWindup = 0.0f,
            .DerivativeTime = 0.015f,
            .DerivativeCutoff = 1.0f,
            .OutputLimit = 1.0f,
            .DeadZone = 0.026f
        };

        PID_DriveA.loadConfiguration(DrivePID);
        PID_DriveB.loadConfiguration(DrivePID);

        PID_s::Config_t CruisePID = {
            .SetPoint = 0.0f,
            .Gain = 0.0f,
            .IntegralTime = 0.0f,
            .IntegralLimit = 0.0f,
            .IntegralRateLimit = 0.0f,
            .IntegralAntiWindup = 0.0f,
            .DerivativeTime = 0.0f,
            .DerivativeCutoff = 1.0f,
            .OutputLimit = 1.0f,
            .DeadZone = 0.0f
        };

        PID_s::Config_t SteeringPID = {
            .SetPoint = 0.0f,
            .Gain = 0.0f,
            .IntegralTime = 0.0f,
            .IntegralLimit = 0.0f,
            .IntegralRateLimit = 0.0f,
            .IntegralAntiWindup = 0.0f,
            .DerivativeTime = 0.0f,
            .DerivativeCutoff = 1.0f,
            .OutputLimit = 1.0f,
            .DeadZone = 0.0f
        };

        PID_Cruise.loadConfiguration(CruisePID);
        PID_Steering.loadConfiguration(SteeringPID);
        
        Config.StateFlags = 0;
        Config.ForwardSpeed = 0.0f;
        Config.TurnBiasPositive = 0.0f;
        Config.TurnBiasNegative = 0.0f;
        Config.TurnMaxPositive = 0.0f;
        Config.TurnMaxNegative = 0.0f;
        Config.SpeedDown = 0.0f;
        
        save();
    }
}

void LineFollower_s::erase(void)
{
    NVM.erase();
    load();
    NVM_ERASE_OK = true;
}


std::tuple<int, float> LineFollower_s::get(int _Enum)
{
    switch(_Enum)
    {
    /*Commands variables                    */
    case CMD_START:                         return {INTERFACE_GET_OK, Start && !Stop};
    case CMD_STOP:                          return {INTERFACE_GET_OK, !Start && Stop};
    case CMD_LINE_CALIBRATE:                return {INTERFACE_GET_OK, LineSensor.isCalibrated()};
    case CMD_IMU_CALIBRATE:                 return {INTERFACE_GET_OK, IMU.isCalibrated()};
    case CMD_NVM_ERASE:                     return {INTERFACE_GET_OK, NVM_ERASE_OK};
    case CMD_NVM_LOAD:                      return {INTERFACE_GET_OK, NVM_LOAD_OK};
    case CMD_NVM_SAVE:                      return {INTERFACE_GET_OK, NVM_SAVE_OK};
    case CMD_LINE_SLEEP:                    return {INTERFACE_GET_OK, !Awake};
    case CMD_LINE_WAKEUP:                   return {INTERFACE_GET_OK, Awake};
    /*Common variables                      */
    case VAR_SPEED:                         return {INTERFACE_GET_OK, Config.ForwardSpeed};
    case VAR_TURN_POS:                      return {INTERFACE_GET_OK, Config.TurnMaxPositive};
    case VAR_TURN_NEG:                      return {INTERFACE_GET_OK, Config.TurnMaxNegative};
    case VAR_TURN_BIAS_POS:                 return {INTERFACE_GET_OK, Config.TurnBiasPositive};
    case VAR_TURN_BIAS_NEG:                 return {INTERFACE_GET_OK, Config.TurnBiasNegative};
    case VAR_SPEED_DOWN:                    return {INTERFACE_GET_OK, Config.SpeedDown};
    /*Driver variables                       */
    case VAR_DRIVE0_PWM_WRAP_VALUE:         return {INTERFACE_GET_OK, DriveA.getPWM_WRAP_VALUE()};
    case VAR_DRIVE0_PWM_CLK_DIV:            return {INTERFACE_GET_OK, DriveA.getPWM_CLK_DIV()};
    case VAR_DRIVE0_BIAS:                   return {INTERFACE_GET_OK, DriveA.getBias()};
    case VAR_DRIVE0_DEADZONE:               return {INTERFACE_GET_OK, DriveA.getDeadZone()};
    case VAR_DRIVE1_PWM_WRAP_VALUE:         return {INTERFACE_GET_OK, DriveB.getPWM_WRAP_VALUE()};
    case VAR_DRIVE1_PWM_CLK_DIV:            return {INTERFACE_GET_OK, DriveB.getPWM_CLK_DIV()};
    case VAR_DRIVE1_BIAS:                   return {INTERFACE_GET_OK, DriveB.getBias()};
    case VAR_DRIVE1_DEADZONE:               return {INTERFACE_GET_OK, DriveB.getDeadZone()};
    /*Line variables                        */
    case VAR_LINE_EMITTER_POWER:            return {INTERFACE_GET_OK, LineSensor.getEmittersPower()};
    case VAR_LINE_LED_BRIGHTNESS:           return {INTERFACE_GET_OK, LineSensor.getLedsBrightness()};
    case VAR_LINE_TURN_GAIN_1:              return {INTERFACE_GET_OK, LineSensor.getTurnGain1()};
    case VAR_LINE_TURN_GAIN_2:              return {INTERFACE_GET_OK, LineSensor.getTurnGain2()};
    /*PID0 variables                        */
    case VAR_PID0_SETPOINT:                 return {INTERFACE_GET_OK, PID_DriveA.getSetPoint()};
    case VAR_PID0_GAIN:                     return {INTERFACE_GET_OK, PID_DriveA.getGain()};
    case VAR_PID0_INTEGRAL_TIME:            return {INTERFACE_GET_OK, PID_DriveA.getIntegralTime()};
    case VAR_PID0_INTEGRAL_LIMIT:           return {INTERFACE_GET_OK, PID_DriveA.getIntegralLimit()};
    case VAR_PID0_INTEGRAL_RATE_LIMIT:      return {INTERFACE_GET_OK, PID_DriveA.getIntegralRateLimit()};
    case VAR_PID0_INTEGRAL_ANTI_WINDUP:     return {INTERFACE_GET_OK, PID_DriveA.getIntegralAntiWindup()};
    case VAR_PID0_DERIVATIVE_TIME:          return {INTERFACE_GET_OK, PID_DriveA.getDerivativeTime()};
    case VAR_PID0_DERIVATIVE_CUTOFF:        return {INTERFACE_GET_OK, PID_DriveA.getDerivativeCutoff()};
    case VAR_PID0_OUTPUT_LIMIT:             return {INTERFACE_GET_OK, PID_DriveA.getOutputLimit()};
    case VAR_PID0_DEADZONE:                 return {INTERFACE_GET_OK, PID_DriveA.getDeadZone()};
    /*PID1 variables                        */
    case VAR_PID1_SETPOINT:                 return {INTERFACE_GET_OK, PID_DriveB.getSetPoint()};
    case VAR_PID1_GAIN:                     return {INTERFACE_GET_OK, PID_DriveB.getGain()};
    case VAR_PID1_INTEGRAL_TIME:            return {INTERFACE_GET_OK, PID_DriveB.getIntegralTime()};
    case VAR_PID1_INTEGRAL_LIMIT:           return {INTERFACE_GET_OK, PID_DriveB.getIntegralLimit()};
    case VAR_PID1_INTEGRAL_RATE_LIMIT:      return {INTERFACE_GET_OK, PID_DriveB.getIntegralRateLimit()};
    case VAR_PID1_INTEGRAL_ANTI_WINDUP:     return {INTERFACE_GET_OK, PID_DriveB.getIntegralAntiWindup()};
    case VAR_PID1_DERIVATIVE_TIME:          return {INTERFACE_GET_OK, PID_DriveB.getDerivativeTime()};
    case VAR_PID1_DERIVATIVE_CUTOFF:        return {INTERFACE_GET_OK, PID_DriveB.getDerivativeCutoff()};
    case VAR_PID1_OUTPUT_LIMIT:             return {INTERFACE_GET_OK, PID_DriveB.getOutputLimit()};
    case VAR_PID1_DEADZONE:                 return {INTERFACE_GET_OK, PID_DriveB.getDeadZone()};
    /*PID2 variables                        */
    case VAR_PID2_SETPOINT:                 return {INTERFACE_GET_OK, PID_Cruise.getSetPoint()};
    case VAR_PID2_GAIN:                     return {INTERFACE_GET_OK, PID_Cruise.getGain()};
    case VAR_PID2_INTEGRAL_TIME:            return {INTERFACE_GET_OK, PID_Cruise.getIntegralTime()};
    case VAR_PID2_INTEGRAL_LIMIT:           return {INTERFACE_GET_OK, PID_Cruise.getIntegralLimit()};
    case VAR_PID2_INTEGRAL_RATE_LIMIT:      return {INTERFACE_GET_OK, PID_Cruise.getIntegralRateLimit()};
    case VAR_PID2_INTEGRAL_ANTI_WINDUP:     return {INTERFACE_GET_OK, PID_Cruise.getIntegralAntiWindup()};
    case VAR_PID2_DERIVATIVE_TIME:          return {INTERFACE_GET_OK, PID_Cruise.getDerivativeTime()};
    case VAR_PID2_DERIVATIVE_CUTOFF:        return {INTERFACE_GET_OK, PID_Cruise.getDerivativeCutoff()};
    case VAR_PID2_OUTPUT_LIMIT:             return {INTERFACE_GET_OK, PID_Cruise.getOutputLimit()};
    case VAR_PID2_DEADZONE:                 return {INTERFACE_GET_OK, PID_Cruise.getDeadZone()};
    /*PID3 variables                        */
    case VAR_PID3_SETPOINT:                 return {INTERFACE_GET_OK, PID_Steering.getSetPoint()};
    case VAR_PID3_GAIN:                     return {INTERFACE_GET_OK, PID_Steering.getGain()};
    case VAR_PID3_INTEGRAL_TIME:            return {INTERFACE_GET_OK, PID_Steering.getIntegralTime()};
    case VAR_PID3_INTEGRAL_LIMIT:           return {INTERFACE_GET_OK, PID_Steering.getIntegralLimit()};
    case VAR_PID3_INTEGRAL_RATE_LIMIT:      return {INTERFACE_GET_OK, PID_Steering.getIntegralRateLimit()};
    case VAR_PID3_INTEGRAL_ANTI_WINDUP:     return {INTERFACE_GET_OK, PID_Steering.getIntegralAntiWindup()};
    case VAR_PID3_DERIVATIVE_TIME:          return {INTERFACE_GET_OK, PID_Steering.getDerivativeTime()};
    case VAR_PID3_DERIVATIVE_CUTOFF:        return {INTERFACE_GET_OK, PID_Steering.getDerivativeCutoff()};
    case VAR_PID3_OUTPUT_LIMIT:             return {INTERFACE_GET_OK, PID_Steering.getOutputLimit()};
    case VAR_PID3_DEADZONE:                 return {INTERFACE_GET_OK, PID_Steering.getDeadZone()};

    case INTERFACE_ENUM_LAST:               return {INTERFACE_GET_EOF, 0.0f};
    default:                                return {INTERFACE_GET_NOVAL, 0.0f};
    }
}


int LineFollower_s::set(int _Enum, float _Value)
{
    switch(_Enum)
    {
    /*Commands variables                                                                                                                         */
    case CMD_START:                         start();                                                                                        return INTERFACE_SET_OK;
    case CMD_STOP:                          stop();                                                                                         return INTERFACE_SET_OK;
    case CMD_LINE_CALIBRATE:                wakeup(); LineSensor.startCalibration();                                                        return INTERFACE_SET_OK;
    case CMD_IMU_CALIBRATE:                 IMU.startCalibration();                                                                         return INTERFACE_SET_OK;
    case CMD_NVM_ERASE:                     erase();                                                                                        return INTERFACE_SET_OK;
    case CMD_NVM_LOAD:                      load();                                                                                         return INTERFACE_SET_OK;
    case CMD_NVM_SAVE:                      save();                                                                                         return INTERFACE_SET_OK;
    case CMD_LINE_SLEEP:                    sleep();                                                                                        return INTERFACE_SET_OK;
    case CMD_LINE_WAKEUP:                   wakeup();                                                                                       return INTERFACE_SET_OK;
    /*Common variables                                                                                                                           */
    case VAR_SPEED:                         Config.ForwardSpeed = _Value;                                                                   return INTERFACE_SET_OK;
    case VAR_TURN_POS:                      Config.TurnMaxPositive = _Value;                                                                return INTERFACE_SET_OK;
    case VAR_TURN_NEG:                      Config.TurnMaxNegative = _Value;                                                                return INTERFACE_SET_OK;
    case VAR_TURN_BIAS_POS:                 Config.TurnBiasPositive = _Value;                                                               return INTERFACE_SET_OK;
    case VAR_TURN_BIAS_NEG:                 Config.TurnBiasNegative = _Value;                                                               return INTERFACE_SET_OK;
    case VAR_SPEED_DOWN:                    Config.SpeedDown = _Value;                                                                      return INTERFACE_SET_OK;
    /*Driver variables                                                                                                                           */
    case VAR_DRIVE0_PWM_WRAP_VALUE:         DriveA.setPWM_WRAP_VALUE(Config.DriveA.PWM_WRAP_VALUE = _Value);                                return INTERFACE_SET_OK;
    case VAR_DRIVE0_PWM_CLK_DIV:            DriveA.setPWM_CLK_DIV(Config.DriveA.PWM_CLK_DIV = _Value);                                      return INTERFACE_SET_OK;
    case VAR_DRIVE0_BIAS:                   DriveA.setBias(Config.DriveA.Bias = _Value);                                                    return INTERFACE_SET_OK;
    case VAR_DRIVE0_DEADZONE:               DriveA.setDeadZone(Config.DriveA.DeadZone = _Value);                                            return INTERFACE_SET_OK;
    case VAR_DRIVE1_PWM_WRAP_VALUE:         DriveB.setPWM_WRAP_VALUE(Config.DriveB.PWM_WRAP_VALUE = _Value);                                return INTERFACE_SET_OK;
    case VAR_DRIVE1_PWM_CLK_DIV:            DriveB.setPWM_CLK_DIV(Config.DriveB.PWM_CLK_DIV = _Value);                                      return INTERFACE_SET_OK;
    case VAR_DRIVE1_BIAS:                   DriveB.setBias(Config.DriveB.Bias = _Value);                                                    return INTERFACE_SET_OK;
    case VAR_DRIVE1_DEADZONE:               DriveB.setDeadZone(Config.DriveB.DeadZone = _Value);                                            return INTERFACE_SET_OK;
    /*Line variables                                                                                                                             */
    case VAR_LINE_EMITTER_POWER:            LineSensor.setEmittersPower(Config.LineSensor.EmittersPower = _Value);                          return INTERFACE_SET_OK;
    case VAR_LINE_LED_BRIGHTNESS:           LineSensor.setLedsBrightness(Config.LineSensor.LedBrightness = _Value);                         return INTERFACE_SET_OK;
    case VAR_LINE_TURN_GAIN_1:              LineSensor.setTurnGain1(Config.LineSensor.TurnGain1 = _Value);                                  return INTERFACE_SET_OK;
    case VAR_LINE_TURN_GAIN_2:              LineSensor.setTurnGain2(Config.LineSensor.TurnGain2 = _Value);                                  return INTERFACE_SET_OK;
    /*PID0 variables                                                                                                                             */
    case VAR_PID0_SETPOINT:                 PID_DriveA.setSetPoint(Config.PID_DriveA.SetPoint = _Value);                                    return INTERFACE_SET_OK;
    case VAR_PID0_GAIN:                     PID_DriveA.setGain(Config.PID_DriveA.Gain = _Value);                                            return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_TIME:            PID_DriveA.setIntegralTime(Config.PID_DriveA.IntegralTime = _Value);                            return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_LIMIT:           PID_DriveA.setIntegralLimit(Config.PID_DriveA.IntegralLimit = _Value);                          return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_RATE_LIMIT:      PID_DriveA.setIntegralRateLimit(Config.PID_DriveA.IntegralRateLimit = _Value);                  return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_ANTI_WINDUP:     PID_DriveA.setIntegralAntiWindup(Config.PID_DriveA.IntegralAntiWindup = _Value);                return INTERFACE_SET_OK;
    case VAR_PID0_DERIVATIVE_TIME:          PID_DriveA.setDerivativeTime(Config.PID_DriveA.DerivativeTime = _Value);                        return INTERFACE_SET_OK;
    case VAR_PID0_DERIVATIVE_CUTOFF:        PID_DriveA.setDerivativeCutoff(Config.PID_DriveA.DerivativeCutoff = _Value);                    return INTERFACE_SET_OK;
    case VAR_PID0_OUTPUT_LIMIT:             PID_DriveA.setOutputLimit(Config.PID_DriveA.OutputLimit = _Value);                              return INTERFACE_SET_OK;
    case VAR_PID0_DEADZONE:                 PID_DriveA.setDeadZone(Config.PID_DriveA.DeadZone = _Value);                                    return INTERFACE_SET_OK;
    /*PID1 variables                                                                                                                             */
    case VAR_PID1_SETPOINT:                 PID_DriveB.setSetPoint(Config.PID_DriveB.SetPoint = _Value);                                    return INTERFACE_SET_OK;
    case VAR_PID1_GAIN:                     PID_DriveB.setGain(Config.PID_DriveB.Gain = _Value);                                            return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_TIME:            PID_DriveB.setIntegralTime(Config.PID_DriveB.IntegralTime = _Value);                            return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_LIMIT:           PID_DriveB.setIntegralLimit(Config.PID_DriveB.IntegralLimit = _Value);                          return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_RATE_LIMIT:      PID_DriveB.setIntegralRateLimit(Config.PID_DriveB.IntegralRateLimit = _Value);                  return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_ANTI_WINDUP:     PID_DriveB.setIntegralAntiWindup(Config.PID_DriveB.IntegralAntiWindup = _Value);                return INTERFACE_SET_OK;
    case VAR_PID1_DERIVATIVE_TIME:          PID_DriveB.setDerivativeTime(Config.PID_DriveB.DerivativeTime = _Value);                        return INTERFACE_SET_OK;
    case VAR_PID1_DERIVATIVE_CUTOFF:        PID_DriveB.setDerivativeCutoff(Config.PID_DriveB.DerivativeCutoff = _Value);                    return INTERFACE_SET_OK;
    case VAR_PID1_OUTPUT_LIMIT:             PID_DriveB.setOutputLimit(Config.PID_DriveB.OutputLimit = _Value);                              return INTERFACE_SET_OK;
    case VAR_PID1_DEADZONE:                 PID_DriveB.setDeadZone(Config.PID_DriveB.DeadZone = _Value);                                    return INTERFACE_SET_OK;
    /*PID2 variables                                                                                                                             */
    case VAR_PID2_SETPOINT:                 PID_Cruise.setSetPoint(Config.PID_Cruise.SetPoint = _Value);                                    return INTERFACE_SET_OK;
    case VAR_PID2_GAIN:                     PID_Cruise.setGain(Config.PID_Cruise.Gain = _Value);                                            return INTERFACE_SET_OK;
    case VAR_PID2_INTEGRAL_TIME:            PID_Cruise.setIntegralTime(Config.PID_Cruise.IntegralTime = _Value);                            return INTERFACE_SET_OK;
    case VAR_PID2_INTEGRAL_LIMIT:           PID_Cruise.setIntegralLimit(Config.PID_Cruise.IntegralLimit = _Value);                          return INTERFACE_SET_OK;
    case VAR_PID2_INTEGRAL_RATE_LIMIT:      PID_Cruise.setIntegralRateLimit(Config.PID_Cruise.IntegralRateLimit = _Value);                  return INTERFACE_SET_OK;
    case VAR_PID2_INTEGRAL_ANTI_WINDUP:     PID_Cruise.setIntegralAntiWindup(Config.PID_Cruise.IntegralAntiWindup = _Value);                return INTERFACE_SET_OK;
    case VAR_PID2_DERIVATIVE_TIME:          PID_Cruise.setDerivativeTime(Config.PID_Cruise.DerivativeTime = _Value);                        return INTERFACE_SET_OK;
    case VAR_PID2_DERIVATIVE_CUTOFF:        PID_Cruise.setDerivativeCutoff(Config.PID_Cruise.DerivativeCutoff = _Value);                    return INTERFACE_SET_OK;
    case VAR_PID2_OUTPUT_LIMIT:             PID_Cruise.setOutputLimit(Config.PID_Cruise.OutputLimit = _Value);                              return INTERFACE_SET_OK;
    case VAR_PID2_DEADZONE:                 PID_Cruise.setDeadZone(Config.PID_Cruise.DeadZone = _Value);                                    return INTERFACE_SET_OK;
    /*PID3 variables                                                                                                                             */
    case VAR_PID3_SETPOINT:                 PID_Steering.setSetPoint(Config.PID_Steering.SetPoint = _Value);                                return INTERFACE_SET_OK;
    case VAR_PID3_GAIN:                     PID_Steering.setGain(Config.PID_Steering.Gain = _Value);                                        return INTERFACE_SET_OK;
    case VAR_PID3_INTEGRAL_TIME:            PID_Steering.setIntegralTime(Config.PID_Steering.IntegralTime = _Value);                        return INTERFACE_SET_OK;
    case VAR_PID3_INTEGRAL_LIMIT:           PID_Steering.setIntegralLimit(Config.PID_Steering.IntegralLimit = _Value);                      return INTERFACE_SET_OK;
    case VAR_PID3_INTEGRAL_RATE_LIMIT:      PID_Steering.setIntegralRateLimit(Config.PID_Steering.IntegralRateLimit = _Value);              return INTERFACE_SET_OK;
    case VAR_PID3_INTEGRAL_ANTI_WINDUP:     PID_Steering.setIntegralAntiWindup(Config.PID_Steering.IntegralAntiWindup = _Value);            return INTERFACE_SET_OK;
    case VAR_PID3_DERIVATIVE_TIME:          PID_Steering.setDerivativeTime(Config.PID_Steering.DerivativeTime = _Value);                    return INTERFACE_SET_OK;
    case VAR_PID3_DERIVATIVE_CUTOFF:        PID_Steering.setDerivativeCutoff(Config.PID_Steering.DerivativeCutoff = _Value);                return INTERFACE_SET_OK;
    case VAR_PID3_OUTPUT_LIMIT:             PID_Steering.setOutputLimit(Config.PID_Steering.OutputLimit = _Value);                          return INTERFACE_SET_OK;
    case VAR_PID3_DEADZONE:                 PID_Steering.setDeadZone(Config.PID_Steering.DeadZone = _Value);                                return INTERFACE_SET_OK;
    
    default:                                                                                                                                return INTERFACE_SET_NOVAL;
    }
}

void LineFollower_s::run(void)
{
    absolute_time_t TimeNow = get_absolute_time();
    IMU.runAsyncProcess(TimeNow);
    LineSensor.process(TimeNow);
    

    computeControl(TimeNow);
    motorControl(TimeNow);

    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    // {
    //     //printf("M1 %d, % .3f rpm, M2 %d, % .3f rpm, % .9f\n", Steps1, RPM1, Steps2, RPM2, PID_DriveB.getIntg());
    //     //printf("%f, %f, %f, %f, %d, %d\n", Speed0, TargetSpeed0, Speed1, TargetSpeed1, Encoder1_err, Encoder2_err);
    //     printf("E:%f I:%f D:%f, SA:%f, G:%f, O:%f\n", PID_DriveA.getError(), PID_DriveA.getIntegral(), PID_DriveA.getDerivative(), EncoderSpeedA, PID_DriveA.getGain(), PID_DriveA.getOutput());
    //     PrintTime = make_timeout_time_ms(200);
    // }
    
    
    Interface.Process();
    LineSensor.updateLeds(TimeNow);
    LED0.process(TimeNow);
}

void LineFollower_s::computeControl(absolute_time_t _TimeNow)
{

    auto [Roll, Pitch, Yaw] = IMU.getOrientation();
    auto LineHeading = LineSensor.computeLineHeading(Yaw);
    
    //auto [Steps1, Steps2] = getSteps();
    
    // if (to_us_since_boot(_TimeNow) > to_us_since_boot(PrintTime))
    // {
    //     printf("YAW:% .9f,LINE:% .9f\n", Yaw, LineHeading);
        
    //     PrintTime = make_timeout_time_ms(100);
    // }
    
    
    // if (!Start || Stop)
    // {
    //     if (to_us_since_boot(_TimeNow) > to_us_since_boot(SleepTime) && Awake)
    //     {
    //         sleep();
    //     }
    //     return;
    // }
    

    // if (Roll > 0.5f || Roll < -0.5f || Pitch > 0.5f || Pitch < -0.5f)
    // {
    //     stop_error();
    //     return;
    // }

    // if (LineSensor.isTimedOut())
    // {
    //     stop_error();
    //     return;
    // }
    float BaseSpeed, SteeringOffset, OffsetA, OffsetB, OffsetMaxPos, OffsetMaxNeg, OverdriveA, OverdriveB;

    SteeringOffset = PID_Steering.compute(LineHeading, _TimeNow);
    BaseSpeed = Config.ForwardSpeed * (1.0f - std::abs(PID_Cruise.compute(LineHeading, _TimeNow)));

    OffsetMaxPos = Config.TurnMaxPositive - BaseSpeed;
    OffsetMaxNeg = -Config.TurnMaxNegative - BaseSpeed;

    OffsetA = SteeringOffset;
    OffsetB = -SteeringOffset;
    OverdriveA = 0.0f;
    OverdriveB = 0.0f;

    if (SteeringOffset > 0.0f)
    {
        OffsetA *= (1.0f - Config.TurnBiasPositive);
        OffsetB *= (1.0f - Config.TurnBiasNegative);
        OverdriveA = std::min(OffsetMaxPos - OffsetA, 0.0f);
        OverdriveB = std::max(OffsetMaxNeg - OffsetB, 0.0f);
    }
    else if (SteeringOffset < 0.0f)
    {
        OffsetA *= (1.0f - Config.TurnBiasNegative);
        OffsetB *= (1.0f - Config.TurnBiasPositive);
        OverdriveA = std::max(OffsetMaxNeg - OffsetA, 0.0f);
        OverdriveB = std::min(OffsetMaxPos - OffsetB, 0.0f);
    }

    if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    {
        //printf("M1 %d, % .3f rpm, M2 %d, % .3f rpm, % .9f\n", Steps1, RPM1, Steps2, RPM2, PID_DriveB.getIntg());
        //printf("%f, %f, %f, %f, %d, %d\n", Speed0, TargetSpeed0, Speed1, TargetSpeed1, Encoder1_err, Encoder2_err);
        printf("B:%f A:%f BS:%f, SO:%f, B:%f, A:%f, B:%f, A:%f, M:%f, N:%f\n", EncoderSpeedB, EncoderSpeedA, BaseSpeed, SteeringOffset, OffsetB, OffsetA, OverdriveB, OverdriveA, OffsetMaxPos, OffsetMaxNeg);
        PrintTime = make_timeout_time_ms(200);
    }

    PID_DriveA.setSetPoint(-(BaseSpeed + OffsetA + OverdriveB));//invert rigth motor
    PID_DriveB.setSetPoint(BaseSpeed + OffsetB + OverdriveA);
}

void LineFollower_s::motorControl(absolute_time_t _TimeNow)
{
    auto [NewData, Ok] = getEncoderData();
    if (Ok)
    {
        float DutyA, DutyB;
        
        //Calculate motors speeds, -1.0f to 1.0f relative to MOTOR_NOMINAL_RPM, valuse below MOTOR_MIN_RPM are set to 0.0f, above MOTOR_MAX_RPM to 1.0f
        int PPA = std::min(NewData.PulsePeriodSumA, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);
        int PPB = std::min(NewData.PulsePeriodSumB, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);

        //constexpr int p = (int)ENCODER_MAX_PULSE_PERIOD_US;
        //constexpr int ptst = std::max(p, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);
        //constexpr float tst = ((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * ptst) / (ptst * ENCODER_CONVERSION_SPAN);

        EncoderSpeedA = ((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * PPA) / (PPA * ENCODER_CONVERSION_SPAN);
        EncoderSpeedB = ((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * PPB) / (PPB * ENCODER_CONVERSION_SPAN);

        EncoderSpeedA *= NewData.PulseDirectionA;
        EncoderSpeedB *= NewData.PulseDirectionB;
        
        EncoderPulseCountA = NewData.PulseCountA;
        EncoderPulseCountB = NewData.PulseCountB;

        DutyA = PID_DriveA.compute(EncoderSpeedA, _TimeNow);
        DutyB = PID_DriveB.compute(EncoderSpeedB, _TimeNow);
        DriveA.setDuty(DutyA);
        DriveB.setDuty(DutyB);
    }

    DriveA.process(_TimeNow);
    DriveB.process(_TimeNow);
}











