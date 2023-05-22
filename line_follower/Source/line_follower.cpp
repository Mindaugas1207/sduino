#include "hardware/adc.h"
#include "line_follower.hpp"


NVM_s NVM;
port_spi_t spi_port;
port_i2c_t i2c_port;
Interface_s Interface;
line_sesnor_hw_inst_t LineSensorHW;
LineSensor_s LineSensor;
MotorDriver_s DriveA;
MotorDriver_s DriveB;
PID_s PID_DriveA;
PID_s PID_DriveB;
IMU_s IMU;
ESC_s ESC;

Blink_s LED0;

void sduino_adc_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
}

#define DRV_R 473.5f
constexpr auto MOTOR_B_ENCODER_A_PIN = 1U;
constexpr auto MOTOR_B_ENCODER_B_PIN = 3U;
constexpr auto MOTOR_A_ENCODER_A_PIN = 24U;
constexpr auto MOTOR_A_ENCODER_B_PIN = 25U;
constexpr auto MOTOR_B_ENCODER_PINS_MASK = 1U << MOTOR_B_ENCODER_A_PIN | 1U << MOTOR_B_ENCODER_B_PIN;
constexpr auto MOTOR_A_ENCODER_PINS_MASK = 1U << MOTOR_A_ENCODER_A_PIN | 1U << MOTOR_A_ENCODER_B_PIN;
constexpr auto MOTOR_MIN_RPM = 260;
//constexpr auto MOTOR_NOMINAL_RPM = 2800;
//constexpr auto MOTOR_MAX_RPM = 2800;
#ifdef MOTORDEF0
constexpr auto MOTOR_MAX_RPM = 2800; //2800, 4300
constexpr auto ENCODER_REDUCTION = 5.4;//1:5.4, 4.4
constexpr auto ENCODER_PPR_HAL = 11;//11, 1? 13?
constexpr auto ENCODER_WHEEL_DIAMETER_MM = 25;
constexpr auto ENCODER_WHEEL_DIAMETER = 0.026;
#endif
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
constexpr auto ENCODER_UPDATE_PERIOD_US = 1000;
constexpr auto ENCODER_UPDATE_RETRY_PERIOD_US = 30;
constexpr auto ENCODER_SAMPLE_BUFFER_SIZE = 32;
constexpr auto ENCODER_CONVERSION_BOT = (float)(1.0 / (int)ENCODER_MAX_PULSE_PERIOD_US);
constexpr auto ENCODER_CONVERSION_TOP = (float)(1.0 / (int)ENCODER_MIN_PULSE_PERIOD_US);
constexpr auto ENCODER_CONVERSION_SPAN = ENCODER_CONVERSION_TOP - ENCODER_CONVERSION_BOT;
constexpr auto ENCODER_WHEEL_LENGTH_MM = ENCODER_WHEEL_DIAMETER_MM * M_PI;
constexpr auto ENCODER_WHEEL_LENGTH = ENCODER_WHEEL_DIAMETER * M_PI;
constexpr auto ENCODER_PULSE_TO_MM = (float)(ENCODER_WHEEL_LENGTH_MM / ENCODER_PPR);
constexpr auto ENCODER_BASE_LENGTH_MM = 184;

constexpr auto ENCODER_PULSE_TO_LENGTH = ENCODER_WHEEL_LENGTH / ENCODER_PPR;
constexpr auto ENCODER_BASE_LENGTH = 0.184;

struct EncoderData_s {
    int PulseCountA, PulseCountB, PulsePeriodSumA, PulsePeriodSumB, PulseDirectionA, PulseDirectionB;
};

float BatteryVoltage = 0.0f;

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

void sduino_adc_read()
{
    //uint16_t Buffer[5];
    //float Volts[5];
    /* 12-bit conversion, assume max value == ADC_VREF == 3.08 V */
    const float conversionFactor = 3.035f / 4095.0f;

    //for (uint i = 0; i < 5; i++)
    //{
        adc_select_input(2);
        //Buffer[i] = adc_read();
        //Volts[i] = (float)Buffer[i] * conversionFactor;
    //}
    BatteryVoltage = adc_read() * conversionFactor * 7.2397;


    //float temperature = 27.0f - (Volts[4] - 0.706f) / 0.001721f;
    //printf("AIN0 = %.9f, AIN1 = %.9f, AIN2 = %.9f, AIN3 = %.9f, AIN4 = %.9f, TC = %.02f C\n", Volts[0], Volts[1], Volts[2]*7.2397, Volts[3], Volts[4], temperature);
}

void ESC_Arm()
{

}

void LineFollower_s::ESC_Start()
{
    if (Start_ESC) return;
    Start_ESC = true;
    Stop_ESC = false;
    ESC.start(0.2f, 2000);
}

void LineFollower_s::ESC_Stop()
{
    Start_ESC = false;
    Stop_ESC = true;
    ESC.stop();
}

void DriversStop() {
    DriveA.stop();
    DriveB.stop();
    PID_DriveA.reset();
    PID_DriveB.reset();
}

void DriversBrake() {
    DriveA.brake();
    DriveB.brake();
    PID_DriveA.reset();
    PID_DriveB.reset();
}

void DriversStart() {
    DriveA.start();
    DriveB.start();
}

void LineFollower_s::sleep(void)
{
    if (!LineSensor.isCalibrated() || !IMU.isCalibrated()) return;
    DriversStop();
    LineSensor.stop();
    Awake = false;
    LED0.stop();
    printf("DBG:SLEEP\n");
}

void LineFollower_s::wakeup(void)
{
    LineSensor.start();
    IMU.reset();
    Awake = true;
    LED0.start(500, 500);
    SleepTime = make_timeout_time_ms(10000);
    printf("DBG:WAKEUP\n");
}

void LineFollower_s::start(void)
{
    if (Start) return;
    wakeup();
    Start = true;
    Stop = false;
    LED0.start(50, 50);
    PID_DriveA.reset();
    PID_DriveB.reset();
    DriversStart();
    printf("DBG:START\n");
}

void LineFollower_s::stop(void)
{
    DriversBrake();
    ESC_Stop();
    Start = false;
    Stop = true;
    LED0.start(1000, 1000);
    PID_DriveA.reset();
    PID_DriveB.reset();
    // MotorA.stopBeep();
    // MotorB.stopBeep();
    SleepTime = make_timeout_time_ms(10000);
    SleepAllowed = true;
    printf("DBG:STOP\n");
}

void LineFollower_s::stop_error(void)
{
    DriversBrake();
    ESC_Stop();
    Start = false;
    Stop = true;
    sleep_ms(1000);
    DriversStop();
    LED0.start(100, 100);
    PID_DriveA.reset();
    PID_DriveB.reset();
    // MotorA.startBeep(500, 500);
    // MotorB.startBeep(500, 500);
    printf("DBG:STOP_ERROR\n");
}

void LineFollower_s::init(void)
{
    LED0.init(SDUINO_INTERNAL_LED_PIN);
    stdio_init_all();
   
    //while (!stdio_usb_connected()) { sleep_ms(500); }

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

    encodersInit();

    Interface.Init(f_getCallback, f_setCallback);
    NVM.init(FLASH_SECTOR_SIZE, &Config, sizeof(Config));

    DriveA.init(SDUINO_INTERNAL_DRV_A_IN1_PIN, SDUINO_INTERNAL_DRV_A_IN2_PIN);
    DriveB.init(SDUINO_INTERNAL_DRV_B_IN1_PIN, SDUINO_INTERNAL_DRV_B_IN2_PIN);
    ESC.init(23);
    sduino_adc_init();

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
    stop();
    wakeup();
    //sleep();
}

void LineFollower_s::save(void)
{
    Config.IMU = IMU.getConfiguration();
    Config.LineSensor = LineSensor.getConfiguration();
    Config.DriveA = DriveA.getConfiguration();
    Config.DriveB = DriveB.getConfiguration();
    Config.PID_Drives = PID_DriveA.getConfiguration();

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
        PID_DriveA.loadConfiguration(Config.PID_Drives);
        PID_DriveB.loadConfiguration(Config.PID_Drives);
        ESC.setSpeed(Config.EscSpeed);
        NVM_CONFIG_OK = true;
        NVM_LOAD_OK = true;
    }
    else
    {
        //NVM empty or corrupted
        //Load Default Configurations

        PID_s::Config_t DrivePID = {
            .SetPoint = 0.0f,
            .Gain = 1.9f,
            .IntegralTime = 0.25f,
            .IntegralLimit = 300.0f,
            .IntegralRateLimit = 4.0f,
            .IntegralAntiWindup = 0.0f,
            .DerivativeTime = 0.001f,
            .DerivativeCutoff = 0.9f,
            .OutputLimit = 1.0f,
            .DeadZone = 0.026f
        };

        PID_DriveA.loadConfiguration(DrivePID);
        PID_DriveB.loadConfiguration(DrivePID);
        Config.PID_Drives = PID_DriveA.getConfiguration();

        Config.DriveA = DriveA.getConfiguration();
        Config.DriveB = DriveB.getConfiguration();

        Config.DriveA.Bias = 0.1f;
        Config.DriveA.DeadZone = 0.024f;
        Config.DriveA.PWM_CLK_DIV = 1.0f;
        Config.DriveB.Bias = 0.1f;
        Config.DriveB.DeadZone = 0.032f;
        Config.DriveB.PWM_CLK_DIV = 1.0f;

        DriveA.loadConfiguration(Config.DriveA);
        DriveB.loadConfiguration(Config.DriveB);
        Config.DriveA = DriveA.getConfiguration();
        Config.DriveB = DriveB.getConfiguration();

        LineSensor_s::Config_t LS_Config = {
            .EmittersPower = 1.0f,
            .LedBrightness = 0.5f,
            .TurnGain1 = 1.0f,
            .TurnGain2 = 1.0f,
            .OffsetDecayCoef = LINE_SENSOR_OFFSET_DECAY_COEF,
            .AverageDecayCoef = LINE_SENSOR_POS_AVG_DECAY_COEF,
            .AnalogWidth = LINE_SENSOR_ANALOG_WIDTH,
            .OffsetThreshold = LINE_SENSOR_OFFSET_THRESHOLD,
            .LeftThreshold  = LINE_SENSOR_CENTER_INDEX + (LINE_SENSOR_ANALOG_WIDTH / 2),
            .RightThreshold = LINE_SENSOR_CENTER_INDEX - (LINE_SENSOR_ANALOG_WIDTH / 2),
            .CenterTimeout = LINE_SENSOR_CENTERLINE_TIMEOUT_US,
            .Calibrated = false,
        };

        LineSensor.loadConfiguration(LS_Config);
        Config.LineSensor = LineSensor.getConfiguration();
        
        Config.StateFlags = 0;
        Config.ForwardSpeed = 0.5f;
        Config.EscSpeed = 0.5f;
        Config.SteeringGain = 0.0f;
        Config.SteeringGain2 = 0.0f;
        Config.SteeringGain3 = 0.0f;
        Config.SteeringGain4 = 0.0f;
        Config.SteeringGain5 = 0.0f;
        Config.SteeringGain6 = 0.0f;
        Config.CruiseGain = 0.0f;
        Config.CruiseGain2 = 0.0f;
        Config.CruiseGain3 = 0.0f;
        Config.CruiseGain4 = 0.0f;
        Config.CruiseGain5 = 0.0f;
        Config.CruiseGain6 = 0.0f;
        Config.BrakeTimeout = 0;
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
    case CMD_ESC_START:                     return {INTERFACE_GET_OK, Start_ESC && !Stop_ESC};
    case CMD_ESC_STOP:                      return {INTERFACE_GET_OK, !Start_ESC && Stop_ESC};
    case VAR_BAT_VOLTAGE:                   return {INTERFACE_GET_OK, BatteryVoltage};
    /*Common variables                      */
    case VAR_SPEED:                         return {INTERFACE_GET_OK, Config.ForwardSpeed};
    case VAR_ESC_SPEED:                     return {INTERFACE_GET_OK, Config.EscSpeed};
    case VAR_STEERING_GAIN:                 return {INTERFACE_GET_OK, Config.SteeringGain};
    case VAR_STEERING_GAIN2:                return {INTERFACE_GET_OK, Config.SteeringGain2};
    case VAR_STEERING_GAIN3:                return {INTERFACE_GET_OK, Config.SteeringGain3};
    case VAR_STEERING_GAIN4:                return {INTERFACE_GET_OK, Config.SteeringGain4};
    case VAR_STEERING_GAIN5:                return {INTERFACE_GET_OK, Config.SteeringGain5};
    case VAR_STEERING_GAIN6:                return {INTERFACE_GET_OK, Config.SteeringGain6};
    case VAR_CRUISE_GAIN:                   return {INTERFACE_GET_OK, Config.CruiseGain};
    case VAR_CRUISE_GAIN2:                  return {INTERFACE_GET_OK, Config.CruiseGain2};
    case VAR_CRUISE_GAIN3:                  return {INTERFACE_GET_OK, Config.CruiseGain3};
    case VAR_CRUISE_GAIN4:                  return {INTERFACE_GET_OK, Config.CruiseGain4};
    case VAR_CRUISE_GAIN5:                  return {INTERFACE_GET_OK, Config.CruiseGain5};
    case VAR_CRUISE_GAIN6:                  return {INTERFACE_GET_OK, Config.CruiseGain6};
    case VAR_BRAKE_TIMEOUT:                 return {INTERFACE_GET_OK, Config.BrakeTimeout};
    /*Driver variables                      */
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
    case CMD_LINE_TOGGLE_DISPLAY:           return {INTERFACE_GET_OK, LineSensor.getDisplayAnalog()};
    case CMD_LINE_TOGGLE_LIVE:              return {INTERFACE_GET_OK, LineSensor.getLiveUpdate()};
    case VAR_LINE_OFFSET_DECAY:             return {INTERFACE_GET_OK, LineSensor.getOffsetDecayCoef()};
    case VAR_LINE_AVERAGE_DECAY:            return {INTERFACE_GET_OK, LineSensor.getAverageDecayCoef()};
    case VAR_LINE_ANALOG_WIDTH:             return {INTERFACE_GET_OK, LineSensor.getAnalogWidth()};
    case VAR_LINE_OFFSET_THRESHOLD:         return {INTERFACE_GET_OK, LineSensor.getOffsetThreshold()};
    case VAR_LINE_LEFT_THRESHOLD:           return {INTERFACE_GET_OK, LineSensor.getLeftThreshold()};
    case VAR_LINE_RIGHT_THRESHOLD:          return {INTERFACE_GET_OK, LineSensor.getRightThreshold()};
    case VAR_LINE_CENTER_TIMEOUT:           return {INTERFACE_GET_OK, LineSensor.getCenterTimeout()};
    /*PID0 variables                        */
    case VAR_PID0_GAIN:                     return {INTERFACE_GET_OK, PID_DriveA.getGain()};
    case VAR_PID0_INTEGRAL_TIME:            return {INTERFACE_GET_OK, PID_DriveA.getIntegralTime()};
    case VAR_PID0_INTEGRAL_LIMIT:           return {INTERFACE_GET_OK, PID_DriveA.getIntegralLimit()};
    case VAR_PID0_INTEGRAL_RATE_LIMIT:      return {INTERFACE_GET_OK, PID_DriveA.getIntegralRateLimit()};
    case VAR_PID0_DERIVATIVE_TIME:          return {INTERFACE_GET_OK, PID_DriveA.getDerivativeTime()};
    case VAR_PID0_DERIVATIVE_CUTOFF:        return {INTERFACE_GET_OK, PID_DriveA.getDerivativeCutoff()};
    case VAR_PID0_DEADZONE:                 return {INTERFACE_GET_OK, PID_DriveA.getDeadZone()};

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
    case CMD_LINE_CALIBRATE:                wakeup(); LineSensor.startCalibration(); LED0.start(100,400);                                   return INTERFACE_SET_OK;
    case CMD_IMU_CALIBRATE:                 wakeup(); IMU.startCalibration(); LED0.start(100,400);                                          return INTERFACE_SET_OK;
    case CMD_NVM_ERASE:                     erase();                                                                                        return INTERFACE_SET_OK;
    case CMD_NVM_LOAD:                      load();                                                                                         return INTERFACE_SET_OK;
    case CMD_NVM_SAVE:                      save();                                                                                         return INTERFACE_SET_OK;
    case CMD_LINE_SLEEP:                    sleep(); SleepAllowed = true;                                                                   return INTERFACE_SET_OK;
    case CMD_LINE_WAKEUP:                   wakeup(); SleepAllowed = false;                                                                 return INTERFACE_SET_OK;
    case CMD_ESC_ARM:                       ESC_Arm();                                                                                      return INTERFACE_SET_OK;
    case CMD_ESC_START:                     ESC_Start();                                                                                    return INTERFACE_SET_OK;
    case CMD_ESC_STOP:                      ESC_Stop();                                                                                     return INTERFACE_SET_OK;
    /*Common variables                                                                                                                           */
    case VAR_SPEED:                         Config.ForwardSpeed = _Value;                                                                   return INTERFACE_SET_OK;
    case VAR_ESC_SPEED:                     ESC.setSpeed(Config.EscSpeed = _Value);                                                         return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN:                 Config.SteeringGain = _Value;                                                                   return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN2:                Config.SteeringGain2 = _Value;                                                                  return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN3:                Config.SteeringGain3 = _Value;                                                                  return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN4:                Config.SteeringGain4 = _Value;                                                                  return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN5:                Config.SteeringGain5 = _Value;                                                                  return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN6:                Config.SteeringGain6 = _Value;                                                                  return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN:                   Config.CruiseGain = _Value;                                                                     return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN2:                  Config.CruiseGain2 = _Value;                                                                    return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN3:                  Config.CruiseGain3 = _Value;                                                                    return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN4:                  Config.CruiseGain4 = _Value;                                                                    return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN5:                  Config.CruiseGain5 = _Value;                                                                    return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN6:                  Config.CruiseGain6 = _Value;                                                                    return INTERFACE_SET_OK;
    case VAR_BRAKE_TIMEOUT:                 Config.BrakeTimeout = _Value;                                                                   return INTERFACE_SET_OK;
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
    case CMD_LINE_TOGGLE_DISPLAY:           LineSensor.setDisplayAnalog(!LineSensor.getDisplayAnalog());                                    return INTERFACE_SET_OK;
    case CMD_LINE_TOGGLE_LIVE:              LineSensor.setLiveUpdate(!LineSensor.getLiveUpdate());                                          return INTERFACE_SET_OK;
    case VAR_LINE_OFFSET_DECAY:             LineSensor.setOffsetDecayCoef(Config.LineSensor.OffsetDecayCoef = _Value);                      return INTERFACE_SET_OK;
    case VAR_LINE_AVERAGE_DECAY:            LineSensor.setAverageDecayCoef(Config.LineSensor.AverageDecayCoef = _Value);                    return INTERFACE_SET_OK;
    case VAR_LINE_ANALOG_WIDTH:             LineSensor.setAnalogWidth(Config.LineSensor.AnalogWidth = _Value);                              return INTERFACE_SET_OK;
    case VAR_LINE_OFFSET_THRESHOLD:         LineSensor.setOffsetThreshold(Config.LineSensor.OffsetThreshold = _Value);                      return INTERFACE_SET_OK;
    case VAR_LINE_LEFT_THRESHOLD:           LineSensor.setLeftThreshold(Config.LineSensor.LeftThreshold = _Value);                          return INTERFACE_SET_OK;
    case VAR_LINE_RIGHT_THRESHOLD:          LineSensor.setRightThreshold(Config.LineSensor.RightThreshold = _Value);                        return INTERFACE_SET_OK;
    case VAR_LINE_CENTER_TIMEOUT:           LineSensor.setCenterTimeout(Config.LineSensor.CenterTimeout = _Value);                          return INTERFACE_SET_OK;
    /*PID0 variables                                                                                                                             */
    case VAR_PID0_GAIN:                     PID_DriveA.setGain(Config.PID_Drives.Gain = _Value);
                                            PID_DriveB.setGain(_Value);                                                                     return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_TIME:            PID_DriveA.setIntegralTime(Config.PID_Drives.IntegralTime = _Value);
                                            PID_DriveB.setIntegralTime(_Value);                                                             return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_LIMIT:           PID_DriveA.setIntegralLimit(Config.PID_Drives.IntegralLimit = _Value);
                                            PID_DriveB.setIntegralLimit(_Value);                                                            return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_RATE_LIMIT:      PID_DriveA.setIntegralRateLimit(Config.PID_Drives.IntegralRateLimit = _Value);
                                            PID_DriveB.setIntegralRateLimit(_Value);                                                        return INTERFACE_SET_OK;
    case VAR_PID0_DERIVATIVE_TIME:          PID_DriveA.setDerivativeTime(Config.PID_Drives.DerivativeTime = _Value);
                                            PID_DriveB.setDerivativeTime(_Value);                                                           return INTERFACE_SET_OK;
    case VAR_PID0_DERIVATIVE_CUTOFF:        PID_DriveA.setDerivativeCutoff(Config.PID_Drives.DerivativeCutoff = _Value);
                                            PID_DriveB.setDerivativeCutoff(_Value);                                                         return INTERFACE_SET_OK;
    case VAR_PID0_DEADZONE:                 PID_DriveA.setDeadZone(Config.PID_Drives.DeadZone = _Value);
                                            PID_DriveB.setDeadZone(_Value);                                                                 return INTERFACE_SET_OK;
    default:                                                                                                                                return INTERFACE_SET_NOVAL;
    }
}

void LineFollower_s::run(void)
{
    absolute_time_t TimeNow = get_absolute_time();
    sduino_adc_read();
    IMU.runAsyncProcess(TimeNow);
    LineSensor.process(TimeNow);

    computeControl(TimeNow);

    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    // {
    //     //printf("M1 %d, % .3f rpm, M2 %d, % .3f rpm, % .9f\n", Steps1, RPM1, Steps2, RPM2, PID_DriveB.getIntg());
    //     //printf("%f, %f, %f, %f, %d, %d\n", Speed0, TargetSpeed0, Speed1, TargetSpeed1, Encoder1_err, Encoder2_err);
    //     printf("E:%f I:%f D:%f, SA:%f, G:%f, O:%f\n", PID_DriveA.getError(), PID_DriveA.getIntegral(), PID_DriveA.getDerivative(), EncoderSpeedA, PID_DriveA.getGain(), PID_DriveA.getOutput());
    //     PrintTime = make_timeout_time_ms(200);
    // }
    DriveA.process(TimeNow);
    DriveB.process(TimeNow);

    ESC.process(TimeNow);
    
    Interface.Process();
    LineSensor.updateLeds(TimeNow);
    LED0.process(TimeNow);
}

float _unwrap(float previous_angle, float new_angle) {
    float d = new_angle - previous_angle;
    d = d > M_PI ? d - (float)(2 * M_PI) : (d < -M_PI ? d + (float)(2 * M_PI) : d);
    return previous_angle + d;
}

float constrainAngle(float x){
    x = std::fmod(x + (float)M_PI, (float)(2 * M_PI));
    if (x < 0)
        x += (float)(2 * M_PI);
    return x - (float)M_PI;
}

std::tuple<float, float, float> XYoffsetEncoders(auto dR, auto dL, float A0)
{
    auto A = dR - dL;
    auto E = (float)(ENCODER_BASE_LENGTH / 2.0) * (dR + dL);
    auto B = E / A;
    float AX, AY, C;
    if (std::isinf(B) || std::isnan(B))
    {
        B = (float)(ENCODER_PULSE_TO_LENGTH / 2.0) * (dR + dL);
        C = 0.0f;
        AX = std::cos(A0);
        AY = std::sin(A0);
    }
    else
    {
        C = (float)(ENCODER_PULSE_TO_LENGTH / ENCODER_BASE_LENGTH) * A;
        AX =   std::sin(C + A0) - std::sin(A0);
        AY = -(std::cos(C + A0) - std::cos(A0));
    }
    
    return {B * AX, B * AY, C};
}

void LineFollower_s::computeControl(absolute_time_t _TimeNow)
{
    auto [ENC_Data, ENC_OK] = getEncoderData();
    auto [Roll, Pitch, Yaw] = IMU.getOrientation();

    float SpeedR, SpeedL;

    if (ENC_OK)
    {
        //Calculate motors speeds, -1.0f to 1.0f relative to MOTOR_NOMINAL_RPM, valuse below MOTOR_MIN_RPM are set to 0.0f, above MOTOR_MAX_RPM to 1.0f
        int PPA = std::min(ENC_Data.PulsePeriodSumA, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);
        int PPB = std::min(ENC_Data.PulsePeriodSumB, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);
        SpeedR = (((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * PPA) / (PPA * ENCODER_CONVERSION_SPAN)) * ENC_Data.PulseDirectionA;
        SpeedL = (((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * PPB) / (PPB * ENCODER_CONVERSION_SPAN)) * ENC_Data.PulseDirectionB;  
        iSpeedR = SpeedR;
        iSpeedL = SpeedL;

        int PulseR = -ENC_Data.PulseCountA; //invert right motor fedback
        int PulseL = ENC_Data.PulseCountB;

        auto [dX, dY, dYaw] = XYoffsetEncoders(PulseR - iPulseR, PulseL - iPulseL, iYaw);
        iPulseR = PulseR;
        iPulseL = PulseL;

        iX_f += dX;
        iY_f += dY;
        iYaw = constrainAngle(iYaw + dYaw);

        // auto flrX = std::floor(iX_f);
        // auto flrY = std::floor(iY_f);

        // if (flrX != 0.0f)
        // {
        //     iX += flrX;
        //     iX_f -= flrX;
        // }

        // if (flrY != 0.0f)
        // {
        //     iY += flrY;
        //     iY_f -= flrY;
        // }
    }
    else
    {
        SpeedR = iSpeedR;
        SpeedL = iSpeedL;
    }

    auto LineHeading = LineSensor.computeLineHeading(Yaw);
    auto [Range, Turn] = LineSensor.getLineDesc();
    auto Detected = LineSensor.isDetected();

    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    // {
    //     //printf("LH: %f \n", LineHeading);
    //     //printf("A: %f, B: %f\n", DriveA.getDuty(), DriveB.getDuty());
    //     //printf("M1 %d, % .3f rpm, M2 %d, % .3f rpm, % .9f\n", Steps1, RPM1, Steps2, RPM2, PID_DriveB.getIntg());
    //     //printf("%f, %f, %f, %f, %d, %d\n", Speed0, TargetSpeed0, Speed1, TargetSpeed1, Encoder1_err, Encoder2_err);
    //     // char str1[15];
    //     // char str2[15];
    //     char buffer[128];
    //     //sprintf(str1, "%.9f", iX_f);
    //     //sprintf(str2, "%.9f", iY_f);
    //     //sprintf(buffer, "CMD:WS(\"X\":%d.%s,\"Y\":%d.%s,\"O\":%.9f,\"L\":%.9f,\"I\":%.9f)\n", iX, str1 + 2, iY, str2 + 2, iYaw, LineHeading, Yaw);
    //     sprintf(buffer, "CMD:WS(\"X\":%.12f,\"Y\":%.12f,\"O\":%.9f,\"L\":%.9f,\"I\":%.9f)\n", iX_f, iY_f, iYaw, LineHeading, Yaw);
    //     uart_puts(uart_internal, buffer);
    //     //printf("R: %d, L: %d, X: %d %f Y: %d %f O: %.9f L: %.9f Yaw: %.9f\n", iPulseR, iPulseL, iX, iX_f, iY, iY_f, iYaw, LineHeading, Yaw);
    //     //printf("sB:%f sA:%f BS:%f, SO:%f, oB:%f, oA:%f, vB:%f, vA:%f, M+:%f, M-:%f, iB:%f, iA:%f, pB:%f, pA:%f\n", SpeedA, SpeedB, BaseSpeed, SteeringOffset, OffsetB, OffsetA, OverdriveB, OverdriveA, OffsetMaxPos, OffsetMaxNeg, PID_DriveB.getIntegral(), PID_DriveA.getIntegral(), DriveB.getDuty(), DriveA.getDuty());
    //     PrintTime = make_timeout_time_ms(500);
    // }
    
    //EncoderPulseCountA = ENC_Data.PulseCountA;
    //EncoderPulseCountB = ENC_Data.PulseCountB;
    //auto [Steps1, Steps2] = getSteps();
    
    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    // {
    //     printf("Detected: %s, Heading: %.6f, Range: %s, Turn: %s\n",
    //         Detected ? "True " : "False",
    //         LineHeading, 
    //         Range == LineSensor_s::LineRange_t::LINE_CENTER ? "CENTER" :
    //         Range == LineSensor_s::LineRange_t::LINE_LEFT ? "LEFT  " : "RIGHT ",
    //         Turn == LineSensor_s::LineTurn_t::LINE_NO_TURN ? "NONE  " :
    //         Turn == LineSensor_s::LineTurn_t::LINE_TURN_LEFT ? "LEFT  " : "RIGHT "
    //     );
    //     PrintTime = make_timeout_time_ms(200);
    // }

    float SPA, SPB;
    float BaseSpeed, SteeringOffset, SteeringGain, CruiseGain;

    bool LineTracking = (Detected && !ReturningOnLine) || (!Detected && Turn != LineSensor_s::LineTurn_t::LINE_NO_TURN);

    if (LineTracking)
    {
        BrakeStart = false;
        BrakeEnd = false;
        SteeringGain = Config.SteeringGain;
        CruiseGain = Config.CruiseGain;
        SteeringOffset = -std::clamp(LineHeading * SteeringGain, -2.0f, 2.0f);
        BaseSpeed = std::abs(LineHeading) < LINE_SENSOR_STEP_VALUE ? Config.ForwardSpeed : Config.ForwardSpeed - std::min(std::abs(LineHeading) * CruiseGain, Config.ForwardSpeed);

        SPA = std::clamp(SteeringOffset >= 0.0f ? BaseSpeed : BaseSpeed + SteeringOffset, -1.0f, 1.0f);
        SPB = std::clamp(SteeringOffset <= 0.0f ? BaseSpeed : BaseSpeed - SteeringOffset, -1.0f, 1.0f);
    }
    else if (ReturningOnLine)
    {
        if (Range == LineSensor_s::LineRange_t::LINE_CENTER)
        {
            ReturningOnLine = false;
        }
    }
    else
    {
        ReturningOnLine = true;
        if (Config.BrakeTimeout != 0)
        {
            if (!BrakeStart)
            {
                BrakeStart = true;
                BrakeTime = make_timeout_time_us(Config.BrakeTimeout);
            }
        }

        SteeringGain = Config.SteeringGain2;
        CruiseGain = Config.CruiseGain2;
        SteeringOffset = -std::clamp(LineHeading * SteeringGain, -2.0f, 2.0f);
        BaseSpeed = -std::clamp(LineHeading * CruiseGain, -2.0f, 2.0f);

        if (LineHeading > 0)
        {
            SPA = std::clamp(Config.ForwardSpeed + SteeringOffset, -1.0f, Config.CruiseGain6);
            SPB = std::clamp(Config.ForwardSpeed - BaseSpeed, -1.0f, Config.CruiseGain5);
        }
        else
        {
            SPA = std::clamp(Config.ForwardSpeed + BaseSpeed, -1.0f, Config.CruiseGain5);
            SPB = std::clamp(Config.ForwardSpeed - SteeringOffset, -1.0f, Config.CruiseGain6);
        }
    }

    if (!Start || Stop)
    {
        if (to_us_since_boot(_TimeNow) > to_us_since_boot(SleepTime) && Awake && SleepAllowed)
        {
            sleep();
        }
        return;
    }
    else if (Start && !Stop)
    {
        if (Roll > 0.6f || Roll < -0.6f || Pitch > 0.6f || Pitch < -0.6f)
        {
            if (!Stop)
                stop_error();
            return;
        }

        if (LineSensor.isTimedOut())
        {
            if (!Stop)
                stop_error();
            return;
        }

        if (BrakeStart && !BrakeEnd)
        {
            if (to_us_since_boot(_TimeNow) < to_us_since_boot(BrakeTime))
            {
                DriversBrake();
            }
            else
            {
                DriversStart();
                BrakeEnd = true;
            }
        }

        if (!BrakeStart || BrakeEnd)
        {
            PID_DriveA.setSetPoint(-SPA);
            PID_DriveB.setSetPoint(SPB);

            DriveA.setDuty(PID_DriveA.compute(SpeedR, _TimeNow)); //invert rigth motor feedback
            DriveB.setDuty(PID_DriveB.compute(SpeedL, _TimeNow));
        }
    }
}








