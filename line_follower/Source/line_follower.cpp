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
constexpr auto MOTOR_MAX_RPM = 2800;
constexpr auto ENCODER_PPR = 237.6f;
constexpr auto ENCODER_MAX_PULSE_PERIOD_US = 60 * 1000000 / MOTOR_MIN_RPM / ENCODER_PPR;
constexpr auto ENCODER_MIN_PULSE_PERIOD_US = 60 * 1000000 / MOTOR_MAX_RPM / ENCODER_PPR;
constexpr auto ENCODER_UPDATE_PERIOD_US = 1000;
constexpr auto ENCODER_UPDATE_RETRY_PERIOD_US = 30;
constexpr auto ENCODER_SAMPLE_BUFFER_SIZE = 32;
constexpr auto ENCODER_CONVERSION_BOT = 1.0f / (int)ENCODER_MAX_PULSE_PERIOD_US;
constexpr auto ENCODER_CONVERSION_TOP = 1.0f / (int)ENCODER_MIN_PULSE_PERIOD_US;
constexpr auto ENCODER_CONVERSION_SPAN = ENCODER_CONVERSION_TOP - ENCODER_CONVERSION_BOT;
constexpr auto ENCODER_WHEEL_DIAMETER_MM = 26;
constexpr auto ENCODER_WHEEL_LENGTH_MM = ENCODER_WHEEL_DIAMETER_MM * M_PI;
constexpr auto ENCODER_PULSE_TO_MM = (float)(ENCODER_WHEEL_LENGTH_MM / ENCODER_PPR);


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
    ESC.start(0.3f, 2000);
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
    LineSensor.setEmittersEnable(false);
    LineSensor.setLedsEnable(false);
    LineSensor.stop();
    Awake = false;
    LED0.stop();
    printf("DBG:SLEEP\n");
}

void LineFollower_s::wakeup(void)
{
    LineSensor.setEmittersEnable(true);
    LineSensor.setLedsEnable(true);
    LineSensor.start();
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
    DriversStop();
    ESC_Stop();
    Start = false;
    Stop = true;
    LED0.start(1000, 1000);
    PID_DriveA.reset();
    PID_DriveB.reset();
    // MotorA.stopBeep();
    // MotorB.stopBeep();
    SleepTime = make_timeout_time_ms(10000);
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
            .Gain = 1.8f,
            .IntegralTime = 0.3f,
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
        
        Config.StateFlags = 0;
        Config.ForwardSpeed = 0.5f;
        Config.EscSpeed = 0.5f;
        Config.SteeringGain = 0.0f;
        Config.SteeringGain2 = 0.0f;
        Config.CruiseGain = 0.0f;
        Config.CruiseGain2 = 0.0f;
        
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
    case VAR_CRUISE_GAIN:                   return {INTERFACE_GET_OK, Config.CruiseGain};
    case VAR_CRUISE_GAIN2:                  return {INTERFACE_GET_OK, Config.CruiseGain2};
    case VAR_STEERING_GAIN3:                return {INTERFACE_GET_OK, Config.SteeringGain3};
    case VAR_CRUISE_GAIN3:                  return {INTERFACE_GET_OK, Config.CruiseGain3};
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
    case CMD_LINE_SLEEP:                    sleep();                                                                                        return INTERFACE_SET_OK;
    case CMD_LINE_WAKEUP:                   wakeup();                                                                                       return INTERFACE_SET_OK;
    case CMD_ESC_ARM:                       ESC_Arm();                                                                                      return INTERFACE_SET_OK;
    case CMD_ESC_START:                     ESC_Start();                                                                                    return INTERFACE_SET_OK;
    case CMD_ESC_STOP:                      ESC_Stop();                                                                                     return INTERFACE_SET_OK;
    /*Common variables                                                                                                                           */
    case VAR_SPEED:                         Config.ForwardSpeed = _Value;                                                                   return INTERFACE_SET_OK;
    case VAR_ESC_SPEED:                     ESC.setSpeed(Config.EscSpeed = _Value);                                                         return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN:                 Config.SteeringGain = _Value;                                                                   return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN2:                Config.SteeringGain2 = _Value;                                                                  return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN:                   Config.CruiseGain = _Value;                                                                     return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN2:                  Config.CruiseGain2 = _Value;                                                                    return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN3:                Config.SteeringGain3 = _Value;                                                                  return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN3:                  Config.CruiseGain3 = _Value;                                                                    return INTERFACE_SET_OK;
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

void LineFollower_s::computeControl(absolute_time_t _TimeNow)
{
    auto [Roll, Pitch, Yaw] = IMU.getOrientation();
    auto LineHeading = LineSensor.computeLineHeading(Yaw);
    auto [ENC_Data, ENC_OK] = getEncoderData();
    auto [Range, Turn] = LineSensor.getLineDesc();
    auto Detected = LineSensor.isDetected();

    if (!ENC_OK)
    {
        return;
    }

    int PPA, PPB;
    float SpeedA, SpeedB;

    //Calculate motors speeds, -1.0f to 1.0f relative to MOTOR_NOMINAL_RPM, valuse below MOTOR_MIN_RPM are set to 0.0f, above MOTOR_MAX_RPM to 1.0f
    PPA = std::min(ENC_Data.PulsePeriodSumA, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);
    PPB = std::min(ENC_Data.PulsePeriodSumB, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);
    SpeedA = (((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * PPA) / (PPA * ENCODER_CONVERSION_SPAN)) * ENC_Data.PulseDirectionA;
    SpeedB = (((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * PPB) / (PPB * ENCODER_CONVERSION_SPAN)) * ENC_Data.PulseDirectionB;
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
    
    if (!Start || Stop)
    {
        if (to_us_since_boot(_TimeNow) > to_us_since_boot(SleepTime) && Awake)
        {
            sleep();
        }
        return;
    }
    

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
    float SPA, SPB;
    float BaseSpeed, SteeringOffset, SteeringGain, CruiseGain;

    if (!Detected)
    {
        //we don't see the line
        switch (Turn)
        {
        case LineSensor_s::LineTurn_t::LINE_NO_TURN:
            SteeringGain = Config.SteeringGain2;
            CruiseGain = Config.CruiseGain2;
            break;
        case LineSensor_s::LineTurn_t::LINE_TURN_LEFT:
        case LineSensor_s::LineTurn_t::LINE_TURN_RIGHT:
            SteeringGain = Config.SteeringGain3;
            CruiseGain = Config.CruiseGain3;
            break;
        }
    }
    else 
    {
        //we see the line
        switch (Range)
        {
        case LineSensor_s::LineRange_t::LINE_CENTER: //Do proportional, straight following
            SteeringGain = Config.SteeringGain;
            CruiseGain = Config.CruiseGain;
            break;
        case LineSensor_s::LineRange_t::LINE_LEFT: //Do proportional, different gain
        case LineSensor_s::LineRange_t::LINE_RIGHT:
            SteeringGain = Config.SteeringGain2;
            CruiseGain = Config.CruiseGain2;
            break;
        }

        
        // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
        // {
        //     //printf("LH: %f \n", LineHeading);
        //     //printf("A: %f, B: %f\n", DriveA.getDuty(), DriveB.getDuty());
        //     //printf("M1 %d, % .3f rpm, M2 %d, % .3f rpm, % .9f\n", Steps1, RPM1, Steps2, RPM2, PID_DriveB.getIntg());
        //     //printf("%f, %f, %f, %f, %d, %d\n", Speed0, TargetSpeed0, Speed1, TargetSpeed1, Encoder1_err, Encoder2_err);
        //     printf("%f, %f, %f\n",SteeringOffset,SPA,SPB);
        //     //printf("sB:%f sA:%f BS:%f, SO:%f, oB:%f, oA:%f, vB:%f, vA:%f, M+:%f, M-:%f, iB:%f, iA:%f, pB:%f, pA:%f\n", SpeedA, SpeedB, BaseSpeed, SteeringOffset, OffsetB, OffsetA, OverdriveB, OverdriveA, OffsetMaxPos, OffsetMaxNeg, PID_DriveB.getIntegral(), PID_DriveA.getIntegral(), DriveB.getDuty(), DriveA.getDuty());
        //     PrintTime = make_timeout_time_ms(200);
        // }
    }
    
    SteeringOffset = -std::clamp(LineHeading * SteeringGain, -2.0f, 2.0f);
    BaseSpeed = std::abs(LineHeading) < LINE_SENSOR_STEP_VALUE ? Config.ForwardSpeed : Config.ForwardSpeed - std::min(std::abs(LineHeading) * CruiseGain, 2.0f);
    
    SPA = std::clamp(SteeringOffset >= 0.0f ? BaseSpeed : BaseSpeed + SteeringOffset, -2.0f, 2.0f);
    SPB = std::clamp(SteeringOffset <= 0.0f ? BaseSpeed : BaseSpeed - SteeringOffset, -2.0f, 2.0f);
    
    PID_DriveA.setSetPoint(-SPA);//invert rigth motor
    PID_DriveB.setSetPoint(SPB);

    DriveA.setDuty(PID_DriveA.compute(SpeedA, _TimeNow));
    DriveB.setDuty(PID_DriveB.compute(SpeedB, _TimeNow));
}








