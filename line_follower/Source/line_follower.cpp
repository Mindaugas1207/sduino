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
PID_s PID_Tracking;
IMU_s IMU;
ESC_s ESC;

Blink_s LED0;

void sduino_adc_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
}

#define DRV_R 473.5f
constexpr auto ESC_PWM_PIN = 26U; //pwm5 A
constexpr auto MOTOR_B_ENCODER_A_PIN = 23U; //pwm0 b
constexpr auto MOTOR_B_ENCODER_B_PIN = 24U; //pwm1 b
constexpr auto MOTOR_A_ENCODER_A_PIN = 1U; // reik pakeist i 23 pwm3 b
constexpr auto MOTOR_A_ENCODER_B_PIN = 2U; //pwm4 b reik pakeist i 24 pwm4 a
constexpr auto MOTOR_B_ENCODER_PINS_MASK = 1U << MOTOR_B_ENCODER_A_PIN | 1U << MOTOR_B_ENCODER_B_PIN;
constexpr auto MOTOR_A_ENCODER_PINS_MASK = 1U << MOTOR_A_ENCODER_A_PIN | 1U << MOTOR_A_ENCODER_B_PIN;
constexpr auto MOTOR_MIN_RPM = 260;
//constexpr auto MOTOR_NOMINAL_RPM = 2800;
//constexpr auto MOTOR_MAX_RPM = 2800;
#ifdef MOTORDEF0
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
constexpr auto ENCODER_PULSE_TO_LENGTH_MM = ENCODER_WHEEL_LENGTH_MM / ENCODER_PPR;
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

SENSORX_VL53L0X_t dev_vlx;
uint16_t vlx_measurement = 0;
bool vlx_ongoing = false;
// volatile bool vlx_sync_int = true;
// #define VLX_INT_PIN 25
// static void vlx_hw_callback(uint gpio, uint32_t events)
// {
//     vlx_sync_int = true;
// }
// inline bool vlx_hw_new_data_available()
// {
//     return vlx_sync_int || gpio_get(VLX_INT_PIN);
// }
// inline void vlx_hw_start()
// {
//     gpio_set_irq_enabled(VLX_INT_PIN, GPIO_IRQ_EDGE_RISE, true);
//     vlx_sync_int = true;
// }
// inline void vlx_hw_stop()
// {
//     gpio_set_irq_enabled(VLX_INT_PIN, GPIO_IRQ_EDGE_RISE, false);
// }
void vlx_handle();
void vlx_getMeasurement();
void vlx_handle()
{
    if (!vlx_ongoing)
    {
        SENSORX_VL53L0X_StartMeasurement(&dev_vlx);
        vlx_ongoing = true;
    }
    else
    {
        if (SENSORX_VL53L0X_PollMeasurementDataReady(&dev_vlx) == 1)
        {
            vlx_ongoing = false;
            vlx_getMeasurement();
        }
    }


}

void vlx_getMeasurement()
{
    int error = SENSORX_OK;
    VL53L0X_RangingMeasurementData_t data;
    error = SENSORX_VL53L0X_GetMeasurementData(&dev_vlx, &data);
    error = SENSORX_VL53L0X_ClearInterrupt(&dev_vlx);
    if(error == SENSORX_OK)
    {
        //SENSORX_VL53L0X_PrintMeasurementData(&dev_vlx, data);
        if(data.RangeStatus == VL53L0X_RANGESTATUS_RANGEVALID)
        {
            vlx_measurement = data.RangeMilliMeter;
        }
        else
        {
            vlx_measurement = 10000;
        }
    }
    else
    {
        vlx_measurement = 12000;
    }
}

bool vlx_init()
{
    vlx_measurement = 20000;
    if (SENSORX_VL53L0X_Init(&dev_vlx, &i2c_port, PORT_CHANNEL_1, VL53L0X_DEFAULT_ADDRESS, 0) != SENSORX_OK) return false;
    if (SENSORX_VL53L0X_GetConfig(&dev_vlx) != SENSORX_OK) return false;
    if (SENSORX_VL53L0X_RunCalibrations(&dev_vlx, VL53L0X_CALIBRATE_SPADS) != SENSORX_OK) return false;
    if (SENSORX_VL53L0X_RunCalibrations(&dev_vlx, VL53L0X_CALIBRATE_REF) != SENSORX_OK) return false;
    dev_vlx.config.RANGING_MODE = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;

    //dev_vlx.config.RANGING_MODE = VL53L0X_DEVICEMODE_SINGLE_RANGING;
    //inst->config.RANGING_MODE = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
    if (SENSORX_VL53L0X_SetConfig(&dev_vlx) != SENSORX_OK) return false;
    if (SENSORX_VL53L0X_PrintAll(&dev_vlx) != SENSORX_OK) return false;

    // gpio_init(VLX_INT_PIN);
    // gpio_set_dir(VLX_INT_PIN, GPIO_IN);
    // gpio_set_irq_enabled_with_callback(VLX_INT_PIN, GPIO_IRQ_EDGE_RISE, true, &vlx_hw_callback);
    // vlx_hw_start();
    if (SENSORX_VL53L0X_StartMeasurement(&dev_vlx) != SENSORX_OK) return false;
    vlx_ongoing = true;
    
    return true;
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
    PID_Tracking.reset();
    PID_DriveA.reset();
    PID_DriveB.reset();
}

void DriversBrake() {
    DriveA.brake();
    DriveB.brake();
    PID_Tracking.reset();
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
    PID_Tracking.reset();
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
    PID_Tracking.reset();
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
    PID_Tracking.reset();
    PID_DriveA.reset();
    PID_DriveB.reset();
    // MotorA.startBeep(500, 500);
    // MotorB.startBeep(500, 500);
    printf("DBG:STOP_ERROR\n");
}

//static const int mapc[LINE_SENSOR_HW_NUM_SENSORS] = {1, 3, 5, 7, 9, 11, 13, 14, 12, 10, 8, 6, 4, 2, 0};

void LineFollower_s::init(void)
{
    LED0.init(SDUINO_INTERNAL_LED_PIN);
    stdio_init_all();
   
    //while (!stdio_usb_connected()) { sleep_ms(500); }

    printf("DBG:INIT\n");

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
    ESC.init(29);//ESC_PWM_PIN
    //sduino_adc_init();
    if (port_spi_init(&spi_port, spi_internal) == PORT_ERROR) printf("DBG:PORT_SPI_INIT->FAIL\n");
    printf("DBG:SPI INIT\n");
    if (port_i2c_init(&i2c_port, i2c_internal, PORT_MUX_DEFAULT_ADDRESS) == PORT_ERROR) printf("DBG:PORT_I2C_INIT->FAIL\n");
    printf("DBG:I2C INIT\n");
    LineSensorHW.port_inst = &i2c_port;
    // if (line_sensor_hw_init(&LineSensorHW) == LINE_SENSOR_HW_ERROR) printf("DBG:PORT_HW_INIT->FAIL\n");
    // printf("DBG:HW INIT\n");

    // uint16_t lows[PADC_BUFFER_SIZE];
    // uint16_t highs[PADC_BUFFER_SIZE];
    // for (int i = 0; i < 15; i++)
    // {
    //     lows[i] = std::numeric_limits<uint16_t>::max();
    //     highs[i] = std::numeric_limits<uint16_t>::min();
    //     line_sensor_hw_set_emitter_power(&LineSensorHW, i, 1.0f);
    //     line_sensor_hw_set_emitter_enable(&LineSensorHW, i, true);
    //     line_sensor_hw_set_led_enable(&LineSensorHW, i, true);
    // }
    // lows[15] = std::numeric_limits<uint16_t>::max();
    // highs[15] = std::numeric_limits<uint16_t>::min();
    // line_sensor_hw_set_emitter_enable(&LineSensorHW, LINE_SENSOR_STROBE_EMITER, true);
    // line_sensor_hw_set_emitter_enable(&LineSensorHW, LINE_SENSOR_STROBE_EMITER, true);
    // line_sensor_hw_emitter_update(&LineSensorHW);
    // line_sensor_hw_led_update(&LineSensorHW);

    // printf("DBG:LEDS INIT\n");
    
    // for(;;)
    // {
    //     uint16_t buffer[PADC_BUFFER_SIZE];
    //     padc_start();
    //     gpio_put(SDUINO_INTERNAL_LED_PIN, false);
    //     while (!padc_new_data_available())
    //     {
            
    //         //printf("DBG:WAITING\n");
    //         //sleep_ms(100);
    //     }

        

    //     gpio_put(SDUINO_INTERNAL_LED_PIN, true);
    //     if (padc_read(buffer))
    //     {
    //         //00      | 01      | 02      | 03      | 04      | 05      | 06      | 07      | 08      | 09      | 10      | 11      | 12      | 13      | 14      | 15      |
    //         //14H 14L | 00H 00L | 13H 13L | 01H 01L | 12H 12L | 02H 02L | 11H 11L | 03H 03L | 10H 10L | 04H 04L | 09H 09L | 05H 05L | 08H 08L | 06H 06L | 07H 07L | DUH DUL |
    //         // printf("%d | %d | %d | %d | %d | %d | %d / %d \\ %d | %d | %d | %d | %d | %d | %d |< %d\n",
    //         //     buffer[1],  buffer[3],  buffer[5],  buffer[7],  buffer[9], buffer[11],
    //         //     buffer[13],  buffer[14],  buffer[12],  buffer[10],  buffer[8], buffer[6],
    //         //     buffer[4], buffer[2], buffer[0], buffer[15]
    //         // );
    //         //sleep_ms(100);
    //         for (int i = 0; i < 15; i++)
    //         {
    //             uint16_t val = buffer[mapc[i]];
    //             lows[i] = std::min(val, lows[i]);
    //             highs[i] = std::max(val, highs[i]);
    //             float Span = highs[i] != lows[i] ? (float)(highs[i] - lows[i]) : 1.0f;
    //             float Value = ((float)(val - lows[i])) / Span;
    //             bool Detected = Value > 0.5f + LINE_SENSOR_THRESHOLD_CLEARANCE ? true : (Value < 0.5f - LINE_SENSOR_THRESHOLD_CLEARANCE ? false : Detected);
    //             line_sensor_hw_set_led_power(&LineSensorHW, i, Detected ? 1.0f : 0.0f);
    //         }
            
    //         line_sensor_hw_led_update(&LineSensorHW);
    //     }
    // }

    // if (vlx_init() == false)
    // {
    //     printf("DBG:VLX_INIT->FAIL\n");
    //     while (true) { tight_loop_contents(); }
    // }
    //while(1){vlx_handle();}

    if(!IMU.init(&i2c_port, &spi_port)) {
       printf("DBG:IMU_INIT->FAIL\n");
       while (true) { tight_loop_contents(); }
    }
    IMU.startAsyncProcess();
    printf("IMU START!\n");
    //while(1){IMU.runAsyncProcess(get_absolute_time());}

    
    if (LineSensor.init(&LineSensorHW) == LINE_SENSOR_ERROR) {
       printf("DBG:LINE_SENSOR_INIT->FAIL\n");
       while (true) { tight_loop_contents(); }
    }
    printf("LS INIT!\n");
    
    load();
    printf("LOAD!\n");
    
    stop();
    wakeup();
    sleep();
}

void LineFollower_s::save(void)
{
    Config.IMU = IMU.getConfiguration();
    Config.LineSensor = LineSensor.getConfiguration();
    Config.DriveA = DriveA.getConfiguration();
    Config.DriveB = DriveB.getConfiguration();
    Config.PID_Drives = PID_DriveA.getConfiguration();
    Config.PID_Tracking = PID_Tracking.getConfiguration();

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
        PID_Tracking.loadConfiguration(Config.PID_Tracking);
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
            .Gain = 1.0f,
            .IntegralTime = 0.25f,
            .IntegralLimit = 300.0f,
            .IntegralRateLimit = 10.0f,
            .IntegralAntiWindup = 0.0f,
            .DerivativeTime = 0.0f,
            .DerivativeCutoff = 1.0f,
            .OutputLimit = 1.0f,
            .DeadZone = 0.026f
        };

        PID_DriveA.loadConfiguration(DrivePID);
        PID_DriveB.loadConfiguration(DrivePID);
        PID_Tracking.loadConfiguration(DrivePID);
        Config.PID_Drives = PID_DriveA.getConfiguration();
        Config.PID_Tracking = PID_Tracking.getConfiguration();

        Config.DriveA = DriveA.getConfiguration();
        Config.DriveB = DriveB.getConfiguration();

        Config.DriveA.Bias = 0.1f;
        Config.DriveA.DeadZone = 0.024f;
        Config.DriveA.PWM_CLK_DIV = 1.0f;
        Config.DriveB.Bias = 0.1f;
        Config.DriveB.DeadZone = 0.024f;
        Config.DriveB.PWM_CLK_DIV = 1.0f;

        DriveA.loadConfiguration(Config.DriveA);
        DriveB.loadConfiguration(Config.DriveB);
        Config.DriveA = DriveA.getConfiguration();
        Config.DriveB = DriveB.getConfiguration();

        LineSensor_s::Config_t LS_Config = {
            .EmittersPower = 1.0f,
            .LedBrightness = 1.0f,
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
    /*PID1 variables                        */
    case VAR_PID1_GAIN:                     return {INTERFACE_GET_OK, PID_Tracking.getGain()};
    case VAR_PID1_INTEGRAL_TIME:            return {INTERFACE_GET_OK, PID_Tracking.getIntegralTime()};
    case VAR_PID1_INTEGRAL_LIMIT:           return {INTERFACE_GET_OK, PID_Tracking.getIntegralLimit()};
    case VAR_PID1_INTEGRAL_RATE_LIMIT:      return {INTERFACE_GET_OK, PID_Tracking.getIntegralRateLimit()};
    case VAR_PID1_DERIVATIVE_TIME:          return {INTERFACE_GET_OK, PID_Tracking.getDerivativeTime()};
    case VAR_PID1_DERIVATIVE_CUTOFF:        return {INTERFACE_GET_OK, PID_Tracking.getDerivativeCutoff()};
    case VAR_PID1_DEADZONE:                 return {INTERFACE_GET_OK, PID_Tracking.getDeadZone()};

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
    /*PID1 variables                                                                                                                             */
    case VAR_PID1_GAIN:                     PID_Tracking.setGain(Config.PID_Tracking.Gain = _Value);                                            return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_TIME:            PID_Tracking.setIntegralTime(Config.PID_Tracking.IntegralTime = _Value);                            return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_LIMIT:           PID_Tracking.setIntegralLimit(Config.PID_Tracking.IntegralLimit = _Value);                          return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_RATE_LIMIT:      PID_Tracking.setIntegralRateLimit(Config.PID_Tracking.IntegralRateLimit = _Value);                  return INTERFACE_SET_OK;
    case VAR_PID1_DERIVATIVE_TIME:          PID_Tracking.setDerivativeTime(Config.PID_Tracking.DerivativeTime = _Value);                        return INTERFACE_SET_OK;
    case VAR_PID1_DERIVATIVE_CUTOFF:        PID_Tracking.setDerivativeCutoff(Config.PID_Tracking.DerivativeCutoff = _Value);                    return INTERFACE_SET_OK;
    case VAR_PID1_DEADZONE:                 PID_Tracking.setDeadZone(Config.PID_Tracking.DeadZone = _Value);                                    return INTERFACE_SET_OK;
    default:                                                                                                                                return INTERFACE_SET_NOVAL;
    }
}

void LineFollower_s::run(void)
{
    absolute_time_t TimeNow = get_absolute_time();
    //sduino_adc_read();
    IMU.runAsyncProcess(TimeNow);
    LineSensor.process(TimeNow);
    ///vlx_handle();

    computeControl(TimeNow);
    
    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    // {
    //     //printf("M1 %d, % .3f rpm, M2 %d, % .3f rpm, % .9f\n", Steps1, RPM1, Steps2, RPM2, PID_DriveB.getIntg());
    //     //printf("%f, %f, %f, %f, %d, %d\n", Speed0, TargetSpeed0, Speed1, TargetSpeed1, Encoder1_err, Encoder2_err);
    //     //printf("E:%f I:%f D:%f, SA:%f, G:%f, O:%f\n", PID_DriveA.getError(), PID_DriveA.getIntegral(), PID_DriveA.getDerivative(), EncoderSpeedA, PID_DriveA.getGain(), PID_DriveA.getOutput());
    //     printf("VLX: %d\n", vlx_measurement);
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

// float LineFollower_s::computePathHeading(float _FrameHeading, bool lock)
// {
//     float h = _unwrap(ph, _FrameHeading);
//     ph = h;
    
//     if (lock)
//     {
//         LastFrameHeading = h;
//         pathHeading = 0.0f;
//     }
//     else
//     {
//         pathHeading = (h - LastFrameHeading);
//     }
    

//     return pathHeading * LINE_SENSOR_ANGLE_RAD_TO_VALUE;
// }

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
        step_counterA += PulseR - iPulseR;
        step_counterB += PulseL - iPulseL;
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

    // if (!doManuver && !manuver_done && (Start && !Stop))
    // {
    //     if (vlx_start)
    //     {
    //         if (vlx_measurement > 700)
    //         {
    //             vlx_start = false;
    //             vlxTime = get_absolute_time();
    //         }
    //         else if (to_ms_since_boot(vlxTime) - to_ms_since_boot(get_absolute_time()) > 20)
    //         {
    //             vlx_start = false;
    //             doManuver = true;
    //             manuverStep = 0;
    //             PathTime = get_absolute_time();
    //         }
    //     }
    //     else
    //     {
    //         if (vlx_measurement < 700)
    //         {
    //             vlx_start = true;
    //             vlxTime = get_absolute_time();
    //         }
    //     }
    // }
    

    auto LineHeading = LineSensor.computeLineHeading(Yaw);
    //auto PathHeading = computePathHeading(Yaw, !doManuver);
    auto [Range, Turn] = LineSensor.getLineDesc();
    auto Detected = LineSensor.isDetected();

    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    // {
    // //     //printf("LH: %f \n", LineHeading);
    // //     //printf("A: %f, B: %f\n", DriveA.getDuty(), DriveB.getDuty());
    // //     //printf("M1 %d, % .3f rpm, M2 %d, % .3f rpm, % .9f\n", Steps1, RPM1, Steps2, RPM2, PID_DriveB.getIntg());
    //     // char buffer[256];
    //     // //sprintf(buffer, "CMD:WS(\"R\":%.2f,\"L\":%.2f,\"Y\":%.3f,\"H\":%.3f)\n", SpeedR, SpeedL, Yaw, LineHeading);

    //     // int n = sprintf(buffer, "CMD:WS(");

    //     // for (int i = 0; i < LineSensor._LineCount; i++)
    //     // {
    //     //     n += sprintf(buffer + n, "[%d,%d,%c]\n", LineSensor._Lines[i].Start, LineSensor._Lines[i].End, LineSensor._Lines[i].Color ? 'B' : 'W');
    //     // }
    //     // n += sprintf(buffer, ")\n");
    //     // uart_puts(uart_internal, buffer);
    // //     // char str1[15];
    // //     // char str2[15];
    // //     char buffer[128];
    // //     //sprintf(str1, "%.9f", iX_f);
    // //     //sprintf(str2, "%.9f", iY_f);
    // //     //sprintf(buffer, "CMD:WS(\"X\":%d.%s,\"Y\":%d.%s,\"O\":%.9f,\"L\":%.9f,\"I\":%.9f)\n", iX, str1 + 2, iY, str2 + 2, iYaw, LineHeading, Yaw);
    // //     sprintf(buffer, "CMD:WS(\"X\":%.12f,\"Y\":%.12f,\"O\":%.9f,\"L\":%.9f,\"I\":%.9f)\n", iX_f, iY_f, iYaw, LineHeading, Yaw);
    // //     uart_puts(uart_internal, buffer);
    // //     //printf("R: %d, L: %d, X: %d %f Y: %d %f O: %.9f L: %.9f Yaw: %.9f\n", iPulseR, iPulseL, iX, iX_f, iY, iY_f, iYaw, LineHeading, Yaw);
    // //     //printf("sB:%f sA:%f BS:%f, SO:%f, oB:%f, oA:%f, vB:%f, vA:%f, M+:%f, M-:%f, iB:%f, iA:%f, pB:%f, pA:%f\n", SpeedA, SpeedB, BaseSpeed, SteeringOffset, OffsetB, OffsetA, OverdriveB, OverdriveA, OffsetMaxPos, OffsetMaxNeg, PID_DriveB.getIntegral(), PID_DriveA.getIntegral(), DriveB.getDuty(), DriveA.getDuty());
    //      PrintTime = make_timeout_time_ms(250);
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

    if (Range == LineSensor_s::LineRange_t::LINE_CENTER)
    {
        ReturningOnLine = false;
    }

    bool LineTracking = (Detected && !ReturningOnLine);// || (!Detected && Turn != LineSensor_s::LineTurn_t::LINE_NO_TURN);
    //bool LineTracking = Detected;

    // if (doManuver)
    // {
    //     float dest = 0.0f;
    //     float dist_mm = (step_counterA + step_counterB * ENCODER_PULSE_TO_LENGTH_MM) / 2;
    //     switch(manuverStep)
    //     {
    //         case 0:
    //         dest = LINE_SENSOR_TURN90_VALUE;
    //         if (std::abs(PathHeading) < LINE_SENSOR_STEP_VALUE / 10.0f)
    //         {
    //             manuverStep++;
    //             step_counterA = 0;
    //             step_counterB = 0;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 1:
    //         dest = LINE_SENSOR_TURN90_VALUE;
    //         if (dist_mm > 500)
    //         {
    //             manuverStep++;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 2:
    //         dest = 0.0f;
    //         if (std::abs(PathHeading) < LINE_SENSOR_STEP_VALUE / 10.0f)
    //         {
    //             manuverStep++;
    //             step_counterA = 0;
    //             step_counterB = 0;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 3:
    //         dest = 0.0f;
            
    //         if (dist_mm > 500)
    //         {
    //             manuverStep++;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 4:
    //         dest = -LINE_SENSOR_TURN90_VALUE;
    //         if (std::abs(PathHeading) < LINE_SENSOR_STEP_VALUE / 10.0f)
    //         {
    //             manuverStep++;
    //             step_counterA = 0;
    //             step_counterB = 0;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 5:
    //         default:
    //         dest = -LINE_SENSOR_TURN90_VALUE;
    //         if (Detected || to_ms_since_boot(PathTime) - to_ms_since_boot(get_absolute_time()) > 5000)
    //         {
    //             doManuver = false;
    //             manuver_done = true;
    //             manuverStep = 0;
    //         }
    //         break;
    //     }
        
    //     SteeringGain = Config.SteeringGain4;
    //     CruiseGain = Config.CruiseGain4;
    //     PathHeading = dest - PathHeading;
    //     SteeringOffset = -PathHeading * SteeringGain;

    //     BaseSpeed = Config.ForwardSpeed - std::clamp(std::abs(PathHeading) * CruiseGain, 0.0f, Config.ForwardSpeed);

    //     SPA = std::clamp(SteeringOffset >= 0.0f ? BaseSpeed : BaseSpeed + SteeringOffset, -10.0f, 10.0f);
    //     SPB = std::clamp(SteeringOffset <= 0.0f ? BaseSpeed : BaseSpeed - SteeringOffset, -10.0f, 10.0f);
    // }
    // else
    // {
        if (LineTracking)
        {
            BrakeStart = false;
            BrakeEnd = false;
            SteeringGain = Config.SteeringGain;
            CruiseGain = Config.CruiseGain;
            LineHeading = std::abs(LineHeading) < LINE_SENSOR_STEP_VALUE / 10.0f ? 0.0f : LineHeading;
            PID_Tracking.setSetPoint(0.0f);
            SteeringOffset = PID_Tracking.compute(LineHeading, _TimeNow);

            BaseSpeed = Config.ForwardSpeed - std::clamp(std::abs(LineHeading) * CruiseGain, 0.0f, Config.ForwardSpeed);

            SPA = std::clamp(SteeringOffset >= 0.0f ? BaseSpeed : BaseSpeed + SteeringOffset, -10.0f, 10.0f);
            SPB = std::clamp(SteeringOffset <= 0.0f ? BaseSpeed : BaseSpeed - SteeringOffset, -10.0f, 10.0f);
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

            float SteeringGainF = Config.SteeringGain2;
            float SteeringGainB = Config.SteeringGain3;
            CruiseGain = Config.CruiseGain2;
            BaseSpeed = Config.ForwardSpeed - std::clamp(std::abs(LineHeading) * CruiseGain, 0.0f, Config.ForwardSpeed);
            float SteeringOffsetF = -LineHeading * SteeringGainF;
            float SteeringOffsetB = -LineHeading * SteeringGainB;

            if (LineHeading > 0)
            {
                SPA = std::clamp(BaseSpeed + SteeringOffsetB, -10.0f, 10.0f);
                SPB = std::clamp(BaseSpeed - SteeringOffsetF, -10.0f, 10.0f);
            }
            else
            {
                SPA = std::clamp(BaseSpeed + SteeringOffsetF, -10.0f, 10.0f);
                SPB = std::clamp(BaseSpeed - SteeringOffsetB, -10.0f, 10.0f);
            }
        }
    //}

    

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
        // if (Roll > 0.6f || Roll < -0.6f || Pitch > 0.6f || Pitch < -0.6f)
        // {
        //     if (!Stop)
        //         stop_error();
        //     return;
        // }

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








