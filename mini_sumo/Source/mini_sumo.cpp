
#include "mini_sumo.hpp"

NVM_s NVM;
port_spi_t spi_port;
port_i2c_t i2c_port;
Interface_s Interface;
MotorDriver_s MotorA;
MotorDriver_s MotorB;
PID_s PID0;
IMU_s IMU;
//SENSORX_VL53L0X_t vlx[NUM_SENSORS];

Blink_s LED0;

void mini_sumo_s::sleep(void)
{
    MotorA.stop();
    MotorB.stop();
    Start = false;
    Stop = true;
    LED0.stop();
    MotorA.stopBeep();
    MotorB.stopBeep();
    Awake = false;
    printf("DBG:SLEEP\n");
}

void mini_sumo_s::wakeup(void)
{
    IMU.startAsyncProcess();
    MotorA.stop();
    MotorB.stop();
    Start = false;
    Stop = true;
    LED0.start(1000, 1000);
    MotorA.stopBeep();
    MotorB.stopBeep();
    Awake = true;
    printf("DBG:WAKEUP\n");
}

void mini_sumo_s::setSpeed(float Left, float Rigth)
{
    MotorA.setSetPoint(Rigth);
    MotorB.setSetPoint(Left);
}

void mini_sumo_s::start(void)
{
    Start = true;
    Stop = false;
    StartTimedOut = false;
    LED0.stop();
    MotorA.stopBeep();
    MotorB.stopBeep();
    StartTimeout = make_timeout_time_ms(5000);
    printf("DBG:START\n");
}

void mini_sumo_s::stop(void)
{
    MotorA.stop();
    MotorB.stop();
    Start = false;
    Stop = true;
    StartTimedOut = false;
    LED0.start(1000, 1000);
    MotorA.stopBeep();
    MotorB.stopBeep();
    printf("DBG:STOP\n");
}

void mini_sumo_s::stop_error(void)
{
    MotorA.stop();
    MotorB.stop();
    Start = false;
    Stop = true;
    StartTimedOut = false;
    sleep_ms(500);
    LED0.start(500, 500);
    MotorA.startBeep(500, 500);
    MotorB.startBeep(500, 500);
    printf("DBG:STOP_ERROR\n");
}

void mini_sumo_s::turnLeft()
{
    setSpeed(-Config.TurnNegative, Config.TurnPositive);
}

void mini_sumo_s::turnRight()
{
    setSpeed(Config.TurnPositive, -Config.TurnNegative);
}

void mini_sumo_s::init(void)
{
    LED0.init(SDUINO_INTERNAL_LED_PIN);
    stdio_init_all();

    i2c_init(i2c_internal, 10 * 1000);
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

    gpio_init(STRT_PIN);
    gpio_set_dir(STRT_PIN, GPIO_IN);
    gpio_init(KILL_PIN);
    gpio_set_dir(KILL_PIN, GPIO_IN);

    gpio_init(EDGE_SENSOR_LEFT_PIN);
    gpio_set_dir(EDGE_SENSOR_LEFT_PIN, GPIO_IN);
    gpio_init(EDGE_SENSOR_CENTER_PIN);
    gpio_set_dir(EDGE_SENSOR_CENTER_PIN, GPIO_IN);
    gpio_init(EDGE_SENSOR_RIGHT_PIN);
    gpio_set_dir(EDGE_SENSOR_RIGHT_PIN, GPIO_IN);
    
    Interface.Init(f_getCallback, f_setCallback, INTERFACE_ENUM_FIRST, INTERFACE_ENUM_LAST, INTERFACE_ENUM_SIZE);

    NVM.init(FLASH_SECTOR_SIZE, &Config, sizeof(Config));
    PID0.init();
    MotorA.init(SDUINO_INTERNAL_DRV_A_IN2_PIN, SDUINO_INTERNAL_DRV_A_IN1_PIN);
    MotorB.init(SDUINO_INTERNAL_DRV_B_IN1_PIN, SDUINO_INTERNAL_DRV_B_IN2_PIN);

    if (port_i2c_init(&i2c_port, i2c_internal, PORT_MUX_NONE) == PORT_ERROR) printf("DBG:PORT_I2C_INIT->FAIL\n");
    if (port_spi_init(&spi_port, spi_internal) == PORT_ERROR) printf("DBG:PORT_SPI_INIT->FAIL\n");

    if(!IMU.init(&i2c_port, &spi_port)) {
        printf("DBG:IMU_INIT->FAIL\n");
        while (true) { tight_loop_contents(); }
    }

    //vlx_multi_init();

    load();

    wakeup();

    stop();
}

void mini_sumo_s::save(void)
{
    Config.PID0 = PID0.getConfiguration();
    Config.MotorA = MotorA.getConfiguration();
    Config.MotorB = MotorB.getConfiguration();
    Config.IMU = IMU.getConfiguration();
    Config.setLockCode(NVM_CONFG_LOCK_CODE);
    NVM.program();
}

void mini_sumo_s::load(void)
{
    NVM_CONFIG_OK = false;
    NVM.load();
    if (Config.getLockCode() == NVM_CONFG_LOCK_CODE)
    {
        //Load PID0
        PID0.loadConfiguration(Config.PID0);
        //Load Motors
        MotorA.loadConfiguration(Config.MotorA);
        MotorB.loadConfiguration(Config.MotorB);
        //Load IMU
        IMU.loadConfiguration(Config.IMU);
        NVM_CONFIG_OK = true;
    }
    else
    {
        //NVM empty or corrupted
        //Load Default Configurations
        PID0.loadDefaultConfiguration();
        MotorA.loadDefaultConfiguration();
        MotorB.loadDefaultConfiguration();
        IMU.loadDefaultConfiguration();
        
        Config.PID0 = PID0.getConfiguration();
        Config.MotorA = MotorA.getConfiguration();
        Config.MotorB = MotorB.getConfiguration();
        Config.IMU = IMU.getConfiguration();
        Config.StateFlags = 0;
        Config.ForwardSpeed = 0.0f;
        Config.TurnPositive = 0.5f;
        Config.TurnNegative = 0.5f;
        Config.TurnBias = 0.0f;
        Config.LockCode = 0;
    }
}

std::tuple<float, bool> mini_sumo_s::get(int _Enum)
{
    switch(_Enum)
    {
    /*Commands variables                    */
    case CMD_START:                         return {Start && !Stop, true};
    case CMD_STOP:                          return {!Start && Stop, true};
    case CMD_IMU_CALIBRATE:                 return {IMU.isCalibrated(), true};
    case CMD_NVM_ERASE:                     return {NVM_CONFIG_OK, true};
    case CMD_NVM_LOAD:                      return {NVM_CONFIG_OK, true};
    case CMD_NVM_SAVE:                      return {NVM_CONFIG_OK, true};
    case CMD_LINE_SLEEP:                    return {!Awake, true};
    case CMD_LINE_WAKEUP:                   return {Awake, true};
    /*Common variables                      */
    case VAR_SPEED:                         return {Config.ForwardSpeed, true};
    case VAR_TURN_POS:                      return {Config.TurnPositive, true};
    case VAR_TURN_NEG:                      return {Config.TurnNegative, true};
    case VAR_TURN_BIAS:                     return {Config.TurnBias, true};
    /*Motor variables                       */
    case VAR_MOTORS_PMW_MAX:                return {MotorA.getPWM_MAX(), true};
    case VAR_MOTORS_CLK_DIV:                return {MotorA.getCLK_DIV(), true};
    case VAR_MOTORS_RAMP_STEP:              return {MotorA.getStep(), true};
    case VAR_MOTORS_RAMP_SLOPE:             return {MotorA.getSlope(), true};
    case VAR_MOTORS_RAMP_STEP_TIME:         return {MotorA.getRampStepTime(), true};
    case VAR_MOTORS_SPEED_MAX_POS:          return {MotorA.getSpeedMaxPos(), true};
    case VAR_MOTORS_SPEED_MAX_NEG:          return {MotorA.getSpeedMaxNeg(), true};
    /*Line variables                        */
    /*PID0 variables                        */
    case VAR_PID0_MODE:                     return {PID0.getMode(), true};
    case VAR_PID0_KP:                       return {PID0.getKp(), true};
    case VAR_PID0_KD:                       return {PID0.getKd(), true};
    case VAR_PID0_KI:                       return {PID0.getKi(), true};
    case VAR_PID0_GAIN:                     return {PID0.getGain(), true};
    case VAR_PID0_OUTPUT_LIMIT:             return {PID0.getOutputLimit(), true};
    case VAR_PID0_SAMPLING_TIME:            return {PID0.getSamplingTime(), true};
    case VAR_PID0_INTEGRAL_TIME:            return {PID0.getIntegralTime(), true};
    case VAR_PID0_INTEGRAL_LIMIT:           return {PID0.getIntegralLimit(), true};
    case VAR_PID0_INTEGRAL_RATE_LIMIT:      return {PID0.getIntegralRateLimit(), true};
    case VAR_PID0_INTEGRAL_ANTI_WINDUP:     return {PID0.getIntegralAntiWindup(), true};
    case VAR_PID0_DERIVATIVE_TIME:          return {PID0.getDerivativeTime(), true};
    case VAR_PID0_DERIVATIVE_CUTOFF:        return {PID0.getDerivativeCutoff(), true};
    case VAR_PID0_SETPOINT:                 return {PID0.getSetPoint(), true};
    case VAR_IMU_YAW:                       return {yaw, true};
    default:                                return {0.0f, false};
    }
}

void mini_sumo_s::set(int _Enum, float _Value)
{
    switch(_Enum)
    {
    /*Commands variables                                                                                                                         */
    case CMD_START:                         start();                                                                                        return;
    case CMD_STOP:                          stop();                                                                                         return;
    case CMD_IMU_CALIBRATE:                 IMU.startCalibration();                                                                         return;
    case CMD_NVM_ERASE:                     NVM.erase();                                                                                    return;
    case CMD_NVM_LOAD:                      load();                                                                                         return;
    case CMD_NVM_SAVE:                      save();                                                                                         return;
    case CMD_LINE_SLEEP:                    sleep();                                                                                        return;
    case CMD_LINE_WAKEUP:                   wakeup();                                                                                       return;
    /*Common variables                                                                                                                           */
    case VAR_SPEED:                         Config.ForwardSpeed = _Value;                                                                   return;
    case VAR_TURN_POS:                      Config.TurnPositive = _Value;                                                                   return;
    case VAR_TURN_NEG:                      Config.TurnNegative = _Value;                                                                   return;
    case VAR_TURN_BIAS:                     Config.TurnBias = _Value;                                                                       return;
    /*Motor variables                                                                                                                            */
    case VAR_MOTORS_PMW_MAX:                MotorA.setPWM_MAX(Config.MotorA.PWM_MAX = _Value);                                              /*   */
                                            MotorB.setPWM_MAX(Config.MotorB.PWM_MAX = _Value);                                              return;
    case VAR_MOTORS_CLK_DIV:                MotorA.setCLK_DIV(Config.MotorA.CLK_DIV = _Value);                                              /*   */
                                            MotorB.setCLK_DIV(Config.MotorB.CLK_DIV = _Value);                                              return;
    case VAR_MOTORS_RAMP_STEP:              MotorA.setStep(Config.MotorA.Step = _Value);                                                    /*   */
                                            MotorB.setStep(Config.MotorB.Step = _Value);                                                    return;
    case VAR_MOTORS_RAMP_SLOPE:             MotorA.setSlope(Config.MotorA.Slope = _Value);                                                  /*   */
                                            MotorB.setSlope(Config.MotorB.Slope = _Value);                                                  return;
    case VAR_MOTORS_RAMP_STEP_TIME:         MotorA.setRampStepTime(Config.MotorA.RampTimeStepTime_ms = _Value);                             /*   */
                                            MotorB.setRampStepTime(Config.MotorB.RampTimeStepTime_ms = _Value);                             return;
    case VAR_MOTORS_SPEED_MAX_POS:          MotorA.setSpeedMaxPos(Config.MotorA.SpeedMaxPos = _Value);                                      /*   */
                                            MotorB.setSpeedMaxPos(Config.MotorB.SpeedMaxPos = _Value);                                      return;
    case VAR_MOTORS_SPEED_MAX_NEG:          MotorA.setSpeedMaxNeg(Config.MotorA.SpeedMaxNeg = _Value);                                      /*   */
                                            MotorB.setSpeedMaxNeg(Config.MotorB.SpeedMaxNeg = _Value);                                      return;
    /*PID0 variables                                                                                                                             */
    case VAR_PID0_MODE:                     PID0.setMode(Config.PID0.Mode = (PID_s::Modes)_Value);                                          return;
    case VAR_PID0_KP:                       PID0.setKp(Config.PID0.Kp = _Value);                                                            return;
    case VAR_PID0_KD:                       PID0.setKd(Config.PID0.Kd = _Value);                                                            return;
    case VAR_PID0_KI:                       PID0.setKi(Config.PID0.Ki = _Value);                                                            return;
    case VAR_PID0_GAIN:                     PID0.setGain(Config.PID0.Gain = _Value);                                                        return;
    case VAR_PID0_OUTPUT_LIMIT:             PID0.setOutputLimit(Config.PID0.OutputLimit = _Value);                                          return;
    case VAR_PID0_SAMPLING_TIME:            PID0.setSamplingTime(Config.PID0.SamplingTime_ms = _Value);                                     return;
    case VAR_PID0_INTEGRAL_TIME:            PID0.setIntegralTime(Config.PID0.IntegralTime = _Value);                                        return;
    case VAR_PID0_INTEGRAL_LIMIT:           PID0.setIntegralLimit(Config.PID0.IntegralLimit = _Value);                                      return;
    case VAR_PID0_INTEGRAL_RATE_LIMIT:      PID0.setIntegralRateLimit(Config.PID0.IntegralRateLimit = _Value);                              return;
    case VAR_PID0_INTEGRAL_ANTI_WINDUP:     PID0.setIntegralAntiWindup(Config.PID0.IntegralAntiWindup = _Value);                            return;
    case VAR_PID0_DERIVATIVE_TIME:          PID0.setDerivativeTime(Config.PID0.DerivativeTime = _Value);                                    return;
    case VAR_PID0_DERIVATIVE_CUTOFF:        PID0.setDerivativeCutoff(Config.PID0.DerivativeCutoff = _Value);                                return;
    case VAR_PID0_SETPOINT:                 PID0.setSetPoint(Config.PID0.SetPoint = _Value);                                                return;
    default:                                                                                                                                return;
    }
}

void mini_sumo_s::run(void)
{
    absolute_time_t TimeNow = get_absolute_time();
    IMU.runAsyncProcess(TimeNow);
    getRemoteStart();

    computeControl(TimeNow);
    MotorA.process(TimeNow);
    MotorB.process(TimeNow);
    Interface.Process();
    LED0.process(TimeNow);
    
}

void mini_sumo_s::computeControl(absolute_time_t _TimeNow)
{
    if (!Start || Stop)
    {
        return;
    }

    if (!StartTimedOut && _TimeNow < StartTimeout)
    {
        return;
    }

    StartTimedOut = true;
    int line_value;
    float VL;
    float VR;
    float Offset;
    auto [Roll, Pitch, Yaw] = IMU.getOrientation();
    yaw = Yaw * 180.0f / M_PI;
    
    line_value = lineRead();

    if (line_value == 0b111)
    {
        LED0.set(true);
    }
    else
    {
        LED0.set(false);
    }

    switch(line_value)
    {
        case 0b000:
        VL = Config.ForwardSpeed + (Config.TurnBias); VR = Config.ForwardSpeed + (1.0f - Config.TurnBias);
        //seek();
        break;
        case 0b100:
        VL = -Config.TurnPositive; VR = -Config.TurnNegative;
        break;
        case 0b001:
        VL = -Config.TurnPositive; VR = -Config.TurnNegative;
        break;
        case 0b010:
        case 0b101:
        case 0b110:
        case 0b011:
        case 0b111:
        VL = -Config.TurnPositive; VR = -Config.TurnNegative;
        break;
        default:
        VL = Config.ForwardSpeed + (Config.TurnBias); VR = Config.ForwardSpeed + (1.0f - Config.TurnBias);
        break;
    }

    // if (to_us_since_boot(_TimeNow) > to_us_since_boot(PrintTime))
    // {
    //     yaw = Yaw * 180.0f / M_PI;
    //     printf("ROLL:% .9f,PITCH:% .9f,YAW:% .9f\n", Roll * 180.0f / M_PI, Pitch * 180.0f / M_PI, yaw );
        
    //     PrintTime = make_timeout_time_ms(100);
    // }
    // Offset = PID0.compute(Position, _TimeNow);
    // VL = Config.ForwardSpeed;
    // VR = Config.ForwardSpeed;
    // if (Offset < 0.0f)
    // {
    //     VL += Offset * Config.TurnBias;
    //     VR -= Offset * (1.0f - Config.TurnBias);
    // }
    // else if (Offset > 0.0f)
    // {
    //     VL += Offset * (1.0f - Config.TurnBias);
    //     VR -= Offset * Config.TurnBias;
    // }
    setSpeed(VL, -VR);
}

void mini_sumo_s::getRemoteStart(void)
{
    if (gpio_get(STRT_PIN) && gpio_get(KILL_PIN) && !RemoteStateChange)
    {
        RemoteStateChange = true;
        if (!Start)
            start();
    }
    else if ((!gpio_get(STRT_PIN) || !gpio_get(KILL_PIN)) && RemoteStateChange)
    {
        RemoteStateChange = false;
        if (!Stop)
            stop();
    }
}

int mini_sumo_s::lineRead(void)
{
    int line_val = gpio_get(EDGE_SENSOR_LEFT_PIN) << 2;
    line_val |= gpio_get(EDGE_SENSOR_CENTER_PIN) << 1;
    line_val |= gpio_get(EDGE_SENSOR_RIGHT_PIN);
    return line_val;
}

// void vlx_init(SENSORX_VL53L0X_t *inst, uint8_t dev_num, int8_t port_ch, uint8_t dev_address)
// {
//     // I2C MUX init
//     if (SENSORX_VL53L0X_Init(inst, &i2c_port, port_ch, dev_address, dev_num) == SENSORX_ERROR) {
//         printf("sensor[%d] init: error!\n", dev_num);
//         return;
//     }
    
//     SENSORX_VL53L0X_GetConfig(inst);
//     SENSORX_VL53L0X_PrintAll(inst);
//     //inst->config.RANGING_MODE = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
//     //SENSORX_VL53L0X_SetConfig(inst);

//     //distance_th_high[dev_num] = 400;
//     //distance_th_low[dev_num] = 0;
// }

// void mini_sumo_s::vlx_multi_init(void)
// {
//     gpio_init(XSHUT_1_PIN);
//     gpio_set_dir(XSHUT_1_PIN, GPIO_IN);
//     gpio_init(XSHUT_2_PIN);
//     gpio_set_dir(XSHUT_2_PIN, GPIO_IN);
//     gpio_init(XSHUT_3_PIN);
//     gpio_set_dir(XSHUT_3_PIN, GPIO_IN);

//     gpio_put(XSHUT_1_PIN, false);
//     gpio_put(XSHUT_2_PIN, false);
//     gpio_put(XSHUT_3_PIN, false);

//     gpio_set_dir(XSHUT_1_PIN, GPIO_OUT);
//     gpio_set_dir(XSHUT_2_PIN, GPIO_OUT);
//     gpio_set_dir(XSHUT_3_PIN, GPIO_OUT);
//     //while(true);
//     int n = 0;
//     sleep_ms(100);
//     vlx_init(&vlx[0], 0, PORT_CHANNEL_NONE, 0x65); n++;
//     sleep_ms(100);
//     gpio_set_dir(XSHUT_1_PIN, GPIO_IN);
//     sleep_ms(100);
//     vlx_init(&vlx[1], 1, PORT_CHANNEL_NONE, 0x66); n++;
//     sleep_ms(100);
//     gpio_set_dir(XSHUT_2_PIN, GPIO_IN);
//     sleep_ms(100);
//     vlx_init(&vlx[2], 2, PORT_CHANNEL_NONE, 0x67); n++;
//     sleep_ms(100);
//     gpio_set_dir(XSHUT_3_PIN, GPIO_IN);
//     sleep_ms(100);
//     vlx_init(&vlx[3], 3, PORT_CHANNEL_NONE, 0x68); n++;
//     // if (port_i2c_device_select(&i2c_port, PORT_CHANNEL_0) == PORT_OK) {
//     //     printf("CH%d\n", PORT_CHANNEL_0);
//     // }
//     //vlx_init(&vlx[1], 1, PORT_CHANNEL_0, VL53L0X_DEFAULT_ADDRESS); n++;
//     // vlx_init(&vlx[2], 2, PORT_CHANNEL_1, VL53L0X_DEFAULT_ADDRESS); n++;
//     // vlx_init(&vlx[3], 3, PORT_CHANNEL_2, VL53L0X_DEFAULT_ADDRESS); n++;
//     // vlx_init(&vlx[4], 4, PORT_CHANNEL_3, VL53L0X_DEFAULT_ADDRESS); n++;
//     // vlx_init(&vlx[5], 5, PORT_CHANNEL_4, VL53L0X_DEFAULT_ADDRESS); n++;
//     // vlx_init(&vlx[6], 6, PORT_CHANNEL_5, VL53L0X_DEFAULT_ADDRESS); n++;
//     // vlx_init(&vlx[7], 7, PORT_CHANNEL_6, VL53L0X_DEFAULT_ADDRESS); n++;
//     // vlx_init(&vlx[8], 8, PORT_CHANNEL_7, VL53L0X_DEFAULT_ADDRESS);

//     // while(true) {
//     //     for (int8_t i = 0; i < NUM_SENSORS; i++) {
//     //         VL53L0X_RangingMeasurementData_t data;
//     //         SENSORX_VL53L0X_PerformSingleMeasurement(&vlx[i]);
//     //         SENSORX_VL53L0X_GetMeasurementData(&vlx[i], &data);
//     //         //SENSORX_VL53L0X_PrintMeasurementData(&vlx[i], data);
//     //         printf("sensor[%d] %d, ", vlx[i].device.device_enum, data.RangeMilliMeter);
//     //     }
//     //     printf("\n");
//     //     sleep_ms(250);
//     // }

// }
