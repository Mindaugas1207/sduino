/**
 * 2022-11-01, Minduagas Mikalauskas.
 */

#include "main.hpp"

const uint8_t *nvm_start_address = (const uint8_t *) (XIP_BASE + NVM_ADDRESS_OFFSET);
struct nvm_data_s {
    uint lock_code;
    int previousState;
    float vars[VARS_COUNT];
    IMU_s::calib_t imu_calib;
    VL53L0X_Config_t vlx_cfg[NUM_SENSORS];
} nvm_data;

port_spi_t spi_port;
port_i2c_t i2c_port;

MotorControl_s Motors;
IMU_s IMU;
Servo_s Servo;

SENSORX_VL53L0X_t vlx[NUM_SENSORS];
VL53L0X_RangingMeasurementData_t vlx_data[NUM_SENSORS];
uint16_t distance[NUM_SENSORS];
uint16_t distance_th_low[NUM_SENSORS];
uint16_t distance_th_high[NUM_SENSORS];
bool detected[NUM_SENSORS];

bool mStart = false;
bool mStop = false;

bool vlx_cfg_ok = false;

//#define XSHUT_1 29
#define XSHUT_1 23
#define XSHUT_2 24
#define XSHUT_3 25

void vlx_init(SENSORX_VL53L0X_t *inst, uint8_t dev_num, int8_t port_ch, uint8_t dev_address)
{
    // I2C MUX init
    if (SENSORX_VL53L0X_Init(inst, &i2c_port, port_ch, dev_address, dev_num) == SENSORX_ERROR) {
        printf("sensor[%d] init: error!\n", dev_num);
        return;
    }
    
    //SENSORX_VL53L0X_GetConfig(inst);
    //SENSORX_VL53L0X_PrintAll(inst);
    //inst->config.RANGING_MODE = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
    //SENSORX_VL53L0X_SetConfig(inst);

    distance_th_high[dev_num] = 400;
    distance_th_low[dev_num] = 0;
}

void vlx_configure(SENSORX_VL53L0X_t *inst)
{
    SENSORX_VL53L0X_RunCalibrations(inst, VL53L0X_CALIBRATE_SPADS);
    SENSORX_VL53L0X_RunCalibrations(inst, VL53L0X_CALIBRATE_REF);
    SENSORX_VL53L0X_GetConfig(inst);
    inst->config.RANGING_MODE = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
    SENSORX_VL53L0X_SetConfig(inst);
}

void vlx_configure_all()
{
    

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        vlx_configure(&vlx[i]);
    }
    vlx_cfg_ok = true;
}

void vlx_multi_init() 
{
    gpio_init(XSHUT_1);
    gpio_set_dir(XSHUT_1, GPIO_IN);
    gpio_init(XSHUT_2);
    gpio_set_dir(XSHUT_2, GPIO_IN);
    gpio_init(XSHUT_3);
    gpio_set_dir(XSHUT_3, GPIO_IN);

    gpio_put(XSHUT_1, false);
    gpio_put(XSHUT_2, false);
    gpio_put(XSHUT_3, false);

    gpio_set_dir(XSHUT_1, GPIO_OUT);
    gpio_set_dir(XSHUT_2, GPIO_OUT);
    gpio_set_dir(XSHUT_3, GPIO_OUT);
    //while(true);
    int n = 0;
    sleep_ms(100);
    vlx_init(&vlx[0], 0, PORT_CHANNEL_ANY, 0x65); n++;
    sleep_ms(100);
    gpio_set_dir(XSHUT_1, GPIO_IN);
    sleep_ms(100);
    vlx_init(&vlx[1], 1, PORT_CHANNEL_ANY, 0x66); n++;
    sleep_ms(100);
    gpio_set_dir(XSHUT_2, GPIO_IN);
    sleep_ms(100);
    vlx_init(&vlx[2], 2, PORT_CHANNEL_ANY, 0x67); n++;
    sleep_ms(100);
    gpio_set_dir(XSHUT_3, GPIO_IN);
    sleep_ms(100);
    vlx_init(&vlx[3], 3, PORT_CHANNEL_ANY, 0x68); n++;
    // if (port_i2c_device_select(&i2c_port, PORT_CHANNEL_0) == PORT_OK) {
    //     printf("CH%d\n", PORT_CHANNEL_0);
    // }
    //vlx_init(&vlx[1], 1, PORT_CHANNEL_0, VL53L0X_DEFAULT_ADDRESS); n++;
    // vlx_init(&vlx[2], 2, PORT_CHANNEL_1, VL53L0X_DEFAULT_ADDRESS); n++;
    // vlx_init(&vlx[3], 3, PORT_CHANNEL_2, VL53L0X_DEFAULT_ADDRESS); n++;
    // vlx_init(&vlx[4], 4, PORT_CHANNEL_3, VL53L0X_DEFAULT_ADDRESS); n++;
    // vlx_init(&vlx[5], 5, PORT_CHANNEL_4, VL53L0X_DEFAULT_ADDRESS); n++;
    // vlx_init(&vlx[6], 6, PORT_CHANNEL_5, VL53L0X_DEFAULT_ADDRESS); n++;
    // vlx_init(&vlx[7], 7, PORT_CHANNEL_6, VL53L0X_DEFAULT_ADDRESS); n++;
    // vlx_init(&vlx[8], 8, PORT_CHANNEL_7, VL53L0X_DEFAULT_ADDRESS);

    // while(true) {
    //     for (int8_t i = 0; i < NUM_SENSORS; i++) {
    //         VL53L0X_RangingMeasurementData_t data;
    //         SENSORX_VL53L0X_PerformSingleMeasurement(&vlx[i]);
    //         SENSORX_VL53L0X_GetMeasurementData(&vlx[i], &data);
    //         //SENSORX_VL53L0X_PrintMeasurementData(&vlx[i], data);
    //         printf("sensor[%d] %d, ", vlx[i].device.device_enum, data.RangeMilliMeter);
    //     }
    //     printf("\n");
    //     sleep_ms(250);
    // }

}

void vlx_multi_start()
{
    if (vlx_cfg_ok)
    {
        for (int8_t i = 0; i < NUM_SENSORS; i++)
        {
            if (SENSORX_VL53L0X_StartMeasurement(&vlx[i]) == SENSORX_ERROR)
                printf("sensor[%d] start: error!\n", i);
            distance[i] = 0;
        }
    }
}

void vlx_read_multi() {
    if (vlx_cfg_ok)
    {
        for (int8_t i = 0; i < NUM_SENSORS; i++)
        {
            if (SENSORX_VL53L0X_PollMeasurementDataReady(&vlx[i]) == 1) {
                SENSORX_VL53L0X_GetMeasurementData(&vlx[i], &vlx_data[i]);
                //printf("[%d]=%d  ", i, vlx_data[i].RangeMilliMeter);
                detected[i] = ((vlx_data[i].RangeMilliMeter < distance_th_high[i]) && (vlx_data[i].RangeMilliMeter > distance_th_low[i])) ? true : false;
                printf("[%d]=%s\n",i, detected[i] ? true : false);
            }
            
        }
        printf("\n");
    }
}

void seek();

int main() {
    //vlx_multi_init();
    
    loadCalibration();
    
    

    // while (true) {
    //     printf("%d, %d, %d\n", gpio_get(EDGE_SENSOR_LEFT_PIN), gpio_get(EDGE_SENSOR_CENTER_PIN), gpio_get(EDGE_SENSOR_RIGHT_PIN));
    //     sleep_ms(300);
    // }
    //Motors.setSpeed(0.4, -0.4);
    // Servo.setAngle(0.0f);
    sleep_ms(1500);
    // Servo.setAngle(90.0f);
    // sleep_ms(1500);
    // Servo.setAngle(180.0f);
    // sleep_ms(1500);

    //Motors.setSpeed(1.0, -1.0);
    //vlx_configure_all();
    //vlx_multi_start();

    int line_value;
    int vlx_value;

    while (true)
    {
        
        

        line_value = lineRead();

        if (line_value == 0b111)
        {
            gpio_put(SDUINO_INTERNAL_LED_PIN, 1);
        }
        else
        {
            gpio_put(SDUINO_INTERNAL_LED_PIN, 0);
        }

        //vlx_read_multi();
        //sleep_ms(100);
        if (isStart())
        {
            switch(line_value)
            {
                case 0b000:
                Motors.setSpeed(0.6f, 0.6f);
                //seek();
                break;
                case 0b100:
                Motors.setSpeed(0.0f, 0.0f);
                sleep_ms(10);
                Motors.setSpeed(-0.6f, -0.6f);
                sleep_ms(60);
                Motors.setSpeed(-0.6f, 0.6f);
                sleep_ms(220);
                Motors.setSpeed(0.0f, 0.0f);
                break;
                case 0b001:
                Motors.setSpeed(0.0f, 0.0f);
                sleep_ms(10);
                Motors.setSpeed(-0.6f, -0.6f);
                sleep_ms(60);
                Motors.setSpeed(0.6f, -0.6f);
                sleep_ms(220);
                Motors.setSpeed(0.0f, 0.0f);
                break;
                case 0b010:
                case 0b101:
                case 0b110:
                case 0b011:
                case 0b111:
                Motors.setSpeed(0.0f, 0.0f);
                sleep_ms(10);
                Motors.setSpeed(-0.6f, -0.6f);
                sleep_ms(200);
                Motors.setSpeed(-0.6f, 0.6f);
                sleep_ms(300);
                Motors.setSpeed(0.0f, 0.0f);
                break;
                default:
                break;
            }
            //fight
            
            // uint8_t EDGE_CODE = (gpio_get(EDGE_SENSOR_LEFT_PIN) << 2) | (gpio_get(EDGE_SENSOR_CENTER_PIN) << 1) | (gpio_get(EDGE_SENSOR_RIGHT_PIN) << 0);
            // switch (EDGE_CODE) {
            //     case 0b000:
            //     //do tactic
            //     seek();
            //     break;
            //     case 0b010:
            //     drive_ms(-0.8, -0.8, 500);
            //     case 0b111:
            //     case 0b101:
            //     //go back
            //     drive_ms(0.0, 0.0, 50);
            //     drive_ms(0.5,-0.5, 250);
            //     drive_ms(0.0, 0.0, 100);
            //     break;
            //     case 0b001:
            //     case 0b011:
            //     drive_ms(-0.8, -0.8, 500);
            //     //go back-left
            //     drive_ms(0.0, 0.0, 50);
            //     drive_ms(-0.5,0.5, 250);
            //     drive_ms(0.0, 0.0, 100);
            //     break;
            //     case 0b100:
            //     case 0b110:
            //     drive_ms(-0.8, -0.8, 500);
            //     //go back right
            //     drive_ms(0.0, 0.0, 50);
            //     drive_ms(0.5,-0.5, 250);
            //     drive_ms(0.0, 0.0, 100);
            //     break;
            //     default:
            //     //stop?
            //     break;
            // }

            //sleep_ms(10);
        }
        else
        {
            Motors.setSpeed(0.0f, 0.0f);
        }



        serialProcess();
        
    }
    return 0;
}

void seek() {
    vlx_read_multi();
    uint8_t DET_CODE =  (detected[2] << 4) | (detected[3] << 3) | (detected[4] << 2) |
                        (detected[5] << 1) | (detected[6] << 0); //[2-6]
    //printf("%d\n", DET_CODE);
    switch (DET_CODE) {
        case 0b00000:
        if (detected[0])
            //turn left 60*
            drive_ms(-0.5, 0.5, 80);
        else if (detected[8])
            //turn right 60*
            drive_ms(-0.5, 0.5, 80);
        else if (detected[1])
            //turn left 130*
            drive_ms(-0.5, 0.5, 200);
        else if (detected[7])
            //turn right 130*
            drive_ms(0.5, -0.5, 200);
        else {
            drive_ms(1.0, 1.0, 0);
        }
        //search
        break;
        case 0b00100:
        case 0b01010:
        case 0b01110:
        case 0b11111:
        case 0b10101:
        case 0b11011:
        case 0b10001:
        case 0b00111:
        case 0b11100:
        case 0b10111:
        case 0b11101:
        case 0b01111:
        case 0b11110:
        case 0b00011:
        case 0b11000:
        case 0b11001:
        case 0b10011:
        //forward 0*
        drive_ms(1.0, 1.0, 20);
        break;
        case 0b00101:
        case 0b01100:
        case 0b01101:
        case 0b01001:
        case 0b01011:
        //forward right 1*
        drive_ms(0.4, 0.5, 15);
        break;
        case 0b00001:
        //arc right 1*
        drive_ms(0.3, 0.5, 15);
        break;
        case 0b01000:
        //arc right 30*
        drive_ms(0.1, 0.5, 30);
        break;
        case 0b10100:
        case 0b00110:
        case 0b10110:
        case 0b10010:
        case 0b11010:
        //forward left 1*
        drive_ms(0.5, 0.4, 15);
        break;
        case 0b10000:
        //arc left 1*
        drive_ms(0.5, 0.3, 15);
        break;
        case 0b00010:
        //arc left 30*
        drive_ms(0.5, 0.1, 30);
        break;
        default:
        //stop?
        break;
    }
}

void saveCalibration()
{
    nvm_data.imu_calib = IMU.getCalibration();
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        SENSORX_VL53L0X_GetConfig(&vlx[i]);
        nvm_data.vlx_cfg[i] = vlx[i].config;
    }
    nvm_data.lock_code = 0xFBCD14BC;
    nvmProgram();
}

void loadCalibration()
{
    nvmLoad();
    if (nvm_data.lock_code == 0xFBCD12BC)
    {
        IMU.loadCalibration(nvm_data.imu_calib);
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            vlx[i].config = nvm_data.vlx_cfg[i];
            SENSORX_VL53L0X_SetConfig(&vlx[i]);
            
        }
        vlx_cfg_ok = true;
    }
}