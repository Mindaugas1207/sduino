
#ifndef IMU_HW_H
#define IMU_HW_H

#include "pico/stdlib.h"
#include "port.h"
#include <math.h>
#include "bmi08x.h"
#include "bmi08x_defs.h"

#define IMU_HW_OK PICO_OK
#define IMU_HW_NO_DATA (PICO_OK + 1)
#define IMU_HW_ERROR PICO_ERROR_GENERIC

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    struct bmi08x_dev BMI08X_Device;
    uint Pin;
    double faccel;
    double fgyro;
} imu_hw_inst_t;

static bool bmi08x_hw_dataSync;

static void bmi08x_hw_callback(uint gpio, uint32_t events)
{
    bmi08x_hw_dataSync = true;
}

inline bool bmi08x_hw_new_data_available()
{
    return bmi08x_hw_dataSync || gpio_get(SDUINO_INTERNAL_IMU_INT_PIN);
}

static int bmi08x_GetConversionFactors(imu_hw_inst_t *inst);

static int imu_hw_init(imu_hw_inst_t *inst)
{
    int status = BMI08X_OK;

    struct bmi08x_dev *devptr = &inst->BMI08X_Device;

    devptr->intf = BMI08X_SPI_INTF;
    devptr->variant = BMI088_VARIANT;
    devptr->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    devptr->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
    devptr->read_write_len = 32;
    // interrupts
    devptr->accel_int_config_1 = {
        .int_channel = BMI08X_INT_CHANNEL_1,
        .int_type = BMI08X_ACCEL_SYNC_INPUT,
        .int_pin_cfg = {
            .lvl = BMI08X_INT_ACTIVE_HIGH,
            .output_mode = BMI08X_INT_MODE_PUSH_PULL,
            .enable_int_pin = BMI08X_ENABLE}};
    devptr->accel_int_config_2 = {
        .int_channel = BMI08X_INT_CHANNEL_2,
        .int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY,
        .int_pin_cfg = {
            .lvl = BMI08X_INT_ACTIVE_HIGH,
            .output_mode = BMI08X_INT_MODE_PUSH_PULL,
            .enable_int_pin = BMI08X_ENABLE}};
    devptr->gyro_int_config_1 = {
        .int_channel = BMI08X_INT_CHANNEL_3,
        .int_type = BMI08X_GYRO_INT_DATA_RDY,
        .int_pin_cfg = {
            .lvl = BMI08X_INT_ACTIVE_HIGH,
            .output_mode = BMI08X_INT_MODE_PUSH_PULL,
            .enable_int_pin = BMI08X_ENABLE}};
    devptr->gyro_int_config_2 = {
        .int_channel = BMI08X_INT_CHANNEL_4,
        .int_type = BMI08X_GYRO_INT_DATA_RDY,
        .int_pin_cfg = {
            .lvl = BMI08X_INT_ACTIVE_HIGH,
            .output_mode = BMI08X_INT_MODE_PUSH_PULL,
            .enable_int_pin = BMI08X_DISABLE}};

    status |= bmi08a_init(devptr);             // init accelerometer
    status |= bmi08g_init(devptr);             // init gyro
    status |= bmi08a_soft_reset(devptr);       // reset accelerometer
    status |= bmi08a_load_config_file(devptr); // load cfg file
    status |= bmi08a_set_power_mode(devptr);   // set accel power mode
    status |= bmi08g_set_power_mode(devptr);   // set gyro power mode
    status |= bmi08a_configure_data_synchronization(devptr);
    status |= bmi08a_set_data_sync_int_config(devptr);

    bmi08x_GetConversionFactors(inst);

    gpio_init(inst->Pin);
    gpio_set_dir(inst->Pin, GPIO_IN);
    gpio_set_irq_enabled_with_callback(inst->Pin, GPIO_IRQ_EDGE_RISE, true, &bmi08x_hw_callback);

    bmi08x_hw_dataSync = true;

    return status != BMI08X_OK ? IMU_HW_ERROR : IMU_HW_OK;
}

#define BMI08X_DUMY_OFFSET (1)
#define BMI08X_RD_LENGTH_1 (BMI08X_DUMY_OFFSET + 4)
#define BMI08X_RD_LENGTH_2 (BMI08X_DUMY_OFFSET + 2)
#define BMI08X_RD_LENGTH_3 (6)

#define BMI08X_RD_OFFSET_1 (BMI08X_DUMY_OFFSET)
#define BMI08X_RD_OFFSET_2 (BMI08X_DUMY_OFFSET + BMI08X_RD_LENGTH_1)
#define BMI08X_RD_OFFSET_3 (BMI08X_RD_LENGTH_1 + BMI08X_RD_LENGTH_2)

// #define BMI08X_GYRO_INT3_LVL_POS 0
// extern const uint8_t bmi08x_config_file[];

typedef struct
{
    struct
    {
        double X, Y, Z;
    } Gyroscope;

    struct
    {
        double X, Y, Z;
    } Accelerometer;
} imu_hw_data_t;

#define G_EARTH (9.81492)

#define BMI08X_ACCEL_FACT_3G (3 * G_EARTH / (1 << 15))
#define BMI08X_ACCEL_FACT_6G (6 * G_EARTH / (1 << 15))
#define BMI08X_ACCEL_FACT_12G (12 * G_EARTH / (1 << 15))
#define BMI08X_ACCEL_FACT_24G (24 * G_EARTH / (1 << 15))

#define BMI08X_GYRO_FACT_125 (125 * M_PI / (180 * (1 << 15)))
#define BMI08X_GYRO_FACT_250 (250 * M_PI / (180 * (1 << 15)))
#define BMI08X_GYRO_FACT_500 (500 * M_PI / (180 * (1 << 15)))
#define BMI08X_GYRO_FACT_1000 (1000 * M_PI / (180 * (1 << 15)))
#define BMI08X_GYRO_FACT_2000 (2000 * M_PI / (180 * (1 << 15)))

static int bmi08x_GetConversionFactors(imu_hw_inst_t *inst)
{
    switch (inst->BMI08X_Device.accel_cfg.range)
    {
    case BMI088_ACCEL_RANGE_3G:
        inst->faccel = BMI08X_ACCEL_FACT_3G;
        break;
    case BMI088_ACCEL_RANGE_6G:
        inst->faccel = BMI08X_ACCEL_FACT_6G;
        break;
    case BMI088_ACCEL_RANGE_12G:
        inst->faccel = BMI08X_ACCEL_FACT_12G;
        break;
    case BMI088_ACCEL_RANGE_24G:
    default:
        inst->faccel = BMI08X_ACCEL_FACT_24G;
        break;
    }

    switch (inst->BMI08X_Device.gyro_cfg.range)
    {
    case BMI08X_GYRO_RANGE_125_DPS:
        inst->fgyro = BMI08X_GYRO_FACT_125;
        break;
    case BMI08X_GYRO_RANGE_250_DPS:
        inst->fgyro = BMI08X_GYRO_FACT_250;
        break;
    case BMI08X_GYRO_RANGE_500_DPS:
        inst->fgyro = BMI08X_GYRO_FACT_500;
        break;
    case BMI08X_GYRO_RANGE_1000_DPS:
        inst->fgyro = BMI08X_GYRO_FACT_1000;
        break;
    case BMI08X_GYRO_RANGE_2000_DPS:
    default:
        inst->fgyro = BMI08X_GYRO_FACT_2000;
        break;
    }

    return BMI08X_OK;
}

static int imu_hw_read_data(imu_hw_inst_t *inst, imu_hw_data_t *result)
{
    if (!bmi08x_hw_dataSync)
        return IMU_HW_NO_DATA;

    bmi08x_hw_dataSync = false;
    uint8_t data[BMI08X_RD_LENGTH_1 + BMI08X_RD_LENGTH_2 + BMI08X_RD_LENGTH_3];
    int16_t iresult[6];

    port_read_mem(&inst->BMI08X_Device.intf_accel, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_GP_0, data, BMI08X_RD_LENGTH_1);
    port_read_mem(&inst->BMI08X_Device.intf_accel, BMI08X_SPI_RD_MASK | BMI08X_REG_ACCEL_GP_4, data + BMI08X_RD_LENGTH_1, BMI08X_RD_LENGTH_2);
    port_read_mem(&inst->BMI08X_Device.intf_gyro,  BMI08X_SPI_RD_MASK | BMI08X_REG_GYRO_X_LSB, data + BMI08X_RD_LENGTH_1 + BMI08X_RD_LENGTH_2, BMI08X_RD_LENGTH_3);

    iresult[0] = (data[BMI08X_RD_OFFSET_1 + 1] << 8) | data[BMI08X_RD_OFFSET_1 + 0];
    iresult[1] = (data[BMI08X_RD_OFFSET_1 + 3] << 8) | data[BMI08X_RD_OFFSET_1 + 2];
    iresult[2] = (data[BMI08X_RD_OFFSET_2 + 1] << 8) | data[BMI08X_RD_OFFSET_2 + 0];
    iresult[3] = (data[BMI08X_RD_OFFSET_3 + 1] << 8) | data[BMI08X_RD_OFFSET_3 + 0];
    iresult[4] = (data[BMI08X_RD_OFFSET_3 + 3] << 8) | data[BMI08X_RD_OFFSET_3 + 2];
    iresult[5] = (data[BMI08X_RD_OFFSET_3 + 5] << 8) | data[BMI08X_RD_OFFSET_3 + 4];

    result->Accelerometer.X = iresult[0] * inst->faccel;
    result->Accelerometer.Y = iresult[1] * inst->faccel;
    result->Accelerometer.Z = iresult[2] * inst->faccel;
    result->Gyroscope.X = iresult[3] * inst->fgyro;
    result->Gyroscope.Y = iresult[4] * inst->fgyro;
    result->Gyroscope.Z = iresult[5] * inst->fgyro;

    return IMU_HW_OK;
}

#ifdef __cplusplus
}
#endif

#endif
