/*
 * sensors.h
 *
 *  Created on: 21 Apr 2022
 *      Author: minda
 */

#include "string.h"
#include "stdlib.h"
#include "vl53l0x_api.h"
#include "port.h"
#ifndef INCLUDE_SENSORX_H_
#define INCLUDE_SENSORX_H_

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORX_ERROR -1
#define SENSORX_OK 0

typedef struct {
	VL53L0X_Dev_t device;
	VL53L0X_Config_t config;
} SENSORX_VL53L0X_t;

int SENSORX_VL53L0X_Init(SENSORX_VL53L0X_t *sensor, port_i2c_t *port_inst, int8_t port_channel, uint8_t device_address, uint8_t device_enum);
int SENSORX_VL53L0X_PerformSingleMeasurement(SENSORX_VL53L0X_t *sensor);
int SENSORX_VL53L0X_StartMeasurement(SENSORX_VL53L0X_t *sensor);
int SENSORX_VL53L0X_GetMeasurementData(SENSORX_VL53L0X_t *sensor, VL53L0X_RangingMeasurementData_t *data);
int SENSORX_VL53L0X_PrintMeasurementData(SENSORX_VL53L0X_t *sensor, VL53L0X_RangingMeasurementData_t data);
int SENSORX_VL53L0X_PollMeasurementDataReady(SENSORX_VL53L0X_t *sensor);
int SENSORX_VL53L0X_RunCalibrations(SENSORX_VL53L0X_t *sensor, uint32_t requestFlags);
int SENSORX_VL53L0X_GetConfig(SENSORX_VL53L0X_t *sensor);
int SENSORX_VL53L0X_SetConfig(SENSORX_VL53L0X_t *sensor);
int SENSORX_VL53L0X_PrintAll(SENSORX_VL53L0X_t *sensor);
int SENSORX_VL53L0X_ClearInterrupt(SENSORX_VL53L0X_t *sensor);
//
//uint8_t SENSORX_Start(SENSORx_t *device);
//uint8_t SENSORX_Stop(SENSORx_t *device);
//
//uint8_t SENSORX_Read(SENSORx_t *device);

int SENSORX_Delay(uint32_t ms);


#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_SENSORX_H_ */
