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

int SENSORX_VL53L0X_Init(VL53L0X_Dev_t *sensor);
int SENSORX_VL53L0X_PerformSingleMeasurement(VL53L0X_Dev_t *device);
int SENSORX_VL53L0X_StartMeasurement(VL53L0X_Dev_t *device);
int SENSORX_VL53L0X_StopMeasurement(VL53L0X_Dev_t *device);
int SENSORX_VL53L0X_GetMeasurementData(VL53L0X_Dev_t *device, VL53L0X_RangingMeasurementData_t *data);
int SENSORX_VL53L0X_PrintMeasurementData(VL53L0X_Dev_t *device, VL53L0X_RangingMeasurementData_t data);
int SENSORX_VL53L0X_PollMeasurementDataReady(VL53L0X_Dev_t *device);
int SENSORX_VL53L0X_RunCalibrations(VL53L0X_Dev_t *device, VL53L0X_Config_t *config, uint32_t requestFlags);
int SENSORX_VL53L0X_GetConfig(VL53L0X_Dev_t *device, VL53L0X_Config_t *config);
int SENSORX_VL53L0X_SetConfig(VL53L0X_Dev_t *device, VL53L0X_Config_t *config);
int SENSORX_VL53L0X_PrintAll(VL53L0X_Dev_t *device, VL53L0X_Config_t *config);
int SENSORX_VL53L0X_ClearInterrupt(VL53L0X_Dev_t *device);
//
//uint8_t SENSORX_Start(SENSORx_t *device);
//uint8_t SENSORX_Stop(SENSORx_t *device);
//
//uint8_t SENSORX_Read(SENSORx_t *device);



#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_SENSORX_H_ */
