/*
 * sensorx_vl53l0x.c
 *
 *  Created on: Oct 15, 2022
 *      Author: minda
 */

#include "sensorx.h"

#define SENSORX_DEBUG

int SENSORX_VL53L0X_Init(SENSORX_VL53L0X_t *sensor, port_device_t *port_device, uint8_t device_enum)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	device->port_device = port_device;
	device->device_enum = device_enum;
	
	// if (port_i2c_device_check(port_inst, port_channel, device_address))
	// 	device->device_address = device_address;
	// else if (port_i2c_device_check(port_inst, port_channel, VL53L0X_DEFAULT_ADDRESS))
	device->port_device->hw_address = VL53L0X_DEFAULT_ADDRESS;
	// else
	// 	return SENSORX_ERROR;

	Status |= VL53L0X_DataInit(device);
	Status |= VL53L0X_StaticInit(device);
	if (device->port_device->hw_address != port_device->hw_address)
	{
		Status |= VL53L0X_SetDeviceAddress(device, port_device->hw_address << 1);
		device->port_device->hw_address = port_device->hw_address;
	}
	
	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}

int SENSORX_VL53L0X_StartMeasurement(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status |= VL53L0X_StartMeasurement(device);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}

int SENSORX_VL53L0X_StopMeasurement(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status |= VL53L0X_StopMeasurement(device);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}

int SENSORX_VL53L0X_GetMeasurementData(SENSORX_VL53L0X_t *sensor, VL53L0X_RangingMeasurementData_t *data)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	Status = VL53L0X_GetRangingMeasurementData(device, data);
	
	if (Status != VL53L0X_ERROR_NONE) {
		char StatusString[VL53L0X_MAX_STRING_LENGTH];
		VL53L0X_get_device_error_string(Status, StatusString);
		printf("%s\n", StatusString);
		return SENSORX_ERROR;
	} 
	return SENSORX_OK;
}

int SENSORX_VL53L0X_PrintMeasurementData(SENSORX_VL53L0X_t *sensor, VL53L0X_RangingMeasurementData_t data)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	char StatusString[VL53L0X_MAX_STRING_LENGTH];
	Status = VL53L0X_GetRangeStatusString(data.RangeStatus, StatusString);

	printf(
		"Device[%d]:\n"
		"\n.......MEASUREMENT.......................\n"
		"TIMESTAMP: %lu\n"
		"MEASUREMENT TIME: %lu us"
		"RANGE: %u mm\n"
		"RANGE DMAX: %u mm\n"
		"SIGNAL RATE: %0.2f Mcps\n"
		"AMBIANT RATE: %0.2f Mcps\n"
		"EFFECTIVE SPAD COUNT: %0.2f\n"
		"ZONE ID: %u\n"
		"RANGE FRACTIONAL PART: %u\n"
		"RANGE STATUS: %s\n"
		"\n.........................................\n"
		, device->device_enum
		, data.TimeStamp
		, data.MeasurementTimeUsec
		, data.RangeMilliMeter
		, data.RangeDMaxMilliMeter
		, fixPoint1616_to_float(data.SignalRateRtnMegaCps)
		, fixPoint1616_to_float(data.AmbientRateRtnMegaCps)
		, (float)data.EffectiveSpadRtnCount / 256.0f
		, data.ZoneId
		, data.RangeFractionalPart
		, StatusString
	);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}

int SENSORX_VL53L0X_PollInterrupt(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint32_t IntStatus;

	Status = VL53L0X_GetInterruptMaskStatus(device, &IntStatus);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return IntStatus;
}

int SENSORX_VL53L0X_ClearInterrupt(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_ClearInterruptMask(device, 0);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}

int SENSORX_VL53L0X_PerformSingleMeasurement(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	Status = VL53L0X_PerformSingleMeasurement(device);
	
	if (Status != VL53L0X_ERROR_NONE) {
		char StatusString[VL53L0X_MAX_STRING_LENGTH];
		VL53L0X_get_device_error_string(Status, StatusString);
		printf("%s\n", StatusString);
		return SENSORX_ERROR;
	}
	return SENSORX_OK;
}

int SENSORX_VL53L0X_PollMeasurementStop(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint32_t StopStatus;

	Status |= VL53L0X_GetStopCompletedStatus(device, &StopStatus);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return StopStatus;
}

int SENSORX_VL53L0X_PollMeasurementDataReady(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t DataReady;

	Status |= VL53L0X_GetMeasurementDataReady(device, &DataReady);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return DataReady;
}

int SENSORX_VL53L0X_RunCalibrations(SENSORX_VL53L0X_t *sensor, uint32_t calibrationNumber)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Config_t *config = &(sensor->config);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	switch (calibrationNumber)
	{
		case VL53L0X_CALIBRATE_SPADS:
		Status |= VL53L0X_PerformRefSpadManagement(device, &config->REFERANCE_SPADS.COUNT, &config->REFERANCE_SPADS.APERATURE);
		break;
		case VL53L0X_CALIBRATE_REF:
		Status |= VL53L0X_PerformRefCalibration(device, &config->REFERANCE_CALIBRATION.VHV, &config->REFERANCE_CALIBRATION.PHASE);
		break;
		case VL53L0X_CALIBRATE_OFFSET:
		SENSORX_Delay(VL53L0X_OFFSET_CALIBRATION_DELAY_S * 1000);
		Status |= VL53L0X_PerformOffsetCalibration(device, config->OFFSET_COMPENSATION.DISTANCE, &config->OFFSET_COMPENSATION.DATA);
		break;
		case VL53L0X_CALIBRATE_XTALK:
		SENSORX_Delay(VL53L0X_XTALK_CALIBRATION_DELAY_S * 1000);
		Status |= VL53L0X_PerformXTalkCalibration(device, config->XTALK_COMPENSATION.DISTANCE, &config->XTALK_COMPENSATION.RATE);
		break;
		default:
		break;
	}

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}

int SENSORX_VL53L0X_SetConfig(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Config_t *config = &(sensor->config);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status |= VL53L0X_SetDeviceMode(device, config->RANGING_MODE);
	Status |= VL53L0X_SetInterMeasurementPeriodMilliSeconds(device, config->INTER_MEASUREMENT_PERIOD);
	Status |= VL53L0X_SetMeasurementTimingBudgetMicroSeconds(device, config->MEASUREMENT_TIMING_BUDGET);
	Status |= VL53L0X_SetLinearityCorrectiveGain(device, config->LINEARITY_CORRECTIVE_GAIN);
	Status |= VL53L0X_SetRangeFractionEnable(device, config->FRACTION_ENABLE);
	Status |= VL53L0X_SetVcselPulsePeriod(device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, config->VCSEL_PERIOD_PRE_RANGE);
	Status |= VL53L0X_SetVcselPulsePeriod(device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, config->VCSEL_PERIOD_FINAL_RANGE);
	Status |= VL53L0X_SetReferenceSpads(device, config->REFERANCE_SPADS.COUNT, config->REFERANCE_SPADS.APERATURE);
	Status |= VL53L0X_SetRefCalibration(device, config->REFERANCE_CALIBRATION.VHV, config->REFERANCE_CALIBRATION.PHASE);
	Status |= VL53L0X_SetOffsetCalibrationDataMicroMeter(device, config->OFFSET_COMPENSATION.DATA);
	Status |= VL53L0X_SetXTalkCompensationEnable(device, config->XTALK_COMPENSATION.ENABLE);
	Status |= VL53L0X_SetXTalkCompensationRateMegaCps(device, config->XTALK_COMPENSATION.RATE);
	Status |= VL53L0X_SetGpioConfig(device, 0, config->RANGING_MODE, config->INTERRUPT.MODE, config->INTERRUPT.POLARITY);
	Status |= VL53L0X_SetInterruptThresholds(device, VL53L0X_DEVICEMODE_SINGLE_RANGING, config->INTERRUPT.THRESHOLD_LOW, config->INTERRUPT.THRESHOLD_HIGH);
	Status |= VL53L0X_SetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_TCC, config->SEQUENCESTEP_ENABLE.TCC);
	Status |= VL53L0X_SetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_DSS, config->SEQUENCESTEP_ENABLE.DSS);
	Status |= VL53L0X_SetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_MSRC, config->SEQUENCESTEP_ENABLE.MSRC);
	Status |= VL53L0X_SetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_PRE_RANGE, config->SEQUENCESTEP_ENABLE.PRE_RANGE);
	Status |= VL53L0X_SetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_FINAL_RANGE, config->SEQUENCESTEP_ENABLE.FINAL_RANGE);
	Status |= VL53L0X_SetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_TCC, config->SEQUENCESTEP_TIMEOUT.TCC);
	Status |= VL53L0X_SetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_DSS, config->SEQUENCESTEP_TIMEOUT.DSS);
	Status |= VL53L0X_SetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_MSRC, config->SEQUENCESTEP_TIMEOUT.MSRC);
	Status |= VL53L0X_SetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_PRE_RANGE, config->SEQUENCESTEP_TIMEOUT.PRE_RANGE);
	Status |= VL53L0X_SetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_FINAL_RANGE, config->SEQUENCESTEP_TIMEOUT.FINAL_RANGE);
	Status |= VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, config->LIMITCHECK_VALUE.SIGMA_FINAL_RANGE);
	Status |= VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, config->LIMITCHECK_VALUE.SIGNAL_RATE_FINAL_RANGE);
	Status |= VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, config->LIMITCHECK_VALUE.SIGNAL_REF_CLIP);
	Status |= VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, config->LIMITCHECK_VALUE.RANGE_IGNORE_THRESHOLD);
	Status |= VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, config->LIMITCHECK_VALUE.SIGNAL_RATE_MSRC);
	Status |= VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, config->LIMITCHECK_VALUE.SIGNAL_RATE_PRE_RANGE);
	Status |= VL53L0X_SetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, config->LIMITCHECK_ENABLE.SIGMA_FINAL_RANGE);
	Status |= VL53L0X_SetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, config->LIMITCHECK_ENABLE.SIGNAL_RATE_FINAL_RANGE);
	Status |= VL53L0X_SetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, config->LIMITCHECK_ENABLE.SIGNAL_REF_CLIP);
	Status |= VL53L0X_SetLimitCheckEnable(device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, config->LIMITCHECK_ENABLE.RANGE_IGNORE_THRESHOLD);
	Status |= VL53L0X_SetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, config->LIMITCHECK_ENABLE.SIGNAL_RATE_MSRC);
	Status |= VL53L0X_SetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, config->LIMITCHECK_ENABLE.SIGNAL_RATE_PRE_RANGE);
	Status |= VL53L0X_SetWrapAroundCheckEnable(device, config->WRAP_ARAUND_CHECK);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}

int SENSORX_VL53L0X_GetConfig(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Config_t *config = &(sensor->config);
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status |= VL53L0X_GetDeviceMode(device, &config->RANGING_MODE);
	Status |= VL53L0X_GetInterMeasurementPeriodMilliSeconds(device, &config->INTER_MEASUREMENT_PERIOD);
	Status |= VL53L0X_GetMeasurementTimingBudgetMicroSeconds(device, &config->MEASUREMENT_TIMING_BUDGET);
	Status |= VL53L0X_GetLinearityCorrectiveGain(device, &config->LINEARITY_CORRECTIVE_GAIN);
	Status |= VL53L0X_GetFractionEnable(device, &config->FRACTION_ENABLE);
	Status |= VL53L0X_GetVcselPulsePeriod(device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &config->VCSEL_PERIOD_PRE_RANGE);
	Status |= VL53L0X_GetVcselPulsePeriod(device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &config->VCSEL_PERIOD_FINAL_RANGE);
	Status |= VL53L0X_GetReferenceSpads(device, &config->REFERANCE_SPADS.COUNT, &config->REFERANCE_SPADS.APERATURE);
	Status |= VL53L0X_GetRefCalibration(device, &config->REFERANCE_CALIBRATION.VHV, &config->REFERANCE_CALIBRATION.PHASE);
	Status |= VL53L0X_GetOffsetCalibrationDataMicroMeter(device, &config->OFFSET_COMPENSATION.DATA);
	Status |= VL53L0X_GetXTalkCompensationEnable(device, &config->XTALK_COMPENSATION.ENABLE);
	Status |= VL53L0X_GetXTalkCompensationRateMegaCps(device, &config->XTALK_COMPENSATION.RATE);
	Status |= VL53L0X_GetGpioConfig(device, 0, &config->RANGING_MODE, &config->INTERRUPT.MODE, &config->INTERRUPT.POLARITY);
	Status |= VL53L0X_GetInterruptThresholds(device, VL53L0X_DEVICEMODE_SINGLE_RANGING, &config->INTERRUPT.THRESHOLD_LOW, &config->INTERRUPT.THRESHOLD_HIGH);
	Status |= VL53L0X_GetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_TCC, &config->SEQUENCESTEP_ENABLE.TCC);
	Status |= VL53L0X_GetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_DSS, &config->SEQUENCESTEP_ENABLE.DSS);
	Status |= VL53L0X_GetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_MSRC, &config->SEQUENCESTEP_ENABLE.MSRC);
	Status |= VL53L0X_GetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_PRE_RANGE, &config->SEQUENCESTEP_ENABLE.PRE_RANGE);
	Status |= VL53L0X_GetSequenceStepEnable(device, VL53L0X_SEQUENCESTEP_FINAL_RANGE, &config->SEQUENCESTEP_ENABLE.FINAL_RANGE);
	Status |= VL53L0X_GetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_TCC, &config->SEQUENCESTEP_TIMEOUT.TCC);
	Status |= VL53L0X_GetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_DSS, &config->SEQUENCESTEP_TIMEOUT.DSS);
	Status |= VL53L0X_GetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_MSRC, &config->SEQUENCESTEP_TIMEOUT.MSRC);
	Status |= VL53L0X_GetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_PRE_RANGE, &config->SEQUENCESTEP_TIMEOUT.PRE_RANGE);
	Status |= VL53L0X_GetSequenceStepTimeout(device, VL53L0X_SEQUENCESTEP_FINAL_RANGE, &config->SEQUENCESTEP_TIMEOUT.FINAL_RANGE);
	Status |= VL53L0X_GetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, &config->LIMITCHECK_VALUE.SIGMA_FINAL_RANGE);
	Status |= VL53L0X_GetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, &config->LIMITCHECK_VALUE.SIGNAL_RATE_FINAL_RANGE);
	Status |= VL53L0X_GetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, &config->LIMITCHECK_VALUE.SIGNAL_REF_CLIP);
	Status |= VL53L0X_GetLimitCheckValue(device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &config->LIMITCHECK_VALUE.RANGE_IGNORE_THRESHOLD);
	Status |= VL53L0X_GetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, &config->LIMITCHECK_VALUE.SIGNAL_RATE_MSRC);
	Status |= VL53L0X_GetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, &config->LIMITCHECK_VALUE.SIGNAL_RATE_PRE_RANGE);
	Status |= VL53L0X_GetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, &config->LIMITCHECK_ENABLE.SIGMA_FINAL_RANGE);
	Status |= VL53L0X_GetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, &config->LIMITCHECK_ENABLE.SIGNAL_RATE_FINAL_RANGE);
	Status |= VL53L0X_GetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, &config->LIMITCHECK_ENABLE.SIGNAL_REF_CLIP);
	Status |= VL53L0X_GetLimitCheckEnable(device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &config->LIMITCHECK_ENABLE.RANGE_IGNORE_THRESHOLD);
	Status |= VL53L0X_GetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, &config->LIMITCHECK_ENABLE.SIGNAL_RATE_MSRC);
	Status |= VL53L0X_GetLimitCheckEnable(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, &config->LIMITCHECK_ENABLE.SIGNAL_RATE_PRE_RANGE);
	Status |= VL53L0X_GetWrapAroundCheckEnable(device, &config->WRAP_ARAUND_CHECK);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}



int SENSORX_VL53L0X_PrintAll(SENSORX_VL53L0X_t *sensor)
{
	VL53L0X_Dev_t *device = &(sensor->device);
	VL53L0X_Config_t *config = &(sensor->config);
	VL53L0X_DeviceInfo_t DeviceInfo;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status |= VL53L0X_GetDeviceInfo(device, &DeviceInfo);

	Status |= SENSORX_VL53L0X_GetConfig(sensor);

	printf(
		"Device[%d]: Name:%s, Type:%s\n"
		"ID: %s, Revision:%d.%d\n"
		"\n.......CONFIG............................\n"
		"RANGING_MODE: %s\n"
		"INTER_MEASUREMENT_PERIOD: %lu ms\n"
		"MEASUREMENT_TIMING_BUDGET: %lu us\n"
		"LINEARITY_CORRECTIVE_GAIN: %u x1000\n"
		"FRACTION_ENABLE: %s\n"
		"VCSEL_PERIOD_PRE_RANGE: %u\n"
		"VCSEL_PERIOD_FINAL_RANGE: %u\n"
		"\n.......REFERANCE SPADS...................\n"
		"COUNT: %lu\n"
		"APERATURE: %u\n"
		"\n.......REFERANCE CALIBRATION.............\n"
		"VHV: %u\n"
		"PHASE: %u\n"
		"\n.......OFFSET COMPENSATION...............\n"
		"DISTANCE: %0.2f mm\n"
		"DATA: %ld um\n"
		"\n.......XTALK COMPENSATION................\n"
		"ENABLE: %s\n"
		"DISTANCE: %0.2f mm\n"
		"RATE: %0.2f Mcps\n"
		"\n.......INTERRUPT.........................\n"
		"MODE: %s\n"
		"POLARITY: %s\n"
		"THRESHOLD_LOW: %0.2f\n"
		"THRESHOLD_HIGH: %0.2f\n"
		"\n.......SEQUENCESTEP ENABLE...............\n"
		"TCC: %s\n"
		"DSS: %s\n"
		"MSRC: %s\n"
		"PRE_RANGE: %s\n"
		"FINAL_RANGE: %s\n"
		"\n.......SEQUENCESTEP TIMEOUT..............\n"
		"TCC: %0.2f\n"
		"DSS: %0.2f\n"
		"MSRC: %0.2f\n"
		"PRE_RANGE: %0.2f\n"
		"FINAL_RANGE: %0.2f\n"
		"\n.......LIMITCHECK ENABLE.................\n"
		"SIGMA_FINAL_RANGE: %s\n"
		"SIGNAL_RATE_FINAL_RANGE: %s\n"
		"SIGNAL_REF_CLIP: %s\n"
		"RANGE_IGNORE_THRESHOLD: %s\n"
		"SIGNAL_RATE_MSRC: %s\n"
		"SIGNAL_RATE_PRE_RANGE: %s\n"
		"WRAP_ARAUND_CHECK: %s\n"
		"\n.......LIMITCHECK VALUE..................\n"
		"SIGMA_FINAL_RANGE: %0.2f mm\n"
		"SIGNAL_RATE_FINAL_RANGE: %0.2f Mcps\n"
		"SIGNAL_REF_CLIP: %0.2f\n"
		"RANGE_IGNORE_THRESHOLD: %0.2f Mcps/SPAD\n"
		"SIGNAL_RATE_MSRC: %0.2f Mcps\n"
		"SIGNAL_RATE_PRE_RANGE: %0.2f Mcps\n"
		"\n.........................................\n"
		, device->device_enum, DeviceInfo.Name, DeviceInfo.Type
		, DeviceInfo.ProductId, DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor
		, VL53L0X_DeviceModeCodeToString(config->RANGING_MODE)
		, config->INTER_MEASUREMENT_PERIOD
		, config->MEASUREMENT_TIMING_BUDGET
		, config->LINEARITY_CORRECTIVE_GAIN
		, config->FRACTION_ENABLE ? "TRUE" : "FALSE"
		, config->VCSEL_PERIOD_PRE_RANGE
		, config->VCSEL_PERIOD_FINAL_RANGE
		, config->REFERANCE_SPADS.COUNT
		, config->REFERANCE_SPADS.APERATURE
		, config->REFERANCE_CALIBRATION.VHV
		, config->REFERANCE_CALIBRATION.PHASE
		, fixPoint1616_to_float(config->OFFSET_COMPENSATION.DISTANCE)
		, config->OFFSET_COMPENSATION.DATA
		, config->XTALK_COMPENSATION.ENABLE ? "TRUE" : "FALSE"
		, fixPoint1616_to_float(config->XTALK_COMPENSATION.DISTANCE)
		, fixPoint1616_to_float(config->XTALK_COMPENSATION.RATE)
		, VL53L0X_GpioFunctionalityCodeToString(config->INTERRUPT.MODE)
		, config->INTERRUPT.POLARITY ? "LOW" : "HIGH"
		, fixPoint1616_to_float(config->INTERRUPT.THRESHOLD_LOW)
		, fixPoint1616_to_float(config->INTERRUPT.THRESHOLD_HIGH)
		, config->SEQUENCESTEP_ENABLE.TCC ? "TRUE" : "FALSE"
		, config->SEQUENCESTEP_ENABLE.DSS ? "TRUE" : "FALSE"
		, config->SEQUENCESTEP_ENABLE.MSRC ? "TRUE" : "FALSE"
		, config->SEQUENCESTEP_ENABLE.PRE_RANGE ? "TRUE" : "FALSE"
		, config->SEQUENCESTEP_ENABLE.FINAL_RANGE ? "TRUE" : "FALSE"
		, fixPoint1616_to_float(config->SEQUENCESTEP_TIMEOUT.TCC)
		, fixPoint1616_to_float(config->SEQUENCESTEP_TIMEOUT.DSS)
		, fixPoint1616_to_float(config->SEQUENCESTEP_TIMEOUT.MSRC)
		, fixPoint1616_to_float(config->SEQUENCESTEP_TIMEOUT.PRE_RANGE)
		, fixPoint1616_to_float(config->SEQUENCESTEP_TIMEOUT.FINAL_RANGE)
		, config->LIMITCHECK_ENABLE.SIGMA_FINAL_RANGE ? "TRUE" : "FALSE"
		, config->LIMITCHECK_ENABLE.SIGNAL_RATE_FINAL_RANGE ? "TRUE" : "FALSE"
		, config->LIMITCHECK_ENABLE.SIGNAL_REF_CLIP ? "TRUE" : "FALSE"
		, config->LIMITCHECK_ENABLE.RANGE_IGNORE_THRESHOLD ? "TRUE" : "FALSE"
		, config->LIMITCHECK_ENABLE.SIGNAL_RATE_MSRC ? "TRUE" : "FALSE"
		, config->LIMITCHECK_ENABLE.SIGNAL_RATE_PRE_RANGE ? "TRUE" : "FALSE"
		, config->WRAP_ARAUND_CHECK ? "TRUE" : "FALSE"
		, fixPoint1616_to_float(config->LIMITCHECK_VALUE.SIGMA_FINAL_RANGE)
		, fixPoint1616_to_float(config->LIMITCHECK_VALUE.SIGNAL_RATE_FINAL_RANGE)
		, fixPoint1616_to_float(config->LIMITCHECK_VALUE.SIGNAL_REF_CLIP)
		, fixPoint1616_to_float(config->LIMITCHECK_VALUE.RANGE_IGNORE_THRESHOLD)
		, fixPoint1616_to_float(config->LIMITCHECK_VALUE.SIGNAL_RATE_MSRC)
		, fixPoint1616_to_float(config->LIMITCHECK_VALUE.SIGNAL_RATE_PRE_RANGE)
	);

	if (Status != VL53L0X_ERROR_NONE) return SENSORX_ERROR;
	return SENSORX_OK;
}





