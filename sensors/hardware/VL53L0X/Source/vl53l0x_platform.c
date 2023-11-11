/*******************************************************************************
Copyright � 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/**
 * @file VL53L0X_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */

#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

float fixPoint1616_to_float(FixPoint1616_t inp)
{
	return ((float)inp) / 65536.0f;
}

FixPoint1616_t float_to_fixPoint1616(float inp)
{
	return (FixPoint1616_t)(inp * 65536);
}

const char* VL53L0X_ErrorCodeToString(VL53L0X_Error code)
{
	const char * name = VL53L0X_STRING_UNKNOW_ERROR_CODE;
	switch(code)
	{
		case VL53L0X_ERROR_NONE:
			name = VL53L0X_STRING_ERROR_NONE;//"ERROR_NONE";
			break;
		case VL53L0X_ERROR_CALIBRATION_WARNING:
			name = VL53L0X_STRING_ERROR_CALIBRATION_WARNING;//"ERROR_CALIBRATION_WARNING";
			break;
		case VL53L0X_ERROR_MIN_CLIPPED:
			name = VL53L0X_STRING_ERROR_MIN_CLIPPED;//"ERROR_MIN_CLIPPED";
			break;
		case VL53L0X_ERROR_UNDEFINED:
			name = VL53L0X_STRING_ERROR_UNDEFINED;//"ERROR_UNDEFINED";
			break;
		case VL53L0X_ERROR_INVALID_PARAMS:
			name = VL53L0X_STRING_ERROR_INVALID_PARAMS;//"ERROR_INVALID_PARAMS";
			break;
		case VL53L0X_ERROR_NOT_SUPPORTED:
			name = VL53L0X_STRING_ERROR_NOT_SUPPORTED;//"ERROR_NOT_SUPPORTED";
			break;
		case VL53L0X_ERROR_RANGE_ERROR:
			name = VL53L0X_STRING_ERROR_RANGE_ERROR;//"ERROR_RANGE_ERROR";
			break;
		case VL53L0X_ERROR_TIME_OUT:
			name = VL53L0X_STRING_ERROR_TIME_OUT;//"ERROR_TIME_OUT";
			break;
		case VL53L0X_ERROR_MODE_NOT_SUPPORTED:
			name = VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED;//"ERROR_MODE_NOT_SUPPORTED";
			break;
		case VL53L0X_ERROR_BUFFER_TOO_SMALL:
			name = VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL;//"ERROR_BUFFER_TOO_SMALL";
			break;
		case VL53L0X_ERROR_GPIO_NOT_EXISTING:
			name = VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING;//"ERROR_GPIO_NOT_EXISTING";
			break;
		case VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED:
			name = VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;//"ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED";
			break;
		case VL53L0X_ERROR_INTERRUPT_NOT_CLEARED:
			name = VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED;//"ERROR_INTERRUPT_NOT_CLEARED";
			break;
		case VL53L0X_ERROR_CONTROL_INTERFACE:
			name = VL53L0X_STRING_ERROR_CONTROL_INTERFACE;//"ERROR_CONTROL_INTERFACE";
			break;
		case VL53L0X_ERROR_INVALID_COMMAND:
			name = VL53L0X_STRING_ERROR_INVALID_COMMAND;//"ERROR_INVALID_COMMAND";
			break;
		case VL53L0X_ERROR_DIVISION_BY_ZERO:
			name = VL53L0X_STRING_ERROR_DIVISION_BY_ZERO;//"ERROR_DIVISION_BY_ZERO";
			break;
		case VL53L0X_ERROR_REF_SPAD_INIT:
			name = VL53L0X_STRING_ERROR_REF_SPAD_INIT;//"ERROR_REF_SPAD_INIT";
			break;
		case VL53L0X_ERROR_NOT_IMPLEMENTED:
			name = VL53L0X_STRING_ERROR_NOT_IMPLEMENTED;//"ERROR_NOT_IMPLEMENTED";
			break;
		default:
			break;
	}
	return name;
}

const char* VL53L0X_DeviceModeCodeToString(uint16_t code)
{
	const char * name = VL53L0X_STRING_UNKNOW_ERROR_CODE;
	switch(code)
	{
		case VL53L0X_DEVICEMODE_SINGLE_RANGING:
			name = VL53L0X_STRING_DEVICEMODE_SINGLE_RANGING;
			break;
		case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
			name = VL53L0X_STRING_DEVICEMODE_CONTINUOUS_RANGING;
			break;
		case VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM:
			name = VL53L0X_STRING_DEVICEMODE_SINGLE_HISTOGRAM;
			break;
		case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
			name = VL53L0X_STRING_DEVICEMODE_CONTINUOUS_TIMED_RANGING;
			break;
		case VL53L0X_DEVICEMODE_SINGLE_ALS:
			name = VL53L0X_STRING_DEVICEMODE_SINGLE_ALS;
			break;
		case VL53L0X_DEVICEMODE_GPIO_DRIVE:
			name = VL53L0X_STRING_DEVICEMODE_GPIO_DRIVE;
			break;
		case VL53L0X_DEVICEMODE_GPIO_OSC:
			name = VL53L0X_STRING_DEVICEMODE_GPIO_OSC;
			break;
		default:
			break;
	}
	return name;
}

const char* VL53L0X_CheckEnableCodeToString(uint16_t code)
{
	const char * name = VL53L0X_STRING_UNKNOW_ERROR_CODE;
	switch(code)
	{
		case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
			name = VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE;
			break;
		case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
			name = VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE;
			break;
		case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
			name = VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP;
			break;
		case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
			name = VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD;
			break;
		case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
			name = VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_MSRC;
			break;
		case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
			name = VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_PRE_RANGE;
			break;
		default:
			break;
	}
	return name;
}

const char* VL53L0X_SequenceStepCodeToString(uint16_t code)
{
	const char * name = VL53L0X_STRING_UNKNOW_ERROR_CODE;
	switch(code)
	{
		case VL53L0X_SEQUENCESTEP_TCC:
			name = VL53L0X_STRING_SEQUENCESTEP_TCC;
			break;
		case VL53L0X_SEQUENCESTEP_DSS:
			name = VL53L0X_STRING_SEQUENCESTEP_DSS;
			break;
		case VL53L0X_SEQUENCESTEP_MSRC:
			name = VL53L0X_STRING_SEQUENCESTEP_MSRC;
			break;
		case VL53L0X_SEQUENCESTEP_PRE_RANGE:
			name = VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE;
			break;
		case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
			name = VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE;
			break;
		default:
			break;
	}
	return name;
}

const char* VL53L0X_RangeStatusCodeToString(uint16_t code)
{
	const char * name = VL53L0X_STRING_RANGESTATUS_NONE;
	switch(code)
	{
		case VL53L0X_RANGESTATUS_RANGEVALID:
			name = VL53L0X_STRING_RANGESTATUS_RANGEVALID;
			break;
		case VL53L0X_RANGESTATUS_SIGMA:
			name = VL53L0X_STRING_RANGESTATUS_SIGMA;
			break;
		case VL53L0X_RANGESTATUS_SIGNAL:
			name = VL53L0X_STRING_RANGESTATUS_SIGNAL;
			break;
		case VL53L0X_RANGESTATUS_MINRANGE:
			name = VL53L0X_STRING_RANGESTATUS_MINRANGE;
			break;
		case VL53L0X_RANGESTATUS_PHASE:
			name = VL53L0X_STRING_RANGESTATUS_PHASE;
			break;
		case VL53L0X_RANGESTATUS_HW:
			name = VL53L0X_STRING_RANGESTATUS_HW;
			break;
		default:
			break;
	}
	return name;
}

const char* VL53L0X_DeviceStateCodeToString(VL53L0X_State code)
{
	const char * name = VL53L0X_STRING_STATE_UNKNOWN;
	switch(code)
	{
		case VL53L0X_STATE_POWERDOWN:
			name = VL53L0X_STRING_STATE_POWERDOWN;
			break;
		case VL53L0X_STATE_WAIT_STATICINIT:
			name = VL53L0X_STRING_STATE_WAIT_STATICINIT;
			break;
		case VL53L0X_STATE_STANDBY:
			name = VL53L0X_STRING_STATE_STANDBY;
			break;
		case VL53L0X_STATE_IDLE:
			name = VL53L0X_STRING_STATE_IDLE;
			break;
		case VL53L0X_STATE_RUNNING:
			name = VL53L0X_STRING_STATE_RUNNING;
			break;
		case VL53L0X_STATE_UNKNOWN:
			name = VL53L0X_STRING_STATE_UNKNOWN;
			break;
		case VL53L0X_STATE_ERROR:
			name = VL53L0X_STRING_STATE_ERROR;
			break;
		default:
			break;
	}
	return name;
}

const char* VL53L0X_DeviceErrorCodeToString(VL53L0X_DeviceError code)
{
	const char * name = VL53L0X_STRING_DEVICEERROR_UNKNOWN;
	switch(code)
	{
		case VL53L0X_DEVICEERROR_NONE:
			name = VL53L0X_STRING_DEVICEERROR_NONE;
			break;
		case VL53L0X_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
			name = VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE;
			break;
		case VL53L0X_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
			name = VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE;
			break;
		case VL53L0X_DEVICEERROR_NOVHVVALUEFOUND:
			name = VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND;
			break;
		case VL53L0X_DEVICEERROR_MSRCNOTARGET:
			name = VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET;
			break;
		case VL53L0X_DEVICEERROR_SNRCHECK:
			name = VL53L0X_STRING_DEVICEERROR_SNRCHECK;
			break;
		case VL53L0X_DEVICEERROR_RANGEPHASECHECK:
			name = VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK;
			break;
		case VL53L0X_DEVICEERROR_SIGMATHRESHOLDCHECK:
			name = VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK;
			break;
		case VL53L0X_DEVICEERROR_TCC:
			name = VL53L0X_STRING_DEVICEERROR_TCC;
			break;
		case VL53L0X_DEVICEERROR_PHASECONSISTENCY:
			name = VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY;
			break;
		case VL53L0X_DEVICEERROR_MINCLIP:
			name = VL53L0X_STRING_DEVICEERROR_MINCLIP;
			break;
		case VL53L0X_DEVICEERROR_RANGECOMPLETE:
			name = VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE;
			break;
		case VL53L0X_DEVICEERROR_ALGOUNDERFLOW:
			name = VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW;
			break;
		case VL53L0X_DEVICEERROR_ALGOOVERFLOW:
			name = VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW;
			break;
		case VL53L0X_DEVICEERROR_RANGEIGNORETHRESHOLD:
			name = VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD;
			break;
		default:
			break;
	}
	return name;
}

const char* VL53L0X_GpioFunctionalityCodeToString(VL53L0X_GpioFunctionality code)
{
	const char * name = VL53L0X_STRING_UNKNOW_ERROR_CODE;
	switch(code)
	{
		case VL53L0X_GPIOFUNCTIONALITY_OFF:
			name = VL53L0X_STRING_GPIOFUNCTIONALITY_OFF;
			break;
		case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
			name = VL53L0X_STRING_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW;
			break;
		case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
			name = VL53L0X_STRING_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH;
			break;
		case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
			name = VL53L0X_STRING_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT;
			break;
		case VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY:
			name = VL53L0X_STRING_GPIOFUNCTIONALITY_NEW_MEASURE_READY;
			break;
		default:
			break;
	}
	return name;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pData, uint32_t count)
{
	int status = VL53L0X_ERROR_NONE;
	uint8_t buff[sizeof(index) + count];

	buff[0] = index;
	memcpy(&buff[1], pData, count);
	
	status |= port_write(Dev->port_device, buff, sizeof(buff));

	if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pData, uint32_t count)
{
	int status = VL53L0X_ERROR_NONE;

    status |= port_write(Dev->port_device, &index, sizeof(index));
	status |= port_read(Dev->port_device, pData, count);

	if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return VL53L0X_ERROR_NONE;
}


VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
	int status = VL53L0X_ERROR_NONE;
    uint8_t buff[] = {index, data};

    status |= port_write(Dev->port_device, buff, sizeof(buff));

	if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
	int status = VL53L0X_ERROR_NONE;
    uint8_t buff[] = {index, data >> 8, data};

    status |= port_write(Dev->port_device, buff, sizeof(buff));

    if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
	int status = VL53L0X_ERROR_NONE;
    uint8_t buff[] = {index, data >> 24, data >> 16, data >> 8, data};

    status |= port_write(Dev->port_device, buff, sizeof(buff));

	if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
	int status = VL53L0X_ERROR_NONE;
	uint8_t data;

    status |= port_write(Dev->port_device, &index, sizeof(index));
	status |= port_read(Dev->port_device, &data, sizeof(uint8_t));

	data = (data & AndData) | OrData;

	uint8_t buff[] = {index, data};
    status |= port_write(Dev->port_device, buff, sizeof(buff));

	if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
	int status = VL53L0X_ERROR_NONE;

    status |= port_write(Dev->port_device, &index, sizeof(index));
	status |= port_read(Dev->port_device, data, sizeof(uint8_t));

	if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
	int status = VL53L0X_ERROR_NONE;
    uint8_t buff[sizeof(uint16_t)];

    status |= port_write(Dev->port_device, &index, sizeof(index));
	status |= port_read(Dev->port_device, buff, sizeof(uint16_t));

    *data = ((uint16_t)buff[0] << 8) | (uint16_t)buff[1];

	if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
	int status = VL53L0X_ERROR_NONE;
    uint8_t buff[sizeof(uint32_t)];

    status |= port_write(Dev->port_device, &index, sizeof(index));
	status |= port_read(Dev->port_device, buff, sizeof(uint32_t));

	*data = ((uint32_t)buff[0] << 24) | ((uint32_t)buff[1] << 16) | ((uint32_t)buff[2] << 8) | (uint32_t)buff[3];

	if (status < PORT_OK) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
	port_delay(VL53L0X_POLLING_DELAY);
	return VL53L0X_ERROR_NONE;
}
