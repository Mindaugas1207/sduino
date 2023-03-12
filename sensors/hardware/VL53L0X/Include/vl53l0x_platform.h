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


#ifndef _VL53L0X_PLATFORM_H_
#define _VL53L0X_PLATFORM_H_

#include "vl53l0x_def.h"
#include "vl53l0x_platform_log.h"
#include "port.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file vl53l0x_platform.h
 *
 * @brief All end user OS/platform/application porting
 */

/**
 * @defgroup VL53L0X_platform_group VL53L0X Platform Functions
 * @brief    VL53L0X Platform Functions
 *  @{
 */

/**
 * @struct  VL53L0X_Dev_t
 * @brief    Generic PAL device type that does link between API and platform abstraction layer
 *
 */
#define VL53L0X_DEFAULT_ADDRESS 0x29

typedef struct {
    VL53L0X_DevData_t Data;               /*!< embed ST Ewok Dev  data as "Data"*/

    /*!< user specific field */
	port_i2c_t *port_inst;
	int8_t port_channel;
    uint8_t device_enum;					//device id, number,
    uint8_t device_address;					//device i2c address.
} VL53L0X_Dev_t;

/**
 * @brief   Declare the device Handle as a pointer of the structure @a VL53L0X_Dev_t.
 *
 */
typedef VL53L0X_Dev_t* VL53L0X_DEV;

#define VL53L0X_CALIBRATE_SPADS (1)
#define VL53L0X_CALIBRATE_REF (2)
#define VL53L0X_CALIBRATE_OFFSET (3)
#define VL53L0X_CALIBRATE_XTALK (4)

#define VL53L0X_POWER_ON_DELAY_MS (30)
#define VL53L0X_POWER_OFF_DELAY_MS (30)

#define VL53L0X_OFFSET_CALIBRATION_DELAY_S (5)
#define VL53L0X_XTALK_CALIBRATION_DELAY_S (5)

#define VL53L0X_OFFSET_CALIBRATION_DISTANCE_MM 100
#define VL53L0X_XTALK_CALIBRATION_DISTANCE_MM 100

#define VL53L0X_POLLING_DELAY 30

typedef struct {
	//Ranging config
	VL53L0X_DeviceModes RANGING_MODE;

	uint32_t INTER_MEASUREMENT_PERIOD; //ms
	uint32_t MEASUREMENT_TIMING_BUDGET; //us

	uint16_t LINEARITY_CORRECTIVE_GAIN; //x1000

	uint8_t FRACTION_ENABLE;

	uint8_t VCSEL_PERIOD_PRE_RANGE;
	uint8_t VCSEL_PERIOD_FINAL_RANGE;

	////Calibration////
	struct {
		uint32_t COUNT;
		uint8_t APERATURE;
	} REFERANCE_SPADS;

	struct {
		uint8_t VHV;
		uint8_t PHASE;
	} REFERANCE_CALIBRATION;

	//Offset compensation
	struct {
		FixPoint1616_t DISTANCE; //mm
		int32_t DATA; //um
	} OFFSET_COMPENSATION;

	//Cross-talk compensation
	struct {
		uint8_t ENABLE; //mm
		FixPoint1616_t DISTANCE; //mm
		FixPoint1616_t RATE; //Mcps
	} XTALK_COMPENSATION;



	//Interrupt config
	struct {
		VL53L0X_GpioFunctionality MODE;
		VL53L0X_InterruptPolarity POLARITY;
		FixPoint1616_t THRESHOLD_LOW;
		FixPoint1616_t THRESHOLD_HIGH;
	} INTERRUPT;

	//Sequence steps
	struct {
		uint8_t TCC;
		uint8_t DSS;
		uint8_t MSRC;
		uint8_t PRE_RANGE;
		uint8_t FINAL_RANGE;
	} SEQUENCESTEP_ENABLE;

	struct {
		FixPoint1616_t TCC; //ms
		FixPoint1616_t DSS; //ms
		FixPoint1616_t MSRC; //ms
		FixPoint1616_t PRE_RANGE; //ms
		FixPoint1616_t FINAL_RANGE; //ms
	} SEQUENCESTEP_TIMEOUT;


	//Limit checks
	struct {
		uint8_t SIGMA_FINAL_RANGE;
		uint8_t SIGNAL_RATE_FINAL_RANGE;
		uint8_t SIGNAL_REF_CLIP;
		uint8_t RANGE_IGNORE_THRESHOLD;
		uint8_t SIGNAL_RATE_MSRC;
		uint8_t SIGNAL_RATE_PRE_RANGE;
	} LIMITCHECK_ENABLE;

	struct {
		FixPoint1616_t SIGMA_FINAL_RANGE; //mm
		FixPoint1616_t SIGNAL_RATE_FINAL_RANGE; //Mcps
		FixPoint1616_t SIGNAL_REF_CLIP;
		FixPoint1616_t RANGE_IGNORE_THRESHOLD; //Mcps/SPAD
		FixPoint1616_t SIGNAL_RATE_MSRC; //Mcps
		FixPoint1616_t SIGNAL_RATE_PRE_RANGE; //Mcps
	} LIMITCHECK_VALUE;

	uint8_t WRAP_ARAUND_CHECK;

} VL53L0X_Config_t;

//Range status defines
#define VL53L0X_RANGESTATUS_RANGEVALID 0
#define VL53L0X_RANGESTATUS_SIGMA 1
#define VL53L0X_RANGESTATUS_SIGNAL 2
#define VL53L0X_RANGESTATUS_MINRANGE 3
#define VL53L0X_RANGESTATUS_PHASE 4
#define VL53L0X_RANGESTATUS_HW 5

#define VL53L0X_STRING_GPIOFUNCTIONALITY_OFF \
	"NO Interrupt"
#define VL53L0X_STRING_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW \
	"Level Low (value < thresh_low)"
#define VL53L0X_STRING_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH \
	"Level High (value > thresh_high)"
#define VL53L0X_STRING_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT \
	"Out Of Window (value < thresh_low OR value > thresh_high)"
#define VL53L0X_STRING_GPIOFUNCTIONALITY_NEW_MEASURE_READY \
	"New Sample Ready"

/**
 * @def PALDevDataGet
 * @brief Get ST private structure @a VL53L0X_DevData_t data access
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * It maybe used and as real data "ref" not just as "get" for sub-structure item
 * like PALDevDataGet(FilterData.field)[i] or PALDevDataGet(FilterData.MeasurementIndex)++
 */
#define PALDevDataGet(Dev, field) (Dev->Data.field)

/**
 * @def PALDevDataSet(Dev, field, data)
 * @brief  Set ST private structure @a VL53L0X_DevData_t data field
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define PALDevDataSet(Dev, field, data) (Dev->Data.field)=(data)


/**
 * @defgroup VL53L0X_registerAccess_group PAL Register Access Functions
 * @brief    PAL Register Access Functions
 *  @{
 */

/**
 * Lock comms interface to serialize all commands to a shared I2C interface for a specific device
 * @param   Dev       Device Handle
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
//VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev);

/**
 * Unlock comms interface to serialize all commands to a shared I2C interface for a specific device
 * @param   Dev       Device Handle
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
//VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev);


float fixPoint1616_to_float(FixPoint1616_t inp);
FixPoint1616_t float_to_fixPoint1616(float inp);
const char* VL53L0X_ErrorCodeToString(VL53L0X_Error code);
const char* VL53L0X_ErrorCodeToString(VL53L0X_Error code);
const char* VL53L0X_CheckEnableCodeToString(uint16_t code);
const char* VL53L0X_SequenceStepCodeToString(uint16_t code);
const char* VL53L0X_RangeStatusCodeToString(uint16_t code);
const char* VL53L0X_DeviceStateCodeToString(VL53L0X_State code);
const char* VL53L0X_DeviceErrorCodeToString(VL53L0X_DeviceError code);
const char* VL53L0X_GpioFunctionalityCodeToString(VL53L0X_GpioFunctionality code);
const char* VL53L0X_DeviceModeCodeToString(uint16_t code);
/**
 * Writes the supplied byte buffer to the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);

/**
 * Reads the requested number of bytes from the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);

/**
 * Write single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      8 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data);

/**
 * Write word register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      16 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data);

/**
 * Write double word (4 byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      32 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data);

/**
 * Read single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 8 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data);

/**
 * Read word (2byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 16 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data);

/**
 * Read dword (4byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 32 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data);

/**
 * Threat safe Update (read/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param   Dev        Device Handle
 * @param   index      The register index
 * @param   AndData    8 bit and data
 * @param   OrData     8 bit or data
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData);

/** @} end of VL53L0X_registerAccess_group */


/**
 * @brief execute delay in all polling API call
 *
 * A typical multi-thread or RTOs implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed)
 * if nothing specific is need you can define it as an empty/void macro
 * @code
 * #define VL53L0X_PollingDelay(...) (void)0
 * @endcode
 * @param Dev       Device Handle
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev); /* usually best implemented as a real function */

/** @} end of VL53L0X_platform_group */

#ifdef __cplusplus
}
#endif

#endif  /* _VL53L0X_PLATFORM_H_ */



