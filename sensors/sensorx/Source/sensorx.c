/*
 * sensors.c
 *
 *  Created on: 21 Apr 2022
 *      Author: minda
 */

#include "sensorx.h"




int SENSORX_Delay(uint32_t ms)
{
	sleep_ms(ms);
	return SENSORX_OK;
}




//struct varstring {
//	const char *str;
//	const char size;
//};

//uint8_t SENSORx_PowerOn(VL53L0X_Dev_t *device)
//{
//	HAL_GPIO_WritePin(device->shutPin.port, device->shutPin.pin, GPIO_PIN_SET);
//	HAL_Delay(SENSOR_POWER_ON_DELAY);
//
//	if (HAL_GPIO_ReadPin(device->shutPin.port, device->shutPin.pin) != GPIO_PIN_SET) {
//		printf("Device[%d] POWER_ON:FAIL\n", device->DevEnum);
//		return 1;
//	}
//
//	printf("Device[%d] POWER_ON:OK\n", device->DevEnum);
//	return 0;
//}
//
//uint8_t SENSORx_PowerOff(VL53L0X_Dev_t *device)
//{
//	HAL_GPIO_WritePin(device->shutPin.port, device->shutPin.pin, GPIO_PIN_RESET);
//	HAL_Delay(SENSOR_POWER_ON_DELAY);
//
//	if (HAL_GPIO_ReadPin(device->shutPin.port, device->shutPin.pin) != GPIO_PIN_RESET) {
//		printf("Device[%d] POWER_ON:FAIL\n", device->DevEnum);
//		return 1;
//	}
//
//	printf("Device[%d] POWER_ON:OK\n", device->DevEnum);
//	return 0;
//}

//VL53L0X_ConfigFile_t sensors_config[10];
//
//void SENSORx_LoadConfig() {
//	FIL file;
//
//	if (f_open(&file, "sconfig.txt", FA_OPEN_EXISTING | FA_READ))
//		return;
//
//	int headerFound = 0;
//	int device = 0;
//
//	do {
//		char newString[STR_MAX_LEN] = "";
//		if (f_gets(newString, STR_MAX_LEN, &file) == 0) {
//			break;
//		}
//
//		str_remove_chr(newString, newString, ' '); //remove whitespace
//		char *pvar = newString;
//		int varnum = findVar(newString, &pvar);
//		if ((varnum < 0) || (pvar == 0) || (*pvar == 0)) {
//			break;
//		}
//
//		if (varnum && headerFound) {
//			setVar(&sensors_config[device], varnum, pvar);
//		}
//		else if (varnum == 0){
//			if (isDigit(*pvar)) {
//				int val = atoi(pvar);
//				if (val >= 0 && val <= 255) {
//					device = val;
//					headerFound = 1;
//				}
//			}
//		}
//
//
//	} while(!f_eof(&file));
//
//	f_close(&file);
//
//}

//const struct varstring varstrings[] = {
//	{"DEVICE", sizeof("DEVICE") - 1},
//	{"SpadCount", sizeof("SpadCount") - 1},
//	{"isAperatureSpads", sizeof("isAperatureSpads") - 1},
//	{"vhvSettings", sizeof("vhvSettings") - 1},
//	{"phaseCal", sizeof("phaseCal") - 1},
//	{"OffsetCalDistanceMilliMeter", sizeof("OffsetCalDistanceMilliMeter") - 1},
//	{"OffsetCalibrationDataMicroMeter", sizeof("OffsetCalibrationDataMicroMeter") - 1},
//	{"XTalkCalDistanceMilliMeter", sizeof("XTalkCalDistanceMilliMeter") - 1},
//	{"XTalkCompensationRateMegaCps", sizeof("XTalkCompensationRateMegaCps") - 1},
//	{"XTalkCompensationEnable", sizeof("XTalkCompensationEnable") - 1},
//	{"rangingMode", sizeof("rangingMode") - 1},
//	{"interruptMode", sizeof("interruptMode") - 1},
//	{"interruptPolarity", sizeof("interruptPolarity") - 1},
//	{"sigmaFinalRange", sizeof("sigmaFinalRange") - 1},
//	{"signalRateFinalRange", sizeof("signalRateFinalRange") - 1},
//	{"signalRefClip", sizeof("signalRefClip") - 1},
//	{"rangeIgnoreThreshold", sizeof("rangeIgnoreThreshold") - 1},
//	{"signalRateMSRC", sizeof("signalRateMSRC") - 1},
//	{"signalRatePreRange", sizeof("signalRatePreRange") - 1},
//	{"timingBudgetMicroSeconds", sizeof("timingBudgetMicroSeconds") - 1},
//};
//
//const int varstrings_count = sizeof(varstrings) / sizeof(struct varstring);
//
//int isDigit(char chr) {
//	return ((chr >= '0') && (chr <= '9'));
//}
//
//void str_remove_chr(char *out, const char *in, char chr) {
//    do {
//        while (*in == chr) {
//            ++in;
//        }
//    } while ((*out++ = *in++));
//}
//
//int findVar(char *str, char **ppvar) {
//
//	char *p = strchr(str, '='); //find assignment operator
//
//	if (p == 0)
//		return -1; //return didn't find assignment operator
//
//	int tstlen = p - str;
//
//	for (int i = 0; i < varstrings_count; i++) {
//		if (tstlen == varstrings[i].size) { // check if length is equal first, to rule out obvious comparisons
//			if (strncmpi(str, varstrings[i].str, varstrings[i].size) == 0) {
//
//				*ppvar = p + 1;
//
//				return i;
//			}
//		}
//	}
//
//	return -1; //no match found
//}
//
//void setVar(VL53L0X_ConfigFile_t *config, int varNum, char *valStr) {
//	switch(varNum) {
//	case 0:
//		return;
//	case 1:
//		if (isDigit(*valStr)) {
//			uint32_t val = atol(valStr);
//			config->SpadCount = val;
//		}
//		return;
//	case 2:
//		if (isDigit(*valStr)) {
//			uint8_t val = atoi(valStr);
//			config->isAperatureSpads = val;
//		}
//		return;
//	case 3:
//		if (isDigit(*valStr)) {
//			uint8_t val = atoi(valStr);
//			config->vhvSettings = val;
//		}
//		return;
//	case 4:
//		if (isDigit(*valStr)) {
//			uint8_t val = atoi(valStr);
//			config->phaseCal = val;
//		}
//		return;
//	case 5:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->OffsetCalDistanceMilliMeter = val;
//		}
//		return;
//	case 6:
//		if (isDigit(*valStr) || *valStr == '-') {
//			int32_t val = atol(valStr);
//			config->OffsetCalibrationDataMicroMeter = val;
//		}
//		return;
//	case 7:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->XTalkCalDistanceMilliMeter = val;
//		}
//		return;
//	case 8:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->XTalkCompensationRateMegaCps = val;
//		}
//		return;
//	case 9:
//		if (isDigit(*valStr)) {
//			uint8_t val = atoi(valStr);
//			config->XTalkCompensationEnable = val;
//		}
//		return;
//	case 10:
//		if (isDigit(*valStr)) {
//			uint8_t val = atoi(valStr);
//			config->rangingMode = val;
//		}
//		return;
//	case 11:
//		if (isDigit(*valStr)) {
//			uint8_t val = atoi(valStr);
//			config->interruptMode = val;
//		}
//		return;
//	case 12:
//		if (isDigit(*valStr)) {
//			uint8_t val = atoi(valStr);
//			config->interruptPolarity = val;
//		}
//		return;
//	case 13:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->sigmaFinalRange = val;
//		}
//		return;
//	case 14:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->signalRateFinalRange = val;
//		}
//		return;
//	case 15:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->signalRefClip = val;
//		}
//		return;
//	case 16:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->rangeIgnoreThreshold = val;
//		}
//		return;
//	case 17:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->signalRateMSRC = val;
//		}
//		return;
//	case 18:
//		if (isDigit(*valStr)) {
//			FixPoint1616_t val = float_to_fixPoint1616(atof(valStr));
//			config->signalRatePreRange = val;
//		}
//		return;
//	case 19:
//		if (isDigit(*valStr)) {
//			uint32_t val = atol(valStr);
//			config->timingBudgetMicroSeconds = val;
//		}
//		return;
//	default:
//		return;
//	}
//}
