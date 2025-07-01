/*
 * BMS.hpp
 *
 *  Created on: Apr 30, 2025
 *      Author: samrb
 */

#include "main.h"

#ifndef INC_BMS_HPP_
#define INC_BMS_HPP_

enum errorState{
	noFault,
	lowCellVoltage,
	highCellVoltage,
	overTempCharge,
	overTempDischarge,
	overCurrentCharge,
	overCurrentDischarge
};

enum chargeState{
	discharging,
	charging
};

struct BMSData {

	//voltage data
	uint16_t highVoltage_mV;
	uint16_t lowVoltage_mV;
	uint16_t avgVoltage_mV;
	uint32_t totalVoltage_mV;

	uint8_t highVoltageIndex;
	uint8_t lowVoltageIndex;

	//temp data
	float highTemp;
	float lowTemp;
	float avgTemp;

	uint16_t highTempIndex;
	uint16_t lowTempIndex;

	//current data
	float lowCurrent_A;
	float highCurrent_A;

	//arrays
	uint16_t cellVoltages[32];
	uint16_t cellTempatures[29];
	float allTemperatures[32];
	bool tempExclusionList[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0};
	bool voltageExclusionList[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0};

};

typedef struct {
    uint16_t voltages[32];    // 32 voltage readings
    float temperatures[32];   // 32 temperature readings
    float current;            // Current reading
} BMS_Data_t;

#endif /* INC_BMS_HPP_ */

