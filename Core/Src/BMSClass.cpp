/*
 *
 *
 *
 */


#ifndef BMS_CLASS_
#define BMS_CLASS_

#include "BMSClass.hpp"

BMSClass::BMSClass(BQ76952 *chip1, BQ76952 *chip2){
	pChip1 = chip1;
	pChip2 = chip2;
}

// read functions
HAL_StatusTypeDef BMSClass::readVoltages(){
	HAL_StatusTypeDef status = pChip1 -> ReadVoltages();
	if (status != HAL_OK)	{return status;}

	status = pChip2 -> ReadVoltages();
	if (status != HAL_OK)	{return status;}

	for (int i = 0; i < 16; i++){
		cellVoltages[i] = pChip1 -> cell_voltages_[i];
	}
	for (int j = 0; j < 13; j++){
		cellVoltages[j+16] = pChip2 -> cell_voltages_[j];
	}

	totalVoltage = (pChip1 -> stack_voltage_) + (pChip2 -> stack_voltage_); // TODO: check that this shouldn't be pack_voltage
	averageVoltage = ((pChip1 -> avg_cell_voltage_) + (pChip2 -> avg_cell_voltage_))/2;

	maxVoltage = pChip1 -> high_cell_voltage_;
	if (pChip2 -> high_cell_voltage_ > maxVoltage){
		maxVoltage = pChip2 -> high_cell_voltage_;
	}

	minVoltage = pChip1 -> low_cell_voltage_;
		if (pChip2 -> low_cell_voltage_ < minVoltage){
			minVoltage = pChip2 -> low_cell_voltage_;
		}

	return status;
}

HAL_StatusTypeDef BMSClass::readCurrents(){
	HAL_StatusTypeDef status = pChip1 -> ReadCurrent();
	if (status != HAL_OK)	{return status;}

	status = pChip2 -> ReadCurrent();
	if (status != HAL_OK)	{return status;}

	for (int i = 3; i >= 0; i--){
		prevCurrents[i] = prevCurrents[i-1];
	}
	prevCurrents[0] = (pChip1 -> pack_current_) + (pChip2 -> pack_current_);

	rollingAvgCurrent = 0;
	for (int j = 0; j < 4; j++){
		rollingAvgCurrent += prevCurrents[j];
	}
	rollingAvgCurrent /= 5;

	return status;
}

HAL_StatusTypeDef BMSClass::readTemperatures(){
	return HAL_OK;
}


//voltages
int16_t BMSClass::getCellVoltage(BMSCellID cellID){
	return cellVoltages[cellID];
}

void BMSClass::getAll29CellVoltages(int16_t *arrData){
	arrData = cellVoltages;
}

int16_t BMSClass::getTotalVoltage(){
	return totalVoltage;
}
int16_t BMSClass::getAverageVoltage(){
	return averageVoltage;
}
int16_t BMSClass::getMaxVoltage(){
	return maxVoltage;
}
int16_t BMSClass::getMinVoltage(){
	return minVoltage;
}

// currents
void BMSClass::getPrevCurrents(int16_t *arrData){
	arrData = prevCurrents;
}
int16_t BMSClass::getRollingAvgCurrent(){
	return rollingAvgCurrent;
}

// temperatures
uint16_t BMSClass::getCellTemperature(BMSCellID cellID){
	return cellTemperatures[cellID];
}

void BMSClass::getAll29CellTemperatures(uint16_t *arrData){
	arrData = cellTemperatures;
}

uint16_t BMSClass::getMiscTemp1(){
	return miscTemp1;
}

uint16_t BMSClass::getMiscTemp2(){
	return miscTemp2;
}

uint16_t BMSClass::getMiscTemp3(){
	return miscTemp3;
}

uint16_t BMSClass::getAvgCellTemp(){
	return avgCellTemp;
}

uint16_t BMSClass::getMaxCellTemp(){
	return maxCellTemp;
}

uint16_t BMSClass::getMinCellTemp(){
	return minCellTemp;
}













#endif
