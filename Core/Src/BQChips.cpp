/*
 *
 *
 *
 */

#ifndef BQCHIPS_
#define BQCHIPS_

#include <BQChips.hpp>

BQChips::BQChips(BQ76952 *chip1, BQ76952 *chip2){
	pChip1 = chip1;
	pChip2 = chip2;
}

// read functions
HAL_StatusTypeDef BQChips::readVoltages(){
	HAL_StatusTypeDef status = pChip1 -> ReadVoltages();
	if (status != HAL_OK)	{return status;}

	status = pChip2 -> ReadVoltages();
	if (status != HAL_OK)	{return status;}

	for (int i = 0; i < 16; i++){
		cellVoltages[i] = pChip1->cell_voltages_[i];
		cellVoltages[i+16] = pChip2->cell_voltages_[i];
	}

	totalVoltage = (pChip1 -> stack_voltage_) + (pChip2 -> stack_voltage_); // check that this shouldn't be pack_voltage
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

//voltages
int16_t BQChips::getCellVoltage(int cellID){
	return cellVoltages[cellID];
}

void BQChips::getAll32CellVoltages(int16_t arrData[]){
	for (int i = 0; i < 32; i++){
		arrData[i] = cellVoltages[i];
	}
}

int16_t BQChips::getTotalVoltage(){
	return totalVoltage;
}
int16_t BQChips::getAverageVoltage(){
	return averageVoltage;
}
int16_t BQChips::getMaxVoltage(){
	return maxVoltage;
}
int16_t BQChips::getMinVoltage(){
	return minVoltage;
}

#endif
