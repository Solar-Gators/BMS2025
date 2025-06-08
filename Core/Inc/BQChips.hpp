#ifndef BQCHIPS_HPP_
#define BQCHIPS_HPP_

#include <main.h>
#include <BQ76952.hpp>
//#include <math.h>

class BQChips{

public:
	BQChips(BQ76952 *chip1, BQ76952 *chip2);

	HAL_StatusTypeDef readVoltages();

	int16_t cellVoltages[32] = {0};				// all cell voltages, in C
	int16_t totalVoltage = 0;
	int16_t averageVoltage = 0;
	int16_t maxVoltage = 0;
	int16_t minVoltage = 0;
	enum BMSCellID {CHIP1_CELL1, CHIP1_CELL2, CHIP1_CELL3, CHIP1_CELL4, CHIP1_CELL5, CHIP1_CELL6, CHIP1_CELL7, CHIP1_CELL8,
		CHIP1_CELL9, CHIP1_CELL10, CHIP1_CELL11, CHIP1_CELL12, CHIP1_CELL13, CHIP1_CELL14, CHIP1_CELL15, CHIP1_CELL16,
		CHIP2_CELL1, CHIP2_CELL2, CHIP2_CELL3, CHIP2_CELL4, CHIP2_CELL5, CHIP2_CELL6, CHIP2_CELL7, CHIP2_CELL8,
		CHIP2_CELL9, CHIP2_CELL10, CHIP2_CELL11, CHIP2_CELL12, CHIP2_CELL13,CHIP2_CELL14,CHIP2_CELL116,CHIP2_CELL16};

	int16_t getCellVoltage(int cellID);
	void getAll32CellVoltages(int16_t *arrData);
	int16_t getTotalVoltage();
	int16_t getAverageVoltage();
	int16_t getMaxVoltage();
	int16_t getMinVoltage();

	BQ76952 *pChip1;
	BQ76952 *pChip2;
};

#endif


