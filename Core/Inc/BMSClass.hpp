#ifndef BMS_CLASS_HPP_
#define BMS_CLASS_HPP_

#include <main.hpp>
#include <BQ76952.hpp>
#include <math.h>

class BMSClass{

public:
	BMSClass(BQ76952 *chip1, BQ76952 *chip2);

	HAL_StatusTypeDef readVoltages();
	HAL_StatusTypeDef readTemperatures();
	HAL_StatusTypeDef readCurrents();

	int16_t cellVoltages[29] = {0};				// all cell voltages, in C
	int16_t totalVoltage = 0;
	int16_t averageVoltage = 0;
	int16_t maxVoltage = 0;
	int16_t minVoltage = 0;
	enum BMSCellID {CHIP1_CELL1, CHIP1_CELL2, CHIP1_CELL3, CHIP1_CELL4, CHIP1_CELL5, CHIP1_CELL6, CHIP1_CELL7, CHIP1_CELL8,
		CHIP1_CELL9, CHIP1_CELL10, CHIP1_CELL11, CHIP1_CELL12, CHIP1_CELL13, CHIP1_CELL14, CHIP1_CELL15, CHIP1_CELL16,
		CHIP2_CELL1, CHIP2_CELL2, CHIP2_CELL3, CHIP2_CELL4, CHIP2_CELL5, CHIP2_CELL6, CHIP2_CELL7, CHIP2_CELL8,
		CHIP2_CELL9, CHIP2_CELL10, CHIP2_CELL11, CHIP2_CELL12, CHIP2_CELL13};

	int16_t getCellVoltage(BMSCellID cellID);
	void getAll29CellVoltages(int16_t *arrData);
	int16_t getTotalVoltage();
	int16_t getAverageVoltage();
	int16_t getMaxVoltage();
	int16_t getMinVoltage();

	int16_t prevCurrents[5] = {0};		// sum of pack currents for both chips, in mA
	int16_t rollingAvgCurrent = 0;
	void getPrevCurrents(int16_t *arrData);
	int16_t getRollingAvgCurrent();

	uint16_t cellTemperatures[29] = {0};			// all temperatures, in C
	uint16_t miscTemp1 = 0;			// three thermistors, currently unknown function
	uint16_t miscTemp2 = 0;
	uint16_t miscTemp3 = 0;
	uint16_t avgCellTemp = 0;
	uint16_t maxCellTemp = 0;
	uint16_t minCellTemp = 0;
	uint16_t getCellTemperature(BMSCellID cellNum);
	void getAll29CellTemperatures(uint16_t *arrData);
	uint16_t getMiscTemp1();
	uint16_t getMiscTemp2();
	uint16_t getMiscTemp3();
	uint16_t getAvgCellTemp();
	uint16_t getMaxCellTemp();
	uint16_t getMinCellTemp();

	BQ76952 *pChip1;
	BQ76952 *pChip2;
};

#endif


