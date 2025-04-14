/*
 *  BQ76952.hpp
 *
 *  Created on: October 20, 2023
 *      Author: Matthew Shen
 * 
 *  https://github.com/LibreSolar/bms-firmware/tree/main
 */

#ifndef BQ76952_HPP_
#define BQ76952_HPP_

#include <main.h>
#include "BQ769X2_Registers.h"
#include <math.h>

#define BQ_I2C_ADDR_WRITE 0x10
#define BQ_I2C_ADDR_READ 0x11

static const uint8_t temp_registers[] = {BQ769X2_CMD_TEMP_TS1, BQ769X2_CMD_TEMP_TS2,
			BQ769X2_CMD_TEMP_TS3, BQ769X2_CMD_TEMP_ALERT, BQ769X2_CMD_TEMP_HDQ, BQ769X2_CMD_TEMP_CFETOFF,
				BQ769X2_CMD_TEMP_DFETOFF, BQ769X2_CMD_TEMP_DCHG, BQ769X2_CMD_TEMP_DDSG};

class BQ76952 {
public:
    HAL_StatusTypeDef Init(I2C_HandleTypeDef *hi2c);
    HAL_StatusTypeDef Init(I2C_HandleTypeDef *hi2c, uint8_t i2cAddress);
    uint8_t i2cAddressWrite;
    uint8_t i2cAddressRead;

    HAL_StatusTypeDef Reset();
    HAL_StatusTypeDef ConfigUpdate(bool config_update); // TODO
    HAL_StatusTypeDef ConfigureVoltageRegs(); // TODO
    HAL_StatusTypeDef ReadVoltages();       // TODO: Add support for connected cells
    HAL_StatusTypeDef ReadSafetyFaults();
    HAL_StatusTypeDef ReadCurrent();
    HAL_StatusTypeDef ReadTemperatures();

    // Balancing Related Functions
    HAL_StatusTypeDef ChangeBalancingStatus(bool disableManualBal, bool enableBalWhileSleep, bool enableBalWhileRelax, bool enableBalWhileCharging); // 1 for enable, 0 for disable
    HAL_StatusTypeDef DisableBalancing();
    HAL_StatusTypeDef SetManualBalancing(bool set);
    HAL_StatusTypeDef SetBalancingWhileSleeping(bool set);
    HAL_StatusTypeDef SetBalancingWhileRelaxing(bool set);
    HAL_StatusTypeDef SetBalancingWhileCharging(bool set);
    HAL_StatusTypeDef StartBalancingOnCells(uint16_t cell_bitmask); // manual balancing, input a bitmask with a 1 in the cells you want to receive active balancing
    HAL_StatusTypeDef ClearManualBalancing();

    HAL_StatusTypeDef SetBalancingMinCellTemp(int8_t minTempCelsius);		// balancing disabled if min cell temp is below this
    HAL_StatusTypeDef SetBalancingMaxCellTemp(int8_t maxTempCelsius);		// balancing disabled if max cell temp is above this
    HAL_StatusTypeDef SetBalancingMaxInternalTemp(int8_t maxTempCelsius);	// balancing disabled if internal temp is above this
    HAL_StatusTypeDef SetBalancingInterval(uint8_t intervalTimeSeconds);	// interval between checks for auto-balancing conditions
    HAL_StatusTypeDef SetBalancingMaxCells(uint8_t numberOfCells);			// automatic balancing will apply to at most this many cells
    enum BalancingType{CHARGING, RELAXING};
    HAL_StatusTypeDef SetBalancingMinCellVoltage(int16_t minCellVoltagemV, BalancingType type);			// balancing disabled if min cell voltage is below this
    HAL_StatusTypeDef SetBalancingStartDeltaVoltage(int16_t startDeltaVoltagemV, BalancingType type);	// balancing begins when (maxV - minV) is above this
    HAL_StatusTypeDef SetBalancingStopDeltaVoltage(int16_t stopDeltaVoltagemV, BalancingType type);		// balancing ends when (maxV - minV) is below this


    //Sleep and Deep Sleep / Mode related functions
    HAL_StatusTypeDef EnterDeepSleep();
    HAL_StatusTypeDef ExitDeepSleep();
    HAL_StatusTypeDef ModifySleepCurrentBoundary(int16_t boundary); // in mA, signed integer but minimum value is zero, max is 32767, startup is 20 mA

    HAL_StatusTypeDef Shutdown();

    int16_t GetCellVoltage(uint32_t cell_num);
    int16_t GetPackVoltage();
    int16_t GetStackVoltage();
    int16_t GetAvgCellVoltage();
    int16_t GetHighCellVoltage();
    int16_t GetLowCellVoltage();
    int16_t GetPackCurrent();
    bool GetConfigUpdateStatus();

    constexpr uint8_t CELL_NO_TO_ADDR(uint8_t cell_no) { return BQ769X2_CMD_VOLTAGE_CELL_1 + ((cell_no-1)*2); }
    HAL_StatusTypeDef WriteBytes(const uint8_t reg_addr, const uint8_t *data, const size_t num_bytes);
    HAL_StatusTypeDef ReadBytes(uint8_t reg_addr, uint8_t *data, const size_t num_bytes);
    HAL_StatusTypeDef DirectReadU2(const uint8_t reg_addr, uint16_t *value); // TODO
    HAL_StatusTypeDef DirectReadI2(const uint8_t reg_addr, int16_t *value); // TODO
    HAL_StatusTypeDef SubcmdRead(const uint16_t subcmd, uint32_t *value, const size_t num_bytes);
    HAL_StatusTypeDef SubcmdReadU1(const uint16_t subcmd, uint8_t *value);
    HAL_StatusTypeDef SubcmdReadU2(const uint16_t subcmd, uint16_t *value);
    HAL_StatusTypeDef SubcmdReadU4(const uint16_t subcmd, uint32_t *value);
    HAL_StatusTypeDef SubcmdReadI1(const uint16_t subcmd, int8_t *value);       // TODO: Remove
    HAL_StatusTypeDef SubcmdReadI2(const uint16_t subcmd, int16_t *value);     // TODO: Remove
    HAL_StatusTypeDef SubcmdReadI4(const uint16_t subcmd, int32_t *value);     // TODO: Remove
    HAL_StatusTypeDef SubcmdReadF4(const uint16_t subcmd, float *value);
    HAL_StatusTypeDef SubcmdWrite(const uint16_t subcmd, const uint32_t value, const size_t num_bytes);
    HAL_StatusTypeDef SubcmdCmdOnly(const uint16_t subcmd);
    HAL_StatusTypeDef SubcmdCmdWriteU1(const uint16_t subcmd, uint8_t value);
    HAL_StatusTypeDef SubcmdCmdWriteU2(const uint16_t subcmd, uint16_t value);
    HAL_StatusTypeDef SubcmdCmdWriteU4(const uint16_t subcmd, uint32_t value);
    HAL_StatusTypeDef SubcmdCmdWriteI1(const uint16_t subcmd, int8_t value);
    HAL_StatusTypeDef SubcmdCmdWriteI2(const uint16_t subcmd, int16_t value);
    HAL_StatusTypeDef SubcmdCmdWriteI4(const uint16_t subcmd, int32_t value);
    HAL_StatusTypeDef SubcmdCmdWriteF4(const uint16_t subcmd, float value);
    HAL_StatusTypeDef DatamemReadU1(const uint16_t reg_addr, uint8_t *value);
    HAL_StatusTypeDef DatamemWriteU1(const uint16_t reg_addr, uint8_t value);
    HAL_StatusTypeDef DatamemWriteU2(const uint16_t reg_addr, uint16_t value);
    HAL_StatusTypeDef DatamemWriteI1(const uint16_t reg_addr, int8_t value);
    HAL_StatusTypeDef DatamemWriteI2(const uint16_t reg_addr, int16_t value);
    HAL_StatusTypeDef DatamemWriteF4(const uint16_t reg_addr, float value);
    HAL_StatusTypeDef UpdateMode();

    I2C_HandleTypeDef *hi2c_;

    int16_t cell_voltages_[16] = {0};         // Voltage of each cell in mV
    int16_t avg_cell_voltage_ = 0;          // Average cell voltage in mV
    int16_t high_cell_voltage_ = 0;         // Highest cell voltage in mV
    int16_t low_cell_voltage_ = 0;          // Lowest cell voltage in mV
    int16_t pack_current_ = 0;              // Brief battery pack current, mA

    int16_t pack_voltage_ = 0;              // Pack voltage in mV
    int16_t stack_voltage_ = 0;             // Stack voltage in mV

    float temperatures_[sizeof(temp_registers)/sizeof(temp_registers[0])] = {0}; // temperatures in C
    float avg_temperature_ = 0; // avg temperature of temp_registers in  C
    float high_temperature_ = 0; // high temperature of temp_registers in  C
    float low_temperature_ = 0; // low temperature of temp_registers in  C
    float chip_temperature_ = 0; // temperature of chip in C

    enum BQMode{BQ_MODE_NORMAL, BQ_MODE_SLEEP, BQ_MODE_DEEPSLEEP, BQ_MODE_SHUTDOWN, BQ_MODE_CONFIGUPDATE};
    BQMode current_mode_ = BQ_MODE_NORMAL; // current mode BMS chip is in

    uint8_t stat_a_byte = 0x00;
    uint8_t stat_b_byte = 0x00;
    uint8_t stat_c_byte = 0x00;

    bool manual_bal_disabled_		= false;
    bool auto_bal_charging_enabled_	= false;
    bool auto_bal_relax_enabled_	= false;
    bool auto_bal_sleep_enabled_	= false;


};

#endif  /* BQ76952_HPP_ */
