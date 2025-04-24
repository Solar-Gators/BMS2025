#include "BQ76952.hpp"
#include <string.h>
#include <stdbool.h>

#ifndef BQ76952_
#define BQ76952_

BQ76952::BQ76952(){

}

HAL_StatusTypeDef BQ76952::Init(I2C_HandleTypeDef *hi2c, uint8_t i2cAddress){
    hi2c_ = hi2c;
    i2cAddressWrite = i2cAddress;
    i2cAddressRead = i2cAddress+1;

    Reset();
    volatile uint32_t delay = 100000;
    while(delay--);

    // DatamemWriteU1(BQ769X2_SET_PROT_ENABLED_A, 12);

    return HAL_I2C_IsDeviceReady(hi2c_, i2cAddressWrite, 100, 50);
}

HAL_StatusTypeDef BQ76952::Init(I2C_HandleTypeDef *hi2c){
	return Init(hi2c, BQ_I2C_ADDR_WRITE);
}

HAL_StatusTypeDef BQ76952::Reset() {
    return SubcmdCmdOnly(BQ769X2_SUBCMD_RESET);
}

HAL_StatusTypeDef BQ76952::ConfigUpdate(bool config_update){
    HAL_StatusTypeDef status;

    if (config_update) {
        status = SubcmdCmdOnly(BQ769X2_SUBCMD_SET_CFGUPDATE);
    }
    else {
        status = SubcmdCmdOnly(BQ769X2_SUBCMD_EXIT_CFGUPDATE);
    }

    if(status != HAL_OK)
        return status;

    if (config_update) {
        current_mode_ = BQ_MODE_CONFIGUPDATE;
    }
    else {
        current_mode_ = BQ_MODE_NORMAL;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::ConfigureVoltageRegs() {
    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::ReadVoltages() {
    HAL_StatusTypeDef status = HAL_OK;

    int16_t voltage_sum = 0;
    low_cell_voltage_ = INT16_MAX;
    high_cell_voltage_ = INT16_MIN;

    for (int i = 0; i < 16; i++) {
        status = DirectReadI2(CELL_NO_TO_ADDR(i+1), &cell_voltages_[i]);
        if (status != HAL_OK) { return status; }

        if (cell_voltages_[i] > high_cell_voltage_) {
            high_cell_voltage_ = cell_voltages_[i];
        }
        else if (cell_voltages_[i] < low_cell_voltage_) {
            low_cell_voltage_ = cell_voltages_[i];
        }

        voltage_sum += cell_voltages_[i];
    }

    avg_cell_voltage_ = voltage_sum / 16;

    int16_t pack_voltage_temp; // this number will be in userV, by default 1 userV = 10 mV
    status = DirectReadI2(BQ769X2_CMD_VOLTAGE_PACK, &pack_voltage_temp);
    if (status != HAL_OK) { return status; }
    pack_voltage_ = pack_voltage_temp*10; // pack_voltage_ is in mV


    int16_t stack_voltage_temp; // this number will be in userV, by default 1 userV = 10 mV
    status = DirectReadI2(BQ769X2_CMD_VOLTAGE_STACK, &stack_voltage_temp);
    if (status != HAL_OK) { return status; }
    stack_voltage_ = stack_voltage_temp*10; //stack_voltage_ is in mV

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::ReadCurrent(){
    HAL_StatusTypeDef status;

    int16_t current = 0;// this value is in userA, where 1 userA = 1 mA by default
    status = DirectReadI2(BQ769X2_CMD_CURRENT_CC2, &current);

    if(status != HAL_OK)
        return status;

    pack_current_ = current; // in mA

    return status;
}

HAL_StatusTypeDef BQ76952::ReadTemperatures(){ 
    HAL_StatusTypeDef status;
    int num_temps = 0;
    float sum_temps = 0;
    int16_t temp = 0;

    //read temperatures of thermistors
    for (int i = 0; i < (int)(sizeof(temp_registers)/sizeof(temp_registers[0])); i++){
        status = DirectReadI2(temp_registers[i], &temp); // returns 0.1k
        if (status != HAL_OK) { return status; }
        
        temperatures_[i] = (temp * 10.0) - 273.15; // convert to cesius

        if(i == 0){
            low_temperature_ = temperatures_[i];
            high_temperature_ = temperatures_[i];
        }else{
            if(temperatures_[i] < low_temperature_){
                low_temperature_ = temperatures_[i];
            }
            if(temperatures_[i] > high_temperature_){
                high_temperature_ = temperatures_[i];
            }
        }

        num_temps++;
        sum_temps += temperatures_[i];
    }

    avg_temperature_ = sum_temps / num_temps;

    // read chips internal temperature value
    status = DirectReadI2(BQ769X2_CMD_TEMP_INT, &temp);
    if (status != HAL_OK) { return status; }
    chip_temperature_ = (temp * 10.0) - 273.15;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::ReadSafetyFaults() {
    HAL_StatusTypeDef status;

    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_A, &stat_a_byte, 1);
    if (status != HAL_OK) 
        return status;
    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_B, &stat_b_byte, 1);
    if (status != HAL_OK) 
        return status;
    status = ReadBytes(BQ769X2_CMD_SAFETY_ALERT_C, &stat_c_byte, 1);
    if (status != HAL_OK) 
        return status;

    return HAL_OK;
}


// BALANCING CONFIG:
HAL_StatusTypeDef BQ76952::ChangeBalancingStatus(bool disableManualBal, 
                                                 bool enableBalWhileSleep, 
                                                 bool enableBalWhileRelax, 
                                                 bool enableBalWhileCharging) {
    HAL_StatusTypeDef status;
    status = ConfigUpdate(true);
    if (status != HAL_OK) 
        return status;

    uint8_t cbalConf = (disableManualBal << 4) | (enableBalWhileSleep << 2) | (enableBalWhileRelax << 1) | (enableBalWhileCharging << 0);
    status = DatamemWriteU1(BQ769X2_SET_CBAL_CONF, cbalConf);
    if (status != HAL_OK) 
        return status;
    
    manual_bal_disabled_ = disableManualBal;
    auto_bal_sleep_enabled_ = enableBalWhileSleep;
    auto_bal_relax_enabled_ = enableBalWhileRelax;
    auto_bal_charging_enabled_ = enableBalWhileCharging;

    status = ConfigUpdate(false);
    return status;
}

HAL_StatusTypeDef BQ76952::DisableBalancing(){
	return ChangeBalancingStatus(1, 0, 0, 0);
}

HAL_StatusTypeDef BQ76952::SetManualBalancing(bool set){ // already enabled by default on startup
    return ChangeBalancingStatus(!set, auto_bal_sleep_enabled_, auto_bal_relax_enabled_ , auto_bal_charging_enabled_);
}

HAL_StatusTypeDef BQ76952::SetBalancingWhileSleeping(bool set){
    return ChangeBalancingStatus(manual_bal_disabled_, set, auto_bal_relax_enabled_ , auto_bal_charging_enabled_);
}

HAL_StatusTypeDef BQ76952::SetBalancingWhileRelaxing(bool set){
    return ChangeBalancingStatus(manual_bal_disabled_, auto_bal_sleep_enabled_, set, auto_bal_charging_enabled_);
}

HAL_StatusTypeDef BQ76952::SetBalancingWhileCharging(bool set){
    return ChangeBalancingStatus(manual_bal_disabled_, auto_bal_sleep_enabled_, auto_bal_relax_enabled_, set);
}

HAL_StatusTypeDef BQ76952::SetBalancingMinCellTemp(int8_t minTempCelsius){
	return DatamemWriteI1(BQ769X2_SET_CBAL_MIN_CELL_TEMP, minTempCelsius);
}

HAL_StatusTypeDef BQ76952::SetBalancingMaxCellTemp(int8_t maxTempCelsius){
	return DatamemWriteI1(BQ769X2_SET_CBAL_MAX_CELL_TEMP, maxTempCelsius);
}

HAL_StatusTypeDef BQ76952::SetBalancingMaxInternalTemp(int8_t maxTempCelsius){
	return DatamemWriteI1(BQ769X2_SET_CBAL_MAX_INT_TEMP, maxTempCelsius);
}

HAL_StatusTypeDef BQ76952::SetBalancingInterval(uint8_t intervalTimeSeconds){
	return DatamemWriteU1(BQ769X2_SET_CBAL_INTERVAL, intervalTimeSeconds);
}

HAL_StatusTypeDef BQ76952::SetBalancingMaxCells(uint8_t numCells){
	return DatamemWriteU1(BQ769X2_SET_CBAL_MAX_CELLS, numCells);
}

HAL_StatusTypeDef BQ76952::SetBalancingMinCellVoltage(int16_t minCellVoltagemV, BalancingType type){
	if (type == CHARGING){
		return DatamemWriteI2(BQ769X2_SET_CBAL_CHG_MIN_CELL_V, minCellVoltagemV);
	}
	else {
		return DatamemWriteI2(BQ769X2_SET_CBAL_RLX_MIN_CELL_V, minCellVoltagemV);
	}
}

HAL_StatusTypeDef BQ76952::SetBalancingStartDeltaVoltage(int16_t startDeltaVoltagemV, BalancingType type){
	if (type == CHARGING){
		return DatamemWriteU1(BQ769X2_SET_CBAL_CHG_MIN_DELTA, startDeltaVoltagemV);
	}
	else {
		return DatamemWriteU1(BQ769X2_SET_CBAL_RLX_MIN_DELTA, startDeltaVoltagemV);
	}
}

HAL_StatusTypeDef BQ76952::SetBalancingStopDeltaVoltage(int16_t stopDeltaVoltagemV, BalancingType type){
	if (type == CHARGING){
		return DatamemWriteU1(BQ769X2_SET_CBAL_CHG_STOP_DELTA, stopDeltaVoltagemV);
	}
	else {
		return DatamemWriteU1(BQ769X2_SET_CBAL_RLX_STOP_DELTA, stopDeltaVoltagemV);
	}
}


// MANUAL BALANCING:
HAL_StatusTypeDef BQ76952::StartBalancingOnCells(uint16_t cell_bitmask){
    return SubcmdCmdWriteU2(BQ769X2_SUBCMD_CB_ACTIVE_CELLS, cell_bitmask);
}

HAL_StatusTypeDef BQ76952::ClearManualBalancing(){
    return SubcmdCmdWriteU2(BQ769X2_SUBCMD_CB_ACTIVE_CELLS, 0x0000);
}

HAL_StatusTypeDef BQ76952::Shutdown() {
    return SubcmdCmdOnly(BQ769X2_SUBCMD_SHUTDOWN);
}

HAL_StatusTypeDef BQ76952::EnterDeepSleep(){
    HAL_StatusTypeDef status = SubcmdCmdOnly(BQ769X2_SUBCMD_DEEPSLEEP);
    if (status != HAL_OK) 
        return status;
    status = SubcmdCmdOnly(BQ769X2_SUBCMD_DEEPSLEEP);
    if (status != HAL_OK) 
        return status;
    
    current_mode_ = BQ_MODE_DEEPSLEEP;
    
    return status;
}

HAL_StatusTypeDef BQ76952::ExitDeepSleep(){
    HAL_StatusTypeDef status = SubcmdCmdOnly(BQ769X2_SUBCMD_EXIT_DEEPSLEEP);
    if (status != HAL_OK) 
        return status;
    current_mode_ = BQ_MODE_NORMAL;

    return status;
}

HAL_StatusTypeDef BQ76952::ModifySleepCurrentBoundary(int16_t boundary){
    HAL_StatusTypeDef status = ConfigUpdate(true);
    if (status != HAL_OK) 
        return status;

    return DatamemWriteI2(BQ769X2_PWR_SLEEP_CURRENT, boundary);
}

int16_t BQ76952::GetPackCurrent(){
	return pack_current_;
}

int16_t BQ76952::GetCellVoltage(uint32_t cell_num) {
    if (cell_num > 16) {
        return INT16_MIN;
    }

    return cell_voltages_[cell_num];
}

int16_t BQ76952::GetPackVoltage() {
    return pack_voltage_;
}

int16_t BQ76952::GetStackVoltage() {
    return stack_voltage_;
}

int16_t BQ76952::GetAvgCellVoltage() {
    return avg_cell_voltage_;
}

int16_t BQ76952::GetHighCellVoltage() {
    return high_cell_voltage_;
}

int16_t BQ76952::GetLowCellVoltage() {
    return low_cell_voltage_;
}

bool BQ76952::GetConfigUpdateStatus(){
    return current_mode_ == BQ_MODE_CONFIGUPDATE;
}

HAL_StatusTypeDef BQ76952::WriteBytes(const uint8_t reg_addr, const uint8_t *data, const size_t num_bytes) {
    uint8_t buf[5];

    if (num_bytes > 4){
        return HAL_ERROR;
    }

    buf[0] = reg_addr;
    memcpy(buf + 1, data, num_bytes);

    return HAL_I2C_Master_Transmit(hi2c_, i2cAddressWrite, buf, num_bytes + 1, 1000);
}

HAL_StatusTypeDef BQ76952::ReadBytes(uint8_t reg_addr, uint8_t *data, const size_t num_bytes) {
    HAL_I2C_Master_Transmit(hi2c_, i2cAddressRead, &reg_addr, 1, 1000);
    
    return HAL_I2C_Master_Receive(hi2c_, i2cAddressRead, data, num_bytes, 1000);
}

HAL_StatusTypeDef BQ76952::DirectReadU2(const uint8_t reg_addr, uint16_t *value) {
    uint8_t buf[2];

    HAL_StatusTypeDef status = ReadBytes(reg_addr, buf, 2);

    if (status != HAL_OK) 
        return status;

    *value = (buf[1] << 8) | buf[0];

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::DirectReadI2(const uint8_t reg_addr, int16_t *value) {
    uint8_t buf[2];

    HAL_StatusTypeDef status = ReadBytes(reg_addr, buf, 2);

    if (status != HAL_OK) 
        return status;

    *value = (int16_t)((buf[1] << 8) | buf[0]);

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdRead(const uint16_t subcmd, uint32_t *value, const size_t num_bytes) {
    static uint8_t buf_data[0x20];

    uint8_t buf_subcmd[2] = { (uint8_t)subcmd, (uint8_t)(subcmd >> 8)}; // put subcmd into a buffer

    HAL_StatusTypeDef status = WriteBytes(BQ769X2_CMD_SUBCMD_LOWER, buf_subcmd, 2);
    if (status != HAL_OK)
        return status;

    HAL_Delay(1);

    int num_tries = 0;
    while(1){
        status = ReadBytes(BQ769X2_CMD_SUBCMD_LOWER, buf_data, 2);

        if (status != HAL_OK){
            return status;
        } else if(num_tries > 10){
            return HAL_ERROR;
        }
        else {
            if(buf_subcmd[0] != buf_data[0] || buf_subcmd[1] != buf_data[1]){
                HAL_Delay(1);
                num_tries++;
            }else{
                break;
            }
        }
    }

    uint8_t data_length;

    status = ReadBytes(BQ769X2_SUBCMD_DATA_LENGTH, &data_length, 1);
    if (status != HAL_OK)
        return status;

    data_length -= 4; // subtract subcmd + checksum + length bytes

    if(data_length > 0x20 || num_bytes > 4){
        return HAL_ERROR; // error
    }

    *value = 0;
    status = ReadBytes(BQ769X2_SUBCMD_DATA_START, buf_data, data_length);
    if (status != HAL_OK)
        return status;

    for(uint8_t i = 0; i < num_bytes; i++){
        *value += buf_data[i] << (i * 8);
    }

    //    just for testing:
//    uint8_t checksum;
//    status = ReadBytes(BQ769X2_SUBCMD_DATA_CHECKSUM, &checksum, 1);

    return status;
}

HAL_StatusTypeDef BQ76952::SubcmdReadU1(const uint16_t subcmd, uint8_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 1);

    if (status != HAL_OK) 
        return status;

    *value = (uint8_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadU2(const uint16_t subcmd, uint16_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 2);

    if (status != HAL_OK) 
        return status;

    *value = (uint16_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadU4(const uint16_t subcmd, uint32_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 4);

    if (status != HAL_OK)
        return status;

    *value = temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadI1(const uint16_t subcmd, int8_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 1);

    if (status != HAL_OK) 
        return status;

    *value = (int8_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadI2(const uint16_t subcmd, int16_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 2);

    if (status != HAL_OK) 
        return status;

    *value = (int16_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadI4(const uint16_t subcmd, int32_t *value) {
    uint32_t temp;

    HAL_StatusTypeDef status = SubcmdRead(subcmd, &temp, 4);

    if (status != HAL_OK) 
        return status;

    *value = (int32_t)temp;

    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdReadF4(const uint16_t subcmd, float *value){
    float f32;
    HAL_StatusTypeDef status = SubcmdRead(subcmd, (uint32_t *)&f32, 4);

    if (status != HAL_OK)
        return status;

    *value = f32;
    return HAL_OK;
}

HAL_StatusTypeDef BQ76952::SubcmdWrite(const uint16_t subcmd, const uint32_t value, const size_t num_bytes) {
    uint8_t buf_data[4];
    uint8_t buf_subcmd[2] = { (uint8_t)(subcmd & 0x00FF), (uint8_t)(subcmd >> 8) };
    uint8_t buf_len_and_check[2];

    HAL_StatusTypeDef status = WriteBytes(BQ769X2_CMD_SUBCMD_LOWER, buf_subcmd, 2);
    if(status != HAL_OK)
        return status;

    if (num_bytes > 4){
        return HAL_ERROR;
    }

    uint8_t checksum = (uint8_t)(buf_subcmd[0] + buf_subcmd[1]);
    if (num_bytes > 0){
        for(int i = 0; i < (int)num_bytes; i++){
            buf_data[i] = (value >> (i * 8) & 0x000000FF);
            checksum = (uint8_t)(checksum + buf_data[i]);
        }
        status = WriteBytes(BQ769X2_SUBCMD_DATA_START, buf_data, num_bytes);
    }
    checksum = ~checksum;

    if (status != HAL_OK)
    	return status;

    buf_len_and_check[1] = num_bytes + 4;
    buf_len_and_check[0] = checksum;

    status = WriteBytes(BQ769X2_SUBCMD_DATA_CHECKSUM, buf_len_and_check, 2);

    return status;
}

HAL_StatusTypeDef BQ76952::SubcmdCmdOnly(const uint16_t subcmd) {
    return SubcmdWrite(subcmd, 0, 0);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteU1(const uint16_t subcmd, uint8_t value) {
    return SubcmdWrite(subcmd, value, 1);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteU2(const uint16_t subcmd, uint16_t value) {
    return SubcmdWrite(subcmd, value, 2);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteU4(const uint16_t subcmd, uint32_t value) {
    return SubcmdWrite(subcmd, value, 4);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteI1(const uint16_t subcmd, int8_t value) {
    return SubcmdWrite(subcmd, value, 1);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteI2(const uint16_t subcmd, int16_t value) {
    return SubcmdWrite(subcmd, value, 2);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteI4(const uint16_t subcmd, int32_t value) {
    return SubcmdWrite(subcmd, value, 4);
}

HAL_StatusTypeDef BQ76952::SubcmdCmdWriteF4(const uint16_t subcmd, float value) {
    uint32_t *u32 = (uint32_t *)&value;
    return SubcmdWrite(subcmd, *u32, 4);
}

HAL_StatusTypeDef BQ76952::DatamemReadU1(const uint16_t reg_addr, uint8_t *value){
    if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr)){
        return HAL_ERROR;
    }
    uint32_t u32;
    HAL_StatusTypeDef status = SubcmdRead(reg_addr, &u32, 1);
    if(status == HAL_OK){
        *value = (uint8_t)u32;
    }
    return status;
}
HAL_StatusTypeDef BQ76952::DatamemWriteU1(const uint16_t reg_addr, uint8_t value){
    if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !(current_mode_ == BQ_MODE_CONFIGUPDATE)){
        return HAL_ERROR;
    }
    return SubcmdWrite(reg_addr, value, 1);
}
HAL_StatusTypeDef BQ76952::DatamemWriteU2(const uint16_t reg_addr, uint16_t value){
    if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !(current_mode_ == BQ_MODE_CONFIGUPDATE)){
        return HAL_ERROR;
    }
    return SubcmdWrite(reg_addr, value, 2);
}
HAL_StatusTypeDef BQ76952::DatamemWriteI1(const uint16_t reg_addr, int8_t value){
    if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !(current_mode_ == BQ_MODE_CONFIGUPDATE)){
        return HAL_ERROR;
    }
    return SubcmdWrite(reg_addr, value	, 1);
}
HAL_StatusTypeDef BQ76952::DatamemWriteI2(const uint16_t reg_addr, int16_t value){
    if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !(current_mode_ == BQ_MODE_CONFIGUPDATE)){
        return HAL_ERROR;
    }
    return SubcmdWrite(reg_addr, value, 2);
}
HAL_StatusTypeDef BQ76952::DatamemWriteF4(const uint16_t reg_addr, float value){
    if(!BQ769X2_IS_DATA_MEM_REG_ADDR(reg_addr) || !(current_mode_ == BQ_MODE_CONFIGUPDATE)){
        return HAL_ERROR;
    }

    uint32_t *u32 = (uint32_t *)&value;

    return SubcmdWrite(reg_addr, *u32, 4);
}

HAL_StatusTypeDef BQ76952::UpdateMode(){
    uint16_t buf;
    HAL_StatusTypeDef status = DirectReadU2(BQ769X2_CMD_BATTERY_STATUS, &buf);
    if(status != HAL_OK)
        return HAL_ERROR;

    if(buf & (1 << 15)){
        current_mode_ = BQ_MODE_SLEEP;
        return status;
    }else if(buf & (1 << 0)){
        current_mode_ = BQ_MODE_CONFIGUPDATE;
        return status;
    }

    status = DirectReadU2(BQ769X2_CMD_CONTROL_STATUS, &buf);
    if(status != HAL_OK)
        return HAL_ERROR;

    if(buf & (1 << 2)){
        current_mode_ = BQ_MODE_DEEPSLEEP;
        return status;
    }

    current_mode_ = BQ_MODE_NORMAL;
    return status;
}

#endif
