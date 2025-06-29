#include "ADS7138.h"

/**
 * @brief Instantiates a new ADS7138 class.
 */
ADS7138::ADS7138() {}

/**
 * @brief Sets up the I2C connection.
 *
 * @param hi2c Pointer to the HAL I2C handle.
 * @param deviceAddress The 7-bit I2C address of the ADS7138.
 */
void ADS7138::begin(I2C_HandleTypeDef* hi2c, uint8_t deviceAddress) {
    _i2cHandle = hi2c;
    _deviceAddress = (deviceAddress != 0) ? deviceAddress << 1 : (ADS7138_I2CADDR_DEFAULT << 1); // STM32 expects 8-bit address
    _currentOsr = OSR_1;
    _referenceVoltage = 3300; // default in mV
    _currentChannel = MANUAL_CHID_AIN0;
}

void ADS7138::setReferenceVoltage(uint32_t reference) {
    _referenceVoltage = reference;
}

void ADS7138::writeRegister8(uint8_t registerAddress, uint8_t value) {
    uint8_t data[3] = { SINGLE_REGISTER_WRITE, registerAddress, value };
    HAL_I2C_Master_Transmit(_i2cHandle, _deviceAddress, data, 3, HAL_MAX_DELAY);
}

void ADS7138::configureOpMode(ADS7138__OSC_SEL oscSel, ADS7138__CONV_MODE convMode, ADS7138__CONV_ON_ERR convOnErr) {
    uint8_t config = (0x00 & CLK_DIV_MASK) | (oscSel & OSC_SEL_MASK) | (convMode & CONV_MODE_MASK) | (convOnErr & CONV_ON_ERR_MASK);
    writeRegister8(OPMODE_CFG_ADDRESS, config);
}

void ADS7138::configureSequenceMode(ADS7138__SEQ_MODE seqMode, ADS7138__SEQ_START seqStart) {
    uint8_t config = (seqMode & SEQ_MODE_MASK) | (seqStart & SEQ_START_MASK);
    writeRegister8(SEQUENCE_CFG_ADDRESS, config);
}

void ADS7138::configureOsr(ADS7138__OSR osr) {
    _currentOsr = osr;
    writeRegister8(OSR_CFG_ADDRESS, osr & OSR_MASK);
}

void ADS7138::selectChannel(ADS7138__MANUAL_CHID channel) {
    _currentChannel = channel;
    writeRegister8(MANUAL_CH_SEL_ADDRESS, channel & MANUAL_CHID_MASK);
}

uint16_t ADS7138::read() {
    uint8_t buf[2] = {0};
    HAL_I2C_Master_Receive(_i2cHandle, _deviceAddress, buf, 2, HAL_MAX_DELAY);

    uint16_t value = 0;
    if (_currentOsr != OSR_1) {
        value = (buf[0] << 4) | (buf[1] >> 4);
    } else {
        value = (buf[0] << 8) | buf[1];
    }
    return value;
}

uint16_t ADS7138::readChannel(ADS7138__MANUAL_CHID channel) {
    if (channel != _currentChannel) {
        selectChannel(channel);
    }
    return read();
}

uint32_t ADS7138::readVoltage() {
    uint16_t raw = read();
    return (static_cast<uint32_t>(raw) * _referenceVoltage) / 4095;
}

uint32_t ADS7138::readChannelVoltage(ADS7138__MANUAL_CHID channel) {
    if (channel != _currentChannel) {
        selectChannel(channel);
    }
    return readVoltage();
}
