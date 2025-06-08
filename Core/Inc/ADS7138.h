//**********************************************************************************
/**
  @file     ADS7138.h

  Author: Adapted by ChatGPT from Dan Alvarez's Arduino version

  STM32 HAL-based library for the TI ADS7138 SAR ADC.

  Register definitions based on SBAC286.
*/
//**********************************************************************************

#ifndef __ADS7138_H
#define __ADS7138_H

#include "stm32l4xx_hal.h"  // Replace with your actual device header, e.g., stm32l4xx_hal.h
#include <stdint.h>

/** Default 7-bit I2C address of ADS7138 */
#define ADS7138_I2CADDR_DEFAULT 0x10

//**********************************************************************************
// Device command
//**********************************************************************************
#define SINGLE_REGISTER_WRITE  ((uint8_t) 0b00001000)

//**********************************************************************************
// Register and Bitfield Definitions
//**********************************************************************************

// ----------------- OSR_CFG (0x03) -----------------
#define OSR_CFG_ADDRESS       ((uint8_t) 0x03)
#define OSR_MASK              ((uint8_t) 0x07)
typedef enum {
    OSR_1   = 0x00,
    OSR_2   = 0x01,
    OSR_4   = 0x02,
    OSR_8   = 0x03,
    OSR_16  = 0x04,
    OSR_32  = 0x05,
    OSR_64  = 0x06,
    OSR_128 = 0x07
} ADS7138__OSR;

// ----------------- OPMODE_CFG (0x04) -----------------
#define OPMODE_CFG_ADDRESS    ((uint8_t) 0x04)
#define CONV_ON_ERR_MASK      ((uint8_t) 0x80)
#define CONV_MODE_MASK        ((uint8_t) 0x60)
#define OSC_SEL_MASK          ((uint8_t) 0x10)
#define CLK_DIV_MASK          ((uint8_t) 0x0F)

typedef enum {
    CONV_ON_ERR_CONTINUE = 0x00,
    CONV_ON_ERR_PAUSE    = 0x80
} ADS7138__CONV_ON_ERR;

typedef enum {
    CONV_MODE_MANUAL = 0x00,
    CONV_MODE_AUTO   = 0x20
} ADS7138__CONV_MODE;

typedef enum {
    OSC_SEL_HIGH_SPEED = 0x00,
    OSC_SEL_LOW_POWER  = 0x10
} ADS7138__OSC_SEL;

// ----------------- SEQUENCE_CFG (0x10) -----------------
#define SEQUENCE_CFG_ADDRESS  ((uint8_t) 0x10)
#define SEQ_START_MASK        ((uint8_t) 0x10)
#define SEQ_MODE_MASK         ((uint8_t) 0x03)

typedef enum {
    SEQ_START_END    = 0x00,
    SEQ_START_ASSEND = 0x10
} ADS7138__SEQ_START;

typedef enum {
    SEQ_MODE_MANUAL = 0x00,
    SEQ_MODE_AUTO   = 0x01
} ADS7138__SEQ_MODE;

// ----------------- MANUAL_CH_SEL (0x11) -----------------
#define MANUAL_CH_SEL_ADDRESS ((uint8_t) 0x11)
#define MANUAL_CHID_MASK      ((uint8_t) 0x0F)

typedef enum {
    MANUAL_CHID_AIN0 = 0x00,
    MANUAL_CHID_AIN1 = 0x01,
    MANUAL_CHID_AIN2 = 0x02,
    MANUAL_CHID_AIN3 = 0x03,
    MANUAL_CHID_AIN4 = 0x04,
    MANUAL_CHID_AIN5 = 0x05,
    MANUAL_CHID_AIN6 = 0x06,
    MANUAL_CHID_AIN7 = 0x07
} ADS7138__MANUAL_CHID;

//**********************************************************************************
// ADS7138 Class Definition
//**********************************************************************************

class ADS7138 {
public:
    ADS7138();

    // Init with STM32 HAL I2C handle and device address (7-bit)
    void begin(I2C_HandleTypeDef* hi2c, uint8_t deviceAddress = ADS7138_I2CADDR_DEFAULT);

    void setReferenceVoltage(uint32_t reference);  // in millivolts

    // Configuration functions
    void configureOpMode(ADS7138__OSC_SEL oscSel, ADS7138__CONV_MODE convMode, ADS7138__CONV_ON_ERR convOnErr);
    void configureSequenceMode(ADS7138__SEQ_MODE seqMode, ADS7138__SEQ_START seqStart);
    void configureOsr(ADS7138__OSR osr);

    // Channel selection and reading
    void selectChannel(ADS7138__MANUAL_CHID channel);
    uint16_t read();  // Read raw ADC result
    uint16_t readChannel(ADS7138__MANUAL_CHID channel);
    uint32_t readVoltage();  // Convert to mV
    uint32_t readChannelVoltage(ADS7138__MANUAL_CHID channel);

    // Register write helper
    void writeRegister8(uint8_t reg, uint8_t val);

private:
    I2C_HandleTypeDef* _i2cHandle;
    uint8_t _deviceAddress;           // 8-bit STM32-compatible I2C address
    uint32_t _referenceVoltage;       // in millivolts
    ADS7138__OSR _currentOsr;
    ADS7138__MANUAL_CHID _currentChannel;
};

#endif  // __ADS7138_H
