#ifndef USER_HPP_
#define USER_HPP_

#include "ADS7138.h"
#include <stddef.h>
#include <stdbool.h>
#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"



void CPP_UserSetup(void);

float ADCToCurrentL(uint32_t adc_val);

/* Converts raw ADC value to current in A for high channel */
float ADCToCurrentH(uint32_t adc_val);

float ADCToTemp(uint32_t adc_val);

void send_bms_data(uint16_t* cell_voltages, float* temperatures, float current);
void setUpCAN1(CAN_TxHeaderTypeDef &Header, uint8_t* data);
void setUpCAN2(CAN_TxHeaderTypeDef &Header, uint8_t* data);
void setUpCAN3(CAN_TxHeaderTypeDef &Header, uint8_t* data);



#endif /* USER_HPP_ */
