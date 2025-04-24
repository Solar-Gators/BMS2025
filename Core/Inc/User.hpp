#ifndef USER_HPP_
#define USER_HPP_

#include <stddef.h>
#include <stdbool.h>
#include "main.h"
#include "ADS7138.hpp"

#include "cmsis_os.h"



void CPP_UserSetup(void);

float ADCToCurrentL(uint16_t adc_val);

/* Converts raw ADC value to current in A for high channel */
float ADCToCurrentH(uint16_t adc_val);


#endif /* USER_HPP_ */
