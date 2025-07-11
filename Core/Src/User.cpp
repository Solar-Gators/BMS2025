// includes
#include "User.hpp"
#include "BQChips.hpp"
#include "BMS.hpp"
#include "stdio.h"

#include <cmath>

//define chips addresses
#define bqChipI2CAddress 0x10

#define USB_BUFLEN 128

//C extern handles
extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;

extern "C" I2C_HandleTypeDef hi2c2;
extern "C" I2C_HandleTypeDef hi2c3;
extern "C" I2C_HandleTypeDef hi2c4;

extern "C" TIM_HandleTypeDef htim2;
extern "C" TIM_HandleTypeDef htim3;


//CAN  rx stuff
CAN_RxHeaderTypeDef RxHeader;
uint8_t datacheck = 0;
uint8_t RxData[8];  // Array to store the received data


//gloabal variables
bool openContactorsDebug;
bool shutdown;
bool closed;

bool VT = 0;

errorState faultCondition = noFault;
uint8_t currentDirrection;
uint8_t fanSpeedPercentage;

uint16_t current_adc_vals[8];

ADS7138 current_adc;
ADS7138 temp_adcs[4];

//global structs
BMSData BMS;
BMS_Data_t data;

BQ76952 bqChip1 = BQ76952(); // 16 cells = i2c4
BQ76952 bqChip2 = BQ76952(); // 13 cells = i2c3

BQChips bqChips = BQChips(&bqChip1, &bqChip2);


//Conversion Unions
union FloatBytes {
    float value;
    uint8_t bytes[4];
};
union FloatBytes fb;

union uint32Bytes {
	uint32_t value;
	uint8_t bytes[4];
};
union uint32Bytes numBytes;

char buffer[512];  // Make sure this is large enough for your data
int pos = 0;

typedef struct {
    uint32_t voltage_exclusions;   // 32 bits for voltage exclusions
    uint32_t temp_exclusions;      // 32 bits for temperature exclusions
} Exclusion_Data_t;


void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    openContactorsDebug = false;
    closed = false;

    //set contactor pins low
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    //set power mux
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

    //toggle BQ reset
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(200);

    //toggle BQ reset
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

	//initalize BMS ICs
	if (HAL_StatusTypeDef::HAL_OK != bqChip1.Init(&hi2c4, bqChipI2CAddress)) {
		Error_Handler();
	}
	if (HAL_StatusTypeDef::HAL_OK != bqChip2.Init(&hi2c3, bqChipI2CAddress)) {
		Error_Handler();
	}

	//initalize current ADC
	current_adc.begin(&hi2c2, 0x10); // Default address: 0x10
	// Configure operating mode (example: internal oscillator, manual mode)
	current_adc.configureOpMode(OSC_SEL_LOW_POWER, CONV_MODE_MANUAL, CONV_ON_ERR_CONTINUE);
	// Use manual channel selection
	current_adc.configureSequenceMode(SEQ_MODE_MANUAL, SEQ_START_END);
	// Set oversampling to 1 (no averaging)
	current_adc.configureOsr(OSR_1);
	// Set reference voltage (e.g., 3300 mV if powered from 3.3 V)
	current_adc.setReferenceVoltage(3300);

    //temp sensor inits
    for (int i = 0; i < 4; i++) {
    	temp_adcs[i].begin(&hi2c2, 0x14+i);
    	temp_adcs[i].configureOpMode(OSC_SEL_LOW_POWER, CONV_MODE_MANUAL, CONV_ON_ERR_CONTINUE);
    	temp_adcs[i].configureSequenceMode(SEQ_MODE_MANUAL, SEQ_START_END);
    	temp_adcs[i].configureOsr(OSR_1);
    	temp_adcs[i].setReferenceVoltage(3300);
    }

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim3);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

}

void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

// CURRENT MONITORING TASK
void StartTask02(void *argument) {

	uint32_t rawData;
	float low;

	for (;;)
	{

		if (hi2c2.State == HAL_I2C_STATE_READY) {

			rawData = current_adc.readChannelVoltage((ADS7138__MANUAL_CHID)(MANUAL_CHID_AIN0));
			low  = ADCToCurrentL(rawData);

			// If positive current, discharging
			if (abs(low) == low) {
				currentDirrection = discharging;
				if (low > 26) {
					faultCondition = overCurrentCharge;
				}
			} else { // Else current will be negative, thus charging
				currentDirrection = charging;
				if (abs(low) > 60) {
					faultCondition = overCurrentDischarge;
				}
			}
		}

		BMS.lowCurrent_A = low;
		fb.value = low;

		osDelay(50);
	}
}

// VOLTAGE MONITORING TASK
void StartTask03(void *argument) {
    int16_t cellVoltages[32] = {0};
    uint16_t highestCell = 0;
    uint16_t lowestCell = 10000;
    BMS.highVoltageIndex = 0;
    BMS.lowVoltageIndex = 0;
    uint32_t total = 0;
    uint8_t active_cell_count = 0;  // Count of non-excluded cells

    for(;;) {
        total = 0;
        highestCell = 0;
        lowestCell = 10000;
        active_cell_count = 0;

        bqChips.readVoltages();
        bqChips.getAll32CellVoltages(cellVoltages);

        for (int i = 0; i < 32; i++) {
            // Skip if cell is excluded from voltage monitoring
        	if (BMS.voltageExclusionList[i] != 0) {
        		continue;
        	}

            BMS.cellVoltages[i] = bqChips.getCellVoltage(i);

            if (BMS.cellVoltages[i] > highestCell) {
                highestCell = BMS.cellVoltages[i];
                BMS.highVoltageIndex = i;
            }

            if(BMS.cellVoltages[i] < lowestCell) {
                lowestCell = BMS.cellVoltages[i];
                BMS.lowVoltageIndex = i;
            }

            total += BMS.cellVoltages[i];
            active_cell_count++;
        }

        // Only calculate average if we have active cells
        if (active_cell_count > 0) {
            BMS.totalVoltage_mV = total;
            BMS.avgVoltage_mV = (uint16_t)(total/active_cell_count);
            BMS.lowVoltage_mV = lowestCell;
            BMS.highVoltage_mV = highestCell;

            numBytes.value = total;

            // Only check voltage limits for non-excluded cells
            if (lowestCell < 2500) {
                faultCondition = lowCellVoltage;
            }
            if (highestCell > 4200) {
                faultCondition = highCellVoltage;
            }
        }

        osDelay(100);
    }
}

// TEMPERATURE MONITORING TASK
void StartTask04(void *argument) {
    uint32_t rawData[32];
    float highestCell = 0.0;
    float lowestCell = 1000.0;
    BMS.highTempIndex = 0;
    BMS.lowTempIndex = 0;
    float total = 0;
    uint8_t active_temp_count = 0;  // Count of non-excluded temperature sensors

    for(;;) {
        total = 0;
        highestCell = 0.0;
        lowestCell = 1000.0;
        active_temp_count = 0;

        for (int i = 0; i < 4; i++) {
            for (uint8_t ch = 0; ch < 8; ch++) {
                uint8_t sensor_index = i*8 + ch;

                if (BMS.tempExclusionList[i] == 0) {
                	continue;
                }

                if (hi2c2.State == HAL_I2C_STATE_READY) {
                    rawData[sensor_index] = temp_adcs[i].readChannelVoltage((ADS7138__MANUAL_CHID)(MANUAL_CHID_AIN0 + ch));
                    BMS.allTemperatures[sensor_index] = ADCToTemp(rawData[sensor_index]);

                    if (BMS.allTemperatures[sensor_index] > highestCell) {
                        highestCell = BMS.allTemperatures[sensor_index];
                        BMS.highTempIndex = sensor_index;
                    }
                    if (BMS.allTemperatures[sensor_index] < lowestCell) {
                        lowestCell = BMS.allTemperatures[sensor_index];
                        BMS.lowTempIndex = sensor_index;
                    }
                    total += BMS.allTemperatures[sensor_index];
                    active_temp_count++;
                }
            }
        }

        // Only calculate average if we have active temperature sensors
        if (0 >= active_temp_count) {
            osDelay(1000);
            continue;
        }

        BMS.avgTemp = total/active_temp_count;
        BMS.lowTemp = lowestCell;
        BMS.highTemp = highestCell;

        // Only check temperature limits for non-excluded sensors
        if (currentDirrection == charging) {
            if (highestCell > 45) {
                faultCondition = overTempCharge;
            }
        }
        if (currentDirrection == discharging) {
            if (highestCell > 60) {
                faultCondition = overTempDischarge;
            }
        }

        osDelay(1000);
    }
}

void StartTask05(void *argument) {

    //setup CAN TX header
    HAL_StatusTypeDef status;
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = { 0 };
    uint32_t TxMailbox = { 0 };

    /* Infinite loop */
    for(;;) {

//        TxData[0] = fb.bytes[0];
//        TxData[1] = fb.bytes[1];
//        TxData[2] = fb.bytes[2];
//        TxData[3] = fb.bytes[3];

    	setUpCAN1(TxHeader, TxData);

        while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
        status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
        if (status == HAL_ERROR) {
            Error_Handler();
        }

        setUpCAN2(TxHeader, TxData);

		while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
		status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		if (status == HAL_ERROR) {
			Error_Handler();
		}

		setUpCAN3(TxHeader, TxData);

		while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
		status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		if (status == HAL_ERROR) {
			Error_Handler();
		}

        osDelay(100);
    }
}

void StartTask06(void *argument) {

    for(;;) {

	    //control contactors
    	if (shutdown) {
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_Delay(500); // ? May replace later
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			closed = false;
    	} else if (openContactorsDebug == true || ((faultCondition == noFault) && (shutdown == false))) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_Delay(500); // ? May replace later
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
            closed = true;
	    } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_Delay(500); // ? May replace later
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
            closed = false;
	    }

        fanSpeedPercentage = 50;
        //set fan speeds
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, fanSpeedPercentage/2.5);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fanSpeedPercentage/2.5);

        send_bms_data(BMS.cellVoltages, BMS.allTemperatures, BMS.lowCurrent_A);

      osDelay(100);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
        Error_Handler();
    }

    if (0x7FF == RxHeader.StdId) {
    	if ((RxData[1] & 0x08) != 0x00) {
			openContactorsDebug = true;
		} else {
			openContactorsDebug = false;
		}
    }

    if (0x7 == RxHeader.StdId) {
    	if (RxData[1] & (1 << 4)) {
    		shutdown = true;
    	} else {
    		shutdown = false;
    	}
    }

}

float ADCToCurrentL(uint32_t adc_val) {
    // Constant slope for linear estimator
    static constexpr float m = 0.001894;

    // Constant offset for linear estimator
    static constexpr float b = -62.87;

    // Convert ADC value to current
    return (float)adc_val * m + b;
}

/* Converts raw ADC value to current in A for high channel */
float ADCToCurrentH(uint32_t adc_val) {
    // Constant slope for linear estimator
    static constexpr float m = 0.007609;

    // Constant offset for linear estimator
    static constexpr float b = -252.4;

    // Convert ADC value to current
    return (float)adc_val * m + b;
}


float ADCToTemperature(uint32_t adc_val) {
	// temperature = Ax^2 + Bx + C, where x = voltage across adc
	static constexpr float thermCoeffA = 11.49;
	static constexpr float thermCoeffB = -61.03;
	static constexpr float thermCoeffC = 93.56;

	float x = ((float)adc_val) * (3.3/4096.0);

	return (thermCoeffA * x*x) + (thermCoeffB * x) + thermCoeffC;
}

float ADCToTemp(uint32_t adc_val) {
    // Constant slope for linear estimator
    static constexpr float m = 1.0 / 1180;

    // Constant offset for linear estimator
    static constexpr float b = 19000.0 / 1180;

    // Convert ADC value to temperature
    return (float)adc_val * m + b;
}



void setUpCAN1(CAN_TxHeaderTypeDef &Header, uint8_t* data){
	Header.IDE = CAN_ID_STD; // Standard ID (not extended)
	Header.StdId = 0x4; // 11 bit Identifier
	Header.RTR = CAN_RTR_DATA; // Std RTR Data frame
	Header.DLC = 8; // 8 bytes being transmitted

	data[0] = numBytes.bytes[0]; // Voltage
	data[1] = numBytes.bytes[1];
	data[2] = numBytes.bytes[2];
	data[3] = numBytes.bytes[3];

	data[4] = fb.bytes[0]; // Current
	data[5] = fb.bytes[1];
	data[6] = fb.bytes[2];
	data[7] = fb.bytes[3];

}

void setUpCAN2(CAN_TxHeaderTypeDef &Header, uint8_t* data){
	Header.IDE = CAN_ID_STD; // Standard ID (not extended)
	Header.StdId = 0x5; // 11 bit Identifier
	Header.RTR = CAN_RTR_DATA; // Std RTR Data frame
	Header.DLC = 8; // 8 bytes being transmitted

	data[0] = (uint8_t)BMS.lowVoltage_mV;
	data[1] = (uint8_t)(BMS.lowVoltage_mV >> 8);
	data[2] = (uint8_t)BMS.highVoltage_mV;
	data[3] = (uint8_t)(BMS.highVoltage_mV >> 8);

	data[4] = (uint8_t)(BMS.highTemp * 1000.0f + 0.5f);
	data[5] = ((uint16_t)(BMS.highTemp * 1000.0f + 0.5f)) >> 8;
	data[6] = BMS.lowVoltageIndex;
	data[7] = BMS.highTempIndex;
}

void setUpCAN3(CAN_TxHeaderTypeDef &Header, uint8_t* data){
	Header.IDE = CAN_ID_STD; // Standard ID (not extended)
	Header.StdId = 0x6; // 11 bit Identifier
	Header.RTR = CAN_RTR_DATA; // Std RTR Data frame
	Header.DLC = 8; // 8 bytes being transmitted

	data[0] = faultCondition;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;

	data[4] = 0;
	data[5] = closed;
}

void send_bms_data(uint16_t* cell_voltages, float* temperatures, float current) {


    // Copy data into structure
    for(int i = 0; i < 32; i++) {
        data.voltages[i] = cell_voltages[i];
        data.temperatures[i] = temperatures[i];
    }
    data.current = current;

    // Send the entire structure as raw data
    CDC_Transmit_FS((uint8_t*)&data, sizeof(BMS_Data_t));
}


