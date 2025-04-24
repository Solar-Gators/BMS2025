#include "User.hpp"
#include "BQChips.hpp"



//needed so that the C++ compiler recongnuzes these a C tpye stucts
extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;

extern "C" I2C_HandleTypeDef hi2c2;
extern "C" I2C_HandleTypeDef hi2c3;
extern "C" I2C_HandleTypeDef hi2c4;


//initalizes CAN RX header
CAN_RxHeaderTypeDef RxHeader;
uint8_t datacheck = 0;
uint8_t RxData[8];  // Array to store the received data

//bolean to store contactor state
bool contactors_on;

//adc object
ADS7138 adc = ADS7138(&hi2c2, 0x10);


BQ76952 bqChip1 = BQ76952(); // 16 cells = i2c3
BQ76952 bqChip2 = BQ76952(); // 13 cells = i2c1

BQChips bqChips = BQChips(&bqChip1, &bqChip2);

union FloatBytes {
    float value;
    uint8_t bytes[4];
};
union FloatBytes fb;

void RxCallback(uint8_t* RxData)
{
//  if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (RxHeader.StdId == 0x7FF){
  if(RxData[0] == 1){
	  //byte 1
	  //ignition switch
	  if((RxData[1] & 0x80) != 0x00){
		  //preform shut down sequence
	  }

	  if((RxData[1] & 0x08) != 0x00){
		  contactors_on = true; // turn brakes on
	  }else{
		  contactors_on = false; // turn breaks off
	  }



  }
//  }
}

void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);

    //set conactors to be off
    contactors_on = false;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

	uint8_t bqChip1I2CAddress = 0x10; // default is 0x10, should configure to something else if adc is already using that
	uint8_t bqChip2I2CAddress = 0x10;
	bqChip1.Init(&hi2c3, bqChip1I2CAddress);
	bqChip2.Init(&hi2c4, bqChip2I2CAddress);

    //setup current ADCs
    adc.Init();
    adc.ConfigureOpmode(false, ConvMode_Type::MANUAL);
    adc.ConfigureData(false, DataCfg_AppendType::ID);
    adc.AutoSelectChannels((0x1 << 0));

}


void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	//toggle OK Led at 1Hz
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
    //setup
	uint16_t rawData;


  for (;;)
  {
	  //take current readings
	  adc.ConversionReadAutoSequence(&rawData, 1);
	  fb.value = ADCToCurrentL(rawData);



    osDelay(20);
  }
  /* USER CODE END StartTask02 */
}

void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
// VOLTAGE MONITORING TASK

	int16_t cellVoltages[29] = {0};
	/* Infinite loop */
	for(;;)
	{

	  bqChips.readVoltages();
	  bqChips.getAll29CellVoltages(cellVoltages);
	  osDelay(50);
  }
  /* USER CODE END StartTask03 */
}


void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
// TEMPERATURE MONITORING TASK

  /* Infinite loop */
  for(;;)
  {



    osDelay(100);
  }
  /* USER CODE END StartTask04 */
}


void StartTask05(void *argument)
{

  //setup CAN TX header
  // CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8] = { 0 };
  // uint32_t TxMailbox = { 0 };

  // TxHeader.IDE = CAN_ID_STD; // Standard ID (not extended)
  // TxHeader.StdId = 0x4; // 11 bit Identifier
  // TxHeader.RTR = CAN_RTR_DATA; // Std RTR Data frame
  // TxHeader.DLC = 8; // 8 bytes being transmitted
  TxData[0] = 4;

  CANFrame ADCMessage(0, 0, 2, 8);
  ADCMessage.LoadData(TxData, 8);

  Controller.Send(&ADCMessage);
  Controller.AddRxMessage(&ADCMessage, &RxCallback);

  //setup stuff for tracking outbound message count
  //int HAL_CAN_BUSY = 0;
  //uint64_t messages_sent = 0;

  /* Infinite loop */
  for(;;)
  {
	  //setup the union
	  TxData[1] = fb.bytes[0];
	  TxData[2] = fb.bytes[1];
	  TxData[3] = fb.bytes[2];
	  TxData[4] = fb.bytes[3];

	  ADCMessage.LoadData(TxData, 8);
	  Controller.Send(&ADCMessage);


	  // while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
	  // HAL_StatusTypeDef status;
	  // status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	  // messages_sent++;
	  // if (status == HAL_ERROR)
	  // {
		//   Error_Handler();
	  // }
	  // else if (status == HAL_BUSY)
	  // {
	  // HAL_CAN_BUSY++;
	  // }


    osDelay(100);
  }

  /* USER CODE END StartTask05 */
}

void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
	  if(contactors_on == true){

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  }else{

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  }
      osDelay(10);
  }
  /* USER CODE END StartTask06 */
}


float ADCToCurrentL(uint16_t adc_val) {
    // Constant slope for linear estimator
    static constexpr float m = 0.001894;

    // Constant offset for linear estimator
    static constexpr float b = -62.87;

    // Convert ADC value to current
    return (float)adc_val * m + b;
}

/* Converts raw ADC value to current in A for high channel */
float ADCToCurrentH(uint16_t adc_val) {
    // Constant slope for linear estimator
    static constexpr float m = 0.007609;

    // Constant offset for linear estimator
    static constexpr float b = -252.4;

    // Convert ADC value to current
    return (float)adc_val * m + b;
}


