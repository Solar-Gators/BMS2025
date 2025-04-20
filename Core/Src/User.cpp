#include "User.hpp"
#include "BQChips.hpp"

extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;

extern "C" I2C_HandleTypeDef hi2c2;



CAN_RxHeaderTypeDef RxHeader;

uint8_t datacheck = 0;
uint8_t RxData[8];  // Array to store the received data

bool contactors_on;

uint16_t adc_vals[8];
ADS7138 adc = ADS7138(&hi2c2, 0x10);

uint8_t bqChip1I2CAddress = 0x12; // default is 0x10, should configure to something else if adc is already using that
uint8_t bqChip2I2CAddress = 0x14;

BQ76952 bqChip1 = BQ76952();
BQ76952 bqChip2 = BQ76952();

bqChip1.Init(&hi2c2, bqChip1I2CAddress);
bqChip2.Init(&hi2c2, BQChip2I2CAddress);
BQChips bqchips = BQChips(&bqChip1, &bqChip2);

union FloatBytes {
    float value;
    uint8_t bytes[4];
};

union FloatBytes fb;



void CPP_UserSetup(void) {
    // Make sure that timer priorities are configured correctly
    HAL_Delay(10);


    contactors_on = false;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);



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
	  //HAL_UART_Receive(&huart4, UART4_rxBuffer, 1, HAL_MAX_DELAY);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
// CURRENT MONITORING TASK
	uint16_t rawData;

	float low;

  for (;;)
  {

	  adc.ConversionReadAutoSequence(&rawData, 1);
	  low  = ADCToCurrentL(rawData);
	  fb.value = low;


    osDelay(20);
  }
  /* USER CODE END StartTask02 */
}

void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
// VOLTAGE MONITORING TASK

	bqChips.readVoltages();
	/* Infinite loop */
	for(;;)
	{

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
  /* USER CODE BEGIN StartTask05 */
// CAN DATA TRANSMISSION TASK

  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8] = { 0 };
  uint32_t TxMailbox = { 0 };
  int HAL_CAN_BUSY = 0;
  uint64_t messages_sent = 0;

  TxHeader.IDE = CAN_ID_STD; // Standard ID (not extended)
  TxHeader.StdId = 0x4; // 11 bit Identifier
  TxHeader.RTR = CAN_RTR_DATA; // Std RTR Data frame
  TxHeader.DLC = 8; // 8 bytes being transmitted
  TxData[0] = 4;



  /* Infinite loop */
  for(;;)
  {
	  TxData[1] = fb.bytes[0];
	  TxData[2] = fb.bytes[1];
	  TxData[3] = fb.bytes[2];
	  TxData[4] = fb.bytes[3];

	  while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
	  HAL_StatusTypeDef status;
	  status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	  messages_sent++;
	  if (status == HAL_ERROR)
	  {
		  Error_Handler();
	  }
	  else if (status == HAL_BUSY)
	  {
	  HAL_CAN_BUSY++;
	  }


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


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }

  if (RxHeader.StdId == 0x7FF){
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
  }
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


