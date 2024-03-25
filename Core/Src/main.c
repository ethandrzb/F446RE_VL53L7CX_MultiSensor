/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_tof.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SEGMENTS 5

#define LAST_SEGMENT_BASE_CAN_ID NUM_SEGMENTS << 4

#define DELAY_TIMER TIM2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;

uint8_t txData[8];
uint8_t rxData[8];

uint32_t txMailbox;

#define UART_RX_BUFFER_SIZE 20
uint8_t UART_Rx_Buffer[UART_RX_BUFFER_SIZE];

#define UART_RESPONSE_LENGTH 50
uint8_t UARTResponseString[UART_RESPONSE_LENGTH];

int8_t turningAngleOffset = 0;
uint32_t targetTurningAnglePWM = 1500;

uint8_t expectedHeartbeatData = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
bool stringToCANMessage(uint8_t *buffer, uint16_t size);
void sendHomingSequence();
void delayMicroseconds(uint32_t usec);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM6)
	{
		if(TIM3->CCR1 > targetTurningAnglePWM)
		{
			TIM3->CCR1--;
		}
		else if(TIM3->CCR1 < targetTurningAnglePWM)
		{
			TIM3->CCR1++;
		}
	}
	else if(htim->Instance == TIM7)
	{
		// Store expected response for comparison in RxFifo0Callback
		// Expected value should be the CAN ID of the node you want to check
		expectedHeartbeatData = (expectedHeartbeatData < LAST_SEGMENT_BASE_CAN_ID) ? expectedHeartbeatData + 0x10 : 0x10;

		txHeader.StdId = expectedHeartbeatData;
		txHeader.DLC = 1;
		txHeader.RTR = CAN_RTR_REMOTE;

		HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData);

	// Check header length
	// Will likely need to be converted to a switch statement as more commands are implemented
	if((rxHeader.DLC == 1) && (rxData[0] == expectedHeartbeatData))
	{
		// Heart beat received if data matches expected value
//		HAL_GPIO_WritePin(CAN_HEARTBEAT_LED_GPIO_Port, CAN_HEARTBEAT_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}
	else if(rxHeader.DLC == 8)
	{
		// Classify peripheral type based on first byte
		switch(rxData[0])
		{
			case PERIPHERAL_NONE:
				sprintf((char *) UARTResponseString, "Segment %lX has no connected peripheral\n", rxHeader.StdId);
				break;
			case PERIPHERAL_TEMP_HUMIDITY_DHT11:
				// Verify checksum
				if(rxData[5] == rxData[1] + rxData[2] + rxData[3] + rxData[4])
				{
					// Reassemble bytes into floating point values like GetData in dht11.h
					sprintf((char *) UARTResponseString, "Segment %lX %.1f degrees C %.1f%% humidity\n",
							rxHeader.StdId,
							rxData[1] + (rxData[2] / 10.0f),
							rxData[3] + (rxData[4] / 10.0f));
				}
				else
				{
					sprintf((char *) UARTResponseString, "Segment %lX Invalid data from DHT11\n", rxHeader.StdId);
					// TODO: Retry once or twice if the checksum is invalid
				}
				break;
		}

		// Send response data over serial (use existing sprintf/strcpy from example project)
		HAL_UART_Transmit(&huart3, UARTResponseString, strlen((char *) UARTResponseString), 100);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

	if(stringToCANMessage(UART_Rx_Buffer, Size))
	{
		strcpy((char *) UARTResponseString, "OK\n");
	}
	else
	{
		strcpy((char *) UARTResponseString, "FAILED\n");
	}

	HAL_UART_Transmit_IT(huart, UARTResponseString, strlen((char *) UARTResponseString));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	HAL_UARTEx_ReceiveToIdle_IT(huart, UART_Rx_Buffer, UART_RX_BUFFER_SIZE);
}

bool stringToCANMessage(uint8_t *buffer, uint16_t size)
{
	// Set individual servo to specific angle
	if(strncmp((char *) buffer, (char *) "SET", 3) == 0)
	{
		unsigned int tmpID = 0;
		unsigned int tmpAngle = 0;

		if(sscanf((char *) buffer, "SET %X %u", &tmpID, &tmpAngle) != 2)
		{
			return false;
		}

		txData[0] = tmpAngle >> 8;
		txData[1] = tmpAngle & 0x00FF;

		txHeader.StdId = tmpID;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.DLC = 2;

		HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

		return true;
	}
	// Control metachronal wave
	else if(strncmp((char *) buffer, (char *) "WAVE", 4) == 0)
	{
		int tmpSpeed = 0;
		int tmpTurningAngleOffset = 0;

		uint8_t numParamsReceived = sscanf((char *) buffer, "WAVE %d %d", &tmpSpeed, &tmpTurningAngleOffset);

		if(numParamsReceived != 1 && numParamsReceived != 2)
		{
			return false;
		}

		if(numParamsReceived == 2)
		{
			// Update turning angle
			turningAngleOffset = tmpTurningAngleOffset;
		}

		txData[0] = tmpSpeed;

		txHeader.StdId = 0xFF;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.DLC = 1;

		HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

		return true;
	}
	// Get peripheral/sensor data from individual segment
	else if(strncmp((char *) buffer, (char *) "GET", 3) == 0)
	{
		unsigned int tmpID = 0;

		if(sscanf((char *) buffer, "GET %X", &tmpID) != 1)
		{
			return false;
		}

		txHeader.StdId = tmpID;
		txHeader.RTR = CAN_RTR_REMOTE;
		txHeader.DLC = 8;

		HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

		return true;
	}
	// Trigger homing sequence
	else if(strncmp((char *) buffer, (char *) "HOME", 4) == 0)
	{
		sendHomingSequence();

		return true;
	}

	return false;
}

void sendHomingSequence()
{
  uint8_t homePosition = 135;

	for(uint8_t id = 0x10; id <= LAST_SEGMENT_BASE_CAN_ID; id += 0x10)
	{
		for(uint8_t servo = 0; servo <= 3; servo++)
		{
			txData[0] = homePosition >> 8;
			txData[1] = homePosition & 0x00FF;

			txHeader.StdId = id + servo;
			txHeader.RTR = CAN_RTR_DATA;
			txHeader.DLC = 2;

			HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

			delayMicroseconds(100000);
		}
	}
}

void delayMicroseconds(uint32_t usec)
{
	DELAY_TIMER->CNT = 0;
	while(DELAY_TIMER->CNT < usec);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_TOF_Init();
  /* USER CODE BEGIN 2 */

  // Initialize turning servo to 135 degrees
  TIM3->CCR1 = 1500;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim6);

  HAL_TIM_Base_Start_IT(&htim7);

  HAL_TIM_Base_Start(&htim2);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // Start UART receive interrupt cycle
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, UART_Rx_Buffer, UART_RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

  MX_TOF_Process();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 72;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canFilterConfig;

  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  canFilterConfig.FilterBank = 0; // 18
  canFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;

  canFilterConfig.FilterIdHigh = 0x020 << 5;	// Can be any value if FilterMaskIDHigh is 0
  canFilterConfig.FilterIdLow = 0;
  //  canFilterConfig.FilterMaskIdHigh = 0x7FC << 5;	// Filter by ID, ignoring 2 LSBs
  canFilterConfig.FilterMaskIdHigh = 0x000 << 5;	// Allow all IDs through filter (i.e., don't check any ID bits)
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.SlaveStartFilterBank = 20;

  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 90-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 180-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 5000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_EN_L_GPIO_Port, PWR_EN_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TOF_PWN_EN_R_Pin|TOF_LPn_R_Pin|LED2_Pin|TOF_I2C_RST_L_Pin
                          |TOF_LPn_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOF_I2C_RST_R_GPIO_Port, TOF_I2C_RST_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PWR_EN_L_Pin */
  GPIO_InitStruct.Pin = PWR_EN_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_EN_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_INT_R_Pin */
  GPIO_InitStruct.Pin = TOF_INT_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_INT_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_PWN_EN_R_Pin TOF_LPn_R_Pin LED2_Pin TOF_I2C_RST_L_Pin
                           TOF_LPn_L_Pin */
  GPIO_InitStruct.Pin = TOF_PWN_EN_R_Pin|TOF_LPn_R_Pin|LED2_Pin|TOF_I2C_RST_L_Pin
                          |TOF_LPn_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_I2C_RST_R_Pin */
  GPIO_InitStruct.Pin = TOF_I2C_RST_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_I2C_RST_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_INT_L_Pin */
  GPIO_InitStruct.Pin = TOF_INT_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_INT_L_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
