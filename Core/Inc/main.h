/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_nucleo.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// NOTE: This should be identical to the enum used by the body segments
typedef enum peripheralType
{
	PERIPHERAL_NONE = 0,
	PERIPHERAL_TEMP_HUMIDITY_DHT11 = 1,
	PERIPHERAL_TEMP_PRESSURE_ALTITUDE_BMP180 = 2
} peripheralType;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR_EN_L_Pin GPIO_PIN_2
#define PWR_EN_L_GPIO_Port GPIOC
#define HEARTBEAT_LED_Pin GPIO_PIN_3
#define HEARTBEAT_LED_GPIO_Port GPIOC
#define TOF_INT_R_Pin GPIO_PIN_0
#define TOF_INT_R_GPIO_Port GPIOA
#define TOF_INT_R_EXTI_IRQn EXTI0_IRQn
#define TOF_PWN_EN_R_Pin GPIO_PIN_1
#define TOF_PWN_EN_R_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TOF_LPn_R_Pin GPIO_PIN_4
#define TOF_LPn_R_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOA
#define TOF_I2C_RST_R_Pin GPIO_PIN_0
#define TOF_I2C_RST_R_GPIO_Port GPIOB
#define TOF_I2C_RST_L_Pin GPIO_PIN_8
#define TOF_I2C_RST_L_GPIO_Port GPIOA
#define TOF_LPn_L_Pin GPIO_PIN_9
#define TOF_LPn_L_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TOF_INT_L_Pin GPIO_PIN_7
#define TOF_INT_L_GPIO_Port GPIOB
#define TOF_INT_L_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
