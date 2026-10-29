/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DBG_ISR_Pin GPIO_PIN_1
#define DBG_ISR_GPIO_Port GPIOB
#define ADC_V_CNVST_Pin GPIO_PIN_8
#define ADC_V_CNVST_GPIO_Port GPIOC
#define ADC_I_CNVST_Pin GPIO_PIN_9
#define ADC_I_CNVST_GPIO_Port GPIOC
#define DIAG1_Q1_Q4_Pin GPIO_PIN_8
#define DIAG1_Q1_Q4_GPIO_Port GPIOA
#define DIAG2_Q2_Q3_Pin GPIO_PIN_9
#define DIAG2_Q2_Q3_GPIO_Port GPIOA
#define ADC_SCLK_Pin GPIO_PIN_3
#define ADC_SCLK_GPIO_Port GPIOB
#define ADC_SDO_Pin GPIO_PIN_4
#define ADC_SDO_GPIO_Port GPIOB
#define SPI1_MOSI_NC_Pin GPIO_PIN_5
#define SPI1_MOSI_NC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
