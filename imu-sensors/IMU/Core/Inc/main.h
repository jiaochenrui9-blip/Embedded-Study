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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IST8310_RSTN_Pin GPIO_PIN_6
#define IST8310_RSTN_GPIO_Port GPIOG
#define IST8310_DRDY_Pin GPIO_PIN_3
#define IST8310_DRDY_GPIO_Port GPIOG
#define Red_OLED_Pin GPIO_PIN_12
#define Red_OLED_GPIO_Port GPIOH
#define Green_OLED_Pin GPIO_PIN_11
#define Green_OLED_GPIO_Port GPIOH
#define Blue_OLED_Pin GPIO_PIN_10
#define Blue_OLED_GPIO_Port GPIOH
#define Buzzer_Port_Pin GPIO_PIN_14
#define Buzzer_Port_GPIO_Port GPIOD
#define BMI088_ACC_CS_Pin GPIO_PIN_4
#define BMI088_ACC_CS_GPIO_Port GPIOA
#define BMI088_GYRO_CS_Pin GPIO_PIN_0
#define BMI088_GYRO_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
