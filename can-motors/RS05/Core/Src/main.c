/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RS05_app.h"
#include "RS05_Command.h"
#include "RS05_receive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS05_MIT_TEST_AMPLITUDE_RAD     0.3f
#define RS05_MIT_TEST_KP                5.0f
#define RS05_MIT_TEST_KD                0.3f
#define RS05_MIT_TEST_PERIOD_MS         10U
#define RS05_MIT_TEST_SWITCH_MS         2000U
#define RS05_MIT_FEEDBACK_TIMEOUT_MS    100U
#define RS05_PRIVATE_COMM_SET_PROTOCOL  0x19U
#define RS05_PROTOCOL_MIT               2U
#define RS05_TEST_STAGE_INIT             0U
#define RS05_TEST_STAGE_NEED_POWER_CYCLE 1U
#define RS05_TEST_STAGE_RUNNING          2U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
static RS05_ManagerTypedef RS05_Manager;
static RS05_MIT_MotorTypedef RS05_MIT_Motor1 = {
  .motor_id = RS05_MOTOR_ID,
};
static RS05_MIT_MotorTypedef *const RS05_MIT_Motors[] = {
  &RS05_MIT_Motor1,
};
static volatile HAL_StatusTypeDef RS05_LastStatus = HAL_OK;
static float RS05_StartPositionRad;
static float RS05_TargetPositionRad;
static int8_t RS05_TestDirection = 1;
static uint32_t RS05_LastSwitchTick;
static volatile uint8_t RS05_TestStage = RS05_TEST_STAGE_INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef RS05_RequestMITProtocol(void)
{
  const uint8_t data[8] = {
    0x01U, 0x02U, 0x03U, 0x04U,
    0x05U, 0x06U, RS05_PROTOCOL_MIT, 0x00U
  };

  return RS05_SendData(&hcan1,
                       RS05_PRIVATE_COMM_SET_PROTOCOL,
                       data,
                       RS05_MASTER_ID,
                       RS05_MOTOR_ID);
}

static uint8_t RS05_WaitMITFeedback(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while ((RS05_MIT_Motor1.last_feedback_tick == 0U) &&
         ((HAL_GetTick() - start) < timeout_ms))
  {
  }
  return (uint8_t)(RS05_MIT_Motor1.last_feedback_tick != 0U);
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  RS05_Manager_Init(&RS05_Manager, &hcan1);
  RS05_LastStatus = CAN_Init(&hcan1);
  if (RS05_LastStatus != HAL_OK)
  {
    Error_Handler();
  }


  HAL_Delay(1000U);

  RS05_MIT_Motor1.last_feedback_tick = 0U;
  RS05_LastStatus = RS05_MIT_Stop(&hcan1, RS05_MOTOR_ID);
  if (RS05_LastStatus != HAL_OK)
  {
    Error_Handler();
  }

  if (RS05_WaitMITFeedback(RS05_MIT_FEEDBACK_TIMEOUT_MS) == 0U)
  {
    RS05_LastStatus = RS05_RequestMITProtocol();
    if (RS05_LastStatus != HAL_OK)
    {
      Error_Handler();
    }

    RS05_TestStage = RS05_TEST_STAGE_NEED_POWER_CYCLE;
    while (1)
    {
      HAL_Delay(100U);
    }
  }

  RS05_LastStatus = RS05_MIT_SetRunMode(&hcan1,
                                        RS05_MOTOR_ID,
                                        RS05_MIT_RUN_MODE_MIT);
  if (RS05_LastStatus != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(20U);

  RS05_LastStatus = RS05_MIT_Enable(&hcan1, RS05_MOTOR_ID);
  if (RS05_LastStatus != HAL_OK)
  {
    Error_Handler();
  }

  RS05_StartPositionRad = RS05_MIT_Motor1.position_rad;
  RS05_TargetPositionRad = RS05_StartPositionRad;
  RS05_LastSwitchTick = HAL_GetTick();
  RS05_TestStage = RS05_TEST_STAGE_RUNNING;

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if ((now - RS05_MIT_Motor1.last_feedback_tick) >
        RS05_MIT_FEEDBACK_TIMEOUT_MS)
    {
      (void)RS05_MIT_Stop(&hcan1, RS05_MOTOR_ID);
      Error_Handler();
    }

    if ((now - RS05_LastSwitchTick) >= RS05_MIT_TEST_SWITCH_MS)
    {
      RS05_TargetPositionRad = RS05_StartPositionRad +
          (float)RS05_TestDirection * RS05_MIT_TEST_AMPLITUDE_RAD;
      if (RS05_TargetPositionRad > RS05_MIT_POSITION_MAX_RAD)
      {
        RS05_TargetPositionRad = RS05_MIT_POSITION_MAX_RAD;
      }
      else if (RS05_TargetPositionRad < RS05_MIT_POSITION_MIN_RAD)
      {
        RS05_TargetPositionRad = RS05_MIT_POSITION_MIN_RAD;
      }
      RS05_TestDirection = (int8_t)-RS05_TestDirection;
      RS05_LastSwitchTick = now;
    }

    RS05_LastStatus = RS05_MIT_Control(&hcan1,
                                       RS05_MOTOR_ID,
                                       RS05_TargetPositionRad,
                                       0.0f,
                                       RS05_MIT_TEST_KP,
                                       RS05_MIT_TEST_KD,
                                       0.0f);
    if (RS05_LastStatus == HAL_ERROR)
    {
      (void)RS05_MIT_Stop(&hcan1, RS05_MOTOR_ID);
      Error_Handler();
    }

    HAL_Delay(RS05_MIT_TEST_PERIOD_MS);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**+
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == RS05_Manager.hcan)
  {
    RS05_ProcessRxFifo0(&RS05_Manager,
                        RS05_MASTER_ID,
                        RS05_MIT_Motors,
                        (uint8_t)(sizeof(RS05_MIT_Motors) /
                                  sizeof(RS05_MIT_Motors[0])));
  }
}

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
#ifdef USE_FULL_ASSERT
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
