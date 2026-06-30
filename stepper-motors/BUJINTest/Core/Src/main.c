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
#include <stdio.h>

#include "PD42S1.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_DIR_GPIO_Port GPIOB
#define RS485_DIR_Pin GPIO_PIN_0
#define RS485_DIR_TX GPIO_PIN_SET
#define RS485_DIR_RX GPIO_PIN_RESET

#define PD42_RX_BUF_SIZE 64
#define USER_POSITION_MIN 0L
#define USER_POSITION_MAX 210000L
#define POSITION_READ_INTERVAL_MS 200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char DebugBuffer[128];
int32_t RawPosition;
int32_t UserPosition;
PD42S1_PositionLimitTypeDef RawPositionLimit = {.min_position = -USER_POSITION_MAX,
                                                .max_position = -USER_POSITION_MIN};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Debug_Print(const char *text);
static void Debug_PrintHex(const char *label, const uint8_t *data, uint16_t len);
void PD42_RunExamples(void);
static uint8_t PD42_CheckStep(HAL_StatusTypeDef status, const char *ok_text, const char *fail_text);
static int32_t PD42_UserToRawPosition(int32_t UserPosition);
static int32_t PD42_RawToUserPosition(int32_t RawPosition);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static PD42S1_HandleTypeDef Pd42;
static uint8_t Rs485RxBuf[PD42_RX_BUF_SIZE];

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  PD42S1_Enable(&Pd42);
  PD42S1_Init(&Pd42, &huart1, RS485_DIR_GPIO_Port, RS485_DIR_Pin);

  HAL_Delay(500);
  HAL_StatusTypeDef DisableStatus = PD42S1_Disable(&Pd42);
  HAL_StatusTypeDef EnableReadStatus;
  uint8_t EnableState = 0xFFU;

  HAL_Delay(200);
  EnableReadStatus = PD42S1_ReadEnableState(&Pd42, &EnableState);
  snprintf(DebugBuffer, sizeof(DebugBuffer), "disable=%d, read=%d, enable_state=%u\r\n",
           (int)DisableStatus, (int)EnableReadStatus, EnableState);
  Debug_Print(DebugBuffer);

  snprintf(DebugBuffer, sizeof(DebugBuffer), "user limit=%ld..%ld, raw limit=%ld..%ld\r\n",
           (long)USER_POSITION_MIN,
           (long)USER_POSITION_MAX,
           (long)RawPositionLimit.min_position,
           (long)RawPositionLimit.max_position);
  Debug_Print(DebugBuffer);

  // Move test example:
  int32_t TargetUserPosition = USER_POSITION_MAX / 2;
  PD42S1_Enable(&Pd42);
  PD42S1_SetClosedLoopPositionMode(&Pd42);
 PD42S1_RunLimitedAbsolutePosition(&Pd42,
                                   &RawPositionLimit,
                                   80,
                                   30,
                                   PD42_UserToRawPosition(0));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_StatusTypeDef ReadStatus = PD42S1_ReadRealtimePosition(&Pd42, &RawPosition);
    if (ReadStatus == HAL_OK)
    {
      UserPosition = PD42_RawToUserPosition(RawPosition);
      snprintf(DebugBuffer, sizeof(DebugBuffer), "Raw=%ld, UserPosition=%ld/%ld\r\n",
               (long)RawPosition, (long)UserPosition, (long)USER_POSITION_MAX);
      Debug_Print(DebugBuffer);
    }
    else
    {
      snprintf(DebugBuffer, sizeof(DebugBuffer), "Read position failed: %d\r\n", (int)ReadStatus);
      Debug_Print(DebugBuffer);
    }
    HAL_Delay(POSITION_READ_INTERVAL_MS);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static int32_t PD42_UserToRawPosition(int32_t UserPosition)
{
  if (UserPosition < USER_POSITION_MIN)
  {
    UserPosition = USER_POSITION_MIN;
  }

  if (UserPosition > USER_POSITION_MAX)
  {
    UserPosition = USER_POSITION_MAX;
  }

  return -UserPosition;
}

static int32_t PD42_RawToUserPosition(int32_t RawPosition)
{
  return -RawPosition;
}

static void Debug_Print(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 200);
}
  
static void Debug_PrintHex(const char *label, const uint8_t *data, uint16_t len)
{
  static const char hex[] = "0123456789ABCDEF";
  char out[4];

  Debug_Print(label);
  Debug_Print(": ");

  for (uint16_t i = 0; i < len; i++)
  {
    out[0] = hex[(data[i] >> 4) & 0x0F];
    out[1] = hex[data[i] & 0x0F];
    out[2] = ' ';
    out[3] = '\0';
    Debug_Print(out);
  }

  Debug_Print("\r\n");
}

void PD42_RunExamples(void)
{
  uint16_t RxLen = 0;

  Debug_Print("\r\n--- PD42S1 ATK motion examples ---\r\n");

  if (PD42S1_ReadVersion(&Pd42, Rs485RxBuf, &RxLen, 200) != HAL_OK || RxLen == 0)
  {
    Debug_Print("No ATK reply. Motor examples skipped.\r\n");
    return;
  }

  Debug_PrintHex("version reply", Rs485RxBuf, RxLen);

  Debug_Print("Example 1: speed mode, forward 100 RPM for 2 seconds\r\n");
  if (!PD42_CheckStep(PD42S1_Enable(&Pd42), "enable ok\r\n", "enable failed\r\n"))
  {
    return;
  }
  HAL_Delay(200);
  if (!PD42_CheckStep(PD42S1_SetSpeedMode(&Pd42), "speed mode ok\r\n", "speed mode failed\r\n"))
  {
    PD42S1_Disable(&Pd42);
    return;
  }
  HAL_Delay(200);
  if (!PD42_CheckStep(PD42S1_RunSpeed(&Pd42, PD42S1_DIR_FORWARD, 80, 100.0f),
                      "run speed ok\r\n", "run speed failed\r\n"))
  {
    PD42S1_Disable(&Pd42);
    return;
  }
  HAL_Delay(2000);
  PD42_CheckStep(PD42S1_Stop(&Pd42), "stop ok\r\n", "stop failed\r\n");
  HAL_Delay(300);
  PD42_CheckStep(PD42S1_Disable(&Pd42), "disable ok\r\n", "disable failed\r\n");
  HAL_Delay(1000);

  Debug_Print("Example 2: relative position, reverse half round\r\n");
  if (!PD42_CheckStep(PD42S1_Enable(&Pd42), "enable ok\r\n", "enable failed\r\n"))
  {
    return;
  }
  HAL_Delay(200);
  if (!PD42_CheckStep(PD42S1_SetPositionMode(&Pd42), "position mode ok\r\n", "position mode failed\r\n"))
  {
    PD42S1_Disable(&Pd42);
    return;
  }
  HAL_Delay(200);
  if (!PD42_CheckStep(PD42S1_RunRelativePosition(&Pd42, PD42S1_DIR_REVERSE, 80, 300,
                                                 PD42S1_POS_PER_ROUND / 2U),
                      "relative position ok\r\n", "relative position failed\r\n"))
  {
    PD42S1_Disable(&Pd42);
    return;
  }
  HAL_Delay(3000);
  PD42_CheckStep(PD42S1_Stop(&Pd42), "stop ok\r\n", "stop failed\r\n");
  HAL_Delay(300);
  PD42_CheckStep(PD42S1_Disable(&Pd42), "disable ok\r\n", "disable failed\r\n");
  HAL_Delay(1000);

  Debug_Print("Example 3: clear current position, then absolute quarter round\r\n");
  if (!PD42_CheckStep(PD42S1_Enable(&Pd42), "enable ok\r\n", "enable failed\r\n"))
  {
    return;
  }
  HAL_Delay(200);
  if (!PD42_CheckStep(PD42S1_SetPositionMode(&Pd42), "position mode ok\r\n", "position mode failed\r\n"))
  {
    PD42S1_Disable(&Pd42);
    return;
  }
  HAL_Delay(200);
  if (!PD42_CheckStep(PD42S1_ClearPosition(&Pd42), "clear position ok\r\n", "clear position failed\r\n"))
  {
    PD42S1_Disable(&Pd42);
    return;
  }
  HAL_Delay(200);
  if (!PD42_CheckStep(PD42S1_RunAbsolutePosition(&Pd42, PD42S1_DIR_FORWARD, 80, 300,
                                                 PD42S1_POS_PER_ROUND / 4U),
                      "absolute position ok\r\n", "absolute position failed\r\n"))
  {
    PD42S1_Disable(&Pd42);
    return;
  }
  HAL_Delay(2500);
  PD42_CheckStep(PD42S1_Stop(&Pd42), "stop ok\r\n", "stop failed\r\n");
  HAL_Delay(300);
  PD42_CheckStep(PD42S1_Disable(&Pd42), "disable ok\r\n", "disable failed\r\n");
}

static uint8_t PD42_CheckStep(HAL_StatusTypeDef status, const char *ok_text, const char *fail_text)
{
  if (status == HAL_OK)
  {
    Debug_Print(ok_text);
    return 1U;
  }

  Debug_Print(fail_text);
  return 0U;
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
