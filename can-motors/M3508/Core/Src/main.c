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
#include "M3508.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 自动锁存“大角度回位途中第一次停顿”时的控制状态。 */
typedef struct
{
  volatile uint8_t valid;
  volatile uint32_t tick;
  volatile float position_degree;
  volatile float position_error;
  volatile float target_speed_rpm;
  volatile int16_t actual_speed_rpm;
  volatile int16_t current_command;
  volatile int16_t feedback_current;
  volatile float speed_integral;
} Motor1_StallSnapshotTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 1号M3508位置-速度串级控制参数：首次上电使用较小P项和限流。 */
#define M3508_SPEED_CONTROL_PERIOD_MS     1U
#define M3508_POSITION_CONTROL_PERIOD_MS  5U
#define M3508_OFFLINE_TIMEOUT_MS         100U

/* M3508反馈是电机转子侧数据，减速箱减速比为3591:187。 */
#define M3508_REDUCTION_RATIO         (3591.0f / 187.0f)
#define M3508_ENCODER_COUNTS_PER_REV  8192.0f

/* 位置环输入为输出轴角度，输出为电机转子侧目标转速。 */
#define M3508_POSITION_TARGET_DEG       0.0f
#define M3508_POSITION_KP               15.0f
#define M3508_POSITION_KI                0.0f
#define M3508_POSITION_KD                0.0f
#define M3508_POSITION_SPEED_LIMIT     300.0f
#define M3508_POSITION_DEADBAND_DEG      0.2f

/* 速度环输入为电机转子侧转速，输出为C620电流指令。 */
#define M3508_SPEED_KP                   3.0f
#define M3508_SPEED_KI                   0.02f
#define M3508_SPEED_KD                   0.0f
#define M3508_PID_CURRENT_LIMIT       1500.0f
#define M3508_SPEED_INTEGRAL_LIMIT   30000.0f
#define M3508_PID_INTEGRAL_LIMIT     50000.0f
#define M3508_FRICTION_FF_CURRENT          180
#define M3508_FRICTION_FF_MAX_SPEED_RPM     20
#define M3508_STALL_MOVING_SPEED_RPM       20
#define M3508_STALL_STOP_SPEED_RPM          2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* CAN1总线上的M3508管理器。 */
static M3508_ManagerTypeDef m3508_manager;

/* 8个实际电机对象，数组下标0~7对应电机编号1~8。 */
static M3508_MotorTypeDef motors[M3508_MOTOR_COUNT];

/* 主循环读取到的稳定反馈副本，便于后续PID使用和调试器观察。 */
static M3508_FeedbackTypeDef motor_feedback_snapshot[M3508_MOTOR_COUNT];

/* 运行状态变量，可直接在调试器中观察。 */

static volatile HAL_StatusTypeDef motor_tx_status = HAL_OK;
static volatile uint8_t motor_online[M3508_MOTOR_COUNT];

/* 上一次执行速度环和位置环的时间；HAL的SysTick每1ms更新一次。 */
static uint32_t last_speed_control_tick = 0U;
static uint32_t last_position_control_tick = 0U;

/* 1号电机的位置PID和速度PID；以后控制多台电机时可以改成PID数组。 */
static PID_TypeDef ID_1_M3508;
static PID_TypeDef position_PID_1_M3508;

/* 调试器可直接观察串级控制的中间量和最终电流指令。 */
static volatile float motor1_pid_output = 0.0f;
static volatile int16_t motor1_current_command = 0;
static volatile float motor1_position_degree = 0.0f;
static volatile float motor1_position_error = 0.0f;
static volatile float motor1_target_speed_rpm = 0.0f;
static float motor1_previous_position_error = 0.0f;
static int32_t motor1_position_zero_encoder = 0;
static uint8_t motor1_position_reference_ready = 0U;

/* 仅用于调参，不参与控制输出。 */
static volatile Motor1_StallSnapshotTypeDef motor1_stall_snapshot = {0};
static uint8_t motor1_return_motion_detected = 0U;

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

/* 将转子侧编码器增量换算成减速箱输出轴角度，单位为度。 */
static float M3508_EncoderToOutputDegree(int32_t encoder_delta)
{
  return ((float)encoder_delta * 360.0f) /
         (M3508_ENCODER_COUNTS_PER_REV * M3508_REDUCTION_RATIO);
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

  /* 第一步：创建管理器，并让它使用C板的CAN1。 */
  if (M3508_Manager_Init(&m3508_manager, &hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 第二步：把1~8号电机对象逐个注册进管理器。 */
  for (uint8_t motor_id = 1U; motor_id <= M3508_MOTOR_COUNT; ++motor_id)
  {
    if (M3508_Manager_RegisterMotor(&m3508_manager,
                                    &motors[motor_id - 1U],
                                    motor_id) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /*
   * 第三步：初始化1号电机的位置PID和速度PID。
   * 位置PID输出目标转速，速度PID输出C620电流指令。
   */
  PID_Init(&ID_1_M3508,
           M3508_SPEED_KP,
           M3508_SPEED_KI,
           M3508_SPEED_KD,
           -M3508_PID_CURRENT_LIMIT,
           M3508_PID_CURRENT_LIMIT,
           -M3508_SPEED_INTEGRAL_LIMIT,
           M3508_SPEED_INTEGRAL_LIMIT);
  PID_Init(&position_PID_1_M3508,
           M3508_POSITION_KP,
           M3508_POSITION_KI,
           M3508_POSITION_KD,
           -M3508_POSITION_SPEED_LIMIT,
           M3508_POSITION_SPEED_LIMIT,
           -M3508_PID_INTEGRAL_LIMIT,
           M3508_PID_INTEGRAL_LIMIT);
  PID_SetTarget(&position_PID_1_M3508, M3508_POSITION_TARGET_DEG);
  PID_SetTarget(&ID_1_M3508, 0.0f);

  /* 收到有效反馈前保持0电流，避免电机在CAN尚未正常时突然动作。 */
  if (M3508_Motor_SetCurrent(&motors[0], 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* 第四步：配置0x201~0x208过滤器，启动CAN接收中断。 */
  if (M3508_Manager_Start(&m3508_manager) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    const uint32_t now_tick = HAL_GetTick();

    /*
     * 每1ms执行一次速度环，位置环每5ms执行一次。
     * 两个PID都只使用最新反馈，CAN控制帧仍保持1ms周期发送。
     */
    if ((now_tick - last_speed_control_tick) >=
        M3508_SPEED_CONTROL_PERIOD_MS)
    {
      uint8_t motor1_feedback_updated = 0U;

      last_speed_control_tick = now_tick;

      /* 读取各电机最新反馈，并更新在线状态。 */
      for (uint8_t index = 0U; index < M3508_MOTOR_COUNT; ++index)
      {
        const uint8_t feedback_updated =
            M3508_Motor_GetFeedback(&motors[index],
                                    &motor_feedback_snapshot[index]);

        if (index == 0U)
        {
          motor1_feedback_updated = feedback_updated;
        }

        motor_online[index] = M3508_Motor_IsOnline(
            &motors[index], now_tick, M3508_OFFLINE_TIMEOUT_MS);
      }

      if (motor_online[0] == 0U)
      {
        /*
         * 失联时立即清零电流和两个PID，但保留上电时建立的位置零点。
         * 否则短暂掉线会把当前位置误当成新零点，造成无法完全回正。
         */
        PID_Reset(&ID_1_M3508);
        PID_Reset(&position_PID_1_M3508);
        motor1_pid_output = 0.0f;
        motor1_current_command = 0;
        motor1_target_speed_rpm = 0.0f;
        motor1_previous_position_error = 0.0f;
        motor1_return_motion_detected = 0U;
        (void)M3508_Motor_SetCurrent(&motors[0], 0);
      }
      else if (motor1_feedback_updated != 0U)
      {
        /* 首次在线时把当前位置记作0度，避免上电后主动寻找绝对零点。 */
        if (motor1_position_reference_ready == 0U)
        {
          motor1_position_zero_encoder =
              motor_feedback_snapshot[0].total_encoder;
          motor1_position_degree = 0.0f;
          motor1_position_error = 0.0f;
          motor1_previous_position_error = 0.0f;
          motor1_target_speed_rpm = 0.0f;
          motor1_position_reference_ready = 1U;
          last_position_control_tick = now_tick;
          PID_Reset(&position_PID_1_M3508);
          PID_Reset(&ID_1_M3508);
          PID_SetTarget(&ID_1_M3508, 0.0f);
        }

        /* 外层位置PID：输出轴角度误差转换为电机转子侧目标转速。 */
        if ((now_tick - last_position_control_tick) >=
            M3508_POSITION_CONTROL_PERIOD_MS)
        {
          motor1_position_degree = M3508_EncoderToOutputDegree(
              motor_feedback_snapshot[0].total_encoder -
              motor1_position_zero_encoder);
          motor1_position_error = position_PID_1_M3508.target -
                                  motor1_position_degree;

          /* 越过目标位置时清除旧方向的速度积分，避免积分继续推过头。 */
          if (((motor1_position_error > 0.0f) &&
               (motor1_previous_position_error < 0.0f)) ||
              ((motor1_position_error < 0.0f) &&
               (motor1_previous_position_error > 0.0f)))
          {
            ID_1_M3508.integral = 0.0f;
          }
          motor1_previous_position_error = motor1_position_error;

          /* 进入目标附近后停止位置修正，由速度环负责把转速刹到0。 */
          if ((motor1_position_error <= M3508_POSITION_DEADBAND_DEG) &&
              (motor1_position_error >= -M3508_POSITION_DEADBAND_DEG))
          {
            motor1_target_speed_rpm = 0.0f;
            motor1_return_motion_detected = 0U;
            PID_Reset(&position_PID_1_M3508);
            ID_1_M3508.integral = 0.0f;
          }
          else
          {
            motor1_target_speed_rpm = PID_Calculate(
                &position_PID_1_M3508,
                motor1_position_degree);
          }
          PID_SetTarget(&ID_1_M3508, motor1_target_speed_rpm);
          last_position_control_tick = now_tick;
        }

        /* 内层速度PID：目标转速误差转换为C620电流指令。 */
        motor1_pid_output = PID_Calculate(
            &ID_1_M3508,
            (float)motor_feedback_snapshot[0].speed_rpm);
        motor1_current_command = (int16_t)motor1_pid_output;

        /*
         * 低速回位时补偿减速箱静摩擦。
         * 仅当PID输出与目标速度同向时加入，若速度环正在反向刹车则不补偿。
         */
        if ((motor_feedback_snapshot[0].speed_rpm <=
             M3508_FRICTION_FF_MAX_SPEED_RPM) &&
            (motor_feedback_snapshot[0].speed_rpm >=
             -M3508_FRICTION_FF_MAX_SPEED_RPM))
        {
          if ((motor1_target_speed_rpm > 0.0f) &&
              (motor1_current_command > 0))
          {
            motor1_current_command += M3508_FRICTION_FF_CURRENT;
          }
          else if ((motor1_target_speed_rpm < 0.0f) &&
                   (motor1_current_command < 0))
          {
            motor1_current_command -= M3508_FRICTION_FF_CURRENT;
          }
        }

        /* 前馈加入后再次限幅，确保不超过C620调试电流限制。 */
        if (motor1_current_command > (int16_t)M3508_PID_CURRENT_LIMIT)
        {
          motor1_current_command = (int16_t)M3508_PID_CURRENT_LIMIT;
        }
        else if (motor1_current_command <
                 (int16_t)(-M3508_PID_CURRENT_LIMIT))
        {
          motor1_current_command = (int16_t)(-M3508_PID_CURRENT_LIMIT);
        }
        motor1_pid_output = (float)motor1_current_command;

        /* 只识别朝零点方向的回位运动，避免把手动移开时的停顿误认为卡顿。 */
        if (((motor1_target_speed_rpm > 0.0f) &&
             (motor_feedback_snapshot[0].speed_rpm >=
              M3508_STALL_MOVING_SPEED_RPM)) ||
            ((motor1_target_speed_rpm < 0.0f) &&
             (motor_feedback_snapshot[0].speed_rpm <=
              -M3508_STALL_MOVING_SPEED_RPM)))
        {
          motor1_return_motion_detected = 1U;
        }

        /* 回位已经运动过，但在死区外再次接近静止时，锁存第一现场。 */
        if ((motor1_stall_snapshot.valid == 0U) &&
            (motor1_return_motion_detected != 0U) &&
            (motor_feedback_snapshot[0].speed_rpm <=
             M3508_STALL_STOP_SPEED_RPM) &&
            (motor_feedback_snapshot[0].speed_rpm >=
             -M3508_STALL_STOP_SPEED_RPM) &&
            ((motor1_position_error > M3508_POSITION_DEADBAND_DEG) ||
             (motor1_position_error < -M3508_POSITION_DEADBAND_DEG)))
        {
          motor1_stall_snapshot.tick = now_tick;
          motor1_stall_snapshot.position_degree = motor1_position_degree;
          motor1_stall_snapshot.position_error = motor1_position_error;
          motor1_stall_snapshot.target_speed_rpm = motor1_target_speed_rpm;
          motor1_stall_snapshot.actual_speed_rpm =
              motor_feedback_snapshot[0].speed_rpm;
          motor1_stall_snapshot.current_command = motor1_current_command;
          motor1_stall_snapshot.feedback_current =
              motor_feedback_snapshot[0].current;
          motor1_stall_snapshot.speed_integral = ID_1_M3508.integral;
          motor1_stall_snapshot.valid = 1U;
        }

        (void)M3508_Motor_SetCurrent(&motors[0],
                                     motor1_current_command);
      }

      /* 即使本周期没有新反馈，也继续发送上一次电流指令。 */
      motor_tx_status = M3508_Manager_SendCurrents(&m3508_manager);
    }
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* HAL收到FIFO0消息后，把处理工作转交给M3508管理器。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  M3508_Manager_RxFifo0Callback(&m3508_manager, hcan);
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{
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
