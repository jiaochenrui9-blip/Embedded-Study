#include "PD42S1_Private.h"

static HAL_StatusTypeDef PD42S1_ValidateReply(const PD42S1_MotorTypeDef *motor,
                                              uint8_t code,
                                              const uint8_t *reply,
                                              uint16_t reply_len);
static void PD42S1_ClearRx(PD42S1_ManagerTypeDef *manager);

void PD42S1_ManagerInit(PD42S1_ManagerTypeDef *manager,
                         UART_HandleTypeDef *huart,
                         GPIO_TypeDef *dir_port,
                         uint16_t dir_pin)
{
  if (manager == NULL)
  {
    return;
  }

  manager->huart = huart;
  manager->dir_port = dir_port;
  manager->dir_pin = dir_pin;
  manager->tx_state = GPIO_PIN_SET;
  manager->rx_state = GPIO_PIN_RESET;
  manager->motor_count = 0U;

  for (uint8_t i = 0U; i < PD42S1_MAX_MOTORS; i++)
  {
    manager->motors[i] = (PD42S1_MotorTypeDef){0};
  }

  if (manager->dir_port != NULL)
  {
    HAL_GPIO_WritePin(manager->dir_port, manager->dir_pin, manager->rx_state);
  }
}

HAL_StatusTypeDef PD42S1_RegisterMotor(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor;

  if (manager == NULL || manager->motor_count >= PD42S1_MAX_MOTORS ||
      PD42S1_FindMotor(manager, device_id) != NULL)
  {
    return HAL_ERROR;
  }

  motor = &manager->motors[manager->motor_count];
  *motor = (PD42S1_MotorTypeDef){0};
  motor->device_id = device_id;
  manager->motor_count++;

  return HAL_OK;
}

PD42S1_MotorTypeDef *PD42S1_FindMotor(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  if (manager == NULL)
  {
    return NULL;
  }

  for (uint8_t i = 0U; i < manager->motor_count; i++)
  {
    if (manager->motors[i].device_id == device_id)
    {
      return &manager->motors[i];
    }
  }

  return NULL;
}

HAL_StatusTypeDef PD42S1_SendCommand(PD42S1_ManagerTypeDef *manager,
                                      uint8_t device_id,
                                      uint8_t code,
                                      const uint8_t *data,
                                      uint8_t data_len,
                                      uint8_t *rx,
                                      uint16_t *rx_len,
                                      uint16_t rx_max,
                                      uint32_t timeout_ms)
{
  PD42S1_MotorTypeDef *motor;
  uint8_t frame[PD42S1_FRAME_MAX_LEN];
  uint8_t ack[PD42S1_FRAME_MAX_LEN];
  uint8_t *target_rx;
  uint16_t target_max;
  uint16_t local_rx_len = 0;
  uint16_t *target_rx_len;
  uint8_t sum = 0U;
  uint16_t frame_len;
  uint32_t start_tick;
  uint32_t receive_timeout;
  HAL_StatusTypeDef status;

  motor = PD42S1_FindMotor(manager, device_id);
  if (manager == NULL || manager->huart == NULL || manager->dir_port == NULL || motor == NULL ||
      data_len > (PD42S1_FRAME_MAX_LEN - 5U) || (data == NULL && data_len > 0U))
  {
    return HAL_ERROR;
  }

  frame[0] = PD42S1_FRAME_HEAD;
  frame[1] = motor->device_id;
  frame[2] = code;
  for (uint8_t i = 0U; i < data_len; i++)
  {
    frame[3U + i] = data[i];
  }

  for (uint8_t i = 0U; i < (uint8_t)(3U + data_len); i++)
  {
    sum = (uint8_t)(sum + frame[i]);
  }
  frame[3U + data_len] = sum;
  frame[4U + data_len] = PD42S1_FRAME_TAIL;
  frame_len = (uint16_t)(5U + data_len);

  PD42S1_ClearRx(manager);

  HAL_GPIO_WritePin(manager->dir_port, manager->dir_pin, manager->tx_state);
  HAL_Delay(1U);
  status = HAL_UART_Transmit(manager->huart, frame, frame_len, 200U);
  if (status != HAL_OK)
  {
    HAL_GPIO_WritePin(manager->dir_port, manager->dir_pin, manager->rx_state);
    return status;
  }

  while (__HAL_UART_GET_FLAG(manager->huart, UART_FLAG_TC) == RESET)
  {
  }
  HAL_GPIO_WritePin(manager->dir_port, manager->dir_pin, manager->rx_state);

  if (rx != NULL && rx_len != NULL && rx_max > 0U)
  {
    target_rx = rx;
    target_rx_len = rx_len;
    target_max = rx_max;
  }
  else
  {
    target_rx = ack;
    target_rx_len = &local_rx_len;
    target_max = sizeof(ack);
  }

  receive_timeout = timeout_ms > 0U ? timeout_ms : PD42S1_DEFAULT_TIMEOUT_MS;
  *target_rx_len = 0U;
  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < receive_timeout && *target_rx_len < target_max)
  {
    if (HAL_UART_Receive(manager->huart, &target_rx[*target_rx_len], 1U, 5U) == HAL_OK)
    {
      (*target_rx_len)++;
      start_tick = HAL_GetTick();

      if (*target_rx_len >= 6U && target_rx[*target_rx_len - 1U] == PD42S1_FRAME_TAIL &&
          PD42S1_ValidateReply(motor, code, target_rx, *target_rx_len) == HAL_OK)
      {
        break;
      }
    }
  }

  if (*target_rx_len == 0U)
  {
    return HAL_TIMEOUT;
  }

  return PD42S1_ValidateReply(motor, code, target_rx, *target_rx_len);
}

HAL_StatusTypeDef PD42S1_ReadData(PD42S1_ManagerTypeDef *manager,
                                   uint8_t device_id,
                                   uint8_t code,
                                   uint8_t *rx,
                                   uint16_t rx_size,
                                   uint8_t data_len)
{
  uint16_t rx_len = 0U;
  uint16_t expected_len = (uint16_t)(data_len + 6U);
  HAL_StatusTypeDef status;

  if (rx == NULL || rx_size < expected_len)
  {
    return HAL_ERROR;
  }

  status = PD42S1_SendCommand(manager,
                              device_id,
                              code,
                              NULL,
                              0U,
                              rx,
                              &rx_len,
                              rx_size,
                              PD42S1_DEFAULT_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  return rx_len >= expected_len ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef PD42S1_ValidateReply(const PD42S1_MotorTypeDef *motor,
                                              uint8_t code,
                                              const uint8_t *reply,
                                              uint16_t reply_len)
{
  uint8_t sum = 0U;

  if (motor == NULL || reply == NULL || reply_len < 6U || reply[0] != PD42S1_FRAME_HEAD ||
      reply[1] != motor->device_id || reply[2] != code ||
      reply[reply_len - 1U] != PD42S1_FRAME_TAIL)
  {
    return HAL_ERROR;
  }

  for (uint16_t i = 0U; i < (uint16_t)(reply_len - 2U); i++)
  {
    sum = (uint8_t)(sum + reply[i]);
  }

  if (sum != reply[reply_len - 2U] || reply[3] != PD42S1_ACK_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void PD42S1_WriteU16BE(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value >> 8);
  dst[1] = (uint8_t)value;
}

void PD42S1_WriteU32BE(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value >> 24);
  dst[1] = (uint8_t)(value >> 16);
  dst[2] = (uint8_t)(value >> 8);
  dst[3] = (uint8_t)value;
}

void PD42S1_WriteI32BE(uint8_t *dst, int32_t value)
{
  PD42S1_WriteU32BE(dst, (uint32_t)value);
}

void PD42S1_WriteFloatBE(uint8_t *dst, float value)
{
  union
  {
    float f;
    uint32_t u32;
  } converter;

  converter.f = value;
  PD42S1_WriteU32BE(dst, converter.u32);
}

uint16_t PD42S1_ReadU16BE(const uint8_t *src)
{
  return (uint16_t)(((uint16_t)src[0] << 8) | src[1]);
}

int16_t PD42S1_ReadI16BE(const uint8_t *src)
{
  return (int16_t)PD42S1_ReadU16BE(src);
}

uint32_t PD42S1_ReadU32BE(const uint8_t *src)
{
  return ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) |
         ((uint32_t)src[2] << 8) | src[3];
}

int32_t PD42S1_ReadI32BE(const uint8_t *src)
{
  return (int32_t)PD42S1_ReadU32BE(src);
}

float PD42S1_ReadFloatBE(const uint8_t *src)
{
  union
  {
    float f;
    uint32_t u32;
  } converter;

  converter.u32 = PD42S1_ReadU32BE(src);
  return converter.f;
}

static void PD42S1_ClearRx(PD42S1_ManagerTypeDef *manager)
{
  if (manager == NULL || manager->huart == NULL || manager->dir_port == NULL)
  {
    return;
  }

  HAL_GPIO_WritePin(manager->dir_port, manager->dir_pin, manager->rx_state);

  while (__HAL_UART_GET_FLAG(manager->huart, UART_FLAG_RXNE) == SET)
  {
    (void)manager->huart->Instance->DR;
  }

  __HAL_UART_CLEAR_OREFLAG(manager->huart);
}
