#include "PD42S1_Private.h"

static HAL_StatusTypeDef PD42S1_ValidateReply(const PD42S1_HandleTypeDef *motor,
                                              uint8_t code,
                                              const uint8_t *reply,
                                              uint16_t reply_len);
static void PD42S1_ClearRx(PD42S1_HandleTypeDef *motor);

void PD42S1_SetAddress(PD42S1_HandleTypeDef *motor, uint8_t addr)
{
  if (motor != NULL)
  {
    motor->addr = addr;
  }
}

void PD42S1_Init(PD42S1_HandleTypeDef *motor,
                 UART_HandleTypeDef *huart,
                 GPIO_TypeDef *dir_port,
                 uint16_t dir_pin)
{
  if (motor == NULL)
  {
    return;
  }

  motor->huart = huart;
  motor->dir_port = dir_port;
  motor->dir_pin = dir_pin;
  motor->tx_state = GPIO_PIN_SET;
  motor->rx_state = GPIO_PIN_RESET;
  motor->addr = 0x01U;

  if (motor->dir_port != NULL)
  {
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->rx_state);
  }
}

HAL_StatusTypeDef PD42S1_SendCommand(PD42S1_HandleTypeDef *motor,
                                      uint8_t code,
                                      const uint8_t *data,
                                      uint8_t data_len,
                                      uint8_t *rx,
                                      uint16_t *rx_len,
                                      uint16_t rx_max,
                                      uint32_t timeout_ms)
{
  uint8_t frame[PD42S1_FRAME_MAX_LEN];
  uint8_t ack[PD42S1_FRAME_MAX_LEN];
  uint8_t *target_rx;
  uint16_t target_max;
  uint16_t local_rx_len = 0;
  uint16_t *target_rx_len;
  uint8_t sum = 0;
  uint16_t frame_len;
  uint32_t start_tick;
  uint32_t receive_timeout;
  HAL_StatusTypeDef status;

  if (motor == NULL || motor->huart == NULL || motor->dir_port == NULL ||
      data_len > (PD42S1_FRAME_MAX_LEN - 5U) ||
      (data == NULL && data_len > 0U))
  {
    return HAL_ERROR;
  }

  frame[0] = PD42S1_FRAME_HEAD;
  frame[1] = motor->addr;
  frame[2] = code;
  for (uint8_t i = 0; i < data_len; i++)
  {
    frame[3U + i] = data[i];
  }

  for (uint8_t i = 0; i < (uint8_t)(3U + data_len); i++)
  {
    sum = (uint8_t)(sum + frame[i]);
  }
  frame[3U + data_len] = sum;
  frame[4U + data_len] = PD42S1_FRAME_TAIL;
  frame_len = (uint16_t)(5U + data_len);

  PD42S1_ClearRx(motor);

  HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->tx_state);
  HAL_Delay(1);
  status = HAL_UART_Transmit(motor->huart, frame, frame_len, 200);
  if (status != HAL_OK)
  {
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->rx_state);
    return status;
  }

  while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_TC) == RESET)
  {
  }
  HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->rx_state);

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
  *target_rx_len = 0;
  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < receive_timeout && *target_rx_len < target_max)
  {
    if (HAL_UART_Receive(motor->huart, &target_rx[*target_rx_len], 1, 5) == HAL_OK)
    {
      (*target_rx_len)++;
      start_tick = HAL_GetTick();

      if (*target_rx_len >= 6U &&
          target_rx[*target_rx_len - 1U] == PD42S1_FRAME_TAIL &&
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

HAL_StatusTypeDef PD42S1_ReadData(PD42S1_HandleTypeDef *motor,
                                   uint8_t code,
                                   uint8_t *rx,
                                   uint16_t rx_size,
                                   uint8_t data_len)
{
  uint16_t rx_len = 0;
  uint16_t expected_len = (uint16_t)(data_len + 6U);
  HAL_StatusTypeDef status;

  if (rx == NULL || rx_size < expected_len)
  {
    return HAL_ERROR;
  }

  status = PD42S1_SendCommand(motor, code, NULL, 0, rx, &rx_len, rx_size, PD42S1_DEFAULT_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  return rx_len >= expected_len ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef PD42S1_ValidateReply(const PD42S1_HandleTypeDef *motor,
                                              uint8_t code,
                                              const uint8_t *reply,
                                              uint16_t reply_len)
{
  uint8_t sum = 0;

  if (motor == NULL ||
      reply == NULL ||
      reply_len < 6U ||
      reply[0] != PD42S1_FRAME_HEAD ||
      reply[1] != motor->addr ||
      reply[2] != code ||
      reply[reply_len - 1U] != PD42S1_FRAME_TAIL)
  {
    return HAL_ERROR;
  }

  for (uint16_t i = 0; i < (uint16_t)(reply_len - 2U); i++)
  {
    sum = (uint8_t)(sum + reply[i]);
  }

  if (sum != reply[reply_len - 2U] || reply[3] != PD42S1_ACK_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef PD42S1_ValidateDirectionAndAccel(uint8_t direction, uint8_t acceleration)
{
  if (direction > PD42S1_DIR_REVERSE || acceleration > 200U)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef PD42S1_ValidatePulseWidth(uint16_t max_width_us, uint16_t min_width_us)
{
  if (max_width_us < PD42S1_PULSE_WIDTH_MIN_US ||
      max_width_us > PD42S1_PULSE_WIDTH_MAX_US ||
      min_width_us < PD42S1_PULSE_WIDTH_MIN_US ||
      min_width_us > PD42S1_PULSE_WIDTH_MAX_US ||
      max_width_us <= min_width_us)
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
  return ((uint32_t)src[0] << 24) |
         ((uint32_t)src[1] << 16) |
         ((uint32_t)src[2] << 8) |
         src[3];
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

static void PD42S1_ClearRx(PD42S1_HandleTypeDef *motor)
{
  if (motor == NULL || motor->huart == NULL || motor->dir_port == NULL)
  {
    return;
  }

  HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->rx_state);

  while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_RXNE) == SET)
  {
    (void)motor->huart->Instance->DR;
  }

  __HAL_UART_CLEAR_OREFLAG(motor->huart);
}
