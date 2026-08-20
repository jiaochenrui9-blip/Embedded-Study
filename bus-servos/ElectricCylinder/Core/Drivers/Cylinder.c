//
// Created by game on 2026/8/19.
//

#include "Cylinder.h"

#define CYLINDER_REQUEST_HEAD_0       0x55U
#define CYLINDER_REQUEST_HEAD_1       0xAAU
#define CYLINDER_REPLY_HEAD_0         0xAAU
#define CYLINDER_REPLY_HEAD_1         0x55U
#define CYLINDER_STATUS_REPLY_LENGTH  0x11U

static HAL_StatusTypeDef Cylinder_ReceiveReply(Cylinder_ManagerTypeDef *manager,
                                               uint8_t *reply,
                                               uint8_t reply_max,
                                               uint8_t *reply_length,
                                               uint32_t timeout_ms);
static HAL_StatusTypeDef Cylinder_ValidateReply(const Cylinder_MotorTypeDef *motor,
                                                const uint8_t *reply,
                                                uint8_t reply_length);
static void Cylinder_ClearRx(Cylinder_ManagerTypeDef *manager);
static uint16_t Cylinder_ReadU16LE(const uint8_t *src);

void Cylinder_ManagerInit(Cylinder_ManagerTypeDef *manager, UART_HandleTypeDef *huart)
{
  if (manager == NULL)
  {
    return;
  }

  manager->huart = huart;
  manager->motor_count = 0U;

  for (uint8_t i = 0U; i < CYLINDER_MAX_MOTORS; i++)
  {
    manager->motors[i] = (Cylinder_MotorTypeDef){0};
  }
}

HAL_StatusTypeDef Cylinder_RegisterMotor(Cylinder_ManagerTypeDef *manager, uint8_t device_id)
{
  Cylinder_MotorTypeDef *motor;

  if (manager == NULL || device_id == 0U || device_id == CYLINDER_BROADCAST_ID ||
      manager->motor_count >= CYLINDER_MAX_MOTORS ||
      Cylinder_FindMotor(manager, device_id) != NULL)
  {
    return HAL_ERROR;
  }

  motor = &manager->motors[manager->motor_count];
  *motor = (Cylinder_MotorTypeDef){0};
  motor->device_id = device_id;
  manager->motor_count++;
  return HAL_OK;
}

Cylinder_MotorTypeDef *Cylinder_FindMotor(Cylinder_ManagerTypeDef *manager, uint8_t device_id)
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

HAL_StatusTypeDef Cylinder_SendCommand(Cylinder_ManagerTypeDef *manager,
                                              uint8_t device_id,
                                              uint8_t command,
                                              uint8_t table_index,
                                              const uint8_t *data,
                                              uint8_t data_length,
                                              uint8_t *reply,
                                              uint8_t reply_max,
                                              uint8_t *reply_length)
{
  Cylinder_MotorTypeDef *motor;
  uint8_t frame[CYLINDER_FRAME_MAX_LEN];
  uint8_t checksum = 0U;
  uint8_t frame_length;
  HAL_StatusTypeDef status;

  motor = Cylinder_FindMotor(manager, device_id);
  if (manager == NULL || manager->huart == NULL || motor == NULL ||
      data_length > (CYLINDER_FRAME_MAX_LEN - 7U) ||
      (data == NULL && data_length > 0U) || reply == NULL ||
      reply_length == NULL || reply_max < 7U)
  {
    return HAL_ERROR;
  }

  Cylinder_ClearRx(manager);

  frame[0] = CYLINDER_REQUEST_HEAD_0;
  frame[1] = CYLINDER_REQUEST_HEAD_1;
  frame[2] = (uint8_t)(data_length + 2U);
  frame[3] = device_id;
  frame[4] = command;
  frame[5] = table_index;
  for (uint8_t i = 0U; i < data_length; i++)
  {
    frame[6U + i] = data[i];
  }

  for (uint8_t i = 2U; i < (uint8_t)(6U + data_length); i++)
  {
    checksum = (uint8_t)(checksum + frame[i]);
  }
  frame[6U + data_length] = checksum;
  frame_length = (uint8_t)(7U + data_length);

  status = HAL_UART_Transmit(manager->huart, frame, frame_length, CYLINDER_DEFAULT_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Cylinder_ReceiveReply(manager, reply, reply_max,
                                 reply_length, CYLINDER_DEFAULT_TIMEOUT_MS);
  return status == HAL_OK ? Cylinder_ValidateReply(motor, reply, *reply_length) : status;
}

static HAL_StatusTypeDef Cylinder_ReceiveReply(Cylinder_ManagerTypeDef *manager,
                                               uint8_t *reply,
                                               uint8_t reply_max,
                                               uint8_t *reply_length,
                                               uint32_t timeout_ms)
{
  uint8_t byte;
  uint32_t start_tick;

  if (manager == NULL || manager->huart == NULL || reply == NULL ||
      reply_length == NULL || reply_max < 6U)
  {
    return HAL_ERROR;
  }

  *reply_length = 0U;
  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    if (HAL_UART_Receive(manager->huart, &byte, 1U, 1U) != HAL_OK)
    {
      continue;
    }

    if (*reply_length == 0U)
    {
      if (byte != CYLINDER_REPLY_HEAD_0)
      {
        continue;
      }
    }
    else if (*reply_length == 1U && byte != CYLINDER_REPLY_HEAD_1)
    {
      *reply_length = byte == CYLINDER_REPLY_HEAD_0 ? 1U : 0U;
      continue;
    }

    reply[*reply_length] = byte;
    (*reply_length)++;

    if (*reply_length == 3U && ((uint8_t)(reply[2] + 5U) > reply_max ||
                                (uint8_t)(reply[2] + 5U) < 7U))
    {
      *reply_length = 0U;
    }
    else if (*reply_length >= 3U && *reply_length == (uint8_t)(reply[2] + 5U))
    {
      return HAL_OK;
    }
  }

  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef Cylinder_ValidateReply(const Cylinder_MotorTypeDef *motor,
                                                const uint8_t *reply,
                                                uint8_t reply_length)
{
  uint8_t checksum = 0U;

  if (motor == NULL || reply == NULL || reply_length < 6U ||
      reply[0] != CYLINDER_REPLY_HEAD_0 || reply[1] != CYLINDER_REPLY_HEAD_1 ||
      reply_length != (uint8_t)(reply[2] + 5U) || reply[3] != motor->device_id)
  {
    return HAL_ERROR;
  }

  for (uint8_t i = 2U; i < (uint8_t)(reply_length - 1U); i++)
  {
    checksum = (uint8_t)(checksum + reply[i]);
  }

  return checksum == reply[reply_length - 1U] ? HAL_OK : HAL_ERROR;
}

static void Cylinder_ClearRx(Cylinder_ManagerTypeDef *manager)
{
  if (manager == NULL || manager->huart == NULL)
  {
    return;
  }

  while (__HAL_UART_GET_FLAG(manager->huart, UART_FLAG_RXNE) == SET)
  {
    (void)manager->huart->Instance->DR;
  }
  __HAL_UART_CLEAR_OREFLAG(manager->huart);
}

HAL_StatusTypeDef Cylinder_ParseFeedback(Cylinder_ManagerTypeDef *manager,
                                         uint8_t device_id,
                                         const uint8_t *reply,
                                         uint8_t reply_length)
{
  Cylinder_MotorTypeDef *motor = Cylinder_FindMotor(manager, device_id);

  if (motor == NULL || reply == NULL || reply_length < 7U)
  {
    return HAL_ERROR;
  }

  motor->online = 1U;
  motor->last_reply_tick = HAL_GetTick();

  if (reply_length != (CYLINDER_STATUS_REPLY_LENGTH + 5U) ||
      reply[4] != CYLINDER_CMD_SINGLE_CONTROL)
  {
    return HAL_OK;
  }

  motor->target_position_0p01mm = Cylinder_ReadU16LE(&reply[7]);
  motor->current_position_0p01mm = (int16_t)Cylinder_ReadU16LE(&reply[9]);
  motor->bus_voltage_0p1v = reply[11];
  motor->q_axis_current_ma = (int16_t)Cylinder_ReadU16LE(&reply[12]);
  motor->offline_check_error = reply[14];
  motor->fault_code = reply[15];
  motor->max_speed_mm_s = reply[16];
  motor->overcurrent_limit_ma = Cylinder_ReadU16LE(&reply[17]);
  motor->software_version = Cylinder_ReadU16LE(&reply[19]);
  return HAL_OK;
}

static uint16_t Cylinder_ReadU16LE(const uint8_t *src)
{
  return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}
