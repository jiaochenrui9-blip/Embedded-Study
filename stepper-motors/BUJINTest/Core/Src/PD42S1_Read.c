#include "PD42S1_Private.h"

HAL_StatusTypeDef PD42S1_ReadVersion(PD42S1_ManagerTypeDef *manager,
                                      uint8_t device_id,
                                      uint8_t *rx,
                                      uint16_t *rx_len,
                                      uint32_t timeout_ms)
{
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_READ_VERSION, NULL, 0U, rx, rx_len, 32U, timeout_ms);
}

HAL_StatusTypeDef PD42S1_ReadPhaseCurrent(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[8];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_PHASE_CURRENT, rx, sizeof(rx), 2U);
  if (status == HAL_OK)
  {
    motor->current_ma = PD42S1_ReadI16BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}
HAL_StatusTypeDef PD42S1_ReadBusVoltage(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[10];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_BUS_VOLTAGE, rx, sizeof(rx), 4U);
  if (status == HAL_OK)
  {
    motor->bus_voltage = PD42S1_ReadFloatBE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadInputPulse(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[10];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_INPUT_PULSE, rx, sizeof(rx), 4U);
  if (status == HAL_OK)
  {
    motor->input_pulse = PD42S1_ReadU32BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadRealtimeSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[8];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_REALTIME_SPEED, rx, sizeof(rx), 2U);
  if (status == HAL_OK)
  {
    motor->realtime_speed_rpm = PD42S1_ReadI16BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadRealtimePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[10];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_REALTIME_POSITION, rx, sizeof(rx), 4U);
  if (status == HAL_OK)
  {
    motor->position = PD42S1_ReadI32BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadPositionError(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[10];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_POSITION_ERROR, rx, sizeof(rx), 4U);
  if (status == HAL_OK)
  {
    motor->position_error = PD42S1_ReadI32BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadRunStatus(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[7];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_RUN_STATUS, rx, sizeof(rx), 1U);
  if (status == HAL_OK)
  {
    motor->run_status = rx[PD42S1_REPLY_DATA_OFFSET];
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadStallFlag(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[7];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_STALL_FLAG, rx, sizeof(rx), 1U);
  if (status == HAL_OK)
  {
    motor->stall_flag = rx[PD42S1_REPLY_DATA_OFFSET];
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadEnableState(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[7];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_ENABLE_STATE, rx, sizeof(rx), 1U);
  if (status == HAL_OK)
  {
    motor->enable_state = rx[PD42S1_REPLY_DATA_OFFSET];
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadArrivedFlag(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  PD42S1_MotorTypeDef *motor = PD42S1_FindMotor(manager, device_id);
  uint8_t rx[7];
  HAL_StatusTypeDef status;

  if (motor == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(manager, device_id, PD42S1_CMD_READ_ARRIVED_FLAG, rx, sizeof(rx), 1U);
  if (status == HAL_OK)
  {
    motor->arrived_flag = rx[PD42S1_REPLY_DATA_OFFSET];
  }

  return status;
}
