#include "PD42S1_Private.h"

HAL_StatusTypeDef PD42S1_ReadVersion(PD42S1_HandleTypeDef *motor,
                                      uint8_t *rx,
                                      uint16_t *rx_len,
                                      uint32_t timeout_ms)
{
  return PD42S1_SendCommand(motor, PD42S1_CMD_READ_VERSION, NULL, 0, rx, rx_len, 32, timeout_ms);
}

HAL_StatusTypeDef PD42S1_ReadPhaseCurrent(PD42S1_HandleTypeDef *motor, int16_t *current_ma)
{
  uint8_t rx[8];
  HAL_StatusTypeDef status;

  if (current_ma == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_PHASE_CURRENT, rx, sizeof(rx), 2);
  if (status == HAL_OK)
  {
    *current_ma = PD42S1_ReadI16BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadBusVoltage(PD42S1_HandleTypeDef *motor, float *voltage)
{
  uint8_t rx[10];
  HAL_StatusTypeDef status;

  if (voltage == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_BUS_VOLTAGE, rx, sizeof(rx), 4);
  if (status == HAL_OK)
  {
    *voltage = PD42S1_ReadFloatBE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadInputPulse(PD42S1_HandleTypeDef *motor, uint32_t *pulse_count)
{
  uint8_t rx[10];
  HAL_StatusTypeDef status;

  if (pulse_count == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_INPUT_PULSE, rx, sizeof(rx), 4);
  if (status == HAL_OK)
  {
    *pulse_count = PD42S1_ReadU32BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadRealtimeSpeed(PD42S1_HandleTypeDef *motor, int16_t *rpm)
{
  uint8_t rx[8];
  HAL_StatusTypeDef status;

  if (rpm == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_REALTIME_SPEED, rx, sizeof(rx), 2);
  if (status == HAL_OK)
  {
    *rpm = PD42S1_ReadI16BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadRealtimePosition(PD42S1_HandleTypeDef *motor, int32_t *position)
{
  uint8_t rx[10];
  HAL_StatusTypeDef status;

  if (position == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_REALTIME_POSITION, rx, sizeof(rx), 4);
  if (status == HAL_OK)
  {
    *position = PD42S1_ReadI32BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadPositionError(PD42S1_HandleTypeDef *motor, int32_t *error)
{
  uint8_t rx[10];
  HAL_StatusTypeDef status;

  if (error == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_POSITION_ERROR, rx, sizeof(rx), 4);
  if (status == HAL_OK)
  {
    *error = PD42S1_ReadI32BE(&rx[PD42S1_REPLY_DATA_OFFSET]);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadRunStatus(PD42S1_HandleTypeDef *motor, uint8_t *run_status)
{
  uint8_t rx[7];
  HAL_StatusTypeDef status;

  if (run_status == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_RUN_STATUS, rx, sizeof(rx), 1);
  if (status == HAL_OK)
  {
    *run_status = rx[PD42S1_REPLY_DATA_OFFSET];
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadStallFlag(PD42S1_HandleTypeDef *motor, uint8_t *stall_flag)
{
  uint8_t rx[7];
  HAL_StatusTypeDef status;

  if (stall_flag == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_STALL_FLAG, rx, sizeof(rx), 1);
  if (status == HAL_OK)
  {
    *stall_flag = rx[PD42S1_REPLY_DATA_OFFSET];
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadEnableState(PD42S1_HandleTypeDef *motor, uint8_t *enable_state)
{
  uint8_t rx[7];
  HAL_StatusTypeDef status;

  if (enable_state == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_ENABLE_STATE, rx, sizeof(rx), 1);
  if (status == HAL_OK)
  {
    *enable_state = rx[PD42S1_REPLY_DATA_OFFSET];
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadArrivedFlag(PD42S1_HandleTypeDef *motor, uint8_t *arrived_flag)
{
  uint8_t rx[7];
  HAL_StatusTypeDef status;

  if (arrived_flag == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_ARRIVED_FLAG, rx, sizeof(rx), 1);
  if (status == HAL_OK)
  {
    *arrived_flag = rx[PD42S1_REPLY_DATA_OFFSET];
  }

  return status;
}
