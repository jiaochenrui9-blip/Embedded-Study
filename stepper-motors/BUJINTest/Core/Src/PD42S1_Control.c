#include "PD42S1_Private.h"

HAL_StatusTypeDef PD42S1_Enable(PD42S1_HandleTypeDef *motor)
{
  uint8_t data[] = {0x00U};
  return PD42S1_SendCommand(motor, PD42S1_CMD_ENABLE, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_Disable(PD42S1_HandleTypeDef *motor)
{
  uint8_t data[] = {0x01U};
  return PD42S1_SendCommand(motor, PD42S1_CMD_ENABLE, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_Stop(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SendCommand(motor, PD42S1_CMD_STOP, NULL, 0, NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_ClearPosition(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SendCommand(motor, PD42S1_CMD_CLEAR_POSITION, NULL, 0, NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_SetWorkMode(PD42S1_HandleTypeDef *motor, uint8_t mode)
{
  uint8_t data[] = {mode};

  if (mode > PD42S1_MODE_IO_START_STOP)
  {
    return HAL_ERROR;
  }

  return PD42S1_SendCommand(motor, PD42S1_CMD_SET_MODE, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_SetTorqueMode(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SetWorkMode(motor, PD42S1_MODE_COMM_TORQUE);
}

HAL_StatusTypeDef PD42S1_SetClosedLoopTorqueMode(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SetTorqueMode(motor);
}

HAL_StatusTypeDef PD42S1_SetSpeedMode(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SetWorkMode(motor, PD42S1_MODE_COMM_SPEED);
}

HAL_StatusTypeDef PD42S1_SetClosedLoopSpeedMode(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SetSpeedMode(motor);
}

HAL_StatusTypeDef PD42S1_SetPositionMode(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SetWorkMode(motor, PD42S1_MODE_COMM_POSITION);
}

HAL_StatusTypeDef PD42S1_SetClosedLoopPositionMode(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SetPositionMode(motor);
}

HAL_StatusTypeDef PD42S1_RunOpenLoopSpeed(PD42S1_HandleTypeDef *motor,
                                           uint8_t direction,
                                           uint8_t acceleration,
                                           float rpm)
{
  uint8_t data[6];

  if (PD42S1_ValidateDirectionAndAccel(direction, acceleration) != HAL_OK ||
      rpm < 0.1f || rpm > 400.0f)
  {
    return HAL_ERROR;
  }

  data[0] = direction;
  data[1] = acceleration;
  PD42S1_WriteFloatBE(&data[2], rpm);

  return PD42S1_SendCommand(motor, PD42S1_CMD_OPEN_SPEED, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunOpenLoopAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                                     uint8_t direction,
                                                     uint8_t acceleration,
                                                     uint16_t rpm,
                                                     uint32_t position)
{
  uint8_t data[8];

  if (PD42S1_ValidateDirectionAndAccel(direction, acceleration) != HAL_OK || rpm > 400U)
  {
    return HAL_ERROR;
  }

  data[0] = direction;
  data[1] = acceleration;
  PD42S1_WriteU16BE(&data[2], rpm);
  PD42S1_WriteU32BE(&data[4], position);

  return PD42S1_SendCommand(motor, PD42S1_CMD_OPEN_ABS_POSITION, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunOpenLoopRelativePosition(PD42S1_HandleTypeDef *motor,
                                                     uint8_t direction,
                                                     uint8_t acceleration,
                                                     uint16_t rpm,
                                                     uint32_t position)
{
  uint8_t data[8];

  if (PD42S1_ValidateDirectionAndAccel(direction, acceleration) != HAL_OK || rpm > 400U)
  {
    return HAL_ERROR;
  }

  data[0] = direction;
  data[1] = acceleration;
  PD42S1_WriteU16BE(&data[2], rpm);
  PD42S1_WriteU32BE(&data[4], position);

  return PD42S1_SendCommand(motor, PD42S1_CMD_OPEN_REL_POSITION, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunOpenLoopPulse(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SendCommand(motor, PD42S1_CMD_OPEN_PULSE, NULL, 0, NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunIOStartStop(PD42S1_HandleTypeDef *motor,
                                        uint8_t direction,
                                        uint8_t acceleration,
                                        float rpm)
{
  uint8_t data[6];

  if (PD42S1_ValidateDirectionAndAccel(direction, acceleration) != HAL_OK ||
      rpm < 0.1f || rpm > 400.0f)
  {
    return HAL_ERROR;
  }

  data[0] = direction;
  data[1] = acceleration;
  PD42S1_WriteFloatBE(&data[2], rpm);

  return PD42S1_SendCommand(motor, PD42S1_CMD_IO_START_STOP, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_TorqueMode(PD42S1_HandleTypeDef *motor,
                                    uint8_t direction,
                                    uint16_t current_ma)
{
  uint8_t data[3];

  if (direction > PD42S1_DIR_REVERSE || current_ma > 3000U)
  {
    return HAL_ERROR;
  }

  data[0] = direction;
  PD42S1_WriteU16BE(&data[1], current_ma);

  return PD42S1_SendCommand(motor, PD42S1_CMD_TORQUE, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunClosedLoopTorque(PD42S1_HandleTypeDef *motor,
                                             uint8_t direction,
                                             uint16_t current_ma)
{
  return PD42S1_TorqueMode(motor, direction, current_ma);
}

HAL_StatusTypeDef PD42S1_RunSpeed(PD42S1_HandleTypeDef *motor,
                                  uint8_t direction,
                                  uint8_t acceleration,
                                  float rpm)
{
  uint8_t data[6];

  if (PD42S1_ValidateDirectionAndAccel(direction, acceleration) != HAL_OK ||
      rpm < 0.1f || rpm > 6000.0f)
  {
    return HAL_ERROR;
  }

  data[0] = direction;
  data[1] = acceleration;
  PD42S1_WriteFloatBE(&data[2], rpm);

  return PD42S1_SendCommand(motor, PD42S1_CMD_SPEED, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunClosedLoopSpeed(PD42S1_HandleTypeDef *motor,
                                            uint8_t direction,
                                            uint8_t acceleration,
                                            float rpm)
{
  return PD42S1_RunSpeed(motor, direction, acceleration, rpm);
}

HAL_StatusTypeDef PD42S1_RunRelativePosition(PD42S1_HandleTypeDef *motor,
                                             uint8_t direction,
                                             uint8_t acceleration,
                                             uint16_t rpm,
                                             uint32_t position)
{
  uint8_t data[8];

  if (PD42S1_ValidateDirectionAndAccel(direction, acceleration) != HAL_OK || rpm > 6000U)
  {
    return HAL_ERROR;
  }

  data[0] = direction;
  data[1] = acceleration;
  PD42S1_WriteU16BE(&data[2], rpm);
  PD42S1_WriteU32BE(&data[4], position);

  return PD42S1_SendCommand(motor, PD42S1_CMD_REL_POSITION, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunClosedLoopRelativePosition(PD42S1_HandleTypeDef *motor,
                                                       uint8_t direction,
                                                       uint8_t acceleration,
                                                       uint16_t rpm,
                                                       uint32_t position)
{
  return PD42S1_RunRelativePosition(motor, direction, acceleration, rpm, position);
}

HAL_StatusTypeDef PD42S1_RunAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                             uint8_t direction,
                                             uint8_t acceleration,
                                             uint16_t rpm,
                                             uint32_t position)
{
  uint8_t data[8];

  if (PD42S1_ValidateDirectionAndAccel(direction, acceleration) != HAL_OK || rpm > 6000U)
  {
    return HAL_ERROR;
  }

  data[0] = direction;
  data[1] = acceleration;
  PD42S1_WriteU16BE(&data[2], rpm);
  PD42S1_WriteU32BE(&data[4], position);

  return PD42S1_SendCommand(motor, PD42S1_CMD_ABS_POSITION, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunClosedLoopAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                                       uint8_t direction,
                                                       uint8_t acceleration,
                                                       uint16_t rpm,
                                                       uint32_t position)
{
  return PD42S1_RunAbsolutePosition(motor, direction, acceleration, rpm, position);
}

HAL_StatusTypeDef PD42S1_RunPulseMode(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SendCommand(motor, PD42S1_CMD_PULSE, NULL, 0, NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_ConfigPulseWidthPosition(PD42S1_HandleTypeDef *motor,
                                                  uint16_t max_width_us,
                                                  uint16_t min_width_us,
                                                  int32_t max_position,
                                                  int32_t min_position)
{
  uint8_t data[12];

  if (PD42S1_ValidatePulseWidth(max_width_us, min_width_us) != HAL_OK)
  {
    return HAL_ERROR;
  }

  PD42S1_WriteU16BE(&data[0], max_width_us);
  PD42S1_WriteU16BE(&data[2], min_width_us);
  PD42S1_WriteI32BE(&data[4], max_position);
  PD42S1_WriteI32BE(&data[8], min_position);

  return PD42S1_SendCommand(motor, PD42S1_CMD_PULSE_WIDTH_POSITION, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_ConfigPulseWidthTorque(PD42S1_HandleTypeDef *motor,
                                                uint16_t max_width_us,
                                                uint16_t min_width_us,
                                                int32_t max_current_ma,
                                                int32_t min_current_ma)
{
  uint8_t data[12];

  if (PD42S1_ValidatePulseWidth(max_width_us, min_width_us) != HAL_OK)
  {
    return HAL_ERROR;
  }

  PD42S1_WriteU16BE(&data[0], max_width_us);
  PD42S1_WriteU16BE(&data[2], min_width_us);
  PD42S1_WriteI32BE(&data[4], max_current_ma);
  PD42S1_WriteI32BE(&data[8], min_current_ma);

  return PD42S1_SendCommand(motor, PD42S1_CMD_PULSE_WIDTH_TORQUE, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_ConfigPulseWidthSpeed(PD42S1_HandleTypeDef *motor,
                                               uint16_t max_width_us,
                                               uint16_t min_width_us,
                                               int32_t max_rpm,
                                               int32_t min_rpm)
{
  uint8_t data[12];

  if (PD42S1_ValidatePulseWidth(max_width_us, min_width_us) != HAL_OK)
  {
    return HAL_ERROR;
  }

  PD42S1_WriteU16BE(&data[0], max_width_us);
  PD42S1_WriteU16BE(&data[2], min_width_us);
  PD42S1_WriteI32BE(&data[4], max_rpm);
  PD42S1_WriteI32BE(&data[8], min_rpm);

  return PD42S1_SendCommand(motor, PD42S1_CMD_PULSE_WIDTH_SPEED, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_ReleaseStallProtection(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SendCommand(motor, PD42S1_CMD_RELEASE_STALL, NULL, 0, NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_ClearState(PD42S1_HandleTypeDef *motor)
{
  return PD42S1_SendCommand(motor, PD42S1_CMD_CLEAR_STATE, NULL, 0, NULL, NULL, 0, 0);
}
