#include "PD42S1_Private.h"

HAL_StatusTypeDef PD42S1_Enable(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  uint8_t data[] = {0x00U};
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_ENABLE, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_Disable(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  uint8_t data[] = {0x01U};
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_ENABLE, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_Stop(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_STOP, NULL, 0U, NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_ClearPosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_CLEAR_POSITION, NULL, 0U, NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_SetWorkMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id, uint8_t mode)
{
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_SET_MODE, &mode, 1U, NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_SetPositionMaxTorqueCurrent(PD42S1_ManagerTypeDef *manager,
                                                      uint8_t device_id,
                                                      uint16_t current_ma)
{
  uint8_t data[2];

  PD42S1_WriteU16BE(data, current_ma);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_SET_POSITION_MAX_TORQUE, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_SetTorqueMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SetWorkMode(manager, device_id, PD42S1_MODE_COMM_TORQUE);
}

HAL_StatusTypeDef PD42S1_SetClosedLoopTorqueMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SetTorqueMode(manager, device_id);
}

HAL_StatusTypeDef PD42S1_SetSpeedMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SetWorkMode(manager, device_id, PD42S1_MODE_COMM_SPEED);
}

HAL_StatusTypeDef PD42S1_SetClosedLoopSpeedMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SetSpeedMode(manager, device_id);
}

HAL_StatusTypeDef PD42S1_SetPositionMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SetWorkMode(manager, device_id, PD42S1_MODE_COMM_POSITION);
}

HAL_StatusTypeDef PD42S1_SetClosedLoopPositionMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SetPositionMode(manager, device_id);
}

HAL_StatusTypeDef PD42S1_RunOpenLoopSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                           uint8_t direction, uint8_t acceleration, float rpm)
{
  uint8_t data[6] = {direction, acceleration};

  PD42S1_WriteFloatBE(&data[2], rpm);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_OPEN_SPEED, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_RunOpenLoopAbsolutePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                      uint8_t direction, uint8_t acceleration, uint16_t rpm, uint32_t position)
{
  uint8_t data[8] = {direction, acceleration};

  PD42S1_WriteU16BE(&data[2], rpm);
  PD42S1_WriteU32BE(&data[4], position);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_OPEN_ABS_POSITION, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_RunOpenLoopRelativePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                      uint8_t direction, uint8_t acceleration, uint16_t rpm, uint32_t position)
{
  uint8_t data[8] = {direction, acceleration};

  PD42S1_WriteU16BE(&data[2], rpm);
  PD42S1_WriteU32BE(&data[4], position);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_OPEN_REL_POSITION, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_RunOpenLoopPulse(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_OPEN_PULSE, NULL, 0U, NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_RunIOStartStop(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                        uint8_t direction, uint8_t acceleration, float rpm)
{
  uint8_t data[6] = {direction, acceleration};

  PD42S1_WriteFloatBE(&data[2], rpm);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_IO_START_STOP, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_TorqueMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                    uint8_t direction, uint16_t current_ma)
{
  uint8_t data[3] = {direction};

  PD42S1_WriteU16BE(&data[1], current_ma);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_TORQUE, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_RunClosedLoopTorque(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                             uint8_t direction, uint16_t current_ma)
{
  return PD42S1_TorqueMode(manager, device_id, direction, current_ma);
}

HAL_StatusTypeDef PD42S1_RunSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                  uint8_t direction, uint8_t acceleration, float rpm)
{
  uint8_t data[6] = {direction, acceleration};

  PD42S1_WriteFloatBE(&data[2], rpm);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_SPEED, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_RunClosedLoopSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                            uint8_t direction, uint8_t acceleration, float rpm)
{
  return PD42S1_RunSpeed(manager, device_id, direction, acceleration, rpm);
}

HAL_StatusTypeDef PD42S1_RunRelativePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                             uint8_t direction, uint8_t acceleration, uint16_t rpm, uint32_t position)
{
  uint8_t data[8] = {direction, acceleration};

  PD42S1_WriteU16BE(&data[2], rpm);
  PD42S1_WriteU32BE(&data[4], position);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_REL_POSITION, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_RunClosedLoopRelativePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                       uint8_t direction, uint8_t acceleration, uint16_t rpm, uint32_t position)
{
  return PD42S1_RunRelativePosition(manager, device_id, direction, acceleration, rpm, position);
}

HAL_StatusTypeDef PD42S1_RunAbsolutePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                             uint8_t direction, uint8_t acceleration, uint16_t rpm, uint32_t position)
{
  uint8_t data[8] = {direction, acceleration};

  PD42S1_WriteU16BE(&data[2], rpm);
  PD42S1_WriteU32BE(&data[4], position);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_ABS_POSITION, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_RunClosedLoopAbsolutePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                       uint8_t direction, uint8_t acceleration, uint16_t rpm, uint32_t position)
{
  return PD42S1_RunAbsolutePosition(manager, device_id, direction, acceleration, rpm, position);
}

HAL_StatusTypeDef PD42S1_RunPulseMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_PULSE, NULL, 0U, NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_ConfigPulseWidthPosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                   uint16_t max_width_us, uint16_t min_width_us,
                                                   int32_t max_position, int32_t min_position)
{
  uint8_t data[12];

  PD42S1_WriteU16BE(&data[0], max_width_us);
  PD42S1_WriteU16BE(&data[2], min_width_us);
  PD42S1_WriteI32BE(&data[4], max_position);
  PD42S1_WriteI32BE(&data[8], min_position);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_PULSE_WIDTH_POSITION, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_ConfigPulseWidthTorque(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                 uint16_t max_width_us, uint16_t min_width_us,
                                                 int32_t max_current_ma, int32_t min_current_ma)
{
  uint8_t data[12];

  PD42S1_WriteU16BE(&data[0], max_width_us);
  PD42S1_WriteU16BE(&data[2], min_width_us);
  PD42S1_WriteI32BE(&data[4], max_current_ma);
  PD42S1_WriteI32BE(&data[8], min_current_ma);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_PULSE_WIDTH_TORQUE, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_ConfigPulseWidthSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                uint16_t max_width_us, uint16_t min_width_us,
                                                int32_t max_rpm, int32_t min_rpm)
{
  uint8_t data[12];

  PD42S1_WriteU16BE(&data[0], max_width_us);
  PD42S1_WriteU16BE(&data[2], min_width_us);
  PD42S1_WriteI32BE(&data[4], max_rpm);
  PD42S1_WriteI32BE(&data[8], min_rpm);
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_PULSE_WIDTH_SPEED, data, sizeof(data), NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_ReleaseStallProtection(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_RELEASE_STALL, NULL, 0U, NULL, NULL, 0U, 0U);
}

HAL_StatusTypeDef PD42S1_ClearState(PD42S1_ManagerTypeDef *manager, uint8_t device_id)
{
  return PD42S1_SendCommand(manager, device_id, PD42S1_CMD_CLEAR_STATE, NULL, 0U, NULL, NULL, 0U, 0U);
}
