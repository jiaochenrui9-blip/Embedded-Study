#include "Cylinder.h"

static void Cylinder_WriteU16LE(uint8_t *dst, uint16_t value);
static HAL_StatusTypeDef Cylinder_SendControlCommand(Cylinder_ManagerTypeDef *manager,
                                                      uint8_t device_id,
                                                      uint8_t command,
                                                      uint8_t table_index,
                                                      const uint8_t *data,
                                                      uint8_t data_length);

HAL_StatusTypeDef Cylinder_WriteControlTable(Cylinder_ManagerTypeDef *manager,
                                             uint8_t device_id,
                                             uint8_t start_index,
                                             const uint8_t *data,
                                             uint8_t data_length)
{
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_WRITE, start_index,
                                     data, data_length);
}

HAL_StatusTypeDef Cylinder_SetTargetPosition(Cylinder_ManagerTypeDef *manager,
                                             uint8_t device_id,
                                             uint16_t position_0p01mm)
{
  uint8_t data[2];

  Cylinder_WriteU16LE(data, position_0p01mm);
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_POSITION_FEEDBACK,
                                     CYLINDER_TABLE_TARGET_POS, data, sizeof(data));
}

HAL_StatusTypeDef Cylinder_SetFollowPosition(Cylinder_ManagerTypeDef *manager,
                                             uint8_t device_id,
                                             uint16_t position_0p01mm)
{
  uint8_t data[2];

  Cylinder_WriteU16LE(data, position_0p01mm);
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_FOLLOW_FEEDBACK,
                                     CYLINDER_TABLE_TARGET_POS, data, sizeof(data));
}

HAL_StatusTypeDef Cylinder_Start(Cylinder_ManagerTypeDef *manager, uint8_t device_id)
{
  static const uint8_t data[] = {CYLINDER_CTRL_START};
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_SINGLE_CONTROL,
                                     0U, data, sizeof(data));
}

HAL_StatusTypeDef Cylinder_Pause(Cylinder_ManagerTypeDef *manager, uint8_t device_id)
{
  static const uint8_t data[] = {CYLINDER_CTRL_PAUSE};
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_SINGLE_CONTROL,
                                     0U, data, sizeof(data));
}

HAL_StatusTypeDef Cylinder_EmergencyStop(Cylinder_ManagerTypeDef *manager, uint8_t device_id)
{
  static const uint8_t data[] = {CYLINDER_CTRL_EMERGENCY_STOP};
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_SINGLE_CONTROL,
                                     0U, data, sizeof(data));
}

HAL_StatusTypeDef Cylinder_ClearFault(Cylinder_ManagerTypeDef *manager, uint8_t device_id)
{
  static const uint8_t data[] = {CYLINDER_CTRL_CLEAR_FAULT};
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_SINGLE_CONTROL,
                                     0U, data, sizeof(data));
}

HAL_StatusTypeDef Cylinder_SaveParameters(Cylinder_ManagerTypeDef *manager, uint8_t device_id)
{
  static const uint8_t data[] = {CYLINDER_CTRL_SAVE_PARAMETER};
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_SINGLE_CONTROL,
                                     0U, data, sizeof(data));
}

HAL_StatusTypeDef Cylinder_QueryStatus(Cylinder_ManagerTypeDef *manager, uint8_t device_id)
{
  static const uint8_t data[] = {CYLINDER_CTRL_QUERY_STATUS};
  return Cylinder_SendControlCommand(manager, device_id, CYLINDER_CMD_SINGLE_CONTROL,
                                     0U, data, sizeof(data));
}

static HAL_StatusTypeDef Cylinder_SendControlCommand(Cylinder_ManagerTypeDef *manager,
                                                      uint8_t device_id,
                                                      uint8_t command,
                                                      uint8_t table_index,
                                                      const uint8_t *data,
                                                      uint8_t data_length)
{
  uint8_t reply[CYLINDER_FRAME_MAX_LEN];
  uint8_t reply_length = 0U;
  HAL_StatusTypeDef status;

  status = Cylinder_SendCommand(manager, device_id, command, table_index, data,
                                data_length, reply, sizeof(reply), &reply_length);
  return status == HAL_OK ? Cylinder_ParseFeedback(manager, device_id, reply, reply_length) : status;
}

static void Cylinder_WriteU16LE(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)value;
  dst[1] = (uint8_t)(value >> 8);
}
