#include "PD42S1_Private.h"

static HAL_StatusTypeDef PD42S1_SendI32Command(PD42S1_HandleTypeDef *motor,
                                               uint8_t code,
                                               int32_t value);
static HAL_StatusTypeDef PD42S1_RunSignedAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                                          uint8_t acceleration,
                                                          uint16_t rpm,
                                                          int32_t target_position);
static void PD42S1_LiftGetRawLimit(const PD42S1_LiftAxisTypeDef *axis,
                                   PD42S1_PositionLimitTypeDef *raw_limit);

int32_t PD42S1_ClampPosition(const PD42S1_PositionLimitTypeDef *limit, int32_t target_position)  //限制位置
{
  if (limit == NULL)
  {
    return target_position;
  }

  if (target_position < limit->min_position)
  {
    return limit->min_position;
  }

  if (target_position > limit->max_position)
  {
    return limit->max_position;
  }

  return target_position;
}

HAL_StatusTypeDef PD42S1_CaptureMinPosition(PD42S1_HandleTypeDef *motor,
                                            PD42S1_PositionLimitTypeDef *limit)  //更新最小位置
{
  int32_t position;
  HAL_StatusTypeDef status;

  if (limit == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadRealtimePosition(motor, &position);
  if (status == HAL_OK)
  {
    limit->min_position = position;
  }

  return status;
}

HAL_StatusTypeDef PD42S1_CaptureMaxPosition(PD42S1_HandleTypeDef *motor,
                                            PD42S1_PositionLimitTypeDef *limit)  //更新最大位置
{
  int32_t position;
  HAL_StatusTypeDef status;

  if (limit == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadRealtimePosition(motor, &position);
  if (status == HAL_OK)
  {
    limit->max_position = position;
  }

  return status;
}

HAL_StatusTypeDef PD42S1_SetDriverLeftLimitPosition(PD42S1_HandleTypeDef *motor,
                                                    int32_t position)
{
  return PD42S1_SendI32Command(motor, PD42S1_CMD_SET_LEFT_LIMIT_POSITION, position);
}

HAL_StatusTypeDef PD42S1_SetDriverRightLimitPosition(PD42S1_HandleTypeDef *motor,
                                                     int32_t position)
{
  return PD42S1_SendI32Command(motor, PD42S1_CMD_SET_RIGHT_LIMIT_POSITION, position);
}

HAL_StatusTypeDef PD42S1_SetDriverLimitSwitch(PD42S1_HandleTypeDef *motor,
                                              uint8_t enable)
{
  uint8_t data[] = {enable ? 1U : 0U};

  return PD42S1_SendCommand(motor, PD42S1_CMD_SET_LIMIT_SWITCH, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_ReadHomeParams(PD42S1_HandleTypeDef *motor,
                                        PD42S1_HomeParamTypeDef *params)
{
  uint8_t rx[24];
  const uint8_t *data = &rx[PD42S1_REPLY_DATA_OFFSET];
  HAL_StatusTypeDef status;

  if (params == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadData(motor, PD42S1_CMD_READ_HOME_PARAM, rx, sizeof(rx), 17);
  if (status == HAL_OK)
  {
    params->auto_home_enabled = data[0];
    params->home_status = data[1];
    params->limit_current_ma = PD42S1_ReadI16BE(&data[2]);
    params->left_limit_position = PD42S1_ReadI32BE(&data[4]);
    params->timeout_ms = PD42S1_ReadU32BE(&data[8]);
    params->right_limit_position = PD42S1_ReadI32BE(&data[12]);
    params->limit_switch_enabled = data[16];
  }

  return status;
}

HAL_StatusTypeDef PD42S1_SetAutoHome(PD42S1_HandleTypeDef *motor,
                                     uint8_t enable)  //上电是否自动回零
{
  uint8_t data[] = {enable ? 1U : 0U};

  return PD42S1_SendCommand(motor, PD42S1_CMD_SET_AUTO_HOME, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_ConfigLimitHome(PD42S1_HandleTypeDef *motor,
                                         uint8_t limit_home_mode,
                                         uint8_t direction,
                                         uint32_t rpm,
                                         uint16_t limit_current_ma)   //配置回零方式
{
  uint8_t data[8];

  if (limit_home_mode > PD42S1_LIMIT_HOME_RIGHT_WITH_SWITCH ||
      direction > PD42S1_DIR_REVERSE ||
      rpm > 6000U ||
      limit_current_ma > 3000U)
  {
    return HAL_ERROR;
  }

  data[0] = limit_home_mode;
  data[1] = direction;
  PD42S1_WriteU32BE(&data[2], rpm);
  PD42S1_WriteU16BE(&data[6], limit_current_ma);

  return PD42S1_SendCommand(motor, PD42S1_CMD_SET_HOME_MODE, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_TriggerHome(PD42S1_HandleTypeDef *motor,
                                     uint8_t home_mode)     //触发回零
{
  uint8_t data[] = {home_mode};

  if (home_mode > PD42S1_HOME_MULTI_TURN)
  {
    return HAL_ERROR;
  }

  return PD42S1_SendCommand(motor, PD42S1_CMD_TRIGGER_HOME, data, sizeof(data), NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_AbortHome(PD42S1_HandleTypeDef *motor)   //中断回零
{
  return PD42S1_SendCommand(motor, PD42S1_CMD_ABORT_HOME, NULL, 0, NULL, NULL, 0, 0);
}

HAL_StatusTypeDef PD42S1_RunLimitedAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                                    const PD42S1_PositionLimitTypeDef *limit,
                                                    uint8_t acceleration,
                                                    uint16_t rpm,
                                                    int32_t target_position)
{
  int32_t limited_position;

  if (limit != NULL && limit->min_position > limit->max_position)
  {
    return HAL_ERROR;
  }

  limited_position = PD42S1_ClampPosition(limit, target_position);
  return PD42S1_RunSignedAbsolutePosition(motor, acceleration, rpm, limited_position);
}

HAL_StatusTypeDef PD42S1_RunLimitedRelativePosition(PD42S1_HandleTypeDef *motor,
                                                    const PD42S1_PositionLimitTypeDef *limit,
                                                    uint8_t acceleration,
                                                    uint16_t rpm,
                                                    int32_t move_distance)
{
  int32_t current_position;
  HAL_StatusTypeDef status;

  status = PD42S1_ReadRealtimePosition(motor, &current_position);
  if (status != HAL_OK)
  {
    return status;
  }

  return PD42S1_RunLimitedAbsolutePosition(motor,
                                           limit,
                                           acceleration,
                                           rpm,
                                           current_position + move_distance);
}

void PD42S1_LiftAxisInit(PD42S1_LiftAxisTypeDef *axis)
{
  if (axis == NULL)
  {
    return;
  }

  axis->zero_position = 0;
  axis->max_height = 0;
  axis->up_direction = 1;
}

int32_t PD42S1_LiftRawToHeight(const PD42S1_LiftAxisTypeDef *axis,
                               int32_t raw_position)
{
  if (axis == NULL || axis->up_direction == 0)
  {
    return raw_position;
  }

  return (raw_position - axis->zero_position) * axis->up_direction;
}

int32_t PD42S1_LiftHeightToRaw(const PD42S1_LiftAxisTypeDef *axis,
                               int32_t height)
{
  int32_t limited_height = height;

  if (axis == NULL || axis->up_direction == 0)
  {
    return height;
  }

  if (limited_height < 0)
  {
    limited_height = 0;
  }

  if (limited_height > axis->max_height)
  {
    limited_height = axis->max_height;
  }

  return axis->zero_position + limited_height * axis->up_direction;
}

HAL_StatusTypeDef PD42S1_LiftCaptureZero(PD42S1_HandleTypeDef *motor,
                                         PD42S1_LiftAxisTypeDef *axis)
{
  int32_t raw_position;
  HAL_StatusTypeDef status;

  if (axis == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadRealtimePosition(motor, &raw_position);
  if (status == HAL_OK)
  {
    axis->zero_position = raw_position;
    axis->max_height = 0;
    axis->up_direction = 1;
  }

  return status;
}

HAL_StatusTypeDef PD42S1_LiftCaptureMax(PD42S1_HandleTypeDef *motor,
                                        PD42S1_LiftAxisTypeDef *axis)
{
  int32_t raw_position;
  int32_t delta;
  HAL_StatusTypeDef status;

  if (axis == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadRealtimePosition(motor, &raw_position);
  if (status == HAL_OK)
  {
    delta = raw_position - axis->zero_position;
    if (delta < 0)
    {
      axis->up_direction = -1;
      axis->max_height = -delta;
    }
    else
    {
      axis->up_direction = 1;
      axis->max_height = delta;
    }
  }

  return status;
}

HAL_StatusTypeDef PD42S1_ReadLiftHeight(PD42S1_HandleTypeDef *motor,
                                        const PD42S1_LiftAxisTypeDef *axis,
                                        int32_t *height)
{
  int32_t raw_position;
  HAL_StatusTypeDef status;

  if (height == NULL)
  {
    return HAL_ERROR;
  }

  status = PD42S1_ReadRealtimePosition(motor, &raw_position);
  if (status == HAL_OK)
  {
    *height = PD42S1_LiftRawToHeight(axis, raw_position);
  }

  return status;
}

HAL_StatusTypeDef PD42S1_RunLiftToHeight(PD42S1_HandleTypeDef *motor,
                                         const PD42S1_LiftAxisTypeDef *axis,
                                         uint8_t acceleration,
                                         uint16_t rpm,
                                         int32_t target_height)
{
  PD42S1_PositionLimitTypeDef raw_limit;
  int32_t target_raw_position;

  if (axis == NULL || axis->up_direction == 0 || axis->max_height < 0)
  {
    return HAL_ERROR;
  }

  PD42S1_LiftGetRawLimit(axis, &raw_limit);
  target_raw_position = PD42S1_LiftHeightToRaw(axis, target_height);

  return PD42S1_RunLimitedAbsolutePosition(motor,
                                           &raw_limit,
                                           acceleration,
                                           rpm,
                                           target_raw_position);
}

HAL_StatusTypeDef PD42S1_RunLiftRelativeHeight(PD42S1_HandleTypeDef *motor,
                                               const PD42S1_LiftAxisTypeDef *axis,
                                               uint8_t acceleration,
                                               uint16_t rpm,
                                               int32_t move_height)
{
  int32_t current_height;
  HAL_StatusTypeDef status;

  status = PD42S1_ReadLiftHeight(motor, axis, &current_height);
  if (status != HAL_OK)
  {
    return status;
  }

  return PD42S1_RunLiftToHeight(motor, axis, acceleration, rpm, current_height + move_height);
}

static HAL_StatusTypeDef PD42S1_SendI32Command(PD42S1_HandleTypeDef *motor,
                                               uint8_t code,
                                               int32_t value)
{
  uint8_t data[4];

  PD42S1_WriteI32BE(data, value);
  return PD42S1_SendCommand(motor, code, data, sizeof(data), NULL, NULL, 0, 0);
}

static HAL_StatusTypeDef PD42S1_RunSignedAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                                          uint8_t acceleration,
                                                          uint16_t rpm,
                                                          int32_t target_position)  //运行有符号的绝对位置
{
  uint8_t direction = PD42S1_DIR_FORWARD;
  uint32_t position = (uint32_t)target_position;

  if (target_position < 0)
  {
    direction = PD42S1_DIR_REVERSE;
    position = (uint32_t)(-target_position);
  }

  return PD42S1_RunClosedLoopAbsolutePosition(motor, direction, acceleration, rpm, position);
}

static void PD42S1_LiftGetRawLimit(const PD42S1_LiftAxisTypeDef *axis,
                                   PD42S1_PositionLimitTypeDef *raw_limit)
{
  int32_t zero_raw;
  int32_t max_raw;

  if (axis == NULL || raw_limit == NULL)
  {
    return;
  }

  zero_raw = axis->zero_position;
  max_raw = PD42S1_LiftHeightToRaw(axis, axis->max_height);

  if (zero_raw <= max_raw)
  {
    raw_limit->min_position = zero_raw;
    raw_limit->max_position = max_raw;
  }
  else
  {
    raw_limit->min_position = max_raw;
    raw_limit->max_position = zero_raw;
  }
}
