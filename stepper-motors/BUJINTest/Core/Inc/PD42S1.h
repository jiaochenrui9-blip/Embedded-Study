#ifndef BUJINTEST_PD42S1_H
#define BUJINTEST_PD42S1_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define PD42S1_DIR_FORWARD 0U
#define PD42S1_DIR_REVERSE 1U
#define PD42S1_POS_PER_ROUND 51200UL
#define PD42S1_PULSE_WIDTH_MIN_US 1U
#define PD42S1_PULSE_WIDTH_MAX_US 50000U

#define PD42S1_MODE_COMM_POSITION 0x00U
#define PD42S1_MODE_COMM_SPEED 0x01U
#define PD42S1_MODE_COMM_TORQUE 0x02U
#define PD42S1_MODE_CLOSED_LOOP_POSITION PD42S1_MODE_COMM_POSITION
#define PD42S1_MODE_CLOSED_LOOP_SPEED PD42S1_MODE_COMM_SPEED
#define PD42S1_MODE_CLOSED_LOOP_TORQUE PD42S1_MODE_COMM_TORQUE
#define PD42S1_MODE_PULSE 0x03U
#define PD42S1_MODE_PULSE_WIDTH_POSITION 0x04U
#define PD42S1_MODE_PULSE_WIDTH_SPEED 0x05U
#define PD42S1_MODE_PULSE_WIDTH_TORQUE 0x06U
#define PD42S1_MODE_HOME 0x07U
#define PD42S1_MODE_OPEN_SPEED 0x08U
#define PD42S1_MODE_OPEN_POSITION 0x09U
#define PD42S1_MODE_OPEN_PULSE 0x0AU
#define PD42S1_MODE_IO_START_STOP 0x0BU

#define PD42S1_HOME_NEAREST 0U
#define PD42S1_HOME_SINGLE_TURN 1U
#define PD42S1_HOME_MULTI_TURN 2U

#define PD42S1_LIMIT_HOME_LEFT_NO_SWITCH 0U
#define PD42S1_LIMIT_HOME_RIGHT_NO_SWITCH 1U
#define PD42S1_LIMIT_HOME_LEFT_WITH_SWITCH 2U
#define PD42S1_LIMIT_HOME_RIGHT_WITH_SWITCH 3U

#define PD42S1_HOME_STATUS_IDLE 0U
#define PD42S1_HOME_STATUS_RUNNING 1U
#define PD42S1_HOME_STATUS_DONE 2U
#define PD42S1_HOME_STATUS_FAILED 3U

typedef struct
{
  UART_HandleTypeDef *huart;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_PinState tx_state;
  GPIO_PinState rx_state;
  uint8_t addr;
} PD42S1_HandleTypeDef;

typedef struct
{
  int32_t min_position;
  int32_t max_position;
} PD42S1_PositionLimitTypeDef;

typedef struct
{
  int32_t zero_position;
  int32_t max_height;
  int8_t up_direction;
} PD42S1_LiftAxisTypeDef;

typedef struct
{
  uint8_t auto_home_enabled;
  uint8_t home_status;
  int16_t limit_current_ma;
  int32_t left_limit_position;
  uint32_t timeout_ms;
  int32_t right_limit_position;
  uint8_t limit_switch_enabled;
} PD42S1_HomeParamTypeDef;

void PD42S1_SetAddress(PD42S1_HandleTypeDef *motor, uint8_t addr);
void PD42S1_Init(PD42S1_HandleTypeDef *motor,
                 UART_HandleTypeDef *huart,
                 GPIO_TypeDef *dir_port,
                 uint16_t dir_pin);

HAL_StatusTypeDef PD42S1_ReadVersion(PD42S1_HandleTypeDef *motor,
                                      uint8_t *rx,
                                      uint16_t *rx_len,
                                      uint32_t timeout_ms);
HAL_StatusTypeDef PD42S1_ReadPhaseCurrent(PD42S1_HandleTypeDef *motor, int16_t *current_ma);
HAL_StatusTypeDef PD42S1_ReadBusVoltage(PD42S1_HandleTypeDef *motor, float *voltage);
HAL_StatusTypeDef PD42S1_ReadInputPulse(PD42S1_HandleTypeDef *motor, uint32_t *pulse_count);
HAL_StatusTypeDef PD42S1_ReadRealtimeSpeed(PD42S1_HandleTypeDef *motor, int16_t *rpm);
HAL_StatusTypeDef PD42S1_ReadRealtimePosition(PD42S1_HandleTypeDef *motor, int32_t *position);
HAL_StatusTypeDef PD42S1_ReadPositionError(PD42S1_HandleTypeDef *motor, int32_t *error);
HAL_StatusTypeDef PD42S1_ReadRunStatus(PD42S1_HandleTypeDef *motor, uint8_t *run_status);
HAL_StatusTypeDef PD42S1_ReadStallFlag(PD42S1_HandleTypeDef *motor, uint8_t *stall_flag);
HAL_StatusTypeDef PD42S1_ReadEnableState(PD42S1_HandleTypeDef *motor, uint8_t *enable_state);
HAL_StatusTypeDef PD42S1_ReadArrivedFlag(PD42S1_HandleTypeDef *motor, uint8_t *arrived_flag);
HAL_StatusTypeDef PD42S1_Enable(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_Disable(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_Stop(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_ClearPosition(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_SetWorkMode(PD42S1_HandleTypeDef *motor, uint8_t mode);
HAL_StatusTypeDef PD42S1_SetTorqueMode(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_SetSpeedMode(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_SetPositionMode(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_SetClosedLoopTorqueMode(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_SetClosedLoopSpeedMode(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_SetClosedLoopPositionMode(PD42S1_HandleTypeDef *motor);

HAL_StatusTypeDef PD42S1_RunOpenLoopSpeed(PD42S1_HandleTypeDef *motor,
                                           uint8_t direction,
                                           uint8_t acceleration,
                                           float rpm);
HAL_StatusTypeDef PD42S1_RunOpenLoopAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                                     uint8_t direction,
                                                     uint8_t acceleration,
                                                     uint16_t rpm,
                                                     uint32_t position);
HAL_StatusTypeDef PD42S1_RunOpenLoopRelativePosition(PD42S1_HandleTypeDef *motor,
                                                     uint8_t direction,
                                                     uint8_t acceleration,
                                                     uint16_t rpm,
                                                     uint32_t position);
HAL_StatusTypeDef PD42S1_RunOpenLoopPulse(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_RunIOStartStop(PD42S1_HandleTypeDef *motor,
                                        uint8_t direction,
                                        uint8_t acceleration,
                                        float rpm);

HAL_StatusTypeDef PD42S1_TorqueMode(PD42S1_HandleTypeDef *motor,
                                     uint8_t direction,
                                     uint16_t current_ma);
HAL_StatusTypeDef PD42S1_RunClosedLoopTorque(PD42S1_HandleTypeDef *motor,
                                             uint8_t direction,
                                             uint16_t current_ma);
HAL_StatusTypeDef PD42S1_RunSpeed(PD42S1_HandleTypeDef *motor,
                                  uint8_t direction,
                                  uint8_t acceleration,
                                  float rpm);
HAL_StatusTypeDef PD42S1_RunClosedLoopSpeed(PD42S1_HandleTypeDef *motor,
                                            uint8_t direction,
                                            uint8_t acceleration,
                                            float rpm);
HAL_StatusTypeDef PD42S1_RunRelativePosition(PD42S1_HandleTypeDef *motor,
                                             uint8_t direction,
                                             uint8_t acceleration,
                                             uint16_t rpm,
                                             uint32_t position);
HAL_StatusTypeDef PD42S1_RunClosedLoopRelativePosition(PD42S1_HandleTypeDef *motor,
                                                       uint8_t direction,
                                                       uint8_t acceleration,
                                                       uint16_t rpm,
                                                       uint32_t position);
HAL_StatusTypeDef PD42S1_RunAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                             uint8_t direction,
                                             uint8_t acceleration,
                                             uint16_t rpm,
                                             uint32_t position);
HAL_StatusTypeDef PD42S1_RunClosedLoopAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                                       uint8_t direction,
                                                       uint8_t acceleration,
                                                       uint16_t rpm,
                                                       uint32_t position);
HAL_StatusTypeDef PD42S1_RunPulseMode(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_ConfigPulseWidthPosition(PD42S1_HandleTypeDef *motor,
                                                  uint16_t max_width_us,
                                                  uint16_t min_width_us,
                                                  int32_t max_position,
                                                  int32_t min_position);
HAL_StatusTypeDef PD42S1_ConfigPulseWidthTorque(PD42S1_HandleTypeDef *motor,
                                                uint16_t max_width_us,
                                                uint16_t min_width_us,
                                                int32_t max_current_ma,
                                                int32_t min_current_ma);
HAL_StatusTypeDef PD42S1_ConfigPulseWidthSpeed(PD42S1_HandleTypeDef *motor,
                                               uint16_t max_width_us,
                                               uint16_t min_width_us,
                                               int32_t max_rpm,
                                               int32_t min_rpm);
HAL_StatusTypeDef PD42S1_ReleaseStallProtection(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_ClearState(PD42S1_HandleTypeDef *motor);

int32_t PD42S1_ClampPosition(const PD42S1_PositionLimitTypeDef *limit, int32_t target_position);
HAL_StatusTypeDef PD42S1_CaptureMinPosition(PD42S1_HandleTypeDef *motor,
                                            PD42S1_PositionLimitTypeDef *limit);
HAL_StatusTypeDef PD42S1_CaptureMaxPosition(PD42S1_HandleTypeDef *motor,
                                            PD42S1_PositionLimitTypeDef *limit);
HAL_StatusTypeDef PD42S1_SetDriverLeftLimitPosition(PD42S1_HandleTypeDef *motor,
                                                    int32_t position);
HAL_StatusTypeDef PD42S1_SetDriverRightLimitPosition(PD42S1_HandleTypeDef *motor,
                                                     int32_t position);
HAL_StatusTypeDef PD42S1_SetDriverLimitSwitch(PD42S1_HandleTypeDef *motor,
                                              uint8_t enable);
HAL_StatusTypeDef PD42S1_ReadHomeParams(PD42S1_HandleTypeDef *motor,
                                        PD42S1_HomeParamTypeDef *params);
HAL_StatusTypeDef PD42S1_SetAutoHome(PD42S1_HandleTypeDef *motor,
                                     uint8_t enable);
HAL_StatusTypeDef PD42S1_ConfigLimitHome(PD42S1_HandleTypeDef *motor,
                                         uint8_t limit_home_mode,
                                         uint8_t direction,
                                         uint32_t rpm,
                                         uint16_t limit_current_ma);
HAL_StatusTypeDef PD42S1_TriggerHome(PD42S1_HandleTypeDef *motor,
                                     uint8_t home_mode);
HAL_StatusTypeDef PD42S1_AbortHome(PD42S1_HandleTypeDef *motor);
HAL_StatusTypeDef PD42S1_RunLimitedAbsolutePosition(PD42S1_HandleTypeDef *motor,
                                                    const PD42S1_PositionLimitTypeDef *limit,
                                                    uint8_t acceleration,
                                                    uint16_t rpm,
                                                    int32_t target_position);
HAL_StatusTypeDef PD42S1_RunLimitedRelativePosition(PD42S1_HandleTypeDef *motor,
                                                    const PD42S1_PositionLimitTypeDef *limit,
                                                    uint8_t acceleration,
                                                    uint16_t rpm,
                                                    int32_t move_distance);
void PD42S1_LiftAxisInit(PD42S1_LiftAxisTypeDef *axis);
int32_t PD42S1_LiftRawToHeight(const PD42S1_LiftAxisTypeDef *axis,
                               int32_t raw_position);
int32_t PD42S1_LiftHeightToRaw(const PD42S1_LiftAxisTypeDef *axis,
                               int32_t height);
HAL_StatusTypeDef PD42S1_LiftCaptureZero(PD42S1_HandleTypeDef *motor,
                                         PD42S1_LiftAxisTypeDef *axis);
HAL_StatusTypeDef PD42S1_LiftCaptureMax(PD42S1_HandleTypeDef *motor,
                                        PD42S1_LiftAxisTypeDef *axis);
HAL_StatusTypeDef PD42S1_ReadLiftHeight(PD42S1_HandleTypeDef *motor,
                                        const PD42S1_LiftAxisTypeDef *axis,
                                        int32_t *height);
HAL_StatusTypeDef PD42S1_RunLiftToHeight(PD42S1_HandleTypeDef *motor,
                                         const PD42S1_LiftAxisTypeDef *axis,
                                         uint8_t acceleration,
                                         uint16_t rpm,
                                         int32_t target_height);
HAL_StatusTypeDef PD42S1_RunLiftRelativeHeight(PD42S1_HandleTypeDef *motor,
                                               const PD42S1_LiftAxisTypeDef *axis,
                                               uint8_t acceleration,
                                               uint16_t rpm,
                                               int32_t move_height);

#endif /* BUJINTEST_PD42S1_H */
