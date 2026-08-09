#ifndef BUJINTEST_PD42S1_H
#define BUJINTEST_PD42S1_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define PD42S1_DIR_FORWARD 0U
#define PD42S1_DIR_REVERSE 1U
#define PD42S1_POS_PER_ROUND 51200UL
#define PD42S1_MAX_MOTORS 8U

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

typedef struct
{
  int32_t position;
  int32_t position_error;
  int16_t current_ma;
  int16_t realtime_speed_rpm;
  float bus_voltage;
  uint32_t input_pulse;
  uint8_t device_id;
  uint8_t run_status;
  uint8_t stall_flag;
  uint8_t enable_state;
  uint8_t arrived_flag;
} PD42S1_MotorTypeDef;

typedef struct
{
  UART_HandleTypeDef *huart;

  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_PinState tx_state;
  GPIO_PinState rx_state;

  PD42S1_MotorTypeDef motors[PD42S1_MAX_MOTORS];
  uint8_t motor_count;
} PD42S1_ManagerTypeDef;

void PD42S1_ManagerInit(PD42S1_ManagerTypeDef *manager,
                         UART_HandleTypeDef *huart,
                         GPIO_TypeDef *dir_port,
                         uint16_t dir_pin);
HAL_StatusTypeDef PD42S1_RegisterMotor(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
PD42S1_MotorTypeDef *PD42S1_FindMotor(PD42S1_ManagerTypeDef *manager, uint8_t device_id);

HAL_StatusTypeDef PD42S1_ReadVersion(PD42S1_ManagerTypeDef *manager,
                                      uint8_t device_id,
                                       uint8_t *rx,
                                       uint16_t *rx_len,
                                       uint32_t timeout_ms);
HAL_StatusTypeDef PD42S1_ReadPhaseCurrent(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadBusVoltage(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadInputPulse(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadRealtimeSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadRealtimePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadPositionError(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadRunStatus(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadStallFlag(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadEnableState(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ReadArrivedFlag(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_Enable(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_Disable(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_Stop(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ClearPosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_SetWorkMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id, uint8_t mode);
HAL_StatusTypeDef PD42S1_SetPositionMaxTorqueCurrent(PD42S1_ManagerTypeDef *manager,
                                                      uint8_t device_id,
                                                      uint16_t current_ma);
HAL_StatusTypeDef PD42S1_SetTorqueMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_SetSpeedMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_SetPositionMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_SetClosedLoopTorqueMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_SetClosedLoopSpeedMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_SetClosedLoopPositionMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id);

HAL_StatusTypeDef PD42S1_RunOpenLoopSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                            uint8_t direction,
                                            uint8_t acceleration,
                                            float rpm);
HAL_StatusTypeDef PD42S1_RunOpenLoopAbsolutePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                      uint8_t direction,
                                                      uint8_t acceleration,
                                                      uint16_t rpm,
                                                      uint32_t position);
HAL_StatusTypeDef PD42S1_RunOpenLoopRelativePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                      uint8_t direction,
                                                      uint8_t acceleration,
                                                      uint16_t rpm,
                                                      uint32_t position);
HAL_StatusTypeDef PD42S1_RunOpenLoopPulse(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_RunIOStartStop(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                         uint8_t direction,
                                         uint8_t acceleration,
                                         float rpm);

HAL_StatusTypeDef PD42S1_TorqueMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                      uint8_t direction,
                                      uint16_t current_ma);
HAL_StatusTypeDef PD42S1_RunClosedLoopTorque(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                              uint8_t direction,
                                              uint16_t current_ma);
HAL_StatusTypeDef PD42S1_RunSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                   uint8_t direction,
                                   uint8_t acceleration,
                                   float rpm);
HAL_StatusTypeDef PD42S1_RunClosedLoopSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                             uint8_t direction,
                                             uint8_t acceleration,
                                             float rpm);
HAL_StatusTypeDef PD42S1_RunRelativePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                              uint8_t direction,
                                              uint8_t acceleration,
                                              uint16_t rpm,
                                              uint32_t position);
HAL_StatusTypeDef PD42S1_RunClosedLoopRelativePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                        uint8_t direction,
                                                        uint8_t acceleration,
                                                        uint16_t rpm,
                                                        uint32_t position);
HAL_StatusTypeDef PD42S1_RunAbsolutePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                              uint8_t direction,
                                              uint8_t acceleration,
                                              uint16_t rpm,
                                              uint32_t position);
HAL_StatusTypeDef PD42S1_RunClosedLoopAbsolutePosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                        uint8_t direction,
                                                        uint8_t acceleration,
                                                        uint16_t rpm,
                                                        uint32_t position);
HAL_StatusTypeDef PD42S1_RunPulseMode(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ConfigPulseWidthPosition(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                   uint16_t max_width_us,
                                                   uint16_t min_width_us,
                                                   int32_t max_position,
                                                   int32_t min_position);
HAL_StatusTypeDef PD42S1_ConfigPulseWidthTorque(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                 uint16_t max_width_us,
                                                 uint16_t min_width_us,
                                                 int32_t max_current_ma,
                                                 int32_t min_current_ma);
HAL_StatusTypeDef PD42S1_ConfigPulseWidthSpeed(PD42S1_ManagerTypeDef *manager, uint8_t device_id,
                                                uint16_t max_width_us,
                                                uint16_t min_width_us,
                                                int32_t max_rpm,
                                                int32_t min_rpm);
HAL_StatusTypeDef PD42S1_ReleaseStallProtection(PD42S1_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef PD42S1_ClearState(PD42S1_ManagerTypeDef *manager, uint8_t device_id);

#endif /* BUJINTEST_PD42S1_H */
