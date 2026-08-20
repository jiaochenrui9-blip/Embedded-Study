//
// Created by game on 2026/8/19.
//

#ifndef ELECTRICCYLINDER_CYLINDER_H
#define ELECTRICCYLINDER_CYLINDER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "Cylinder_Command.h"

#define CYLINDER_MAX_MOTORS            8U
#define CYLINDER_FRAME_MAX_LEN          32U
#define CYLINDER_DEFAULT_TIMEOUT_MS     20U
typedef struct
{
  uint16_t target_position_0p01mm;
  int16_t current_position_0p01mm;
  uint16_t bus_voltage_0p1v;
  int16_t q_axis_current_ma;
  uint16_t overcurrent_limit_ma;
  uint16_t software_version;
  uint8_t max_speed_mm_s;
  uint8_t offline_check_error;
  uint8_t fault_code;
  uint8_t device_id;
  uint8_t online;
  uint32_t last_reply_tick;
} Cylinder_MotorTypeDef;

typedef struct
{
  UART_HandleTypeDef *huart;
  Cylinder_MotorTypeDef motors[CYLINDER_MAX_MOTORS];
  uint8_t motor_count;
} Cylinder_ManagerTypeDef;

void Cylinder_ManagerInit(Cylinder_ManagerTypeDef *manager, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Cylinder_RegisterMotor(Cylinder_ManagerTypeDef *manager, uint8_t device_id);
Cylinder_MotorTypeDef *Cylinder_FindMotor(Cylinder_ManagerTypeDef *manager, uint8_t device_id);

HAL_StatusTypeDef Cylinder_SendCommand(Cylinder_ManagerTypeDef *manager,
                                       uint8_t device_id,
                                       uint8_t command,
                                       uint8_t table_index,
                                       const uint8_t *data,
                                       uint8_t data_length,
                                       uint8_t *reply,
                                       uint8_t reply_max,
                                       uint8_t *reply_length);
HAL_StatusTypeDef Cylinder_ParseFeedback(Cylinder_ManagerTypeDef *manager,
                                         uint8_t device_id,
                                         const uint8_t *reply,
                                         uint8_t reply_length);

HAL_StatusTypeDef Cylinder_WriteControlTable(Cylinder_ManagerTypeDef *manager,
                                             uint8_t device_id,
                                             uint8_t start_index,
                                             const uint8_t *data,
                                             uint8_t data_length);
HAL_StatusTypeDef Cylinder_SetTargetPosition(Cylinder_ManagerTypeDef *manager,
                                             uint8_t device_id,
                                             uint16_t position_0p01mm);
HAL_StatusTypeDef Cylinder_SetFollowPosition(Cylinder_ManagerTypeDef *manager,
                                             uint8_t device_id,
                                             uint16_t position_0p01mm);
HAL_StatusTypeDef Cylinder_Start(Cylinder_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef Cylinder_Pause(Cylinder_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef Cylinder_EmergencyStop(Cylinder_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef Cylinder_ClearFault(Cylinder_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef Cylinder_SaveParameters(Cylinder_ManagerTypeDef *manager, uint8_t device_id);
HAL_StatusTypeDef Cylinder_QueryStatus(Cylinder_ManagerTypeDef *manager, uint8_t device_id);

#endif /* ELECTRICCYLINDER_CYLINDER_H */
