//
// Created by game on 2026/7/16.
//

#ifndef RS05_RS05_APP_H
#define RS05_RS05_APP_H

#include "CAN_bus.h"
#include "RS05_parameter.h"
#include "RS05_Command.h"

typedef struct
{
    uint8_t motor_id;

    /* 以下状态由 CAN 接收中断写入，应用层可在主循环中读取。 */
     float position_rad;
     float velocity_rad_s;
     float torque_nm;
     float temperature_c;

     uint8_t state;
     uint8_t fault;
     uint32_t last_feedback_tick;

    /* Type 17 参数读取响应，协议中的浮点值采用小端字节序。 */
    RS05_ParameterCache parameter;

} RS05_MotorTypedef;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    RS05_MotorTypedef *motors[RS05_MOTOR_MAX_COUNT];
    uint8_t num_motors;
} RS05_ManagerTypedef;

uint32_t RS05_BuildExtId(uint8_t comm_type,
                         uint16_t data2,
                         uint8_t motor_id);
HAL_StatusTypeDef RS05_SendData(CAN_HandleTypeDef *hcan,
                                uint8_t comm_type,
                                const uint8_t *data,
                                uint16_t data2,
                                uint8_t motor_id);
HAL_StatusTypeDef RS05_Enable(CAN_HandleTypeDef *hcan,
                              uint8_t master_id,
                              uint8_t motor_id);
HAL_StatusTypeDef RS05_MotionControl(CAN_HandleTypeDef *hcan,
                                     uint8_t motor_id,
                                     float position_rad,
                                     float velocity_rad_s,
                                     float kp,
                                     float kd,
                                     float torque_nm);
HAL_StatusTypeDef RS05_Stop(CAN_HandleTypeDef *hcan,
                            uint8_t master_id,
                            uint8_t motor_id);
HAL_StatusTypeDef RS05_SetMotionMode(CAN_HandleTypeDef *hcan,
                                     uint8_t master_id,
                                     uint8_t motor_id);
HAL_StatusTypeDef RS05_Command_ParaRead(RS05_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        uint16_t index);
HAL_StatusTypeDef RS05_WriteParameterU8(RS05_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        uint16_t index,
                                        uint8_t value);
HAL_StatusTypeDef RS05_WriteParameterU16(RS05_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index,
                                         uint16_t value);
HAL_StatusTypeDef RS05_WriteParameterU32(RS05_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index,
                                         uint32_t value);
HAL_StatusTypeDef RS05_WriteParameterFloat(RS05_ManagerTypedef *manager,
                                           uint8_t motor_id,
                                           uint16_t index,
                                           float value);
HAL_StatusTypeDef RS05_SetMode(RS05_ManagerTypedef *manager,
                               RS05_MotorTypedef *motor,
                               uint8_t mode);
HAL_StatusTypeDef RS05_SetSpeedMode(RS05_ManagerTypedef *manager,
                                    RS05_MotorTypedef *motor,
                                    float target_speed_rad_s,
                                    float acceleration_rad_s2,
                                    float current_limit_a);
HAL_StatusTypeDef RS05_SetCurrentMode(RS05_ManagerTypedef *manager,
                                      RS05_MotorTypedef *motor,
                                      float target_current_a);
HAL_StatusTypeDef RS05_SetPPMode(RS05_ManagerTypedef *manager,
                                 RS05_MotorTypedef *motor,
                                 float target_position_rad,
                                 float max_velocity_rad_s,
                                 float acceleration_rad_s2,
                                 float current_limit_a);
HAL_StatusTypeDef RS05_SetCSPMode(RS05_ManagerTypedef *manager,
                                  RS05_MotorTypedef *motor,
                                  float target_position_rad,
                                  float speed_limit_rad_s,
                                  float current_limit_a);
HAL_StatusTypeDef RS05_SetMotionControl(RS05_ManagerTypedef *manager,
                                        RS05_MotorTypedef *motor,
                                        float position_rad,
                                        float velocity_rad_s,
                                        float kp,
                                        float kd,
                                        float torque_ff_nm);

void RS05_Manager_Init(RS05_ManagerTypedef *manager, CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef RS05_Manager_RegisterMotor(RS05_ManagerTypedef *manager,
                                              RS05_MotorTypedef *motor);
uint8_t RS05_IsOnline(RS05_MotorTypedef *motor);

#endif // RS05_RS05_APP_H
