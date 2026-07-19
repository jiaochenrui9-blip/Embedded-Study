#ifndef RS05_MIT_H
#define RS05_MIT_H

#include "CAN_bus.h"

#define RS05_MIT_POSITION_MIN_RAD    (-12.57f)
#define RS05_MIT_POSITION_MAX_RAD    (12.57f)
#define RS05_MIT_VELOCITY_MIN_RAD_S  (-50.0f)
#define RS05_MIT_VELOCITY_MAX_RAD_S  (50.0f)
#define RS05_MIT_KP_MIN              (0.0f)
#define RS05_MIT_KP_MAX              (500.0f)
#define RS05_MIT_KD_MIN              (0.0f)
#define RS05_MIT_KD_MAX              (5.0f)
#define RS05_MIT_TORQUE_MIN_NM       (-5.5f)
#define RS05_MIT_TORQUE_MAX_NM       (5.5f)
#define RS05_MIT_FAULT_TIMEOUT_MS     30U
#define RS05_MIT_MOTOR_MAX_COUNT      10U

typedef enum
{
    RS05_MIT_FRAME_CONTROL = 0U,
    RS05_MIT_FRAME_POSITION = 1U,
    RS05_MIT_FRAME_SPEED = 2U,
    RS05_MIT_FRAME_READ_PARAMETER = 3U,
    RS05_MIT_FRAME_WRITE_PARAMETER = 4U
} RS05_MIT_FrameType;

typedef enum
{
    RS05_MIT_RUN_MODE_MIT = 0U,
    RS05_MIT_RUN_MODE_POSITION = 1U,
    RS05_MIT_RUN_MODE_SPEED = 2U
} RS05_MIT_RunMode;

typedef struct
{
    uint8_t motor_id;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
    uint8_t state;
    uint32_t last_feedback_tick;
    uint32_t fault_code;
    uint32_t fault_deadline_tick;
    uint16_t parameter_index;
    uint8_t parameter_data[4];
} RS05_MIT_MotorTypedef;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t master_id;
    RS05_MIT_MotorTypedef *motors[RS05_MIT_MOTOR_MAX_COUNT];
    uint8_t num_motors;
} RS05_MIT_ManagerTypedef;

void RS05_MIT_Manager_Init(RS05_MIT_ManagerTypedef *manager,
                           CAN_HandleTypeDef *hcan,
                           uint8_t master_id);
HAL_StatusTypeDef RS05_MIT_Manager_RegisterMotor(
    RS05_MIT_ManagerTypedef *manager,
    RS05_MIT_MotorTypedef *motor);
RS05_MIT_MotorTypedef *RS05_MIT_Manager_FindMotor(
    const RS05_MIT_ManagerTypedef *manager,
    uint8_t motor_id);

uint16_t RS05_MIT_BuildStdId(uint8_t frame_type, uint8_t motor_id);
HAL_StatusTypeDef RS05_MIT_PackControl(uint8_t data[8],
                                       float position_rad,
                                       float velocity_rad_s,
                                       float kp,
                                       float kd,
                                       float torque_nm);
HAL_StatusTypeDef RS05_MIT_ParseFeedback(RS05_MIT_MotorTypedef *motor,
                                         const uint8_t data[8]);
HAL_StatusTypeDef RS05_MIT_ProcessFrame(
    RS05_MIT_ManagerTypedef *manager,
    uint16_t std_id,
    const uint8_t data[8]);

HAL_StatusTypeDef RS05_MIT_Enable(RS05_MIT_ManagerTypedef *manager,
                                  uint8_t motor_id);
HAL_StatusTypeDef RS05_MIT_Stop(RS05_MIT_ManagerTypedef *manager,
                                uint8_t motor_id);
HAL_StatusTypeDef RS05_MIT_SetZero(RS05_MIT_ManagerTypedef *manager,
                                   uint8_t motor_id);
HAL_StatusTypeDef RS05_MIT_SetRunMode(RS05_MIT_ManagerTypedef *manager,
                                      uint8_t motor_id,
                                      RS05_MIT_RunMode mode);
HAL_StatusTypeDef RS05_MIT_Control(RS05_MIT_ManagerTypedef *manager,
                                   uint8_t motor_id,
                                   float position_rad,
                                   float velocity_rad_s,
                                   float kp,
                                   float kd,
                                   float torque_nm);
HAL_StatusTypeDef RS05_MIT_PositionControl(RS05_MIT_ManagerTypedef *manager,
                                           uint8_t motor_id,
                                           float position_rad,
                                           float velocity_rad_s);
HAL_StatusTypeDef RS05_MIT_SpeedControl(RS05_MIT_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        float velocity_rad_s,
                                        float current_limit_a);
HAL_StatusTypeDef RS05_MIT_ReadFault(RS05_MIT_ManagerTypedef *manager,
                                     RS05_MIT_MotorTypedef *motor);
HAL_StatusTypeDef RS05_MIT_SetActiveReport(RS05_MIT_ManagerTypedef *manager,
                                           uint8_t motor_id,
                                           uint8_t enable);
HAL_StatusTypeDef RS05_MIT_ReadParameter(RS05_MIT_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index);
HAL_StatusTypeDef RS05_MIT_WriteParameterRaw(RS05_MIT_ManagerTypedef *manager,
                                              uint8_t motor_id,
                                              uint16_t index,
                                              const uint8_t value[4]);
HAL_StatusTypeDef RS05_MIT_WriteParameterU8(RS05_MIT_ManagerTypedef *manager,
                                             uint8_t motor_id,
                                             uint16_t index,
                                             uint8_t value);
HAL_StatusTypeDef RS05_MIT_WriteParameterU16(RS05_MIT_ManagerTypedef *manager,
                                              uint8_t motor_id,
                                              uint16_t index,
                                              uint16_t value);
HAL_StatusTypeDef RS05_MIT_WriteParameterU32(RS05_MIT_ManagerTypedef *manager,
                                              uint8_t motor_id,
                                              uint16_t index,
                                              uint32_t value);
HAL_StatusTypeDef RS05_MIT_WriteParameterFloat(RS05_MIT_ManagerTypedef *manager,
                                                uint8_t motor_id,
                                                uint16_t index,
                                                float value);
uint8_t RS05_MIT_IsOnline(const RS05_MIT_MotorTypedef *motor,
                          uint32_t timeout_ms);

#endif /* RS05_MIT_H */
