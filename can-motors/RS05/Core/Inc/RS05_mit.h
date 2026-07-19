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
    uint8_t master_id,
    RS05_MIT_MotorTypedef *const motors[],
    uint8_t motor_count,
    uint16_t std_id,
    const uint8_t data[8]);

HAL_StatusTypeDef RS05_MIT_Enable(CAN_HandleTypeDef *hcan, uint8_t motor_id);
HAL_StatusTypeDef RS05_MIT_Stop(CAN_HandleTypeDef *hcan, uint8_t motor_id);
HAL_StatusTypeDef RS05_MIT_SetZero(CAN_HandleTypeDef *hcan, uint8_t motor_id);
HAL_StatusTypeDef RS05_MIT_SetRunMode(CAN_HandleTypeDef *hcan,
                                      uint8_t motor_id,
                                      RS05_MIT_RunMode mode);
HAL_StatusTypeDef RS05_MIT_Control(CAN_HandleTypeDef *hcan,
                                   uint8_t motor_id,
                                   float position_rad,
                                   float velocity_rad_s,
                                   float kp,
                                   float kd,
                                   float torque_nm);
HAL_StatusTypeDef RS05_MIT_PositionControl(CAN_HandleTypeDef *hcan,
                                           uint8_t motor_id,
                                           float position_rad,
                                           float velocity_rad_s);
HAL_StatusTypeDef RS05_MIT_SpeedControl(CAN_HandleTypeDef *hcan,
                                        uint8_t motor_id,
                                        float velocity_rad_s,
                                        float current_limit_a);
HAL_StatusTypeDef RS05_MIT_ReadFault(CAN_HandleTypeDef *hcan,
                                     RS05_MIT_MotorTypedef *motor);
HAL_StatusTypeDef RS05_MIT_SetActiveReport(CAN_HandleTypeDef *hcan,
                                           uint8_t motor_id,
                                           uint8_t enable);
HAL_StatusTypeDef RS05_MIT_ReadParameter(CAN_HandleTypeDef *hcan,
                                         uint8_t motor_id,
                                         uint16_t index);
HAL_StatusTypeDef RS05_MIT_WriteParameter(CAN_HandleTypeDef *hcan,
                                          uint8_t motor_id,
                                          uint16_t index,
                                          const uint8_t value[4]);
uint8_t RS05_MIT_IsOnline(const RS05_MIT_MotorTypedef *motor,
                          uint32_t timeout_ms);

#endif /* RS05_MIT_H */
