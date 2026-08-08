#ifndef M8010_MOTOR_H
#define M8010_MOTOR_H

#include "../BSP/M8010_Bus.h"

#define M8010_OFFLINE_TIMEOUT_MS 100U
#define M8010_RX_TIMEOUT_MS      10U
typedef struct
{
    M8010_Mode mode;
    float torque_nm;
    float omega_rad_s;
    float position_rad;
    float kp;
    float kw;
} M8010_Command_t;

typedef struct
{
    uint8_t motor_id;
    volatile uint8_t online;
    M8010_Command_t command;
    M8010_Data feedback;

    volatile uint8_t feedback_updated;
    volatile uint32_t feedback_count;
    volatile uint32_t last_feedback_tick;
} M8010_Motor_t;

HAL_StatusTypeDef M8010_Motor_SetCommand(M8010_Motor_t *motor,
                                          M8010_Mode mode,
                                          float torque_nm,
                                          float omega_rad_s,
                                          float position_rad,
                                          float kp,
                                          float kw);

uint8_t M8010_Motor_GetFeedback(M8010_Motor_t *motor,
                                 M8010_Data *feedback);

uint8_t M8010_Motor_Isonline(M8010_Motor_t *motor, uint32_t now);

#endif /* M8010_MOTOR_H */
