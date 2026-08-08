#include "m8010_motor.h"

HAL_StatusTypeDef M8010_Motor_SetCommand(M8010_Motor_t *motor,
                                          M8010_Mode mode,
                                          float torque_nm,
                                          float omega_rad_s,
                                          float position_rad,
                                          float kp,
                                          float kw)
{
    uint8_t frame[M8010_FRAME_SIZE];

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    if (M8010_BuildFrame(frame, motor->motor_id, mode, torque_nm,
                         omega_rad_s, position_rad, kp, kw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    motor->command.mode = mode;
    motor->command.torque_nm = torque_nm;
    motor->command.omega_rad_s = omega_rad_s;
    motor->command.position_rad = position_rad;
    motor->command.kp = kp;
    motor->command.kw = kw;
    return HAL_OK;
}

uint8_t M8010_Motor_GetFeedback(M8010_Motor_t *motor,
                                 M8010_Data *feedback)
{
    uint32_t primask;

    if ((motor == NULL) || (feedback == NULL))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (motor->feedback_updated == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    *feedback = motor->feedback;
    motor->feedback_updated = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}
