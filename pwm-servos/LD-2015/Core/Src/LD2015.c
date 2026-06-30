#include "LD2015.h"

extern TIM_HandleTypeDef htim1;

HAL_StatusTypeDef LD2015_Start(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, LD2015_MID_PULSE_US);
    return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

HAL_StatusTypeDef LD2015_Stop(void)
{
    return HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

void LD2015_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < LD2015_MIN_PULSE_US)
    {
        pulse_us = LD2015_MIN_PULSE_US;
    }
    else if (pulse_us > LD2015_MAX_PULSE_US)
    {
        pulse_us = LD2015_MAX_PULSE_US;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

void LD2015_SetAngle(uint16_t angle)
{
    uint32_t pulse_us;

    if (angle > LD2015_MAX_ANGLE)
    {
        angle = LD2015_MAX_ANGLE;
    }

    pulse_us = LD2015_MIN_PULSE_US
             + ((uint32_t)angle * (LD2015_MAX_PULSE_US - LD2015_MIN_PULSE_US)
                / LD2015_MAX_ANGLE);
    LD2015_SetPulseUs((uint16_t)pulse_us);
}
