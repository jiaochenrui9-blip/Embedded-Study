#ifndef UNITREE_M8010_M8010_H
#define UNITREE_M8010_M8010_H

#include "../../Inc/main.h"

#define M8010_FRAME_SIZE          17U
#define M8010_RESPONSE_SIZE       16U

typedef enum
{
    M8010_MODE_LOCKED = 0U,
    M8010_MODE_FOC = 1U,
    M8010_MODE_CALIBRATE = 2U
} M8010_Mode;


typedef struct
{
    uint8_t status;
    float torque_nm;
    float omega_rad_s;
    float position_rad;
    int8_t temperature_c;
    uint8_t error_code;
    uint16_t force_raw;
} M8010_Data;

HAL_StatusTypeDef M8010_BuildFrame(uint8_t frame[M8010_FRAME_SIZE],
                                   uint8_t motor_id,
                                   M8010_Mode mode,
                                   float torque_nm,
                                   float omega_rad_s,
                                   float position_rad,
                                   float kp, 
                                   float kw);

HAL_StatusTypeDef M8010_ParseFrame(const uint8_t frame[M8010_RESPONSE_SIZE],
                                   uint8_t *motor_id,
                                   M8010_Data *data);

#endif /* UNITREE_M8010_M8010_H */
