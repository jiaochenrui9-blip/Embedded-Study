//
// Created by game on 2026/8/17.
//

#ifndef BMI088_ATTITUDE_H
#define BMI088_ATTITUDE_H

#include "BMI088.h"
#include "IST8310.h"

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float gyro_corrected_dps[3];
    float integral_error[3];
} Attitude_Data_t;

typedef struct
{
    float acc_g[3];
    float mag_uT[3];
} Attitude_StartupAverage_t;

/* The BMI088 and IST8310 axes must both use the same body coordinate system. */
void Attitude_Init(void);
uint8_t Attitude_CollectStartupAverage(Attitude_StartupAverage_t *average);
uint8_t Attitude_InitializeQuaternion(const Attitude_StartupAverage_t *average);
void Attitude_Update(const BMI088_Data_t *imu, const IST8310_Data_t *mag, float dt_s);
void Attitude_GetData(Attitude_Data_t *data);

#endif //BMI088_ATTITUDE_H
