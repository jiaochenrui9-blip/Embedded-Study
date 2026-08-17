//
// Created by game on 2026/8/17.
//

#include "Attitude.h"

#include <math.h>
#include <string.h>

#define ATTITUDE_RAD_PER_DEG       0.01745329251994329577f
#define ATTITUDE_DEG_PER_RAD       57.295779513082320876f
#define ATTITUDE_KP                2.0f
#define ATTITUDE_KI                0.02f
#define ATTITUDE_VECTOR_MIN_NORM   0.000001f

static Attitude_Data_t ATTITUDE_DATA;

static float Attitude_InvSqrt(float value)
{
    return 1.0f / sqrtf(value);
}

static void Attitude_UpdateEuler(void)
{
    const float q0 = ATTITUDE_DATA.q0;
    const float q1 = ATTITUDE_DATA.q1;
    const float q2 = ATTITUDE_DATA.q2;
    const float q3 = ATTITUDE_DATA.q3;

    ATTITUDE_DATA.roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                    1.0f - 2.0f * (q1 * q1 + q2 * q2)) * ATTITUDE_DEG_PER_RAD;
    ATTITUDE_DATA.pitch_deg = asinf(2.0f * (q0 * q2 - q3 * q1)) * ATTITUDE_DEG_PER_RAD;
    ATTITUDE_DATA.yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                   1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ATTITUDE_DEG_PER_RAD;
}

void Attitude_Init(void)
{
    memset(&ATTITUDE_DATA, 0, sizeof(ATTITUDE_DATA));
    ATTITUDE_DATA.q0 = 1.0f;
}

void Attitude_Update(const BMI088_Data_t *imu, const IST8310_Data_t *mag, float dt_s)
{
    float q0;
    float q1;
    float q2;
    float q3;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
    float norm_sq;
    float vx;
    float vy;
    float vz;
    float wx;
    float wy;
    float wz;
    float hx;
    float hy;
    float bx;
    float bz;
    float ex;
    float ey;
    float ez;
    float half_dt;

    if (imu == NULL || mag == NULL || dt_s <= 0.0f)
    {
        return;
    }

    q0 = ATTITUDE_DATA.q0;
    q1 = ATTITUDE_DATA.q1;
    q2 = ATTITUDE_DATA.q2;
    q3 = ATTITUDE_DATA.q3;

    gx = imu->gyro_dps[0] * ATTITUDE_RAD_PER_DEG;
    gy = imu->gyro_dps[1] * ATTITUDE_RAD_PER_DEG;
    gz = imu->gyro_dps[2] * ATTITUDE_RAD_PER_DEG;

    ax = imu->acc_g[0];
    ay = imu->acc_g[1];
    az = imu->acc_g[2];
    mx = mag->mag_uT[0];
    my = mag->mag_uT[1];
    mz = mag->mag_uT[2];

    norm_sq = ax * ax + ay * ay + az * az;
    if (norm_sq > ATTITUDE_VECTOR_MIN_NORM)
    {
        const float inv_norm = Attitude_InvSqrt(norm_sq);
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;

        /* 用四元数预测重力方向 */
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* e * v，计算误差 */
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        norm_sq = mx * mx + my * my + mz * mz;
        if (norm_sq > ATTITUDE_VECTOR_MIN_NORM)
        {
            const float inv_norm = Attitude_InvSqrt(norm_sq);
            mx *= inv_norm;
            my *= inv_norm;
            mz *= inv_norm;

            /* 计算水平分量和竖直分量 */
            hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3)
               + 2.0f * my * (q1 * q2 - q0 * q3)
               + 2.0f * mz * (q1 * q3 + q0 * q2);
            hy = 2.0f * mx * (q1 * q2 + q0 * q3)
               + 2.0f * my * (0.5f - q1 * q1 - q3 * q3)
               + 2.0f * mz * (q2 * q3 - q0 * q1);
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * mx * (q1 * q3 - q0 * q2)
               + 2.0f * my * (q2 * q3 + q0 * q1)
               + 2.0f * mz * (0.5f - q1 * q1 - q2 * q2);

            wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3)
               + 2.0f * bz * (q1 * q3 - q0 * q2);
            wy = 2.0f * bx * (q1 * q2 - q0 * q3)
               + 2.0f * bz * (q0 * q1 + q2 * q3);
            wz = 2.0f * bx * (q0 * q2 + q1 * q3)
               + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2);

            /* 计算误差*/
            ex += my * wz - mz * wy;
            ey += mz * wx - mx * wz;
            ez += mx * wy - my * wx;
        }

        ATTITUDE_DATA.integral_error[0] += ATTITUDE_KI * ex * dt_s;
        ATTITUDE_DATA.integral_error[1] += ATTITUDE_KI * ey * dt_s;
        ATTITUDE_DATA.integral_error[2] += ATTITUDE_KI * ez * dt_s;

        gx += ATTITUDE_KP * ex + ATTITUDE_DATA.integral_error[0];
        gy += ATTITUDE_KP * ey + ATTITUDE_DATA.integral_error[1];
        gz += ATTITUDE_KP * ez + ATTITUDE_DATA.integral_error[2];
    }

    ATTITUDE_DATA.gyro_corrected_dps[0] = gx * ATTITUDE_DEG_PER_RAD;
    ATTITUDE_DATA.gyro_corrected_dps[1] = gy * ATTITUDE_DEG_PER_RAD;
    ATTITUDE_DATA.gyro_corrected_dps[2] = gz * ATTITUDE_DEG_PER_RAD;

    half_dt = 0.5f * dt_s;
    gx *= half_dt;
    gy *= half_dt;
    gz *= half_dt;

    ATTITUDE_DATA.q0 += -q1 * gx - q2 * gy - q3 * gz;
    ATTITUDE_DATA.q1 += q0 * gx + q2 * gz - q3 * gy;
    ATTITUDE_DATA.q2 += q0 * gy - q1 * gz + q3 * gx;
    ATTITUDE_DATA.q3 += q0 * gz + q1 * gy - q2 * gx;

    norm_sq = ATTITUDE_DATA.q0 * ATTITUDE_DATA.q0 + ATTITUDE_DATA.q1 * ATTITUDE_DATA.q1
            + ATTITUDE_DATA.q2 * ATTITUDE_DATA.q2 + ATTITUDE_DATA.q3 * ATTITUDE_DATA.q3;
    if (norm_sq > ATTITUDE_VECTOR_MIN_NORM)
    {
        const float inv_norm = Attitude_InvSqrt(norm_sq);
        ATTITUDE_DATA.q0 *= inv_norm;
        ATTITUDE_DATA.q1 *= inv_norm;
        ATTITUDE_DATA.q2 *= inv_norm;
        ATTITUDE_DATA.q3 *= inv_norm;
    }
    else
    {
        Attitude_Init();
    }

    Attitude_UpdateEuler();
}

void Attitude_GetData(Attitude_Data_t *data)
{
    if (data != NULL)
    {
        *data = ATTITUDE_DATA;
    }
}
