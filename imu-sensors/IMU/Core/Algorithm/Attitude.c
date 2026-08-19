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
#define ATTITUDE_ACC_MIN_G         0.8f
#define ATTITUDE_ACC_MAX_G         1.2f
#define ATTITUDE_INTEGRAL_LIMIT    0.5f
#define ATTITUDE_MAG_MIN_UT        30.0f
#define ATTITUDE_MAG_MAX_UT        70.0f
#define ATTITUDE_MAG_MIN_NORM_SQ   (ATTITUDE_MAG_MIN_UT * ATTITUDE_MAG_MIN_UT)
#define ATTITUDE_MAG_MAX_NORM_SQ   (ATTITUDE_MAG_MAX_UT * ATTITUDE_MAG_MAX_UT)

static Attitude_Data_t ATTITUDE_DATA;

static float Attitude_InvSqrt(float value)
{
    return 1.0f / sqrtf(value);
}

static float Attitude_Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

static void Attitude_UpdateEuler(void)
{
    const float q0 = ATTITUDE_DATA.q0;
    const float q1 = ATTITUDE_DATA.q1;
    const float q2 = ATTITUDE_DATA.q2;
    const float q3 = ATTITUDE_DATA.q3;
    const float pitch_sin = Attitude_Clamp(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f);

    ATTITUDE_DATA.roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                    1.0f - 2.0f * (q1 * q1 + q2 * q2)) * ATTITUDE_DEG_PER_RAD;
    ATTITUDE_DATA.pitch_deg = asinf(pitch_sin) * ATTITUDE_DEG_PER_RAD;
    ATTITUDE_DATA.yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                   1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ATTITUDE_DEG_PER_RAD;
}

void Attitude_Init(void)
{
    memset(&ATTITUDE_DATA, 0, sizeof(ATTITUDE_DATA));
    ATTITUDE_DATA.q0 = 1.0f;
}

uint8_t Attitude_CollectStartupAverage(Attitude_StartupAverage_t *average)
{
    BMI088_Data_t imu;
    IST8310_Data_t mag;
    float acc_sum[3] = {0.0f};
    float mag_sum[3] = {0.0f};
    uint16_t acc_count = 0u;
    uint16_t mag_count = 0u;

    if (average == NULL)
    {
        return 0u;
    }

    while ((acc_count < 500u) || (mag_count < 50u))
    {
        if (BMI088_IsSampleReady() != 0u)
        {
            BMI088_ProcessSample();
            BMI088_GetData(&imu);

            if (acc_count < 500u)
            {
                acc_sum[0] += imu.acc_g[0];
                acc_sum[1] += imu.acc_g[1];
                acc_sum[2] += imu.acc_g[2];
                acc_count++;
            }
        }

        if (IST8310_Update() != 0u)
        {
            if (IST8310_GetValidData(&mag) != 0u && mag_count < 50u)
            {
                mag_sum[0] += mag.mag_uT[0];
                mag_sum[1] += mag.mag_uT[1];
                mag_sum[2] += mag.mag_uT[2];
                mag_count++;
            }
        }
    }

    average->acc_g[0] = acc_sum[0] / (float)acc_count;
    average->acc_g[1] = acc_sum[1] / (float)acc_count;
    average->acc_g[2] = acc_sum[2] / (float)acc_count;
    average->mag_uT[0] = mag_sum[0] / (float)mag_count;
    average->mag_uT[1] = mag_sum[1] / (float)mag_count;
    average->mag_uT[2] = mag_sum[2] / (float)mag_count;

    return 1u;
}

uint8_t Attitude_InitializeQuaternion(const Attitude_StartupAverage_t *average)
{
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
    float acc_norm;
    float mag_norm_sq;
    float roll;
    float pitch;
    float yaw;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float mag_x_horizontal;
    float mag_y_horizontal;
    float half_roll;
    float half_pitch;
    float half_yaw;
    float cos_half_roll;
    float sin_half_roll;
    float cos_half_pitch;
    float sin_half_pitch;
    float cos_half_yaw;
    float sin_half_yaw;

    if (average == NULL)
    {
        return 0u;
    }

    ax = average->acc_g[0];
    ay = average->acc_g[1];
    az = average->acc_g[2];
    acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm <= ATTITUDE_ACC_MIN_G || acc_norm >= ATTITUDE_ACC_MAX_G)
    {
        return 0u;
    }

    mx = average->mag_uT[0];
    my = average->mag_uT[1];
    mz = average->mag_uT[2];
    mag_norm_sq = mx * mx + my * my + mz * mz;
    if (mag_norm_sq <= ATTITUDE_MAG_MIN_NORM_SQ ||
        mag_norm_sq >= ATTITUDE_MAG_MAX_NORM_SQ)
    {
        return 0u;
    }

    ax /= acc_norm;
    ay /= acc_norm;
    az /= acc_norm;

    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    sin_roll = sinf(roll);
    cos_roll = cosf(roll);
    sin_pitch = sinf(pitch);
    cos_pitch = cosf(pitch);

    mag_x_horizontal = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    mag_y_horizontal = my * cos_roll - mz * sin_roll;
    yaw = atan2f(-mag_y_horizontal, mag_x_horizontal);

    half_roll = 0.5f * roll;
    half_pitch = 0.5f * pitch;
    half_yaw = 0.5f * yaw;
    cos_half_roll = cosf(half_roll);
    sin_half_roll = sinf(half_roll);
    cos_half_pitch = cosf(half_pitch);
    sin_half_pitch = sinf(half_pitch);
    cos_half_yaw = cosf(half_yaw);
    sin_half_yaw = sinf(half_yaw);

    Attitude_Init();
    ATTITUDE_DATA.q0 = cos_half_roll * cos_half_pitch * cos_half_yaw +
                       sin_half_roll * sin_half_pitch * sin_half_yaw;
    ATTITUDE_DATA.q1 = sin_half_roll * cos_half_pitch * cos_half_yaw -
                       cos_half_roll * sin_half_pitch * sin_half_yaw;
    ATTITUDE_DATA.q2 = cos_half_roll * sin_half_pitch * cos_half_yaw +
                       sin_half_roll * cos_half_pitch * sin_half_yaw;
    ATTITUDE_DATA.q3 = cos_half_roll * cos_half_pitch * sin_half_yaw -
                       sin_half_roll * sin_half_pitch * cos_half_yaw;
    Attitude_UpdateEuler();

    return 1u;
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
    float acc_norm;

    if (imu == NULL || dt_s <= 0.0f)
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
    norm_sq = ax * ax + ay * ay + az * az;
    acc_norm = sqrtf(norm_sq);

    if (acc_norm > ATTITUDE_ACC_MIN_G && acc_norm < ATTITUDE_ACC_MAX_G)
    {
        const float inv_norm = 1.0f / acc_norm;
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

        if (mag != NULL)
        {
            mx = mag->mag_uT[0];
            my = mag->mag_uT[1];
            mz = mag->mag_uT[2];

            norm_sq = mx * mx + my * my + mz * mz;
            if (norm_sq > ATTITUDE_MAG_MIN_NORM_SQ &&
                norm_sq < ATTITUDE_MAG_MAX_NORM_SQ)
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

                /*计算预测的磁力计方向*/
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
        }
        /*误差积分*/
        ATTITUDE_DATA.integral_error[0] += ATTITUDE_KI * ex * dt_s;
        ATTITUDE_DATA.integral_error[1] += ATTITUDE_KI * ey * dt_s;
        ATTITUDE_DATA.integral_error[2] += ATTITUDE_KI * ez * dt_s;
        ATTITUDE_DATA.integral_error[0] = Attitude_Clamp(ATTITUDE_DATA.integral_error[0],
                                                         -ATTITUDE_INTEGRAL_LIMIT,
                                                         ATTITUDE_INTEGRAL_LIMIT);
        ATTITUDE_DATA.integral_error[1] = Attitude_Clamp(ATTITUDE_DATA.integral_error[1],
                                                         -ATTITUDE_INTEGRAL_LIMIT,
                                                         ATTITUDE_INTEGRAL_LIMIT);
        ATTITUDE_DATA.integral_error[2] = Attitude_Clamp(ATTITUDE_DATA.integral_error[2],
                                                         -ATTITUDE_INTEGRAL_LIMIT,
                                                         ATTITUDE_INTEGRAL_LIMIT);

        /*修正角速度*/
        gx += ATTITUDE_KP * ex + ATTITUDE_DATA.integral_error[0];
        gy += ATTITUDE_KP * ey + ATTITUDE_DATA.integral_error[1];
        gz += ATTITUDE_KP * ez + ATTITUDE_DATA.integral_error[2];
    }

    ATTITUDE_DATA.gyro_corrected_dps[0] = gx * ATTITUDE_DEG_PER_RAD;
    ATTITUDE_DATA.gyro_corrected_dps[1] = gy * ATTITUDE_DEG_PER_RAD;
    ATTITUDE_DATA.gyro_corrected_dps[2] = gz * ATTITUDE_DEG_PER_RAD;

    /*更新四元数*/
    half_dt = 0.5f * dt_s;
    gx *= half_dt;
    gy *= half_dt;
    gz *= half_dt;

    ATTITUDE_DATA.q0 += -q1 * gx - q2 * gy - q3 * gz;
    ATTITUDE_DATA.q1 += q0 * gx + q2 * gz - q3 * gy;
    ATTITUDE_DATA.q2 += q0 * gy - q1 * gz + q3 * gx;
    ATTITUDE_DATA.q3 += q0 * gz + q1 * gy - q2 * gx;

    /*归一化四元数值*/
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
