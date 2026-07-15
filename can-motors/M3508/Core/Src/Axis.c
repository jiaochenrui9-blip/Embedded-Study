#include "Axis.h"

/* 当前1号M3508位置轴的实测参数。 */
#define AXIS_POSITION_PERIOD_MS          5U
#define AXIS_OFFLINE_TIMEOUT_MS        100U

#define AXIS_REDUCTION_RATIO        (3591.0f / 187.0f)
#define AXIS_ENCODER_COUNTS_PER_REV 8192.0f

#define AXIS_POSITION_KP              15.0f
#define AXIS_POSITION_KI               0.0f
#define AXIS_POSITION_KD               0.0f
#define AXIS_POSITION_SPEED_LIMIT     300.0f
#define AXIS_POSITION_DEADBAND_DEG      0.2f

#define AXIS_SPEED_KP                   3.0f
#define AXIS_SPEED_KI                   0.02f
#define AXIS_SPEED_KD                   0.0f
#define AXIS_CURRENT_LIMIT           1500.0f
#define AXIS_SPEED_INTEGRAL_LIMIT   30000.0f
#define AXIS_POSITION_INTEGRAL_LIMIT 50000.0f

#define AXIS_FRICTION_CURRENT             180
#define AXIS_FRICTION_MAX_SPEED_RPM        20

static float Axis_EncoderToOutputDegree(int32_t encoder_delta)
{
    return ((float)encoder_delta * 360.0f) /
           (AXIS_ENCODER_COUNTS_PER_REV * AXIS_REDUCTION_RATIO);
}

/* 离线时清除控制状态和电流，但保留本次上电建立的位置零点。 */
static void Axis_Stop(Axis_TypeDef *axis)
{
    PID_Reset(&axis->speed_pid);
    PID_Reset(&axis->position_pid);
    axis->target_speed_rpm = 0.0f;
    axis->previous_position_error = 0.0f;
    (void)M3508_Motor_SetCurrent(axis->motor, 0);
}

HAL_StatusTypeDef Axis_Init(Axis_TypeDef *axis, M3508_MotorTypeDef *motor)
{
    if ((axis == NULL) || (motor == NULL) || (motor->motor_id == 0U))
    {
        return HAL_ERROR;
    }

    *axis = (Axis_TypeDef){0};
    axis->motor = motor;

    PID_Init(&axis->speed_pid,
             AXIS_SPEED_KP,
             AXIS_SPEED_KI,
             AXIS_SPEED_KD,
             -AXIS_CURRENT_LIMIT,
             AXIS_CURRENT_LIMIT,
             -AXIS_SPEED_INTEGRAL_LIMIT,
             AXIS_SPEED_INTEGRAL_LIMIT);

    PID_Init(&axis->position_pid,
             AXIS_POSITION_KP,
             AXIS_POSITION_KI,
             AXIS_POSITION_KD,
             -AXIS_POSITION_SPEED_LIMIT,
             AXIS_POSITION_SPEED_LIMIT,
             -AXIS_POSITION_INTEGRAL_LIMIT,
             AXIS_POSITION_INTEGRAL_LIMIT);

    PID_SetTarget(&axis->position_pid, 0.0f);
    PID_SetTarget(&axis->speed_pid, 0.0f);
    return M3508_Motor_SetCurrent(motor, 0);
}

void Axis_SetTargetDegree(Axis_TypeDef *axis, float target_degree)
{
    if (axis != NULL)
    {
        PID_SetTarget(&axis->position_pid, target_degree);
    }
}

void Axis_Update(Axis_TypeDef *axis, uint32_t now_tick)
{
    float position_degree;
    float position_error;
    int16_t current_command;

    if ((axis == NULL) || (axis->motor == NULL))
    {
        return;
    }

    if (M3508_Motor_IsOnline(axis->motor, now_tick,
                             AXIS_OFFLINE_TIMEOUT_MS) == 0U)
    {
        Axis_Stop(axis);
        return;
    }

    /* 没有新反馈时保持上一次电流，等待下一帧再计算。 */
    if (M3508_Motor_GetFeedback(axis->motor, &axis->feedback) == 0U)
    {
        return;
    }

    /* 首帧位置作为本次上电的输出轴零点。 */
    if (axis->reference_ready == 0U)
    {
        axis->zero_encoder = axis->feedback.total_encoder;
        axis->reference_ready = 1U;
        axis->last_position_tick = now_tick;
        Axis_Stop(axis);
    }

    if ((now_tick - axis->last_position_tick) >= AXIS_POSITION_PERIOD_MS)
    {
        position_degree = Axis_EncoderToOutputDegree(
            axis->feedback.total_encoder - axis->zero_encoder);
        position_error = axis->position_pid.target - position_degree;

        /* 越过目标位置时清除旧方向的速度积分，防止继续冲过头。 */
        if (((position_error > 0.0f) &&
             (axis->previous_position_error < 0.0f)) ||
            ((position_error < 0.0f) &&
             (axis->previous_position_error > 0.0f)))
        {
            axis->speed_pid.integral = 0.0f;
        }
        axis->previous_position_error = position_error;

        if ((position_error <= AXIS_POSITION_DEADBAND_DEG) &&
            (position_error >= -AXIS_POSITION_DEADBAND_DEG))
        {
            axis->target_speed_rpm = 0.0f;
            PID_Reset(&axis->position_pid);
            axis->speed_pid.integral = 0.0f;
        }
        else
        {
            axis->target_speed_rpm = PID_Calculate(&axis->position_pid,
                                                   position_degree);
        }

        PID_SetTarget(&axis->speed_pid, axis->target_speed_rpm);
        axis->last_position_tick = now_tick;
    }

    /* 速度环输出C620电流，并在低速回位时补偿减速箱静摩擦。 */
    current_command = (int16_t)PID_Calculate(
        &axis->speed_pid, (float)axis->feedback.speed_rpm);

    if ((axis->feedback.speed_rpm <= AXIS_FRICTION_MAX_SPEED_RPM) &&
        (axis->feedback.speed_rpm >= -AXIS_FRICTION_MAX_SPEED_RPM))
    {
        if ((axis->target_speed_rpm > 0.0f) && (current_command > 0))
        {
            current_command += AXIS_FRICTION_CURRENT;
        }
        else if ((axis->target_speed_rpm < 0.0f) &&
                 (current_command < 0))
        {
            current_command -= AXIS_FRICTION_CURRENT;
        }
    }

    if (current_command > (int16_t)AXIS_CURRENT_LIMIT)
    {
        current_command = (int16_t)AXIS_CURRENT_LIMIT;
    }
    else if (current_command < (int16_t)(-AXIS_CURRENT_LIMIT))
    {
        current_command = (int16_t)(-AXIS_CURRENT_LIMIT);
    }

    (void)M3508_Motor_SetCurrent(axis->motor, current_command);
}
