#ifndef AXIS_H
#define AXIS_H

#include "M3508.h"
#include "PID.h"

/*
 * 一个Axis表示由一台M3508驱动的位置轴。
 * 它保存位置-速度串级PID和本轴运行状态，main只需周期调用Axis_Update。
 */
typedef struct
{
    M3508_MotorTypeDef *motor;
    M3508_FeedbackTypeDef feedback;
    PID_TypeDef position_pid;
    PID_TypeDef speed_pid;
    uint32_t last_position_tick;
    int32_t zero_encoder;
    float target_speed_rpm;
    float previous_position_error;
    uint8_t reference_ready;
} Axis_TypeDef;

/* 初始化轴并绑定已经注册的M3508，默认把目标位置设为0度。 */
HAL_StatusTypeDef Axis_Init(Axis_TypeDef *axis, M3508_MotorTypeDef *motor);

/* 设置减速箱输出轴的目标位置，单位为度。 */
void Axis_SetTargetDegree(Axis_TypeDef *axis, float target_degree);

/* 每个速度环周期调用一次，内部完成反馈读取、串级PID和电流设置。 */
void Axis_Update(Axis_TypeDef *axis, uint32_t now_tick);

#endif /* AXIS_H */
