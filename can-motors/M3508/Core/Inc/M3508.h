#ifndef M3508_H
#define M3508_H

#include "main.h"

/* 一条CAN总线最多管理8台M3508电机。 */
#define M3508_MOTOR_COUNT 8U

/* C620电调控制电流的协议范围，超出范围时拒绝设置。 */
#define M3508_CURRENT_MIN (-16384)
#define M3508_CURRENT_MAX 16384

/* 0x200控制1~4号电机，0x1FF控制5~8号电机。 */
#define M3508_CONTROL_ID_1_4 0x200U
#define M3508_CONTROL_ID_5_8 0x1FFU

/* 1~8号电机的反馈ID依次为0x201~0x208。 */
#define M3508_FEEDBACK_ID_FIRST 0x201U
#define M3508_FEEDBACK_ID_LAST 0x208U

/* 电机反馈数据：内容来自电调返回的8字节CAN帧。 */
typedef struct
{
    uint16_t angle;       /* 转子机械角度，范围0~8191。 */
    int16_t speed_rpm;    /* 当前转速，单位rpm，正负号表示方向。 */
    int16_t current;      /* 电调反馈的转矩电流原始值。 */
    uint8_t temperature;  /* 电机温度，单位摄氏度。 */
    int32_t total_encoder;/* 从首次反馈开始累计的多圈编码器值。 */
} M3508_FeedbackTypeDef;

/*
 * 单台电机对象。
 * target_current由主循环设置，feedback相关字段由CAN接收中断更新。
 */
typedef struct
{
    uint8_t motor_id;                         /* 逻辑编号1~8。 */
    uint16_t feedback_id;                     /* 对应反馈ID 0x201~0x208。 */
    int16_t target_current;                   /* 下一周期需要发送的目标电流。 */
    volatile M3508_FeedbackTypeDef feedback;  /* 最近一次有效反馈。 */
    volatile uint16_t last_angle;             /* 上一帧角度，用于识别编码器过零。 */
    volatile uint8_t encoder_initialized;     /* 收到首帧反馈后置1。 */
    volatile uint8_t feedback_updated;        /* 新反馈标志：收到置1，读取清0。 */
    volatile uint32_t feedback_count;         /* 累计收到的有效反馈数量。 */
    volatile uint32_t last_feedback_tick;     /* 最近反馈的HAL毫秒时间。 */
} M3508_MotorTypeDef;

/*
 * 电机管理器：绑定一条CAN总线并保存已注册电机。
 * motors[0]~motors[7]固定对应1~8号电机，因此接收时无需遍历查找。
 */
typedef struct
{
    CAN_HandleTypeDef *hcan;                         /* 使用的CAN外设。 */
    M3508_MotorTypeDef *motors[M3508_MOTOR_COUNT];  /* 已注册电机指针表。 */
    uint8_t registered_count;                       /* 当前已注册数量。 */
} M3508_ManagerTypeDef;

/* 初始化管理器，并绑定一个CAN外设。 */
HAL_StatusTypeDef M3508_Manager_Init(M3508_ManagerTypeDef *manager,
                                     CAN_HandleTypeDef *hcan);

/* 注册一台电机，同一个motor_id不能重复注册。 */
HAL_StatusTypeDef M3508_Manager_RegisterMotor(M3508_ManagerTypeDef *manager,
                                              M3508_MotorTypeDef *motor,
                                              uint8_t motor_id);

/* 配置反馈过滤器、启动CAN并开启FIFO0接收中断。 */
HAL_StatusTypeDef M3508_Manager_Start(M3508_ManagerTypeDef *manager);

/* 收集已注册电机的目标电流，并发送0x200和0x1FF两帧。 */
HAL_StatusTypeDef M3508_Manager_SendCurrents(M3508_ManagerTypeDef *manager);

/* 在HAL的FIFO0回调中调用，将反馈分配给对应电机对象。 */
void M3508_Manager_RxFifo0Callback(M3508_ManagerTypeDef *manager,
                                   CAN_HandleTypeDef *hcan);

/* 设置单台电机的目标电流，并检查是否超出协议范围。 */
HAL_StatusTypeDef M3508_Motor_SetCurrent(M3508_MotorTypeDef *motor,
                                         int16_t target_current);

/* 安全复制一份完整反馈；当前没有新反馈时返回0。 */
uint8_t M3508_Motor_GetFeedback(M3508_MotorTypeDef *motor,
                                M3508_FeedbackTypeDef *feedback);

/* 根据最后反馈时间判断电机是否在线。 */
uint8_t M3508_Motor_IsOnline(const M3508_MotorTypeDef *motor,
                             uint32_t now_tick,
                             uint32_t timeout_ms);

#endif /* M3508_H */
