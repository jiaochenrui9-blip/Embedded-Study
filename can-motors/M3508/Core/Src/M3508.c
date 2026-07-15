#include "M3508.h"
#include "can_bus.h"

/* M3508协议中的16位数据均按高字节在前的顺序传输。 */
static uint16_t M3508_ReadU16(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

static int16_t M3508_ReadS16(const uint8_t *data)
{
    return (int16_t)M3508_ReadU16(data);
}

/* 把4台电机的int16_t目标电流打包为一个8字节CAN数据区。 */
static void M3508_WriteCurrentData(const int16_t current[4], uint8_t data[8])
{
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        const uint16_t raw = (uint16_t)current[i];
        data[2U * i] = (uint8_t)(raw >> 8);
        data[2U * i + 1U] = (uint8_t)raw;
    }
}

HAL_StatusTypeDef M3508_Manager_Init(M3508_ManagerTypeDef *manager,
                                     CAN_HandleTypeDef *hcan)
{
    if ((manager == NULL) || (hcan == NULL))
    {
        return HAL_ERROR;
    }

    /* 先清空所有注册槽位，再绑定当前管理器使用的CAN外设。 */
    *manager = (M3508_ManagerTypeDef){0};
    manager->hcan = hcan;
    return HAL_OK;
}

HAL_StatusTypeDef M3508_Manager_RegisterMotor(M3508_ManagerTypeDef *manager,
                                              M3508_MotorTypeDef *motor,
                                              uint8_t motor_id)
{
    uint8_t index;

    if ((manager == NULL) || (manager->hcan == NULL) || (motor == NULL) ||
        (motor_id < 1U) || (motor_id > M3508_MOTOR_COUNT))
    {
        return HAL_ERROR;
    }

    /* 1号电机放入motors[0]，8号电机放入motors[7]。 */
    index = motor_id - 1U;

    /* 一个编号只能对应一个电机对象，防止重复注册覆盖原对象。 */
    if (manager->motors[index] != NULL)
    {
        return HAL_ERROR;
    }

    /* 注册时清空旧状态，并建立逻辑编号与反馈ID的固定映射。 */
    *motor = (M3508_MotorTypeDef){0};
    motor->motor_id = motor_id;
    motor->feedback_id = M3508_FEEDBACK_ID_FIRST + index;
    manager->motors[index] = motor;
    manager->registered_count++;
    return HAL_OK;
}

HAL_StatusTypeDef M3508_Manager_Start(M3508_ManagerTypeDef *manager)
{
    /* 两个16位列表过滤器，每个过滤器刚好接收4个电机反馈ID。 */
    static const uint16_t feedback_ids_1_4[4] = {0x201U, 0x202U, 0x203U, 0x204U};
    static const uint16_t feedback_ids_5_8[4] = {0x205U, 0x206U, 0x207U, 0x208U};
    HAL_StatusTypeDef status;

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return HAL_ERROR;
    }

    /* 过滤器0接收1~4号，过滤器1接收5~8号，全部进入FIFO0。 */
    status = CAN_Bus_ConfigStdIdList(manager->hcan, 0U, 14U, feedback_ids_1_4);
    if (status != HAL_OK)
    {
        return status;
    }

    status = CAN_Bus_ConfigStdIdList(manager->hcan, 1U, 14U, feedback_ids_5_8);
    if (status != HAL_OK)
    {
        return status;
    }

    return CAN_Bus_StartRxFifo0(manager->hcan);
}

HAL_StatusTypeDef M3508_Motor_SetCurrent(M3508_MotorTypeDef *motor,
                                         int16_t target_current)
{
    /* motor_id为0表示这个对象尚未成功注册。 */
    if ((motor == NULL) || (motor->motor_id == 0U) ||
        (target_current < M3508_CURRENT_MIN) ||
        (target_current > M3508_CURRENT_MAX))
    {
        return HAL_ERROR;
    }

    motor->target_current = target_current;
    return HAL_OK;
}

HAL_StatusTypeDef M3508_Manager_SendCurrents(M3508_ManagerTypeDef *manager)
{
    HAL_StatusTypeDef status;
    int16_t current_1_4[4] = {0};
    int16_t current_5_8[4] = {0};
    uint8_t tx_data_1_4[8];
    uint8_t tx_data_5_8[8];

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return HAL_ERROR;
    }

    /* 本次需要连续发送0x200和0x1FF，因此至少保留两个发送邮箱。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(manager->hcan) < 2U)
    {
        return HAL_BUSY;
    }

    /*
     * 从注册表收集目标电流：
     * motors[0]~[3]写入0x200，motors[4]~[7]写入0x1FF。
     * 未注册的槽位保持0电流。
     */
    for (uint8_t index = 0U; index < M3508_MOTOR_COUNT; ++index)
    {
        M3508_MotorTypeDef *motor = manager->motors[index];
        if (motor == NULL)
        {
            continue;
        }

        if (index < 4U)
        {
            current_1_4[index] = motor->target_current;
        }
        else
        {
            current_5_8[index - 4U] = motor->target_current;
        }
    }

    /* 转换为M3508协议要求的高字节在前格式。 */
    M3508_WriteCurrentData(current_1_4, tx_data_1_4);
    M3508_WriteCurrentData(current_5_8, tx_data_5_8);

    status = CAN_Bus_SendStdData(manager->hcan,
                                 M3508_CONTROL_ID_1_4,
                                 tx_data_1_4,
                                 8U);
    if (status != HAL_OK)
    {
        return status;
    }

    return CAN_Bus_SendStdData(manager->hcan,
                               M3508_CONTROL_ID_5_8,
                               tx_data_5_8,
                               8U);
}

void M3508_Manager_RxFifo0Callback(M3508_ManagerTypeDef *manager,
                                   CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    M3508_MotorTypeDef *motor;
    uint8_t data[8];
    uint8_t index;
    uint16_t angle;
    int32_t encoder_delta;

    /* 只处理属于当前管理器所绑定CAN外设的消息。 */
    if ((manager == NULL) || (hcan == NULL) || (hcan != manager->hcan))
    {
        return;
    }

    if (CAN_Bus_ReadFifo0(hcan, &header, data) != HAL_OK)
    {
        return;
    }

    /* 只接受DLC为8、ID在0x201~0x208范围内的标准数据帧。 */
    if ((header.IDE != CAN_ID_STD) ||
        (header.RTR != CAN_RTR_DATA) ||
        (header.DLC != 8U) ||
        (header.StdId < M3508_FEEDBACK_ID_FIRST) ||
        (header.StdId > M3508_FEEDBACK_ID_LAST))
    {
        return;
    }

    /* 由反馈ID直接算出注册槽位，无需遍历全部电机。 */
    index = (uint8_t)(header.StdId - M3508_FEEDBACK_ID_FIRST);
    motor = manager->motors[index];
    if (motor == NULL)
    {
        return;
    }

    /* 按M3508反馈协议解析角度、转速、电流和温度。 */
    angle = M3508_ReadU16(&data[0]);

    /*
     * 编码器一圈为8192。跨过0点时，原始角度会在0和8191之间跳变；
     * 将该跳变修正为连续增量，得到从首帧开始累计的多圈编码器值。
     */
    if (motor->encoder_initialized == 0U)
    {
        motor->last_angle = angle;
        motor->feedback.total_encoder = 0;
        motor->encoder_initialized = 1U;
    }
    else
    {
        encoder_delta = (int32_t)angle - (int32_t)motor->last_angle;
        if (encoder_delta > 4096)
        {
            encoder_delta -= 8192;
        }
        else if (encoder_delta < -4096)
        {
            encoder_delta += 8192;
        }

        motor->feedback.total_encoder += encoder_delta;
        motor->last_angle = angle;
    }

    motor->feedback.angle = angle;
    motor->feedback.speed_rpm = M3508_ReadS16(&data[2]);
    motor->feedback.current = M3508_ReadS16(&data[4]);
    motor->feedback.temperature = data[6];
    motor->last_feedback_tick = HAL_GetTick();
    motor->feedback_count++;
    motor->feedback_updated = 1U;
}

uint8_t M3508_Motor_GetFeedback(M3508_MotorTypeDef *motor,
                                M3508_FeedbackTypeDef *feedback)
{
    uint32_t primask;

    if ((motor == NULL) || (feedback == NULL))
    {
        return 0U;
    }

    /*
     * 反馈由中断更新，复制期间短暂关闭中断，避免读到一半新帧到来。
     * 保存PRIMASK是为了不破坏调用前已有的中断状态。
     */
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

    feedback->angle = motor->feedback.angle;
    feedback->speed_rpm = motor->feedback.speed_rpm;
    feedback->current = motor->feedback.current;
    feedback->temperature = motor->feedback.temperature;
    feedback->total_encoder = motor->feedback.total_encoder;
    motor->feedback_updated = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

uint8_t M3508_Motor_IsOnline(const M3508_MotorTypeDef *motor,
                             uint32_t now_tick,
                             uint32_t timeout_ms)
{
    /* 从未收到反馈的电机直接判定为离线。 */
    if ((motor == NULL) || (motor->feedback_count == 0U))
    {
        return 0U;
    }

    return ((now_tick - motor->last_feedback_tick) <= timeout_ms) ? 1U : 0U;
}
