//
// Created by game on 2026/7/16.
//
#include "app_RS05.h"
#include <string.h>

#define RS05_POSITION_MIN_RAD       (-12.57f)
#define RS05_POSITION_MAX_RAD       (12.57f)
#define RS05_VELOCITY_MIN_RAD_S     (-50.0f)
#define RS05_VELOCITY_MAX_RAD_S     (50.0f)
#define RS05_KP_MIN                 (0.0f)
#define RS05_KP_MAX                 (500.0f)
#define RS05_KD_MIN                 (0.0f)
#define RS05_KD_MAX                 (5.0f)
#define RS05_TORQUE_MIN_NM          (-5.5f)
#define RS05_TORQUE_MAX_NM          (5.5f)

#define RS05_COMM_TYPE_MOTION_CONTROL      0x01U
#define RS05_COMM_TYPE_FEEDBACK            0x02U
#define RS05_COMM_TYPE_ENABLE              0x03U
#define RS05_COMM_TYPE_STOP                0x04U
#define RS05_COMM_TYPE_READ_PARAMETER      0x11U
#define RS05_COMM_TYPE_WRITE_PARAMETER     0x12U

#define RS05_PARAMETER_RUN_MODE            0x7005U
#define RS05_RUN_MODE_MOTION                0.0f

static RS05_MotorTypedef *RS05_Manager_FindMotor(const RS05_ManagerTypedef *manager,
                                                  uint8_t motor_id)
{
    uint8_t i;

    if (manager == NULL)
    {
        return NULL;
    }

    for (i = 0U; i < manager->num_motors; i++)
    {
        if ((manager->motors[i] != NULL) &&
            (manager->motors[i]->motor_id == motor_id))
        {
            return manager->motors[i];
        }
    }

    return NULL;
}

static void RS05_WriteUint16BE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static void RS05_WriteUint16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void RS05_WriteFloatLE(uint8_t *data, float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8);
    data[2] = (uint8_t)(raw >> 16);
    data[3] = (uint8_t)(raw >> 24);
}

static uint16_t RS05_ReadUint16BE(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

static uint16_t RS05_ReadUint16LE(const uint8_t *data)
{
    return ((uint16_t)data[1] << 8) | data[0];
}

static uint16_t RS05_FloatToUint16(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        value = minimum;
    }
    else if (value > maximum)
    {
        value = maximum;
    }

    return (uint16_t)((value - minimum) * 65535.0f / (maximum - minimum));
}

static float RS05_Uint16ToFloat(uint16_t value, float minimum, float maximum)
{
    return ((float)value * (maximum - minimum) / 65535.0f) + minimum;
}

static float RS05_ReadFloatLE(const uint8_t *data)
{
    uint32_t raw = ((uint32_t)data[0]) |
                   ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void RS05_ParseType2Feedback(RS05_MotorTypedef *motor,
                                    uint32_t ext_id,
                                    const uint8_t *data)
{
    motor->position_rad = RS05_Uint16ToFloat(RS05_ReadUint16BE(&data[0]),
                                              RS05_POSITION_MIN_RAD,
                                              RS05_POSITION_MAX_RAD);
    motor->velocity_rad_s = RS05_Uint16ToFloat(RS05_ReadUint16BE(&data[2]),
                                                RS05_VELOCITY_MIN_RAD_S,
                                                RS05_VELOCITY_MAX_RAD_S);
    motor->torque_nm = RS05_Uint16ToFloat(RS05_ReadUint16BE(&data[4]),
                                           RS05_TORQUE_MIN_NM,
                                           RS05_TORQUE_MAX_NM);
    motor->temperature_c = (float)RS05_ReadUint16BE(&data[6]) * 0.1f;
    motor->state = (uint8_t)((ext_id >> 22) & 0x03U);
    motor->fault = (uint8_t)((ext_id >> 16) & 0x3FU);
    motor->last_feedback_tick = HAL_GetTick();
}

static void RS05_ParseType17ParameterReply(RS05_MotorTypedef *motor,
                                            const uint8_t *data)
{
    motor->last_parameter_index = RS05_ReadUint16LE(&data[0]);
    motor->last_parameter_value = RS05_ReadFloatLE(&data[4]);
    motor->parameter_reply_valid = 1U;
}

uint32_t RS05_BuildExtId(uint8_t comm_type,
                         uint16_t data2,
                         uint8_t motor_id)
{
    return ((uint32_t)(comm_type & 0x1FU) << 24) |
           ((uint32_t)data2 << 8) |
           motor_id;
}

HAL_StatusTypeDef RS05_SendData(CAN_HandleTypeDef *hcan,
                                uint8_t comm_type,
                                const uint8_t *data,
                                uint16_t data2,
                                uint8_t motor_id)
{
    return CAN_SendExtFrame(hcan,
                            RS05_BuildExtId(comm_type, data2, motor_id),
                            data,
                            8U);
}

HAL_StatusTypeDef RS05_Enable(CAN_HandleTypeDef *hcan,
                              uint8_t master_id,
                              uint8_t motor_id)
{
    static const uint8_t data[8] = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    return RS05_SendData(hcan, RS05_COMM_TYPE_ENABLE, data, master_id, motor_id);
}

HAL_StatusTypeDef RS05_MotionControl(CAN_HandleTypeDef *hcan,
                                     uint8_t motor_id,
                                     float position_rad,
                                     float velocity_rad_s,
                                     float kp,
                                     float kd,
                                     float torque_nm)
{
    uint8_t data[8] = {0};
    uint16_t torque_raw;

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    RS05_WriteUint16BE(&data[0], RS05_FloatToUint16(position_rad,
                                                     RS05_POSITION_MIN_RAD,
                                                     RS05_POSITION_MAX_RAD));
    RS05_WriteUint16BE(&data[2], RS05_FloatToUint16(velocity_rad_s,
                                                     RS05_VELOCITY_MIN_RAD_S,
                                                     RS05_VELOCITY_MAX_RAD_S));
    RS05_WriteUint16BE(&data[4], RS05_FloatToUint16(kp, RS05_KP_MIN, RS05_KP_MAX));
    RS05_WriteUint16BE(&data[6], RS05_FloatToUint16(kd, RS05_KD_MIN, RS05_KD_MAX));
    torque_raw = RS05_FloatToUint16(torque_nm, RS05_TORQUE_MIN_NM,
                                    RS05_TORQUE_MAX_NM);

    return RS05_SendData(hcan, RS05_COMM_TYPE_MOTION_CONTROL,
                         data, torque_raw, motor_id);
}

HAL_StatusTypeDef RS05_Stop(CAN_HandleTypeDef *hcan,
                            uint8_t master_id,
                            uint8_t motor_id)
{
    static const uint8_t data[8] = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    return RS05_SendData(hcan, RS05_COMM_TYPE_STOP, data, master_id, motor_id);
}

HAL_StatusTypeDef RS05_SetMotionMode(CAN_HandleTypeDef *hcan,
                                     uint8_t master_id,
                                     uint8_t motor_id)
{
    uint8_t data[8] = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    RS05_WriteUint16LE(&data[0], RS05_PARAMETER_RUN_MODE);
    RS05_WriteFloatLE(&data[4], RS05_RUN_MODE_MOTION);

    return RS05_SendData(hcan, RS05_COMM_TYPE_WRITE_PARAMETER,
                         data, master_id, motor_id);
}

void RS05_Manager_Init(RS05_ManagerTypedef *manager, CAN_HandleTypeDef *hcan)
{
    if (manager == NULL)
    {
        return;
    }

    memset(manager, 0, sizeof(*manager));
    manager->hcan = hcan;
}

HAL_StatusTypeDef RS05_Manager_RegisterMotor(RS05_ManagerTypedef *manager,
                                              RS05_MotorTypedef *motor)
{
    if ((manager == NULL) || (motor == NULL) ||
        (manager->num_motors >= RS05_MOTOR_MAX_COUNT))
    {
        return HAL_ERROR;
    }

    if (RS05_Manager_FindMotor(manager, motor->motor_id) != NULL)
    {
        return HAL_ERROR;
    }

    manager->motors[manager->num_motors] = motor;
    manager->num_motors++;
    return HAL_OK;
}

void RS05_Manager_ProcessRxFifo0(RS05_ManagerTypedef *manager)
{
    CAN_RxHeaderTypeDef rx_header;
    HAL_StatusTypeDef status;
    uint8_t rx_data[8];

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(manager->hcan, CAN_RX_FIFO0) > 0U)
    {
        uint8_t comm_type;
        uint8_t motor_id;
        RS05_MotorTypedef *motor;

        status = HAL_CAN_GetRxMessage(manager->hcan, CAN_RX_FIFO0,
                                       &rx_header, rx_data);
        if (status != HAL_OK)
        {
            break;
        }

        if ((rx_header.IDE != CAN_ID_EXT) ||
            (rx_header.RTR != CAN_RTR_DATA) ||
            (rx_header.DLC != 8U))
        {
            continue;
        }

        comm_type = (uint8_t)((rx_header.ExtId >> 24) & 0x1FU);
        /* RS05 反馈帧的 Data Area 2 低字节是发出反馈的电机 ID。 */
        motor_id = (uint8_t)((rx_header.ExtId >> 8) & 0xFFU);
        motor = RS05_Manager_FindMotor(manager, motor_id);
        if (motor == NULL)
        {
            continue;
        }



        switch (comm_type)
        {
            case RS05_COMM_TYPE_FEEDBACK:
                RS05_ParseType2Feedback(motor, rx_header.ExtId, rx_data);
                break;

            case RS05_COMM_TYPE_READ_PARAMETER:
                RS05_ParseType17ParameterReply(motor, rx_data);
                break;

            default:
                /* 其他通信类型保留原始帧，后续按手册添加专用解析器。 */
                break;
        }
    }
}
