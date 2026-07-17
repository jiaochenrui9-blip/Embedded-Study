//
// Created by game on 2026/7/16.
//
#include "RS05_app.h"
#include "RS05_codec.h"
#include "RS05_receive.h"
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
#define RS05_COMM_TYPE_ENABLE              0x03U
#define RS05_COMM_TYPE_STOP                0x04U
#define RS05_COMM_TYPE_READ_PARAMETER      0x11U
#define RS05_COMM_TYPE_WRITE_PARAMETER     0x12U

#define RS05_PARAMETER_RUN_MODE            0x7005U
#define RS05_RUN_MODE_MOTION                0U


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

    RS05_Codec_WriteU16BE(&data[0], RS05_Codec_FloatToU16(position_rad,
                                                     RS05_POSITION_MIN_RAD,
                                                     RS05_POSITION_MAX_RAD));
    RS05_Codec_WriteU16BE(&data[2], RS05_Codec_FloatToU16(velocity_rad_s,
                                                     RS05_VELOCITY_MIN_RAD_S,
                                                     RS05_VELOCITY_MAX_RAD_S));
    RS05_Codec_WriteU16BE(&data[4], RS05_Codec_FloatToU16(kp, RS05_KP_MIN, RS05_KP_MAX));
    RS05_Codec_WriteU16BE(&data[6], RS05_Codec_FloatToU16(kd, RS05_KD_MIN, RS05_KD_MAX));
    torque_raw = RS05_Codec_FloatToU16(torque_nm, RS05_TORQUE_MIN_NM,
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

    RS05_Codec_WriteU16LE(&data[0], RS05_PARAMETER_RUN_MODE);
    data[4] = RS05_RUN_MODE_MOTION;

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

static HAL_StatusTypeDef RS05_WriteParameterRaw(RS05_ManagerTypedef *manager,
                                                 uint8_t motor_id,
                                                 uint16_t index,
                                                 const uint8_t value[4])
{
    uint8_t data[8] = {0};

    if ((manager == NULL) || (manager->hcan == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteU16LE(&data[0], index);
    memcpy(&data[4], value, 4U);

    return RS05_SendData(manager->hcan,
                         RS05_COMM_TYPE_WRITE_PARAMETER,
                         data,
                         RS05_MASTER_ID,
                         motor_id);
}

HAL_StatusTypeDef RS05_Command_ParaRead(RS05_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        uint16_t index)
{

    uint8_t data[8] = {0};

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteU16LE(&data[0], index);
    return RS05_SendData(manager->hcan,
                         RS05_COMM_TYPE_READ_PARAMETER,
                         data,
                         RS05_MASTER_ID,
                         motor_id);

}

HAL_StatusTypeDef RS05_WriteParameterU8(RS05_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        uint16_t index,
                                        uint8_t value)
{
    uint8_t raw[4] = {value, 0U, 0U, 0U};

    return RS05_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_WriteParameterU16(RS05_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index,
                                         uint16_t value)
{
    uint8_t raw[4] = {0};

    RS05_Codec_WriteU16LE(raw, value);
    return RS05_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_WriteParameterU32(RS05_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index,
                                         uint32_t value)
{
    uint8_t raw[4] = {0};

    RS05_Codec_WriteU32LE(raw, value);
    return RS05_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_WriteParameterFloat(RS05_ManagerTypedef *manager,
                                           uint8_t motor_id,
                                           uint16_t index,
                                           float value)
{
    uint8_t raw[4] = {0};

    RS05_Codec_WriteFloatLE(raw, value);
    return RS05_WriteParameterRaw(manager, motor_id, index, raw);
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

        RS05_ProcessFrame(manager, rx_header.ExtId, rx_data);
    }
}
