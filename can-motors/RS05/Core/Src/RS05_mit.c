#include "RS05_mit.h"

#include "RS05_codec.h"

#include <string.h>

#define RS05_MIT_OPCODE_ENABLE_OR_MODE  0xFCU
#define RS05_MIT_OPCODE_STOP            0xFDU
#define RS05_MIT_OPCODE_ZERO            0xFEU
#define RS05_MIT_OPCODE_FAULT           0xFBU
#define RS05_MIT_OPCODE_ACTIVE_REPORT   0xF9U

static uint8_t RS05_MIT_InRange(float value, float minimum, float maximum)
{
    return (uint8_t)((value >= minimum) && (value <= maximum));
}

static uint16_t RS05_MIT_FloatToUint(float value,
                                     float minimum,
                                     float maximum,
                                     uint8_t bits)
{
    const uint32_t span = (1UL << bits) - 1UL;

    return (uint16_t)((value - minimum) * (float)span /
                      (maximum - minimum));
}

static float RS05_MIT_UintToFloat(uint16_t value,
                                  float minimum,
                                  float maximum,
                                  uint8_t bits)
{
    const uint32_t span = (1UL << bits) - 1UL;

    return ((float)value * (maximum - minimum) / (float)span) + minimum;
}

static uint8_t RS05_MIT_Manager_IsValid(
    const RS05_MIT_ManagerTypedef *manager)
{
    return (uint8_t)((manager != NULL) && (manager->hcan != NULL));
}

static HAL_StatusTypeDef RS05_MIT_SendSpecial(
                                               RS05_MIT_ManagerTypedef *manager,
                                               uint8_t motor_id,
                                               uint8_t argument,
                                               uint8_t opcode)
{
    uint8_t data[8] = {
        0xFFU, 0xFFU, 0xFFU, 0xFFU,
        0xFFU, 0xFFU, argument, opcode
    };

    if (!RS05_MIT_Manager_IsValid(manager))
    {
        return HAL_ERROR;
    }

    return CAN_SendStdFrame(manager->hcan, motor_id, data, 8U);
}

RS05_MIT_MotorTypedef *RS05_MIT_Manager_FindMotor(
    const RS05_MIT_ManagerTypedef *manager,
    uint8_t motor_id)
{
    uint8_t i;

    if (manager == NULL)
    {
        return NULL;
    }

    for (i = 0U; i < manager->num_motors; ++i)
    {
        if ((manager->motors[i] != NULL) &&
            (manager->motors[i]->motor_id == motor_id))
        {
            return manager->motors[i];
        }
    }

    return NULL;
}

static uint8_t RS05_MIT_DeadlineReached(uint32_t now, uint32_t deadline)
{
    return (uint8_t)((int32_t)(now - deadline) >= 0);
}

void RS05_MIT_Manager_Init(RS05_MIT_ManagerTypedef *manager,
                           CAN_HandleTypeDef *hcan,
                           uint8_t master_id)
{
    if (manager == NULL)
    {
        return;
    }

    memset(manager, 0, sizeof(*manager));
    manager->hcan = hcan;
    manager->master_id = master_id;
}

HAL_StatusTypeDef RS05_MIT_Manager_RegisterMotor(
    RS05_MIT_ManagerTypedef *manager,
    RS05_MIT_MotorTypedef *motor)
{
    if ((manager == NULL) || (motor == NULL) ||
        (manager->num_motors >= RS05_MIT_MOTOR_MAX_COUNT) ||
        (RS05_MIT_Manager_FindMotor(manager, motor->motor_id) != NULL))
    {
        return HAL_ERROR;
    }

    manager->motors[manager->num_motors] = motor;
    ++manager->num_motors;
    return HAL_OK;
}

uint16_t RS05_MIT_BuildStdId(uint8_t frame_type, uint8_t motor_id)
{
    return (uint16_t)(((uint16_t)(frame_type & 0x07U) << 8) | motor_id);
}

HAL_StatusTypeDef RS05_MIT_PackControl(uint8_t data[8],
                                       float position_rad,
                                       float velocity_rad_s,
                                       float kp,
                                       float kd,
                                       float torque_nm)
{
    uint16_t position;
    uint16_t velocity;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque;

    if ((data == NULL) ||
        !RS05_MIT_InRange(position_rad, RS05_MIT_POSITION_MIN_RAD,
                          RS05_MIT_POSITION_MAX_RAD) ||
        !RS05_MIT_InRange(velocity_rad_s, RS05_MIT_VELOCITY_MIN_RAD_S,
                          RS05_MIT_VELOCITY_MAX_RAD_S) ||
        !RS05_MIT_InRange(kp, RS05_MIT_KP_MIN, RS05_MIT_KP_MAX) ||
        !RS05_MIT_InRange(kd, RS05_MIT_KD_MIN, RS05_MIT_KD_MAX) ||
        !RS05_MIT_InRange(torque_nm, RS05_MIT_TORQUE_MIN_NM,
                          RS05_MIT_TORQUE_MAX_NM))
    {
        return HAL_ERROR;
    }

    position = RS05_MIT_FloatToUint(position_rad,
                                    RS05_MIT_POSITION_MIN_RAD,
                                    RS05_MIT_POSITION_MAX_RAD, 16U);
    velocity = RS05_MIT_FloatToUint(velocity_rad_s,
                                    RS05_MIT_VELOCITY_MIN_RAD_S,
                                    RS05_MIT_VELOCITY_MAX_RAD_S, 12U);
    kp_raw = RS05_MIT_FloatToUint(kp, RS05_MIT_KP_MIN,
                                  RS05_MIT_KP_MAX, 12U);
    kd_raw = RS05_MIT_FloatToUint(kd, RS05_MIT_KD_MIN,
                                  RS05_MIT_KD_MAX, 12U);
    torque = RS05_MIT_FloatToUint(torque_nm,
                                  RS05_MIT_TORQUE_MIN_NM,
                                  RS05_MIT_TORQUE_MAX_NM, 12U);

    data[0] = (uint8_t)(position >> 8);
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(velocity >> 4);
    data[3] = (uint8_t)((velocity << 4) | (kp_raw >> 8));
    data[4] = (uint8_t)kp_raw;
    data[5] = (uint8_t)(kd_raw >> 4);
    data[6] = (uint8_t)((kd_raw << 4) | (torque >> 8));
    data[7] = (uint8_t)torque;

    return HAL_OK;
}

HAL_StatusTypeDef RS05_MIT_ParseFeedback(RS05_MIT_MotorTypedef *motor,
                                         const uint8_t data[8])
{
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint16_t temperature;

    if ((motor == NULL) || (data == NULL) ||
        (data[0] != motor->motor_id))
    {
        return HAL_ERROR;
    }

    position = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    velocity = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    torque = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);
    temperature = (uint16_t)((((uint16_t)data[6] & 0x0FU) << 8) |
                             data[7]);

    motor->position_rad = RS05_MIT_UintToFloat(
        position, RS05_MIT_POSITION_MIN_RAD, RS05_MIT_POSITION_MAX_RAD, 16U);
    motor->velocity_rad_s = RS05_MIT_UintToFloat(
        velocity, RS05_MIT_VELOCITY_MIN_RAD_S,
        RS05_MIT_VELOCITY_MAX_RAD_S, 12U);
    motor->torque_nm = RS05_MIT_UintToFloat(
        torque, RS05_MIT_TORQUE_MIN_NM, RS05_MIT_TORQUE_MAX_NM, 12U);
    motor->state = (uint8_t)((data[6] >> 6) & 0x03U);
    motor->temperature_c = (float)temperature * 0.1f;
    motor->last_feedback_tick = HAL_GetTick();

    return HAL_OK;
}

HAL_StatusTypeDef RS05_MIT_ProcessFrame(
    RS05_MIT_ManagerTypedef *manager,
    uint16_t std_id,
    const uint8_t data[8])
{
    RS05_MIT_MotorTypedef *motor;
    uint8_t frame_type;
    uint32_t now;

    if (!RS05_MIT_Manager_IsValid(manager) ||
        (data == NULL) || (std_id > 0x07FFU))
    {
        return HAL_ERROR;
    }

    if (std_id == manager->master_id)
    {
        motor = RS05_MIT_Manager_FindMotor(manager, data[0]);
        if (motor == NULL)
        {
            return HAL_ERROR;
        }

        now = HAL_GetTick();
        if ((motor->fault_deadline_tick != 0U) &&
            RS05_MIT_DeadlineReached(now, motor->fault_deadline_tick))
        {
            motor->fault_deadline_tick = 0U;
        }

        if ((motor->fault_deadline_tick != 0U) &&
            (data[5] == 0U) && (data[6] == 0U) && (data[7] == 0U))
        {
            motor->fault_code = RS05_Codec_ReadU32LE(&data[1]);
            motor->fault_deadline_tick = 0U;
            return HAL_OK;
        }

        return RS05_MIT_ParseFeedback(motor, data);
    }

    frame_type = (uint8_t)((std_id >> 8) & 0x07U);
    motor = RS05_MIT_Manager_FindMotor(manager,
                                      (uint8_t)(std_id & 0xFFU));
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    if ((frame_type != RS05_MIT_FRAME_READ_PARAMETER) &&
        (frame_type != RS05_MIT_FRAME_WRITE_PARAMETER))
    {
        return HAL_ERROR;
    }

    motor->parameter_index = RS05_Codec_ReadU16LE(&data[0]);
    memcpy(motor->parameter_data, &data[4], sizeof(motor->parameter_data));
    return HAL_OK;
}

HAL_StatusTypeDef RS05_MIT_Enable(RS05_MIT_ManagerTypedef *manager,
                                  uint8_t motor_id)
{
    return RS05_MIT_SendSpecial(manager, motor_id, 0xFFU,
                                RS05_MIT_OPCODE_ENABLE_OR_MODE);
}

HAL_StatusTypeDef RS05_MIT_Stop(RS05_MIT_ManagerTypedef *manager,
                                uint8_t motor_id)
{
    return RS05_MIT_SendSpecial(manager, motor_id, 0xFFU,
                                RS05_MIT_OPCODE_STOP);
}

HAL_StatusTypeDef RS05_MIT_SetZero(RS05_MIT_ManagerTypedef *manager,
                                   uint8_t motor_id)
{
    return RS05_MIT_SendSpecial(manager, motor_id, 0xFFU,
                                RS05_MIT_OPCODE_ZERO);
}

HAL_StatusTypeDef RS05_MIT_SetRunMode(RS05_MIT_ManagerTypedef *manager,
                                      uint8_t motor_id,
                                      RS05_MIT_RunMode mode)
{
    if (mode > RS05_MIT_RUN_MODE_SPEED)
    {
        return HAL_ERROR;
    }

    return RS05_MIT_SendSpecial(manager, motor_id, (uint8_t)mode,
                                RS05_MIT_OPCODE_ENABLE_OR_MODE);
}

HAL_StatusTypeDef RS05_MIT_Control(RS05_MIT_ManagerTypedef *manager,
                                   uint8_t motor_id,
                                   float position_rad,
                                   float velocity_rad_s,
                                   float kp,
                                   float kd,
                                   float torque_nm)
{
    uint8_t data[8];
    HAL_StatusTypeDef status;

    status = RS05_MIT_PackControl(data, position_rad, velocity_rad_s,
                                  kp, kd, torque_nm);
    if (status != HAL_OK)
    {
        return status;
    }

    if (!RS05_MIT_Manager_IsValid(manager))
    {
        return HAL_ERROR;
    }

    return CAN_SendStdFrame(manager->hcan, motor_id, data, 8U);
}

HAL_StatusTypeDef RS05_MIT_PositionControl(RS05_MIT_ManagerTypedef *manager,
                                           uint8_t motor_id,
                                           float position_rad,
                                           float velocity_rad_s)
{
    uint8_t data[8];

    if (!RS05_MIT_Manager_IsValid(manager))
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteFloatLE(&data[0], position_rad);
    RS05_Codec_WriteFloatLE(&data[4], velocity_rad_s);
    return CAN_SendStdFrame(manager->hcan,
                            RS05_MIT_BuildStdId(RS05_MIT_FRAME_POSITION,
                                                motor_id),
                            data, 8U);
}

HAL_StatusTypeDef RS05_MIT_SpeedControl(RS05_MIT_ManagerTypedef *manager,
                                        uint8_t motor_id,
                                        float velocity_rad_s,
                                        float current_limit_a)
{
    uint8_t data[8];

    if (!RS05_MIT_Manager_IsValid(manager))
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteFloatLE(&data[0], velocity_rad_s);
    RS05_Codec_WriteFloatLE(&data[4], current_limit_a);
    return CAN_SendStdFrame(manager->hcan,
                            RS05_MIT_BuildStdId(RS05_MIT_FRAME_SPEED,
                                                motor_id),
                            data, 8U);
}

HAL_StatusTypeDef RS05_MIT_ReadFault(RS05_MIT_ManagerTypedef *manager,
                                     RS05_MIT_MotorTypedef *motor)
{
    uint32_t now;
    HAL_StatusTypeDef status;

    if (!RS05_MIT_Manager_IsValid(manager) || (motor == NULL))
    {
        return HAL_ERROR;
    }

    now = HAL_GetTick();
    status = RS05_MIT_SendSpecial(manager, motor->motor_id, 0x00U,
                                  RS05_MIT_OPCODE_FAULT);
    if (status == HAL_OK)
    {
        motor->fault_deadline_tick = now + RS05_MIT_FAULT_TIMEOUT_MS;
    }
    return status;
}

HAL_StatusTypeDef RS05_MIT_SetActiveReport(RS05_MIT_ManagerTypedef *manager,
                                           uint8_t motor_id,
                                           uint8_t enable)
{
    if (enable > 1U)
    {
        return HAL_ERROR;
    }

    return RS05_MIT_SendSpecial(manager, motor_id, enable,
                                RS05_MIT_OPCODE_ACTIVE_REPORT);
}

HAL_StatusTypeDef RS05_MIT_ReadParameter(RS05_MIT_ManagerTypedef *manager,
                                         uint8_t motor_id,
                                         uint16_t index)
{
    uint8_t data[8] = {0U};

    if (!RS05_MIT_Manager_IsValid(manager))
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteU16LE(data, index);
    return CAN_SendStdFrame(manager->hcan,
                            RS05_MIT_BuildStdId(
                                RS05_MIT_FRAME_READ_PARAMETER, motor_id),
                            data, 8U);
}

HAL_StatusTypeDef RS05_MIT_WriteParameterRaw(RS05_MIT_ManagerTypedef *manager,
                                              uint8_t motor_id,
                                              uint16_t index,
                                              const uint8_t value[4])
{
    uint8_t data[8] = {0U};

    if (!RS05_MIT_Manager_IsValid(manager) || (value == NULL))
    {
        return HAL_ERROR;
    }

    RS05_Codec_WriteU16LE(&data[0], index);
    memcpy(&data[4], value, 4U);

    return CAN_SendStdFrame(manager->hcan,
                            RS05_MIT_BuildStdId(
                                RS05_MIT_FRAME_WRITE_PARAMETER, motor_id),
                            data, 8U);
}



HAL_StatusTypeDef RS05_MIT_WriteParameterU8(RS05_MIT_ManagerTypedef *manager,
                                             uint8_t motor_id,
                                             uint16_t index,
                                             uint8_t value)
{
    const uint8_t raw[4] = {value, 0U, 0U, 0U};

    return RS05_MIT_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_MIT_WriteParameterU16(RS05_MIT_ManagerTypedef *manager,
                                              uint8_t motor_id,
                                              uint16_t index,
                                              uint16_t value)
{
    uint8_t raw[4] = {0U};

    RS05_Codec_WriteU16LE(raw, value);
    return RS05_MIT_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_MIT_WriteParameterU32(RS05_MIT_ManagerTypedef *manager,
                                              uint8_t motor_id,
                                              uint16_t index,
                                              uint32_t value)
{
    uint8_t raw[4] = {0U};

    RS05_Codec_WriteU32LE(raw, value);
    return RS05_MIT_WriteParameterRaw(manager, motor_id, index, raw);
}

HAL_StatusTypeDef RS05_MIT_WriteParameterFloat(RS05_MIT_ManagerTypedef *manager,
                                                uint8_t motor_id,
                                                uint16_t index,
                                                float value)
{
    uint8_t raw[4] = {0U};

    RS05_Codec_WriteFloatLE(raw, value);
    return RS05_MIT_WriteParameterRaw(manager, motor_id, index, raw);
}

uint8_t RS05_MIT_IsOnline(const RS05_MIT_MotorTypedef *motor,
                          uint32_t timeout_ms)
{
    if ((motor == NULL) || (motor->last_feedback_tick == 0U))
    {
        return 0U;
    }

    return (uint8_t)((HAL_GetTick() - motor->last_feedback_tick) <= timeout_ms);
}
