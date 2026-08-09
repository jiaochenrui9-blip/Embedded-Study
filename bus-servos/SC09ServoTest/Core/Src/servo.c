#include "servo_private.h"

void Servo_ManagerInit(Servo_ManagerTypeDef *manager, UART_HandleTypeDef *huart)
{
    if (manager == NULL)
    {
        return;
    }

    manager->huart = huart;
    manager->motor_count = 0U;
    for (uint8_t i = 0U; i < SERVO_MAX_MOTORS; i++)
    {
        manager->motors[i] = (Servo_MotorTypeDef){0};
    }
}

uint8_t Servo_RegisterMotor(Servo_ManagerTypeDef *manager, uint8_t device_id)
{
    Servo_MotorTypeDef *motor;

    if (manager == NULL || device_id >= SERVO_BROADCAST_ID ||
        manager->motor_count >= SERVO_MAX_MOTORS || Servo_FindMotor(manager, device_id) != NULL)
    {
        return 0U;
    }

    motor = &manager->motors[manager->motor_count];
    *motor = (Servo_MotorTypeDef){0};
    motor->device_id = device_id;
    manager->motor_count++;
    return 1U;
}

Servo_MotorTypeDef *Servo_FindMotor(Servo_ManagerTypeDef *manager, uint8_t device_id)
{
    if (manager == NULL)
    {
        return NULL;
    }

    for (uint8_t i = 0U; i < manager->motor_count; i++)
    {
        if (manager->motors[i].device_id == device_id)
        {
            return &manager->motors[i];
        }
    }

    return NULL;
}

void Servo_SendPacket(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t instruction,
                      const uint8_t *params, uint8_t param_len)
{
    if (id != SERVO_BROADCAST_ID && Servo_FindMotor(manager, id) == NULL)
    {
        return;
    }

    Servo_SendRawPacket(manager, id, instruction, params, param_len);
}

void Servo_SendRawPacket(Servo_ManagerTypeDef *manager, uint8_t id, uint8_t instruction,
                         const uint8_t *params, uint8_t param_len)
{
    uint8_t buf[128];
    uint8_t length;
    uint8_t sum;

    if (manager == NULL || manager->huart == NULL || param_len > SERVO_MAX_PARAM_LEN ||
        (params == NULL && param_len != 0U))
    {
        return;
    }

    length = (uint8_t)(param_len + 2U);
    buf[0] = 0xFFU;
    buf[1] = 0xFFU;
    buf[2] = id;
    buf[3] = length;
    buf[4] = instruction;
    sum = (uint8_t)(id + length + instruction);

    for (uint8_t i = 0U; i < param_len; i++)
    {
        buf[5U + i] = params[i];
        sum = (uint8_t)(sum + params[i]);
    }

    buf[5U + param_len] = (uint8_t)~sum;
    (void)HAL_UART_Transmit(manager->huart, buf, (uint16_t)(param_len + 6U), 1000U);
}
