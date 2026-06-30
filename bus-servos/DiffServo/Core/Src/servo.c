#include <stdint.h>
#include "servo.h"

#define SERVO_MAX_PARAM_LEN 122U
#define SERVO_VERSION_ADDR 0x03U
#define SERVO_ID_ADDR 0x05U
#define SERVO_MIN_ANGLE_LIMIT_ADDR 0x09U
#define SERVO_TORQUE_ENABLE_ADDR 0x28U
#define SERVO_GOAL_POSITION_ADDR 0x2AU
#define SERVO_GOAL_TIME_ADDR 0x2CU
#define SERVO_LOCK_ADDR 0x30U
#define SERVO_PRESENT_POSITION_ADDR 0x38U
#define SERVO_PRESENT_SPEED_ADDR 0x3AU
#define SERVO_PRESENT_LOAD_ADDR 0x3CU
#define SERVO_PRESENT_VOLTAGE_ADDR 0x3EU
#define SERVO_PRESENT_TEMPERATURE_ADDR 0x3FU
#define SERVO_MOVING_ADDR 0x42U
#define SERVO_PRESENT_CURRENT_ADDR 0x46U
#define SERVO_SYNC_MOVE_DATA_LEN 7U
#define SERVO_SYNC_MOVE_MAX_COUNT ((SERVO_MAX_PARAM_LEN - 2U) / SERVO_SYNC_MOVE_DATA_LEN)

static void Servo_ClearRx(void);

//
// Created by game on 2026/6/8.
//
void Servo_SendPacket(uint8_t id,uint8_t instruction,uint8_t *params,uint8_t param_len)
{
    uint8_t buf[128];
    uint8_t length=param_len + 2;
    uint8_t sum;

    if (param_len > SERVO_MAX_PARAM_LEN)
    {
        return;
    }
    if (params == NULL && param_len != 0)
    {
        return;
    }

    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = length;
    buf[4] = instruction;


    sum = id + length + instruction;
    for (int i =0; i< param_len; i++)
    {
        buf[5 + i] = params[i];
        sum += params[i];
    }
    buf[5 + param_len] = ~(sum & 0xFF);
    HAL_UART_Transmit(&huart1, buf,  param_len +6 , 1000);
}

void Servo_Ping(uint8_t id)
{
    Servo_SendPacket(id, 0x01, NULL, 0);
}

uint8_t Servo_PingCheck(uint8_t id, uint32_t timeout)
{
    uint8_t rx_id;
    uint8_t error;
    uint8_t data_len;

    Servo_ClearRx();
    Servo_Ping(id);

    if (Servo_ReceivePacket(&rx_id, &error, NULL, &data_len, 0, timeout) == 0)
    {
        return 0;
    }

    if (rx_id != id)
    {
        return 0;
    }

    if (error != 0 || data_len != 0)
    {
        return 0;
    }

    return 1;
}
void Servo_ReadData(uint8_t id, uint8_t address, uint8_t read_len)
{
    uint8_t params[2];

    params[0] = address;
    params[1] = read_len;
    Servo_SendPacket(id, 0x02, params, 2);
}
void Servo_WriteData(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_len)
{
    uint8_t params[SERVO_MAX_PARAM_LEN];

    if (data_len + 1U > SERVO_MAX_PARAM_LEN)
    {
        return;
    }

    params[0] = address;
    for (int i = 0; i < data_len; i++)
    {
        params[i + 1] = data[i];
    }

    Servo_SendPacket(id, 0x03, params, data_len + 1);
}
void Servo_RegWrite(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_len)
{
    uint8_t params[SERVO_MAX_PARAM_LEN];

    if (data_len + 1U > SERVO_MAX_PARAM_LEN)
    {
        return;
    }
    params[0] = address;
    for (int i = 0; i < data_len; i++)
    {
        params[i + 1] = data[i];
    }

    Servo_SendPacket(id, 0x04, params, data_len + 1);
}
void Servo_Action(void)
{
    Servo_SendPacket(0xFE, 0x05, NULL, 0);
}
void Servo_SyncRead(uint8_t address, uint8_t read_len, uint8_t *ids, uint8_t id_count)
{
    uint8_t params[SERVO_MAX_PARAM_LEN];
    uint16_t param_len = (uint16_t)id_count + 2U;

    if (param_len > SERVO_MAX_PARAM_LEN)
    {
        return;
    }

    params[0] = address;
    params[1] = read_len;
    for (uint16_t i = 0; i < id_count; i++)
    {
        params[i + 2U] = ids[i];
    }

    Servo_SendPacket(0xFE, 0x82, params, (uint8_t)param_len);
}
void Servo_SyncWrite(uint8_t address, uint8_t data_len, uint8_t *data, uint8_t servo_count)
{
    uint8_t params[SERVO_MAX_PARAM_LEN];
    uint16_t data_total_len = (uint16_t)servo_count * ((uint16_t)data_len + 1U);
    uint16_t param_len = data_total_len + 2U;

    if (param_len > SERVO_MAX_PARAM_LEN)
    {
        return;
    }

    params[0] = address;
    params[1] = data_len;
    for (uint16_t i = 0; i < data_total_len; i++)
    {
        params[i + 2U] = data[i];
    }

    Servo_SendPacket(0xFE, 0x83, params, (uint8_t)param_len);
}
void Servo_Reset(uint8_t id)
{
    Servo_SendPacket(id, 0x06, NULL, 0);
}
void Servo_SetID(uint8_t old_id, uint8_t new_id)
{
    uint8_t data[1];

    if (new_id >= 0xFE)
    {
        return;
    }

    Servo_UnlockEprom(old_id);
    HAL_Delay(20);
    data[0] = new_id;
    Servo_WriteData(old_id, SERVO_ID_ADDR, data, 1);
    HAL_Delay(20);
    Servo_LockEprom(new_id);
}
void Servo_SetID_Broadcast(uint8_t new_id)
{

    Servo_SetID(0xFE, new_id);
}

void Servo_EnableTorque(uint8_t id, uint8_t enable)
{
    uint8_t data[1];

    data[0] = enable;
    Servo_WriteData(id, SERVO_TORQUE_ENABLE_ADDR, data, 1);
}
void Servo_UnlockEprom(uint8_t id)
{
    uint8_t data[1];

    data[0] = 0;
    Servo_WriteData(id, SERVO_LOCK_ADDR, data, 1);
}
void Servo_LockEprom(uint8_t id)
{
    uint8_t data[1];

    data[0] = 1;
    Servo_WriteData(id, SERVO_LOCK_ADDR, data, 1);
}
void Servo_SetAngleLimit(uint8_t id, uint16_t min_position, uint16_t max_position)
{
    uint8_t data[4];

    data[0] = min_position >> 8;
    data[1] = min_position & 0xFF;
    data[2] = max_position >> 8;
    data[3] = max_position & 0xFF;
    Servo_WriteData(id, SERVO_MIN_ANGLE_LIMIT_ADDR, data, 4);
}
void Servo_PositionMode(uint8_t id)
{
    Servo_SetAngleLimit(id, 0, 1023);
}
void Servo_PWMMode(uint8_t id)
{
    Servo_SetAngleLimit(id, 0, 0);
}
void Servo_WritePWM(uint8_t id, int16_t pwm)
{
    uint8_t data[2];
    uint16_t value;

    if (pwm > 1000)
    {
        pwm = 1000;
    }
    if (pwm < -1000)
    {
        pwm = -1000;
    }

    if (pwm < 0)
    {
        value = (uint16_t)(-pwm);
        value |= (1U << 10);
    }
    else
    {
        value = (uint16_t)pwm;
    }

    data[0] = value >> 8;
    data[1] = value & 0xFF;
    Servo_WriteData(id, SERVO_GOAL_TIME_ADDR, data, 2);
}
void Servo_Move(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
    uint8_t params[7];

    params[0] = SERVO_GOAL_POSITION_ADDR;
    params[1] = position >> 8;
    params[2] = position & 0xFF;
    params[3] = time >> 8;
    params[4] = time & 0xFF;
    params[5] = speed >> 8;
    params[6] = speed & 0xFF;

    Servo_SendPacket(id, 0x03, params, 7);
}

void Servo_SyncMove(uint8_t *ids, uint8_t servo_count, uint16_t position, uint16_t time, uint16_t speed)
{
    uint8_t data[SERVO_SYNC_MOVE_MAX_COUNT * SERVO_SYNC_MOVE_DATA_LEN];
    uint8_t i;
    uint8_t offset;

    if (ids == NULL || servo_count == 0)
    {
        return;
    }

    if (servo_count > SERVO_SYNC_MOVE_MAX_COUNT)
    {
        servo_count = SERVO_SYNC_MOVE_MAX_COUNT;
    }

    for (i = 0; i < servo_count; i++)
    {
        offset = i * SERVO_SYNC_MOVE_DATA_LEN;
        data[offset] = ids[i];
        data[offset + 1] = position >> 8;
        data[offset + 2] = position & 0xFF;
        data[offset + 3] = time >> 8;
        data[offset + 4] = time & 0xFF;
        data[offset + 5] = speed >> 8;
        data[offset + 6] = speed & 0xFF;
    }

    Servo_SyncWrite(SERVO_GOAL_POSITION_ADDR, 6, data, servo_count);
}

void Servo_SyncMoveList(uint8_t *ids, uint16_t *positions, uint8_t servo_count, uint16_t time, uint16_t speed)
{
    uint8_t data[SERVO_SYNC_MOVE_MAX_COUNT * SERVO_SYNC_MOVE_DATA_LEN];
    uint8_t i;
    uint8_t offset;

    if (ids == NULL || positions == NULL || servo_count == 0)
    {
        return;
    }

    if (servo_count > SERVO_SYNC_MOVE_MAX_COUNT)
    {
        servo_count = SERVO_SYNC_MOVE_MAX_COUNT;
    }

    for (i = 0; i < servo_count; i++)
    {
        offset = i * SERVO_SYNC_MOVE_DATA_LEN;
        data[offset] = ids[i];
        data[offset + 1] = positions[i] >> 8;
        data[offset + 2] = positions[i] & 0xFF;
        data[offset + 3] = time >> 8;
        data[offset + 4] = time & 0xFF;
        data[offset + 5] = speed >> 8;
        data[offset + 6] = speed & 0xFF;
    }

    Servo_SyncWrite(SERVO_GOAL_POSITION_ADDR, 6, data, servo_count);
}

void Servo_SyncMove123(uint16_t position, uint16_t time, uint16_t speed)
{
    uint8_t ids[3] = {1, 2, 3};

    Servo_SyncMove(ids, 3, position, time, speed);
}
static uint16_t Servo_MakeWord(uint8_t high, uint8_t low)
{
    return ((uint16_t)high << 8) | low;
}

static int16_t Servo_MakeSignedWord(uint16_t value, uint8_t sign_bit)
{
    if (value & (1U << sign_bit))
    {
        value &= (uint16_t)~(1U << sign_bit);
        return -(int16_t)value;
    }

    return (int16_t)value;
}

static void Servo_ClearRx(void)
{
    uint8_t data;

    while (HAL_UART_Receive(&huart1, &data, 1, 1) == HAL_OK)
    {
    }
}

uint8_t Servo_ReceivePacket(uint8_t *rx_id, uint8_t *error, uint8_t *data, uint8_t *data_len, uint8_t max_len, uint32_t timeout)
{
    uint8_t byte = 0;
    uint8_t last = 0;
    uint8_t head_count = 0;
    uint8_t length;
    uint8_t rx_data_len;
    uint8_t sum;
    uint8_t checksum;

    if (rx_id == NULL || error == NULL || data_len == NULL)
    {
        return 0;
    }

    while (head_count < 20)
    {
        if (HAL_UART_Receive(&huart1, &byte, 1, timeout) != HAL_OK)
        {
            return 0;
        }

        if (last == 0xFF && byte == 0xFF)
        {
            break;
        }

        last = byte;
        head_count++;
    }

    if (head_count >= 20)
    {
        return 0;
    }
    if (HAL_UART_Receive(&huart1, rx_id, 1, timeout) != HAL_OK) return 0;

    if (HAL_UART_Receive(&huart1, &length, 1, timeout) != HAL_OK) return 0;
    if (HAL_UART_Receive(&huart1, error, 1, timeout) != HAL_OK) return 0;

    if (length < 2)
    {
        return 0;
    }

    rx_data_len = length - 2;
    sum = *rx_id + length + *error;

    for (uint8_t i = 0; i < rx_data_len; i++)
    {
        if (HAL_UART_Receive(&huart1, &byte, 1, timeout) != HAL_OK)
        {
            return 0;
        }

        if (data != NULL && i < max_len)
        {
            data[i] = byte;
        }

        sum += byte;
    }

    if (HAL_UART_Receive(&huart1, &checksum, 1, timeout) != HAL_OK)
    {
        return 0;
    }

    if ((uint8_t)(~sum) != checksum)
    {
        return 0;
    }

    if (rx_data_len > max_len)
    {
        return 0;
    }
    *data_len = rx_data_len;
    return 1;
}

uint8_t Servo_ReadValue(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_len, uint32_t timeout)
{
    uint8_t rx_id;
    uint8_t error;
    uint8_t rx_len;
    uint8_t rx_data[SERVO_MAX_PARAM_LEN];

    if (data == NULL || data_len == 0 || data_len > SERVO_MAX_PARAM_LEN)
    {
        return 0;
    }

    Servo_ClearRx();
    Servo_ReadData(id, address, data_len);

    for (uint8_t retry = 0; retry < 4; retry++)
    {
        if (Servo_ReceivePacket(&rx_id, &error, rx_data, &rx_len, SERVO_MAX_PARAM_LEN, timeout) == 0)
        {
            return 0;
        }

        if (rx_id != id)
        {
            continue;
        }

        if (error != 0)
        {
            continue;
        }

        if (rx_len != data_len)
        {
            continue;
        }

        for (uint8_t i = 0; i < data_len; i++)
        {
            data[i] = rx_data[i];
        }

        return 1;
    }

    return 0;
}

uint8_t Servo_GetByte(uint8_t id, uint8_t address, uint8_t *value, uint32_t timeout)
{
    uint8_t data[1];

    if (value == NULL)
    {
        return 0;
    }

    if (Servo_ReadValue(id, address, data, 1, timeout) == 0)
    {
        return 0;
    }

    *value = data[0];
    return 1;
}

uint8_t Servo_GetWord(uint8_t id, uint8_t address, uint16_t *value, uint32_t timeout)
{
    uint8_t data[2];

    if (value == NULL)
    {
        return 0;
    }

    if (Servo_ReadValue(id, address, data, 2, timeout) == 0)
    {
        return 0;
    }

    *value = Servo_MakeWord(data[0], data[1]);
    return 1;
}

uint8_t Servo_GetID(uint8_t id, uint8_t *servo_id, uint32_t timeout)
{
    return Servo_GetByte(id, SERVO_ID_ADDR, servo_id, timeout);
}

uint8_t Servo_GetVersion(uint8_t id, uint16_t *version, uint32_t timeout)
{
    return Servo_GetWord(id, SERVO_VERSION_ADDR, version, timeout);
}

uint8_t Servo_GetPosition(uint8_t id, uint16_t *position, uint32_t timeout)
{
    return Servo_GetWord(id, SERVO_PRESENT_POSITION_ADDR, position, timeout);
}

uint8_t Servo_GetSpeed(uint8_t id, int16_t *speed, uint32_t timeout)
{
    uint16_t value;

    if (speed == NULL)
    {
        return 0;
    }

    if (Servo_GetWord(id, SERVO_PRESENT_SPEED_ADDR, &value, timeout) == 0)
    {
        return 0;
    }

    *speed = Servo_MakeSignedWord(value, 15);
    return 1;
}

uint8_t Servo_GetLoad(uint8_t id, int16_t *load, uint32_t timeout)
{
    uint16_t value;

    if (load == NULL)
    {
        return 0;
    }

    if (Servo_GetWord(id, SERVO_PRESENT_LOAD_ADDR, &value, timeout) == 0)
    {
        return 0;
    }

    *load = Servo_MakeSignedWord(value, 10);
    return 1;
}

uint8_t Servo_GetVoltage(uint8_t id, uint8_t *voltage, uint32_t timeout)
{
    return Servo_GetByte(id, SERVO_PRESENT_VOLTAGE_ADDR, voltage, timeout);
}

uint8_t Servo_GetTemperature(uint8_t id, uint8_t *temperature, uint32_t timeout)
{
    return Servo_GetByte(id, SERVO_PRESENT_TEMPERATURE_ADDR, temperature, timeout);
}

uint8_t Servo_GetMoving(uint8_t id, uint8_t *moving, uint32_t timeout)
{
    return Servo_GetByte(id, SERVO_MOVING_ADDR, moving, timeout);
}

uint8_t Servo_GetCurrent(uint8_t id, int16_t *current, uint32_t timeout)
{
    uint16_t value;

    if (current == NULL)
    {
        return 0;
    }

    if (Servo_GetWord(id, SERVO_PRESENT_CURRENT_ADDR, &value, timeout) == 0)
    {
        return 0;
    }

    *current = Servo_MakeSignedWord(value, 15);
    return 1;
}

uint8_t Servo_GetAngleLimit(uint8_t id, uint16_t *min_position, uint16_t *max_position, uint32_t timeout)
{
    uint8_t data[4];

    if (min_position == NULL || max_position == NULL)
    {
        return 0;
    }

    if (Servo_ReadValue(id, SERVO_MIN_ANGLE_LIMIT_ADDR, data, 4, timeout) == 0)
    {
        return 0;
    }

    *min_position = Servo_MakeWord(data[0], data[1]);
    *max_position = Servo_MakeWord(data[2], data[3]);
    return 1;
}

uint8_t Servo_GetMode(uint8_t id, uint8_t *mode, uint32_t timeout)
{
    uint16_t min_position;
    uint16_t max_position;

    if (mode == NULL)
    {
        return 0;
    }

    if (Servo_GetAngleLimit(id, &min_position, &max_position, timeout) == 0)
    {
        return 0;
    }

    *mode = (min_position == 0 && max_position == 0) ? 1 : 0;
    return 1;
}

uint8_t Servo_GetFeedback(uint8_t id, Servo_Feedback_t *feedback, uint32_t timeout)
{
    uint8_t data[16];

    if (feedback == NULL)
    {
        return 0;
    }

    if (Servo_ReadValue(id, SERVO_PRESENT_POSITION_ADDR, data, 16, timeout) == 0)
    {
        return 0;
    }

    feedback->position = Servo_MakeWord(data[0], data[1]);
    feedback->speed = Servo_MakeSignedWord(Servo_MakeWord(data[2], data[3]), 15);
    feedback->load = Servo_MakeSignedWord(Servo_MakeWord(data[4], data[5]), 10);
    feedback->voltage = data[6];
    feedback->temperature = data[7];
    feedback->moving = data[10];
    feedback->current = Servo_MakeSignedWord(Servo_MakeWord(data[14], data[15]), 15);
    return 1;
}
