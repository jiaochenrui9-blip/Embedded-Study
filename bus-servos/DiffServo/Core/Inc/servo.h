//
// Created by game on 2026/6/8.
//

#ifndef SC09SERVOTEST_SERVO_H
#define SC09SERVOTEST_SERVO_H
#include "main.h"

typedef struct
{
    uint16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
    uint8_t moving;
    int16_t current;
} Servo_Feedback_t;

extern UART_HandleTypeDef huart1;
void Servo_SendPacket(uint8_t id,uint8_t instruction,uint8_t *params,uint8_t param_len);
uint8_t Servo_ReceivePacket(uint8_t *rx_id, uint8_t *error, uint8_t *data, uint8_t *data_len, uint8_t max_len, uint32_t timeout);
uint8_t Servo_ReadValue(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_len, uint32_t timeout);
uint8_t Servo_GetByte(uint8_t id, uint8_t address, uint8_t *value, uint32_t timeout);
uint8_t Servo_GetWord(uint8_t id, uint8_t address, uint16_t *value, uint32_t timeout);
uint8_t Servo_GetID(uint8_t id, uint8_t *servo_id, uint32_t timeout);
uint8_t Servo_GetVersion(uint8_t id, uint16_t *version, uint32_t timeout);
uint8_t Servo_GetPosition(uint8_t id, uint16_t *position, uint32_t timeout);
uint8_t Servo_GetSpeed(uint8_t id, int16_t *speed, uint32_t timeout);
uint8_t Servo_GetLoad(uint8_t id, int16_t *load, uint32_t timeout);
uint8_t Servo_GetVoltage(uint8_t id, uint8_t *voltage, uint32_t timeout);
uint8_t Servo_GetTemperature(uint8_t id, uint8_t *temperature, uint32_t timeout);
uint8_t Servo_GetMoving(uint8_t id, uint8_t *moving, uint32_t timeout);
uint8_t Servo_GetCurrent(uint8_t id, int16_t *current, uint32_t timeout);
uint8_t Servo_GetAngleLimit(uint8_t id, uint16_t *min_position, uint16_t *max_position, uint32_t timeout);
uint8_t Servo_GetMode(uint8_t id, uint8_t *mode, uint32_t timeout);
uint8_t Servo_GetFeedback(uint8_t id, Servo_Feedback_t *feedback, uint32_t timeout);
void Servo_Ping(uint8_t id);
uint8_t Servo_PingCheck(uint8_t id, uint32_t timeout);
void Servo_ReadData(uint8_t id, uint8_t address, uint8_t read_len);
void Servo_WriteData(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_len);
void Servo_RegWrite(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_len);
void Servo_Action(void);
void Servo_SyncRead(uint8_t address, uint8_t read_len, uint8_t *ids, uint8_t id_count);
void Servo_SyncWrite(uint8_t address, uint8_t data_len, uint8_t *data, uint8_t servo_count);
void Servo_Reset(uint8_t id);
void Servo_SetID(uint8_t old_id, uint8_t new_id);
void Servo_SetID_Broadcast(uint8_t new_id);
void Servo_EnableTorque(uint8_t id, uint8_t enable);
void Servo_UnlockEprom(uint8_t id);
void Servo_LockEprom(uint8_t id);
void Servo_SetAngleLimit(uint8_t id, uint16_t min_position, uint16_t max_position);
void Servo_PositionMode(uint8_t id);
void Servo_PWMMode(uint8_t id);
void Servo_WritePWM(uint8_t id, int16_t pwm);
void Servo_Move(uint8_t id, uint16_t position, uint16_t time, uint16_t speed);
void Servo_SyncMove(uint8_t *ids, uint8_t servo_count, uint16_t position, uint16_t time, uint16_t speed);
void Servo_SyncMoveList(uint8_t *ids, uint16_t *positions, uint8_t servo_count, uint16_t time, uint16_t speed);
void Servo_SyncMove123(uint16_t position, uint16_t time, uint16_t speed);
#endif //SC09SERVOTEST_SERVO_H
