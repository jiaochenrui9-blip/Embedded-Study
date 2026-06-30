//
// Created by game on 2026/6/10.
//
#include "main.h"
#ifndef ZP15DSERVOTESST_ZP15D_H
#define ZP15DSERVOTESST_ZP15D_H
extern UART_HandleTypeDef huart1;
void ZP15D_SendRaw(const char *cmd);
void ZP15D_ClearRx(void);
uint8_t ZP15D_ParseNumberAfter(const char *buf, const char *key, uint16_t *value);
uint8_t ZP15D_ParseReturnNumber(const char *buf, uint16_t *value);
uint16_t ZP15D_ReadReturn(char *buf, uint16_t len, uint32_t timeout);
void ZP15D_Move(uint8_t id, uint16_t pos, uint16_t time);
void ZP15D_ReadVersion(uint8_t id);
uint8_t ZP15D_ReadVersionValue(uint8_t id, uint16_t *version, uint32_t timeout);
void ZP15D_ReadID(uint8_t id);
uint8_t ZP15D_ReadIDValue(uint8_t id, uint8_t *servo_id, uint32_t timeout);
void ZP15D_SetID(uint8_t old_id, uint8_t new_id);
void ZP15D_SetIDByBroadcast(uint8_t new_id);
void ZP15D_ReleaseTorque(uint8_t id);
void ZP15D_RestoreTorque(uint8_t id);
void ZP15D_ReadMode(uint8_t id);
uint8_t ZP15D_ReadModeValue(uint8_t id, uint8_t *mode, uint32_t timeout);
void ZP15D_SetMode(uint8_t id, uint8_t mode);
void ZP15D_ReadPosition(uint8_t id);
uint8_t ZP15D_ReadPositionValue(uint8_t id, uint16_t *position, uint32_t timeout);
void ZP15D_Stop(uint8_t id);
void ZP15D_Pause(uint8_t id);
void ZP15D_Continue(uint8_t id);
void ZP15D_SetBaudRate(uint8_t id, uint8_t baud_index);
void ZP15D_CalibrateMiddle(uint8_t id);
void ZP15D_SetStartPosition(uint8_t id);
void ZP15D_DisableStartPosition(uint8_t id);
void ZP15D_EnableStartPosition(uint8_t id);
void ZP15D_SetMinPosition(uint8_t id);
void ZP15D_SetMaxPosition(uint8_t id);
void ZP15D_ResetExceptID(uint8_t id);
void ZP15D_ResetAll(uint8_t id);
void ZP15D_ReadTempVoltage(uint8_t id);
uint8_t ZP15D_ParseTempVoltageReturn(const char *ret, uint16_t *temperature, uint16_t *voltage_x10);
uint8_t ZP15D_ReadTempVoltageValue(uint8_t id, uint16_t *temperature, uint16_t *voltage_x10, uint32_t timeout);
uint8_t ZP15D_ReturnIsOK(const char *ret);
uint8_t ZP15D_ReadOK(uint32_t timeout);
#endif //ZP15DSERVOTESST_ZP15D_H
