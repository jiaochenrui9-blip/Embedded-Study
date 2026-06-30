#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ZP15D.h"



//
// Created by game on 2026/6/10.
//
void ZP15D_SendRaw(const char *cmd)
{
   HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);
}

void ZP15D_ClearRx(void)
{
   uint8_t data;

   while (HAL_UART_Receive(&huart1, &data, 1, 1) == HAL_OK)
   {
   }
}

void ZP15D_Move(uint8_t id, uint16_t pos, uint16_t time)
{
   char cmd[24];
   if (pos < 500) pos = 500;
   if (pos > 2500) pos = 2500;
   if (time > 9999) time = 9999;
   snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!", id, pos, time);
   ZP15D_SendRaw(cmd);
}
//if integer
static uint8_t ZP15D_IsDigit(char ch)
{
   return (ch >= '0' && ch <= '9');
}
//parse the number starting from the current positon (At)
static uint8_t ZP15D_ParseNumberAt(const char *str, uint16_t *value)
{
   uint32_t num = 0;
   uint8_t has_digit = 0;

   if (str == NULL || value == NULL) return 0;

   while (ZP15D_IsDigit(*str))
   {
      has_digit = 1;
      num = num * 10U + (uint32_t)(*str - '0');
      if (num > 65535U)
      {
         return 0;
      }
      str++;
   }

   if (has_digit == 0)
   {
      return 0;
   }

   *value = (uint16_t)num;
   return 1;
}
//return the number after the target key
uint8_t ZP15D_ParseNumberAfter(const char *buf, const char *key, uint16_t *value)
{
   const char *p;

   if (buf == NULL || key == NULL || value == NULL) return 0;

   p = strstr(buf, key);
   if (p == NULL)
   {
      return 0;
   }

   p += strlen(key);
   while (*p != '\0' && ZP15D_IsDigit(*p) == 0)
   {
      p++;
   }

   return ZP15D_ParseNumberAt(p, value);
}
//return the last array
uint8_t ZP15D_ParseReturnNumber(const char *buf, uint16_t *value)
{
   const char *p;
   const char *last_number = NULL;

   if (buf == NULL || value == NULL) return 0;

   for (p = buf; *p != '\0'; p++)
   {
      if (ZP15D_IsDigit(*p) && (p == buf || ZP15D_IsDigit(*(p - 1)) == 0))
      {
         last_number = p;
      }
   }

   if (last_number == NULL)
   {
      return 0;
   }

   return ZP15D_ParseNumberAt(last_number, value);
}

static uint8_t ZP15D_ParseVersionNumber(const char *buf, uint16_t *version)
{
   const char *p;
   uint16_t major = 0;
   uint16_t minor = 0;

   if (buf == NULL || version == NULL) return 0;

   p = strstr(buf, "PV");
   if (p == NULL)
   {
      return 0;
   }

   p += 2;
   if (ZP15D_ParseNumberAt(p, &major) == 0)
   {
      return 0;
   }

   while (ZP15D_IsDigit(*p))
   {
      p++;
   }

   if (*p == '.')
   {
      p++;
      if (ZP15D_ParseNumberAt(p, &minor) == 0)
      {
         minor = 0;
      }
   }

   *version = (uint16_t)(major * 100U + minor);
   return 1;
}

static uint8_t ZP15D_ParseVoltageX10(const char *str, uint16_t *voltage_x10)
{
   uint16_t integer = 0;
   uint16_t decimal = 0;

   if (str == NULL || voltage_x10 == NULL) return 0;

   while (*str != '\0' && ZP15D_IsDigit(*str) == 0)
   {
      str++;
   }

   if (ZP15D_ParseNumberAt(str, &integer) == 0)
   {
      return 0;
   }

   while (ZP15D_IsDigit(*str))
   {
      str++;
   }

   if (*str == '.')
   {
      str++;
      if (ZP15D_IsDigit(*str))
      {
         decimal = (uint16_t)(*str - '0');
      }
   }

   *voltage_x10 = (uint16_t)(integer * 10U + decimal);
   return 1;
}

//return the string
uint16_t ZP15D_ReadReturn(char* buf, uint16_t len, uint32_t timeout)
{
   uint8_t data;
   uint16_t i = 0;

   if (buf == NULL || len == 0) return 0;

   while (i < len - 1)
   {
      if (HAL_UART_Receive(&huart1, &data, 1, timeout) != HAL_OK)
      {
         break;
      }

      buf[i++] = (char)data;
      if (data == '!')
      {
         break;
      }
   }

   buf[i] = '\0';
   return i;
}
void ZP15D_ReadVersion(uint8_t id)
{
   char cmd[16];
   ZP15D_ClearRx();
   snprintf(cmd, sizeof(cmd), "#%03dPVER!", id);
   ZP15D_SendRaw(cmd);
}

uint8_t ZP15D_ReadVersionValue(uint8_t id, uint16_t *version, uint32_t timeout)
{
   char ret[32];

   if (version == NULL) return 0;

   ZP15D_ReadVersion(id);
   if (ZP15D_ReadReturn(ret, sizeof(ret), timeout) == 0)
   {
      return 0;
   }

   if (ZP15D_ParseVersionNumber(ret, version))
   {
      return 1;
   }
   return ZP15D_ParseReturnNumber(ret, version);
}

void ZP15D_ReadID(uint8_t id)
{
   char cmd[16];
   ZP15D_ClearRx();
   snprintf(cmd, sizeof(cmd), "#%03dPID!", id);
   ZP15D_SendRaw(cmd);
}

uint8_t ZP15D_ReadIDValue(uint8_t id, uint8_t *servo_id, uint32_t timeout)
{
   char ret[32];
   uint16_t value;

   if (servo_id == NULL) return 0;

   ZP15D_ReadID(id);
   if (ZP15D_ReadReturn(ret, sizeof(ret), timeout) == 0)
   {
      return 0;
   }

   if (ZP15D_ParseNumberAfter(ret, "PID", &value) == 0)
   {
      if (ZP15D_ParseReturnNumber(ret, &value) == 0)
      {
         return 0;
      }
   }

   if (value > 255U)
   {
      return 0;
   }

   *servo_id = (uint8_t)value;
   return 1;
}

void ZP15D_SetID(uint8_t old_id, uint8_t new_id)
{
   char cmd[20];
   if (new_id > 254) new_id = 254;
   snprintf(cmd, sizeof(cmd), "#%03dPID%03d!", old_id, new_id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_SetIDByBroadcast(uint8_t new_id)
{
   ZP15D_SetID(255, new_id);
}

void ZP15D_ReleaseTorque(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPULK!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_RestoreTorque(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPULR!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_ReadMode(uint8_t id)
{
   char cmd[16];
   ZP15D_ClearRx();
   snprintf(cmd, sizeof(cmd), "#%03dPMOD!", id);
   ZP15D_SendRaw(cmd);
}

uint8_t ZP15D_ReadModeValue(uint8_t id, uint8_t *mode, uint32_t timeout)
{
   char ret[32];
   uint16_t value;

   if (mode == NULL) return 0;

   ZP15D_ReadMode(id);
   if (ZP15D_ReadReturn(ret, sizeof(ret), timeout) == 0)
   {
      return 0;
   }

   if (ZP15D_ParseNumberAfter(ret, "PMOD", &value) == 0)
   {
      if (ZP15D_ParseReturnNumber(ret, &value) == 0)
      {
         return 0;
      }
   }

   if (value > 255U)
   {
      return 0;
   }

   *mode = (uint8_t)value;
   return 1;
}

void ZP15D_SetMode(uint8_t id, uint8_t mode)
{
   char cmd[16];
   if (mode < 1) mode = 1;
   if (mode > 8) mode = 8;
   snprintf(cmd, sizeof(cmd), "#%03dPMOD%d!", id, mode);
   ZP15D_SendRaw(cmd);
}

void ZP15D_ReadPosition(uint8_t id)
{
   char cmd[16];
   ZP15D_ClearRx();
   snprintf(cmd, sizeof(cmd), "#%03dPRAD!", id);
   ZP15D_SendRaw(cmd);
}

uint8_t ZP15D_ReadPositionValue(uint8_t id, uint16_t *position, uint32_t timeout)
{
   char ret[32];

   if (position == NULL) return 0; 

   ZP15D_ReadPosition(id);
   if (ZP15D_ReadReturn(ret, sizeof(ret), timeout) == 0)
   {
      return 0;
   }

   if (ZP15D_ParseNumberAfter(ret, "PRAD", position))
   {
      return 1;
   }
   if (ZP15D_ParseNumberAfter(ret, "P", position))
   {
      return 1;
   }
   return ZP15D_ParseReturnNumber(ret, position);
}

void ZP15D_Stop(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPDST!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_Pause(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPDPT!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_Continue(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPDCT!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_SetBaudRate(uint8_t id, uint8_t baud_index)
{
   char cmd[16];

   if (baud_index > 7)
   {
      baud_index = 7;
   }

   snprintf(cmd, sizeof(cmd), "#%03dPBD%d!", id, baud_index);
   ZP15D_SendRaw(cmd);
}

void ZP15D_CalibrateMiddle(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPSCK!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_SetStartPosition(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPCSD!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_DisableStartPosition(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPCSM!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_EnableStartPosition(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPCSR!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_SetMinPosition(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPSMI!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_SetMaxPosition(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPSMX!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_ResetExceptID(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPCLE0!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_ResetAll(uint8_t id)
{
   char cmd[16];
   snprintf(cmd, sizeof(cmd), "#%03dPCLE!", id);
   ZP15D_SendRaw(cmd);
}

void ZP15D_ReadTempVoltage(uint8_t id)
{
   char cmd[16];
   ZP15D_ClearRx();
   snprintf(cmd, sizeof(cmd), "#%03dPRTE!", id);
   ZP15D_SendRaw(cmd);
}

uint8_t ZP15D_ParseTempVoltageReturn(const char *ret, uint16_t *temperature, uint16_t *voltage_x10)
{
   const char *t;
   const char *dash;

   if (ret == NULL || temperature == NULL || voltage_x10 == NULL) return 0;

   t = strchr(ret, 'T');
   if (t == NULL)
   {
      return 0;
   }

   t++;
   while (*t != '\0' && ZP15D_IsDigit(*t) == 0)
   {
      t++;
   }

   if (ZP15D_ParseNumberAt(t, temperature) == 0)
   {
      return 0;
   }

   dash = strchr(t, '-');
   if (dash == NULL)
   {
      return 0;
   }

   return ZP15D_ParseVoltageX10(dash + 1, voltage_x10);
}

uint8_t ZP15D_ReadTempVoltageValue(uint8_t id, uint16_t *temperature, uint16_t *voltage_x10, uint32_t timeout)
{
   char ret[32];

   if (temperature == NULL || voltage_x10 == NULL) return 0;

   ZP15D_ReadTempVoltage(id);
   if (ZP15D_ReadReturn(ret, sizeof(ret), timeout) == 0)
   {
      return 0;
   }

   return ZP15D_ParseTempVoltageReturn(ret, temperature, voltage_x10);
}

uint8_t ZP15D_ReturnIsOK(const char *ret)
{
   if (ret == NULL)
   {
      return 0;
   }

   return strstr(ret, "OK") != NULL;
}

uint8_t ZP15D_ReadOK(uint32_t timeout)
{
   char ret[16];

   if (ZP15D_ReadReturn(ret, sizeof(ret), timeout) == 0)
   {
      return 0;
   }

   return ZP15D_ReturnIsOK(ret);
}
