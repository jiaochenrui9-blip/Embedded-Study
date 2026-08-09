#ifndef BUJINTEST_PD42S1_PRIVATE_H
#define BUJINTEST_PD42S1_PRIVATE_H

#include "PD42S1.h"

#define PD42S1_FRAME_HEAD 0xC5U
#define PD42S1_FRAME_TAIL 0x5CU
#define PD42S1_FRAME_MAX_LEN 32U
#define PD42S1_DEFAULT_TIMEOUT_MS 80U
#define PD42S1_ACK_OK 0x01U
#define PD42S1_REPLY_DATA_OFFSET 4U

#define PD42S1_CMD_READ_VERSION 0x20U
#define PD42S1_CMD_READ_PHASE_CURRENT 0x23U
#define PD42S1_CMD_READ_BUS_VOLTAGE 0x24U
#define PD42S1_CMD_READ_INPUT_PULSE 0x28U
#define PD42S1_CMD_READ_REALTIME_SPEED 0x29U
#define PD42S1_CMD_READ_REALTIME_POSITION 0x2AU
#define PD42S1_CMD_READ_POSITION_ERROR 0x2BU
#define PD42S1_CMD_READ_RUN_STATUS 0x2CU
#define PD42S1_CMD_READ_STALL_FLAG 0x2DU
#define PD42S1_CMD_READ_ENABLE_STATE 0x2FU
#define PD42S1_CMD_READ_ARRIVED_FLAG 0x30U
#define PD42S1_CMD_SET_MODE 0x62U
#define PD42S1_CMD_SET_POSITION_MAX_TORQUE 0x64U
#define PD42S1_CMD_OPEN_SPEED 0xE0U
#define PD42S1_CMD_OPEN_ABS_POSITION 0xE1U
#define PD42S1_CMD_OPEN_REL_POSITION 0xE2U
#define PD42S1_CMD_OPEN_PULSE 0xE3U
#define PD42S1_CMD_IO_START_STOP 0xE4U
#define PD42S1_CMD_TORQUE 0xF0U
#define PD42S1_CMD_SPEED 0xF1U
#define PD42S1_CMD_ABS_POSITION 0xF2U
#define PD42S1_CMD_REL_POSITION 0xF3U
#define PD42S1_CMD_PULSE 0xF4U
#define PD42S1_CMD_PULSE_WIDTH_POSITION 0xF5U
#define PD42S1_CMD_PULSE_WIDTH_TORQUE 0xF6U
#define PD42S1_CMD_PULSE_WIDTH_SPEED 0xF7U
#define PD42S1_CMD_CLEAR_POSITION 0xF8U
#define PD42S1_CMD_RELEASE_STALL 0xF9U
#define PD42S1_CMD_ENABLE 0xFAU
#define PD42S1_CMD_CLEAR_STATE 0xFBU
#define PD42S1_CMD_STOP 0xFCU

HAL_StatusTypeDef PD42S1_SendCommand(PD42S1_ManagerTypeDef *manager,
                                       uint8_t device_id,
                                       uint8_t code,
                                      const uint8_t *data,
                                      uint8_t data_len,
                                      uint8_t *rx,
                                      uint16_t *rx_len,
                                      uint16_t rx_max,
                                      uint32_t timeout_ms);
HAL_StatusTypeDef PD42S1_ReadData(PD42S1_ManagerTypeDef *manager,
                                   uint8_t device_id,
                                   uint8_t code,
                                   uint8_t *rx,
                                   uint16_t rx_size,
                                   uint8_t data_len);

void PD42S1_WriteU16BE(uint8_t *dst, uint16_t value);
void PD42S1_WriteU32BE(uint8_t *dst, uint32_t value);
void PD42S1_WriteI32BE(uint8_t *dst, int32_t value);
void PD42S1_WriteFloatBE(uint8_t *dst, float value);

uint16_t PD42S1_ReadU16BE(const uint8_t *src);
int16_t PD42S1_ReadI16BE(const uint8_t *src);
uint32_t PD42S1_ReadU32BE(const uint8_t *src);
int32_t PD42S1_ReadI32BE(const uint8_t *src);
float PD42S1_ReadFloatBE(const uint8_t *src);

#endif /* BUJINTEST_PD42S1_PRIVATE_H */
