#ifndef IST8310_H
#define IST8310_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    IST8310_OK = 0,
    IST8310_ERR_NULL = 1,
    IST8310_ERR_ID = 2,
    IST8310_ERR_I2C = 3,
    IST8310_ERR_CONFIG = 4,
    IST8310_ERR_NOT_INIT = 5,
    IST8310_ERR_BUSY = 6
} IST8310_Status_t;

typedef struct
{
    int16_t raw[3];
    float mag_uT[3];
    float offset_uT[3];
    uint8_t id;
    uint8_t initialized;
    uint8_t data_ready;
} IST8310_Data_t;

uint8_t IST8310_Init(I2C_HandleTypeDef *hi2c);
uint8_t IST8310_ReadID(uint8_t *id);
uint8_t IST8310_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length);
uint8_t IST8310_WriteRegister(uint8_t reg, uint8_t data);
HAL_StatusTypeDef IST8310_StartSample(void);
uint8_t IST8310_IsSampleReady(void);
void IST8310_ProcessSample(void);
void IST8310_GetData(IST8310_Data_t *data);
void IST8310_SetOffset(float x_uT, float y_uT, float z_uT);

void IST8310_DRDY_IRQHandler(void);
void IST8310_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void IST8310_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void IST8310_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif
