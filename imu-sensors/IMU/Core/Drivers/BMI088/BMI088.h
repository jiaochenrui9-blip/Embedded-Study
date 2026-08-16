#ifndef BMI088_H
#define BMI088_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_ACC_RANGE_3G          0x00u
#define BMI088_ACC_RANGE_6G          0x01u
#define BMI088_ACC_RANGE_12G         0x02u
#define BMI088_ACC_RANGE_24G         0x03u

#define BMI088_GYRO_RANGE_2000DPS    0x00u
#define BMI088_GYRO_RANGE_1000DPS    0x01u
#define BMI088_GYRO_RANGE_500DPS     0x02u
#define BMI088_GYRO_RANGE_250DPS     0x03u
#define BMI088_GYRO_RANGE_125DPS     0x04u

#ifndef BMI088_ACC_RANGE_SETTING
#define BMI088_ACC_RANGE_SETTING     BMI088_ACC_RANGE_6G
#endif

#ifndef BMI088_GYRO_RANGE_SETTING
#define BMI088_GYRO_RANGE_SETTING    BMI088_GYRO_RANGE_2000DPS
#endif

typedef enum
{
    BMI088_OK = 0,
    BMI088_ERR_ACC_ID = 1,
    BMI088_ERR_GYRO_ID = 2,
    BMI088_ERR_NULL = 3,
    BMI088_ERR_SPI = 4,
    BMI088_ERR_CONFIG = 5,
    BMI088_ERR_NOT_INIT = 6,
    BMI088_ERR_DATA = 7,
    BMI088_ERR_BUSY = 8
} BMI088_Status_t;

typedef enum
{
    BMI088_STATE_IDLE = 0,
    BMI088_STATE_READING_ACC,
    BMI088_STATE_READING_GYRO,
    BMI088_STATE_ERROR
} BMI088_State_t;

typedef struct
{
    int16_t acc_raw[3];
    int16_t gyro_raw[3];

    float acc_g[3];
    float gyro_dps[3];
    float gyro_offset[3];
    float temperature;

    uint8_t acc_id;
    uint8_t gyro_id;
    uint8_t initialized;
} BMI088_Data_t;

uint8_t BMI088_Init(SPI_HandleTypeDef *hspi);
uint8_t BMI088_ReadID(uint8_t *acc_id, uint8_t *gyro_id);
uint8_t BMI088_ReadTemperature(float *temp);
void BMI088_GetData(BMI088_Data_t *data);
uint8_t BMI088_CalibrateGyro(uint16_t samples);

HAL_StatusTypeDef BMI088_StartSample(void);
uint8_t BMI088_IsSampleReady(void);
void BMI088_ClearSampleReady(void);
void BMI088_ProcessSample(void);
void BMI088_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void BMI088_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif
