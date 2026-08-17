#include "IST8310.h"

#include <stddef.h>
#include <string.h>

#define IST8310_I2C_ADDRESS       (0x0Eu << 1u)
#define IST8310_WHO_AM_I_REG      0x00u
#define IST8310_WHO_AM_I_VALUE    0x10u
#define IST8310_DATA_REG          0x03u
#define IST8310_CNTL1_REG         0x0Au
#define IST8310_CNTL2_REG         0x0Bu
#define IST8310_AVGCNTL_REG       0x41u
#define IST8310_PDCNTL_REG        0x42u

#define IST8310_SINGLE_MEASURE    0x01u
#define IST8310_DRDY_CONFIG       0x0Cu
#define IST8310_AVERAGE_16        0x24u
#define IST8310_PULSE_NORMAL      0xC0u
#define IST8310_UT_PER_LSB        0.3f
#define IST8310_I2C_TIMEOUT_MS    100u
#define IST8310_DRDY_TIMEOUT_MS   20u
#define IST8310_OFFLINE_TIMEOUT_MS 50u

static I2C_HandleTypeDef *IST8310_I2C;
static IST8310_Data_t IST8310_DATA;
static uint8_t IST8310_DMA_BUFFER[6];
static uint8_t IST8310_TRIGGER_VALUE = IST8310_SINGLE_MEASURE;
static volatile uint8_t IST8310_SAMPLE_READY;
static volatile uint8_t IST8310_SAMPLE_BUFFER[6];

typedef enum
{
    IST8310_STATE_IDLE = 0,
    IST8310_STATE_TRIGGERING,
    IST8310_STATE_WAITING_DRDY,
    IST8310_STATE_READING_DATA,
    IST8310_STATE_ERROR
} IST8310_State_t;

static volatile IST8310_State_t IST8310_STATE = IST8310_STATE_IDLE;

static HAL_StatusTypeDef IST8310_StartTriggerDMA(void)
{
    HAL_StatusTypeDef status;

    IST8310_STATE = IST8310_STATE_TRIGGERING;
    status = HAL_I2C_Mem_Write_DMA(IST8310_I2C, IST8310_I2C_ADDRESS,
                                   IST8310_CNTL1_REG, I2C_MEMADD_SIZE_8BIT,
                                   &IST8310_TRIGGER_VALUE, 1u);
    if (status != HAL_OK)
    {
        IST8310_STATE = IST8310_STATE_ERROR;
        IST8310_DATA.online = 0u;
    }
    return status;
}

static HAL_StatusTypeDef IST8310_StartDataReadDMA(void)
{
    HAL_StatusTypeDef status;

    IST8310_STATE = IST8310_STATE_READING_DATA;
    status = HAL_I2C_Mem_Read_DMA(IST8310_I2C, IST8310_I2C_ADDRESS,
                                  IST8310_DATA_REG, I2C_MEMADD_SIZE_8BIT,
                                  IST8310_DMA_BUFFER, sizeof(IST8310_DMA_BUFFER));
    if (status != HAL_OK)
    {
        IST8310_STATE = IST8310_STATE_ERROR;
        IST8310_DATA.online = 0u;
    }
    return status;
}

static void IST8310_StoreRaw(const uint8_t buffer[6])
{
    for (uint8_t i = 0u; i < 3u; i++)
    {
        IST8310_DATA.raw[i] =
            (int16_t)(((uint16_t)buffer[(2u * i) + 1u] << 8u) |
                      buffer[2u * i]);
        IST8310_DATA.mag_uT[i] =
            ((float)IST8310_DATA.raw[i] * IST8310_UT_PER_LSB) -
            IST8310_DATA.offset_uT[i];
    }
    IST8310_DATA.online = 1u;
    IST8310_DATA.last_update_tick = HAL_GetTick();
}

uint8_t IST8310_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
    if (IST8310_I2C == NULL || data == NULL || length == 0u)
    {
        return IST8310_ERR_NULL;
    }
    if (IST8310_STATE != IST8310_STATE_IDLE)
    {
        return IST8310_ERR_BUSY;
    }

    return (HAL_I2C_Mem_Read(IST8310_I2C, IST8310_I2C_ADDRESS,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             data, length, IST8310_I2C_TIMEOUT_MS) == HAL_OK) ?
           IST8310_OK : IST8310_ERR_I2C;
}

uint8_t IST8310_WriteRegister(uint8_t reg, uint8_t data)
{
    if (IST8310_I2C == NULL)
    {
        return IST8310_ERR_NULL;
    }
    if (IST8310_STATE != IST8310_STATE_IDLE)
    {
        return IST8310_ERR_BUSY;
    }

    return (HAL_I2C_Mem_Write(IST8310_I2C, IST8310_I2C_ADDRESS,
                              reg, I2C_MEMADD_SIZE_8BIT,
                              &data, 1u, IST8310_I2C_TIMEOUT_MS) == HAL_OK) ?
           IST8310_OK : IST8310_ERR_I2C;
}

static uint8_t IST8310_WriteAndVerify(uint8_t reg, uint8_t value)
{
    uint8_t readback;
    uint8_t status = IST8310_WriteRegister(reg, value);

    if (status != IST8310_OK)
    {
        return status;
    }
    HAL_Delay(1u);
    status = IST8310_ReadRegisters(reg, &readback, 1u);
    if (status != IST8310_OK)
    {
        return status;
    }
    return (readback == value) ? IST8310_OK : IST8310_ERR_CONFIG;
}

uint8_t IST8310_ReadID(uint8_t *id)
{
    uint8_t status;

    if (id == NULL)
    {
        return IST8310_ERR_NULL;
    }

    status = IST8310_ReadRegisters(IST8310_WHO_AM_I_REG, id, 1u);
    if (status == IST8310_OK)
    {
        IST8310_DATA.id = *id;
    }
    return status;
}

uint8_t IST8310_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;
    uint8_t status;

    if (hi2c == NULL)
    {
        return IST8310_ERR_NULL;
    }

    IST8310_I2C = hi2c;
    memset(&IST8310_DATA, 0, sizeof(IST8310_DATA));
    IST8310_SAMPLE_READY = 0u;
    IST8310_STATE = IST8310_STATE_IDLE;

    HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_RESET);
    HAL_Delay(50u);
    HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_SET);
    HAL_Delay(50u);

    status = IST8310_ReadID(&id);
    if (status != IST8310_OK)
    {
        return status;
    }
    if (id != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_ERR_ID;
    }

    status = IST8310_WriteAndVerify(IST8310_CNTL2_REG, IST8310_DRDY_CONFIG);
    if (status != IST8310_OK)
    {
        return status;
    }
    status = IST8310_WriteAndVerify(IST8310_AVGCNTL_REG, IST8310_AVERAGE_16);
    if (status != IST8310_OK)
    {
        return status;
    }
    status = IST8310_WriteAndVerify(IST8310_PDCNTL_REG, IST8310_PULSE_NORMAL);
    if (status != IST8310_OK)
    {
        return status;
    }

    IST8310_DATA.initialized = 1u;
    return IST8310_OK;
}

HAL_StatusTypeDef IST8310_StartSample(void)
{
    if (IST8310_I2C == NULL || IST8310_DATA.initialized == 0u)
    {
        return HAL_ERROR;
    }

    if (IST8310_STATE == IST8310_STATE_ERROR)
    {
        IST8310_STATE = IST8310_STATE_IDLE;
    }

    if (IST8310_STATE != IST8310_STATE_IDLE || IST8310_SAMPLE_READY != 0u)
    {
        return HAL_BUSY;
    }

    return IST8310_StartTriggerDMA();
}

uint8_t IST8310_IsSampleReady(void)
{
    return IST8310_SAMPLE_READY;
}

void IST8310_ProcessSample(void)
{
    uint8_t sample[6];
    uint32_t primask;

    if (IST8310_SAMPLE_READY == 0u)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(sample, (const void *)IST8310_SAMPLE_BUFFER, sizeof(sample));
    if (primask == 0u)
    {
        __enable_irq();
    }

    IST8310_StoreRaw(sample);
    IST8310_SAMPLE_READY = 0u;
}

void IST8310_Update(void)
{
    if (IST8310_SAMPLE_READY != 0u)
    {
        IST8310_ProcessSample();
    }

    if (IST8310_STATE == IST8310_STATE_WAITING_DRDY &&
        (uint32_t)(HAL_GetTick() - IST8310_DATA.drdy_wait_start_tick) >= IST8310_DRDY_TIMEOUT_MS)
    {
        IST8310_STATE = IST8310_STATE_ERROR;
        IST8310_DATA.online = 0u;
    }

    if (IST8310_DATA.online != 0u &&
        (uint32_t)(HAL_GetTick() - IST8310_DATA.last_update_tick) >= IST8310_OFFLINE_TIMEOUT_MS)
    {
        IST8310_DATA.online = 0u;
    }
}

void IST8310_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != IST8310_I2C)
    {
        return;
    }

    if (IST8310_STATE == IST8310_STATE_READING_DATA)
    {
        memcpy((void *)IST8310_SAMPLE_BUFFER, IST8310_DMA_BUFFER,
               sizeof(IST8310_DMA_BUFFER));
        IST8310_STATE = IST8310_STATE_IDLE;
        __DMB();
        IST8310_SAMPLE_READY = 1u;
    }
}

void IST8310_DRDY_IRQHandler(void)
{
    if (IST8310_STATE == IST8310_STATE_WAITING_DRDY)
    {
        (void)IST8310_StartDataReadDMA();
    }
}

void IST8310_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == IST8310_I2C && IST8310_STATE == IST8310_STATE_TRIGGERING)
    {
        IST8310_STATE = IST8310_STATE_WAITING_DRDY;
        IST8310_DATA.drdy_wait_start_tick = HAL_GetTick();
    }
}

void IST8310_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == IST8310_I2C)
    {
        IST8310_STATE = IST8310_STATE_ERROR;
        IST8310_SAMPLE_READY = 0u;
        IST8310_DATA.online = 0u;
    }
}

void IST8310_GetData(IST8310_Data_t *data)
{
    if (data != NULL)
    {
        *data = IST8310_DATA;
    }
}

uint8_t IST8310_GetValidData(IST8310_Data_t *data)
{
    if (data == NULL || IST8310_DATA.online == 0u)
    {
        return 0u;
    }

    *data = IST8310_DATA;
    return 1u;
}

void IST8310_SetOffset(float x_uT, float y_uT, float z_uT)
{
    IST8310_DATA.offset_uT[0] = x_uT;
    IST8310_DATA.offset_uT[1] = y_uT;
    IST8310_DATA.offset_uT[2] = z_uT;
}
