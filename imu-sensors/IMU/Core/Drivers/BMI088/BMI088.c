#include "BMI088.h"

#include <string.h>

#define BMI088_ACC_CHIP_ID_REG       0x00u
#define BMI088_ACC_CHIP_ID_VALUE     0x1Eu
#define BMI088_ACC_DATA_REG          0x12u
#define BMI088_ACC_TEMP_REG          0x22u
#define BMI088_ACC_CONF_REG          0x40u
#define BMI088_ACC_RANGE_REG         0x41u
#define BMI088_ACC_PWR_CONF_REG      0x7Cu
#define BMI088_ACC_PWR_CTRL_REG      0x7Du
#define BMI088_ACC_SOFTRESET_REG     0x7Eu

#define BMI088_GYRO_CHIP_ID_REG      0x00u
#define BMI088_GYRO_CHIP_ID_VALUE    0x0Fu
#define BMI088_GYRO_DATA_REG         0x02u
#define BMI088_GYRO_RANGE_REG        0x0Fu
#define BMI088_GYRO_BANDWIDTH_REG    0x10u
#define BMI088_GYRO_LPM1_REG         0x11u
#define BMI088_GYRO_SOFTRESET_REG    0x14u

#define BMI088_SOFTRESET_VALUE       0xB6u
#define BMI088_ACC_PWR_ACTIVE        0x00u
#define BMI088_ACC_ENABLE            0x04u
#define BMI088_ACC_CONF_1600HZ_NORMAL 0xACu
#define BMI088_GYRO_NORMAL_MODE      0x00u
#define BMI088_GYRO_1000HZ_116HZ     0x02u

#define BMI088_SPI_READ              0x80u
#define BMI088_SPI_WRITE             0x7Fu
#define BMI088_SPI_TIMEOUT_MS         100u

static SPI_HandleTypeDef *BMI088_SPI;
static BMI088_Data_t BMI088_DATA;
static uint8_t BMI088_IO_OK;
static volatile BMI088_State_t BMI088_STATE = BMI088_STATE_IDLE;
static volatile uint8_t BMI088_SAMPLE_READY;
//选择加速度计
static void BMI088_Acc_Select(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET);
}
//关闭加速度计
static void BMI088_Acc_Deselect(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
}
//选择陀螺仪
static void BMI088_Gyro_Select(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
}
//关闭陀螺仪
static void BMI088_Gyro_Deselect(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}
//加速度计读取寄存器
static void BMI088_Acc_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t address = (uint8_t)(reg | BMI088_SPI_READ);
    uint8_t tx_dummy = 0xFFu;
    uint8_t rx_dummy = 0u;

    if (BMI088_SPI == NULL || buf == NULL || len == 0u)
    {
        BMI088_IO_OK = 0u;
        return;
    }

    BMI088_Acc_Select();
    //多读一个字节dummy
    if (HAL_SPI_Transmit(BMI088_SPI, &address, 1u, BMI088_SPI_TIMEOUT_MS) != HAL_OK ||
        HAL_SPI_TransmitReceive(BMI088_SPI, &tx_dummy, &rx_dummy, 1u,
                                BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        BMI088_IO_OK = 0u;
    }
    for (uint16_t i = 0u; i < len && BMI088_IO_OK != 0u; i++)
    {
        if (HAL_SPI_TransmitReceive(BMI088_SPI, &tx_dummy, &buf[i], 1u,
                                    BMI088_SPI_TIMEOUT_MS) != HAL_OK)
        {
            BMI088_IO_OK = 0u;
        }
    }
    BMI088_Acc_Deselect();
}
//加速度计读一个字节
static uint8_t BMI088_Acc_ReadReg(uint8_t reg)
{
    uint8_t value = 0u;
    BMI088_Acc_ReadRegs(reg, &value, 1u);
    return value;
}
//加速度计写一个字节
static void BMI088_Acc_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {(uint8_t)(reg & BMI088_SPI_WRITE), data};

    if (BMI088_SPI == NULL)
    {
        BMI088_IO_OK = 0u;
        return;
    }

    BMI088_Acc_Select();
    if (HAL_SPI_Transmit(BMI088_SPI, tx, 2u, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        BMI088_IO_OK = 0u;
    }
    BMI088_Acc_Deselect();
}
//陀螺仪读取多个字节
static void BMI088_Gyro_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t address = (uint8_t)(reg | BMI088_SPI_READ);
    uint8_t tx_dummy = 0xFFu;

    if (BMI088_SPI == NULL || buf == NULL || len == 0u)
    {
        BMI088_IO_OK = 0u;
        return;
    }

    BMI088_Gyro_Select();
    //发送地址
    if (HAL_SPI_Transmit(BMI088_SPI, &address, 1u, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        BMI088_IO_OK = 0u;
    }
    for (uint16_t i = 0u; i < len && BMI088_IO_OK != 0u; i++)
    {
        if (HAL_SPI_TransmitReceive(BMI088_SPI, &tx_dummy, &buf[i], 1u,
                                    BMI088_SPI_TIMEOUT_MS) != HAL_OK)
        {
            BMI088_IO_OK = 0u;
        }
    }
    BMI088_Gyro_Deselect();
}
//陀螺仪读取一个字节
static uint8_t BMI088_Gyro_ReadReg(uint8_t reg)
{
    uint8_t value = 0u;
    BMI088_Gyro_ReadRegs(reg, &value, 1u);
    return value;
}
//陀螺仪写一个字节
static void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {(uint8_t)(reg & BMI088_SPI_WRITE), data};

    if (BMI088_SPI == NULL)
    {
        BMI088_IO_OK = 0u;
        return;
    }

    BMI088_Gyro_Select();
    if (HAL_SPI_Transmit(BMI088_SPI, tx, 2u, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        BMI088_IO_OK = 0u;
    }
    BMI088_Gyro_Deselect();
}
//设置灵敏度
static float BMI088_AccelSensitivity(void)
{
    switch (BMI088_ACC_RANGE_SETTING)
    {
    case BMI088_ACC_RANGE_3G:  return 10920.0f;
    case BMI088_ACC_RANGE_12G: return 2730.0f;
    case BMI088_ACC_RANGE_24G: return 1365.0f;
    case BMI088_ACC_RANGE_6G:
    default:                   return 5460.0f;
    }
}

static float BMI088_GyroSensitivity(void)
{
    switch (BMI088_GYRO_RANGE_SETTING)
    {
    case BMI088_GYRO_RANGE_125DPS:  return 262.144f;
    case BMI088_GYRO_RANGE_250DPS:  return 131.072f;
    case BMI088_GYRO_RANGE_500DPS:  return 65.536f;
    case BMI088_GYRO_RANGE_1000DPS: return 32.768f;
    case BMI088_GYRO_RANGE_2000DPS:
    default:                        return 16.384f;
    }
}
//读取加速度计和陀螺仪的ID
uint8_t BMI088_ReadID(uint8_t *acc_id, uint8_t *gyro_id)
{
    if (BMI088_SPI == NULL || acc_id == NULL || gyro_id == NULL)
    {
        return BMI088_ERR_NULL;
    }
    if (BMI088_STATE != BMI088_STATE_IDLE)
    {
        return BMI088_ERR_BUSY;
    }

    BMI088_IO_OK = 1u;
    *acc_id = BMI088_Acc_ReadReg(BMI088_ACC_CHIP_ID_REG);
    *gyro_id = BMI088_Gyro_ReadReg(BMI088_GYRO_CHIP_ID_REG);
    BMI088_DATA.acc_id = *acc_id;
    BMI088_DATA.gyro_id = *gyro_id;

    return (BMI088_IO_OK != 0u) ? BMI088_OK : BMI088_ERR_SPI;
}
//初始化陀螺仪和加速度计
uint8_t BMI088_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t status;
    uint8_t acc_id;
    uint8_t gyro_id;

    if (hspi == NULL)
    {
        return BMI088_ERR_NULL;
    }
    if (BMI088_STATE != BMI088_STATE_IDLE)
    {
        return BMI088_ERR_BUSY;
    }

    BMI088_SPI = hspi;
    memset(&BMI088_DATA, 0, sizeof(BMI088_DATA));
    BMI088_STATE = BMI088_STATE_IDLE;
    BMI088_SAMPLE_READY = 0u;
    BMI088_IO_OK = 1u;
    BMI088_Acc_Deselect();
    BMI088_Gyro_Deselect();
    HAL_Delay(5u);

    /* The accelerometer powers up in I2C mode. This dummy read creates the
       required CS rising edge and switches it to SPI mode. */
    (void)BMI088_Acc_ReadReg(BMI088_ACC_CHIP_ID_REG);//dummy加速度计，改为SPI模式
    //软复位加速度计，恢复默认状态
    BMI088_Acc_WriteReg(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_VALUE);
    HAL_Delay(2u);
    (void)BMI088_Acc_ReadReg(BMI088_ACC_CHIP_ID_REG);
    //软复位陀螺仪，恢复默认状态
    BMI088_Gyro_WriteReg(BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_VALUE);
    HAL_Delay(30u);
    //读取加速度计和陀螺仪ID
    status = BMI088_ReadID(&acc_id, &gyro_id);
    if (status != BMI088_OK)
    {
        return status;
    }
    //判断ID是否正确
    if (acc_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_ERR_ACC_ID;
    }
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_ERR_GYRO_ID;
    }
    //配置加速度计
    BMI088_Acc_WriteReg(BMI088_ACC_PWR_CTRL_REG,BMI088_ACC_ENABLE);
    HAL_Delay(1u);   // > 450 us
    BMI088_Acc_WriteReg(BMI088_ACC_PWR_CONF_REG,BMI088_ACC_PWR_ACTIVE);
    HAL_Delay(5u);
    BMI088_Acc_WriteReg(BMI088_ACC_CONF_REG, BMI088_ACC_CONF_1600HZ_NORMAL);//配置采样率和滤波
    HAL_Delay(1u);
    BMI088_Acc_WriteReg(BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_SETTING);//配置加速度计量程
    HAL_Delay(5u);
    //配置陀螺仪
    BMI088_Gyro_WriteReg(BMI088_GYRO_LPM1_REG, BMI088_GYRO_NORMAL_MODE);//配置成正常模式
    HAL_Delay(30u);
    BMI088_Gyro_WriteReg(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_SETTING);//设置陀螺仪量程
    HAL_Delay(1u);
    BMI088_Gyro_WriteReg(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_1000HZ_116HZ);//设置陀螺仪带宽/采样率
    HAL_Delay(5u);
    //判断是否是就绪状态
    if (BMI088_IO_OK == 0u)
    {
        return BMI088_ERR_SPI;
    }
    //回读配置寄存器，确认真的写进去了
    if (BMI088_Acc_ReadReg(BMI088_ACC_CONF_REG) != BMI088_ACC_CONF_1600HZ_NORMAL ||
        (BMI088_Acc_ReadReg(BMI088_ACC_RANGE_REG) & 0x03u) != BMI088_ACC_RANGE_SETTING ||
        (BMI088_Gyro_ReadReg(BMI088_GYRO_RANGE_REG) & 0x07u) != BMI088_GYRO_RANGE_SETTING ||
        (BMI088_Gyro_ReadReg(BMI088_GYRO_BANDWIDTH_REG) & 0x07u) != BMI088_GYRO_1000HZ_116HZ)
    {
        return (BMI088_IO_OK != 0u) ? BMI088_ERR_CONFIG : BMI088_ERR_SPI;
    }
    //初始化完成
    BMI088_DATA.initialized = 1u;
    return BMI088_OK;
}
//读取陀螺仪原始数据
static uint8_t BMI088_ReadGyroRawBlocking(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    if (gx == NULL || gy == NULL || gz == NULL)
    {
        return BMI088_ERR_NULL;
    }
    if (BMI088_DATA.initialized == 0u)
    {
        return BMI088_ERR_NOT_INIT;
    }
    if (BMI088_STATE != BMI088_STATE_IDLE)
    {
        return BMI088_ERR_BUSY;
    }

    BMI088_IO_OK = 1u;
    BMI088_Gyro_ReadRegs(BMI088_GYRO_DATA_REG, buf, sizeof(buf));
    if (BMI088_IO_OK == 0u)
    {
        return BMI088_ERR_SPI;
    }

    *gx = (int16_t)(((uint16_t)buf[1] << 8u) | buf[0]);
    *gy = (int16_t)(((uint16_t)buf[3] << 8u) | buf[2]);
    *gz = (int16_t)(((uint16_t)buf[5] << 8u) | buf[4]);
    BMI088_DATA.gyro_raw[0] = *gx;
    BMI088_DATA.gyro_raw[1] = *gy;
    BMI088_DATA.gyro_raw[2] = *gz;
    return BMI088_OK;
}
//读取温度
uint8_t BMI088_ReadTemperature(float *temp)
{
    uint8_t buf[2];
    uint16_t temp_uint11;
    int16_t temp_int11;

    if (temp == NULL)
    {
        return BMI088_ERR_NULL;
    }
    if (BMI088_DATA.initialized == 0u)
    {
        return BMI088_ERR_NOT_INIT;
    }
    if (BMI088_STATE != BMI088_STATE_IDLE)
    {
        return BMI088_ERR_BUSY;
    }

    BMI088_IO_OK = 1u;
    BMI088_Acc_ReadRegs(BMI088_ACC_TEMP_REG, buf, sizeof(buf));
    if (BMI088_IO_OK == 0u)
    {
        return BMI088_ERR_SPI;
    }
    if (buf[0] == 0x80u)
    {
        return BMI088_ERR_DATA;
    }

    temp_uint11 = (uint16_t)(((uint16_t)buf[0] << 3u) | (buf[1] >> 5u));
    temp_int11 = (temp_uint11 > 1023u) ?
                 (int16_t)(temp_uint11 - 2048u) : (int16_t)temp_uint11;
    *temp = ((float)temp_int11 * 0.125f) + 23.0f;
    BMI088_DATA.temperature = *temp;
    return BMI088_OK;
}
static uint8_t BMI088_ProcessRawSample(const int16_t acc_raw[3],
                                       const int16_t gyro_raw[3])
{
    float acc_sensitivity;
    float gyro_sensitivity;

    if (acc_raw == NULL || gyro_raw == NULL)
    {
        return BMI088_ERR_NULL;
    }
    if (BMI088_DATA.initialized == 0u)
    {
        return BMI088_ERR_NOT_INIT;
    }
    if (BMI088_STATE != BMI088_STATE_IDLE)
    {
        return BMI088_ERR_BUSY;
    }

    acc_sensitivity = BMI088_AccelSensitivity();
    gyro_sensitivity = BMI088_GyroSensitivity();
    for (uint8_t i = 0u; i < 3u; i++)
    {
        BMI088_DATA.acc_raw[i] = acc_raw[i];
        BMI088_DATA.gyro_raw[i] = gyro_raw[i];
        BMI088_DATA.acc_g[i] = (float)acc_raw[i] / acc_sensitivity;
        BMI088_DATA.gyro_dps[i] =
            ((float)gyro_raw[i] / gyro_sensitivity) -
            BMI088_DATA.gyro_offset[i];
    }

    return BMI088_OK;
}

void BMI088_GetData(BMI088_Data_t *data)
{
    if (data != NULL)
    {
        *data = BMI088_DATA;
    }
}

uint8_t BMI088_CalibrateGyro(uint16_t samples)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float sensitivity;
    int16_t raw[3];
    uint8_t status;

    if (BMI088_DATA.initialized == 0u)
    {
        return BMI088_ERR_NOT_INIT;
    }
    if (samples == 0u)
    {
        return BMI088_ERR_NULL;
    }

    sensitivity = BMI088_GyroSensitivity();
    for (uint16_t i = 0u; i < samples; i++)
    {
        status = BMI088_ReadGyroRawBlocking(&raw[0], &raw[1], &raw[2]);
        if (status != BMI088_OK)
        {
            return status;
        }
        sum[0] += (float)raw[0] / sensitivity;
        sum[1] += (float)raw[1] / sensitivity;
        sum[2] += (float)raw[2] / sensitivity;
        HAL_Delay(2u);
    }

    BMI088_DATA.gyro_offset[0] = sum[0] / (float)samples;
    BMI088_DATA.gyro_offset[1] = sum[1] / (float)samples;
    BMI088_DATA.gyro_offset[2] = sum[2] / (float)samples;
    return BMI088_OK;
}

#define BMI088_ACC_DMA_TRANSFER_LEN  8u
#define BMI088_GYRO_DMA_TRANSFER_LEN 7u

static uint8_t BMI088_ACC_TX[BMI088_ACC_DMA_TRANSFER_LEN];
static uint8_t BMI088_GYRO_TX[BMI088_GYRO_DMA_TRANSFER_LEN];
static volatile uint8_t BMI088_ACC_RX[BMI088_ACC_DMA_TRANSFER_LEN];
static volatile uint8_t BMI088_GYRO_RX[BMI088_GYRO_DMA_TRANSFER_LEN];
static volatile uint8_t BMI088_SAMPLE_ACC_RX[BMI088_ACC_DMA_TRANSFER_LEN];
static volatile uint8_t BMI088_SAMPLE_GYRO_RX[BMI088_GYRO_DMA_TRANSFER_LEN];

static void BMI088_DMA_DeselectAll(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

static void BMI088_DMA_Fail(void)
{
    BMI088_DMA_DeselectAll();
    BMI088_STATE = BMI088_STATE_ERROR;
    BMI088_SAMPLE_READY = 0u;
}

HAL_StatusTypeDef BMI088_StartSample(void)
{
    HAL_StatusTypeDef status;

    if (BMI088_SPI == NULL || BMI088_DATA.initialized == 0u)
    {
        return HAL_ERROR;
    }
    if (BMI088_STATE == BMI088_STATE_ERROR)
    {
        BMI088_STATE = BMI088_STATE_IDLE;
    }
    if (BMI088_STATE != BMI088_STATE_IDLE || BMI088_SAMPLE_READY != 0u)
    {
        return HAL_BUSY;
    }

    memset(BMI088_ACC_TX, 0xFF, sizeof(BMI088_ACC_TX));
    memset(BMI088_GYRO_TX, 0xFF, sizeof(BMI088_GYRO_TX));
    BMI088_ACC_TX[0] = BMI088_ACC_DATA_REG | BMI088_SPI_READ;
    BMI088_GYRO_TX[0] = BMI088_GYRO_DATA_REG | BMI088_SPI_READ;

    BMI088_SAMPLE_READY = 0u;
    BMI088_STATE = BMI088_STATE_READING_ACC;
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive_DMA(BMI088_SPI, BMI088_ACC_TX,
                                         (uint8_t *)BMI088_ACC_RX,
                                         BMI088_ACC_DMA_TRANSFER_LEN);
    if (status != HAL_OK)
    {
        BMI088_DMA_Fail();
    }
    return status;
}

uint8_t BMI088_IsSampleReady(void)
{
    return BMI088_SAMPLE_READY;
}

void BMI088_ClearSampleReady(void)
{
    BMI088_SAMPLE_READY = 0u;
}

void BMI088_ProcessSample(void)
{
    uint8_t acc_copy[BMI088_ACC_DMA_TRANSFER_LEN];
    uint8_t gyro_copy[BMI088_GYRO_DMA_TRANSFER_LEN];
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    uint32_t primask;

    if (BMI088_SAMPLE_READY == 0u)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(acc_copy, (const void *)BMI088_SAMPLE_ACC_RX, sizeof(acc_copy));
    memcpy(gyro_copy, (const void *)BMI088_SAMPLE_GYRO_RX, sizeof(gyro_copy));
    if (primask == 0u)
    {
        __enable_irq();
    }

    acc_raw[0] = (int16_t)(((uint16_t)acc_copy[3] << 8u) | acc_copy[2]);
    acc_raw[1] = (int16_t)(((uint16_t)acc_copy[5] << 8u) | acc_copy[4]);
    acc_raw[2] = (int16_t)(((uint16_t)acc_copy[7] << 8u) | acc_copy[6]);
    gyro_raw[0] = (int16_t)(((uint16_t)gyro_copy[2] << 8u) | gyro_copy[1]);
    gyro_raw[1] = (int16_t)(((uint16_t)gyro_copy[4] << 8u) | gyro_copy[3]);
    gyro_raw[2] = (int16_t)(((uint16_t)gyro_copy[6] << 8u) | gyro_copy[5]);
    (void)BMI088_ProcessRawSample(acc_raw, gyro_raw);
    BMI088_SAMPLE_READY = 0u;
}

void BMI088_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef status;

    if (hspi != BMI088_SPI)
    {
        return;
    }

    if (BMI088_STATE == BMI088_STATE_READING_ACC)
    {
        HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
        BMI088_STATE = BMI088_STATE_READING_GYRO;
        HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
        status = HAL_SPI_TransmitReceive_DMA(BMI088_SPI, BMI088_GYRO_TX,
                                             (uint8_t *)BMI088_GYRO_RX,
                                             BMI088_GYRO_DMA_TRANSFER_LEN);
        if (status != HAL_OK)
        {
            BMI088_DMA_Fail();
        }
    }
    else if (BMI088_STATE == BMI088_STATE_READING_GYRO)
    {
        HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
        memcpy((void *)BMI088_SAMPLE_ACC_RX, (const void *)BMI088_ACC_RX,
               sizeof(BMI088_ACC_RX));
        memcpy((void *)BMI088_SAMPLE_GYRO_RX, (const void *)BMI088_GYRO_RX,
               sizeof(BMI088_GYRO_RX));
        BMI088_STATE = BMI088_STATE_IDLE;
        __DMB();
        BMI088_SAMPLE_READY = 1u;
    }
    else
    {
        BMI088_DMA_Fail();
    }
}

void BMI088_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == BMI088_SPI)
    {
        BMI088_DMA_Fail();
    }
}
