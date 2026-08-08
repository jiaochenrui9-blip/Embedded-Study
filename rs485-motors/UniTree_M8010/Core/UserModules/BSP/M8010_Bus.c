#include "M8010_Bus.h"

#define M8010_PI                    3.14159265358979323846f
#define M8010_TORQUE_SCALE          256.0f
#define M8010_SPEED_SCALE           (256.0f / (2.0f * M8010_PI))
#define M8010_POSITION_SCALE        (32768.0f / (2.0f * M8010_PI))
#define M8010_GAIN_SCALE            1280.0f
#define M8010_TORQUE_LIMIT_NM       127.99f
#define M8010_SPEED_LIMIT_RAD_S     804.0f
#define M8010_POSITION_RAW_MAX      2147483520.0f
#define M8010_GAIN_LIMIT            25.599f

static uint16_t M8010_Read16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t M8010_Read32LE(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static void M8010_Write16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void M8010_Write32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint16_t M8010_Crc16Ccitt(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0U;

    while (length-- > 0U)
    {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0x8408U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint8_t M8010_ParametersValid(uint8_t motor_id,
                                     M8010_Mode mode,
                                     float torque_nm,
                                     float omega_rad_s,
                                     float position_raw,
                                     float kp,
                                     float kw)
{
    if ((motor_id > 15U) || (mode > M8010_MODE_CALIBRATE))
    {
        return 0U;
    }

    if ((torque_nm != torque_nm) ||
        (omega_rad_s != omega_rad_s) ||
        (position_raw != position_raw) ||
        (kp != kp) ||
        (kw != kw))
    {
        return 0U;
    }

    if ((torque_nm < -M8010_TORQUE_LIMIT_NM) ||
        (torque_nm > M8010_TORQUE_LIMIT_NM) ||
        (omega_rad_s < -M8010_SPEED_LIMIT_RAD_S) ||
        (omega_rad_s > M8010_SPEED_LIMIT_RAD_S) ||
        (position_raw < -2147483648.0f) ||
        (position_raw > M8010_POSITION_RAW_MAX) ||
        (kp < 0.0f) ||
        (kp > M8010_GAIN_LIMIT) ||
        (kw < 0.0f) ||
        (kw > M8010_GAIN_LIMIT))
    {
        return 0U;
    }

    return 1U;
}

HAL_StatusTypeDef M8010_BuildFrame(uint8_t frame[M8010_FRAME_SIZE],
                                   uint8_t motor_id,
                                   M8010_Mode mode,
                                   float torque_nm,
                                   float omega_rad_s,
                                   float position_rad,
                                   float kp,
                                   float kw)
{
    const float position_raw_f = position_rad * M8010_POSITION_SCALE;
    int16_t torque_raw;
    int16_t speed_raw;
    int32_t position_raw;
    uint16_t kp_raw;
    uint16_t kw_raw;
    uint16_t crc;

    if ((frame == NULL) ||
        (M8010_ParametersValid(motor_id, mode, torque_nm, omega_rad_s,
                               position_raw_f, kp, kw) == 0U))
    {
        return HAL_ERROR;
    }

    torque_raw = (int16_t)(torque_nm * M8010_TORQUE_SCALE);
    speed_raw = (int16_t)(omega_rad_s * M8010_SPEED_SCALE);
    position_raw = (int32_t)position_raw_f;
    kp_raw = (uint16_t)(kp * M8010_GAIN_SCALE);
    kw_raw = (uint16_t)(kw * M8010_GAIN_SCALE);

    frame[0] = 0xFEU;
    frame[1] = 0xEEU;
    frame[2] = (uint8_t)(((uint8_t)mode << 4U) | motor_id);
    M8010_Write16LE(&frame[3], (uint16_t)torque_raw);
    M8010_Write16LE(&frame[5], (uint16_t)speed_raw);
    M8010_Write32LE(&frame[7], (uint32_t)position_raw);
    M8010_Write16LE(&frame[11], kp_raw);
    M8010_Write16LE(&frame[13], kw_raw);

    crc = M8010_Crc16Ccitt(frame, M8010_FRAME_SIZE - 2U);
    M8010_Write16LE(&frame[15], crc);
    return HAL_OK;
}

HAL_StatusTypeDef M8010_ParseFrame(const uint8_t frame[M8010_RESPONSE_SIZE],
                                   uint8_t *motor_id,
                                   M8010_Data *data)
{
    M8010_Data parsed;
    uint16_t status_word;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((frame == NULL) || (motor_id == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    received_crc = M8010_Read16LE(&frame[M8010_RESPONSE_SIZE - 2U]);
    calculated_crc = M8010_Crc16Ccitt(frame, M8010_RESPONSE_SIZE - 2U);
    if ((frame[0] != 0xFDU) ||
        (frame[1] != 0xEEU) ||
        (received_crc != calculated_crc))
    {
        return HAL_ERROR;
    }

    *motor_id = frame[2] & 0x0FU;
    status_word = M8010_Read16LE(&frame[12]);
    parsed.status = (frame[2] >> 4U) & 0x07U;
    parsed.torque_nm =
        (float)(int16_t)M8010_Read16LE(&frame[3]) / M8010_TORQUE_SCALE;
    parsed.omega_rad_s =
        (float)(int16_t)M8010_Read16LE(&frame[5]) / M8010_SPEED_SCALE;
    parsed.position_rad =
        (float)(int32_t)M8010_Read32LE(&frame[7]) / M8010_POSITION_SCALE;
    parsed.temperature_c = (int8_t)frame[11];
    parsed.error_code = (uint8_t)(status_word & 0x0007U);
    parsed.force_raw = (status_word >> 3U) & 0x0FFFU;

    *data = parsed;
    return HAL_OK;
}
