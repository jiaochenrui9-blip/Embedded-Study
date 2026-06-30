#include "DiffServo.h"
#include "servo.h"
#include "ZP15D.h"

extern UART_HandleTypeDef huart1;

static uint8_t DiffServo_SetBaudRate(uint32_t baud_rate)
{
    if (huart1.Init.BaudRate == baud_rate)
    {
        return 1;
    }

    /* Leave a short quiet interval before changing the shared bus baud rate. */
    HAL_Delay(2);
    huart1.Init.BaudRate = baud_rate;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        return 0;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_Delay(2);
    return 1;
}

uint8_t DiffServo_SelectSC09(void)
{
    return DiffServo_SetBaudRate(DIFFSERVO_SC09_BAUDRATE);
}

uint8_t DiffServo_SelectZP15D(void)
{
    return DiffServo_SetBaudRate(DIFFSERVO_ZP15D_BAUDRATE);
}

uint8_t DiffServo_MoveBoth(uint8_t sc09_id, uint16_t sc09_position,
                           uint8_t zp15d_id, uint16_t zp15d_position,
                           uint16_t move_time)
{
    if (DiffServo_SelectSC09() == 0)
    {
        return 0;
    }
    Servo_Move(sc09_id, sc09_position, move_time, 0);

    if (DiffServo_SelectZP15D() == 0)
    {
        return 0;
    }
    ZP15D_Move(zp15d_id, zp15d_position, move_time);

    return 1;
}
