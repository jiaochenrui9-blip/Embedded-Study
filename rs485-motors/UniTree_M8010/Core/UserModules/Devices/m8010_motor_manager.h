#ifndef M8010_MOTOR_MANAGER_H
#define M8010_MOTOR_MANAGER_H

#include "m8010_motor.h"

#define M8010_MOTOR_COUNT 15U

typedef enum
{
    M8010_IDLE,
    M8010_TX,
    M8010_WAIT_RX
} M8010_State;

typedef struct
{
    volatile M8010_State state;
    uint32_t rx_start_tick;
    uint8_t tx_buffer[M8010_FRAME_SIZE];
    uint8_t rx_buffer[M8010_RESPONSE_SIZE];

    UART_HandleTypeDef *huart;
    M8010_Motor_t *motors[M8010_MOTOR_COUNT];
    uint8_t current_index;
    uint8_t registered_count;

    GPIO_TypeDef *GPIO_Port;
    uint16_t GPIO_Pin;
} M8010_MotorManager_t;

HAL_StatusTypeDef M8010_MotorManager_Init(M8010_MotorManager_t *manager,
                                           UART_HandleTypeDef *huart,
                                           GPIO_TypeDef *GPIO_Port,
                                           uint16_t GPIO_Pin);

HAL_StatusTypeDef M8010_MotorManager_Register(M8010_MotorManager_t *manager,
                                               M8010_Motor_t *motor,
                                               uint8_t motor_id);

void M8010_MotorManager_Parse(M8010_MotorManager_t *manager,
                              uint8_t rx_buffer[M8010_RESPONSE_SIZE]);

HAL_StatusTypeDef M8010_MotorManager_Update(M8010_MotorManager_t *manager,
                                             uint32_t now);
void M8010_MotorManager_TxCallBack(M8010_MotorManager_t *manager);
void M8010_MotorManager_RxCallBack(M8010_MotorManager_t *manager);
void M8010_MotorManager_ErrorCallBack(M8010_MotorManager_t *manager);

#endif /* M8010_MOTOR_MANAGER_H */
