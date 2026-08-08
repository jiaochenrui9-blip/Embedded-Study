#include "m8010_motor_manager.h"

HAL_StatusTypeDef M8010_MotorManager_Init(M8010_MotorManager_t *manager,
                                           UART_HandleTypeDef *huart,
                                           GPIO_TypeDef *GPIO_Port,
                                           uint16_t GPIO_Pin)
{
    if ((manager == NULL) || (huart == NULL) || (GPIO_Port == NULL))
    {
        return HAL_ERROR;
    }

    *manager = (M8010_MotorManager_t){0};
    manager->huart = huart;
    manager->GPIO_Port = GPIO_Port;
    manager->GPIO_Pin = GPIO_Pin;
    manager->state = M8010_IDLE;
    manager->current_index = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef M8010_MotorManager_Register(M8010_MotorManager_t *manager,
                                               M8010_Motor_t *motor,
                                               uint8_t motor_id)
{
    uint8_t index;

    if ((manager == NULL) || (manager->huart == NULL) || (motor == NULL) ||
        (motor_id >= M8010_MOTOR_COUNT) ||
        (manager->registered_count >= M8010_MOTOR_COUNT))
    {
        return HAL_ERROR;
    }

    for (index = 0U; index < manager->registered_count; ++index)
    {
        if ((manager->motors[index] != NULL) &&
            (manager->motors[index]->motor_id == motor_id))
        {
            return HAL_ERROR;
        }
    }

    *motor = (M8010_Motor_t){0};
    motor->motor_id = motor_id;
    motor->command.mode = M8010_MODE_LOCKED;

    manager->motors[manager->registered_count] = motor;
    manager->registered_count++;
    return HAL_OK;
}

HAL_StatusTypeDef M8010_MotorManager_Update(M8010_MotorManager_t *manager)
{
    M8010_Motor_t *motor;
    HAL_StatusTypeDef status;

    if ((manager == NULL) || (manager->huart == NULL) ||
        (manager->GPIO_Port == NULL) ||
        (manager->registered_count == 0U))
    {
        return HAL_ERROR;
    }
    if (manager->state != M8010_IDLE)
    {
        return HAL_BUSY;
    }
    if (manager->current_index >= manager->registered_count)
    {
        manager->current_index = 0U;
    }

    motor = manager->motors[manager->current_index];
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    if (M8010_BuildFrame(manager->tx_buffer,
                         motor->motor_id,
                         motor->command.mode,
                         motor->command.torque_nm,
                         motor->command.omega_rad_s,
                         motor->command.position_rad,
                         motor->command.kp,
                         motor->command.kw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(manager->GPIO_Port, manager->GPIO_Pin, GPIO_PIN_SET);
    status = HAL_UART_Transmit_DMA(manager->huart,
                                   manager->tx_buffer,
                                   M8010_FRAME_SIZE);
    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(manager->GPIO_Port, manager->GPIO_Pin,
                          GPIO_PIN_RESET);
        return status;
    }

    manager->state = M8010_TX;
    return status;
}

void M8010_MotorManager_Parse(M8010_MotorManager_t *manager,uint8_t rx_buffer[M8010_RESPONSE_SIZE])
{
    M8010_Data feedback;
    M8010_Motor_t *motor = NULL;
    uint8_t motor_id;

    if ((manager == NULL) || (rx_buffer == NULL))
    {
        return;
    }

    if (M8010_ParseFrame(rx_buffer, &motor_id, &feedback) != HAL_OK)
    {
        return;
    }

    for (uint8_t index = 0U; index < manager->registered_count; ++index)
    {
        if ((manager->motors[index] != NULL) &&
            (manager->motors[index]->motor_id == motor_id))
        {
            motor = manager->motors[index];
            break;
        }
    }

    if (motor == NULL)
    {
        return;
    }

    motor->feedback = feedback;
    motor->feedback_updated = 1U;
    motor->feedback_count++;
    motor->last_feedback_tick = HAL_GetTick();

}

void M8010_MotorManager_TxCallBack(M8010_MotorManager_t *manager)
{
    HAL_StatusTypeDef status;

    if ((manager == NULL) || (manager->state != M8010_TX))
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(manager->huart);
    status = HAL_UART_Receive_DMA(manager->huart,
                                  manager->rx_buffer,
                                  M8010_RESPONSE_SIZE);
    HAL_GPIO_WritePin(manager->GPIO_Port, manager->GPIO_Pin,
                      GPIO_PIN_RESET);

    manager->state = (status == HAL_OK) ? M8010_WAIT_RX : M8010_IDLE;
}

void M8010_MotorManager_RxCallBack(M8010_MotorManager_t *manager)
{
    if ((manager == NULL) || (manager->state != M8010_WAIT_RX))
    {
        return;
    }
    M8010_MotorManager_Parse(manager, manager->rx_buffer);

    manager->current_index++;
    if (manager->current_index >= manager->registered_count)
    {
        manager->current_index = 0U;
    }

    manager->state = M8010_IDLE;
}
