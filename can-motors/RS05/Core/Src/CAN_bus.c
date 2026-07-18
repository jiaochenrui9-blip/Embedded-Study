#include "CAN_bus.h"

HAL_StatusTypeDef CAN_Filter_Start(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    /* Filter bank 0: accept extended data frames used by the private protocol. */
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = CAN_ID_EXT;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Filter bank 1: accept standard data frames used by the MIT protocol. */
    filter.FilterBank = 1U;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }
    if (CAN_Filter_Start(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_ActivateNotification(hcan,
                                    CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef CAN_SendExtFrame(CAN_HandleTypeDef *hcan,
                                   uint32_t ext_id,
                                   const uint8_t *data,
                                   uint32_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if ((hcan == NULL) || (data == NULL) || (dlc > 8U) ||
        (ext_id > 0x1FFFFFFFU))
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    header.IDE = CAN_ID_EXT;
    header.ExtId = ext_id;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

HAL_StatusTypeDef CAN_SendStdFrame(CAN_HandleTypeDef *hcan,
                                   uint16_t std_id,
                                   const uint8_t *data,
                                   uint32_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if ((hcan == NULL) || (data == NULL) || (dlc > 8U) ||
        (std_id > 0x07FFU))
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    header.IDE = CAN_ID_STD;
    header.StdId = std_id;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}
