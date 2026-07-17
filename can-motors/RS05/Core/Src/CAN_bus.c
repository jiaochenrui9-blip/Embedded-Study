//
// Created by game on 2026/7/16.
//


#include "CAN_bus.h"

HAL_StatusTypeDef CAN_Filter_Start(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef Filter = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    Filter.FilterBank = 0;

    Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    Filter.FilterScale = CAN_FILTERSCALE_32BIT;

    Filter.FilterIdHigh = 0;
    Filter.FilterIdLow = CAN_ID_EXT;  // IDE必须为1

    Filter.FilterMaskIdHigh = 0;
    Filter.FilterMaskIdLow =
        CAN_ID_EXT | CAN_RTR_REMOTE;   // 检查IDE和RTR

    Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    Filter.FilterActivation = ENABLE;
    Filter.SlaveStartFilterBank =14;
    if (HAL_CAN_ConfigFilter(hcan, &Filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef CAN_Init (CAN_HandleTypeDef *hcan)
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
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
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
    CAN_TxHeaderTypeDef txHeader = {0};
    uint32_t TxMailbox;

    if (hcan == NULL || data == NULL || dlc > 8U || ext_id > 0x1FFFFFFFU)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        return HAL_ERROR;
    }
    txHeader.IDE = CAN_ID_EXT;
    txHeader.ExtId = ext_id;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = dlc;
    txHeader.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &txHeader, data, &TxMailbox);
}
