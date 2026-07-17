//
// Created by game on 2026/7/16.
//

#ifndef RS05_CAN_BUS_H
#define RS05_CAN_BUS_H

#include "main.h"

HAL_StatusTypeDef CAN_Filter_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Init (CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_SendExtFrame(CAN_HandleTypeDef *hcan,
                                   uint32_t ext_id,
                                   const uint8_t *data,
                                   uint32_t dlc);
#endif //RS05_CAN_BUS_H
