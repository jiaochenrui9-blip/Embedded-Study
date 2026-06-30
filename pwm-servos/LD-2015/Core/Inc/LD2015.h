#ifndef LD2015_H
#define LD2015_H

#include "main.h"

#define LD2015_MIN_PULSE_US 500U
#define LD2015_MID_PULSE_US 1500U
#define LD2015_MAX_PULSE_US 2500U
#define LD2015_MAX_ANGLE    180U

HAL_StatusTypeDef LD2015_Start(void);
HAL_StatusTypeDef LD2015_Stop(void);
void LD2015_SetPulseUs(uint16_t pulse_us);
void LD2015_SetAngle(uint16_t angle);

#endif
