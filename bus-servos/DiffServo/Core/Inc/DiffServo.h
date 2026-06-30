#ifndef DIFFSERVO_H
#define DIFFSERVO_H

#include "main.h"

#define DIFFSERVO_SC09_BAUDRATE  1000000U
#define DIFFSERVO_ZP15D_BAUDRATE 115200U

uint8_t DiffServo_SelectSC09(void);
uint8_t DiffServo_SelectZP15D(void);
uint8_t DiffServo_MoveBoth(uint8_t sc09_id, uint16_t sc09_position,
                           uint8_t zp15d_id, uint16_t zp15d_position,
                           uint16_t move_time);

#endif
