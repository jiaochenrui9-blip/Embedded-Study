#ifndef CHASSIS_REMOTE_H
#define CHASSIS_REMOTE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_WZ_CHANNEL        2u
#define CHASSIS_VY_CHANNEL        3u
#define CHASSIS_VX_CHANNEL        4u
#define CHASSIS_SA_CHANNEL        5u
#define CHASSIS_SB_CHANNEL        6u
#define CHASSIS_RC_DEADZONE      80
#define CHASSIS_SWITCH_THRESHOLD 500
#define CHASSIS_COMMAND_MAX    1000
#define CHASSIS_HIGH_LIMIT     1000
#define CHASSIS_MEDIUM_LIMIT    650
#define CHASSIS_LOW_LIMIT       350

typedef enum
{
    CHASSIS_STATE_OFFLINE = 0,
    CHASSIS_STATE_LOCKED,
    CHASSIS_STATE_ENABLED
} Chassis_State_t;

typedef enum
{
    CHASSIS_GEAR_LOW = 0,
    CHASSIS_GEAR_MEDIUM,
    CHASSIS_GEAR_HIGH
} Chassis_Gear_t;

typedef struct
{
    Chassis_State_t state;
    Chassis_Gear_t gear;
    int16_t speed_limit;
    int16_t vx;
    int16_t vy;
    int16_t wz;
} Chassis_Command_t;

void Chassis_Remote_Update(void);
const Chassis_Command_t *Chassis_Remote_GetCommand(void);

#ifdef __cplusplus
}
#endif

#endif
