#include "Chassis_Remote.h"

#include "elrs_crsf.h"

static Chassis_Command_t chassis_command;

static int32_t Chassis_Remote_Abs(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int16_t Chassis_Remote_ApplyDeadzone(int16_t value)
{
    int32_t magnitude = Chassis_Remote_Abs(value);

    if (magnitude <= CHASSIS_RC_DEADZONE)
    {
        return 0;
    }

    magnitude = ((magnitude - CHASSIS_RC_DEADZONE) * CHASSIS_COMMAND_MAX) /
                (CHASSIS_COMMAND_MAX - CHASSIS_RC_DEADZONE);
    return (value >= 0) ? (int16_t)magnitude : (int16_t)-magnitude;
}

static void Chassis_Remote_SetStopped(void)
{
    chassis_command.vx = 0;
    chassis_command.vy = 0;
    chassis_command.wz = 0;
}

static void Chassis_Remote_UpdateGear(int16_t switch_value)
{
    if (switch_value < -CHASSIS_SWITCH_THRESHOLD)
    {
        chassis_command.gear = CHASSIS_GEAR_HIGH;
        chassis_command.speed_limit = CHASSIS_HIGH_LIMIT;
    }
    else if (switch_value > CHASSIS_SWITCH_THRESHOLD)
    {
        chassis_command.gear = CHASSIS_GEAR_LOW;
        chassis_command.speed_limit = CHASSIS_LOW_LIMIT;
    }
    else
    {
        chassis_command.gear = CHASSIS_GEAR_MEDIUM;
        chassis_command.speed_limit = CHASSIS_MEDIUM_LIMIT;
    }
}

void Chassis_Remote_Update(void)
{
    int16_t sa_value;
    int16_t sb_value;

    if (ELRS_CRSF_IsOnline() == 0u)
    {
        chassis_command.state = CHASSIS_STATE_OFFLINE;
        chassis_command.gear = CHASSIS_GEAR_LOW;
        chassis_command.speed_limit = CHASSIS_LOW_LIMIT;
        Chassis_Remote_SetStopped();
        return;
    }

    sa_value = ELRS_CRSF_RawToSigned(ELRS_CRSF_GetChannel(CHASSIS_SA_CHANNEL));
    sb_value = ELRS_CRSF_RawToSigned(ELRS_CRSF_GetChannel(CHASSIS_SB_CHANNEL));
    Chassis_Remote_UpdateGear(sb_value);

    if (sa_value >= -CHASSIS_SWITCH_THRESHOLD)
    {
        chassis_command.state = CHASSIS_STATE_LOCKED;
        Chassis_Remote_SetStopped();
        return;
    }

    chassis_command.state = CHASSIS_STATE_ENABLED;
    chassis_command.vx = Chassis_Remote_ApplyDeadzone(
        ELRS_CRSF_RawToSigned(ELRS_CRSF_GetChannel(CHASSIS_VX_CHANNEL)));
    chassis_command.vy = Chassis_Remote_ApplyDeadzone(
        ELRS_CRSF_RawToSigned(ELRS_CRSF_GetChannel(CHASSIS_VY_CHANNEL)));
    chassis_command.wz = Chassis_Remote_ApplyDeadzone(
        ELRS_CRSF_RawToSigned(ELRS_CRSF_GetChannel(CHASSIS_WZ_CHANNEL)));

    chassis_command.vx = (int16_t)((int32_t)chassis_command.vx *
                                   chassis_command.speed_limit / CHASSIS_COMMAND_MAX);
    chassis_command.vy = (int16_t)((int32_t)chassis_command.vy *
                                   chassis_command.speed_limit / CHASSIS_COMMAND_MAX);
    chassis_command.wz = (int16_t)((int32_t)chassis_command.wz *
                                   chassis_command.speed_limit / CHASSIS_COMMAND_MAX);
}

const Chassis_Command_t *Chassis_Remote_GetCommand(void)
{
    return &chassis_command;
}
