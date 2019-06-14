/* 
 * scout_can_protocol.h
 * 
 * Created on: Jun 10, 2019 23:23
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_CAN_PROTOCOL_H
#define SCOUT_CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

enum ScoutCANMsgIDs
{
    MSG_MOTION_CONTROL_ID = 0x130,
    MSG_MOTION_CONTROL_FEEDBACK_ID = 0x131,
    MSG_LIGHT_CONTROL_ID = 0x140,
    MSG_LIGHT_CONTROL_FEEDBACK_ID = 0x141,
    MSG_SYSTEM_STATUS_FEEDBACK_ID = 0x151,
    MSG_MOTOR1_DRIVER_FEEDBACK_ID = 0x200,
    MSG_MOTOR2_DRIVER_FEEDBACK_ID = 0x201,
    MSG_MOTOR3_DRIVER_FEEDBACK_ID = 0x202,
    MSG_MOTOR4_DRIVER_FEEDBACK_ID = 0x203,
    MSG_LAST_ID
};

static uint8_t Agilex_CANMsgChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i)
        checksum += data[i];
    return checksum;
};

/*------------------------- Motion Control Message -------------------------*/

enum ControlMode
{
    REMOTE_MODE = 0x00,
    CMD_MODE = 0x01
};

enum FaultClearFlag
{
    NO_FAULT = 0x00,
    BAT_UNDER_VOL = 0x01,
    BAT_OVER_VOL = 0x02,
    MOTOR1_COMM = 0x03,
    MOTOR2_COMM = 0x04,
    MOTOR3_COMM = 0x05,
    MOTOR4_COMM = 0x06,
    MOTOR_DRV_OVERHEAT = 0x07,
    MOTOR_OVERCURRENT = 0x08
};

typedef struct
{
    const uint16_t id = MSG_MOTION_CONTROL_ID;
    const uint8_t dlc = 0x08;
    union {
        struct CmdDef
        {
            uint8_t control_mode;
            uint8_t fault_clear_flag;
            int8_t linear_velocity_cmd;
            int8_t angular_velocity_cmd;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t count;
            uint8_t checksum;
        } cmd;
        uint8_t raw[8];
    } data;
} MotionControlMessage;

/*-------------------------- Light Control Message -------------------------*/

enum LightControlFlag
{
    DISABLE_LIGHT_CTRL = 0x00,
    ENABLE_LIGHT_CTRL = 0x01
};

enum LightMode
{
    CONST_OFF = 0x00,
    CONST_ON = 0x01,
    BREATH = 0x02,
    CUSTOM = 0x03
};

typedef struct
{
    const uint16_t id = MSG_LIGHT_CONTROL_ID;
    const uint8_t dlc = 0x08;
    union {
        struct CmdDef
        {
            uint8_t light_ctrl_enable;
            uint8_t front_light_mode;
            uint8_t front_light_custom;
            uint8_t rear_light_mode;
            uint8_t rear_light_custom;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } cmd;
        uint8_t raw[8];
    } data;
} LightControlMessage;

/*--------------------- Motion Control Feedback Message --------------------*/

typedef struct
{
    const uint16_t id = MSG_MOTION_CONTROL_FEEDBACK_ID;
    const uint8_t dlc = 0x08;
    union {
        struct StatusDef
        {
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                int16_t value;
            } linear_velocity;
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                int16_t value;
            } angular_velocity;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} MotionStatusMessage;

/*---------------------- Light Control Feedback Message --------------------*/

typedef struct
{
    const uint16_t id = MSG_LIGHT_CONTROL_FEEDBACK_ID;
    const uint8_t dlc = 0x08;
    union {
        struct StatusDef
        {
            uint8_t light_ctrl_enable;
            uint8_t front_light_mode;
            uint8_t front_light_custom;
            uint8_t rear_light_mode;
            uint8_t rear_light_custom;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} LightStatusMessage;

/*---------------------- System Status Feedback Message --------------------*/

enum BaseState
{
    BASE_NORMAL,
    BASE_ESTOP,
    BASE_EXCEPTION
};

typedef struct
{
    const uint16_t id = MSG_SYSTEM_STATUS_FEEDBACK_ID;
    const uint8_t dlc = 0x08;
    union {
        struct StatusDef
        {
            uint8_t base_state;
            uint8_t control_mode;
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                uint16_t value;
            } battery_voltage;
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                uint16_t value;
            } fault_code;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} SystemStatusMessage;

/*--------------------- Motor 1 Driver Feedback Message --------------------*/

typedef struct
{
    const uint16_t id = MSG_MOTOR1_DRIVER_FEEDBACK_ID;
    const uint8_t dlc = 0x08;
    union {
        struct StatusDef
        {
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                uint16_t value;
            } current;
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                int16_t value;
            } rpm;
            int8_t temperature;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} Motor1DriverStatusMessage;

/*--------------------- Motor 2 Driver Feedback Message --------------------*/

typedef struct
{
    const uint16_t id = MSG_MOTOR2_DRIVER_FEEDBACK_ID;
    const uint8_t dlc = 0x08;
    union {
        struct StatusDef
        {
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                uint16_t value;
            } current;
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                int16_t value;
            } rpm;
            int8_t temperature;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} Motor2DriverStatusMessage;

/*--------------------- Motor 3 Driver Feedback Message --------------------*/

typedef struct
{
    const uint16_t id = MSG_MOTOR3_DRIVER_FEEDBACK_ID;
    const uint8_t dlc = 0x08;
    union {
        struct StatusDef
        {
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                uint16_t value;
            } current;
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                int16_t value;
            } rpm;
            int8_t temperature;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} Motor3DriverStatusMessage;

/*--------------------- Motor 4 Driver Feedback Message --------------------*/

typedef struct
{
    const uint16_t id = MSG_MOTOR4_DRIVER_FEEDBACK_ID;
    const uint8_t dlc = 0x08;
    union {
        struct StatusDef
        {
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                uint16_t value;
            } current;
            union {
                struct
                {
                    uint8_t high_byte;
                    uint8_t low_byte;
                };
                int16_t value;
            } rpm;
            int8_t temperature;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} Motor4DriverStatusMessage;

#ifdef __cplusplus
}
#endif

#endif /* SCOUT_CAN_PROTOCOL_H */
