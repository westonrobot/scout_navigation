/* 
 * scout_can_parser.c
 * 
 * Created on: Aug 31, 2019 04:25
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_protocol/scout_can_parser.h"

#include "string.h"

bool DecodeScoutStatusMsgFromCAN(const struct can_frame *rx_frame, ScoutStatusMessage *msg)
{
    msg->msg_type = ScoutStatusNone;

    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_STATUS_ID:
    {
        msg->msg_type = ScoutMotionStatusMsg;
        // msg->motion_status_msg.id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        memcpy(msg->motion_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_CONTROL_STATUS_ID:
    {
        msg->msg_type = ScoutLightStatusMsg;
        // msg->light_status_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        memcpy(msg->light_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_SYSTEM_STATUS_STATUS_ID:
    {
        msg->msg_type = ScoutSystemStatusMsg;
        // msg->system_status_msg.id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        memcpy(msg->system_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR1_DRIVER_STATUS_ID:
    {
        msg->msg_type = ScoutMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        msg->motor_driver_status_msg.motor_id = SCOUT_MOTOR1_ID;
        memcpy(msg->motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR2_DRIVER_STATUS_ID:
    {
        msg->msg_type = ScoutMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        msg->motor_driver_status_msg.motor_id = SCOUT_MOTOR2_ID;
        memcpy(msg->motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR3_DRIVER_STATUS_ID:
    {
        msg->msg_type = ScoutMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
        msg->motor_driver_status_msg.motor_id = SCOUT_MOTOR3_ID;
        memcpy(msg->motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR4_DRIVER_STATUS_ID:
    {
        msg->msg_type = ScoutMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR4_DRIVER_STATUS_ID;
        msg->motor_driver_status_msg.motor_id = SCOUT_MOTOR4_ID;
        memcpy(msg->motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return true;
}

bool DecodeScoutControlMsgFromCAN(const struct can_frame *rx_frame, ScoutControlMessage *msg)
{
    msg->msg_type = ScoutControlNone;

    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_CMD_ID:
    {
        msg->msg_type = ScoutMotionControlMsg;
        // msg->motion_control_msg.id = CAN_MSG_MOTION_CONTROL_CMD_ID;
        memcpy(msg->motion_control_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_CONTROL_CMD_ID:
    {
        msg->msg_type = ScoutLightControlMsg;
        // msg->light_control_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        memcpy(msg->light_control_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return true;
}

void EncodeScoutStatusMsgToCAN(const ScoutStatusMessage *msg, struct can_frame *tx_frame)
{
    switch (msg->msg_type)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case ScoutMotionStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->motion_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case ScoutLightStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->light_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case ScoutSystemStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->system_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case ScoutMotorDriverStatusMsg:
    {
        if (msg->motor_driver_status_msg.motor_id == SCOUT_MOTOR1_ID)
            tx_frame->can_id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        else if (msg->motor_driver_status_msg.motor_id == SCOUT_MOTOR2_ID)
            tx_frame->can_id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        else if (msg->motor_driver_status_msg.motor_id == SCOUT_MOTOR3_ID)
            tx_frame->can_id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
        else if (msg->motor_driver_status_msg.motor_id == SCOUT_MOTOR4_ID)
            tx_frame->can_id = CAN_MSG_MOTOR4_DRIVER_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->motor_driver_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    default:
        break;
    }
    tx_frame->data[7] = CalcScoutCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

void EncodeScoutControlMsgToCAN(const ScoutControlMessage *msg, struct can_frame *tx_frame)
{
    switch (msg->msg_type)
    {
    case ScoutMotionControlMsg:
    {
        EncodeScoutMotionControlMsgToCAN(&(msg->motion_control_msg), tx_frame);
        break;
    }
    case ScoutLightControlMsg:
    {
        EncodeScoutLightControlMsgToCAN(&(msg->light_control_msg), tx_frame);
        break;
    }
    default:
        break;
    }
}

void EncodeScoutMotionControlMsgToCAN(const MotionControlMessage *msg, struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_MOTION_CONTROL_CMD_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
    tx_frame->data[7] = CalcScoutCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

void EncodeScoutLightControlMsgToCAN(const LightControlMessage *msg, struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_LIGHT_CONTROL_CMD_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
    tx_frame->data[7] = CalcScoutCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

uint8_t CalcScoutCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i)
        checksum += data[i];
    return checksum;
}