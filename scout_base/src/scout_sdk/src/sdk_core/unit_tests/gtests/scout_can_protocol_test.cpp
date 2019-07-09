/* 
 * scout_can_protocol_test.cpp
 * 
 * Created on: Jun 11, 2019 23:26
 * Description: 
 * 
 * Reference:
 * [1] https://cryptii.com/pipes/integer-encoder
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "scout/scout_base.hpp"
#include "scout/scout_can_protocol.h"

using namespace wescore;

struct ScoutCANProtocolTest : testing::Test
{
    ScoutCANProtocolTest()
    {
        // control frames
        motion_ctrl_frame.can_id = MSG_MOTION_CONTROL_ID;
        motion_ctrl_frame.can_dlc = 8;
        motion_ctrl_frame.data[0] = 0x01;
        motion_ctrl_frame.data[1] = 0x00;
        motion_ctrl_frame.data[2] = 0x0a;
        motion_ctrl_frame.data[3] = 0x00;
        motion_ctrl_frame.data[4] = 0x00;
        motion_ctrl_frame.data[5] = 0x00;
        motion_ctrl_frame.data[6] = 0x00;
        motion_ctrl_frame.data[7] = Agilex_CANMsgChecksum(motion_ctrl_frame.can_id,
                                                          motion_ctrl_frame.data,
                                                          motion_ctrl_frame.can_dlc);

        light_ctrl_frame.can_id = MSG_LIGHT_CONTROL_ID;
        light_ctrl_frame.can_dlc = 8;
        light_ctrl_frame.data[0] = 0x01;
        light_ctrl_frame.data[1] = 0x00;
        light_ctrl_frame.data[2] = 0x0a;
        light_ctrl_frame.data[3] = 0x00;
        light_ctrl_frame.data[4] = 0x00;
        light_ctrl_frame.data[5] = 0x00;
        light_ctrl_frame.data[6] = 0x00;
        light_ctrl_frame.data[7] = Agilex_CANMsgChecksum(light_ctrl_frame.can_id,
                                                         light_ctrl_frame.data,
                                                         light_ctrl_frame.can_dlc);

        // feedback frames
        motion_status_frame.can_id = MSG_MOTION_CONTROL_FEEDBACK_ID;
        motion_status_frame.can_dlc = 8;
        motion_status_frame.data[0] = 0x04;
        motion_status_frame.data[1] = 0xe8; // 1.256
        motion_status_frame.data[2] = 0x00;
        motion_status_frame.data[3] = 0x7b; // 0.123
        motion_status_frame.data[4] = 0x00;
        motion_status_frame.data[5] = 0x00;
        motion_status_frame.data[6] = 0x00;
        motion_status_frame.data[7] = Agilex_CANMsgChecksum(motion_status_frame.can_id,
                                                            motion_status_frame.data,
                                                            motion_status_frame.can_dlc);

        motion_status_frame2.can_id = MSG_MOTION_CONTROL_FEEDBACK_ID;
        motion_status_frame2.can_dlc = 8;
        motion_status_frame2.data[0] = 0xfb;
        motion_status_frame2.data[1] = 0x18; // -1.256
        motion_status_frame2.data[2] = 0xff;
        motion_status_frame2.data[3] = 0x85; // -0.123
        motion_status_frame2.data[4] = 0x00;
        motion_status_frame2.data[5] = 0x00;
        motion_status_frame2.data[6] = 0x00;
        motion_status_frame2.data[7] = Agilex_CANMsgChecksum(motion_status_frame2.can_id,
                                                            motion_status_frame2.data,
                                                            motion_status_frame2.can_dlc);

        light_status_frame.can_id = MSG_LIGHT_CONTROL_FEEDBACK_ID;
        light_status_frame.can_dlc = 8;
        light_status_frame.data[0] = 0x01;
        light_status_frame.data[1] = 0x03;
        light_status_frame.data[2] = 0x55; // 85
        light_status_frame.data[3] = 0x02;
        light_status_frame.data[4] = 0x32; // 50
        light_status_frame.data[5] = 0x00;
        light_status_frame.data[6] = 0x00;
        light_status_frame.data[7] = Agilex_CANMsgChecksum(light_status_frame.can_id,
                                                           light_status_frame.data,
                                                           light_status_frame.can_dlc);

        system_status_frame.can_id = MSG_SYSTEM_STATUS_FEEDBACK_ID;
        system_status_frame.can_dlc = 8;
        system_status_frame.data[0] = 0x01;
        system_status_frame.data[1] = 0x01;
        system_status_frame.data[2] = 0x01;
        system_status_frame.data[3] = 0x09; // 26.5
        system_status_frame.data[4] = 0xf0;
        system_status_frame.data[5] = 0xff;
        system_status_frame.data[6] = 0x00;
        system_status_frame.data[7] = Agilex_CANMsgChecksum(system_status_frame.can_id,
                                                            system_status_frame.data,
                                                            system_status_frame.can_dlc);

        motor1_driver_status_frame.can_id = MSG_MOTOR1_DRIVER_FEEDBACK_ID;
        motor1_driver_status_frame.can_dlc = 8;
        motor1_driver_status_frame.data[0] = 0x00;
        motor1_driver_status_frame.data[1] = 0x7d;  // 12.5
        motor1_driver_status_frame.data[2] = 0x04;
        motor1_driver_status_frame.data[3] = 0xd2;  // 1234
        motor1_driver_status_frame.data[4] = 0x38;  // 56
        motor1_driver_status_frame.data[5] = 0x00;
        motor1_driver_status_frame.data[6] = 0x00;
        motor1_driver_status_frame.data[7] = Agilex_CANMsgChecksum(motor1_driver_status_frame.can_id,
                                                                   motor1_driver_status_frame.data,
                                                                   motor1_driver_status_frame.can_dlc);

        motor2_driver_status_frame = motor1_driver_status_frame;
        motor2_driver_status_frame.can_id = MSG_MOTOR2_DRIVER_FEEDBACK_ID;
        motor2_driver_status_frame.data[2] = 0xfb;
        motor2_driver_status_frame.data[3] = 0x2e;  // -1234
        motor2_driver_status_frame.data[4] = 0xc8;  // -56
        motor2_driver_status_frame.data[7] = Agilex_CANMsgChecksum(motor2_driver_status_frame.can_id,
                                                                   motor2_driver_status_frame.data,
                                                                   motor2_driver_status_frame.can_dlc);

        motor3_driver_status_frame = motor1_driver_status_frame;
        motor3_driver_status_frame.can_id = MSG_MOTOR3_DRIVER_FEEDBACK_ID;
        motor3_driver_status_frame.data[7] = Agilex_CANMsgChecksum(motor3_driver_status_frame.can_id,
                                                                   motor3_driver_status_frame.data,
                                                                   motor3_driver_status_frame.can_dlc);

        motor4_driver_status_frame = motor2_driver_status_frame;
        motor4_driver_status_frame.can_id = MSG_MOTOR4_DRIVER_FEEDBACK_ID;
        motor4_driver_status_frame.data[7] = Agilex_CANMsgChecksum(motor4_driver_status_frame.can_id,
                                                                   motor4_driver_status_frame.data,
                                                                   motor4_driver_status_frame.can_dlc);
    }

    ScoutBase scout_base;

    can_frame motion_ctrl_frame;
    can_frame light_ctrl_frame;

    can_frame motion_status_frame;
    can_frame motion_status_frame2;

    can_frame light_status_frame;
    can_frame system_status_frame;
    can_frame motor1_driver_status_frame;
    can_frame motor2_driver_status_frame;
    can_frame motor3_driver_status_frame;
    can_frame motor4_driver_status_frame;
};

TEST_F(ScoutCANProtocolTest, MotionStatusMsg)
{
    ScoutState state;
    scout_base.UpdateScoutState(state, &motion_status_frame);

    ASSERT_FLOAT_EQ(state.linear_velocity, 1.256);
    ASSERT_FLOAT_EQ(state.angular_velocity, 0.123);

    ScoutState state2;
    scout_base.UpdateScoutState(state2, &motion_status_frame2);

    ASSERT_FLOAT_EQ(state2.linear_velocity, -1.256);
    ASSERT_FLOAT_EQ(state2.angular_velocity, -0.123);
}

TEST_F(ScoutCANProtocolTest, LightStatusMsg)
{
    ScoutState state;
    scout_base.UpdateScoutState(state, &light_status_frame);

    ASSERT_EQ(state.light_control_enabled, true);
    ASSERT_EQ(state.front_light_state.mode, 0x03);
    ASSERT_EQ(state.front_light_state.custom_value, 85);
    ASSERT_EQ(state.rear_light_state.mode, 0x02);
    ASSERT_EQ(state.rear_light_state.custom_value, 50);
}

TEST_F(ScoutCANProtocolTest, SystemStatusMsg)
{
    ScoutState state;
    scout_base.UpdateScoutState(state, &system_status_frame);

    ASSERT_EQ(state.base_state, true);
    ASSERT_EQ(state.control_mode, 0x01);
    ASSERT_FLOAT_EQ(state.battery_voltage, 26.5);
    ASSERT_EQ(state.fault_code, 0xf0ff);
}

TEST_F(ScoutCANProtocolTest, MotorDriverStatusMsg)
{
    ScoutState state;
    scout_base.UpdateScoutState(state, &motor1_driver_status_frame);
    scout_base.UpdateScoutState(state, &motor2_driver_status_frame);
    scout_base.UpdateScoutState(state, &motor3_driver_status_frame);
    scout_base.UpdateScoutState(state, &motor4_driver_status_frame);

    ASSERT_FLOAT_EQ(state.motor_states[0].current, 12.5);
    ASSERT_FLOAT_EQ(state.motor_states[0].rpm, 1234);
    ASSERT_FLOAT_EQ(state.motor_states[0].temperature, 56);
    
    ASSERT_FLOAT_EQ(state.motor_states[1].current, 12.5);
    ASSERT_FLOAT_EQ(state.motor_states[1].rpm, -1234);
    ASSERT_FLOAT_EQ(state.motor_states[1].temperature, -56);

    ASSERT_FLOAT_EQ(state.motor_states[2].current, 12.5);
    ASSERT_FLOAT_EQ(state.motor_states[2].rpm, 1234);
    ASSERT_FLOAT_EQ(state.motor_states[2].temperature, 56);

    ASSERT_FLOAT_EQ(state.motor_states[3].current, 12.5);
    ASSERT_FLOAT_EQ(state.motor_states[3].rpm, -1234);
    ASSERT_FLOAT_EQ(state.motor_states[3].temperature, -56);
}

