#include "scout/scout_base.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

#include "scout/scout_can_protocol.h"

namespace
{
// source: https://github.com/rxdu/stopwatch
struct StopWatch
{
    using Clock = std::chrono::high_resolution_clock;
    using time_point = typename Clock::time_point;
    using duration = typename Clock::duration;

    StopWatch() { tic_point = Clock::now(); };

    time_point tic_point;

    void tic()
    {
        tic_point = Clock::now();
    };

    double toc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count() / 1000000.0;
    };

    // for different precisions
    double stoc()
    {
        return std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - tic_point).count();
    };

    double mtoc()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();
    };

    double utoc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();
    };

    double ntoc()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - tic_point).count();
    };

    // you have to call tic() before calling this function
    void sleep_until_ms(int64_t period_ms)
    {
        int64_t duration = period_ms - std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(duration));
    };

    void sleep_until_us(int64_t period_us)
    {
        int64_t duration = period_us - std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(duration));
    };
};
} // namespace

namespace wescore
{
ScoutBase::~ScoutBase()
{
    if (cmd_thread_.joinable())
        cmd_thread_.join();
}

void ScoutBase::ConnectCANBus(const std::string &can_if_name)
{
    can_if_ = std::make_shared<ASyncCAN>(can_if_name);

    can_if_->set_receive_callback(std::bind(&ScoutBase::ParseCANFrame, this, std::placeholders::_1));
}

void ScoutBase::StartCmdThread(int32_t period_ms)
{
    cmd_thread_ = std::thread(std::bind(&ScoutBase::ControlLoop, this, period_ms));
    cmd_thread_started_ = true;
}

void ScoutBase::ControlLoop(int32_t period_ms)
{
    StopWatch ctrl_sw;
    uint8_t cmd_count = 0;
    uint8_t light_cmd_count = 0;
    while (true)
    {
        ctrl_sw.tic();

        // motion control message
        {
            MotionControlMessage m_msg;

            m_msg.data.cmd.control_mode = CMD_CAN_MODE;

            motion_cmd_mutex_.lock();
            m_msg.data.cmd.fault_clear_flag = static_cast<uint8_t>(current_motion_cmd_.fault_clear_flag);
            m_msg.data.cmd.linear_velocity_cmd = current_motion_cmd_.linear_velocity;
            m_msg.data.cmd.angular_velocity_cmd = current_motion_cmd_.angular_velocity;
            motion_cmd_mutex_.unlock();

            m_msg.data.cmd.reserved0 = 0;
            m_msg.data.cmd.reserved1 = 0;
            m_msg.data.cmd.count = cmd_count++;
            m_msg.data.cmd.checksum = Agilex_CANMsgChecksum(m_msg.id, m_msg.data.raw, m_msg.dlc);

            // send to can bus
            can_frame m_frame;
            m_frame.can_id = m_msg.id;
            m_frame.can_dlc = m_msg.dlc;
            std::memcpy(m_frame.data, m_msg.data.raw, m_msg.dlc * sizeof(uint8_t));
            can_if_->send_frame(m_frame);
        }

        // check if there is request for light control
        if (light_ctrl_requested_)
        {
            LightControlMessage l_msg;

            light_cmd_mutex_.lock();
            if (light_ctrl_enabled_)
            {
                l_msg.data.cmd.light_ctrl_enable = ENABLE_LIGHT_CTRL;

                l_msg.data.cmd.front_light_mode = static_cast<uint8_t>(current_light_cmd_.front_mode);
                l_msg.data.cmd.front_light_custom = current_light_cmd_.front_custom_value;
                l_msg.data.cmd.rear_light_mode = static_cast<uint8_t>(current_light_cmd_.rear_mode);
                l_msg.data.cmd.rear_light_custom = current_light_cmd_.rear_custom_value;
            }
            else
            {
                l_msg.data.cmd.light_ctrl_enable = DISABLE_LIGHT_CTRL;

                l_msg.data.cmd.front_light_mode = CONST_OFF;
                l_msg.data.cmd.front_light_custom = 0;
                l_msg.data.cmd.rear_light_mode = CONST_OFF;
                l_msg.data.cmd.rear_light_custom = 0;
            }
            light_ctrl_requested_ = false;
            light_cmd_mutex_.unlock();

            l_msg.data.cmd.reserved0 = 0;
            l_msg.data.cmd.count = light_cmd_count++;
            l_msg.data.cmd.checksum = Agilex_CANMsgChecksum(l_msg.id, l_msg.data.raw, l_msg.dlc);

            can_frame l_frame;
            l_frame.can_id = l_msg.id;
            l_frame.can_dlc = l_msg.dlc;
            std::memcpy(l_frame.data, l_msg.data.raw, l_msg.dlc * sizeof(uint8_t));
            can_if_->send_frame(l_frame);
        }

        ctrl_sw.sleep_until_ms(period_ms);
        // std::cout << "control loop update frequency: " << 1.0 / ctrl_sw.toc() << std::endl;
    }
}

ScoutState ScoutBase::GetScoutState()
{
    std::lock_guard<std::mutex> guard(scout_state_mutex_);
    return scout_state_;
}

void ScoutBase::SetMotionCommand(double linear_vel, double angular_vel, ScoutMotionCmd::FaultClearFlag fault_clr_flag)
{
    // make sure cmd thread is started before attempting to send commands
    if (!cmd_thread_started_)
        StartCmdThread(10);

    if (linear_vel < ScoutMotionCmd::min_linear_velocity)
        linear_vel = ScoutMotionCmd::min_linear_velocity;
    if (linear_vel > ScoutMotionCmd::max_linear_velocity)
        linear_vel = ScoutMotionCmd::max_linear_velocity;
    if (angular_vel < ScoutMotionCmd::min_angular_velocity)
        angular_vel = ScoutMotionCmd::min_angular_velocity;
    if (angular_vel > ScoutMotionCmd::max_angular_velocity)
        angular_vel = ScoutMotionCmd::max_angular_velocity;

    std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
    current_motion_cmd_.linear_velocity = static_cast<uint8_t>(linear_vel / ScoutMotionCmd::max_linear_velocity * 100.0);
    current_motion_cmd_.angular_velocity = static_cast<uint8_t>(angular_vel / ScoutMotionCmd::max_angular_velocity * 100.0);
    current_motion_cmd_.fault_clear_flag = fault_clr_flag;
}

void ScoutBase::SetLightCommand(ScoutLightCmd cmd)
{
    std::lock_guard<std::mutex> guard(light_cmd_mutex_);
    current_light_cmd_ = cmd;
    light_ctrl_enabled_ = true;
    light_ctrl_requested_ = true;
}

void ScoutBase::DisableLightCmdControl()
{
    std::lock_guard<std::mutex> guard(light_cmd_mutex_);
    light_ctrl_enabled_ = false;
    light_ctrl_requested_ = true;
}

void ScoutBase::ParseCANFrame(can_frame *rx_frame)
{
    // validate checksum, discard frame if fails
    if (!rx_frame->data[7] == Agilex_CANMsgChecksum(rx_frame->can_id, rx_frame->data, rx_frame->can_dlc))
    {
        std::cerr << "ERROR: checksum mismatch, discard frame with id " << rx_frame->can_id << std::endl;
        return;
    }

    // otherwise, update robot state with new frame
    std::lock_guard<std::mutex> guard(scout_state_mutex_);
    UpdateScoutState(scout_state_, rx_frame);
}

void ScoutBase::UpdateScoutState(ScoutState &state, can_frame *rx_frame)
{
    switch (rx_frame->can_id)
    {
    case MSG_MOTION_CONTROL_FEEDBACK_ID:
    {
        // std::cout << "motion control feedback received" << std::endl;
        MotionStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.linear_velocity = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.linear_velocity.low_byte) | static_cast<uint16_t>(msg.data.status.linear_velocity.high_byte) << 8) / 1000.0;
        state.angular_velocity = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.angular_velocity.low_byte) | static_cast<uint16_t>(msg.data.status.angular_velocity.high_byte) << 8) / 1000.0;
        break;
    }
    case MSG_LIGHT_CONTROL_FEEDBACK_ID:
    {
        // std::cout << "light control feedback received" << std::endl;
        LightStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        if (msg.data.status.light_ctrl_enable == DISABLE_LIGHT_CTRL)
            state.light_control_enabled = false;
        else
            state.light_control_enabled = true;
        state.front_light_state.mode = msg.data.status.front_light_mode;
        state.front_light_state.custom_value = msg.data.status.front_light_custom;
        state.rear_light_state.mode = msg.data.status.rear_light_mode;
        state.rear_light_state.custom_value = msg.data.status.rear_light_custom;
        break;
    }
    case MSG_SYSTEM_STATUS_FEEDBACK_ID:
    {
        // std::cout << "system status feedback received" << std::endl;
        SystemStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.control_mode = msg.data.status.control_mode;
        state.base_state = msg.data.status.base_state;
        state.battery_voltage = (static_cast<uint16_t>(msg.data.status.battery_voltage.low_byte) | static_cast<uint16_t>(msg.data.status.battery_voltage.high_byte) << 8) / 10.0;
        state.fault_code = (static_cast<uint16_t>(msg.data.status.fault_code.low_byte) | static_cast<uint16_t>(msg.data.status.fault_code.high_byte) << 8);
        break;
    }
    case MSG_MOTOR1_DRIVER_FEEDBACK_ID:
    {
        // std::cout << "motor 1 driver feedback received" << std::endl;
        Motor1DriverStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.motor_states[0].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
        state.motor_states[0].rpm = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        ;
        state.motor_states[0].temperature = msg.data.status.temperature;
        break;
    }
    case MSG_MOTOR2_DRIVER_FEEDBACK_ID:
    {
        // std::cout << "motor 2 driver feedback received" << std::endl;
        Motor2DriverStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.motor_states[1].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
        state.motor_states[1].rpm = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        ;
        state.motor_states[1].temperature = msg.data.status.temperature;
        break;
    }
    case MSG_MOTOR3_DRIVER_FEEDBACK_ID:
    {
        // std::cout << "motor 3 driver feedback received" << std::endl;
        Motor3DriverStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.motor_states[2].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
        state.motor_states[2].rpm = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        ;
        state.motor_states[2].temperature = msg.data.status.temperature;
        break;
    }
    case MSG_MOTOR4_DRIVER_FEEDBACK_ID:
    {
        // std::cout << "motor 4 driver feedback received" << std::endl;
        Motor4DriverStatusMessage msg;
        std::memcpy(msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        state.motor_states[3].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
        state.motor_states[3].rpm = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        state.motor_states[3].temperature = msg.data.status.temperature;
        break;
    }
    }
}
} // namespace wescore
