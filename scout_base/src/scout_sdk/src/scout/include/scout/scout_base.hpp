/* 
 * scout_base.hpp
 * 
 * Created on: Jun 04, 2019 01:22
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_BASE_HPP
#define SCOUT_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "stopwatch/stopwatch.h"
#include "async_io/async_can.hpp"

#include "scout/scout_state.hpp"
#include "scout/scout_command.hpp"

namespace wescore
{
class ScoutBase
{
public:
    ScoutBase() = default;
    ~ScoutBase();

    // do not allow copy
    ScoutBase(const ScoutBase &scout) = delete;
    ScoutBase &operator=(const ScoutBase &scout) = delete;

public:
    void ConnectSerialPort(const std::string &port_name = "/dev/ttyUSB0", int32_t baud_rate = 115200);
    void ConnectCANBus(const std::string &can_if_name = "can1");
    bool IsConnectionActive() const { return serial_connected_; }

    void StartCmdThread(int32_t period_ms);

    // motion control
    void SetMotionCommand(double linear_vel, double angular_vel,
                          ScoutMotionCmd::FaultClearFlag fault_clr_flag = ScoutMotionCmd::FaultClearFlag::NO_FAULT);
    
    // light control
    void SetLightCommand(ScoutLightCmd cmd);
    void DisableLightCmdControl();

    // get robot state
    ScoutState GetScoutState();

    // TODO for testing, will be set private in release
    void UpdateScoutState(ScoutState &state, can_frame *rx_frame);

private:
    bool serial_connected_ = false;

    std::shared_ptr<ASyncCAN> can_if_;

    std::thread cmd_thread_;
    std::mutex scout_state_mutex_;
    std::mutex motion_cmd_mutex_;
    std::mutex light_cmd_mutex_;

    ScoutState scout_state_;
    ScoutMotionCmd current_motion_cmd_;
    ScoutLightCmd current_light_cmd_;

    bool light_ctrl_enabled_ = false;
    bool light_ctrl_requested_ = false;

    void ControlLoop(int32_t period_ms);
    void ParseCANFrame(can_frame *rx_frame);
};
} // namespace wescore

#endif /* SCOUT_BASE_HPP */
