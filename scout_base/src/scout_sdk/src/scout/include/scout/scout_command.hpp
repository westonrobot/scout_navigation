/* 
 * scout_command.hpp
 * 
 * Created on: Jun 11, 2019 10:33
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_COMMAND_HPP
#define SCOUT_COMMAND_HPP

#include <cstdint>

namespace wescore
{
struct ScoutMotionCmd
{
    enum class FaultClearFlag
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

    ScoutMotionCmd(int8_t linear = 0, int8_t angular = 0,
                   FaultClearFlag fault_clr_flag = FaultClearFlag::NO_FAULT)
        : linear_velocity(linear), angular_velocity(angular),
          fault_clear_flag(fault_clr_flag) {}

    int8_t linear_velocity;
    int8_t angular_velocity;
    FaultClearFlag fault_clear_flag;

    static constexpr double max_linear_velocity = 1.5;      // 1.5m/s
    static constexpr double min_linear_velocity = -1.5;     // -1.5m/s
    static constexpr double max_angular_velocity = 0.7853;  // 0.7853rad/s
    static constexpr double min_angular_velocity = -0.7853; // -0.7853rad/s
};

struct ScoutLightCmd
{
    enum class LightMode
    {
        CONST_OFF = 0x00,
        CONST_ON = 0x01,
        BREATH = 0x02,
        CUSTOM = 0x03
    };

    ScoutLightCmd() = default;
    ScoutLightCmd(LightMode f_mode, uint8_t f_value, LightMode r_mode, uint8_t r_value) : front_mode(f_mode), front_custom_value(f_value),
                                                                                          rear_mode(r_mode), rear_custom_value(r_value) {}

    LightMode front_mode;
    uint8_t front_custom_value;
    LightMode rear_mode;
    uint8_t rear_custom_value;
};
} // namespace wescore

#endif /* SCOUT_COMMAND_HPP */
