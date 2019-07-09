/* 
 * scout_state.hpp
 * 
 * Created on: Jun 11, 2019 08:48
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_STATE_HPP
#define SCOUT_STATE_HPP

#include <cstdint>
#include <iostream>

namespace wescore
{
struct ScoutState
{
    enum MotorID
    {
        FRONT_RIGHT = 0,
        FRONT_LEFT = 1,
        REAR_LEFT = 2,
        REAR_RIGHT = 3
    };

    struct MotorState
    {
        double current = 0; // in A
        double rpm = 0;
        double temperature = 0;
    };

    struct LightState
    {
        uint8_t mode = 0;
        uint8_t custom_value = 0;
    };

    // base state
    uint8_t base_state = 0;
    uint8_t control_mode = 0;
    uint16_t fault_code = 0;
    double battery_voltage = 0.0;

    // motor state
    MotorState motor_states[4];

    // light state
    bool light_control_enabled = false;
    LightState front_light_state;
    LightState rear_light_state;

    // motion state
    double linear_velocity;
    double angular_velocity;
};
} // namespace wescore

#endif /* SCOUT_STATE_HPP */
