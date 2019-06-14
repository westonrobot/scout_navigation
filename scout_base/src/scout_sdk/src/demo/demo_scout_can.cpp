/* 
 * demo_scout_can.cpp
 * 
 * Created on: Jun 12, 2019 05:03
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#include "scout/scout_base.hpp"

using namespace wescore;

int main(int argc, char **argv)
{
    ScoutBase scout;
    scout.ConnectCANBus("can1");
    scout.StartCmdThread(10);

    scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_ON, 0, ScoutLightCmd::LightMode::CONST_ON, 0});

    int count = 0;
    while (true)
    {
        scout.SetMotionCommand(0.5, 0.2);

        if (count == 10)
            scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_OFF, 0, ScoutLightCmd::LightMode::CONST_OFF, 0});

        auto state = scout.GetScoutState();
        std::cout << "-------------------------------" << std::endl;
        std::cout << "control mode: " << static_cast<int>(state.control_mode) << " , base state: " << static_cast<int>(state.base_state) << std::endl;
        std::cout << "battery voltage: " << state.battery_voltage << std::endl;
        std::cout << "velocity (linear, angular): " << state.linear_velocity << ", " << state.angular_velocity << std::endl;
        std::cout << "-------------------------------" << std::endl;

        sleep(1);
        ++count;
    }

    return 0;
}