/* 
 * scout_webots_interface.hpp
 * 
 * Created on: Sep 26, 2019 23:04
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_WEBOTS_INTERFACE_HPP
#define SCOUT_WEBOTS_INTERFACE_HPP

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>

#include "scout_base/scout_messenger.hpp"

namespace wescore
{
class ScoutWebotsInterface
{
public:
    ScoutWebotsInterface(ros::NodeHandle *nh, ScoutROSMessenger* msger, uint32_t time_step);

    void InitComponents(std::string controller_name);
    void UpdateSimState();

private:
    ros::NodeHandle *nh_;
    ScoutROSMessenger* messenger_;
    uint32_t time_step_;

    std::string robot_name_ = "agilex_scout";
    const std::vector<std::string> motor_names_{"motor_fr", "motor_fl", "motor_rl", "motor_rr"};
};
} // namespace wescore

#endif /* SCOUT_WEBOTS_INTERFACE_HPP */
