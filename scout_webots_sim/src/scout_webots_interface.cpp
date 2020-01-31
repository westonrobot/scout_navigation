/* 
 * scout_webots_interface.cpp
 * 
 * Created on: Sep 26, 2019 23:19
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_webots_sim/scout_webots_interface.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>

#include <webots_ros/set_float.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>

#include "scout_webots_sim/scout_sim_params.hpp"

namespace wescore
{
ScoutWebotsInterface::ScoutWebotsInterface(ros::NodeHandle *nh, ScoutROSMessenger *msger, uint32_t time_step)
    : nh_(nh), messenger_(msger), time_step_(time_step)
{
}

void ScoutWebotsInterface::InitComponents(std::string controller_name)
{
    // reset controller name
    robot_name_ = controller_name;

    // init motors
    for (int i = 0; i < 4; ++i)
    {
        // position
        webots_ros::set_float set_position_srv;
        ros::ServiceClient set_position_client = nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" + std::string(motor_names_[i]) +
                                                                                           std::string("/set_position"));

        set_position_srv.request.value = INFINITY;
        if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
            ROS_INFO("Position set to INFINITY for motor %s.", motor_names_[i].c_str());
        else
            ROS_ERROR("Failed to call service set_position on motor %s.", motor_names_[i].c_str());

        // speed
        ros::ServiceClient set_velocity_client;
        webots_ros::set_float set_velocity_srv;
        set_velocity_client = nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" + std::string(motor_names_[i]) +
                                                                        std::string("/set_velocity"));

        set_velocity_srv.request.value = 0.0;
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
            ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
        else
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motor_names_[i].c_str());
    }
}

void ScoutWebotsInterface::UpdateSimState()
{
    // constants for calculation
    constexpr double rotation_radius = std::hypot(ScoutSimParams::wheelbase / 2.0, ScoutSimParams::track / 2.0) * 2.0;
    constexpr double rotation_theta = std::atan2(ScoutSimParams::wheelbase, ScoutSimParams::track);

    // update robot state
    double wheel_speeds[4];
    for (int i = 0; i < 4; ++i)
    {
        webots_ros::get_float get_velocity_srv;
        ros::ServiceClient get_velocity_client = nh_->serviceClient<webots_ros::get_float>(robot_name_ + "/" + std::string(motor_names_[i]) +
                                                                                           std::string("/get_velocity"));

        if (get_velocity_client.call(get_velocity_srv))
        {
            wheel_speeds[i] = get_velocity_srv.response.value;
            ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
        }
        else
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motor_names_[i].c_str());
    }
    float left_speed = (wheel_speeds[1] + wheel_speeds[2]) / 2.0 * ScoutSimParams::wheel_radius;
    float right_speed = (wheel_speeds[0] + wheel_speeds[3]) / 2.0 * ScoutSimParams::wheel_radius;
    double linear_speed = (right_speed + left_speed) / 2.0;
    double angular_speed = (right_speed - left_speed) * std::cos(rotation_theta) / rotation_radius;

    messenger_->PublishSimStateToROS(linear_speed, angular_speed);

    // send robot command
    double linear, angular;
    messenger_->GetCurrentMotionCmdForSim(linear, angular);

    if (linear > ScoutSimParams::max_linear_speed)
        linear = ScoutSimParams::max_linear_speed;
    if (linear < -ScoutSimParams::max_linear_speed)
        linear = -ScoutSimParams::max_linear_speed;

    if (angular > ScoutSimParams::max_angular_speed)
        angular = ScoutSimParams::max_angular_speed;
    if (angular < -ScoutSimParams::max_angular_speed)
        angular = -ScoutSimParams::max_angular_speed;

    double vel_left_cmd = (linear - angular * rotation_radius / std::cos(rotation_theta)) / ScoutSimParams::wheel_radius;
    double vel_right_cmd = (linear + angular * rotation_radius / std::cos(rotation_theta)) / ScoutSimParams::wheel_radius;

    double wheel_cmds[4];
    wheel_cmds[0] = vel_right_cmd;
    wheel_cmds[1] = vel_left_cmd;
    wheel_cmds[2] = vel_left_cmd;
    wheel_cmds[3] = vel_right_cmd;
    for (int i = 0; i < 4; ++i)
    {
        ros::ServiceClient set_velocity_client;
        webots_ros::set_float set_velocity_srv;
        set_velocity_client = nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" + std::string(motor_names_[i]) +
                                                                        std::string("/set_velocity"));

        set_velocity_srv.request.value = wheel_cmds[i];
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
            ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
        else
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motor_names_[i].c_str());
    }
}

} // namespace wescore