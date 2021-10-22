/*
 * scout_webots_interface.cpp
 *
 * Created on: Sep 26, 2019 23:19
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_webots_sim/scout_webots_interface.hpp"

#include <webots_ros/set_float.h>
#include <webots_ros/get_float.h>

#include "ugv_sdk/details/protocol_v1/robot_limits.hpp"

namespace westonrobot {
namespace {
struct ScoutParams {
  // Scout Parameters
  static constexpr double track =
      0.58306;  // in meter (left & right wheel distance)
  static constexpr double wheelbase =
      0.498;  // in meter (front & rear wheel distance)
  static constexpr double wheel_radius = 0.165;  // in meter
};
}  // namespace

ScoutWebotsInterface::ScoutWebotsInterface(ros::NodeHandle *nh) : nh_(nh) {}

void ScoutWebotsInterface::Initialize(std::string controller_name) {
  // reset controller name
  robot_name_ = controller_name;

  // init motors
  for (int i = 0; i < 4; ++i) {
    // position
    webots_ros::set_float set_position_srv;
    ros::ServiceClient set_position_client =
        nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    if (set_position_client.call(set_position_srv) &&
        set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.",
               motor_names_[i].c_str());
    else
      ROS_ERROR("Failed to call service set_position on motor %s.",
                motor_names_[i].c_str());

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = nh_->serviceClient<webots_ros::set_float>(
        robot_name_ + "/" + std::string(motor_names_[i]) +
        std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) &&
        set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
  }
}

void ScoutWebotsInterface::SetMotionCommand(double linear, double angular) {
  // send robot command
  if (linear > ScoutV2Limits::max_linear) linear = ScoutV2Limits::max_linear;
  if (linear < ScoutV2Limits::min_linear) linear = ScoutV2Limits::min_linear;

  if (angular > ScoutV2Limits::max_angular)
    angular = ScoutV2Limits::max_angular;
  if (angular < ScoutV2Limits::min_angular)
    angular = ScoutV2Limits::min_angular;

  double vel_left_cmd = (linear - angular * ScoutParams::wheelbase / 2.0) /
                        ScoutParams::wheel_radius;
  double vel_right_cmd = (linear + angular * ScoutParams::wheelbase / 2.0) /
                         ScoutParams::wheel_radius;

  // left side motor are installed in a opposite direction, so negated
  double wheel_cmds[4];
  wheel_cmds[0] = vel_right_cmd;
  wheel_cmds[1] = -vel_left_cmd;
  wheel_cmds[2] = -vel_left_cmd;
  wheel_cmds[3] = vel_right_cmd;
  for (int i = 0; i < 4; ++i) {
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = nh_->serviceClient<webots_ros::set_float>(
        robot_name_ + "/" + std::string(motor_names_[i]) +
        std::string("/set_velocity"));

    set_velocity_srv.request.value = wheel_cmds[i];
    if (set_velocity_client.call(set_velocity_srv) &&
        set_velocity_srv.response.success == 1) {
      //   ROS_INFO("Velocity set to %.2f for motor %s.", wheel_cmds[i],
      //            motor_names_[i].c_str());
    } else {
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
    }
  }
}

void ScoutWebotsInterface::SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                                           AgxLightMode r_mode,
                                           uint8_t r_value) {
  // do nothing for the simulator
}

ScoutCoreState ScoutWebotsInterface::GetRobotState() {
  // constants for calculation
  constexpr double rotation_radius =
      std::hypot(ScoutParams::wheelbase / 2.0, ScoutParams::track / 2.0) * 2.0;
  constexpr double rotation_theta =
      std::atan2(ScoutParams::wheelbase, ScoutParams::track);

  // update robot state
  double wheel_speeds[4];
  for (int i = 0; i < 4; ++i) {
    webots_ros::get_float get_velocity_srv;
    ros::ServiceClient get_velocity_client =
        nh_->serviceClient<webots_ros::get_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/get_velocity"));

    if (get_velocity_client.call(get_velocity_srv)) {
      wheel_speeds[i] = get_velocity_srv.response.value;
    } else
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
  }
  float left_speed =
      (wheel_speeds[1] + wheel_speeds[2]) / 2.0 * ScoutParams::wheel_radius;
  float right_speed =
      (wheel_speeds[0] + wheel_speeds[3]) / 2.0 * ScoutParams::wheel_radius;
  left_speed = -left_speed;

  double linear_speed = (right_speed + left_speed) / 2.0;
  double angular_speed =
      (right_speed - left_speed) * std::cos(rotation_theta) / rotation_radius;
  //   double angular_speed = (right_speed - left_speed) /
  //   ScoutV2Limits::wheelbase;

  //   std::cout << "left: " << left_speed << " , right : " << right_speed
  //             << " , linear: " << linear_speed << " , angular: " <<
  //             angular_speed
  //             << std::endl;

  ScoutCoreState msg;
  msg.motion_state.linear_velocity = linear_speed;
  msg.motion_state.angular_velocity = angular_speed;

  return msg;
}

ScoutActuatorState ScoutWebotsInterface::GetActuatorState() {
  double wheel_speeds[4];
  for (int i = 0; i < 4; ++i) {
    webots_ros::get_float get_velocity_srv;
    ros::ServiceClient get_velocity_client =
        nh_->serviceClient<webots_ros::get_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/get_velocity"));

    if (get_velocity_client.call(get_velocity_srv)) {
      wheel_speeds[i] = get_velocity_srv.response.value;
    } else
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
  }

  ScoutActuatorState msg;
  for (int i = 0; i < 4; ++i) {
    msg.actuator_hs_state[i].motor_id = i;
    msg.actuator_hs_state[i].rpm = wheel_speeds[i];
    msg.actuator_hs_state[i].current = 0;
    msg.actuator_hs_state[i].pulse_count = 0;

    msg.actuator_ls_state[i].motor_id = i;
    msg.actuator_ls_state[i].driver_voltage = 26;
    msg.actuator_ls_state[i].driver_temp = 25;
    msg.actuator_ls_state[i].motor_temp = 25;
    msg.actuator_ls_state[i].driver_state = 0;
  }
  return msg;
}
}  // namespace westonrobot
