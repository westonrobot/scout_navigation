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
#include <vector>

#include "ugv_sdk/details/interface/scout_interface.hpp"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "scout_webots_extension.hpp"

namespace westonrobot {
class ScoutWebotsInterface : public ScoutInterface {
 public:
  ScoutWebotsInterface(ros::NodeHandle* nh);

  void Initialize(std::string controller_name);

  void SetMotionCommand(double linear, double angular) override;
  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                       AgxLightMode r_mode, uint8_t r_value) override;
  void DisableLightControl() {}
  ScoutCoreState GetRobotState() override;
  ScoutActuatorState GetActuatorState() override;

 private:
  ros::NodeHandle* nh_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  uint32_t time_step_;
  std::string robot_name_ = "scout_v2";
  const std::vector<std::string> motor_names_{
      "front_right_wheel", "front_left_wheel", "rear_left_wheel",
      "rear_right_wheel"};
  std::vector<WebotsExtension*> extensions_;

  void Connect(std::string uart_name, uint32_t baudrate) override{};
};
}  // namespace westonrobot

#endif /* SCOUT_WEBOTS_INTERFACE_HPP */
