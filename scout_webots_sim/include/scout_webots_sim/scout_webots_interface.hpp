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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <memory>
#include "scout_base/scout_messenger.hpp"
#include "scout_base/scout_params.hpp"
#include "scout_webots_extension.hpp"


namespace westonrobot {
class ScoutWebotsInterface {
 public:
  ScoutWebotsInterface(ros::NodeHandle* nh, ScoutROSMessenger* msger,
                       uint32_t time_step);

  void InitComponents(std::string controller_name);
  void UpdateSimState();
  
  void AddExtensions(std::vector<std::shared_ptr<westonrobot::WebotsExtension>> extensionVec);
  void InitExtensions();

 private:
  uint32_t time_step_;
  ScoutROSMessenger* messenger_;
  std::vector<std::shared_ptr<westonrobot::WebotsExtension>> extension_vector;
  ros::NodeHandle* nh_;
  sensor_msgs::Imu accel_data_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  std::string robot_name_ = "scout_v2";
  const std::vector<std::string> motor_names_{
      "front_right_wheel", "front_left_wheel", "rear_left_wheel",
      "rear_right_wheel"};

  void SetupRobot();
  void SetupLidar();

};
}  // namespace westonrobot

#endif /* SCOUT_WEBOTS_INTERFACE_HPP */
