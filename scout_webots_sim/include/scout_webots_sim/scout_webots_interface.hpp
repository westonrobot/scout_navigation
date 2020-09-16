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
<<<<<<< HEAD

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "scout_base/scout_messenger.hpp"
#include "scout_base/scout_params.hpp"
=======
#include <vector>
#include <ros/ros.h>


#include <tf2_ros/static_transform_broadcaster.h>

#include "scout_base/scout_messenger.hpp"
#include "scout_webots_extension.hpp"

>>>>>>> scout_simulation/main_dev

namespace westonrobot {
class ScoutWebotsInterface {
 public:
  ScoutWebotsInterface(ros::NodeHandle* nh, ScoutROSMessenger* msger,
                       uint32_t time_step);

  void InitComponents(std::string controller_name);
  void UpdateSimState();
<<<<<<< HEAD
=======
  
  void AddExtensions(std::vector<WebotsExtension*> extension_vector);
  void InitExtensions();
>>>>>>> scout_simulation/main_dev

 private:
  uint32_t time_step_;
  ScoutROSMessenger* messenger_;
<<<<<<< HEAD

  ros::NodeHandle* nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc2_pub_;
  ros::Subscriber gyro_sub_;
  ros::Subscriber accel_sub_;
  ros::Publisher imu_pub_;

  sensor_msgs::Imu accel_data_;
=======
  std::vector<WebotsExtension*> extension_vector_;
  ros::NodeHandle* nh_;
  
>>>>>>> scout_simulation/main_dev
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  std::string robot_name_ = "scout_v2";
  const std::vector<std::string> motor_names_{
      "front_right_wheel", "front_left_wheel", "rear_left_wheel",
      "rear_right_wheel"};

  void SetupRobot();
  void SetupLidar();
<<<<<<< HEAD
  void SetupIMU();

  void PublishLidarTF();
  void PublishIMUTF();

  void GyroNewDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void AccelNewDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void LidarNewPointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
=======

>>>>>>> scout_simulation/main_dev
};
}  // namespace westonrobot

#endif /* SCOUT_WEBOTS_INTERFACE_HPP */
