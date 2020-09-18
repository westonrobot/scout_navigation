/*
 * imu_extension.hpp
 *
 * Created on: Sep 18, 2020 11:20
 * Description:
 *
 * Copyright (c) 2020 Pinda Tan
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef IMU_EXTENSION_HPP
#define IMU_EXTENSION_HPP

#include <sensor_msgs/Imu.h>

#include "scout_webots_sim/scout_webots_extension.hpp"

namespace westonrobot {
class ImuExtension : public WebotsExtension {
 public:
  ImuExtension() = default;
  ~ImuExtension() = default;

  void Setup(ros::NodeHandle &nh, std::string robot_name,
             tf2_ros::StaticTransformBroadcaster &static_broadcaster) override;

 private:
  ros::Subscriber gyro_sub_;
  ros::Subscriber accel_sub_;
  ros::Publisher imu_pub_;

  sensor_msgs::Imu accel_data_;

  void AccelNewDataCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void GyroNewDataCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void PublishIMUTF(std::string robot_name,
                    tf2_ros::StaticTransformBroadcaster &static_broadcaster);
};
}  // namespace westonrobot

#endif