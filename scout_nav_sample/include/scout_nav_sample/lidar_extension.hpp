/*
 * lidar_extension.hpp
 *
 * Created on: Sep 18, 2020 11:21
 * Description:
 *
 * Copyright (c) 2020 Pinda Tan
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef LIDAR_EXTENSION_HPP
#define LIDAR_EXTENSION_HPP

#include <sensor_msgs/PointCloud.h>
#include "scout_webots_sim/scout_webots_extension.hpp"

namespace westonrobot {
class LidarExtension : public WebotsExtension {
 public:
  LidarExtension() = default;
  ~LidarExtension() = default;

  void Setup(ros::NodeHandle &nh, std::string robot_name,
             tf2_ros::StaticTransformBroadcaster &static_broadcaster) override;

 private:
  ros::Subscriber sub;
  ros::Publisher pub;

  void subscriber_callback(const sensor_msgs::PointCloud::ConstPtr &msg);
  void publish_TF(std::string robot_name,
                  tf2_ros::StaticTransformBroadcaster &static_broadcaster);
};
}  // namespace westonrobot
#endif