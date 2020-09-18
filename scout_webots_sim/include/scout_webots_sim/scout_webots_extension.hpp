/*
 * scout_webots_extension.hpp
 *
 * Created on: Sep 18, 2020 11:07
 * Description:
 *
 * Copyright (c) 2020 Pinda Tan (rdu)
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef SCOUT_WEBOTS_EXTENSION_HPP
#define SCOUT_WEBOTS_EXTENSION_HPP

#include <string>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace westonrobot {
class WebotsExtension {
 public:
  WebotsExtension() = default;
  virtual ~WebotsExtension() = default;

  virtual void Setup(
      ros::NodeHandle& nh, std::string robot_name,
      tf2_ros::StaticTransformBroadcaster& static_broadcaster) = 0;
};

}  // namespace westonrobot

#endif /* SCOUT_WEBOTS_EXTENSION_HPP */
