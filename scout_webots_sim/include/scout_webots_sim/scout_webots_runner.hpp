/*
 * scout_webots_runner.hpp
 *
 * Created on: Sep 18, 2020 11:11
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef SCOUT_WEBOTS_RUNNER_HPP
#define SCOUT_WEBOTS_RUNNER_HPP

#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <webots_ros/set_int.h>

#include "scout_webots_sim/scout_webots_extension.hpp"

namespace westonrobot {
class ScoutWebotsRunner {
 public:
  ScoutWebotsRunner(ros::NodeHandle *nh, ros::NodeHandle *pnh);

  void AddExtension(std::shared_ptr<WebotsExtension> extension);
  int Run();
  void Stop();

 private:
  ros::NodeHandle *nh_;
  ros::NodeHandle *pnh_;

  void ControllerNameCallback(const std_msgs::String::ConstPtr &name);

  int controller_count_ = 0;
  ros::ServiceClient time_step_client_;
  webots_ros::set_int time_step_srv_;
  std::vector<std::string> controller_list_;

  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  std::vector<std::shared_ptr<WebotsExtension>> extensions_;
};

}  // namespace westonrobot

#endif