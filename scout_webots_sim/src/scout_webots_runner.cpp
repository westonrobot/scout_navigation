/*
 * scout_webots_runner.cpp
 *
 * Created on: Sep 18, 2020 11:13
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include <signal.h>
#include <webots_ros/set_float.h>

#include "scout_base/scout_messenger.hpp"

#include "scout_webots_sim/scout_webots_runner.hpp"
#include "scout_webots_sim/scout_webots_interface.hpp"

namespace westonrobot {
ScoutWebotsRunner::ScoutWebotsRunner(ros::NodeHandle *nh, ros::NodeHandle *pnh)
    : nh_(nh), pnh_(pnh) {
  std::cout << "Creating runner" << std::endl;
}

void ScoutWebotsRunner::AddExtension(
    std::shared_ptr<WebotsExtension> extension) {
  extensions_.push_back(extension);
}

int ScoutWebotsRunner::Run() {
  // get the list of availables controllers
  std::string controllerName;
  ros::Subscriber name_sub = nh_->subscribe(
      "model_name", 100, &ScoutWebotsRunner::ControllerNameCallback, this);
  while (controller_count_ == 0 ||
         controller_count_ < name_sub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();

  // if there is more than one controller available, it let the user choose
  if (controller_count_ == 1)
    controllerName = controller_list_[0];
  else {
    int wantedController = 0;
    std::cout << "Choose the # of the controller you want to use:\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controller_count_)
      controllerName = controller_list_[wantedController - 1];
    else {
      ROS_ERROR("Invalid number for controller choice.");
      return 1;
    }
  }
  ROS_INFO("Using controller: '%s'", controllerName.c_str());
  name_sub.shutdown();

  // fetch parameters before connecting to robot
  std::string port_name;
  std::string odom_frame;
  std::string base_frame;
  std::string odom_topic_name;
  bool is_omni_wheel = false;
  bool is_simulated = false;
  int sim_rate = 50;
  bool is_scout_mini = false;

  pnh_->param<std::string>("port_name", port_name, std::string("can0"));
  pnh_->param<std::string>("odom_frame", odom_frame, std::string("odom"));
  pnh_->param<std::string>("base_frame", base_frame, std::string("base_link"));
  pnh_->param<bool>("is_omni_wheel", is_omni_wheel, false);
  pnh_->param<bool>("simulated_robot", is_simulated, true);
  pnh_->param<int>("control_rate", sim_rate, 50);
  pnh_->param<std::string>("odom_topic_name", odom_topic_name,
                           std::string("odom"));
  pnh_->param<bool>("is_scout_mini", is_scout_mini, false);

  std::shared_ptr<ScoutWebotsInterface> robot =
      std::make_shared<ScoutWebotsInterface>(nh_);

  // init robot components
  robot->Initialize(controllerName);
  if (!extensions_.empty()) {
    for (auto &ext : extensions_)
      ext->Setup(nh_, controllerName, static_broadcaster_);
  }

  std::unique_ptr<ScoutMessenger<ScoutWebotsInterface>> messenger =
      std::unique_ptr<ScoutMessenger<ScoutWebotsInterface>>(
          new ScoutMessenger<ScoutWebotsInterface>(robot, nh_));

  messenger->SetOdometryFrame(odom_frame);
  messenger->SetBaseFrame(base_frame);
  messenger->SetOdometryTopicName(odom_topic_name);
  messenger->SetSimulationMode();

  messenger->SetupSubscription();

  const uint32_t time_step = 1000 / sim_rate;
  ROS_INFO(
      "Chosen time step: '%d', make sure you set the same time step in Webots \"basicTimeStep\""
      "scene",
      time_step);

  ROS_INFO("Entering ROS main loop...");

  // main loop
  time_step_client_ = nh_->serviceClient<webots_ros::set_int>(
      "/" + controllerName + "/robot/time_step");
  time_step_srv_.request.value = time_step;
  ros::Rate loop_rate(sim_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  while (ros::ok()) {
    if (time_step_client_.call(time_step_srv_) &&
        time_step_srv_.response.success) {
      messenger->Update();
    } else {
      static int32_t error_cnt = 0;
      ROS_ERROR("Failed to call service time_step for next step.");
      if (++error_cnt > 50) break;
    }
    // ros::spinOnce();
    loop_rate.sleep();
  }
  time_step_srv_.request.value = 0;
  time_step_client_.call(time_step_srv_);

  spinner.stop();
  ros::shutdown();
  return 0;
}

void ScoutWebotsRunner::Stop() {
  ROS_INFO("User stopped the 'scout_webots_node'.");
  time_step_srv_.request.value = 0;
  time_step_client_.call(time_step_srv_);
  ros::shutdown();
}

void ScoutWebotsRunner::ControllerNameCallback(
    const std_msgs::String::ConstPtr &name) {
  controller_count_++;
  controller_list_.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controller_count_,
           controller_list_.back().c_str());
}
}  // namespace westonrobot