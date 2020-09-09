/*
 * scout_webots_node.cpp
 *
 * Created on: Sep 26, 2019 23:03
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <iostream>

#include "scout_webots_sim/scout_webots_interface.hpp"

using namespace westonrobot;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

static int controllerCount;
static std::vector<std::string> controllerList;

void quit(int sig)
{
  ROS_INFO("User stopped the 'scout_webots_node'.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name)
{
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount,
           controllerList.back().c_str());
}

int main(int argc, char *argv[])
{
  
}