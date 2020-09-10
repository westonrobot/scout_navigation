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



#include "scout_webots_sim/scout_webots_runner.hpp"
using namespace westonrobot;

int main(int argc, char *argv[])
{
  ScoutWebotsRunner runner;
  return runner.Run(argc,argv);
  
}