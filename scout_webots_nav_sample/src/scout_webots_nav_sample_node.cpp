/*
 * scout_webots_node.cpp
 *
 * Created on: Sep 26, 2019 23:03
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_webots_sim/scout_webots_runner.hpp"
#include "scout_webots_nav_sample/IMU_extension.hpp"
#include "scout_webots_nav_sample/lidar_extension.hpp"

using namespace westonrobot;

int main(int argc, char *argv[])
{
  ScoutWebotsRunner runner;
  LidarExtension lidar = LidarExtension();
  IMUExtension IMU = IMUExtension();

  runner.AddExtension(&lidar);
  runner.AddExtension(&IMU);
  return runner.Run(argc,argv);
  
}