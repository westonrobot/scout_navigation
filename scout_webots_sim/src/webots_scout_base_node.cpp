/*
 * scout_webots_node.cpp
 *
 * Created on: Sep 26, 2019 23:03
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include <csignal>
#include <memory>

#include "scout_webots_sim/scout_webots_runner.hpp"

using namespace westonrobot;

std::shared_ptr<ScoutWebotsRunner> runner;

void Quit(int sig) { runner->Stop(); }

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "webots_scout_base_node");
  signal(SIGINT, Quit);

  ros::NodeHandle nh, pnh("~");

  runner = std::make_shared<ScoutWebotsRunner>(&nh, &pnh);
  runner->Run();

  return 0;
}