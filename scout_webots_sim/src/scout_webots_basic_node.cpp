/*
 * scout_webots_node.cpp
 *
 * Created on: Sep 26, 2019 23:03
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */


#include "scout_webots_sim/scout_webots_runner.hpp"
using namespace westonrobot;

int main(int argc, char *argv[])
{
  ScoutWebotsRunner runner;
  return runner.Run(argc,argv);
  
}