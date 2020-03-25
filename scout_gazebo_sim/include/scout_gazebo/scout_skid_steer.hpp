/*
 * scout_skid_steer.hpp
 *
 * Created on: Mar 25, 2020 22:52
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_SKID_STEER_HPP
#define SCOUT_SKID_STEER_HPP

#include <string>

#include <ros/ros.h>

namespace wescore {
class ScoutSkidSteer {
 public:
  ScoutSkidSteer(std::string robot_name);

 private:
  std::string robot_name;
};
}  // namespace wescore

#endif /* SCOUT_SKID_STEER_HPP */
