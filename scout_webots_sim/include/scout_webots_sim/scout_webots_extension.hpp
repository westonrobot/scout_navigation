#ifndef SCOUT_WEBOTS_EXTENSION_HPP
#define SCOUT_WEBOTS_EXTENSION_HPP
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ros/ros.h>

namespace westonrobot
{
    class WebotsExtension
    {
    public:
        virtual void setup(ros::NodeHandle& nh_, std::string robot_name_, tf2_ros::StaticTransformBroadcaster& static_broadcaster_) = 0;
        virtual ~WebotsExtension() =default;
        WebotsExtension() = default;
    };

} // namespace westonrobot

#endif