#ifndef SCOUT_WEBOTS_RUNNER_HPP
#define SCOUT_WEBOTS_RUNNER_HPP

#include "scout_webots_extension.hpp"
#include "scout_webots_interface.hpp"
#include <memory>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <iostream>

namespace westonrobot
{
    class ScoutWebotsRunner
    {
    public:
        ScoutWebotsRunner();
        int Run(int argc, char *argv[]);
        void AddExtension(std::shared_ptr<WebotsExtension> extensionPointer);
        void InitExtensions();

    private:        

        std::vector<std::shared_ptr<westonrobot::WebotsExtension>> extensions;

        ros::ServiceClient timeStepClient;
        webots_ros::set_int timeStepSrv;
        int controllerCount;
        std::vector<std::string> controllerList;

        static void Quit(int sig);
        static void ControllerNameCallback(const std_msgs::String::ConstPtr &name);
    };

} // namespace westonrobot
#endif