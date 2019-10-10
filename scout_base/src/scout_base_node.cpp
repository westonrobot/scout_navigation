#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "scout_sdk/scout_base.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace wescore;

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "scout_odom");
    ros::NodeHandle node(""), private_node("~");

    // instantiate a robot object
    ScoutBase robot;
    ScoutROSMessenger messenger(&robot, &node);

    // fetch parameters before connecting to robot
    std::string port_name;
    private_node.param<std::string>("port_name", port_name, std::string("can0"));
    private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));
    private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false);

    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos)
    {
        robot.Connect(port_name);
        ROS_INFO("Using CAN bus to talk with the robot");
    }
    else
    {
        robot.Connect(port_name, 115200);
        ROS_INFO("Using UART to talk with the robot");
    }
    messenger.SetupSubscription();

    // publish robot state at 20Hz while listening to twist commands
    ros::Rate rate_20hz(20); // 20Hz
    while (true)
    {
        messenger.PublishStateToROS();
        ros::spinOnce();
        rate_20hz.sleep();
    }

    return 0;
}