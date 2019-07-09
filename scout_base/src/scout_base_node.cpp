#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "scout/scout_base.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace wescore;

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "scout_odom");
    ros::NodeHandle node("scout_robot"), private_node("~");

    // instantiate a robot
    ScoutBase robot;
    ScoutROSMessenger messenger(&robot, node);

    std::string scout_can_port;
    private_node.param<std::string>("port_name", scout_can_port, std::string("can1"));
    private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_footprint"));
    private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false);

    // connect to scout and setup ROS subscription
    robot.ConnectCANBus(scout_can_port);
    robot.StartCmdThread(10);
    messenger.SetupSubscription();

    // publish robot state at 20Hz while listening to twist commands
    ros::Rate rt(20); // 20Hz
    while (true)
    {
        messenger.PublishStateToROS();
        ros::spinOnce();
        rt.sleep();
    }

    return 0;
}