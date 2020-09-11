#ifndef LIDAR_EXTENSION_HPP
#define LIDAR_EXTENSION_HPP

#include "scout_webots_sim/scout_webots_extension.hpp"
#include <ros/ros.h>
#include <webots_ros/set_float.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/Imu.h>
namespace westonrobot
{

    class LidarExtension : public WebotsExtension
    {
    public:
        void setup(ros::NodeHandle &nh_, std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_)override;
        void subscriber_callback(const sensor_msgs::PointCloud::ConstPtr &msg);
        void publish_TF(std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_);
        ~LidarExtension() = default;

    private:
        ros::Subscriber sub;
        ros::Publisher pub;
    };
} // namespace westonrobot
#endif