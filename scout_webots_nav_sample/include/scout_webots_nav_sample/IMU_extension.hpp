#ifndef IMU_EXTENSION_HPP
#define IMU_EXTENSION_HPP

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
 class IMUExtension : public WebotsExtension
    {

    public:
        void setup(ros::NodeHandle &nh_, std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_);
    private:
        ros::Subscriber gyro_sub_;
        ros::Subscriber accel_sub_;
        ros::Publisher imu_pub_;
        sensor_msgs::Imu accel_data_;
        void AccelNewDataCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void GyroNewDataCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void PublishIMUTF(std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_);
    };

} // namespace westonrobot
#endif