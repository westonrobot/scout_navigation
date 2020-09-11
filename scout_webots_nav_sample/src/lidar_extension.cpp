#include "scout_webots_nav_sample/lidar_extension.hpp"
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
    // Lidar_extension::Lidar_extension(){
    //     ROS_INFO("MADE LIDAR");
    // }
    void LidarExtension::setup(ros::NodeHandle &nh_, std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_)
    {
        std::string lidar_enable_srv_name = robot_name_ + "/rslidar/enable";
        if (ros::service::exists(lidar_enable_srv_name, true))
        {
            // enable lidar
            ros::ServiceClient enable_lidar_client;
            webots_ros::set_int enable_lidar_srv;
            enable_lidar_client =
                nh_.serviceClient<webots_ros::set_int>(lidar_enable_srv_name);
            enable_lidar_srv.request.value = 10;
            if (enable_lidar_client.call(enable_lidar_srv) &&
                enable_lidar_srv.response.success == 1)
            {
                ROS_INFO("Lidar Enabled.");

                // enable lidar pointcloud
                std::string lidar_enable_pc_srv_name =
                    robot_name_ + "/rslidar/enable_point_cloud";
                ros::ServiceClient enable_lidar_pc_client;
                webots_ros::set_bool enable_lidar_pc_srv;
                enable_lidar_pc_client =
                    nh_.serviceClient<webots_ros::set_bool>(lidar_enable_pc_srv_name);
                enable_lidar_pc_srv.request.value = true;
                if (enable_lidar_pc_client.call(enable_lidar_pc_srv) &&
                    enable_lidar_pc_srv.response.success == 1)
                {
                    ROS_INFO("Lidar Pointcloud Enabled.");

                    sub = nh_.subscribe(
                        robot_name_ + "/rslidar/point_cloud", 20,
                        &LidarExtension::subscriber_callback, this);
                    pub =
                        nh_.advertise<sensor_msgs::PointCloud2>("/rslidar_points", 1);

                    // publish tf
                    publish_TF(robot_name_, static_broadcaster_);
                }
                else
                {
                    ROS_ERROR("Failed to enable Lidar Pointcloud");
                }
            }
            else
            {
                ROS_ERROR("Failed to enable Lidar");
            }
        }
    }

    void LidarExtension::subscriber_callback(
        const sensor_msgs::PointCloud::ConstPtr &msg)
    {
        sensor_msgs::PointCloud2 pc2_msg;
        sensor_msgs::convertPointCloudToPointCloud2(*msg.get(), pc2_msg);

        pc2_msg.header.stamp = ros::Time::now();
        pc2_msg.header.frame_id = "base_laser";

        // transform pointcloud
        Eigen::Matrix4f transform;
        Eigen::Quaternionf quat = Eigen::Quaternionf{
            Eigen::AngleAxisf{M_PI / 2.0, Eigen::Vector3f{1, 0, 0}}};
        transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
        transform(3, 0) = 0;
        transform(3, 1) = 0;
        transform(3, 2) = 0;
        transform(3, 3) = 1;
        sensor_msgs::PointCloud2 pc_transformed;
        pcl_ros::transformPointCloud(transform, pc2_msg, pc_transformed);

        // publish to ROS
        pub.publish(pc_transformed);
    }

    void LidarExtension::publish_TF(std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_)
    {
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "base_laser";
        static_transformStamped.child_frame_id = robot_name_ + "/rslidar";
        static_transformStamped.transform.translation.x = 0;
        static_transformStamped.transform.translation.y = 0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0);
        static_transformStamped.transform.rotation.x = quat.x();
        static_transformStamped.transform.rotation.y = quat.y();
        static_transformStamped.transform.rotation.z = quat.z();
        static_transformStamped.transform.rotation.w = quat.w();
        static_broadcaster_.sendTransform(static_transformStamped);
    }

 
  

} // namespace westonrobot