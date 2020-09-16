#include "scout_webots_nav_sample/IMU_extension.hpp"

#include <webots_ros/set_int.h>

#include <pcl_ros/transforms.h>

namespace westonrobot
{

    void IMUExtension::setup(ros::NodeHandle &nh_, std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_)
    {
        gyro_sub_ = nh_.subscribe(robot_name_ + "/gyro/values", 1,
                                  &IMUExtension::GyroNewDataCallback, this);
        accel_sub_ =
            nh_.subscribe(robot_name_ + "/accel/values", 1,
                          &IMUExtension::AccelNewDataCallback, this);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 1);

        std::string gyro_enable_srv_name = robot_name_ + "/gyro/enable";
        std::string accel_enable_srv_name = robot_name_ + "/accel/enable";

        if (ros::service::exists(gyro_enable_srv_name, true))
        {
            // enable gyro
            ros::ServiceClient enable_gyro_client;
            webots_ros::set_int enable_gyro_srv;
            enable_gyro_client =
                nh_.serviceClient<webots_ros::set_int>(gyro_enable_srv_name);
            enable_gyro_srv.request.value = 100;
            if (enable_gyro_client.call(enable_gyro_srv) &&
                enable_gyro_srv.response.success == 1)
                ROS_INFO("Gyro Enabled.");
            else
                ROS_ERROR("Failed to enable Gyro");

            // enable accel
            ros::ServiceClient enable_accel_client;
            webots_ros::set_int enable_accel_srv;
            enable_accel_client =
                nh_.serviceClient<webots_ros::set_int>(accel_enable_srv_name);
            enable_accel_srv.request.value = 100;
            if (enable_accel_client.call(enable_accel_srv) &&
                enable_accel_srv.response.success == 1)
            {
                ROS_INFO("Gyro Enabled.");

                // publish tf
                PublishIMUTF(robot_name_, static_broadcaster_);
            }
            else
            {
                ROS_ERROR("Failed to enable Gyro");
            }
        }
    }

    void IMUExtension::AccelNewDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        accel_data_ = *msg;
    }
    void IMUExtension::GyroNewDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        sensor_msgs::Imu imu_msg;
        imu_msg = *msg;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.linear_acceleration = accel_data_.linear_acceleration;
        imu_msg.linear_acceleration_covariance =
            accel_data_.linear_acceleration_covariance;
        imu_pub_.publish(imu_msg);
    }
    void IMUExtension::PublishIMUTF(std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_)
    {
        // publish tf
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "imu_link";
        static_transformStamped.child_frame_id = robot_name_ + "/imu";
        static_transformStamped.transform.translation.x = 0.32;
        static_transformStamped.transform.translation.y = 0;
        static_transformStamped.transform.translation.z = 0.18;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0);
        static_transformStamped.transform.rotation.x = quat.x();
        static_transformStamped.transform.rotation.y = quat.y();
        static_transformStamped.transform.rotation.z = quat.z();
        static_transformStamped.transform.rotation.w = quat.w();
        static_broadcaster_.sendTransform(static_transformStamped);
    }

} // namespace westonrobot