#ifndef IMU_EXTENSION_HPP
#define IMU_EXTENSION_HPP

#include "scout_webots_sim/scout_webots_extension.hpp"
#include <sensor_msgs/Imu.h>
namespace westonrobot
{
 class IMUExtension : public WebotsExtension
    {

    public:
        void setup(ros::NodeHandle &nh_, std::string robot_name_, tf2_ros::StaticTransformBroadcaster &static_broadcaster_) override;
        ~IMUExtension() = default;
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