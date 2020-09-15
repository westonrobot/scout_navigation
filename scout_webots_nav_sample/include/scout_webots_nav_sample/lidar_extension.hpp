#ifndef LIDAR_EXTENSION_HPP
#define LIDAR_EXTENSION_HPP

#include "scout_webots_sim/scout_webots_extension.hpp"
#include <sensor_msgs/PointCloud.h>
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