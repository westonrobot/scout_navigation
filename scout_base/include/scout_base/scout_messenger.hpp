/* 
 * scout_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MESSENGER_HPP
#define SCOUT_MESSENGER_HPP

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "scout/scout_base.hpp"

namespace wescore
{
class ScoutROSMessenger
{
public:
    ScoutROSMessenger(ScoutBase &scout, ros::NodeHandle nh);

    int control_rate_;
    std::string odom_frame_;
    std::string base_frame_;

    void SetupSubscription();
    void PublishStateToROS();

private:
    ScoutBase &scout_;
    ros::NodeHandle nh_;
    int32_t cmd_timeout_counter_ = 0;

    std::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;

    ros::Publisher odom_publisher_;
    ros::Subscriber cmd_subscriber_;
    tf::TransformBroadcaster tf_broadcaster_;

private:
    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    ros::Time last_time_;
    ros::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
};
} // namespace scout

#endif /* SCOUT_MESSENGER_HPP */
