/* 
 * scout_messenger.cpp
 * 
 * Created on: Apr 26, 2019 22:14
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_base/scout_messenger.hpp"

namespace wescore
{
ScoutROSMessenger::ScoutROSMessenger(ScoutBase &scout, ros::NodeHandle nh) : scout_(scout), nh_(nh)
{
}

void ScoutROSMessenger::SetupSubscription()
{
    // odometry publisher
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(odom_frame_, 50);

    // cmd subscriber
    cmd_subscriber_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 5, &ScoutROSMessenger::TwistCmdCallback, this); //不启用平滑包则订阅“cmd_vel”
}

void ScoutROSMessenger::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    scout_.SetMotionCommand(msg->linear.x, msg->angular.z);
}

void ScoutROSMessenger::PublishStateToROS()
{
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    auto state = scout_.GetScoutState();
    double linear = state.linear_velocity;
    double angular = state.angular_velocity;

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(tf_msg);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;

    //publish the message
    odom_publisher_.publish(odom_msg);

    last_time_ = current_time_;
}
} // namespace scout