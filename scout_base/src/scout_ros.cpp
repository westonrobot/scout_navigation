#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "scout/scout_robot.h"

#define Send_Speed_Debug 0

using namespace std;
using namespace scout;

class ScoutROSMessenger
{
public:
    void run(void);

    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void SendSpeedCmdCallback(const ros::TimerEvent &);
    void PublishOdometry(void);

private:
    ScoutRobot robot_;

    boost::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;
    ros::Publisher odom_publisher_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::string odom_frame_;
    std::string base_frame_;

    int ctrl_rate_;
    std::string port_name_;
    int baud_rate_;

    double x;
    double y;
    double th;
    double vx;
    double vy;
    double vth;

    ros::Time last_time_;
    ros::Time current_time_;

    void PublishStateToROS(short linear, short angular);
    void SendCmdToRobot(geometry_msgs::Twist *current_twist_);
};

void ScoutROSMessenger::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    twist_mutex_.lock();
    current_twist_ = *msg.get();
    twist_mutex_.unlock();
}

void ScoutROSMessenger::SendSpeedCmdCallback(const ros::TimerEvent &)
{
    SendCmdToRobot(&current_twist_);
}

void ScoutROSMessenger::PublishOdometry(void)
{
    RobotState data;
    while (1)
    {
        if (robot_.QueryRobotState(&data))
            PublishStateToROS(data.linear, data.angular);
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }
}

void ScoutROSMessenger::PublishStateToROS(short linear, short angular)
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

    double Angular = angular;
    double Linear = linear;
    Angular /= 10000;
    Linear /= 10000;

    vth = Angular; //旋转角速度，在角度很小的情况下。约等于（SPEED右 - SPEED左）/ 轮子间距
    vx = Linear;   //直线X轴的速度 =  （SPEED右 - SPEED左） / 2
    vy = 0.0;      //2轮差速车，不能左右横移

    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = base_frame_;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tf_broadcaster_.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_publisher_.publish(odom);

    last_time_ = current_time_;
}

void ScoutROSMessenger::SendCmdToRobot(geometry_msgs::Twist *current_twist_)
{
    static unsigned char index = 0;
    index++;
    robot_.SendCommand({current_twist_->linear.x, current_twist_->angular.z, index++});
#if Send_Speed_Debug
    ROS_INFO_STREAM("send -> linear: " << cent_speed << "; angular: " << cmd_twist_rotation);
#endif
}

void ScoutROSMessenger::run(void)
{
    ros::NodeHandle private_node("~");
    ros::NodeHandle node;

    private_node.param<std::string>("port_name_", port_name_, std::string("/dev/ttyUSB0"));
    private_node.param<std::string>("odom_frame_", odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame_", base_frame_, std::string("base_footprint"));
    private_node.param<int>("baud_rate_", baud_rate_, 115200);
    private_node.param<int>("ctrl_rate_", ctrl_rate_, 10);

    // connect to scout
    robot_.ConnectSerialPort(port_name_, baud_rate_);
    if (!robot_.IsConnectionActive())
    {
        std::cerr << "Failed to connect to robot" << std::endl;
        return;
    }

    // Odometry publisher
    odom_publisher_ = node.advertise<nav_msgs::Odometry>(odom_frame_, 50);
    boost::thread parse_thread(boost::bind(&ScoutROSMessenger::PublishOdometry, this));

    // Cmd subscriber
    ros::Subscriber cmd_sub = node.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &ScoutROSMessenger::TwistCmdCallback, this); //不启用平滑包则订阅“cmd_vel”
    ros::Timer send_speed_timer = node.createTimer(ros::Duration(1.0 / ctrl_rate_), &ScoutROSMessenger::SendSpeedCmdCallback, this);

    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scout_odom");
    
    // start ROS messenger
    ScoutROSMessenger scout_ros;
    scout_ros.run();

    return 0;
}