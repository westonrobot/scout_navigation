/*
 * scout_webots_interface.cpp
 *
 * Created on: Sep 26, 2019 23:19
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_webots_sim/scout_webots_interface.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>

#include <webots_ros/set_float.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>

namespace westonrobot {
ScoutWebotsInterface::ScoutWebotsInterface(ros::NodeHandle *nh,
                                           ScoutROSMessenger *msger,
                                           uint32_t time_step)
    : nh_(nh), messenger_(msger), time_step_(time_step) {}

void ScoutWebotsInterface::InitComponents(std::string controller_name) {
  // reset controller name
  robot_name_ = controller_name;

  SetupRobot();
  SetupLidar();
  SetupIMU();
}

void ScoutWebotsInterface::SetupRobot() {
  // init motors
  for (int i = 0; i < 4; ++i) {
    // position
    webots_ros::set_float set_position_srv;
    ros::ServiceClient set_position_client =
        nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    if (set_position_client.call(set_position_srv) &&
        set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.",
               motor_names_[i].c_str());
    else
      ROS_ERROR("Failed to call service set_position on motor %s.",
                motor_names_[i].c_str());

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = nh_->serviceClient<webots_ros::set_float>(
        robot_name_ + "/" + std::string(motor_names_[i]) +
        std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) &&
        set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
  }
}

void ScoutWebotsInterface::SetupLidar() {
  std::string lidar_enable_srv_name = robot_name_ + "/rslidar/enable";
  if (ros::service::exists(lidar_enable_srv_name, true)) {
    // enable lidar
    ros::ServiceClient enable_lidar_client;
    webots_ros::set_int enable_lidar_srv;
    enable_lidar_client =
        nh_->serviceClient<webots_ros::set_int>(lidar_enable_srv_name);
    enable_lidar_srv.request.value = 10;
    if (enable_lidar_client.call(enable_lidar_srv) &&
        enable_lidar_srv.response.success == 1) {
      ROS_INFO("Lidar Enabled.");

      // enable lidar pointcloud
      std::string lidar_enable_pc_srv_name =
          robot_name_ + "/rslidar/enable_point_cloud";
      ros::ServiceClient enable_lidar_pc_client;
      webots_ros::set_bool enable_lidar_pc_srv;
      enable_lidar_pc_client =
          nh_->serviceClient<webots_ros::set_bool>(lidar_enable_pc_srv_name);
      enable_lidar_pc_srv.request.value = true;
      if (enable_lidar_pc_client.call(enable_lidar_pc_srv) &&
          enable_lidar_pc_srv.response.success == 1) {
        ROS_INFO("Lidar Pointcloud Enabled.");

        pc_sub_ = nh_->subscribe(
            robot_name_ + "/rslidar/point_cloud", 20,
            &ScoutWebotsInterface::LidarNewPointCloudCallback, this);
        pc2_pub_ =
            nh_->advertise<sensor_msgs::PointCloud2>("/rslidar_points", 1);

        // publish tf
        PublishLidarTF();
      } else {
        ROS_ERROR("Failed to enable Lidar Pointcloud");
      }
    } else {
      ROS_ERROR("Failed to enable Lidar");
    }
  }
}

void ScoutWebotsInterface::SetupIMU() {
  gyro_sub_ = nh_->subscribe(robot_name_ + "/gyro/values", 1,
                             &ScoutWebotsInterface::GyroNewDataCallback, this);
  accel_sub_ =
      nh_->subscribe(robot_name_ + "/accel/values", 1,
                     &ScoutWebotsInterface::AccelNewDataCallback, this);
  imu_pub_ = nh_->advertise<sensor_msgs::Imu>("/imu", 1);

  std::string gyro_enable_srv_name = robot_name_ + "/gyro/enable";
  std::string accel_enable_srv_name = robot_name_ + "/accel/enable";

  if (ros::service::exists(gyro_enable_srv_name, true)) {
    // enable gyro
    ros::ServiceClient enable_gyro_client;
    webots_ros::set_int enable_gyro_srv;
    enable_gyro_client =
        nh_->serviceClient<webots_ros::set_int>(gyro_enable_srv_name);
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
        nh_->serviceClient<webots_ros::set_int>(accel_enable_srv_name);
    enable_accel_srv.request.value = 100;
    if (enable_accel_client.call(enable_accel_srv) &&
        enable_accel_srv.response.success == 1) {
      ROS_INFO("Gyro Enabled.");

      // publish tf
      PublishIMUTF();
    } else {
      ROS_ERROR("Failed to enable Gyro");
    }
  }
}

void ScoutWebotsInterface::UpdateSimState() {
  // constants for calculation
  constexpr double rotation_radius =
      std::hypot(ScoutParams::wheelbase / 2.0, ScoutParams::track / 2.0) * 2.0;
  constexpr double rotation_theta =
      std::atan2(ScoutParams::wheelbase, ScoutParams::track);

  // update robot state
  double wheel_speeds[4];
  for (int i = 0; i < 4; ++i) {
    webots_ros::get_float get_velocity_srv;
    ros::ServiceClient get_velocity_client =
        nh_->serviceClient<webots_ros::get_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/get_velocity"));

    if (get_velocity_client.call(get_velocity_srv)) {
      wheel_speeds[i] = get_velocity_srv.response.value;
    } else
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
  }
  float left_speed =
      (wheel_speeds[1] + wheel_speeds[2]) / 2.0 * ScoutParams::wheel_radius;
  float right_speed =
      (wheel_speeds[0] + wheel_speeds[3]) / 2.0 * ScoutParams::wheel_radius;
  left_speed = -left_speed;
  double linear_speed = (right_speed + left_speed) / 2.0;
  //   double angular_speed = (right_speed - left_speed) /
  //   ScoutParams::wheelbase;
  double angular_speed =
      (right_speed - left_speed) * std::cos(rotation_theta) / rotation_radius;

  //   std::cout << "left: " << left_speed << " , right : " << right_speed
  //             << " , linear: " << linear_speed << " , angular: " <<
  //             angular_speed
  //             << std::endl;
  messenger_->PublishSimStateToROS(linear_speed, angular_speed);

  // send robot command
  double linear, angular;
  messenger_->GetCurrentMotionCmdForSim(linear, angular);

  if (linear > ScoutParams::max_linear_speed)
    linear = ScoutParams::max_linear_speed;
  if (linear < -ScoutParams::max_linear_speed)
    linear = -ScoutParams::max_linear_speed;

  if (angular > ScoutParams::max_angular_speed)
    angular = ScoutParams::max_angular_speed;
  if (angular < -ScoutParams::max_angular_speed)
    angular = -ScoutParams::max_angular_speed;

  double vel_left_cmd = (linear - angular * ScoutParams::wheelbase / 2.0) /
                        ScoutParams::wheel_radius;
  double vel_right_cmd = (linear + angular * ScoutParams::wheelbase / 2.0) /
                         ScoutParams::wheel_radius;

  // left side motor are installed in a opposite direction, so negated
  double wheel_cmds[4];
  wheel_cmds[0] = vel_right_cmd;
  wheel_cmds[1] = -vel_left_cmd;
  wheel_cmds[2] = -vel_left_cmd;
  wheel_cmds[3] = vel_right_cmd;
  for (int i = 0; i < 4; ++i) {
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = nh_->serviceClient<webots_ros::set_float>(
        robot_name_ + "/" + std::string(motor_names_[i]) +
        std::string("/set_velocity"));

    set_velocity_srv.request.value = wheel_cmds[i];
    if (set_velocity_client.call(set_velocity_srv) &&
        set_velocity_srv.response.success == 1) {
      //   ROS_INFO("Velocity set to %.2f for motor %s.", wheel_cmds[i],
      //            motor_names_[i].c_str());
    } else {
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
    }
  }
}

void ScoutWebotsInterface::PublishLidarTF() {
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "base_laser";
  static_transformStamped.child_frame_id = robot_name_ + "/rslidar";
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster_.sendTransform(static_transformStamped);
}

void ScoutWebotsInterface::PublishIMUTF() {
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

void ScoutWebotsInterface::GyroNewDataCallback(
    const sensor_msgs::Imu::ConstPtr &msg) {
  sensor_msgs::Imu imu_msg;
  imu_msg = *msg;
  imu_msg.header.frame_id = "imu_link";
  imu_msg.linear_acceleration = accel_data_.linear_acceleration;
  imu_msg.linear_acceleration_covariance =
      accel_data_.linear_acceleration_covariance;
  imu_pub_.publish(imu_msg);
}

void ScoutWebotsInterface::AccelNewDataCallback(
    const sensor_msgs::Imu::ConstPtr &msg) {
  accel_data_ = *msg;
}

void ScoutWebotsInterface::LidarNewPointCloudCallback(
    const sensor_msgs::PointCloud::ConstPtr &msg) {
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
  pc2_pub_.publish(pc_transformed);
}
}  // namespace westonrobot