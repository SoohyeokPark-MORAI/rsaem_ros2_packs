// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#include "odometry.hpp"

#include <memory>
#include <string>
#include <utility>


using namespace std::chrono_literals;

OdometryE::OdometryE(
  std::shared_ptr<rclcpp::Node> & nh,
  const double wheels_separation,
  const double wheels_radius)
: nh_(nh),
  wheels_separation_(wheels_separation),
  wheels_radius_(wheels_radius),
  use_imu_(false),
  publish_tf_(false),
  imu_angle_(0.0f)
{
  RCLCPP_INFO(nh_->get_logger(), "Init Odometry");

  nh_->declare_parameter("odometry.frame_id");
  nh_->declare_parameter("odometry.child_frame_id");
  nh_->declare_parameter("odometry.publish_tf");

  nh_->get_parameter_or<bool>(
    "odometry.publish_tf", publish_tf_, false);
  printf("publish_tf_:%d\n",publish_tf_);

  nh_->get_parameter_or<std::string>(
    "odometry.frame_id", frame_id_of_odometry_, std::string("odom"));
  printf("frame_id_of_odometry_:%s\n",frame_id_of_odometry_);

  nh_->get_parameter_or<std::string>(
    "odometry.child_frame_id", child_frame_id_of_odometry_, std::string("base_footprint"));
  printf("child_frame_id_of_odometry_:%s\n",child_frame_id_of_odometry_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);

  update_timer_ = this->create_wall_timer(30ms, std::bind(&OdometryE::joint_state_enc_callback, this));
  //joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
  //    "joint_states", qos, std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));
}

void OdometryE::joint_state_enc_callback(int32_t LeftEncoder, int32_t RightEncoder)
{
  int32_t LeftEncoder = 0;
  int32_t RightEncoder = 0;
  //static rclcpp::Time last_time = joint_state_msg->header.stamp;
  //rclcpp::Duration duration(joint_state_msg->header.stamp.nanosec - last_time.nanoseconds());
  static ros::Time current_time, last_time;
  current_time = ros::Time::now();
  rclcpp::Duration duration((current_time - last_time).toSec())

  update_joint_state(LeftEncoder, RightEncoder);
  calculate_odometry(duration);
  publish(joint_state_msg->header.stamp);

  //last_time = joint_state_msg->header.stamp;
  last_time = current_time;
}

void OdometryE::publish(const rclcpp::Time & now)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = frame_id_of_odometry_;
  odom_msg->child_frame_id = child_frame_id_of_odometry_;
  odom_msg->header.stamp = now;

  odom_msg->pose.pose.position.x = robot_pose_[0];
  odom_msg->pose.pose.position.y = robot_pose_[1];
  odom_msg->pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose_[2]);

  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = robot_vel_[0];
  odom_msg->twist.twist.angular.z = robot_vel_[2];

  geometry_msgs::msg::TransformStamped odom_tf;

  odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
  odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

  odom_tf.header.frame_id = frame_id_of_odometry_;
  odom_tf.child_frame_id = child_frame_id_of_odometry_;
  odom_tf.header.stamp = now;

  odom_pub_->publish(std::move(odom_msg));

  if (publish_tf_) {
    tf_broadcaster_->sendTransform(odom_tf);
  }
}

void OdometryE::update_joint_state(int32_t LeftEncoder, int32_t RightEncoder)
{
  static std::array<double, 2> last_joint_positions = {0.0f, 0.0f};

  diff_joint_positions_[0] = LeftEncoder - last_joint_positions[0];
  diff_joint_positions_[1] = RightEncoder - last_joint_positions[1];

  last_joint_positions[0] = LeftEncoder;
  last_joint_positions[1] = RightEncoder;
}

bool OdometryE::calculate_odometry(const rclcpp::Duration & duration)
{
  // rotation value of wheel [rad]
  double wheel_l = diff_joint_positions_[0];
  double wheel_r = diff_joint_positions_[1];

  double delta_s = 0.0;
  double delta_theta = 0.0;

  double theta = 0.0;
  static double last_theta = 0.0;

  double v = 0.0;
  double w = 0.0;

  double step_time = duration.seconds();

  if (step_time == 0.0) {
    return false;
  }

  if (std::isnan(wheel_l)) {
    wheel_l = 0.0;
  }

  if (std::isnan(wheel_r)) {
    wheel_r = 0.0;
  }

  delta_s = wheels_radius_ * (wheel_r + wheel_l) / 2.0;

  if (use_imu_) {
    theta = imu_angle_;
    delta_theta = theta - last_theta;
  } else {
    theta = wheels_radius_ * (wheel_r - wheel_l) / wheels_separation_;
    delta_theta = theta;
  }

  // compute odometric pose
  robot_pose_[0] += delta_s * cos(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[1] += delta_s * sin(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[2] += delta_theta;

  RCLCPP_DEBUG(nh_->get_logger(), "x : %f, y : %f", robot_pose_[0], robot_pose_[1]);

  // compute odometric instantaneouse velocity
  v = delta_s / step_time;
  w = delta_theta / step_time;

  robot_vel_[0] = v;
  robot_vel_[1] = 0.0;
  robot_vel_[2] = w;

  last_theta = theta;
  return true;
}
