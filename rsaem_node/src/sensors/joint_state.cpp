// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#include <array>

#include <memory>
#include <string>
#include <utility>

#include "rsaem_node/sensors/joint_state.hpp"

using jetsonai::rsaem::sensors::JointState;

JointState::JointState(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  pub_ = nh->create_publisher<sensor_msgs::msg::JointState>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create joint state publisher");
}

void JointState::publish(
    const rclcpp::Time & now)
  //const rclcpp::Time & now,
  //std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();

  static std::array<int32_t, JOINT_NUM> last_diff_position, last_position;
/*
  std::array<int32_t, JOINT_NUM> position =
  {dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_position_left.addr,
      extern_control_table.present_position_left.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_position_right.addr,
      extern_control_table.present_position_right.length)};

  std::array<int32_t, JOINT_NUM> velocity =
  {dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_velocity_left.addr,
      extern_control_table.present_velocity_left.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_velocity_right.addr,
      extern_control_table.present_velocity_right.length)};*/

  // std::array<int32_t, JOINT_NUM> current =
  //   {dxl_sdk_wrapper->get_data_from_device<int32_t>(
  //     extern_control_table.resent_current_left.addr,
  //     extern_control_table.resent_current_left.length),
  //   dxl_sdk_wrapper->get_data_from_device<int32_t>(
  //     extern_control_table.resent_current_right.addr,
  //     extern_control_table.resent_current_right.length)};

  msg->header.frame_id = this->frame_id_;
  msg->header.stamp = now;

  msg->name.push_back("wheel_left_joint");
  msg->name.push_back("wheel_right_joint");

  msg->position.push_back(TICK_TO_RAD * last_diff_position[0]);
  msg->position.push_back(TICK_TO_RAD * last_diff_position[1]);

  msg->velocity.push_back(RPM_TO_MS * velocity[0]);
  msg->velocity.push_back(RPM_TO_MS * velocity[1]);


  last_diff_position[0] += (position[0] - last_position[0]);
  last_diff_position[1] += (position[1] - last_position[1]);

  last_position = position;

  pub_->publish(std::move(msg));
}
