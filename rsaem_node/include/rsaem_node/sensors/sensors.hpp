// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#ifndef RSAEM_NODE__SENSORS__SENSORS_HPP_
#define RSAEM_NODE__SENSORS__SENSORS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

#include "rsaem_node/control_table.hpp"
//#include "rsaem_node/dynamixel_sdk_wrapper.hpp"

namespace jetsonai
{
namespace rsaem
{
extern const ControlTable extern_control_table;
namespace sensors
{
class Sensors
{
public:
  explicit Sensors(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & frame_id = "")
  : nh_(nh),
    frame_id_(frame_id)
  {
  }

  virtual void publish(
    const rclcpp::Time & now);
    //const rclcpp::Time & now,
    //std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) = 0;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::string frame_id_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
};
}  // namespace sensors
}  // namespace rsaem
}  // namespace jetsonai
#endif  // RSAEM_NODE__SENSORS__SENSORS_HPP_
