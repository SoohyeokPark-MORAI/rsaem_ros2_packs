// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim


#ifndef RSAEM_NODE__DEVICES__DEVICES_HPP_
#define RSAEM_NODE__DEVICES__DEVICES_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

#include "rsaem_node/control_table.hpp"
//#include "turtlebot3_node/dynamixel_sdk_wrapper.hpp"

namespace jetsonai
{
namespace rsaem
{
extern const ControlTable extern_control_table;
namespace devices
{
class Devices
{
public:
  explicit Devices(
    std::shared_ptr<rclcpp::Node> & nh)
    //std::shared_ptr<rclcpp::Node> & nh,
    //std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
  : nh_(nh) //, dxl_sdk_wrapper_(dxl_sdk_wrapper)
  {
  }

  virtual void command(const void * request, void * response) = 0;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  //std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::ServicesQoS());
};
}  // namespace devices
}  // namespace rsaem
}  // namespace jetsonai
#endif  // RSAEM_NODE__DEVICES__DEVICES_HPP_
