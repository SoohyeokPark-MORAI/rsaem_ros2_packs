// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#ifndef RSAEM_NODE__SENSORS__BATTERY_STATE_HPP_
#define RSAEM_NODE__SENSORS__BATTERY_STATE_HPP_

#include <sensor_msgs/msg/battery_state.hpp>

#include <memory>
#include <string>

#include "rsaem_node/sensors/sensors.hpp"

namespace jetsonai
{
namespace rsaem
{
namespace sensors
{
class BatteryState : public Sensors
{
public:
  explicit BatteryState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "battery_state");

  void publish(
    const rclcpp::Time & now) override;
    //const rclcpp::Time & now,
    //std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
};
}  // namespace sensors
}  // namespace rsaem
}  // namespace jetsonai
#endif  // RSAEM_NODE__SENSORS__BATTERY_STATE_HPP_
