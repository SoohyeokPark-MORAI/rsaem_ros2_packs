// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#ifndef RSAEM_NODE__SENSORS__SENSOR_STATE_HPP_
#define RSAEM_NODE__SENSORS__SENSOR_STATE_HPP_

#include <rsaem_msgs/msg/sensor_state.hpp>

#include <memory>
#include <string>

#include "rsaem_node/sensors/sensors.hpp"

namespace jetsonai
{
namespace rsaem
{
namespace sensors
{
class SensorState : public Sensors
{
public:
  explicit SensorState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "sensor_state",
    const bool & bumper_forward = false,
    const bool & bumper_backward = false,
    const bool & illumination = false,
    const bool & cliff = false,
    const bool & sonar = false);

  void publish(
    const rclcpp::Time & now) override;
    //const rclcpp::Time & now,
    //std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<rsaem_msgs::msg::SensorState>::SharedPtr pub_;

  bool bumper_forward_;
  bool bumper_backward_;
  bool illumination_;
  bool cliff_;
  bool sonar_;
};
}  // namespace sensors
}  // namespace rsaem
}  // namespace jetsonai
#endif  // RSAEM_NODE__SENSORS__SENSOR_STATE_HPP_
