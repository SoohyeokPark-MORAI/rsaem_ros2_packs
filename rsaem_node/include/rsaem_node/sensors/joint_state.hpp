// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#ifndef RSAEM_NODE__SENSORS__JOINT_STATE_HPP_
#define RSAEM_NODE__SENSORS__JOINT_STATE_HPP_

#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>

#include "rsaem_node/sensors/sensors.hpp"

namespace jetsonai
{
namespace rsaem
{
namespace sensors
{
constexpr uint8_t JOINT_NUM = 2;

// ref) http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#goal-velocity104
constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

// 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
constexpr double TICK_TO_RAD = 0.001533981;

class JointState : public Sensors
{
public:
  explicit JointState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "joint_states",
    const std::string & frame_id = "base_link");

  void publish( 
    const rclcpp::Time & now) override;
    //const rclcpp::Time & now,
    //std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};
}  // namespace sensors
}  // namespace rsaem
}  // namespace jetsonai
#endif  // RSAEM_NODE__SENSORS__JOINT_STATE_HPP_
