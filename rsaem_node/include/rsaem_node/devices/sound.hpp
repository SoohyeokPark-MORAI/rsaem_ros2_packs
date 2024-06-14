// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#ifndef RSAEM_NODE__DEVICES__SOUND_HPP_
#define RSAEM_NODE__DEVICES__SOUND_HPP_

#include <rsaem_msgs/srv/sound.hpp>

#include <memory>
#include <string>

#include "rsaem_node/devices/devices.hpp"

namespace jetsonai
{
namespace rsaem
{
namespace devices
{
class Sound : public Devices
{
public:
  static void request(
    rclcpp::Client<rsaem_msgs::srv::Sound>::SharedPtr client,
    rsaem_msgs::srv::Sound::Request req);

  explicit Sound(
    std::shared_ptr<rclcpp::Node> & nh,
    //std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "sound");

  void command(const void * request, void * response) override;

private:
  rclcpp::Service<rsaem_msgs::srv::Sound>::SharedPtr srv_;
};
}  // namespace devices
}  // namespace rsaem
}  // namespace jetsonai
#endif  // RSAEM_NODE__DEVICES__SOUND_HPP_
