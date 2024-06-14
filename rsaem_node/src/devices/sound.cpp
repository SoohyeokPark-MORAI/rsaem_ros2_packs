// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#include "rsaem_node/devices/sound.hpp"

#include <memory>
#include <string>

using jetsonai::rsaem::devices::Sound;

Sound::Sound(
  std::shared_ptr<rclcpp::Node> & nh,
  //std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
  const std::string & server_name)
//: Devices(nh, dxl_sdk_wrapper)
: Devices(nh)
{
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create sound server");
  srv_ = nh_->create_service<rsaem_msgs::srv::Sound>(
    server_name,
    [this](
      const std::shared_ptr<rsaem_msgs::srv::Sound::Request> request,
      std::shared_ptr<rsaem_msgs::srv::Sound::Response> response) -> void
    {
      this->command(static_cast<void *>(request.get()), static_cast<void *>(response.get()));
    }
  );
}

void Sound::command(const void * request, void * response)
{
  rsaem_msgs::srv::Sound::Request req = *(rsaem_msgs::srv::Sound::Request *)request;
  rsaem_msgs::srv::Sound::Response * res = (rsaem_msgs::srv::Sound::Response *)response;
/*
  res->success = dxl_sdk_wrapper_->set_data_to_device(
    extern_control_table.sound.addr,
    extern_control_table.sound.length,
    reinterpret_cast<uint8_t *>(&req.value),
    &res->message);*/
}

void Sound::request(
  rclcpp::Client<rsaem_msgs::srv::Sound>::SharedPtr client,
  rsaem_msgs::srv::Sound::Request req)
{
  auto request = std::make_shared<rsaem_msgs::srv::Sound::Request>(req);
  auto result = client->async_send_request(request);
}
