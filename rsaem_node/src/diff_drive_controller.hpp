// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#ifndef RSAEM_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define RSAEM_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "odometry.hpp"


class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}
  std::unique_ptr<Odometry> odometry_;

private:
  std::shared_ptr<rclcpp::Node> nh_;
};

#endif  // RSAEM_NODE__DIFF_DRIVE_CONTROLLER_HPP_
