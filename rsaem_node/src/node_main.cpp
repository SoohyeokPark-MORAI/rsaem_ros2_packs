// Copyright 2024 JetsonAI CO., LTD.
//
// Author: Kate Kim

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "diff_drive_controller.hpp"
//#include "rsaem_node/rsaembot.hpp"

void help_print()
{
  printf("For rsaembot node : \n");
  printf("rsaembot [-i usb_port] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-i usb_port: Connected USB port with OpenCR.");
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    help_print();
    return 0;
  }

  rclcpp::init(argc, argv);

  std::string can_port = "can0";
  char * cli_options;
  cli_options = rcutils_cli_get_option(argv, argv + argc, "-i");
  if (nullptr != cli_options) {
    usb_port = std::string(cli_options);
  }

  rclcpp::executors::SingleThreadedExecutor executor;

  auto rsaembot = std::make_shared<jetsonai::rsaem::RSaemBot>(can_port);
  auto diff_drive_controller =
    std::make_shared<jetsonai::rsaem::DiffDriveController>(
    rsaembot->get_wheels()->separation,
    rsaembot->get_wheels()->radius);

  executor.add_node(rsaembot);
  executor.add_node(diff_drive_controller);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
