// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "run_swath_action/run_swath_action_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<run_swath_action::RunSwathActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
