// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "approach_swath_action/approach_swath_action_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<approach_swath_action::ApproachSwathActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
