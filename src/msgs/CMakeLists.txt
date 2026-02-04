cmake_minimum_required(VERSION 3.8)
project(solbot4_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MoveSegment.msg"
  "msg/Pause.msg"
  "action/Move.action"
  "action/MoveSequence.action"
  "action/RunMoveSequenceLoop.action"
  "action/RunOneLine.action"
  "action/RunSwath.action"
  "action/RunApproachSwath.action"
  DEPENDENCIES builtin_interfaces geographic_msgs geometry_msgs
)

ament_package()
