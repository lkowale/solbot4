cmake_minimum_required(VERSION 3.8)
project(solbot4_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MoveSegment.msg"
  "action/Move.action"
  "action/MoveSequence.action"
  "action/RunMoveSequenceLoop.action"
  DEPENDENCIES builtin_interfaces
)

ament_package()
