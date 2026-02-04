cmake_minimum_required(VERSION 3.8)
project(approach_swath_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(solbot4_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  nav_msgs
  geometry_msgs
  solbot4_msgs
)

# Action server library (component)
add_library(approach_swath_action_server SHARED
  src/approach_swath_action_server.cpp
)
ament_target_dependencies(approach_swath_action_server ${dependencies})
rclcpp_components_register_nodes(approach_swath_action_server
  "approach_swath_action::ApproachSwathActionServer")

# Standalone executable
add_executable(approach_swath_action_node
  src/main.cpp
  src/approach_swath_action_server.cpp
)
ament_target_dependencies(approach_swath_action_node ${dependencies})

# Install
install(TARGETS
  approach_swath_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  approach_swath_action_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(approach_swath_action_server)
ament_export_dependencies(${dependencies})

ament_package()
