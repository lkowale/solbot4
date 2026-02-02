cmake_minimum_required(VERSION 3.8)
project(convert_geo_points_bt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  robot_localization
  geographic_msgs
  geometry_msgs
)

# BT node for converting geo points to map coordinates
add_library(convert_geo_points_bt_node SHARED
  src/convert_geo_points.cpp
)
ament_target_dependencies(convert_geo_points_bt_node ${dependencies})
target_compile_definitions(convert_geo_points_bt_node PRIVATE BT_PLUGIN_EXPORT)

# Install libraries
install(TARGETS
  convert_geo_points_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(convert_geo_points_bt_node)
ament_export_dependencies(${dependencies})

ament_package()
