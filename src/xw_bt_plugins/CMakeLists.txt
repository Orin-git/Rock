cmake_minimum_required(VERSION 3.8)
project(xw_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(xw_bt_plugins SHARED src/ultrasonic_proximity_node.cpp)
target_compile_definitions(xw_bt_plugins PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(xw_bt_plugins
  rclcpp
  rclcpp_components
  behaviortree_cpp_v3
  nav2_behavior_tree
  sensor_msgs
)

install(TARGETS xw_bt_plugins
  LIBRARY DESTINATION lib
)

ament_export_libraries(xw_bt_plugins)
ament_export_dependencies(rclcpp behaviortree_cpp_v3 nav2_behavior_tree)
ament_package()
