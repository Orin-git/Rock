cmake_minimum_required(VERSION 3.8)
project(xw_pc_nav_filter_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pc_nav_filter_node src/pc_nav_filter_node.cpp)
ament_target_dependencies(pc_nav_filter_node rclcpp sensor_msgs std_msgs)

install(TARGETS pc_nav_filter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
