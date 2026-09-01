cmake_minimum_required(VERSION 3.8)
project(xw_cmd_arbiter_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cmd_arbiter_node src/cmd_arbiter_node.cpp)
ament_target_dependencies(cmd_arbiter_node rclcpp geometry_msgs std_msgs)

install(TARGETS cmd_arbiter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
