cmake_minimum_required(VERSION 3.8)
project(xw_wt901_imu_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(wt901_imu_node src/wt901_imu_node.cpp)
ament_target_dependencies(wt901_imu_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS wt901_imu_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
