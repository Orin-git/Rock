cmake_minimum_required(VERSION 3.8)
project(xw_safety_gate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xw_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(safety_gate_node src/safety_gate_node.cpp)
ament_target_dependencies(safety_gate_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  xw_interfaces
)
target_link_libraries(safety_gate_node nlohmann_json::nlohmann_json)

install(TARGETS safety_gate_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Python reference kept under python_legacy/ (not installed as a node).
install(FILES python_legacy/README.md
  DESTINATION share/${PROJECT_NAME}/python_legacy
  OPTIONAL
)

ament_package()
