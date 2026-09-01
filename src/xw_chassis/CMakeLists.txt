cmake_minimum_required(VERSION 3.8)
project(xw_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xw_interfaces REQUIRED)

include_directories(include)

add_library(xw_chassis_proto STATIC
  src/serial_protocol.cpp
  src/bms_protocol.cpp
  src/posix_serial.cpp
)
target_include_directories(xw_chassis_proto PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(chassis_node src/chassis_node.cpp)
target_link_libraries(chassis_node xw_chassis_proto)
ament_target_dependencies(chassis_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  xw_interfaces
)

install(TARGETS chassis_node xw_chassis_proto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Python reference kept under python_legacy/ (not installed).
install(FILES python_legacy/README.md
  DESTINATION share/${PROJECT_NAME}/python_legacy
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_serial_protocol test/test_serial_protocol.cpp)
  if(TARGET test_serial_protocol)
    target_link_libraries(test_serial_protocol xw_chassis_proto)
  endif()
endif()

ament_package()
