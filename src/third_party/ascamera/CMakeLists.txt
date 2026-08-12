cmake_minimum_required(VERSION 3.8)
project(ascamera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

add_compile_options(-std=c++17)

execute_process(COMMAND gcc -dumpmachine OUTPUT_VARIABLE GCC_MACHINE OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND gcc -dumpversion OUTPUT_VARIABLE GCC_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Detected GCC machine: ${GCC_MACHINE}")
message(STATUS "Detected GCC version: ${GCC_VERSION}")
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)" GCC_VERSION_MATCH ${GCC_VERSION})
set(GCC_MAJOR_VERSION ${CMAKE_MATCH_1})
set(GCC_MINOR_VERSION ${CMAKE_MATCH_2})
message(STATUS "Detected GCC version: ${GCC_VERSION} (Major: ${GCC_MAJOR_VERSION}, Minor: ${GCC_MINOR_VERSION})")

if(GCC_MACHINE MATCHES "x86_64")
  if(NOT GCC_VERSION VERSION_LESS "5")
    set(LIB_DIR "x86_64-linux-gnu-gcc-5")
  else()
    set(LIB_DIR "x86_64-linux-gnu-gcc-${GCC_MAJOR_VERSION}.${GCC_MINOR_VERSION}")
  endif()
elseif(GCC_MACHINE MATCHES "aarch64")
  if(NOT GCC_VERSION VERSION_LESS "5")
    set(LIB_DIR "aarch64-linux-gnu-gcc-5")
  else()
    set(LIB_DIR "aarch64-linux-gnu-gcc-${GCC_MAJOR_VERSION}.${GCC_MINOR_VERSION}")
  endif()
elseif(GCC_MACHINE MATCHES "arm")
  set(LIB_DIR "arm-linux-gnueabihf-gcc-${GCC_MAJOR_VERSION}.${GCC_MINOR_VERSION}")
else()
  message(FATAL_ERROR "Unsupported GCC machine: ${GCC_MACHINE}")
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libs/lib/${LIB_DIR}")
  message(FATAL_ERROR "Vendor libs missing: libs/lib/${LIB_DIR}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/lib/${LIB_DIR}
)

add_executable(
  ascamera_node
  src/ascamera_node.cpp
  src/CameraPublisher.cpp
  src/CameraSrv.cpp
  src/Camera.cpp
  src/TfTreeFrameIdInfo.cpp
)
ament_target_dependencies(
  ascamera_node
  rclcpp std_msgs sensor_msgs pcl_conversions geometry_msgs tf2 tf2_ros
)
target_include_directories(ascamera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ascamera_node PUBLIC c_std_99 cxx_std_17)

# Install .so next to the binary; $ORIGIN keeps runtime linkage portable.
set_target_properties(ascamera_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/libs/lib/${LIB_DIR}"
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH FALSE
)

target_link_libraries(ascamera_node
  AngstrongCameraSdk
  ${PCL_LIBRARIES}
)

install(TARGETS ascamera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/lib/${LIB_DIR}/"
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.so*")

install(DIRECTORY launch configurationfiles scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
