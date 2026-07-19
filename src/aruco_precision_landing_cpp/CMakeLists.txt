cmake_minimum_required(VERSION 3.8)
project(aruco_precision_landing_cpp)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  px4_aruco_landing_node
  src/px4_aruco_landing_node.cpp
)
target_compile_features(px4_aruco_landing_node PUBLIC cxx_std_17)
target_include_directories(
  px4_aruco_landing_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  px4_aruco_landing_node
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
)

install(
  TARGETS px4_aruco_landing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
