cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

add_library(aruco_detector_core src/marker_selector.cpp)
target_compile_features(aruco_detector_core PUBLIC cxx_std_17)
target_include_directories(
  aruco_detector_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(aruco_detector_node src/aruco_detector_node.cpp)
target_compile_features(aruco_detector_node PUBLIC cxx_std_17)
target_include_directories(
  aruco_detector_node
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(aruco_detector_node aruco_detector_core)
ament_target_dependencies(
  aruco_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  message_filters
)
target_link_libraries(aruco_detector_node ${OpenCV_LIBS})

install(
  TARGETS aruco_detector_core aruco_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(marker_selector_test test/marker_selector_test.cpp)
  target_link_libraries(marker_selector_test aruco_detector_core)
endif()

ament_package()
