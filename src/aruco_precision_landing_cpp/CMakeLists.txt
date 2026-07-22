cmake_minimum_required(VERSION 3.8)
project(aruco_precision_landing_cpp)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(
  aruco_landing_math
  src/coordinate_transform.cpp
  src/geodetic_converter.cpp
  src/gnss_rendezvous_guidance.cpp
  src/visual_handover_guidance.cpp
  src/target_state_estimator.cpp
  src/motion_predictor.cpp
  src/moving_target_tracking_controller.cpp
)
target_compile_features(aruco_landing_math PUBLIC cxx_std_17)
target_include_directories(
  aruco_landing_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aruco_landing_math PUBLIC Eigen3::Eigen)

add_executable(
  px4_aruco_landing_node
  src/px4_aruco_landing_node.cpp
)
target_compile_features(px4_aruco_landing_node PUBLIC cxx_std_17)
target_link_libraries(px4_aruco_landing_node aruco_landing_math)
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
  nav_msgs
  sensor_msgs
  std_msgs
)

install(
  TARGETS aruco_landing_math
  EXPORT export_aruco_landing_math
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    coordinate_transform_test
    test/coordinate_transform_test.cpp
  )
  target_link_libraries(coordinate_transform_test aruco_landing_math)

  ament_add_gtest(
    geodetic_converter_test
    test/geodetic_converter_test.cpp
  )
  target_link_libraries(geodetic_converter_test aruco_landing_math)

  ament_add_gtest(
    gnss_rendezvous_guidance_test
    test/gnss_rendezvous_guidance_test.cpp
  )
  target_link_libraries(gnss_rendezvous_guidance_test aruco_landing_math)

  ament_add_gtest(
    visual_handover_guidance_test
    test/visual_handover_guidance_test.cpp
  )
  target_link_libraries(visual_handover_guidance_test aruco_landing_math)

  ament_add_gtest(
    target_state_estimator_test
    test/target_state_estimator_test.cpp
  )
  target_link_libraries(target_state_estimator_test aruco_landing_math)

  ament_add_gtest(
    motion_predictor_test
    test/motion_predictor_test.cpp
  )
  target_link_libraries(motion_predictor_test aruco_landing_math)

  ament_add_gtest(
    moving_target_tracking_controller_test
    test/moving_target_tracking_controller_test.cpp
  )
  target_link_libraries(moving_target_tracking_controller_test aruco_landing_math)
endif()

ament_export_dependencies(Eigen3)
ament_export_targets(export_aruco_landing_math HAS_LIBRARY_TARGET)
ament_package()
