cmake_minimum_required(VERSION 3.8)
project(moving_deck_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(moving_deck_motion src/motion_profile.cpp)
target_compile_features(moving_deck_motion PUBLIC cxx_std_17)
target_include_directories(
  moving_deck_motion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(deck_gnss_model src/gnss_sensor_model.cpp)
target_compile_features(deck_gnss_model PUBLIC cxx_std_17)
target_include_directories(
  deck_gnss_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(deck_gnss_model gz-math7::gz-math7)

add_executable(moving_deck_controller src/moving_deck_controller.cpp)
target_compile_features(moving_deck_controller PUBLIC cxx_std_17)
target_link_libraries(
  moving_deck_controller
  moving_deck_motion
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)
ament_target_dependencies(moving_deck_controller nav_msgs rclcpp std_msgs std_srvs)

add_executable(deck_gnss_simulator src/deck_gnss_simulator.cpp)
target_compile_features(deck_gnss_simulator PUBLIC cxx_std_17)
target_link_libraries(deck_gnss_simulator deck_gnss_model)
ament_target_dependencies(
  deck_gnss_simulator
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS moving_deck_motion deck_gnss_model moving_deck_controller deck_gnss_simulator
  EXPORT export_moving_deck_motion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(motion_profile_test test/motion_profile_test.cpp)
  target_link_libraries(motion_profile_test moving_deck_motion)

  ament_add_gtest(gnss_sensor_model_test test/gnss_sensor_model_test.cpp)
  target_link_libraries(gnss_sensor_model_test deck_gnss_model)
endif()

ament_export_include_directories(include)
ament_export_targets(export_moving_deck_motion HAS_LIBRARY_TARGET)
ament_package()
