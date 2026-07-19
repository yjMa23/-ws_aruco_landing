cmake_minimum_required(VERSION 3.8)
project(moving_deck_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(moving_deck_motion src/motion_profile.cpp)
target_compile_features(moving_deck_motion PUBLIC cxx_std_17)
target_include_directories(
  moving_deck_motion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(moving_deck_controller src/moving_deck_controller.cpp)
target_compile_features(moving_deck_controller PUBLIC cxx_std_17)
target_link_libraries(
  moving_deck_controller
  moving_deck_motion
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)
ament_target_dependencies(moving_deck_controller rclcpp std_srvs)

install(
  TARGETS moving_deck_motion moving_deck_controller
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
endif()

ament_export_include_directories(include)
ament_export_targets(export_moving_deck_motion HAS_LIBRARY_TARGET)
ament_package()
