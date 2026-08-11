cmake_minimum_required(VERSION 3.8)
project(moving_deck_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS graphics)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
find_package(gz-rendering8 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-utils2 REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Marine 仅从固定 VRX commit 导入 WAM-V 与 WaveVisual 所需最小内容；运行时不依赖完整 VRX/Fuel。
find_package(Git REQUIRED)
set(VRX_UPSTREAM_REPOSITORY "https://github.com/osrf/vrx.git")
set(VRX_UPSTREAM_BRANCH "jazzy")
set(VRX_UPSTREAM_COMMIT "7609d1bd90ce7edb29d040a082f949e8b089c864")
set(VRX_SOURCE_STAGE_DIR "${CMAKE_CURRENT_BINARY_DIR}/vrx_upstream")
set(VRX_ASSET_STAGE_DIR "${CMAKE_CURRENT_BINARY_DIR}/vrx_assets")

function(clone_vrx_upstream)
  set(needs_clone TRUE)
  if(EXISTS "${VRX_SOURCE_STAGE_DIR}/.git")
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${VRX_SOURCE_STAGE_DIR}" rev-parse HEAD
      RESULT_VARIABLE revision_result
      OUTPUT_VARIABLE existing_revision
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(revision_result EQUAL 0 AND existing_revision STREQUAL VRX_UPSTREAM_COMMIT)
      set(needs_clone FALSE)
    else()
      file(REMOVE_RECURSE "${VRX_SOURCE_STAGE_DIR}")
    endif()
  endif()

  if(needs_clone)
    execute_process(
      COMMAND
        "${GIT_EXECUTABLE}" -c http.version=HTTP/1.1 clone
        --filter=blob:none --no-checkout --depth 1 --branch "${VRX_UPSTREAM_BRANCH}"
        "${VRX_UPSTREAM_REPOSITORY}" "${VRX_SOURCE_STAGE_DIR}"
      RESULT_VARIABLE clone_result
      OUTPUT_QUIET
      ERROR_VARIABLE clone_error
      TIMEOUT 240
    )
    if(NOT clone_result EQUAL 0)
      file(REMOVE_RECURSE "${VRX_SOURCE_STAGE_DIR}")
      message(FATAL_ERROR "Failed to clone pinned VRX upstream: ${clone_error}")
    endif()
  endif()

  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${VRX_SOURCE_STAGE_DIR}" rev-parse HEAD
    RESULT_VARIABLE revision_result
    OUTPUT_VARIABLE revision
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_VARIABLE revision_error
  )
  if(NOT revision_result EQUAL 0 OR NOT revision STREQUAL VRX_UPSTREAM_COMMIT)
    message(FATAL_ERROR
      "VRX upstream revision mismatch: expected ${VRX_UPSTREAM_COMMIT}, got '${revision}': ${revision_error}")
  endif()
endfunction()

function(import_vrx_file relative_destination upstream_path expected_size)
  set(destination "${VRX_ASSET_STAGE_DIR}/${relative_destination}")
  get_filename_component(destination_dir "${destination}" DIRECTORY)
  file(MAKE_DIRECTORY "${destination_dir}")

  set(needs_import TRUE)
  if(EXISTS "${destination}")
    file(SIZE "${destination}" existing_size)
    if(existing_size EQUAL expected_size)
      set(needs_import FALSE)
    else()
      file(REMOVE "${destination}")
    endif()
  endif()

  if(needs_import)
    execute_process(
      COMMAND
        "${GIT_EXECUTABLE}" -c http.version=HTTP/1.1 -C "${VRX_SOURCE_STAGE_DIR}"
        show "${VRX_UPSTREAM_COMMIT}:${upstream_path}"
      RESULT_VARIABLE import_result
      OUTPUT_FILE "${destination}"
      ERROR_VARIABLE import_error
      TIMEOUT 240
    )
    if(NOT import_result EQUAL 0)
      file(REMOVE "${destination}")
      message(FATAL_ERROR "Failed to import VRX file ${upstream_path}: ${import_error}")
    endif()
  endif()

  file(SIZE "${destination}" imported_size)
  if(NOT imported_size EQUAL expected_size)
    message(FATAL_ERROR
      "VRX file size mismatch for ${upstream_path}: expected ${expected_size}, got ${imported_size}")
  endif()
endfunction()

function(import_vrx_asset relative_destination upstream_path expected_size)
  import_vrx_file("${relative_destination}" "${upstream_path}" "${expected_size}")
  get_filename_component(install_dir "${relative_destination}" DIRECTORY)
  install(
    FILES "${VRX_ASSET_STAGE_DIR}/${relative_destination}"
    DESTINATION "share/${PROJECT_NAME}/models/${install_dir}"
  )
endfunction()

clone_vrx_upstream()

import_vrx_asset(
  "vrx_wamv_landing/meshes/WAM-V-Base.dae"
  "vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V-Base.dae"
  911707
)
import_vrx_asset(
  "vrx_wamv_landing/meshes/WAM-V_Albedo.png"
  "vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Albedo.png"
  6636672
)
import_vrx_asset(
  "vrx_wamv_landing/meshes/WAM-V_Normal.png"
  "vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Normal.png"
  5036763
)
import_vrx_asset(
  "vrx_wamv_landing/meshes/WAM-V_Roughness.png"
  "vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Roughness.png"
  3134399
)
import_vrx_asset(
  "vrx_wamv_landing/meshes/WAM-V_Metalness.png"
  "vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Metalness.png"
  240962
)
import_vrx_asset(
  "vrx_ocean_visual/materials/textures/wave_normals.dds"
  "vrx_gz/models/coast_waves/materials/textures/wave_normals.dds"
  262271
)
import_vrx_asset(
  "vrx_ocean_visual/materials/textures/skybox_lowres.dds"
  "vrx_gz/models/coast_waves/materials/textures/skybox_lowres.dds"
  786560
)
import_vrx_asset(
  "vrx_ocean_visual/meshes/waterlow.dae"
  "vrx_gz/models/coast_waves/meshes/waterlow.dae"
  13323357
)
import_vrx_asset(
  "vrx_ocean_visual/materials/programs/GerstnerWaves_vs_330.glsl"
  "vrx_gz/models/coast_waves/materials/programs/GerstnerWaves_vs_330.glsl"
  4097
)
import_vrx_asset(
  "vrx_ocean_visual/materials/programs/GerstnerWaves_fs_330.glsl"
  "vrx_gz/models/coast_waves/materials/programs/GerstnerWaves_fs_330.glsl"
  2159
)

import_vrx_file(
  "vrx_wave_visual/WaveVisual.cc"
  "vrx_gz/src/WaveVisual.cc"
  15228
)
import_vrx_file(
  "vrx_wave_visual/WaveVisual.hh"
  "vrx_gz/src/WaveVisual.hh"
  1916
)
import_vrx_file(
  "vrx_wave_visual/Wavefield.cc"
  "vrx_gz/src/Wavefield.cc"
  26812
)
import_vrx_file(
  "vrx_wave_visual/Wavefield.hh"
  "vrx_gz/src/Wavefield.hh"
  11653
)

add_library(
  WaveVisual SHARED
  "${VRX_ASSET_STAGE_DIR}/vrx_wave_visual/WaveVisual.cc"
  "${VRX_ASSET_STAGE_DIR}/vrx_wave_visual/Wavefield.cc"
)
target_compile_features(WaveVisual PUBLIC cxx_std_17)
target_include_directories(WaveVisual PRIVATE "${VRX_ASSET_STAGE_DIR}/vrx_wave_visual")
target_link_libraries(
  WaveVisual
  PRIVATE
    Eigen3::Eigen
    gz-common5::gz-common5
    gz-math7::gz-math7
    gz-msgs10::gz-msgs10
    gz-plugin2::gz-plugin2
    gz-rendering8::gz-rendering8
    gz-sim8::core
    gz-transport13::gz-transport13
    gz-utils2::gz-utils2
)
install(TARGETS WaveVisual LIBRARY DESTINATION lib)

add_library(moving_deck_motion src/motion_profile.cpp)
target_compile_features(moving_deck_motion PUBLIC cxx_std_17)
target_include_directories(
  moving_deck_motion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(rigid_body_kinematics src/rigid_body_kinematics.cpp)
target_compile_features(rigid_body_kinematics PUBLIC cxx_std_17)
target_include_directories(
  rigid_body_kinematics
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
  rigid_body_kinematics
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
  TARGETS moving_deck_motion rigid_body_kinematics deck_gnss_model moving_deck_controller deck_gnss_simulator
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

  ament_add_gtest(rigid_body_kinematics_test test/rigid_body_kinematics_test.cpp)
  target_link_libraries(rigid_body_kinematics_test rigid_body_kinematics)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME marine_environment_tests
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_marine_environment.py
  )
  set_tests_properties(
    marine_environment_tests
    PROPERTIES
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
      ENVIRONMENT "VRX_ASSET_STAGE_DIR=${VRX_ASSET_STAGE_DIR}"
  )
endif()

ament_export_include_directories(include)
ament_export_targets(export_moving_deck_motion HAS_LIBRARY_TARGET)
ament_package()
