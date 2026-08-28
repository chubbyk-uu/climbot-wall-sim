cmake_minimum_required(VERSION 3.16)
project(climbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(wall_wheel_odom_adapter_core src/wall_wheel_odom_adapter.cpp)
target_compile_features(wall_wheel_odom_adapter_core PUBLIC cxx_std_17)
target_include_directories(wall_wheel_odom_adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(wall_wheel_odom_adapter_core nav_msgs)

add_executable(wall_wheel_odom_adapter_node src/wall_wheel_odom_adapter_node.cpp)
target_compile_features(wall_wheel_odom_adapter_node PUBLIC cxx_std_17)
target_link_libraries(wall_wheel_odom_adapter_node wall_wheel_odom_adapter_core)
ament_target_dependencies(wall_wheel_odom_adapter_node nav_msgs rclcpp)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS wall_wheel_odom_adapter_core wall_wheel_odom_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(
  FILES
    models/climbot/model.config
    models/climbot/model.sdf.xacro
  DESTINATION share/${PROJECT_NAME}/models/climbot
)

install(
  PROGRAMS
    scripts/calibrate_wall_slip.py
    scripts/camera_distortion_adapter.py
    scripts/evaluate_localization.py
    scripts/evaluate_coverage_execution.py
    scripts/evaluate_g1_camera.py
    scripts/evaluate_g2_inspection.py
    scripts/evaluate_slip_compensation.py
    scripts/gz_sim_supervisor.py
    scripts/measure_normal_loads.py
    scripts/measure_turn_band.py
    scripts/measure_turn_slip.py
    scripts/plot_wall_slip.py
    scripts/total_station_sim.py
    scripts/wall_imu_adapter.py
    scripts/wall_wheel_odom_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_wall_wheel_odom_adapter test/test_wall_wheel_odom_adapter.cpp)
  target_link_libraries(test_wall_wheel_odom_adapter wall_wheel_odom_adapter_core)
  target_include_directories(test_wall_wheel_odom_adapter PRIVATE include)
  ament_add_pytest_test(test_command_routing test/test_command_routing.py)
  ament_add_pytest_test(
    test_camera_distortion test/test_camera_distortion.py)
  ament_add_pytest_test(
    test_camera_projection test/test_camera_projection.py)
  ament_add_pytest_test(test_coverage_metrics test/test_coverage_metrics.py)
  ament_add_pytest_test(test_ekf_startup test/test_ekf_startup.py)
  ament_add_pytest_test(test_execution_metrics test/test_execution_metrics.py)
  ament_add_pytest_test(
    test_g2_inspection_metrics test/test_g2_inspection_metrics.py)
  ament_add_pytest_test(test_inspection_contract test/test_inspection_contract.py)
  ament_add_pytest_test(test_model_defaults test/test_model_defaults.py)
  ament_add_pytest_test(
    test_normal_load_statistics test/test_normal_load_statistics.py)
  ament_add_pytest_test(test_safe_stop test/test_safe_stop.py)
  ament_add_pytest_test(
    test_total_station_model test/test_total_station_model.py)
  ament_add_pytest_test(test_trajectory_io test/test_trajectory_io.py)
  ament_add_pytest_test(test_turn_slip_model test/test_turn_slip_model.py)
  ament_add_pytest_test(test_turn_profile test/test_turn_profile.py)
  ament_add_pytest_test(
    test_wall_slip_sampling test/test_wall_slip_sampling.py)
  ament_add_pytest_test(test_wall_texture test/test_wall_texture.py)
  ament_add_pytest_test(test_results_are_machine_readable
    test/test_results_are_machine_readable.py)
  ament_add_pytest_test(
    test_wall_world_matches_description
    test/test_wall_world_matches_description.py)
endif()

ament_package()
