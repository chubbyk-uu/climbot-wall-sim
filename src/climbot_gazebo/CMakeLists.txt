cmake_minimum_required(VERSION 3.16)
project(climbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    models/climbot/model.config
    models/climbot/model.sdf.xacro
  DESTINATION share/${PROJECT_NAME}/models/climbot
)

install(
  PROGRAMS
    scripts/calibrate_wall_slip.py
    scripts/evaluate_localization.py
    scripts/evaluate_coverage_execution.py
    scripts/evaluate_slip_compensation.py
    scripts/measure_normal_loads.py
    scripts/measure_turn_slip.py
    scripts/plot_wall_slip.py
    scripts/total_station_sim.py
    scripts/wall_imu_adapter.py
    scripts/wall_wheel_odom_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_command_routing test/test_command_routing.py)
  ament_add_pytest_test(test_coverage_metrics test/test_coverage_metrics.py)
  ament_add_pytest_test(test_execution_metrics test/test_execution_metrics.py)
  ament_add_pytest_test(test_model_defaults test/test_model_defaults.py)
  ament_add_pytest_test(
    test_normal_load_statistics test/test_normal_load_statistics.py)
  ament_add_pytest_test(test_safe_stop test/test_safe_stop.py)
  ament_add_pytest_test(test_trajectory_io test/test_trajectory_io.py)
  ament_add_pytest_test(test_turn_profile test/test_turn_profile.py)
  ament_add_pytest_test(
    test_wall_slip_sampling test/test_wall_slip_sampling.py)
endif()

ament_package()
