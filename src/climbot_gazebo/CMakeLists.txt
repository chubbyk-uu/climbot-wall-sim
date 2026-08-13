cmake_minimum_required(VERSION 3.16)
project(climbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/calibrate_wall_slip.py
    scripts/evaluate_localization.py
    scripts/measure_normal_loads.py
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
  ament_add_pytest_test(test_wall_frame test/test_wall_frame.py)
endif()

ament_package()
