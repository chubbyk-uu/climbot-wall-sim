cmake_minimum_required(VERSION 3.16)
project(climbot_control)

find_package(ament_cmake REQUIRED)
find_package(climbot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(line_tracker src/line_tracker.cpp src/command_watchdog.cpp src/coverage_execution.cpp
  src/turn_profile.cpp)
target_compile_features(line_tracker PUBLIC cxx_std_17)
target_include_directories(line_tracker PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(line_tracker climbot_interfaces geometry_msgs)

add_executable(line_tracker_node src/line_tracker_node.cpp)
target_compile_features(line_tracker_node PUBLIC cxx_std_17)
target_link_libraries(line_tracker_node line_tracker)
ament_target_dependencies(line_tracker_node
  climbot_interfaces geometry_msgs nav_msgs rclcpp rclcpp_action std_msgs)

add_executable(cmd_vel_watchdog_node src/cmd_vel_watchdog_node.cpp)
target_compile_features(cmd_vel_watchdog_node PUBLIC cxx_std_17)
target_link_libraries(cmd_vel_watchdog_node line_tracker)
ament_target_dependencies(cmd_vel_watchdog_node geometry_msgs rclcpp)

add_executable(coverage_manager_node src/coverage_manager_node.cpp)
target_compile_features(coverage_manager_node PUBLIC cxx_std_17)
target_link_libraries(coverage_manager_node line_tracker)
ament_target_dependencies(coverage_manager_node
  climbot_interfaces rclcpp rclcpp_action std_msgs std_srvs)

install(TARGETS line_tracker line_tracker_node cmd_vel_watchdog_node coverage_manager_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_line_tracker test/test_line_tracker.cpp)
  target_link_libraries(test_line_tracker line_tracker)
  target_include_directories(test_line_tracker PRIVATE include)
  add_launch_test(test/test_cmd_vel_watchdog_node.py TIMEOUT 30)
  add_launch_test(test/test_line_tracker_alignment.py TIMEOUT 30)
  add_launch_test(test/test_line_tracker_completion.py TIMEOUT 30)
  add_launch_test(test/test_line_tracker_node.py TIMEOUT 30)
  add_launch_test(test/test_coverage_executor.py TIMEOUT 120)
  add_launch_test(test/test_coverage_manager.py TIMEOUT 30)
endif()

ament_export_include_directories(include)
ament_export_libraries(line_tracker)
ament_package()
