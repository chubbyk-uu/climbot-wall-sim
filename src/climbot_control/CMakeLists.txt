cmake_minimum_required(VERSION 3.16)
project(climbot_control)

find_package(ament_cmake REQUIRED)
find_package(climbot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(line_tracker src/line_tracker.cpp src/command_watchdog.cpp src/coverage_execution.cpp
  src/schedule_estimate.cpp src/scan_entry.cpp src/segment_arrival.cpp
  src/turn_profile.cpp src/travel_profile.cpp src/segment_duration.cpp)
target_compile_features(line_tracker PUBLIC cxx_std_17)
target_include_directories(line_tracker PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(line_tracker climbot_interfaces geometry_msgs)

add_executable(line_tracker_node src/line_tracker_node.cpp)
target_compile_features(line_tracker_node PUBLIC cxx_std_17)
target_link_libraries(line_tracker_node line_tracker)
ament_target_dependencies(line_tracker_node
  climbot_interfaces geometry_msgs nav_msgs rcl_interfaces rclcpp rclcpp_action std_msgs)

add_executable(cmd_vel_watchdog_node src/cmd_vel_watchdog_node.cpp)
target_compile_features(cmd_vel_watchdog_node PUBLIC cxx_std_17)
target_link_libraries(cmd_vel_watchdog_node line_tracker)
ament_target_dependencies(cmd_vel_watchdog_node geometry_msgs rclcpp std_msgs std_srvs)

add_executable(coverage_manager_node src/coverage_manager_node.cpp)
target_compile_features(coverage_manager_node PUBLIC cxx_std_17)
target_link_libraries(coverage_manager_node line_tracker)
ament_target_dependencies(coverage_manager_node
  climbot_interfaces geometry_msgs rclcpp rclcpp_action std_msgs std_srvs)

install(TARGETS line_tracker line_tracker_node cmd_vel_watchdog_node coverage_manager_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_line_tracker test/test_line_tracker.cpp)
  ament_add_gtest(test_control_clock test/test_control_clock.cpp)
  ament_target_dependencies(test_control_clock rclcpp)
  target_include_directories(test_control_clock PRIVATE include)
  ament_add_gtest(test_segment_arrival test/test_segment_arrival.cpp)
  target_link_libraries(test_segment_arrival line_tracker)
  ament_add_gtest(test_scan_entry test/test_scan_entry.cpp)
  target_link_libraries(test_scan_entry line_tracker)
  ament_add_gtest(test_schedule_estimate test/test_schedule_estimate.cpp)
  target_link_libraries(test_schedule_estimate line_tracker)
  ament_add_gtest(test_segment_duration test/test_segment_duration.cpp)
  target_link_libraries(test_segment_duration line_tracker)
  ament_add_gtest(test_travel_profile test/test_travel_profile.cpp)
  target_link_libraries(test_travel_profile line_tracker)
  target_link_libraries(test_line_tracker line_tracker)
  target_include_directories(test_line_tracker PRIVATE include)
  # Every launch test gets its own ROS_DOMAIN_ID. They all start real nodes on
  # well-known topic names, colcon runs packages in parallel, and a stray node
  # outliving its test is enough to make the next one latch the wrong data -
  # which has already been mistaken here for a regression in the planner.
  # Unique domains also let CTEST_PARALLEL_LEVEL be raised safely.
  add_launch_test(test/test_cmd_vel_watchdog_node.py TIMEOUT 30 ENV ROS_DOMAIN_ID=41)
  add_launch_test(test/test_line_tracker_alignment.py TIMEOUT 30 ENV ROS_DOMAIN_ID=42)
  add_launch_test(test/test_line_tracker_completion.py TIMEOUT 30 ENV ROS_DOMAIN_ID=43)
  add_launch_test(test/test_line_tracker_node.py TIMEOUT 30 ENV ROS_DOMAIN_ID=44)
  add_launch_test(test/test_coverage_executor.py TIMEOUT 120 ENV ROS_DOMAIN_ID=45)
  add_launch_test(test/test_coverage_manager.py TIMEOUT 30 ENV ROS_DOMAIN_ID=46)
  add_launch_test(test/test_coverage_progress.py TIMEOUT 60 ENV ROS_DOMAIN_ID=47)
  add_launch_test(test/test_coverage_manager_states.py TIMEOUT 120 ENV ROS_DOMAIN_ID=48)
  add_launch_test(test/test_coverage_manager_executor_loss.py TIMEOUT 120 ENV ROS_DOMAIN_ID=49)
  add_launch_test(test/test_coverage_executor_config.py TIMEOUT 60 ENV ROS_DOMAIN_ID=50)
  add_launch_test(test/test_coverage_arc_entry.py TIMEOUT 90 ENV ROS_DOMAIN_ID=51)
  add_launch_test(test/test_line_tracker_time_mode.py TIMEOUT 90 ENV ROS_DOMAIN_ID=52)
  add_launch_test(test/test_coverage_schedule_pose.py TIMEOUT 60 ENV ROS_DOMAIN_ID=53)
  add_launch_test(test/test_coverage_manager_late_callbacks.py TIMEOUT 120 ENV ROS_DOMAIN_ID=54)
  add_launch_test(test/test_coverage_manager_hold_release.py TIMEOUT 60 ENV ROS_DOMAIN_ID=57)
  add_launch_test(test/test_coverage_manager_forced_recovery.py TIMEOUT 60 ENV ROS_DOMAIN_ID=55)
  add_launch_test(test/test_coverage_manager_archive.py TIMEOUT 60 ENV ROS_DOMAIN_ID=56)
  ament_add_pytest_test(
    test_watchdog_rejects_bad_numbers test/test_watchdog_rejects_bad_numbers.py TIMEOUT 60)
endif()

ament_export_include_directories(include)
ament_export_libraries(line_tracker)
ament_package()
