cmake_minimum_required(VERSION 3.16)
project(climbot_coverage)

find_package(ament_cmake REQUIRED)
find_package(climbot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(coverage_geometry src/coverage_geometry.cpp)
target_compile_features(coverage_geometry PUBLIC cxx_std_17)
target_include_directories(coverage_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(coverage_planner_node src/coverage_planner_node.cpp)
target_compile_features(coverage_planner_node PUBLIC cxx_std_17)
target_link_libraries(coverage_planner_node coverage_geometry)
ament_target_dependencies(coverage_planner_node
  climbot_interfaces geometry_msgs nav_msgs rclcpp std_msgs std_srvs visualization_msgs)

install(TARGETS coverage_geometry coverage_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
# scripts/ is deliberately not installed. make_rviz_window_state.py regenerates
# the dock layout inside rviz/coverage.rviz and needs PyQt5, which nothing else
# here depends on; it is run from the source tree when a panel is added or
# renamed, never at runtime. The linters still cover it.

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_coverage_geometry test/test_coverage_geometry.cpp)
  target_link_libraries(test_coverage_geometry coverage_geometry)
  target_include_directories(test_coverage_geometry PRIVATE include)
  # Every launch test gets its own ROS_DOMAIN_ID. They all start real nodes on
  # well-known topic names, colcon runs packages in parallel, and a stray node
  # outliving its test is enough to make the next one latch the wrong data -
  # which has already been mistaken here for a regression in the planner.
  # Unique domains also let CTEST_PARALLEL_LEVEL be raised safely.
  add_launch_test(test/test_coverage_planner_node.py TIMEOUT 30 ENV ROS_DOMAIN_ID=61)
  add_launch_test(test/test_coverage_planner_failure.py TIMEOUT 30 ENV ROS_DOMAIN_ID=62)
  add_launch_test(test/test_coverage_planner_rviz.py TIMEOUT 60 ENV ROS_DOMAIN_ID=63)
  add_launch_test(test/test_coverage_top_edge_scan.py TIMEOUT 90 ENV ROS_DOMAIN_ID=64)
  add_launch_test(test/test_coverage_configure.py TIMEOUT 90 ENV ROS_DOMAIN_ID=65)
  add_launch_test(test/test_coverage_wall_grid_off.py TIMEOUT 30 ENV ROS_DOMAIN_ID=66)
  ament_add_pytest_test(test_rviz_config test/test_rviz_config.py TIMEOUT 30)
  ament_add_pytest_test(
    test_planner_launch_grid test/test_planner_launch_grid.py TIMEOUT 30)
  ament_add_pytest_test(
    test_planner_rejects_bad_numbers test/test_planner_rejects_bad_numbers.py
    TIMEOUT 120)
endif()

ament_export_include_directories(include)
ament_export_libraries(coverage_geometry)
ament_package()
