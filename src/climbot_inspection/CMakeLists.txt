cmake_minimum_required(VERSION 3.16)
project(climbot_inspection)

find_package(ament_cmake REQUIRED)
find_package(climbot_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(capture_once_node src/capture_once_node.cpp)
target_compile_features(capture_once_node PUBLIC cxx_std_17)
ament_target_dependencies(capture_once_node
  rclcpp sensor_msgs std_msgs std_srvs)

add_executable(automatic_capture_node src/automatic_capture_node.cpp)
target_compile_features(automatic_capture_node PUBLIC cxx_std_17)
ament_target_dependencies(automatic_capture_node
  climbot_interfaces nav_msgs rclcpp sensor_msgs std_srvs)

install(TARGETS capture_once_node automatic_capture_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_launch_test(test/test_capture_once_node.py TIMEOUT 45 ENV ROS_DOMAIN_ID=91)
  add_launch_test(test/test_automatic_capture_node.py TIMEOUT 45 ENV ROS_DOMAIN_ID=92)
endif()

ament_package()
