cmake_minimum_required(VERSION 3.16)
project(climbot_control)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(line_tracker src/line_tracker.cpp)
target_compile_features(line_tracker PUBLIC cxx_std_17)
target_include_directories(line_tracker PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(line_tracker_node src/line_tracker_node.cpp)
target_compile_features(line_tracker_node PUBLIC cxx_std_17)
target_link_libraries(line_tracker_node line_tracker)
ament_target_dependencies(line_tracker_node geometry_msgs nav_msgs rclcpp)

install(TARGETS line_tracker line_tracker_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_line_tracker test/test_line_tracker.cpp)
  target_link_libraries(test_line_tracker line_tracker)
  target_include_directories(test_line_tracker PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(line_tracker)
ament_package()
