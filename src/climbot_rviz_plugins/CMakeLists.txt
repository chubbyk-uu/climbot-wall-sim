cmake_minimum_required(VERSION 3.16)
project(climbot_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(climbot_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# The panel derives from QObject, so its header needs moc processing.
set(CMAKE_AUTOMOC ON)

add_library(climbot_rviz_plugins SHARED
  src/coverage_panel.cpp
  include/climbot_rviz_plugins/coverage_panel.hpp)
target_compile_features(climbot_rviz_plugins PUBLIC cxx_std_17)
target_include_directories(climbot_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(climbot_rviz_plugins Qt5::Core Qt5::Gui Qt5::Widgets)
ament_target_dependencies(climbot_rviz_plugins
  climbot_interfaces pluginlib rclcpp rviz_common std_srvs)
target_compile_definitions(climbot_rviz_plugins PRIVATE "QT_NO_KEYWORDS")

install(TARGETS climbot_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins_description.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_coverage_panel_plugin test/test_coverage_panel_plugin.cpp)
  ament_target_dependencies(test_coverage_panel_plugin pluginlib rviz_common)
  ament_add_gtest(test_coverage_panel_layout test/test_coverage_panel_layout.cpp)
  target_link_libraries(test_coverage_panel_layout climbot_rviz_plugins)
  ament_target_dependencies(test_coverage_panel_layout
    climbot_interfaces rclcpp rviz_common)
  ament_add_gtest(test_request_gate test/test_request_gate.cpp)
  target_link_libraries(test_request_gate climbot_rviz_plugins)
  ament_add_gtest(test_coverage_panel_config test/test_coverage_panel_config.cpp)
  target_link_libraries(test_coverage_panel_config climbot_rviz_plugins)
  ament_target_dependencies(test_coverage_panel_config
    climbot_interfaces rclcpp rviz_common)
endif()

ament_export_include_directories(include)
ament_export_libraries(climbot_rviz_plugins)
ament_export_dependencies(climbot_interfaces rclcpp rviz_common std_srvs)
ament_package()
