cmake_minimum_required(VERSION 3.16)
project(climbot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # This package owns no node and nothing depends on it, so it is the only
  # place a test may read across every package without creating an edge.
  ament_add_pytest_test(
    test_workspace_consistency test/test_workspace_consistency.py TIMEOUT 60)
endif()

ament_package()
