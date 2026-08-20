cmake_minimum_required(VERSION 3.16)
project(climbot_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_robot_description_defaults test/test_robot_description_defaults.py)
  ament_add_pytest_test(test_wall_frame test/test_wall_frame.py)
endif()

ament_package()
