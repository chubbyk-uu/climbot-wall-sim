cmake_minimum_required(VERSION 3.16)
project(climbot_common)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_provenance test/test_provenance.py)
  ament_add_pytest_test(test_hashing test/test_hashing.py)
  ament_add_pytest_test(test_atomic test/test_atomic.py)
endif()

ament_package()
