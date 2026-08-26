cmake_minimum_required(VERSION 3.16)
project(climbot_mosaic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/build_local_matches
  scripts/build_overlap_candidates
  scripts/build_initial_projection
  scripts/validate_mosaic_inputs
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_mosaic_inputs test/test_mosaic_inputs.py)
  ament_add_pytest_test(test_candidates test/test_candidates.py)
  ament_add_pytest_test(test_matching test/test_matching.py)
  ament_add_pytest_test(test_projection test/test_projection.py)
endif()

ament_package()
