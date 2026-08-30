cmake_minimum_required(VERSION 3.16)
project(climbot_mosaic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/build_wall_mosaic
  scripts/build_pose_graph
  scripts/build_local_matches
  scripts/build_overlap_candidates
  scripts/build_initial_projection
  scripts/evaluate_diagnostic_mosaic
  scripts/inspect_diagnostic_mosaic
  scripts/preflight_diagnostic_coverage
  scripts/validate_mosaic_inputs
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_cli_install test/test_cli_install.py)
  ament_add_pytest_test(test_mosaic_inputs test/test_mosaic_inputs.py)
  ament_add_pytest_test(test_candidates test/test_candidates.py)
  ament_add_pytest_test(test_matching test/test_matching.py)
  ament_add_pytest_test(test_projection test/test_projection.py)
  ament_add_pytest_test(test_pose_graph test/test_pose_graph.py)
  ament_add_pytest_test(test_fusion test/test_fusion.py)
  ament_add_pytest_test(test_evidence_chain test/test_evidence_chain.py)
  ament_add_pytest_test(test_stage_provenance test/test_stage_provenance.py)
  ament_add_pytest_test(test_diagnostic_truth test/test_diagnostic_truth.py)
  ament_add_pytest_test(test_diagnostic_inspection test/test_diagnostic_inspection.py)
  ament_add_pytest_test(test_diagnostic_coverage_preflight test/test_diagnostic_coverage_preflight.py)
endif()

ament_package()
