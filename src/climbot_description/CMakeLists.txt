cmake_minimum_required(VERSION 3.16)
project(climbot_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# pybind11_add_module needs python3_add_library, which only exists once the
# Python development component has been found.
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# The wall transform has one implementation. C++ nodes link the library and
# the Python package binds to it, so the convention cannot drift between them.
add_library(climbot_description_core src/geometry.cpp src/wall_frame.cpp)
target_compile_features(climbot_description_core PUBLIC cxx_std_17)
target_include_directories(climbot_description_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(climbot_description_core PUBLIC yaml-cpp::yaml-cpp)
# The same objects are linked into the Python extension module, which is a
# shared object, so they must be position independent.
set_target_properties(climbot_description_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(_climbot_description SHARED src/bindings.cpp)
target_link_libraries(_climbot_description PRIVATE climbot_description_core)

install(TARGETS climbot_description_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS _climbot_description
  DESTINATION "${PYTHON_INSTALL_DIR}")

ament_export_include_directories(include)
ament_export_libraries(climbot_description_core)
# Consumers link the static core, so they need its yaml-cpp symbols too.
ament_export_dependencies(yaml-cpp)

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
  ament_add_pytest_test(
    test_inspection_camera_config test/test_inspection_camera_config.py)
  ament_add_pytest_test(test_wall_frame test/test_wall_frame.py)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry_cpp test/test_geometry.cpp)
  target_link_libraries(test_geometry_cpp climbot_description_core)
  ament_add_gtest(test_wall_frame_cpp test/test_wall_frame.cpp)
  target_link_libraries(test_wall_frame_cpp climbot_description_core)
endif()

ament_package()
