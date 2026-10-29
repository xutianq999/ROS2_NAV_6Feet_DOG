cmake_minimum_required(VERSION 3.5)
project(rosapi)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()

install(PROGRAMS scripts/rosapi_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_test_action_type test/test_action_type.py)
  ament_add_pytest_test(${PROJECT_NAME}_test_stringify_field_types test/test_stringify_field_types.py)
  ament_add_pytest_test(${PROJECT_NAME}_test_typedefs test/test_typedefs.py)

  # Humble 的 apt 仓库没有 ament_cmake_mypy；装了才跑 mypy，否则跳过
  find_package(ament_cmake_mypy QUIET)
  if(ament_cmake_mypy_FOUND)
    ament_mypy()
  endif()
endif()
