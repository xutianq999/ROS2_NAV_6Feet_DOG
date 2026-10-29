cmake_minimum_required(VERSION 3.5)
project(rosbridge_library)

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()

if (BUILD_TESTING)
  ament_add_ros_isolated_pytest_test(test_capabilities "test/capabilities/")
  ament_add_ros_isolated_pytest_test(test_internal "test/internal/")

  # Humble 的 apt 仓库没有 ament_cmake_mypy；装了才跑 mypy，否则跳过
  find_package(ament_cmake_mypy QUIET)
  if(ament_cmake_mypy_FOUND)
    ament_mypy()
  endif()
endif()
