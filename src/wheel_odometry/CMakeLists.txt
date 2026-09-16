cmake_minimum_required(VERSION 3.10)
project(wheel_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(odometry_integrator
  src/odometry_integrator.cpp
  src/mcu_clock_mapper.cpp)
target_include_directories(odometry_integrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
target_link_libraries(wheel_odometry_node odometry_integrator)
ament_target_dependencies(
  wheel_odometry_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS odometry_integrator wheel_odometry_node
  EXPORT export_odometry_integrator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(_warn_deprecated_was_defined FALSE)
  if(DEFINED CMAKE_WARN_DEPRECATED)
    set(_warn_deprecated_was_defined TRUE)
    set(_warn_deprecated "${CMAKE_WARN_DEPRECATED}")
  endif()
  # Humble's vendored gtest still declares CMake 3.8. Suppress only that
  # external deprecation while the first test target initializes gtest.
  set(CMAKE_WARN_DEPRECATED OFF CACHE INTERNAL
    "Suppress external gtest_vendor deprecation during initialization" FORCE)
  ament_add_gtest(test_odometry_integrator test/test_odometry_integrator.cpp)
  if(_warn_deprecated_was_defined)
    set(CMAKE_WARN_DEPRECATED "${_warn_deprecated}" CACHE INTERNAL
      "Restored CMake deprecation warning setting" FORCE)
  else()
    unset(CMAKE_WARN_DEPRECATED CACHE)
    unset(CMAKE_WARN_DEPRECATED)
  endif()
  target_link_libraries(test_odometry_integrator odometry_integrator)
  ament_add_gtest(test_mcu_clock_mapper test/test_mcu_clock_mapper.cpp)
  target_link_libraries(test_mcu_clock_mapper odometry_integrator)
endif()

ament_export_targets(export_odometry_integrator HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
