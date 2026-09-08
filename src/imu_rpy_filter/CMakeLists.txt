cmake_minimum_required(VERSION 3.10)
project(imu_rpy_filter)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Group the node by its actual responsibilities: initialization, sensor input,
# filter updates and corrections, output publishing, and diagnostics.
add_executable(imu_rpy_filter_node
  src/imu_rpy_filter_node.cpp
  src/node_init.cpp
  src/inputs.cpp
  src/update.cpp
  src/corrections.cpp
  src/output.cpp
  src/diagnostics.cpp)
target_include_directories(imu_rpy_filter_node PRIVATE include)
ament_target_dependencies(
  imu_rpy_filter_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS imu_rpy_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yaw_filter test/test_yaw_filter.cpp)
  target_include_directories(test_yaw_filter PRIVATE include)
endif()

ament_package()
