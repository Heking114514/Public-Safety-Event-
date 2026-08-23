cmake_minimum_required(VERSION 3.10)
project(fused_odometry)

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
find_package(tf2_ros REQUIRED)

add_library(fusion_logic
  src/fusion_logic.cpp
  src/fusion_health.cpp)
target_include_directories(fusion_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(fusion_gate_node src/fusion_gate_node.cpp)
target_link_libraries(fusion_gate_node fusion_logic)
ament_target_dependencies(
  fusion_gate_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs)

add_executable(map_odom_correction_node src/map_odom_correction_node.cpp)
target_link_libraries(map_odom_correction_node fusion_logic)
ament_target_dependencies(
  map_odom_correction_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2_ros)

install(TARGETS fusion_logic fusion_gate_node map_odom_correction_node
  EXPORT export_fusion_logic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_fusion_logic test/test_fusion_logic.cpp)
  target_link_libraries(test_fusion_logic fusion_logic)
  ament_add_gtest(test_fusion_health test/test_fusion_health.cpp)
  target_link_libraries(test_fusion_health fusion_logic)
  ament_add_gtest(test_fusion_status_authority test/test_fusion_status_authority.cpp)
  target_link_libraries(test_fusion_status_authority fusion_logic)
  ament_add_gtest(test_sensor_input_validation test/test_sensor_input_validation.cpp)
  target_link_libraries(test_sensor_input_validation fusion_logic)
  ament_add_pytest_test(test_fusion_config test/test_fusion_config.py)
endif()

ament_export_targets(export_fusion_logic HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
