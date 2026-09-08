cmake_minimum_required(VERSION 3.8)
project(arena_path_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanArenaPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

add_library(arena_planner_core
  src/config.cpp
  src/geometry.cpp
  src/grid.cpp
  src/path.cpp
  src/coverage.cpp
  src/planner.cpp)
target_include_directories(arena_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(arena_planner_core yaml-cpp)

add_executable(arena_planner_node src/node.cpp)
target_link_libraries(arena_planner_node arena_planner_core)
ament_target_dependencies(arena_planner_node geometry_msgs nav_msgs rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(arena_planner_node "${cpp_typesupport_target}")

install(TARGETS arena_planner_core arena_planner_node
  EXPORT arena_planner_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include
  PATTERN "planner_internal.hpp" EXCLUDE)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(arena_planner_test test/planner_test.cpp)
  target_link_libraries(arena_planner_test arena_planner_core)
  ament_target_dependencies(arena_planner_test ament_index_cpp)
  ament_add_gtest(dynamic_obstacle_test test/dynamic_obstacle_test.cpp)
  target_include_directories(dynamic_obstacle_test PRIVATE include)
  target_link_libraries(dynamic_obstacle_test arena_planner_core)
  ament_add_gtest(request_validation_test test/request_validation_test.cpp)
  target_include_directories(request_validation_test PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_targets(arena_planner_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rosidl_default_runtime
  yaml_cpp_vendor)
ament_package()
