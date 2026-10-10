cmake_minimum_required(VERSION 3.10)
project(new_map_planner)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_safety REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visual_navigation REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Maneuver.msg"
  "msg/RoutePlan.msg"
  "srv/PlanRoute.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

add_library(new_map_planner_core
  src/collision.cpp
  src/config.cpp
  src/coverage.cpp
  src/geometry.cpp
  src/graph.cpp
  src/maneuver.cpp
  src/planner.cpp
  src/route.cpp
  src/stage.cpp
  src/transform.cpp)
target_include_directories(new_map_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(new_map_planner_core PRIVATE ${visual_navigation_INCLUDE_DIRS})
target_link_libraries(new_map_planner_core yaml-cpp)
ament_target_dependencies(new_map_planner_core navigation_safety visual_navigation)

add_executable(new_map_planner_node src/node.cpp)
target_link_libraries(new_map_planner_node new_map_planner_core)
ament_target_dependencies(new_map_planner_node
  ament_index_cpp geometry_msgs nav_msgs rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(new_map_planner_node "${cpp_typesupport_target}")

ament_python_install_package(new_map_planner_ui)
ament_python_install_package(new_map_mission)

install(TARGETS new_map_planner_core
  EXPORT new_map_planner_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS new_map_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md CHANGELOG.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/new_map_planner_ui
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(new_map_planner_test test/planner_test.cpp)
  target_link_libraries(new_map_planner_test new_map_planner_core)
  ament_target_dependencies(new_map_planner_test ament_index_cpp)
  target_compile_definitions(new_map_planner_test PRIVATE
    NEW_MAP_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(new_map_planner_ui_test test/test_ui_model.py
    ENV
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
      "PYTHONDONTWRITEBYTECODE=1")
  ament_add_pytest_test(new_map_planner_mission_test test/test_mission.py
    ENV
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
      "PYTHONDONTWRITEBYTECODE=1")
  ament_add_pytest_test(new_map_planner_interface_test test/test_interface.py
    ENV
      "PYTHONDONTWRITEBYTECODE=1")
endif()

ament_export_include_directories(include)
ament_export_targets(new_map_planner_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  geometry_msgs
  nav_msgs
  navigation_safety
  rosidl_default_runtime
  std_msgs
  visual_navigation
  yaml_cpp_vendor)
ament_package()
