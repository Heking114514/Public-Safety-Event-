cmake_minimum_required(VERSION 3.10)
project(rectangle_odometry_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(rectangle_geometry src/geometry.cpp)
target_include_directories(rectangle_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(rectangle_route_node src/route_node.cpp)
target_link_libraries(rectangle_route_node rectangle_geometry)
ament_target_dependencies(rectangle_route_node geometry_msgs nav_msgs rclcpp std_msgs)

ament_python_install_package(rectangle_odometry_test)

install(TARGETS rectangle_geometry rectangle_route_node
  EXPORT rectangle_geometryTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md CHANGELOG.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/primitive_controller_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(rectangle_geometry_test test/geometry_test.cpp)
  target_link_libraries(rectangle_geometry_test rectangle_geometry)
  ament_add_pytest_test(rectangle_python_test test
    ENV
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"
      "PYTHONDONTWRITEBYTECODE=1")
endif()

ament_export_include_directories(include)
ament_export_targets(rectangle_geometryTargets HAS_LIBRARY_TARGET)
ament_package()
