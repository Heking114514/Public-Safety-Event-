cmake_minimum_required(VERSION 3.8)
project(visual_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(waypoint_navigator src/waypoint_navigator.cpp)
ament_target_dependencies(
  waypoint_navigator
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
)

install(TARGETS waypoint_navigator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch routes
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md NAVIGATION_CODE_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
