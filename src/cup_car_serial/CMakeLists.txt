cmake_minimum_required(VERSION 3.10)
project(cup_car_serial)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mission_control_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(cup_car_serial_protocol
  src/mcu_clock_mapper.cpp
  src/protocol.cpp)
target_include_directories(cup_car_serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(cmd_vel_serial_node src/cmd_vel_serial_node.cpp)
target_link_libraries(cmd_vel_serial_node cup_car_serial_protocol)
ament_target_dependencies(
  cmd_vel_serial_node rclcpp geometry_msgs mission_control_interfaces sensor_msgs std_msgs)

install(TARGETS cmd_vel_serial_node cup_car_serial_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

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
  ament_add_gtest(protocol_test test/protocol_test.cpp)
  if(_warn_deprecated_was_defined)
    set(CMAKE_WARN_DEPRECATED "${_warn_deprecated}" CACHE INTERNAL
      "Restored CMake deprecation warning setting" FORCE)
  else()
    unset(CMAKE_WARN_DEPRECATED CACHE)
    unset(CMAKE_WARN_DEPRECATED)
  endif()
  target_link_libraries(protocol_test cup_car_serial_protocol)
  ament_add_gtest(test_mcu_clock_mapper test/test_mcu_clock_mapper.cpp)
  target_link_libraries(test_mcu_clock_mapper cup_car_serial_protocol)
  ament_add_gtest(actuator_tracking_monitor_test test/actuator_tracking_monitor_test.cpp)
  target_include_directories(actuator_tracking_monitor_test PRIVATE include)
endif()

ament_package()
