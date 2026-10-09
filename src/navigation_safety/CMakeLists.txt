cmake_minimum_required(VERSION 3.10)
project(navigation_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(navigation_safety src/checker.cpp)
set_target_properties(navigation_safety PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(navigation_safety PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS navigation_safety
  EXPORT navigation_safetyTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md CHANGELOG.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(navigation_safety_test test/checker_test.cpp)
  target_link_libraries(navigation_safety_test navigation_safety)
endif()

ament_export_include_directories(include)
ament_export_targets(navigation_safetyTargets HAS_LIBRARY_TARGET)
ament_package()
