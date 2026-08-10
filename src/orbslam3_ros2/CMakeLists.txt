cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

# Use the relocatable Sophus package bundled with this workspace so a clean
# colcon build does not depend on a manually supplied Sophus_DIR.
set(_BUNDLED_SOPHUS_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../deps/share/sophus/cmake")
if(EXISTS "${_BUNDLED_SOPHUS_DIR}/SophusConfig.cmake")
  if(NOT Sophus_DIR OR NOT EXISTS "${Sophus_DIR}/SophusConfig.cmake")
    set(Sophus_DIR "${_BUNDLED_SOPHUS_DIR}" CACHE PATH
      "Directory containing SophusConfig.cmake" FORCE)
  endif()
endif()
find_package(Sophus REQUIRED CONFIG)
unset(_BUNDLED_SOPHUS_DIR)

# Prefer the relocatable Pangolin installation shipped with this workspace.
# This also repairs stale caches that reference another checkout's
# build/pangolin directory and its non-existent generated include directory.
set(_BUNDLED_PANGOLIN_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../deps/lib/cmake/Pangolin")
if(EXISTS "${_BUNDLED_PANGOLIN_DIR}/PangolinConfig.cmake")
  if(NOT Pangolin_DIR
     OR NOT EXISTS "${Pangolin_DIR}/PangolinConfig.cmake"
     OR Pangolin_DIR MATCHES "[/\\\\]build[/\\\\]pangolin$")
    set(Pangolin_DIR "${_BUNDLED_PANGOLIN_DIR}" CACHE PATH
      "Directory containing PangolinConfig.cmake" FORCE)
  endif()
endif()
find_package(Pangolin REQUIRED CONFIG)
unset(_BUNDLED_PANGOLIN_DIR)
find_package(ORB_SLAM3 REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(mono Sophus::Sophus)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(rgbd Sophus::Sophus)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters
ORB_SLAM3 Pangolin)
target_link_libraries(stereo ${OpenCV_LIBS} Sophus::Sophus)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial
  rclcpp sensor_msgs cv_bridge message_filters nav_msgs geometry_msgs std_msgs
  diagnostic_msgs tf2 tf2_ros ORB_SLAM3 Pangolin)
target_link_libraries(stereo-inertial ${OpenCV_LIBS} Sophus::Sophus)

install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(FILES ${ORB_SLAM3_ROOT_DIR}/Vocabulary/ORBvoc.txt
  DESTINATION share/${PROJECT_NAME}/vocabulary)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pose_continuity_test test/pose_continuity_test.cpp)
  target_include_directories(pose_continuity_test PRIVATE include)
  target_link_libraries(pose_continuity_test Sophus::Sophus)

  ament_add_gtest(pose_velocity_estimator_test test/pose_velocity_estimator_test.cpp)
  target_include_directories(pose_velocity_estimator_test PRIVATE include)
  target_link_libraries(pose_velocity_estimator_test Sophus::Sophus)
endif()

ament_package()
