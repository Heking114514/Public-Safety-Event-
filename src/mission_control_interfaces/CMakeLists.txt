cmake_minimum_required(VERSION 3.8)
project(mission_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionHoldState.msg"
  "msg/Waypoint.msg"
  "msg/WaypointRoute.msg"
  "srv/SetMotionHold.srv"
  DEPENDENCIES std_msgs
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
