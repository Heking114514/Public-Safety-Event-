cmake_minimum_required(VERSION 3.10)
project(mission_control_interfaces)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlTelemetry.msg"
  "msg/EventCandidate.msg"
  "msg/MissionState.msg"
  "msg/MotionHoldState.msg"
  "srv/ContinueRetreat.srv"
  "srv/ControlMission.srv"
  "srv/SetMotionHold.srv"
  "action/ReportEvent.action"
  DEPENDENCIES action_msgs geometry_msgs std_msgs
)

install(FILES README.md CHANGELOG.md
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
