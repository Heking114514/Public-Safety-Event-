ament_add_gtest(mission_manager_core_test core_test.cpp)
target_link_libraries(mission_manager_core_test mission_manager_core)
target_compile_definitions(mission_manager_core_test PRIVATE
  MISSION_MANAGER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")

ament_add_gtest(mission_manager_recovery_test
  recovery_test.cpp
  rfid_test.cpp
  ../src/environment.cpp
  ../src/node.cpp
  ../src/planning.cpp
  ../src/pose_recovery.cpp
  ../src/recovery.cpp
  ../src/road_recovery.cpp
  ../src/rfid.cpp
  ../src/services.cpp)
target_link_libraries(mission_manager_recovery_test mission_manager_core yaml-cpp)
ament_target_dependencies(mission_manager_recovery_test
  ament_index_cpp geometry_msgs mission_control_interfaces nav_msgs new_map_planner
  rclcpp rclcpp_action sensor_msgs std_msgs)
