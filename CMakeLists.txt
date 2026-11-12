cmake_minimum_required(VERSION 3.8)
project(mission_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(crl_commander_interfaces REQUIRED)
find_package(commander_operator_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadMission.srv"
  "srv/SetMode.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ── BT nodes and the tree runner ──────────────────────────────────────────────
add_library(mission_bt_nodes SHARED
  src/request_monitor.cpp
  src/operator_request_action.cpp
  src/wait_for_message_action.cpp
  src/commander_switch_mode_action.cpp
  src/blackboard_parameters.cpp
  src/mission_behavior_tree.cpp
)
target_compile_features(mission_bt_nodes PUBLIC cxx_std_17)
target_include_directories(mission_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(mission_bt_nodes PUBLIC
  rclcpp::rclcpp
  behaviortree_cpp::behaviortree_cpp
  ament_index_cpp::ament_index_cpp
  ${crl_commander_interfaces_TARGETS}
  ${commander_operator_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  "${cpp_typesupport_target}"
)

# ── Tree runner node ──────────────────────────────────────────────────────────
add_executable(mission_bt_node
  src/mission_bt_node.cpp
)
target_link_libraries(mission_bt_node
  mission_bt_nodes
)

install(TARGETS mission_bt_nodes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mission_bt_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_request_monitor
    test/test_request_monitor.cpp
    src/request_monitor.cpp
  )
  target_include_directories(test_request_monitor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_request_monitor
    ${crl_commander_interfaces_TARGETS}
  )

  # Runs battery_patrol.xml (with its included subtrees) against fake
  # IsBatteryLow / OperatorRequest / WaitForMessage nodes, with the blackboard
  # filled from config/battery_patrol.yaml the same way mission_bt_node does.
  ament_add_gtest(test_battery_patrol_tree
    test/test_battery_patrol_tree.cpp
  )
  target_link_libraries(test_battery_patrol_tree
    mission_bt_nodes
  )
  target_compile_definitions(test_battery_patrol_tree PRIVATE
    BATTERY_PATROL_TREE_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/battery_patrol.xml"
    BATTERY_PATROL_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/battery_patrol.yaml"
  )

  ament_add_gtest(test_blackboard_parameters
    test/test_blackboard_parameters.cpp
  )
  target_link_libraries(test_blackboard_parameters
    mission_bt_nodes
  )

  ament_add_gtest(test_wait_for_message
    test/test_wait_for_message.cpp
  )
  target_link_libraries(test_wait_for_message
    mission_bt_nodes
    ${std_msgs_TARGETS}
  )

  ament_add_gtest(test_commander_switch_mode
    test/test_commander_switch_mode.cpp
  )
  target_link_libraries(test_commander_switch_mode
    mission_bt_nodes
  )

  # Reloads the real mission configs and trees in a running MissionBehaviorTree.
  ament_add_gtest(test_load_mission
    test/test_load_mission.cpp
  )
  target_link_libraries(test_load_mission
    mission_bt_nodes
  )
  target_compile_definitions(test_load_mission PRIVATE
    BATTERY_PATROL_TREE_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/battery_patrol.xml"
    BATTERY_PATROL_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/battery_patrol.yaml"
    STOP_TREE_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/stop.xml"
    STOP_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/stop.yaml"
    PATROL_HOME_STOP_TREE_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/patrol_home_stop.xml"
    PATROL_HOME_STOP_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/patrol_home_stop.yaml"
  )

  # Runs patrol_home_stop.xml against fake nodes, switching its mode.
  ament_add_gtest(test_patrol_home_stop_tree
    test/test_patrol_home_stop_tree.cpp
  )
  target_link_libraries(test_patrol_home_stop_tree
    mission_bt_nodes
  )
  target_compile_definitions(test_patrol_home_stop_tree PRIVATE
    PATROL_HOME_STOP_TREE_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/patrol_home_stop.xml"
    PATROL_HOME_STOP_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/patrol_home_stop.yaml"
  )
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(mission_bt_nodes)
ament_export_dependencies(rclcpp behaviortree_cpp crl_commander_interfaces commander_operator_interfaces std_msgs rosidl_default_runtime)

ament_package()
