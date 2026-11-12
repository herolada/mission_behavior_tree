cmake_minimum_required(VERSION 3.8)
project(mission_behavior_tree_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadMission.srv"
  "srv/SetMode.srv"
)

ament_export_dependencies(std_msgs rosidl_default_runtime)

ament_package()
