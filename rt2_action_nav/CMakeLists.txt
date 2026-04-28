cmake_minimum_required(VERSION 3.8)
project(rt2_action_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(navigation_action_server SHARED src/navigation_action_server.cpp)
add_library(UI_action_client SHARED src/UI_action_client.cpp)

ament_target_dependencies(navigation_action_server rclcpp rclcpp_action action_interfaces geometry_msgs nav_msgs rclcpp_components tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(UI_action_client rclcpp rclcpp_action action_interfaces geometry_msgs nav_msgs rclcpp_components)

rclcpp_components_register_node(navigation_action_server PLUGIN "nav_action::RobotNavigatorActionServer" EXECUTABLE navigation_action_server_exe)
rclcpp_components_register_node(UI_action_client PLUGIN "nav_action::RobotNavigatorUI" EXECUTABLE UI_action_client_exe)

ament_export_targets(export_${PROJECT_NAME}_action_server)
ament_export_targets(export_${PROJECT_NAME}_action_client)

install(TARGETS
  navigation_action_server_exe
  UI_action_client_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS navigation_action_server
  EXPORT export_${PROJECT_NAME}_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS UI_action_client
  EXPORT export_${PROJECT_NAME}_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
