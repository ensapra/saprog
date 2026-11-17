cmake_minimum_required(VERSION 3.8)
project(saprog)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlayTimeline.action"
  DEPENDENCIES action_msgs)

add_executable(saprog_nodes
  src/main.cpp
  src/dynamixel_bus.cpp
  src/readback_node.cpp
  src/position_controller_node.cpp)
target_include_directories(saprog_nodes PRIVATE include)
ament_target_dependencies(saprog_nodes rclcpp rclcpp_action dynamixel_sdk sensor_msgs std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(saprog_nodes "${cpp_typesupport_target}")
target_link_libraries(saprog_nodes Threads::Threads)

install(TARGETS saprog_nodes
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/snapshot_window.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME snapshot_window)

install(PROGRAMS scripts/snapshot_player.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME snapshot_player)

install(PROGRAMS scripts/temperature_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME temperature_monitor)

install(FILES scripts/snapshot_units.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/snapshot_arrival.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/snapshot_limits_client.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/desktop_style.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/motion_controls.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/timeline_model.py scripts/timeline_window.py DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  enable_testing()
  add_executable(dynamixel_bus_test test/dynamixel_bus_test.cpp src/dynamixel_bus.cpp)
  target_include_directories(dynamixel_bus_test PRIVATE test/stubs include)
  add_test(NAME dynamixel_bus_test COMMAND dynamixel_bus_test)
endif()

ament_package()
