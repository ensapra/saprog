cmake_minimum_required(VERSION 3.8)
project(saprog)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

option(SAPROG_BUILD_ROBOT_VIEW "Build the embedded RViz robot view" ON)
if(SAPROG_BUILD_ROBOT_VIEW)
  find_package(rviz_common REQUIRED)
  find_package(rviz_rendering REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  find_package(X11 REQUIRED)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlayTimeline.action"
  "msg/ImuStatus.msg"
  "srv/ConnectImu.srv"
  DEPENDENCIES action_msgs std_msgs)

add_executable(saprog_nodes
  src/main.cpp
  src/dynamixel_bus.cpp
  src/dynamixel_register_transport.cpp
  src/readback_node.cpp
  src/position_controller_node.cpp)
target_include_directories(saprog_nodes PRIVATE include)
ament_target_dependencies(saprog_nodes rclcpp rclcpp_action dynamixel_sdk sensor_msgs std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(saprog_nodes "${cpp_typesupport_target}")
target_link_libraries(saprog_nodes Threads::Threads)

add_executable(imu_node src/imu_node.cpp src/imu_transport.cpp)
target_include_directories(imu_node PRIVATE include)
ament_target_dependencies(imu_node rclcpp rcl_interfaces sensor_msgs geometry_msgs std_msgs std_srvs)
target_link_libraries(imu_node "${cpp_typesupport_target}")

set(SAPROG_INSTALL_TARGETS saprog_nodes imu_node)
if(SAPROG_BUILD_ROBOT_VIEW)
  add_executable(robot_view_embed src/robot_view_embed.cpp)
  ament_target_dependencies(robot_view_embed rclcpp rviz_common rviz_rendering)
  target_link_libraries(robot_view_embed Qt5::Widgets X11::X11)
  list(APPEND SAPROG_INSTALL_TARGETS robot_view_embed)
endif()

install(TARGETS ${SAPROG_INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/ensure_imu_usb.bash"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/usb_attach.bash"
  DESTINATION share/${PROJECT_NAME}/scripts)

install(PROGRAMS scripts/snapshot_window.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME snapshot_window)

install(PROGRAMS scripts/snapshot_player.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME snapshot_player)

install(PROGRAMS scripts/static_animator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_animator)

install(PROGRAMS scripts/temperature_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME temperature_monitor)

install(PROGRAMS scripts/imu_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_monitor)

install(PROGRAMS scripts/robot_view_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_view_bridge)

# The package namespace lives beside installed window entry points, while
# writable preferences and snapshots remain in user/source workspace data.
install(DIRECTORY scripts/saprog_desktop
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py")

install(FILES
  scripts/snapshot_limits_client.py
  scripts/timeline_model.py
  scripts/timeline_window.py
  scripts/snapshot_window.py
  scripts/snapshot_player.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME saprog_pure_python_tests
    COMMAND "${Python3_EXECUTABLE}" -m unittest
      test_player_settings test_snapshot_units test_monitor_models test_playback_adapters
      test_project_data test_controller)
  set_tests_properties(saprog_pure_python_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
    LABELS "python;pure"
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  add_test(NAME saprog_ros_tk_python_tests
    COMMAND "${Python3_EXECUTABLE}" -m unittest
      test_async_failures test_snapshot_arrival test_snapshot_limits test_snapshot_window
      test_timeline test_runtime)
  set_tests_properties(saprog_ros_tk_python_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
    LABELS "python;ros;tk"
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:$ENV{PYTHONPATH}")
  add_test(NAME saprog_process_python_tests
    COMMAND "${Python3_EXECUTABLE}" -m unittest test_robot_view test_simulation)
  set_tests_properties(saprog_process_python_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
    LABELS "python;process"
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  add_executable(config_test test/config_test.cpp)
  target_include_directories(config_test PRIVATE include)
  ament_target_dependencies(config_test rclcpp)
  add_test(NAME config_test COMMAND config_test)
  set_tests_properties(config_test PROPERTIES LABELS "native;ros")
  add_executable(imu_filter_test test/imu_filter_test.cpp)
  add_executable(magnetometer_calibration_test test/magnetometer_calibration_test.cpp)
  target_include_directories(magnetometer_calibration_test PRIVATE include)
  add_test(NAME magnetometer_calibration_test COMMAND magnetometer_calibration_test)
  set_tests_properties(magnetometer_calibration_test PROPERTIES LABELS "native;pure")
  target_include_directories(imu_filter_test PRIVATE include)
  add_test(NAME imu_filter_test COMMAND imu_filter_test)
  set_tests_properties(imu_filter_test PROPERTIES LABELS "native;pure")
  add_executable(imu_transport_test test/imu_transport_test.cpp src/imu_transport.cpp)
  target_include_directories(imu_transport_test PRIVATE include)
  target_link_libraries(imu_transport_test Threads::Threads)
  add_test(NAME imu_transport_test COMMAND imu_transport_test)
  set_tests_properties(imu_transport_test PROPERTIES LABELS "native;pty;process")
  add_executable(imu_message_adapter_test test/imu_message_adapter_test.cpp)
  target_include_directories(imu_message_adapter_test PRIVATE include)
  ament_target_dependencies(imu_message_adapter_test sensor_msgs geometry_msgs)
  add_test(NAME imu_message_adapter_test COMMAND imu_message_adapter_test)
  set_tests_properties(imu_message_adapter_test PROPERTIES LABELS "native;ros_messages")
  add_executable(dynamixel_bus_test test/dynamixel_bus_test.cpp src/dynamixel_bus.cpp
    src/dynamixel_register_transport.cpp)
  target_include_directories(dynamixel_bus_test PRIVATE test/stubs include)
  add_test(NAME dynamixel_bus_test COMMAND dynamixel_bus_test)
  set_tests_properties(dynamixel_bus_test PROPERTIES LABELS "native;hardware_stub")
endif()

ament_package()
