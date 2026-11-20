cmake_minimum_required(VERSION 3.8)
project(saprog_dynamixel)

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(dynamixel_control_node
  src/main.cpp
  src/dynamixel_readback_node.cpp
  src/servo_bus.cpp
  src/servo_readings_publisher.cpp
)
target_include_directories(dynamixel_control_node PRIVATE include)
target_compile_features(dynamixel_control_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamixel_control_node dynamixel_sdk rcl rclcpp sensor_msgs std_srvs)

install(TARGETS dynamixel_control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(dynamixel_command_test
    test/dynamixel_command_test.cpp
    src/dynamixel_readback_node.cpp
    src/servo_bus.cpp
    src/servo_readings_publisher.cpp
  )
  target_include_directories(dynamixel_command_test BEFORE PRIVATE test/stubs include)
  target_compile_features(dynamixel_command_test PUBLIC cxx_std_17)
  ament_target_dependencies(dynamixel_command_test rcl rclcpp sensor_msgs std_srvs)
  set_tests_properties(dynamixel_command_test PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=183")
endif()

ament_package()
