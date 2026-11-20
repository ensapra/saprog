cmake_minimum_required(VERSION 3.8)
project(saprog_dynamixel)

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dynamixel_readback_node
  src/main.cpp
  src/dynamixel_readback_node.cpp
  src/servo_bus.cpp
  src/servo_readings_publisher.cpp
)
target_include_directories(dynamixel_readback_node PRIVATE include)
target_compile_features(dynamixel_readback_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamixel_readback_node dynamixel_sdk rclcpp sensor_msgs)

install(TARGETS dynamixel_readback_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
