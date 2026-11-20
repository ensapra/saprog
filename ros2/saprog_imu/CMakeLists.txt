cmake_minimum_required(VERSION 3.8)
project(saprog_imu)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_publisher_node
  src/imu_publisher_node.cpp
  src/imu_csv_parser.cpp
  src/serial_imu_reader.cpp
  src/imu_message_publisher.cpp
  src/imu_orientation_estimator.cpp
)
target_include_directories(imu_publisher_node PRIVATE include)
target_compile_features(imu_publisher_node PUBLIC cxx_std_17)
ament_target_dependencies(imu_publisher_node builtin_interfaces geometry_msgs rclcpp sensor_msgs)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS imu_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(imu_orientation_estimator_test
    test/imu_orientation_estimator_test.cpp
    src/imu_orientation_estimator.cpp
  )
  target_include_directories(imu_orientation_estimator_test PRIVATE include)
  target_compile_features(imu_orientation_estimator_test PUBLIC cxx_std_17)
endif()

ament_package()
