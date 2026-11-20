#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "saprog_dynamixel/dynamixel_readback_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<saprog_dynamixel::DynamixelReadbackNode>(options));
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("saprog_dynamixel_control"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
