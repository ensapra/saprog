#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "saprog/position_controller_node.hpp"
#include "saprog/readback_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto readback = std::make_shared<ReadbackNode>(options);
    auto controller = std::make_shared<PositionControllerNode>(options, readback->bus());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(readback);
    executor.add_node(controller);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("saprog"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
