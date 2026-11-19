#include <cmath>
#include <limits>
#include <stdexcept>

#include "saprog/config.hpp"
#include "saprog/config_parameters.hpp"
#include "saprog/servo_commands.hpp"
#include "saprog/timeline_deadline.hpp"
#include "saprog/timeline_executor.hpp"

namespace
{
void check(bool condition)
{
  if (!condition) {
    throw std::runtime_error("Configuration regression check failed");
  }
}

void read_with(double poll_rate, const std::vector<int64_t> &ids)
{
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
      rclcpp::Parameter("device_name", "unused"),
      rclcpp::Parameter("baud_rate", 1000000),
      rclcpp::Parameter("protocol_version", 1.0),
      rclcpp::Parameter("poll_rate", poll_rate),
      rclcpp::Parameter("moving_speed", 1023),
      rclcpp::Parameter("torque_limit", 1023),
      rclcpp::Parameter("target_tolerance", 0.1),
      rclcpp::Parameter("status_return_level", 1),
      rclcpp::Parameter("verify_goal_registers", false),
      rclcpp::Parameter("servo_ids", ids),
      rclcpp::Parameter("zero_positions", std::vector<int64_t>(ids.size(), 512)),
      rclcpp::Parameter("directions", std::vector<int64_t>(ids.size(), -1)),
  });
  rclcpp::Node node("config_test", options);
  const auto config = read_config(node);
  check(config.servos.size() == ids.size());
}

void rejects(double rate, const std::vector<int64_t> &ids)
{
  try {
    read_with(rate, ids);
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("Invalid configuration was accepted");
}

SaprogConfig valid_config()
{
  SaprogConfig config{};
  config.device_name = "unused";
  config.baud_rate = 1000000;
  config.protocol_version = 1.0;
  config.poll_rate = 10.0;
  config.moving_speed = 100;
  config.torque_limit = 500;
  config.target_tolerance = 0.1;
  config.status_return_level = 1;
  config.servos = {{11, 512, -1}, {12, 400, 1}};
  return config;
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  read_with(1.0, {11, 12});
  rejects(1.0, {11, 11});
  for (const double rate : {0.0, -1.0, 1e-300, 1e300, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
    rejects(rate, {11, 12});
  }
  // Exercise both calibration directions, including encoder extremes.
  for (const int direction : {-1, 1}) {
    for (const uint16_t raw : {0, 350, 512, 1023}) {
      const double radians = saprog::ax::to_radians(raw, 512, direction);
      check(std::abs(saprog::ax::to_raw(radians, 512, direction) - raw) < 1e-9);
    }
  }
  auto pure_config = valid_config();
  validate_config(pure_config);
  check(configured_servo_index(pure_config, "servo_11") == 0);
  check(configured_servo_index(pure_config, "servo_12") == 1);
  check(configured_servo_index(pure_config, "servo_99") == pure_config.servos.size());
  check(is_startup_only_parameter("servo_ids"));
  check(is_startup_only_parameter("moving_speed"));
  check(!is_startup_only_parameter("lower_limits"));
  check(!is_startup_only_parameter("description"));
  pure_config.servos[1].id = pure_config.servos[0].id;
  try {
    validate_config(pure_config);
    throw std::runtime_error("Pure validation accepted duplicate servo IDs");
  } catch (const std::invalid_argument &) {
  }
  check(saprog::commands::position_tick(0.0, 512, -1, 350, 700) == 512);
  check(saprog::commands::position_tick(0.0, 400, 1, 350, 700) == 400);
  try {
    (void)saprog::commands::position_tick(std::numeric_limits<double>::quiet_NaN(), 512, 1, 0,
                                          1023);
    throw std::runtime_error("Command conversion accepted a non-finite target");
  } catch (const std::invalid_argument &) {
  }
  try {
    (void)saprog::commands::position_tick(100.0, 512, 1, 0, 1023);
    throw std::runtime_error("Command conversion accepted an out-of-limit target");
  } catch (const std::out_of_range &) {
  }
  std::chrono::steady_clock::time_point deadline;
  check(saprog::timeline::checked_deadline(0.0, std::chrono::steady_clock::now(), deadline));
  check(!saprog::timeline::checked_deadline(std::numeric_limits<double>::max(),
                                            std::chrono::steady_clock::time_point{}, deadline));
  check(!saprog::timeline::checked_deadline(
      1.0, std::chrono::steady_clock::time_point::max() - std::chrono::nanoseconds(1), deadline));
  using Clock = std::chrono::steady_clock;
  Clock::time_point fake_now{};
  std::vector<size_t> sent_steps;
  bool final_action_called = false;
  saprog::timeline::ExecutorHooks hooks;
  hooks.now = [&]() { return fake_now; };
  hooks.canceled = []() { return false; };
  hooks.wait_until = [&](size_t, Clock::time_point target, std::string &) {
    fake_now = target;
    return true;
  };
  hooks.send_motion = [&](size_t step, std::optional<Clock::time_point> scheduled,
                          std::string &) {
    check(scheduled.has_value() == (step != 0));
    sent_steps.push_back(step);
    return true;
  };
  hooks.wait_for_arrival = [&](size_t step, std::string &) {
    check(step == 2);
    return saprog::timeline::ArrivalStatus::arrived;
  };
  hooks.run_end_action = [&](bool torque, std::optional<Clock::time_point> latest_start,
                             std::string &) {
    check(torque && !latest_start.has_value());
    final_action_called = true;
    return true;
  };
  const saprog::timeline::ExecutionPlan plan{{{1, 1, 0.5}, {0, 1, 0.25}, {0, 0, 0.0}},
                                              false, true, true};
  const auto execution = saprog::timeline::execute(plan, hooks);
  check(execution.succeeded && execution.completed_steps == 3);
  check((sent_steps == std::vector<size_t>{1, 2}));
  check(final_action_called);
  // doStandUp.json: two timed motions followed by an arrival-controlled pose.
  // Use the action's wire values, which the controller forwards unchanged.
  sent_steps.clear();
  final_action_called = false;
  const saprog::timeline::ExecutionPlan stand_up{
      {{0, 1, 0.3}, {0, 1, 0.3}, {0, 0, 1.0}}, false, true, true};
  const auto stand_up_result = saprog::timeline::execute(stand_up, hooks);
  check(stand_up_result.succeeded && stand_up_result.completed_steps == 3);
  check((sent_steps == std::vector<size_t>{0, 1, 2}) && final_action_called);
  const auto sends_before_invalid_plan = sent_steps.size();
  const auto invalid_plan = saprog::timeline::execute({{{0, 3, 0.0}}, false, false, false}, hooks);
  check(!invalid_plan.succeeded && sent_steps.size() == sends_before_invalid_plan);

  hooks.wait_for_arrival = [](size_t, std::string &error) {
    error = "Stale telemetry";
    return saprog::timeline::ArrivalStatus::failed;
  };
  final_action_called = false;
  const auto stale = saprog::timeline::execute({{{0, 0, 0.0}}, false, true, true}, hooks);
  check(!stale.succeeded && stale.error == "Stale telemetry" && !final_action_called);

  hooks.wait_for_arrival = [](size_t, std::string &error) {
    error = "Timed out waiting for joint arrival";
    return saprog::timeline::ArrivalStatus::timed_out;
  };
  const auto timed_out = saprog::timeline::execute({{{0, 0, 0.0}}, false, true, true}, hooks);
  check(!timed_out.succeeded && !timed_out.canceled && timed_out.timed_out &&
        !final_action_called);

  hooks.wait_for_arrival = [](size_t, std::string &) {
    return saprog::timeline::ArrivalStatus::arrived;
  };
  hooks.wait_until = [&](size_t, Clock::time_point target, std::string &) {
    fake_now = target + std::chrono::milliseconds(21);
    return true;
  };
  const auto late = saprog::timeline::execute({{{1, 1, 0.5}}, false, true, true}, hooks);
  check(!late.succeeded && late.error.find("deadline") != std::string::npos &&
        !final_action_called);

  bool cancel = false;
  hooks.canceled = [&]() { return cancel; };
  hooks.wait_until = [&](size_t, Clock::time_point, std::string &error) {
    cancel = true;
    error = "Timeline canceled";
    return false;
  };
  const auto canceled = saprog::timeline::execute({{{1, 1, 0.5}}, false, true, false}, hooks);
  check(canceled.canceled && !canceled.succeeded && !final_action_called);
  rclcpp::shutdown();
  return 0;
}
