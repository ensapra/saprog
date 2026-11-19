#include "saprog/position_controller_node.hpp"
#include "saprog/config_parameters.hpp"
#include "saprog/servo_commands.hpp"
#include "saprog/timeline_executor.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
constexpr double kArrivalTimeoutSeconds = 5.0;
constexpr double kTelemetryFreshSeconds = 2.5;
constexpr auto kTimelineFeedbackInterval = std::chrono::milliseconds(100);
constexpr auto kTimelineDeadlineLateness = std::chrono::milliseconds(20);
constexpr auto kTimelineBusLockTimeout = std::chrono::milliseconds(250);
} // namespace

PositionControllerNode::PositionControllerNode(const rclcpp::NodeOptions &options,
                                               std::shared_ptr<DynamixelBus> bus)
    : Node("saprog_position_controller", options), config_(read_config_parameters(*this)),
      bus_(std::move(bus))
{
  for (const auto *name : {"lower_limits", "upper_limits"}) {
    if (!has_parameter(name)) {
      declare_parameter<std::vector<int64_t>>(name, std::vector<int64_t>{});
    }
  }
  parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          if (is_startup_only_parameter(parameter.get_name()) ||
              ((parameter.get_name() == "lower_limits" || parameter.get_name() == "upper_limits") &&
               !setting_measured_limits_.load())) {
            result.successful = false;
            result.reason = "Controller configuration is startup-only; measured limits are backend-owned";
            return result;
          }
        }
        return result;
      });
  limits_service_ = create_service<std_srvs::srv::Trigger>(
      "/saprog/refresh_limits", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                       std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (timeline_active_) {
          response->success = false;
          response->message = "Cannot refresh limits while a timeline is executing";
        } else {
          response->success = refresh_limits(response->message);
        }
      });
  std::string limits_error;
  if (!refresh_limits(limits_error)) {
    RCLCPP_WARN(get_logger(), "Hardware limits unavailable: %s", limits_error.c_str());
  }
  RCLCPP_INFO(get_logger(),
              "Controller ready for %zu configured servos; startup leaves torque unchanged",
              config_.servos.size());

  command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/saprog/position_command", rclcpp::QoS(rclcpp::KeepLast(config_.servos.size() + 10)),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        if (timeline_active_) {
          RCLCPP_WARN(get_logger(),
                      "Ignoring position topic command while timeline action owns the bus");
          return;
        }
        const int64_t command_stamp_ns =
            static_cast<int64_t>(message->header.stamp.sec) * 1000000000LL +
            message->header.stamp.nanosec;
        if (command_stamp_ns != 0 && command_stamp_ns <= topic_command_barrier_ns_) {
          RCLCPP_WARN(get_logger(),
                      "Ignoring queued position command older than the completed timeline");
          return;
        }
        set_goal_position(message);
      });
  state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/saprog/joint_states", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) { on_joint_states(message); });
  for (const auto &servo : config_.servos) {
    const auto id = servo.id;
    torque_services_.push_back(create_service<std_srvs::srv::SetBool>(
        "/saprog/servo_" + std::to_string(id) + "/torque",
        [this, id](const std_srvs::srv::SetBool::Request::SharedPtr request,
                   std_srvs::srv::SetBool::Response::SharedPtr response) {
          if (timeline_active_) {
            response->success = false;
            response->message = "Cannot change torque while a timeline action owns the bus";
            return;
          }
          set_joint_torque(id, request, response);
        }));
  }
  timeline_action_server_ = rclcpp_action::create_server<PlayTimeline>(
      this, "/saprog/play_timeline",
      [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PlayTimeline::Goal> goal) {
        return handle_timeline_goal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<TimelineGoalHandle> goal_handle) {
        return handle_timeline_cancel(goal_handle);
      },
      [this](const std::shared_ptr<TimelineGoalHandle> goal_handle) {
        handle_timeline_accepted(goal_handle);
      });
}

PositionControllerNode::~PositionControllerNode()
{
  timeline_cancel_requested_ = true;
  timeline_cv_.notify_all();
  state_cv_.notify_all();
  if (timeline_thread_.joinable()) {
    timeline_thread_.join();
  }
}

bool PositionControllerNode::refresh_limits(std::string &error)
{
  limits_ready_ = false;
  std::vector<int64_t> lower, upper;
  error.clear();
  for (const auto &servo : config_.servos) {
    const auto id = servo.id;
    uint16_t low = 0, high = 0;
    std::string detail;
    if (!bus_->get_limits(id, low, high, detail)) {
      error += "servo_" + std::to_string(id) + ": " + detail + "; ";
    }
    lower.push_back(low);
    upper.push_back(high);
  }
  if (!error.empty()) {
    setting_measured_limits_ = true;
    set_parameters({rclcpp::Parameter("lower_limits", std::vector<int64_t>{}),
                    rclcpp::Parameter("upper_limits", std::vector<int64_t>{})});
    setting_measured_limits_ = false;
    return false;
  }
  for (size_t i = 0; i < config_.servos.size(); ++i) {
    config_.servos[i].lower_limit = static_cast<uint16_t>(lower[i]);
    config_.servos[i].upper_limit = static_cast<uint16_t>(upper[i]);
  }
  setting_measured_limits_ = true;
  set_parameters(
      {rclcpp::Parameter("lower_limits", lower), rclcpp::Parameter("upper_limits", upper)});
  setting_measured_limits_ = false;
  limits_ready_ = true;
  error = "Read fresh hardware limits for all configured servos";
  return true;
}

void PositionControllerNode::set_joint_torque(
    uint8_t id, const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
{
  std::string error;
  if (!request->data) {
    response->success = bus_->set_torque(id, false, error);
    response->message = response->success ? "Torque disabled" : error;
    return;
  }
  std::array<uint8_t, 8> data{};
  if (!limits_ready_) {
    response->success = false;
    response->message = "Hardware limits unavailable; reopen Snapshot Creator to retry";
    return;
  }
  if (!bus_->read(id, data, error)) {
    response->success = false;
    response->message = "Could not read present position: " + error;
    return;
  }
  const uint16_t raw = static_cast<uint16_t>(data[0] | (data[1] << 8));
  const auto servo = std::find_if(config_.servos.begin(), config_.servos.end(),
                                  [id](const auto &entry) { return entry.id == id; });
  if (servo == config_.servos.end() || raw < servo->lower_limit || raw > servo->upper_limit) {
    response->success = false;
    response->message = "Present position is outside the AX range";
    return;
  }
  response->success =
      bus_->set_position(id, raw, config_.moving_speed, config_.torque_limit, error);
  response->message = response->success ? "Holding freshly read present position" : error;
}

bool PositionControllerNode::validate_timeline_goal(const PlayTimeline::Goal &goal,
                                                    std::string &error) const
{
  if (!limits_ready_) {
    error = "Hardware limits are unavailable";
    return false;
  }
  std::vector<std::string> configured_names;
  configured_names.reserve(config_.servos.size());
  for (const auto &servo : config_.servos) {
    configured_names.push_back("servo_" + std::to_string(servo.id));
  }
  if (goal.joint_names != configured_names || goal.step_kinds.empty() ||
      goal.duration_modes.size() != goal.step_kinds.size() ||
      goal.durations.size() != goal.step_kinds.size()) {
    error = "Timeline joints or step metadata do not match the controller configuration";
    return false;
  }
  const size_t value_count = goal.step_kinds.size() * config_.servos.size();
  if (goal.positions.size() != value_count || goal.speeds.size() != value_count ||
      goal.torque_limits.size() != value_count || (goal.loop && goal.run_end_action)) {
    error = "Timeline command arrays have invalid lengths or lifecycle settings";
    return false;
  }
  for (size_t step = 0; step < goal.step_kinds.size(); ++step) {
    std::chrono::steady_clock::time_point checked;
    if (!saprog::timeline::checked_deadline(goal.durations[step],
                                           std::chrono::steady_clock::time_point{}, checked)) {
      error = "Timeline durations must be finite, non-negative and representable by steady_clock";
      return false;
    }
    if (goal.step_kinds[step] == 1) {
      if (goal.duration_modes[step] != 1) {
        error = "Wait steps must use timed duration mode";
        return false;
      }
      continue;
    }
    if (goal.step_kinds[step] != 0 || goal.duration_modes[step] > 1) {
      error = "Timeline contains an unknown step or duration mode";
      return false;
    }
    for (size_t joint = 0; joint < config_.servos.size(); ++joint) {
      const size_t index = step * config_.servos.size() + joint;
      const double position = goal.positions[index];
      const int speed = goal.speeds[index];
      const int torque = goal.torque_limits[index];
      if (!std::isfinite(position) || speed < 1 || speed > 1023 || torque < 1 || torque > 1023) {
        error = "Timeline motion values must be finite with speed and torque from 1 to 1023";
        return false;
      }
      try {
        (void)saprog::commands::position_tick(
            position, config_.servos[joint].zero_position, config_.servos[joint].direction,
            config_.servos[joint].lower_limit, config_.servos[joint].upper_limit);
      } catch (const std::exception &) {
        error =
            "Timeline target is outside measured hardware limits for " + configured_names[joint];
        return false;
      }
    }
  }
  error.clear();
  return true;
}

rclcpp_action::GoalResponse
PositionControllerNode::handle_timeline_goal(const rclcpp_action::GoalUUID &,
                                             std::shared_ptr<const PlayTimeline::Goal> goal)
{
  std::string error;
  if (!validate_timeline_goal(*goal, error)) {
    RCLCPP_WARN(get_logger(), "Rejecting timeline goal: %s", error.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!timeline_active_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(get_logger(), "Rejecting timeline goal because another timeline is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  timeline_cancel_requested_ = false;
  timeline_final_action_started_ = false;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PositionControllerNode::handle_timeline_cancel(
    const std::shared_ptr<TimelineGoalHandle> goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(timeline_mutex_);
    if (!active_timeline_goal_ || active_timeline_goal_ != goal_handle) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    if (timeline_final_action_started_) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    timeline_cancel_requested_ = true;
  }
  timeline_cv_.notify_all();
  state_cv_.notify_all();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PositionControllerNode::handle_timeline_accepted(
    const std::shared_ptr<TimelineGoalHandle> goal_handle)
{
  if (timeline_thread_.joinable()) {
    timeline_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(timeline_mutex_);
    active_timeline_goal_ = goal_handle;
  }
  try {
    timeline_thread_ = std::thread([this, goal_handle]() { execute_timeline(goal_handle); });
  } catch (const std::exception &exception) {
    auto result = std::make_shared<PlayTimeline::Result>();
    result->success = false;
    result->message = std::string("Could not start timeline worker: ") + exception.what();
    result->completed_steps = 0;
    goal_handle->abort(result);
    {
      std::lock_guard<std::mutex> lock(timeline_mutex_);
      active_timeline_goal_.reset();
      timeline_active_ = false;
    }
  }
}

void PositionControllerNode::on_joint_states(
    const sensor_msgs::msg::JointState::ConstSharedPtr &message)
{
  if (message->name.size() != message->position.size()) {
    return;
  }
  std::map<std::string, double> positions;
  for (size_t index = 0; index < message->name.size(); ++index) {
    if (!std::isfinite(message->position[index])) {
      return;
    }
    positions[message->name[index]] = message->position[index];
  }
  for (const auto &servo : config_.servos) {
    if (positions.find("servo_" + std::to_string(servo.id)) == positions.end()) {
      return;
    }
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    observed_positions_ = std::move(positions);
    observed_stamp_ns_ = static_cast<int64_t>(message->header.stamp.sec) * 1000000000LL +
                         message->header.stamp.nanosec;
    state_received_ = std::chrono::steady_clock::now();
  }
  state_cv_.notify_all();
}

void PositionControllerNode::publish_timeline_feedback(
    const std::shared_ptr<TimelineGoalHandle> &goal_handle, uint32_t step, uint32_t count,
    const std::string &status, double remaining_seconds)
{
  auto feedback = std::make_shared<PlayTimeline::Feedback>();
  feedback->current_step = step;
  feedback->step_count = count;
  feedback->status = status;
  feedback->seconds_remaining = std::max(0.0, remaining_seconds);
  goal_handle->publish_feedback(feedback);
}

bool PositionControllerNode::wait_for_timeline_deadline(
    const std::shared_ptr<TimelineGoalHandle> &goal_handle, uint32_t step, uint32_t count,
    std::chrono::steady_clock::time_point deadline, std::string &error)
{
  auto next_feedback = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(timeline_mutex_);
  while (true) {
    if (timeline_cancelled()) {
      error = "Timeline canceled";
      return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      const auto lateness = now - deadline;
      if (lateness > kTimelineDeadlineLateness) {
        error = "Missed scheduled timeline deadline by " +
                std::to_string(std::chrono::duration<double, std::milli>(lateness).count()) + " ms";
        return false;
      }
      return true;
    }
    if (now >= next_feedback) {
      publish_timeline_feedback(goal_handle, step, count, "Waiting for scheduled time",
                                std::chrono::duration<double>(deadline - now).count());
      next_feedback = now + kTimelineFeedbackInterval;
    }
    timeline_cv_.wait_until(lock, std::min(deadline, next_feedback),
                            [this]() { return timeline_cancel_requested_.load(); });
  }
}

saprog::timeline::ArrivalStatus PositionControllerNode::wait_for_timeline_arrival(
    const std::shared_ptr<TimelineGoalHandle> &goal_handle, uint32_t step, uint32_t count,
    const std::vector<double> &targets, int64_t sent_stamp_ns, std::string &error)
{
  const auto started = std::chrono::steady_clock::now();
  auto last_feedback = started;
  int64_t last_sample_stamp = sent_stamp_ns;
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  while (true) {
    if (timeline_cancelled()) {
      error = "Timeline canceled";
      return saprog::timeline::ArrivalStatus::failed;
    }
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - started).count() >= kArrivalTimeoutSeconds) {
      {
        std::lock_guard<std::mutex> lock(timeline_mutex_);
        if (timeline_cancel_requested_) {
          error = "Timeline canceled";
          return saprog::timeline::ArrivalStatus::failed;
        }
        timeline_final_action_started_ = true;
      }
      error = "Timed out waiting for all joints to reach their timeline targets within 5 seconds";
      return saprog::timeline::ArrivalStatus::timed_out;
    }
    const int64_t ros_now_ns = this->now().nanoseconds();
    const double source_age = static_cast<double>(ros_now_ns - observed_stamp_ns_) / 1e9;
    if (observed_stamp_ns_ > sent_stamp_ns && observed_stamp_ns_ > last_sample_stamp &&
        source_age >= 0.0 && source_age <= kTelemetryFreshSeconds &&
        std::chrono::duration<double>(now - state_received_).count() <= kTelemetryFreshSeconds) {
      last_sample_stamp = observed_stamp_ns_;
      bool arrived = true;
      for (size_t joint = 0; joint < config_.servos.size(); ++joint) {
        const auto name = "servo_" + std::to_string(config_.servos[joint].id);
        const double error_rad = std::abs(observed_positions_.at(name) - targets[joint]);
        if (error_rad > config_.target_tolerance + saprog::ax::tick_radians / 2.0) {
          arrived = false;
        }
      }
      if (arrived) {
        return saprog::timeline::ArrivalStatus::arrived;
      }
    }
    if (std::chrono::duration<double>(now - last_feedback).count() >= 0.1) {
      publish_timeline_feedback(goal_handle, step, count, "Waiting for joint arrival", 0.0);
      last_feedback = now;
    }
    state_cv_.wait_for(state_lock, kTimelineFeedbackInterval);
  }
}

bool PositionControllerNode::apply_timeline_end_torque(
    bool enabled, std::optional<std::chrono::steady_clock::time_point> latest_start,
    std::string &error)
{
  if (!enabled) {
    if (latest_start) {
      return bus_->set_torques_before(configured_servo_ids(config_), false, *latest_start, error,
                                      [this]() { return timeline_cancelled(); });
    }
    return bus_->set_torques(configured_servo_ids(config_), false, error);
  }
  std::vector<uint16_t> positions;
  positions.reserve(config_.servos.size());
  for (const auto &servo : config_.servos) {
    const auto id = servo.id;
    std::array<uint8_t, 8> data{};
    if (!bus_->read(id, data, error)) {
      error = "Could not read present position before timeline torque-on: " + error;
      return false;
    }
    const auto raw = static_cast<uint16_t>(data[0] | (data[1] << 8));
    if (raw < servo.lower_limit || raw > servo.upper_limit) {
      error = "Present position for servo_" + std::to_string(id) +
              " is outside measured hardware limits; refusing final hold";
      return false;
    }
    positions.push_back(raw);
  }
  const std::vector<int> speeds(config_.servos.size(), config_.moving_speed);
  const std::vector<int> torque_limits(config_.servos.size(), config_.torque_limit);
  if (latest_start) {
    return bus_->set_positions_before(configured_servo_ids(config_), positions, speeds,
                                      torque_limits, config_.target_tolerance, *latest_start, error);
  }
  return bus_->set_positions(configured_servo_ids(config_), positions, speeds, torque_limits,
                             config_.target_tolerance, error);
}

void PositionControllerNode::execute_timeline(const std::shared_ptr<TimelineGoalHandle> goal_handle)
{
  auto result = std::make_shared<PlayTimeline::Result>();
  const auto goal = goal_handle->get_goal();
  const uint32_t step_count = static_cast<uint32_t>(goal->step_kinds.size());
  uint32_t completed_steps = 0;
  bool succeeded = false;
  bool canceled = false;
  std::string error;
  try {
    saprog::timeline::ExecutionPlan plan;
    plan.loop = goal->loop;
    plan.run_end_action = goal->run_end_action;
    plan.end_torque_enabled = goal->end_torque_enabled;
    plan.steps.reserve(goal->step_kinds.size());
    for (size_t step = 0; step < goal->step_kinds.size(); ++step) {
      plan.steps.push_back({goal->step_kinds[step], goal->duration_modes[step], goal->durations[step]});
    }
    const auto servo_ids = configured_servo_ids(config_);
    std::vector<double> pending_positions;
    int64_t pending_stamp_ns = 0;
    saprog::timeline::ExecutorHooks hooks;
    hooks.now = []() { return std::chrono::steady_clock::now(); };
    hooks.canceled = [this]() { return timeline_cancelled(); };
    hooks.wait_until = [this, &goal_handle, step_count](size_t step,
                                                       std::chrono::steady_clock::time_point deadline,
                                                       std::string &detail) {
      const auto feedback_step = static_cast<uint32_t>(std::min(step + 1, size_t(step_count)));
      return wait_for_timeline_deadline(goal_handle, feedback_step, step_count, deadline, detail);
    };
    hooks.send_motion = [this, &goal_handle, &goal, &servo_ids, &pending_positions,
                         &pending_stamp_ns, step_count](
                            size_t step,
                            std::optional<std::chrono::steady_clock::time_point> scheduled,
                            std::string &detail) {
      std::vector<uint16_t> targets;
      std::vector<int> speeds, torque_limits;
      std::vector<double> target_positions;
      targets.reserve(config_.servos.size());
      speeds.reserve(config_.servos.size());
      torque_limits.reserve(config_.servos.size());
      target_positions.reserve(config_.servos.size());
      for (size_t joint = 0; joint < config_.servos.size(); ++joint) {
        const size_t index = step * config_.servos.size() + joint;
        const double position = goal->positions[index];
        targets.push_back(saprog::commands::position_tick(
            position, config_.servos[joint].zero_position, config_.servos[joint].direction,
            config_.servos[joint].lower_limit, config_.servos[joint].upper_limit));
        target_positions.push_back(position);
        speeds.push_back(goal->speeds[index]);
        torque_limits.push_back(goal->torque_limits[index]);
      }
      const auto feedback_step = static_cast<uint32_t>(step + 1);
      publish_timeline_feedback(goal_handle, feedback_step, step_count,
                                "Sending synchronized joint command", 0.0);
      bool commanded = false;
      if (scheduled) {
        const auto latest_start = std::chrono::steady_clock::now() + kTimelineBusLockTimeout;
        commanded = bus_->set_positions_before(servo_ids, targets, speeds, torque_limits,
                                               config_.target_tolerance, latest_start, detail,
                                               [this]() { return timeline_cancelled(); });
      } else {
        commanded = bus_->set_positions(servo_ids, targets, speeds, torque_limits,
                                        config_.target_tolerance, detail);
      }
      if (commanded) {
        const int64_t sent_stamp_ns = now().nanoseconds();
        // The arrival callback uses this immutable command-specific snapshot.
        pending_positions = std::move(target_positions);
        pending_stamp_ns = sent_stamp_ns;
      }
      return commanded;
    };
    hooks.wait_for_arrival = [this, &goal_handle, &pending_positions, &pending_stamp_ns,
                              step_count](size_t step, std::string &detail) {
      return wait_for_timeline_arrival(goal_handle, static_cast<uint32_t>(step + 1), step_count,
                                       pending_positions, pending_stamp_ns, detail);
    };
    hooks.run_end_action = [this, &goal_handle, step_count](
                               bool enabled,
                               std::optional<std::chrono::steady_clock::time_point> latest_start,
                               std::string &detail) {
      {
        std::lock_guard<std::mutex> lock(timeline_mutex_);
        if (timeline_cancel_requested_) {
          detail = "Timeline canceled";
          return false;
        }
        timeline_final_action_started_ = true;
      }
      publish_timeline_feedback(goal_handle, step_count, step_count,
                                "Applying after-timeline torque action", 0.0);
      return apply_timeline_end_torque(enabled, latest_start, detail);
    };
    const auto execution = saprog::timeline::execute(plan, hooks, kTimelineDeadlineLateness);
    succeeded = execution.succeeded;
    canceled = execution.canceled;
    completed_steps = execution.completed_steps;
    error = execution.error;
    if (execution.timed_out) {
      publish_timeline_feedback(goal_handle, step_count, step_count,
                                "Arrival timeout; disabling torque on all joints", 0.0);
      std::string torque_error;
      if (!bus_->set_torques(servo_ids, false, torque_error)) {
        RCLCPP_ERROR(get_logger(),
                     "Timeline arrival timed out; failed to disable torque on all joints: %s",
                     torque_error.c_str());
        error += "; failed to disable torque on all joints: " + torque_error;
      } else {
        RCLCPP_WARN(get_logger(),
                    "Timeline arrival timed out; disabled torque on all configured joints");
      }
    }
  } catch (const std::exception &exception) {
    error = std::string("Timeline execution failed: ") + exception.what();
  }

  result->success = succeeded;
  result->message = succeeded ? "Timeline completed" : (error.empty() ? "Timeline failed" : error);
  result->completed_steps = completed_steps;
  if (canceled) {
    for (int attempt = 0; attempt < 20 && !goal_handle->is_canceling(); ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  {
    std::lock_guard<std::mutex> lock(timeline_mutex_);
    active_timeline_goal_.reset();
    topic_command_barrier_ns_ = now().nanoseconds();
    timeline_active_ = false;
    timeline_final_action_started_ = false;
  }
  if (canceled && goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (succeeded) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

bool PositionControllerNode::timeline_cancelled() const
{
  return timeline_cancel_requested_.load();
}

void PositionControllerNode::set_goal_position(
    const sensor_msgs::msg::JointState::ConstSharedPtr &message)
{
  if (!limits_ready_) {
    RCLCPP_WARN(get_logger(),
                "Hardware limits unavailable; refresh limits before commanding positions");
    return;
  }
  if (message->name.empty() || message->name.size() != message->position.size() ||
      (!message->velocity.empty() && message->velocity.size() != message->name.size()) ||
      (!message->effort.empty() && message->effort.size() != message->name.size())) {
    RCLCPP_WARN(get_logger(),
                "Command matching joint names, positions, optional speeds and torque strengths");
    return;
  }

  std::vector<uint8_t> ids;
  std::vector<uint16_t> targets;
  std::vector<int> torque_limits;
  int speed = config_.moving_speed;
  std::vector<bool> seen(config_.servos.size(), false);
  for (size_t i = 0; i < message->name.size(); ++i) {
    const size_t index = configured_servo_index(config_, message->name[i]);
    if (index == config_.servos.size()) {
      RCLCPP_WARN(get_logger(), "Unknown joint '%s'", message->name[i].c_str());
      return;
    }
    if (seen[index] || !std::isfinite(message->position[i])) {
      RCLCPP_WARN(get_logger(), "Duplicate joint or non-finite position in command for %s",
                  message->name[i].c_str());
      return;
    }
    seen[index] = true;
    if (!message->velocity.empty()) {
      const double requested_speed = message->velocity[i] / saprog::ax::speed_unit_radians;
      if (!std::isfinite(requested_speed) || requested_speed < 1.0 - 1e-9 ||
          requested_speed > 1023.0 + 1e-9) {
        RCLCPP_WARN(get_logger(), "Speed must correspond to AX values 1 through 1023");
        return;
      }
      const int joint_speed = static_cast<int>(std::lround(requested_speed));
      if (i > 0 && joint_speed != speed) {
        RCLCPP_WARN(get_logger(), "All joints in a command must use the same speed");
        return;
      }
      speed = joint_speed;
    }
    int torque_limit = config_.torque_limit;
    if (!message->effort.empty()) {
      const double requested_torque = message->effort[i];
      if (!std::isfinite(requested_torque) || requested_torque < 0.0 || requested_torque > 1023.0) {
        RCLCPP_WARN(get_logger(), "Torque strength must be between 0 and 1023");
        return;
      }
      torque_limit = static_cast<int>(std::lround(requested_torque));
    }
    uint16_t target = 0;
    try {
      target = saprog::commands::position_tick(
          message->position[i], config_.servos[index].zero_position,
          config_.servos[index].direction, config_.servos[index].lower_limit,
          config_.servos[index].upper_limit);
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "Position for %s is outside the AX joint range",
                  message->name[i].c_str());
      return;
    }
    ids.push_back(config_.servos[index].id);
    targets.push_back(target);
    torque_limits.push_back(torque_limit);
  }
  std::string error;
  double source_age_ms = -1.0;
  if (message->header.stamp.sec != 0 || message->header.stamp.nanosec != 0) {
    const rclcpp::Time source_stamp(message->header.stamp, get_clock()->get_clock_type());
    source_age_ms = (now() - source_stamp).seconds() * 1000.0;
  }
  const auto bus_started = std::chrono::steady_clock::now();
  bool commanded = false;
  if (ids.size() == 1) {
    commanded = bus_->set_position(ids[0], targets[0], speed, torque_limits[0], error);
  } else {
    commanded =
        bus_->set_positions(ids, targets, speed, torque_limits, config_.target_tolerance, error);
  }
  const auto bus_elapsed =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - bus_started)
          .count();
  RCLCPP_DEBUG(
      get_logger(),
      "Position command: source age %.2f ms, bus transaction %.2f ms, goal verification %s",
      source_age_ms, bus_elapsed, config_.verify_goal_registers ? "enabled" : "disabled");
  if (!commanded) {
    if (ids.size() == 1) {
      RCLCPP_ERROR(get_logger(), "Servo %u: %s", ids[0], error.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Synchronized position command failed: %s", error.c_str());
    }
    return;
  }
  RCLCPP_INFO(get_logger(), "Commanded %zu servos%s", ids.size(),
              ids.size() > 1 ? " with synchronized start" : "");
}
