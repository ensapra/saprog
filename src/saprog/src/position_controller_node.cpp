#include "saprog/position_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <string>
#include <array>
#include <limits>
#include <optional>
#include <stdexcept>
#include <condition_variable>
#include <thread>

namespace
{
constexpr double kAxPositionRangeRadians = 300.0 * 3.14159265358979323846 / 180.0;
constexpr double kAxPositionRangeRaw = 1023.0;
constexpr double kAxTickRadians = kAxPositionRangeRadians / kAxPositionRangeRaw;
constexpr double kArrivalTimeoutSeconds = 30.0;
constexpr double kNoProgressSeconds = 8.0;
constexpr double kTelemetryFreshSeconds = 2.5;
constexpr auto kTimelineFeedbackInterval = std::chrono::milliseconds(100);
constexpr auto kTimelineDeadlineLateness = std::chrono::milliseconds(20);
constexpr auto kTimelineBusLockTimeout = std::chrono::milliseconds(250);
}

PositionControllerNode::PositionControllerNode(
  const rclcpp::NodeOptions & options, std::shared_ptr<DynamixelBus> bus)
: Node("saprog_position_controller", options), config_(read_config(*this)), bus_(std::move(bus))
{
  for (const auto * name : {"lower_limits", "upper_limits"}) {
    if (!has_parameter(name)) {
      declare_parameter<std::vector<int64_t>>(name, std::vector<int64_t>{});
    }
  }
  limits_service_ = create_service<std_srvs::srv::Trigger>(
    "/saprog/refresh_limits",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
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
  RCLCPP_INFO(get_logger(), "Controller ready for %zu configured servos; startup leaves torque unchanged", config_.servo_ids.size());

  command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/saprog/position_command", rclcpp::QoS(rclcpp::KeepLast(config_.servo_ids.size() + 10)),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
      if (timeline_active_) {
        RCLCPP_WARN(get_logger(), "Ignoring position topic command while timeline action owns the bus");
        return;
      }
      const int64_t command_stamp_ns =
        static_cast<int64_t>(message->header.stamp.sec) * 1000000000LL +
        message->header.stamp.nanosec;
      if (command_stamp_ns != 0 && command_stamp_ns <= topic_command_barrier_ns_) {
        RCLCPP_WARN(get_logger(), "Ignoring queued position command older than the completed timeline");
        return;
      }
      set_goal_position(message);
    });
  state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/saprog/joint_states", rclcpp::QoS(10),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {on_joint_states(message);});
  for (const auto id : config_.servo_ids) {
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
    [this](const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const PlayTimeline::Goal> goal) {
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

bool PositionControllerNode::refresh_limits(std::string & error)
{
  limits_ready_ = false;
  std::vector<int64_t> lower, upper;
  error.clear();
  for (const auto id : config_.servo_ids) {
    uint16_t low = 0, high = 0;
    std::string detail;
    if (!bus_->get_limits(id, low, high, detail)) {
      error += "servo_" + std::to_string(id) + ": " + detail + "; ";
    }
    lower.push_back(low);
    upper.push_back(high);
  }
  if (!error.empty()) {
    set_parameters({rclcpp::Parameter("lower_limits", std::vector<int64_t>{}),
      rclcpp::Parameter("upper_limits", std::vector<int64_t>{})});
    return false;
  }
  config_.lower_limits.assign(lower.begin(), lower.end());
  config_.upper_limits.assign(upper.begin(), upper.end());
  set_parameters({rclcpp::Parameter("lower_limits", lower), rclcpp::Parameter("upper_limits", upper)});
  limits_ready_ = true;
  error = "Read fresh hardware limits for all configured servos";
  return true;
}

void PositionControllerNode::set_joint_torque(uint8_t id,
  const std_srvs::srv::SetBool::Request::SharedPtr request,
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
  const size_t index = std::distance(config_.servo_ids.begin(),
    std::find(config_.servo_ids.begin(), config_.servo_ids.end(), id));
  if (raw < config_.lower_limits[index] || raw > config_.upper_limits[index]) {
    response->success = false;
    response->message = "Present position is outside the AX range";
    return;
  }
  response->success = bus_->set_position(id, raw, config_.moving_speed, config_.torque_limit, error);
  response->message = response->success ? "Holding freshly read present position" : error;
}

bool PositionControllerNode::validate_timeline_goal(
  const PlayTimeline::Goal & goal, std::string & error) const
{
  if (!limits_ready_) {
    error = "Hardware limits are unavailable";
    return false;
  }
  std::vector<std::string> configured_names;
  configured_names.reserve(config_.servo_ids.size());
  for (const auto id : config_.servo_ids) {
    configured_names.push_back("servo_" + std::to_string(id));
  }
  if (goal.joint_names != configured_names || goal.step_kinds.empty() ||
    goal.duration_modes.size() != goal.step_kinds.size() ||
    goal.durations.size() != goal.step_kinds.size())
  {
    error = "Timeline joints or step metadata do not match the controller configuration";
    return false;
  }
  const size_t value_count = goal.step_kinds.size() * config_.servo_ids.size();
  if (goal.positions.size() != value_count || goal.speeds.size() != value_count ||
    goal.torque_limits.size() != value_count || (goal.loop && goal.run_end_action))
  {
    error = "Timeline command arrays have invalid lengths or lifecycle settings";
    return false;
  }
  for (size_t step = 0; step < goal.step_kinds.size(); ++step) {
    if (!std::isfinite(goal.durations[step]) || goal.durations[step] < 0.0) {
      error = "Timeline durations must be finite and non-negative";
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
    for (size_t joint = 0; joint < config_.servo_ids.size(); ++joint) {
      const size_t index = step * config_.servo_ids.size() + joint;
      const double position = goal.positions[index];
      const int speed = goal.speeds[index];
      const int torque = goal.torque_limits[index];
      if (!std::isfinite(position) || speed < 1 || speed > 1023 ||
        torque < 1 || torque > 1023)
      {
        error = "Timeline motion values must be finite with speed and torque from 1 to 1023";
        return false;
      }
      const double raw = config_.zero_positions[joint] + config_.directions[joint] *
        position * kAxPositionRangeRaw / kAxPositionRangeRadians;
      if (raw < config_.lower_limits[joint] - 1e-9 ||
        raw > config_.upper_limits[joint] + 1e-9)
      {
        error = "Timeline target is outside measured hardware limits for " + configured_names[joint];
        return false;
      }
    }
  }
  error.clear();
  return true;
}

rclcpp_action::GoalResponse PositionControllerNode::handle_timeline_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const PlayTimeline::Goal> goal)
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
    timeline_thread_ = std::thread(
      [this, goal_handle]() {execute_timeline(goal_handle);});
  } catch (const std::exception & exception) {
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
  const sensor_msgs::msg::JointState::ConstSharedPtr & message)
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
  for (const auto id : config_.servo_ids) {
    if (positions.find("servo_" + std::to_string(id)) == positions.end()) {
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
  const std::shared_ptr<TimelineGoalHandle> & goal_handle,
  uint32_t step, uint32_t count, const std::string & status, double remaining_seconds)
{
  auto feedback = std::make_shared<PlayTimeline::Feedback>();
  feedback->current_step = step;
  feedback->step_count = count;
  feedback->status = status;
  feedback->seconds_remaining = std::max(0.0, remaining_seconds);
  goal_handle->publish_feedback(feedback);
}

bool PositionControllerNode::wait_for_timeline_deadline(
  const std::shared_ptr<TimelineGoalHandle> & goal_handle,
  uint32_t step, uint32_t count, std::chrono::steady_clock::time_point deadline,
  std::string & error)
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
    timeline_cv_.wait_until(lock, std::min(deadline, next_feedback), [this]() {
      return timeline_cancel_requested_.load();
    });
  }
}

bool PositionControllerNode::wait_for_timeline_arrival(
  const std::shared_ptr<TimelineGoalHandle> & goal_handle, uint32_t step, uint32_t count,
  const std::vector<double> & targets, int64_t sent_stamp_ns, std::string & error)
{
  const auto started = std::chrono::steady_clock::now();
  auto last_progress = started;
  auto last_feedback = started;
  std::vector<double> best_errors(targets.size(), std::numeric_limits<double>::infinity());
  int64_t last_sample_stamp = sent_stamp_ns;
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  while (true) {
    if (timeline_cancelled()) {
      error = "Timeline canceled";
      return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - started).count() >= kArrivalTimeoutSeconds) {
      error = "Timed out waiting for fresh joint arrival telemetry";
      return false;
    }
    const int64_t ros_now_ns = this->now().nanoseconds();
    const double source_age = static_cast<double>(ros_now_ns - observed_stamp_ns_) / 1e9;
    if (observed_stamp_ns_ > sent_stamp_ns && observed_stamp_ns_ > last_sample_stamp &&
      source_age >= 0.0 && source_age <= kTelemetryFreshSeconds &&
      std::chrono::duration<double>(now - state_received_).count() <= kTelemetryFreshSeconds)
    {
      last_sample_stamp = observed_stamp_ns_;
      bool arrived = true;
      bool progress = false;
      for (size_t joint = 0; joint < config_.servo_ids.size(); ++joint) {
        const auto name = "servo_" + std::to_string(config_.servo_ids[joint]);
        const double error_rad = std::abs(observed_positions_.at(name) - targets[joint]);
        if (error_rad > config_.target_tolerance + kAxTickRadians / 2.0) {
          arrived = false;
          if (error_rad < best_errors[joint] - kAxTickRadians / 2.0) {
            best_errors[joint] = error_rad;
            progress = true;
          }
        }
      }
      if (arrived) {
        return true;
      }
      if (progress) {
        last_progress = now;
      }
      if (std::chrono::duration<double>(now - last_progress).count() >= kNoProgressSeconds) {
        error = "No progress toward timeline targets for 8 seconds";
        return false;
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
  std::string & error)
{
  if (!enabled) {
    if (latest_start) {
      return bus_->set_torques_before(config_.servo_ids, false, *latest_start, error);
    }
    return bus_->set_torques(config_.servo_ids, false, error);
  }
  std::vector<uint16_t> positions;
  positions.reserve(config_.servo_ids.size());
  for (const auto id : config_.servo_ids) {
    std::array<uint8_t, 8> data{};
    if (!bus_->read(id, data, error)) {
      error = "Could not read present position before timeline torque-on: " + error;
      return false;
    }
    positions.push_back(static_cast<uint16_t>(data[0] | (data[1] << 8)));
  }
  const std::vector<int> speeds(config_.servo_ids.size(), config_.moving_speed);
  const std::vector<int> torque_limits(config_.servo_ids.size(), config_.torque_limit);
  return bus_->set_positions(config_.servo_ids, positions, speeds, torque_limits,
    config_.target_tolerance, error);
}

void PositionControllerNode::execute_timeline(
  const std::shared_ptr<TimelineGoalHandle> goal_handle)
{
  auto result = std::make_shared<PlayTimeline::Result>();
  const auto goal = goal_handle->get_goal();
  const uint32_t step_count = static_cast<uint32_t>(goal->step_kinds.size());
  uint32_t completed_steps = 0;
  bool succeeded = false;
  bool canceled = false;
  std::string error;
  std::optional<std::chrono::steady_clock::time_point> next_command_deadline;
  try {
    bool repeat = true;
    while (repeat) {
      repeat = goal->loop;
      for (size_t step = 0; step < goal->step_kinds.size(); ++step) {
        const auto step_number = static_cast<uint32_t>(step + 1);
        if (timeline_cancelled()) {
          canceled = true;
          error = "Timeline canceled";
          break;
        }

        auto dispatch_deadline = next_command_deadline;
        if (dispatch_deadline) {
          if (!wait_for_timeline_deadline(
              goal_handle, step_number, step_count, *dispatch_deadline, error))
          {
            canceled = timeline_cancelled();
            break;
          }
          next_command_deadline.reset();
        }

        if (goal->step_kinds[step] == 1) {
          const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(goal->durations[step]));
          if (!wait_for_timeline_deadline(
              goal_handle, step_number, step_count, deadline, error))
          {
            canceled = timeline_cancelled();
            break;
          }
          next_command_deadline = deadline;
          ++completed_steps;
          continue;
        }

        std::vector<uint16_t> targets;
        std::vector<int> speeds, torque_limits;
        std::vector<double> target_positions;
        targets.reserve(config_.servo_ids.size());
        speeds.reserve(config_.servo_ids.size());
        torque_limits.reserve(config_.servo_ids.size());
        target_positions.reserve(config_.servo_ids.size());
        for (size_t joint = 0; joint < config_.servo_ids.size(); ++joint) {
          const size_t index = step * config_.servo_ids.size() + joint;
          const double position = goal->positions[index];
          const double raw = config_.zero_positions[joint] + config_.directions[joint] *
            position * kAxPositionRangeRaw / kAxPositionRangeRadians;
          targets.push_back(static_cast<uint16_t>(std::lround(raw)));
          target_positions.push_back(position);
          speeds.push_back(goal->speeds[index]);
          torque_limits.push_back(goal->torque_limits[index]);
        }

        publish_timeline_feedback(goal_handle, step_number, step_count, "Sending synchronized joint command", 0.0);
        bool commanded = false;
        if (dispatch_deadline) {
          const auto latest_start = std::chrono::steady_clock::now() + kTimelineBusLockTimeout;
          commanded = bus_->set_positions_before(config_.servo_ids, targets, speeds,
            torque_limits, config_.target_tolerance, latest_start, error);
        } else {
          commanded = bus_->set_positions(config_.servo_ids, targets, speeds,
            torque_limits, config_.target_tolerance, error);
        }
        if (!commanded) {
          break;
        }
        const int64_t sent_stamp_ns = now().nanoseconds();

        if (goal->duration_modes[step] == 1) {
          const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(goal->durations[step]));
          if (!wait_for_timeline_deadline(
              goal_handle, step_number, step_count, deadline, error))
          {
            canceled = timeline_cancelled();
            break;
          }
          next_command_deadline = deadline;
        } else if (!wait_for_timeline_arrival(
            goal_handle, step_number, step_count, target_positions, sent_stamp_ns, error))
        {
          canceled = timeline_cancelled();
          break;
        }
        ++completed_steps;
      }

      if (timeline_cancelled()) {
        canceled = true;
        error = "Timeline canceled";
      }
      if (canceled || !error.empty() || !goal->loop) {
        repeat = false;
      }
    }

    if (!canceled && error.empty() && goal->run_end_action) {
      std::optional<std::chrono::steady_clock::time_point> latest_start;
      if (next_command_deadline) {
        if (!wait_for_timeline_deadline(
            goal_handle, step_count, step_count, *next_command_deadline, error))
        {
          canceled = timeline_cancelled();
        } else {
          latest_start = std::chrono::steady_clock::now() + kTimelineBusLockTimeout;
        }
      }
      if (!canceled && error.empty()) {
        std::lock_guard<std::mutex> lock(timeline_mutex_);
        if (timeline_cancel_requested_) {
          canceled = true;
          error = "Timeline canceled";
        } else {
          timeline_final_action_started_ = true;
        }
      }
      if (!canceled && error.empty()) {
        publish_timeline_feedback(goal_handle, step_count, step_count,
          "Applying after-timeline torque action", 0.0);
        if (!apply_timeline_end_torque(goal->end_torque_enabled, latest_start, error)) {
          error = "After-timeline torque action failed: " + error;
        }
      }
    }
    succeeded = !canceled && error.empty();
  } catch (const std::exception & exception) {
    error = std::string("Timeline execution failed: ") + exception.what();
  }

  result->success = succeeded;
  result->message = succeeded ? "Timeline completed" :
    (error.empty() ? "Timeline failed" : error);
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
  const sensor_msgs::msg::JointState::ConstSharedPtr & message)
{
  if (!limits_ready_) {
    RCLCPP_WARN(get_logger(), "Hardware limits unavailable; refresh limits before commanding positions");
    return;
  }
  if (message->name.empty() || message->name.size() != message->position.size() ||
    (!message->velocity.empty() && message->velocity.size() != message->name.size()) ||
    (!message->effort.empty() && message->effort.size() != message->name.size()))
  {
    RCLCPP_WARN(get_logger(), "Command matching joint names, positions, optional speeds and torque strengths");
    return;
  }

  std::vector<uint8_t> ids;
  std::vector<uint16_t> targets;
  std::vector<int> torque_limits;
  int speed = config_.moving_speed;
  constexpr double speed_unit = 0.111 * 2.0 * 3.14159265358979323846 / 60.0;
  std::vector<bool> seen(config_.servo_ids.size(), false);
  for (size_t i = 0; i < message->name.size(); ++i) {
    const auto servo = std::find_if(
      config_.servo_ids.begin(), config_.servo_ids.end(), [&message, i](uint8_t id) {
        return message->name[i] == "servo_" + std::to_string(id);
      });
    if (servo == config_.servo_ids.end()) {
      RCLCPP_WARN(get_logger(), "Unknown joint '%s'", message->name[i].c_str());
      return;
    }
    const auto index = static_cast<size_t>(std::distance(config_.servo_ids.begin(), servo));
    if (seen[index] || !std::isfinite(message->position[i])) {
      RCLCPP_WARN(get_logger(), "Duplicate joint or non-finite position in command for %s", message->name[i].c_str());
      return;
    }
    seen[index] = true;
    if (!message->velocity.empty()) {
      const double requested_speed = message->velocity[i] / speed_unit;
      if (!std::isfinite(requested_speed) || requested_speed < 1.0 - 1e-9 ||
        requested_speed > 1023.0 + 1e-9)
      {
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
    const double raw = config_.zero_positions[index] + config_.directions[index] *
      message->position[i] * kAxPositionRangeRaw / kAxPositionRangeRadians;
    if (raw < config_.lower_limits[index] - 1e-9 || raw > config_.upper_limits[index] + 1e-9) {
      RCLCPP_WARN(get_logger(), "Position for %s is outside the AX joint range", message->name[i].c_str());
      return;
    }
    ids.push_back(*servo);
    targets.push_back(static_cast<uint16_t>(std::lround(raw)));
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
    commanded = bus_->set_positions(ids, targets, speed, torque_limits,
      config_.target_tolerance, error);
  }
  const auto bus_elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - bus_started).count();
  RCLCPP_DEBUG(get_logger(),
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
  RCLCPP_INFO(get_logger(), "Commanded %zu servos%s", ids.size(), ids.size() > 1 ? " with synchronized start" : "");
}
