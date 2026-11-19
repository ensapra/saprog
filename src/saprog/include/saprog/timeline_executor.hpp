#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "saprog/timeline_deadline.hpp"

namespace saprog::timeline
{
struct PlannedStep
{
  uint8_t kind; // PlayTimeline.action: 0 is motion; 1 is wait.
  uint8_t duration_mode; // PlayTimeline.action: 0 waits for arrival; 1 is timed.
  double duration_seconds;
};

struct ExecutionPlan
{
  std::vector<PlannedStep> steps;
  bool loop{false};
  bool run_end_action{false};
  bool end_torque_enabled{false};
};

struct ExecutionResult
{
  bool succeeded{false};
  bool canceled{false};
  bool timed_out{false};
  uint32_t completed_steps{0};
  std::string error;
};

enum class ArrivalStatus
{
  arrived,
  failed,
  timed_out
};

// The adapter supplies monotonic time, cancellable waits, command dispatch,
// fresh telemetry arrival and the hardware torque action. No ROS or bus object
// is retained, so this sequencing policy is deterministic under fake hooks.
struct ExecutorHooks
{
  std::function<std::chrono::steady_clock::time_point()> now;
  std::function<bool()> canceled;
  std::function<bool(size_t, std::chrono::steady_clock::time_point, std::string &)> wait_until;
  std::function<bool(size_t, std::optional<std::chrono::steady_clock::time_point>, std::string &)>
      send_motion;
  std::function<ArrivalStatus(size_t, std::string &)> wait_for_arrival;
  std::function<bool(bool, std::optional<std::chrono::steady_clock::time_point>, std::string &)>
      run_end_action;
};

inline ExecutionResult execute(const ExecutionPlan &plan, const ExecutorHooks &hooks,
                               std::chrono::steady_clock::duration allowed_lateness =
                                   std::chrono::milliseconds(20))
{
  ExecutionResult result;
  if (!hooks.now || !hooks.canceled || !hooks.wait_until || !hooks.send_motion ||
      !hooks.wait_for_arrival || !hooks.run_end_action || allowed_lateness <
                                                            std::chrono::steady_clock::duration::zero()) {
    result.error = "Timeline executor hooks or lateness policy are invalid";
    return result;
  }
  if (plan.steps.empty() || (plan.loop && plan.run_end_action)) {
    result.error = "Timeline execution plan has invalid lifecycle settings";
    return result;
  }
  for (const auto &step : plan.steps) {
    const bool valid_wait = step.kind == 1 && step.duration_mode == 1 &&
                            std::isfinite(step.duration_seconds) && step.duration_seconds >= 0.0;
    const bool valid_motion = step.kind == 0 &&
                              (step.duration_mode == 1 || step.duration_mode == 0) &&
                              (step.duration_mode != 1 ||
                               (std::isfinite(step.duration_seconds) && step.duration_seconds >= 0.0));
    if (!valid_wait && !valid_motion) {
      result.error = "Timeline execution plan contains invalid step data";
      return result;
    }
  }
  std::optional<std::chrono::steady_clock::time_point> next_dispatch;
  bool repeat = true;
  while (repeat) {
    repeat = plan.loop;
    for (size_t i = 0; i < plan.steps.size(); ++i) {
      if (hooks.canceled()) {
        result.canceled = true;
        result.error = "Timeline canceled";
        break;
      }
      const auto check_wait = [&](std::chrono::steady_clock::time_point deadline) {
        if (!hooks.wait_until(i, deadline, result.error)) {
          result.canceled = hooks.canceled();
          return false;
        }
        if (hooks.now() > deadline && hooks.now() - deadline > allowed_lateness) {
          result.error = "Missed scheduled timeline deadline";
          return false;
        }
        return true;
      };
      const auto dispatch = next_dispatch;
      if (dispatch) {
        if (!check_wait(*dispatch)) {
          break;
        }
        next_dispatch.reset();
      }
      const auto &step = plan.steps[i];
      if (step.kind == 1) {
        std::chrono::steady_clock::time_point deadline;
        if (!checked_deadline(step.duration_seconds, hooks.now(), deadline)) {
          result.error = "Timeline wait duration exceeds steady-clock deadline range";
          break;
        }
        if (!check_wait(deadline)) {
          break;
        }
        next_dispatch = deadline;
        ++result.completed_steps;
        continue;
      }
      if (step.kind != 0) {
        result.error = "Timeline plan contains an unknown step kind";
        break;
      }
      if (!hooks.send_motion(i, dispatch, result.error)) {
        break;
      }
      if (step.duration_mode == 1) {
        std::chrono::steady_clock::time_point deadline;
        if (!checked_deadline(step.duration_seconds, hooks.now(), deadline)) {
          result.error = "Timeline transition duration exceeds steady-clock deadline range";
          break;
        }
        if (!check_wait(deadline)) {
          break;
        }
        next_dispatch = deadline;
      } else if (step.duration_mode == 0) {
        const auto arrival = hooks.wait_for_arrival(i, result.error);
        if (arrival != ArrivalStatus::arrived) {
          result.canceled = hooks.canceled();
          result.timed_out = !result.canceled && arrival == ArrivalStatus::timed_out;
          break;
        }
      } else {
        result.error = "Timeline plan contains an unknown duration mode";
        break;
      }
      ++result.completed_steps;
    }
    if (hooks.canceled()) {
      result.canceled = true;
      result.error = "Timeline canceled";
    }
    if (result.canceled || !result.error.empty() || !plan.loop) {
      repeat = false;
    }
  }

  if (!result.canceled && result.error.empty() && plan.run_end_action) {
    std::optional<std::chrono::steady_clock::time_point> latest_start;
    if (next_dispatch) {
      if (!hooks.wait_until(plan.steps.size(), *next_dispatch, result.error)) {
        result.canceled = hooks.canceled();
      } else if (hooks.now() > *next_dispatch &&
                 hooks.now() - *next_dispatch > allowed_lateness) {
        result.error = "Missed scheduled timeline deadline before final action";
      } else {
        next_dispatch.reset();
        latest_start = hooks.now() + std::chrono::milliseconds(250);
      }
    }
    if (!result.canceled && result.error.empty() && hooks.canceled()) {
      result.canceled = true;
      result.error = "Timeline canceled";
    }
    if (!result.canceled && result.error.empty() &&
        !hooks.run_end_action(plan.end_torque_enabled, latest_start, result.error)) {
      if (hooks.canceled()) {
        result.canceled = true;
        if (result.error.empty()) {
          result.error = "Timeline canceled";
        }
      } else {
        result.error = "After-timeline torque action failed: " + result.error;
      }
    }
  }
  result.succeeded = !result.canceled && result.error.empty();
  return result;
}
} // namespace saprog::timeline
