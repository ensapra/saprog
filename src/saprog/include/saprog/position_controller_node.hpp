#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "saprog/action/play_timeline.hpp"
#include "saprog/config.hpp"
#include "saprog/dynamixel_bus.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode(const rclcpp::NodeOptions &options, std::shared_ptr<DynamixelBus> bus);
  ~PositionControllerNode() override;

private:
  using PlayTimeline = saprog::action::PlayTimeline;
  using TimelineGoalHandle = rclcpp_action::ServerGoalHandle<PlayTimeline>;

  bool refresh_limits(std::string &error);
  bool validate_timeline_goal(const PlayTimeline::Goal &goal, std::string &error) const;
  rclcpp_action::GoalResponse handle_timeline_goal(const rclcpp_action::GoalUUID &uuid,
                                                   std::shared_ptr<const PlayTimeline::Goal> goal);
  rclcpp_action::CancelResponse
  handle_timeline_cancel(const std::shared_ptr<TimelineGoalHandle> goal_handle);
  void handle_timeline_accepted(const std::shared_ptr<TimelineGoalHandle> goal_handle);
  void execute_timeline(const std::shared_ptr<TimelineGoalHandle> goal_handle);
  void on_joint_states(const sensor_msgs::msg::JointState::ConstSharedPtr &message);
  void publish_timeline_feedback(const std::shared_ptr<TimelineGoalHandle> &goal_handle,
                                 uint32_t step, uint32_t count, const std::string &status,
                                 double remaining_seconds);
  bool wait_for_timeline_deadline(const std::shared_ptr<TimelineGoalHandle> &goal_handle,
                                  uint32_t step, uint32_t count,
                                  std::chrono::steady_clock::time_point deadline,
                                  std::string &error);
  bool wait_for_timeline_arrival(const std::shared_ptr<TimelineGoalHandle> &goal_handle,
                                 uint32_t step, uint32_t count, const std::vector<double> &targets,
                                 int64_t sent_stamp_ns, std::string &error);
  bool apply_timeline_end_torque(bool enabled,
                                 std::optional<std::chrono::steady_clock::time_point> latest_start,
                                 std::string &error);
  bool timeline_cancelled() const;
  bool limits_ready_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr limits_service_;
  void set_goal_position(const sensor_msgs::msg::JointState::ConstSharedPtr &message);
  void set_joint_torque(uint8_t id, const std_srvs::srv::SetBool::Request::SharedPtr request,
                        std_srvs::srv::SetBool::Response::SharedPtr response);

  SaprogConfig config_;
  std::shared_ptr<DynamixelBus> bus_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> torque_services_;
  rclcpp_action::Server<PlayTimeline>::SharedPtr timeline_action_server_;
  // Atomics cross the executor/worker boundary; timeline_mutex_ protects the
  // active goal and the transition into the final, non-cancellable torque action.
  std::atomic_bool timeline_active_{false};
  std::atomic_bool timeline_cancel_requested_{false};
  std::atomic_bool timeline_final_action_started_{false};
  std::atomic<int64_t> topic_command_barrier_ns_{0};
  mutable std::mutex timeline_mutex_;
  std::condition_variable timeline_cv_;
  std::thread timeline_thread_;
  std::shared_ptr<TimelineGoalHandle> active_timeline_goal_;
  // Telemetry callbacks publish a complete sample under state_mutex_; the
  // worker waits on state_cv_ without blocking the ROS executor.
  std::mutex state_mutex_;
  std::condition_variable state_cv_;
  std::map<std::string, double> observed_positions_;
  int64_t observed_stamp_ns_{0};
  std::chrono::steady_clock::time_point state_received_;
};
