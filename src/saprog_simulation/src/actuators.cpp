// Force-based simulated servos. No ROS, serial bus, or hardware dependencies.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/joint_trajectory.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace saprog_sim
{
class Actuators : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate,
                  public gz::sim::ISystemPostUpdate
{
  struct Servo
  {
    gz::sim::Joint joint;
    double lower, upper, max_speed, max_effort;
    double target = 0, reference = 0, speed = 0, effort = 0;
    bool initialized = false;
  };
  std::map<std::string, Servo> servos;
  std::mutex mutex;
  gz::msgs::JointTrajectory pending;
  bool new_command = false;
  bool enabled = false;
  std::chrono::steady_clock::duration last_publish{};
  std::chrono::steady_clock::time_point last_wall_publish{};
  // Transport is destroyed before callback state and its mutex.
  gz::transport::Node node;
  gz::transport::Node::Publisher feedback;

 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    for (auto id : gz::sim::Model(entity).Joints(ecm))
    {
      gz::sim::Joint joint(id);
      auto axes = joint.Axis(ecm);
      if (!axes || axes->size() != 1) continue;
      const auto &axis = axes->front();
      joint.EnablePositionCheck(ecm);
      joint.EnableVelocityCheck(ecm);
      servos.emplace(*joint.Name(ecm), Servo{
        joint, axis.Lower(), axis.Upper(), axis.MaxVelocity(), axis.Effort()});
    }
    feedback = node.Advertise<gz::msgs::JointTrajectory>("/saprog_simulation/feedback");
    node.Subscribe("/saprog_simulation/targets", &Actuators::Command, this);
  }

  void Command(const gz::msgs::JointTrajectory &message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    pending = message;
    new_command = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    // Accept Stop/Release while paused, without advancing the servo references.
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (new_command)
      {
        new_command = false;
        if (pending.points_size() == 0)
          enabled = false;
        else
        {
          const auto &point = pending.points(0);
          const int count = pending.joint_names_size();
          bool valid = count == static_cast<int>(servos.size()) &&
            point.positions_size() == count && point.velocities_size() == count &&
            point.effort_size() == count;
          std::map<std::string, int> names;
          for (int i = 0; valid && i < count; ++i)
          {
            const auto &name = pending.joint_names(i);
            auto found = servos.find(name);
            valid = found != servos.end() && names.emplace(name, i).second &&
              std::isfinite(point.positions(i)) && std::isfinite(point.velocities(i)) &&
              std::isfinite(point.effort(i));
            if (valid)
            {
              const auto &servo = found->second;
              valid = point.positions(i) >= servo.lower && point.positions(i) <= servo.upper &&
                point.velocities(i) > 0 && point.effort(i) > 0;
            }
          }
          if (valid)
          {
            for (const auto &[name, i] : names)
            {
              auto &servo = servos.at(name);
              auto position = servo.joint.Position(ecm);
              if (position && !position->empty())
                servo.reference = position->front();
              servo.target = point.positions(i);
              servo.speed = std::min(point.velocities(i), servo.max_speed);
              servo.effort = std::min(point.effort(i), servo.max_effort);
            }
            enabled = true;
          }
        }
      }
    }
    if (info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0) return;
    for (auto &[name, servo] : servos)
    {
      auto position = servo.joint.Position(ecm);
      auto velocity = servo.joint.Velocity(ecm);
      if (!position || position->empty() || !velocity || velocity->empty()) continue;
      if (!servo.initialized)
      {
        servo.reference = position->front();
        servo.initialized = true;
      }
      double force = 0;
      if (enabled)
      {
        const double delta = servo.speed * dt;
        servo.reference += std::clamp(servo.target - servo.reference, -delta, delta);
        // PD gains are simulation defaults, not measured AX servo parameters.
        force = std::clamp(8.0 * (servo.reference - position->front()) -
          0.005 * velocity->front(), -servo.effort, servo.effort);
      }
      servo.joint.SetForce(ecm, {force});
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    const auto wall = std::chrono::steady_clock::now();
    if (info.paused && wall - last_wall_publish < std::chrono::milliseconds(50)) return;
    if (!info.paused && info.simTime >= last_publish &&
        info.simTime - last_publish < std::chrono::milliseconds(20)) return;
    last_publish = info.simTime;
    last_wall_publish = wall;
    gz::msgs::JointTrajectory message;
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    message.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
    auto point = message.add_points();
    for (const auto &[name, servo] : servos)
    {
      auto position = servo.joint.Position(ecm);
      auto velocity = servo.joint.Velocity(ecm);
      if (!position || position->empty()) continue;
      message.add_joint_names(name);
      point->add_positions(position->front());
      point->add_velocities(velocity && !velocity->empty() ? velocity->front() : 0);
    }
    feedback.Publish(message);
  }
};
}  // namespace saprog_sim

GZ_ADD_PLUGIN(saprog_sim::Actuators, gz::sim::System,
              saprog_sim::Actuators::ISystemConfigure,
              saprog_sim::Actuators::ISystemPreUpdate,
              saprog_sim::Actuators::ISystemPostUpdate)
