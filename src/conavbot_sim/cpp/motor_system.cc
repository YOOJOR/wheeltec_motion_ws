// Fortress physics system: all vehicle motion comes from wheel joint torques.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <string>
#include <stdexcept>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float_v.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace conavbot {
class MotorSystem final : public ignition::gazebo::System,
                          public ignition::gazebo::ISystemConfigure,
                          public ignition::gazebo::ISystemPreUpdate {
 public:
  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 ignition::gazebo::EntityComponentManager &ecm,
                 ignition::gazebo::EventManager &) override {
    ignition::gazebo::Model model(entity);
    delay_ = sdf->Get<double>("command_delay", 0.20).first;
    kp_ = sdf->Get<double>("kp", 0.08).first;
    kd_ = sdf->Get<double>("kd", 0.0).first;
    ki_ = sdf->Get<double>("ki", 0.10).first;
    stall_ = sdf->Get<double>("max_torque", 0.8).first;
    noLoad_ = sdf->Get<double>("max_wheel_speed", 20.0).first;
    response_ = sdf->Get<double>("torque_time_constant", 0.05).first;
    watchdog_ = sdf->Get<double>("watchdog", 0.5).first;
    period_ = 1.0 / sdf->Get<double>("control_rate", 100.0).first;
    radius_ = sdf->Get<double>("wheel_radius", 0.05).first;
    lever_ = (sdf->Get<double>("axle", 0.313).first +
              sdf->Get<double>("track", 0.355).first) / 2.0;
    for (double value : {stall_, noLoad_, response_, watchdog_, period_, radius_, lever_})
      if (!std::isfinite(value) || value <= 0.0)
        throw std::runtime_error("Motor physical parameters must be finite and positive");
    for (double value : {delay_, kp_, ki_, kd_})
      if (!std::isfinite(value) || value < 0.0)
        throw std::runtime_error("Motor delay and PID gains must be finite and nonnegative");
    const std::array<std::string, 4> names{
        "lf_wheel_joint", "lb_wheel_joint", "rb_wheel_joint", "rf_wheel_joint"};
    for (size_t i = 0; i < joints_.size(); ++i) {
      joints_[i] = model.JointByName(ecm, names[i]);
      if (joints_[i] == ignition::gazebo::kNullEntity)
        throw std::runtime_error("Missing mecanum joint: " + names[i]);
      ecm.CreateComponent(joints_[i], ignition::gazebo::components::JointVelocity());
      ecm.CreateComponent(joints_[i], ignition::gazebo::components::JointForceCmd({0.0}));
    }
    node_.Subscribe("/sim/cmd_vel", &MotorSystem::Command, this);
    node_.Subscribe("/sim/emergency_stop", &MotorSystem::Emergency, this);
    telemetry_ = node_.Advertise<ignition::msgs::Float_V>("/sim/motor_state");
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                 ignition::gazebo::EntityComponentManager &ecm) override {
    const double now = std::chrono::duration<double>(info.simTime).count();
    simulationTime_.store(now);
    if (info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0.0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (now - lastCommand_ > watchdog_) {
      pending_.clear();
      target_.fill(0.0);
      delayed_.fill(0.0);
    }
    while (!pending_.empty() && pending_.front().time + delay_ <= now) {
      const auto &cmd = pending_.front();
      delayed_ = {cmd.x, cmd.y, cmd.yaw};
      const double x = cmd.x, y = cmd.y, yaw = lever_ * cmd.yaw;
      target_ = {(x-y-yaw)/radius_, (x+y-yaw)/radius_,
                 (x-y+yaw)/radius_, (x+y+yaw)/radius_};
      pending_.pop_front();
    }
    accumulator_ += dt;
    const bool controlTick = accumulator_ >= period_;
    const double controlDt = accumulator_;
    if (controlTick) accumulator_ = 0.0;
    std::array<double, 4> measured{};
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto *velocity = ecm.Component<ignition::gazebo::components::JointVelocity>(joints_[i]);
      if (velocity && !velocity->Data().empty()) measured[i] = velocity->Data()[0];
      // DC motor envelope: zero positive drive torque at its no-load speed;
      // opposing torque remains available for braking and reversing.
      const double low = -stall_ * std::clamp(1.0 + measured[i]/noLoad_, 0.0, 1.0);
      const double high = stall_ * std::clamp(1.0 - measured[i]/noLoad_, 0.0, 1.0);
      if (controlTick) {
        const double error = (stopped_ ? 0.0 : target_[i]) - measured[i];
        const double candidate = integral_[i] + error * controlDt;
        const double derivative = (error-previousError_[i])/controlDt;
        previousError_[i] = error;
        const double raw = kp_*error + ki_*candidate + kd_*derivative;
        if ((raw >= low && raw <= high) || (raw > high && error < 0.0) ||
            (raw < low && error > 0.0)) integral_[i] = candidate;
        requested_[i] = std::clamp(kp_*error + ki_*integral_[i] + kd_*derivative, low, high);
      }
      torque_[i] += (1.0-std::exp(-dt/response_)) * (requested_[i]-torque_[i]);
      torque_[i] = std::clamp(torque_[i], low, high);
      ecm.Component<ignition::gazebo::components::JointForceCmd>(joints_[i])->Data() = {torque_[i]};
    }
    if (controlTick) {
      // [simulation seconds, emergency latch (0/1), target LF LB RB RF,
      //  measured LF LB RB RF (rad/s), applied LF LB RB RF (N m),
      //  received vx vy yaw, delayed vx vy yaw (m/s, m/s, rad/s)].
      ignition::msgs::Float_V msg;
      msg.add_data(now);
      msg.add_data(stopped_ ? 1.0 : 0.0);
      for (auto value : target_) msg.add_data(stopped_ ? 0.0 : value);
      for (auto value : measured) msg.add_data(value);
      for (auto value : torque_) msg.add_data(value);
      for (auto value : received_) msg.add_data(value);
      for (auto value : delayed_) msg.add_data(value);
      telemetry_.Publish(msg);
    }
  }
 private:
  struct CommandSample { double time, x, y, yaw; };
  void Command(const ignition::msgs::Twist &msg) {
    const double x = msg.linear().x(), y = msg.linear().y(), yaw = msg.angular().z();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = simulationTime_.load();
    received_ = {x, y, yaw};
    if (!stopped_) pending_.push_back({lastCommand_, x, y, yaw});
  }
  void Emergency(const ignition::msgs::Boolean &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = msg.data();
    pending_.clear();
    target_.fill(0.0);
    delayed_.fill(0.0);
    previousError_.fill(0.0);
    integral_.fill(0.0);
    requested_.fill(0.0);
    // Emergency bypasses transport delay and the drive torque response.
    torque_.fill(0.0);
  }
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher telemetry_;
  std::array<ignition::gazebo::Entity, 4> joints_{};
  std::array<double, 4> target_{}, integral_{}, requested_{}, torque_{};
  std::array<double, 4> previousError_{};
  std::array<double, 3> received_{}, delayed_{};
  double lastCommand_{-1.0};
  double delay_{0.20}, kp_{0.08}, kd_{0.0}, ki_{0.10};
  double stall_{0.8}, noLoad_{20.0}, response_{0.05}, watchdog_{0.5}, period_{0.01};
  std::deque<CommandSample> pending_;
  std::mutex mutex_;
  std::atomic<double> simulationTime_{0.0};
  bool stopped_{false};
  double radius_{0.05}, lever_{0.334}, accumulator_{0.0};
};
}
IGNITION_ADD_PLUGIN(conavbot::MotorSystem, ignition::gazebo::System,
                   conavbot::MotorSystem::ISystemConfigure,
                   conavbot::MotorSystem::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(conavbot::MotorSystem, "conavbot::MotorSystem")
