#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
namespace mycobot_280_live_control {
// Compare measured TCP travel with the requested direction. Joint tolerance
// alone can hide a Cartesian error on this arm. This guard never compensates
// encoders or increases motor commands; it requests a verified stop.
class PhysicalProgressGuard {
public:
  using Point = std::array<double, 3>;
  void reset(const Point &measured, double now) {
    previous_ = measured;
    progress_time_ = now;
    adverse_ = credit_ = 0.;
  }
  std::string check(const Point &measured, const Point &goal, double now) {
    double length = 0., error = 0., projection = 0.;
    for (size_t i = 0; i < 3; ++i) {
      length += std::pow(goal[i] - previous_[i], 2);
      error += std::pow(goal[i] - measured[i], 2);
      projection += (measured[i] - previous_[i]) * (goal[i] - previous_[i]);
    }
    projection /= std::max(1e-12, std::sqrt(length));
    previous_ = measured;
    if (std::sqrt(error) <= .001) {
      progress_time_ = now;
      adverse_ = credit_ = 0.;
      return {};
    }
    adverse_ = std::max(0., adverse_ - projection);
    credit_ = std::max(0., credit_ + projection);
    if (credit_ >= .0005) {
      progress_time_ = now;
      credit_ = 0.;
    }
    if (adverse_ > .002)
      return "El brazo físico se aleja del objetivo; seguimiento detenido";
    if (now - progress_time_ > 3.)
      return "Sin progreso cartesiano medido; seguimiento físico detenido";
    return {};
  }
private:
  Point previous_{};
  double progress_time_ = 0., adverse_ = 0., credit_ = 0.;
};

class MarkerGesture {
public:
  bool accept(unsigned event, const std::string &client,
              const std::string &control) {
    const bool axis = control == "move_0" || control == "move_1" ||
                      control == "move_2" || control == "rotate_0" ||
                      control == "rotate_1" || control == "rotate_2";
    // InteractiveMarkerFeedback: POSE_UPDATE=1, MOUSE_DOWN=4, MOUSE_UP=5.
    if (event == 4 && axis) {
      client_ = client;
      control_ = control;
      dragging_ = true;
      return false;
    }
    if (!dragging_ || client != client_ || control != control_)
      return false;
    if (event == 5) {
      dragging_ = false;
      return false;
    }
    return event == 1;
  }
private:
  bool dragging_ = false;
  std::string client_, control_;
};
enum class Mode { Disabled, Starting, Active, Stopping, Fault };
class ControlSession {
public:
  explicit ControlSession(bool has_gripper) : has_gripper_(has_gripper) {}
  bool requestEnable(bool trajectory_busy, bool state_fresh) {
    if (mode_ != Mode::Disabled || trajectory_busy || !state_fresh)
      return false;
    mode_ = Mode::Starting;
    return true;
  }
  void requestDisable() { mode_ = Mode::Stopping; }
  void confirmControllers(bool live_active, bool trajectory_active) {
    if (mode_ == Mode::Starting && live_active && !trajectory_active)
      mode_ = Mode::Active;
    else if (mode_ == Mode::Stopping && !live_active && trajectory_active)
      mode_ = Mode::Disabled;
    else
      mode_ = Mode::Fault;
  }
  void fault(const std::string &reason) {
    reason_ = reason;
    mode_ = Mode::Fault;
  }
  Mode mode() const { return mode_; }
  static bool needsTracking(bool received_target, bool changed,
                            double linear_error, double angular_error) {
    return received_target &&
           (changed || linear_error > .001 || angular_error > .01);
  }
  static double physicalGoalStep(double measured, double goal) {
    return measured + std::clamp(goal - measured, -.03, .03);
  }
  static std::vector<double> physicalGoalPreview(
      const std::vector<double> &measured, const std::vector<double> &goal) {
    auto result = measured;
    double fraction = 1.;
    for (size_t i = 0; i < measured.size(); ++i)
      fraction = std::min(fraction, .03 / std::max(1e-12, std::abs(goal[i] - measured[i])));
    for (size_t i = 0; i < measured.size(); ++i)
      result[i] += fraction * (goal[i] - measured[i]);
    return result;
  }
  static double cartesianStepScale(double linear_step, double angular_step,
                                   double period = .02) {
    double scale = 1.;
    if (linear_step > .03 * period)
      scale = std::min(scale, .03 * period / linear_step);
    if (angular_step > .2 * period)
      scale = std::min(scale, .2 * period / angular_step);
    return scale;
  }
  static double gripperInitialSetpoint(double measured) {
    // Gazebo can report a few nanoradians outside a stopped joint limit.
    // Normalize only roundoff-sized excursions; keep genuinely invalid readings
    // invalid.
    if (measured >= -.5 - 1e-6 && measured <= .15 + 1e-6)
      return std::clamp(measured, -.5, .15);
    return measured;
  }
  static bool validGripper(double v) {
    return std::isfinite(v) && v >= -.5 && v <= .15;
  }
  std::string checkWatchdog(double now, double heartbeat, double joints,
                            double servo, double joints_timeout = .5) const {
    if (now - heartbeat > .75)
      return "Se perdió la conexión con el panel";
    if (now - joints > joints_timeout)
      return "Se perdió el estado medido";
    if (now - servo > .25)
      return "El control continuo dejó de responder";
    return {};
  }

private:
  bool has_gripper_;
  Mode mode_ = Mode::Disabled;
  std::string reason_;
};
} // namespace mycobot_280_live_control
