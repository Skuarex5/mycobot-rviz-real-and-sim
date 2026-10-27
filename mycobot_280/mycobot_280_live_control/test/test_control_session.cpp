#include "mycobot_280_live_control/control_session.hpp"
#include <cassert>
#include <limits>
using namespace mycobot_280_live_control;
int main() {
  PhysicalProgressGuard progress;
  progress.reset({0., 0., .1}, 1.);
  assert(progress.check({0., 0., .099}, {0., 0., .103}, 1.2).empty());
  // Real failure: +3 mm requested, encoder/FK endpoint moved -3.976 mm.
  assert(!progress.check({0., 0., .096024}, {0., 0., .103}, 1.4).empty());
  progress.reset({0., 0., .1}, 1.);
  assert(progress.check({0., 0., .101}, {0., 0., .103}, 2.).empty());
  assert(progress.check({0., 0., .102}, {0., 0., .103}, 3.).empty());
  assert(progress.check({0., 0., .103}, {0., 0., .103}, 9.).empty());
  progress.reset({0., 0., .1}, 1.);
  assert(!progress.check({0., 0., .1}, {0., 0., .105}, 4.1).empty());
  // Retargeting without measured progress cannot keep renewing the timer.
  progress.reset({0., 0., .1}, 1.);
  assert(progress.check({0., 0., .1}, {0., 0., .104}, 2.).empty());
  assert(!progress.check({0., 0., .1}, {0., 0., .106}, 4.1).empty());
  MarkerGesture gesture;
  assert(!gesture.accept(1, "rviz", "move_0"));
  assert(!gesture.accept(4, "rviz", "move_0"));
  assert(gesture.accept(1, "rviz", "move_0"));
  assert(!gesture.accept(1, "other", "move_0"));
  assert(!gesture.accept(1, "rviz", "rotate_0"));
  assert(!gesture.accept(5, "rviz", "move_0"));
  assert(!gesture.accept(1, "rviz", "move_0"));
  assert(!gesture.accept(4, "rviz", "unknown"));
  assert(!gesture.accept(1, "rviz", "unknown"));
  assert(ControlSession::cartesianStepScale(.0012, 0.) == .5);
  assert(ControlSession::cartesianStepScale(0., .008) == .5);
  assert(ControlSession::cartesianStepScale(.0001, .001) == 1.);
  // Collision preview follows the same coordinated joint direction as USB.
  auto preview = ControlSession::physicalGoalPreview(
      {0., 0., 0.}, {-.014, .057, -.043});
  assert(std::abs(preview[0] / -.014 - preview[1] / .057) < 1e-12);
  assert(std::abs(preview[2] / -.043 - preview[1] / .057) < 1e-12);
  assert(std::abs(preview[1] - .03) < 1e-12);
  // Physical point targets remain bounded toward the actual IK goal.
  assert(ControlSession::physicalGoalStep(0., .08) == .03);
  assert(ControlSession::physicalGoalStep(0., -.08) == -.03);
  assert(ControlSession::physicalGoalStep(0., .01) == .01);
  assert(ControlSession::cartesianStepScale(.003, .02, .1) == 1.);
  // A successful call for an older pose must not consume a newer goal.
  assert(ControlSession::needsTracking(true, false, .005, 0.));
  assert(ControlSession::needsTracking(true, false, 0., .04));
  assert(!ControlSession::needsTracking(true, false, .0001, .001));
  assert(!ControlSession::needsTracking(false, true, .1, .1));
  assert(ControlSession::gripperInitialSetpoint(.15 + 1e-8) == .15);
  assert(ControlSession::gripperInitialSetpoint(-.5 - 1e-8) == -.5);
  assert(ControlSession::gripperInitialSetpoint(.1) == .1);
  assert(!ControlSession::validGripper(
      ControlSession::gripperInitialSetpoint(.151)));
  ControlSession s(true);
  assert(s.mode() == Mode::Disabled);
  assert(!s.requestEnable(true, true));
  assert(!s.requestEnable(false, false));
  assert(s.requestEnable(false, true));
  assert(s.mode() == Mode::Starting);
  s.confirmControllers(false, true);
  assert(s.mode() == Mode::Fault);
  s.requestDisable();
  s.confirmControllers(false, true);
  assert(s.mode() == Mode::Disabled);
  assert(s.requestEnable(false, true));
  s.confirmControllers(true, false);
  assert(s.mode() == Mode::Active);
  assert(s.checkWatchdog(1., .9, .9, .9).empty());
  assert(!s.checkWatchdog(1., .2, .9, .9).empty());
  assert(!s.checkWatchdog(1., .9, .4, .9).empty());
  assert(!s.checkWatchdog(1., .9, .9, .7).empty());
  assert(ControlSession::validGripper(-.5));
  assert(ControlSession::validGripper(.15));
  assert(!ControlSession::validGripper(.151));
  assert(
      !ControlSession::validGripper(std::numeric_limits<double>::quiet_NaN()));
  assert(
      !ControlSession::validGripper(std::numeric_limits<double>::infinity()));
  s.requestDisable();
  assert(s.mode() == Mode::Stopping);
  s.confirmControllers(false, true);
  assert(s.mode() == Mode::Disabled);
  ControlSession bare(false);
  assert(bare.requestEnable(false, true));
  bare.requestDisable();
  bare.confirmControllers(false, true);
  assert(bare.mode() == Mode::Disabled);
}
