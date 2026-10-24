#include "navigation2d/application/execution_watchdog.h"

#include <stdexcept>

#include "navigation2d/application/navigation_failure.h"
#include "navigation2d/geometry/pose_2d.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("execution watchdog contract failed");
  }
}
}  // namespace
int main() {
  using namespace navigation2d;
  using Failure = NavigationFailure;
  const ExecutionWatchdogSettings settings{.1, .1, 1., 1};
  ExecutionWatchdog zero(settings);
  for (int i = 0; i < 2; ++i) {
    Require(zero.ObserveController(false, false, false) == Failure::kNone);
  }
  Require(zero.ObserveController(false, false, false) == Failure::kControllerRetry);
  zero.NewRoute();  // A new route cannot renew the per-goal recovery budget.
  for (int i = 0; i < 2; ++i) {
    Require(zero.ObserveController(false, false, false) == Failure::kNone);
  }
  Require(zero.ObserveController(false, false, false) == Failure::kControllerExhausted);
  zero.NewGoal();
  for (int i = 0; i < 2; ++i) {
    Require(zero.ObserveController(false, false, false) == Failure::kNone);
  }
  Require(zero.ObserveController(false, false, false) == Failure::kControllerRetry);
  Require(zero.ObserveController(false, false, false) == Failure::kNone);
  Require(zero.ObserveController(false, false, false) == Failure::kNone);
  zero.ObserveController(false, false, true);  // Deliberate stops end the failure streak.
  Require(zero.ObserveController(false, false, false) == Failure::kNone);

  ExecutionWatchdog progress(settings);
  progress.BeginCycle(0.);
  const auto retry = progress.ObservePrevious({{.2, 0.}, 0., 1.01, false, false});
  Require(retry == Failure::kProgressRetry);
  const auto retry_effects = EffectsFor(retry);
  Require(retry_effects.clear_route && retry_effects.record_recovery && !retry_effects.blocked &&
          retry_effects.reset_replan_timer);
  const auto exhausted = progress.ObservePrevious({{.2, 0.}, 0., 2.02, false, false});
  Require(exhausted == Failure::kProgressExhausted);
  Require(!EffectsFor(exhausted).clear_route && EffectsFor(exhausted).blocked);
  progress.NewGoal();
  progress.BeginCycle(0.);
  Require(progress.ObservePrevious({{}, 0., 100., false, false}) == Failure::kNone);
  Require(progress.ObservePrevious({{.2, 0.}, 0., 100.5, false, false}) == Failure::kNone);
  Require(progress.ObservePrevious({{.2, 0.}, .11, 100.6, false, false}) == Failure::kNone);
  Require(progress.path_anchor_m() == .11);

  ExecutionWatchdog rotating(settings);
  rotating.BeginCycle(0.);
  for (int i = 1; i <= 62; ++i) {
    Require(rotating.ObservePrevious({{0., 1.}, 0., i * .1, false, false}) == Failure::kNone);
  }
  Require(rotating.ObservePrevious({{0., 1.}, 0., 6.3, false, false}) ==
          Failure::kTrackingFullTurn);

  ExecutionWatchdog held(settings);
  Require(held.ObserveSafetyStop(true, 0.) == Failure::kNone);
  Require(held.ObserveSafetyStop(true, 1.99) == Failure::kNone);
  Require(held.ObserveSafetyStop(true, 2.) == Failure::kCollisionHold);
  Require(held.ObserveSafetyStop(true, 2.1) == Failure::kNone);

  ExecutionWatchdog alignment(settings);
  alignment.BeginCycle(.5);
  Require(!alignment.ObserveAlignment(1., .5, 0., .5));
  Require(!alignment.ObserveAlignment(.99, .5, 0., 1.));
  const auto docking_timeout = alignment.DockingTimeout(3.51);
  Require(docking_timeout == Failure::kDockingTimeout);
  Require(!EffectsFor(docking_timeout).record_recovery && !EffectsFor(docking_timeout).clear_route);
  Require(alignment.ObserveAlignment(1., .2, 0., 4.));
  Require(alignment.DockingTimeout(6.9) == Failure::kNone);

  ExecutionWatchdog stalled({.06, .1, 1., 1});
  const auto pose = MakePose2d(0., 0., 0.);
  for (int i = 0; i < 16; ++i) {
    Require(stalled.ObserveStall({pose, 0., {}, 0., i * .06, true, true}, StallMode::kTracking) ==
            Failure::kNone);
  }
  Require(stalled.ObserveStall({pose, 0., {}, 0., .96, true, true}, StallMode::kTracking) ==
          Failure::kControllerNoMotion);
  Require(stalled.ObserveStall({pose, 0., {}, 0., 1.02, true, true}, StallMode::kTracking) ==
          Failure::kNone);
  stalled.NewRoute();
  Require(stalled.ObserveStall({pose, 0., {}, 0., 2., true, true}, StallMode::kTracking) ==
          Failure::kNone);
}
