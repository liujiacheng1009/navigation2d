#include "navigation2d/application/exploration/goal_execution_watch.h"

#include <cassert>
#include <limits>

#include "navigation2d/application/navigation_system.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
struct Clock final : GoalExecutionClock {
  std::int64_t now_ns = 0;
  mutable int reads = 0;
  std::int64_t ExecutionNowNs() const override {
    ++reads;
    return now_ns;
  }
};
}  // namespace
int main() {
  GoalExecutionWatch watch;
  Clock clock;
  assert(!watch.ProgressOlderThan(0., clock) && clock.reads == 0);
  watch.Start(1000000000LL, 1000000000LL);
  watch.SetTimeout(10.);
  clock.now_ns = 11000000000LL;
  assert(!watch.TimedOut(clock));
  ++clock.now_ns;
  assert(watch.TimedOut(clock));
  clock.now_ns = 0;
  assert(!watch.TimedOut(clock) && !watch.ProgressOlderThan(0., clock));
  clock.reads = 0;
  watch.ObserveProgress(.079, clock);
  assert(clock.reads == 0 && watch.progress_ns() == 1000000000LL);
  watch.ObserveProgress(.08, clock);
  assert(clock.reads == 1 && watch.progress_ns() == 0);
  watch.ObserveProgress(.159, clock);
  watch.ObserveProgress(std::numeric_limits<double>::quiet_NaN(), clock);
  assert(clock.reads == 1);
  watch.SetTimeout(55.);
  watch.Start(1000000000LL, 1000000000LL);
  NavigationState state;
  state.status = NavigationStatus::kBlocked;
  clock.reads = 0;
  assert(watch.FailureDue(state, false, 1., false, true, clock) && clock.reads == 0);
  state.status = NavigationStatus::kNavigating;
  state.phase = NavigationPhase::kDockToGoal;
  state.path_progress_m = 1.;
  state.requested_command.linear = .2;
  clock.now_ns = 100000000000LL;
  assert(!watch.FailureDue(state, true, 1., false, true, clock));
  assert(watch.FailureDue(state, false, 1., false, true, clock));
  state.path_progress_m = 0.;
  state.recoveries = 3;
  clock.now_ns = 21000000000LL;
  assert(!watch.FailureDue(state, true, 1., false, true, clock));
  ++clock.now_ns;
  assert(watch.FailureDue(state, true, 1., false, true, clock));
  state.recoveries = 0;
  state.requested_command.linear = 0.;
  clock.now_ns = 9000000000LL;
  assert(!watch.FailureDue(state, true, .1, true, false, clock));
  ++clock.now_ns;
  assert(watch.FailureDue(state, true, .1, true, false, clock));
  state.path_progress_m = .1;
  clock.now_ns = 6000000000LL;
  assert(!watch.FailureDue(state, true, .1, true, false, clock));
  ++clock.now_ns;
  assert(watch.FailureDue(state, true, .1, true, false, clock));
  assert(!watch.FailureDue(state, true, 1., false, true, clock));
  state.collision_monitor_action = CollisionMonitorAction::kStop;
  state.requested_command.linear = .2;
  state.minimum_ttc_s = .1;
  clock.now_ns = 3000000000LL;
  assert(!watch.FailureDue(state, true, .1, true, false, clock));
  ++clock.now_ns;
  assert(watch.FailureDue(state, true, .1, true, false, clock));
  state.minimum_ttc_s = .15;
  assert(!watch.FailureDue(state, true, .1, true, false, clock));
}
