#include <algorithm>
#include <limits>
#include <stdexcept>

#include "navigation2d/test/goal_failure_fixture.h"
using namespace navigation2d;
using namespace navigation2d::exploration;
using navigation2d::test::FailureScene;
int main() {
  FailureScene exhausted;
  auto pending = exhausted.goal;
  pending.x = pending.frontier_x = 2.;
  exhausted.entry.SetPending(pending);
  exhausted.recovery.Begin(1., 1.);
  exhausted.recovery.wall_scrapes = 2;
  exhausted.on_peel = [&] {
    assert(!exhausted.entry.pending() && !exhausted.entry.locked());
    assert(!exhausted.recovery.active && exhausted.recovery.wall_scrapes == 0);
    assert(exhausted.tour.goals().size() == 1);
  };
  exhausted.execution.LeavePinch("pinch", exhausted);
  assert(
      (exhausted.events == std::vector<std::string>{"cool-route", "report-pinch", "stop", "demote",
                                                    "cancel", "clear-display", "peel", "rescue",
                                                    "delay-selection", "status:SELECTING"}));
  assert(exhausted.demoted_goal && exhausted.demoted_goal->x == 2.);
  assert(exhausted.explorer.failed_goals() == 1 && exhausted.message == "pinch");

  FailureScene peel;
  peel.escape = true;
  peel.on_peel = [&] { peel.recovery.resume_after_escape = true; };
  peel.execution.LeavePinch("pinch", peel);
  assert(peel.events.back() == "status:RECOVERING" && peel.recovery.resume_after_escape);
  assert(std::find(peel.events.begin(), peel.events.end(), "rescue") == peel.events.end());
  FailureScene rescue;
  rescue.rescue = true;
  rescue.execution.LeavePinch("pinch", rescue);
  assert(rescue.events.back() == "rescue");

  FailureScene home;
  home.mission.BeginReturning();
  home.execution.LeavePinch("pinch", home);
  assert(home.explorer.failed_goals() == 0 && !home.demoted_goal);
  assert(home.events[3] == "cancel");
  FailureScene invalid;
  invalid.goal.x = std::numeric_limits<double>::quiet_NaN();
  invalid.execution.LeavePinch("pinch", invalid);
  assert(invalid.explorer.failed_goals() == 0 && !invalid.demoted_goal);
  assert(invalid.tour.goals().size() == 1 && invalid.events.back() == "status:SELECTING");
  FailureScene exception;
  exception.on_peel = [] { throw std::runtime_error("peel"); };
  try {
    exception.execution.LeavePinch("pinch", exception);
    assert(false);
  } catch (const std::runtime_error&) {
  }
  assert(exception.events.back() == "peel" && exception.tour.goals().size() == 1);
}
