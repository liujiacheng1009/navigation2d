#include <stdexcept>

#include "navigation2d/test/goal_failure_fixture.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
using navigation2d::test::FailureScene;
void Recover(FailureScene& scene, bool entry = false, bool tip = false) {
  scene.execution.RecoverImmobility(scene.motion, {1., 1., .1, entry, tip}, scene);
}
}  // namespace
int main() {
  FailureScene home;
  home.mission.BeginReturning();
  home.entry.SetPending(home.goal);
  Recover(home, true, true);
  assert(home.events == std::vector<std::string>{"return-stalled"});
  FailureScene intent;
  intent.returning.AcceptNavigationMode(true);
  Recover(intent);
  assert(intent.events == std::vector<std::string>{"return-stalled"});

  FailureScene entry;
  entry.entry.Lock(entry.goal, false, {});
  entry.entry.ClearPending();
  Recover(entry, true, true);
  assert((entry.events == std::vector<std::string>{"prepare-entry", "peel", "probe"}));
  assert(entry.entry.pending() && entry.explorer.failed_goals() == 0);

  FailureScene open;
  open.on_replan = [&] { open.motion.ResetOpenReplans(); };
  Recover(open);
  assert((open.events == std::vector<std::string>{"cool-route", "replan", "status:NAVIGATING"}));
  open.events.clear();
  Recover(open);
  assert((open.events ==
          std::vector<std::string>{"cool-route", "delay-selection", "status:SELECTING"}));
  assert(open.explorer.failed_goals() == 0 && open.tour.goals().size() == 2);
  open.events.clear();
  Recover(open);
  assert(open.events[1] == "replan");  // Exhaustion reset opens the next bounded session.
  FailureScene rejected;
  rejected.replan = false;
  Recover(rejected);
  assert((rejected.events ==
          std::vector<std::string>{"cool-route", "replan", "delay-selection", "status:SELECTING"}));

  FailureScene tip;
  tip.recovery.Begin(1., 1.);
  tip.recovery.wall_scrapes = 1;
  tip.escape = true;
  Recover(tip, false, true);
  assert((tip.events ==
          std::vector<std::string>{"demote", "replay-start", "delay-selection", "peel"}));
  assert(tip.explorer.failed_goals() == 1 && tip.recovery.wall_scrapes == 1);
  assert(tip.tour.goals().size() == 2);
  FailureScene rescue;
  rescue.rescue = true;
  Recover(rescue, false, true);
  assert((rescue.events ==
          std::vector<std::string>{"demote", "replay-start", "delay-selection", "peel", "rescue"}));
  FailureScene exhausted;
  Recover(exhausted, false, true);
  assert((exhausted.events == std::vector<std::string>{"demote", "replay-start", "delay-selection",
                                                       "peel", "rescue", "demote-active"}));
  assert(exhausted.message == "immobility tip thrash; demoting cooled viewpoint");

  FailureScene exception;
  exception.on_replan = [] { throw std::runtime_error("retry"); };
  try {
    Recover(exception);
    assert(false);
  } catch (const std::runtime_error&) {
  }
  assert((exception.events == std::vector<std::string>{"cool-route", "replan"}));
}
