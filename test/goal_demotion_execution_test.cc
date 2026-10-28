#include <cmath>
#include <limits>
#include <stdexcept>

#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/application/exploration/viewpoint_selection.h"
#include "navigation2d/test/goal_tracking_fixture.h"
using namespace navigation2d;
using namespace navigation2d::exploration;
using navigation2d::test::TrackingScene;
namespace {
struct Scene : TrackingScene {
  CompletionExecution completion;
  ViewpointRejections rejections;
  ViewpointSelection selection{explorer, entry, tour, completion, rejections};
  Scene() {
    recovery.Begin(goal.x, goal.y);
    recovery.wall_scrapes = 2;
    recovery.resume_after_escape = true;
  }
  void ClearRecoveryDisplay() override {
    assert(!entry.pending() && !entry.locked() && !navigation.has_session());
    assert(recovery.active && tour.goals().size() == 2);
    events.emplace_back("clear-display");
    goal.x = goal.y = std::numeric_limits<double>::quiet_NaN();
  }
  void Demote() { execution.DemoteActive(navigation, selection, "leave", *this); }
  void CheckCommitted() {
    assert(!recovery.active && recovery.wall_scrapes == 0 && !recovery.resume_after_escape);
    assert(tour.goals().size() == 1 && std::isnan(goal.x));
    assert(!selection.Due(3499999999LL) && selection.Due(3500000000LL));
    assert(commands.empty());  // Stop remains owned by the triggering execution stage.
  }
};
}  // namespace
int main() {
  Scene pending;
  auto mouth = pending.goal;
  mouth.x = mouth.frontier_x = 1.2;
  pending.entry.SetPending(mouth);
  pending.mission.BeginCompleting();
  pending.on_status = [&] { pending.CheckCommitted(); };
  pending.Demote();
  assert(pending.demoted_goal && pending.demoted_goal->x == mouth.x);
  assert(pending.explorer.failed_goals() == 1);
  assert(pending.mission.phase() == MissionPhase::kCompleting);
  assert((pending.events == std::vector<std::string>{"demote", "clear-display", "deadline", "clock",
                                                     "status:SELECTING"}));
  assert(pending.message == "leave");

  Scene active;
  const auto original = active.goal;
  active.Demote();
  assert(active.demoted_goal && active.demoted_goal->x == original.x);
  active.CheckCommitted();

  Scene intent;
  intent.returning.AcceptNavigationMode(true);
  intent.entry.SetPending(intent.goal);
  intent.Demote();
  assert(!intent.demoted_goal && intent.explorer.failed_goals() == 0);
  intent.CheckCommitted();
  Scene phase;
  phase.mission.BeginReturning();
  phase.Demote();
  assert(!phase.demoted_goal && phase.explorer.failed_goals() == 0);
  phase.CheckCommitted();

  Scene report;
  report.on_status = [] { throw std::runtime_error("status"); };
  try {
    report.Demote();
    assert(false);
  } catch (const std::runtime_error&) {
    report.CheckCommitted();
  }
}
