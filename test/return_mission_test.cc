#include <cassert>
#include <string>
#include <utility>
#include <vector>

#include "navigation2d/geometry/robot_geometry.h"
#include "navigation2d/test/return_mission_fixture.h"
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  using Events = std::vector<std::string>;
  ReturnCoordinator visits;
  test::ReturnMissionRecorder visit(visits);
  visit.visit = true;
  visits.BeginMission(true, visit);
  assert(visit.events == Events{"visit"} && !visits.returning() && !visits.partial());

  ReturnCoordinator direct;
  test::ReturnMissionRecorder accepted(direct);
  direct.CountEscape();
  direct.BeginMission(false, accepted);
  assert(accepted.events ==
         Events({"visit", "phase", "suspend", "coverage", "cool", "home", "accepted"}));
  assert(direct.returning() && direct.escapes() == 0 && !direct.breadcrumbs());
  direct.AcceptNavigationMode(false);
  assert(!direct.returning());
  accepted.events.clear();
  direct.BeginMission(true, accepted);
  assert(accepted.events ==
         Events({"visit", "phase", "suspend", "partial", "cool", "home", "accepted"}));
  assert(direct.partial() && direct.returning());

  ReturnCoordinator fallback;
  test::ReturnMissionRecorder escape(fallback);
  escape.plan = false;
  escape.escape = true;
  escape.coverage = false;
  fallback.BeginMission(false, escape);
  assert(escape.events == Events({"visit", "phase", "suspend", "coverage", "incomplete", "cool",
                                  "home", "trail", "escape", "resume"}));
  assert(fallback.returning() && fallback.escapes() == 1 && !escape.finished);

  ReturnCoordinator failure;
  test::ReturnMissionRecorder unavailable(failure);
  unavailable.plan = false;
  failure.BeginMission(false, unavailable);
  assert(unavailable.finished && !unavailable.success && !failure.partial());
  assert(unavailable.message == "return path unavailable; stopping to protect localization");
  for (int i = 0; i < 5; ++i) {
    failure.CountEscape();
  }
  unavailable.events.clear();
  unavailable.finished = false;
  failure.RecoverMission(ReturnRecoveryMode::kImmobility, "immobile near home", unavailable);
  assert(unavailable.events == Events({"trail", "finish"}));
  assert(unavailable.finished && unavailable.success && failure.partial());
  assert(unavailable.message == "immobile near home");

  ReturnCoordinator idle;
  test::ReturnMissionRecorder pending(idle);
  idle.TickIdle(true, pending);
  assert(pending.events.empty());
  pending.escape = true;
  idle.TickIdle(false, pending);
  assert(pending.events == Events({"escape", "resume-start"}));
  assert(idle.escapes() == 1);
  pending.events.clear();
  pending.escape = false;
  idle.MarkPartial();
  idle.TickIdle(false, pending);
  assert(pending.events == Events({"escape", "trail", "restart"}) && pending.restart_partial);

  const double hop = 1.2 * kSwingClearance();
  const std::vector<std::pair<double, double>> trail{
      {0., 0.}, {hop, 0.}, {2 * hop, 0.}, {3 * hop, 0.}};
  ExplorationGoal home;
  ReturnCoordinator hops;
  test::ReturnMissionRecorder following(hops);
  assert(hops.FollowTrail(trail, home, 3 * hop, 0., {}, following));
  assert(following.events == Events({"phase", "cool", "evidence-start", "navigate", "hop"}));
  assert(following.planned.x == 2 * hop && hops.returning() && hops.breadcrumbs());
  following.events.clear();
  assert(!hops.FollowTrail(trail, home, 3 * hop, 0., following.planned, following));
  assert(following.events.empty());
  ReturnCoordinator near;
  test::ReturnMissionRecorder near_home(near);
  near_home.navigate = false;
  assert(!near.FollowTrail(trail, home, .1, 0., {}, near_home));
  assert(near_home.events == Events({"evidence-start", "navigate"}) && !near.returning());
  near_home.navigate = true;
  assert(near.FollowTrail(trail, home, .1, 0., {}, near_home) && near.returning());
}
