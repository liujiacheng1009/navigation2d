#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/navigation_dispatch_execution_fixture.h"
#include "navigation2d/test/navigation_plugin_fixture.h"
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  test::NavigationPluginFixture fixture;
  const auto valid = fixture.calls.plan;
  NavigationProfiles profiles;
  profiles.explore.global = fixture.config.planner;
  profiles.explore.local = fixture.config.controller;
  FrontierExplorer explorer;
  NavigationSession session(fixture.Registry());
  NavigationDispatch dispatch(explorer, profiles);
  test::NavigationDispatchRecorder runtime(session, fixture.map_path);
  ExplorationGoal goal;
  goal.x = 2.;
  goal.y = 1.;
  assert(dispatch.Start(goal, false, false, session, runtime));
  assert(runtime.events ==
         std::vector<std::string>({"begin", "map", "signatures", "observed", "probe", "retreat",
                                   "wall", "margin", "timeout", "accepted"}));
  assert(dispatch.goal().x == 2. && dispatch.algorithms().global == fixture.config.planner);
  fixture.calls.plan = [](const auto&) {
    return GlobalPlanResult{GlobalPlanStatus::kNoPath, {}, {}, "custom no route"};
  };
  runtime.events.clear();
  auto rejected = goal;
  rejected.x = 2.5;
  assert(!dispatch.Start(rejected, false, false, session, runtime));
  assert(runtime.events ==
         std::vector<std::string>({"begin", "map", "signatures", "observed", "probe", "retreat",
                                   "rejected", "notice", "retry"}));
  assert(!session.has_session() && dispatch.goal().x == 2.);

  fixture.calls.plan = valid;
  // A valid first zero sample is accepted; handoff execution is temporal.
  fixture.calls.control = [](const auto&) {
    return LocalControlResult{LocalControlStatus::kSuccess, {}, {}};
  };
  assert(dispatch.Start(goal, false, false, session, runtime));
  assert(dispatch.ReplanLatest("map update", false, session, runtime));
  assert(dispatch.ReplanLatest("map update", false, session, runtime));
  assert(!dispatch.ReplanLatest("map update", false, session, runtime));
  assert(runtime.latest == 2);
  for (int i = 0; i < 3; ++i) {
    assert(dispatch.ReplanAfterCut(false, session, runtime));
  }
  assert(!dispatch.ReplanAfterCut(false, session, runtime) && dispatch.HasRouteCutReplans());
  assert(runtime.cuts == 3);
  dispatch.ClearGoal();
  assert(std::isnan(dispatch.goal().x) && session.stage() == SessionStage::kActive);

  // Preserve the existing exception boundary: map preparation is outside the
  // guarded navigation phase; a post-accept observer failure cancels execution
  // while retaining already-committed target identity for diagnostics.
  runtime.throw_commit = true;
  assert(!dispatch.Start(rejected, false, false, session, runtime));
  assert(!session.has_session() && dispatch.goal().x == 2.5);
  assert(runtime.diagnostic == "accepted goal observer failed while reporting a transient publisher exception");
  runtime.throw_commit = false;
  assert(dispatch.Start(goal, false, false, session, runtime));
  runtime.throw_prepare = true;
  runtime.events.clear();
  try {
    static_cast<void>(dispatch.Start(rejected, false, false, session, runtime));
    assert(false);
  } catch (const std::runtime_error& error) {
    assert(std::string(error.what()) == "snapshot stream");
  }
  assert(runtime.events == std::vector<std::string>({"begin", "map"}));
  assert(session.stage() == SessionStage::kActive && dispatch.goal().x == 2.);
}
