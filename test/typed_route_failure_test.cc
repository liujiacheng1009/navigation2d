#include <cassert>
#include <fstream>
#include <string>

#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/test/navigation_plugin_fixture.h"

int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  const auto start = MakePose2d(1., 1., 0.), goal = MakePose2d(2., 1., 0.);
  struct Case {
    GlobalPlanStatus status;
    const char* reason;
    InitialPlanFailure expected;
  };
  // Diagnostics are for people. Localized, empty or misleading text must not
  // change application recovery policy for a modern planner plugin.
  for (const auto& sample :
       {Case{GlobalPlanStatus::kStartOccupied, "起点碰撞", InitialPlanFailure::kStartOccupied},
        Case{GlobalPlanStatus::kGoalOccupied, "no route", InitialPlanFailure::kGoalOccupied},
        Case{GlobalPlanStatus::kNoPath, "goal occupied", InitialPlanFailure::kNoRoute},
        Case{GlobalPlanStatus::kNoPath, "", InitialPlanFailure::kNoRoute},
        Case{GlobalPlanStatus::kBackendError, "no path", InitialPlanFailure::kBackendError},
        Case{GlobalPlanStatus::kInvalidInput, "start occupied", InitialPlanFailure::kInvalidInput},
        Case{GlobalPlanStatus::kBudgetExceeded, "goal occupied",
             InitialPlanFailure::kBudgetExceeded},
        Case{GlobalPlanStatus::kCancelled, "no route", InitialPlanFailure::kCancelled}}) {
    test::NavigationPluginFixture fixture;
    fixture.calls.plan = [&](const auto&) {
      return GlobalPlanResult{sample.status, {}, {}, sample.reason};
    };
    NavigationSession session(fixture.Registry());
    session.Begin(fixture.config, fixture.map_path, goal, {});
    const auto probe = session.Probe(start, {}, 0.);
    assert(probe.failure == sample.expected && !probe.has_route());
    assert(probe.state.planning_status == sample.status);
    assert(fixture.calls.plans == 1 && fixture.calls.controls == 0);
    if (sample.status != GlobalPlanStatus::kBudgetExceeded &&
        sample.status != GlobalPlanStatus::kCancelled) {
      assert(probe.failure_reason() ==
             (std::string(sample.reason).empty() ? "global planning failed" : sample.reason));
    }
  }
  // A geometrically invalid returned chord must not trigger an open-area
  // pinch retry. This rejection belongs to route admission, not the backend.
  test::NavigationPluginFixture crossing;
  {
    std::ofstream file(crossing.map_path);
    file
        << R"({"width":40,"height":40,"resolution":0.1,"rectangles":[[0,0,40,40,0],[15,0,17,40,254]]})";
  }
  NavigationSession invalid(crossing.Registry());
  invalid.Begin(crossing.config, crossing.map_path, goal, {});
  const auto unsafe = invalid.Probe(start, {}, 0.);
  assert(unsafe.failure == InitialPlanFailure::kInvalidRoute);
  assert(unsafe.state.route_rejection == RouteRejection::kFootprintInvalid);
  assert(crossing.calls.controls == 0 && invalid.path().empty());
  // A real A* route rejected by signature cooling is not an arbitrary NoPath.
  test::NavigationPluginFixture fixture;
  fixture.config.planner = "astar";
  fixture.config.controller = "rpp";
  NavigationSession session;
  session.Begin(fixture.config, fixture.map_path, goal, {});
  const auto route = session.Probe(start, {}, 0.);
  assert(route.has_route() && route.state.path_signature != 0);
  session.Begin(fixture.config, fixture.map_path, goal, {route.state.path_signature});
  const auto excluded = session.Probe(start, {}, 1.);
  assert(excluded.failure == InitialPlanFailure::kExcludedRoute);
  assert(excluded.state.planning_status == GlobalPlanStatus::kNoPath);
  assert(excluded.state.route_rejection == RouteRejection::kSignatureExcluded);
  assert(excluded.failure_reason() == "failed first-segment replay");
  assert(session.path().empty());
  session.Begin(fixture.config, fixture.map_path, goal, {});
  const auto fresh = session.Probe(start, {}, 2.);
  assert(fresh.has_route() && fresh.state.route_rejection == RouteRejection::kNone);
}
