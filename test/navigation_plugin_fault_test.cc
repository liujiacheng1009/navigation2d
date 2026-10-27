#include <cassert>
#include <limits>
#include <stdexcept>

#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/test/navigation_plugin_fixture.h"

int main() {
  using namespace navigation2d;
  const auto start = MakePose2d(1., 1., 0.), goal = MakePose2d(2., 1., 0.);
  for (int fault = 0; fault < 5; ++fault) {
    test::NavigationPluginFixture fixture;
    NavigationSystem nav(fixture.config, fixture.map_path, fixture.Registry());
    nav.SetGoal(goal);
    auto state = nav.ComputeCommand(start, {}, 0.);
    assert(state.status == NavigationStatus::kNavigating && fixture.calls.controls > 0);
    assert(nav.path_metadata().frame_id == "navigation_map");
    if (fault == 0) {
      fixture.calls.control = [](const auto&) {
        return LocalControlResult{
            LocalControlStatus::kSuccess, {std::numeric_limits<double>::quiet_NaN(), 0.}, {}};
      };
    } else if (fault == 1) {
      fixture.calls.control = [](const auto&) -> LocalControlResult {
        throw std::logic_error("faulted solver");
      };
    } else if (fault == 2) {
      fixture.calls.control = [](const auto&) {
        return LocalControlResult{LocalControlStatus::kInfeasible, {.4, .1}, "infeasible"};
      };
    } else if (fault == 3) {
      fixture.calls.control = [](const auto&) {
        return LocalControlResult{LocalControlStatus::kSuccess, {2., 0.}, {}};
      };
    } else {
      fixture.calls.control = [](const auto&) {
        return LocalControlResult{LocalControlStatus::kSuccess, {0., 4.}, {}};
      };
    }
    state = nav.ComputeCommand(start, {}, .06);
    assert(state.status == NavigationStatus::kBlocked);
    assert(state.command.linear == 0. && state.command.angular == 0.);
    assert(state.published_command.linear == 0. && state.published_command.angular == 0.);
    const int controls = fixture.calls.controls;
    fixture.calls.control = [](const auto&) {
      return LocalControlResult{LocalControlStatus::kSuccess, {.1, 0.}, {}};
    };
    assert(nav.ComputeCommand(start, {}, .12).status == NavigationStatus::kBlocked);
    assert(fixture.calls.controls == controls);  // Fault requires a new explicit goal.
    nav.SetGoal(goal);
    assert(nav.ComputeCommand(start, {}, .18).status == NavigationStatus::kNavigating);
  }
  for (int fault = 0; fault < 3; ++fault) {
    test::NavigationPluginFixture fixture;
    fixture.calls.plan = [=](const auto& r) {
      auto target = r.goal;
      if (fault == 0) {
        target.translation().x() = 1e200;
      }
      auto metadata = PlannedPathMetadata{std::string(r.frame_id), r.costmap.revision(),
                                          PathHeadingSemantics::kForwardTangent};
      if (fault == 1) {
        metadata.frame_id = "wrong";
      }
      if (fault == 2) {
        ++metadata.map_revision;
      }
      return GlobalPlanResult{GlobalPlanStatus::kSuccess, {r.start, target}, metadata, {}};
    };
    exploration::NavigationSession session(fixture.Registry());
    session.Begin(fixture.config, fixture.map_path, goal, {});
    const auto probe = session.Probe(start, {}, 0.);
    assert(!probe.has_route() && probe.failure == exploration::InitialPlanFailure::kInvalidInput);
    assert(probe.state.planning_status == GlobalPlanStatus::kInvalidOutput);
    assert(fixture.calls.controls == 0);
    assert(session.stage() == exploration::SessionStage::kPreparing);
  }
  test::NavigationPluginFixture fixture;
  std::stop_source cancellation;
  AlgorithmBudget budget{cancellation.get_token()};
  NavigationSystem nav(fixture.config, fixture.map_path, fixture.Registry());
  nav.SetGoal(goal);
  budget.deadline = std::chrono::steady_clock::now();
  auto state = nav.ComputeCommand(start, {}, 0., budget);
  assert(state.algorithm_stop == AlgorithmStop::kBudgetExceeded && fixture.calls.plans == 0);
  budget.deadline = std::chrono::steady_clock::time_point::max();
  fixture.calls.control = [&](const auto&) {
    cancellation.request_stop();
    return LocalControlResult{LocalControlStatus::kSuccess, {.2, 0.}, {}};
  };
  state = nav.ComputeCommand(start, {}, .06, budget);
  assert(state.algorithm_stop == AlgorithmStop::kCancelled &&
         state.status == NavigationStatus::kIdle);
  assert(state.command.linear == 0. && nav.path().empty());
  const int controls = fixture.calls.controls;
  assert(nav.ComputeCommand(start, {}, .12).status == NavigationStatus::kIdle);
  assert(fixture.calls.controls == controls);
  // Backend factories may fail while resetting a controller for a new route.
  test::NavigationPluginFixture resetting;
  auto broken = std::make_shared<AlgorithmRegistry>();
  broken->RegisterPlanner(resetting.config.planner, [&](const auto&) {
    return std::make_unique<test::RequestPlanner>(resetting.calls);
  });
  int creations = 0;
  broken->RegisterController(resetting.config.controller,
                             [&](const auto&) -> std::unique_ptr<LocalController> {
                               if (++creations == 2) {
                                 throw 42;
                               }
                               return std::make_unique<test::RequestController>(resetting.calls);
                             });
  exploration::NavigationSession reset_session(broken);
  reset_session.Begin(resetting.config, resetting.map_path, goal, {});
  const auto rejected = reset_session.Probe(start, {}, 0.);
  assert(!rejected.has_route() &&
         rejected.failure == exploration::InitialPlanFailure::kBackendError);
  assert(reset_session.path().empty() && resetting.calls.controls == 0);

  // A cancelled probe is never ready, even when the requested goal is nearby.
  test::NavigationPluginFixture near;
  exploration::NavigationSession near_session(near.Registry());
  near_session.Begin(near.config, near.map_path, start, {});
  std::stop_source stopped;
  stopped.request_stop();
  const auto cancelled_probe = near_session.Probe(start, {}, 0., {stopped.get_token()});
  assert(cancelled_probe.failure == exploration::InitialPlanFailure::kCancelled);
  assert(!near_session.has_session() && near.calls.plans == 0 && near.calls.controls == 0);

  // Explicit holds remain distinct from a missing or invalid command.
  test::NavigationPluginFixture holding;
  holding.calls.control = [](const auto&) {
    return LocalControlResult{LocalControlStatus::kIntentionalStop, {}, "holding"};
  };
  NavigationSystem held(holding.config, holding.map_path, holding.Registry());
  held.SetGoal(goal);
  for (int tick = 0; tick < 10; ++tick) {
    auto observed = held.ComputeCommand(start, {}, tick * .06);
    assert(observed.status == NavigationStatus::kNavigating);
    assert(observed.control_status == LocalControlStatus::kIntentionalStop);
    assert(observed.controller_diagnostics.intentional_stop && observed.command.linear == 0.);
  }
  test::NavigationPluginFixture inputs;
  NavigationSystem bounded(inputs.config, inputs.map_path, inputs.Registry());
  bounded.SetGoal(goal);
  auto unindexable = start;
  unindexable.translation().x() = 1e200;
  state = bounded.ComputeCommand(unindexable, {}, 0.);
  assert(state.control_status == LocalControlStatus::kInvalidInput && state.command.linear == 0.);
  assert(inputs.calls.plans == 0 && inputs.calls.controls == 0);
  assert(bounded.ComputeCommand(start, {}, .06).status == NavigationStatus::kNavigating);
}
