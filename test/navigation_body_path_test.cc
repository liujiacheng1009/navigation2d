#include <cassert>
#include <cmath>

#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/planning/path_geometry.h"
#include "navigation2d/test/navigation_plugin_fixture.h"
int main() {
  using namespace navigation2d;
  // Rotation-only waypoint and reverse motion keep body yaw and endpoints.
  const Path sparse{MakePose2d(1., 1., 0.), MakePose2d(1., 1., .5), MakePose2d(.6, 1., .5)};
  const auto dense = planning_internal::DensifyBodyHeadingRoute(sparse, .08);
  assert(dense.size() > sparse.size());
  assert((dense[1].translation() - sparse.front().translation()).norm() == 0.);
  assert(std::abs(Yaw(dense[1]) - .5) < 1e-12 && std::abs(Yaw(dense.back()) - .5) < 1e-12);

  test::NavigationPluginFixture fixture;
  fixture.calls.plan = [](const auto& request) {
    return GlobalPlanResult{GlobalPlanStatus::kSuccess,
                            {request.start, request.goal},
                            {std::string(request.frame_id), request.costmap.revision(),
                             PathHeadingSemantics::kBodyHeading},
                            {}};
  };
  fixture.calls.control = [](const auto& request) {
    assert(request.path_metadata && request.path_metadata->frame_id == "body-map");
    assert(request.path_metadata->headings == PathHeadingSemantics::kBodyHeading);
    assert(std::abs(Yaw(request.path.front())) < 1e-12);
    return LocalControlResult{LocalControlStatus::kSuccess, {-.1, 0.}, {}};
  };
  const auto start = MakePose2d(1.5, 1., 0.), goal = MakePose2d(.6, 1., 0.);
  const auto registry = fixture.Registry(
      {.reverse_paths = true},
      {.reverse_tracking = true, .in_place_rotation = true, .body_heading_paths = true});
  registry->Validate(fixture.config, {.reverse_paths = true});
  // A new body-heading backend cannot silently fall into legacy projection.
  NavigationSystem legacy(fixture.config, fixture.map_path, registry);
  legacy.SetGoal(goal);
  assert(legacy.ComputeCommand(start, {}, 0.).status == NavigationStatus::kBlocked);
  assert(fixture.calls.controls == 0);

  NavigationExecutionOptions options{"body-map", RouteHeadingPolicy::kPreservePlannerHeading};
  exploration::NavigationSession session(registry);
  session.Begin(fixture.config, fixture.map_path, goal, {}, options);
  auto probe = session.Probe(start, {}, 0.);
  assert(probe.has_route() && probe.state.command.linear < 0.);
  session.Accept();
  auto pose = start;
  NavigationState state;
  for (int tick = 1; tick < 200; ++tick) {
    state = session.Step(pose, {-.1, 0.}, tick * fixture.config.control_period);
    if (state.status == NavigationStatus::kSucceeded) {
      break;
    }
    assert(state.status == NavigationStatus::kNavigating && state.command.angular == 0.);
    pose.translation().x() += state.command.linear * fixture.config.control_period;
  }
  assert(state.status == NavigationStatus::kSucceeded && pose.translation().x() < .8);
  assert(session.path_metadata().headings == PathHeadingSemantics::kBodyHeading);
  std::stop_source cancel;
  cancel.request_stop();
  state = session.Step(pose, {}, 13., {cancel.get_token()});
  assert(state.algorithm_stop == AlgorithmStop::kCancelled && !session.has_session());

  NavigationSystem unsupported(fixture.config, fixture.map_path,
                               fixture.Registry({.reverse_paths = true}), options);
  unsupported.SetGoal(goal);
  state = unsupported.ComputeCommand(start, {}, 0.);
  assert(state.status == NavigationStatus::kBlocked && state.command.linear == 0.);
}
