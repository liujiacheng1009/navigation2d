#include "navigation2d/application/command_safety_pipeline.h"

#include <stdexcept>
#include <type_traits>
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("command safety pipeline contract failed");
  }
}
}  // namespace
int main() {
  using namespace navigation2d;
  static_assert(!std::is_constructible_v<CommandSafetyPipeline, NavigationConfig&&>);
  NavigationConfig config;
  config.robot_radius = .18;
  config.collision_monitor_trigger_cycles = 1;
  config.collision_monitor_release_cycles = 1;
  config.collision_monitor_source_timeout = .3;
  CommandSafetyPipeline pipeline(config);
  const auto pose = MakePose2d(0., 0., 0.);
  LaserScan clear{0., .1, .02, 5., {4.}};
  pipeline.UpdateLaserScan(pose, clear);
  pipeline.SetPredictions({{.1, 0., 0., 0., .1, 0., 0.}});
  const auto vetoed = pipeline.Filter(pose, {.2, 0.}, 0.);
  Require(vetoed.command.linear == 0. && vetoed.command.angular == 0.);
  Require(vetoed.action == CollisionMonitorAction::kNone);
  Require(pipeline.predictions().size() == 1);
  pipeline.SetPredictions({});
  // The dynamic veto must still have consumed the pending lidar observation.
  // Skipping raw monitoring would falsely refresh that old scan at t=0.4.
  const auto stale = pipeline.Filter(pose, {.2, 0.}, .4);
  Require(stale.action == CollisionMonitorAction::kSourceTimeout && stale.command.linear == 0.);
  pipeline.UpdateLaserScan(pose, clear);
  const auto resumed = pipeline.Filter(pose, {.2, 0.}, .41);
  Require(resumed.action == CollisionMonitorAction::kNone && resumed.command.linear == .2);
  pipeline.SetPredictions({{.4, 0., 0., 0., .1, 0., 0.}});
  Require(pipeline.DynamicTtc(pose, {.2, 0.}) > 0.);
  NavigationConfig raw_disabled = config;
  raw_disabled.collision_monitor_enabled = false;
  CommandSafetyPipeline independent(raw_disabled);
  independent.SetPredictions({{.1, 0., 0., 0., .1, 0., 0.}});
  Require(independent.Filter(pose, {.2, 0.}, 0.).command.linear == 0.);
}
