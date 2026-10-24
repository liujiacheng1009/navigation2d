#include <cmath>
#include <limits>
#include <stdexcept>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/navigation_profile.h"
#include "navigation2d/application/exploration/navigation_telemetry.h"
#include "navigation2d/application/exploration/planning_snapshot.h"
#include "navigation2d/application/exploration/recovery_policy.h"
#include "navigation2d/application/exploration/recovery_session.h"
#include "navigation2d/application/exploration/return_coordinator.h"
#include "navigation2d/application/exploration/room_visit_ledger.h"
#include "navigation2d/application/exploration/route_validator.h"
#include "navigation2d/geometry/robot_geometry.h"

namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("exploration contract failed");
  }
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  NavigationProfiles profiles;
  auto normal = BuildNavigationProfile({.1, false, false, false, false}, profiles.explore);
  auto entry = BuildNavigationProfile({.1, true, false, false, false}, profiles.entry);
  auto pinch = BuildNavigationProfile({.1, false, true, false, false}, profiles.explore);
  auto home = BuildNavigationProfile({.1, false, false, true, true}, profiles.returning);
  Require(normal.planner == "theta_star" && normal.controller == "rpp");
  Require(entry.planner == "astar" && !entry.smoother_enabled);
  Require(normal.inflation_radius == .36 && pinch.inflation_radius == .28 &&
          entry.inflation_radius == .26);
  Require(normal.robot_radius == entry.robot_radius && entry.robot_radius == kPlanningRadius());
  Require(home.goal_xy_tolerance == .04 && normal.goal_xy_tolerance == .18);
  Require(entry.collision_monitor_enabled && entry.collision_monitor_passage_mode);
  auto custom = BuildNavigationProfile({}, {"custom_global", "custom_local"});
  Require(custom.planner == "custom_global" && custom.controller == "custom_local");

  ExplorationGrid map{40, 30, .1, -1., -1., std::vector<std::int8_t>(1200, -1)};
  auto index = [&](double x, double y) {
    return static_cast<int>(std::floor((y - map.origin_y) / map.resolution)) * map.width +
           static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
  };
  const int endpoint = index(.18 + .8, 0.);
  map.cells[endpoint] = 100;
  const auto original = map.cells;
  std::vector<float> ranges{.8F};
  SnapshotRequest request;
  request.robot = MakePose2d(0., 0., 0.);
  request.goal.x = 1.;
  request.goal.y = 0.;
  request.scan = ScanEvidenceView{0., 0., .02, 5., ranges};
  request.map_revision = 42;
  auto snapshot = BuildPlanningSnapshot(map, request);
  Require(map.cells == original && snapshot.snapshot.map_revision == 42);
  Require(snapshot.snapshot.free_mask[index(.5, 0.)] != 0);
  Require(snapshot.snapshot.free_mask[endpoint] == 0);
  Require(snapshot.snapshot.free_mask[index(2., 1.)] == 0);
  Require(snapshot.snapshot.free_mask[index(0., 0.)] != 0);
  PlanningSnapshotBuilder reusable_builder;
  auto first = reusable_builder.Build(map, request);
  const auto* mask_storage = first.snapshot.free_mask.data();
  auto second = reusable_builder.Build(map, request, std::move(first.snapshot));
  Require(second.snapshot.free_mask.data() == mask_storage);
  Require(second.snapshot.free_mask == snapshot.snapshot.free_mask);

  ExplorationGoal mouth;
  mouth.x = .5;
  mouth.frontier_x = .6;
  request.entry = mouth;
  auto entering = BuildPlanningSnapshot(map, request);
  Require(entering.cells[index(.8, 0.)] == 0);
  Require(entering.cells[endpoint] == 100);

  ExplorationGrid live{40, 30, .1, -1., -1., std::vector<std::int8_t>(1200, 0)};
  live.cells[index(1., 0.)] = 100;
  std::vector<std::pair<double, double>> path{{0., 0.}, {1.5, 0.}};
  Require(RouteCutsMappedObstacle(live, path, {}, 0., 0., 0., {}, false));
  Require(!RouteCutsMappedObstacle(live, {{0., 0.}, {0., .8}}, {}, 0., 0., 0., {}, false));
  // The old plan's evidence is reprojected by world position, not raw index.
  PlanningSnapshot accepted;
  accepted.grid = {60, 50, .1, -2., -2.};
  accepted.free_mask.assign(3000, 1);
  Require(!RouteCutsMappedObstacle(live, path, accepted, 0., 0., 0., {}, false));

  RoomVisitLedger ledger;
  ledger.Remember(mouth, request.goal);
  ledger.Remember(mouth, request.goal);
  Require(ledger.Seeds().size() == 1);
  Require(ledger.TryVisit(request.goal) == 1 && ledger.TryVisit(request.goal) == 2);
  auto nearby = request.goal;
  nearby.x += .1;
  Require(ledger.TryVisit(nearby) == 0);
  ledger.Forget(mouth);
  Require(ledger.empty());
  NavigationTelemetry telemetry;
  NavigationState state;
  state.command = {.2, -.1};
  state.controller_diagnostics.backend = ControllerBackend::kRpp;
  state.controller_solve_us = 12.;
  telemetry.Capture(state, {.15, -.05});
  Require(telemetry.state().backend == "rpp" && telemetry.state().cmd_linear == .2);
  Require(telemetry.state().measured_linear == .15 && telemetry.state().controller_solve_us == 12.);
  EntryCoordinator entry_session;
  entry_session.Lock(mouth, true, {100, 7});
  entry_session.CountAttempt();
  Require(entry_session.pending().has_value() && entry_session.attempts() == 1);
  Require(entry_session.Satisfied(.95, 0., request.goal, .7));
  Require(!entry_session.Satisfied(.5, 0., request.goal, .7));
  entry_session.StartCrawl();
  entry_session.TickCrawl();
  Require(entry_session.crawl_ticks() == 2);
  entry_session.ClearPending();
  Require(entry_session.locked().has_value() && !entry_session.pending());
  entry_session.SetPending(entry_session.locked());
  Require(entry_session.attempts() == 1);
  entry_session.Clear();
  Require(!entry_session.locked() && entry_session.attempts() == 0);

  BodyRanges boxed{.1, .5, .1, .1, .1};
  const auto escape = SelectEscape(boxed, false);
  Require(escape && escape->command.linear == -.20 && escape->hold_s == 1.8);
  Require(SelectEscape(boxed, true)->hold_s == 2.2);
  boxed.back = .1;
  Require(!SelectEscape(boxed, false));
  Require(AdjustEscape({-.2, 0.}, boxed).linear == 0.);
  GoalRecoverySession recovery;
  recovery.Begin(1., 2.);
  recovery.side_peels = 1;
  recovery.Begin(1.1, 2.);
  Require(recovery.side_peels == 1);
  recovery.Begin(3., 2.);
  Require(recovery.side_peels == 0);

  ReturnCoordinator returning;
  ExplorationGoal spawn;
  spawn.x = .5;
  spawn.y = .5;
  ExplorationGrid open{50, 50, .1, -1., -1., std::vector<std::int8_t>(2500, 0)};
  const auto home_goal = returning.Home(&open, spawn);
  Require(home_goal.x == spawn.x && home_goal.y == spawn.y);
  const std::vector<std::pair<double, double>> return_trail{
      {.5, .5}, {1., .5}, {1.5, .5}, {2., .5}};
  auto step = returning.NextStep(return_trail, spawn, 2., .5, {});
  Require(step.kind == ReturnStepKind::kBreadcrumb && step.goal.x < 2.);
  returning.UseBreadcrumbs(true);
  Require(returning.NextStep(return_trail, spawn, 2., .5, step.goal).kind ==
          ReturnStepKind::kRepeated);
  returning.MarkPartial();
  returning.CountEscape();
  Require(returning.partial() && returning.escapes() == 1);
}
