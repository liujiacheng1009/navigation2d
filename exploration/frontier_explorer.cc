#include "navigation2d/exploration/frontier_explorer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "navigation2d/exploration/doorway_geometry.h"
#include "navigation2d/exploration/entry_probe_search.h"
#include "navigation2d/exploration/frontier_boundary_queries.h"
#include "navigation2d/exploration/frontier_candidate_stage.h"
#include "navigation2d/exploration/frontier_reachability.h"
#include "navigation2d/exploration/frontier_travel_graph.h"
#include "navigation2d/exploration/leftover_viewpoints.h"
#include "navigation2d/exploration/mouth_goal_stages.h"
#include "navigation2d/exploration/unvisited_space.h"
#include "navigation2d/exploration/viewpoint_settlement.h"

namespace navigation2d {
using frontier_internal::CorridorMouthAt;

FrontierExplorer::FrontierExplorer(FrontierExplorerConfig config)
    : config_(std::move(config)) {}

void FrontierExplorer::UpdateMap(ExplorationGrid map) {
  maps_.Update(std::move(map), [&](const ExplorationGrid& observed) {
    unresolved_frontier_cells_ = maps_.observation_view().CountFrontiers();
    regions_.Update(observed, kPlanningRadius());
  });
}

bool FrontierExplorer::RawFrontier(int col, int row) const {
  return maps_.observation_view().Frontier(col, row);
}

std::pair<double, double> FrontierExplorer::CellCenter(int col, int row) const {
  return maps_.planning_view(config_.footprint_clearance).CellCenter(col, row);
}

bool FrontierExplorer::InSameReachableRegion(double anchor_x, double anchor_y,
                                             double target_x, double target_y,
                                             double max_distance_m) const {
  return frontier_internal::FrontierReachability(
      maps_.planning(), maps_.observation(), config_.footprint_clearance)
      .SameRegion(anchor_x, anchor_y, target_x, target_y, max_distance_m);
}

bool FrontierExplorer::PersistentlyBlacklisted(double x, double y) const {
  return visits_.Persistent(x, y);
}
void FrontierExplorer::ClearFailedBlacklist() {
  visits_.ClearFailures();
}

void FrontierExplorer::RefreshDebugFrontierCells() {
  frontier_internal::RefreshFrontierDebugCells(maps_.observation(), debug_snapshot_);
}

std::vector<ExplorationGoal> FrontierExplorer::SelectGoals(
    double robot_x, double robot_y) {
  const auto& map = maps_.planning();
  const auto& observation = maps_.observation();
  raw_frontier_cells_ = 0;
  raw_frontier_clusters_ = 0;
  executable_frontier_goals_ = 0;
  candidate_frontier_goals_ = 0;
  debug_snapshot_ = {};
  auto candidates = frontier_internal::CollectFrontierCandidates(
      {map, observation, config_, visits_, tips_}, robot_x, robot_y);
  debug_snapshot_ = std::move(candidates.debug);
  raw_frontier_clusters_ = candidates.raw_frontier_clusters;
  if (!candidates.reachable_frontier_cells) {
    return {};
  }
  unresolved_frontier_cells_ = *candidates.reachable_frontier_cells;
  raw_frontier_cells_ = *candidates.reachable_frontier_cells;
  auto goals = std::move(candidates.goals);
  // Stage A/B: mouth is the task unit — one goal per doorway/alcove, not per
  // frontier micro-cluster. Entry probe moves the pose into the room when the
  // planning graph already opened that corridor.
  auto mouths = OneGoalPerMouth(std::move(goals), robot_x, robot_y);
  candidate_frontier_goals_ = mouths.size();
  auto stable = StableGoals(std::move(mouths));
  executable_frontier_goals_ = stable.size();
  frontier_internal::AppendStableFrontierDebug(debug_snapshot_, stable);
  return stable;
}

std::optional<ExplorationGoal> FrontierExplorer::SelectLeftoverApproach(
    double robot_x, double robot_y) {
  const auto leftovers = SelectLeftoverApproaches(robot_x, robot_y, 1);
  if (leftovers.empty()) return std::nullopt;
  return leftovers.front();
}

std::vector<ExplorationGoal> FrontierExplorer::SelectLeftoverApproaches(
    double robot_x, double robot_y, int max_count) {
  auto selection = frontier_internal::SelectLeftoverViewpoints(
      {maps_.planning(), maps_.observation(), config_.footprint_clearance, tips_, visits_},
      robot_x, robot_y, max_count);
  if (selection.raw_frontier_cells) {
    unresolved_frontier_cells_ = *selection.raw_frontier_cells;
    raw_frontier_cells_ = unresolved_frontier_cells_;
  }
  return std::move(selection.goals);
}

std::vector<ExplorationGoal> FrontierExplorer::StableGoals(
    std::vector<ExplorationGoal> goals) {
  return tracks_.Filter(std::move(goals), config_.frontier_stability_radius,
                        config_.required_frontier_observations);
}

std::vector<ExplorationGoal> FrontierExplorer::OneGoalPerMouth(
    std::vector<ExplorationGoal> goals, double robot_x, double robot_y) const {
  return frontier_internal::PlaceMouthGoals(
      frontier_internal::MergeMouthGoals(std::move(goals)), maps_.planning(), maps_.observation(),
      config_.footprint_clearance, robot_x, robot_y);
}

std::vector<ExplorationGoal> FrontierExplorer::BuildTour(
    double robot_x, double robot_y, double robot_yaw) {
  // Exploring work is admitted goals only. Residuals wait for the coverage
  // pass. While a room is open, the tour does not include other rooms.
  auto remaining = SelectGoals(robot_x, robot_y);
  RefreshPassages();
  if (ActiveRegion()) {
    std::vector<ExplorationGoal> local;
    for (auto& goal : remaining) {
      if (GoalInActiveRegion(goal)) local.push_back(std::move(goal));
    }
    if (!local.empty()) {
      return regions_.Order(std::move(local), robot_x, robot_y, robot_yaw);
    }
  }
  // An empty candidate list still has to settle the stack. Returning here
  // left the active room and its passage mark in place, so coverage never
  // started and the robot waited on a map update.
  return regions_.Order(std::move(remaining), robot_x, robot_y, robot_yaw);
}

std::vector<ExplorationGoal> FrontierExplorer::OrderByRegion(
    std::vector<ExplorationGoal> goals, double robot_x, double robot_y,
    double robot_yaw) {
  if (goals.empty()) return goals;
  RefreshPassages();
  return regions_.Order(std::move(goals), robot_x, robot_y, robot_yaw);
}

void FrontierExplorer::RefreshPassages() {
  const auto& map = maps_.planning();
  regions_.ClearPassages();
  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.) return;
  for (int row = 0; row < map.height; ++row) {
    for (int col = 0; col < map.width; ++col) {
      if (!RawFrontier(col, row)) continue;
      const auto [x, y] = CellCenter(col, row);
      if (tips_.Blocked(x, y) || PersistentlyBlacklisted(x, y)) continue;
      if (!CorridorMouthAt(map, x, y)) continue;
      const RegionId id = regions_.RegionAt(x, y);
      if (id) regions_.MarkPassage(id);
    }
  }
}

bool FrontierExplorer::ActiveHoldsPassage() const {
  const RegionId active = ActiveRegion();
  if (!active) return false;
  for (const auto& [id, region] : regions_.regions()) {
    if (region.passage && regions_.InActiveContext(id)) return true;
  }
  return false;
}

bool FrontierExplorer::HasExploringWork() const {
  // A mouth flag without an admitted goal is not a task. Treating it as work
  // blocked the coverage pass while the only approachable frontier had
  // already been rejected.
  return candidate_frontier_goals_ > 0;
}

bool FrontierExplorer::ShouldCoverResiduals() const {
  return ::navigation2d::ShouldCoverResiduals(
      HasExploringWork(), config_.cover_residuals, raw_frontier_cells_ > 0);
}

void FrontierExplorer::CommitRegion(const ExplorationGoal& goal) {
  regions_.Enter(goal);
}

void FrontierExplorer::CommitRegion(double x, double y) {
  regions_.Enter(x, y);
}

bool FrontierExplorer::GoalInActiveRegion(const ExplorationGoal& goal) const {
  RegionId id = goal.region_id;
  if (!id) id = regions_.RegionAt(goal.frontier_x, goal.frontier_y);
  if (!id) id = regions_.RegionAt(goal.x, goal.y);
  return regions_.InActiveContext(id);
}

bool FrontierExplorer::ShouldPreemptForNearerMouth(
    double robot_x, double robot_y, const ExplorationGoal& active_goal,
    double path_remaining_m) {
  const auto& map = maps_.planning();
  // Mid-drive preempt is only for a clearly nearer local mouth while still
  // far from the active goal. Loose thresholds rebuilt the tour every scan
  // and re-selected the same far T-label (open-space stutter).
  constexpr double kLocalPocketM = 2.5;
  constexpr double kFarSlackM = 1.5;
  constexpr double kFarRatio = 2.5;
  constexpr double kSameMouthM = .80;
  constexpr double kMinFarRemainingM = 5.0;
  if (path_remaining_m > 0. && path_remaining_m < kMinFarRemainingM) return false;
  if (map.width <= 2 || map.height <= 2 || map.resolution <= 0. ||
      map.cells.size() != static_cast<std::size_t>(map.width * map.height))
    return false;

  const auto graph = frontier_internal::BuildSafeFrontierTravelGraph(
      map, maps_.observation(), config_.footprint_clearance, robot_x, robot_y);
  if (!graph) {
    return false;
  }
  const auto bfs_m = [&](double x, double y) -> double {
    const int cell = frontier_internal::NearestSafeTravelCell(map, graph->safe, x, y);
    if (cell < 0 || graph->travel_cells[cell] < 0) {
      return std::numeric_limits<double>::infinity();
    }
    return graph->travel_cells[cell] * map.resolution;
  };

  double d_far = path_remaining_m;
  if (!(d_far >= 0.)) d_far = bfs_m(active_goal.x, active_goal.y);
  if (!std::isfinite(d_far)) return false;

  const auto mouths = SelectGoals(robot_x, robot_y);
  double d_near = std::numeric_limits<double>::infinity();
  for (const auto& mouth : mouths) {
    if (std::hypot(mouth.x - active_goal.x, mouth.y - active_goal.y) < kSameMouthM ||
        std::hypot(mouth.frontier_x - active_goal.frontier_x,
                   mouth.frontier_y - active_goal.frontier_y) < kSameMouthM)
      continue;
    // Do not abandon an unfinished room for a sibling region's mouth.
    if (!GoalInActiveRegion(mouth)) continue;
    d_near = std::min(d_near, bfs_m(mouth.x, mouth.y));
  }
  if (!std::isfinite(d_near) || d_near > kLocalPocketM) return false;
  return d_far >= kFarRatio * d_near + kFarSlackM;
}

double FrontierExplorer::MouthGapWidth(double frontier_x, double frontier_y) const {
  return frontier_internal::MouthGapWidthAt(maps_.planning(), frontier_x, frontier_y);
}

bool FrontierExplorer::IsCorridorMouth(double frontier_x,
                                       double frontier_y) const {
  const auto& map = maps_.planning();
  return CorridorMouthAt(map, frontier_x, frontier_y);
}

double FrontierExplorer::MouthFreeDepth(const ExplorationGoal& frontier_goal, double robot_x,
                                        double robot_y) const {
  return frontier_internal::MouthFreeDepthIn(maps_.planning(), maps_.observation(),
                                             config_.footprint_clearance, frontier_goal, robot_x,
                                             robot_y);
}

std::optional<ExplorationGoal> FrontierExplorer::SelectEntryProbe(
    const ExplorationGoal& frontier_goal, double robot_x, double robot_y,
    double minimum_depth_m, double maximum_depth_m) const {
  return frontier_internal::SelectEntryProbeAt(
      maps_.planning(), maps_.observation(), config_.footprint_clearance,
      frontier_goal, robot_x, robot_y, minimum_depth_m, maximum_depth_m);
}

bool FrontierExplorer::GoalStillFrontier(const ExplorationGoal& goal) const {
  return frontier_internal::GoalStillFrontierIn(maps_.observation(), goal);
}

bool FrontierExplorer::UnenteredRoomBeyond(const ExplorationGoal& goal) const {
  return frontier_internal::UnenteredRoomBeyondIn(maps_.observation(), goal);
}

bool FrontierExplorer::GoalRegionStillFrontier(const ExplorationGoal& goal) const {
  return frontier_internal::GoalRegionStillFrontierIn(maps_.observation(), goal);
}

void FrontierExplorer::RecordAttempt(const ExplorationGoal& goal, bool succeeded,
                                     bool region_complete, bool cool_frontier) {
  const auto& map = maps_.planning();
  // Failure describes an attempted observation pose, not an entire doorway.
  // A distant standoff must never create a room-sized exclusion disk around
  // its frontier; other safe viewpoints on the same boundary remain useful.
  const std::size_t known = KnownCells();
  const bool door = CorridorMouthAt(map, goal.frontier_x, goal.frontier_y) ||
                    UnenteredRoomBeyond(goal);
  if (succeeded) {
    if (!door && GoalRegionStillFrontier(goal)) tips_.NoteNoNewArea(goal.frontier_x, goal.frontier_y);
    else tips_.NoteProgress(goal.frontier_x, goal.frontier_y);
    visits_.Add({goal.frontier_x, goal.frontier_y,
                          kVisitedFrontierRadius(), known, 20000u, true});
    visits_.Add({goal.x, goal.y, kVisitedViewpointRadius(),
                          known, 20000u, true});
    if (region_complete) ++completed_goals_;
  } else {
    tips_.NoteUnreached(goal.frontier_x, goal.frontier_y, door);
    // A failed route is evidence about the doorway/frontier, not just the
    // exact viewpoint.  Only cooling the pose lets the next map revision
    // regenerate the same three poses around one mouth, which produced the
    // long pure-rotation loops seen on the robot.  Cool the frontier mouth as
    // well; fresh map growth can still make it eligible again.
    const double pose_radius = std::max(map.resolution,
                                        config_.footprint_clearance);
    // Failed bathroom/kitchen mouths were re-queued within a few map updates
    // (600/1500 known-cell cools), so the robot spent minutes retrying the
    // same tip. Cool longer and wider so the tour moves on.
    visits_.Add({goal.x, goal.y, std::max(.55, pose_radius), known,
                          2500u, false});
    if (cool_frontier) {
      visits_.Add(
          {goal.frontier_x, goal.frontier_y, 1.10, known, 4000u, false});
    }
    ++failed_goals_;
  }
  if (tips_.Blocked(goal.frontier_x, goal.frontier_y)) {
    visits_.Add({goal.frontier_x, goal.frontier_y,
                          std::max(config_.blacklist_radius, kSameTipM()), known,
                          20000u, true});
  }
}

void FrontierExplorer::RecordAttempt(const ExplorationGoal& goal, bool succeeded) {
  RecordAttempt(goal, succeeded, true, true);
}

void FrontierExplorer::CoolRegion(double x, double y, double radius_m,
                                  std::size_t required_known_growth) {
  if (!(radius_m > 0.) || !std::isfinite(x) || !std::isfinite(y)) return;
  visits_.Add(
      {x, y, radius_m, KnownCells(), required_known_growth, false});
}

std::size_t FrontierExplorer::KnownCells() const {
  return maps_.observation_view().KnownCells();
}

std::size_t FrontierExplorer::ReachableUnknownCells(double robot_x, double robot_y) const {
  return frontier_internal::FrontierReachability(
      maps_.planning(), maps_.observation(), config_.footprint_clearance)
      .UnknownCells(robot_x, robot_y);
}

std::optional<ExplorationGoal> FrontierExplorer::SettleGoal(const ExplorationGoal& goal) const {
  return frontier_internal::SettleViewpoint(
      maps_.planning(), maps_.observation(), config_.footprint_clearance, goal);
}

std::vector<ExplorationGoal> FrontierExplorer::UnvisitedSpaceGoals(
    const std::vector<std::pair<double, double>>& trail, double visit_radius_m,
    double minimum_area_m2, const std::vector<std::pair<double, double>>& room_seeds,
    double seed_radius_m) const {
  return frontier_internal::UnvisitedSpaceGoalsAt(
      maps_.planning(), maps_.observation(), config_.footprint_clearance,
      trail, visit_radius_m, minimum_area_m2, room_seeds, seed_radius_m);
}

std::size_t FrontierExplorer::RegionFrontierCells(double anchor_x, double anchor_y,
                                                  double radius_m) const {
  const auto& observation = maps_.observation();
  if (observation.width <= 0 || observation.height <= 0 ||
      observation.resolution <= 0. || radius_m <= 0.) return 0;
  std::size_t count = 0;
  for (int row = 0; row < observation.height; ++row) {
    for (int col = 0; col < observation.width; ++col) {
      if (!RawFrontier(col, row)) continue;
      const auto x = observation.origin_x + (col + .5) * observation.resolution;
      const auto y = observation.origin_y + (row + .5) * observation.resolution;
      if (std::hypot(x - anchor_x, y - anchor_y) <= radius_m) ++count;
    }
  }
  return count;
}

bool FrontierExplorer::CompletionEligible(std::size_t initial_known_cells) const {
  const auto growth = std::max(config_.minimum_known_cell_growth,
      static_cast<std::size_t>(std::ceil(initial_known_cells * config_.minimum_known_growth_ratio)));
  return completed_goals_ >= config_.minimum_completed_goals &&
      unresolved_frontier_cells_ == 0 &&
      KnownCells() >= initial_known_cells + growth;
}

}  // namespace navigation2d
