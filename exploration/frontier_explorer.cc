#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/exploration/leftover_viewpoints.h"
#include "navigation2d/exploration/mouth_goal_stages.h"
#include "navigation2d/exploration/unvisited_space.h"
#include "navigation2d/exploration/entry_probe_search.h"
#include "navigation2d/exploration/frontier_travel_graph.h"
#include "navigation2d/exploration/frontier_components.h"
#include "navigation2d/exploration/frontier_viewpoint_search.h"
#include "navigation2d/exploration/frontier_viewpoint_score.h"
#include "navigation2d/exploration/frontier_reachability.h"
#include "navigation2d/exploration/viewpoint_settlement.h"
#include "navigation2d/exploration/doorway_geometry.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <utility>

namespace navigation2d {
using frontier_internal::OccupiedClearanceAt;
using frontier_internal::ViewpointScore;
using frontier_internal::CorridorMouthAt;
using frontier_internal::UnknownRoomBeyond;

FrontierExplorer::FrontierExplorer(FrontierExplorerConfig config)
    : config_(std::move(config)) {}

void FrontierExplorer::UpdateMap(ExplorationGrid map) {
  maps_.Update(std::move(map), [&](const ExplorationGrid& observed) {
    unresolved_frontier_cells_ = maps_.observation_view().CountFrontiers();
    regions_.Update(observed, kPlanningRadius());
  });
}

bool FrontierExplorer::Free(int col, int row) const {
  return maps_.planning_view(config_.footprint_clearance).Free(col, row);
}

bool FrontierExplorer::RawFree(int col, int row) const {
  return maps_.observation_view().Free(col, row);
}

bool FrontierExplorer::RawFrontier(int col, int row) const {
  return maps_.observation_view().Frontier(col, row);
}

std::size_t FrontierExplorer::CountRawFrontierCells() const {
  return maps_.observation_view().CountFrontiers();
}

bool FrontierExplorer::ClearAt(int col, int row, double clearance) const {
  return maps_.planning_view(config_.footprint_clearance).ClearAt(col, row, clearance);
}

bool FrontierExplorer::SafeViewpoint(int col, int row) const {
  return maps_.planning_view(config_.footprint_clearance).SafeViewpoint(col, row);
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

bool FrontierExplorer::Blacklisted(double x, double y, bool boundary) const {
  return visits_.Contains(x, y, boundary, KnownCells());
}
bool FrontierExplorer::PersistentlyBlacklisted(double x, double y) const {
  return visits_.Persistent(x, y);
}
void FrontierExplorer::ClearFailedBlacklist() {
  visits_.ClearFailures();
}

bool FrontierExplorer::HasFreeLineOfSight(
    int from_col, int from_row, int to_col, int to_row) const {
  return maps_.planning_view(config_.footprint_clearance).HasFreeLineOfSight(from_col, from_row, to_col, to_row);
}

void FrontierExplorer::RefreshDebugFrontierCells() {
  const auto& observation = maps_.observation();
  debug_snapshot_.cells.clear();
  if (observation.resolution <= 0. || observation.width <= 2 ||
      observation.height <= 2 ||
      observation.cells.size() !=
          static_cast<std::size_t>(observation.width *
                                   observation.height))
    return;
  const int width = observation.width;
  const int height = observation.height;
  // Color live RawFrontier cells by nearest last-SelectGoals cluster goal so
  // the FR overlay stays tied to CG markers (independent recluster ids made
  // C# labels look unrelated to CG/SG).
  constexpr double kAssociateM = 1.5;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      if (!RawFrontier(col, row)) continue;
      const double x =
          observation.origin_x + (col + .5) * observation.resolution;
      const double y =
          observation.origin_y + (row + .5) * observation.resolution;
      int best_id = 0;
      double best_d = kAssociateM;
      for (const auto& cg : debug_snapshot_.cluster_goals) {
        const double d = std::hypot(cg.frontier_x - x, cg.frontier_y - y);
        if (d < best_d) {
          best_d = d;
          best_id = cg.cluster_id;
        }
      }
      debug_snapshot_.cells.push_back({x, y, best_id});
    }
  }
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
  if (map.width <= 2 || map.height <= 2 || map.resolution <= 0. ||
      map.cells.size() != static_cast<std::size_t>(map.width * map.height)) return {};
  frontier_internal::FrontierComponents components(map.width, map.height);
  const auto graph = frontier_internal::BuildFrontierTravelGraph(
      map, observation, config_.footprint_clearance, robot_x, robot_y);
  if (!graph) {
    return {};
  }
  const auto& safe = graph->safe;
  const auto& travel_cells = graph->travel_cells;
  const auto reachable_frontier_cells = components.Discover(observation, travel_cells);
  unresolved_frontier_cells_ = reachable_frontier_cells;
  raw_frontier_cells_ = reachable_frontier_cells;
  std::vector<ExplorationGoal> goals;
  while (const auto component = components.Next()) {
    const auto& cluster = *component;
    ++raw_frontier_clusters_;
    const int cluster_id = static_cast<int>(raw_frontier_clusters_);
    // Clusters below the minimum stay out of the exploring tour unless a cell
    // is a passable door or still has a room of unknown behind it.
    bool cluster_is_door = false;
    if (cluster.size() < static_cast<std::size_t>(config_.minimum_frontier_cells)) {
      for (const int cell : cluster) {
        const int cell_row = cell / map.width, cell_col = cell % map.width;
        const auto [cx, cy] = CellCenter(cell_col, cell_row);
        if (CorridorMouthAt(map, cx, cy) ||
            UnknownRoomBeyond(map, cell_col, cell_row)) {
          cluster_is_door = true;
          break;
        }
      }
      if (!cluster_is_door) continue;
    }
    const auto anchor = frontier_internal::LocateCluster(cluster, map);
    const double frontier_x = anchor.frontier_x, frontier_y = anchor.frontier_y;
    if (Blacklisted(frontier_x, frontier_y, true)) continue;
    for (const int cell : cluster) {
      const auto [cx, cy] = CellCenter(cell % map.width, cell / map.width);
      debug_snapshot_.cells.push_back({cx, cy, cluster_id});
    }
    const auto best = frontier_internal::SelectComponentViewpoint(
        {map, observation, config_, visits_, safe, travel_cells, cluster, anchor});
    if (best) {
      ExploringGoalFacts facts;
      facts.frontier_cells = best->frontier_cells;
      facts.information_gain = best->information_gain;
      facts.cell_area = map.resolution * map.resolution;
      facts.passable_mouth =
          CorridorMouthAt(map, best->frontier_x, best->frontier_y);
      const int frontier_col = static_cast<int>(std::floor(
          (best->frontier_x - map.origin_x) / map.resolution));
      const int frontier_row = static_cast<int>(std::floor(
          (best->frontier_y - map.origin_y) / map.resolution));
      facts.unentered_room =
          UnknownRoomBeyond(map, frontier_col, frontier_row) ||
          UnenteredRoomBeyond(*best);
      if (!AdmitExploringGoal(facts, config_.minimum_frontier_cells) ||
          tips_.Blocked(best->frontier_x, best->frontier_y)) {
        continue;
      }
      goals.push_back(*best);
      debug_snapshot_.cluster_goals.push_back(
          {best->x, best->y, best->frontier_x, best->frontier_y, cluster_id,
           best->score, best->information_gain, best->frontier_cells});
    }
  }
  std::sort(goals.begin(), goals.end(),
            [](const auto& a, const auto& b) { return a.score > b.score; });
  // Stage A/B: mouth is the task unit — one goal per doorway/alcove, not per
  // frontier micro-cluster. Entry probe moves the pose into the room when the
  // planning graph already opened that corridor.
  auto mouths = OneGoalPerMouth(std::move(goals), robot_x, robot_y);
  candidate_frontier_goals_ = mouths.size();
  auto stable = StableGoals(std::move(mouths));
  executable_frontier_goals_ = stable.size();
  const auto nearest_cluster_id = [&](double fx, double fy) {
    int best_id = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (const auto& cg : debug_snapshot_.cluster_goals) {
      const double d = std::hypot(cg.frontier_x - fx, cg.frontier_y - fy);
      if (d < best_d) {
        best_d = d;
        best_id = cg.cluster_id;
      }
    }
    return best_id;
  };
  for (const auto& goal : stable) {
    debug_snapshot_.goals.push_back(
        {goal.x, goal.y, goal.frontier_x, goal.frontier_y,
         nearest_cluster_id(goal.frontier_x, goal.frontier_y), goal.score,
         goal.information_gain, goal.frontier_cells});
  }
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

  std::vector<unsigned char> safe(map.cells.size(), 0);
  for (int row = 0; row < map.height; ++row) {
    for (int col = 0; col < map.width; ++col) {
      safe[static_cast<std::size_t>(row) * map.width + col] = SafeViewpoint(col, row);
    }
  }
  const auto nearest_safe = [&](double x, double y) -> int {
    const int center_col = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
    const int center_row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
    int best = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    const int radius = static_cast<int>(std::ceil(1.0 / map.resolution));
    for (int row = std::max(0, center_row - radius);
         row <= std::min(map.height - 1, center_row + radius); ++row) {
      for (int col = std::max(0, center_col - radius);
           col <= std::min(map.width - 1, center_col + radius); ++col) {
        const int index = row * map.width + col;
        if (!safe[index]) continue;
        const double distance = std::hypot(col - center_col, row - center_row);
        if (distance < best_distance) { best_distance = distance; best = index; }
      }
    }
    return best;
  };
  const int start = nearest_safe(robot_x, robot_y);
  if (start < 0) return false;
  std::vector<int> distance(map.cells.size(), -1);
  std::deque<int> queue{start};
  distance[start] = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    const int row = current / map.width, col = current % map.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int x = col + dx[direction], y = row + dy[direction];
      if (x < 0 || y < 0 || x >= map.width || y >= map.height) continue;
      const int next = y * map.width + x;
      if (!safe[next] || distance[next] >= 0) continue;
      distance[next] = distance[current] + 1;
      queue.push_back(next);
    }
  }
  const auto bfs_m = [&](double x, double y) -> double {
    const int cell = nearest_safe(x, y);
    if (cell < 0 || distance[cell] < 0) return std::numeric_limits<double>::infinity();
    return distance[cell] * map.resolution;
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

double FrontierExplorer::MouthFreeDepth(const ExplorationGoal& frontier_goal,
                                        double robot_x, double robot_y) const {
  const auto& map = maps_.planning();
  if (map.resolution <= 0. || map.width <= 2 || map.height <= 2) return 0.;
  double dir_x = frontier_goal.frontier_x - robot_x;
  double dir_y = frontier_goal.frontier_y - robot_y;
  double len = std::hypot(dir_x, dir_y);
  if (len < .05) {
    dir_x = frontier_goal.frontier_x - frontier_goal.x;
    dir_y = frontier_goal.frontier_y - frontier_goal.y;
    len = std::hypot(dir_x, dir_y);
  }
  if (len < .05) return 0.;
  dir_x /= len;
  dir_y /= len;
  double depth = 0.;
  for (double step = map.resolution; step <= 1.40 + 1e-6;
       step += map.resolution) {
    const double x = frontier_goal.frontier_x + step * dir_x;
    const double y = frontier_goal.frontier_y + step * dir_y;
    const int col =
        static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
    const int row =
        static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
    if (col < 0 || row < 0 || col >= map.width || row >= map.height) break;
    if (!Free(col, row) || !ClearAt(col, row, .14)) break;
    depth = step;
  }
  return depth;
}

std::optional<ExplorationGoal> FrontierExplorer::SelectEntryProbe(
    const ExplorationGoal& frontier_goal, double robot_x, double robot_y,
    double minimum_depth_m, double maximum_depth_m) const {
  return frontier_internal::SelectEntryProbeAt(
      maps_.planning(), maps_.observation(), config_.footprint_clearance,
      frontier_goal, robot_x, robot_y, minimum_depth_m, maximum_depth_m);
}

bool FrontierExplorer::GoalStillFrontier(const ExplorationGoal& goal) const {
  const auto& observation = maps_.observation();
  if (observation.resolution <= 0. || observation.width <= 0 ||
      observation.height <= 0) return false;
  const int col = static_cast<int>(std::floor(
      (goal.frontier_x - observation.origin_x) / observation.resolution));
  const int row = static_cast<int>(std::floor(
      (goal.frontier_y - observation.origin_y) / observation.resolution));
  return RawFrontier(col, row);
}

bool FrontierExplorer::UnenteredRoomBeyond(const ExplorationGoal& goal) const {
  const auto& observation = maps_.observation();
  const ExplorationGrid& map = observation;
  if (map.resolution <= 0. || map.width <= 2 || map.height <= 2 ||
      map.cells.size() != static_cast<std::size_t>(map.width) * map.height)
    return false;
  const double dx = goal.frontier_x - goal.x;
  const double dy = goal.frontier_y - goal.y;
  const double length = std::hypot(dx, dy);
  if (length < map.resolution) return false;
  const double ux = dx / length;
  const double uy = dy / length;
  // Same distances already used to tell a room from a fringe: CorridorMouthAt's
  // 1.4 m free-run probe, and UnknownRoomBeyond's 0.55 m unknown depth.
  const int steps = static_cast<int>(std::ceil(1.4 / map.resolution));
  const int need =
      std::max(1, static_cast<int>(std::ceil(.55 / map.resolution)));
  auto at = [&](int col, int row) -> std::int8_t {
    if (col < 0 || row < 0 || col >= map.width || row >= map.height) return 100;
    return map.cells[static_cast<std::size_t>(row) * map.width + col];
  };
  bool have_free = false;
  double free_x = 0.;
  double free_y = 0.;
  int unknown_run = 0;
  for (int step = -1; step <= steps; ++step) {
    const double wx = goal.frontier_x + step * map.resolution * ux;
    const double wy = goal.frontier_y + step * map.resolution * uy;
    const int col =
        static_cast<int>(std::floor((wx - map.origin_x) / map.resolution));
    const int row =
        static_cast<int>(std::floor((wy - map.origin_y) / map.resolution));
    const auto cell = at(col, row);
    if (cell > 0) {
      // A jamb beside the standoff is not the room. Occupied on the
      // into-room ray means this axis is a wall, not an entrance.
      if (step < 0) continue;
      return false;
    }
    if (cell == 0) {
      have_free = true;
      free_x = map.origin_x + (col + .5) * map.resolution;
      free_y = map.origin_y + (row + .5) * map.resolution;
      unknown_run = 0;
      continue;
    }
    ++unknown_run;
    if (have_free && unknown_run >= need &&
        CorridorMouthAt(map, free_x, free_y))
      return true;
  }
  return false;
}

bool FrontierExplorer::GoalRegionStillFrontier(const ExplorationGoal& goal) const {
  const auto& observation = maps_.observation();
  if (GoalStillFrontier(goal) || observation.resolution <= 0.)
    return GoalStillFrontier(goal);
  const int center_col = static_cast<int>(std::floor(
      (goal.frontier_x - observation.origin_x) / observation.resolution));
  const int center_row = static_cast<int>(std::floor(
      (goal.frontier_y - observation.origin_y) / observation.resolution));
  // The association radius is deliberately smaller than the viewpoint
  // standoff. It follows a shifted sensor boundary but cannot jump across a
  // shelf to an unrelated frontier on its far side.
  const int radius = static_cast<int>(std::ceil(.35 / observation.resolution));
  for (int row = std::max(0, center_row - radius);
       row <= std::min(observation.height - 1, center_row + radius); ++row) {
    for (int col = std::max(0, center_col - radius);
         col <= std::min(observation.width - 1, center_col + radius); ++col) {
      if (std::hypot(col - center_col, row - center_row) > radius) continue;
      if (RawFrontier(col, row)) return true;
    }
  }
  return false;
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
