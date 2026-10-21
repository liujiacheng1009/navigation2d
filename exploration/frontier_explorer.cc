#include "navigation2d/exploration/frontier_explorer.h"

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

namespace {

// Ranks observation poses inside one frontier cluster (which standoff cell to
// stand on). Tour order between rooms is decided separately by BuildTour with
// nearest-first coverage — do not use this score to jump across the map.
// Prefer standing near the preferred standoff: the old −2·standoff term rewarded
// wall-hugging and sent the robot into side-clearance thrash mid-room.
// Clearance pulls doorway/standoff goals off the jamb toward the free center
// (UI: GOAL glued to the left threshold wall → stuck at the mouth).
double ViewpointScore(double information_gain, double travel, double standoff,
                      double gain_weight, double clearance_m = .20) {
  return gain_weight * information_gain / (0.45 + travel) -
         2.5 * std::abs(standoff - navigation2d::kPreferredStandoff()) +
         4.0 * std::min(clearance_m, .40);
}

}  // namespace

namespace navigation2d {

namespace {

// Distance from cell center to the nearest occupied cell (planning map).
double OccupiedClearanceAt(const ExplorationGrid& map, int col, int row) {
  if (map.resolution <= 0. || col < 0 || row < 0 || col >= map.width ||
      row >= map.height)
    return 0.;
  if (map.cells[static_cast<std::size_t>(row) * map.width + col] != 0) return 0.;
  const double wx = map.origin_x + (col + .5) * map.resolution;
  const double wy = map.origin_y + (row + .5) * map.resolution;
  const double half = .5 * map.resolution;
  const int reach = static_cast<int>(std::ceil(.55 / map.resolution));
  double best = .55;
  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      const int x = col + dx, y = row + dy;
      if (x < 0 || y < 0 || x >= map.width || y >= map.height) {
        best = std::min(best, std::hypot(static_cast<double>(dx),
                                         static_cast<double>(dy)) *
                                  map.resolution);
        continue;
      }
      if (map.cells[static_cast<std::size_t>(y) * map.width + x] <= 0) continue;
      const double ox = map.origin_x + (x + .5) * map.resolution;
      const double oy = map.origin_y + (y + .5) * map.resolution;
      const double nearest_x = std::max(std::abs(wx - ox) - half, 0.);
      const double nearest_y = std::max(std::abs(wy - oy) - half, 0.);
      best = std::min(best, std::hypot(nearest_x, nearest_y));
    }
  }
  return best;
}

// True when the frontier sits in a door/alcove throat (narrow free run), not
// an open-boundary wall expansion. Used to prefer unfinished rooms over
// nearby hallway frontier crawling.
// True when unknown extends ≥0.55 m past a free↔unknown frontier — a room
// pocket — rather than a 1-cell open-wall fringe. Wide doorways that hit the
// free-run probe cap still count as mouths when a room lies beyond.
bool UnknownRoomBeyond(const ExplorationGrid& map, int col, int row) {
  if (map.resolution <= 0.) return false;
  const int need =
      std::max(1, static_cast<int>(std::ceil(.55 / map.resolution)));
  auto value = [&](int c, int r) -> std::int8_t {
    if (c < 0 || r < 0 || c >= map.width || r >= map.height) return 100;
    return map.cells[static_cast<std::size_t>(r) * map.width + c];
  };
  int max_depth = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  for (int direction = 0; direction < 4; ++direction) {
    const int step_c = dx[direction];
    const int step_r = dy[direction];
    if (value(col + step_c, row + step_r) >= 0) continue;
    int depth = 0;
    for (int step = 1; step <= need + 2; ++step) {
      if (value(col + step * step_c, row + step * step_r) >= 0) break;
      depth = step;
    }
    max_depth = std::max(max_depth, depth);
  }
  return max_depth >= need;
}

bool CorridorMouthAt(const ExplorationGrid& map, double frontier_x,
                     double frontier_y) {
  if (map.resolution <= 0. || map.width <= 0 || map.height <= 0) return false;
  const int col = static_cast<int>(
      std::floor((frontier_x - map.origin_x) / map.resolution));
  const int row = static_cast<int>(
      std::floor((frontier_y - map.origin_y) / map.resolution));
  if (col < 0 || row < 0 || col >= map.width || row >= map.height) return false;
  auto free = [&](int c, int r) {
    if (c < 0 || r < 0 || c >= map.width || r >= map.height) return false;
    return map.cells[static_cast<std::size_t>(r) * map.width + c] == 0;
  };
  const int probe = static_cast<int>(std::ceil(1.4 / map.resolution));
  int horizontal = 0, vertical = 0;
  for (int x = col; x >= 0 && x > col - probe && free(x, row); --x) ++horizontal;
  for (int x = col + 1; x < map.width && x <= col + probe && free(x, row); ++x)
    ++horizontal;
  for (int y = row; y >= 0 && y > row - probe && free(col, y); --y) ++vertical;
  for (int y = row + 1; y < map.height && y <= row + probe && free(col, y); ++y)
    ++vertical;
  const int narrow_min = static_cast<int>(
      std::ceil(navigation2d::kNarrowMinWidth() / map.resolution));
  const int narrow_max = static_cast<int>(
      std::floor(navigation2d::kNarrowMaxWidth() / map.resolution));
  // Open-room free space hits the probe cap on BOTH axes — that is a wall
  // expansion, not a door throat (live: gap=1.65 "easy door" on open wall).
  // Exception: a still-unknown room behind a wide free gap also caps both
  // axes after the throat is flooded; keep those as mouths so entry is not
  // abandoned mid-approach (live: frontier≈(-2.20,2.49) → candidates=0).
  if (horizontal >= narrow_max && vertical >= narrow_max)
    return UnknownRoomBeyond(map, col, row);
  const bool narrow_h = horizontal >= narrow_min && horizontal <= narrow_max;
  const bool narrow_v = vertical >= narrow_min && vertical <= narrow_max;
  const int smaller = std::min(horizontal, vertical);
  return narrow_h || narrow_v ||
         (smaller >= narrow_min && smaller <= narrow_max);
}



}  // namespace

bool PreferTipAbandonAfterPathCut(int mapped_obstacle_rejects,
                                  double path_min_clearance_m) {
  (void)path_min_clearance_m;
  // Only leave after repeated live-occupied cuts. A single low-clearance
  // chord near a door jamb must replan, not abandon the room.
  return mapped_obstacle_rejects >= 2;
}

bool PathCutsLiveOccupied(const ExplorationGrid& live,
                          const std::vector<std::pair<double, double>>& path_xy,
                          double robot_x, double robot_y, double body_radius,
                          double under_robot_m) {
  if (live.resolution <= 0. || live.width <= 2 || live.height <= 2 ||
      live.cells.size() != static_cast<std::size_t>(live.width) * live.height ||
      path_xy.size() < 2 || body_radius <= 0.)
    return false;
  const double resolution = live.resolution;
  const int width = live.width;
  const int height = live.height;
  const auto lethal = [&](int col, int row) {
    if (col < 0 || row < 0 || col >= width || row >= height) return true;
    return live.cells[static_cast<std::size_t>(row) * width + col] > 0;
  };
  const auto hits_body = [&](double world_x, double world_y) {
    if (std::hypot(world_x - robot_x, world_y - robot_y) < under_robot_m)
      return false;
    const int col =
        static_cast<int>(std::floor((world_x - live.origin_x) / resolution));
    const int row =
        static_cast<int>(std::floor((world_y - live.origin_y) / resolution));
    const int reach = static_cast<int>(
        std::ceil((body_radius + std::sqrt(.5) * resolution) / resolution));
    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        const int x = col + dx, y = row + dy;
        if (!lethal(x, y)) continue;
        const double center_x = live.origin_x + (x + .5) * resolution;
        const double center_y = live.origin_y + (y + .5) * resolution;
        const double half = .5 * resolution;
        const double nearest_x =
            std::max(std::abs(world_x - center_x) - half, 0.);
        const double nearest_y =
            std::max(std::abs(world_y - center_y) - half, 0.);
        if (std::hypot(nearest_x, nearest_y) <= body_radius) return true;
      }
    }
    return false;
  };
  const auto hits_centerline = [&](double world_x, double world_y) {
    if (std::hypot(world_x - robot_x, world_y - robot_y) < under_robot_m)
      return false;
    const int col =
        static_cast<int>(std::floor((world_x - live.origin_x) / resolution));
    const int row =
        static_cast<int>(std::floor((world_y - live.origin_y) / resolution));
    if (col < 0 || row < 0 || col >= width || row >= height) return true;
    return lethal(col, row);
  };
  for (std::size_t index = 0; index < path_xy.size(); ++index) {
    const double x = path_xy[index].first;
    const double y = path_xy[index].second;
    if (hits_body(x, y) || hits_centerline(x, y)) return true;
    if (index == 0) continue;
    const double x0 = path_xy[index - 1].first;
    const double y0 = path_xy[index - 1].second;
    const double length = std::hypot(x - x0, y - y0);
    const int samples = std::max(
        2, static_cast<int>(
               std::ceil(length / std::max(.01, .35 * resolution))));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const double wx = x0 + ratio * (x - x0);
      const double wy = y0 + ratio * (y - y0);
      if (hits_body(wx, wy) || hits_centerline(wx, wy)) return true;
    }
  }
  return false;
}

FrontierExplorer::FrontierExplorer(FrontierExplorerConfig config)
    : config_(std::move(config)) {}

void FrontierExplorer::UpdateMap(ExplorationGrid map) {
  observation_map_ = map;
  unresolved_frontier_cells_ = CountRawFrontierCells();
  // Partition observed free space into topological regions. Door necks stay
  // as edges between rooms so DFS can finish one room first. Planning uses
  // the same occupancy as the mapper — SLAM occupied is never carved open.
  regions_.Update(observation_map_, kPlanningRadius());
  map_ = std::move(map);
}

bool FrontierExplorer::Free(int col, int row) const {
  return col >= 0 && row >= 0 && col < map_.width && row < map_.height &&
      map_.cells[static_cast<std::size_t>(row) * map_.width + col] == 0;
}

bool FrontierExplorer::RawFree(int col, int row) const {
  return col >= 0 && row >= 0 && col < observation_map_.width &&
      row < observation_map_.height &&
      observation_map_.cells[static_cast<std::size_t>(row) * observation_map_.width + col] == 0;
}

bool FrontierExplorer::RawFrontier(int col, int row) const {
  if (!RawFree(col, row)) return false;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  for (int direction = 0; direction < 4; ++direction) {
    const int x = col + dx[direction], y = row + dy[direction];
    if (x >= 0 && y >= 0 && x < observation_map_.width && y < observation_map_.height &&
        observation_map_.cells[static_cast<std::size_t>(y) * observation_map_.width + x] < 0)
      return true;
  }
  return false;
}

std::size_t FrontierExplorer::CountRawFrontierCells() const {
  if (observation_map_.width <= 0 || observation_map_.height <= 0 ||
      observation_map_.cells.size() != static_cast<std::size_t>(
          observation_map_.width * observation_map_.height)) return 0;
  std::size_t count = 0;
  for (int row = 0; row < observation_map_.height; ++row)
    for (int col = 0; col < observation_map_.width; ++col)
      if (RawFrontier(col, row)) ++count;
  return count;
}

bool FrontierExplorer::ClearAt(int col, int row, double clearance) const {
  // Match LayeredCostmap::lethal: continuous distance to occupied cells with
  // a half-cell box margin. The old discrete disk accepted poses the planner
  // then rejected (and vice versa) because GridSearch uses this geometry.
  if (col < 0 || row < 0 || col >= map_.width || row >= map_.height) return false;
  if (map_.cells[static_cast<std::size_t>(row) * map_.width + col] != 0) return false;
  const auto [wx, wy] = CellCenter(col, row);
  const double half = .5 * map_.resolution;
  const int cells = static_cast<int>(std::ceil(
      (std::max(0., clearance) + std::sqrt(.5) * map_.resolution) / map_.resolution));
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      const int x = col + dx, y = row + dy;
      if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) return false;
      // Unknown is unobserved space, not a jamb — same as costmap lethal().
      if (map_.cells[static_cast<std::size_t>(y) * map_.width + x] <= 0) continue;
      const auto [ox, oy] = CellCenter(x, y);
      const double nearest_x = std::max(std::abs(wx - ox) - half, 0.);
      const double nearest_y = std::max(std::abs(wy - oy) - half, 0.);
      if (std::hypot(nearest_x, nearest_y) <= clearance) return false;
    }
  }
  return true;
}

bool FrontierExplorer::SafeViewpoint(int col, int row) const {
  // The search graph stays on known free cells. Unknown may lie inside the
  // clearance disk, but the viewpoint itself is not placed in unknown space.
  if (!Free(col, row)) return false;
  // Tour goals must sit on mapper-observed free (planning map == observation).
  if (!RawFree(col, row)) return false;
  if (ClearAt(col, row, config_.footprint_clearance)) return true;
  // Narrow-passage candidate: permit only the physical footprint radius when
  // the free run is corridor-shaped and its measured width is in the range
  // where a robot can fit. Open areas retain the normal clearance contract.
  const int probe = static_cast<int>(std::ceil(1.4 / map_.resolution));
  int horizontal = 0, vertical = 0;
  for (int x = col; x >= 0 && x > col - probe && Free(x, row); --x) ++horizontal;
  for (int x = col + 1; x < map_.width && x <= col + probe && Free(x, row); ++x) ++horizontal;
  for (int y = row; y >= 0 && y > row - probe && Free(col, y); --y) ++vertical;
  for (int y = row + 1; y < map_.height && y <= row + probe && Free(col, y); ++y) ++vertical;
  const int narrow_min = static_cast<int>(std::ceil(kNarrowMinWidth() / map_.resolution));
  const int narrow_max = static_cast<int>(std::floor(kNarrowMaxWidth() / map_.resolution));
  const bool narrow_h = horizontal >= narrow_min && horizontal <= narrow_max;
  const bool narrow_v = vertical >= narrow_min && vertical <= narrow_max;
  // A kitchen mouth often has BOTH axes in the door band (short vestibule).
  // The old XOR treated that as open space and demanded footprint_clearance
  // (.30), which sealed 0.50–0.60 m free throats after SLAM thickened jambs.
  const int smaller = std::min(horizontal, vertical);
  const bool corridor = narrow_h || narrow_v ||
                        (smaller >= narrow_min && smaller <= narrow_max);
  // Wide open space keeps footprint_clearance (typically .30 with inflation).
  // Corridor / doorway cells must use kPlanningRadius(): requiring
  // .30 m half-width rejects a physical 0.70 m kitchen door after SLAM
  // thickens the jambs, so entry probes never reach the room side.
  const double corridor_clearance =
      std::max(kNarrowClearance(), std::min(config_.footprint_clearance, kPlanningRadius()));
  return corridor && ClearAt(col, row, corridor_clearance);
}

std::pair<double, double> FrontierExplorer::CellCenter(int col, int row) const {
  return {map_.origin_x + (col + .5) * map_.resolution,
          map_.origin_y + (row + .5) * map_.resolution};
}

bool FrontierExplorer::InSameReachableRegion(double anchor_x, double anchor_y,
                                             double target_x, double target_y,
                                             double max_distance_m) const {
  if (map_.width <= 0 || map_.height <= 0 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height) ||
      max_distance_m < 0.) return false;
  const auto nearest_safe = [&](double x, double y) {
    const int center_col = static_cast<int>(std::floor((x - map_.origin_x) / map_.resolution));
    const int center_row = static_cast<int>(std::floor((y - map_.origin_y) / map_.resolution));
    const int search = static_cast<int>(std::ceil(1.0 / map_.resolution));
    int best = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int row = std::max(0, center_row - search);
         row <= std::min(map_.height - 1, center_row + search); ++row) {
      for (int col = std::max(0, center_col - search);
           col <= std::min(map_.width - 1, center_col + search); ++col) {
        if (!SafeViewpoint(col, row)) continue;
        const double distance = std::hypot(col - center_col, row - center_row);
        if (distance < best_distance) {
          best_distance = distance;
          best = row * map_.width + col;
        }
      }
    }
    return best;
  };
  const int start = nearest_safe(anchor_x, anchor_y);
  const int target = nearest_safe(target_x, target_y);
  if (start < 0 || target < 0) return false;
  const int max_cells = static_cast<int>(std::ceil(max_distance_m / map_.resolution));
  std::vector<int> distance(map_.cells.size(), -1);
  std::deque<int> queue{start};
  distance[start] = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    if (current == target) return distance[current] <= max_cells;
    if (distance[current] >= max_cells) continue;
    const int row = current / map_.width, col = current % map_.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + dx[direction], next_row = row + dy[direction];
      if (next_col < 0 || next_row < 0 || next_col >= map_.width || next_row >= map_.height)
        continue;
      const int next = next_row * map_.width + next_col;
      if (distance[next] >= 0 || !SafeViewpoint(next_col, next_row)) continue;
      distance[next] = distance[current] + 1;
      queue.push_back(next);
    }
  }
  return false;
}

bool FrontierExplorer::Blacklisted(double x, double y, bool boundary) const {
  const std::size_t known = KnownCells();
  return std::any_of(blacklist_.begin(), blacklist_.end(), [&](const auto& entry) {
    // Eligible again after the map has materially expanded (or forever when
    // persistent). Temporary mouth/pocket cools must also apply to frontier
    // queries: the old `boundary ⇒ persistent-only` gate ignored every failed
    // doorway cool, so nearest-first kept re-queuing the same alcove crumbs.
    if (!(entry.persistent || known < entry.known_cells + entry.required_growth))
      return false;
    // Tiny pose-only cools stay viewpoint-local; mouth/pocket radii (>=.40)
    // suppress the frontier component as well.
    if (boundary && !entry.persistent && entry.radius < .40) return false;
    return std::hypot(x - entry.x, y - entry.y) < entry.radius;
  });
}

bool FrontierExplorer::PersistentlyBlacklisted(double x, double y) const {
  return std::any_of(blacklist_.begin(), blacklist_.end(), [&](const auto& entry) {
    return entry.persistent && std::hypot(x - entry.x, y - entry.y) < entry.radius;
  });
}

void FrontierExplorer::ClearFailedBlacklist() {
  blacklist_.erase(std::remove_if(blacklist_.begin(), blacklist_.end(),
                                  [](const BlacklistEntry& entry) {
                                    return !entry.persistent;
                                  }),
                   blacklist_.end());
}

bool FrontierExplorer::HasFreeLineOfSight(
    int from_col, int from_row, int to_col, int to_row) const {
  // A standoff pose is useful only when its lidar can actually observe the
  // free/unknown boundary that created the task.  Euclidean-only selection
  // used to choose poses across the corner of a shelf: reachable by a long
  // detour, but unable to reveal that frontier once reached.
  int x = from_col, y = from_row;
  const int dx = std::abs(to_col - from_col), sx = from_col < to_col ? 1 : -1;
  const int dy = -std::abs(to_row - from_row), sy = from_row < to_row ? 1 : -1;
  int error = dx + dy;
  while (true) {
    if (!Free(x, y)) return false;
    if (x == to_col && y == to_row) return true;
    const int twice_error = 2 * error;
    if (twice_error >= dy) { error += dy; x += sx; }
    if (twice_error <= dx) { error += dx; y += sy; }
    if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) return false;
  }
}

void FrontierExplorer::RefreshDebugFrontierCells() {
  debug_snapshot_.cells.clear();
  if (observation_map_.resolution <= 0. || observation_map_.width <= 2 ||
      observation_map_.height <= 2 ||
      observation_map_.cells.size() !=
          static_cast<std::size_t>(observation_map_.width *
                                   observation_map_.height))
    return;
  const int width = observation_map_.width;
  const int height = observation_map_.height;
  // Color live RawFrontier cells by nearest last-SelectGoals cluster goal so
  // the FR overlay stays tied to CG markers (independent recluster ids made
  // C# labels look unrelated to CG/SG).
  constexpr double kAssociateM = 1.5;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      if (!RawFrontier(col, row)) continue;
      const double x =
          observation_map_.origin_x + (col + .5) * observation_map_.resolution;
      const double y =
          observation_map_.origin_y + (row + .5) * observation_map_.resolution;
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
  raw_frontier_cells_ = 0;
  raw_frontier_clusters_ = 0;
  executable_frontier_goals_ = 0;
  candidate_frontier_goals_ = 0;
  debug_snapshot_ = {};
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height)) return {};
  std::vector<unsigned char> frontier(map_.cells.size(), 0), visited(map_.cells.size(), 0);
  std::vector<unsigned char> safe(map_.cells.size(), 0);
  std::vector<unsigned char> discovery_safe(map_.cells.size(), 0);
  for (int row = 0; row < map_.height; ++row) for (int col = 0; col < map_.width; ++col)
    safe[static_cast<std::size_t>(row) * map_.width + col] = SafeViewpoint(col, row);
  // Discovery graph only finds room boundaries. Executable viewpoints still
  // require SafeViewpoint (== planning radius); nominating discovery-only
  // poses produced candidates the planner rejected as goal-occupied.
  const double discovery_clearance = std::clamp(
      config_.footprint_clearance - .12, .14, kPlanningRadius());
  for (int row = 0; row < map_.height; ++row) for (int col = 0; col < map_.width; ++col) {
    const std::size_t index = static_cast<std::size_t>(row) * map_.width + col;
    discovery_safe[index] = Free(col, row) && ClearAt(col, row, discovery_clearance);
  }
  const int robot_col = static_cast<int>(std::floor((robot_x - map_.origin_x) / map_.resolution));
  const int robot_row = static_cast<int>(std::floor((robot_y - map_.origin_y) / map_.resolution));
  int start = -1;
  double start_distance = std::numeric_limits<double>::infinity();
  const int start_search = static_cast<int>(std::ceil(1.0 / map_.resolution));
  for (int row = std::max(0, robot_row - start_search);
       row <= std::min(map_.height - 1, robot_row + start_search); ++row)
  for (int col = std::max(0, robot_col - start_search);
       col <= std::min(map_.width - 1, robot_col + start_search); ++col) {
    const int index = row * map_.width + col;
    if (!safe[index]) continue;
    const double distance = std::hypot(col - robot_col, row - robot_row);
    if (distance < start_distance) { start_distance = distance; start = index; }
  }
  if (start < 0) return {};
  std::vector<int> travel_cells(map_.cells.size(), -1);
  std::deque<int> reachable{start};
  travel_cells[start] = 0;
  constexpr int cardinal_x[] = {-1, 1, 0, 0};
  constexpr int cardinal_y[] = {0, 0, -1, 1};
  while (!reachable.empty()) {
    const int current = reachable.front(); reachable.pop_front();
    const int row = current / map_.width, col = current % map_.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int x = col + cardinal_x[direction], y = row + cardinal_y[direction];
      if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) continue;
      const int next = y * map_.width + x;
      if (!safe[next] || travel_cells[next] >= 0) continue;
      travel_cells[next] = travel_cells[current] + 1;
      reachable.push_back(next);
    }
  }
  // Frontier discovery and goal execution use different contracts. The
  // execution graph requires complete footprint clearance, while a physical
  // doorway may be narrower than that graph and still be passable. Build a
  // relaxed discovery graph for finding room boundaries only; StartNavigation
  // continues to validate every dispatched goal against the real inflated map.
  std::vector<int> discovery_travel_cells(map_.cells.size(), -1);
  int discovery_start = start;
  if (!discovery_safe[static_cast<std::size_t>(discovery_start)]) {
    double nearest_discovery = std::numeric_limits<double>::infinity();
    for (int row = std::max(0, robot_row - start_search);
         row <= std::min(map_.height - 1, robot_row + start_search); ++row) {
      for (int col = std::max(0, robot_col - start_search);
           col <= std::min(map_.width - 1, robot_col + start_search); ++col) {
        const int index = row * map_.width + col;
        if (!discovery_safe[static_cast<std::size_t>(index)]) continue;
        const double distance = std::hypot(col - robot_col, row - robot_row);
        if (distance < nearest_discovery) {
          nearest_discovery = distance;
          discovery_start = index;
        }
      }
    }
  }
  if (discovery_start >= 0 &&
      discovery_safe[static_cast<std::size_t>(discovery_start)]) {
    std::deque<int> discovery_queue{discovery_start};
    discovery_travel_cells[static_cast<std::size_t>(discovery_start)] = 0;
    while (!discovery_queue.empty()) {
      const int current = discovery_queue.front();
      discovery_queue.pop_front();
      const int row = current / map_.width;
      const int col = current % map_.width;
      for (int direction = 0; direction < 4; ++direction) {
        const int x = col + cardinal_x[direction], y = row + cardinal_y[direction];
        if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) continue;
        const int next = y * map_.width + x;
        if (!discovery_safe[static_cast<std::size_t>(next)] ||
            discovery_travel_cells[static_cast<std::size_t>(next)] >= 0) continue;
        discovery_travel_cells[static_cast<std::size_t>(next)] =
            discovery_travel_cells[static_cast<std::size_t>(current)] + 1;
        discovery_queue.push_back(next);
      }
    }
  }
  // Retain exact safe-graph distances where available. Relaxed-only cells are
  // discovery hints and will be rejected individually if the live planner
  // cannot execute their path; they must not suppress the whole room.
  for (std::size_t index = 0; index < travel_cells.size(); ++index) {
    if (travel_cells[index] < 0) travel_cells[index] = discovery_travel_cells[index];
  }
  // During the first scan, costmap inflation can temporarily mark every
  // viewpoint around the robot unsafe even though the raw map already has a
  // connected free corridor and visible unknown boundary.  Without a first
  // goal the robot cannot move far enough for the mapper to resolve that
  // boundary.  Use the raw free-cell graph only for this small bootstrap
  // window; mature maps continue to require the full footprint-safe graph.
  const std::size_t raw_frontier_count = CountRawFrontierCells();
  const bool bootstrap_map = KnownCells() < 10000 && raw_frontier_count > 0;
  if (bootstrap_map) {
    bool reachable_raw_frontier = false;
    for (int index = 0; index < static_cast<int>(travel_cells.size()); ++index) {
      if (travel_cells[index] < 0) continue;
      const int row = index / map_.width, col = index % map_.width;
      if (RawFrontier(col, row)) { reachable_raw_frontier = true; break; }
    }
    if (!reachable_raw_frontier) {
      std::fill(travel_cells.begin(), travel_cells.end(), -1);
      int raw_start = -1;
      double raw_start_distance = std::numeric_limits<double>::infinity();
      for (int row = std::max(0, robot_row - start_search);
           row <= std::min(map_.height - 1, robot_row + start_search); ++row) {
        for (int col = std::max(0, robot_col - start_search);
             col <= std::min(map_.width - 1, robot_col + start_search); ++col) {
          const int index = row * map_.width + col;
          if (!Free(col, row)) continue;
          const double distance = std::hypot(col - robot_col, row - robot_row);
          if (distance < raw_start_distance) {
            raw_start_distance = distance;
            raw_start = index;
          }
        }
      }
      if (raw_start >= 0) {
        std::deque<int> raw_reachable{raw_start};
        travel_cells[raw_start] = 0;
        while (!raw_reachable.empty()) {
          const int current = raw_reachable.front(); raw_reachable.pop_front();
          const int row = current / map_.width, col = current % map_.width;
          for (int direction = 0; direction < 4; ++direction) {
            const int x = col + cardinal_x[direction], y = row + cardinal_y[direction];
            if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) continue;
            const int next = y * map_.width + x;
            if (travel_cells[next] >= 0 || !Free(x, y)) continue;
            travel_cells[next] = travel_cells[current] + 1;
            raw_reachable.push_back(next);
          }
        }
      }
    }
  }
  // Approximate the lidar field of view from a candidate and count unique
  // unknown cells before the first occupied cell on each ray. This is the
  // actual NBV gain, rather than the size of the frontier component alone.
  const auto visible_unknown_gain = [&](int col, int row) {
    std::vector<unsigned char> seen(observation_map_.cells.size(), 0);
    const int max_range = static_cast<int>(std::ceil(3.0 / map_.resolution));
    for (int ray = 0; ray < 144; ++ray) {
      const double angle = 2.0 * std::acos(-1.0) * static_cast<double>(ray) / 144.0;
      for (int step = 1; step <= max_range; ++step) {
        const int x = col + static_cast<int>(std::lround(step * std::cos(angle)));
        const int y = row + static_cast<int>(std::lround(step * std::sin(angle)));
        if (x < 0 || y < 0 || x >= observation_map_.width ||
            y >= observation_map_.height) break;
        const int index = y * observation_map_.width + x;
        if (observation_map_.cells[index] > 50) break;
        if (observation_map_.cells[index] < 0) seen[index] = 1;
      }
    }
    return static_cast<double>(std::count(seen.begin(), seen.end(), 1)) *
           observation_map_.resolution * observation_map_.resolution;
  };
  // Actionable frontier: raw free↔unknown boundary that is on the travel
  // graph, OR a pinched tip cell adjacent to a travel cell. Narrow alcove
  // tips often fail ClearAt on the frontier cell itself while a centerline
  // pose one step back is still reachable — requiring travel on the tip
  // left candidates=0 with the alcove still visibly unfinished.
  auto approachable_frontier = [&](int col, int row) {
    if (!RawFrontier(col, row)) return false;
    const int index = row * map_.width + col;
    if (travel_cells[static_cast<std::size_t>(index)] >= 0) return true;
    constexpr int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    constexpr int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    for (int direction = 0; direction < 8; ++direction) {
      const int nc = col + dx[direction];
      const int nr = row + dy[direction];
      if (nc < 0 || nr < 0 || nc >= map_.width || nr >= map_.height) continue;
      if (travel_cells[static_cast<std::size_t>(nr * map_.width + nc)] >= 0)
        return true;
    }
    return false;
  };
  for (int row = 0; row < map_.height; ++row) for (int col = 0; col < map_.width; ++col) {
    const int index = row * map_.width + col;
    // Use the raw mapper grid for the boundary.  `map_` may contain unknown
    // cells opened in a narrow passage solely to make the navigation graph
    // traversable; using it here erases the doorway's room frontier.
    if (approachable_frontier(col, row)) frontier[index] = 1;
  }
  // Only approachable boundaries count. Unknown behind walls or outside the
  // observed component must not block completion.
  std::size_t reachable_frontier_cells = 0;
  for (int row = 0; row < map_.height; ++row) {
    for (int col = 0; col < map_.width; ++col) {
      if (approachable_frontier(col, row)) ++reachable_frontier_cells;
    }
  }
  unresolved_frontier_cells_ = reachable_frontier_cells;
  raw_frontier_cells_ = reachable_frontier_cells;
  std::vector<ExplorationGoal> goals;
  for (int seed = 0; seed < map_.width * map_.height; ++seed) {
    if (!frontier[seed] || visited[seed]) continue;
    std::deque<int> queue{seed};
    std::vector<int> cluster;
    visited[seed] = 1;
    while (!queue.empty()) {
      const int current = queue.front(); queue.pop_front(); cluster.push_back(current);
      const int row = current / map_.width, col = current % map_.width;
      for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        const int nr = row + dy, nc = col + dx;
        if (nr < 0 || nc < 0 || nr >= map_.height || nc >= map_.width) continue;
        const int next = nr * map_.width + nc;
        if (frontier[next] && !visited[next]) { visited[next] = 1; queue.push_back(next); }
      }
    }
    ++raw_frontier_clusters_;
    const int cluster_id = static_cast<int>(raw_frontier_clusters_);
    // Clusters below the minimum stay out of the exploring tour unless a cell
    // is a passable door or still has a room of unknown behind it.
    bool cluster_is_door = false;
    if (cluster.size() < static_cast<std::size_t>(config_.minimum_frontier_cells)) {
      for (const int cell : cluster) {
        const int cell_row = cell / map_.width, cell_col = cell % map_.width;
        const auto [cx, cy] = CellCenter(cell_col, cell_row);
        if (CorridorMouthAt(map_, cx, cy) ||
            UnknownRoomBeyond(map_, cell_col, cell_row)) {
          cluster_is_door = true;
          break;
        }
      }
      if (!cluster_is_door) continue;
    }
    double mean_col = 0., mean_row = 0.;
    for (int index : cluster) { mean_col += index % map_.width; mean_row += index / map_.width; }
    mean_col /= cluster.size(); mean_row /= cluster.size();
    // The arithmetic centroid need not belong to a non-convex frontier
    // component. Keep a real member cell as the topology node identity, so a
    // later GoalStillFrontier() query tests the very component that produced
    // this viewpoint rather than an arbitrary free cell in its interior.
    const int representative = *std::min_element(cluster.begin(), cluster.end(),
        [&](int lhs, int rhs) {
          const double lhs_distance = std::hypot(lhs % map_.width - mean_col,
                                                 lhs / map_.width - mean_row);
          const double rhs_distance = std::hypot(rhs % map_.width - mean_col,
                                                 rhs / map_.width - mean_row);
          return lhs_distance < rhs_distance;
        });
    const auto [frontier_x, frontier_y] = CellCenter(
        representative % map_.width, representative / map_.width);
    if (Blacklisted(frontier_x, frontier_y, true)) continue;
    for (const int cell : cluster) {
      const auto [cx, cy] = CellCenter(cell % map_.width, cell / map_.width);
      debug_snapshot_.cells.push_back({cx, cy, cluster_id});
    }
  const int search = static_cast<int>(std::ceil(config_.maximum_standoff / map_.resolution));
    const auto narrow_viewpoint = [&](int col, int row) {
      // Match SafeViewpoint / CorridorMouthAt: kitchen vestibules are often
      // narrow on BOTH axes. XOR here dropped every both-narrow mouth so
      // alcove/room frontiers stayed raw_cells>0 with candidates=0.
      const int probe = static_cast<int>(std::ceil(1.4 / map_.resolution));
      int horizontal = 0, vertical = 0;
      for (int x = col; x >= 0 && x > col - probe && Free(x, row); --x) ++horizontal;
      for (int x = col + 1; x < map_.width && x <= col + probe && Free(x, row); ++x) ++horizontal;
      for (int y = row; y >= 0 && y > row - probe && Free(col, y); --y) ++vertical;
      for (int y = row + 1; y < map_.height && y <= row + probe && Free(col, y); ++y) ++vertical;
      const int narrow_min = static_cast<int>(std::ceil(kNarrowMinWidth() / map_.resolution));
      const int narrow_max = static_cast<int>(std::floor(kNarrowMaxWidth() / map_.resolution));
      const bool narrow_h = horizontal >= narrow_min && horizontal <= narrow_max;
      const bool narrow_v = vertical >= narrow_min && vertical <= narrow_max;
      const int smaller = std::min(horizontal, vertical);
      return narrow_h || narrow_v ||
             (smaller >= narrow_min && smaller <= narrow_max);
    };
    std::optional<ExplorationGoal> best;
    for (int row = std::max(0, static_cast<int>(mean_row) - search);
         row <= std::min(map_.height - 1, static_cast<int>(mean_row) + search); ++row) {
      for (int col = std::max(0, static_cast<int>(mean_col) - search);
           col <= std::min(map_.width - 1, static_cast<int>(mean_col) + search); ++col) {
        const int viewpoint_index = row * map_.width + col;
        // Only planning-radius viewpoints on observed free. Carved/flooded
        // cells past a seal must not become tour goals (through-wall paths).
        if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 ||
            !RawFree(col, row))
          continue;
        const auto [x, y] = CellCenter(col, row);
        if (Blacklisted(x, y)) continue;
        const double standoff = std::hypot(x - frontier_x, y - frontier_y);
        if (standoff > config_.maximum_standoff ||
            (standoff < config_.minimum_standoff &&
             !(narrow_viewpoint(col, row) && standoff >= kCloseStandoff()))) continue;
        if (!HasFreeLineOfSight(col, row, representative % map_.width,
                                representative / map_.width)) continue;
        // Same floor as SafeViewpoint / planner radius — 0.16 m nominated
        // tip pinches the chassis cannot enter (UI: SG forced into a V-gap).
        const double clearance = OccupiedClearanceAt(map_, col, row);
        if (clearance < kPlanningRadius()) continue;
        const double travel = travel_cells[viewpoint_index] * map_.resolution;
        const double information_gain = std::max(
            visible_unknown_gain(col, row),
            static_cast<double>(cluster.size()) * map_.resolution * map_.resolution);
        ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                  frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                  information_gain,
                                  ViewpointScore(information_gain, travel, standoff, 2.,
                                                 clearance)};
        if (!best || candidate.score > best->score) best = candidate;
      }
    }
    if (!best) {
      // A representative cell at an L-corner may have no straight free LOS
      // even though a nearby safe pose can observe the same component.  Use a
      // conservative fallback that still requires BFS reachability and full
      // footprint clearance, but relaxes only the LOS test.
      for (int row = std::max(0, static_cast<int>(mean_row) - search);
           row <= std::min(map_.height - 1, static_cast<int>(mean_row) + search); ++row) {
        for (int col = std::max(0, static_cast<int>(mean_col) - search);
             col <= std::min(map_.width - 1, static_cast<int>(mean_col) + search); ++col) {
          const int viewpoint_index = row * map_.width + col;
          if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 ||
              !RawFree(col, row))
            continue;
          const auto [x, y] = CellCenter(col, row);
          if (Blacklisted(x, y)) continue;
          const double standoff = std::hypot(x - frontier_x, y - frontier_y);
          if (standoff > config_.maximum_standoff ||
              (standoff < config_.minimum_standoff &&
               !(narrow_viewpoint(col, row) && standoff >= kCloseStandoff()))) continue;
          const bool sees_component = std::any_of(cluster.begin(), cluster.end(), [&](int cell) {
            return HasFreeLineOfSight(col, row, cell % map_.width, cell / map_.width);
          });
          if (!sees_component) continue;
          const double clearance = OccupiedClearanceAt(map_, col, row);
          if (clearance < kPlanningRadius()) continue;
          const double travel = travel_cells[viewpoint_index] * map_.resolution;
          const double information_gain = std::max(
              visible_unknown_gain(col, row),
              static_cast<double>(cluster.size()) * map_.resolution * map_.resolution);
          ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                    frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                    information_gain,
                                    ViewpointScore(information_gain, travel, standoff, 1.5,
                                                   clearance)};
          if (!best || candidate.score > best->score) best = candidate;
        }
      }
    }
    if (!best) {
      // Corridor entrance fallback: at a tight doorway the grid ray from a
      // safe approach pose can be occluded by the near jamb even though the
      // pose is the correct place to enter and acquire a new scan.  Keep the
      // hard footprint and BFS requirements, restrict this fallback to a
      // corridor-shaped free run, and let the next scan establish LOS.
      for (int row = std::max(0, static_cast<int>(mean_row) - search * 2);
           row <= std::min(map_.height - 1, static_cast<int>(mean_row) + search * 2); ++row) {
        for (int col = std::max(0, static_cast<int>(mean_col) - search * 2);
             col <= std::min(map_.width - 1, static_cast<int>(mean_col) + search * 2); ++col) {
          const int viewpoint_index = row * map_.width + col;
          if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 ||
              !RawFree(col, row) || !narrow_viewpoint(col, row))
            continue;
          const auto [x, y] = CellCenter(col, row);
          if (Blacklisted(x, y)) continue;
          const double standoff = std::hypot(x - frontier_x, y - frontier_y);
          if (standoff < kCloseStandoff() || standoff > 1.5) continue;
          const double clearance = OccupiedClearanceAt(map_, col, row);
          if (clearance < kPlanningRadius()) continue;
          const double travel = travel_cells[viewpoint_index] * map_.resolution;
          const double information_gain = std::max(
              visible_unknown_gain(col, row),
              static_cast<double>(cluster.size()) * map_.resolution * map_.resolution);
          ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                    frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                    information_gain,
                                    ViewpointScore(information_gain, travel, standoff, 1.,
                                                   clearance)};
          if (!best || candidate.score > best->score) best = candidate;
        }
      }
    }
    if (!best) {
      // Last-resort approach in open space on the planning-radius graph.
      // Still require SafeViewpoint so StartNavigation does not bounce the
      // same unplannable T-labels until PARTIAL.
      constexpr double kApproachRadius = 2.5;
      const int approach_search = static_cast<int>(std::ceil(kApproachRadius / map_.resolution));
      for (const int frontier_cell : cluster) {
        const int target_row = frontier_cell / map_.width;
        const int target_col = frontier_cell % map_.width;
        for (int row = std::max(0, target_row - approach_search);
             row <= std::min(map_.height - 1, target_row + approach_search); ++row) {
          for (int col = std::max(0, target_col - approach_search);
               col <= std::min(map_.width - 1, target_col + approach_search); ++col) {
            const int viewpoint_index = row * map_.width + col;
            if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 ||
                !RawFree(col, row))
              continue;
            const auto [x, y] = CellCenter(col, row);
            if (Blacklisted(x, y)) continue;
            const auto [target_x, target_y] = CellCenter(target_col, target_row);
            const double standoff = std::hypot(x - target_x, y - target_y);
            if (standoff < kCloseStandoff() || standoff > kApproachRadius) continue;
            const double clearance = OccupiedClearanceAt(map_, col, row);
            if (clearance < kPlanningRadius()) continue;
            const double travel = travel_cells[viewpoint_index] * map_.resolution;
            const double information_gain = std::max(
                visible_unknown_gain(col, row),
                static_cast<double>(cluster.size()) * map_.resolution * map_.resolution);
            ExplorationGoal candidate{x, y, std::atan2(target_y - y, target_x - x),
                                      frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                      information_gain,
                                      ViewpointScore(information_gain, travel, standoff, .8,
                                                     clearance)};
            if (!best || candidate.score > best->score) best = candidate;
          }
        }
      }
    }
    if (best) {
      ExploringGoalFacts facts;
      facts.frontier_cells = best->frontier_cells;
      facts.information_gain = best->information_gain;
      facts.cell_area = map_.resolution * map_.resolution;
      facts.passable_mouth =
          CorridorMouthAt(map_, best->frontier_x, best->frontier_y);
      const int frontier_col = static_cast<int>(std::floor(
          (best->frontier_x - map_.origin_x) / map_.resolution));
      const int frontier_row = static_cast<int>(std::floor(
          (best->frontier_y - map_.origin_y) / map_.resolution));
      facts.unentered_room =
          UnknownRoomBeyond(map_, frontier_col, frontier_row) ||
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
  std::vector<ExplorationGoal> leftovers;
  if (max_count <= 0 || map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height))
    return leftovers;
  std::vector<unsigned char> frontier(map_.cells.size(), 0);
  // Same planning-radius graph as SelectGoals / StartNavigation. A looser
  // ClearAt(.18) leftover was rejected as goal-occupied before execution.
  std::vector<unsigned char> safe(map_.cells.size(), 0);
  for (int row = 0; row < map_.height; ++row) for (int col = 0; col < map_.width; ++col) {
    const std::size_t index = static_cast<std::size_t>(row) * map_.width + col;
    safe[index] = SafeViewpoint(col, row);
  }
  const int robot_col = static_cast<int>(std::floor((robot_x - map_.origin_x) / map_.resolution));
  const int robot_row = static_cast<int>(std::floor((robot_y - map_.origin_y) / map_.resolution));
  int start = -1;
  double start_distance = std::numeric_limits<double>::infinity();
  const int start_search = static_cast<int>(std::ceil(1.0 / map_.resolution));
  for (int row = std::max(0, robot_row - start_search);
       row <= std::min(map_.height - 1, robot_row + start_search); ++row)
  for (int col = std::max(0, robot_col - start_search);
       col <= std::min(map_.width - 1, robot_col + start_search); ++col) {
    const int index = row * map_.width + col;
    if (!safe[index]) continue;
    const double distance = std::hypot(col - robot_col, row - robot_row);
    if (distance < start_distance) { start_distance = distance; start = index; }
  }
  if (start < 0) return leftovers;
  std::vector<int> travel_cells(map_.cells.size(), -1);
  std::deque<int> reachable{start};
  travel_cells[start] = 0;
  constexpr int cardinal_x[] = {-1, 1, 0, 0};
  constexpr int cardinal_y[] = {0, 0, -1, 1};
  while (!reachable.empty()) {
    const int current = reachable.front(); reachable.pop_front();
    const int row = current / map_.width, col = current % map_.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int x = col + cardinal_x[direction], y = row + cardinal_y[direction];
      if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) continue;
      const int next = y * map_.width + x;
      if (!safe[next] || travel_cells[next] >= 0) continue;
      travel_cells[next] = travel_cells[current] + 1;
      reachable.push_back(next);
    }
  }
  for (int row = 0; row < map_.height; ++row) for (int col = 0; col < map_.width; ++col) {
    const int index = row * map_.width + col;
    if (RawFrontier(col, row)) frontier[index] = 1;
  }
  unresolved_frontier_cells_ = CountRawFrontierCells();
  raw_frontier_cells_ = unresolved_frontier_cells_;
  if (raw_frontier_cells_ == 0) return leftovers;
  // Leftover free/unknown specks are often smaller than
  // minimum_frontier_cells, so the exploring tour drops them.  The pocket
  // they belong to is usually still reachable through ordinary open space.
  // Place a normal viewpoint on that open graph; the planner takes the wide
  // route, it does not squeeze a 1.2 m end gap.
  constexpr double kLeftoverRadius = 6.0;
  const int leftover_search = static_cast<int>(std::ceil(kLeftoverRadius / map_.resolution));
  struct RankedLeftover {
    ExplorationGoal goal;
    bool room_mouth = false;
  };
  std::vector<RankedLeftover> ranked;
  // Completing must still see mouths that narrow failure cools hid from the
  // exploring tour. Wide pocket cools (>=1.5 m, tip-leave / thrash) must
  // suppress leftovers too — otherwise the robot re-enters a wall tip with a
  // frontier_cells=0 retreat ping-pong and never escapes.
  const std::size_t known_now = KnownCells();
  const auto leftover_suppressed = [&](double x, double y) {
    if (tips_.Blocked(x, y) || PersistentlyBlacklisted(x, y)) return true;
    return std::any_of(blacklist_.begin(), blacklist_.end(), [&](const auto& entry) {
      if (entry.persistent || entry.radius < 1.5) return false;
      if (known_now >= entry.known_cells + entry.required_growth) return false;
      return std::hypot(x - entry.x, y - entry.y) < entry.radius;
    });
  };
  for (int index = 0; index < map_.width * map_.height; ++index) {
    if (!frontier[index]) continue;
    const int target_row = index / map_.width, target_col = index % map_.width;
    const auto [frontier_x, frontier_y] = CellCenter(target_col, target_row);
    if (leftover_suppressed(frontier_x, frontier_y)) continue;
    // Skip tips with no reachable approach cell — wall-external unknown.
    bool approachable = travel_cells[static_cast<std::size_t>(index)] >= 0;
    if (!approachable) {
      for (int dy = -1; dy <= 1 && !approachable; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nc = target_col + dx, nr = target_row + dy;
          if (nc < 0 || nr < 0 || nc >= map_.width || nr >= map_.height) continue;
          if (travel_cells[static_cast<std::size_t>(nr * map_.width + nc)] >= 0) {
            approachable = true;
            break;
          }
        }
      }
    }
    if (!approachable) continue;
    const bool room_mouth = CorridorMouthAt(map_, frontier_x, frontier_y);
    std::optional<ExplorationGoal> best_for_tip;
    for (int row = std::max(0, target_row - leftover_search);
         row <= std::min(map_.height - 1, target_row + leftover_search); ++row) {
      for (int col = std::max(0, target_col - leftover_search);
           col <= std::min(map_.width - 1, target_col + leftover_search); ++col) {
        const int viewpoint_index = row * map_.width + col;
        if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 ||
            !RawFree(col, row))
          continue;
        const auto [x, y] = CellCenter(col, row);
        if (leftover_suppressed(x, y)) continue;
        const double standoff = std::hypot(x - frontier_x, y - frontier_y);
        if (standoff < kCloseStandoff() || standoff > kLeftoverRadius) continue;
        const double clearance = OccupiedClearanceAt(map_, col, row);
        if (clearance < kPlanningRadius()) continue;
        const double travel = travel_cells[viewpoint_index] * map_.resolution;
        ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                  frontier_x, frontier_y, 1, map_.resolution,
                                  ViewpointScore(map_.resolution, travel, standoff, .8,
                                                 clearance),
                                  true};
        if (!best_for_tip || candidate.score > best_for_tip->score)
          best_for_tip = candidate;
      }
    }
    if (!best_for_tip) continue;
    ranked.push_back({*best_for_tip, room_mouth});
  }
  std::sort(ranked.begin(), ranked.end(), [](const RankedLeftover& a,
                                             const RankedLeftover& b) {
    if (a.room_mouth != b.room_mouth) return a.room_mouth && !b.room_mouth;
    return a.goal.score > b.goal.score;
  });
  for (const auto& entry : ranked) {
    const bool duplicate = std::any_of(
        leftovers.begin(), leftovers.end(), [&](const ExplorationGoal& prior) {
          return std::hypot(prior.frontier_x - entry.goal.frontier_x,
                            prior.frontier_y - entry.goal.frontier_y) < .80 ||
                 std::hypot(prior.x - entry.goal.x, prior.y - entry.goal.y) < .80;
        });
    if (duplicate) continue;
    leftovers.push_back(entry.goal);
    if (static_cast<int>(leftovers.size()) >= max_count) break;
  }
  return leftovers;
}

std::vector<ExplorationGoal> FrontierExplorer::StableGoals(
    std::vector<ExplorationGoal> goals) {
  // Frontiers are sensor-derived hypotheses, not facts.  A clipped range ray
  // can create a large enough one-frame unknown/free boundary to pass the
  // geometry test.  Track components by their representative position across
  // map revisions and promote only persistent components to navigation work.
  ++stability_observation_;
  const auto stale = std::remove_if(frontier_tracks_.begin(), frontier_tracks_.end(),
      [&](const FrontierTrack& track) { return stability_observation_ > track.last_observation + 2; });
  frontier_tracks_.erase(stale, frontier_tracks_.end());
  std::vector<ExplorationGoal> stable;
  for (const auto& goal : goals) {
    auto track = std::min_element(frontier_tracks_.begin(), frontier_tracks_.end(),
        [&](const FrontierTrack& first, const FrontierTrack& second) {
          const double first_distance = std::min(
              std::hypot(first.x - goal.frontier_x, first.y - goal.frontier_y),
              std::hypot(first.goal_x - goal.x, first.goal_y - goal.y));
          const double second_distance = std::min(
              std::hypot(second.x - goal.frontier_x, second.y - goal.frontier_y),
              std::hypot(second.goal_x - goal.x, second.goal_y - goal.y));
          return first_distance < second_distance;
        });
    const double association_distance = track == frontier_tracks_.end()
        ? std::numeric_limits<double>::infinity()
        : std::min(std::hypot(track->x - goal.frontier_x, track->y - goal.frontier_y),
                   std::hypot(track->goal_x - goal.x, track->goal_y - goal.y));
    if (track == frontier_tracks_.end() ||
        association_distance > config_.frontier_stability_radius) {
      frontier_tracks_.push_back({goal.frontier_x, goal.frontier_y, goal.x, goal.y,
                                  1, stability_observation_});
      if (config_.required_frontier_observations <= 1) stable.push_back(goal);
      continue;
    }
    if (track->last_observation != stability_observation_) ++track->observations;
    track->x = goal.frontier_x;
    track->y = goal.frontier_y;
    track->goal_x = goal.x;
    track->goal_y = goal.y;
    track->last_observation = stability_observation_;
    if (track->observations >= config_.required_frontier_observations) stable.push_back(goal);
  }
  return stable;
}

std::vector<ExplorationGoal> FrontierExplorer::OneGoalPerMouth(
    std::vector<ExplorationGoal> goals, double robot_x, double robot_y) const {
  // Spatial hypercluster: micro-cluster frontiers within ~1.75 m are one mouth
  // (doorway / alcove). Only the strongest viewpoint survives; then prefer an
  // entry pose on the unknown side when the planning graph permits it.
  constexpr double kMouthMergeM = 1.5;
  const auto better = [](const ExplorationGoal& a, const ExplorationGoal& b) {
    if (a.information_gain > b.information_gain + 1e-9) return true;
    if (b.information_gain > a.information_gain + 1e-9) return false;
    if (a.frontier_cells != b.frontier_cells) return a.frontier_cells > b.frontier_cells;
    return a.score > b.score;
  };
  std::vector<ExplorationGoal> mouths;
  mouths.reserve(goals.size());
  for (const auto& goal : goals) {
    bool absorbed = false;
    for (auto& keep : mouths) {
      if (std::hypot(keep.frontier_x - goal.frontier_x,
                     keep.frontier_y - goal.frontier_y) > kMouthMergeM &&
          std::hypot(keep.x - goal.x, keep.y - goal.y) > kMouthMergeM)
        continue;
      if (better(goal, keep)) {
        const int cells = std::max(keep.frontier_cells, goal.frontier_cells);
        const double gain = std::max(keep.information_gain, goal.information_gain);
        keep = goal;
        keep.frontier_cells = cells;
        keep.information_gain = gain;
      } else {
        keep.frontier_cells = std::max(keep.frontier_cells, goal.frontier_cells);
        keep.information_gain = std::max(keep.information_gain, goal.information_gain);
      }
      absorbed = true;
      break;
    }
    if (!absorbed) mouths.push_back(goal);
  }
  // Prefer a room-side pose when the viewpoint is still on the approach side.
  // If the viewpoint search already placed the goal past the mouth, do not let
  // SelectEntryProbe reverse the approach vector and pull it back into the hall.
  const auto depth_past_frontier = [&](const ExplorationGoal& goal, double x,
                                       double y) {
    const double to_fx = goal.frontier_x - robot_x;
    const double to_fy = goal.frontier_y - robot_y;
    const double length = std::hypot(to_fx, to_fy);
    if (length < .05) return 0.;
    return ((x - goal.frontier_x) * to_fx + (y - goal.frontier_y) * to_fy) / length;
  };
  const auto corridor_mouth = [&](const ExplorationGoal& goal) {
    return CorridorMouthAt(map_, goal.frontier_x, goal.frontier_y);
  };
  const auto snap_to_door_centerline = [&](ExplorationGoal& goal) {
    const double to_fx = goal.frontier_x - robot_x;
    const double to_fy = goal.frontier_y - robot_y;
    const double length = std::hypot(to_fx, to_fy);
    if (length < .05) return;
    const double dir_x = to_fx / length;
    const double dir_y = to_fy / length;
    const int search = static_cast<int>(std::ceil(.70 / map_.resolution));
    const int fcol = static_cast<int>(std::floor(
        (goal.frontier_x - map_.origin_x) / map_.resolution));
    const int frow = static_cast<int>(std::floor(
        (goal.frontier_y - map_.origin_y) / map_.resolution));
    // Mid-gap: maximize clearance (not depth). Old depth-max parked the
    // standoff on the deeper jamb side so RPP froze at an easy doorway.
    double best_score = -1.;
    int best_col = -1, best_row = -1;
    for (int row = std::max(0, frow - search);
         row <= std::min(map_.height - 1, frow + search); ++row) {
      for (int col = std::max(0, fcol - search);
           col <= std::min(map_.width - 1, fcol + search); ++col) {
        // Stay on observed free. Carved/flooded cells past a SLAM seal look
        // safe on map_ but force through-wall plans on the live grid.
        if (!SafeViewpoint(col, row) || !RawFree(col, row)) continue;
        const auto [x, y] = CellCenter(col, row);
        const double depth = depth_past_frontier(goal, x, y);
        const double lateral = std::abs((x - goal.frontier_x) * dir_y -
                                        (y - goal.frontier_y) * dir_x);
        if (lateral > .28 || depth < -.10 || depth > .55) continue;
        const double clearance = OccupiedClearanceAt(map_, col, row);
        if (clearance < kPlanningRadius()) continue;
        const double score = 5.0 * clearance - 2.0 * lateral + 0.4 * depth;
        if (score > best_score) {
          best_score = score;
          best_col = col;
          best_row = row;
        }
      }
    }
    if (best_col < 0) return;
    const auto [x, y] = CellCenter(best_col, best_row);
    goal.x = x;
    goal.y = y;
    goal.yaw = std::atan2(dir_y, dir_x);
  };
  for (auto& goal : mouths) {
    const double current_depth = depth_past_frontier(goal, goal.x, goal.y);
    if (current_depth >= kCloseStandoff()) continue;
    bool advanced = false;
    // Only pull the viewpoint into the room when that cell is already free
    // on the live observation map. Entry into still-unknown rooms is owned
    // by StartEntryProbe after the hall standoff is reached.
    if (auto probe = SelectEntryProbe(goal, robot_x, robot_y, kCloseStandoff(),
                                      .5 * kNarrowMaxWidth())) {
      const int pcol = static_cast<int>(std::floor(
          (probe->x - map_.origin_x) / map_.resolution));
      const int prow = static_cast<int>(std::floor(
          (probe->y - map_.origin_y) / map_.resolution));
      if (RawFree(pcol, prow) &&
          depth_past_frontier(goal, probe->x, probe->y) > current_depth) {
        goal.x = probe->x;
        goal.y = probe->y;
        goal.yaw = probe->yaw;
        advanced = true;
      }
    }
    if (!advanced || corridor_mouth(goal)) snap_to_door_centerline(goal);
  }
  return mouths;
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
  regions_.ClearPassages();
  if (map_.width <= 0 || map_.height <= 0 || map_.resolution <= 0.) return;
  for (int row = 0; row < map_.height; ++row) {
    for (int col = 0; col < map_.width; ++col) {
      if (!RawFrontier(col, row)) continue;
      const auto [x, y] = CellCenter(col, row);
      if (tips_.Blocked(x, y) || PersistentlyBlacklisted(x, y)) continue;
      if (!CorridorMouthAt(map_, x, y)) continue;
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
  // Mid-drive preempt is only for a clearly nearer local mouth while still
  // far from the active goal. Loose thresholds rebuilt the tour every scan
  // and re-selected the same far T-label (open-space stutter).
  constexpr double kLocalPocketM = 2.5;
  constexpr double kFarSlackM = 1.5;
  constexpr double kFarRatio = 2.5;
  constexpr double kSameMouthM = .80;
  constexpr double kMinFarRemainingM = 5.0;
  if (path_remaining_m > 0. && path_remaining_m < kMinFarRemainingM) return false;
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height))
    return false;

  std::vector<unsigned char> safe(map_.cells.size(), 0);
  for (int row = 0; row < map_.height; ++row) {
    for (int col = 0; col < map_.width; ++col) {
      safe[static_cast<std::size_t>(row) * map_.width + col] = SafeViewpoint(col, row);
    }
  }
  const auto nearest_safe = [&](double x, double y) -> int {
    const int center_col = static_cast<int>(std::floor((x - map_.origin_x) / map_.resolution));
    const int center_row = static_cast<int>(std::floor((y - map_.origin_y) / map_.resolution));
    int best = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    const int radius = static_cast<int>(std::ceil(1.0 / map_.resolution));
    for (int row = std::max(0, center_row - radius);
         row <= std::min(map_.height - 1, center_row + radius); ++row) {
      for (int col = std::max(0, center_col - radius);
           col <= std::min(map_.width - 1, center_col + radius); ++col) {
        const int index = row * map_.width + col;
        if (!safe[index]) continue;
        const double distance = std::hypot(col - center_col, row - center_row);
        if (distance < best_distance) { best_distance = distance; best = index; }
      }
    }
    return best;
  };
  const int start = nearest_safe(robot_x, robot_y);
  if (start < 0) return false;
  std::vector<int> distance(map_.cells.size(), -1);
  std::deque<int> queue{start};
  distance[start] = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    const int row = current / map_.width, col = current % map_.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int x = col + dx[direction], y = row + dy[direction];
      if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) continue;
      const int next = y * map_.width + x;
      if (!safe[next] || distance[next] >= 0) continue;
      distance[next] = distance[current] + 1;
      queue.push_back(next);
    }
  }
  const auto bfs_m = [&](double x, double y) -> double {
    const int cell = nearest_safe(x, y);
    if (cell < 0 || distance[cell] < 0) return std::numeric_limits<double>::infinity();
    return distance[cell] * map_.resolution;
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

double FrontierExplorer::MouthGapWidth(double frontier_x,
                                       double frontier_y) const {
  if (map_.resolution <= 0. || map_.width <= 0 || map_.height <= 0) return 0.;
  const int col = static_cast<int>(
      std::floor((frontier_x - map_.origin_x) / map_.resolution));
  const int row = static_cast<int>(
      std::floor((frontier_y - map_.origin_y) / map_.resolution));
  if (col < 0 || row < 0 || col >= map_.width || row >= map_.height) return 0.;
  auto free = [&](int c, int r) {
    if (c < 0 || r < 0 || c >= map_.width || r >= map_.height) return false;
    return map_.cells[static_cast<std::size_t>(r) * map_.width + c] == 0;
  };
  const int probe = static_cast<int>(std::ceil(1.4 / map_.resolution));
  int horizontal = 0, vertical = 0;
  for (int x = col; x >= 0 && x > col - probe && free(x, row); --x) ++horizontal;
  for (int x = col + 1; x < map_.width && x <= col + probe && free(x, row); ++x)
    ++horizontal;
  for (int y = row; y >= 0 && y > row - probe && free(col, y); --y) ++vertical;
  for (int y = row + 1; y < map_.height && y <= row + probe && free(col, y); ++y)
    ++vertical;
  // Throat width is the narrow free run (door gap), not the deep room axis.
  return static_cast<double>(std::min(horizontal, vertical)) * map_.resolution;
}

bool FrontierExplorer::IsCorridorMouth(double frontier_x,
                                       double frontier_y) const {
  return CorridorMouthAt(map_, frontier_x, frontier_y);
}

double FrontierExplorer::MouthFreeDepth(const ExplorationGoal& frontier_goal,
                                        double robot_x, double robot_y) const {
  if (map_.resolution <= 0. || map_.width <= 2 || map_.height <= 2) return 0.;
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
  for (double step = map_.resolution; step <= 1.40 + 1e-6;
       step += map_.resolution) {
    const double x = frontier_goal.frontier_x + step * dir_x;
    const double y = frontier_goal.frontier_y + step * dir_y;
    const int col =
        static_cast<int>(std::floor((x - map_.origin_x) / map_.resolution));
    const int row =
        static_cast<int>(std::floor((y - map_.origin_y) / map_.resolution));
    if (col < 0 || row < 0 || col >= map_.width || row >= map_.height) break;
    if (!Free(col, row) || !ClearAt(col, row, .14)) break;
    depth = step;
  }
  return depth;
}

std::optional<ExplorationGoal> FrontierExplorer::SelectEntryProbe(
    const ExplorationGoal& frontier_goal, double robot_x, double robot_y,
    double minimum_depth_m, double maximum_depth_m) const {
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height) ||
      minimum_depth_m <= 0. || maximum_depth_m < minimum_depth_m)
    return std::nullopt;

  // Entry axis must point into the unexplored room. Viewpoint→frontier is
  // often lateral (standoff beside the jamb), which placed the probe along
  // the hall wall and left the robot stuck on the threshold with CARROT
  // inside the room. Prefer the raw unknown centroid (observation map).
  double direction_x = 0.;
  double direction_y = 0.;
  {
    const ExplorationGrid& obs =
        (observation_map_.width == map_.width &&
         observation_map_.height == map_.height &&
         observation_map_.resolution > 0.)
            ? observation_map_
            : map_;
    const int fcol = static_cast<int>(std::floor(
        (frontier_goal.frontier_x - obs.origin_x) / obs.resolution));
    const int frow = static_cast<int>(std::floor(
        (frontier_goal.frontier_y - obs.origin_y) / obs.resolution));
    const int reach = static_cast<int>(std::ceil(1.20 / obs.resolution));
    double unknown_x = 0.;
    double unknown_y = 0.;
    int unknown_n = 0;
    for (int row = std::max(0, frow - reach);
         row <= std::min(obs.height - 1, frow + reach); ++row) {
      for (int col = std::max(0, fcol - reach);
           col <= std::min(obs.width - 1, fcol + reach); ++col) {
        if (obs.cells[static_cast<std::size_t>(row) * obs.width + col] >= 0)
          continue;
        const double x = obs.origin_x + (col + .5) * obs.resolution;
        const double y = obs.origin_y + (row + .5) * obs.resolution;
        unknown_x += x - frontier_goal.frontier_x;
        unknown_y += y - frontier_goal.frontier_y;
        ++unknown_n;
      }
    }
    if (unknown_n >= 3) {
      direction_x = unknown_x / static_cast<double>(unknown_n);
      direction_y = unknown_y / static_cast<double>(unknown_n);
    }
  }
  if (std::hypot(direction_x, direction_y) < .05) {
    direction_x = frontier_goal.frontier_x - robot_x;
    direction_y = frontier_goal.frontier_y - robot_y;
  }
  if (std::hypot(direction_x, direction_y) < .05) {
    direction_x = frontier_goal.frontier_x - frontier_goal.x;
    direction_y = frontier_goal.frontier_y - frontier_goal.y;
  }
  const double approach_length = std::hypot(direction_x, direction_y);
  if (approach_length < .05) return std::nullopt;
  direction_x /= approach_length;
  direction_y /= approach_length;
  // Match the planning radius. The old 0.14 m floor let SelectGoals / entry
  // push poses into V-tip pinches the chassis cannot traverse.
  const double kEntryTraverseClearance = kPlanningRadius();
  const double kEntryGoalClearance = kPlanningRadius();
  const auto near_observed_free = [&](int col, int row) {
    if (RawFree(col, row)) return true;
    // Allow cells that touch observed free so probes stay near mapped space.
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {
        if (RawFree(col + dx, row + dy)) return true;
      }
    }
    return false;
  };
  const auto entry_traverse = [&](int col, int row) {
    return Free(col, row) && ClearAt(col, row, kEntryTraverseClearance) &&
           near_observed_free(col, row);
  };
  const auto entry_goal = [&](int col, int row) {
    return Free(col, row) && ClearAt(col, row, kEntryGoalClearance) &&
           near_observed_free(col, row);
  };
  const int robot_col = static_cast<int>(std::floor(
      (robot_x - map_.origin_x) / map_.resolution));
  const int robot_row = static_cast<int>(std::floor(
      (robot_y - map_.origin_y) / map_.resolution));
  const int start_search = static_cast<int>(std::ceil(1.0 / map_.resolution));
  int start = -1;
  double start_distance = std::numeric_limits<double>::infinity();
  for (int row = std::max(0, robot_row - start_search);
       row <= std::min(map_.height - 1, robot_row + start_search); ++row) {
    for (int col = std::max(0, robot_col - start_search);
         col <= std::min(map_.width - 1, robot_col + start_search); ++col) {
      if (!entry_traverse(col, row)) continue;
      const double distance = std::hypot(col - robot_col, row - robot_row);
      if (distance < start_distance) {
        start_distance = distance;
        start = row * map_.width + col;
      }
    }
  }
  if (start < 0) return std::nullopt;

  // Entry probe walks the planning map (same occupancy as the mapper).
  std::vector<int> distance(map_.cells.size(), -1);
  std::deque<int> queue{start};
  distance[start] = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    const int row = current / map_.width;
    const int col = current % map_.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + dx[direction];
      const int next_row = row + dy[direction];
      if (next_col < 0 || next_row < 0 || next_col >= map_.width ||
          next_row >= map_.height)
        continue;
      const int next = next_row * map_.width + next_col;
      if (distance[next] >= 0 || !entry_traverse(next_col, next_row)) continue;
      distance[next] = distance[current] + 1;
      queue.push_back(next);
    }
  }

  std::optional<ExplorationGoal> best;
  double best_score = -std::numeric_limits<double>::infinity();
  const int search = static_cast<int>(std::ceil(maximum_depth_m / map_.resolution)) + 2;
  const int frontier_col = static_cast<int>(std::floor(
      (frontier_goal.frontier_x - map_.origin_x) / map_.resolution));
  const int frontier_row = static_cast<int>(std::floor(
      (frontier_goal.frontier_y - map_.origin_y) / map_.resolution));
  for (int row = std::max(0, frontier_row - search);
       row <= std::min(map_.height - 1, frontier_row + search); ++row) {
    for (int col = std::max(0, frontier_col - search);
         col <= std::min(map_.width - 1, frontier_col + search); ++col) {
      const int index = row * map_.width + col;
      if (distance[index] < 0 || !entry_goal(col, row)) continue;
      const auto [x, y] = CellCenter(col, row);
      const double relative_x = x - frontier_goal.frontier_x;
      const double relative_y = y - frontier_goal.frontier_y;
      const double depth = relative_x * direction_x + relative_y * direction_y;
      const double lateral = std::abs(relative_x * direction_y - relative_y * direction_x);
      // Keep probes in the door throat center — .38 m lateral parked on the
      // jamb (UI: GOAL on left threshold wall, path clear ~0.06 m). Wide easy
      // mouths allow a hair more lateral so angled hall approaches still land
      // mid-gap instead of returning nullopt and abandoning the door.
      const double gap =
          MouthGapWidth(frontier_goal.frontier_x, frontier_goal.frontier_y);
      const double max_lateral = gap >= .50 ? .28 : .20;
      if (depth < minimum_depth_m || depth > maximum_depth_m ||
          lateral > max_lateral)
        continue;
      // Do not select a pose that is merely on the approach side of the
      // frontier or an unrelated free cell behind a nearby wall.
      if (std::hypot(relative_x, relative_y) > maximum_depth_m + .20) continue;
      const double clearance = OccupiedClearanceAt(map_, col, row);
      if (clearance < kPlanningRadius()) continue;
      // Clearance dominates: mid-gap beats a deeper jamb-hugging cell.
      const double score = 0.35 * depth + 5.5 * clearance - 3.0 * lateral -
                           .01 * static_cast<double>(distance[index]);
      if (score > best_score) {
        best_score = score;
        ExplorationGoal probe = frontier_goal;
        probe.x = x;
        probe.y = y;
        probe.yaw = std::atan2(direction_y, direction_x);
        probe.score = score;
        best = probe;
      }
    }
  }
  if (best) return best;
  // Geometric fallback: place the probe on the approach axis when BFS cannot
  // reach a candidate. Prefer the lateral with the largest jamb clearance.
  const double lateral_unit_x = -direction_y;
  const double lateral_unit_y = direction_x;
  std::optional<ExplorationGoal> fallback;
  double fallback_clear = -1.;
  for (double depth = minimum_depth_m; depth <= maximum_depth_m + 1e-6;
       depth += std::max(.05, map_.resolution)) {
    for (const double lateral : {0., .08, -.08, .14, -.14}) {
      const double x = frontier_goal.frontier_x + depth * direction_x +
                       lateral * lateral_unit_x;
      const double y = frontier_goal.frontier_y + depth * direction_y +
                       lateral * lateral_unit_y;
      const int col = static_cast<int>(std::floor((x - map_.origin_x) / map_.resolution));
      const int row = static_cast<int>(std::floor((y - map_.origin_y) / map_.resolution));
      if (!entry_goal(col, row)) continue;
      const double clearance = OccupiedClearanceAt(map_, col, row);
      if (clearance < kPlanningRadius()) continue;
      if (clearance > fallback_clear) {
        fallback_clear = clearance;
        ExplorationGoal probe = frontier_goal;
        probe.x = map_.origin_x + (col + .5) * map_.resolution;
        probe.y = map_.origin_y + (row + .5) * map_.resolution;
        probe.yaw = std::atan2(direction_y, direction_x);
        probe.score = depth + 4.5 * clearance;
        fallback = probe;
      }
    }
    if (fallback && fallback_clear >= kPlanningRadius()) return fallback;
  }
  return fallback;
}

bool FrontierExplorer::GoalStillFrontier(const ExplorationGoal& goal) const {
  if (observation_map_.resolution <= 0. || observation_map_.width <= 0 ||
      observation_map_.height <= 0) return false;
  const int col = static_cast<int>(std::floor(
      (goal.frontier_x - observation_map_.origin_x) / observation_map_.resolution));
  const int row = static_cast<int>(std::floor(
      (goal.frontier_y - observation_map_.origin_y) / observation_map_.resolution));
  return RawFrontier(col, row);
}

bool FrontierExplorer::UnenteredRoomBeyond(const ExplorationGoal& goal) const {
  const ExplorationGrid& map = observation_map_;
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
  if (GoalStillFrontier(goal) || observation_map_.resolution <= 0.)
    return GoalStillFrontier(goal);
  const int center_col = static_cast<int>(std::floor(
      (goal.frontier_x - observation_map_.origin_x) / observation_map_.resolution));
  const int center_row = static_cast<int>(std::floor(
      (goal.frontier_y - observation_map_.origin_y) / observation_map_.resolution));
  // The association radius is deliberately smaller than the viewpoint
  // standoff. It follows a shifted sensor boundary but cannot jump across a
  // shelf to an unrelated frontier on its far side.
  const int radius = static_cast<int>(std::ceil(.35 / observation_map_.resolution));
  for (int row = std::max(0, center_row - radius);
       row <= std::min(observation_map_.height - 1, center_row + radius); ++row) {
    for (int col = std::max(0, center_col - radius);
         col <= std::min(observation_map_.width - 1, center_col + radius); ++col) {
      if (std::hypot(col - center_col, row - center_row) > radius) continue;
      if (RawFrontier(col, row)) return true;
    }
  }
  return false;
}

void FrontierExplorer::RecordAttempt(const ExplorationGoal& goal, bool succeeded,
                                     bool region_complete, bool cool_frontier) {
  // Failure describes an attempted observation pose, not an entire doorway.
  // A distant standoff must never create a room-sized exclusion disk around
  // its frontier; other safe viewpoints on the same boundary remain useful.
  const std::size_t known = KnownCells();
  const bool door = CorridorMouthAt(map_, goal.frontier_x, goal.frontier_y) ||
                    UnenteredRoomBeyond(goal);
  if (succeeded) {
    if (!door && GoalRegionStillFrontier(goal)) tips_.NoteNoNewArea(goal.frontier_x, goal.frontier_y);
    else tips_.NoteProgress(goal.frontier_x, goal.frontier_y);
    blacklist_.push_back({goal.frontier_x, goal.frontier_y,
                          kVisitedFrontierRadius(), known, 20000u, true});
    blacklist_.push_back({goal.x, goal.y, kVisitedViewpointRadius(),
                          known, 20000u, true});
    if (region_complete) ++completed_goals_;
  } else {
    tips_.NoteUnreached(goal.frontier_x, goal.frontier_y, door);
    // A failed route is evidence about the doorway/frontier, not just the
    // exact viewpoint.  Only cooling the pose lets the next map revision
    // regenerate the same three poses around one mouth, which produced the
    // long pure-rotation loops seen on the robot.  Cool the frontier mouth as
    // well; fresh map growth can still make it eligible again.
    const double pose_radius = std::max(map_.resolution,
                                        config_.footprint_clearance);
    // Failed bathroom/kitchen mouths were re-queued within a few map updates
    // (600/1500 known-cell cools), so the robot spent minutes retrying the
    // same tip. Cool longer and wider so the tour moves on.
    blacklist_.push_back({goal.x, goal.y, std::max(.55, pose_radius), known,
                          2500u, false});
    if (cool_frontier) {
      blacklist_.push_back(
          {goal.frontier_x, goal.frontier_y, 1.10, known, 4000u, false});
    }
    ++failed_goals_;
  }
  if (tips_.Blocked(goal.frontier_x, goal.frontier_y)) {
    blacklist_.push_back({goal.frontier_x, goal.frontier_y,
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
  blacklist_.push_back(
      {x, y, radius_m, KnownCells(), required_known_growth, false});
}

std::size_t FrontierExplorer::KnownCells() const {
  return static_cast<std::size_t>(std::count_if(
      observation_map_.cells.begin(), observation_map_.cells.end(),
      [](std::int8_t value) { return value >= 0; }));
}

std::size_t FrontierExplorer::ReachableUnknownCells(double robot_x, double robot_y) const {
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height) ||
      observation_map_.cells.size() != map_.cells.size())
    return 0;
  const int robot_col = static_cast<int>(std::floor(
      (robot_x - map_.origin_x) / map_.resolution));
  const int robot_row = static_cast<int>(std::floor(
      (robot_y - map_.origin_y) / map_.resolution));
  // Traverse carved door noses with the narrow clearance, not full SafeViewpoint:
  // a freshly opened 0.32–0.50 m throat often fails the soft footprint floor.
  auto traversable = [&](int col, int row) {
    return Free(col, row) && ClearAt(col, row, kNarrowClearance());
  };
  const int search = static_cast<int>(std::ceil(1.0 / map_.resolution));
  int start = -1;
  double nearest = std::numeric_limits<double>::infinity();
  for (int row = std::max(0, robot_row - search);
       row <= std::min(map_.height - 1, robot_row + search); ++row) {
    for (int col = std::max(0, robot_col - search);
         col <= std::min(map_.width - 1, robot_col + search); ++col) {
      if (!traversable(col, row)) continue;
      const double distance = std::hypot(col - robot_col, row - robot_row);
      if (distance < nearest) {
        nearest = distance;
        start = row * map_.width + col;
      }
    }
  }
  if (start < 0) return 0;
  std::vector<unsigned char> visited(map_.cells.size(), 0);
  std::deque<int> queue{start};
  visited[start] = 1;
  std::size_t reachable_unknown = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    if (observation_map_.cells[static_cast<std::size_t>(current)] < 0 &&
        map_.cells[static_cast<std::size_t>(current)] == 0)
      ++reachable_unknown;
    const int row = current / map_.width;
    const int col = current % map_.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + dx[direction];
      const int next_row = row + dy[direction];
      if (next_col < 0 || next_row < 0 || next_col >= map_.width ||
          next_row >= map_.height)
        continue;
      const int next = next_row * map_.width + next_col;
      if (visited[static_cast<std::size_t>(next)] ||
          !traversable(next_col, next_row))
        continue;
      visited[static_cast<std::size_t>(next)] = 1;
      queue.push_back(next);
    }
  }
  return reachable_unknown;
}

std::optional<ExplorationGoal> FrontierExplorer::SettleGoal(
    const ExplorationGoal& goal) const {
  if (map_.width <= 0 || map_.height <= 0 || map_.resolution <= 0. ||
      !std::isfinite(goal.x) || !std::isfinite(goal.y))
    return std::nullopt;
  const double clearance = kPlanningRadius() + map_.resolution;
  const int col0 = static_cast<int>(std::floor((goal.x - map_.origin_x) / map_.resolution));
  const int row0 = static_cast<int>(std::floor((goal.y - map_.origin_y) / map_.resolution));
  auto settled = [&](int col, int row) {
    return Free(col, row) && RawFree(col, row) && ClearAt(col, row, clearance);
  };
  if (settled(col0, row0)) return goal;
  const int reach = static_cast<int>(std::ceil(kSwingClearance() / map_.resolution));
  double best = std::numeric_limits<double>::infinity();
  int best_col = -1, best_row = -1;
  for (int row = row0 - reach; row <= row0 + reach; ++row) {
    for (int col = col0 - reach; col <= col0 + reach; ++col) {
      const double d = std::hypot(col - col0, row - row0);
      if (d > reach || d >= best || !settled(col, row)) continue;
      best = d;
      best_col = col;
      best_row = row;
    }
  }
  if (best_col < 0) return std::nullopt;
  ExplorationGoal moved = goal;
  std::tie(moved.x, moved.y) = CellCenter(best_col, best_row);
  return moved;
}

std::vector<ExplorationGoal> FrontierExplorer::UnvisitedSpaceGoals(
    const std::vector<std::pair<double, double>>& trail, double visit_radius_m,
    double minimum_area_m2, const std::vector<std::pair<double, double>>& room_seeds,
    double seed_radius_m) const {
  std::vector<ExplorationGoal> goals;
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height) ||
      trail.empty() || room_seeds.empty())
    return goals;
  const int n = map_.width * map_.height;
  const double inf = std::numeric_limits<double>::infinity();
  constexpr int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  constexpr int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
  const double step[] = {1., 1., 1., 1., std::sqrt(2.), std::sqrt(2.),
                         std::sqrt(2.), std::sqrt(2.)};
  // `ascend` limits the flood to steps that do not get closer to the trail,
  // so a seed spreads only into the space behind its door.
  auto free_distance = [&](const std::vector<std::pair<double, double>>& sources,
                           double limit_m, int snap_cells, const std::vector<double>* ascend) {
    std::vector<double> distance(n, inf);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    for (const auto& [x, y] : sources) {
      const int col = static_cast<int>(std::floor((x - map_.origin_x) / map_.resolution));
      const int row = static_cast<int>(std::floor((y - map_.origin_y) / map_.resolution));
      int best = -1;
      double best_d = inf;
      for (int r = row - snap_cells; r <= row + snap_cells; ++r) {
        for (int c = col - snap_cells; c <= col + snap_cells; ++c) {
          if (!Free(c, r)) continue;
          const double d = std::hypot(c - col, r - row) * map_.resolution;
          if (d < best_d) { best_d = d; best = r * map_.width + c; }
        }
      }
      if (best >= 0 && distance[best] > 0.) { distance[best] = 0.; open.emplace(0., best); }
    }
    while (!open.empty()) {
      const auto [d, i] = open.top();
      open.pop();
      if (d > distance[i]) continue;
      const int col = i % map_.width, row = i / map_.width;
      for (int k = 0; k < 8; ++k) {
        const int c = col + dx[k], r = row + dy[k];
        if (!Free(c, r)) continue;
        const int j = r * map_.width + c;
        if (ascend && (*ascend)[j] < (*ascend)[i]) continue;
        const double next = d + step[k] * map_.resolution;
        if (next <= limit_m && next < distance[j]) { distance[j] = next; open.emplace(next, j); }
      }
    }
    return distance;
  };
  const auto distance = free_distance(trail, inf, 0, nullptr);
  // Probe targets sit just past a jamb and may land on a wall cell once the
  // room is mapped.
  const auto seed_distance = free_distance(
      room_seeds, seed_radius_m,
      static_cast<int>(std::ceil(kPlanningRadius() / map_.resolution)), &distance);
  // Unreachable free cells (inf) are not candidates: no free path from the
  // trail reaches them, so a visit goal there would never plan.
  auto far = [&](int i) {
    return std::isfinite(distance[i]) && distance[i] > visit_radius_m;
  };
  std::vector<unsigned char> seen(n, 0);
  const double cell_area = map_.resolution * map_.resolution;
  for (int seed = 0; seed < n; ++seed) {
    if (seen[seed] || !far(seed)) continue;
    std::vector<int> component{seed};
    seen[seed] = 1;
    for (std::size_t head = 0; head < component.size(); ++head) {
      const int i = component[head];
      const int col = i % map_.width, row = i / map_.width;
      for (int k = 0; k < 8; ++k) {
        const int c = col + dx[k], r = row + dy[k];
        if (c < 0 || r < 0 || c >= map_.width || r >= map_.height) continue;
        const int j = r * map_.width + c;
        if (!seen[j] && far(j)) { seen[j] = 1; component.push_back(j); }
      }
    }
    if (component.size() * cell_area < minimum_area_m2) continue;
    if (std::none_of(component.begin(), component.end(),
                     [&](int i) { return std::isfinite(seed_distance[i]); }))
      continue;
    std::sort(component.begin(), component.end(),
              [&](int a, int b) { return distance[a] > distance[b]; });
    for (const int i : component) {
      const int col = i % map_.width, row = i / map_.width;
      if (!ClearAt(col, row, kPlanningRadius())) continue;
      const auto [wx, wy] = CellCenter(col, row);
      ExplorationGoal goal;
      goal.x = goal.frontier_x = wx;
      goal.y = goal.frontier_y = wy;
      goal.information_gain = component.size() * cell_area;
      goals.push_back(goal);
      break;
    }
  }
  return goals;
}

std::size_t FrontierExplorer::RegionFrontierCells(double anchor_x, double anchor_y,
                                                  double radius_m) const {
  if (observation_map_.width <= 0 || observation_map_.height <= 0 ||
      observation_map_.resolution <= 0. || radius_m <= 0.) return 0;
  std::size_t count = 0;
  for (int row = 0; row < observation_map_.height; ++row) {
    for (int col = 0; col < observation_map_.width; ++col) {
      if (!RawFrontier(col, row)) continue;
      const auto x = observation_map_.origin_x + (col + .5) * observation_map_.resolution;
      const auto y = observation_map_.origin_y + (row + .5) * observation_map_.resolution;
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
