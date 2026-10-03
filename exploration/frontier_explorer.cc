#include "navigation2d/exploration/frontier_explorer.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>

namespace {

// Chassis half-width is 0.147 m. A 0.42 m opening leaves about 0.20 m from
// the centerline to each wall, which the 0.22 m planning radius can cross.
// The old 0.56 m / 0.28 m test kept those openings off the safe graph, so the
// viewpoint stayed on the near side and the same mouth was scheduled again.
constexpr double kNarrowMinWidth = .42;
constexpr double kNarrowMaxWidth = 1.40;
constexpr double kNarrowClearance = .20;
// The frontier cell is already the last free cell beside unknown. A 0.22 m
// floor left the goal in the corridor, and the 0.18 m arrival tolerance
// stopped the robot before a 0.50 m kitchen door. SafeViewpoint still keeps
// the pose off the walls.
constexpr double kCloseStandoff = .08;
// A successful doorway observation needs only a small local guard.  A wide
// permanent disk can hide a legitimate neighbouring doorway; room coverage
// is enforced by the active-region tour, not by suppressing that doorway.
constexpr double kVisitedViewpointRadius = .45;
constexpr double kVisitedFrontierRadius = .60;

// NBV utility: maximize newly observable unknown area per metre of travel.
// Region ordering is only a tie-breaker; it must never override information
// gain, otherwise the robot enters a nearby low-value room while a much larger
// unknown area is visible from another candidate.
double ViewpointScore(double information_gain, double travel, double standoff,
                      double gain_weight) {
  const double standoff_penalty = standoff > .30 ? 60.0 + 4.0 * standoff
                                                    : 4.0 * standoff;
  return gain_weight * 8.0 * information_gain / (0.45 + travel) -
         standoff_penalty;
}

}  // namespace

namespace navigation2d {

namespace {

int OpeningCells(const ExplorationGrid& grid, int col, int row, int step_col, int step_row) {
  if (grid.resolution <= 0.) return -1;
  const int limit = static_cast<int>(std::floor(kNarrowMaxWidth / grid.resolution)) + 1;
  const int min_cells = static_cast<int>(std::ceil(kNarrowMinWidth / grid.resolution));
  auto value_at = [&](int c, int r) -> std::int8_t {
    return grid.cells[static_cast<std::size_t>(r) * grid.width + c];
  };
  auto inside = [&](int c, int r) {
    return c >= 0 && r >= 0 && c < grid.width && r < grid.height;
  };
  if (!inside(col, row) || value_at(col, row) > 0) return -1;
  int count = 1;
  for (const int sign : {-1, 1}) {
    int c = col + sign * step_col;
    int r = row + sign * step_row;
    while (inside(c, r) && value_at(c, r) <= 0) {
      ++count;
      if (count > limit) return -1;
      c += sign * step_col;
      r += sign * step_row;
    }
    if (!inside(c, r) || value_at(c, r) <= 0) return -1;
  }
  return count >= min_cells ? count : -1;
}

}  // namespace

void OpenPassableOpenings(ExplorationGrid* grid) {
  if (grid == nullptr || grid->resolution <= 0. || grid->width <= 2 || grid->height <= 2 ||
      grid->cells.size() != static_cast<std::size_t>(grid->width) * grid->height) return;
  const int width = grid->width;
  const int height = grid->height;
  auto value_at = [&](int col, int row) -> std::int8_t {
    return grid->cells[static_cast<std::size_t>(row) * width + col];
  };
  std::vector<int> budget(grid->cells.size(), -1);
  auto seed_span = [&](bool horizontal, int fixed, int begin, int end) {
    const int span = end - begin;
    bool known_free = false;
    bool unknown = false;
    for (int cursor = begin; cursor < end; ++cursor) {
      const int col = horizontal ? cursor : fixed;
      const int row = horizontal ? fixed : cursor;
      const std::int8_t value = value_at(col, row);
      if (value == 0) known_free = true;
      else if (value < 0) unknown = true;
    }
    // A gap that is already entirely free is an observed corridor. Unknown
    // beyond it is the next room, not a missing piece of this opening.
    if (!known_free || !unknown) return;
    for (int cursor = begin; cursor < end; ++cursor) {
      const int col = horizontal ? cursor : fixed;
      const int row = horizontal ? fixed : cursor;
      if (value_at(col, row) >= 0) continue;
      const std::size_t index = static_cast<std::size_t>(row) * width + col;
      budget[index] = std::max(budget[index], span);
    }
  };
  for (int row = 0; row < height; ++row) {
    int col = 0;
    while (col < width) {
      if (value_at(col, row) <= 0) { ++col; continue; }
      const int begin = col + 1;
      int end = begin;
      while (end < width && value_at(end, row) <= 0) ++end;
      if (end < width && value_at(end, row) > 0 &&
          OpeningCells(*grid, begin, row, 1, 0) > 0)
        seed_span(true, row, begin, end);
      col = std::max(end, col + 1);
    }
  }
  for (int col = 0; col < width; ++col) {
    int row = 0;
    while (row < height) {
      if (value_at(col, row) <= 0) { ++row; continue; }
      const int begin = row + 1;
      int end = begin;
      while (end < height && value_at(col, end) <= 0) ++end;
      if (end < height && value_at(col, end) > 0 &&
          OpeningCells(*grid, col, begin, 0, 1) > 0)
        seed_span(false, col, begin, end);
      row = std::max(end, row + 1);
    }
  }
  std::deque<int> queue;
  for (std::size_t index = 0; index < budget.size(); ++index)
    if (budget[index] >= 0) queue.push_back(static_cast<int>(index));
  constexpr int cardinal_col[] = {-1, 1, 0, 0};
  constexpr int cardinal_row[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    if (budget[static_cast<std::size_t>(current)] == 0) continue;
    const int row = current / width;
    const int col = current % width;
    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + cardinal_col[direction];
      const int next_row = row + cardinal_row[direction];
      if (next_col < 0 || next_row < 0 || next_col >= width || next_row >= height) continue;
      const int next = next_row * width + next_col;
      if (value_at(next_col, next_row) >= 0) continue;
      if (OpeningCells(*grid, next_col, next_row, 1, 0) < 0 &&
          OpeningCells(*grid, next_col, next_row, 0, 1) < 0) continue;
      const int next_budget = budget[static_cast<std::size_t>(current)] - 1;
      if (next_budget <= budget[static_cast<std::size_t>(next)]) continue;
      budget[static_cast<std::size_t>(next)] = next_budget;
      queue.push_back(next);
    }
  }
  for (std::size_t index = 0; index < budget.size(); ++index)
    if (budget[index] >= 0) grid->cells[index] = 0;
}

FrontierExplorer::FrontierExplorer(FrontierExplorerConfig config)
    : config_(std::move(config)) {}

void FrontierExplorer::UpdateMap(ExplorationGrid map) {
  observation_map_ = map;
  unresolved_frontier_cells_ = CountRawFrontierCells();
  OpenPassableOpenings(&map);
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
  const int radius = static_cast<int>(std::ceil(clearance / map_.resolution));
  for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
    if (dx * dx + dy * dy > radius * radius) continue;
    const int x = col + dx, y = row + dy;
    if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) return false;
    // Unknown is unobserved space, not a jamb. It does not push a viewpoint
    // back from the opening; the viewpoint itself still has to be free.
    if (map_.cells[static_cast<std::size_t>(y) * map_.width + x] > 0) return false;
  }
  return true;
}

bool FrontierExplorer::SafeViewpoint(int col, int row) const {
  // The search graph stays on known free cells. Unknown may lie inside the
  // clearance disk, but the viewpoint itself is not placed in unknown space.
  if (!Free(col, row)) return false;
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
  const int narrow_min = static_cast<int>(std::ceil(kNarrowMinWidth / map_.resolution));
  const int narrow_max = static_cast<int>(std::floor(kNarrowMaxWidth / map_.resolution));
  const bool corridor = (horizontal >= narrow_min && horizontal <= narrow_max) !=
                        (vertical >= narrow_min && vertical <= narrow_max);
  // The configured footprint clearance is the contract with the executable
  // planner. The historical narrow constant is only a lower bound for tests
  // and legacy callers; production exploration config is .30 m.
  const double corridor_clearance = config_.footprint_clearance >= .30
      ? config_.footprint_clearance : kNarrowClearance;
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
    // A failed frontier becomes eligible again after the map has materially
    // expanded, since a new approach through a doorway may now be visible.
    return (!boundary || entry.persistent) &&
        (entry.persistent || known < entry.known_cells + entry.required_growth) &&
        std::hypot(x - entry.x, y - entry.y) < entry.radius;
  });
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

std::vector<ExplorationGoal> FrontierExplorer::SelectGoals(
    double robot_x, double robot_y) {
  raw_frontier_cells_ = 0;
  raw_frontier_clusters_ = 0;
  executable_frontier_goals_ = 0;
  candidate_frontier_goals_ = 0;
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height)) return {};
  std::vector<unsigned char> frontier(map_.cells.size(), 0), visited(map_.cells.size(), 0);
  std::vector<unsigned char> safe(map_.cells.size(), 0);
  std::vector<unsigned char> discovery_safe(map_.cells.size(), 0);
  for (int row = 0; row < map_.height; ++row) for (int col = 0; col < map_.width; ++col)
    safe[static_cast<std::size_t>(row) * map_.width + col] = SafeViewpoint(col, row);
  // Frontier discovery is deliberately less conservative than execution.
  // It may nominate a viewpoint near a doorway; StartNavigation still checks
  // the full inflated planning graph before any velocity is sent.
  const double discovery_clearance = std::clamp(
      config_.footprint_clearance - .12, .14, .22);
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
  for (int row = 0; row < map_.height; ++row) for (int col = 0; col < map_.width; ++col) {
    const int index = row * map_.width + col;
    // Use the raw mapper grid for the boundary.  `map_` may contain unknown
    // cells opened in a narrow passage solely to make the navigation graph
    // traversable; using it here erases the doorway's room frontier.
    if (travel_cells[index] >= 0 && RawFrontier(col, row)) frontier[index] = 1;
  }
  // Only boundaries reachable through the safe planning graph are actionable
  // frontiers. Unknown cells behind occupied walls, outside the observed
  // component, or stranded by a completed room must not block completion.
  std::size_t reachable_frontier_cells = 0;
  for (int row = 0; row < map_.height; ++row) {
    for (int col = 0; col < map_.width; ++col) {
      const int index = row * map_.width + col;
      if (travel_cells[index] >= 0 && RawFrontier(col, row))
        ++reachable_frontier_cells;
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
    // Keep short doorway boundaries: stability filtering removes one-frame
    // sensor speckles, while a hard component-size cut discards real maze
    // branches.  A 1--5 cell component is retained only when a full-clearance
    // viewpoint is already beside it. The narrow-passage exception is not
    // used here: an open-room speck shortens one free run and would otherwise
    // look like a doorway. The stability observer still rejects one-frame
    // lidar speckles.
    if (cluster.size() < static_cast<std::size_t>(config_.minimum_frontier_cells)) {
      bool doorway_component = false;
      if (!cluster.empty()) {
        for (const int cell : cluster) {
          const int cell_row = cell / map_.width, cell_col = cell % map_.width;
          for (int dy = -2; dy <= 2 && !doorway_component; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
              if (ClearAt(cell_col + dx, cell_row + dy, discovery_clearance)) {
                doorway_component = true;
                goto doorway_viewpoint_found;
              }
            }
          }
        }
      }
      doorway_viewpoint_found:
      // Once the map is mature, a 1--5 cell component is almost always a
      // residual ray/raster fragment from an already observed boundary. Keep
      // the doorway exception only during the initial map phase; real narrow
      // entrances are represented by their larger connected boundary after
      // the first scan through them.
      if (!doorway_component || KnownCells() >= 10000) continue;
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
  const int search = static_cast<int>(std::ceil(config_.maximum_standoff / map_.resolution));
    const auto narrow_viewpoint = [&](int col, int row) {
      const int probe = static_cast<int>(std::ceil(1.4 / map_.resolution));
      int horizontal = 0, vertical = 0;
      for (int x = col; x >= 0 && x > col - probe && Free(x, row); --x) ++horizontal;
      for (int x = col + 1; x < map_.width && x <= col + probe && Free(x, row); ++x) ++horizontal;
      for (int y = row; y >= 0 && y > row - probe && Free(col, y); --y) ++vertical;
      for (int y = row + 1; y < map_.height && y <= row + probe && Free(col, y); ++y) ++vertical;
      const int narrow_min = static_cast<int>(std::ceil(kNarrowMinWidth / map_.resolution));
      const int narrow_max = static_cast<int>(std::floor(kNarrowMaxWidth / map_.resolution));
      return (horizontal >= narrow_min && horizontal <= narrow_max) !=
             (vertical >= narrow_min && vertical <= narrow_max);
    };
    std::optional<ExplorationGoal> best;
    for (int row = std::max(0, static_cast<int>(mean_row) - search);
         row <= std::min(map_.height - 1, static_cast<int>(mean_row) + search); ++row) {
      for (int col = std::max(0, static_cast<int>(mean_col) - search);
           col <= std::min(map_.width - 1, static_cast<int>(mean_col) + search); ++col) {
        const int viewpoint_index = row * map_.width + col;
        if ((!safe[viewpoint_index] && !discovery_safe[viewpoint_index]) ||
            travel_cells[viewpoint_index] < 0) continue;
        const auto [x, y] = CellCenter(col, row);
        if (Blacklisted(x, y)) continue;
        const double standoff = std::hypot(x - frontier_x, y - frontier_y);
        if (standoff > config_.maximum_standoff ||
            (standoff < config_.minimum_standoff &&
             !(narrow_viewpoint(col, row) && standoff >= kCloseStandoff))) continue;
        if (!HasFreeLineOfSight(col, row, representative % map_.width,
                                representative / map_.width)) continue;
        const double travel = travel_cells[viewpoint_index] * map_.resolution;
        const double information_gain = std::max(
            visible_unknown_gain(col, row),
            static_cast<double>(cluster.size()) * map_.resolution * map_.resolution);
        ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                  frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                  information_gain,
                                  ViewpointScore(information_gain, travel, standoff, 2.)};
        // A relaxed doorway viewpoint is discovery-only. Prefer a full
        // footprint-safe viewpoint whenever one exists; the planner must not
        // repeatedly reject a relaxed candidate before trying the safe one.
        if (!safe[viewpoint_index]) candidate.score -= 1000.;
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
          if ((!safe[viewpoint_index] && !discovery_safe[viewpoint_index]) ||
              travel_cells[viewpoint_index] < 0) continue;
          const auto [x, y] = CellCenter(col, row);
          if (Blacklisted(x, y)) continue;
          const double standoff = std::hypot(x - frontier_x, y - frontier_y);
          if (standoff > config_.maximum_standoff ||
              (standoff < config_.minimum_standoff &&
               !(narrow_viewpoint(col, row) && standoff >= kCloseStandoff))) continue;
          const bool sees_component = std::any_of(cluster.begin(), cluster.end(), [&](int cell) {
            return HasFreeLineOfSight(col, row, cell % map_.width, cell / map_.width);
          });
          if (!sees_component) continue;
          const double travel = travel_cells[viewpoint_index] * map_.resolution;
          const double information_gain = std::max(
              visible_unknown_gain(col, row),
              static_cast<double>(cluster.size()) * map_.resolution * map_.resolution);
          ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                    frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                    information_gain,
                                    ViewpointScore(information_gain, travel, standoff, 1.5)};
          if (!safe[viewpoint_index]) candidate.score -= 1000.;
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
          if ((!safe[viewpoint_index] && !discovery_safe[viewpoint_index]) ||
              travel_cells[viewpoint_index] < 0 ||
              !narrow_viewpoint(col, row)) continue;
          const auto [x, y] = CellCenter(col, row);
          if (Blacklisted(x, y)) continue;
          const double standoff = std::hypot(x - frontier_x, y - frontier_y);
          if (standoff < kCloseStandoff || standoff > 1.5) continue;
          const double travel = travel_cells[viewpoint_index] * map_.resolution;
          const double information_gain = std::max(
              visible_unknown_gain(col, row),
              static_cast<double>(cluster.size()) * map_.resolution * map_.resolution);
          ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                    frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                    information_gain,
                                    ViewpointScore(information_gain, travel, standoff, 1.)};
          if (!safe[viewpoint_index]) candidate.score -= 1000.;
          if (!best || candidate.score > best->score) best = candidate;
        }
      }
    }
    if (!best) {
      // Last-resort approach pose in already-observed open space: choose a
      // 0.40 m-clear cell the robot can reach by the ordinary free-space
      // graph (wide aisle / gap), then acquire the missing LOS after arrival.
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
            if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0) continue;
            const auto [x, y] = CellCenter(col, row);
            if (Blacklisted(x, y)) continue;
            const auto [target_x, target_y] = CellCenter(target_col, target_row);
            const double standoff = std::hypot(x - target_x, y - target_y);
            if (standoff < kCloseStandoff || standoff > kApproachRadius) continue;
            const double travel = travel_cells[viewpoint_index] * map_.resolution;
            const double information_gain = std::max(
                visible_unknown_gain(col, row),
                static_cast<double>(cluster.size()) * map_.resolution * map_.resolution);
            ExplorationGoal candidate{x, y, std::atan2(target_y - y, target_x - x),
                                      frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                      information_gain,
                                      ViewpointScore(information_gain, travel, standoff, .8)};
            if (!best || candidate.score > best->score) best = candidate;
          }
        }
      }
    }
    if (best) goals.push_back(*best);
  }
  std::sort(goals.begin(), goals.end(),
            [](const auto& a, const auto& b) { return a.score > b.score; });
  candidate_frontier_goals_ = goals.size();
  auto stable = StableGoals(std::move(goals));
  executable_frontier_goals_ = stable.size();
  return stable;
}

std::optional<ExplorationGoal> FrontierExplorer::SelectLeftoverApproach(
    double robot_x, double robot_y) {
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height))
    return std::nullopt;
  std::vector<unsigned char> frontier(map_.cells.size(), 0);
  std::vector<unsigned char> safe(map_.cells.size(), 0);
  for (int row = 0; row < map_.height; ++row) for (int col = 0; col < map_.width; ++col)
    safe[static_cast<std::size_t>(row) * map_.width + col] = SafeViewpoint(col, row);
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
  if (start < 0) return std::nullopt;
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
  if (raw_frontier_cells_ == 0) return std::nullopt;
  // Leftover free/unknown specks are often smaller than
  // minimum_frontier_cells, so the exploring tour drops them.  The pocket
  // they belong to is usually still reachable through ordinary open space.
  // Place a normal 0.40 m viewpoint on that open graph; the planner takes
  // the wide route, it does not squeeze a 1.2 m end gap.
  constexpr double kLeftoverRadius = 6.0;
  const int leftover_search = static_cast<int>(std::ceil(kLeftoverRadius / map_.resolution));
  std::optional<ExplorationGoal> leftover;
  for (int index = 0; index < map_.width * map_.height; ++index) {
    if (!frontier[index]) continue;
    const int target_row = index / map_.width, target_col = index % map_.width;
    const auto [frontier_x, frontier_y] = CellCenter(target_col, target_row);
    if (Blacklisted(frontier_x, frontier_y, true)) continue;
    for (int row = std::max(0, target_row - leftover_search);
         row <= std::min(map_.height - 1, target_row + leftover_search); ++row) {
      for (int col = std::max(0, target_col - leftover_search);
           col <= std::min(map_.width - 1, target_col + leftover_search); ++col) {
        const int viewpoint_index = row * map_.width + col;
        if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0) continue;
        const auto [x, y] = CellCenter(col, row);
        if (Blacklisted(x, y)) continue;
        const double standoff = std::hypot(x - frontier_x, y - frontier_y);
        if (standoff < kCloseStandoff || standoff > kLeftoverRadius) continue;
        const double travel = travel_cells[viewpoint_index] * map_.resolution;
        ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                  frontier_x, frontier_y, 1, map_.resolution,
                                  ViewpointScore(map_.resolution, travel, standoff, .8), true};
        if (!leftover || candidate.score > leftover->score) leftover = candidate;
      }
    }
  }
  return leftover;
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

std::vector<ExplorationGoal> FrontierExplorer::BuildTour(
    double robot_x, double robot_y) {
  auto remaining = SelectGoals(robot_x, robot_y);
  if (remaining.empty()) return {};

  // `safe` is the vertex set of the free-space connectivity graph.  Edges are
  // the four cardinal grid neighbours, so BFS distance is the path length the
  // robot can actually traverse, not a straight-line proxy through walls.
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
    // A viewpoint always has an exact safe cell.  The small local search only
    // accommodates a robot pose that lies on a grid boundary.
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
  const auto distance_field = [&](int start) {
    std::vector<int> distance(map_.cells.size(), -1);
    if (start < 0) return distance;
    std::deque<int> queue{start};
    distance[start] = 0;
    constexpr int dx[] = {-1, 1, 0, 0};
    constexpr int dy[] = {0, 0, -1, 1};
    while (!queue.empty()) {
      const int current = queue.front(); queue.pop_front();
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
    return distance;
  };

  std::vector<ExplorationGoal> tour;
  int current = nearest_safe(robot_x, robot_y);
  // Frontier components are observation tasks, not independent destinations.
  // First choose a worthwhile reachable region, then exhaust its adjacent
  // components before paying for another cross-map transit.  This is the
  // region-growing counterpart to frontier utility planning: it retains an
  // information/travel objective between regions while preventing a global
  // greedy sort from interleaving opposite sides of the same warehouse.
  std::vector<int> region_distance;
  const int region_radius_cells = std::max(
      1, static_cast<int>(std::ceil(3. * config_.maximum_standoff / map_.resolution)));
  while (!remaining.empty() && current >= 0) {
    const auto distances = distance_field(current);
    auto next = remaining.end();
    int best_distance = std::numeric_limits<int>::max();
    double best_utility = -std::numeric_limits<double>::infinity();
    // Keep the tour local, but retain NBV discrimination inside that local
    // window. A fixed one-metre allowance avoids a far candidate winning over
    // a nearby doorway while still allowing a clearly higher-gain viewpoint
    // to win when it is only a short detour away.
    int nearest_distance = std::numeric_limits<int>::max();
    for (const auto& candidate : remaining) {
      const int viewpoint = nearest_safe(candidate.x, candidate.y);
      if (viewpoint < 0 || distances[viewpoint] < 0) continue;
      if (!region_distance.empty() &&
          (region_distance[viewpoint] < 0 ||
           region_distance[viewpoint] > region_radius_cells)) continue;
      nearest_distance = std::min(nearest_distance, distances[viewpoint]);
    }
    const int local_distance_limit = nearest_distance == std::numeric_limits<int>::max()
        ? nearest_distance
        : nearest_distance + std::max(1, static_cast<int>(std::ceil(1.0 / map_.resolution)));
    for (auto candidate = remaining.begin(); candidate != remaining.end(); ++candidate) {
      const int viewpoint = nearest_safe(candidate->x, candidate->y);
      if (viewpoint < 0 || distances[viewpoint] < 0) continue;
      // After entering a region, only serve frontiers connected to the same
      // local free-space neighbourhood.  If none remain, the second pass
      // below selects the next high-value region globally.
      if (!region_distance.empty() &&
          (region_distance[viewpoint] < 0 ||
           region_distance[viewpoint] > region_radius_cells))
        continue;
      const int distance = distances[viewpoint];
      if (distance > local_distance_limit) continue;
      // The tour is a local coverage walk. The distance window prevents a
      // remote high-gain boundary from skipping a nearby doorway; within the
      // window, NBV gain per metre is the decision criterion.
      const double travel_m = distance * map_.resolution;
      const double utility = 1.5 * candidate->information_gain - .65 * travel_m;
      if (utility > best_utility + 1e-9 ||
          (std::abs(utility - best_utility) <= 1e-9 &&
           (distance < best_distance ||
            (distance == best_distance &&
             (next == remaining.end() || candidate->frontier_cells > next->frontier_cells))))) {
        next = candidate;
        best_distance = distance;
        best_utility = utility;
      }
    }
    if (next == remaining.end() && !region_distance.empty()) {
      // The committed region is complete.  Deliberately reset the locality
      // constraint rather than silently alternating between far components.
      region_distance.clear();
      continue;
    }
    if (next == remaining.end()) break;
    current = nearest_safe(next->x, next->y);
    // The first task after a global selection becomes the anchor for a
    // contiguous coverage sweep.  BFS makes this a free-space region, not a
    // Euclidean circle leaking through a shelf or wall.
    if (region_distance.empty()) region_distance = distance_field(current);
    tour.push_back(*next);
    remaining.erase(next);
  }
  return tour;
}

std::optional<ExplorationGoal> FrontierExplorer::SelectEntryProbe(
    const ExplorationGoal& frontier_goal, double robot_x, double robot_y,
    double minimum_depth_m, double maximum_depth_m) const {
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height) ||
      minimum_depth_m <= 0. || maximum_depth_m < minimum_depth_m)
    return std::nullopt;

  const double approach_x = frontier_goal.frontier_x - frontier_goal.x;
  const double approach_y = frontier_goal.frontier_y - frontier_goal.y;
  const double approach_length = std::hypot(approach_x, approach_y);
  if (approach_length < .05) return std::nullopt;
  const double direction_x = approach_x / approach_length;
  const double direction_y = approach_y / approach_length;
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
      if (!SafeViewpoint(col, row)) continue;
      const double distance = std::hypot(col - robot_col, row - robot_row);
      if (distance < start_distance) {
        start_distance = distance;
        start = row * map_.width + col;
      }
    }
  }
  if (start < 0) return std::nullopt;

  // The entry probe is allowed to use only the planning copy. OpenPassable-
  // Openings() may have converted a narrow, partly observed doorway to free,
  // while the unknown room beyond it remains blocked.
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
      if (distance[next] >= 0 || !SafeViewpoint(next_col, next_row)) continue;
      distance[next] = distance[current] + 1;
      queue.push_back(next);
    }
  }

  std::optional<ExplorationGoal> best;
  double best_depth = -std::numeric_limits<double>::infinity();
  int best_travel = std::numeric_limits<int>::max();
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
      if (distance[index] < 0 || !SafeViewpoint(col, row)) continue;
      const auto [x, y] = CellCenter(col, row);
      const double relative_x = x - frontier_goal.frontier_x;
      const double relative_y = y - frontier_goal.frontier_y;
      const double depth = relative_x * direction_x + relative_y * direction_y;
      const double lateral = std::abs(relative_x * direction_y - relative_y * direction_x);
      if (depth < minimum_depth_m || depth > maximum_depth_m || lateral > .38)
        continue;
      // Do not select a pose that is merely on the approach side of the
      // frontier or an unrelated free cell behind a nearby wall.
      if (std::hypot(relative_x, relative_y) > maximum_depth_m + .20) continue;
      if (depth > best_depth + .01 ||
          (std::abs(depth - best_depth) <= .01 && distance[index] < best_travel)) {
        best_depth = depth;
        best_travel = distance[index];
        ExplorationGoal probe = frontier_goal;
        probe.x = x;
        probe.y = y;
        probe.yaw = std::atan2(direction_y, direction_x);
        probe.score = depth - .01 * static_cast<double>(distance[index]);
        best = probe;
      }
    }
  }
  return best;
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
                                     bool region_complete) {
  // Failure describes an attempted observation pose, not an entire doorway.
  // A distant standoff must never create a room-sized exclusion disk around
  // its frontier; other safe viewpoints on the same boundary remain useful.
  const std::size_t known = KnownCells();
  if (succeeded) {
    blacklist_.push_back({goal.frontier_x, goal.frontier_y,
                          kVisitedFrontierRadius, known, 20000u, true});
    blacklist_.push_back({goal.x, goal.y, kVisitedViewpointRadius,
                          known, 20000u, true});
    if (region_complete) ++completed_goals_;
  } else {
    blacklist_.push_back({goal.x, goal.y,
                          std::max(map_.resolution, config_.footprint_clearance),
                          known, 600u, false});
    ++failed_goals_;
  }
}

void FrontierExplorer::RecordAttempt(const ExplorationGoal& goal, bool succeeded) {
  RecordAttempt(goal, succeeded, true);
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
  const int search = static_cast<int>(std::ceil(1.0 / map_.resolution));
  int start = -1;
  double nearest = std::numeric_limits<double>::infinity();
  for (int row = std::max(0, robot_row - search);
       row <= std::min(map_.height - 1, robot_row + search); ++row) {
    for (int col = std::max(0, robot_col - search);
         col <= std::min(map_.width - 1, robot_col + search); ++col) {
      if (!SafeViewpoint(col, row)) continue;
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
          !SafeViewpoint(next_col, next_row))
        continue;
      visited[static_cast<std::size_t>(next)] = 1;
      queue.push_back(next);
    }
  }
  return reachable_unknown;
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
