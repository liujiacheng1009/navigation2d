#include "navigation2d/exploration/frontier_explorer.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>

namespace navigation2d {

FrontierExplorer::FrontierExplorer(FrontierExplorerConfig config)
    : config_(std::move(config)) {}

void FrontierExplorer::UpdateMap(ExplorationGrid map) {
  map_ = std::move(map);
}

bool FrontierExplorer::Free(int col, int row) const {
  return col >= 0 && row >= 0 && col < map_.width && row < map_.height &&
      map_.cells[static_cast<std::size_t>(row) * map_.width + col] == 0;
}

bool FrontierExplorer::SafeViewpoint(int col, int row) const {
  const int radius = static_cast<int>(std::ceil(config_.footprint_clearance / map_.resolution));
  for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
    if (dx * dx + dy * dy <= radius * radius && !Free(col + dx, row + dy)) return false;
  }
  return true;
}

std::pair<double, double> FrontierExplorer::CellCenter(int col, int row) const {
  return {map_.origin_x + (col + .5) * map_.resolution,
          map_.origin_y + (row + .5) * map_.resolution};
}

bool FrontierExplorer::Blacklisted(double x, double y) const {
  const std::size_t known = KnownCells();
  return std::any_of(blacklist_.begin(), blacklist_.end(), [&](const auto& entry) {
    // A failed frontier becomes eligible again after the map has materially
    // expanded, since a new approach through a doorway may now be visible.
    return known < entry.known_cells + 600 &&
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
  if (map_.width <= 2 || map_.height <= 2 || map_.resolution <= 0. ||
      map_.cells.size() != static_cast<std::size_t>(map_.width * map_.height)) return {};
  std::vector<unsigned char> frontier(map_.cells.size(), 0), visited(map_.cells.size(), 0);
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
  for (int row = 1; row + 1 < map_.height; ++row) for (int col = 1; col + 1 < map_.width; ++col) {
    const int index = row * map_.width + col;
    if (map_.cells[index] == 0 &&
        (map_.cells[index - 1] < 0 || map_.cells[index + 1] < 0 ||
         map_.cells[index - map_.width] < 0 || map_.cells[index + map_.width] < 0))
      frontier[index] = 1;
  }
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
    if (cluster.size() < static_cast<std::size_t>(config_.minimum_frontier_cells)) continue;
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
    if (Blacklisted(frontier_x, frontier_y)) continue;
    const int search = static_cast<int>(std::ceil(config_.maximum_standoff / map_.resolution));
    std::optional<ExplorationGoal> best;
    for (int row = std::max(0, static_cast<int>(mean_row) - search);
         row <= std::min(map_.height - 1, static_cast<int>(mean_row) + search); ++row) {
      for (int col = std::max(0, static_cast<int>(mean_col) - search);
           col <= std::min(map_.width - 1, static_cast<int>(mean_col) + search); ++col) {
        const int viewpoint_index = row * map_.width + col;
        if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0) continue;
        const auto [x, y] = CellCenter(col, row);
        const double standoff = std::hypot(x - frontier_x, y - frontier_y);
        if (standoff < config_.minimum_standoff || standoff > config_.maximum_standoff) continue;
        if (!HasFreeLineOfSight(col, row, representative % map_.width,
                                representative / map_.width)) continue;
        const double travel = travel_cells[viewpoint_index] * map_.resolution;
        const double information_gain = static_cast<double>(cluster.size()) * map_.resolution;
        ExplorationGoal candidate{x, y, std::atan2(frontier_y - y, frontier_x - x),
                                  frontier_x, frontier_y, static_cast<int>(cluster.size()),
                                  information_gain,
                                  2. * std::log1p(information_gain) -
                                      .9 * travel - .25 * standoff};
        if (!best || candidate.score > best->score) best = candidate;
      }
    }
    if (best) goals.push_back(*best);
  }
  std::sort(goals.begin(), goals.end(),
            [](const auto& a, const auto& b) { return a.score > b.score; });
  return StableGoals(std::move(goals));
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
          return std::hypot(first.x - goal.frontier_x, first.y - goal.frontier_y) <
                 std::hypot(second.x - goal.frontier_x, second.y - goal.frontier_y);
        });
    if (track == frontier_tracks_.end() ||
        std::hypot(track->x - goal.frontier_x, track->y - goal.frontier_y) >
            config_.frontier_stability_radius) {
      frontier_tracks_.push_back({goal.frontier_x, goal.frontier_y, 1, stability_observation_});
      if (config_.required_frontier_observations <= 1) stable.push_back(goal);
      continue;
    }
    if (track->last_observation != stability_observation_) ++track->observations;
    track->x = goal.frontier_x;
    track->y = goal.frontier_y;
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
      // This is the potential-minus-gain cost used by the established
      // explore_lite frontier search: prefer reachable information gain but
      // retain a linear travel penalty. Using a ratio here made the policy
      // repeatedly switch to tiny nearby boundaries and produced zig-zags.
      const double travel_m = distance * map_.resolution;
      // This is an informative-path objective, not a nearest-frontier rule.
      // A stable, long visible boundary is worth one deliberate transit;
      // repeatedly serving neighbouring one-cell fragments is not.  The
      // cost term remains present so this is still a feasible online tour.
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

bool FrontierExplorer::GoalStillFrontier(const ExplorationGoal& goal) const {
  if (map_.resolution <= 0. || map_.width <= 0 || map_.height <= 0) return false;
  const int col = static_cast<int>(std::floor(
      (goal.frontier_x - map_.origin_x) / map_.resolution));
  const int row = static_cast<int>(std::floor(
      (goal.frontier_y - map_.origin_y) / map_.resolution));
  if (!Free(col, row)) return false;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  for (int index = 0; index < 4; ++index) {
    const int x = col + dx[index], y = row + dy[index];
    if (x >= 0 && y >= 0 && x < map_.width && y < map_.height &&
        map_.cells[static_cast<std::size_t>(y) * map_.width + x] < 0) return true;
  }
  return false;
}

bool FrontierExplorer::GoalRegionStillFrontier(const ExplorationGoal& goal) const {
  if (GoalStillFrontier(goal) || map_.resolution <= 0.) return GoalStillFrontier(goal);
  const int center_col = static_cast<int>(std::floor(
      (goal.frontier_x - map_.origin_x) / map_.resolution));
  const int center_row = static_cast<int>(std::floor(
      (goal.frontier_y - map_.origin_y) / map_.resolution));
  // The association radius is deliberately smaller than the viewpoint
  // standoff. It follows a shifted sensor boundary but cannot jump across a
  // shelf to an unrelated frontier on its far side.
  const int radius = static_cast<int>(std::ceil(.35 / map_.resolution));
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  for (int row = std::max(0, center_row - radius);
       row <= std::min(map_.height - 1, center_row + radius); ++row) {
    for (int col = std::max(0, center_col - radius);
         col <= std::min(map_.width - 1, center_col + radius); ++col) {
      if (std::hypot(col - center_col, row - center_row) > radius || !Free(col, row)) continue;
      for (int direction = 0; direction < 4; ++direction) {
        const int x = col + dx[direction], y = row + dy[direction];
        if (x >= 0 && y >= 0 && x < map_.width && y < map_.height &&
            map_.cells[static_cast<std::size_t>(y) * map_.width + x] < 0)
          return true;
      }
    }
  }
  return false;
}

void FrontierExplorer::RecordAttempt(const ExplorationGoal& goal, bool succeeded) {
  // Cool down both resolved and failed boundaries for the current map state.
  // The entry automatically expires after sufficient new observations, so a
  // doorway can expose a genuinely new nearby frontier without target churn.
  // Suppress the entire failed frontier component, rather than only its
  // representative cell. Otherwise the next map update emits another point
  // on the same wall and causes an identical recovery loop.
  const double component_radius = std::max(
      config_.blacklist_radius,
      std::sqrt(static_cast<double>(std::max(1, goal.frontier_cells))) * map_.resolution);
  blacklist_.push_back({goal.frontier_x, goal.frontier_y, component_radius, KnownCells()});
  if (succeeded) {
    ++completed_goals_;
  } else {
    ++failed_goals_;
  }
}

std::size_t FrontierExplorer::KnownCells() const {
  return static_cast<std::size_t>(std::count_if(
      map_.cells.begin(), map_.cells.end(), [](std::int8_t value) { return value >= 0; }));
}

bool FrontierExplorer::CompletionEligible(std::size_t initial_known_cells) const {
  const auto growth = std::max(config_.minimum_known_cell_growth,
      static_cast<std::size_t>(std::ceil(initial_known_cells * config_.minimum_known_growth_ratio)));
  return completed_goals_ >= config_.minimum_completed_goals &&
      KnownCells() >= initial_known_cells + growth;
}

}  // namespace navigation2d
