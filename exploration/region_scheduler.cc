#include "navigation2d/exploration/region_scheduler.h"
#include "navigation2d/exploration/frontier_explorer.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <sstream>
#include <tuple>

namespace navigation2d {
namespace {
constexpr int dx[] = {-1, 1, 0, 0};
constexpr int dy[] = {0, 0, -1, 1};
const char* StateName(RegionState s) {
  switch (s) {
    case RegionState::kPending: return "PENDING";
    case RegionState::kActive: return "ACTIVE";
    case RegionState::kWaiting: return "WAITING_MAP";
    case RegionState::kBlocked: return "BLOCKED";
    case RegionState::kComplete: return "COMPLETE";
  }
  return "UNKNOWN";
}
}
int RegionScheduler::Cell(double x, double y) const {
  if (resolution_ <= 0. || !std::isfinite(x) || !std::isfinite(y)) return -1;
  const int c = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int r = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return c < 0 || r < 0 || c >= width_ || r >= height_ ? -1 : r * width_ + c;
}
RegionId RegionScheduler::RegionAt(double x, double y) const {
  const int i = Cell(x, y);
  return i < 0 || static_cast<std::size_t>(i) >= labels_.size() ? 0 : labels_[i];
}

void RegionScheduler::Update(const ExplorationGrid& g, double clearance) {
  if (g.width <= 0 || g.height <= 0 || g.resolution <= 0. ||
      g.cells.size() != static_cast<std::size_t>(g.width) * g.height) return;
  const int n = static_cast<int>(g.cells.size());
  auto neighbor = [&](int i, int d) {
    const int c = i % g.width + dx[d], r = i / g.width + dy[d];
    return c < 0 || r < 0 || c >= g.width || r >= g.height ? -1 : r * g.width + c;
  };
  // Erode observed free space at a chassis-derived scale, then geodesically
  // grow the cores back to the actual walls/unknown boundary. A narrow neck
  // becomes an edge between cores, not a circular "room" around a goal.
  // Small components without a core still get a region (no area threshold).
  std::vector<int> distance(n, n), components(n, 0);
  std::deque<int> queue;
  for (int i = 0; i < n; ++i) {
    if (g.cells[i] != 0) { distance[i] = 0; queue.push_back(i); }
    else if (i % g.width == 0 || i % g.width == g.width-1 ||
             i / g.width == 0 || i / g.width == g.height-1) {
      distance[i] = 1; queue.push_back(i);
    }
  }
  while (!queue.empty()) {
    const int i = queue.front(); queue.pop_front();
    for (int d = 0; d < 4; ++d) {
      const int j = neighbor(i, d);
      if (j >= 0 && distance[j] > distance[i] + 1) {
        distance[j] = distance[i] + 1; queue.push_back(j);
      }
    }
  }
  const int erosion = std::max(1, static_cast<int>(std::ceil(2. * clearance / g.resolution)));
  int count = 0;
  for (int seed = 0; seed < n; ++seed) {
    if (g.cells[seed] != 0 || distance[seed] <= erosion || components[seed]) continue;
    components[seed] = ++count; queue.push_back(seed);
    while (!queue.empty()) {
      const int i = queue.front(); queue.pop_front();
      for (int d = 0; d < 4; ++d) {
        const int j = neighbor(i, d);
        if (j >= 0 && g.cells[j] == 0 && distance[j] > erosion && !components[j]) {
          components[j] = count; queue.push_back(j);
        }
      }
    }
  }
  // Keep the eroded cores separate.  The previous implementation grew each
  // core through every free cell, which reconnected rooms through a narrow
  // doorway and made the scheduler lose its DFS context.  The remaining
  // corridor cells are assigned below by a multi-source wavefront, so they
  // still belong to a region without becoming a bridge between regions.
  for (int i = 0; i < n; ++i) if (components[i]) queue.push_back(i);
  auto grow_core = [&]() {
    while (!queue.empty()) {
      const int i = queue.front(); queue.pop_front();
      for (int d = 0; d < 4; ++d) {
        const int j = neighbor(i, d);
        if (j >= 0 && g.cells[j] == 0 && distance[j] > erosion && !components[j]) {
          components[j] = components[i]; queue.push_back(j);
        }
      }
    }
  };
  grow_core();
  if (count == 0) {
    // A map that contains only corridors still needs one stable region.  Do
    // not manufacture one region per cell when there is no eroded core.
    for (int seed = 0; seed < n; ++seed) {
      if (g.cells[seed] != 0) continue;
      if (components[seed]) continue;
      components[seed] = ++count;
      queue.push_back(seed);
      while (!queue.empty()) {
        const int i = queue.front(); queue.pop_front();
        for (int d = 0; d < 4; ++d) {
          const int j = neighbor(i, d);
          if (j >= 0 && g.cells[j] == 0 && !components[j]) {
            components[j] = count; queue.push_back(j);
          }
        }
      }
      break;
    }
  } else {
    // Assign non-core free cells to the nearest core.  A tie is deterministic
    // and the label boundary remains an explicit graph edge for BFS fallback.
    std::deque<int> wave;
    for (int i = 0; i < n; ++i) if (components[i]) wave.push_back(i);
    while (!wave.empty()) {
      const int i = wave.front(); wave.pop_front();
      for (int d = 0; d < 4; ++d) {
        const int j = neighbor(i, d);
        if (j >= 0 && g.cells[j] == 0 && !components[j]) {
          components[j] = components[i]; wave.push_back(j);
        }
      }
    }
  }

  // Match old/new partitions by overlapping WORLD cells, not local indices
  // or frontier centroids. Map padding/origin changes preserve identity.
  std::map<std::pair<int, RegionId>, std::size_t> overlaps;
  for (int i = 0; i < n; ++i) if (components[i]) {
    const auto old = RegionAt(g.origin_x + (i % g.width + .5) * g.resolution,
                              g.origin_y + (i / g.width + .5) * g.resolution);
    if (old) ++overlaps[{components[i], old}];
  }
  std::vector<std::tuple<std::size_t, int, RegionId>> matches;
  for (auto [key, size] : overlaps) matches.emplace_back(size, key.first, key.second);
  std::sort(matches.rbegin(), matches.rend());
  std::vector<RegionId> ids(count + 1, 0), parents(count + 1, 0);
  std::set<RegionId> used;
  for (auto [size, c, old] : matches) {
    (void)size;
    if (!parents[c]) parents[c] = old;
    if (!ids[c] && !used.count(old)) { ids[c] = old; used.insert(old); }
  }
  for (int c = 1; c <= count; ++c) if (!ids[c]) {
    ids[c] = next_id_++;
    RegionRecord record;
    record.id = ids[c]; record.parent = parents[c];
    regions_[ids[c]] = record;
  }
  for (auto& [id, r] : regions_) {
    (void)id;
    r.visible = false; r.free_cells = r.frontier_cells = r.candidates = 0;
    r.neighbors.clear();
  }
  std::map<RegionId, std::uint64_t> hashes;
  std::vector<RegionId> labels(n, 0);
  for (int i = 0; i < n; ++i) if (components[i]) labels[i] = ids[components[i]];
  for (int i = 0; i < n; ++i) if (labels[i]) {
    auto& r = regions_.at(labels[i]);
    r.visible = true; ++r.free_cells;
    bool frontier = false;
    for (int d = 0; d < 4; ++d) {
      const int j = neighbor(i, d);
      if (j >= 0 && g.cells[j] < 0) frontier = true;
      if (j >= 0 && labels[j] && labels[j] != labels[i]) r.neighbors.insert(labels[j]);
    }
    if (frontier) ++r.frontier_cells;
    // Commutative world-cell digest unaffected by origin padding / iteration
    // order. No global known-cell growth can complete or unblock a room.
    const auto wx = static_cast<std::int64_t>(std::llround(
        (g.origin_x + (i % g.width + .5) * g.resolution) / g.resolution));
    const auto wy = static_cast<std::int64_t>(std::llround(
        (g.origin_y + (i / g.width + .5) * g.resolution) / g.resolution));
    std::uint64_t h = static_cast<std::uint64_t>(wx) * 0x9e3779b97f4a7c15ULL;
    h ^= static_cast<std::uint64_t>(wy) * 0xbf58476d1ce4e5b9ULL;
    h ^= frontier ? 0x94d049bb133111ebULL : 0;
    hashes[r.id] += h ^ (h >> 31);
  }
  for (auto& [id, r] : regions_) if (r.visible) {
    const bool changed = r.evidence != hashes[id];
    r.stable_empty_updates = r.frontier_cells ? 0 :
        (changed ? 1 : std::min(2u, r.stable_empty_updates + 1));
    if (r.stable_empty_updates >= 2) r.state = RegionState::kComplete;
    else if (changed || r.state == RegionState::kComplete) r.state = RegionState::kPending;
    r.evidence = hashes[id];
  }
  // Merged/removed contexts must not permanently lock the stack. A newly
  // split child is a fresh pending region, never inherits COMPLETE.
  stack_.erase(std::remove_if(stack_.begin(), stack_.end(), [&](RegionId id) {
    return !regions_.at(id).visible;
  }), stack_.end());
  width_ = g.width; height_ = g.height; resolution_ = g.resolution;
  origin_x_ = g.origin_x; origin_y_ = g.origin_y; labels_ = std::move(labels);
}

void RegionScheduler::Enter(double x, double y) {
  const RegionId id = RegionAt(x, y);
  if (!id) return;
  last_region_ = id;
  if (regions_.at(id).state == RegionState::kComplete) return;
  if (stack_.empty() || stack_.back() != id) {
    auto existing = std::find(stack_.begin(), stack_.end(), id);
    if (existing != stack_.end()) stack_.erase(existing + 1, stack_.end());
    else stack_.push_back(id);
  }
  regions_.at(id).state = RegionState::kActive;
}

void RegionScheduler::Enter(const ExplorationGoal& goal) {
  if (!goal.region_id) {
    Enter(goal.x, goal.y);
    return;
  }
  const auto it = regions_.find(goal.region_id);
  if (it == regions_.end() || !it->second.visible) return;
  const RegionId id = goal.region_id;
  last_region_ = id;
  if (it->second.state == RegionState::kComplete) return;
  if (stack_.empty() || stack_.back() != id) {
    const auto existing = std::find(stack_.begin(), stack_.end(), id);
    if (existing != stack_.end()) stack_.erase(existing + 1, stack_.end());
    else stack_.push_back(id);
  }
  it->second.state = RegionState::kActive;
}

std::vector<ExplorationGoal> RegionScheduler::Order(
    std::vector<ExplorationGoal> goals, double x, double y) {
  for (auto& [id, r] : regions_) { (void)id; r.candidates = 0; }
  for (auto& g : goals) {
    g.region_id = RegionAt(g.frontier_x, g.frontier_y);
    if (!g.region_id) g.region_id = RegionAt(g.x, g.y);
    if (g.region_id) ++regions_.at(g.region_id).candidates;
  }
  if (!last_region_) Enter(x, y);
  // Failed observations do not complete a room. Suspend a blocked context
  // and resume the DFS parent if it still has work. New local map evidence
  // will make suspended regions eligible again through their candidates.
  for (auto& [id, r] : regions_) if (r.visible) {
    (void)id;
    if (r.candidates) r.state = RegionState::kPending;
    else if (r.frontier_cells) r.state = RegionState::kBlocked;
    else if (r.state != RegionState::kComplete) r.state = RegionState::kWaiting;
  }
  while (!stack_.empty()) {
    const auto& r = regions_.at(stack_.back());
    if (r.candidates || r.state == RegionState::kWaiting) break;
    last_region_ = stack_.back(); stack_.pop_back();
  }
  RegionId preferred = stack_.empty() ? 0 : stack_.back();
  // Wait for the second fresh map observation before declaring an empty
  // active region complete. Do not reinterpret a stale map as confirmation.
  if (preferred && regions_.at(preferred).state == RegionState::kWaiting) return {};
  RegionId origin = last_region_;
  if (!origin || !regions_.at(origin).visible) origin = RegionAt(x, y);
  std::map<RegionId, int> hops;
  std::deque<RegionId> bfs;
  if (origin) { hops[origin] = 0; bfs.push_back(origin); }
  while (!bfs.empty()) {
    auto id = bfs.front(); bfs.pop_front();
    for (auto next : regions_.at(id).neighbors) if (!hops.count(next)) {
      hops[next] = hops[id] + 1; bfs.push_back(next);
    }
  }
  auto depth = [&](RegionId id) {
    auto it = hops.find(id);
    return it == hops.end() ? std::numeric_limits<int>::max() : it->second;
  };
  auto rank = [&](const ExplorationGoal& g) {
    return std::tuple(preferred && g.region_id != preferred, depth(g.region_id),
                      -g.score, g.region_id, g.x, g.y);
  };
  std::stable_sort(goals.begin(), goals.end(), [&](const auto& a, const auto& b) {
    return rank(a) < rank(b);
  });
  if (!goals.empty() && goals.front().region_id) {
    const auto id = goals.front().region_id;
    if (stack_.empty() || stack_.back() != id) stack_.push_back(id);
    regions_.at(id).state = RegionState::kActive;
    last_region_ = id;
  }
  return goals;
}

bool RegionScheduler::Complete() const {
  if (regions_.empty()) return false;
  for (const auto& [id, r] : regions_) {
    (void)id;
    if (r.visible && r.state != RegionState::kComplete) return false;
  }
  return true;
}
std::string RegionScheduler::Audit() const {
  std::ostringstream out;
  out << "stack=[";
  for (auto id : stack_) out << id << ',';
  out << "] regions=[";
  for (const auto& [id, r] : regions_) if (r.visible)
    out << "{id=" << id << ",parent=" << r.parent << ",state=" << StateName(r.state)
        << ",raw=" << r.frontier_cells << ",candidates=" << r.candidates
        << ",empty_updates=" << r.stable_empty_updates << "},";
  out << ']';
  return out.str();
}
}  // namespace navigation2d
