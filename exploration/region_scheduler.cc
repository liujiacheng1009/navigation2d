#include "navigation2d/exploration/region_scheduler.h"
#include "navigation2d/exploration/region_partition.h"
#include "navigation2d/exploration/frontier_types.h"

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
RegionId RegionScheduler::RegionAt(double x, double y) const {
  return identity_.RegionAt(x, y);
}

void RegionScheduler::Update(const ExplorationGrid& g, double clearance) {
  if (g.width <= 0 || g.height <= 0 || g.resolution <= 0. ||
      g.cells.size() != static_cast<std::size_t>(g.width) * g.height) return;
  const int n = static_cast<int>(g.cells.size());
  auto neighbor = [&](int i, int d) {
    const int c = i % g.width + dx[d], r = i / g.width + dy[d];
    return c < 0 || r < 0 || c >= g.width || r >= g.height ? -1 : r * g.width + c;
  };
  auto [components, count] = exploration_internal::PartitionRegions(g, clearance);

  const auto assigned = identity_.Assign(g, components, count);
  if (assigned.domain_reset) {
    regions_.clear();
    stack_.clear();
    last_region_ = 0;
  }
  const auto& ids = assigned.ids;
  const auto& matches = assigned.matches;
  for (int c = 1; c <= count; ++c) if (ids[c] >= assigned.first_new_id) {
    RegionRecord record;
    record.id = ids[c];
    record.parent = assigned.parents[c];
    regions_[ids[c]] = record;
  }
  for (auto& [id, r] : regions_) {
    (void)id;
    r.visible = false; r.free_cells = r.frontier_cells = r.candidates = 0;
    r.passage = false;
    r.neighbors.clear();
  }
  std::map<RegionId, std::uint64_t> hashes;
  const auto& labels = identity_.labels();
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
  for (auto& id : stack_) {
    if (regions_.at(id).visible) continue;
    // An active partition can merge into a larger neighbour during mapping.
    // Transfer the context by overlap instead of silently discarding it.
    for (const auto& [size, component, old] : matches) {
      (void)size;
      if (old == id) { id = ids[component]; break; }
    }
  }
  stack_.erase(std::remove_if(stack_.begin(), stack_.end(), [&](RegionId id) {
    return !regions_.at(id).visible;
  }), stack_.end());
  stack_.erase(std::unique(stack_.begin(), stack_.end()), stack_.end());
  // Retain only identities supported by recent cells or live graph context.
  // Preserve ancestors so DFS parent links cannot dangle.
  std::set<RegionId> retained(stack_.begin(), stack_.end());
  if (last_region_) retained.insert(last_region_);
  identity_.RetainOwners(retained);
  for (const auto& [id, r] : regions_) if (r.visible) retained.insert(id);
  const auto roots = retained;
  for (auto id : roots) {
    std::set<RegionId> visited;
    while (id && visited.insert(id).second) {
      const auto it = regions_.find(id);
      if (it == regions_.end()) break;
      retained.insert(id); id = it->second.parent;
    }
  }
  for (auto it = regions_.begin(); it != regions_.end();) {
    if (!retained.count(it->first)) it = regions_.erase(it);
    else ++it;
  }
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

void RegionScheduler::ClearPassages() {
  for (auto& [id, r] : regions_) {
    (void)id;
    r.passage = false;
  }
}

void RegionScheduler::MarkPassage(RegionId id) {
  const auto it = regions_.find(id);
  if (it == regions_.end() || !it->second.visible) return;
  it->second.passage = true;
}

bool RegionScheduler::HasPassage(RegionId id) const {
  const auto it = regions_.find(id);
  return it != regions_.end() && it->second.passage;
}

bool RegionScheduler::InActiveContext(RegionId id) const {
  if (!id) return false;
  if (stack_.empty()) return true;
  const RegionId preferred = stack_.back();
  std::set<RegionId> seen;
  RegionId cur = id;
  while (cur && seen.insert(cur).second) {
    if (cur == preferred) return true;
    const auto it = regions_.find(cur);
    if (it == regions_.end()) break;
    cur = it->second.parent;
  }
  return false;
}

std::vector<ExplorationGoal> RegionScheduler::Order(
    std::vector<ExplorationGoal> goals, double x, double y, double robot_yaw) {
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
    else if (r.passage) r.state = RegionState::kBlocked;
    else if (r.frontier_cells == 0 && r.state != RegionState::kComplete)
      r.state = RegionState::kWaiting;
    else if (r.state == RegionState::kBlocked) r.state = RegionState::kPending;
  }
  auto descendant = [&](RegionId id, RegionId ancestor) {
    std::set<RegionId> seen;
    while (id && seen.insert(id).second) {
      if (id == ancestor) return true;
      const auto it = regions_.find(id);
      if (it == regions_.end()) break;
      id = it->second.parent;
    }
    return false;
  };
  while (!stack_.empty()) {
    const auto& r = regions_.at(stack_.back());
    const bool child_work = std::any_of(goals.begin(), goals.end(), [&](const auto& g) {
      return descendant(g.region_id, r.id);
    });
    // Only an admitted goal holds the room. A mouth with no viewpoint does not.
    if (r.candidates || child_work) break;
    if (r.state == RegionState::kWaiting && goals.empty()) break;
    last_region_ = stack_.back(); stack_.pop_back();
  }
  RegionId preferred = stack_.empty() ? 0 : stack_.back();
  // Wait for the second fresh map observation before declaring an empty
  // active region complete. Do not reinterpret a stale map as confirmation.
  // A frontier can be in a waiting region while still being a valid global
  // fallback. Never discard a non-empty goal set merely because the previous
  // DFS context is waiting for another map update; only an actually empty set
  // may wait here. The navigation planner remains authoritative for safety.
  if (preferred && regions_.at(preferred).state == RegionState::kWaiting &&
      goals.empty()) return {};
  // Admitted goals keep the tour inside this room. When the room has none,
  // other rooms' goals stay in the list and become the tour.
  const bool room_open =
      preferred &&
      (regions_.at(preferred).candidates > 0 ||
       std::any_of(goals.begin(), goals.end(), [&](const ExplorationGoal& g) {
         return descendant(g.region_id, preferred);
       }));
  if (room_open) {
    goals.erase(std::remove_if(goals.begin(), goals.end(),
                               [&](const ExplorationGoal& g) {
                                 return !descendant(g.region_id, preferred);
                               }),
                goals.end());
    if (goals.empty()) return {};
  }
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
  const bool use_heading = std::isfinite(robot_yaw);
  auto rank = [&](const ExplorationGoal& g) {
    // Finish the active room before crossing a doorway (coverage + fewer
    // revisits). Within that room clear the local pocket first — a tip 1–2 m
    // behind the robot is still cheaper than fleeing to a far forward mouth.
    // Reverse demotion only applies among similarly distant goals so demote
    // does not U-turn across the apartment.
    const double distance = std::hypot(g.x - x, g.y - y);
    const double frontier_range =
        std::hypot(g.frontier_x - x, g.frontier_y - y);
    const double gain_per_meter = g.information_gain / (0.45 + distance);
    double turn = 0.;
    if (use_heading) {
      const double bearing = std::atan2(g.y - y, g.x - x);
      turn = std::abs(std::atan2(std::sin(bearing - robot_yaw),
                                 std::cos(bearing - robot_yaw)));
    }
    constexpr double kLocalPocketM = 2.5;
    const bool far_pocket =
        std::min(distance, frontier_range) > kLocalPocketM;
    // A local tip behind the robot stays ahead of a far forward mouth.
    const bool reverse = use_heading && turn > 1.75 && far_pocket;
    const bool outside =
        preferred && !descendant(g.region_id, preferred);
    return std::tuple(outside, far_pocket, reverse, turn, depth(g.region_id),
                      distance - .20 * gain_per_meter, -gain_per_meter,
                      g.region_id, g.x, g.y);
  };
  std::stable_sort(goals.begin(), goals.end(), [&](const auto& a, const auto& b) {
    return rank(a) < rank(b);
  });
  // Ranking is not execution: a rejected first candidate must not push a
  // speculative region onto the DFS stack. Enter() commits an accepted goal.
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
  // Audit is emitted from the navigation executor. Dumping every historical
  // region increases diagnostic overhead. Bound output independently of
  // identity history; a causal link to heartbeat loss has not been measured.
  std::size_t emitted = 0, visible = 0;
  for (const auto& [id, r] : regions_) if (r.visible) {
    ++visible;
    if (emitted++ >= 24) continue;
    out << "{id=" << id << ",parent=" << r.parent << ",state=" << StateName(r.state)
        << ",raw=" << r.frontier_cells << ",candidates=" << r.candidates
        << ",empty_updates=" << r.stable_empty_updates << "},";
  }
  if (visible > 24) out << "...+" << (visible - 24) << " more";
  out << ']';
  return out.str();
}
}  // namespace navigation2d
