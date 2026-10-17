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

  // Every disconnected observed component needs an identity, even if another
  // component has an eroded core. Previously narrow islands were label zero.
  for (int seed = 0; seed < n; ++seed) {
    if (g.cells[seed] != 0 || components[seed]) continue;
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
  }

  // Preserve ownership evidence across temporary missing cells and merges.
  // Matching only the preceding raster loses every absorbed/disappearing ID.
  // Resolution changes start a new identity domain; padding does not.
  if (resolution_ > 0. && std::abs(resolution_ - g.resolution) > 1e-9) {
    history_.clear(); labels_.clear(); regions_.clear(); stack_.clear();
    last_region_ = 0; width_ = height_ = 0;
  }
  ++revision_;
  constexpr std::uint64_t kHistoryUpdates = 64;
  for (auto it = history_.begin(); it != history_.end();) {
    if (revision_ - it->second.seen > kHistoryUpdates) it = history_.erase(it);
    else ++it;
  }
  const auto world_cell = [&](int i) {
    return std::make_pair(
        static_cast<std::int64_t>(std::floor(
            (g.origin_x + (i % g.width + .5) * g.resolution) / g.resolution)),
        static_cast<std::int64_t>(std::floor(
            (g.origin_y + (i / g.width + .5) * g.resolution) / g.resolution)));
  };
  std::map<std::pair<int, RegionId>, std::size_t> overlaps;
  for (int i = 0; i < n; ++i) if (components[i]) {
    const auto remembered = history_.find(world_cell(i));
    const auto old = remembered != history_.end() ? remembered->second.owner :
        RegionAt(g.origin_x + (i % g.width + .5) * g.resolution,
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
  for (int i = 0; i < n; ++i) if (labels[i]) {
    const auto key = world_cell(i);
    auto [it, inserted] = history_.try_emplace(key, CellHistory{labels[i], revision_});
    // Do not overwrite an absorbed region's cells with the merge winner:
    // they are what lets a later split recover its original identity.
    // On a true split the old owner is already assigned to another child.
    // Transfer these cells to this child once, so subsequent updates reuse it.
    if (it->second.owner != labels[i] && used.count(it->second.owner))
      it->second.owner = labels[i];
    it->second.seen = revision_;
    (void)inserted;
  }
  // Retain only identities supported by recent cells or live graph context.
  // Preserve ancestors so DFS parent links cannot dangle.
  std::set<RegionId> retained(stack_.begin(), stack_.end());
  if (last_region_) retained.insert(last_region_);
  for (const auto& [cell, h] : history_) { (void)cell; retained.insert(h.owner); }
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
    else if (r.frontier_cells) r.state = RegionState::kBlocked;
    else if (r.state != RegionState::kComplete) r.state = RegionState::kWaiting;
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
    // Keep the DFS lock while the room still has raw free↔unknown cells,
    // even when SelectGoals found no SafeViewpoint (pinched alcove tip).
    // Popping on candidates==0 was the "ran away before finishing" bug.
    if (r.candidates || child_work || r.state == RegionState::kWaiting ||
        r.frontier_cells > 0)
      break;
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
  // Unfinished PENDING/ACTIVE rooms hard-lock while they still have executable
  // mouths. BLOCKED tips (raw frontiers, often unproductive tip thrash) must
  // soft-unlock: hard-locking them discarded large unfinished sibling rooms
  // and returned an empty tour (PARTIAL) while white alcoves remained visible.
  // Soft-unlock prefers nearby siblings, but keeps farther unfinished rooms
  // when nothing is within range — the old 4 m erase returned {} at ~4–8 m.
  bool soft_unlocked = false;
  if (preferred && regions_.at(preferred).frontier_cells > 0) {
    const bool preferred_has_mouths =
        regions_.at(preferred).candidates > 0 ||
        std::any_of(goals.begin(), goals.end(), [&](const ExplorationGoal& g) {
          return descendant(g.region_id, preferred);
        });
    const bool hard_lock =
        preferred_has_mouths &&
        regions_.at(preferred).state != RegionState::kBlocked;
    if (hard_lock) {
      goals.erase(std::remove_if(goals.begin(), goals.end(),
                                 [&](const ExplorationGoal& g) {
                                   return !descendant(g.region_id, preferred);
                                 }),
                  goals.end());
      if (goals.empty()) return {};
    } else {
      soft_unlocked = true;
      constexpr double kSoftUnlockPreferM = 4.0;
      // When tip-local mouths force a leave, still prefer unfinished rooms in
      // the local wing (~8 m) over apartment-crossing hops (log: tip at
      // (-4.8,-2.2) → (-0.3,4.1) while siblings sat a few metres away).
      constexpr double kSoftUnlockLocalWingM = 8.0;
      std::vector<ExplorationGoal> nearby;
      std::vector<ExplorationGoal> far;
      nearby.reserve(goals.size());
      far.reserve(goals.size());
      for (const auto& g : goals) {
        if (std::hypot(g.x - x, g.y - y) <= kSoftUnlockPreferM)
          nearby.push_back(g);
        else
          far.push_back(g);
      }
      if (!nearby.empty() && !far.empty() &&
          regions_.at(preferred).state == RegionState::kBlocked) {
        const bool nearby_only_tip = std::all_of(
            nearby.begin(), nearby.end(), [&](const ExplorationGoal& g) {
              return descendant(g.region_id, preferred);
            });
        // Tip-local mouths alone would win the nearby filter forever; leave
        // the exhausted pocket for unfinished rooms elsewhere.
        if (nearby_only_tip) {
          std::vector<ExplorationGoal> wing;
          for (const auto& g : far) {
            if (std::hypot(g.x - x, g.y - y) <= kSoftUnlockLocalWingM)
              wing.push_back(g);
          }
          goals = !wing.empty() ? std::move(wing) : std::move(far);
        } else {
          goals = std::move(nearby);
        }
      } else if (!nearby.empty()) {
        goals = std::move(nearby);
      }
      // else: keep all goals (far-only unfinished rooms)
      if (goals.empty()) return {};
    }
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
    // Soft-unlock: any large reverse is a wasted U-turn; hard-lock keeps the
    // older "local tip behind OK" contract. BLOCKED tip mouths sort after
    // unfinished sibling rooms so tip thrash cannot starve coverage.
    const bool reverse = use_heading && turn > 1.75 &&
                         (soft_unlocked || far_pocket);
    const bool tip_mouth =
        soft_unlocked && preferred && descendant(g.region_id, preferred);
    return std::tuple(tip_mouth,
                      !soft_unlocked && preferred &&
                          !descendant(g.region_id, preferred),
                      soft_unlocked && reverse, far_pocket, reverse, turn,
                      depth(g.region_id), distance - .20 * gain_per_meter,
                      -gain_per_meter, g.region_id, g.x, g.y);
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
