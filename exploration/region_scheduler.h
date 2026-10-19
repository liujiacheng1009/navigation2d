#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace navigation2d {
struct ExplorationGrid;
struct ExplorationGoal;
using RegionId = std::uint64_t;
enum class RegionState { kPending, kActive, kWaiting, kBlocked, kComplete };
struct RegionRecord {
  RegionId id = 0, parent = 0;
  RegionState state = RegionState::kPending;
  bool visible = false;
  // Set by the explorer when this region still has a passable door.
  // Raw frontier_cells do not keep the room open.
  bool passage = false;
  std::size_t free_cells = 0, frontier_cells = 0, candidates = 0;
  unsigned stable_empty_updates = 0;
  std::uint64_t evidence = 0;
  std::set<RegionId> neighbors;
};

// Mission-local topological regions, not semantic room recognition. Wide
// connected spaces can be one region. Geometry only orders observations; it
// never clears unknown/occupied cells or relaxes the navigation footprint.
class RegionScheduler {
 public:
  void Update(const ExplorationGrid& observed, double footprint_clearance);
  RegionId RegionAt(double x, double y) const;
  // While the active room has an admitted goal, keep only that room and its
  // children. A mouth with no viewpoint does not hold the room: the stack
  // pops and other rooms' goals become the tour. `robot_yaw` ranks a far
  // reverse after a forward goal.
  // `robot_yaw` (rad) ranks reverse U-turn goals after forward ones inside the
  // same DFS region. Pass NaN when heading is unknown.
  std::vector<ExplorationGoal> Order(std::vector<ExplorationGoal> goals,
                                     double robot_x, double robot_y,
                                     double robot_yaw =
                                         std::numeric_limits<double>::quiet_NaN());
  void Enter(double x, double y);
  void Enter(const ExplorationGoal& goal);
  // True when `id` is the active DFS region or a child of it. Empty stack
  // means no commitment yet (any region is eligible).
  bool InActiveContext(RegionId id) const;
  void ClearPassages();
  void MarkPassage(RegionId id);
  bool HasPassage(RegionId id) const;
  bool Complete() const;
  const std::map<RegionId, RegionRecord>& regions() const { return regions_; }
  const std::vector<RegionId>& stack() const { return stack_; }
  RegionId ActiveRegion() const {
    return stack_.empty() ? 0 : stack_.back();
  }
  std::string Audit() const;
 private:
  int Cell(double x, double y) const;
  int width_ = 0, height_ = 0;
  double resolution_ = 0., origin_x_ = 0., origin_y_ = 0.;
  RegionId next_id_ = 1, last_region_ = 0;
  std::vector<RegionId> labels_, stack_;
  std::map<RegionId, RegionRecord> regions_;
  // Last owner of a world cell survives temporary disappearance and merges.
  struct CellHistory { RegionId owner; std::uint64_t seen; };
  std::map<std::pair<std::int64_t, std::int64_t>, CellHistory> history_;
  std::uint64_t revision_ = 0;

};
}  // namespace navigation2d
