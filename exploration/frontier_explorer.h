#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace navigation2d {

struct ExplorationGrid {
  int width = 0;
  int height = 0;
  double resolution = 0.;
  double origin_x = 0.;
  double origin_y = 0.;
  std::vector<std::int8_t> cells;
};

// The chassis already has a corridor it can enter: occupied wall to occupied
// wall, wide enough to pass and not an open room. Unknown cells in a gap that
// is already partly free are the rest of that opening. Mark only those cells
// free, and continue only while the gap stays in that same range, so the
// viewpoint and the plan use the centerline. A fully observed corridor, a
// wider unknown room, and a gap the chassis cannot enter stay as mapped.
void OpenPassableOpenings(ExplorationGrid* grid);

struct ExplorationGoal {
  double x = 0.;
  double y = 0.;
  double yaw = 0.;
  double frontier_x = 0.;
  double frontier_y = 0.;
  int frontier_cells = 0;
  // Number of frontier cells represented by this observation task.  Kept
  // separately from the raw component size so tour ordering can favour one
  // informative viewpoint over several tiny fragments of the same boundary.
  double information_gain = 0.;
  double score = 0.;
  // True when this goal was synthesized from leftover specks that failed
  // the ordinary cluster/standoff tests.  Used only as one completing probe
  // after the exploring tour is empty.
  bool leftover_approach = false;
  // Assigned by RegionScheduler after frontier extraction. Kept at the end
  // so existing aggregate construction of goals remains source compatible.
  std::uint64_t region_id = 0;
};

struct FrontierExplorerConfig {
  int minimum_frontier_cells = 6;
  double footprint_clearance = .38;
  double minimum_standoff = .38;
  double maximum_standoff = .9;
  double blacklist_radius = .7;
  // A frontier caused by one bad ray must not immediately become a navigation
  // task.  Require it to survive consecutive online-map observations.
  double frontier_stability_radius = .35;
  // Frontier hypotheses are filtered by the current-map geometry and then
  // validated by the real planner before execution. Do not block navigation
  // on repeated representative observations; online maps may move them every
  // update. Association and cooldown still prevent duplicate exploration.
  int required_frontier_observations = 1;
  int minimum_completed_goals = 3;
  std::size_t minimum_known_cell_growth = 500;
  double minimum_known_growth_ratio = .25;
};

// Navigation2D exploration policy. It converts a partial occupancy grid into
// safe, scored frontier viewpoints and owns attempt/completion bookkeeping.
// ROS and simulators are deliberately absent from this interface.
class FrontierExplorer {
 public:
  explicit FrontierExplorer(FrontierExplorerConfig config = {});
  void UpdateMap(ExplorationGrid map);
  std::vector<ExplorationGoal> SelectGoals(double robot_x, double robot_y);
  // One open-space approach to leftover free/unknown specks that failed the
  // ordinary cluster/standoff tests.  Not part of the exploring tour.
  std::optional<ExplorationGoal> SelectLeftoverApproach(double robot_x, double robot_y);
  // Builds a utility-ordered tour over the connected safe-space graph. Each
  // entry is a collision-safe viewpoint for a distinct reachable frontier.
  // The next viewpoint minimizes travel potential minus frontier information
  // gain, following the established explore_lite frontier policy.
  std::vector<ExplorationGoal> BuildTour(double robot_x, double robot_y);
  // Returns a reachable pose beyond a doorway frontier.  The pose is chosen
  // from the private planning grid, where only physically passable narrow
  // openings have been opened; it never treats an arbitrary unknown room as
  // traversable.
  std::optional<ExplorationGoal> SelectEntryProbe(
      const ExplorationGoal& frontier_goal, double robot_x, double robot_y,
      double minimum_depth_m = .22, double maximum_depth_m = .70) const;
  // Tests whether two frontier viewpoints belong to the same nearby
  // reachable free-space region.  This deliberately follows the planning
  // graph instead of using a Euclidean circle that can cross a wall.
  bool InSameReachableRegion(double anchor_x, double anchor_y,
                             double target_x, double target_y,
                             double max_distance_m) const;
  bool GoalStillFrontier(const ExplorationGoal& goal) const;
  // Frontiers move by a few cells as each scan converts their old boundary
  // from unknown to free.  This associates a queued task with its local
  // frontier region instead of treating that benign movement as completion.
  bool GoalRegionStillFrontier(const ExplorationGoal& goal) const;
  // `region_complete` is false for a successful doorway probe: crossing an
  // entrance observes the room but does not prove the room has been swept.
  void RecordAttempt(const ExplorationGoal& goal, bool succeeded,
                     bool region_complete);
  void RecordAttempt(const ExplorationGoal& goal, bool succeeded);
  void ClearBlacklist() { blacklist_.clear(); }
  bool CompletionEligible(std::size_t initial_known_cells) const;
  // Unknown cells that are reachable through the planner's safe graph,
  // including only narrow openings explicitly opened by OpenPassableOpenings.
  // This distinguishes an unvisited room entrance from unknown space behind
  // a wall or outside the observed map boundary.
  std::size_t ReachableUnknownCells(double robot_x, double robot_y) const;
  std::size_t RegionFrontierCells(double anchor_x, double anchor_y,
                                  double radius_m) const;
  std::size_t KnownCells() const;
  int completed_goals() const { return completed_goals_; }
  int failed_goals() const { return failed_goals_; }
  std::size_t raw_frontier_cells() const { return raw_frontier_cells_; }
  // Frontiers measured from the unmodified mapper output.  The planner may
  // open a narrow, partly observed passage in its private planning copy, but
  // that must never erase evidence that an adjacent room is still unknown.
  std::size_t unresolved_frontier_cells() const { return unresolved_frontier_cells_; }
  std::size_t raw_frontier_clusters() const { return raw_frontier_clusters_; }
  std::size_t executable_frontier_goals() const { return executable_frontier_goals_; }
  std::size_t candidate_frontier_goals() const { return candidate_frontier_goals_; }

 private:
  bool Free(int col, int row) const;
  bool ClearAt(int col, int row, double clearance) const;
  bool SafeViewpoint(int col, int row) const;
  bool HasFreeLineOfSight(int from_col, int from_row, int to_col, int to_row) const;
  bool RawFree(int col, int row) const;
  bool RawFrontier(int col, int row) const;
  std::size_t CountRawFrontierCells() const;
  bool Blacklisted(double x, double y, bool boundary = false) const;
  std::vector<ExplorationGoal> StableGoals(std::vector<ExplorationGoal> goals);
  std::pair<double, double> CellCenter(int col, int row) const;

  FrontierExplorerConfig config_;
  // observation_map_ is the exact occupancy grid received from the mapper.
  // map_ is a planning copy in which only physically passable partial
  // openings may be opened.  Completion and frontier identity always use the
  // observation map; path safety continues to use the planning copy.
  ExplorationGrid observation_map_;
  ExplorationGrid map_;
  struct BlacklistEntry {
    double x = 0., y = 0.;
    double radius = 0.;
    std::size_t known_cells = 0;
    // Successful observation viewpoints are cooled for a larger map-growth
    // budget so the planner cannot immediately revisit the same room edge.
    std::size_t required_growth = 600;
    // A successful doorway is an observation already performed in this
    // mission. It must not become eligible again merely because the map grew.
    bool persistent = false;
  };
  std::vector<BlacklistEntry> blacklist_;
  struct FrontierTrack {
    double x = 0., y = 0.;
    double goal_x = 0., goal_y = 0.;
    int observations = 0;
    std::uint64_t last_observation = 0;
  };
  std::vector<FrontierTrack> frontier_tracks_;
  std::uint64_t stability_observation_ = 0;
  int completed_goals_ = 0;
  int failed_goals_ = 0;
  std::size_t raw_frontier_cells_ = 0;
  std::size_t unresolved_frontier_cells_ = 0;
  std::size_t raw_frontier_clusters_ = 0;
  std::size_t executable_frontier_goals_ = 0;
  std::size_t candidate_frontier_goals_ = 0;
};

}  // namespace navigation2d
