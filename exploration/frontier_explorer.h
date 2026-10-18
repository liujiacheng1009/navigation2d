#pragma once

#include "navigation2d/exploration/region_scheduler.h"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
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

// True when a world-frame polyline puts the body on a live occupied cell.
// Unknown is not lethal. SLAM occupied is never carved open for passage.
bool PathCutsLiveOccupied(const ExplorationGrid& live,
                          const std::vector<std::pair<double, double>>& path_xy,
                          double robot_x, double robot_y,
                          double body_radius = .22,
                          double under_robot_m = .22);

// After repeated live-map path cuts (>=2), leave the tip pocket instead of
// replan thrash. A single low-clearance graze near a passable door must not
// abandon the room.
bool PreferTipAbandonAfterPathCut(int mapped_obstacle_rejects,
                                  double path_min_clearance_m);

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

// Debug overlay for SelectGoals: approachable frontier cells, per-cluster
// viewpoints, and final mouth-stable goals. World frame = map frame.
struct FrontierDebugSnapshot {
  struct Cell {
    double x = 0.;
    double y = 0.;
    int cluster_id = -1;
  };
  struct Goal {
    double x = 0.;
    double y = 0.;
    double frontier_x = 0.;
    double frontier_y = 0.;
    int cluster_id = -1;
    double score = 0.;
    double information_gain = 0.;
    int frontier_cells = 0;
  };
  std::vector<Cell> cells;
  std::vector<Goal> cluster_goals;
  std::vector<Goal> goals;
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
  // Mouth-normalized goals: micro-clusters are hyperclustered to doorway/
  // alcove mouths (one task each), preferring a room-side entry pose.
  std::vector<ExplorationGoal> SelectGoals(double robot_x, double robot_y);
  // One open-space approach to leftover free/unknown specks that failed the
  // ordinary cluster/standoff tests.  Not part of the exploring tour.
  std::optional<ExplorationGoal> SelectLeftoverApproach(double robot_x, double robot_y);
  // Up to `max_count` leftover approaches on distinct unfinished mouths.
  // Temporary failure cools are ignored so demoted alcoves stay finishable.
  std::vector<ExplorationGoal> SelectLeftoverApproaches(double robot_x, double robot_y,
                                                        int max_count = 3);
  // Drop non-persistent blacklist entries so completing can retry mouths that
  // failed once while their unknown tip is still visible.
  void ClearFailedBlacklist();
  // Order SelectGoals by RegionScheduler: finish the active topological
  // region (and its children) before crossing to a sibling room. Geometry
  // only ranks observations; it never relaxes planning obstacles.
  std::vector<ExplorationGoal> BuildTour(double robot_x, double robot_y,
                                         double robot_yaw =
                                             std::numeric_limits<double>::quiet_NaN());
  // Same region DFS ordering for leftover completing approaches.
  std::vector<ExplorationGoal> OrderByRegion(std::vector<ExplorationGoal> goals,
                                             double robot_x, double robot_y,
                                             double robot_yaw =
                                                 std::numeric_limits<double>::quiet_NaN());
  // Commit DFS context when a goal is accepted for execution (not on inspect).
  void CommitRegion(const ExplorationGoal& goal);
  void CommitRegion(double x, double y);
  bool GoalInActiveRegion(const ExplorationGoal& goal) const;
  RegionId ActiveRegion() const { return regions_.ActiveRegion(); }
  bool RegionsComplete() const { return regions_.Complete(); }
  std::string RegionAudit() const { return regions_.Audit(); }
  // True when a nearer executable mouth sits in the active region while the
  // active goal is a clearly longer transit — used to interrupt drive-past
  // behaviour at junctions without abandoning an unfinished room.
  // `path_remaining_m` < 0 falls back to BFS distance to `active_goal`.
  bool ShouldPreemptForNearerMouth(double robot_x, double robot_y,
                                   const ExplorationGoal& active_goal,
                                   double path_remaining_m = -1.);
  // Free throat width at a frontier (metres). Wide easy doors (~≥0.50 m)
  // use shallower entry probes; pinch mouths stay on the deep-probe path.
  double MouthGapWidth(double frontier_x, double frontier_y) const;
  // True only for a door/alcove throat (narrow free run in [0.32, 1.40] m).
  // Open-wall frontier expansion is NOT a mouth — MouthGapWidth alone can
  // read 1.6 m+ in an open room and must not trigger entry.
  bool IsCorridorMouth(double frontier_x, double frontier_y) const;
  // How far Free space continues past the frontier along the into-room axis.
  double MouthFreeDepth(const ExplorationGoal& frontier_goal, double robot_x,
                        double robot_y) const;
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
  // `cool_frontier` false cools only the viewpoint — open-wall demotes must
  // not 1.1 m-blacklist a nearby real door (live: sealed rooms → candidates=0).
  void RecordAttempt(const ExplorationGoal& goal, bool succeeded,
                     bool region_complete, bool cool_frontier = true);
  void RecordAttempt(const ExplorationGoal& goal, bool succeeded);
  // Temporary cool-down of a local pocket (dead-end alcove crumbs) so
  // nearest-first touring cannot re-queue the same room immediately.
  // Radii >= 1.5 m also suppress leftover approaches (wall-tip thrash).
  void CoolRegion(double x, double y, double radius_m,
                  std::size_t required_known_growth = 2500u);
  void ClearBlacklist() { blacklist_.clear(); }
  bool CompletionEligible(std::size_t initial_known_cells) const;
  // Unknown cells that are reachable through the planner's safe graph on the
  // observed occupancy map (no door-seal carving). Distinguishes an unvisited
  // room entrance from unknown space behind a wall or outside the map.
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
  // Last SelectGoals geometry for the debug UI (cells / clusters / goals).
  const FrontierDebugSnapshot& debug_snapshot() const { return debug_snapshot_; }

 private:
  bool Free(int col, int row) const;
  bool ClearAt(int col, int row, double clearance) const;
  bool SafeViewpoint(int col, int row) const;
  bool HasFreeLineOfSight(int from_col, int from_row, int to_col, int to_row) const;
  bool RawFree(int col, int row) const;
  bool RawFrontier(int col, int row) const;
  std::size_t CountRawFrontierCells() const;
  bool Blacklisted(double x, double y, bool boundary = false) const;
  bool PersistentlyBlacklisted(double x, double y) const;
  std::vector<ExplorationGoal> StableGoals(std::vector<ExplorationGoal> goals);
  // Collapse micro-cluster viewpoints into one task per doorway/alcove mouth.
  // Prefers SelectEntryProbe (room side) over hallway standoffs.
  std::vector<ExplorationGoal> OneGoalPerMouth(std::vector<ExplorationGoal> goals,
                                               double robot_x, double robot_y) const;
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
  FrontierDebugSnapshot debug_snapshot_;
  RegionScheduler regions_;
};

}  // namespace navigation2d
