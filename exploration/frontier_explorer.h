#pragma once

#include "navigation2d/exploration/exploration_policy.h"
#include "navigation2d/exploration/frontier_types.h"
#include "navigation2d/exploration/frontier_debug_snapshot.h"
#include "navigation2d/exploration/live_route_safety.h"
#include "navigation2d/exploration/map_evidence.h"
#include "navigation2d/exploration/frontier_tracks.h"
#include "navigation2d/exploration/frontier_visit_memory.h"
#include "navigation2d/exploration/exploration_config.h"
#include "navigation2d/exploration/region_scheduler.h"
#include "navigation2d/geometry/robot_geometry.h"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace navigation2d {

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
  // True when the active room or one of its children still has a passable door.
  bool ActiveHoldsPassage() const;
  // Admitted goals or a passable door. Raw residual cells are not exploring work.
  bool HasExploringWork() const;
  // Coverage finish: exploring work is gone, residuals remain, and this task
  // asked to sweep them.
  bool ShouldCoverResiduals() const;
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
  // True when a room of unknown still lies past this viewpoint. A doorway
  // peek moves the frontier farther than GoalRegionStillFrontier's disk;
  // the viewpoint is not done until that room is entered.
  bool UnenteredRoomBeyond(const ExplorationGoal& goal) const;
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
  void ClearBlacklist() { visits_.Clear(); }
  bool CompletionEligible(std::size_t initial_known_cells) const;
  // Unknown cells that are reachable through the planner's safe graph on the
  // observed occupancy map (no door-seal carving). Distinguishes an unvisited
  // room entrance from unknown space behind a wall or outside the map.
  std::size_t ReachableUnknownCells(double robot_x, double robot_y) const;
  // Known free space the robot has not driven into. Distance to the driven
  // trail is measured through free cells, so a room behind a wall is far even
  // when the hallway runs beside it. Each component farther than
  // `visit_radius_m` with at least `minimum_area_m2` of free cells yields one
  // goal at its deepest cell that clears the planning circle. A room seen
  // through its doorway has no frontier left but still appears here.
  // Moves a goal pose to the nearest observed-free cell within
  // kSwingClearance() that clears the planning circle by one more map cell.
  // A viewpoint picked at exactly kPlanningRadius() beside a jamb turns
  // "goal occupied" as soon as the next scan adds the jamb's end cell.
  // The frontier target is kept. Returns nullopt when no such cell exists.
  std::optional<ExplorationGoal> SettleGoal(const ExplorationGoal& goal) const;
  // Free-space components farther than `visit_radius_m` from the trail that
  // a room seed reaches within `seed_radius_m` through free cells while
  // never stepping closer to the trail. Seeds are doorway probe targets of
  // rooms never entered.
  std::vector<ExplorationGoal> UnvisitedSpaceGoals(
      const std::vector<std::pair<double, double>>& trail,
      double visit_radius_m, double minimum_area_m2,
      const std::vector<std::pair<double, double>>& room_seeds,
      double seed_radius_m) const;
  std::size_t RegionFrontierCells(double anchor_x, double anchor_y,
                                  double radius_m) const;
  const ExplorationGrid& observation_map() const { return maps_.observation(); }
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
  // Rebuild frontier-cell overlay from live RawFrontier; color by nearest
  // last-SelectGoals cluster goal. Keeps cluster_goals / goals unchanged.
  void RefreshDebugFrontierCells();

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
  void RefreshPassages();
  std::vector<ExplorationGoal> StableGoals(std::vector<ExplorationGoal> goals);
  // Collapse micro-cluster viewpoints into one task per doorway/alcove mouth.
  // Prefers SelectEntryProbe (room side) over hallway standoffs.
  std::vector<ExplorationGoal> OneGoalPerMouth(std::vector<ExplorationGoal> goals,
                                               double robot_x, double robot_y) const;
  std::pair<double, double> CellCenter(int col, int row) const;

  FrontierExplorerConfig config_;
  frontier_internal::MapEvidence maps_;
  frontier_internal::FrontierVisitMemory visits_;
  frontier_internal::FrontierTracks tracks_;
  int completed_goals_ = 0;
  int failed_goals_ = 0;
  std::size_t raw_frontier_cells_ = 0;
  std::size_t unresolved_frontier_cells_ = 0;
  std::size_t raw_frontier_clusters_ = 0;
  std::size_t executable_frontier_goals_ = 0;
  std::size_t candidate_frontier_goals_ = 0;
  FrontierDebugSnapshot debug_snapshot_;
  RegionScheduler regions_;
  TipLedger tips_;
};

}  // namespace navigation2d
