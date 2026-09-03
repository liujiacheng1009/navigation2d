#pragma once

#include <cstdint>
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
  int required_frontier_observations = 2;
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
  // Builds a utility-ordered tour over the connected safe-space graph. Each
  // entry is a collision-safe viewpoint for a distinct reachable frontier.
  // The next viewpoint minimizes travel potential minus frontier information
  // gain, following the established explore_lite frontier policy.
  std::vector<ExplorationGoal> BuildTour(double robot_x, double robot_y);
  bool GoalStillFrontier(const ExplorationGoal& goal) const;
  // Frontiers move by a few cells as each scan converts their old boundary
  // from unknown to free.  This associates a queued task with its local
  // frontier region instead of treating that benign movement as completion.
  bool GoalRegionStillFrontier(const ExplorationGoal& goal) const;
  void RecordAttempt(const ExplorationGoal& goal, bool succeeded);
  void ClearBlacklist() { blacklist_.clear(); }
  bool CompletionEligible(std::size_t initial_known_cells) const;
  std::size_t KnownCells() const;
  int completed_goals() const { return completed_goals_; }
  int failed_goals() const { return failed_goals_; }
  std::size_t raw_frontier_cells() const { return raw_frontier_cells_; }
  std::size_t raw_frontier_clusters() const { return raw_frontier_clusters_; }
  std::size_t executable_frontier_goals() const { return executable_frontier_goals_; }
  std::size_t candidate_frontier_goals() const { return candidate_frontier_goals_; }

 private:
  bool Free(int col, int row) const;
  bool SafeViewpoint(int col, int row) const;
  bool HasFreeLineOfSight(int from_col, int from_row, int to_col, int to_row) const;
  bool Blacklisted(double x, double y) const;
  std::vector<ExplorationGoal> StableGoals(std::vector<ExplorationGoal> goals);
  std::pair<double, double> CellCenter(int col, int row) const;

  FrontierExplorerConfig config_;
  ExplorationGrid map_;
  struct BlacklistEntry {
    double x = 0., y = 0.;
    double radius = 0.;
    std::size_t known_cells = 0;
  };
  std::vector<BlacklistEntry> blacklist_;
  struct FrontierTrack {
    double x = 0., y = 0.;
    int observations = 0;
    std::uint64_t last_observation = 0;
  };
  std::vector<FrontierTrack> frontier_tracks_;
  std::uint64_t stability_observation_ = 0;
  int completed_goals_ = 0;
  int failed_goals_ = 0;
  std::size_t raw_frontier_cells_ = 0;
  std::size_t raw_frontier_clusters_ = 0;
  std::size_t executable_frontier_goals_ = 0;
  std::size_t candidate_frontier_goals_ = 0;
};

}  // namespace navigation2d
