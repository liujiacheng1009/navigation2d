#pragma once
#include <deque>
#include <string>

#include "navigation2d/application/exploration/navigation_telemetry.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
// Borrowed synchronous output view. Never supplies mission decision state.
struct ExplorationStatusView {
  const std::string& state;
  const std::string& message;
  const char* phase;
  std::size_t known_cells;
  int completed_goals, failed_goals;
  std::size_t raw_frontier_cells, unresolved_frontier_cells, candidates;
  const std::deque<ExplorationGoal>& tour;
  const ExplorationGoal& active_goal;
  const NavigationTelemetryState& navigation;
  const std::string& planning_failure;
  double path_min_clearance_m;
  double carrot_x, carrot_y, carrot_yaw;
  bool carrot_valid;
  const std::string& pose_source;
};
std::string SerializeExplorationStatus(const ExplorationStatusView& view);
}  // namespace navigation2d::exploration
