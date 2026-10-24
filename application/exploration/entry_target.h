#pragma once
#include <array>
#include <optional>

#include "navigation2d/exploration/frontier_explorer.h"

namespace navigation2d::exploration {
struct EntryProbeSelection {
  std::optional<ExplorationGoal> goal;
  double minimum_depth_m = 0.;
  double gap_m = 0.;
  double free_depth_m = 0.;
  bool easy = false;
};
struct EntryDepthWindow {
  double minimum_m, maximum_m;
};
// Ordered, bounded search windows. No map queries or attempt-budget mutation.
std::array<EntryDepthWindow, 4> EntryDepthWindows(bool easy, double free_depth_m, int attempts);
// World-frame metres. Queries the current explorer map without changing attempt
// budgets or committing navigation. First valid depth window wins.
EntryProbeSelection SelectEntryTarget(const FrontierExplorer& explorer,
                                      const ExplorationGoal& mouth, double rx, double ry,
                                      int attempts);
}  // namespace navigation2d::exploration
