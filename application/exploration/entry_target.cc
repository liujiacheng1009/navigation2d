#include "navigation2d/application/exploration/entry_target.h"

#include <algorithm>

namespace navigation2d::exploration {
std::array<EntryDepthWindow, 4> EntryDepthWindows(bool easy_door, double free_depth, int attempts) {
  std::array<EntryDepthWindow, 4> schedules = {{
      {easy_door ? .10 : (.42 + .10 * attempts), easy_door ? .30 : (attempts == 0 ? 1.30 : 1.60)},
      {easy_door ? (.22 + .08 * attempts) : (.42 + .10 * attempts),
       easy_door ? .85 : (attempts == 0 ? 1.30 : 1.60)},
      {.18, .55},
      {.12, .35},
  }};
  for (auto& schedule : schedules) {
    double& minimum_depth = schedule.minimum_m;
    double& maximum_depth = schedule.maximum_m;
    if (free_depth > .05) {
      minimum_depth = std::min(minimum_depth, std::max(.10, free_depth - .05));
      maximum_depth = std::max(maximum_depth, minimum_depth + .10);
      maximum_depth = std::min(maximum_depth, std::max(minimum_depth + .10, free_depth + .08));
    } else if (easy_door) {
      // Tiny free nose: keep the shallow window, do not demand 0.28 m depth.
      maximum_depth = std::max(maximum_depth, minimum_depth + .10);
    }
  }
  return schedules;
}

EntryProbeSelection SelectEntryTarget(const FrontierExplorer& explorer,
                                      const ExplorationGoal& mouth, double rx, double ry,
                                      int attempts) {
  using Goal = ExplorationGoal;
  const double gap = explorer.MouthGapWidth(mouth.frontier_x, mouth.frontier_y);
  const double free_depth = explorer.MouthFreeDepth(mouth, rx, ry);
  const bool easy_door =
      explorer.IsCorridorMouth(mouth.frontier_x, mouth.frontier_y) && gap >= .50 && gap <= 1.40;
  std::optional<Goal> probe;
  double used_min_depth = 0.;
  // Wide mouths with almost no mapped free beyond the jamb (live: gap=0.70
  // free_depth=0.05) must try a shallow poke first — deep schedules only
  // return nullopt and abandon an enterable door.
  for (const auto& window : EntryDepthWindows(easy_door, free_depth, attempts)) {
    const double minimum_depth = window.minimum_m;
    const double maximum_depth = window.maximum_m;
    if (maximum_depth < minimum_depth) {
      continue;
    }
    probe = explorer.SelectEntryProbe(mouth, rx, ry, minimum_depth, maximum_depth);
    if (probe) {
      used_min_depth = minimum_depth;
      break;
    }
  }
  return {probe, used_min_depth, gap, free_depth, easy_door};
}
}  // namespace navigation2d::exploration
