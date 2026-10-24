#pragma once

#include <cstddef>

namespace navigation2d {

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
  // Sweep coverage visits non-blacklisted residuals after exploring work is
  // gone. Mapping tasks leave this false.
  bool cover_residuals = true;
  int minimum_completed_goals = 3;
  std::size_t minimum_known_cell_growth = 500;
  double minimum_known_growth_ratio = .25;
};

}  // namespace navigation2d
