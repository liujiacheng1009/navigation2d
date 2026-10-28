#pragma once

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/robot_geometry.h"

namespace navigation2d::test {
inline FrontierExplorerConfig BasicFrontierConfig() {
  FrontierExplorerConfig config;
  config.minimum_frontier_cells = 3;
  config.footprint_clearance = .1;
  config.required_frontier_observations = 1;
  return config;
}
inline FrontierExplorerConfig DoorFrontierConfig() {
  FrontierExplorerConfig config;
  config.minimum_frontier_cells = 6;
  config.footprint_clearance = kPlanningRadius();
  config.minimum_standoff = .10;
  config.maximum_standoff = 1.10;
  config.required_frontier_observations = 1;
  return config;
}
inline FrontierExplorerConfig LeftoverFrontierConfig() {
  FrontierExplorerConfig config;
  config.minimum_frontier_cells = 6;
  config.footprint_clearance = .40;
  config.minimum_standoff = .55;
  config.maximum_standoff = 1.10;
  config.required_frontier_observations = 1;
  return config;
}
// Shared immutable input; every test owns its returned copy and explorer.
inline ExplorationGrid MatureLeftoverMap() {
  ExplorationGrid map;
  map.width = map.height = 110;
  map.resolution = .1;
  map.cells.assign(static_cast<std::size_t>(map.width * map.height), 100);
  for (int row = 2; row < 108; ++row) {
    for (int col = 2; col < 108; ++col) {
      map.cells[row * map.width + col] = 0;
    }
  }
  map.cells[100 * map.width + 100] = -1;
  return map;
}
}  // namespace navigation2d::test
