#include "navigation2d/application/exploration/debug_overlay.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"

namespace navigation2d::exploration {
void PaintPlanningDebugInput(const ExplorationGrid& grid, const Pose2d& robot_local,
                             std::span<std::int8_t> output) {
  const int robot_col = static_cast<int>(std::floor((X(robot_local)) / grid.resolution));
  const int robot_row = static_cast<int>(std::floor((Y(robot_local)) / grid.resolution));
  const int evidence_cells = static_cast<int>(std::ceil(.30 / grid.resolution));

  for (std::size_t index = 0; index < grid.cells.size(); ++index) {
    const int row = static_cast<int>(index / grid.width);
    const int col = static_cast<int>(index % grid.width);
    const bool under_robot =
        (col - robot_col) * (col - robot_col) + (row - robot_row) * (row - robot_row) <=
        evidence_cells * evidence_cells;
    const auto cell = grid.cells[index];
    if (cell == 0 || under_robot) {
      output[index] = 0;
    } else if (cell < 0) {
      output[index] = -1;
    } else {
      output[index] = 100;
    }
  }
}

void PaintInflatedDebugMap(const ExplorationGrid& grid, std::span<const std::int8_t> input,
                           std::span<std::int8_t> output) {
  const double kRobotRadius = navigation2d::kPlanningRadius();
  const double kInflationRadius = navigation2d::kDebugInflationRadius();
  constexpr double kInflationCostScaling = 8.;
  const int inflation_cells = static_cast<int>(std::ceil(kInflationRadius / grid.resolution));
  for (int row = 0; row < static_cast<int>(grid.height); ++row) {
    for (int col = 0; col < static_cast<int>(grid.width); ++col) {
      if (input[static_cast<std::size_t>(row) * grid.width + col] != 100) {
        continue;
      }
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          const int x = col + dx, y = row + dy;
          if (x < 0 || y < 0 || x >= static_cast<int>(grid.width) ||
              y >= static_cast<int>(grid.height)) {
            continue;
          }
          const double distance = std::hypot(dx, dy) * grid.resolution;
          if (distance > kInflationRadius || distance == 0.) {
            continue;
          }
          const std::size_t index =
              static_cast<std::size_t>(y) * grid.width + static_cast<std::size_t>(x);
          // Match Reinflate: unknown stays unknown; only free is painted.
          if (input[index] != 0) {
            continue;
          }
          const int8_t value =
              distance <= kRobotRadius
                  ? static_cast<int8_t>(99)
                  : static_cast<int8_t>(std::clamp(
                        static_cast<int>(std::lround(
                            98. * std::exp(-kInflationCostScaling * (distance - kRobotRadius)))),
                        1, 98));
          output[index] = std::max(output[index], value);
        }
      }
    }
  }
}

std::string SerializeFrontierDebug(const FrontierDebugSnapshot& snap) {
  std::string json = "{\"cells\":[";
  bool first = true;
  constexpr std::size_t kMaxCells = 4000;
  const std::size_t cell_step =
      snap.cells.size() > kMaxCells ? snap.cells.size() / kMaxCells + 1 : 1;
  for (std::size_t i = 0; i < snap.cells.size(); i += cell_step) {
    const auto& c = snap.cells[i];
    if (!first) {
      json += ",";
    }
    first = false;
    json += "[" + std::to_string(c.x) + "," + std::to_string(c.y) + "," +
            std::to_string(c.cluster_id) + "]";
  }
  json += "],\"cluster_goals\":[";
  first = true;
  for (const auto& g : snap.cluster_goals) {
    if (!first) {
      json += ",";
    }
    first = false;
    json += "{\"x\":" + std::to_string(g.x) + ",\"y\":" + std::to_string(g.y) +
            ",\"fx\":" + std::to_string(g.frontier_x) + ",\"fy\":" + std::to_string(g.frontier_y) +
            ",\"cid\":" + std::to_string(g.cluster_id) + ",\"score\":" + std::to_string(g.score) +
            ",\"gain\":" + std::to_string(g.information_gain) +
            ",\"n\":" + std::to_string(g.frontier_cells) + "}";
  }
  json += "],\"goals\":[";
  first = true;
  for (const auto& g : snap.goals) {
    if (!first) {
      json += ",";
    }
    first = false;
    json += "{\"x\":" + std::to_string(g.x) + ",\"y\":" + std::to_string(g.y) +
            ",\"fx\":" + std::to_string(g.frontier_x) + ",\"fy\":" + std::to_string(g.frontier_y) +
            ",\"cid\":" + std::to_string(g.cluster_id) + ",\"score\":" + std::to_string(g.score) +
            ",\"gain\":" + std::to_string(g.information_gain) +
            ",\"n\":" + std::to_string(g.frontier_cells) + "}";
  }
  json += "]}";
  return json;
}
}  // namespace navigation2d::exploration
