#include "navigation2d/application/exploration/planning_snapshot.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
PreparedSnapshot PlanningSnapshotBuilder::Build(const ExplorationGrid& map,
    const SnapshotRequest& request, PlanningSnapshot reusable) {
  if (map.width <= 0 || map.height <= 0 || !std::isfinite(map.resolution) || map.resolution <= 0. ||
      !std::isfinite(map.origin_x) || !std::isfinite(map.origin_y) ||
      static_cast<std::size_t>(map.width) >
          std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(map.height) ||
      map.cells.size() != static_cast<std::size_t>(map.width) * map.height) {
    throw std::invalid_argument("invalid planning observation grid");
  }
  PreparedSnapshot prepared;
  prepared.snapshot = std::move(reusable);
  auto& result = prepared.snapshot;
  result.map_revision = request.map_revision;
  result.trail_budget_m = request.trail_budget_m;
  const int robot_col =
      static_cast<int>(std::floor((X(request.robot) - map.origin_x) / map.resolution));
  const int robot_row =
      static_cast<int>(std::floor((Y(request.robot) - map.origin_y) / map.resolution));
  // The simulator's ground-truth pose guarantees that the complete base
  // footprint is free even when SLAM has not yet converted the cells under
  // the robot from unknown.  Keep that local evidence slightly wider than
  // the planner radius (the body circle), so
  // a valid current pose is not rejected as "start occupied" on a replan
  // after a tight-corner approach.  This only clears the measured current
  // footprint; all cells ahead remain lethal unless observed free.
  // Unknown next to the robot is not yet observed. Occupied is cleared only
  // inside the planning circle — the same disk PathCuts ignores. Clearing
  // occupied out to this wider disk let the planner thread a door jamb and
  // PathCuts then reject every entry as a first-segment replay.
  const double snapshot_clearance = .40;
  const int robot_clearance = static_cast<int>(std::ceil(snapshot_clearance / map.resolution));
  const int body_cells =
      static_cast<int>(std::ceil(navigation2d::kPlanningRadius() / map.resolution));
  // Near-field lidar is fresher than the online occupancy grid. Preserve
  // only ray interiors (never the hit endpoint) as temporary free evidence
  // for this plan, allowing a certified narrow passage to be entered before
  // the mapper has committed every cell. The hard footprint check remains
  // authoritative and the evidence expires with this plan.
  std::unordered_set<std::size_t> near_field_free;
  if (request.scan) {
    const double yaw = Yaw(request.robot);
    const double lidar_forward = navigation2d::kLidarForwardOffset();
    const double sensor_x = X(request.robot) + lidar_forward * std::cos(yaw);
    const double sensor_y = Y(request.robot) + lidar_forward * std::sin(yaw);
    const double evidence_range = std::min(3.0, static_cast<double>(request.scan->range_max));
    const double step = std::max(.5 * map.resolution, .02);
    for (std::size_t beam = 0; beam < request.scan->ranges.size(); ++beam) {
      const double range = request.scan->ranges[beam];
      if (!std::isfinite(range) || range < request.scan->range_min || range > evidence_range) {
        continue;
      }
      const double angle =
          yaw + request.scan->angle_min + static_cast<double>(beam) * request.scan->angle_increment;
      const double usable = std::max(0., range - std::max(.06, .5 * map.resolution));
      for (double distance = 0.; distance < usable; distance += step) {
        const double x = sensor_x + distance * std::cos(angle);
        const double y = sensor_y + distance * std::sin(angle);
        const int col = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
        const int row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
        if (col < 0 || row < 0 || col >= static_cast<int>(map.width) ||
            row >= static_cast<int>(map.height)) {
          continue;
        }
        near_field_free.insert(static_cast<std::size_t>(row) * map.width + col);
      }
    }
  }
  auto& driven_corridor = driven_corridor_;
  driven_corridor.clear();
  const bool entry_plan = request.entry.has_value();
  const double trail_budget_m = request.trail_budget_m;
  if (trail_budget_m > 0. && map.resolution > 0. && request.trail.size() >= 2) {
    // The chassis already drove this corridor. A later scan can pinch the
    // costmap below the planning radius and trap the robot; reopen the
    // driven trail for this one plan. Return uses a long budget so door
    // seals along the outbound path stay plannable. Entry must NOT force
    // occupied cells free — that invented through-wall chords PathCuts then
    // rejected (live: Entry probe rejected reason=path intersects…).
    const double paint_radius_m =
        entry_plan ? .12 : ((request.returning || trail_budget_m > 4.) ? .28 : .16);
    const int radius = static_cast<int>(std::ceil(paint_radius_m / map.resolution));
    double walked = 0.;
    for (std::size_t i = request.trail.size(); i-- > 0;) {
      if (i + 1 < request.trail.size()) {
        walked += std::hypot(request.trail[i].first - request.trail[i + 1].first,
                             request.trail[i].second - request.trail[i + 1].second);
      }
      if (walked > trail_budget_m) {
        break;
      }
      const int trail_col =
          static_cast<int>(std::floor((request.trail[i].first - map.origin_x) / map.resolution));
      const int trail_row =
          static_cast<int>(std::floor((request.trail[i].second - map.origin_y) / map.resolution));
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (dx * dx + dy * dy > radius * radius) {
            continue;
          }
          const int x = trail_col + dx, y = trail_row + dy;
          if (x < 0 || y < 0 || x >= static_cast<int>(map.width) ||
              y >= static_cast<int>(map.height)) {
            continue;
          }
          const std::size_t index = static_cast<std::size_t>(y) * map.width + x;
          // Entry: only reopen unknown / free trail cells, never occupied.
          if (entry_plan && map.cells[index] >= 50) {
            continue;
          }
          driven_corridor.insert(index);
        }
      }
    }
  }
  // Planning uses mapper occupancy as-is (no door-seal carving).
  ExplorationGrid planning_grid;
  planning_grid.width = static_cast<int>(map.width);
  planning_grid.height = static_cast<int>(map.height);
  planning_grid.resolution = map.resolution;
  planning_grid.origin_x = map.origin_x;
  planning_grid.origin_y = map.origin_y;
  planning_grid.cells = map.cells;
  // Locked-mouth entry: paint unknown along frontier → this navigation goal
  // (the room-side probe). Hallway-standoff→frontier pointed the wrong way
  // when the locked mouth was already room-side.
  if (entry_plan && map.resolution > 0.) {
    const ExplorationGoal& mouth = *request.entry;
    const double fx = mouth.frontier_x;
    const double fy = mouth.frontier_y;
    double ax = request.goal.x - fx;
    double ay = request.goal.y - fy;
    if (std::hypot(ax, ay) < .12) {
      ax = fx - mouth.x;
      ay = fy - mouth.y;
    }
    const double alen = std::hypot(ax, ay);
    if (alen > .05) {
      const double ux = ax / alen, uy = ay / alen;
      const double lx = -uy, ly = ux;
      const double res = map.resolution;
      const int width = static_cast<int>(map.width);
      const int height = static_cast<int>(map.height);
      for (double depth = -.10; depth <= 1.20; depth += res) {
        for (double lat = -.14; lat <= .14 + 1e-9; lat += res) {
          const double wx = fx + depth * ux + lat * lx;
          const double wy = fy + depth * uy + lat * ly;
          const int col = static_cast<int>(std::floor((wx - map.origin_x) / res));
          const int row = static_cast<int>(std::floor((wy - map.origin_y) / res));
          if (col < 0 || row < 0 || col >= width || row >= height) {
            continue;
          }
          const std::size_t index = static_cast<std::size_t>(row) * map.width + col;
          if (planning_grid.cells[index] < 0) {
            planning_grid.cells[index] = 0;
          }
        }
      }
    }
  }
  result.free_mask.assign(planning_grid.cells.size(), 0);
  for (std::size_t index = 0; index < planning_grid.cells.size(); ++index) {
    // Free is 0. A real obstacle is 254 and inflates. Unknown is 255:
    // search may not enter it, and it is not a wall, so a frontier
    // viewpoint beside it stays a valid goal.
    const int row = static_cast<int>(index / map.width);
    const int col = static_cast<int>(index % map.width);
    const bool under_robot =
        (col - robot_col) * (col - robot_col) + (row - robot_row) * (row - robot_row) <=
        robot_clearance * robot_clearance;
    const bool under_body =
        (col - robot_col) * (col - robot_col) + (row - robot_row) * (row - robot_row) <=
        body_cells * body_cells;
    // The robot's current footprint is physically known to be free. SLAM's
    // confidence threshold can otherwise leave isolated unknown cells below
    // the base and make every first plan fail its start-collision check.
    // Occupied outside that circle stays lethal so a doorway plan cannot
    // ride a jamb the live-map check will cut.
    const auto cell = planning_grid.cells[index];
    const bool lidar_evidence = cell < 0 && near_field_free.count(index) > 0;
    const bool driven = driven_corridor.count(index) > 0;
    const bool unknown_near_robot = under_robot && cell < 0;
    const bool forced_free =
        cell == 0 || under_body || unknown_near_robot || lidar_evidence || driven;
    result.free_mask[index] = forced_free;
  }
  result.grid = {planning_grid.width, planning_grid.height, planning_grid.resolution,
                 planning_grid.origin_x, planning_grid.origin_y};
  prepared.cells = std::move(planning_grid.cells);
  driven_corridor.clear();
  return prepared;
}
PreparedSnapshot BuildPlanningSnapshot(const ExplorationGrid& map, const SnapshotRequest& request) {
  return PlanningSnapshotBuilder{}.Build(map, request);
}
}  // namespace navigation2d::exploration
