#include <cassert>
#include <cmath>
#include <fstream>

#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/costmap/layered_costmap.h"

int main() {
  // Regression for the warehouse return stall. The base reaches a shelf
  // corner with forward velocity while the validated route turns north. The
  // desired pure rotation is safe, but applying a one-cycle acceleration
  // limit retains enough eastbound velocity to hit the shelf. This must be a
  // finite stopping -> rotate-to-path manoeuvre, not an unlabelled stage-4
  // zero command which the navigation watchdog replans forever.
  const char* corner_map_path = "/tmp/navigation2d_rpp_corner_test.json";
  std::ofstream corner_output(corner_map_path);
  corner_output << R"({"width":80,"height":80,"resolution":0.05,"cells":[)";
  for (int index = 0; index < 6400; ++index) {
    corner_output << (index ? ",0" : "0");
  }
  corner_output << "]}";
  corner_output.close();
  // Shelf-corner stop→rotate regression uses the historical radii that made
  // the eastbound accel clamp collide; CAD doorway cases below use 0.22 m.
  navigation2d::NavigationConfig corner_config;
  corner_config.map_resolution = .05;
  corner_config.robot_radius = .28;
  corner_config.inflation_radius = .32;
  corner_config.collision_horizon = .35;
  navigation2d::LayeredCostmap corner_costmap(navigation2d::Grid2d::Load(corner_map_path),
                                              corner_config);
  corner_costmap.MarkObstacle(1.325, 1.0);
  navigation2d::Path corner_path;
  for (int index = 0; index <= 20; ++index) {
    corner_path.push_back(navigation2d::MakePose2d(1., 1. + .05 * index, 1.57));
  }
  navigation2d::RegulatedPurePursuit corner_rpp(corner_config);
  const auto corner_pose = navigation2d::MakePose2d(1., 1., 0.);
  auto corner_command = corner_rpp.Compute(corner_path, corner_pose, {.24, 0.}, corner_costmap);
  auto corner_diagnostics = corner_rpp.Diagnostics();
  assert(corner_command.linear == 0.);
  assert(corner_command.angular == 0.);
  assert(corner_diagnostics.maneuver == navigation2d::ControllerManeuver::kStopping);
  assert(corner_diagnostics.intentional_stop);
  corner_command = corner_rpp.Compute(corner_path, corner_pose, {}, corner_costmap);
  assert(corner_rpp.Diagnostics().maneuver == navigation2d::ControllerManeuver::kStopping);
  corner_command = corner_rpp.Compute(corner_path, corner_pose, {}, corner_costmap);
  corner_diagnostics = corner_rpp.Diagnostics();
  assert(corner_command.linear == 0.);
  assert(corner_command.angular > 0.);
  assert(corner_diagnostics.maneuver == navigation2d::ControllerManeuver::kRotateToPath);
  const auto aligned_corner_pose = navigation2d::MakePose2d(1., 1., .5 * std::acos(-1.));
  corner_rpp.Compute(corner_path, aligned_corner_pose, {}, corner_costmap);
  corner_command = corner_rpp.Compute(corner_path, aligned_corner_pose, {}, corner_costmap);
  assert(corner_command.linear > 0.);
  assert(corner_rpp.Diagnostics().maneuver == navigation2d::ControllerManeuver::kTracking);

  // The local controller checks the physical footprint continuously.  A pose
  // whose centre is 0.275 m from the occupied cell must be rejected before
  // the base reaches contact; the raster half-diagonal is applied by the
  // global planner/path validator, not twice in RPP.
  corner_costmap.ClearObstacle(1.325, 1.0);
  corner_costmap.MarkObstacle(1.025, 1.0);
  navigation2d::RegulatedPurePursuit margin_rpp(corner_config);
  // Centre 0.275 m from the marked cell (comment contract); with r=0.28 the
  // projected footprint is already in contact.
  assert(margin_rpp.CollisionImminent(navigation2d::MakePose2d(1.300, 1.0, M_PI), {.048, 0.},
                                      corner_costmap));
}
