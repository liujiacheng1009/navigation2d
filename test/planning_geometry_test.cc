#include <cassert>
#include <cmath>
#include <fstream>
#include <vector>

#include "navigation2d/geometry/robot_geometry.h"
#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/motion_primitives.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  navigation2d::NavigationConfig config;
  auto map = navigation2d::test::CommonPlannerMap("geometry");
  assert(!map.lethal(2.26, 2.05, 0.));
  assert(map.lethal(2.26, 2.05, .22));
  const navigation2d::planning_internal::DistanceField field(map);
  assert(field.distance(2., 2.) == 0.);
  assert(std::abs(field.distance(2.5, 2.) - .5) < .11);
  assert(!field.CircleCollisionFree(2.26, 2.05, .22));
  assert(field.CircleCollisionFree(3., 3., .22));
  // A cell beside unknown is a frontier, not a wall. Margin checks must not
  // see that adjacency as contact; collision queries still do.
  {
    const char* frontier_path = "/tmp/navigation2d_frontier_distance.json";
    std::ofstream frontier_out(frontier_path);
    frontier_out << R"({"width":5,"height":1,"resolution":0.05,"cells":[0,0,0,0,255]})";
    frontier_out.close();
    navigation2d::LayeredCostmap frontier_map(navigation2d::Grid2d::Load(frontier_path), config);
    const navigation2d::planning_internal::DistanceField as_wall(frontier_map);
    const navigation2d::planning_internal::DistanceField as_free(frontier_map,
                                                                 /*unknown_is_obstacle=*/false);
    // Cell beside the unknown frontier: one resolution step, not a wall.
    assert(std::abs(as_wall.distance(0.175, 0.025) - 0.05) < 1e-6);
    assert(as_free.distance(0.175, 0.025) > 1.);
    assert(navigation2d::PathClearsOpen(0.07));
    assert(!navigation2d::PathClearsOpen(0.));
    assert(!navigation2d::PathClearsPinch(0.));
  }

  const std::vector<Eigen::Vector2d> rectangle{{-.30, -.12}, {.30, -.12}, {.30, .12}, {-.30, .12}};
  const navigation2d::planning_internal::FootprintLookup footprint(rectangle, 16, .1);
  assert(footprint.CollisionFree(map, navigation2d::MakePose2d(3., 3., 0.)));
  assert(!footprint.CollisionFree(map, navigation2d::MakePose2d(2.25, 2., 0.)));
  assert(!footprint.SweptCollisionFree(
      map, {navigation2d::MakePose2d(3., 3., 0.), navigation2d::MakePose2d(2.25, 2., 0.)}));

  const navigation2d::planning_internal::DifferentialDrivePrimitiveSet primitives(16, .1, .3, true);
  for (int bin = 0; bin < primitives.yaw_bins(); ++bin) {
    const auto& motions = primitives.FromYawBin(bin);
    assert(motions.size() == 10);
    bool has_forward = false, has_reverse = false, has_rotate = false;
    for (const auto& motion : motions) {
      assert(!motion.samples.empty());
      assert(motion.samples.back().translation().isApprox(
          Eigen::Vector2d(motion.dx_cells * .1, motion.dy_cells * .1), 1e-12));
      has_forward =
          has_forward || motion.type == navigation2d::planning_internal::MotionType::kForward;
      has_reverse =
          has_reverse || motion.type == navigation2d::planning_internal::MotionType::kReverse;
      has_rotate =
          has_rotate || motion.type == navigation2d::planning_internal::MotionType::kRotate;
    }
    assert(has_forward && has_reverse && has_rotate);
  }
}
