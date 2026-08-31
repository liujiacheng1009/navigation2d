#include <cassert>
#include <cmath>
#include <fstream>

#include "navigation2d/control/mpc_controller.h"
#include "navigation2d/control/path_tracking.h"
#ifdef NAVIGATION2D_TEST_ACADOS
#include "navigation2d/control/acados_mpc_backend.h"
#endif
#include "navigation2d/costmap/grid_2d.h"

int main() {
  // Regression: reference lookup must remain bounded for long global paths,
  // including its first lookup after a replan.
  navigation2d::Path long_path;
  constexpr int kLongPathPoints = 10000;
  for (int index = 0; index < kLongPathPoints; ++index)
    long_path.push_back(navigation2d::MakePose2d(index * .05, 0., 0.));
  navigation2d::control_internal::PathSearchState path_search;
  auto nearest = navigation2d::control_internal::FindNearestPathPoint(
      long_path, navigation2d::MakePose2d(321.05, 0., 0.), &path_search);
  assert(nearest == 6421);
  assert(path_search.last_evaluations < 4 * 128);
  nearest = navigation2d::control_internal::FindNearestPathPoint(
      long_path, navigation2d::MakePose2d(322.05, 0., 0.), &path_search);
  assert(nearest == 6441);
  assert(path_search.last_evaluations <= 64 + 192 + 1);
  // A large localization jump falls outside the progress window, but still
  // re-localizes without ever reading the whole global path.
  nearest = navigation2d::control_internal::FindNearestPathPoint(
      long_path, navigation2d::MakePose2d(430.05, 0., 0.), &path_search);
  assert(nearest == 8601);
  assert(path_search.last_evaluations < 7 * 128);

  const char* map_path = "/tmp/navigation2d_mpc_test.json";
  std::ofstream output(map_path);
  output << R"({"width":100,"height":60,"resolution":0.1,"cells":[)";
  for (int index = 0; index < 6000; ++index) output << (index ? ",0" : "0");
  output << "]}";
  output.close();

  navigation2d::NavigationConfig config;
  config.mpc_time_steps = 12;
  navigation2d::LayeredCostmap map(navigation2d::Grid2d::Load(map_path), config);
  navigation2d::Path path;
  for (int index = 0; index <= 50; ++index)
    path.push_back(navigation2d::MakePose2d(1. + index * .1, 2., 0.));
  const auto pose = navigation2d::MakePose2d(1., 2., 0.);
  navigation2d::MpcController controller(config);
  navigation2d::Path alternate = path;
  for (auto& point : alternate) point.translation().y() += .15;
  controller.SetGuidanceCandidates({{7, alternate, 0.}});
  const auto command = controller.Compute(path, pose, {}, map);
  assert(std::isfinite(command.linear));
  assert(std::isfinite(command.angular));
  assert(command.linear > 0.);
  assert(command.linear <= config.max_linear_acceleration * config.control_period + 1e-12);
  assert(controller.Diagnostics().backend != navigation2d::ControllerBackend::kNone);

#ifdef NAVIGATION2D_TEST_ACADOS
  navigation2d::AcadosMpcBackend acados(config);
  assert(acados.available());
  const auto acados_command = acados.Solve(path, pose, {}, map, {});
  assert(acados_command.has_value());
  assert(std::isfinite(acados_command->linear));
  assert(std::isfinite(acados_command->angular));
  const auto warm_command = acados.Solve(path, pose, *acados_command, map, {});
  assert(warm_command.has_value());
  assert(acados.Diagnostics().solver_us > 0.);
  assert(std::isfinite(acados.Diagnostics().kkt_residual));

  // Exercise the production Acados integration: its first reference lookup on
  // a long global path remains bounded, rather than scanning every waypoint.
  navigation2d::Path acados_long_path;
  for (int index = 0; index < kLongPathPoints; ++index)
    acados_long_path.push_back(navigation2d::MakePose2d(1. + index * .0005, 2., 0.));
  navigation2d::AcadosMpcBackend long_acados(config);
  const auto long_command = long_acados.Solve(
      acados_long_path, navigation2d::MakePose2d(3.5, 2., 0.), {}, map, {});
  assert(long_command.has_value());
  assert(long_acados.Diagnostics().path_search_evaluations < 4 * 128);
#endif

  // More tracker objects than generated constraint slots must be ranked and
  // passed to the solver without making this otherwise feasible test setup
  // infeasible during its short horizon.
  const std::vector<navigation2d::PredictedObstacle> obstacles{
      {4.50, 2., 0., 0., .20, .03, .03},
      {4.65, 2.15, -.05, 0., .20, .04, .04},
      {4.35, 2.30, .03, -.02, .18, .02, .03},
      {4.80, 2.40, 0., -.04, .20, .03, .02},
      {5.00, 2.50, 0., 0., .20, .03, .03}};
  const auto constrained = controller.Compute(path, pose, {}, map, obstacles);
  assert(constrained.linear <= command.linear);
#ifdef NAVIGATION2D_TEST_ACADOS
  // The backend ranks/fills its fixed constraint slots per stage.
  const auto multi_obstacle_command = acados.Solve(path, pose, {}, map, obstacles);
  assert(multi_obstacle_command.has_value());
  assert(std::isfinite(multi_obstacle_command->linear));
  assert(std::isfinite(multi_obstacle_command->angular));
#endif
}
