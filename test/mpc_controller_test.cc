#include <cassert>
#include <cmath>
#include <fstream>

#include "navigation2d/control/mpc_controller.h"
#ifdef NAVIGATION2D_TEST_ACADOS
#include "navigation2d/control/acados_mpc_backend.h"
#endif
#include "navigation2d/costmap/grid_2d.h"

int main() {
  const char* map_path = "/tmp/navigation2d_mpc_test.json";
  std::ofstream output(map_path);
  output << R"({"width":100,"height":60,"resolution":0.1,"cells":[)";
  for (int index = 0; index < 6000; ++index) output << (index ? ",0" : "0");
  output << "]}";
  output.close();

  navigation2d::NavigationConfig config;
  config.mpc_time_steps = 12;
  config.mpc_beam_width = 24;
  navigation2d::LayeredCostmap map(navigation2d::Grid2d::Load(map_path), config);
  navigation2d::Path path;
  for (int index = 0; index <= 50; ++index)
    path.push_back(navigation2d::MakePose2d(1. + index * .1, 2., 0.));
  const auto pose = navigation2d::MakePose2d(1., 2., 0.);
  navigation2d::MpcController controller(config);
  const auto command = controller.Compute(path, pose, {}, map);
  assert(std::isfinite(command.linear));
  assert(std::isfinite(command.angular));
  assert(command.linear > 0.);
  assert(command.linear <= config.max_linear_acceleration * config.control_period + 1e-12);

#ifdef NAVIGATION2D_TEST_ACADOS
  navigation2d::AcadosMpcBackend acados(config);
  assert(acados.available());
  const auto acados_command = acados.Solve(path, pose, {}, {});
  assert(acados_command.has_value());
  assert(std::isfinite(acados_command->linear));
  assert(std::isfinite(acados_command->angular));
#endif

  // The chance-expanded disc blocks the direct topology. The controller must
  // not accelerate into it; it may brake or choose a turning topology.
  const std::vector<navigation2d::PredictedObstacle> obstacles{
      {1.45, 2., 0., 0., .20, .03, .03},
      {1.60, 2.15, -.05, 0., .20, .04, .04},
      {1.30, 2.30, .03, -.02, .18, .02, .03},
      {1.75, 2.40, 0., -.04, .20, .03, .02},
      {2.00, 2.50, 0., 0., .20, .03, .03}};
  const auto constrained = controller.Compute(path, pose, {}, map, obstacles);
  assert(constrained.linear <= command.linear);
#ifdef NAVIGATION2D_TEST_ACADOS
  // More tracker objects than generated constraint slots must remain safe and
  // dimensionally valid; the backend ranks/fills its slots per stage.
  const auto multi_obstacle_command = acados.Solve(path, pose, {}, obstacles);
  assert(multi_obstacle_command.has_value());
  assert(std::isfinite(multi_obstacle_command->linear));
  assert(std::isfinite(multi_obstacle_command->angular));
#endif
}
