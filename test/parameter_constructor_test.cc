#include <cassert>
#include <concepts>

#include "navigation2d/control/acados_mpc_backend.h"
#include "navigation2d/control/dwa_controller.h"
#include "navigation2d/control/mpc_controller.h"
#include "navigation2d/control/mppi_controller.h"
#include "navigation2d/control/nav2_mppi_core.h"
#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/planning/path_smoother.h"
#include "navigation2d/planning/state_lattice_planner.h"
#include "navigation2d/test/planner_test_maps.h"

int main() {
  using namespace navigation2d;
  static_assert(std::constructible_from<DwaController, DwaParameters>);
  static_assert(std::constructible_from<MppiController, MppiParameters>);
  static_assert(std::constructible_from<control_internal::Nav2MppiCore, MppiRolloutParameters>);
  static_assert(std::constructible_from<AcadosMpcBackend, AcadosParameters>);
  static_assert(std::constructible_from<StateLatticePlanner, StateLatticeParameters>);
  static_assert(std::constructible_from<planning_internal::ConstrainedPathSmoother,
                                        planning_internal::SmootherParameters>);
  // These legacy expressions must select NavigationConfig defaults. An
  // unconstrained second aggregate overload makes them fail to compile.
  DwaController dwa({});
  MppiController mppi({});
  control_internal::Nav2MppiCore rollout({});
  AcadosMpcBackend acados({});
  MpcController mpc({});
  RegulatedPurePursuit pursuit({});
  StateLatticePlanner lattice({});
  planning_internal::ConstrainedPathSmoother smoother({});
  const auto pose = MakePose2d(1., 1., 0.);
  const auto samples = rollout.Generate(pose, {});
  assert(samples.x.rows() == 256 && samples.x.cols() == 30);
  auto map = test::CommonPlannerMap("parameter_constructors");
  Path path{pose, MakePose2d(1.8, 1., 0.), MakePose2d(2.5, 1., 0.)};
  NavigationConfig config;
  DwaController explicit_defaults(config);
  const auto expected = explicit_defaults.Compute(path, pose, {}, map);
  const auto actual = dwa.Compute(path, pose, {}, map);
  assert(actual.linear == expected.linear && actual.angular == expected.angular);
  config.mppi_batch_size = 32;
  config.mppi_time_steps = 8;
  MppiController legacy(config), typed(MppiParametersFrom(config));
  const auto first = legacy.Compute(path, pose, {}, map);
  const auto second = typed.Compute(path, pose, {}, map);
  assert(first.linear == second.linear && first.angular == second.angular);
  const auto smoothed = smoother.Smooth(path, map, config.robot_radius);
  planning_internal::ConstrainedPathSmoother typed_smoother(
      planning_internal::SmootherParametersFrom(config));
  const auto typed_path = typed_smoother.Smooth(path, map, config.robot_radius);
  assert(smoothed.size() == typed_path.size());
  for (std::size_t index = 0; index < smoothed.size(); ++index) {
    assert(X(smoothed[index]) == X(typed_path[index]));
    assert(Y(smoothed[index]) == Y(typed_path[index]));
    assert(Yaw(smoothed[index]) == Yaw(typed_path[index]));
  }
}
