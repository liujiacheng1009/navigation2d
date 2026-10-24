#include "navigation2d/control/pursuit_alignment.h"

#include <cassert>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/test/temporary_directory.h"
int main() {
  using namespace navigation2d;
  using namespace navigation2d::pursuit_internal;
  NavigationConfig config;
  config.map_resolution = .1;
  config.control_period = .05;
  config.max_linear_acceleration = .5;
  config.max_angular_acceleration = 1.;
  config.max_angular_velocity = 1.;
  config.regulated_min_speed = .08;
  test::TemporaryDirectory directory;
  const auto load = [&](bool blocked) {
    const auto path = directory.path() / (blocked ? "blocked.json" : "open.json");
    std::ofstream file(path);
    file << R"({"width":60,"height":60,"resolution":0.1,"rectangles":)"
         << (blocked ? "[[0,0,60,60,100]]" : "[]") << '}';
    file.close();
    if (!file) {
      throw std::runtime_error("cannot write pursuit fixture");
    }
    return LayeredCostmap(Grid2d::Load(path.string()), config);
  };
  const auto free_map = load(false), blocked_map = load(true);
  const auto pose = MakePose2d(2., 2., 0.);
  const Path path{pose, MakePose2d(3., 2., 0.)};
  Alignment alignment(ParametersFrom(config).alignment);
  const auto step = [&](Twist2d measured, const Pose2d& observed) {
    alignment.BeginCycle();
    return alignment.Continue(path, 0, observed, measured, free_map);
  };
  assert(!step({}, pose));
  const auto brake = alignment.StartAndBrake(path, 0, pose, {.2, .1}, free_map);
  assert(std::abs(brake.linear - .175) < 1e-12 && std::abs(brake.angular - .05) < 1e-12);
  assert(alignment.Diagnostics().intentional_stop && alignment.Diagnostics().fallback_level == 5);
  assert(step({}, pose));
  assert(alignment.Diagnostics().maneuver == ControllerManeuver::kStopping);
  assert(step({}, pose));
  assert(alignment.Diagnostics().maneuver == ControllerManeuver::kRotateToPath);
  assert(!alignment.Diagnostics().intentional_stop);
  assert(!step({}, pose));
  assert(alignment.Diagnostics().maneuver == ControllerManeuver::kTracking);
  // The stopped deadband is strict and requires two consecutive observations.
  alignment.StartAndBrake(path, 0, pose, {}, free_map);
  assert(step({}, pose));
  assert(step({.015, 0.}, pose));
  assert(step({}, pose));
  assert(step({0., .05}, pose));
  assert(alignment.Diagnostics().maneuver == ControllerManeuver::kStopping);
  assert(step({}, pose));
  assert(step({}, pose));
  // Rotate-only means exactly zero translation even with residual wheel speed.
  const auto rotate = step({.2, 0.}, MakePose2d(2., 2., .5));
  assert(rotate && rotate->linear == 0. && rotate->angular < 0.);
  alignment.BeginCycle();
  alignment.NewPath();
  assert(!alignment.Continue(path, 0, pose, {}, free_map));
  // A costmap change between aligned observations revalidates the forward probe.
  alignment.StartAndBrake(path, 0, pose, {}, free_map);
  assert(step({}, pose));
  assert(step({}, pose));
  alignment.BeginCycle();
  const auto rejected = alignment.Continue(path, 0, pose, {}, blocked_map);
  assert(rejected && rejected->linear == 0. && rejected->angular == 0.);
  assert(alignment.Diagnostics().status == ControllerSolveStatus::kUnsafe);
  assert(alignment.Diagnostics().fallback_level == 7);
  assert(step({}, pose));  // Restoring the map alone cannot clear an infeasible route.
  assert(alignment.Diagnostics().status == ControllerSolveStatus::kUnsafe);
  alignment.NewPath();
  alignment.BeginCycle();
  assert(!alignment.Continue(path, 0, pose, {}, free_map));
  alignment.BeginCycle();
  const auto impossible = alignment.StartAndBrake(path, 0, pose, {.2, 0.}, blocked_map);
  assert(impossible.linear == 0. && impossible.angular == 0.);
  assert(alignment.Diagnostics().maneuver == ControllerManeuver::kRouteInfeasible);
}
