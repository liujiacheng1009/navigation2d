#include <cassert>
#include <cmath>
#include <limits>

#include "navigation2d/control/collision_latch.h"
#include "navigation2d/control/range_observation.h"

namespace {
template <typename T>
concept BorrowsTemporary = requires { T{}.points(); };
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace collision_internal;
  static_assert(!BorrowsTemporary<RangeObservation>);
  RangeObservation observation;
  assert(!observation.FreshAt(0., .25));
  const double pi = std::acos(-1.);
  LaserScan scan{
      0.,
      pi / 2.,
      .1,
      2.,
      {.5, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::infinity(), 3., .05}};
  observation.UpdateLaserScan(MakePose2d(1., 2., pi / 2.), scan);
  assert(observation.points().size() == 1);
  assert((observation.points().front() - Eigen::Vector2d(1., 2.5)).norm() < 1e-12);
  assert(observation.blind_zone_points().size() == 2);
  for (const auto& point : observation.blind_zone_points()) {
    assert((point - Eigen::Vector2d(.9, 2.)).norm() < 1e-12);
  }
  // Arrival is acknowledged by the filter clock, not by the update call.
  assert(observation.FreshAt(10., .25));
  assert(observation.FreshAt(10.25, .25));
  assert(!observation.FreshAt(10.251, .25));
  assert(observation.FreshAt(9., .25));  // Preserve legacy backward-clock behavior.
  PointCloud2d cloud;
  cloud.range_max = 1.;
  cloud.points = {{.2, .3}, {std::numeric_limits<double>::quiet_NaN(), .3}, {4., 0.}};
  observation.UpdatePointCloud(MakePose2d(2., 1., pi), cloud);
  assert(observation.blind_zone_points().empty());
  assert(observation.points().size() == 1);
  assert((observation.points().front() - Eigen::Vector2d(1.8, .7)).norm() < 1e-12);
  assert(observation.FreshAt(20., .25));
  assert(!observation.FreshAt(20.251, .25));
  cloud.points.clear();
  observation.UpdatePointCloud(MakePose2d(0., 0., 0.), cloud);
  assert(observation.points().empty() && observation.FreshAt(30., .25));

  using Action = CollisionMonitorAction;
  CollisionLatch latch;
  assert(latch.Update(Action::kSlowdown, 2, 2) == Action::kNone);
  assert(latch.Update(Action::kStop, 2, 2) == Action::kStop);
  assert(latch.Update(Action::kNone, 2, 2) == Action::kStop);
  assert(latch.Update(Action::kBlindZoneStop, 2, 2) == Action::kStop);
  assert(latch.Update(Action::kBlindZoneStop, 2, 2) == Action::kBlindZoneStop);
  assert(latch.Update(Action::kSlowdown, 2, 2) == Action::kSlowdown);
  assert(latch.Update(Action::kNone, 2, 2) == Action::kSlowdown);
  assert(latch.Update(Action::kNone, 2, 2) == Action::kNone);
}
