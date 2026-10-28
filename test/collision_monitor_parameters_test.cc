#include "navigation2d/control/collision_monitor_parameters.h"

#include <cassert>

#include "navigation2d/test/collision_monitor_workload.h"

int main() {
  for (int variant = 0; variant < 16; ++variant) {
    auto source = navigation2d::test::CollisionWorkloadConfig(variant);
    navigation2d::CollisionMonitor legacy(source);
    auto scoped = navigation2d::CollisionMonitor::WithParameters(
        navigation2d::CollisionMonitorParametersFrom(source));
    // Both monitors own snapshots: a later caller mutation cannot disable
    // safety, change the body polygon or invalidate a borrowed footprint.
    source.collision_monitor_enabled = false;
    source.footprint.assign(32, Eigen::Vector2d(10., 10.));
    const auto expected = navigation2d::test::CollisionWorkload(legacy);
    const auto actual = navigation2d::test::CollisionWorkload(scoped);
    assert(expected.size() == actual.size());
    for (std::size_t i = 0; i < actual.size(); ++i) {
      assert(expected[i].action == actual[i].action);
      assert(expected[i].command.linear == actual[i].command.linear);
      assert(expected[i].command.angular == actual[i].command.angular);
      assert(expected[i].min_distance_m == actual[i].min_distance_m);
      assert(expected[i].time_to_collision_s == actual[i].time_to_collision_s);
    }
  }
}
