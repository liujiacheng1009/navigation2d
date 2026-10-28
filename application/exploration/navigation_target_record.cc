#include "navigation2d/application/exploration/navigation_target_record.h"

#include <limits>
#include <type_traits>

#include "navigation2d/application/exploration/navigation_profile.h"
#include "navigation2d/application/exploration/navigation_session.h"

namespace navigation2d::exploration {
NavigationTargetRecord::NavigationTargetRecord() {
  const AlgorithmSelection defaults;
  algorithms_ = {defaults.global, defaults.local};
}

void NavigationTargetRecord::Accept(NavigationSession& session, const ExplorationGoal& world_goal,
                                    const AlgorithmSelection& configured_algorithms) {
  static_assert(std::is_nothrow_copy_assignable_v<ExplorationGoal>);
  AlgorithmNames next{configured_algorithms.global, configured_algorithms.local};
  session.Accept();
  goal_ = world_goal;
  algorithms_.global.swap(next.global);
  algorithms_.local.swap(next.local);
  has_target_ = true;
}

void NavigationTargetRecord::Clear() noexcept {
  static_assert(std::is_nothrow_copy_assignable_v<ExplorationGoal>);
  goal_ = ExplorationGoal{};
  goal_.x = goal_.y = goal_.frontier_x = goal_.frontier_y =
      std::numeric_limits<double>::quiet_NaN();
  has_target_ = false;
}
}  // namespace navigation2d::exploration
