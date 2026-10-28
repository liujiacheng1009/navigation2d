#pragma once
#include "navigation2d/application/exploration/dispatch_policy.h"
#include "navigation2d/application/exploration/navigation_dispatch_runtime.h"
#include "navigation2d/application/exploration/navigation_profile.h"
#include "navigation2d/application/exploration/navigation_target_record.h"
namespace navigation2d {
class FrontierExplorer;
}
namespace navigation2d::exploration {
class NavigationSession;
// Owns accepted target identity and dispatch retry scopes. Explorer and
// immutable profiles outlive this single-writer role; no mutable context or
// ROS state is retained. Session ownership stays with execution/handoff.
class NavigationDispatch {
 public:
  NavigationDispatch(const FrontierExplorer& explorer, const NavigationProfiles& profiles)
      : explorer_(explorer), profiles_(profiles) {}
  NavigationDispatch(const NavigationDispatch&) = delete;
  NavigationDispatch& operator=(const NavigationDispatch&) = delete;
  bool Start(const ExplorationGoal&, bool returning, bool latest_map_retry, NavigationSession&,
             NavigationDispatchRuntime&);
  bool ReplanLatest(const char* reason, bool returning, NavigationSession&,
                    NavigationDispatchRuntime&);
  bool ReplanAfterCut(bool returning, NavigationSession&, NavigationDispatchRuntime&);
  void ResetLatestMapReplans() { policy_.ResetLatestMapReplans(); }
  bool HasRouteCutReplans() const { return policy_.HasRouteCutReplans(); }
  const ExplorationGoal& goal() const& { return target_.legacy_goal(); }
  const ExplorationGoal& goal() const&& = delete;
  const AlgorithmNames& algorithms() const& { return target_.algorithms(); }
  const AlgorithmNames& algorithms() const&& = delete;
  void ClearGoal() { target_.Clear(); }

 private:
  const FrontierExplorer& explorer_;
  const NavigationProfiles& profiles_;
  DispatchPolicy policy_;
  NavigationTargetRecord target_;
};
}  // namespace navigation2d::exploration
