#include "navigation2d/application/exploration/navigation_dispatch_execution.h"

#include <cmath>
#include <exception>

#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
bool NavigationDispatch::Start(const ExplorationGoal& requested_goal, bool returning,
                               bool latest_map_retry, NavigationSession& navigation,
                               NavigationDispatchRuntime& runtime) {
  policy_.BeginNavigationAttempt(latest_map_retry);
  runtime.BeginNavigationPreparation();
  ExplorationGoal goal = requested_goal;
  if (!returning) {
    if (const auto settled = explorer_.SettleGoal(goal)) {
      if (std::hypot(settled->x - goal.x, settled->y - goal.y) > 1e-6) {
        runtime.ReportSettledGoal(goal, *settled);
      }
      goal = *settled;
    }
  }
  // Preparation/configuration errors retain their original outer boundary.
  const auto map = runtime.PrepareNavigationMap(goal, returning);
  const auto purpose = map.entry
                           ? NavigationPurpose::kEntry
                           : (returning ? NavigationPurpose::kReturn : NavigationPurpose::kExplore);
  const bool pinch_retry = policy_.active(DispatchPolicy::RetryKind::kPinch);
  const bool pinch_plan = pinch_retry && !map.entry && !returning;
  const auto config = BuildNavigationProfile({.resolution = map.resolution,
                                              .entry = map.entry,
                                              .pinch_retry = pinch_retry,
                                              .returning = returning,
                                              .home_goal = map.home_goal},
                                             profiles_.For(purpose));
  try {
    navigation.Begin(config, map.path,
                     MakePose2d(goal.x - map.origin_x, goal.y - map.origin_y, goal.yaw),
                     runtime.CooledNavigationSignatures(),
                     {.heading_policy = profiles_.For(purpose).heading_policy});
    runtime.ObservePreparedNavigation();
    const auto input = runtime.ReadNavigationProbe();
    const auto probe = navigation.Probe(input.pose, input.measured_velocity, input.timestamp);
    const auto verdict = policy_.Evaluate(goal,
                                          {.entry = map.entry,
                                           .returning = returning,
                                           .retreat = runtime.DispatchRetreating(),
                                           .pinch = pinch_plan},
                                          latest_map_retry, probe, runtime);
    if (verdict != DispatchVerdict::kAcceptPrepared) {
      return verdict == DispatchVerdict::kAcceptedRetry;
    }
    // The first controller sample may legitimately be zero at goal handoff.
    // Executability remains a temporal property of navigation execution.
    runtime.SetDispatchTimeout(
        DispatchPolicy::GoalTimeout(returning, probe.state.global_path_length_m));
    target_.Accept(navigation, goal, profiles_.For(purpose));
    runtime.CommitAcceptedNavigation(goal, returning, probe.state);
    return true;
  } catch (const std::exception& error) {
    runtime.RecordNavigationException(error.what());
    navigation.Cancel();
    return false;
  }
}
bool NavigationDispatch::ReplanLatest(const char* reason, bool returning,
                                      NavigationSession& navigation,
                                      NavigationDispatchRuntime& runtime) {
  return policy_.ReplanOnLatestMap([&](auto attempt) {
    navigation.Cancel();
    runtime.ReportDispatchReplan(
        {DispatchReplanNotice::Kind::kLatestMap, reason, returning, attempt.number, attempt.limit});
    return Start(goal(), returning, true, navigation, runtime);
  });
}
bool NavigationDispatch::ReplanAfterCut(bool returning, NavigationSession& navigation,
                                        NavigationDispatchRuntime& runtime) {
  return policy_.ReplanAfterRouteCut([&](auto attempt) {
    navigation.Cancel();
    runtime.ResumePreparedTrailReplay();
    runtime.ReportDispatchReplan(
        {DispatchReplanNotice::Kind::kRouteCut, nullptr, returning, attempt.number, attempt.limit});
    return Start(goal(), returning, true, navigation, runtime);
  });
}
}  // namespace navigation2d::exploration
