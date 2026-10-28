#include <cmath>

#include "navigation2d/application/exploration/exploration_mission.h"
#include "navigation2d/application/exploration/goal_execution.h"
#include "navigation2d/application/exploration/goal_tracking_actions.h"
#include "navigation2d/application/exploration/motion_output.h"
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/application/exploration/navigation_telemetry.h"
#include "navigation2d/application/exploration/route_failure_history.h"
#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
bool GoalExecution::ObserveArrival(const NavigationState& state, NavigationSession& navigation,
                                   ViewpointArrival& arrival, MotionOutput& motion,
                                   GoalTrackingActions& actions) {
  if (entry_.pending() && !returning_.returning() && !retreat_.active()) {
    if (const auto robot = actions.TourRobot();
        robot && entry_.Satisfied(robot->x, robot->y, actions.ActiveGoal(), explorer_)) {
      actions.AcknowledgeEntry(EntryArrival::kDriving);
      return true;
    }
  }
  if (mission_.phase() == MissionPhase::kExploring && !returning_.returning()) {
    if (const auto robot = actions.TourRobot(); robot && !retreat_.active()) {
      const double distance =
          std::hypot(actions.ActiveGoal().x - robot->x, actions.ActiveGoal().y - robot->y);
      if (distance < .28 && state.path_progress_m >= .05) {
        navigation.Cancel();
        motion.Send({});
        arrival.Near(explorer_, entry_, tour_, actions.ActiveGoal(), *actions.TourRobot(), actions);
        return true;
      }
    }
  }
  return false;
}
bool GoalExecution::GuardCorner(const NavigationState& state, MotionOutput& motion,
                                GoalTrackingActions& actions) {
  if (mission_.phase() == MissionPhase::kExploring && !returning_.returning() &&
      !retreat_.active() && !entry_.pending() && !entry_.locked()) {
    if (const auto robot = actions.TourRobot()) {
      const double clearance = actions.ActivePathMargin(state.global_planning.path_min_clearance_m);
      const double distance =
          std::hypot(actions.ActiveGoal().x - robot->x, actions.ActiveGoal().y - robot->y);
      const auto body = actions.ReadBodyRanges();
      if (distance < 4. * kSwingClearance() && state.path_progress_m >= kPlanningRadius() &&
          actions.IsCornerTipStall(clearance, body.front, body.side)) {
        motion.Send({});
        actions.BreakWallScrapeThrash("driving scrape path into corner tip");
        return true;
      }
    }
  }
  return false;
}
Twist2d GoalExecution::CycleCommand(const NavigationState& state, GoalTrackingActions& actions) {
  const bool safety_held = state.safety_stopped_motion ||
                           state.collision_monitor_action == CollisionMonitorAction::kStop ||
                           state.collision_monitor_action == CollisionMonitorAction::kBlindZoneStop;
  std::optional<EntryApproach> approach;
  if (!safety_held && (entry_.pending() || entry_.locked())) {
    if (actions.TourRobot() && !retreat_.active() && state.status != NavigationStatus::kSucceeded) {
      approach = EntryApproach{*entry_.ActiveMouth(), actions.ActiveGoal(), *actions.TourRobot()};
    }
  }
  return ExecutionCommand(state.command, safety_held, approach, actions);
}
void GoalExecution::Track(NavigationSession& navigation, NavigationTelemetry& telemetry,
                          ViewpointArrival& arrival, RouteFailureHistory& routes,
                          PoseMotionWatch& pose_motion, MotionOutput& motion,
                          GoalTrackingActions& actions, const GoalExecutionClock& clock) {
  if (actions.HandoffToNextTourGoal()) {
    return;
  }
  const auto input = actions.ReadStepInput();
  const auto state = navigation.Step(input.pose, input.measured_velocity, input.timestamp);
  if (WatchImmobility(state, navigation, pose_motion, motion, actions, clock)) {
    return;
  }
  if (ObserveArrival(state, navigation, arrival, motion, actions)) {
    return;
  }
  if (actions.PathCutsMappedObstacle(true)) {
    actions.ReportCutNavigationRoute();
    motion.Send({});
    actions.ClearPublishedNavPaths();
    if (actions.ReplanAfterRouteCut()) {
      return;
    }
    if (Returning()) {
      navigation.Cancel();
      routes.ClearSignatures();
      actions.RequestRecoveryStartReplay();
      actions.HandleReturnRecovery(ReturnRecoveryMode::kPathFailure,
                                   "return path intersected mapped obstacles");
    }
    return;
  }
  actions.CaptureNavTelemetry(state, input.measured_velocity);
  watch_.ObserveProgress(state.path_progress_m, clock);
  if (GuardCorner(state, motion, actions)) {
    return;
  }
  // A committed region-ordered transit is not preempted for a nearer mouth.
  telemetry.NoteCommandBackend(state.controller_diagnostics.backend);
  motion.Send(CycleCommand(state, actions));
  actions.ReportExecutionCycle(state, input.measured_velocity);
  FinishCycle(state, navigation, arrival, routes, actions, clock);
}
}  // namespace navigation2d::exploration
