#pragma once
#include "navigation2d/application/exploration/goal_failure_actions.h"
namespace navigation2d::exploration {
class ExplorationMission;
class ReturnCoordinator;
class PoseMotionWatch;
class RouteFailureHistory;
struct ImmobilityRecoveryObservation {
  double robot_x, robot_y, path_progress_m;  // World-frame metres.
  bool entry_drive, tip_wedged;
};
// One failed-navigation stage. Explicit, lifetime-stable collaborators retain
// their own state; this coordinator holds no mutable context or duplicate flags.
// Returning/phase are deliberately read live after synchronous callbacks.
class GoalFailureExecution {
 public:
  GoalFailureExecution(ExplorationMission& mission, ReturnCoordinator& returning,
                       EntryCoordinator& entry, TourExecution& tour, GoalRecoverySession& recovery,
                       RetreatExecution& retreat, FrontierExplorer& explorer,
                       const GoalExecutionWatch& watch)
      : mission_(mission),
        returning_(returning),
        entry_(entry),
        tour_(tour),
        recovery_(recovery),
        retreat_(retreat),
        explorer_(explorer),
        watch_(watch) {}
  // Consume two consecutive mapped-route rejects before trying local entry
  // or cooling/leaving a tip. Prepared-route acceptance resets this budget.
  bool HandleMappedThrash(RouteFailureHistory&, GoalFailureActions&);
  // Cool/demote a pinched viewpoint, discard it and continue with bounded
  // peel, distant-mouth rescue, or delayed selection, in that priority.
  void LeavePinch(const char* reason, GoalFailureActions&);
  // Caller has detected immobility, sent stop, cancelled navigation and
  // debounced the pose watch. Branch priority is return, entry, open, tip.
  void RecoverImmobility(PoseMotionWatch&, ImmobilityRecoveryObservation, GoalFailureActions&);
  // FailureDue was already evaluated. Body ranges are current metres; clock
  // shares the watch's mission time domain. No command precedes a stop/retry.
  void Resolve(const NavigationState&, double front_m, double side_m, const GoalExecutionClock&,
               GoalFailureActions&);

 private:
  bool Returning() const;
  bool HandleMonitorStop(const NavigationState&, double front_m, double side_m,
                         const GoalExecutionClock&, GoalFailureActions&);
  bool HandleSegmentStall(const NavigationState&, bool first_segment, double front_m, double side_m,
                          GoalFailureActions&);
  void FinishFailure(const NavigationState&, bool first_segment, bool mid_path,
                     GoalFailureActions&);
  ExplorationMission& mission_;
  ReturnCoordinator& returning_;
  EntryCoordinator& entry_;
  TourExecution& tour_;
  GoalRecoverySession& recovery_;
  RetreatExecution& retreat_;
  FrontierExplorer& explorer_;
  const GoalExecutionWatch& watch_;
};
}  // namespace navigation2d::exploration
