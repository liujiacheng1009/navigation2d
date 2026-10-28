#pragma once
#include "navigation2d/application/exploration/alcove_exit.h"
#include "navigation2d/application/exploration/execution_command.h"
#include "navigation2d/application/exploration/goal_failure_actions.h"
#include "navigation2d/application/exploration/recovery_policy.h"
#include "navigation2d/application/exploration/return_coordinator.h"
#include "navigation2d/application/exploration/viewpoint_arrival.h"
#include "navigation2d/geometry/pose_2d.h"
namespace navigation2d::exploration {
struct GoalStepInput {
  Pose2d pose;  // Navigation snapshot's local frame, metres/radians.
  Twist2d measured_velocity;
  double timestamp;  // Seconds in the mission clock domain.
};
// Existing child-role effects share one adapter; the execution coordinator
// owns their ordering. ReadStepInput preserves pose->clock->velocity order.
class GoalTrackingActions : public GoalFailureActions,
                            public ViewpointArrivalActions,
                            public ExecutionCommandEvidence,
                            public AlcoveExitActions {
 public:
  void CancelRecoveryNavigation() override = 0;
  void LockEntryMouth(const ExplorationGoal&) override = 0;
  void PrepareDoorjambRetry() override = 0;
  bool StartEntryProbe() override = 0;
  bool EscapeOrShortRetreat(const char*, const char*) override = 0;
  void SelectAfterEntry() override = 0;
  void PublishStatus(const std::string&, const std::string&) override = 0;
  void AbandonLockedEntryMouth(const std::string&) override = 0;
  virtual void ReportAcceptedNavigation(const ExplorationGoal&, const NavigationState&) = 0;
  virtual bool HandoffToNextTourGoal() = 0;
  virtual GoalStepInput ReadStepInput() const = 0;
  virtual bool PoseImmobilityWatchdog(const NavigationState&) = 0;
  virtual bool PathCutsMappedObstacle(bool execution_tick) const = 0;
  virtual void ReportCutNavigationRoute() = 0;
  virtual void ClearPublishedNavPaths() = 0;
  virtual bool ReplanAfterRouteCut() = 0;
  virtual void HandleReturnRecovery(ReturnRecoveryMode, const char* message) = 0;
  virtual BodyRanges ReadBodyRanges() const = 0;
  virtual void CaptureNavTelemetry(const NavigationState&, const Twist2d&) = 0;
  // Output throttling/recovery notices retain their adapter's lazy clock reads.
  virtual void ReportExecutionCycle(const NavigationState&, Twist2d measured_velocity) = 0;
  virtual void CompleteReturnNavigation() = 0;
  virtual void CompleteRetreatNavigation() = 0;
};
}  // namespace navigation2d::exploration
