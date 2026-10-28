#pragma once
#include <optional>
#include <span>
#include <utility>

#include "navigation2d/application/exploration/return_geometry.h"
namespace navigation2d::exploration {
enum class ReturnPlanStage { kInitial, kAfterEscapes, kIdleFinal };
enum class ReturnRecoveryMode { kPathFailure, kImmobility };
enum class ReturnIdleResult { kWaiting, kTracking, kEscaping, kRestart, kUnavailable };
enum class ReturnExecutionResult { kTracking, kEscaping, kUnavailable };
// Synchronous mission capabilities. The adapter executes each request before
// returning its acceptance result; the coordinator owns ordering and budgets.
class ReturnActions {
 public:
  virtual ~ReturnActions() = default;
  virtual bool PlanHome(ReturnPlanStage stage) = 0;
  virtual void HomeAccepted(ReturnPlanStage stage) = 0;
  virtual bool StartBreadcrumbTowardHome() = 0;
  virtual bool TryStartEscape() = 0;
};
class ReturnArrivalActions : public ReturnActions {
 public:
  virtual bool PlanDirectAfterHop(const ExplorationGoal& home) = 0;
  virtual void DirectHomeAfterHopAccepted() = 0;
  virtual void FinishReturn(bool partial) = 0;
};
class ReturnMissionActions;
class ReturnCoordinator {
 public:
  // Caller has checked mission home and that no return path is active.
  void BeginMission(bool force_partial, ReturnMissionActions&);
  bool FollowTrail(std::span<const std::pair<double, double>> trail, const ExplorationGoal& home,
                   double robot_x, double robot_y, const ExplorationGoal& active_goal,
                   ReturnMissionActions&);
  void RecoverMission(ReturnRecoveryMode, const char* unavailable_message, ReturnMissionActions&);
  void TickIdle(bool escape_pending, ReturnMissionActions&);
  // Return intent can precede a route. An accepted explore/retreat route
  // overrides this flag; it is not derived only from the displayed target.
  void AcceptNavigationMode(bool returning) { returning_ = returning; }
  bool returning() const { return returning_; }
  ReturnExecutionResult Execute(ReturnActions& actions);
  // Called after cancelling the reached route. Home/robot use world-frame
  // metres; direct retry precedes another breadcrumb hop and final completion.
  void CompleteArrival(const ExplorationGoal& home, double robot_x, double robot_y,
                       ReturnArrivalActions& actions);
  ReturnExecutionResult TryTrailOrEscape(ReturnActions& actions);
  ReturnExecutionResult Recover(ReturnRecoveryMode mode, ReturnActions& actions);
  ReturnIdleResult ContinueIdle(bool escape_pending, ReturnActions& actions);
  void MarkPartial() { partial_ = true; }
  bool partial() const { return partial_; }
  void UseBreadcrumbs(bool enabled) { breadcrumbs_ = enabled; }
  bool breadcrumbs() const { return breadcrumbs_; }
  void ResetEscapes() { escapes_ = 0; }
  void CountEscape() { ++escapes_; }
  int escapes() const { return escapes_; }
  ExplorationGoal Home(const ExplorationGrid* map, ExplorationGoal home) const;
  ReturnStep NextStep(std::span<const std::pair<double, double>> trail, const ExplorationGoal& home,
                      double rx, double ry, const ExplorationGoal& active_goal) const;

 private:
  bool StartBoundedEscape(ReturnActions& actions);
  bool returning_ = false;
  bool partial_ = false;
  bool breadcrumbs_ = false;
  int escapes_ = 0;
};
}  // namespace navigation2d::exploration
