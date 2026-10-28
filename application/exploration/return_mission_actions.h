#pragma once
#include <string>

#include "navigation2d/application/exploration/return_coordinator.h"
namespace navigation2d::exploration {
enum class ReturnStartNotice { kForcedPartial, kCoverageIncomplete };
// Mission/ROS effects are synchronous. The coordinator owns return intent,
// breadcrumb mode and retry budgets; adapters own phase/output bindings.
class ReturnMissionActions : public ReturnArrivalActions {
 public:
  virtual bool TryVisitUnenteredSpace() = 0;
  virtual void CommitReturnPhase() = 0;
  virtual void SuspendExplorationForReturn() = 0;
  virtual bool ReturnCoverageComplete() const = 0;
  virtual void ReportReturnStart(ReturnStartNotice) = 0;
  virtual void ClearReturnRouteCooling() = 0;
  virtual void PrepareReturnEvidence(bool replay_start) = 0;
  virtual bool NavigateReturnGoal(const ExplorationGoal&) = 0;
  virtual void ReportBreadcrumbHop(const ExplorationGoal&, double robot_x, double robot_y,
                                   double home_distance) = 0;
  virtual void ResumeReturnEscape(bool replay_start) = 0;
  virtual void Finish(bool success, const std::string& message) = 0;
  virtual void BeginReturn(bool partial) = 0;
};
}  // namespace navigation2d::exploration
