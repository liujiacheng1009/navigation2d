#include "navigation2d/application/exploration/planning_evidence_budget.h"

#include <limits>
#include <stdexcept>
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("planning evidence budget contract failed");
  }
}
}  // namespace
int main() {
  using navigation2d::exploration::PlanningEvidenceBudget;
  PlanningEvidenceBudget budget;
  Require(budget.Consume(.05, 20) == 0.);
  budget.RequestStartReplay();
  Require(budget.Consume(.05, 0) == 0.);
  Require(budget.Consume(.05, 1) == 0.);
  Require(budget.Consume(.05, 2) == 2.);
  Require(budget.Consume(.05, 2) == 0.);
  budget.RequestTrailReplay(80.);
  Require(budget.Consume(0., 20) == 0.);
  Require(budget.Consume(-.1, 20) == 0.);
  Require(budget.Consume(std::numeric_limits<double>::quiet_NaN(), 20) == 0.);
  Require(budget.Consume(.05, 20) == 80.);
  budget.ResumeTrailReplay();
  Require(budget.Consume(.05, 20) == 2.);
  budget.RequestTrailReplay(12.);
  budget.CancelTrailReplay();
  Require(budget.Consume(.05, 20) == 0.);
  budget.ResumeTrailReplay();
  Require(budget.Consume(.05, 20) == 12.);
  budget.RequestTrailReplay(8.);
  budget.RequestStartReplay();
  Require(budget.Consume(.05, 20) == 8.);
  Require(budget.Consume(.05, 20) == 0.);
  budget.RequestTrailReplay(4.);
  Require(budget.Consume(.05, 1) == 0.);
  budget.CancelTrailReplay();
  budget.RequestStartReplay();
  Require(budget.Consume(.05, 2) == 4.);
  budget.RequestTrailReplay(-1.);
  Require(budget.Consume(.05, 2) == 2.);
}
