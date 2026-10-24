#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/return_coordinator.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("return arrival contract");
  }
}
struct Actions final : ReturnArrivalActions {
  bool direct = false, breadcrumb = false, partial = false;
  std::vector<std::string> events;
  bool PlanHome(ReturnPlanStage) override { throw std::runtime_error("unexpected plan stage"); }
  void HomeAccepted(ReturnPlanStage) override {
    throw std::runtime_error("unexpected home notice");
  }
  bool TryStartEscape() override { throw std::runtime_error("unexpected escape"); }
  bool PlanDirectAfterHop(const ExplorationGoal&) override {
    events.push_back("direct");
    return direct;
  }
  bool StartBreadcrumbTowardHome() override {
    events.push_back("breadcrumb");
    return breadcrumb;
  }
  void DirectHomeAfterHopAccepted() override { events.push_back("accepted"); }
  void FinishReturn(bool incomplete) override {
    partial = incomplete;
    events.push_back("finish");
  }
};
}  // namespace
int main() {
  ReturnCoordinator returning;
  ExplorationGoal home;
  Actions actions;
  returning.UseBreadcrumbs(true);
  actions.direct = true;
  returning.CompleteArrival(home, 1., 0., actions);
  Require(actions.events == std::vector<std::string>{"direct", "accepted"} &&
          !returning.breadcrumbs());
  returning.UseBreadcrumbs(true);
  actions = {};
  actions.breadcrumb = true;
  returning.CompleteArrival(home, 1., 0., actions);
  Require(actions.events == std::vector<std::string>{"direct", "breadcrumb"} &&
          returning.breadcrumbs());
  actions = {};
  returning.MarkPartial();
  returning.CompleteArrival(home, 1., 0., actions);
  Require(actions.events == std::vector<std::string>{"direct", "breadcrumb", "finish"});
  Require(actions.partial && !returning.breadcrumbs());
  actions = {};
  returning.UseBreadcrumbs(true);
  returning.CompleteArrival(home, .4, 0., actions);
  Require(actions.events == std::vector<std::string>{"finish"} && actions.partial &&
          !returning.breadcrumbs());
}
