#include <algorithm>
#include <stdexcept>

#include "navigation2d/application/exploration/route_failure_history.h"
#include "navigation2d/test/goal_failure_fixture.h"
using namespace navigation2d;
using namespace navigation2d::exploration;
using navigation2d::test::FailureScene;
namespace {
void RejectTwice(RouteFailureHistory& routes) {
  routes.NoteMappedReject();
  routes.NoteMappedReject();
}
void Pocket(FailureScene& scene) {
  // Inject an already-opened planning pocket fact; an ordinary observation
  // map alone does not open unknown cells in the current map evidence model.
  scene.recovery_unknown = 100;
}
}  // namespace
int main() {
  RouteFailureHistory routes;
  FailureScene tip;
  routes.NoteMappedReject();
  assert(!tip.execution.HandleMappedThrash(routes, tip) && tip.events.empty());
  routes.ClearSignatures();
  routes.ObserveCell({2, 3});
  routes.NoteMappedReject();
  routes.Cool(42);
  tip.recovery.Begin(1., 1.);
  assert(tip.execution.HandleMappedThrash(routes, tip));
  assert((tip.events == std::vector<std::string>{"cancel", "unknown", "report-tip", "robot",
                                                 "status:SELECTING", "peel", "delay-mapped"}));
  assert(routes.size() == 0 && !tip.recovery.active && tip.tour.goals().empty());
  assert(!routes.TakeMappedThrash());
  RejectTwice(routes);
  routes.ResetMappedRejects();
  assert(!routes.TakeMappedThrash());

  FailureScene missing;
  missing.pose = false;
  RejectTwice(routes);
  assert(missing.execution.HandleMappedThrash(routes, missing));
  assert(missing.events[2] == "report-tip" && missing.events.back() == "delay-mapped");

  FailureScene entry;
  Pocket(entry);
  RejectTwice(routes);
  assert(entry.execution.HandleMappedThrash(routes, entry));
  assert((entry.events == std::vector<std::string>{"cancel", "unknown", "robot", "report-pocket",
                                                   "lock", "evidence", "probe"}));
  assert(entry.entry.pending() && entry.tour.goals().size() == 2 &&
         entry.explorer.failed_goals() == 0);

  FailureScene rejected;
  Pocket(rejected);
  rejected.probe = false;
  RejectTwice(routes);
  assert(rejected.execution.HandleMappedThrash(routes, rejected));
  assert((rejected.events == std::vector<std::string>{"cancel", "unknown", "robot", "report-pocket",
                                                      "lock", "evidence", "probe", "peel", "select",
                                                      "status:SELECTING"}));
  assert(!rejected.entry.pending() && rejected.tour.goals().size() == 1 &&
         rejected.explorer.failed_goals() == 1);

  FailureScene peel;
  peel.escape = true;
  RejectTwice(routes);
  assert(peel.execution.HandleMappedThrash(routes, peel));
  assert(peel.events.back() == "peel");
  FailureScene boundary;
  boundary.recovery_unknown = 80;
  RejectTwice(routes);
  assert(boundary.execution.HandleMappedThrash(routes, boundary));
  assert(std::find(boundary.events.begin(), boundary.events.end(), "lock") ==
         boundary.events.end());
  FailureScene missing_entry;
  missing_entry.pose = false;
  missing_entry.recovery_unknown = 81;
  RejectTwice(routes);
  assert(missing_entry.execution.HandleMappedThrash(routes, missing_entry));
  assert(std::find(missing_entry.events.begin(), missing_entry.events.end(), "lock") ==
         missing_entry.events.end());
  FailureScene exception;
  exception.on_peel = [] { throw std::runtime_error("peel"); };
  RejectTwice(routes);
  try {
    exception.execution.HandleMappedThrash(routes, exception);
    assert(false);
  } catch (const std::runtime_error&) {
  }
  assert(!routes.TakeMappedThrash() && exception.tour.goals().empty());
}
