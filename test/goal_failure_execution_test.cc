#include <algorithm>
#include <stdexcept>

#include "navigation2d/application/navigation_failure.h"
#include "navigation2d/test/goal_failure_fixture.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
using navigation2d::test::FailureScene;
bool Has(const FailureScene& scene, const char* event) {
  return std::find(scene.events.begin(), scene.events.end(), event) != scene.events.end();
}
}  // namespace
int main() {
  FailureScene first;
  first.recovery.Begin(1., 1.);
  first.recovery.wall_scrapes = 1;
  first.Run();
  assert((first.events ==
          std::vector<std::string>{"report", "clock", "first", "demote", "pinch", "replan"}));
  assert(first.recovery.wall_scrapes == 1 && first.tour.goals().size() == 2);
  assert(first.explorer.failed_goals() == 1);

  FailureScene corner;
  corner.corner = true;
  corner.now_ns = 3000000001LL;
  corner.state.collision_monitor_action = CollisionMonitorAction::kStop;
  corner.Run(.1, .1);
  assert((corner.events == std::vector<std::string>{"corner", "clock", "scrape"}));
  FailureScene exact;
  exact.corner = true;
  exact.now_ns = 3000000000LL;
  exact.state.collision_monitor_action = CollisionMonitorAction::kStop;
  exact.escape = true;
  exact.Run(.1, .1);
  assert(!Has(exact, "scrape") && Has(exact, "escape") && !Has(exact, "report"));

  FailureScene timeout;
  timeout.state.path_progress_m = .1;
  timeout.now_ns = 57000000000LL;
  timeout.watch.ObserveProgress(.1, timeout);
  timeout.events.clear();
  timeout.Run();
  assert((timeout.events == std::vector<std::string>{"report", "clock", "clock", "demote-active"}));
  assert(timeout.message == "goal timeout; demoting viewpoint");

  FailureScene entry;
  entry.entry.Lock(entry.goal, false, {});
  entry.entry.ClearPending();
  entry.state.execution_failure = NavigationFailure::kControllerNoMotion;
  entry.Run();
  assert((entry.events ==
          std::vector<std::string>{"report", "clock", "first", "cancel", "pose", "crawl"}));
  assert(entry.entry.pending() && entry.entry.crawl_ticks() > 0 && !Has(entry, "replan"));
  FailureScene plugin_text;
  plugin_text.entry.SetPending(plugin_text.goal);
  plugin_text.state.planning_failure_reason = "no measured motion";
  plugin_text.Run();
  assert(Has(plugin_text, "probe") && !Has(plugin_text, "crawl"));

  FailureScene returning;
  returning.mission.BeginReturning();
  returning.fallback = true;
  returning.Run();
  assert((returning.events == std::vector<std::string>{"report", "clock", "first", "fallback"}));
  FailureScene forced;
  forced.force_return = true;
  forced.Run();
  assert(Has(forced, "return") && forced.tour.goals().empty() && !Has(forced, "replan"));

  FailureScene immobile;
  immobile.replan = false;
  immobile.state.execution_failure = NavigationFailure::kControllerNoMotion;
  immobile.Run();
  assert(!Has(immobile, "replan") && immobile.tour.goals().size() == 1);
  assert(immobile.message == "no motion against the jamb; leaving this route");

  // A rejected synchronous retry may still change mission mode. Read it live,
  // instead of finishing under the exploring snapshot taken before the call.
  FailureScene changed;
  changed.on_replan = [&] { changed.returning.AcceptNavigationMode(true); };
  changed.replan = false;
  changed.Run();
  assert(changed.returning.partial() && Has(changed, "breadcrumbs") && Has(changed, "finish"));
  FailureScene completion;
  completion.mission.BeginCompleting();
  completion.replan = false;
  completion.state.path_progress_m = .1;
  completion.Run();
  assert(Has(completion, "completion") && completion.explorer.failed_goals() == 1);
  FailureScene exception;
  exception.on_replan = [] { throw std::runtime_error("retry"); };
  try {
    exception.Run();
    assert(false);
  } catch (const std::runtime_error&) {
  }
  assert(exception.events.back() == "replan" && !Has(exception, "cancel"));
}
