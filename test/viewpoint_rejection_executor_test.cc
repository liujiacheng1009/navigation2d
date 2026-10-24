#include "navigation2d/application/exploration/viewpoint_rejection_executor.h"

#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
struct Actions final : ViewpointRejectionActions {
  const FrontierExplorer& explorer;
  const ViewpointRejections& history;
  mutable std::vector<std::string> events;
  bool fail_cooling = false;
  int failures_before = 0;
  ExplorationGoal last{};
  Actions(const FrontierExplorer& e, const ViewpointRejections& h) : explorer(e), history(h) {}
  void CoolRejectedSignature(std::uint64_t signature) override {
    assert(signature == 7);
    events.emplace_back("cool");
    if (fail_cooling) {
      throw std::runtime_error("route cooling failed");
    }
  }
  std::int64_t RejectionDeadlineNs(double seconds) const override {
    assert(seconds == 10. && explorer.failed_goals() == failures_before + 1);
    events.emplace_back("deadline");
    return 15000000000LL;
  }
  void ReportViewpointRejection(const ViewpointRejectionNotice& notice) override {
    last = notice.goal;
    if (notice.kind == ViewpointRejectionKind::kSoftScrape) {
      assert(notice.scrape_count == 1 && explorer.failed_goals() == failures_before);
      events.emplace_back("soft");
    } else {
      assert(explorer.failed_goals() == failures_before + 1);
      assert(history.RecentlyDemoted(notice.goal, 5000000000LL));
      events.emplace_back("hard");
    }
  }
};
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  const std::string wall = "wall-scrape rejected", tip = "tip-scrape rejected", hard = "no path";
  FrontierExplorer explorer;
  TourExecution tour;
  ViewpointRejections history;
  Actions actions(explorer, history);
  ExplorationGoal goal{};
  goal.x = .5;
  goal.frontier_x = 1.;
  RejectOpenWallViewpoint(explorer, tour, history, {goal, wall, 7}, actions);
  assert((actions.events == std::vector<std::string>{"cool", "soft"}));
  assert(explorer.failed_goals() == 0 && tour.goals().size() == 1);
  assert(!history.HasDemotion());
  tour.Advance();  // The caller consumes the current task before rejection handling.
  actions.events.clear();
  auto nearby = goal;
  nearby.x += .5;
  RejectOpenWallViewpoint(explorer, tour, history, {nearby, tip, 7}, actions);
  assert((actions.events == std::vector<std::string>{"cool", "deadline", "hard"}));
  assert(tour.goals().empty() && explorer.failed_goals() == 1 && actions.last.x == nearby.x);
  assert(history.NoteScrape(nearby) == 1);  // Hard demotion cleared the soft budget.
  history.ResetScrapes();
  history.ClearDemotion();
  actions.events.clear();
  actions.failures_before = explorer.failed_goals();
  RejectOpenWallViewpoint(explorer, tour, history, {goal, hard, 7}, actions);
  assert((actions.events == std::vector<std::string>{"deadline", "hard"}));
  actions.events.clear();
  actions.failures_before = explorer.failed_goals();
  history.ClearDemotion();
  RejectOpenWallViewpoint(explorer, tour, history, {goal, wall, 0}, actions);
  assert(actions.events == std::vector<std::string>{"soft"});
  assert(tour.goals().size() == 1);
  // Route-cooling failure precedes scrape mutation and tour append.
  history.ResetScrapes();
  tour.Suspend();
  actions.events.clear();
  actions.fail_cooling = true;
  bool threw = false;
  try {
    RejectOpenWallViewpoint(explorer, tour, history, {goal, wall, 7}, actions);
  } catch (const std::runtime_error&) {
    threw = true;
  }
  assert(threw && tour.goals().empty() && !history.HasDemotion());
  assert(actions.events == std::vector<std::string>{"cool"});
  assert(history.NoteScrape(goal) == 1);
}
