#include <algorithm>

#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/application/exploration/viewpoint_selection.h"
#include "navigation2d/test/goal_tracking_fixture.h"
using namespace navigation2d;
using namespace navigation2d::exploration;
using namespace navigation2d::test;
namespace {
struct IdleScene : TrackingScene {
  CompletionExecution completion;
  ViewpointRejections rejections;
  ViewpointSelection selection{explorer, entry, tour, completion, rejections};
  bool Idle() { return execution.TickIdle(navigation, selection, *this, *this, *this); }
};
bool Has(const IdleScene& scene, const char* event) {
  return std::find(scene.events.begin(), scene.events.end(), event) != scene.events.end();
}
}  // namespace
int main() {
  IdleScene active;
  assert(!active.Idle() && active.events.empty() && active.commands.empty());

  IdleScene waiting;
  waiting.navigation.Cancel();
  waiting.selection.ScheduleAt(3000000000LL);
  assert(waiting.Idle());
  assert((waiting.events == std::vector<std::string>{"motion", "clock"}));
  assert(waiting.commands.size() == 1 && waiting.commands.front().linear == 0.);

  IdleScene due;
  due.navigation.Cancel();
  assert(due.Idle());
  assert((due.events ==
          std::vector<std::string>{"motion", "clock", "deadline", "clock", "select-next"}));
  assert(!due.selection.Due(2999999999LL) && due.selection.Due(3000000000LL));

  IdleScene returning;
  returning.navigation.Cancel();
  returning.returning.AcceptNavigationMode(true);
  returning.entry.Lock(returning.goal, false, {});
  returning.entry.StartCrawl();
  assert(returning.Idle());
  assert(returning.events == std::vector<std::string>{"return-idle"} && returning.commands.empty());
  IdleScene phase;
  phase.navigation.Cancel();
  phase.mission.BeginReturning();
  assert(phase.Idle() && phase.events == std::vector<std::string>{"return-idle"});

  IdleScene crawl;
  crawl.navigation.Cancel();
  auto mouth = crawl.goal;
  mouth.x = 1.6;
  crawl.entry.Lock(mouth, false, {});
  crawl.entry.StartCrawl();
  assert(crawl.Idle());
  assert(!Has(crawl, "clock") && !Has(crawl, "select-next"));
  assert(Has(crawl, "crawl-publish") || Has(crawl, "ack") || Has(crawl, "probe"));

  IdleScene missing;
  missing.navigation.Cancel();
  missing.pose = false;
  missing.entry.Lock(missing.goal, false, {});
  missing.entry.StartCrawl();
  missing.selection.ScheduleAt(3000000000LL);
  assert(missing.Idle() && !Has(missing, "crawl-publish"));
  assert(missing.commands.size() == 1 && missing.commands.front().linear == 0.);
}
