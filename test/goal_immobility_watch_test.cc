#include <algorithm>
#include <stdexcept>

#include "navigation2d/test/goal_tracking_fixture.h"
using namespace navigation2d;
using namespace navigation2d::exploration;
using navigation2d::test::TrackingScene;
namespace {
bool Watch(TrackingScene& scene) {
  return scene.execution.WatchImmobility(scene.state, scene.navigation, scene.motion, scene, scene,
                                         scene);
}
void Prime(TrackingScene& scene) {
  scene.motion.Observe(1., 1., scene.ExecutionNowNs());
  scene.now_ns = 10000000000LL;
  scene.events.clear();
}
bool Has(const TrackingScene& scene, const char* event) {
  return std::find(scene.events.begin(), scene.events.end(), event) != scene.events.end();
}
}  // namespace
int main() {
  TrackingScene absent;
  absent.navigation.Cancel();
  assert(!Watch(absent));
  assert((absent.events == std::vector<std::string>{"robot", "clock"}));
  TrackingScene missing;
  missing.pose = false;
  assert(!Watch(missing));
  assert((missing.events == std::vector<std::string>{"robot"}));
  assert(missing.navigation.has_session() && missing.commands.empty());

  TrackingScene young;
  Prime(young);
  young.now_ns = 7000000000LL;
  assert(!Watch(young));
  assert(young.motion.recoveries() == 0 && young.navigation.has_session());
  assert(!Has(young, "immobility-report") && young.commands.empty());
  young.now_ns = 0;  // Backward clock cannot turn the goal age into a stall.
  assert(!Watch(young));

  TrackingScene stalled;
  Prime(stalled);
  int clocks = 0;
  stalled.on_clock = [&] {
    ++clocks;
    if (clocks == 3) {
      assert(!stalled.navigation.has_session());
      assert(stalled.commands.size() == 1 && stalled.commands[0].linear == 0. &&
             stalled.commands[0].angular == 0.);
      return std::int64_t{11000000000LL};
    }
    return stalled.now_ns;
  };
  stalled.on_immobility_report = [&](PoseImmobilityNotice notice) {
    assert(stalled.navigation.has_session() && stalled.commands.empty());
    assert(notice.recoveries == 1 && !notice.returning && !notice.entry_pending &&
           !notice.tip_wedged && notice.front_m == 1.);
  };
  stalled.on_replan = [&] {
    assert(clocks == 3 && !stalled.navigation.has_session());
    assert(!stalled.motion.Stalled(
        {11000000000LL, 1000000000LL, 1000000000LL, 0., false, false, true}));
  };
  assert(Watch(stalled));
  assert((stalled.events == std::vector<std::string>{"robot", "clock", "body", "pinch",
                                                     "clock", "immobility-report", "motion",
                                                     "clock", "robot", "cool-route", "replan",
                                                     "status:NAVIGATING"}));
  assert(stalled.motion.recoveries() == 1);

  TrackingScene tip;
  Prime(tip);
  tip.now_ns = 6000000000LL;
  tip.body = {.2, .2, .2, .2, .2};
  tip.pinch = true;
  tip.escape = true;
  tip.on_immobility_report = [](PoseImmobilityNotice notice) {
    assert(notice.tip_wedged && !notice.entry_pending);
  };
  assert(Watch(tip));
  assert(Has(tip, "peel") && !Has(tip, "replan") && tip.explorer.failed_goals() == 1);

  TrackingScene returning;
  Prime(returning);
  returning.mission.BeginReturning();
  returning.entry.SetPending(returning.goal);
  returning.now_ns = 16000000000LL;
  returning.on_immobility_report = [](PoseImmobilityNotice notice) {
    // Intent and mission phase remain independent in the original diagnostic.
    assert(!notice.returning && notice.entry_pending && !notice.tip_wedged);
  };
  assert(Watch(returning));
  assert(returning.events.back() == "return-stalled");
  assert(!Has(returning, "probe") && !Has(returning, "pinch"));

  TrackingScene advancing;
  Prime(advancing);
  advancing.entry.Lock(advancing.goal, false, {});
  advancing.entry.ClearPending();
  advancing.now_ns = 16000000000LL;
  advancing.watch.Start(1000000000LL, 15000000000LL);
  advancing.state.path_progress_m = .2;
  assert(!Watch(advancing));
  assert(!Has(advancing, "pinch") && advancing.commands.empty());
  advancing.state.path_progress_m = 0.;
  advancing.on_immobility_report = [](PoseImmobilityNotice notice) {
    assert(!notice.entry_pending && !notice.tip_wedged);
  };
  assert(Watch(advancing));
  assert(Has(advancing, "probe"));

  TrackingScene failed_report;
  Prime(failed_report);
  failed_report.on_immobility_report = [](PoseImmobilityNotice) {
    throw std::runtime_error("report");
  };
  try {
    Watch(failed_report);
    assert(false);
  } catch (const std::runtime_error&) {
    assert(failed_report.navigation.has_session() && failed_report.commands.empty());
    assert(failed_report.motion.recoveries() == 1);
  }
}
